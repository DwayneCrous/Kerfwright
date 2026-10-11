//! The progressive path tracer: scene description shared by the GPU tracer ([`gpu`]), the CPU reference
//! ([`cpu_ref`], f64, its own BVH) and the denoiser.
//!
//! Light transport model (identical in all implementations):
//! * flat-shaded triangles, double sided; emission leaves a triangle on the side its winding faces;
//! * material: `f = (1 - metallic)(1 - specular F) albedo / pi + specular D G F / (4 cos_i cos_o)` with GGX `D`,
//!   Smith height-correlated `G`, Schlick `F` (`F0 = mix(0.04, albedo, metallic)`);
//! * constant environment radiance on a miss;
//! * unidirectional path tracing with next-event estimation on the emissive triangles (area-proportional
//!   choice, uniform on the triangle) and the power heuristic against BSDF sampling, Russian roulette from the
//!   third bounce, at most `max_depth` bounces.

pub mod cpu_ref;
pub mod denoise;
pub mod gpu;
pub mod lbvh;
pub mod metrics;

pub type V3 = [f32; 3];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    pub albedo: V3,
    pub roughness: f32,
    pub metallic: f32,
    /// Scale of the specular lobe (0 = pure Lambert).
    pub specular: f32,
    pub emission: V3,
}

impl Material {
    pub fn lambert(albedo: V3) -> Material {
        Material {
            albedo,
            roughness: 1.0,
            metallic: 0.0,
            specular: 0.0,
            emission: [0.0; 3],
        }
    }
    pub fn glossy(albedo: V3, roughness: f32, metallic: f32) -> Material {
        Material {
            albedo,
            roughness,
            metallic,
            specular: 1.0,
            emission: [0.0; 3],
        }
    }
    pub fn light(emission: V3) -> Material {
        Material {
            albedo: [0.0; 3],
            roughness: 1.0,
            metallic: 0.0,
            specular: 0.0,
            emission,
        }
    }
    pub fn is_emissive(&self) -> bool {
        self.emission.iter().any(|e| *e > 0.0)
    }
}

#[derive(Debug, Clone)]
pub struct PtScene {
    pub tris: Vec<[V3; 3]>,
    /// Material index per triangle.
    pub mat: Vec<u32>,
    pub materials: Vec<Material>,
    /// Radiance seen by rays that leave the scene.
    pub env: V3,
}

impl PtScene {
    pub fn new(env: V3) -> PtScene {
        PtScene {
            tris: Vec::new(),
            mat: Vec::new(),
            materials: Vec::new(),
            env,
        }
    }

    pub fn add_material(&mut self, m: Material) -> u32 {
        self.materials.push(m);
        self.materials.len() as u32 - 1
    }

    pub fn add_tri(&mut self, t: [V3; 3], mat: u32) {
        self.tris.push(t);
        self.mat.push(mat);
    }

    /// Quad `a b c d` (counter-clockwise seen from the side the normal points to).
    pub fn add_quad(&mut self, a: V3, b: V3, c: V3, d: V3, mat: u32) {
        self.add_tri([a, b, c], mat);
        self.add_tri([a, c, d], mat);
    }

    /// Disc of radius `r` about `c`, facing `normal`, as a fan of `n` triangles.
    pub fn add_disc(&mut self, c: V3, normal: V3, r: f32, n: usize, mat: u32) {
        let nrm = normalize(normal);
        let h = if nrm[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let u = normalize(cross(h, nrm));
        let v = cross(nrm, u);
        let p = |k: usize| {
            let a = std::f32::consts::TAU * k as f32 / n as f32;
            [
                c[0] + r * (a.cos() * u[0] + a.sin() * v[0]),
                c[1] + r * (a.cos() * u[1] + a.sin() * v[1]),
                c[2] + r * (a.cos() * u[2] + a.sin() * v[2]),
            ]
        };
        for k in 0..n {
            self.add_tri([c, p(k), p(k + 1)], mat);
        }
    }

    /// Geodesic sphere with `subdiv` rounds of subdivision (outward normals).
    pub fn add_sphere(&mut self, c: V3, r: f32, subdiv: usize, mat: u32) {
        let t = (1.0 + 5f32.sqrt()) / 2.0;
        let mut v: Vec<V3> = vec![
            [-1.0, t, 0.0],
            [1.0, t, 0.0],
            [-1.0, -t, 0.0],
            [1.0, -t, 0.0],
            [0.0, -1.0, t],
            [0.0, 1.0, t],
            [0.0, -1.0, -t],
            [0.0, 1.0, -t],
            [t, 0.0, -1.0],
            [t, 0.0, 1.0],
            [-t, 0.0, -1.0],
            [-t, 0.0, 1.0],
        ]
        .into_iter()
        .map(normalize)
        .collect();
        let mut f: Vec<[usize; 3]> = vec![
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        for _ in 0..subdiv {
            let mut cache = std::collections::HashMap::new();
            let mut mid = |a: usize, b: usize, v: &mut Vec<V3>| -> usize {
                let key = (a.min(b), a.max(b));
                *cache.entry(key).or_insert_with(|| {
                    let m = normalize([
                        (v[a][0] + v[b][0]) / 2.0,
                        (v[a][1] + v[b][1]) / 2.0,
                        (v[a][2] + v[b][2]) / 2.0,
                    ]);
                    v.push(m);
                    v.len() - 1
                })
            };
            let mut nf = Vec::new();
            for [a, b, c] in f {
                let (ab, bc, ca) = (mid(a, b, &mut v), mid(b, c, &mut v), mid(c, a, &mut v));
                nf.extend_from_slice(&[[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
            }
            f = nf;
        }
        for [a, b, cc] in f {
            let p = |i: usize| [c[0] + r * v[i][0], c[1] + r * v[i][1], c[2] + r * v[i][2]];
            self.add_tri([p(a), p(b), p(cc)], mat);
        }
    }
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn normalize(a: V3) -> V3 {
    let n = dot(a, a).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

/// A pinhole camera.
#[derive(Debug, Clone, Copy)]
pub struct PtCamera {
    pub origin: V3,
    pub forward: V3,
    pub up: V3,
    /// Vertical field of view, radians.
    pub fov: f32,
    pub size: (u32, u32),
}

impl PtCamera {
    /// Direction through the continuous pixel position `(px, py)` (pixel centres at +0.5).
    pub fn ray(&self, px: f32, py: f32) -> (V3, V3) {
        let f = normalize(self.forward);
        let r = normalize(cross(f, self.up));
        let u = cross(r, f);
        let aspect = self.size.0 as f32 / self.size.1 as f32;
        let t = (self.fov / 2.0).tan();
        let x = (2.0 * px / self.size.0 as f32 - 1.0) * t * aspect;
        let y = (1.0 - 2.0 * py / self.size.1 as f32) * t;
        let d = normalize([
            f[0] + r[0] * x + u[0] * y,
            f[1] + r[1] * x + u[1] * y,
            f[2] + r[2] * x + u[2] * y,
        ]);
        (self.origin, d)
    }
}

/// The scenes of the validation suite.
pub mod scenes {
    use super::*;

    /// A closed room with a ceiling light, a diffuse red and green wall, a glossy ball and a diffuse box.
    pub fn cornell() -> (PtScene, PtCamera) {
        let mut s = PtScene::new([0.0; 3]);
        let white = s.add_material(Material::lambert([0.73, 0.73, 0.73]));
        let red = s.add_material(Material::lambert([0.65, 0.05, 0.05]));
        let green = s.add_material(Material::lambert([0.12, 0.45, 0.15]));
        let lamp = s.add_material(Material::light([14.0, 12.0, 10.0]));
        let gloss = s.add_material(Material::glossy([0.9, 0.8, 0.5], 0.3, 1.0));
        let plastic = s.add_material(Material::glossy([0.2, 0.3, 0.8], 0.5, 0.0));
        let (x0, x1, y0, y1, z0, z1) = (-1.0, 1.0, -1.0, 1.0, 0.0, 2.0);
        // floor, ceiling, back, left, right (normals into the room)
        s.add_quad(
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            white,
        );
        s.add_quad(
            [x0, y1, z1],
            [x1, y1, z1],
            [x1, y0, z1],
            [x0, y0, z1],
            white,
        );
        s.add_quad(
            [x0, y1, z0],
            [x1, y1, z0],
            [x1, y1, z1],
            [x0, y1, z1],
            white,
        );
        s.add_quad([x0, y0, z0], [x0, y1, z0], [x0, y1, z1], [x0, y0, z1], red);
        s.add_quad(
            [x1, y1, z0],
            [x1, y0, z0],
            [x1, y0, z1],
            [x1, y1, z1],
            green,
        );
        // the lamp just under the ceiling, facing down
        s.add_quad(
            [-0.3, 0.3, z1 - 0.01],
            [0.3, 0.3, z1 - 0.01],
            [0.3, -0.3, z1 - 0.01],
            [-0.3, -0.3, z1 - 0.01],
            lamp,
        );
        s.add_sphere([-0.4, 0.2, 0.45], 0.45, 3, gloss);
        // a box
        let (bx0, bx1, by0, by1, bz1) = (0.1, 0.7, -0.6, 0.0, 0.7);
        s.add_quad(
            [bx0, by0, bz1],
            [bx1, by0, bz1],
            [bx1, by1, bz1],
            [bx0, by1, bz1],
            plastic,
        );
        s.add_quad(
            [bx0, by0, 0.0],
            [bx1, by0, 0.0],
            [bx1, by0, bz1],
            [bx0, by0, bz1],
            plastic,
        );
        s.add_quad(
            [bx1, by1, 0.0],
            [bx0, by1, 0.0],
            [bx0, by1, bz1],
            [bx1, by1, bz1],
            plastic,
        );
        s.add_quad(
            [bx0, by1, 0.0],
            [bx0, by0, 0.0],
            [bx0, by0, bz1],
            [bx0, by1, bz1],
            plastic,
        );
        s.add_quad(
            [bx1, by0, 0.0],
            [bx1, by1, 0.0],
            [bx1, by1, bz1],
            [bx1, by0, bz1],
            plastic,
        );
        let cam = PtCamera {
            origin: [0.0, -3.4, 1.0],
            forward: [0.0, 1.0, 0.0],
            up: [0.0, 0.0, 1.0],
            fov: 0.78,
            size: (96, 96),
        };
        (s, cam)
    }
}
