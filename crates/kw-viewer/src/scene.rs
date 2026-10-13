use bytemuck::{Pod, Zeroable};
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_tess::{tessellate_body, TessOptions, Tessellation};
use nalgebra::Isometry3;

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    /// Pick id (index into [`Scene::pick`] + 1; 0 is "nothing").
    pub id: u32,
    pub color: [u8; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Segment {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickTarget {
    Face { object: u32, face: u32 },
    Edge { object: u32, edge: u32 },
}

pub struct SceneObject<'a> {
    pub tess: &'a Tessellation,
    pub transform: Isometry3<f64>,
    pub color: [u8; 3],
}

/// World-space, scene-centred GPU-ready geometry plus a pick table.
pub struct Scene {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub segments: Vec<Segment>,
    pub pick: Vec<PickTarget>,
    /// Absolute position of the scene centre (subtracted from all vertices).
    pub center: Vector3<f64>,
    pub radius: f64,
}

pub struct Hit {
    pub triangle: usize,
    pub t: f64,
    /// Barycentric coordinates of the hit (w.r.t. vertices 1 and 2 of the triangle).
    pub bary: (f64, f64),
    pub id: u32,
}

impl Scene {
    pub fn build(objects: &[SceneObject]) -> Scene {
        // bounds in f64 world space
        let mut lo = Vector3::repeat(f64::INFINITY);
        let mut hi = Vector3::repeat(f64::NEG_INFINITY);
        for o in objects {
            for p in &o.tess.mesh.positions {
                let q = o.transform.transform_point(p);
                lo = lo.inf(&q.coords);
                hi = hi.sup(&q.coords);
            }
        }
        let center = (lo + hi) * 0.5;
        let radius = ((hi - lo) * 0.5).norm().max(1e-9);

        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut segments = Vec::new();
        let mut pick = Vec::new();
        for (oi, o) in objects.iter().enumerate() {
            let m = &o.tess.mesh;
            let base = vertices.len() as u32;
            // per-face pick ids
            let face_id_base = pick.len() as u32;
            for f in 0..o.tess.face_keys.len() {
                pick.push(PickTarget::Face {
                    object: oi as u32,
                    face: f as u32,
                });
            }
            let mut vertex_face = vec![0u32; m.positions.len()];
            for t in 0..m.triangle_count() {
                for k in 0..3 {
                    vertex_face[m.indices[3 * t + k] as usize] = m.tri_face[t];
                }
            }
            for (i, p) in m.positions.iter().enumerate() {
                let q = o.transform.transform_point(p) - center;
                let n = o.transform.transform_vector(&m.normals[i]);
                vertices.push(Vertex {
                    pos: [q.x as f32, q.y as f32, q.z as f32],
                    normal: [n.x as f32, n.y as f32, n.z as f32],
                    id: face_id_base + vertex_face[i] + 1,
                    color: [o.color[0], o.color[1], o.color[2], 255],
                });
            }
            indices.extend(m.indices.iter().map(|i| i + base));
            // edges
            let edge_id_base = pick.len() as u32;
            for (ei, start, count) in &o.tess.edges.ranges {
                let pts = &o.tess.edges.points[*start as usize..(*start + *count) as usize];
                for w in pts.windows(2) {
                    let a = o.transform.transform_point(&w[0]) - center;
                    let b = o.transform.transform_point(&w[1]) - center;
                    segments.push(Segment {
                        p0: [a.x as f32, a.y as f32, a.z as f32],
                        p1: [b.x as f32, b.y as f32, b.z as f32],
                        id: edge_id_base + ei + 1,
                    });
                }
            }
            for e in 0..o.tess.edge_keys.len() as u32 {
                pick.push(PickTarget::Edge {
                    object: oi as u32,
                    edge: e,
                });
            }
        }
        Scene {
            vertices,
            indices,
            segments,
            pick,
            center,
            radius,
        }
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Nearest front-facing triangle hit by a ray in the scene-centred frame
    /// (brute force). Used as the CPU ground truth for the pick-buffer test.
    pub fn raycast(&self, origin: Vector3<f64>, dir: Vector3<f64>) -> Option<Hit> {
        (0..self.triangle_count())
            .filter_map(|t| {
                let v = |k: usize| {
                    let p = self.vertices[self.indices[3 * t + k] as usize].pos;
                    Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64)
                };
                let (a, b, c) = (v(0), v(1), v(2));
                let (e1, e2) = (b - a, c - a);
                let p = dir.cross(&e2);
                let det = e1.dot(&p);
                if det.abs() < 1e-18 {
                    return None;
                }
                let inv = 1.0 / det;
                let s = origin - a;
                let u = s.dot(&p) * inv;
                if !(0.0..=1.0).contains(&u) {
                    return None;
                }
                let q = s.cross(&e1);
                let w = dir.dot(&q) * inv;
                if w < 0.0 || u + w > 1.0 {
                    return None;
                }
                let tt = e2.dot(&q) * inv;
                if tt <= 0.0 {
                    return None;
                }
                // only front faces are rasterised (back-face culling): for CCW
                // winding a ray hitting the front side gives det > 0
                if det < 0.0 {
                    return None;
                }
                Some(Hit {
                    triangle: t,
                    t: tt,
                    bary: (u, w),
                    id: self.vertices[self.indices[3 * t] as usize].id,
                })
            })
            .reduce(|a, b| if a.t <= b.t { a } else { b })
    }

    pub fn world_to_scene(&self, p: &Vector3<f64>) -> Vector3<f64> {
        p - self.center
    }
}

// ---------------------------------------------------------------------------
// Named scenes
// ---------------------------------------------------------------------------

pub const PALETTE: [[u8; 3]; 6] = [
    [196, 120, 80],
    [90, 150, 200],
    [120, 180, 110],
    [200, 170, 70],
    [160, 110, 190],
    [170, 175, 185],
];

pub struct Primitives {
    pub tess: Vec<Tessellation>,
}

pub fn tessellated_primitives(tol: f64) -> Primitives {
    let opts = TessOptions {
        chord_tol: tol,
        ..TessOptions::default()
    };
    let f = Frame3::world();
    let mut out = Vec::new();
    let mut add = |build: &dyn Fn(&mut BRep) -> BodyId| {
        let mut b = BRep::new();
        let body = build(&mut b);
        out.push(tessellate_body(&b, body, &opts).expect("primitive tessellates"));
    };
    add(&|b| make_box(b, &f, 2.0, 2.0, 2.0).body);
    add(&|b| make_cylinder(b, &f, 1.0, 2.0).body);
    add(&|b| make_cone(b, &f, 1.0, 0.0, 2.0).body);
    add(&|b| make_cone(b, &f, 1.0, 0.5, 2.0).body);
    add(&|b| make_sphere(b, &f, 1.1).body);
    add(&|b| make_torus(b, &f, 1.0, 0.4).body);
    Primitives { tess: out }
}

/// Six primitives in a 3×2 grid.
pub fn primitives_scene(tol: f64) -> Scene {
    let p = tessellated_primitives(tol);
    let objs: Vec<SceneObject> = p
        .tess
        .iter()
        .enumerate()
        .map(|(i, t)| SceneObject {
            tess: t,
            transform: Isometry3::translation(
                (i % 3) as f64 * 3.2 - 3.2,
                (i / 3) as f64 * 3.2 - 1.6,
                0.0,
            ),
            color: PALETTE[i % PALETTE.len()],
        })
        .collect();
    Scene::build(&objs)
}

/// A grid of copies of a fine torus/sphere totalling at least `target_tris`.
pub fn million_scene(target_tris: usize) -> Scene {
    let f = Frame3::world();
    let opts = TessOptions {
        chord_tol: 0.002,
        ..TessOptions::default()
    };
    let mut b = BRep::new();
    let sph = make_sphere(&mut b, &f, 1.0).body;
    let tor = make_torus(&mut b, &f, 0.8, 0.3).body;
    let ts = tessellate_body(&b, sph, &opts).unwrap();
    let tt = tessellate_body(&b, tor, &opts).unwrap();
    let per = (ts.mesh.triangle_count() + tt.mesh.triangle_count()) / 2;
    let n = (target_tris + per - 1) / per.max(1);
    let side = (n as f64).cbrt().ceil() as usize;
    let mut objs = Vec::new();
    for i in 0..n {
        let (x, y, z) = (i % side, (i / side) % side, i / (side * side));
        objs.push(SceneObject {
            tess: if i % 2 == 0 { &ts } else { &tt },
            transform: Isometry3::translation(x as f64 * 2.6, y as f64 * 2.6, z as f64 * 2.6),
            color: PALETTE[i % PALETTE.len()],
        });
    }
    Scene::build(&objs)
}

#[allow(dead_code)]
fn _unused(_: Point3<f64>, _: Vector3<f64>) {}
