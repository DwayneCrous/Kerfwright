//! The CPU reference tracer: f64 throughout, its own median-split BVH, its own BRDF code. It implements the
//! light-transport model described in [`super`] and is what the GPU tracer is validated against. It shares
//! no code with the GPU path (written by the same author, so it is a second implementation, not a third-party
//! renderer).

use super::{PtCamera, PtScene};
use rayon::prelude::*;

type V = [f64; 3];

fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn mul(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn vmul(a: V, b: V) -> V {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: V) -> V {
    mul(a, 1.0 / dot(a, a).sqrt())
}
fn up(a: [f32; 3]) -> V {
    [a[0] as f64, a[1] as f64, a[2] as f64]
}

#[derive(Clone, Copy)]
pub struct Mat {
    pub albedo: V,
    pub rough: f64,
    pub metallic: f64,
    pub specular: f64,
    pub emission: V,
}

impl Mat {
    pub fn from(m: &super::Material) -> Mat {
        Mat {
            albedo: up(m.albedo),
            rough: m.roughness as f64,
            metallic: m.metallic as f64,
            specular: m.specular as f64,
            emission: up(m.emission),
        }
    }
}

fn pi() -> f64 {
    std::f64::consts::PI
}

fn fresnel(m: &Mat, vh: f64) -> V {
    let f0 = [
        0.04 + (m.albedo[0] - 0.04) * m.metallic,
        0.04 + (m.albedo[1] - 0.04) * m.metallic,
        0.04 + (m.albedo[2] - 0.04) * m.metallic,
    ];
    let k = (1.0 - vh).clamp(0.0, 1.0).powi(5);
    [
        f0[0] + (1.0 - f0[0]) * k,
        f0[1] + (1.0 - f0[1]) * k,
        f0[2] + (1.0 - f0[2]) * k,
    ]
}

fn ggx_d(a2: f64, nh: f64) -> f64 {
    let t = nh * nh * (a2 - 1.0) + 1.0;
    a2 / (pi() * t * t)
}

fn lambda(a2: f64, c: f64) -> f64 {
    let t2 = (1.0 - c * c) / (c * c);
    (-1.0 + (1.0 + a2 * t2).sqrt()) / 2.0
}

/// BRDF value; `n` faces the viewer, `wo` and `wi` point away from the surface.
pub fn brdf(m: &Mat, n: V, wo: V, wi: V) -> V {
    let (co, ci) = (dot(n, wo), dot(n, wi));
    if co <= 0.0 || ci <= 0.0 {
        return [0.0; 3];
    }
    let a = (m.rough * m.rough).max(1e-3);
    let a2 = a * a;
    let h = norm(add(wo, wi));
    let (nh, vh) = (dot(n, h).max(0.0), dot(wo, h).max(0.0));
    let f = fresnel(m, vh);
    let d = ggx_d(a2, nh);
    let g = 1.0 / (1.0 + lambda(a2, co) + lambda(a2, ci));
    let spec = m.specular * d * g / (4.0 * ci * co);
    let mut out = [0.0; 3];
    for k in 0..3 {
        out[k] = (1.0 - m.metallic) * (1.0 - m.specular * f[k]) * m.albedo[k] / pi() + spec * f[k];
    }
    out
}

fn p_spec(m: &Mat) -> f64 {
    if m.specular <= 0.0 {
        0.0
    } else if m.metallic >= 1.0 {
        1.0
    } else {
        0.5
    }
}

/// Density (solid angle) with which [`sample`] returns `wi`.
pub fn pdf(m: &Mat, n: V, wo: V, wi: V) -> f64 {
    let (co, ci) = (dot(n, wo), dot(n, wi));
    if co <= 0.0 || ci <= 0.0 {
        return 0.0;
    }
    let ps = p_spec(m);
    let a = (m.rough * m.rough).max(1e-3);
    let h = norm(add(wo, wi));
    let (nh, vh) = (dot(n, h).max(0.0), dot(wo, h).max(1e-9));
    (1.0 - ps) * ci / pi() + ps * ggx_d(a * a, nh) * nh / (4.0 * vh)
}

fn frame(n: V) -> (V, V) {
    let h = if n[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let t = norm(cross(h, n));
    (t, cross(n, t))
}

/// Draw `wi` with density [`pdf`].
pub fn sample(m: &Mat, n: V, wo: V, u: [f64; 3]) -> Option<V> {
    let (t, b) = frame(n);
    let to_world = |x: f64, y: f64, z: f64| add(add(mul(t, x), mul(b, y)), mul(n, z));
    let ps = p_spec(m);
    let wi = if u[2] >= ps {
        let r = u[0].sqrt();
        let ph = 2.0 * pi() * u[1];
        to_world(r * ph.cos(), r * ph.sin(), (1.0 - u[0]).max(0.0).sqrt())
    } else {
        let a = (m.rough * m.rough).max(1e-3);
        let a2 = a * a;
        let ct = ((1.0 - u[0]) / (1.0 + (a2 - 1.0) * u[0])).sqrt();
        let st = (1.0 - ct * ct).max(0.0).sqrt();
        let ph = 2.0 * pi() * u[1];
        let h = to_world(st * ph.cos(), st * ph.sin(), ct);
        let vh = dot(wo, h);
        if vh <= 0.0 {
            return None;
        }
        sub(mul(h, 2.0 * vh), wo)
    };
    (dot(n, wi) > 0.0).then_some(wi)
}

struct Bvh {
    // nodes: (min, max, left_or_first, count) ; count>0 leaf
    nodes: Vec<(V, V, usize, usize)>,
    order: Vec<usize>,
}

impl Bvh {
    fn build(tris: &[[V; 3]]) -> Bvh {
        let mut order: Vec<usize> = (0..tris.len()).collect();
        let cent = |i: usize| mul(add(add(tris[i][0], tris[i][1]), tris[i][2]), 1.0 / 3.0);
        let mut nodes: Vec<(V, V, usize, usize)> = Vec::new();
        let mut stack = vec![(0usize, 0usize, tris.len())];
        nodes.push(([0.0; 3], [0.0; 3], 0, 0));
        while let Some((ni, b, e)) = stack.pop() {
            let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
            for &i in &order[b..e] {
                for v in &tris[i] {
                    for k in 0..3 {
                        lo[k] = lo[k].min(v[k]);
                        hi[k] = hi[k].max(v[k]);
                    }
                }
            }
            if e - b <= 2 {
                nodes[ni] = (lo, hi, b, e - b);
                continue;
            }
            let ext = sub(hi, lo);
            let ax = if ext[0] >= ext[1] && ext[0] >= ext[2] {
                0
            } else if ext[1] >= ext[2] {
                1
            } else {
                2
            };
            let mid = (b + e) / 2;
            order[b..e]
                .select_nth_unstable_by(mid - b, |&p, &q| cent(p)[ax].total_cmp(&cent(q)[ax]));
            let l = nodes.len();
            nodes.push(([0.0; 3], [0.0; 3], 0, 0));
            nodes.push(([0.0; 3], [0.0; 3], 0, 0));
            nodes[ni] = (lo, hi, l, 0);
            stack.push((l, b, mid));
            stack.push((l + 1, mid, e));
        }
        Bvh { nodes, order }
    }

    fn hit(&self, tris: &[[V; 3]], o: V, d: V, tmin: f64, tmax: f64) -> Option<(f64, usize)> {
        let inv = [1.0 / d[0], 1.0 / d[1], 1.0 / d[2]];
        let mut best = None;
        let mut tmax = tmax;
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let (lo, hi, a, count) = self.nodes[ni];
            let (mut t0, mut t1) = (tmin, tmax);
            for k in 0..3 {
                let (p, q) = ((lo[k] - o[k]) * inv[k], (hi[k] - o[k]) * inv[k]);
                t0 = t0.max(p.min(q));
                t1 = t1.min(p.max(q));
            }
            if t0 > t1 {
                continue;
            }
            if count > 0 {
                for &i in &self.order[a..a + count] {
                    let t = &tris[i];
                    let (e1, e2) = (sub(t[1], t[0]), sub(t[2], t[0]));
                    let p = cross(d, e2);
                    let det = dot(e1, p);
                    if det.abs() < 1e-18 {
                        continue;
                    }
                    let s = sub(o, t[0]);
                    let u = dot(s, p) / det;
                    if !(0.0..=1.0).contains(&u) {
                        continue;
                    }
                    let q = cross(s, e1);
                    let v = dot(d, q) / det;
                    if v < 0.0 || u + v > 1.0 {
                        continue;
                    }
                    let tt = dot(e2, q) / det;
                    if tt > tmin && tt < tmax {
                        tmax = tt;
                        best = Some((tt, i));
                    }
                }
            } else {
                stack.push(a);
                stack.push(a + 1);
            }
        }
        best
    }
}

pub struct Reference {
    tris: Vec<[V; 3]>,
    mat: Vec<usize>,
    mats: Vec<Mat>,
    env: V,
    bvh: Bvh,
    /// Emissive triangles with the cumulative area.
    lights: Vec<(usize, f64)>,
    total_area: f64,
    pub max_depth: usize,
}

struct Rng([u64; 4]);
impl Rng {
    fn new(seed: u64, stream: u64) -> Rng {
        let mut z = seed ^ stream.wrapping_mul(0x9E37_79B9_7F4A_7C15);
        let mut s = [0u64; 4];
        for v in s.iter_mut() {
            z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            *v = x ^ (x >> 31);
        }
        Rng(s)
    }
    fn next(&mut self) -> f64 {
        let s = &mut self.0;
        let r = s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        (r >> 11) as f64 / (1u64 << 53) as f64
    }
}

impl Reference {
    pub fn new(scene: &PtScene) -> Reference {
        let tris: Vec<[V; 3]> = scene
            .tris
            .iter()
            .map(|t| [up(t[0]), up(t[1]), up(t[2])])
            .collect();
        let mats: Vec<Mat> = scene.materials.iter().map(Mat::from).collect();
        let mut lights = Vec::new();
        let mut acc = 0.0;
        for (i, t) in tris.iter().enumerate() {
            let m = &mats[scene.mat[i] as usize];
            if m.emission.iter().any(|e| *e > 0.0) {
                acc += 0.5
                    * dot(
                        cross(sub(t[1], t[0]), sub(t[2], t[0])),
                        cross(sub(t[1], t[0]), sub(t[2], t[0])),
                    )
                    .sqrt();
                lights.push((i, acc));
            }
        }
        Reference {
            bvh: Bvh::build(&tris),
            tris,
            mat: scene.mat.iter().map(|m| *m as usize).collect(),
            mats,
            env: up(scene.env),
            lights,
            total_area: acc,
            max_depth: 8,
        }
    }

    fn normal(&self, i: usize) -> V {
        let t = &self.tris[i];
        norm(cross(sub(t[1], t[0]), sub(t[2], t[0])))
    }

    /// Radiance along a ray: one sample.
    fn radiance(&self, mut o: V, mut d: V, rng: &mut Rng) -> V {
        let mut l = [0.0; 3];
        let mut thr = [1.0; 3];
        let mut prev_pdf = 0.0f64;
        for depth in 0..=self.max_depth {
            let Some((t, ti)) = self.bvh.hit(&self.tris, o, d, 1e-9, f64::INFINITY) else {
                l = add(l, vmul(thr, self.env));
                break;
            };
            let p = add(o, mul(d, t));
            let ng = self.normal(ti);
            let facing = dot(ng, d) < 0.0;
            let n = if facing { ng } else { mul(ng, -1.0) };
            let m = &self.mats[self.mat[ti]];
            let wo = mul(d, -1.0);
            if m.emission.iter().any(|e| *e > 0.0) {
                if facing {
                    let w = if depth == 0 || self.lights.is_empty() {
                        1.0
                    } else {
                        let area = 0.5 * {
                            let c = cross(
                                sub(self.tris[ti][1], self.tris[ti][0]),
                                sub(self.tris[ti][2], self.tris[ti][0]),
                            );
                            dot(c, c).sqrt()
                        };
                        let _ = area;
                        let pdf_l = t * t / (dot(ng, wo).abs() * self.total_area);
                        prev_pdf * prev_pdf / (prev_pdf * prev_pdf + pdf_l * pdf_l)
                    };
                    l = add(l, mul(vmul(thr, m.emission), w));
                }
                break;
            }
            let po = add(p, mul(n, 1e-7));
            // next-event estimation
            if !self.lights.is_empty() {
                let u = rng.next() * self.total_area;
                let k = self
                    .lights
                    .partition_point(|x| x.1 < u)
                    .min(self.lights.len() - 1);
                let li = self.lights[k].0;
                let lt = &self.tris[li];
                let (mut r1, mut r2) = (rng.next(), rng.next());
                if r1 + r2 > 1.0 {
                    r1 = 1.0 - r1;
                    r2 = 1.0 - r2;
                }
                let q = add(
                    lt[0],
                    add(mul(sub(lt[1], lt[0]), r1), mul(sub(lt[2], lt[0]), r2)),
                );
                let to = sub(q, po);
                let dist = dot(to, to).sqrt();
                let wi = mul(to, 1.0 / dist);
                let nl = self.normal(li);
                let cl = -dot(nl, wi);
                let ci = dot(n, wi);
                if cl > 0.0
                    && ci > 0.0
                    && self
                        .bvh
                        .hit(&self.tris, po, wi, 1e-9, dist * (1.0 - 1e-7) - 1e-7)
                        .is_none()
                {
                    let pdf_l = dist * dist / (cl * self.total_area);
                    let pdf_b = pdf(m, n, wo, wi);
                    let f = brdf(m, n, wo, wi);
                    let le = self.mats[self.mat[li]].emission;
                    let w = pdf_l * pdf_l / (pdf_l * pdf_l + pdf_b * pdf_b);
                    for c in 0..3 {
                        l[c] += thr[c] * f[c] * ci * le[c] * w / pdf_l;
                    }
                }
            }
            // continue the path
            let Some(wi) = sample(m, n, wo, [rng.next(), rng.next(), rng.next()]) else {
                break;
            };
            let pb = pdf(m, n, wo, wi);
            if pb <= 0.0 {
                break;
            }
            let f = brdf(m, n, wo, wi);
            let ci = dot(n, wi);
            for c in 0..3 {
                thr[c] *= f[c] * ci / pb;
            }
            prev_pdf = pb;
            if depth >= 2 {
                let q = thr[0].max(thr[1]).max(thr[2]).clamp(0.05, 0.95);
                if rng.next() > q {
                    break;
                }
                thr = mul(thr, 1.0 / q);
            }
            o = po;
            d = wi;
        }
        l
    }

    /// Mean radiance per pixel with `spp` samples; deterministic for a given `seed`.
    pub fn render(&self, cam: &PtCamera, spp: usize, seed: u64) -> Vec<[f64; 3]> {
        let (w, h) = cam.size;
        (0..(w * h) as usize)
            .into_par_iter()
            .map(|pix| {
                let (x, y) = ((pix as u32 % w) as f64, (pix as u32 / w) as f64);
                let mut rng = Rng::new(seed, pix as u64);
                let mut acc = [0.0; 3];
                for _ in 0..spp {
                    let (o, d) = cam.ray((x + rng.next()) as f32, (y + rng.next()) as f32);
                    let (o, d) = (up(o), up(d));
                    acc = add(acc, self.radiance(o, norm(d), &mut rng));
                }
                mul(acc, 1.0 / spp as f64)
            })
            .collect()
    }

    /// First-hit guides for the denoiser: albedo (emission for lights), normal and depth.
    pub fn guides(&self, cam: &PtCamera) -> Vec<([f64; 3], [f64; 3], f64)> {
        let (w, h) = cam.size;
        (0..(w * h) as usize)
            .into_par_iter()
            .map(|pix| {
                let (x, y) = ((pix as u32 % w) as f32 + 0.5, (pix as u32 / w) as f32 + 0.5);
                let (o, d) = cam.ray(x, y);
                let (o, d) = (up(o), norm(up(d)));
                match self.bvh.hit(&self.tris, o, d, 1e-9, f64::INFINITY) {
                    None => (self.env, [0.0; 3], 1e9),
                    Some((t, ti)) => {
                        let ng = self.normal(ti);
                        let n = if dot(ng, d) < 0.0 { ng } else { mul(ng, -1.0) };
                        let m = &self.mats[self.mat[ti]];
                        let alb = if m.emission.iter().any(|e| *e > 0.0) {
                            m.emission
                        } else {
                            m.albedo
                        };
                        (alb, n, t)
                    }
                }
            })
            .collect()
    }
}
