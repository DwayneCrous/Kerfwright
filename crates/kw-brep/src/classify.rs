//! Point-in-solid classification by robust multi-direction ray casting.

use crate::ids::*;
use crate::store::BRep;
use crate::topology::*;
use crate::uvgeom::{unwrap_uv, LoopIndex, UV};
use kw_math::{Aabb, Point3, Vector3};
use kw_nurbs::proj::surface_param_of;
use kw_nurbs::{Surface, SurfaceGeom};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointClass {
    In,
    Out,
    On,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClassifyError {
    /// Rays disagreed or kept grazing edges: the caller should treat the answer as unknown.
    Ambiguous,
    /// A face lacks pcurves or geometry.
    BadModel,
}

struct FaceData {
    surface: SurfaceGeom,
    sigma: f64,
    index: std::sync::Arc<LoopIndex>,
    uv_lo: UV,
    uv_hi: UV,
    bbox: Aabb,
}

/// Bounding-volume hierarchy over the face boxes of a body: point and ray queries visit only the faces
/// whose boxes are hit, instead of every face.
struct Bvh {
    nodes: Vec<BvhNode>,
    order: Vec<u32>,
}

struct BvhNode {
    bb: Aabb,
    /// Inner node: child node indices. Leaf (`count > 0`): `order[first..first + count]`.
    left: u32,
    right: u32,
    first: u32,
    count: u32,
}

impl Bvh {
    fn build(boxes: &[Aabb]) -> Bvh {
        let mut order: Vec<u32> = (0..boxes.len() as u32).collect();
        let mut nodes = Vec::new();
        if !boxes.is_empty() {
            Self::build_rec(boxes, &mut order, 0, boxes.len(), &mut nodes);
        }
        Bvh { nodes, order }
    }

    fn build_rec(
        boxes: &[Aabb],
        order: &mut [u32],
        lo: usize,
        hi: usize,
        nodes: &mut Vec<BvhNode>,
    ) -> u32 {
        let mut bb = Aabb::EMPTY;
        for &i in &order[lo..hi] {
            bb = bb.union(&boxes[i as usize]);
        }
        let me = nodes.len() as u32;
        if hi - lo <= 4 {
            nodes.push(BvhNode {
                bb,
                left: 0,
                right: 0,
                first: lo as u32,
                count: (hi - lo) as u32,
            });
            return me;
        }
        nodes.push(BvhNode {
            bb,
            left: 0,
            right: 0,
            first: 0,
            count: 0,
        });
        let e = bb.extent();
        let axis = if e.x >= e.y && e.x >= e.z {
            0
        } else if e.y >= e.z {
            1
        } else {
            2
        };
        order[lo..hi].sort_by(|&a, &b| {
            boxes[a as usize].center()[axis]
                .partial_cmp(&boxes[b as usize].center()[axis])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mid = (lo + hi) / 2;
        let l = Self::build_rec(boxes, order, lo, mid, nodes);
        let r = Self::build_rec(boxes, order, mid, hi, nodes);
        nodes[me as usize].left = l;
        nodes[me as usize].right = r;
        me
    }

    /// Faces whose box satisfies `hit` (called on node boxes too, so it must be monotone: a box that
    /// contains a hit box is a hit).
    fn query(&self, hit: &dyn Fn(&Aabb) -> bool, out: &mut Vec<u32>) {
        out.clear();
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![0u32];
        while let Some(n) = stack.pop() {
            let node = &self.nodes[n as usize];
            if !hit(&node.bb) {
                continue;
            }
            if node.count > 0 {
                out.extend_from_slice(
                    &self.order[node.first as usize..(node.first + node.count) as usize],
                );
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
    }
}

/// Per-face data a caller may already hold (so it need not be recomputed for classification).
pub struct PreparedFace {
    pub surface: SurfaceGeom,
    pub sense: Sense,
    pub index: std::sync::Arc<LoopIndex>,
    pub uv_lo: UV,
    pub uv_hi: UV,
    pub bbox: Aabb,
}

/// Precomputed per-face data of one body for repeated classification queries.
pub struct SolidIndex {
    faces: Vec<FaceData>,
    bvh: Bvh,
    pub bbox: Aabb,
}

#[derive(Debug, Clone, Copy)]
struct Hit {
    t: f64,
    /// +1 leaving the solid, −1 entering.
    sign: f64,
    invalid: bool,
}

/// Fixed set of ray directions (irrational-ish, no axis alignment).
#[allow(clippy::approx_constant)] // arbitrary, deliberately irrational-looking ray directions
const DIRS: [[f64; 3]; 11] = [
    [0.577_350_269, 0.577_350_269, 0.577_350_269],
    [-0.267_261_242, 0.534_522_484, 0.801_783_726],
    [0.912_870_929, -0.182_574_186, 0.365_148_372],
    [-0.331_722_211, -0.883_242_107, 0.332_340_141],
    [0.123_456_789, 0.234_567_891, -0.962_911_201],
    [0.707_106_781, -0.707_106_781, 0.011_111_111],
    [-0.640_184_399, 0.426_789_599, -0.640_184_399],
    [0.301_511_345, 0.904_534_034, -0.301_511_345],
    [-0.801_783_726, -0.267_261_242, -0.534_522_484],
    [0.420_084_025, -0.560_112_034, -0.714_142_843],
    [-0.150_755_672, 0.452_267_017, 0.878_731_383],
];

impl SolidIndex {
    pub fn new(rep: &BRep, body: BodyId) -> Option<SolidIndex> {
        Self::from_faces(rep, &rep.body_faces(body))
    }

    /// Index over an explicit set of faces (e.g. one shell).
    pub fn from_faces(rep: &BRep, face_ids: &[FaceId]) -> Option<SolidIndex> {
        let mut prepared = Vec::new();
        for &f in face_ids {
            let face = rep.faces.get(f)?;
            let surface = rep.surfaces.get(face.surface)?.clone();
            let loops = rep.face_uv_loops(f, 2)?;
            let (uv_lo, uv_hi) = crate::uvgeom::loops_bbox(&loops)?;
            let index = LoopIndex::new(&loops);
            let b = rep.face_bbox_with(f, &loops, &index, uv_lo, uv_hi)?;
            prepared.push(PreparedFace {
                surface,
                sense: face.sense,
                index: std::sync::Arc::new(index),
                uv_lo,
                uv_hi,
                bbox: b,
            });
        }
        Some(Self::from_prepared(prepared))
    }

    /// Index over faces whose parameter-space data the caller already holds.
    pub fn from_prepared(prepared: Vec<PreparedFace>) -> SolidIndex {
        let mut bbox = Aabb::EMPTY;
        let mut faces = Vec::with_capacity(prepared.len());
        for p in prepared {
            bbox = bbox.union(&p.bbox);
            faces.push(FaceData {
                surface: p.surface,
                sigma: if p.sense == Sense::Forward { 1.0 } else { -1.0 },
                index: p.index,
                uv_lo: p.uv_lo,
                uv_hi: p.uv_hi,
                bbox: p.bbox,
            });
        }
        let bvh = Bvh::build(&faces.iter().map(|f| f.bbox).collect::<Vec<_>>());
        SolidIndex { faces, bvh, bbox }
    }

    /// Outward normals of every face lying within `tol` of `p` (empty if `p` is not on the boundary).
    pub fn on_normals(&self, p: &Point3<f64>, tol: f64) -> Vec<Vector3<f64>> {
        let mut cand = Vec::new();
        self.bvh.query(&|b| b.contains(p, tol), &mut cand);
        cand.sort_unstable();
        cand.iter()
            .map(|&i| &self.faces[i as usize])
            .filter(|f| f.contains_point(p, tol))
            .map(|f| {
                let uv = f.uv_of(p);
                f.surface.normal_at(uv.0, uv.1) * f.sigma
            })
            .collect()
    }

    /// Classify `p` against the solid. `tol` is the distance within which a point counts as on the boundary.
    pub fn classify(&self, p: &Point3<f64>, tol: f64) -> Result<PointClass, ClassifyError> {
        if !self.bbox.contains(p, tol) {
            return Ok(PointClass::Out);
        }
        let mut near = Vec::new();
        self.bvh.query(&|b| b.contains(p, tol), &mut near);
        if near
            .iter()
            .any(|&i| self.faces[i as usize].contains_point(p, tol))
        {
            return Ok(PointClass::On);
        }
        let mut cand: Vec<u32> = Vec::new();
        let diag = self.bbox.diagonal().max(1e-9);
        let mut votes_in = 0;
        let mut votes_out = 0;
        for dir in DIRS.iter() {
            let d = Vector3::new(dir[0], dir[1], dir[2]).normalize();
            let mut net = 0.0;
            let mut invalid = false;
            let t_max = 2.0 * diag + 1.0;
            self.bvh
                .query(&|b| ray_hits_box(b, p, &d, t_max), &mut cand);
            cand.sort_unstable();
            for &fi in &cand {
                let f = &self.faces[fi as usize];
                for h in f.ray_hits(p, &d, t_max, tol) {
                    if h.t <= 0.0 {
                        continue;
                    }
                    if h.invalid {
                        invalid = true;
                    }
                    net += h.sign;
                }
            }
            if invalid {
                continue;
            }
            if net > 0.5 {
                votes_in += 1;
            } else {
                votes_out += 1;
            }
            if votes_in >= 3 && votes_out == 0 {
                return Ok(PointClass::In);
            }
            if votes_out >= 3 && votes_in == 0 {
                return Ok(PointClass::Out);
            }
            if votes_in >= 1 && votes_out >= 1 && votes_in + votes_out >= 5 {
                break;
            }
        }
        match (votes_in, votes_out) {
            (a, 0) if a >= 2 => Ok(PointClass::In),
            (0, b) if b >= 2 => Ok(PointClass::Out),
            _ => Err(ClassifyError::Ambiguous),
        }
    }
}

/// Convenience wrapper building a [`SolidIndex`] for a single query.
pub fn classify_point(
    rep: &BRep,
    body: BodyId,
    p: &Point3<f64>,
    tol: f64,
) -> Result<PointClass, ClassifyError> {
    SolidIndex::new(rep, body)
        .ok_or(ClassifyError::BadModel)?
        .classify(p, tol)
}

impl FaceData {
    fn centre(&self) -> UV {
        (
            0.5 * (self.uv_lo.0 + self.uv_hi.0),
            0.5 * (self.uv_lo.1 + self.uv_hi.1),
        )
    }

    fn uv_of(&self, p: &Point3<f64>) -> UV {
        let r = surface_param_of(&self.surface, p, Some(self.centre()));
        unwrap_uv(&self.surface, (r.u, r.v), self.centre())
    }

    /// Is the point within `tol` of this face (surface distance and inside the trim)?
    fn contains_point(&self, p: &Point3<f64>, tol: f64) -> bool {
        if !self.bbox.contains(p, tol) {
            return false;
        }
        let r = surface_param_of(&self.surface, p, Some(self.centre()));
        if r.distance > tol {
            return false;
        }
        let uv = unwrap_uv(&self.surface, (r.u, r.v), self.centre());
        if self.index.contains(uv) {
            return true;
        }
        // just outside the trim: within tolerance of the boundary
        let (su, sv) = self.surface.derivatives(uv.0, uv.1);
        let scale = su.norm().max(sv.norm()).max(1e-12);
        self.index.near(uv, tol / scale)
    }

    fn make_hit(&self, o: &Point3<f64>, d: &Vector3<f64>, t: f64, tol: f64) -> Option<Hit> {
        let p = o + d * t;
        let uv = self.uv_of(&p);
        let (su, sv) = self.surface.derivatives(uv.0, uv.1);
        let scale = su.norm().max(sv.norm()).max(1e-12);
        let inside = self.index.contains(uv);
        let near = self.index.near(uv, (1e-6f64).max(10.0 * tol) / scale);
        if !inside && !near {
            return None;
        }
        let n = self.surface.normal_at(uv.0, uv.1) * self.sigma;
        let dn = d.dot(&n);
        Some(Hit {
            t,
            sign: dn.signum(),
            invalid: near || dn.abs() < 1e-5,
        })
    }

    fn ray_hits(&self, o: &Point3<f64>, d: &Vector3<f64>, t_max: f64, tol: f64) -> Vec<Hit> {
        // slab test against the (inflated) face box
        if !ray_hits_box(&self.bbox, o, d, t_max) {
            return Vec::new();
        }
        let roots = ray_surface(
            &self.surface,
            o,
            d,
            t_max,
            &self.bbox,
            (self.uv_lo, self.uv_hi),
        );
        roots
            .into_iter()
            .filter_map(|t| self.make_hit(o, d, t, tol))
            .collect()
    }
}

fn ray_hits_box(b: &Aabb, o: &Point3<f64>, d: &Vector3<f64>, t_max: f64) -> bool {
    let (mut t0, mut t1) = (0.0f64, t_max);
    for k in 0..3 {
        if d[k].abs() < 1e-15 {
            if o[k] < b.min[k] || o[k] > b.max[k] {
                return false;
            }
        } else {
            let (a, c) = ((b.min[k] - o[k]) / d[k], (b.max[k] - o[k]) / d[k]);
            t0 = t0.max(a.min(c));
            t1 = t1.min(a.max(c));
            if t0 > t1 {
                return false;
            }
        }
    }
    true
}

fn quad(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() < 1e-14 {
        return if b.abs() > 1e-14 {
            vec![-c / b]
        } else {
            vec![]
        };
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return vec![];
    }
    let s = disc.sqrt();
    // numerically stable
    let sign = if b >= 0.0 { 1.0 } else { -1.0 };
    let q = -0.5 * (b + sign * s);
    let mut r = vec![q / a];
    if q != 0.0 {
        r.push(c / q);
    }
    r
}

/// Parameters `t ∈ (0, t_max]` where the ray `o + t·d` meets the (untrimmed) surface.
pub fn ray_surface(
    s: &SurfaceGeom,
    o: &Point3<f64>,
    d: &Vector3<f64>,
    t_max: f64,
    bbox: &Aabb,
    uvbox: (UV, UV),
) -> Vec<f64> {
    let mut ts: Vec<f64> = match s {
        SurfaceGeom::Plane(p) => {
            let denom = d.dot(&p.frame.z);
            if denom.abs() < 1e-14 {
                vec![]
            } else {
                vec![(p.frame.origin - o).dot(&p.frame.z) / denom]
            }
        }
        SurfaceGeom::Sphere(sp) => {
            let q = o - sp.frame.origin;
            quad(1.0, 2.0 * q.dot(d), q.dot(&q) - sp.radius * sp.radius)
        }
        SurfaceGeom::Cylinder(c) => {
            let q = o - c.frame.origin;
            let (qp, dp) = (
                q - c.frame.z * q.dot(&c.frame.z),
                d - c.frame.z * d.dot(&c.frame.z),
            );
            quad(
                dp.dot(&dp),
                2.0 * qp.dot(&dp),
                qp.dot(&qp) - c.radius * c.radius,
            )
        }
        SurfaceGeom::Cone(c) => {
            let q = o - c.frame.origin;
            let tn = c.half_angle.tan();
            let (qh, dh) = (q.dot(&c.frame.z), d.dot(&c.frame.z));
            let (qp, dp) = (q - c.frame.z * qh, d - c.frame.z * dh);
            // |qp + t dp|² = (r0 + tn (qh + t dh))²
            let r = c.radius_at_origin + tn * qh;
            let a = dp.dot(&dp) - tn * tn * dh * dh;
            let b = 2.0 * (qp.dot(&dp) - r * tn * dh);
            let cc = qp.dot(&qp) - r * r;
            quad(a, b, cc)
                .into_iter()
                .filter(|t| c.radius_at_origin + tn * (qh + t * dh) >= -1e-12)
                .collect()
        }
        _ => implicit_or_generic(s, o, d, t_max, bbox, uvbox),
    };
    ts.retain(|t| t.is_finite() && *t > 0.0 && *t <= t_max);
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    ts
}

fn torus_f(t: &kw_nurbs::Torus, x: &Point3<f64>) -> f64 {
    let q = x - t.frame.origin;
    let h = q.dot(&t.frame.z);
    let rho = (q - t.frame.z * h).norm();
    (rho - t.major_radius).hypot(h) - t.minor_radius
}

fn implicit_or_generic(
    s: &SurfaceGeom,
    o: &Point3<f64>,
    d: &Vector3<f64>,
    t_max: f64,
    bbox: &Aabb,
    uvbox: (UV, UV),
) -> Vec<f64> {
    match s {
        SurfaceGeom::Torus(t) => {
            // sign changes of the signed distance along the ray inside the box
            let (t0, t1) = box_interval(bbox, o, d, t_max);
            let n = 400;
            let mut out = Vec::new();
            let mut prev = (t0, torus_f(t, &(o + d * t0)));
            for i in 1..=n {
                let ti = t0 + (t1 - t0) * i as f64 / n as f64;
                let fi = torus_f(t, &(o + d * ti));
                if (prev.1 > 0.0) != (fi > 0.0) {
                    let (mut a, mut b) = (prev.0, ti);
                    for _ in 0..80 {
                        let m = 0.5 * (a + b);
                        if (torus_f(t, &(o + d * m)) > 0.0) == (prev.1 > 0.0) {
                            a = m;
                        } else {
                            b = m;
                        }
                    }
                    out.push(0.5 * (a + b));
                }
                prev = (ti, fi);
            }
            out
        }
        _ => ray_parametric(s, o, d, t_max, uvbox),
    }
}

fn box_interval(b: &Aabb, o: &Point3<f64>, d: &Vector3<f64>, t_max: f64) -> (f64, f64) {
    let (mut t0, mut t1) = (0.0f64, t_max);
    for k in 0..3 {
        if d[k].abs() > 1e-15 {
            let (a, c) = ((b.min[k] - o[k]) / d[k], (b.max[k] - o[k]) / d[k]);
            t0 = t0.max(a.min(c));
            t1 = t1.min(a.max(c));
        }
    }
    (t0, t1.max(t0))
}

/// Generic parametric surfaces: the ray is the intersection of two planes through it; find the
/// zeros of the 2-vector `((S − o)·n₁, (S − o)·n₂)` by cell seeding + Newton.
fn ray_parametric(
    s: &SurfaceGeom,
    o: &Point3<f64>,
    d: &Vector3<f64>,
    t_max: f64,
    uvbox: (UV, UV),
) -> Vec<f64> {
    let (lo, hi) = uvbox;
    let pu = s.period_u();
    let pv = s.period_v();
    // periodic directions: search a full period if the trim spans most of it
    let (u0, u1) = match pu {
        Some(p) if hi.0 - lo.0 > 0.5 * p => (lo.0, lo.0 + p.max(hi.0 - lo.0)),
        _ => (lo.0, hi.0),
    };
    let (v0, v1) = match pv {
        Some(p) if hi.1 - lo.1 > 0.5 * p => (lo.1, lo.1 + p.max(hi.1 - lo.1)),
        _ => (lo.1, hi.1),
    };
    let helper = if d.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let n1 = d.cross(&helper).normalize();
    let n2 = d.cross(&n1);
    let f = |u: f64, v: f64| {
        let r = s.point_at(u, v) - o;
        (r.dot(&n1), r.dot(&n2))
    };
    let n = 40usize;
    let mut found: Vec<f64> = Vec::new();
    let sgn = |x: f64| x > 0.0;
    for i in 0..n {
        for j in 0..n {
            let (ua, ub) = (
                u0 + (u1 - u0) * i as f64 / n as f64,
                u0 + (u1 - u0) * (i + 1) as f64 / n as f64,
            );
            let (va, vb) = (
                v0 + (v1 - v0) * j as f64 / n as f64,
                v0 + (v1 - v0) * (j + 1) as f64 / n as f64,
            );
            let c = [f(ua, va), f(ub, va), f(ub, vb), f(ua, vb)];
            let ch1 = c.iter().any(|x| sgn(x.0)) && c.iter().any(|x| !sgn(x.0));
            let ch2 = c.iter().any(|x| sgn(x.1)) && c.iter().any(|x| !sgn(x.1));
            if !(ch1 && ch2) {
                continue;
            }
            let (mut u, mut v) = (0.5 * (ua + ub), 0.5 * (va + vb));
            let mut ok = false;
            for _ in 0..40 {
                let (su, sv) = s.derivatives(u, v);
                let (a, b, c2, dd) = (su.dot(&n1), sv.dot(&n1), su.dot(&n2), sv.dot(&n2));
                let det = a * dd - b * c2;
                let r = f(u, v);
                if det.abs() < 1e-300 {
                    break;
                }
                let du = -(dd * r.0 - b * r.1) / det;
                let dv = -(-c2 * r.0 + a * r.1) / det;
                u += du.clamp(-(ub - ua), ub - ua);
                v += dv.clamp(-(vb - va), vb - va);
                if du.abs() + dv.abs() < 1e-15 {
                    ok = true;
                    break;
                }
            }
            let r = f(u, v);
            if (ok || r.0.hypot(r.1) < 1e-10)
                && u >= ua - (ub - ua)
                && u <= ub + (ub - ua)
                && v >= va - (vb - va)
                && v <= vb + (vb - va)
            {
                let t = (s.point_at(u, v) - o).dot(d);
                if r.0.hypot(r.1) < 1e-9
                    && t > 0.0
                    && t <= t_max
                    && !found.iter().any(|x| (x - t).abs() < 1e-7)
                {
                    found.push(t);
                }
            }
        }
    }
    found
}
