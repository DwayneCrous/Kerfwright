//! Signed-distance field of a *parametric* surface (spline, extrusion, revolution, procedural).
//!
//! The level-set tracer needs, for the surface `B` it intersects `A` with, a smooth function that is
//! zero exactly on `B`. For the analytic quadrics that is a closed form ([`crate::implicit`]); for
//! everything else this module builds one: a tessellation of `B`'s parameter window held in a bounding
//! volume hierarchy gives the nearest sample triangle, a few Gauss–Newton steps on
//! `|B(u, v) − x|²` polish the footpoint, and the field value is the signed distance to the tangent
//! plane there, `(x − B(u*, v*)) · n(u*, v*)`. That is the signed distance itself wherever the footpoint
//! is interior, is continuous across `B`, and has the unit normal as its gradient.
//!
//! Outside the region where the footpoint is interior (past a boundary of the window) the field is the
//! distance to the tangent plane at the boundary footpoint: its zero set continues `B` smoothly beyond
//! the patch. The Boolean clips traced curves to both faces afterwards, so the continuation is harmless.

use kw_nurbs::geom::*;
use nalgebra::{Point3, Vector3};
use std::cell::Cell;

use crate::Window;

#[derive(Debug, Clone)]
struct BvhNode {
    min: [f64; 3],
    max: [f64; 3],
    /// Leaf: first quad index and count (`left == u32::MAX`); inner: children indices.
    left: u32,
    right: u32,
    first: u32,
    count: u32,
}

/// Signed distance field of a parametric surface restricted to a finite parameter window.
#[derive(Debug, Clone)]
pub struct DistField {
    s: SurfaceGeom,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
    per_u: bool,
    per_v: bool,
    /// The two ends of the window coincide geometrically (a closed spline whose seam is a crease): the
    /// footpoint of a point near the seam may lie on either side of it.
    closed_u: bool,
    closed_v: bool,
    nu: usize,
    nv: usize,
    pts: Vec<Point3<f64>>,
    quads: Vec<u32>,
    nodes: Vec<BvhNode>,
    /// Footpoint parameters of the last query (the next query is usually a neighbour).
    last: Cell<Option<(f64, f64)>>,
}

/// Number of sample intervals per direction for `s` over `w`.
fn sampling(s: &SurfaceGeom, w: Window) -> (usize, usize) {
    let clampn = |x: usize| x.clamp(8, 96);
    let curve_n = |c: &CurveGeom| -> usize {
        match c {
            CurveGeom::Nurbs(n) => {
                let spans = n.knots().distinct().len().max(2) - 1;
                clampn(4 * spans * n.degree().max(1))
            }
            CurveGeom::Line(_) => 4,
            _ => 48,
        }
    };
    match s {
        SurfaceGeom::Nurbs(n) => {
            let su = n.knots_u().distinct().len().max(2) - 1;
            let sv = n.knots_v().distinct().len().max(2) - 1;
            (
                clampn(4 * su * n.knots_u().degree().max(1)),
                clampn(4 * sv * n.knots_v().degree().max(1)),
            )
        }
        SurfaceGeom::Procedural(p) => {
            let n = &p.approximation;
            let su = n.knots_u().distinct().len().max(2) - 1;
            let sv = n.knots_v().distinct().len().max(2) - 1;
            (
                clampn(4 * su * n.knots_u().degree().max(1)),
                clampn(4 * sv * n.knots_v().degree().max(1)),
            )
        }
        SurfaceGeom::Extrusion { profile, .. } => (curve_n(profile), 4),
        SurfaceGeom::Revolution { profile, .. } => (curve_n(profile), 64),
        _ => {
            let _ = w;
            (48, 48)
        }
    }
}

fn tri_closest(
    p: &Vector3<f64>,
    a: &Vector3<f64>,
    b: &Vector3<f64>,
    c: &Vector3<f64>,
) -> (Vector3<f64>, [f64; 3]) {
    // Ericson, Real-Time Collision Detection 5.1.5; returns closest point and barycentrics (a, b, c)
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (*a, [1.0, 0.0, 0.0]);
    }
    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (*b, [0.0, 1.0, 0.0]);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, [1.0 - v, v, 0.0]);
    }
    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (*c, [0.0, 0.0, 1.0]);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, [1.0 - w, 0.0, w]);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, [0.0, 1.0 - w, w]);
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (a + ab * v + ac * w, [1.0 - v - w, v, w])
}

impl DistField {
    /// `None` when the window is not finite or the surface cannot be sampled.
    pub fn new(s: &SurfaceGeom, w: Window) -> Option<DistField> {
        let (u0, mut u1) = w.u;
        let (v0, mut v1) = w.v;
        if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite())
            || u1 <= u0
            || v1 <= v0
        {
            return None;
        }
        let mut per_u = false;
        let mut per_v = false;
        if let Some(p) = s.period_u() {
            if u1 - u0 >= p - 1e-9 {
                per_u = true;
                u1 = u0 + p;
            }
        }
        if let Some(p) = s.period_v() {
            if v1 - v0 >= p - 1e-9 {
                per_v = true;
                v1 = v0 + p;
            }
        }
        let (nu, nv) = sampling(s, w);
        let mut pts = Vec::with_capacity((nu + 1) * (nv + 1));
        for i in 0..=nu {
            let u = u0 + (u1 - u0) * i as f64 / nu as f64;
            for j in 0..=nv {
                let v = v0 + (v1 - v0) * j as f64 / nv as f64;
                let p = s.point_at(u, v);
                if !p.coords.iter().all(|c| c.is_finite()) {
                    return None;
                }
                pts.push(p);
            }
        }
        // closed directions: the iso-curves at both ends coincide
        let close = |a: &dyn Fn(f64) -> Point3<f64>, b: &dyn Fn(f64) -> Point3<f64>| -> bool {
            (0..=16).all(|k| {
                let t = k as f64 / 16.0;
                (a(t) - b(t)).norm() <= 1e-9 * (1.0 + a(t).coords.amax())
            })
        };
        let closed_u = !per_u
            && close(&|t| s.point_at(u0, v0 + (v1 - v0) * t), &|t| {
                s.point_at(u1, v0 + (v1 - v0) * t)
            });
        let closed_v = !per_v
            && close(&|t| s.point_at(u0 + (u1 - u0) * t, v0), &|t| {
                s.point_at(u0 + (u1 - u0) * t, v1)
            });
        let mut f = DistField {
            s: s.clone(),
            u0,
            u1,
            v0,
            v1,
            per_u,
            per_v,
            closed_u,
            closed_v,
            nu,
            nv,
            pts,
            quads: Vec::new(),
            nodes: Vec::new(),
            last: Cell::new(None),
        };
        f.build_bvh();
        Some(f)
    }

    fn pt(&self, i: usize, j: usize) -> &Point3<f64> {
        &self.pts[i * (self.nv + 1) + j]
    }

    fn quad_box(&self, q: u32) -> ([f64; 3], [f64; 3]) {
        let (i, j) = (q as usize / self.nv, q as usize % self.nv);
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for (a, b) in [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)] {
            let p = self.pt(a, b);
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        (lo, hi)
    }

    fn build_bvh(&mut self) {
        let nq = self.nu * self.nv;
        let mut ids: Vec<u32> = (0..nq as u32).collect();
        let boxes: Vec<([f64; 3], [f64; 3])> = (0..nq as u32).map(|q| self.quad_box(q)).collect();
        let mut nodes: Vec<BvhNode> = Vec::with_capacity(2 * nq / 4 + 2);
        // iterative build with an explicit stack: (node index, range)
        struct Job {
            node: usize,
            lo: usize,
            hi: usize,
        }
        nodes.push(BvhNode {
            min: [0.0; 3],
            max: [0.0; 3],
            left: u32::MAX,
            right: u32::MAX,
            first: 0,
            count: 0,
        });
        let mut stack = vec![Job {
            node: 0,
            lo: 0,
            hi: nq,
        }];
        while let Some(Job { node, lo, hi }) = stack.pop() {
            let mut bmin = [f64::INFINITY; 3];
            let mut bmax = [f64::NEG_INFINITY; 3];
            for &q in &ids[lo..hi] {
                let (a, b) = &boxes[q as usize];
                for k in 0..3 {
                    bmin[k] = bmin[k].min(a[k]);
                    bmax[k] = bmax[k].max(b[k]);
                }
            }
            nodes[node].min = bmin;
            nodes[node].max = bmax;
            if hi - lo <= 4 {
                nodes[node].first = lo as u32;
                nodes[node].count = (hi - lo) as u32;
                continue;
            }
            let ext = [bmax[0] - bmin[0], bmax[1] - bmin[1], bmax[2] - bmin[2]];
            let axis = if ext[0] >= ext[1] && ext[0] >= ext[2] {
                0
            } else if ext[1] >= ext[2] {
                1
            } else {
                2
            };
            ids[lo..hi].sort_by(|x, y| {
                let cx = boxes[*x as usize].0[axis] + boxes[*x as usize].1[axis];
                let cy = boxes[*y as usize].0[axis] + boxes[*y as usize].1[axis];
                cx.partial_cmp(&cy).unwrap_or(std::cmp::Ordering::Equal)
            });
            let mid = (lo + hi) / 2;
            let l = nodes.len();
            for _ in 0..2 {
                nodes.push(BvhNode {
                    min: [0.0; 3],
                    max: [0.0; 3],
                    left: u32::MAX,
                    right: u32::MAX,
                    first: 0,
                    count: 0,
                });
            }
            nodes[node].left = l as u32;
            nodes[node].right = (l + 1) as u32;
            stack.push(Job {
                node: l,
                lo,
                hi: mid,
            });
            stack.push(Job {
                node: l + 1,
                lo: mid,
                hi,
            });
        }
        self.quads = ids;
        self.nodes = nodes;
    }

    fn box_dist2(n: &BvhNode, p: &Point3<f64>) -> f64 {
        let mut d2 = 0.0;
        for k in 0..3 {
            let d = (n.min[k] - p[k]).max(p[k] - n.max[k]).max(0.0);
            d2 += d * d;
        }
        d2
    }

    /// Parameter guess `(u, v)` of the nearest point of the tessellation to `p`.
    fn nearest_sample(&self, p: &Point3<f64>) -> (f64, f64) {
        let mut best = (f64::INFINITY, (self.u0, self.v0));
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let n = &self.nodes[ni];
            if Self::box_dist2(n, p) >= best.0 {
                continue;
            }
            if n.left == u32::MAX {
                for &q in &self.quads[n.first as usize..(n.first + n.count) as usize] {
                    let (i, j) = (q as usize / self.nv, q as usize % self.nv);
                    let c = [
                        self.pt(i, j).coords,
                        self.pt(i + 1, j).coords,
                        self.pt(i + 1, j + 1).coords,
                        self.pt(i, j + 1).coords,
                    ];
                    let uv = |a: usize, b: usize, bc: [f64; 3], idx: [usize; 3]| {
                        // barycentric blend of corner parameters (corners: 0 (i,j) 1 (i+1,j) 2 (i+1,j+1) 3 (i,j+1))
                        let _ = (a, b);
                        let cu = [0.0, 1.0, 1.0, 0.0];
                        let cv = [0.0, 0.0, 1.0, 1.0];
                        let fu: f64 = (0..3).map(|k| bc[k] * cu[idx[k]]).sum();
                        let fv: f64 = (0..3).map(|k| bc[k] * cv[idx[k]]).sum();
                        (
                            self.u0 + (self.u1 - self.u0) * (i as f64 + fu) / self.nu as f64,
                            self.v0 + (self.v1 - self.v0) * (j as f64 + fv) / self.nv as f64,
                        )
                    };
                    for idx in [[0usize, 1, 2], [0, 2, 3]] {
                        let (cp, bc) = tri_closest(&p.coords, &c[idx[0]], &c[idx[1]], &c[idx[2]]);
                        let d2 = (cp - p.coords).norm_squared();
                        if d2 < best.0 {
                            best = (d2, uv(i, j, bc, idx));
                        }
                    }
                }
            } else {
                let (l, r) = (n.left as usize, n.right as usize);
                let (dl, dr) = (
                    Self::box_dist2(&self.nodes[l], p),
                    Self::box_dist2(&self.nodes[r], p),
                );
                // visit the nearer child first (pushed last)
                if dl < dr {
                    stack.push(r);
                    stack.push(l);
                } else {
                    stack.push(l);
                    stack.push(r);
                }
            }
        }
        best.1
    }

    fn clamp_uv(&self, mut u: f64, mut v: f64) -> (f64, f64) {
        if !self.per_u {
            u = u.clamp(self.u0, self.u1);
        }
        if !self.per_v {
            v = v.clamp(self.v0, self.v1);
        }
        (u, v)
    }

    /// Point and first partial derivatives at `(u, v)` (one evaluation for the spline surfaces).
    fn pd(&self, u: f64, v: f64) -> (Point3<f64>, Vector3<f64>, Vector3<f64>) {
        match &self.s {
            SurfaceGeom::Nurbs(n) => {
                let d = n.derivatives(u, v, 1);
                (Point3::from(d[0][0]), d[1][0], d[0][1])
            }
            SurfaceGeom::Procedural(pr) => {
                let d = pr.approximation.derivatives(u, v, 1);
                (Point3::from(d[0][0]), d[1][0], d[0][1])
            }
            s => {
                let (su, sv) = s.derivatives(u, v);
                (s.point_at(u, v), su, sv)
            }
        }
    }

    /// Gauss–Newton polish of the footpoint of `x`, starting at `(u, v)`; returns the final
    /// parameters with the surface point and partials there.
    fn polish(
        &self,
        x: &Point3<f64>,
        u: f64,
        v: f64,
    ) -> (f64, f64, Point3<f64>, Vector3<f64>, Vector3<f64>) {
        let (mut u, mut v) = (u, v);
        let (mut p, mut su, mut sv) = self.pd(u, v);
        let mut best = (p - x).norm_squared();
        let span = (self.u1 - self.u0).abs().max((self.v1 - self.v0).abs());
        for _ in 0..24 {
            let r = p - x;
            let a11 = su.dot(&su);
            let a12 = su.dot(&sv);
            let a22 = sv.dot(&sv);
            let det = a11 * a22 - a12 * a12;
            if det.abs() <= 1e-24 * a11.max(1e-300) * a22.max(1e-300) {
                break;
            }
            let (b1, b2) = (-r.dot(&su), -r.dot(&sv));
            let (mut du, mut dv) = ((a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det);
            if !du.is_finite() || !dv.is_finite() {
                break;
            }
            let lim = 0.25 * span;
            let l = du.abs().max(dv.abs());
            if l > lim {
                du *= lim / l;
                dv *= lim / l;
            }
            let mut t = 1.0;
            let mut accepted = false;
            for _ in 0..6 {
                let (nu_, nv_) = self.clamp_uv(u + t * du, v + t * dv);
                let (np, nsu, nsv) = self.pd(nu_, nv_);
                let d = (np - x).norm_squared();
                if d <= best * (1.0 + 1e-14) {
                    let moved = (nu_ - u).abs().max((nv_ - v).abs());
                    u = nu_;
                    v = nv_;
                    p = np;
                    su = nsu;
                    sv = nsv;
                    best = d;
                    accepted = true;
                    // far points need only a rough footpoint; near ones converge to machine precision
                    let step3 = (su * (t * du)).norm().max((sv * (t * dv)).norm());
                    if moved < 1e-13 * (1.0 + span) || step3 <= 1e-3 * d.sqrt() * 1e-3 {
                        return (u, v, p, su, sv);
                    }
                    break;
                }
                t *= 0.5;
            }
            if !accepted {
                break;
            }
        }
        (u, v, p, su, sv)
    }

    /// Polished footpoint of `x`, trying the other side of a seam when the first guess ends near it.
    fn solve(&self, x: &Point3<f64>) -> (f64, f64, Point3<f64>, Vector3<f64>, Vector3<f64>) {
        let (u, v) = self.nearest_sample(x);
        let first = self.polish(x, u, v);
        let near_u = self.closed_u
            && ((first.0 - self.u0).abs() < 0.05 * (self.u1 - self.u0)
                || (self.u1 - first.0).abs() < 0.05 * (self.u1 - self.u0));
        let near_v = self.closed_v
            && ((first.1 - self.v0).abs() < 0.05 * (self.v1 - self.v0)
                || (self.v1 - first.1).abs() < 0.05 * (self.v1 - self.v0));
        if !(near_u || near_v) {
            return first;
        }
        let mut best = first;
        let mut best_d = (first.2 - x).norm_squared();
        let flip = |c: f64, lo: f64, hi: f64| {
            if (c - lo).abs() < (hi - c).abs() {
                hi
            } else {
                lo
            }
        };
        let mut starts: Vec<(f64, f64)> = Vec::new();
        if near_u {
            starts.push((flip(first.0, self.u0, self.u1), first.1));
        }
        if near_v {
            starts.push((first.0, flip(first.1, self.v0, self.v1)));
        }
        if near_u && near_v {
            starts.push((
                flip(first.0, self.u0, self.u1),
                flip(first.1, self.v0, self.v1),
            ));
        }
        for (su, sv) in starts {
            // start slightly inside the window on the far side so the first step can cross the seam region
            let cand = self.polish(x, su, sv);
            let d = (cand.2 - x).norm_squared();
            if d < best_d {
                best = cand;
                best_d = d;
            }
        }
        best
    }

    /// Footpoint parameters of `x` on the surface.
    pub fn foot(&self, x: &Point3<f64>) -> (f64, f64) {
        let (u, v, ..) = self.solve(x);
        self.last.set(Some((u, v)));
        (u, v)
    }

    /// Signed distance value and unit gradient at `x`.
    pub fn eval(&self, x: &Point3<f64>) -> (f64, Vector3<f64>) {
        let (u, v, p, su, sv) = self.solve(x);
        self.last.set(Some((u, v)));
        let d = x - p;
        let n = su.cross(&sv);
        let l = n.norm();
        if l < 1e-300 {
            return (d.norm(), Vector3::zeros());
        }
        let mut n = n / l;
        // a footpoint on the seam of a closed surface is on a crease: use the bisector of the two
        // one-sided normals, so the sign is right in the whole wedge beside the crease
        let eps_u = 1e-12 * (self.u1 - self.u0).abs();
        let eps_v = 1e-12 * (self.v1 - self.v0).abs();
        let on_u = self.closed_u && ((u - self.u0).abs() <= eps_u || (self.u1 - u).abs() <= eps_u);
        let on_v = self.closed_v && ((v - self.v0).abs() <= eps_v || (self.v1 - v).abs() <= eps_v);
        if on_u || on_v {
            let normal_at = |uu: f64, vv: f64| {
                let (a, b) = self.s.derivatives(uu, vv);
                let c = a.cross(&b);
                let l = c.norm();
                (l > 1e-300).then(|| c / l)
            };
            let (na, nb) = if on_u {
                (normal_at(self.u0, v), normal_at(self.u1, v))
            } else {
                (normal_at(u, self.v0), normal_at(u, self.v1))
            };
            if let (Some(na), Some(nb)) = (na, nb) {
                let m = na + nb;
                if m.norm() > 1e-6 {
                    n = m / m.norm();
                }
            }
        }
        (d.dot(&n), n)
    }

    /// Parameter window the field covers (one period in a periodic direction).
    pub fn window(&self) -> Window {
        Window {
            u: (self.u0, self.u1),
            v: (self.v0, self.v1),
        }
    }

    pub fn periodic(&self) -> (bool, bool) {
        (self.per_u, self.per_v)
    }

    pub fn surface(&self) -> &SurfaceGeom {
        &self.s
    }
}
