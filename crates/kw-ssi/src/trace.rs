//! Level-set tracing: the intersection of a parametric surface `A(u, v)` with an implicit
//! surface `f(x) = 0` is the zero set of `g(u, v) = f(A(u, v))`.
//!
//! 1. marching squares on a (periodically wrapped) grid over `A`'s parameter window finds
//!    every contour component,
//! 2. each contour vertex is Newton-projected onto `g = 0` to machine precision,
//! 3. nodes are refined adaptively until a C¹ Hermite spline in *arc length* reproduces the
//!    curve and both pcurves to the requested tolerance.

use crate::implicit::Implicit;
use crate::{SsiError, Window};
use kw_nurbs::fit::hermite_curve;
use kw_nurbs::geom::*;
use kw_nurbs::proj::{surface_param_of, wrap_near};
use kw_nurbs::NurbsCurve;
use nalgebra::{Matrix3x2, Point3, Vector2, Vector3};
use std::collections::HashMap;

pub struct Traced {
    pub curve: NurbsCurve,
    pub pcurve_a: NurbsCurve,
    pub pcurve_b: NurbsCurve,
    pub range: (f64, f64),
    pub closed: bool,
}

#[derive(Clone, Copy)]
struct Node {
    uv: (f64, f64),
    x: Point3<f64>,
    t: Vector3<f64>,
    duv: (f64, f64),
    uvb: (f64, f64),
    duvb: (f64, f64),
}

struct Ctx<'a> {
    a: &'a SurfaceGeom,
    b: &'a SurfaceGeom,
    imp: &'a Implicit,
    wb_center: (f64, f64),
    wb: Window,
    scale: f64,
    /// Parameter bounds of non-periodic directions on `a` (evaluation outside is clamped/undefined).
    bound_u: Option<(f64, f64)>,
    bound_v: Option<(f64, f64)>,
}

impl Ctx<'_> {
    fn g(&self, u: f64, v: f64) -> f64 {
        self.imp.eval(&self.a.point_at(u, v)).0
    }

    /// g and its uv-gradient.
    fn g_grad(&self, u: f64, v: f64) -> (f64, f64, f64) {
        let x = self.a.point_at(u, v);
        let (f, gf) = self.imp.eval(&x);
        let (su, sv) = self.a.derivatives(u, v);
        (f, gf.dot(&su), gf.dot(&sv))
    }

    /// Newton-project `(u, v)` onto g = 0 along the gradient, staying inside the bounded
    /// parameter directions (active-set: a variable pinned at its bound is held there).
    fn project(&self, u: f64, v: f64, max_step: f64) -> Option<(f64, f64)> {
        self.project_held(u, v, max_step, false, false)
    }

    /// As [`project`](Self::project), keeping `u` and/or `v` fixed (a contour end that lies on the
    /// boundary of the window must stay there: it is where the curve crosses a seam or an edge).
    fn project_held(
        &self,
        mut u: f64,
        mut v: f64,
        max_step: f64,
        hold_u: bool,
        hold_v: bool,
    ) -> Option<(f64, f64)> {
        let clampu = |x: f64| self.bound_u.map_or(x, |(a, b)| x.clamp(a, b));
        let clampv = |x: f64| self.bound_v.map_or(x, |(a, b)| x.clamp(a, b));
        u = clampu(u);
        v = clampv(v);
        for _ in 0..60 {
            let (g, mut gu, mut gv) = self.g_grad(u, v);
            if g.abs() <= 1e-15 * self.scale {
                return Some((u, v));
            }
            // pinned variables cannot move further outwards
            if let Some((a, b)) = self.bound_u {
                if (u <= a && g * gu > 0.0) || (u >= b && g * gu < 0.0) {
                    gu = 0.0;
                }
            }
            if let Some((a, b)) = self.bound_v {
                if (v <= a && g * gv > 0.0) || (v >= b && g * gv < 0.0) {
                    gv = 0.0;
                }
            }
            if hold_u {
                gu = 0.0;
            }
            if hold_v {
                gv = 0.0;
            }
            let n2 = gu * gu + gv * gv;
            if n2 < 1e-300 {
                return None;
            }
            let (mut du, mut dv) = (-g * gu / n2, -g * gv / n2);
            let l = du.hypot(dv);
            if l > max_step {
                du *= max_step / l;
                dv *= max_step / l;
            }
            u = clampu(u + du);
            v = clampv(v + dv);
        }
        (self.g(u, v).abs() <= 1e-13 * self.scale).then_some((u, v))
    }

    /// Parameters of the point `x` of the intersection on surface B (near `hint` in periodic directions).
    fn param_b(&self, x: &Point3<f64>, hint: (f64, f64)) -> (f64, f64) {
        match self.imp.foot(x) {
            Some((fu, fv)) => (
                self.b.period_u().map_or(fu, |p| wrap_near(fu, hint.0, p)),
                self.b.period_v().map_or(fv, |p| wrap_near(fv, hint.1, p)),
            ),
            None => {
                let pb = surface_param_of(self.b, x, Some(hint));
                (pb.u, pb.v)
            }
        }
    }

    fn node_at(
        &self,
        uv: (f64, f64),
        uvb_hint: Option<(f64, f64)>,
        dir_hint: Option<(f64, f64)>,
    ) -> Option<Node> {
        let (g, gu, gv) = self.g_grad(uv.0, uv.1);
        let _ = g;
        let n = gu.hypot(gv);
        if n < 1e-12 * (1.0 + self.scale) * 1e-3 {
            return None;
        }
        let mut d = (-gv / n, gu / n);
        if let Some(h) = dir_hint {
            if d.0 * h.0 + d.1 * h.1 < 0.0 {
                d = (-d.0, -d.1);
            }
        }
        let x = self.a.point_at(uv.0, uv.1);
        let (su, sv) = self.a.derivatives(uv.0, uv.1);
        let x1 = su * d.0 + sv * d.1;
        let l = x1.norm();
        if l < 1e-12 {
            return None;
        }
        let t = x1 / l;
        let duv = (d.0 / l, d.1 / l);
        let hint = uvb_hint.unwrap_or(self.wb_center);
        let uvb = self.param_b(&x, hint);
        let duvb = uv_velocity(self.b, uvb, &t).or_else(|| {
            // singular parameterisation: finite difference along the curve
            let h = 1e-6 * (1.0 + self.scale);
            let xa = self.a.point_at(uv.0 - duv.0 * h, uv.1 - duv.1 * h);
            let xb = self.a.point_at(uv.0 + duv.0 * h, uv.1 + duv.1 * h);
            let pa = self.param_b(&xa, uvb);
            let pbb = self.param_b(&xb, uvb);
            Some(((pbb.0 - pa.0) / (2.0 * h), (pbb.1 - pa.1) / (2.0 * h)))
        })?;
        Some(Node {
            uv,
            x,
            t,
            duv,
            uvb,
            duvb,
        })
    }
}

fn uv_velocity(s: &SurfaceGeom, uv: (f64, f64), c1: &Vector3<f64>) -> Option<(f64, f64)> {
    let (su, sv) = s.derivatives(uv.0, uv.1);
    let j = Matrix3x2::from_columns(&[su, sv]);
    let jt = j.transpose();
    let a = jt * j;
    let scale = a[(0, 0)] * a[(1, 1)];
    if a.determinant().abs().is_nan() || a.determinant().abs() <= 1e-10 * scale.max(1e-300) {
        return None;
    }
    let sol = a.try_inverse()? * (jt * c1);
    Some((sol.x, sol.y))
}

// ------------------------------------------------------------------ Hermite helpers

fn herm(
    p0: &Vector3<f64>,
    p1: &Vector3<f64>,
    d0: &Vector3<f64>,
    d1: &Vector3<f64>,
    h: f64,
    tau: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let s = tau / h;
    let (s2, s3) = (s * s, s * s * s);
    let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    let h10 = s3 - 2.0 * s2 + s;
    let h01 = -2.0 * s3 + 3.0 * s2;
    let h11 = s3 - s2;
    let p = p0 * h00 + d0 * (h10 * h) + p1 * h01 + d1 * (h11 * h);
    let (e00, e10, e01, e11) = (
        (6.0 * s2 - 6.0 * s) / h,
        3.0 * s2 - 4.0 * s + 1.0,
        (-6.0 * s2 + 6.0 * s) / h,
        3.0 * s2 - 2.0 * s,
    );
    let dp = p0 * e00 + d0 * e10 + p1 * e01 + d1 * e11;
    (p, dp)
}

const GL5: [(f64, f64); 5] = [
    (0.0, 0.568_888_888_888_888_9),
    (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
    (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
    (0.906_179_845_938_664, 0.236_926_885_056_189_08),
    (-0.906_179_845_938_664, 0.236_926_885_056_189_08),
];

/// Arc-length parameters of the nodes (fixed point of Hermite arc-length integration).
fn arclength_params(nodes: &[Node]) -> Vec<f64> {
    let n = nodes.len();
    let mut h: Vec<f64> = (0..n - 1)
        .map(|i| (nodes[i + 1].x - nodes[i].x).norm().max(1e-300))
        .collect();
    for _ in 0..5 {
        for i in 0..n - 1 {
            let (a, b) = (&nodes[i], &nodes[i + 1]);
            let (p0, p1) = (a.x.coords, b.x.coords);
            let mut len = 0.0;
            for (xi, w) in GL5 {
                let tau = 0.5 * h[i] * (1.0 + xi);
                let (_, dp) = herm(&p0, &p1, &a.t, &b.t, h[i], tau);
                len += w * dp.norm();
            }
            h[i] = (0.5 * h[i] * len).max(1e-300);
        }
    }
    let mut s = vec![0.0; n];
    for i in 0..n - 1 {
        s[i + 1] = s[i] + h[i];
    }
    s
}

// ------------------------------------------------------------------ marching squares

struct Grid {
    nu: usize,
    nv: usize,
    u0: f64,
    v0: f64,
    du: f64,
    dv: f64,
    per_u: bool,
    per_v: bool,
    g: Vec<f64>, // (nu+1)*(nv+1), index i*(nv+1)+j
}

impl Grid {
    fn node(&self, i: i64, j: i64) -> f64 {
        let ii = if self.per_u {
            i.rem_euclid(self.nu as i64)
        } else {
            i
        } as usize;
        let jj = if self.per_v {
            j.rem_euclid(self.nv as i64)
        } else {
            j
        } as usize;
        self.g[ii * (self.nv + 1) + jj]
    }
    fn pos(&self, i: i64, j: i64) -> (f64, f64) {
        (self.u0 + self.du * i as f64, self.v0 + self.dv * j as f64)
    }
    fn cell_exists(&self, ci: i64, cj: i64) -> bool {
        (self.per_u || (0..self.nu as i64).contains(&ci))
            && (self.per_v || (0..self.nv as i64).contains(&cj))
    }
    fn canon(&self, ci: i64, cj: i64) -> (usize, usize) {
        let a = if self.per_u {
            ci.rem_euclid(self.nu as i64)
        } else {
            ci
        } as usize;
        let b = if self.per_v {
            cj.rem_euclid(self.nv as i64)
        } else {
            cj
        } as usize;
        (a, b)
    }
}

/// Segments of a cell as pairs of local edges (0 bottom, 1 right, 2 top, 3 left).
fn cell_segments(grid: &Grid, ci: i64, cj: i64) -> Vec<(u8, u8)> {
    let c = [
        grid.node(ci, cj),
        grid.node(ci + 1, cj),
        grid.node(ci + 1, cj + 1),
        grid.node(ci, cj + 1),
    ];
    let pos = [c[0] > 0.0, c[1] > 0.0, c[2] > 0.0, c[3] > 0.0];
    // edge e joins corners e and e+1
    let active: Vec<u8> = (0..4u8)
        .filter(|e| pos[*e as usize] != pos[((*e + 1) % 4) as usize])
        .collect();
    match active.len() {
        2 => vec![(active[0], active[1])],
        4 => {
            let centre = 0.25 * (c[0] + c[1] + c[2] + c[3]);
            if (centre > 0.0) == pos[0] {
                // corner 0 and 2 joined through the centre: segments isolate corners 1 and 3
                vec![(0, 1), (2, 3)]
            } else {
                vec![(3, 0), (1, 2)]
            }
        }
        _ => vec![],
    }
}

/// Crossing position on local edge `e` of unwrapped cell `(ci, cj)`.
fn crossing(grid: &Grid, ci: i64, cj: i64, e: u8) -> (f64, f64) {
    let ((i0, j0), (i1, j1)) = match e {
        0 => ((ci, cj), (ci + 1, cj)),
        1 => ((ci + 1, cj), (ci + 1, cj + 1)),
        2 => ((ci, cj + 1), (ci + 1, cj + 1)),
        _ => ((ci, cj), (ci, cj + 1)),
    };
    let (g0, g1) = (grid.node(i0, j0), grid.node(i1, j1));
    let w = if (g0 - g1).abs() > 0.0 {
        g0 / (g0 - g1)
    } else {
        0.5
    };
    let (p0, p1) = (grid.pos(i0, j0), grid.pos(i1, j1));
    (p0.0 + (p1.0 - p0.0) * w, p0.1 + (p1.1 - p0.1) * w)
}

fn neighbour(ci: i64, cj: i64, e: u8) -> (i64, i64, u8) {
    match e {
        0 => (ci, cj - 1, 2),
        1 => (ci + 1, cj, 3),
        2 => (ci, cj + 1, 0),
        _ => (ci - 1, cj, 1),
    }
}

struct Polyline {
    uv: Vec<(f64, f64)>,
    closed_3d: bool,
}

fn contours(grid: &Grid) -> Vec<Polyline> {
    let mut segs: HashMap<(usize, usize), Vec<(u8, u8)>> = HashMap::new();
    for ci in 0..grid.nu as i64 {
        for cj in 0..grid.nv as i64 {
            let s = cell_segments(grid, ci, cj);
            if !s.is_empty() {
                segs.insert((ci as usize, cj as usize), s);
            }
        }
    }
    let mut visited: HashMap<(usize, usize, usize), bool> = HashMap::new();
    let mut out = Vec::new();

    // walk from (ci, cj) entering through edge `entry`
    let walk = |ci0: i64,
                cj0: i64,
                entry0: u8,
                visited: &mut HashMap<(usize, usize, usize), bool>|
     -> Option<Polyline> {
        let mut pts = vec![crossing(grid, ci0, cj0, entry0)];
        let (mut ci, mut cj, mut entry) = (ci0, cj0, entry0);
        let start_canon = grid.canon(ci0, cj0);
        let mut steps = 0usize;
        loop {
            steps += 1;
            if steps > 4 * grid.nu * grid.nv + 16 {
                return None;
            }
            let key = grid.canon(ci, cj);
            let list = segs.get(&key)?;
            let idx = list.iter().position(|s| s.0 == entry || s.1 == entry)?;
            visited.insert((key.0, key.1, idx), true);
            let s = list[idx];
            let out_e = if s.0 == entry { s.1 } else { s.0 };
            pts.push(crossing(grid, ci, cj, out_e));
            let (ni, nj, ne) = neighbour(ci, cj, out_e);
            if !grid.cell_exists(ni, nj) {
                return Some(Polyline {
                    uv: pts,
                    closed_3d: false,
                });
            }
            // returned to the start cell through the start edge?
            if grid.canon(ni, nj) == start_canon && ne == entry0 {
                let closed_uv = (ni, nj) == (ci0, cj0);
                // the final vertex duplicates the first (shifted by whole periods if wrapped)
                let last = *pts.last().unwrap();
                let _ = last;
                let first_shifted = crossing(grid, ni, nj, ne);
                *pts.last_mut().unwrap() = first_shifted;
                return Some(Polyline {
                    uv: pts,
                    closed_3d: true,
                })
                .map(|mut p| {
                    if closed_uv {
                        // exact repeat of the start for contractible loops
                        let f = p.uv[0];
                        *p.uv.last_mut().unwrap() = f;
                    }
                    p
                });
            }
            ci = ni;
            cj = nj;
            entry = ne;
        }
    };

    // open contours first: start from cells having an edge on the grid boundary
    let mut starts: Vec<(i64, i64, u8)> = Vec::new();
    for (&(ci, cj), list) in &segs {
        for s in list {
            for e in [s.0, s.1] {
                let (ni, nj, _) = neighbour(ci as i64, cj as i64, e);
                if !grid.cell_exists(ni, nj) {
                    starts.push((ci as i64, cj as i64, e));
                }
            }
        }
    }
    starts.sort();
    for (ci, cj, e) in starts {
        let key = grid.canon(ci, cj);
        let list = &segs[&key];
        let idx = list.iter().position(|s| s.0 == e || s.1 == e).unwrap();
        if visited.contains_key(&(key.0, key.1, idx)) {
            continue;
        }
        if let Some(p) = walk(ci, cj, e, &mut visited) {
            out.push(p);
        }
    }
    // then closed contours
    let mut keys: Vec<_> = segs.keys().copied().collect();
    keys.sort();
    for key in keys {
        let list = &segs[&key];
        for (idx, s) in list.iter().enumerate() {
            if visited.contains_key(&(key.0, key.1, idx)) {
                continue;
            }
            if let Some(p) = walk(key.0 as i64, key.1 as i64, s.0, &mut visited) {
                out.push(p);
            }
        }
    }
    out
}

/// Closed contours smaller than a grid cell: a cell whose corners all share a sign can still hide a
/// loop around a local extremum of `g`. Candidate cells (corner value small relative to the local
/// gradient) are searched for a sign flip and, if found, re-contoured on a finer local grid.
fn small_loops(ctx: &Ctx, grid: &Grid) -> Vec<Polyline> {
    let mut out: Vec<Polyline> = Vec::new();
    let diag = grid.du.hypot(grid.dv);
    for ci in 0..grid.nu as i64 {
        for cj in 0..grid.nv as i64 {
            let c = [
                grid.node(ci, cj),
                grid.node(ci + 1, cj),
                grid.node(ci + 1, cj + 1),
                grid.node(ci, cj + 1),
            ];
            let sign = if c[0] > 0.0 { 1.0 } else { -1.0 };
            if c.iter().any(|v| (*v > 0.0) != (c[0] > 0.0)) {
                continue;
            }
            let (p0, _p1) = (grid.pos(ci, cj), grid.pos(ci + 1, cj + 1));
            let centre = (p0.0 + 0.5 * grid.du, p0.1 + 0.5 * grid.dv);
            let (_, gu, gv) = ctx.g_grad(centre.0, centre.1);
            let gn = gu.hypot(gv);
            let gmin = c.iter().fold(f64::INFINITY, |m, v| m.min(v.abs()));
            // a loop inside the cell needs |g| to dip across zero: only cells close to the surface qualify
            if gmin > 1.5 * gn.max(1e-12) * diag + 1e-9 * ctx.scale {
                continue;
            }
            // a hidden loop surrounds a local extremum of g: its corner gradients then point (mostly) away
            // from the cell centre, towards growing s·g
            let corners = [
                p0,
                (p0.0 + grid.du, p0.1),
                (p0.0 + grid.du, p0.1 + grid.dv),
                (p0.0, p0.1 + grid.dv),
            ];
            let outward = corners
                .iter()
                .filter(|q| {
                    let (_, gu, gv) = ctx.g_grad(q.0, q.1);
                    sign * (gu * (q.0 - centre.0) + gv * (q.1 - centre.1)) > 0.0
                })
                .count();
            if outward < 3 {
                continue;
            }
            // descend s·g inside the (slightly enlarged) cell
            let mut found: Option<(f64, f64)> = None;
            let starts = [
                (p0.0, p0.1),
                (p0.0 + grid.du, p0.1),
                (p0.0 + grid.du, p0.1 + grid.dv),
                (p0.0, p0.1 + grid.dv),
                centre,
            ];
            for st in starts {
                let (mut u, mut v) = st;
                let mut step = 0.25 * grid.du.min(grid.dv);
                let mut cur = sign * ctx.g(u, v);
                for _ in 0..60 {
                    if cur < 0.0 {
                        found = Some((u, v));
                        break;
                    }
                    let (_, gu, gv) = ctx.g_grad(u, v);
                    let n = gu.hypot(gv);
                    if n < 1e-14 {
                        break;
                    }
                    let (du_, dv_) = (-sign * gu / n * step, -sign * gv / n * step);
                    let (nu_, nv_) = (
                        (u + du_).clamp(p0.0 - 0.5 * grid.du, p0.0 + 1.5 * grid.du),
                        (v + dv_).clamp(p0.1 - 0.5 * grid.dv, p0.1 + 1.5 * grid.dv),
                    );
                    let nxt = sign * ctx.g(nu_, nv_);
                    if nxt < cur {
                        u = nu_;
                        v = nv_;
                        cur = nxt;
                    } else {
                        step *= 0.5;
                        if step < 1e-9 * grid.du {
                            break;
                        }
                    }
                }
                if found.is_some() {
                    break;
                }
            }
            let Some(seed) = found else { continue };
            // fine local grid around the seed
            let m = 40usize;
            let half = (1.5 * grid.du, 1.5 * grid.dv);
            let (lu0, lv0) = (seed.0 - half.0, seed.1 - half.1);
            let (ldu, ldv) = (2.0 * half.0 / m as f64, 2.0 * half.1 / m as f64);
            let mut g = vec![0.0; (m + 1) * (m + 1)];
            for i in 0..=m {
                for j in 0..=m {
                    let v = ctx.g(lu0 + ldu * i as f64, lv0 + ldv * j as f64);
                    g[i * (m + 1) + j] = if v == 0.0 { 1e-300 } else { v };
                }
            }
            let sub = Grid {
                nu: m,
                nv: m,
                u0: lu0,
                v0: lv0,
                du: ldu,
                dv: ldv,
                per_u: false,
                per_v: false,
                g,
            };
            for poly in contours(&sub) {
                // only loops fully inside the local window (open pieces belong to the main grid)
                if !poly.closed_3d || poly.uv.len() < 3 {
                    continue;
                }
                let inside = poly.uv.iter().all(|p| {
                    p.0 > lu0 + 1.5 * ldu
                        && p.0 < lu0 + 2.0 * half.0 - 1.5 * ldu
                        && p.1 > lv0 + 1.5 * ldv
                        && p.1 < lv0 + 2.0 * half.1 - 1.5 * ldv
                });
                if !inside {
                    continue;
                }
                let cen = poly
                    .uv
                    .iter()
                    .fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
                let cen = (cen.0 / poly.uv.len() as f64, cen.1 / poly.uv.len() as f64);
                let dup = out.iter().any(|q| {
                    let qc = q.uv.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
                    let qc = (qc.0 / q.uv.len() as f64, qc.1 / q.uv.len() as f64);
                    (qc.0 - cen.0).hypot(qc.1 - cen.1) < 4.0 * ldu.max(ldv)
                });
                if !dup {
                    out.push(poly);
                }
            }
        }
    }
    out
}

/// Do two traced curves lie on top of each other (one covers the other)?
fn same_curve(a: &Traced, b: &Traced, scale: f64) -> bool {
    let dense = |c: &Traced, n: usize| -> Vec<Point3<f64>> {
        (0..=n)
            .map(|i| {
                c.curve
                    .point_at(c.range.0 + (c.range.1 - c.range.0) * i as f64 / n as f64)
            })
            .collect()
    };
    let (pa, pb) = (dense(a, 3000), dense(b, 64));
    let tol = 1e-6 * scale;
    pb.iter().all(|q| {
        let mut best = f64::INFINITY;
        for w in pa.windows(2) {
            let d = w[1] - w[0];
            let l2 = d.norm_squared();
            let t = if l2 > 0.0 {
                ((q - w[0]).dot(&d) / l2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            best = best.min((q - (w[0] + d * t)).norm());
            if best < tol {
                return true;
            }
        }
        best < tol
    })
}

fn dist_to_polyline(p: (f64, f64), poly: &[(f64, f64)]) -> f64 {
    let mut best = f64::INFINITY;
    for w in poly.windows(2) {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min((p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy));
    }
    best
}

// ------------------------------------------------------------------ main entry

fn period_of(s: &SurfaceGeom, dir_u: bool) -> Option<f64> {
    if dir_u {
        s.period_u()
    } else {
        s.period_v()
    }
}

pub fn trace(
    a: &SurfaceGeom,
    wa: Window,
    b: &SurfaceGeom,
    wb: Window,
    imp: &Implicit,
    tol: f64,
) -> Result<Vec<Traced>, SsiError> {
    let n = 72usize;
    // periodic directions that the window covers entirely wrap around
    let mut per_u = false;
    let mut per_v = false;
    let (u0, mut u1) = wa.u;
    let (v0, mut v1) = wa.v;
    if let Some(p) = period_of(a, true) {
        if u1 - u0 >= p - 1e-9 {
            per_u = true;
            u1 = u0 + p;
        }
    }
    if let Some(p) = period_of(a, false) {
        if v1 - v0 >= p - 1e-9 {
            per_v = true;
            v1 = v0 + p;
        }
    }
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite())
        || u1 <= u0
        || v1 <= v0
    {
        return Err(SsiError::Unsupported(
            "tracing needs finite, non-empty parameter windows",
        ));
    }
    // avoid degenerate rows at exact poles by nudging windows is unnecessary: g is smooth there
    let grid_pts = (u0, u1, v0, v1);
    let _ = grid_pts;
    let (du, dv) = ((u1 - u0) / n as f64, (v1 - v0) / n as f64);
    let scale = 1.0
        + a.point_at(0.5 * (u0 + u1), 0.5 * (v0 + v1)).coords.amax()
        + (a.point_at(u0, v0) - a.point_at(u1, v1)).norm();
    let ctx = Ctx {
        a,
        b,
        imp,
        wb_center: (0.5 * (wb.u.0 + wb.u.1), 0.5 * (wb.v.0 + wb.v.1)),
        wb,
        scale,
        bound_u: (!per_u && a.period_u().is_none()).then_some((u0, u1)),
        bound_v: (!per_v && a.period_v().is_none()).then_some((v0, v1)),
    };
    let mut g = vec![0.0; (n + 1) * (n + 1)];
    for i in 0..=n {
        for j in 0..=n {
            let (iu, ju) = (
                if per_u && i == n { 0 } else { i },
                if per_v && j == n { 0 } else { j },
            );
            let v = ctx.g(u0 + du * iu as f64, v0 + dv * ju as f64);
            g[i * (n + 1) + j] = if v == 0.0 { 1e-300 } else { v };
        }
    }
    let grid = Grid {
        nu: n,
        nv: n,
        u0,
        v0,
        du,
        dv,
        per_u,
        per_v,
        g,
    };
    let mut polys = contours(&grid);
    let extra = small_loops(&ctx, &grid);
    // a loop the main grid already traced (it spans at least one cell edge) must not be added again
    for e in extra {
        let dup = polys.iter().any(|p| {
            e.uv.iter()
                .all(|q| dist_to_polyline(*q, &p.uv) < 0.2 * du.hypot(dv))
        });
        if !dup {
            polys.push(e);
        }
    }
    polys = restrict_to_b_window(&ctx, polys, du.hypot(dv));
    let mut result: Vec<Traced> = Vec::new();
    for poly in polys {
        if poly.uv.len() < 2 {
            continue;
        }
        if let Some(t) = refine_polyline(&ctx, &poly, du.hypot(dv), tol)? {
            // the same intersection curve may have been found twice (main grid and local refinement)
            if !result.iter().any(|r| same_curve(r, &t, ctx.scale)) {
                result.push(t);
            }
        }
    }
    Ok(result)
}

/// How a point of the zero set relates to the window of surface `B`.
#[derive(Clone, Copy, PartialEq)]
enum Side {
    /// The footpoint is interior (or on the boundary): the point lies on the surface patch.
    In { uvb: (f64, f64) },
    /// Past a boundary of the window, or the projection failed.
    Out,
}

/// Cut traced contours to the parameter window of `B`. Beyond it the zero set is only a continuation
/// of the surface (for a distance field there is no valid pcurve on `B` there; for an analytic
/// surface the continuation may run through singularities such as a cone apex), and across the seam
/// of a closed spline the footpoint jumps by a whole parameter range. Each maximal run of vertices
/// lying in the window becomes an open contour whose ends are located on the window boundary (or the
/// seam) by bisection.
fn restrict_to_b_window(ctx: &Ctx, polys: Vec<Polyline>, cell: f64) -> Vec<Polyline> {
    let field = ctx.imp.as_field();
    let w = ctx.wb;
    let (per_u, per_v) = match field {
        Some(f) => f.periodic(),
        None => (
            ctx.b.period_u().is_some_and(|p| w.u.1 - w.u.0 >= p - 1e-9),
            ctx.b.period_v().is_some_and(|p| w.v.1 - w.v.0 >= p - 1e-9),
        ),
    };
    let w = field.map_or(w, |f| f.window());
    let ranges = ((w.u.1 - w.u.0).abs(), (w.v.1 - w.v.0).abs());
    // nothing to cut when the window cannot exclude anything
    if field.is_none()
        && (per_u || !w.u.0.is_finite() || !w.u.1.is_finite())
        && (per_v || !w.v.0.is_finite() || !w.v.1.is_finite())
    {
        return polys;
    }
    let tol_on = 2e-11 * ctx.scale;
    let eps_u = 1e-9 * (1.0 + ranges.0.min(1e6));
    let eps_v = 1e-9 * (1.0 + ranges.1.min(1e6));
    let on_bound = |x: f64, b: Option<(f64, f64)>| {
        b.is_some_and(|(lo, hi)| {
            (x - lo).abs() <= 1e-14 * (1.0 + lo.abs()) || (x - hi).abs() <= 1e-14 * (1.0 + hi.abs())
        })
    };
    let classify = |uv: (f64, f64)| -> (Side, (f64, f64)) {
        // a vertex on the boundary of A's window (an open contour's end) stays on it
        let Some(q) = ctx.project_held(
            uv.0,
            uv.1,
            cell,
            on_bound(uv.0, ctx.bound_u),
            on_bound(uv.1, ctx.bound_v),
        ) else {
            return (Side::Out, uv);
        };
        let x = ctx.a.point_at(q.0, q.1);
        match field {
            Some(f) => {
                let (fu, fv) = f.foot(&x);
                let dist = (f.surface().point_at(fu, fv) - x).norm();
                if dist <= tol_on {
                    (Side::In { uvb: (fu, fv) }, q)
                } else {
                    (Side::Out, q)
                }
            }
            None => {
                let pb = surface_param_of(ctx.b, &x, Some(ctx.wb_center));
                let inside = |val: f64, lo: f64, hi: f64, per: Option<f64>, eps: f64| -> bool {
                    if !lo.is_finite() || !hi.is_finite() {
                        return true;
                    }
                    match per {
                        Some(p) => {
                            let r = (val - lo).rem_euclid(p);
                            r <= hi - lo + eps || r >= p - eps
                        }
                        None => val >= lo - eps && val <= hi + eps,
                    }
                };
                let pu = ctx.b.period_u().filter(|_| !per_u);
                let pv = ctx.b.period_v().filter(|_| !per_v);
                let ok = (per_u || inside(pb.u, w.u.0, w.u.1, pu, eps_u))
                    && (per_v || inside(pb.v, w.v.0, w.v.1, pv, eps_v));
                if ok {
                    (Side::In { uvb: (pb.u, pb.v) }, q)
                } else {
                    (Side::Out, q)
                }
            }
        }
    };
    // two inside vertices straddle a seam when the footpoint of a distance field jumps across the window
    let seam = |a: (f64, f64), b: (f64, f64)| -> bool {
        field.is_some()
            && ((!per_u && (a.0 - b.0).abs() > 0.5 * ranges.0)
                || (!per_v && (a.1 - b.1).abs() > 0.5 * ranges.1))
    };
    let mut out: Vec<Polyline> = Vec::new();
    for poly in polys {
        let n = poly.uv.len();
        if n < 2 {
            continue;
        }
        let mut cls: Vec<(Side, (f64, f64))> = poly.uv.iter().map(|p| classify(*p)).collect();
        let mut pts: Vec<(f64, f64)> = cls.iter().map(|c| c.1).collect();
        let closed = poly.closed_3d;
        let breaks = |i: usize, cls: &Vec<(Side, (f64, f64))>| -> bool {
            match (cls[i].0, cls[i + 1].0) {
                (Side::In { uvb: a }, Side::In { uvb: b }) => seam(a, b),
                (Side::Out, Side::Out) => false,
                _ => true,
            }
        };
        let any_break = (0..n - 1).any(|i| breaks(i, &cls));
        if !any_break {
            if cls.iter().all(|c| matches!(c.0, Side::In { .. })) {
                out.push(Polyline {
                    uv: pts,
                    closed_3d: closed,
                });
            }
            continue;
        }
        // rotate a closed contour so that it starts at a break (runs never wrap)
        if closed {
            let start = (0..n - 1).find(|i| breaks(*i, &cls)).unwrap() + 1;
            let planar =
                (pts[0].0 - pts[n - 1].0).hypot(pts[0].1 - pts[n - 1].1) < 1e-9 * (1.0 + cell);
            if start < n - 1 && planar {
                let mut rc = Vec::with_capacity(n);
                let mut rp = Vec::with_capacity(n);
                for k in 0..n - 1 {
                    rc.push(cls[(start + k) % (n - 1)]);
                    rp.push(pts[(start + k) % (n - 1)]);
                }
                rc.push(rc[0]);
                rp.push(rp[0]);
                cls = rc;
                pts = rp;
            }
        }
        let mut run: Vec<(f64, f64)> = Vec::new();
        let flush = |run: &mut Vec<(f64, f64)>, out: &mut Vec<Polyline>| {
            if run.len() >= 2 {
                out.push(Polyline {
                    uv: std::mem::take(run),
                    closed_3d: false,
                });
            } else {
                run.clear();
            }
        };
        for i in 0..n {
            if let Side::In { .. } = cls[i].0 {
                run.push(pts[i]);
            }
            if i + 1 == n {
                break;
            }
            if !breaks(i, &cls) {
                continue;
            }
            match (cls[i].0, cls[i + 1].0) {
                (Side::In { .. }, Side::Out) => {
                    // leaving the patch: bisect for the last inside point
                    let (mut lo, mut hi) = (pts[i], pts[i + 1]);
                    for _ in 0..50 {
                        let mid = (0.5 * (lo.0 + hi.0), 0.5 * (lo.1 + hi.1));
                        let (s, q) = classify(mid);
                        match s {
                            Side::In { .. } => lo = q,
                            Side::Out => hi = mid,
                        }
                    }
                    run.push(lo);
                    flush(&mut run, &mut out);
                }
                (Side::Out, Side::In { .. }) => {
                    let (mut lo, mut hi) = (pts[i], pts[i + 1]);
                    for _ in 0..50 {
                        let mid = (0.5 * (lo.0 + hi.0), 0.5 * (lo.1 + hi.1));
                        let (s, q) = classify(mid);
                        match s {
                            Side::In { .. } => hi = q,
                            Side::Out => lo = mid,
                        }
                    }
                    flush(&mut run, &mut out);
                    run.push(hi);
                }
                (Side::In { uvb: a }, Side::In { .. }) => {
                    // seam: find where the footpoint flips
                    let (mut lo, mut hi) = (pts[i], pts[i + 1]);
                    for _ in 0..50 {
                        let mid = (0.5 * (lo.0 + hi.0), 0.5 * (lo.1 + hi.1));
                        let (s, q) = classify(mid);
                        match s {
                            Side::In { uvb } => {
                                if seam(uvb, a) {
                                    hi = q
                                } else {
                                    lo = q
                                }
                            }
                            Side::Out => break,
                        }
                    }
                    run.push(lo);
                    flush(&mut run, &mut out);
                    run.push(hi);
                }
                (Side::Out, Side::Out) => {}
            }
        }
        flush(&mut run, &mut out);
    }
    out
}

fn refine_polyline(
    ctx: &Ctx,
    poly: &Polyline,
    cell: f64,
    tol: f64,
) -> Result<Option<Traced>, SsiError> {
    // 1. project vertices onto g = 0 and build nodes sequentially
    let mut uvs: Vec<(f64, f64)> = Vec::with_capacity(poly.uv.len());
    for (k, p) in poly.uv.iter().enumerate() {
        let is_closing = poly.closed_3d && k + 1 == poly.uv.len();
        // the ends of an open contour on the window boundary stay on it
        let end = !poly.closed_3d && (k == 0 || k + 1 == poly.uv.len());
        let on_bound = |x: f64, b: Option<(f64, f64)>| {
            b.is_some_and(|(lo, hi)| {
                (x - lo).abs() <= 1e-14 * (1.0 + lo.abs())
                    || (x - hi).abs() <= 1e-14 * (1.0 + hi.abs())
            })
        };
        let (hold_u, hold_v) = (
            end && on_bound(p.0, ctx.bound_u),
            end && on_bound(p.1, ctx.bound_v),
        );
        let q = ctx
            .project_held(p.0, p.1, cell, hold_u, hold_v)
            .ok_or(SsiError::Failed(
                "singular or tangent intersection while tracing",
            ))?;
        // keep the projected vertex near its grid location
        if (q.0 - p.0).hypot(q.1 - p.1) > 2.0 * cell {
            return Err(SsiError::Failed(
                "contour vertex projected too far (tangent contact?)",
            ));
        }
        let _ = is_closing;
        uvs.push(q);
    }
    // closing vertex of a wrapped loop is the first one shifted by whole periods
    if poly.closed_3d {
        let f = uvs[0];
        let l = uvs.len() - 1;
        let shift = (uvs[l].0 - f.0, uvs[l].1 - f.1);
        // snap: the last vertex equals the first plus the (rounded) period shift
        let pu = ctx.a.period_u().unwrap_or(1.0);
        let pv = ctx.a.period_v().unwrap_or(1.0);
        let ku = (shift.0 / pu).round();
        let kv = (shift.1 / pv).round();
        uvs[l] = (
            f.0 + ku * if ctx.a.period_u().is_some() { pu } else { 0.0 },
            f.1 + kv * if ctx.a.period_v().is_some() { pv } else { 0.0 },
        );
    }
    // drop repeated vertices
    let mut keep: Vec<(f64, f64)> = vec![uvs[0]];
    for p in &uvs[1..] {
        let l = keep.last().unwrap();
        let far3d =
            (ctx.a.point_at(p.0, p.1) - ctx.a.point_at(l.0, l.1)).norm() > 1e-11 * ctx.scale;
        if (p.0 - l.0).hypot(p.1 - l.1) > 1e-12 * cell && far3d {
            keep.push(*p);
        }
    }
    if keep.len() < 2 {
        return Ok(None);
    }
    let m = keep.len();
    let mut nodes: Vec<Node> = Vec::with_capacity(m);
    let mut hint_b: Option<(f64, f64)> = None;
    for k in 0..m {
        let dir = if k + 1 < m {
            (keep[k + 1].0 - keep[k].0, keep[k + 1].1 - keep[k].1)
        } else {
            (keep[k].0 - keep[k - 1].0, keep[k].1 - keep[k - 1].1)
        };
        let nd = ctx
            .node_at(keep[k], hint_b, Some(dir))
            .ok_or(SsiError::Failed(
                "degenerate tangent while tracing (singular point)",
            ))?;
        hint_b = Some(nd.uvb);
        nodes.push(nd);
    }
    // 2. adaptive refinement
    let mut guard = 0;
    loop {
        guard += 1;
        if guard > 40 || nodes.len() > 30000 {
            if std::env::var("KW_SSI_DEBUG").is_ok() {
                eprintln!(
                    "refine failed: guard {guard} nodes {} first {:?} last {:?}",
                    nodes.len(),
                    nodes[0].x,
                    nodes[nodes.len() - 1].x
                );
            }
            return Err(SsiError::Failed("curve refinement did not converge"));
        }
        let s = arclength_params(&nodes);
        let mut inserts: Vec<(usize, Node)> = Vec::new();
        for i in 0..nodes.len() - 1 {
            let (a, b) = (&nodes[i], &nodes[i + 1]);
            let h = s[i + 1] - s[i];
            let tau = 0.5 * h;
            let (px, _) = herm(&a.x.coords, &b.x.coords, &a.t, &b.t, h, tau);
            let au = Vector3::new(a.uv.0, a.uv.1, 0.0);
            let bu = Vector3::new(b.uv.0, b.uv.1, 0.0);
            let ad = Vector3::new(a.duv.0, a.duv.1, 0.0);
            let bd = Vector3::new(b.duv.0, b.duv.1, 0.0);
            let (puv, _) = herm(&au, &bu, &ad, &bd, h, tau);
            let ab = Vector3::new(a.uvb.0, a.uvb.1, 0.0);
            let bb = Vector3::new(b.uvb.0, b.uvb.1, 0.0);
            let adb = Vector3::new(a.duvb.0, a.duvb.1, 0.0);
            let bdb = Vector3::new(b.duvb.0, b.duvb.1, 0.0);
            let (pb, _) = herm(&ab, &bb, &adb, &bdb, h, tau);
            let xm = Point3::from(px);
            // errors: curve to surface B (implicit), curve to surface A, pcurve A, pcurve B
            let e_b = ctx.imp.eval(&xm).0.abs();
            let e_pa = (ctx.a.point_at(puv.x, puv.y) - xm).norm();
            let e_pb = (ctx.b.point_at(pb.x, pb.y) - xm).norm();
            // |A(uv_hermite) − x| already measures the distance to the parametric surface
            let err = e_b.max(e_pa).max(e_pb);
            if err > tol || !err.is_finite() {
                let q = ctx
                    .project(puv.x, puv.y, cell)
                    .ok_or(SsiError::Failed("projection failed during refinement"))?;
                let dir = (b.uv.0 - a.uv.0, b.uv.1 - a.uv.1);
                let nd = ctx
                    .node_at(q, Some((pb.x, pb.y)), Some(dir))
                    .ok_or(SsiError::Failed("degenerate tangent during refinement"))?;
                inserts.push((i + 1, nd));
            }
        }
        if inserts.is_empty() {
            let curve = hermite_curve(
                &s,
                &nodes.iter().map(|n| n.x).collect::<Vec<_>>(),
                &nodes.iter().map(|n| n.t).collect::<Vec<_>>(),
            );
            let pa = hermite_curve(
                &s,
                &nodes
                    .iter()
                    .map(|n| Point3::new(n.uv.0, n.uv.1, 0.0))
                    .collect::<Vec<_>>(),
                &nodes
                    .iter()
                    .map(|n| Vector3::new(n.duv.0, n.duv.1, 0.0))
                    .collect::<Vec<_>>(),
            );
            let pb = hermite_curve(
                &s,
                &nodes
                    .iter()
                    .map(|n| Point3::new(n.uvb.0, n.uvb.1, 0.0))
                    .collect::<Vec<_>>(),
                &nodes
                    .iter()
                    .map(|n| Vector3::new(n.duvb.0, n.duvb.1, 0.0))
                    .collect::<Vec<_>>(),
            );
            let (Some(curve), Some(pcurve_a), Some(pcurve_b)) = (curve, pa, pb) else {
                return Err(SsiError::Failed("could not build spline"));
            };
            let closed = (nodes[0].x - nodes[nodes.len() - 1].x).norm() < 1e-9 * ctx.scale;
            let range = (s[0], *s.last().unwrap());
            return Ok(Some(Traced {
                curve,
                pcurve_a,
                pcurve_b,
                range,
                closed,
            }));
        }
        for (k, (idx, nd)) in inserts.into_iter().enumerate() {
            nodes.insert(idx + k, nd);
        }
    }
}

#[allow(dead_code)]
fn _unused(_: Vector2<f64>, _: f64) -> f64 {
    wrap_near(0.0, 0.0, 1.0)
}
