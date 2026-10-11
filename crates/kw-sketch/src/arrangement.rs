//! The planar arrangement of a sketch: every curve is cut where it crosses or touches another one, and the
//! pieces form a planar graph whose faces are the regions of the sketch (as in Fusion 360 and Onshape).
//!
//! * Lines, arcs and circles are intersected in closed form; ellipses and splines are intersected by
//!   sampling and refined with Newton's method, so every crossing is a vertex shared by both curves.
//! * Dangling pieces and bridges (a piece with the same face on both sides) are dropped: a free-standing open
//!   curve is simply not part of any region.
//! * A face lies to the left of its boundary half-edges. Components that lie inside a face of another component
//!   become holes of that face; [`Face::depth`] counts how many faces enclose a face (0 = outermost).
//! * [`Arrangement::merge_faces`] gives the outline of any set of faces (the shared walls between selected
//!   neighbours vanish), which is what extruding several regions at once needs.

use crate::entity::{EntityId, EntityKind};
use crate::geom;
use crate::sketch::Sketch;
use kw_nurbs::NurbsCurve;
use std::collections::{BTreeSet, HashMap};
use std::f64::consts::{PI, TAU};

pub type P2 = (f64, f64);

// ---------------------------------------------------------------------------------------- curves

/// A curve with the parametrisation the kernel will use for it.
#[derive(Debug, Clone)]
pub enum Prim {
    /// `a + t (b - a)`, `t` in `[0, 1]`.
    Line { a: P2, b: P2 },
    /// Counter-clockwise from angle `a0` to `a1 > a0`.
    Arc { c: P2, r: f64, a0: f64, a1: f64 },
    /// `t` is the angle, a full turn is `[0, 2π)`.
    Circle { c: P2, r: f64 },
    /// `c + u·major·cos t + v·minor·sin t` with `v` = `u` turned a quarter counter-clockwise.
    Ellipse {
        c: P2,
        u: P2,
        major: f64,
        minor: f64,
    },
    /// A B-spline in the sketch plane (`z = 0`); `t` is the knot parameter.
    Spline(NurbsCurve),
}

fn sub(a: P2, b: P2) -> P2 {
    (a.0 - b.0, a.1 - b.1)
}
fn dist(a: P2, b: P2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

impl Prim {
    pub fn domain(&self) -> (f64, f64) {
        match self {
            Prim::Line { .. } => (0.0, 1.0),
            Prim::Arc { a0, a1, .. } => (*a0, *a1),
            Prim::Circle { .. } | Prim::Ellipse { .. } => (0.0, TAU),
            Prim::Spline(c) => c.domain(),
        }
    }

    /// The period of a closed analytic curve.
    pub fn period(&self) -> Option<f64> {
        matches!(self, Prim::Circle { .. } | Prim::Ellipse { .. }).then_some(TAU)
    }

    pub fn point(&self, t: f64) -> P2 {
        match self {
            Prim::Line { a, b } => (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t),
            Prim::Arc { c, r, .. } | Prim::Circle { c, r } => {
                (c.0 + r * t.cos(), c.1 + r * t.sin())
            }
            Prim::Ellipse { c, u, major, minor } => {
                let (ct, st) = (t.cos(), t.sin());
                (
                    c.0 + u.0 * major * ct - u.1 * minor * st,
                    c.1 + u.1 * major * ct + u.0 * minor * st,
                )
            }
            Prim::Spline(s) => {
                let p = s.point_at(t);
                (p.x, p.y)
            }
        }
    }

    /// First and second derivative with respect to the parameter.
    pub fn deriv(&self, t: f64) -> (P2, P2) {
        match self {
            Prim::Line { a, b } => (sub(*b, *a), (0.0, 0.0)),
            Prim::Arc { r, .. } | Prim::Circle { r, .. } => {
                ((-r * t.sin(), r * t.cos()), (-r * t.cos(), -r * t.sin()))
            }
            Prim::Ellipse {
                u, major, minor, ..
            } => {
                let (ct, st) = (t.cos(), t.sin());
                (
                    (
                        -u.0 * major * st - u.1 * minor * ct,
                        -u.1 * major * st + u.0 * minor * ct,
                    ),
                    (
                        -u.0 * major * ct + u.1 * minor * st,
                        -u.1 * major * ct - u.0 * minor * st,
                    ),
                )
            }
            Prim::Spline(s) => {
                let (lo, hi) = s.domain();
                let ders = s.derivatives(t.clamp(lo, hi), 2);
                let d1 = ders.get(1).copied().unwrap_or_else(kw_math::Vector3::zeros);
                let d2 = ders.get(2).copied().unwrap_or_else(kw_math::Vector3::zeros);
                ((d1.x, d1.y), (d2.x, d2.y))
            }
        }
    }

    /// Signed curvature (positive when the curve turns to the left as the parameter grows).
    pub fn curvature(&self, t: f64) -> f64 {
        let (d1, d2) = self.deriv(t);
        let s = d1.0.hypot(d1.1);
        if s < 1e-300 {
            return 0.0;
        }
        (d1.0 * d2.1 - d1.1 * d2.0) / (s * s * s)
    }

    /// Is the curve closed (a full circle or ellipse, or a spline that ends where it starts)?
    pub fn is_closed(&self) -> bool {
        match self {
            Prim::Circle { .. } | Prim::Ellipse { .. } => true,
            Prim::Spline(s) => {
                let (lo, hi) = s.domain();
                dist(self.point(lo), self.point(hi)) < 1e-9 && s.degree() >= 1
            }
            _ => false,
        }
    }

    fn samples(&self, t0: f64, t1: f64, n: usize) -> Vec<(f64, P2)> {
        (0..=n)
            .map(|i| {
                let t = t0 + (t1 - t0) * i as f64 / n as f64;
                (t, self.point(t))
            })
            .collect()
    }

    /// The piece `[t0, t1]` as a polyline with chord error about `tol`.
    pub fn polyline(&self, t0: f64, t1: f64, tol: f64) -> Vec<P2> {
        let n = match self {
            Prim::Line { .. } => 1,
            Prim::Arc { r, .. } | Prim::Circle { r, .. } => {
                let step = if tol >= *r {
                    PI / 2.0
                } else {
                    2.0 * (1.0 - tol / r).acos()
                };
                (((t1 - t0).abs() / step).ceil() as usize).clamp(2, 4096)
            }
            Prim::Ellipse { major, minor, .. } => {
                let r = major.max(*minor);
                let step = if tol >= r {
                    PI / 2.0
                } else {
                    2.0 * (1.0 - tol / r).acos()
                };
                (((t1 - t0).abs() / step).ceil() as usize).clamp(8, 4096)
            }
            Prim::Spline(s) => 8 * s.points().len().max(4),
        };
        self.samples(t0, t1, n)
            .into_iter()
            .map(|(_, p)| p)
            .collect()
    }

    /// `½ ∫ (x dy − y dx)` over `[t0, t1]` (the signed area swept against the origin).
    pub fn area_integral(&self, t0: f64, t1: f64) -> f64 {
        if let Prim::Line { a, b } = self {
            let (pa, pb) = (self.point(t0), self.point(t1));
            let _ = (a, b);
            return 0.5 * (pa.0 * pb.1 - pb.0 * pa.1);
        }
        // 16 panels of 6-point Gauss-Legendre: exact to rounding for circles and ellipses, ~1e-12 for splines
        const X: [f64; 3] = [
            0.238_619_186_083_196_9,
            0.661_209_386_466_264_5,
            0.932_469_514_203_152,
        ];
        const W: [f64; 3] = [
            0.467_913_934_572_691,
            0.360_761_573_048_138_6,
            0.171_324_492_379_170_3,
        ];
        // a spline is integrated knot span by knot span (it is only C² across a knot)
        let mut cuts = vec![t0, t1];
        if let Prim::Spline(c) = self {
            cuts.extend(
                c.knots()
                    .distinct()
                    .into_iter()
                    .map(|(k, _)| k)
                    .filter(|k| *k > t0 + 1e-12 && *k < t1 - 1e-12),
            );
            cuts.sort_by(f64::total_cmp);
        }
        let mut total = 0.0;
        for w in cuts.windows(2) {
            let panels = 16;
            for k in 0..panels {
                let a = w[0] + (w[1] - w[0]) * k as f64 / panels as f64;
                let b = w[0] + (w[1] - w[0]) * (k + 1) as f64 / panels as f64;
                let (m, h) = (0.5 * (a + b), 0.5 * (b - a));
                for (x, wt) in X.iter().zip(W.iter()) {
                    for sgn in [-1.0, 1.0] {
                        let t = m + sgn * x * h;
                        let (p, (dx, dy)) = (self.point(t), self.deriv(t).0);
                        total += wt * h * 0.5 * (p.0 * dy - p.1 * dx);
                    }
                }
            }
        }
        total
    }

    fn bbox(&self) -> (P2, P2) {
        let (lo, hi) = self.domain();
        let pts = self.samples(lo, hi, 96);
        let mut b = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
        for (_, p) in pts {
            b.0 = (b.0 .0.min(p.0), b.0 .1.min(p.1));
            b.1 = (b.1 .0.max(p.0), b.1 .1.max(p.1));
        }
        b
    }

    /// The parameter of `p` if it lies on the curve (within `tol`), and its distance.
    fn project(&self, p: P2, tol: f64) -> Option<(f64, f64)> {
        match self {
            Prim::Line { a, b } => {
                let d = sub(*b, *a);
                let l2 = d.0 * d.0 + d.1 * d.1;
                if l2 < 1e-300 {
                    return None;
                }
                let t = (((p.0 - a.0) * d.0 + (p.1 - a.1) * d.1) / l2).clamp(0.0, 1.0);
                let q = self.point(t);
                (dist(p, q) <= tol).then_some((t, dist(p, q)))
            }
            Prim::Arc { c, r, .. } | Prim::Circle { c, r } => {
                let dd = (dist(p, *c) - r).abs();
                if dd > tol {
                    return None;
                }
                let t = self.angle_param(p)?;
                Some((t, dd))
            }
            _ => {
                let (lo, hi) = self.domain();
                let s = self.samples(lo, hi, 128);
                let (mut bt, mut bd) = (lo, f64::MAX);
                for (t, q) in &s {
                    let d = dist(p, *q);
                    if d < bd {
                        (bt, bd) = (*t, d);
                    }
                }
                if bd > tol + self.coarse_step(lo, hi) {
                    return None;
                }
                // Newton on the squared distance
                let mut t = bt;
                for _ in 0..40 {
                    let (q, (d1, d2)) = (self.point(t), self.deriv(t));
                    let e = sub(q, p);
                    let g = e.0 * d1.0 + e.1 * d1.1;
                    let h = d1.0 * d1.0 + d1.1 * d1.1 + e.0 * d2.0 + e.1 * d2.1;
                    if h.abs() < 1e-300 {
                        break;
                    }
                    let nt = t - g / h;
                    let nt = match self.period() {
                        Some(_) => nt,
                        None => nt.clamp(lo, hi),
                    };
                    if (nt - t).abs() < 1e-15 {
                        t = nt;
                        break;
                    }
                    t = nt;
                }
                let d = dist(p, self.point(t));
                (d <= tol).then_some((self.normalize_param(t), d))
            }
        }
    }

    fn coarse_step(&self, lo: f64, hi: f64) -> f64 {
        let s = self.samples(lo, hi, 128);
        s.windows(2)
            .map(|w| dist(w[0].1, w[1].1))
            .fold(0.0, f64::max)
    }

    fn normalize_param(&self, t: f64) -> f64 {
        match self.period() {
            Some(p) => t.rem_euclid(p),
            None => t,
        }
    }

    /// The angle parameter of a point of a circle or arc (`None` when it lies outside the arc).
    fn angle_param(&self, p: P2) -> Option<f64> {
        match self {
            Prim::Circle { c, .. } => Some((p.1 - c.1).atan2(p.0 - c.0).rem_euclid(TAU)),
            Prim::Arc { c, r, a0, a1 } => {
                let th = (p.1 - c.1).atan2(p.0 - c.0);
                let t = a0 + (th - a0).rem_euclid(TAU);
                let eps = 1e-9 / r.max(1e-12);
                if t <= a1 + eps {
                    Some(t.min(*a1))
                } else if t - TAU >= a0 - eps {
                    Some(*a0)
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}

// ----------------------------------------------------------------------------------- intersections

fn line_line(a: &Prim, b: &Prim) -> Vec<(f64, f64)> {
    let (Prim::Line { a: a1, b: a2 }, Prim::Line { a: b1, b: b2 }) = (a, b) else {
        return Vec::new();
    };
    let (d1, d2) = (sub(*a2, *a1), sub(*b2, *b1));
    let det = d1.0 * d2.1 - d1.1 * d2.0;
    let scale = d1.0.hypot(d1.1) * d2.0.hypot(d2.1);
    if det.abs() <= 1e-12 * scale.max(1e-300) {
        return Vec::new();
    }
    let w = sub(*b1, *a1);
    let (s, t) = (
        (w.0 * d2.1 - w.1 * d2.0) / det,
        (w.0 * d1.1 - w.1 * d1.0) / det,
    );
    let eps = 1e-9;
    if (-eps..=1.0 + eps).contains(&s) && (-eps..=1.0 + eps).contains(&t) {
        vec![(s.clamp(0.0, 1.0), t.clamp(0.0, 1.0))]
    } else {
        Vec::new()
    }
}

fn line_circle(line: &Prim, circ: &Prim) -> Vec<(f64, f64)> {
    let (Prim::Line { a: p1, b: p2 }, Prim::Arc { c, r, .. } | Prim::Circle { c, r }) =
        (line, circ)
    else {
        return Vec::new();
    };
    let d = sub(*p2, *p1);
    let f = sub(*p1, *c);
    let (qa, qb, qc) = (
        d.0 * d.0 + d.1 * d.1,
        2.0 * (f.0 * d.0 + f.1 * d.1),
        f.0 * f.0 + f.1 * f.1 - r * r,
    );
    if qa < 1e-300 {
        return Vec::new();
    }
    let mut disc = qb * qb - 4.0 * qa * qc;
    if disc < 0.0 {
        if disc > -1e-10 * (qb * qb + 4.0 * qa * r * r) {
            disc = 0.0;
        } else {
            return Vec::new();
        }
    }
    let mut out = Vec::new();
    let roots: Vec<f64> = if disc == 0.0 {
        vec![-qb / (2.0 * qa)]
    } else {
        vec![
            (-qb - disc.sqrt()) / (2.0 * qa),
            (-qb + disc.sqrt()) / (2.0 * qa),
        ]
    };
    for t in roots {
        if !(-1e-9..=1.0 + 1e-9).contains(&t) {
            continue;
        }
        let t = t.clamp(0.0, 1.0);
        let q = line.point(t);
        if let Some(u) = circ.angle_param(q) {
            out.push((t, u));
        }
    }
    out
}

fn circle_circle(a: &Prim, b: &Prim) -> Vec<(f64, f64)> {
    let (
        Prim::Arc { c: c1, r: r1, .. } | Prim::Circle { c: c1, r: r1 },
        Prim::Arc { c: c2, r: r2, .. } | Prim::Circle { c: c2, r: r2 },
    ) = (a, b)
    else {
        return Vec::new();
    };
    let (dx, dy) = (c2.0 - c1.0, c2.1 - c1.1);
    let d = dx.hypot(dy);
    let eps = 1e-9 * (r1 + r2).max(1e-12);
    if d < 1e-12 || d > r1 + r2 + eps || d < (r1 - r2).abs() - eps {
        return Vec::new();
    }
    let al = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h = (r1 * r1 - al * al).max(0.0).sqrt();
    let (mx, my) = (c1.0 + al * dx / d, c1.1 + al * dy / d);
    let mut out = Vec::new();
    let signs: &[f64] = if h < 1e-9 * (r1 + r2) {
        &[0.0]
    } else {
        &[-1.0, 1.0]
    };
    for s in signs {
        let q = (mx - s * h * dy / d, my + s * h * dx / d);
        if let (Some(ta), Some(tb)) = (a.angle_param(q), b.angle_param(q)) {
            out.push((ta, tb));
        }
    }
    out
}

/// Intersections of two curves found by sampling and refined with Newton's method.
fn generic(a: &Prim, b: &Prim) -> Vec<(f64, f64)> {
    let (ba, bb) = (a.bbox(), b.bbox());
    let pad = 1e-9;
    if ba.1 .0 < bb.0 .0 - pad
        || bb.1 .0 < ba.0 .0 - pad
        || ba.1 .1 < bb.0 .1 - pad
        || bb.1 .1 < ba.0 .1 - pad
    {
        return Vec::new();
    }
    let (da, db) = (a.domain(), b.domain());
    let n = 128;
    let (sa, sb) = (a.samples(da.0, da.1, n), b.samples(db.0, db.1, n));
    let mut found: Vec<(f64, f64)> = Vec::new();
    for wa in sa.windows(2) {
        let (ax0, ax1) = (wa[0].1 .0.min(wa[1].1 .0), wa[0].1 .0.max(wa[1].1 .0));
        let (ay0, ay1) = (wa[0].1 .1.min(wa[1].1 .1), wa[0].1 .1.max(wa[1].1 .1));
        for wb in sb.windows(2) {
            if wb[0].1 .0.max(wb[1].1 .0) < ax0 - pad
                || wb[0].1 .0.min(wb[1].1 .0) > ax1 + pad
                || wb[0].1 .1.max(wb[1].1 .1) < ay0 - pad
                || wb[0].1 .1.min(wb[1].1 .1) > ay1 + pad
            {
                continue;
            }
            let (d1, d2) = (sub(wa[1].1, wa[0].1), sub(wb[1].1, wb[0].1));
            let det = d1.0 * d2.1 - d1.1 * d2.0;
            if det.abs() < 1e-300 {
                continue;
            }
            let w = sub(wb[0].1, wa[0].1);
            let (s, t) = (
                (w.0 * d2.1 - w.1 * d2.0) / det,
                (w.0 * d1.1 - w.1 * d1.0) / det,
            );
            if !(-1e-6..=1.0 + 1e-6).contains(&s) || !(-1e-6..=1.0 + 1e-6).contains(&t) {
                continue;
            }
            let mut ta = wa[0].0 + (wa[1].0 - wa[0].0) * s;
            let mut tb = wb[0].0 + (wb[1].0 - wb[0].0) * t;
            // Newton on P_a(ta) - P_b(tb)
            let mut ok = false;
            for _ in 0..40 {
                let (pa, (ua, _)) = (a.point(ta), a.deriv(ta));
                let (pb, (ub, _)) = (b.point(tb), b.deriv(tb));
                let f = sub(pa, pb);
                if f.0.hypot(f.1) < 1e-13 {
                    ok = true;
                    break;
                }
                // [ua  -ub] [dta dtb]^T = -f
                let det = -ua.0 * ub.1 + ua.1 * ub.0;
                if det.abs() < 1e-300 {
                    break;
                }
                let (rx, ry) = (-f.0, -f.1);
                let dta = (-rx * ub.1 + ry * ub.0) / det;
                let dtb = (ua.0 * ry - ua.1 * rx) / det;
                ta += dta;
                tb += dtb;
                if a.period().is_none() {
                    ta = ta.clamp(da.0, da.1);
                }
                if b.period().is_none() {
                    tb = tb.clamp(db.0, db.1);
                }
            }
            let f = sub(a.point(ta), b.point(tb));
            if !ok && f.0.hypot(f.1) > 1e-9 {
                continue;
            }
            let (ta, tb) = (a.normalize_param(ta), b.normalize_param(tb));
            let p = a.point(ta);
            if found.iter().all(|(x, _)| dist(a.point(*x), p) > 1e-7) {
                found.push((ta, tb));
            }
        }
    }
    found
}

/// Intersections `(t_a, t_b)` of two curves, including end points that lie on the other curve.
fn intersect(a: &Prim, b: &Prim, tol: f64) -> Vec<(f64, f64)> {
    let mut out = match (a, b) {
        (Prim::Line { .. }, Prim::Line { .. }) => line_line(a, b),
        (Prim::Line { .. }, Prim::Arc { .. } | Prim::Circle { .. }) => line_circle(a, b),
        (Prim::Arc { .. } | Prim::Circle { .. }, Prim::Line { .. }) => {
            line_circle(b, a).into_iter().map(|(t, u)| (u, t)).collect()
        }
        (Prim::Arc { .. } | Prim::Circle { .. }, Prim::Arc { .. } | Prim::Circle { .. }) => {
            circle_circle(a, b)
        }
        _ => generic(a, b),
    };
    // end points lying on the other curve (T junctions, overlapping collinear / co-circular pieces)
    for (p, ta) in endpoints(a) {
        if let Some((tb, _)) = b.project(p, tol) {
            out.push((ta, tb));
        }
    }
    for (p, tb) in endpoints(b) {
        if let Some((ta, _)) = a.project(p, tol) {
            out.push((ta, tb));
        }
    }
    out
}

fn endpoints(p: &Prim) -> Vec<(P2, f64)> {
    if p.is_closed() {
        return Vec::new();
    }
    let (lo, hi) = p.domain();
    vec![(p.point(lo), lo), (p.point(hi), hi)]
}

// -------------------------------------------------------------------------------------- the graph

#[derive(Debug, Clone)]
pub struct Curve {
    pub entity: EntityId,
    pub prim: Prim,
}

/// A piece of a curve between two vertices; it runs from `from` to `to` as the parameter grows from `t0` to
/// `t1` (for a piece of a periodic curve that wraps past the seam, `t1` may exceed the period).
#[derive(Debug, Clone)]
pub struct Edge {
    pub curve: usize,
    pub t0: f64,
    pub t1: f64,
    pub from: usize,
    pub to: usize,
    /// Every sketch entity this piece belongs to (several when overlapping pieces were merged).
    pub sources: Vec<EntityId>,
}

/// One side of an edge: `forward` runs `from → to`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Half {
    pub edge: usize,
    pub forward: bool,
}

impl Half {
    fn id(self) -> usize {
        self.edge * 2 + usize::from(!self.forward)
    }
    fn from_id(i: usize) -> Half {
        Half {
            edge: i / 2,
            forward: i.is_multiple_of(2),
        }
    }
    fn twin(self) -> Half {
        Half {
            edge: self.edge,
            forward: !self.forward,
        }
    }
}

/// A region of the sketch: its outline (counter-clockwise, face on the left) and the outlines of the holes
/// (clockwise).
#[derive(Debug, Clone)]
pub struct Face {
    pub outer: Vec<Half>,
    pub holes: Vec<Vec<Half>>,
    /// How many faces of other shapes enclose this one (0 = not inside anything).
    pub depth: usize,
    /// Area of the outline minus the holes.
    pub area: f64,
    pub sources: BTreeSet<EntityId>,
}

/// The outline of a set of faces: an outer loop with the loops of its holes.
#[derive(Debug, Clone)]
pub struct MergedRegion {
    pub outer: Vec<Half>,
    pub holes: Vec<Vec<Half>>,
}

#[derive(Debug, Clone, Default)]
pub struct Arrangement {
    pub curves: Vec<Curve>,
    pub vertices: Vec<P2>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    alive: Vec<bool>,
    /// Outgoing half-edges of each vertex, counter-clockwise.
    around: Vec<Vec<Half>>,
    scale: f64,
}

struct Builder {
    verts: Vec<P2>,
    tol: f64,
}

impl Builder {
    fn vertex(&mut self, p: P2) -> usize {
        if let Some(i) = self.verts.iter().position(|q| dist(*q, p) <= self.tol) {
            return i;
        }
        self.verts.push(p);
        self.verts.len() - 1
    }
}

fn prim_of(sk: &Sketch, id: EntityId) -> Option<Prim> {
    match sk.kind(id).ok()? {
        EntityKind::Line { p1, p2 } => Some(Prim::Line {
            a: sk.point(*p1).ok()?,
            b: sk.point(*p2).ok()?,
        }),
        EntityKind::Arc { .. } => {
            let (c, r, a0, a1) = geom::arc_params(sk, id).ok()?;
            (r > 1e-12).then_some(Prim::Arc { c, r, a0, a1 })
        }
        EntityKind::Circle { center, radius } => {
            let r = sk.value(*radius);
            (r > 1e-12).then(|| Prim::Circle {
                c: sk.point(*center).unwrap_or((0.0, 0.0)),
                r,
            })
        }
        EntityKind::Ellipse {
            center,
            major_end,
            minor_radius,
        } => {
            let (c, m) = (sk.point(*center).ok()?, sk.point(*major_end).ok()?);
            let major = dist(c, m);
            let minor = sk.value(*minor_radius).abs();
            (major > 1e-12 && minor > 1e-12).then(|| Prim::Ellipse {
                c,
                u: ((m.0 - c.0) / major, (m.1 - c.1) / major),
                major,
                minor,
            })
        }
        EntityKind::Spline { .. } | EntityKind::FitSpline { .. } => {
            geom::spline_curve(sk, id).ok().map(Prim::Spline)
        }
        EntityKind::Point(_) => None,
    }
}

impl Arrangement {
    /// Build the arrangement of the sketch's curves (construction geometry is left out).
    pub fn build(sk: &Sketch) -> Arrangement {
        let curves: Vec<Curve> = sk
            .entities
            .iter()
            .filter(|(_, e)| !e.construction)
            .filter_map(|(id, _)| prim_of(sk, id).map(|prim| Curve { entity: id, prim }))
            .collect();
        Arrangement::from_curves(curves)
    }

    /// The arrangement of ready-made curves.
    pub fn from_curves(curves: Vec<Curve>) -> Arrangement {
        let mut arr = Arrangement {
            curves,
            ..Default::default()
        };
        // size of the sketch, for tolerances
        let mut lo = (f64::MAX, f64::MAX);
        let mut hi = (f64::MIN, f64::MIN);
        for c in &arr.curves {
            let b = c.prim.bbox();
            lo = (lo.0.min(b.0 .0), lo.1.min(b.0 .1));
            hi = (hi.0.max(b.1 .0), hi.1.max(b.1 .1));
        }
        arr.scale = if arr.curves.is_empty() {
            1.0
        } else {
            (hi.0 - lo.0).hypot(hi.1 - lo.1).max(1.0)
        };
        let tol = 1e-8 * arr.scale;
        let mut b = Builder {
            verts: Vec::new(),
            tol,
        };
        let n = arr.curves.len();
        // cuts[i] = (parameter, vertex) pairs of curve i
        let mut cuts: Vec<Vec<(f64, usize)>> = vec![Vec::new(); n];
        let boxes: Vec<(P2, P2)> = arr.curves.iter().map(|c| c.prim.bbox()).collect();
        for i in 0..n {
            for j in i + 1..n {
                let (bi, bj) = (boxes[i], boxes[j]);
                if bi.1 .0 < bj.0 .0 - tol
                    || bj.1 .0 < bi.0 .0 - tol
                    || bi.1 .1 < bj.0 .1 - tol
                    || bj.1 .1 < bi.0 .1 - tol
                {
                    continue;
                }
                for (ta, tb) in intersect(&arr.curves[i].prim, &arr.curves[j].prim, tol * 10.0) {
                    let p = arr.curves[i].prim.point(ta);
                    let v = b.vertex(p);
                    cuts[i].push((ta, v));
                    cuts[j].push((tb, v));
                }
            }
        }
        // end points are vertices; periodic curves get a seam
        let mut pieces: Vec<(usize, f64, f64, usize, usize)> = Vec::new();
        for (i, c) in arr.curves.iter().enumerate() {
            let prim = &c.prim;
            let (lo, hi) = prim.domain();
            let closed = prim.is_closed();
            let periodic = prim.period();
            if !closed {
                for t in [lo, hi] {
                    let v = b.vertex(prim.point(t));
                    cuts[i].push((t, v));
                }
            } else if periodic.is_none() {
                // a closed spline: its seam is a vertex (a piece may not wrap)
                let v = b.vertex(prim.point(lo));
                cuts[i].push((lo, v));
            }
            let cs = &mut cuts[i];
            cs.sort_by(|a, b| a.0.total_cmp(&b.0));
            // one cut per vertex and per parameter
            let mut kept: Vec<(f64, usize)> = Vec::new();
            for (t, v) in cs.iter().copied() {
                if kept
                    .iter()
                    .all(|(kt, kv)| *kv != v && (kt - t).abs() > 1e-12)
                {
                    kept.push((t, v));
                }
            }
            if closed && kept.len() <= 1 {
                // nothing cuts it (or one touching point): a closed edge, or two halves
                if kept.is_empty() {
                    let v = b.vertex(prim.point(lo));
                    kept.push((lo, v));
                } else {
                    let span = hi - lo;
                    let t = if periodic.is_some() {
                        kept[0].0 + PI
                    } else {
                        lo + 0.5 * span
                    };
                    let t = if periodic.is_some() {
                        t.rem_euclid(TAU)
                    } else {
                        t
                    };
                    let v = b.vertex(prim.point(t));
                    kept.push((t, v));
                    kept.sort_by(|a, b| a.0.total_cmp(&b.0));
                }
            }
            if closed {
                match periodic {
                    Some(p) => {
                        for k in 0..kept.len() {
                            let (t0, v0) = kept[k];
                            let (t1, v1) = if k + 1 < kept.len() {
                                kept[k + 1]
                            } else {
                                (kept[0].0 + p, kept[0].1)
                            };
                            pieces.push((i, t0, t1, v0, v1));
                        }
                    }
                    None => {
                        // the seam at `lo` is among the cuts; the last piece ends at `hi`, back at the seam vertex
                        for k in 0..kept.len() {
                            let (t0, v0) = kept[k];
                            let (t1, v1) = if k + 1 < kept.len() {
                                kept[k + 1]
                            } else {
                                (hi, kept[0].1)
                            };
                            pieces.push((i, t0, t1, v0, v1));
                        }
                    }
                }
            } else {
                for w in kept.windows(2) {
                    pieces.push((i, w[0].0, w[1].0, w[0].1, w[1].1));
                }
            }
        }
        arr.vertices = b.verts;
        // edges; overlapping duplicates merge
        for (ci, t0, t1, v0, v1) in pieces {
            if v0 == v1 && !arr.curves[ci].prim.is_closed() {
                continue;
            }
            let prim = &arr.curves[ci].prim;
            let mid = prim.point(0.5 * (t0 + t1));
            let dup = (v0 != v1)
                .then(|| {
                    arr.edges.iter().position(|e| {
                        ((e.from == v0 && e.to == v1) || (e.from == v1 && e.to == v0))
                            && dist(arr.curves[e.curve].prim.point(0.5 * (e.t0 + e.t1)), mid)
                                <= tol * 10.0
                    })
                })
                .flatten();
            match dup {
                Some(k) => {
                    let ent = arr.curves[ci].entity;
                    if !arr.edges[k].sources.contains(&ent) {
                        arr.edges[k].sources.push(ent);
                    }
                }
                None => arr.edges.push(Edge {
                    curve: ci,
                    t0,
                    t1,
                    from: v0,
                    to: v1,
                    sources: vec![arr.curves[ci].entity],
                }),
            }
        }
        arr.alive = vec![true; arr.edges.len()];
        arr.prune();
        arr.link();
        arr.make_faces();
        arr
    }

    // ---------------------------------------------------------------------------------- helpers

    fn origin(&self, h: Half) -> usize {
        if h.forward {
            self.edges[h.edge].from
        } else {
            self.edges[h.edge].to
        }
    }

    fn target(&self, h: Half) -> usize {
        self.origin(h.twin())
    }

    /// Direction and signed curvature of the half-edge where it leaves its origin.
    fn leaving(&self, h: Half) -> (f64, f64) {
        let e = &self.edges[h.edge];
        let prim = &self.curves[e.curve].prim;
        if h.forward {
            let d = prim.deriv(e.t0).0;
            (d.1.atan2(d.0), prim.curvature(e.t0))
        } else {
            let d = prim.deriv(e.t1).0;
            ((-d.1).atan2(-d.0), -prim.curvature(e.t1))
        }
    }

    /// Remove dangling pieces and bridges until none are left.
    fn prune(&mut self) {
        loop {
            let mut changed = false;
            // dangling: a vertex with a single live piece end
            let mut degree = vec![0usize; self.vertices.len()];
            for (i, e) in self.edges.iter().enumerate() {
                if self.alive[i] {
                    degree[e.from] += 1;
                    degree[e.to] += 1;
                }
            }
            for (i, e) in self.edges.iter().enumerate() {
                if self.alive[i] && e.from != e.to && (degree[e.from] == 1 || degree[e.to] == 1) {
                    self.alive[i] = false;
                    changed = true;
                }
            }
            if changed {
                continue;
            }
            // bridges: both sides of a piece lie in one face cycle
            self.link();
            let mut seen = vec![usize::MAX; self.edges.len() * 2];
            let mut cycle = 0usize;
            for i in 0..self.edges.len() * 2 {
                let h = Half::from_id(i);
                if !self.alive[h.edge] || seen[i] != usize::MAX {
                    continue;
                }
                let mut cur = h;
                loop {
                    seen[cur.id()] = cycle;
                    cur = self.next(cur);
                    if cur == h {
                        break;
                    }
                }
                cycle += 1;
            }
            for i in 0..self.edges.len() {
                if self.alive[i] && seen[2 * i] == seen[2 * i + 1] {
                    self.alive[i] = false;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
    }

    /// Sort the outgoing half-edges of every vertex counter-clockwise.
    fn link(&mut self) {
        let mut around: Vec<Vec<Half>> = vec![Vec::new(); self.vertices.len()];
        for (i, _) in self.edges.iter().enumerate() {
            if !self.alive[i] {
                continue;
            }
            for forward in [true, false] {
                let h = Half { edge: i, forward };
                around[self.origin(h)].push(h);
            }
        }
        for list in &mut around {
            let keys: Vec<(Half, f64, f64)> = list
                .iter()
                .map(|h| {
                    let (a, k) = self.leaving(*h);
                    (*h, a, k)
                })
                .collect();
            let mut keys = keys;
            keys.sort_by(|x, y| {
                if (x.1 - y.1).abs() < 1e-9 {
                    x.2.total_cmp(&y.2)
                } else {
                    x.1.total_cmp(&y.1)
                }
            });
            *list = keys.into_iter().map(|k| k.0).collect();
        }
        self.around = around;
    }

    /// The half-edge that follows `h` around its face (the face stays on the left).
    fn next(&self, h: Half) -> Half {
        let v = self.target(h);
        let list = &self.around[v];
        let tw = h.twin();
        let i = list
            .iter()
            .position(|x| *x == tw)
            .expect("twin leaves the target");
        list[(i + list.len() - 1) % list.len()]
    }

    fn half_area(&self, h: Half) -> f64 {
        let e = &self.edges[h.edge];
        let a = self.curves[e.curve].prim.area_integral(e.t0, e.t1);
        if h.forward {
            a
        } else {
            -a
        }
    }

    /// The outline of a loop of half-edges as a polygon.
    pub fn loop_polygon(&self, halves: &[Half], tol: f64) -> Vec<P2> {
        let mut out: Vec<P2> = Vec::new();
        for h in halves {
            let e = &self.edges[h.edge];
            let mut pts = self.curves[e.curve].prim.polyline(e.t0, e.t1, tol);
            if !h.forward {
                pts.reverse();
            }
            pts.pop();
            out.extend(pts);
        }
        out
    }

    /// Signed area of a loop of half-edges (positive = counter-clockwise).
    pub fn loop_area(&self, halves: &[Half]) -> f64 {
        halves.iter().map(|h| self.half_area(*h)).sum()
    }

    fn make_faces(&mut self) {
        let nh = self.edges.len() * 2;
        let mut done = vec![false; nh];
        // cycles of every connected piece of the graph
        let mut cycles: Vec<(Vec<Half>, f64)> = Vec::new();
        for i in 0..nh {
            let h = Half::from_id(i);
            if !self.alive[h.edge] || done[i] {
                continue;
            }
            let mut cyc = Vec::new();
            let mut cur = h;
            loop {
                done[cur.id()] = true;
                cyc.push(cur);
                cur = self.next(cur);
                if cur == h {
                    break;
                }
            }
            let area = self.loop_area(&cyc);
            cycles.push((cyc, area));
        }
        // connected components by shared vertices
        let mut comp: Vec<usize> = (0..self.vertices.len()).collect();
        fn find(c: &mut [usize], x: usize) -> usize {
            let mut r = x;
            while c[r] != r {
                r = c[r];
            }
            let mut y = x;
            while c[y] != r {
                let n = c[y];
                c[y] = r;
                y = n;
            }
            r
        }
        for (i, e) in self.edges.iter().enumerate() {
            if self.alive[i] {
                let (a, b) = (find(&mut comp, e.from), find(&mut comp, e.to));
                comp[a] = b;
            }
        }
        let eps = 1e-12 * self.scale * self.scale;
        // faces (positive cycles) and one outer boundary (the most negative cycle) per component
        struct Comp {
            outer: Option<usize>,
            faces: Vec<usize>,
        }
        let mut comps: HashMap<usize, Comp> = HashMap::new();
        for (ci, (cyc, area)) in cycles.iter().enumerate() {
            let root = find(&mut comp, self.origin(cyc[0]));
            let c = comps.entry(root).or_insert(Comp {
                outer: None,
                faces: Vec::new(),
            });
            if *area > eps {
                c.faces.push(ci);
            } else if c.outer.is_none_or(|o| *area < cycles[o].1) {
                c.outer = Some(ci);
            }
        }
        let tol = 1e-3 * self.scale;
        let polys: Vec<Vec<P2>> = cycles
            .iter()
            .map(|(c, _)| self.loop_polygon(c, tol))
            .collect();
        // components, biggest outer boundary first: a host always comes before what it holds
        let mut order: Vec<usize> = comps.keys().copied().collect();
        order.sort_by(|a, b| {
            let area = |k: &usize| {
                let c = &comps[k];
                c.outer
                    .map(|o| cycles[o].1.abs())
                    .or_else(|| c.faces.first().map(|f| cycles[*f].1))
                    .unwrap_or(0.0)
            };
            area(b).total_cmp(&area(a)).then(a.cmp(b))
        });
        // host face (a cycle index) of every component, and component depth
        let mut host: HashMap<usize, Option<usize>> = HashMap::new();
        let mut depth: HashMap<usize, usize> = HashMap::new();
        let mut face_comp: HashMap<usize, usize> = HashMap::new();
        for (root, c) in &comps {
            for f in &c.faces {
                face_comp.insert(*f, *root);
            }
        }
        for root in &order {
            let c = &comps[root];
            let Some(o) = c.outer else {
                host.insert(*root, None);
                depth.insert(*root, 0);
                continue;
            };
            let p = polys[o][0];
            let mut best: Option<(usize, f64)> = None;
            for (f, fr) in &face_comp {
                if fr == root || !depth.contains_key(fr) {
                    continue;
                }
                if point_in_polygon(&polys[*f], p) {
                    let a = cycles[*f].1;
                    if best.is_none_or(|b| a < b.1) {
                        best = Some((*f, a));
                    }
                }
            }
            match best {
                Some((f, _)) => {
                    host.insert(*root, Some(f));
                    depth.insert(*root, depth[&face_comp[&f]] + 1);
                }
                None => {
                    host.insert(*root, None);
                    depth.insert(*root, 0);
                }
            }
        }
        let mut holes_of: HashMap<usize, Vec<usize>> = HashMap::new();
        for root in &order {
            if let (Some(Some(f)), Some(o)) = (host.get(root), comps[root].outer) {
                holes_of.entry(*f).or_default().push(o);
            }
        }
        let mut faces: Vec<Face> = Vec::new();
        for root in &order {
            let mut fs = comps[root].faces.clone();
            fs.sort_by_key(|f| {
                let h = cycles[*f].0[0];
                (self.edges[h.edge].curve, h.edge, h.forward)
            });
            for f in fs {
                let holes: Vec<Vec<Half>> = holes_of
                    .get(&f)
                    .map(|v| v.iter().map(|o| cycles[*o].0.clone()).collect())
                    .unwrap_or_default();
                let mut sources = BTreeSet::new();
                for h in cycles[f].0.iter().chain(holes.iter().flatten()) {
                    sources.extend(self.edges[h.edge].sources.iter().copied());
                }
                let area = cycles[f].1 + holes.iter().map(|h| self.loop_area(h)).sum::<f64>();
                faces.push(Face {
                    outer: cycles[f].0.clone(),
                    holes,
                    depth: depth[root],
                    area,
                    sources,
                });
            }
        }
        self.faces = faces;
    }

    // --------------------------------------------------------------------------------- queries

    /// A point strictly inside `face` (inside the outline, outside the holes).
    pub fn interior_point(&self, face: &Face) -> P2 {
        let tol = 1e-4 * self.scale;
        let outer = self.loop_polygon(&face.outer, tol);
        let holes: Vec<Vec<P2>> = face
            .holes
            .iter()
            .map(|h| self.loop_polygon(h, tol))
            .collect();
        interior_of(&outer, &holes)
    }

    /// The polygons of a face: its outline first, then its holes.
    pub fn face_polygons(&self, face: &Face, tol: f64) -> Vec<Vec<P2>> {
        let mut v = vec![self.loop_polygon(&face.outer, tol)];
        v.extend(face.holes.iter().map(|h| self.loop_polygon(h, tol)));
        v
    }

    /// Is `p` inside `face`?
    pub fn face_contains(&self, face: &Face, p: P2) -> bool {
        let tol = 1e-4 * self.scale;
        point_in_polygon(&self.loop_polygon(&face.outer, tol), p)
            && !face
                .holes
                .iter()
                .any(|h| point_in_polygon(&self.loop_polygon(h, tol), p))
    }

    /// The outline of the union of the given faces: the half-edges whose other side is not selected, chained
    /// into loops (counter-clockwise loops are outer boundaries, clockwise ones are holes).
    pub fn merge_faces(&self, ids: &[usize]) -> Vec<MergedRegion> {
        let mut selected: BTreeSet<usize> = BTreeSet::new();
        for i in ids {
            let f = &self.faces[*i];
            for h in f.outer.iter().chain(f.holes.iter().flatten()) {
                selected.insert(h.id());
            }
        }
        let boundary: BTreeSet<usize> = selected
            .iter()
            .copied()
            .filter(|h| !selected.contains(&Half::from_id(*h).twin().id()))
            .collect();
        let mut used: BTreeSet<usize> = BTreeSet::new();
        let mut loops: Vec<(Vec<Half>, f64)> = Vec::new();
        for &start in &boundary {
            if used.contains(&start) {
                continue;
            }
            let h0 = Half::from_id(start);
            let mut cyc = vec![h0];
            used.insert(start);
            let mut cur = h0;
            for _ in 0..boundary.len() + 1 {
                // the first boundary half-edge clockwise after the twin around the target vertex
                let v = self.target(cur);
                let list = &self.around[v];
                let tw = cur.twin();
                let i = list.iter().position(|x| *x == tw).expect("twin");
                let mut next = None;
                for k in 1..=list.len() {
                    let cand = list[(i + list.len() - k) % list.len()];
                    if boundary.contains(&cand.id()) && (cand == h0 || !used.contains(&cand.id())) {
                        next = Some(cand);
                        break;
                    }
                }
                match next {
                    Some(n) if n == h0 => break,
                    Some(n) => {
                        used.insert(n.id());
                        cyc.push(n);
                        cur = n;
                    }
                    None => break,
                }
            }
            let a = self.loop_area(&cyc);
            loops.push((cyc, a));
        }
        let tol = 1e-3 * self.scale;
        let polys: Vec<Vec<P2>> = loops
            .iter()
            .map(|(l, _)| self.loop_polygon(l, tol))
            .collect();
        let mut out: Vec<MergedRegion> = Vec::new();
        let mut outer_idx: Vec<usize> = Vec::new();
        for (i, (l, a)) in loops.iter().enumerate() {
            if *a > 0.0 {
                out.push(MergedRegion {
                    outer: l.clone(),
                    holes: Vec::new(),
                });
                outer_idx.push(i);
            }
        }
        for (i, (l, a)) in loops.iter().enumerate() {
            if *a > 0.0 {
                continue;
            }
            let p = polys[i][0];
            let best = outer_idx
                .iter()
                .enumerate()
                .filter(|(_, o)| point_in_polygon(&polys[**o], p))
                .min_by(|x, y| loops[*x.1].1.total_cmp(&loops[*y.1].1))
                .map(|(k, _)| k);
            if let Some(k) = best {
                out[k].holes.push(l.clone());
            }
        }
        out
    }
}

/// Is `p` inside the polygon (even-odd)?
pub fn point_in_polygon(poly: &[P2], p: P2) -> bool {
    let mut inside = false;
    if poly.is_empty() {
        return false;
    }
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// A point inside `outer` and outside every hole: the middle of the widest stretch of a horizontal line.
pub fn interior_of(outer: &[P2], holes: &[Vec<P2>]) -> P2 {
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for p in outer {
        lo = lo.min(p.1);
        hi = hi.max(p.1);
    }
    let mut best: Option<(f64, P2)> = None;
    for k in 1..16 {
        // an irrational-ish fraction so that the line rarely passes through a vertex
        let y = lo + (hi - lo) * (k as f64 + 0.137) / 16.0;
        let mut xs: Vec<f64> = Vec::new();
        for poly in std::iter::once(outer).chain(holes.iter().map(|h| h.as_slice())) {
            let n = poly.len();
            for i in 0..n {
                let (a, b) = (poly[i], poly[(i + 1) % n]);
                if (a.1 > y) != (b.1 > y) {
                    xs.push(a.0 + (b.0 - a.0) * (y - a.1) / (b.1 - a.1));
                }
            }
        }
        xs.sort_by(f64::total_cmp);
        for pair in xs.chunks(2) {
            if let [x0, x1] = pair {
                let w = x1 - x0;
                if best.is_none_or(|b| w > b.0) {
                    best = Some((w, (0.5 * (x0 + x1), y)));
                }
            }
        }
    }
    best.map(|b| b.1).unwrap_or(outer[0])
}
