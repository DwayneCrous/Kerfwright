//! 2-D geometry queries on sketch entities: sampling for display and hit
//! testing, nearest points, and curve–curve intersections.

use crate::entity::{EntityId, EntityKind};
use crate::sketch::{Sketch, SketchError};
use kw_nurbs::{KnotVector, NurbsCurve};
use std::f64::consts::TAU;

pub type P2 = (f64, f64);

/// `(centre, radius, start angle, end angle)` with `end > start` (counter-clockwise).
pub fn arc_params(sk: &Sketch, e: EntityId) -> Result<(P2, f64, f64, f64), SketchError> {
    match sk.kind(e)? {
        EntityKind::Arc { center, start, end } => {
            let c = sk.point(*center)?;
            let (s, t) = (sk.point(*start)?, sk.point(*end)?);
            let r = (s.0 - c.0).hypot(s.1 - c.1);
            let a0 = (s.1 - c.1).atan2(s.0 - c.0);
            let mut a1 = (t.1 - c.1).atan2(t.0 - c.0);
            while a1 <= a0 {
                a1 += TAU;
            }
            Ok((c, r, a0, a1))
        }
        _ => Err(SketchError::WrongEntity("expected an arc")),
    }
}

/// B-spline curve (clamped uniform, non-rational) of a spline entity.
pub fn spline_curve(sk: &Sketch, e: EntityId) -> Result<NurbsCurve, SketchError> {
    match sk.kind(e)? {
        EntityKind::Spline { degree, control } => {
            let pts = control
                .iter()
                .map(|p| sk.point(*p).map(|(x, y)| kw_math::Point3::new(x, y, 0.0)))
                .collect::<Result<Vec<_>, _>>()?;
            let kv = KnotVector::clamped_uniform(*degree, pts.len())
                .map_err(|_| SketchError::Degenerate("bad spline"))?;
            NurbsCurve::new_polynomial(kv, pts).map_err(|_| SketchError::Degenerate("bad spline"))
        }
        EntityKind::FitSpline { points } => {
            let pts = points
                .iter()
                .map(|p| sk.point(*p).map(|(x, y)| kw_math::Point3::new(x, y, 0.0)))
                .collect::<Result<Vec<_>, _>>()?;
            NurbsCurve::interpolate(&pts, 3.min(pts.len().saturating_sub(1)).max(1))
                .map_err(|_| SketchError::Degenerate("bad fit spline"))
        }
        _ => Err(SketchError::WrongEntity("expected a spline")),
    }
}

/// Sample an entity as a polyline whose chord error stays below `tol` (model units).
pub fn polyline(sk: &Sketch, e: EntityId, tol: f64) -> Result<Vec<P2>, SketchError> {
    let seg = |r: f64, span: f64| -> usize {
        let step = if tol >= r {
            1.0
        } else {
            2.0 * (1.0 - tol / r).acos()
        };
        ((span / step.clamp(0.02, 1.0)).ceil() as usize).max(2)
    };
    Ok(match sk.kind(e)?.clone() {
        EntityKind::Point(p) => vec![(sk.value(p.x), sk.value(p.y))],
        EntityKind::Line { p1, p2 } => vec![sk.point(p1)?, sk.point(p2)?],
        EntityKind::Circle { center, radius } => {
            let (c, r) = (sk.point(center)?, sk.value(radius).abs());
            let n = seg(r, TAU);
            (0..=n)
                .map(|i| {
                    (
                        c.0 + r * (TAU * i as f64 / n as f64).cos(),
                        c.1 + r * (TAU * i as f64 / n as f64).sin(),
                    )
                })
                .collect()
        }
        EntityKind::Arc { .. } => {
            let (c, r, a0, a1) = arc_params(sk, e)?;
            let n = seg(r, a1 - a0);
            (0..=n)
                .map(|i| {
                    let a = a0 + (a1 - a0) * i as f64 / n as f64;
                    (c.0 + r * a.cos(), c.1 + r * a.sin())
                })
                .collect()
        }
        EntityKind::Ellipse {
            center,
            major_end,
            minor_radius,
        } => {
            let (c, m) = (sk.point(center)?, sk.point(major_end)?);
            let (ux, uy) = (m.0 - c.0, m.1 - c.1);
            let a = ux.hypot(uy);
            let b = sk.value(minor_radius).abs();
            let (cx, cy) = (ux / a, uy / a);
            let n = seg(a.max(b), TAU);
            (0..=n)
                .map(|i| {
                    let t = TAU * i as f64 / n as f64;
                    let (px, py) = (a * t.cos(), b * t.sin());
                    (c.0 + px * cx - py * cy, c.1 + px * cy + py * cx)
                })
                .collect()
        }
        EntityKind::Spline { .. } | EntityKind::FitSpline { .. } => {
            let curve = spline_curve(sk, e)?;
            let (a, b) = curve.domain();
            let n = 8 * (curve.points().len()).max(4);
            (0..=n)
                .map(|i| {
                    let p = curve.point_at(a + (b - a) * i as f64 / n as f64);
                    (p.x, p.y)
                })
                .collect()
        }
    })
}

fn seg_dist(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}

/// Distance from `p` to the entity.
pub fn distance_to(sk: &Sketch, e: EntityId, p: P2) -> Result<f64, SketchError> {
    let pl = polyline(sk, e, 1e-3)?;
    if pl.len() == 1 {
        return Ok((p.0 - pl[0].0).hypot(p.1 - pl[0].1));
    }
    Ok(pl
        .windows(2)
        .map(|w| seg_dist(p, w[0], w[1]))
        .fold(f64::INFINITY, f64::min))
}

/// Nearest entity to `p` within `tol`; points win ties so they can be picked on top of curves.
pub fn pick(sk: &Sketch, p: P2, tol: f64) -> Option<EntityId> {
    let mut best: Option<(f64, EntityId)> = None;
    for (id, ent) in &sk.entities {
        if let Ok(d) = distance_to(sk, id, p) {
            let d = if matches!(ent.kind, EntityKind::Point(_)) {
                d * 0.5
            } else {
                d
            };
            if d <= tol && best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, id));
            }
        }
    }
    best.map(|(_, id)| id)
}

/// Closest point on the entity to `p`.
pub fn closest_point(sk: &Sketch, e: EntityId, p: P2) -> Result<P2, SketchError> {
    match sk.kind(e)? {
        EntityKind::Circle { .. } | EntityKind::Arc { .. } => {
            let (c, r) = sk.circle_geometry(e)?;
            let (dx, dy) = (p.0 - c.0, p.1 - c.1);
            let l = dx.hypot(dy).max(1e-300);
            if let EntityKind::Arc { .. } = sk.kind(e)? {
                let (_, _, a0, a1) = arc_params(sk, e)?;
                let mut a = dy.atan2(dx);
                while a < a0 {
                    a += TAU;
                }
                if a > a1 {
                    // outside the arc: nearer end
                    let (s, t) = (
                        (c.0 + r * a0.cos(), c.1 + r * a0.sin()),
                        (c.0 + r * a1.cos(), c.1 + r * a1.sin()),
                    );
                    let (ds, dt) = ((p.0 - s.0).hypot(p.1 - s.1), (p.0 - t.0).hypot(p.1 - t.1));
                    return Ok(if ds < dt { s } else { t });
                }
            }
            Ok((c.0 + dx / l * r, c.1 + dy / l * r))
        }
        _ => {
            let pl = polyline(sk, e, 1e-3)?;
            let mut best = (f64::INFINITY, pl[0]);
            for w in pl.windows(2) {
                let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
                let l2 = dx * dx + dy * dy;
                let t = if l2 > 0.0 {
                    (((p.0 - w[0].0) * dx + (p.1 - w[0].1) * dy) / l2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let q = (w[0].0 + t * dx, w[0].1 + t * dy);
                let d = (p.0 - q.0).hypot(p.1 - q.1);
                if d < best.0 {
                    best = (d, q);
                }
            }
            Ok(best.1)
        }
    }
}

fn on_arc(sk: &Sketch, e: EntityId, q: P2) -> bool {
    match arc_params(sk, e) {
        Ok((c, _, a0, a1)) => {
            let mut a = (q.1 - c.1).atan2(q.0 - c.0);
            while a < a0 - 1e-9 {
                a += TAU;
            }
            a <= a1 + 1e-9
        }
        Err(_) => true,
    }
}

/// Intersections of two entities (lines as segments, circles/arcs as such).
pub fn intersections(sk: &Sketch, a: EntityId, b: EntityId) -> Result<Vec<P2>, SketchError> {
    use EntityKind::*;
    let line_pts = |e| -> Result<(P2, P2), SketchError> {
        let (p, q) = sk.line_points(e)?;
        Ok((sk.point(p)?, sk.point(q)?))
    };
    let (ka, kb) = (sk.kind(a)?.clone(), sk.kind(b)?.clone());
    let is_circ = |k: &EntityKind| matches!(k, Circle { .. } | Arc { .. });
    let mut out: Vec<P2> = Vec::new();
    match (&ka, &kb) {
        (Line { .. }, Line { .. }) => {
            let ((a1, a2), (b1, b2)) = (line_pts(a)?, line_pts(b)?);
            let (d1, d2) = ((a2.0 - a1.0, a2.1 - a1.1), (b2.0 - b1.0, b2.1 - b1.1));
            let det = d1.0 * d2.1 - d1.1 * d2.0;
            if det.abs() > 1e-12 {
                let (s, t) = (
                    ((b1.0 - a1.0) * d2.1 - (b1.1 - a1.1) * d2.0) / det,
                    ((b1.0 - a1.0) * d1.1 - (b1.1 - a1.1) * d1.0) / det,
                );
                if (-1e-9..=1.0 + 1e-9).contains(&s) && (-1e-9..=1.0 + 1e-9).contains(&t) {
                    out.push((a1.0 + s * d1.0, a1.1 + s * d1.1));
                }
            }
        }
        (Line { .. }, k) | (k, Line { .. }) if is_circ(k) => {
            let (line, circ) = if matches!(ka, Line { .. }) {
                (a, b)
            } else {
                (b, a)
            };
            let (p1, p2) = line_pts(line)?;
            let (c, r) = sk.circle_geometry(circ)?;
            let d = (p2.0 - p1.0, p2.1 - p1.1);
            let f = (p1.0 - c.0, p1.1 - c.1);
            let (qa, qb, qc) = (
                d.0 * d.0 + d.1 * d.1,
                2.0 * (f.0 * d.0 + f.1 * d.1),
                f.0 * f.0 + f.1 * f.1 - r * r,
            );
            let disc = qb * qb - 4.0 * qa * qc;
            if disc >= 0.0 && qa > 0.0 {
                for sgn in [-1.0, 1.0] {
                    let t = (-qb + sgn * disc.sqrt()) / (2.0 * qa);
                    if (-1e-9..=1.0 + 1e-9).contains(&t) {
                        let q = (p1.0 + t * d.0, p1.1 + t * d.1);
                        if matches!(sk.kind(circ)?, Circle { .. }) || on_arc(sk, circ, q) {
                            out.push(q);
                        }
                        if disc == 0.0 {
                            break;
                        }
                    }
                }
            }
        }
        (x, y) if is_circ(x) && is_circ(y) => {
            let ((c1, r1), (c2, r2)) = (sk.circle_geometry(a)?, sk.circle_geometry(b)?);
            let (dx, dy) = (c2.0 - c1.0, c2.1 - c1.1);
            let d = dx.hypot(dy);
            if d > 1e-12 && d <= r1 + r2 + 1e-9 && d >= (r1 - r2).abs() - 1e-9 {
                let al = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
                let h = (r1 * r1 - al * al).max(0.0).sqrt();
                let (mx, my) = (c1.0 + al * dx / d, c1.1 + al * dy / d);
                for sgn in [-1.0, 1.0] {
                    let q = (mx - sgn * h * dy / d, my + sgn * h * dx / d);
                    if (matches!(ka, Circle { .. }) || on_arc(sk, a, q))
                        && (matches!(kb, Circle { .. }) || on_arc(sk, b, q))
                    {
                        out.push(q);
                    }
                    if h == 0.0 {
                        break;
                    }
                }
            }
        }
        _ => {
            // generic: polyline–polyline
            let (pa, pb) = (polyline(sk, a, 1e-4)?, polyline(sk, b, 1e-4)?);
            for wa in pa.windows(2) {
                for wb in pb.windows(2) {
                    let (d1, d2) = (
                        (wa[1].0 - wa[0].0, wa[1].1 - wa[0].1),
                        (wb[1].0 - wb[0].0, wb[1].1 - wb[0].1),
                    );
                    let det = d1.0 * d2.1 - d1.1 * d2.0;
                    if det.abs() < 1e-14 {
                        continue;
                    }
                    let s = ((wb[0].0 - wa[0].0) * d2.1 - (wb[0].1 - wa[0].1) * d2.0) / det;
                    let t = ((wb[0].0 - wa[0].0) * d1.1 - (wb[0].1 - wa[0].1) * d1.0) / det;
                    if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
                        out.push((wa[0].0 + s * d1.0, wa[0].1 + s * d1.1));
                    }
                }
            }
        }
    }
    Ok(out)
}
