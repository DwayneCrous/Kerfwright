//! Curve–curve intersection in 3D (sampling to find candidates, Gauss–Newton to polish).

use crate::geom::*;
use nalgebra::{Point3, Vector3};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveHit {
    pub t1: f64,
    pub t2: f64,
    pub point: Point3<f64>,
}

fn period(c: &CurveGeom) -> Option<f64> {
    match c {
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => Some(TAU),
        _ => None,
    }
}

fn sample_count(c: &CurveGeom, r: (f64, f64)) -> usize {
    match c {
        CurveGeom::Line(_) => 1,
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => {
            ((((r.1 - r.0).abs() / TAU) * 96.0).ceil() as usize).max(8)
        }
        CurveGeom::Nurbs(n) => ((n.knots().distinct().len().max(2) - 1) * 3).clamp(16, 600),
        CurveGeom::Procedural(p) => {
            ((p.approximation.knots().distinct().len().max(2) - 1) * 3).clamp(16, 600)
        }
    }
}

struct Poly {
    ts: Vec<f64>,
    pts: Vec<Point3<f64>>,
}

fn sample(c: &CurveGeom, r: (f64, f64)) -> Poly {
    let n = sample_count(c, r);
    let ts: Vec<f64> = (0..=n)
        .map(|i| r.0 + (r.1 - r.0) * i as f64 / n as f64)
        .collect();
    let pts = ts.iter().map(|t| c.point_at(*t)).collect();
    Poly { ts, pts }
}

/// Closest points between segments `p0-p1` and `q0-q1`: parameters in [0,1] and distance.
fn seg_seg(
    p0: &Point3<f64>,
    p1: &Point3<f64>,
    q0: &Point3<f64>,
    q1: &Point3<f64>,
) -> (f64, f64, f64) {
    let (d1, d2, r) = (p1 - p0, q1 - q0, p0 - q0);
    let (a, e, f) = (d1.dot(&d1), d2.dot(&d2), d2.dot(&r));
    let (mut s, mut t);
    if a <= 1e-300 && e <= 1e-300 {
        return (0.0, 0.0, r.norm());
    }
    if a <= 1e-300 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(&r);
        if e <= 1e-300 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            s = if denom > 1e-14 * a * e {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    let diff = (p0 + d1 * s) - (q0 + d2 * t);
    (s, t, diff.norm())
}

type CellRange = ((i64, i64), (i64, i64), (i64, i64));

fn derivative(c: &CurveGeom, t: f64) -> Vector3<f64> {
    c.derivative(t)
}

/// All intersections of `c1` over `r1` with `c2` over `r2` (transversal and near-tangent ones the
/// sampling resolves), refined to near machine precision. `tol` is the accepted residual distance.
pub fn curve_curve(
    c1: &CurveGeom,
    r1: (f64, f64),
    c2: &CurveGeom,
    r2: (f64, f64),
    tol: f64,
) -> Vec<CurveHit> {
    if let Some(h) = closed_form(c1, r1, c2, r2, tol) {
        return h;
    }
    let (p1, p2) = (sample(c1, r1), sample(c2, r2));
    // chord-deviation allowance for candidate pairs: the largest sagitta of either polyline
    let sagitta = |c: &CurveGeom, p: &Poly| -> f64 {
        let mut m = 0.0f64;
        for i in 0..p.pts.len() - 1 {
            let mid = c.point_at(0.5 * (p.ts[i] + p.ts[i + 1]));
            let chord_mid = nalgebra::center(&p.pts[i], &p.pts[i + 1]);
            m = m.max((mid - chord_mid).norm());
        }
        m
    };
    let slack = 1.5 * (sagitta(c1, &p1) + sagitta(c2, &p2)) + 10.0 * tol;
    let mut hits: Vec<CurveHit> = Vec::new();
    let (lo1, hi1) = (r1.0.min(r1.1), r1.0.max(r1.1));
    let (lo2, hi2) = (r2.0.min(r2.1), r2.0.max(r2.1));
    // uniform grid over the segments of the second polyline
    let cell = {
        let mean2 = p2.pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>()
            / (p2.pts.len().max(2) - 1) as f64;
        (2.0 * mean2).max(4.0 * slack).max(1e-9)
    };
    let key = |x: f64| (x / cell).floor() as i64;
    // a segment spanning more than this many cells is handled outside the grid
    const MAX_CELLS: i64 = 400;
    let span = |lo: Point3<f64>, hi: Point3<f64>| -> CellRange {
        (
            (key(lo.x - slack), key(hi.x + slack)),
            (key(lo.y - slack), key(hi.y + slack)),
            (key(lo.z - slack), key(hi.z + slack)),
        )
    };
    let count =
        |r: &CellRange| (r.0 .1 - r.0 .0 + 1) * (r.1 .1 - r.1 .0 + 1) * (r.2 .1 - r.2 .0 + 1);
    let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<usize>> =
        std::collections::HashMap::new();
    let mut long2: Vec<usize> = Vec::new();
    for j in 0..p2.pts.len() - 1 {
        let (b0, b1) = (p2.pts[j], p2.pts[j + 1]);
        let r = span(
            Point3::new(b0.x.min(b1.x), b0.y.min(b1.y), b0.z.min(b1.z)),
            Point3::new(b0.x.max(b1.x), b0.y.max(b1.y), b0.z.max(b1.z)),
        );
        if count(&r) > MAX_CELLS {
            long2.push(j);
            continue;
        }
        for x in r.0 .0..=r.0 .1 {
            for y in r.1 .0..=r.1 .1 {
                for z in r.2 .0..=r.2 .1 {
                    grid.entry((x, y, z)).or_default().push(j);
                }
            }
        }
    }
    let mut cand: Vec<usize> = Vec::new();
    for i in 0..p1.pts.len() - 1 {
        let (a0, a1) = (p1.pts[i], p1.pts[i + 1]);
        cand.clear();
        let r = span(
            Point3::new(a0.x.min(a1.x), a0.y.min(a1.y), a0.z.min(a1.z)),
            Point3::new(a0.x.max(a1.x), a0.y.max(a1.y), a0.z.max(a1.z)),
        );
        if count(&r) > MAX_CELLS {
            cand.extend(0..p2.pts.len() - 1);
        } else {
            cand.extend_from_slice(&long2);
            for x in r.0 .0..=r.0 .1 {
                for y in r.1 .0..=r.1 .1 {
                    for z in r.2 .0..=r.2 .1 {
                        if let Some(l) = grid.get(&(x, y, z)) {
                            cand.extend_from_slice(l);
                        }
                    }
                }
            }
        }
        cand.sort_unstable();
        cand.dedup();
        for &j in &cand {
            let (b0, b1) = (p2.pts[j], p2.pts[j + 1]);
            let (s, t, d) = seg_seg(&a0, &a1, &b0, &b1);
            if d > slack {
                continue;
            }
            let mut u = p1.ts[i] + (p1.ts[i + 1] - p1.ts[i]) * s;
            let mut v = p2.ts[j] + (p2.ts[j + 1] - p2.ts[j]) * t;
            // Gauss–Newton on |C1(u) − C2(v)|²
            for _ in 0..50 {
                let r = c1.point_at(u) - c2.point_at(v);
                let (d1, d2) = (derivative(c1, u), derivative(c2, v));
                // J = [d1, -d2] (3×2); solve JᵀJ δ = −Jᵀ r
                let (a, b, c) = (d1.dot(&d1), -d1.dot(&d2), d2.dot(&d2));
                let (g1, g2) = (d1.dot(&r), -d2.dot(&r));
                let det = a * c - b * b;
                if det.abs() < 1e-300 {
                    break;
                }
                let du = -(c * g1 - b * g2) / det;
                let dv = -(-b * g1 + a * g2) / det;
                let lim1 = 0.5 * (p1.ts[1] - p1.ts[0]).abs().max(1e-3);
                let lim2 = 0.5 * (p2.ts[1] - p2.ts[0]).abs().max(1e-3);
                u += du.clamp(
                    -lim1.max((r1.1 - r1.0).abs() * 0.1),
                    lim1.max((r1.1 - r1.0).abs() * 0.1),
                );
                v += dv.clamp(
                    -lim2.max((r2.1 - r2.0).abs() * 0.1),
                    lim2.max((r2.1 - r2.0).abs() * 0.1),
                );
                if du.abs() < 1e-15 * (1.0 + u.abs()) && dv.abs() < 1e-15 * (1.0 + v.abs()) {
                    break;
                }
            }
            let pt1 = c1.point_at(u);
            let pt2 = c2.point_at(v);
            if (pt1 - pt2).norm() > tol {
                continue;
            }
            // normalise periodic parameters into the requested ranges
            let norm = |c: &CurveGeom, x: f64, lo: f64, hi: f64| -> Option<f64> {
                let eps = 1e-9 * (1.0 + hi.abs().max(lo.abs()));
                match period(c) {
                    Some(p) => {
                        let mut y = x;
                        for k in -3..=3 {
                            let z = x + p * k as f64;
                            if z >= lo - eps && z <= hi + eps {
                                y = z;
                                return Some(y.clamp(lo, hi));
                            }
                        }
                        let _ = y;
                        None
                    }
                    None => (x >= lo - eps && x <= hi + eps).then_some(x.clamp(lo, hi)),
                }
            };
            let (Some(u), Some(v)) = (norm(c1, u, lo1, hi1), norm(c2, v, lo2, hi2)) else {
                continue;
            };
            if hits
                .iter()
                .any(|h| (h.point - pt1).norm() < 1e-8 * (1.0 + pt1.coords.amax()))
            {
                continue;
            }
            hits.push(CurveHit {
                t1: u,
                t2: v,
                point: Point3::from((pt1.coords + pt2.coords) * 0.5),
            });
        }
    }
    hits.sort_by(|a, b| a.t1.partial_cmp(&b.t1).unwrap_or(std::cmp::Ordering::Equal));
    hits
}

fn in_range(t: f64, r: (f64, f64)) -> bool {
    let (lo, hi) = (r.0.min(r.1), r.0.max(r.1));
    let eps = 1e-10 * (1.0 + lo.abs().max(hi.abs()));
    t >= lo - eps && t <= hi + eps
}

/// Exact solutions for line/line and line/circle pairs (`None` for everything else).
fn closed_form(
    c1: &CurveGeom,
    r1: (f64, f64),
    c2: &CurveGeom,
    r2: (f64, f64),
    tol: f64,
) -> Option<Vec<CurveHit>> {
    match (c1, c2) {
        (CurveGeom::Line(a), CurveGeom::Line(b)) => {
            // closest points of two lines; a hit only if they actually meet
            let (d1, d2, w) = (a.dir, b.dir, a.origin - b.origin);
            let (aa, bb, cc, dd, ee) = (
                d1.dot(&d1),
                d1.dot(&d2),
                d2.dot(&d2),
                d1.dot(&w),
                d2.dot(&w),
            );
            let det = aa * cc - bb * bb;
            if det.abs() < 1e-14 * aa * cc {
                return None; // parallel: leave to the generic routine (overlap handling)
            }
            let t = (bb * ee - cc * dd) / det;
            let u = (aa * ee - bb * dd) / det;
            let (pa, pb) = (a.origin + d1 * t, b.origin + d2 * u);
            if (pa - pb).norm() > tol || !in_range(t, r1) || !in_range(u, r2) {
                return Some(Vec::new());
            }
            Some(vec![CurveHit {
                t1: t,
                t2: u,
                point: nalgebra::center(&pa, &pb),
            }])
        }
        (CurveGeom::Line(_), CurveGeom::Circle(_)) => {
            closed_form_line_circle(c1, r1, c2, r2, tol, false)
        }
        (CurveGeom::Circle(_), CurveGeom::Line(_)) => {
            closed_form_line_circle(c2, r2, c1, r1, tol, true)
        }
        _ => None,
    }
}

fn closed_form_line_circle(
    l: &CurveGeom,
    rl: (f64, f64),
    c: &CurveGeom,
    rc: (f64, f64),
    tol: f64,
    swap: bool,
) -> Option<Vec<CurveHit>> {
    let (CurveGeom::Line(line), CurveGeom::Circle(ci)) = (l, c) else {
        return None;
    };
    // coplanar only; skew configurations use the generic routine
    let n = ci.frame.z;
    if line.dir.dot(&n).abs() > 1e-9 * line.dir.norm()
        || (line.origin - ci.frame.origin).dot(&n).abs() > 1e-9 * (1.0 + ci.radius)
    {
        return None;
    }
    let (o, d) = (line.origin - ci.frame.origin, line.dir);
    let dd = d.dot(&d);
    // parameter of the foot of the perpendicular from the centre, and its distance
    let t_foot = -o.dot(&d) / dd;
    let dperp = (o + d * t_foot).norm();
    let mut out = Vec::new();
    if dperp > ci.radius + tol {
        return Some(out);
    }
    let roots: Vec<f64> = if (dperp - ci.radius).abs() <= tol {
        // tangent contact: one point (the foot), however the discriminant rounds
        vec![t_foot]
    } else {
        let h = (ci.radius * ci.radius - dperp * dperp).max(0.0).sqrt() / dd.sqrt();
        vec![t_foot - h, t_foot + h]
    };
    for t in roots {
        if !in_range(t, rl) {
            continue;
        }
        let p = line.origin + d * t;
        let q = p - ci.frame.origin;
        let mut ang = q.dot(&ci.frame.y).atan2(q.dot(&ci.frame.x));
        let (lo, hi) = (rc.0.min(rc.1), rc.0.max(rc.1));
        // bring the angle into the requested window
        let mut ok = false;
        for k in -2..=3 {
            let cand = ang + TAU * k as f64;
            if in_range(cand, (lo, hi)) {
                ang = cand;
                ok = true;
                break;
            }
        }
        if !ok {
            continue;
        }
        let hit = if swap {
            CurveHit {
                t1: ang,
                t2: t,
                point: p,
            }
        } else {
            CurveHit {
                t1: t,
                t2: ang,
                point: p,
            }
        };
        out.push(hit);
    }
    Some(out)
}
