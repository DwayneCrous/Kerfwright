//! Parameter inversion ("param of") and closest-point queries on the analytic
//! and procedural geometry in [`crate::geom`].
//!
//! For the analytic quadrics the inversion is the *natural* one (radial /
//! axial projection) and is exact for points lying on the geometry; the
//! returned `distance` is always the true distance between the query point
//! and the evaluated geometry at the returned parameters. Periodic parameters
//! are unwrapped to lie within half a period of `hint` when one is given.

use crate::geom::*;
use nalgebra::{Point3, Vector3};
use std::f64::consts::{PI, TAU};

/// `x + k·period` closest to `hint`.
pub fn wrap_near(x: f64, hint: f64, period: f64) -> f64 {
    x + period * ((hint - x) / period).round()
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveParam {
    pub t: f64,
    pub distance: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceParam {
    pub u: f64,
    pub v: f64,
    pub distance: f64,
}

fn angle_of(y: f64, x: f64, hint: Option<f64>) -> f64 {
    let a = y.atan2(x);
    match hint {
        Some(h) => wrap_near(a, h, TAU),
        None => {
            if a < 0.0 {
                a + TAU
            } else {
                a
            }
        }
    }
}

/// Parameter of the point on `c` closest to `p`.
pub fn curve_param_of(c: &CurveGeom, p: &Point3<f64>, hint: Option<f64>) -> CurveParam {
    let finish = |t: f64| CurveParam {
        t,
        distance: (c.point_at(t) - p).norm(),
    };
    match c {
        CurveGeom::Line(l) => {
            let t = (p - l.origin).dot(&l.dir) / l.dir.norm_squared();
            finish(t)
        }
        CurveGeom::Circle(ci) => {
            let q = p - ci.frame.origin;
            finish(angle_of(q.dot(&ci.frame.y), q.dot(&ci.frame.x), hint))
        }
        CurveGeom::Ellipse(e) => {
            let q = p - e.frame.origin;
            let (qx, qy) = (q.dot(&e.frame.x), q.dot(&e.frame.y));
            let mut best: Option<CurveParam> = None;
            let t0 = (qy / e.minor).atan2(qx / e.major);
            let mut seeds = vec![t0];
            for k in 0..8 {
                seeds.push(k as f64 * TAU / 8.0);
            }
            for s in seeds {
                let mut t = s;
                for _ in 0..40 {
                    let (ct, st) = (t.cos(), t.sin());
                    let d = (e.major * ct - qx, e.minor * st - qy);
                    let d1 = (-e.major * st, e.minor * ct);
                    let d2 = (-e.major * ct, -e.minor * st);
                    let f = d.0 * d1.0 + d.1 * d1.1;
                    let fp = d1.0 * d1.0 + d1.1 * d1.1 + d.0 * d2.0 + d.1 * d2.1;
                    if fp.abs() < 1e-300 {
                        break;
                    }
                    let step = -f / fp;
                    if !step.is_finite() {
                        break;
                    }
                    t += step.clamp(-0.5, 0.5);
                    if step.abs() < 1e-15 {
                        break;
                    }
                }
                let cand = finish(t);
                if best.is_none_or(|b| cand.distance < b.distance - 1e-15) {
                    best = Some(cand);
                }
            }
            let b = best.unwrap();
            let t = match hint {
                Some(h) => wrap_near(b.t, h, TAU),
                None => b.t.rem_euclid(TAU),
            };
            finish(t)
        }
        CurveGeom::Nurbs(n) => {
            let pr = n.closest_point(p);
            CurveParam {
                t: pr.u,
                distance: pr.distance,
            }
        }
        CurveGeom::Procedural(pc) => {
            let pr = pc.approximation.closest_point(p);
            CurveParam {
                t: pr.u,
                distance: pr.distance,
            }
        }
    }
}

/// Generic 1-D minimiser of `|C(t) - p|²` over `[a, b]` by sampling + Newton.
fn curve_closest_sampled(c: &CurveGeom, p: &Point3<f64>, a: f64, b: f64) -> (f64, f64) {
    let n = 128;
    let mut best = (a, f64::INFINITY);
    let mut samples = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = a + (b - a) * i as f64 / n as f64;
        let d = (c.point_at(t) - p).norm_squared();
        samples.push((t, d));
    }
    for i in 0..=n {
        let lo = if i == 0 {
            f64::INFINITY
        } else {
            samples[i - 1].1
        };
        let hi = if i == n {
            f64::INFINITY
        } else {
            samples[i + 1].1
        };
        if samples[i].1 <= lo && samples[i].1 <= hi {
            let mut t = samples[i].0;
            let h = (b - a) / n as f64;
            // golden-section refinement in the bracket
            let (mut l, mut r) = ((t - h).max(a), (t + h).min(b));
            let g = 0.618_033_988_749_894_9;
            for _ in 0..80 {
                let m1 = r - g * (r - l);
                let m2 = l + g * (r - l);
                let f1 = (c.point_at(m1) - p).norm_squared();
                let f2 = (c.point_at(m2) - p).norm_squared();
                if f1 < f2 {
                    r = m2;
                } else {
                    l = m1;
                }
            }
            t = 0.5 * (l + r);
            let d = (c.point_at(t) - p).norm();
            if d < best.1 || best.1.is_infinite() {
                best = (t, d);
            }
        }
    }
    best
}

/// Parameters `(u, v)` of `p` on the surface.
pub fn surface_param_of(
    s: &SurfaceGeom,
    p: &Point3<f64>,
    hint: Option<(f64, f64)>,
) -> SurfaceParam {
    let hu = hint.map(|h| h.0);
    let hv = hint.map(|h| h.1);
    let finish = |u: f64, v: f64| SurfaceParam {
        u,
        v,
        distance: (s.point_at(u, v) - p).norm(),
    };
    match s {
        SurfaceGeom::Plane(pl) => {
            let q = p - pl.frame.origin;
            finish(q.dot(&pl.frame.x), q.dot(&pl.frame.y))
        }
        SurfaceGeom::Cylinder(c) => {
            let q = p - c.frame.origin;
            finish(
                angle_of(q.dot(&c.frame.y), q.dot(&c.frame.x), hu),
                q.dot(&c.frame.z),
            )
        }
        SurfaceGeom::Cone(c) => {
            let q = p - c.frame.origin;
            let (qx, qy, h) = (q.dot(&c.frame.x), q.dot(&c.frame.y), q.dot(&c.frame.z));
            let u = if qx.hypot(qy) < 1e-14 {
                hu.unwrap_or(0.0)
            } else {
                angle_of(qy, qx, hu)
            };
            finish(u, h)
        }
        SurfaceGeom::Sphere(sp) => {
            let q = p - sp.frame.origin;
            let (qx, qy, qz) = (q.dot(&sp.frame.x), q.dot(&sp.frame.y), q.dot(&sp.frame.z));
            let rho = qx.hypot(qy);
            let u = if rho < 1e-14 * sp.radius {
                hu.unwrap_or(0.0)
            } else {
                angle_of(qy, qx, hu)
            };
            finish(u, qz.atan2(rho))
        }
        SurfaceGeom::Torus(t) => {
            let q = p - t.frame.origin;
            let (qx, qy, qz) = (q.dot(&t.frame.x), q.dot(&t.frame.y), q.dot(&t.frame.z));
            let rho = qx.hypot(qy);
            let u = if rho < 1e-14 {
                hu.unwrap_or(0.0)
            } else {
                angle_of(qy, qx, hu)
            };
            let w = rho - t.major_radius;
            let mut v = qz.atan2(w);
            v = match hv {
                Some(h) => wrap_near(v, h, TAU),
                None => v.rem_euclid(TAU),
            };
            finish(u, v)
        }
        SurfaceGeom::Nurbs(n) => {
            // with a hint, a local Newton iteration is enough when it lands on the surface
            if let Some((hu0, hv0)) = hint {
                if let Some(local) = nurbs_local_projection(n, p, hu0, hv0) {
                    return local;
                }
            }
            let pr = n.closest_point(p);
            SurfaceParam {
                u: pr.u,
                v: pr.v,
                distance: pr.distance,
            }
        }
        SurfaceGeom::Procedural(pc) => {
            let pr = pc.approximation.closest_point(p);
            SurfaceParam {
                u: pr.u,
                v: pr.v,
                distance: pr.distance,
            }
        }
        SurfaceGeom::Extrusion { profile, dir } => {
            let d = dir.normalize();
            // distance from p to the extrusion line through profile(u) = |perp(p - C(u))|
            let (a0, b0) = profile.domain();
            let (a, b) = if a0.is_finite() && b0.is_finite() {
                (a0, b0)
            } else {
                // infinite profile (line): the closed form
                match &**profile {
                    CurveGeom::Line(l) => {
                        // minimise over plane spanned; solve linear system for (t, v)
                        let e1 = l.dir;
                        let w = p - l.origin;
                        let (a11, a12, a22) = (e1.dot(&e1), e1.dot(dir), dir.dot(dir));
                        let (b1, b2) = (w.dot(&e1), w.dot(dir));
                        let det = a11 * a22 - a12 * a12;
                        if det.abs() > 1e-300 {
                            let u = (b1 * a22 - b2 * a12) / det;
                            let v = (a11 * b2 - a12 * b1) / det;
                            return finish(u, v);
                        }
                        return finish(0.0, 0.0);
                    }
                    _ => (hu.unwrap_or(0.0) - TAU, hu.unwrap_or(0.0) + TAU),
                }
            };
            let perp = |c: &CurveGeom, t: f64| {
                let w = p - c.point_at(t);
                w - d * w.dot(&d)
            };
            // sample |perp|² over u; among equally good minima (a closed profile touches the same
            // point at both ends) prefer the one nearest the hint
            let mut u = {
                let n = 256;
                let vals: Vec<(f64, f64)> = (0..=n)
                    .map(|i| {
                        let t = a + (b - a) * i as f64 / n as f64;
                        (t, perp(profile, t).norm_squared())
                    })
                    .collect();
                let fmin = vals.iter().map(|v| v.1).fold(f64::INFINITY, f64::min);
                let mut cands: Vec<f64> = Vec::new();
                for i in 0..=n {
                    let lo = if i == 0 { f64::INFINITY } else { vals[i - 1].1 };
                    let hi = if i == n { f64::INFINITY } else { vals[i + 1].1 };
                    if vals[i].1 <= lo
                        && vals[i].1 <= hi
                        && vals[i].1 <= fmin + 1e-9 * (1.0 + fmin) + 1e-12
                    {
                        cands.push(vals[i].0);
                    }
                }
                match hu {
                    Some(h) => cands
                        .into_iter()
                        .min_by(|x, y| {
                            (x - h)
                                .abs()
                                .partial_cmp(&(y - h).abs())
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .unwrap_or(a),
                    None => cands.first().copied().unwrap_or(a),
                }
            };
            // Newton on g(u) = perp · (-C'⊥)
            for _ in 0..60 {
                let h = 1e-6 * (b - a).max(1e-3);
                let g = |t: f64| {
                    let w = perp(profile, t);
                    let c1 = profile.derivative(t);
                    let c1p = c1 - d * c1.dot(&d);
                    w.dot(&c1p)
                };
                let (g0, g1) = (g(u), g(u + h));
                let dg = (g1 - g0) / h;
                if dg.abs() < 1e-300 {
                    break;
                }
                let step = -g0 / dg;
                if !step.is_finite() {
                    break;
                }
                let un = (u + step.clamp(-(b - a) / 8.0, (b - a) / 8.0)).clamp(a, b);
                let done = (un - u).abs() < 1e-15;
                u = un;
                if done {
                    break;
                }
            }
            // S = C(u) + dir·v  ⇒ v = (p − C)·dir / |dir|²
            let v = (p - profile.point_at(u)).dot(dir) / dir.norm_squared();
            finish(u, v)
        }
        SurfaceGeom::Revolution {
            profile,
            axis_origin,
            axis_dir,
        } => {
            let axis = axis_dir.normalize();
            let q = p - axis_origin;
            let h = q.dot(&axis);
            let radial = q - axis * h;
            let rho = radial.norm();
            let (a0, b0) = profile.domain();
            let (a, b) = if a0.is_finite() && b0.is_finite() {
                (a0, b0)
            } else {
                (hu.unwrap_or(0.0) - 100.0, hu.unwrap_or(0.0) + 100.0)
            };
            // profile in (axial, radial) coordinates
            let cyl = |t: f64| {
                let w = profile.point_at(t) - axis_origin;
                let ax = w.dot(&axis);
                let r = (w - axis * ax).norm();
                (ax, r)
            };
            let dist2 = |t: f64| {
                let (ax, r) = cyl(t);
                (ax - h).powi(2) + (r - rho).powi(2)
            };
            let n = 256;
            let mut bt = (a, f64::INFINITY);
            for i in 0..=n {
                let t = a + (b - a) * i as f64 / n as f64;
                let f = dist2(t);
                if f < bt.1 {
                    bt = (t, f);
                }
            }
            let (mut l, mut r) = (
                (bt.0 - (b - a) / n as f64).max(a),
                (bt.0 + (b - a) / n as f64).min(b),
            );
            let g = 0.618_033_988_749_894_9;
            for _ in 0..80 {
                let m1 = r - g * (r - l);
                let m2 = l + g * (r - l);
                if dist2(m1) < dist2(m2) {
                    r = m2;
                } else {
                    l = m1;
                }
            }
            let u = 0.5 * (l + r);
            // a closed periodic profile (circle, ellipse): continue the caller's branch across the seam
            let u = match (hu, s.period_u()) {
                (Some(h), Some(per)) => wrap_near(u, h, per),
                _ => u,
            };
            // angle between the profile point's radial direction and p's radial direction
            let w = profile.point_at(u) - axis_origin;
            let pr = w - axis * w.dot(&axis);
            let v = if pr.norm() < 1e-14 || rho < 1e-14 {
                hv.unwrap_or(0.0)
            } else {
                let sa = axis.dot(&pr.cross(&radial));
                let ca = pr.dot(&radial);
                let a = sa.atan2(ca);
                match hv {
                    Some(hh) => wrap_near(a, hh, TAU),
                    None => a.rem_euclid(TAU),
                }
            };
            finish(u, v)
        }
    }
}

/// Closest point on a curve restricted to `[a, b]` (generic sampled search; used for
/// infinite lines clipped to a window).
pub fn curve_closest_in(c: &CurveGeom, p: &Point3<f64>, a: f64, b: f64) -> CurveParam {
    let (t, d) = curve_closest_sampled(c, p, a, b);
    CurveParam { t, distance: d }
}

/// Unit axis direction helper.
pub fn unit(v: &Vector3<f64>) -> Vector3<f64> {
    v / v.norm()
}

#[allow(dead_code)]
const _: f64 = PI;

/// Local Newton projection of `p` onto a NURBS surface from `(u0, v0)`; `Some` only if it converged to a
/// point essentially *on* the surface (distance below 1e-9 relative), otherwise the caller should search globally.
fn nurbs_local_projection(
    n: &crate::NurbsSurface,
    p: &Point3<f64>,
    u0: f64,
    v0: f64,
) -> Option<SurfaceParam> {
    let ((ua, ub), (va, vb)) = n.domain();
    let (mut u, mut v) = (u0.clamp(ua, ub), v0.clamp(va, vb));
    for _ in 0..30 {
        let d = n.derivatives(u, v, 2);
        let r = d[0][0] - p.coords;
        let (su, sv) = (d[1][0], d[0][1]);
        let f = [r.dot(&su), r.dot(&sv)];
        let j00 = su.dot(&su) + r.dot(&d[2][0]);
        let j01 = su.dot(&sv) + r.dot(&d[1][1]);
        let j11 = sv.dot(&sv) + r.dot(&d[0][2]);
        let det = j00 * j11 - j01 * j01;
        if det.abs() < 1e-300 {
            return None;
        }
        let du = -(j11 * f[0] - j01 * f[1]) / det;
        let dv = -(-j01 * f[0] + j00 * f[1]) / det;
        if !du.is_finite() || !dv.is_finite() {
            return None;
        }
        let (un, vn) = ((u + du).clamp(ua, ub), (v + dv).clamp(va, vb));
        let moved = (un - u).abs() + (vn - v).abs();
        u = un;
        v = vn;
        if moved < 1e-15 {
            break;
        }
    }
    let q = n.point_at(u, v);
    let dist = (q - p).norm();
    (dist <= 1e-9 * (1.0 + p.coords.amax())).then_some(SurfaceParam {
        u,
        v,
        distance: dist,
    })
}
