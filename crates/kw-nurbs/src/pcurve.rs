//! Parametric-space curves ("pcurves"): the image of a 3D curve under the
//! inverse of a surface parameterisation, sharing the 3D curve's parameter.
//!
//! A pcurve is returned as a [`CurveGeom`] in the `z = 0` plane (`x = u`,
//! `y = v`). Exact lines / circles / ellipses are produced where the geometry
//! allows; otherwise an adaptive cubic Hermite spline reproduces the curve to
//! `tol`.

use crate::curve::NurbsCurve;
use crate::fit::hermite_fit;
use crate::geom::*;
use crate::knots::KnotVector;
use crate::proj::{surface_param_of, wrap_near};
use kw_math::Frame3;
use nalgebra::{Matrix3x2, Point3, Vector2, Vector3};
use std::f64::consts::TAU;

fn xy(u: f64, v: f64) -> Point3<f64> {
    Point3::new(u, v, 0.0)
}

fn line2(o: (f64, f64), d: (f64, f64)) -> CurveGeom {
    CurveGeom::Line(Line {
        origin: xy(o.0, o.1),
        dir: Vector3::new(d.0, d.1, 0.0),
    })
}

/// Solve `[Su Sv]·(u', v') = c'` in the least-squares sense.
fn uv_velocity(s: &SurfaceGeom, u: f64, v: f64, c1: &Vector3<f64>) -> Option<(f64, f64)> {
    let (su, sv) = s.derivatives(u, v);
    let j = Matrix3x2::from_columns(&[su, sv]);
    let jt = j.transpose();
    let a = jt * j;
    let det = a.determinant();
    let scale = a[(0, 0)] * a[(1, 1)];
    if det.is_nan() || det.abs() <= 1e-10 * scale.max(1e-300) {
        return None;
    }
    let b = jt * c1;
    let sol = a.try_inverse()? * Vector2::new(b.x, b.y);
    Some((sol.x, sol.y))
}

fn plane_pcurve(pl: &Plane, c: &CurveGeom) -> Option<CurveGeom> {
    let f = &pl.frame;
    let proj_pt = |p: &Point3<f64>| {
        let q = p - f.origin;
        (q.dot(&f.x), q.dot(&f.y))
    };
    let proj_vec = |v: &Vector3<f64>| (v.dot(&f.x), v.dot(&f.y));
    match c {
        CurveGeom::Line(l) => Some(line2(proj_pt(&l.origin), proj_vec(&l.dir))),
        CurveGeom::Circle(ci) => {
            // circle must lie in the plane (normals parallel)
            if ci.frame.z.cross(&f.z).norm() > 1e-9 {
                return None;
            }
            let o = proj_pt(&ci.frame.origin);
            let (ax, ay) = (proj_vec(&ci.frame.x), proj_vec(&ci.frame.y));
            let fr = Frame3 {
                origin: xy(o.0, o.1),
                x: Vector3::new(ax.0, ax.1, 0.0),
                y: Vector3::new(ay.0, ay.1, 0.0),
                z: Vector3::z(),
            };
            Some(CurveGeom::Circle(Circle {
                frame: fr,
                radius: ci.radius,
            }))
        }
        CurveGeom::Ellipse(e) => {
            if e.frame.z.cross(&f.z).norm() > 1e-9 {
                return None;
            }
            let o = proj_pt(&e.frame.origin);
            let (ax, ay) = (proj_vec(&e.frame.x), proj_vec(&e.frame.y));
            Some(CurveGeom::Ellipse(Ellipse {
                frame: Frame3 {
                    origin: xy(o.0, o.1),
                    x: Vector3::new(ax.0, ax.1, 0.0),
                    y: Vector3::new(ay.0, ay.1, 0.0),
                    z: Vector3::z(),
                },
                major: e.major,
                minor: e.minor,
            }))
        }
        CurveGeom::Nurbs(n) => {
            let pts = n
                .points()
                .iter()
                .map(|p| {
                    let q = proj_pt(p);
                    xy(q.0, q.1)
                })
                .collect();
            NurbsCurve::new(n.knots().clone(), pts, n.weights().to_vec())
                .ok()
                .map(CurveGeom::Nurbs)
        }
        CurveGeom::Procedural(_) => None,
    }
}

/// Closed-form pcurves for the common simple cases.
fn exact_pcurve(
    s: &SurfaceGeom,
    c: &CurveGeom,
    range: (f64, f64),
    hint: Option<(f64, f64)>,
) -> Option<CurveGeom> {
    match (s, c) {
        (SurfaceGeom::Plane(pl), _) => plane_pcurve(pl, c),
        // circle about the axis of a cylinder / cone / sphere (latitude circle): v const, u linear in t
        (SurfaceGeom::Cylinder(cy), CurveGeom::Circle(ci)) => {
            let f = &cy.frame;
            if ci.frame.z.cross(&f.z).norm() > 1e-9 || (ci.radius - cy.radius).abs() > 1e-9 {
                return None;
            }
            axial_circle_pcurve(f, ci, range, hint)
        }
        (SurfaceGeom::Cone(co), CurveGeom::Circle(ci)) => {
            let f = &co.frame;
            if ci.frame.z.cross(&f.z).norm() > 1e-9 {
                return None;
            }
            axial_circle_pcurve(f, ci, range, hint)
        }
        // great circle through the poles (a meridian): u constant on the branch around the mid-point
        (SurfaceGeom::Sphere(sp), CurveGeom::Circle(ci))
            if ci.frame.z.dot(&sp.frame.z).abs() < 1e-9
                && (ci.frame.origin - sp.frame.origin).norm() < 1e-9 * (1.0 + sp.radius)
                && (ci.radius - sp.radius).abs() < 1e-9 * (1.0 + sp.radius) =>
        {
            let f = &sp.frame;
            let (a, b) = (ci.frame.x.dot(&f.z), ci.frame.y.dot(&f.z));
            let phi = a.atan2(b);
            let tm = 0.5 * (range.0 + range.1);
            let wrap = |x: f64| x - TAU * ((x + std::f64::consts::PI) / TAU).floor();
            let w = wrap(tm + phi);
            let (sgn, v_mid) = if w.abs() <= std::f64::consts::FRAC_PI_2 + 1e-12 {
                (1.0, w)
            } else {
                (-1.0, wrap(std::f64::consts::PI - (tm + phi)))
            };
            let q = ci.frame.origin + (ci.frame.x * tm.cos() + ci.frame.y * tm.sin()) * ci.radius
                - f.origin;
            let u = wrap_near(
                q.dot(&f.y).atan2(q.dot(&f.x)),
                hint.map_or(0.0, |h| h.0),
                TAU,
            );
            Some(line2((u, v_mid - sgn * tm), (0.0, sgn)))
        }
        (SurfaceGeom::Sphere(sp), CurveGeom::Circle(ci)) => {
            let f = &sp.frame;
            if ci.frame.z.cross(&f.z).norm() > 1e-9 {
                return None;
            }
            // v = latitude of the circle plane
            axial_circle_pcurve_sphere(f, ci, sp.radius, range, hint)
        }
        // generatrix of a cone (possibly through the apex): u constant, v = axial coordinate
        (SurfaceGeom::Cone(co), CurveGeom::Line(l)) => {
            let f = &co.frame;
            let (t0, t1) = range;
            // pick the sample farther from the axis to define u
            let radial = |t: f64| {
                let q = l.origin + l.dir * t - f.origin;
                (q.dot(&f.x), q.dot(&f.y))
            };
            let (r0, r1) = (radial(t0), radial(t1));
            let rr = if r0.0.hypot(r0.1) >= r1.0.hypot(r1.1) {
                r0
            } else {
                r1
            };
            if rr.0.hypot(rr.1) < 1e-12 {
                return None;
            }
            let u = wrap_near(rr.1.atan2(rr.0), hint.map_or(0.0, |h| h.0), TAU);
            let q0 = l.origin - f.origin;
            Some(line2((u, q0.dot(&f.z)), (0.0, l.dir.dot(&f.z))))
        }
        (SurfaceGeom::Cylinder(cy), CurveGeom::Line(l)) => {
            let f = &cy.frame;
            if l.dir.cross(&f.z).norm() > 1e-9 * l.dir.norm() {
                return None;
            }
            let q = l.origin - f.origin;
            let rad = Vector3::new(q.dot(&f.x), q.dot(&f.y), 0.0);
            if (rad.norm() - cy.radius).abs() > 1e-8 {
                return None;
            }
            let u = wrap_near(rad.y.atan2(rad.x), hint.map_or(0.0, |h| h.0), TAU);
            Some(line2((u, q.dot(&f.z)), (0.0, l.dir.dot(&f.z))))
        }
        _ => None,
    }
}

fn axial_circle_pcurve(
    f: &Frame3,
    ci: &Circle,
    range: (f64, f64),
    hint: Option<(f64, f64)>,
) -> Option<CurveGeom> {
    // circle: centre + r(cos t · ex + sin t · ey), with ex, ey ⟂ axis. Angle of point at t about the
    // surface axis: phi(t) = t·dir + phi0 where dir = ±1 (sense of rotation).
    let f_axis = f.z;
    let dir = if ci.frame.z.dot(&f_axis) > 0.0 {
        1.0
    } else {
        -1.0
    };
    let ex = ci.frame.x;
    let phi0 = ex.dot(&f.y).atan2(ex.dot(&f.x));
    let t0 = range.0;
    let u0 = wrap_near(phi0 + dir * t0, hint.map_or(phi0 + dir * t0, |h| h.0), TAU) - dir * t0;
    let v = (ci.frame.origin - f.origin).dot(&f_axis);
    Some(line2((u0, v), (dir, 0.0)))
}

fn axial_circle_pcurve_sphere(
    f: &Frame3,
    ci: &Circle,
    radius: f64,
    range: (f64, f64),
    hint: Option<(f64, f64)>,
) -> Option<CurveGeom> {
    let h = (ci.frame.origin - f.origin).dot(&f.z);
    let v = (h / radius).clamp(-1.0, 1.0).asin();
    let dir = if ci.frame.z.dot(&f.z) > 0.0 {
        1.0
    } else {
        -1.0
    };
    let phi0 = ci.frame.x.dot(&f.y).atan2(ci.frame.x.dot(&f.x));
    let t0 = range.0;
    let u0 = wrap_near(phi0 + dir * t0, hint.map_or(phi0 + dir * t0, |h| h.0), TAU) - dir * t0;
    Some(line2((u0, v), (dir, 0.0)))
}

/// Maximum deviation of `surface(pcurve(t))` from `curve(t)` over `range` (sampled).
pub fn pcurve_error(
    s: &SurfaceGeom,
    c: &CurveGeom,
    pc: &CurveGeom,
    range: (f64, f64),
    samples: usize,
) -> f64 {
    let mut worst: f64 = 0.0;
    for i in 0..=samples {
        let t = range.0 + (range.1 - range.0) * i as f64 / samples as f64;
        let q = pc.point_at(t);
        worst = worst.max((s.point_at(q.x, q.y) - c.point_at(t)).norm());
    }
    worst
}

/// Build the pcurve of `curve` over `range` on surface `s`. `hint` selects the periodic branch
/// at the start of the range. Returns `None` if the projection is ill-defined or the fit does
/// not reach `tol`.
pub fn pcurve_on(
    s: &SurfaceGeom,
    curve: &CurveGeom,
    range: (f64, f64),
    hint: Option<(f64, f64)>,
    tol: f64,
) -> Option<CurveGeom> {
    pcurve_on_anchored(s, curve, range, hint.map(|h| (range.0, h)), tol)
}

/// Like [`pcurve_on`], but the periodic branch is fixed by `anchor = (t, (u, v))`: the pcurve
/// passes (modulo periods) near `(u, v)` at parameter `t`, and is continuous elsewhere.
pub fn pcurve_on_anchored(
    s: &SurfaceGeom,
    curve: &CurveGeom,
    range: (f64, f64),
    anchor: Option<(f64, (f64, f64))>,
    tol: f64,
) -> Option<CurveGeom> {
    let hint = anchor
        .filter(|a| (a.0 - range.0).abs() < 1e-12 * (1.0 + range.0.abs()))
        .map(|a| a.1);
    if anchor.is_none() || hint.is_some() {
        if let Some(pc) = exact_pcurve(s, curve, range, hint) {
            if pcurve_error(s, curve, &pc, range, 16) <= tol {
                return Some(pc);
            }
        }
    } else if let Some((ta, uv)) = anchor {
        // exact forms are branch-fixed at range.0: evaluate the exact curve there and shift by periods
        let probe = exact_pcurve(s, curve, range, None);
        if let Some(pc) = probe {
            let q = pc.point_at(ta);
            let du = s.period_u().map_or(0.0, |p| p * ((uv.0 - q.x) / p).round());
            let dv = s.period_v().map_or(0.0, |p| p * ((uv.1 - q.y) / p).round());
            let pc = crate::fit::translate_pcurve(&pc, du, dv);
            if pcurve_error(s, curve, &pc, range, 16) <= tol {
                return Some(pc);
            }
        }
    }
    // coarse unwrapped lookup table so the fit can find the periodic branch at any t
    // closest-point queries on spline surfaces are expensive: a coarser branch table suffices there
    let n = if matches!(s, SurfaceGeom::Nurbs(_) | SurfaceGeom::Procedural(_)) {
        96usize
    } else {
        512usize
    };
    let (t0, t1) = range;
    let mut table: Vec<(f64, f64)> = vec![(0.0, 0.0); n + 1];
    let (a_idx, a_uv) = match anchor {
        Some((ta, uv)) => (
            (((ta - t0) / (t1 - t0) * n as f64).round() as usize).min(n),
            Some(uv),
        ),
        None => (0, None),
    };
    let mut last = a_uv;
    for i in a_idx..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let r = surface_param_of(s, &curve.point_at(t), last);
        table[i] = (r.u, r.v);
        last = Some((r.u, r.v));
    }
    let mut last = Some(table[a_idx]);
    for i in (0..a_idx).rev() {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let r = surface_param_of(s, &curve.point_at(t), last);
        table[i] = (r.u, r.v);
        last = Some((r.u, r.v));
    }
    let lookup = |t: f64| -> (f64, f64) {
        let x = ((t - t0) / (t1 - t0) * n as f64).clamp(0.0, n as f64);
        let i = (x.floor() as usize).min(n - 1);
        let w = x - i as f64;
        (
            table[i].0 * (1.0 - w) + table[i + 1].0 * w,
            table[i].1 * (1.0 - w) + table[i + 1].1 * w,
        )
    };
    let h = 1e-6 * (t1 - t0);
    let mut f = |t: f64| -> Option<(Point3<f64>, Vector3<f64>)> {
        let p = curve.point_at(t);
        let r = surface_param_of(s, &p, Some(lookup(t)));
        if r.distance > (1e-6 * (1.0 + p.coords.amax())).max(10.0 * tol) {
            return None;
        }
        let c1 = curve.derivative(t);
        let (du, dv) = match uv_velocity(s, r.u, r.v, &c1) {
            Some(d) => d,
            None => {
                let a = surface_param_of(s, &curve.point_at(t - h), Some((r.u, r.v)));
                let b = surface_param_of(s, &curve.point_at(t + h), Some((r.u, r.v)));
                ((b.u - a.u) / (2.0 * h), (b.v - a.v) / (2.0 * h))
            }
        };
        Some((xy(r.u, r.v), Vector3::new(du, dv, 0.0)))
    };
    // the fit tolerance is in uv units: convert the 3D tolerance with the largest surface speed
    // sampled along the curve
    let mut speed: f64 = 1e-3;
    for i in 0..=8 {
        let (u, v) = lookup(t0 + (t1 - t0) * i as f64 / 8.0);
        let p = s.point_at(u, v);
        let e = 1e-4;
        speed = speed
            .max((s.point_at(u + e, v) - p).norm() / e)
            .max((s.point_at(u, v + e) - p).norm() / e);
    }
    let fitted = hermite_fit(&mut f, t0, t1, tol * 0.1 / speed.max(1.0), 8, 20000)?;
    let pc = CurveGeom::Nurbs(fitted);
    (pcurve_error(s, curve, &pc, range, 64) <= tol).then_some(pc)
}

/// A degree-1 NURBS pcurve through the given uv points parameterised by `ts` (used when only
/// samples are available).
pub fn polyline_pcurve(ts: &[f64], uv: &[(f64, f64)]) -> Option<CurveGeom> {
    if ts.len() < 2 || ts.len() != uv.len() {
        return None;
    }
    let mut knots = vec![ts[0]];
    knots.extend_from_slice(ts);
    knots.push(*ts.last().unwrap());
    let kv = KnotVector::new(1, knots).ok()?;
    let pts: Vec<Point3<f64>> = uv.iter().map(|p| xy(p.0, p.1)).collect();
    NurbsCurve::new_polynomial(kv, pts)
        .ok()
        .map(CurveGeom::Nurbs)
}
