//! Curve fitting and rigid/affine re-placement of geometry.
//!
//! * [`hermite_fit`] builds an exact piecewise-cubic C¹ B-spline through
//!   points with given tangents, refining adaptively until it reproduces a
//!   reference function within a tolerance. Used for intersection curves and
//!   pcurves, which must share the edge parameterisation.
//! * [`transform_curve`] / [`transform_surface`] apply a rigid transform.

use crate::curve::NurbsCurve;
use crate::geom::*;
use crate::knots::KnotVector;
use crate::surface::NurbsSurface;
use kw_math::Transform3;
use nalgebra::{Point3, Vector3};

/// Piecewise cubic Hermite C¹ interpolant as a degree-3 B-spline (interior knots of
/// multiplicity 2). `ts` strictly increasing, `n ≥ 2`.
pub fn hermite_curve(
    ts: &[f64],
    pts: &[Point3<f64>],
    tangents: &[Vector3<f64>],
) -> Option<NurbsCurve> {
    let n = ts.len();
    if n < 2 || pts.len() != n || tangents.len() != n {
        return None;
    }
    let mut knots = vec![ts[0]; 4];
    for t in &ts[1..n - 1] {
        knots.push(*t);
        knots.push(*t);
    }
    knots.extend(std::iter::repeat_n(ts[n - 1], 4));
    let mut ctrl = Vec::with_capacity(2 * n);
    ctrl.push(pts[0]);
    for i in 0..n - 1 {
        let h = ts[i + 1] - ts[i];
        ctrl.push(pts[i] + tangents[i] * (h / 3.0));
        ctrl.push(pts[i + 1] - tangents[i + 1] * (h / 3.0));
    }
    ctrl.push(pts[n - 1]);
    let kv = KnotVector::new(3, knots).ok()?;
    NurbsCurve::new_polynomial(kv, ctrl).ok()
}

fn hermite_eval(
    t: f64,
    t0: f64,
    t1: f64,
    p0: &Point3<f64>,
    p1: &Point3<f64>,
    d0: &Vector3<f64>,
    d1: &Vector3<f64>,
) -> Point3<f64> {
    let h = t1 - t0;
    let s = (t - t0) / h;
    let (s2, s3) = (s * s, s * s * s);
    let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    let h10 = s3 - 2.0 * s2 + s;
    let h01 = -2.0 * s3 + 3.0 * s2;
    let h11 = s3 - s2;
    Point3::from(p0.coords * h00 + d0 * (h10 * h) + p1.coords * h01 + d1 * (h11 * h))
}

/// Adaptive Hermite fit of `f(t) → (point, derivative)` on `[t0, t1]`: segments are bisected
/// until the interpolant matches `f` at three interior probes to within `tol`.
/// `max_segments` bounds the work.
pub fn hermite_fit(
    f: &mut dyn FnMut(f64) -> Option<(Point3<f64>, Vector3<f64>)>,
    t0: f64,
    t1: f64,
    tol: f64,
    initial: usize,
    max_segments: usize,
) -> Option<NurbsCurve> {
    let initial = initial.max(1);
    let mut ts: Vec<f64> = (0..=initial)
        .map(|i| t0 + (t1 - t0) * i as f64 / initial as f64)
        .collect();
    let mut vals: Vec<(Point3<f64>, Vector3<f64>)> = Vec::new();
    for t in &ts {
        vals.push(f(*t)?);
    }
    let mut i = 0;
    while i + 1 < ts.len() {
        let (a, b) = (ts[i], ts[i + 1]);
        let mut bad = false;
        for k in [0.25, 0.5, 0.75] {
            let t = a + (b - a) * k;
            let (exact, _) = f(t)?;
            let approx = hermite_eval(
                t,
                a,
                b,
                &vals[i].0,
                &vals[i + 1].0,
                &vals[i].1,
                &vals[i + 1].1,
            );
            if (exact - approx).norm() > tol {
                bad = true;
                break;
            }
        }
        if bad && ts.len() < max_segments && (b - a) > 1e-12 {
            let m = 0.5 * (a + b);
            let v = f(m)?;
            ts.insert(i + 1, m);
            vals.insert(i + 1, v);
        } else if bad && ts.len() >= max_segments {
            return None;
        } else {
            i += 1;
        }
    }
    let pts: Vec<_> = vals.iter().map(|v| v.0).collect();
    let tans: Vec<_> = vals.iter().map(|v| v.1).collect();
    hermite_curve(&ts, &pts, &tans)
}

fn tp(t: &Transform3, p: &Point3<f64>) -> Point3<f64> {
    t.apply_point(p)
}

fn tframe(t: &Transform3, f: &kw_math::Frame3) -> kw_math::Frame3 {
    kw_math::Frame3 {
        origin: tp(t, &f.origin),
        x: t.apply_vector(&f.x),
        y: t.apply_vector(&f.y),
        z: t.apply_vector(&f.z),
    }
}

pub fn transform_nurbs_curve(c: &NurbsCurve, t: &Transform3) -> NurbsCurve {
    let pts = c.points().iter().map(|p| tp(t, p)).collect();
    NurbsCurve::new(c.knots().clone(), pts, c.weights().to_vec()).expect("same counts")
}

pub fn transform_nurbs_surface(s: &NurbsSurface, t: &Transform3) -> NurbsSurface {
    let pts = s.points().iter().map(|p| tp(t, p)).collect();
    NurbsSurface::new(
        s.knots_u().clone(),
        s.knots_v().clone(),
        pts,
        s.weights().to_vec(),
    )
    .expect("same counts")
}

/// Rigidly transform a curve.
pub fn transform_curve(c: &CurveGeom, t: &Transform3) -> CurveGeom {
    match c {
        CurveGeom::Line(l) => CurveGeom::Line(Line {
            origin: tp(t, &l.origin),
            dir: t.apply_vector(&l.dir),
        }),
        CurveGeom::Circle(ci) => CurveGeom::Circle(Circle {
            frame: tframe(t, &ci.frame),
            radius: ci.radius,
        }),
        CurveGeom::Ellipse(e) => CurveGeom::Ellipse(Ellipse {
            frame: tframe(t, &e.frame),
            major: e.major,
            minor: e.minor,
        }),
        CurveGeom::Nurbs(n) => CurveGeom::Nurbs(transform_nurbs_curve(n, t)),
        CurveGeom::Procedural(p) => CurveGeom::Procedural(Box::new(ProceduralCurve {
            kind: p.kind.clone(),
            approximation: transform_nurbs_curve(&p.approximation, t),
            tolerance: p.tolerance,
        })),
    }
}

/// Rigidly transform a surface.
pub fn transform_surface(s: &SurfaceGeom, t: &Transform3) -> SurfaceGeom {
    match s {
        SurfaceGeom::Plane(p) => SurfaceGeom::Plane(Plane {
            frame: tframe(t, &p.frame),
        }),
        SurfaceGeom::Cylinder(c) => SurfaceGeom::Cylinder(Cylinder {
            frame: tframe(t, &c.frame),
            radius: c.radius,
        }),
        SurfaceGeom::Cone(c) => SurfaceGeom::Cone(Cone {
            frame: tframe(t, &c.frame),
            radius_at_origin: c.radius_at_origin,
            half_angle: c.half_angle,
        }),
        SurfaceGeom::Sphere(c) => SurfaceGeom::Sphere(Sphere {
            frame: tframe(t, &c.frame),
            radius: c.radius,
        }),
        SurfaceGeom::Torus(c) => SurfaceGeom::Torus(Torus {
            frame: tframe(t, &c.frame),
            major_radius: c.major_radius,
            minor_radius: c.minor_radius,
        }),
        SurfaceGeom::Nurbs(n) => SurfaceGeom::Nurbs(transform_nurbs_surface(n, t)),
        SurfaceGeom::Extrusion { profile, dir } => SurfaceGeom::Extrusion {
            profile: Box::new(transform_curve(profile, t)),
            dir: t.apply_vector(dir),
        },
        SurfaceGeom::Revolution {
            profile,
            axis_origin,
            axis_dir,
        } => SurfaceGeom::Revolution {
            profile: Box::new(transform_curve(profile, t)),
            axis_origin: tp(t, axis_origin),
            axis_dir: t.apply_vector(axis_dir),
        },
        SurfaceGeom::Procedural(p) => SurfaceGeom::Procedural(Box::new(ProceduralSurface {
            kind: p.kind.clone(),
            approximation: transform_nurbs_surface(&p.approximation, t),
            tolerance: p.tolerance,
        })),
    }
}

/// Translate a curve that lives in the `z = 0` parameter plane (a pcurve) by `(du, dv)`.
pub fn translate_pcurve(c: &CurveGeom, du: f64, dv: f64) -> CurveGeom {
    let d = Vector3::new(du, dv, 0.0);
    match c {
        CurveGeom::Line(l) => CurveGeom::Line(Line {
            origin: l.origin + d,
            dir: l.dir,
        }),
        CurveGeom::Circle(ci) => {
            let mut f = ci.frame;
            f.origin += d;
            CurveGeom::Circle(Circle {
                frame: f,
                radius: ci.radius,
            })
        }
        CurveGeom::Ellipse(e) => {
            let mut f = e.frame;
            f.origin += d;
            CurveGeom::Ellipse(Ellipse {
                frame: f,
                major: e.major,
                minor: e.minor,
            })
        }
        CurveGeom::Nurbs(n) => {
            let pts = n.points().iter().map(|p| p + d).collect();
            CurveGeom::Nurbs(
                NurbsCurve::new(n.knots().clone(), pts, n.weights().to_vec()).expect("counts"),
            )
        }
        CurveGeom::Procedural(p) => {
            translate_pcurve(&CurveGeom::Nurbs(p.approximation.clone()), du, dv)
        }
    }
}

/// Exact surface offset by signed distance `d` along the surface normal, when the offset is
/// again an analytic surface of the same family.
pub fn offset_surface(s: &SurfaceGeom, d: f64) -> Option<SurfaceGeom> {
    match s {
        SurfaceGeom::Plane(p) => {
            let mut f = p.frame;
            f.origin += f.z * d;
            Some(SurfaceGeom::Plane(Plane { frame: f }))
        }
        SurfaceGeom::Cylinder(c) => {
            let r = c.radius + d;
            (r > 0.0).then_some(SurfaceGeom::Cylinder(Cylinder {
                frame: c.frame,
                radius: r,
            }))
        }
        SurfaceGeom::Cone(c) => {
            // normal (cos a · radial − sin a · axis); offsetting by d moves radius_at_origin by d / cos a
            let r = c.radius_at_origin + d / c.half_angle.cos();
            Some(SurfaceGeom::Cone(Cone {
                frame: c.frame,
                radius_at_origin: r,
                half_angle: c.half_angle,
            }))
        }
        SurfaceGeom::Sphere(c) => {
            let r = c.radius + d;
            (r > 0.0).then_some(SurfaceGeom::Sphere(Sphere {
                frame: c.frame,
                radius: r,
            }))
        }
        SurfaceGeom::Torus(c) => {
            let r = c.minor_radius + d;
            (r > 0.0).then_some(SurfaceGeom::Torus(Torus {
                frame: c.frame,
                major_radius: c.major_radius,
                minor_radius: r,
            }))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------------- uniform scaling

fn sp(p: &Point3<f64>, k: f64, c: &Point3<f64>) -> Point3<f64> {
    c + (p - c) * k
}

fn sframe(f: &kw_math::Frame3, k: f64, c: &Point3<f64>) -> kw_math::Frame3 {
    kw_math::Frame3 {
        origin: sp(&f.origin, k, c),
        ..*f
    }
}

fn scale_nurbs_curve(n: &NurbsCurve, k: f64, c: &Point3<f64>) -> NurbsCurve {
    let pts = n.points().iter().map(|p| sp(p, k, c)).collect();
    NurbsCurve::new(n.knots().clone(), pts, n.weights().to_vec()).expect("same counts")
}

fn scale_nurbs_surface(s: &NurbsSurface, k: f64, c: &Point3<f64>) -> NurbsSurface {
    let pts = s.points().iter().map(|p| sp(p, k, c)).collect();
    NurbsSurface::new(
        s.knots_u().clone(),
        s.knots_v().clone(),
        pts,
        s.weights().to_vec(),
    )
    .expect("same counts")
}

/// Scale a curve by `k > 0` about `centre`. Curve parameters keep their meaning (angles stay angles, a
/// line's direction vector is scaled), so edge parameter ranges stay valid.
pub fn scale_curve(c: &CurveGeom, k: f64, centre: &Point3<f64>) -> CurveGeom {
    match c {
        CurveGeom::Line(l) => CurveGeom::Line(Line {
            origin: sp(&l.origin, k, centre),
            dir: l.dir * k,
        }),
        CurveGeom::Circle(ci) => CurveGeom::Circle(Circle {
            frame: sframe(&ci.frame, k, centre),
            radius: ci.radius * k,
        }),
        CurveGeom::Ellipse(e) => CurveGeom::Ellipse(Ellipse {
            frame: sframe(&e.frame, k, centre),
            major: e.major * k,
            minor: e.minor * k,
        }),
        CurveGeom::Nurbs(n) => CurveGeom::Nurbs(scale_nurbs_curve(n, k, centre)),
        CurveGeom::Procedural(p) => CurveGeom::Procedural(Box::new(ProceduralCurve {
            kind: p.kind.clone(),
            approximation: scale_nurbs_curve(&p.approximation, k, centre),
            tolerance: p.tolerance * k,
        })),
    }
}

/// Scale a surface by `k > 0` about `centre`. Angular parameters are unchanged; the length parameters
/// of a plane (`u`, `v`), a cylinder and a cone (`v`) are multiplied by `k` — see [`scale_pcurve`].
pub fn scale_surface(s: &SurfaceGeom, k: f64, centre: &Point3<f64>) -> SurfaceGeom {
    match s {
        SurfaceGeom::Plane(p) => SurfaceGeom::Plane(Plane {
            frame: sframe(&p.frame, k, centre),
        }),
        SurfaceGeom::Cylinder(c) => SurfaceGeom::Cylinder(Cylinder {
            frame: sframe(&c.frame, k, centre),
            radius: c.radius * k,
        }),
        SurfaceGeom::Cone(c) => SurfaceGeom::Cone(Cone {
            frame: sframe(&c.frame, k, centre),
            radius_at_origin: c.radius_at_origin * k,
            half_angle: c.half_angle,
        }),
        SurfaceGeom::Sphere(c) => SurfaceGeom::Sphere(Sphere {
            frame: sframe(&c.frame, k, centre),
            radius: c.radius * k,
        }),
        SurfaceGeom::Torus(c) => SurfaceGeom::Torus(Torus {
            frame: sframe(&c.frame, k, centre),
            major_radius: c.major_radius * k,
            minor_radius: c.minor_radius * k,
        }),
        SurfaceGeom::Nurbs(n) => SurfaceGeom::Nurbs(scale_nurbs_surface(n, k, centre)),
        SurfaceGeom::Extrusion { profile, dir } => SurfaceGeom::Extrusion {
            profile: Box::new(scale_curve(profile, k, centre)),
            dir: dir * k,
        },
        SurfaceGeom::Revolution {
            profile,
            axis_origin,
            axis_dir,
        } => SurfaceGeom::Revolution {
            profile: Box::new(scale_curve(profile, k, centre)),
            axis_origin: sp(axis_origin, k, centre),
            axis_dir: *axis_dir,
        },
        SurfaceGeom::Procedural(p) => SurfaceGeom::Procedural(Box::new(ProceduralSurface {
            kind: match &p.kind {
                ProceduralSurfaceKind::Offset { distance } => ProceduralSurfaceKind::Offset {
                    distance: distance * k,
                },
                other => other.clone(),
            },
            approximation: scale_nurbs_surface(&p.approximation, k, centre),
            tolerance: p.tolerance * k,
        })),
    }
}

/// How a surface's `(u, v)` parameters change under [`scale_surface`] with factor `k`.
pub fn scale_param_factors(s: &SurfaceGeom, k: f64) -> (f64, f64) {
    match s {
        SurfaceGeom::Plane(_) => (k, k),
        SurfaceGeom::Cylinder(_) | SurfaceGeom::Cone(_) => (1.0, k),
        _ => (1.0, 1.0),
    }
}

/// Scale a pcurve (a curve in the `z = 0` parameter plane) by `(su, sv)`. A circle or ellipse only stays
/// one when `su == sv`; otherwise `None`.
pub fn scale_pcurve(c: &CurveGeom, su: f64, sv: f64) -> Option<CurveGeom> {
    if su == 1.0 && sv == 1.0 {
        return Some(c.clone());
    }
    let m = |p: &Point3<f64>| Point3::new(p.x * su, p.y * sv, p.z);
    let mv = |v: &Vector3<f64>| Vector3::new(v.x * su, v.y * sv, v.z);
    Some(match c {
        CurveGeom::Line(l) => CurveGeom::Line(Line {
            origin: m(&l.origin),
            dir: mv(&l.dir),
        }),
        CurveGeom::Circle(ci) if su == sv => {
            let mut f = ci.frame;
            f.origin = m(&f.origin);
            CurveGeom::Circle(Circle {
                frame: f,
                radius: ci.radius * su,
            })
        }
        CurveGeom::Ellipse(e) if su == sv => {
            let mut f = e.frame;
            f.origin = m(&f.origin);
            CurveGeom::Ellipse(Ellipse {
                frame: f,
                major: e.major * su,
                minor: e.minor * su,
            })
        }
        CurveGeom::Nurbs(n) => {
            let pts = n.points().iter().map(m).collect();
            CurveGeom::Nurbs(NurbsCurve::new(n.knots().clone(), pts, n.weights().to_vec()).ok()?)
        }
        CurveGeom::Procedural(p) => {
            let pts = p.approximation.points().iter().map(m).collect();
            CurveGeom::Procedural(Box::new(ProceduralCurve {
                kind: p.kind.clone(),
                approximation: NurbsCurve::new(
                    p.approximation.knots().clone(),
                    pts,
                    p.approximation.weights().to_vec(),
                )
                .ok()?,
                tolerance: p.tolerance * su.max(sv),
            }))
        }
        _ => return None,
    })
}
