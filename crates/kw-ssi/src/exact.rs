//! Closed-form intersections of analytic surfaces (the ~80 % of mechanical cases),
//! returning exact lines, circles and ellipses.

use kw_math::Frame3;
use kw_nurbs::geom::*;
use nalgebra::{Point3, Vector3};
use std::f64::consts::TAU;

#[derive(Debug, Clone)]
pub struct RawCurve {
    pub curve: CurveGeom,
    pub range: (f64, f64),
    pub closed: bool,
}

#[derive(Debug, Clone, Default)]
pub struct Exact {
    pub curves: Vec<RawCurve>,
    pub points: Vec<Point3<f64>>,
    pub tangent: bool,
    pub coincident: Option<bool>,
}

fn scale_of(s: &SurfaceGeom) -> f64 {
    match s {
        SurfaceGeom::Plane(p) => p.frame.origin.coords.amax(),
        SurfaceGeom::Cylinder(c) => c.frame.origin.coords.amax().max(c.radius),
        SurfaceGeom::Cone(c) => c.frame.origin.coords.amax().max(c.radius_at_origin),
        SurfaceGeom::Sphere(c) => c.frame.origin.coords.amax().max(c.radius),
        SurfaceGeom::Torus(c) => c.frame.origin.coords.amax().max(c.major_radius),
        _ => 1.0,
    }
}

fn perp_frame(origin: Point3<f64>, axis: Vector3<f64>) -> Frame3 {
    Frame3::from_origin_normal(origin, axis)
}

fn circle(origin: Point3<f64>, axis: Vector3<f64>, radius: f64) -> RawCurve {
    RawCurve {
        curve: CurveGeom::Circle(Circle {
            frame: perp_frame(origin, axis),
            radius,
        }),
        range: (0.0, TAU),
        closed: true,
    }
}

// ------------------------------------------------------------------- revolution family

#[derive(Debug, Clone, Copy)]
enum Prof {
    /// z = c
    H(f64),
    /// ρ = r
    V(f64),
    /// ρ = r0 + m·z
    S { r0: f64, m: f64 },
    /// (z − zc)² + (ρ − rc)² = rad²
    C { zc: f64, rc: f64, rad: f64 },
}

#[derive(Debug, Clone, Copy)]
struct Rev {
    o: Point3<f64>,
    d: Vector3<f64>,
    prof: Prof,
}

fn rev_of(s: &SurfaceGeom) -> Option<Rev> {
    Some(match s {
        SurfaceGeom::Plane(p) => Rev {
            o: p.frame.origin,
            d: p.frame.z,
            prof: Prof::H(0.0),
        },
        SurfaceGeom::Cylinder(c) => Rev {
            o: c.frame.origin,
            d: c.frame.z,
            prof: Prof::V(c.radius),
        },
        SurfaceGeom::Cone(c) => Rev {
            o: c.frame.origin,
            d: c.frame.z,
            prof: Prof::S {
                r0: c.radius_at_origin,
                m: c.half_angle.tan(),
            },
        },
        SurfaceGeom::Sphere(c) => Rev {
            o: c.frame.origin,
            d: c.frame.z,
            prof: Prof::C {
                zc: 0.0,
                rc: 0.0,
                rad: c.radius,
            },
        },
        SurfaceGeom::Torus(c) => Rev {
            o: c.frame.origin,
            d: c.frame.z,
            prof: Prof::C {
                zc: 0.0,
                rc: c.major_radius,
                rad: c.minor_radius,
            },
        },
        _ => return None,
    })
}

/// Express `b`'s profile in `a`'s axial coordinate; `None` if the axes differ.
fn coaxial(a: &Rev, b: &Rev, eps: f64) -> Option<Prof> {
    let sgn = a.d.dot(&b.d);
    if a.d.cross(&b.d).norm() > 1e-10 {
        return None;
    }
    let w = b.o - a.o;
    let along = w.dot(&a.d);
    if (w - a.d * along).norm() > eps {
        return None;
    }
    let sgn = sgn.signum();
    Some(match b.prof {
        Prof::H(c) => Prof::H(along + sgn * c),
        Prof::V(r) => Prof::V(r),
        Prof::S { r0, m } => Prof::S {
            r0: r0 - m * sgn * along,
            m: m * sgn,
        },
        Prof::C { zc, rc, rad } => Prof::C {
            zc: along + sgn * zc,
            rc,
            rad,
        },
    })
}

/// 2-D intersections of two profiles in the (z, ρ) half-plane. `(z, ρ, tangent)`.
fn prof_intersections(a: Prof, b: Prof, eps: f64) -> Vec<(f64, f64, bool)> {
    use Prof::*;
    let mut out = Vec::new();
    let mut push = |z: f64, r: f64, t: bool| out.push((z, r, t));
    let quad_roots = |disc: f64, scale: f64| -> Vec<(f64, bool)> {
        // returns signs for ±sqrt; tangent when disc ≈ 0
        if disc < -eps * scale.max(1.0) * scale.max(1.0) {
            vec![]
        } else if disc.abs() <= eps * scale.max(1.0) * scale.max(1.0) {
            vec![(0.0, true)]
        } else {
            vec![(disc.sqrt(), false), (-disc.sqrt(), false)]
        }
    };
    match (a, b) {
        (H(c), V(r)) | (V(r), H(c)) => push(c, r, false),
        (H(c), S { r0, m }) | (S { r0, m }, H(c)) => push(c, r0 + m * c, false),
        (H(c), C { zc, rc, rad }) | (C { zc, rc, rad }, H(c)) => {
            for (s, t) in quad_roots(rad * rad - (c - zc).powi(2), rad) {
                push(c, rc + s, t);
            }
        }
        (V(r), S { r0, m }) | (S { r0, m }, V(r)) => {
            if m.abs() > 1e-14 {
                push((r - r0) / m, r, false);
            }
        }
        (V(r), C { zc, rc, rad }) | (C { zc, rc, rad }, V(r)) => {
            for (s, t) in quad_roots(rad * rad - (r - rc).powi(2), rad) {
                push(zc + s, r, t);
            }
        }
        (S { r0: r1, m: m1 }, S { r0: r2, m: m2 }) => {
            if (m1 - m2).abs() > 1e-14 {
                let z = (r2 - r1) / (m1 - m2);
                push(z, r1 + m1 * z, false);
            }
        }
        (S { r0, m }, C { zc, rc, rad }) | (C { zc, rc, rad }, S { r0, m }) => {
            // (z − zc)² + (r0 + m z − rc)² = rad²
            let k = r0 - rc;
            let qa = 1.0 + m * m;
            let qb = -2.0 * zc + 2.0 * m * k;
            let qc = zc * zc + k * k - rad * rad;
            let disc = qb * qb - 4.0 * qa * qc;
            let sc = rad.max(1.0);
            if disc < -eps * sc * sc {
            } else if disc.abs() <= eps * sc * sc {
                let z = -qb / (2.0 * qa);
                push(z, r0 + m * z, true);
            } else {
                for sg in [1.0, -1.0] {
                    let z = (-qb + sg * disc.sqrt()) / (2.0 * qa);
                    push(z, r0 + m * z, false);
                }
            }
        }
        (
            C {
                zc: z1,
                rc: r1,
                rad: a1,
            },
            C {
                zc: z2,
                rc: r2,
                rad: a2,
            },
        ) => {
            let (dz, dr) = (z2 - z1, r2 - r1);
            let d = dz.hypot(dr);
            if d < 1e-14 {
                return out;
            }
            let x = (d * d + a1 * a1 - a2 * a2) / (2.0 * d);
            let h2 = a1 * a1 - x * x;
            let sc = a1.max(a2).max(1.0);
            let (ex, er) = (dz / d, dr / d);
            if h2 < -eps * sc * sc {
            } else if h2.abs() <= eps * sc * sc {
                push(z1 + ex * x, r1 + er * x, true);
            } else {
                let h = h2.sqrt();
                for sg in [1.0, -1.0] {
                    push(z1 + ex * x - er * h * sg, r1 + er * x + ex * h * sg, false);
                }
            }
        }
        _ => {}
    }
    out
}

fn same_profile(a: Prof, b: Prof, eps: f64) -> bool {
    use Prof::*;
    match (a, b) {
        (H(x), H(y)) | (V(x), V(y)) => (x - y).abs() < eps,
        (S { r0: a0, m: am }, S { r0: b0, m: bm }) => {
            (a0 - b0).abs() < eps && (am - bm).abs() < 1e-10
        }
        (
            C {
                zc: z1,
                rc: r1,
                rad: a1,
            },
            C {
                zc: z2,
                rc: r2,
                rad: a2,
            },
        ) => (z1 - z2).abs() < eps && (r1 - r2).abs() < eps && (a1 - a2).abs() < eps,
        _ => false,
    }
}

// ------------------------------------------------------------------- pair solvers

/// Plane / plane.
fn plane_plane(a: &Plane, b: &Plane, eps: f64) -> Exact {
    let (n1, n2) = (a.frame.z, b.frame.z);
    let d = n1.cross(&n2);
    let dn = d.norm();
    if dn < 1e-10 {
        let dist = (b.frame.origin - a.frame.origin).dot(&n1);
        if dist.abs() < eps {
            return Exact {
                coincident: Some(n1.dot(&n2) > 0.0),
                ..Default::default()
            };
        }
        return Exact::default();
    }
    let d = d / dn;
    // point on both planes: o = α n1 + β n2 with n1·o = h1, n2·o = h2
    let h1 = n1.dot(&a.frame.origin.coords);
    let h2 = n2.dot(&b.frame.origin.coords);
    let c = n1.dot(&n2);
    let det = 1.0 - c * c;
    let alpha = (h1 - h2 * c) / det;
    let beta = (h2 - h1 * c) / det;
    let o = Point3::from(n1 * alpha + n2 * beta);
    Exact {
        curves: vec![RawCurve {
            curve: CurveGeom::Line(Line { origin: o, dir: d }),
            range: (f64::NEG_INFINITY, f64::INFINITY),
            closed: false,
        }],
        ..Default::default()
    }
}

fn plane_cylinder(p: &Plane, c: &Cylinder, eps: f64) -> Exact {
    let n = p.frame.z;
    let d = c.frame.z;
    let r = c.radius;
    let nd = n.dot(&d);
    if nd.abs() < 1e-10 {
        // axis parallel to the plane: lines
        let delta = (c.frame.origin - p.frame.origin).dot(&n); // signed distance of axis from plane
        let p0 = c.frame.origin - n * delta; // axis point projected onto the plane
        let w = n.cross(&d);
        let w = w / w.norm();
        if delta.abs() > r + eps {
            return Exact::default();
        }
        let h2 = r * r - delta * delta;
        if h2.abs() <= eps * r.max(1.0) {
            return Exact {
                curves: vec![RawCurve {
                    curve: CurveGeom::Line(Line { origin: p0, dir: d }),
                    range: (f64::NEG_INFINITY, f64::INFINITY),
                    closed: false,
                }],
                tangent: true,
                ..Default::default()
            };
        }
        let h = h2.max(0.0).sqrt();
        return Exact {
            curves: [1.0, -1.0]
                .iter()
                .map(|s| RawCurve {
                    curve: CurveGeom::Line(Line {
                        origin: p0 + w * (h * s),
                        dir: d,
                    }),
                    range: (f64::NEG_INFINITY, f64::INFINITY),
                    closed: false,
                })
                .collect(),
            ..Default::default()
        };
    }
    // axis ∩ plane
    let t = (p.frame.origin - c.frame.origin).dot(&n) / nd;
    let ctr = c.frame.origin + d * t;
    if (nd.abs() - 1.0).abs() < 1e-12 {
        return Exact {
            curves: vec![circle(ctr, d, r)],
            ..Default::default()
        };
    }
    let m = d.cross(&n);
    let m = m / m.norm();
    let major_dir = n.cross(&m);
    let major_dir = major_dir / major_dir.norm();
    let a = r / nd.abs();
    let frame = Frame3 {
        origin: ctr,
        x: major_dir,
        y: m,
        z: major_dir.cross(&m),
    };
    Exact {
        curves: vec![RawCurve {
            curve: CurveGeom::Ellipse(Ellipse {
                frame,
                major: a,
                minor: r,
            }),
            range: (0.0, TAU),
            closed: true,
        }],
        ..Default::default()
    }
}

fn plane_sphere(p: &Plane, s: &Sphere, eps: f64) -> Exact {
    let n = p.frame.z;
    let delta = (s.frame.origin - p.frame.origin).dot(&n);
    if delta.abs() > s.radius + eps {
        return Exact::default();
    }
    let ctr = s.frame.origin - n * delta;
    let h2 = s.radius * s.radius - delta * delta;
    if h2 <= eps * s.radius.max(1.0) {
        return Exact {
            points: vec![ctr],
            tangent: true,
            ..Default::default()
        };
    }
    Exact {
        curves: vec![circle(ctr, n, h2.sqrt())],
        ..Default::default()
    }
}

fn sphere_sphere(a: &Sphere, b: &Sphere, eps: f64) -> Exact {
    let w = b.frame.origin - a.frame.origin;
    let d = w.norm();
    if d < eps {
        return if (a.radius - b.radius).abs() < eps {
            Exact {
                coincident: Some(true),
                ..Default::default()
            }
        } else {
            Exact::default()
        };
    }
    let x = (d * d + a.radius * a.radius - b.radius * b.radius) / (2.0 * d);
    let h2 = a.radius * a.radius - x * x;
    let sc = a.radius.max(b.radius).max(1.0);
    if h2 < -eps * sc {
        return Exact::default();
    }
    let ax = w / d;
    let ctr = a.frame.origin + ax * x;
    if h2 <= eps * sc {
        return Exact {
            points: vec![ctr],
            tangent: true,
            ..Default::default()
        };
    }
    Exact {
        curves: vec![circle(ctr, ax, h2.sqrt())],
        ..Default::default()
    }
}

fn cylinder_cylinder_parallel(a: &Cylinder, b: &Cylinder, eps: f64) -> Option<Exact> {
    if a.frame.z.cross(&b.frame.z).norm() > 1e-10 {
        return None;
    }
    let d = a.frame.z;
    let w = b.frame.origin - a.frame.origin;
    let w = w - d * w.dot(&d);
    let dist = w.norm();
    let (r1, r2) = (a.radius, b.radius);
    if dist < eps {
        return Some(if (r1 - r2).abs() < eps {
            Exact {
                coincident: Some(true),
                ..Default::default()
            }
        } else {
            Exact::default()
        });
    }
    let ex = w / dist;
    let ey = d.cross(&ex);
    let x = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
    let h2 = r1 * r1 - x * x;
    let sc = r1.max(r2).max(1.0);
    if h2 < -eps * sc {
        return Some(Exact::default());
    }
    let base = a.frame.origin;
    let line = |o: Point3<f64>| RawCurve {
        curve: CurveGeom::Line(Line { origin: o, dir: d }),
        range: (f64::NEG_INFINITY, f64::INFINITY),
        closed: false,
    };
    if h2 <= eps * sc {
        return Some(Exact {
            curves: vec![line(base + ex * x)],
            tangent: true,
            ..Default::default()
        });
    }
    let h = h2.sqrt();
    Some(Exact {
        curves: vec![line(base + ex * x + ey * h), line(base + ex * x - ey * h)],
        ..Default::default()
    })
}

/// Two cylinders of equal radius whose axes intersect: the intersection lies in the two planes through
/// the axes' crossing point with normals `d1 − d2` and `d1 + d2` (two ellipses crossing at singular points).
fn cylinder_cylinder_equal_crossing(a: &Cylinder, b: &Cylinder, eps: f64) -> Option<Exact> {
    if (a.radius - b.radius).abs() > eps {
        return None;
    }
    let (d1, d2) = (a.frame.z, b.frame.z);
    let cr = d1.cross(&d2);
    let cn = cr.norm();
    if cn < 1e-9 {
        return None;
    }
    // closest points of the two axes
    let w = a.frame.origin - b.frame.origin;
    let (bb, c, d, e) = (d1.dot(&d2), d2.dot(&d2), d1.dot(&w), d2.dot(&w));
    let det = 1.0 - bb * bb;
    let s = (bb * e - c * d) / det;
    let t = (e - bb * d) / det;
    let (p1, p2) = (a.frame.origin + d1 * s, b.frame.origin + d2 * t);
    if (p1 - p2).norm() > eps * 10.0 {
        return None;
    }
    let o = nalgebra::center(&p1, &p2);
    let mut curves = Vec::new();
    for n in [d1 - d2, d1 + d2] {
        if n.norm() < 1e-9 {
            continue;
        }
        let plane = Plane {
            frame: Frame3::from_origin_normal(o, n),
        };
        curves.extend(plane_cylinder(&plane, a, eps).curves);
    }
    Some(Exact {
        curves,
        ..Default::default()
    })
}

/// Coincidence of two analytic surfaces: `Some(same_normal)`.
pub fn coincident(a: &SurfaceGeom, b: &SurfaceGeom) -> Option<bool> {
    let eps = 1e-9 * (1.0 + scale_of(a).max(scale_of(b)));
    match (a, b) {
        (SurfaceGeom::Plane(p), SurfaceGeom::Plane(q)) => {
            if p.frame.z.cross(&q.frame.z).norm() < 1e-10
                && (q.frame.origin - p.frame.origin).dot(&p.frame.z).abs() < eps
            {
                Some(p.frame.z.dot(&q.frame.z) > 0.0)
            } else {
                None
            }
        }
        _ => {
            let (ra, rb) = (rev_of(a)?, rev_of(b)?);
            if std::mem::discriminant(a) != std::mem::discriminant(b) {
                return None;
            }
            // cones: same-handed axes and identical profile; spheres: any axis
            if let (SurfaceGeom::Sphere(s1), SurfaceGeom::Sphere(s2)) = (a, b) {
                return ((s1.frame.origin - s2.frame.origin).norm() < eps
                    && (s1.radius - s2.radius).abs() < eps)
                    .then_some(true);
            }
            let pb = coaxial(&ra, &rb, eps)?;
            same_profile(ra.prof, pb, eps).then_some(true)
        }
    }
}

/// Exact intersection when a closed form exists for this pair.
pub fn exact(a: &SurfaceGeom, b: &SurfaceGeom) -> Option<Exact> {
    let eps = 1e-9 * (1.0 + scale_of(a).max(scale_of(b)));
    if let Some(same) = coincident(a, b) {
        return Some(Exact {
            coincident: Some(same),
            ..Default::default()
        });
    }
    use SurfaceGeom::*;
    match (a, b) {
        (Plane(p), Plane(q)) => return Some(plane_plane(p, q, eps)),
        (Plane(p), Cylinder(c)) | (Cylinder(c), Plane(p)) => {
            return Some(plane_cylinder(p, c, eps))
        }
        (Plane(p), Sphere(s)) | (Sphere(s), Plane(p)) => return Some(plane_sphere(p, s, eps)),
        (Sphere(s), Sphere(t)) => return Some(sphere_sphere(s, t, eps)),
        (Cylinder(c), Cylinder(d)) => {
            if let Some(e) = cylinder_cylinder_parallel(c, d, eps) {
                return Some(e);
            }
            if let Some(e) = cylinder_cylinder_equal_crossing(c, d, eps) {
                return Some(e);
            }
        }
        _ => {}
    }
    // coaxial surfaces of revolution (plane ⟂ axis, cylinder, cone, sphere centred on the axis, torus)
    let (ra, rb) = (rev_of(a)?, rev_of(b)?);
    // a plane can use any parallel axis: re-seat its axis onto the other surface's axis
    let (ra, rb) = match (a, b) {
        (Plane(p), _) => (
            Rev {
                o: rb.o,
                d: p.frame.z,
                prof: Prof::H((p.frame.origin - rb.o).dot(&p.frame.z)),
            },
            rb,
        ),
        (_, Plane(p)) => (
            ra,
            Rev {
                o: ra.o,
                d: p.frame.z,
                prof: Prof::H((p.frame.origin - ra.o).dot(&p.frame.z)),
            },
        ),
        // a sphere has no preferred axis: use the other surface's
        (Sphere(_), _) => (Rev { d: rb.d, ..ra }, rb),
        (_, Sphere(_)) => (ra, Rev { d: ra.d, ..rb }),
        _ => (ra, rb),
    };
    let pb = coaxial(&ra, &rb, eps)?;
    let mut out = Exact::default();
    let ts = prof_intersections(ra.prof, pb, 1e-10);
    for (z, rho, tan) in ts {
        let ctr = ra.o + ra.d * z;
        if rho < 1e-12 {
            out.points.push(ctr);
            out.tangent = true;
        } else {
            out.curves.push(circle(ctr, ra.d, rho));
            out.tangent |= tan;
        }
    }
    Some(out)
}
