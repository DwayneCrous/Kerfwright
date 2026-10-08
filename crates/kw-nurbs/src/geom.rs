//! Analytic and procedural curve/surface definitions with analytic first
//! derivatives and normals (needed by the tessellator and renderer).

use crate::curve::NurbsCurve;
use crate::surface::NurbsSurface;
use kw_math::Frame3;
use nalgebra::{Point3, Rotation3, Unit, Vector3};
use std::f64::consts::{FRAC_PI_2, TAU};

/// Parametric 3D curve.
pub trait Curve3 {
    fn domain(&self) -> (f64, f64);
    fn point_at(&self, t: f64) -> Point3<f64>;
    /// First derivative with respect to the parameter.
    fn derivative(&self, t: f64) -> Vector3<f64>;
}

/// Parametric surface `S(u, v)`.
pub trait Surface {
    fn domain(&self) -> ((f64, f64), (f64, f64));
    fn point_at(&self, u: f64, v: f64) -> Point3<f64>;
    /// `(∂S/∂u, ∂S/∂v)`.
    fn derivatives(&self, u: f64, v: f64) -> (Vector3<f64>, Vector3<f64>);
    /// Unit normal in the direction of `Su × Sv`; well-defined at poles/apexes
    /// for the analytic types.
    fn normal_at(&self, u: f64, v: f64) -> Vector3<f64> {
        let (su, sv) = self.derivatives(u, v);
        let n = su.cross(&sv);
        let l = n.norm();
        if l > 1e-300 {
            n / l
        } else {
            Vector3::z()
        }
    }
    fn period_u(&self) -> Option<f64> {
        None
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub origin: Point3<f64>,
    /// Full direction vector; the natural parameter range is usually [0, 1]
    /// for an edge between its end vertices.
    pub dir: Vector3<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Circle {
    pub frame: Frame3,
    pub radius: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Ellipse {
    pub frame: Frame3,
    pub major: f64,
    pub minor: f64,
}

/// What produced a procedural curve; the cached `approximation` is what
/// downstream code evaluates, within `tolerance`.
#[derive(Debug, Clone, PartialEq)]
pub enum ProceduralCurveKind {
    SurfaceIntersection,
    Offset,
    Projection,
    BlendSpine,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ProceduralCurve {
    pub kind: ProceduralCurveKind,
    pub approximation: NurbsCurve,
    pub tolerance: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum CurveGeom {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
    Nurbs(NurbsCurve),
    Procedural(Box<ProceduralCurve>),
}

impl Curve3 for CurveGeom {
    fn domain(&self) -> (f64, f64) {
        match self {
            CurveGeom::Line(_) => (f64::NEG_INFINITY, f64::INFINITY),
            CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => (0.0, TAU),
            CurveGeom::Nurbs(c) => c.domain(),
            CurveGeom::Procedural(p) => p.approximation.domain(),
        }
    }

    fn point_at(&self, t: f64) -> Point3<f64> {
        match self {
            CurveGeom::Line(l) => l.origin + l.dir * t,
            CurveGeom::Circle(c) => {
                c.frame.origin + (c.frame.x * t.cos() + c.frame.y * t.sin()) * c.radius
            }
            CurveGeom::Ellipse(e) => {
                e.frame.origin + e.frame.x * (e.major * t.cos()) + e.frame.y * (e.minor * t.sin())
            }
            CurveGeom::Nurbs(c) => c.point_at(t),
            CurveGeom::Procedural(p) => p.approximation.point_at(t),
        }
    }

    fn derivative(&self, t: f64) -> Vector3<f64> {
        match self {
            CurveGeom::Line(l) => l.dir,
            CurveGeom::Circle(c) => (c.frame.y * t.cos() - c.frame.x * t.sin()) * c.radius,
            CurveGeom::Ellipse(e) => {
                e.frame.y * (e.minor * t.cos()) - e.frame.x * (e.major * t.sin())
            }
            CurveGeom::Nurbs(c) => c.derivatives(t, 1)[1],
            CurveGeom::Procedural(p) => p.approximation.derivatives(t, 1)[1],
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Plane {
    pub frame: Frame3,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Cylinder {
    pub frame: Frame3,
    pub radius: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Cone {
    pub frame: Frame3,
    pub radius_at_origin: f64,
    /// Half-angle between axis and surface line, radians (may be negative
    /// for a cone that narrows along +z).
    pub half_angle: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Sphere {
    pub frame: Frame3,
    pub radius: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Torus {
    pub frame: Frame3,
    pub major_radius: f64,
    pub minor_radius: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ProceduralSurfaceKind {
    Offset {
        distance: f64,
    },
    /// Rolling-ball blend; radius law and supports are held by the feature that created it.
    Blend,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ProceduralSurface {
    pub kind: ProceduralSurfaceKind,
    pub approximation: NurbsSurface,
    pub tolerance: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceGeom {
    Plane(Plane),
    Cylinder(Cylinder),
    Cone(Cone),
    Sphere(Sphere),
    Torus(Torus),
    Nurbs(NurbsSurface),
    /// `S(u, v) = profile(u) + v * dir`
    Extrusion {
        profile: Box<CurveGeom>,
        dir: Vector3<f64>,
    },
    /// `S(u, v) = rotate(profile(u), axis, v)`; `v` in radians.
    Revolution {
        profile: Box<CurveGeom>,
        axis_origin: Point3<f64>,
        axis_dir: Vector3<f64>,
    },
    Procedural(Box<ProceduralSurface>),
}

impl Surface for SurfaceGeom {
    fn domain(&self) -> ((f64, f64), (f64, f64)) {
        let inf = (f64::NEG_INFINITY, f64::INFINITY);
        match self {
            SurfaceGeom::Plane(_) => (inf, inf),
            SurfaceGeom::Cylinder(_) | SurfaceGeom::Cone(_) => ((0.0, TAU), inf),
            SurfaceGeom::Sphere(_) => ((0.0, TAU), (-FRAC_PI_2, FRAC_PI_2)),
            SurfaceGeom::Torus(_) => ((0.0, TAU), (0.0, TAU)),
            SurfaceGeom::Nurbs(s) => s.domain(),
            SurfaceGeom::Extrusion { profile, .. } => (profile.domain(), inf),
            SurfaceGeom::Revolution { profile, .. } => (profile.domain(), (0.0, TAU)),
            SurfaceGeom::Procedural(p) => p.approximation.domain(),
        }
    }

    fn point_at(&self, u: f64, v: f64) -> Point3<f64> {
        match self {
            SurfaceGeom::Plane(p) => p.frame.origin + p.frame.x * u + p.frame.y * v,
            SurfaceGeom::Cylinder(c) => {
                let f = &c.frame;
                f.origin + (f.x * u.cos() + f.y * u.sin()) * c.radius + f.z * v
            }
            SurfaceGeom::Cone(c) => {
                let f = &c.frame;
                let r = c.radius_at_origin + v * c.half_angle.tan();
                f.origin + (f.x * u.cos() + f.y * u.sin()) * r + f.z * v
            }
            SurfaceGeom::Sphere(s) => {
                let f = &s.frame;
                f.origin + ((f.x * u.cos() + f.y * u.sin()) * v.cos() + f.z * v.sin()) * s.radius
            }
            SurfaceGeom::Torus(t) => {
                let f = &t.frame;
                let ring = f.x * u.cos() + f.y * u.sin();
                f.origin
                    + ring * (t.major_radius + t.minor_radius * v.cos())
                    + f.z * (t.minor_radius * v.sin())
            }
            SurfaceGeom::Nurbs(s) => s.point_at(u, v),
            SurfaceGeom::Extrusion { profile, dir } => profile.point_at(u) + dir * v,
            SurfaceGeom::Revolution {
                profile,
                axis_origin,
                axis_dir,
            } => {
                let rot = Rotation3::from_axis_angle(&Unit::new_normalize(*axis_dir), v);
                *axis_origin + rot * (profile.point_at(u) - *axis_origin)
            }
            SurfaceGeom::Procedural(p) => p.approximation.point_at(u, v),
        }
    }

    fn derivatives(&self, u: f64, v: f64) -> (Vector3<f64>, Vector3<f64>) {
        match self {
            SurfaceGeom::Plane(p) => (p.frame.x, p.frame.y),
            SurfaceGeom::Cylinder(c) => {
                let f = &c.frame;
                ((f.y * u.cos() - f.x * u.sin()) * c.radius, f.z)
            }
            SurfaceGeom::Cone(c) => {
                let f = &c.frame;
                let t = c.half_angle.tan();
                let r = c.radius_at_origin + v * t;
                (
                    (f.y * u.cos() - f.x * u.sin()) * r,
                    (f.x * u.cos() + f.y * u.sin()) * t + f.z,
                )
            }
            SurfaceGeom::Sphere(s) => {
                let f = &s.frame;
                let ring = f.x * u.cos() + f.y * u.sin();
                let dring = f.y * u.cos() - f.x * u.sin();
                (
                    dring * (v.cos() * s.radius),
                    (f.z * v.cos() - ring * v.sin()) * s.radius,
                )
            }
            SurfaceGeom::Torus(t) => {
                let f = &t.frame;
                let ring = f.x * u.cos() + f.y * u.sin();
                let dring = f.y * u.cos() - f.x * u.sin();
                (
                    dring * (t.major_radius + t.minor_radius * v.cos()),
                    (f.z * v.cos() - ring * v.sin()) * t.minor_radius,
                )
            }
            SurfaceGeom::Nurbs(s) => {
                let d = s.derivatives(u, v, 1);
                (d[1][0], d[0][1])
            }
            SurfaceGeom::Extrusion { profile, dir } => (profile.derivative(u), *dir),
            SurfaceGeom::Revolution {
                profile,
                axis_origin,
                axis_dir,
            } => {
                let axis = Unit::new_normalize(*axis_dir);
                let rot = Rotation3::from_axis_angle(&axis, v);
                let su = rot * profile.derivative(u);
                let sv = axis.cross(&(rot * (profile.point_at(u) - *axis_origin)));
                (su, sv)
            }
            SurfaceGeom::Procedural(p) => {
                let d = p.approximation.derivatives(u, v, 1);
                (d[1][0], d[0][1])
            }
        }
    }

    fn normal_at(&self, u: f64, v: f64) -> Vector3<f64> {
        // Closed forms stay valid at poles and apexes where Su vanishes.
        match self {
            SurfaceGeom::Plane(p) => p.frame.z,
            SurfaceGeom::Cylinder(c) => c.frame.x * u.cos() + c.frame.y * u.sin(),
            SurfaceGeom::Cone(c) => ((c.frame.x * u.cos() + c.frame.y * u.sin())
                - c.frame.z * c.half_angle.tan())
            .normalize(),
            SurfaceGeom::Sphere(s) => {
                (s.frame.x * u.cos() + s.frame.y * u.sin()) * v.cos() + s.frame.z * v.sin()
            }
            SurfaceGeom::Torus(t) => {
                (t.frame.x * u.cos() + t.frame.y * u.sin()) * v.cos() + t.frame.z * v.sin()
            }
            _ => {
                let (su, sv) = self.derivatives(u, v);
                let n = su.cross(&sv);
                let l = n.norm();
                if l > 1e-300 {
                    n / l
                } else {
                    Vector3::z()
                }
            }
        }
    }

    fn period_u(&self) -> Option<f64> {
        match self {
            SurfaceGeom::Cylinder(_)
            | SurfaceGeom::Cone(_)
            | SurfaceGeom::Sphere(_)
            | SurfaceGeom::Torus(_) => Some(TAU),
            _ => None,
        }
    }

    fn period_v(&self) -> Option<f64> {
        match self {
            SurfaceGeom::Torus(_) | SurfaceGeom::Revolution { .. } => Some(TAU),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn all_analytic() -> Vec<SurfaceGeom> {
        let f =
            Frame3::from_origin_normal(Point3::new(0.5, -1.0, 2.0), Vector3::new(0.2, 0.5, 1.0));
        vec![
            SurfaceGeom::Plane(Plane { frame: f }),
            SurfaceGeom::Cylinder(Cylinder {
                frame: f,
                radius: 1.5,
            }),
            SurfaceGeom::Cone(Cone {
                frame: f,
                radius_at_origin: 2.0,
                half_angle: 0.3,
            }),
            SurfaceGeom::Cone(Cone {
                frame: f,
                radius_at_origin: 2.0,
                half_angle: -0.4,
            }),
            SurfaceGeom::Sphere(Sphere {
                frame: f,
                radius: 2.0,
            }),
            SurfaceGeom::Torus(Torus {
                frame: f,
                major_radius: 5.0,
                minor_radius: 1.0,
            }),
            SurfaceGeom::Extrusion {
                profile: Box::new(CurveGeom::Circle(Circle {
                    frame: f,
                    radius: 1.0,
                })),
                dir: Vector3::new(0.1, 0.2, 3.0),
            },
            SurfaceGeom::Revolution {
                profile: Box::new(CurveGeom::Line(Line {
                    origin: Point3::new(2.0, 0.0, 0.0),
                    dir: Vector3::new(0.3, 0.0, 1.0),
                })),
                axis_origin: Point3::origin(),
                axis_dir: Vector3::z(),
            },
        ]
    }

    #[test]
    fn analytic_derivatives_match_finite_differences() {
        let h = 1e-6;
        for s in all_analytic() {
            for &(u, v) in &[(0.4, 0.3), (2.1, 0.8), (4.0, 0.2)] {
                let (su, sv) = s.derivatives(u, v);
                let fu = (s.point_at(u + h, v).coords - s.point_at(u - h, v).coords) / (2.0 * h);
                let fv = (s.point_at(u, v + h).coords - s.point_at(u, v - h).coords) / (2.0 * h);
                assert!((su - fu).norm() < 1e-6 * (1.0 + su.norm()), "{s:?}");
                assert!((sv - fv).norm() < 1e-6 * (1.0 + sv.norm()), "{s:?}");
            }
        }
    }

    #[test]
    fn closed_form_normals_agree_with_cross_product_direction() {
        for s in all_analytic() {
            for &(u, v) in &[(0.4, 0.3), (2.1, 0.8), (4.0, 0.2)] {
                let (su, sv) = s.derivatives(u, v);
                let n = su.cross(&sv).normalize();
                let m = s.normal_at(u, v);
                assert!((n - m).norm() < 1e-9, "{s:?} at ({u},{v}): {n:?} vs {m:?}");
            }
        }
    }

    #[test]
    fn circle_derivative_is_tangent() {
        let c = CurveGeom::Circle(Circle {
            frame: Frame3::world(),
            radius: 2.0,
        });
        let p = c.point_at(0.7);
        let d = c.derivative(0.7);
        assert!(d.dot(&p.coords).abs() < 1e-14);
        assert!((d.norm() - 2.0).abs() < 1e-14);
    }

    #[test]
    fn torus_point_on_outer_equator() {
        let t = SurfaceGeom::Torus(Torus {
            frame: Frame3::world(),
            major_radius: 5.0,
            minor_radius: 1.0,
        });
        let p = t.point_at(0.0, 0.0);
        assert!((p.x - 6.0).abs() < 1e-12 && p.y.abs() < 1e-12 && p.z.abs() < 1e-12);
    }
}
