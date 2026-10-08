use crate::frame::Frame3;
use crate::nurbs::{NurbsCurve, NurbsSurface};
use nalgebra::{Point3, Rotation3, Unit, Vector3};

/// Parametric 3D curve.
pub trait Curve3 {
    fn domain(&self) -> (f64, f64);
    fn point_at(&self, t: f64) -> Point3<f64>;
}

/// Parametric surface `S(u, v)`.
pub trait Surface {
    fn domain(&self) -> ((f64, f64), (f64, f64));
    fn point_at(&self, u: f64, v: f64) -> Point3<f64>;
}

#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub origin: Point3<f64>,
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
            CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => (0.0, std::f64::consts::TAU),
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
    /// Half-angle between axis and surface line, radians.
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
    Offset { distance: f64 },
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
    Extrusion { profile: Box<CurveGeom>, dir: Vector3<f64> },
    /// `S(u, v) = rotate(profile(u), axis, v)`; `v` in radians.
    Revolution { profile: Box<CurveGeom>, axis_origin: Point3<f64>, axis_dir: Vector3<f64> },
    Procedural(Box<ProceduralSurface>),
}

impl Surface for SurfaceGeom {
    fn domain(&self) -> ((f64, f64), (f64, f64)) {
        use std::f64::consts::{FRAC_PI_2, TAU};
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
                f.origin
                    + ((f.x * u.cos() + f.y * u.sin()) * v.cos() + f.z * v.sin()) * s.radius
            }
            SurfaceGeom::Torus(t) => {
                let f = &t.frame;
                let ring = f.x * u.cos() + f.y * u.sin();
                f.origin + ring * (t.major_radius + t.minor_radius * v.cos())
                    + f.z * (t.minor_radius * v.sin())
            }
            SurfaceGeom::Nurbs(s) => s.point_at(u, v),
            SurfaceGeom::Extrusion { profile, dir } => profile.point_at(u) + dir * v,
            SurfaceGeom::Revolution { profile, axis_origin, axis_dir } => {
                let rot = Rotation3::from_axis_angle(&Unit::new_normalize(*axis_dir), v);
                *axis_origin + rot * (profile.point_at(u) - *axis_origin)
            }
            SurfaceGeom::Procedural(p) => p.approximation.point_at(u, v),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn revolve_line_gives_cylinder_radius() {
        let profile = CurveGeom::Line(Line {
            origin: Point3::new(2.0, 0.0, 0.0),
            dir: Vector3::new(0.0, 0.0, 1.0),
        });
        let s = SurfaceGeom::Revolution {
            profile: Box::new(profile),
            axis_origin: Point3::origin(),
            axis_dir: Vector3::z(),
        };
        let p = s.point_at(3.0, 1.234);
        assert!(((p.x * p.x + p.y * p.y).sqrt() - 2.0).abs() < 1e-12);
        assert!((p.z - 3.0).abs() < 1e-12);
    }
}
