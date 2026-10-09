//! Implicit (signed-distance-like) forms of the analytic quadrics and the torus.
//! `value` is positive on the side the surface normal points to.

use crate::field::DistField;
use crate::Window;
use kw_nurbs::geom::*;
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone)]
pub enum Implicit {
    Plane {
        o: Point3<f64>,
        n: Vector3<f64>,
    },
    Cylinder {
        o: Point3<f64>,
        a: Vector3<f64>,
        r: f64,
    },
    Cone {
        o: Point3<f64>,
        a: Vector3<f64>,
        r0: f64,
        tan: f64,
        cos: f64,
    },
    Sphere {
        c: Point3<f64>,
        r: f64,
    },
    Torus {
        c: Point3<f64>,
        a: Vector3<f64>,
        big: f64,
        small: f64,
    },
    /// Signed distance field of a parametric surface (spline, extrusion, revolution, ...).
    Field(Box<DistField>),
}

impl Implicit {
    pub fn of(s: &SurfaceGeom) -> Option<Implicit> {
        Some(match s {
            SurfaceGeom::Plane(p) => Implicit::Plane {
                o: p.frame.origin,
                n: p.frame.z,
            },
            SurfaceGeom::Cylinder(c) => Implicit::Cylinder {
                o: c.frame.origin,
                a: c.frame.z,
                r: c.radius,
            },
            SurfaceGeom::Cone(c) => Implicit::Cone {
                o: c.frame.origin,
                a: c.frame.z,
                r0: c.radius_at_origin,
                tan: c.half_angle.tan(),
                cos: c.half_angle.cos(),
            },
            SurfaceGeom::Sphere(c) => Implicit::Sphere {
                c: c.frame.origin,
                r: c.radius,
            },
            SurfaceGeom::Torus(c) => Implicit::Torus {
                c: c.frame.origin,
                a: c.frame.z,
                big: c.major_radius,
                small: c.minor_radius,
            },
            _ => return None,
        })
    }

    /// Distance field of `s` over the window `w` (any surface; costs a tessellation).
    pub fn field(s: &SurfaceGeom, w: Window) -> Option<Implicit> {
        DistField::new(s, w).map(|f| Implicit::Field(Box::new(f)))
    }

    pub fn as_field(&self) -> Option<&DistField> {
        match self {
            Implicit::Field(f) => Some(f),
            _ => None,
        }
    }

    /// Parameters of the footpoint of `x` when this is a field.
    pub fn foot(&self, x: &Point3<f64>) -> Option<(f64, f64)> {
        match self {
            Implicit::Field(f) => Some(f.foot(x)),
            _ => None,
        }
    }

    /// Value and gradient at `x` (gradient has unit length for all but the torus/cone apex regions
    /// where it is still unit away from singular sets).
    pub fn eval(&self, x: &Point3<f64>) -> (f64, Vector3<f64>) {
        match self {
            Implicit::Plane { o, n } => ((x - o).dot(n), *n),
            Implicit::Cylinder { o, a, r } => {
                let q = x - o;
                let rad = q - a * q.dot(a);
                let rho = rad.norm();
                if rho < 1e-300 {
                    (-r, Vector3::zeros())
                } else {
                    (rho - r, rad / rho)
                }
            }
            Implicit::Cone { o, a, r0, tan, cos } => {
                let q = x - o;
                let h = q.dot(a);
                let rad = q - a * h;
                let rho = rad.norm();
                let f = (rho - r0 - h * tan) * cos;
                if rho < 1e-300 {
                    (f, -a * (tan * cos))
                } else {
                    (f, (rad / rho - a * *tan) * *cos)
                }
            }
            Implicit::Sphere { c, r } => {
                let q = x - c;
                let l = q.norm();
                if l < 1e-300 {
                    (-r, Vector3::zeros())
                } else {
                    (l - r, q / l)
                }
            }
            Implicit::Torus { c, a, big, small } => {
                let q = x - c;
                let h = q.dot(a);
                let rad = q - a * h;
                let rho = rad.norm();
                let (w, dir) = if rho < 1e-300 {
                    (-big, Vector3::zeros())
                } else {
                    (rho - big, rad / rho)
                };
                let d = w.hypot(h);
                if d < 1e-300 {
                    (-small, Vector3::zeros())
                } else {
                    (d - small, (dir * w + a * h) / d)
                }
            }
            Implicit::Field(f) => f.eval(x),
        }
    }
}
