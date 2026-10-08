//! Math foundation: tolerance model, knot vectors, NURBS data types, analytic
//! curve/surface definitions. See docs/01-research/01-brep-kernel.md.

pub mod frame;
pub mod geom;
pub mod knots;
pub mod nurbs;
pub mod tolerance;

pub use nalgebra::{Point2, Point3, Unit, Vector2, Vector3};

pub use frame::Frame3;
pub use geom::{Curve3, CurveGeom, Surface, SurfaceGeom};
pub use knots::{KnotError, KnotVector};
pub use nurbs::{NurbsCurve, NurbsError, NurbsSurface};
pub use tolerance::Tolerance;
