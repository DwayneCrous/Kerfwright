//! NURBS toolkit (Piegl & Tiller) and analytic/procedural geometry.
//!
//! * [`KnotVector`] — spans, basis functions and their derivatives
//! * [`NurbsCurve`] / [`NurbsSurface`] — evaluation with derivatives, knot
//!   insertion/removal, degree elevation, Bézier extraction, fitting,
//!   closest-point projection
//! * [`build`] — exact rational arcs, circles and surfaces of revolution
//! * [`geom`] — `CurveGeom` / `SurfaceGeom` used by the B-rep

// The algorithms follow the index notation of the cited literature (Piegl & Tiller etc.);
// rewriting those loops as iterators would obscure the correspondence.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments)]

pub mod build;
pub mod curve;
pub mod fit;
pub mod geom;
pub mod hom;
pub mod intersect;
pub mod knots;
pub mod pcurve;
pub mod proj;
pub mod shape;
pub mod surface;

pub use curve::{CurveProjection, NurbsCurve, NurbsError};
pub use geom::{
    Circle, Cone, Curve3, CurveGeom, Cylinder, Ellipse, Line, Plane, ProceduralCurve,
    ProceduralCurveKind, ProceduralSurface, ProceduralSurfaceKind, Sphere, Surface, SurfaceGeom,
    Torus,
};
pub use knots::{KnotError, KnotVector};
pub use shape::normal_curvature;
pub use surface::{ExtendDir, NurbsSurface, SurfaceProjection};
