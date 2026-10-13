//! Surface diagnostics: curvature, draft angle, zebra stripes and the colour ramps that show them.
//! Pure maths with no GPU: the viewer's shader repeats the formulas of [`zebra`] and [`ramp`] and
//! the tests check the two against each other.

pub mod curvature;
pub mod draft;
pub mod ramp;
pub mod zebra;

pub use curvature::{curvature_at, mesh_curvature, Curv, CurvKind};
pub use draft::{draft_degrees, mesh_draft};
