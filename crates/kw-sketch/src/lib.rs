//! 2D sketch model, constraint library, solver and analysis.
//! See docs/01-research/02-constraint-solver.md.

// Numerical kernels follow the index notation of the cited literature and carry
// many tightly-coupled parameters.
#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod ad;
pub mod analysis;
pub mod constraint;
pub mod editor;
pub mod entity;
pub mod geom;
pub mod graph;
pub mod ops;
pub mod residual;
pub mod sketch;
pub mod solve;
pub mod system;
#[doc(hidden)]
pub mod testgen;

pub use analysis::{DofReport, SketchStatus};
pub use constraint::{
    Constraint, ConstraintId, ConstraintKind, Continuity, CurveEnd, TangentBranch,
};
pub use entity::{Entity, EntityId, EntityKind, PointVars, VarId};
pub use sketch::{Sketch, SketchError, VarStore};
pub use solve::{DragOutcome, SolveOptions, SolveOutcome, SolveStatus, Solver};
pub use system::System;
