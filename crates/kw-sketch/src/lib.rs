//! 2D sketch model: entities, constraints, DOF analysis, solver traits.
//! See docs/01-research/02-constraint-solver.md.

pub mod analysis;
pub mod constraint;
pub mod entity;
pub mod solver;

pub use analysis::{DofReport, SketchStatus};
pub use constraint::{
    Constraint, ConstraintId, ConstraintKind, Continuity, CurveEnd, TangentBranch,
};
pub use entity::{Entity, EntityId, EntityKind, PointVars, VarId};
pub use solver::{
    ConstraintSolver, Residual, SolveOptions, SolveOutcome, SolveStatus, SparseEntry,
};

use slotmap::SlotMap;

/// Scalar unknowns of the sketch (coordinates, radii, ...). Solvers operate
/// on `values` as the vector `x`.
#[derive(Debug, Default, Clone)]
pub struct VarStore {
    pub values: Vec<f64>,
    /// Variables the user has pinned (e.g. dragged-to values or fixed points).
    pub fixed: Vec<bool>,
}

impl VarStore {
    pub fn push(&mut self, value: f64) -> VarId {
        self.values.push(value);
        self.fixed.push(false);
        VarId(self.values.len() - 1)
    }
}

/// Entities + constraints + variables for one sketch.
#[derive(Debug, Default)]
pub struct Sketch {
    pub vars: VarStore,
    pub entities: SlotMap<EntityId, Entity>,
    pub constraints: SlotMap<ConstraintId, Constraint>,
}
