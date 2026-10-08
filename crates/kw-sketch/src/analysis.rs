use crate::constraint::ConstraintId;
use crate::entity::VarId;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchStatus {
    /// `dof > 0`, residual ≈ 0.
    UnderConstrained,
    /// `dof == 0`, rank full, residual ≈ 0.
    FullyConstrained,
    /// Redundant but consistent constraints (rank < m, residual ≈ 0).
    OverConstrained,
    /// Residual does not reach tolerance: contradictory constraints.
    Conflicting,
}

/// Result of Jacobian rank analysis at the current solution.
#[derive(Debug, Clone)]
pub struct DofReport {
    pub status: SketchStatus,
    /// Remaining degrees of freedom (n − rank − unremoved rigid motions).
    pub dof: usize,
    /// Variables that still have non-zero motion in the Jacobian null space.
    pub free_vars: Vec<VarId>,
    /// Constraints that are linearly dependent on others (consistent).
    pub redundant: Vec<ConstraintId>,
    /// Minimal-ish set of constraints that cannot be satisfied together.
    pub conflicting: Vec<ConstraintId>,
    pub residual_norm: f64,
}
