use crate::analysis::DofReport;
use crate::entity::VarId;
use crate::Sketch;

/// One non-zero of the sparse Jacobian.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SparseEntry {
    pub row: usize,
    pub col: usize,
    pub value: f64,
}

/// Residual block contributed by one constraint (or one continuity level of
/// a joint). Hand-written analytic Jacobians; finite-difference/AD versions
/// exist only as test oracles.
pub trait Residual: Send + Sync {
    /// Number of scalar equations.
    fn arity(&self) -> usize;
    /// Variables this block reads (defines Jacobian sparsity and cluster graph edges).
    fn variables(&self) -> &[VarId];
    /// Write `arity()` residuals into `out`.
    fn eval(&self, x: &[f64], out: &mut [f64]);
    /// Append non-zeros with *local* row indices `0..arity()`.
    fn jacobian(&self, x: &[f64], out: &mut Vec<SparseEntry>);
}

#[derive(Debug, Clone)]
pub struct SolveOptions {
    pub max_iterations: usize,
    /// `‖F‖∞` convergence threshold (relative to sketch scale).
    pub residual_tol: f64,
    pub step_tol: f64,
    /// Initial Levenberg–Marquardt damping.
    pub lambda0: f64,
    /// Minimise movement from the starting point when under-determined.
    pub minimum_norm: bool,
    /// Variables to treat as soft targets (dragging): (var, target value).
    pub drag_targets: Vec<(VarId, f64)>,
}

impl Default for SolveOptions {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            residual_tol: 1e-10,
            step_tol: 1e-12,
            lambda0: 1e-3,
            minimum_norm: true,
            drag_targets: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveStatus {
    Converged,
    MaxIterations,
    Singular,
    Diverged,
}

#[derive(Debug, Clone)]
pub struct SolveOutcome {
    pub status: SolveStatus,
    pub iterations: usize,
    pub residual_norm: f64,
    /// Number of independent clusters solved (for profiling decomposition).
    pub clusters: usize,
}

/// Pluggable solver. The default implementation (Phase 2) decomposes the
/// constraint graph into clusters, solves each with damped Gauss–Newton/LM
/// on rank-revealing QR, and falls back to dog-leg then BFGS.
pub trait ConstraintSolver {
    /// Solve in place, updating `sketch.vars.values`.
    fn solve(&mut self, sketch: &mut Sketch, opts: &SolveOptions) -> SolveOutcome;

    /// Rank/null-space analysis at the current variable values.
    fn analyze(&self, sketch: &Sketch) -> DofReport;

    /// Incremental re-solve after moving `var` to `target`, restricted to the
    /// clusters reachable from `var`, warm-started from current values.
    fn drag(&mut self, sketch: &mut Sketch, var: VarId, target: f64) -> SolveOutcome;
}
