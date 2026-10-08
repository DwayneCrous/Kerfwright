use crate::entity::{EntityId, VarId};
use slotmap::new_key_type;

new_key_type! {
    pub struct ConstraintId;
}

/// Geometric continuity level at a joint between two curves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Continuity {
    /// Position.
    G0,
    /// + tangent direction.
    G1,
    /// + signed curvature.
    G2,
    /// + curvature derivative w.r.t. arc length.
    G3,
}

impl Continuity {
    /// Number of scalar equations in 2D (G0 = 2, G1 = 3, G2 = 4, G3 = 5).
    pub fn equation_count(self) -> usize {
        match self {
            Continuity::G0 => 2,
            Continuity::G1 => 3,
            Continuity::G2 => 4,
            Continuity::G3 => 5,
        }
    }
}

/// Root selection captured from the initial geometry so tangency, offsets
/// and angles do not flip between valid solutions during a drag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TangentBranch {
    Positive,
    Negative,
}

/// Which end of a curve a joint constraint refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveEnd {
    Start,
    End,
}

#[derive(Debug, Clone)]
pub enum ConstraintKind {
    Coincident {
        a: EntityId,
        b: EntityId,
    },
    PointOnCurve {
        point: EntityId,
        curve: EntityId,
    },
    Horizontal(EntityId),
    Vertical(EntityId),
    Parallel {
        a: EntityId,
        b: EntityId,
    },
    Perpendicular {
        a: EntityId,
        b: EntityId,
    },
    Collinear {
        a: EntityId,
        b: EntityId,
    },
    Concentric {
        a: EntityId,
        b: EntityId,
    },
    EqualLength {
        a: EntityId,
        b: EntityId,
    },
    EqualRadius {
        a: EntityId,
        b: EntityId,
    },
    Midpoint {
        point: EntityId,
        line: EntityId,
    },
    Symmetric {
        a: EntityId,
        b: EntityId,
        axis: EntityId,
    },
    Fix(EntityId),
    Tangent {
        a: EntityId,
        b: EntityId,
        branch: TangentBranch,
    },
    /// Continuity joint between two curves at the given ends.
    Joint {
        a: EntityId,
        a_end: CurveEnd,
        b: EntityId,
        b_end: CurveEnd,
        level: Continuity,
        /// Same-direction branch for the G1 tangent test.
        branch: TangentBranch,
    },
    // Dimensional: `value` is a driving variable so expressions can bind to it.
    Distance {
        a: EntityId,
        b: EntityId,
        value: VarId,
        branch: TangentBranch,
    },
    Angle {
        a: EntityId,
        b: EntityId,
        value: VarId,
        branch: TangentBranch,
    },
    Radius {
        circle: EntityId,
        value: VarId,
    },
    Diameter {
        circle: EntityId,
        value: VarId,
    },
}

#[derive(Debug, Clone)]
pub struct Constraint {
    pub kind: ConstraintKind,
    /// Reference (driven) dimensions are reported, not enforced.
    pub driven: bool,
}
