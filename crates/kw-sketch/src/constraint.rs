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
}

impl Continuity {
    /// Number of scalar equations in 2D (G0 = 2, G1 = 3, G2 = 4).
    pub fn equation_count(self) -> usize {
        match self {
            Continuity::G0 => 2,
            Continuity::G1 => 3,
            Continuity::G2 => 4,
        }
    }
}

/// Root selection captured from the initial geometry so tangency, offsets
/// and joints do not flip between valid solutions during a drag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TangentBranch {
    Positive,
    Negative,
}

impl TangentBranch {
    pub fn sign(self) -> f64 {
        match self {
            TangentBranch::Positive => 1.0,
            TangentBranch::Negative => -1.0,
        }
    }
    pub fn from_sign(x: f64) -> Self {
        if x >= 0.0 {
            TangentBranch::Positive
        } else {
            TangentBranch::Negative
        }
    }
}

/// Which end of a curve a joint constraint refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveEnd {
    Start,
    End,
}

#[derive(Debug, Clone)]
pub enum ConstraintKind {
    /// Two points coincide.
    Coincident {
        a: EntityId,
        b: EntityId,
    },
    /// Point lies on a line (extended), circle, arc's circle or ellipse.
    PointOnCurve {
        point: EntityId,
        curve: EntityId,
    },
    /// Line is horizontal / vertical.
    Horizontal(EntityId),
    Vertical(EntityId),
    /// Two points share y / x.
    HorizontalPts {
        a: EntityId,
        b: EntityId,
    },
    VerticalPts {
        a: EntityId,
        b: EntityId,
    },
    Parallel {
        a: EntityId,
        b: EntityId,
    },
    Perpendicular {
        a: EntityId,
        b: EntityId,
    },
    /// Line `a` lies on the infinite line of `b`.
    Collinear {
        a: EntityId,
        b: EntityId,
    },
    /// Circle/arc/ellipse centres coincide.
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
    /// Pins every variable of the entity at its current value.
    Fix(EntityId),
    /// Line–circle/arc or circle/arc–circle/arc tangency.
    Tangent {
        a: EntityId,
        b: EntityId,
        branch: TangentBranch,
    },
    /// Continuity joint between two curves (lines, arcs, splines) at the given ends.
    Joint {
        a: EntityId,
        a_end: CurveEnd,
        b: EntityId,
        b_end: CurveEnd,
        level: Continuity,
        /// Sign of `To_A · To_B` when created (for branch-flip diagnostics).
        branch: TangentBranch,
    },
    /// Point–point, point–line (signed) or line–line (parallel) distance; `value` is a fixed variable.
    Distance {
        a: EntityId,
        b: EntityId,
        value: VarId,
        branch: TangentBranch,
    },
    /// Directed angle between two lines.
    Angle {
        a: EntityId,
        b: EntityId,
        value: VarId,
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
    /// Disabled constraints are ignored by the solver.
    pub enabled: bool,
}

impl ConstraintKind {
    /// Entities this constraint refers to.
    pub fn entities(&self) -> Vec<EntityId> {
        use ConstraintKind::*;
        match self {
            Horizontal(a) | Vertical(a) | Fix(a) => vec![*a],
            Coincident { a, b }
            | HorizontalPts { a, b }
            | VerticalPts { a, b }
            | Parallel { a, b }
            | Perpendicular { a, b }
            | Collinear { a, b }
            | Concentric { a, b }
            | EqualLength { a, b }
            | EqualRadius { a, b }
            | Tangent { a, b, .. }
            | Distance { a, b, .. }
            | Angle { a, b, .. }
            | Joint { a, b, .. } => vec![*a, *b],
            PointOnCurve { point, curve } => vec![*point, *curve],
            Midpoint { point, line } => vec![*point, *line],
            Symmetric { a, b, axis } => vec![*a, *b, *axis],
            Radius { circle, .. } | Diameter { circle, .. } => vec![*circle],
        }
    }
}
