use slotmap::new_key_type;

new_key_type! {
    /// Persistent within a document: used by kw-feature to build stable
    /// topological names for geometry derived from this sketch.
    pub struct EntityId;
}

/// Index into `VarStore::values`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VarId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PointVars {
    pub x: VarId,
    pub y: VarId,
}

#[derive(Debug, Clone)]
pub enum EntityKind {
    Point(PointVars),
    Line {
        p1: EntityId,
        p2: EntityId,
    },
    Circle {
        center: EntityId,
        radius: VarId,
    },
    /// Counter-clockwise arc from `start` to `end` about `center`. The radius
    /// is implicit (`|start − center|`); `|end − center|` is kept equal by an
    /// implicit consistency equation.
    Arc {
        center: EntityId,
        start: EntityId,
        end: EntityId,
    },
    Ellipse {
        center: EntityId,
        major_end: EntityId,
        minor_radius: VarId,
    },
    /// Clamped uniform non-rational B-spline of `degree`; control points are point entities.
    Spline {
        degree: usize,
        control: Vec<EntityId>,
    },
    /// Fit-point spline interpolating the listed points (geometry only; not solved).
    FitSpline {
        points: Vec<EntityId>,
    },
}

#[derive(Debug, Clone)]
pub struct Entity {
    pub kind: EntityKind,
    /// Construction geometry is solved but not used for profiles.
    pub construction: bool,
    /// Pulled in from the model (Project / Intersect): shown in purple, and locked in place.
    pub projected: bool,
}
