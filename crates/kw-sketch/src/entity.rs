use slotmap::new_key_type;

new_key_type! {
    /// Persistent within a document: used by kw-feature to build stable
    /// topological names for geometry derived from this sketch.
    pub struct EntityId;
}

/// Index into `VarStore::values`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
    /// Counter-clockwise arc from `start` to `end` about `center`.
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
    /// Degree-`degree` B-spline; control points are point entities.
    Spline {
        degree: usize,
        control: Vec<EntityId>,
        weights: Vec<VarId>,
    },
    /// Fit-point spline interpolating the listed points.
    FitSpline {
        points: Vec<EntityId>,
    },
}

#[derive(Debug, Clone)]
pub struct Entity {
    pub kind: EntityKind,
    /// Construction geometry is solved but not used for profiles.
    pub construction: bool,
}
