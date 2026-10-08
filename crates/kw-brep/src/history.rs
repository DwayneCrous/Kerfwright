use crate::ids::*;
use std::collections::{HashMap, HashSet};

/// Type-erased reference to any topological entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EntityRef {
    Vertex(VertexId),
    Edge(EdgeId),
    Face(FaceId),
}

/// How an output entity came to be, relative to the operation's inputs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Origin {
    Preserved(EntityRef),
    Modified(EntityRef),
    /// New entity generated from `from` in `role` (e.g. side face from a profile edge).
    Generated { from: EntityRef, role: &'static str },
    /// k-th fragment of a split entity.
    SplitFrom { parent: EntityRef, ordinal: u32 },
    MergedFrom(Vec<EntityRef>),
    New { role: &'static str },
}

/// Every kernel operation returns one of these next to the new body so the
/// naming layer can build exact lineage names. Mirrors the public
/// OpenCASCADE `Modified/Generated/IsDeleted` model.
#[derive(Debug, Clone, Default)]
pub struct ShapeHistory {
    pub origins: HashMap<EntityRef, Origin>,
    pub deleted_inputs: HashSet<EntityRef>,
}

impl ShapeHistory {
    pub fn record(&mut self, output: EntityRef, origin: Origin) {
        self.origins.insert(output, origin);
    }

    pub fn mark_deleted(&mut self, input: EntityRef) {
        self.deleted_inputs.insert(input);
    }
}
