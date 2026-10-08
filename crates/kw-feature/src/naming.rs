use crate::dag::NodeId;
use kw_brep::EntityRef;
use kw_sketch::EntityId as SketchEntityId;
use std::collections::HashMap;

/// How an entity was created, expressed in terms that survive upstream edits.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum NameOrigin {
    /// Created from a persistent sketch entity (e.g. side face of an extrude
    /// from a profile segment).
    SketchEdge {
        entity: SketchEntityId,
        role: &'static str,
    },
    /// Generated from another named entity (fillet face from its edge, ...).
    Generated {
        from: Box<TopoName>,
        role: &'static str,
    },
    /// k-th fragment of a split entity, ordered canonically.
    Split { parent: Box<TopoName>, ordinal: u32 },
    /// Merge of several entities; children sorted by their stable hash.
    Merge(Vec<TopoName>),
    /// Kernel primitive output with a fixed role ("top-cap", "bottom-cap", ...).
    Primitive { role: &'static str },
}

/// Persistent identity of a B-rep entity. This — not `FaceId`/`EdgeId` — is
/// what is written to the document.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TopoName {
    pub feature: NodeId,
    pub origin: NameOrigin,
    /// Canonical-order tie-breaker among otherwise identical siblings.
    pub ordinal: u32,
}

/// Per-evaluation lookup from persistent name to the live generational key.
#[derive(Debug, Default, Clone)]
pub struct NameMap {
    pub by_name: HashMap<TopoName, EntityRef>,
}

/// Geometric description used only as a fallback when lineage resolution fails.
#[derive(Debug, Clone, PartialEq)]
pub struct GeomFingerprint {
    pub kind: &'static str,
    pub axis_or_normal: Option<[f64; 3]>,
    pub centroid: [f64; 3],
    pub size: f64,
    /// Sorted hashes of adjacent entities' kinds/sizes.
    pub adjacency_signature: u64,
}

/// What downstream features store instead of a raw id.
#[derive(Debug, Clone, PartialEq)]
pub struct Reference {
    pub name: TopoName,
    pub fingerprint: GeomFingerprint,
    pub tolerance: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ResolveOutcome {
    Exact(EntityRef),
    /// Entity was split; caller decides if one-to-many is acceptable.
    Split(Vec<EntityRef>),
    /// Matched geometrically with sufficient score and margin.
    Matched {
        entity: EntityRef,
        score: f64,
    },
    /// More than one plausible candidate: surface a repair prompt, never guess.
    Ambiguous {
        candidates: Vec<(EntityRef, f64)>,
    },
    Broken,
}
