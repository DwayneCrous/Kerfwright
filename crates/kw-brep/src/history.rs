use crate::ids::*;
use std::borrow::Cow;
use std::collections::{HashMap, HashSet};

/// Role names are usually static but operations may build them (e.g. `side#3`).
pub type Role = Cow<'static, str>;

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
    Generated {
        from: EntityRef,
        role: Role,
    },
    /// k-th fragment of a split entity.
    SplitFrom {
        parent: EntityRef,
        ordinal: u32,
    },
    MergedFrom(Vec<EntityRef>),
    New {
        role: Role,
    },
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

impl ShapeHistory {
    /// The input entity an output entity descends from through `Preserved` / `Modified` links.
    pub fn source_of(&self, e: EntityRef) -> Option<EntityRef> {
        match self.origins.get(&e)? {
            Origin::Preserved(s) | Origin::Modified(s) => Some(*s),
            _ => None,
        }
    }

    /// Compose two histories: `first` maps stage-1 outputs to the original inputs, `second` maps
    /// stage-2 outputs to stage-1 entities (and to other inputs of stage 2, e.g. tool bodies, which
    /// stay as they are). The result maps stage-2 outputs to original inputs.
    pub fn compose(first: &ShapeHistory, second: &ShapeHistory) -> ShapeHistory {
        // reference to a stage-1 entity → what it descends from (itself if stage 1 knows nothing about it)
        let lift = |r: EntityRef| -> EntityRef { first.source_of(r).unwrap_or(r) };
        let mut out = ShapeHistory::default();
        for (e, o) in &second.origins {
            let mapped = match o {
                Origin::Preserved(i) => match first.origins.get(i) {
                    Some(Origin::Preserved(j)) => Origin::Preserved(*j),
                    Some(Origin::Modified(j)) => Origin::Modified(*j),
                    Some(other) => other.clone(),
                    None => Origin::Preserved(*i),
                },
                Origin::Modified(i) => match first.origins.get(i) {
                    Some(Origin::Preserved(j)) | Some(Origin::Modified(j)) => Origin::Modified(*j),
                    Some(other) => other.clone(),
                    None => Origin::Modified(*i),
                },
                Origin::Generated { from, role } => Origin::Generated {
                    from: lift(*from),
                    role: role.clone(),
                },
                Origin::SplitFrom { parent, ordinal } => match first.origins.get(parent) {
                    // a fragment of something that was itself a fragment keeps the original parent
                    Some(Origin::SplitFrom {
                        parent: p0,
                        ordinal: o0,
                    }) => Origin::SplitFrom {
                        parent: *p0,
                        ordinal: o0 * 1000 + ordinal,
                    },
                    _ => Origin::SplitFrom {
                        parent: lift(*parent),
                        ordinal: *ordinal,
                    },
                },
                Origin::MergedFrom(v) => Origin::MergedFrom(v.iter().map(|r| lift(*r)).collect()),
                Origin::New { role } => Origin::New { role: role.clone() },
            };
            out.origins.insert(*e, mapped);
        }
        // inputs deleted by stage 1 stay deleted; inputs deleted by stage 2 are deleted if they descend from an original
        for d in &first.deleted_inputs {
            out.deleted_inputs.insert(*d);
        }
        for d in &second.deleted_inputs {
            if first.origins.contains_key(d) {
                if let Some(src) = first.source_of(*d) {
                    // a stage-1 output vanished: its source is gone only if nothing else descends from it
                    let survives = out.origins.values().any(
                        |o| matches!(o, Origin::Preserved(s) | Origin::Modified(s) if *s == src),
                    );
                    if !survives {
                        out.deleted_inputs.insert(src);
                    }
                }
            } else {
                out.deleted_inputs.insert(*d);
            }
        }
        out
    }
}
