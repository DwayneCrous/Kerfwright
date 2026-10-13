//! The parametric engine: a serialisable document of features, expressions with units, an incremental
//! evaluator with early cutoff, persistent topological names, and the `.kwf` file format.

// Numerical code indexes small fixed-size arrays; `!(x > 0.0)` is deliberate (it also rejects NaN).
#![allow(
    clippy::needless_range_loop,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod builder;
pub mod doc;
pub mod engine;
pub mod eval;
pub mod geomhash;
pub mod id;
pub mod kwf;
pub mod legacy_profile;
pub mod legacy_regions;
pub mod namer;
pub mod naming;
pub mod ops;
pub mod profile;
pub mod recipe;
pub mod regions;
pub mod spec;
pub mod threads;
pub mod tools;

pub use doc::{Appearance, Document, Feature, History, TimelineError};
pub use engine::{CancelToken, Engine, Evaluation, FeatureState, RunStats, KERNEL_VERSION};
pub use eval::{
    EvalError, EvalOutput, Product, Resolution, SketchProduct, SolidProduct, ToolProduct,
    Unresolved,
};
pub use id::{FeatureId, Hasher128};
pub use naming::{
    GeomFingerprint, NameMap, NameOrigin, Reference, ResolveKind, ResolveOutcome, Resolved,
    TopoName,
};
pub use profile::{sketch_profile, sketch_profile_labeled, sketch_wires, LabeledProfile, Wire};
pub use recipe::SketchRecipe;
pub use spec::*;

/// The old name for a feature identifier (kept for `kw-app`).
pub type NodeId = FeatureId;
