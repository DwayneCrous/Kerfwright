//! Parametric feature DAG, persistent topological names, references and the
//! evaluator contract. See docs/01-research/03-parametric-dag.md.

pub mod dag;
pub mod eval;
pub mod naming;

pub use dag::{DagError, DagNode, EvalState, FeatureDag, NodeId, NodeKind};
pub use eval::{EvalError, EvalOutput, FeatureEvaluator, InputHash};
pub use naming::{GeomFingerprint, NameMap, NameOrigin, Reference, ResolveOutcome, TopoName};
