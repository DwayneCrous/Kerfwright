use crate::dag::{DagNode, NodeId};
use crate::naming::NameMap;
use kw_brep::{BRep, ShapeHistory};
use std::sync::Arc;

/// Content hash of `(kernel_version, node params, input output-hashes)`.
/// Key of the evaluation cache.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InputHash(pub u128);

/// Immutable result of evaluating one node. Shared via `Arc` so unchanged
/// bodies are never copied.
#[derive(Debug)]
pub struct EvalOutput {
    pub brep: Arc<BRep>,
    pub names: NameMap,
    pub history: ShapeHistory,
    /// Hash of geometry + names; equal hashes trigger early cutoff.
    pub output_hash: u128,
}

#[derive(Debug)]
pub enum EvalError {
    /// A stored `Reference` could not be resolved uniquely.
    Reference(String),
    Kernel(String),
    Cancelled,
}

/// Implemented per feature kind (kw-modeling, kw-sheet, ...). Must be
/// deterministic: same inputs ⇒ bit-identical output and names on any thread count.
pub trait FeatureEvaluator: Send + Sync {
    fn evaluate(
        &self,
        node: &DagNode,
        id: NodeId,
        inputs: &[Arc<EvalOutput>],
    ) -> Result<EvalOutput, EvalError>;
}
