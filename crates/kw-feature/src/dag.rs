use petgraph::algo::{is_cyclic_directed, toposort};
use petgraph::stable_graph::{NodeIndex, StableDiGraph};
use petgraph::visit::Dfs;
use petgraph::Direction;

/// Node identifier. Stable for the life of the document (backed by the graph index).
pub type NodeId = NodeIndex;

#[derive(Debug, Clone, PartialEq)]
pub enum NodeKind {
    Parameter { name: String, expression: String },
    Sketch,
    Extrude,
    Revolve,
    Sweep,
    Loft,
    Fillet,
    Chamfer,
    Shell,
    Draft,
    Hole,
    Boolean,
    Pattern,
    Mirror,
    SheetMetalFlange,
    Unfold,
    FormFinish,
    /// Analysis and manufacturing setups that consume bodies.
    FemStudy,
    CamSetup,
}

#[derive(Debug, Clone, PartialEq)]
pub enum EvalState {
    Clean,
    Dirty,
    /// Past the rollback bar; not evaluated but last-good cache retained.
    Suspended,
    Failed(String),
}

#[derive(Debug, Clone)]
pub struct DagNode {
    pub kind: NodeKind,
    pub name: String,
    pub suppressed: bool,
    pub state: EvalState,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DagError {
    UnknownNode,
    /// Adding this dependency would create a cycle; the edge was not added.
    WouldCreateCycle,
}

/// Data-dependency graph of features. Edges point from *dependency* to
/// *dependent* (`a -> b` means `b` reads `a`). The user-visible timeline is
/// one topological linearisation of this graph.
#[derive(Debug, Default)]
pub struct FeatureDag {
    graph: StableDiGraph<DagNode, ()>,
}

impl FeatureDag {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, node: DagNode) -> NodeId {
        self.graph.add_node(node)
    }

    pub fn node(&self, id: NodeId) -> Option<&DagNode> {
        self.graph.node_weight(id)
    }

    /// Declare that `dependent` reads the output of `dependency`.
    pub fn add_dependency(&mut self, dependency: NodeId, dependent: NodeId) -> Result<(), DagError> {
        if self.graph.node_weight(dependency).is_none() || self.graph.node_weight(dependent).is_none() {
            return Err(DagError::UnknownNode);
        }
        let edge = self.graph.add_edge(dependency, dependent, ());
        if is_cyclic_directed(&self.graph) {
            self.graph.remove_edge(edge);
            return Err(DagError::WouldCreateCycle);
        }
        Ok(())
    }

    pub fn dependencies(&self, id: NodeId) -> Vec<NodeId> {
        self.graph.neighbors_directed(id, Direction::Incoming).collect()
    }

    /// Evaluation order: dependencies before dependents.
    pub fn evaluation_order(&self) -> Result<Vec<NodeId>, DagError> {
        toposort(&self.graph, None).map_err(|_| DagError::WouldCreateCycle)
    }

    /// Mark `id` and every transitive dependent `Dirty` (suspended nodes keep
    /// their state). Returns the nodes that were marked.
    pub fn mark_dirty(&mut self, id: NodeId) -> Vec<NodeId> {
        let mut reached = Vec::new();
        let mut dfs = Dfs::new(&self.graph, id);
        while let Some(n) = dfs.next(&self.graph) {
            reached.push(n);
        }
        for n in &reached {
            if let Some(node) = self.graph.node_weight_mut(*n) {
                if node.state != EvalState::Suspended {
                    node.state = EvalState::Dirty;
                }
            }
        }
        reached
    }

    pub fn set_state(&mut self, id: NodeId, state: EvalState) {
        if let Some(n) = self.graph.node_weight_mut(id) {
            n.state = state;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(name: &str) -> DagNode {
        DagNode {
            kind: NodeKind::Sketch,
            name: name.into(),
            suppressed: false,
            state: EvalState::Clean,
        }
    }

    #[test]
    fn dirty_propagates_downstream_only() {
        let mut d = FeatureDag::new();
        let a = d.add_node(node("a"));
        let b = d.add_node(node("b"));
        let c = d.add_node(node("c"));
        let side = d.add_node(node("side"));
        d.add_dependency(a, b).unwrap();
        d.add_dependency(b, c).unwrap();
        d.mark_dirty(b);
        assert_eq!(d.node(a).unwrap().state, EvalState::Clean);
        assert_eq!(d.node(b).unwrap().state, EvalState::Dirty);
        assert_eq!(d.node(c).unwrap().state, EvalState::Dirty);
        assert_eq!(d.node(side).unwrap().state, EvalState::Clean);
    }

    #[test]
    fn cycles_are_rejected() {
        let mut d = FeatureDag::new();
        let a = d.add_node(node("a"));
        let b = d.add_node(node("b"));
        d.add_dependency(a, b).unwrap();
        assert_eq!(d.add_dependency(b, a), Err(DagError::WouldCreateCycle));
        assert_eq!(d.evaluation_order().unwrap(), vec![a, b]);
    }
}
