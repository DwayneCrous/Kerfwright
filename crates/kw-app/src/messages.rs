use kw_feature::NodeId;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    /// Background: CAM/FEA jobs, autosave, cache warming.
    Background,
    /// Normal edits: feature parameter change.
    Interactive,
    /// Per-frame: sketch drag solves. Preempts everything else.
    Realtime,
}

/// UI/collab → engine.
#[derive(Debug, Clone)]
pub enum Command {
    EditParameter {
        node: NodeId,
        name: String,
        expression: String,
    },
    DragSketchVar {
        sketch: NodeId,
        var: usize,
        target: f64,
    },
    RollbackTo(NodeId),
    Undo,
    Redo,
    Save,
}

/// Engine → UI/collab.
#[derive(Debug, Clone)]
pub enum Event {
    NodeEvaluated { node: NodeId },
    NodeFailed { node: NodeId, reason: String },
    SketchSolved { sketch: NodeId, residual_norm: f64 },
    TessellationReady { node: NodeId },
    Progress { job: u64, fraction: f32 },
}
