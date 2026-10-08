use slotmap::new_key_type;

// Generational keys: valid only within one evaluation of one body.
// Cross-edit identity is the job of kw-feature's TopoName, not these keys.
new_key_type! {
    pub struct VertexId;
    pub struct EdgeId;
    pub struct CoedgeId;
    pub struct LoopId;
    pub struct FaceId;
    pub struct ShellId;
    pub struct LumpId;
    pub struct BodyId;
    pub struct CurveId;
    pub struct SurfaceId;
}
