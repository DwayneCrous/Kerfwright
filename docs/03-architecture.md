# Kerfwright — Rust Technical Architecture

Companion to `docs/01-research/` and `docs/02-PRD.md`. The Rust definitions referenced here are real code in `crates/` (compiled and tested; `kw-math`, `kw-nurbs`, `kw-brep`, `kw-tess`, `kw-io`, `kw-viewer` are implemented in Phase 1, `kw-sketch` is implemented in Phase 2, the kernel crates in Phase 3, `kw-feature`/`kw-expr`/`kw-sheet`/`kw-subd`/`kw-assembly`/`kw-script` in Phase 4, `kw-job`/`kw-mesh3d`/`kw-fem`/`kw-cam`/`kw-gsim`/`kw-ecad`/`kw-fabcheck`/`kw-render`/`kw-store` in Phase 5 — `kw-voronoi` was cut, see `docs/phase5-report.md` — and `kw-app` is the egui application with the Rayon ⇄ Tokio bridge, see `docs/app-shell.md`); this document shows the key ones in condensed form plus the diagrams.

## 1. Crate map and dependency direction
```
                    ┌────────────┐
                    │  kw-app    │  Rayon⇄Tokio bridge, Command/Event, cancellation
                    └─────┬──────┘
                          │
                    ┌─────▼──────┐
                    │ kw-feature │  DAG, TopoName, Reference, FeatureEvaluator
                    └──┬──────┬──┘
                       │      │
              ┌────────▼┐   ┌─▼─────────┐
              │ kw-sketch│   │  kw-brep  │  topology arenas, ShapeHistory, validation
              └────┬─────┘   └─────┬─────┘
                   │               │
                   │         ┌─────▼─────┐
                   └────────►│ kw-nurbs  │  KnotVector, NURBS, analytic geometry (Phase 1)
                             │ ▲ kw-math │  Tolerance, Frame3, Transform3, predicates
                             └───────────┘
 implemented in Phase 1: kw-nurbs (between kw-math and kw-brep), kw-tess, kw-io, kw-viewer
 planned: kw-ssi kw-boolean kw-blend kw-step kw-iges kw-subd kw-sheet
          kw-cam kw-voronoi kw-mesh3d kw-fem kw-ecad kw-collab kw-ui
```
Rule: arrows only point downward; `kw-math` has no kernel knowledge; UI and renderer depend on read-only snapshots, never on mutable kernel state.

## 2. Domain models (condensed from the crates)

### 2.1 B-rep topology (`kw-brep`)
```rust
new_key_type! { VertexId; EdgeId; CoedgeId; LoopId; FaceId; ShellId; LumpId; BodyId; CurveId; SurfaceId; }

pub struct BRep {
    vertices: SlotMap<VertexId, Vertex>,   edges:   SlotMap<EdgeId, Edge>,
    coedges:  SlotMap<CoedgeId, Coedge>,   loops:   SlotMap<LoopId, Loop>,
    faces:    SlotMap<FaceId, Face>,       shells:  SlotMap<ShellId, Shell>,
    lumps:    SlotMap<LumpId, Lump>,       bodies:  SlotMap<BodyId, Body>,
    curves:   SlotMap<CurveId, CurveGeom>, surfaces: SlotMap<SurfaceId, SurfaceGeom>,
}
pub struct Coedge { edge: EdgeId, owner_loop: LoopId, next: CoedgeId, prev: CoedgeId,
                    radial_next: CoedgeId, sense: Sense, pcurve: Option<CurveId> }
pub struct Edge   { start: VertexId, end: VertexId, curve: Option<CurveId>,
                    param_range: (f64, f64), tolerance: f64, first_coedge: Option<CoedgeId> }
pub struct Face   { shell: ShellId, surface: SurfaceId, sense: Sense, loops: Vec<LoopId> }
pub enum CurveGeom   { Line, Circle, Ellipse, Nurbs(NurbsCurve), Procedural(Box<ProceduralCurve>) }
pub enum SurfaceGeom { Plane, Cylinder, Cone, Sphere, Torus, Nurbs(NurbsSurface),
                       Extrusion{..}, Revolution{..}, Procedural(Box<ProceduralSurface>) }
pub struct ShapeHistory { origins: HashMap<EntityRef, Origin>, deleted_inputs: HashSet<EntityRef> }
```

### 2.2 Constraint entities (`kw-sketch`)
```rust
pub enum EntityKind { Point(PointVars), Line{p1,p2}, Circle{center,radius: VarId}, Arc{..},
                      Ellipse{..}, Spline{degree, control, weights}, FitSpline{points} }
pub enum ConstraintKind { Coincident, PointOnCurve, Horizontal, Vertical, Parallel, Perpendicular,
    Collinear, Concentric, EqualLength, EqualRadius, Midpoint, Symmetric, Fix, Tangent{branch},
    Joint{ a, a_end, b, b_end, level: Continuity /*G0..G3*/, branch },
    Distance{value: VarId, branch}, Angle{value, branch}, Radius{..}, Diameter{..} }
pub trait Residual: Send + Sync { fn arity(&self)->usize; fn variables(&self)->&[VarId];
    fn eval(&self,x:&[f64],out:&mut [f64]); fn jacobian(&self,x:&[f64],out:&mut Vec<SparseEntry>); }
pub trait ConstraintSolver { fn solve(..)->SolveOutcome; fn analyze(..)->DofReport; fn drag(..)->SolveOutcome; }
```

### 2.3 DAG nodes and naming (`kw-feature`)
```rust
pub struct DagNode { kind: NodeKind, name: String, suppressed: bool, state: EvalState }
pub enum EvalState { Clean, Dirty, Suspended, Failed(String) }
pub struct FeatureDag { graph: StableDiGraph<DagNode, ()> }   // edge: dependency -> dependent
pub struct TopoName { feature: NodeId, origin: NameOrigin, ordinal: u32 }
pub enum NameOrigin { SketchEdge{..}, Generated{from,role}, Split{parent,ordinal}, Merge(Vec<TopoName>), Primitive{role} }
pub struct Reference { name: TopoName, fingerprint: GeomFingerprint, tolerance: f64 }
pub enum ResolveOutcome { Exact(..), Split(..), Matched{..}, Ambiguous{..}, Broken }
pub trait FeatureEvaluator: Send + Sync { fn evaluate(&self, node:&DagNode, id:NodeId, inputs:&[Arc<EvalOutput>]) -> Result<EvalOutput, EvalError>; }
```

## 3. Memory layouts

### 3.1 Generational key (64 bit)
```
 63                         32 31                          0
┌──────────────────────────────┬──────────────────────────────┐
│  version (u32, odd=occupied) │  slot index (u32)            │   slotmap::KeyData
└──────────────────────────────┴──────────────────────────────┘
 stale key  ⇒  version mismatch  ⇒  lookup returns None (never aliases a new entity)
```

### 3.2 One body's arenas (per-body, contiguous, cache-friendly)
```
BRep ──┬─ vertices : [ver|Vertex{pt:3×f64,tol}] …              (hot: tessellation, stitching)
       ├─ edges    : [ver|Edge{v0,v1,curve,range,tol,first_coedge}] …
       ├─ coedges  : [ver|Coedge{edge,loop,next,prev,radial_next,sense}] …   (hot: traversal)
       ├─ loops/faces/shells/lumps/bodies : small, pointer-chasing only at build time
       ├─ curves   : [ver|CurveGeom]   ◄── shared by many edges (split faces reuse a surface)
       └─ surfaces : [ver|SurfaceGeom]

 Topology walk:           Face ─loops→ Loop ─first_coedge→ Coedge ─next→ Coedge …(closed ring)
 Radial ring per edge:    Edge ─first_coedge→ C₁ ─radial_next→ C₂ ─radial_next→ … ─→ C₁
                          (len 1 = sheet boundary, 2 = manifold, ≥3 = non-manifold)
```
Hot-path optimisation (Phase 3+): `BRep` exposes read-only *struct-of-arrays* snapshots (`Vec<[f64;3]>` positions, `Vec<u32>` coedge links) for Rayon traversals without per-element key lookups.

### 3.3 Evaluation cache entry
```
HashMap<InputHash(u128), Arc<EvalOutput>>
EvalOutput ─ brep: Arc<BRep> ─ names: NameMap ─ history: ShapeHistory ─ output_hash
InputHash = H( kernel_version ‖ feature id ‖ spec with every expression evaluated ‖ body input ‖ [ input.output_hash … ] )
output_hash = H( geometry hash (canonical, 1e-7 mm) ‖ name-set hash )   — equal output_hash ⇒ dependents stay cache hits (early cutoff)
Unchanged bodies = Arc clone (O(1)); edits create new Arc, old kept for undo.
```

## 4. State synchronisation

```
        UI thread (winit + egui)                      Engine (owns document)
   ┌──────────────────────────────┐          ┌───────────────────────────────────────┐
   │ input → Command              │─cmd tx──►│ Command loop (single writer)           │
   │                              │          │   Doc = Arc<DocState>  (immutable)      │
   │ render ◄─ Arc<DocSnapshot> ◄─┼─watch rx─┤   apply(cmd) → new DocState → publish   │
   │ (read-only, lock-free)       │          │   undo stack = Vec<Arc<DocState>>       │
   └──────────────┬───────────────┘          └───────────────┬───────────────────────┘
                  │ Event (progress, errors)                  │ dirty set
                  ◄───────────────────────────────────────────┘
                                                              ▼
                                             Evaluation scheduler (DAG, topological waves)
```
* **Single writer, many readers**: only the command loop creates new `DocState`; readers hold `Arc` snapshots → no locks on the render path.
* **Snapshot** = `{ dag, parameters, eval_cache_index, tessellations }`; persistent (structural-sharing) maps keep per-edit cost O(changed nodes).
* **Supersession**: each interactive command takes `CancelToken::next()`; in-flight evaluations of older generations return `Cancelled` at their next checkpoint (every ≤ 1 ms of kernel work).
* **Sketch drag** bypasses the DAG: `Priority::Realtime` command → incremental cluster solve → direct snapshot patch → full DAG re-evaluation debounced to drag-end (or 50 ms).

## 5. Thread communication (Rayon + Tokio)

```
┌────────────────────────────── process ──────────────────────────────────┐
│                                                                          │
│  Main/UI thread ──────────────► winit event loop + egui + wgpu submit    │
│        ▲  │ Command (mpsc)                          ▲ Event (mpsc)       │
│        │  ▼                                          │                    │
│  ┌───────────────────────────┐  spawn_compute()  ┌───┴──────────────────┐│
│  │ Engine task (Tokio,       │──────────────────►│ Rayon pool            ││
│  │ 1 logical writer)         │◄── oneshot ───────│ N = cores − 2         ││
│  │  • command loop           │    result         │  • SSI/Boolean/fillet ││
│  │  • DAG scheduler          │                   │  • constraint clusters││
│  └───────────┬───────────────┘                   │  • tessellation       ││
│              │                                   │  • FEA assembly/solve ││
│              │ async I/O                         │  • CAM toolpaths      ││
│  ┌───────────▼───────────────┐                   └───────────────────────┘│
│  │ Tokio multi-thread runtime│                                            │
│  │  • file I/O, autosave     │      GPU upload thread (wgpu queue)        │
│  │  • collab WebSocket/QUIC  │◄──── staging buffers from tessellation     │
│  │  • job queue (CAM/FEA)    │                                            │
│  └───────────────────────────┘                                            │
└──────────────────────────────────────────────────────────────────────────┘
Rules
 R1  CPU-bound work NEVER runs on Tokio workers; use `spawn_compute` (rayon::spawn + oneshot).
 R2  Rayon tasks NEVER block on Tokio futures (no `block_on` inside the pool).
 R3  Rayon closures capture `Arc` snapshots; no shared mutable state, no locks in kernel code.
 R4  Determinism: parallel reductions use fixed chunking and ordered merges; naming-affecting
     iteration is sorted by canonical keys.
 R5  Priority lanes: Realtime (drag) > Interactive (edit) > Background (CAM/FEA); background
     jobs use a separate low-priority Rayon pool so they cannot starve interactive solves.
 R6  Every long task polls `CancelToken::is_cancelled()` at loop heads.
```

## 6. Evaluation wave schedule (partial re-evaluation)
```
 edit Sketch2.dim ──► mark_dirty (DFS) ──► toposort(dirty) ──► waves by in-degree
   Sketch1 ✓cache        Wave 0: Sketch2*                       (parallel within wave)
   Sketch2*  dirty       Wave 1: Extrude2*, Hole1*  ──┐ rayon::scope
   Extrude1 ✓cache       Wave 2: Boolean2*          ◄─┘
   Extrude2* dirty       Wave 3: Fillet1*, Pattern1* ──► early cutoff if output_hash unchanged
   …                     Tessellation of changed bodies only → GPU upload
```

## 7. Rendering data path
```
 EvalOutput.brep ──tessellate(face,edge)──► MeshChunk{ pos f32×3, nrm oct16×2, face_id u32 } ─┐
                                           EdgePolyline{ pos f32×3 (camera-relative), edge_id }│
 wgpu:  [Depth+ID pass] → [Opaque PBR] → [WBOIT accum/reveal] → [GTAO] → [Edge lines] → [Composite]
        ID buffer (R32Uint) → async readback → hover/selection → Command::Select
```

## 8. Cross-cutting
* **Errors**: kernel and solver functions return `Result<_, KernelError>`; no `unwrap`/panics in shipping paths; `catch_unwind` at the evaluator boundary converts any residual panic into `EvalState::Failed` with a crash report.
* **Testing**: `proptest` for math/topology invariants; golden `insta` snapshots for names and G-code; `criterion` benches gated in CI; `cargo-fuzz` for importers/Booleans.
* **Observability**: `tracing` spans per DAG node, solver cluster, and frame; flame graphs from CI perf runs.
