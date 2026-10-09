# Kerfwright

> *kerf* — the width of material a cutting tool removes; *-wright* — a maker (shipwright, wheelwright).
> **Kerfwright**: a pure-Rust, clean-room CAD/CAM/CAE/ECAD suite.

This repository holds the **research, specification and architecture blueprint** plus a Cargo workspace. **Phase 1 (math foundation, kernel skeleton, tessellator, exporters, wgpu viewer) is implemented** — see [`docs/phase1-report.md`](docs/phase1-report.md) for results against the acceptance criteria and a list of known gaps. **Phase 2 (sketcher & constraint solver, DXF, egui sketcher app) is implemented** — see [`docs/phase2-report.md`](docs/phase2-report.md). **Phase 3 (B-rep modelling kernel: SSI, Booleans, extrude/revolve/sweep/loft, fillet/chamfer, shell/offset/draft, mass properties, STEP, IGES) is implemented** — see [`docs/phase3-report.md`](docs/phase3-report.md) for the measured results, the criteria that are only partly met, and the oracle set-up. Phases 4–6 are not started.

## Read in this order
| # | File | What it is |
|---|---|---|
| 0 | [`CLEAN_ROOM.md`](CLEAN_ROOM.md) | Rules on sources, licensing, naming; epistemic labels used in the docs |
| 1 | [`docs/01-research/01-brep-kernel.md`](docs/01-research/01-brep-kernel.md) | B-rep topology, NURBS, SSI, Booleans, fillets, T-splines vs SubD |
| 1 | [`docs/01-research/02-constraint-solver.md`](docs/01-research/02-constraint-solver.md) | 2D constraints, G0–G3, LM/Newton/BFGS, decomposition, DOF analysis |
| 1 | [`docs/01-research/03-parametric-dag.md`](docs/01-research/03-parametric-dag.md) | Topological naming, feature DAG, incremental evaluation |
| 1 | [`docs/01-research/04-multidomain.md`](docs/01-research/04-multidomain.md) | CAM, FEA, ECAD, `wgpu` rendering |
| 1 | [`docs/01-research/05-rust-ecosystem.md`](docs/01-research/05-rust-ecosystem.md) | Crate evaluation and gap analysis |
| 2 | [`docs/02-PRD.md`](docs/02-PRD.md) | Product requirements, 8 workspaces, SLAs, file formats |
| 3 | [`docs/03-architecture.md`](docs/03-architecture.md) | Rust domain models, ASCII memory/state/thread diagrams |
| 3 | [`docs/04-roadmap.md`](docs/04-roadmap.md) | 6-phase plan with acceptance criteria |
| — | [`LICENSES/patent-watch.md`](LICENSES/patent-watch.md) | Patent items needing counsel review |

## Workspace layout
```
crates/
  kw-math      tolerance, frames, transforms, AABB, robust predicates
  kw-nurbs     knot vectors, NURBS curves/surfaces (derivatives, insertion/removal,
               elevation, Bézier extraction, fitting, closest point), analytic geometry
  kw-brep      slotmap B-rep (coedge + radial ring), validation, primitive builders, shape history
  kw-tess      crack-free edge-first tessellator
  kw-io        STL / OBJ+MTL / glTF / GLB writers
  kw-viewer    wgpu viewer (PBR, edges, ID-buffer picking); windowed + headless
  kw-sketch    sketch entities, residual library + analytic Jacobians, DM decomposition,
               sparse LM solver, DOF/redundancy/conflict analysis, drag solver,
               trim/extend/offset, toolkit-independent Editor
  kw-dxf       ASCII DXF (R2000) reader/writer + sketch adapters
  kw-sketcher  egui sketcher application (binary)
  kw-ssi       surface–surface intersection (exact quadric cases + certified level-set tracing, signed-distance field for free-form surfaces, pcurves on both surfaces)
  kw-boolean   union / intersect / subtract (imprint, arrangement, classification, stitching, ShapeHistory)
  kw-model     profiles → extrude / revolve / sweep / loft (open and closed); shell, offset, draft; rebuild engine
  kw-blend     constant- and variable-radius (any law) fillets, vertex blends, chamfers (swept-tool Booleans)
  kw-step      ISO 10303-21 parser, AP203/214/242 B-rep import and export (`step_tool` example for oracle checks)
  kw-iges      IGES 5.3 import (trimmed surfaces → sewn solids)
  kw-feature   feature DAG, persistent topological names, evaluator trait, Phase 3 kernel evaluator
  kw-app       Rayon ⇄ Tokio bridge, command/event types                (skeleton)
```

## Try it
```bash
# needs Rust (rustup) and a Vulkan/Metal/DX12 GPU
cargo run --release -p kw-viewer                       # window: drag = orbit, right-drag = pan, wheel = zoom, E = edges, F = frame
cargo run --release -p kw-viewer -- --scene million    # 1M-triangle scene
cargo run --release -p kw-viewer -- --headless --scene primitives --pick-test
cargo run -p kw-io --example export_primitives -- out/ # STL / OBJ / glTF / GLB of every primitive
scripts/phase1_acceptance.sh                           # every automated Phase 1 check
cargo run --release -p kw-sketcher -- --demo           # 2D sketcher (L line, R rectangle, C circle, D dimension, Ctrl+Z undo)
scripts/phase2_acceptance.sh                           # every automated Phase 2 check
scripts/phase3_acceptance.sh                           # every automated Phase 3 check (100,000-pair Boolean fuzz: ~25 min)
```

## Status / caveats
* Phase 3: Boolean fuzz (99.80 % on valid-expected cases, 0 invalid outputs), free-form-body fuzz (99.83 %), SSI (including spline/spline), fillet benchmark, vertex blends, variable-radius fillets, STEP/IGES and public-corpus criteria are met or measured as described in the Phase 3 report, and the Boolean (500 ms) and fillet (300 ms) performance SLAs are met at the stated sizes. Not done: vertex blends beyond three equal-radius planar faces (other corners meet in a mitre), variable radius on curved edges that are not circular rims, setback, and "≥ 3 external CAD tools" is satisfied by OpenCASCADE, Gmsh (OpenCASCADE-based) and the independent truck reader — see the report for exactly what each tool verified.
* Phase 2 meets all six automated criteria; the drag-latency SLA is asserted on decomposable sketches and is exceeded on row-coupled ones (8.6 ms p99 at 500 entities), G3/mirror/pattern are not done, and the sketcher window has been smoke-tested but not visually reviewed — details in the Phase 2 report.
* Phase 1 is complete against its automated criteria; the exceptions (published Piegl–Tiller examples beyond Ex. 2.3, opening exports in third-party viewers, "reference GPU" undefined) are listed in the Phase 1 report.
* `kw-app` is still a skeleton and `kw-feature` has only the thin Phase 3 kernel adapter (the parametric engine is Phase 4).
* Statements about proprietary kernels (ASM, Parasolid, ACIS) are labelled **[Public]** or **[Inferred]** in the research docs; nothing here is derived from Autodesk code or binaries.
* Performance numbers in the PRD are targets; measured Phase 1 numbers are in the report.
