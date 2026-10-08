# Kerfwright

> *kerf* — the width of material a cutting tool removes; *-wright* — a maker (shipwright, wheelwright).
> **Kerfwright**: a pure-Rust, clean-room CAD/CAM/CAE/ECAD suite.

This repository currently holds the **research, specification, and architecture blueprint** plus a Cargo workspace of domain-model skeletons. There is no working application yet.

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
  kw-math      tolerance, NURBS data types, knot-vector basis functions
  kw-brep      slotmap-keyed B-rep topology (coedge + radial ring), geometry enums, shape history
  kw-sketch    sketch entities, constraints, DOF report, solver traits
  kw-feature   feature DAG, persistent topological names, references, evaluator traits
  kw-app       Rayon ⇄ Tokio bridge, command/event types, cancellation
```

## Status / caveats
* The Rust crates are **type and trait definitions with a few small algorithms**. They were written without a Rust toolchain on the authoring machine, so **they have not been compiled yet** — run `cargo check --workspace` first and fix whatever the compiler finds.
* Statements about proprietary kernels (ASM, Parasolid, ACIS) are labelled **[Public]** or **[Inferred]** in the research docs; nothing here is derived from Autodesk code or binaries.
* Performance numbers in the PRD are targets, not measurements.
