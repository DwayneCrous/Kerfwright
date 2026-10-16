---
title: Crate map
description: "Every crate, the layer it belongs to, the dependency direction and the rules that keep the kernel clean."
sidebar:
  order: 1
---

Kerfwright is a Cargo workspace of small crates. The layering is strict, because it is what lets the kernel be tested without a window and the UI be replaced without touching geometry.

## The spine

```text
kw-app        egui shell, Rayon ⇄ Tokio bridge, Command / Event, cancellation
  └─ kw-feature   DAG, persistent topological names, incremental evaluator
       ├─ kw-sketch   entities, residuals, DM decomposition, sparse LM solver
       └─ kw-brep     slotmap topology arenas, ShapeHistory, validation
            └─ kw-nurbs   knot vectors, NURBS, analytic geometry
                 └─ kw-math   tolerance, frames, transforms, robust predicates
```

**The rule:** arrows point downward only. `kw-math` knows nothing about the kernel. The UI and the renderer read immutable snapshots, never mutable kernel state.

## All crates, by layer

| Layer | Crate | What it holds |
|---|---|---|
| Foundation | `kw-math` | Tolerance, frames, rigid transforms, bounding boxes, robust orientation and in-circle predicates |
| | `kw-nurbs` | Knot vectors and basis functions, NURBS curves and surfaces with derivatives, knot insertion and removal, degree elevation, Bézier extraction, fitting, closest point, analytic geometry |
| | `kw-job` | Cancel flag, progress, a capped thread pool. A leaf crate so the mesher and FEM do not depend on the feature engine |
| Topology | `kw-brep` | Slotmap B-rep with coedge and radial ring, validation, primitive builders, `ShapeHistory`, mass properties, sewing |
| | `kw-tess` | Crack-free edge-first tessellator |
| Kernel | `kw-ssi` | Surface–surface intersection: exact quadric cases and certified level-set tracing |
| | `kw-boolean` | Union, intersection and subtraction with verification and healing |
| | `kw-model` | Profiles to extrude, revolve, sweep and loft; shell, offset, draft; the surface builders |
| | `kw-blend` | Constant and variable radius fillets, chamfers |
| | `kw-analysis` | Curvature, draft angle, zebra stripes, edge continuity (G0, G1, G2) |
| Sketching | `kw-sketch` | Sketch entities, the residual library and solver, DOF analysis, drag, trim, extend, offset, the toolkit-independent `Editor` |
| | `kw-dxf` | ASCII DXF R2000 reader and writer with sketch adapters |
| | `kw-sketcher` | The standalone egui sketcher binary |
| Parametric | `kw-expr` | Expressions with units |
| | `kw-feature` | Feature document, timeline, incremental engine, persistent names, the `.kwf` format |
| | `kw-assembly` | Parts, instances, joints, interference, mass, STEP export |
| | `kw-sheet` | Sheet-metal strips, bend allowance, flat pattern |
| | `kw-subd` | Catmull–Clark subdivision with creases, finish to a B-rep of bicubic patches |
| | `kw-script` | Rhai scripting over the document builder |
| | `kw-materials` | The material library |
| Exchange | `kw-step` | ISO 10303-21 parser, AP203, AP214 and AP242 B-rep import and export |
| | `kw-iges` | IGES 5.3 import |
| | `kw-io` | STL, OBJ with MTL, glTF and GLB writers |
| Manufacture | `kw-cam` | 2.5-D operations, drop-cutter, tri-dexel stock, five-axis kinematics, posts |
| | `kw-gsim` | Independent interpreters for five G-code dialects |
| Simulate | `kw-mesh3d` | Tetrahedral mesher, TET10, quality |
| | `kw-fem` | TET10 static, modal, thermal, nonlinear and SIMP, stress recovery |
| Electronics | `kw-ecad` | Board model, placement, router, DRC, pour, Gerber, Excellon, IDF, STEP, pick-and-place |
| | `kw-fabcheck` | Two Gerber viewers and an Excellon parser for checking the outputs |
| Output | `kw-viewer` | The `wgpu` viewer: PBR, edges, ID-buffer picking, windowed and headless |
| | `kw-render` | WBOIT, GTAO, edge frame, LBVH path tracer, denoiser |
| | `kw-drawing` | 2-D sheets, hidden-line views, sections, details, dimensions, parts list, PDF and DXF |
| | `kw-store` | `cam`, `fea`, `board` and drawing entries inside the `.kwf` container |
| Application | `kw-app` | The desktop application |

## Why independent checkers exist

Several crates exist only to *check* others, and they deliberately share no code with what they check: `kw-gsim` replays G-code from `kw-cam`, and `kw-fabcheck` reads Gerber from `kw-ecad`. A bug that was written twice the same way is still possible, which is why the reports call these checkers "ours" and not third-party.

## Design decisions worth knowing

- **Coedge with radial ring B-rep** in slotmap arenas, and every entity has a typed, generational id.
- **The kernel returns `Result`.** Failures in Booleans and importers are reported errors, never panics, and a `catch_unwind` at the evaluator boundary turns any remaining panic into a failed feature.
- **Determinism.** Same inputs give bit-identical results on 1 to 32 threads. See [Determinism](/docs/concepts/determinism/).
- **Independent implementation.** See [Provenance and IP policy](/docs/project/clean-room/).

The full architecture document, with memory, state and thread diagrams, is [`docs/03-architecture.md`](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/03-architecture.md).
