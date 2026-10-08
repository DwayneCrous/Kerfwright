# 05 — Rust Ecosystem & Gap Analysis

> **Verification status.** Crate descriptions below come from public docs I could check this session (truck, fornjot, faer, tritet, delaunay, Weighted-Blended OIT in wgpu-based viewers) and from prior knowledge for the rest. **Pin versions and re-check maintenance status/licence before adopting any crate** — this table is a starting point, not an audit.

## 1. Adopt (production-ready building blocks)
| Crate | Role in Kerfwright | Notes |
|---|---|---|
| `wgpu` | Rendering (Vulkan/Metal/DX12/WebGPU) | Mature; compute shaders for culling, LBVH, path tracer. No portable hardware RT. |
| `nalgebra` | f64 small-matrix linear algebra, transforms | Fixed-size types are the right fit for kernel code. |
| `glam` | f32 SIMD math for GPU-facing data | Keep f64 in kernel, f32 only at the render boundary. |
| `faer` (+ sparse) | Dense QR/SVD/LU, sparse LLT/LDLT (supernodal) | Pure Rust, high performance on medium/large matrices; **not** designed for tiny vectors → don't use for 3×3 geometry. |
| `sprs` / `nalgebra-sparse` | CSR/CSC containers | Use whichever interoperates with faer's sparse API in the pinned versions. |
| `petgraph` (`StableGraph`) | Feature DAG, constraint graphs, toposort, SCC | No incremental evaluation — we build that on top. |
| `slotmap` | Generational arena keys for B-rep entities | Core to the topology store. |
| `rayon` | Data-parallel kernel work | |
| `tokio` | I/O, network, collaboration | Never run CPU-bound kernel work on Tokio workers. |
| `parry3d-f64` | Broad-phase BVH, ray casts, convex hulls, distance queries | Mesh/primitive-oriented, **not** B-rep aware; use for collision checks (CAM, assembly), not for Booleans. |
| `rstar` | R-tree spatial index | Stitching, DRC. |
| `robust` | Shewchuk adaptive predicates | Orientation/in-circle for meshing and polyhedral classification. |
| `spade` | 2D CDT + Voronoi (points) | Face tessellation in `(u,v)`. |
| `cavalier_contours` | Arc-aware polyline offset/boolean | CAM 2D offsetting. |
| `i_overlay` or Clipper2 port | Polygon booleans / offsets | Cleared-area model for adaptive clearing, copper pours. |
| `lyon` | 2D path tessellation | UI/sketch drawing. |
| `gltf`, `stl_io`, `tobj`, `dxf` | glTF/STL/OBJ/DXF I/O | Wrap behind Kerfwright importer traits for fuzzing and tolerance handling. |
| `serde`, `rkyv`/`bincode`, `zip` | `.kwf` container | |
| `egui`/`eframe` (or `iced`) | UI shell | Decide in Phase 1 spike. |
| `proptest`, `criterion`, `insta`, `approx`, `tracing` | Testing, benches, logging | |
| `num-dual` | Forward-mode AD | Test oracle for Jacobians. |

## 2. Evaluate (exists but likely insufficient)
| Crate/project | Status | Why not just adopt |
|---|---|---|
| **truck** (ricosjp; fork "monstertruck") | Pure-Rust B-rep with NURBS, topology, meshing, a Boolean crate (`truck-shapeops`) | Apache-licensed and a useful reference/benchmark. Boolean robustness, fillet, shell, tolerance model, history reporting are not at production level; API is generic-heavy. **Study + possibly vendor early prototypes; do not depend on its Booleans.** |
| **fornjot** | Repository reported archived (June 2026) | Useful design ideas (validation, stored topology); not a base. |
| `curvo` / other NURBS crates | Partial NURBS curve/surface eval and some intersection | Check fidelity (rational surfaces, derivatives to order 3, closest-point). |
| `opencascade-rs` | Bindings to OCCT (C++) | Violates "pure Rust". Useful as a *test oracle* in dev-only tooling. |
| `ruststep` | ISO 10303-21 parser + EXPRESS tooling | AP203/214 B-rep entity coverage incomplete. |
| `levenberg-marquardt`, `argmin` | Dense LM / general optimisers | No sparsity, no decomposition, no DOF analysis. |
| `tritet` | Wraps C Triangle/TetGen | Not pure Rust. |
| `delaunay` (D-dim) | Exact-predicate Delaunay + hulls | Needs boundary recovery + quality refinement for FEA. |
| `slvs` | SolveSpace bindings | C++ GPL dependency. |
| `rapier3d`/`parry` | Physics/collision | Overkill / wrong abstraction for CAD. |

## 3. Gaps — must be built (ordered by project risk)
| # | Component | Crate | Difficulty | Notes |
|---|---|---|---|---|
| 1 | **B-rep kernel**: topology store, tolerance model, validation, history reporting | `kw-brep` | ★★★★★ | Everything depends on this. |
| 2 | **Robust Boolean engine** (SSI, imprint, classify, stitch, heal) | `kw-boolean` | ★★★★★ | Multi-year; start with analytic+planar, widen. |
| 3 | **NURBS toolkit** (eval to order 3, knot ops, fitting, closest point, projection, intersection) | `kw-nurbs` | ★★★★ | Piegl–Tiller well documented. |
| 4 | **SSI** (analytic + marching + subdivision) | `kw-ssi` | ★★★★★ | |
| 5 | **Fillet / chamfer / shell / offset / draft** | `kw-blend` | ★★★★★ | Vertex blends hardest. |
| 6 | **Sparse clustered constraint solver + DOF analysis** | `kw-sketch` | ★★★★ | Spec in 02. |
| 7 | **Topological naming engine** | `kw-feature` | ★★★★ | Spec in 03. |
| 8 | **Incremental feature evaluator** | `kw-feature` | ★★★ | |
| 9 | **STEP AP203/214/242 B-rep read/write** | `kw-step` | ★★★★ | Large EXPRESS schema subset; import robustness is the product. |
| 10 | **IGES 5.3** (entities 126/128/142/144/186/…) | `kw-iges` | ★★★ | No Rust crate exists. |
| 11 | **SubD (Catmull–Clark) + Finish→B-rep**; T-spline (after legal review) | `kw-subd` | ★★★★ | |
| 12 | **Sheet-metal** (flange, hem, bend tables, unfold) | `kw-sheet` | ★★★ | |
| 13 | **Segment/arc Voronoi + medial axis** | `kw-voronoi` | ★★★★ | Needed for MAT-based CAM. |
| 14 | **CAM engine**: adaptive clearing, drop-cutter, 5-axis, stock sim, posts | `kw-cam` | ★★★★ | |
| 15 | **Pure-Rust quality tet mesher with surface recovery** | `kw-mesh3d` | ★★★★ | |
| 16 | **FEM assembly + solvers + topology optimisation** | `kw-fem` | ★★★ | Rest on faer. |
| 17 | **PCB router/placer/DRC + Gerber/IDF** | `kw-ecad` | ★★★ | |
| 18 | **GPU path tracer + denoiser** | `kw-render` | ★★★ | |
| 19 | **Collaboration (CRDT/OT) for feature recipes** | `kw-collab` | ★★★ | |

## 4. Build-vs-adopt policy **[Design]**
* Adopt where the crate is generic and replaceable (math, graphs, I/O formats, windowing).
* Build where correctness is the product (kernel, Booleans, solver, naming, exchange formats).
* Wrap every external crate behind a small Kerfwright trait/type at the boundary so it can be swapped.
* Dev-only oracles (OCCT via `opencascade-rs`, SolveSpace bindings) live in a separate `xtask`/`dev-oracles` workspace and are never linked into shipping binaries.

## Sources
- Truck — https://github.com/ricosjp/truck ; docs — https://docs.rs/monstertruck
- Fornjot — https://github.com/hannobraun/fornjot
- faer — https://docs.rs/faer
- tritet — https://docs.rs/crate/tritet/latest
- delaunay crate — https://docs.rs/crate/delaunay/latest
- slvs — https://docs.rs/slvs/ ; arael-sketch-solver — https://docs.rs/arael-sketch-solver
