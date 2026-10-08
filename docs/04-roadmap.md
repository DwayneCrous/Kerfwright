# Kerfwright — 6-Phase Implementation Roadmap

Assumptions: a team of ~8–12 senior engineers (2 kernel, 1 solver/numerics, 1 graphics, 1 formats/IO, 1 UI, 1 CAM, 1 FEA/ECAD, 1 test/infra). Durations are **planning estimates**, not commitments; a credible v1 of this scope is a multi-year programme, and Phases 3 and 5 are the schedule risk. Each phase ends with a *demoable vertical slice* and automated acceptance gates in CI.

| Phase | Months | Theme |
|---|---|---|
| 1 | 0–4 | Math foundation, kernel skeleton, viewport |
| 2 | 4–8 | Sketcher & constraint solver |
| 3 | 8–18 | B-rep modelling: SSI, Booleans, fillets, STEP |
| 4 | 18–26 | Parametric engine, naming, assemblies, sheet metal, SubD |
| 5 | 26–38 | CAM, FEA, ECAD, advanced rendering |
| 6 | 38–46 | Cloud collaboration, WASM client, hardening |

---

## Phase 1 — Core math foundation (months 0–4)
**Build:** `kw-math` (tolerance, predicates, transforms), `kw-nurbs` (curve/surface eval to order 3, knot insertion/removal, degree elevation, Bézier extraction, fitting, closest point), `kw-brep` topology store (slotmap arenas, coedge/radial ring, validation, Euler–Poincaré), primitive builders (box, cylinder, sphere, cone, torus), tessellator v1, `wgpu` viewer (PBR, edges, ID-buffer picking), STL/OBJ/glTF export, CI + benchmark harness.

**Acceptance criteria**
- NURBS eval matches published Piegl–Tiller worked examples to ≤ 1e-12; `∂ⁿC` (n ≤ 3) matches finite-difference oracle to 1e-6 relative; partition of unity of basis functions ≤ 1e-14 over randomized knot vectors (proptest, ≥ 10⁶ cases).
- Knot insertion/removal and degree elevation preserve geometry within 1e-10.
- All primitives pass `validate()` (Euler–Poincaré, loop closure, orientation, positive volume); fuzzed random topology mutations never produce an unreported invalid body.
- Tessellation is crack-free (zero T-junction vertices) and within the requested chordal tolerance on the primitive corpus.
- Viewport ≥ 60 FPS on a 1 M-triangle scene on the reference GPU; ID-buffer pick returns correct face/edge id on 100 % of a scripted click test.
- Exported glTF/STL/OBJ load without warnings in two independent third-party viewers.

## Phase 2 — Sketcher & constraint solver (months 4–8)
**Status: implemented 2026-10-08** — results, deviations and caveats in [`phase2-report.md`](phase2-report.md); reproduce with `scripts/phase2_acceptance.sh`.

**Build:** `kw-sketch` entities and full constraint library (including G0–G2), analytic sparse Jacobians, component + DM decomposition, clustered LM/dog-leg/BFGS solver on `faer`, DOF/redundancy/conflict analysis, drag solver, sketch UI (create/edit/dimension/trim/offset), DXF import/export.

**Acceptance criteria**
- Jacobians match finite-difference/AD oracles to 1e-7 for every constraint type across randomized configurations.
- Benchmark: 500-entity mixed-constraint sketch, *drag* solve p99 ≤ 1 ms; cold solve ≤ 25 ms; 5 000 entities cold ≤ 250 ms.
- Classification matches ground truth on a ≥ 300-case suite (under/fully/over/conflict), including special-position cases; conflicting set returned is minimal on ≥ 95 % of cases and always a superset of a minimal set.
- No branch flip during scripted drag of 100 reference sketches (tangent/angle/offset heavy).
- Solver output bit-identical across 1–32 threads.
- DXF round-trip of 2D entities within 1e-6.

## Phase 3 — B-rep modelling kernel (months 8–18)
**Build:** `kw-ssi` (analytic + marching + subdivision), `kw-boolean` (imprint/classify/stitch/heal), extrude/revolve/sweep/loft, `kw-blend` (constant-radius fillet + chamfer, then variable), shell/offset/draft, mass properties, STEP AP203/214/242 import/export, IGES import, `ShapeHistory` reporting from every op.

**Acceptance criteria**
- Boolean fuzz corpus (≥ 100 000 random primitive/NURBS pairs incl. tangent, coincident, near-miss): **0 invalid outputs** (every result passes `validate()`); ≥ 99 % of valid-expected cases succeed; failures return `Err` (never panic/loop).
- Volume of union/intersect/difference satisfies inclusion–exclusion within 1e-9 relative on analytic-volume cases.
- SSI: max distance of traced curve to both surfaces ≤ tolerance/10 on 10 000 analytic and NURBS pairs; no missed branches on the labelled-branch suite.
- Fillet success ≥ 95 % on a 500-part benchmark (constant radius, tangent chains, vertex blends of 3-edge corners); failures are reported with the offending edge.
- STEP round-trip with ≥ 3 external CAD tools: geometry deviation ≤ 1e-5, topology counts preserved; imports of a public 200-file STEP corpus produce valid solids for ≥ 95 %.
- Performance SLAs for Boolean (500 ms) and fillet (300 ms) met.

## Phase 4 — Parametric engine, assemblies, sheet metal, SubD (months 18–26)
**Build:** `kw-feature` DAG, expression engine with units, incremental evaluator with caching and early cutoff, topological naming (lineage names + fingerprints + repair UI), timeline UI (rollback/reorder/suppress), `.kwf` format v1, assemblies/joints, `kw-sheet` (flange/hem/unfold, DXF flat pattern), `kw-subd` (Catmull–Clark + creases, Finish→B-rep), scripting API.

**Acceptance criteria**
- **Naming corpus** (≥ 500 parametric edit scenarios: dimension change, reorder, insert upstream, pattern count change, face split/merge): downstream references resolve to the correct entity or report `Broken/Ambiguous`; **0 silent wrong resolutions**; ≥ 97 % resolve automatically.
- Determinism: rebuilding a document from `.kwf` without caches yields identical geometry hash and `TopoName` set on 1, 4, and 32 threads.
- Incremental evaluation: tail edit on 200-feature model ≤ 150 ms; full cold replay ≤ 2 s; early cutoff verified (no-op edits trigger 0 dependent re-evaluations).
- Cycle-introducing reorder is rejected with an explanatory error in 100 % of test cases.
- Sheet metal: unfold→refold round trip volume error ≤ 1e-6 relative; flat-pattern lengths match analytical bend allowance within 1e-6 mm on test suite.
- SubD→B-rep deviation ≤ 0.01 mm (default) with G1 continuity across patch boundaries verified numerically; output passes `validate()`.
- `.kwf` forward/backward compatibility tests across 3 schema versions.

## Phase 5 — CAM, FEA, ECAD, advanced rendering (months 26–38)
**Build:**
- `kw-cam`: offsetting, `kw-voronoi` medial axis, adaptive clearing, contour/pocket/drill, drop-cutter 3D, tri-dexel stock simulation, 3+2 and swarf 5-axis, machine kinematics/IK, post-processor DSL and 5 dialects.
- `kw-mesh3d` + `kw-fem`: Delaunay/advancing-front TET10 meshing, assembly, direct + PCG solvers, modal, SIMP topology optimisation.
- `kw-ecad`: board model, placement, A\*/Lee router, DRC, Gerber/Excellon/IDF/STEP export.
- `kw-render`: WBOIT, GTAO, LBVH path tracer + denoiser.

**Acceptance criteria**
- CAM: adaptive clearing never exceeds target engagement angle by > 5 % on the 50-pocket suite; zero gouges and zero tool/holder collisions on the verification suite; simulated stock matches analytic removed volume within 0.5 %; G-code from each dialect executes in an independent simulator (e.g. LinuxCNC/`gcode` viewer) without errors; 5-axis IK round-trips (FK(IK(pose)) = pose) within 1e-9 for all test machines; singularity handling produces no axis jumps > limit.
- FEA: cantilever, plate-with-hole, and pressure-vessel benchmarks within 2 % of analytic/NAFEMS references with TET10; mesh quality: ≥ 99 % of elements with radius-edge ratio ≤ 2 and no inverted elements; 100 k-DOF static ≤ 10 s; PCG residual 1e-8 reached on 1 M DOF ≤ 120 s; SIMP MBB-beam benchmark reproduces published compliance within 5 %.
- ECAD: A\* router completes ≥ 98 % of nets on the 20-board benchmark with 0 DRC violations; Gerber output validates in two independent viewers; IDF/STEP board opens in the MCAD workspace with correct component placement.
- Rendering: 60 FPS at 1 M triangles with OIT+GTAO+edges on reference GPU; path-tracer converges to ≤ 1 % mean-squared error vs. a reference image after N spp with denoiser PSNR ≥ 30 dB.

## Phase 6 — Cloud real-time collaboration & hardening (months 38–46)
**Build:** `kw-collab` (CRDT for feature recipes: ordered list w/ stable IDs, LWW parameters, move ops), sync server on Tokio (WebSocket/QUIC, auth, presence), versioning/branching/merge of documents, server-side headless evaluation workers (cached by `input_hash`), WASM/WebGPU viewer + light editor, background CAM/FEA job queue, telemetry (opt-in), crash recovery, signed installers, accessibility pass, security review.

**Acceptance criteria**
- Convergence: 10 clients applying random concurrent edits (property tests, ≥ 10⁵ runs) always converge to identical documents; reorder/move conflicts never create DAG cycles (invariant checked after every merge).
- Latency: remote edit visible to peers p95 ≤ 150 ms (same region); full re-evaluation on peers consistent (identical geometry hash) in 100 % of runs.
- Offline edits for ≥ 1 hour merge without data loss in the scripted scenario suite.
- WASM viewer opens a 100-feature `.kwf` and renders at ≥ 30 FPS in current Chrome/Firefox/Safari WebGPU builds.
- Security: external review with no unresolved high-severity findings; importers fuzzed ≥ 72 CPU-hours with no crashes.
- Soak test: 24 h with 50 simulated users, 0 data loss, memory growth < 5 %.

---

## Cross-phase tracks
- **Test oracles:** dev-only OCCT bindings and public STEP corpora for differential testing; never shipped.
- **Benchmarks in CI** with regression gates (> 10 % slowdown fails the build).
- **Fuzzing:** `cargo-fuzz` targets for every parser and the Boolean/fillet entry points.
- **Legal:** patent-watch (`LICENSES/patent-watch.md`) cleared before Phases 4 (T-splines) and 5 (adaptive clearing).
- **Risk register (top 5):** (1) Boolean/SSI robustness; (2) vertex-blend fillets; (3) STEP import breadth; (4) naming ambiguity on symmetric parts; (5) scope creep in Phase 5 — mitigate via vertical slices and cutting P2 items first.
