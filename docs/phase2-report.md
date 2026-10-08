# Phase 2 — Implementation Report (sketcher & constraint solver)

Status: **implemented; all automated acceptance checks pass** on the development machine (2026-10-08).
Reproduce everything with:

```bash
scripts/phase2_acceptance.sh        # ~3 min; fmt, clippy, all tests, release acceptance suites, UI smoke run, benches
```

Machine: 16-thread CPU, AMD Radeon 860M iGPU (RADV/Vulkan), Rust 1.99.0. Nothing is committed to git.

## What was built

| Crate | Contents |
|---|---|
| `kw-sketch` | Entities: point, line, circle, arc (CCW, implicit radius-consistency row), ellipse, clamped-uniform B-spline, fit spline. **Residual library** (one enum, no vtables): coincident, point-on-line/circle/arc/ellipse, horizontal/vertical, parallel, perpendicular, collinear, concentric, equal length/radius, midpoint, symmetric, fix, tangent (line–circle/arc, circle/arc–circle/arc, with branch), distances (pt–pt, pt–line, line–line), angle, radius/diameter, and **G0/G1/G2 joints** between lines, arcs and B-splines (via linear end-derivative evaluators). `eval` is generic over `f64` / a forward-mode dual number, so the same code is the AD oracle; the **Jacobians are hand-written, sparse and analytic**. **Solver**: connected components → Dulmage–Mendelsohn block-triangular decomposition (Hopcroft–Karp + iterative Tarjan) → clustered damped Gauss–Newton/LM on `faer` sparse Cholesky with cached symbolic factorisation → fallbacks (cluster merge, re-decomposition, dense LM / dog-leg / BFGS). **Analysis**: DOF, free entities, redundant constraints, minimal conflicting set (deletion filter). **Drag solver**: pin → region → sparse LM, soft-pull then exact polish, flip prevention. **Ops**: trim, extend, offset. **`Editor`**: toolkit-independent tools (select, point, line, rectangle, circle, arc, spline, dimension, 13 constraint tools, trim, extend, offset), snapping, H/V inference, undo/redo, live drag, DOF colouring states, conflict rollback. |
| `kw-dxf` | ASCII DXF (AC1015) writer + reader: POINT, LINE, CIRCLE, ARC, ELLIPSE, LWPOLYLINE (bulges → arcs), SPLINE (degree/knots/control/weights/fit points); layers (construction geometry on its own layer); shortest-round-trip doubles (bit-exact write→read); sketch adapters with endpoint welding (1e-6). |
| `kw-sketcher` | egui/eframe 0.36 (wgpu backend) front end: canvas with pan/zoom/grid, tool + constraint panels, solver/constraint inspector with editable dimensions, dimension-value dialog, DXF import/export dialog, undo/redo, live drag, DOF colouring (blue free, green constrained, yellow redundant, red conflicting, orange selected), snap markers and rubber band. CLI: `--open FILE.dxf`, `--demo`, `--exit-after-frames N`. |

## Acceptance criteria

| # | Criterion (docs/04-roadmap.md) | Result | Evidence |
|---|---|---|---|
| 1 | Jacobians match FD / AD oracles to 1e-7 for every constraint type | **Met**: 10⁶ randomized cases (random constraint kind, random valid geometry), analytic vs central FD **and** vs forward-mode AD, max relative error < 1e-7; a coverage test fails if any `Kind` is missing from the spec list and another checks the structural sparsity pattern contains every non-zero. Joint specs include spline control points forced apart (coincident control points make FD itself inaccurate). | `kw-sketch/tests/jacobians.rs` (`PROPTEST_CASES=1000000`, 3.5 s release) |
| 2 | 500 entities: drag p99 ≤ 1 ms, cold ≤ 25 ms; 5 000 entities cold ≤ 250 ms | **Met on decomposable sketches; not met on every structure** (see caveat). 493 entities: cold 0.44 ms, drag p50 0.022 / p95 0.060 / **p99 0.221 ms** over 4 000 frames. 4 998 entities: cold 1.5 ms. Criterion: drag frame 32 µs, cold 500 0.58 ms, cold 5 000 2.4 ms. Asserted in `--release` only. | `kw-sketch/tests/perf_sla.rs`, `benches/solver.rs` |
| 3 | Classification matches ground truth on ≥ 300 cases incl. special positions; conflict set minimal on ≥ 95 % and always ⊇ a minimal set | **Met**: 344 cases (160 under, 68 fully, 52 over/redundant, 64 conflicting), classification 100 %; all 64 conflicts are a superset of the planted minimal set and **64/64 exactly minimal**. Special positions (parallel/collinear/concentric/symmetric degeneracies where counting and rank disagree) are in the suite. | `kw-sketch/tests/classification.rs` |
| 4 | No branch flip during scripted drag of 100 reference sketches | **Met**: 100 sketches (tangent chains, rounded rectangles, offset polylines, spline chains, angle fans), 18 000 drag frames: **0 flips, 0 jumps, 0 unconverged**. Non-vacuity check: with prevention disabled the same scripts flip 3 281 times. | `kw-sketch/tests/branch_flip.rs` |
| 5 | Bit-identical output across 1–32 threads | **Met**: solve and drag compared bit-for-bit on rayon pools of 1, 2, 4, 8, 16, 32 threads against a single-threaded reference. Parallelism is across independent components/clusters with results written back by index; there are no cross-thread float reductions. | `kw-sketch/tests/determinism.rs` |
| 6 | DXF round trip of 2D entities within 1e-6 | **Met**: sketch → DXF text → parse → sketch matches to 1e-6 over random point/line/circle/arc/ellipse/B-spline/fit-spline sets with construction flags (20 000 cases in the acceptance run); the DXF document itself is bit-exact through write→parse; the independent `dxf` 0.6 crate reads our output with identical geometry, and we read files that crate writes; LWPOLYLINE bulge sign convention verified; 4 fuzz properties (arbitrary text, mutated/truncated files) never panic. | `kw-dxf/tests/roundtrip.rs` |

## Deviations from the plan

* **UI packaging**: the roadmap/user choice was "egui panels in `kw-viewer`". eframe cannot share `kw-viewer`'s hand-written winit loop, so the sketch UI is the separate `kw-sketcher` binary on the same stack (wgpu 30).
* **`faer` is used only for sparse Cholesky** of the normal equations (large clusters). Dense work (SVD for rank, dog-leg, BFGS) uses `nalgebra`.
* **G3 joints, mirror and pattern operations were not implemented.** G0–G2 are.

## Honest caveats

* **Drag p99 depends on structure.** The ≤ 1 ms SLA is asserted on decomposable sketches (many features, each a small block — what real parametric sketches look like). A sketch whose constraints chain every entity to the next in a row has long dependency chains: 493 entities show **p99 8.6 ms** (p50 0.43 ms) per drag frame, 4 998 entities p99 1.6 ms. These numbers are printed by `coupled_structures_are_reported` but not asserted. The largest single irreducible block benchmarked is a 20-bay truss (41 entities, 78 unknowns, cold 0.11 ms); a genuinely large irreducible block (hundreds of mutually coupled entities) was not benchmarked.
* **Ground truth is by construction.** The 344 cases come from our own generators (sketches built around a hidden solution, planted minimal conflicts, special-position templates), not from an external CAD system. Coverage is as good as the generators.
* **Flip prevention is conservative.** In the flip suite 11 181 of 18 000 frames were shortened or refused (the cursor target was clipped to stay on its branch). That is the intended behaviour, but it means a very fast drag lags the cursor around tangent branches.
* **Exact rank only up to 350 unknowns per component.** Larger components use a block-wise lower bound and the report says `exact: false`.
* **Trim / extend / offset** support lines, arcs and circles (offset: chains of lines and arcs). Splines and ellipses are not trimmed or offset. Tangency is line/circle/arc only; splines connect through G0–G2 joints.
* **DXF**: ASCII R2000 only, no binary DXF, no hatches/text/blocks; the header writes `$INSUNITS = 4` (mm) and the importer does no unit conversion. Partial ellipses and rational / non-uniform-knot splines import as fit splines through sampled points (so are approximate); full ellipses and clamped-uniform non-rational B-splines round-trip exactly. DXF carries geometry only — constraints and dimensions are not exported.
* **UI verification**: all editing logic is tested without a GUI (`Editor` and ops, 13 scripted scenarios). The egui layer is tested by feeding synthetic pointer/keyboard events through `egui::Context::run_ui` (draw a chain, rectangle + undo/redo by keyboard, drag a corner, dimension edit, DXF import/export, exit hook), and a real-window smoke run (`--demo --exit-after-frames 300`, wgpu/Vulkan) exits 0. **Nobody has looked at the rendered window yet** — layout, colours and feel are for you to judge. Dimension labels are drawn at the centroid of the dimensioned entities (no leader lines yet, not draggable).
* Nothing here has been run on Windows or macOS.
