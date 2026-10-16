# Kerfwright — Product Requirements Document (PRD)

Derived from `docs/01-research/`. Priorities: **P0** = required for the workspace to be usable, **P1** = expected by professional users, **P2** = differentiator / later.

## 1. Product statement
Kerfwright is a pure-Rust, cross-platform (Linux, macOS, Windows, WebGPU/WASM) integrated CAD/CAM/CAE/ECAD application for mechanical product design, built on its own B-rep kernel, parametric history engine, and GPU renderer. It is an independent implementation from public sources (see `CLEAN_ROOM.md`); it targets *workflow familiarity* with modern all-in-one tools, not file- or UI-level cloning.

**Target users:** hobbyists/makers, small manufacturers, and product designers who want one tool from sketch to G-code, with an open, scriptable, auditable core.

**Non-goals (v1):** full Class-A surfacing, large-assembly PDM, explicit FEA (nonlinear static FEA — large displacement and plasticity at moderate strain — is built, see `docs/reference-workflow-comparison.md`), full-featured EDA schematic capture, cloud CAM post libraries for every machine, parity with any vendor's feature count.

## 2. Workspaces

### 2.1 Sketcher
| Feature | Pri |
|---|---|
| Line, arc (3-pt/centre/tangent), circle, ellipse, rectangle, polygon, slot, point, construction geometry | P0 |
| Fit-point & control-point splines (NURBS, degree 2–5), conic | P0 |
| Constraints: coincident, horizontal/vertical, parallel, perpendicular, tangent, concentric, collinear, equal, fix, midpoint, symmetric | P0 |
| Dimensions: distance, angle, radius/diameter, with expressions & named parameters | P0 |
| Curvature continuity G0–G2 on splines; G3 | P1 / P2 |
| Live DOF display; conflict/redundancy diagnostics with minimal conflicting set | P0 |
| Trim, extend, offset, fillet, mirror, pattern (linear/circular), project/intersect edges | P0 |
| Import DXF (2D) as sketch geometry | P1 |
| Drag with constraint-aware solving | P0 |

### 2.2 Parametric B-Rep modelling
| Feature | Pri |
|---|---|
| Extrude (one/two-sided, symmetric, to-object, taper), Revolve | P0 |
| Boolean: join, cut, intersect, new body | P0 |
| Hole (simple/counterbore/countersink/threaded-cosmetic), Shell, Draft | P0 |
| Fillet (constant radius), Chamfer (equal / two-distance / distance-angle) | P0 |
| Variable-radius fillet, setbacks, G2 blends, variable chamfer | P1 / P2 |
| Sweep, Loft (with guide rails), Coil | P1 |
| Patterns (rectangular, circular, along path), Mirror | P0 |
| Reference geometry: planes, axes, points | P0 |
| Timeline: rollback, reorder (with cycle validation), edit-in-place, suppress | P0 |
| Persistent references (topological naming) with repair UI | P0 |
| Parameters & expressions with units | P0 |
| Measure, section analysis, mass properties | P0 |
| Assemblies: components, rigid/revolute/slider joints, interference check | P1 |
| Direct-edit (push-pull face, move face) | P2 |

### 2.3 Form (SubD, T-splines)
| Feature | Pri |
|---|---|
| Catmull–Clark SubD primitives, extrude/insert-edge/bridge/crease, symmetry | P1 |
| Finish → B-rep with deviation report (exact regular patches + G1 near extraordinary vertices) | P1 |
| True T-spline local refinement (⚠ gated by patent review) | P2 |

### 2.4 Sheet Metal
| Feature | Pri |
|---|---|
| Sheet-metal rules (thickness, bend radius, K-factor, bend tables) | P0 |
| Flange (edge), contour flange, hem, corner relief, rip | P0 |
| Bend allowance `BA = θ(R + K·T)`; unfold/refold with flat pattern | P0 |
| Flat pattern export: DXF with bend lines | P0 |
| Lofted/ forming features | P2 |

### 2.5 Simulation & Generative Design (FEA)
| Feature | Pri |
|---|---|
| Linear static stress (TET10), materials library, fixed/force/pressure/gravity loads | P0 |
| Modal analysis; thermal steady-state | P1 |
| Mesh controls (global size, local refinement, quality metrics) | P0 |
| Result visualisation: displacement, von Mises, safety factor, animated | P0 |
| Topology optimisation (SIMP) with preserve/obstacle regions, min-member-size | P1 |
| Result → SubD → B-rep | P2 |

### 2.6 ECAD
| Feature | Pri |
|---|---|
| Netlist import (KiCad-style netlist/IPC-D-356), footprint library | P0 |
| Board outline, stackup, rules (clearance, width, via) | P0 |
| Interactive placement, force-directed auto-placement | P1 |
| Interactive routing (push-and-shove), auto-router (A\*/Lee, rip-up) | P1 |
| DRC, copper pour, differential pairs, length tuning | P1 |
| 3D board with component models; STEP/IDF export to MCAD | P0 |
| Gerber RS-274X / Excellon / pick-and-place export | P0 |

### 2.7 CAM (2.5D → 5-axis)
| Feature | Pri |
|---|---|
| Setup: stock, WCS, machine & tool library, feeds/speeds | P0 |
| 2D: face, contour, pocket, adaptive clearing, drill/bore, chamfer, engrave | P0 |
| 3D: parallel, scallop, pencil, rest machining, ramp/helix entries | P1 |
| 4/5-axis: positioning (3+2), swarf, multi-axis contour, tool-axis collision avoidance | P1 / P2 |
| Machine kinematics model; RTCP; inverse-time feed | P1 |
| Stock simulation (tri-dexel) with gouge & collision check | P1 |
| Post-processor DSL; Fanuc, Haas, Siemens, Heidenhain, LinuxCNC/GRBL | P0 / P1 |
| Toolpath verification & setup sheets | P1 |

### 2.8 Rendering
| Feature | Pri |
|---|---|
| Realtime viewport: PBR shading, edge overlay, ambient occlusion, section views, OIT transparency | P0 |
| Selection/hover via ID buffer; orbit/pan/zoom with large-model precision | P0 |
| Appearance library, decals, environment lighting | P1 |
| Progressive GPU path-traced renders with denoising | P2 |
| Turntable/exploded-view animation export | P2 |

## 3. Non-functional requirements & Performance SLAs
SLAs are **targets** to be proven by the benchmark suite (`benches/`, tracked in CI); the reference machine is a 8-core/16-thread 3.5 GHz desktop CPU, 32 GB RAM, mid-range GPU (≈ GTX 1660 / Apple M1 class), 1080p.

| Area | SLA |
|---|---|
| Viewport | ≥ 60 FPS (p95 frame ≤ 16.7 ms) with ≥ 1 M displayed triangles (100 k+ polys baseline; 10 M LOD-managed) incl. edges + SSAO + OIT |
| Input latency | Orbit/pan input-to-photon ≤ 50 ms |
| Sketch solve (drag / incremental) | ≤ 1 ms for a 500-entity sketch with warm start and cluster decomposition |
| Sketch solve (cold) | ≤ 25 ms for 500 entities; ≤ 250 ms for 5 000 |
| Feature rebuild (tail edit) | ≤ 100 ms to re-evaluate and tessellate a single extrude/cut on a 1 000-face body |
| Full timeline replay | ≤ 2 s for 200 features (cold cache), ≤ 150 ms when the edit is in the last 5 features |
| Boolean | ≤ 500 ms for 10 k-face × 10 k-face prismatic bodies; zero invalid results in the fuzz corpus (hard requirement) |
| Fillet | ≤ 300 ms for 50 constant-radius edges on a 2 k-face body |
| STEP import | 50 MB file ≤ 5 s to a validated B-rep; healing report generated |
| FEA | 100 k-DOF linear static ≤ 10 s (direct), 1 M-DOF ≤ 120 s (PCG+AMG) |
| CAM | 2D adaptive on a 200×200 mm pocket with 50 islands ≤ 2 s; 3 M-triangle drop-cutter surface pass ≤ 30 s |
| Memory | ≤ 2 GB resident for a 5 000-face, 300-feature part with caches |
| Startup | Cold start to interactive ≤ 2 s; open 100-feature file ≤ 1 s with cache |
| Determinism | Same inputs ⇒ bit-identical B-rep and names on the same build, any thread count |
| Reliability | No kernel panics in the shipping path: kernel returns `Result`; fuzzers (cargo-fuzz) on all importers and Boolean inputs; autosave + crash recovery |

## 4. Data model & interoperability

| Format | Direction | Scope |
|---|---|---|
| **STEP** (ISO 10303-21; AP203, AP214, AP242) | Import + Export | B-rep solids/shells, assemblies, colours, names; PMI later (P2) |
| **IGES 5.3** | Import + Export | Trimmed NURBS surfaces (128/143/144), curves (126), B-rep (186) |
| **STL** (binary/ASCII) | Import + Export | Triangle meshes, units & tolerance settings |
| **OBJ** (+MTL) | Import + Export | Meshes with materials; SubD cage export |
| **DXF** (R12–2018) | Import + Export | 2D sketches, flat patterns, board outlines, layers |
| **glTF 2.0 / GLB** | Export (+Import P1) | Tessellated scene graph, PBR materials |
| **Gerber RS-274X, Excellon, IPC-2581, IDF 3.0** | Export (IDF also import) | ECAD |
| **G-code (multiple dialects)** | Export | CAM posts |
| **`.kwf`** (native) | Read/Write | ZIP container: recipe (CBOR), parameters, names, optional B-rep/tessellation caches, thumbnail; versioned with forward-compat rules |

Import pipeline: parse → build topology → sew within tolerance → validate (Euler–Poincaré, orientation) → heal → report. Importers are fuzzed and run under resource limits.

## 5. Platform & UX
* Desktop-first (egui or iced shell; decision in Phase 1), WebGPU build for read-only viewer in Phase 6.
* Scripting: Rust API + embedded scripting language (Rhai or Lua) for parametric macros in Phase 4.
* Accessibility: keyboard-navigable commands, colour-blind-safe constraint display.
* Telemetry: opt-in only.

## 6. Success criteria for v1.0
1. Model, constrain, and rebuild a 200-feature mechanical part with fillets and patterns without a wrong-reference failure in the regression corpus.
2. Round-trip STEP with ≥ 3 independent third-party CAD tools within 1e-5 model units.
3. Machine a real part on a 3-axis mill from sketch to G-code using only Kerfwright.
4. Pass all SLAs on the reference machine.
