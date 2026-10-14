# Phase 5 report — CAM, FEA, ECAD, advanced rendering

Status: **implemented and tested; every numeric acceptance criterion in `docs/04-roadmap.md` is met as measured here, with the qualifications in §3.** The qualifications matter more than the table: the roadmap names suites, simulators, viewers and a GPU that did not exist, so this phase defined them (seeded, documented, §2) and wrote the independent checkers itself. A checker written by the author of the code it checks is weaker than a third-party tool; where that applies it is stated.

Everything below is produced by `scripts/phase5_acceptance.sh` (logs in `target/phase5/`, ~25 min with the whole-workspace test run, `SKIP_WORKSPACE_TESTS=1` skips it). Measured on an 8-thread laptop APU; "reference GPU" is this machine's **AMD Radeon 860M iGPU (RADV, Vulkan, Mesa 26.0.8)**. Nothing is committed.

## 1. Results against the acceptance criteria

| # | Criterion | Result | Status |
|---|---|---|---|
| **CAM** | | | |
| 1 | Adaptive clearing ≤ 5 % over the target engagement angle, 50-pocket suite | worst **1.010 × target** over 50 seeded pockets (convex, concave, with islands, slots); worst gouge 0.3 µm; min coverage 0.935 | met (see §3: *engagement is measured outside the entry/link/finish moves, 13 % of the length*) |
| 2 | Zero gouges, zero tool/holder collisions on the verification suite | 21 seeded 3-D parts with vise-jaw fixtures, ≈ 1 M poses: **0 gouges** (worst penetration 1.0 µm), **0 shank/holder collisions**, 0 hits in the independent point sampler; planted faults (0.2 mm gouge, 3 mm short stick-out) are detected by both checkers | met |
| 3 | Stock simulation within 0.5 % of analytic removed volume | five cases (pocket, stadium slot, ball sweep, helical ramp, flat and ball drill): errors **+0.002 %, +0.042 %, +0.012 %, −0.128 %, −0.051 %/−0.018 %**; the x- and y-ray estimates agree | met |
| 4 | G-code of each dialect runs in an independent simulator | Fanuc, Haas, Siemens 840D, Heidenhain TNC, LinuxCNC: pocket, island pocket, contour, drilling (plain and pecked), the 10 578-event adaptive path and 50 five-axis programs replay without errors and reproduce the cutter-location data (≤ 1.1 µm, chorded arcs match in length) | met (the simulators are ours, §3) |
| 5 | FK(IK(pose)) = pose within 1e-9 on all test machines | 5 machines × 2000 random poses, both branches in every pose: worst **2.6e-12** | met |
| 6 | Singularity handling: no axis jump above the limit | 400-pose sweep past the pole at 0.05° offset: largest rotary step **4.83°** against a 5° cap on all five machines; tool tip stays on the commanded line to 1.8e-14 mm; intermediate orientation error ≤ 0.004° there, **0.196°** worst on a harsher wavy path | met |
| **FEA** | | | |
| 7 | Cantilever, plate with hole, pressure vessel within 2 % with TET10 | Timoshenko–Goodier cantilever tip **−0.015 %** (−0.001 % finer); Kirsch peak hoop stress **−0.085 %**; Lamé cylinder radial displacement 0.018 %, hoop stress 0.93 % (1.31 % at edge nodes); Lamé sphere 0.045 % / 1.26 % (1.82 % at edge nodes); bending frequencies 1–3 within 0.3 % of Euler–Bernoulli with shear and rotary inertia | met (exact elasticity solutions rather than NAFEMS tables, §3) |
| 8 | ≥ 99 % of elements radius-edge ≤ 2, none inverted | box, cylinder, sphere, plate with hole, shell octant at several sizes: 99.85–100 % (benchmark meshes 99.32–100 %), **0 inverted** (TET4 and TET10) everywhere; dihedral angles 15°–155° | met; weak spots in §3 |
| 9 | 100 k-DOF static ≤ 10 s | 107 811 DOF, assembly + BCs + factor + solve **3.6 s** | met |
| 10 | PCG residual 1e-8 on 1 M DOF ≤ 120 s | 1 075 275 DOF, Jacobi-PCG, 3 301 iterations, residual **9.0e-9**, **81 s** | met (structured mesh, §3) |
| 11 | SIMP MBB within 5 % of published compliance | 60×20, V = 0.5, p = 3, r = 2.4, sensitivity filter: **216.8137** after 106 iterations against 216.81 (+0.002 %) | met, but see §3: *the reference value is quoted by a third party, not printed in the paper* |
| **ECAD** | | | |
| 12 | A* router ≥ 98 % of nets on the 20-board benchmark, 0 DRC violations | **1 023 / 1 023 nets, every board 100 %, 0 violations**; connectivity recomputed from the finished geometry agrees on all 20 boards | met |
| 13 | Gerber validates in two independent viewers | copper (all layers), mask, profile of 6 routed boards and the pour layer read by two viewers of different architecture (polygon / raster): areas agree < 1 % (0.00 % on the pour), copper IoU > 0.995 against the board model; 8 classes of malformed file rejected by both; Excellon hole count = pads + vias | met (the viewers are ours, §3) |
| 14 | IDF/STEP board opens in the MCAD workspace with correct placement | STEP re-imported by `kw-step` into the kernel: 296 component and pin-1 bodies on three boards with bottom-side parts, **worst placement error 8.4e-8 mm**; IDF 3.0 board/library files and pick-and-place CSV read back to the same placement | met for STEP; IDF convention unverified (§3) |
| **Rendering** | | | |
| 15 | 60 FPS at 1 M triangles with OIT + GTAO + edges | 1 003 424 triangles (≈ 30 % transparent), 600 orbit frames, GPU timestamps: **1080p p95 2.59 ms** (max 2.68), **720p p95 1.50 ms** — 6× under the 16.7 ms budget | met |
| 16 | Path tracer ≤ 1 % MSE vs reference after N spp; denoiser PSNR ≥ 30 dB | relative MSE vs a 16 384-spp f64 CPU reference: **0.79 % at N = 16 spp**, 0.18 % at 64, 0.003 % at 4096; denoised PSNR (tonemapped, peak 1) **30.53 dB at 64 spp** (noisy 28.07 dB), 31.52 dB at 128 | met |
| — | Furnace / disc-light / BRDF checks | Lambert furnace 1.00000 (CPU and GPU); GGX directional albedo equals quadrature to 0.03 % and never exceeds 1; disc-light radiance 0.09 % (CPU) / 0.04 % (GPU) from the closed form; WGSL BRDF and pdf equal the f64 code to 6.3e-5 over 4096 random cases | — |

Also verified: `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`, the whole workspace test suite.

## 2. What was built, and how the undefined suites are defined

| Crate | Contents |
|---|---|
| `kw-job` | `CancelFlag`, `Progress`, `par_pool` (threads ≤ min(8, cores − 2)); leaf crate so the mesher and FEM do not depend on the feature engine. |
| `kw-mesh3d` | Constrained Bowyer–Watson Delaunay with exact predicates (`robust`), surface remeshing, feature-aware Rivara refinement, two-stage radius-edge refinement (3 then 2), edge removal/flips and smoothing, end-of-run snapping to the true surface, TET10 promotion with Jacobian guard, structured box tets. |
| `kw-fem` | TET10 assembly (coloured, parallel), direct LLT (faer), Jacobi/block-Jacobi PCG, boundary conditions, tractions as functions, L2 stress recovery, shift-invert modal, SIMP (transcription of top88), closed-form references with self-checks. |
| `kw-cam` | `geom2d` (cavalier_contours), contour/pocket/drill, `adaptive` + independent `verify` + `suite` (50 pockets), `bvh`, `dropcutter` (flat/ball/bull, exact), `finish3d` + `suite3d`, `dexel` (tri-dexel), `kin` (5 machines, analytic IK, joint-step cap, RTCP re-solve), `multiaxis` (3+2, swarf), `post` (template DSL, 5 posts, 5-axis events). |
| `kw-gsim` | Interpreters for Fanuc, Haas, Siemens 840D, Heidenhain TNC (a different grammar), LinuxCNC. They reject unknown words/codes, track modal state, understand rotary words and TCP modes; no dependency on `kw-cam`. |
| `kw-ecad` | Board model, footprints, annealing placement, grid router, DRC (R-tree broad phase, exact stadium/polygon clearance, separate from the router), connectivity check, copper pour (i_overlay), Gerber RS-274X + X2, Excellon, IDF 3.0, STEP, pick-and-place, benchmark generator. |
| `kw-fabcheck` | Gerber viewer A (polygons, dark/clear by boolean), viewer B (raster), Excellon parser; both reject malformed files; no dependency on `kw-ecad`. |
| `kw-render` | WBOIT, GTAO, edges, timestamp bench (`frame`); path tracer: CPU LBVH (Morton + Karras, rayon), WGSL traversal/shading/accumulation, à-trous denoiser, f64 CPU reference with its own BVH and BRDF, metrics. |
| `kw-store` | `cam.cbor`, `fea.cbor`, `board.cbor` as extra `.kwf` entries (like `assembly.cbor`); round-trips through save/load. |

**Definitions made in this phase** (the roadmap leaves them open):
* *50-pocket suite*: `kw-cam::suite::pocket_suite(seed)` — 12 convex (rectangles, hexagons/heptagons, rounded rectangles), concave (L, T, stars), pockets with islands and slots, tools 6–10 mm, radial engagement 0.9–2.3 mm, seeded. *Engagement* is measured by an independent analytic verifier (exact swept-stadium distances, bisection on the engagement predicate, arc spacing 0.05 R); *adaptive cut moves* exclude entry helices, the seed cut, links and the finish contour, and the excluded fraction is reported (mean 13.1 %).
* *Verification suite (3-D)*: `kw-cam::suite3d` — 21 closed height-field parts (terrain, domes and cones, terraces; 6 720 triangles each) with a vise jaw on each side at random heights; ball-nose Ø6 raster finishing with chord-sag refinement to 2 µm; stick-out chosen as the least that clears part and jaws plus 0.5 mm. Gouge = tip below the drop-cutter height by > 5 µm.
* *20-board benchmark*: `kw-ecad::bench_gen::benchmark_specs()` — seeds 1000–1019, 28–91 parts (0603, SOIC8, QFP32, 6-pin headers), 22–88 nets, GND/VCC plus signal nets of 2–5 pads formed among logically adjacent parts, 2 and 4 layers, clearance/track 0.2 or 0.15 mm, part area 28 % → 46 % of the board, passives on both sides on every third board. All parameters are printed with the results.
* *Reference GPU*: the Radeon 860M above. *Frame budget*: p95 GPU time (timestamp queries) over 600 orbiting frames of a fixed 1 003 424-triangle scene, ≈ 30 % transparent.

## 3. Qualifications and known gaps (read these)

1. **The checkers are ours.** The five G-code interpreters, the two Gerber viewers, the DRC, the dexel/gouge checkers, the point-in-solid sampler and the CPU path tracer were written by the author of the code they verify. They are separate implementations (no shared code, different algorithms where possible: ray-parity sampling vs drop-cutter, polygon vs raster viewers, f64 reference vs WGSL) and each catches planted faults, but none is a third-party LinuxCNC, CAMotics, KiCad/gerbv or Blender. The roadmap's "independent simulator (e.g. LinuxCNC)" is met in spirit only.
2. **Dialects are written from general conventions of public manuals.** The posts and interpreters agree with each other; nothing was run on a real controller. LinuxCNC's tool-centre-point control comes from machine kinematics, not G-code, so its post emits a comment. The IDF 3.0 bottom-side convention (outline mirrored about y, then rotated by the written angle seen from the top) is from memory; a web search could not confirm it, and different ECAD tools disagree on it (CircuitWorks has a per-tool option). Treat the IDF rotation of bottom parts as unverified.
3. **Adaptive clearing**: the planner and the verifier have the same author. The 5 % bound holds outside entry, links and the finish pass; coverage is 93.5–100 % (the rest is left for the finish pass). It follows boundaries and steers a spiral from a helical entry; the planned trochoidal corner fallback was not built. **The freedom-to-operate review is not done** — `LICENSES/patent-watch.md` records that adaptive clearing must not ship until it is. The medial-axis method of US 9,164,503 is deliberately not used, which is why `kw-voronoi` (planned for entry points only) was **cut**.
4. **Meshing**: radius-edge ≥ 99 % holds everywhere measured, but steep size grading on thick curved parts is weak (a quarter ring at h = 1.4 gives 99.85 % with 2 unfixable tets; earlier at h = 1.6 it was 96.8 %), and unfixed elements sit at trihedral corners. Stress recovery at edge nodes is 1.3–1.8 % (face and volume nodes < 1.3 %), inside the 2 % criterion but close. The references are closed-form elasticity solutions (Timoshenko–Goodier, Kirsch, Lamé), not NAFEMS tables. The 1 M-DOF PCG run uses a structured (box-split) mesh with a Jacobi preconditioner; its 81 s is for that mesh, and block-Jacobi was not faster.
5. **SIMP reference**: Andreassen et al. (2011) print no compliance number for the MBB beam. The value 216.81 is quoted by a third-party port that cites the paper's figure; the transcription of top88 reproduces it (216.8137 in 106 iterations) and the density-filter variant gives 233.71 as that port does. The original MATLAB could not be run here (no Octave). The criterion is met against that reference, not against a number in the paper.
6. **Stock simulation** handles a vertical cutter in the setup frame, and since the CAM-scale work (`docs/fusion-parity.md`) a tilted one too: each dexel ray is intersected with the cutter's body of revolution (one interval, found by sampling and bisection), checked against an oblique cylinder (3 %) and against the vertical fast path; the fixture check still assumes an upright tool. Tool and holder are checked against the part and fixtures; the **machine housing is not modelled**, so axis-limit and housing collisions are out of scope. The drop-cutter treats the tool as infinitely long (shank and holder are separate cylinder tests).
7. **Not built**: PathFinder negotiated congestion (the router is A* with rip-up-and-retry, which sufficed: 100 % on all boards), thermal-relief pours (the pour connects directly to same-net copper and is added after the net is routed), GPU LBVH construction (CPU build), hardware-validated 5-axis posts, `kw-voronoi`.
8. **Path tracer scene** is small (Cornell box, 1 302 triangles, 96×96); the 1 % relative-MSE criterion is met at 16 spp because the metric is dominated by bright regions — the PSNR figures are the more telling noise measure. The denoiser's 30 dB needs 64 spp on this scene; it is a spatial filter that divides by the first-hit albedo, so the glossy ball is filtered with the diffuse model and its reflections are smoothed rather than preserved.
9. **WBOIT** is an approximation: against an exact sorted blend of two translucent layers the worst channel difference is 24/255, as expected for the technique.
10. A 100 k-DOF direct solve took 28.6 s once, when the two performance tests ran at the same time; the tests now exclude each other and the measured 3.6 s is for an otherwise idle machine.

## 4. Reproducing

```bash
scripts/phase5_acceptance.sh                 # all of it (~25 min)
SKIP_WORKSPACE_TESTS=1 scripts/phase5_acceptance.sh
KW_PT_REF_SPP=65536 scripts/phase5_acceptance.sh   # a cleaner reference image
```

CPU steps run under a soft `ulimit -v` and ≤ 8 threads; the GPU steps run uncapped (Mesa/Vulkan reserve huge virtual ranges) and skip themselves when no adapter is found.

## 5. CAM at scale (setups, libraries, more operations, rotary axes)

Added after the criteria above were met; everything here is checked by `cargo test -p kw-cam --release` and `cargo test -p kw-app --test cam_setups --test cam_library --test cam_ops`.

| Check | Result |
|---|---|
| Material sampled from a closed mesh (box, cylinders along z and x) | aligned box exact; off-cell box within the sideways quantisation (6 %); cylinders 2–5 % at 0.5 mm cells |
| The stock's surface read back as a mesh | volume within 0.5 % at the same cell, 3 % at twice the cell |
| Two-setup flip (top 4 mm and bottom 4 mm of waste) | setup 2 starts at setup 1's final volume (1 %); both removals within 4 % of 4 000 mm³; what is left is the part (≤ 4 %); no overcut |
| Fixture beside the part | the contour around the part reports the tip in the fixture; ignoring the jaw clears it |
| Face, inside contour, chamfer (90°, 1 mm), drill (3 holes, cone tip), helical bore (2 holes ⌀12) | removed volume vs closed form: 2 %, inside the walls, 8 % (corner effect), 3 %, 3 %; no rapid through stock |
| Waterline on the 21-part suite | 0 gouges at a 0.03 mm tolerance; > 80 % of the poses touch the part within 0.03 mm |
| Rest finishing | cuts the strips the rough pass left; sideways cutting length 40 % or less of the full finish after the pocket |
| Tilted flat cutter, oblique cylinder | within 3 % of π R² d / cos θ at 0°, 30°, 45° |
| Drill and chamfer cones in the dexel | within 2 % / 3 % of the closed form |
| 3+2 on an AC trunnion machine, front face | A = ±90°, tool axis within 1e-3°, tip within 1e-3 mm; replays in all five dialects |
| Feeds and speeds | match the hand calculation (n = 1000 Vc / π D, F = n z fz), clamped by the machine |

Qualifications: nothing was run on a real machine; the machine housing is not modelled; the axis-limit report covers the travel in the machine model only; the dexel grid sets the sideways accuracy of every simulated surface; the work offsets stop at G59; the post templates for a tilted work plane (G68.2) are not used, tilted setups are written with tool-centre-point control instead.
