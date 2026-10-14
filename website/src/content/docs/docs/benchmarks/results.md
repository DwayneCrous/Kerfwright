---
title: Benchmark results
description: "Every measured number next to its target and its caveat, organised by area, with how each was measured."
sidebar:
  order: 1
---

The PRD sets performance **targets**. This page lists what has been **measured** and what each measurement does not cover. The raw tables and reproduction commands are in the phase reports, linked at the end.

## How to read this page

- All numbers come from one development machine: a laptop-class CPU (16 threads in Phases 1 and 2, 8 threads in Phases 4 and 5) with an **AMD Radeon 860M integrated GPU** (RADV, Vulkan, Mesa), on Linux.
- The PRD's "reference machine" (8-core / 16-thread CPU, mid-range GPU, 1080p) and "reference GPU" were never pinned down, so a result *meets* a target only on this machine.
- Many verification suites, checkers and benchmark sets were defined and written inside this project, because the roadmap named ones that did not exist. Each such case says so.
- **Met** means the number satisfied the criterion as written. It does not mean the criterion is the one you care about.

## Viewport and rendering

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Viewport, 1,003,424 triangles + edges, 1080p, no SSAO or OIT | ≥ 60 FPS | ≈ 197 FPS windowed; mean 0.97 ms/frame headless | Phase 1. Windowed is compositor-limited and includes a blocking one-pixel pick readback |
| Same scene with OIT, GTAO and edges (30 % transparent) | ≥ 60 FPS | **p95 2.59 ms** at 1080p, 1.50 ms at 720p over 600 orbit frames, from GPU timestamps | One integrated GPU |
| ID-buffer pick | 100 % correct | 0 mismatches on 19,673 unambiguous face samples and 171 edge samples | "Unambiguous" excludes silhouette boundaries, 327 ambiguous face samples were excluded |
| Path tracer vs 16,384 spp f64 reference | ≤ 1 % MSE | 0.79 % at 16 spp, 0.18 % at 64, 0.003 % at 4,096 | Small test scene (1,302 triangles, 96 × 96) |
| Denoiser | ≥ 30 dB PSNR | 30.53 dB at 64 spp (28.07 dB noisy) | Needs 64 spp on this scene |

## Sketch solver

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Drag, 493 entities | p99 ≤ 1 ms | **p99 0.22 ms**, p50 0.022 ms | Decomposable sketches. A fully chained 493-entity sketch is p99 8.6 ms |
| Cold solve, 493 / 4,998 entities | ≤ 25 ms / ≤ 250 ms | 0.44 ms / 1.5 ms | Same caveat |
| Jacobians vs finite-difference and AD oracles | ≤ 1e-7 | < 1e-7 on 10⁶ random cases | |
| Classification (under, fully, over, conflicting) | ≥ 300 cases | 344 cases, 100 % correct. Conflict set exactly minimal in 64 of 64 | Cases are generated around a hidden solution |
| Branch flips during drag | 0 on 100 sketches | 0 flips in 18,000 frames | 11,181 of 18,000 frames were shortened or refused to achieve this |
| Determinism | bit-identical on 1 to 32 threads | Met | |
| DXF round trip | ≤ 1e-6 | Met, 20,000 cases | ASCII R2000 only |

## Kernel

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Boolean fuzz, 100,000 pairs × 3 operations | 0 invalid, ≥ 99 % of valid-expected succeed | 0 invalid, 0 panics, **99.90 %** (270,116 of 270,393) | One seed (34267): the independent point oracle and a free-form result disagree, cause not found |
| Free-form fuzz success | | 99.61 % | |
| Tangent-contact category | | 88.8 % succeed (22 % before); the rest return `Err` | Not a valid-expected category |
| Wrong results found by independent point check | 0 | 0 | |
| Boolean, 10k × 10k-face prismatic | ≤ 500 ms | union 0.48 s, intersect 0.34 s, subtract 0.37 s | Union is closest to the budget (0.50–0.55 s when busier) |
| Boolean, outlines crossing everywhere (stress) | not a target | 0.45 s at 1,600 faces, ~2.0 s at 3,200, ~3.5 s at 4,000; `Err` from ~5,000 faces | A degenerate stress case |
| Fillet, 50 edges on a 2,002-face body | ≤ 300 ms | ~170 ms | |
| Fillet benchmark | ≥ 95 % | 500 of 500 | Tangent chains and vertex blends included |
| STEP import, 50 MB | ≤ 5 s | 28.3 MB in 1.39 s, ≈ 2.5 s scaled to 50 MB | Scaled, not measured at 50 MB |
| Public STEP corpus | ≥ 95 % valid | 259 of 259 | |
| NURBS point, 32 control points, p = 3, rational | none | 31 ns | Micro-benchmark |
| `validate()` box / sphere | none | 5.2 µs / 1.5 µs | |

## Parametric engine

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Naming corpus | ≥ 500 scenarios, 0 silent wrong, ≥ 97 % automatic | 1,716 scenarios: 0 wrong, 99.88 % | Generated corpus. Thresholds tuned on the generator |
| Cold replay, 200 features (8 parts) | ≤ 2 s | 335 ms (8 threads), 1,327 ms (1 thread) | |
| Cold replay, one chain of 100 Booleans | ≤ 2 s | **8.7 s. Not met** | One body cannot use parallel waves |
| Tail edit, 200 features | ≤ 150 ms | 53 ms (8 parts). **192–202 ms on one chain. Not met** | |
| No-op edit | 0 re-evaluations | 0 executed, 0.6 ms | |
| Cycle or forward-reference move refused | 100 % | 1,272 of 1,272 | Judged against an independent dependency oracle |
| Sheet metal unfold and refold | ≤ 1e-6 | 1.4e-14 | Strips only |
| SubD deviation | ≤ 0.01 mm | worst 8.9e-3 mm | G1 at extraordinary vertices is 0.41°–1.38°, not exact |

## CAM

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Adaptive clearing engagement | ≤ 1.05× target | worst **1.010×** over 50 pockets | Excludes entry, links and finish (13 % of length). Planner and verifier share an author |
| Gouges and holder collisions | 0 | 0 on 21 parts and ≈ 1 M poses | Machine housing not modelled |
| Stock volume vs analytic | ≤ 0.5 % | +0.002 %, +0.042 %, +0.012 %, −0.128 %, −0.051 % | Five cases |
| G-code replay in an independent simulator | all five dialects | Met | Simulators written in this project. Nothing run on a real controller |
| FK(IK(pose)) | ≤ 1e-9 | 2.6e-12 on 5 machines | |
| Singularity axis step | ≤ cap | 4.83° against a 5° cap | |

## FEA

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Cantilever, plate with hole, pressure vessel | within 2 % | −0.015 %, −0.085 %, 0.93 %–1.82 % | Closed-form references |
| Mesh quality | ≥ 99 % radius-edge ≤ 2, none inverted | 99.32 %–100 %, 0 inverted | Steep size grading is weak |
| 100 k-DOF static | ≤ 10 s | 3.6 s (107,811 DOF) | Idle machine. Once 28.6 s when two perf tests ran together |
| 1 M-DOF PCG to 1e-8 | ≤ 120 s | 81 s (1,075,275 DOF, 3,301 iterations) | Structured mesh |
| SIMP MBB compliance | within 5 % | 216.8137 against 216.81 | Reference quoted by a third party, not printed in the paper |

## ECAD

| Check | Target | Measured | Caveat |
|---|---|---|---|
| Router completion, 20 boards | ≥ 98 %, 0 DRC | 1,023 of 1,023, 0 violations | Generated boards |
| Gerber in two viewers | agree | Areas within 1 % (0.00 % on the pour) | Both viewers written here |
| STEP board placement | correct | worst 8.4e-8 mm on 296 bodies | IDF bottom-side convention unverified |

## Reproduce

```sh
scripts/phase1_acceptance.sh        # kernel foundations, viewport
scripts/phase2_acceptance.sh        # sketch solver (~3 min)
scripts/phase3_acceptance.sh        # kernel, Booleans, fillets (~2.5 h with both fuzzes)
scripts/phase4_acceptance.sh        # parametric engine (~10 min)
scripts/phase5_acceptance.sh        # CAM, FEA, ECAD, rendering (~25 min, needs a GPU)
```

## Sources

[Phase 1](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase1-report.md) · [Phase 2](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase2-report.md) · [Phase 3](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase3-report.md) · [Phase 4](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase4-report.md) · [Phase 5](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase5-report.md) · [Surfaces](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/surface-report.md) · [Drawings](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/drawing-report.md)
