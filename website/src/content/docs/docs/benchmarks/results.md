---
title: Benchmark results
description: Measured numbers versus PRD targets, with the caveats attached.
sidebar:
  order: 1
---

The PRD sets performance **targets** (reference machine: 8-core / 16-thread CPU, mid-range GPU, 1080p). This page lists what has been *measured* so far and what those measurements do not cover. Raw tables are in the phase reports.

| Area | PRD target | Measured | Caveat |
|---|---|---|---|
| Viewport | ≥ 60 FPS, ≥ 1 M triangles, with edges, SSAO, OIT | 1,003,424 triangles + edges at 1080p: ≈ 197 FPS windowed, mean 0.97 ms/frame headless (integrated Radeon 860M) | Phase 1 had no SSAO/OIT; reference GPU undefined |
| Sketch drag | ≤ 1 ms, 500 entities | p99 **0.22 ms**, p50 0.022 ms (493 entities) | Decomposable sketches. A fully chained 493-entity sketch has p99 8.6 ms |
| Sketch cold solve | ≤ 25 ms (500), ≤ 250 ms (5,000) | 0.44 ms (493), 1.5 ms (4,998) | Same caveat |
| Boolean robustness | no invalid results in fuzz corpus | 300,000 operations: 0 panics, 0 invalid outputs; 99.61 % succeed, the rest return `Err` | One seed disagrees with the point-classification oracle |
| Determinism | bit-identical on any thread count | Met for sketch solve and drag on 1–32 threads | |

Sources: [Phase 1](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase1-report.md), [Phase 2](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase2-report.md), [Phase 3](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase3-report.md).
