---
title: Electronics
description: "Board placement, routing, design-rule checking, copper pour and fabrication output."
sidebar:
  order: 4
---

Electronics (ECAD) places parts on a board, routes the nets, checks the design rules and writes the files a board house needs. The board is also available as STEP, so the mechanical design can use it. Start it with `--workspace ecad`.

:::caution[Sample boards only]
The workspace currently offers **generated sample boards only**. It exercises the placement, routing, checking and output engines; it is not yet a tool for designing your own board.
:::

## The flow

1. **Sample board**: generates a board (parts, nets, layers).
2. **Place**: simulated-annealing placement.
3. **Route All**: an A* grid router with rip-up and retry.
4. **Check**: design-rule checks, listed in the Violations panel.
5. **Pour**: copper pour on a layer.
6. **Fabrication files**: Gerber, Excellon and the rest.

The left panel shows **Layers**, **Violations** and **Nets**.

## Placement and routing

The board model holds footprints, pads, nets and layers (two or four in the benchmark). Placement uses simulated annealing. The router is A* on a grid with rip-up and retry: when a net cannot be routed, it removes blocking routes and tries again with a different order. It does not use negotiated congestion, which was not needed: it routed 1,023 of 1,023 nets on the benchmark.

## Design rules

Design-rule checking uses an R-tree broad phase and exact stadium and polygon clearance tests. It is written separately from the router, so a routing mistake is caught by a different piece of code. The benchmark boards route with 0 violations, and the connectivity recomputed from the finished geometry agrees on all 20 boards.

## Copper pour

The pour connects directly to same-net copper and is added after the net is routed, using polygon Booleans. Thermal-relief spokes are **not** built.

## Outputs

| Output | Format |
|---|---|
| Copper, mask and board profile | Gerber RS-274X with X2 attributes |
| Drills | Excellon, with a hole count equal to pads plus vias |
| Board for mechanical design | STEP, with components and pin-1 bodies |
| Board exchange | IDF 3.0 |
| Assembly | Pick-and-place CSV |

## How the outputs are checked

Two Gerber viewers of different architecture (one polygon-based, one raster) read the same files, and their areas agree within 1 % (0.00 % on the pour). Copper matches the board model with IoU above 0.995. Eight kinds of malformed file are rejected by both. The STEP board was re-imported into the kernel with 296 component and pin-1 bodies on three boards with bottom-side parts, with a worst placement error of 8.4e-8 mm.

## Limits

- **The IDF 3.0 convention for bottom-side parts is unverified.** The implementation mirrors the outline about y and then rotates by the written angle as seen from the top. That comes from memory, a web search could not confirm it, and ECAD tools disagree on it.
- The Gerber viewers and the DRC were written by the author of the code they check.
- Benchmark boards are generated: 28 to 91 parts (0603, SOIC8, QFP32, headers), 22 to 88 nets.
- Not built: negotiated-congestion routing, thermal-relief pours, hardware-validated fab output.
