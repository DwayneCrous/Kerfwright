---
title: Status and limits
description: "What each phase delivered, which criteria are only partly met, and every known limit by area."
sidebar:
  order: 1
---

Kerfwright is **pre-release**. This page collects, in one place, what is finished, what is partly finished and what is not done. Every item links back to the report that states it.

## Roadmap

| Phase | Theme | State |
|---|---|---|
| 1 | Math foundation, kernel skeleton, viewport | Implemented. [Report](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase1-report.md) |
| 2 | Sketcher and constraint solver | Implemented. [Report](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase2-report.md) |
| 3 | B-rep modelling kernel: intersection, Booleans, fillets, STEP | Implemented. [Report](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase3-report.md) |
| 4 | Parametric engine, assemblies, sheet metal, SubD | Implemented, two criteria only partly met. [Report](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase4-report.md) |
| 5 | CAM, FEA, ECAD, advanced rendering | Implemented. [Report](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase5-report.md) |
| 6 | Collaboration, WASM client, hardening | **Not started** |

After Phase 5 the application shell, drawings and surfaces were added. See the [app shell](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/app-shell.md), [drawings](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/drawing-report.md) and [surfaces](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/surface-report.md) reports.

## Criteria only partly met

| Criterion | What is true |
|---|---|
| Cold replay ≤ 2 s on a 200-feature model | Met on the 8-part model (335 ms). **Not met** on one linear chain of 100 Booleans (8.7 s) |
| Tail edit ≤ 150 ms | Met on the 8-part model (53 ms). **Not met** on one chain (192–202 ms) |
| SubD G1 | Exact only where the cage is regular. Around extraordinary vertices the worst tangent mismatch is 0.41° to 1.38° |
| Sketch drag p99 ≤ 1 ms | Met on decomposable sketches. A fully chained sketch is 8.6 ms |
| NURBS vs published Piegl–Tiller examples | Only Example 2.3. Other oracles are analytic |
| Exports open in third-party viewers | Parsed by three independent Rust parsers. Never opened in Blender or MeshLab |

## By area

### Kernel
- Vertex blends cover three-face corners with equal radii and convexity on planar faces. Other corners meet in a mitre.
- Variable radius fillets cover straight and circular-rim edges only. Setback is not done.
- One free-form Boolean fuzz seed (34267) disagrees with the independent point-classification check. The cause is not found.

### Sketching
- G3 joints are not done. Tangency is line, circle and arc only, and trim, extend and offset do not handle splines or ellipses.
- Ellipse and spline crossings that only graze can be missed when splitting a sketch into regions.
- Sketching is on a flat 2-D canvas, not in the 3-D view.

### Parametric and assemblies
- Patterns and mirrors repeat the tools of extrude, revolve and hole features, not patterns of patterns or fillets.
- Sheet metal is the strip model. SubD is closed cages only.
- Loop mechanisms are solved numerically by a finite-difference Newton iteration. It suits linkages of a few links. There are no gears, belts, cams or contacts, and the branch can flip near a singular position.
- No cache entries or thumbnails are written to the `.kwf`.

### CAM
- Adaptive clearing's patent freedom-to-operate review is **not done**, and it must not ship before it is.
- Posts and interpreters come from public manuals. Nothing ran on a real controller.
- The machine housing is not modelled. Turning and additive are not built.

### FEA
- Bonded contact only. No buckling. Results are not saved in the file.
- The mesher works on the faceted tessellation, and mesh grading on thick curved parts is weak.

### ECAD
- Generated sample boards only. No thermal-relief pours. IDF bottom-side convention is unverified.

### Application
- The feature document is evaluated on the UI thread.
- Real mouse interaction and native file dialogs were not driven by automated tests. They need a hand test.
- Some `kw-app` shell tests fail on a clean checkout (an egui `TexturesDelta` panic).
- Development and testing were on Linux. Nothing has run on Windows or macOS.
- Still missing: a Mesh tab, split face, patterns along a path, per-body textures, 3MF import, a scripting window, autosave and version history, contact solving in assemblies, and sheet-metal flanges added to a body's edges.

## Who checked the checkers

Several verification tools were written by the author of the code they verify: the five G-code interpreters, the two Gerber viewers, the DRC, the dexel and gouge checkers, the point-in-solid sampler and the CPU path tracer. They are separate implementations and each catches planted faults, but none is a third-party tool such as LinuxCNC, CAMotics, KiCad, gerbv or Blender. Treat results that depend on them accordingly.
