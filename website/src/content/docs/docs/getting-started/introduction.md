---
title: Introduction
description: "What Kerfwright is, how a project is organised, what works today and where to read next."
sidebar:
  order: 1
---

Kerfwright is a CAD, CAM, FEA and ECAD suite written in Rust. It runs as one desktop program, `kerfwright`, built on its own B-rep modelling kernel. The kernel and the applications on top of it were written independently, from papers, standards and published documentation (see [Provenance and IP policy](/docs/project/clean-room/)).

The name is the job: a *kerf* is the width of material a cut removes, and a *wright* makes things. The program follows a part from the first sketch to the program that cuts it.

## The idea: one document

A project is a single `.kwf` file. At its centre is the **feature document**: sketches with dimensions and constraints, named parameters, and an ordered timeline of features such as extrude, fillet and hole. Everything else hangs off that history and refers back to it:

- **CAM** setups, stock and operations are stored as `cam.cbor`.
- **FEA** studies, supports and loads are stored as `fea.cbor`.
- **ECAD** boards are stored as `board.cbor`.
- **Assemblies**, their joints and rigid groups are stored as `assembly.cbor`.

Because they share one history, changing a dimension rebuilds the features that depend on it. Things built on the model notice the change too: CAM operations show as *out of date* until you regenerate them, and drawing sheets show a banner until you press **Update**. See [The .kwf project file](/docs/architecture/kwf-format/) for the container itself.

## What is in the box

| Area | What you can do | Main crates |
|---|---|---|
| Kernel | Exact quadrics and NURBS, surface intersection, Booleans, fillets and chamfers, shell, offset, draft, mass properties, STEP and IGES | `kw-math`, `kw-nurbs`, `kw-brep`, `kw-ssi`, `kw-boolean`, `kw-blend`, `kw-model` |
| Sketching | Constrained 2-D sketches with 13 constraint types, a live degrees-of-freedom report, DXF in and out | `kw-sketch`, `kw-dxf`, `kw-sketcher` |
| Parametric engine | Expressions with units, an incremental timeline, persistent names, assemblies, sheet metal, subdivision forms, scripting | `kw-expr`, `kw-feature`, `kw-assembly`, `kw-sheet`, `kw-subd`, `kw-script` |
| Manufacture, simulate, electronics | Toolpaths and G-code, tetrahedral meshing and FEA, board placement and routing | `kw-cam`, `kw-gsim`, `kw-mesh3d`, `kw-fem`, `kw-ecad`, `kw-fabcheck` |
| Output | Tessellation, the GPU viewer, path-traced rendering, 2-D drawings, mesh and exchange formats | `kw-tess`, `kw-viewer`, `kw-render`, `kw-drawing`, `kw-io`, `kw-step`, `kw-iges` |

## Status

Phases 1 to 5 of the [roadmap](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/04-roadmap.md) are implemented. Phase 6 (collaboration, a WASM client and hardening) has not started. Each phase report lists the acceptance criteria that are only partly met, and [Status and limits](/docs/project/status-and-limits/) collects them in one place.

:::caution[Pre-release]
There are no installers, and development and testing so far were on Linux. The Design workspace exposes a large part of the engine but not all of it. Treat files from this version as disposable until the format is declared stable.
:::

## Where to go next

- **Run it:** [Build from source](/docs/getting-started/build-from-source/), then [Make your first part](/docs/getting-started/first-part/).
- **Understand it:** the [Concepts](/docs/concepts/parametric-engine/) pages explain how the timeline rebuilds, how faces keep their names, and how the sketch solver works.
- **Use a workspace:** start with [Design](/docs/workspaces/design/), then [Manufacture](/docs/workspaces/manufacture/).
- **Check a claim:** [Benchmark results](/docs/benchmarks/results/) lists every measured number next to its target and caveat.
