---
title: Introduction
description: What Kerfwright is, what exists today, and where to read next.
sidebar:
  order: 1
---

Kerfwright is a pure-Rust, clean-room CAD / CAM / CAE / ECAD suite. One feature document (`.kwf`) carries the sketches, the parametric history, and the CAM, FEA and board data that hang off it.

## Status

Phases 1–5 of the [roadmap](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/04-roadmap.md) are implemented; Phase 6 (collaboration, WASM client, hardening) has not started. Each phase report lists the acceptance criteria that are only partly met. The Model workspace exposes only a subset of the engine so far.

| Area | Crates |
|---|---|
| Kernel | `kw-math`, `kw-nurbs`, `kw-brep`, `kw-ssi`, `kw-boolean`, `kw-blend`, `kw-model` |
| Sketching | `kw-sketch`, `kw-dxf`, `kw-sketcher` |
| Parametric engine | `kw-expr`, `kw-feature`, `kw-assembly`, `kw-sheet`, `kw-subd`, `kw-script` |
| Manufacture / simulate / electronics | `kw-cam`, `kw-gsim`, `kw-mesh3d`, `kw-fem`, `kw-ecad`, `kw-fabcheck` |
| Output | `kw-tess`, `kw-viewer`, `kw-render`, `kw-drawing`, `kw-io`, `kw-step`, `kw-iges` |

## Where to go next

- [Build from source](/docs/getting-started/build-from-source/)
- [Crate map and dependency rules](/docs/architecture/crate-map/)
- [Benchmark results](/docs/benchmarks/results/)
