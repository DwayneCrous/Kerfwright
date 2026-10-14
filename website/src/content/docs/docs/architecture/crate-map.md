---
title: Crate map
description: Dependency direction and the rules that keep the kernel clean.
sidebar:
  order: 1
---

```text
kw-app        Rayon ⇄ Tokio bridge, Command/Event, cancellation
  └─ kw-feature   DAG, persistent topological names, incremental evaluator
       ├─ kw-sketch   entities, residuals, DM decomposition, sparse LM solver
       └─ kw-brep     slotmap topology arenas, ShapeHistory, validation
            └─ kw-nurbs   knot vectors, NURBS, analytic geometry
                 └─ kw-math   tolerance, frames, transforms, robust predicates
```

**Rule:** arrows point downward only. `kw-math` knows nothing about the kernel, and the UI and renderer read immutable snapshots, never mutable kernel state.

## Design decisions worth knowing

- **Coedge + radial ring B-rep** in slotmap arenas; every entity has a typed, generational id.
- **Kernel returns `Result`.** Failures in Booleans and importers are reported errors, not panics.
- **Determinism.** The same inputs give bit-identical solves on 1–32 threads; parallelism is across independent components, with no cross-thread float reductions.
- **Clean-room.** Sources, licensing and naming rules are in [`CLEAN_ROOM.md`](https://github.com/DwayneCrous/Kerfwright/blob/main/CLEAN_ROOM.md).

The full architecture document, with memory, state and thread diagrams, is [`docs/03-architecture.md`](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/03-architecture.md).
