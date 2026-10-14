---
title: Threading and jobs
description: "The single-writer document model, snapshots, the Rayon and Tokio split, priorities and cancellation."
sidebar:
  order: 3
---

Kerfwright separates *who owns the document*, *who does heavy maths* and *who draws*, so that a long Boolean cannot freeze the interface and two threads can never disagree about what the model is.

:::note[Target design and current state]
This page describes the architecture the code is built around (`docs/03-architecture.md`). In the current application the **feature document is still evaluated on the UI thread**, so a very heavy model pauses the window. CAM, FEA, routing, stock simulation and denoising already run on background threads. A background evaluation thread is listed as missing.
:::

## One writer, many readers

```text
 UI thread (winit + egui)                 Engine (owns the document)
┌───────────────────────────┐          ┌──────────────────────────────────────┐
│ input → Command           │─cmd tx──►│ command loop (single writer)          │
│                           │          │   Doc = Arc<DocState>  (immutable)     │
│ render ◄ Arc<DocSnapshot> ◄──watch──┤   apply(cmd) → new DocState → publish  │
│ (read-only, lock-free)    │          │   undo stack = Vec<Arc<DocState>>      │
└─────────────┬─────────────┘          └─────────────────┬────────────────────┘
              │ Event (progress, errors)                  │ dirty set
              ◄───────────────────────────────────────────┘
                                                          ▼
                                       evaluation scheduler (topological waves)
```

- **Only the command loop creates a new document state.** Everyone else holds an `Arc` to an immutable snapshot, so the render path takes no locks.
- A snapshot is the DAG, the parameters, an index into the evaluation cache and the tessellations. Persistent maps with structural sharing keep an edit's cost proportional to the nodes that changed.
- **Undo** is a stack of earlier snapshots. Unchanged bodies are `Arc` clones, which cost O(1).

## Rayon for compute, Tokio for waiting

Two pools do two different jobs.

| Pool | Used for | Rule |
|---|---|---|
| **Rayon**, N = cores − 2 | Surface intersection, Booleans, fillets, constraint clusters, tessellation, FEM assembly and solve, CAM toolpaths | CPU-bound work only |
| **Tokio** | File I/O, autosave, the job queue | Never runs CPU-bound work |

The rules that keep them from deadlocking or starving each other:

1. CPU-bound work **never** runs on a Tokio worker. It goes through `spawn_compute` (a Rayon task plus a one-shot channel for the result).
2. Rayon tasks **never** block on a Tokio future. There is no `block_on` inside the pool.
3. Rayon closures capture `Arc` snapshots. No shared mutable state, and no locks in kernel code.
4. Parallel reductions use fixed chunking and ordered merges ([Determinism](/docs/concepts/determinism/)).
5. Every long task polls `CancelToken::is_cancelled()` at loop heads.

## Priority lanes

| Lane | Work | Notes |
|---|---|---|
| Realtime | Sketch drag | Bypasses the DAG: an incremental cluster solve patches the snapshot directly, and the full rebuild is debounced to drag end (or 50 ms) |
| Interactive | Feature edits | Runs on the main pool |
| Background | CAM, FEA | Runs on a separate low-priority pool so it cannot starve interactive solves |

The pool for background jobs is capped at the smaller of 8 threads and cores − 2.

## Cancellation and supersession

Each interactive command takes a new `CancelToken` generation. An evaluation from an older generation returns `Cancelled` at its next checkpoint, every kernel call polls at least every millisecond or so. So dragging a slider never queues up a backlog of stale rebuilds: only the newest one finishes.

## Errors

Kernel and solver functions return `Result<_, KernelError>`, never panic in shipping paths, and a `catch_unwind` at the evaluator boundary turns any residual panic into a failed feature with a crash report, not a dead program.
