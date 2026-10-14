---
title: How the engine rebuilds
description: "The feature timeline, the dependency graph, the evaluation cache and early cutoff."
sidebar:
  order: 1
---

Editing a dimension near the start of a long model should not cost a full rebuild, and it should never leave the model half-updated. This page explains the machinery in `kw-feature` that makes both true.

## The document

A document is an ordered list of **features**. Each has a stable numeric id, a name, a *suppressed* flag and a specification. The specification is a serde enum. When Phase 4 was written it had 20 kinds (Parameter, Sketch, DatumPlane, Extrude, Revolve, Loft, Sweep, Fillet, Chamfer, VariableFillet, Shell, Draft, Offset, Combine, Hole, LinearPattern, CircularPattern, Mirror, SheetMetal and Subd). Surface features, threads and imports have been added since.

Every number in a specification is a **parameter value**: either a literal quantity such as `10 mm`, or an expression such as `width / 2`. Expressions are evaluated by `kw-expr`, which understands units and checks dimensions, so `10 mm + 3` is an error that names the two dimensions, not a silent wrong number.

## Dependencies are derived, never stored

Nothing in the file says "feature 7 depends on feature 3". The engine works that out from what each feature reads:

- the **parameters** its expressions mention,
- the **explicit ids** it names (a sketch, a plane),
- the **faces** it references, which point back to the feature that created them,
- the **body** it modifies: the previous solid feature, or an explicit base.

A suppressed feature passes its input straight through. Because dependencies are derived, they cannot go stale or disagree with the geometry.

## Timeline edits are checked before they happen

Every edit to the timeline is applied to a *copy* first and validated:

| Edit | What the check refuses |
|---|---|
| Move a feature | A feature that would come before something it reads (`ForwardReference`) |
| Change an expression or a reference | A dependency cycle, reported with its path, for example `a → b → a` |
| Rename | An unknown or duplicate name |
| Delete | A feature that others still read (`HasDependents`), unless you delete the dependants too |

A refused edit changes nothing. In the app this is the message you see when a dragged chip snaps back. Undo and redo keep snapshots of the whole document, and each snapshot is shared, not copied.

## The evaluation cache

Each feature's result is stored under an **input hash** made from:

1. the kernel version,
2. the feature's id,
3. its specification with every expression already evaluated,
4. the output hash of everything it reads.

If the key matches, the cached result is used. So an edit re-runs exactly the features whose inputs changed, and nothing else.

### Early cutoff

A result also has an **output hash**: a hash of its canonical geometry plus the set of face names. If you edit a feature and it re-runs but produces the *same* output, its dependants see the same input hash as before and stay cache hits. This is why a no-op edit costs almost nothing.

Measured on a 201-feature document, an edit that rewrites a parameter as an equal expression, a rename, and an undo/redo each re-executed **0 features** and took 0.6 ms.

## Waves and parallelism

After an edit the engine marks everything downstream as dirty, sorts the dirty set topologically and groups it into **waves**: features in the same wave do not depend on each other, so they run in parallel on a Rayon pool.

```text
edit Sketch2.dim ─► mark dirty ─► topological order ─► waves
  Sketch1   cached                Wave 0: Sketch2*
  Sketch2*  dirty                 Wave 1: Extrude2*, Hole1*   (parallel)
  Extrude1  cached                Wave 2: Boolean2*
  Extrude2* dirty                 Wave 3: Fillet1*, Pattern1* ─► cutoff if output unchanged
```

Only the bodies that changed are tessellated again and uploaded to the GPU.

## When a feature fails

A kernel failure, or even a panic inside a kernel call, becomes a `Failed` state with a message. The failing feature **suspends its dependants** and keeps its **last good output**, so the model stays on screen instead of vanishing. In the timeline the chip turns red. Fix the cause and the dependants resume.

Long evaluations can be cancelled. Each interactive command takes a new cancellation token, and an evaluation from an older generation returns `Cancelled` at its next checkpoint, at most about a millisecond of kernel work later.

## Measured behaviour

| Model | Cold replay | Edit near the end | No-op edit |
|---|---|---|---|
| 8 parts × (plate + 11 holes and bosses) + 7 joins + 1 parameter, 200 features | 335 ms on 8 threads (1,327 ms on 1 thread) | 52–55 ms, 4 features re-executed | 0 executed, 0.6 ms |
| One chain: a plate and 99 hole or boss features, 201 features | 8.7 s | 192–202 ms | 0 executed, 0.6 ms |

The first row meets the roadmap targets. The second does not: one body that grows through 100 Booleans cannot use the parallel waves, and each Boolean spends its time on large faces with many hole boundaries. A further speed-up would need converted faces cached between Booleans, which is kernel work. The report states this plainly, and so does this page.

## Related

- [Topological naming](/docs/concepts/topological-naming/): how a reference to a face survives an edit.
- [Determinism](/docs/concepts/determinism/): why the answer does not depend on thread count.
- [The .kwf project file](/docs/architecture/kwf-format/): what is stored, and why caches are not.
