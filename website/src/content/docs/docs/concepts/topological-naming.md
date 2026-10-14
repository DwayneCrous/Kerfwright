---
title: Topological naming
description: "How a fillet or a sketch keeps pointing at the right face after you edit the model upstream."
sidebar:
  order: 2
---

Every parametric CAD system has to answer one awkward question: you pick a face, add a fillet to one of its edges, then go back and change the model. Which face is "the same face" now?

Systems that answer it badly produce the familiar failure: a fillet that jumps to a different edge, or a sketch that loses its plane. Kerfwright's answer is **persistent names** with an honest fallback.

## A name says how a face was made

A `TopoName` records the *recipe* for a face, never its position. Kinds include:

| Origin | Meaning |
|---|---|
| `SketchEdge` | The face swept out by one entity of one sketch, with a role (side, cap) |
| `Primitive` | A face of a built-in primitive, by role |
| `Generated` | A face created by a feature from another named face |
| `Split` | One piece of a face a later feature cut apart |
| `Merge` | A face made by joining several named faces |
| `Instance` | The same face on a copy made by a pattern or mirror, by source and index |

Because a name does not change when geometry moves, changing a sketch dimension or reordering unrelated features leaves it valid.

## A reference is a name plus a fingerprint

What a feature stores is a `Reference`: the name, and a `GeomFingerprint` of the face when it was picked. The fingerprint holds the surface kind, direction, centre, extent, radius, area and a hash of the neighbouring faces.

## How a reference is resolved

When a feature is rebuilt, its references are resolved in this order. The first step that succeeds wins.

1. **Exact name.** The face with that name exists.
2. **Lineage.** The named face was split or merged. If it was split into pieces, the reference follows the piece with at least 1.5 times the area of each other piece. If two pieces are nearly equal, the result is *Ambiguous* and is reported. If the cut that split it is gone, the reference resolves to the whole face again.
3. **Fingerprint.** A geometric match that must beat a score threshold **and** the runner-up by a margin. Displacement is measured against the face's own size, so the wall of one hole never matches the next hole in a pattern.
4. **Otherwise** the evaluation fails with `Unresolved`, listing the candidates.

The design rule is *report, never guess*. A missing instance of a pattern is flagged, never silently matched to its neighbour.

## Repair

A broken reference is not a dead end. The headless API offers `Document::repair_reference(feature, broken_name, replacement)`, and `Evaluation::healed(doc)` rewrites every reference that was followed by lineage or by geometry so that it resolves by exact name from then on. The application has no repair dialog yet, so a failed feature shows as a red chip with the engine's message.

## What the numbers say

The naming corpus has 6 families of generated parametric edits (a box with bosses, holes, pockets, patterns and slots): dimension changes, reorders, inserts upstream, pattern count changes and face splits and merges.

| Result on 1,716 scenarios | Count |
|---|---|
| Resolved by name | 1,060 |
| Resolved by lineage | 185 |
| Resolved by geometry | 271 |
| Correctly flagged as gone or ambiguous | 198 |
| Flagged although resolvable | 2 |
| **Resolved to the wrong face** | **0** |

That is 99.88 % resolved or correctly flagged, against a criterion of 500 scenarios, no silent errors and 97 % automatic.

:::caution[What this does not prove]
The corpus is generated, and its ground truth is analytic geometry, so it measures the resolution *policy* rather than every kind of model. The thresholds were tuned on this generator. In the 1,800-scenario run, 84 cases where the generator could not identify exactly one face are skipped and listed in the log.
:::

## Related

- [How the engine rebuilds](/docs/concepts/parametric-engine/)
- [Phase 4 report](https://github.com/DwayneCrous/Kerfwright/blob/main/docs/phase4-report.md) for the full corpus definition and policy.
