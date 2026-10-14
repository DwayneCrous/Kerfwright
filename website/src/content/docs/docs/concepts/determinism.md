---
title: Determinism
description: "Why a model and its names are identical on any number of threads, and how that is tested."
sidebar:
  order: 5
---

Parallel code often gives slightly different answers from run to run, because floating-point addition depends on the order of the additions. For a CAD system that is more than untidy. A different order can mean a different face name, and a different face name means a reference that resolves differently on your colleague's machine.

Kerfwright's rule is: **the same inputs give bit-identical results on any thread count.**

## How it is kept

- **Parallel across independent work only.** The sketch solver parallelises across independent components and clusters. The feature engine parallelises across independent features in a wave.
- **No cross-thread float reductions.** Each task writes its result to a slot chosen by index. Results are merged in a fixed order afterwards.
- **Fixed chunking, ordered merges.** Where a parallel loop must combine values, the chunk boundaries do not depend on the number of threads.
- **Sorted iteration where names are affected.** Anything that assigns names iterates in an order sorted by a canonical key.
- **Canonical output order.** After a Boolean, result faces are put in a canonical order (source face, then the position of the fragment) before shells are built and split ordinals are assigned.

That last rule was added because the test found a real bug. In the eight-part performance model the split ordinals of the Boolean's result depended on scheduling, so 1 thread and 32 threads produced different names. Fixing it required the canonical order, and it is now covered by a permanent test.

## The geometry hash

Equality has to be tested without depending on how a body happens to be stored. The geometry hash is **canonical**: independent of identifiers, slot order, the origin of a parametrisation and thread count. It samples surfaces and curves at 1e-7 mm quantisation, sorts loops and edges, and includes a hash of the content of the face names. The same hash is the *output hash* that powers early cutoff in [the evaluation cache](/docs/concepts/parametric-engine/).

## What is tested

| Test | Result |
|---|---|
| Sketch solve and drag compared bit for bit on pools of 1, 2, 4, 8, 16 and 32 threads against a single-threaded reference | Identical |
| A `.kwf` rebuilt from the recipe with no caches, on 1, 4 and 32 threads, each twice, for documents of 10, 50 and 122 features | Same geometry hash and same name set |
| Writing the same document twice | Same bytes |

## Limits

Determinism is tested within one build on one machine. It is not a promise about different compilers, CPU architectures or operating systems, because libm functions and fused multiply-add instructions can differ between them. Cross-platform behaviour has not been tested, since the project has only been run on Linux.
