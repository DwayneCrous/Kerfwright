---
title: The sketch solver
description: "Constraints as equations, decomposition, the solver, degrees of freedom and dragging."
sidebar:
  order: 3
---

A sketch is a set of geometric entities and a set of constraints between them. The solver finds positions that satisfy every constraint, tells you what is still free, and keeps up while you drag.

## Entities and constraints

**Entities:** point, line, circle, arc, ellipse, a clamped-uniform B-spline and a fit spline (a spline through chosen points).

**Constraints**, 13 in the app: coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, fix, midpoint, concentric, collinear, point on curve and symmetric. **Dimensions** are constraints with a value: distance, angle, radius and diameter. The solver library goes further and has G0, G1 and G2 joints between lines, arcs and B-splines.

Each constraint is a small set of equations called *residuals*, which are zero when the constraint holds. For example "these two lines are perpendicular" is one residual: the dot product of their directions. Every residual has a hand-written analytic Jacobian, and these are checked against finite-difference and forward-mode automatic differentiation oracles on a million random cases.

## How a solve runs

1. **Split into components.** Entities that share no constraints are solved independently.
2. **Decompose.** Each component goes through a Dulmage–Mendelsohn block-triangular decomposition, so a sketch of many small features becomes many small systems instead of one big one.
3. **Solve each cluster.** A damped Gauss–Newton / Levenberg–Marquardt iteration, on a sparse Cholesky factorisation whose symbolic structure is cached between solves.
4. **Fall back** if a cluster does not converge: merge clusters, re-decompose, and then try dense Levenberg–Marquardt, dog-leg and BFGS.

## What the colours mean

The solver also *analyses* the sketch. That is what drives the colours on screen:

| Report | Meaning |
|---|---|
| Degrees of freedom | How many independent motions are left. Zero means fully constrained. |
| Free entities | Which parts can still move (drawn blue). |
| Redundant constraints | Constraints that repeat what others already say (yellow). |
| Conflicting set | The smallest group of constraints that cannot all hold (red). It is found by a deletion filter and is exactly minimal in 64 of 64 planted test conflicts. |

Exact rank is computed for components up to 350 unknowns. Larger ones use a block-wise lower bound and report `exact: false`.

## Dragging

Dragging a point pins it to the cursor, solves the region around it, applies a soft pull and then an exact polish. **Branch-flip prevention** stops a tangent or angle constraint from jumping to its mirror-image solution mid-drag: the cursor target is clipped to stay on the current branch. That means a very fast drag can lag the cursor around a tangent. It is the intended trade-off, and without it the same 100 reference sketches flip 3,281 times.

## Performance and its limits

| Case | Result |
|---|---|
| Drag, 493 entities, decomposable | p99 **0.22 ms** (p50 0.022 ms) |
| Cold solve, 493 entities | 0.44 ms |
| Cold solve, 4,998 entities | 1.5 ms |
| Drag, 493 entities, every constraint chained to the next | p99 **8.6 ms** (p50 0.43 ms) |

The 1 ms drag target is asserted on sketches that split into small blocks, which is what real parametric sketches look like. A long chain has long dependency chains and exceeds it. A large irreducible block of hundreds of mutually coupled entities was not benchmarked.

## Not done

- Trim, extend and offset work on lines, arcs and circles (offset on chains of lines and arcs). Splines and ellipses are not trimmed or offset in the solver's operations.
- Tangency is line, circle and arc only. Splines connect through G0 to G2 joints.
- G3 joints are not implemented.

## Related

- [Determinism](/docs/concepts/determinism/): solves are bit-identical on 1 to 32 threads.
- [Design workspace](/docs/workspaces/design/): snapping, regions and how a sketch becomes a feature.
