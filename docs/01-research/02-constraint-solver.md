# 02 — 2D Geometric Constraint Solver

## 1. Formulation
Unknown vector `x ∈ ℝⁿ` (point coordinates, radii, angles, spline control points). Each constraint contributes residual equations `fₖ(x) = 0`; the system is `F(x) = 0`, `F: ℝⁿ → ℝᵐ`, Jacobian `J = ∂F/∂x` (sparse: each row touches 2–8 variables). Geometry remains in the user's sketch coordinates; the solver moves it minimally.

### Constraint library (residuals) **[Design, standard formulations]**
| Constraint | Equations |
|---|---|
| Coincident (pt–pt) | `p₁ − p₂ = 0` (2) |
| Point on line | `(p − a) × (b − a) = 0` (1) |
| Point on circle | `‖p − c‖² − r² = 0` (1) |
| Horizontal / Vertical | `y₁ − y₂ = 0` / `x₁ − x₂ = 0` (1) |
| Parallel | `d₁ × d₂ = 0` (1) |
| Perpendicular | `d₁ · d₂ = 0` (1) |
| Concentric | `c₁ − c₂ = 0` (2) |
| Equal length / radius | `‖d₁‖² − ‖d₂‖² = 0` / `r₁ − r₂ = 0` (1) |
| Distance (pt–pt) | `‖p₁ − p₂‖² − d² = 0` (1) — squared form avoids the √0 derivative singularity |
| Distance (pt–line) | `n̂·(p − a) − s·d = 0`, `s = ±1` fixed from the initial configuration (1) |
| Angle (line–line) | `d₁ · d₂ − ‖d₁‖‖d₂‖ cos θ = 0` plus sign/branch flag (1) |
| Tangent line–circle | `n̂·(c − a) − s·r = 0` (1) |
| Tangent circle–circle | `‖c₁ − c₂‖² − (r₁ + s·r₂)² = 0` (1), `s=+1` external / `−1` internal |
| Symmetric, Midpoint, Fixed, Collinear | combinations of the above |

**Branch flags** (`s`) are captured when the constraint is created from the current geometry. Without them tangent/angle/offset constraints flip between valid roots during drag. [Design]

### Curvature continuity at a joint between curves A and B (end parameter conventions fixed per curve)
| Level | Condition | Added equations (2D) |
|---|---|---|
| G⁰ | `A(end) = B(start)` | 2 |
| G¹ | G⁰ + `A′ × B′ = 0` with `A′·B′ > 0` (same direction branch flag) | +1 |
| G² | G¹ + `κ_A = κ_B`, where `κ = (x′y″ − y′x″)/‖r′‖³` (signed) | +1 |
| G³ | G² + `dκ/ds` equal; `dκ/ds = ((x′y‴ − y′x‴)‖r′‖² − 3(x′y″ − y′x″)(r′·r″)) / ‖r′‖⁶` | +1 |

Spline control points are variables, so G²/G³ constraints couple the first 3–4 control points of each curve at the joint. Curvature constraints are *numerically stiff*; scale equations by local length scale and solve them last (in a second stage) after G⁰/G¹ have converged. [Design]

## 2. Numerical solvers
| Method | Use | Pros | Cons |
|---|---|---|---|
| **Newton–Raphson** | Square, well-posed systems | Quadratic convergence | Diverges from poor starts; fails on singular J |
| **Gauss–Newton w/ min-norm step** | Under-determined (m < n) | Picks the *smallest movement* solution — the CAD-desirable one | Needs rank-revealing QR/SVD |
| **Levenberg–Marquardt** `(JᵀJ + λD)Δ = −JᵀF` | Default workhorse | Robust to rank deficiency, handles over-constraint gracefully | Squares condition number if normal equations are used → prefer QR on `[J; √λ·D½]` |
| **Powell dog-leg** | Trust-region alternative | Fewer factorizations per iteration | More code |
| **BFGS** on `½‖F‖²` | Fallback | No Jacobian rank issues | Linear convergence, slow, local minima with `F≠0` |
| **SQP / constrained min** | Dragging (minimise ‖x − x_drag‖² s.t. F(x)=0) | Correct physical behaviour | Heavier; PlaneGCS offers it |

**[Design] Chosen stack:** analytic sparse Jacobians → per-cluster damped Gauss–Newton/LM using rank-revealing QR → dog-leg fallback → BFGS last resort. Finite-difference Jacobian and forward-mode dual-number AD are used **only as test oracles** for the hand-coded derivatives.

Convergence criteria: `‖F‖∞ < 1e-10·scale`, `‖Δ‖ < 1e-12·scale`, max 100 iterations; warm start from previous solution.

### Honest performance model
Dense factorization of a 500-entity sketch (≈1 500 variables) costs ≈ ⅔n³ ≈ 2 GFLOP ≈ tens of ms. A **<1 ms** target is therefore only reachable via (a) **decomposition** into clusters of ≤ ~30 variables (each factorization ≈ microseconds), and (b) **incremental re-solve** of only the clusters downstream of a dragged entity with warm starts. The PRD splits the SLA accordingly: *drag/edit* ≤ 1 ms, *cold full solve* ≤ 25 ms.

## 3. Graph-based decomposition

Constraint graph: bipartite (variables ↔ equations) or entity graph with constraints as edges. [Public] literature:
* **Structural DOF counting / Laman's theorem** (2D rigidity of bar-joint frameworks: `2n − 3` edges and `≤ 2k − 3` in every k-subset). The **pebble game** (Jacobs & Hendrickson 1997) decides this in O(n²) and identifies rigid clusters and over-constrained regions.
* **Dulmage–Mendelsohn decomposition** of the (variables × equations) incidence matrix by maximum bipartite matching: splits into under-, well-, and over-determined blocks, and the well-determined block into strongly-connected components that can be solved in sequence (block-triangular form).
* **Cluster / DR-planning** (Fudos & Hoffmann 1997; Owen 1991 – the basis of D-Cubed's DCM): build the graph, repeatedly merge rigid triangles into clusters, solve each cluster in a local frame, then solve the cluster-assembly problem as a smaller system. Handles ruler-and-compass-solvable systems directly; leaves a small "irreducible" core for numerics.
* Generic (structural) analysis is only a *necessary* condition: special positions (parallel lines, collinear points) make Jacobian rank drop below the generic rank. Hence structure picks the *decomposition*, numerics decide the *classification*.

**[Design] Algorithm**
1. Build bipartite graph; connected components (union-find) → independent subsystems, solved in parallel with Rayon.
2. Within a component: DM decomposition → block-triangular order.
3. Within an irreducible block: pebble-game rigid-cluster merge when it is a bar-joint-like subsystem; otherwise numeric.
4. Solve blocks in order; on drag, re-solve only blocks reachable from the dragged variable.

## 4. DOF analysis and classification

At a (near-)solution `x*`:
* `r = rank(J)` via column-pivoted QR / SVD with tolerance `ε·σ₁·max(m,n)`.
* `DOF = n − r − (number of unremoved rigid-body motions, 3 if the sketch is not anchored)`.
* Null space of `J` (last `n − r` right singular vectors) gives the **motion directions** → UI colours entities that can still move (conventional "blue vs. black").
* Redundancy: `m − r > 0` ⇒ some constraints dependent.
  * Residual `‖F(x*)‖ ≈ 0` ⇒ **redundant but consistent** (warn).
  * Residual `> tol` ⇒ **conflicting** (error); find a minimal conflicting set by a *deletion filter* (drop each constraint, re-solve; keep those whose removal restores solvability) or via pivoted-QR ordering of rows (the rows that fall in the dependent tail are the suspects).

| Status | Condition |
|---|---|
| Under-constrained | `DOF > 0`, residual ≈ 0 |
| Fully constrained | `DOF = 0`, `rank = n − 3 or n`, residual ≈ 0 |
| Over-constrained (redundant) | `m > r`, residual ≈ 0 |
| Conflicting / unsolvable | residual > tol after LM |

## 5. Failure modes and mitigations
| Failure | Mitigation |
|---|---|
| Branch flip when dragging | Branch flags; solve from previous state; limit step per frame |
| Singular Jacobian (zero-length line, zero radius) | Variable bounds (`r ≥ ε`), LM damping, degenerate-geometry guard |
| Poor scaling (µm vs m) | Per-variable scaling by sketch bounding box |
| Spline G² stiffness | Staged solve; equation scaling; optional relaxed (soft) curvature mode |
| Non-determinism from parallel clusters | Fixed cluster ordering; no floating-point reductions across threads |

## 6. Prior art and licensing
* **SolveSpace** (GPL): constraints as symbolic expressions, modified Newton. Read the documentation, not the code. [OSS]
* **PlaneGCS** (FreeCAD, LGPL): DogLeg/LM/BFGS/SQP, DM-style decomposition. [OSS]
* **D-Cubed DCM** (Siemens): proprietary, public through Owen's and Hoffmann's papers. [Public]
* Rust today: `slvs` (bindings to SolveSpace, not pure), `arael-sketch-solver` (small), `levenberg-marquardt` (dense). None gives sparse, clustered, drag-aware DOF analysis → build in-house (`kw-sketch`).

## Sources
- Review: "A review on geometric constraint solving," https://arxiv.org/pdf/2202.13795
- SolveSpace technology notes — https://solvespace.com/tech.pl ; constraints — https://solvespace.com/constraints.pl
- planegcs docs — https://planegcs.readthedocs.io/en/stable/
- "From Sketchpad to D-Cubed" survey — https://novedge.com/blogs/design-news/design-software-history-from-sketchpad-to-d-cubed-the-evolution-and-algorithms-of-constraint-based-sketchers
- Jacobs & Hendrickson, "An algorithm for two-dimensional rigidity percolation: the pebble game," J. Comput. Phys. 1997.
- Fudos & Hoffmann, "A graph-constructive approach to solving systems of geometric constraints," ACM TOG 1997.
- Owen, "Algebraic solution for geometry from dimensional constraints," ACM Solid Modeling 1991.
- Nocedal & Wright, *Numerical Optimization*; Moré, "The Levenberg–Marquardt algorithm: implementation and theory."
