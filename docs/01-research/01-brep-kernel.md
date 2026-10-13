# 01 — B-Rep Geometric Kernel & Topology

Labels: **[Public] [OSS] [Inferred] [Design]** (see `CLEAN_ROOM.md`).

## 1. What the commercial kernels store

| Kernel | Lineage | Topology classes (as publicly documented) |
|---|---|---|
| **ACIS** (Spatial/Dassault) | 1980s Spatial Technology | Body → Lump → Shell → (Subshell) → Face → Loop → **Coedge** → Edge → Vertex; plus Wire. Coedges are doubly linked (`next`/`prev`) and have a `partner` pointer for the radial neighbour. [Public] |
| **Parasolid** (Siemens) | Shape Data, 1988 | Body → Region (solid or void) → Shell → Face → Loop → **Fin** → Edge → Vertex. Fins are the oriented edge uses; an edge has a ring of fins. Transmit format (XT) is publicly documented. [Public] |
| **ASM** (Autodesk Shape Manager) | Forked from ACIS by Autodesk in the early 2000s; used by Inventor and Fusion | Understood to follow the ACIS topology model. [Public: Autodesk states ASM derives from ACIS; Inferred: internals track ACIS's] |
| **OpenCASCADE** | Matra Datavision | Compound → Solid → Shell → Face → Wire → Edge → Vertex; orientation flags on shared TShapes; `BRepBuilderAPI_MakeShape::Modified/Generated/IsDeleted` history API. [OSS] |

Common points across all of them: topology is separate from geometry; faces reference surfaces, edges reference 3D curves (and, for trimmed faces, 2D parametric *pcurves* on the face's surface); each edge-use carries an orientation relative to the edge; *tolerance* lives on topology (tolerant edges/vertices in Parasolid/OCCT, global resolution `SPAresabs`-style constants in ACIS).

## 2. Connectivity structures

| Structure | Edge→faces | Manifold only? | Notes |
|---|---|---|---|
| Winged-edge (Baumgart) | exactly 2 | Yes | Edge stores 4 wing pointers; orientation case analysis is painful. |
| Half-edge / DCEL | exactly 2 (twin) | Yes | Compact, fast traversal; basis of most mesh libraries. |
| **Coedge + partner ring** (ACIS/Parasolid) | ≥ 1 via ring | **Yes and limited non-manifold** | Closest to production CAD; what Kerfwright adopts. |
| **Radial-edge** (Weiler 1986) | N faces around an edge in a radial cyclic order | **Fully non-manifold** | Needed for general sheets/wires/solids mixed in one body, and for Boolean *intermediate* results. |

**[Design]** Kerfwright uses an ACIS-style hierarchy with *coedges* (a.k.a. half-edges with a face-loop owner) and a **radial ring** on every edge (`radial_next`). A manifold edge's ring has two coedges; sheet bodies have one; non-manifold junctions have ≥3. This subsumes half-edge and radial-edge in one representation.

Euler–Poincaré invariant for validity checks: `V − E + F − R = 2(S − H)` where R = inner loops (rings), S = shells, H = through-holes (genus). Every kernel op ends with this check in debug builds.

## 3. Persistent indexing with generational maps

* In-memory handles: `slotmap` keys (`u32 index` + `u32 version`). A stale key after deletion fails lookup instead of aliasing a new entity. Per-body arenas keep keys small and cache-friendly. [Design]
* **Generational keys are not persistent names.** They are only valid within one evaluation of one body. Cross-edit identity is the job of the topological-naming layer (see `03-parametric-dag.md`). Mixing the two is the most common architectural mistake.
* Layout: struct-of-arrays inside each arena where hot loops exist (tessellation reads face→surface→loops; Boolean reads edge→curve→bbox); AoS elsewhere. Geometry (curves/surfaces) lives in its own arenas so many topological entities can share one surface (e.g. split faces).

## 4. Parametric representations

### Curves
| Type | Definition |
|---|---|
| Line | `C(t) = P + t·d` |
| Circle | `C(t) = O + r(cos t·X + sin t·Y)` |
| Ellipse | `C(t) = O + a cos t·X + b sin t·Y` |
| Parabola / Hyperbola | `O + t²/(4f)·X + t·Y` / `O + a cosh t·X + b sinh t·Y` |
| **Rational B-spline (NURBS)** | `C(u) = Σᵢ Nᵢ,ₚ(u) wᵢ Pᵢ / Σᵢ Nᵢ,ₚ(u) wᵢ`, basis by Cox–de Boor recursion over knot vector `U` (clamped, non-decreasing). Evaluate in homogeneous 4D `(wP, w)` and project. |
| Procedural | Intersection curves, offset curves, projected curves, blend spines — stored as *definition + cached B-spline approximation within tolerance* (ACIS "intcurve"-style). [Public concept] |

Exact conics as rational quadratics: a circular arc of angle θ ≤ 90° is three control points with middle weight `cos(θ/2)`; full circle = 9 control points, 4 spans (Piegl & Tiller, *The NURBS Book*). [Public]

### Surfaces
| Type | Definition |
|---|---|
| Plane | `S(u,v) = O + uX + vY` |
| Cylinder | `O + r(cos u·X + sin u·Y) + v·Z` |
| Cone | `O + (r + v tan α)(cos u·X + sin u·Y) + v·Z` |
| Sphere | `O + r(cos v(cos u·X + sin u·Y) + sin v·Z)` |
| Torus | `O + (R + r cos v)(cos u·X + sin u·Y) + r sin v·Z` |
| Extrusion / Revolution | Generic curve swept along a vector / about an axis |
| **NURBS surface** | `S(u,v) = ΣᵢΣⱼ Nᵢ,ₚ(u) Nⱼ,q(v) wᵢⱼ Pᵢⱼ / ΣᵢΣⱼ Nᵢ,ₚ(u) Nⱼ,q(v) wᵢⱼ` |
| Offset | `S(u,v) + d·n(u,v)` (self-intersection / non-regular handling required) |
| Blend (procedural) | Rolling-ball fillet surfaces, stored as spine + radius law + support surfaces |

Core NURBS toolkit (all required, all [Public] in Piegl & Tiller): knot insertion/removal, degree elevation/reduction, Bézier decomposition, point inversion/closest-point (Newton with multi-start), global/local interpolation & least-squares fitting, derivative evaluation to order ≥ 3 (needed for G2/G3 and curvature-based tessellation).

### Tolerance model
Single document-scale `Tolerance { linear, angular }` (default linear 1e-6 model units, angular 1e-10 rad), a *modelling box* bound (Parasolid documents ±500 m with 1e-8 m resolution; ACIS uses a comparable `resabs`), and **per-entity tolerance** on edges/vertices that grows when healing. [Public concept; values are Design]

## 5. Surface–Surface Intersection (SSI)

Taxonomy (Patrikalakis & Maekawa, *Shape Interrogation for CAD/CAM*): algebraic, subdivision, lattice, continuation/marching, hybrid. [Public]

Kerfwright pipeline **[Design]**:
1. **Analytic fast paths**: plane–plane (line), plane–{cyl, cone, sphere, torus} (conics, quartics for torus), coaxial/coplanar-axis quadric pairs, sphere–sphere (circle). These cover ~80% of mechanical CAD.
2. **Seed finding** for general pairs: hierarchical subdivision with bounding volumes (OBB or control-polygon hulls on Bézier-decomposed patches) → candidate patch pairs → Newton refine to seed points on every loop/branch (also at boundary edges and at critical points where the intersection is tangent).
3. **Tracing**: marching along `t = n₁ × n₂`; adaptive step from curvature of the intersection curve; Newton correction onto both surfaces each step; terminate at boundary, at a seed already visited (closed loop), or at a singular point.
4. **Singular/tangent cases**: detect `|n₁ × n₂| → 0`; branch using second-order (Hessian-based) tangent analysis; fall back to subdivision with tolerance-driven termination.
5. **Fit**: curve fit to a B-spline within `tol/10`; store pcurves on both surfaces.
6. **Robustness**: interval arithmetic for in/out decisions; Shewchuk adaptive predicates for the polyhedral (tessellated) pre-classification.

Known hard cases to put in the regression corpus: tangent cylinders, near-coincident faces, thin slivers, tori with self-intersecting offsets, tiny gaps within tolerance.

## 6. Boolean operations

**[Public]** pipeline (Mäntylä; Hoffmann *Geometric & Solid Modeling*; Requicha–Voelcker):
1. Broad phase: BVH on face bounding boxes (parallel traversal with Rayon).
2. Narrow phase: SSI for each overlapping face pair → intersection curves, edge–face intersection points.
3. **Imprint / split**: insert the curves as edges into both bodies; split faces along them in parameter space (planar arrangement in `(u,v)` of each face including periodic seam handling).
4. **Classify** each resulting face as IN / OUT / ON relative to the other body: propagate classification across shared edges from one seed face classified by ray-casting/parity or winding number; ON faces resolved by normal agreement (same/opposite).
5. **Select** faces by operation (union: OUT∪ON-same; subtract: …; intersect: …).
6. **Stitch** (sew): merge coincident edges within tolerance, orient coedges, close shells, discard slivers.
7. **Heal & validate**: Euler–Poincaré, no free edges for solids, shell orientation (outward volume > 0), tolerance growth report.

Hard parts in order of cost: coincident/tangent faces, tolerant geometry, periodic/degenerate parameter domains (poles, seams), and *determinism* (so topological naming works).

**CSG** is the *semantic* layer (a timeline of booleans and primitives); B-rep is the *evaluated* layer. A CSG tree is kept in the feature DAG, never as the kernel's data model.

## 7. Stitching / sewing
Edge-matching by position and tangent within tolerance via spatial hashing (R-tree) → union-find on vertices → merge edges → orient shells by propagating coherent coedge direction → compute signed volume to flip to outward. Used for STEP import, surface-patch import, and as step 6 above.

## 8. Fillets, chamfers, blends
* **Rolling-ball fillet** [Public]: the blend surface is the envelope of a sphere of radius `r` tangent to both support surfaces. Compute offset surfaces `S₁ ⊕ r` and `S₂ ⊕ r`; their intersection is the **spine** (ball-centre locus); project spine onto supports to get **contact curves**; cross-section is a circular arc (exact rational quadratic). Then: trim supports at contact curves, insert blend face, repair adjacent topology.
* **Variable radius**: `r(t)` is a law curve (B-spline in the spine parameter); contact curves are found by solving two tangency equations per station. Variable *chamfer* uses distance laws `d₁(t), d₂(t)` or distance + angle.
* **Vertex blends / setbacks**: where 3+ blended edges meet — corner patch (Coons/Gregory/N-sided) with setback; the largest source of failures. Overflow handling when the ball hits adjacent faces ("cliff edges") requires rolling-ball over a *chain* of faces.
* **G2 (curvature-continuous) blends**: conic-section cross-sections or curvature-matched splines; defer to a later phase.
* ⚠ Some blend methods are patented (see `LICENSES/patent-watch.md` once created); prefer methods from the public academic literature (Vida/Martin/Varady survey; Choi & Ju).

## 9. T-splines vs SubD

| | T-splines | Catmull–Clark SubD |
|---|---|---|
| Control mesh | Quad mesh with T-junctions; each control point has local knot-interval vectors | Quad (or n-gon) mesh with extraordinary vertices |
| Surface | Rational bicubic spline, blending functions from local knot vectors (Sederberg et al., SIGGRAPH 2003) | Limit surface of recursive subdivision; bicubic B-spline in regular regions |
| Local refinement | Yes, without changing the surface | Global (uniform) refinement, adaptive with extra bookkeeping |
| Exactness vs NURBS | T-spline **without** T-junctions = NURBS; any T-spline can be converted into a NURBS by knot insertion but with extra control points [Public] | Regular regions = exact bicubic B-spline patches; near extraordinary vertices the limit surface is not a finite NURBS (infinite sequence of patches) |
| Evaluation at extraordinary points | T-NURCC generalises Catmull-Clark, so same issue | Exact via eigen-decomposition (Stam 1998) |
| Conversion to B-rep | Regular regions → exact bicubic NURBS patches via Bézier extraction; extraordinary regions → **G1 approximation** (e.g. Gregory/Peters patches, Loop–Schaefer approximate CC) within tolerance | Same |
| Legal | ⚠ T-spline patents (e.g. US 7,274,364, T-Splines Inc. → Autodesk 2011); expiry/continuations must be checked by counsel | Catmull-Clark patents long expired |

**[Design]** Phase 4 ships **Catmull–Clark SubD with creases** as the "Form" workspace and a **Finish → B-rep** step (exact patches in regular regions + G1 patches around extraordinary points, with deviation reporting). True T-splines (T-junction local refinement) are gated on the patent review. Analysis-suitable T-splines (Li/Scott) are noted as a future research item.

## 10. Tessellation (the viewport's contract with the kernel)
* Edge-first: discretise B-rep edges by chordal deviation + angular deviation, then faces so neighbouring faces share identical edge vertices (crack-free).
* Face meshing: trim-curve-aware constrained Delaunay in `(u,v)` (e.g. `spade`) with interior points from curvature-adaptive grid; map to 3D; normals analytic.
* Output carries `face_id` and `edge_id` per primitive for GPU picking and for sharp-edge rendering from *topological* edges.

## Sources
- Piegl & Tiller, *The NURBS Book*, 2nd ed., Springer 1997.
- Hoffmann, *Geometric and Solid Modeling*; Mäntylä, *An Introduction to Solid Modeling*.
- Weiler, "Edge-based data structures for solid modeling in curved-surface environments," IEEE CG&A 1985; radial-edge: Weiler 1986.
- Patrikalakis, Maekawa, Cho, *Shape Interrogation for CAD/CAM* — https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/mathe.html
- Boundary representation overview — https://en.wikipedia.org/wiki/Boundary_representation
- Sederberg, Zheng, Bakenov, Nasri, "T-splines and T-NURCCs," SIGGRAPH 2003 — https://history.siggraph.org/?p=115890 ; patent US7274364 — https://patents.google.com/patent/US7274364
- Stam, "Exact evaluation of Catmull-Clark subdivision surfaces at arbitrary parameter values," SIGGRAPH 1998.
- Truck (Rust B-rep/NURBS kernel) — https://github.com/ricosjp/truck
- Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates."
- Vida, Martin, Várady, "A survey of blending methods that use parametric surfaces," CAD 1994.
