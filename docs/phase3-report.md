# Phase 3 report — B-rep modelling kernel

Status: implemented and measured on 2026-10-09/11. Nothing is committed. Numbers below were produced on a 16-thread
x86-64 Linux machine (release builds, rayon default thread pool) by the commands in the last section;
`scripts/phase3_acceptance.sh` repeats all of them.

**Verdict in one line:** criteria 1, 2, 3, 4 and 6 are met as measured (with the caveats listed under each), criterion 5 is
met for geometry and validity but "≥ 3 independent external CAD tools" is only partly met. A second pass (2026-10-11)
removed the limits the first pass had listed as "not implemented": vertex blends, variable-radius fillets beyond straight
plane/plane edges, NURBS-against-NURBS intersections and Booleans, the 97 % success rate on free-form bodies, and the two
performance SLAs (the Boolean and fillet timings are now inside their budgets; see criterion 6 and the changes list below).

## Second pass: what changed

| Item | Before | Now |
|---|---|---|
| Vertex blends | adjacent fillets met in a mitre | rolling-ball sphere patch where three equal-radius fillets of three planar faces meet (convex or concave corner); the rounded box matches the closed-form (Steiner) volume to 1e-9 |
| Variable radius | linear law, straight edge between two planes | any law (`RadiusLaw::{Linear, Table, Function}`): straight edges between planes and between a plane and a cylinder parallel to the edge, closed circular rims between coaxial plane / cylinder / cone faces |
| NURBS × NURBS | `Err(Ssi(Unsupported))` | traced against a signed-distance field of the second surface; Booleans of spline extrusions, revolutions, lofts and offsets now work |
| Free-form body fuzz | 97.12 % of operations | **99.83 %** (18,000 operations over 6,000 pairs), 0 invalid outputs |
| Boolean 10k × 10k | 0.76 – 1.03 s | 0.32 – 0.43 s |
| Analytic Boolean fuzz, valid-expected cases | 99.71 % | 99.80 % |
| Fillet, 50 edges, 2,002 faces | ~480 ms | ~120 ms |

## What was built

| Crate | Content |
|---|---|
| `kw-nurbs` (extended) | analytic `param_of` / closest point, exact analytic offsets, curve–curve intersection, hermite fitting, pcurve builder (exact forms for plane/cylinder/cone/sphere, fitted otherwise) |
| `kw-brep` (extended) | uv trim geometry with a grid index, exact quadrature mass properties (volume, area, centroid, inertia), point-in-solid classification with a BVH, `copy_body_from`, `merge_bodies`, `add_face_auto` (face construction from bare edge chains), ShapeHistory v2 (`compose`, `source_of`) |
| `kw-ssi` | exact closed forms (plane/quadric/torus, coaxial, parallel cylinders) + certified level-set tracing with adaptive Hermite refinement; pcurves on both surfaces; **signed-distance field of any parametric surface** (spline, extrusion, revolution, procedural) so that two free-form surfaces can be intersected; traced curves are cut to the other surface's window (and at the seam of a closed spline); special case for a spline face tangent to an analytic face along a boundary iso-curve |
| `kw-boolean` | broad phase → SSI per face pair → clipping → global edge/vertex registry → arrangement per face → classification → selection → assembly, `verify` before returning; every failure is an `Err` |
| `kw-model` | `Profile`, extrude (symmetric), revolve, sweep (rotation-minimising frames), loft, **closed (periodic) loft `loft_ring`**, shell, offset, draft; a topology-preserving "rebuild" engine behind offset/draft/shell |
| `kw-blend` | constant-radius fillet, chamfer (two distances), **vertex blends** (sphere patches), **variable-radius fillet with an arbitrary radius law** on straight edges (plane/plane, plane/cylinder) and closed circular rims (plane/cylinder/cone); realised as Booleans with a swept (lofted) spandrel tool |
| `kw-step` | own ISO 10303-21 parser (limits, no panics on garbage), AP203/214/242 B-rep reader and writer (`step_tool` example for the oracle scripts) |
| `kw-iges` | IGES 5.3 reader (entities 100/102/110/124/126/128/141–144/186/502/504/508/510/514), trimmed-surface sewing into solids |
| `kw-feature` | thin `KernelEvaluator`: extrude/revolve/loft/sweep/Boolean/fillet/variable fillet/chamfer/shell/draft/offset features with persistent names derived from the history |

Every operation returns a `ShapeHistory`; `tests` assert that each output face has an origin and every input face
is either preserved or reported deleted (extrude, revolve, loft, sweep, Boolean, fillet/chamfer, offset, shell, draft).

## Acceptance criteria

### 1. Boolean fuzz — met
`cargo test -p kw-boolean --release --test fuzz` with `KW_BOOL_FUZZ_PAIRS=100000`: 100,000 random operand pairs ×
union/intersect/subtract = 300,000 operations. Operands are box, cylinder, cone, sphere, torus with random rigid
transforms, in categories *generic*, *coplanar boxes*, *cylinder on box*, *coaxial cylinders*, *contained*, *disjoint*,
*near-miss* (offsets 1e-9…1e-3) and *tangent*.

| | |
|---|---|
| invalid outputs (`validate()` fails, non-positive volume) | **0** |
| panics / hangs | **0** |
| volume identity violations (see 2) | **0** |
| success on valid-expected categories | **269,854 / 270,393 = 99.80 %** |
| tangent category (not valid-expected) | 22 % succeed; the rest are `Err(Tangent)` / `Err(Topology)` |

The 539 failures on valid-expected cases are all `Err` values (`Topology` 439, `Ssi` 87, `Classification` 12, `Invalid` 1: the
post-assembly check refused the result). Caveats, stated plainly:

* **NURBS operands.** The generator above uses analytic primitives; free-form surfaces are covered by a second fuzz over extruded /
  revolved / spline-prism / bottle bodies (`kw-model/tests/fuzz_models.rs`, 6,000 pairs × 3 ops = 18,000 ops):
  **99.83 %** succeed (16,907 ok + 1,062 empty results; 31 `Err`: `Topology` 22, `Classification` 6, `Ssi` 3), 0 invalid outputs,
  all volume identities hold to 1e-8. This was 97.12 % when NURBS–NURBS pairs returned `Err(Ssi(Unsupported))`; they are now intersected
  (see criterion 3), and the remaining dominant failure mode that the fuzz exposed — an intersection curve that crosses the seam of a
  closed spline surface ending 1e-5 short of the seam, so that the two halves did not meet — is fixed.
  `kw-model/tests/nurbs_nurbs.rs` adds deterministic spline/spline Boolean checks (including a prism crossing another at a right angle) and
  compares the point-in-solid classification of a spline prism with an independent test.
* "Valid-expected" is decided by the generator (the answer is known to be representable), not by an external kernel.

### 2. Inclusion–exclusion volumes within 1e-9 — met
`V(A∪B)+V(A∩B)=V(A)+V(B)` and `V(A∖B)=V(A)−V(A∩B)` hold to **1e-9 relative on all 100,000 fuzz pairs** and on the
closed-form cases of `tests/primitives.rs` (box/cylinder, sphere cap, sphere–sphere lens, …).
The first full run exposed three violations of ~2.6e-7 (torus ∩ torus). They were not Boolean errors: the Green's-theorem
mass integral ignored a uv gap of ~1e-7 between pcurve ends of a loop (vertices merge within tolerance, pcurves are fitted
independently). The integral now bridges such gaps; the identity test was tightened from 1e-8 to 1e-9 at the same time.

### 3. SSI — met
* Labelled suite (`labelled_branch_suite`): plane/sphere/cylinder/cone/torus pairs with known branch counts
  (two-circle cylinder/cylinder, plane/torus with Villarceau and two-loop cases, tangent and coincident cases): no missed
  branches, all passing.
* 10,000 random analytic pairs (`SSI_PAIRS=10000`): worst distance of a traced curve to either surface
  **2.96e-9**, against a bound of tolerance/10 = 1e-7 for the model tolerance of 1e-6 (`kw_math::Tolerance`), and
  also below the stricter 1e-8 assert. 4 pairs returned an honest `Err`; 4 pairs were flagged by the sampling oracle as
  possibly missing a branch (a conservative check, not a confirmed miss).
* Spline surfaces: spline against plane/sphere/cylinder (`nurbs_surfaces_against_analytic_surfaces`), and **spline against
  spline** (`nurbs_surfaces_against_nurbs_surfaces`: four labelled pairs in both argument orders, every returned curve within
  1e-8 of both surfaces, and a coverage oracle that finds no near-intersection sample without a nearby curve) plus
  **300 random spline/spline pairs** (`random_nurbs_pairs_curves_are_on_both_surfaces`, `SSI_NURBS_PAIRS`): 0 failures,
  worst distance of a traced curve to either surface **8.4e-10**.
* How two non-analytic surfaces are intersected: the one with the smaller window is traced over its parameter grid against
  the signed-distance field of the other (`kw_ssi::field`: BVH over a tessellation of its window, nearest triangle, Gauss–Newton
  polish, value `(x − S(u*, v*))·n`). A traced curve is cut where it leaves the other surface's window and where the
  footpoint jumps across the seam of a closed spline, with the cut located by bisection; a contour end on the window boundary
  stays on it. Cost: ~70 ms per pair of spline patches.

### 4. Fillets — met; vertex blends and general variable radius now implemented
`KW_BLEND_PARTS=500`: **500 / 500** parts succeed (box, prism, L-prism, cylinder, cone, turned parts; fillet and chamfer;
single edges and *adjacent* edge sets), each result passes `validate()`, tessellates (mesh volume within 3 % of the exact volume) and has a
plausible positive volume change. Failures are `BlendError { edge, kind }` naming the input edge.

**Vertex blends (new).** At a vertex of three planar faces whose three straight edges are all filleted with the same radius
and the same convexity, the three edge tools (which run through the vertex and would meet in a mitre) are followed by a
corner tool: the hexahedron bounded by the three faces (pushed out by a margin) and the three planes through the ball
centre perpendicular to the edges, minus the ball. The ball touches the three faces at once; the result has a sphere patch
bounded by the three great-circle arcs where it meets the fillet cylinders tangentially. Checks:
* box with all 12 edges filleted: volume equals Steiner's formula for the parallel body of the inner box to **1e-9** and the
  body has 6 planes + 12 cylinders + 8 spheres (`tests/vertex.rs`);
* `tests/corners.rs`, 600 random parts (box, oblique prism, pentagon prism, box with a notch for concave corners), either one
  corner or every edge: **590 / 600 (98.3 %)** valid and volume-consistent, 2,537 sphere patches built. The 10 failures are all-edge
  fillets at 0.2–0.3 of the shortest edge on prisms (the blends eat whole faces) and are `Err`, never invalid bodies.
Not covered: vertices of more than three faces, non-planar faces at the vertex, corners whose three edges have different radii
or mixed convexity, and chamfer corners — these still meet in a mitre.

**Variable radius (extended).** `fillet_law(rep, body, edge, &RadiusLaw)` with `RadiusLaw::Linear`, `Table` (monotone cubic
through (t, r) pairs) or `Function`:
* straight edge between two planes: removed volume equals `(1 − π/4)·L·∫r(t)² dt` for a right angle (sine law and a 4-point table,
  relative error < 2e-6, which is the skinning error of the lofted tool; the linear law stays exact at 1e-9);
* straight edge between a plane and a cylinder parallel to the edge (a circular segment prism): constant radius matches the
  analytic area of the section to 1e-8, variable radius matches the integrated area to 1e-6;
* closed circular rim between coaxial faces, periodic law `r(0) = r(1)`: cylinder rim, cone rim and a concave collar
  (plane against cylinder) match the Pappus integral of the section to 1e-6 (`tests/variable.rs`). The tool is a closed
  periodic loft (`kw_model::loft_ring`, 256 stations, solid of genus one) so that contact curves lie on the coaxial faces to
  within the tolerance of the tangent-contact detection.
The blend section for a curved face is the fillet circle tangent to the *normal section* of the faces (exact rolling ball for
constant radius, rim edges and edges between translation-invariant faces; with a varying radius on a straight edge it is
the section-plane fillet, which is tangent to both faces along the contact curves but not the exact envelope of the rolling ball).
Not covered: curved edges that are not circular rims (e.g. ellipse edges, spline edges), sphere/torus/spline faces, setback.
### 5. STEP / IGES — partly met
**Geometry and validity.**
* Kerfwright → external, 136 exported bodies (primitives, extrudes, revolves, sweeps, lofts, Booleans, fillets, offsets, drafts):
  * **OpenCASCADE 8.0** (`cadquery-ocp`): all 136 read, `BRepCheck_Analyzer` valid, one solid each, volume / area / centroid
    / bounding box within **1.3e-6** (worst), face counts equal, edge and vertex counts equal in 130 files (6 sphere-with-needle-seam files
    differ by one edge in OCCT's count; faces, volume and area agree).
  * **Gmsh 4.15** (OpenCASCADE kernel through its own `occ` module): 136/136 imported, one volume each, face counts equal, surface meshes generated.
    Gmsh only exposes OCCT's default-precision mass integration (3e-4 off on swept spline bodies where OCCT with eps 1e-10 agrees to 1e-7);
    its meshes converge quadratically to the Kerfwright volume, so the mass tolerance there is 1e-3.
  * **truck-stepio 0.3** (independent Rust kernel, `scripts/oracles/truck_check`): 136/136 parse and convert to shells with the right face count (parse/topology only, no geometry comparison).
* External → Kerfwright:
  * 200 OpenCASCADE-authored solids (as generated by `occt_make_corpus.py`: primitives, prisms, revolves, Booleans, fillets, chamfers, thick solids, lofts, pipes, offsets): **200 / 200** import as valid solids; volume and area agree with OCCT to **7.9e-8**, face counts equal.
  * Public corpus: 259 STEP files sampled evenly (every k-th path) from the FreeCAD-library repository (files ≤ 600 kB, 17 MB; `scripts/oracles/fetch_public_corpus.sh`): **258 / 259 = 99.6 %** import as valid solids. All 258 agree with OCCT's own reading of the same file to volume 3.0e-7 / area 1.1e-7 (median 5e-15). The remaining file (a sprocket) fails `validate()` with orientation errors. Failure classes fixed along the way: whole-sphere faces bounded by a vertex loop, unmarked outer bounds, complex-instance B-splines, circle-profile surfaces of revolution (normal and periodicity), hyperbola/parabola edges, tolerant (approximate) edge curves, small circles on spheres.
* Round trip inside Kerfwright: 136 / 136 within 1e-8 with topology counts preserved.
* IGES: reader exercised on synthetic fixtures written by the test (natural-boundary cube, explicit boundaries with a hole, 186 solid, open shell → error, fuzzing never panics). There is **no external IGES corpus** here and no IGES export (roadmap lists import only).

**What is not met:** "round-trip with ≥ 3 external CAD tools". Obtainable here without administrator rights: OpenCASCADE (through `cadquery-ocp`), Gmsh (which embeds OpenCASCADE) and truck (independent, but parse-only). FreeCAD, Fusion, SolidWorks and similar were not available, so the three tools above are one real CAD kernel used twice plus one independent reader. A second independent kernel for the geometry comparison (e.g. FreeCAD with its own checks — also OCCT — or Parasolid/ACIS-based tools) was not possible.
Also: the public corpus is one family of parts (small mechanical/electronic library parts), not a diverse benchmark; the criterion's own corpus was not specified.

### 6. Performance — met
Measured (16 threads, release, `verify` on):

| SLA | Target | Measured |
|---|---|---|
| Boolean, 10k × 10k-face prismatic, outlines overlapping locally | 500 ms | **0.32 – 0.43 s** (union 0.43, intersect 0.32, subtract 0.34; 20,000 faces) |
| Boolean, outlines crossing everywhere (stress, not an SLA) | – | 0.35 – 0.4 s at 1,600 faces, ~1.5 s at 3,200, ~3 s at 4,000; some sizes from ~5,000 faces up return `Err` (a degenerate stress: every outline vertex lies on the other body's walls) |
| Fillet, 50 edges on a 2,002-face body | 300 ms | **~120 ms** (batched into 2 Booleans) |
| STEP import | 50 MB ≤ 5 s | **met**: 28.3 MB (445k instances, one 13,500-sided prism) imports in 1.39 s (20.4 MB/s) → ≈ 2.5 s for 50 MB by scaling; parse alone 125 MB/s |

What moved the numbers in the second pass: the classification index and every face's bounding box were rebuilt from the B-rep
for every Boolean (three passes over every pcurve, with an O(boundary) point-in-loop test per interior sample); they are now built
once from the parameter-space data the Boolean already holds, with the grid-indexed containment test. For fillets, every tool
used to recompute the bounding box of the whole body (2,000 faces) even when no end plane needed it. The first pass had
reached 0.76 – 1.03 s and ~480 ms by parallelising and indexing the phases (exact planar volumes by a 1-D boundary
integral, per-face arrangement, SSI pairs, setup, edge geometry and the classification ray queries, one-cell probes in the vertex registry).

## Honest limits
* Vertex blends exist only for a vertex of three planar faces with three straight, equally-sized, same-convexity filleted
  edges. Other vertices (4+ faces, curved faces, different radii, mixed convexity) and chamfer corners meet in a mitre;
  there is no setback and no tangent-chain propagation (each edge is blended independently, a chain of tangent edges is not
  selected automatically).
* Variable radius: straight edges between planes or a plane and a parallel cylinder, and closed circular rims between
  coaxial plane/cylinder/cone faces. A radius law on a closed edge must be periodic. With a *varying* radius the section is the
  fillet in the normal section (tangent to both faces, but not the exact envelope of the rolling ball). Other curved edges
  (ellipses, splines), sphere/torus/spline faces: `Err` naming the edge.
* Free-form Booleans: the signed-distance field makes surfaces of any kind intersectable, and the fuzz still shows ~0.1 % `Err`
  (an arrangement that keeps both sides of a cut; three of the four remaining seeds involve a closed spline extrusion, whose seam
  is a crease because the spline is closed by interpolation, not periodic). Tangent contact between free-form solids is refused.
* Tangent contact between solids is mostly refused (`Err`), succeeding only in simple tangent-line cases and for tools that touch
  a face along a boundary iso-curve (blend tools).
* Shell/offset/draft use the topology-preserving rebuild: they fail with an error on self-intersecting offsets rather than repairing them.
* The 100,000-pair corpus is generated by this repository; ground truth for "valid-expected" is the generator, not another kernel.
* "Round-trip with ≥ 3 external CAD tools" is not fully met (see criterion 5).

## Dev-only oracle set-up (never linked into the shipped crates)
```
python3 -m venv .venv-oracle && .venv-oracle/bin/pip install cadquery-ocp gmsh numpy   # ~110 MB
# gmsh needs libGLU.so.1 (apt: libglu1-mesa)
cargo build --release -p kw-step --example step_tool
KW_CORPUS_PER_KIND=8 target/release/examples/step_tool export /tmp/ours         # Kerfwright's own STEP files + manifest
python3 scripts/oracles/self_roundtrip.py /tmp/ours                              # re-import by Kerfwright
.venv-oracle/bin/python scripts/oracles/occt_check_ours.py /tmp/ours             # OpenCASCADE
.venv-oracle/bin/python scripts/oracles/gmsh_check.py /tmp/ours                  # Gmsh
(cd scripts/oracles/truck_check && cargo run --release -- /tmp/ours)             # truck (own Cargo workspace)
.venv-oracle/bin/python scripts/oracles/occt_make_corpus.py /tmp/occt            # OpenCASCADE writes 200 solids
python3 scripts/oracles/import_corpus.py /tmp/occt                               # Kerfwright reads them
scripts/oracles/fetch_public_corpus.sh /tmp/public                               # ~17 MB from GitHub (FreeCAD-library)
python3 scripts/oracles/import_corpus.py /tmp/public
.venv-oracle/bin/python scripts/oracles/occt_compare_import.py /tmp/public       # compare with OCCT per file
```
Set `ORACLE_PY=.venv-oracle/bin/python` (and `ORACLE_CORPUS=/tmp/public`) for `scripts/phase3_acceptance.sh` to run these.

## Reproduce
```
scripts/phase3_acceptance.sh                       # everything (about 40 minutes)
BOOL_PAIRS=2000 SSI_PAIRS=2000 BLEND_PARTS=100 scripts/phase3_acceptance.sh   # quick pass
```
