---
title: The modelling kernel
description: "B-rep topology, surfaces, intersection, Booleans and blends, and how failures are reported."
sidebar:
  order: 4
---

The kernel is the part of Kerfwright that knows what a solid is. Everything above it asks it questions (what is the union of these two bodies, what is the volume of this one) and expects either a valid answer or an honest error.

## Representation

A solid is a **boundary representation**: faces bounded by edges bounded by vertices. Kerfwright uses a **coedge with radial ring** structure stored in slotmap arenas.

- A **coedge** is one use of an edge by one face loop. It links to the next and previous coedges in its loop and to the next coedge around the same edge, the *radial ring*.
- The radial ring length tells you the edge type: one coedge is a sheet boundary, two is manifold, three or more is non-manifold.
- Every entity has a typed, generational id. A stale id looks up as `None`; it can never alias a newer entity.

Geometry is separate from topology. Curves are lines, circles, ellipses, NURBS or procedural curves. Surfaces are planes, cylinders, cones, spheres, tori, NURBS, extrusions, revolutions or procedural surfaces. Exact quadrics stay exact, and NURBS use the Piegl–Tiller algorithms with derivatives to any order.

A *surface body* (a sheet) is a body whose faces have open edges and no volume. Solid-only features such as Booleans, fillet and shell refuse sheets.

## Validity is checked, not assumed

`validate()` checks reference integrity, loop closure and continuity, radial rings, manifold orientation, the Euler–Poincaré relation per shell with genus, and agreement between parametric curves, surfaces and 3-D curves. Mutation fuzzing of 23 kinds of topology damage on six primitives, a million cases each, confirms that every damage that changes the body's signature is caught.

## Intersection

Booleans need curves where two surfaces meet. `kw-ssi` has exact closed forms for plane, quadric and torus cases (coaxial and parallel cylinders included) and a certified level-set tracer for the rest. For free-form surfaces it builds a **signed-distance field** of any parametric surface, so a spline can be intersected with a spline. Traced curves carry parametric curves on both surfaces.

## Booleans

`kw-boolean` runs these stages:

1. a broad phase to find face pairs that might meet,
2. surface intersection per pair,
3. clipping and a global registry of edges and vertices,
4. an arrangement of the pieces on each face,
5. classification of each piece as inside, outside or on the other body,
6. selection for union, intersection or difference,
7. assembly into shells, merging coplanar neighbours into single faces,
8. **verification** before anything is returned.

A **heal** stage handles failed, suspect or doubtful results. It re-runs the operation at other merge tolerances and accepts a result only if it passes inclusion–exclusion against the complementary operation and an independent point-classification check. An unconfirmed "no material" result is an error, never a result.

**Every failure is an `Err`.** The kernel does not panic and does not hang. In the 100,000-pair fuzz, with three operations each, this gave 0 invalid outputs, 0 panics and 0 wrong results by an independent point check.

## Blends

`kw-blend` makes constant-radius fillets, two-distance chamfers and variable-radius fillets. Naming one edge blends its whole tangent chain (lines and circular arcs, convex and concave, open or closed). Fillets are realised as Booleans with a swept tool, so they inherit the verification above.

**Limits:** corners meet in a mitre. A rolling-ball vertex blend for three-face corners exists in the source behind the off-by-default `vertex-blend` feature, withheld until its patent search is recorded. Variable radius covers straight edges and circular rims, not ellipse or spline edges, and not sphere, torus or spline faces. Setback is not done.

## Exchange

STEP (AP203, AP214, AP242) is read and written by an in-house ISO 10303-21 parser that has limits and does not panic on garbage. IGES 5.3 is read (trimmed surfaces are sewn into solids). On the 259-file public STEP corpus, 259 import as valid solids. Round trips were compared with OpenCASCADE, truck and Gmsh.

## Related

- [Benchmark results](/docs/benchmarks/results/) for the Boolean and fillet timing.
- [Status and limits](/docs/project/status-and-limits/)
