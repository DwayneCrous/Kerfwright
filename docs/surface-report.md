# Surfaces and continuity analysis: what was built, how it was checked, and what is left

Added after the drawings. This is our own design for explicit surface bodies (sheets), the tools that make, join and reshape them,
and the diagnostics that show how good they are. Fusion 360 was used only as a user to learn the expected workflow (see
`CLEAN_ROOM.md`). Labels as in the other documents: [Public] standard or paper, [Design] our own.

## 1. What it covers

| Step | Where | Checked by |
|---|---|---|
| **Analysis**: zebra stripes, Gaussian / mean / principal curvature maps, draft angle, with colour bar | `kw-analysis::{zebra,curvature,draft,ramp}`, shader `mesh.wgsl`, `kw-app::design::analysis` | sphere K = 1/r², H = 1/r, cylinder K = 0, H = 1/2r, plane 0, torus outer equator, an exact NURBS paraboloid, a hole's sign; draft angle of a tilted face; ramp end points; four GPU pixel tests (a scalar ramp shows exactly the CPU colour, zebra stripes break up a sphere and barely touch a flat face, draft colours walls and floors, the section cap stays orange) |
| **Sheet bodies**: a surface is a body with no volume; open edges drawn in magenta; Measure gives area, free edges, box | `kw-brep` (`BodyKind::Sheet`, `free_edges`, `validate`), `kw-app::design::{measure,overlay}` | plane sheet valid, area exact, volume refused, 4 free edges; a sheet next to a solid does not loosen the solid's checks; solid-only features refuse a sheet |
| **Surfaces from sketches**: Extrude, Revolve, Loft (smooth or ruled), Sweep with **twist** and **guide rail**, as an *Output: Surface* of the solid dialogs and in the SURFACE tab; open and closed curves | `kw-model::{extrude_sheet,revolve_sheet,loft_sheet,sweep_sheet}`, `kw-feature` (`surface` flag) | strip 6, tube 2(w+h)d, cylinder 2πrh, cone π(r1+r2)·slant, annulus, hemisphere 2πr², frustum, helicoid for twist, trapezoid 2h for a guide rail; `.kwf` round trip; old files read |
| **Interpolation, Patch, Ruled**: tensor-product interpolation; four-sided Coons patch (G0, or tangent to the surface beside an open edge); three, five or more sides split about the centre; ruled surface between two edges | `kw-nurbs::NurbsSurface::interpolate*`, `kw-model::{patch_sheet,ruled_sheet}` | flat hole → plane, skew quad → z = xy, two lines + two arcs → cylinder, triangle / pentagon / hexagon areas, tangent side within 1e-3 rad (G1), a cap tangent to a wall at 90°, ruled areas |
| **Stitch, Unstitch, Thicken, Offset Surface** | `kw-brep::sew`, `kw-model::sheetops` | six loose sheets → cube V = 8; tube + two caps → box 72; box, cylinder, sphere taken apart and sewn again; thicken plane 6x4x0.5 = 12, pipe π(R₂²-R₁²)h; offset of a NURBS cone equals the cone area to 1e-4 |
| **Trim, Extend, Match** | `kw-model::{trim_sheet,extend_sheet,match_boundary}` | plane, slanted plane, cylinder across its seam, tilted plane (mean height), NURBS frustum, plate by a cylinder tool (disc / plate with hole); strips on plane, tube, NURBS cone; G1 normal angle < 2e-3 rad, G2 curvature within 5 % of the neighbour's |
| **Edge continuity**: every shared edge classed gap / G0 / G1 / G2 and drawn in colour | `kw-analysis::continuity`, `kw-app::design::continuity` | box = twelve G0 at 90°, fillet = G1 with a curvature jump 1/r, seam = G2, a cap goes G0 → G1 → G2 as it is matched at each level |

Tests: about 135 new ones. `kw-analysis` 13 (8 + 5), `kw-viewer` 4 (GPU), `kw-nurbs` 6, `kw-brep` 9 (4 sheet + 5 sewing), `kw-tess` 1, `kw-model` 51 (17 surfaces from
sketches, 9 patch, 9 stitch / offset / thicken, 8 trim, 5 extend, 3 match), `kw-feature` 35 (5 wires, 10 surface features, 7 patch, 6 stitch, 6 trim, 1 guard), and
15 in `kw-app/tests/shell.rs` (headless egui frames, synthetic pointer events), among them a full workflow: tube → Patch cap → Match → Stitch → Edge Continuity.

## 2. Methods and sources
* **Curvature** [Public]: the second fundamental form and the Weingarten map (do Carmo, *Differential Geometry of Curves and Surfaces*); NURBS use the exact derivatives
  (Piegl & Tiller, *The NURBS Book*, ch. 3-4), other surfaces the shape operator from differences of their exact normals. Sign follows the outward normal.
* **Zebra and draft** [Public / Design]: reflection-line analysis (Hagen et al., *Surface interrogation algorithms*, IEEE CG&A 12(5), 1992); the stripes are computed per
  fragment in the shader from the reflected view ray, the draft angle is asin(n · pull). Curvature is computed per vertex on the CPU and uploaded as a second vertex buffer.
* **Sheet bodies** [Design]: a surface is a body of kind `Sheet` whose edges may have one use; `validate` checks closedness per body.
* **Guided sweep** [Design]: where the guide rail meets the plane of each section (found by sampling and bisection) it sets the section's direction and, relative to the
  start, its scale; a twist is spread evenly along the path. Sections are interpolated as in the solid sweep, with the same tolerance test against the exact sweep.
* **Coons patch** [Public]: S. A. Coons, *Surfaces for computer-aided design of space forms*, MIT TR-41, 1967; G. Farin, *Curves and Surfaces for CAGD*, the chapter on
  Coons patches. Boundary curves are re-parametrised by arc length. [Design] A side that borders a surface gets a smooth correction of the cross derivative (the neighbour's
  outward direction, as long as 0.6 of the chord to the opposite side), faded out near the ends of the side. The patch is evaluated on a grid and interpolated by a bicubic
  surface with even parameters (so that patches sharing a boundary share its curve), the grid refined until it follows the blend to a millionth of the size. N-sided
  boundaries are split about their centre into one four-sided patch per corner.
* **Interpolation** [Public]: Piegl & Tiller, algorithms A9.1 / A9.4 (global interpolation with chord-length parameters and averaged knots). Collocation at the Greville
  abscissae on a given knot vector [Design] keeps the parameters of the function fitted; it is used for offsets and for the new edges of strips.
* **Sewing** [Design]: faces are described as a soup of surfaces, loops of edge uses and edges; open edges that lie on one another within a tolerance (end points and three
  interior points) are merged when each fits exactly one other; faces are turned to agree by propagation over shared edges (two uses of an edge must run opposite ways); a
  group whose edges are all used twice becomes a solid, turned to positive volume, else a sheet. Pcurves are kept where the edge's parameters are unchanged and fitted otherwise.
* **Offset** [Public / Design]: analytic surfaces move exactly (kw-nurbs `offset_surface`); a NURBS surface is refitted by collocation on its own knots so that the parameters
  stay, with the deviation checked between the nodes (cf. Maekawa, *An overview of offset curves and surfaces*, Computer-Aided Design 31(3), 1999, on why exact offsets of
  free-form surfaces do not exist in closed form). Thicken adds the walls along the open edges and sews.
* **Trim** [Design]: the edges are cut where they cross the tool (sampled and bisected on the exact curves); each face is cut along the curve `kw_ssi::intersect` gives, in
  the face's own parameter space; the loops of the kept region are traced through the cut points (boundary pieces on the kept side, then the cut arc, then the next boundary
  piece). Periodic surfaces are handled by shifting the cut curve by whole periods until it lies in the face.
* **Extend** [Design / Public]: a NURBS surface is continued with its own last polynomial piece: the Bezier points of the same polynomial over the longer interval come from
  blossoming (Ramshaw, *Blossoms are polar forms*, CAGD 6(4), 1989), so the old part is unchanged, with the same parameters. The strip is a new face joined along the edge.
* **Match** [Public / Design]: the end derivatives of a B-spline in terms of its first control points (Piegl & Tiller, eqs. 3.7-3.8); the second row of control points sets the
  cross derivative (G1), the third the second derivative (G2); the new rows are found by collocation along the edge, after refining its knots twice so that the change can fade out
  near the ends of the edge without ringing.
* **Continuity** [Design]: along the middle three fifths of an edge, the gap, the angle between the face normals and the difference of the normal curvatures across the edge,
  taken along the in-face direction perpendicular to the edge (the same for both faces, so the sign of the direction does not matter).

## 3. Qualifications and known gaps
1. **Sheet bodies are bodies of their own.** Booleans, fillets, shells and other solid features refuse a surface (with a message); a surface can only be joined to a solid by
   stitching it closed or thickening it. The kernel Boolean was not extended to sheets.
2. **Surface extrude, revolve, loft and sweep** take no taper, thin wall, second side or end condition; they always make a new body. Loft has no per-section twist (twist is on Sweep).
   A guide rail must cross the plane of every section.
3. **Patch.** Tangency only for boundaries of exactly four edges, and only along open edges of a surface (an edge between two faces has two sides); it fades near the corners, where
   the neighbours' tangent planes conflict (G1 holds over the middle half of a side, to about 1e-3 rad). Boundaries of other sizes are G0 across the pieces they are split into, and
   their outline has two edges per original edge (cut at the middle). A patch around a hole in a solid is not offered: the boundary must be edges you can pick.
4. **Stitch.** Refuses solids; an edge that fits more than one other is left open (reported); the faces must form one group, else the operation fails and says how many groups.
   Sewing was written for existing bodies and is **not shared with the IGES importer**, which keeps its own code.
5. **Unstitch** always splits every face (no choosing a subset).
6. **Offset Surface and Thicken** need a smooth surface: faces that meet at a crease (normals differing by more than 0.3 degrees along an edge) are refused, and a closed
   surface cannot be thickened. NURBS offsets are refits, accurate to 1e-7 of the size, not exact; an offset larger than a radius of curvature fails. Walls are fitted through
   129 samples (about 1e-7). Only planes, cylinders, cones, spheres, tori and NURBS surfaces can be offset.
7. **Trim.** The tool is a plane or the surface of one face, carried on without limit; the kept side is chosen by the tool's normal, not by clicking the part to remove. A cut
   that passes through a vertex or is tangent to an edge is refused; a cut curve that ends inside a face is refused; a closed cut that contains another loop is not supported. A cut
   across a seam is stored as two arcs.
8. **Extend.** Only edges that run along a parameter line of their surface (edges of lofts, sweeps, patches, extrusions and revolutions; not those left by Trim). Analytic: planes,
   cylinders and cones along the axis, extrusions; spheres, tori and revolved surfaces cannot be extended. Only the natural continuation (no straight / tangent mode); extending
   two neighbouring edges leaves the corner gap open; a rational surface fails if its weight would turn negative.
9. **Match.** Only non-rational NURBS patches with at least five rows of control points across the edge (what Patch and Offset make; a lofted circle is rational and is refused);
   only a whole boundary of the patch; the match is exact in the middle of the edge and fades at its ends. The match fails (message) if it cannot bring the tangent plane within 1.1
   degrees. G2 was measured within 5 % of a cylinder's curvature, not exactly. Curvature (G3) continuity and control-point editing are out of scope: this is not a Class-A tool.
10. **Continuity display.** Only edges shared by two faces have a class; separate bodies that were never stitched have none. A stitch refuses to build over a gap larger than its
    tolerance, so the *Gap* class appears only for bodies that were edited afterwards. The coloured edges are drawn over the view with no depth test.
11. **Analysis.** Curvature is per vertex of the tessellation (coarse meshes give coarse maps); zebra and draft are per fragment. The curvature range is the 2nd to 98th
    percentile of the model unless set by hand.
12. **The SURFACE tab has no Mesh counterpart**: there are no mesh bodies, no reduction or repair.
13. Real-mouse interaction and native file dialogs could not be driven where this was built; the logic behind each click is tested through the same code paths, a hand test is due.

## 4. Reproducing
```
scripts/surface_acceptance.sh                      # fmt, clippy, the surface tests and a screenshot of each stage
cargo test --release -p kw-nurbs -p kw-brep -p kw-model -p kw-feature -p kw-analysis -p kw-app -p kw-viewer
target/release/kerfwright --design-demo continuity --screenshot out.png   # also: surface, surface-zebra, patch, patch-zebra, thicken, trim, zebra, curvature, draft
```
