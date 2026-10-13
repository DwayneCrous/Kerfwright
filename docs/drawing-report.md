# Drawings: what was built, how it was checked, and what is left

Added after Phase 5. This is our own design for 2-D production drawings from the 3-D model; Fusion 360 was used only as a user to
learn the expected workflow (see `CLEAN_ROOM.md`). Labels as in the other documents: [Public] standard or paper, [Design] our own.

## 1. What the workflow covers

| Step of the workflow | Where | Checked by |
|---|---|---|
| New drawing from the design or the assembly (sheet size, ISO / ASME, units, projection angle, title block) | `kw-drawing::sheet`, `kw-app::drawing` | frame inside paper for all 10 sizes x 2 standards; title block carries every field |
| Base view, projected views, scale, style | `projection`, `hlr`, `update` | analytic solids: box 4 / 9 + 3 hidden edges, cylinder lines and circles, plate with hole, stadium (tangent edges absent), body behind body; third / first angle |
| Section and detail views | `section`, `detail`, `render` | plate with a hole cut through the hole: hatch length x spacing = cut area (+-4 %), hole stays open, loops from the slice, removed half not drawn, detail clipped to its circle |
| Dimensions and annotation | `dim`, `annot`, `snap` | value = model value at any scale; layout geometry; tolerances; hole counts; every GD&T symbol inside its cell |
| Parts list and balloons | `bom`, `pick`, `kw-app::drawing::parts` | rows equal `Pose::bom` field by field, CSV equal to the assembly's; balloon item = row number, follows a moved component |
| Live associativity | `update`, `kw-app::drawing` | a hole 6 -> 10 mm: badge, then Update changes the view, the dimension (6 -> 10) and the BOM volume; undo of the model change makes it stale again; opening a saved drawing against a changed model is stale |
| PDF / DXF | `pdf`, `dxf_out`, `kw-dxf` | xref offsets, stream lengths, byte-identical output, read by poppler and ghostscript; DXF read by the independent `dxf` crate |

Tests: `kw-drawing` 48 (8 unit, 9 HLR, 8 output, 6 section, 12 dimension, 5 parts list), `kw-dxf` 16, `kw-store` 2, and 30 Drawing tests among the
104 in `kw-app/tests/shell.rs` (headless egui frames, synthetic pointer events, scripted file dialogs).

## 2. Methods and sources
* **Hidden lines** [Design]: every body is tessellated, triangles projected and indexed on a grid; edges are sampled along their exact curves and tested against
  nearer triangles; transitions are found by bisection. A point on its own face's boundary is not hidden by that face. Cylinder silhouettes are placed analytically,
  other curved faces from the mesh (facets turning from facing to facing away). Tangent edges (normals within 1.5 degrees) and periodic seams are not drawn.
* **Sections** [Design]: the cut outline is the tessellation sliced by the plane (bit-identical crossings chained into loops), the removed half is clipped away, the cut is an
  occluder and hatched by scan lines under the even-odd rule. The kernel's Boolean was not used: it fails on a face with an inner loop (also true of Split Body).
* **Drawing conventions** [Public]: ISO 128 line types and the 2:1 thick / thin ratio, ISO 129-1 and ASME Y14.5 dimension layout, ISO 5457 / ASME Y14.1 sheet layout,
  ISO 5456-2 / 128-30 projection angle, ISO 7573 parts-list position, ISO 1101 / ASME Y14.5 characteristic symbols (drawn by us), ISO 1302 surface-finish symbol.
* **Files** [Public]: PDF 1.4 (ISO 32000-1) written by hand with the built-in Helvetica font and Adobe's published AFM widths; DXF R2000 (Autodesk's published format description).
* **Associativity** [Design]: a dimension stores the two face names (kw-feature `Reference`) that meet at its edge and which point of the edge; Update resolves them again and
  never guesses (ambiguous or missing is reported).

## 3. Qualifications and known gaps
1. Section cuts are straight lines; curved cut edges are polylines at about 1e-4 of the part size, so the cut face is a little larger than the exact one around a hole.
2. No detail of a section, no dimensions on section or detail views, no break / auxiliary / broken-out views, no ordinate or baseline dimensions.
3. In an **assembly** drawing dimensions are fixed in space: they are measured but do not follow the model; balloons do follow it.
4. GD&T symbols are placed, not validated. Notes use one font size per note. Text is the built-in Helvetica (WinAnsi), so other scripts print as `?`.
5. No automatic balloons; one parts list per sheet; the parts list of a sheet comes from its first view's source.
6. Update regenerates every view (not only the stale ones). Hidden-line removal of very large meshes has not been timed.
7. A drawing of a single component of an assembly (`ViewSource::Component`) is not offered.
8. Real-mouse interaction and native file dialogs could not be driven where this was built; the logic behind each click is tested, a hand test is due.

## 4. Reproducing
```
cargo test --release -p kw-drawing -p kw-dxf -p kw-store -p kw-app
cargo clippy -p kw-drawing -p kw-dxf -p kw-store -p kw-app --all-targets -- -D warnings
target/release/kerfwright --drawing-demo --screenshot out.png      # or scripts/app_acceptance.sh
```
