# Kerfwright: everything the program does today

A pure-Rust CAD suite written independently from public sources. Everything below is implemented and covered by tests unless marked *(limit)*. Real mouse interaction and native file dialogs could not be driven where it was built, so they need a hand test.

## 1. Application shell
* Welcome page: New design (name + folder, saved at once as `<name>.kwf`), Open project, Open demo part, ten most recent projects (missing files marked, "Forget").
* Application frame: light theme, application bar (home, File, Undo, Redo, Save, project name with `*` when unsaved), workspace menu (DESIGN, MANUFACTURE, SIMULATION, ELECTRONICS, RENDER), ribbon with tabs, browser, timeline, status line.
* Unsaved-changes prompts when closing, opening another project, opening the demo or quitting; the window closes after the answer.
* `.kwf` project file: feature document plus CAM, FEA, board and assembly data as extra entries; unknown fields and entries survive a save; three schema versions migrate; files from newer versions are read, files that need a newer reader are refused cleanly.
* File ▸ Import STEP / IGES / STL (kept inside the project), Export STEP / STL / OBJ (+MTL) / GLB.
* Command-line: `--demo`, `--workspace`, `--screenshot`, `--design-demo`, `--open-menu`, per-workspace demos, `--exit-after-frames`.

## 2. Ribbon and interface widgets
* Icon buttons in groups, each with a `TITLE ▾` dropdown listing every command with icon, name, shortcut and category headings; split buttons for variants; disabled commands explain why; the ribbon scrolls sideways in a narrow window.
* Tabs: SOLID (CREATE, MODIFY, CONSTRUCT, INSPECT, ASSEMBLE, SELECT), SURFACE (CREATE, MODIFY, INSPECT), SHEET METAL, FORM, TOOLS, and SKETCH while sketching (CREATE, MODIFY, PROJECT, CONSTRAINTS, OPTIONS, FINISH). MESH is not shown.
* Painter-drawn icon set (no image files).
* Orientation widget: bevelled cube with clickable faces, edges and corners, turn arrows (quarter turn, 45° up/down), menu (Isometric, Zoom to fit, Previous view, Shaded/wireframe, Show edges, Section view).
* Navigation: orbit, pan, zoom, home, fit, shaded/wireframe, edges on/off, origin planes; clip planes follow the model so panning never cuts it.
* Browser tree: Document Settings, Origin (planes with eye toggles), Sketches (eye), Construction (eye), Bodies (eye, rename, colour, material), Features; context menus (Edit, Suppress, Roll back, Delete with a count of dependent steps).
* Timeline: a chip per feature, drag to reorder (refused with a message when something depends on it), rollback marker, suppress, edit in place, undo / redo.
* Delete takes the step and everything built on it, as one undo step.
* Parameters window: named values and expressions usable anywhere.
* Display options: seams of round faces are not drawn as edges; sketches used by a feature are hidden afterwards (eye brings them back); construction lines of a sketch are not drawn in 3-D.

## 3. Sketching (2-D, solver-driven)
**Drawing tools:** line, rectangle (corner, centre, 3-point/aligned), circle (centre, 3-point), arc (centre, 3-point), spline (control points), fit spline, polygon (n sides), slot, ellipse, point, text (letters from fonts as profiles).
**Modify tools:** trim, extend, offset, corner fillet, corner chamfer, mirror, rectangular pattern, circular pattern, Project (model edges, locked, purple), Intersect (where the plane cuts the model).
**Constraints (13):** coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, fix, midpoint, concentric, collinear, point on curve, symmetric.
**Dimensions:** one line = length; two parallel lines = distance; two meeting lines = angle; point + line = perpendicular distance; two points = distance; circle = diameter; arc = radius. Driven (reference) dimensions in grey brackets when the sketch is already determined; names `d1, d2 …`; formulas over document parameters (live) and over other dimensions (evaluated once); double-click a label to edit.
**Snapping and inference:** points, origin, midpoints, intersections, on-curve, tracking lines (horizontal/vertical/polar angle steps/parallel/perpendicular/alignment with rested-on points), grid, auto-constraints, Alt to ignore.
**Visual language:** blue under-constrained, black fully constrained, red conflicting, yellow redundant, purple projected; closed regions shaded light blue (per region, holes clear); constraint icons beside geometry; Display toggles (grid, profiles, constraints, dimensions); drag-select box (window left→right, crossing right→left, Shift adds).
**Solver:** constraint solver with degrees-of-freedom report, redundancy and conflict detection, drag of free geometry.
**Regions:** every curve is split where it crosses or touches another (lines, arcs, circles, ellipses, splines); every enclosed area is a region (a line across a circle, crossing bars, overlaps, rings with holes).
**Other:** sketch on any origin plane or planar face, DXF import / export from the sketch palette, undo / redo, reference outline of the face being sketched on, standalone sketcher binary.

## 4. Solid modelling (feature timeline)
**Create:** Extrude, Revolve, Sweep, Loft, Pipe, Rib, Hole, Thread, Rectangular pattern, Circular pattern, Mirror.
* **Extrude:** distance, symmetric, flip, through all, second side, taper, to a face, start offset, thin wall (inside / outside / centred), New Body / Join / Cut / Intersect, live preview, drag arrow, region picking (click or tick regions; Default / All / None; neighbours merge into one profile).
* **Revolve:** about the sketch X or Y axis, a clicked body edge, or a construction axis.
* **Sweep:** arc or spline path, or a single straight line. **Loft** between profiles. **Pipe** along a path (outer diameter + wall). **Rib** from open lines and arcs.
* **Hole:** simple, counterbore, countersink, tapped (ISO metric coarse/fine, UNC, UNF tables with tap drills), through all; **Thread** on a hole wall or pin, cosmetic or modelled (helical groove cut into the body, right- or left-hand).
* Patterns of any feature; mirror about origin planes, construction planes or planar faces.

**Modify:** Fillet, Variable fillet, Chamfer, Shell, Draft, Offset body, Combine (Join / Cut / Intersect, keep tools), Move / Copy body, Scale (uniform), Split Body, Change Parameters.
**Construct:** planes (offset/tilted, midplane, through 3 points), axes (2 points, edge, normal to face, two planes), points (coordinates, edge middle, axis ∩ plane); drawn in the view with eye toggles; used by Revolve, patterns, Mirror, Section, joints.
**Bodies:** multi-body documents, active body, per-body name, colour and material (Appearance), body list.
**Inspect:** Measure (face area/kind, edge length/radius, distances and angles between picks, volume, area, mass in 10 materials, centre of mass, bounding box, inertia, threads), Interference (overlap volumes), Section Analysis (clip plane with cap; origin or construction plane, offset, flip).
**Import:** STEP / IGES solids as a feature; STL as a mesh body (units, up axis, welded and wound outward) with *Convert to solid* into a faceted B-rep (coplanar triangles merged into faces).
**Persistent naming:** faces keep their names through edits and later features; references survive rollback and reorder.

## 4a. Surfaces and analysis
**Surface bodies** (sheets) have area but no volume; Booleans, fillet, shell and the other solid features refuse them. Open edges are drawn in magenta; Measure shows area, free edges and bounding box; the browser marks them *(surface)*.
**SURFACE tab, CREATE:** Extrude, Revolve, Loft (smooth or ruled), Sweep (twist, guide rail) of open or closed sketch curves (also *Output: Surface* in the solid dialogs); **Patch** (three or more picked edges; four give one Coons patch, optionally tangent to the surface an open edge belongs to; more or fewer split about the centre); **Ruled Surface** between two edges.
**MODIFY:** **Trim** (plane or a picked face, keep either side), **Extend** (strip past an open edge, surface carried on), **Match** (G1 / G2 of a NURBS patch to a neighbouring face along an open edge), **Stitch** (surfaces whose open edges coincide within a tolerance; a closed group becomes a solid), **Unstitch** (a body into its faces), **Thicken** (normal / reverse / both sides, walls along the open edges), **Offset Surface**.
**INSPECT:** **Zebra** (reflection stripes, axis and density), **Curvature** (Gaussian, mean, maximum, minimum; range from the model or set by hand; colour bar), **Draft** (pull direction, limit angle), **Edge Continuity** (each edge shared by two faces coloured gap / G0 / G1 / G2, tolerance slider, counts and the worst joins). Analysis shows unlit on every body, solid or surface, and keeps the section cap.
**Features** `Patch`, `Ruled`, `Stitch`, `Unstitch`, `Thicken`, `OffsetSurface`, `Trim`, `Extend`, `Match` and the `surface` flag of Extrude / Revolve / Loft / Sweep are in the timeline with persistent face names and in the `.kwf` (older files read unchanged). Details, sources and limits: `docs/surface-report.md`.

## 5. Assemblies (Design workspace)
* **Components inside the project:** New (empty) Component, Component from Body (copies the feature chain that builds the body), Component from Design, Insert from a `.kwf`; activate a component to edit it in place and finish it; copies share a part; sub-assemblies (group components, their joints and groups go with them); stored in the project file (`assembly.cbor`; files of the first version still open).
* **Joints:** Rigid, Revolute, Slider, Cylindrical, Planar, **Ball**; made by clicking a point on each of two components (snap to face centre, corners, edge middles, circle centres, cylinder axes, sphere centres) with Flip, offset, angle, motion limits and a live preview; **As-Built Joint** (components keep their positions); values may be formulas over named parameters; limits are enforced and editable; joint properties window.
* **Grounding and free components:** several grounded components, free components stay where they are, **Rigid Groups**, releasing a ground or deleting a joint leaves components in place; a joint between two grounded components or inside a rigid group is refused with the reason.
* **Browser tree:** components (eye, grounded badge, sub-assemblies), joints, rigid groups; selection (Shift adds), rename, copy, isolate, delete with joints.
* **In the view:** pick components, **drag** them (free: in the plane facing the camera; on a joint: drives it within its limits; rigid groups move as one), numeric Move, Drive Joints sliders, joint origin markers.
* **Inspect:** interference with the overlap outlined (optionally re-checked after every change), **Bill of Materials** (part, quantity, material, volume, mass, size; CSV), mass and centre, STEP export of the posed assembly.
* Undo / Redo of assembly changes (in the assembly view).

## 6. Sheet metal
L, U, Z and custom strips (thickness, K-factor, bend radius, width, flats and bends), edit strip, flat pattern with bend table and allowances, DXF export, refold, flat pattern of a recognised folded body.

## 7. Form (subdivision)
Box, cylinder, sphere, torus cages; the cage is drawn in the view and edited there: click or box-select vertices, edges, faces; drag them in the camera plane or locked to an axis or plane (X / Y / Z keys), with grid snap and mirror symmetry; sharp / smooth per edge; live preview of the smooth surface while dragging; Ctrl+Z inside the dialog; numbers still work; scale and tolerance; finished into a body of patches on OK. *(limits: no pyramid, no adding / removing cage faces, smooth cages are slow to finish)*

## 8. TOOLS tab
Measure, Section Analysis, Appearance, Parameters, Import, Export.

## 9. Manufacture (CAM)
Pocket, 2D Contour, 3D finishing (exact drop-cutter with shank/holder collision checks), stock simulation (dexel), multi-axis kinematics (5 machine types, 3+2, swarf, tool-centre-point), five G-code post dialects with a post template language, G-code replay interpreters for all five.

## 10. Simulation (FEA)
Tetrahedral meshing of solids, linear static analysis (second-order elements), modal analysis, topology optimisation (SIMP), supports and loads on picked faces, von Mises / displacement colouring, validated against closed-form solutions.

## 11. Electronics (ECAD)
Board model, placement (simulated annealing), router (A* with rip-up), design-rule checks, copper pour, Gerber fabrication files, fabrication checks, STEP of the board, IDF, pick-and-place files.

## 12. Render
Realtime PBR viewport (ambient occlusion, edge lines, picking buffer), CPU/GPU path tracer with denoiser, save PNG.

## 12a. Drawings (2-D sheets)
**DRAWING** workspace. *Sheet:* New Drawing from the design or the assembly (ISO or ASME, A0-A4 / ANSI A-E, third or first angle, title block); more sheets; sheet tabs with an orange dot when the model has changed. *Views:* Base (Front, Back, Top, Bottom, Left, Right, Isometric) with visible / visible+hidden / wireframe / shaded styles at the largest standard scale that fits; Projected views on any side, kept aligned and moved with their parent; Section views (click the view, then the cutting line; hatched cut faces; flip); Detail views (centre and radius, enlarged, with circle and letter). Hidden lines are removed from the tessellation with exact lines, circles, arcs and ellipses kept where possible; tangent edges and seams are not drawn. *Annotation:* dimensions (linear, aligned, diameter, radius, angle) picked on the model's points and edges, with tolerance (symmetric, bilateral, limits), text before / after and decimals; centre marks and centre lines; notes with leaders; datum, surface-finish and feature control frame symbols (14 GD&T characteristics). *Tables:* parts list (choose columns) and balloons numbered by the list, CSV export. *Associativity:* each view remembers the model it was made from; **Update** makes views, dimensions, parts list and balloons again; a dimension whose face has gone turns red and keeps its last value. *Output:* PDF (one page per sheet) and DXF (one sheet, layers with line types and weights). Undo / Redo cover every drawing change; everything is stored in the `.kwf`.

## 13. Geometry kernel and libraries (crates)
* **kw-math / kw-nurbs:** transforms, frames, NURBS curves and surfaces, fitting, analytic surfaces, intersection helpers, uniform scaling of all surface and curve kinds.
* **kw-brep:** B-rep topology, validation, mass properties, copy with transform/scale.
* **kw-ssi / kw-boolean:** surface–surface intersection and Booleans (analytic and free-form), healing, face merging.
* **kw-model:** extrude, revolve, sweep, loft, shell, offset, draft; surfaces (sheet builders, Coons patches, ruled surfaces, stitch / unstitch, offset, thicken, trim, extend, match). **kw-analysis:** curvature, zebra, draft and edge continuity. kw-brep also holds the sewing (`sew`); kw-nurbs surface interpolation, collocation and extension. **kw-blend:** fillet, chamfer, variable radius, tangent chains, vertex blends.
* **kw-tess:** tessellation. **kw-step / kw-iges / kw-io / kw-dxf:** file formats (kw-dxf also writes text, line types and layer weights). **kw-drawing:** sheets, projection and hidden lines, sections, details, dimensions, annotation, parts lists, PDF and DXF writers.
* **kw-sketch:** sketch model, constraint solver, planar arrangement, curves, text. **kw-expr:** unit-aware expression language. **kw-feature:** the parametric engine (timeline, evaluation, naming, parameters, sheet-metal and Form features, regions). **kw-script:** scripting. **kw-store:** extra project data. **kw-assembly, kw-sheet, kw-subd:** engines behind the three UIs above.
* **kw-cam / kw-gsim / kw-fem / kw-mesh3d / kw-ecad / kw-fabcheck / kw-render / kw-job:** the manufacturing, simulation, electronics and rendering engines.

## 14. Known limits
assembly joints form a tree (no closed kinematic loops) and components are copies in the project (no linked files); no Mesh tab (surfaces are in `docs/surface-report.md`); Form cages cannot get new faces or edges (only their vertices and creases move); ellipse/spline crossings that only graze can be missed when splitting sketches; chosen regions that touch only at a corner become separate pieces of one body; CAM and FEA use the last solid only; STL meshes convert to faceted solids only (no cylinder / sphere fitting, 150 000 triangle cap, FEA and CAM need the converted solid); drawing limits are listed in `docs/reference-workflow-comparison.md`, no collaboration.
