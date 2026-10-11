# Kerfwright vs. Fusion 360 (Design / Solid): where we are

Status after the "daily-use essentials" round. **Done** = in the engine, in the UI and covered by tests against closed forms.
The rest of the list is what is still missing; it is ordered roughly by how often people need it.

## Done
| Area | What works | Tests |
|---|---|---|
| **File exchange** | File ▸ Import STEP / IGES (kept inside the `.kwf` as an `Import` feature), Export STEP / STL / OBJ (+ MTL) / GLB of the model's solid | STEP round trip 1e-6, STL volume from the triangles, OBJ/GLB structure, junk files refused |
| **Sketch tools** | Line, rectangle, circle, arc, spline, **fit spline**, point, **polygon (n sides)**, **slot**, **ellipse**, **3-point arc / circle**, trim, extend, offset, **corner fillet / chamfer**, **mirror**, dimensions, 13 constraints, snapping with inference lines | `kw-sketch/tests/{shapes,snaps}.rs`: DOF, redundancy, geometry vs formulas |
| **Profiles** | Lines, arcs, circles, **ellipses and splines** all extrude (with holes) | ellipse π·a·b·h exact, parabola segment 2/3·b·h exact |
| **Extrude** | distance, symmetric, flip, **through all**, **second side**, **taper**, New / Join / Cut / Intersect, live preview | through-all cut, two-sided, frustum volume of a tapered pocket |
| **Hole** | simple / counterbore / countersink, **through all** | volume = block − π r² t |
| **More solid tools** | **Loft**, **Sweep** (arc / spline path), **Draft**, **Offset body**, **Construction plane**, **Variable fillet**, Revolve, Fillet, Chamfer, Shell, patterns, Mirror, Combine | loft = frustum formula, sweep = Pappus, draft = frustum, offset = Minkowski / mitred box |
| **Inspect** | Measure: face area / kind, edge length / radius, distance between parallel faces, angle between planes, distance and angle between edges; body volume, area, mass in 10 materials, centre of mass, bounding box, inertia | box closed forms, cylinder radius / area / rim length |
| **Workflow** | timeline (reorder, roll back, suppress, edit in place), parameters, undo / redo, browser, view cube, snap / track / grid, edge picking with hover feedback | `crates/kw-app/tests/shell.rs` (34 tests) |

A real bug found on the way: any saved document with a **Hole** feature could not be opened again (the hole's `kind` field collided with the feature tag in the file). Fixed and covered by a save / reopen test.

## Not done yet
* **Profile selection** in Extrude: all closed loops of a sketch are used (outer + holes); no choosing of single regions of overlapping shapes.
* Extrude: **to object / to face**, **thin (wall) extrude**, start offset. Revolve: axis only the sketch X or Y axis.
* **Straight-line sweep** (the kernel wants an arc or spline path), guide rails, twist; **Pipe**; **Rib / Web**; **Thread** and tapped / standard hole sizes.
* **Scale** and **Move / Copy** body, **Split Body / Split Face**: the feature model holds one body per chain and the kernel can only move bodies rigidly, so these need a kernel step first (scaling every surface and curve consistently, multi-body documents).
* Construction **axes and points**; mirror / pattern about faces, datum planes, edges and along paths; pattern of bodies / faces.
* Sketch: **project / intersect body edges**, rectangular / circular **sketch patterns**, text, tangent arc, conic, break, sketch scale / move; snapping while dragging geometry; typed length / angle.
* **Inspect**: section analysis, interference check, curvature / zebra, draft analysis.
* Appearance / materials per body, STL / 3MF **import** (mesh bodies), DXF as a File ▸ item of the Design workspace (the sketch palette has it).
* Components, assemblies, joints, motion (the engine has `kw-assembly`; no UI), sheet metal and Form (engines exist; no UI), Surface / Mesh tabs, 2-D drawings, scripting UI, autosave / version history, background evaluation thread.

## How it was checked
Every number above is compared against a closed form in the tests, not against a screenshot. Real mouse interaction (dragging, clicking in the 3-D view, native file dialogs) could not be driven in the environment this was built in and needs a hand test.
