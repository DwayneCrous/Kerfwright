# Kerfwright vs. Fusion 360 (Design / Solid): where we are

Status after round 2 (multi-body, extrude options, construction geometry, threads, sketch project / patterns / text, section
analysis, assemblies, sheet metal, Form, and the new ribbon). **Done** = in the engine, in the UI and covered by tests against
closed forms. What is still missing is listed at the end.

## The ribbon
Icon buttons grouped as in Fusion 360; every group has a **dropdown (`CREATE ▾`, `MODIFY ▾`, …)** that lists *all* of its commands
with icon, name and shortcut under category headings, and the most used ones are big buttons next to it. Tabs: **SOLID**
(CREATE, MODIFY, CONSTRUCT, INSPECT, ASSEMBLE, SELECT), **SHEET METAL**, **FORM**, **TOOLS** (inspect, appearance and parameters,
import / export) and, while sketching, **SKETCH** (CREATE, MODIFY, PROJECT, CONSTRAINTS, OPTIONS, FINISH). SURFACE and MESH are not
shown: nothing is behind them. A narrow window scrolls the ribbon sideways.

## Done
| Area | What works | Tests |
|---|---|---|
| **File exchange** | Import STEP / IGES (kept inside the `.kwf`), export STEP / STL / OBJ / GLB | STEP round trip 1e-6, STL volume, junk refused |
| **Sketch tools** | line, rectangle, circle, arc, spline, fit spline, point, polygon, slot, ellipse, 3-point arc / circle, trim, extend, offset, corner fillet / chamfer, mirror, **rectangular and circular patterns**, **text** (letters as profiles), **Project** (model edges onto the sketch plane, locked) and **Intersect** (where the plane cuts the model), dimensions, 13 constraints, snapping with inference lines | `kw-sketch/tests/{snaps,shapes,patterns,curves}.rs`; text extrudes to glyph area x height within 1 % |
| **Profiles** | lines, arcs, circles, ellipses, splines all extrude (with holes); **a single region of overlapping shapes can be chosen** (click regions in the view) | 3 regions of circle + rectangle sum to the union, closed forms |
| **Extrude** | distance, symmetric, flip, through all, second side, taper, **to a face**, **start offset**, **thin (wall) extrude** inside / outside / centred, New / Join / Cut / Intersect | tapered frustum, thin = perimeter x t x h, to-face stops on the face |
| **Revolve** | about the sketch X / Y axis, **a picked body edge**, or **a construction axis** | Pappus |
| **More solid tools** | Loft, Sweep (arc / spline path **and a straight line**), **Pipe**, **Rib**, Draft, Offset body, Variable fillet, Fillet, Chamfer, Shell, patterns, Mirror, Combine (keep tools) | frustum, Pappus, pipe pi(ro^2 - ri^2)L |
| **Hole** | simple / counterbore / countersink, through all, **tapped** (ISO metric coarse + fine, UNC, UNF tables with tap drills) and cosmetic **Thread** on a hole wall or pin | tap-drill diameters from the table, volume = block - pi r^2 t |
| **Bodies** | **Move / Copy**, **Scale** (uniform; every surface and curve type), **Split Body**, Bodies list with eye, name, colour, material (Appearance); features target the body you click | volume x k^3, area x k^2, halves sum to V |
| **Construction** | **planes** (offset / tilted, midplane, through 3 points), **axes** (2 points, edge, normal to face, 2 planes), **points** (coordinates, edge middle, axis and plane); used by Revolve, Circular pattern, Mirror; drawn in the view with eye toggles | each construction against its closed form |
| **Inspect** | Measure (faces, edges, distances, angles, mass, inertia, threads), **Interference** (volumes), **Section Analysis** (clip plane with a cap) | box closed forms; GPU test: the cap shows exactly when a section cuts the solid |
| **Assemblies** | components (parts with connection frames), rigid / revolute / slider / cylindrical / planar **joints**, motion sliders, posed view, interference check, mass, STEP export; stored in the `.kwf` (`assembly.cbor`) | joint math against the rotation, save / reopen |
| **Sheet metal** | tab with L / U / Z strips and a custom strip editor, **flat pattern** with bend table (also from a recognised folded body), **DXF** export, **Refold** | volume = analytic strip volume, unfolded length = flats + bend allowances |
| **Form** | box, cylinder, sphere, torus cages; move vertices numerically, sharp / smooth edges, scale; finished into a body of patches | box 20^3 = 8000 within 1 % |
| **Workflow** | timeline, parameters, undo / redo, browser, view cube, snapping, edge and face picking | `crates/kw-app/tests/shell.rs` (53 tests) |

## What fell short (be aware before relying on it)
* **Modelled threads** are not built: the kernel has no helix, and a swept thread plus Boolean could not be made to validate. Threads are **cosmetic** (designation, length, tap drill, listed in Measure); the hole is drilled to the tap-drill size.
* **Pyramid Form** is not offered: its apex makes the patch fit take over a minute. Smooth cages (sphere) take seconds to finish; the body of a Form is slow to measure (volume integration of many patches).
* Form cage vertices are moved by typing numbers; there is no dragging in the view. No crease editing per edge (all sharp or all smooth), no bridge / extrude of faces.
* **Thin extrude** outside / centred has mitred (sharp) corners; **regions** are limited to sketches of 10 loops, and coincident loops can make the region Booleans fail (the dialog says so).
* **Interference** reports volumes only (no overlap body shown in the view). The assembly view cannot be picked in; joints are set with named frames (frames come from picked faces, the origin, or numbers).
* **Project** locks the curves; it projects every model edge (delete the ones you do not want). **Intersect** is read from the tessellation (lines, circles and arcs recognised, the rest as fit splines).
* CAM and FEA still use the last solid only. STL import is not built; IGES import is untested against a real sample. The cut cap in Section Analysis has a one-pixel fringe along silhouettes (edge lines cover it).
* Real mouse interaction (dragging, clicking in the 3-D view) and native file dialogs could not be driven where this was built; the logic behind each click is tested through the same code paths and needs a hand test.

## Still missing
* Surface and Mesh tabs, guide-rail sweeps, twist, Split Face, pattern of bodies / faces / along a path, sketch tangent arc / conic / break / scale.
* Curvature / zebra / draft analysis, per-body textures, STL / 3MF import, 2-D drawings, scripting UI, autosave / version history, background evaluation thread, joint limits editing and contact solving in assemblies.
* Sheet-metal flanges added to a body's edges (the strip builder makes the whole part), hems, corner reliefs, rip.
