---
title: Design
description: "Sketching, solid and surface modelling, the browser, the timeline, assemblies, sheet metal and subdivision forms."
sidebar:
  order: 1
---

Design is where parts are made. Sketch and Model are one workspace: you sketch on a plane or a face, finish the sketch, and a feature dialog opens at once with a live preview.

## The frame

| Area | What it is |
|---|---|
| Application bar | Home (back to the welcome page), File, Undo, Redo, Save, and the project name with `*` while unsaved. |
| Ribbon | The workspace menu at the left, then the tabs and tool groups of the current workspace. Each group has big icon buttons and a `TITLE ▾` dropdown listing every command with its icon, name and shortcut. A command that cannot run says why in its tooltip. |
| Browser (left) | Document Settings, Origin, Sketches, Construction, Bodies, Features. |
| Timeline (bottom) | One chip per feature. |
| View | The view cube, orbit, pan, zoom and a navigation bar. |

Closing a project, opening another, or quitting asks first if there are unsaved changes.

## Sketching

Click **Create Sketch**, then a plane or a *planar* face of a body. Curved faces are refused with a message. The ribbon gains a **SKETCH** tab and a palette at the right shows the solver's state.

**Draw:** line, rectangle (corner, centre, three-point), circle (centre, three-point), arc (centre, three-point), spline, fit spline, polygon, slot, ellipse, point and text. **Modify:** trim, extend, offset, corner fillet and chamfer, mirror, rectangular and circular pattern. **Project** and **Intersect** bring model edges into a sketch as locked purple geometry.

### Snapping and inference

The cursor snaps, in this order, to an existing **point**, the **origin**, a line's **midpoint**, the **intersection** of two curves, a point **on a curve**, dashed **inference lines**, and finally the **grid**. A label shows length, angle and the name of the snap.

Inference lines are horizontal and vertical, multiples of the **angle step** from the start of the line (15° by default, 5° to 90° selectable), **parallel** and **perpendicular** to existing lines, and **alignments** with points the cursor has rested on for 0.3 s.

Completing a snap adds the matching constraint: midpoint, point-on-curve on both curves at an intersection, a fixed point at the origin, and horizontal, vertical, parallel or perpendicular for an inferred line. Angle, alignment and grid snaps only place geometry. Hold **Alt** to ignore snaps for a click. Each kind can be switched off in the palette's **Snaps** list.

### How a sketch reads

| Appearance | Meaning |
|---|---|
| Blue line | Can still move |
| Black line | Fully constrained |
| Red | Conflicting constraints |
| Yellow | Redundant constraint |
| Purple | Projected from the model |
| Light-blue shading | A closed profile an extrude can use (holes stay clear) |
| Small icons beside geometry | The constraints that apply |
| Grey brackets | A *driven* (reference) dimension: it measures but does not constrain |

**Drag a box** in empty space to select. Left to right takes what lies completely inside, right to left also takes what the box touches, and Shift adds. Pan with the middle or right button.

### Dimensions

One line gives its length. Two parallel lines give their distance, two meeting lines their angle, a point and a line the perpendicular distance, two points their distance, a circle its diameter and an arc its radius. Names are `d1`, `d2`, and so on. Double-click a label to edit it. A value can be a formula over **document parameters** (live) or over other dimensions (evaluated once). See [The sketch solver](/docs/concepts/sketch-solver/).

## Regions

Every curve in a sketch is cut where it crosses or touches another one, and the enclosed areas are **regions**: the overlap of two circles, the two halves of a circle cut by a line, the arms and centre of a "+" drawn from lines, a ring with a hole, the shape inside the hole. Open curves and whiskers belong to no region and are ignored.

The Extrude dialog draws the regions over the sketch plane and lists them with **Default**, **All** and **None**. Hover shows the one under the pointer and a click toggles it. The dialog starts with the usual regions chosen and will not continue with none. Neighbouring chosen regions merge into one profile with no wall where the line was. Each extrude takes its own regions, depth and operation, and every feature stays tied to its one sketch, so changing a dimension updates them all.

**Limits:** crossings that only graze (very shallow) between ellipses or splines can be missed. Chosen regions that touch only at a corner extrude as separate pieces and may fail to unite. The side-face names of an existing feature can change when a sketch gains intersections.

## Solid features

On the **SOLID** tab:

| Group | Tools |
|---|---|
| Create | Extrude, Revolve, Sweep, Loft, Pipe, Rib, Hole, Thread, Rectangular pattern, Circular pattern, Mirror |
| Modify | Fillet, Variable fillet, Chamfer, Shell, Draft, Offset body, Combine, Move / Copy, Scale, Split Body, Change Parameters |
| Construct | Planes (offset, tilted, midplane, through three points), axes, points |
| Inspect | Measure, Interference, Section Analysis |

**Extrude** has an extent (distance or through all), a second side, a taper angle, *to face*, a start offset and a thin-wall mode (inside, outside, centred). The operation is New Body, Join, Cut or Intersect. An arrow in the view drags the distance. A preview that fails is drawn red and blocks **OK**. A taper with a symmetric extent or a second side is something the kernel cannot do, and the dialog says so.

**Revolve** can turn about the sketch's X or Y axis, a clicked body edge, or a construction axis. **Hole** can be simple, counterbore, countersink or tapped (ISO metric coarse and fine, UNC and UNF with tap-drill tables), and can go through all. **Thread** is cosmetic or modelled as a helical groove, right- or left-hand.

**Fillet** and **Chamfer** take edges you click (Shift adds). Edges are hit within 8 px of the pointer, light up orange under it and stay blue once picked. The filleted result appears after **OK**. **Shell** takes the faces to remove.

Every tool's dialog also *edits* an existing feature: double-click it in the browser or timeline.

### Bodies and parameters

Documents can have several bodies. The browser lists each with an eye toggle, rename, colour and material; a click makes a body the target of new features. **Parameters** are named values usable in any expression, for example `plate_t` or `width / 2`, with units.

## The timeline

A chip per feature. A chip is **red** when the feature fails and **grey** when suppressed. You can:

- **Drag** a chip to reorder. A move that would put a feature before something it needs is refused with the engine's message.
- Move the **rollback marker** (or use the `|<` `<` `>` `>|` buttons) to view the model at an earlier step. While rolled back, adding features is refused.
- **Right-click** for Edit, Suppress, Roll back here and Delete. Delete removes the feature and everything built on it as one undo step, and tells you how many steps depend on it.

How the rebuild works is in [How the engine rebuilds](/docs/concepts/parametric-engine/).

## Surfaces

The **SURFACE** tab makes sheets: Extrude, Revolve, Loft and Sweep with *Output: Surface*, **Patch** (three or more edges; four give one Coons patch, optionally tangent to a neighbouring surface), and **Ruled Surface**. To modify them: **Trim**, **Extend**, **Match** (G1 or G2 to a neighbouring face), **Stitch** (a closed group becomes a solid), **Unstitch**, **Thicken** and **Offset Surface**.

Analysis tools colour any body: **Zebra** stripes, **Curvature** (Gaussian, mean, maximum, minimum), **Draft** against a pull direction, and **Edge Continuity**, which colours each shared edge by gap, G0, G1 or G2 with a tolerance slider and counts.

## Assemblies

The **ASSEMBLE** menu holds components, joints, motion and inspection.

- **Components** can be created empty, copied from a body, made from a whole design, or inserted from a `.kwf`. *Insert Component* keeps a link to the file; *Insert Copy* does not. Double-click a component to edit it in place.
- **Joints** are Rigid, Revolute, Slider, Cylindrical, Planar and Ball. Click a point on each of two components (a flat face gives its centre, corners and edge middles; a round face its axis), choose the type and set offsets, angles and limits. The first component moves to the second immediately as a preview. **As-Built Joint** joins components where they already are. Values can be formulas over named parameters.
- **Grounding and rigid groups.** Several components can be grounded. A rigid group moves as one. A joint between two grounded components, or inside a rigid group, is refused with the reason.
- **Assembly View** poses every component. Drag a free component in the plane facing the camera; dragging one on a joint drives the joint within its limits. **Drive Joints** has a slider per degree of freedom.
- **Linked components** carry a badge: **L** up to date, amber **!** the file changed, red **?** the file is missing, red **!** unreadable. The project keeps its own copy, so a missing file never stops it from opening.
- **Mechanisms.** A joint that closes a kinematic loop is marked *solved*. Drive Joints then shows the mechanism's degrees of freedom and an **Animate** control; a position the mechanism cannot reach is explained instead of drawn.
- **Interference** lists overlaps and outlines them in red. The **Bill of Materials** lists quantity, material, volume, mass and size per part and exports CSV.

Undo and Redo act on the assembly while the assembly view is shown, and on the design otherwise.

## Sheet metal

L, U and Z strips, or a custom strip with thickness, K-factor, bend radius, width, flats and bends. The **Flat Pattern** window shows a bend table, exports DXF and can refold. Bend allowance is `BA = θ(R + K·t)`. This is the *strip* model, one cross-section of constant width, so it does not cover flanges on several edges of a plate, corner reliefs or cut-outs.

## Form (subdivision)

Start from a box, cylinder, sphere or torus cage. The cage is drawn over a live preview of the smooth surface. Click or box-select vertices, edges and faces, and drag them in the camera plane or along an axis (press X, Y or Z; Shift+X, Y or Z for the plane across it). Set grid snap and mirror symmetry in the EDIT CAGE window, and mark edges Sharp or Smooth. **OK** finishes the cage into a body of bicubic patches.

**Limits:** closed cages only, no pyramid, no adding or removing cage faces, and G1 at extraordinary vertices is measured (worst 0.41° to 1.38°) rather than exact.

## Import and export

**File ▸ Import** reads STEP, IGES and STL. STEP and IGES arrive as an `Import` feature, with the file's text stored in the project so the `.kwf` stays self-contained. An STL opens an *Import mesh* window first (units, up axis, what the file holds), then becomes a mesh body marked "(mesh)" in the browser. Right-click it to **Convert to solid**. **File ▸ Export** writes STEP, STL, OBJ with MTL, or GLB.

## Limits

- Sketching happens on a flat 2-D canvas, not in the 3-D view.
- The model is evaluated on the UI thread, so a very heavy model pauses the window.
- Fillet, chamfer and shell reference faces by name, and a feature added while rolled back is refused rather than inserted at the marker.
- STL meshes convert to faceted solids only, with a 150,000-triangle cap.
- Snapping while dragging existing geometry, and typing an exact length or angle while drawing, are not done.
- Real mouse interaction and native file dialogs were not driven in automated tests. The logic behind every click is tested through the same code paths, but pointer handling needs a hand test.
