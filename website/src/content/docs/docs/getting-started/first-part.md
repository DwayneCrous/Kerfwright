---
title: Make your first part
description: "Sketch a plate, constrain it, extrude it, cut a hole and change a dimension to see the timeline rebuild."
sidebar:
  order: 3
---

This walk-through makes a small plate with a hole, then edits a dimension. It uses the Design workspace and takes about ten minutes. It also shows the one idea that matters most in Kerfwright: the model is a *history*, and editing an early step rebuilds the later ones.

## 1. Start a project

Run `cargo run --release -p kw-app`. On the welcome page choose **New design**, type a name and pick a folder. The project is saved at once as `<name>.kwf`, so there is never an unsaved project with no file behind it. The application bar shows the name, with `*` after it while changes are unsaved.

If you only want to look around, choose **Open the demo part** instead.

## 2. Create a sketch

Click **Create Sketch** in the ribbon. The view shows the three origin planes. Hover to highlight one, then click the plane you want. The view turns into the flat sketch canvas and the **SKETCH** tab opens in the ribbon.

:::tip
You can also click a *planar face* of an existing body to sketch on it. Curved faces are refused with a message.
:::

Choose **Rectangle** and click two corners. As you move the cursor the app **snaps** and shows what it snapped to: an existing point, the origin, a line's midpoint, an intersection, a point on a curve, or an inference line (horizontal, vertical, a multiple of the angle step, parallel or perpendicular). Completing a snap adds the matching constraint for you. Hold **Alt** to ignore snaps for one click.

## 3. Constrain it

The line colours tell you where you are:

| Colour | Meaning |
|---|---|
| Blue | Can still move: under-constrained |
| Black | Fully constrained |
| Red | Conflicting constraints |
| Yellow | A redundant constraint |
| Purple | Projected from the model, locked |

Use **Dimension** and click an edge to give the rectangle a length, then another edge for the width. Dimension names are `d1`, `d2` and so on, and a value can be a formula over document parameters. When every line is black, the sketch is fully defined. The sketch palette at the right shows the solver's state, including the remaining degrees of freedom.

## 4. Extrude

Click **Finish Sketch**. The **Extrude** dialog opens straight away with that sketch. It defaults to *New Body* because the sketch is not on a body yet. The model shows a live preview as you change the distance, the direction or the operation. Drag the arrow in the view or type a distance, then click **OK**.

A sketch that a feature has used is hidden afterwards. Its eye icon in the browser brings it back.

## 5. Cut a hole

Click the plate's top face to sketch on it, draw a circle, dimension its diameter and position, and finish the sketch. In the Extrude dialog set the operation to **Cut**. The preview shows the cut. If a preview fails it turns red and **OK** is disabled until you fix it.

Alternatively use **Hole** from the SOLID tab, click a face to place it, and choose simple, counterbore, countersink or tapped.

## 6. Watch the history work

Look at the **timeline** along the bottom. There is one chip per feature: the sketch, the extrude, the second sketch, the cut. Now:

1. **Double-click** the first sketch in the browser, or right-click its chip and choose **Edit**.
2. Change the width dimension and finish the sketch.
3. The plate and the hole update. The hole's position follows because it is dimensioned in a sketch that depends on the plate.

Try these on the timeline too:

- **Roll back.** Drag the rollback marker, or use the `|<` `<` `>` `>|` buttons, to see the model as it was at an earlier step. While you are rolled back, adding features is refused so a new feature is never inserted behind your back. Roll forward first.
- **Suppress.** Right-click a chip and choose **Suppress**. The chip turns grey and the model behaves as if the feature were not there.
- **Reorder.** Drag a chip. If it would come before something it needs, the move is refused with the engine's own message and nothing changes.

## 7. Undo, save, export

Undo and Redo are `Ctrl+Z` and `Ctrl+Y`. Save is `Ctrl+S`. **File ▸ Export** writes the model as STEP, STL, OBJ (with MTL) or GLB.

## Next

- [The Design workspace](/docs/workspaces/design/) covers every tool, the browser and assemblies.
- [How the engine rebuilds](/docs/concepts/parametric-engine/) explains what happened in step 6.
- [Manufacture](/docs/workspaces/manufacture/) takes this part to G-code.
