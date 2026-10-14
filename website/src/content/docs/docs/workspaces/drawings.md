---
title: Drawings
description: "Production drawings from the model: views, sections, dimensions, tolerances, GD&T symbols, parts lists and PDF or DXF output."
sidebar:
  order: 6
---

The Drawing workspace makes 2-D sheets from a design or an assembly. Start with `--workspace drawing`, or `--drawing-demo` for a demo sheet.

## Layout

The **left panel** lists sheets with their views and annotations. The **middle** is the sheet: the wheel zooms, middle or right drag pans, drag a view or annotation to move it, and Delete removes the selection. The **right panel** edits the title block and whatever is selected. **Sheet tabs** run along the bottom. The ribbon groups are SHEET, VIEWS, ANNOTATE, TABLES, MODIFY and OUTPUT.

Click tools (Section, Detail, dimensions, notes, symbols, Balloon) show a live preview, snap to the model's points and edges, and are cancelled with **Esc**.

## Sheets

**New Drawing** starts from the design or the assembly. Choose the standard (ISO or ASME), the size (A0 to A4, ANSI A to E), first or third angle projection, units and the title block fields. You can add more sheets.

## Views

| View | Notes |
|---|---|
| Base | Front, Back, Top, Bottom, Left, Right or Isometric. Styles: visible, visible and hidden, wireframe, shaded. Placed at the largest standard scale that fits. |
| Projected | On any side of a parent view. It stays aligned and moves with its parent. |
| Section | Click the view, then the cutting line. Cut faces are hatched. Flip chooses the side. |
| Detail | A centre and radius, enlarged, with a circle and a letter on the parent. |

Hidden-line removal tessellates every body, projects the triangles and tests sampled edges against nearer triangles, keeping exact lines, circles, arcs and ellipses where possible. Tangent edges and seams are not drawn.

## Annotation

- **Dimensions**: linear, aligned, diameter, radius, angle. Pick the model's points and edges. Add a tolerance (symmetric, bilateral or limits), text before and after, and the decimals.
- **Centre marks and centre lines.**
- **Notes** with leaders.
- **Symbols**: datums, surface finish (ISO 1302) and feature control frames covering 14 GD&T characteristics.

## Tables

A **parts list** with the columns you choose, and **balloons** numbered by the list. The list exports to CSV, equal to the assembly's bill of materials.

## Staying in sync

A dimension stores the two [face names](/docs/concepts/topological-naming/) that meet at its edge, plus which point of the edge. When the model changes, the sheet shows a banner, its tab gets an orange dot, and the ribbon's **Update** lights up. Update resolves the names again and never guesses: an ambiguous or missing one is reported. In the test, changing a hole from 6 to 10 mm updated the view, the dimension (6 → 10) and the BOM volume. Undoing the model change makes the sheet stale again.

## Output

**PDF** (1.4, written by hand with the built-in Helvetica font, checked in poppler and ghostscript) and **DXF R2000**. The output is byte-identical for the same input.

## Limits

- Section cuts are straight lines. There is no detail of a section, no dimensions on section or detail views, and no break, auxiliary, broken-out, ordinate or baseline views.
- In an assembly drawing, dimensions are fixed in space and do not follow the model. Balloons do follow it.
- GD&T symbols are placed, not validated. There are no automatic balloons, and only one parts list per sheet.
- Text is built-in Helvetica (WinAnsi), so other scripts print as `?`.
- A section's cut outline is read from the tessellation, so curved cut edges are polylines at about 1e-4 of the part size.
- **Update** regenerates every view, not only the stale ones.
