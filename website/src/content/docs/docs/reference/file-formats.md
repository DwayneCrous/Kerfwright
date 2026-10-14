---
title: File formats
description: "What each import and export format supports, and where it stops."
sidebar:
  order: 2
---

## Summary

| Format | Import | Export | Used for |
|---|---|---|---|
| `.kwf` | yes | yes | The native project |
| STEP (AP203, AP214, AP242) | yes | yes | Solids |
| IGES 5.3 | yes | no | Trimmed surfaces sewn into solids |
| STL (binary, ASCII) | yes | yes | Meshes |
| OBJ with MTL | no | yes | Meshes with materials |
| glTF 2.0 and GLB | no | yes | Meshes for the web |
| DXF R2000 (ASCII) | yes | yes | Sketches, flat patterns, drawings |
| PDF 1.4 | no | yes | Drawings |
| Gerber RS-274X with X2 | no | yes | Board copper, mask and profile |
| Excellon | no | yes | Board drills |
| IDF 3.0 | no | yes | Board for MCAD |
| Pick-and-place CSV | no | yes | Assembly |
| G-code (Fanuc, Haas, Siemens 840D, Heidenhain TNC, LinuxCNC) | no | yes | Toolpaths |

## STEP

An in-house ISO 10303-21 parser reads AP203, AP214 and AP242 B-reps. It enforces limits and does not panic on malformed input. A STEP import becomes an `Import` feature that stores the file text in the project. Export writes the model's solid, and the posed assembly as one product with one solid per instance (no nested product structure).

Checked by round trip with OpenCASCADE, truck and Gmsh, with geometry deviation ≤ 1e-5 and topology counts preserved. A 28.3 MB file with 445,000 instances imports in 1.39 s. On a public 259-file corpus, 259 imports yield valid solids.

## IGES

IGES 5.3 is read for entities 100, 102, 110, 124, 126, 128, 141 to 144, 186, 502, 504, 508, 510 and 514, and trimmed surfaces are sewn into solids. Import is untested against a real-world sample from a CAD system.

## STL

Binary files are read as float32, so coordinates keep about seven significant digits. Corners closer than 1e-6 of the bounding-box diagonal are welded, degenerate triangles are dropped, and the winding is made consistent and outward. Units (mm, cm, m, inch) and the up axis (z or y) are chosen in the *Import mesh* window.

A mesh imports as a **mesh body**, which can be moved, scaled, measured and exported (STL, OBJ, GLB, but not STEP). Solid features, FEA and CAM refuse a mesh body with "convert it to a solid first". **Convert to solid** makes a faceted B-rep: triangles in one plane (within 2e-6 of the diagonal) become one face. The mesh must be closed, with at most 150,000 triangles. There is no cylinder or sphere fitting.

## DXF

ASCII R2000 (`AC1015`) only. Sketch import reads POINT, LINE, CIRCLE, ARC, ELLIPSE, LWPOLYLINE (bulges become arcs) and SPLINE. Construction geometry goes on its own layer. Writing uses the shortest round-trip doubles, so a write then read is bit-exact; round trips agree to 1e-6 over random sketches.

**Limits:** the header writes `$INSUNITS = 4` (millimetres) and the importer does no unit conversion. Partial ellipses and rational or non-uniform-knot splines import as fit splines through sampled points, so they are approximate. DXF carries geometry only, with no constraints or dimensions. No binary DXF, hatches or blocks.

## glTF, OBJ

Written by `kw-io`. glTF is Y-up (converted from the Z-up model). Files re-parse cleanly in three independent Rust parsers. They have **not** been opened in Blender, MeshLab or the Khronos validator.

## Gerber, Excellon, IDF

See [Electronics](/docs/workspaces/electronics/). The IDF convention for bottom-side parts is unverified.

## PDF

Drawings are written by hand to PDF 1.4 using the built-in Helvetica font and Adobe's published font metrics. Output is byte-identical for the same input, and was read by poppler and ghostscript. Text outside WinAnsi prints as `?`.
