---
title: Manufacture
description: "Setups, stock, operations, tool and machine libraries, stock simulation and G-code posts."
sidebar:
  order: 2
---

Manufacture (CAM) turns a model into a program a machine can run. Start it from the ribbon's workspace menu, or open a project with `--workspace cam`. Setups, operation settings, simulation results and the G-code post live in the left panel.

## The flow

1. **New Setup**: say how the part sits on the machine.
2. **Add operations**: face, pocket, adaptive clearing, contour, drill and so on.
3. **Simulate**: cut a virtual stock with every operation and look for gouges, leftover material and collisions.
4. **Post**: write one G-code program per setup, and a setup sheet.

## Setups

A **setup** is one clamping of the part. It holds:

| Item | Detail |
|---|---|
| Work coordinate system | An origin, a tool direction (Top, Bottom, Front, Back, Left, Right, or typed) and a work offset G54 to G59 |
| Stock | A block, a round bar, an explicit block, or **what the previous setup left** |
| Body roles | Each body of the design or posed assembly is the *workpiece*, a *fixture* or *ignored* |
| Machine and post | Chosen from the libraries |

A project can hold several setups, so a part cut on its top and then flipped to cut its bottom is two setups where the second starts from the first one's result. In the test with two 4 mm cuts, the second setup began from the first setup's final volume within 1 %, both removed volumes were within 4 % of 4,000 mm³, and what remained was the part to within 4 %, with no overcut.

## Operations

| Operation | What it does |
|---|---|
| Face | Mills the top of the stock flat |
| Pocket | Clears the inside of a closed outline, leaving islands |
| Adaptive clearing | Clears material while holding the tool's engagement angle near constant |
| Contour (outside, inside) | Follows an outline |
| Chamfer | V-tool; the width on the top edge sets the depth |
| Drill | Plain or pecked, with holes found from the model's cylindrical faces |
| Bore | Helical interpolation for larger holes |
| 3-D raster finish | Ball, flat or bull-nose finishing by exact drop-cutter, with shank and holder collision checks |
| Waterline | Constant-Z finishing |
| Rest finishing | Cuts only where earlier operations left more than a threshold above the part |
| Swarf | Flank milling a wall, with optional draft |
| Tilted contour | Contour with the tool leaning against the wall |

**Regions** come from the outline of a flat face of the model (islands included), the model's footprint, or a typed rectangle. Curved outlines are faceted at 2 µm.

An operation keeps its settings and a **hash of what it was made from**. If the model, the tool, the stock, the work system or an earlier operation changes, the operation shows as *out of date*, and **Regenerate** remakes it.

### Adaptive clearing

Adaptive clearing follows boundaries and steers a spiral from a helical entry so that the **radial engagement stays near a target angle**. Over 50 seeded pockets (convex, concave, with islands and slots) the worst measured engagement was 1.010× the target, outside the entry, links and finish moves (13 % of path length on average). Coverage was 93.5 % to 100 %, and the rest is left for the finish pass.

:::caution[Patent review pending]
The method was derived from public literature, and it deliberately avoids the medial-axis method of US 9,164,503. The freedom-to-operate review is **not done**, and `LICENSES/patent-watch.md` records that adaptive clearing must not ship until it is.
:::

## Libraries

Open **Libraries** to manage:

- **Tools** of nine kinds, with flutes, material, coating, holder and cutting data per material group.
- **Holders**.
- **Machines**: spindle and feed limits, travel, tool-changer size, a post and, for five-axis machines, the rotary axes.
- **Post-processors**: your own `.post` files are checked as you type and errors name the line.

A built-in set ships with the program. Yours live in `~/.config/kerfwright/cam/` (or `$KERFWRIGHT_CONFIG`), and **the project embeds copies of the machine, the post and the tools it uses**, so it opens the same elsewhere. Speeds and feeds follow the material's surface speed and chip load and are held to the machine's limits. Tool numbers are unique across the project.

## Simulation

**Simulate** cuts the stock with every operation, each with its own tool. A **tri-dexel** model stores the stock as grids of rays along three axes, and each operation trims the rays its tool sweeps through. The result is compared with the workpiece and coloured on the stock: **material left**, and **material cut too deep**. Tool, shank and holder are checked against the part and the fixtures.

In validation, stock volume matched analytic removed volume to within 0.13 % on five cases, and a 21-part suite of about a million tool poses had 0 gouges (worst penetration 1.0 µm) and 0 holder collisions. Planted faults (a 0.2 mm gouge, a stick-out 3 mm too short) were detected by both checkers.

## Multi-axis

Five machine types are modelled with analytic inverse kinematics. A machine with two rotary axes gets **tool-centre-point programs**: a tilted setup (3+2) and the swarf and leaning-contour operations are converted with the inverse kinematics and posted as `Rapid5` and `Feed5`. The report lists rotary-axis travel, axis-limit violations and the error of the axes between poses. A step cap stops an axis from jumping near a singularity: in a 400-pose sweep past the pole, the largest rotary step was 4.83° against a 5° cap.

## Posting

One program per setup is written under its work offset, plus a **setup sheet** as a web page and CSV. Five dialects are supported:

| Dialect | Note |
|---|---|
| Fanuc | |
| Haas | |
| Siemens 840D | |
| Heidenhain TNC | A different grammar (Klartext) |
| LinuxCNC | Tool-centre-point control comes from machine kinematics, not G-code, so its post emits a comment |

Each dialect is replayed by an **independent interpreter** (`kw-gsim`) that rejects unknown words, tracks modal state and understands rotary words, with no dependency on the CAM crate. All five replay the pocket, island pocket, contour, drilling, the 10,578-event adaptive path and 50 five-axis programs without errors.

:::caution[Not run on a real machine]
The posts and the interpreters were written from the general conventions of public manuals and agree with each other. Nothing has run on a real controller. Prove out any program in a simulator you trust, with the spindle off, before cutting.
:::

## Limits

- Stock simulation checks fixtures with an upright tool only. The **machine housing is not modelled**, so axis-limit and housing collisions are out of scope.
- The waterline works on single-valued surfaces (undercuts are not seen) and stays within about 0.03 mm of the surface on a 0.5 mm grid.
- The stock surface is as accurate sideways as the dexel cell (0.25 mm and up).
- Canned drilling cycles are written out as moves on a rotary machine. Work offsets above G59 are not offered.
- Turning and additive manufacturing are not built.
- CAM uses the last solid only, and a mesh body must be converted to a solid first.
- The checkers (dexel and gouge checks, the interpreters) were written by the author of the code they check. They are separate implementations and catch planted faults, but they are not a third-party tool.
