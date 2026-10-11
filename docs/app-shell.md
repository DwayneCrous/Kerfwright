# The application: welcome page, projects and workspaces

`cargo run --release -p kw-app` starts **Kerfwright**, the one program that ties the engines of Phases 1–5 together. It opens on a welcome page; a project then shows six workspaces on a rail at the left. `scripts/app_acceptance.sh` runs the tests and writes a screenshot of every screen to `target/app/`.

## Welcome page
New project (name + folder, saved at once as `<name>.kwf`), Open project (native dialog), Open the demo part, and the ten most recent projects (kept in `~/.config/kerfwright/recent.json`; missing files are struck out with a "forget" button). `kerfwright FILE.kwf`, `--demo` and `--workspace NAME` skip it.

## Project
A project is a `.kwf` file: the feature document of Phase 4 plus the Phase 5 data as extra entries (`cam.cbor`, `fea.cbor`, `board.cbor`, through `kw-store`). File → Save / Save as / Open / Close; Ctrl+S, Ctrl+Z, Ctrl+Y; the title shows a dot while there are unsaved changes, and closing, opening another project or quitting asks before discarding them.

## Workspaces
| Workspace | What it does |
|---|---|
| **Sketch** | The constraint-solving 2-D sketcher of `kw-sketcher`, embedded. Start a sketch on the XY / YZ / XZ plane, or edit an existing sketch feature (it shows the evaluated dimensions); *Finish sketch* writes it into the timeline (editing keeps the parameter bindings of dimensions you did not change). |
| **Model** | Feature timeline (suppress, roll back, delete, per-feature properties with expression fields), toolbar (rectangle / circle sketch, Extrude, Fillet on edges picked in the 3-D view with Shift to add, Linear pattern, Parameter), undo/redo, and a 3-D view: orbit, pan, zoom, click to select faces and edges. |
| **CAM** | Pocket, adaptive clearing, outside contour and ball-nose 3-D finishing on the model's footprint (editable region, heights, tool, feeds); toolpaths are drawn inside a wireframe of the part; operations are listed and stored in the project; *Simulate stock* runs the tri-dexel check; *Post* writes Fanuc, Haas, Siemens 840D, Heidenhain or LinuxCNC code (preview and *Save G-code*). |
| **FEA** | Material presets, element size, supports and loads on faces picked in the 3-D view (force in N or pressure in MPa), static solve with quadratic tetrahedra on a background thread, colour by von Mises stress or displacement with an exaggeration slider and a legend. The study is stored in the project. |
| **ECAD** | A board in 2-D (pan/zoom, layer toggles): generate one of the 20 seeded sample boards, Place, Route all (background thread), design-rule check (violations marked and listed), copper pour, and export Gerber / Excellon / IDF / pick-and-place files into a folder or a STEP file. The board is stored in the project. |
| **Render** | The model drawn with the realtime pipeline (OIT, ambient occlusion, edges) or path traced on the GPU with progressive samples, an optional denoise and *Save PNG*. |

## What is verified, and what is not
* `crates/kw-app/tests/shell.rs` (17 tests): welcome page, create / open / save / reopen with identical document hashes, recent list, unsaved-changes prompts for close / open / quit, damaged files, every workspace drawing; sketch → feature → extrude → edit → undo; extrude / fillet / undo / expressions with volumes checked; CAM operations posted in all five dialects and replayed by the independent interpreters, stock removal; the FEA workspace against beam theory (tip deflection 0.1909 mm vs 0.1905 mm, bending stress 60.8 vs 60 MPa); the ECAD flow (generate, background route, DRC, pour, export, Gerber re-read); Render (GPU): realtime image, path trace, denoise, PNG; viewport (GPU): offscreen render and id-buffer picking of faces and edges.
* The screenshots are taken by the real application through eframe, so the GPU path into egui is exercised end to end for the static screens.
* **Not tested here:** mouse and keyboard interaction in a real window (orbit, click-to-pick, dragging in the sketcher — the sketcher's own UI tests drive it with synthetic events), and the native file dialogs (`rfd` needs a desktop portal; the tests use scripted dialogs).

## Limits of this first version
* Model features exposed in the UI: sketch, extrude, fillet, linear pattern, parameter. The engine has many more (revolve, loft, sweep, shell, hole, chamfer, mirror, sheet metal, SubD …) that are not reachable from the UI yet; the timeline cannot be reordered by dragging; sketches are placed on the origin planes (not on faces).
* The model is evaluated on the UI thread: a model whose Booleans take seconds will make the window pause. CAM, FEA, routing, stock simulation and denoising run on background threads.
* CAM works on rectangles (typed or taken from the model's bounding box), not on selected faces or pockets; there is no tool library and one tool per operation. 3-D finishing has no holder-collision handling in the UI (the library has it).
* FEA meshes the faceted tessellation of the first body (curved surfaces are approximated); supports and loads are stored by face index and go stale if the geometry changes shape.
* ECAD offers generated sample boards, not a schematic or netlist editor, manual part placement or interactive routing.
* The Render workspace uses its own GPU device and rebuilds the path-tracer scene when the camera rests; it renders one fixed 960×600 image size.
