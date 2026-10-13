# Patent Watch (not legal advice — for counsel review)

| Area | Reference | Status / action |
|---|---|---|
| T-splines / T-NURCC | US 7,274,364 (and family; T-Splines Inc., acquired by Autodesk 2011) | Verify expiry & continuations before shipping true T-spline local refinement. Ship Catmull-Clark SubD first. |
| Medial-axis toolpath optimisation | US 9,164,503 | Do not implement the claimed method; use engagement-angle control from open literature. |
| Adaptive / constant-engagement clearing | Various (Freesteel/HSMWorks lineage and others) | **Open.** `kw-cam/src/adaptive.rs` (Phase 5) controls the cutter engagement angle on a raster model of the stock and steers a spiral from a helical entry; it does not use a medial axis. It was written from the open literature on engagement-angle control and without reading or copying any CAM source, but **no freedom-to-operate search has been done**: until one is recorded here the adaptive-clearing operation must not ship. |
| Constant-engagement paths, spiral-in with trochoidal corners | Same family | Same action; the trochoid fallback of the plan was not built (the planner follows the boundary instead), which may or may not change the picture. |
| Open-source CAM code | OpenCAMLib, LinuxCNC | Not consulted. The drop-cutter, dexel, kinematics and post code are original; LinuxCNC/Fanuc/Siemens/Heidenhain dialects are implemented from public manuals' general conventions, and the interpreters that check them are written in this repository (see the Phase 5 report: they are not third-party simulators). |
| Blend/fillet methods | Various CAD vendor filings | Prefer academic-literature methods; search before shipping vertex-blend algorithms. |

Rule: nothing listed here ships until a freedom-to-operate review is recorded in this file.
