# Patent Watch (not legal advice — for counsel review)

Rule: nothing listed here ships until a freedom-to-operate review is recorded in this file (see "Review records" below).
`scripts/check_gated_features.sh` runs in CI and fails if a gated feature is switched on by default.

States: **Open** (no review, not gated: must be fixed before release), **Gated** (no review, switched off in the default
build), **Cleared** (a review is recorded below), **Not implemented**.

| Area | State | Reference | Status / action |
|---|---|---|---|
| T-splines / T-NURCC | Not implemented | US 7,274,364 (and family; T-Splines Inc., acquired by Autodesk 2011) | Verify expiry & continuations before shipping true T-spline local refinement. Ship Catmull-Clark SubD first. |
| Medial-axis toolpath optimisation | Not implemented | US 9,164,503 | Do not implement the claimed method; use engagement-angle control from open literature. |
| Adaptive / constant-engagement clearing | **Gated** | Various (Freesteel/HSMWorks lineage and others) | `kw-cam/src/adaptive.rs` (Phase 5) controls the cutter engagement angle on a raster model of the stock and steers a spiral from a helical entry; it does not use a medial axis. It was written from the open literature on engagement-angle control and without reading or copying any CAM source, but **no freedom-to-operate search has been done**. It is behind the off-by-default `adaptive-clearing` cargo feature (`kw-cam`, forwarded by `kw-app`): no ribbon button, not in `OpKind::ALL`, and `OpKind::Adaptive` refuses to generate. It must not be enabled in a release until a review is recorded here. |
| Constant-engagement paths, spiral-in with trochoidal corners | **Gated** | Same family | Same action and same feature gate. The trochoid fallback of the plan was not built (the planner follows the boundary instead), which may or may not change the picture. |
| Open-source CAM code | n/a | OpenCAMLib, LinuxCNC | Not consulted. The drop-cutter, dexel, kinematics and post code are original; LinuxCNC/Fanuc/Siemens/Heidenhain dialects are implemented from public manuals' general conventions, and the interpreters that check them are written in this repository (see the Phase 5 report: they are not third-party simulators). |
| Blend/fillet methods: vertex blends | **Gated** | Various CAD vendor filings | The rolling-ball corner patch in `kw-blend/src/vertex.rs` is behind the off-by-default `vertex-blend` cargo feature of `kw-blend`. Without it, corners of filleted edges meet in a mitre (a valid solid; checked by the downstream suites). Search before enabling it by default. Constant and variable radius fillets and chamfers (academic-literature methods) are not gated. |
| Orientation widgets (clickable cube showing the camera orientation) | **Open** | Autodesk's ViewCube trademark; patent status **unverified** | `kw-app/src/chrome/orientation_widget.rs` is a cube with clickable faces, edges and corners. It carries no vendor name in code or UI. Counsel should check whether any patent covers this interaction. If one does, replace it with a simpler axis gizmo. |
| Name "Kerfwright" | **Open** | Trademark registers (USPTO, EUIPO, WIPO) | No search recorded. See `TRADEMARKS.md`. |

## Review records

Add one entry per item when its review is done. Do not mark an item **Cleared** without an entry. When an entry is added,
change the state above, and (for a gated feature) say in the same pull request whether the feature may become a default.

```
### <item> — <date, YYYY-MM-DD>
- Reviewer: <patent counsel or firm; "none" is not a review>
- Scope: <jurisdictions, claims or patent families examined>
- Search method: <databases, queries, date range>
- Conclusion: <cleared / cleared with a design-around / not cleared>
- Conditions: <changes the reviewer required, and where they are in the code>
- Document: <link or location of the opinion>
```

(No reviews recorded yet.)
