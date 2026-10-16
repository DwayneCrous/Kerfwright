# Brief for patent and trademark counsel

Kerfwright is an MIT-licensed, pure-Rust CAD/CAM/CAE/ECAD suite, pre-release, built from public sources. It is not
affiliated with any CAD vendor. This page lists what we need reviewed before a release or advertising. It is not legal
advice and it records no conclusions: `LICENSES/patent-watch.md` is where a finished review goes.

| # | Question | Where it is in the code | Current state |
|---|---|---|---|
| 1 | Freedom to operate for **adaptive / constant-engagement clearing** (engagement-angle control on a raster stock model, spiral from a helical entry; no medial axis). Known nearby: US 9,164,503 (medial axis), Freesteel/HSMWorks-lineage families. | `crates/kw-cam/src/adaptive.rs`, `verify.rs`; UI entry `crates/kw-app/src/cam/mod.rs` | Off by default: cargo feature `adaptive-clearing` |
| 2 | Freedom to operate for **vertex blends** (rolling-ball sphere patch where three equal-radius fillets of three planes meet). | `crates/kw-blend/src/vertex.rs`; call site in `crates/kw-blend/src/blend.rs` (`fillet`) | Off by default: cargo feature `vertex-blend` |
| 3 | Does any patent cover a clickable **orientation cube** (faces, edges and corners turn the camera; arrows for quarter turns)? The vendor's name for its version is a trademark; we use neither the name nor its artwork. | `crates/kw-app/src/chrome/orientation_widget.rs` | Shipping; patent status unverified |
| 4 | **Trademark search** for "Kerfwright" (USPTO, EUIPO, WIPO; software and CAD classes) and advice on registering it. | Binary name `kerfwright`, window title, website | No search recorded |
| 5 | Review of our **comparison wording**: public text names vendors only as a plain factual comparison, with no logos or screenshots. Is that safe in advertising? | `README.md`, `TRADEMARKS.md`, `website/` | For review |
| 6 | Whether "independent implementation from public sources" is an accurate public description given that some code was drafted with AI assistance. We do not claim "clean room". | `CLEAN_ROOM.md` (Provenance and IP Policy) | For review |
| 7 | **UI look-alike risk.** The Design workspace's ribbon (SOLID / SHEET METAL / FORM / TOOLS tabs; CREATE / MODIFY / CONSTRUCT / INSPECT / ASSEMBLE groups), browser tree, bottom timeline and light-blue theme closely follow a commercial CAD application's layout. `CLEAN_ROOM.md` rule 3 says layouts are not copied. Is this layout and naming a trade-dress or copyright concern, and should tab and group names or the arrangement be changed before the screenshots in `docs/images/` are used in advertising? | `crates/kw-app/src/design/ribbon.rs`, `crates/kw-app/src/chrome/theme.rs`; screenshots in `docs/images/` | Shipping; not reviewed |

Also available: the research documents under `docs/01-research/` (each ends with a sources section), the per-phase
reports under `docs/`, and the git history.
