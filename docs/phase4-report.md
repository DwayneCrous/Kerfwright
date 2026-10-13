# Phase 4 report — parametric engine, assemblies, sheet metal, SubD, scripting

Status: **implemented and tested; two of the seven acceptance criteria are met only in part** (cold replay of a *single* 200-feature chain, G1 exactly zero at extraordinary vertices). Everything below is measured by `scripts/phase4_acceptance.sh` (logs in `target/phase4/`); every number was produced on the development machine (8 threads) and is reproducible with the command given.

## 1. Results against the acceptance criteria

| # | Criterion (docs/04-roadmap.md) | Result | Status |
|---|---|---|---|
| 1 | Naming corpus ≥ 500 scenarios, 0 silent wrong, ≥ 97 % automatic | **1 716 scenarios** (6 families × 300, seed 99) and 580+ with the default run: **0 wrong**, **99.88 %** resolved or correctly flagged (1 060 by name, 185 by lineage, 271 by geometry, 198 correctly flagged as gone/ambiguous, 2 flagged although resolvable, 0 wrong) | met |
| 2 | Same geometry hash and `TopoName` set from a `.kwf` without caches on 1 / 4 / 32 threads | identical on 1, 4, 32 threads, twice each, for three documents (10, 50 and 122 features; hashes in `target/phase4/determinism.log`) | met |
| 3a | Tail edit on a 200-feature model ≤ 150 ms | **53 ms** (8-part model, the edit re-runs a hole, its sketch, the parameter and the join); **192–202 ms** on one linear chain of 100 solid features | met on the part model; **not met** on the single chain |
| 3b | Cold replay ≤ 2 s | **335 ms** (8 threads) / **1 327 ms** (1 thread) on the 200-feature part model; **8.7 s** on one linear chain of 100 solid features | met on the part model; **not met** on the single chain |
| 3c | No-op edit → 0 dependent re-evaluations | **0** features executed, 0.6 ms for the whole 201-feature document (checked after an edit that rewrites a parameter as an equal expression, after a rename, and after undo/redo) | met |
| 4 | Cycle-introducing reorder rejected with an explanation in 100 % of cases | 1 272 / 1 272 moves of generated documents judged exactly like an independent dependency oracle (every valid move accepted, every invalid one refused with a message, document unchanged); plus 1 557 random move / expression-edit / removal operations (637 accepted, 920 refused, 323 cycles printed with their path) | met |
| 5 | Sheet metal: unfold→refold volume error ≤ 1e-6, flat lengths = analytic bend allowance within 1e-6 mm | 135 flat patterns written as DXF, read back and measured: worst error **1.4e-14 mm**; 16 bodies recognised from their B-rep without the recipe, unfolded, refolded: worst volume error **1.4e-14** relative; B-rep volume equals the analytic strip volume to 1e-9 | met (for straight-bend strips, see §4) |
| 6 | SubD→B-rep deviation ≤ 0.01 mm, G1 verified numerically, `validate()` passes | 8 cages (cube, box, pentagonal / hexagonal prism, pentagonal / hexagonal / triangular bipyramid, torus) + 3 creased variants: worst deviation **8.9e-3 mm**; every body passes `validate()`; regular surfaces exact (1e-13 mm, G1 1e-6°); at extraordinary vertices the largest tangent-plane mismatch across a smooth edge is **0.41°–1.38°** (mean ≤ 0.014°) | deviation and `validate()` met; **G1 is measured, not exact, at extraordinary vertices** |
| 7 | `.kwf` forward/backward compatibility, 3 schema versions | files written by frozen v1, v2 and v3 writers read to identical models (migration notes checked); a file from a *newer* writer is read, unknown fields and the `cache/` entry survive a re-save; a file whose `min_reader_version` is too new is refused with a message; 400 mutated/truncated files and arbitrary byte strings never panic | met |

Also verified: `cargo fmt --check`, `cargo clippy --workspace --all-targets -- -D warnings`, and the whole workspace test suite (Phase 1–3 tests included) after the kernel changes listed in §5.

## 2. What was built

| Crate | Contents |
|---|---|
| `kw-expr` | Quantities with dimensions (length, angle, mass, time), unit literals (`10 mm`, `0.5in`, `30deg`), `+ - * / ^`, `sin cos tan sqrt min max abs floor ceil round if`, comparison; parse errors with positions, dimension errors that name the dimensions (`cannot add length and dimensionless`); `free_vars()` makes parameter references graph edges. |
| `kw-feature` | The document and engine (below). |
| `kw-sheet` | `SheetStrip` (thickness, K-factor, width, flats and bends), exact B-rep (one extrusion of the thick cross-section), self-intersection check, flat pattern (`BA = θ(R + K t)`), DXF output (`CUT`, `BEND_UP`/`BEND_DOWN`, `BEND_ZONE`), `recognize()` of a strip from a B-rep, `refold()`. |
| `kw-subd` | `SubdMesh` cage (any polygons, validated manifold, creases), Catmull–Clark with DeRose semi-sharp rules, exact limit points, `finish()` → closed B-rep of bicubic patches. |
| `kw-assembly` | Parts with named frames, instances, joints (rigid, revolute, slider, cylindrical, planar) with parameter/expression values and limits, kinematic-tree solver, posed bodies, interference by Boolean, mass, STEP export, storage inside the `.kwf`. |
| `kw-script` | rhai scripting over the builder API (§6). |

### 2.1 `kw-feature`
* **Document** (`doc.rs`, `spec.rs`): an ordered list of `Feature { id, name, suppressed, spec }` with stable `FeatureId(u64)`s. `FeatureSpec` is a serde enum of 20 kinds: Parameter, Sketch, DatumPlane, Extrude, Revolve, Loft, Sweep, Fillet, Chamfer, VariableFillet, Shell, Draft, Offset, Combine, Hole (simple / counterbore / countersink), LinearPattern, CircularPattern, Mirror, SheetMetal, Subd. Every number is a `Param` (literal quantity or expression). Dependencies are derived, never stored: expression free variables → parameter features, explicit ids, face references → the feature that made the name, and the body a feature modifies (previous solid feature, or an explicit `base`; suppressed features pass their input through).
* **Timeline** (`doc.rs`): `add / insert_at / remove / move_feature / replace_spec / rename / set_suppressed / rollback_to / clear_rollback`. Every edit is validated on a copy: a feature that would read something after it → `ForwardReference`; a dependency cycle → `Cycle` with its path (`a → b → a`); unknown or duplicate names; removing a feature others read → `HasDependents`. A refused edit changes nothing. `History` keeps `Arc<Document>` snapshots for undo/redo.
* **Engine** (`engine.rs`): every feature's result is cached under an *input hash* (kernel version, feature id, the specification with all expressions evaluated, the output hashes of everything it reads). Equal key → cache hit, so an edit re-runs exactly what it changes, and a feature that is re-run but produces the same output (a *no-op edit*) leaves all its dependents as cache hits (**early cutoff**). Independent features run in parallel, level by level, on an explicit rayon pool (`Engine::with_threads(n)`); panics inside the kernel become `Failed`; a failing feature suspends its dependents and keeps its last good output; `CancelToken` cancels cooperatively; the cache keeps the last few generations so undo/redo hit it.
* **Executor** (`ops.rs`, `tools.rs`, `namer.rs`, `profile.rs`): maps each spec onto the Phase 3 kernel and turns the kernel's `ShapeHistory` into names. Extrude/Revolve/Hole tools are kept as *recipes* (`ToolGen`), so patterns and mirrors rebuild them at another place and name the faces `Instance { source, index }` — Mirror regenerates the tool because `Transform3` is rigid-only. Sketches are solved with the Phase 2 solver; bound dimensions/positions are applied by a rigid move of the whole sketch to its anchor followed by a gradual walk of the remaining values, which keeps a rectangle on the same side of its corner whatever the numbers (found by the 200-feature test, where plates at x ≥ 50 came out mirrored).
* **Persistent names** (`naming.rs`): a `TopoName` says how a face came to be (`SketchEdge { sketch, entity, role }`, `Primitive`, `Generated`, `Split`, `Merge`, `Instance`), never where it is. A `Reference` is a name plus a `GeomFingerprint` (surface kind, direction, centre, extent, radius, area, neighbour hash). Resolution order: exact name → fragments of the named face (`Split` / `Merge` lineage; a face split into pieces follows the piece with ≥ 1.5× the area of each other piece, otherwise it is *Ambiguous*; a fragment whose cut is gone resolves to the whole face) → fingerprint match, which must beat a score threshold **and** the runner-up by a margin (displacement is measured against the face's own size, so a hole wall never matches the next hole of a pattern). Otherwise `EvalError::Unresolved` carries the candidates. Repair is a headless API: `Document::repair_reference(feature, broken_name, replacement)` and `Evaluation::healed(doc)` (rewrites every reference that was followed by lineage or geometry so it resolves by name from then on).
* **`.kwf`** (`kwf.rs`): a ZIP of `manifest.json` (format, schema version, `min_reader_version`, kernel version, recipe hash) and `recipe.cbor`. Schema 1 = the base document; 2 adds suppression flags and the rollback bar; 3 adds the tilt of datum planes (with the sheet-metal, SubD and assembly entries). Migration is a chain of functions on the decoded tree; unknown fields (and ZIP entries such as `cache/…`, `assembly.cbor`) are preserved by merging the freshly written recipe over the one that was read. Writing is deterministic (same document → same bytes). The reader limits entry sizes and CBOR depth.
* **Determinism**: the geometry hash (`geomhash.rs`) is canonical — independent of identifiers, slot order, parametrisation origin and thread count (1e-7 mm quantisation of surface/curve samples, sorted loops and edges, name hashes) — and the name-set hash is a 128-bit hash of the content of the names.

## 3. Performance

Measured by `tests/perf.rs` (8 threads unless stated):

| model | cold replay | tail edit | no-op edit |
|---|---|---|---|
| 8 parts × (plate + 11 holes/bosses) + 7 joins + 1 parameter = 200 features | 335 ms (1 thread: 1 327 ms) | 52–55 ms (4 features re-executed) | 0 executed, 0.6 ms |
| one chain: plate + 99 hole/boss features (201 features, every Boolean works on a top face with up to 99 holes) | 8.7 s | 192–202 ms | 0 executed, 0.6 ms |

The first row meets the roadmap numbers; the second does not, and the report does not hide that. A 200-feature model made of independent bodies joined at the end uses the parallel levels; one body that grows through 100 Booleans cannot. The cost per Boolean is dominated by the kernel's per-face work on the large faces: each hole boundary is a 3 120-vertex polygon (the 1e-6 deviation Phase 3 chose for robust classification), so a plate with 80 holes spends ~30 ms in setup and ~12 ms in assembly per operation. Two changes this phase made it cheaper (parallel loop sampling for faces with many loops; fewer Gauss panels for analytic faces in mass integration — volume of the 87-face body 37 → 7 ms); a further 4× would need caching converted faces between Booleans, which is kernel work for Phase 5.

## 4. Known gaps and honest caveats

* **Timeline UI**: engine and headless API only, by decision (no egui panel). The kw-app command/event types are not wired to the new API.
* **Naming corpus**: the corpus is generated (box + bosses + holes + pockets + patterns + slots) and its ground truth is analytic geometry, so it measures the resolution policy rather than every kind of model; the thresholds were tuned on this generator (the 1 716-scenario run uses a different seed from the default run). Cases where the generator could not identify exactly one face (84 of 1 800) are skipped and listed in the log. The policies that define "correct": a split face follows the piece with ≥ 1.5× the other's area; two nearly equal pieces are *flagged*; a missing pattern instance must be *flagged* (never matched to a neighbour).
* **Patterns / mirror** repeat the tools of extrude / revolve / hole features (not patterns of patterns, not fillets); a pattern instance that falls off the part is an error that says so, as does a cut tool that misses the body. Hole depth equal to the part thickness works; the tool starts 1 mm above the entry plane.
* **Sheet metal** is the *strip* model: one cross-section (flats and bends of constant width) extruded across the width. It covers L, U, Z, hat, omega and open-hem profiles; it does not cover flanges on several edges of a plate, corner reliefs, cut-outs, or conical/lofted bends. Hems are open hems (gap ≥ 0). `recognize()` reads strips only.
* **SubD**: closed cages only. *Finish* subdivides uniformly (level 1–6, up to 60 000 patches), samples exact limit points, fits one bicubic Bézier patch per face and shares one cubic edge between neighbours (so the body is watertight and `validate()`s). The deviation is measured against exact limit points at twice the fit's sampling density, not by a nearest-point search. Hexagonal prism needs level 6 (36 864 patches, 3.9 s) because the patches around a valence-6 vertex converge only at first order; adaptive refinement with T-junction edges would cut that and is not done. G1 is exact only where the cage is regular; around extraordinary vertices it is approximate (largest tangent-plane mismatch 1.38° at level 5, shrinking with the level; the level is raised until the mismatch is ≤ 1.5° or the deviation target would no longer hold). Semi-sharp creases give a worst mismatch that depends on the sharpness. Volume integration over NURBS faces is slow in the kernel (~2.6 ms per patch).
* **Assemblies**: kinematic trees only (a joint that closes a loop is refused); frames are explicit, there are no geometric mates by face selection; the STEP file is one product with one solid per instance (no nested product structure).
* **`.kwf`**: no cache entries are *written* (the recipe is the only truth, and a file without caches is what the determinism test rebuilds); thumbnails are not written. The roadmap's cargo-fuzz targets are replaced by proptest mutation tests.
* **Scripting** covers the common builder calls (§6), not the whole API.

## 5. Kernel changes made in this phase

* `kw-boolean`: result faces are put in a canonical order (source face, then position of the fragment) before shells and split ordinals are assigned. Without it `SplitFrom` ordinals depended on scheduling, and two thread counts gave different names — found by the 8-part performance model, fixed, and now covered by the determinism test. Faces with many loops are sampled in parallel in `Work::new`.
* `kw-brep` `mass.rs`: planar faces use one Gauss panel (exact for the integrands used), analytic quadrics 1.5 rad instead of 0.6 (error ~1e-20 for 8-point panels).

## 6. Scripting API

```
let d = new_doc();
let w = d.param("width", "40 mm");
let s = d.rect_sketch("Base sketch", "xy", 0, 0, "width", "width / 2");
let base = d.extrude("Base", s, "10 mm", "new");          // new | join | cut | cut_down | join_down | intersect
let hs = d.circle_sketch("Hole sketch", "xy:10", 20, 10, 6);
d.extrude("Hole", hs, 12, "cut_down");
d.linear_pattern("Row", boss, 8, 0, 0, 3);
let r = d.evaluate();            // { ok, evaluated, failures }
let f = d.fillet("F", [[d.side_ref(base, 0), d.side_ref(base, 1)]], 2);
d.sheet("Bracket", "t", 0.44, "t * 1.5", 30, ["F40", "B90", "F25"]);
d.subd("Pillow", "cube", 20, 0.01);
d.set_param(w, "50 mm"); d.suppress(f, true); d.move_to(f, 2); d.rollback(f); d.save("part.kwf");
d
```
Numbers are mm (counts plain); strings are expressions with units. A scripted document equals the document the Rust builder makes and saves to the same bytes (test). Scripts run under operation, depth and size limits and cannot reach the file system (only `save`) or start processes.

## 7. Reproduce

```
scripts/phase4_acceptance.sh                      # all checks (add CORPUS_PER_FAMILY=1000 for 6 000 naming scenarios)
SKIP_WORKSPACE_TESTS=1 scripts/phase4_acceptance.sh
KW_REGEN_FIXTURES=1 cargo test -p kw-feature --test kwf regenerate   # rewrite the checked-in .kwf fixtures
```
