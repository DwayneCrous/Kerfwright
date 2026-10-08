# 03 — Parametric Dependency Graph & Topological Naming

## 1. The Topological Naming Problem (TNP)
A downstream feature refers to "that face / that edge" (fillet this edge, sketch on that face). When an upstream parameter changes, the kernel rebuilds the body and its entity numbering/ordering may change, so the reference silently points to the wrong entity or none. Not unique to any one CAD system; commercial systems mitigate it with heuristics. [Public]

### Approaches in the literature and in products
| Approach | Idea | Weakness |
|---|---|---|
| Index-based (`Face12`) | Position in kernel's list | Breaks on any upstream change (classic FreeCAD ≤0.19 behaviour) |
| **History / generation-based names** (Kripac 1995; Capoyleas–Chen–Hoffmann 1996) | Name = *how the entity was created* (feature + operation + source entity names) | Needs the kernel to report modify/split/merge history; ambiguity when one op yields many look-alike entities |
| **Geometric/semantic matching** (Bidarra & Bronsvoort; Marcheix & Pierra) | Match by surface type, normal, area, centroid, adjacency | Fragile for symmetric parts; can pick wrong twin |
| **Query-based selection** (Onshape FeatureScript queries) | Store a *query* ("faces created by extrude 3, with normal +Z") rather than an ID, re-evaluate each rebuild; Parasolid persistent IDs underneath | Query authoring is user-visible complexity [Public] |
| **FreeCAD 1.0 element map** (realthunder) | Each shape carries a map from stable *mapped names* to elements; mapped names encode operation + source-name lineage; used throughout Part/PartDesign [OSS/Public] | Names can grow long; still needs geometric fallback in edge cases |

## 2. Kerfwright's design **[Design]** (hybrid: lineage names + geometric fallback + explicit repair)

### 2.1 Kernel contract: history reporting
Every kernel operation returns, alongside the new body, a **`ShapeHistory`** (mirroring the public OpenCASCADE model `Modified / Generated / IsDeleted` and Parasolid's tracking):

```
output entity  ──origin──►  Preserved(input) | Modified(input) | Generated{from: input, role}
                          | SplitFrom(input, k) | MergedFrom({inputs}) | New(role)
input entity   ──fate────►  Deleted
```
Without this, naming cannot be exact. Kernel ops must be *deterministic*: no iteration order that depends on hash randomisation, thread scheduling, or pointer addresses; ties are broken by a canonical geometric key (e.g. rounded centroid, then area).

### 2.2 Persistent names
```
TopoName {
    feature: FeatureId,          // which feature created this entity
    origin:  NameOrigin,         // Generated{from: Box<TopoName>, role} | SketchEdge(SketchEntityId, Role) | Split{parent, ordinal} | Merge(sorted children) | Primitive(role)
    ordinal: u32,                // canonical-order disambiguator among siblings
}
```
* Names for sketch-derived geometry are built from **persistent sketch entity IDs** (not array indices): "side face of extrude E from profile segment S" is stable as long as S exists.
* Each evaluated feature publishes a `NameMap: TopoName → EntityKey` (the generational key valid for that evaluation). Maps are rebuilt each evaluation; the *names* are what is stored in the document.
* Names are hashed (stable 128-bit hash of the structured name) for compact storage and equality.

### 2.3 References
A downstream feature stores a `Reference`:
```
Reference { name: TopoName, fingerprint: GeomFingerprint, tolerance: f64 }
GeomFingerprint { kind, normal/axis, centroid, area_or_length, adjacency_signature }
```
Resolution order:
1. **Exact** name lookup in the upstream NameMap.
2. **Lineage walk**: if the named entity was split, resolve to the set of fragments (feature declares whether it accepts one-to-many); if merged, to the merged entity; if deleted, mark broken.
3. **Geometric match** on candidates (same feature neighbourhood) scored by fingerprint; accept only if best score ≥ threshold *and* margin to runner-up ≥ delta.
4. Otherwise `ResolveOutcome::Ambiguous{candidates}` or `Broken` — the feature goes to `Failed` state; the UI offers a **repair** picker. We never silently guess.

### 2.4 Testing strategy (this is where TNP implementations die)
Corpus of "parametric edit" scenarios: change a dimension, reorder features, insert an upstream feature, mirror/pattern changes, split a face with a cut, merge faces. Property: *the downstream fillet still lands on the same physical edge, or the reference reports broken—never a wrong edge.*

## 3. Feature DAG architecture

### 3.1 Model
* **Nodes**: `Parameter`, `Sketch`, `Feature` (Extrude, Revolve, Sweep, Loft, Fillet, Chamfer, Shell, Draft, Boolean, Pattern, Mirror, SheetMetal*, FormFinish, …), `Body` (output handle), `Analysis` (FEA/CAM setups consume bodies).
* **Edges**: *data* dependencies — feature → the features/bodies/sketches/parameters it reads. The **timeline** is a user-visible *linearisation* (a topological order plus a rollback marker), but evaluation uses the DAG so independent branches parallelise and reordering is validated by cycle detection.
* Expression language for parameters with units (`d1 = 2 * width + 3 mm`); expression references are edges too.

### 3.2 Evaluation: incremental, cached, early-cutoff
Inspired by build systems and incremental-computation frameworks (Salsa, Adapton, Bazel):
1. Each node has `input_hash = H(kernel_version, node_params, [output_hash of each input])`.
2. Output cache: `input_hash → Arc<EvalResult>` (immutable B-rep + NameMap + ShapeHistory + tessellation handle). Structural sharing: unmodified bodies are `Arc` clones; B-rep arenas are copy-on-write by chunk.
3. **Dirty propagation**: editing node N marks N and its transitive dependents (DFS over out-edges) dirty.
4. **Evaluate in topological order**; for each dirty node recompute `input_hash`; hit in cache → reuse; miss → evaluate. **Early cutoff**: if the new `output_hash` equals the previous one (e.g. a no-op parameter tweak, or a change that doesn't alter the geometry), do not dirty its dependents.
5. Independent subtrees evaluated in parallel on the Rayon pool; each node's kernel call is itself internally parallel only at coarse grain, to avoid oversubscription.
6. **Rollback bar**: nodes after the marker are `Suspended` (not evaluated) but retain their last-good cache for instant roll-forward.
7. **Failure**: node → `Failed(reason, location)`; dependents → `Suspended`; last-good geometry remains displayed greyed. Errors never corrupt downstream caches.
8. Cancellation: each evaluation carries a generation token; a new edit supersedes in-flight work (`AtomicU64` compare).

### 3.3 Document state
* Immutable-snapshot document (`Arc<DocState>`) with a persistent (structural-sharing) map for nodes; undo/redo = pointer to an earlier snapshot + command log. Reads never block writes.
* On-disk `.kwf`: recipe (features, parameters, names, references) + *optional* cached B-rep/tessellation for fast open; a file without caches must still rebuild identically.
* Future collaboration (Phase 6): the recipe is a CRDT-friendly structure (ordered list of nodes with stable IDs; parameters as LWW registers; reorder as move-ops).

## 4. Interaction of naming and caching
Cache keys include the *names consumed* by the node, so a changed upstream NameMap invalidates dependents even if geometry is identical. Early cutoff compares `(geometry_hash, namemap_hash)`.

## Sources
- Kripac, "A mechanism for persistently naming topological entities in history-based parametric solid models," CAD 29(2), 1997.
- Capoyleas, Chen, Hoffmann, "Generic naming in generative, constraint-based design," CAD 28(1), 1996.
- Bidarra & Bronsvoort, "Semantic feature modelling," CAD 32(3), 2000.
- FreeCAD wiki, Topological naming problem — https://wiki.freecad.org/Topological_naming_problem
- Ondsel blog on the FreeCAD fix — https://ondsel.com/blog/freecad-topological-naming/
- Onshape FeatureScript query documentation (public).
- Salsa incremental computation (Rust) — https://github.com/salsa-rs/salsa ; Adapton (Hammer et al., PLDI 2014).
- OpenCASCADE `BRepBuilderAPI_MakeShape` history API (public docs).
