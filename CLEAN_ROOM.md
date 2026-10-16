# Provenance and IP Policy

Kerfwright is an independent implementation of well-known CAD/CAM/CAE techniques. It is **not** affiliated with, derived from, or endorsed by Autodesk, Siemens, Spatial, or any other CAD vendor.

This is not a "clean room" in the legal sense (no documented separation between people who studied a product and people who wrote the code), and we do not describe it that way. Some code is drafted with AI assistance, and nobody can audit what such a model has seen. The policy therefore rests on the forbidden-inputs list, a cited source for every non-trivial algorithm, the patent watch list, and the epistemic labels below. (The file keeps its old name so links keep working.)

## Rules

1. **Inputs allowed:** peer-reviewed papers, textbooks, standards (ISO 10303 STEP, IGES, DXF, Gerber, glTF), vendor-published API/format documentation, expired patents, and permissively licensed or copyleft source *read only for ideas about public algorithms* (never copy code).
2. **Inputs forbidden:** decompiled/disassembled binaries, leaked source, reverse-engineered undocumented file formats, proprietary SDK headers/libraries, ToS-restricted cloud API behaviour captured by traffic inspection.
3. **Behavioural reference only:** we may *use* commercial CAD as a user to learn what a feature is expected to do (workflow, terminology). We do not copy UI layouts pixel-for-pixel, icons, strings, or file layouts.
4. **Copyleft hygiene:** GPL/LGPL solvers and CAM libraries (SolveSpace, PlaneGCS/FreeCAD, OpenCAMLib, LinuxCNC) are studied through their *papers and documentation*. Contributors who have read their source for a given subsystem should say so in the PR and not author that subsystem's equivalent.
5. **Patent diligence:** items flagged ⚠ in `docs/01-research/` (T-splines, some adaptive-clearing methods, some fillet/blend methods) need a legal review before they ship. Default to the unencumbered alternative (e.g. Catmull-Clark SubD, trochoidal/engagement-angle clearing derived from public literature).
6. **Naming:** no Autodesk/Fusion trademarks (including ViewCube) in crate names, identifiers, UI strings, or file extensions. Native format is `.kwf`. Public text (README, website, ads) may name a vendor only as a plain factual comparison ("alternative to"), never with a logo or screenshot. See `TRADEMARKS.md`.
7. **Provenance log:** every research doc ends with a Sources section; every non-trivial algorithm PR cites the source it was implemented from.

## Epistemic labels used in the docs

| Label | Meaning |
|---|---|
| **[Public]** | Documented in papers, standards, or vendor-published docs. |
| **[OSS]** | Observable from open-source implementations' published docs/papers. |
| **[Inferred]** | Plausible engineering inference about proprietary systems; **not verified**. |
| **[Design]** | Kerfwright's own decision. |
