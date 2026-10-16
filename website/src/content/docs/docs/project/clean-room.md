---
title: Provenance and IP policy
description: "The rules on sources, licensing and naming, and the labels used in the research documents."
sidebar:
  order: 2
---

Kerfwright is an independent implementation of well-known CAD, CAM, CAE and ECAD techniques. It is **not** affiliated with, derived from, or endorsed by Autodesk, Siemens, Spatial or any other CAD vendor. The policy lives in [`CLEAN_ROOM.md`](https://github.com/DwayneCrous/Kerfwright/blob/main/CLEAN_ROOM.md) and is summarised here.

## Rules

1. **Allowed inputs:** peer-reviewed papers, textbooks, standards (ISO 10303 STEP, IGES, DXF, Gerber, glTF), vendor-published API and format documentation, expired patents, and permissively licensed or copyleft source *read only for ideas about public algorithms*, never copied.
2. **Forbidden inputs:** decompiled or disassembled binaries, leaked source, reverse-engineered undocumented formats, proprietary SDK headers or libraries, and the behaviour of restricted cloud APIs captured by traffic inspection.
3. **Behavioural reference only.** Commercial CAD may be *used as a user* to learn what a feature is expected to do (workflow, terminology). Layouts, icons, strings and file layouts are not copied.
4. **Copyleft hygiene.** GPL and LGPL solvers and CAM libraries (SolveSpace, PlaneGCS and FreeCAD, OpenCAMLib, LinuxCNC) are studied through their papers and documentation. A contributor who has read the source of one of them for a subsystem says so in the pull request and does not write that subsystem's equivalent.
5. **Patent diligence.** Items flagged in the research documents (T-splines, some adaptive-clearing methods, some fillet and blend methods) need a legal review before they ship. The default is the unencumbered alternative, for example Catmull–Clark subdivision instead of T-splines.
6. **Naming.** No Autodesk or Fusion trademarks (including ViewCube) in crate names, identifiers, interface strings or file extensions. The native format is `.kwf`. Public text may name a vendor only as a plain factual comparison, never with a logo or screenshot.
7. **Provenance.** Every research document ends with a sources section, and every non-trivial algorithm pull request cites what it was implemented from.

## What this policy does not claim

Kerfwright does not claim to be a "clean room" implementation in the legal sense. A clean room needs a documented separation between people who studied a product and people who wrote the code, and nobody can audit what an AI model used in drafting code has seen. What the project relies on instead is the list of forbidden inputs above, a cited source for every non-trivial algorithm, the patent watch list, and the fact that every statement in the research documents is labelled with how it is known.

## Epistemic labels

The research documents mark each statement with how it is known.

| Label | Meaning |
|---|---|
| **[Public]** | Documented in papers, standards or vendor-published documents |
| **[OSS]** | Observable from the published documentation or papers of open-source implementations |
| **[Inferred]** | A plausible engineering inference about proprietary systems, **not verified** |
| **[Design]** | Kerfwright's own decision |

## Patents

`LICENSES/patent-watch.md` lists the items that need counsel's review. Today the notable one is adaptive clearing. The implementation derives its method from public literature and deliberately avoids the medial-axis method of US 9,164,503, but its freedom-to-operate review is not done, so it is withheld from the default build behind the `adaptive-clearing` cargo feature.
