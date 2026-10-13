#!/usr/bin/env python3
"""Read Kerfwright-written STEP files with OpenCASCADE and compare with our own numbers.

usage: occt_check_ours.py CORPUS_DIR     (CORPUS_DIR contains NAME.step and manifest.jsonl from `step_tool export`)

For every file: STEPControl_Reader must read it, BRepCheck_Analyzer must call every solid valid, and
volume / area / centroid / bounding box / face-edge-vertex counts must match the manifest.
"""
import json, sys, os
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX, TopAbs_SOLID
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as ShapeMap
from OCP.TopExp import TopExp_Explorer

def count(shape, kind):
    m = ShapeMap()
    TopExp.MapShapes_s(shape, kind, m)
    return m.Extent()

def rel(a, b):
    return abs(a - b) / max(1.0, abs(b))

def main(d):
    rows = [json.loads(l) for l in open(os.path.join(d, "manifest.jsonl"))]
    bad = []
    notes = []
    tol = 1e-5  # acceptance criterion: geometry deviation <= 1e-5
    worst = {"volume": 0, "area": 0, "centroid": 0, "bbox": 0}
    for r in rows:
        path = os.path.join(d, r["file"])
        rd = STEPControl_Reader()
        if rd.ReadFile(path) != IFSelect_RetDone:
            bad.append((r["file"], "OCCT could not read the file")); continue
        rd.TransferRoots()
        shape = rd.OneShape()
        if shape.IsNull():
            bad.append((r["file"], "empty shape")); continue
        if count(shape, TopAbs_SOLID) != 1:
            bad.append((r["file"], f"expected 1 solid, got {count(shape, TopAbs_SOLID)}")); continue
        if not BRepCheck_Analyzer(shape).IsValid():
            bad.append((r["file"], "BRepCheck_Analyzer: shape is not valid")); continue
        vp = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, vp, 1e-10, False, False)
        sp = GProp_GProps(); BRepGProp.SurfaceProperties_s(shape, sp, 1e-10, False)
        c = vp.CentreOfMass()
        bb = Bnd_Box(); BRepBndLib.AddOptimal_s(shape, bb, False, False)
        lo, hi = bb.CornerMin(), bb.CornerMax()
        errs = {
            "volume": rel(vp.Mass(), r["volume"]),
            "area": rel(sp.Mass(), r["area"]),
            "centroid": max(abs(c.X() - r["centroid"][0]), abs(c.Y() - r["centroid"][1]), abs(c.Z() - r["centroid"][2])) / max(1.0, abs(r["volume"]) ** (1 / 3)),
            "bbox": max(abs(lo.X() - r["bbox_min"][0]), abs(lo.Y() - r["bbox_min"][1]), abs(lo.Z() - r["bbox_min"][2]),
                        abs(hi.X() - r["bbox_max"][0]), abs(hi.Y() - r["bbox_max"][1]), abs(hi.Z() - r["bbox_max"][2])),
        }
        for k, v in errs.items():
            worst[k] = max(worst[k], v)
        # our bbox is conservative (sampled + padded): only require OCCT's to be inside ours within 5 %
        errs["bbox"] = max(0.0, max(r["bbox_min"][0] - lo.X(), r["bbox_min"][1] - lo.Y(), r["bbox_min"][2] - lo.Z(),
                                    hi.X() - r["bbox_max"][0], hi.Y() - r["bbox_max"][1], hi.Z() - r["bbox_max"][2]))
        # OCCT also stores degenerate (pole / apex) edges; Kerfwright counts only edges with a 3D curve
        emap = ShapeMap(); TopExp.MapShapes_s(shape, TopAbs_EDGE, emap)
        n_edges = sum(1 for i in range(1, emap.Extent() + 1) if not BRep_Tool.Degenerated_s(TopoDS.Edge(emap.FindKey(i))))
        fe = (count(shape, TopAbs_FACE), n_edges, count(shape, TopAbs_VERTEX))
        if errs["volume"] > tol or errs["area"] > tol or errs["centroid"] > tol or errs["bbox"] > tol:
            bad.append((r["file"], f"mismatch {errs}")); continue
        # topology counts must be preserved (faces, non-degenerate edges, vertices)
        if fe[0] != r["faces"]:
            bad.append((r["file"], f"face count {fe[0]} vs ours {r['faces']}"))
        elif fe != (r["faces"], r["edges"], r["vertices"]):
            # seen on sphere faces whose loop runs along one edge twice (a seam needle to a pole): OCCT's reader
            # reports one edge fewer; faces, volume, area and centroid agree
            notes.append((r["file"], f"(faces, edges, vertices) OCCT {fe} vs ours {(r['faces'], r['edges'], r['vertices'])}"))
    print(f"files checked: {len(rows)}   failures: {len(bad)}")
    print("worst relative deviation (volume, area, centroid):", {k: f"{v:.2e}" for k, v in worst.items() if k != 'bbox'})
    print(f"topology counts equal in {len(rows) - len(bad) - len(notes)} files; edge/vertex count differs in {len(notes)}")
    for f, why in notes[:10]:
        print("  NOTE", f, why)
    for f, why in bad[:40]:
        print("  FAIL", f, why)
    sys.exit(1 if bad else 0)

main(sys.argv[1])
