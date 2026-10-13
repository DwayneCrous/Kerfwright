#!/usr/bin/env python3
"""Have OpenCASCADE author a corpus of STEP files (varied solids) plus a manifest of OCCT's own numbers.

usage: occt_make_corpus.py OUT_DIR [COUNT]
Each line of manifest.jsonl: {"file", "volume", "area", "centroid", "faces", "edges", "vertices", "valid"}
"""
import json, sys, os, random, math
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_Vec, gp_Trsf, gp_Ax1, gp_Pln, gp_Circ
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeThickSolid, BRepOffsetAPI_ThruSections, BRepOffsetAPI_MakePipe, BRepOffsetAPI_MakeOffsetShape
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX, TopAbs_SOLID
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as ShapeMap, List_TopoDS_Shape as ShapeList
from OCP.TopoDS import TopoDS
from OCP.ShapeFix import ShapeFix_Shape

def count(shape, kind):
    m = ShapeMap(); TopExp.MapShapes_s(shape, kind, m); return m.Extent()

def transform(shape, rng):
    t = gp_Trsf()
    t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(0.2, 1))), rng.uniform(0, 6.28))
    t2 = gp_Trsf(); t2.SetTranslation(gp_Vec(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5)))
    return BRepBuilderAPI_Transform(shape, t2 * t, True).Shape()

def poly_face(rng, n=None):
    n = n or rng.randint(4, 9)
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        r = rng.uniform(1.0, 2.0)
        pts.append(gp_Pnt(r * math.cos(a), r * math.sin(a), 0))
    mk = BRepBuilderAPI_MakePolygon()
    for p in pts: mk.Add(p)
    mk.Close()
    return BRepBuilderAPI_MakeFace(mk.Wire()).Face()

def make(kind, rng):
    if kind == "box": return BRepPrimAPI_MakeBox(rng.uniform(.5, 4), rng.uniform(.5, 4), rng.uniform(.5, 4)).Shape()
    if kind == "cylinder": return BRepPrimAPI_MakeCylinder(rng.uniform(.4, 2), rng.uniform(.5, 4)).Shape()
    if kind == "cone": return BRepPrimAPI_MakeCone(rng.uniform(.8, 2), rng.uniform(0, .7), rng.uniform(.8, 3)).Shape()
    if kind == "sphere": return BRepPrimAPI_MakeSphere(rng.uniform(.4, 2)).Shape()
    if kind == "torus": return BRepPrimAPI_MakeTorus(rng.uniform(1.5, 3), rng.uniform(.2, .8)).Shape()
    if kind == "prism": return BRepPrimAPI_MakePrism(poly_face(rng), gp_Vec(0, 0, rng.uniform(.5, 3))).Shape()
    if kind == "revolved":
        pts = [(0, 0), (rng.uniform(1, 2), 0), (rng.uniform(1, 2), rng.uniform(.8, 2)), (rng.uniform(.3, .8), rng.uniform(2, 3)), (0, rng.uniform(3, 4))]
        mk = BRepBuilderAPI_MakePolygon()
        for x, z in pts: mk.Add(gp_Pnt(x, 0, z))
        mk.Close()
        face = BRepBuilderAPI_MakeFace(mk.Wire()).Face()
        return BRepPrimAPI_MakeRevol(face, gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))).Shape()
    if kind == "cut":
        a = BRepPrimAPI_MakeBox(3, 3, 3).Shape()
        b = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(rng.uniform(.8, 2.2), rng.uniform(.8, 2.2), -1), gp_Dir(0, 0, 1)), rng.uniform(.3, .7), 5).Shape()
        return BRepAlgoAPI_Cut(a, b).Shape()
    if kind == "fuse":
        a = BRepPrimAPI_MakeSphere(rng.uniform(1.0, 1.6)).Shape()
        b = BRepPrimAPI_MakeBox(gp_Pnt(rng.uniform(0, .6), 0, 0), 2, 2, 2).Shape()
        return BRepAlgoAPI_Fuse(a, b).Shape()
    if kind == "common":
        a = BRepPrimAPI_MakeBox(3, 3, 3).Shape()
        b = BRepPrimAPI_MakeSphere(gp_Pnt(1.5, 1.5, 1.5), rng.uniform(1.4, 2.2)).Shape()
        return BRepAlgoAPI_Common(a, b).Shape()
    if kind == "fillet":
        b = BRepPrimAPI_MakeBox(4, 3, 2).Shape()
        mk = BRepFilletAPI_MakeFillet(b)
        e = TopExp_Explorer(b, TopAbs_EDGE); k = 0
        while e.More() and k < 3:
            mk.Add(rng.uniform(.2, .5), TopoDS.Edge_s(e.Current())); k += 1; e.Next()
        return mk.Shape()
    if kind == "chamfer":
        b = BRepPrimAPI_MakeBox(4, 3, 2).Shape()
        mk = BRepFilletAPI_MakeChamfer(b)
        e = TopExp_Explorer(b, TopAbs_EDGE); k = 0
        while e.More() and k < 3:
            mk.Add(rng.uniform(.2, .5), TopoDS.Edge_s(e.Current())); k += 1; e.Next()
        return mk.Shape()
    if kind == "thick":
        b = BRepPrimAPI_MakeBox(4, 3, 2).Shape()
        e = TopExp_Explorer(b, TopAbs_FACE); f = TopoDS.Face_s(e.Current())
        l = ShapeList(); l.Append(f)
        mk = BRepOffsetAPI_MakeThickSolid(); mk.MakeThickSolidByJoin(b, l, -rng.uniform(.1, .3), 1e-3)
        return mk.Shape()
    if kind == "loft":
        def sq(z, s):
            mk = BRepBuilderAPI_MakePolygon()
            for x, y in [(-s, -s), (s, -s), (s, s), (-s, s)]: mk.Add(gp_Pnt(x, y, z))
            mk.Close(); return mk.Wire()
        mk = BRepOffsetAPI_ThruSections(True, False)
        mk.AddWire(sq(0, rng.uniform(1, 2))); mk.AddWire(sq(rng.uniform(2, 4), rng.uniform(.5, 1.5)))
        mk.Build(); return mk.Shape()
    if kind == "pipe":
        path_pts = [gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 3)]
        spine = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(path_pts[0], path_pts[1]).Edge()).Wire()
        circ = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), rng.uniform(.3, .9))).Edge()).Wire()
        return BRepOffsetAPI_MakePipe(spine, circ).Shape()
    raise ValueError(kind)

KINDS = ["box", "cylinder", "cone", "sphere", "torus", "prism", "revolved", "cut", "fuse", "common", "fillet", "chamfer", "thick", "loft", "pipe"]

def main(out, count_files=200):
    os.makedirs(out, exist_ok=True)
    rng = random.Random(20261009)
    rows = []
    i = 0
    while len(rows) < count_files and i < count_files * 4:
        kind = KINDS[i % len(KINDS)]; i += 1
        try:
            shape = transform(make(kind, rng), rng)
        except Exception as e:
            continue
        if shape.IsNull() or count(shape, TopAbs_SOLID) != 1: continue
        valid = BRepCheck_Analyzer(shape).IsValid()
        if not valid: continue
        name = f"{kind}_{len(rows):03d}.step"
        w = STEPControl_Writer(); w.Transfer(shape, STEPControl_AsIs); w.Write(os.path.join(out, name))
        vp = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, vp)
        sp = GProp_GProps(); BRepGProp.SurfaceProperties_s(shape, sp)
        c = vp.CentreOfMass()
        rows.append({"file": name, "kind": kind, "volume": vp.Mass(), "area": sp.Mass(), "centroid": [c.X(), c.Y(), c.Z()],
                     "faces": count(shape, TopAbs_FACE), "edges": count(shape, TopAbs_EDGE), "vertices": count(shape, TopAbs_VERTEX)})
    with open(os.path.join(out, "manifest.jsonl"), "w") as f:
        for r in rows: f.write(json.dumps(r) + "\n")
    print(f"wrote {len(rows)} OCCT-authored STEP files to {out}")

main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 200)
