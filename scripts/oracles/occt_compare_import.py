#!/usr/bin/env python3
"""For every STEP file in a directory: read it with OpenCASCADE and with Kerfwright's importer and compare
solid count, total volume and total area (the geometric check behind the public-corpus validity rate).

usage: occt_compare_import.py <dir> [tol=1e-5]
"""
import glob, json, os, subprocess, sys
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS

d = sys.argv[1]
tol = float(sys.argv[2]) if len(sys.argv) > 2 else 1e-5
tool = os.path.join(os.path.dirname(__file__), "../../target/release/examples/step_tool")
files = sorted(set(glob.glob(os.path.join(d, "*.st*p")) + glob.glob(os.path.join(d, "*.STEP"))))
rows, skipped = [], []
for f in files:
    r = subprocess.run([tool, "import", f], capture_output=True, text=True, timeout=600)
    j = json.loads(r.stdout.splitlines()[-1])
    if j["status"] != "ok" or not j["bodies"] or j.get("failed"):
        continue
    rd = STEPControl_Reader()
    if rd.ReadFile(f) != IFSelect_RetDone:
        skipped.append((os.path.basename(f), "OCCT could not read"))
        continue
    rd.TransferRoots()
    shape = rd.OneShape()
    vol = area = 0.0
    n = 0
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        s = TopoDS.Solid(ex.Current())
        vp = GProp_GProps(); BRepGProp.VolumeProperties_s(s, vp, 1e-9, False, False)
        sp = GProp_GProps(); BRepGProp.SurfaceProperties_s(s, sp, 1e-9, False)
        vol += abs(vp.Mass()); area += sp.Mass(); n += 1
        ex.Next()
    ov = sum(abs(b["volume"]) for b in j["bodies"])
    oa = sum(b["area"] for b in j["bodies"])
    dv = abs(ov - vol) / max(vol, 1e-12)
    da = abs(oa - area) / max(area, 1e-12)
    rows.append((os.path.basename(f), n, len(j["bodies"]), dv, da))
bad = [r for r in rows if r[3] > tol or r[4] > tol]
print(f"compared {len(rows)} files that Kerfwright imported; {len(rows) - len(bad)} agree with OCCT within {tol:g} (volume and area)")
print(f"solid-count differences: {sum(1 for r in rows if r[1] != r[2])}")
import statistics
if rows:
    print("median dev vol %.2e  area %.2e   max vol %.2e  area %.2e" % (statistics.median(r[3] for r in rows), statistics.median(r[4] for r in rows), max(r[3] for r in rows), max(r[4] for r in rows)))
for r in sorted(bad, key=lambda r: -max(r[3], r[4]))[:25]:
    print("  DEV %-60s solids occt=%d ours=%d vol %.2e area %.2e" % r)
for s in skipped:
    print("  SKIP", *s)
