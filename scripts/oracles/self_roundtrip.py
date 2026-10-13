#!/usr/bin/env python3
"""Re-import every STEP file written by `step_tool export <dir>` with Kerfwright's own importer and
compare volume / area / centroid / topology counts (the kernel's bbox is a conservative estimate that depends on
the face parameterisation, so it is not compared; the OCCT check compares exact bboxes) with the manifest written at export time.

usage: self_roundtrip.py <dir> [tol=1e-5]
"""
import json, subprocess, sys, os

d = sys.argv[1]
tol = float(sys.argv[2]) if len(sys.argv) > 2 else 1e-5
tool = os.path.join(os.path.dirname(__file__), "../../target/release/examples/step_tool")
man = {}
for line in open(os.path.join(d, "manifest.jsonl")):
    m = json.loads(line)
    man[m["file"]] = m
bad, worst = [], 0.0
for name, m in sorted(man.items()):
    out = subprocess.run([tool, "import", os.path.join(d, name)], capture_output=True, text=True).stdout
    r = json.loads(out)
    if r["status"] != "ok" or len(r["bodies"]) != 1:
        bad.append((name, r.get("detail", "bodies=%d" % len(r.get("bodies", [])))))
        continue
    b = r["bodies"][0]
    scale = max(abs(m["volume"]), 1e-12)
    ext = max(max(m["bbox_max"][i] - m["bbox_min"][i] for i in range(3)), 1e-12)
    dev = {
        "volume": abs(b["volume"] - m["volume"]) / scale,
        "area": abs(b["area"] - m["area"]) / max(m["area"], 1e-12),
        "centroid": max(abs(b["centroid"][i] - m["centroid"][i]) for i in range(3)) / ext,
    }
    worst = max(worst, *dev.values())
    topo = all(b[k] == m[k] for k in ("faces", "edges", "vertices"))
    if max(dev.values()) > tol or not topo:
        bad.append((name, dict(dev, topo=topo, ours=(b["faces"], b["edges"], b["vertices"]), exp=(m["faces"], m["edges"], m["vertices"]))))
print(f"files: {len(man)}  failures: {len(bad)}  worst deviation: {worst:.2e}")
for n, w in bad[:30]:
    print("FAIL", n, w)
sys.exit(1 if bad else 0)
