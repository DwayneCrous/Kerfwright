#!/usr/bin/env python3
"""Import a directory of STEP files with Kerfwright and report the valid-solid rate, failure classes
and (when the directory has a manifest.jsonl written by OpenCASCADE) deviation from OCCT's numbers.

usage: import_corpus.py <dir> [tol=1e-5]
The kernel's own import already runs BRep::validate() on every solid; "valid" means validate() is clean.
"""
import collections, glob, json, os, re, subprocess, sys

d = sys.argv[1]
tol = float(sys.argv[2]) if len(sys.argv) > 2 else 1e-5
tool = os.path.join(os.path.dirname(__file__), "../../target/release/examples/step_tool")
man = {}
mp = os.path.join(d, "manifest.jsonl")
if os.path.exists(mp):
    for line in open(mp):
        m = json.loads(line)
        man[m["file"]] = m
files = sorted(glob.glob(os.path.join(d, "**", "*.st*p"), recursive=True) + glob.glob(os.path.join(d, "**", "*.STEP"), recursive=True))
files = sorted(set(files))
valid = 0
classes = collections.Counter()
examples = {}
worst = 0.0
dev_fail = []
total_ms = 0.0
for f in files:
    r = subprocess.run([tool, "import", f], capture_output=True, text=True, timeout=600)
    try:
        j = json.loads(r.stdout.splitlines()[-1])
    except Exception:  # noqa: BLE001
        classes["tool crashed"] += 1
        examples.setdefault("tool crashed", os.path.basename(f))
        continue
    total_ms += j.get("ms", 0)
    if j["status"] == "ok" and j["bodies"] and not j.get("failed"):
        valid += 1
        name = os.path.basename(f)
        if name in man and len(j["bodies"]) == 1:
            m, b = man[name], j["bodies"][0]
            dv = abs(b["volume"] - m["volume"]) / max(abs(m["volume"]), 1e-12)
            da = abs(b["area"] - m["area"]) / max(m["area"], 1e-12)
            ext = max(max(abs(x) for x in m["centroid"]), 1.0)
            dc = max(abs(b["centroid"][i] - m["centroid"][i]) for i in range(3)) / ext
            worst = max(worst, dv, da, dc)
            if max(dv, da, dc) > tol or b["faces"] != m["faces"]:
                dev_fail.append((name, dv, da, dc, b["faces"], m["faces"]))
    else:
        detail = j.get("detail") or str(j.get("failed"))
        key = re.sub(r"[#\d.\-e+]+", "N", detail)[:110]
        classes[key] += 1
        examples.setdefault(key, os.path.basename(f))
n = len(files)
print(f"files: {n}  imported as valid solids: {valid} ({100.0 * valid / max(n, 1):.1f}%)  total import time {total_ms/1000:.1f}s")
if man:
    print(f"compared with OCCT: worst deviation {worst:.2e}; {len(dev_fail)} files outside {tol:g} or with a different face count")
    for x in dev_fail[:15]:
        print("  DEV", x)
for k, v in classes.most_common(15):
    print(f"  {v:4d}  {k}   (e.g. {examples[k]})")
