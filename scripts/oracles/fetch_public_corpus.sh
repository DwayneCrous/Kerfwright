#!/usr/bin/env bash
# Download a reproducible sample of public STEP files (FreeCAD-library, ~17 MB for 260 files) into OUT_DIR.
# The files are NOT part of this repository (they carry their own licences); only the selection rule is.
#
# Selection: every k-th .step/.stp file (sorted by path) of at most 600 kB, k chosen to give ~260 files.
# usage: fetch_public_corpus.sh OUT_DIR
set -euo pipefail
OUT="${1:?usage: fetch_public_corpus.sh OUT_DIR}"
mkdir -p "$OUT"
TREE="$OUT/.tree.json"
curl -s -m 120 "https://api.github.com/repos/FreeCAD/FreeCAD-library/git/trees/HEAD?recursive=1" -o "$TREE"
python3 - "$TREE" "$OUT/.selection.txt" <<'PY'
import json, sys
t = json.load(open(sys.argv[1]))
f = sorted((x for x in t["tree"] if x["path"].lower().endswith((".step", ".stp")) and x.get("size", 0) <= 600_000), key=lambda x: x["path"])
step = max(1, len(f) // 260)
sel = f[::step][:260]
open(sys.argv[2], "w").write("\n".join(x["path"] for x in sel))
print(f"{len(f)} candidate files, selecting {len(sel)} ({sum(x['size'] for x in sel) / 1e6:.1f} MB)")
PY
i=0
while IFS= read -r p; do
  i=$((i + 1))
  n=$(printf "%03d" "$i")
  enc=$(python3 -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$p")
  curl -s -m 60 -L -o "$OUT/${n}_$(basename "$p" | tr ' ' '_')" "https://raw.githubusercontent.com/FreeCAD/FreeCAD-library/HEAD/$enc" || echo "failed: $p"
done < "$OUT/.selection.txt"
ls "$OUT" | wc -l
