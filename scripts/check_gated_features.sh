#!/usr/bin/env bash
# Fails if a feature withheld by LICENSES/patent-watch.md is switched on by default.
set -euo pipefail
cd "$(dirname "$0")/.."
gated=(adaptive-clearing vertex-blend)
for f in "${gated[@]}"; do
  crate=kw-cam; [ "$f" = vertex-blend ] && crate=kw-blend
  if cargo tree --workspace -e features -i "$crate" 2>/dev/null | grep -q "$crate feature \"$f\""; then
    echo "error: '$f' is enabled in the default build, but is listed as open in LICENSES/patent-watch.md" >&2
    exit 1
  fi
done
echo "ok: no patent-watch feature is on by default"
