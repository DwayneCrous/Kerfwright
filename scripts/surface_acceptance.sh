#!/usr/bin/env bash
# Checks the surface modelling and analysis work and takes a screenshot of each stage.
# Usage: scripts/surface_acceptance.sh [out_dir]
#
# The GPU tests (kw-viewer) skip themselves without an adapter; the screenshots need a display.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/surface}"
mkdir -p "$OUT"

step() { printf '\n=== %s ===\n' "$*"; }

step "fmt + clippy (whole workspace)"
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

step "tests: kernel, features, analysis, application"
cargo test --release -p kw-nurbs -p kw-brep -p kw-tess -p kw-model -p kw-feature -p kw-analysis -p kw-viewer -p kw-app 2>&1 | tee "$OUT/tests.log"

step "regression: the importers, Booleans and drawings still pass"
cargo test --release -p kw-iges -p kw-boolean -p kw-drawing 2>&1 | tee "$OUT/regression.log"

step "screenshots of each stage"
cargo build --release -p kw-app
BIN=target/release/kerfwright
shot() { name="$1"; shift; "$BIN" "$@" --screenshot "$OUT/$name.png" --screenshot-after "${AFTER:-40}"; }
shot zebra --design-demo zebra
shot curvature --design-demo curvature
shot draft --design-demo draft
shot surface_loft --design-demo surface
shot surface_zebra --design-demo surface-zebra
shot surface_patch --design-demo patch
shot surface_patch_zebra --design-demo patch-zebra
shot surface_thicken --design-demo thicken
shot surface_trim --design-demo trim
shot surface_continuity --design-demo continuity
echo "screenshots in $OUT"
