#!/usr/bin/env bash
# Checks the Kerfwright application shell and takes a screenshot of every screen.
# Usage: scripts/app_acceptance.sh [out_dir]
#
# The headless tests drive real egui frames with scripted file dialogs; the GPU tests (viewport, render) skip
# themselves without an adapter and must not run under `ulimit -v`. The screenshots need a display.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/app}"
mkdir -p "$OUT"

step() { printf '\n=== %s ===\n' "$*"; }

step "fmt + clippy (app crates)"
cargo fmt --all -- --check
cargo clippy -p kw-app -p kw-sketcher -p kw-viewer -p kw-store --all-targets -- -D warnings

step "tests: shell, projects, workspaces, viewport (release, uncapped for the GPU ones)"
cargo test --release -p kw-app -p kw-sketcher -p kw-viewer -p kw-store 2>&1 | tee "$OUT/tests.log"

step "screenshots of every screen"
cargo build --release -p kw-app
BIN=target/release/kerfwright
shot() { name="$1"; shift; "$BIN" "$@" --screenshot "$OUT/$name.png" --screenshot-after "${AFTER:-40}"; }
shot welcome
shot design_idle --demo
shot design_pick_plane --design-demo pick
shot design_sketch_on_plane --design-demo sketch
shot design_extrude_preview --design-demo extrude
shot design_snap --design-demo snap
shot design_snap_midpoint --design-demo snap-mid
shot design_snap_intersection --design-demo snap-cross
shot design_sketch_on_face --design-demo face
shot design_cut_through_face --design-demo faceextrude
shot design_hole_dialog --design-demo hole
shot design_fillet_dialog --design-demo fillet
shot design_fillet_edges_picked --design-demo fillet-picked
shot design_parameters --design-demo params
shot cam --cam-demo
shot fea --fea-demo
shot ecad --ecad-demo
shot render --render-demo
AFTER=90 shot render_path_traced --render-demo --render-pt

printf '\nApplication checks passed. Screenshots and logs are in %s\n' "$OUT"
