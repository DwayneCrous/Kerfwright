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
shot() { name="$1"; shift; "$BIN" "$@" --screenshot "$OUT/$name.png"; }
shot welcome
shot model --demo
shot sketch --demo --edit-sketch
shot cam --cam-demo
shot fea --fea-demo
shot ecad --ecad-demo
shot render --render-demo
shot render_path_traced --render-demo --render-pt --screenshot-after 90

printf '\nApplication checks passed. Screenshots and logs are in %s\n' "$OUT"
