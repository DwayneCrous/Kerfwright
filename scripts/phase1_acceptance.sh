#!/usr/bin/env bash
# Runs every automated Phase 1 acceptance check and prints a summary.
# Usage: scripts/phase1_acceptance.sh [out_dir]
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/phase1}"
mkdir -p "$OUT"

step() { printf '\n=== %s ===\n' "$*"; }

step "fmt + clippy"
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

step "unit + integration tests"
cargo test --workspace

step "NURBS properties, 1,000,000 cases each (release)"
PROPTEST_CASES=1000000 cargo test -p kw-nurbs --test properties --release

step "topology mutation fuzz, 1,000,000 cases each (release)"
PROPTEST_CASES=1000000 cargo test -p kw-brep --test fuzz --release

step "tessellation corpus: crack-free + chordal tolerance"
cargo test -p kw-tess --release --test primitives

step "exporters: STL / OBJ / glTF / GLB round trips"
cargo test -p kw-io --release
cargo run --release -p kw-io --example export_primitives -- "$OUT/export" 0.01

step "viewer: pick-buffer test (headless)"
cargo run --release -p kw-viewer -- --headless --scene primitives --pick-test --pick-samples 20000

step "viewer: frame time, 1M triangles @1920x1080 (headless, GPU-inclusive)"
cargo run --release -p kw-viewer -- --headless --scene million --frames 600 --screenshot "$OUT/million.png"

step "criterion benchmarks (quick)"
cargo bench --workspace -- --quick

echo
echo "All Phase 1 automated checks passed."
