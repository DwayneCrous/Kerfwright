#!/usr/bin/env bash
# Runs every automated Phase 2 (sketcher + constraint solver) acceptance check.
# Usage: scripts/phase2_acceptance.sh [out_dir]
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/phase2}"
mkdir -p "$OUT"

step() { printf '\n=== %s ===\n' "$*"; }

step "fmt + clippy"
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

step "unit + integration tests"
cargo test --workspace

step "(1) Jacobians vs finite differences AND forward-mode AD, 1,000,000 cases (release)"
PROPTEST_CASES=1000000 cargo test -p kw-sketch --release --test jacobians

step "(3) classification vs ground truth (>= 300 cases) + minimal conflict sets (release)"
cargo test -p kw-sketch --release --test classification -- --nocapture

step "(4) no branch flip in scripted drags of 100 reference sketches (release)"
cargo test -p kw-sketch --release --test branch_flip -- --nocapture

step "(5) bit-identical results for 1, 2, 4, 8, 16, 32 threads (release)"
cargo test -p kw-sketch --release --test determinism

step "(2) performance SLAs: drag p99, cold 500 / 5000 entities (release)"
cargo test -p kw-sketch --release --test perf_sla -- --nocapture --test-threads=1 | tee "$OUT/perf_sla.log"

step "(6) DXF round trip 1e-6 + independent 'dxf' parser, 20,000 cases (release)"
PROPTEST_CASES=20000 cargo test -p kw-dxf --release

step "sketch ops + editor logic + egui front end (headless UI events)"
cargo test -p kw-sketch --release --test ops --test editor --test solve_basic
cargo test -p kw-sketcher --release

step "kw-sketcher smoke run (opens a window; skipped without a display)"
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  cargo run --release -p kw-sketcher -- --demo --exit-after-frames 300
else
  echo "no DISPLAY / WAYLAND_DISPLAY: skipped"
fi

step "criterion benchmarks (quick)"
cargo bench -p kw-sketch -- --quick | tee "$OUT/bench.log"

echo
echo "All Phase 2 automated checks passed."
