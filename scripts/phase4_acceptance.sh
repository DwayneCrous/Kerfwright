#!/usr/bin/env bash
# Runs every automated Phase 4 (parametric engine, assemblies, sheet metal, SubD, scripting) acceptance check.
# Usage: scripts/phase4_acceptance.sh [out_dir]
#
# Sizes can be raised or reduced, e.g.
#   CORPUS_PER_FAMILY=1000 scripts/phase4_acceptance.sh        # 6 families x 1000 naming scenarios
#   SKIP_WORKSPACE_TESTS=1 scripts/phase4_acceptance.sh        # only the Phase 4 checks
#
# Heavy runs are memory-capped (ulimit -v) and use at most 8 threads.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/phase4}"
mkdir -p "$OUT"
CORPUS_PER_FAMILY="${CORPUS_PER_FAMILY:-300}"
export RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-8}"
ulimit -v "${VMEM_KB:-16000000}" || true

step() { printf '\n=== %s ===\n' "$*"; }

step "fmt + clippy"
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

if [ -z "${SKIP_WORKSPACE_TESTS:-}" ]; then
  step "unit + integration tests, whole workspace (release)"
  cargo test --workspace --release
fi

step "(1) expressions with units: parse / evaluate / dimension errors"
cargo test -p kw-expr --release

step "(2) incremental engine: caching, early cutoff, failures, rollback, suppress, undo, cancel (release)"
cargo test -p kw-feature --release --test engine --test kernel --test features

step "(3) topological naming corpus: ${CORPUS_PER_FAMILY} scenarios per family x 6 families, 0 silent wrong resolutions, >= 97 % correct"
KW_CORPUS_PER_FAMILY="$CORPUS_PER_FAMILY" cargo test -p kw-feature --release --test naming_corpus -- --nocapture | tee "$OUT/naming_corpus.log"

step "(4) determinism: rebuild from .kwf with no cache on 1 / 4 / 32 threads gives identical geometry hash and TopoName set"
cargo test -p kw-feature --release --test determinism -- --nocapture --test-threads=1 | tee "$OUT/determinism.log"

step "(5) timeline: cycle / forward-reference edits always rejected with an explanation; repair of a broken reference"
cargo test -p kw-feature --release --test timeline -- --nocapture | tee "$OUT/timeline.log"

step "(6) performance: 200-feature models (tail edit, cold replay, no-op edit)"
cargo test -p kw-feature --release --test perf -- --nocapture --test-threads=1 | tee "$OUT/perf.log"

step "(7) .kwf: round trip, three schema versions both ways, newer-writer files, damaged input"
cargo test -p kw-feature --release --test kwf -- --nocapture | tee "$OUT/kwf.log"

step "(8) sheet metal: analytic volume, bend allowance measured from the DXF flat pattern, unfold -> refold"
cargo test -p kw-sheet --release -- --nocapture --test-threads=1 | tee "$OUT/sheet.log"

step "(9) SubD: Finish -> B-rep deviation, G1, validate(), creases"
cargo test -p kw-subd --release -- --nocapture --test-threads=1 | tee "$OUT/subd.log"

step "(10) assemblies: joints, limits, loops refused, interference, STEP, .kwf storage"
cargo test -p kw-assembly --release -- --nocapture

step "(11) scripting: scripted document == builder document, errors, sandbox"
cargo test -p kw-script --release -- --nocapture

printf '\nAll Phase 4 checks passed. Logs are in %s\n' "$OUT"
