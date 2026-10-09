#!/usr/bin/env bash
# Runs every automated Phase 3 (B-rep modelling kernel) acceptance check.
# Usage: scripts/phase3_acceptance.sh [out_dir]
#
# Sizes can be reduced for a quick run, e.g.
#   BOOL_PAIRS=2000 SSI_PAIRS=2000 BLEND_PARTS=100 scripts/phase3_acceptance.sh
#
# External-tool checks (OpenCASCADE / Gmsh / truck, public STEP corpus) need the dev-only oracle
# environment described in docs/phase3-report.md; they run when ORACLE_PY (a python with cadquery-ocp and
# gmsh installed) and ORACLE_CORPUS (a directory of public .step files) are set, and are skipped otherwise.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/phase3}"
mkdir -p "$OUT"
BOOL_PAIRS="${BOOL_PAIRS:-100000}"
SSI_PAIRS="${SSI_PAIRS:-10000}"
BLEND_PARTS="${BLEND_PARTS:-500}"
MODEL_PAIRS="${MODEL_PAIRS:-2000}"
CORNER_PARTS="${CORNER_PARTS:-200}"

step() { printf '\n=== %s ===\n' "$*"; }

step "fmt + clippy"
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

step "unit + integration tests"
cargo test --workspace

step "(3) SSI: labelled branch suite + ${SSI_PAIRS} analytic / spline pairs, off-surface <= tol/10 (release)"
SSI_PAIRS="$SSI_PAIRS" cargo test -p kw-ssi --release -- --nocapture | tee "$OUT/ssi.log"

step "(2) Boolean inclusion-exclusion volumes (1e-9) and closed-form volumes (release)"
cargo test -p kw-boolean --release --test primitives --test basic

step "(1) Boolean fuzz: ${BOOL_PAIRS} pairs x 3 operations, 0 invalid outputs, success rate on valid-expected cases (release)"
KW_BOOL_FUZZ_PAIRS="$BOOL_PAIRS" cargo test -p kw-boolean --release --test fuzz -- --nocapture | tee "$OUT/boolean_fuzz.log"

step "extrude / revolve / sweep / loft / shell / offset / draft + model fuzz (release)"
cargo test -p kw-model --release --test extrude --test revolve --test offset --test skin --test tangent_tool --test nurbs_nurbs
# free-form operands (spline extrusions, revolutions, lofts): every non-analytic pair goes through the distance-field tracer
KW_MODEL_MIN_SUCCESS=0.99 KW_MODEL_FUZZ_PAIRS="$MODEL_PAIRS" cargo test -p kw-model --release --test fuzz_models -- --nocapture | tee "$OUT/model_fuzz.log"

step "mass properties, classification, copy (release)"
cargo test -p kw-brep --release --test mass --test classify_copy

step "(4) fillet / chamfer: ${BLEND_PARTS}-part benchmark, failures name the edge; history (release)"
KW_BLEND_PARTS="$BLEND_PARTS" cargo test -p kw-blend --release --test benchmark -- --nocapture | tee "$OUT/fillet_benchmark.log"
cargo test -p kw-blend --release --test basic --test history

step "(4) vertex blends: rolling-ball sphere patches at three-face corners (exact Steiner volume of the rounded box; ${CORNER_PARTS}-part corner benchmark)"
cargo test -p kw-blend --release --test vertex
KW_CORNER_MIN_SUCCESS=0.95 KW_CORNER_PARTS="$CORNER_PARTS" cargo test -p kw-blend --release --test corners -- --nocapture | tee "$OUT/corner_benchmark.log"

step "(4) variable-radius fillets: arbitrary laws, straight edges (planes, plane/cylinder), circular rims (cylinder, cone, concave collar)"
cargo test -p kw-blend --release --test variable

step "(5) STEP round trip + corpus-free checks, IGES import (release)"
PROPTEST_CASES=2000 cargo test -p kw-step --release --test roundtrip
PROPTEST_CASES=2000 cargo test -p kw-iges --release --test import

step "ShapeHistory completeness through the feature kernel (release)"
cargo test -p kw-feature --release

step "(6) performance: Boolean on large prismatic bodies, fillets on a 2k-face body, 50 MB STEP import (release)"
# bodies whose outlines cross everywhere (worst case) and bodies that overlap locally
KW_PERF_SIZES="${KW_PERF_SIZES:-1600,3200}" cargo test -p kw-boolean --release --test perf -- --nocapture --test-threads=1 | tee "$OUT/boolean_perf.log"
KW_PERF_LOCAL=1 KW_PERF_SIZES="${KW_PERF_SIZES_LOCAL:-6400,9998}" cargo test -p kw-boolean --release --test perf -- --nocapture --test-threads=1 | tee -a "$OUT/boolean_perf.log"
KW_BLEND_PERF_N=2000 KW_BLEND_PERF_EDGES=50 cargo test -p kw-blend --release --test perf -- --nocapture --test-threads=1 | tee "$OUT/fillet_perf.log"
cargo test -p kw-step --release --test perf -- --nocapture --test-threads=1 | tee "$OUT/step_perf.log"

if [ -n "${ORACLE_PY:-}" ]; then
  step "(5) external tools: export a corpus, check it with OpenCASCADE, Gmsh and truck"
  cargo build --release -p kw-step --example step_tool
  rm -rf "$OUT/ours" && mkdir -p "$OUT/ours"
  KW_CORPUS_PER_KIND=8 ./target/release/examples/step_tool export "$OUT/ours"
  python3 scripts/oracles/self_roundtrip.py "$OUT/ours"
  "$ORACLE_PY" scripts/oracles/occt_check_ours.py "$OUT/ours"
  "$ORACLE_PY" scripts/oracles/gmsh_check.py "$OUT/ours"
  (cd scripts/oracles/truck_check && cargo run --release -q -- "../../../$OUT/ours")
  step "(5) OpenCASCADE-authored corpus imported by Kerfwright"
  rm -rf "$OUT/occt" && mkdir -p "$OUT/occt"
  "$ORACLE_PY" scripts/oracles/occt_make_corpus.py "$OUT/occt"
  python3 scripts/oracles/import_corpus.py "$OUT/occt"
  if [ -n "${ORACLE_CORPUS:-}" ]; then
    step "(5) public STEP corpus: valid-solid rate and agreement with OpenCASCADE"
    python3 scripts/oracles/import_corpus.py "$ORACLE_CORPUS" | tee "$OUT/public_corpus.log"
    "$ORACLE_PY" scripts/oracles/occt_compare_import.py "$ORACLE_CORPUS" | tee -a "$OUT/public_corpus.log"
  else
    echo "ORACLE_CORPUS not set: public corpus check skipped"
  fi
else
  echo
  echo "ORACLE_PY not set: external-tool STEP checks skipped (see docs/phase3-report.md)"
fi

echo
echo "All Phase 3 automated checks passed."
