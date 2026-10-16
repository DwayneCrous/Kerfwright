#!/usr/bin/env bash
# Runs every automated Phase 5 (CAM, FEA, ECAD, advanced rendering) acceptance check.
# Usage: scripts/phase5_acceptance.sh [out_dir]
#
# Knobs:
#   SKIP_WORKSPACE_TESTS=1   only the Phase 5 checks (skip the whole-workspace test run)
#   KW_PT_REF_SPP=16384      samples per pixel of the CPU reference image (default here 16384)
#   KW_RENDER_FRAMES=600     frames of the realtime benchmark
#   KW_FEM_PCG_N=22          cells along the short side of the 1M-DOF PCG run
#
# CPU steps are memory-capped (soft ulimit -v) and use at most 8 threads. GPU steps run without the cap:
# Mesa/Vulkan reserve very large virtual ranges and fail under `ulimit -v`. They skip themselves when the
# machine has no adapter.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"
OUT="${1:-target/phase5}"
mkdir -p "$OUT"
export RAYON_NUM_THREADS="${RAYON_NUM_THREADS:-8}"
CAP="${VMEM_KB:-16000000}"

step() { printf '\n=== %s ===\n' "$*"; }
cpu() { ( ulimit -S -v "$CAP" || true; "$@" ); }
gpu() { ( ulimit -S -v unlimited 2>/dev/null || true; "$@" ); }

step "fmt + clippy"
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

if [ -z "${SKIP_WORKSPACE_TESTS:-}" ]; then
  step "unit + integration tests, whole workspace (release; GPU tests run capped here and skip themselves if they cannot get an adapter)"
  cpu cargo test --workspace --release
fi

step "(1) tetrahedral meshing: empty-sphere property, kernel solids, radius-edge <= 2, no inverted elements, determinism, cancel"
cpu cargo test -p kw-job -p kw-mesh3d --release -- --nocapture --test-threads=1 2>&1 | tee "$OUT/mesher.log"

step "(2) FEA benchmarks: cantilever, plate with hole (Kirsch), Lame cylinder and sphere, modal; SIMP MBB"
cpu cargo test -p kw-fem --release --test basic --test benchmarks --test simp -- --nocapture --test-threads=1 2>&1 | tee "$OUT/fem.log"

step "(3) FEA performance: 100k DOF direct, 1M DOF PCG to 1e-8"
cpu cargo test -p kw-fem --release --test perf -- --nocapture --test-threads=1 2>&1 | tee "$OUT/fem_perf.log"

step "(4) CAM: adaptive clearing (feature-gated, patent review pending), 50 seeded pockets, independent verifier"
cpu cargo test -p kw-cam --release --features adaptive-clearing --test adaptive -- --nocapture 2>&1 | tee "$OUT/adaptive.log"

step "(5) CAM: drop-cutter vs closed forms and brute force; tri-dexel vs 5 closed-form volumes"
cpu cargo test -p kw-cam --release --test dropcutter --test dexel -- --nocapture --test-threads=1 2>&1 | tee "$OUT/dropcutter_dexel.log"

step "(6) CAM: 3-D verification suite (21 parts, gouge + holder collisions, two checkers, planted faults)"
cpu cargo test -p kw-cam --release --test verify3d -- --nocapture --test-threads=2 2>&1 | tee "$OUT/verify3d.log"

step "(7) CAM: kinematics (5 machines, both branches), singularity, 3+2, swarf"
cpu cargo test -p kw-cam --release --lib kin -- --nocapture 2>&1 | tee "$OUT/kin.log"
cpu cargo test -p kw-cam --release --test multiaxis -- --nocapture 2>&1 | tee "$OUT/multiaxis.log"

step "(8) posts: every dialect replayed by the independent interpreters (2.5-D, adaptive, 50 five-axis programs)"
cpu cargo test -p kw-gsim --release -- --nocapture 2>&1 | tee "$OUT/gsim.log"
cpu cargo test -p kw-cam --release --test thin --test fiveaxis -- --nocapture 2>&1 | tee "$OUT/posts.log"

step "(9) ECAD: 20 seeded boards, router, independent DRC and connectivity, pour, Gerber/Excellon in two viewers, IDF/STEP/pick-and-place"
cpu cargo test -p kw-ecad -p kw-fabcheck --release -- --nocapture --test-threads=1 2>&1 | tee "$OUT/ecad.log"

step "(10) .kwf storage of cam / fea / board entries"
cpu cargo test -p kw-store --release -- --nocapture 2>&1 | tee "$OUT/store.log"

step "(11) rendering, realtime: OIT blend equation, GTAO, 1M-triangle frame time (GPU, uncapped)"
gpu cargo test -p kw-render --release --test frame -- --nocapture --test-threads=1 2>&1 | tee "$OUT/render_frame.log"

step "(12) path tracer: LBVH, furnace, disc light, GGX albedo (CPU reference)"
cpu cargo test -p kw-render --release --test pt_cpu -- --nocapture 2>&1 | tee "$OUT/pt_cpu.log"

step "(13) path tracer on the GPU: WGSL BRDF vs f64, furnace/disc light, convergence vs the CPU reference, denoiser PSNR (GPU, uncapped)"
KW_PT_REF_SPP="${KW_PT_REF_SPP:-16384}" gpu cargo test -p kw-render --release --test pt_gpu -- --nocapture --test-threads=1 2>&1 | tee "$OUT/pt_gpu.log"

printf '\nAll Phase 5 checks passed. Logs are in %s\n' "$OUT"
