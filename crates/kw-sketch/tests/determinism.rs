//! Phase 2 acceptance: solver output is bit-identical across 1–32 threads.

use kw_sketch::testgen::{feature_grid, perturb, Rng};
use kw_sketch::*;

fn solve_with_threads(base: &Sketch, threads: usize, parallel: bool) -> (Vec<u64>, SolveOutcome) {
    let mut sk = base.clone();
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .unwrap();
    let out = pool.install(|| {
        let mut solver = Solver::new(SolveOptions {
            parallel,
            par_min_rows: 0,
            ..SolveOptions::default()
        });
        solver.solve(&mut sk).unwrap()
    });
    (sk.vars.values.iter().map(|v| v.to_bits()).collect(), out)
}

#[test]
fn solve_is_bit_identical_across_thread_counts() {
    // many components so the parallel path is really exercised
    let mut sk = feature_grid(120, false, 7);
    let mut rng = Rng::new(3);
    perturb(&mut sk, &mut rng, 0.3);
    let (reference, ref_out) = solve_with_threads(&sk, 1, false);
    assert_eq!(ref_out.status, SolveStatus::Converged);
    assert!(ref_out.components >= 100);
    for threads in [1usize, 2, 4, 8, 16, 32] {
        let (bits, out) = solve_with_threads(&sk, threads, true);
        assert_eq!(out.status, SolveStatus::Converged, "threads={threads}");
        assert!(
            bits == reference,
            "threads={threads}: result differs from the single-threaded solve"
        );
    }
}

#[test]
fn drag_is_bit_identical_across_thread_counts() {
    let base = {
        let mut sk = feature_grid(40, false, 11);
        let mut rng = Rng::new(5);
        perturb(&mut sk, &mut rng, 0.2);
        Solver::default().solve(&mut sk).unwrap();
        sk
    };
    let run = |threads: usize| -> Vec<u64> {
        let mut sk = base.clone();
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let mut solver = Solver::default();
            let rep = solver.analyze(&mut sk).unwrap();
            let pts: Vec<_> = rep
                .free_entities
                .iter()
                .copied()
                .filter(|e| matches!(sk.kind(*e).unwrap(), EntityKind::Point(_)))
                .collect();
            let mut rng = Rng::new(9);
            for _ in 0..8 {
                let p = pts[rng.int(pts.len())];
                let v = sk.point_vars(p).unwrap();
                let (x0, y0) = sk.point(p).unwrap();
                for k in 1..=15 {
                    let t = k as f64 * 0.15;
                    solver
                        .drag(&mut sk, &[(v.x, x0 + t), (v.y, y0 - 0.5 * t)])
                        .unwrap();
                }
            }
        });
        sk.vars.values.iter().map(|v| v.to_bits()).collect()
    };
    let reference = run(1);
    for threads in [2usize, 4, 8, 16, 32] {
        assert!(
            run(threads) == reference,
            "threads={threads}: drag result differs"
        );
    }
}
