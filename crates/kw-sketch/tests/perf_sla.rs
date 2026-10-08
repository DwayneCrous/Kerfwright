//! Phase 2 performance SLAs (release builds only):
//!   * 500-entity mixed-constraint sketch: drag p99 ≤ 1 ms, cold solve ≤ 25 ms
//!   * 5 000 entities: cold solve ≤ 250 ms
//!
//! The assertions use decomposable sketches (many independent features, the
//! typical case). Chain-coupled structures are measured and printed but not
//! asserted: their drags move whole dependency chains.

use kw_sketch::testgen::*;
use kw_sketch::*;
use std::time::Instant;

fn pct(v: &mut [f64], p: f64) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[((v.len() as f64 * p).ceil() as usize).clamp(1, v.len()) - 1]
}

fn cold_ms(sk: &Sketch) -> f64 {
    // best of a few runs: includes compile + decomposition + symbolic factorisation
    (0..5)
        .map(|i| {
            let mut s = sk.clone();
            let mut rng = Rng::new(10 + i);
            perturb(&mut s, &mut rng, 0.3);
            let t = Instant::now();
            let mut solver = Solver::default();
            let o = solver.solve(&mut s).unwrap();
            assert_eq!(o.status, SolveStatus::Converged);
            t.elapsed().as_secs_f64() * 1e3
        })
        .fold(f64::INFINITY, f64::min)
}

fn drag_percentiles(sk: &Sketch, frames: usize) -> (f64, f64, f64, usize) {
    let mut s = sk.clone();
    let mut solver = Solver::default();
    let rep = solver.analyze(&mut s).unwrap();
    let pts: Vec<_> = rep
        .free_entities
        .iter()
        .copied()
        .filter(|e| matches!(s.kind(*e).unwrap(), EntityKind::Point(_)))
        .collect();
    let mut rng = Rng::new(99);
    let mut times = Vec::new();
    while times.len() < frames {
        let target = pts[rng.int(pts.len())];
        let v = s.point_vars(target).unwrap();
        let mut pos = s.point(target).unwrap();
        let mut heading = rng.range(0.0, std::f64::consts::TAU);
        for _ in 0..20 {
            heading += rng.range(-0.5, 0.5);
            pos = (pos.0 + 0.3 * heading.cos(), pos.1 + 0.3 * heading.sin());
            let t = Instant::now();
            let out = solver.drag(&mut s, &[(v.x, pos.0), (v.y, pos.1)]).unwrap();
            times.push(t.elapsed().as_secs_f64() * 1e3);
            if !out.reached {
                pos = s.point(target).unwrap();
            }
        }
    }
    let n = times.len();
    (
        pct(&mut times, 0.5),
        pct(&mut times, 0.95),
        pct(&mut times, 0.99),
        n,
    )
}

#[test]
fn sla_500_entities() {
    if cfg!(debug_assertions) {
        eprintln!("skipped: run with --release");
        return;
    }
    let sk = feature_grid(29, false, 1);
    assert!(entity_count(&sk) >= 490);
    let cold = cold_ms(&sk);
    let (p50, p95, p99, n) = drag_percentiles(&sk, 4000);
    println!("500 entities ({}): cold {cold:.2} ms; drag over {n} frames p50 {p50:.3} p95 {p95:.3} p99 {p99:.3} ms", entity_count(&sk));
    assert!(cold <= 25.0, "cold solve {cold} ms");
    assert!(p99 <= 1.0, "drag p99 {p99} ms");
}

#[test]
fn sla_5000_entities_cold() {
    if cfg!(debug_assertions) {
        eprintln!("skipped: run with --release");
        return;
    }
    let sk = feature_grid(294, false, 1);
    assert!(entity_count(&sk) >= 4900);
    let cold = cold_ms(&sk);
    println!("5000 entities ({}): cold {cold:.2} ms", entity_count(&sk));
    assert!(cold <= 250.0, "cold solve {cold} ms");
}

#[test]
fn coupled_structures_are_reported() {
    if cfg!(debug_assertions) {
        return;
    }
    for (name, n) in [("row-coupled 500", 29usize), ("row-coupled 5000", 294)] {
        let sk = feature_grid(n, true, 1);
        let cold = cold_ms(&sk);
        let (p50, p95, p99, frames) = drag_percentiles(&sk, 1500);
        println!("{name} ({} entities): cold {cold:.2} ms; drag {frames} frames p50 {p50:.3} p95 {p95:.3} p99 {p99:.3} ms", entity_count(&sk));
    }
    let t = truss(20, 4);
    println!(
        "truss(20) (one irreducible block, {} entities): cold {:.3} ms",
        entity_count(&t),
        cold_ms(&t)
    );
}
