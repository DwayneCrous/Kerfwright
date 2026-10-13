use criterion::{criterion_group, criterion_main, Criterion};
use kw_sketch::testgen::*;
use kw_sketch::*;

fn bench(c: &mut Criterion) {
    // ~500 and ~5 000 entities of mixed constraints (decomposable structure)
    let s500 = feature_grid(29, false, 1);
    let s5000 = feature_grid(294, false, 1);
    for (name, sk) in [("500", &s500), ("5000", &s5000)] {
        c.bench_function(&format!("cold_solve_{name}_entities"), |b| {
            b.iter_batched(
                || {
                    let mut s = sk.clone();
                    let mut rng = Rng::new(1);
                    perturb(&mut s, &mut rng, 0.3);
                    s
                },
                |mut s| {
                    let mut solver = Solver::default();
                    solver.solve(&mut s).unwrap()
                },
                criterion::BatchSize::SmallInput,
            )
        });
    }
    // one drag frame on the 500-entity sketch
    let mut sk = s500.clone();
    let mut solver = Solver::default();
    let rep = solver.analyze(&mut sk).unwrap();
    let pts: Vec<_> = rep
        .free_entities
        .iter()
        .copied()
        .filter(|e| matches!(sk.kind(*e).unwrap(), EntityKind::Point(_)))
        .collect();
    let target = pts[pts.len() / 2];
    let v = sk.point_vars(target).unwrap();
    let (x0, y0) = sk.point(target).unwrap();
    let mut k = 0u32;
    c.bench_function("drag_frame_500_entities", |b| {
        b.iter(|| {
            k += 1;
            let t = (k % 200) as f64 * 0.01;
            solver
                .drag(&mut sk, &[(v.x, x0 + t), (v.y, y0 + 0.5 * t)])
                .unwrap()
        })
    });
    // rigid truss: one irreducible block
    let truss = truss(20, 4);
    c.bench_function("cold_solve_truss_78_unknowns", |b| {
        b.iter_batched(
            || {
                let mut s = truss.clone();
                let mut rng = Rng::new(1);
                perturb(&mut s, &mut rng, 0.2);
                s
            },
            |mut s| Solver::default().solve(&mut s).unwrap(),
            criterion::BatchSize::SmallInput,
        )
    });
}

criterion_group!(g, bench);
criterion_main!(g);
