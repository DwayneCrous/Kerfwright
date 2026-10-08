use criterion::{criterion_group, criterion_main, Criterion};
use kw_brep::{make_box, make_sphere, make_torus, BRep};
use kw_math::Frame3;
use kw_tess::{tessellate_body, TessOptions};

fn benches(c: &mut Criterion) {
    let opts = TessOptions {
        chord_tol: 0.001,
        ..TessOptions::default()
    };
    let mut b = BRep::new();
    let r = make_box(&mut b, &Frame3::world(), 1.0, 2.0, 3.0);
    c.bench_function("tess_box", |bn| {
        bn.iter(|| tessellate_body(&b, r.body, &opts).unwrap())
    });
    let mut s = BRep::new();
    let r = make_sphere(&mut s, &Frame3::world(), 1.0);
    c.bench_function("tess_sphere_1e-3", |bn| {
        bn.iter(|| tessellate_body(&s, r.body, &opts).unwrap())
    });
    let mut t = BRep::new();
    let r = make_torus(&mut t, &Frame3::world(), 3.0, 1.0);
    c.bench_function("tess_torus_1e-3", |bn| {
        bn.iter(|| tessellate_body(&t, r.body, &opts).unwrap())
    });
}

criterion_group!(g, benches);
criterion_main!(g);
