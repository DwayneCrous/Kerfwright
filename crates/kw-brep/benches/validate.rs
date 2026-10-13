use criterion::{criterion_group, criterion_main, Criterion};
use kw_brep::{make_box, make_sphere, BRep};
use kw_math::Frame3;

fn benches(c: &mut Criterion) {
    let mut b = BRep::new();
    make_box(&mut b, &Frame3::world(), 1.0, 2.0, 3.0);
    c.bench_function("validate_box", |bn| bn.iter(|| b.validate()));
    let mut s = BRep::new();
    make_sphere(&mut s, &Frame3::world(), 1.0);
    c.bench_function("validate_sphere", |bn| bn.iter(|| s.validate()));
    c.bench_function("build_box", |bn| {
        bn.iter(|| {
            let mut b = BRep::new();
            make_box(&mut b, &Frame3::world(), 1.0, 2.0, 3.0);
            b
        })
    });
}

criterion_group!(g, benches);
criterion_main!(g);
