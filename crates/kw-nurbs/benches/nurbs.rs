use criterion::{criterion_group, criterion_main, Criterion};
use kw_nurbs::{KnotVector, NurbsCurve, NurbsSurface};
use nalgebra::Point3;
use std::hint::black_box;

fn curve() -> NurbsCurve {
    let n = 32;
    let kv = KnotVector::clamped_uniform(3, n).unwrap();
    let pts = (0..n)
        .map(|i| Point3::new(i as f64, (i as f64 * 0.7).sin(), (i as f64 * 0.3).cos()))
        .collect();
    let w = (0..n).map(|i| 1.0 + 0.1 * (i % 3) as f64).collect();
    NurbsCurve::new(kv, pts, w).unwrap()
}

fn surface() -> NurbsSurface {
    let (nu, nv) = (16, 16);
    let ku = KnotVector::clamped_uniform(3, nu).unwrap();
    let kv = KnotVector::clamped_uniform(3, nv).unwrap();
    let mut pts = Vec::new();
    for i in 0..nu {
        for j in 0..nv {
            pts.push(Point3::new(
                i as f64,
                j as f64,
                ((i * j) as f64 * 0.1).sin(),
            ));
        }
    }
    NurbsSurface::new(ku, kv, pts, vec![1.0; nu * nv]).unwrap()
}

fn benches(c: &mut Criterion) {
    let cv = curve();
    let sf = surface();
    c.bench_function("curve_point", |b| b.iter(|| cv.point_at(black_box(0.4137))));
    c.bench_function("curve_derivs3", |b| {
        b.iter(|| cv.derivatives(black_box(0.4137), 3))
    });
    c.bench_function("surface_point", |b| {
        b.iter(|| sf.point_at(black_box(0.31), black_box(0.62)))
    });
    c.bench_function("surface_derivs2", |b| {
        b.iter(|| sf.derivatives(black_box(0.31), black_box(0.62), 2))
    });
    c.bench_function("curve_closest_point", |b| {
        b.iter(|| cv.closest_point(black_box(&Point3::new(10.2, 0.3, 0.9))))
    });
    c.bench_function("curve_insert_knot", |b| {
        b.iter(|| cv.insert_knot(black_box(0.4137), 1))
    });
}

criterion_group!(g, benches);
criterion_main!(g);
