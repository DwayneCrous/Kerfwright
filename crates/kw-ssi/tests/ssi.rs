#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
use kw_math::Frame3;
use kw_nurbs::geom::*;
use kw_nurbs::proj::surface_param_of;
use kw_ssi::*;
use nalgebra::{Point3, Vector3};
use std::f64::consts::{FRAC_PI_2, TAU};

const W: f64 = 6.0;

fn window(s: &SurfaceGeom) -> Window {
    match s {
        SurfaceGeom::Plane(_) => Window {
            u: (-W, W),
            v: (-W, W),
        },
        SurfaceGeom::Cylinder(_) => Window {
            u: (0.0, TAU),
            v: (-W, W),
        },
        SurfaceGeom::Cone(_) => Window {
            u: (0.0, TAU),
            v: (0.0, 3.0),
        },
        SurfaceGeom::Sphere(_) => Window {
            u: (0.0, TAU),
            v: (-FRAC_PI_2, FRAC_PI_2),
        },
        SurfaceGeom::Torus(_) => Window {
            u: (0.0, TAU),
            v: (0.0, TAU),
        },
        SurfaceGeom::Nurbs(n) => {
            let (u, v) = n.domain();
            Window { u, v }
        }
        _ => Window {
            u: (0.0, 1.0),
            v: (0.0, 1.0),
        },
    }
}

fn frame(o: [f64; 3], z: [f64; 3], rot: f64) -> Frame3 {
    let mut f = Frame3::from_origin_normal(
        Point3::new(o[0], o[1], o[2]),
        Vector3::new(z[0], z[1], z[2]),
    );
    let (c, s) = (rot.cos(), rot.sin());
    let (x, y) = (f.x * c + f.y * s, -f.x * s + f.y * c);
    f.x = x;
    f.y = y;
    f
}

fn plane(o: [f64; 3], z: [f64; 3]) -> SurfaceGeom {
    SurfaceGeom::Plane(Plane {
        frame: frame(o, z, 0.3),
    })
}
fn cyl(o: [f64; 3], z: [f64; 3], r: f64) -> SurfaceGeom {
    SurfaceGeom::Cylinder(Cylinder {
        frame: frame(o, z, 0.7),
        radius: r,
    })
}
fn sph(o: [f64; 3], r: f64) -> SurfaceGeom {
    SurfaceGeom::Sphere(Sphere {
        frame: frame(o, [0.3, 0.4, 1.0], 1.1),
        radius: r,
    })
}
fn cone(o: [f64; 3], z: [f64; 3], r0: f64, a: f64) -> SurfaceGeom {
    SurfaceGeom::Cone(Cone {
        frame: frame(o, z, 0.2),
        radius_at_origin: r0,
        half_angle: a,
    })
}
fn tor(o: [f64; 3], z: [f64; 3], big: f64, small: f64) -> SurfaceGeom {
    SurfaceGeom::Torus(Torus {
        frame: frame(o, z, 2.0),
        major_radius: big,
        minor_radius: small,
    })
}

/// Every returned curve lies on both surfaces and its pcurves reproduce it.
fn check_curves(a: &SurfaceGeom, b: &SurfaceGeom, curves: &[SsiCurve]) -> f64 {
    let mut worst: f64 = 0.0;
    for c in curves {
        for i in 0..=200 {
            let t = c.range.0 + (c.range.1 - c.range.0) * i as f64 / 200.0;
            let p = c.curve.point_at(t);
            let qa = c.pcurve_a.point_at(t);
            let qb = c.pcurve_b.point_at(t);
            let scale = 1.0 + p.coords.amax();
            worst = worst.max((a.point_at(qa.x, qa.y) - p).norm() / scale);
            worst = worst.max((b.point_at(qb.x, qb.y) - p).norm() / scale);
            let ra = surface_param_of(a, &p, Some((qa.x, qa.y)));
            let rb = surface_param_of(b, &p, Some((qb.x, qb.y)));
            worst = worst.max(ra.distance / scale).max(rb.distance / scale);
        }
        if c.closed {
            let d = (c.curve.point_at(c.range.0) - c.curve.point_at(c.range.1)).norm();
            assert!(d < 1e-8, "closed curve not closed: {d}");
        }
    }
    worst
}

fn run(a: &SurfaceGeom, b: &SurfaceGeom) -> SsiOutcome {
    intersect(a, window(a), b, window(b), DEFAULT_TOL)
        .unwrap_or_else(|e| panic!("{e}: {a:?} vs {b:?}"))
}

fn branches(a: &SurfaceGeom, b: &SurfaceGeom) -> usize {
    match run(a, b) {
        SsiOutcome::Curves { curves, .. } => {
            let w = check_curves(a, b, &curves);
            assert!(w < 1e-8, "curve off surfaces by {w}: {a:?} vs {b:?}");
            curves.len()
        }
        SsiOutcome::Empty => 0,
        SsiOutcome::Points(_) => 0,
        SsiOutcome::Coincident { .. } => 99,
    }
}

#[test]
fn labelled_branch_suite() {
    // (name, a, b, expected curves)
    let o = [0.0, 0.0, 0.0];
    let cases: Vec<(&str, SurfaceGeom, SurfaceGeom, usize)> = vec![
        (
            "plane/plane",
            plane(o, [0.0, 0.0, 1.0]),
            plane(o, [1.0, 0.2, 0.3]),
            1,
        ),
        (
            "plane/plane parallel",
            plane(o, [0.0, 0.0, 1.0]),
            plane([0.0, 0.0, 2.0], [0.0, 0.0, 1.0]),
            0,
        ),
        (
            "plane/sphere",
            plane([0.0, 0.0, 0.5], [0.3, 0.1, 1.0]),
            sph(o, 2.0),
            1,
        ),
        (
            "plane/sphere miss",
            plane([0.0, 0.0, 5.0], [0.0, 0.0, 1.0]),
            sph(o, 2.0),
            0,
        ),
        (
            "plane/cyl perpendicular",
            plane([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]),
            cyl(o, [0.0, 0.0, 1.0], 1.5),
            1,
        ),
        (
            "plane/cyl oblique",
            plane(o, [0.5, 0.0, 1.0]),
            cyl(o, [0.0, 0.0, 1.0], 1.5),
            1,
        ),
        (
            "plane/cyl parallel 2 lines",
            plane([0.5, 0.0, 0.0], [1.0, 0.0, 0.0]),
            cyl(o, [0.0, 0.0, 1.0], 1.5),
            2,
        ),
        (
            "plane/cone ellipse",
            plane([0.0, 0.0, 1.5], [0.2, 0.0, 1.0]),
            cone(o, [0.0, 0.0, 1.0], 0.5, 0.3),
            1,
        ),
        (
            "plane/cone circle",
            plane([0.0, 0.0, 1.5], [0.0, 0.0, 1.0]),
            cone(o, [0.0, 0.0, 1.0], 0.5, 0.3),
            1,
        ),
        (
            "plane/torus through axis",
            plane(o, [0.0, 1.0, 0.0]),
            tor(o, [0.0, 0.0, 1.0], 3.0, 1.0),
            2,
        ),
        (
            "plane/torus flat 2 circles",
            plane([0.0, 0.0, 0.3], [0.0, 0.0, 1.0]),
            tor(o, [0.0, 0.0, 1.0], 3.0, 1.0),
            2,
        ),
        ("sphere/sphere", sph(o, 2.0), sph([1.5, 0.0, 0.0], 1.5), 1),
        (
            "sphere/cyl coaxial-ish",
            sph(o, 2.0),
            cyl(o, [0.0, 0.0, 1.0], 1.0),
            2,
        ),
        (
            "cyl/cyl parallel",
            cyl(o, [0.0, 0.0, 1.0], 1.0),
            cyl([1.2, 0.0, 0.0], [0.0, 0.0, 1.0], 1.0),
            2,
        ),
        (
            "cyl/cyl perpendicular small-in-big (2 loops)",
            cyl(o, [0.0, 0.0, 1.0], 2.0),
            cyl(o, [1.0, 0.0, 0.0], 1.0),
            2,
        ),
        (
            "cyl/cyl perpendicular offset (1 loop)",
            cyl(o, [0.0, 0.0, 1.0], 2.0),
            cyl([0.0, 1.6, 0.0], [1.0, 0.0, 0.0], 1.0),
            1,
        ),
        (
            "cyl/cone coaxial",
            cyl(o, [0.0, 0.0, 1.0], 1.0),
            cone(o, [0.0, 0.0, 1.0], 0.5, 0.3),
            1,
        ),
        (
            "sphere/torus coaxial",
            sph(o, 3.0),
            tor(o, [0.0, 0.0, 1.0], 3.0, 1.0),
            2,
        ),
        (
            "sphere/cone",
            sph(o, 2.0),
            cone([0.0, 0.0, -1.0], [0.0, 0.0, 1.0], 0.5, 0.3),
            1,
        ),
        (
            "torus/cyl through hole",
            tor(o, [0.0, 0.0, 1.0], 3.0, 1.0),
            cyl([3.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.5),
            2,
        ),
        (
            "sphere/cyl oblique",
            sph(o, 2.0),
            cyl([0.8, 0.5, 0.0], [0.2, 0.3, 1.0], 1.0),
            2,
        ),
        (
            "plane/cone hyperbola-ish",
            plane([0.4, 0.0, 0.0], [1.0, 0.0, 0.0]),
            cone([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.2, 0.5),
            1,
        ),
    ];
    let mut failures = Vec::new();
    for (name, a, b, want) in &cases {
        let got = std::panic::catch_unwind(|| branches(a, b));
        match got {
            Ok(n) if n == *want => {}
            Ok(n) => failures.push(format!("{name}: expected {want} branches, got {n}")),
            Err(_) => failures.push(format!("{name}: panicked / error")),
        }
        // symmetric call must agree
        if let Ok(n) = std::panic::catch_unwind(|| branches(b, a)) {
            if n != *want {
                failures.push(format!("{name} (swapped): expected {want}, got {n}"));
            }
        }
    }
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

#[test]
fn coincident_and_tangent_cases() {
    let o = [0.0, 0.0, 0.0];
    let p = plane(o, [0.0, 0.0, 1.0]);
    let q = plane([1.0, 2.0, 0.0], [0.0, 0.0, -1.0]);
    assert!(matches!(
        run(&p, &q),
        SsiOutcome::Coincident { same_normal: false }
    ));
    assert!(matches!(
        run(&sph(o, 2.0), &sph(o, 2.0)),
        SsiOutcome::Coincident { same_normal: true }
    ));
    let c = cyl(o, [0.0, 0.0, 1.0], 1.0);
    assert!(matches!(
        run(&c, &cyl([0.0, 0.0, 3.0], [0.0, 0.0, 1.0], 1.0)),
        SsiOutcome::Coincident { .. }
    ));
    // plane tangent to a sphere: one point
    match run(&plane([0.0, 0.0, 2.0], [0.0, 0.0, 1.0]), &sph(o, 2.0)) {
        SsiOutcome::Points(p) => assert_eq!(p.len(), 1),
        other => panic!("{other:?}"),
    }
    // plane tangent to a cylinder: one line flagged tangent
    match run(&plane([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]), &c) {
        SsiOutcome::Curves { curves, tangent } => {
            assert!(tangent);
            assert_eq!(curves.len(), 1);
        }
        other => panic!("{other:?}"),
    }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    fn vec3(&mut self, a: f64, b: f64) -> [f64; 3] {
        [self.range(a, b), self.range(a, b), self.range(a, b)]
    }
}

/// Random analytic / analytic pairs. Hard requirements: no panic, every returned curve lies on
/// both surfaces and its pcurves reproduce it. Measured (and bounded): the fraction of pairs
/// where a near-intersection sample has no nearby returned curve (missed branch).
#[test]
fn random_pairs_curves_are_on_both_surfaces_and_few_branches_are_missed() {
    let n: usize = std::env::var("SSI_PAIRS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) { 150 } else { 2000 });
    let mut rng = Rng(0x5eed);
    let (mut pairs, mut errors, mut with_curves, mut missed_pairs, mut worst): (
        usize,
        usize,
        usize,
        usize,
        f64,
    ) = (0, 0, 0, 0, 0.0);
    while pairs < n {
        let kind = (rng.next() * 10.0) as usize;
        let (o1, z1, o2, z2) = (
            rng.vec3(-1.0, 1.0),
            rng.vec3(-1.0, 1.0),
            rng.vec3(-1.0, 1.0),
            rng.vec3(-1.0, 1.0),
        );
        let (r1, r2) = (rng.range(0.6, 2.5), rng.range(0.6, 2.5));
        if Vector3::from(z1).norm() < 0.2 || Vector3::from(z2).norm() < 0.2 {
            continue;
        }
        let (a, b) = match kind {
            0 => (plane(o1, z1), sph(o2, r2)),
            1 => (plane(o1, z1), cyl(o2, z2, r2)),
            2 => (plane(o1, z1), cone(o2, z2, r2, 0.25)),
            3 => (plane(o1, z1), tor(o2, z2, r1 + 1.5, 0.6 * r2.min(r1 + 1.0))),
            4 => (sph(o1, r1), sph(o2, r2)),
            5 => (sph(o1, r1), cyl(o2, z2, r2)),
            6 => (cyl(o1, z1, r1), cyl(o2, z2, r2)),
            7 => (sph(o1, r1), cone(o2, z2, r2, 0.3)),
            8 => (cyl(o1, z1, r1), tor(o2, z2, r2 + 1.0, 0.5)),
            _ => (cone(o1, z1, r1, 0.2), cyl(o2, z2, r2)),
        };
        pairs += 1;
        let out = match intersect(&a, window(&a), &b, window(&b), DEFAULT_TOL) {
            Ok(o) => o,
            Err(_) => {
                errors += 1;
                continue;
            }
        };
        let SsiOutcome::Curves { curves, .. } = &out else {
            continue;
        };
        with_curves += 1;
        let w = check_curves(&a, &b, curves);
        worst = worst.max(w);
        assert!(
            w < 1e-8,
            "returned curve is off the surfaces by {w}: {a:?} vs {b:?}"
        );
        // completeness oracle (see comments in the sampling): skip astronomically long curves
        if curves.iter().any(|c| {
            (c.curve.point_at(c.range.0) - c.curve.point_at(0.5 * (c.range.0 + c.range.1))).norm()
                > 300.0
        }) {
            continue;
        }
        let imp = kw_ssi::implicit::Implicit::of(&b).unwrap();
        let wa = window(&a);
        let polys: Vec<Vec<Point3<f64>>> = curves
            .iter()
            .map(|c| {
                (0..=4000)
                    .map(|i| {
                        c.curve
                            .point_at(c.range.0 + (c.range.1 - c.range.0) * i as f64 / 4000.0)
                    })
                    .collect()
            })
            .collect();
        let mut samples = Vec::new();
        let (mut pos, mut neg) = (false, false);
        for iu in 0..60 {
            for iv in 0..60 {
                let u = wa.u.0 + (wa.u.1 - wa.u.0) * (iu as f64 + 0.37) / 60.0;
                let v = wa.v.0 + (wa.v.1 - wa.v.0) * (iv as f64 + 0.61) / 60.0;
                let x = a.point_at(u, v);
                let f = imp.eval(&x).0;
                pos |= f > 1e-3;
                neg |= f < -1e-3;
                samples.push((x, f));
            }
        }
        if pos && neg {
            // curves are cut to B's window: only samples whose footpoint lies in it count
            let wb = window(&b);
            let in_wb = |x: &Point3<f64>| {
                let q = surface_param_of(&b, x, None);
                let inside = |val: f64, lo: f64, hi: f64, per: Option<f64>| match per {
                    Some(p) if hi - lo >= p - 1e-9 => true,
                    Some(p) => {
                        let r = (val - lo).rem_euclid(p);
                        r <= hi - lo + 1e-9 || r >= p - 1e-9
                    }
                    None => val >= lo - 1e-9 && val <= hi + 1e-9,
                };
                inside(q.u, wb.u.0, wb.u.1, b.period_u())
                    && inside(q.v, wb.v.0, wb.v.1, b.period_v())
            };
            let missed = samples.iter().any(|(x, f)| {
                f.abs() < 0.01
                    && in_wb(x)
                    && !polys
                        .iter()
                        .any(|p| p.iter().any(|q| (q - x).norm() < 0.15 + 120.0 * f.abs()))
            });
            missed_pairs += usize::from(missed);
        }
    }
    println!("{pairs} pairs: {errors} honest errors, {with_curves} with curves, {missed_pairs} pairs with a possibly missed branch, worst off-surface {worst:.2e}");
    assert!(errors * 100 <= pairs, "too many failures: {errors}/{pairs}");
    assert!(
        missed_pairs * 200 <= with_curves.max(1),
        "missed-branch rate too high: {missed_pairs}/{with_curves}"
    );
}

fn wavy_patch(seed: f64) -> SurfaceGeom {
    use kw_nurbs::{KnotVector, NurbsSurface};
    let n = 6;
    let ku = KnotVector::clamped_uniform(3, n).unwrap();
    let kv = KnotVector::clamped_uniform(3, n).unwrap();
    let mut pts = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (
                -3.0 + 6.0 * i as f64 / (n - 1) as f64,
                -3.0 + 6.0 * j as f64 / (n - 1) as f64,
            );
            let z = 0.8 * (1.3 * x + seed).sin() * (0.9 * y - seed).cos() + 0.2 * x;
            pts.push(Point3::new(x, y, z));
        }
    }
    SurfaceGeom::Nurbs(NurbsSurface::new(ku, kv, pts, vec![1.0; n * n]).unwrap())
}

#[test]
fn nurbs_surfaces_against_analytic_surfaces() {
    for (k, seed) in [0.3, 1.1, 2.4].into_iter().enumerate() {
        let patch = wavy_patch(seed);
        let others = [
            plane([0.0, 0.0, 0.2], [0.1, 0.0, 1.0]),
            sph([0.2, 0.1, 0.3], 2.0),
            cyl([0.0, 0.0, 0.0], [0.0, 1.0, 0.1], 1.2),
            plane([0.5, 0.0, 0.0], [1.0, 0.2, 0.0]),
        ];
        for (m, other) in others.iter().enumerate() {
            let out = intersect(&patch, window(&patch), other, window(other), DEFAULT_TOL)
                .unwrap_or_else(|e| panic!("patch {k} vs {m}: {e}"));
            match out {
                SsiOutcome::Curves { curves, .. } => {
                    assert!(!curves.is_empty());
                    let w = check_curves(&patch, other, &curves);
                    assert!(w < 2e-9, "patch {k} vs {m}: off by {w}");
                }
                SsiOutcome::Empty => panic!("patch {k} vs {m}: expected an intersection"),
                o => panic!("{o:?}"),
            }
        }
    }
}

/// A vertical wavy wall `x ∈ [-3, 3]`, `z ∈ [-2, 2]`, bulging in `y`.
fn wavy_wall(seed: f64, tilt: f64) -> SurfaceGeom {
    use kw_nurbs::{KnotVector, NurbsSurface};
    let n = 6;
    let ku = KnotVector::clamped_uniform(3, n).unwrap();
    let kv = KnotVector::clamped_uniform(3, n).unwrap();
    let mut pts = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let x = -3.0 + 6.0 * i as f64 / (n - 1) as f64;
            let z = -2.0 + 4.0 * j as f64 / (n - 1) as f64;
            let y = 0.6 * (1.1 * x + seed).sin() + 0.4 * (1.7 * z - seed).cos() + tilt * z;
            pts.push(Point3::new(x, y, z));
        }
    }
    SurfaceGeom::Nurbs(NurbsSurface::new(ku, kv, pts, vec![1.0; n * n]).unwrap())
}

/// Coverage oracle: grid points of `a` that lie within `eps` of `b` must be near some traced curve point.
fn nurbs_coverage(a: &SurfaceGeom, b: &SurfaceGeom, curves: &[SsiCurve], eps: f64) -> usize {
    let wa = window(a);
    let mut polyline: Vec<Point3<f64>> = Vec::new();
    for c in curves {
        for i in 0..=400 {
            let t = c.range.0 + (c.range.1 - c.range.0) * i as f64 / 400.0;
            polyline.push(c.curve.point_at(t));
        }
    }
    let mut uncovered = 0;
    let n = 50;
    for i in 0..=n {
        for j in 0..=n {
            let u = wa.u.0 + (wa.u.1 - wa.u.0) * i as f64 / n as f64;
            let v = wa.v.0 + (wa.v.1 - wa.v.0) * j as f64 / n as f64;
            let p = a.point_at(u, v);
            let d = surface_param_of(b, &p, None).distance;
            if d < eps {
                let near = polyline.iter().any(|q| (q - p).norm() < 0.12);
                if !near {
                    uncovered += 1;
                }
            }
        }
    }
    uncovered
}

#[test]
fn nurbs_surfaces_against_nurbs_surfaces() {
    let mut found = 0;
    for (k, (s1, s2, tilt)) in [
        (0.3, 1.0, 0.0),
        (1.1, 0.4, 0.3),
        (2.4, 2.0, -0.4),
        (0.9, 0.2, 0.1),
    ]
    .into_iter()
    .enumerate()
    {
        let a = wavy_patch(s1);
        let b = wavy_wall(s2, tilt);
        let out = intersect(&a, window(&a), &b, window(&b), DEFAULT_TOL)
            .unwrap_or_else(|e| panic!("pair {k}: {e}"));
        match out {
            SsiOutcome::Curves { curves, .. } => {
                assert!(!curves.is_empty());
                let w = check_curves(&a, &b, &curves);
                assert!(w < 1e-8, "pair {k}: off by {w}");
                let missed = nurbs_coverage(&a, &b, &curves, 0.01);
                assert_eq!(
                    missed, 0,
                    "pair {k}: {missed} near-surface samples not covered"
                );
                found += 1;
            }
            SsiOutcome::Empty => panic!("pair {k}: expected an intersection"),
            o => panic!("{o:?}"),
        }
        // both argument orders
        let out2 = intersect(&b, window(&b), &a, window(&a), DEFAULT_TOL)
            .unwrap_or_else(|e| panic!("pair {k} swapped: {e}"));
        if let SsiOutcome::Curves { curves, .. } = out2 {
            let w = check_curves(&b, &a, &curves);
            assert!(w < 1e-8, "pair {k} swapped: off by {w}");
        } else {
            panic!("pair {k} swapped: expected curves");
        }
    }
    assert_eq!(found, 4);
}

fn random_patch(rng: &mut Rng, centre: [f64; 3], tilt: [f64; 3], amp: f64) -> SurfaceGeom {
    use kw_nurbs::{KnotVector, NurbsSurface};
    let n = 6;
    let ku = KnotVector::clamped_uniform(3, n).unwrap();
    let kv = KnotVector::clamped_uniform(3, n).unwrap();
    let rot = nalgebra::Rotation3::from_scaled_axis(Vector3::from(tilt));
    let mut pts = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (
                -2.5 + 5.0 * i as f64 / (n - 1) as f64,
                -2.5 + 5.0 * j as f64 / (n - 1) as f64,
            );
            let z = amp * (rng.next() - 0.5) * 2.0;
            pts.push(rot * Point3::new(x, y, z) + Vector3::from(centre));
        }
    }
    SurfaceGeom::Nurbs(NurbsSurface::new(ku, kv, pts, vec![1.0; n * n]).unwrap())
}

/// Random free-form patches against each other: no panics, every returned curve on both surfaces.
#[test]
fn random_nurbs_pairs_curves_are_on_both_surfaces() {
    let n: usize = std::env::var("SSI_NURBS_PAIRS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) { 6 } else { 120 });
    let threads = std::thread::available_parallelism().map_or(4, |v| v.get().min(8));
    let results: Vec<(usize, usize, f64)> = std::thread::scope(|s| {
        let handles: Vec<_> = (0..threads)
            .map(|t| {
                s.spawn(move || {
                    let (mut errors, mut with_curves, mut worst) = (0usize, 0usize, 0.0f64);
                    for k in (t..n).step_by(threads) {
                        let mut rng =
                            Rng(0x9E37_79B9 ^ (k as u64 + 1).wrapping_mul(0x2545_F491_4F6C_DD1D));
                        for _ in 0..4 {
                            rng.next();
                        }
                        let (c1, t1, a1) = (
                            rng.vec3(-0.3, 0.3),
                            rng.vec3(-0.4, 0.4),
                            rng.range(0.3, 1.2),
                        );
                        let a = random_patch(&mut rng, c1, t1, a1);
                        let (c2, t2, a2) = (
                            rng.vec3(-0.5, 0.5),
                            rng.vec3(-1.6, 1.6),
                            rng.range(0.3, 1.2),
                        );
                        let b = random_patch(&mut rng, c2, t2, a2);
                        match intersect(&a, window(&a), &b, window(&b), DEFAULT_TOL) {
                            Err(_) => errors += 1,
                            Ok(SsiOutcome::Curves { curves, .. }) => {
                                with_curves += 1;
                                worst = worst.max(check_curves(&a, &b, &curves));
                            }
                            Ok(_) => {}
                        }
                    }
                    (errors, with_curves, worst)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let errors: usize = results.iter().map(|r| r.0).sum();
    let with_curves: usize = results.iter().map(|r| r.1).sum();
    let worst = results.iter().fold(0.0f64, |m, r| m.max(r.2));
    println!("{n} random NURBS/NURBS pairs: {errors} honest errors, {with_curves} with curves, worst off-surface {worst:.2e}");
    assert!(worst < 1e-8, "a curve is off the surfaces by {worst}");
    assert!(errors * 20 <= n, "too many failures: {errors}/{n}");
}

#[test]
fn plane_tangent_to_a_cone_along_a_generator() {
    // cone of half-angle 0.4 and radius 1.5 at its frame origin; the plane through the apex tangent along
    // the generator at azimuth 0.9 meets the cone in exactly that line
    let a: f64 = 0.4;
    let fr = frame([0.3, -0.2, 0.1], [0.2, -0.3, 1.0], 0.2);
    let r0 = 1.5;
    let c = SurfaceGeom::Cone(Cone {
        frame: fr,
        radius_at_origin: r0,
        half_angle: a,
    });
    let apex = fr.origin - fr.z * (r0 / a.tan());
    let th: f64 = 0.9;
    let rho = fr.x * th.cos() + fr.y * th.sin();
    let g = rho * a.tan() + fr.z;
    let n = (fr.z * a.tan() - rho).normalize();
    let p = SurfaceGeom::Plane(Plane {
        frame: Frame3::from_origin_normal(apex, n),
    });
    let cw = Window {
        u: (0.5, 1.4),
        v: (0.2, 2.5),
    };
    let pw = Window {
        u: (-6.0, 6.0),
        v: (-6.0, 6.0),
    };
    match intersect(&p, pw, &c, cw, DEFAULT_TOL).unwrap() {
        SsiOutcome::Curves { curves, tangent } => {
            assert!(tangent);
            assert_eq!(curves.len(), 1);
            let w = check_curves(&p, &c, &curves);
            assert!(w < 1e-9, "off the surfaces by {w}");
            let q = curves[0]
                .curve
                .point_at(0.5 * (curves[0].range.0 + curves[0].range.1));
            assert!(((q - apex).normalize().cross(&g.normalize())).norm() < 1e-9);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn cylinder_continuing_a_torus_tube_touches_it_along_a_circle() {
    // torus (R = 1, ρ = 0.5) about z; a cylinder of radius 0.5 whose axis is the tangent line of the major
    // circle at azimuth 0.6. Faces on opposite sides of the contact plane meet in the tube circle only.
    let t = tor([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 1.0, 0.5);
    let SurfaceGeom::Torus(tk) = &t else { panic!() };
    let phi: f64 = 0.6;
    let radial = tk.frame.x * phi.cos() + tk.frame.y * phi.sin();
    let tangent = tk.frame.z.cross(&radial);
    let foot = tk.frame.origin + radial;
    let c = SurfaceGeom::Cylinder(Cylinder {
        frame: Frame3::from_origin_normal(foot, tangent),
        radius: 0.5,
    });
    // cylinder face: 0 ≤ s ≤ 3 along the tangent (quarter of the tube); torus face: the same sweep backwards
    let cw = Window {
        u: (0.0, FRAC_PI_2),
        v: (-0.15, 3.15),
    };
    let tw = Window {
        u: (phi - FRAC_PI_2 - 0.08, phi + 0.08),
        v: (-0.08, FRAC_PI_2 + 0.08),
    };
    match intersect(&c, cw, &t, tw, DEFAULT_TOL).unwrap() {
        SsiOutcome::Curves { curves, tangent } => {
            assert!(tangent);
            assert_eq!(curves.len(), 1);
            assert!(curves[0].closed);
            let w = check_curves(&c, &t, &curves);
            assert!(w < 1e-9, "off the surfaces by {w}");
        }
        other => panic!("{other:?}"),
    }
}
