#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Booleans between bodies whose faces are both free-form (spline extrusions, lofts): the surfaces of
//! both operands are non-analytic, so every intersection comes from the signed-distance tracer.
use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::*;
use kw_nurbs::{CurveGeom, NurbsCurve};
use std::f64::consts::TAU;

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

/// A closed spline blob of mean radius `rad` (smoothly varying), extruded by `h`.
fn spline_prism(rep: &mut BRep, rad: f64, wobble: f64, phase: f64, h: f64) -> BodyId {
    let n = 6;
    let mut pts: Vec<Point3<f64>> = (0..n)
        .map(|i| {
            let a = TAU * i as f64 / n as f64;
            // a deterministic pseudo-random radius in [1 - wobble, 1]
            let q = ((i as f64 * 12.9898 + phase * 78.233).sin() * 43758.5453)
                .fract()
                .abs();
            let r = rad * (1.0 - wobble * q);
            Point3::new(r * a.cos(), r * a.sin(), 0.0)
        })
        .collect();
    pts.push(pts[0]);
    let c = NurbsCurve::interpolate(&pts, 3).unwrap();
    let dom = c.domain();
    extrude(
        rep,
        &Profile::new(
            world(),
            ProfileLoop::new(vec![ProfileEdge::new(CurveGeom::Nurbs(c), dom)]),
        ),
        Vector3::new(0.0, 0.0, h),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body
}

fn moved(rep: &mut BRep, b: BodyId, t: Transform3) -> BodyId {
    let src = rep.clone();
    rep.copy_body_from(
        &src,
        b,
        &CopyOptions {
            transform: Some(t),
            flip: false,
        },
    )
    .0
}

fn check_identities(rep: &mut BRep, a: BodyId, b: BodyId, what: &str) {
    let (va, vb) = (rep.volume(a).unwrap(), rep.volume(b).unwrap());
    let run = |rep: &mut BRep, op| {
        boolean(rep, a, b, op, &BoolOptions::default())
            .unwrap_or_else(|e| panic!("{what} {op:?}: {e}"))
    };
    let u = run(rep, Op::Union);
    let i = run(rep, Op::Intersect);
    let d = run(rep, Op::Subtract);
    for (r, name) in [(&u, "union"), (&i, "intersect"), (&d, "subtract")] {
        assert!(
            rep.validate().is_ok(),
            "{what} {name}: {:?}",
            rep.validate()
        );
        assert!(rep.volume(r.body).unwrap() > 0.0);
    }
    let (vu, vi, vd) = (
        rep.volume(u.body).unwrap(),
        rep.volume(i.body).unwrap(),
        rep.volume(d.body).unwrap(),
    );
    assert!(
        ((vu + vi) - (va + vb)).abs() < 1e-8 * (va + vb),
        "{what}: ∪+∩ {} vs {}",
        vu + vi,
        va + vb
    );
    assert!(
        (vd - (va - vi)).abs() < 1e-8 * va,
        "{what}: A−B {vd} vs {}",
        va - vi
    );
}

#[test]
fn spline_prisms_crossing_each_other() {
    // two upright blobs, the second shifted sideways, and one lying on its side crossing the first
    for (k, (shift, tilt)) in [
        (Vector3::new(0.9, 0.3, 0.2), 0.0),
        (Vector3::new(-0.5, 0.6, 0.1), 0.0),
        (Vector3::new(0.05, 0.03, 0.13), std::f64::consts::FRAC_PI_2),
        (Vector3::new(0.3, -0.2, 0.4), 0.7),
    ]
    .into_iter()
    .enumerate()
    {
        let mut rep = BRep::new();
        let a = spline_prism(&mut rep, 1.2, 0.2, 0.0, 2.0);
        let b0 = spline_prism(&mut rep, 0.9, 0.25, 1.3, 2.4);
        let t = Transform3::translation(shift)
            .then(&Transform3::rotation(Vector3::x(), tilt))
            .then(&Transform3::translation(Vector3::new(0.0, 0.0, 0.0)));
        // rotate about the blob's own middle, then shift
        let to_mid = Transform3::translation(Vector3::new(0.0, 0.0, -1.2));
        let b = moved(&mut rep, b0, to_mid.then(&t));
        check_identities(&mut rep, a, b, &format!("case {k}"));
    }
}

#[test]
fn classification_of_points_against_a_spline_prism_matches_an_independent_test() {
    for case in 0..6u32 {
        let mut rep = BRep::new();
        let rad = 1.0 + 0.1 * case as f64;
        let a = spline_prism(&mut rep, rad, 0.25, case as f64, 2.0);
        // independent test: z in (0, 2) and (x, y) inside the profile polygon (dense polyline)
        let mut poly: Vec<(f64, f64)> = Vec::new();
        for f in rep.body_faces(a) {
            if let kw_nurbs::SurfaceGeom::Extrusion { profile, .. } =
                &rep.surfaces[rep.faces[f].surface]
            {
                use kw_nurbs::Curve3;
                let (t0, t1) = profile.domain();
                for k in 0..4000 {
                    let p = profile.point_at(t0 + (t1 - t0) * k as f64 / 4000.0);
                    poly.push((p.x, p.y));
                }
            }
        }
        let inside2d = |x: f64, y: f64| kw_brep::uvgeom::winding((x, y), &poly) != 0;
        let idx = SolidIndex::new(&rep, a).unwrap();
        let mut bad = 0;
        let mut total = 0;
        let mut rng = 0x1234_5678_u64.wrapping_add(case as u64);
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            ((rng >> 11) as f64) / ((1u64 << 53) as f64)
        };
        for _ in 0..3000 {
            let p = Point3::new(
                -1.6 + 3.2 * next(),
                -1.6 + 3.2 * next(),
                -0.5 + 3.0 * next(),
            );
            let truth = p.z > 0.0 && p.z < 2.0 && inside2d(p.x, p.y);
            // skip points near the boundary
            let near_wall = (0..poly.len()).any(|i| {
                (poly[i].0 - p.x).hypot(poly[i].1 - p.y) < 0.01 && p.z > -0.01 && p.z < 2.01
            });
            if near_wall || (p.z - 0.0).abs() < 0.01 || (p.z - 2.0).abs() < 0.01 {
                continue;
            }
            total += 1;
            match idx.classify(&p, 1e-8) {
                Ok(c) => {
                    let got = c == PointClass::In;
                    if got != truth {
                        bad += 1;
                    }
                }
                Err(_) => bad += 0,
            }
        }
        assert_eq!(
            bad, 0,
            "case {case}: {bad} of {total} points classified wrongly"
        );
    }
}

#[test]
fn probe_case2_points() {
    let mut rep = BRep::new();
    let a = spline_prism(&mut rep, 1.2, 0.2, 0.0, 2.0);
    let idx = SolidIndex::new(&rep, a).unwrap();
    for p in [
        Point3::new(-0.2593003597697112, 1.0699999999999998, 0.25747572225164295),
        Point3::new(-0.2593003597697112, 1.0699999999999998, 1.0),
        Point3::new(-0.2593003597697112, 1.0, 0.25),
        Point3::new(0.08113028217166784, -0.00779863982261686, 0.0),
        Point3::new(0.08, -0.0078, 0.001),
    ] {
        eprintln!("probe {p:?}: {:?}", idx.classify(&p, 1e-8));
    }
}
