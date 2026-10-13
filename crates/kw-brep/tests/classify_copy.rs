use kw_brep::*;
use kw_math::{Frame3, Point3, Transform3, Vector3};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

fn frame() -> Frame3 {
    Frame3::from_origin_normal(Point3::new(0.4, -0.7, 0.3), Vector3::new(0.3, 0.2, 1.0))
}

/// Compare ray-cast classification with a closed-form membership test.
fn check(rep: &BRep, body: BodyId, inside: impl Fn(&Point3<f64>) -> f64, extent: f64, n: usize) {
    let idx = SolidIndex::new(rep, body).unwrap();
    let mut rng = Rng(12345);
    let mut ambiguous = 0;
    for _ in 0..n {
        let p = frame().origin
            + Vector3::new(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5) * extent;
        let sd = inside(&p); // signed distance-like: negative inside
        if sd.abs() < 1e-3 {
            continue;
        }
        match idx.classify(&p, 1e-7) {
            Ok(PointClass::In) => assert!(sd < 0.0, "{p:?} classified In but sd={sd}"),
            Ok(PointClass::Out) => assert!(sd > 0.0, "{p:?} classified Out but sd={sd}"),
            Ok(PointClass::On) => panic!("{p:?} classified On, sd={sd}"),
            Err(_) => ambiguous += 1,
        }
    }
    assert!(ambiguous * 200 <= n, "{ambiguous}/{n} ambiguous");
}

#[test]
fn ray_casting_agrees_with_closed_form_membership() {
    let f = frame();
    let local = |p: &Point3<f64>| {
        let q = p - f.origin;
        Vector3::new(q.dot(&f.x), q.dot(&f.y), q.dot(&f.z))
    };
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &f, 2.0, 3.0, 4.0).body;
    check(
        &rep,
        b,
        |p| {
            let l = local(p);
            (l.x.abs() - 1.0).max(l.y.abs() - 1.5).max(l.z.abs() - 2.0)
        },
        7.0,
        3000,
    );
    let c = make_cylinder(&mut rep, &f, 1.5, 2.0).body;
    check(
        &rep,
        c,
        |p| {
            let l = local(p);
            (l.x.hypot(l.y) - 1.5).max((l.z - 1.0).abs() - 1.0)
        },
        5.0,
        3000,
    );
    let k = make_cone(&mut rep, &f, 2.0, 0.5, 3.0).body;
    check(
        &rep,
        k,
        |p| {
            let l = local(p);
            let r = 2.0 - 0.5 * l.z;
            (l.x.hypot(l.y) - r).max((l.z - 1.5).abs() - 1.5)
        },
        6.0,
        3000,
    );
    let s = make_sphere(&mut rep, &f, 1.7).body;
    check(&rep, s, |p| local(p).norm() - 1.7, 5.0, 3000);
    let t = make_torus(&mut rep, &f, 3.0, 0.8).body;
    check(
        &rep,
        t,
        |p| {
            let l = local(p);
            (l.x.hypot(l.y) - 3.0).hypot(l.z) - 0.8
        },
        9.0,
        3000,
    );
}

#[test]
fn points_on_the_boundary_are_on() {
    let f = frame();
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &f, 2.0, 3.0, 4.0).body;
    let idx = SolidIndex::new(&rep, b).unwrap();
    for p in [
        f.origin + f.x * 1.0,
        f.origin + f.y * -1.5 + f.x * 0.3,
        f.origin + f.z * 2.0 + f.y * 0.5,
    ] {
        assert_eq!(idx.classify(&p, 1e-7), Ok(PointClass::On));
    }
    let s = make_sphere(&mut rep, &f, 1.7).body;
    let idx = SolidIndex::new(&rep, s).unwrap();
    let p = f.origin + (f.x * 0.3 + f.y * 0.5 + f.z * 0.2).normalize() * 1.7;
    assert_eq!(idx.classify(&p, 1e-7), Ok(PointClass::On));
}

#[test]
fn copy_transform_and_flip() {
    let f = frame();
    let mut src = BRep::new();
    let ids = [
        make_box(&mut src, &f, 2.0, 3.0, 4.0).body,
        make_cylinder(&mut src, &f, 1.5, 2.0).body,
        make_cone(&mut src, &f, 2.0, 0.0, 3.0).body,
        make_sphere(&mut src, &f, 1.7).body,
        make_torus(&mut src, &f, 3.0, 0.8).body,
    ];
    let xf = Transform3::rotation(Vector3::new(1.0, 2.0, 3.0), 0.9)
        .then(&Transform3::translation(Vector3::new(5.0, -1.0, 2.0)));
    for b in ids {
        let v0 = src.volume(b).unwrap();
        let a0 = src.area(b).unwrap();
        let mut dst = BRep::new();
        let (c, _) = dst.copy_body_from(&src, b, &CopyOptions::default());
        assert!(dst.validate().is_ok());
        assert!((dst.volume(c).unwrap() - v0).abs() < 1e-9 * v0.abs().max(1.0));
        let (t, _) = dst.copy_body_from(
            &src,
            b,
            &CopyOptions {
                transform: Some(xf),
                flip: false,
                ..Default::default()
            },
        );
        assert!(dst.validate().is_ok(), "{:?}", dst.validate());
        assert!((dst.volume(t).unwrap() - v0).abs() < 1e-9 * v0.abs().max(1.0));
        assert!((dst.area(t).unwrap() - a0).abs() < 1e-9 * a0);
        let (fl, _) = dst.copy_body_from(
            &src,
            b,
            &CopyOptions {
                transform: Some(xf),
                flip: true,
                ..Default::default()
            },
        );
        assert!(
            dst.validate().is_ok(),
            "flipped copy invalid: {:?}",
            dst.validate()
        );
        assert!(
            (dst.volume(fl).unwrap() + v0).abs() < 1e-9 * v0.abs().max(1.0),
            "flipped volume"
        );
        // flipped solid: classification swaps inside and outside
        let m = src.mass_properties(b).unwrap();
        let ctr = xf.apply_point(&m.centroid);
        let idx = SolidIndex::new(&dst, fl).unwrap();
        let _ = idx.classify(&ctr, 1e-7); // must not panic
    }
}
