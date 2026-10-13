#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
use kw_brep::BRep;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

/// Profile plane = world xz-plane (normal −y... y), axis = z axis; profile coordinates (x = radius, z = height).
fn xz() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    }
}

fn check(rep: &BRep, body: kw_brep::BodyId, vol: f64, area: f64, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let v = rep.volume(body).unwrap();
    let a = rep.area(body).unwrap();
    assert!(rel(v, vol) < 1e-9, "{what}: volume {v} vs {vol}");
    assert!(rel(a, area) < 1e-9, "{what}: area {a} vs {area}");
    let t = kw_tess::tessellate_body(
        rep,
        body,
        &kw_tess::TessOptions {
            chord_tol: 0.004,
            angle_tol: 0.15,
        },
    )
    .unwrap_or_else(|e| panic!("{what}: tessellation {e:?}"));
    assert!(
        (t.mesh.signed_volume() - vol).abs() < 0.03 * vol.abs(),
        "{what}: mesh volume {} vs {vol}",
        t.mesh.signed_volume()
    );
}

#[test]
fn full_revolutions() {
    let (o, z) = (Point3::origin(), Vector3::z());
    // rectangle touching the axis → cylinder
    let mut rep = BRep::new();
    let p = Profile::polygon(xz(), &[(0.0, 0.0), (1.5, 0.0), (1.5, 2.0), (0.0, 2.0)]);
    let r = revolve(&mut rep, &p, o, z, TAU).unwrap();
    check(
        &rep,
        r.body,
        PI * 2.25 * 2.0,
        2.0 * PI * 2.25 + TAU * 1.5 * 2.0,
        "cylinder",
    );
    // right triangle → cone
    let mut rep = BRep::new();
    let p = Profile::polygon(xz(), &[(0.0, 0.0), (2.0, 0.0), (0.0, 3.0)]);
    let r = revolve(&mut rep, &p, o, z, TAU).unwrap();
    let slant = (4.0f64 + 9.0).sqrt();
    check(
        &rep,
        r.body,
        PI * 4.0 * 3.0 / 3.0,
        PI * 4.0 + PI * 2.0 * slant,
        "cone",
    );
    // washer (rectangle off the axis)
    let mut rep = BRep::new();
    let p = Profile::polygon(xz(), &[(1.0, 0.0), (2.0, 0.0), (2.0, 0.5), (1.0, 0.5)]);
    let r = revolve(&mut rep, &p, o, z, TAU).unwrap();
    check(
        &rep,
        r.body,
        PI * (4.0 - 1.0) * 0.5,
        2.0 * PI * 3.0 + TAU * 2.0 * 0.5 + TAU * 1.0 * 0.5,
        "washer",
    );
    // frustum
    let mut rep = BRep::new();
    let p = Profile::polygon(xz(), &[(0.0, 0.0), (2.0, 0.0), (1.0, 3.0), (0.0, 3.0)]);
    let r = revolve(&mut rep, &p, o, z, TAU).unwrap();
    let slant = (1.0f64 + 9.0).sqrt();
    check(
        &rep,
        r.body,
        PI * 3.0 / 3.0 * (4.0 + 2.0 + 1.0),
        PI * 4.0 + PI * 1.0 + PI * (2.0 + 1.0) * slant,
        "frustum",
    );
}

#[test]
fn spheres_and_tori() {
    let (o, z) = (Point3::origin(), Vector3::z());
    // half disc (arc + diameter on the axis) → sphere
    let f = xz();
    let mut rep = BRep::new();
    let edges = vec![
        ProfileEdge::arc(&f, 1.2, -FRAC_PI_2, FRAC_PI_2),
        ProfileEdge::line(Point3::new(0.0, 0.0, 1.2), Point3::new(0.0, 0.0, -1.2)),
    ];
    // frame xz() has x = +x, y = +z, so arc angle a ↦ (cos a, 0, sin a): from (0,0,-1.2) to (0,0,1.2) through (1.2,0,0)
    let p = Profile::new(f, ProfileLoop::new(edges));
    let r = revolve(&mut rep, &p, o, z, TAU).unwrap();
    check(
        &rep,
        r.body,
        4.0 / 3.0 * PI * 1.728,
        4.0 * PI * 1.44,
        "sphere",
    );
    // disc offset from the axis → torus
    let mut rep = BRep::new();
    let cf = Frame3 {
        origin: Point3::new(2.5, 0.0, 0.0),
        ..xz()
    };
    let p = Profile::new(xz(), ProfileLoop::circle(&cf, 0.7));
    let r = revolve(&mut rep, &p, o, z, TAU).unwrap();
    check(
        &rep,
        r.body,
        2.0 * PI * PI * 2.5 * 0.49,
        4.0 * PI * PI * 2.5 * 0.7,
        "torus",
    );
}

#[test]
fn partial_revolutions_and_tilted_axis() {
    let (o, z) = (Point3::origin(), Vector3::z());
    for frac in [0.25, 0.5, 0.8] {
        let mut rep = BRep::new();
        let p = Profile::polygon(xz(), &[(0.5, 0.0), (1.5, 0.0), (1.5, 1.0), (0.5, 1.0)]);
        let r = revolve(&mut rep, &p, o, z, TAU * frac).unwrap();
        let vol = frac * PI * (2.25 - 0.25) * 1.0;
        let area = frac * (2.0 * PI * (2.25 - 0.25) + TAU * 1.5 * 1.0 + TAU * 0.5 * 1.0)
            + 2.0 * (1.0 * 1.0);
        check(&rep, r.body, vol, area, &format!("partial {frac}"));
    }
    // quarter of a cone with the apex on the axis (degenerate edge in a partial sweep)
    let mut rep = BRep::new();
    let p = Profile::polygon(xz(), &[(0.0, 0.0), (1.0, 0.0), (0.0, 2.0)]);
    let r = revolve(&mut rep, &p, o, z, FRAC_PI_2).unwrap();
    check(
        &rep,
        r.body,
        0.25 * PI * 2.0 / 3.0,
        0.25 * (PI + PI * 5.0f64.sqrt()) + 2.0 * 1.0,
        "quarter cone",
    );
}

#[test]
fn bad_revolutions_are_errors() {
    let mut rep = BRep::new();
    let p = Profile::polygon(xz(), &[(-1.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-1.0, 1.0)]);
    assert!(
        revolve(&mut rep, &p, Point3::origin(), Vector3::z(), TAU).is_err(),
        "profile crosses the axis"
    );
    let p = Profile::polygon(xz(), &[(1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]);
    assert!(
        revolve(&mut rep, &p, Point3::origin(), Vector3::y(), TAU).is_err(),
        "axis not in the profile plane"
    );
    assert!(revolve(&mut rep, &p, Point3::origin(), Vector3::z(), 0.0).is_err());
}
