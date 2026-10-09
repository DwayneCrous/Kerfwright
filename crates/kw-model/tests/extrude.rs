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
use kw_nurbs::geom::Circle;
use kw_nurbs::CurveGeom;
use std::f64::consts::{PI, TAU};

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn plane() -> Frame3 {
    Frame3::from_origin_normal(Point3::new(0.5, -0.3, 0.2), Vector3::new(0.2, 0.3, 1.0))
}

fn check(rep: &BRep, body: kw_brep::BodyId, vol: f64, area: f64, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let v = rep.volume(body).unwrap();
    let a = rep.area(body).unwrap();
    assert!(rel(v, vol) < 1e-10, "{what}: volume {v} vs {vol}");
    assert!(rel(a, area) < 1e-10, "{what}: area {a} vs {area}");
    let t = kw_tess::tessellate_body(
        rep,
        body,
        &kw_tess::TessOptions {
            chord_tol: 0.005,
            angle_tol: 0.2,
        },
    )
    .unwrap();
    assert!(
        (t.mesh.signed_volume() - vol).abs() < 0.02 * vol,
        "{what}: mesh volume {}",
        t.mesh.signed_volume()
    );
}

#[test]
fn prism_with_both_orientations() {
    // extrusion up and down along the plane normal, and oblique
    for d in [Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, -1.0)] {
        let f = Frame3::world();
        let prof = Profile::polygon(
            f,
            &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 2.5), (0.0, 2.0)],
        );
        let mut rep = BRep::new();
        let r = extrude(&mut rep, &prof, d * 2.0, &ExtrudeOptions::default()).unwrap();
        // polygon area via shoelace
        let pts: [(f64, f64); 5] = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 2.5), (0.0, 2.0)];
        let mut a2: f64 = 0.0;
        for i in 0..5 {
            let (p, q) = (pts[i], pts[(i + 1) % 5]);
            a2 += p.0 * q.1 - q.0 * p.1;
        }
        let base = 0.5 * a2.abs();
        let perim: f64 = (0..5)
            .map(|i| {
                let (p, q) = (pts[i], pts[(i + 1) % 5]);
                ((q.0 - p.0).powi(2) + (q.1 - p.1).powi(2)).sqrt()
            })
            .sum();
        check(&rep, r.body, base * 2.0, 2.0 * base + perim * 2.0, "prism");
    }
    // tilted profile plane
    let prof = Profile::rectangle(plane(), 2.0, 3.0);
    let mut rep = BRep::new();
    let r = extrude(&mut rep, &prof, plane().z * 1.5, &ExtrudeOptions::default()).unwrap();
    check(&rep, r.body, 9.0, 2.0 * 6.0 + 2.0 * 5.0 * 1.5, "tilted box");
}

#[test]
fn cylinder_and_hole() {
    let f = plane();
    let mut rep = BRep::new();
    let disc = Profile::disc(f, 1.5);
    let r = extrude(&mut rep, &disc, f.z * 2.0, &ExtrudeOptions::default()).unwrap();
    check(
        &rep,
        r.body,
        PI * 2.25 * 2.0,
        2.0 * PI * 2.25 + 2.0 * PI * 1.5 * 2.0,
        "cylinder",
    );
    // reversed extrusion
    let mut rep = BRep::new();
    let r = extrude(&mut rep, &disc, -f.z * 2.0, &ExtrudeOptions::default()).unwrap();
    check(
        &rep,
        r.body,
        PI * 2.25 * 2.0,
        2.0 * PI * 2.25 + 2.0 * PI * 1.5 * 2.0,
        "cylinder down",
    );
    // plate with a round hole (hole winds opposite to the outer loop)
    let mut hole_frame = f;
    hole_frame.origin += f.x * 0.3;
    let hole = {
        // clockwise circle: reversed frame
        let hf = Frame3 {
            origin: hole_frame.origin,
            x: f.x,
            y: -f.y,
            z: -f.z,
        };
        kw_model::ProfileLoop::circle(&hf, 0.4)
    };
    let prof = Profile::rectangle(f, 4.0, 3.0).with_hole(hole);
    let mut rep = BRep::new();
    let r = extrude(&mut rep, &prof, f.z * 0.5, &ExtrudeOptions::default()).unwrap();
    let vol = (12.0 - PI * 0.16) * 0.5;
    let area = 2.0 * (12.0 - PI * 0.16) + 0.5 * (14.0 + TAU * 0.4);
    check(&rep, r.body, vol, area, "plate with hole");
    // symmetric extrusion keeps the same volume
    let mut rep = BRep::new();
    let r = extrude(
        &mut rep,
        &Profile::rectangle(f, 4.0, 3.0),
        f.z * 2.0,
        &ExtrudeOptions { symmetric: true },
    )
    .unwrap();
    check(
        &rep,
        r.body,
        24.0,
        24.0 + 2.0 * 14.0 * 2.0 / 2.0 * 1.0,
        "symmetric",
    );
}

#[test]
fn stadium_profile_with_arcs() {
    // slot: two lines and two semicircles
    let f = Frame3::world();
    let (r, l) = (0.8, 2.0);
    let left = Frame3 {
        origin: Point3::new(0.0, 0.0, 0.0),
        ..f
    };
    let right = Frame3 {
        origin: Point3::new(l, 0.0, 0.0),
        ..f
    };
    let edges = vec![
        ProfileEdge::line(Point3::new(0.0, -r, 0.0), Point3::new(l, -r, 0.0)),
        ProfileEdge::arc(&right, r, -PI / 2.0, PI / 2.0),
        ProfileEdge::line(Point3::new(l, r, 0.0), Point3::new(0.0, r, 0.0)),
        ProfileEdge::arc(&left, r, PI / 2.0, 3.0 * PI / 2.0),
    ];
    let prof = Profile::new(f, ProfileLoop::new(edges));
    let mut rep = BRep::new();
    let res = extrude(
        &mut rep,
        &prof,
        Vector3::new(0.0, 0.0, 1.2),
        &ExtrudeOptions::default(),
    )
    .unwrap();
    let base = 2.0 * r * l + PI * r * r;
    let perim = 2.0 * l + TAU * r;
    check(&rep, res.body, base * 1.2, 2.0 * base + perim * 1.2, "slot");
    let _ = CurveGeom::Circle(Circle {
        frame: f,
        radius: 1.0,
    });
}

#[test]
fn bad_input_is_an_error() {
    let f = Frame3::world();
    let mut rep = BRep::new();
    // direction in the plane
    let p = Profile::rectangle(f, 1.0, 1.0);
    assert!(extrude(
        &mut rep,
        &p,
        Vector3::new(1.0, 0.0, 0.0),
        &ExtrudeOptions::default()
    )
    .is_err());
    assert!(extrude(&mut rep, &p, Vector3::zeros(), &ExtrudeOptions::default()).is_err());
    // open loop
    let bad = Profile::new(
        f,
        ProfileLoop::new(vec![
            ProfileEdge::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ProfileEdge::line(Point3::new(2.0, 0.0, 0.0), Point3::origin()),
        ]),
    );
    assert!(extrude(&mut rep, &bad, Vector3::z(), &ExtrudeOptions::default()).is_err());
}
