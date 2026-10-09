#![allow(unused_imports, dead_code)]
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::PI;

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}
fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}
fn mesh_ok(rep: &BRep, b: BodyId, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let v = rep.volume(b).unwrap();
    let t = kw_tess::tessellate_body(
        rep,
        b,
        &kw_tess::TessOptions {
            chord_tol: 0.004,
            angle_tol: 0.2,
        },
    )
    .unwrap_or_else(|e| panic!("{what}: {e:?}"));
    assert!(
        (t.mesh.signed_volume() - v).abs() < 0.02 * v.abs(),
        "{what}: mesh {} vs {v}",
        t.mesh.signed_volume()
    );
}

#[test]
fn offset_of_primitives_matches_closed_forms() {
    let f = Frame3::from_origin_normal(Point3::new(0.5, -0.3, 0.2), Vector3::new(0.2, 0.3, 1.0));
    let mut rep = BRep::new();
    // box: mitred offset adds d to every half-extent
    let b = make_box(&mut rep, &f, 2.0, 3.0, 4.0).body;
    for d in [0.3, -0.3] {
        let r = offset_body(&mut rep, b, d).unwrap();
        let want = (2.0 + 2.0 * d) * (3.0 + 2.0 * d) * (4.0 + 2.0 * d);
        assert!(
            rel(rep.volume(r.body).unwrap(), want) < 1e-9,
            "box d={d}: {} vs {want}",
            rep.volume(r.body).unwrap()
        );
        mesh_ok(&rep, r.body, "box offset");
    }
    // cylinder: radius and half-height grow by d
    let c = make_cylinder(&mut rep, &f, 1.5, 2.0).body;
    for d in [0.25, -0.25] {
        let r = offset_body(&mut rep, c, d).unwrap();
        let want = PI * (1.5 + d) * (1.5 + d) * (2.0 + 2.0 * d);
        assert!(
            rel(rep.volume(r.body).unwrap(), want) < 1e-9,
            "cyl d={d}: {} vs {want}",
            rep.volume(r.body).unwrap()
        );
        mesh_ok(&rep, r.body, "cylinder offset");
    }
    // sphere & torus
    let s = make_sphere(&mut rep, &f, 1.5).body;
    let r = offset_body(&mut rep, s, 0.4).unwrap();
    assert!(rel(rep.volume(r.body).unwrap(), 4.0 / 3.0 * PI * 1.9f64.powi(3)) < 1e-9);
    let t = make_torus(&mut rep, &f, 3.0, 0.8).body;
    let r = offset_body(&mut rep, t, -0.3).unwrap();
    assert!(rel(rep.volume(r.body).unwrap(), 2.0 * PI * PI * 3.0 * 0.25) < 1e-9);
    // frustum (cone)
    let k = make_cone(&mut rep, &f, 2.0, 1.0, 3.0).body;
    let r = offset_body(&mut rep, k, -0.2).unwrap();
    assert!(rep.validate().is_ok() && rep.volume(r.body).unwrap() < rep.volume(k).unwrap());
    mesh_ok(&rep, r.body, "cone offset");
}

#[test]
fn offset_that_collapses_is_an_error() {
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &world(), 2.0, 2.0, 2.0).body;
    assert!(offset_body(&mut rep, b, -1.2).is_err());
    assert!(offset_body(&mut rep, b, 0.0).is_err());
}

#[test]
fn shell_closed_and_open_boxes() {
    let f = Frame3 {
        origin: Point3::new(1.0, 2.0, 0.5),
        ..world()
    };
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &f, 4.0, 3.0, 2.0).body;
    let t = 0.2;
    // closed hollow box
    let r = shell(&mut rep, b, t, &[]).unwrap();
    let want = 24.0 - (4.0 - 2.0 * t) * (3.0 - 2.0 * t) * (2.0 - 2.0 * t);
    assert!(
        rel(rep.volume(r.body).unwrap(), want) < 1e-9,
        "{} vs {want}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "closed shell");
    // open on the +z face
    let top = rep.body_faces(b).into_iter().find(|fc| {
        let face = &rep.faces[*fc];
        match &rep.surfaces[face.surface] {
            kw_nurbs::SurfaceGeom::Plane(p) => {
                (p.frame.z
                    * if face.sense == Sense::Forward {
                        1.0
                    } else {
                        -1.0
                    })
                .dot(&Vector3::z())
                    > 0.99
            }
            _ => false,
        }
    });
    let top = top.unwrap();
    let r = shell(&mut rep, b, t, &[top]).unwrap();
    let want = 24.0 - (4.0 - 2.0 * t) * (3.0 - 2.0 * t) * (2.0 - t);
    assert!(
        rel(rep.volume(r.body).unwrap(), want) < 1e-9,
        "open: {} vs {want}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "open shell");
    // cylinder cup
    let c = make_cylinder(&mut rep, &world(), 1.5, 2.0).body;
    let ctop = rep
        .body_faces(c)
        .into_iter()
        .find(|fc| matches!(&rep.surfaces[rep.faces[*fc].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.origin.z > 1.0))
        .unwrap();
    let r = shell(&mut rep, c, 0.25, &[ctop]).unwrap();
    let want = PI * 2.25 * 2.0 - PI * 1.25 * 1.25 * (2.0 - 0.25);
    assert!(
        rel(rep.volume(r.body).unwrap(), want) < 1e-9,
        "cup: {} vs {want}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "cup");
}

#[test]
fn draft_narrows_walls_and_bosses_consistently() {
    let mut rep = BRep::new();
    let b = make_box(
        &mut rep,
        &Frame3 {
            origin: Point3::new(0.0, 0.0, 1.0),
            ..world()
        },
        4.0,
        3.0,
        2.0,
    )
    .body; // z in [0,2]
    let sides: Vec<FaceId> = rep
        .body_faces(b)
        .into_iter()
        .filter(|fc| matches!(&rep.surfaces[rep.faces[*fc].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.z.dot(&Vector3::z()).abs() < 0.5))
        .collect();
    assert_eq!(sides.len(), 4);
    let a = 5.0f64.to_radians();
    let r = draft_faces(&mut rep, b, &sides, Point3::origin(), Vector3::z(), a).unwrap();
    // walls lean inwards by tan(a) per unit height; volume of the frustum-like solid
    let (w, d, h) = (4.0, 3.0, 2.0);
    let k = a.tan();
    let want = (0..1)
        .map(|_| h * (w * d - k * h * (w + d) + k * k * h * h / 3.0 * 1.0))
        .sum::<f64>();
    let _ = want;
    // exact: ∫0^h (w - 2kz)(d - 2kz) dz
    let exact = w * d * h - k * h * h * (w + d) + 4.0 * k * k * h * h * h / 3.0;
    assert!(
        rel(rep.volume(r.body).unwrap(), exact) < 1e-9,
        "{} vs {exact}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "drafted box");
    // cylinder boss drafted the same way is a cone: radius shrinks with height at tan(a)
    let c = make_cylinder(&mut rep, &world(), 1.5, 2.0).body;
    let side = rep
        .body_faces(c)
        .into_iter()
        .find(|fc| {
            matches!(
                &rep.surfaces[rep.faces[*fc].surface],
                kw_nurbs::SurfaceGeom::Cylinder(_)
            )
        })
        .unwrap();
    let r = draft_faces(&mut rep, c, &[side], Point3::origin(), Vector3::z(), a).unwrap();
    // frustum volume: r0 = 1.5, r1 = 1.5 - 2k
    let (r0, r1) = (1.5, 1.5 - 2.0 * k);
    let exact = PI * 2.0 / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
    assert!(
        rel(rep.volume(r.body).unwrap(), exact) < 1e-9,
        "{} vs {exact}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "drafted cylinder");
    // a face perpendicular to the pull direction cannot be drafted
    let bottom = rep.body_faces(b).into_iter().find(|fc| matches!(&rep.surfaces[rep.faces[*fc].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.z.dot(&Vector3::z()).abs() > 0.9)).unwrap();
    assert!(draft_faces(&mut rep, b, &[bottom], Point3::origin(), Vector3::z(), a).is_err());
}
