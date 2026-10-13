#![allow(unused_imports, dead_code)]
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::geom::Circle;
use kw_nurbs::CurveGeom;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}
fn at_z(z: f64) -> Frame3 {
    Frame3 {
        origin: Point3::new(0.0, 0.0, z),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}
fn mesh_ok(rep: &BRep, b: BodyId, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let v = rep.volume(b).unwrap();
    let t = kw_tess::tessellate_body(
        rep,
        b,
        &kw_tess::TessOptions {
            chord_tol: 0.003,
            angle_tol: 0.15,
        },
    )
    .unwrap_or_else(|e| panic!("{what}: {e:?}"));
    assert!(
        (t.mesh.signed_volume() - v).abs() < 0.03 * v.abs(),
        "{what}: mesh {} vs {v}",
        t.mesh.signed_volume()
    );
}

#[test]
fn loft_pyramid_frustum_and_cone_frustum() {
    let mut rep = BRep::new();
    let a = Profile::rectangle(at_z(0.0), 4.0, 2.0);
    let b = Profile::rectangle(at_z(3.0), 2.0, 1.0);
    let r = loft(&mut rep, &[a, b]).unwrap();
    let (a1, a2) = (8.0f64, 2.0f64);
    assert!(
        rel(
            rep.volume(r.body).unwrap(),
            3.0 / 3.0 * (a1 + a2 + (a1 * a2).sqrt())
        ) < 1e-9,
        "{}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "pyramid frustum");
    let mut rep = BRep::new();
    let c1 = Profile::disc(at_z(0.0), 2.0);
    let c2 = Profile::disc(at_z(3.0), 1.0);
    let r = loft(&mut rep, &[c1, c2]).unwrap();
    assert!(
        rel(
            rep.volume(r.body).unwrap(),
            PI * 3.0 / 3.0 * (4.0 + 2.0 + 1.0)
        ) < 1e-9,
        "{}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "cone frustum");
}

#[test]
fn loft_three_sections_is_smooth_and_exact_in_radius() {
    let mut rep = BRep::new();
    let secs = [
        Profile::disc(at_z(0.0), 1.0),
        Profile::disc(at_z(1.0), 1.6),
        Profile::disc(at_z(2.0), 0.8),
    ];
    let r = loft(&mut rep, &secs).unwrap();
    // radius is the quadratic through (0,1) (1,1.6) (2,0.8): r(z) = 1 + 1.0 z - 0.4 z²... solve
    let (a0, a1, a2) = (1.0, 1.6, 0.8);
    let c = 0.5 * (a2 - 2.0 * a1 + a0);
    let b = a1 - a0 - c;
    let vol = PI
        * (0..4000)
            .map(|i| {
                let z = 2.0 * (i as f64 + 0.5) / 4000.0;
                let r = a0 + b * z + c * z * z;
                r * r
            })
            .sum::<f64>()
        * 2.0
        / 4000.0;
    assert!(
        rel(rep.volume(r.body).unwrap(), vol) < 1e-6,
        "{} vs {vol}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "barrel");
}

#[test]
fn sweep_straight_and_along_an_arc() {
    // circle swept along a line is a cylinder
    let mut rep = BRep::new();
    let prof = Profile::disc(at_z(0.0), 0.7);
    let path = CurveGeom::Line(kw_nurbs::Line {
        origin: Point3::origin(),
        dir: Vector3::new(0.0, 0.0, 3.0),
    });
    let path = {
        // domain of a Line is infinite: use a NURBS line
        let c = kw_nurbs::NurbsCurve::new_polynomial(
            kw_nurbs::KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
            vec![Point3::origin(), Point3::new(0.0, 0.0, 3.0)],
        )
        .unwrap();
        let _ = path;
        CurveGeom::Nurbs(c)
    };
    let r = sweep(&mut rep, &prof, &path, &SweepOptions::default()).unwrap();
    assert!(
        rel(rep.volume(r.body).unwrap(), PI * 0.49 * 3.0) < 1e-6,
        "{}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "straight sweep");

    // disc swept along a quarter circle of radius 4: Pappus
    let mut rep = BRep::new();
    // profile in the plane x = 4 (path starts at (4,0,0) heading +y)
    let pf = Frame3 {
        origin: Point3::new(4.0, 0.0, 0.0),
        x: Vector3::z(),
        y: Vector3::x(),
        z: Vector3::y(),
    };
    let prof = Profile::disc(pf, 0.5);
    let arc = kw_nurbs::build::make_arc(&Frame3::world(), 4.0, 0.0, FRAC_PI_2).unwrap();
    let r = sweep(
        &mut rep,
        &prof,
        &CurveGeom::Nurbs(arc),
        &SweepOptions::default(),
    )
    .unwrap();
    let want = PI * 0.25 * 4.0 * FRAC_PI_2;
    assert!(
        rel(rep.volume(r.body).unwrap(), want) < 1e-4,
        "{} vs {want}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "bend");
    // square swept along the arc as well
    let mut rep = BRep::new();
    let sq = Profile::polygon(pf, &[(-0.4, -0.3), (0.4, -0.3), (0.4, 0.3), (-0.4, 0.3)]);
    let arc = kw_nurbs::build::make_arc(&Frame3::world(), 4.0, 0.0, FRAC_PI_2).unwrap();
    let r = sweep(
        &mut rep,
        &sq,
        &CurveGeom::Nurbs(arc),
        &SweepOptions::default(),
    )
    .unwrap();
    // centroid of the square sits on the path: Pappus with the centroid radius 4 (offsets are symmetric)
    let want = 0.8 * 0.6 * 4.0 * FRAC_PI_2;
    assert!(
        rel(rep.volume(r.body).unwrap(), want) < 1e-4,
        "{} vs {want}",
        rep.volume(r.body).unwrap()
    );
    mesh_ok(&rep, r.body, "square bend");
}

#[test]
fn bad_lofts_are_errors() {
    let mut rep = BRep::new();
    assert!(loft(&mut rep, &[Profile::disc(at_z(0.0), 1.0)]).is_err());
    assert!(loft(
        &mut rep,
        &[
            Profile::disc(at_z(0.0), 1.0),
            Profile::rectangle(at_z(1.0), 1.0, 1.0)
        ]
    )
    .is_err());
    assert!(loft(
        &mut rep,
        &[Profile::disc(at_z(1.0), 1.0), Profile::disc(at_z(1.0), 2.0)]
    )
    .is_err());
}
