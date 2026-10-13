//! Surfaces (sheet bodies) from open and closed chains: areas against closed forms.
use kw_brep::{BRep, BodyId, BodyKind};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::geom::Circle;
use kw_nurbs::CurveGeom;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn world() -> Frame3 {
    Frame3::world()
}

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

fn chain(edges: Vec<ProfileEdge>, closed: bool) -> Chain {
    Chain::new(world(), edges, closed)
}

fn line(a: Point3<f64>, b: Point3<f64>) -> Chain {
    chain(vec![ProfileEdge::line(a, b)], false)
}

/// The body is a valid surface with `area` and `free` open edges, and encloses no volume.
fn check(rep: &BRep, b: BodyId, area: f64, free: usize, what: &str) {
    assert_eq!(rep.bodies[b].kind, BodyKind::Sheet, "{what}");
    assert!(
        rep.validate().is_ok(),
        "{what}: {:?}",
        rep.validate().errors
    );
    let a = rep.area(b).unwrap();
    assert!(
        (a - area).abs() < 1e-6 * area.max(1.0),
        "{what}: area {a} vs {area}"
    );
    assert_eq!(rep.free_edges(b).len(), free, "{what}: free edges");
    assert_eq!(rep.volume(b), None, "{what}");
}

#[test]
fn an_open_chain_extrudes_into_a_strip() {
    let mut rep = BRep::new();
    let c = line(p(0.0, 0.0, 0.0), p(3.0, 0.0, 0.0));
    let r = extrude_sheet(
        &mut rep,
        &[c],
        Vector3::new(0.0, 0.0, 2.0),
        &Default::default(),
    )
    .unwrap();
    check(&rep, r.body, 6.0, 4, "strip");
    assert_eq!(rep.body_faces(r.body).len(), 1);
}

#[test]
fn a_bent_open_chain_extrudes_one_face_per_segment() {
    let mut rep = BRep::new();
    let c = chain(
        vec![
            ProfileEdge::line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0)),
            ProfileEdge::line(p(2.0, 0.0, 0.0), p(2.0, 3.0, 0.0)),
        ],
        false,
    );
    let r = extrude_sheet(
        &mut rep,
        &[c],
        Vector3::new(0.0, 0.0, 4.0),
        &Default::default(),
    )
    .unwrap();
    check(&rep, r.body, 20.0, 6, "bent strip");
    assert_eq!(rep.body_faces(r.body).len(), 2);
}

#[test]
fn a_closed_chain_extrudes_into_an_open_tube() {
    let mut rep = BRep::new();
    let pts = [
        p(0.0, 0.0, 0.0),
        p(2.0, 0.0, 0.0),
        p(2.0, 3.0, 0.0),
        p(0.0, 3.0, 0.0),
    ];
    let c = chain(ProfileLoop::polygon(&pts).edges, true);
    let r = extrude_sheet(
        &mut rep,
        &[c],
        Vector3::new(0.0, 0.0, 4.0),
        &Default::default(),
    )
    .unwrap();
    // two rings of four edges are open; the four verticals are shared
    check(&rep, r.body, 40.0, 8, "tube");
}

#[test]
fn a_circle_extrudes_into_a_cylinder_surface() {
    let mut rep = BRep::new();
    let ProfileLoop { edges } = ProfileLoop::circle(&world(), 1.5);
    let r = extrude_sheet(
        &mut rep,
        &[chain(edges, true)],
        Vector3::new(0.0, 0.0, 2.0),
        &ExtrudeOptions { symmetric: true },
    )
    .unwrap();
    check(&rep, r.body, TAU * 1.5 * 2.0, 2, "cylinder");
    let bb = rep.body_bbox(r.body);
    // the box is padded around curved faces, but it must reach the ends
    assert!(bb.min.z <= -1.0 + 1e-9 && bb.max.z >= 1.0 - 1e-9, "{bb:?}");
}

#[test]
fn a_chain_in_the_extrusion_direction_is_refused() {
    let mut rep = BRep::new();
    let c = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    assert!(extrude_sheet(
        &mut rep,
        &[c],
        Vector3::new(1.0, 0.0, 0.0),
        &Default::default()
    )
    .is_err());
}

/// The profile plane that contains the z axis, with x along the world x axis.
fn xz() -> Frame3 {
    Frame3 {
        origin: p(0.0, 0.0, 0.0),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    }
}

#[test]
fn a_line_revolves_into_a_cylinder_cone_or_disc() {
    let z = Vector3::z();
    let o = p(0.0, 0.0, 0.0);
    for (a, b, area, what) in [
        (p(1.0, 0.0, 0.0), p(1.0, 0.0, 2.0), TAU * 2.0, "cylinder"),
        (
            p(1.0, 0.0, 0.0),
            p(3.0, 0.0, 2.0),
            PI * (1.0 + 3.0) * (8.0_f64).sqrt(),
            "cone",
        ),
        (
            p(1.0, 0.0, 1.0),
            p(3.0, 0.0, 1.0),
            PI * (9.0 - 1.0),
            "annulus",
        ),
    ] {
        let mut rep = BRep::new();
        let c = Chain::new(xz(), vec![ProfileEdge::line(a, b)], false);
        let r = revolve_sheet(&mut rep, &[c], o, z, TAU).unwrap();
        check(&rep, r.body, area, 2, what);
    }
}

#[test]
fn a_partial_revolution_is_open_at_its_sides() {
    let mut rep = BRep::new();
    let c = Chain::new(
        xz(),
        vec![ProfileEdge::line(p(1.0, 0.0, 0.0), p(1.0, 0.0, 2.0))],
        false,
    );
    let r = revolve_sheet(&mut rep, &[c], p(0.0, 0.0, 0.0), Vector3::z(), PI).unwrap();
    check(&rep, r.body, PI * 2.0, 4, "half cylinder");
}

#[test]
fn an_arc_revolves_into_a_sphere_zone() {
    let mut rep = BRep::new();
    // a quarter circle of radius 2 from the pole down to the equator
    let frame = Frame3 {
        origin: p(0.0, 0.0, 0.0),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    };
    let arc = ProfileEdge::arc(&frame, 2.0, 0.0, FRAC_PI_2);
    let c = Chain::new(xz(), vec![arc], false);
    let r = revolve_sheet(&mut rep, &[c], p(0.0, 0.0, 0.0), Vector3::z(), TAU).unwrap();
    // a hemisphere: 2 pi r^2
    check(&rep, r.body, 2.0 * PI * 4.0, 1, "hemisphere");
}

fn section(r: f64, z: f64) -> Vec<Chain> {
    let frame = Frame3 {
        origin: p(0.0, 0.0, z),
        ..world()
    };
    let ProfileLoop { edges } = ProfileLoop::circle(&frame, r);
    vec![Chain::new(frame, edges, true)]
}

#[test]
fn a_loft_between_circles_is_a_frustum_surface() {
    let mut rep = BRep::new();
    let r = loft_sheet(&mut rep, &[section(1.0, 0.0), section(2.0, 3.0)], false).unwrap();
    let slant = (9.0f64 + 1.0).sqrt();
    check(&rep, r.body, PI * 3.0 * slant, 2, "frustum");
}

#[test]
fn a_ruled_loft_through_three_sections_is_straight_between_them() {
    let mut rep = BRep::new();
    let secs = [section(1.0, 0.0), section(2.0, 1.0), section(1.0, 3.0)];
    let r = loft_sheet(&mut rep, &secs, true).unwrap();
    let area = PI * 3.0 * 2.0f64.sqrt() + PI * 3.0 * (4.0 + 1.0f64).sqrt();
    check(&rep, r.body, area, 2, "ruled");
}

#[test]
fn a_loft_of_open_chains_is_a_strip() {
    let mut rep = BRep::new();
    let secs = [
        vec![line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0))],
        vec![line(p(0.0, 0.0, 3.0), p(2.0, 0.0, 3.0))],
    ];
    let r = loft_sheet(&mut rep, &secs, false).unwrap();
    check(&rep, r.body, 6.0, 4, "flat loft");
}

#[test]
fn lofted_sections_must_match() {
    let mut rep = BRep::new();
    let secs = [
        vec![line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0))],
        section(1.0, 3.0),
    ];
    assert!(loft_sheet(&mut rep, &secs, false).is_err());
}

fn seg(a: Point3<f64>, b: Point3<f64>) -> CurveGeom {
    CurveGeom::Nurbs(
        kw_nurbs::NurbsCurve::new_polynomial(
            kw_nurbs::KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
            vec![a, b],
        )
        .unwrap(),
    )
}

/// A straight path along z (a degree-1 curve over [0, 1], since a bare line has no finite domain).
fn path(h: f64) -> CurveGeom {
    CurveGeom::Nurbs(
        kw_nurbs::NurbsCurve::new_polynomial(
            kw_nurbs::KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
            vec![p(0.0, 0.0, 0.0), p(0.0, 0.0, h)],
        )
        .unwrap(),
    )
}

#[test]
fn a_sweep_along_a_straight_path_is_an_extrusion() {
    let mut rep = BRep::new();
    let c = line(p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0));
    let r = sweep_sheet(&mut rep, &[c], &path(5.0), &SweepOptions::default()).unwrap();
    check(&rep, r.body, 10.0, 4, "swept strip");
}

#[test]
fn a_twisted_sweep_is_a_helicoid() {
    let (w, h, theta) = (1.5f64, 4.0f64, FRAC_PI_2);
    let mut rep = BRep::new();
    let c = line(p(0.0, 0.0, 0.0), p(w, 0.0, 0.0));
    let opts = SweepOptions {
        twist: theta,
        ..Default::default()
    };
    let r = sweep_sheet(&mut rep, &[c], &path(h), &opts).unwrap();
    let k = theta / h;
    let area = h * 0.5 * (w * (1.0 + k * k * w * w).sqrt() + (k * w).asinh() / k);
    let got = rep.area(r.body).unwrap();
    assert!((got - area).abs() < 2e-4 * area, "helicoid {got} vs {area}");
    assert_eq!(rep.bodies[r.body].kind, BodyKind::Sheet);
    // the far end really is turned a quarter
    let bb = rep.body_bbox(r.body);
    assert!(bb.max.y > w - 1e-6, "{bb:?}");
}

#[test]
fn a_guide_rail_scales_the_section_along_the_path() {
    let h = 4.0;
    let mut rep = BRep::new();
    // the section reaches the guide at distance 1 from the path, and the guide moves out to 3
    let c = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    let guide = seg(p(1.0, 0.0, 0.0), p(3.0, 0.0, h));
    let opts = SweepOptions {
        guide: Some(guide),
        ..Default::default()
    };
    let r = sweep_sheet(&mut rep, &[c], &path(h), &opts).unwrap();
    // a trapezoid in the x-z plane: widths 1 to 3 over the height
    check(&rep, r.body, 2.0 * h, 4, "guided");
    let bb = rep.body_bbox(r.body);
    assert!(bb.max.x > 2.99 && bb.max.x < 3.5, "{bb:?}");
}

#[test]
fn a_guide_that_misses_the_sections_is_refused() {
    let mut rep = BRep::new();
    let c = line(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
    let guide = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: p(0.0, 0.0, 100.0),
            ..world()
        },
        radius: 1.0,
    });
    let opts = SweepOptions {
        guide: Some(guide),
        ..Default::default()
    };
    assert!(sweep_sheet(&mut rep, &[c], &path(4.0), &opts).is_err());
}

#[test]
fn a_twisted_solid_sweep_still_closes() {
    // the twist and guide options apply to solids as well
    let mut rep = BRep::new();
    let prof = Profile::rectangle(world(), 1.0, 0.5);
    let opts = SweepOptions {
        twist: PI,
        ..Default::default()
    };
    let r = sweep(&mut rep, &prof, &path(3.0), &opts).unwrap();
    assert!(rep.validate().is_ok());
    // a twist does not change the volume of a straight sweep
    assert!((rep.volume(r.body).unwrap() - 1.0 * 0.5 * 3.0).abs() < 1e-4);
}
