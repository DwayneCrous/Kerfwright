//! Trimming surfaces with planes and other surfaces: areas against closed forms.
use kw_brep::builders::make_plane_sheet;
use kw_brep::{BRep, BodyId, BodyKind};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::geom::{Cylinder, Plane};
use kw_nurbs::SurfaceGeom;
use std::f64::consts::{PI, TAU};

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

fn plane_tool(origin: Point3<f64>, normal: Vector3<f64>) -> TrimTool {
    TrimTool {
        surface: SurfaceGeom::Plane(Plane {
            frame: Frame3::from_origin_normal(origin, normal),
        }),
        sign: 1.0,
    }
}

fn check(rep: &BRep, b: BodyId, area: f64, what: &str) {
    assert_eq!(rep.bodies[b].kind, BodyKind::Sheet, "{what}");
    assert!(
        rep.validate().is_ok(),
        "{what}: {:?}",
        rep.validate().errors
    );
    let a = rep.area(b).unwrap();
    assert!(
        (a - area).abs() < 2e-5 * area.max(1.0),
        "{what}: area {a} vs {area}"
    );
}

fn sheet(rep: &mut BRep) -> BodyId {
    make_plane_sheet(rep, &Frame3::world(), 6.0, 4.0).body
}

#[test]
fn a_plane_cuts_a_flat_sheet_either_way() {
    for (keep, area) in [(true, 8.0), (false, 16.0)] {
        let mut rep = BRep::new();
        let s = sheet(&mut rep);
        let r = trim_sheet(
            &mut rep,
            s,
            &plane_tool(p(1.0, 0.0, 0.0), Vector3::x()),
            keep,
        )
        .unwrap();
        check(&rep, r.body, area, "flat");
        assert_eq!(rep.body_faces(r.body).len(), 1);
        // four open edges: the sheet is still a rectangle
        assert_eq!(rep.free_edges(r.body).len(), 4);
    }
}

#[test]
fn a_slanted_plane_cuts_a_corner_off() {
    let mut rep = BRep::new();
    let s = sheet(&mut rep);
    // the line x + y = 4 cuts the corner (3, 2) off: a triangle with legs 1 and 1 = area 0.5
    let tool = plane_tool(p(2.0, 2.0, 0.0), Vector3::new(1.0, 1.0, 0.0));
    let r = trim_sheet(&mut rep, s, &tool, false).unwrap();
    check(&rep, r.body, 24.0 - 0.5, "corner removed");
    let r = trim_sheet(&mut rep, s, &tool, true).unwrap();
    check(&rep, r.body, 0.5, "corner kept");
}

fn tube(rep: &mut BRep, r: f64, h: f64) -> BodyId {
    let ProfileLoop { edges } = ProfileLoop::circle(&Frame3::world(), r);
    let chain = Chain::new(Frame3::world(), edges, true);
    extrude_sheet(
        rep,
        &[chain],
        Vector3::new(0.0, 0.0, h),
        &Default::default(),
    )
    .unwrap()
    .body
}

#[test]
fn a_plane_cuts_a_cylinder_across_its_seam() {
    let (r, h) = (2.0, 3.0);
    for (keep, area) in [(false, TAU * r * 1.0), (true, TAU * r * 2.0)] {
        let mut rep = BRep::new();
        let t = tube(&mut rep, r, h);
        let out = trim_sheet(
            &mut rep,
            t,
            &plane_tool(p(0.0, 0.0, 1.0), Vector3::z()),
            keep,
        )
        .unwrap();
        check(&rep, out.body, area, "cylinder");
        // the cut is a new open circle, the old rim the other
        // the old rim, and the cut (a circle, in two arcs where it crosses the seam)
        assert!(rep.free_edges(out.body).len() >= 2);
    }
}

#[test]
fn a_tilted_plane_cuts_a_cylinder_to_the_same_mean_height() {
    // z = 1.5 + 0.5 x: the edge height varies by +-1 round the cylinder, the area is that of height 1.5
    let mut rep = BRep::new();
    let t = tube(&mut rep, 2.0, 3.0);
    let n = Vector3::new(-0.5, 0.0, 1.0);
    let out = trim_sheet(&mut rep, t, &plane_tool(p(0.0, 0.0, 1.5), n), false).unwrap();
    check(&rep, out.body, TAU * 2.0 * 1.5, "tilted");
}

#[test]
fn a_plane_cuts_a_nurbs_frustum() {
    let mut rep = BRep::new();
    let section = |r: f64, z: f64| {
        let frame = Frame3 {
            origin: p(0.0, 0.0, z),
            ..Frame3::world()
        };
        let ProfileLoop { edges } = ProfileLoop::circle(&frame, r);
        vec![Chain::new(frame, edges, true)]
    };
    let f = loft_sheet(&mut rep, &[section(1.0, 0.0), section(2.0, 3.0)], false)
        .unwrap()
        .body;
    let slant = (1.5f64 * 1.5 + 0.5 * 0.5).sqrt();
    let tool = plane_tool(p(0.0, 0.0, 1.5), Vector3::z());
    let lower = trim_sheet(&mut rep, f, &tool, false).unwrap().body;
    let upper = trim_sheet(&mut rep, f, &tool, true).unwrap().body;
    // r(z) = 1 + z / 3
    check(&rep, lower, PI * (1.0 + 1.5) * slant, "lower");
    check(&rep, upper, PI * (1.5 + 2.0) * slant, "upper");
}

fn box_tube(rep: &mut BRep, w: f64, h: f64, d: f64) -> BodyId {
    let pts = [
        p(0.0, 0.0, 0.0),
        p(w, 0.0, 0.0),
        p(w, h, 0.0),
        p(0.0, h, 0.0),
    ];
    let chain = Chain::new(Frame3::world(), ProfileLoop::polygon(&pts).edges, true);
    extrude_sheet(
        rep,
        &[chain],
        Vector3::new(0.0, 0.0, d),
        &Default::default(),
    )
    .unwrap()
    .body
}

#[test]
fn a_cut_across_several_faces_keeps_their_shared_edges_shared() {
    let mut rep = BRep::new();
    let t = box_tube(&mut rep, 6.0, 5.0, 4.0);
    let out = trim_sheet(
        &mut rep,
        t,
        &plane_tool(p(0.0, 0.0, 1.5), Vector3::z()),
        false,
    )
    .unwrap();
    check(&rep, out.body, 2.0 * (6.0 + 5.0) * 1.5, "box tube");
    assert_eq!(rep.body_faces(out.body).len(), 4);
    // the bottom rim and the new top rim: 8 open edges, the 4 vertical edges are shared
    assert_eq!(rep.free_edges(out.body).len(), 8);
    // a plane in the other direction through a face only
    let out = trim_sheet(
        &mut rep,
        t,
        &plane_tool(p(2.0, 0.0, 0.0), Vector3::x()),
        true,
    )
    .unwrap();
    check(
        &rep,
        out.body,
        // x from 2 to 6: two walls 4 wide, one 5 wide, all 4 high
        2.0 * 4.0 * 4.0 + 5.0 * 4.0,
        "x cut",
    );
}

#[test]
fn a_cylinder_tool_makes_a_disc_or_a_plate_with_a_hole() {
    // a 10 x 10 plate, a cylinder of radius 2 standing on it
    let tool = TrimTool {
        surface: SurfaceGeom::Cylinder(Cylinder {
            frame: Frame3::world(),
            radius: 2.0,
        }),
        // positive side = outside the cylinder
        sign: 1.0,
    };
    let mut rep = BRep::new();
    let plate = make_plane_sheet(&mut rep, &Frame3::world(), 10.0, 10.0).body;
    let outside = trim_sheet(&mut rep, plate, &tool, true).unwrap();
    check(&rep, outside.body, 100.0 - PI * 4.0, "plate with a hole");
    let disc = trim_sheet(&mut rep, plate, &tool, false).unwrap();
    check(&rep, disc.body, PI * 4.0, "disc");
}

#[test]
fn trims_that_do_nothing_or_take_everything_are_refused() {
    let mut rep = BRep::new();
    let s = sheet(&mut rep);
    // beside the sheet: nothing to cut, and keeping the far side removes it
    let far = plane_tool(p(10.0, 0.0, 0.0), Vector3::x());
    assert!(trim_sheet(&mut rep, s, &far, false).is_err());
    assert!(trim_sheet(&mut rep, s, &far, true).is_err());
    // through a vertex
    let corner = plane_tool(p(3.0, 0.0, 0.0), Vector3::x());
    assert!(trim_sheet(&mut rep, s, &corner, true).is_err());
    // a solid cannot be trimmed
    let b = kw_brep::make_box(&mut rep, &Frame3::world(), 1.0, 1.0, 1.0).body;
    assert!(trim_sheet(&mut rep, b, &far, true).is_err());
}
