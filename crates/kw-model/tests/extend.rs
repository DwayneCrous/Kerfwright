//! Extending surfaces past their open edges: areas against closed forms.
use kw_brep::builders::make_plane_sheet;
use kw_brep::{BRep, BodyId, BodyKind, EdgeId};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::{PI, TAU};

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

/// The open edge of the body whose two ends satisfy `at`.
fn open_edge(rep: &BRep, b: BodyId, at: impl Fn(&Point3<f64>) -> bool) -> EdgeId {
    rep.free_edges(b)
        .into_iter()
        .find(|e| {
            let ed = &rep.edges[*e];
            at(&rep.vertices[ed.start].point) && at(&rep.vertices[ed.end].point)
        })
        .expect("an open edge there")
}

fn check(rep: &BRep, b: BodyId, area: f64, what: &str) {
    assert_eq!(rep.bodies[b].kind, BodyKind::Sheet, "{what}");
    assert!(
        rep.validate().is_ok(),
        "{what}: {:?}",
        rep.validate().errors
    );
    let a = rep.area(b).unwrap();
    assert!((a - area).abs() < 1e-5 * area, "{what}: area {a} vs {area}");
}

#[test]
fn a_flat_sheet_grows_by_a_strip_on_one_side() {
    let mut rep = BRep::new();
    let s = make_plane_sheet(&mut rep, &Frame3::world(), 6.0, 4.0).body;
    let e = open_edge(&rep, s, |q| (q.x - 3.0).abs() < 1e-9);
    let r = extend_sheet(&mut rep, s, &[e], 2.0).unwrap();
    check(&rep, r.body, 24.0 + 2.0 * 4.0, "plane");
    assert_eq!(rep.body_faces(r.body).len(), 2);
    // three old open edges and three of the strip
    assert_eq!(rep.free_edges(r.body).len(), 6);
    // two sides at once
    let e2 = open_edge(&rep, s, |q| (q.x + 3.0).abs() < 1e-9);
    let r = extend_sheet(&mut rep, s, &[e, e2], 1.0).unwrap();
    check(&rep, r.body, 24.0 + 2.0 * 4.0 * 1.0, "both sides");
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
fn a_tube_is_carried_on_at_either_rim() {
    let mut rep = BRep::new();
    let t = tube(&mut rep, 2.0, 3.0);
    for z in [3.0, 0.0] {
        let e = open_edge(&rep, t, |q| (q.z - z).abs() < 1e-9);
        let r = extend_sheet(&mut rep, t, &[e], 2.0).unwrap();
        check(&rep, r.body, TAU * 2.0 * 5.0, "tube");
        let bb = rep.body_bbox(r.body);
        if z > 1.0 {
            assert!(bb.max.z > 4.9, "{bb:?}");
        } else {
            assert!(bb.min.z < -1.9, "{bb:?}");
        }
    }
}

fn frustum(rep: &mut BRep) -> BodyId {
    let section = |r: f64, z: f64| {
        let frame = Frame3 {
            origin: p(0.0, 0.0, z),
            ..Frame3::world()
        };
        let ProfileLoop { edges } = ProfileLoop::circle(&frame, r);
        vec![Chain::new(frame, edges, true)]
    };
    loft_sheet(rep, &[section(1.0, 0.0), section(2.0, 3.0)], false)
        .unwrap()
        .body
}

#[test]
fn a_nurbs_cone_continues_along_its_slant_at_both_ends() {
    let slant = 10.0f64.sqrt();
    let mut rep = BRep::new();
    let f = frustum(&mut rep);
    // the top rim: radius 2 grows by 1/slant per unit of slant
    let e = open_edge(&rep, f, |q| (q.z - 3.0).abs() < 1e-9);
    let l = 1.5;
    let r = extend_sheet(&mut rep, f, &[e], l).unwrap();
    let top = 2.0 + l / slant;
    check(&rep, r.body, PI * 3.0 * slant + PI * (2.0 + top) * l, "top");
    // the bottom rim: the radius shrinks, but stays positive for a short extension
    let e = open_edge(&rep, f, |q| q.z.abs() < 1e-9);
    let r = extend_sheet(&mut rep, f, &[e], 0.5).unwrap();
    let bottom = 1.0 - 0.5 / slant;
    check(
        &rep,
        r.body,
        PI * 3.0 * slant + PI * (1.0 + bottom) * 0.5,
        "bottom",
    );
}

#[test]
fn a_patch_is_extended_as_the_plane_it_is() {
    let mut rep = BRep::new();
    let pts = [
        p(0.0, 0.0, 1.0),
        p(6.0, 0.0, 1.0),
        p(6.0, 4.0, 1.0),
        p(0.0, 4.0, 1.0),
    ];
    let edges: Vec<PatchEdge> = (0..4)
        .map(|i| PatchEdge::new(ProfileEdge::line(pts[i], pts[(i + 1) % 4])))
        .collect();
    let s = patch_sheet(&mut rep, &edges).unwrap().body;
    let e = open_edge(&rep, s, |q| (q.y - 4.0).abs() < 1e-6);
    let r = extend_sheet(&mut rep, s, &[e], 1.5).unwrap();
    check(&rep, r.body, 24.0 + 6.0 * 1.5, "patch");
    // the strip lies in the same plane
    let bb = rep.body_bbox(r.body);
    assert!(bb.max.y > 5.4);
}

#[test]
fn edges_that_cannot_be_extended_say_so() {
    let mut rep = BRep::new();
    let t = tube(&mut rep, 2.0, 3.0);
    // the seam is shared by the face with itself, not an open edge
    let seam = rep
        .edges
        .keys()
        .find(|e| rep.radial_ring(*e).len() == 2)
        .unwrap();
    assert!(extend_sheet(&mut rep, t, &[seam], 1.0).is_err());
    let rim = open_edge(&rep, t, |q| (q.z - 3.0).abs() < 1e-9);
    assert!(extend_sheet(&mut rep, t, &[rim], 0.0).is_err());
    assert!(extend_sheet(&mut rep, t, &[], 1.0).is_err());
    let b = kw_brep::make_box(&mut rep, &Frame3::world(), 1.0, 1.0, 1.0).body;
    assert!(extend_sheet(&mut rep, b, &[rim], 1.0).is_err());
}
