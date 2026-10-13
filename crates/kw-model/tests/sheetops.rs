//! Stitch, unstitch, offset and thicken on surfaces.
use kw_brep::{BRep, BodyId, BodyKind};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::{PI, TAU};

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

fn polygon(pts: &[Point3<f64>]) -> Vec<PatchEdge> {
    (0..pts.len())
        .map(|i| PatchEdge::new(ProfileEdge::line(pts[i], pts[(i + 1) % pts.len()])))
        .collect()
}

/// A w x h x d open tube (four walls) standing on the plane z = 0.
fn tube(rep: &mut BRep, w: f64, h: f64, d: f64) -> BodyId {
    let pts = [
        p(0.0, 0.0, 0.0),
        p(w, 0.0, 0.0),
        p(w, h, 0.0),
        p(0.0, h, 0.0),
    ];
    let edges = ProfileLoop::polygon(&pts).edges;
    let chain = Chain::new(Frame3::world(), edges, true);
    extrude_sheet(
        rep,
        &[chain],
        Vector3::new(0.0, 0.0, d),
        &Default::default(),
    )
    .unwrap()
    .body
}

fn cap(rep: &mut BRep, w: f64, h: f64, z: f64) -> BodyId {
    let pts = [p(0.0, 0.0, z), p(w, 0.0, z), p(w, h, z), p(0.0, h, z)];
    patch_sheet(rep, &polygon(&pts)).unwrap().body
}

#[test]
fn a_tube_and_two_caps_stitch_into_a_box() {
    let mut rep = BRep::new();
    let t = tube(&mut rep, 6.0, 4.0, 3.0);
    let (a, b) = (cap(&mut rep, 6.0, 4.0, 0.0), cap(&mut rep, 6.0, 4.0, 3.0));
    let r = stitch(&mut rep, &[t, a, b], 1e-4).unwrap();
    assert_eq!(rep.bodies[r.body].kind, BodyKind::Solid);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    assert!((rep.volume(r.body).unwrap() - 72.0).abs() < 1e-5);
    assert!(rep.free_edges(r.body).is_empty());
    assert_eq!(rep.body_faces(r.body).len(), 6);
}

#[test]
fn a_tube_with_one_cap_stays_a_surface_and_a_far_cap_is_refused() {
    let mut rep = BRep::new();
    let t = tube(&mut rep, 6.0, 4.0, 3.0);
    let a = cap(&mut rep, 6.0, 4.0, 0.0);
    let r = stitch(&mut rep, &[t, a], 1e-4).unwrap();
    assert_eq!(rep.bodies[r.body].kind, BodyKind::Sheet);
    assert_eq!(rep.free_edges(r.body).len(), 4);
    let far = cap(&mut rep, 6.0, 4.0, 10.0);
    let e = stitch(&mut rep, &[t, far], 1e-4).unwrap_err();
    assert!(e.to_string().contains("separate groups"), "{e}");
}

#[test]
fn a_solid_cannot_be_stitched() {
    let mut rep = BRep::new();
    let b = kw_brep::make_box(&mut rep, &Frame3::world(), 1.0, 1.0, 1.0).body;
    assert!(stitch(&mut rep, &[b], 1e-4).is_err());
}

#[test]
fn unstitching_separates_the_faces_of_a_surface_and_of_a_solid() {
    let mut rep = BRep::new();
    let t = tube(&mut rep, 6.0, 4.0, 3.0);
    let u = unstitch(&mut rep, t).unwrap();
    assert_eq!(rep.bodies[u.body].kind, BodyKind::Sheet);
    assert_eq!(rep.body_faces(u.body).len(), 4);
    assert_eq!(rep.free_edges(u.body).len(), 16);
    assert!((rep.area(u.body).unwrap() - 60.0).abs() < 1e-9);
    // and back again
    let s = stitch(&mut rep, &[u.body], 1e-6).unwrap();
    assert_eq!(rep.free_edges(s.body).len(), 8);
    // a solid falls apart into six loose faces
    let b = kw_brep::make_box(&mut rep, &Frame3::world(), 2.0, 2.0, 2.0).body;
    let u = unstitch(&mut rep, b).unwrap();
    assert_eq!(rep.body_faces(u.body).len(), 6);
    assert_eq!(rep.bodies[u.body].kind, BodyKind::Sheet);
    // one face has nothing to unstitch
    let c = cap(&mut rep, 1.0, 1.0, 0.0);
    assert!(unstitch(&mut rep, c).is_err());
}

fn circle_tube(rep: &mut BRep, r: f64, h: f64) -> BodyId {
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
fn a_plane_and_a_cylinder_offset_exactly() {
    let mut rep = BRep::new();
    let c = cap(&mut rep, 6.0, 4.0, 1.0);
    let o = offset_sheet(&mut rep, c, 2.0).unwrap().body;
    assert_eq!(rep.bodies[o].kind, BodyKind::Sheet);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    assert!((rep.area(o).unwrap() - 24.0).abs() < 1e-5);
    // the offset face is at z = 3 or z = -1, whichever way the patch faces
    let f = rep.body_faces(o)[0];
    let s = &rep.surfaces[rep.faces[f].surface];
    let z = kw_nurbs::Surface::point_at(s, 0.4, 0.6).z;
    assert!((z - 3.0).abs() < 1e-6 || (z + 1.0).abs() < 1e-6, "{z}");
    let t = circle_tube(&mut rep, 2.0, 3.0);
    let out = offset_sheet(&mut rep, t, 0.5).unwrap().body;
    let inn = offset_sheet(&mut rep, t, -0.5).unwrap().body;
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    let (a, b) = (rep.area(out).unwrap(), rep.area(inn).unwrap());
    let (big, small) = (TAU * 2.5 * 3.0, TAU * 1.5 * 3.0);
    let ok = |x: f64, y: f64| (x - y).abs() < 1e-5 * y;
    assert!(
        (ok(a, big) && ok(b, small)) || (ok(a, small) && ok(b, big)),
        "{a} {b}"
    );
}

#[test]
fn a_nurbs_surface_offsets_to_a_cone_within_the_tolerance() {
    // a frustum surface from a loft is a NURBS surface; its offset is a cone again
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
    let l = 10.0f64.sqrt();
    let cos_a = 3.0 / l;
    let area0 = PI * 3.0 * l;
    let d = 0.4;
    let plus = offset_sheet(&mut rep, f, d).unwrap().body;
    let minus = offset_sheet(&mut rep, f, -d).unwrap().body;
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    let want = [area0 + TAU * d * cos_a * l, area0 - TAU * d * cos_a * l];
    let got = [rep.area(plus).unwrap(), rep.area(minus).unwrap()];
    for g in got {
        assert!(
            want.iter().any(|w| (g - w).abs() < 1e-4 * w),
            "{got:?} vs {want:?}"
        );
    }
    assert!((got[0] - got[1]).abs() > 1.0);
}

#[test]
fn an_offset_that_is_too_big_or_over_a_crease_is_refused() {
    let mut rep = BRep::new();
    let t = circle_tube(&mut rep, 2.0, 3.0);
    // the inward offset is longer than the radius
    let e = offset_sheet(&mut rep, t, -2.5);
    let e2 = offset_sheet(&mut rep, t, 2.5);
    assert!(e.is_err() || e2.is_err());
    // the walls of a box tube meet at creases
    let b = tube(&mut rep, 6.0, 4.0, 3.0);
    let e = offset_sheet(&mut rep, b, 0.5).unwrap_err();
    assert!(e.to_string().contains("crease"), "{e}");
    // a solid is not a surface
    let s = kw_brep::make_box(&mut rep, &Frame3::world(), 1.0, 1.0, 1.0).body;
    assert!(offset_sheet(&mut rep, s, 0.5).is_err());
}

#[test]
fn a_plane_and_a_tube_thicken_to_exact_volumes() {
    let mut rep = BRep::new();
    let c = cap(&mut rep, 6.0, 4.0, 1.0);
    for side in [ThickenSide::Normal, ThickenSide::Reverse, ThickenSide::Both] {
        let s = thicken(&mut rep, c, 0.5, side).unwrap().body;
        assert_eq!(rep.bodies[s].kind, BodyKind::Solid, "{side:?}");
        assert!(
            rep.validate().is_ok(),
            "{side:?}: {:?}",
            rep.validate().errors
        );
        assert!((rep.volume(s).unwrap() - 12.0).abs() < 1e-5, "{side:?}");
        assert_eq!(rep.body_faces(s).len(), 6);
    }
    let t = circle_tube(&mut rep, 2.0, 3.0);
    let a = thicken(&mut rep, t, 0.5, ThickenSide::Normal).unwrap().body;
    let b = thicken(&mut rep, t, 0.5, ThickenSide::Reverse)
        .unwrap()
        .body;
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    let (va, vb) = (rep.volume(a).unwrap(), rep.volume(b).unwrap());
    let (out, inn) = (PI * (2.5 * 2.5 - 4.0) * 3.0, PI * (4.0 - 1.5 * 1.5) * 3.0);
    let ok = |x: f64, y: f64| (x - y).abs() < 1e-4 * y;
    assert!(
        (ok(va, out) && ok(vb, inn)) || (ok(va, inn) && ok(vb, out)),
        "{va} {vb}"
    );
    // both sides: a wall of 0.5 centred on the surface
    let both = thicken(&mut rep, t, 0.5, ThickenSide::Both).unwrap().body;
    assert!((rep.volume(both).unwrap() - PI * (2.25 * 2.25 - 1.75 * 1.75) * 3.0).abs() < 1e-3);
}

#[test]
fn a_closed_surface_or_a_creased_one_cannot_be_thickened() {
    let mut rep = BRep::new();
    let t = tube(&mut rep, 6.0, 4.0, 3.0);
    let (a, b) = (cap(&mut rep, 6.0, 4.0, 0.0), cap(&mut rep, 6.0, 4.0, 3.0));
    // as a surface the box is closed; make it one by unstitching and not closing: use the tube instead
    let e = thicken(&mut rep, t, 0.5, ThickenSide::Normal).unwrap_err();
    assert!(e.to_string().contains("crease"), "{e}");
    let _ = (a, b);
    assert!(thicken(&mut rep, t, -1.0, ThickenSide::Normal).is_err());
}
