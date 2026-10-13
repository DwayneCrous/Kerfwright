//! Continuity of edges: boxes, fillets, seams, matched patches, gaps.
use kw_analysis::{body_continuity, ContClass, Tolerances};
use kw_brep::builders::make_plane_sheet;
use kw_brep::{make_box, BRep, BodyId};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;
use std::sync::Arc;

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

fn classes(rep: &BRep, b: BodyId, size: f64) -> HashMap<ContClass, usize> {
    let mut m = HashMap::new();
    for c in body_continuity(rep, b, &Tolerances::for_size(size)) {
        *m.entry(c.class).or_insert(0) += 1;
    }
    m
}

#[test]
fn the_edges_of_a_box_are_creases() {
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &Frame3::world(), 4.0, 3.0, 2.0).body;
    let all = body_continuity(&rep, b, &Tolerances::for_size(4.0));
    assert_eq!(all.len(), 12);
    for c in &all {
        assert_eq!(c.class, ContClass::G0);
        assert!((c.angle - FRAC_PI_2).abs() < 1e-6, "{}", c.angle);
        assert!(c.gap < 1e-9);
        assert!(c.curvature.is_none());
    }
}

#[test]
fn a_fillet_is_tangent_but_not_curvature_continuous_with_the_planes_beside_it() {
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &Frame3::world(), 10.0, 8.0, 6.0).body;
    // the edge along z at the corner x = +5, y = +4
    let e = rep
        .edges
        .keys()
        .find(|e| {
            let ed = &rep.edges[*e];
            let (a, c) = (rep.vertices[ed.start].point, rep.vertices[ed.end].point);
            (a.x - 5.0).abs() < 1e-9
                && (a.y - 4.0).abs() < 1e-9
                && (c.x - 5.0).abs() < 1e-9
                && (c.y - 4.0).abs() < 1e-9
        })
        .unwrap();
    let r = kw_blend::fillet(&mut rep, b, &[e], 1.5).unwrap();
    let m = classes(&rep, r.body, 10.0);
    // the blend meets the two planes tangentially (G1, curvature jumps from 0 to 1/r) and the two end
    // faces at creases; the other eight edges of the box are creases
    assert_eq!(m.get(&ContClass::G1), Some(&2), "{m:?}");
    assert_eq!(m.get(&ContClass::G0), Some(&13), "{m:?}");
    assert!(!m.contains_key(&ContClass::G2) && !m.contains_key(&ContClass::Broken));
    for c in body_continuity(&rep, r.body, &Tolerances::for_size(10.0)) {
        if c.class == ContClass::G1 {
            // curvature across the edge jumps by about 1 / r
            assert!(
                (c.curvature.unwrap() - 1.0 / 1.5).abs() < 0.02,
                "{:?}",
                c.curvature
            );
            assert!(c.angle < 1e-6);
        }
    }
}

fn circle_tube(rep: &mut BRep) -> BodyId {
    let ProfileLoop { edges } = ProfileLoop::circle(&Frame3::world(), 2.0);
    let chain = Chain::new(Frame3::world(), edges, true);
    extrude_sheet(
        rep,
        &[chain],
        Vector3::new(0.0, 0.0, 3.0),
        &Default::default(),
    )
    .unwrap()
    .body
}

#[test]
fn the_seam_of_a_cylinder_is_perfectly_smooth_and_its_rims_are_open() {
    let mut rep = BRep::new();
    let t = circle_tube(&mut rep);
    let m = classes(&rep, t, 3.0);
    assert_eq!(m.get(&ContClass::G2), Some(&1), "{m:?}");
    assert_eq!(m.get(&ContClass::Open), Some(&2), "{m:?}");
}

/// The wall of a 6 x 5 box tube lying in the plane y = 0 as a neighbour: normal -y, out of the wall
/// upwards (+z), flat.
fn wall() -> NeighbourFn {
    Arc::new(|_p: &Point3<f64>, _b: &Vector3<f64>| {
        Some(NeighbourSample {
            normal: -Vector3::y(),
            out: Vector3::z(),
            curvature: 0.0,
        })
    })
}

fn box_tube_and_cap(rep: &mut BRep) -> (BodyId, BodyId) {
    let pts = [
        p(0.0, 0.0, 0.0),
        p(6.0, 0.0, 0.0),
        p(6.0, 5.0, 0.0),
        p(0.0, 5.0, 0.0),
    ];
    let chain = Chain::new(Frame3::world(), ProfileLoop::polygon(&pts).edges, true);
    let tube = extrude_sheet(
        rep,
        &[chain],
        Vector3::new(0.0, 0.0, 4.0),
        &Default::default(),
    )
    .unwrap()
    .body;
    let top = [
        p(0.0, 0.0, 4.0),
        p(6.0, 0.0, 4.0),
        p(6.0, 5.0, 4.0),
        p(0.0, 5.0, 4.0),
    ];
    let edges: Vec<PatchEdge> = (0..4)
        .map(|i| PatchEdge::new(ProfileEdge::line(top[i], top[(i + 1) % 4])))
        .collect();
    (tube, patch_sheet(rep, &edges).unwrap().body)
}

#[test]
fn a_cap_goes_from_a_crease_to_tangent_to_curvature_continuous_as_it_is_matched() {
    let size = 6.0;
    let tol = Tolerances::for_size(size);
    // the edge of interest is the one the cap shares with the wall y = 0
    let on_wall = |rep: &BRep, b: BodyId| {
        body_continuity(rep, b, &tol)
            .into_iter()
            .find(|c| {
                c.points
                    .iter()
                    .all(|q| q.y.abs() < 1e-6 && (q.z - 4.0).abs() < 1e-6)
            })
            .expect("the edge on the wall")
    };
    let mut seen = Vec::new();
    for level in [None, Some(Continuity::G1), Some(Continuity::G2)] {
        let mut rep = BRep::new();
        let (tube, mut cap) = box_tube_and_cap(&mut rep);
        if let Some(level) = level {
            let edge = rep
                .free_edges(cap)
                .into_iter()
                .find(|e| {
                    let ed = &rep.edges[*e];
                    [ed.start, ed.end]
                        .iter()
                        .all(|v| rep.vertices[*v].point.y.abs() < 1e-9)
                })
                .unwrap();
            cap = match_boundary(&mut rep, cap, edge, &wall(), level)
                .unwrap()
                .0
                .body;
        }
        let s = stitch(&mut rep, &[tube, cap], 1e-5).unwrap().body;
        let c = on_wall(&rep, s);
        seen.push((c.class, c.angle, c.curvature));
    }
    assert_eq!(seen[0].0, ContClass::G0, "{seen:?}");
    assert!((seen[0].1 - FRAC_PI_2).abs() < 1e-3);
    assert_eq!(seen[1].0, ContClass::G1, "{seen:?}");
    assert!(seen[1].1 < 0.5f64.to_radians());
    assert_eq!(seen[2].0, ContClass::G2, "{seen:?}");
}

#[test]
fn faces_that_do_not_meet_are_reported_as_a_gap() {
    // two sheets stitched along an edge, then one of them moved off by 0.05
    let mut rep = BRep::new();
    let a = make_plane_sheet(&mut rep, &Frame3::world(), 6.0, 4.0).body;
    let frame = Frame3 {
        origin: p(6.0, 0.0, 0.0),
        ..Frame3::world()
    };
    let b = make_plane_sheet(&mut rep, &frame, 6.0, 4.0).body;
    let s = stitch(&mut rep, &[a, b], 1e-5).unwrap().body;
    let m = classes(&rep, s, 12.0);
    assert_eq!(m.get(&ContClass::G2), Some(&1), "{m:?}");
    let second = rep.body_faces(s)[1];
    let id = rep.faces[second].surface;
    let moved = kw_nurbs::fit::transform_surface(
        &rep.surfaces[id],
        &kw_math::Transform3::translation(Vector3::new(0.0, 0.0, 0.05)),
    );
    rep.surfaces[id] = moved;
    let m = classes(&rep, s, 12.0);
    assert_eq!(m.get(&ContClass::Broken), Some(&1), "{m:?}");
}
