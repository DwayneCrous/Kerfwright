//! Matching a patch to a neighbouring surface across an edge: G1 then G2.
use kw_brep::{BRep, BodyId, BodyKind, EdgeId};
use kw_math::{Point3, Vector3};
use kw_model::*;
use kw_nurbs::{NurbsSurface, Surface, SurfaceGeom};
use std::sync::Arc;

const R: f64 = 5.0;
const THETA: f64 = 0.3;

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

/// A cylinder of radius 5 about the y axis, as the neighbour: normals point out, the patch leaves
/// towards increasing angle, and the surface curves away at 1 / R.
fn cylinder() -> NeighbourFn {
    Arc::new(|q: &Point3<f64>, _b: &Vector3<f64>| {
        let th = q.x.atan2(q.z);
        Some(NeighbourSample {
            normal: Vector3::new(th.sin(), 0.0, th.cos()),
            out: Vector3::new(th.cos(), 0.0, -th.sin()),
            curvature: -1.0 / R,
        })
    })
}

/// A patch with one edge on the cylinder (a generator of length 4 at the angle 0.3), the rest
/// leaving it at the wrong angle.
fn patch(rep: &mut BRep) -> BodyId {
    let (s, c) = THETA.sin_cos();
    let a = p(R * s, 0.0, R * c);
    let b = p(R * s, 4.0, R * c);
    let off = Vector3::new(2.0, 0.0, -1.5);
    let pts = [a, b, b + off, a + off];
    let edges: Vec<PatchEdge> = (0..4)
        .map(|i| PatchEdge::new(ProfileEdge::line(pts[i], pts[(i + 1) % 4])))
        .collect();
    patch_sheet(rep, &edges).unwrap().body
}

fn matched_edge(rep: &BRep, b: BodyId) -> EdgeId {
    let (s, c) = THETA.sin_cos();
    rep.free_edges(b)
        .into_iter()
        .find(|e| {
            let ed = &rep.edges[*e];
            [ed.start, ed.end].iter().all(|v| {
                let q = rep.vertices[*v].point;
                (q.x - R * s).abs() < 1e-9 && (q.z - R * c).abs() < 1e-9
            })
        })
        .expect("the edge on the cylinder")
}

fn nurbs(rep: &BRep, b: BodyId) -> NurbsSurface {
    let f = rep.body_faces(b)[0];
    match &rep.surfaces[rep.faces[f].surface] {
        SurfaceGeom::Nurbs(s) => s.clone(),
        o => panic!("{o:?}"),
    }
}

/// The angle between the normals of the surface and of the cylinder along the middle of the edge.
fn worst_angle(s: &NurbsSurface) -> f64 {
    let ((_, _), (_, _)) = s.domain();
    let cyl = cylinder();
    (0..9)
        .map(|k| {
            let u = 0.25 + 0.5 * k as f64 / 8.0;
            let q = s.point_at(u, 0.0);
            let n = s.normal_at(u, 0.0).unwrap();
            let c = cyl(&q, &Vector3::y()).unwrap().normal;
            n.dot(&c).abs().clamp(0.0, 1.0).acos()
        })
        .fold(0.0, f64::max)
}

#[test]
fn matching_g1_turns_the_patch_to_the_tangent_plane_and_nothing_else() {
    let mut rep = BRep::new();
    let b = patch(&mut rep);
    let before = nurbs(&rep, b);
    assert!(
        worst_angle(&before) > 0.1,
        "the patch starts off at an angle: {}",
        worst_angle(&before)
    );
    let e = matched_edge(&rep, b);
    let (r, res) = match_boundary(&mut rep, b, e, &cylinder(), Continuity::G1).unwrap();
    assert_eq!(rep.bodies[r.body].kind, BodyKind::Sheet);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    let after = nurbs(&rep, r.body);
    assert!(worst_angle(&after) < 2e-3, "{} rad", worst_angle(&after));
    assert!(res.angle < 2e-3, "{res:?}");
    // the edge, the far edge and the sides are where they were
    for t in [0.0, 0.3, 0.7, 1.0] {
        assert!((after.point_at(t, 0.0) - before.point_at(t, 0.0)).norm() < 1e-9);
        assert!((after.point_at(t, 1.0) - before.point_at(t, 1.0)).norm() < 1e-9);
        assert!((after.point_at(0.0, t) - before.point_at(0.0, t)).norm() < 1e-9);
        assert!((after.point_at(1.0, t) - before.point_at(1.0, t)).norm() < 1e-9);
    }
    // the surface did change inside
    assert!((after.point_at(0.5, 0.1) - before.point_at(0.5, 0.1)).norm() > 1e-3);
}

#[test]
fn matching_g2_also_gives_the_neighbours_curvature_across_the_edge() {
    let mut rep = BRep::new();
    let b = patch(&mut rep);
    let e = matched_edge(&rep, b);
    let (g1, r1) = match_boundary(&mut rep, b, e, &cylinder(), Continuity::G1).unwrap();
    let e1 = matched_edge(&rep, g1.body);
    let (g2, r2) = match_boundary(&mut rep, g1.body, e1, &cylinder(), Continuity::G2).unwrap();
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    // after G1 the patch is flat across the edge, 1/R off the neighbour; after G2 it is within 5 % of it
    assert!(r1.angle < 2e-3 && r2.angle < 2e-3, "{r1:?} {r2:?}");
    assert!(
        r2.curvature < 0.05 / R,
        "G2 residual {} (neighbour 1/R = {})",
        r2.curvature,
        1.0 / R
    );
    let s = nurbs(&rep, g2.body);
    // measured on the surface itself: the curvature along the cross direction
    let sg = SurfaceGeom::Nurbs(s.clone());
    let cyl = cylinder();
    for k in 0..5 {
        let u = 0.3 + 0.4 * k as f64 / 4.0;
        let q = s.point_at(u, 0.0);
        let n = cyl(&q, &Vector3::y()).unwrap();
        let (_, sv) = Surface::derivatives(&sg, u, 0.0);
        let d = sv.normalize();
        let kappa = normal_curvature(
            &sg,
            (u, 0.0),
            &d,
            n.normal.dot(&s.normal_at(u, 0.0).unwrap()).signum(),
        );
        assert!((kappa - (-1.0 / R)).abs() < 0.05 / R, "u = {u}: {kappa}");
    }
}

#[test]
fn what_cannot_be_matched_is_refused() {
    let mut rep = BRep::new();
    let b = patch(&mut rep);
    let e = matched_edge(&rep, b);
    // a solid
    let s = kw_brep::make_box(&mut rep, &kw_math::Frame3::world(), 1.0, 1.0, 1.0).body;
    assert!(match_boundary(&mut rep, s, e, &cylinder(), Continuity::G1).is_err());
    // an edge that is not open
    let shared = rep
        .edges
        .keys()
        .find(|k| rep.radial_ring(*k).len() == 2)
        .unwrap();
    assert!(match_boundary(&mut rep, b, shared, &cylinder(), Continuity::G1).is_err());
    // a neighbour that cannot be measured
    let none: NeighbourFn = Arc::new(|_, _| None);
    assert!(match_boundary(&mut rep, b, e, &none, Continuity::G1).is_err());
}
