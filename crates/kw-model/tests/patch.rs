//! Patches and ruled surfaces: boundaries kept, planes stay planes, tangent sides are tangent.
use kw_brep::{BRep, BodyId, BodyKind};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::{CurveGeom, SurfaceGeom};
use std::f64::consts::{PI, TAU};
use std::sync::Arc;

fn p(x: f64, y: f64, z: f64) -> Point3<f64> {
    Point3::new(x, y, z)
}

fn lines(pts: &[Point3<f64>]) -> Vec<PatchEdge> {
    (0..pts.len())
        .map(|i| PatchEdge::new(ProfileEdge::line(pts[i], pts[(i + 1) % pts.len()])))
        .collect()
}

fn sheet_area(rep: &BRep, b: BodyId) -> f64 {
    assert_eq!(rep.bodies[b].kind, BodyKind::Sheet);
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    assert_eq!(rep.volume(b), None);
    rep.area(b).unwrap()
}

/// The surfaces of a body, in face order.
fn surfaces(rep: &BRep, b: BodyId) -> Vec<kw_nurbs::NurbsSurface> {
    rep.body_faces(b)
        .into_iter()
        .map(|f| match &rep.surfaces[rep.faces[f].surface] {
            SurfaceGeom::Nurbs(s) => s.clone(),
            other => panic!("{other:?}"),
        })
        .collect()
}

#[test]
fn a_flat_four_sided_hole_is_filled_by_a_plane() {
    let mut rep = BRep::new();
    let pts = [
        p(0.0, 0.0, 1.0),
        p(6.0, 0.0, 1.0),
        p(6.0, 4.0, 1.0),
        p(0.0, 4.0, 1.0),
    ];
    let r = patch_sheet(&mut rep, &lines(&pts)).unwrap();
    assert!((sheet_area(&rep, r.body) - 24.0).abs() < 1e-5);
    let s = &surfaces(&rep, r.body)[0];
    for (u, v) in [(0.2, 0.7), (0.5, 0.5), (0.93, 0.11)] {
        let q = s.point_at(u, v);
        assert!((q.z - 1.0).abs() < 1e-9, "{q:?}");
    }
    // it keeps its four boundary edges as free edges
    assert_eq!(rep.free_edges(r.body).len(), 4);
}

#[test]
fn a_skew_quadrilateral_gives_the_bilinear_surface_and_keeps_its_boundary() {
    let mut rep = BRep::new();
    let pts = [
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(1.0, 1.0, 1.0),
        p(0.0, 1.0, 0.0),
    ];
    let r = patch_sheet(&mut rep, &lines(&pts)).unwrap();
    let s = &surfaces(&rep, r.body)[0];
    // the hyperbolic paraboloid z = x y
    for (u, v) in [(0.2, 0.7), (0.5, 0.5), (0.93, 0.11), (0.0, 0.4), (1.0, 0.9)] {
        let q = s.point_at(u, v);
        assert!((q.z - q.x * q.y).abs() < 1e-6, "{q:?}");
    }
    // the boundary is the given line: points on it lie on the edge of the patch
    for t in [0.1, 0.35, 0.8] {
        let on_bottom = s.point_at(t, 0.0);
        assert!(on_bottom.y.abs() < 1e-9 && on_bottom.z.abs() < 1e-9);
    }
}

#[test]
fn two_lines_and_two_arcs_give_the_cylinder_between_them() {
    let mut rep = BRep::new();
    let (r, h) = (2.0, 3.0);
    let frame = |z: f64| Frame3 {
        origin: p(0.0, 0.0, z),
        ..Frame3::world()
    };
    let bottom = ProfileEdge::arc(&frame(0.0), r, 0.0, PI / 2.0);
    let top = ProfileEdge::arc(&frame(h), r, 0.0, PI / 2.0);
    let edges = vec![
        PatchEdge::new(bottom),
        PatchEdge::new(ProfileEdge::line(p(0.0, r, 0.0), p(0.0, r, h))),
        PatchEdge::new(ProfileEdge {
            reversed: true,
            ..top
        }),
        PatchEdge::new(ProfileEdge::line(p(r, 0.0, h), p(r, 0.0, 0.0))),
    ];
    let b = patch_sheet(&mut rep, &edges).unwrap().body;
    let s = &surfaces(&rep, b)[0];
    for (u, v) in [(0.1, 0.9), (0.5, 0.5), (0.77, 0.3)] {
        let q = s.point_at(u, v);
        assert!((q.x.hypot(q.y) - r).abs() < 1e-5, "{q:?}");
    }
    let want = r * PI / 2.0 * h;
    assert!((sheet_area(&rep, b) - want).abs() < 1e-4 * want);
}

#[test]
fn a_triangle_and_a_pentagon_split_about_their_centres() {
    for n in [3usize, 5, 6] {
        let mut rep = BRep::new();
        let pts: Vec<Point3<f64>> = (0..n)
            .map(|i| {
                let a = TAU * i as f64 / n as f64;
                p(3.0 * a.cos(), 3.0 * a.sin(), 0.5)
            })
            .collect();
        let b = patch_sheet(&mut rep, &lines(&pts)).unwrap().body;
        let area = 0.5 * n as f64 * 9.0 * (TAU / n as f64).sin();
        assert!((sheet_area(&rep, b) - area).abs() < 1e-5 * area, "n = {n}");
        assert_eq!(rep.body_faces(b).len(), n);
        // each side is cut at its middle; the spokes are shared by two faces, so only the outline is open
        assert_eq!(rep.free_edges(b).len(), 2 * n, "n = {n}");
        for s in surfaces(&rep, b) {
            let q = s.point_at(0.4, 0.6);
            assert!((q.z - 0.5).abs() < 1e-9);
        }
    }
}

#[test]
fn fewer_than_three_edges_or_a_gap_is_refused() {
    let mut rep = BRep::new();
    assert!(patch_sheet(&mut rep, &lines(&[p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)])).is_err());
    let mut e = lines(&[
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(1.0, 1.0, 0.0),
        p(0.0, 1.0, 0.0),
    ]);
    e[2] = PatchEdge::new(ProfileEdge::line(p(1.0, 1.5, 0.0), p(0.0, 1.0, 0.0)));
    assert!(patch_sheet(&mut rep, &e).is_err());
}

/// The plane z = 0, lying on the side of negative y of the edge: the patch leaves it towards +y.
fn flat_below() -> NormalFn {
    Arc::new(|_, _| Some((Vector3::z(), Vector3::y())))
}

#[test]
fn a_tangent_side_leaves_the_neighbouring_plane_flat_and_a_plain_one_does_not() {
    // the bottom edge lies in the plane z = 0; the far side is raised
    let pts = [
        p(0.0, 0.0, 0.0),
        p(4.0, 0.0, 0.0),
        p(4.0, 3.0, 2.0),
        p(0.0, 3.0, 2.0),
    ];
    let angle = |s: &kw_nurbs::NurbsSurface, u: f64| {
        let n = s.normal_at(u, 0.0).unwrap();
        n.dot(&Vector3::z()).abs().clamp(0.0, 1.0).acos()
    };
    let mut plain = BRep::new();
    let a = patch_sheet(&mut plain, &lines(&pts)).unwrap().body;
    let sa = surfaces(&plain, a).remove(0);
    let mut tangent = BRep::new();
    let mut e = lines(&pts);
    e[0].neighbour = Some(flat_below());
    let b = patch_sheet(&mut tangent, &e).unwrap().body;
    let sb = surfaces(&tangent, b).remove(0);
    for u in [0.25, 0.5, 0.75] {
        assert!(
            angle(&sa, u) > 0.3,
            "the plain patch is tilted: {}",
            angle(&sa, u)
        );
        assert!(
            angle(&sb, u) < 1e-3,
            "tangent at u = {u}: {} rad",
            angle(&sb, u)
        );
    }
    // the boundary does not move
    for u in [0.1, 0.6] {
        let q = sb.point_at(u, 0.0);
        assert!(q.y.abs() < 1e-8 && q.z.abs() < 1e-8);
    }
    assert!(tangent.validate().is_ok());
}

#[test]
fn tangency_needs_four_edges() {
    let mut rep = BRep::new();
    let mut e = lines(&[p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0), p(2.0, 3.0, 0.0)]);
    e[0].neighbour = Some(flat_below());
    assert!(patch_sheet(&mut rep, &e).is_err());
}

#[test]
fn a_ruled_surface_between_parallel_segments_and_between_circles() {
    let mut rep = BRep::new();
    let a = ProfileEdge::line(p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0));
    // given the other way round: the surface must not twist
    let b = ProfileEdge::line(p(4.0, 3.0, 0.0), p(0.0, 3.0, 0.0));
    let s = ruled_sheet(&mut rep, &a, &b).unwrap().body;
    assert!((sheet_area(&rep, s) - 12.0).abs() < 1e-9);

    let mut rep = BRep::new();
    let circle = |r: f64, z: f64| {
        ProfileEdge::new(
            CurveGeom::Circle(kw_nurbs::geom::Circle {
                frame: Frame3 {
                    origin: p(0.0, 0.0, z),
                    ..Frame3::world()
                },
                radius: r,
            }),
            (0.0, TAU),
        )
    };
    let s = ruled_sheet(&mut rep, &circle(1.0, 0.0), &circle(3.0, 2.0))
        .unwrap()
        .body;
    let want = PI * (1.0 + 3.0) * 8.0f64.sqrt();
    assert!((sheet_area(&rep, s) - want).abs() < 1e-6 * want);
}

#[test]
fn a_ruled_surface_accepts_curves_the_loft_cannot_take_directly() {
    // an ellipse is replaced by a cubic through samples of it
    let mut rep = BRep::new();
    let ell = ProfileEdge::new(
        CurveGeom::Ellipse(kw_nurbs::geom::Ellipse {
            frame: Frame3::world(),
            major: 3.0,
            minor: 2.0,
        }),
        (0.0, 1.5),
    );
    let top = ProfileEdge::line(p(0.0, 0.0, 4.0), p(1.0, 0.0, 4.0));
    let s = ruled_sheet(&mut rep, &ell, &top).unwrap().body;
    assert!(sheet_area(&rep, s) > 1.0);
}
