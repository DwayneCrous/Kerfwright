#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
use kw_blend::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::{FRAC_PI_2, PI};

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

fn mesh_ok(rep: &BRep, body: BodyId, vol: f64, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let t = kw_tess::tessellate_body(
        rep,
        body,
        &kw_tess::TessOptions {
            chord_tol: 0.004,
            angle_tol: 0.2,
        },
    )
    .unwrap_or_else(|e| panic!("{what}: tess {e:?}"));
    assert!(
        (t.mesh.signed_volume() - vol).abs() < 0.02 * vol,
        "{what}: mesh volume {} vs {vol}",
        t.mesh.signed_volume()
    );
}

/// Edges of the body parallel to `dir` (within tolerance), as (edge, midpoint).
fn edges_along(rep: &BRep, body: BodyId, dir: Vector3<f64>) -> Vec<(EdgeId, Point3<f64>)> {
    use kw_nurbs::Curve3;
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !seen.insert(e) {
                    continue;
                }
                let ed = &rep.edges[e];
                if let Some(c) = ed.curve.and_then(|c| rep.curves.get(c)) {
                    if let kw_nurbs::CurveGeom::Line(_) = c {
                        let tm = 0.5 * (ed.param_range.0 + ed.param_range.1);
                        if c.derivative(tm).normalize().cross(&dir).norm() < 1e-9 {
                            out.push((e, c.point_at(tm)));
                        }
                    }
                }
            }
        }
    }
    out
}

#[test]
fn fillet_and_chamfer_of_box_edges() {
    // 4 x 3 x 2 box, corner at origin
    let mk = |rep: &mut BRep| {
        let f = Frame3 {
            origin: Point3::new(2.0, 1.5, 1.0),
            ..world()
        };
        make_box(rep, &f, 4.0, 3.0, 2.0).body
    };
    let r = 0.5;
    // one vertical edge
    let mut rep = BRep::new();
    let b = mk(&mut rep);
    let edges = edges_along(&rep, b, Vector3::z());
    let (e, _) = edges.iter().find(|(_, p)| p.x > 3.9 && p.y > 2.9).unwrap();
    let res = fillet(&mut rep, b, &[*e], r).unwrap();
    let removed = r * r * (1.0 - PI / 4.0) * 2.0;
    assert!(
        rel(rep.volume(res.body).unwrap(), 24.0 - removed) < 1e-9,
        "{}",
        rep.volume(res.body).unwrap()
    );
    mesh_ok(&rep, res.body, 24.0 - removed, "vertical fillet");
    // chamfer
    let mut rep = BRep::new();
    let b = mk(&mut rep);
    let edges = edges_along(&rep, b, Vector3::z());
    let (e, _) = edges.iter().find(|(_, p)| p.x > 3.9 && p.y > 2.9).unwrap();
    let res = chamfer(&mut rep, b, &[*e], 0.6, 0.4).unwrap();
    let removed = 0.5 * 0.6 * 0.4 * 2.0;
    assert!(
        rel(rep.volume(res.body).unwrap(), 24.0 - removed) < 1e-9,
        "{}",
        rep.volume(res.body).unwrap()
    );
    mesh_ok(&rep, res.body, 24.0 - removed, "chamfer");
    // all four vertical edges: four independent fillets
    let mut rep = BRep::new();
    let b = mk(&mut rep);
    let all: Vec<EdgeId> = edges_along(&rep, b, Vector3::z())
        .into_iter()
        .map(|e| e.0)
        .collect();
    assert_eq!(all.len(), 4);
    let res = fillet(&mut rep, b, &all, r).unwrap();
    assert!(
        rel(
            rep.volume(res.body).unwrap(),
            24.0 - 4.0 * removed_f(r, 2.0)
        ) < 1e-9
    );
    mesh_ok(
        &rep,
        res.body,
        24.0 - 4.0 * removed_f(r, 2.0),
        "four fillets",
    );
}

fn removed_f(r: f64, len: f64) -> f64 {
    r * r * (1.0 - PI / 4.0) * len
}

#[test]
fn rim_fillets_of_a_cylinder() {
    let mut rep = BRep::new();
    let b = make_cylinder(&mut rep, &world(), 1.5, 2.0).body;
    // circular rim edges: top and bottom
    let mut rims = Vec::new();
    for f in rep.body_faces(b) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                let ed = &rep.edges[e];
                if let Some(kw_nurbs::CurveGeom::Circle(_)) =
                    ed.curve.and_then(|c| rep.curves.get(c))
                {
                    if !rims.contains(&e) {
                        rims.push(e);
                    }
                }
            }
        }
    }
    assert_eq!(rims.len(), 2);
    let r = 0.4;
    let res = fillet(&mut rep, b, &rims[..1], r).unwrap();
    // volume removed by rounding one rim: 2π ∫ ... = (π r²)(1 − π/4)·(2 (R − r) + 4r/3 ... ) use Pappus on the spandrel
    // spandrel area A = r²(1 − π/4); its centroid is at distance R − r(1 − ... ) — compute numerically via the formula
    // centroid offset from the corner along each axis: x̄ = r (10 − 3π) / (3 (4 − π)) from the corner
    let a = r * r * (1.0 - PI / 4.0);
    let xbar = r * (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));
    let removed = a * circumference(1.5 - xbar);
    let want = PI * 2.25 * 2.0 - removed;
    assert!(
        rel(rep.volume(res.body).unwrap(), want) < 1e-9,
        "{} vs {want}",
        rep.volume(res.body).unwrap()
    );
    mesh_ok(&rep, res.body, want, "rim fillet");
    // both rims
    let mut rep = BRep::new();
    let b = make_cylinder(&mut rep, &world(), 1.5, 2.0).body;
    let mut rims = Vec::new();
    for f in rep.body_faces(b) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if let Some(kw_nurbs::CurveGeom::Circle(_)) =
                    rep.edges[e].curve.and_then(|c| rep.curves.get(c))
                {
                    if !rims.contains(&e) {
                        rims.push(e);
                    }
                }
            }
        }
    }
    let res = fillet(&mut rep, b, &rims, r).unwrap();
    let want = PI * 2.25 * 2.0 - 2.0 * removed;
    assert!(rel(rep.volume(res.body).unwrap(), want) < 1e-9);
    mesh_ok(&rep, res.body, want, "both rims");
    let _ = FRAC_PI_2;
}

fn circumference(r: f64) -> f64 {
    2.0 * PI * r
}

#[test]
fn concave_edge_gets_material_added() {
    // L-shaped prism: fillet the inside (concave) vertical edge
    let mut rep = BRep::new();
    let prof = Profile::polygon(
        world(),
        &[
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 1.0),
            (1.0, 1.0),
            (1.0, 3.0),
            (0.0, 3.0),
        ],
    );
    let b = extrude(
        &mut rep,
        &prof,
        Vector3::new(0.0, 0.0, 1.5),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let v0 = rep.volume(b).unwrap();
    let edges = edges_along(&rep, b, Vector3::z());
    let (e, _) = edges
        .iter()
        .find(|(_, p)| (p.x - 1.0).abs() < 1e-9 && (p.y - 1.0).abs() < 1e-9)
        .unwrap();
    let r = 0.3;
    let res = fillet(&mut rep, b, &[*e], r).unwrap();
    let added = removed_f(r, 1.5);
    assert!(
        rel(rep.volume(res.body).unwrap(), v0 + added) < 1e-9,
        "{} vs {}",
        rep.volume(res.body).unwrap(),
        v0 + added
    );
    mesh_ok(&rep, res.body, v0 + added, "concave");
}

#[test]
fn unsupported_edges_report_the_edge() {
    let mut rep = BRep::new();
    let b = make_sphere(&mut rep, &world(), 1.0).body;
    // sphere seam edge
    let e = rep
        .faces
        .values()
        .next()
        .map(|f| rep.loop_coedges(f.loops[0])[1])
        .map(|c| rep.coedges[c].edge)
        .unwrap();
    let err = fillet(&mut rep, b, &[e], 0.1).unwrap_err();
    assert_eq!(err.edge, e);
}

/// Variable-radius fillet on a straight edge: removed volume = ∫ (1 − π/4) r(s)² ds for a right angle.
#[test]
fn variable_radius_fillet_removes_the_integrated_area() {
    for (r0, r1) in [(0.2, 0.8), (0.9, 0.3), (0.5, 0.5)] {
        let mut rep = BRep::new();
        let b = make_box(&mut rep, &world(), 4.0, 3.0, 2.0).body;
        let (e, _) = edges_along(&rep, b, Vector3::z())[0];
        let r = fillet_variable(&mut rep, b, e, r0, r1).unwrap();
        let want = 24.0 - (1.0 - PI / 4.0) * 2.0 * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
        let got = rep.volume(r.body).unwrap();
        assert!(rel(got, want) < 1e-9, "r {r0}->{r1}: {got} vs {want}");
        mesh_ok(&rep, r.body, want, "variable fillet");
        // the blend face is generated from the requested edge
        assert!(r
            .history
            .origins
            .values()
            .any(|o| matches!(o, Origin::Generated { from: EntityRef::Edge(x), .. } if *x == e)));
    }
}

#[test]
fn variable_radius_is_refused_on_curved_edges_naming_the_edge() {
    let mut rep = BRep::new();
    let c = make_cylinder(&mut rep, &world(), 1.0, 2.0).body;
    let rim = rep
        .body_faces(c)
        .into_iter()
        .flat_map(|f| rep.faces[f].loops.clone())
        .flat_map(|lp| rep.loop_coedges(lp))
        .map(|co| rep.coedges[co].edge)
        .find(|e| {
            rep.edges[*e].curve.is_some()
                && rep.radial_ring(*e).len() == 2
                && !rep.edges[*e].is_degenerate()
        })
        .unwrap();
    let err = fillet_variable(&mut rep, c, rim, 0.1, 0.2).unwrap_err();
    assert!(
        matches!(
            err.kind,
            BlendErrorKind::Unsupported(_) | BlendErrorKind::Size(_)
        ),
        "{err}"
    );
}

#[test]
fn variable_radius_fillet_on_a_concave_edge_adds_the_integrated_area() {
    let mut rep = BRep::new();
    let l_shape = Profile::polygon(
        world(),
        &[
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 2.0),
            (2.0, 2.0),
            (2.0, 4.0),
            (0.0, 4.0),
        ],
    );
    let b = extrude(
        &mut rep,
        &l_shape,
        Vector3::new(0.0, 0.0, 2.0),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let (e, _) = edges_along(&rep, b, Vector3::z())
        .into_iter()
        .find(|(_, m)| (m.x - 2.0).abs() < 1e-9 && (m.y - 2.0).abs() < 1e-9)
        .unwrap();
    let (r0, r1) = (0.3, 0.9);
    let r = fillet_variable(&mut rep, b, e, r0, r1).unwrap();
    let want = 24.0 + (1.0 - PI / 4.0) * 2.0 * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
    let got = rep.volume(r.body).unwrap();
    assert!(rel(got, want) < 1e-9, "{got} vs {want}");
    mesh_ok(&rep, r.body, want, "concave variable fillet");
}
