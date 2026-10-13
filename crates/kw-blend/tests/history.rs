#![allow(unused_imports, dead_code)]
//! `ShapeHistory` completeness for every modelling operation: every output face has an origin, every
//! reference points at an entity of the operation's inputs, and every input face either has a
//! descendant or is reported deleted.

use kw_blend::*;
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::collections::HashSet;
use std::f64::consts::TAU;

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn faces(rep: &BRep, b: BodyId) -> HashSet<FaceId> {
    rep.body_faces(b).into_iter().collect()
}

/// `inputs`: faces the history may refer to. `closed_over_inputs`: every input face must have a descendant or be deleted.
fn check(
    rep: &BRep,
    out: BodyId,
    h: &ShapeHistory,
    inputs: &HashSet<FaceId>,
    closed_over_inputs: bool,
    what: &str,
) {
    let out_faces = faces(rep, out);
    for f in &out_faces {
        assert!(
            h.origins.contains_key(&EntityRef::Face(*f)),
            "{what}: output face {f:?} has no origin"
        );
    }
    let mut alive: HashSet<FaceId> = HashSet::new();
    for (k, o) in &h.origins {
        if let EntityRef::Face(f) = k {
            assert!(
                out_faces.contains(f),
                "{what}: history entry for a face that is not in the output ({f:?})"
            );
        }
        let src = match o {
            Origin::Preserved(s) | Origin::Modified(s) => Some(*s),
            Origin::Generated { from, .. } => Some(*from),
            Origin::SplitFrom { parent, .. } => Some(*parent),
            _ => None,
        };
        if let Some(EntityRef::Face(sf)) = src {
            assert!(
                inputs.contains(&sf),
                "{what}: {k:?} refers to {sf:?}, which is not an input face"
            );
            if !matches!(o, Origin::Generated { .. }) {
                alive.insert(sf);
            }
        }
    }
    if closed_over_inputs {
        for f in inputs {
            assert!(
                alive.contains(f) || h.deleted_inputs.contains(&EntityRef::Face(*f)),
                "{what}: input face {f:?} neither survives nor is reported deleted"
            );
        }
    }
}

#[test]
fn constructive_operations_describe_all_their_faces() {
    let none = HashSet::new();
    let mut rep = BRep::new();
    let e = extrude(
        &mut rep,
        &Profile::rectangle(world(), 2.0, 1.0),
        Vector3::new(0.0, 0.0, 1.0),
        &ExtrudeOptions::default(),
    )
    .unwrap();
    check(&rep, e.body, &e.history, &none, false, "extrude");
    let f = Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    };
    let r = revolve(
        &mut rep,
        &Profile::polygon(f, &[(0.5, 0.0), (1.5, 0.0), (1.5, 1.0), (0.5, 1.0)]),
        Point3::origin(),
        Vector3::z(),
        TAU,
    )
    .unwrap();
    check(&rep, r.body, &r.history, &none, false, "revolve");
    let top = Frame3 {
        origin: Point3::new(0.0, 0.0, 3.0),
        ..world()
    };
    let l = loft(
        &mut rep,
        &[
            Profile::rectangle(world(), 2.0, 2.0),
            Profile::rectangle(top, 1.0, 1.0),
        ],
    )
    .unwrap();
    check(&rep, l.body, &l.history, &none, false, "loft");
    let path = kw_nurbs::NurbsCurve::new_polynomial(
        kw_nurbs::KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
        vec![Point3::origin(), Point3::new(0.0, 0.0, 2.0)],
    )
    .unwrap();
    let s = sweep(
        &mut rep,
        &Profile::disc(world(), 0.5),
        &kw_nurbs::CurveGeom::Nurbs(path),
        &SweepOptions::default(),
    )
    .unwrap();
    check(&rep, s.body, &s.history, &none, false, "sweep");
}

#[test]
fn boolean_history_is_complete() {
    let mut rep = BRep::new();
    let a = make_box(&mut rep, &world(), 2.0, 2.0, 2.0).body;
    let b = make_cylinder(
        &mut rep,
        &Frame3 {
            origin: Point3::new(0.3, 0.2, -2.0),
            ..world()
        },
        0.5,
        4.0,
    )
    .body;
    let inputs: HashSet<FaceId> = faces(&rep, a).union(&faces(&rep, b)).copied().collect();
    for op in [Op::Union, Op::Intersect, Op::Subtract] {
        let r = boolean(&mut rep, a, b, op, &BoolOptions::default()).unwrap();
        check(&rep, r.body, &r.history, &inputs, true, &format!("{op:?}"));
    }
}

#[test]
fn fillet_and_chamfer_history_points_at_the_original_body_and_edges() {
    for chamfer_op in [false, true] {
        let mut rep = BRep::new();
        let b = make_box(&mut rep, &world(), 4.0, 3.0, 2.0).body;
        let orig = faces(&rep, b);
        // two vertical edges
        let mut edges = Vec::new();
        for f in rep.body_faces(b) {
            for lp in &rep.faces[f].loops {
                for co in rep.loop_coedges(*lp) {
                    let e = rep.coedges[co].edge;
                    let ed = &rep.edges[e];
                    let c = ed.curve.and_then(|c| rep.curves.get(c)).unwrap();
                    if matches!(c, kw_nurbs::CurveGeom::Line(l) if l.dir.normalize().cross(&Vector3::z()).norm() < 1e-9)
                        && !edges.contains(&e)
                    {
                        edges.push(e);
                    }
                }
            }
        }
        let chosen = vec![edges[0], edges[2]];
        let r = if chamfer_op {
            chamfer(&mut rep, b, &chosen, 0.3, 0.3)
        } else {
            fillet(&mut rep, b, &chosen, 0.3)
        }
        .unwrap();
        // blend faces are Generated from a requested edge, everything else descends from the original faces
        let mut generated = 0;
        for f in rep.body_faces(r.body) {
            match r
                .history
                .origins
                .get(&EntityRef::Face(f))
                .unwrap_or_else(|| panic!("face without origin"))
            {
                Origin::Generated {
                    from: EntityRef::Edge(e),
                    ..
                } => {
                    assert!(
                        chosen.contains(e),
                        "blend face generated from an edge that was not requested"
                    );
                    generated += 1;
                }
                Origin::Generated { .. } => panic!("generated from a non-edge"),
                _ => {}
            }
        }
        assert_eq!(generated, 2, "one blend face per requested edge");
        // the Generated entries are excluded from the face check: use only original faces as valid sources
        let mut h = r.history.clone();
        h.origins
            .retain(|_, o| !matches!(o, Origin::Generated { .. }));
        let out_faces = faces(&rep, r.body);
        for (k, o) in &h.origins {
            let _ = k;
            if let Origin::Preserved(EntityRef::Face(s))
            | Origin::Modified(EntityRef::Face(s))
            | Origin::SplitFrom {
                parent: EntityRef::Face(s),
                ..
            } = o
            {
                assert!(orig.contains(s), "history refers to a face that is not on the original body (tool or intermediate face leaked)");
            }
        }
        let _ = out_faces;
        for f in &orig {
            let alive = h.origins.values().any(|o| matches!(o, Origin::Preserved(EntityRef::Face(s)) | Origin::Modified(EntityRef::Face(s)) | Origin::SplitFrom { parent: EntityRef::Face(s), .. } if s == f));
            assert!(
                alive || h.deleted_inputs.contains(&EntityRef::Face(*f)),
                "original face {f:?} neither survives nor is deleted"
            );
        }
    }
}

#[test]
fn offset_shell_and_draft_history() {
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
    .body;
    let inputs = faces(&rep, b);
    let o = offset_body(&mut rep, b, 0.2).unwrap();
    check(&rep, o.body, &o.history, &inputs, true, "offset");
    let top = rep.body_faces(b).into_iter().find(|f| matches!(&rep.surfaces[rep.faces[*f].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.origin.z > 1.9 && p.frame.z.z.abs() > 0.9)).unwrap();
    let sh = shell(&mut rep, b, 0.2, &[top]).unwrap();
    // shell: outer faces survive, inner walls descend from the original faces, the opening face is deleted
    let out_faces = faces(&rep, sh.body);
    for f in &out_faces {
        let o = sh
            .history
            .origins
            .get(&EntityRef::Face(*f))
            .unwrap_or_else(|| panic!("shell face without origin"));
        match o {
            Origin::Preserved(EntityRef::Face(s))
            | Origin::Modified(EntityRef::Face(s))
            | Origin::SplitFrom {
                parent: EntityRef::Face(s),
                ..
            } => {
                assert!(
                    inputs.contains(s),
                    "shell history refers to a non-input face {s:?} ({o:?})"
                )
            }
            Origin::New { .. } => {}
            other => panic!("unexpected shell origin {other:?}"),
        }
    }
    let sides: Vec<FaceId> = rep.body_faces(b).into_iter().filter(|f| matches!(&rep.surfaces[rep.faces[*f].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.z.z.abs() < 0.5)).collect();
    let d = draft_faces(
        &mut rep,
        b,
        &sides,
        Point3::new(0.0, 0.0, 0.0),
        Vector3::z(),
        0.05,
    )
    .unwrap();
    check(&rep, d.body, &d.history, &inputs, true, "draft");
}
