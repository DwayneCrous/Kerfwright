//! Incremental evaluation: caching, early cutoff, failure handling, timeline edits and undo.

use kw_expr::Param;
use kw_feature::*;

fn mm(v: f64) -> Param {
    Param::length(v)
}

/// Parameter `w`, a box 10 x w x 5, then a boss and a counterbored hole.
fn model() -> (Document, Vec<FeatureId>) {
    let mut doc = Document::new();
    let w = doc.add_parameter("w", mm(8.0)).unwrap();
    let s1 = doc
        .add_rect_sketch(
            "Base sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(10.0),
            Param::expr("w").unwrap(),
        )
        .unwrap();
    let base = doc.add_extrude("Base", s1, mm(5.0), BodyOp::New).unwrap();
    let s2 = doc
        .add_rect_sketch(
            "Boss sketch",
            PlaneSpec::Xy(mm(5.0)),
            mm(1.0),
            mm(1.0),
            mm(2.0),
            mm(2.0),
        )
        .unwrap();
    let boss = doc.add_extrude("Boss", s2, mm(3.0), BodyOp::Join).unwrap();
    let hs = doc
        .add_circle_sketch(
            "Hole sketch",
            PlaneSpec::Xy(mm(5.0)),
            mm(7.0),
            mm(4.0),
            mm(2.0),
        )
        .unwrap();
    let hole = doc.add_extrude("Hole", hs, mm(10.0), BodyOp::Cut).unwrap();
    // flip the hole direction so it cuts downward: an extrude along the plane normal from z=5 goes up,
    // so rebuild it from below instead
    let _ = hole;
    (doc, vec![w, s1, base, s2, boss, hs, hole])
}

#[test]
fn no_op_edit_reexecutes_nothing() {
    let (mut doc, ids) = model();
    let mut e = Engine::with_threads(2);
    let ev = e.evaluate(&doc);
    assert!(ev.stats.executed.len() >= 7, "{:?}", ev.stats);
    // same value, written differently
    doc.replace_spec(
        ids[0],
        FeatureSpec::Parameter {
            name: "w".into(),
            value: Param::expr("4 mm * 2").unwrap(),
        },
    )
    .unwrap();
    let ev2 = e.evaluate(&doc);
    assert_eq!(
        ev2.stats.executed,
        Vec::<FeatureId>::new(),
        "{:?}",
        ev2.stats
    );
    assert_eq!(ev.document_hash(), ev2.document_hash());
    // renaming does not even count as a change
    doc.rename(ids[2], "Renamed").unwrap();
    assert!(e.evaluate(&doc).stats.executed.is_empty());
}

#[test]
fn edit_reexecutes_only_what_depends_on_it() {
    let (mut doc, ids) = model();
    let mut e = Engine::with_threads(2);
    e.evaluate(&doc);
    // change the boss depth: boss and hole (downstream of it) re-run, base and sketches do not
    let mut spec = doc.get(ids[4]).unwrap().spec.clone();
    if let FeatureSpec::Extrude(x) = &mut spec {
        x.depth = mm(2.0);
    }
    doc.replace_spec(ids[4], spec).unwrap();
    let ev = e.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    let mut ran = ev.stats.executed.clone();
    ran.sort();
    assert_eq!(ran, vec![ids[4], ids[6]], "{:?}", ev.stats);
    // changing the parameter re-runs the dependents of `w`
    doc.replace_spec(
        ids[0],
        FeatureSpec::Parameter {
            name: "w".into(),
            value: mm(9.0),
        },
    )
    .unwrap();
    let ev = e.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    assert!(ev.stats.executed.contains(&ids[0]));
    assert!(ev.stats.executed.contains(&ids[1]));
    assert!(
        !ev.stats.executed.contains(&ids[3]),
        "an unrelated sketch re-ran"
    );
}

#[test]
fn undo_returns_to_cached_results() {
    let (doc, ids) = model();
    let mut hist = History::new(doc);
    let mut e = Engine::with_threads(1);
    let h0 = e.evaluate(&hist.current()).document_hash();
    hist.edit(|d| {
        d.replace_spec(
            ids[0],
            FeatureSpec::Parameter {
                name: "w".into(),
                value: mm(9.0),
            },
        )
    })
    .unwrap();
    let h1 = e.evaluate(&hist.current()).document_hash();
    assert_ne!(h0, h1);
    assert!(hist.undo());
    let ev = e.evaluate(&hist.current());
    assert_eq!(ev.document_hash(), h0);
    assert!(
        ev.stats.executed.is_empty(),
        "undo recomputed {:?}",
        ev.stats.executed
    );
    assert!(hist.redo());
    let ev = e.evaluate(&hist.current());
    assert_eq!(ev.document_hash(), h1);
    assert!(ev.stats.executed.is_empty());
}

#[test]
fn failure_suspends_dependents_and_keeps_last_good() {
    let (mut doc, ids) = model();
    let mut e = Engine::with_threads(2);
    e.evaluate(&doc);
    // a zero-depth extrusion fails
    let mut spec = doc.get(ids[2]).unwrap().spec.clone();
    if let FeatureSpec::Extrude(x) = &mut spec {
        x.depth = mm(0.0);
    }
    doc.replace_spec(ids[2], spec).unwrap();
    let ev = e.evaluate(&doc);
    assert!(matches!(
        ev.state(ids[2]),
        Some(FeatureState::Failed {
            last_good: Some(_),
            ..
        })
    ));
    assert!(matches!(
        ev.state(ids[4]),
        Some(FeatureState::Suspended { cause, .. }) if *cause == ids[2]
    ));
    assert!(
        ev.output(ids[3]).is_some(),
        "an unrelated sketch still evaluates"
    );
}

#[test]
fn rollback_and_suppress() {
    let (mut doc, ids) = model();
    let mut e = Engine::with_threads(1);
    doc.rollback_to(ids[2]).unwrap();
    let ev = e.evaluate(&doc);
    assert_eq!(ev.order.len(), 3);
    assert_eq!(ev.final_solid().unwrap().0, &ids[2]);
    doc.clear_rollback();
    // suppressing the boss lets the hole pass the base body through
    doc.set_suppressed(ids[4], true).unwrap();
    let ev = e.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    assert!(ev.state(ids[4]).is_none());
    assert!(ev.output(ids[6]).is_some());
    // suppressing a sketch that a feature needs is reported, not hidden
    doc.set_suppressed(ids[4], false).unwrap();
    doc.set_suppressed(ids[3], true).unwrap();
    let ev = e.evaluate(&doc);
    assert!(matches!(ev.error(ids[4]), Some(EvalError::Dependency(_))));
}

#[test]
fn timeline_reorder_validation() {
    let (mut doc, ids) = model();
    // moving the base extrude before its sketch is refused with an explanation
    let err = doc.move_feature(ids[2], 0).unwrap_err();
    assert!(
        matches!(
            err,
            TimelineError::ForwardReference { .. } | TimelineError::Cycle { .. }
        ),
        "{err}"
    );
    assert!(err.to_string().contains("Base"));
    // moving an independent sketch is fine
    doc.move_feature(ids[3], 1).unwrap();
    assert!(Engine::with_threads(1).evaluate(&doc).is_clean());
    // a parameter that depends on itself is a cycle
    let p = doc.add_parameter("a", mm(1.0)).unwrap();
    let err = doc
        .replace_spec(
            p,
            FeatureSpec::Parameter {
                name: "a".into(),
                value: Param::expr("a + 1").unwrap(),
            },
        )
        .unwrap_err();
    assert!(matches!(err, TimelineError::Cycle { .. }), "{err}");
    // removing a feature something depends on is refused
    let err = doc.remove(ids[1]).unwrap_err();
    assert!(matches!(err, TimelineError::HasDependents { .. }));
}

#[test]
fn cancellation_is_reported() {
    let (doc, _) = model();
    let mut e = Engine::with_threads(1);
    e.cancel.cancel();
    let ev = e.evaluate(&doc);
    assert!(ev
        .order
        .iter()
        .any(|id| matches!(ev.error(*id), Some(EvalError::Cancelled))));
    e.cancel.reset();
    assert!(e.evaluate(&doc).is_clean());
}

#[test]
fn sheet_metal_feature_builds_unfolds_and_follows_parameters() {
    use std::f64::consts::FRAC_PI_2;
    let mut doc = Document::new();
    let t = doc.add_parameter("t", mm(2.0)).unwrap();
    let _ = t;
    let sheet = doc
        .add(
            "Bracket",
            FeatureSpec::SheetMetal(SheetMetalSpec {
                thickness: Param::expr("t").unwrap(),
                k_factor: Param::scalar(0.44),
                default_radius: Param::expr("t * 1.5").unwrap(),
                width: mm(30.0),
                segs: vec![
                    SheetSegSpec::Flat { length: mm(40.0) },
                    SheetSegSpec::Bend {
                        angle: Param::Lit(kw_expr::Quantity::angle(FRAC_PI_2)),
                        radius: None,
                    },
                    SheetSegSpec::Flat { length: mm(25.0) },
                ],
            }),
        )
        .unwrap();
    let mut e = Engine::with_threads(2);
    let ev = e.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    let out = ev.output(sheet).unwrap();
    let vol = out.brep().unwrap().volume(out.body().unwrap()).unwrap();
    let ba = FRAC_PI_2 * (3.0 + 0.44 * 2.0);
    let flat = ev.flat_pattern(&doc, sheet).unwrap();
    assert!((flat.length() - (65.0 + ba)).abs() < 1e-9);
    assert!(
        (vol - 30.0 * 2.0 * (65.0 + FRAC_PI_2 * (3.0 + 1.0))).abs() < 1e-6,
        "{vol}"
    );
    assert!(out
        .names
        .by_name
        .keys()
        .any(|n| matches!(&n.origin, NameOrigin::Primitive { role } if role == "right-0")));
    // the thickness parameter drives both the solid and the flat pattern
    doc.replace_spec(
        t,
        FeatureSpec::Parameter {
            name: "t".into(),
            value: mm(3.0),
        },
    )
    .unwrap();
    let ev2 = e.evaluate(&doc);
    assert!(ev2.is_clean());
    let flat2 = ev2.flat_pattern(&doc, sheet).unwrap();
    assert!((flat2.length() - (65.0 + FRAC_PI_2 * (4.5 + 0.44 * 3.0))).abs() < 1e-9);
}

#[test]
fn subd_feature_finishes_a_cage_and_scales_with_a_parameter() {
    let mut doc = Document::new();
    let s = doc.add_parameter("grow", Param::scalar(1.0)).unwrap();
    let cube = doc
        .add(
            "Pillow",
            FeatureSpec::Subd(SubdSpec {
                cage: kw_subd::SubdMesh::cube(20.0),
                scale: Param::expr("grow").unwrap(),
                tolerance: mm(0.01),
            }),
        )
        .unwrap();
    let mut e = Engine::with_threads(2);
    let ev = e.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    let o = ev.output(cube).unwrap();
    let v1 = o.brep().unwrap().volume(o.body().unwrap()).unwrap();
    assert!(o.brep().unwrap().validate().is_ok());
    assert_eq!(
        o.names.by_name.len(),
        o.brep().unwrap().body_faces(o.body().unwrap()).len()
    );
    assert!(o
        .names
        .by_name
        .keys()
        .all(|n| matches!(&n.origin, NameOrigin::Primitive { role } if role.starts_with("cage-"))));
    doc.replace_spec(
        s,
        FeatureSpec::Parameter {
            name: "grow".into(),
            value: Param::scalar(2.0),
        },
    )
    .unwrap();
    let ev2 = e.evaluate(&doc);
    let o2 = ev2.output(cube).unwrap();
    let v2 = o2.brep().unwrap().volume(o2.body().unwrap()).unwrap();
    assert!((v2 / v1 - 8.0).abs() < 1e-3, "{v1} {v2}");
}

#[test]
fn removing_with_dependents_takes_the_whole_chain_and_keeps_unrelated_features() {
    use kw_feature::*;
    // sketch -> extrude -> hole, plus a separate parameter that nothing reads
    let mut doc = Document::new();
    let p = doc.add_parameter("unrelated", mm(3.0)).unwrap();
    let s = doc
        .add_rect_sketch(
            "S",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(20.0),
            mm(20.0),
        )
        .unwrap();
    let e = doc.add_extrude("E", s, mm(10.0), BodyOp::New).unwrap();
    let h = doc
        .add(
            "H",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(10.0)),
                points: vec![[mm(10.0), mm(10.0)]],
                kind: HoleKind::Simple { diameter: mm(4.0) },
                depth: mm(5.0),
                through_all: false,
                base: None,
            }),
        )
        .unwrap();
    assert_eq!(doc.all_dependents_of(s), vec![e, h]);
    assert_eq!(doc.all_dependents_of(e), vec![h]);
    // the strict removal still refuses
    assert!(doc.remove(s).is_err());
    // deleting the middle step takes the hole, not the sketch
    let mut d2 = doc.clone();
    let gone = d2.remove_with_dependents(e).unwrap();
    assert_eq!(gone.iter().map(|f| f.id).collect::<Vec<_>>(), vec![e, h]);
    assert_eq!(
        d2.features.iter().map(|f| f.id).collect::<Vec<_>>(),
        vec![p, s]
    );
    // deleting the sketch takes everything built on it
    let gone = doc.remove_with_dependents(s).unwrap();
    assert_eq!(gone.len(), 3);
    assert_eq!(
        doc.features.iter().map(|f| f.id).collect::<Vec<_>>(),
        vec![p]
    );
    assert!(Engine::with_threads(1).evaluate(&doc).is_clean());
}
