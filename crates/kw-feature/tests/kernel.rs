//! Sketch → extrude → fillet → shell through the document and engine, with persistent names that
//! survive parameter edits (the Phase 3 scenarios, on the Phase 4 engine).

use kw_expr::Param;
use kw_feature::*;

fn mm(v: f64) -> Param {
    Param::length(v)
}

fn volume(ev: &Evaluation, id: FeatureId) -> f64 {
    let o = ev
        .output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    o.brep().unwrap().volume(o.body().unwrap()).unwrap()
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

struct Fixture {
    doc: Document,
    #[allow(dead_code)]
    sketch: FeatureId,
    ext: FeatureId,
}

fn box_doc(w: f64, h: f64, d: f64) -> Fixture {
    let mut doc = Document::new();
    let sketch = doc
        .add_rect_sketch(
            "Sketch1",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(w),
            mm(h),
        )
        .unwrap();
    let ext = doc
        .add_extrude("Extrude1", sketch, mm(d), BodyOp::New)
        .unwrap();
    Fixture { doc, sketch, ext }
}

#[test]
fn sketch_extrude_fillet_shell_with_stable_names() {
    let Fixture { mut doc, ext, .. } = box_doc(4.0, 3.0, 2.0);
    let mut engine = Engine::with_threads(2);
    let ev = engine.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    assert!(rel(volume(&ev, ext), 24.0) < 1e-9);
    let names1: Vec<TopoName> = ev
        .output(ext)
        .unwrap()
        .names
        .by_name
        .keys()
        .cloned()
        .collect();
    assert_eq!(names1.len(), 6);
    // faces swept from sketch entities carry the entity label
    let sides = names1
        .iter()
        .filter(|n| matches!(n.origin, NameOrigin::SketchEdge { .. }))
        .count();
    assert_eq!(sides, 4);

    // fillet the edge shared by two side faces, selected by persistent references
    let sides: Vec<TopoName> = names1
        .iter()
        .filter(|n| matches!(n.origin, NameOrigin::SketchEdge { .. }))
        .cloned()
        .collect();
    let (a, b) = (sides[0].clone(), sides[1].clone());
    let ra = ev.reference(ext, &a).unwrap();
    let rb = ev.reference(ext, &b).unwrap();
    // pick two adjacent sides: try pairs until one shares an edge
    let mut fil = None;
    'outer: for i in 0..sides.len() {
        for j in i + 1..sides.len() {
            let (ra, rb) = (
                ev.reference(ext, &sides[i]).unwrap(),
                ev.reference(ext, &sides[j]).unwrap(),
            );
            let mut d2 = doc.clone();
            let f = d2.add_fillet("Fillet1", vec![[ra, rb]], mm(0.5)).unwrap();
            let e2 = Engine::with_threads(1).evaluate(&d2);
            if e2.output(f).is_some() {
                doc = d2;
                fil = Some(f);
                break 'outer;
            }
        }
    }
    let _ = (ra, rb);
    let fil = fil.expect("a fillet between two adjacent sides");
    let ev = engine.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    assert!(volume(&ev, fil) < 24.0);
    let fnames = &ev.output(fil).unwrap().names.by_name;
    assert_eq!(
        fnames.len(),
        7,
        "six original faces keep their names, plus the fillet face"
    );
    for n in &names1 {
        assert!(fnames.contains_key(n), "{n:?} was lost by the fillet");
    }
    assert!(fnames.keys().any(
        |n| matches!(&n.origin, NameOrigin::Generated { role, .. } if role.starts_with("fillet"))
    ));

    // shell with the top cap open
    let cap = ev.find_name(ext, "end-cap").unwrap();
    let rcap = ev.reference(fil, &cap).unwrap();
    let shl = doc
        .add(
            "Shell1",
            FeatureSpec::Shell(ShellSpec {
                thickness: mm(0.2),
                open: vec![rcap],
                base: None,
            }),
        )
        .unwrap();
    let ev = engine.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    assert!(volume(&ev, shl) < volume(&ev, fil));
    let o = ev.output(shl).unwrap();
    assert!(o.brep().unwrap().validate().is_ok());

    // a dimension change renames nothing and everything downstream still resolves
    let mut spec = doc.get(ext).unwrap().spec.clone();
    if let FeatureSpec::Extrude(e) = &mut spec {
        e.depth = mm(3.5);
    }
    doc.replace_spec(ext, spec).unwrap();
    let ev2 = engine.evaluate(&doc);
    assert!(ev2.is_clean(), "{:?}", ev2.failures());
    let names2: Vec<TopoName> = ev2
        .output(ext)
        .unwrap()
        .names
        .by_name
        .keys()
        .cloned()
        .collect();
    assert_eq!(names1, names2);
    assert!(rel(volume(&ev2, ext), 42.0) < 1e-9);
    assert!(volume(&ev2, shl) < volume(&ev2, fil));
}

#[test]
fn expressions_drive_dimensions_and_unresolvable_references_are_errors() {
    let mut doc = Document::new();
    let w = doc.add_parameter("width", mm(4.0)).unwrap();
    let sk = doc
        .add_rect_sketch(
            "S",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            Param::expr("width").unwrap(),
            Param::expr("width / 2").unwrap(),
        )
        .unwrap();
    let ext = doc
        .add_extrude("E", sk, Param::expr("width * 0.5").unwrap(), BodyOp::New)
        .unwrap();
    let mut engine = Engine::with_threads(1);
    let ev = engine.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    assert!(rel(volume(&ev, ext), 4.0 * 2.0 * 2.0) < 1e-9);
    // change the parameter: all three features follow
    doc.replace_spec(
        w,
        FeatureSpec::Parameter {
            name: "width".into(),
            value: mm(6.0),
        },
    )
    .unwrap();
    let ev = engine.evaluate(&doc);
    assert!(
        rel(volume(&ev, ext), 6.0 * 3.0 * 3.0) < 1e-9,
        "{}",
        volume(&ev, ext)
    );
    // a fillet that refers to a face that does not exist fails cleanly, and downstream is suspended
    let bogus = Reference {
        name: TopoName::primitive(ext, "nope", 0),
        fingerprint: GeomFingerprint {
            kind: naming::SurfKind::Sphere,
            dir: None,
            center: [100.0, 100.0, 100.0],
            extent: 1.0,
            radius: 1.0,
            adjacency: 0,
            area: 1.0,
        },
        scale: 10.0,
    };
    let fil = doc
        .add_fillet("F", vec![[bogus.clone(), bogus]], mm(0.1))
        .unwrap();
    let ev = engine.evaluate(&doc);
    assert!(
        matches!(ev.error(fil), Some(EvalError::Unresolved(_))),
        "{:?}",
        ev.state(fil)
    );
    let after = doc.add_parameter("later", mm(1.0)).unwrap();
    let ev = engine.evaluate(&doc);
    assert!(ev.output(after).is_some());
    assert!(matches!(
        ev.state(fil),
        Some(FeatureState::Failed {
            last_good: None,
            ..
        })
    ));
}

#[test]
fn sketch_with_a_hole_and_open_loops() {
    use kw_math::Frame3;
    use kw_sketch::Sketch;
    let mut sk = Sketch::new();
    let p: Vec<_> = [(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)]
        .iter()
        .map(|c| sk.add_point(c.0, c.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
    let c = sk.add_point(3.0, 2.0);
    sk.add_circle(c, 0.8);
    let profile = sketch_profile(&sk, Frame3::world()).unwrap();
    assert_eq!(profile.holes.len(), 1);
    let mut open = Sketch::new();
    open.add_line_xy(0.0, 0.0, 1.0, 0.0);
    open.add_line_xy(1.0, 0.0, 1.0, 1.0);
    assert!(sketch_profile(&open, Frame3::world()).is_err());
}

#[test]
fn boolean_cut_keeps_names_of_the_target() {
    let Fixture { mut doc, ext, .. } = box_doc(4.0, 4.0, 2.0);
    let hole = doc
        .add_circle_sketch("S2", PlaneSpec::Xy(mm(-1.0)), mm(2.0), mm(2.0), mm(1.0))
        .unwrap();
    let cut = doc.add_extrude("Cut", hole, mm(4.0), BodyOp::Cut).unwrap();
    let mut engine = Engine::with_threads(2);
    let ev = engine.evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    let v = volume(&ev, cut);
    assert!(
        rel(v, 32.0 - std::f64::consts::PI * 0.25 * 2.0) < 1e-9,
        "{v}"
    );
    let before = &ev.output(ext).unwrap().names.by_name;
    let after = &ev.output(cut).unwrap().names.by_name;
    for n in before.keys() {
        assert!(after.contains_key(n), "{n:?} vanished");
    }
    assert_eq!(after.len(), 7);
    assert!(ev.output(cut).unwrap().brep().unwrap().validate().is_ok());
}

#[test]
fn sketches_far_from_the_origin_and_tools_that_miss() {
    // a plate and a hole at x = 200: positions are applied exactly, whatever their size
    let mut doc = Document::new();
    let s = doc
        .add_rect_sketch(
            "S",
            PlaneSpec::Xy(mm(0.0)),
            mm(200.0),
            mm(0.0),
            mm(30.0),
            mm(60.0),
        )
        .unwrap();
    let base = doc.add_extrude("Plate", s, mm(6.0), BodyOp::New).unwrap();
    let c = doc
        .add_circle_sketch("C", PlaneSpec::Xy(mm(6.0)), mm(207.0), mm(7.0), mm(2.0))
        .unwrap();
    let cut = |doc: &mut Document, name: &str, sk| {
        doc.add(
            name,
            FeatureSpec::Extrude(ExtrudeSpec {
                sketch: sk,
                depth: mm(7.0),
                symmetric: false,
                flip: true,
                op: BodyOp::Cut,
                base: None,
                extent: Default::default(),
            }),
        )
        .unwrap()
    };
    let hole = cut(&mut doc, "Hole", c);
    // and one that misses the plate entirely
    let far = doc
        .add_circle_sketch("Far", PlaneSpec::Xy(mm(6.0)), mm(5.0), mm(5.0), mm(2.0))
        .unwrap();
    let miss = cut(&mut doc, "Miss", far);
    let ev = Engine::with_threads(1).evaluate(&doc);
    assert!(
        rel(
            volume(&ev, base) - volume(&ev, hole),
            std::f64::consts::PI * 6.0
        ) < 1e-9
    );
    assert!(
        matches!(ev.error(miss), Some(EvalError::Kernel(m)) if m.contains("does not touch")),
        "{:?}",
        ev.state(miss)
    );
}
