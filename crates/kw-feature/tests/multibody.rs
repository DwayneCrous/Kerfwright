//! Several bodies in one document: heads, targeting a body, move / copy / scale / split, appearance.

use kw_expr::Param;
use kw_feature::*;
use std::f64::consts::PI;

fn mm(v: f64) -> Param {
    Param::length(v)
}

fn eval(doc: &Document) -> Evaluation {
    Engine::with_threads(2).evaluate(doc)
}

fn vol(ev: &Evaluation, id: FeatureId) -> f64 {
    let o = ev
        .output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(o.brep().unwrap().validate().is_ok());
    o.brep().unwrap().volume(o.body().unwrap()).unwrap()
}

/// A w x h block with its corner at (x, y) and height d, as a new body.
fn block_at(doc: &mut Document, name: &str, x: f64, y: f64, w: f64, h: f64, d: f64) -> FeatureId {
    let s = doc
        .add_rect_sketch(
            &format!("{name} sketch"),
            PlaneSpec::Xy(mm(0.0)),
            mm(x),
            mm(y),
            mm(w),
            mm(h),
        )
        .unwrap();
    doc.add_extrude(name, s, mm(d), BodyOp::New).unwrap()
}

fn move_spec(base: Option<FeatureId>, d: [f64; 3], copy: bool) -> FeatureSpec {
    FeatureSpec::MoveBody(MoveBodySpec {
        base,
        delta: [mm(d[0]), mm(d[1]), mm(d[2])],
        axis: 2,
        angle: Param::angle_deg(0.0),
        copy,
    })
}

#[test]
fn new_bodies_stay_separate_and_a_feature_can_target_either() {
    let mut doc = Document::new();
    let a = block_at(&mut doc, "A", 0.0, 0.0, 30.0, 20.0, 10.0);
    assert_eq!(doc.heads(), vec![a]);
    let b = block_at(&mut doc, "B", 50.0, 0.0, 10.0, 10.0, 10.0);
    assert_eq!(
        doc.heads(),
        vec![a, b],
        "a New body no longer replaces the one before it"
    );
    let ev = eval(&doc);
    assert!((vol(&ev, a) - 6000.0).abs() < 1e-6 && (vol(&ev, b) - 1000.0).abs() < 1e-6);

    // a hole in the *first* body, although the second is the last feature: it names its base
    let top = {
        let o = ev.output(a).unwrap();
        let br = o.brep().unwrap();
        let f = br
            .faces
            .keys()
            .find(|f| {
                br.face_bbox(*f).is_some_and(|bb| {
                    (bb.min.z - 10.0).abs() < 1e-6 && (bb.max.z - 10.0).abs() < 1e-6
                })
            })
            .unwrap();
        o.names.reference_to(br, f).unwrap()
    };
    let hole = doc
        .add(
            "Hole in A",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Face(top),
                points: vec![[mm(15.0), mm(10.0)]],
                kind: HoleKind::Simple { diameter: mm(4.0) },
                depth: mm(10.0),
                base: Some(a),
                through_all: false,
            }),
        )
        .unwrap();
    assert_eq!(
        doc.heads(),
        vec![b, hole],
        "A was used up by the hole; B is untouched"
    );
    let ev = eval(&doc);
    assert!((vol(&ev, hole) - (6000.0 - PI * 4.0 * 10.0)).abs() < 1e-3);
    assert!((vol(&ev, b) - 1000.0).abs() < 1e-6);

    // Combine joins the two (disjoint) bodies into one head; keep_tool keeps the tool alive
    let c = doc
        .add(
            "Combine",
            FeatureSpec::Combine(CombineSpec {
                target: hole,
                tool: b,
                op: BodyOp::Join,
                keep_tool: false,
            }),
        )
        .unwrap();
    assert_eq!(doc.heads(), vec![c]);
    let mut kept = doc.clone();
    let spec = FeatureSpec::Combine(CombineSpec {
        target: hole,
        tool: b,
        op: BodyOp::Join,
        keep_tool: true,
    });
    kept.replace_spec(c, spec).unwrap();
    assert_eq!(kept.heads(), vec![b, c]);

    // rolling back hides the later bodies from the list; suppressing a body feature hands the body back
    let mut rb = doc.clone();
    rb.rollback_to(b).unwrap();
    assert_eq!(rb.heads(), vec![a, b]);
}

#[test]
fn move_copy_scale_and_split_give_the_expected_bodies() {
    let mut doc = Document::new();
    let a = block_at(&mut doc, "A", 0.0, 0.0, 30.0, 20.0, 5.0);
    // a moved copy: two bodies, same volume, the copy sits 100 mm to the right
    let cp = doc
        .add("Copy", move_spec(Some(a), [100.0, 0.0, 0.0], true))
        .unwrap();
    assert_eq!(doc.heads(), vec![a, cp]);
    let ev = eval(&doc);
    assert!((vol(&ev, cp) - 3000.0).abs() < 1e-6);
    let bb = {
        let o = ev.output(cp).unwrap();
        o.brep().unwrap().body_bbox(o.body().unwrap())
    };
    assert!(
        (bb.min.x - 100.0).abs() < 1e-6 && (bb.max.x - 130.0).abs() < 1e-6,
        "{bb:?}"
    );
    // a move (not a copy) uses its source up
    let mv = doc
        .add("Move", move_spec(Some(a), [0.0, 0.0, 7.0], false))
        .unwrap();
    assert_eq!(doc.heads(), vec![cp, mv]);
    let ev = eval(&doc);
    let bb = {
        let o = ev.output(mv).unwrap();
        o.brep().unwrap().body_bbox(o.body().unwrap())
    };
    assert!((bb.min.z - 7.0).abs() < 1e-6 && (bb.max.z - 12.0).abs() < 1e-6);
    // a rotation by 90 degrees about Z: the 30 x 20 footprint becomes 20 x 30
    let rot = doc
        .add(
            "Rotate",
            FeatureSpec::MoveBody(MoveBodySpec {
                base: Some(cp),
                delta: [mm(0.0), mm(0.0), mm(0.0)],
                axis: 2,
                angle: Param::angle_deg(90.0),
                copy: false,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let bb = {
        let o = ev.output(rot).unwrap();
        o.brep().unwrap().body_bbox(o.body().unwrap())
    };
    assert!(
        ((bb.max.x - bb.min.x) - 20.0).abs() < 1e-6 && ((bb.max.y - bb.min.y) - 30.0).abs() < 1e-6,
        "{bb:?}"
    );

    // scale by 1.5 about the origin: volume x 3.375
    let sc = doc
        .add(
            "Scale",
            FeatureSpec::ScaleBody(ScaleBodySpec {
                base: Some(mv),
                factor: Param::scalar(1.5),
                centre: [mm(0.0), mm(0.0), mm(0.0)],
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(
        (vol(&ev, sc) - 3000.0 * 3.375).abs() < 1e-4,
        "{}",
        vol(&ev, sc)
    );
    // a bad factor is refused with a message
    let bad = doc
        .add(
            "Bad scale",
            FeatureSpec::ScaleBody(ScaleBodySpec {
                base: Some(rot),
                factor: Param::scalar(-2.0),
                centre: [mm(0.0), mm(0.0), mm(0.0)],
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(bad).is_none() && !ev.failures().is_empty());
}

#[test]
fn splitting_a_body_by_a_plane_gives_two_halves_that_add_up() {
    let mut doc = Document::new();
    let a = block_at(&mut doc, "A", 0.0, 0.0, 30.0, 20.0, 5.0);
    let split = |keep| {
        FeatureSpec::SplitBody(SplitBodySpec {
            base: Some(a),
            plane: PlaneSpec::Yz(mm(10.0)), // the plane x = 10, normal +X
            keep,
        })
    };
    let hi = doc.add("Split +", split(SplitSide::Positive)).unwrap();
    let lo = doc.add("Split -", split(SplitSide::Negative)).unwrap();
    assert_eq!(
        doc.heads(),
        vec![hi, lo],
        "both halves are bodies, the source is used up"
    );
    let ev = eval(&doc);
    let (vh, vl) = (vol(&ev, hi), vol(&ev, lo));
    assert!(
        (vh - 2000.0).abs() < 1e-6 && (vl - 1000.0).abs() < 1e-6,
        "{vh} {vl}"
    );
    assert!((vh + vl - 3000.0).abs() < 1e-6);
    // the halves meet exactly at the plane
    let min_x = |id: FeatureId| {
        let o = ev.output(id).unwrap();
        o.brep().unwrap().body_bbox(o.body().unwrap())
    };
    assert!((min_x(hi).min.x - 10.0).abs() < 1e-6 && (min_x(lo).max.x - 10.0).abs() < 1e-6);
}

#[test]
fn bodies_carry_names_colours_and_materials_through_save_and_reopen() {
    let mut doc = Document::new();
    let a = block_at(&mut doc, "A", 0.0, 0.0, 10.0, 10.0, 10.0);
    let fil = {
        let ev = eval(&doc);
        let o = ev.output(a).unwrap();
        let br = o.brep().unwrap();
        let top = br
            .faces
            .keys()
            .find(|f| {
                br.face_bbox(*f).is_some_and(|bb| {
                    (bb.min.z - 10.0).abs() < 1e-6 && (bb.max.z - 10.0).abs() < 1e-6
                })
            })
            .unwrap();
        let side = br
            .faces
            .keys()
            .find(|f| {
                br.face_bbox(*f)
                    .is_some_and(|bb| (bb.min.x - bb.max.x).abs() < 1e-6 && bb.min.x.abs() < 1e-6)
            })
            .unwrap();
        (
            o.names.reference_to(br, top).unwrap(),
            o.names.reference_to(br, side).unwrap(),
        )
    };
    let f = doc
        .add(
            "Fillet",
            FeatureSpec::Fillet(FilletSpec {
                edges: vec![EdgeRef {
                    faces: [fil.0, fil.1],
                }],
                radius: mm(1.0),
                base: None,
            }),
        )
        .unwrap();
    // the body started by A is still "A" after the fillet
    assert_eq!(doc.body_root(f), a);
    doc.set_appearance(
        a,
        Appearance {
            name: Some("Bracket".into()),
            colour: Some([200, 60, 40]),
            material: Some("Aluminium".into()),
        },
    );
    let bytes = kw_feature::kwf::KwfFile::new(doc.clone())
        .to_bytes()
        .unwrap();
    let back = kw_feature::kwf::KwfFile::from_bytes(&bytes)
        .unwrap()
        .document;
    assert_eq!(back, doc);
    let ap = back.appearance_of(back.body_root(f)).unwrap();
    assert_eq!(ap.colour, Some([200, 60, 40]));
    assert_eq!(ap.name.as_deref(), Some("Bracket"));
    // clearing it removes the entry
    let mut cleared = back.clone();
    cleared.set_appearance(a, Appearance::default());
    assert!(cleared.appearance_of(a).is_none());
}

#[test]
fn splitting_a_combined_body_in_its_overlap_region() {
    let mut doc = Document::new();
    let a = block_at(&mut doc, "A", 0.0, 0.0, 30.0, 20.0, 5.0);
    let cp = doc
        .add("Copy", move_spec(Some(a), [20.0, 0.0, 0.0], true))
        .unwrap();
    let c = doc
        .add(
            "Combine",
            FeatureSpec::Combine(CombineSpec {
                target: a,
                tool: cp,
                op: BodyOp::Join,
                keep_tool: false,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!((vol(&ev, c) - 5000.0).abs() < 1e-3, "{}", vol(&ev, c));
    for keep in [SplitSide::Positive, SplitSide::Negative] {
        let mut d = doc.clone();
        let s = d
            .add(
                "Split",
                FeatureSpec::SplitBody(SplitBodySpec {
                    base: Some(c),
                    plane: PlaneSpec::Yz(mm(25.0)),
                    keep,
                }),
            )
            .unwrap();
        let ev = eval(&d);
        assert!(ev.output(s).is_some(), "{keep:?}: {:?}", ev.failures());
        assert!(
            (vol(&ev, s) - 2500.0).abs() < 1e-3,
            "{keep:?}: {}",
            vol(&ev, s)
        );
    }
}
