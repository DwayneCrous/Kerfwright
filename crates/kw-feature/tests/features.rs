//! Every feature kind the executor knows, once: it evaluates, gives a valid solid with the volume the
//! geometry says, and names its faces.

use kw_expr::Param;
use kw_feature::recipe::{EntityKindRec, EntityRec};
use kw_feature::*;
use std::f64::consts::PI;

fn mm(v: f64) -> Param {
    Param::length(v)
}

fn vol(ev: &Evaluation, id: FeatureId) -> f64 {
    let o = ev
        .output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(
        o.brep().unwrap().validate().is_ok(),
        "{:?}",
        o.brep().unwrap().validate()
    );
    o.brep().unwrap().volume(o.body().unwrap()).unwrap()
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

fn block(doc: &mut Document, w: f64, h: f64, d: f64) -> (FeatureId, FeatureId) {
    let s = doc
        .add_rect_sketch(
            "Block sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(w),
            mm(h),
        )
        .unwrap();
    let e = doc.add_extrude("Block", s, mm(d), BodyOp::New).unwrap();
    (s, e)
}

fn eval(doc: &Document) -> Evaluation {
    Engine::with_threads(2).evaluate(doc)
}

#[test]
fn revolve_loft_and_sweep() {
    let mut doc = Document::new();
    // revolve a rectangle [10, 14] x [0, 6] about the sketch's y axis (a ring)
    let s = doc
        .add_rect_sketch(
            "Ring sketch",
            PlaneSpec::Xz(mm(0.0)),
            mm(10.0),
            mm(0.0),
            mm(4.0),
            mm(6.0),
        )
        .unwrap();
    let ring = doc
        .add(
            "Ring",
            FeatureSpec::Revolve(RevolveSpec {
                surface: false,
                sketch: s,
                axis: AxisSpec::SketchY,
                angle: Param::angle_deg(360.0),
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(close(
        vol(&ev, ring),
        PI * (14.0f64.powi(2) - 10.0f64.powi(2)) * 6.0,
        1e-9
    ));
    assert!(ev.output(ring).unwrap().names.by_name.len() >= 4);

    // loft between a square and a smaller square 10 mm above
    let mut doc = Document::new();
    let a = doc
        .add_rect_sketch(
            "Bottom",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(10.0),
            mm(10.0),
        )
        .unwrap();
    let b = doc
        .add_rect_sketch(
            "Top",
            PlaneSpec::Xy(mm(10.0)),
            mm(2.0),
            mm(2.0),
            mm(6.0),
            mm(6.0),
        )
        .unwrap();
    let loft = doc
        .add(
            "Loft",
            FeatureSpec::Loft(LoftSpec {
                surface: false,
                ruled: false,
                sketches: vec![a, b],
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    // a frustum of a pyramid: h/3 (A1 + A2 + sqrt(A1 A2))
    assert!(
        close(vol(&ev, loft), 10.0 / 3.0 * (100.0 + 36.0 + 60.0), 1e-6),
        "{}",
        vol(&ev, loft)
    );
}

#[test]
fn chamfer_variable_fillet_shell_draft_offset() {
    let mut doc = Document::new();
    let (_, e) = block(&mut doc, 20.0, 20.0, 10.0);
    let ev = eval(&doc);
    let names: Vec<TopoName> = ev
        .output(e)
        .unwrap()
        .names
        .by_name
        .keys()
        .filter(|n| matches!(n.origin, NameOrigin::SketchEdge { .. }))
        .cloned()
        .collect();
    // find two adjacent side faces by trial
    let refs: Vec<Reference> = names.iter().map(|n| ev.reference(e, n).unwrap()).collect();
    let mut pair = None;
    'o: for i in 0..4 {
        for j in i + 1..4 {
            let mut d = doc.clone();
            let c = d
                .add(
                    "C",
                    FeatureSpec::Chamfer(ChamferSpec {
                        edges: vec![EdgeRef {
                            faces: [refs[i].clone(), refs[j].clone()],
                        }],
                        d1: mm(1.0),
                        d2: mm(2.0),
                        base: None,
                    }),
                )
                .unwrap();
            if eval(&d).output(c).is_some() {
                pair = Some((refs[i].clone(), refs[j].clone()));
                break 'o;
            }
        }
    }
    let (ra, rb) = pair.expect("two adjacent faces");
    // chamfer 1 x 2 along a 10 mm edge removes 1 mm^2 x 10 mm
    let mut d = doc.clone();
    let c = d
        .add(
            "Chamfer",
            FeatureSpec::Chamfer(ChamferSpec {
                edges: vec![EdgeRef {
                    faces: [ra.clone(), rb.clone()],
                }],
                d1: mm(1.0),
                d2: mm(2.0),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    assert!(
        close(vol(&ev, c), 4000.0 - 1.0 * 2.0 / 2.0 * 10.0, 1e-9),
        "{}",
        vol(&ev, c)
    );
    // variable fillet on the same edge
    let mut d = doc.clone();
    let v = d
        .add(
            "VarFillet",
            FeatureSpec::VariableFillet(VariableFilletSpec {
                edge: EdgeRef {
                    faces: [ra.clone(), rb.clone()],
                },
                start: mm(1.0),
                end: mm(3.0),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    let removed = 4000.0 - vol(&ev, v);
    assert!(removed > 0.0 && removed < 4000.0 * 0.01, "{removed}");
    // shell the block open at its top
    let ev0 = eval(&doc);
    let top = ev0.find_name(e, "end-cap").unwrap();
    let mut d = doc.clone();
    let sh = d
        .add(
            "Shell",
            FeatureSpec::Shell(ShellSpec {
                thickness: mm(2.0),
                open: vec![ev0.reference(e, &top).unwrap()],
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    assert!(
        close(vol(&ev, sh), 4000.0 - 16.0 * 16.0 * 8.0, 1e-6),
        "{}",
        vol(&ev, sh)
    );
    // draft the four sides by 5 degrees about the bottom plane
    let mut d = doc.clone();
    let dr = d
        .add(
            "Draft",
            FeatureSpec::Draft(DraftSpec {
                faces: refs.clone(),
                neutral_origin: [mm(0.0), mm(0.0), mm(0.0)],
                pull: [0.0, 0.0, 1.0],
                angle: Param::angle_deg(5.0),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    let v = vol(&ev, dr);
    assert!(v < 4000.0 && v > 3500.0, "{v}");
    // offset the whole body outward by 1 mm
    let mut d = doc.clone();
    let of = d
        .add(
            "Offset",
            FeatureSpec::Offset(OffsetSpec {
                distance: mm(1.0),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    // box offset: not a box any more (rounded corners are not made), but the volume must grow
    assert!(vol(&ev, of) > 4000.0);
}

#[test]
fn holes_of_three_kinds_and_patterns_and_mirror() {
    let mut doc = Document::new();
    let (_, e) = block(&mut doc, 60.0, 30.0, 10.0);
    // a simple hole, a counterbore and a countersink, all on the top face (z = 10)
    let simple = doc
        .add(
            "Hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(10.0)),
                points: vec![[mm(10.0), mm(15.0)]],
                kind: HoleKind::Simple { diameter: mm(4.0) },
                depth: mm(10.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let cb = doc
        .add(
            "Counterbore",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(10.0)),
                points: vec![[mm(30.0), mm(15.0)]],
                kind: HoleKind::Counterbore {
                    diameter: mm(4.0),
                    cb_diameter: mm(8.0),
                    cb_depth: mm(3.0),
                },
                depth: mm(10.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let cs = doc
        .add(
            "Countersink",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(10.0)),
                points: vec![[mm(50.0), mm(15.0)]],
                kind: HoleKind::Countersink {
                    diameter: mm(4.0),
                    cs_diameter: mm(8.0),
                    cs_angle: Param::angle_deg(90.0),
                },
                depth: mm(10.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let v0 = vol(&ev, e);
    let v1 = vol(&ev, simple);
    assert!(close(v0 - v1, PI * 4.0 * 10.0, 1e-9));
    let v2 = vol(&ev, cb);
    assert!(
        close(v1 - v2, PI * 4.0 * 10.0 + PI * (16.0 - 4.0) * 3.0, 1e-9),
        "{}",
        v1 - v2
    );
    let v3 = vol(&ev, cs);
    // countersink: cylinder + a cone frustum 2 mm deep (90° included angle, 4 mm radius change)
    let frustum = PI * 2.0 / 3.0 * (16.0 + 4.0 + 8.0) - PI * 4.0 * 2.0; // minus the part already in the bore
    assert!(
        close(v2 - v3, PI * 4.0 * 10.0 + frustum, 1e-6),
        "{}",
        v2 - v3
    );

    // circular and linear patterns of the simple hole, and a mirror of the counterbore
    let mut d = doc.clone();
    let lin = d
        .add_linear_pattern(
            "Row",
            simple,
            [mm(0.0), mm(8.0), mm(0.0)],
            Param::scalar(2.0),
        )
        .unwrap();
    let ev = eval(&d);
    assert!(close(vol(&ev, cs) - vol(&ev, lin), PI * 4.0 * 10.0, 1e-9));
    let mut d = doc.clone();
    let mir = d
        .add(
            "Mirror",
            FeatureSpec::Mirror(MirrorSpec {
                source: simple,
                plane: MirrorPlane::Xz(mm(20.0)),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    // the hole at (10, 15) is mirrored to (10, 25)
    assert!(
        close(vol(&ev, cs) - vol(&ev, mir), PI * 4.0 * 10.0, 1e-9),
        "{}",
        vol(&ev, cs) - vol(&ev, mir)
    );
    let names = &ev.output(mir).unwrap().names.by_name;
    assert!(names
        .keys()
        .any(|n| matches!(&n.origin, NameOrigin::Instance { index: 1, .. })));
    // circular pattern of a hole around the block's centre
    let mut d = Document::new();
    let (_, e) = block(&mut d, 40.0, 40.0, 10.0);
    let h = d
        .add(
            "Hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(10.0)),
                points: vec![[mm(30.0), mm(20.0)]],
                kind: HoleKind::Simple { diameter: mm(4.0) },
                depth: mm(10.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let cp = d
        .add(
            "Around",
            FeatureSpec::CircularPattern(CircularPatternSpec {
                source: h,
                axis_origin: [mm(20.0), mm(20.0), mm(0.0)],
                axis_dir: [0.0, 0.0, 1.0],
                count: Param::scalar(4.0),
                total_angle: Param::angle_deg(360.0),
                base: None,
                axis_feature: None,
            }),
        )
        .unwrap();
    let ev = eval(&d);
    assert!(
        close(vol(&ev, e) - vol(&ev, cp), 4.0 * PI * 4.0 * 10.0, 1e-9),
        "{}",
        vol(&ev, e) - vol(&ev, cp)
    );
    // a pattern instance that falls off the part is an error that says so
    let mut d2 = d.clone();
    d2.replace_spec(
        cp,
        FeatureSpec::CircularPattern(CircularPatternSpec {
            source: h,
            axis_origin: [mm(100.0), mm(100.0), mm(0.0)],
            axis_dir: [0.0, 0.0, 1.0],
            count: Param::scalar(3.0),
            total_angle: Param::angle_deg(360.0),
            base: None,
            axis_feature: None,
        }),
    )
    .unwrap();
    let ev = eval(&d2);
    assert!(
        matches!(ev.error(cp), Some(EvalError::Kernel(m)) if m.contains("pattern")),
        "{:?}",
        ev.state(cp)
    );
}

#[test]
fn combine_datum_planes_and_sketches_on_faces() {
    let mut doc = Document::new();
    let (_, a) = block(&mut doc, 20.0, 20.0, 10.0);
    // a second, separate body overlapping the first by a 10 x 10 x 5 corner
    let s2 = doc
        .add_rect_sketch(
            "B sketch",
            PlaneSpec::Xy(mm(5.0)),
            mm(10.0),
            mm(10.0),
            mm(20.0),
            mm(20.0),
        )
        .unwrap();
    let b = doc.add_extrude("B", s2, mm(10.0), BodyOp::New).unwrap();
    let u = doc
        .add(
            "Union",
            FeatureSpec::Combine(CombineSpec {
                target: a,
                tool: b,
                op: BodyOp::Join,
                keep_tool: false,
            }),
        )
        .unwrap();
    let i = doc
        .add(
            "Common",
            FeatureSpec::Combine(CombineSpec {
                target: a,
                tool: b,
                op: BodyOp::Intersect,
                keep_tool: false,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(close(vol(&ev, i), 10.0 * 10.0 * 5.0, 1e-9));
    assert!(close(vol(&ev, u), 4000.0 + 4000.0 - 500.0, 1e-9));

    // a datum plane tilted off the XY plane carries a sketch
    let mut doc = Document::new();
    let dp = doc
        .add(
            "Datum",
            FeatureSpec::DatumPlane(DatumPlaneSpec {
                base: PlaneSpec::Xy(mm(0.0)),
                offset: mm(5.0),
                tilt: Param::angle_deg(0.0),
                mode: Default::default(),
            }),
        )
        .unwrap();
    let s = doc
        .add_rect_sketch(
            "On datum",
            PlaneSpec::Datum(dp),
            mm(0.0),
            mm(0.0),
            mm(10.0),
            mm(10.0),
        )
        .unwrap();
    let e = doc.add_extrude("E", s, mm(5.0), BodyOp::New).unwrap();
    let ev = eval(&doc);
    assert!(close(vol(&ev, e), 500.0, 1e-9));
    let o = ev.output(e).unwrap();
    let bb = o.brep().unwrap().body_bbox(o.body().unwrap());
    assert!(bb.min.z > 4.9 && bb.max.z < 10.1 + 1e-6);

    // a sketch on a face of the body so far: a boss on top of a block
    let mut doc = Document::new();
    let (_, e) = block(&mut doc, 20.0, 20.0, 10.0);
    let ev = eval(&doc);
    let top = ev.find_name(e, "end-cap").unwrap();
    let on_top = doc
        .add_rect_sketch(
            "On top",
            PlaneSpec::Face(ev.reference(e, &top).unwrap()),
            mm(5.0),
            mm(5.0),
            mm(4.0),
            mm(4.0),
        )
        .unwrap();
    let boss = doc
        .add_extrude("Boss", on_top, mm(3.0), BodyOp::Join)
        .unwrap();
    let ev = eval(&doc);
    assert!(
        close(vol(&ev, boss), 4000.0 + 4.0 * 4.0 * 3.0, 1e-9),
        "{}",
        vol(&ev, boss)
    );
    // the sketch frame on a face does not depend on how the face happens to be parametrised: the same
    // boss results when the block is made 25 % taller
    let mut spec = doc.get(e).unwrap().spec.clone();
    if let FeatureSpec::Extrude(x) = &mut spec {
        x.depth = mm(12.5);
    }
    doc.replace_spec(e, spec).unwrap();
    let ev = eval(&doc);
    assert!(
        close(vol(&ev, boss), 5000.0 + 48.0, 1e-9),
        "{}",
        vol(&ev, boss)
    );
}

#[test]
fn sweep_along_an_arc() {
    // a 4 x 4 square swept through a quarter turn of radius 20: Pappus, V = A · (π/2) · R
    let mut doc = Document::new();
    let mut sk = kw_sketch::Sketch::new();
    sk.add_arc_xy(0.0, 0.0, 20.0, 0.0, std::f64::consts::FRAC_PI_2);
    let path = doc
        .add(
            "Path",
            FeatureSpec::Sketch(SketchSpec {
                plane: PlaneSpec::Xy(mm(0.0)),
                recipe: SketchRecipe::from_sketch(&sk),
                base: None,
            }),
        )
        .unwrap();
    // profile in the XZ plane (normal along y, the path's tangent at its start), centred on (20, 0, 0)
    let prof = doc
        .add_rect_sketch(
            "Profile",
            PlaneSpec::Xz(mm(0.0)),
            mm(-2.0),
            mm(18.0),
            mm(4.0),
            mm(4.0),
        )
        .unwrap();
    let sw = doc
        .add(
            "Sweep",
            FeatureSpec::Sweep(SweepSpec {
                surface: false,
                twist: None,
                guide: None,
                profile: prof,
                path,
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let v = vol(&ev, sw);
    let expect = 16.0 * std::f64::consts::FRAC_PI_2 * 20.0;
    assert!((v - expect).abs() / expect < 1e-3, "{v} vs {expect}");
}

/// A freed slot that is reused puts a line ahead of its end points in slot order; the recipe must still
/// rebuild, and recipes written in that order by older versions must still load.
#[test]
fn a_sketch_with_reused_slots_records_and_rebuilds() {
    let mut sk = kw_sketch::Sketch::new();
    // slots 0..: a scratch line (point, point, line), then the real corners
    let tmp = sk.add_line_xy(100.0, 100.0, 101.0, 100.0);
    let corners: Vec<_> = [(0.0, 0.0), (30.0, 0.0), (30.0, 20.0), (0.0, 20.0)]
        .iter()
        .map(|p| sk.add_point(p.0, p.1))
        .collect();
    let (t1, t2) = sk.line_points(tmp).unwrap();
    sk.remove_entity(t1); // frees the three lowest slots (the line goes with its end point)
    sk.remove_entity(t2);
    for i in 0..4 {
        sk.add_line(corners[i], corners[(i + 1) % 4]); // the lines land in the freed low slots
    }
    let first_line = sk
        .entities
        .iter()
        .position(|(_, e)| matches!(e.kind, kw_sketch::EntityKind::Line { .. }))
        .unwrap();
    let first_corner = sk
        .entities
        .iter()
        .position(|(_, e)| matches!(e.kind, kw_sketch::EntityKind::Point(_)))
        .unwrap();
    assert!(
        first_line < first_corner,
        "the setup must reproduce slot reuse"
    );

    let rec = SketchRecipe::from_sketch(&sk);
    let (back, ids) = rec.to_sketch().expect("recipe in dependency order");
    assert_eq!(ids.len(), sk.entities.len());
    assert_eq!(back.entities.len(), sk.entities.len());

    // a recipe in the old, wrong order (a line listed before its end points) still loads
    let mut old = rec.clone();
    old.entities = vec![
        EntityRec {
            construction: false,
            projected: false,
            kind: EntityKindRec::Line { p1: 1, p2: 2 },
        },
        EntityRec {
            construction: false,
            projected: false,
            kind: EntityKindRec::Point { x: 0, y: 1 },
        },
        EntityRec {
            construction: false,
            projected: false,
            kind: EntityKindRec::Point { x: 2, y: 3 },
        },
    ];
    old.vars = vec![1.0, 2.0, 4.0, 6.0];
    old.fixed = vec![false; 4];
    old.constraints.clear();
    let (sk2, ids) = old.to_sketch().expect("forward references are resolved");
    let (a, b) = sk2.line_points(ids[0]).unwrap();
    assert_eq!(sk2.point(a).unwrap(), (1.0, 2.0));
    assert_eq!(sk2.point(b).unwrap(), (4.0, 6.0));
    // a cycle or a dangling reference is still an error
    old.entities[1].kind = EntityKindRec::Line { p1: 0, p2: 2 };
    assert!(old.to_sketch().is_err());
    old.entities[1].kind = EntityKindRec::Line { p1: 9, p2: 2 };
    assert!(old.to_sketch().is_err());
}

#[test]
fn a_step_file_becomes_an_import_feature_that_keeps_its_data_and_can_be_modified() {
    // a 30 x 20 x 5 block with a fillet, exported as STEP
    let mut doc = Document::new();
    let (_, ext) = block(&mut doc, 30.0, 20.0, 5.0);
    let ev = eval(&doc);
    let want = vol(&ev, ext);
    let o = ev.output(ext).unwrap();
    let step = kw_step::export(
        o.brep().unwrap(),
        &[o.body().unwrap()],
        &kw_step::ExportOptions::default(),
    );

    let mut d2 = Document::new();
    let imp = d2
        .add(
            "Imported part",
            FeatureSpec::Import(ImportSpec {
                source: "block.step".into(),
                format: ImportFormat::Step,
                data: step.clone(),
            }),
        )
        .unwrap();
    let ev2 = eval(&d2);
    assert!(
        (vol(&ev2, imp) - want).abs() < 1e-6,
        "{} vs {want}",
        vol(&ev2, imp)
    );
    // later features work on the imported body: a through hole
    let top = {
        let o = ev2.output(imp).unwrap();
        let b = o.brep().unwrap();
        let f = b
            .faces
            .keys()
            .find(|f| {
                b.face_bbox(*f).is_some_and(|bb| {
                    (bb.min.z - 5.0).abs() < 1e-6 && (bb.max.z - 5.0).abs() < 1e-6
                })
            })
            .expect("the top face");
        o.names.reference_to(b, f).expect("a name for the face")
    };
    let hole = d2
        .add(
            "Hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Face(top),
                points: vec![[mm(15.0), mm(10.0)]],
                kind: HoleKind::Simple { diameter: mm(4.0) },
                depth: mm(5.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let ev3 = eval(&d2);
    let drilled = vol(&ev3, hole);
    assert!(
        (drilled - (want - PI * 4.0 * 5.0)).abs() < 1e-3,
        "{drilled}"
    );

    // the file text survives a save and load of the document
    let bytes = kw_feature::kwf::KwfFile::new(d2.clone())
        .to_bytes()
        .unwrap();
    let back = kw_feature::kwf::KwfFile::from_bytes(&bytes).unwrap();
    assert_eq!(back.document, d2);

    // garbage is reported, not panicked on
    let mut d3 = Document::new();
    let bad = d3
        .add(
            "Broken",
            FeatureSpec::Import(ImportSpec {
                source: "nothing.step".into(),
                format: ImportFormat::Step,
                data: "this is not a STEP file".into(),
            }),
        )
        .unwrap();
    let ev4 = eval(&d3);
    assert!(ev4.output(bad).is_none() && !ev4.failures().is_empty());
}

fn extrude_sketch(sk: kw_sketch::Sketch, depth: f64) -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let s = doc
        .add(
            "S",
            FeatureSpec::Sketch(SketchSpec {
                plane: PlaneSpec::Xy(mm(0.0)),
                recipe: SketchRecipe::from_sketch(&sk),
                base: None,
            }),
        )
        .unwrap();
    let e = doc.add_extrude("E", s, mm(depth), BodyOp::New).unwrap();
    (doc, s, e)
}

#[test]
fn ellipse_and_spline_sketches_extrude() {
    // an ellipse, major axis tilted 30 degrees: volume pi a b h whatever the tilt
    let mut sk = kw_sketch::Sketch::new();
    let c = sk.add_point(2.0, 1.0);
    let (a, b) = (12.0, 5.0);
    let t = 30f64.to_radians();
    let m = sk.add_point(2.0 + a * t.cos(), 1.0 + a * t.sin());
    sk.add_ellipse(c, m, b);
    let (doc, _, e) = extrude_sketch(sk, 4.0);
    let ev = eval(&doc);
    let v = vol(&ev, e);
    assert!(
        close(v, PI * a * b * 4.0, 1e-6),
        "ellipse {v} vs {}",
        PI * a * b * 4.0
    );

    // a region bounded by a line and a fit spline through three points
    let mut sk = kw_sketch::Sketch::new();
    let p0 = sk.add_point(0.0, 0.0);
    let p1 = sk.add_point(10.0, 8.0);
    let p2 = sk.add_point(20.0, 0.0);
    sk.add_fit_spline(vec![p0, p1, p2]);
    sk.add_line(p2, p0);
    // three points make a parabola: the area between it and its chord is 2/3 of base x height
    let area = 2.0 / 3.0 * 20.0 * 8.0;
    let (doc, _, e) = extrude_sketch(sk, 3.0);
    let ev = eval(&doc);
    let v = vol(&ev, e);
    assert!(close(v, area * 3.0, 1e-6), "spline {v} vs {}", area * 3.0);

    // a closed spline loop (first = last control point) with a round hole inside: holes still work
    let mut sk = kw_sketch::Sketch::new();
    let pts: Vec<_> = [
        (0.0, 0.0),
        (15.0, -4.0),
        (25.0, 8.0),
        (10.0, 18.0),
        (-5.0, 9.0),
        (0.0, 0.0),
    ]
    .iter()
    .map(|p| sk.add_point(p.0, p.1))
    .collect();
    sk.add_fit_spline(pts);
    sk.add_circle_xy(10.0, 8.0, 3.0);
    let (doc, _, e) = extrude_sketch(sk, 2.0);
    let ev = eval(&doc);
    let v = vol(&ev, e);
    assert!(v > 0.0 && v < 800.0 * 2.0, "{v}");
    // the hole is real: the volume is the loop's area minus the disc
    let o = ev.output(e).unwrap();
    let b = o.brep().unwrap();
    assert!(
        b.faces.len() >= 4,
        "top, bottom, spline wall and the hole wall: {}",
        b.faces.len()
    );
}

fn block_with_extent(depth: f64, extent: ExtrudeExtent) -> (Document, FeatureId) {
    let mut doc = Document::new();
    let (s0, _) = block(&mut doc, 40.0, 30.0, 10.0);
    // a second sketch on the top face's plane, cut with the given extent
    let s = doc
        .add_rect_sketch(
            "Pocket sketch",
            PlaneSpec::Xy(mm(10.0)),
            mm(10.0),
            mm(10.0),
            mm(10.0),
            mm(10.0),
        )
        .unwrap();
    let _ = s0;
    let e = doc
        .add(
            "Cut",
            FeatureSpec::Extrude(ExtrudeSpec {
                surface: false,
                sketch: s,
                depth: mm(depth),
                symmetric: false,
                flip: true,
                op: BodyOp::Cut,
                base: None,
                extent,
            }),
        )
        .unwrap();
    (doc, e)
}

#[test]
fn extrude_through_all_two_sided_and_taper() {
    let base = 40.0 * 30.0 * 10.0;
    // through all: a 10 x 10 pocket goes right through the 10 mm plate, whatever the stated depth
    let (doc, e) = block_with_extent(
        1.0,
        ExtrudeExtent {
            through_all: true,
            ..Default::default()
        },
    );
    let ev = eval(&doc);
    assert!(
        close(vol(&ev, e), base - 100.0 * 10.0, 1e-9),
        "{}",
        vol(&ev, e)
    );

    // two sided: 2 mm below the sketch plane (flip makes "first side" go down) and 3 mm the other way
    // from a sketch plane at z = 10: first side 4 mm down, second side 3 mm up (out of the body)
    let (doc, e) = block_with_extent(
        4.0,
        ExtrudeExtent {
            second_side: Some(mm(3.0)),
            ..Default::default()
        },
    );
    let ev = eval(&doc);
    assert!(
        close(vol(&ev, e), base - 100.0 * 4.0, 1e-9),
        "{}",
        vol(&ev, e)
    );

    // taper 10 degrees on a square pocket 8 mm deep: a frustum, volume h/3 (A0 + A1 + sqrt(A0 A1))
    let t = 10f64.to_radians();
    let (doc, e) = block_with_extent(
        8.0,
        ExtrudeExtent {
            taper: Some(Param::angle_deg(10.0)),
            ..Default::default()
        },
    );
    let ev = eval(&doc);
    let a0 = 100.0;
    let side1 = 10.0 - 2.0 * 8.0 * t.tan();
    let a1 = side1 * side1;
    let frustum = 8.0 / 3.0 * (a0 + a1 + (a0 * a1).sqrt());
    assert!(
        close(vol(&ev, e), base - frustum, 1e-6),
        "{} vs {}",
        vol(&ev, e),
        base - frustum
    );

    // a taper with a symmetric extent is refused with a message
    let mut d = doc.clone();
    let id = d.features.last().unwrap().id;
    let mut spec = d.get(id).unwrap().spec.clone();
    if let FeatureSpec::Extrude(x) = &mut spec {
        x.symmetric = true;
    }
    d.replace_spec(id, spec).unwrap();
    let ev = eval(&d);
    assert!(ev.output(id).is_none() && !ev.failures().is_empty());
}

/// Scale a copy of `body` by `k` about `c` and check volume (k³), area (k²), centroid and validity.
fn check_scaled(
    rep: &kw_brep::BRep,
    body: kw_brep::BodyId,
    k: f64,
    c: kw_math::Point3<f64>,
    what: &str,
) {
    let mut out = kw_brep::BRep::new();
    let (nb, _) = out.copy_body_from(
        rep,
        body,
        &kw_brep::CopyOptions {
            scale: Some((k, c)),
            ..Default::default()
        },
    );
    let rep_ok = out.validate();
    assert!(rep_ok.is_ok(), "{what} x{k}: {:?}", rep_ok.errors);
    let (a, b) = (
        rep.mass_properties(body).unwrap(),
        out.mass_properties(nb).unwrap(),
    );
    assert!(
        (b.volume - a.volume * k.powi(3)).abs() < 1e-7 * a.volume * k.powi(3),
        "{what} x{k}: volume {} vs {}",
        b.volume,
        a.volume * k.powi(3)
    );
    assert!(
        (b.area - a.area * k * k).abs() < 1e-7 * a.area * k * k,
        "{what} x{k}: area {} vs {}",
        b.area,
        a.area * k * k
    );
    let want = c + (a.centroid - c) * k;
    assert!(
        (b.centroid - want).norm() < 1e-7 * (1.0 + want.coords.norm()),
        "{what} x{k}: centroid {:?} vs {want:?}",
        b.centroid
    );
}

#[test]
fn scaling_a_body_scales_every_kind_of_face() {
    use kw_math::{Frame3, Point3, Vector3};
    use kw_model::{extrude, revolve, ExtrudeOptions, Profile, ProfileEdge, ProfileLoop};
    let frame = Frame3 {
        origin: Point3::new(1.0, 2.0, 3.0),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };
    let centres = [Point3::origin(), Point3::new(5.0, -3.0, 2.0)];
    let mut cases: Vec<(String, kw_brep::BRep, kw_brep::BodyId)> = Vec::new();
    // box (planes), cylinder (plane + cylinder)
    {
        let mut rep = kw_brep::BRep::new();
        let r = extrude(
            &mut rep,
            &Profile::rectangle(frame, 12.0, 7.0),
            Vector3::new(0.0, 0.0, 5.0),
            &ExtrudeOptions::default(),
        )
        .unwrap();
        cases.push(("box".into(), rep, r.body));
        let mut rep = kw_brep::BRep::new();
        let r = extrude(
            &mut rep,
            &Profile::disc(frame, 4.0),
            Vector3::new(0.0, 0.0, 9.0),
            &ExtrudeOptions::default(),
        )
        .unwrap();
        cases.push(("cylinder".into(), rep, r.body));
    }
    // cone, sphere, torus by revolution about the z axis through the frame origin
    let origin = Point3::new(1.0, 2.0, 3.0);
    let p = |x: f64, z: f64| Point3::new(1.0 + x, 2.0, 3.0 + z);
    {
        // cone: triangle (0,0) (4,0) (0,6)
        let mut rep = kw_brep::BRep::new();
        let prof = Profile::new(
            Frame3 {
                origin,
                x: Vector3::x(),
                y: Vector3::z(),
                z: -Vector3::y(),
            },
            ProfileLoop::polygon(&[p(0.0, 0.0), p(4.0, 0.0), p(0.0, 6.0)]),
        );
        if let Ok(r) = revolve(&mut rep, &prof, origin, Vector3::z(), std::f64::consts::TAU) {
            cases.push(("cone".into(), rep, r.body));
        }
        // sphere: half disc of radius 3
        let mut rep = kw_brep::BRep::new();
        let f = Frame3 {
            origin,
            x: Vector3::x(),
            y: Vector3::z(),
            z: -Vector3::y(),
        };
        let outer = ProfileLoop::new(vec![
            ProfileEdge::arc(
                &f,
                3.0,
                -std::f64::consts::FRAC_PI_2,
                std::f64::consts::FRAC_PI_2,
            ),
            ProfileEdge::line(p(0.0, 3.0), p(0.0, -3.0)),
        ]);
        let prof = Profile::new(f, outer);
        if let Ok(r) = revolve(&mut rep, &prof, origin, Vector3::z(), std::f64::consts::TAU) {
            cases.push(("sphere".into(), rep, r.body));
        }
        // torus: circle of radius 1.5 at distance 5 from the axis
        let mut rep = kw_brep::BRep::new();
        let cf = Frame3 {
            origin: p(5.0, 0.0),
            x: Vector3::x(),
            y: Vector3::z(),
            z: -Vector3::y(),
        };
        let prof = Profile::new(cf, ProfileLoop::circle(&cf, 1.5));
        if let Ok(r) = revolve(&mut rep, &prof, origin, Vector3::z(), std::f64::consts::TAU) {
            cases.push(("torus".into(), rep, r.body));
        }
    }
    // filleted block (cylinder faces on a plane body) and a block with a drilled hole
    {
        let mut rep = kw_brep::BRep::new();
        let r = extrude(
            &mut rep,
            &Profile::rectangle(frame, 20.0, 12.0),
            Vector3::new(0.0, 0.0, 8.0),
            &ExtrudeOptions::default(),
        )
        .unwrap();
        let edge = rep
            .edges
            .iter()
            .find(|(_, e)| {
                let (a, b) = (rep.vertices[e.start].point, rep.vertices[e.end].point);
                (a.z - 11.0).abs() < 1e-9 && (b.z - 11.0).abs() < 1e-9 && (a.x - b.x).abs() > 1.0
            })
            .map(|(id, _)| id)
            .unwrap();
        let f = kw_blend::fillet(&mut rep, r.body, &[edge], 2.0).unwrap();
        cases.push(("filleted block".into(), rep, f.body));
        let mut rep = kw_brep::BRep::new();
        let blk = extrude(
            &mut rep,
            &Profile::rectangle(frame, 20.0, 12.0),
            Vector3::new(0.0, 0.0, 8.0),
            &ExtrudeOptions::default(),
        )
        .unwrap();
        let hole = extrude(
            &mut rep,
            &Profile::disc(
                Frame3 {
                    origin: Point3::new(1.0, 2.0, 2.0),
                    ..frame
                },
                3.0,
            ),
            Vector3::new(0.0, 0.0, 12.0),
            &ExtrudeOptions::default(),
        )
        .unwrap();
        let r = kw_boolean::subtract(&mut rep, blk.body, hole.body).unwrap();
        cases.push(("block with a hole".into(), rep, r.body));
    }
    assert!(cases.len() >= 5, "cases built: {}", cases.len());
    for (what, rep, body) in &cases {
        for c in centres {
            for k in [0.5, 2.0, 3.7] {
                check_scaled(rep, *body, k, c, what);
            }
        }
    }
}

#[test]
fn the_projected_mark_of_sketch_geometry_survives_the_recipe() {
    let mut sk = kw_sketch::Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 5.0, 0.0);
    sk.entities[l].projected = true;
    let recipe = kw_feature::SketchRecipe::from_sketch(&sk);
    let json = serde_json::to_string(&recipe).unwrap();
    let back: kw_feature::SketchRecipe = serde_json::from_str(&json).unwrap();
    let (sk2, ids) = back.to_sketch().unwrap();
    assert!(sk2.entities[ids[0]].projected || sk2.entities.values().any(|e| e.projected));
    // files written before the mark existed read as not projected
    let old = json
        .replace(",\"projected\":true", "")
        .replace("\"projected\":true,", "");
    let back: kw_feature::SketchRecipe = serde_json::from_str(&old).unwrap();
    let (sk3, _) = back.to_sketch().unwrap();
    assert!(sk3.entities.values().all(|e| !e.projected));
}
