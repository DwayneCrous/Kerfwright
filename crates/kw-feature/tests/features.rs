//! Every feature kind the executor knows, once: it evaluates, gives a valid solid with the volume the
//! geometry says, and names its faces.

use kw_expr::Param;
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
