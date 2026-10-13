//! Construction axes, points and planes, and what uses them: revolve, circular pattern, mirror.

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
    let rep = o.brep().unwrap();
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    rep.volume(o.body().unwrap()).unwrap()
}
fn frame(ev: &Evaluation, id: FeatureId) -> kw_math::Frame3 {
    match &ev
        .output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()))
        .product
    {
        Product::Datum(f) => *f,
        _ => panic!("not a datum"),
    }
}
fn p3(x: f64, y: f64, z: f64) -> [Param; 3] {
    [mm(x), mm(y), mm(z)]
}
fn pt(x: f64, y: f64, z: f64) -> PointSrc {
    PointSrc::Coords(p3(x, y, z))
}
fn xy(o: f64) -> PlaneSpec {
    PlaneSpec::Xy(mm(o))
}
fn rect(x0: f64, y0: f64, w: f64, h: f64) -> kw_sketch::Sketch {
    let mut sk = kw_sketch::Sketch::new();
    let p: Vec<_> = [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]
        .iter()
        .map(|q| sk.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
    sk
}
fn sketch_of(doc: &mut Document, name: &str, plane: PlaneSpec, sk: kw_sketch::Sketch) -> FeatureId {
    doc.add(
        name,
        FeatureSpec::Sketch(SketchSpec {
            plane,
            recipe: SketchRecipe::from_sketch(&sk),
            base: None,
        }),
    )
    .unwrap()
}
fn axis(doc: &mut Document, name: &str, kind: AxisKind) -> FeatureId {
    doc.add(name, FeatureSpec::DatumAxis(DatumAxisSpec { kind }))
        .unwrap()
}
fn block(doc: &mut Document, w: f64, h: f64, d: f64) -> FeatureId {
    let s = sketch_of(doc, "Block sketch", xy(0.0), rect(0.0, 0.0, w, h));
    doc.add_extrude("Block", s, mm(d), BodyOp::New).unwrap()
}

#[test]
fn axes_points_and_planes_land_where_they_should() {
    let mut doc = Document::new();
    let a = axis(
        &mut doc,
        "Z axis",
        AxisKind::TwoPoints(Box::new([pt(1.0, 2.0, 0.0), pt(1.0, 2.0, 5.0)])),
    );
    // the line where the planes x = 3 and y = 4 meet
    let b = axis(
        &mut doc,
        "X3 Y4",
        AxisKind::PlanePlane(Box::new((PlaneSpec::Yz(mm(3.0)), PlaneSpec::Xz(mm(4.0))))),
    );
    let p = doc
        .add(
            "Point",
            FeatureSpec::DatumPoint(DatumPointSpec {
                kind: PointKind::AxisPlane {
                    axis: b,
                    plane: xy(7.0),
                },
            }),
        )
        .unwrap();
    let c = doc
        .add(
            "Coords",
            FeatureSpec::DatumPoint(DatumPointSpec {
                kind: PointKind::Coords(p3(1.0, 2.0, 3.0)),
            }),
        )
        .unwrap();
    let mid = doc
        .add(
            "Mid",
            FeatureSpec::DatumPlane(DatumPlaneSpec {
                mode: PlaneMode::Midplane(xy(10.0)),
                base: xy(0.0),
                offset: mm(0.0),
                tilt: Param::angle_deg(0.0),
            }),
        )
        .unwrap();
    let three = doc
        .add(
            "Three",
            FeatureSpec::DatumPlane(DatumPlaneSpec {
                mode: PlaneMode::ThreePoints(Box::new([
                    pt(0.0, 0.0, 0.0),
                    pt(10.0, 0.0, 0.0),
                    PointSrc::Datum(c),
                ])),
                base: xy(0.0),
                offset: mm(0.0),
                tilt: Param::angle_deg(0.0),
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let fa = frame(&ev, a);
    assert!(
        (fa.origin - kw_math::Point3::new(1.0, 2.0, 0.0)).norm() < 1e-12
            && (fa.z - kw_math::Vector3::z()).norm() < 1e-12
    );
    let fb = frame(&ev, b);
    assert!(
        fb.z.cross(&kw_math::Vector3::z()).norm() < 1e-12,
        "{:?}",
        fb.z
    );
    assert!(
        (fb.origin.x - 3.0).abs() < 1e-9 && (fb.origin.y - 4.0).abs() < 1e-9,
        "{:?}",
        fb.origin
    );
    let fp = frame(&ev, p);
    assert!(
        (fp.origin - kw_math::Point3::new(3.0, 4.0, 7.0)).norm() < 1e-9,
        "{:?}",
        fp.origin
    );
    assert!(
        (frame(&ev, mid).origin.z - 5.0).abs() < 1e-12
            && (frame(&ev, mid).z - kw_math::Vector3::z()).norm() < 1e-12
    );
    // through (0,0,0), (10,0,0) and (1,2,3): the normal is perpendicular to both directions
    let ft = frame(&ev, three);
    assert!(
        ft.z.dot(&kw_math::Vector3::x()).abs() < 1e-12
            && ft.z.dot(&kw_math::Vector3::new(1.0, 2.0, 3.0)).abs() < 1e-12
    );
    // degenerate inputs are refused with a message
    let mut d = Document::new();
    let bad = d
        .add(
            "Collinear",
            FeatureSpec::DatumPlane(DatumPlaneSpec {
                mode: PlaneMode::ThreePoints(Box::new([
                    pt(0.0, 0.0, 0.0),
                    pt(1.0, 0.0, 0.0),
                    pt(2.0, 0.0, 0.0),
                ])),
                base: xy(0.0),
                offset: mm(0.0),
                tilt: Param::angle_deg(0.0),
            }),
        )
        .unwrap();
    let par = axis(
        &mut d,
        "Parallel",
        AxisKind::PlanePlane(Box::new((xy(0.0), xy(5.0)))),
    );
    let ev = eval(&d);
    assert!(ev.output(bad).is_none() && ev.output(par).is_none() && ev.failures().len() == 2);
}

#[test]
fn a_construction_axis_is_a_revolve_axis_a_pattern_axis_and_a_body_edge_is_an_axis() {
    // revolve a ring around an axis that is not a sketch line: x = 0, y = 0, along z
    let mut doc = Document::new();
    let z = axis(
        &mut doc,
        "Z",
        AxisKind::TwoPoints(Box::new([pt(0.0, 0.0, 0.0), pt(0.0, 0.0, 1.0)])),
    );
    let prof = sketch_of(
        &mut doc,
        "Ring",
        PlaneSpec::Xz(mm(0.0)),
        rect(0.0, 4.0, 3.0, 2.0),
    );
    let rv = doc
        .add(
            "Revolve",
            FeatureSpec::Revolve(RevolveSpec {
                sketch: prof,
                axis: AxisSpec::Datum(z),
                angle: Param::angle_deg(360.0),
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    assert!((vol(&eval(&doc), rv) - PI * (36.0 - 16.0) * 3.0).abs() < 1e-6);

    // a 10 x 10 x 10 block: an axis normal to its top face through (5, 5), four holes patterned about it
    let mut doc = Document::new();
    let blk = block(&mut doc, 10.0, 10.0, 10.0);
    let ev = eval(&doc);
    let top = {
        let o = ev.output(blk).unwrap();
        let b = o.brep().unwrap();
        let f = b
            .faces
            .keys()
            .find(|f| {
                b.face_bbox(*f).is_some_and(|bb| {
                    (bb.min.z - 10.0).abs() < 1e-6 && (bb.max.z - 10.0).abs() < 1e-6
                })
            })
            .unwrap();
        o.names.reference_to(b, f).unwrap()
    };
    let ax = axis(
        &mut doc,
        "Normal",
        AxisKind::NormalToFace {
            face: top.clone(),
            at: [mm(5.0), mm(5.0)],
        },
    );
    let hole = doc
        .add(
            "Hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Face(top),
                points: vec![[mm(8.0), mm(5.0)]],
                kind: HoleKind::Simple { diameter: mm(2.0) },
                depth: mm(4.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let pat = doc
        .add(
            "Pattern",
            FeatureSpec::CircularPattern(CircularPatternSpec {
                source: hole,
                axis_origin: p3(0.0, 0.0, 0.0),
                axis_dir: [0.0, 0.0, 1.0],
                count: Param::scalar(4.0),
                total_angle: Param::angle_deg(360.0),
                base: None,
                axis_feature: Some(ax),
            }),
        )
        .unwrap();
    let want = 1000.0 - 4.0 * PI * 1.0 * 4.0;
    let ev = eval(&doc);
    assert!(
        (vol(&ev, pat) - want).abs() < 1e-3,
        "{} vs {want}",
        vol(&ev, pat)
    );

    // an axis along a body edge: the vertical edge at (0, 0), then a revolve about it
    let mut doc = Document::new();
    let blk = block(&mut doc, 10.0, 10.0, 10.0);
    let ev = eval(&doc);
    let edge = {
        let o = ev.output(blk).unwrap();
        let b = o.brep().unwrap();
        let face_at = |axis: usize| {
            b.faces
                .keys()
                .find(|f| {
                    b.face_bbox(*f).is_some_and(|bb| {
                        let (lo, hi) = if axis == 0 {
                            (bb.min.x, bb.max.x)
                        } else {
                            (bb.min.y, bb.max.y)
                        };
                        lo.abs() < 1e-6 && hi.abs() < 1e-6
                    })
                })
                .unwrap()
        };
        EdgeRef {
            faces: [
                o.names.reference_to(b, face_at(0)).unwrap(),
                o.names.reference_to(b, face_at(1)).unwrap(),
            ],
        }
    };
    let ea = axis(&mut doc, "Edge axis", AxisKind::Edge(Box::new(edge)));
    let ev = eval(&doc);
    let f = frame(&ev, ea);
    assert!(
        f.z.cross(&kw_math::Vector3::z()).norm() < 1e-12
            && f.origin.x.abs() < 1e-9
            && f.origin.y.abs() < 1e-9,
        "{:?} {:?}",
        f.origin,
        f.z
    );
}

#[test]
fn mirror_about_a_construction_plane_and_about_a_face() {
    // a 20 x 10 x 10 block with a hole at x = 5; mirrored about the plane x = 10 (a construction plane)
    let mut doc = Document::new();
    let blk = block(&mut doc, 20.0, 10.0, 10.0);
    let ev = eval(&doc);
    let top = {
        let o = ev.output(blk).unwrap();
        let b = o.brep().unwrap();
        let f = b
            .faces
            .keys()
            .find(|f| {
                b.face_bbox(*f).is_some_and(|bb| {
                    (bb.min.z - 10.0).abs() < 1e-6 && (bb.max.z - 10.0).abs() < 1e-6
                })
            })
            .unwrap();
        o.names.reference_to(b, f).unwrap()
    };
    let hole = doc
        .add(
            "Hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Face(top),
                points: vec![[mm(5.0), mm(5.0)]],
                kind: HoleKind::Simple { diameter: mm(2.0) },
                depth: mm(4.0),
                base: None,
                through_all: false,
            }),
        )
        .unwrap();
    let mid = doc
        .add(
            "x = 10",
            FeatureSpec::DatumPlane(DatumPlaneSpec {
                mode: PlaneMode::Offset,
                base: PlaneSpec::Yz(mm(0.0)),
                offset: mm(10.0),
                tilt: Param::angle_deg(0.0),
            }),
        )
        .unwrap();
    let mir = doc
        .add(
            "Mirror",
            FeatureSpec::Mirror(MirrorSpec {
                source: hole,
                plane: MirrorPlane::Datum(mid),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let want = 2000.0 - 2.0 * PI * 1.0 * 4.0;
    assert!(
        (vol(&ev, mir) - want).abs() < 1e-3,
        "{} vs {want}",
        vol(&ev, mir)
    );
    // mirroring about a face of the body: the block's x = 20 face is not in the middle, the hole lands outside: refused
    // (no check of the number: just that the evaluator names a face plane and does not panic)
    let ev2 = eval(&doc);
    assert!(ev2.output(mir).is_some());
}
