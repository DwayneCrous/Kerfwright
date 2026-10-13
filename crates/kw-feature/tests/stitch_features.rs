//! Stitch, Unstitch, Thicken and Offset surface as features.
use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::*;
use std::f64::consts::PI;

fn mm(v: f64) -> Param {
    Param::length(v)
}
fn eval(doc: &Document) -> Evaluation {
    Engine::with_threads(2).evaluate(doc)
}
fn sketch_of(doc: &mut Document, name: &str, sk: kw_sketch::Sketch) -> FeatureId {
    doc.add(
        name,
        FeatureSpec::Sketch(SketchSpec {
            plane: PlaneSpec::Xy(mm(0.0)),
            recipe: SketchRecipe::from_sketch(&sk),
            base: None,
        }),
    )
    .unwrap()
}
fn surface_extrude(doc: &mut Document, name: &str, s: FeatureId, depth: f64) -> FeatureId {
    doc.add(
        name,
        FeatureSpec::Extrude(ExtrudeSpec {
            surface: true,
            sketch: s,
            depth: mm(depth),
            symmetric: false,
            flip: false,
            op: BodyOp::New,
            base: None,
            extent: Default::default(),
        }),
    )
    .unwrap()
}
fn rect_tube(doc: &mut Document, depth: f64) -> FeatureId {
    let mut sk = kw_sketch::Sketch::new();
    let p: Vec<_> = [(0.0, 0.0), (6.0, 0.0), (6.0, 5.0), (0.0, 5.0)]
        .iter()
        .map(|q| sk.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
    let s = sketch_of(doc, "Outline", sk);
    surface_extrude(doc, "Tube", s, depth)
}
fn circle_tube(doc: &mut Document) -> FeatureId {
    let mut sk = kw_sketch::Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 2.0);
    let s = sketch_of(doc, "Circle", sk);
    surface_extrude(doc, "Pipe", s, 3.0)
}
/// The open edges of the body of `f` that lie at height `z`, as references to patch between.
fn rim(ev: &Evaluation, f: FeatureId, z: f64) -> Vec<BoundaryEdge> {
    let out = ev
        .output(f)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    let rep = out.brep().unwrap();
    let body = out.body().unwrap();
    let free = rep.free_edges(body);
    let mut edges = Vec::new();
    for face in rep.body_faces(body) {
        let mut index = 0u32;
        for lp in &rep.faces[face].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !free.contains(&e) {
                    continue;
                }
                let (a, b) = (
                    &rep.vertices[rep.edges[e].start],
                    &rep.vertices[rep.edges[e].end],
                );
                if (a.point.z - z).abs() < 1e-9 && (b.point.z - z).abs() < 1e-9 {
                    edges.push(BoundaryEdge {
                        source: f,
                        edge: BoundaryRef::Free {
                            face: out.names.reference_to(rep, face).unwrap(),
                            index,
                        },
                        tangent: false,
                    });
                }
                index += 1;
            }
        }
    }
    edges
}
fn solid_volume(ev: &Evaluation, f: FeatureId) -> f64 {
    let o = ev
        .output(f)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(!o.is_sheet(), "still a surface");
    let rep = o.brep().unwrap();
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    rep.volume(o.body().unwrap()).unwrap()
}

#[test]
fn a_tube_and_two_caps_stitch_into_a_solid_that_uses_them_up() {
    let mut doc = Document::new();
    let tube = rect_tube(&mut doc, 4.0);
    let ev = eval(&doc);
    let bottom = doc
        .add(
            "Bottom",
            FeatureSpec::Patch(PatchSpec {
                edges: rim(&ev, tube, 0.0),
            }),
        )
        .unwrap();
    let top = doc
        .add(
            "Top",
            FeatureSpec::Patch(PatchSpec {
                edges: rim(&ev, tube, 4.0),
            }),
        )
        .unwrap();
    assert_eq!(doc.heads().len(), 3);
    let st = doc
        .add(
            "Stitch",
            FeatureSpec::Stitch(StitchSpec {
                bodies: vec![tube, bottom, top],
                tolerance: mm(0.01),
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!((solid_volume(&ev, st) - 120.0).abs() < 1e-4);
    // the three surfaces are used up: one body is left, and it is the model's solid
    assert_eq!(doc.heads(), vec![st]);
    assert_eq!(ev.final_solid().map(|(id, _)| *id), Some(st));
    // faces keep the names they had
    let names = &ev.output(st).unwrap().names.by_name;
    assert_eq!(names.len(), 6);
}

#[test]
fn stitching_refuses_solids_and_distant_surfaces() {
    let mut doc = Document::new();
    let tube = rect_tube(&mut doc, 4.0);
    let mut far = kw_sketch::Sketch::new();
    far.add_line_xy(50.0, 0.0, 56.0, 0.0);
    let s = sketch_of(&mut doc, "Far", far);
    let strip = surface_extrude(&mut doc, "Strip", s, 4.0);
    let apart = doc
        .add(
            "Apart",
            FeatureSpec::Stitch(StitchSpec {
                bodies: vec![tube, strip],
                tolerance: mm(0.01),
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(apart).is_none());
    assert!(format!("{:?}", ev.failures()).contains("separate groups"));
}

#[test]
fn a_pipe_thickens_to_the_exact_volume_and_follows_its_parameter() {
    let mut doc = Document::new();
    let pipe = circle_tube(&mut doc);
    let th = doc
        .add(
            "Thicken",
            FeatureSpec::Thicken(ThickenSpec {
                thickness: mm(0.5),
                side: ThickenDir::Both,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let want = |t: f64| PI * ((2.0 + t / 2.0f64).powi(2) - (2.0 - t / 2.0f64).powi(2)) * 3.0;
    assert!((solid_volume(&ev, th) - want(0.5)).abs() < 1e-3);
    // the thickened pipe is the model's solid, and the surface was used up
    assert_eq!(ev.final_solid().map(|(id, _)| *id), Some(th));
    assert_eq!(doc.heads(), vec![th]);
    let names_before: std::collections::BTreeSet<String> = ev
        .output(th)
        .unwrap()
        .names
        .by_name
        .keys()
        .map(|n| format!("{n:?}"))
        .collect();
    // a thicker wall: the same faces, a different volume
    let mut spec = doc.get(th).unwrap().spec.clone();
    if let FeatureSpec::Thicken(t) = &mut spec {
        t.thickness = mm(1.0);
    }
    doc.replace_spec(th, spec).unwrap();
    let ev = eval(&doc);
    assert!((solid_volume(&ev, th) - want(1.0)).abs() < 1e-3);
    let names_after: std::collections::BTreeSet<String> = ev
        .output(th)
        .unwrap()
        .names
        .by_name
        .keys()
        .map(|n| format!("{n:?}"))
        .collect();
    assert_eq!(names_before, names_after);
    let _ = pipe;
}

#[test]
fn a_solid_cannot_be_thickened_or_offset_as_a_surface() {
    let mut doc = Document::new();
    let s = doc
        .add_rect_sketch(
            "S",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(4.0),
            mm(4.0),
        )
        .unwrap();
    doc.add_extrude("Block", s, mm(4.0), BodyOp::New).unwrap();
    let th = doc
        .add(
            "Thicken",
            FeatureSpec::Thicken(ThickenSpec {
                thickness: mm(1.0),
                side: ThickenDir::Normal,
                base: None,
            }),
        )
        .unwrap();
    let off = doc
        .add(
            "Offset",
            FeatureSpec::OffsetSurface(OffsetSurfaceSpec {
                distance: mm(1.0),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(th).is_none() && ev.output(off).is_none());
    assert!(format!("{:?}", ev.failures()).contains("needs a surface"));
}

#[test]
fn offset_surface_moves_a_pipe_and_unstitch_splits_a_tube() {
    let mut doc = Document::new();
    let _pipe = circle_tube(&mut doc);
    let off = doc
        .add(
            "Offset",
            FeatureSpec::OffsetSurface(OffsetSurfaceSpec {
                distance: mm(0.5),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let o = ev
        .output(off)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(o.is_sheet());
    let area = o.brep().unwrap().area(o.body().unwrap()).unwrap();
    let (out, inn) = (2.0 * PI * 2.5 * 3.0, 2.0 * PI * 1.5 * 3.0);
    assert!(
        (area - out).abs() < 1e-4 * out || (area - inn).abs() < 1e-4 * inn,
        "{area}"
    );

    let mut doc = Document::new();
    let tube = rect_tube(&mut doc, 4.0);
    let un = doc
        .add(
            "Unstitch",
            FeatureSpec::Unstitch(UnstitchSpec { base: None }),
        )
        .unwrap();
    let ev = eval(&doc);
    let o = ev
        .output(un)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    let rep = o.brep().unwrap();
    assert_eq!(rep.body_faces(o.body().unwrap()).len(), 4);
    assert_eq!(rep.free_edges(o.body().unwrap()).len(), 16);
    assert_eq!(doc.heads(), vec![un]);
    let _ = tube;
}

#[test]
fn stitch_and_thicken_round_trip_through_a_kwf_file() {
    let mut doc = Document::new();
    let pipe = circle_tube(&mut doc);
    doc.add(
        "Thicken",
        FeatureSpec::Thicken(ThickenSpec {
            thickness: mm(0.5),
            side: ThickenDir::Reverse,
            base: None,
        }),
    )
    .unwrap();
    let st = doc
        .add(
            "Stitch",
            FeatureSpec::Stitch(StitchSpec {
                bodies: vec![pipe],
                tolerance: mm(0.05),
            }),
        )
        .unwrap();
    let bytes = KwfFile::new(doc.clone()).to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap().document;
    assert_eq!(back, doc);
    let _ = st;
}
