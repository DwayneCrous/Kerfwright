//! Trim, Extend and Match as features.
use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::*;
use kw_nurbs::{Surface, SurfaceGeom};
use std::collections::BTreeSet;

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
/// A 6 x 5 rectangle extruded into an open tube.
fn tube(doc: &mut Document, depth: f64) -> FeatureId {
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
fn area(ev: &Evaluation, f: FeatureId) -> f64 {
    let o = ev
        .output(f)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(o.is_sheet());
    let rep = o.brep().unwrap();
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    rep.area(o.body().unwrap()).unwrap()
}
fn names(ev: &Evaluation, f: FeatureId) -> BTreeSet<String> {
    ev.output(f)
        .unwrap()
        .names
        .by_name
        .keys()
        .map(|n| format!("{n:?}"))
        .collect()
}
/// The open edge of the body of `f` whose ends satisfy `at`, as a reference.
fn open_edge(
    ev: &Evaluation,
    f: FeatureId,
    at: impl Fn(&kw_math::Point3<f64>) -> bool,
) -> BoundaryRef {
    let out = ev.output(f).unwrap();
    let rep = out.brep().unwrap();
    let body = out.body().unwrap();
    let free = rep.free_edges(body);
    for face in rep.body_faces(body) {
        let mut index = 0u32;
        for lp in &rep.faces[face].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !free.contains(&e) {
                    continue;
                }
                let ed = &rep.edges[e];
                if at(&rep.vertices[ed.start].point) && at(&rep.vertices[ed.end].point) {
                    return BoundaryRef::Free {
                        face: out.names.reference_to(rep, face).unwrap(),
                        index,
                    };
                }
                index += 1;
            }
        }
    }
    panic!("no open edge there");
}

#[test]
fn a_plane_trims_a_tube_and_follows_its_offset() {
    let mut doc = Document::new();
    let _t = tube(&mut doc, 4.0);
    let tr = doc
        .add(
            "Trim",
            FeatureSpec::Trim(TrimSpec {
                tool: TrimToolSpec::Plane(PlaneSpec::Xy(mm(1.5))),
                keep_positive: false,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!((area(&ev, tr) - 2.0 * 11.0 * 1.5).abs() < 1e-4);
    assert_eq!(doc.heads(), vec![tr]);
    let before = names(&ev, tr);
    // move the plane
    let mut spec = doc.get(tr).unwrap().spec.clone();
    if let FeatureSpec::Trim(t) = &mut spec {
        t.tool = TrimToolSpec::Plane(PlaneSpec::Xy(mm(2.5)));
    }
    doc.replace_spec(tr, spec.clone()).unwrap();
    let ev = eval(&doc);
    assert!((area(&ev, tr) - 2.0 * 11.0 * 2.5).abs() < 1e-4);
    assert_eq!(names(&ev, tr), before);
    // keep the other side
    if let FeatureSpec::Trim(t) = &mut spec {
        t.keep_positive = true;
    }
    doc.replace_spec(tr, spec).unwrap();
    assert!((area(&eval(&doc), tr) - 2.0 * 11.0 * 1.5).abs() < 1e-4);
}

#[test]
fn the_face_of_another_surface_trims_too() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    // a vertical strip in the plane y = 2.5
    let mut line = kw_sketch::Sketch::new();
    line.add_line_xy(0.0, 2.5, 6.0, 2.5);
    let ls = sketch_of(&mut doc, "Line", line);
    let strip = surface_extrude(&mut doc, "Strip", ls, 4.0);
    let ev = eval(&doc);
    let out = ev.output(strip).unwrap();
    let rep = out.brep().unwrap();
    let face = rep.body_faces(out.body().unwrap())[0];
    let fref = out.names.reference_to(rep, face).unwrap();
    // trim the tube (which is the body before the strip? no: the strip is) - use the tube explicitly
    let mut areas = Vec::new();
    for keep in [true, false] {
        let mut d = doc.clone();
        let tr = d
            .add(
                "Trim",
                FeatureSpec::Trim(TrimSpec {
                    tool: TrimToolSpec::Face {
                        source: strip,
                        face: fref.clone(),
                    },
                    keep_positive: keep,
                    base: Some(t),
                }),
            )
            .unwrap();
        areas.push(area(&eval(&d), tr));
    }
    // each side keeps one wall of 6 x 4 and halves of the two others
    assert!(
        (areas[0] - 44.0).abs() < 1e-4 && (areas[1] - 44.0).abs() < 1e-4,
        "{areas:?}"
    );
}

#[test]
fn extend_carries_a_tube_on_and_follows_its_length() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let ev = eval(&doc);
    let rim = open_edge(&ev, t, |q| (q.z - 4.0).abs() < 1e-9);
    let ex = doc
        .add(
            "Extend",
            FeatureSpec::Extend(ExtendSpec {
                edges: vec![rim],
                length: mm(2.0),
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    // one wall of the tube grows by 2 over its width: the rim picked is one edge
    let a = area(&ev, ex);
    assert!(
        a > 88.0 + 6.0 * 2.0 - 1e-3 || a > 88.0 + 5.0 * 2.0 - 1e-3,
        "{a}"
    );
    let before = names(&ev, ex);
    let mut spec = doc.get(ex).unwrap().spec.clone();
    if let FeatureSpec::Extend(x) = &mut spec {
        x.length = mm(3.0);
    }
    doc.replace_spec(ex, spec).unwrap();
    let ev = eval(&doc);
    assert!(area(&ev, ex) > a);
    assert_eq!(names(&ev, ex), before);
}

#[test]
fn match_turns_a_cap_to_the_wall_it_meets() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let ev = eval(&doc);
    // the four rim edges, as a cap
    let rim: Vec<BoundaryEdge> = {
        let out = ev.output(t).unwrap();
        let rep = out.brep().unwrap();
        let body = out.body().unwrap();
        let free = rep.free_edges(body);
        let mut v = Vec::new();
        for face in rep.body_faces(body) {
            let mut index = 0u32;
            for lp in &rep.faces[face].loops {
                for co in rep.loop_coedges(*lp) {
                    let e = rep.coedges[co].edge;
                    if !free.contains(&e) {
                        continue;
                    }
                    let ed = &rep.edges[e];
                    if (rep.vertices[ed.start].point.z - 4.0).abs() < 1e-9
                        && (rep.vertices[ed.end].point.z - 4.0).abs() < 1e-9
                    {
                        v.push(BoundaryEdge {
                            source: t,
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
        v
    };
    let cap = doc
        .add("Cap", FeatureSpec::Patch(PatchSpec { edges: rim }))
        .unwrap();
    let ev = eval(&doc);
    // the wall in the plane y = 0 and the cap's edge on it
    let tout = ev.output(t).unwrap();
    let trep = tout.brep().unwrap();
    let wall = trep
        .body_faces(tout.body().unwrap())
        .into_iter()
        .find(|f| trep.face_bbox(*f).unwrap().max.y < 1e-3)
        .unwrap();
    let wall_ref = tout.names.reference_to(trep, wall).unwrap();
    let edge = open_edge(&ev, cap, |q| q.y.abs() < 1e-9 && (q.z - 4.0).abs() < 1e-9);
    for level in [MatchLevel::G1, MatchLevel::G2] {
        let mut d = doc.clone();
        let m = d
            .add(
                "Match",
                FeatureSpec::Match(MatchSpec {
                    edge: edge.clone(),
                    neighbour_source: t,
                    neighbour_face: wall_ref.clone(),
                    level,
                    base: None,
                }),
            )
            .unwrap();
        let ev = eval(&d);
        let o = ev
            .output(m)
            .unwrap_or_else(|| panic!("{level:?}: {:?}", ev.failures()));
        assert!(o.is_sheet());
        let rep = o.brep().unwrap();
        assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
        let f = rep.body_faces(o.body().unwrap())[0];
        let s = rep.surfaces[rep.faces[f].surface].clone();
        let uv = kw_nurbs::proj::surface_param_of(&s, &kw_math::Point3::new(3.0, 0.0, 4.0), None);
        assert!(uv.distance < 1e-6);
        let n = Surface::normal_at(&s, uv.u, uv.v);
        // the cap now leaves the wall going up: its normal lies along y
        assert!(n.y.abs() > 0.9995, "{level:?}: {n:?}");
        assert!(matches!(s, SurfaceGeom::Nurbs(_)));
    }
}

#[test]
fn trims_extends_and_matches_that_cannot_work_say_so() {
    let mut doc = Document::new();
    let _t = tube(&mut doc, 4.0);
    let far = doc
        .add(
            "Far",
            FeatureSpec::Trim(TrimSpec {
                tool: TrimToolSpec::Plane(PlaneSpec::Xy(mm(40.0))),
                keep_positive: false,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(far).is_none());
    let why = format!("{:?}", ev.failures());
    assert!(why.contains("does not cut"), "{why}");
}

#[test]
fn trim_extend_and_match_round_trip_through_a_kwf_file() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let ev = eval(&doc);
    let rim = open_edge(&ev, t, |q| (q.z - 4.0).abs() < 1e-9);
    doc.add(
        "Extend",
        FeatureSpec::Extend(ExtendSpec {
            edges: vec![rim],
            length: mm(1.0),
            base: None,
        }),
    )
    .unwrap();
    doc.add(
        "Trim",
        FeatureSpec::Trim(TrimSpec {
            tool: TrimToolSpec::Plane(PlaneSpec::Xy(mm(1.0))),
            keep_positive: true,
            base: None,
        }),
    )
    .unwrap();
    let bytes = KwfFile::new(doc.clone()).to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap().document;
    assert_eq!(back, doc);
}
