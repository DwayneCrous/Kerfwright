//! Patch and Ruled features: surfaces between the open edges of other surfaces.
use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::*;
use kw_math::Vector3;
use kw_nurbs::{Surface, SurfaceGeom};

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
/// A 6 x 5 rectangle extruded into an open tube of the given height.
fn tube(doc: &mut Document, height: f64) -> FeatureId {
    let mut sk = kw_sketch::Sketch::new();
    let p: Vec<_> = [(0.0, 0.0), (6.0, 0.0), (6.0, 5.0), (0.0, 5.0)]
        .iter()
        .map(|q| sk.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
    let s = sketch_of(doc, "Outline", sk);
    surface_extrude(doc, "Tube", s, height)
}

/// Which open edge of a face lies at the height `z` (its index among the face's open edges).
fn open_edge_at(ev: &Evaluation, f: FeatureId, face: usize, z: f64) -> BoundaryEdge {
    let out = ev
        .output(f)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    let rep = out.brep().unwrap();
    let body = out.body().unwrap();
    let faces = rep.body_faces(body);
    let fid = faces[face];
    let free = rep.free_edges(body);
    let mut index = 0u32;
    let mut found = None;
    for lp in &rep.faces[fid].loops {
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
                found = Some(index);
            }
            index += 1;
        }
    }
    BoundaryEdge {
        source: f,
        edge: BoundaryRef::Free {
            face: out.names.reference_to(rep, fid).expect("a named face"),
            index: found.expect("an open edge at that height"),
        },
        tangent: false,
    }
}

fn rim(ev: &Evaluation, tube: FeatureId, z: f64) -> Vec<BoundaryEdge> {
    (0..4).map(|i| open_edge_at(ev, tube, i, z)).collect()
}

fn area_of(ev: &Evaluation, id: FeatureId) -> f64 {
    let o = ev
        .output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(o.is_sheet());
    let rep = o.brep().unwrap();
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    rep.area(o.body().unwrap()).unwrap()
}

#[test]
fn a_patch_caps_the_open_rim_of_a_surface_and_keeps_the_tube() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let ev = eval(&doc);
    let edges = rim(&ev, t, 4.0);
    let patch = doc
        .add("Cap", FeatureSpec::Patch(PatchSpec { edges }))
        .unwrap();
    let ev = eval(&doc);
    assert!((area_of(&ev, patch) - 30.0).abs() < 1e-5);
    let out = ev.output(patch).unwrap();
    let rep = out.brep().unwrap();
    let s = match &rep.surfaces[rep.faces[rep.body_faces(out.body().unwrap())[0]].surface] {
        SurfaceGeom::Nurbs(s) => s.clone(),
        o => panic!("{o:?}"),
    };
    assert!((s.point_at(0.3, 0.6).z - 4.0).abs() < 1e-9);
    // both surfaces are bodies of the model: the patch did not use up the tube
    assert_eq!(doc.heads().len(), 2);
}

#[test]
fn the_patch_follows_the_tube_when_its_height_changes() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let edges = rim(&eval(&doc), t, 4.0);
    let patch = doc
        .add("Cap", FeatureSpec::Patch(PatchSpec { edges }))
        .unwrap();
    assert!((area_of(&eval(&doc), patch) - 30.0).abs() < 1e-5);
    // the tube becomes 7 tall: the same references find the same edges
    let mut spec = doc.get(t).unwrap().spec.clone();
    if let FeatureSpec::Extrude(x) = &mut spec {
        x.depth = mm(7.0);
    }
    doc.replace_spec(t, spec).unwrap();
    let ev = eval(&doc);
    assert!((area_of(&ev, patch) - 30.0).abs() < 1e-5);
    let out = ev.output(patch).unwrap();
    let rep = out.brep().unwrap();
    let f = rep.body_faces(out.body().unwrap())[0];
    let SurfaceGeom::Nurbs(surf) = &rep.surfaces[rep.faces[f].surface] else {
        panic!("not a NURBS surface")
    };
    assert!((surf.point_at(0.4, 0.4).z - 7.0).abs() < 1e-9);
}

#[test]
fn a_tangent_edge_makes_the_patch_leave_the_wall_the_way_the_wall_does() {
    let normal_at_middle_of_wall_edge = |tangent: bool| {
        let mut doc = Document::new();
        let t = tube(&mut doc, 4.0);
        let mut edges = rim(&eval(&doc), t, 4.0);
        // the face behind each rim edge: find the one lying in the plane y = 0 and make it tangent
        let ev = eval(&doc);
        let out = ev.output(t).unwrap();
        let rep = out.brep().unwrap();
        let wall = rep
            .body_faces(out.body().unwrap())
            .into_iter()
            .position(|f| rep.face_bbox(f).unwrap().max.y < 1e-3)
            .unwrap();
        edges[wall].tangent = tangent;
        // the wall's edge comes first so that the loop starts there
        edges.swap(0, wall);
        let patch = doc
            .add("Cap", FeatureSpec::Patch(PatchSpec { edges }))
            .unwrap();
        let ev = eval(&doc);
        let o = ev
            .output(patch)
            .unwrap_or_else(|| panic!("{:?}", ev.failures()));
        let rep = o.brep().unwrap();
        let f = rep.body_faces(o.body().unwrap())[0];
        let n = rep.surfaces[rep.faces[f].surface].clone();
        // the point in the middle of the wall edge (3, 0, 4)
        let uv = kw_nurbs::proj::surface_param_of(&n, &kw_math::Point3::new(3.0, 0.0, 4.0), None);
        assert!(uv.distance < 1e-5, "{}", uv.distance);
        n.normal_at(uv.u, uv.v)
    };
    let plain = normal_at_middle_of_wall_edge(false);
    let tangent = normal_at_middle_of_wall_edge(true);
    // the plain cap is horizontal; the tangent one starts vertical (its normal lies along y)
    assert!(plain.dot(&Vector3::z()).abs() > 0.99, "{plain:?}");
    assert!(tangent.dot(&Vector3::y()).abs() > 0.999, "{tangent:?}");
}

#[test]
fn a_ruled_surface_joins_two_rims() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    // a second, shorter tube beside it
    let mut sk = kw_sketch::Sketch::new();
    sk.add_line_xy(0.0, 20.0, 6.0, 20.0);
    let s = sketch_of(&mut doc, "Line", sk);
    let strip = surface_extrude(&mut doc, "Strip", s, 4.0);
    let ev = eval(&doc);
    // the tube's wall in the plane y = 0 and the strip are both 6 wide: the ruled surface is a 6 x 20 plane at z = 4
    let out = ev.output(t).unwrap();
    let rep = out.brep().unwrap();
    let wall = rep
        .body_faces(out.body().unwrap())
        .into_iter()
        .position(|f| rep.face_bbox(f).unwrap().max.y < 1e-3)
        .unwrap();
    let a = open_edge_at(&ev, t, wall, 4.0);
    let b = open_edge_at(&ev, strip, 0, 4.0);
    let ruled = doc
        .add("Ruled", FeatureSpec::Ruled(RuledSpec { a, b }))
        .unwrap();
    assert!((area_of(&eval(&doc), ruled) - 120.0).abs() < 1e-6);
}

#[test]
fn bad_patches_say_why() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let ev = eval(&doc);
    let mut edges = rim(&ev, t, 4.0);
    // two edges do not make a loop
    let two = doc
        .add(
            "Two",
            FeatureSpec::Patch(PatchSpec {
                edges: edges[..2].to_vec(),
            }),
        )
        .unwrap();
    // three of the four leave a gap
    let three = doc
        .add(
            "Three",
            FeatureSpec::Patch(PatchSpec {
                edges: edges[..3].to_vec(),
            }),
        )
        .unwrap();
    // an edge that is not on the face
    if let BoundaryRef::Free { index, .. } = &mut edges[0].edge {
        *index = 9;
    }
    let nowhere = doc
        .add("Nowhere", FeatureSpec::Patch(PatchSpec { edges }))
        .unwrap();
    let ev = eval(&doc);
    for f in [two, three, nowhere] {
        assert!(ev.output(f).is_none());
    }
    let why = format!("{:?}", ev.failures());
    assert!(
        why.contains("at least three") && why.contains("closed loop"),
        "{why}"
    );
    assert!(why.contains("no open edge"), "{why}");
}

#[test]
fn a_shared_edge_cannot_be_made_tangent() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let ev = eval(&doc);
    let out = ev.output(t).unwrap();
    let rep = out.brep().unwrap();
    let f = rep.body_faces(out.body().unwrap());
    let r = |i: usize| out.names.reference_to(rep, f[i]).unwrap();
    let edge = BoundaryEdge {
        source: t,
        edge: BoundaryRef::Shared(EdgeRef {
            faces: [r(0), r(1)],
        }),
        tangent: true,
    };
    let other = open_edge_at(&ev, t, 2, 4.0);
    let ruled = doc
        .add("Ruled", FeatureSpec::Ruled(RuledSpec { a: edge, b: other }))
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(ruled).is_none());
    assert!(format!("{:?}", ev.failures()).contains("two sides"));
}

#[test]
fn patch_features_round_trip_through_a_kwf_file() {
    let mut doc = Document::new();
    let t = tube(&mut doc, 4.0);
    let edges = rim(&eval(&doc), t, 4.0);
    let patch = doc
        .add("Cap", FeatureSpec::Patch(PatchSpec { edges }))
        .unwrap();
    let bytes = KwfFile::new(doc.clone()).to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap().document;
    assert_eq!(back, doc);
    assert!((area_of(&eval(&back), patch) - 30.0).abs() < 1e-5);
}
