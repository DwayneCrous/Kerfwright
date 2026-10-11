//! Regions, thin walls, ribs, extrude to a face, start offset, revolve about an edge, straight sweep, pipe.

use kw_expr::Param;
use kw_feature::regions::sketch_regions;
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
fn extrude_with(
    doc: &mut Document,
    name: &str,
    s: FeatureId,
    depth: f64,
    op: BodyOp,
    extent: ExtrudeExtent,
) -> FeatureId {
    doc.add(
        name,
        FeatureSpec::Extrude(ExtrudeSpec {
            sketch: s,
            depth: mm(depth),
            symmetric: false,
            flip: false,
            op,
            base: None,
            extent,
        }),
    )
    .unwrap()
}
fn rect_sketch(x0: f64, y0: f64, w: f64, h: f64) -> kw_sketch::Sketch {
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
fn xy() -> PlaneSpec {
    PlaneSpec::Xy(mm(0.0))
}

/// The sketch product of an evaluated sketch feature.
fn sketch_product(ev: &Evaluation, s: FeatureId) -> std::sync::Arc<SketchProduct> {
    match ev.output(s).map(|o| &o.product) {
        Some(Product::Sketch(sp)) => sp.clone(),
        _ => panic!("not a sketch"),
    }
}

#[test]
fn chosen_regions_of_overlapping_loops_extrude_to_exactly_their_shape() {
    // a 20 x 10 rectangle and a circle of radius 5 centred on its right edge
    let mut sk = rect_sketch(0.0, 0.0, 20.0, 10.0);
    sk.add_circle_xy(20.0, 5.0, 5.0);
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "S", xy(), sk);
    let ev = eval(&doc);
    let sp = sketch_product(&ev, s);
    let sr = sketch_regions(&sp.sketch, &sp.entities, sp.plane);
    let regs = &sr.regions;
    assert_eq!(regs.len(), 3, "rectangle only, circle only, both");
    let half_disc = 0.5 * PI * 25.0;
    // the three regions, found by where they are (two of them have the same area)
    let at = |p: [f64; 2]| regs[sr.region_at(p).expect("a region there")].pick.clone();
    let both = at([17.0, 5.0]);
    let rect_only = at([5.0, 5.0]);
    let circle_only = at([23.0, 5.0]);
    assert!((regs[sr.region_at([17.0, 5.0]).unwrap()].area - half_disc).abs() < 1e-6);
    assert!((regs[sr.region_at([5.0, 5.0]).unwrap()].area - (200.0 - half_disc)).abs() < 1e-6);
    let depth = 4.0;
    let one = |doc: &Document, picks: Vec<RegionPick>| -> f64 {
        let mut d = doc.clone();
        let e = extrude_with(
            &mut d,
            "E",
            s,
            depth,
            BodyOp::New,
            ExtrudeExtent {
                regions: picks,
                ..Default::default()
            },
        );
        vol(&eval(&d), e)
    };
    let tol = 1e-6;
    assert!(
        (one(&doc, vec![both.clone()]) - half_disc * depth).abs() < tol,
        "overlap"
    );
    assert!(
        (one(&doc, vec![rect_only.clone()]) - (200.0 - half_disc) * depth).abs() < tol,
        "rectangle only"
    );
    assert!(
        (one(&doc, vec![circle_only.clone()]) - (PI * 25.0 - half_disc) * depth).abs() < tol,
        "circle only"
    );
    // two regions united: the rectangle and its overlap with the circle are the whole rectangle
    assert!(
        (one(&doc, vec![rect_only.clone(), both.clone()]) - 200.0 * depth).abs() < tol,
        "rectangle"
    );
    // all three: the union of both shapes
    assert!(
        (one(&doc, vec![rect_only, both, circle_only]) - (200.0 + PI * 25.0 - half_disc) * depth)
            .abs()
            < tol,
        "union"
    );
    // nothing chosen: everything the shapes enclose
    assert!(
        (one(&doc, vec![]) - (200.0 + PI * 25.0 - half_disc) * depth).abs() < tol,
        "default"
    );
    // a region that does not exist is refused with a message
    let mut d = doc.clone();
    let e = extrude_with(
        &mut d,
        "E",
        s,
        depth,
        BodyOp::New,
        ExtrudeExtent {
            regions: vec![RegionPick::Legacy(vec![999])],
            ..Default::default()
        },
    );
    let ev = eval(&d);
    assert!(ev.output(e).is_none() && !ev.failures().is_empty());
}

#[test]
fn a_hole_inside_a_loop_is_a_region_too() {
    // a 20 x 20 square with a 6 x 6 square inside: the ring and the core
    let mut sk = rect_sketch(0.0, 0.0, 20.0, 20.0);
    let pts: Vec<_> = [(7.0, 7.0), (13.0, 7.0), (13.0, 13.0), (7.0, 13.0)]
        .iter()
        .map(|q| sk.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        sk.add_line(pts[i], pts[(i + 1) % 4]);
    }
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "S", xy(), sk);
    let ev = eval(&doc);
    let sp = sketch_product(&ev, s);
    let sr = sketch_regions(&sp.sketch, &sp.entities, sp.plane);
    let ring = sr
        .regions
        .iter()
        .find(|r| (r.area - (400.0 - 36.0)).abs() < 1e-6)
        .expect("the ring")
        .pick
        .clone();
    let core = sr
        .regions
        .iter()
        .find(|r| (r.area - 36.0).abs() < 1e-6)
        .expect("the inner square")
        .pick
        .clone();
    let run = |picks: Vec<RegionPick>| {
        let mut d = doc.clone();
        let e = extrude_with(
            &mut d,
            "E",
            s,
            3.0,
            BodyOp::New,
            ExtrudeExtent {
                regions: picks,
                ..Default::default()
            },
        );
        vol(&eval(&d), e)
    };
    assert!((run(vec![ring.clone()]) - 364.0 * 3.0).abs() < 1e-6);
    assert!((run(vec![core.clone()]) - 36.0 * 3.0).abs() < 1e-6);
    // both: the whole square, with no wall between ring and core
    assert!((run(vec![ring, core]) - 400.0 * 3.0).abs() < 1e-6);
    // nothing chosen: a plate with a hole
    assert!((run(vec![]) - 364.0 * 3.0).abs() < 1e-6);
}

#[test]
fn a_line_across_a_circle_gives_two_regions_that_extrude_alone_or_together() {
    let mut sk = kw_sketch::Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 5.0);
    sk.add_line_xy(-6.0, 2.0, 6.0, 2.0);
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "S", xy(), sk);
    let ev = eval(&doc);
    let sp = sketch_product(&ev, s);
    let sr = sketch_regions(&sp.sketch, &sp.entities, sp.plane);
    assert_eq!(sr.regions.len(), 2);
    // the two halves have the same boundary entities: they are told apart by position
    assert_eq!(sr.regions[0].labels, sr.regions[1].labels);
    let small = 25.0 * (2.0f64 / 5.0).acos() - 2.0 * 21.0f64.sqrt();
    let run = |picks: Vec<RegionPick>| -> (f64, usize) {
        let mut d = doc.clone();
        let e = extrude_with(
            &mut d,
            "E",
            s,
            3.0,
            BodyOp::New,
            ExtrudeExtent {
                regions: picks,
                ..Default::default()
            },
        );
        let ev = eval(&d);
        let faces = {
            let o = ev.output(e).unwrap();
            o.brep().unwrap().body_faces(o.body().unwrap()).len()
        };
        (vol(&ev, e), faces)
    };
    let (small_pick, big_pick) = {
        let mut v: Vec<_> = sr
            .regions
            .iter()
            .map(|r| (r.area, r.pick.clone()))
            .collect();
        v.sort_by(|a, b| a.0.total_cmp(&b.0));
        (v[0].1.clone(), v[1].1.clone())
    };
    assert!((run(vec![small_pick.clone()]).0 - small * 3.0).abs() < 1e-6);
    assert!((run(vec![big_pick.clone()]).0 - (25.0 * PI - small) * 3.0).abs() < 1e-6);
    // both together: the whole disc, and no wall where the line was
    let (v, faces) = run(vec![small_pick, big_pick]);
    assert!((v - 25.0 * PI * 3.0).abs() < 1e-6);
    assert_eq!(faces, 4, "two caps and the two arcs of the wall");
    // nothing chosen: the whole disc as well
    assert!((run(vec![]).0 - 25.0 * PI * 3.0).abs() < 1e-6);
}

#[test]
fn region_choices_saved_in_the_old_form_still_find_their_region() {
    use kw_feature::legacy_regions::{regions, sketch_loops};
    let mut sk = rect_sketch(0.0, 0.0, 20.0, 10.0);
    sk.add_circle_xy(20.0, 5.0, 5.0);
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "S", xy(), sk);
    let ev = eval(&doc);
    let sp = sketch_product(&ev, s);
    // what an older version saved for the circle-only region: the keys of the loops it lies inside
    let loops = sketch_loops(&sp.sketch, &sp.entities, sp.plane).unwrap();
    let regs = regions(&loops);
    let half_disc = 0.5 * PI * 25.0;
    let old = regs
        .iter()
        .find(|r| (r.area - (PI * 25.0 - half_disc)).abs() < 0.2)
        .unwrap()
        .signature
        .clone();
    let mut d = doc.clone();
    let e = extrude_with(
        &mut d,
        "E",
        s,
        4.0,
        BodyOp::New,
        ExtrudeExtent {
            regions: vec![RegionPick::Legacy(old.clone())],
            ..Default::default()
        },
    );
    assert!((vol(&eval(&d), e) - (PI * 25.0 - half_disc) * 4.0).abs() < 1e-6);
    // the old file form reads as a legacy pick, the new one as a face
    let legacy: ExtrudeExtent =
        serde_json::from_str(&format!("{{\"regions\":[{:?}]}}", old)).unwrap();
    assert_eq!(legacy.regions, vec![RegionPick::Legacy(old)]);
    let new = RegionPick::Face {
        labels: vec![1, 4],
        at: [3.5, -2.0],
    };
    let text = serde_json::to_string(&new).unwrap();
    assert_eq!(serde_json::from_str::<RegionPick>(&text).unwrap(), new);
}

#[test]
fn thin_extrusions_make_walls_of_the_right_thickness() {
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "S", xy(), rect_sketch(0.0, 0.0, 20.0, 10.0));
    let thin = |side| ExtrudeExtent {
        thin: Some(ThinWall {
            thickness: mm(2.0),
            side,
        }),
        ..Default::default()
    };
    // wall inside the 20 x 10 profile: 200 - 16 x 6 = 104 mm2; outside: 24 x 14 - 200 = 136; centred: 22 x 12 - 18 x 8 = 120
    for (side, area) in [
        (ThinSide::Inside, 104.0),
        (ThinSide::Outside, 136.0),
        (ThinSide::Centered, 120.0),
    ] {
        let mut d = doc.clone();
        let e = extrude_with(&mut d, "T", s, 6.0, BodyOp::New, thin(side));
        let ev = eval(&d);
        let v = vol(&ev, e);
        assert!(
            (v - area * 6.0).abs() < 1e-3,
            "{side:?}: {v} vs {}",
            area * 6.0
        );
        // the wall stands exactly on the sketch plane: z from 0 to 6
        let o = ev.output(e).unwrap();
        let bb = o.brep().unwrap().body_bbox(o.body().unwrap());
        assert!(
            bb.min.z.abs() < 1e-6 && (bb.max.z - 6.0).abs() < 1e-6,
            "{side:?}: {bb:?}"
        );
    }
    // a circle: inside wall of a 10 mm disc, 1.5 thick: pi (25 - 3.5^2)
    let mut sk = kw_sketch::Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    sk.add_circle(c, 5.0);
    let mut d = Document::new();
    let sc = sketch_of(&mut d, "C", xy(), sk);
    let e = extrude_with(
        &mut d,
        "Tube",
        sc,
        8.0,
        BodyOp::New,
        ExtrudeExtent {
            thin: Some(ThinWall {
                thickness: mm(1.5),
                side: ThinSide::Inside,
            }),
            ..Default::default()
        },
    );
    assert!((vol(&eval(&d), e) - PI * (25.0 - 3.5 * 3.5) * 8.0).abs() < 1e-3);
}

#[test]
fn ribs_thicken_lines_and_arcs_in_the_sketch_plane() {
    let rib = |sk: kw_sketch::Sketch, t: f64, depth: f64| -> f64 {
        let mut d = Document::new();
        let s = sketch_of(&mut d, "S", xy(), sk);
        let r = d
            .add(
                "Rib",
                FeatureSpec::Rib(RibSpec {
                    sketch: s,
                    thickness: mm(t),
                    depth: mm(depth),
                    symmetric: false,
                    flip: false,
                    op: BodyOp::New,
                    base: None,
                }),
            )
            .unwrap();
        vol(&eval(&d), r)
    };
    let mut line = kw_sketch::Sketch::new();
    line.add_line_xy(0.0, 0.0, 30.0, 0.0);
    assert!((rib(line, 2.0, 8.0) - 30.0 * 2.0 * 8.0).abs() < 1e-6);
    let mut arc = kw_sketch::Sketch::new();
    arc.add_arc_xy(0.0, 0.0, 20.0, 0.0, PI / 2.0);
    let want = PI / 4.0 * (21.0 * 21.0 - 19.0 * 19.0) * 5.0;
    assert!(
        (rib(arc, 2.0, 5.0) - want).abs() < 1e-3 * want,
        "{}",
        rib(
            {
                let mut a = kw_sketch::Sketch::new();
                a.add_arc_xy(0.0, 0.0, 20.0, 0.0, PI / 2.0);
                a
            },
            2.0,
            5.0
        )
    );
    // an L of two lines: the strips overlap in a 1 x 1 square at the corner
    let mut l = kw_sketch::Sketch::new();
    l.add_line_xy(0.0, 0.0, 20.0, 0.0);
    l.add_line_xy(20.0, 0.0, 20.0, 15.0);
    assert!((rib(l, 2.0, 8.0) - (40.0 + 30.0 - 1.0) * 8.0).abs() < 1e-4);
}

#[test]
fn extrude_up_to_a_face_and_from_an_offset() {
    let mut doc = Document::new();
    let s0 = sketch_of(
        &mut doc,
        "Block sketch",
        xy(),
        rect_sketch(0.0, 0.0, 30.0, 20.0),
    );
    let block = extrude_with(
        &mut doc,
        "Block",
        s0,
        10.0,
        BodyOp::New,
        ExtrudeExtent::default(),
    );
    let ev = eval(&doc);
    let top = {
        let o = ev.output(block).unwrap();
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
    // a 5 x 5 pillar beside the block (touching its x = 30 face), grown from z = 0 up to the block's top face
    let s1 = sketch_of(&mut doc, "Pillar", xy(), rect_sketch(30.0, 0.0, 5.0, 5.0));
    let pillar = extrude_with(
        &mut doc,
        "Pillar extrude",
        s1,
        1.0,
        BodyOp::Join,
        ExtrudeExtent {
            to_face: Some(top.clone()),
            ..Default::default()
        },
    );
    let ev = eval(&doc);
    assert!(
        (vol(&ev, pillar) - (6000.0 + 250.0)).abs() < 1e-3,
        "{}",
        vol(&ev, pillar)
    );
    // a face behind the sketch plane is refused with a message
    let s2 = sketch_of(
        &mut doc,
        "Behind",
        PlaneSpec::Xy(mm(20.0)),
        rect_sketch(0.0, 0.0, 5.0, 5.0),
    );
    let bad = extrude_with(
        &mut doc,
        "Bad",
        s2,
        1.0,
        BodyOp::Cut,
        ExtrudeExtent {
            to_face: Some(top),
            ..Default::default()
        },
    );
    let ev = eval(&doc);
    assert!(ev.output(bad).is_none());
    assert!(
        ev.failures()
            .iter()
            .any(|(_, e)| e.to_string().contains("behind")),
        "{:?}",
        ev.failures()
    );
    // a start offset: the wall stands 3 mm above the sketch plane and is 4 mm tall
    let mut d = Document::new();
    let s = sketch_of(&mut d, "S", xy(), rect_sketch(0.0, 0.0, 10.0, 10.0));
    let e = extrude_with(
        &mut d,
        "E",
        s,
        4.0,
        BodyOp::New,
        ExtrudeExtent {
            start_offset: Some(mm(3.0)),
            ..Default::default()
        },
    );
    let ev = eval(&d);
    let o = ev.output(e).unwrap();
    let bb = o.brep().unwrap().body_bbox(o.body().unwrap());
    assert!(
        (bb.min.z - 3.0).abs() < 1e-6 && (bb.max.z - 7.0).abs() < 1e-6,
        "{bb:?}"
    );
}

#[test]
fn revolve_about_a_body_edge_straight_sweep_and_pipe() {
    // a 10 x 10 x 10 block, a ring cut around its vertical edge at (0, 0)
    let mut doc = Document::new();
    let s0 = sketch_of(
        &mut doc,
        "Block sketch",
        xy(),
        rect_sketch(0.0, 0.0, 10.0, 10.0),
    );
    let block = extrude_with(
        &mut doc,
        "Block",
        s0,
        10.0,
        BodyOp::New,
        ExtrudeExtent::default(),
    );
    let ev = eval(&doc);
    let edge = {
        let o = ev.output(block).unwrap();
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
        let (fx, fy) = (face_at(0), face_at(1));
        EdgeRef {
            faces: [
                o.names.reference_to(b, fx).unwrap(),
                o.names.reference_to(b, fy).unwrap(),
            ],
        }
    };
    // profile on the XZ plane (sketch x = world Z, y = world X): rectangle z 0..3, x 4..6
    let prof = sketch_of(
        &mut doc,
        "Ring",
        PlaneSpec::Xz(mm(0.0)),
        rect_sketch(0.0, 4.0, 3.0, 2.0),
    );
    let cut = doc
        .add(
            "Revolve",
            FeatureSpec::Revolve(RevolveSpec {
                sketch: prof,
                axis: AxisSpec::Edge(Box::new(edge)),
                angle: Param::angle_deg(360.0),
                op: BodyOp::Cut,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    // a quarter of the ring (x, y >= 0) lies inside the block
    let want = 1000.0 - PI * (36.0 - 16.0) * 3.0 / 4.0;
    assert!(
        (vol(&ev, cut) - want).abs() < 1e-3 * want,
        "{} vs {want}",
        vol(&ev, cut)
    );

    // straight sweep: a disc swept 12 mm along a straight path = an extrusion
    let mut d = Document::new();
    let mut disc = kw_sketch::Sketch::new();
    let c = disc.add_point(0.0, 0.0);
    disc.add_circle(c, 3.0);
    let sp = sketch_of(&mut d, "Disc", xy(), disc);
    let mut path = kw_sketch::Sketch::new();
    path.add_line_xy(0.0, 0.0, 12.0, 0.0); // XZ plane: sketch x runs along world Z
    let spath = sketch_of(&mut d, "Path", PlaneSpec::Xz(mm(0.0)), path);
    let sw = d
        .add(
            "Sweep",
            FeatureSpec::Sweep(SweepSpec {
                profile: sp,
                path: spath,
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    assert!((vol(&eval(&d), sw) - PI * 9.0 * 12.0).abs() < 1e-6);
    // pipe along the same straight path: outer diameter 6, wall 1 -> pi (9 - 4) x 12
    let pipe = d
        .add(
            "Pipe",
            FeatureSpec::Pipe(PipeSpec {
                path: spath,
                diameter: mm(6.0),
                wall: mm(1.0),
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    assert!((vol(&eval(&d), pipe) - PI * 5.0 * 12.0).abs() < 1e-6);
    // pipe along a quarter circle of radius 20: wall area x arc length
    let mut arc = kw_sketch::Sketch::new();
    arc.add_arc_xy(0.0, 20.0, 20.0, -PI / 2.0, 0.0);
    let sarc = sketch_of(&mut d, "Arc path", PlaneSpec::Xz(mm(0.0)), arc);
    let pipe2 = d
        .add(
            "Pipe 2",
            FeatureSpec::Pipe(PipeSpec {
                path: sarc,
                diameter: mm(6.0),
                wall: mm(1.0),
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    let want = PI * 5.0 * (PI / 2.0 * 20.0);
    assert!((vol(&eval(&d), pipe2) - want).abs() < 1e-3 * want);
    // a wall at least half the diameter is a solid rod
    let rod = d
        .add(
            "Rod",
            FeatureSpec::Pipe(PipeSpec {
                path: spath,
                diameter: mm(6.0),
                wall: mm(3.0),
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    assert!((vol(&eval(&d), rod) - PI * 9.0 * 12.0).abs() < 1e-6);
}
