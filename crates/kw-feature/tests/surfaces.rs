//! Surface features: extrude, revolve, loft and sweep with `surface` set make sheet bodies.
use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::*;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn mm(v: f64) -> Param {
    Param::length(v)
}
fn eval(doc: &Document) -> Evaluation {
    Engine::with_threads(2).evaluate(doc)
}
fn xy(z: f64) -> PlaneSpec {
    PlaneSpec::Xy(mm(z))
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
fn line_sketch(x0: f64, y0: f64, x1: f64, y1: f64) -> kw_sketch::Sketch {
    let mut sk = kw_sketch::Sketch::new();
    sk.add_line_xy(x0, y0, x1, y1);
    sk
}
fn extrude_surface(doc: &mut Document, s: FeatureId, depth: f64) -> FeatureId {
    doc.add(
        "Surface",
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
/// A valid surface of the feature, with its area.
fn area(ev: &Evaluation, id: FeatureId) -> f64 {
    let o = ev
        .output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    assert!(o.is_sheet(), "not a surface");
    let rep = o.brep().unwrap();
    assert!(rep.validate().is_ok(), "{:?}", rep.validate().errors);
    rep.area(o.body().unwrap()).unwrap()
}

#[test]
fn an_extruded_open_curve_is_a_surface_with_the_exact_area() {
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "Curve", xy(0.0), line_sketch(0.0, 0.0, 6.0, 0.0));
    let f = extrude_surface(&mut doc, s, 4.0);
    let ev = eval(&doc);
    assert!((area(&ev, f) - 24.0).abs() < 1e-9);
    // a surface is not a solid: nothing downstream sees it as the model's solid
    assert!(ev.final_solid().is_none());
    assert_eq!(ev.final_body().map(|(id, _)| *id), Some(f));
}

#[test]
fn a_closed_sketch_gives_an_open_tube() {
    let mut doc = Document::new();
    let mut sk = kw_sketch::Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 2.0);
    let s = sketch_of(&mut doc, "Circle", xy(0.0), sk);
    let f = extrude_surface(&mut doc, s, 5.0);
    assert!((area(&eval(&doc), f) - TAU * 2.0 * 5.0).abs() < 1e-6);
}

#[test]
fn a_surface_cannot_be_joined_to_or_cut_from_a_solid() {
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "Curve", xy(0.0), line_sketch(0.0, 0.0, 6.0, 0.0));
    let f = doc
        .add(
            "Surface",
            FeatureSpec::Extrude(ExtrudeSpec {
                surface: true,
                sketch: s,
                depth: mm(3.0),
                symmetric: false,
                flip: false,
                op: BodyOp::Join,
                base: None,
                extent: Default::default(),
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(f).is_none());
    let why = format!("{:?}", ev.failures());
    assert!(why.contains("body of its own"), "{why}");
}

#[test]
fn a_surface_extrude_refuses_the_solid_only_extents() {
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "Curve", xy(0.0), line_sketch(0.0, 0.0, 6.0, 0.0));
    let f = doc
        .add(
            "Surface",
            FeatureSpec::Extrude(ExtrudeSpec {
                surface: true,
                sketch: s,
                depth: mm(3.0),
                symmetric: false,
                flip: false,
                op: BodyOp::New,
                base: None,
                extent: ExtrudeExtent {
                    taper: Some(Param::angle_deg(5.0)),
                    ..Default::default()
                },
            }),
        )
        .unwrap();
    assert!(eval(&doc).output(f).is_none());
}

#[test]
fn a_revolved_line_is_a_cone_surface() {
    let mut doc = Document::new();
    // the XZ plane: sketch x runs along world X, sketch y along world Z
    let cone = sketch_of(
        &mut doc,
        "Line",
        PlaneSpec::Xz(mm(0.0)),
        line_sketch(1.0, 0.0, 3.0, 2.0),
    );
    let f = doc
        .add(
            "Revolve",
            FeatureSpec::Revolve(RevolveSpec {
                surface: true,
                sketch: cone,
                axis: AxisSpec::SketchY,
                angle: Param::angle_deg(360.0),
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    let want = PI * (1.0 + 3.0) * 8.0f64.sqrt();
    assert!((area(&eval(&doc), f) - want).abs() < 1e-6 * want);
}

#[test]
fn a_lofted_surface_through_two_sketches_is_a_frustum() {
    let mut doc = Document::new();
    let mut a = kw_sketch::Sketch::new();
    a.add_circle_xy(0.0, 0.0, 1.0);
    let mut b = kw_sketch::Sketch::new();
    b.add_circle_xy(0.0, 0.0, 2.0);
    let sa = sketch_of(&mut doc, "A", xy(0.0), a);
    let sb = sketch_of(&mut doc, "B", xy(3.0), b);
    let f = doc
        .add(
            "Loft",
            FeatureSpec::Loft(LoftSpec {
                surface: true,
                ruled: true,
                sketches: vec![sa, sb],
                op: BodyOp::New,
                base: None,
            }),
        )
        .unwrap();
    let want = PI * 3.0 * 10.0f64.sqrt();
    assert!((area(&eval(&doc), f) - want).abs() < 1e-4 * want);
}

fn sweep_surface(
    doc: &mut Document,
    profile: FeatureId,
    path: FeatureId,
    twist: Option<Param>,
    guide: Option<FeatureId>,
) -> FeatureId {
    doc.add(
        "Sweep",
        FeatureSpec::Sweep(SweepSpec {
            surface: true,
            twist,
            guide,
            profile,
            path,
            op: BodyOp::New,
            base: None,
        }),
    )
    .unwrap()
}

#[test]
fn a_twisted_sweep_of_a_line_along_a_straight_path_is_a_helicoid() {
    let (w, h, theta) = (1.5f64, 4.0f64, FRAC_PI_2);
    let mut doc = Document::new();
    let prof = sketch_of(&mut doc, "Profile", xy(0.0), line_sketch(0.0, 0.0, w, 0.0));
    // the XZ plane: sketch x runs along world Z, so this is a straight path up the z axis
    let path = sketch_of(
        &mut doc,
        "Path",
        PlaneSpec::Xz(mm(0.0)),
        line_sketch(0.0, 0.0, h, 0.0),
    );
    let f = sweep_surface(&mut doc, prof, path, Some(Param::angle_deg(90.0)), None);
    let k = theta / h;
    let want = h * 0.5 * (w * (1.0 + k * k * w * w).sqrt() + (k * w).asinh() / k);
    let got = area(&eval(&doc), f);
    assert!((got - want).abs() < 2e-4 * want, "{got} vs {want}");
}

#[test]
fn a_guide_rail_widens_a_swept_surface() {
    let h = 4.0;
    let mut doc = Document::new();
    let prof = sketch_of(
        &mut doc,
        "Profile",
        xy(0.0),
        line_sketch(0.0, 0.0, 1.0, 0.0),
    );
    let path = sketch_of(
        &mut doc,
        "Path",
        PlaneSpec::Xz(mm(0.0)),
        line_sketch(0.0, 0.0, h, 0.0),
    );
    // in the XZ plane (sketch x = world Z, sketch y = world X or its negative): out from 1 to 3
    let ev0 = {
        let g = sketch_of(
            &mut doc,
            "Guide",
            PlaneSpec::Xz(mm(0.0)),
            line_sketch(0.0, 1.0, h, 3.0),
        );
        let f = sweep_surface(&mut doc, prof, path, None, Some(g));
        eval(&doc).output(f).map(|_| f).ok_or(g)
    };
    let f = ev0.unwrap_or_else(|_| panic!("{:?}", eval(&doc).failures()));
    // a trapezoid with widths 1 and 3: area 2 h
    assert!((area(&eval(&doc), f) - 2.0 * h).abs() < 1e-6 * h);
}

#[test]
fn the_names_of_a_surface_survive_a_parameter_edit() {
    let build = |depth: f64| {
        let mut doc = Document::new();
        let mut sk = kw_sketch::Sketch::new();
        let pts: Vec<_> = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)]
            .iter()
            .map(|p| sk.add_point(p.0, p.1))
            .collect();
        sk.add_line(pts[0], pts[1]);
        sk.add_line(pts[1], pts[2]);
        let s = sketch_of(&mut doc, "Bend", xy(0.0), sk);
        let f = extrude_surface(&mut doc, s, depth);
        let ev = eval(&doc);
        let names: std::collections::BTreeSet<String> = ev
            .output(f)
            .unwrap()
            .names
            .by_name
            .keys()
            .map(|n| format!("{n:?}"))
            .collect();
        names
    };
    let (a, b) = (build(2.0), build(7.0));
    assert!(!a.is_empty());
    assert_eq!(a, b);
}

#[test]
fn surface_features_round_trip_through_a_kwf_file_and_old_files_still_load() {
    let mut doc = Document::new();
    let s = sketch_of(&mut doc, "Curve", xy(0.0), line_sketch(0.0, 0.0, 6.0, 0.0));
    let f = extrude_surface(&mut doc, s, 4.0);
    let bytes = KwfFile::new(doc.clone()).to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap().document;
    assert_eq!(back, doc);
    assert!((area(&eval(&back), f) - 24.0).abs() < 1e-9);
    // a spec written before surfaces existed has none of the new fields
    let Some(Feature {
        spec: FeatureSpec::Extrude(spec),
        ..
    }) = doc.features.iter().find(|x| x.id == f).cloned()
    else {
        panic!("no extrude");
    };
    let mut v = serde_json::to_value(&spec).unwrap();
    assert!(v.as_object_mut().unwrap().remove("surface").is_some());
    let old: ExtrudeSpec = serde_json::from_value(v).unwrap();
    assert!(!old.surface);
}
