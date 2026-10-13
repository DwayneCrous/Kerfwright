use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::*;

const BRACKET: &str = r#"
    let d = new_doc();
    let w = d.param("width", "40 mm");
    let s = d.rect_sketch("Base sketch", "xy", 0, 0, "width", "width / 2");
    let base = d.extrude("Base", s, "10 mm", "new");
    let hs = d.circle_sketch("Hole sketch", "xy:10", 20, 10, 6);
    d.extrude("Hole", hs, 12, "cut_down");
    let bs = d.rect_sketch("Boss sketch", "xy:10", 4, 4, 5, 5);
    let boss = d.extrude("Boss", bs, 3, "join");
    d.linear_pattern("Boss pattern", boss, 8, 0, 0, 3);
    d
"#;

fn mm(v: f64) -> Param {
    Param::length(v)
}

fn rust_bracket() -> Document {
    let mut d = Document::new();
    d.add_parameter("width", Param::expr("40 mm").unwrap())
        .unwrap();
    let s = d
        .add_rect_sketch(
            "Base sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            Param::expr("width").unwrap(),
            Param::expr("width / 2").unwrap(),
        )
        .unwrap();
    d.add_extrude("Base", s, Param::expr("10 mm").unwrap(), BodyOp::New)
        .unwrap();
    let hs = d
        .add_circle_sketch(
            "Hole sketch",
            PlaneSpec::Xy(mm(10.0)),
            mm(20.0),
            mm(10.0),
            mm(6.0),
        )
        .unwrap();
    d.add(
        "Hole",
        FeatureSpec::Extrude(ExtrudeSpec {
            surface: false,
            sketch: hs,
            depth: mm(12.0),
            symmetric: false,
            flip: true,
            op: BodyOp::Cut,
            base: None,
            extent: Default::default(),
        }),
    )
    .unwrap();
    let bs = d
        .add_rect_sketch(
            "Boss sketch",
            PlaneSpec::Xy(mm(10.0)),
            mm(4.0),
            mm(4.0),
            mm(5.0),
            mm(5.0),
        )
        .unwrap();
    let boss = d.add_extrude("Boss", bs, mm(3.0), BodyOp::Join).unwrap();
    d.add_linear_pattern(
        "Boss pattern",
        boss,
        [mm(8.0), mm(0.0), mm(0.0)],
        Param::scalar(3.0),
    )
    .unwrap();
    d
}

#[test]
fn a_scripted_document_is_the_document_the_builder_makes() {
    let scripted = kw_script::run(BRACKET).unwrap_or_else(|e| panic!("{e}"));
    let built = rust_bracket();
    assert_eq!(scripted, built);
    assert_eq!(
        KwfFile::new(scripted).to_bytes().unwrap(),
        KwfFile::new(built).to_bytes().unwrap(),
        "byte for byte"
    );
}

#[test]
fn scripts_evaluate_edit_and_reference_faces() {
    let doc = kw_script::run(
        r#"
        let d = new_doc();
        let s = d.rect_sketch("S", "xy", 0, 0, 30, 20);
        let e = d.extrude("E", s, 10, "new");
        let r = d.evaluate();
        if !r.ok { throw "base failed"; }
        let before = d.volume(e);
        // fillet the edge between two swept faces
        let f = d.fillet("F", [[d.side_ref(e, 0), d.side_ref(e, 1)]], 2);
        let r = d.evaluate();
        if !r.ok { throw r.failures; }
        if d.volume(f) >= before { throw "the fillet should remove material"; }
        let p = d.param("depth", 4);
        d.set_param(p, "6 mm");
        d
    "#,
    )
    .unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(doc.len(), 4);
    assert!(Engine::with_threads(1).evaluate(&doc).is_clean());
}

#[test]
fn sheet_metal_and_subd_from_a_script() {
    let doc = kw_script::run(
        r#"
        let d = new_doc();
        let t = d.param("t", "2 mm");
        d.sheet("Bracket", "t", 0.44, "t * 1.5", 30, ["F40", "B90", "F25"]);
        d.subd("Pillow", "cube", 20, 0.01);
        d
    "#,
    )
    .unwrap_or_else(|e| panic!("{e}"));
    let ev = Engine::with_threads(2).evaluate(&doc);
    assert!(ev.is_clean(), "{:?}", ev.failures());
    let sheet = doc.find_by_name("Bracket").unwrap();
    assert!(ev.flat_pattern(&doc, sheet).unwrap().length() > 65.0);
}

#[test]
fn script_errors_are_explained_and_scripts_are_contained() {
    let e = kw_script::run(r#"let d = new_doc(); d.rect_sketch("S", "ab", 0, 0, 1, 1); d"#)
        .unwrap_err();
    assert!(e.to_string().contains("unknown plane"), "{e}");
    // a timeline error comes through with its explanation
    let e = kw_script::run(
        r#"
        let d = new_doc();
        let p = d.param("a", "1 mm");
        d.set_param(p, "a + 1 mm");
        d
    "#,
    )
    .unwrap_err();
    assert!(e.to_string().contains("cycle"), "{e}");
    // runaway scripts are stopped
    let e = kw_script::run("let n = 0; loop { n += 1; }").unwrap_err();
    assert!(e.to_string().to_lowercase().contains("operations"), "{e}");
    // there is no way to reach the file system or run programs
    assert!(kw_script::run(r#"open("/etc/passwd")"#).is_err());
    assert!(kw_script::run(r#"import "x" as y; 1"#).is_err());
    // and a script that forgets the document says so
    assert!(matches!(
        kw_script::run("1 + 1"),
        Err(kw_script::ScriptError::NoDocument)
    ));
}
