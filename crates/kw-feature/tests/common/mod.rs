//! Documents shared by the file-format and determinism tests.
#![allow(dead_code)]

use kw_expr::Param;
use kw_feature::*;

pub fn mm(v: f64) -> Param {
    Param::length(v)
}

/// A model that uses most of what the file has to carry: parameters with expressions, sketches with
/// bindings, boolean features, patterns, a datum plane, and fillets with persistent references.
pub fn sample_document() -> Document {
    let mut doc = Document::new();
    let w = doc.add_parameter("w", mm(40.0)).unwrap();
    let _ = w;
    let s = doc
        .add_rect_sketch(
            "Base sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            Param::expr("w").unwrap(),
            Param::expr("w * 0.6").unwrap(),
        )
        .unwrap();
    let base = doc
        .add_extrude("Base", s, Param::expr("w / 4").unwrap(), BodyOp::New)
        .unwrap();
    let bs = doc
        .add_rect_sketch(
            "Boss sketch",
            PlaneSpec::Xy(Param::expr("w / 4").unwrap()),
            mm(3.0),
            mm(3.0),
            mm(3.0),
            mm(4.0),
        )
        .unwrap();
    let boss = doc.add_extrude("Boss", bs, mm(4.0), BodyOp::Join).unwrap();
    doc.add_linear_pattern(
        "Boss pattern",
        boss,
        [mm(8.0), mm(0.0), mm(0.0)],
        Param::scalar(3.0),
    )
    .unwrap();
    let hs = doc
        .add_circle_sketch(
            "Hole sketch",
            PlaneSpec::Xy(Param::expr("w / 4").unwrap()),
            mm(10.0),
            mm(18.0),
            mm(4.0),
        )
        .unwrap();
    doc.add(
        "Hole",
        FeatureSpec::Extrude(ExtrudeSpec {
            surface: false,
            sketch: hs,
            depth: Param::expr("w / 4 + 1 mm").unwrap(),
            symmetric: false,
            flip: true,
            op: BodyOp::Cut,
            base: None,
            extent: Default::default(),
        }),
    )
    .unwrap();
    doc.add(
        "Datum",
        FeatureSpec::DatumPlane(DatumPlaneSpec {
            base: PlaneSpec::Xz(mm(5.0)),
            offset: mm(2.0),
            tilt: Param::angle_deg(0.0),
            mode: Default::default(),
        }),
    )
    .unwrap();
    // fillet two base edges, selected through references made on the evaluated model
    let ev = Engine::with_threads(1).evaluate(&doc);
    let out = ev.output(
        doc.features
            .iter()
            .rev()
            .find(|f| f.spec.produces_solid())
            .unwrap()
            .id,
    );
    let out = out.unwrap_or_else(|| panic!("sample evaluates: {:?}", ev.failures()));
    let names = &out.names.by_name;
    let side: Vec<TopoName> = names
        .keys()
        .filter(|n| n.feature == base && matches!(n.origin, NameOrigin::SketchEdge { .. }))
        .cloned()
        .collect();
    let last = doc
        .features
        .iter()
        .rev()
        .find(|f| f.spec.produces_solid())
        .unwrap()
        .id;
    let rep = out.brep().unwrap();
    'p: for i in 0..side.len() {
        for j in i + 1..side.len() {
            let (a, b) = (
                out.names
                    .reference_to(rep, out.names.face(&side[i]).unwrap())
                    .unwrap(),
                out.names
                    .reference_to(rep, out.names.face(&side[j]).unwrap())
                    .unwrap(),
            );
            let mut d2 = doc.clone();
            let f = d2.add_fillet("Fillet", vec![[a, b]], mm(1.0)).unwrap();
            if Engine::with_threads(1).evaluate(&d2).output(f).is_some() {
                doc = d2;
                break 'p;
            }
        }
    }
    let _ = last;
    doc
}

/// Many independent chains joined by booleans: lots of parallel work and many names.
pub fn parallel_document(chains: usize) -> Document {
    let mut doc = Document::new();
    let s = doc
        .add_rect_sketch(
            "Plate sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(10.0 * chains as f64 + 10.0),
            mm(30.0),
        )
        .unwrap();
    doc.add_extrude("Plate", s, mm(6.0), BodyOp::New).unwrap();
    for k in 0..chains {
        let x = 5.0 + 10.0 * k as f64;
        let cs = doc
            .add_circle_sketch(
                &format!("Hole {k} sketch"),
                PlaneSpec::Xy(mm(6.0)),
                mm(x),
                mm(8.0),
                mm(3.0 + (k % 3) as f64),
            )
            .unwrap();
        doc.add(
            &format!("Hole {k}"),
            FeatureSpec::Extrude(ExtrudeSpec {
                surface: false,
                sketch: cs,
                depth: mm(7.0),
                symmetric: false,
                flip: true,
                op: BodyOp::Cut,
                base: None,
                extent: Default::default(),
            }),
        )
        .unwrap();
        let bs = doc
            .add_rect_sketch(
                &format!("Boss {k} sketch"),
                PlaneSpec::Xy(mm(6.0)),
                mm(x - 2.0),
                mm(18.0),
                mm(4.0),
                mm(4.0),
            )
            .unwrap();
        doc.add_extrude(
            &format!("Boss {k}"),
            bs,
            mm(2.0 + (k % 4) as f64),
            BodyOp::Join,
        )
        .unwrap();
    }
    doc
}
