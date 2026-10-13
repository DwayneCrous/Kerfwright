//! Thread tables, tapped holes and cosmetic threads.

use kw_expr::Param;
use kw_feature::threads::{self, ThreadFamily};
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

fn block_with_tapped_hole(designation: &str, depth: f64, thread: f64) -> (Document, FeatureId) {
    let mut doc = Document::new();
    let s = doc
        .add_rect_sketch(
            "Sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(40.0),
            mm(40.0),
        )
        .unwrap();
    doc.add_extrude("Block", s, mm(20.0), BodyOp::New).unwrap();
    let h = doc
        .add(
            "Tapped hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(20.0)),
                points: vec![[mm(20.0), mm(20.0)]],
                kind: HoleKind::Tapped {
                    designation: designation.into(),
                    thread_length: mm(thread),
                },
                depth: mm(depth),
                through_all: false,
                base: None,
            }),
        )
        .unwrap();
    (doc, h)
}

/// The hole's wall: the one cylindrical face of the body.
fn wall(ev: &Evaluation, feature: FeatureId) -> Reference {
    let o = ev.output(feature).unwrap();
    let rep = o.brep().unwrap();
    for (name, e) in &o.names.by_name {
        if let kw_brep::EntityRef::Face(f) = e {
            if matches!(
                rep.surfaces[rep.faces[*f].surface],
                kw_nurbs::SurfaceGeom::Cylinder(_)
            ) {
                return ev.reference(feature, name).expect("reference");
            }
        }
    }
    panic!("no cylindrical face");
}

#[test]
fn the_tables_agree_with_the_standards() {
    // M6: pitch 1, tap drill 5.0, minor diameter 6 - 1.082532 = 4.917
    let m6 = threads::lookup("M6").unwrap();
    assert_eq!((m6.major, m6.pitch, m6.tap_drill), (6.0, 1.0, 5.0));
    assert!((m6.minor_internal() - 4.917468).abs() < 1e-6);
    assert!((m6.pitch_diameter() - 5.350481).abs() < 1e-6);
    // 1/4-20 UNC: 6.35 mm, pitch 1.27 mm, #7 drill 5.1054 mm
    let q = threads::lookup("1/4-20 unc").unwrap();
    assert!((q.major - 6.35).abs() < 1e-9 && (q.pitch - 1.27).abs() < 1e-9);
    assert!((q.tap_drill - 5.1054).abs() < 1e-4);
    // every tap drill lies between the minor and the major diameter, and fits its own thread
    for s in threads::SIZES {
        assert!(
            s.tap_drill > s.minor_internal() - 0.1 && s.tap_drill < s.major,
            "{}",
            s.designation
        );
        assert!(s.fits_hole(s.tap_drill), "{}", s.designation);
        assert!(s.fits_shaft(s.major), "{}", s.designation);
    }
    // names are unique and the families are populated
    let mut names: Vec<_> = threads::SIZES.iter().map(|s| s.designation).collect();
    names.sort_unstable();
    names.dedup();
    assert_eq!(names.len(), threads::SIZES.len());
    for f in [
        ThreadFamily::MetricCoarse,
        ThreadFamily::MetricFine,
        ThreadFamily::Unc,
        ThreadFamily::Unf,
    ] {
        assert!(threads::family(f).count() >= 4, "{f:?}");
    }
    assert!(threads::lookup("M7").is_none());
}

#[test]
fn a_tapped_hole_is_drilled_to_the_tap_drill_size() {
    let (doc, h) = block_with_tapped_hole("M8", 12.0, 9.0);
    let ev = eval(&doc);
    let want = 40.0 * 40.0 * 20.0 - PI * (6.8_f64 / 2.0).powi(2) * 12.0;
    assert!(
        (vol(&ev, h) - want).abs() < 1e-6 * want,
        "{} vs {want}",
        vol(&ev, h)
    );
    // the report lists one internal M8 thread of 9 mm
    let report = threads::report(&doc, &ev.scope(&doc));
    assert_eq!(report.len(), 1);
    assert_eq!(report[0].size.designation, "M8");
    assert!(report[0].internal && report[0].from_hole && (report[0].length - 9.0).abs() < 1e-9);
    assert!(report[0].text().contains("tap drill dia 6.800"));
}

#[test]
fn a_thread_longer_than_the_hole_or_of_unknown_size_is_refused() {
    let (doc, h) = block_with_tapped_hole("M8", 8.0, 9.0);
    let ev = eval(&doc);
    assert!(ev.output(h).is_none(), "thread longer than the hole");
    let (doc, h) = block_with_tapped_hole("M7", 8.0, 5.0);
    let ev = eval(&doc);
    assert!(ev.output(h).is_none(), "unknown size");
}

#[test]
fn a_cosmetic_thread_keeps_the_geometry_and_checks_the_diameter() {
    let (mut doc, h) = block_with_tapped_hole("M8", 12.0, 9.0);
    let ev = eval(&doc);
    let v0 = vol(&ev, h);
    let face = wall(&ev, h);
    let t = doc
        .add(
            "Thread",
            FeatureSpec::Thread(ThreadSpec {
                face: face.clone(),
                designation: "M8".into(),
                length: mm(9.0),
                internal: true,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!((vol(&ev, t) - v0).abs() < 1e-9 * v0);
    let report = threads::report(&doc, &ev.scope(&doc));
    assert_eq!(report.len(), 2);
    assert!(!report[1].from_hole);
    // M10 does not belong in a 6.8 mm hole
    let bad = doc
        .add(
            "Bad thread",
            FeatureSpec::Thread(ThreadSpec {
                face,
                designation: "M10".into(),
                length: mm(5.0),
                internal: true,
                base: None,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    assert!(ev.output(bad).is_none());
    // a plain block face is not cylindrical
    assert!(ev.error(bad).is_some());
}

#[test]
fn tapped_holes_and_threads_survive_save_and_load() {
    let (doc, _) = block_with_tapped_hole("1/2-13 UNC", 15.0, 12.0);
    let bytes = serde_json::to_string(
        &doc.features
            .iter()
            .map(|f| f.spec.clone())
            .collect::<Vec<_>>(),
    )
    .unwrap();
    let back: Vec<FeatureSpec> = serde_json::from_str(&bytes).unwrap();
    assert_eq!(back.last(), doc.features.last().map(|f| &f.spec));
}
