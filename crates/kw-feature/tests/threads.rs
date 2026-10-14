//! Thread tables, tapped holes and cosmetic threads.

use kw_expr::Param;
use kw_feature::threads::{self, ThreadFamily};
use kw_feature::*;
use std::f64::consts::PI;

fn mm(v: f64) -> Param {
    Param::length(v)
}
fn eval(doc: &Document) -> Evaluation {
    let t0 = std::time::Instant::now();
    let ev = Engine::with_threads(2).evaluate(doc);
    eprintln!("   (evaluation: {:.2} s)", t0.elapsed().as_secs_f64());
    ev
}
fn vol(ev: &Evaluation, id: FeatureId) -> f64 {
    let t0 = std::time::Instant::now();
    let r = vol_inner(ev, id);
    eprintln!(
        "   (volume and validation: {:.2} s)",
        t0.elapsed().as_secs_f64()
    );
    r
}
fn vol_inner(ev: &Evaluation, id: FeatureId) -> f64 {
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
                    modelled: false,
                    right_hand: true,
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
                modelled: false,
                right_hand: true,
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
                modelled: false,
                right_hand: true,
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

// ---------------------------------------------------------------------------------------- modelled threads

use kw_feature::thread_geom::ThreadSection;

/// The internal-thread cavity of `size` as the tool makes it.
fn cavity(size: &threads::ThreadSize) -> ThreadSection {
    ThreadSection::new(
        size.major / 2.0,
        size.minor_internal() / 2.0 - 0.02,
        size.pitch,
    )
    .unwrap()
}

fn mesh_ok(ev: &Evaluation, id: FeatureId) {
    let o = ev.output(id).unwrap();
    let rep = o.brep().unwrap();
    let b = o.body().unwrap();
    let t = kw_tess::tessellate_body(
        rep,
        b,
        &kw_tess::TessOptions {
            chord_tol: 0.003,
            angle_tol: 0.15,
        },
    )
    .expect("tessellates");
    let v = rep.volume(b).unwrap();
    assert!(
        (t.mesh.signed_volume() - v).abs() < 0.02 * v,
        "mesh {} vs {v}",
        t.mesh.signed_volume()
    );
}

fn set_modelled(doc: &mut Document, id: FeatureId, modelled: bool, right_hand: bool) {
    let mut spec = doc
        .features
        .iter()
        .find(|f| f.id == id)
        .unwrap()
        .spec
        .clone();
    match &mut spec {
        FeatureSpec::Hole(h) => {
            if let HoleKind::Tapped {
                modelled: m,
                right_hand: r,
                ..
            } = &mut h.kind
            {
                *m = modelled;
                *r = right_hand;
            }
        }
        FeatureSpec::Thread(t) => {
            t.modelled = modelled;
            t.right_hand = right_hand;
        }
        _ => {}
    }
    doc.replace_spec(id, spec).unwrap();
}

#[test]
fn a_modelled_tapped_hole_removes_the_volume_of_its_thread_and_its_bore() {
    let size = threads::lookup("M8").unwrap();
    let sec = cavity(size);
    // 12 mm deep, 9 mm of it threaded; and a hole threaded all the way down
    for (depth, thread) in [(12.0, 9.0), (8.0, 8.0)] {
        let (mut doc, h) = block_with_tapped_hole("M8", depth, thread);
        let plain = vol(&eval(&doc), h);
        set_modelled(&mut doc, h, true, true);
        let ev = eval(&doc);
        let v = vol(&ev, h);
        let block = 40.0 * 40.0 * 20.0;
        let r_bore = sec.small - 0.01;
        let removed = sec.area() * thread + PI * r_bore * r_bore * (depth - thread);
        assert!(
            (block - v - removed).abs() < 2e-4 * removed,
            "depth {depth}, thread {thread}: removed {} vs {removed}",
            block - v
        );
        // more material is gone than from the plain tap-drill hole of the same depth only when it has a thread:
        // the drilled hole is narrower than the tap drill, the groove reaches past it
        assert_ne!(v, plain);
        mesh_ok(&ev, h);
    }
}

#[test]
fn a_modelled_thread_goes_into_a_drilled_hole_and_onto_a_pin() {
    // an M8 thread in a 6.8 mm hole
    let size = threads::lookup("M8").unwrap();
    let (mut doc, h) = block_with_tapped_hole("M8", 12.0, 9.0);
    let ev = eval(&doc);
    let v0 = vol(&ev, h);
    let face = wall(&ev, h);
    let t = doc
        .add(
            "Thread",
            FeatureSpec::Thread(ThreadSpec {
                face,
                designation: "M8".into(),
                length: mm(9.0),
                internal: true,
                base: None,
                modelled: true,
                right_hand: true,
            }),
        )
        .unwrap();
    let ev = eval(&doc);
    let v = vol(&ev, t);
    // the groove is the cavity beyond the existing hole
    let sec = cavity(size);
    let r_h = size.tap_drill / 2.0;
    let area = |f: &dyn Fn(f64) -> f64| {
        let n = 100_000;
        0.5 * 2.0 * PI / n as f64
            * (0..n)
                .map(|k| f(2.0 * PI * (k as f64 + 0.5) / n as f64).powi(2))
                .sum::<f64>()
    };
    let groove = area(&|a| sec.radius(a).max(r_h)) - PI * r_h * r_h;
    assert!(
        (v0 - v - groove * 9.0).abs() < 3e-4 * groove * 9.0,
        "removed {} vs {}",
        v0 - v,
        groove * 9.0
    );
    mesh_ok(&ev, t);

    // an M8 pin 20 mm long, threaded for 12 mm from its free end
    let mut pin = Document::new();
    let c = pin
        .add_circle_sketch(
            "Pin sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(8.0),
        )
        .unwrap();
    let body = pin.add_extrude("Pin", c, mm(20.0), BodyOp::New).unwrap();
    let ev = eval(&pin);
    let pin_v = vol(&ev, body);
    let face = wall(&ev, body);
    let t = pin
        .add(
            "Thread",
            FeatureSpec::Thread(ThreadSpec {
                face,
                designation: "M8".into(),
                length: mm(12.0),
                internal: false,
                base: None,
                modelled: true,
                right_hand: true,
            }),
        )
        .unwrap();
    let ev = eval(&pin);
    let v = vol(&ev, t);
    let ext = ThreadSection::new(4.02, size.minor_external() / 2.0, size.pitch).unwrap();
    let clipped = area(&|a| ext.radius(a).min(4.0));
    let removed = (PI * 16.0 - clipped) * 12.0;
    assert!(
        (pin_v - v - removed).abs() < 3e-4 * removed,
        "removed {} vs {removed}",
        pin_v - v
    );
    mesh_ok(&ev, t);
}

#[test]
fn a_left_hand_thread_turns_the_other_way_and_old_files_stay_cosmetic() {
    let (mut doc, h) = block_with_tapped_hole("M6", 10.0, 8.0);
    set_modelled(&mut doc, h, true, true);
    let right = eval(&doc);
    set_modelled(&mut doc, h, true, false);
    let left = eval(&doc);
    let (vr, vl) = (vol(&right, h), vol(&left, h));
    assert!((vr - vl).abs() < 1e-6 * vr, "{vr} vs {vl}");
    // a ring of points half way between the root and the crest of the thread is in the right-hand body where
    // the left-hand one has its groove, at some heights
    let size = threads::lookup("M6").unwrap();
    let r = 0.5 * (size.major / 2.0 + size.minor_internal() / 2.0);
    let class = |ev: &Evaluation, a: f64, z: f64| {
        let o = ev.output(h).unwrap();
        kw_brep::classify_point(
            o.brep().unwrap(),
            o.body().unwrap(),
            &kw_math::Point3::new(20.0 + r * a.cos(), 20.0 + r * a.sin(), z),
            1e-7,
        )
        .unwrap()
    };
    let mut differ = 0;
    let n = 48;
    for k in 0..n {
        let a = 2.0 * PI * k as f64 / n as f64;
        if class(&right, a, 17.3) != class(&left, a, 17.3) {
            differ += 1;
        }
    }
    assert!(differ > n / 4, "only {differ} of {n} points differ");
    // a file written before threads could be modelled has neither field
    let spec = doc
        .features
        .iter()
        .find(|f| f.id == h)
        .unwrap()
        .spec
        .clone();
    let mut v = serde_json::to_value(&spec).unwrap();
    let kind = v["style"].as_object_mut().expect("the hole style");
    kind.get_mut("Tapped")
        .and_then(|t| t.as_object_mut())
        .map(|t| {
            t.remove("modelled");
            t.remove("right_hand");
        })
        .expect("a tapped style");
    let old: FeatureSpec = serde_json::from_value(v).unwrap();
    match old {
        FeatureSpec::Hole(h) => match h.kind {
            HoleKind::Tapped {
                modelled,
                right_hand,
                ..
            } => assert!(!modelled && right_hand),
            _ => panic!(),
        },
        _ => panic!(),
    }
}
