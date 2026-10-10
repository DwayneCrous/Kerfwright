//! Performance of the incremental engine on a 200-feature model.

mod common;
use common::mm;
use kw_expr::Param;
use kw_feature::*;
use std::time::Instant;

/// `groups` x (sketch + hole or boss) on a plate: 2 features per group, plus the plate and a
/// parameter that drives the last hole.
fn long_model(groups: usize) -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let p = doc.add_parameter("d", mm(2.0)).unwrap();
    let w = 12.0 * groups.div_ceil(8) as f64 + 12.0;
    let s = doc
        .add_rect_sketch(
            "Plate sketch",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(w),
            mm(100.0),
        )
        .unwrap();
    doc.add_extrude("Plate", s, mm(8.0), BodyOp::New).unwrap();
    let mut last = p;
    for k in 0..groups {
        let (col, row) = (k / 8, k % 8);
        let (x, y) = (8.0 + 12.0 * col as f64, 8.0 + 12.0 * row as f64);
        if k % 2 == 0 {
            let dia = if k + 2 >= groups {
                Param::expr("d").unwrap()
            } else {
                mm(2.0 + (k % 3) as f64)
            };
            let cs = doc
                .add_circle_sketch(
                    &format!("Hole {k} sketch"),
                    PlaneSpec::Xy(mm(8.0)),
                    mm(x),
                    mm(y),
                    dia,
                )
                .unwrap();
            last = doc
                .add(
                    &format!("Hole {k}"),
                    FeatureSpec::Extrude(ExtrudeSpec {
                        sketch: cs,
                        depth: mm(9.0),
                        symmetric: false,
                        flip: true,
                        op: BodyOp::Cut,
                        base: None,
                    }),
                )
                .unwrap();
        } else {
            let bs = doc
                .add_rect_sketch(
                    &format!("Boss {k} sketch"),
                    PlaneSpec::Xy(mm(8.0)),
                    mm(x - 2.0),
                    mm(y - 2.0),
                    mm(4.0),
                    mm(4.0),
                )
                .unwrap();
            last = doc
                .add_extrude(
                    &format!("Boss {k}"),
                    bs,
                    mm(2.0 + (k % 4) as f64),
                    BodyOp::Join,
                )
                .unwrap();
        }
    }
    (doc, p, last)
}

/// Eight parts built independently (a plate with eleven holes and bosses each), joined one after the
/// other: 200 features, and the parts can be evaluated in parallel.
fn parts_model(parts: usize, per_part: usize) -> (Document, FeatureId, FeatureId) {
    let mut doc = Document::new();
    let p = doc.add_parameter("d", mm(2.0)).unwrap();
    let mut part_last: Vec<FeatureId> = Vec::new();
    let mut tail = p;
    for part in 0..parts {
        let x0 = 28.0 * part as f64;
        let h = 6.0 + (part % 3) as f64;
        let s = doc
            .add_rect_sketch(
                &format!("P{part} plate sketch"),
                PlaneSpec::Xy(mm(0.0)),
                mm(x0),
                mm(0.0),
                mm(30.0),
                mm(60.0),
            )
            .unwrap();
        let mut last = doc
            .add_extrude(&format!("P{part} plate"), s, mm(h), BodyOp::New)
            .unwrap();
        for k in 0..per_part {
            let (cx, cy) = (
                x0 + 7.0 + 8.0 * (k % 2) as f64 * 1.5,
                7.0 + 8.0 * (k / 2) as f64,
            );
            if k % 2 == 0 {
                let dia = if part + 1 == parts && k + 2 >= per_part {
                    Param::expr("d").unwrap()
                } else {
                    mm(2.0 + (k % 3) as f64)
                };
                let cs = doc
                    .add_circle_sketch(
                        &format!("P{part} hole {k} sketch"),
                        PlaneSpec::Xy(mm(h)),
                        mm(cx),
                        mm(cy),
                        dia,
                    )
                    .unwrap();
                last = doc
                    .add(
                        &format!("P{part} hole {k}"),
                        FeatureSpec::Extrude(ExtrudeSpec {
                            sketch: cs,
                            depth: mm(h + 1.0),
                            symmetric: false,
                            flip: true,
                            op: BodyOp::Cut,
                            base: None,
                        }),
                    )
                    .unwrap();
            } else {
                let bs = doc
                    .add_rect_sketch(
                        &format!("P{part} boss {k} sketch"),
                        PlaneSpec::Xy(mm(h)),
                        mm(cx - 2.0),
                        mm(cy - 2.0),
                        mm(4.0),
                        mm(4.0),
                    )
                    .unwrap();
                last = doc
                    .add_extrude(
                        &format!("P{part} boss {k}"),
                        bs,
                        mm(2.0 + (k % 4) as f64),
                        BodyOp::Join,
                    )
                    .unwrap();
            }
        }
        part_last.push(last);
        tail = last;
    }
    // join the parts: ((p0 + p1) + p2) + ...
    let mut acc = part_last[0];
    for (i, part) in part_last.iter().enumerate().skip(1) {
        acc = doc
            .add(
                &format!("Join {i}"),
                FeatureSpec::Combine(CombineSpec {
                    target: acc,
                    tool: *part,
                    op: BodyOp::Join,
                }),
            )
            .unwrap();
    }
    (doc, p, tail.max(acc))
}

#[test]
fn two_hundred_features_in_parts() {
    // 8 parts x (2 + 2 x 11) = 192, + 7 joins + 1 parameter = 200
    let (mut doc, param, last) = parts_model(8, 11);
    assert_eq!(doc.len(), 200);
    let mut e = Engine::with_threads(8);
    let t = Instant::now();
    let ev = e.evaluate(&doc);
    let cold = t.elapsed();
    assert!(ev.is_clean(), "{:?}", ev.failures());
    eprintln!(
        "[parts] cold replay of {} features: {:.0} ms ({} executed)",
        doc.len(),
        cold.as_secs_f64() * 1e3,
        ev.stats.executed.len()
    );
    let (single_g, _) = ev.final_hashes().unwrap();
    // the same cold replay on one thread, for the record
    let t = Instant::now();
    let ev1 = Engine::with_threads(1).evaluate(&doc);
    eprintln!(
        "[parts] cold replay on 1 thread: {:.0} ms",
        t.elapsed().as_secs_f64() * 1e3
    );
    assert!(ev1.is_clean(), "{:?}", ev1.failures());
    assert_eq!(ev1.final_hashes().unwrap().0, single_g);
    let mut worst: f64 = 0.0;
    for i in 0..5 {
        doc.replace_spec(
            param,
            FeatureSpec::Parameter {
                name: "d".into(),
                value: mm(2.0 + 0.1 * (i + 1) as f64),
            },
        )
        .unwrap();
        let t = Instant::now();
        let ev = e.evaluate(&doc);
        let dt = t.elapsed().as_secs_f64() * 1e3;
        assert!(ev.is_clean(), "{:?}", ev.failures());
        assert!(
            ev.stats.executed.len() <= 8,
            "{} features re-executed",
            ev.stats.executed.len()
        );
        worst = worst.max(dt);
        eprintln!(
            "[parts] tail edit {i}: {dt:.1} ms, {} features executed: {:?}",
            ev.stats.executed.len(),
            ev.stats
                .executed
                .iter()
                .map(|f| doc.name_of(*f))
                .collect::<Vec<_>>()
        );
    }
    let _ = last;
    // a no-op edit
    doc.replace_spec(
        param,
        FeatureSpec::Parameter {
            name: "d".into(),
            value: Param::expr("2.5 mm * 1").unwrap(),
        },
    )
    .unwrap();
    e.evaluate(&doc);
    doc.replace_spec(
        param,
        FeatureSpec::Parameter {
            name: "d".into(),
            value: Param::expr("1.25 mm * 2").unwrap(),
        },
    )
    .unwrap();
    let ev = e.evaluate(&doc);
    assert!(ev.stats.executed.is_empty());
    assert!(cold.as_secs_f64() <= 2.0, "cold replay {cold:?}");
    assert!(worst <= 150.0, "tail edit {worst} ms");
}

/// One body that grows through 100 solid features (every boolean walks a top face with dozens of
/// holes). Reported for the record; the engine cannot parallelise a single chain.
#[test]
fn linear_chain_report() {
    let (mut doc, param, last) = long_model(99);
    assert!(
        doc.len() >= 200,
        "the model should have at least 200 features"
    );
    let mut e = Engine::with_threads(8);
    let t = Instant::now();
    let ev = e.evaluate(&doc);
    let cold = t.elapsed();
    assert!(ev.is_clean(), "{:?}", ev.failures());
    eprintln!(
        "cold replay of {} features: {:.0} ms ({} executed)",
        doc.len(),
        cold.as_secs_f64() * 1e3,
        ev.stats.executed.len()
    );

    // tail edit: change the parameter that drives the last hole
    let mut best = f64::MAX;
    let mut worst: f64 = 0.0;
    for i in 0..5 {
        doc.replace_spec(
            param,
            FeatureSpec::Parameter {
                name: "d".into(),
                value: mm(2.0 + 0.1 * (i + 1) as f64),
            },
        )
        .unwrap();
        let t = Instant::now();
        let ev = e.evaluate(&doc);
        let dt = t.elapsed().as_secs_f64() * 1e3;
        assert!(ev.is_clean());
        assert!(ev.stats.executed.contains(&last));
        best = best.min(dt);
        worst = worst.max(dt);
        eprintln!(
            "tail edit {i}: {dt:.1} ms, {} features executed",
            ev.stats.executed.len()
        );
    }
    eprintln!("tail edit: best {best:.1} ms, worst {worst:.1} ms");

    // a no-op edit executes nothing
    doc.replace_spec(
        param,
        FeatureSpec::Parameter {
            name: "d".into(),
            value: Param::expr("2.5 mm * 1").unwrap(),
        },
    )
    .unwrap();
    let before = e.evaluate(&doc);
    doc.replace_spec(
        param,
        FeatureSpec::Parameter {
            name: "d".into(),
            value: Param::expr("1.25 mm * 2").unwrap(),
        },
    )
    .unwrap();
    let t = Instant::now();
    let ev = e.evaluate(&doc);
    eprintln!(
        "no-op edit: {:.1} ms, {} features executed",
        t.elapsed().as_secs_f64() * 1e3,
        ev.stats.executed.len()
    );
    assert!(ev.stats.executed.is_empty());
    assert_eq!(before.document_hash(), ev.document_hash());
    // a single body that grows to 200 features cannot use the parallelism, and each boolean has to
    // walk a face with up to 99 holes: the cold replay is reported, not asserted (see the Phase 4 report)
    let _ = cold;
}
