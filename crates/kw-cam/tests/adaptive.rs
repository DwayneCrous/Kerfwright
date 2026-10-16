#![cfg(feature = "adaptive-clearing")]
#![allow(clippy::needless_range_loop)]
//! Adaptive clearing, checked by the independent verifier.

use kw_cam::adaptive::{adaptive, AdaptiveParams, EXCLUDE_BEGIN, EXCLUDE_END, LEVEL};
use kw_cam::ops2d::Cut;
use kw_cam::verify::{verify, VerifyParams};
use kw_cam::{Region, Tool};

#[test]
fn rectangle_engagement_and_gouge() {
    let region = Region::rect(0.0, 0.0, 40.0, 30.0);
    let tool = Tool::flat(1, 6.0, 20.0);
    let mut cut = Cut::new(0.0, -3.0);
    cut.stepover = 1.2;
    cut.step_down = 3.0;
    for (cell, step) in [(0.06, 0.1)] {
        let par = AdaptiveParams {
            cell,
            step,
            simplify: 0.0,
            ..AdaptiveParams::default()
        };
        let t = std::time::Instant::now();
        let (tp, pl) = adaptive(&region, &tool, &cut, &par);
        let vp = VerifyParams::new(3.0, 0.0, pl.target);
        let rep = verify(&region, &tp.events, &vp, LEVEL, EXCLUDE_BEGIN, EXCLUDE_END);
        for x in rep.top.iter().take(6) {
            eprintln!(
                "     top: {:.1}° at {:.2?} move {} sample {}",
                x.0, x.1, x.2, x.3
            );
        }
        for k in rep.worst_event.saturating_sub(4)..(rep.worst_event + 3).min(tp.events.len()) {
            eprintln!("     event {k}: {:?}", tp.events[k]);
        }
        eprintln!(
            "   worst move: {:.3?}\n   previous move: {:.3?}",
            rep.worst_move, rep.prev_move
        );
        eprintln!("cell {cell} step {step}: max ratio {:.3} at {:.1?}, mean {:.1}°, excluded {:.1} %, coverage {:.4}, {:.1} s", rep.max_engagement_ratio, rep.worst_at, rep.mean_engagement_deg, 100.0*rep.excluded_fraction, rep.coverage, t.elapsed().as_secs_f64());
    }
    let (tp, pl) = adaptive(&region, &tool, &cut, &AdaptiveParams::default());
    let vp = VerifyParams::new(3.0, 0.0, pl.target);
    let rep = verify(&region, &tp.events, &vp, LEVEL, EXCLUDE_BEGIN, EXCLUDE_END);
    eprintln!("{rep:#?}");
    assert!(
        rep.max_engagement_ratio <= 1.05,
        "{}",
        rep.max_engagement_ratio
    );
    assert!(rep.gouge < 1e-3);
}

#[test]
fn the_fifty_pocket_suite() {
    use rayon::prelude::*;
    let suite = kw_cam::suite::pocket_suite(20260101);
    assert_eq!(suite.len(), 50);
    let results: Vec<_> = suite
        .par_iter()
        .map(|c| {
            let tool = Tool::flat(1, c.tool_diameter, 25.0);
            let mut cut = Cut::new(0.0, -3.0);
            cut.stepover = c.stepover;
            cut.step_down = 3.0;
            let t = std::time::Instant::now();
            let (tp, pl) = adaptive(&c.region, &tool, &cut, &AdaptiveParams::default());
            let vp = VerifyParams::new(tool.radius(), 0.0, pl.target);
            let rep = verify(
                &c.region,
                &tp.events,
                &vp,
                LEVEL,
                EXCLUDE_BEGIN,
                EXCLUDE_END,
            );
            (
                c.clone(),
                pl.target.to_degrees(),
                rep,
                pl.passes.len(),
                t.elapsed().as_secs_f64(),
            )
        })
        .collect();
    for (c, tg, r, p, t) in &results {
        eprintln!(
            "{:28} {:8} target {tg:5.1}° max {:5.1}° ratio {:.3} mean {:4.1}° excl {:4.1} % cover {:.3} gouge {:.4} passes {p:3} {t:5.1} s  [{}]",
            c.name, c.family, r.max_engagement_deg, r.max_engagement_ratio, r.mean_engagement_deg, 100.0 * r.excluded_fraction, r.coverage, r.gouge, c.params
        );
    }
    let worst = results
        .iter()
        .map(|r| r.2.max_engagement_ratio)
        .fold(0.0, f64::max);
    let gouge = results.iter().map(|r| r.2.gouge).fold(0.0, f64::max);
    let cover_min = results.iter().map(|r| r.2.coverage).fold(1.0, f64::min);
    let excl = results.iter().map(|r| r.2.excluded_fraction).sum::<f64>() / results.len() as f64;
    eprintln!("suite: worst engagement {worst:.3} × target, worst gouge {gouge:.4} mm, minimum coverage {cover_min:.3}, mean excluded fraction {:.1} %", 100.0 * excl);
    // the criteria: never above the target by more than 5 %, no gouging (a micrometre is the resolution)
    assert!(worst <= 1.05, "worst engagement {worst}");
    assert!(gouge <= 1e-3, "gouge {gouge} mm");
    assert!(cover_min > 0.9, "coverage {cover_min}");
}

#[test]
#[ignore]
fn debug_case() {
    let idx: usize = std::env::var("KW_CASE")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(38);
    let suite = kw_cam::suite::pocket_suite(20260101);
    let c = &suite[idx];
    eprintln!("{}: {}", c.name, c.params);
    let tool = Tool::flat(1, c.tool_diameter, 25.0);
    let mut cut = Cut::new(0.0, -3.0);
    cut.stepover = c.stepover;
    cut.step_down = 3.0;
    let (tp, pl) = adaptive(&c.region, &tool, &cut, &AdaptiveParams::default());
    for (k, p) in pl.passes.iter().enumerate() {
        eprintln!(
            "  pass {k}: {} pts {:.1?} -> {:.1?} {}",
            p.points.len(),
            p.points[0],
            p.points[p.points.len() - 1],
            p.ended
        );
    }
    let vp = VerifyParams::new(tool.radius(), 0.0, pl.target);
    let rep = verify(
        &c.region,
        &tp.events,
        &vp,
        LEVEL,
        EXCLUDE_BEGIN,
        EXCLUDE_END,
    );
    for x in rep.top.iter().take(6) {
        eprintln!(
            "     top: {:.1}° at {:.2?} move {} sample {}",
            x.0, x.1, x.2, x.3
        );
    }
    eprintln!(
        "coverage {:.3} gouge {:.4} max ratio {:.3}; planner leftover {:.1}/{:.1}",
        rep.coverage, rep.gouge, rep.max_engagement_ratio, pl.leftover_area, pl.reachable_area
    );
}
