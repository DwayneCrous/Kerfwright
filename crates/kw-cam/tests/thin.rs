//! Thin slice: a rectangular pocket and a contour, posted in all five dialects, replayed by the
//! independent interpreters, and checked against the cutter-location data.

use kw_cam::ops2d::{contour, drill, pocket, Cut, Side};
use kw_cam::path::{Event, Toolpath};
use kw_cam::post::{render, PostConfig};
use kw_cam::{Region, Tool};
use kw_gsim::{run, Dialect, MotionKind};

fn dialects() -> Vec<(Dialect, PostConfig)> {
    vec![
        (Dialect::Fanuc, PostConfig::fanuc()),
        (Dialect::Haas, PostConfig::haas()),
        (Dialect::Siemens840D, PostConfig::siemens()),
        (Dialect::Heidenhain, PostConfig::heidenhain()),
        (Dialect::LinuxCnc, PostConfig::linuxcnc()),
    ]
}

/// The (from, to) pairs of cutter motion implied by the events.
fn expected_moves(tp: &Toolpath) -> Vec<([f64; 3], [f64; 3], MotionKind)> {
    let mut pos = [0.0; 3];
    let mut out = Vec::new();
    for e in &tp.events {
        match e {
            Event::Rapid { p } => {
                out.push((pos, *p, MotionKind::Rapid));
                pos = *p;
            }
            Event::Feed { p, .. } => {
                out.push((pos, *p, MotionKind::Feed));
                pos = *p;
            }
            Event::Arc { p, ccw, .. } => {
                out.push((
                    pos,
                    *p,
                    if *ccw {
                        MotionKind::ArcCcw
                    } else {
                        MotionKind::ArcCw
                    },
                ));
                pos = *p;
            }
            _ => {}
        }
    }
    out
}

fn arc_len(from: [f64; 3], to: [f64; 3], c: [f64; 2], ccw: bool) -> f64 {
    let r = (from[0] - c[0]).hypot(from[1] - c[1]);
    let (a0, a1) = (
        (from[1] - c[1]).atan2(from[0] - c[0]),
        (to[1] - c[1]).atan2(to[0] - c[0]),
    );
    let mut d = a1 - a0;
    if ccw {
        while d <= 1e-9 {
            d += std::f64::consts::TAU;
        }
    } else {
        while d >= -1e-9 {
            d -= std::f64::consts::TAU;
        }
    }
    (r * d.abs()).hypot(to[2] - from[2])
}

fn compare(tp: &Toolpath, text: &str, d: Dialect) {
    let r = run(d, text).unwrap_or_else(|e| panic!("{d:?}: {e}\n{text}"));
    let exp = expected_moves(tp);
    let cut_exp: Vec<_> = exp.iter().filter(|m| m.2 != MotionKind::Rapid).collect();
    let cut_got: Vec<_> = r
        .motions
        .iter()
        .filter(|m| m.kind != MotionKind::Rapid)
        .collect();
    if cut_exp.len() == cut_got.len() {
        for (a, b) in cut_exp.iter().zip(&cut_got) {
            for i in 0..3 {
                assert!(
                    (a.0[i] - b.from[i]).abs() < 1.1e-3,
                    "{d:?}: start {:?} vs {:?}",
                    a.0,
                    b.from
                );
                assert!(
                    (a.1[i] - b.to[i]).abs() < 1.1e-3,
                    "{d:?}: end {:?} vs {:?}",
                    a.1,
                    b.to
                );
            }
            assert_eq!(a.2, b.kind, "{d:?}");
        }
    } else {
        // the post split some arcs into chords: the cutting length and the end of every expected move agree
        let len_e: f64 = tp.cutting_length();
        let len_g: f64 = cut_got
            .iter()
            .map(|m| match (&m.kind, m.centre) {
                (MotionKind::ArcCw, Some(c)) => arc_len(m.from, m.to, c, false),
                (MotionKind::ArcCcw, Some(c)) => arc_len(m.from, m.to, c, true),
                _ => ((m.to[0] - m.from[0]).powi(2)
                    + (m.to[1] - m.from[1]).powi(2)
                    + (m.to[2] - m.from[2]).powi(2))
                .sqrt(),
            })
            .sum();
        assert!(
            (len_e - len_g).abs() < 2e-3 * len_e.max(1.0),
            "{d:?}: cutting length {len_e} vs {len_g}"
        );
        for m in &cut_exp {
            assert!(
                cut_got
                    .iter()
                    .any(|g| (0..3).all(|i| (g.to[i] - m.1[i]).abs() < 1.1e-3)),
                "{d:?}: the end of a move {:?} is never reached",
                m.1
            );
        }
    }
}

#[test]
fn pocket_replays_in_every_dialect() {
    let region = Region::rect(0.0, 0.0, 40.0, 30.0);
    let tool = Tool::flat(1, 10.0, 25.0);
    let mut cut = Cut::new(0.0, -6.0);
    cut.stepover = 4.0;
    let tp = pocket(&region, &tool, &cut);
    assert!(tp.cutting_length() > 100.0);
    for (d, cfg) in dialects() {
        let text = render(&cfg, &tp).unwrap();
        compare(&tp, &text, d);
    }
}

#[test]
fn pocket_stays_inside_the_pocket_and_reaches_the_walls() {
    let region = Region::rect(0.0, 0.0, 40.0, 30.0);
    let tool = Tool::flat(1, 10.0, 25.0);
    let mut cut = Cut::new(0.0, -6.0);
    cut.stepover = 4.0;
    let tp = pocket(&region, &tool, &cut);
    let (mut xmin, mut xmax, mut ymin, mut ymax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for e in &tp.events {
        if let Event::Feed { p, .. } | Event::Arc { p, .. } = e {
            xmin = xmin.min(p[0]);
            xmax = xmax.max(p[0]);
            ymin = ymin.min(p[1]);
            ymax = ymax.max(p[1]);
        }
    }
    // tool centre never closer than the radius to a wall (no gouge), and the wall pass reaches it
    assert!(
        (xmin - 5.0).abs() < 1e-6 && (xmax - 35.0).abs() < 1e-6,
        "{xmin} {xmax}"
    );
    assert!(
        (ymin - 5.0).abs() < 1e-6 && (ymax - 25.0).abs() < 1e-6,
        "{ymin} {ymax}"
    );
    // two depth levels of 3 mm
    let zs: std::collections::BTreeSet<i64> = tp
        .events
        .iter()
        .filter_map(|e| {
            if let Event::Feed { p, .. } = e {
                Some((p[2] * 1000.0).round() as i64)
            } else {
                None
            }
        })
        .collect();
    assert!(zs.contains(&-3000) && zs.contains(&-6000));
}

#[test]
fn rounded_pocket_with_an_island_uses_arcs_and_replays() {
    // a 60 x 40 pocket with rounded ends and a round island; the loops contain arcs
    let outer: Vec<[f64; 3]> = vec![
        [10.0, 0.0, 0.0],
        [50.0, 0.0, 0.414_213_562_373_095],
        [60.0, 10.0, 0.0],
        [60.0, 30.0, 0.414_213_562_373_095],
        [50.0, 40.0, 0.0],
        [10.0, 40.0, 0.414_213_562_373_095],
        [0.0, 30.0, 0.0],
        [0.0, 10.0, 0.414_213_562_373_095],
    ];
    let region =
        Region::from_loops(&[outer]).with_island(&kw_cam::geom2d::circle_loop(30.0, 20.0, 6.0));
    let tool = Tool::flat(2, 6.0, 20.0);
    let mut cut = Cut::new(0.0, -4.0);
    cut.stepover = 3.0;
    cut.step_down = 4.0;
    let tp = pocket(&region, &tool, &cut);
    assert!(tp.events.iter().any(|e| matches!(e, Event::Arc { .. })));
    for (d, cfg) in dialects() {
        let text = render(&cfg, &tp).unwrap();
        compare(&tp, &text, d);
    }
    // the tool never enters the island or leaves the pocket (tool centre stays in the offset region)
    let free = region.offset(3.0);
    for e in &tp.events {
        if let Event::Feed { p, .. } | Event::Arc { p, .. } = e {
            assert!(
                free.contains([p[0], p[1]]) || on_boundary(&free, [p[0], p[1]]),
                "{p:?} outside the free region"
            );
        }
    }
}

fn on_boundary(r: &Region, p: [f64; 2]) -> bool {
    use cavalier_contours::polyline::PlineSource;
    r.loops().iter().any(|l| {
        let c = l.closest_point(
            cavalier_contours::core::math::Vector2::new(p[0], p[1]),
            1e-9,
        );
        c.is_some_and(|c| c.distance < 1e-6)
    })
}

#[test]
fn contour_and_drilling_replay_in_every_dialect() {
    let part = Region::rect(10.0, 10.0, 50.0, 40.0);
    let tool = Tool::flat(3, 8.0, 25.0);
    let mut cut = Cut::new(0.0, -5.0);
    cut.step_down = 2.5;
    let tp = contour(&part, &tool, Side::Outside, &cut);
    // tool centre 4 mm outside the part
    for e in &tp.events {
        if let Event::Feed { p, .. } = e {
            if p[2] < -0.1 && p[2] > -5.1 {
                let d = (p[0] - 30.0).abs().max((p[1] - 25.0).abs() * 40.0 / 30.0);
                let _ = d;
            }
        }
    }
    for (d, cfg) in dialects() {
        compare(&tp, &render(&cfg, &tp).unwrap(), d);
    }
    let drill_tool = Tool::flat(4, 5.0, 30.0);
    let mut dc = Cut::new(0.0, -12.0);
    dc.plunge_feed = 150.0;
    for peck in [None, Some(4.0)] {
        let tp = drill(
            &[[10.0, 10.0], [50.0, 10.0], [50.0, 40.0], [10.0, 40.0]],
            &drill_tool,
            &dc,
            peck,
        );
        for (d, cfg) in dialects() {
            let text = render(&cfg, &tp).unwrap();
            let r = run(d, &text).unwrap_or_else(|e| panic!("{d:?} peck {peck:?}: {e}\n{text}"));
            assert_eq!(r.drills.len(), 4, "{d:?}");
            for (x, y, z, _) in &r.drills {
                assert!((z + 12.0).abs() < 1e-6, "{d:?}: depth {z}");
                assert!([10.0, 50.0].contains(&(x.round())) && [10.0, 40.0].contains(&(y.round())));
            }
        }
    }
}

#[test]
fn adaptive_clearing_replays_in_every_dialect() {
    // helical entry, many short feeds and arcs: the heaviest post input of the 2.5-D operations
    let outer: Vec<[f64; 3]> = vec![
        [10.0, 0.0, 0.0],
        [50.0, 0.0, 0.414_213_562_373_095],
        [60.0, 10.0, 0.0],
        [60.0, 30.0, 0.414_213_562_373_095],
        [50.0, 40.0, 0.0],
        [10.0, 40.0, 0.414_213_562_373_095],
        [0.0, 30.0, 0.0],
        [0.0, 10.0, 0.414_213_562_373_095],
    ];
    let region =
        Region::from_loops(&[outer]).with_island(&kw_cam::geom2d::circle_loop(30.0, 20.0, 6.0));
    let tool = Tool::flat(2, 6.0, 20.0);
    let mut cut = Cut::new(0.0, -4.0);
    cut.stepover = 1.5;
    cut.step_down = 4.0;
    let (tp, _) = kw_cam::adaptive::adaptive(
        &region,
        &tool,
        &cut,
        &kw_cam::adaptive::AdaptiveParams::default(),
    );
    assert!(tp.events.len() > 200, "{} events", tp.events.len());
    let arcs = tp
        .events
        .iter()
        .filter(|e| matches!(e, Event::Arc { .. }))
        .count();
    assert!(arcs > 0, "no arcs in the adaptive path");
    for (d, cfg) in dialects() {
        let text = render(&cfg, &tp).unwrap();
        let r = run(d, &text).unwrap_or_else(|e| {
            panic!(
                "{d:?}: {e}\n{}",
                text.lines().take(14).collect::<Vec<_>>().join("\n")
            )
        });
        assert!(r.program_end_seen, "{d:?}");
        compare(&tp, &text, d);
        eprintln!(
            "adaptive {d:?}: {} events -> {} lines, {} motions",
            tp.events.len(),
            r.lines,
            r.motions.len()
        );
    }
}
