//! Curves put into a sketch: chains, simplification, copies and text.

use kw_sketch::curves::{add_curves, chain_segments, copy_entities, simplify_chain, Curve2};
use kw_sketch::entity::EntityKind;
use kw_sketch::text::{text_curves, TextStyle};
use kw_sketch::Sketch;
use std::f64::consts::{PI, TAU};

fn circle_pts(c: (f64, f64), r: f64, a0: f64, a1: f64, n: usize) -> Vec<(f64, f64)> {
    (0..=n)
        .map(|i| {
            let a = a0 + (a1 - a0) * i as f64 / n as f64;
            (c.0 + r * a.cos(), c.1 + r * a.sin())
        })
        .collect()
}

#[test]
fn a_tessellated_circle_becomes_a_circle_and_an_arc_stays_an_arc() {
    let full = circle_pts((3.0, -2.0), 5.0, 0.0, TAU, 64);
    match simplify_chain(&full, 0.01).as_slice() {
        [Curve2::Circle { c, r }] => {
            assert!((c.0 - 3.0).abs() < 1e-6 && (c.1 + 2.0).abs() < 1e-6 && (r - 5.0).abs() < 1e-6)
        }
        o => panic!("{o:?}"),
    }
    // a quarter turn from 0 to 90 degrees (counter-clockwise): one arc whose end points are the originals
    let quarter = circle_pts((0.0, 0.0), 5.0, 0.0, PI / 2.0, 20);
    match simplify_chain(&quarter, 0.01).as_slice() {
        [Curve2::Arc { c, start, end }] => {
            assert!(c.0.abs() < 1e-6 && c.1.abs() < 1e-6);
            assert!(
                (start.0 - 5.0).abs() < 1e-9 && start.1.abs() < 1e-9,
                "{start:?}"
            );
            assert!(end.0.abs() < 1e-9 && (end.1 - 5.0).abs() < 1e-9, "{end:?}");
        }
        o => panic!("{o:?}"),
    }
    // the same quarter walked clockwise is the same arc, stored counter-clockwise
    let mut rev = quarter.clone();
    rev.reverse();
    match simplify_chain(&rev, 0.01).as_slice() {
        [Curve2::Arc { start, end, .. }] => {
            assert!((start.0 - 5.0).abs() < 1e-9 && (end.1 - 5.0).abs() < 1e-9)
        }
        o => panic!("{o:?}"),
    }
}

#[test]
fn a_rounded_rectangle_section_is_lines_and_arcs() {
    // 20 x 10 with corner radius 2, walked counter-clockwise
    let mut pts = Vec::new();
    let r = 2.0;
    pts.extend(circle_pts((18.0, 2.0), r, -PI / 2.0, 0.0, 12));
    pts.extend(circle_pts((18.0, 8.0), r, 0.0, PI / 2.0, 12));
    pts.extend(circle_pts((2.0, 8.0), r, PI / 2.0, PI, 12));
    pts.extend(circle_pts((2.0, 2.0), r, PI, 1.5 * PI, 12));
    pts.push(pts[0]);
    let curves = simplify_chain(&pts, 0.005);
    let lines = curves
        .iter()
        .filter(|c| matches!(c, Curve2::Line(..)))
        .count();
    let arcs = curves
        .iter()
        .filter(|c| matches!(c, Curve2::Arc { .. }))
        .count();
    assert_eq!((lines, arcs), (4, 4), "{curves:#?}");
    // and they go into a sketch as one closed loop of 8 curves
    let mut sk = Sketch::new();
    let made = add_curves(&mut sk, &curves, false);
    assert_eq!(made.len(), 8);
}

#[test]
fn segments_are_chained_into_closed_loops() {
    let segs = vec![
        ((0.0, 0.0), (1.0, 0.0)),
        ((1.0, 1.0), (1.0, 0.0)),
        ((0.0, 1.0), (1.0, 1.0)),
        ((0.0, 0.0), (0.0, 1.0)),
        ((5.0, 5.0), (6.0, 5.0)),
    ];
    let mut chains = chain_segments(&segs, 1e-9);
    chains.sort_by_key(|c| c.len());
    assert_eq!(chains.len(), 2);
    assert_eq!(chains[0].len(), 2);
    assert_eq!(chains[1].len(), 5);
    assert_eq!(chains[1][0], chains[1][4]);
}

#[test]
fn copies_keep_arcs_and_share_points() {
    let mut sk = Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 4.0, 0.0);
    let a = sk.add_arc_xy(4.0, 1.0, 1.0, -PI / 2.0, PI / 2.0);
    let c = sk.add_circle_xy(0.0, 5.0, 2.0);
    let made = copy_entities(&mut sk, &[l, a, c], &|p| (p.0 + 10.0, p.1 + 1.0)).unwrap();
    assert_eq!(made.len(), 3);
    match sk.kind(made[1]).unwrap().clone() {
        EntityKind::Arc { center, start, end } => {
            assert_eq!(sk.point(center).unwrap(), (14.0, 2.0));
            let (s, e) = (sk.point(start).unwrap(), sk.point(end).unwrap());
            assert!((s.0 - 14.0).abs() < 1e-9 && (s.1 - 1.0).abs() < 1e-9);
            assert!((e.0 - 14.0).abs() < 1e-9 && (e.1 - 3.0).abs() < 1e-9);
        }
        k => panic!("{k:?}"),
    }
    // the line's end point 4,0 and the arc's start are different points; each copy uses its own
    match sk.kind(made[0]).unwrap().clone() {
        EntityKind::Line { p1, p2 } => {
            assert_eq!(sk.point(p1).unwrap(), (10.0, 1.0));
            assert_eq!(sk.point(p2).unwrap(), (14.0, 1.0));
        }
        k => panic!("{k:?}"),
    }
}

/// Area of the polygon through the sampled outline (nonzero winding, holes subtract through their
/// opposite orientation).
fn signed_area(curves: &[Curve2]) -> f64 {
    let mut a = 0.0;
    let bez = |pts: &[(f64, f64)], t: f64| -> (f64, f64) {
        let mut p = pts.to_vec();
        while p.len() > 1 {
            p = p
                .windows(2)
                .map(|w| {
                    (
                        w[0].0 + (w[1].0 - w[0].0) * t,
                        w[0].1 + (w[1].1 - w[0].1) * t,
                    )
                })
                .collect();
        }
        p[0]
    };
    for c in curves {
        let seg: Vec<(f64, f64)> = match c {
            Curve2::Line(p, q) => vec![*p, *q],
            Curve2::Bezier(v) => (0..=32).map(|i| bez(v, i as f64 / 32.0)).collect(),
            _ => continue,
        };
        for w in seg.windows(2) {
            a += w[0].0 * w[1].1 - w[1].0 * w[0].1;
        }
    }
    a / 2.0
}

#[test]
fn text_outlines_have_the_requested_cap_height_and_close_up() {
    let font = epaint_default_fonts::UBUNTU_LIGHT;
    let style = TextStyle {
        height: 10.0,
        origin: (5.0, 7.0),
        ..Default::default()
    };
    let (curves, width) = text_curves(font, "H", &style).unwrap();
    // an H is 12 straight segments, 10 mm tall, standing on the baseline y = 7
    let ys: Vec<f64> = curves
        .iter()
        .flat_map(|c| match c {
            Curve2::Line(a, b) => vec![a.1, b.1],
            Curve2::Bezier(v) => v.iter().map(|p| p.1).collect(),
            _ => vec![],
        })
        .collect();
    let (lo, hi) = ys
        .iter()
        .fold((f64::MAX, f64::MIN), |(l, h), y| (l.min(*y), h.max(*y)));
    assert!(
        (lo - 7.0).abs() < 1e-3 && (hi - 17.0).abs() < 1e-3,
        "{lo} {hi}"
    );
    assert!(width > 5.0 && width < 12.0, "{width}");
    // the outline closes: every curve end is the start of another
    let mut sk = Sketch::new();
    let made = add_curves(&mut sk, &curves, false);
    assert_eq!(made.len(), curves.len());
    // an O has two loops (outer and inner) with opposite orientation: the area is the ring's
    let (o, _) = text_curves(font, "O", &TextStyle::default()).unwrap();
    let ring = signed_area(&o).abs();
    let (i, _) = text_curves(font, "I", &TextStyle::default()).unwrap();
    assert!(ring > 5.0 && ring < 60.0, "{ring}");
    assert!(signed_area(&i).abs() > 2.0);
    // rotated by 90 degrees the cap height lies along -x
    let (r, _) = text_curves(
        font,
        "H",
        &TextStyle {
            angle_deg: 90.0,
            ..Default::default()
        },
    )
    .unwrap();
    let xs: Vec<f64> = r
        .iter()
        .flat_map(|c| match c {
            Curve2::Line(a, b) => vec![a.0, b.0],
            _ => vec![],
        })
        .collect();
    let xmin = xs.iter().cloned().fold(f64::MAX, f64::min);
    assert!((xmin + 10.0).abs() < 1e-3, "{xmin}");
}

#[test]
fn a_loop_that_starts_mid_side_still_gives_four_sides() {
    let pts = vec![
        (5.0, 0.0),
        (10.0, 0.0),
        (10.0, 5.0),
        (10.0, 10.0),
        (0.0, 10.0),
        (0.0, 0.0),
        (5.0, 0.0),
    ];
    let c = simplify_chain(&pts, 1e-6);
    assert_eq!(c.len(), 4, "{c:?}");
}
