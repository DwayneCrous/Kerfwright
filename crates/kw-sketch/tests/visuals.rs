//! Box selection and the closed regions that get shaded.

use kw_sketch::editor::*;
use kw_sketch::regions::{closed_loops, fill_regions};
use kw_sketch::*;

fn rect(sk: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) {
    let p: Vec<_> = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        .iter()
        .map(|q| sk.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
}

fn lines(ed: &Editor) -> usize {
    ed.selection
        .iter()
        .filter(|e| matches!(ed.sketch.kind(**e), Ok(EntityKind::Line { .. })))
        .count()
}
fn points(ed: &Editor) -> usize {
    ed.selection
        .iter()
        .filter(|e| matches!(ed.sketch.kind(**e), Ok(EntityKind::Point(_))))
        .count()
}

#[test]
fn a_window_takes_what_is_inside_and_a_crossing_box_what_it_touches() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 10.0, 6.0);
    sk.add_circle_xy(30.0, 0.0, 3.0);
    let mut ed = Editor::default();
    ed.replace_sketch(sk);
    // a window around the rectangle: its four sides and four corners, not the circle
    ed.select_in_box((-1.0, -1.0), (11.0, 7.0), false, false);
    assert_eq!((lines(&ed), points(&ed)), (4, 4));
    assert_eq!(ed.selection.len(), 8, "the circle's centre is outside");
    // a window over half the rectangle: only the corners in it, no side is complete
    ed.select_in_box((-1.0, -1.0), (5.0, 7.0), false, false);
    assert_eq!(
        (lines(&ed), points(&ed)),
        (1, 2),
        "the left side and its two corners"
    );
    // a crossing box over the same half also takes the two sides it cuts
    ed.select_in_box((-1.0, -1.0), (5.0, 7.0), true, false);
    assert_eq!(lines(&ed), 3);
    // a crossing box through only the circle's rim
    ed.select_in_box((26.0, -1.0), (28.0, 1.0), true, false);
    assert_eq!(ed.selection.len(), 1);
    // additive keeps what was selected
    ed.select_in_box((-1.0, -1.0), (11.0, 7.0), false, true);
    assert_eq!(ed.selection.len(), 1 + 8);
    // an empty box clears (unless additive)
    ed.select_in_box((100.0, 100.0), (101.0, 101.0), false, false);
    assert!(ed.selection.is_empty());
}

#[test]
fn closed_outlines_are_shaded_and_open_ones_are_not() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 10.0, 6.0);
    // an open "U"
    let a = sk.add_point(20.0, 0.0);
    let b = sk.add_point(20.0, 5.0);
    let c = sk.add_point(25.0, 5.0);
    sk.add_line(a, b);
    sk.add_line(b, c);
    let regions = fill_regions(&sk, 0.01);
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].holes.len(), 0);
    assert_eq!(regions[0].outer.len(), 4);
    // close the U: a second region
    let d = sk.add_point(25.0, 0.0);
    sk.add_line(c, d);
    sk.add_line(d, a);
    assert_eq!(fill_regions(&sk, 0.01).len(), 2);
}

#[test]
fn a_shape_inside_another_is_a_hole_and_the_shape_in_the_hole_is_shaded_again() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 30.0, 30.0);
    sk.add_circle_xy(15.0, 15.0, 10.0);
    sk.add_circle_xy(15.0, 15.0, 4.0);
    let regions = fill_regions(&sk, 0.01);
    // the square with the big circle as a hole, and the small circle inside that hole
    assert_eq!(regions.len(), 3 - 1);
    let outer = regions.iter().find(|r| r.outer.len() == 4).unwrap();
    assert_eq!(outer.holes.len(), 1);
    assert_eq!(closed_loops(&sk, 0.01).len(), 3);
    // construction geometry is never shaded
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 10.0, 6.0);
    let ids: Vec<_> = sk.entities.keys().collect();
    for e in ids {
        sk.set_construction(e, true);
    }
    assert!(fill_regions(&sk, 0.01).is_empty());
}

#[test]
fn projected_geometry_is_marked_and_survives_a_pin() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 10.0, 6.0);
    let mut ed = Editor::default();
    ed.replace_sketch(sk);
    ed.select_in_box((-1.0, -1.0), (11.0, 7.0), false, false);
    ed.pin_selection();
    let marked = ed.sketch.entities.values().filter(|e| e.projected).count();
    assert!(marked >= 4, "{marked}");
}
