//! The planar arrangement: curves split at their intersections, regions as faces, closed-form areas.

use kw_sketch::arrangement::Arrangement;
use kw_sketch::curves::add_curves;
use kw_sketch::text::{text_curves, TextStyle};
use kw_sketch::Sketch;
use std::f64::consts::PI;

fn rect(sk: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) {
    let p: Vec<_> = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        .iter()
        .map(|q| sk.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        sk.add_line(p[i], p[(i + 1) % 4]);
    }
}

fn areas(arr: &Arrangement) -> Vec<f64> {
    let mut v: Vec<f64> = arr.faces.iter().map(|f| f.area).collect();
    v.sort_by(f64::total_cmp);
    v
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * (1.0 + b.abs())
}

#[test]
fn a_line_across_a_circle_cuts_it_into_two_halves_and_its_ends_outside_are_ignored() {
    let mut sk = Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 5.0);
    sk.add_line_xy(-6.0, 0.0, 6.0, 0.0);
    let arr = Arrangement::build(&sk);
    assert_eq!(arr.faces.len(), 2);
    for a in areas(&arr) {
        assert!(close(a, 25.0 * PI / 2.0), "{a}");
    }
    // an off-centre chord: segments r² acos(d/r) − d √(r² − d²) and the rest
    let mut sk = Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 5.0);
    sk.add_line_xy(-6.0, 2.0, 6.0, 2.0);
    let arr = Arrangement::build(&sk);
    let small = 25.0 * (2.0f64 / 5.0).acos() - 2.0 * 21.0f64.sqrt();
    let a = areas(&arr);
    assert_eq!(a.len(), 2);
    assert!(
        close(a[0], small) && close(a[1], 25.0 * PI - small),
        "{a:?}"
    );
}

#[test]
fn two_bars_crossing_make_a_centre_and_four_arms() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 4.0, 10.0, 6.0);
    rect(&mut sk, 4.0, 0.0, 6.0, 10.0);
    let arr = Arrangement::build(&sk);
    let a = areas(&arr);
    assert_eq!(a.len(), 5, "{a:?}");
    assert!(close(a[0], 4.0));
    for x in &a[1..] {
        assert!(close(*x, 8.0), "{x}");
    }
    assert!(arr.faces.iter().all(|f| f.depth == 0 && f.holes.is_empty()));
}

#[test]
fn a_circle_over_a_square_edge_gives_three_regions() {
    let mut sk = Sketch::new();
    rect(&mut sk, 5.0, 0.0, 15.0, 10.0);
    sk.add_circle_xy(15.0, 5.0, 3.0);
    let arr = Arrangement::build(&sk);
    let half = 0.5 * PI * 9.0;
    let a = areas(&arr);
    assert_eq!(a.len(), 3, "{a:?}");
    assert!(
        close(a[0], half) && close(a[1], half) && close(a[2], 100.0 - half),
        "{a:?}"
    );
}

#[test]
fn shapes_inside_shapes_are_holes_and_the_core_is_a_region_of_its_own() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 30.0, 30.0);
    sk.add_circle_xy(15.0, 15.0, 10.0);
    sk.add_circle_xy(15.0, 15.0, 4.0);
    let arr = Arrangement::build(&sk);
    assert_eq!(arr.faces.len(), 3);
    let by_depth = |d: usize| arr.faces.iter().find(|f| f.depth == d).unwrap();
    assert!(close(by_depth(0).area, 900.0 - 100.0 * PI));
    assert_eq!(by_depth(0).holes.len(), 1);
    assert!(close(by_depth(1).area, 100.0 * PI - 16.0 * PI));
    assert!(close(by_depth(2).area, 16.0 * PI));
    // the core of the square-with-hole is a region too: merged outline of ring + core = the square
    let ring = arr.faces.iter().position(|f| f.depth == 0).unwrap();
    let mid = arr.faces.iter().position(|f| f.depth == 1).unwrap();
    let core = arr.faces.iter().position(|f| f.depth == 2).unwrap();
    let all = arr.merge_faces(&[ring, mid, core]);
    assert_eq!(all.len(), 1);
    assert!(all[0].holes.is_empty());
    assert!(close(arr.loop_area(&all[0].outer), 900.0));
}

#[test]
fn a_lone_closed_curve_stays_one_closed_edge() {
    let mut sk = Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 3.0);
    let arr = Arrangement::build(&sk);
    assert_eq!((arr.faces.len(), arr.edges.len()), (1, 1));
    assert_eq!(arr.edges[0].from, arr.edges[0].to);
    assert!(close(arr.faces[0].area, 9.0 * PI));
}

#[test]
fn tangent_circles_touch_without_a_region_between_them() {
    let mut sk = Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 3.0);
    sk.add_circle_xy(6.0, 0.0, 3.0);
    let arr = Arrangement::build(&sk);
    let a = areas(&arr);
    assert_eq!(a.len(), 2, "{a:?}");
    assert!(close(a[0], 9.0 * PI) && close(a[1], 9.0 * PI));
}

#[test]
fn rectangles_sharing_part_of_an_edge_share_those_pieces() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 10.0, 10.0);
    rect(&mut sk, 10.0, 2.0, 20.0, 8.0);
    let arr = Arrangement::build(&sk);
    let a = areas(&arr);
    assert_eq!(a.len(), 2, "{a:?}");
    assert!(close(a[0], 60.0) && close(a[1], 100.0));
}

#[test]
fn free_standing_open_curves_are_not_regions() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 10.0, 6.0);
    sk.add_line_xy(10.0, 6.0, 14.0, 9.0); // a whisker from a corner
    sk.add_line_xy(30.0, 0.0, 30.0, 5.0); // a stray line
    let arr = Arrangement::build(&sk);
    assert_eq!(arr.faces.len(), 1);
    assert!(close(arr.faces[0].area, 60.0));
    // an open L alone has no region at all
    let mut sk = Sketch::new();
    sk.add_line_xy(0.0, 0.0, 5.0, 0.0);
    sk.add_line_xy(5.0, 0.0, 5.0, 5.0);
    assert!(Arrangement::build(&sk).faces.is_empty());
}

#[test]
fn a_line_across_an_ellipse_cuts_it_in_halves_exactly() {
    let mut sk = Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    let m = sk.add_point(6.0, 0.0);
    sk.add_ellipse(c, m, 3.0);
    sk.add_line_xy(-7.0, 0.0, 7.0, 0.0);
    let arr = Arrangement::build(&sk);
    let a = areas(&arr);
    assert_eq!(a.len(), 2, "{a:?}");
    for x in a {
        assert!(close(x, 9.0 * PI), "{x}");
    }
    // a tilted chord through the middle splits it in halves as well
    let mut sk = Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    let m = sk.add_point(6.0, 0.0);
    sk.add_ellipse(c, m, 3.0);
    sk.add_line_xy(-5.0, -5.0, 5.0, 5.0);
    let a = areas(&Arrangement::build(&sk));
    assert_eq!(a.len(), 2);
    assert!(close(a[0], 9.0 * PI) && close(a[1], 9.0 * PI), "{a:?}");
}

#[test]
fn a_chord_across_a_closed_spline_splits_its_area_and_meets_it_exactly() {
    let build = |with_line: bool| {
        let mut sk = Sketch::new();
        let pts = [
            (0.0, 0.0),
            (6.0, -3.0),
            (12.0, 0.0),
            (12.0, 8.0),
            (6.0, 11.0),
            (0.0, 8.0),
            (0.0, 0.0),
        ];
        let ids: Vec<_> = pts.iter().map(|p| sk.add_point(p.0, p.1)).collect();
        // the first and last control point are the same point entity: the spline is closed
        let mut ctrl = ids.clone();
        *ctrl.last_mut().unwrap() = ids[0];
        sk.add_spline(3, ctrl).unwrap();
        if with_line {
            sk.add_line_xy(-2.0, 4.0, 14.0, 4.0);
        }
        Arrangement::build(&sk)
    };
    let whole = build(false);
    assert_eq!(whole.faces.len(), 1);
    let cut = build(true);
    let a = areas(&cut);
    assert_eq!(a.len(), 2, "{a:?}");
    assert!(
        (a[0] + a[1] - whole.faces[0].area).abs() < 1e-8,
        "{a:?} vs {}",
        whole.faces[0].area
    );
    // the crossing vertices lie on the chord to kernel precision
    for v in &cut.vertices {
        if (v.1 - 4.0).abs() < 1.0 {
            assert!((v.1 - 4.0).abs() < 1e-9, "{v:?}");
        }
    }
}

#[test]
fn merging_the_two_halves_of_a_cut_circle_leaves_the_plain_circle() {
    let mut sk = Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 5.0);
    sk.add_line_xy(-6.0, 0.0, 6.0, 0.0);
    let arr = Arrangement::build(&sk);
    let both = arr.merge_faces(&[0, 1]);
    assert_eq!(both.len(), 1);
    assert_eq!(both[0].outer.len(), 2, "two arcs, no chord");
    assert!(close(arr.loop_area(&both[0].outer), 25.0 * PI));
    let half = arr.merge_faces(&[0]);
    assert_eq!(half.len(), 1);
    assert_eq!(half[0].outer.len(), 2, "one arc and the chord");
    assert!(close(arr.loop_area(&half[0].outer), 25.0 * PI / 2.0));
}

#[test]
fn interior_points_lie_inside_their_faces_and_outside_the_holes() {
    let mut sk = Sketch::new();
    rect(&mut sk, 0.0, 0.0, 30.0, 30.0);
    sk.add_circle_xy(15.0, 15.0, 10.0);
    let arr = Arrangement::build(&sk);
    for f in &arr.faces {
        let p = arr.interior_point(f);
        assert!(arr.face_contains(f, p), "{p:?}");
    }
    let ring = arr.faces.iter().find(|f| f.depth == 0).unwrap();
    assert!(!arr.face_contains(ring, (15.0, 15.0)));
}

#[test]
fn the_letter_o_is_a_ring_and_a_counter() {
    let (curves, _) = text_curves(
        epaint_default_fonts::UBUNTU_LIGHT,
        "O",
        &TextStyle {
            height: 20.0,
            ..Default::default()
        },
    )
    .unwrap();
    let mut sk = Sketch::new();
    add_curves(&mut sk, &curves, false);
    let arr = Arrangement::build(&sk);
    assert_eq!(arr.faces.len(), 2, "{:?}", areas(&arr));
    let outer = arr.faces.iter().find(|f| f.depth == 0).unwrap();
    let core = arr.faces.iter().find(|f| f.depth == 1).unwrap();
    assert_eq!(outer.holes.len(), 1);
    assert!(outer.area > 0.0 && core.area > 0.0 && outer.area < 400.0);
}

#[test]
fn two_perpendicular_ellipses_make_a_centre_and_four_equal_tips() {
    let mut sk = Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    let m1 = sk.add_point(6.0, 0.0);
    sk.add_ellipse(c, m1, 3.0);
    let m2 = sk.add_point(0.0, 6.0);
    sk.add_ellipse(c, m2, 3.0);
    let arr = Arrangement::build(&sk);
    let a = areas(&arr);
    assert_eq!(a.len(), 5, "{a:?}");
    // four equal tips and the middle; the middle plus two tips is one whole ellipse
    for t in &a[..4] {
        assert!((t - a[0]).abs() < 1e-8, "{a:?}");
    }
    assert!((a[4] + 2.0 * a[0] - 18.0 * PI).abs() < 1e-8, "{a:?}");
}
