//! Snapping and inference: midpoint, intersection, origin, polar angles, parallel / perpendicular,
//! alignment with acquired points and grid, with the constraints each snap implies.

use kw_sketch::constraint::ConstraintKind;
use kw_sketch::editor::*;
use kw_sketch::*;

fn click(ed: &mut Editor, x: f64, y: f64) {
    ed.handle(Input::Click((x, y)));
}

fn all_snaps() -> Editor {
    let mut ed = Editor::default();
    ed.snaps = SnapSettings::all_on();
    ed.snaps.grid = false;
    ed.tolerance = 0.3;
    ed
}

fn n_constraints(ed: &Editor, f: fn(&ConstraintKind) -> bool) -> usize {
    ed.sketch
        .constraints
        .values()
        .filter(|c| f(&c.kind))
        .count()
}

fn not_conflicting(ed: &Editor) {
    let r = ed.report.as_ref().expect("solved");
    assert_ne!(r.status, SketchStatus::Conflicting, "{r:?}");
}

fn last_line(ed: &Editor) -> (P2, P2) {
    let (_, e) = ed
        .sketch
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .last()
        .expect("a line");
    let EntityKind::Line { p1, p2 } = e.kind else {
        unreachable!()
    };
    (ed.sketch.point(p1).unwrap(), ed.sketch.point(p2).unwrap())
}

type P2 = (f64, f64);

#[test]
fn default_settings_keep_the_first_behaviour() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Line);
    click(&mut ed, 5.0, 5.0);
    // 1 degree off horizontal: inferred and constrained, as before
    click(&mut ed, 15.0, 5.17);
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Horizontal(_))),
        1
    );
    assert!((last_line(&ed).1 .1 - 5.0).abs() < 1e-9);
    // no grid, no midpoint, no origin by default
    assert!(matches!(ed.snap((0.01, 0.01)), Snap::Free(_)));
}

#[test]
fn midpoint_snap_places_a_point_with_a_midpoint_constraint() {
    let mut ed = all_snaps();
    ed.sketch.add_line_xy(0.0, 0.0, 10.0, 4.0);
    ed.refresh();
    let s = ed.snap((5.1, 2.05));
    assert!(
        matches!(s, Snap::Midpoint(_, p) if (p.0 - 5.0).abs() < 1e-12 && (p.1 - 2.0).abs() < 1e-12)
    );
    ed.set_tool(Tool::Line);
    click(&mut ed, 5.1, 2.05);
    click(&mut ed, 5.0, 9.0);
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Midpoint { .. })),
        1
    );
    not_conflicting(&ed);
    let (a, _) = last_line(&ed);
    assert!((a.0 - 5.0).abs() < 1e-9 && (a.1 - 2.0).abs() < 1e-9);
}

#[test]
fn endpoint_beats_midpoint_of_a_short_line() {
    let mut ed = all_snaps();
    ed.sketch.add_line_xy(0.0, 0.0, 0.4, 0.0);
    ed.refresh();
    assert!(matches!(ed.snap((0.35, 0.0)), Snap::Point(..)));
}

#[test]
fn intersection_of_two_lines_and_of_line_and_circle() {
    let mut ed = all_snaps();
    ed.sketch.add_line_xy(0.0, 0.0, 12.0, 8.0);
    ed.sketch.add_line_xy(0.0, 10.0, 12.0, 2.0);
    ed.sketch.add_circle_xy(20.0, 20.0, 5.0);
    ed.sketch.add_line_xy(15.0, 20.0, 30.0, 20.0);
    ed.refresh();
    let Snap::Intersection(_, _, q) = ed.snap((7.6, 4.9)) else {
        panic!("{:?}", ed.snap((7.6, 4.9)))
    };
    assert!((q.0 - 7.5).abs() < 1e-9 && (q.1 - 5.0).abs() < 1e-9);
    // line and circle
    let Snap::Intersection(_, _, q) = ed.snap((25.1, 19.9)) else {
        panic!("{:?}", ed.snap((25.1, 19.9)))
    };
    assert!((q.0 - 25.0).abs() < 1e-9 && (q.1 - 20.0).abs() < 1e-9);
    ed.set_tool(Tool::Line);
    click(&mut ed, 7.6, 4.9);
    click(&mut ed, 5.0, -3.0);
    // the new point is on both lines
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::PointOnCurve { .. })),
        2
    );
    not_conflicting(&ed);
    let (s, _) = last_line(&ed);
    assert!(
        (s.0 - 7.5).abs() < 1e-7 && (s.1 - 5.0).abs() < 1e-7,
        "{s:?}"
    );
    // line-circle: the point lands on both
    ed.handle(Input::Escape);
    click(&mut ed, 25.1, 19.9);
    click(&mut ed, 40.0, 40.0);
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::PointOnCurve { .. })),
        4
    );
    not_conflicting(&ed);
}

#[test]
fn origin_snap_makes_one_fixed_point() {
    let mut ed = all_snaps();
    ed.set_tool(Tool::Line);
    click(&mut ed, 0.1, -0.1);
    click(&mut ed, 10.0, 7.0);
    ed.handle(Input::Escape);
    click(&mut ed, 0.05, 0.05);
    click(&mut ed, -8.0, 3.0);
    let at_origin = ed
        .sketch
        .entities
        .iter()
        .filter(|(id, e)| {
            matches!(e.kind, EntityKind::Point(_))
                && ed
                    .sketch
                    .point(*id)
                    .is_ok_and(|p| p.0.abs() < 1e-12 && p.1.abs() < 1e-12)
        })
        .count();
    assert_eq!(at_origin, 1, "the second click reuses the origin point");
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Fix(_))),
        1
    );
    not_conflicting(&ed);
}

#[test]
fn polar_snap_rounds_the_angle_and_keeps_the_length() {
    for (deg, want) in [
        (29.0_f64, 30.0_f64),
        (44.0, 45.0),
        (-61.0, -60.0),
        (136.0, 135.0),
    ] {
        let mut ed = all_snaps();
        ed.set_tool(Tool::Line);
        click(&mut ed, 100.0, 100.0);
        let len = 10.0;
        let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
        let probe = (100.0 + len * c, 100.0 + len * s);
        let snap = ed.snap(probe);
        let Snap::Track(q, Ray::Angle { deg: d, .. }, None) = snap else {
            panic!("{deg}: {snap:?}")
        };
        assert!((d - want.rem_euclid(360.0)).abs() < 1e-9, "{d}");
        let (dx, dy) = (q.0 - 100.0, q.1 - 100.0);
        let got = dy.atan2(dx).to_degrees();
        assert!((got - want).abs() < 1e-9, "{deg}: {got}");
        // the length follows the cursor's projection on the ray
        assert!((dx.hypot(dy) - len * (deg - want).to_radians().cos()).abs() < 1e-9);
        click(&mut ed, probe.0, probe.1);
        // an angle snap is geometry only
        assert_eq!(ed.sketch.constraints.len(), 0);
    }
}

#[test]
fn polar_snap_adds_h_v_constraints_on_the_axes_and_ignores_off_angles() {
    let mut ed = all_snaps();
    ed.set_tool(Tool::Line);
    click(&mut ed, 50.0, 50.0);
    click(&mut ed, 50.15, 60.0); // ~0.9 degrees off vertical
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Vertical(_))),
        1
    );
    let (a, b) = last_line(&ed);
    assert!((a.0 - b.0).abs() < 1e-9);
    // 37 degrees is more than 3 degrees from any multiple of 15: free
    let mut ed = all_snaps();
    ed.set_tool(Tool::Line);
    click(&mut ed, 0.0, 30.0);
    let probe = (
        10.0 * 37.0_f64.to_radians().cos(),
        30.0 + 10.0 * 37.0_f64.to_radians().sin(),
    );
    assert!(
        matches!(ed.snap(probe), Snap::Free(_)),
        "{:?}",
        ed.snap(probe)
    );
    // polar off never snaps to angles
    ed.snaps.polar = None;
    let probe = (
        10.0 * 29.0_f64.to_radians().cos(),
        30.0 + 10.0 * 29.0_f64.to_radians().sin(),
    );
    assert!(matches!(ed.snap(probe), Snap::Free(_)));
}

#[test]
fn parallel_and_perpendicular_to_an_existing_line() {
    let mut ed = all_snaps();
    ed.snaps.polar = None;
    let base = ed.sketch.add_line_xy(0.0, 0.0, 20.0, 7.3); // not a multiple of any step
    ed.refresh();
    let phi = (7.3_f64).atan2(20.0);
    ed.set_tool(Tool::Line);
    click(&mut ed, 5.0, 40.0);
    let probe = (
        5.0 + 10.0 * (phi + 0.01).cos(),
        40.0 + 10.0 * (phi + 0.01).sin(),
    );
    let snap = ed.snap(probe);
    assert!(
        matches!(snap, Snap::Track(_, Ray::Parallel { line, .. }, None) if line == base),
        "{snap:?}"
    );
    click(&mut ed, probe.0, probe.1);
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Parallel { .. })),
        1
    );
    not_conflicting(&ed);
    // perpendicular from a new start
    ed.handle(Input::Escape);
    click(&mut ed, 30.0, 40.0);
    let psi = phi + std::f64::consts::FRAC_PI_2;
    let probe = (
        30.0 + 8.0 * (psi - 0.01).cos(),
        40.0 + 8.0 * (psi - 0.01).sin(),
    );
    let snap = ed.snap(probe);
    assert!(
        matches!(snap, Snap::Track(_, Ray::Perpendicular { line, .. }, None) if line == base),
        "{snap:?}"
    );
    click(&mut ed, probe.0, probe.1);
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Perpendicular { .. })),
        1
    );
    not_conflicting(&ed);
}

#[test]
fn acquired_points_give_alignment_lines_and_a_crossing() {
    let mut ed = all_snaps();
    let a = ed.sketch.add_point(0.0, 10.0);
    let b = ed.sketch.add_point(25.0, 0.0);
    ed.refresh();
    ed.set_tool(Tool::Point);
    // nothing acquired yet: no tracking
    assert!(matches!(ed.snap((12.0, 10.1)), Snap::Free(_)));
    ed.acquire(a);
    let Snap::Track(q, Ray::Horizontal { anchor: false, .. }, None) = ed.snap((12.0, 10.1)) else {
        panic!("{:?}", ed.snap((12.0, 10.1)))
    };
    assert!((q.1 - 10.0).abs() < 1e-12 && (q.0 - 12.0).abs() < 1e-12);
    ed.acquire(b);
    let Snap::Track(q, _, Some(_)) = ed.snap((24.9, 10.2)) else {
        panic!("{:?}", ed.snap((24.9, 10.2)))
    };
    assert!(
        (q.0 - 25.0).abs() < 1e-12 && (q.1 - 10.0).abs() < 1e-12,
        "{q:?}"
    );
    // a click places the point there and forgets the acquired points
    click(&mut ed, 24.9, 10.2);
    let found = ed.sketch.entities.iter().any(|(id, _)| {
        ed.sketch
            .point(id)
            .is_ok_and(|p| (p.0 - 25.0).abs() < 1e-12 && (p.1 - 10.0).abs() < 1e-12)
    });
    assert!(found);
    assert!(ed.acquired_points().is_empty());
    // only three points are remembered
    let extra: Vec<_> = (0..5)
        .map(|i| ed.sketch.add_point(i as f64 * 7.0, 90.0))
        .collect();
    for e in extra {
        ed.acquire(e);
    }
    assert_eq!(ed.acquired_points().len(), 3);
}

#[test]
fn alignment_crossing_with_a_polar_ray() {
    let mut ed = all_snaps();
    let a = ed.sketch.add_point(40.0, 10.0);
    ed.refresh();
    ed.set_tool(Tool::Line);
    click(&mut ed, 0.0, 0.0);
    ed.acquire(a);
    // the 45 degree ray from the start meets the horizontal through (40, 10) at (10, 10)
    let Snap::Track(q, _, Some(_)) = ed.snap((10.1, 9.9)) else {
        panic!("{:?}", ed.snap((10.1, 9.9)))
    };
    assert!(
        (q.0 - 10.0).abs() < 1e-9 && (q.1 - 10.0).abs() < 1e-9,
        "{q:?}"
    );
}

#[test]
fn grid_snap_rounds_and_loses_to_everything_else() {
    let mut ed = all_snaps();
    ed.snaps.grid = true;
    ed.snaps.grid_step = 5.0;
    ed.set_tool(Tool::Point);
    click(&mut ed, 12.3, 7.4);
    assert!(ed
        .sketch
        .entities
        .iter()
        .any(|(id, _)| ed.sketch.point(id).is_ok_and(|p| p == (10.0, 5.0))));
    // a point within the tolerance beats the grid node
    ed.sketch.add_point(20.2, 20.1);
    assert!(matches!(ed.snap((20.25, 20.0)), Snap::Point(..)));
    // suspended snapping (Alt) gives the raw position
    ed.suspend_snaps = true;
    assert_eq!(ed.snap((12.3, 7.4)), Snap::Free((12.3, 7.4)));
    ed.suspend_snaps = false;
    ed.snaps.grid = false;
    assert!(matches!(ed.snap((12.3, 7.4)), Snap::Free(_)));
}

#[test]
fn rectangle_corners_snap_to_the_origin_and_midpoints() {
    let mut ed = all_snaps();
    ed.sketch.add_line_xy(10.0, -20.0, 10.0, 30.0);
    ed.refresh();
    ed.set_tool(Tool::Rectangle);
    click(&mut ed, 0.1, 0.1); // origin
    click(&mut ed, 10.05, 5.1); // midpoint of the vertical line
    not_conflicting(&ed);
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Midpoint { .. })),
        1
    );
    assert_eq!(
        n_constraints(&ed, |k| matches!(k, ConstraintKind::Fix(_))),
        1
    );
}

#[test]
fn random_clicks_with_every_snap_on_never_conflict_or_panic() {
    let mut state = 0x2545_F491_4F6C_DD1D_u64;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    for round in 0..20 {
        let mut ed = all_snaps();
        ed.snaps.grid = round % 2 == 0;
        ed.snaps.grid_step = 2.0;
        ed.tolerance = 0.5;
        let tools = [
            Tool::Line,
            Tool::Rectangle,
            Tool::Circle,
            Tool::Arc,
            Tool::Point,
            Tool::Line,
        ];
        for step in 0..60 {
            if step % 10 == 0 {
                ed.set_tool(tools[(rnd() * tools.len() as f64) as usize % tools.len()]);
            }
            let p = (rnd() * 40.0 - 20.0, rnd() * 40.0 - 20.0);
            ed.handle(Input::Move(p));
            if rnd() < 0.3 {
                if let Some(id) = ed.sketch.entities.keys().next() {
                    ed.acquire(id);
                }
            }
            ed.handle(Input::Click(p));
            if step % 17 == 16 {
                ed.handle(Input::Escape);
            }
        }
        not_conflicting(&ed);
    }
}
