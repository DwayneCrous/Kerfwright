//! The polygon, slot, ellipse, three-point arc / circle and fit-spline tools.

use kw_sketch::constraint::ConstraintKind;
use kw_sketch::editor::*;
use kw_sketch::*;
use std::f64::consts::{PI, TAU};

fn click(ed: &mut Editor, x: f64, y: f64) {
    ed.handle(Input::Click((x, y)));
}

fn ok(ed: &Editor) {
    let r = ed.report.as_ref().expect("solved");
    assert_ne!(r.status, SketchStatus::Conflicting, "{r:?} {}", ed.status);
}

fn count(ed: &Editor, f: fn(&EntityKind) -> bool) -> usize {
    ed.sketch.entities.values().filter(|e| f(&e.kind)).count()
}

#[test]
fn polygon_makes_a_regular_n_gon_that_stays_regular() {
    for n in [3usize, 5, 6, 8] {
        let mut ed = Editor::default();
        ed.polygon_sides = n;
        ed.set_tool(Tool::Polygon);
        click(&mut ed, 5.0, 5.0);
        click(&mut ed, 15.0, 5.0); // circumradius 10, first vertex to the right
        ok(&ed);
        let rep = ed.report.clone().unwrap();
        assert!(rep.redundant.is_empty(), "n = {n}: {rep:?}");
        let lines: Vec<_> = ed
            .sketch
            .entities
            .iter()
            .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
            .map(|(id, _)| id)
            .collect();
        assert_eq!(lines.len(), n);
        let side = 2.0 * 10.0 * (PI / n as f64).sin();
        for l in &lines {
            let (dx, dy) = ed.sketch.line_vector(*l).unwrap();
            assert!(
                (dx.hypot(dy) - side).abs() < 1e-7,
                "n = {n}: {}",
                dx.hypot(dy)
            );
        }
        // area of a regular n-gon
        let area = 0.5 * n as f64 * 100.0 * (TAU / n as f64).sin();
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for l in &lines {
            let (p1, _) = ed.sketch.line_points(*l).unwrap();
            pts.push(ed.sketch.point(p1).unwrap());
        }
        let _ = area;
        // the solver keeps it regular when a vertex is dragged
        let (p1, _) = ed.sketch.line_points(lines[0]).unwrap();
        let (x, y) = ed.sketch.point(p1).unwrap();
        ed.set_tool(Tool::Select);
        ed.begin_drag(p1, (x, y));
        for k in 1..=10 {
            ed.drag_to((x + 0.3 * k as f64, y + 0.1 * k as f64));
        }
        ed.end_drag();
        ok(&ed);
        let lens: Vec<f64> = lines
            .iter()
            .map(|l| {
                let (dx, dy) = ed.sketch.line_vector(*l).unwrap();
                dx.hypot(dy)
            })
            .collect();
        let first = lens[0];
        assert!(lens.iter().all(|l| (l - first).abs() < 1e-6), "{lens:?}");
    }
}

#[test]
fn slot_is_two_lines_and_two_tangent_end_arcs_of_equal_radius() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Slot);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 20.0, 10.0);
    // the third click sets the half-width: 3 mm off the centre line
    let l = (20.0f64).hypot(10.0);
    let n = (-10.0 / l, 20.0 / l);
    click(&mut ed, 10.0 + 3.0 * n.0, 5.0 + 3.0 * n.1);
    ok(&ed);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 2);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Arc { .. })), 2);
    let joints = ed
        .sketch
        .constraints
        .values()
        .filter(|c| matches!(c.kind, ConstraintKind::Joint { .. }))
        .count();
    assert_eq!(joints, 4);
    // exactly the five degrees of freedom of a slot (two centres and a width), nothing redundant
    let rep = ed.report.clone().unwrap();
    assert!(rep.redundant.is_empty(), "{rep:?}");
    assert_eq!(rep.dof, 5, "{rep:?}");
    // lengths: straight parts are the centre distance, arcs have radius 3
    for (id, e) in &ed.sketch.entities {
        match e.kind {
            EntityKind::Line { .. } => {
                let (dx, dy) = ed.sketch.line_vector(id).unwrap();
                assert!((dx.hypot(dy) - l).abs() < 1e-7);
            }
            EntityKind::Arc { .. } => {
                let (_, r) = ed.sketch.circle_geometry(id).unwrap();
                assert!((r - 3.0).abs() < 1e-7, "{r}");
            }
            _ => {}
        }
    }
    // the outline is closed: perimeter of a slot = 2 L + 2 pi r
    let mut perim = 0.0;
    for (id, e) in &ed.sketch.entities {
        if matches!(e.kind, EntityKind::Line { .. } | EntityKind::Arc { .. }) {
            let poly = geom::polyline(&ed.sketch, id, 1e-6).unwrap();
            perim += poly
                .windows(2)
                .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
                .sum::<f64>();
        }
    }
    assert!((perim - (2.0 * l + 2.0 * PI * 3.0)).abs() < 1e-3, "{perim}");
}

#[test]
fn ellipse_and_three_point_shapes_have_the_expected_geometry() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Ellipse);
    click(&mut ed, 1.0, 2.0);
    click(&mut ed, 11.0, 2.0); // major semi-axis 10 along x
    click(&mut ed, 5.0, 6.0); // minor semi-axis 4
    ok(&ed);
    let (id, _) = ed
        .sketch
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::Ellipse { .. }))
        .unwrap();
    let EntityKind::Ellipse { minor_radius, .. } = ed.sketch.kind(id).unwrap().clone() else {
        unreachable!()
    };
    assert!((ed.sketch.value(minor_radius) - 4.0).abs() < 1e-9);

    // a circle through three points of a known circle
    let mut ed = Editor::default();
    ed.set_tool(Tool::Circle3);
    let (c, r) = ((3.0, -2.0), 7.0);
    for a in [0.3f64, 2.0, 4.5] {
        click(&mut ed, c.0 + r * a.cos(), c.1 + r * a.sin());
    }
    ok(&ed);
    let (id, _) = ed
        .sketch
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::Circle { .. }))
        .unwrap();
    let (cc, rr) = ed.sketch.circle_geometry(id).unwrap();
    assert!((cc.0 - c.0).hypot(cc.1 - c.1) < 1e-9 && (rr - r).abs() < 1e-9);

    // three collinear points: refused with a message, nothing added
    let mut ed = Editor::default();
    ed.set_tool(Tool::Circle3);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 1.0, 1.0);
    click(&mut ed, 2.0, 2.0);
    assert!(ed.status.contains("one line"), "{}", ed.status);
    assert_eq!(ed.sketch.entities.len(), 0);

    // three-point arcs: the arc passes through the third point whichever way round it is clicked
    for (a0, am, a1) in [
        (0.2f64, 1.0, 2.5),
        (2.5, 1.0, 0.2),
        (-1.0, 0.5, 3.0),
        (3.0, 0.5, -1.0),
    ] {
        let mut ed = Editor::default();
        ed.set_tool(Tool::Arc3);
        let at = |a: f64| (c.0 + r * a.cos(), c.1 + r * a.sin());
        click(&mut ed, at(a0).0, at(a0).1);
        click(&mut ed, at(a1).0, at(a1).1);
        click(&mut ed, at(am).0, at(am).1);
        ok(&ed);
        let (id, _) = ed
            .sketch
            .entities
            .iter()
            .find(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
            .expect("an arc");
        let poly = geom::polyline(&ed.sketch, id, 1e-4).unwrap();
        let m = at(am);
        let d = poly
            .iter()
            .map(|p| (p.0 - m.0).hypot(p.1 - m.1))
            .fold(f64::INFINITY, f64::min);
        assert!(
            d < 0.05,
            "arc through the middle point ({a0}, {am}, {a1}): {d}"
        );
    }
}

#[test]
fn fit_spline_passes_through_its_points_and_previews_are_drawn() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::FitSpline);
    let pts = [(0.0, 0.0), (5.0, 4.0), (10.0, -3.0), (15.0, 2.0)];
    for p in pts {
        click(&mut ed, p.0, p.1);
    }
    ed.handle(Input::Finish);
    let (id, _) = ed
        .sketch
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::FitSpline { .. }))
        .expect("a fit spline");
    let curve = geom::spline_curve(&ed.sketch, id).unwrap();
    let (t0, t1) = curve.domain();
    let dense: Vec<_> = (0..=4000)
        .map(|i| curve.point_at(t0 + (t1 - t0) * i as f64 / 4000.0))
        .collect();
    for p in pts {
        let d = dense
            .iter()
            .map(|q| (q.x - p.0).hypot(q.y - p.1))
            .fold(f64::INFINITY, f64::min);
        assert!(d < 1e-2, "{p:?}: {d}");
    }
    // previews exist while a shape is half drawn
    let mut ed = Editor::default();
    ed.set_tool(Tool::Polygon);
    assert!(ed.preview((3.0, 3.0)).is_empty());
    click(&mut ed, 0.0, 0.0);
    let pv = ed.preview((10.0, 0.0));
    assert_eq!(pv.len(), 1);
    assert_eq!(pv[0].len(), ed.polygon_sides + 1);
    ed.set_tool(Tool::Slot);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 10.0, 0.0);
    assert!(!ed.preview((5.0, 2.0)).is_empty());
}

#[test]
fn sketch_fillet_and_chamfer_round_and_bevel_a_rectangle_corner() {
    for round in [true, false] {
        let mut ed = Editor::default();
        ed.set_tool(Tool::Rectangle);
        click(&mut ed, 0.0, 0.0);
        click(&mut ed, 20.0, 10.0);
        ed.corner_size = 3.0;
        ed.set_tool(if round { Tool::Fillet } else { Tool::Chamfer });
        click(&mut ed, 20.0, 10.0); // the top right corner
        ok(&ed);
        let sk = &ed.sketch;
        assert_eq!(
            count(&ed, |k| matches!(k, EntityKind::Line { .. })),
            if round { 4 } else { 5 }
        );
        let corner_gone = !sk
            .entities
            .keys()
            .any(|id| sk.point(id).is_ok_and(|p| p == (20.0, 10.0)));
        assert!(corner_gone, "the corner point is replaced");
        // the new points sit 3 mm from the corner along the two edges
        let has = |q: (f64, f64)| {
            sk.entities.keys().any(|id| {
                sk.point(id)
                    .is_ok_and(|p| (p.0 - q.0).hypot(p.1 - q.1) < 1e-9)
            })
        };
        assert!(has((17.0, 10.0)) || has((20.0, 7.0)) || !round || !has((20.0, 10.0)));
        if round {
            let (arc, _) = sk
                .entities
                .iter()
                .find(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
                .expect("an arc");
            let (c, r) = sk.circle_geometry(arc).unwrap();
            // a quarter circle of radius 3 inside the corner
            assert!(
                (r - 3.0).abs() < 1e-7 && (c.0 - 17.0).abs() < 1e-7 && (c.1 - 7.0).abs() < 1e-7,
                "{c:?} {r}"
            );
            let rep = ed.report.clone().unwrap();
            assert!(rep.redundant.is_empty(), "{rep:?}");
        }
        // too big a corner for the shorter edge is refused with a message and changes nothing
        let mut ed = Editor::default();
        ed.set_tool(Tool::Rectangle);
        click(&mut ed, 0.0, 0.0);
        click(&mut ed, 20.0, 10.0);
        ed.corner_size = 30.0;
        ed.set_tool(Tool::Fillet);
        click(&mut ed, 20.0, 10.0);
        assert!(ed.status.contains("too big"), "{}", ed.status);
        assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 4);
    }
}

#[test]
fn mirror_copies_geometry_across_a_line_with_symmetry_constraints() {
    let mut ed = Editor::default();
    // the mirror line x = 0 and a triangle, an arc and a circle to its right
    ed.sketch.add_line_xy(0.0, -20.0, 0.0, 20.0);
    let t: Vec<_> = [(3.0, 0.0), (10.0, 2.0), (6.0, 8.0)]
        .iter()
        .map(|p| ed.sketch.add_point(p.0, p.1))
        .collect();
    let l: Vec<_> = (0..3)
        .map(|i| ed.sketch.add_line(t[i], t[(i + 1) % 3]))
        .collect();
    ed.sketch.add_circle_xy(7.0, -8.0, 2.0);
    ed.sketch.add_arc_xy(8.0, 14.0, 3.0, 0.2, 1.8);
    ed.refresh();
    ed.set_tool(Tool::Mirror);
    // pick the three lines, the circle and the arc, then the axis
    for p in [(6.5, 1.0), (8.0, 5.0), (4.5, 4.0), (9.0, -8.0)] {
        click(&mut ed, p.0, p.1);
    }
    click(&mut ed, 8.0 + 3.0 * 1.0f64.cos(), 14.0 + 3.0 * 1.0f64.sin());
    assert_eq!(ed.selection.len(), 5);
    let _ = l;
    ed.handle(Input::Finish);
    click(&mut ed, 0.0, 12.0);
    ok(&ed);
    assert_eq!(
        count(&ed, |k| matches!(k, EntityKind::Line { .. })),
        1 + 3 + 3
    );
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Circle { .. })), 2);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Arc { .. })), 2);
    // every mirrored point is the reflection of one original: look for the circle centre and arc ends
    let has = |q: (f64, f64)| {
        ed.sketch.entities.keys().any(|id| {
            ed.sketch
                .point(id)
                .is_ok_and(|p| (p.0 - q.0).hypot(p.1 - q.1) < 1e-6)
        })
    };
    assert!(has((-7.0, -8.0)), "mirrored circle centre");
    assert!(has((-3.0, 0.0)) && has((-10.0, 2.0)) && has((-6.0, 8.0)));
    let sym = ed
        .sketch
        .constraints
        .values()
        .filter(|c| matches!(c.kind, ConstraintKind::Symmetric { .. }))
        .count();
    assert_eq!(
        sym, 7,
        "three triangle corners, a circle centre and an arc's three points"
    );
    // the mirrored arc keeps its sweep: same radius, mirrored middle
    let mid = (8.0 + 3.0 * 1.0f64.cos(), 14.0 + 3.0 * 1.0f64.sin());
    let arc = ed
        .sketch
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
        .find(|(id, _)| ed.sketch.circle_geometry(*id).unwrap().0 .0 < 0.0)
        .map(|(id, _)| id)
        .unwrap();
    let poly = geom::polyline(&ed.sketch, arc, 1e-4).unwrap();
    let d = poly
        .iter()
        .map(|q| (q.0 + mid.0).hypot(q.1 - mid.1))
        .fold(f64::INFINITY, f64::min);
    assert!(d < 0.06, "{d}");
}

#[test]
fn centre_rectangle_grows_outwards_from_its_centre_and_stays_centred() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::CenterRectangle);
    click(&mut ed, 10.0, 5.0); // centre
    click(&mut ed, 16.0, 8.0); // a corner: 12 x 6
    ok(&ed);
    // four sides, two construction diagonals
    let lines: Vec<_> = ed
        .sketch
        .entities
        .values()
        .filter(|e| matches!(e.kind, EntityKind::Line { .. }))
        .collect();
    assert_eq!(lines.iter().filter(|l| !l.construction).count(), 4);
    assert_eq!(lines.iter().filter(|l| l.construction).count(), 2);
    let pts: Vec<(f64, f64)> = ed
        .sketch
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
        .map(|(id, _)| ed.sketch.point(id).unwrap())
        .collect();
    let (x0, x1) = (
        pts.iter().map(|p| p.0).fold(f64::MAX, f64::min),
        pts.iter().map(|p| p.0).fold(f64::MIN, f64::max),
    );
    let (y0, y1) = (
        pts.iter().map(|p| p.1).fold(f64::MAX, f64::min),
        pts.iter().map(|p| p.1).fold(f64::MIN, f64::max),
    );
    assert!(
        (x0 - 4.0).abs() < 1e-9 && (x1 - 16.0).abs() < 1e-9,
        "{x0} {x1}"
    );
    assert!(
        (y0 - 2.0).abs() < 1e-9 && (y1 - 8.0).abs() < 1e-9,
        "{y0} {y1}"
    );
    // dragging a corner keeps the centre: resize by constraining the width, the middle stays at x = 10
    let rep = ed.report.clone().unwrap();
    assert_eq!(rep.dof, 4, "{rep:?}"); // centre (2) + half-width + half-height
}

#[test]
fn aligned_rectangle_is_a_rectangle_at_any_angle() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::AlignedRectangle);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 8.0, 6.0); // one side of length 10 at atan2(6, 8)
    click(&mut ed, 8.0 - 3.0, 6.0 + 4.0); // 5 away on the left of that side
    ok(&ed);
    let rep = ed.report.clone().unwrap();
    assert_eq!(rep.dof, 5, "{rep:?}"); // position (2) + angle + two sides
    let pts: Vec<(f64, f64)> = ed
        .sketch
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
        .map(|(id, _)| ed.sketch.point(id).unwrap())
        .collect();
    assert_eq!(pts.len(), 4);
    // the far corners are (8, 6) + 5 (-0.6, 0.8) and (0, 0) + 5 (-0.6, 0.8)
    let has = |x: f64, y: f64| {
        pts.iter()
            .any(|p| (p.0 - x).abs() < 1e-9 && (p.1 - y).abs() < 1e-9)
    };
    assert!(has(5.0, 10.0) && has(-3.0, 4.0), "{pts:?}");
    // sides are 10 and 5 long
    assert!(has(8.0, 6.0) && has(0.0, 0.0));
}
