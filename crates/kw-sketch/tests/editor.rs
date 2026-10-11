//! Scripted interaction tests for the toolkit-independent editor.

use kw_sketch::editor::*;
use kw_sketch::*;

fn click(ed: &mut Editor, x: f64, y: f64) {
    ed.handle(Input::Click((x, y)));
}

fn count(ed: &Editor, f: fn(&EntityKind) -> bool) -> usize {
    ed.sketch.entities.values().filter(|e| f(&e.kind)).count()
}

#[test]
fn rectangle_tool_builds_a_constrained_rectangle_and_dimensions_fix_it() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Rectangle);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 8.0, 5.0);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 4);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Point(_))), 4);
    assert_eq!(ed.sketch.constraints.len(), 4);

    // dimension the bottom edge, then the right edge
    ed.set_tool(Tool::Dimension);
    click(&mut ed, 4.0, 0.0);
    click(&mut ed, 4.0, -6.0); // empty space places the length
    let w = ed.last_dimension.expect("dimension created");
    assert!((ed.sketch.dimension_value(w).unwrap() - 8.0).abs() < 1e-9);
    click(&mut ed, 8.0, 2.5);
    click(&mut ed, 14.0, 2.5);
    let h = ed.last_dimension.unwrap();
    assert!(ed.edit_dimension(w, 10.0));
    assert!(ed.edit_dimension(h, 4.0));
    // pin the first corner
    // (the minimum-norm solve may have shifted the free rectangle: click where the corner is now)
    let (lo, _) = ed.sketch.bounds().unwrap();
    ed.set_tool(Tool::Constrain(ConstraintTool::Fix));
    click(&mut ed, lo.0, lo.1);
    let rep = ed.report.clone().unwrap();
    assert_eq!(rep.status, SketchStatus::FullyConstrained, "{rep:?}");
    let (lo2, hi) = ed.sketch.bounds().unwrap();
    assert!(
        (hi.0 - lo2.0 - 10.0).abs() < 1e-8 && (hi.1 - lo2.1 - 4.0).abs() < 1e-8,
        "{lo2:?} {hi:?}"
    );
}

#[test]
fn conflicting_constraint_is_rolled_back() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Rectangle);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 6.0, 4.0);
    ed.set_tool(Tool::Constrain(ConstraintTool::Parallel));
    // bottom edge parallel to the right edge contradicts H/V
    click(&mut ed, 3.0, 0.0);
    click(&mut ed, 6.0, 2.0);
    assert!(ed.status.contains("conflict"), "{}", ed.status);
    assert_eq!(
        ed.sketch.constraints.len(),
        4,
        "the rejected constraint is not kept"
    );
}

#[test]
fn line_tool_snaps_infers_horizontal_and_chains() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Line);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 5.0, 0.05); // nearly horizontal → inferred
    click(&mut ed, 5.0, 4.0); // chains from the previous end
    ed.handle(Input::Escape);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 2);
    assert_eq!(
        count(&ed, |k| matches!(k, EntityKind::Point(_))),
        3,
        "chained lines share their joint"
    );
    assert!(ed
        .sketch
        .constraints
        .values()
        .any(|c| matches!(c.kind, ConstraintKind::Horizontal(_))));
    // closing onto the first point reuses it
    ed.set_tool(Tool::Line);
    click(&mut ed, 5.0, 4.0);
    click(&mut ed, 0.1, 0.1);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Point(_))), 3);
}

#[test]
fn circle_arc_spline_and_point_tools() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Circle);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 3.0, 0.0);
    ed.set_tool(Tool::Arc);
    click(&mut ed, 10.0, 0.0);
    click(&mut ed, 12.0, 0.0);
    click(&mut ed, 10.0, 2.0);
    ed.set_tool(Tool::Spline);
    for p in [(0.0, 10.0), (2.0, 12.0), (4.0, 10.0), (6.0, 12.0)] {
        click(&mut ed, p.0, p.1);
    }
    ed.handle(Input::Finish);
    ed.set_tool(Tool::Point);
    click(&mut ed, 20.0, 20.0);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Circle { .. })), 1);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Arc { .. })), 1);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Spline { .. })), 1);
    let arc = ed
        .sketch
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
        .map(|(i, _)| i)
        .unwrap();
    let (_, r, a0, a1) = geom::arc_params(&ed.sketch, arc).unwrap();
    assert!((r - 2.0).abs() < 1e-9 && (a1 - a0 - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
}

#[test]
fn undo_redo_select_delete() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Line);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 4.0, 0.0);
    ed.handle(Input::Escape);
    assert!(ed.can_undo());
    ed.undo();
    ed.undo();
    assert!(ed.sketch.entities.is_empty());
    ed.redo();
    ed.redo();
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 1);
    ed.set_tool(Tool::Select);
    click(&mut ed, 2.0, 0.0);
    assert_eq!(ed.selection.len(), 1);
    ed.handle(Input::Delete);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 0);
    assert_eq!(
        count(&ed, |k| matches!(k, EntityKind::Point(_))),
        0,
        "orphaned points are cleaned up"
    );
}

#[test]
fn trim_and_offset_tools() {
    let mut ed = Editor::default();
    ed.sketch.add_line_xy(0.0, 0.0, 10.0, 0.0);
    ed.sketch.add_line_xy(3.0, -2.0, 3.0, 2.0);
    ed.set_tool(Tool::Trim);
    click(&mut ed, 8.0, 0.0);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 2);
    let (lo, hi) = ed.sketch.bounds().unwrap();
    assert!(
        hi.0 < 10.0 - 1e-9 && lo.0 > -1e-9,
        "the right part was cut off: {hi:?}"
    );

    ed.offset_distance = 2.0;
    ed.set_tool(Tool::Offset);
    click(&mut ed, 1.0, 0.0);
    ed.handle(Input::Finish);
    click(&mut ed, 1.0, 5.0);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 3);
}

#[test]
fn dragging_moves_a_free_point_and_colours_follow_dof() {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Rectangle);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, 6.0, 4.0);
    ed.set_tool(Tool::Constrain(ConstraintTool::Fix));
    click(&mut ed, 0.0, 0.0);
    ed.set_tool(Tool::Select);
    let corner = ed.snap((6.0, 4.0));
    let Snap::Point(p, _) = corner else {
        panic!("expected to snap to the corner")
    };
    assert_eq!(ed.entity_state(p), EntityState::Free);
    ed.begin_drag(p, (6.0, 4.0));
    for k in 1..=10 {
        ed.drag_to((6.0 + 0.3 * k as f64, 4.0 + 0.2 * k as f64));
    }
    ed.end_drag();
    let (x, y) = ed.sketch.point(p).unwrap();
    assert!(
        (x - 9.0).abs() < 1e-6 && (y - 6.0).abs() < 1e-6,
        "({x},{y})"
    );
    // the dragged rectangle is still a rectangle
    let rep = ed.report.clone().unwrap();
    assert_ne!(rep.status, SketchStatus::Conflicting);
    let anchor = ed.snap((0.0, 0.0));
    let Snap::Point(a, _) = anchor else { panic!() };
    assert_eq!(ed.entity_state(a), EntityState::Constrained);
}
