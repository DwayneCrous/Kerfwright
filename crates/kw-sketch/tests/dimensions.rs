//! The dimension tool reads its selection the way common parametric CAD does, and makes reference (driven) dimensions
//! when the sketch is already determined.

use kw_sketch::constraint::ConstraintKind;
use kw_sketch::editor::*;
use kw_sketch::*;
use std::f64::consts::PI;

fn click(ed: &mut Editor, x: f64, y: f64) {
    ed.handle(Input::Click((x, y)));
}

fn rectangle(w: f64, h: f64) -> Editor {
    let mut ed = Editor::default();
    ed.set_tool(Tool::Rectangle);
    click(&mut ed, 0.0, 0.0);
    click(&mut ed, w, h);
    ed.set_tool(Tool::Dimension);
    ed
}

fn last(ed: &Editor) -> ConstraintId {
    ed.sketch.constraints.keys().last().unwrap()
}

fn dims(ed: &Editor) -> Vec<ConstraintId> {
    ed.sketch
        .constraints
        .iter()
        .filter(|(id, _)| ed.sketch.dimension_var(*id).is_some())
        .map(|(id, _)| id)
        .collect()
}

#[test]
fn one_line_gives_its_length_and_a_new_value_resizes_it() {
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 0.0); // the bottom side: waits for a second pick
    assert_eq!(dims(&ed).len(), 0);
    click(&mut ed, 5.0, -8.0); // empty space places its length
    let c = last(&ed);
    assert!(matches!(
        ed.sketch.constraints[c].kind,
        ConstraintKind::Distance { .. }
    ));
    assert!((ed.sketch.dimension_value(c).unwrap() - 10.0).abs() < 1e-9);
    assert!(!ed.sketch.constraints[c].driven);
    assert!(ed.edit_dimension(c, 14.0));
    assert!((ed.sketch.dimension_value(c).unwrap() - 14.0).abs() < 1e-9);
    // the side really is 14 long now
    let r = ed.sketch.measure(c).unwrap();
    assert!((r - 14.0).abs() < 1e-6, "{r}");
}

#[test]
fn a_circle_gets_a_diameter_and_an_arc_a_radius() {
    let mut ed = Editor::default();
    ed.sketch.add_circle_xy(0.0, 0.0, 4.0);
    ed.sketch.add_arc_xy(20.0, 0.0, 5.0, 0.0, PI / 2.0);
    ed.set_tool(Tool::Dimension);
    click(&mut ed, 4.0, 0.0);
    let c = last(&ed);
    assert!(matches!(
        ed.sketch.constraints[c].kind,
        ConstraintKind::Diameter { .. }
    ));
    assert!((ed.sketch.dimension_value(c).unwrap() - 8.0).abs() < 1e-9);
    click(
        &mut ed,
        20.0 + 5.0 * (PI / 4.0).cos(),
        5.0 * (PI / 4.0).sin(),
    );
    let c = last(&ed);
    assert!(matches!(
        ed.sketch.constraints[c].kind,
        ConstraintKind::Radius { .. }
    ));
    assert!((ed.sketch.dimension_value(c).unwrap() - 5.0).abs() < 1e-9);
}

#[test]
fn two_parallel_lines_give_their_distance_and_two_meeting_lines_their_angle() {
    // the rectangle's top and bottom: parallel, 6 apart
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 5.0, 6.0);
    let c = last(&ed);
    assert!(matches!(
        ed.sketch.constraints[c].kind,
        ConstraintKind::Distance { .. }
    ));
    assert!((ed.sketch.dimension_value(c).unwrap() - 6.0).abs() < 1e-9);
    // two lines that meet: bottom and left side, 90 degrees
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 0.0, 3.0);
    let c = last(&ed);
    assert!(matches!(
        ed.sketch.constraints[c].kind,
        ConstraintKind::Angle { .. }
    ));
    assert!((ed.sketch.dimension_value(c).unwrap().abs() - PI / 2.0).abs() < 1e-9);
}

#[test]
fn a_point_and_a_line_give_the_perpendicular_distance() {
    let mut ed = Editor::default();
    ed.sketch.add_line_xy(0.0, 0.0, 10.0, 0.0);
    let p = ed.sketch.add_point(4.0, 7.0);
    let _ = p;
    ed.set_tool(Tool::Dimension);
    click(&mut ed, 4.0, 7.0); // the point
    click(&mut ed, 8.0, 0.0); // the line
    let c = last(&ed);
    assert!(matches!(
        ed.sketch.constraints[c].kind,
        ConstraintKind::Distance { .. }
    ));
    assert!((ed.sketch.dimension_value(c).unwrap() - 7.0).abs() < 1e-9);
}

#[test]
fn a_dimension_on_a_fully_constrained_sketch_is_driven_and_follows_the_geometry() {
    let mut ed = rectangle(10.0, 6.0);
    // fully dimension it: width, height, and pin the corner
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 5.0, -8.0);
    click(&mut ed, 10.0, 3.0);
    click(&mut ed, 18.0, 3.0);
    let corner = ed
        .sketch
        .entities
        .iter()
        .find(|(id, e)| {
            matches!(e.kind, EntityKind::Point(_)) && ed.sketch.point(*id).unwrap() == (0.0, 0.0)
        })
        .map(|(id, _)| id)
        .unwrap();
    ed.sketch.fix(corner).unwrap();
    ed.refresh();
    assert_eq!(
        ed.report.as_ref().unwrap().status,
        SketchStatus::FullyConstrained
    );
    let before = ed.sketch.constraints.len();
    // the opposite side: already determined, so a reference dimension
    click(&mut ed, 5.0, 6.0);
    click(&mut ed, 5.0, 14.0);
    assert_eq!(ed.sketch.constraints.len(), before + 1);
    let c = last(&ed);
    assert!(ed.sketch.constraints[c].driven);
    assert!((ed.sketch.dimension_value(c).unwrap() - 10.0).abs() < 1e-9);
    assert!(
        ed.last_dimension.is_none(),
        "a reference dimension opens no value box"
    );
    assert_eq!(
        ed.report.as_ref().unwrap().status,
        SketchStatus::FullyConstrained
    );
    // it cannot be edited ...
    assert!(!ed.edit_dimension(c, 99.0));
    // ... but it follows the driving dimension
    let width = dims(&ed)[0];
    assert!(ed.edit_dimension(width, 16.0));
    assert!((ed.sketch.dimension_value(c).unwrap() - 16.0).abs() < 1e-6);
}

#[test]
fn a_dimension_that_would_repeat_what_is_there_becomes_a_reference_dimension() {
    // top and bottom are tied together by the rectangle's constraints: dimensioning both is redundant
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 5.0, -8.0);
    let first = last(&ed);
    assert!(!ed.sketch.constraints[first].driven);
    // the bottom was already dimensioned: dimensioning it again repeats the first one
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 5.0, -8.0);
    let again = last(&ed);
    assert!(
        ed.sketch.constraints[again].driven,
        "{:?}",
        ed.report.as_ref().map(|r| r.status)
    );
    assert!((ed.sketch.dimension_value(again).unwrap() - 10.0).abs() < 1e-9);
    // the solver is not upset by it
    assert_ne!(
        ed.report.as_ref().unwrap().status,
        SketchStatus::Conflicting
    );
}

#[test]
fn dimensions_are_named_in_the_order_they_were_made() {
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 5.0, -8.0);
    click(&mut ed, 10.0, 3.0);
    click(&mut ed, 18.0, 3.0);
    let all = dims(&ed);
    assert_eq!(ed.sketch.dimension_name(all[0]).as_deref(), Some("d1"));
    assert_eq!(ed.sketch.dimension_name(all[1]).as_deref(), Some("d2"));
}

#[test]
fn a_formula_is_kept_with_the_sketch_and_a_plain_number_clears_it() {
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 0.0);
    click(&mut ed, 5.0, -8.0);
    let c = last(&ed);
    assert!(ed.edit_dimension_expr(c, 12.0, Some("Width / 2".into())));
    let var = ed.sketch.dimension_var(c).unwrap().0;
    assert_eq!(
        ed.sketch.exprs.get(&var).map(String::as_str),
        Some("Width / 2")
    );
    assert!((ed.sketch.dimension_value(c).unwrap() - 12.0).abs() < 1e-9);
    assert!(ed.edit_dimension(c, 11.0));
    assert!(ed.sketch.exprs.is_empty());
}

#[test]
fn enter_places_the_length_of_a_selected_line() {
    let mut ed = rectangle(10.0, 6.0);
    click(&mut ed, 5.0, 6.0);
    assert!(dims(&ed).is_empty());
    ed.handle(Input::Finish);
    assert_eq!(dims(&ed).len(), 1);
    let c = last(&ed);
    assert!((ed.sketch.dimension_value(c).unwrap() - 10.0).abs() < 1e-9);
    // Escape drops a selection instead
    click(&mut ed, 0.0, 3.0);
    ed.handle(Input::Escape);
    click(&mut ed, 5.0, -8.0);
    assert_eq!(dims(&ed).len(), 1);
}
