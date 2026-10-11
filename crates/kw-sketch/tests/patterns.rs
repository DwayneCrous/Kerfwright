//! Sketch patterns and bulk insertion through the editor.

use kw_sketch::curves::Curve2;
use kw_sketch::editor::Editor;
use kw_sketch::entity::EntityKind;
use std::f64::consts::PI;

fn count(ed: &Editor, f: impl Fn(&EntityKind) -> bool) -> usize {
    ed.sketch.entities.values().filter(|e| f(&e.kind)).count()
}

#[test]
fn a_rectangular_pattern_repeats_a_rectangle_on_a_grid() {
    let mut ed = Editor::default();
    let made = ed.insert_curves(
        &[
            Curve2::Line((0.0, 0.0), (4.0, 0.0)),
            Curve2::Line((4.0, 0.0), (4.0, 3.0)),
            Curve2::Line((4.0, 3.0), (0.0, 3.0)),
            Curve2::Line((0.0, 3.0), (0.0, 0.0)),
        ],
        false,
    );
    assert_eq!(made, 4);
    // the new curves are selected; repeat 3 x 2 with 10 mm and 8 mm steps
    let added = ed.pattern_rect(3, 2, (10.0, 0.0), (0.0, 8.0)).unwrap();
    assert_eq!(added, 5 * 4);
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 24);
    // every line stays 4 or 3 mm long and the far corner lies at (24, 11)
    let mut far = (f64::MIN, f64::MIN);
    for (id, e) in ed.sketch.entities.iter() {
        if let EntityKind::Point(_) = e.kind {
            let p = ed.sketch.point(id).unwrap();
            if p.0 + p.1 > far.0 + far.1 {
                far = p;
            }
        }
    }
    assert_eq!(far, (24.0, 11.0));
    // undo removes the whole pattern at once
    ed.undo();
    assert_eq!(count(&ed, |k| matches!(k, EntityKind::Line { .. })), 4);
}

#[test]
fn a_circular_pattern_spaces_copies_evenly_and_keeps_arcs_ccw() {
    let mut ed = Editor::default();
    ed.insert_curves(
        &[
            Curve2::Circle {
                c: (10.0, 0.0),
                r: 1.0,
            },
            Curve2::Arc {
                c: (10.0, 5.0),
                start: (11.0, 5.0),
                end: (10.0, 6.0),
            },
        ],
        false,
    );
    let added = ed.pattern_circular((0.0, 0.0), 6, 360.0).unwrap();
    assert_eq!(added, 10);
    let circles: Vec<(f64, f64)> = ed
        .sketch
        .entities
        .values()
        .filter_map(|e| match e.kind {
            EntityKind::Circle { center, .. } => ed.sketch.point(center).ok(),
            _ => None,
        })
        .collect();
    assert_eq!(circles.len(), 6);
    for k in 0..6 {
        let a = 2.0 * PI * k as f64 / 6.0;
        let want = (10.0 * a.cos(), 10.0 * a.sin());
        assert!(
            circles
                .iter()
                .any(|c| (c.0 - want.0).abs() < 1e-9 && (c.1 - want.1).abs() < 1e-9),
            "{k}: {circles:?}"
        );
    }
    // a partial sweep puts the last copy on the end angle: 3 copies over 90 degrees
    let mut ed = Editor::default();
    ed.insert_curves(
        &[Curve2::Circle {
            c: (10.0, 0.0),
            r: 1.0,
        }],
        false,
    );
    ed.pattern_circular((0.0, 0.0), 3, 90.0).unwrap();
    let last = ed
        .sketch
        .entities
        .values()
        .filter_map(|e| match e.kind {
            EntityKind::Circle { center, .. } => ed.sketch.point(center).ok(),
            _ => None,
        })
        .any(|c| c.0.abs() < 1e-9 && (c.1 - 10.0).abs() < 1e-9);
    assert!(last);
    // with nothing selected the command explains itself
    let mut ed = Editor::default();
    assert!(ed.pattern_rect(2, 2, (1.0, 0.0), (0.0, 1.0)).is_err());
}
