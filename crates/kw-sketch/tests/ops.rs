use kw_sketch::ops::{extend, offset, trim, TrimOutcome};
use kw_sketch::*;

fn solves(sk: &mut Sketch) {
    let rep = Solver::default().analyze(sk).unwrap();
    assert_ne!(rep.status, SketchStatus::Conflicting, "{rep:?}");
}

#[test]
fn trim_line_between_two_cutters_splits_it() {
    let mut sk = Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 10.0, 0.0);
    let c1 = sk.add_line_xy(3.0, -2.0, 3.0, 2.0);
    let c2 = sk.add_line_xy(7.0, -2.0, 7.0, 2.0);
    match trim(&mut sk, l, (5.0, 0.0)).unwrap() {
        TrimOutcome::Split(a, b) => {
            let (_, a2) = sk.line_points(a).unwrap();
            let (b1, b2) = sk.line_points(b).unwrap();
            assert!((sk.point(a2).unwrap().0 - 3.0).abs() < 1e-9);
            assert!((sk.point(b1).unwrap().0 - 7.0).abs() < 1e-9);
            assert!((sk.point(b2).unwrap().0 - 10.0).abs() < 1e-9);
        }
        other => panic!("{other:?}"),
    }
    let _ = (c1, c2);
    solves(&mut sk);
}

#[test]
fn trim_line_with_one_cutter_shortens_it_and_without_deletes_it() {
    let mut sk = Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 10.0, 0.0);
    sk.add_line_xy(4.0, -1.0, 4.0, 1.0);
    assert_eq!(
        trim(&mut sk, l, (8.0, 0.0)).unwrap(),
        TrimOutcome::Shortened(l)
    );
    let (_, b) = sk.line_points(l).unwrap();
    assert!((sk.point(b).unwrap().0 - 4.0).abs() < 1e-9);
    let lonely = sk.add_line_xy(0.0, 5.0, 3.0, 5.0);
    assert_eq!(
        trim(&mut sk, lonely, (1.0, 5.0)).unwrap(),
        TrimOutcome::Deleted
    );
    assert!(sk.entities.get(lonely).is_none());
}

#[test]
fn trim_circle_between_two_lines_leaves_an_arc() {
    let mut sk = Sketch::new();
    let c = sk.add_circle_xy(0.0, 0.0, 5.0);
    sk.add_line_xy(-10.0, 2.0, 10.0, 2.0);
    sk.add_line_xy(-10.0, -2.0, 10.0, -2.0);
    // click on the right-hand part between the lines
    match trim(&mut sk, c, (5.0, 0.0)).unwrap() {
        TrimOutcome::CircleToArc(arc) => {
            let (_, r, a0, a1) = kw_sketch::geom::arc_params(&sk, arc).unwrap();
            assert!((r - 5.0).abs() < 1e-9);
            // remaining arc is the long way round (more than half the circle)
            assert!(a1 - a0 > std::f64::consts::PI);
        }
        other => panic!("{other:?}"),
    }
    assert!(sk.entities.get(c).is_none());
}

#[test]
fn trim_arc_and_extend_line() {
    let mut sk = Sketch::new();
    let arc = sk.add_arc_xy(0.0, 0.0, 5.0, 0.0, std::f64::consts::PI);
    sk.add_line_xy(-10.0, 3.0, 10.0, 3.0);
    // the line cuts the arc at y = 3; trimming the right-hand piece shortens it
    assert!(matches!(
        trim(&mut sk, arc, (4.9, 0.5)).unwrap(),
        TrimOutcome::Shortened(_)
    ));

    let mut sk = Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 4.0, 0.0);
    let wall = sk.add_line_xy(9.0, -5.0, 9.0, 5.0);
    extend(&mut sk, l, (3.9, 0.0)).unwrap();
    let (_, b) = sk.line_points(l).unwrap();
    assert!((sk.point(b).unwrap().0 - 9.0).abs() < 1e-9);
    let _ = wall;
    solves(&mut sk);
}

#[test]
fn offset_line_circle_and_chain() {
    let mut sk = Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 10.0, 0.0);
    let new = offset(&mut sk, &[l], 2.0, (5.0, 5.0)).unwrap();
    assert_eq!(new.len(), 1);
    let (p, _) = sk.line_points(new[0]).unwrap();
    assert!((sk.point(p).unwrap().1 - 2.0).abs() < 1e-9);
    solves(&mut sk);

    let mut sk = Sketch::new();
    let c = sk.add_circle_xy(0.0, 0.0, 3.0);
    let o = offset(&mut sk, &[c], 1.0, (10.0, 0.0)).unwrap();
    assert!((sk.circle_geometry(o[0]).unwrap().1 - 4.0).abs() < 1e-9);
    let inner = offset(&mut sk, &[c], 1.0, (0.0, 0.0)).unwrap();
    assert!((sk.circle_geometry(inner[0]).unwrap().1 - 2.0).abs() < 1e-9);

    // an L-shaped chain meets at the intersection of the offsets
    let mut sk = Sketch::new();
    let p0 = sk.add_point(0.0, 0.0);
    let p1 = sk.add_point(10.0, 0.0);
    let p2 = sk.add_point(10.0, 8.0);
    let (a, b) = (sk.add_line(p0, p1), sk.add_line(p1, p2));
    let new = offset(&mut sk, &[a, b], 1.5, (5.0, 4.0)).unwrap();
    let (_, a2) = sk.line_points(new[0]).unwrap();
    let (b1, _) = sk.line_points(new[1]).unwrap();
    assert_eq!(a2, b1, "offset corner is shared");
    let c = sk.point(a2).unwrap();
    assert!(
        (c.0 - 8.5).abs() < 1e-9 && (c.1 - 1.5).abs() < 1e-9,
        "{c:?}"
    );
    solves(&mut sk);
}

#[test]
fn remove_entity_cascades_to_dependents() {
    let mut sk = Sketch::new();
    let l = sk.add_line_xy(0.0, 0.0, 1.0, 0.0);
    let (p, _) = sk.line_points(l).unwrap();
    let h = sk.horizontal(l).unwrap();
    sk.remove_entity(p); // the line is built from p, so it goes too
    assert!(sk.entities.get(l).is_none());
    assert!(sk.constraints.get(h).is_none());
    sk.remove_unused_points();
    assert!(sk.entities.is_empty());
}
