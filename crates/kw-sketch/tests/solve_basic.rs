use kw_sketch::*;

fn solved(sk: &mut Sketch) -> SolveOutcome {
    let mut s = Solver::default();
    let o = s.solve(sk).unwrap();
    assert_eq!(o.status, SolveStatus::Converged, "{o:?}");
    o
}

#[test]
fn dimensioned_rectangle() {
    let mut sk = Sketch::new();
    let p: Vec<_> = [(0.1, -0.2), (9.0, 0.5), (9.5, 4.0), (-0.3, 3.7)]
        .iter()
        .map(|&(x, y)| sk.add_point(x, y))
        .collect();
    let l: Vec<_> = (0..4).map(|i| sk.add_line(p[i], p[(i + 1) % 4])).collect();
    sk.fix(p[0]).unwrap();
    sk.horizontal(l[0]).unwrap();
    sk.horizontal(l[2]).unwrap();
    sk.vertical(l[1]).unwrap();
    sk.vertical(l[3]).unwrap();
    let w = sk.dim_distance(p[0], p[1]).unwrap();
    let h = sk.dim_distance(p[1], p[2]).unwrap();
    sk.set_dimension(w, 8.0).unwrap();
    sk.set_dimension(h, 3.0).unwrap();
    solved(&mut sk);
    let pt = |e| sk.point(e).unwrap();
    assert!(
        (pt(p[1]).0 - (pt(p[0]).0 + 8.0)).abs() < 1e-9 && (pt(p[1]).1 - pt(p[0]).1).abs() < 1e-9
    );
    assert!((pt(p[2]).0 - pt(p[1]).0).abs() < 1e-9 && (pt(p[2]).1 - pt(p[1]).1 - 3.0).abs() < 1e-9);
    assert!((pt(p[3]).0 - pt(p[0]).0).abs() < 1e-9 && (pt(p[3]).1 - pt(p[2]).1).abs() < 1e-9);
}

#[test]
fn circle_tangent_to_two_lines_with_radius() {
    let mut sk = Sketch::new();
    let a = sk.add_line_xy(0.0, 0.0, 10.0, 0.0);
    let b = sk.add_line_xy(0.0, 0.0, 0.0, 10.0);
    let c = sk.add_circle_xy(2.5, 2.8, 2.0);
    sk.fix(a).unwrap();
    sk.fix(b).unwrap();
    sk.tangent(a, c).unwrap();
    sk.tangent(b, c).unwrap();
    let r = sk.dim_radius(c).unwrap();
    sk.set_dimension(r, 3.0).unwrap();
    solved(&mut sk);
    let (ctr, rad) = sk.circle_geometry(c).unwrap();
    assert!(
        (rad - 3.0).abs() < 1e-9 && (ctr.0 - 3.0).abs() < 1e-8 && (ctr.1 - 3.0).abs() < 1e-8,
        "{ctr:?} {rad}"
    );
}

#[test]
fn arc_line_joint_g1_and_g2() {
    let mut sk = Sketch::new();
    let line = sk.add_line_xy(-5.0, 0.0, 0.0, 0.1);
    let arc = sk.add_arc_xy(0.2, 2.0, 2.0, -1.5, 0.3);
    let (l0, l1) = sk.line_points(line).unwrap();
    sk.fix(l0).unwrap();
    sk.horizontal(line).unwrap();
    let EntityKind::Arc { start, .. } = sk.kind(arc).unwrap().clone() else {
        panic!()
    };
    sk.coincident(l1, start).unwrap();
    sk.joint(line, CurveEnd::End, arc, CurveEnd::Start, Continuity::G1)
        .unwrap();
    let r = sk.dim_radius(arc).unwrap();
    sk.set_dimension(r, 2.0).unwrap();
    solved(&mut sk);
    // tangent continuation of a horizontal line: arc centre sits straight above/below the join
    let (c, rad) = sk.circle_geometry(arc).unwrap();
    let j = sk.point(l1).unwrap();
    assert!(
        (c.0 - j.0).abs() < 1e-8 && ((c.1 - j.1).abs() - rad).abs() < 1e-8,
        "centre {c:?} join {j:?} r {rad}"
    );
}

#[test]
fn conflicting_constraints_do_not_converge() {
    let mut sk = Sketch::new();
    let a = sk.add_point(0.0, 0.0);
    let b = sk.add_point(3.0, 0.1);
    sk.fix(a).unwrap();
    sk.fix(b).unwrap();
    let d = sk.dim_distance(a, b).unwrap();
    sk.set_dimension(d, 10.0).unwrap(); // but they are pinned 3 apart
    let mut s = Solver::default();
    let o = s.solve(&mut sk).unwrap();
    assert_eq!(o.status, SolveStatus::Failed);
}

#[test]
fn drag_moves_a_free_corner_and_respects_constraints() {
    let mut sk = Sketch::new();
    let p0 = sk.add_point(0.0, 0.0);
    let p1 = sk.add_point(4.0, 0.0);
    let p2 = sk.add_point(4.0, 3.0);
    let l01 = sk.add_line(p0, p1);
    let l12 = sk.add_line(p1, p2);
    sk.fix(p0).unwrap();
    sk.horizontal(l01).unwrap();
    sk.vertical(l12).unwrap();
    let mut s = Solver::default();
    solved(&mut sk);
    let v = sk.point_vars(p2).unwrap();
    let out = s.drag(&mut sk, &[(v.x, 6.0), (v.y, 5.0)]).unwrap();
    assert!(out.moved && out.reached, "{out:?}");
    // p1 follows p2.x (vertical), p1.y stays 0 (horizontal), p2 reaches its target
    assert!(
        (sk.point(p2).unwrap().0 - 6.0).abs() < 1e-9
            && (sk.point(p2).unwrap().1 - 5.0).abs() < 1e-9
    );
    assert!((sk.point(p1).unwrap().0 - 6.0).abs() < 1e-9 && sk.point(p1).unwrap().1.abs() < 1e-9);
}
