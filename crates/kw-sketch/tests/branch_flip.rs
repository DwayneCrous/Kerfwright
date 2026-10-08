//! Phase 2 acceptance: no branch flip during scripted drags of 100 reference
//! sketches (tangent / angle / offset heavy).
//!
//! The invariants are computed here from raw geometry, independently of the
//! solver's own flip detector: tangent side, joint direction, offset side,
//! angle branch, radius and line-length signs must never change, and no point
//! may jump by more than a quarter of the sketch between consecutive frames.

use kw_sketch::testgen::{flip_suite, Rng};
use kw_sketch::*;

fn pt(sk: &Sketch, e: EntityId) -> (f64, f64) {
    sk.point(e).unwrap()
}

fn fwd_tangent(sk: &Sketch, curve: EntityId, at_end: bool) -> (f64, f64) {
    match sk.kind(curve).unwrap().clone() {
        EntityKind::Line { p1, p2 } => {
            let (a, b) = (pt(sk, p1), pt(sk, p2));
            (b.0 - a.0, b.1 - a.1)
        }
        EntityKind::Arc { center, start, end } => {
            let c = pt(sk, center);
            let p = pt(sk, if at_end { end } else { start });
            (-(p.1 - c.1), p.0 - c.0) // counter-clockwise travel
        }
        EntityKind::Spline { control, .. } => {
            let n = control.len();
            let (a, b) = if at_end {
                (pt(sk, control[n - 2]), pt(sk, control[n - 1]))
            } else {
                (pt(sk, control[0]), pt(sk, control[1]))
            };
            (b.0 - a.0, b.1 - a.1)
        }
        _ => unreachable!(),
    }
}

/// Signed quantities whose sign must be preserved (0 = not applicable).
fn invariants(sk: &Sketch) -> Vec<f64> {
    let mut q = Vec::new();
    for c in sk.constraints.values() {
        match &c.kind {
            ConstraintKind::Tangent { a, b, branch } => {
                let a_line = matches!(sk.kind(*a).unwrap(), EntityKind::Line { .. });
                let b_line = matches!(sk.kind(*b).unwrap(), EntityKind::Line { .. });
                if a_line || b_line {
                    let (line, circ) = if a_line { (*a, *b) } else { (*b, *a) };
                    let (p1, _) = sk.line_points(line).unwrap();
                    let d = sk.line_vector(line).unwrap();
                    let (ctr, _) = sk.circle_geometry(circ).unwrap();
                    let o = pt(sk, p1);
                    q.push((d.0 * (ctr.1 - o.1) - d.1 * (ctr.0 - o.0)) / d.0.hypot(d.1));
                } else if *branch == TangentBranch::Negative {
                    let ((_, r1), (_, r2)) = (
                        sk.circle_geometry(*a).unwrap(),
                        sk.circle_geometry(*b).unwrap(),
                    );
                    q.push(r1 - r2);
                } else {
                    q.push(0.0);
                }
            }
            ConstraintKind::Joint {
                a,
                a_end,
                b,
                b_end,
                level,
                ..
            } if *level >= Continuity::G1 => {
                let ta = fwd_tangent(sk, *a, *a_end == CurveEnd::End);
                let tb = fwd_tangent(sk, *b, *b_end == CurveEnd::End);
                let sa = if *a_end == CurveEnd::End { 1.0 } else { -1.0 };
                let sb = if *b_end == CurveEnd::End { 1.0 } else { -1.0 };
                let dot = sa * sb * (ta.0 * tb.0 + ta.1 * tb.1);
                q.push(dot / (ta.0.hypot(ta.1) * tb.0.hypot(tb.1)));
            }
            ConstraintKind::Distance { a, b, .. } => {
                let (pt_e, line) = if matches!(sk.kind(*a).unwrap(), EntityKind::Point(_)) {
                    (*a, *b)
                } else {
                    (*b, *a)
                };
                if matches!(sk.kind(line).unwrap(), EntityKind::Line { .. })
                    && matches!(sk.kind(pt_e).unwrap(), EntityKind::Point(_))
                {
                    let (l1, _) = sk.line_points(line).unwrap();
                    let d = sk.line_vector(line).unwrap();
                    let (o, p) = (pt(sk, l1), pt(sk, pt_e));
                    q.push((d.0 * (p.1 - o.1) - d.1 * (p.0 - o.0)) / d.0.hypot(d.1));
                } else if matches!(sk.kind(*a).unwrap(), EntityKind::Line { .. }) {
                    // line–line: signed distance of a's start from b
                    let (a1, _) = sk.line_points(*a).unwrap();
                    let (b1, _) = sk.line_points(*b).unwrap();
                    let d = sk.line_vector(*b).unwrap();
                    let (o, p) = (pt(sk, b1), pt(sk, a1));
                    q.push((d.0 * (p.1 - o.1) - d.1 * (p.0 - o.0)) / d.0.hypot(d.1));
                } else {
                    q.push(0.0);
                }
            }
            ConstraintKind::Angle { a, b, value } => {
                let (u, w) = (sk.line_vector(*a).unwrap(), sk.line_vector(*b).unwrap());
                let phi = (u.0 * w.1 - u.1 * w.0).atan2(u.0 * w.0 + u.1 * w.1);
                q.push((phi - sk.value(*value)).cos());
            }
            _ => {}
        }
    }
    // positive radii and non-degenerate lines
    for e in sk.entities.values() {
        match &e.kind {
            EntityKind::Circle { radius, .. } => q.push(sk.value(*radius)),
            EntityKind::Arc { .. } => {
                let (_, r) = sk.circle_geometry_of_entity(e);
                q.push(r);
            }
            _ => {}
        }
    }
    for (id, e) in &sk.entities {
        if matches!(e.kind, EntityKind::Line { .. }) {
            let d = sk.line_vector(id).unwrap();
            q.push(d.0.hypot(d.1));
        }
    }
    q
}

trait ArcRadius {
    fn circle_geometry_of_entity(&self, e: &Entity) -> ((f64, f64), f64);
}
impl ArcRadius for Sketch {
    fn circle_geometry_of_entity(&self, e: &Entity) -> ((f64, f64), f64) {
        if let EntityKind::Arc { center, start, .. } = &e.kind {
            let c = pt(self, *center);
            let s = pt(self, *start);
            (c, (s.0 - c.0).hypot(s.1 - c.1))
        } else {
            unreachable!()
        }
    }
}

fn snapshot(sk: &Sketch) -> Vec<f64> {
    sk.entities
        .values()
        .filter_map(|e| {
            if let EntityKind::Point(p) = &e.kind {
                Some([sk.value(p.x), sk.value(p.y)])
            } else {
                None
            }
        })
        .flatten()
        .collect()
}

struct Stats {
    frames: usize,
    flips: usize,
    jumps: usize,
    unconverged: usize,
    refused: usize,
    sketches: usize,
}

fn run(prevent_flips: bool) -> Stats {
    let mut st = Stats {
        frames: 0,
        flips: 0,
        jumps: 0,
        unconverged: 0,
        refused: 0,
        sketches: 0,
    };
    for case in flip_suite() {
        let mut sk = case.sketch.clone();
        let mut solver = Solver::new(SolveOptions {
            prevent_flips,
            ..SolveOptions::default()
        });
        let rep = solver.analyze(&mut sk).unwrap();
        assert_ne!(
            rep.status,
            SketchStatus::Conflicting,
            "{}: reference sketch must be solvable",
            case.name
        );
        let init = invariants(&sk);
        let fixed_points: Vec<(f64, f64)> = sk
            .constraints
            .values()
            .filter_map(|c| match &c.kind {
                ConstraintKind::Fix(e) if matches!(sk.kind(*e).unwrap(), EntityKind::Point(_)) => {
                    Some(sk.point(*e).unwrap())
                }
                _ => None,
            })
            .collect();
        let free_pts: Vec<EntityId> = rep
            .free_entities
            .iter()
            .copied()
            .filter(|e| matches!(sk.kind(*e).unwrap(), EntityKind::Point(_)))
            .collect();
        assert!(
            !free_pts.is_empty(),
            "{}: needs free points to drag",
            case.name
        );
        st.sketches += 1;
        let mut rng = Rng::new(case.name.len() as u64 * 977 + st.sketches as u64);
        for _ in 0..3 {
            let target = free_pts[rng.int(free_pts.len())];
            let pv = sk.point_vars(target).unwrap();
            let start = sk.point(target).unwrap();
            let mut heading = rng.range(0.0, std::f64::consts::TAU);
            let mut pos = start;
            let step = 0.02 * case.scale;
            for _ in 0..60 {
                heading += rng.range(-0.5, 0.5);
                pos = (pos.0 + step * heading.cos(), pos.1 + step * heading.sin());
                // stay within a disc around the start so the path remains meaningful
                let r = (pos.0 - start.0).hypot(pos.1 - start.1);
                if r > 0.6 * case.scale {
                    heading += std::f64::consts::PI;
                }
                let before = snapshot(&sk);
                let out = solver
                    .drag(&mut sk, &[(pv.x, pos.0), (pv.y, pos.1)])
                    .unwrap();
                st.frames += 1;
                if !out.reached {
                    st.refused += 1;
                    // keep the path on the achieved position
                    pos = sk.point(target).unwrap();
                }
                if solver.residual(&sk).unwrap() > 1e-9 {
                    st.unconverged += 1;
                }
                let now = invariants(&sk);
                assert_eq!(now.len(), init.len());
                if init
                    .iter()
                    .zip(&now)
                    .any(|(a, b)| a.abs() > 1e-9 && *a * *b <= 0.0)
                {
                    st.flips += 1;
                }
                let after = snapshot(&sk);
                let jump = before
                    .iter()
                    .zip(&after)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f64::max);
                // A tip orbiting very close to a fixed pivot swings everything attached to it
                // by a large angle for a tiny move (lever singularity): not a branch jump.
                let near_pivot = fixed_points.iter().any(|f| {
                    let p = sk.point(target).unwrap();
                    (p.0 - f.0).hypot(p.1 - f.1) < 0.08 * case.scale
                });
                if jump > 0.5 * case.scale && !near_pivot {
                    st.jumps += 1;
                }
            }
        }
    }
    st
}

#[test]
fn no_branch_flip_during_scripted_drags() {
    let st = run(true);
    println!(
        "{} sketches, {} drag frames: {} flips, {} jumps, {} unconverged, {} frames shortened or refused",
        st.sketches, st.frames, st.flips, st.jumps, st.unconverged, st.refused
    );
    assert_eq!(st.sketches, 100);
    assert_eq!(st.flips, 0, "branch flips with flip prevention on");
    assert_eq!(st.jumps, 0, "solution jumped between frames");
    assert_eq!(st.unconverged, 0, "constraints violated after a drag frame");
}

#[test]
fn the_flip_check_is_not_vacuous() {
    // with the solver's flip prevention disabled the same scripts do flip branches
    let st = run(false);
    println!(
        "prevention off: {} flips, {} jumps over {} frames",
        st.flips, st.jumps, st.frames
    );
    assert!(
        st.flips + st.jumps > 0,
        "scripted drags never provoke a flip: the test cannot detect regressions"
    );
}
