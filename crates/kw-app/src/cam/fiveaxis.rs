//! Programs for machines with rotary axes. The toolpaths of a setup are written in its work coordinate
//! system, with the tool along z (or tilted, for the five-axis operations). A machine with two rotary axes
//! gets them converted: every move becomes a tool pose in the workpiece frame, the inverse kinematics of the
//! machine gives the axis values, and the program is written with tool-centre-point control — which is how
//! a tilted setup (3+2) is cut, too. A three-axis machine gets the paths as they are.

use super::setup::{frame_of, setup_program, P3};
use kw_cam::kin::{angle_between, path_to_axes, IkOptions, Pose};
use kw_cam::library::MachineDef;
use kw_cam::path::{Event, Toolpath};
use kw_store::CamSetup;

/// What the conversion did, for the person who has to run the program.
#[derive(Debug, Clone, Default)]
pub struct FiveAxisReport {
    /// The program was converted to tool-centre-point moves (else it is the paths as they are).
    pub converted: bool,
    /// Moves of the paths, and axis blocks written for them (long rotary moves are split).
    pub moves: usize,
    pub blocks: usize,
    /// Largest angle between the wanted tool axis and the one the axes make at the poses of the path
    /// (degrees).
    pub max_orientation_error: f64,
    /// The same at the extra blocks written between two poses whose orientations differ much: the axes
    /// swing along the joints rather than along the great circle between the two orientations.
    pub interpolation_error: f64,
    /// Largest distance between the wanted tip and the one the axes make (mm).
    pub max_tip_error: f64,
    /// Axes that leave their travel, with the number of blocks that do.
    pub limit_violations: Vec<(String, usize)>,
    /// The travel used by each rotary axis in degrees (name, lowest, highest).
    pub rotary_range: Vec<(String, f64, f64)>,
}

impl FiveAxisReport {
    pub fn ok(&self) -> bool {
        self.limit_violations.is_empty() && self.max_orientation_error < 0.01
    }

    pub fn summary(&self) -> String {
        if !self.converted {
            return "three-axis moves".into();
        }
        let mut s = format!(
            "{} moves in {} blocks, tool axis within {:.4}° at the poses (up to {:.1}° between them), tip within {:.4} mm",
            self.moves,
            self.blocks,
            self.max_orientation_error,
            self.interpolation_error,
            self.max_tip_error
        );
        for (n, lo, hi) in &self.rotary_range {
            s.push_str(&format!("; {n} {lo:.1}° to {hi:.1}°"));
        }
        for (n, c) in &self.limit_violations {
            s.push_str(&format!("; {n} beyond its travel in {c} block(s)"));
        }
        s
    }
}

fn is_vertical(a: P3) -> bool {
    a[0].abs() < 1e-6 && a[1].abs() < 1e-6 && a[2] > 0.0
}

/// Does the setup need rotary axes: a tilted work coordinate system, or tilted tool poses in its paths?
pub fn needs_rotary(setup: &CamSetup) -> bool {
    let f = frame_of(&setup.wcs);
    !is_vertical(f.w)
        || setup.operations.iter().any(|o| {
            o.path
                .events
                .iter()
                .any(|e| matches!(e, Event::Pose { axis, .. } if !is_vertical(*axis)))
        })
}

fn rot3(f: &kw_cam::multiaxis::Frame, v: P3) -> P3 {
    [
        f.u[0] * v[0] + f.v[0] * v[1] + f.w[0] * v[2],
        f.u[1] * v[0] + f.v[1] * v[1] + f.w[1] * v[2],
        f.u[2] * v[0] + f.v[2] * v[1] + f.w[2] * v[2],
    ]
}

/// A move for the conversion: tip and axis in the work coordinate system, and how it is driven.
struct Move {
    tip: P3,
    axis: P3,
    feed: Option<f64>,
}

/// The program of `setup` for `machine`.
pub fn program(
    setup: &CamSetup,
    machine: &MachineDef,
) -> Result<(Toolpath, FiveAxisReport), String> {
    let base = setup_program(setup);
    let Some(m) = &machine.kinematics else {
        return Ok((flatten_poses(base, setup)?, FiveAxisReport::default()));
    };
    if !needs_rotary(setup) {
        return Ok((flatten_poses(base, setup)?, FiveAxisReport::default()));
    }
    let rot = m.rotary_indices();
    if rot.len() < 2 {
        return Err(format!(
            "{} has {} rotary axis: the conversion needs two",
            machine.name,
            rot.len()
        ));
    }
    let names: [char; 2] = [
        m.axis_names[rot[0]].chars().next().unwrap_or('A'),
        m.axis_names[rot[1]].chars().next().unwrap_or('C'),
    ];
    let frame = frame_of(&setup.wcs);
    let opt = IkOptions::default();
    let mut out = Toolpath::new(&base.name);
    let mut report = FiveAxisReport {
        converted: true,
        ..FiveAxisReport::default()
    };
    let mut range: Vec<(f64, f64)> = vec![(f64::MAX, f64::MIN); rot.len()];
    let mut violations: Vec<usize> = vec![0; m.axis_count()];
    let mut prev_q: Option<Vec<f64>> = None;
    let mut tcp_on = false;
    // where the tool is in work coordinates, for arcs and the first move of a path
    let mut pos: Option<P3> = None;
    let mut axis_now: P3 = [0.0, 0.0, 1.0];
    let mut drill: Option<(f64, f64, Option<f64>, f64)> = None; // depth, retract, peck, feed

    let mut emit = |out: &mut Toolpath,
                    report: &mut FiveAxisReport,
                    tcp_on: &mut bool,
                    prev_q: &mut Option<Vec<f64>>,
                    moves: Vec<Move>|
     -> Result<(), String> {
        for mv in moves {
            report.moves += 1;
            // pose in the workpiece frame (a rotation of the work coordinates; the work offset is the origin)
            let pose = Pose {
                tip: rot3(&frame, mv.tip),
                axis: rot3(&frame, mv.axis),
            };
            let start = match prev_q {
                Some(q) => q.clone(),
                None => {
                    let sols = m.inverse(pose.tip, pose.axis, &vec![0.0; m.axis_count()]);
                    let first = sols
                        .first()
                        .ok_or_else(|| unreachable_msg(&machine.name, pose.axis))?;
                    first.q.clone()
                }
            };
            let blocks = path_to_axes(m, &[pose], &start, &opt);
            if blocks.is_empty() {
                return Err(unreachable_msg(&machine.name, pose.axis));
            }
            if !*tcp_on {
                out.push(Event::Tcp { on: true });
                *tcp_on = true;
            }
            let last = blocks.len() - 1;
            for (bi, b) in blocks.into_iter().enumerate() {
                report.blocks += 1;
                let slot = if bi == last {
                    &mut report.max_orientation_error
                } else {
                    &mut report.interpolation_error
                };
                *slot = slot.max(b.orientation_error.to_degrees());
                report.max_tip_error = report.max_tip_error.max(b.tip_error);
                for (i, (lo, hi)) in m.limits.iter().enumerate() {
                    if b.q[i] < lo - 1e-9 || b.q[i] > hi + 1e-9 {
                        violations[i] += 1;
                    }
                }
                let deg = [b.q[rot[0]].to_degrees(), b.q[rot[1]].to_degrees()];
                for k in 0..2 {
                    range[k].0 = range[k].0.min(deg[k]);
                    range[k].1 = range[k].1.max(deg[k]);
                }
                out.push(match mv.feed {
                    None => Event::Rapid5 {
                        p: b.want_tip,
                        rot: deg,
                        names,
                    },
                    Some(feed) => Event::Feed5 {
                        p: b.want_tip,
                        rot: deg,
                        names,
                        feed,
                    },
                });
                *prev_q = Some(b.q);
            }
        }
        Ok(())
    };

    for e in base.events {
        match e {
            Event::Rapid { p } => {
                emit(
                    &mut out,
                    &mut report,
                    &mut tcp_on,
                    &mut prev_q,
                    vec![Move {
                        tip: p,
                        axis: [0.0, 0.0, 1.0],
                        feed: None,
                    }],
                )?;
                pos = Some(p);
                axis_now = [0.0, 0.0, 1.0];
            }
            Event::Feed { p, feed } => {
                emit(
                    &mut out,
                    &mut report,
                    &mut tcp_on,
                    &mut prev_q,
                    vec![Move {
                        tip: p,
                        axis: [0.0, 0.0, 1.0],
                        feed: Some(feed),
                    }],
                )?;
                pos = Some(p);
                axis_now = [0.0, 0.0, 1.0];
            }
            Event::Pose { p, axis, feed } => {
                emit(
                    &mut out,
                    &mut report,
                    &mut tcp_on,
                    &mut prev_q,
                    vec![Move { tip: p, axis, feed }],
                )?;
                pos = Some(p);
                axis_now = axis;
            }
            Event::Arc {
                p,
                centre,
                ccw,
                feed,
            } => {
                let q = pos.ok_or("an arc without a start")?;
                let r = (q[0] - centre[0]).hypot(q[1] - centre[1]);
                let a0 = (q[1] - centre[1]).atan2(q[0] - centre[0]);
                let a1 = (p[1] - centre[1]).atan2(p[0] - centre[0]);
                let mut sweep = a1 - a0;
                if ccw {
                    while sweep <= 1e-12 {
                        sweep += std::f64::consts::TAU;
                    }
                } else {
                    while sweep >= -1e-12 {
                        sweep -= std::f64::consts::TAU;
                    }
                }
                // chords no more than 2 µm from the arc
                let step = 2.0
                    * (1.0 - 0.002 / r.max(0.01))
                        .clamp(-1.0, 1.0)
                        .acos()
                        .max(1e-3);
                let n = ((sweep.abs() / step).ceil() as usize).clamp(2, 4000);
                let moves = (1..=n)
                    .map(|k| {
                        let t = k as f64 / n as f64;
                        let a = a0 + sweep * t;
                        Move {
                            tip: if k == n {
                                p
                            } else {
                                [
                                    centre[0] + r * a.cos(),
                                    centre[1] + r * a.sin(),
                                    q[2] + (p[2] - q[2]) * t,
                                ]
                            },
                            axis: axis_now,
                            feed: Some(feed),
                        }
                    })
                    .collect();
                emit(&mut out, &mut report, &mut tcp_on, &mut prev_q, moves)?;
                pos = Some(p);
            }
            Event::DrillStart {
                depth,
                retract,
                peck,
                feed,
                ..
            } => drill = Some((depth, retract, peck, feed)),
            Event::DrillAt { x, y } => {
                // a canned cycle is positioned in the machine's own frame: write it out as moves
                let (depth, retract, peck, feed) =
                    drill.ok_or("a drill position without a cycle")?;
                let z = [0.0, 0.0, 1.0];
                let mut moves = vec![Move {
                    tip: [x, y, retract],
                    axis: z,
                    feed: None,
                }];
                let mut at = retract;
                match peck {
                    Some(q) if q > 1e-6 => {
                        let mut cut = retract;
                        while cut > depth + 1e-9 {
                            cut = (cut - q).max(depth);
                            moves.push(Move {
                                tip: [x, y, cut],
                                axis: z,
                                feed: Some(feed),
                            });
                            moves.push(Move {
                                tip: [x, y, retract],
                                axis: z,
                                feed: None,
                            });
                            at = retract;
                            if cut > depth + 1e-9 {
                                moves.push(Move {
                                    tip: [x, y, cut + 0.5],
                                    axis: z,
                                    feed: None,
                                });
                            }
                        }
                    }
                    _ => {
                        moves.push(Move {
                            tip: [x, y, depth],
                            axis: z,
                            feed: Some(feed),
                        });
                        moves.push(Move {
                            tip: [x, y, retract],
                            axis: z,
                            feed: None,
                        });
                    }
                }
                let _ = at;
                emit(&mut out, &mut report, &mut tcp_on, &mut prev_q, moves)?;
                pos = Some([x, y, retract]);
                axis_now = z;
            }
            Event::DrillEnd => drill = None,
            Event::ToolChange { .. } => {
                if tcp_on {
                    out.push(Event::Tcp { on: false });
                    tcp_on = false;
                }
                out.push(e);
            }
            Event::ProgramEnd => {
                if tcp_on {
                    out.push(Event::Tcp { on: false });
                    tcp_on = false;
                }
                out.push(e);
            }
            other => out.push(other),
        }
    }
    for (k, i) in rot.iter().enumerate() {
        if range[k].0 <= range[k].1 {
            report
                .rotary_range
                .push((m.axis_names[*i].clone(), range[k].0, range[k].1));
        }
    }
    report.limit_violations = violations
        .iter()
        .enumerate()
        .filter(|(_, c)| **c > 0)
        .map(|(i, c)| (m.axis_names[i].clone(), *c))
        .collect();
    let _ = angle_between;
    Ok((out, report))
}

fn unreachable_msg(machine: &str, axis: P3) -> String {
    format!(
        "{machine} cannot hold the tool along ({:.3}, {:.3}, {:.3})",
        axis[0], axis[1], axis[2]
    )
}

/// A path for a three-axis machine: poses with the tool upright become ordinary moves; a tilted pose is an
/// error naming the operation.
fn flatten_poses(tp: Toolpath, setup: &CamSetup) -> Result<Toolpath, String> {
    let mut out = Toolpath::new(&tp.name);
    for e in tp.events {
        match e {
            Event::Pose { p, axis, feed } => {
                if !is_vertical(axis) {
                    let name = setup
                        .operations
                        .iter()
                        .find(|o| {
                            o.path
                                .events
                                .iter()
                                .any(|x| matches!(x, Event::Pose { .. }))
                        })
                        .map_or("an operation".to_string(), |o| o.name.clone());
                    return Err(format!(
                        "{name} tilts the tool: choose a machine with rotary axes for this setup"
                    ));
                }
                out.push(match feed {
                    None => Event::Rapid { p },
                    Some(feed) => Event::Feed { p, feed },
                });
            }
            other => out.push(other),
        }
    }
    Ok(out)
}
