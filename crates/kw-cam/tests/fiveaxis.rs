//! Five-axis programs through the posts of all five dialects and back through the independent
//! interpreters: the replayed axes must reproduce the tool poses through the machine's forward kinematics.

use kw_cam::kin::{path_to_axes, test_machines, to_toolpath, IkOptions, Machine, Pose};
use kw_cam::post::{render, PostConfig};
use kw_cam::Tool;
use kw_gsim::{run, Dialect, MotionKind};

fn dialects() -> Vec<(Dialect, PostConfig)> {
    vec![
        (Dialect::Fanuc, PostConfig::fanuc()),
        (Dialect::Haas, PostConfig::haas()),
        (Dialect::Siemens840D, PostConfig::siemens()),
        (Dialect::Heidenhain, PostConfig::heidenhain()),
        (Dialect::LinuxCnc, PostConfig::linuxcnc()),
    ]
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

/// A wavy five-axis path: the tip follows a sine on a plane while the axis tilts and swings; it passes close
/// to the singular direction once.
fn sample_path() -> Vec<Pose> {
    let n = 160;
    (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let tip = [
                -30.0 + 60.0 * t,
                8.0 * (6.0 * t).sin(),
                -2.0 + 3.0 * (4.0 * t).cos(),
            ];
            let tilt = (28.0 * (3.0 * t).sin()).to_radians();
            let swing = (40.0 * (5.0 * t) - 90.0 + 180.0 * t).to_radians();
            let axis = unit([
                tilt.sin() * swing.cos(),
                tilt.sin() * swing.sin(),
                tilt.cos(),
            ]);
            Pose { tip, axis }
        })
        .collect()
}

fn axes_from(m: &Machine, to: [f64; 3], rot: [f64; 3]) -> Vec<f64> {
    let mut q = vec![0.0; 5];
    q[..3].copy_from_slice(&to);
    for &i in &m.rotary_indices() {
        let letter = m.axis_names[i].chars().next().unwrap();
        q[i] = rot[(letter as u8 - b'A') as usize].to_radians();
    }
    q
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[test]
fn every_machine_and_dialect_replays_to_the_requested_poses() {
    let tool = Tool::ball(1, 6.0, 20.0);
    let opt = IkOptions::default();
    let poses = sample_path();
    let (mut worst_tip, mut worst_axis, mut worst_orient) = (0.0f64, 0.0f64, 0.0f64);
    let mut programs = 0;
    for m in test_machines() {
        let start = m.inverse(poses[0].tip, poses[0].axis, &[0.0; 5])[0]
            .q
            .clone();
        for tcp in [false, true] {
            let (tp, blocks) = to_toolpath(&m, &poses, &start, &opt, tcp, 800.0, &tool);
            for (d, cfg) in dialects() {
                let text = render(&cfg, &tp).unwrap_or_else(|e| panic!("{} {d:?}: {e}", m.name));
                let r = run(d, &text)
                    .unwrap_or_else(|e| panic!("{} {d:?} tcp={tcp}: {e}\n{text}", m.name));
                assert!(r.program_end_seen);
                let moves: Vec<_> = r
                    .motions
                    .iter()
                    .filter(|x| x.kind == MotionKind::Rapid || x.kind == MotionKind::Feed)
                    .collect();
                assert_eq!(
                    moves.len(),
                    blocks.len(),
                    "{} {d:?}: one motion per block",
                    m.name
                );
                if tcp && d != Dialect::LinuxCnc {
                    assert!(
                        moves.iter().all(|x| x.tcp),
                        "{} {d:?}: TCP not active during the moves",
                        m.name
                    );
                } else {
                    // without TCP the controller must not be in a tool-centre-point mode while axes are written
                    assert!(tcp || moves.iter().all(|x| !x.tcp), "{} {d:?}", m.name);
                }
                for (mo, b) in moves.iter().zip(&blocks) {
                    let q = axes_from(&m, mo.to, mo.rot);
                    let pose = if tcp {
                        // the controller computes the linear axes: its X Y Z are the tip, the rotaries give
                        // the orientation
                        let o = m.forward(&q);
                        Pose {
                            tip: mo.to,
                            axis: o.axis,
                        }
                    } else {
                        m.forward(&q)
                    };
                    worst_tip = worst_tip.max(dist(pose.tip, b.want_tip));
                    // the posted axes reproduce the axes of the block (the block itself may differ from the
                    // ideal intermediate orientation of a split move; `orientation_error` reports that)
                    worst_axis = worst_axis.max(dist(pose.axis, m.forward(&b.q).axis));
                    worst_orient = worst_orient.max(b.orientation_error);
                }
                programs += 1;
            }
        }
    }
    eprintln!(
        "{programs} programs replayed: worst tip deviation {worst_tip:.2e} mm, worst axis deviation {worst_axis:.2e}, worst intermediate orientation error {:.3}°",
        worst_orient.to_degrees()
    );
    assert!(worst_tip < 2e-3, "{worst_tip}");
    assert!(worst_axis < 1e-4, "{worst_axis}");
}

#[test]
fn the_ik_blocks_follow_the_poses_and_respect_the_step_cap() {
    let opt = IkOptions::default();
    let poses = sample_path();
    for m in test_machines() {
        let start = m.inverse(poses[0].tip, poses[0].axis, &[0.0; 5])[0]
            .q
            .clone();
        let blocks = path_to_axes(&m, &poses, &start, &opt);
        let mut prev = start.clone();
        for b in &blocks {
            for &i in &m.rotary_indices() {
                assert!(
                    (b.q[i] - prev[i]).abs() <= opt.max_joint_step + 1e-9,
                    "{}",
                    m.name
                );
            }
            prev = b.q.clone();
        }
        // every requested pose is reached exactly by some block
        for p in &poses {
            let hit = blocks
                .iter()
                .any(|b| dist(b.want_tip, p.tip) < 1e-12 && b.orientation_error < 1e-9);
            assert!(hit, "{}: pose {:?} never reached", m.name, p.tip);
        }
    }
}

#[test]
fn a_post_without_the_five_axis_templates_is_refused() {
    let cfg = PostConfig::parse("name = Bare\nrapid = G0 {X} {Y} {Z}\n").unwrap();
    let tool = Tool::flat(1, 6.0, 20.0);
    let m = &test_machines()[0];
    let poses = sample_path();
    let start = m.inverse(poses[0].tip, poses[0].axis, &[0.0; 5])[0]
        .q
        .clone();
    let (tp, _) = to_toolpath(
        m,
        &poses[..3],
        &start,
        &IkOptions::default(),
        false,
        500.0,
        &tool,
    );
    assert!(render(&cfg, &tp).is_err());
}
