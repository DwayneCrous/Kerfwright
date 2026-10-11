//! 3+2 setups and swarf milling, through the machine kinematics.

use kw_cam::dexel::TriDexel;
use kw_cam::kin::{angle_between, path_to_axes, test_machines, IkOptions, Pose};
use kw_cam::multiaxis::{flank_deviation, setup_poses, swarf_poses, Frame, Side};
use kw_cam::ops2d::{pocket, Cut};
use kw_cam::{Region, Tool};

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[test]
fn a_three_plus_two_pocket_holds_the_rotary_axes_and_removes_the_same_volume() {
    let region = Region::rect(0.0, 0.0, 30.0, 20.0);
    let tool = Tool::flat(1, 6.0, 20.0);
    let mut cut = Cut::new(0.0, -4.0);
    cut.stepover = 4.0;
    cut.step_down = 4.0;
    let tp = pocket(&region, &tool, &cut);
    let frame = Frame::from_axis([10.0, -5.0, 20.0], [0.35, -0.5, 0.8]);
    let poses = setup_poses(&tp, &frame);
    assert!(poses.len() > 20);
    for m in test_machines() {
        let start = m.inverse(poses[0].tip, poses[0].axis, &[0.0; 5])[0]
            .q
            .clone();
        let blocks = path_to_axes(&m, &poses, &start, &IkOptions::default());
        // 3+2: the tool axis is fixed, so the rotary axes do not move after the first block
        let rot = m.rotary_indices();
        for b in &blocks {
            for &i in &rot {
                assert!(
                    (b.q[i] - blocks[0].q[i]).abs() < 1e-9,
                    "{}: rotary moved",
                    m.name
                );
            }
            // the machine reaches every tip
            let got = m.forward(&b.q);
            assert!(dist(got.tip, b.want_tip) < 1e-9, "{}", m.name);
            assert!(angle_between(got.axis, frame.w) < 1e-9, "{}", m.name);
        }
        assert_eq!(blocks.len(), poses.len(), "{}: no splitting needed", m.name);
    }
    // simulate in the setup frame: the volume removed equals that of an untilted pocket (the stock is a block
    // in the frame; tilting a block does not change what a vertical tool removes from it)
    let mut d = TriDexel::block([-10.0, -10.0, -10.0], [40.0, 30.0, 0.0], 0.1);
    d.run(&tool, &tp);
    let removed = d.initial_volume() - d.volume()[0];
    let (w, h, r) = (20.0, 10.0, 3.0);
    // tool-centre rectangle is the region shrunk by the radius: 24 x 14
    let _ = (w, h);
    let (cw, ch) = (30.0 - 2.0 * r, 20.0 - 2.0 * r);
    let want = (cw * ch + 2.0 * r * (cw + ch) + std::f64::consts::PI * r * r) * 4.0;
    assert!((removed - want).abs() / want < 0.005, "{removed} vs {want}");
}

fn circle(r: f64, z: f64, n: usize, a0: f64, a1: f64) -> Vec<[f64; 3]> {
    (0..n)
        .map(|i| {
            let a = a0 + (a1 - a0) * i as f64 / (n - 1) as f64;
            [r * a.cos(), r * a.sin(), z]
        })
        .collect()
}

#[test]
fn swarf_on_a_cone_is_exact_and_the_machines_can_follow_it() {
    // an inclined cone wall: radius 30 at z = 0 to radius 22 at z = 25, a quarter turn
    let (a0, a1) = (0.2, 0.2 + std::f64::consts::FRAC_PI_2);
    let bottom = circle(30.0, 0.0, 200, a0, a1);
    let top = circle(22.0, 25.0, 200, a0, a1);
    let r = 3.0;
    // tool inside the cone wall: normal pointing to the axis
    let poses = swarf_poses(&bottom, &top, r, Side::Left);
    let dev_left = flank_deviation(&bottom, &top, &poses, r, 4);
    let poses_r = swarf_poses(&bottom, &top, r, Side::Right);
    let dev_right = flank_deviation(&bottom, &top, &poses_r, r, 4);
    eprintln!("cone swarf: flank deviation left {dev_left:.2e} mm, right {dev_right:.2e} mm");
    assert!(dev_left < 5e-3 && dev_right < 5e-3);
    // one side puts the tool inside the cone, the other outside: the tips are at radius 30 ∓ r (projected)
    let radial = |p: &Pose| (p.tip[0].powi(2) + p.tip[1].powi(2)).sqrt();
    let inner = radial(&poses[100]).min(radial(&poses_r[100]));
    let outer = radial(&poses[100]).max(radial(&poses_r[100]));
    assert!(outer > 30.0 && inner < 30.0, "{inner} {outer}");
    let opt = IkOptions::default();
    for m in test_machines() {
        let start = m
            .inverse(poses[0].tip, poses[0].axis, &[0.0; 5])
            .first()
            .map(|s| s.q.clone())
            .expect("a start");
        let blocks = path_to_axes(&m, &poses, &start, &opt);
        let mut worst = 0.0f64;
        for b in &blocks {
            let got = m.forward(&b.q);
            worst = worst
                .max(dist(got.tip, b.want_tip))
                .max(angle_between(got.axis, b.want_axis) * 100.0);
        }
        assert!(worst < 1e-3, "{}: {worst}", m.name);
    }
}

#[test]
fn swarf_on_a_twisted_ruled_surface_reports_its_flank_error() {
    // a hyperbolic (non-developable) ruled surface: the straight tool flank cannot follow it exactly, and the
    // deviation function says by how much
    let n = 120;
    let bottom: Vec<[f64; 3]> = (0..n).map(|i| [i as f64 * 0.5, 0.0, 0.0]).collect();
    let top: Vec<[f64; 3]> = (0..n)
        .map(|i| {
            let x = i as f64 * 0.5;
            [x, 8.0 * (x / 60.0 * std::f64::consts::PI).sin(), 20.0]
        })
        .collect();
    let r = 3.0;
    let poses = swarf_poses(&bottom, &top, r, Side::Left);
    let dev = flank_deviation(&bottom, &top, &poses, r, 4);
    eprintln!("twisted surface swarf: flank deviation {dev:.3} mm");
    assert!(
        dev > 0.05,
        "a twisted surface cannot be flank-milled exactly"
    );
    assert!(dev < 3.0);
}
