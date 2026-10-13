use kw_assembly::*;
use kw_expr::{Param, Quantity};
use kw_feature::kwf::KwfFile;
use kw_feature::{BodyOp, Document, Engine};
use kw_math::Point3;
use std::f64::consts::FRAC_PI_2;

fn mm(v: f64) -> Param {
    Param::length(v)
}

/// A box `w × h × d` with one corner at the origin (x ∈ [0, w], y ∈ [0, h], z ∈ [0, d]).
fn box_part(name: &str, w: f64, h: f64, d: f64, frames: Vec<Frame>) -> Part {
    let mut doc = Document::new();
    let s = doc
        .add_rect_sketch(
            "S",
            kw_feature::PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(w),
            mm(h),
        )
        .unwrap();
    doc.add_extrude("Box", s, mm(d), BodyOp::New).unwrap();
    Part::new(name, doc, frames)
}

fn end(instance: usize, frame: &str) -> JointEnd {
    JointEnd {
        instance,
        frame: frame.into(),
    }
}

/// A joint of `kind` with the given values and limits.
fn joint(
    name: &str,
    kind: JointKind,
    parent: (usize, &str),
    child: (usize, &str),
    values: Vec<Param>,
    limits: Vec<Option<(f64, f64)>>,
) -> Joint {
    let mut j = Joint::new(name, kind, end(parent.0, parent.1), end(child.0, child.1));
    j.values = values;
    j.limits = limits;
    j
}

fn hinge(angle: Param) -> Assembly {
    // base 40 x 20 x 4; door 30 x 20 x 2, hinged along the base's top back edge (y = 0, z = 4)
    let mut a = Assembly::new("Hinged lid");
    let base = a.add_part(box_part(
        "Base",
        40.0,
        20.0,
        4.0,
        // z axis along +x: the hinge axis; x axis = +z
        vec![Frame::new(
            "hinge",
            [0.0, 0.0, 4.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let lid = a.add_part(box_part(
        "Lid",
        40.0,
        20.0,
        2.0,
        vec![Frame::new(
            "hinge",
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let i0 = a.add_instance("base", base);
    let i1 = a.add_instance("lid", lid);
    a.ground = i0;
    a.add_joint(joint(
        "hinge",
        JointKind::Revolute,
        (i0, "hinge"),
        (i1, "hinge"),
        vec![angle],
        vec![Some((0.0, std::f64::consts::PI))],
    ));
    a
}

fn close(a: Point3<f64>, b: Point3<f64>) -> bool {
    (a - b).norm() < 1e-9
}

#[test]
fn a_hinge_places_the_lid_exactly() {
    // closed: the lid sits on the base
    let a = hinge(Param::angle_deg(0.0));
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(0.0, 0.0, 0.0)),
        Point3::new(0.0, 0.0, 4.0)
    ));
    assert!(close(
        t[1].apply_point(&Point3::new(40.0, 20.0, 2.0)),
        Point3::new(40.0, 20.0, 6.0)
    ));
    // opened by 90 degrees about the hinge (the +x axis): the lid's far edge swings up
    let a = hinge(Param::angle_deg(90.0));
    let t = a.solve().unwrap();
    // lid point (x, y, z) → hinge-frame (x, y, z) → rotation about x by +90°: (x, -z, y) → + (0,0,4)
    assert!(close(
        t[1].apply_point(&Point3::new(10.0, 20.0, 0.0)),
        Point3::new(10.0, 0.0, 24.0)
    ));
    assert!(close(
        t[1].apply_point(&Point3::new(10.0, 0.0, 2.0)),
        Point3::new(10.0, -2.0, 4.0)
    ));
}

#[test]
fn joint_values_can_be_parameters_and_limits_are_enforced() {
    let mut a = hinge(Param::expr("open * 2").unwrap());
    a.params.push(("open".into(), Param::angle_deg(30.0)));
    assert!(a.solve().is_ok());
    a.params[0].1 = Param::angle_deg(100.0); // 200° > 180° limit
    let err = a.solve().unwrap_err();
    assert!(matches!(err, AssemblyError::OutOfLimits { .. }), "{err}");
    // a length where an angle belongs is explained
    let bad = hinge(mm(5.0));
    assert!(matches!(
        bad.solve().unwrap_err(),
        AssemblyError::Expression { .. }
    ));
}

#[test]
fn loops_are_refused_with_an_explanation_and_loose_components_float() {
    let mut a = hinge(Param::angle_deg(10.0));
    let extra = a.add_instance("second lid", 1);
    // not joined to anything: it floats where it was put
    a.instances[extra].at = Placement {
        t: [7.0, 8.0, 9.0],
        ..Default::default()
    };
    let t = a.solve().unwrap();
    assert!(close(
        t[extra].apply_point(&Point3::new(1.0, 1.0, 1.0)),
        Point3::new(8.0, 9.0, 10.0)
    ));
    // join it to the base, then close a loop through the first lid
    a.add_joint(joint(
        "second hinge",
        JointKind::Rigid,
        (0, "hinge"),
        (extra, "hinge"),
        vec![],
        vec![],
    ));
    // now the joint places it, not `at`
    let t = a.solve().unwrap();
    assert!(close(
        t[extra].apply_point(&Point3::new(0.0, 0.0, 0.0)),
        Point3::new(0.0, 0.0, 4.0)
    ));
    a.add_joint(joint(
        "closing joint",
        JointKind::Rigid,
        (1, "hinge"),
        (extra, "hinge"),
        vec![],
        vec![],
    ));
    // the lid is open 10 degrees and the copy is not: a rigid joint between them cannot close
    let err = a.solve().unwrap_err();
    assert!(
        matches!(&err, AssemblyError::LoopNotClosed { joint, free: 0, .. } if joint == "closing joint"),
        "{err}"
    );
    assert!(err.to_string().contains("loop"));
    // with the lid shut the loop closes by itself (the joints agree)
    a.joints[0].values = vec![Param::angle_deg(0.0)];
    assert!(a.solve().is_ok());
}

// ------------------------------------------------------------------------------- kinematic loops

/// A part that is a thin bar with named frames (z out of the plane, x along the bar).
fn bar(name: &str, frames: &[(&str, [f64; 3])]) -> Part {
    box_part(
        name,
        1.0,
        1.0,
        1.0,
        frames
            .iter()
            .map(|(n, o)| Frame::new(n, *o, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]))
            .collect(),
    )
}

fn angle_of(p: &Param) -> f64 {
    p.eval(&std::collections::HashMap::new())
        .unwrap()
        .expect(kw_expr::Dim::ANGLE)
        .unwrap()
}

fn wrap(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

/// A planar four-bar: ground pivots at (0,0) and (d,0), crank `a`, coupler `b`, rocker `c`. The crank is
/// the driven input; the other joints are found by the loop solve.
fn four_bar(a: f64, b: f64, c: f64, d: f64) -> Assembly {
    let mut m = Assembly::new("Four-bar");
    let ground = m.add_part(bar("Ground", &[("g0", [0.0; 3]), ("g1", [d, 0.0, 0.0])]));
    let crank = m.add_part(bar("Crank", &[("o", [0.0; 3]), ("tip", [a, 0.0, 0.0])]));
    let coupler = m.add_part(bar("Coupler", &[("p", [0.0; 3]), ("q", [b, 0.0, 0.0])]));
    let rocker = m.add_part(bar("Rocker", &[("s", [0.0; 3]), ("t", [c, 0.0, 0.0])]));
    let g = m.add_instance("ground", ground);
    let k = m.add_instance("crank", crank);
    let p = m.add_instance("coupler", coupler);
    let r = m.add_instance("rocker", rocker);
    m.ground = g;
    let rev = |n: &str, pa: (usize, &str), ch: (usize, &str)| {
        joint(
            n,
            JointKind::Revolute,
            pa,
            ch,
            vec![Param::angle_deg(0.0)],
            vec![None],
        )
    };
    m.add_joint(rev("input", (g, "g0"), (k, "o")));
    m.add_joint(rev("elbow", (k, "tip"), (p, "p")));
    m.add_joint(rev("wrist", (p, "q"), (r, "t")));
    // this one closes the loop
    m.add_joint(rev("pivot", (g, "g1"), (r, "s")));
    m
}

/// The closed-form solution of the four-bar for a crank angle: (coupler angle, rocker angle) in the world.
fn four_bar_angles(a: f64, b: f64, c: f64, d: f64, theta: f64, up: bool) -> (f64, f64) {
    let (ax, ay) = (a * theta.cos(), a * theta.sin());
    let (vx, vy) = (d - ax, -ay);
    let l = (vx * vx + vy * vy).sqrt();
    let x = (b * b - c * c + l * l) / (2.0 * l);
    let h = (b * b - x * x).sqrt() * if up { 1.0 } else { -1.0 };
    let (bx, by) = (ax + (x * vx - h * vy) / l, ay + (x * vy + h * vx) / l);
    ((by - ay).atan2(bx - ax), by.atan2(bx - d))
}

#[test]
fn a_four_bar_closes_its_loop_and_follows_the_closed_form_solution() {
    let (a, b, c, d) = (40.0, 80.0, 60.0, 70.0);
    let mut m = four_bar(a, b, c, d);
    // the loop's joints are solved for, the input is not
    let solved: Vec<bool> = m.joints.iter().map(|j| j.solved).collect();
    assert_eq!(solved, vec![false, true, true, true]);
    // start near the branch we want
    let theta0 = 30f64.to_radians();
    let (psi, phi) = four_bar_angles(a, b, c, d, theta0, true);
    m.joints[0].values = vec![Param::Lit(Quantity::angle(theta0))];
    m.joints[1].values = vec![Param::Lit(Quantity::angle(psi - theta0 + 0.05))];
    m.joints[2].values = vec![Param::Lit(Quantity::angle(phi - psi - 0.05))];
    m.joints[3].values = vec![Param::Lit(Quantity::angle(phi + 0.05))];
    let mut prev = None;
    for deg in (30..=150).step_by(10) {
        let theta = (deg as f64).to_radians();
        m.joints[0].values = vec![Param::Lit(Quantity::angle(theta))];
        let t = m.settle().unwrap();
        let (psi, phi) = four_bar_angles(a, b, c, d, theta, true);
        // the coupler and rocker joints really meet, and sit where the closed form puts them
        let q = t[2].apply_point(&Point3::new(b, 0.0, 0.0));
        let tip = t[3].apply_point(&Point3::new(c, 0.0, 0.0));
        assert!((q - tip).norm() < 1e-6, "{deg}: {}", (q - tip).norm());
        let rocker_end = Point3::new(d + c * phi.cos(), c * phi.sin(), 0.0);
        assert!(
            (tip - rocker_end).norm() < 1e-5,
            "{deg}: {tip} vs {rocker_end}"
        );
        let v = angle_of(&m.joints[3].values[0]);
        assert!(wrap(v - phi).abs() < 1e-6, "{deg}: {v} vs {phi}");
        let elbow = angle_of(&m.joints[1].values[0]);
        assert!(wrap(elbow - (psi - theta)).abs() < 1e-6);
        // one step of the input moves the output a little: the same branch is followed
        if let Some(p) = prev {
            assert!(wrap(v - p).abs() < 0.5);
        }
        prev = Some(v);
    }
    // a four-bar has one degree of freedom (three of its six closure equations are independent)
    let mob = m.mobility().unwrap();
    assert_eq!(mob.dof, 1, "{mob:?}");
    assert_eq!((mob.variables, mob.equations, mob.rank), (4, 6, 3));
    assert_eq!((mob.driven, mob.solved), (1, 3));
}

#[test]
fn a_sweep_follows_one_branch_and_leaves_the_assembly_as_it_was() {
    let (a, b, c, d) = (40.0, 80.0, 60.0, 70.0);
    let mut m = four_bar(a, b, c, d);
    let theta0 = 20f64.to_radians();
    let (psi, phi) = four_bar_angles(a, b, c, d, theta0, true);
    m.joints[0].values = vec![Param::Lit(Quantity::angle(theta0))];
    m.joints[1].values = vec![Param::Lit(Quantity::angle(psi - theta0))];
    m.joints[2].values = vec![Param::Lit(Quantity::angle(phi - psi))];
    m.joints[3].values = vec![Param::Lit(Quantity::angle(phi))];
    m.settle().unwrap();
    let before = m.joints.clone();
    let (steps, stopped) = m.sweep(0, 0, theta0, 170f64.to_radians(), 60).unwrap();
    assert!(stopped.is_none(), "{stopped:?}");
    assert_eq!(steps.len(), 61);
    // each step is a valid, closed pose and the rocker never jumps
    let mut last = f64::NAN;
    for st in &steps {
        let (_, phi) = four_bar_angles(a, b, c, d, st.value, true);
        let tip = st.placed[3].apply_point(&Point3::new(c, 0.0, 0.0));
        assert!((tip - Point3::new(d + c * phi.cos(), c * phi.sin(), 0.0)).norm() < 1e-5);
        let ang = tip.y.atan2(tip.x - d);
        if last.is_finite() {
            assert!(wrap(ang - last).abs() < 0.2);
        }
        last = ang;
    }
    assert_eq!(m.joints, before, "a sweep puts the joint values back");
    // a solved joint cannot be swept
    assert!(m.sweep(2, 0, 0.0, 1.0, 4).is_err());
}

#[test]
fn a_slider_crank_puts_the_piston_where_the_closed_form_says() {
    let (r, l) = (30.0, 90.0);
    let mut m = Assembly::new("Slider-crank");
    let ground = m.add_part(Part::new(
        "Ground",
        box_part("g", 1.0, 1.0, 1.0, vec![]).document,
        vec![
            Frame::new("o", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            // the rail: the slider moves along the joint's z, which is the world x here
            Frame::new("rail", [0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ],
    ));
    let crank = m.add_part(bar("Crank", &[("o", [0.0; 3]), ("tip", [r, 0.0, 0.0])]));
    let rod = m.add_part(bar("Rod", &[("p", [0.0; 3]), ("q", [l, 0.0, 0.0])]));
    let piston = m.add_part(Part::new(
        "Piston",
        box_part("p", 1.0, 1.0, 1.0, vec![]).document,
        vec![
            Frame::new("pin", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            Frame::new("slide", [0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ],
    ));
    let g = m.add_instance("ground", ground);
    let k = m.add_instance("crank", crank);
    let o = m.add_instance("rod", rod);
    let p = m.add_instance("piston", piston);
    m.ground = g;
    let rev = |n: &str, pa: (usize, &str), ch: (usize, &str)| {
        joint(
            n,
            JointKind::Revolute,
            pa,
            ch,
            vec![Param::angle_deg(0.0)],
            vec![None],
        )
    };
    m.add_joint(rev("input", (g, "o"), (k, "o")));
    m.add_joint(rev("big end", (k, "tip"), (o, "p")));
    m.add_joint(rev("gudgeon", (o, "q"), (p, "pin")));
    m.add_joint(joint(
        "bore",
        JointKind::Slider,
        (g, "rail"),
        (p, "slide"),
        vec![mm(0.0)],
        vec![None],
    ));
    assert_eq!(
        m.joints.iter().map(|j| j.solved).collect::<Vec<_>>(),
        vec![false, true, true, true]
    );
    // start at top dead centre, where the rod lies along the rail
    m.joints[3].values = vec![mm(r + l)];
    m.joints[2].values = vec![Param::angle_deg(0.0)];
    let mut worst = 0.0f64;
    for deg in (0..=360).step_by(15) {
        let th = (deg as f64).to_radians();
        m.joints[0].values = vec![Param::Lit(Quantity::angle(th))];
        let t = m.settle().unwrap();
        let x = r * th.cos() + (l * l - r * r * th.sin().powi(2)).sqrt();
        let at = t[3].apply_point(&Point3::origin());
        worst = worst.max((at - Point3::new(x, 0.0, 0.0)).norm());
    }
    assert!(worst < 1e-5, "piston error {worst}");
    assert_eq!(m.mobility().unwrap().dof, 1);
}

#[test]
fn a_mechanism_that_cannot_reach_a_position_says_so() {
    // a crank longer than the other links can reach: at 90 degrees the loop cannot close
    let mut m = four_bar(40.0, 30.0, 20.0, 70.0);
    m.joints[1].values = vec![Param::angle_deg(10.0)];
    m.joints[2].values = vec![Param::angle_deg(-10.0)];
    m.joints[3].values = vec![Param::angle_deg(0.0)];
    m.joints[0].values = vec![Param::angle_deg(0.0)];
    assert!(m.settle().is_ok());
    m.joints[0].values = vec![Param::angle_deg(90.0)];
    let err = m.solve().unwrap_err();
    assert!(
        matches!(err, AssemblyError::LoopNotClosed { free, .. } if free == 3),
        "{err}"
    );
    assert!(err.to_string().contains("cannot reach"), "{err}");
    // a failed settle changes nothing
    let before = m.joints.clone();
    assert!(m.settle().is_err());
    assert_eq!(m.joints, before);
}

#[test]
fn limits_of_a_solved_joint_stop_the_mechanism() {
    let (a, b, c, d) = (40.0, 80.0, 60.0, 70.0);
    let mut m = four_bar(a, b, c, d);
    let theta0 = 30f64.to_radians();
    let (psi, phi) = four_bar_angles(a, b, c, d, theta0, true);
    m.joints[0].values = vec![Param::Lit(Quantity::angle(theta0))];
    m.joints[1].values = vec![Param::Lit(Quantity::angle(psi - theta0))];
    m.joints[2].values = vec![Param::Lit(Quantity::angle(phi - psi))];
    m.joints[3].values = vec![Param::Lit(Quantity::angle(phi))];
    // the rocker may only swing a quarter turn either side of where it starts
    m.joints[3].limits = vec![Some((phi - 0.25, phi + 0.25))];
    m.settle().unwrap();
    let (steps, stopped) = m.sweep(0, 0, theta0, 170f64.to_radians(), 70).unwrap();
    assert!(!steps.is_empty() && steps.len() < 71, "{}", steps.len());
    assert!(
        matches!(stopped, Some(AssemblyError::LoopNotClosed { .. })),
        "{stopped:?}"
    );
}

#[test]
fn dragging_any_link_of_a_four_bar_turns_the_input() {
    let (a, b, c, d) = (40.0, 80.0, 60.0, 70.0);
    let mut m = four_bar(a, b, c, d);
    let theta0 = 30f64.to_radians();
    let (psi, phi) = four_bar_angles(a, b, c, d, theta0, true);
    m.joints[0].values = vec![Param::Lit(Quantity::angle(theta0))];
    m.joints[1].values = vec![Param::Lit(Quantity::angle(psi - theta0))];
    m.joints[2].values = vec![Param::Lit(Quantity::angle(phi - psi))];
    m.joints[3].values = vec![Param::Lit(Quantity::angle(phi))];
    m.settle().unwrap();
    // the crank is dragged by its own joint; the coupler and rocker by the input, since their own joints are solved
    assert_eq!(m.drive_joint(1), Some(0));
    assert_eq!(m.drive_joint(2), Some(0));
    assert_eq!(m.drive_joint(3), Some(0));
    assert_eq!(m.drive_joint(0), None);
}

#[test]
fn dragging_the_coupler_of_a_four_bar_follows_the_pointer_along_its_path() {
    let (a, b, c, d) = (40.0, 80.0, 60.0, 70.0);
    let mut m = four_bar(a, b, c, d);
    let theta0 = 30f64.to_radians();
    let (psi, phi) = four_bar_angles(a, b, c, d, theta0, true);
    m.joints[0].values = vec![Param::Lit(Quantity::angle(theta0))];
    m.joints[1].values = vec![Param::Lit(Quantity::angle(psi - theta0))];
    m.joints[2].values = vec![Param::Lit(Quantity::angle(phi - psi))];
    m.joints[3].values = vec![Param::Lit(Quantity::angle(phi))];
    m.settle().unwrap();
    // grab the coupler's far end, where it meets the rocker
    let t = m.solve().unwrap();
    let grab = t[2].apply_point(&Point3::new(b, 0.0, 0.0));
    let mut drag = Drag::begin(&m, 2, [grab.x, grab.y, grab.z], [0.0, 0.0, -1.0]).unwrap();
    assert_eq!(drag.joint(), Some(0), "the pointer turns the input");
    // the pointer goes to where that end is when the crank is at 70 degrees
    let theta = 70f64.to_radians();
    let (psi, _) = four_bar_angles(a, b, c, d, theta, true);
    let target = [
        a * theta.cos() + b * psi.cos(),
        a * theta.sin() + b * psi.sin(),
        0.0,
    ];
    drag.update(&mut m, &ray_through(target, [0.0, 0.0, -1.0]))
        .unwrap();
    let v = angle_of(&m.joints[0].values[0]);
    assert!((v - theta).abs() < 1e-4, "{}°", v.to_degrees());
    let end = m.solve().unwrap()[2].apply_point(&Point3::new(b, 0.0, 0.0));
    assert!((end - Point3::new(target[0], target[1], target[2])).norm() < 1e-3);
}

#[test]
fn a_chain_with_no_grounded_component_is_disconnected() {
    let mut a = hinge(Param::angle_deg(10.0));
    let lid_before = a.solve().unwrap()[1].to_matrix();
    // releasing the only ground leaves the jointed chain with nothing to hang from
    a.set_grounded(0, false);
    assert!(a.grounded().is_empty());
    let err = a.solve().unwrap_err();
    assert!(matches!(err, AssemblyError::Disconnected(_)), "{err}");
    // grounding the lid instead: it stays where it was, the base hangs from it through the joint
    a.set_grounded(1, true);
    let t = a.solve().unwrap();
    assert!((t[1].to_matrix() - lid_before).norm() < 1e-9);
    assert!(close(
        t[0].apply_point(&Point3::new(0.0, 0.0, 0.0)),
        Point3::origin()
    ));
}

#[test]
fn several_grounds_sit_at_their_own_placements() {
    let mut a = Assembly::new("Two grounds");
    let p = a.add_part(box_part("Block", 10.0, 10.0, 10.0, vec![]));
    let (x, y, z) = (
        a.add_instance("x", p),
        a.add_instance("y", p),
        a.add_instance("free", p),
    );
    a.set_grounded(x, true);
    a.set_grounded(y, true);
    a.instances[y].at = Placement {
        t: [100.0, 0.0, 0.0],
        axis: [0.0, 0.0, 1.0],
        angle: FRAC_PI_2,
    };
    a.instances[z].at = Placement {
        t: [0.0, 50.0, 0.0],
        ..Default::default()
    };
    assert_eq!(a.grounded(), vec![x, y]);
    let t = a.solve().unwrap();
    assert!(close(
        t[x].apply_point(&Point3::new(1.0, 2.0, 3.0)),
        Point3::new(1.0, 2.0, 3.0)
    ));
    // y is turned a quarter about z, then moved: (1,2,3) -> (-2,1,3) + (100,0,0)
    assert!(close(
        t[y].apply_point(&Point3::new(1.0, 2.0, 3.0)),
        Point3::new(98.0, 1.0, 3.0)
    ));
    assert!(close(
        t[z].apply_point(&Point3::new(1.0, 2.0, 3.0)),
        Point3::new(1.0, 52.0, 3.0)
    ));
}

fn rx(a: f64, p: [f64; 3]) -> [f64; 3] {
    [
        p[0],
        p[1] * a.cos() - p[2] * a.sin(),
        p[1] * a.sin() + p[2] * a.cos(),
    ]
}
fn ry(a: f64, p: [f64; 3]) -> [f64; 3] {
    [
        p[0] * a.cos() + p[2] * a.sin(),
        p[1],
        -p[0] * a.sin() + p[2] * a.cos(),
    ]
}
fn rz(a: f64, p: [f64; 3]) -> [f64; 3] {
    [
        p[0] * a.cos() - p[1] * a.sin(),
        p[0] * a.sin() + p[1] * a.cos(),
        p[2],
    ]
}

/// Two blocks with a frame at their origin whose axes are the world axes.
fn pair() -> Assembly {
    let mut a = Assembly::new("Pair");
    let f = || vec![Frame::new("o", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])];
    let p = a.add_part(box_part("Block", 10.0, 10.0, 10.0, f()));
    let q = a.add_part(box_part("Other", 10.0, 10.0, 10.0, f()));
    a.add_instance("base", p);
    a.add_instance("moving", q);
    a
}

#[test]
fn a_ball_joint_is_the_composed_rotation() {
    let mut a = pair();
    let (ax, ay, az) = (0.5_f64, 0.7_f64, -0.3_f64);
    a.add_joint(joint(
        "ball",
        JointKind::Ball,
        (0, "o"),
        (1, "o"),
        vec![
            Param::Lit(Quantity::angle(ax)),
            Param::Lit(Quantity::angle(ay)),
            Param::Lit(Quantity::angle(az)),
        ],
        vec![None, None, None],
    ));
    let t = a.solve().unwrap();
    let p = [1.0, 2.0, 3.0];
    let want = rz(az, ry(ay, rx(ax, p)));
    let got = t[1].apply_point(&Point3::new(p[0], p[1], p[2]));
    assert!(
        close(got, Point3::new(want[0], want[1], want[2])),
        "{got:?} {want:?}"
    );
    // the ball's centre does not move
    assert!(close(t[1].apply_point(&Point3::origin()), Point3::origin()));
    // each axis has its own limit
    a.joints[0].limits = vec![Some((-0.1, 0.1)), None, None];
    assert!(matches!(
        a.solve().unwrap_err(),
        AssemblyError::OutOfLimits { .. }
    ));
}

#[test]
fn flip_offset_and_angle_move_the_child_in_the_joint_frame() {
    let mut a = pair();
    a.add_joint(joint(
        "rigid",
        JointKind::Rigid,
        (0, "o"),
        (1, "o"),
        vec![],
        vec![],
    ));
    a.joints[0].offset = [1.0, 2.0, 3.0];
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(0.0, 0.0, 0.0)),
        Point3::new(1.0, 2.0, 3.0)
    ));
    // flipped: the child's z points the other way (a half turn about x)
    a.joints[0].offset = [0.0; 3];
    a.joints[0].flip = true;
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(0.0, 1.0, 1.0)),
        Point3::new(0.0, -1.0, -1.0)
    ));
    // turned about z by a quarter
    a.joints[0].flip = false;
    a.joints[0].angle = FRAC_PI_2;
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(1.0, 0.0, 0.0)),
        Point3::new(0.0, 1.0, 0.0)
    ));
}

#[test]
fn a_rigid_group_is_carried_by_the_joint_of_one_member() {
    let mut a = hinge(Param::angle_deg(0.0));
    let knob = a.add_instance("knob", 1);
    a.instances[knob].at = Placement {
        t: [10.0, 10.0, 6.0],
        ..Default::default()
    };
    // lid and knob keep their relative position from now on
    a.make_rigid_group("lid and knob", &[1, knob]).unwrap();
    let t = a.solve().unwrap();
    assert!(close(
        t[knob].apply_point(&Point3::origin()),
        Point3::new(10.0, 10.0, 6.0)
    ));
    // open the lid 90°: the knob (relative position (10,10,2) from the hinge) turns with it
    a.joints[0].values = vec![Param::angle_deg(90.0)];
    let t = a.solve().unwrap();
    assert!(
        close(
            t[knob].apply_point(&Point3::origin()),
            Point3::new(10.0, -2.0, 14.0)
        ),
        "{:?}",
        t[knob].apply_point(&Point3::origin())
    );
    // and it keeps its own orientation relative to the lid
    let lid_pt = t[1].apply_point(&Point3::new(1.0, 0.0, 0.0));
    let knob_pt = t[knob].apply_point(&Point3::new(1.0, 0.0, 0.0));
    let lid_o = t[1].apply_point(&Point3::origin());
    let knob_o = t[knob].apply_point(&Point3::origin());
    assert!(close(
        Point3::from(lid_pt - lid_o),
        Point3::from(knob_pt - knob_o)
    ));
    // a joint between two members of the group cannot be satisfied
    let mut b = a.clone();
    b.add_joint(joint(
        "inside",
        JointKind::Rigid,
        (1, "hinge"),
        (knob, "hinge"),
        vec![],
        vec![],
    ));
    assert!(matches!(
        b.solve().unwrap_err(),
        AssemblyError::Redundant { .. }
    ));
    // and a group refuses a pair that is already jointed
    let mut c = hinge(Param::angle_deg(0.0));
    let err = c.make_rigid_group("bad", &[0, 1]).unwrap_err();
    assert!(matches!(err, AssemblyError::Redundant { .. }), "{err}");
    assert!(c.groups.is_empty());
}

#[test]
fn a_rigid_group_with_a_grounded_member_follows_its_placement() {
    let mut a = pair();
    a.make_rigid_group("together", &[0, 1]).unwrap();
    a.instances[1].at = Placement {
        t: [30.0, 0.0, 0.0],
        ..Default::default()
    };
    a.make_rigid_group("again", &[0, 1]).unwrap_err(); // already grouped
    a.groups.clear();
    a.make_rigid_group("together", &[0, 1]).unwrap();
    // the base is grounded; move it and the other member follows at the same offset
    a.instances[0].at = Placement {
        t: [0.0, 100.0, 0.0],
        ..Default::default()
    };
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::origin()),
        Point3::new(30.0, 100.0, 0.0)
    ));
}

#[test]
fn an_as_built_joint_keeps_the_positions_and_turns_about_the_chosen_axis() {
    let mut a = pair();
    let pose0 = Placement {
        t: [3.0, 4.0, 5.0],
        axis: [0.0, 0.0, 1.0],
        angle: 0.35,
    };
    a.instances[1].at = pose0;
    let before = a.solve().unwrap();
    // a revolute joint about the world z axis through (3, 4, 5)
    let k = a
        .as_built_joint(
            0,
            1,
            JointKind::Revolute,
            [3.0, 4.0, 5.0],
            [0.0, 0.0, 1.0],
            "pin",
        )
        .unwrap();
    assert!(a.joints[k].as_built);
    let after = a.solve().unwrap();
    assert!(
        (before[1].to_matrix() - after[1].to_matrix()).norm() < 1e-9,
        "value 0 must reproduce the placement"
    );
    // a quarter turn about that axis: every point swings about (3, 4, 5)
    a.joints[k].values = vec![Param::angle_deg(90.0)];
    let turned = a.solve().unwrap();
    let p = Point3::new(7.0, 1.0, 2.0);
    let w0 = before[1].apply_point(&p);
    let rel = rz(FRAC_PI_2, [w0.x - 3.0, w0.y - 4.0, w0.z - 5.0]);
    assert!(
        close(
            turned[1].apply_point(&p),
            Point3::new(3.0 + rel[0], 4.0 + rel[1], 5.0 + rel[2])
        ),
        "{:?}",
        turned[1].apply_point(&p)
    );
    // the parts gained one frame each, with names that do not clash
    assert_eq!(a.parts[0].frames.len(), 2);
    assert_eq!(a.parts[1].frames.len(), 2);
    // an as-built joint between a component and itself, or about no axis, is refused
    assert!(a
        .as_built_joint(0, 0, JointKind::Rigid, [0.0; 3], [0.0, 0.0, 1.0], "x")
        .is_err());
    assert!(a
        .as_built_joint(0, 1, JointKind::Rigid, [0.0; 3], [0.0; 3], "x")
        .is_err());
}

#[test]
fn a_sub_assembly_composes_its_inner_joints_with_the_outer_ones() {
    // the hinge unit (base + lid hinged at 90°) is one component of a bigger assembly
    let mut outer = Assembly::new("Outer");
    let mut unit = Part::new("Unit", Document::new(), vec![]);
    unit.sub = Some(Box::new(hinge(Param::angle_deg(90.0))));
    let u = outer.add_part(unit);
    let table = outer.add_part(box_part(
        "Table",
        100.0,
        100.0,
        10.0,
        vec![Frame::new(
            "slot",
            [50.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let t0 = outer.add_instance("table", table);
    let u0 = outer.add_instance("unit", u);
    outer.add_joint(joint(
        "mount",
        JointKind::Rigid,
        (t0, "slot"),
        (u0, "base.hinge"),
        vec![],
        vec![],
    ));
    assert!(outer.parts[u]
        .frame_names()
        .contains(&"lid.hinge".to_string()));
    let leaves = outer.leaves().unwrap();
    let names: Vec<&str> = leaves.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["table", "unit/base", "unit/lid"]);
    // the unit's base hinge sits on the table's slot: base origin (0,0,0) is (0,0,4) from its hinge
    assert!(close(
        leaves[1].transform.apply_point(&Point3::origin()),
        Point3::new(50.0, 0.0, -4.0)
    ));
    // inner hinge (lid point (10,20,0) -> (10,0,24)) then the outer shift (50,0,-4)
    assert!(close(
        leaves[2]
            .transform
            .apply_point(&Point3::new(10.0, 20.0, 0.0)),
        Point3::new(60.0, 0.0, 20.0)
    ));
    // the unit posed as bodies: three solids
    let mut engine = Engine::with_threads(2);
    let pose = outer.pose(&mut engine).unwrap();
    assert_eq!(pose.bodies.len(), 3);
    assert_eq!(pose.names[2], "unit/lid");
    assert_eq!(pose.tops, vec![0, 1, 1]);
}

#[test]
fn components_can_be_grouped_into_a_sub_assembly_without_moving() {
    let mut a = hinge(Param::angle_deg(30.0));
    let plate = a.add_part(box_part(
        "Plate",
        50.0,
        50.0,
        2.0,
        vec![Frame::new(
            "top",
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        )],
    ));
    let pl = a.add_instance("plate", plate);
    a.add_joint(joint(
        "plate to base",
        JointKind::Rigid,
        (pl, "top"),
        (0, "hinge"),
        vec![],
        vec![],
    ));
    let before = a.leaves().unwrap();
    let k = a.group_into_subassembly(&[0, 1], "Hinge unit").unwrap();
    assert_eq!(a.instances.len(), 2);
    assert_eq!(a.instances[k].name, "Hinge unit");
    let after = a.leaves().unwrap();
    assert_eq!(after.len(), 3);
    for l in &before {
        let found = after
            .iter()
            .find(|m| m.name.ends_with(&l.name))
            .unwrap_or_else(|| panic!("{} is gone", l.name));
        assert!(
            (found.transform.to_matrix() - l.transform.to_matrix()).norm() < 1e-9,
            "{} moved",
            l.name
        );
    }
    // the joint that reached the base now reaches it through the unit
    let outside = a.joints.iter().find(|j| j.name == "plate to base").unwrap();
    assert_eq!(outside.child.frame, "base.hinge");
    // too few components, or half a rigid group, is refused
    assert!(a.group_into_subassembly(&[0], "x").is_err());
}

#[test]
fn removing_an_instance_keeps_joints_groups_and_grounds_consistent() {
    let mut a = hinge(Param::angle_deg(0.0));
    let k1 = a.add_instance("k1", 1);
    let k2 = a.add_instance("k2", 1);
    a.make_rigid_group("g", &[1, k1, k2]).unwrap();
    a.set_grounded(0, true);
    a.set_grounded(k2, true);
    a.remove_instance(k1);
    // k2 moved up one; the group lost a member but keeps two
    assert_eq!(a.instances.len(), 3);
    assert_eq!(a.groups[0].members, vec![1, 2]);
    assert_eq!(a.groups[0].rel.len(), 2);
    assert_eq!(a.grounded(), vec![0, 2]);
    // removing the lid removes its joint and dissolves the group (one member left)
    a.remove_instance(1);
    assert!(a.joints.is_empty());
    assert!(a.groups.is_empty());
    assert_eq!(a.grounded(), vec![0, 1]);
    assert!(a.solve().is_ok());
}

/// The first version of the file format, written by hand.
mod legacy {
    use kw_assembly::{Frame, JointEnd, JointKind};
    use kw_expr::Param;
    use kw_feature::Document;
    use serde::Serialize;

    #[derive(Serialize)]
    pub struct Part {
        pub name: String,
        pub document: Document,
        pub frames: Vec<Frame>,
    }
    #[derive(Serialize)]
    pub struct Instance {
        pub name: String,
        pub part: usize,
    }
    #[derive(Serialize)]
    pub struct Joint {
        pub name: String,
        pub kind: JointKind,
        pub parent: JointEnd,
        pub child: JointEnd,
        pub values: Vec<Param>,
        pub limits: Vec<Option<(f64, f64)>>,
    }
    #[derive(Serialize)]
    pub struct Assembly {
        pub name: String,
        pub parts: Vec<Part>,
        pub instances: Vec<Instance>,
        pub ground: usize,
        pub joints: Vec<Joint>,
        pub params: Vec<(String, Param)>,
    }
}

#[test]
fn an_assembly_file_of_the_first_version_still_loads() {
    let new = hinge(Param::angle_deg(90.0));
    let old = legacy::Assembly {
        name: new.name.clone(),
        parts: new
            .parts
            .iter()
            .map(|p| legacy::Part {
                name: p.name.clone(),
                document: p.document.clone(),
                frames: p.frames.clone(),
            })
            .collect(),
        instances: new
            .instances
            .iter()
            .map(|i| legacy::Instance {
                name: i.name.clone(),
                part: i.part,
            })
            .collect(),
        ground: 0,
        joints: new
            .joints
            .iter()
            .map(|j| legacy::Joint {
                name: j.name.clone(),
                kind: j.kind,
                parent: j.parent.clone(),
                child: j.child.clone(),
                values: j.values.clone(),
                limits: j.limits.clone(),
            })
            .collect(),
        params: vec![],
    };
    let mut bytes = Vec::new();
    ciborium::into_writer(&old, &mut bytes).unwrap();
    let loaded = Assembly::from_bytes(&bytes).unwrap();
    assert_eq!(loaded, new);
    assert_eq!(loaded.grounded(), vec![0]);
    assert_eq!(
        loaded.solve().unwrap()[1].to_matrix(),
        new.solve().unwrap()[1].to_matrix()
    );
}

#[test]
fn the_bill_of_materials_counts_copies_and_the_overlap_is_a_solid() {
    let mut engine = Engine::with_threads(2);
    let mut a = Assembly::new("Blocks");
    let mut block = box_part(
        "Block",
        20.0,
        20.0,
        20.0,
        vec![Frame::new(
            "f",
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        )],
    );
    block.material = Some("Steel".into());
    let p = a.add_part(block);
    let (i0, i1) = (a.add_instance("a", p), a.add_instance("b", p));
    a.add_joint(joint(
        "j",
        JointKind::Slider,
        (i0, "f"),
        (i1, "f"),
        vec![mm(15.0)],
        vec![None],
    ));
    let mut pose = a.pose(&mut engine).unwrap();
    let rows = pose.bom(&|m| if m == Some("Steel") { 7850.0 } else { 1000.0 });
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].count, 2);
    assert!((rows[0].volume - 8000.0).abs() < 1e-6);
    // 8000 mm³ = 8e-6 m³ of steel at 7850 kg/m³ = 62.8 g
    assert!((rows[0].mass_g - 62.8).abs() < 1e-9, "{}", rows[0].mass_g);
    for s in rows[0].size {
        assert!((s - 20.0).abs() < 1e-3, "{:?}", rows[0].size);
    }
    assert_eq!(rows[0].material.as_deref(), Some("Steel"));
    // the overlap: a 20 x 20 x 5 solid
    let hits = pose.interference(1e-6).unwrap();
    assert_eq!(hits.len(), 1);
    let body = hits[0].body.expect("the overlap is kept");
    let v = pose.rep.volume(body).unwrap();
    assert!((v - 2000.0).abs() < 1e-6, "{v}");
    // hidden components are left out of the pose and the list
    a.instances[i1].hidden = true;
    let pose = a.pose(&mut engine).unwrap();
    assert_eq!(pose.bodies.len(), 1);
    assert_eq!(pose.bom(&|_| 1000.0)[0].count, 1);
}

#[test]
fn the_pose_remembers_which_part_face_every_display_face_came_from() {
    let mut engine = Engine::with_threads(2);
    let a = hinge(Param::angle_deg(0.0));
    let pose = a.pose(&mut engine).unwrap();
    for (i, map) in pose.maps.iter().enumerate() {
        let faces = pose.rep.body_faces(pose.bodies[i]);
        assert!(!faces.is_empty());
        for f in &faces {
            assert!(
                map.faces.values().any(|v| v == f),
                "face of body {i} has no origin"
            );
            assert_eq!(pose.body_of_face(*f), Some(i));
        }
    }
}

#[test]
fn sliders_and_cylindrical_joints() {
    let mut a = Assembly::new("Slide");
    let rail = a.add_part(box_part(
        "Rail",
        100.0,
        10.0,
        10.0,
        vec![Frame::new(
            "track",
            [0.0, 5.0, 10.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let car = a.add_part(box_part(
        "Car",
        20.0,
        10.0,
        5.0,
        vec![Frame::new(
            "foot",
            [0.0, 5.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let (r, c) = (a.add_instance("rail", rail), a.add_instance("car", car));
    a.ground = r;
    a.add_joint(joint(
        "slide",
        JointKind::Slider,
        (r, "track"),
        (c, "foot"),
        vec![mm(35.0)],
        vec![Some((0.0, 80.0))],
    ));
    let t = a.solve().unwrap();
    // the car moved 35 mm along the rail's +x
    assert!(close(
        t[1].apply_point(&Point3::new(0.0, 5.0, 0.0)),
        Point3::new(35.0, 5.0, 10.0)
    ));
    a.joints[0].kind = JointKind::Cylindrical;
    a.joints[0].values = vec![Param::angle_deg(90.0), mm(35.0)];
    a.joints[0].limits = vec![None, None];
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(0.0, 5.0, 0.0)),
        Point3::new(35.0, 5.0, 10.0)
    ));
    // a point on the joint axis only slides; one off the axis keeps its distance from it and is turned
    assert!(close(
        t[1].apply_point(&Point3::new(1.0, 5.0, 0.0)),
        Point3::new(36.0, 5.0, 10.0)
    ));
    let p = t[1].apply_point(&Point3::new(0.0, 6.0, 0.0));
    assert!(
        ((p.y - 5.0).powi(2) + (p.z - 10.0).powi(2)).sqrt() - 1.0 < 1e-9
            && (p.x - 35.0).abs() < 1e-9
    );
    assert!(
        !close(p, Point3::new(35.0, 6.0, 10.0)),
        "the car was not rotated"
    );
}

#[test]
fn poses_interference_mass_and_step() {
    let mut engine = Engine::with_threads(2);
    // closed lid: the two boxes touch but do not overlap
    let mut pose = hinge(Param::angle_deg(0.0)).pose(&mut engine).unwrap();
    assert!(pose.interference(1e-6).unwrap().is_empty());
    let (vol, com) = pose.mass().unwrap();
    assert!(
        (vol - (40.0 * 20.0 * 4.0 + 40.0 * 20.0 * 2.0)).abs() < 1e-6,
        "{vol}"
    );
    assert!((com.x - 20.0).abs() < 1e-6 && (com.y - 10.0).abs() < 1e-6);
    let step = pose.to_step();
    assert_eq!(step.matches("MANIFOLD_SOLID_BREP").count(), 2);
    // two instances of the same part placed to overlap by 5 mm
    let mut a = Assembly::new("Overlap");
    let p = a.add_part(box_part(
        "Block",
        20.0,
        20.0,
        20.0,
        vec![Frame::new(
            "f",
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        )],
    ));
    let (i0, i1) = (a.add_instance("a", p), a.add_instance("b", p));
    a.ground = i0;
    a.add_joint(joint(
        "j",
        JointKind::Slider,
        (i0, "f"),
        (i1, "f"),
        vec![mm(15.0)],
        vec![None],
    ));
    let mut pose = a.pose(&mut engine).unwrap();
    let hits = pose.interference(1e-6).unwrap();
    assert_eq!(hits.len(), 1);
    assert!(
        (hits[0].volume - 20.0 * 20.0 * 5.0).abs() < 1e-6,
        "{}",
        hits[0].volume
    );
    // moving the joint re-evaluates no part
    a.joints[0].values = vec![mm(25.0)];
    let before = engine.cache_len();
    let mut pose = a.pose(&mut engine).unwrap();
    assert_eq!(engine.cache_len(), before);
    assert!(pose.interference(1e-6).unwrap().is_empty());
}

#[test]
fn an_assembly_is_stored_in_the_kwf_file() {
    let mut a = hinge(Param::expr("open").unwrap());
    a.params
        .push(("open".into(), Param::Lit(Quantity::angle(FRAC_PI_2 / 3.0))));
    let mut file = KwfFile::new(a.parts[0].document.clone());
    a.attach(&mut file);
    let bytes = file.to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap();
    let b = Assembly::from_file(&back).unwrap().unwrap();
    assert_eq!(a, b);
    assert_eq!(
        a.solve().unwrap()[1].to_matrix(),
        b.solve().unwrap()[1].to_matrix()
    );
}

#[test]
fn deleting_a_joint_or_a_ground_leaves_components_where_they_are() {
    // the lid, opened 90°, stays open when its joint is deleted
    let mut a = hinge(Param::angle_deg(90.0));
    let open = a.solve().unwrap()[1].to_matrix();
    a.remove_joint(0);
    assert!(a.joints.is_empty());
    assert!((a.solve().unwrap()[1].to_matrix() - open).norm() < 1e-9);
    // releasing the ground of a component that nothing joins leaves it in place; the other stays grounded
    let mut b = pair();
    b.instances[0].at.t = [5.0, 6.0, 7.0];
    b.set_grounded(1, true);
    b.set_grounded(0, false);
    assert_eq!(b.grounded(), vec![1]);
    let t = b.solve().unwrap();
    assert!(close(
        t[0].apply_point(&Point3::origin()),
        Point3::new(5.0, 6.0, 7.0)
    ));
    // removing an instance keeps its neighbours in place too
    let mut c = hinge(Param::angle_deg(90.0));
    let k = c.add_instance("extra", 1);
    c.add_joint(joint(
        "second",
        JointKind::Rigid,
        (1, "hinge"),
        (k, "hinge"),
        vec![],
        vec![],
    ));
    let before = c.solve().unwrap()[k].to_matrix();
    c.remove_instance(0); // the grounded base goes; the lid and extra remain
    c.set_grounded(0, true);
    assert!((c.solve().unwrap()[1].to_matrix() - before).norm() < 1e-9);
}

#[test]
fn two_grounded_components_cannot_be_joined() {
    let mut a = hinge(Param::angle_deg(0.0));
    a.set_grounded(1, true);
    let err = a.solve().unwrap_err();
    assert!(matches!(err, AssemblyError::BothGrounded { .. }), "{err}");
    assert!(err.to_string().contains("grounded"));
}

/// A ray through `target` coming from `dir` (towards the target).
fn ray_through(target: [f64; 3], dir: [f64; 3]) -> Ray {
    Ray {
        origin: [
            target[0] - dir[0] * 100.0,
            target[1] - dir[1] * 100.0,
            target[2] - dir[2] * 100.0,
        ],
        dir,
    }
}

#[test]
fn dragging_a_hinged_lid_turns_it_to_the_pointer_and_stops_at_its_limit() {
    let mut a = hinge(Param::angle_deg(0.0));
    // grab the far corner of the lid (lid coordinates (10, 20, 0): world (10, 20, 4) while closed)
    let mut drag = Drag::begin(&a, 1, [10.0, 20.0, 4.0], [0.0, 0.0, -1.0]).unwrap();
    assert_eq!(drag.joint(), Some(0));
    // the pointer ray comes straight down through the place the corner has at 40°
    let (s, c) = 40f64.to_radians().sin_cos();
    let target = [10.0, 20.0 * c, 4.0 + 20.0 * s];
    drag.update(&mut a, &ray_through(target, [0.0, 0.0, -1.0]))
        .unwrap();
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!((v - 40f64.to_radians()).abs() < 1e-6, "{}", v.to_degrees());
    // and the lid really is there
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(10.0, 20.0, 0.0)),
        Point3::new(target[0], target[1], target[2])
    ));
    // following the pointer on to 100°
    let (s, c) = 100f64.to_radians().sin_cos();
    drag.update(
        &mut a,
        &ray_through([10.0, 20.0 * c, 4.0 + 20.0 * s], [0.0, 0.0, -1.0]),
    )
    .unwrap();
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!((v - 100f64.to_radians()).abs() < 1e-6, "{}", v.to_degrees());
    // a pointer that would need a negative angle is held at the 0° limit
    drag.update(&mut a, &ray_through([10.0, 20.0, -30.0], [0.0, 0.0, -1.0]))
        .unwrap();
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!(v.abs() < 1e-3, "held at the 0° limit: {}°", v.to_degrees());
    assert!(a.solve().is_ok());
}

#[test]
fn dragging_a_slider_follows_the_pointer_along_the_rail() {
    let mut a = Assembly::new("Slide");
    let rail = a.add_part(box_part(
        "Rail",
        100.0,
        10.0,
        10.0,
        vec![Frame::new(
            "track",
            [0.0, 5.0, 10.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let car = a.add_part(box_part(
        "Car",
        20.0,
        10.0,
        5.0,
        vec![Frame::new(
            "foot",
            [0.0, 5.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        )],
    ));
    let (r, c) = (a.add_instance("rail", rail), a.add_instance("car", car));
    a.add_joint(joint(
        "slide",
        JointKind::Slider,
        (r, "track"),
        (c, "foot"),
        vec![mm(0.0)],
        vec![Some((0.0, 80.0))],
    ));
    // grab the car at its middle top, (10, 5, 15) while the slider is at 0
    let mut drag = Drag::begin(&a, c, [10.0, 5.0, 15.0], [0.0, -1.0, 0.0]).unwrap();
    drag.update(
        &mut a,
        &ray_through([10.0 + 35.0, 5.0, 15.0], [0.0, -1.0, 0.0]),
    )
    .unwrap();
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!((v - 35.0).abs() < 1e-5, "{v}");
    // beyond the end of the rail: clamped to the limit
    drag.update(
        &mut a,
        &ray_through([10.0 + 300.0, 5.0, 15.0], [0.0, -1.0, 0.0]),
    )
    .unwrap();
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!((v - 80.0).abs() < 1e-9, "{v}");
    // the grounded rail cannot be dragged, and says why
    let err = Drag::begin(&a, r, [50.0, 5.0, 5.0], [0.0, -1.0, 0.0]).unwrap_err();
    assert!(err.to_string().contains("cannot be moved"), "{err}");
}

#[test]
fn dragging_a_ball_joint_swings_the_grabbed_point_onto_the_ray() {
    let mut a = pair();
    a.add_joint(joint(
        "ball",
        JointKind::Ball,
        (0, "o"),
        (1, "o"),
        vec![
            Param::angle_deg(0.0),
            Param::angle_deg(0.0),
            Param::angle_deg(0.0),
        ],
        vec![None, None, None],
    ));
    // grab the point (0, 0, 10) of the moving block, straight above the ball's centre
    let mut drag = Drag::begin(&a, 1, [0.0, 0.0, 10.0], [0.0, -1.0, 0.0]).unwrap();
    // the pointer ray runs along y through (6, ?, 8): the point swings to x = 6, z = 8 (|p| stays 10)
    drag.update(&mut a, &ray_through([6.0, 0.0, 8.0], [0.0, 1.0, 0.0]))
        .unwrap();
    let t = a.solve().unwrap();
    let p = t[1].apply_point(&Point3::new(0.0, 0.0, 10.0));
    assert!((p.coords.norm() - 10.0).abs() < 1e-9, "{p:?}");
    assert!(
        (p.x - 6.0).abs() < 1e-5 && (p.z - 8.0).abs() < 1e-5,
        "{p:?}"
    );
}

#[test]
fn a_free_component_slides_in_the_plane_facing_the_camera() {
    let mut a = pair();
    // the second block floats at (30, 0, 0); the camera looks along -y
    a.instances[1].at.t = [30.0, 0.0, 0.0];
    let mut drag = Drag::begin(&a, 1, [35.0, 5.0, 5.0], [0.0, -1.0, 0.0]).unwrap();
    assert_eq!(drag.joint(), None);
    // the pointer moves to where the grabbed point would be at (50, 5, 25)
    drag.update(&mut a, &ray_through([50.0, 5.0, 25.0], [0.0, -1.0, 0.0]))
        .unwrap();
    let t = a.solve().unwrap();
    assert!(close(
        t[1].apply_point(&Point3::new(5.0, 5.0, 5.0)),
        Point3::new(50.0, 5.0, 25.0)
    ));
    // a tilted view moves it in the plane perpendicular to the viewing direction: depth does not change
    let d = [0.0, -0.6, -0.8];
    let mut drag = Drag::begin(&a, 1, [50.0, 5.0, 25.0], d).unwrap();
    drag.update(&mut a, &ray_through([60.0, 8.0, 21.0], d))
        .unwrap();
    let t = a.solve().unwrap();
    let p = t[1].apply_point(&Point3::new(5.0, 5.0, 5.0));
    // the move is perpendicular to the view direction
    let m = [p.x - 50.0, p.y - 5.0, p.z - 25.0];
    assert!(
        (m[0] * d[0] + m[1] * d[1] + m[2] * d[2]).abs() < 1e-9,
        "{m:?}"
    );
    // a grounded component refuses
    assert!(Drag::begin(&a, 0, [1.0, 1.0, 1.0], d).is_err());
}

#[test]
fn a_floating_rigid_group_moves_as_one_and_a_jointed_group_drives_its_joint() {
    let mut a = pair();
    let third = a.add_instance("third", 1);
    a.instances[1].at.t = [30.0, 0.0, 0.0];
    a.instances[third].at.t = [60.0, 0.0, 0.0];
    a.make_rigid_group("pair", &[1, third]).unwrap();
    let before = a.solve().unwrap();
    let mut drag = Drag::begin(&a, third, [65.0, 5.0, 5.0], [0.0, -1.0, 0.0]).unwrap();
    drag.update(&mut a, &ray_through([65.0, 5.0, 25.0], [0.0, -1.0, 0.0]))
        .unwrap();
    let after = a.solve().unwrap();
    for k in [1, third] {
        let b = before[k].apply_point(&Point3::origin());
        let c = after[k].apply_point(&Point3::origin());
        assert!(
            close(c, Point3::new(b.x, b.y, b.z + 20.0)),
            "{c:?} vs {b:?}"
        );
    }
    // with the group on a slider, dragging a member drives the slider
    let mut b = pair();
    let third = b.add_instance("third", 1);
    b.make_rigid_group("pair", &[1, third]).unwrap();
    b.add_joint(joint(
        "slide",
        JointKind::Slider,
        (0, "o"),
        (1, "o"),
        vec![mm(0.0)],
        vec![None],
    ));
    assert_eq!(b.drive_joint(third), Some(0));
    assert_eq!(b.drive_joint(1), Some(0));
    assert_eq!(b.drive_joint(0), None);
}

#[test]
fn parts_nobody_uses_are_pruned_and_the_rest_renumbered() {
    let mut a = hinge(Param::angle_deg(0.0));
    let extra = a.add_part(box_part("Spare", 5.0, 5.0, 5.0, vec![]));
    let k = a.add_instance("spare", extra);
    // the lid (part 1) loses its only instance: part 1 goes, the spare moves down to index 1
    a.remove_instance(1);
    assert_eq!(a.prune_unused_parts(), 1);
    assert_eq!(a.parts.len(), 2);
    assert_eq!(a.parts[1].name, "Spare");
    assert_eq!(a.instances[k - 1].part, 1);
    assert!(a.solve().is_ok());
    // nothing to prune the second time
    assert_eq!(a.prune_unused_parts(), 0);
}

// ------------------------------------------------------------------------------------ linked files

fn scratch(name: &str) -> std::path::PathBuf {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static N: AtomicUsize = AtomicUsize::new(0);
    let d = std::env::temp_dir().join(format!(
        "kw-asm-{name}-{}-{}",
        std::process::id(),
        N.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn frame0() -> Vec<Frame> {
    vec![Frame::new("f", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])]
}

/// Write a `.kwf` holding a cube of the given side and return its model.
fn write_cube(path: &std::path::Path, side: f64) -> Document {
    let doc = box_part("Cube", side, side, side, vec![]).document;
    KwfFile::new(doc.clone()).save(path).unwrap();
    doc
}

/// An assembly with one part linked to `file`, taken as a cube of side 10.
fn linked(dir: &std::path::Path, file: &std::path::Path) -> Assembly {
    let doc = write_cube(file, 10.0);
    let mut part = Part::new("Cube", doc.clone(), frame0());
    part.link = Some(PartLink::new(file, Some(dir), &doc));
    let mut a = Assembly::new("Linked");
    let p = a.add_part(part);
    a.ground = a.add_instance("cube", p);
    a
}

fn volume(a: &Assembly) -> f64 {
    a.pose(&mut Engine::with_threads(2))
        .unwrap()
        .mass()
        .unwrap()
        .0
}

#[test]
fn a_linked_part_goes_stale_when_its_file_changes_and_updates_on_request() {
    let dir = scratch("stale");
    let file = dir.join("cube.kwf");
    let mut a = linked(&dir, &file);
    assert_eq!(a.link_status(Some(&dir)), vec![(0, LinkStatus::Current)]);
    assert!((volume(&a) - 1000.0).abs() < 1e-6);
    let frames = a.parts[0].frames.clone();
    // someone makes the cube bigger upstream
    write_cube(&file, 20.0);
    assert_eq!(a.link_status(Some(&dir)), vec![(0, LinkStatus::Stale)]);
    // nothing changes until asked: the cached model is still the old one
    assert!((volume(&a) - 1000.0).abs() < 1e-6);
    let report = a.update_link(0, Some(&dir)).unwrap();
    assert!(report.changed && report.part == "Cube");
    assert_eq!(a.link_status(Some(&dir)), vec![(0, LinkStatus::Current)]);
    assert!((volume(&a) - 8000.0).abs() < 1e-6);
    assert_eq!(a.parts[0].frames, frames, "frames survive an update");
    // updating when nothing changed says so
    assert!(!a.update_link(0, Some(&dir)).unwrap().changed);
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn update_links_takes_every_out_of_date_part_and_reports_the_rest() {
    let dir = scratch("many");
    let (f1, f2) = (dir.join("one.kwf"), dir.join("two.kwf"));
    let mut a = linked(&dir, &f1);
    let doc2 = write_cube(&f2, 5.0);
    let mut p2 = Part::new("Two", doc2.clone(), frame0());
    p2.link = Some(PartLink::new(&f2, Some(&dir), &doc2));
    a.add_part(p2);
    write_cube(&f1, 30.0);
    let done = a.update_links(Some(&dir));
    assert_eq!(done.len(), 1);
    assert_eq!(done[0].0, 0);
    assert!(done[0].1.as_ref().unwrap().changed);
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn a_missing_or_damaged_file_leaves_the_cached_model_in_use() {
    let dir = scratch("missing");
    let file = dir.join("cube.kwf");
    let mut a = linked(&dir, &file);
    std::fs::remove_file(&file).unwrap();
    assert_eq!(a.link_status(Some(&dir)), vec![(0, LinkStatus::Missing)]);
    // the assembly still solves and poses from the copy it carries
    assert!(a.solve().is_ok());
    assert!((volume(&a) - 1000.0).abs() < 1e-6);
    let err = a.update_link(0, Some(&dir)).unwrap_err().to_string();
    assert!(err.contains("not there"), "{err}");
    assert!((volume(&a) - 1000.0).abs() < 1e-6);
    // a file that is not a model is reported as unreadable
    std::fs::write(&file, b"this is not a kwf file").unwrap();
    assert!(matches!(
        a.link_status(Some(&dir))[0].1,
        LinkStatus::Unreadable(_)
    ));
    assert!(a.update_link(0, Some(&dir)).is_err());
    assert!((volume(&a) - 1000.0).abs() < 1e-6);
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn links_are_relative_to_the_project_and_follow_it_when_it_moves() {
    let root = scratch("move");
    let project = root.join("project");
    let parts = root.join("parts");
    std::fs::create_dir_all(&project).unwrap();
    std::fs::create_dir_all(&parts).unwrap();
    let file = parts.join("cube.kwf");
    let mut a = linked(&project, &file);
    let link = a.parts[0].link.clone().unwrap();
    assert_eq!(link.rel.as_deref(), Some("../parts/cube.kwf"));
    // the whole tree is moved: the relative path still leads to the file, the absolute one does not
    let moved = std::env::temp_dir().join(format!("kw-asm-moved-{}", std::process::id()));
    std::fs::remove_dir_all(&moved).ok();
    std::fs::rename(&root, &moved).unwrap();
    let project2 = moved.join("project");
    assert_eq!(
        a.link_status(Some(&project2)),
        vec![(0, LinkStatus::Current)]
    );
    assert_eq!(a.link_status(None), vec![(0, LinkStatus::Missing)]);
    // opening refreshes the stored paths
    assert!(a.rebase_links(Some(&project2), &project2));
    assert!(
        !a.rebase_links(Some(&project2), &project2),
        "nothing more to change"
    );
    assert_eq!(a.link_status(None), vec![(0, LinkStatus::Current)]);
    // saving in another folder keeps the link pointing at the same file
    let elsewhere = moved.join("deeper").join("again");
    std::fs::create_dir_all(&elsewhere).unwrap();
    a.rebase_links(Some(&project2), &elsewhere);
    assert_eq!(
        a.parts[0].link.as_ref().unwrap().rel.as_deref(),
        Some("../../parts/cube.kwf")
    );
    assert_eq!(
        a.link_status(Some(&elsewhere)),
        vec![(0, LinkStatus::Current)]
    );
    std::fs::remove_dir_all(&moved).ok();
}

#[test]
fn links_can_be_broken_or_pointed_elsewhere_and_are_stored_with_the_assembly() {
    let dir = scratch("break");
    let file = dir.join("cube.kwf");
    let other = dir.join("other.kwf");
    let mut a = linked(&dir, &file);
    // stored with the assembly and read back the same
    let back = Assembly::from_bytes(&a.to_bytes()).unwrap();
    assert_eq!(a, back);
    // pointing it at another file takes that file's model
    write_cube(&other, 40.0);
    let r = a.relink(0, &other, Some(&dir)).unwrap();
    assert!(r.changed);
    assert!((volume(&a) - 64000.0).abs() < 1e-6);
    assert_eq!(a.link_status(Some(&dir)), vec![(0, LinkStatus::Current)]);
    // breaking it keeps the model and stops tracking the file
    assert!(a.break_link(0));
    assert!(a.link_status(Some(&dir)).is_empty());
    assert!(!a.break_link(0));
    write_cube(&other, 50.0);
    assert!((volume(&a) - 64000.0).abs() < 1e-6);
    assert!(a.update_link(0, Some(&dir)).is_err());
    std::fs::remove_dir_all(&dir).ok();
}

fn write_asm_file(path: &std::path::Path, asm: &Assembly) {
    let mut f = KwfFile::new(Document::new());
    asm.attach(&mut f);
    f.save(path).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(5));
}

/// A part that is a sub-assembly linked to the assembly stored in `file`.
fn sub_part(name: &str, file: &std::path::Path, dir: &std::path::Path, sub: Assembly) -> Part {
    let mut p = Part::new(name, Document::new(), frame0());
    p.link = Some(PartLink::to_assembly(file, Some(dir), &sub.to_bytes()));
    p.sub = Some(Box::new(sub));
    p
}

#[test]
fn an_assembly_file_can_be_linked_as_a_sub_assembly_and_follows_its_file() {
    let dir = scratch("subasm");
    let file = dir.join("hinge.kwf");
    let sub = hinge(Param::angle_deg(0.0));
    write_asm_file(&file, &sub);
    let mut top = Assembly::new("Top");
    let p = top.add_part(sub_part("Hinge", &file, &dir, sub));
    top.ground = top.add_instance("hinge", p);
    assert_eq!(top.link_status(Some(&dir)), vec![(0, LinkStatus::Current)]);
    assert!(top.solve().is_ok());
    // the hinge is opened in its own file
    let mut opened = hinge(Param::angle_deg(90.0));
    opened.name = "Hinged lid".into();
    write_asm_file(&file, &opened);
    assert_eq!(top.link_status(Some(&dir)), vec![(0, LinkStatus::Stale)]);
    let r = top.update_link(0, Some(&dir)).unwrap();
    assert!(r.changed);
    assert_eq!(top.parts[0].sub.as_deref(), Some(&opened));
    assert_eq!(top.link_status(Some(&dir)), vec![(0, LinkStatus::Current)]);
    // the lid really is open in the combined assembly
    let leaves = top.leaves().unwrap();
    assert_eq!(leaves.len(), 2);
    let lid = leaves.iter().find(|l| l.name.ends_with("lid")).unwrap();
    let p = lid.transform.apply_point(&Point3::new(10.0, 20.0, 0.0));
    assert!((p - Point3::new(10.0, 0.0, 24.0)).norm() < 1e-9, "{p}");
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn links_inside_a_linked_assembly_are_followed_down_to_their_own_files() {
    let root = scratch("nested");
    let lib = root.join("lib");
    let proj = root.join("proj");
    std::fs::create_dir_all(&lib).unwrap();
    std::fs::create_dir_all(&proj).unwrap();
    // lib/inner.kwf is an assembly of one part linked to lib/cube.kwf
    let cube = lib.join("cube.kwf");
    let doc = write_cube(&cube, 10.0);
    let mut inner = Assembly::new("Inner");
    let mut part = Part::new("Cube", doc.clone(), frame0());
    part.link = Some(PartLink::new(&cube, Some(&lib), &doc));
    let ip = inner.add_part(part);
    inner.ground = inner.add_instance("cube", ip);
    let inner_file = lib.join("inner.kwf");
    write_asm_file(&inner_file, &inner);
    // proj links to lib/inner.kwf
    let mut top = Assembly::new("Top");
    let p = top.add_part(sub_part("Inner", &inner_file, &proj, inner));
    top.ground = top.add_instance("inner", p);
    assert_eq!(top.link_status(Some(&proj)), vec![(0, LinkStatus::Current)]);
    assert!((volume(&top) - 1000.0).abs() < 1e-6);
    // the cube changes; the inner assembly's own file does not
    write_cube(&cube, 20.0);
    assert_eq!(top.link_status(Some(&proj)), vec![(0, LinkStatus::Stale)]);
    assert!(top.update_link(0, Some(&proj)).unwrap().changed);
    assert_eq!(top.link_status(Some(&proj)), vec![(0, LinkStatus::Current)]);
    assert!((volume(&top) - 8000.0).abs() < 1e-6);
    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn files_that_link_to_each_other_in_a_circle_are_refused() {
    let dir = scratch("circle");
    let (fa, fb) = (dir.join("a.kwf"), dir.join("b.kwf"));
    // placeholders so that the links can be made
    write_asm_file(&fa, &Assembly::new("A"));
    write_asm_file(&fb, &Assembly::new("B"));
    let make = |name: &str, to: &std::path::Path| {
        let mut a = Assembly::new(name);
        let mut p = Part::new("Other", Document::new(), frame0());
        p.link = Some(PartLink::to_assembly(to, Some(&dir), b"x"));
        p.sub = Some(Box::new(Assembly::new("Other")));
        let i = a.add_part(p);
        a.ground = a.add_instance("other", i);
        a
    };
    let (a, b) = (make("A", &fb), make("B", &fa));
    write_asm_file(&fa, &a);
    write_asm_file(&fb, &b);
    // a project that links to a.kwf
    let mut top = Assembly::new("Top");
    let mut p = Part::new("A", Document::new(), frame0());
    p.link = Some(PartLink::to_assembly(&fa, Some(&dir), b"x"));
    p.sub = Some(Box::new(Assembly::new("A")));
    let i = top.add_part(p);
    top.ground = top.add_instance("a", i);
    let err = top.update_link(0, Some(&dir)).unwrap_err().to_string();
    assert!(err.contains("circle"), "{err}");
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn a_part_with_several_solid_bodies_places_all_of_them() {
    // two separate blocks in one part, the second with its own material
    let mut doc = Document::new();
    for (k, x0) in [0.0, 30.0].into_iter().enumerate() {
        let s = doc
            .add_rect_sketch(
                &format!("S{k}"),
                kw_feature::PlaneSpec::Xy(mm(0.0)),
                mm(x0),
                mm(0.0),
                mm(10.0),
                mm(10.0),
            )
            .unwrap();
        doc.add_extrude(&format!("Block{k}"), s, mm(5.0), BodyOp::New)
            .unwrap();
    }
    let heads = doc.heads();
    assert_eq!(heads.len(), 2);
    let root = doc.body_root(heads[1]);
    doc.set_appearance(
        root,
        kw_feature::Appearance {
            name: Some("Lug".into()),
            colour: None,
            material: Some("Aluminium".into()),
        },
    );
    let mut a = Assembly::new("Two-body part");
    let p = a.add_part(Part::new("Bracket", doc, frame0()));
    a.ground = a.add_instance("bracket", p);
    a.instances[0].at.t = [0.0, 0.0, 100.0];
    let pose = a.pose(&mut Engine::with_threads(2)).unwrap();
    assert_eq!(pose.bodies.len(), 2);
    assert_eq!(pose.names, vec!["bracket/Body1", "bracket/Lug"]);
    assert_eq!(pose.materials, vec![None, Some("Aluminium".to_string())]);
    for v in &pose.volumes {
        assert!((v - 500.0).abs() < 1e-6, "{:?}", pose.volumes);
    }
    // both are moved with the component
    let (vol, com) = pose.mass().unwrap();
    assert!((vol - 1000.0).abs() < 1e-6);
    assert!(
        (com.z - 102.5).abs() < 1e-6 && (com.x - 20.0).abs() < 1e-6,
        "{com}"
    );
}
