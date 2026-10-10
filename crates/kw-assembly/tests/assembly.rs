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
    Part {
        name: name.into(),
        document: doc,
        frames,
    }
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
    a.add_joint(Joint {
        name: "hinge".into(),
        kind: JointKind::Revolute,
        parent: JointEnd {
            instance: i0,
            frame: "hinge".into(),
        },
        child: JointEnd {
            instance: i1,
            frame: "hinge".into(),
        },
        values: vec![angle],
        limits: vec![Some((0.0, std::f64::consts::PI))],
    });
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
fn loops_and_loose_instances_are_refused_with_an_explanation() {
    let mut a = hinge(Param::angle_deg(10.0));
    let extra = a.add_instance("second lid", 1);
    // not yet connected
    let err = a.solve().unwrap_err();
    assert!(
        matches!(&err, AssemblyError::Disconnected(n) if n == "second lid"),
        "{err}"
    );
    // connect it to the base, then close a loop through the first lid
    a.add_joint(Joint {
        name: "second hinge".into(),
        kind: JointKind::Rigid,
        parent: JointEnd {
            instance: 0,
            frame: "hinge".into(),
        },
        child: JointEnd {
            instance: extra,
            frame: "hinge".into(),
        },
        values: vec![],
        limits: vec![],
    });
    assert!(a.solve().is_ok());
    a.add_joint(Joint {
        name: "closing joint".into(),
        kind: JointKind::Rigid,
        parent: JointEnd {
            instance: 1,
            frame: "hinge".into(),
        },
        child: JointEnd {
            instance: extra,
            frame: "hinge".into(),
        },
        values: vec![],
        limits: vec![],
    });
    let err = a.solve().unwrap_err();
    assert!(
        matches!(&err, AssemblyError::Loop { joint, .. } if joint == "closing joint"),
        "{err}"
    );
    assert!(err.to_string().contains("loop"));
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
    a.add_joint(Joint {
        name: "slide".into(),
        kind: JointKind::Slider,
        parent: JointEnd {
            instance: r,
            frame: "track".into(),
        },
        child: JointEnd {
            instance: c,
            frame: "foot".into(),
        },
        values: vec![mm(35.0)],
        limits: vec![Some((0.0, 80.0))],
    });
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
    a.add_joint(Joint {
        name: "j".into(),
        kind: JointKind::Slider,
        parent: JointEnd {
            instance: i0,
            frame: "f".into(),
        },
        child: JointEnd {
            instance: i1,
            frame: "f".into(),
        },
        values: vec![mm(15.0)],
        limits: vec![None],
    });
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
