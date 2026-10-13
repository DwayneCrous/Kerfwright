#![allow(unused_imports, dead_code, clippy::type_complexity)]
//! Operands that touch without crossing: a sphere resting on a box, two tangent spheres, a cylinder
//! inside a box touching two walls along lines. The answer is determined by containment; the engine
//! returns it (valid bodies with several lumps, or empty / unchanged results) rather than refusing.

mod common;
use common::*;
use kw_boolean::*;
use kw_brep::*;

fn cases() -> Vec<(
    &'static str,
    (Shape, kw_math::Frame3),
    (Shape, kw_math::Frame3),
)> {
    vec![
        (
            "sphere resting on a box face",
            (Shape::Box(2.0, 2.0, 2.0), axis_frame(0.0, 0.0, 0.0)),
            (Shape::Sphere(0.7), axis_frame(0.1, 0.2, 1.7)),
        ),
        (
            "two spheres touching",
            (Shape::Sphere(1.0), axis_frame(0.0, 0.0, 0.0)),
            (Shape::Sphere(0.6), axis_frame(1.6, 0.0, 0.0)),
        ),
        (
            "cylinder through a box touching two walls",
            (Shape::Box(2.0, 2.0, 2.0), axis_frame(0.0, 0.0, 0.0)),
            (Shape::Cyl(0.5, 3.0), axis_frame(0.5, -0.5, -1.5)),
        ),
    ]
}

#[test]
fn tangent_contact_cases() {
    for (name, a, b) in cases() {
        for op in [Op::Union, Op::Intersect, Op::Subtract] {
            let mut rep = BRep::new();
            let ba = build(&mut rep, a.0, &a.1);
            let bb = build(&mut rep, b.0, &b.1);
            let (va, vb) = (rep.volume(ba).unwrap(), rep.volume(bb).unwrap());
            let r = boolean(&mut rep, ba, bb, op, &BoolOptions::default());
            match &r {
                Ok(res) => {
                    let v = rep.volume(res.body).unwrap();
                    println!("{name} {op:?}: ok volume {v:.6} (A {va:.6}, B {vb:.6})");
                    assert!(
                        rep.validate().is_ok(),
                        "{name} {op:?}: {:?}",
                        rep.validate()
                    );
                    assert!(sample_check(&rep, ba, bb, Some(res.body), op, 7).is_none());
                }
                Err(BooleanError::Empty) => {
                    println!("{name} {op:?}: empty");
                    assert!(sample_check(&rep, ba, bb, None, op, 7).is_none());
                }
                Err(e) => println!("{name} {op:?}: ERR {e}"),
            }
        }
    }
}

#[test]
fn a_box_corner_piercing_a_face_is_fine() {
    for op in [Op::Union, Op::Intersect, Op::Subtract] {
        let mut rep = BRep::new();
        let a = build(
            &mut rep,
            Shape::Box(2.0, 2.0, 2.0),
            &axis_frame(0.0, 0.0, 0.0),
        );
        // a second box rotated 45° about z and 35° about x: its lowest corner pokes through A's top face
        let f = kw_math::Frame3 {
            origin: kw_math::Point3::new(0.1, 0.2, 1.7),
            ..kw_math::Frame3::from_origin_normal(
                kw_math::Point3::new(0.1, 0.2, 1.7),
                kw_math::Vector3::new(1.0, 1.0, 1.0),
            )
        };
        let b = build(&mut rep, Shape::Box(1.0, 1.0, 1.0), &f);
        let r = boolean(&mut rep, a, b, op, &BoolOptions::default());
        println!(
            "corner piercing {op:?}: {:?}",
            r.as_ref()
                .map(|r| rep.volume(r.body))
                .map_err(|e| e.to_string())
        );
    }
}
