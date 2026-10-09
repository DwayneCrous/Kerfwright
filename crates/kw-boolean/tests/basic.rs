#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};

fn at(x: f64, y: f64, z: f64) -> Frame3 {
    Frame3 {
        origin: Point3::new(x, y, z),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

#[test]
fn overlapping_boxes() {
    for (name, op, want) in [
        ("union", Op::Union, 13.45),
        ("intersect", Op::Intersect, 2.55),
        ("subtract", Op::Subtract, 5.45),
    ] {
        let mut rep = BRep::new();
        let a = make_box(&mut rep, &at(0.0, 0.0, 0.0), 2.0, 2.0, 2.0).body;
        let b = make_box(&mut rep, &at(1.0, 0.5, 0.3), 2.0, 2.0, 2.0).body;
        let r = boolean(&mut rep, a, b, op, &BoolOptions::default())
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let v = rep.volume(r.body).unwrap();
        assert!(rel(v, want) < 1e-10, "{name}: volume {v} vs {want}");
        assert!(rep.validate().is_ok(), "{name}: {:?}", rep.validate());
    }
}
