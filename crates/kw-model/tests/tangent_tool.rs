#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! A fillet "spandrel" tool tangent to the two faces of a box edge: tangent cut curves must work.
use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::PI;

#[test]
fn subtract_a_tangent_spandrel_from_a_box_edge() {
    // box [0,4]x[0,3]x[0,2]; fillet the edge along z at x=4,y=3 (convex) with radius r
    let r = 0.5;
    let m = 0.4;
    let mut rep = BRep::new();
    let f = Frame3 {
        origin: Point3::new(2.0, 1.5, 1.0),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };
    let a = make_box(&mut rep, &f, 4.0, 3.0, 2.0).body;
    // cross-section tool polygon in the xy plane (z extruded from -m to 2+m)
    let p = |x: f64, y: f64| Point3::new(x, y, -m);
    // corner P=(4,3); faces: x=4 (outward +x) and y=3 (outward +y); material is x<4, y<3
    let t1 = (4.0, 3.0 - r); // on the x=4 face
    let t2 = (4.0 - r, 3.0); // on the y=3 face
    let c = (4.0 - r, 3.0 - r);
    let frame = Frame3 {
        origin: Point3::new(c.0, c.1, -m),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };
    let arc = ProfileEdge::arc(&frame, r, 0.0, PI / 2.0); // from (c.0 + r, c.1) = t1 to (c.0, c.1 + r) = t2
    let edges = vec![
        arc,
        ProfileEdge::line(p(t2.0, t2.1), p(t2.0, t2.1 + m)),
        ProfileEdge::line(p(t2.0, t2.1 + m), p(4.0 + m, 3.0 + m)),
        ProfileEdge::line(p(4.0 + m, 3.0 + m), p(t1.0 + m, t1.1)),
        ProfileEdge::line(p(t1.0 + m, t1.1), p(t1.0, t1.1)),
    ];
    let prof = Profile::new(
        Frame3 {
            origin: Point3::new(0.0, 0.0, -m),
            x: Vector3::x(),
            y: Vector3::y(),
            z: Vector3::z(),
        },
        ProfileLoop::new(edges),
    );
    let tool = extrude(
        &mut rep,
        &prof,
        Vector3::new(0.0, 0.0, 2.0 + 2.0 * m),
        &ExtrudeOptions::default(),
    )
    .expect("tool");
    assert!(rep.validate().is_ok(), "{:?}", rep.validate());
    let res = boolean(
        &mut rep,
        a,
        tool.body,
        Op::Subtract,
        &BoolOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));
    let v = rep.volume(res.body).unwrap();
    let expect = 24.0 - 2.0 * r * r * (1.0 - PI / 4.0);
    assert!((v - expect).abs() < 1e-9, "volume {v} vs {expect}");
    assert!(rep.validate().is_ok());
}
