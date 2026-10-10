#![allow(unused_imports, dead_code, clippy::type_complexity)]
//! Neighbouring result faces that lie in one surface are one face of the result.

mod common;
use common::*;
use kw_boolean::*;
use kw_brep::*;

#[test]
fn union_of_overlapping_boxes_is_a_box_of_six_faces() {
    // two boxes sharing the planes of their top, bottom and sides: the union is a 3 × 1 × 1 box
    let mut rep = BRep::new();
    let a = build(
        &mut rep,
        Shape::Box(2.0, 1.0, 1.0),
        &axis_frame(0.0, 0.0, 0.0),
    );
    let b = build(
        &mut rep,
        Shape::Box(2.0, 1.0, 1.0),
        &axis_frame(1.0, 0.0, 0.0),
    );
    let r = boolean(&mut rep, a, b, Op::Union, &BoolOptions::default()).unwrap();
    assert!(rep.validate().is_ok());
    assert!((rep.volume(r.body).unwrap() - 3.0).abs() < 1e-9);
    assert_eq!(rep.body_faces(r.body).len(), 6, "coplanar faces are merged");
    // without merging the faces stay in pieces
    let o = BoolOptions {
        merge_faces: false,
        ..Default::default()
    };
    let r2 = boolean(&mut rep, a, b, Op::Union, &o).unwrap();
    assert!(rep.body_faces(r2.body).len() > 6);
    assert!((rep.volume(r2.body).unwrap() - 3.0).abs() < 1e-9);
}

#[test]
fn a_box_on_a_box_merges_the_shared_side_faces() {
    // a 1×1×1 box (centred) standing on a 2×2×1 slab, flush with two sides of it
    let mut rep = BRep::new();
    let slab = build(
        &mut rep,
        Shape::Box(2.0, 2.0, 1.0),
        &axis_frame(0.0, 0.0, 0.0),
    );
    let cube = build(
        &mut rep,
        Shape::Box(1.0, 1.0, 1.0),
        &axis_frame(-0.5, -0.5, 1.0),
    );
    let r = boolean(&mut rep, slab, cube, Op::Union, &BoolOptions::default()).unwrap();
    assert!(rep.validate().is_ok());
    assert!((rep.volume(r.body).unwrap() - 5.0).abs() < 1e-9);
    // bottom, top (with the cube standing on it), four slab sides, the cube's top and its two free sides: 9.
    // The cube's flush sides (x = -1 and y = -1) continue the slab's sides in the same plane: one face each.
    let faces = rep.body_faces(r.body).len();
    assert_eq!(faces, 9, "got {faces} faces");
}
