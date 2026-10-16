#![cfg(feature = "vertex-blend")]
#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Vertex blends: the rolling-ball sphere patch where three equal-radius fillets meet.
use kw_blend::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use std::f64::consts::PI;

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn all_edges(rep: &BRep, body: BodyId) -> Vec<EdgeId> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if seen.insert(e) {
                    out.push(e);
                }
            }
        }
    }
    out
}

/// Volume of the box `[0,a]×[0,b]×[0,c]` with all edges rounded by `r` and all corners by spheres
/// (the parallel body of the inner box, Steiner's formula).
fn rounded_box_volume(a: f64, b: f64, c: f64, r: f64) -> f64 {
    let (x, y, z) = (a - 2.0 * r, b - 2.0 * r, c - 2.0 * r);
    x * y * z
        + 2.0 * r * (x * y + y * z + x * z)
        + PI * r * r * (x + y + z)
        + 4.0 / 3.0 * PI * r * r * r
}

fn mk_box(rep: &mut BRep, a: f64, b: f64, c: f64) -> BodyId {
    let f = Frame3 {
        origin: Point3::new(0.5 * a, 0.5 * b, 0.5 * c),
        ..world()
    };
    make_box(rep, &f, a, b, c).body
}

#[test]
fn one_corner_of_a_box_gets_a_sphere_patch() {
    // box 4 x 3 x 2; fillet the three edges meeting at the origin corner
    let r = 0.5;
    let mut rep = BRep::new();
    let b = mk_box(&mut rep, 4.0, 3.0, 2.0);
    let corner = rep
        .vertices
        .iter()
        .find(|(_, v)| v.point.coords.norm() < 1e-9)
        .map(|(id, _)| id)
        .unwrap();
    let edges: Vec<EdgeId> = rep
        .edges
        .iter()
        .filter(|(_, e)| e.start == corner || e.end == corner)
        .map(|(id, _)| id)
        .collect();
    assert_eq!(edges.len(), 3);
    let res = fillet(&mut rep, b, &edges, r).unwrap_or_else(|e| panic!("{e}"));
    assert!(rep.validate().is_ok(), "{:?}", rep.validate());
    // the three cylindrical strips plus the corner: material removed by the opening of the corner
    // x,y,z >= 0 by a ball of radius r, restricted to the box
    // = three cylinder spandrels over the lengths beyond the corner cube + the corner cube spandrel
    let (lx, ly, lz) = (4.0 - r, 3.0 - r, 2.0 - r);
    let span = r * r * (1.0 - PI / 4.0);
    let cube = r * r * r - (4.0 / 3.0 * PI * r * r * r) / 8.0 - 3.0 * span * r;
    // cube [0,r]^3 minus the ball octant is the corner cube spandrel; the three edge spandrels inside
    // that cube are counted once by `cube` above (subtracting 3 span r removes them from it)
    let removed = span * (lx + ly + lz) + (r * r * r - PI / 6.0 * r * r * r) - 0.0 * cube;
    let vol = rep.volume(res.body).unwrap();
    let expect = 24.0 - removed;
    assert!(
        rel(vol, expect) < 1e-9,
        "corner volume {vol} vs {expect} (removed {removed})"
    );
}

#[test]
fn all_edges_of_a_box_give_the_rounded_box() {
    let r = 0.4;
    let mut rep = BRep::new();
    let b = mk_box(&mut rep, 4.0, 3.0, 2.0);
    let edges = all_edges(&rep, b);
    assert_eq!(edges.len(), 12);
    let res = fillet(&mut rep, b, &edges, r).unwrap_or_else(|e| panic!("{e}"));
    assert!(rep.validate().is_ok(), "{:?}", rep.validate());
    let vol = rep.volume(res.body).unwrap();
    let expect = rounded_box_volume(4.0, 3.0, 2.0, r);
    assert!(rel(vol, expect) < 1e-9, "volume {vol} vs {expect}");
    // 6 planes + 12 cylinders + 8 spheres
    assert_eq!(rep.body_faces(res.body).len(), 26);
}
