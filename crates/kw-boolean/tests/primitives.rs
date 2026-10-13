#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Boolean operations between the analytic primitives: structural validity (B-rep + tessellation),
//! inclusion–exclusion of volumes, and closed-form volumes where available.

use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_tess::TessOptions;
use std::f64::consts::PI;

fn at(x: f64, y: f64, z: f64) -> Frame3 {
    Frame3 {
        origin: Point3::new(x, y, z),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn tilted(x: f64, y: f64, z: f64, n: [f64; 3]) -> Frame3 {
    Frame3::from_origin_normal(Point3::new(x, y, z), Vector3::new(n[0], n[1], n[2]))
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

type Builder = Box<dyn Fn(&mut BRep) -> BodyId>;

fn bx(f: Frame3, x: f64, y: f64, z: f64) -> Builder {
    Box::new(move |r| make_box(r, &f, x, y, z).body)
}
fn cy(f: Frame3, rad: f64, h: f64) -> Builder {
    Box::new(move |r| make_cylinder(r, &f, rad, h).body)
}
fn co(f: Frame3, r0: f64, r1: f64, h: f64) -> Builder {
    Box::new(move |r| make_cone(r, &f, r0, r1, h).body)
}
fn sp(f: Frame3, rad: f64) -> Builder {
    Box::new(move |r| make_sphere(r, &f, rad).body)
}
fn to(f: Frame3, big: f64, small: f64) -> Builder {
    Box::new(move |r| make_torus(r, &f, big, small).body)
}

/// Exact B-rep validity plus a tessellation sanity check: the mesh must build and enclose the exact
/// volume within the chordal error. (A CDT may legitimately connect the same two boundary vertices
/// from two different faces, which a strict mesh-edge-manifold test would flag; that is a mesher
/// property, not a B-rep defect.)
fn tess_ok(rep: &BRep, body: BodyId, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let opts = TessOptions {
        chord_tol: 0.01,
        angle_tol: 0.3,
    };
    let t = kw_tess::tessellate_body(rep, body, &opts)
        .unwrap_or_else(|e| panic!("{what}: tessellation failed: {e:?}"));
    let exact = rep.volume(body).unwrap();
    let area = rep.area(body).unwrap();
    let vm = t.mesh.signed_volume();
    assert!(
        (vm - exact).abs() <= 2.0 * opts.chord_tol * area + 1e-9,
        "{what}: mesh volume {vm} vs exact {exact}"
    );
}

/// Returns the three volumes (union, intersection, difference) or the error of the first failing op.
fn run(
    name: &str,
    a: &Builder,
    b: &Builder,
) -> Result<(f64, f64, f64, f64, f64), (Op, BooleanError)> {
    let mut vols = [0.0; 3];
    let (mut va, mut vb) = (0.0, 0.0);
    for (i, op) in [Op::Union, Op::Intersect, Op::Subtract]
        .into_iter()
        .enumerate()
    {
        let mut rep = BRep::new();
        let (ba, bb) = (a(&mut rep), b(&mut rep));
        va = rep.volume(ba).unwrap();
        vb = rep.volume(bb).unwrap();
        match boolean(&mut rep, ba, bb, op, &BoolOptions::default()) {
            Ok(r) => {
                tess_ok(&rep, r.body, &format!("{name} {op:?}"));
                vols[i] = rep.volume(r.body).unwrap();
            }
            Err(BooleanError::Empty) => vols[i] = 0.0,
            Err(e) => return Err((op, e)),
        }
    }
    Ok((vols[0], vols[1], vols[2], va, vb))
}

fn check_identities(name: &str, a: &Builder, b: &Builder) -> (f64, f64, f64, f64, f64) {
    let (u, i, d, va, vb) =
        run(name, a, b).unwrap_or_else(|(op, e)| panic!("{name}: {op:?} failed: {e}"));
    assert!(
        rel(u + i, va + vb) < 1e-9,
        "{name}: V(A∪B)+V(A∩B) = {} but V(A)+V(B) = {}",
        u + i,
        va + vb
    );
    assert!(
        rel(d, va - i) < 1e-9,
        "{name}: V(A−B) = {d} but V(A)−V(A∩B) = {}",
        va - i
    );
    (u, i, d, va, vb)
}

#[test]
fn box_with_cylinder_hole() {
    // cylinder (r=0.5) drilled through a 2×2×2 box along z, off-centre
    let a = bx(at(0.0, 0.0, 0.0), 2.0, 2.0, 2.0);
    let b = cy(at(0.3, 0.2, -2.0), 0.5, 4.0);
    let (_, i, d, _, _) = check_identities("box-hole", &a, &b);
    assert!(rel(i, PI * 0.25 * 2.0) < 1e-9, "intersection volume {i}");
    assert!(rel(d, 8.0 - PI * 0.25 * 2.0) < 1e-9);
}

#[test]
fn cylinder_end_inside_box() {
    // blind hole: cylinder enters through the top face and stops inside
    let a = bx(at(0.0, 0.0, 0.0), 2.0, 2.0, 2.0);
    let b = cy(at(0.2, -0.1, 0.0), 0.6, 3.0);
    let (_, i, _, _, _) = check_identities("blind", &a, &b);
    assert!(rel(i, PI * 0.36 * 1.0) < 1e-9, "{i}");
}

#[test]
fn sphere_and_box() {
    // sphere poking out of one face of the box
    let a = bx(at(0.0, 0.0, 0.0), 2.0, 2.0, 2.0);
    let b = sp(at(0.3, 0.15, 0.9), 0.8);
    check_identities("sphere-box", &a, &b);
}

#[test]
fn sphere_sphere_and_cylinder_cylinder() {
    let a = sp(at(0.0, 0.0, 0.0), 1.0);
    let b = sp(at(0.9, 0.2, 0.1), 0.8);
    let (_, i, _, _, _) = check_identities("sphere-sphere", &a, &b);
    assert!(i > 0.0);
    // crossing cylinders of different radii (Steinmetz-like)
    let c1 = cy(at(0.0, 0.0, -1.5), 0.9, 3.0);
    let c2 = cy(tilted(-1.5, 0.1, 0.0, [1.0, 0.0, 0.0]), 0.5, 3.0);
    check_identities("cyl-cyl", &c1, &c2);
}

#[test]
fn cone_torus_combinations() {
    let a = bx(at(0.0, 0.0, 0.0), 3.0, 3.0, 3.0);
    let b = co(at(0.1, 0.0, -1.0), 1.0, 0.3, 2.5);
    check_identities("cone-box", &a, &b);
    let t = to(at(0.0, 0.0, 0.0), 1.0, 0.35);
    let c = bx(at(0.5, 0.0, 0.0), 1.2, 4.0, 4.0);
    check_identities("torus-box", &t, &c);
}

#[test]
fn contained_disjoint_and_identical_cases() {
    let big = bx(at(0.0, 0.0, 0.0), 4.0, 4.0, 4.0);
    let small = bx(at(0.2, 0.1, 0.3), 1.0, 1.0, 1.0);
    // small fully inside big: union = big, intersection = small, difference = big with a void
    let (u, i, d, va, vb) = check_identities("contained", &big, &small);
    assert!(rel(u, va) < 1e-10 && rel(i, vb) < 1e-10 && rel(d, va - vb) < 1e-10);
    // disjoint
    let far = bx(at(10.0, 0.0, 0.0), 1.0, 1.0, 1.0);
    let (u, i, d, va, vb) = check_identities("disjoint", &big, &far);
    assert!(rel(u, va + vb) < 1e-10 && i == 0.0 && rel(d, va) < 1e-10);
    // sphere inside box: hollow result
    let s = sp(at(0.0, 0.0, 0.0), 1.0);
    let (_, _, d, va, vb) = check_identities("sphere-in-box", &big, &s);
    assert!(rel(d, va - vb) < 1e-9);
}
