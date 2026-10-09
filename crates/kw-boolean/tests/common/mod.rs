//! Shared random case generator for the Boolean fuzz tests.
#![allow(dead_code, unused_imports, clippy::type_complexity)]
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use std::f64::consts::TAU;

pub struct Rng(pub u64);
impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut r = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03);
        for _ in 0..4 {
            r.next();
        }
        r
    }
    pub fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    pub fn pick(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n - 1)
    }
    pub fn unit(&mut self) -> Vector3<f64> {
        loop {
            let v = Vector3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if v.norm() > 0.2 && v.norm() < 1.0 {
                return v.normalize();
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Shape {
    Box(f64, f64, f64),
    Cyl(f64, f64),
    Cone(f64, f64, f64),
    Sphere(f64),
    Torus(f64, f64),
}

pub fn build(rep: &mut BRep, s: Shape, f: &Frame3) -> BodyId {
    match s {
        Shape::Box(x, y, z) => make_box(rep, f, x, y, z).body,
        Shape::Cyl(r, h) => make_cylinder(rep, f, r, h).body,
        Shape::Cone(r0, r1, h) => make_cone(rep, f, r0, r1, h).body,
        Shape::Sphere(r) => make_sphere(rep, f, r).body,
        Shape::Torus(a, b) => make_torus(rep, f, a, b).body,
    }
}

pub fn random_shape(r: &mut Rng) -> Shape {
    match r.pick(5) {
        0 => Shape::Box(r.range(0.6, 3.0), r.range(0.6, 3.0), r.range(0.6, 3.0)),
        1 => Shape::Cyl(r.range(0.3, 1.5), r.range(0.6, 3.0)),
        2 => Shape::Cone(
            r.range(0.5, 1.5),
            if r.next() < 0.3 {
                0.0
            } else {
                r.range(0.2, 1.2)
            },
            r.range(0.8, 3.0),
        ),
        3 => Shape::Sphere(r.range(0.4, 1.6)),
        _ => {
            let big = r.range(1.0, 2.0);
            Shape::Torus(big, r.range(0.25, 0.6 * big.min(1.0) + 0.1))
        }
    }
}

pub fn random_frame(r: &mut Rng, spread: f64) -> Frame3 {
    let n = r.unit();
    let mut f = Frame3::from_origin_normal(
        Point3::new(
            r.range(-spread, spread),
            r.range(-spread, spread),
            r.range(-spread, spread),
        ),
        n,
    );
    let a = r.range(0.0, TAU);
    let (c, s) = (a.cos(), a.sin());
    let (x, y) = (f.x * c + f.y * s, -f.x * s + f.y * c);
    f.x = x;
    f.y = y;
    f
}

pub fn axis_frame(x: f64, y: f64, z: f64) -> Frame3 {
    Frame3 {
        origin: Point3::new(x, y, z),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

#[derive(Debug, Clone)]
pub struct Case {
    pub category: &'static str,
    /// Cases where a valid result is expected for every operation.
    pub expect_valid: bool,
    pub a: (Shape, Frame3),
    pub b: (Shape, Frame3),
}

pub fn gen_case(seed: u64) -> Case {
    let mut r = Rng::new(seed);
    let cat = r.pick(10);
    match cat {
        // generic overlapping pairs with arbitrary rigid placements
        0..=3 => Case {
            category: "generic",
            expect_valid: true,
            a: (random_shape(&mut r), random_frame(&mut r, 0.3)),
            b: (random_shape(&mut r), random_frame(&mut r, 1.2)),
        },
        // aligned boxes sharing / flush with faces
        4 => {
            let (ax, ay, az) = (r.range(1.0, 3.0), r.range(1.0, 3.0), r.range(1.0, 3.0));
            let (bx, by, bz) = (r.range(0.5, 3.0), r.range(0.5, 3.0), r.range(0.5, 3.0));
            // B's centre placed so that one face plane of B coincides with one of A
            let axis = r.pick(3);
            let side = if r.next() < 0.5 { 1.0 } else { -1.0 };
            let touching = r.next() < 0.5; // externally touching vs flush (same side)
            let aext = [ax, ay, az];
            let bext = [bx, by, bz];
            let mut c = [r.range(-0.6, 0.6), r.range(-0.6, 0.6), r.range(-0.6, 0.6)];
            c[axis] = if touching {
                side * (aext[axis] + bext[axis]) * 0.5
            } else {
                side * (aext[axis] - bext[axis]) * 0.5
            };
            Case {
                category: "coplanar-boxes",
                expect_valid: true,
                a: (Shape::Box(ax, ay, az), axis_frame(0.0, 0.0, 0.0)),
                b: (Shape::Box(bx, by, bz), axis_frame(c[0], c[1], c[2])),
            }
        }
        // cylinder standing on / flush with a box face
        5 => {
            let (ax, ay, az) = (r.range(1.5, 3.0), r.range(1.5, 3.0), r.range(1.0, 2.5));
            let rad = r.range(0.2, 0.7);
            let h = r.range(0.5, 2.0);
            let flush_inside = r.next() < 0.5;
            let (cx, cy) = (r.range(-0.5, 0.5), r.range(-0.5, 0.5));
            let z0 = if flush_inside {
                az / 2.0 - h.min(az * 0.9)
            } else {
                az / 2.0
            };
            Case {
                category: "cyl-on-box",
                expect_valid: true,
                a: (Shape::Box(ax, ay, az), axis_frame(0.0, 0.0, 0.0)),
                b: (Shape::Cyl(rad, h.min(az * 0.9)), axis_frame(cx, cy, z0)),
            }
        }
        // coaxial cylinders: stacked (shared cap plane) or nested with equal radius (coincident side)
        6 => {
            let (r1, h1) = (r.range(0.5, 1.5), r.range(0.8, 2.0));
            let (r2, h2) = (
                if r.next() < 0.5 {
                    r1
                } else {
                    r.range(0.3, 1.5)
                },
                r.range(0.5, 2.0),
            );
            let overlap = r.next() < 0.5;
            let z2 = if overlap { r.range(0.2, h1 - 0.1) } else { h1 };
            Case {
                category: "coaxial-cyls",
                expect_valid: true,
                a: (Shape::Cyl(r1, h1), axis_frame(0.0, 0.0, 0.0)),
                b: (Shape::Cyl(r2, h2), axis_frame(0.0, 0.0, z2)),
            }
        }
        // contained / disjoint
        7 => {
            let big = Shape::Box(4.0, 4.0, 4.0);
            let small = match r.pick(3) {
                0 => Shape::Box(r.range(0.5, 1.5), r.range(0.5, 1.5), r.range(0.5, 1.5)),
                1 => Shape::Sphere(r.range(0.3, 1.0)),
                _ => Shape::Cyl(r.range(0.3, 0.8), r.range(0.5, 1.5)),
            };
            let far = r.next() < 0.4;
            let p = if far { 5.5 } else { 0.0 };
            Case {
                category: if far { "disjoint" } else { "contained" },
                expect_valid: true,
                a: (big, random_frame(&mut r, 0.0)),
                b: (small, {
                    let mut f = random_frame(&mut r, 0.4);
                    f.origin.x += p;
                    f
                }),
            }
        }
        // near-miss: overlap or gap of 1e-6 .. 1e-3
        8 => {
            let eps = 10f64.powf(r.range(-6.0, -3.0)) * if r.next() < 0.5 { 1.0 } else { -1.0 };
            let s = r.range(1.0, 2.0);
            Case {
                category: "near-miss",
                expect_valid: true,
                a: (Shape::Box(s, s, s), axis_frame(0.0, 0.0, 0.0)),
                b: (Shape::Sphere(r.range(0.4, 1.0)), {
                    let rad = 0.0;
                    let _ = rad;
                    axis_frame(r.range(-0.2, 0.2), r.range(-0.2, 0.2), s / 2.0 + 0.7 + eps)
                }),
            }
        }
        // tangent contacts (a valid result is generally not expected)
        _ => {
            let kind = r.pick(3);
            let (a, b) = match kind {
                0 => (
                    (Shape::Box(2.0, 2.0, 2.0), axis_frame(0.0, 0.0, 0.0)),
                    (Shape::Sphere(0.7), axis_frame(0.1, 0.2, 1.7)),
                ),
                1 => (
                    (Shape::Sphere(1.0), axis_frame(0.0, 0.0, 0.0)),
                    (Shape::Sphere(0.6), axis_frame(1.6, 0.0, 0.0)),
                ),
                _ => (
                    (Shape::Box(2.0, 2.0, 2.0), axis_frame(0.0, 0.0, 0.0)),
                    (Shape::Cyl(0.5, 3.0), axis_frame(0.5, -0.5, -1.5)),
                ),
            };
            Case {
                category: "tangent",
                expect_valid: false,
                a,
                b,
            }
        }
    }
}
