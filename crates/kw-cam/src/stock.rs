//! The stock a setup starts from: a box or a cylinder around the workpiece, an explicit block, or what an
//! earlier setup left. Everything is in the work coordinate system of the setup.

use crate::bvh::{Aabb, TriMesh};
use crate::P3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum StockSpec {
    /// The bounding box of the workpiece, grown by `side` in x and y and by `top` and `bottom` in z.
    Box { side: f64, top: f64, bottom: f64 },
    /// A round bar along work axis `axis` (0 x, 1 y, 2 z) that just contains the workpiece, grown by
    /// `margin` on the radius and by `ends` at both ends.
    Cylinder { axis: u8, margin: f64, ends: f64 },
    /// A block given by its corners.
    Explicit { min: P3, max: P3 },
    /// What the previous setup left, moved into this setup's frame.
    Previous,
}

impl Default for StockSpec {
    fn default() -> Self {
        StockSpec::Box {
            side: 0.0,
            top: 0.0,
            bottom: 0.0,
        }
    }
}

/// The shape of the stock as a closed mesh and its box.
pub struct StockShape {
    pub mesh: TriMesh,
    pub min: P3,
    pub max: P3,
}

impl StockShape {
    fn of_box(min: P3, max: P3) -> StockShape {
        StockShape {
            mesh: TriMesh::from_box(min, max),
            min,
            max,
        }
    }
}

/// A closed prism-approximated cylinder along `axis`.
pub fn cylinder_mesh(axis: usize, centre: [f64; 2], r: f64, lo: f64, hi: f64) -> TriMesh {
    const N: usize = 96;
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let pt = |a: f64, k: usize| -> P3 {
        let t = std::f64::consts::TAU * k as f64 / N as f64;
        let mut p = [0.0; 3];
        p[axis] = a;
        p[u] = centre[0] + r * t.cos();
        p[v] = centre[1] + r * t.sin();
        p
    };
    let mid = |a: f64| -> P3 {
        let mut p = [0.0; 3];
        p[axis] = a;
        p[u] = centre[0];
        p[v] = centre[1];
        p
    };
    let mut t = Vec::new();
    for k in 0..N {
        let (a, b) = (k, (k + 1) % N);
        t.push([pt(lo, a), pt(lo, b), pt(hi, b)]);
        t.push([pt(lo, a), pt(hi, b), pt(hi, a)]);
        t.push([mid(hi), pt(hi, a), pt(hi, b)]);
        t.push([mid(lo), pt(lo, b), pt(lo, a)]);
    }
    TriMesh::new(t)
}

impl StockSpec {
    /// The stock around `part` (the workpiece in work coordinates). `Previous` cannot be made from the part
    /// alone: the caller supplies the earlier stock, and this answers `None`.
    pub fn shape(&self, part: &TriMesh) -> Option<StockShape> {
        let bb = part.bounds();
        if bb.min[0] > bb.max[0] {
            return None;
        }
        match *self {
            StockSpec::Box { side, top, bottom } => Some(StockShape::of_box(
                [bb.min[0] - side, bb.min[1] - side, bb.min[2] - bottom],
                [bb.max[0] + side, bb.max[1] + side, bb.max[2] + top],
            )),
            StockSpec::Cylinder { axis, margin, ends } => {
                let a = (axis as usize).min(2);
                let (u, v) = ((a + 1) % 3, (a + 2) % 3);
                let centre = [0.5 * (bb.min[u] + bb.max[u]), 0.5 * (bb.min[v] + bb.max[v])];
                let mut r2 = 0.0f64;
                for t in &part.tris {
                    for p in t {
                        r2 = r2.max((p[u] - centre[0]).powi(2) + (p[v] - centre[1]).powi(2));
                    }
                }
                let r = r2.sqrt() + margin;
                let (lo, hi) = (bb.min[a] - ends, bb.max[a] + ends);
                let mut min = [0.0; 3];
                let mut max = [0.0; 3];
                min[a] = lo;
                max[a] = hi;
                min[u] = centre[0] - r;
                max[u] = centre[0] + r;
                min[v] = centre[1] - r;
                max[v] = centre[1] + r;
                Some(StockShape {
                    mesh: cylinder_mesh(a, centre, r, lo, hi),
                    min,
                    max,
                })
            }
            StockSpec::Explicit { min, max } => Some(StockShape::of_box(min, max)),
            StockSpec::Previous => None,
        }
    }
}

/// The bounding box of the triangles.
pub fn bounds_of(tris: &[[P3; 3]]) -> Aabb {
    let mut b = Aabb::empty();
    for t in tris {
        for p in t {
            b.grow(*p);
        }
    }
    b
}
