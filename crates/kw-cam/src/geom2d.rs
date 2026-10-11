//! Planar regions with islands, offsetting (arcs stay arcs) and conversion of boundaries to cutter moves.

use crate::path::Event;
use crate::{P2, P3};
use cavalier_contours::polyline::{PlineSource, PlineSourceMut, PlineVertex, Polyline};
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions};

pub use cavalier_contours::polyline::Polyline as Pline;

/// A region of the plane bounded by closed loops of lines and arcs: counter-clockwise loops are
/// boundaries, clockwise loops are islands (holes).
#[derive(Debug, Clone)]
pub struct Region {
    pub shape: Shape<f64>,
}

fn closed(pts: &[[f64; 3]]) -> Polyline<f64> {
    let mut p = Polyline::new_closed();
    for v in pts {
        p.add(v[0], v[1], v[2]);
    }
    p
}

impl Region {
    /// Loops given as `(x, y, bulge)` vertices (bulge = tan(θ/4) of the arc to the next vertex).
    pub fn from_loops(loops: &[Vec<[f64; 3]>]) -> Region {
        Region {
            shape: Shape::from_plines(loops.iter().map(|l| closed(l))),
        }
    }

    pub fn polygon(pts: &[P2]) -> Region {
        Region::from_loops(&[pts.iter().map(|p| [p[0], p[1], 0.0]).collect()])
    }

    pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
        Region::polygon(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    }

    pub fn circle(cx: f64, cy: f64, r: f64) -> Region {
        Region::from_loops(&[circle_loop(cx, cy, r)])
    }

    pub fn from_plines(plines: Vec<Polyline<f64>>) -> Region {
        Region {
            shape: Shape::from_plines(plines),
        }
    }

    /// The same region with an island (a loop of any orientation) cut out of it.
    pub fn with_island(self, island: &[[f64; 3]]) -> Region {
        let mut plines: Vec<Polyline<f64>> = self.loops();
        let mut hole = closed(island);
        if hole.area() > 0.0 {
            hole.invert_direction_mut();
        }
        plines.push(hole);
        Region::from_plines(plines)
    }

    /// Offset: positive moves every boundary inwards (islands grow), negative outwards.
    pub fn offset(&self, d: f64) -> Region {
        if d == 0.0 {
            return self.clone();
        }
        Region {
            shape: self.shape.parallel_offset(d, ShapeOffsetOptions::new()),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.shape.ccw_plines.is_empty() && self.shape.cw_plines.is_empty()
    }

    /// All loops, boundaries first.
    pub fn loops(&self) -> Vec<Polyline<f64>> {
        self.shape
            .ccw_plines
            .iter()
            .chain(self.shape.cw_plines.iter())
            .map(|p| p.polyline.clone())
            .collect()
    }

    /// Net area (boundaries minus islands).
    pub fn area(&self) -> f64 {
        self.shape
            .ccw_plines
            .iter()
            .chain(self.shape.cw_plines.iter())
            .map(|p| p.polyline.area())
            .sum()
    }

    pub fn contains(&self, p: P2) -> bool {
        let v = cavalier_contours::core::math::Vector2::new(p[0], p[1]);
        let w: i32 = self
            .shape
            .ccw_plines
            .iter()
            .chain(self.shape.cw_plines.iter())
            .map(|l| l.polyline.winding_number(v))
            .sum();
        w != 0
    }

    pub fn bounds(&self) -> Option<[f64; 4]> {
        let mut b: Option<[f64; 4]> = None;
        for l in self
            .shape
            .ccw_plines
            .iter()
            .chain(self.shape.cw_plines.iter())
        {
            if let Some(e) = l.polyline.extents() {
                b = Some(match b {
                    None => [e.min_x, e.min_y, e.max_x, e.max_y],
                    Some(o) => [
                        o[0].min(e.min_x),
                        o[1].min(e.min_y),
                        o[2].max(e.max_x),
                        o[3].max(e.max_y),
                    ],
                });
            }
        }
        b
    }
}

pub fn circle_loop(cx: f64, cy: f64, r: f64) -> Vec<[f64; 3]> {
    vec![[cx - r, cy, 1.0], [cx + r, cy, 1.0]]
}

/// Centre of the arc from `a` to `b` with the given bulge, and whether it runs counter-clockwise.
pub fn arc_centre(a: P2, b: P2, bulge: f64) -> (P2, bool) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let d = dx.hypot(dy);
    let s = bulge.abs() * d / 2.0;
    let r = (d * d / 4.0 + s * s) / (2.0 * s);
    let n = (-dy / d, dx / d);
    let h = (r - s) * bulge.signum();
    (
        [a[0] + dx / 2.0 + n.0 * h, a[1] + dy / 2.0 + n.1 * h],
        bulge > 0.0,
    )
}

/// Perimeter of a closed polyline.
pub fn perimeter(pl: &Polyline<f64>) -> f64 {
    pl.path_length()
}

/// Append the moves that trace the closed polyline starting at vertex `start`, descending linearly in
/// total length from `z0` to `z1`. The tool is assumed to be at the start vertex at height `z0`.
pub fn trace_loop(
    out: &mut Vec<Event>,
    pl: &Polyline<f64>,
    start: usize,
    z0: f64,
    z1: f64,
    feed: f64,
) {
    let n = pl.vertex_count();
    let total = perimeter(pl).max(1e-12);
    let mut done = 0.0;
    for k in 0..n {
        let v0 = pl.at((start + k) % n);
        let v1 = pl.at((start + k + 1) % n);
        let seg_len = segment_length(v0, v1);
        done += seg_len;
        let z = z0 + (z1 - z0) * (done / total);
        let p: P3 = [v1.x, v1.y, z];
        if v0.bulge.abs() < 1e-12 {
            out.push(Event::Feed { p, feed });
        } else {
            let (c, ccw) = arc_centre([v0.x, v0.y], [v1.x, v1.y], v0.bulge);
            out.push(Event::Arc {
                p,
                centre: c,
                ccw,
                feed,
            });
        }
    }
}

pub fn segment_length(v0: PlineVertex<f64>, v1: PlineVertex<f64>) -> f64 {
    let d = (v1.x - v0.x).hypot(v1.y - v0.y);
    if v0.bulge.abs() < 1e-12 {
        d
    } else {
        let theta = 4.0 * v0.bulge.atan();
        let r = d / (2.0 * (theta / 2.0).sin().abs());
        r * theta.abs()
    }
}

/// Index of the vertex of `pl` nearest to `p`.
pub fn nearest_vertex(pl: &Polyline<f64>, p: P2) -> usize {
    let mut best = (0, f64::INFINITY);
    for i in 0..pl.vertex_count() {
        let v = pl.at(i);
        let d = (v.x - p[0]).hypot(v.y - p[1]);
        if d < best.1 {
            best = (i, d);
        }
    }
    best.0
}

pub fn start_point(pl: &Polyline<f64>, i: usize) -> P2 {
    let v = pl.at(i);
    [v.x, v.y]
}
