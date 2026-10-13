//! Planar profiles: closed loops of curve segments in a plane.

use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::geom::{Circle, Line};
use kw_nurbs::{Curve3, CurveGeom};
use std::f64::consts::TAU;

#[derive(Debug, Clone, PartialEq)]
pub enum ModelError {
    /// The profile is not a set of closed, consecutive, planar loops.
    BadProfile(&'static str),
    BadParameter(&'static str),
    Construction(String),
}

impl std::fmt::Display for ModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for ModelError {}

/// One segment of a profile loop: the curve over `range`, traversed from `range.0` to `range.1`
/// unless `reversed`.
#[derive(Debug, Clone)]
pub struct ProfileEdge {
    pub curve: CurveGeom,
    /// `range.0 < range.1`.
    pub range: (f64, f64),
    pub reversed: bool,
}

impl ProfileEdge {
    pub fn new(curve: CurveGeom, range: (f64, f64)) -> Self {
        Self {
            curve,
            range,
            reversed: false,
        }
    }

    pub fn start(&self) -> Point3<f64> {
        self.curve.point_at(if self.reversed {
            self.range.1
        } else {
            self.range.0
        })
    }

    pub fn end(&self) -> Point3<f64> {
        self.curve.point_at(if self.reversed {
            self.range.0
        } else {
            self.range.1
        })
    }

    /// Straight segment from `a` to `b`.
    pub fn line(a: Point3<f64>, b: Point3<f64>) -> Self {
        Self::new(
            CurveGeom::Line(Line {
                origin: a,
                dir: b - a,
            }),
            (0.0, 1.0),
        )
    }

    /// Counter-clockwise (about `frame.z`) arc of the circle centred at `frame.origin`.
    pub fn arc(frame: &Frame3, radius: f64, a0: f64, a1: f64) -> Self {
        Self::new(
            CurveGeom::Circle(Circle {
                frame: *frame,
                radius,
            }),
            (a0, a1),
        )
    }
}

#[derive(Debug, Clone)]
pub struct ProfileLoop {
    pub edges: Vec<ProfileEdge>,
}

impl ProfileLoop {
    pub fn new(edges: Vec<ProfileEdge>) -> Self {
        Self { edges }
    }

    /// Closed polygon through `pts`.
    pub fn polygon(pts: &[Point3<f64>]) -> Self {
        let n = pts.len();
        Self {
            edges: (0..n)
                .map(|i| ProfileEdge::line(pts[i], pts[(i + 1) % n]))
                .collect(),
        }
    }

    /// Full circle.
    pub fn circle(frame: &Frame3, radius: f64) -> Self {
        Self {
            edges: vec![ProfileEdge::arc(frame, radius, 0.0, TAU)],
        }
    }

    /// Check consecutiveness and closure; returns the signed area about `normal` (positive = CCW).
    pub fn check(&self, normal: &Vector3<f64>, tol: f64) -> Result<f64, ModelError> {
        let n = self.edges.len();
        if n == 0 {
            return Err(ModelError::BadProfile("empty loop"));
        }
        for i in 0..n {
            if (self.edges[i].end() - self.edges[(i + 1) % n].start()).norm() > tol {
                return Err(ModelError::BadProfile("loop edges are not consecutive"));
            }
        }
        // signed area from sampled points
        let mut pts: Vec<Point3<f64>> = Vec::new();
        for e in &self.edges {
            let (a, b) = if e.reversed {
                (e.range.1, e.range.0)
            } else {
                (e.range.0, e.range.1)
            };
            for k in 0..16 {
                pts.push(e.curve.point_at(a + (b - a) * k as f64 / 16.0));
            }
        }
        let o = pts[0];
        let mut area = Vector3::zeros();
        for i in 0..pts.len() {
            area += (pts[i] - o).cross(&(pts[(i + 1) % pts.len()] - o));
        }
        Ok(0.5 * area.dot(normal))
    }
}

/// An outer loop with optional holes, all in the plane of `plane`.
#[derive(Debug, Clone)]
pub struct Profile {
    pub plane: Frame3,
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
}

impl Profile {
    pub fn new(plane: Frame3, outer: ProfileLoop) -> Self {
        Self {
            plane,
            outer,
            holes: Vec::new(),
        }
    }

    pub fn with_hole(mut self, hole: ProfileLoop) -> Self {
        self.holes.push(hole);
        self
    }

    /// Rectangle `w × h` centred on the plane origin.
    pub fn rectangle(plane: Frame3, w: f64, h: f64) -> Self {
        let p = |x: f64, y: f64| plane.origin + plane.x * x + plane.y * y;
        let pts = [
            p(-w / 2.0, -h / 2.0),
            p(w / 2.0, -h / 2.0),
            p(w / 2.0, h / 2.0),
            p(-w / 2.0, h / 2.0),
        ];
        Self::new(plane, ProfileLoop::polygon(&pts))
    }

    /// Disc of radius `r` centred on the plane origin.
    pub fn disc(plane: Frame3, r: f64) -> Self {
        Self::new(plane, ProfileLoop::circle(&plane, r))
    }

    /// Polygon from plane coordinates.
    pub fn polygon(plane: Frame3, pts2: &[(f64, f64)]) -> Self {
        let pts: Vec<Point3<f64>> = pts2
            .iter()
            .map(|(x, y)| plane.origin + plane.x * *x + plane.y * *y)
            .collect();
        Self::new(plane, ProfileLoop::polygon(&pts))
    }
}

/// A run of consecutive curve segments in a plane that need not enclose anything: the input of the
/// surface (sheet) builders. A closed chain returns to its start.
#[derive(Debug, Clone)]
pub struct Chain {
    pub plane: Frame3,
    pub edges: Vec<ProfileEdge>,
    pub closed: bool,
}

impl Chain {
    pub fn new(plane: Frame3, edges: Vec<ProfileEdge>, closed: bool) -> Self {
        Self {
            plane,
            edges,
            closed,
        }
    }

    /// The points where consecutive edges meet: the start of every edge, then (for an open chain)
    /// the end of the last one. `junctions()[i + 1]` is where edge `i` ends (wrapping if closed).
    pub fn junctions(&self) -> Vec<Point3<f64>> {
        let mut pts: Vec<Point3<f64>> = self.edges.iter().map(|e| e.start()).collect();
        if !self.closed {
            if let Some(last) = self.edges.last() {
                pts.push(last.end());
            }
        }
        pts
    }

    /// Index of the junction at which edge `i` ends.
    pub fn next(&self, i: usize) -> usize {
        if self.closed {
            (i + 1) % self.edges.len()
        } else {
            i + 1
        }
    }

    /// Check that the edges are consecutive (and, if closed, that the last meets the first).
    pub fn check(&self, tol: f64) -> Result<(), ModelError> {
        let n = self.edges.len();
        if n == 0 {
            return Err(ModelError::BadProfile("empty chain"));
        }
        let last = if self.closed { n } else { n - 1 };
        for i in 0..last {
            if (self.edges[i].end() - self.edges[(i + 1) % n].start()).norm() > tol {
                return Err(ModelError::BadProfile("chain edges are not consecutive"));
            }
        }
        Ok(())
    }

    /// Signed area about `normal` of a closed chain (positive = counter-clockwise); 0 if open.
    pub fn signed_area(&self, normal: &Vector3<f64>) -> f64 {
        if !self.closed {
            return 0.0;
        }
        ProfileLoop::new(self.edges.clone())
            .check(normal, f64::INFINITY)
            .unwrap_or(0.0)
    }
}
