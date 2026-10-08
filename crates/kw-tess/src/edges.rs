use crate::mesh::EdgePolylines;
use crate::{TessError, TessOptions};
use kw_brep::{BRep, EdgeId, VertexId};
use kw_nurbs::{Curve3, CurveGeom};
use nalgebra::Point3;
use std::collections::HashMap;
use std::f64::consts::PI;

pub(crate) struct EdgeSamples {
    pub params: Vec<f64>,
    pub points: Vec<Point3<f64>>,
    pub weld: Vec<u32>,
}

pub(crate) struct EdgeContext {
    pub samples: HashMap<EdgeId, EdgeSamples>,
    pub vertex_weld: HashMap<VertexId, u32>,
    pub next_weld: u32,
}

/// Maximum turning angle per segment of a circle of `radius` so that both
/// the chordal and angular tolerances hold (never more than 120° per segment).
pub(crate) fn circle_step(radius: f64, opts: &TessOptions) -> f64 {
    let chord = if opts.chord_tol >= radius {
        PI
    } else {
        2.0 * (1.0 - opts.chord_tol / radius).acos()
    };
    chord.min(opts.angle_tol).clamp(1e-4, 2.0 * PI / 3.0)
}

fn adaptive(
    curve: &CurveGeom,
    t0: f64,
    t1: f64,
    opts: &TessOptions,
    depth: u32,
    out: &mut Vec<f64>,
) {
    let (p0, p1) = (curve.point_at(t0), curve.point_at(t1));
    let tm = 0.5 * (t0 + t1);
    let pm = curve.point_at(tm);
    let chord = p1 - p0;
    let len2 = chord.norm_squared();
    let dev = if len2 > 0.0 {
        let s = (pm - p0).dot(&chord) / len2;
        (pm - (p0 + chord * s.clamp(0.0, 1.0))).norm()
    } else {
        (pm - p0).norm()
    };
    let (d0, d1) = (curve.derivative(t0), curve.derivative(t1));
    let ang = if d0.norm() > 0.0 && d1.norm() > 0.0 {
        d0.normalize().dot(&d1.normalize()).clamp(-1.0, 1.0).acos()
    } else {
        0.0
    };
    if depth >= 14 || (depth >= 2 && dev <= opts.chord_tol && ang <= opts.angle_tol) {
        out.push(t1);
    } else {
        adaptive(curve, t0, tm, opts, depth + 1, out);
        adaptive(curve, tm, t1, opts, depth + 1, out);
    }
}

/// Parameters (increasing from `range.0` to `range.1`) at which to sample an edge curve.
pub(crate) fn discretize(curve: &CurveGeom, range: (f64, f64), opts: &TessOptions) -> Vec<f64> {
    let (a, b) = range;
    let lerp = |n: usize| {
        (0..=n)
            .map(|i| a + (b - a) * i as f64 / n as f64)
            .collect::<Vec<_>>()
    };
    match curve {
        CurveGeom::Line(_) => vec![a, b],
        CurveGeom::Circle(c) => {
            let n = (((b - a).abs()) / circle_step(c.radius, opts))
                .ceil()
                .max(1.0) as usize;
            lerp(n)
        }
        CurveGeom::Ellipse(e) => {
            let n = (((b - a).abs()) / circle_step(e.major.max(e.minor), opts))
                .ceil()
                .max(1.0) as usize;
            lerp(n)
        }
        _ => {
            let mut out = vec![a];
            let n0 = 4;
            for i in 0..n0 {
                let t0 = a + (b - a) * i as f64 / n0 as f64;
                let t1 = a + (b - a) * (i + 1) as f64 / n0 as f64;
                adaptive(curve, t0, t1, opts, 0, &mut out);
            }
            out
        }
    }
}

impl EdgeContext {
    pub fn build(brep: &BRep, edge_keys: &[EdgeId], opts: &TessOptions) -> Result<Self, TessError> {
        let mut vertex_weld = HashMap::new();
        let mut next = 0u32;
        for v in brep.vertices.keys() {
            vertex_weld.insert(v, next);
            next += 1;
        }
        let mut samples = HashMap::new();
        for &eid in edge_keys {
            let e = &brep.edges[eid];
            let Some(cid) = e.curve else { continue };
            let curve = brep.curves.get(cid).ok_or_else(|| {
                TessError::BadTopology(format!("edge {eid:?} has dangling curve"))
            })?;
            let (vs, ve) = match (brep.vertices.get(e.start), brep.vertices.get(e.end)) {
                (Some(a), Some(b)) => (a.point, b.point),
                _ => {
                    return Err(TessError::BadTopology(format!(
                        "edge {eid:?} has dangling vertex"
                    )))
                }
            };
            let params = discretize(curve, e.param_range, opts);
            let n = params.len();
            let mut points = Vec::with_capacity(n);
            let mut weld = Vec::with_capacity(n);
            for (i, &t) in params.iter().enumerate() {
                if i == 0 {
                    points.push(vs);
                    weld.push(vertex_weld[&e.start]);
                } else if i == n - 1 {
                    points.push(ve);
                    weld.push(vertex_weld[&e.end]);
                } else {
                    points.push(curve.point_at(t));
                    weld.push(next);
                    next += 1;
                }
            }
            samples.insert(
                eid,
                EdgeSamples {
                    params,
                    points,
                    weld,
                },
            );
        }
        Ok(Self {
            samples,
            vertex_weld,
            next_weld: next,
        })
    }

    pub fn polylines(&self, edge_keys: &[EdgeId]) -> EdgePolylines {
        let mut out = EdgePolylines::default();
        for (i, e) in edge_keys.iter().enumerate() {
            if let Some(s) = self.samples.get(e) {
                out.ranges
                    .push((i as u32, out.points.len() as u32, s.points.len() as u32));
                out.points.extend(s.points.iter().copied());
            }
        }
        out
    }
}
