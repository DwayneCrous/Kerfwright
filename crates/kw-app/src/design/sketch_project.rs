//! Project and Intersect: turn the model's edges, or its section by the sketch plane, into sketch curves.

use super::geom::PlaneFrame;
use kw_brep::BRep;
use kw_nurbs::{Curve3, CurveGeom};
use kw_sketch::curves::{chain_segments, simplify_chain, Curve2, P2};
use kw_tess::{tessellate_body, TessOptions};
use std::collections::HashSet;
use std::f64::consts::TAU;

fn plane_pt(f: &PlaneFrame, p: [f64; 3]) -> P2 {
    let q = f.to_plane(p);
    (q.0, q.1)
}

fn arr(v: &kw_math::Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dist(a: P2, b: P2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// A key that is the same for the same curve however it was found (the top and bottom edge of a prism
/// project onto each other).
fn key(c: &Curve2) -> String {
    let r = |v: f64| (v * 1e5).round() as i64;
    let p = |p: P2| (r(p.0), r(p.1));
    match c {
        Curve2::Line(a, b) => {
            let (a, b) = (p(*a), p(*b));
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            format!("L{a:?}{b:?}")
        }
        Curve2::Circle { c, r: rad } => format!("C{:?}{}", p(*c), r(*rad)),
        Curve2::Arc { c, start, end } => format!("A{:?}{:?}{:?}", p(*c), p(*start), p(*end)),
        Curve2::Ellipse {
            c,
            major_end,
            minor,
        } => format!("E{:?}{:?}{}", p(*c), p(*major_end), r(*minor)),
        Curve2::Bezier(v) | Curve2::Fit(v) => {
            let (a, b) = (p(v[0]), p(*v.last().unwrap()));
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            format!("S{a:?}{b:?}{}", v.len())
        }
    }
}

/// Every edge of every body of `brep`, projected orthogonally onto the plane: lines stay lines, circles
/// parallel to the plane stay circles (a tilted circle becomes an ellipse), everything else a spline
/// through sample points. Edges that project onto each other are listed once.
pub fn project(brep: &BRep, frame: &PlaneFrame) -> Vec<Curve2> {
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for (_, e) in brep.edges.iter() {
        let Some(cid) = e.curve else { continue };
        let curve = &brep.curves[cid];
        let (a, b) = (brep.vertices[e.start].point, brep.vertices[e.end].point);
        let (pa, pb) = (
            plane_pt(frame, [a.x, a.y, a.z]),
            plane_pt(frame, [b.x, b.y, b.z]),
        );
        let (t0, t1) = e.param_range;
        let full = e.start == e.end;
        let curves: Vec<Curve2> = match curve {
            CurveGeom::Line(_) => {
                if dist(pa, pb) < 1e-9 {
                    continue;
                }
                vec![Curve2::Line(pa, pb)]
            }
            CurveGeom::Circle(c) => {
                let axis = arr(&c.frame.z);
                let centre = plane_pt(
                    frame,
                    [c.frame.origin.x, c.frame.origin.y, c.frame.origin.z],
                );
                let tilt = dot(axis, frame.n);
                if (tilt.abs() - 1.0).abs() < 1e-9 {
                    if full || (t1 - t0 - TAU).abs() < 1e-9 {
                        vec![Curve2::Circle {
                            c: centre,
                            r: c.radius,
                        }]
                    } else if tilt > 0.0 {
                        vec![Curve2::Arc {
                            c: centre,
                            start: pa,
                            end: pb,
                        }]
                    } else {
                        // seen from the other side the sense of rotation is reversed
                        vec![Curve2::Arc {
                            c: centre,
                            start: pb,
                            end: pa,
                        }]
                    }
                } else if full || (t1 - t0 - TAU).abs() < 1e-9 {
                    if tilt.abs() < 1e-9 {
                        // seen edge-on: a line along the circle's diameter
                        let d = cross(axis, frame.n);
                        let n = dot(d, d).sqrt().max(1e-300);
                        let u = (
                            dot(d, frame.x) / n * c.radius,
                            dot(d, frame.y) / n * c.radius,
                        );
                        vec![Curve2::Line(
                            (centre.0 - u.0, centre.1 - u.1),
                            (centre.0 + u.0, centre.1 + u.1),
                        )]
                    } else {
                        // the axis of the tilt keeps its length; the other shrinks by cos(tilt)
                        let d = cross(axis, frame.n);
                        let n = dot(d, d).sqrt();
                        let u = (
                            dot(d, frame.x) / n * c.radius,
                            dot(d, frame.y) / n * c.radius,
                        );
                        vec![Curve2::Ellipse {
                            c: centre,
                            major_end: (centre.0 + u.0, centre.1 + u.1),
                            minor: c.radius * tilt.abs(),
                        }]
                    }
                } else {
                    sampled(curve, t0, t1, frame)
                }
            }
            _ => sampled(curve, t0, t1, frame),
        };
        for c in curves {
            if seen.insert(key(&c)) {
                out.push(c);
            }
        }
    }
    out
}

fn sampled(curve: &CurveGeom, t0: f64, t1: f64, frame: &PlaneFrame) -> Vec<Curve2> {
    let pts: Vec<P2> = (0..=24)
        .map(|i| {
            let p = curve.point_at(t0 + (t1 - t0) * i as f64 / 24.0);
            plane_pt(frame, [p.x, p.y, p.z])
        })
        .collect();
    let (a, z) = (pts[0], pts[pts.len() - 1]);
    // all on one line: a line
    let straight = pts.iter().all(|p| {
        let l = dist(a, z).max(1e-12);
        ((p.0 - a.0) * (z.1 - a.1) - (p.1 - a.1) * (z.0 - a.0)).abs() / l < 1e-7
    });
    if straight {
        if dist(a, z) < 1e-9 {
            return Vec::new();
        }
        return vec![Curve2::Line(a, z)];
    }
    vec![Curve2::Fit(pts)]
}

/// Where the plane cuts the bodies: the section outline, with straight runs and round runs recognised.
pub fn intersect(brep: &BRep, frame: &PlaneFrame) -> Vec<Curve2> {
    let opts = TessOptions {
        chord_tol: 0.005,
        angle_tol: 0.1,
    };
    let mut segs: Vec<(P2, P2)> = Vec::new();
    for body in brep.bodies.keys() {
        let Ok(t) = tessellate_body(brep, body, &opts) else {
            continue;
        };
        let m = &t.mesh;
        for tri in m.indices.as_chunks::<3>().0 {
            let p: Vec<[f64; 3]> = tri
                .iter()
                .map(|i| {
                    let q = m.positions[*i as usize];
                    [q.x, q.y, q.z]
                })
                .collect();
            let d: Vec<f64> = p.iter().map(|q| frame.to_plane(*q).2).collect();
            // vertices on the plane count as below it, so a cut through vertices still yields segments
            let above: Vec<bool> = d.iter().map(|v| *v > 1e-9).collect();
            if above.iter().all(|a| *a) || above.iter().all(|a| !*a) {
                continue;
            }
            let mut hits = Vec::new();
            for (i, j) in [(0, 1), (1, 2), (2, 0)] {
                if above[i] != above[j] {
                    let t = d[i] / (d[i] - d[j]);
                    hits.push(plane_pt(
                        frame,
                        [
                            p[i][0] + (p[j][0] - p[i][0]) * t,
                            p[i][1] + (p[j][1] - p[i][1]) * t,
                            p[i][2] + (p[j][2] - p[i][2]) * t,
                        ],
                    ));
                }
            }
            if hits.len() == 2 {
                segs.push((hits[0], hits[1]));
            }
        }
    }
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for chain in chain_segments(&segs, 1e-6) {
        for c in simplify_chain(&chain, 0.02) {
            if seen.insert(key(&c)) {
                out.push(c);
            }
        }
    }
    out
}
