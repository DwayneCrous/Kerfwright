//! The points and edges of the model a dimension can be attached to, as they appear in a view.

use crate::dim::PointKind;
use crate::projection::{ViewFrame, V3};
use crate::scene::{flatten, Layer, Scene2d, Shape, P};
use kw_brep::{BRep, BodyId, EdgeId};
use kw_nurbs::CurveGeom;
use std::collections::HashSet;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CandKind {
    /// A point of an edge: an end, the middle of a straight edge, the centre of a circle.
    Point(PointKind),
    /// A whole circular edge (for diameters and radii).
    Circle,
    /// A whole straight edge (for angles).
    Line,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Cand {
    /// Index of the body in the list given to [`candidates`].
    pub body: usize,
    pub edge: EdgeId,
    pub kind: CandKind,
    /// Where it is in the view, relative to the view origin (sheet mm).
    pub p2: P,
    pub p3: V3,
    /// Centre, radius and axis of a circular edge.
    pub circle: Option<(V3, f64, V3)>,
    /// The ends of a straight edge, in 3-D and in the view.
    pub line: Option<(V3, V3)>,
    pub line2: Option<(P, P)>,
}

fn v3(p: &kw_math::Point3<f64>) -> V3 {
    [p.x, p.y, p.z]
}

/// Every attachable point and edge of the bodies, seen in a view at `scale` whose geometry is
/// centred on `centre` (as stored with the view). Seams and degenerate edges are left out.
pub fn candidates(
    brep: &BRep,
    bodies: &[BodyId],
    f: &ViewFrame,
    scale: f64,
    centre: P,
) -> Vec<Cand> {
    let local = |p: V3| -> P {
        let q = f.project(p).0;
        [q[0] * scale - centre[0], q[1] * scale - centre[1]]
    };
    let mut out = Vec::new();
    for (bi, b) in bodies.iter().enumerate() {
        let mut seen: HashSet<EdgeId> = HashSet::new();
        for face in brep.body_faces(*b) {
            let Some(fc) = brep.faces.get(face) else {
                continue;
            };
            for lp in &fc.loops {
                for co in brep.loop_coedges(*lp) {
                    let Some(c) = brep.coedges.get(co) else {
                        continue;
                    };
                    let e = c.edge;
                    if !seen.insert(e) {
                        continue;
                    }
                    let faces = brep.faces_around_edge(e);
                    if faces.len() < 2 || faces.iter().all(|x| *x == faces[0]) {
                        continue; // a seam, or a free edge
                    }
                    let Some(edge) = brep.edges.get(e) else {
                        continue;
                    };
                    let Some(curve) = edge.curve.and_then(|c| brep.curves.get(c)) else {
                        continue;
                    };
                    let (Some(sv), Some(ev)) =
                        (brep.vertices.get(edge.start), brep.vertices.get(edge.end))
                    else {
                        continue;
                    };
                    let (a, z) = (v3(&sv.point), v3(&ev.point));
                    let mut point = |kind: PointKind, p: V3| {
                        out.push(Cand {
                            body: bi,
                            edge: e,
                            kind: CandKind::Point(kind),
                            p2: local(p),
                            p3: p,
                            circle: None,
                            line: None,
                            line2: None,
                        });
                    };
                    match curve {
                        CurveGeom::Line(_) => {
                            point(PointKind::Start, a);
                            point(PointKind::End, z);
                            point(
                                PointKind::Mid,
                                [
                                    (a[0] + z[0]) / 2.0,
                                    (a[1] + z[1]) / 2.0,
                                    (a[2] + z[2]) / 2.0,
                                ],
                            );
                            out.push(Cand {
                                body: bi,
                                edge: e,
                                kind: CandKind::Line,
                                p2: local([
                                    (a[0] + z[0]) / 2.0,
                                    (a[1] + z[1]) / 2.0,
                                    (a[2] + z[2]) / 2.0,
                                ]),
                                p3: a,
                                circle: None,
                                line: Some((a, z)),
                                line2: Some((local(a), local(z))),
                            });
                        }
                        CurveGeom::Circle(c) => {
                            let centre3 = v3(&c.frame.origin);
                            let axis = [c.frame.z.x, c.frame.z.y, c.frame.z.z];
                            if edge.start != edge.end {
                                point(PointKind::Start, a);
                                point(PointKind::End, z);
                            }
                            point(PointKind::Centre, centre3);
                            if let Some(last) = out.last_mut() {
                                last.circle = Some((centre3, c.radius, axis));
                            }
                            out.push(Cand {
                                body: bi,
                                edge: e,
                                kind: CandKind::Circle,
                                p2: local(centre3),
                                p3: centre3,
                                circle: Some((centre3, c.radius, axis)),
                                line: None,
                                line2: None,
                            });
                        }
                        CurveGeom::Ellipse(el) => {
                            point(PointKind::Centre, v3(&el.frame.origin));
                        }
                        _ => {
                            point(PointKind::Start, a);
                            point(PointKind::End, z);
                        }
                    }
                }
            }
        }
    }
    out
}

fn dist_to_segment(p: P, a: P, b: P) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1e-24 {
        0.0
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0)
    };
    (a[0] + t * dx - p[0]).hypot(a[1] + t * dy - p[1])
}

/// Is `p` within `tol` mm of a drawn visible line? (So hidden features are not offered.)
pub fn near_visible(scene: &Scene2d, p: P, tol: f64) -> bool {
    scene
        .prims
        .iter()
        .filter(|x| x.layer == Layer::Visible)
        .any(|x| match &x.shape {
            Shape::Fill { .. } | Shape::Text { .. } => false,
            shape => {
                let pts = flatten(shape, 0.05);
                pts.windows(2)
                    .any(|w| dist_to_segment(p, w[0], w[1]) <= tol)
            }
        })
}

/// Distance from a point to a cand's feature in the view: to the point itself, to the circle's
/// circumference or to the segment.
pub fn distance(c: &Cand, p: P, scale: f64, f: &ViewFrame) -> f64 {
    match c.kind {
        CandKind::Point(_) => (c.p2[0] - p[0]).hypot(c.p2[1] - p[1]),
        CandKind::Circle => {
            let Some((_, r, axis)) = c.circle else {
                return f64::INFINITY;
            };
            // only a circle seen face-on can be picked
            if crate::projection::dot(axis, f.dir).abs() < 0.999 {
                return f64::INFINITY;
            }
            ((c.p2[0] - p[0]).hypot(c.p2[1] - p[1]) - r * scale).abs()
        }
        CandKind::Line => c
            .line2
            .map_or(f64::INFINITY, |(a, b)| dist_to_segment(p, a, b)),
    }
}

/// Is the feature of a candidate drawn visible in the view? A point must lie on a drawn line, a
/// straight edge is judged at its middle, and anything on a circle (the circle, its centre) at
/// four points around the circle.
pub fn cand_visible(
    c: &Cand,
    scene: &Scene2d,
    f: &ViewFrame,
    scale: f64,
    centre: P,
    tol: f64,
) -> bool {
    if let Some((ctr, r, axis)) = c.circle {
        // two directions across the circle, perpendicular to its axis
        let helper = if axis[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let u = crate::projection::cross(axis, helper);
        let ul = crate::projection::dot(u, u).sqrt().max(1e-12);
        let u = [u[0] / ul, u[1] / ul, u[2] / ul];
        let w = crate::projection::cross(axis, u);
        return [u, w, [-u[0], -u[1], -u[2]], [-w[0], -w[1], -w[2]]]
            .iter()
            .any(|d| {
                let p = [ctr[0] + d[0] * r, ctr[1] + d[1] * r, ctr[2] + d[2] * r];
                let q = f.project(p).0;
                near_visible(
                    scene,
                    [q[0] * scale - centre[0], q[1] * scale - centre[1]],
                    tol,
                )
            });
    }
    near_visible(scene, c.p2, tol)
}
