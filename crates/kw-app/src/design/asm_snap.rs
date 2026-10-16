//! Joint origins on geometry: what a click on a face or an edge of a component offers as the origin of a
//! joint (the joint snap points), in world coordinates.

use super::geom;
use crate::viewport::Pick;
use kw_brep::{BRep, EdgeId, FaceId};
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};

/// A point with a direction on a component: the origin of a joint (z along `z`).
#[derive(Debug, Clone, PartialEq)]
pub struct Origin {
    pub point: [f64; 3],
    pub z: [f64; 3],
    pub x: [f64; 3],
    pub what: &'static str,
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: [f64; 3]) -> [f64; 3] {
    let n = norm(a);
    if n < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        [a[0] / n, a[1] / n, a[2] / n]
    }
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Some x axis perpendicular to `z`.
pub fn perpendicular(z: [f64; 3]) -> [f64; 3] {
    let helper = if z[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let d = dot(helper, z);
    unit([
        helper[0] - z[0] * d,
        helper[1] - z[1] * d,
        helper[2] - z[2] * d,
    ])
}

fn origin(point: [f64; 3], z: [f64; 3], x: Option<[f64; 3]>, what: &'static str) -> Origin {
    let z = unit(z);
    let x = match x {
        Some(x) => {
            let d = dot(x, z);
            let p = [x[0] - z[0] * d, x[1] - z[1] * d, x[2] - z[2] * d];
            if norm(p) < 1e-9 {
                perpendicular(z)
            } else {
                unit(p)
            }
        }
        None => perpendicular(z),
    };
    Origin { point, z, x, what }
}

fn vertex_point(brep: &BRep, v: kw_brep::VertexId) -> Option<[f64; 3]> {
    let p = brep.vertices.get(v)?.point;
    Some([p.x, p.y, p.z])
}

/// Edges on the boundary of a face.
fn face_edges(brep: &BRep, f: FaceId) -> Vec<EdgeId> {
    let mut out = Vec::new();
    let Some(face) = brep.faces.get(f) else {
        return out;
    };
    for lp in &face.loops {
        for c in brep.loop_coedges(*lp) {
            if let Some(co) = brep.coedges.get(c) {
                if !out.contains(&co.edge) {
                    out.push(co.edge);
                }
            }
        }
    }
    out
}

/// What an edge offers: its midpoint (a line, with its direction as z), or its centre (a circle, with its
/// axis as z); its end points.
fn edge_origins(brep: &BRep, e: EdgeId, z_default: Option<[f64; 3]>) -> Vec<Origin> {
    let mut out = Vec::new();
    let Some(edge) = brep.edges.get(e) else {
        return out;
    };
    let Some(ci) = edge.curve else {
        return out;
    };
    let (t0, t1) = edge.param_range;
    match &brep.curves[ci] {
        CurveGeom::Line(l) => {
            let d = unit([l.dir.x, l.dir.y, l.dir.z]);
            let m = brep.curves[ci].point_at(0.5 * (t0 + t1));
            out.push(origin(
                [m.x, m.y, m.z],
                z_default.unwrap_or(d),
                None,
                "edge midpoint",
            ));
            for v in [edge.start, edge.end] {
                if let Some(p) = vertex_point(brep, v) {
                    out.push(origin(p, z_default.unwrap_or(d), None, "vertex"));
                }
            }
        }
        CurveGeom::Circle(c) => {
            let o = c.frame.origin;
            let z = [c.frame.z.x, c.frame.z.y, c.frame.z.z];
            let x = [c.frame.x.x, c.frame.x.y, c.frame.x.z];
            out.push(origin([o.x, o.y, o.z], z, Some(x), "circle centre"));
            for v in [edge.start, edge.end] {
                if let Some(p) = vertex_point(brep, v) {
                    if z_default.is_some() {
                        out.push(origin(p, z_default.unwrap_or(z), None, "vertex"));
                    }
                }
            }
        }
        other => {
            let tm = 0.5 * (t0 + t1);
            let m = other.point_at(tm);
            let h = 1e-4 * (t1 - t0).abs().max(1e-9);
            let (a, b) = (other.point_at(tm - h), other.point_at(tm + h));
            let d = unit([b.x - a.x, b.y - a.y, b.z - a.z]);
            out.push(origin(
                [m.x, m.y, m.z],
                z_default.unwrap_or(d),
                None,
                "edge midpoint",
            ));
        }
    }
    out
}

/// The snap points a pick offers, the most useful first (the centre of a face, the axis of a round face,
/// the centre of a circle, the middle of a line).
pub fn candidates(brep: &BRep, pick: Pick) -> Vec<Origin> {
    match pick {
        Pick::Edge(e) => edge_origins(brep, e, None),
        Pick::Face(f) => face_candidates(brep, f),
    }
}

fn bbox_centre(brep: &BRep, f: FaceId) -> Option<[f64; 3]> {
    let bb = brep.face_bbox(f)?;
    Some([
        0.5 * (bb.min.x + bb.max.x),
        0.5 * (bb.min.y + bb.max.y),
        0.5 * (bb.min.z + bb.max.z),
    ])
}

fn face_candidates(brep: &BRep, f: FaceId) -> Vec<Origin> {
    let mut out = Vec::new();
    let Some(face) = brep.faces.get(f) else {
        return out;
    };
    let centre = bbox_centre(brep, f);
    match &brep.surfaces[face.surface] {
        SurfaceGeom::Plane(_) => {
            if let (Some(fr), Some(c)) = (geom::face_frame(brep, f), centre) {
                // the middle of the face's box, moved onto its plane
                let d = dot(sub(c, fr.origin), fr.n);
                let p = [c[0] - d * fr.n[0], c[1] - d * fr.n[1], c[2] - d * fr.n[2]];
                out.push(origin(p, fr.n, Some(fr.x), "face centre"));
                for e in face_edges(brep, f) {
                    out.extend(edge_origins(brep, e, Some(fr.n)));
                }
            }
        }
        SurfaceGeom::Cylinder(cy) => {
            let o = [cy.frame.origin.x, cy.frame.origin.y, cy.frame.origin.z];
            let z = unit([cy.frame.z.x, cy.frame.z.y, cy.frame.z.z]);
            let x = [cy.frame.x.x, cy.frame.x.y, cy.frame.x.z];
            if let Some(c) = centre {
                // the axis at the middle of the face
                let t = dot(sub(c, o), z);
                let p = [o[0] + t * z[0], o[1] + t * z[1], o[2] + t * z[2]];
                out.push(origin(p, z, Some(x), "cylinder axis"));
            }
            for e in face_edges(brep, f) {
                for mut cand in edge_origins(brep, e, None) {
                    if cand.what == "circle centre" {
                        cand.z = z;
                        out.push(cand);
                    }
                }
            }
        }
        SurfaceGeom::Sphere(s) => {
            let o = s.frame.origin;
            out.push(origin(
                [o.x, o.y, o.z],
                [s.frame.z.x, s.frame.z.y, s.frame.z.z],
                Some([s.frame.x.x, s.frame.x.y, s.frame.x.z]),
                "sphere centre",
            ));
        }
        SurfaceGeom::Cone(c) => {
            let o = [c.frame.origin.x, c.frame.origin.y, c.frame.origin.z];
            let z = unit([c.frame.z.x, c.frame.z.y, c.frame.z.z]);
            if let Some(m) = centre {
                let t = dot(sub(m, o), z);
                out.push(origin(
                    [o[0] + t * z[0], o[1] + t * z[1], o[2] + t * z[2]],
                    z,
                    None,
                    "cone axis",
                ));
            }
        }
        _ => {
            if let Some(c) = centre {
                out.push(origin(c, [0.0, 0.0, 1.0], None, "face centre"));
            }
        }
    }
    out
}

/// The candidate nearest to `target` (a world point under the pointer) within `reach`; otherwise the first.
pub fn choose(cands: &[Origin], target: Option<[f64; 3]>, reach: f64) -> Option<Origin> {
    let first = cands.first()?;
    let Some(t) = target else {
        return Some(first.clone());
    };
    let best = cands
        .iter()
        .min_by(|a, b| norm(sub(a.point, t)).total_cmp(&norm(sub(b.point, t))))?;
    Some(if norm(sub(best.point, t)) <= reach {
        best.clone()
    } else {
        first.clone()
    })
}

/// The cross product helper kept public for callers building frames.
pub fn y_axis(o: &Origin) -> [f64; 3] {
    cross(o.z, o.x)
}
