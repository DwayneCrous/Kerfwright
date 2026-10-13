//! Vertex blends: the rolling-ball corner patch where three filleted edges of three planar faces meet.
//!
//! For constant radius `r` the ball touches all three faces at once at the vertex, its centre `c` is at
//! distance `r` from the three planes, and the corner of the blend is the part of the sphere
//! `(c, r)` inside the *normal cone* of the corner: the cone at `c` spanned by the face normals. Its
//! three bounding planes `P_ij` contain `c` and are perpendicular to the edges `E_ij`, so they are exactly
//! the planes in which the three fillet cylinders end; cylinders and sphere meet tangentially along the
//! great-circle arcs `T_i T_j`.
//!
//! The material that the three edge tools (which run through the vertex and meet in a mitre) leave
//! behind inside that cone is `(wedge ∩ cone) ∖ ball`. This module builds that region as a tool: the
//! hexahedron bounded by the three faces (pushed outwards by a margin so no tool face is coincident with
//! a body face) and the three `P_ij`, minus the ball.

use crate::blend::{err, BlendError, BlendErrorKind, Section};
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{make_box, make_sphere, BRep, BodyId, EdgeId, FaceId, VertexId};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::{CurveGeom, SurfaceGeom};
use nalgebra::Matrix3;

/// Geometry of one eligible corner.
pub struct Corner {
    pub vertex: VertexId,
    pub edges: [EdgeId; 3],
    /// Convex corner (material removed) or concave (material added).
    pub convex: bool,
    pub centre: Point3<f64>,
    pub normals: [Vector3<f64>; 3],
    pub dirs: [Vector3<f64>; 3],
}

/// Is `vertex` a corner of three planar faces whose three (straight) edges are all in `blended`, all
/// of one convexity? `Ok(None)` when it is not (the edges then simply meet in a mitre).
pub fn analyse_corner(
    rep: &BRep,
    body: BodyId,
    vertex: VertexId,
    blended: &[EdgeId],
    sections: &dyn Fn(EdgeId) -> Option<Section>,
    radius: f64,
) -> Result<Option<Corner>, BlendError> {
    let mut inc: Vec<EdgeId> = Vec::new();
    for (eid, e) in rep.edges.iter() {
        if (e.start == vertex || e.end == vertex) && e.start != e.end {
            inc.push(eid);
        }
    }
    if inc.len() != 3 || !inc.iter().all(|e| blended.contains(e)) {
        return Ok(None);
    }
    let body_faces = rep.body_faces(body);
    let mut faces: Vec<FaceId> = Vec::new();
    for &e in &inc {
        let ring = rep.faces_around_edge(e);
        if ring.len() != 2 || ring[0] == ring[1] {
            return Ok(None);
        }
        for f in ring {
            if !body_faces.contains(&f) {
                return Ok(None);
            }
            if !faces.contains(&f) {
                faces.push(f);
            }
        }
    }
    if faces.len() != 3 {
        return Ok(None);
    }
    if !faces
        .iter()
        .all(|f| matches!(rep.surfaces[rep.faces[*f].surface], SurfaceGeom::Plane(_)))
    {
        return Ok(None);
    }
    if !inc.iter().all(|e| {
        rep.edges[*e]
            .curve
            .and_then(|c| rep.curves.get(c))
            .is_some_and(|c| matches!(c, CurveGeom::Line(_)))
    }) {
        return Ok(None);
    }
    let secs: Vec<Section> = inc
        .iter()
        .map(|e| sections(*e))
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| err(inc[0], BlendErrorKind::BadEdge))?;
    let convex = secs[0].convex;
    if secs.iter().any(|s| s.convex != convex) {
        return Ok(None);
    }
    // outward normals of the planes
    let vp = rep.vertices[vertex].point;
    let normals: Vec<Vector3<f64>> = faces
        .iter()
        .map(|f| crate::blend::face_normal(rep, *f, &vp))
        .collect();
    let n = Matrix3::from_rows(&[
        normals[0].transpose(),
        normals[1].transpose(),
        normals[2].transpose(),
    ]);
    let det = n.determinant();
    if det.abs() < 1e-3 {
        // (nearly) coplanar or degenerate corner
        return Ok(None);
    }
    let inv = n
        .try_inverse()
        .ok_or_else(|| err(inc[0], BlendErrorKind::Size("degenerate corner")))?;
    // (c - v) . n_i = -s r  with s = +1 for a convex corner
    let s = if convex { 1.0 } else { -1.0 };
    let off = inv * Vector3::new(-s * radius, -s * radius, -s * radius);
    let centre = vp + off;
    // direction of each edge pointing away from the vertex
    let mut dirs = [Vector3::zeros(); 3];
    for (k, &e) in inc.iter().enumerate() {
        let ed = &rep.edges[e];
        let other = if ed.start == vertex { ed.end } else { ed.start };
        let d = rep.vertices[other].point - vp;
        let len = d.norm();
        dirs[k] = d / len;
        // the fillet ends at the plane through the centre perpendicular to the edge
        let t = (centre - vp).dot(&dirs[k]);
        if t <= 1e-9 * (1.0 + radius) {
            return Err(err(
                e,
                BlendErrorKind::Unsupported(
                    "corner geometry: blend would run backwards along an edge",
                ),
            ));
        }
        if t >= len * (1.0 - 1e-6) {
            return Err(err(
                e,
                BlendErrorKind::Size("radius too large for the edge at the corner"),
            ));
        }
    }
    let mut es = [inc[0]; 3];
    let mut ns = [Vector3::zeros(); 3];
    es.copy_from_slice(&inc[..3]);
    ns.copy_from_slice(&normals[..3]);
    Ok(Some(Corner {
        vertex,
        edges: es,
        convex,
        centre,
        normals: ns,
        dirs,
    }))
}

/// Half space `{x : (x - p)·ν ≤ 0}` as a (huge) box.
fn half_space(rep: &mut BRep, p: &Point3<f64>, nu: &Vector3<f64>, big: f64) -> BodyId {
    let helper = if nu.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let hx = helper.cross(nu).normalize();
    let hy = nu.cross(&hx);
    let frame = Frame3 {
        origin: *p - *nu * (big / 2.0),
        x: hx,
        y: hy,
        z: *nu,
    };
    make_box(rep, &frame, big, big, big).body
}

/// The vertex tool and the operation that applies it.
pub fn build_corner_tool(
    rep: &mut BRep,
    corner: &Corner,
    radius: f64,
) -> Result<(BodyId, Op), BlendError> {
    let e0 = corner.edges[0];
    let vp = rep.vertices[corner.vertex].point;
    let m = 0.3 * radius;
    let big = 40.0 * ((corner.centre - vp).norm() + radius + m);
    let mut planes: Vec<(Point3<f64>, Vector3<f64>)> = Vec::new();
    for k in 0..3 {
        let n = corner.normals[k];
        if corner.convex {
            // tool reaches beyond the body's face
            planes.push((vp + n * m, n));
        } else {
            // tool reaches into the material
            planes.push((vp - n * m, -n));
        }
    }
    for k in 0..3 {
        // plane through the centre, perpendicular to the edge; keep the side that contains the vertex
        let d = corner.dirs[k];
        let side = (vp - corner.centre).dot(&d);
        let nu = if side > 0.0 { -d } else { d };
        planes.push((corner.centre, nu));
    }
    let mut q: Option<BodyId> = None;
    for (p, nu) in &planes {
        let hs = half_space(rep, p, nu, big);
        q = Some(match q {
            None => hs,
            Some(prev) => {
                boolean(rep, prev, hs, Op::Intersect, &BoolOptions::default())
                    .map_err(|e| err(e0, BlendErrorKind::Boolean(e)))?
                    .body
            }
        });
    }
    let q = q.expect("six planes");
    // the ball: poles ⟂ and seam pointing away from the vertex, so the patch (towards the vertex)
    // keeps clear of the poles and the seam
    let to_v = (vp - corner.centre).normalize();
    let helper = if to_v.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let e1 = helper.cross(&to_v).normalize();
    let e2 = to_v.cross(&e1);
    // the great circles that bound the patch are the traces of the planes P_ij; the poles must stay clear
    // of all of them (a bounding circle through a pole is a degenerate meridian)
    let traces: Vec<Vector3<f64>> = corner
        .dirs
        .iter()
        .map(|d| to_v.cross(d))
        .filter(|t| t.norm() > 1e-9)
        .map(|t| t.normalize())
        .collect();
    // choose the pole direction (an angle in the plane perpendicular to `to_v`) farthest from all traces
    let mut best = (f64::NEG_INFINITY, 0.0f64);
    for k in 0..720 {
        let th = std::f64::consts::PI * k as f64 / 720.0;
        let z = e1 * th.cos() + e2 * th.sin();
        // angle between z and the nearest trace line
        let ang = traces
            .iter()
            .map(|t| t.dot(&z).abs().clamp(0.0, 1.0).acos())
            .fold(f64::INFINITY, f64::min);
        if ang > best.0 {
            best = (ang, th);
        }
    }
    let zax = e1 * best.1.cos() + e2 * best.1.sin();
    let xax = -to_v;
    let yax = zax.cross(&xax);
    let frame = Frame3 {
        origin: corner.centre,
        x: xax,
        y: yax,
        z: zax,
    };
    let ball = make_sphere(rep, &frame, radius).body;
    let tool = boolean(rep, q, ball, Op::Subtract, &BoolOptions::default())
        .map_err(|e| err(e0, BlendErrorKind::Boolean(e)))?
        .body;
    Ok((
        tool,
        if corner.convex {
            Op::Subtract
        } else {
            Op::Union
        },
    ))
}
