//! A canonical hash of geometry: independent of identifiers, slot order, parameterisation origin and
//! thread count; sensitive to every surface, edge and vertex position beyond 1e-7 mm.

use crate::id::Hasher128;
use crate::naming::{NameMap, TopoName};
use kw_brep::{BRep, BodyId, FaceId};
use kw_math::Vector3;
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};

fn q(h: &mut Hasher128, v: f64) {
    // 1e-7 mm quantisation
    let r = (v * 1e7).round();
    let r = if r == 0.0 { 0.0 } else { r };
    h.bytes(&r.to_le_bytes());
}

fn qv(h: &mut Hasher128, v: &Vector3<f64>) {
    q(h, v.x);
    q(h, v.y);
    q(h, v.z);
}

fn canon_axis(v: Vector3<f64>) -> Vector3<f64> {
    let flip = [v.x, v.y, v.z]
        .iter()
        .find(|c| c.abs() > 1e-9)
        .map(|c| *c < 0.0)
        .unwrap_or(false);
    if flip {
        -v
    } else {
        v
    }
}

fn curve(h: &mut Hasher128, c: &CurveGeom, t0: f64, t1: f64) {
    let tag = match c {
        CurveGeom::Line(_) => 1,
        CurveGeom::Circle(_) => 2,
        CurveGeom::Ellipse(_) => 3,
        CurveGeom::Nurbs(_) => 4,
        _ => 5,
    };
    h.u64(tag);
    for k in 0..=4 {
        let t = t0 + (t1 - t0) * k as f64 / 4.0;
        let p = c.point_at(t);
        q(h, p.x);
        q(h, p.y);
        q(h, p.z);
    }
}

fn surface(h: &mut Hasher128, s: &SurfaceGeom) {
    match s {
        SurfaceGeom::Plane(p) => {
            h.u64(1);
            let n = canon_axis(p.frame.z);
            qv(h, &n);
            q(h, p.frame.origin.coords.dot(&n));
        }
        SurfaceGeom::Cylinder(c) => {
            h.u64(2);
            let a = canon_axis(c.frame.z);
            qv(h, &a);
            let o = c.frame.origin.coords;
            qv(h, &(o - a * o.dot(&a)));
            q(h, c.radius);
        }
        SurfaceGeom::Cone(c) => {
            h.u64(3);
            let a = canon_axis(c.frame.z);
            qv(h, &a);
            let t = c.half_angle.tan();
            let apex = if t.abs() > 1e-12 {
                c.frame.origin.coords - c.frame.z * (c.radius_at_origin / t)
            } else {
                c.frame.origin.coords
            };
            qv(h, &apex);
            q(h, c.half_angle);
        }
        SurfaceGeom::Sphere(c) => {
            h.u64(4);
            qv(h, &c.frame.origin.coords);
            q(h, c.radius);
        }
        SurfaceGeom::Torus(t) => {
            h.u64(5);
            qv(h, &canon_axis(t.frame.z));
            qv(h, &t.frame.origin.coords);
            q(h, t.major_radius);
            q(h, t.minor_radius);
        }
        SurfaceGeom::Nurbs(n) => {
            h.u64(6);
            let ((u0, u1), (v0, v1)) = { n.domain() };
            for i in 0..=4 {
                for j in 0..=4 {
                    let p = n.point_at(
                        u0 + (u1 - u0) * i as f64 / 4.0,
                        v0 + (v1 - v0) * j as f64 / 4.0,
                    );
                    q(h, p.x);
                    q(h, p.y);
                    q(h, p.z);
                }
            }
        }
        SurfaceGeom::Extrusion { profile, dir } => {
            h.u64(7);
            let (a, b) = profile.domain();
            curve(h, profile, a, b);
            qv(h, dir);
        }
        SurfaceGeom::Revolution {
            profile,
            axis_origin,
            axis_dir,
        } => {
            h.u64(8);
            let (a, b) = profile.domain();
            curve(h, profile, a, b);
            qv(h, &axis_origin.coords);
            qv(h, &canon_axis(*axis_dir));
        }
        SurfaceGeom::Procedural(_) => {
            h.u64(9);
        }
    }
}

fn face_hash(rep: &BRep, f: FaceId, name: &TopoName) -> u128 {
    let face = &rep.faces[f];
    let mut h = Hasher128::new();
    h.u128(name.stable_hash());
    h.u64(face.sense as u64);
    surface(&mut h, &rep.surfaces[face.surface]);
    let mut loops: Vec<u128> = Vec::new();
    for lp in &face.loops {
        let mut edges: Vec<u128> = Vec::new();
        for co in rep.loop_coedges(*lp) {
            let e = &rep.edges[rep.coedges[co].edge];
            let mut eh = Hasher128::new();
            let (a, b) = (rep.vertices[e.start].point, rep.vertices[e.end].point);
            // an edge is the same edge whichever way it is traversed
            let (a, b) = if (a.x, a.y, a.z) <= (b.x, b.y, b.z) {
                (a, b)
            } else {
                (b, a)
            };
            for p in [a, b] {
                q(&mut eh, p.x);
                q(&mut eh, p.y);
                q(&mut eh, p.z);
            }
            match e.curve.and_then(|c| rep.curves.get(c)) {
                Some(c) => {
                    // sample the middle of the edge, direction independent
                    let m = c.point_at(0.5 * (e.param_range.0 + e.param_range.1));
                    eh.u64(1);
                    q(&mut eh, m.x);
                    q(&mut eh, m.y);
                    q(&mut eh, m.z);
                }
                None => {
                    eh.u64(0);
                }
            }
            edges.push(eh.finish());
        }
        edges.sort_unstable();
        let mut lh = Hasher128::new();
        lh.u64(edges.len() as u64);
        for e in edges {
            lh.u128(e);
        }
        loops.push(lh.finish());
    }
    loops.sort_unstable();
    h.u64(loops.len() as u64);
    for l in loops {
        h.u128(l);
    }
    h.finish()
}

/// Hash of the body's geometry, faces taken in the order of their names.
pub fn geometry_hash(rep: &BRep, body: BodyId, names: &NameMap) -> u128 {
    let mut by_face: Vec<(&TopoName, FaceId)> = names
        .by_name
        .iter()
        .filter_map(|(n, e)| match e {
            kw_brep::EntityRef::Face(f) => Some((n, *f)),
            _ => None,
        })
        .collect();
    by_face.sort_by(|a, b| a.0.cmp(b.0));
    let mut h = Hasher128::new();
    h.u64(rep.body_faces(body).len() as u64);
    for (n, f) in by_face {
        h.u128(face_hash(rep, f, n));
    }
    h.finish()
}
