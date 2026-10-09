//! Face offset, draft and shell, built on [`rebuild`](crate::rebuild::rebuild).

use crate::extrude::{extrude, ExtrudeOptions};
use crate::profile::{ModelError, Profile, ProfileEdge, ProfileLoop};
use crate::rebuild::rebuild;
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{BRep, BodyId, BuildResult, EntityRef, FaceId, Origin, Sense, ShapeHistory};
use kw_math::{Frame3, Point3, Unit, Vector3};
use kw_nurbs::fit::offset_surface;
use kw_nurbs::geom::{Cone, Plane};
use kw_nurbs::proj::surface_param_of;
use kw_nurbs::{Curve3, Surface, SurfaceGeom};
use nalgebra::Rotation3;
use std::collections::HashMap;

fn outward_offset(rep: &BRep, f: FaceId, d: f64) -> Option<SurfaceGeom> {
    let face = &rep.faces[f];
    let s = &rep.surfaces[face.surface];
    // `offset_surface` moves along the *surface* normal; the outward normal is flipped for reversed faces
    let signed = if face.sense == Sense::Forward { d } else { -d };
    offset_surface(s, signed)
}

/// Move every face of `body` outward by `d` (negative = inward). Edges and vertices are recomputed
/// as intersections of the offset faces, so convex edges stay sharp (a mitred offset). Fails with a
/// description if a face collapses or surfaces cannot be offset in closed form.
pub fn offset_body(rep: &mut BRep, body: BodyId, d: f64) -> Result<BuildResult, ModelError> {
    if d == 0.0 {
        return Err(ModelError::BadParameter("zero offset"));
    }
    let mut map = HashMap::new();
    for f in rep.body_faces(body) {
        let s = outward_offset(rep, f, d).ok_or_else(|| ModelError::Construction(format!("face {f:?}: surface cannot be offset in closed form (or the offset radius is not positive)")))?;
        map.insert(f, s);
    }
    Ok(rebuild(rep, body, &map)?.0)
}

/// Tilt planar and cylindrical faces by `angle` about their intersection with the neutral plane
/// `(origin, normal)`. `pull` is the mould-opening direction; positive angles make the body narrower
/// along `pull` (material removed on the far side of the neutral plane).
pub fn draft_faces(
    rep: &mut BRep,
    body: BodyId,
    faces: &[FaceId],
    neutral_origin: Point3<f64>,
    pull: Vector3<f64>,
    angle: f64,
) -> Result<BuildResult, ModelError> {
    let pull = pull.normalize();
    let mut map = HashMap::new();
    for &f in faces {
        let face = &rep.faces[f];
        let s = rep.surfaces[face.surface].clone();
        let out_sign = if face.sense == Sense::Forward {
            1.0
        } else {
            -1.0
        };
        match s {
            SurfaceGeom::Plane(p) => {
                let n = p.frame.z * out_sign; // outward normal
                if n.dot(&pull).abs() > 1.0 - 1e-9 {
                    return Err(ModelError::Construction(format!(
                        "face {f:?} is perpendicular to the pull direction"
                    )));
                }
                // hinge: intersection of the face plane with the neutral plane
                let hinge = n.cross(&pull); // rotation axis taking n towards pull
                if hinge.norm() < 1e-12 {
                    return Err(ModelError::Construction(format!(
                        "face {f:?}: degenerate hinge"
                    )));
                }
                let h = hinge.normalize();
                // point on the hinge line: in the face plane, at neutral height
                let dist_n = (neutral_origin - p.frame.origin).dot(&pull);
                let along = pull.cross(&h); // direction in the face plane towards increasing pull
                let _ = along;
                // find p0 on the face plane with (p0 - neutral_origin)·pull = 0
                let in_plane_up = pull - n * pull.dot(&n); // projection of pull onto the face plane
                let t = dist_n / in_plane_up.dot(&pull);
                let p0 = p.frame.origin + in_plane_up * t;
                // tilt the outward normal towards `pull` by `angle` (positive angle: the body narrows along `pull`)
                let rot = Rotation3::from_axis_angle(&Unit::new_normalize(h), angle);
                let n2 = rot * n;
                let frame = Frame3::from_origin_normal(p0, n2 * out_sign);
                map.insert(f, SurfaceGeom::Plane(Plane { frame }));
            }
            SurfaceGeom::Cylinder(c) => {
                if c.frame.z.cross(&pull).norm() > 1e-9 {
                    return Err(ModelError::Construction(format!(
                        "face {f:?}: cylinder axis is not parallel to the pull direction"
                    )));
                }
                // radius at the neutral plane
                let h = (neutral_origin - c.frame.origin).dot(&c.frame.z);
                let axis_up = if c.frame.z.dot(&pull) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
                // a boss (outward normal radial): positive draft narrows along `pull`
                let half = (-angle * axis_up) * out_sign;
                let mut fr = c.frame;
                fr.origin += c.frame.z * h;
                map.insert(
                    f,
                    SurfaceGeom::Cone(Cone {
                        frame: fr,
                        radius_at_origin: c.radius,
                        half_angle: half,
                    }),
                );
            }
            _ => {
                return Err(ModelError::Construction(format!(
                    "face {f:?}: only planar and cylindrical faces can be drafted"
                )))
            }
        }
    }
    Ok(rebuild(rep, body, &map)?.0)
}

fn face_profile(rep: &BRep, f: FaceId) -> Result<Profile, ModelError> {
    let face = &rep.faces[f];
    let SurfaceGeom::Plane(pl) = &rep.surfaces[face.surface] else {
        return Err(ModelError::Construction(format!(
            "open face {f:?} must be planar"
        )));
    };
    let n = if face.sense == Sense::Forward {
        pl.frame.z
    } else {
        -pl.frame.z
    };
    let plane = Frame3 {
        origin: pl.frame.origin,
        x: pl.frame.x,
        y: n.cross(&pl.frame.x),
        z: n,
    };
    let mut loops: Vec<ProfileLoop> = Vec::new();
    for lp in &face.loops {
        let mut edges = Vec::new();
        for co in rep.loop_coedges(*lp) {
            let c = &rep.coedges[co];
            let e = &rep.edges[c.edge];
            let curve = rep
                .curves
                .get(e.curve.ok_or_else(|| {
                    ModelError::Construction("open face has a degenerate edge".into())
                })?)
                .unwrap()
                .clone();
            let mut pe = ProfileEdge::new(
                curve,
                (
                    e.param_range.0.min(e.param_range.1),
                    e.param_range.0.max(e.param_range.1),
                ),
            );
            pe.reversed = c.sense == Sense::Reversed;
            edges.push(pe);
        }
        loops.push(ProfileLoop::new(edges));
    }
    let mut it = loops.into_iter();
    let outer = it
        .next()
        .ok_or_else(|| ModelError::Construction("face without loops".into()))?;
    let mut p = Profile::new(plane, outer);
    p.holes = it.collect();
    Ok(p)
}

/// Hollow out `body` leaving walls of `thickness`; `open` faces are removed (planar faces only).
/// The inner surface is the inward mitred offset of the body (see [`offset_body`]).
pub fn shell(
    rep: &mut BRep,
    body: BodyId,
    thickness: f64,
    open: &[FaceId],
) -> Result<BuildResult, ModelError> {
    if thickness.is_nan() || thickness <= 0.0 {
        return Err(ModelError::BadParameter("shell thickness must be positive"));
    }
    // inner solid + map from old faces to inner faces
    let mut map = HashMap::new();
    for f in rep.body_faces(body) {
        let s = outward_offset(rep, f, -thickness).ok_or_else(|| {
            ModelError::Construction(format!(
                "face {f:?}: surface cannot be offset inward by {thickness}"
            ))
        })?;
        map.insert(f, s);
    }
    let (inner_res, rb) = rebuild(rep, body, &map)?;
    let mut cutter = inner_res.body;
    // cutter faces → faces of the original body (through the inner offset) or "opening" prism faces
    let mut cutter_hist = inner_res.history.clone();
    for &f in open {
        let nf = *rb.face_map.get(&f).ok_or_else(|| {
            ModelError::Construction(format!("open face {f:?} is not a face of the body"))
        })?;
        let profile = face_profile(rep, nf)?;
        let n = profile.plane.z;
        let prism = extrude(
            rep,
            &profile,
            n * (3.0 * thickness),
            &ExtrudeOptions::default(),
        )?;
        let prism_faces: std::collections::HashSet<FaceId> =
            rep.body_faces(prism.body).into_iter().collect();
        let u = boolean(rep, cutter, prism.body, Op::Union, &BoolOptions::default())
            .map_err(|e| ModelError::Construction(format!("opening face {f:?}: {e}")))?;
        // anything that comes from the prism is part of the opening, not of the original body
        let mut uh = ShapeHistory {
            deleted_inputs: u.history.deleted_inputs.clone(),
            ..Default::default()
        };
        for (k, o) in u.history.origins {
            let from_prism = matches!(&o, Origin::Preserved(EntityRef::Face(x)) | Origin::Modified(EntityRef::Face(x)) | Origin::SplitFrom { parent: EntityRef::Face(x), .. } if prism_faces.contains(x));
            uh.record(
                k,
                if from_prism {
                    Origin::New {
                        role: "shell-opening".into(),
                    }
                } else {
                    o
                },
            );
        }
        cutter_hist = ShapeHistory::compose(&cutter_hist, &uh);
        cutter = u.body;
    }
    let out = boolean(rep, body, cutter, Op::Subtract, &BoolOptions::default())
        .map_err(|e| ModelError::Construction(format!("shell: {e}")))?;
    Ok(BuildResult {
        body: out.body,
        history: ShapeHistory::compose(&cutter_hist, &out.history),
    })
}

#[allow(dead_code)]
fn _unused(rep: &BRep, f: FaceId) -> f64 {
    let s = &rep.surfaces[rep.faces[f].surface];
    let r = surface_param_of(s, &Point3::origin(), None);
    s.point_at(r.u, r.v).x
        + s.domain().0 .0
        + 0.0
            * Curve3::derivative(
                &kw_nurbs::CurveGeom::Line(kw_nurbs::Line {
                    origin: Point3::origin(),
                    dir: Vector3::x(),
                }),
                0.0,
            )
            .x
}
