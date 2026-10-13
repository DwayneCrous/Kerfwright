//! Extrusion of a planar profile along a vector.

use crate::profile::{ModelError, Profile, ProfileLoop};
use kw_brep::{
    add_face_auto, BRep, BodyKind, BuildResult, EdgeId, EntityRef, LoopKind, LoopSpec, Origin,
    Sense, ShapeHistory, VertexId,
};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::fit::transform_curve;
use kw_nurbs::geom::{Cylinder, Line, Plane};
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};

#[derive(Debug, Clone, Default)]
pub struct ExtrudeOptions {
    /// Extrude `±d/2` around the profile plane instead of `0..d`.
    pub symmetric: bool,
}

struct LoopData {
    /// Bottom / top vertices at each junction.
    vb: Vec<VertexId>,
    vt: Vec<VertexId>,
    eb: Vec<EdgeId>,
    et: Vec<EdgeId>,
    vert: Vec<EdgeId>,
    ccw_about_d: bool,
}

/// Extrude `profile` by the vector `d`.
pub fn extrude(
    rep: &mut BRep,
    profile: &Profile,
    d: Vector3<f64>,
    opts: &ExtrudeOptions,
) -> Result<BuildResult, ModelError> {
    let dn = d.norm();
    if dn < 1e-12 {
        return Err(ModelError::BadParameter("zero extrusion vector"));
    }
    let dh = d / dn;
    let np = profile.plane.z;
    if np.dot(&dh).abs() < 1e-9 {
        return Err(ModelError::BadParameter(
            "extrusion direction lies in the profile plane",
        ));
    }
    let scale = 1.0 + profile.plane.origin.coords.amax() + dn;
    let tol = 1e-9 * scale;
    // loops with their orientation about the extrusion direction
    let mut loops: Vec<(&ProfileLoop, LoopKind)> = vec![(&profile.outer, LoopKind::Outer)];
    loops.extend(profile.holes.iter().map(|h| (h, LoopKind::Inner)));
    let mut ccw: Vec<bool> = Vec::new();
    for (l, _) in &loops {
        let a = l.check(&np, tol)?;
        if a.abs() < 1e-14 {
            return Err(ModelError::BadProfile("loop has zero area"));
        }
        ccw.push((a > 0.0) == (np.dot(&dh) > 0.0));
    }
    if ccw[0] != ccw.iter().skip(1).all(|c| !*c)
        && loops.len() > 1
        && !ccw.iter().skip(1).all(|c| *c != ccw[0])
    {
        return Err(ModelError::BadProfile(
            "holes must wind opposite to the outer loop",
        ));
    }

    let base = if opts.symmetric {
        -d * 0.5
    } else {
        Vector3::zeros()
    };
    let mut history = ShapeHistory::default();
    let body = rep.add_body(BodyKind::Solid);
    let lump = rep.add_lump(body);
    let shell = rep.add_shell(lump);

    // ---- vertices and edges
    let mut data: Vec<LoopData> = Vec::new();
    for (li, (l, _)) in loops.iter().enumerate() {
        let n = l.edges.len();
        let junction: Vec<Point3<f64>> = l.edges.iter().map(|e| e.start() + base).collect();
        let vb: Vec<VertexId> = junction.iter().map(|p| rep.add_vertex(*p)).collect();
        let vt: Vec<VertexId> = junction.iter().map(|p| rep.add_vertex(*p + d)).collect();
        let mut eb = Vec::new();
        let mut et = Vec::new();
        let mut vert = Vec::new();
        for i in 0..n {
            let e = &l.edges[i];
            let (s, t) = if e.reversed {
                (vb[(i + 1) % n], vb[i])
            } else {
                (vb[i], vb[(i + 1) % n])
            };
            let (st, tt) = if e.reversed {
                (vt[(i + 1) % n], vt[i])
            } else {
                (vt[i], vt[(i + 1) % n])
            };
            let cb = transform_curve(&e.curve, &kw_math::Transform3::translation(base));
            let ct = transform_curve(&e.curve, &kw_math::Transform3::translation(base + d));
            let cbi = rep.add_curve(cb);
            let cti = rep.add_curve(ct);
            eb.push(rep.add_edge(s, t, Some(cbi), e.range));
            et.push(rep.add_edge(st, tt, Some(cti), e.range));
            let vc = rep.add_curve(CurveGeom::Line(Line {
                origin: junction[i],
                dir: d,
            }));
            vert.push(rep.add_edge(vb[i], vt[i], Some(vc), (0.0, 1.0)));
        }
        let _ = li;
        data.push(LoopData {
            vb,
            vt,
            eb,
            et,
            vert,
            ccw_about_d: ccw[li],
        });
    }

    // ---- caps
    let bottom_frame = Frame3 {
        origin: profile.plane.origin + base,
        ..profile.plane
    };
    let top_frame = Frame3 {
        origin: profile.plane.origin + base + d,
        ..profile.plane
    };
    let out_bottom = if np.dot(&dh) > 0.0 { -np } else { np };
    for (cap, frame, outward, role) in [
        (0, bottom_frame, out_bottom, "start-cap"),
        (1, top_frame, -out_bottom, "end-cap"),
    ] {
        let surf = rep.add_surface(SurfaceGeom::Plane(Plane { frame }));
        let specs: Vec<LoopSpec> = loops
            .iter()
            .enumerate()
            .map(|(li, (l, kind))| LoopSpec {
                kind: *kind,
                chain: (0..l.edges.len())
                    .map(|i| {
                        let e = if cap == 0 {
                            data[li].eb[i]
                        } else {
                            data[li].et[i]
                        };
                        (
                            e,
                            if l.edges[i].reversed {
                                Sense::Reversed
                            } else {
                                Sense::Forward
                            },
                        )
                    })
                    .collect(),
            })
            .collect();
        let f = add_face_auto(rep, shell, surf, &move |_| outward, &specs)
            .map_err(ModelError::Construction)?;
        history.record(EntityRef::Face(f), Origin::New { role: role.into() });
    }

    // ---- side faces
    for (li, (l, kind)) in loops.iter().enumerate() {
        let n = l.edges.len();
        for i in 0..n {
            let e = &l.edges[i];
            let sign = if (data[li].ccw_about_d) == (*kind == LoopKind::Outer) {
                1.0
            } else {
                -1.0
            };
            let mid = 0.5 * (e.range.0 + e.range.1);
            let tangent = {
                let t = e.curve.derivative(mid);
                if e.reversed {
                    -t
                } else {
                    t
                }
            };
            let outward_dir = tangent.cross(&dh).normalize() * sign;
            let surface = side_surface(&e.curve, e.range, base, d, dh)?;
            let surf = rep.add_surface(surface);
            let sense_i = if e.reversed {
                Sense::Reversed
            } else {
                Sense::Forward
            };
            let chain = vec![
                (data[li].eb[i], sense_i),
                (data[li].vert[(i + 1) % n], Sense::Forward),
                (data[li].et[i], sense_i.flip()),
                (data[li].vert[i], Sense::Reversed),
            ];
            let f = add_face_auto(
                rep,
                shell,
                surf,
                &move |_| outward_dir,
                &[LoopSpec {
                    kind: LoopKind::Outer,
                    chain,
                }],
            )
            .map_err(ModelError::Construction)?;
            history.record(
                EntityRef::Face(f),
                Origin::New {
                    role: format!("side-{li}-{i}").into(),
                },
            );
        }
        let _ = (&data[li].vb, &data[li].vt);
    }
    Ok(BuildResult { body, history })
}

fn side_surface(
    curve: &CurveGeom,
    range: (f64, f64),
    base: Vector3<f64>,
    d: Vector3<f64>,
    dh: Vector3<f64>,
) -> Result<SurfaceGeom, ModelError> {
    match curve {
        CurveGeom::Line(l) => {
            let x = l.dir.normalize();
            let y = d - x * d.dot(&x);
            if y.norm() < 1e-9 * d.norm() {
                return Err(ModelError::BadProfile(
                    "profile edge parallel to the extrusion direction",
                ));
            }
            let y = y.normalize();
            Ok(SurfaceGeom::Plane(Plane {
                frame: Frame3 {
                    origin: l.origin + base,
                    x,
                    y,
                    z: x.cross(&y),
                },
            }))
        }
        CurveGeom::Circle(c) if c.frame.z.cross(&dh).norm() < 1e-9 => {
            let f = if c.frame.z.dot(&dh) > 0.0 {
                c.frame
            } else {
                Frame3 {
                    origin: c.frame.origin,
                    x: c.frame.x,
                    y: -c.frame.y,
                    z: -c.frame.z,
                }
            };
            let _ = range;
            Ok(SurfaceGeom::Cylinder(Cylinder {
                frame: Frame3 {
                    origin: f.origin + base,
                    ..f
                },
                radius: c.radius,
            }))
        }
        other => {
            let c = transform_curve(other, &kw_math::Transform3::translation(base));
            Ok(SurfaceGeom::Extrusion {
                profile: Box::new(c),
                dir: d,
            })
        }
    }
}
