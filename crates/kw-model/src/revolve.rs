//! Revolution of a planar profile about an axis lying in its plane.

use crate::profile::{Chain, ModelError, Profile};
use kw_brep::{
    add_face_auto, BRep, BodyKind, BuildResult, EdgeId, EntityRef, LoopKind, LoopSpec, Origin,
    Sense, ShapeHistory, VertexId,
};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_nurbs::fit::transform_curve;
use kw_nurbs::geom::{Circle, Cone, Cylinder, Plane, Sphere, Torus};
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};
use std::f64::consts::TAU;

struct Axis {
    o: Point3<f64>,
    a: Vector3<f64>,
    /// In-plane unit direction perpendicular to the axis, pointing to the side the profile lies on.
    r: Vector3<f64>,
}

impl Axis {
    fn rho_z(&self, p: &Point3<f64>) -> (f64, f64) {
        let q = p - self.o;
        (q.dot(&self.r), q.dot(&self.a))
    }
    fn frame_at(&self, z: f64) -> Frame3 {
        Frame3 {
            origin: self.o + self.a * z,
            x: self.r,
            y: self.a.cross(&self.r),
            z: self.a,
        }
    }
}

struct JunctionData {
    rho: f64,
    z: f64,
    on_axis: bool,
    vb: VertexId,
    vr: VertexId,
    /// Circle around the axis (non-degenerate) or a degenerate point edge.
    arc: EdgeId,
}

/// Revolve `profile` by `angle` (radians, `0 < angle ≤ 2π`, right-handed about `axis`).
pub fn revolve(
    rep: &mut BRep,
    profile: &Profile,
    axis_origin: Point3<f64>,
    axis: Vector3<f64>,
    angle: f64,
) -> Result<BuildResult, ModelError> {
    let np = profile.plane.z;
    let scale = 1.0 + profile.plane.origin.coords.amax() + axis_origin.coords.amax();
    let tol = 1e-9 * scale;
    let mut loops: Vec<(Chain, LoopKind, bool)> = Vec::new();
    for (l, kind) in std::iter::once((&profile.outer, LoopKind::Outer))
        .chain(profile.holes.iter().map(|h| (h, LoopKind::Inner)))
    {
        let area = l.check(&np, tol)?;
        if area.abs() < 1e-14 {
            return Err(ModelError::BadProfile("loop has zero area"));
        }
        loops.push((
            Chain::new(profile.plane, l.edges.clone(), true),
            kind,
            area > 0.0,
        ));
    }
    revolve_core(rep, &profile.plane, &loops, axis_origin, axis, angle, false)
}

/// Revolve chains (open or closed) into a surface: one face per segment, no caps. Normals follow
/// `tangent × profile normal` for an open chain and point away from the inside of a closed one.
pub fn revolve_sheet(
    rep: &mut BRep,
    chains: &[Chain],
    axis_origin: Point3<f64>,
    axis: Vector3<f64>,
    angle: f64,
) -> Result<BuildResult, ModelError> {
    let first = chains
        .first()
        .ok_or(ModelError::BadProfile("nothing to revolve"))?;
    let tol = 1e-9 * (1.0 + first.plane.origin.coords.amax() + axis_origin.coords.amax());
    let mut loops: Vec<(Chain, LoopKind, bool)> = Vec::new();
    for c in chains {
        c.check(tol)?;
        let area = c.signed_area(&c.plane.z);
        loops.push((c.clone(), LoopKind::Outer, !c.closed || area > 0.0));
    }
    revolve_core(rep, &first.plane, &loops, axis_origin, axis, angle, true)
}

/// The shared builder: `loops` are the chains with their kind and (for closed ones) whether they run
/// counter-clockwise about the profile normal; a `sheet` has no caps and is a surface.
fn revolve_core(
    rep: &mut BRep,
    plane: &Frame3,
    loops: &[(Chain, LoopKind, bool)],
    axis_origin: Point3<f64>,
    axis: Vector3<f64>,
    angle: f64,
    sheet: bool,
) -> Result<BuildResult, ModelError> {
    if !(angle > 1e-9 && angle <= TAU + 1e-12) {
        return Err(ModelError::BadParameter(
            "revolution angle must be in (0, 2π]",
        ));
    }
    let full = (angle - TAU).abs() < 1e-9;
    let np = plane.z;
    let a = axis.normalize();
    if a.dot(&np).abs() > 1e-9
        || (axis_origin - plane.origin).dot(&np).abs() > 1e-9 * (1.0 + axis_origin.coords.amax())
    {
        return Err(ModelError::BadProfile(
            "the axis must lie in the plane of the profile",
        ));
    }
    let scale = 1.0 + plane.origin.coords.amax() + axis_origin.coords.amax();
    let tol = 1e-9 * scale;
    // the profile must stay on one side of the axis
    let mut r = np.cross(&a).normalize();
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for (l, _, _) in loops {
        for e in &l.edges {
            for k in 0..=24 {
                let t = e.range.0 + (e.range.1 - e.range.0) * k as f64 / 24.0;
                let rho = (e.curve.point_at(t) - axis_origin).dot(&r);
                lo = lo.min(rho);
                hi = hi.max(rho);
            }
        }
    }
    if lo < -tol && hi > tol {
        return Err(ModelError::BadProfile("the profile crosses the axis"));
    }
    if hi <= tol {
        r = -r;
    }
    let ax = Axis {
        o: axis_origin,
        a,
        r,
    };

    let mut history = ShapeHistory::default();
    let body = rep.add_body(if sheet {
        BodyKind::Sheet
    } else {
        BodyKind::Solid
    });
    let lump = rep.add_lump(body);
    let shell = rep.add_shell(lump);
    let rot = Transform3::translation(-axis_origin.coords)
        .then(&Transform3::rotation(a, angle))
        .then(&Transform3::translation(axis_origin.coords));

    // ---- junctions, profile edges and arcs
    let mut jd: Vec<Vec<JunctionData>> = Vec::new();
    let mut eb: Vec<Vec<Option<EdgeId>>> = Vec::new();
    let mut er: Vec<Vec<Option<EdgeId>>> = Vec::new();
    let mut edge_on_axis: Vec<Vec<bool>> = Vec::new();
    for (l, _, _) in loops {
        let n = l.edges.len();
        let mut js: Vec<JunctionData> = Vec::new();
        for p in l.junctions() {
            let (rho, z) = ax.rho_z(&p);
            let on_axis = rho.abs() < tol;
            let vb = rep.add_vertex(p);
            let vr = if full || on_axis {
                vb
            } else {
                rep.add_vertex(rot.apply_point(&p))
            };
            let arc = if on_axis {
                rep.add_edge(vb, vb, None, (0.0, 1.0))
            } else {
                let c = rep.add_curve(CurveGeom::Circle(Circle {
                    frame: ax.frame_at(z),
                    radius: rho,
                }));
                rep.add_edge(vb, vr, Some(c), (0.0, if full { TAU } else { angle }))
            };
            js.push(JunctionData {
                rho,
                z,
                on_axis,
                vb,
                vr,
                arc,
            });
        }
        let mut ebs = Vec::new();
        let mut ers = Vec::new();
        let mut on_ax = Vec::new();
        for i in 0..n {
            let e = &l.edges[i];
            let (s, t) = (&js[i], &js[l.next(i)]);
            let axis_edge = s.on_axis && t.on_axis && matches!(e.curve, CurveGeom::Line(_));
            on_ax.push(axis_edge);
            let (vs, ve) = if e.reversed { (t, s) } else { (s, t) };
            if axis_edge && full {
                ebs.push(None);
                ers.push(None);
                continue;
            }
            let c0 = rep.add_curve(e.curve.clone());
            let e0 = rep.add_edge(vs.vb, ve.vb, Some(c0), e.range);
            ebs.push(Some(e0));
            if full || axis_edge {
                ers.push(Some(e0));
            } else {
                let c1 = rep.add_curve(transform_curve(&e.curve, &rot));
                ers.push(Some(rep.add_edge(vs.vr, ve.vr, Some(c1), e.range)));
            }
        }
        jd.push(js);
        eb.push(ebs);
        er.push(ers);
        edge_on_axis.push(on_ax);
    }

    // ---- swept faces
    for (li, (l, kind, ccw)) in loops.iter().enumerate() {
        let n = l.edges.len();
        for i in 0..n {
            if edge_on_axis[li][i] {
                continue;
            }
            let e = &l.edges[i];
            let (s, t) = (&jd[li][i], &jd[li][l.next(i)]);
            let surface = swept_surface(&ax, &e.curve, e.range, s, t, tol)?;
            let is_plane = matches!(surface, SurfaceGeom::Plane(_));
            // outward direction (in-plane, at angle 0)
            let sign = if *ccw == (*kind == LoopKind::Outer) {
                1.0
            } else {
                -1.0
            };
            let mid = 0.5 * (e.range.0 + e.range.1);
            let tangent = if e.reversed {
                -e.curve.derivative(mid)
            } else {
                e.curve.derivative(mid)
            };
            let outward = tangent.cross(&np).normalize() * sign;
            let surf = rep.add_surface(surface);
            let sense_i = if e.reversed {
                Sense::Reversed
            } else {
                Sense::Forward
            };
            let face = if full && is_plane {
                // annulus / disc about the axis: bounded by the circles only (no seam needed)
                let mut specs = Vec::new();
                let mut circles: Vec<&JunctionData> = vec![s, t];
                circles.retain(|j| !j.on_axis);
                circles.sort_by(|x, y| {
                    y.rho
                        .partial_cmp(&x.rho)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
                for (k, j) in circles.iter().enumerate() {
                    specs.push(LoopSpec {
                        kind: if k == 0 {
                            LoopKind::Outer
                        } else {
                            LoopKind::Inner
                        },
                        chain: vec![(j.arc, Sense::Forward)],
                    });
                }
                if specs.is_empty() {
                    return Err(ModelError::BadProfile("degenerate planar sweep"));
                }
                let _ = &eb[li][i];
                add_face_auto(rep, shell, surf, &move |_| outward, &specs)
            } else {
                let b = eb[li][i].unwrap();
                let (ar_t, ar_s) = (t.arc, s.arc);
                let chain = if full {
                    vec![
                        (b, sense_i),
                        (ar_t, Sense::Forward),
                        (b, sense_i.flip()),
                        (ar_s, Sense::Reversed),
                    ]
                } else {
                    let r_edge = er[li][i].unwrap();
                    vec![
                        (b, sense_i),
                        (ar_t, Sense::Forward),
                        (r_edge, sense_i.flip()),
                        (ar_s, Sense::Reversed),
                    ]
                };
                add_face_auto(
                    rep,
                    shell,
                    surf,
                    &move |_| outward,
                    &[LoopSpec {
                        kind: LoopKind::Outer,
                        chain,
                    }],
                )
            }
            .map_err(ModelError::Construction)?;
            history.record(
                EntityRef::Face(face),
                Origin::New {
                    role: format!("sweep-{li}-{i}").into(),
                },
            );
        }
    }

    // ---- end caps for partial revolutions
    if !full && !sheet {
        let tangent0 = a.cross(&r); // rotation direction at angle 0 (profile side is +r)
        for (cap, frame, outward) in [
            (0, Frame3 { z: np, ..*plane }, -tangent0),
            (
                1,
                Frame3 {
                    origin: rot.apply_point(&plane.origin),
                    x: rot.apply_vector(&plane.x),
                    y: rot.apply_vector(&plane.y),
                    z: rot.apply_vector(&np),
                },
                rot.apply_vector(&tangent0),
            ),
        ] {
            let surf = rep.add_surface(SurfaceGeom::Plane(Plane { frame }));
            let specs: Vec<LoopSpec> = loops
                .iter()
                .enumerate()
                .map(|(li, (l, kind, _))| LoopSpec {
                    kind: *kind,
                    chain: (0..l.edges.len())
                        .map(|i| {
                            let e = if cap == 0 {
                                eb[li][i].unwrap()
                            } else {
                                er[li][i].unwrap()
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
            history.record(
                EntityRef::Face(f),
                Origin::New {
                    role: if cap == 0 { "start-cap" } else { "end-cap" }.into(),
                },
            );
        }
    }
    rep.prune_orphans();
    Ok(BuildResult { body, history })
}

fn swept_surface(
    ax: &Axis,
    curve: &CurveGeom,
    range: (f64, f64),
    s: &JunctionData,
    t: &JunctionData,
    tol: f64,
) -> Result<SurfaceGeom, ModelError> {
    match curve {
        CurveGeom::Line(_) => {
            let (r0, z0, r1, z1) = (s.rho, s.z, t.rho, t.z);
            if (r1 - r0).abs() < tol {
                let f = ax.frame_at(0.0);
                Ok(SurfaceGeom::Cylinder(Cylinder {
                    frame: f,
                    radius: r0,
                }))
            } else if (z1 - z0).abs() < tol {
                Ok(SurfaceGeom::Plane(Plane {
                    frame: ax.frame_at(z0),
                }))
            } else {
                let tan = (r1 - r0) / (z1 - z0);
                let f = ax.frame_at(z0);
                Ok(SurfaceGeom::Cone(Cone {
                    frame: f,
                    radius_at_origin: r0,
                    half_angle: tan.atan(),
                }))
            }
        }
        CurveGeom::Circle(c) => {
            let (rc, zc) = ax.rho_z(&c.frame.origin);
            if rc.abs() < tol {
                let mut f = ax.frame_at(zc);
                f.origin = c.frame.origin;
                Ok(SurfaceGeom::Sphere(Sphere {
                    frame: f,
                    radius: c.radius,
                }))
            } else if c.radius < rc - tol {
                Ok(SurfaceGeom::Torus(Torus {
                    frame: ax.frame_at(zc),
                    major_radius: rc,
                    minor_radius: c.radius,
                }))
            } else {
                Err(ModelError::BadProfile(
                    "arc too large for a torus (self-intersecting revolution)",
                ))
            }
        }
        other => {
            let _ = range;
            Ok(SurfaceGeom::Revolution {
                profile: Box::new(other.clone()),
                axis_origin: ax.o,
                axis_dir: ax.a,
            })
        }
    }
}
