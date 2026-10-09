//! Topology-preserving rebuild of a body with some (or all) face surfaces replaced.
//!
//! Every edge curve is recomputed as the intersection of its two adjacent (new) surfaces and every
//! vertex as the common point of the surfaces around it; seam edges are lifted from the old
//! parameterisation. This is the engine behind face offset, draft and shell.

use crate::profile::ModelError;
use kw_brep::{
    add_face_auto, BRep, BodyId, BuildResult, CoedgeId, EdgeId, EntityRef, FaceId, LoopKind,
    LoopSpec, Origin, Sense, ShapeHistory, VertexId,
};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::geom::{Circle, Line};
use kw_nurbs::proj::{curve_param_of, surface_param_of, wrap_near};
use kw_nurbs::{Curve3, CurveGeom, Surface, SurfaceGeom};
use nalgebra::{DMatrix, DVector};
use std::collections::{BTreeSet, HashMap};
use std::f64::consts::TAU;

fn fail(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// Common point of `surfaces` nearest to `start` (Gauss–Newton, minimum-norm steps).
fn solve_vertex(surfaces: &[&SurfaceGeom], start: Point3<f64>, scale: f64) -> Option<Point3<f64>> {
    let mut p = start;
    for _ in 0..60 {
        let mut jac = DMatrix::<f64>::zeros(surfaces.len(), 3);
        let mut res = DVector::<f64>::zeros(surfaces.len());
        let mut worst = 0.0f64;
        for (i, s) in surfaces.iter().enumerate() {
            let r = surface_param_of(s, &p, None);
            let foot = s.point_at(r.u, r.v);
            let n = s.normal_at(r.u, r.v);
            let d = (p - foot).dot(&n);
            worst = worst.max(d.abs());
            for k in 0..3 {
                jac[(i, k)] = n[k];
            }
            res[i] = d;
        }
        if worst <= 1e-14 * (1.0 + scale) {
            return Some(p);
        }
        let svd = jac.clone().svd(true, true);
        let sol = svd.solve(&(-&res), 1e-9).ok()?;
        let step = Vector3::new(sol[0], sol[1], sol[2]);
        if !step.iter().all(|x| x.is_finite()) {
            return None;
        }
        p += step;
    }
    let worst = surfaces
        .iter()
        .map(|s| {
            let r = surface_param_of(s, &p, None);
            r.distance
        })
        .fold(0.0f64, f64::max);
    (worst <= 1e-9 * (1.0 + scale)).then_some(p)
}

struct NewEdge {
    curve: Option<CurveGeom>,
    range: (f64, f64),
    /// New edge runs from the *old end* vertex to the old start vertex (curve direction opposite to the old one).
    swapped: bool,
}

fn dist_to_curve(c: &CurveGeom, p: &Point3<f64>) -> f64 {
    curve_param_of(c, p, None).distance
}

fn circle_through(
    a: Point3<f64>,
    b: Point3<f64>,
    c: Point3<f64>,
) -> Option<(Point3<f64>, Vector3<f64>, f64)> {
    let (ab, ac) = (b - a, c - a);
    let n = ab.cross(&ac);
    let n2 = n.norm_squared();
    if n2 < 1e-24 {
        return None;
    }
    let center =
        a + (n.cross(&ab) * ac.norm_squared() + ac.cross(&n) * ab.norm_squared()) / (2.0 * n2);
    Some((center, n.normalize(), (a - center).norm()))
}

/// Geometry of the new edge between new vertices `pa` (old start) and `pb` (old end).
#[allow(clippy::too_many_arguments)]
fn new_edge(
    rep: &BRep,
    e: EdgeId,
    faces: &[FaceId],
    surf_of: &dyn Fn(FaceId) -> SurfaceGeom,
    changed: &dyn Fn(FaceId) -> bool,
    pa: Point3<f64>,
    pb: Point3<f64>,
    scale: f64,
) -> Result<NewEdge, ModelError> {
    let edge = &rep.edges[e];
    let old = rep
        .curves
        .get(edge.curve.ok_or_else(|| fail("degenerate edge"))?)
        .ok_or_else(|| fail("edge without curve"))?;
    let tol = 1e-7 * (1.0 + scale);
    let (t0, t1) = edge.param_range;
    let tm = 0.5 * (t0 + t1);
    let old_mid = old.point_at(tm);
    let old_dir = old.derivative(tm);
    if !faces.iter().any(|f| changed(*f)) {
        return Ok(NewEdge {
            curve: Some(old.clone()),
            range: edge.param_range,
            swapped: false,
        });
    }
    let periodic = |c: &CurveGeom| matches!(c, CurveGeom::Circle(_) | CurveGeom::Ellipse(_));
    let closed = edge.start == edge.end;
    // ---- candidate curves
    let mut cands: Vec<CurveGeom> = Vec::new();
    if faces.len() == 2 && faces[0] != faces[1] {
        let (sa, sb) = (surf_of(faces[0]), surf_of(faces[1]));
        let ex = kw_ssi::exact::exact(&sa, &sb).ok_or_else(|| {
            fail(format!(
                "edge {e:?}: no closed-form intersection of the new surfaces"
            ))
        })?;
        cands.extend(ex.curves.into_iter().map(|c| c.curve));
    } else {
        // seam: lift the old curve through the surface change
        let face = faces[0];
        let (so, sn) = (&rep.surfaces[rep.faces[face].surface], surf_of(face));
        let lift = |t: f64, hint: Option<(f64, f64)>| -> Point3<f64> {
            let r = surface_param_of(so, &old.point_at(t), hint);
            sn.point_at(r.u, r.v)
        };
        let p0 = lift(t0, None);
        let p1 = lift(t1, None);
        let pm = lift(tm, None);
        match old {
            CurveGeom::Line(_) => cands.push(CurveGeom::Line(Line {
                origin: p0,
                dir: p1 - p0,
            })),
            CurveGeom::Circle(_) => {
                // three well-separated samples (end points coincide for a closed seam)
                let (q0, q1, q2) = (
                    lift(t0, None),
                    lift(t0 + (t1 - t0) / 3.0, None),
                    lift(t0 + 2.0 * (t1 - t0) / 3.0, None),
                );
                let _ = (pm, p1);
                let (c, n, r) = circle_through(q0, q1, q2)
                    .ok_or_else(|| fail(format!("seam edge {e:?} degenerates")))?;
                let x = (q0 - c).normalize();
                cands.push(CurveGeom::Circle(Circle {
                    frame: Frame3 {
                        origin: c,
                        x,
                        y: n.cross(&x),
                        z: n,
                    },
                    radius: r,
                }));
            }
            _ => return Err(fail(format!("seam edge {e:?}: unsupported curve type"))),
        }
    }
    let best = cands
        .into_iter()
        .map(|c| {
            (
                dist_to_curve(&c, &pa)
                    + dist_to_curve(&c, &pb)
                    + (dist_to_curve(&c, &old_mid) - 0.0).abs() * 1e-3,
                c,
            )
        })
        .filter(|(_, c)| dist_to_curve(c, &pa) <= tol && dist_to_curve(c, &pb) <= tol)
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
        .ok_or_else(|| {
            fail(format!(
                "edge {e:?}: the new end vertices do not lie on any intersection curve"
            ))
        })?
        .1;
    // ---- parameter range and direction
    let (ta, tb) = (
        curve_param_of(&best, &pa, None).t,
        curve_param_of(&best, &pb, None).t,
    );
    let (range, swapped) = if periodic(&best) {
        let wrap_up = |from: f64, to: f64| from + (to - from).rem_euclid(TAU);
        if closed {
            // full circle; orientation relative to the old edge decided by the tangent direction
            let d = best.derivative(ta);
            ((ta, ta + TAU), d.dot(&old_dir) < 0.0)
        } else {
            // two arcs join the vertices: pick the one passing nearest the old mid-point
            let fwd = (ta, wrap_up(ta, tb));
            let bwd = (tb, wrap_up(tb, ta));
            let mid = |r: (f64, f64)| best.point_at(0.5 * (r.0 + r.1));
            if (mid(fwd) - old_mid).norm() <= (mid(bwd) - old_mid).norm() {
                (fwd, false)
            } else {
                (bwd, true)
            }
        }
    } else if tb >= ta {
        ((ta, tb), false)
    } else {
        ((tb, ta), true)
    };
    // `swapped` as defined above is relative to curve direction; convert to "opposite the old edge direction"
    let new_dir_matches_old = {
        let tmid = 0.5 * (range.0 + range.1);
        let d = best.derivative(tmid);
        // direction of travel from the new start to the new end vertex along increasing parameter
        let travel = if swapped { -d } else { d };
        travel.dot(&old_dir) >= 0.0 || closed
    };
    let _ = new_dir_matches_old;
    // An edge always runs along increasing curve parameter; `swapped` says that this goes from the
    // old end vertex to the old start vertex.
    Ok(NewEdge {
        curve: Some(best),
        range,
        swapped,
    })
}

pub struct Rebuilt {
    pub body: BodyId,
    pub face_map: HashMap<FaceId, FaceId>,
}

/// Rebuild `body` into `rep` with the surfaces in `new_surf` swapped in. The result is validated.
pub fn rebuild(
    rep: &mut BRep,
    body: BodyId,
    new_surf: &HashMap<FaceId, SurfaceGeom>,
) -> Result<(BuildResult, Rebuilt), ModelError> {
    let faces_all = rep.body_faces(body);
    let scale = 1.0
        + faces_all
            .iter()
            .flat_map(|f| rep.faces[*f].loops.iter())
            .flat_map(|l| rep.loop_coedges(*l))
            .map(|c| {
                rep.vertices[rep.coedge_start(c).unwrap()]
                    .point
                    .coords
                    .amax()
            })
            .fold(0.0f64, f64::max);
    let effective: HashMap<FaceId, SurfaceGeom> = faces_all
        .iter()
        .map(|f| {
            (
                *f,
                new_surf
                    .get(f)
                    .cloned()
                    .unwrap_or_else(|| rep.surfaces[rep.faces[*f].surface].clone()),
            )
        })
        .collect();
    let surf_of = |f: FaceId| -> SurfaceGeom { effective[&f].clone() };
    let changed = |f: FaceId| new_surf.contains_key(&f);

    // ---- adjacency
    let mut vfaces: HashMap<VertexId, BTreeSet<usize>> = HashMap::new();
    let face_idx: HashMap<FaceId, usize> =
        faces_all.iter().enumerate().map(|(i, f)| (*f, i)).collect();
    let mut edges: Vec<EdgeId> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for &f in &faces_all {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                let ed = &rep.edges[e];
                for v in [ed.start, ed.end] {
                    vfaces.entry(v).or_default().insert(face_idx[&f]);
                }
                if seen.insert(e) {
                    edges.push(e);
                }
            }
        }
    }
    // ---- new vertex positions
    let mut vpos: HashMap<VertexId, Point3<f64>> = HashMap::new();
    for (v, fs) in &vfaces {
        let old = rep.vertices[*v].point;
        let surfs: Vec<SurfaceGeom> = fs.iter().map(|i| surf_of(faces_all[*i])).collect();
        if !fs.iter().any(|i| changed(faces_all[*i])) {
            vpos.insert(*v, old);
            continue;
        }
        let refs: Vec<&SurfaceGeom> = surfs.iter().collect();
        let p = solve_vertex(&refs, old, scale).ok_or_else(|| {
            fail(format!(
                "vertex {v:?}: the new surfaces do not meet in a point"
            ))
        })?;
        vpos.insert(*v, p);
    }
    // ---- new edges
    let mut nedge: HashMap<EdgeId, NewEdge> = HashMap::new();
    for &e in &edges {
        let ed = &rep.edges[e];
        if ed.curve.is_none() {
            nedge.insert(
                e,
                NewEdge {
                    curve: None,
                    range: (0.0, 1.0),
                    swapped: false,
                },
            );
            continue;
        }
        let ring = rep.radial_ring(e);
        if ring.len() != 2 {
            return Err(fail(format!("edge {e:?} is not manifold")));
        }
        let fs: Vec<FaceId> = ring
            .iter()
            .map(|c| rep.loops[rep.coedges[*c].owner_loop].face)
            .collect();
        let ne = new_edge(
            rep,
            e,
            &fs,
            &surf_of,
            &changed,
            vpos[&ed.start],
            vpos[&ed.end],
            scale,
        )?;
        nedge.insert(e, ne);
    }
    // ---- emit
    let new_body = rep.add_body(rep.bodies[body].kind);
    let mut history = ShapeHistory::default();
    let mut face_map: HashMap<FaceId, FaceId> = HashMap::new();
    let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
    let mut emap: HashMap<EdgeId, EdgeId> = HashMap::new();
    let old_lumps = rep.bodies[body].lumps.clone();
    for lump in old_lumps {
        let nl = rep.add_lump(new_body);
        for sh in rep.lumps[lump].shells.clone() {
            let ns = rep.add_shell(nl);
            for f in rep.shells[sh].faces.clone() {
                let face = rep.faces[f].clone();
                let sign = if face.sense == Sense::Forward {
                    1.0
                } else {
                    -1.0
                };
                let surface = surf_of(f);
                let sid = rep.add_surface(surface.clone());
                let mut specs: Vec<LoopSpec> = Vec::new();
                for lp in &face.loops {
                    let mut chain = Vec::new();
                    let cos: Vec<CoedgeId> = rep.loop_coedges(*lp);
                    for co in cos {
                        let c = rep.coedges[co].clone();
                        let ed = rep.edges[c.edge].clone();
                        let ne = &nedge[&c.edge];
                        let id = if let Some(id) = emap.get(&c.edge) {
                            *id
                        } else {
                            let mut vget = |v: VertexId, rep: &mut BRep| {
                                *vmap.entry(v).or_insert_with(|| rep.add_vertex(vpos[&v]))
                            };
                            let (vs, ve) = (vget(ed.start, rep), vget(ed.end, rep));
                            let (a, b) = if ne.swapped { (ve, vs) } else { (vs, ve) };
                            let cid = ne.curve.clone().map(|g| rep.add_curve(g));
                            let id = rep.add_edge(a, b, cid, ne.range);
                            emap.insert(c.edge, id);
                            history.record(
                                EntityRef::Edge(id),
                                Origin::Modified(EntityRef::Edge(c.edge)),
                            );
                            id
                        };
                        let sense = if ne.swapped { c.sense.flip() } else { c.sense };
                        chain.push((id, sense));
                    }
                    specs.push(LoopSpec {
                        kind: face
                            .loops
                            .iter()
                            .position(|x| x == lp)
                            .map(|_| rep.loops[*lp].kind)
                            .unwrap_or(LoopKind::Outer),
                        chain,
                    });
                }
                let s2 = surface.clone();
                let nf = add_face_auto(
                    rep,
                    ns,
                    sid,
                    &move |p| {
                        let r = surface_param_of(&s2, p, None);
                        s2.normal_at(r.u, r.v) * sign
                    },
                    &specs,
                )
                .map_err(|e| fail(format!("face {f:?}: {e}")))?;
                face_map.insert(f, nf);
                history.record(EntityRef::Face(nf), Origin::Modified(EntityRef::Face(f)));
            }
        }
    }
    for (v, nv) in &vmap {
        history.record(
            EntityRef::Vertex(*nv),
            Origin::Modified(EntityRef::Vertex(*v)),
        );
    }
    // ---- verification
    if let Err(errs) = rep.validate().into_result() {
        return Err(fail(format!(
            "rebuilt body is invalid: {:?}",
            &errs[..errs.len().min(3)]
        )));
    }
    let vol = rep
        .volume(new_body)
        .ok_or_else(|| fail("volume not computable"))?;
    if vol.is_nan() || vol <= 0.0 {
        return Err(fail(format!(
            "rebuilt body has non-positive volume {vol} (the change collapses the body)"
        )));
    }
    for (f, nf) in &face_map {
        let area = rep.face_area(*nf).unwrap_or(f64::NAN);
        if area.is_nan() || area <= 0.0 {
            return Err(fail(format!("face {f:?} collapsed (area {area})")));
        }
    }
    Ok((
        BuildResult {
            body: new_body,
            history,
        },
        Rebuilt {
            body: new_body,
            face_map,
        },
    ))
}

#[allow(dead_code)]
fn _wrap(x: f64) -> f64 {
    wrap_near(x, 0.0, TAU)
}
