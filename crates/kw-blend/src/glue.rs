//! Exact gluing of the tools of a tangent chain.
//!
//! The tool of each edge of a chain ends in a planar cap, and the caps of neighbouring tools are the same
//! polygon seen from opposite sides. Gluing the tools means dropping each such pair of caps and sharing the
//! boundary edges between the faces that were beside them. No intersection is computed, so the result has
//! no numerical slivers: the cylinder, torus, cone and plane faces of the tools meet in exactly the edges
//! their caps had.

use kw_brep::{BRep, BodyId, BodyKind, CoedgeSpec, CopyOptions, FaceId, LoopKind, Sense};
use kw_math::Point3;
use kw_nurbs::pcurve::pcurve_on;
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};
use std::collections::HashMap;

/// One coedge of a face loop with the geometry needed to rebuild it.
struct Co {
    /// Positions where the traversal of the loop starts / ends, and the middle of the edge.
    a: Point3<f64>,
    b: Point3<f64>,
    mid: Point3<f64>,
    /// The edge's own curve and parameter range, and the direction it runs (its start / end positions).
    curve: Option<CurveGeom>,
    range: (f64, f64),
    edge_start: Point3<f64>,
    edge_end: Point3<f64>,
    sense: Sense,
    pcurve: Option<CurveGeom>,
}

struct FaceData {
    tool: usize,
    old: FaceId,
    surface: SurfaceGeom,
    sense: Sense,
    loops: Vec<(LoopKind, Vec<Co>)>,
}

fn read_faces(rep: &BRep, tool: usize, body: BodyId) -> Option<Vec<FaceData>> {
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        let face = &rep.faces[f];
        let mut loops = Vec::new();
        for lp in &face.loops {
            let mut cos = Vec::new();
            for c in rep.loop_coedges(*lp) {
                let co = &rep.coedges[c];
                let ed = &rep.edges[co.edge];
                let curve = ed.curve.and_then(|k| rep.curves.get(k)).cloned();
                // closed and degenerate edges are not handled here
                if ed.start == ed.end || curve.is_none() {
                    return None;
                }
                let (s, e) = (rep.vertices[ed.start].point, rep.vertices[ed.end].point);
                let mid = curve
                    .as_ref()?
                    .point_at(0.5 * (ed.param_range.0 + ed.param_range.1));
                let (a, b) = match co.sense {
                    Sense::Forward => (s, e),
                    Sense::Reversed => (e, s),
                };
                cos.push(Co {
                    a,
                    b,
                    mid,
                    curve,
                    range: ed.param_range,
                    edge_start: s,
                    edge_end: e,
                    sense: co.sense,
                    pcurve: co.pcurve.and_then(|k| rep.curves.get(k)).cloned(),
                });
            }
            loops.push((rep.loops[*lp].kind, cos));
        }
        out.push(FaceData {
            tool,
            old: f,
            surface: rep.surfaces[face.surface].clone(),
            sense: face.sense,
            loops,
        });
    }
    Some(out)
}

fn outward_normal(f: &FaceData) -> Option<nalgebra::Vector3<f64>> {
    let SurfaceGeom::Plane(p) = &f.surface else {
        return None;
    };
    Some(if f.sense == Sense::Forward {
        p.frame.z
    } else {
        -p.frame.z
    })
}

fn dbg(msg: &str) {
    if std::env::var("KW_BLEND_DEBUG").is_ok() {
        eprintln!("GLUE: {msg}");
    }
}

fn near(p: &Point3<f64>, q: &Point3<f64>, tol: f64) -> bool {
    (p - q).norm() <= tol
}

/// Do two planar single-loop faces bound the same polygon (same edges, same ends, same middles)?
fn same_polygon(a: &FaceData, b: &FaceData, tol: f64) -> bool {
    if a.loops.len() != 1 || b.loops.len() != 1 {
        return false;
    }
    let (la, lb) = (&a.loops[0].1, &b.loops[0].1);
    if la.len() != lb.len() {
        return false;
    }
    let mut used = vec![false; lb.len()];
    for ca in la {
        let hit = lb.iter().enumerate().position(|(k, cb)| {
            !used[k]
                && near(&ca.mid, &cb.mid, tol)
                && ((near(&ca.a, &cb.a, tol) && near(&ca.b, &cb.b, tol))
                    || (near(&ca.a, &cb.b, tol) && near(&ca.b, &cb.a, tol)))
        });
        match hit {
            Some(k) => used[k] = true,
            None => return false,
        }
    }
    true
}

/// Glue `tools` (solid bodies) along their coincident caps into one solid, built in `rep`. Returns the new
/// body and, for every face of it, the face of the input tools it came from. `None` if the tools do not
/// glue into a valid solid (the caller falls back to a Boolean union).
pub fn glue_tools(rep: &mut BRep, tools: &[BodyId]) -> Option<(BodyId, HashMap<FaceId, FaceId>)> {
    let mut all: Vec<FaceData> = Vec::new();
    for (i, t) in tools.iter().enumerate() {
        all.extend(read_faces(rep, i, *t)?);
    }
    let scale = 1.0
        + all
            .iter()
            .flat_map(|f| f.loops.iter().flat_map(|l| l.1.iter()))
            .map(|c| c.a.coords.amax().max(c.b.coords.amax()))
            .fold(0.0f64, f64::max);
    let tol = 1e-9 * scale;
    // pairs of caps: planar faces of different tools with opposite outward normals and one polygon
    let mut removed = vec![false; all.len()];
    let mut glued_any = false;
    for i in 0..all.len() {
        if removed[i] {
            continue;
        }
        let Some(ni) = outward_normal(&all[i]) else {
            continue;
        };
        for j in i + 1..all.len() {
            if removed[j] || all[j].tool == all[i].tool {
                continue;
            }
            let Some(nj) = outward_normal(&all[j]) else {
                continue;
            };
            if ni.dot(&nj) > -1.0 + 1e-9 {
                continue;
            }
            if same_polygon(&all[i], &all[j], tol) {
                removed[i] = true;
                removed[j] = true;
                glued_any = true;
                break;
            }
        }
    }
    if !glued_any {
        dbg("no cap pair found");
        return None;
    }
    // ---- rebuild
    let mut scratch = BRep::new();
    let body = scratch.add_body(BodyKind::Solid);
    let lump = scratch.add_lump(body);
    let shell = scratch.add_shell(lump);
    let mut verts: Vec<(Point3<f64>, kw_brep::VertexId)> = Vec::new();
    let mut vertex = |scratch: &mut BRep, p: Point3<f64>| -> kw_brep::VertexId {
        if let Some((_, v)) = verts.iter().find(|(q, _)| near(&p, q, tol)) {
            return *v;
        }
        let v = scratch.add_vertex(p);
        verts.push((p, v));
        v
    };
    struct NewEdge {
        id: kw_brep::EdgeId,
        start: kw_brep::VertexId,
        end: kw_brep::VertexId,
        mid: Point3<f64>,
        curve: CurveGeom,
        range: (f64, f64),
    }
    let mut edges: Vec<NewEdge> = Vec::new();
    let mut origin: HashMap<FaceId, FaceId> = HashMap::new();
    let mut face_ids: Vec<(FaceId, FaceId)> = Vec::new(); // (scratch face, old face)
    for (k, fd) in all.iter().enumerate() {
        if removed[k] {
            continue;
        }
        let surf = scratch.add_surface(fd.surface.clone());
        let nf = scratch.add_face(shell, surf, fd.sense);
        face_ids.push((nf, fd.old));
        for (kind, cos) in &fd.loops {
            let mut pend: Vec<(kw_brep::EdgeId, Sense, Option<CurveGeom>, (f64, f64))> =
                Vec::with_capacity(cos.len());
            for c in cos {
                let (va, vb) = (vertex(&mut scratch, c.a), vertex(&mut scratch, c.b));
                if va == vb {
                    dbg("an edge collapsed to a point");
                    return None;
                }
                let existing = edges.iter().position(|e| {
                    near(&e.mid, &c.mid, tol)
                        && ((e.start == va && e.end == vb) || (e.start == vb && e.end == va))
                });
                let (eid, forward_of_edge, reuse_pcurve) = match existing {
                    Some(i) => {
                        let e = &edges[i];
                        // the coedge runs from va to vb: forward on the new edge iff that is its direction
                        (e.id, e.start == va, same_curve(&e.curve, e.range, c))
                    }
                    None => {
                        let (s, en) = (
                            vertex(&mut scratch, c.edge_start),
                            vertex(&mut scratch, c.edge_end),
                        );
                        let curve = c.curve.clone()?;
                        let cid = scratch.add_curve(curve.clone());
                        let id = scratch.add_edge(s, en, Some(cid), c.range);
                        edges.push(NewEdge {
                            id,
                            start: s,
                            end: en,
                            mid: c.mid,
                            curve,
                            range: c.range,
                        });
                        (id, s == va, true)
                    }
                };
                // sense of this use relative to the new edge's direction
                let sense = if forward_of_edge {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                let pc = if reuse_pcurve {
                    // the same curve over the same range: the face's pcurve is valid as it is
                    c.pcurve.clone()
                } else {
                    // re-express the shared edge on this face's surface
                    let e = edges.iter().find(|e| e.id == eid)?;
                    let q = kw_nurbs::proj::surface_param_of(
                        &fd.surface,
                        &e.curve.point_at(e.range.0),
                        None,
                    );
                    Some(pcurve_on(
                        &fd.surface,
                        &e.curve,
                        e.range,
                        Some((q.u, q.v)),
                        100.0 * tol,
                    )?)
                };
                let sense = if existing.is_none() { c.sense } else { sense };
                let range = edges.iter().find(|e| e.id == eid)?.range;
                pend.push((eid, sense, pc, range));
            }
            // pcurves of a periodic surface were fitted one by one: carry each across the period seam so
            // that every coedge starts where the previous one ended
            {
                use kw_nurbs::Surface;
                let (pu, pv) = (fd.surface.period_u(), fd.surface.period_v());
                let ends = |p: &CurveGeom, sense: Sense, r: (f64, f64)| {
                    let (t0, t1) = if sense == Sense::Forward {
                        (r.0, r.1)
                    } else {
                        (r.1, r.0)
                    };
                    (p.point_at(t0), p.point_at(t1))
                };
                for k in 1..pend.len() {
                    let prev_end = match &pend[k - 1].2 {
                        Some(p) => ends(p, pend[k - 1].1, pend[k - 1].3).1,
                        None => continue,
                    };
                    let Some(pc) = pend[k].2.clone() else {
                        continue;
                    };
                    let start = ends(&pc, pend[k].1, pend[k].3).0;
                    let du = pu.map_or(0.0, |p| p * ((prev_end.x - start.x) / p).round());
                    let dv = pv.map_or(0.0, |p| p * ((prev_end.y - start.y) / p).round());
                    if du != 0.0 || dv != 0.0 {
                        pend[k].2 = Some(kw_nurbs::fit::translate_pcurve(&pc, du, dv));
                    }
                }
            }
            let specs: Vec<CoedgeSpec> = pend
                .into_iter()
                .map(|(edge, sense, pc, _)| CoedgeSpec {
                    edge,
                    sense,
                    pcurve: pc.map(|p| scratch.add_curve(p)),
                })
                .collect();
            scratch.add_loop(nf, *kind, &specs);
        }
    }
    if let Err(e) = scratch.validate().into_result() {
        dbg(&format!("glued body invalid: {:?}", &e[..e.len().min(4)]));
        return None;
    }
    let vol = scratch.volume(body)?;
    if vol.is_nan() || vol <= 0.0 {
        dbg(&format!("glued body has volume {vol}"));
        return None;
    }
    let (new_body, map) = rep.copy_body_from(&scratch, body, &CopyOptions::default());
    for (sf, old) in face_ids {
        if let Some(nf) = map.faces.get(&sf) {
            origin.insert(*nf, old);
        }
    }
    Some((new_body, origin))
}

/// Is the coedge's own curve the edge's curve over the same range (so its pcurve can be kept)?
fn same_curve(curve: &CurveGeom, range: (f64, f64), c: &Co) -> bool {
    let Some(other) = &c.curve else { return false };
    if (range.0 - c.range.0).abs() > 1e-12 || (range.1 - c.range.1).abs() > 1e-12 {
        return false;
    }
    [0.0, 0.3, 0.7, 1.0].iter().all(|f| {
        let t = range.0 + (range.1 - range.0) * f;
        (curve.point_at(t) - other.point_at(t)).norm() < 1e-9
    })
}
