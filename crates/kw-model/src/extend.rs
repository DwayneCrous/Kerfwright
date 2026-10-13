//! Extend: carry a surface on past one of its open edges.
//!
//! For every chosen open edge a strip is added on the continuation of the surface the edge belongs to
//! (an analytic surface carried on, a NURBS surface continued with its own last polynomial piece), as
//! wide as the edge and as long as asked, and joined to the surface along the edge. The surface is not
//! changed, so the join is as smooth as the surface itself.

use crate::profile::ModelError;
use crate::sheetops::{greville, interior_breaks, knots_over};
use kw_brep::sew::{BuildOptions, EdgeDesc, FaceDesc, LoopDesc, Soup, UseDesc};
use kw_brep::uvgeom::{point_in_loops, UV};
use kw_brep::{BRep, BodyId, BodyKind, BuildResult, EdgeId, FaceId, LoopKind, Origin, Sense};
use kw_math::{Point3, Vector3};
use kw_nurbs::geom::Line;
use kw_nurbs::{Curve3, CurveGeom, ExtendDir, NurbsCurve, Surface, SurfaceGeom};
use std::collections::HashMap;

fn bad(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// A fit of `f` over `[a, b]` close enough to be an edge, with `f`'s own parameter.
fn fit(
    f: &dyn Fn(f64) -> Point3<f64>,
    a: f64,
    b: f64,
    breaks: &[f64],
    scale: f64,
) -> Result<NurbsCurve, ModelError> {
    let mut per = 4usize;
    loop {
        let knots = knots_over(a, b, breaks, per, 3);
        let g = greville(&knots);
        let c = NurbsCurve::collocate(knots, f).map_err(|e| bad(format!("{e:?}")))?;
        let worst = g
            .windows(2)
            .map(|w| {
                let t = 0.5 * (w[0] + w[1]);
                (c.point_at(t) - f(t)).norm()
            })
            .fold(0.0f64, f64::max);
        if worst <= 2e-8 * scale {
            return Ok(c);
        }
        if per >= 128 {
            return Err(bad(
                "an edge of the extension cannot be followed closely enough",
            ));
        }
        per *= 2;
    }
}

/// The surface carried on past `c` in direction `sigma` along the parameter `across_u`, as a
/// surface whose natural evaluation covers the extension. For an analytic surface that is the
/// surface itself; a NURBS surface is continued by `delta`.
fn continued(
    s: &SurfaceGeom,
    across_u: bool,
    sigma: f64,
    delta: f64,
) -> Result<SurfaceGeom, ModelError> {
    match s {
        SurfaceGeom::Plane(_) => Ok(s.clone()),
        SurfaceGeom::Cylinder(_) | SurfaceGeom::Cone(_) | SurfaceGeom::Extrusion { .. }
            if !across_u =>
        {
            Ok(s.clone())
        }
        SurfaceGeom::Nurbs(n) => {
            let dir = match (across_u, sigma > 0.0) {
                (true, true) => ExtendDir::UEnd,
                (true, false) => ExtendDir::UStart,
                (false, true) => ExtendDir::VEnd,
                (false, false) => ExtendDir::VStart,
            };
            n.extended(dir, delta)
                .map(SurfaceGeom::Nurbs)
                .map_err(|e| bad(format!("the surface cannot be continued that far ({e:?})")))
        }
        _ => Err(bad(
            "only planes, cylinders and cones (along their axis), extrusions and NURBS surfaces can be extended",
        )),
    }
}

/// An open edge that runs along a parameter line of the surface of its face.
pub(crate) struct IsoEdge {
    pub face: FaceId,
    pub pcurve: CurveGeom,
    /// The edge is a line of constant `u` (so the surface continues across `u`).
    pub across_u: bool,
    /// The constant parameter, and the range of the other one along the edge.
    pub c: f64,
    pub w0: f64,
    pub w1: f64,
}

pub(crate) fn iso_edge(rep: &BRep, e: EdgeId) -> Result<IsoEdge, ModelError> {
    let ring = rep.radial_ring(e);
    if ring.len() != 1 {
        return Err(bad("only an open edge of a surface can be used here"));
    }
    let co = ring[0];
    let lp = rep.coedges[co].owner_loop;
    let face = rep.loops[lp].face;
    let edge = &rep.edges[e];
    let pc = rep.coedges[co]
        .pcurve
        .and_then(|id| rep.curves.get(id))
        .ok_or_else(|| bad("the edge has no parameter curve on its face"))?
        .clone();
    let (t0, t1) = edge.param_range;
    let uvs: Vec<UV> = (0..=12)
        .map(|i| {
            let q = pc.point_at(t0 + (t1 - t0) * i as f64 / 12.0);
            (q.x, q.y)
        })
        .collect();
    let flat = |sel: fn(&UV) -> f64| {
        let (lo, hi) = uvs.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, p| {
            (a.0.min(sel(p)), a.1.max(sel(p)))
        });
        hi - lo < 1e-9 * (1.0 + lo.abs())
    };
    let across_u = if flat(|p| p.0) {
        true
    } else if flat(|p| p.1) {
        false
    } else {
        return Err(bad(
            "only an edge that runs along a parameter line of its surface can be used here",
        ));
    };
    let (c, along): (f64, Vec<f64>) = if across_u {
        (uvs[0].0, uvs.iter().map(|p| p.1).collect())
    } else {
        (uvs[0].1, uvs.iter().map(|p| p.0).collect())
    };
    let (w0, w1) = along
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |a, w| {
            (a.0.min(*w), a.1.max(*w))
        });
    Ok(IsoEdge {
        face,
        pcurve: pc,
        across_u,
        c,
        w0,
        w1,
    })
}

/// Extend each of the open `edges` of the surface `body` by `length`.
pub fn extend_sheet(
    rep: &mut BRep,
    body: BodyId,
    edges: &[EdgeId],
    length: f64,
) -> Result<BuildResult, ModelError> {
    if rep.bodies[body].kind != BodyKind::Sheet {
        return Err(bad("only a surface can be extended"));
    }
    if length.is_nan() || length <= 0.0 || !length.is_finite() {
        return Err(ModelError::BadParameter("the extension must be positive"));
    }
    if edges.is_empty() {
        return Err(bad("no edge to extend"));
    }
    let bb = rep.body_bbox(body);
    let scale = 1.0 + bb.min.coords.amax().max(bb.max.coords.amax()) + length;
    // the soup, and which of its edges is which
    let mut order: Vec<EdgeId> = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !order.contains(&e) {
                    order.push(e);
                }
            }
        }
    }
    let index: HashMap<EdgeId, usize> = order.iter().enumerate().map(|(i, e)| (*e, i)).collect();
    let mut soup = Soup::default();
    soup.add_body(rep, body, true);
    let face_list = rep.body_faces(body);
    let free = rep.free_edges(body);
    for (k, &e) in edges.iter().enumerate() {
        if !free.contains(&e) {
            return Err(bad("only an open edge of a surface can be extended"));
        }
        let iso = iso_edge(rep, e)?;
        let (fid, across_u, c, w0, w1) = (iso.face, iso.across_u, iso.c, iso.w0, iso.w1);
        let pc = iso.pcurve;
        let fi = face_list
            .iter()
            .position(|f| *f == fid)
            .expect("a face of the body");
        let face = &rep.faces[fid];
        let surface = rep.surfaces[face.surface].clone();
        let _ = &pc;
        let wm = 0.5 * (w0 + w1);
        let uv = |a: f64, w: f64| if across_u { (a, w) } else { (w, a) };
        // which way is out of the face
        let loops = rep
            .face_uv_loops(fid, 3)
            .ok_or_else(|| bad("a face has no parameter loops"))?;
        let span = match rep.face_uv_bbox(fid) {
            Some((lo, hi)) => {
                if across_u {
                    hi.0 - lo.0
                } else {
                    hi.1 - lo.1
                }
            }
            None => 1.0,
        };
        let probe = 1e-3 * span.max(1e-9);
        let inside = |a: f64| point_in_loops(uv(a, wm), &loops);
        let sigma = match (inside(c - probe), inside(c + probe)) {
            (true, false) => 1.0,
            (false, true) => -1.0,
            _ => {
                return Err(bad(
                    "cannot tell which side of the edge the surface lies on",
                ))
            }
        };
        // the extension in parameters for the length asked: along the middle of the edge
        let speed = {
            let h = 1e-6 * span.max(1e-9);
            let (p, q) = (
                surface.point_at(uv(c, wm).0, uv(c, wm).1),
                surface.point_at(uv(c + sigma * h, wm).0, uv(c + sigma * h, wm).1),
            );
            ((q - p).norm() / h).max(1e-12)
        };
        let arc = |s: &SurfaceGeom, delta: f64| -> f64 {
            let n = 64;
            let pts: Vec<Point3<f64>> = (0..=n)
                .map(|i| {
                    let a = c + sigma * delta * i as f64 / n as f64;
                    let q = uv(a, wm);
                    s.point_at(q.0, q.1)
                })
                .collect();
            pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
        };
        let mut trial = 3.0 * length / speed;
        let mut ext = continued(&surface, across_u, sigma, trial)?;
        let mut guard = 0;
        while arc(&ext, trial) < length {
            guard += 1;
            if guard > 8 {
                return Err(bad("the surface cannot be extended that far"));
            }
            trial *= 2.0;
            ext = continued(&surface, across_u, sigma, trial)?;
        }
        let (mut lo, mut hi) = (0.0, trial);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if arc(&ext, mid) < length {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let delta = 0.5 * (lo + hi);
        let ext = continued(&surface, across_u, sigma, delta)?;
        // the edges of the strip
        let c2 = c + sigma * delta;
        let point = |a: f64, w: f64| {
            let q = uv(a, w);
            ext.point_at(q.0, q.1)
        };
        // where the surface's own parametrisation along the edge has breaks
        let breaks = match &surface {
            SurfaceGeom::Nurbs(n) => {
                interior_breaks(if across_u { n.knots_v() } else { n.knots_u() })
            }
            _ => Vec::new(),
        };
        let far = fit(&|w| point(c2, w), w0, w1, &breaks, scale)?;
        let side = |w: f64| fit(&|s| point(c + sigma * s, w), 0.0, delta, &[], scale);
        let (side0, side1) = (side(w0)?, side(w1)?);
        let pcurve_along = |a: f64| {
            let (o, d) = if across_u {
                (Point3::new(a, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0))
            } else {
                (Point3::new(0.0, a, 0.0), Vector3::new(1.0, 0.0, 0.0))
            };
            CurveGeom::Line(Line { origin: o, dir: d })
        };
        let pcurve_across = |w: f64| {
            let (o, d) = if across_u {
                (Point3::new(c, w, 0.0), Vector3::new(sigma, 0.0, 0.0))
            } else {
                (Point3::new(w, c, 0.0), Vector3::new(0.0, sigma, 0.0))
            };
            CurveGeom::Line(Line { origin: o, dir: d })
        };
        let desc = |curve: NurbsCurve, range: (f64, f64)| EdgeDesc {
            start: curve.point_at(range.0),
            end: curve.point_at(range.1),
            curve: Some(CurveGeom::Nurbs(curve)),
            range,
            tolerance: 0.0,
        };
        let far_e = soup.edges.len();
        soup.edges.push(desc(far, (w0, w1)));
        let s0_e = soup.edges.len();
        soup.edges.push(desc(side0, (0.0, delta)));
        let s1_e = soup.edges.len();
        soup.edges.push(desc(side1, (0.0, delta)));
        // the strip's loop: along the shared edge from the w0 end to the w1 end, out along the w1
        // side, back along the far edge, in along the w0 side
        let shared = index[&e];
        let a0 = point(c, w0);
        let shared_reversed = (soup.edges[shared].start - a0).norm() > 1e-6 * scale;
        let shared_pcurve = soup.faces[fi]
            .loops
            .iter()
            .flat_map(|l| l.uses.iter())
            .find(|u| u.edge == shared)
            .and_then(|u| u.pcurve.clone());
        let uses = vec![
            UseDesc {
                edge: shared,
                reversed: shared_reversed,
                pcurve: shared_pcurve,
                hint: None,
            },
            UseDesc {
                edge: s1_e,
                reversed: false,
                pcurve: Some(pcurve_across(w1)),
                hint: None,
            },
            UseDesc {
                edge: far_e,
                reversed: true,
                pcurve: Some(pcurve_along(c2)),
                hint: None,
            },
            UseDesc {
                edge: s0_e,
                reversed: true,
                pcurve: Some(pcurve_across(w0)),
                hint: None,
            },
        ];
        // the loop must run counter-clockwise for a forward face and clockwise for a reversed one
        let corners = [uv(c, w0), uv(c, w1), uv(c2, w1), uv(c2, w0)];
        let area: f64 = (0..4)
            .map(|i| {
                let (p, q) = (corners[i], corners[(i + 1) % 4]);
                p.0 * q.1 - q.0 * p.1
            })
            .sum();
        soup.faces.push(FaceDesc {
            surface: ext,
            sense: if area > 0.0 {
                Sense::Forward
            } else {
                Sense::Reversed
            },
            loops: vec![LoopDesc {
                kind: LoopKind::Outer,
                uses,
            }],
            origin: Origin::New {
                role: format!("extend-{k}").into(),
            },
        });
    }
    let r = soup.build(
        rep,
        &BuildOptions {
            vertex_tol: 1e-6 * scale,
            one_body: false,
        },
    );
    match r.bodies.as_slice() {
        [b] => Ok(BuildResult {
            body: *b,
            history: r.history,
        }),
        _ => Err(bad(format!(
            "the extended surface could not be built: {}",
            r.rejected.join("; ")
        ))),
    }
}
