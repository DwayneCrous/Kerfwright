//! Operations on surfaces: stitch, unstitch, offset and thicken.
//!
//! Stitching joins the faces of surface bodies along edges that coincide within a tolerance
//! ([`kw_brep::sew`]); a group that closes becomes a solid. Offsetting moves every face along its
//! normal: analytic surfaces exactly, NURBS surfaces by sampling and refitting, with the deviation
//! checked. Thickening is an offset plus walls along the open boundary, sewn into a solid.

use crate::profile::ModelError;
use kw_brep::sew::{BuildOptions, EdgeDesc, FaceDesc, LoopDesc, Soup, UseDesc};
use kw_brep::{BRep, BodyId, BodyKind, BuildResult, EntityRef, FaceId, LoopKind, Origin, Sense};
use kw_math::{Point3, Vector3};
use kw_nurbs::fit::{offset_surface, transform_curve};
use kw_nurbs::geom::Line;
use kw_nurbs::proj::surface_param_of;
use kw_nurbs::{Curve3, CurveGeom, KnotVector, NurbsCurve, NurbsSurface, Surface, SurfaceGeom};
use std::collections::HashMap;

fn bad(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// The scale of a body: the larger of one and its largest coordinate.
fn scale_of(rep: &BRep, body: BodyId) -> f64 {
    let bb = rep.body_bbox(body);
    1.0 + bb.min.coords.amax().max(bb.max.coords.amax())
}

// ---------------------------------------------------------------------------------------------
// stitch and unstitch

/// Stitch the surface bodies into one body: faces whose open edges lie on one another within `tol`
/// are joined, and a group that closes becomes a solid. Fails if the faces make more than one
/// group, or none that can be built.
pub fn stitch(rep: &mut BRep, bodies: &[BodyId], tol: f64) -> Result<BuildResult, ModelError> {
    if bodies.is_empty() {
        return Err(ModelError::BadParameter("nothing to stitch"));
    }
    if tol.is_nan() || tol <= 0.0 {
        return Err(ModelError::BadParameter("the tolerance must be positive"));
    }
    for b in bodies {
        if rep.bodies[*b].kind == BodyKind::Solid {
            return Err(bad("a solid has no open edges to stitch"));
        }
    }
    let mut soup = Soup::default();
    for b in bodies {
        soup.add_body(rep, *b, true);
    }
    let (merged, mut notes) = soup.weld(tol);
    let r = soup.build(
        rep,
        &BuildOptions {
            vertex_tol: tol,
            one_body: false,
        },
    );
    notes.extend(r.rejected.iter().cloned());
    match r.bodies.as_slice() {
        [body] => Ok(BuildResult {
            body: *body,
            history: r.history,
        }),
        [] => Err(bad(format!(
            "nothing could be stitched: {}",
            notes.join("; ")
        ))),
        many => Err(bad(format!(
            "the faces form {} separate groups ({merged} edges were joined): bodies that do not touch within {tol} cannot be stitched into one{}",
            many.len(),
            if notes.is_empty() {
                String::new()
            } else {
                format!(" ({})", notes.join("; "))
            }
        ))),
    }
}

/// Split a body into its faces: one sheet body whose faces share no edges.
pub fn unstitch(rep: &mut BRep, body: BodyId) -> Result<BuildResult, ModelError> {
    if rep.body_faces(body).len() < 2 {
        return Err(bad(
            "the body is a single face: there is nothing to unstitch",
        ));
    }
    let mut soup = Soup::default();
    soup.add_body(rep, body, false);
    let r = soup.build(
        rep,
        &BuildOptions {
            vertex_tol: 1e-9 * scale_of(rep, body),
            one_body: true,
        },
    );
    match r.bodies.as_slice() {
        [b] => Ok(BuildResult {
            body: *b,
            history: r.history,
        }),
        _ => Err(bad(format!(
            "the faces could not be separated: {}",
            r.rejected.join("; ")
        ))),
    }
}

// ---------------------------------------------------------------------------------------------
// offset

/// Which way a thickened surface grows from the face normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThickenSide {
    /// Along the normal.
    Normal,
    /// Against the normal.
    Reverse,
    /// Half each way.
    Both,
}

struct FaceInfo {
    surface: SurfaceGeom,
    /// +1 where the face looks along the surface normal, -1 against it.
    sign: f64,
}

/// The unit normal the face has at `t` along the first use of the edge.
struct EdgeSampler<'a> {
    edge: &'a EdgeDesc,
    face: &'a FaceInfo,
    pcurve: Option<&'a CurveGeom>,
}

impl EdgeSampler<'_> {
    fn point(&self, t: f64) -> Point3<f64> {
        match &self.edge.curve {
            Some(c) => c.point_at(t),
            None => self.edge.start,
        }
    }

    /// The face normal (looking the way the face does) at the edge point of parameter `t`.
    fn normal(&self, t: f64) -> Result<Vector3<f64>, ModelError> {
        let p = self.point(t);
        let uv = match self.pcurve {
            // the edge's own pcurve is exact, and far cheaper than projecting onto a free-form surface
            Some(pc) if matches!(self.face.surface, SurfaceGeom::Nurbs(_)) => {
                let q = pc.point_at(t);
                (q.x, q.y)
            }
            Some(pc) => {
                let q = pc.point_at(t);
                let r = surface_param_of(&self.face.surface, &p, Some((q.x, q.y)));
                (r.u, r.v)
            }
            None => {
                let r = surface_param_of(&self.face.surface, &p, None);
                (r.u, r.v)
            }
        };
        let n = self.face.surface.normal_at(uv.0, uv.1) * self.face.sign;
        if n.norm() < 0.5 {
            return Err(bad("a surface has no normal where its edge runs"));
        }
        Ok(n.normalize())
    }
}

/// A clamped knot vector over `[a, b]` with a break (a knot of multiplicity `deg`) at every value of
/// `breaks`, and `per` equal spans between neighbouring breaks.
pub(crate) fn knots_over(a: f64, b: f64, breaks: &[f64], per: usize, deg: usize) -> KnotVector {
    let mut k = vec![a; deg + 1];
    let mut at = vec![a];
    at.extend(breaks.iter().copied().filter(|x| *x > a && *x < b));
    at.push(b);
    for w in at.windows(2) {
        for i in 1..per {
            k.push(w[0] + (w[1] - w[0]) * i as f64 / per as f64);
        }
        if w[1] < b {
            k.extend(std::iter::repeat_n(w[1], deg));
        }
    }
    k.extend(vec![b; deg + 1]);
    KnotVector::new(deg, k).expect("a valid clamped knot vector")
}

/// The interior knot values of a knot vector.
pub(crate) fn interior_breaks(k: &KnotVector) -> Vec<f64> {
    let ks = k.as_slice();
    let (a, b) = (ks[0], ks[ks.len() - 1]);
    k.distinct()
        .into_iter()
        .map(|(v, _)| v)
        .filter(|v| *v > a && *v < b)
        .collect()
}

/// The midpoints of neighbouring parameters, thinned to at most `max`.
fn midpoints(g: &[f64], max: usize) -> Vec<f64> {
    let step = (g.len() / max).max(1);
    (0..g.len() - 1)
        .step_by(step)
        .map(|i| 0.5 * (g[i] + g[i + 1]))
        .collect()
}

pub(crate) fn greville(k: &KnotVector) -> Vec<f64> {
    let p = k.degree();
    let ks = k.as_slice();
    (0..k.num_ctrl())
        .map(|i| ks[i + 1..=i + p].iter().sum::<f64>() / p as f64)
        .collect()
}

/// A curve over `range` that follows `f` with the same parameter, closely enough to be an edge.
fn fit_offset_curve(
    f: &dyn Fn(f64) -> Result<Point3<f64>, ModelError>,
    range: (f64, f64),
    breaks: &[f64],
    scale: f64,
) -> Result<NurbsCurve, ModelError> {
    let tol = 2e-8 * scale;
    let failure = std::cell::RefCell::new(None);
    let safe = |t: f64| match f(t) {
        Ok(p) => p,
        Err(e) => {
            *failure.borrow_mut() = Some(e);
            Point3::origin()
        }
    };
    let mut per = 4usize;
    loop {
        let knots = knots_over(range.0, range.1, breaks, per, 3);
        let g = greville(&knots);
        let curve = NurbsCurve::collocate(knots, &safe).map_err(|e| bad(format!("{e:?}")))?;
        let mut worst = 0.0f64;
        for t in midpoints(&g, 200) {
            worst = worst.max((curve.point_at(t) - safe(t)).norm());
        }
        if let Some(e) = failure.borrow_mut().take() {
            return Err(e);
        }
        if worst <= tol {
            return Ok(curve);
        }
        if per >= 128 {
            return Err(bad(format!(
                "an edge of the offset surface cannot be followed closely enough (deviation {worst:.1e})"
            )));
        }
        per *= 2;
    }
}

/// The surface moved by `signed` along its normal.
fn offset_face_surface(
    s: &SurfaceGeom,
    signed: f64,
    scale: f64,
) -> Result<SurfaceGeom, ModelError> {
    if let Some(o) = offset_surface(s, signed) {
        return Ok(o);
    }
    match s {
        SurfaceGeom::Plane(_)
        | SurfaceGeom::Cylinder(_)
        | SurfaceGeom::Cone(_)
        | SurfaceGeom::Sphere(_)
        | SurfaceGeom::Torus(_) => Err(bad(
            "the offset is larger than the radius of curvature of a face",
        )),
        SurfaceGeom::Nurbs(n) => offset_nurbs(n, signed, scale).map(SurfaceGeom::Nurbs),
        _ => Err(bad(
            "only planes, cylinders, cones, spheres, tori and NURBS surfaces can be offset",
        )),
    }
}

/// Offset a NURBS surface: a surface on the original's own breaks that follows `P + d N` with the same
/// parameters (so that edges and their pcurves stay valid), refined until it is within a ten-millionth
/// of the surface's size of the true offset between the nodes.
fn offset_nurbs(s: &NurbsSurface, d: f64, scale: f64) -> Result<NurbsSurface, ModelError> {
    let ((u0, u1), (v0, v1)) = s.domain();
    let (bu, bv) = (interior_breaks(s.knots_u()), interior_breaks(s.knots_v()));
    let failure = std::cell::RefCell::new(None);
    let at = |u: f64, v: f64| -> Point3<f64> {
        match s.normal_at(u, v) {
            Some(n) => s.point_at(u, v) + n * d,
            None => {
                *failure.borrow_mut() = Some(bad("a surface is degenerate where it is offset"));
                Point3::origin()
            }
        }
    };
    let tol = 1e-7 * scale;
    let mut per = 2usize;
    loop {
        let (ku, kv) = (
            knots_over(u0, u1, &bu, per, 3),
            knots_over(v0, v1, &bv, per, 3),
        );
        let (gu, gv) = (greville(&ku), greville(&kv));
        let fit = NurbsSurface::collocate(ku, kv, &at).map_err(|e| bad(format!("{e:?}")))?;
        let mut worst = 0.0f64;
        for u in midpoints(&gu, 40) {
            for v in midpoints(&gv, 40) {
                worst = worst.max((fit.point_at(u, v) - at(u, v)).norm());
            }
        }
        if let Some(e) = failure.borrow_mut().take() {
            return Err(e);
        }
        if worst <= tol {
            return Ok(fit);
        }
        if per >= 64 {
            return Err(bad(format!(
                "the surface cannot be offset closely enough (deviation {worst:.1e}): its curvature may be too tight for this distance"
            )));
        }
        per *= 2;
    }
}

/// The body's faces and edges as a soup (edges shared as in the body) with, per face, its surface.
struct Skin {
    soup: Soup,
    faces: Vec<FaceId>,
    info: Vec<FaceInfo>,
    /// For every edge: the first use of it as (face, loop, use).
    first_use: HashMap<usize, (usize, usize, usize)>,
}

fn skin_of(rep: &BRep, body: BodyId) -> Skin {
    let mut soup = Soup::default();
    soup.add_body(rep, body, true);
    let faces = rep.body_faces(body);
    let info = faces
        .iter()
        .map(|f| FaceInfo {
            surface: rep.surfaces[rep.faces[*f].surface].clone(),
            sign: if rep.faces[*f].sense == Sense::Forward {
                1.0
            } else {
                -1.0
            },
        })
        .collect();
    let mut first_use = HashMap::new();
    for (fi, f) in soup.faces.iter().enumerate() {
        for (li, l) in f.loops.iter().enumerate() {
            for (ui, u) in l.uses.iter().enumerate() {
                first_use.entry(u.edge).or_insert((fi, li, ui));
            }
        }
    }
    Skin {
        soup,
        faces,
        info,
        first_use,
    }
}

impl Skin {
    fn sampler(&self, edge: usize) -> EdgeSampler<'_> {
        let (fi, li, ui) = self.first_use[&edge];
        EdgeSampler {
            edge: &self.soup.edges[edge],
            face: &self.info[fi],
            pcurve: self.soup.faces[fi].loops[li].uses[ui].pcurve.as_ref(),
        }
    }

    /// Faces meeting at an edge must agree about the normal for the offset of one edge to serve both.
    fn check_creases(&self) -> Result<(), ModelError> {
        let mut by_edge: HashMap<usize, Vec<(usize, usize, usize)>> = HashMap::new();
        for (fi, f) in self.soup.faces.iter().enumerate() {
            for (li, l) in f.loops.iter().enumerate() {
                for (ui, u) in l.uses.iter().enumerate() {
                    by_edge.entry(u.edge).or_default().push((fi, li, ui));
                }
            }
        }
        for (e, uses) in by_edge {
            let edge = &self.soup.edges[e];
            let [(fa, la, ua), (fb, lb, ub)] = uses[..] else {
                continue;
            };
            if fa == fb || edge.degenerate() {
                continue;
            }
            let mk = |fi: usize, li: usize, ui: usize| EdgeSampler {
                edge,
                face: &self.info[fi],
                pcurve: self.soup.faces[fi].loops[li].uses[ui].pcurve.as_ref(),
            };
            let (a, b) = (mk(fa, la, ua), mk(fb, lb, ub));
            for k in 0..=4 {
                let t = edge.range.0 + (edge.range.1 - edge.range.0) * k as f64 / 4.0;
                let (na, nb) = (a.normal(t)?, b.normal(t)?);
                if na.dot(&nb) < (0.005f64).cos() {
                    return Err(bad(
                        "two faces meet at a crease (their normals differ along a shared edge): a surface can only be offset where it is smooth. Unstitch it and offset the faces one by one.",
                    ));
                }
            }
        }
        Ok(())
    }
}

/// `skin` moved by `d` along its normals: every face and every edge offset. Where `d` is zero the
/// skin is returned unchanged.
fn offset_skin(
    skin: &Skin,
    d: f64,
    scale: f64,
    role: Option<&'static str>,
) -> Result<Soup, ModelError> {
    let mut soup = skin.soup.clone();
    if d == 0.0 {
        return Ok(soup);
    }
    // the edges
    for (e, desc) in skin.soup.edges.iter().enumerate() {
        let Some(&(fi, li, ui)) = skin.first_use.get(&e) else {
            continue;
        };
        let sampler = skin.sampler(e);
        let face = &skin.info[fi];
        if desc.degenerate() {
            let n = sampler.normal(desc.range.0)?;
            soup.edges[e].start = desc.start + n * d;
            soup.edges[e].end = desc.end + n * d;
            continue;
        }
        let plane_exact = matches!(&face.surface, SurfaceGeom::Plane(_));
        let curve = if plane_exact {
            // a flat face moves rigidly
            let n = sampler.normal(desc.range.0)?;
            transform_curve(
                desc.curve.as_ref().expect("not degenerate"),
                &kw_math::Transform3::translation(n * d),
            )
        } else {
            let breaks = match &desc.curve {
                Some(CurveGeom::Nurbs(c)) => interior_breaks(c.knots()),
                _ => Vec::new(),
            };
            CurveGeom::Nurbs(fit_offset_curve(
                &|t| Ok(sampler.point(t) + sampler.normal(t)? * d),
                desc.range,
                &breaks,
                scale,
            )?)
        };
        // the edge keeps its range and its parametrisation, so the pcurves of the faces stay valid
        let (a, b) = (curve.point_at(desc.range.0), curve.point_at(desc.range.1));
        soup.edges[e] = EdgeDesc {
            curve: Some(curve),
            range: desc.range,
            start: a,
            end: b,
            tolerance: desc.tolerance,
        };
        let _ = (fi, li, ui);
    }
    // the faces
    for (i, f) in soup.faces.iter_mut().enumerate() {
        let info = &skin.info[i];
        let signed = d * info.sign;
        f.surface = offset_face_surface(&info.surface, signed, scale)?;
        f.origin = match role {
            Some(r) => Origin::Generated {
                from: EntityRef::Face(skin.faces[i]),
                role: r.into(),
            },
            None => Origin::Modified(EntityRef::Face(skin.faces[i])),
        };
    }
    Ok(soup)
}

/// Move every face of the surface `body` by `d` along its normal. The surface must be smooth: faces
/// that meet at a crease would offset to different edges.
pub fn offset_sheet(rep: &mut BRep, body: BodyId, d: f64) -> Result<BuildResult, ModelError> {
    if rep.bodies[body].kind != BodyKind::Sheet {
        return Err(bad(
            "only a surface can be offset this way (use Offset Body for a solid)",
        ));
    }
    if d == 0.0 || !d.is_finite() {
        return Err(ModelError::BadParameter("zero offset"));
    }
    let scale = scale_of(rep, body);
    let skin = skin_of(rep, body);
    skin.check_creases()?;
    let soup = offset_skin(&skin, d, scale, None)?;
    let r = soup.build(
        rep,
        &BuildOptions {
            vertex_tol: 1e-5 * scale,
            one_body: false,
        },
    );
    match r.bodies.as_slice() {
        [b] => Ok(BuildResult {
            body: *b,
            history: r.history,
        }),
        _ => Err(bad(format!(
            "the offset surface could not be built: {}",
            r.rejected.join("; ")
        ))),
    }
}

/// Give the surface `body` thickness `t`: a solid between the surface and its offset, closed by walls
/// along the open edges.
pub fn thicken(
    rep: &mut BRep,
    body: BodyId,
    t: f64,
    side: ThickenSide,
) -> Result<BuildResult, ModelError> {
    if rep.bodies[body].kind != BodyKind::Sheet {
        return Err(bad("only a surface can be thickened"));
    }
    if t.is_nan() || t <= 0.0 || !t.is_finite() {
        return Err(ModelError::BadParameter("the thickness must be positive"));
    }
    let scale = scale_of(rep, body);
    let skin = skin_of(rep, body);
    let counts = {
        let mut n = vec![0usize; skin.soup.edges.len()];
        for f in &skin.soup.faces {
            for l in &f.loops {
                for u in &l.uses {
                    n[u.edge] += 1;
                }
            }
        }
        n
    };
    let free: Vec<usize> = (0..skin.soup.edges.len())
        .filter(|e| counts[*e] == 1 && !skin.soup.edges[*e].degenerate())
        .collect();
    if free.is_empty() {
        return Err(bad(
            "the surface is closed: it has no open edge to give walls to (use Shell on a solid)",
        ));
    }
    skin.check_creases()?;
    let (a, b) = match side {
        ThickenSide::Normal => (0.0, t),
        ThickenSide::Reverse => (0.0, -t),
        ThickenSide::Both => (-t / 2.0, t / 2.0),
    };
    let inner = offset_skin(&skin, a, scale, if a == 0.0 { None } else { Some("inner") })?;
    let outer = offset_skin(&skin, b, scale, Some("outer"))?;
    // the two skins, then the walls
    let mut soup = Soup::default();
    let shift = inner.edges.len();
    soup.edges.extend(inner.edges.iter().cloned());
    soup.edges.extend(outer.edges.iter().cloned());
    soup.faces.extend(inner.faces.iter().cloned());
    for f in &outer.faces {
        let mut f = f.clone();
        for l in &mut f.loops {
            for u in &mut l.uses {
                u.edge += shift;
            }
        }
        soup.faces.push(f);
    }
    // a rung joins a vertex of the inner skin to the same vertex of the outer one
    let mut rungs: Vec<(Point3<f64>, usize)> = Vec::new();
    let vtol = 1e-5 * scale;
    let mut rung = |soup: &mut Soup, p: Point3<f64>, q: Point3<f64>| -> usize {
        if let Some((_, i)) = rungs.iter().find(|(x, _)| (x - p).norm() <= vtol) {
            return *i;
        }
        soup.edges.push(EdgeDesc {
            curve: Some(CurveGeom::Line(Line {
                origin: p,
                dir: q - p,
            })),
            range: (0.0, 1.0),
            start: p,
            end: q,
            tolerance: 0.0,
        });
        let i = soup.edges.len() - 1;
        rungs.push((p, i));
        i
    };
    for (k, &e) in free.iter().enumerate() {
        let desc = &skin.soup.edges[e];
        let sampler = skin.sampler(e);
        // samples along the edge on both skins
        let m = 129usize;
        let ts: Vec<f64> = (0..m)
            .map(|i| desc.range.0 + (desc.range.1 - desc.range.0) * i as f64 / (m - 1) as f64)
            .collect();
        let mut grid = Vec::with_capacity(m);
        for t in &ts {
            let (p, n) = (sampler.point(*t), sampler.normal(*t)?);
            grid.push(vec![p + n * a, p + n * b]);
        }
        let wall =
            NurbsSurface::interpolate_uniform(&grid, 3, 1).map_err(|e| bad(format!("{e:?}")))?;
        let (ie, oe) = (e, e + shift);
        let (i_start, i_end) = (soup.edges[ie].start, soup.edges[ie].end);
        let (o_start, o_end) = (soup.edges[oe].start, soup.edges[oe].end);
        let r0 = rung(&mut soup, i_start, o_start);
        let r1 = rung(&mut soup, i_end, o_end);
        // the wall's loop: inner edge, the rung at the end, the outer edge back, the rung at the start.
        // The wall runs from 0 to 1 along the edge and across it, so every pcurve is a straight line.
        let along = |y: f64| {
            let span = desc.range.1 - desc.range.0;
            CurveGeom::Line(Line {
                origin: Point3::new(-desc.range.0 / span, y, 0.0),
                dir: Vector3::new(1.0 / span, 0.0, 0.0),
            })
        };
        let across = |x: f64| {
            CurveGeom::Line(Line {
                origin: Point3::new(x, 0.0, 0.0),
                dir: Vector3::new(0.0, 1.0, 0.0),
            })
        };
        let uses = vec![
            UseDesc {
                edge: ie,
                reversed: false,
                pcurve: Some(along(0.0)),
                hint: None,
            },
            UseDesc {
                edge: r1,
                reversed: false,
                pcurve: Some(across(1.0)),
                hint: None,
            },
            UseDesc {
                edge: oe,
                reversed: true,
                pcurve: Some(along(1.0)),
                hint: None,
            },
            UseDesc {
                edge: r0,
                reversed: true,
                pcurve: Some(across(0.0)),
                hint: None,
            },
        ];
        soup.faces.push(FaceDesc {
            surface: SurfaceGeom::Nurbs(wall),
            sense: Sense::Forward,
            loops: vec![LoopDesc {
                kind: LoopKind::Outer,
                uses,
            }],
            origin: Origin::New {
                role: format!("wall-{k}").into(),
            },
        });
    }
    let r = soup.build(
        rep,
        &BuildOptions {
            vertex_tol: vtol,
            one_body: false,
        },
    );
    match r.bodies.as_slice() {
        [b] if rep.bodies[*b].kind == BodyKind::Solid => Ok(BuildResult {
            body: *b,
            history: r.history,
        }),
        [_] => Err(bad("the thickened surface does not close into a solid")),
        _ => Err(bad(format!(
            "the thickened surface could not be built: {}",
            r.rejected.join("; ")
        ))),
    }
}
