#[cfg(feature = "vertex-blend")]
use crate::vertex::{analyse_corner, build_corner_tool};
use kw_boolean::{boolean, BoolOptions, BooleanError, Op};
use kw_brep::{BRep, BodyId, EdgeId, EntityRef, FaceId, Origin, Sense, ShapeHistory, VertexId};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{extrude, revolve, ExtrudeOptions, ModelError, Profile, ProfileEdge, ProfileLoop};
use kw_nurbs::proj::surface_param_of;
use kw_nurbs::{Curve3, CurveGeom, Surface, SurfaceGeom};
use std::f64::consts::{PI, TAU};

#[derive(Debug, Clone, PartialEq)]
pub enum BlendErrorKind {
    /// Not a manifold edge between two faces of this body.
    BadEdge,
    /// The edge or its adjacent faces are of a kind this implementation does not blend.
    Unsupported(&'static str),
    /// The requested size does not fit (too large for the faces, or degenerate geometry).
    Size(&'static str),
    /// The Boolean that applies the blend failed.
    Boolean(BooleanError),
    Model(ModelError),
}

#[derive(Debug, Clone, PartialEq)]
pub struct BlendError {
    /// The edge (of the input body) that could not be blended.
    pub edge: EdgeId,
    pub kind: BlendErrorKind,
}

impl std::fmt::Display for BlendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "edge {:?}: {:?}", self.edge, self.kind)
    }
}
impl std::error::Error for BlendError {}

#[derive(Clone, Copy)]
enum Style<'a> {
    Fillet(f64),
    Chamfer(f64, f64),
    /// Radius following a law along the edge.
    Law(&'a RadiusLaw),
}

/// How the radius of a variable-radius fillet changes along the edge. The argument is the fraction
/// `t ∈ [0, 1]` of the edge's parameter range (for a closed circular edge: of the turn, from the
/// edge's parameter origin, so that a law on a closed edge must satisfy `r(0) = r(1)`).
#[derive(Clone)]
pub enum RadiusLaw {
    /// `r(t) = start + (end - start) t`.
    Linear { start: f64, end: f64 },
    /// `(t, r)` pairs with strictly increasing `t`, first `0` and last `1`, joined by a monotone
    /// piecewise-cubic (Fritsch–Carlson) interpolant: C¹, no overshoot between the given radii.
    Table(Vec<(f64, f64)>),
    /// Any continuous function (the caller keeps it positive and smooth).
    Function(std::sync::Arc<dyn Fn(f64) -> f64 + Send + Sync>),
}

impl std::fmt::Debug for RadiusLaw {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadiusLaw::Linear { start, end } => write!(f, "Linear({start}, {end})"),
            RadiusLaw::Table(t) => write!(f, "Table({} points)", t.len()),
            RadiusLaw::Function(_) => write!(f, "Function"),
        }
    }
}

impl RadiusLaw {
    fn validate(&self) -> Result<(), &'static str> {
        match self {
            RadiusLaw::Linear { start, end } => {
                if !(*start > 0.0 && *end > 0.0) || !start.is_finite() || !end.is_finite() {
                    return Err("radii must be positive");
                }
            }
            RadiusLaw::Table(t) => {
                if t.len() < 2
                    || (t[0].0 - 0.0).abs() > 1e-12
                    || (t[t.len() - 1].0 - 1.0).abs() > 1e-12
                    || t.windows(2).any(|w| w[1].0 <= w[0].0)
                    || t.iter()
                        .any(|p| p.1 <= 0.0 || p.1.is_nan() || !p.1.is_finite())
                {
                    return Err(
                        "a radius table needs increasing parameters from 0 to 1 and positive radii",
                    );
                }
            }
            RadiusLaw::Function(_) => {}
        }
        Ok(())
    }

    /// Radius at fraction `t` (linearly continued outside `[0, 1]`).
    pub fn at(&self, t: f64) -> f64 {
        match self {
            RadiusLaw::Linear { start, end } => start + (end - start) * t,
            RadiusLaw::Function(f) => {
                if (0.0..=1.0).contains(&t) {
                    f(t)
                } else {
                    let tc = t.clamp(0.0, 1.0);
                    let h = 1e-4;
                    let slope = if t < 0.0 {
                        (f(h) - f(0.0)) / h
                    } else {
                        (f(1.0) - f(1.0 - h)) / h
                    };
                    f(tc) + slope * (t - tc)
                }
            }
            RadiusLaw::Table(pts) => {
                let n = pts.len();
                // Fritsch–Carlson slopes
                let delta: Vec<f64> = pts
                    .windows(2)
                    .map(|w| (w[1].1 - w[0].1) / (w[1].0 - w[0].0))
                    .collect();
                let mut m = vec![0.0; n];
                m[0] = delta[0];
                m[n - 1] = delta[n - 2];
                for k in 1..n - 1 {
                    m[k] = if delta[k - 1] * delta[k] <= 0.0 {
                        0.0
                    } else {
                        0.5 * (delta[k - 1] + delta[k])
                    };
                }
                for k in 0..n - 1 {
                    if delta[k] == 0.0 {
                        m[k] = 0.0;
                        m[k + 1] = 0.0;
                    } else {
                        let (a, b) = (m[k] / delta[k], m[k + 1] / delta[k]);
                        let h = a.hypot(b);
                        if h > 3.0 {
                            let tau = 3.0 / h;
                            m[k] = tau * a * delta[k];
                            m[k + 1] = tau * b * delta[k];
                        }
                    }
                }
                if t <= 0.0 {
                    return pts[0].1 + m[0] * t;
                }
                if t >= 1.0 {
                    return pts[n - 1].1 + m[n - 1] * (t - 1.0);
                }
                let k = pts.partition_point(|p| p.0 <= t).clamp(1, n - 1) - 1;
                let (x0, x1) = (pts[k].0, pts[k + 1].0);
                let h = x1 - x0;
                let s = (t - x0) / h;
                let (s2, s3) = (s * s, s * s * s);
                (2.0 * s3 - 3.0 * s2 + 1.0) * pts[k].1
                    + (s3 - 2.0 * s2 + s) * h * m[k]
                    + (-2.0 * s3 + 3.0 * s2) * pts[k + 1].1
                    + (s3 - s2) * h * m[k + 1]
            }
        }
    }

    fn is_linear(&self) -> Option<(f64, f64)> {
        match self {
            RadiusLaw::Linear { start, end } => Some((*start, *end)),
            _ => None,
        }
    }

    /// Smallest and largest radius over `[0, 1]` (sampled).
    fn range(&self) -> (f64, f64) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for i in 0..=256 {
            let r = self.at(i as f64 / 256.0);
            lo = lo.min(r);
            hi = hi.max(r);
        }
        (lo, hi)
    }
}

/// How the cross-section of the edge is swept.
enum Sweep {
    /// Straight edge: extrude along `dir` from `start` for `length`.
    Line {
        start: Point3<f64>,
        dir: Vector3<f64>,
        length: f64,
    },
    /// Circular rim: revolve about the axis; a partial arc (`span` = edge parameter range) is revolved
    /// over its angular extent only.
    Rim {
        axis_origin: Point3<f64>,
        axis: Vector3<f64>,
        span: Option<(f64, f64)>,
    },
}

pub(crate) struct Section {
    /// Edge point at which the cross-section is taken and the 2D frame (x, y span the section).
    pub(crate) p: Point3<f64>,
    x: Vector3<f64>,
    y: Vector3<f64>,
    u1: Vector3<f64>,
    u2: Vector3<f64>,
    n1: Vector3<f64>,
    n2: Vector3<f64>,
    pub(crate) convex: bool,
    sweep: Sweep,
    /// Planes (point, outward normal) the tool must be trimmed against at the start/end of a straight edge.
    end_planes: [Option<(Point3<f64>, Vector3<f64>)>; 2],
    /// Cross-sections of the two faces when at least one is a cylinder (straight edge, axis parallel to it).
    curved: Option<[Prof2; 2]>,
    /// Does a tangent-continuous edge continue the edge at the start / end of the sweep? The tool then ends
    /// exactly there (no extension, no trimming): the neighbouring tool's cap coincides with it.
    chain_end: [bool; 2],
}

/// Cross-section of a face in the 2D frame of a section (origin at the edge point).
#[derive(Clone, Copy, Debug)]
pub(crate) enum Prof2 {
    /// A line through the origin; `n` is the outward unit normal.
    Line { n: (f64, f64) },
    /// A circle; `sigma` = +1 when the outward normal points away from its centre.
    Circle { c: (f64, f64), r: f64, sigma: f64 },
}

pub(crate) fn err(edge: EdgeId, kind: BlendErrorKind) -> BlendError {
    BlendError { edge, kind }
}

/// Outward unit normal of `face` at the 3D point `p` (on its surface).
pub(crate) fn face_normal(rep: &BRep, face: FaceId, p: &Point3<f64>) -> Vector3<f64> {
    let f = &rep.faces[face];
    let s = &rep.surfaces[f.surface];
    let r = surface_param_of(s, p, None);
    let n = s.normal_at(r.u, r.v);
    if f.sense == Sense::Forward {
        n
    } else {
        -n
    }
}

fn is_axisym(s: &SurfaceGeom) -> Option<(Point3<f64>, Vector3<f64>)> {
    match s {
        SurfaceGeom::Cylinder(c) => Some((c.frame.origin, c.frame.z)),
        SurfaceGeom::Cone(c) => Some((c.frame.origin, c.frame.z)),
        _ => None,
    }
}

/// Direction of `e` leaving `vertex` (unit), for a straight or circular edge.
fn leaving_dir(rep: &BRep, e: EdgeId, vertex: VertexId) -> Option<Vector3<f64>> {
    let edge = rep.edges.get(e)?;
    let curve = rep.curves.get(edge.curve?)?;
    let (t0, t1) = edge.param_range;
    let vp = rep.vertices[vertex].point;
    let t = if (curve.point_at(t0) - vp).norm() <= (curve.point_at(t1) - vp).norm() {
        t0
    } else {
        t1
    };
    let d = curve.derivative(t);
    if d.norm() < 1e-300 {
        return None;
    }
    let d = d.normalize();
    let inward = curve.point_at(0.5 * (t0 + t1)) - vp;
    Some(if d.dot(&inward) >= 0.0 { d } else { -d })
}

/// The edge that continues `e` tangentially through `vertex`: at a vertex of three edges, the edge that
/// shares one face with `e`, leaves in the opposite direction, and closes a tangent-continuous (smooth)
/// third edge between the two remaining faces. The rounded-rectangle corner: top line, top arc and the
/// vertical seam between the side plane and the corner cylinder.
pub(crate) fn chain_neighbor(
    rep: &BRep,
    body: BodyId,
    e: EdgeId,
    vertex: VertexId,
) -> Option<EdgeId> {
    let mut inc: Vec<EdgeId> = Vec::new();
    for (eid, ed) in rep.edges.iter() {
        if (ed.start == vertex || ed.end == vertex) && ed.start != ed.end {
            inc.push(eid);
        }
    }
    if inc.len() != 3 || !inc.contains(&e) {
        return None;
    }
    let body_faces = rep.body_faces(body);
    let ring = |x: EdgeId| -> Option<[FaceId; 2]> {
        let r = rep.faces_around_edge(x);
        (r.len() == 2 && r[0] != r[1] && r.iter().all(|f| body_faces.contains(f)))
            .then(|| [r[0], r[1]])
    };
    let fe = ring(e)?;
    let de = leaving_dir(rep, e, vertex)?;
    let vp = rep.vertices[vertex].point;
    for &o in inc.iter().filter(|x| **x != e) {
        let Some(fo) = ring(o) else { continue };
        let shared: Vec<FaceId> = fe.iter().copied().filter(|f| fo.contains(f)).collect();
        if shared.len() != 1 {
            continue;
        }
        let b = *fe.iter().find(|f| **f != shared[0])?;
        let c = *fo.iter().find(|f| **f != shared[0])?;
        let Some(d_o) = leaving_dir(rep, o, vertex) else {
            continue;
        };
        if de.dot(&d_o) > -1.0 + 1e-6 {
            continue;
        }
        let (nb, nc) = (face_normal(rep, b, &vp), face_normal(rep, c, &vp));
        if nb.dot(&nc) > 1.0 - 1e-6 {
            return Some(o);
        }
    }
    None
}

/// The tangent chain through `edges`: `edges` plus every edge reached from them across tangent-continuous
/// junctions (see [`fillet`]). The order is the order of discovery. This is what a fillet or chamfer of
/// `edges` blends; a UI can use it to show the selection that a click will really affect.
pub fn tangent_chain(rep: &BRep, body: BodyId, edges: &[EdgeId]) -> Vec<EdgeId> {
    expand_chains(rep, body, edges)
}

/// The two faces of `e` in the order the blend analysis uses for them (the radial ring).
fn face_pair(rep: &BRep, e: EdgeId) -> Option<[FaceId; 2]> {
    let ring = rep.radial_ring(e);
    (ring.len() == 2).then(|| {
        [
            rep.loops[rep.coedges[ring[0]].owner_loop].face,
            rep.loops[rep.coedges[ring[1]].owner_loop].face,
        ]
    })
}

/// `edges` plus every edge reached from them through tangent-continuous junctions, each with a flag that
/// tells whether its two faces are in the opposite order to the ones of the edge it was reached from
/// (so that "the first distance of a chamfer" stays on the same side of the chain).
pub(crate) fn expand_chains_oriented(
    rep: &BRep,
    body: BodyId,
    edges: &[EdgeId],
) -> Vec<(EdgeId, bool)> {
    let mut out: Vec<(EdgeId, bool)> = Vec::new();
    for &root in edges {
        if out.iter().any(|(e, _)| *e == root) {
            continue;
        }
        let start = out.len();
        out.push((root, false));
        let mut k = start;
        while k < out.len() {
            let (e, flip) = out[k];
            k += 1;
            let Some(ed) = rep.edges.get(e) else { continue };
            for v in [ed.start, ed.end] {
                let Some(n) = chain_neighbor(rep, body, e, v) else {
                    continue;
                };
                if out.iter().any(|(x, _)| *x == n) {
                    continue;
                }
                let (Some(fe), Some(fnn)) = (face_pair(rep, e), face_pair(rep, n)) else {
                    out.push((n, flip));
                    continue;
                };
                let Some(shared) = fe.iter().position(|f| fnn.contains(f)) else {
                    out.push((n, flip));
                    continue;
                };
                let shared_face = fe[shared];
                let in_n = if fnn[0] == shared_face { 0 } else { 1 };
                out.push((n, flip ^ (in_n != shared)));
            }
        }
    }
    out
}

/// `edges` plus every edge reached from them through tangent-continuous junctions.
pub(crate) fn expand_chains(rep: &BRep, body: BodyId, edges: &[EdgeId]) -> Vec<EdgeId> {
    expand_chains_oriented(rep, body, edges)
        .into_iter()
        .map(|(e, _)| e)
        .collect()
}

/// The planar face that closes `e` at `vertex` (the one face of the body there other than the two of `e`).
fn closing_plane(
    rep: &BRep,
    body: BodyId,
    e: EdgeId,
    faces: [FaceId; 2],
    vertex: VertexId,
) -> Result<(Point3<f64>, Vector3<f64>), BlendError> {
    let body_faces = rep.body_faces(body);
    let mut third: Vec<FaceId> = Vec::new();
    for (eid, other) in rep.edges.iter() {
        if (other.start == vertex || other.end == vertex) && eid != e {
            for f in rep.faces_around_edge(eid) {
                if f != faces[0] && f != faces[1] && body_faces.contains(&f) && !third.contains(&f)
                {
                    third.push(f);
                }
            }
        }
    }
    if third.len() != 1 {
        return Err(err(
            e,
            BlendErrorKind::Unsupported("edge end vertices must join exactly three faces"),
        ));
    }
    let SurfaceGeom::Plane(_) = &rep.surfaces[rep.faces[third[0]].surface] else {
        return Err(err(
            e,
            BlendErrorKind::Unsupported(
                "edge ends must be closed by planar faces (or continue tangentially)",
            ),
        ));
    };
    let vp = rep.vertices[vertex].point;
    Ok((vp, face_normal(rep, third[0], &vp)))
}

pub(crate) fn analyse(rep: &BRep, body: BodyId, e: EdgeId) -> Result<Section, BlendError> {
    let edge = rep
        .edges
        .get(e)
        .ok_or_else(|| err(e, BlendErrorKind::BadEdge))?;
    let ring = rep.radial_ring(e);
    if ring.len() != 2 {
        return Err(err(e, BlendErrorKind::BadEdge));
    }
    let body_faces = rep.body_faces(body);
    let faces: Vec<FaceId> = ring
        .iter()
        .map(|c| rep.loops[rep.coedges[*c].owner_loop].face)
        .collect();
    if faces[0] == faces[1] || !faces.iter().all(|f| body_faces.contains(f)) {
        return Err(err(
            e,
            BlendErrorKind::Unsupported("edge is a seam or does not belong to the body"),
        ));
    }
    let curve = edge
        .curve
        .and_then(|c| rep.curves.get(c))
        .ok_or_else(|| err(e, BlendErrorKind::BadEdge))?;
    let (t0, t1) = edge.param_range;
    let tm = 0.5 * (t0 + t1);
    let p = curve.point_at(tm);
    // coedge tangents (in each face's loop direction) and inward directions
    let mut u = [Vector3::zeros(); 2];
    let mut n = [Vector3::zeros(); 2];
    for (k, co) in ring.iter().enumerate() {
        let c = &rep.coedges[*co];
        let mut t = curve.derivative(tm);
        if c.sense == Sense::Reversed {
            t = -t;
        }
        let t = t.normalize();
        let nk = face_normal(rep, faces[k], &p);
        n[k] = nk;
        u[k] = nk.cross(&t).normalize();
    }
    let d_curve = curve.derivative(tm).normalize();
    // cross-section frame ⟂ edge direction
    let convex = u[0].dot(&n[1]) < 0.0;
    let s1 = &rep.surfaces[rep.faces[faces[0]].surface];
    let s2 = &rep.surfaces[rep.faces[faces[1]].surface];
    match curve {
        CurveGeom::Line(_) => {
            // each face is a plane or a cylinder whose axis runs along the edge
            let parallel_cyl = |s: &SurfaceGeom| match s {
                SurfaceGeom::Cylinder(c) => c.frame.z.cross(&d_curve).norm() < 1e-9,
                _ => false,
            };
            let ok = |s: &SurfaceGeom| matches!(s, SurfaceGeom::Plane(_)) || parallel_cyl(s);
            if !ok(s1) || !ok(s2) {
                return Err(err(
                    e,
                    BlendErrorKind::Unsupported(
                        "straight edges are blended only between planes and cylinders parallel to the edge",
                    ),
                ));
            }
            // the face closing each end must be a plane; if it is not perpendicular to the edge the tool is
            // trimmed against it. An end where a tangent-continuous edge goes on is a chain junction: the
            // tool ends exactly there.
            let mut end_planes: [Option<(Point3<f64>, Vector3<f64>)>; 2] = [None, None];
            let mut chain_v = [false; 2];
            for (k, vtx) in [edge.start, edge.end].into_iter().enumerate() {
                if chain_neighbor(rep, body, e, vtx).is_some() {
                    chain_v[k] = true;
                    continue;
                }
                let (vp, n3) = closing_plane(rep, body, e, [faces[0], faces[1]], vtx)?;
                if n3.cross(&d_curve).norm() > 1e-9 {
                    end_planes[k] = Some((vp, n3));
                }
            }
            let (pa, pb) = (rep.vertices[edge.start].point, rep.vertices[edge.end].point);
            let length = (pb - pa).norm();
            let x = u[0] - d_curve * u[0].dot(&d_curve);
            let x = x.normalize();
            let y = d_curve.cross(&x);
            let to2 = |v: Vector3<f64>| (v.dot(&x), v.dot(&y));
            let prof = |k: usize, s: &SurfaceGeom| -> Prof2 {
                match s {
                    SurfaceGeom::Cylinder(c) => {
                        let axis = c.frame.z;
                        let c3 = c.frame.origin + axis * (p - c.frame.origin).dot(&axis);
                        let radial = p - c3;
                        Prof2::Circle {
                            c: to2(c3 - p),
                            r: c.radius,
                            sigma: if n[k].dot(&radial) >= 0.0 { 1.0 } else { -1.0 },
                        }
                    }
                    _ => {
                        let q = to2(n[k]);
                        let l = q.0.hypot(q.1).max(1e-300);
                        Prof2::Line {
                            n: (q.0 / l, q.1 / l),
                        }
                    }
                }
            };
            let curved = (matches!(s1, SurfaceGeom::Cylinder(_))
                || matches!(s2, SurfaceGeom::Cylinder(_)))
            .then(|| [prof(0, s1), prof(1, s2)]);
            let (start, dir) = if (curve.point_at(t1) - curve.point_at(t0)).dot(&d_curve) > 0.0 {
                (pa, d_curve)
            } else {
                (pb, d_curve)
            };
            let _ = start;
            let origin = if (pb - pa).dot(&d_curve) >= 0.0 {
                pa
            } else {
                pb
            };
            Ok(Section {
                p,
                x,
                y,
                u1: u[0],
                u2: u[1],
                n1: n[0],
                n2: n[1],
                convex,
                sweep: Sweep::Line {
                    start: origin,
                    dir,
                    length,
                },
                end_planes,
                curved,
                chain_end: if origin == pa {
                    chain_v
                } else {
                    [chain_v[1], chain_v[0]]
                },
            })
        }
        CurveGeom::Circle(c) => {
            // both faces must be coaxial with the circle: a plane ⟂ axis, a cylinder or a cone
            let axis = c.frame.z;
            let coaxial = |s: &SurfaceGeom| -> bool {
                match s {
                    SurfaceGeom::Plane(pl) => pl.frame.z.cross(&axis).norm() < 1e-9,
                    other => is_axisym(other).is_some_and(|(o, a)| {
                        a.cross(&axis).norm() < 1e-9
                            && (o - c.frame.origin).cross(&axis).norm() < 1e-7 * (1.0 + c.radius)
                    }),
                }
            };
            if !coaxial(s1) || !coaxial(s2) {
                return Err(err(
                    e,
                    BlendErrorKind::Unsupported(
                        "rim edges are blended only between faces coaxial with the edge",
                    ),
                ));
            }
            let full = (t1 - t0 - TAU).abs() < 1e-9;
            // a partial arc ends at a tangent junction or at a plane perpendicular to it (through the axis)
            let mut chain_end = [false; 2];
            if !full {
                for (k, tt) in [t0, t1].into_iter().enumerate() {
                    let pe = curve.point_at(tt);
                    let vtx = if (rep.vertices[edge.start].point - pe).norm()
                        <= (rep.vertices[edge.end].point - pe).norm()
                    {
                        edge.start
                    } else {
                        edge.end
                    };
                    if chain_neighbor(rep, body, e, vtx).is_some() {
                        chain_end[k] = true;
                        continue;
                    }
                    let (_, n3) = closing_plane(rep, body, e, [faces[0], faces[1]], vtx)?;
                    let tangent = curve.derivative(tt).normalize();
                    if n3.cross(&tangent).norm() > 1e-6 {
                        return Err(err(
                            e,
                            BlendErrorKind::Unsupported(
                                "a partial circular edge must end at a tangent junction or at a plane perpendicular to it",
                            ),
                        ));
                    }
                }
            }
            let radial = (p - c.frame.origin) - axis * (p - c.frame.origin).dot(&axis);
            let x = radial.normalize();
            let y = axis;
            Ok(Section {
                p,
                x,
                y,
                u1: u[0],
                u2: u[1],
                n1: n[0],
                n2: n[1],
                convex,
                sweep: Sweep::Rim {
                    axis_origin: c.frame.origin,
                    axis,
                    span: (!full).then_some((t0, t1)),
                },
                end_planes: [None, None],
                curved: None,
                chain_end,
            })
        }
        _ => Err(err(
            e,
            BlendErrorKind::Unsupported("only straight and circular edges are blended"),
        )),
    }
}

/// The spandrel cross-section of the blend at the section point of `sec` (profile edges in 3D), and the
/// blend size. The region between the sharp corner and the blend profile, plus a margin that keeps the
/// tool's faces clear of the body's.
fn section_loop(
    sec: &Section,
    e: EdgeId,
    style: Style<'_>,
) -> Result<(ProfileLoop, f64), BlendError> {
    if let Some(profs) = &sec.curved {
        return match style {
            Style::Fillet(r) => section_loop_curved(sec, e, profs, r),
            _ => Err(err(
                e,
                BlendErrorKind::Unsupported(
                    "chamfers of edges with cylindrical faces are not supported",
                ),
            )),
        };
    }
    // ---- 2D section geometry
    let to2 = |v: Vector3<f64>| (v.dot(&sec.x), v.dot(&sec.y));
    let (u1, u2) = (to2(sec.u1), to2(sec.u2));
    let (n1, n2) = (to2(sec.n1), to2(sec.n2));
    let norm = |v: (f64, f64)| (v.0.hypot(v.1)).max(1e-300);
    let (u1, u2, n1, n2) = (
        (u1.0 / norm(u1), u1.1 / norm(u1)),
        (u2.0 / norm(u2), u2.1 / norm(u2)),
        (n1.0 / norm(n1), n1.1 / norm(n1)),
        (n2.0 / norm(n2), n2.1 / norm(n2)),
    );
    let cosa = (u1.0 * u2.0 + u1.1 * u2.1).clamp(-1.0, 1.0);
    let alpha = cosa.acos();
    if alpha < 1e-3 || alpha > PI - 1e-3 {
        return Err(err(
            e,
            BlendErrorKind::Size("edge is (nearly) tangent-continuous or degenerate"),
        ));
    }
    let (t1, t2, center, radius) = match style {
        Style::Law(_) => unreachable!("sections of a radius law are built per station"),
        Style::Fillet(r) => {
            let t = r / (alpha / 2.0).tan();
            let bis = (u1.0 + u2.0, u1.1 + u2.1);
            let bn = norm(bis);
            let c_dist = r / (alpha / 2.0).sin();
            (
                (u1.0 * t, u1.1 * t),
                (u2.0 * t, u2.1 * t),
                Some(((bis.0 / bn) * c_dist, (bis.1 / bn) * c_dist)),
                r,
            )
        }
        Style::Chamfer(d1, d2) => ((u1.0 * d1, u1.1 * d1), (u2.0 * d2, u2.1 * d2), None, 0.0),
    };
    // the section point P is the origin of the 2D frame; sizes must fit the faces
    let size = match style {
        Style::Law(_) => unreachable!(),
        Style::Fillet(r) => r,
        Style::Chamfer(d1, d2) => d1.max(d2),
    };
    if size.is_nan() || size <= 0.0 {
        return Err(err(e, BlendErrorKind::Size("size must be positive")));
    }
    let tmax = norm(t1).max(norm(t2));
    let m = (0.3 * size).max(1e-3 * tmax);
    // margin direction: out of the material for a convex edge (subtract), into it for a concave one (add)
    let sgn = if sec.convex { 1.0 } else { -1.0 };
    let a = (t1.0 + sgn * n1.0 * m, t1.1 + sgn * n1.1 * m);
    let d = (t2.0 + sgn * n2.0 * m, t2.1 + sgn * n2.1 * m);
    let b = (sgn * (n1.0 + n2.0) * m, sgn * (n1.1 + n2.1) * m);
    // polygon: T1 → (arc | straight) → T2 → D → B → A → T1
    let to3 = |q: (f64, f64)| sec.p + sec.x * q.0 + sec.y * q.1;
    let plane_dir = sec.x.cross(&sec.y);
    let mut edges: Vec<ProfileEdge> = Vec::new();
    match center {
        Some(c) => {
            let cf = Frame3 {
                origin: to3(c),
                x: sec.x,
                y: sec.y,
                z: plane_dir,
            };
            let ang = |q: (f64, f64)| (q.1 - c.1).atan2(q.0 - c.0);
            let (a1, a2) = (ang(t1), ang(t2));
            // shorter way round from T1 to T2
            let mut da = a2 - a1;
            while da > PI {
                da -= TAU;
            }
            while da < -PI {
                da += TAU;
            }
            let (lo, hi, rev) = if da >= 0.0 {
                (a1, a1 + da, false)
            } else {
                (a1 + da, a1, true)
            };
            let mut arc = ProfileEdge::arc(&cf, radius, lo, hi);
            arc.reversed = rev;
            edges.push(arc);
        }
        None => edges.push(ProfileEdge::line(to3(t1), to3(t2))),
    }
    edges.push(ProfileEdge::line(to3(t2), to3(d)));
    edges.push(ProfileEdge::line(to3(d), to3(b)));
    edges.push(ProfileEdge::line(to3(b), to3(a)));
    edges.push(ProfileEdge::line(to3(a), to3(t1)));
    let loop_ = ProfileLoop::new(edges);
    Ok((loop_, size))
}

/// Spandrel section of a fillet of radius `rho` between faces whose cross-sections are lines or circles.
fn section_loop_curved(
    sec: &Section,
    e: EdgeId,
    profs: &[Prof2; 2],
    rho: f64,
) -> Result<(ProfileLoop, f64), BlendError> {
    if rho <= 0.0 || rho.is_nan() || !rho.is_finite() {
        return Err(err(e, BlendErrorKind::Size("size must be positive")));
    }
    let s = if sec.convex { 1.0 } else { -1.0 };
    let to2 = |v: Vector3<f64>| (v.dot(&sec.x), v.dot(&sec.y));
    let (n1, n2) = (to2(sec.n1), to2(sec.n2));
    let unit = |v: (f64, f64)| {
        let l = v.0.hypot(v.1).max(1e-300);
        (v.0 / l, v.1 / l)
    };
    let (n1, n2) = (unit(n1), unit(n2));
    let denom = 1.0 + n1.0 * n2.0 + n1.1 * n2.1;
    if denom < 1e-6 {
        return Err(err(
            e,
            BlendErrorKind::Size("edge is (nearly) tangent-continuous or degenerate"),
        ));
    }
    // initial centre as for two lines
    let c0 = (
        -s * rho * (n1.0 + n2.0) / denom,
        -s * rho * (n1.1 + n2.1) / denom,
    );
    // offset curves of the two sections at signed distance -s·rho along the outward normal
    #[derive(Clone, Copy)]
    enum Off {
        Line { n: (f64, f64), d: f64 },
        Circle { c: (f64, f64), r: f64 },
    }
    let offsets: Vec<Off> = profs
        .iter()
        .map(|p| match *p {
            Prof2::Line { n } => Ok(Off::Line { n, d: -s * rho }),
            Prof2::Circle { c, r, sigma } => {
                let rr = r - sigma * s * rho;
                if rr <= 1e-12 {
                    Err(err(
                        e,
                        BlendErrorKind::Size("radius too large for the cylindrical face"),
                    ))
                } else {
                    Ok(Off::Circle { c, r: rr })
                }
            }
        })
        .collect::<Result<_, _>>()?;
    // intersections of two offset curves
    let mut cands: Vec<(f64, f64)> = Vec::new();
    match (offsets[0], offsets[1]) {
        (Off::Line { n: na, d: da }, Off::Line { n: nb, d: db }) => {
            let det = na.0 * nb.1 - na.1 * nb.0;
            if det.abs() > 1e-12 {
                cands.push(((da * nb.1 - db * na.1) / det, (na.0 * db - nb.0 * da) / det));
            }
        }
        (Off::Line { n, d }, Off::Circle { c, r }) | (Off::Circle { c, r }, Off::Line { n, d }) => {
            // points x with n·x = d and |x - c| = r
            let h = d - (n.0 * c.0 + n.1 * c.1); // signed distance from c to the line along n
            if h.abs() <= r {
                let w = (r * r - h * h).sqrt();
                let foot = (c.0 + n.0 * h, c.1 + n.1 * h);
                cands.push((foot.0 - n.1 * w, foot.1 + n.0 * w));
                cands.push((foot.0 + n.1 * w, foot.1 - n.0 * w));
            }
        }
        (Off::Circle { c: ca, r: ra }, Off::Circle { c: cb, r: rb }) => {
            let (dx, dy) = (cb.0 - ca.0, cb.1 - ca.1);
            let dd = dx.hypot(dy);
            if dd > 1e-12 && dd <= ra + rb && dd >= (ra - rb).abs() {
                let a = (ra * ra - rb * rb + dd * dd) / (2.0 * dd);
                let hh = (ra * ra - a * a).max(0.0).sqrt();
                let m = (ca.0 + dx * a / dd, ca.1 + dy * a / dd);
                cands.push((m.0 - dy / dd * hh, m.1 + dx / dd * hh));
                cands.push((m.0 + dy / dd * hh, m.1 - dx / dd * hh));
            }
        }
    }
    let c = cands
        .into_iter()
        .min_by(|a, b| {
            let da = (a.0 - c0.0).hypot(a.1 - c0.1);
            let db = (b.0 - c0.0).hypot(b.1 - c0.1);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or_else(|| {
            err(
                e,
                BlendErrorKind::Size("the fillet does not fit between the faces"),
            )
        })?;
    // contact points and the face normals there
    let contact = |p: &Prof2| -> ((f64, f64), (f64, f64)) {
        match *p {
            Prof2::Line { n } => ((c.0 + s * rho * n.0, c.1 + s * rho * n.1), n),
            Prof2::Circle { c: cc, r, sigma } => {
                let d = unit((c.0 - cc.0, c.1 - cc.1));
                let t = (cc.0 + r * d.0, cc.1 + r * d.1);
                (t, (sigma * d.0, sigma * d.1))
            }
        }
    };
    let ((t1, m1), (t2, m2)) = (contact(&profs[0]), contact(&profs[1]));
    let size = rho;
    let m = (0.3 * size).max(1e-9);
    let a = (t1.0 + s * m1.0 * m, t1.1 + s * m1.1 * m);
    let d = (t2.0 + s * m2.0 * m, t2.1 + s * m2.1 * m);
    let b = (s * (n1.0 + n2.0) * m, s * (n1.1 + n2.1) * m);
    let to3 = |q: (f64, f64)| sec.p + sec.x * q.0 + sec.y * q.1;
    let plane_dir = sec.x.cross(&sec.y);
    let cf = Frame3 {
        origin: to3(c),
        x: sec.x,
        y: sec.y,
        z: plane_dir,
    };
    let ang = |q: (f64, f64)| (q.1 - c.1).atan2(q.0 - c.0);
    let (a1, a2) = (ang(t1), ang(t2));
    let mut da = a2 - a1;
    while da > PI {
        da -= TAU;
    }
    while da < -PI {
        da += TAU;
    }
    let (lo, hi, rev) = if da >= 0.0 {
        (a1, a1 + da, false)
    } else {
        (a1 + da, a1, true)
    };
    let mut arc = ProfileEdge::arc(&cf, rho, lo, hi);
    arc.reversed = rev;
    let edges = vec![
        arc,
        ProfileEdge::line(to3(t2), to3(d)),
        ProfileEdge::line(to3(d), to3(b)),
        ProfileEdge::line(to3(b), to3(a)),
        ProfileEdge::line(to3(a), to3(t1)),
    ];
    Ok((ProfileLoop::new(edges), size))
}

/// Build the blend tool for edge `e` of `body` (analysed on that body) and return it with its operation.
fn build_tool(
    rep: &mut BRep,
    body: BodyId,
    e: EdgeId,
    style: Style<'_>,
) -> Result<(BodyId, Op), BlendError> {
    let sec = analyse(rep, body, e)?;
    if let Style::Law(law) = style {
        return build_var_tool(rep, body, e, &sec, law);
    }
    let (loop_, size) = section_loop(&sec, e, style)?;
    let m = (0.3 * size).max(1e-3 * size);
    let plane_dir = sec.x.cross(&sec.y);
    let frame = Frame3 {
        origin: sec.p,
        x: sec.x,
        y: sec.y,
        z: plane_dir,
    };

    // ---- sweep
    let tool = match &sec.sweep {
        Sweep::Line { start, dir, length } => {
            // concave blends *add* the tool, so it must not extend past the edge ends; convex ones subtract
            // it, where extending clear of the end faces avoids coincident faces. A chain junction is not an
            // end face: the tool stops exactly there.
            let ext = if sec.convex { (0.5 * size).max(m) } else { 0.0 };
            let ext0 = if sec.chain_end[0] { 0.0 } else { ext };
            let ext1 = if sec.chain_end[1] { 0.0 } else { ext };
            // translate the profile (built at the edge midpoint) to the start of the extended edge
            let shift = (*start - *dir * ext0) - project_to_line(&sec.p, start, dir);
            let moved = shift_loop(&loop_, shift);
            let prof = Profile::new(
                Frame3 {
                    origin: *start - *dir * ext0,
                    ..frame
                },
                moved,
            );
            extrude(
                rep,
                &prof,
                *dir * (*length + ext0 + ext1),
                &ExtrudeOptions::default(),
            )
            .map_err(|er| err(e, BlendErrorKind::Model(er)))?
        }
        Sweep::Rim {
            axis_origin,
            axis,
            span,
        } => {
            // profile plane = meridian plane through the section point
            let o = *axis_origin + *axis * (sec.p - *axis_origin).dot(axis);
            let f = Frame3 {
                origin: o,
                x: sec.x,
                y: *axis,
                z: sec.x.cross(axis),
            };
            // the polygon was built with y = axis already
            match span {
                None => {
                    let prof = Profile::new(f, loop_);
                    revolve(rep, &prof, o, *axis, TAU)
                        .map_err(|er| err(e, BlendErrorKind::Model(er)))?
                }
                Some((t0, t1)) => {
                    // the section was taken at the middle of the arc: rotate it back to the start of the sweep,
                    // extended past ends that are closed by a plane (convex tools only)
                    let radius = (sec.p - o).norm().max(1e-300);
                    let ext = if sec.convex {
                        ((0.5 * size).max(m) / radius).min(0.5)
                    } else {
                        0.0
                    };
                    let a0 = if sec.chain_end[0] { 0.0 } else { ext };
                    let a1 = if sec.chain_end[1] { 0.0 } else { ext };
                    let back = -(0.5 * (t1 - t0)) - a0;
                    let rot = kw_math::Transform3::translation(-o.coords)
                        .then(&kw_math::Transform3::rotation(*axis, back))
                        .then(&kw_math::Transform3::translation(o.coords));
                    let x = rot.apply_vector(&sec.x);
                    let prof = Profile::new(
                        Frame3 {
                            origin: o,
                            x,
                            y: *axis,
                            z: x.cross(axis),
                        },
                        transform_loop(&loop_, &rot),
                    );
                    revolve(rep, &prof, o, *axis, (t1 - t0) + a0 + a1)
                        .map_err(|er| err(e, BlendErrorKind::Model(er)))?
                }
            }
        }
    };
    let tool_body = trim_to_end_planes(rep, body, e, &sec, tool.body)?;
    Ok((tool_body, if sec.convex { Op::Subtract } else { Op::Union }))
}

/// Trim a tool swept along a straight edge against the (slanted) planes that close the edge's ends.
fn trim_to_end_planes(
    rep: &mut BRep,
    body: BodyId,
    e: EdgeId,
    sec: &Section,
    mut tool_body: BodyId,
) -> Result<BodyId, BlendError> {
    if let (Sweep::Line { .. }, true) = (&sec.sweep, sec.end_planes.iter().any(|p| p.is_some())) {
        let big = 8.0 * (rep.body_bbox(tool_body).diagonal() + rep.body_bbox(body).diagonal());
        for (p3, n3) in sec.end_planes.iter().flatten() {
            let helper = if n3.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let hx = helper.cross(n3).normalize();
            let hy = n3.cross(&hx);
            let frame = Frame3 {
                origin: *p3 - *n3 * (big / 2.0),
                x: hx,
                y: hy,
                z: *n3,
            };
            let half = kw_brep::make_box(rep, &frame, big, big, big).body;
            let trimmed = boolean(rep, tool_body, half, Op::Intersect, &BoolOptions::default())
                .map_err(|be| err(e, BlendErrorKind::Boolean(be)))?;
            tool_body = trimmed.body;
        }
    }
    Ok(tool_body)
}

/// Tool of a variable-radius fillet. The spandrel section (radius from the law) is built at stations along
/// the edge and the stations are skinned: a straight edge between two planes is lofted along its length
/// (two stations are exact for a linear law), a closed circular edge between coaxial faces is a ring of
/// four lofted segments around the axis whose coincident end sections are united.
fn build_var_tool(
    rep: &mut BRep,
    body: BodyId,
    e: EdgeId,
    sec: &Section,
    law: &RadiusLaw,
) -> Result<(BodyId, Op), BlendError> {
    law.validate()
        .map_err(|m| err(e, BlendErrorKind::Size(m)))?;
    let (rmin, rmax) = law.range();
    if rmin <= 0.0 || rmin.is_nan() {
        return Err(err(e, BlendErrorKind::Size("radii must be positive")));
    }
    let op = if sec.convex { Op::Subtract } else { Op::Union };
    if sec.chain_end.iter().any(|c| *c) {
        return Err(err(
            e,
            BlendErrorKind::Unsupported(
                "a radius law is not applied to an edge in a tangent chain",
            ),
        ));
    }
    match &sec.sweep {
        Sweep::Line { start, dir, length } => {
            let (start, dir, length) = (*start, *dir, *length);
            // concave blends add the tool and must not run past the edge ends; convex ones extend clear of
            // them, but never so far that the radius law would collapse
            let mut ext_t = if sec.convex { 0.5 * rmax / length } else { 0.0 };
            while ext_t > 0.0
                && (law.at(-ext_t) < 0.3 * law.at(0.0) || law.at(1.0 + ext_t) < 0.3 * law.at(1.0))
            {
                ext_t *= 0.5;
            }
            let ts: Vec<f64> = if law.is_linear().is_some() {
                vec![-ext_t, 1.0 + ext_t]
            } else {
                let n = 81usize;
                (0..n)
                    .map(|i| -ext_t + (1.0 + 2.0 * ext_t) * i as f64 / (n - 1) as f64)
                    .collect()
            };
            let mid = project_to_line(&sec.p, &start, &dir);
            let mut profiles = Vec::new();
            for t in ts {
                let s_len = t * length;
                let (lp, _) = section_loop(sec, e, Style::Fillet(law.at(t)))?;
                let origin = start + dir * s_len;
                profiles.push(Profile::new(
                    Frame3 {
                        origin,
                        x: sec.x,
                        y: sec.y,
                        z: sec.x.cross(&sec.y),
                    },
                    shift_loop(&lp, origin - mid),
                ));
            }
            let tool =
                kw_model::loft(rep, &profiles).map_err(|er| err(e, BlendErrorKind::Model(er)))?;
            let tool_body = trim_to_end_planes(rep, body, e, sec, tool.body)?;
            Ok((tool_body, op))
        }
        Sweep::Rim {
            axis_origin,
            axis,
            span,
        } => {
            if span.is_some() {
                return Err(err(
                    e,
                    BlendErrorKind::Unsupported("a radius law needs a full circular edge"),
                ));
            }
            if (law.at(0.0) - law.at(1.0)).abs() > 1e-9 * rmax {
                return Err(err(
                    e,
                    BlendErrorKind::Size("a radius law on a closed edge must have r(0) = r(1)"),
                ));
            }
            let (axis_origin, axis) = (*axis_origin, *axis);
            // the edge's parameter origin and the angle at which the base section was built
            let edge = &rep.edges[e];
            let curve = rep.curves[edge.curve.expect("edge curve")].clone();
            let CurveGeom::Circle(ci) = &curve else {
                return Err(err(
                    e,
                    BlendErrorKind::Unsupported("variable radius needs a circular rim edge"),
                ));
            };
            let t_start = edge.param_range.0;
            let t_mid = 0.5 * (edge.param_range.0 + edge.param_range.1);
            let _ = ci;
            let o = axis_origin + axis * (sec.p - axis_origin).dot(&axis);
            // stations around the turn; the periodic skin keeps the contact curves on the coaxial faces to
            // well below the tolerance of the tangent-contact detection
            let stations = 256usize;
            let mut profiles = Vec::with_capacity(stations);
            for k in 0..stations {
                let t = k as f64 / stations as f64;
                let (lp, _) = section_loop(sec, e, Style::Fillet(law.at(t)))?;
                let rot = kw_math::Transform3::translation(-o.coords)
                    .then(&kw_math::Transform3::rotation(
                        axis,
                        t_start + t * TAU - t_mid,
                    ))
                    .then(&kw_math::Transform3::translation(o.coords));
                let lp = transform_loop(&lp, &rot);
                let x = rot.apply_vector(&sec.x);
                profiles.push(Profile::new(
                    Frame3 {
                        origin: o,
                        x,
                        y: axis,
                        z: x.cross(&axis),
                    },
                    lp,
                ));
            }
            let ring = kw_model::loft_ring(rep, &profiles)
                .map_err(|er| err(e, BlendErrorKind::Model(er)))?
                .body;
            Ok((ring, op))
        }
    }
}

fn transform_loop(l: &ProfileLoop, t: &kw_math::Transform3) -> ProfileLoop {
    use kw_nurbs::fit::transform_curve;
    ProfileLoop::new(
        l.edges
            .iter()
            .map(|e| {
                let mut ne = ProfileEdge::new(transform_curve(&e.curve, t), e.range);
                ne.reversed = e.reversed;
                ne
            })
            .collect(),
    )
}

fn project_to_line(p: &Point3<f64>, a: &Point3<f64>, d: &Vector3<f64>) -> Point3<f64> {
    a + d * (p - a).dot(d)
}

fn shift_loop(l: &ProfileLoop, shift: Vector3<f64>) -> ProfileLoop {
    use kw_nurbs::fit::transform_curve;
    let t = kw_math::Transform3::translation(shift);
    ProfileLoop::new(
        l.edges
            .iter()
            .map(|e| {
                let mut ne = ProfileEdge::new(transform_curve(&e.curve, &t), e.range);
                ne.reversed = e.reversed;
                ne
            })
            .collect(),
    )
}

/// Unite the tools of tangent-connected edges (in chain order, so each touches the ones before it).
fn unite_chains(
    rep: &mut BRep,
    body: BodyId,
    tools: Vec<(EdgeId, BodyId, Op, kw_math::Aabb)>,
    tool_face_edge: &mut std::collections::HashMap<FaceId, EntityRef>,
) -> Result<Vec<(EdgeId, BodyId, Op, kw_math::Aabb)>, BlendError> {
    let index_of = |e: EdgeId| tools.iter().position(|t| t.0 == e);
    let mut done = vec![false; tools.len()];
    let mut out: Vec<(EdgeId, BodyId, Op, kw_math::Aabb)> = Vec::new();
    for start in 0..tools.len() {
        if done[start] {
            continue;
        }
        // breadth-first along the chain
        let mut order = vec![start];
        done[start] = true;
        let mut k = 0;
        while k < order.len() {
            let e = tools[order[k]].0;
            k += 1;
            let Some(ed) = rep.edges.get(e) else { continue };
            for v in [ed.start, ed.end] {
                if let Some(n) = chain_neighbor(rep, body, e, v) {
                    if let Some(i) = index_of(n) {
                        if !done[i] {
                            done[i] = true;
                            order.push(i);
                        }
                    }
                }
            }
        }
        if order.len() == 1 {
            out.push(tools[start]);
            continue;
        }
        let (e0, first, op, mut bb) = tools[order[0]];
        for &i in &order[1..] {
            if tools[i].2 != op {
                return Err(err(
                    tools[i].0,
                    BlendErrorKind::Unsupported(
                        "a tangent chain must be convex or concave all along",
                    ),
                ));
            }
            bb.min = bb.min.inf(&tools[i].3.min);
            bb.max = bb.max.sup(&tools[i].3.max);
        }
        // The tools end in coincident caps: gluing them along those is exact. (A Boolean union of the same
        // tools would also work but leaves numerical slivers around the tangent junctions in some cases;
        // it remains the fallback.)
        let bodies: Vec<BodyId> = order.iter().map(|&i| tools[i].1).collect();
        let mut acc = first;
        let glued_result = crate::glue::glue_tools(rep, &bodies);
        if std::env::var("KW_BLEND_DEBUG").is_ok() {
            eprintln!("GLUE {} tools -> {}", bodies.len(), glued_result.is_some());
        }
        if let Some((glued, origin)) = glued_result {
            for (nf, old) in origin {
                if let Some(l) = tool_face_edge.get(&old).cloned() {
                    tool_face_edge.insert(nf, l);
                }
            }
            acc = glued;
        } else {
            for &i in &order[1..] {
                let (ei, ti, _, _) = tools[i];
                let res = boolean(rep, acc, ti, Op::Union, &BoolOptions::default())
                    .map_err(|be| err(ei, BlendErrorKind::Boolean(be)))?;
                // the faces of the united tool keep the label of the edge they were generated for
                for (k, o) in &res.history.origins {
                    let EntityRef::Face(nf) = k else { continue };
                    let src = match o {
                        Origin::Preserved(EntityRef::Face(f))
                        | Origin::Modified(EntityRef::Face(f))
                        | Origin::SplitFrom {
                            parent: EntityRef::Face(f),
                            ..
                        } => tool_face_edge.get(f).cloned(),
                        _ => None,
                    };
                    if let Some(l) = src {
                        tool_face_edge.insert(*nf, l);
                    }
                }
                acc = res.body;
            }
        }
        out.push((e0, acc, op, bb));
    }
    Ok(out)
}

fn run(
    rep: &mut BRep,
    body: BodyId,
    edges: &[EdgeId],
    style: Style<'_>,
) -> Result<kw_brep::BuildResult, BlendError> {
    let role = match style {
        Style::Fillet(_) | Style::Law(_) => "fillet",
        Style::Chamfer(..) => "chamfer",
    };
    // a blend follows a tangent chain to its end: the neighbour of a blended edge across a tangent
    // junction is blended too. A chamfer keeps its first distance on the same side of the chain.
    let propagate = !matches!(style, Style::Law(_));
    let oriented: Vec<(EdgeId, bool)> = if propagate {
        expand_chains_oriented(rep, body, edges)
    } else {
        edges.iter().map(|e| (*e, false)).collect()
    };
    let expanded: Vec<EdgeId> = oriented.iter().map(|(e, _)| *e).collect();
    let edges = &expanded[..];
    let style_of = |e: EdgeId| -> Style<'_> {
        match style {
            Style::Chamfer(d1, d2) if oriented.iter().any(|(x, f)| *x == e && *f) => {
                Style::Chamfer(d2, d1)
            }
            other => other,
        }
    };
    for &e in edges {
        let Some(ed) = rep.edges.get(e) else { continue };
        for v in [ed.start, ed.end] {
            if chain_neighbor(rep, body, e, v).is_some_and(|n| !edges.contains(&n)) {
                return Err(err(
                    e,
                    BlendErrorKind::Unsupported(
                        "the tangent neighbour of the edge is not blended (it cannot be left out of a chain)",
                    ),
                ));
            }
        }
    }
    // every tool is derived from the *original* body, so adjacent blends are independent of order and
    // meet in a mitre
    let mut tools: Vec<(EdgeId, BodyId, Op, kw_math::Aabb)> = Vec::new();
    let mut tool_face_edge: std::collections::HashMap<FaceId, EntityRef> =
        std::collections::HashMap::new();
    for &e in edges {
        let (tool, op) = build_tool(rep, body, e, style_of(e))?;
        for f in rep.body_faces(tool) {
            tool_face_edge.insert(f, EntityRef::Edge(e));
        }
        let mut bb = rep.body_bbox(tool);
        let pad = 1e-6 * (1.0 + bb.diagonal());
        bb.min -= Vector3::repeat(pad);
        bb.max += Vector3::repeat(pad);
        tools.push((e, tool, op, bb));
    }
    // The tools of one tangent chain are united into a single tool first: their end caps coincide, which is
    // exact and clean between two tools but would leave coincident walls on the body if applied one by one.
    let tools = unite_chains(rep, body, tools, &mut tool_face_edge)?;
    // Tools whose boxes are disjoint (and of the same operation) are applied together in one Boolean:
    // the cost of a Boolean is dominated by the faces of the body, which are processed once per batch.
    let mut batches: Vec<(Op, Vec<usize>)> = Vec::new();
    for (i, t) in tools.iter().enumerate() {
        let slot = batches.iter_mut().find(|(op, members)| {
            *op == t.2 && members.iter().all(|m| !tools[*m].3.intersects(&t.3))
        });
        match slot {
            Some((_, members)) => members.push(i),
            None => batches.push((t.2, vec![i])),
        }
    }
    let mut cur = body;
    let mut history = ShapeHistory::default();
    for (op, members) in batches {
        let first = tools[members[0]].1;
        let rest: Vec<BodyId> = members[1..].iter().map(|m| tools[*m].1).collect();
        rep.merge_bodies(first, &rest);
        let res = match boolean(rep, cur, first, op, &BoolOptions::default()) {
            Ok(r) => r,
            Err(_) if members.len() > 1 => {
                // a failed batch is retried tool by tool (tools were merged: rebuild them) so the error
                // names the edge that really fails
                let mut cur2 = cur;
                let mut hist = ShapeHistory::default();
                for m in &members {
                    let e = tools[*m].0;
                    let (t2, op2) = build_tool(rep, body, e, style_of(e))?;
                    for f in rep.body_faces(t2) {
                        tool_face_edge.insert(f, EntityRef::Edge(e));
                    }
                    let r = boolean(rep, cur2, t2, op2, &BoolOptions::default())
                        .map_err(|be| err(e, BlendErrorKind::Boolean(be)))?;
                    cur2 = r.body;
                    hist = ShapeHistory::compose(
                        &hist,
                        &relabel_tool_faces(r.history, &tool_face_edge, role),
                    );
                }
                kw_boolean::BooleanResult {
                    body: cur2,
                    history: hist,
                }
            }
            Err(be) => return Err(err(tools[members[0]].0, BlendErrorKind::Boolean(be))),
        };
        cur = res.body;
        history = ShapeHistory::compose(
            &history,
            &relabel_tool_faces(res.history, &tool_face_edge, role),
        );
    }
    // vertex blends: the rolling-ball patch where three equal-radius fillets of three planes meet
    // (withheld from the default build until its patent search is recorded)
    #[cfg(feature = "vertex-blend")]
    if let Style::Fillet(r) = style {
        let mut verts: Vec<VertexId> = Vec::new();
        for &e in edges {
            for v in [rep.edges[e].start, rep.edges[e].end] {
                if !verts.contains(&v) {
                    verts.push(v);
                }
            }
        }
        let mut corner_tools: Vec<(VertexId, BodyId, Op, kw_math::Aabb, EdgeId)> = Vec::new();
        for v in verts {
            let sections = |e: EdgeId| analyse(rep, body, e).ok();
            let Some(corner) = analyse_corner(rep, body, v, edges, &sections, r)? else {
                continue;
            };
            let (tool, op) = build_corner_tool(rep, &corner, r)?;
            for f in rep.body_faces(tool) {
                tool_face_edge.insert(f, EntityRef::Vertex(v));
            }
            let mut bb = rep.body_bbox(tool);
            let pad = 1e-6 * (1.0 + bb.diagonal());
            bb.min -= Vector3::repeat(pad);
            bb.max += Vector3::repeat(pad);
            corner_tools.push((v, tool, op, bb, corner.edges[0]));
        }
        // corner tools with disjoint boxes (and the same operation) go through one Boolean
        let mut cbatches: Vec<(Op, Vec<usize>)> = Vec::new();
        for (i, t) in corner_tools.iter().enumerate() {
            let slot = cbatches.iter_mut().find(|(op, members)| {
                *op == t.2 && members.iter().all(|m| !corner_tools[*m].3.intersects(&t.3))
            });
            match slot {
                Some((_, members)) => members.push(i),
                None => cbatches.push((t.2, vec![i])),
            }
        }
        for (op, members) in cbatches {
            let first = corner_tools[members[0]].1;
            let rest: Vec<BodyId> = members[1..].iter().map(|m| corner_tools[*m].1).collect();
            rep.merge_bodies(first, &rest);
            let res = boolean(rep, cur, first, op, &BoolOptions::default())
                .map_err(|be| err(corner_tools[members[0]].4, BlendErrorKind::Boolean(be)))?;
            cur = res.body;
            history = ShapeHistory::compose(
                &history,
                &relabel_tool_faces(res.history, &tool_face_edge, "vertex blend"),
            );
        }
    }
    Ok(kw_brep::BuildResult { body: cur, history })
}

/// Faces that come from a tool are the blend surfaces: generated from the edge being blended.
fn relabel_tool_faces(
    h: ShapeHistory,
    tool_face_edge: &std::collections::HashMap<FaceId, EntityRef>,
    role: &'static str,
) -> ShapeHistory {
    let mut out = ShapeHistory {
        deleted_inputs: h.deleted_inputs.clone(),
        ..Default::default()
    };
    for (k, o) in h.origins {
        let from_tool = match &o {
            Origin::Preserved(EntityRef::Face(f))
            | Origin::Modified(EntityRef::Face(f))
            | Origin::SplitFrom {
                parent: EntityRef::Face(f),
                ..
            } => tool_face_edge.get(f).cloned(),
            _ => None,
        };
        let o = match from_tool {
            Some(from) => Origin::Generated {
                from,
                role: role.into(),
            },
            None => o,
        };
        out.record(k, o);
    }
    // tool faces are not inputs of the feature
    out.deleted_inputs
        .retain(|d| !matches!(d, EntityRef::Face(f) if tool_face_edge.contains_key(f)));
    out
}

/// Round each edge of `body` with a constant `radius`.
///
/// **Tangent chains.** A blend follows the edges that continue each other tangentially (the top outline
/// of a rounded-rectangle block: straight edges and corner arcs): naming one edge blends the whole
/// chain, as in other CAD systems ([`tangent_chain`] returns it). The blends of a chain meet in exact
/// caps at the junctions, so the result is smooth there; chain ends are handled like any edge end (an end
/// face perpendicular to the chain, or the mitre with an adjacent selected edge). Straight edges and
/// partial or full circular arcs are supported; a chain may mix convex corners and concave ones.
pub fn fillet(
    rep: &mut BRep,
    body: BodyId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<kw_brep::BuildResult, BlendError> {
    run(rep, body, edges, Style::Fillet(radius))
}

/// Round one straight edge between two planar faces with a radius that varies linearly from `r_start`
/// at the edge's start vertex to `r_end` at its end vertex.
pub fn fillet_variable(
    rep: &mut BRep,
    body: BodyId,
    edge: EdgeId,
    r_start: f64,
    r_end: f64,
) -> Result<kw_brep::BuildResult, BlendError> {
    let law = RadiusLaw::Linear {
        start: r_start,
        end: r_end,
    };
    run(rep, body, &[edge], Style::Law(&law))
}

/// Round one edge with a radius that follows `law` along it. Supported: straight edges between two
/// planes, and closed circular rim edges between faces coaxial with them (plane, cylinder, cone), for
/// which the law must be periodic (`r(0) = r(1)`).
pub fn fillet_law(
    rep: &mut BRep,
    body: BodyId,
    edge: EdgeId,
    law: &RadiusLaw,
) -> Result<kw_brep::BuildResult, BlendError> {
    run(rep, body, &[edge], Style::Law(law))
}

/// Chamfer each edge with distances `d1` (along the first adjacent face) and `d2`.
pub fn chamfer(
    rep: &mut BRep,
    body: BodyId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<kw_brep::BuildResult, BlendError> {
    run(rep, body, edges, Style::Chamfer(d1, d2))
}
