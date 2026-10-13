//! How smoothly two faces meet along an edge: position (G0), tangent plane (G1), curvature (G2).
//!
//! Along the middle three fifths of the edge (the ends are corners, where the neighbouring edges
//! decide, and a match fades out there), at nine points, the two faces are asked where they are, which way
//! their normals point, and how much they curve across the edge. The gap, the angle between the
//! normals and the difference of the normal curvatures across the edge are the worst of those
//! samples. Two faces of a valid body look the same way, so tangent faces have equal normals.
//!
//! Classes: a gap larger than the position tolerance is [`ContClass::Broken`]; an angle larger than
//! the angle tolerance leaves the join at [`ContClass::G0`]; equal tangent planes but different
//! curvature across the edge is [`ContClass::G1`]; and curvature that agrees within the tolerance is
//! [`ContClass::G2`]. An edge with one face is [`ContClass::Open`].

use kw_brep::{BRep, BodyId, EdgeId, FaceId};
use kw_math::{Point3, Vector3};
use kw_nurbs::{normal_curvature, Curve3, Surface};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ContClass {
    /// One face only: nothing to compare with.
    Open,
    /// The faces do not even meet.
    Broken,
    /// They meet, at a crease.
    G0,
    /// Same tangent plane.
    G1,
    /// Same tangent plane and curvature across the edge.
    G2,
}

impl ContClass {
    pub fn name(self) -> &'static str {
        match self {
            ContClass::Open => "Open",
            ContClass::Broken => "Gap",
            ContClass::G0 => "G0 (crease)",
            ContClass::G1 => "G1 (tangent)",
            ContClass::G2 => "G2 (curvature)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerances {
    /// Largest gap between the faces (model units).
    pub position: f64,
    /// Largest angle between the normals (radians).
    pub angle: f64,
    /// Curvatures agree if they differ by less than this fraction of the larger one ...
    pub curvature: f64,
    /// ... or by less than this (1 / model units).
    pub curvature_floor: f64,
}

impl Tolerances {
    /// Tolerances for a model of the given size: a ten-thousandth of it, half a degree, a tenth of
    /// the curvature (or a hundredth of the size's reciprocal).
    pub fn for_size(size: f64) -> Tolerances {
        Tolerances {
            position: 1e-4 * (1.0 + size),
            angle: 0.5f64.to_radians(),
            curvature: 0.1,
            curvature_floor: 0.01 / (1.0 + size),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EdgeContinuity {
    pub edge: EdgeId,
    pub class: ContClass,
    /// Worst gap between the faces (0 for an open edge).
    pub gap: f64,
    /// Worst angle between the normals, radians.
    pub angle: f64,
    /// Worst difference of the normal curvatures across the edge; `None` where the faces are not
    /// tangent (it means nothing there) or the edge is open.
    pub curvature: Option<f64>,
    /// The edge as a polyline, for drawing.
    pub points: Vec<Point3<f64>>,
    pub faces: Vec<FaceId>,
}

struct Side {
    point: Point3<f64>,
    normal: Vector3<f64>,
    kappa: f64,
}

fn side(rep: &BRep, co: kw_brep::CoedgeId, t: f64, b: &Vector3<f64>) -> Option<Side> {
    let c = rep.coedges.get(co)?;
    let face = rep.faces.get(rep.loops.get(c.owner_loop)?.face)?;
    let surface = rep.surfaces.get(face.surface)?;
    let q = rep.curves.get(c.pcurve?)?.point_at(t);
    let sign = if face.sense == kw_brep::Sense::Forward {
        1.0
    } else {
        -1.0
    };
    let normal = surface.normal_at(q.x, q.y) * sign;
    let across = normal.cross(b).try_normalize(1e-12)?;
    Some(Side {
        point: surface.point_at(q.x, q.y),
        normal,
        kappa: normal_curvature(surface, (q.x, q.y), &across, sign),
    })
}

/// The continuity along one edge. `None` for a degenerate edge or one with more than two faces.
pub fn edge_continuity(rep: &BRep, edge: EdgeId, tol: &Tolerances) -> Option<EdgeContinuity> {
    let e = rep.edges.get(edge)?;
    let curve = rep.curves.get(e.curve?)?;
    let (t0, t1) = e.param_range;
    let points: Vec<Point3<f64>> = (0..=24)
        .map(|i| curve.point_at(t0 + (t1 - t0) * i as f64 / 24.0))
        .collect();
    let ring = rep.radial_ring(edge);
    let faces = rep.faces_around_edge(edge);
    match ring.len() {
        1 => Some(EdgeContinuity {
            edge,
            class: ContClass::Open,
            gap: 0.0,
            angle: 0.0,
            curvature: None,
            points,
            faces,
        }),
        2 => {
            let (mut gap, mut angle, mut dk) = (0.0f64, 0.0f64, 0.0f64);
            let mut kmax = 0.0f64;
            for i in 0..9 {
                let t = t0 + (t1 - t0) * (0.2 + 0.6 * i as f64 / 8.0);
                let b = curve.derivative(t).try_normalize(1e-12)?;
                let (s1, s2) = (side(rep, ring[0], t, &b)?, side(rep, ring[1], t, &b)?);
                gap = gap.max((s1.point - s2.point).norm());
                angle = angle.max(s1.normal.dot(&s2.normal).clamp(-1.0, 1.0).acos());
                dk = dk.max((s1.kappa - s2.kappa).abs());
                kmax = kmax.max(s1.kappa.abs()).max(s2.kappa.abs());
            }
            let tangent = angle <= tol.angle;
            let curvature = tangent.then_some(dk);
            let class = if gap > tol.position {
                ContClass::Broken
            } else if !tangent {
                ContClass::G0
            } else if dk <= tol.curvature * kmax + tol.curvature_floor {
                ContClass::G2
            } else {
                ContClass::G1
            };
            Some(EdgeContinuity {
                edge,
                class,
                gap,
                angle,
                curvature,
                points,
                faces,
            })
        }
        _ => None,
    }
}

/// The continuity of every edge of a body (edges that are a single point, or shared by three or
/// more faces, are left out).
pub fn body_continuity(rep: &BRep, body: BodyId, tol: &Tolerances) -> Vec<EdgeContinuity> {
    let mut seen: Vec<EdgeId> = Vec::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if seen.contains(&e) {
                    continue;
                }
                seen.push(e);
                if let Some(c) = edge_continuity(rep, e, tol) {
                    out.push(c);
                }
            }
        }
    }
    out
}
