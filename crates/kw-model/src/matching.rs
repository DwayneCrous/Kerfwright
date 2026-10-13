//! Boundary matching: reshape a NURBS patch near one of its open edges so that it meets a
//! neighbouring surface tangentially (G1) or with the same normal curvature across the edge (G2).
//!
//! Only the control rows next to the edge move. The second row sets the cross-boundary derivative
//! (G1); the third sets the second derivative (G2). The edge itself, the far edge and the two side
//! edges keep their curves, and near the ends of the edge the change fades out. The new rows are
//! found by collocation at the Greville points of the edge's knots, so the match is as close as the
//! patch's own knot spacing allows; the residual is reported.
//! (Piegl & Tiller, *The NURBS Book*, eqs. 3.7-3.8 for the end derivatives of a B-spline.)

use crate::extend::iso_edge;
use crate::profile::ModelError;
use crate::sheetops::greville;
use kw_brep::{BRep, BodyId, BodyKind, BuildResult, EdgeId, EntityRef, Origin, ShapeHistory};
use kw_math::{Point3, Vector3};
pub use kw_nurbs::normal_curvature;
use kw_nurbs::{NurbsCurve, NurbsSurface, SurfaceGeom};
use std::sync::Arc;

fn bad(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// What the neighbour looks like at a point of the shared edge.
#[derive(Debug, Clone, Copy)]
pub struct NeighbourSample {
    /// Unit normal.
    pub normal: Vector3<f64>,
    /// Unit vector in the tangent plane, across the edge, pointing out of the neighbour.
    pub out: Vector3<f64>,
    /// Normal curvature of the neighbour along `out`, with respect to `normal`.
    pub curvature: f64,
}

/// The neighbour at a point of the edge (given with the unit tangent of the edge there).
pub type NeighbourFn =
    Arc<dyn Fn(&Point3<f64>, &Vector3<f64>) -> Option<NeighbourSample> + Send + Sync>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Continuity {
    G1,
    G2,
}

/// How well the patch now meets the neighbour, measured along the middle of the edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MatchResidual {
    /// Largest angle between the normals, radians.
    pub angle: f64,
    /// Largest difference of the normal curvatures across the edge (1 / length).
    pub curvature: f64,
}

/// 0 at the ends of the edge, 1 over its middle, smooth between.
fn window(s: f64) -> f64 {
    let x = (s.min(1.0 - s) / 0.12).clamp(0.0, 1.0);
    x * x * x * (x * (6.0 * x - 15.0) + 10.0)
}

/// The surface with the boundary to match at `v = v0`, running along `u`.
fn canonical(s: &NurbsSurface, across_u: bool, at_end: bool) -> NurbsSurface {
    let c = if across_u { s.transposed() } else { s.clone() };
    if at_end {
        c.reversed_v()
    } else {
        c
    }
}

fn restore(c: &NurbsSurface, across_u: bool, at_end: bool) -> NurbsSurface {
    let r = if at_end { c.reversed_v() } else { c.clone() };
    if across_u {
        r.transposed()
    } else {
        r
    }
}

fn rebuild(c: &NurbsSurface, pts: Vec<Point3<f64>>) -> Result<NurbsSurface, ModelError> {
    NurbsSurface::new(
        c.knots_u().clone(),
        c.knots_v().clone(),
        pts,
        c.weights().to_vec(),
    )
    .map_err(|e| bad(format!("{e:?}")))
}

/// Reshape the NURBS face behind the open `edge` of the surface `body`.
pub fn match_boundary(
    rep: &mut BRep,
    body: BodyId,
    edge: EdgeId,
    neighbour: &NeighbourFn,
    continuity: Continuity,
) -> Result<(BuildResult, MatchResidual), ModelError> {
    if rep.bodies[body].kind != BodyKind::Sheet {
        return Err(bad("only a surface can be matched to another"));
    }
    if !rep.free_edges(body).contains(&edge) {
        return Err(bad("only an open edge of a surface can be matched"));
    }
    let iso = iso_edge(rep, edge)?;
    let face = iso.face;
    let SurfaceGeom::Nurbs(surf) = rep.surfaces[rep.faces[face].surface].clone() else {
        return Err(bad(
            "only a NURBS patch can be reshaped to match a neighbour (a plane, cylinder or cone has no control points)",
        ));
    };
    if surf.weights().iter().any(|w| (w - 1.0).abs() > 1e-9) {
        return Err(bad(
            "a rational surface (an exact circle or cone, for one) cannot be matched: its control points are weighted",
        ));
    }
    let ((u0, u1), (v0, v1)) = surf.domain();
    let (a_dom, b_dom) = if iso.across_u {
        ((u0, u1), (v0, v1))
    } else {
        ((v0, v1), (u0, u1))
    };
    let tol = 1e-9 * (1.0 + a_dom.1.abs() + b_dom.1.abs());
    // the edge must be a whole boundary of the patch
    let at_end = if (iso.c - a_dom.0).abs() < tol {
        false
    } else if (iso.c - a_dom.1).abs() < tol {
        true
    } else {
        return Err(bad("the edge is not on the boundary of the patch"));
    };
    if (iso.w0 - b_dom.0).abs() > tol || (iso.w1 - b_dom.1).abs() > tol {
        return Err(bad("only a whole boundary of a patch can be matched"));
    }
    let mut c = canonical(&surf, iso.across_u, at_end);
    // finer knots along the edge, so that the change can fade out near its ends without ringing
    for _ in 0..2 {
        let distinct = c.knots_u().distinct();
        for w in distinct.windows(2) {
            c = c.insert_knot_u(0.5 * (w[0].0 + w[1].0), 1);
        }
    }
    let (nu, nv) = (c.num_u(), c.num_v());
    let p = c.knots_v().degree();
    if p < 2 || nv < 5 {
        return Err(bad(
            "the patch needs at least five rows of control points and degree two across the edge to be matched",
        ));
    }
    let kv = c.knots_v().as_slice();
    let (d1, d2, d12) = (kv[p + 1] - kv[1], kv[p + 2] - kv[2], kv[p + 1] - kv[2]);
    let a1 = p as f64 / d1;
    let c2 = (p as f64 - 1.0) / d12;
    let ku = c.knots_u().clone();
    let g = greville(&ku);
    let ((cu0, cu1), (cv0, _)) = c.domain();
    let at = |i: usize, j: usize| i * nv + j;
    let ask = |p3: &Point3<f64>, su: &Vector3<f64>| -> Result<NeighbourSample, ModelError> {
        let b = su.try_normalize(1e-300).unwrap_or_else(Vector3::x);
        neighbour(p3, &b).ok_or_else(|| bad("the neighbour could not be measured along the edge"))
    };
    let collocate = |values: &[Point3<f64>]| -> Result<Vec<Point3<f64>>, ModelError> {
        NurbsCurve::collocate_values(ku.clone(), values)
            .map(|c| c.points().to_vec())
            .map_err(|e| bad(format!("{e:?}")))
    };
    let mut pts: Vec<Point3<f64>> = c.points().to_vec();

    // ---- G1: the cross-boundary derivative leaves along the tangent plane of the neighbour,
    // as fast as it did
    let mut targets: Vec<Point3<f64>> = Vec::with_capacity(g.len());
    for &gi in &g {
        let d = c.derivatives(gi, cv0, 1);
        let (p3, su, sv) = (Point3::from(d[0][0]), d[1][0], d[0][1]);
        let s = ask(&p3, &su)?;
        let w = window((gi - cu0) / (cu1 - cu0));
        let want = s.out * sv.norm();
        targets.push(Point3::from(sv + (want - sv) * w));
    }
    let w1 = collocate(&targets)?;
    for i in 0..nu {
        let row0 = pts[at(i, 0)];
        pts[at(i, 1)] = row0 + w1[i].coords / a1;
    }
    let after_g1 = rebuild(&c, pts.clone())?;

    // ---- G2: the second derivative across the edge has the neighbour's normal curvature
    let after = if continuity == Continuity::G2 {
        let mut targets: Vec<Point3<f64>> = Vec::with_capacity(g.len());
        for &gi in &g {
            let d = after_g1.derivatives(gi, cv0, 2);
            let (p3, su, sv, svv) = (Point3::from(d[0][0]), d[1][0], d[0][1], d[0][2]);
            let s = ask(&p3, &su)?;
            let w = window((gi - cu0) / (cu1 - cu0));
            let want = svv + s.normal * (s.curvature * sv.norm_squared() - svv.dot(&s.normal));
            targets.push(Point3::from(svv + (want - svv) * w));
        }
        let v2 = collocate(&targets)?;
        for i in 0..nu {
            let (p0, p1) = (pts[at(i, 0)], pts[at(i, 1)]);
            let d1v = p1 - p0;
            pts[at(i, 2)] = p1 + (v2[i].coords / c2 + d1v * (p as f64 / d1)) * (d2 / p as f64);
        }
        rebuild(&c, pts)?
    } else {
        after_g1
    };

    // ---- how well it worked
    let mut residual = MatchResidual {
        angle: 0.0,
        curvature: 0.0,
    };
    for k in 0..=12 {
        let u = cu0 + (cu1 - cu0) * (0.2 + 0.6 * k as f64 / 12.0);
        let d = after.derivatives(u, cv0, 2);
        let (p3, su, sv, svv) = (Point3::from(d[0][0]), d[1][0], d[0][1], d[0][2]);
        let s = ask(&p3, &su)?;
        let np = su.cross(&sv).try_normalize(1e-300).unwrap_or(s.normal);
        residual.angle = residual
            .angle
            .max(np.dot(&s.normal).abs().clamp(0.0, 1.0).acos());
        if continuity == Continuity::G2 {
            let kp = svv.dot(&s.normal) / sv.norm_squared().max(1e-300);
            residual.curvature = residual.curvature.max((kp - s.curvature).abs());
        }
    }
    let result = restore(&after, iso.across_u, at_end);
    let sid = rep.add_surface(SurfaceGeom::Nurbs(result));
    rep.faces[face].surface = sid;
    let mut history = ShapeHistory::default();
    for f in rep.body_faces(body) {
        history.record(
            EntityRef::Face(f),
            if f == face {
                Origin::Modified(EntityRef::Face(f))
            } else {
                Origin::Preserved(EntityRef::Face(f))
            },
        );
    }
    Ok((BuildResult { body, history }, residual))
}
