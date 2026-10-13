//! Independent verification of a Boolean result by point classification.
//!
//! The Boolean builds its answer from an arrangement of intersection curves; if that arrangement is
//! incomplete the result can be a perfectly valid solid that has lost a piece of volume. Ray casting does
//! not depend on the arrangement, so a result is checked by classifying points against both operands and
//! against the result: the result must contain exactly the points the operation says it contains.

use crate::model::Work;
use crate::Op;
use kw_brep::{BRep, BodyId, PointClass, SolidIndex};
use kw_math::{Point3, Vector3};
use kw_nurbs::{Curve3, Surface};

/// Points just beside the intersection curves: on both sides of both faces at the middle of every cut.
pub(crate) fn cut_probes(w: &Work, max_cuts: usize) -> Vec<Point3<f64>> {
    let mut out = Vec::new();
    let step = w.cuts.len().div_ceil(max_cuts.max(1)).max(1);
    let diag = w.faces.iter().fold(0.0f64, |m, f| m.max(f.bbox.diagonal()));
    for c in w.cuts.iter().step_by(step) {
        let tm = 0.5 * (c.range.0 + c.range.1);
        let p = c.curve.point_at(tm);
        let normal = |face: usize, pc: &kw_nurbs::CurveGeom| -> Option<Vector3<f64>> {
            let f = &w.faces[face];
            let q = pc.point_at(tm);
            let n = f.surface.normal_at(q.x, q.y) * f.sgn;
            (n.norm() > 0.5).then_some(n)
        };
        let (Some(na), Some(nb)) = (normal(c.fa, &c.pcurve_a), normal(c.fb, &c.pcurve_b)) else {
            continue;
        };
        for scale in [1e-3, 1e-5] {
            let d = scale * diag.max(1e-3);
            for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                out.push(p + na * (sa * d) + nb * (sb * d));
            }
        }
    }
    out
}

/// Does `result` (`None`: no material) agree with the operation at the probe points and at random points
/// of the operands' common box? Points near any boundary are skipped; a result that cannot be indexed
/// does not verify.
pub(crate) fn sample_verify(
    rep: &BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    result: Option<BodyId>,
    probes: &[Point3<f64>],
) -> bool {
    // a point query against a free-form surface is two orders of magnitude dearer than against an analytic one
    let free_form = [a, b].iter().any(|body| {
        rep.body_faces(*body).iter().any(|f| {
            matches!(
                rep.surfaces[rep.faces[*f].surface],
                kw_nurbs::SurfaceGeom::Nurbs(_)
                    | kw_nurbs::SurfaceGeom::Extrusion { .. }
                    | kw_nurbs::SurfaceGeom::Revolution { .. }
                    | kw_nurbs::SurfaceGeom::Procedural(_)
            )
        })
    });
    sample_verify_n(
        rep,
        a,
        b,
        op,
        result,
        probes,
        if free_form { 160 } else { 600 },
    )
}

/// [`sample_verify`] with `n` random points: half in the box where the operation's material can be (the
/// operands' common box for an intersection, the first operand's box for a difference), half in the
/// union box.
pub(crate) fn sample_verify_n(
    rep: &BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    result: Option<BodyId>,
    probes: &[Point3<f64>],
    n: usize,
) -> bool {
    let (Some(ia), Some(ib)) = (SolidIndex::new(rep, a), SolidIndex::new(rep, b)) else {
        return false;
    };
    let ir = match result {
        Some(r) => match SolidIndex::new(rep, r) {
            Some(ix) => Some(ix),
            None => return false,
        },
        None => None,
    };
    let (box_a, box_b) = (rep.body_bbox(a), rep.body_bbox(b));
    let bb = box_a.union(&box_b);
    let tol = 1e-7 * (1.0 + bb.diagonal());
    // where the operation's material can be
    let focus = match op {
        Op::Union => bb,
        Op::Subtract => box_a,
        Op::Intersect => kw_math::Aabb {
            min: Point3::new(
                box_a.min.x.max(box_b.min.x),
                box_a.min.y.max(box_b.min.y),
                box_a.min.z.max(box_b.min.z),
            ),
            max: Point3::new(
                box_a.max.x.min(box_b.max.x),
                box_a.max.y.min(box_b.max.y),
                box_a.max.z.min(box_b.max.z),
            ),
        },
    };
    let focus_ok =
        focus.min.x <= focus.max.x && focus.min.y <= focus.max.y && focus.min.z <= focus.max.z;
    // deterministic pseudo-random points
    let mut state = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        ((state >> 11) as f64) / ((1u64 << 53) as f64)
    };
    let mut points: Vec<Point3<f64>> = probes.to_vec();
    for k in 0..n {
        let b = if k % 2 == 0 && focus_ok { &focus } else { &bb };
        points.push(Point3::new(
            b.min.x + (b.max.x - b.min.x) * next(),
            b.min.y + (b.max.y - b.min.y) * next(),
            b.min.z + (b.max.z - b.min.z) * next(),
        ));
    }
    for p in points {
        let (Ok(ca), Ok(cb)) = (ia.classify(&p, tol), ib.classify(&p, tol)) else {
            continue;
        };
        let cr = match &ir {
            Some(ix) => match ix.classify(&p, tol) {
                Ok(c) => c,
                Err(_) => continue,
            },
            None => PointClass::Out,
        };
        if [ca, cb, cr].contains(&PointClass::On) {
            continue;
        }
        let (in_a, in_b, in_r) = (
            ca == PointClass::In,
            cb == PointClass::In,
            cr == PointClass::In,
        );
        let want = match op {
            Op::Union => in_a || in_b,
            Op::Intersect => in_a && in_b,
            Op::Subtract => in_a && !in_b,
        };
        if want != in_r {
            return false;
        }
    }
    true
}
