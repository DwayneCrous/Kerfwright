//! The profile builder of before the planar arrangement: closed loops chained by shared end points. Kept only
//! to read region choices saved in that form (see `legacy_regions.rs`).

use crate::eval::EvalError;
use kw_math::Frame3;
use kw_model::{Profile, ProfileEdge, ProfileLoop};
use kw_nurbs::geom::Ellipse;
use kw_nurbs::{CurveGeom, NurbsCurve};
use kw_sketch::{EntityId, EntityKind, Sketch};
use std::collections::HashMap;

/// A profile together with, for every loop and edge, the label of the sketch entity behind it.
#[derive(Debug, Clone)]
pub struct LabeledProfile {
    pub profile: Profile,
    /// `labels[loop][edge]`; loop 0 is the outer loop, the rest are holes, in profile order.
    pub labels: Vec<Vec<u32>>,
}

fn sk_err<E: std::fmt::Display>(e: E) -> EvalError {
    EvalError::Sketch(e.to_string())
}

/// Turn the closed curves of a sketch into a planar profile: lines, arcs, circles, ellipses and splines
/// that are not construction geometry are chained into loops; the loop with the largest area is the outer one.
/// `entities[i]` is the entity with label `i`.
pub fn sketch_profile_labeled(
    sk: &Sketch,
    entities: &[EntityId],
    plane: Frame3,
) -> Result<LabeledProfile, EvalError> {
    use kw_sketch::geom::arc_params;
    let label_of: HashMap<EntityId, u32> = entities
        .iter()
        .enumerate()
        .map(|(i, e)| (*e, i as u32))
        .collect();
    let map = |p: (f64, f64)| plane.origin + plane.x * p.0 + plane.y * p.1;
    struct Seg {
        a: (f64, f64),
        b: (f64, f64),
        edge: ProfileEdge,
        label: u32,
    }
    let mut segs: Vec<Seg> = Vec::new();
    let mut circles: Vec<(ProfileLoop, u32)> = Vec::new();
    for (id, e) in &sk.entities {
        if e.construction {
            continue;
        }
        let label = label_of.get(&id).copied().unwrap_or(u32::MAX);
        match &e.kind {
            EntityKind::Line { p1, p2 } => {
                let (a, b) = (
                    sk.point(*p1).map_err(sk_err)?,
                    sk.point(*p2).map_err(sk_err)?,
                );
                segs.push(Seg {
                    a,
                    b,
                    edge: ProfileEdge::line(map(a), map(b)),
                    label,
                });
            }
            EntityKind::Arc { .. } => {
                let (c, r, a0, a1) = arc_params(sk, id).map_err(sk_err)?;
                let frame = Frame3 {
                    origin: map(c),
                    ..plane
                };
                let (pa, pb) = (
                    (c.0 + r * a0.cos(), c.1 + r * a0.sin()),
                    (c.0 + r * a1.cos(), c.1 + r * a1.sin()),
                );
                segs.push(Seg {
                    a: pa,
                    b: pb,
                    edge: ProfileEdge::arc(&frame, r, a0, a1),
                    label,
                });
            }
            EntityKind::Circle { center, radius } => {
                let c = sk.point(*center).map_err(sk_err)?;
                circles.push((
                    ProfileLoop::circle(
                        &Frame3 {
                            origin: map(c),
                            ..plane
                        },
                        sk.value(*radius),
                    ),
                    label,
                ));
            }
            EntityKind::Ellipse {
                center,
                major_end,
                minor_radius,
            } => {
                let (c, m) = (
                    sk.point(*center).map_err(sk_err)?,
                    sk.point(*major_end).map_err(sk_err)?,
                );
                let (ux, uy) = (m.0 - c.0, m.1 - c.1);
                let major = ux.hypot(uy);
                let minor = sk.value(*minor_radius).abs();
                if major < 1e-12 || minor < 1e-12 {
                    return Err(EvalError::Sketch("an ellipse has no size".into()));
                }
                let x = plane.x * (ux / major) + plane.y * (uy / major);
                let frame = Frame3 {
                    origin: map(c),
                    x,
                    y: plane.z.cross(&x),
                    z: plane.z,
                };
                circles.push((
                    ProfileLoop::new(vec![ProfileEdge::new(
                        CurveGeom::Ellipse(Ellipse {
                            frame,
                            major,
                            minor,
                        }),
                        (0.0, std::f64::consts::TAU),
                    )]),
                    label,
                ));
            }
            EntityKind::Spline { .. } | EntityKind::FitSpline { .. } => {
                let flat = kw_sketch::geom::spline_curve(sk, id).map_err(sk_err)?;
                let pts: Vec<_> = flat.points().iter().map(|p| map((p.x, p.y))).collect();
                let curve = NurbsCurve::new(flat.knots().clone(), pts, flat.weights().to_vec())
                    .map_err(|e| EvalError::Sketch(format!("bad spline: {e:?}")))?;
                let (t0, t1) = curve.domain();
                let (pa, pb) = (curve.point_at(t0), curve.point_at(t1));
                let to2 = |q: kw_math::Point3<f64>| {
                    let d = q - plane.origin;
                    (d.dot(&plane.x), d.dot(&plane.y))
                };
                segs.push(Seg {
                    a: to2(pa),
                    b: to2(pb),
                    edge: ProfileEdge::new(CurveGeom::Nurbs(curve), (t0, t1)),
                    label,
                });
            }
            // points are the end points of curves (or free points, which carry no area)
            EntityKind::Point(_) => {}
        }
    }
    let tol = 1e-7;
    let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1) <= tol;
    let mut loops: Vec<(ProfileLoop, Vec<u32>)> = Vec::new();
    while let Some(first) = segs.pop() {
        let mut chain = vec![first.edge];
        let mut labels = vec![first.label];
        let mut end = first.b;
        let start = first.a;
        while !near(end, start) {
            let Some(i) = segs.iter().position(|s| near(s.a, end) || near(s.b, end)) else {
                return Err(EvalError::Sketch("sketch profile is not closed".into()));
            };
            let s = segs.remove(i);
            labels.push(s.label);
            if near(s.a, end) {
                end = s.b;
                chain.push(s.edge);
            } else {
                let mut e = s.edge;
                e.reversed = true;
                end = s.a;
                chain.push(e);
            }
        }
        loops.push((ProfileLoop::new(chain), labels));
    }
    loops.extend(circles.into_iter().map(|(l, lab)| (l, vec![lab])));
    if loops.is_empty() {
        return Err(EvalError::Sketch("the sketch has no closed profile".into()));
    }
    let area = |l: &ProfileLoop| l.check(&plane.z, 1e-6).unwrap_or(0.0);
    let mut idx: Vec<usize> = (0..loops.len()).collect();
    idx.sort_by(|a, b| {
        area(&loops[*b].0)
            .abs()
            .partial_cmp(&area(&loops[*a].0).abs())
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.cmp(b))
    });
    let reverse = |l: &ProfileLoop, labels: &[u32]| -> (ProfileLoop, Vec<u32>) {
        let edges: Vec<ProfileEdge> = l
            .edges
            .iter()
            .rev()
            .map(|e| ProfileEdge {
                reversed: !e.reversed,
                ..e.clone()
            })
            .collect();
        (
            ProfileLoop::new(edges),
            labels.iter().rev().copied().collect(),
        )
    };
    let mut ordered: Vec<(ProfileLoop, Vec<u32>)> = Vec::new();
    for (k, i) in idx.into_iter().enumerate() {
        let (l, labels) = &loops[i];
        let ccw = area(l) > 0.0;
        let want_ccw = k == 0;
        if ccw == want_ccw {
            ordered.push((l.clone(), labels.clone()));
        } else if l.edges.len() == 1 {
            // a full circle's orientation comes from its frame: rebuild it mirrored
            if let CurveGeom::Circle(c) = &l.edges[0].curve {
                let f = Frame3 {
                    origin: c.frame.origin,
                    x: c.frame.x,
                    y: -c.frame.y,
                    z: -c.frame.z,
                };
                ordered.push((ProfileLoop::circle(&f, c.radius), labels.clone()));
            } else if let CurveGeom::Ellipse(e) = &l.edges[0].curve {
                let f = Frame3 {
                    origin: e.frame.origin,
                    x: e.frame.x,
                    y: -e.frame.y,
                    z: -e.frame.z,
                };
                ordered.push((
                    ProfileLoop::new(vec![ProfileEdge::new(
                        CurveGeom::Ellipse(Ellipse {
                            frame: f,
                            major: e.major,
                            minor: e.minor,
                        }),
                        (0.0, std::f64::consts::TAU),
                    )]),
                    labels.clone(),
                ));
            } else {
                ordered.push(reverse(l, labels));
            }
        } else {
            ordered.push(reverse(l, labels));
        }
    }
    let labels: Vec<Vec<u32>> = ordered.iter().map(|(_, l)| l.clone()).collect();
    let mut it = ordered.into_iter().map(|(l, _)| l);
    let mut profile = Profile::new(plane, it.next().unwrap());
    profile.holes = it.collect();
    Ok(LabeledProfile { profile, labels })
}

/// As [`sketch_profile_labeled`] with entity labels taken from the sketch's slot order.
pub fn sketch_profile(sk: &Sketch, plane: Frame3) -> Result<Profile, EvalError> {
    let ents: Vec<EntityId> = sk.entities.keys().collect();
    Ok(sketch_profile_labeled(sk, &ents, plane)?.profile)
}
