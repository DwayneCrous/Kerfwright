//! Sketch editing operations: trim, extend and offset.

use crate::entity::{EntityId, EntityKind};
use crate::geom::{self, arc_params, intersections, P2};
use crate::sketch::{Sketch, SketchError};
use std::f64::consts::TAU;

#[derive(Debug, Clone, PartialEq)]
pub enum TrimOutcome {
    /// The whole entity was removed (nothing cut it).
    Deleted,
    /// One piece remains (the original entity, shortened).
    Shortened(EntityId),
    /// The entity was split: the original (first part) and a new entity (second part).
    Split(EntityId, EntityId),
    /// A circle became an arc.
    CircleToArc(EntityId),
}

fn others(sk: &Sketch, e: EntityId) -> Vec<EntityId> {
    sk.entities
        .iter()
        .filter(|(id, ent)| {
            *id != e
                && matches!(
                    ent.kind,
                    EntityKind::Line { .. } | EntityKind::Circle { .. } | EntityKind::Arc { .. }
                )
        })
        .map(|(id, _)| id)
        .collect()
}

fn drop_constraints_of(sk: &mut Sketch, e: EntityId) {
    let dead: Vec<_> = sk
        .constraints
        .iter()
        .filter(|(_, c)| c.kind.entities().contains(&e))
        .map(|(id, _)| id)
        .collect();
    for c in dead {
        sk.constraints.remove(c);
    }
    sk.touch();
}

fn set_line_ends(sk: &mut Sketch, e: EntityId, p1: Option<EntityId>, p2: Option<EntityId>) {
    if let Some(ent) = sk.entities.get_mut(e) {
        if let EntityKind::Line { p1: a, p2: b } = &mut ent.kind {
            if let Some(p) = p1 {
                *a = p;
            }
            if let Some(p) = p2 {
                *b = p;
            }
        }
    }
    sk.touch();
}

fn set_arc_ends(sk: &mut Sketch, e: EntityId, start: Option<EntityId>, end: Option<EntityId>) {
    if let Some(ent) = sk.entities.get_mut(e) {
        if let EntityKind::Arc {
            start: s, end: t, ..
        } = &mut ent.kind
        {
            if let Some(p) = start {
                *s = p;
            }
            if let Some(p) = end {
                *t = p;
            }
        }
    }
    sk.touch();
}

/// Cut the part of `e` under `click` out between the nearest intersections with
/// the other curves. New end points are tied to the cutting curves with
/// point-on-curve constraints; dimensions and constraints of `e` are dropped.
pub fn trim(sk: &mut Sketch, e: EntityId, click: P2) -> Result<TrimOutcome, SketchError> {
    let out = trim_inner(sk, e, click)?;
    sk.remove_unused_points();
    Ok(out)
}

fn trim_inner(sk: &mut Sketch, e: EntityId, click: P2) -> Result<TrimOutcome, SketchError> {
    match sk.kind(e)?.clone() {
        EntityKind::Line { p1, p2 } => {
            let (a, b) = (sk.point(p1)?, sk.point(p2)?);
            let d = (b.0 - a.0, b.1 - a.1);
            let l2 = d.0 * d.0 + d.1 * d.1;
            if l2 == 0.0 {
                return Err(SketchError::Degenerate("zero-length line"));
            }
            let param = |q: P2| ((q.0 - a.0) * d.0 + (q.1 - a.1) * d.1) / l2;
            let tc = param(click).clamp(0.0, 1.0);
            let mut left: Option<(f64, P2, EntityId)> = None;
            let mut right: Option<(f64, P2, EntityId)> = None;
            for o in others(sk, e) {
                for q in intersections(sk, e, o)? {
                    let t = param(q);
                    if t > 1e-9 && t < 1.0 - 1e-9 {
                        if t < tc - 1e-12 && left.is_none_or(|(lt, _, _)| t > lt) {
                            left = Some((t, q, o));
                        }
                        if t > tc + 1e-12 && right.is_none_or(|(rt, _, _)| t < rt) {
                            right = Some((t, q, o));
                        }
                    }
                }
            }
            drop_constraints_of(sk, e);
            let new_point =
                |sk: &mut Sketch, q: P2, cutter: EntityId| -> Result<EntityId, SketchError> {
                    let p = sk.add_point(q.0, q.1);
                    sk.point_on(p, cutter)?;
                    Ok(p)
                };
            match (left, right) {
                (None, None) => {
                    sk.remove_entity(e);
                    sk.remove_unused_points();
                    Ok(TrimOutcome::Deleted)
                }
                (None, Some((_, q, o))) => {
                    let np = new_point(sk, q, o)?;
                    set_line_ends(sk, e, Some(np), None);
                    Ok(TrimOutcome::Shortened(e))
                }
                (Some((_, q, o)), None) => {
                    let np = new_point(sk, q, o)?;
                    set_line_ends(sk, e, None, Some(np));
                    Ok(TrimOutcome::Shortened(e))
                }
                (Some((_, ql, ol)), Some((_, qr, or))) => {
                    let (pl, pr) = (new_point(sk, ql, ol)?, new_point(sk, qr, or)?);
                    let second = sk.add_line(pr, p2);
                    sk.set_construction(second, sk.entities[e].construction);
                    set_line_ends(sk, e, None, Some(pl));
                    Ok(TrimOutcome::Split(e, second))
                }
            }
        }
        EntityKind::Arc { center, start, end } => {
            let (c, r, a0, a1) = arc_params(sk, e)?;
            let ang = |q: P2| {
                let mut a = (q.1 - c.1).atan2(q.0 - c.0);
                while a < a0 - 1e-12 {
                    a += TAU;
                }
                a
            };
            let ac = ang(click).min(a1);
            let (mut left, mut right): (Option<(f64, P2, EntityId)>, Option<(f64, P2, EntityId)>) =
                (None, None);
            for o in others(sk, e) {
                for q in intersections(sk, e, o)? {
                    let a = ang(q);
                    if a > a0 + 1e-9 && a < a1 - 1e-9 {
                        if a < ac - 1e-12 && left.is_none_or(|(x, _, _)| a > x) {
                            left = Some((a, q, o));
                        }
                        if a > ac + 1e-12 && right.is_none_or(|(x, _, _)| a < x) {
                            right = Some((a, q, o));
                        }
                    }
                }
            }
            let _ = r;
            drop_constraints_of(sk, e);
            let new_point =
                |sk: &mut Sketch, q: P2, cutter: EntityId| -> Result<EntityId, SketchError> {
                    let p = sk.add_point(q.0, q.1);
                    sk.point_on(p, cutter)?;
                    Ok(p)
                };
            match (left, right) {
                (None, None) => {
                    sk.remove_entity(e);
                    sk.remove_unused_points();
                    Ok(TrimOutcome::Deleted)
                }
                (None, Some((_, q, o))) => {
                    let np = new_point(sk, q, o)?;
                    set_arc_ends(sk, e, Some(np), None);
                    Ok(TrimOutcome::Shortened(e))
                }
                (Some((_, q, o)), None) => {
                    let np = new_point(sk, q, o)?;
                    set_arc_ends(sk, e, None, Some(np));
                    Ok(TrimOutcome::Shortened(e))
                }
                (Some((_, ql, ol)), Some((_, qr, or))) => {
                    let (pl, pr) = (new_point(sk, ql, ol)?, new_point(sk, qr, or)?);
                    let second = sk.add_arc(center, pr, end);
                    set_arc_ends(sk, e, Some(start), Some(pl));
                    Ok(TrimOutcome::Split(e, second))
                }
            }
        }
        EntityKind::Circle { center, .. } => {
            let (c, r) = sk.circle_geometry(e)?;
            let ang = |q: P2| (q.1 - c.1).atan2(q.0 - c.0).rem_euclid(TAU);
            let ac = ang(click);
            let mut hits: Vec<(f64, P2, EntityId)> = Vec::new();
            for o in others(sk, e) {
                for q in intersections(sk, e, o)? {
                    hits.push((ang(q), q, o));
                }
            }
            hits.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
            hits.dedup_by(|x, y| (x.0 - y.0).abs() < 1e-9);
            if hits.len() < 2 {
                return Err(SketchError::Unsupported(
                    "a circle needs at least two intersections to be trimmed",
                ));
            }
            // neighbours around the click (cyclic)
            let after = hits.iter().find(|h| h.0 > ac).copied().unwrap_or(hits[0]);
            let before = hits
                .iter()
                .rev()
                .find(|h| h.0 < ac)
                .copied()
                .unwrap_or(*hits.last().unwrap());
            let _ = r;
            let (s_pt, e_pt) = (
                sk.add_point(after.1 .0, after.1 .1),
                sk.add_point(before.1 .0, before.1 .1),
            );
            sk.point_on(s_pt, after.2)?;
            sk.point_on(e_pt, before.2)?;
            let arc = sk.add_arc(center, s_pt, e_pt);
            sk.remove_entity(e);
            Ok(TrimOutcome::CircleToArc(arc))
        }
        _ => Err(SketchError::Unsupported(
            "trim supports lines, arcs and circles",
        )),
    }
}

/// Extend the end of line `e` nearest to `near` to the closest curve it would hit.
pub fn extend(sk: &mut Sketch, e: EntityId, near: P2) -> Result<(), SketchError> {
    let (p1, p2) = sk.line_points(e)?;
    let (a, b) = (sk.point(p1)?, sk.point(p2)?);
    let end_is_b = (near.0 - b.0).hypot(near.1 - b.1) <= (near.0 - a.0).hypot(near.1 - a.1);
    let (from, to, moving) = if end_is_b { (a, b, p2) } else { (b, a, p1) };
    let d = (to.0 - from.0, to.1 - from.1);
    let l = d.0.hypot(d.1);
    if l == 0.0 {
        return Err(SketchError::Degenerate("zero-length line"));
    }
    let far = (to.0 + d.0 / l * 1e7, to.1 + d.1 / l * 1e7);
    let mut tmp = sk.clone();
    let probe = tmp.add_line_xy(to.0, to.1, far.0, far.1);
    let mut best: Option<(f64, P2, EntityId)> = None;
    for o in others(sk, e) {
        for q in intersections(&tmp, probe, o)? {
            let t = (q.0 - to.0).hypot(q.1 - to.1);
            if t > 1e-9 && best.is_none_or(|(bt, _, _)| t < bt) {
                best = Some((t, q, o));
            }
        }
    }
    let (_, q, cutter) = best.ok_or(SketchError::Degenerate("nothing to extend to"))?;
    sk.set_point(moving, q.0, q.1)?;
    sk.point_on(moving, cutter)?;
    Ok(())
}

/// Offset `entities` by `distance` towards the side of `side`. Lines get a
/// parallel copy tied by an offset-distance dimension (and equal length for a
/// single line); consecutive lines sharing an end point are joined at the
/// intersection of their offsets. Circles and arcs get a concentric copy with a
/// radius dimension. Returns the new entities.
pub fn offset(
    sk: &mut Sketch,
    entities: &[EntityId],
    distance: f64,
    side: P2,
) -> Result<Vec<EntityId>, SketchError> {
    if distance <= 0.0 {
        return Err(SketchError::Degenerate("offset distance must be positive"));
    }
    let mut created = Vec::new();
    let mut line_pairs: Vec<(EntityId, EntityId)> = Vec::new(); // (old, new)
    for &e in entities {
        match sk.kind(e)?.clone() {
            EntityKind::Line { p1, p2 } => {
                let (a, b) = (sk.point(p1)?, sk.point(p2)?);
                let d = (b.0 - a.0, b.1 - a.1);
                let l = d.0.hypot(d.1);
                if l == 0.0 {
                    return Err(SketchError::Degenerate("zero-length line"));
                }
                let n = (-d.1 / l, d.0 / l);
                let s = ((side.0 - a.0) * n.0 + (side.1 - a.1) * n.1).signum();
                let off = (n.0 * s * distance, n.1 * s * distance);
                let new = sk.add_line_xy(a.0 + off.0, a.1 + off.1, b.0 + off.0, b.1 + off.1);
                sk.set_construction(new, sk.entities[e].construction);
                sk.dim_distance(new, e)?;
                created.push(new);
                line_pairs.push((e, new));
            }
            EntityKind::Circle { center, .. } | EntityKind::Arc { center, .. } => {
                let (c, r) = sk.circle_geometry(e)?;
                let outside = (side.0 - c.0).hypot(side.1 - c.1) > r;
                let nr = if outside { r + distance } else { r - distance };
                if nr <= 1e-9 {
                    return Err(SketchError::Degenerate("offset collapses the circle"));
                }
                match sk.kind(e)?.clone() {
                    EntityKind::Circle { .. } => {
                        let new = sk.add_circle(center, nr);
                        sk.dim_radius(new)?;
                        created.push(new);
                    }
                    _ => {
                        let (_, _, a0, a1) = arc_params(sk, e)?;
                        let k = nr / r;
                        let s = sk.add_point(c.0 + (nr) * a0.cos(), c.1 + nr * a0.sin());
                        let t = sk.add_point(c.0 + nr * a1.cos(), c.1 + nr * a1.sin());
                        let new = sk.add_arc(center, s, t);
                        let _ = k;
                        sk.dim_radius(new)?;
                        created.push(new);
                    }
                }
            }
            _ => {
                return Err(SketchError::Unsupported(
                    "offset supports lines, arcs and circles",
                ))
            }
        }
    }
    // join consecutive line offsets that shared an end point
    let mut joined_any = false;
    for w in 0..line_pairs.len().saturating_sub(1) {
        let ((o1, n1), (o2, n2)) = (line_pairs[w], line_pairs[w + 1]);
        let (a1, a2) = sk.line_points(o1)?;
        let (b1, b2) = sk.line_points(o2)?;
        let shared = [a1, a2].into_iter().find(|p| *p == b1 || *p == b2);
        let Some(sp) = shared else { continue };
        let (n1p1, n1p2) = sk.line_points(n1)?;
        let (n2p1, n2p2) = sk.line_points(n2)?;
        let (m1, m2) = (
            if sp == a1 { n1p1 } else { n1p2 },
            if sp == b1 { n2p1 } else { n2p2 },
        );
        // intersection of the infinite offset lines
        let (q1, q2) = (sk.point(n1p1)?, sk.point(n1p2)?);
        let (r1, r2) = (sk.point(n2p1)?, sk.point(n2p2)?);
        let (d1, d2) = ((q2.0 - q1.0, q2.1 - q1.1), (r2.0 - r1.0, r2.1 - r1.1));
        let det = d1.0 * d2.1 - d1.1 * d2.0;
        if det.abs() < 1e-12 {
            continue;
        }
        let s = ((r1.0 - q1.0) * d2.1 - (r1.1 - q1.1) * d2.0) / det;
        let x = (q1.0 + s * d1.0, q1.1 + s * d1.1);
        sk.set_point(m1, x.0, x.1)?;
        // make n2 use n1's corner point
        if let Some(ent) = sk.entities.get_mut(n2) {
            if let EntityKind::Line { p1, p2 } = &mut ent.kind {
                if *p1 == m2 {
                    *p1 = m1;
                } else {
                    *p2 = m1;
                }
            }
        }
        sk.touch();
        sk.remove_entity(m2);
        joined_any = true;
    }
    if !joined_any && line_pairs.len() == 1 {
        let (old, new) = line_pairs[0];
        sk.equal_length(new, old)?;
    }
    let _ = geom::polyline;
    Ok(created)
}
