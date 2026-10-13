//! Plain 2-D curves that can be put into a sketch at once: projected body edges, section curves and the
//! outlines of text. Points that coincide share one point entity, so closed outlines form loops.

use crate::entity::{EntityId, EntityKind};
use crate::sketch::Sketch;
use std::collections::HashMap;

pub type P2 = (f64, f64);

#[derive(Debug, Clone, PartialEq)]
pub enum Curve2 {
    Line(P2, P2),
    Circle {
        c: P2,
        r: f64,
    },
    /// Counter-clockwise from `start` to `end` about `c`.
    Arc {
        c: P2,
        start: P2,
        end: P2,
    },
    Ellipse {
        c: P2,
        major_end: P2,
        minor: f64,
    },
    /// A Bézier segment as a clamped B-spline of degree `points.len() - 1` (2 = quadratic, 3 = cubic).
    Bezier(Vec<P2>),
    /// A spline through the points.
    Fit(Vec<P2>),
}

/// Put `curves` into the sketch; returns the new curve entities. `construction` makes them reference
/// geometry (not used by profiles).
pub fn add_curves(sk: &mut Sketch, curves: &[Curve2], construction: bool) -> Vec<EntityId> {
    let mut pts: HashMap<(i64, i64), EntityId> = HashMap::new();
    let mut point = |sk: &mut Sketch, p: P2| -> EntityId {
        let key = ((p.0 * 1e6).round() as i64, (p.1 * 1e6).round() as i64);
        *pts.entry(key).or_insert_with(|| sk.add_point(p.0, p.1))
    };
    let mut out = Vec::new();
    for c in curves {
        let e = match c {
            Curve2::Line(a, b) => {
                let (pa, pb) = (point(sk, *a), point(sk, *b));
                if pa == pb {
                    continue;
                }
                sk.add_line(pa, pb)
            }
            Curve2::Circle { c, r } => {
                let pc = point(sk, *c);
                sk.add_circle(pc, *r)
            }
            Curve2::Arc { c, start, end } => {
                let (pc, ps, pe) = (point(sk, *c), point(sk, *start), point(sk, *end));
                sk.add_arc(pc, ps, pe)
            }
            Curve2::Ellipse {
                c,
                major_end,
                minor,
            } => {
                let (pc, pm) = (point(sk, *c), point(sk, *major_end));
                sk.add_ellipse(pc, pm, *minor)
            }
            Curve2::Bezier(v) => {
                let ids: Vec<_> = v.iter().map(|p| point(sk, *p)).collect();
                match sk.add_spline(v.len() - 1, ids) {
                    Ok(e) => e,
                    Err(_) => continue,
                }
            }
            Curve2::Fit(v) => {
                let ids: Vec<_> = v.iter().map(|p| point(sk, *p)).collect();
                sk.add_fit_spline(ids)
            }
        };
        if construction {
            sk.set_construction(e, true);
        }
        out.push(e);
    }
    out
}

/// Copy `es` (and the points they use) with every point mapped through `f`; returns the new curves.
/// `f` must preserve orientation (a translation or rotation): arcs keep their sense of rotation.
pub fn copy_entities(
    sk: &mut Sketch,
    es: &[EntityId],
    f: &dyn Fn(P2) -> P2,
) -> Result<Vec<EntityId>, crate::sketch::SketchError> {
    let mut map: HashMap<EntityId, EntityId> = HashMap::new();
    let mut out = Vec::new();
    let mut pt = |sk: &mut Sketch, p: EntityId| -> Result<EntityId, crate::sketch::SketchError> {
        if let Some(m) = map.get(&p) {
            return Ok(*m);
        }
        let q = f(sk.point(p)?);
        let n = sk.add_point(q.0, q.1);
        map.insert(p, n);
        Ok(n)
    };
    for &e in es {
        let construction = sk.entities.get(e).is_some_and(|x| x.construction);
        let new = match sk.kind(e)?.clone() {
            EntityKind::Point(_) => {
                pt(sk, e)?;
                continue;
            }
            EntityKind::Line { p1, p2 } => {
                let (a, b) = (pt(sk, p1)?, pt(sk, p2)?);
                sk.add_line(a, b)
            }
            EntityKind::Circle { center, radius } => {
                let c = pt(sk, center)?;
                let r = sk.value(radius);
                sk.add_circle(c, r)
            }
            EntityKind::Arc { center, start, end } => {
                let (c, s, en) = (pt(sk, center)?, pt(sk, start)?, pt(sk, end)?);
                sk.add_arc(c, s, en)
            }
            EntityKind::Ellipse {
                center,
                major_end,
                minor_radius,
            } => {
                let (c, m) = (pt(sk, center)?, pt(sk, major_end)?);
                let minor = sk.value(minor_radius);
                sk.add_ellipse(c, m, minor)
            }
            EntityKind::Spline { degree, control } => {
                let ids = control
                    .iter()
                    .map(|q| pt(sk, *q))
                    .collect::<Result<Vec<_>, _>>()?;
                sk.add_spline(degree, ids)?
            }
            EntityKind::FitSpline { points } => {
                let ids = points
                    .iter()
                    .map(|q| pt(sk, *q))
                    .collect::<Result<Vec<_>, _>>()?;
                sk.add_fit_spline(ids)
            }
        };
        if construction {
            sk.set_construction(new, true);
        }
        out.push(new);
    }
    Ok(out)
}

fn dist(a: P2, b: P2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// Join loose segments into chains of points; a chain whose ends meet is closed (first point repeated
/// at the end). `tol` is the largest gap bridged.
pub fn chain_segments(segs: &[(P2, P2)], tol: f64) -> Vec<Vec<P2>> {
    let mut left: Vec<(P2, P2)> = segs
        .iter()
        .copied()
        .filter(|(a, b)| dist(*a, *b) > 1e-12)
        .collect();
    let mut chains = Vec::new();
    while let Some((a, b)) = left.pop() {
        let mut chain = vec![a, b];
        loop {
            let (head, tail) = (chain[0], *chain.last().unwrap());
            if chain.len() > 2 && dist(head, tail) <= tol {
                break;
            }
            // extend at the tail, then at the head
            let mut extended = false;
            for i in 0..left.len() {
                let (p, q) = left[i];
                if dist(p, tail) <= tol {
                    chain.push(q);
                } else if dist(q, tail) <= tol {
                    chain.push(p);
                } else if dist(p, head) <= tol {
                    chain.insert(0, q);
                } else if dist(q, head) <= tol {
                    chain.insert(0, p);
                } else {
                    continue;
                }
                left.swap_remove(i);
                extended = true;
                break;
            }
            if !extended {
                break;
            }
        }
        if chain.len() > 2 && dist(chain[0], *chain.last().unwrap()) <= tol {
            let first = chain[0];
            *chain.last_mut().unwrap() = first;
        }
        chains.push(chain);
    }
    chains
}

/// Fit a circle through three points; `None` when they are (nearly) collinear.
fn circle3(a: P2, b: P2, c: P2) -> Option<(P2, f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (
        a.0 * a.0 + a.1 * a.1,
        b.0 * b.0 + b.1 * b.1,
        c.0 * c.0 + c.1 * c.1,
    );
    let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
    let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
    Some(((ux, uy), dist((ux, uy), a)))
}

fn line_dev(p: P2, a: P2, b: P2) -> f64 {
    let l = dist(a, b);
    if l < 1e-12 {
        return dist(p, a);
    }
    ((p.0 - a.0) * (b.1 - a.1) - (p.1 - a.1) * (b.0 - a.0)).abs() / l
}

/// Turn a polyline (a section or a tessellated edge) back into lines, circles and arcs where it is
/// straight or round to within `tol`; what is neither becomes a fit spline.
pub fn simplify_chain(pts: &[P2], tol: f64) -> Vec<Curve2> {
    let n = pts.len();
    if n < 2 {
        return Vec::new();
    }
    let closed = n > 3 && dist(pts[0], pts[n - 1]) < 1e-9;
    // a closed chain that is one circle
    if closed && n >= 8 {
        let pp = &pts[..n - 1];
        if let Some((c, r)) = circle3(pp[0], pp[pp.len() / 3], pp[2 * pp.len() / 3]) {
            if pp.iter().all(|p| (dist(*p, c) - r).abs() <= tol) {
                return vec![Curve2::Circle { c, r }];
            }
        }
    }
    let mut out = Vec::new();
    let mut i = 0;
    while i + 1 < n {
        // longest straight run starting at i
        let mut j = i + 1;
        while j + 1 < n && (i + 1..=j + 1).all(|k| line_dev(pts[k], pts[i], pts[j + 1]) <= tol) {
            j += 1;
        }
        // a run of at least 4 points on a common circle, if it reaches further than the straight run
        let mut arc: Option<(P2, usize)> = None;
        let mut m = i + 3;
        while m < n {
            let mid = i + (m - i) / 2;
            match circle3(pts[i], pts[mid], pts[m]) {
                Some((c, r)) if r < 1e6 && (i..=m).all(|q| (dist(pts[q], c) - r).abs() <= tol) => {
                    arc = Some((c, m));
                    m += 1;
                }
                _ => break,
            }
        }
        if let Some((c, m)) = arc.filter(|(_, m)| *m > j) {
            // counter-clockwise when the first chords turn left
            let cross = (pts[i + 1].0 - pts[i].0) * (pts[i + 2].1 - pts[i + 1].1)
                - (pts[i + 1].1 - pts[i].1) * (pts[i + 2].0 - pts[i + 1].0);
            let (s, e) = (pts[i], pts[m]);
            out.push(if cross >= 0.0 {
                Curve2::Arc {
                    c,
                    start: s,
                    end: e,
                }
            } else {
                Curve2::Arc {
                    c,
                    start: e,
                    end: s,
                }
            });
            i = m;
            continue;
        }
        out.push(Curve2::Line(pts[i], pts[j]));
        i = j;
    }
    // a closed chain may begin in the middle of a side: join the last and first pieces if they are one
    if closed && out.len() > 2 {
        let merged = match (out.last().unwrap(), &out[0]) {
            (Curve2::Line(a, m), Curve2::Line(m2, b))
                if dist(*m, *m2) < 1e-9 && line_dev(*m, *a, *b) <= tol =>
            {
                Some(Curve2::Line(*a, *b))
            }
            (
                Curve2::Arc { c, start, end },
                Curve2::Arc {
                    c: c2,
                    start: s2,
                    end: e2,
                },
            ) if dist(*end, *s2) < 1e-9
                && dist(*c, *c2) <= tol
                && (dist(*c, *start) - dist(*c2, *e2)).abs() <= tol =>
            {
                Some(Curve2::Arc {
                    c: *c,
                    start: *start,
                    end: *e2,
                })
            }
            _ => None,
        };
        if let Some(m) = merged {
            out.pop();
            out[0] = m;
        }
    }
    out
}
