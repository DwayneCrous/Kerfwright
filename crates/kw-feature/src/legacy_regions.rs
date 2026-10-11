//! Regions of a sketch as they were named before the planar arrangement: the pieces the closed loops cut the plane
//! into, named by the loops they lie inside. Used once, to turn a saved choice of that kind into a position.
//!
//! Every closed loop of a sketch is a loop of its own, whether it lies inside another one (a hole) or crosses
//! it. A *region* is named by the set of loops it lies inside: for a circle that overlaps a rectangle the
//! regions are {rectangle}, {circle} and {rectangle, circle}. Polygons (the loops sampled finely) decide which
//! regions exist and let the view draw and pick them; the solids themselves are made exactly by the kernel
//! (see `tools.rs`).

use crate::eval::EvalError;
use crate::legacy_profile::sketch_profile_labeled;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;
use kw_math::Frame3;
use kw_model::ProfileLoop;
use kw_sketch::{EntityId, Sketch};

pub type P2 = [f64; 2];
/// Contours of a shape: the outline and then its holes.
pub type Shape = Vec<Vec<P2>>;
pub type Shapes = Vec<Shape>;

/// A closed loop of the sketch.
#[derive(Debug, Clone)]
pub struct SketchLoop {
    /// Names the loop in a region signature: the smallest label of its entities.
    pub key: u32,
    /// Labels of the loop's entities, in loop order.
    pub labels: Vec<u32>,
    pub profile: ProfileLoop,
    /// The loop sampled, in sketch-plane coordinates.
    pub polygon: Vec<P2>,
}

/// A region and where it is.
#[derive(Debug, Clone, PartialEq)]
pub struct Region {
    pub signature: Vec<u32>,
    pub shapes: Shapes,
    pub area: f64,
}

fn sample(l: &ProfileLoop, plane: &Frame3) -> Vec<P2> {
    use kw_nurbs::Curve3;
    let to2 = |p: kw_math::Point3<f64>| {
        let d = p - plane.origin;
        [d.dot(&plane.x), d.dot(&plane.y)]
    };
    let mut out: Vec<P2> = Vec::new();
    for e in &l.edges {
        let (a, b) = if e.reversed {
            (e.range.1, e.range.0)
        } else {
            (e.range.0, e.range.1)
        };
        let n = match &e.curve {
            kw_nurbs::CurveGeom::Line(_) => 1,
            _ => 64,
        };
        for k in 0..n {
            out.push(to2(e.curve.point_at(a + (b - a) * k as f64 / n as f64)));
        }
    }
    out
}

/// The closed loops of a sketch, each on its own. `entities[i]` is the entity with label `i`.
pub fn sketch_loops(
    sk: &Sketch,
    entities: &[EntityId],
    plane: Frame3,
) -> Result<Vec<SketchLoop>, EvalError> {
    let lp = sketch_profile_labeled(sk, entities, plane)?;
    let all = std::iter::once(&lp.profile.outer).chain(lp.profile.holes.iter());
    Ok(all
        .enumerate()
        .map(|(i, l)| {
            let labels = lp.labels[i].clone();
            SketchLoop {
                key: labels.iter().copied().min().unwrap_or(u32::MAX),
                labels,
                profile: l.clone(),
                polygon: sample(l, &plane),
            }
        })
        .collect())
}

fn contour_area(c: &[P2]) -> f64 {
    let n = c.len();
    0.5 * (0..n)
        .map(|i| c[i][0] * c[(i + 1) % n][1] - c[(i + 1) % n][0] * c[i][1])
        .sum::<f64>()
}

pub fn shapes_area(s: &Shapes) -> f64 {
    s.iter()
        .map(|sh| {
            sh.iter()
                .enumerate()
                .map(|(i, c)| {
                    let a = contour_area(c).abs();
                    if i == 0 {
                        a
                    } else {
                        -a
                    }
                })
                .sum::<f64>()
        })
        .sum()
}

fn as_shapes(poly: &[P2]) -> Shapes {
    vec![vec![poly.to_vec()]]
}

fn overlay(a: &Shapes, b: &Shapes, rule: OverlayRule) -> Shapes {
    if a.is_empty() {
        return Vec::new();
    }
    a.overlay(b, rule, FillRule::NonZero)
}

/// The region lying inside exactly the loops whose keys are in `signature`: `None` when there is no such
/// region. The loops inside it come first in the second result, then the loops it must be cut away from
/// because they overlap it.
pub fn region_of(
    loops: &[SketchLoop],
    signature: &[u32],
) -> Option<(Region, Vec<usize>, Vec<usize>)> {
    let members: Vec<usize> = (0..loops.len())
        .filter(|i| signature.contains(&loops[*i].key))
        .collect();
    if members.is_empty() || members.len() != signature.len() {
        return None;
    }
    let mut cur = as_shapes(&loops[members[0]].polygon);
    for m in &members[1..] {
        cur = overlay(&cur, &as_shapes(&loops[*m].polygon), OverlayRule::Intersect);
    }
    // overlap tolerance relative to the loop sizes
    let scale: f64 = loops
        .iter()
        .map(|l| contour_area(&l.polygon).abs())
        .fold(0.0, f64::max);
    let eps = 1e-9 * scale.max(1.0);
    let mut cut = Vec::new();
    for j in 0..loops.len() {
        if members.contains(&j) {
            continue;
        }
        let p = as_shapes(&loops[j].polygon);
        let common = overlay(&cur, &p, OverlayRule::Intersect);
        if shapes_area(&common) > eps {
            cur = overlay(&cur, &p, OverlayRule::Difference);
            cut.push(j);
        }
    }
    let area = shapes_area(&cur);
    if area <= eps {
        return None;
    }
    let mut sig: Vec<u32> = signature.to_vec();
    sig.sort_unstable();
    Some((
        Region {
            signature: sig,
            shapes: cur,
            area,
        },
        members,
        cut,
    ))
}

/// Every region of the sketch (up to ten loops; more is too many to list).
pub fn regions(loops: &[SketchLoop]) -> Vec<Region> {
    let n = loops.len();
    if n == 0 || n > 10 {
        return Vec::new();
    }
    let mut out = Vec::new();
    for mask in 1u32..(1 << n) {
        let sig: Vec<u32> = (0..n)
            .filter(|i| mask >> i & 1 == 1)
            .map(|i| loops[i].key)
            .collect();
        if let Some((r, ..)) = region_of(loops, &sig) {
            out.push(r);
        }
    }
    out
}

/// Is `p` inside the polygon?
pub fn inside(poly: &[P2], p: P2) -> bool {
    let mut c = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            c = !c;
        }
        j = i;
    }
    c
}

/// The signature of the region containing `p`: the keys of the loops it lies inside (empty = outside all).
pub fn signature_at(loops: &[SketchLoop], p: P2) -> Vec<u32> {
    let mut s: Vec<u32> = loops
        .iter()
        .filter(|l| inside(&l.polygon, p))
        .map(|l| l.key)
        .collect();
    s.sort_unstable();
    s
}

/// A point inside the region a [`RegionPick::Legacy`] choice named, so that it can be found among the faces of
/// the new arrangement.
pub fn legacy_point(
    sk: &Sketch,
    entities: &[EntityId],
    plane: Frame3,
    signature: &[u32],
) -> Option<[f64; 2]> {
    let loops = sketch_loops(sk, entities, plane).ok()?;
    let (region, _, _) = region_of(&loops, signature)?;
    let shape = region.shapes.first()?;
    let to = |c: &Vec<P2>| -> Vec<(f64, f64)> { c.iter().map(|p| (p[0], p[1])).collect() };
    let outer = to(shape.first()?);
    let holes: Vec<Vec<(f64, f64)>> = shape.iter().skip(1).map(to).collect();
    let p = kw_sketch::arrangement::interior_of(&outer, &holes);
    Some([p.0, p.1])
}
