//! Phase 1: broad phase, surface intersection, and clipping of the intersection curves to the
//! trimmed faces.

use crate::model::*;
use crate::BooleanError;
use kw_math::Point3;
use kw_nurbs::fit::translate_pcurve;
use kw_nurbs::intersect::curve_curve;
use kw_nurbs::{Curve3, CurveGeom, Surface};
use kw_ssi::{intersect, SsiCurve, SsiOutcome, Window, DEFAULT_TOL};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cont {
    In,
    Boundary,
    Out,
}

/// Where does the surface point `p` lie relative to the trimmed face?
pub fn contains(f: &FaceRec, p: &Point3<f64>, tol: f64) -> Cont {
    let uv = f.uv_of(p);
    let (su, sv) = f.surface.derivatives(uv.0, uv.1);
    let scale = su.norm().max(sv.norm()).max(1e-12);
    if f.loop_index.near(uv, 20.0 * tol / scale) {
        return Cont::Boundary;
    }
    if f.loop_index.contains(uv) {
        Cont::In
    } else {
        Cont::Out
    }
}

fn window(f: &FaceRec) -> Window {
    let widen = |lo: f64, hi: f64, period: Option<f64>| -> (f64, f64) {
        let span = hi - lo;
        match period {
            Some(p) if span >= p - 1e-9 => (lo, lo + p),
            Some(p) => {
                let m = (0.05 * span).max(1e-3).min(0.5 * (p - span).max(0.0));
                (lo - m, hi + m)
            }
            None => {
                let m = (0.05 * span).max(1e-3);
                (lo - m, hi + m)
            }
        }
    };
    use kw_nurbs::Surface;
    let ((du0, du1), (dv0, dv1)) = f.surface.domain();
    let clamp = |r: (f64, f64), dom: (f64, f64), periodic: bool| -> (f64, f64) {
        // never run past a bounded, non-periodic parameter domain (e.g. a sphere's poles: beyond
        // them the parameterisation mirrors back onto the surface and would be traced twice)
        if periodic || !dom.0.is_finite() || !dom.1.is_finite() {
            r
        } else {
            (r.0.max(dom.0), r.1.min(dom.1))
        }
    };
    let u = widen(f.uv_lo.0, f.uv_hi.0, f.surface.period_u());
    let v = widen(f.uv_lo.1, f.uv_hi.1, f.surface.period_v());
    Window {
        u: clamp(u, (du0, du1), f.surface.period_u().is_some()),
        v: clamp(v, (dv0, dv1), f.surface.period_v().is_some()),
    }
}

/// Sub-range of the curve whose points can lie in both (padded) face boxes; `None` if there is none.
fn restrict_range(w: &Work, c: &SsiCurve, fa: usize, fb: usize) -> Option<(f64, f64)> {
    let mut boxes = [w.faces[fa].bbox, w.faces[fb].bbox];
    for b in boxes.iter_mut() {
        // generous margin: end points of the window must lie clearly outside the face, otherwise a
        // sliver beyond the true boundary would be classified "on the boundary" and kept as a cut
        let pad = 4.0 * w.tol + 0.02 * b.diagonal();
        b.min -= kw_math::Vector3::repeat(pad);
        b.max += kw_math::Vector3::repeat(pad);
    }
    let (r0, r1) = c.range;
    if let CurveGeom::Line(l) = &c.curve {
        // slab clipping of the parametrised line
        let (mut lo, mut hi) = (r0, r1);
        for b in &boxes {
            for k in 0..3 {
                let (o, d) = (l.origin[k], l.dir[k]);
                if d.abs() < 1e-300 {
                    if o < b.min[k] || o > b.max[k] {
                        return None;
                    }
                } else {
                    let (t0, t1) = ((b.min[k] - o) / d, (b.max[k] - o) / d);
                    lo = lo.max(t0.min(t1));
                    hi = hi.min(t0.max(t1));
                }
            }
        }
        return (lo <= hi).then_some((lo, hi));
    }
    // other curves: sample; keep the full range if any sample is inside both boxes. A curve far longer
    // than the boxes (nearly parallel surfaces) needs a sampling that cannot step over their overlap.
    let n = {
        let coarse = 64usize;
        let mut len = 0.0;
        let mut prev = c.curve.point_at(r0);
        for i in 1..=coarse {
            let q = c.curve.point_at(r0 + (r1 - r0) * i as f64 / coarse as f64);
            len += (q - prev).norm();
            prev = q;
        }
        let small = boxes
            .iter()
            .map(|b| b.diagonal())
            .fold(f64::INFINITY, f64::min)
            .max(1e-9);
        ((len / (0.2 * small)).ceil() as usize).clamp(coarse, 200_000)
    };
    let inside = (0..=n).any(|i| {
        let p = c.curve.point_at(r0 + (r1 - r0) * i as f64 / n as f64);
        boxes.iter().all(|b| b.contains(&p, 0.0))
    });
    inside.then_some((r0, r1))
}

/// Hits of two curves that stay within tolerance of each other come at every sample of that stretch.
/// A long stretch is a real overlap: only its two ends are breakpoints. A short one is a tangential
/// touch (two curves grazing within tolerance): it counts once, in the middle, and not at all when it
/// sits at an end of the curve's window (the curve merely ends on the edge).
pub fn coalesce_overlaps(
    mut hits: Vec<kw_nurbs::intersect::CurveHit>,
    c1: &CurveGeom,
    win1: (f64, f64),
    c2: &CurveGeom,
    range2: (f64, f64),
    tol: f64,
) -> Vec<kw_nurbs::intersect::CurveHit> {
    if hits.is_empty() {
        return hits;
    }
    hits.sort_by(|x, y| x.t1.partial_cmp(&y.t1).unwrap_or(std::cmp::Ordering::Equal));
    let mid2 = 0.5 * (range2.0 + range2.1);
    let on_c2 = |t: f64| -> bool {
        let q = c1.point_at(t);
        kw_nurbs::proj::curve_param_of(c2, &q, Some(mid2)).distance <= 4.0 * tol
    };
    let joined: Vec<bool> = hits
        .windows(2)
        .map(|p| on_c2(0.5 * (p[0].t1 + p[1].t1)))
        .collect();
    let n = hits.len();
    let short = 5000.0 * tol;
    // a short stretch of the curve between parameters `a` and `b` that stays within a few tolerances of c2
    let grazes = |a: f64, b: f64| -> bool {
        let len = (c1.point_at(a) - c1.point_at(b)).norm();
        len <= short
            && (0..=8).all(|k| {
                let q = c1.point_at(a + (b - a) * k as f64 / 8.0);
                kw_nurbs::proj::curve_param_of(c2, &q, Some(mid2)).distance <= 50.0 * tol
            })
    };
    let mut out: Vec<kw_nurbs::intersect::CurveHit> = Vec::with_capacity(n);
    let mut i = 0;
    while i < n {
        let mut j = i;
        while j + 1 < n && joined[j] {
            j += 1;
        }
        if j == i {
            // a lone hit that is only the far end of a grazing stretch reaching the window's end
            let t = hits[i].t1;
            let to_start = (c1.point_at(t) - c1.point_at(win1.0)).norm();
            let to_end = (c1.point_at(t) - c1.point_at(win1.1)).norm();
            // a hit that only marks the far end of a grazing stretch leading to a window end that itself
            // lies on the edge (the curve ends on the edge, tangentially)
            let end_on_edge = |w: f64| {
                kw_nurbs::proj::curve_param_of(c2, &c1.point_at(w), Some(mid2)).distance
                    <= 3.0 * tol
            };
            let sliver = (to_start > 5.0 * tol && end_on_edge(win1.0) && grazes(win1.0, t))
                || (to_end > 5.0 * tol && end_on_edge(win1.1) && grazes(t, win1.1));
            if !sliver {
                out.push(hits[i]);
            }
        } else {
            let (pi, pj) = (c1.point_at(hits[i].t1), c1.point_at(hits[j].t1));
            let len = (pj - pi).norm();
            if len <= short {
                // tangential touch
                let at_end = [win1.0, win1.1].iter().any(|w| {
                    (c1.point_at(*w) - pi)
                        .norm()
                        .min((c1.point_at(*w) - pj).norm())
                        <= 2.0 * len + 10.0 * tol
                });
                if !at_end {
                    out.push(hits[(i + j) / 2]);
                }
            } else {
                out.push(hits[i]);
                out.push(hits[j]);
            }
        }
        i = j + 1;
    }
    out
}

/// Parameter intervals of `c` (over its range) lying inside both faces.
fn clip(w: &Work, c: &SsiCurve, fa: usize, fb: usize) -> Vec<(f64, f64)> {
    let Some((r0, r1)) = restrict_range(w, c, fa, fb) else {
        return Vec::new();
    };
    let narrowed = r0 > c.range.0 || r1 < c.range.1;
    // an unchanged-range closed curve keeps its closed flag; otherwise work on the narrowed window
    let window = (r0, r1);
    let cbox = {
        let n = 32;
        let pts: Vec<Point3<f64>> = (0..=n)
            .map(|i| {
                c.curve
                    .point_at(window.0 + (window.1 - window.0) * i as f64 / n as f64)
            })
            .collect();
        let mut b = kw_math::Aabb::from_points(&pts);
        let pad = 0.03 * b.diagonal() + 4.0 * w.tol;
        b.min -= kw_math::Vector3::repeat(pad);
        b.max += kw_math::Vector3::repeat(pad);
        b
    };
    let _ = narrowed;
    let mut cuts: Vec<f64> = Vec::new();
    let mut cand: Vec<u32> = Vec::new();
    for fi in [fa, fb] {
        let ids = &w.face_edge_ids[fi];
        match &w.face_edge_grid[fi] {
            Some(g) => g.query(&cbox, ids.len(), &mut cand),
            None => {
                cand.clear();
                cand.extend(0..ids.len() as u32);
            }
        }
        for &k in &cand {
            let e = &w.edges[&ids[k as usize]];
            if !e.bbox.intersects(&cbox) {
                continue;
            }
            if let Some(ec) = &e.curve {
                let hits = curve_curve(&c.curve, window, ec, e.range, w.tol * 0.5);
                let hits = coalesce_overlaps(hits, &c.curve, window, ec, e.range, w.tol);
                for h in hits {
                    cuts.push(h.t1);
                }
            }
        }
    }
    let (r0, r1) = window;
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let eps = 1e-10 * (r1 - r0).abs().max(1.0);
    let mut brk: Vec<f64> = vec![r0];
    for t in cuts {
        // tangential contacts (a blend arc touching an edge) yield clusters of nearly identical hits:
        // keep one breakpoint per cluster (cluster = within 20 tolerances of the previous one in 3D)
        let last = brk.last().copied().unwrap();
        let apart = (c.curve.point_at(t) - c.curve.point_at(last)).norm() > 20.0 * w.tol;
        if t > last + eps && t < r1 - eps && apart {
            brk.push(t);
        }
    }
    brk.push(r1);
    let keep = |ta: f64, tb: f64| -> bool {
        if tb - ta <= eps {
            return false;
        }
        let p = c.curve.point_at(0.5 * (ta + tb));
        contains(&w.faces[fa], &p, w.tol) != Cont::Out
            && contains(&w.faces[fb], &p, w.tol) != Cont::Out
    };
    // Note: the intervals on either side of a closed curve's parameter origin are deliberately NOT
    // merged — pcurves are generally not periodic in the curve parameter, and an extra vertex at the
    // origin is harmless.
    let intervals: Vec<(f64, f64, bool)> = brk
        .windows(2)
        .map(|p| (p[0], p[1], keep(p[0], p[1])))
        .collect();
    if std::env::var("KW_BOOL_DEBUG").is_ok() {
        eprintln!(
            "    clip faces ({fa},{fb}) range {:?} closed {} breaks {:?} intervals {:?}",
            c.range,
            c.closed,
            brk,
            intervals
                .iter()
                .map(|i| (i.0, i.1, i.2))
                .collect::<Vec<_>>()
        );
    }
    intervals
        .into_iter()
        .filter(|i| i.2)
        .map(|i| (i.0, i.1))
        .collect()
}

/// Shift a pcurve by whole periods so its value at `t` is nearest the face centre.
fn shift_to_face(f: &FaceRec, pc: &CurveGeom, t: f64) -> CurveGeom {
    let q = pc.point_at(t);
    let c = f.centre();
    let du = f
        .surface
        .period_u()
        .map_or(0.0, |p| p * ((c.0 - q.x) / p).round());
    let dv = f
        .surface
        .period_v()
        .map_or(0.0, |p| p * ((c.1 - q.y) / p).round());
    if du == 0.0 && dv == 0.0 {
        pc.clone()
    } else {
        translate_pcurve(pc, du, dv)
    }
}

pub fn compute(w: &mut Work) -> Result<(), BooleanError> {
    // edge indices for faces with long boundaries
    w.face_edge_grid = (0..w.faces.len())
        .map(|fi| {
            let ids = &w.face_edge_ids[fi];
            (ids.len() > 24).then(|| {
                crate::spatial::GridIndex::build(
                    &ids.iter().map(|e| w.edges[e].bbox).collect::<Vec<_>>(),
                )
            })
        })
        .collect();
    // broad phase: uniform grid over the second operand's face boxes
    let (na, nf) = (
        w.faces.iter().filter(|f| f.body == 0).count(),
        w.faces.len(),
    );
    let pad = w.tol * 4.0;
    w.pairs = broad_phase(&w.faces[..na], &w.faces[na..nf], pad)
        .into_iter()
        .map(|(i, j)| (i, na + j))
        .collect();
    let pairs = w.pairs.clone();
    // face pairs are independent; outcomes are merged in pair order so the result (and the first error
    // reported) does not depend on the thread count
    let outcomes: Vec<Result<PairOutcome, BooleanError>> = {
        use rayon::prelude::*;
        let wr: &Work = w;
        pairs
            .par_iter()
            .map(|&(fa, fb)| pair_cuts(wr, fa, fb))
            .collect()
    };
    for o in outcomes {
        let o = o?;
        if let Some(c) = o.coincident {
            w.coincident.push(c);
        }
        w.cuts.extend(o.cuts);
    }
    if std::env::var("KW_BOOL_DEBUG").is_ok() {
        for c in &w.cuts {
            eprintln!(
                "  cut faces ({},{}) range ({:.6},{:.6}) closed {} from {:?} to {:?}",
                c.fa,
                c.fb,
                c.range.0,
                c.range.1,
                c.closed,
                c.curve.point_at(c.range.0),
                c.curve.point_at(c.range.1)
            );
        }
    }
    for c in &w.cuts {
        w.faces[c.fa].involved = true;
        w.faces[c.fb].involved = true;
    }
    Ok(())
}

#[derive(Default)]
struct PairOutcome {
    coincident: Option<(usize, usize, bool)>,
    cuts: Vec<CutSeg>,
}

/// Intersect the surfaces of faces `fa`, `fb` and clip the curves to both faces.
fn pair_cuts(w: &Work, fa: usize, fb: usize) -> Result<PairOutcome, BooleanError> {
    let mut res = PairOutcome::default();
    let (wa, wb) = (window(&w.faces[fa]), window(&w.faces[fb]));
    let out = intersect(
        &w.faces[fa].surface,
        wa,
        &w.faces[fb].surface,
        wb,
        DEFAULT_TOL,
    )?;
    if std::env::var("KW_BOOL_DEBUG").is_ok() {
        let d = match &out {
            SsiOutcome::Empty => "empty".to_string(),
            SsiOutcome::Coincident { .. } => "coincident".to_string(),
            SsiOutcome::Points(p) => format!("{} points", p.len()),
            SsiOutcome::Curves { curves, tangent } => format!(
                "{} curves (tangent {tangent}) ranges {:?}",
                curves.len(),
                curves.iter().map(|c| c.range).collect::<Vec<_>>()
            ),
        };
        eprintln!(
            "  pair ({fa},{fb}) {:?} x {:?}: {d}  windows {wa:?} {wb:?}",
            std::mem::discriminant(&w.faces[fa].surface),
            std::mem::discriminant(&w.faces[fb].surface)
        );
    }
    match out {
        SsiOutcome::Empty => {}
        SsiOutcome::Coincident { same_normal } => {
            // surfaces coincide: only relevant if the trims overlap (decided by imprinting)
            res.coincident = Some((fa, fb, same_normal));
        }
        SsiOutcome::Points(pts) => {
            for p in pts {
                let (ca, cb) = (
                    contains(&w.faces[fa], &p, w.tol),
                    contains(&w.faces[fb], &p, w.tol),
                );
                // a tangent point that lies on the boundary of both faces (a corner of a blend patch
                // touching the face it is tangent to) changes nothing: the boundaries meet there anyway
                if ca != Cont::Out
                    && cb != Cont::Out
                    && !(ca == Cont::Boundary && cb == Cont::Boundary)
                {
                    return Err(BooleanError::Tangent("point contact between faces"));
                }
            }
        }
        SsiOutcome::Curves { curves, tangent } => {
            for c in &curves {
                let ivs = clip(w, c, fa, fb);
                // Tangent contact along a curve (e.g. a fillet cylinder meeting its support plane) is
                // treated like any other cut: the arrangement splits both faces there. Contacts that
                // would make the result non-manifold are rejected later by the usage check.
                let _ = tangent;
                for (t0, t1) in ivs {
                    let tm = 0.5 * (t0 + t1);
                    let pa = shift_to_face(&w.faces[fa], &c.pcurve_a, tm);
                    let pb = shift_to_face(&w.faces[fb], &c.pcurve_b, tm);
                    let closed = c.closed
                        && (t1 - t0 - (c.range.1 - c.range.0)).abs()
                            < 1e-9 * (1.0 + (c.range.1 - c.range.0).abs());
                    res.cuts.push(CutSeg {
                        curve: c.curve.clone(),
                        range: (t0, t1),
                        closed,
                        fa,
                        fb,
                        pcurve_a: pa,
                        pcurve_b: pb,
                    });
                }
            }
        }
    }
    Ok(res)
}

/// Candidate pairs `(i, j)` of overlapping padded bounding boxes between `a` and `b`, sorted.
fn broad_phase(a: &[FaceRec], b: &[FaceRec], pad: f64) -> Vec<(usize, usize)> {
    use std::collections::HashMap;
    let padded = |f: &FaceRec| {
        let mut bb = f.bbox;
        bb.min -= kw_math::Vector3::repeat(pad);
        bb.max += kw_math::Vector3::repeat(pad);
        bb
    };
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let ba: Vec<_> = a.iter().map(padded).collect();
    let bb: Vec<_> = b.iter().map(padded).collect();
    let mean =
        |v: &[kw_math::Aabb]| v.iter().map(|x| x.extent().amax()).sum::<f64>() / v.len() as f64;
    let cell = (2.0 * (mean(&ba) + mean(&bb)) / 2.0).max(1e-9);
    let key = |x: f64| (x / cell).floor() as i64;
    const MAX_CELLS: i64 = 512;
    let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    let mut big: Vec<usize> = Vec::new();
    for (j, x) in bb.iter().enumerate() {
        let (lo, hi) = (
            (key(x.min.x), key(x.min.y), key(x.min.z)),
            (key(x.max.x), key(x.max.y), key(x.max.z)),
        );
        if (hi.0 - lo.0 + 1) * (hi.1 - lo.1 + 1) * (hi.2 - lo.2 + 1) > MAX_CELLS {
            big.push(j);
            continue;
        }
        for i in lo.0..=hi.0 {
            for jj in lo.1..=hi.1 {
                for k in lo.2..=hi.2 {
                    grid.entry((i, jj, k)).or_default().push(j);
                }
            }
        }
    }
    let mut out = Vec::new();
    let mut cand: Vec<usize> = Vec::new();
    for (i, x) in ba.iter().enumerate() {
        cand.clear();
        cand.extend_from_slice(&big);
        let (lo, hi) = (
            (key(x.min.x), key(x.min.y), key(x.min.z)),
            (key(x.max.x), key(x.max.y), key(x.max.z)),
        );
        if (hi.0 - lo.0 + 1) * (hi.1 - lo.1 + 1) * (hi.2 - lo.2 + 1) > MAX_CELLS {
            cand.extend(0..bb.len());
        } else {
            for ci in lo.0..=hi.0 {
                for cj in lo.1..=hi.1 {
                    for ck in lo.2..=hi.2 {
                        if let Some(l) = grid.get(&(ci, cj, ck)) {
                            cand.extend_from_slice(l);
                        }
                    }
                }
            }
        }
        cand.sort_unstable();
        cand.dedup();
        for &j in &cand {
            if x.intersects(&bb[j]) {
                out.push((i, j));
            }
        }
    }
    out
}
