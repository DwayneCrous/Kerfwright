//! Phase 3: the planar arrangement of every face in its (internal) parameter plane, from which
//! the face fragments are read off as cycles of half-edges.

use crate::model::*;
use crate::BooleanError;
use kw_brep::uvgeom::UV;
use kw_nurbs::{Curve3, CurveGeom};

/// One use of a canonical piece by a face, with the geometry needed to trace and output it.
#[derive(Debug, Clone)]
pub struct FEdge {
    pub piece: usize,
    pub use_idx: usize,
    pub pcurve: CurveGeom,
    /// Parameter range of `pcurve` and whether canonical forward travel runs it backwards.
    pub prange: (f64, f64),
    pub reversed: bool,
    pub boundary_fwd: Option<bool>,
    pub v0: usize,
    pub v1: usize,
    /// Polyline (internal coordinates) from canonical start to end.
    pub poly: Vec<UV>,
}

#[derive(Debug, Clone, Copy)]
pub struct He {
    pub fe: usize,
    pub fwd: bool,
}

#[derive(Debug, Clone)]
pub struct Fragment {
    pub face: usize,
    pub fedges: std::sync::Arc<Vec<FEdge>>,
    pub outer: Vec<He>,
    pub holes: Vec<Vec<He>>,
    /// Interior sample in the face's own (original) parameter coordinates.
    pub sample_uv: UV,
    /// True if this fragment is exactly the original face (no cuts, no splits).
    pub whole: bool,
}

fn internal(sgn: f64, p: UV) -> UV {
    (p.0, sgn * p.1)
}

pub fn cycle_poly_pub(fedges: &[FEdge], cyc: &[He]) -> Vec<UV> {
    cycle_poly(fedges, cyc)
}

fn cycle_poly(fedges: &[FEdge], cyc: &[He]) -> Vec<UV> {
    let mut pts = Vec::new();
    for h in cyc {
        let e = &fedges[h.fe];
        let n = e.poly.len();
        if h.fwd {
            pts.extend(e.poly[..n - 1].iter().copied());
        } else {
            pts.extend(e.poly[1..].iter().rev().copied());
        }
    }
    pts
}

fn signed_area(poly: &[UV]) -> f64 {
    let n = poly.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (poly[i], poly[(i + 1) % n]);
        a += p.0 * q.1 - q.0 * p.1;
    }
    0.5 * a
}

fn winding_at(p: UV, poly: &[UV]) -> i32 {
    kw_brep::uvgeom::winding(p, poly)
}

fn seg_dist(p: UV, a: UV, b: UV) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}

fn clearance(p: UV, polys: &[&[UV]]) -> f64 {
    let mut d = f64::INFINITY;
    for poly in polys {
        let n = poly.len();
        for i in 0..n {
            d = d.min(seg_dist(p, poly[i], poly[(i + 1) % n]));
        }
    }
    d
}

/// A point well inside the region bounded by `outer` minus `holes` (maximin clearance).
pub fn interior_point(outer: &[UV], holes: &[&[UV]]) -> Option<UV> {
    use kw_brep::uvgeom::{LoopIndex, UvLoop};
    use kw_brep::LoopKind;
    let mut lo = (f64::INFINITY, f64::INFINITY);
    let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in outer {
        lo = (lo.0.min(p.0), lo.1.min(p.1));
        hi = (hi.0.max(p.0), hi.1.max(p.1));
    }
    let polys: Vec<&[UV]> = std::iter::once(outer)
        .chain(holes.iter().copied())
        .collect();
    // Spatial indexes keep containment and clearance queries cheap for very long boundaries
    let index = |p: &[UV], kind: LoopKind| {
        LoopIndex::new(&[UvLoop {
            kind,
            points: p.to_vec(),
        }])
    };
    let outer_ix = index(outer, LoopKind::Outer);
    let hole_ix: Vec<LoopIndex> = holes.iter().map(|h| index(h, LoopKind::Inner)).collect();
    let inside = |p: UV| outer_ix.winding(p) != 0 && hole_ix.iter().all(|h| h.winding(p) == 0);
    let clearance = |p: UV| -> f64 {
        std::iter::once(&outer_ix)
            .chain(hole_ix.iter())
            .map(|ix| ix.distance(p))
            .fold(f64::INFINITY, f64::min)
    };
    let mut best: Option<(f64, UV)> = None;
    // clearance can be at most half the smaller bounding-box side
    let extent = (hi.0 - lo.0).min(hi.1 - lo.1).max(1e-12);
    let big = outer.len() > 400;
    for n in [4usize, 10, 24] {
        for i in 0..=n {
            for j in 0..=n {
                let p = (
                    lo.0 + (hi.0 - lo.0) * (i as f64 + 0.5) / (n as f64 + 1.0),
                    lo.1 + (hi.1 - lo.1) * (j as f64 + 0.5) / (n as f64 + 1.0),
                );
                if inside(p) {
                    let c = clearance(p);
                    if best.is_none_or(|b| c > b.0) {
                        best = Some((c, p));
                    }
                }
            }
        }
        // good enough: comfortably away from the boundary relative to the region size
        if best.is_some_and(|b| b.0 > 0.15 * extent) || (big && n >= 10 && best.is_some()) {
            break;
        }
    }
    // thin regions: probe the middle of the band at boundary edges (a ray from the edge midpoint to the
    // nearest boundary on the other side, halved), then just inside the midpoints of the boundary edges
    if best.is_none() || best.unwrap().0 < 1e-3 * (hi.0 - lo.0).max(hi.1 - lo.1) {
        let total: usize = polys.iter().map(|p| p.len()).sum();
        let stride = (total / 64).max(1);
        let mut count = 0usize;
        for poly in &polys {
            let m = poly.len();
            for i in 0..m {
                count += 1;
                if !count.is_multiple_of(stride) {
                    continue;
                }
                let (a, b) = (poly[i], poly[(i + 1) % m]);
                let mid = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let l = dx.hypot(dy);
                if l < 1e-14 {
                    continue;
                }
                for s in [1.0, -1.0] {
                    let d = (-s * dy / l, s * dx / l);
                    // nearest crossing of the ray with any boundary segment (other than the edge itself)
                    let mut tmin = f64::INFINITY;
                    for q in &polys {
                        let n = q.len();
                        for k in 0..n {
                            let (c, e) = (q[k], q[(k + 1) % n]);
                            let (ex, ey) = (e.0 - c.0, e.1 - c.1);
                            let den = d.0 * ey - d.1 * ex;
                            if den.abs() < 1e-18 {
                                continue;
                            }
                            let (wx, wy) = (c.0 - mid.0, c.1 - mid.1);
                            let t = (wx * ey - wy * ex) / den;
                            let u = (wx * d.1 - wy * d.0) / den;
                            if t > 1e-9 * extent && (0.0..=1.0).contains(&u) && t < tmin {
                                tmin = t;
                            }
                        }
                    }
                    if tmin.is_finite() {
                        let p = (mid.0 + d.0 * 0.5 * tmin, mid.1 + d.1 * 0.5 * tmin);
                        if inside(p) {
                            let c = clearance(p);
                            if best.is_none_or(|b| c > b.0) {
                                best = Some((c, p));
                            }
                        }
                    }
                }
            }
        }
    }
    if best.is_none() || best.unwrap().0 < 1e-3 * (hi.0 - lo.0).max(hi.1 - lo.1) {
        for poly in &polys {
            let m = poly.len();
            for i in 0..m {
                let (a, b) = (poly[i], poly[(i + 1) % m]);
                let mid = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let l = dx.hypot(dy);
                if l < 1e-14 {
                    continue;
                }
                for k in [1e-2, 1e-3, 1e-4] {
                    for s in [1.0, -1.0] {
                        let off = k * l.min((hi.0 - lo.0).max(hi.1 - lo.1));
                        let p = (mid.0 - s * dy / l * off, mid.1 + s * dx / l * off);
                        if inside(p) {
                            let c = clearance(p);
                            if best.is_none_or(|b| c > b.0) {
                                best = Some((c, p));
                            }
                        }
                    }
                }
            }
        }
    }
    best.map(|b| b.1)
}

fn sample_poly(pc: &CurveGeom, prange: (f64, f64), reversed: bool, sgn: f64) -> Vec<UV> {
    let n = match pc {
        CurveGeom::Line(_) => 2,
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => {
            ((prange.1 - prange.0).abs() / 0.1).ceil() as usize + 2
        }
        CurveGeom::Nurbs(c) => ((c.knots().distinct().len().max(2) - 1) * 4).clamp(2, 48) + 1,
        CurveGeom::Procedural(_) => 24,
    }
    .max(2);
    let (a, b) = if reversed {
        (prange.1, prange.0)
    } else {
        prange
    };
    (0..=n)
        .map(|i| {
            let p = pc.point_at(a + (b - a) * i as f64 / n as f64);
            internal(sgn, (p.x, p.y))
        })
        .collect()
}

pub fn build(w: &mut Work) -> Result<(), BooleanError> {
    use rayon::prelude::*;
    let nfaces = w.faces.len();
    // faces are independent of each other; results are collected in face order, so the output (and the
    // first error reported) does not depend on the thread count
    let per_face: Vec<Result<Vec<Fragment>, BooleanError>> = {
        let w: &Work = w;
        (0..nfaces)
            .into_par_iter()
            .map(|fi| arrange_face(w, fi))
            .collect()
    };
    for frags in per_face {
        w.fragments.extend(frags?);
    }
    if has_slits(w) {
        w.suspect = true;
    }
    Ok(())
}

/// A fragment that runs along an intersection curve in both directions has a slit: the curve ends inside
/// the face instead of separating it, i.e. a neighbouring cut was lost. Whatever the operation selects,
/// such an arrangement is suspect (it can give a valid solid that is missing volume), so the caller
/// verifies the result before trusting it.
fn has_slits(w: &Work) -> bool {
    for fr in &w.fragments {
        let mut seen: std::collections::HashMap<usize, u8> = std::collections::HashMap::new();
        for cyc in std::iter::once(&fr.outer).chain(fr.holes.iter()) {
            for h in cyc {
                let fe = &fr.fedges[h.fe];
                let piece = &w.pieces[fe.piece];
                if !matches!(w.gedges[piece.gedge].src, GSrc::Cut) {
                    continue;
                }
                let e = seen.entry(fe.piece).or_default();
                *e |= if h.fwd { 1u8 } else { 2u8 };
                if *e == 3 {
                    return true;
                }
            }
        }
    }
    false
}

fn arrange_face(w: &Work, fi: usize) -> Result<Vec<Fragment>, BooleanError> {
    let face = &w.faces[fi];
    let sgn = face.sgn;
    // ---- face edges from canonical pieces
    let mut fedges: Vec<FEdge> = Vec::new();
    for &(pi, ui) in &w.pieces_of_face[fi] {
        let p = &w.pieces[pi];
        let u = &p.uses[ui];
        {
            let (prange, reversed) = (u.range, u.reversed);
            fedges.push(FEdge {
                piece: pi,
                use_idx: ui,
                pcurve: u.pcurve.clone(),
                prange,
                reversed,
                boundary_fwd: u.boundary_fwd,
                v0: p.v0,
                v1: p.v1,
                poly: Vec::new(),
            });
        }
    }
    if fedges.is_empty() {
        return Err(BooleanError::Topology("face without edges".into()));
    }
    // an untouched face has exactly its original coedges as boundary uses
    let untouched = fedges.iter().all(|e| e.boundary_fwd.is_some())
        && fedges.len() == face.loops.iter().map(|l| l.coedges.len()).sum::<usize>();
    // fast path: a face with no cuts, imprints or splits keeps its loops; no arrangement is needed
    if untouched && fedges.iter().all(|e| e.v0 != usize::MAX) {
        let mut loops_he: Vec<Vec<He>> = Vec::new();
        let mut k = 0;
        // fedges are in piece order, not loop order: map each original coedge to its piece's fedge
        let mut by_edge: std::collections::HashMap<(usize, bool), usize> =
            std::collections::HashMap::new();
        for (k2, e) in fedges.iter().enumerate() {
            if let (Some(f), false) = (e.boundary_fwd, e.reversed) {
                by_edge.insert((w.pieces[e.piece].gedge, f), k2);
            }
        }
        let mut ok = true;
        for l in &face.loops {
            let mut cyc: Vec<He> = Vec::with_capacity(l.coedges.len());
            for c in &l.coedges {
                // the piece carrying this coedge: the unique fedge whose gedge is the coedge's edge
                let gi = w.gedge_of_edge.get(&c.edge).copied();
                let Some(gi) = gi else {
                    ok = false;
                    break;
                };
                let Some(&fe) = by_edge.get(&(gi, c.fwd)) else {
                    ok = false;
                    break;
                };
                let piece = &w.pieces[fedges[fe].piece];
                let _ = piece;
                // the half-edge runs along the original loop direction; `fwd` is relative to the canonical piece direction
                let canon_fwd = !fedges[fe].reversed_vs_gedge(w);
                cyc.push(He {
                    fe,
                    fwd: c.fwd == canon_fwd,
                });
                k += 1;
            }
            if !ok {
                break;
            }
            loops_he.push(cyc);
        }
        if ok && k == fedges.len() {
            let sample = {
                let outer_poly: Vec<UV> = face.uv_loops[0]
                    .points
                    .iter()
                    .map(|p| internal(sgn, *p))
                    .collect();
                let holes: Vec<Vec<UV>> = face.uv_loops[1..]
                    .iter()
                    .map(|l| l.points.iter().map(|p| internal(sgn, *p)).collect())
                    .collect();
                let hrefs: Vec<&[UV]> = holes.iter().map(|h| h.as_slice()).collect();
                interior_point(&outer_poly, &hrefs)
            };
            if let Some(sm) = sample {
                let mut it = loops_he.into_iter();
                let outer = it.next().unwrap();
                return Ok(vec![Fragment {
                    face: fi,
                    fedges: std::sync::Arc::new(fedges),
                    outer,
                    holes: it.collect(),
                    sample_uv: (sm.0, sgn * sm.1),
                    whole: true,
                }]);
            }
        }
    }
    for e in fedges.iter_mut() {
        e.poly = sample_poly(&e.pcurve, e.prange, e.reversed, sgn);
    }
    // ---- nodes
    #[derive(Clone)]
    struct Node {
        uv: UV,
    }
    let mut nodes: Vec<Node> = Vec::new();
    let mut node_index: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    let mut node_of = |vid: usize, uv: UV, nodes: &mut Vec<Node>| -> usize {
        if let Some(list) = node_index.get(&vid) {
            for &i in list {
                let n = &nodes[i];
                if (n.uv.0 - uv.0).hypot(n.uv.1 - uv.1) < 1e-6 * (1.0 + uv.0.abs().max(uv.1.abs()))
                {
                    return i;
                }
            }
        }
        nodes.push(Node { uv });
        node_index.entry(vid).or_default().push(nodes.len() - 1);
        nodes.len() - 1
    };
    // half-edge arrays: he id = 2*fe (fwd) / 2*fe+1 (backward)
    let nhe = fedges.len() * 2;
    let mut from = vec![0usize; nhe];
    let mut to = vec![0usize; nhe];
    let mut angle = vec![0.0f64; nhe];
    let mut bend = vec![0.0f64; nhe];
    // characteristic uv size for secant look-ahead
    let uv_size = fedges
        .iter()
        .flat_map(|e| e.poly.iter())
        .fold(1e-9f64, |m, p| m.max(p.0.abs()).max(p.1.abs()));
    let delta = 2e-3 * uv_size.min(10.0);
    for (k, e) in fedges.iter().enumerate() {
        let (a, b) = (e.poly[0], *e.poly.last().unwrap());
        let na = node_of(e.v0, a, &mut nodes);
        let nb = node_of(e.v1, b, &mut nodes);
        from[2 * k] = na;
        to[2 * k] = nb;
        from[2 * k + 1] = nb;
        to[2 * k + 1] = na;
        // leaving directions from short secants (robust to zero derivatives)
        let dir = |poly: &[UV], rev: bool| -> f64 {
            let pts: Vec<UV> = if rev {
                poly.iter().rev().copied().collect()
            } else {
                poly.to_vec()
            };
            let p0 = pts[0];
            let mut acc = 0.0;
            let mut seen = 0.0f64;
            let mut prev = p0;
            for q in pts.iter().skip(1) {
                acc += (q.0 - prev.0).hypot(q.1 - prev.1);
                prev = *q;
                if acc > 1e-9 {
                    let d = (q.1 - p0.1).atan2(q.0 - p0.0);
                    seen = d;
                    break;
                }
            }
            seen
        };
        // refine with the analytic tangent when available (polyline secants are coarse for arcs)
        let tangent = |at_start: bool| -> Option<f64> {
            let t = if (at_start && !e.reversed) || (!at_start && e.reversed) {
                e.prange.0
            } else {
                e.prange.1
            };
            let d = e.pcurve.derivative(t);
            if d.x.hypot(d.y) < 1e-12 {
                return None;
            }
            // direction of canonical travel: param increases unless reversed
            let travel = if e.reversed { -1.0 } else { 1.0 };
            let (dx, dy) = (d.x * travel, d.y * travel * sgn);
            // at the start we leave along +travel; at the end, the backward half-edge leaves along −travel
            let (dx, dy) = if at_start { (dx, dy) } else { (-dx, -dy) };
            Some(dy.atan2(dx))
        };
        // directions exactly along −x come out as ±π depending on the sign of a zero: fold them together
        let fold = |a: f64| {
            if a <= -std::f64::consts::PI + 1e-6 {
                a + std::f64::consts::TAU
            } else {
                a
            }
        };
        angle[2 * k] = fold(tangent(true).unwrap_or_else(|| dir(&e.poly, false)));
        angle[2 * k + 1] = fold(tangent(false).unwrap_or_else(|| dir(&e.poly, true)));
        // second-order information for tangent ties: signed turn of a short secant relative to the tangent
        let bend_of = |at_start: bool| -> f64 {
            let t = if (at_start && !e.reversed) || (!at_start && e.reversed) {
                e.prange.0
            } else {
                e.prange.1
            };
            let d = e.pcurve.derivative(t);
            let speed = d.x.hypot(d.y);
            if speed < 1e-12 {
                return 0.0;
            }
            // travel direction along the pcurve parameter for this half-edge
            let along = if at_start {
                if e.reversed {
                    -1.0
                } else {
                    1.0
                }
            } else if e.reversed {
                1.0
            } else {
                -1.0
            };
            let p0 = e.pcurve.point_at(t);
            let q = e.pcurve.point_at(t + along * delta / speed);
            let (dx, dy) = (q.x - p0.x, q.y - p0.y);
            let sec = (dy * sgn).atan2(dx);
            let tan = if at_start {
                angle[2 * k]
            } else {
                angle[2 * k + 1]
            };
            let mut b = sec - tan;
            while b > std::f64::consts::PI {
                b -= std::f64::consts::TAU;
            }
            while b < -std::f64::consts::PI {
                b += std::f64::consts::TAU;
            }
            b
        };
        bend[2 * k] = bend_of(true);
        bend[2 * k + 1] = bend_of(false);
    }
    // outgoing lists sorted CCW
    let mut out: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for h in 0..nhe {
        out[from[h]].push(h);
    }
    let mut pos_in_out = vec![0usize; nhe];
    for list in &mut out {
        list.sort_by(|&a, &b| {
            let da = angle[a] - angle[b];
            // equal tangent directions (tangent blends): the curve bending further counter-clockwise comes later
            if da.abs() < 1e-6 {
                bend[a]
                    .partial_cmp(&bend[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            } else {
                angle[a]
                    .partial_cmp(&angle[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            }
        });
        for (i, &h) in list.iter().enumerate() {
            pos_in_out[h] = i;
        }
    }
    let next = |h: usize| -> usize {
        let twin = h ^ 1;
        let n = to[h];
        let list = &out[n];
        let p = pos_in_out[twin];
        list[(p + list.len() - 1) % list.len()]
    };
    // ---- trace cycles
    let mut visited = vec![false; nhe];
    let mut cycles: Vec<Vec<He>> = Vec::new();
    for s in 0..nhe {
        if visited[s] {
            continue;
        }
        let mut cyc = Vec::new();
        let mut h = s;
        loop {
            visited[h] = true;
            cyc.push(He {
                fe: h / 2,
                fwd: h % 2 == 0,
            });
            h = next(h);
            if h == s {
                break;
            }
            if visited[h] || cyc.len() > nhe {
                return Err(BooleanError::Topology(
                    "half-edge cycle did not close".into(),
                ));
            }
        }
        cycles.push(cyc);
    }
    let polys: Vec<Vec<UV>> = cycles.iter().map(|c| cycle_poly(&fedges, c)).collect();
    let areas: Vec<f64> = polys.iter().map(|p| signed_area(p)).collect();
    let mut scale = 0.0f64;
    for p in &polys {
        for q in p {
            scale = scale.max(q.0.abs()).max(q.1.abs());
        }
    }
    let atol = 1e-12 * (1.0 + scale * scale);
    let pos: Vec<usize> = (0..cycles.len()).filter(|&i| areas[i] > atol).collect();
    let neg: Vec<usize> = (0..cycles.len()).filter(|&i| areas[i] < -atol).collect();
    // ---- assign negative cycles (component outer boundaries) to the smallest enclosing positive cycle
    let mut holes_of: Vec<Vec<usize>> = vec![Vec::new(); cycles.len()];
    for &n in &neg {
        // a vertex of the negative cycle that is not on a positive cycle's boundary
        let probe = polys[n][0];
        let mut best: Option<(f64, usize)> = None;
        for &p in &pos {
            if winding_at(probe, &polys[p]) != 0
                && clearance(probe, &[&polys[p]]) > 1e-9 * (1.0 + scale)
                && best.is_none_or(|b| areas[p] < b.0)
            {
                best = Some((areas[p], p));
            }
        }
        if let Some((_, p)) = best {
            holes_of[p].push(n);
        }
    }
    if std::env::var("KW_BOOL_DEBUG2").is_ok()
        && std::env::var("KW_BOOL_DEBUG2").unwrap() == fi.to_string()
    {
        eprintln!(
            "ARRANGE face {fi}: {} fedges, {} cycles, pos {:?} neg {:?}",
            fedges.len(),
            cycles.len(),
            pos,
            neg
        );
        for (i, c) in cycles.iter().enumerate() {
            eprintln!(
                "   cycle {i}: area {:.6e} len {} pieces {:?}",
                areas[i],
                c.len(),
                c.iter()
                    .map(|h| (fedges[h.fe].piece, h.fwd))
                    .collect::<Vec<_>>()
            );
        }
        for (k, e) in fedges.iter().enumerate() {
            eprintln!(
                "   fedge {k}: piece {} v {}->{} bnd {:?} start {:?} end {:?}",
                e.piece,
                e.v0,
                e.v1,
                e.boundary_fwd,
                e.poly[0],
                e.poly.last().unwrap()
            );
        }
    }
    // ---- fragments
    let fedges_rc = std::sync::Arc::new(fedges);
    let mut frags = Vec::new();
    for &p in &pos {
        let holes: Vec<&[UV]> = holes_of[p].iter().map(|&h| polys[h].as_slice()).collect();
        let Some(sample) = interior_point(&polys[p], &holes) else {
            continue;
        };
        if !face.loop_index.contains((sample.0, sgn * sample.1)) {
            continue;
        }
        let whole = untouched;
        frags.push(Fragment {
            face: fi,
            fedges: fedges_rc.clone(),
            outer: cycles[p].clone(),
            holes: holes_of[p].iter().map(|&h| cycles[h].clone()).collect(),
            sample_uv: (sample.0, sgn * sample.1),
            whole,
        });
    }
    if frags.is_empty() {
        if std::env::var("KW_BOOL_DEBUG").is_ok() {
            eprintln!(
                "face {fi}: no fragments; {} fedges, {} cycles",
                fedges_rc.len(),
                cycles.len()
            );
            for (i, c) in cycles.iter().enumerate() {
                eprintln!(
                    "   cycle {i}: len {} area {:.6e} pieces {:?}",
                    c.len(),
                    areas[i],
                    c.iter()
                        .map(|h| (fedges_rc[h.fe].piece, h.fwd))
                        .collect::<Vec<_>>()
                );
            }
            for (k, e) in fedges_rc.iter().enumerate() {
                eprintln!(
                    "   fedge {k}: piece {} v {}->{} boundary {:?} poly_start {:?} poly_end {:?}",
                    e.piece,
                    e.v0,
                    e.v1,
                    e.boundary_fwd,
                    e.poly[0],
                    e.poly.last().unwrap()
                );
            }
        }
        return Err(BooleanError::Topology(
            "arrangement produced no fragments".into(),
        ));
    }
    Ok(frags)
}

impl FEdge {
    /// Does the use's pcurve run against the direction of its piece's *gedge*? (canonical piece direction == gedge direction)
    fn reversed_vs_gedge(&self, _w: &Work) -> bool {
        self.reversed
    }
}
