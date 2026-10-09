//! Phase 2: global edges, the vertex registry, edge splitting into pieces, imprinting of
//! coincident-face boundaries, and merging of coincident pieces.

use crate::cuts::{contains, Cont};
use crate::model::*;
use crate::BooleanError;
use kw_math::{Aabb, Point3, Vector3};
use kw_nurbs::intersect::curve_curve;
use kw_nurbs::pcurve::pcurve_on;
use kw_nurbs::proj::curve_param_of;
use kw_nurbs::{Curve3, CurveGeom};
use std::collections::HashMap;
use std::f64::consts::TAU;

fn period(c: &CurveGeom) -> Option<f64> {
    match c {
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => Some(TAU),
        _ => None,
    }
}

fn coarse_polyline(c: Option<&CurveGeom>, r: (f64, f64)) -> (Vec<Point3<f64>>, f64) {
    let Some(c) = c else { return (Vec::new(), 0.0) };
    if let CurveGeom::Line(_) = c {
        // a segment is its own polyline: a handful of points keep the quick-reject test meaningful
        let n = 8;
        let pts: Vec<Point3<f64>> = (0..=n)
            .map(|i| c.point_at(r.0 + (r.1 - r.0) * i as f64 / n as f64))
            .collect();
        let gap = (pts[1] - pts[0]).norm();
        return (pts, gap);
    }
    let n = 96;
    let pts: Vec<Point3<f64>> = (0..=n)
        .map(|i| c.point_at(r.0 + (r.1 - r.0) * i as f64 / n as f64))
        .collect();
    let gap = pts
        .windows(2)
        .map(|w| (w[1] - w[0]).norm())
        .fold(0.0f64, f64::max);
    (pts, gap)
}

fn curve_bbox(c: &CurveGeom, r: (f64, f64), pad: f64) -> Aabb {
    let n = if matches!(c, CurveGeom::Line(_)) {
        1
    } else {
        64
    };
    let pts: Vec<Point3<f64>> = (0..=n)
        .map(|i| c.point_at(r.0 + (r.1 - r.0) * i as f64 / n as f64))
        .collect();
    let mut b = Aabb::from_points(&pts);
    let p = pad + 0.02 * b.diagonal();
    b.min -= Vector3::repeat(p);
    b.max += Vector3::repeat(p);
    b
}

/// Parameter of `p` on the gedge's curve if the point lies on it strictly inside the range.
fn interior_param(g: &GEdge, p: &Point3<f64>, tol: f64) -> Option<f64> {
    let c = g.curve.as_ref()?;
    if !g.bbox.contains(p, 0.0) {
        return None;
    }
    // quick rejection against the coarse polyline
    if !g.coarse.is_empty() {
        let reach = 1.5 * g.coarse_gap + 4.0 * tol;
        if g.coarse.iter().all(|q| (q - p).norm() > reach) {
            return None;
        }
    }
    let mid = 0.5 * (g.range.0 + g.range.1);
    let r = curve_param_of(c, p, Some(mid));
    if r.distance > 2.0 * tol {
        return None;
    }
    let mut t = r.t;
    if let Some(per) = period(c) {
        // bring into [range.0, range.0 + per)
        t = g.range.0 + (t - g.range.0).rem_euclid(per);
    }
    let eps = 1e-9 * (1.0 + (g.range.1 - g.range.0).abs());
    (t > g.range.0 + eps && t < g.range.1 - eps).then_some(t)
}

pub fn build(w: &mut Work) -> Result<(), BooleanError> {
    let tol = w.tol;
    // ---- 1. global edges for every original edge
    let mut ids: Vec<_> = w.edges.keys().copied().collect();
    ids.sort();
    let mut edge_g = HashMap::new();
    // bounding boxes and coarse polylines are independent per edge
    let geo: Vec<(Aabb, Vec<Point3<f64>>, f64)> = {
        use rayon::prelude::*;
        let edges = &w.edges;
        ids.par_iter()
            .map(|id| {
                let e = &edges[id];
                let bbox = match &e.curve {
                    Some(c) => curve_bbox(c, e.range, tol),
                    None => Aabb::from_points(&[e.p0]),
                };
                let (coarse, gap) = coarse_polyline(e.curve.as_ref(), e.range);
                (bbox, coarse, gap)
            })
            .collect()
    };
    for (id, (bbox, coarse, coarse_gap)) in ids.into_iter().zip(geo) {
        let e = w.edges[&id].clone();
        let (vs, ve) = (w.reg.add(e.p0), w.reg.add(e.p1));
        edge_g.insert(id, w.gedges.len());
        w.gedges.push(GEdge {
            coarse,
            coarse_gap,
            curve: e.curve.clone(),
            range: e.range,
            closed: e.v0 == e.v1,
            src: GSrc::Orig(id),
            uses: Vec::new(),
            bbox,
            v_start: vs,
            v_end: ve,
            splits: Vec::new(),
        });
    }
    for fi in 0..w.faces.len() {
        let mut uses = Vec::new();
        for l in &w.faces[fi].loops {
            for c in &l.coedges {
                let gi = edge_g[&c.edge];
                uses.push((
                    gi,
                    GUse {
                        face: fi,
                        defined_on: gi,
                        pcurve: c.pcurve.clone(),
                        range: (0.0, 0.0),
                        reversed: false,
                        boundary_fwd: Some(c.fwd),
                    },
                ));
            }
        }
        for (gi, u) in uses {
            w.gedges[gi].uses.push(u);
        }
    }
    // ---- 2. cut edges
    w.gedge_of_edge = edge_g.clone();
    let first_cut = w.gedges.len();
    for c in w.cuts.clone() {
        let (p0, p1) = (c.curve.point_at(c.range.0), c.curve.point_at(c.range.1));
        let (vs, ve) = (w.reg.add(p0), w.reg.add(p1));
        let gi = w.gedges.len();
        let (coarse, coarse_gap) = coarse_polyline(Some(&c.curve), c.range);
        w.gedges.push(GEdge {
            coarse,
            coarse_gap,
            bbox: curve_bbox(&c.curve, c.range, tol),
            curve: Some(c.curve.clone()),
            range: c.range,
            closed: vs == ve,
            src: GSrc::Cut,
            uses: vec![
                GUse {
                    face: c.fa,
                    defined_on: gi,
                    pcurve: c.pcurve_a.clone(),
                    range: (0.0, 0.0),
                    reversed: false,
                    boundary_fwd: None,
                },
                GUse {
                    face: c.fb,
                    defined_on: gi,
                    pcurve: c.pcurve_b.clone(),
                    range: (0.0, 0.0),
                    reversed: false,
                    boundary_fwd: None,
                },
            ],
            v_start: vs,
            v_end: ve,
            splits: Vec::new(),
        });
    }
    if std::env::var("KW_BOOL_DEBUG").is_ok() {
        eprintln!("vertices after step 2: {}", w.reg.pts.len());
    }
    // ---- 3. extra vertices: cut/cut crossings inside a face, and edge/edge crossings of coincident faces
    let mut cuts_of_face: Vec<Vec<usize>> = vec![Vec::new(); w.faces.len()];
    for g in first_cut..w.gedges.len() {
        for u in &w.gedges[g].uses {
            cuts_of_face[u.face].push(g);
        }
    }
    for mine in cuts_of_face.iter() {
        if mine.len() < 2 {
            continue;
        }
        let boxes: Vec<kw_math::Aabb> = mine.iter().map(|g| w.gedges[*g].bbox).collect();
        let grid = crate::spatial::GridIndex::build(&boxes);
        let mut cand: Vec<u32> = Vec::new();
        for a in 0..mine.len() {
            grid.query(&boxes[a], mine.len(), &mut cand);
            for &b in &cand {
                let b = b as usize;
                if b <= a {
                    continue;
                }
                let (ga, gb) = (&w.gedges[mine[a]], &w.gedges[mine[b]]);
                if !ga.bbox.intersects(&gb.bbox) {
                    continue;
                }
                let hits = curve_curve(
                    ga.curve.as_ref().unwrap(),
                    ga.range,
                    gb.curve.as_ref().unwrap(),
                    gb.range,
                    tol * 0.5,
                );
                let hits = crate::cuts::coalesce_overlaps(
                    hits,
                    ga.curve.as_ref().unwrap(),
                    ga.range,
                    gb.curve.as_ref().unwrap(),
                    gb.range,
                    tol,
                );
                for h in hits {
                    w.reg.add(h.point);
                }
            }
        }
    }
    // probe the edge index of the face with more edges (built once per face in the cut phase)
    for (fa, fb, _) in w.coincident.clone() {
        let (big_f, small_f) = if w.face_edge_ids[fa].len() >= w.face_edge_ids[fb].len() {
            (fa, fb)
        } else {
            (fb, fa)
        };
        let mut hits_at: Vec<Point3<f64>> = Vec::new();
        {
            let big_ids = &w.face_edge_ids[big_f];
            let mut cand: Vec<u32> = Vec::new();
            for sid in &w.face_edge_ids[small_f] {
                let x = &w.edges[sid];
                match &w.face_edge_grid[big_f] {
                    Some(grid) => grid.query(&x.bbox, big_ids.len(), &mut cand),
                    None => {
                        cand.clear();
                        cand.extend(0..big_ids.len() as u32);
                    }
                }
                for &j in &cand {
                    let y = &w.edges[&big_ids[j as usize]];
                    if !x.bbox.intersects(&y.bbox) {
                        continue;
                    }
                    if let (Some(cx), Some(cy)) = (&x.curve, &y.curve) {
                        let hits = curve_curve(cx, x.range, cy, y.range, tol * 0.5);
                        for h in crate::cuts::coalesce_overlaps(hits, cx, x.range, cy, y.range, tol)
                        {
                            hits_at.push(h.point);
                        }
                    }
                }
            }
        }
        for p in hits_at {
            w.reg.add(p);
        }
    }
    // ---- 4. split every edge at registry vertices lying on it (candidates from a sort-by-x index)
    let npts = w.reg.pts.len();
    let mut by_x: Vec<usize> = (0..npts).collect();
    by_x.sort_by(|a, b| {
        w.reg.pts[*a]
            .x
            .partial_cmp(&w.reg.pts[*b].x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let xs: Vec<f64> = by_x.iter().map(|i| w.reg.pts[*i].x).collect();
    for gi in 0..w.gedges.len() {
        let g = &w.gedges[gi];
        let slack = 2.0 * tol;
        let lo = xs.partition_point(|x| *x < g.bbox.min.x - slack);
        let hi = xs.partition_point(|x| *x <= g.bbox.max.x + slack);
        let mut splits: Vec<(f64, usize)> = Vec::new();
        for &vi in &by_x[lo..hi] {
            if vi == g.v_start || vi == g.v_end {
                continue;
            }
            if let Some(t) = interior_param(g, &w.reg.pts[vi], tol) {
                splits.push((t, vi));
            }
        }
        splits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        splits.dedup_by(|a, b| a.1 == b.1);
        w.gedges[gi].splits = splits;
    }
    // ---- 5. pieces
    for gi in 0..w.gedges.len() {
        let g = &w.gedges[gi];
        let mut stops: Vec<(f64, usize)> = vec![(g.range.0, g.v_start)];
        stops.extend(g.splits.iter().copied());
        stops.push((g.range.1, g.v_end));
        for k in 0..stops.len() - 1 {
            let idx = w.pieces.len();
            let range = (stops[k].0, stops[k + 1].0);
            w.pieces.push(Piece {
                gedge: gi,
                range,
                v0: stops[k].1,
                v1: stops[k + 1].1,
                uses: g.uses.iter().map(|u| GUse { range, ..u.clone() }).collect(),
                canon: idx,
                flipped: false,
            });
        }
    }
    imprint(w)?;
    canonicalize(w);
    // per-face index of canonical piece uses
    w.pieces_of_face = vec![Vec::new(); w.faces.len()];
    for (pi, p) in w.pieces.iter().enumerate() {
        if p.canon != pi {
            continue;
        }
        for (ui, u) in p.uses.iter().enumerate() {
            w.pieces_of_face[u.face].push((pi, ui));
        }
    }
    Ok(())
}

/// Boundary pieces of one face that lie inside a coincident partner face become cuts there.
fn imprint(w: &mut Work) -> Result<(), BooleanError> {
    let tol = w.tol;
    let coin = w.coincident.clone();
    for (fa, fb, _) in coin {
        for (src, dst) in [(fb, fa), (fa, fb)] {
            for pi in 0..w.pieces.len() {
                let p = &w.pieces[pi];
                let g = &w.gedges[p.gedge];
                if g.curve.is_none() || !matches!(g.src, GSrc::Orig(_)) {
                    continue;
                }
                if !p
                    .uses
                    .iter()
                    .any(|u| u.face == src && u.boundary_fwd.is_some())
                {
                    continue;
                }
                if p.uses.iter().any(|u| u.face == dst) {
                    continue;
                }
                let curve = g.curve.clone().unwrap();
                let mid = curve.point_at(0.5 * (p.range.0 + p.range.1));
                if contains(&w.faces[dst], &mid, tol) == Cont::Out {
                    continue;
                }
                // the piece must also lie on the partner surface (coincident surfaces: always)
                let hint = w.faces[dst].centre();
                let pc = pcurve_on(
                    &w.faces[dst].surface,
                    &curve,
                    p.range,
                    Some(hint),
                    1e-9 * (1.0 + w.scale),
                )
                .ok_or(BooleanError::Unsupported(
                    "could not project a boundary onto a coincident face",
                ))?;
                let pc = shift_into(&w.faces[dst], &pc, 0.5 * (p.range.0 + p.range.1));
                let gi = p.gedge;
                let range = w.pieces[pi].range;
                w.pieces[pi].uses.push(GUse {
                    face: dst,
                    defined_on: gi,
                    pcurve: pc,
                    range,
                    reversed: false,
                    boundary_fwd: None,
                });
            }
        }
    }
    Ok(())
}

fn shift_into(f: &FaceRec, pc: &CurveGeom, t: f64) -> CurveGeom {
    use kw_nurbs::Surface;
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
        kw_nurbs::fit::translate_pcurve(pc, du, dv)
    }
}

/// Do two pieces (same end vertices) trace the same geometric curve?
fn same_geometry(w: &Work, p: &Piece, q: &Piece) -> bool {
    let (gp, gq) = (&w.gedges[p.gedge], &w.gedges[q.gedge]);
    let (Some(cp), Some(cq)) = (&gp.curve, &gq.curve) else {
        return false;
    };
    let tol = 4.0 * w.tol;
    for f in [0.2, 0.5, 0.8] {
        let pt = cp.point_at(p.range.0 + (p.range.1 - p.range.0) * f);
        let mid = 0.5 * (q.range.0 + q.range.1);
        let r = curve_param_of(cq, &pt, Some(mid));
        if r.distance > tol {
            return false;
        }
        // the foot must lie within q's range (unwrapped for periodic curves)
        let mut t = r.t;
        if let Some(per) = period(cq) {
            t = q.range.0 + (t - q.range.0).rem_euclid(per);
        }
        let eps = 1e-6 * (1.0 + (q.range.1 - q.range.0).abs());
        if t < q.range.0.min(q.range.1) - eps || t > q.range.0.max(q.range.1) + eps {
            return false;
        }
    }
    true
}

fn canonicalize(w: &mut Work) {
    let mut groups: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (i, p) in w.pieces.iter().enumerate() {
        if w.gedges[p.gedge].curve.is_none() {
            continue;
        }
        groups
            .entry((p.v0.min(p.v1), p.v0.max(p.v1)))
            .or_default()
            .push(i);
    }
    let mut keys: Vec<_> = groups.keys().copied().collect();
    keys.sort();
    for k in keys {
        let list = groups[&k].clone();
        if list.len() < 2 {
            continue;
        }
        let mut reps: Vec<usize> = Vec::new();
        for &q in &list {
            let mut merged = false;
            for &p in &reps {
                if same_geometry(w, &w.pieces[p], &w.pieces[q]) {
                    let (pp, qq) = (w.pieces[p].clone(), w.pieces[q].clone());
                    let flipped = if pp.v0 != pp.v1 {
                        qq.v0 != pp.v0
                    } else {
                        // closed: compare tangent directions at the start
                        let (gp, gq) = (&w.gedges[pp.gedge], &w.gedges[qq.gedge]);
                        let dp = gp.curve.as_ref().unwrap().derivative(pp.range.0);
                        let dq = gq.curve.as_ref().unwrap().derivative(qq.range.0);
                        dp.dot(&dq) < 0.0
                    };
                    w.pieces[q].canon = p;
                    w.pieces[q].flipped = flipped;
                    let moved: Vec<GUse> = qq
                        .uses
                        .iter()
                        .map(|u| GUse {
                            boundary_fwd: u.boundary_fwd.map(|f| f ^ flipped),
                            reversed: u.reversed ^ flipped,
                            ..u.clone()
                        })
                        .collect();
                    w.pieces[p].uses.extend(moved);
                    merged = true;
                    break;
                }
            }
            if !merged {
                reps.push(q);
            }
        }
    }
    // drop redundant uses: an imprint/cut use on a face that already has a boundary use, duplicates
    for p in &mut w.pieces {
        if p.canon != usize::MAX {
            let uses = std::mem::take(&mut p.uses);
            let mut out: Vec<GUse> = Vec::new();
            for u in uses {
                if u.boundary_fwd.is_none() {
                    if out.iter().any(|o| o.face == u.face) {
                        continue;
                    }
                } else {
                    out.retain(|o| !(o.face == u.face && o.boundary_fwd.is_none()));
                }
                out.push(u);
            }
            p.uses = out;
        }
    }
}
