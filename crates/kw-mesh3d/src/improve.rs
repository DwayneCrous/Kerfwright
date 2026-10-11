//! Topological mesh improvement: edge removal (the 3-2 flip and its generalisations: a ring of 3…7 tets
//! round an interior edge is replaced by the best triangulation of the ring's polygon) and face removal
//! (the 2-3 flip), accepted when they raise the worst mean-ratio quality of the tets involved. Boundary
//! faces and vertices never change. Slivers — four nearly cocircular points — are the typical victims:
//! a Delaunay-refined mesh has them, and a flip turns them into well-shaped tets.

use crate::smooth::mean_ratio;
use crate::tri3::{orient, Tri3, NONE};
use crate::P3;

#[derive(Debug, Clone, Copy, Default)]
pub struct ImproveStats {
    pub edge_removals: usize,
    pub face_removals: usize,
    pub rounds: usize,
}

const MAX_RING: usize = 7;

fn quality(pts: &[P3], t: &[u32; 4]) -> f64 {
    mean_ratio(pts, t)
}

/// Orient `[a,b,c,d]` so its signed volume is positive; `None` if it is flat.
fn positive(pts: &[P3], mut t: [u32; 4]) -> Option<[u32; 4]> {
    let o = orient(
        &pts[t[0] as usize],
        &pts[t[1] as usize],
        &pts[t[2] as usize],
        &pts[t[3] as usize],
    );
    if o > 0.0 {
        return Some(t);
    }
    if o < 0.0 {
        t.swap(2, 3);
        return Some(t);
    }
    None
}

/// Improve `tets` (positively oriented) in place; returns what was done.
pub fn improve(
    nodes: &[P3],
    tets: &mut Vec<[u32; 4]>,
    target: f64,
    max_rounds: usize,
) -> ImproveStats {
    let mut tri = Tri3::from_tets(nodes.to_vec(), tets);
    let mut stats = ImproveStats::default();
    for round in 0..max_rounds {
        stats.rounds = round + 1;
        // the tets below the target quality, worst first
        let mut work: Vec<(f64, u32)> = (0..tri.tets.len() as u32)
            .filter(|&t| tri.tets[t as usize].alive)
            .map(|t| (quality(&tri.pts, &tri.tets[t as usize].v), t))
            .filter(|(q, _)| *q < target)
            .collect();
        work.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut changed = 0;
        for (_, t) in work {
            if !tri.tets[t as usize].alive {
                continue;
            }
            // quality may have changed since the list was made
            let q0 = quality(&tri.pts, &tri.tets[t as usize].v);
            if q0 >= target {
                continue;
            }
            if try_edges(&mut tri, t, &mut stats) || try_faces(&mut tri, t, &mut stats) {
                changed += 1;
            }
        }
        if changed == 0 {
            break;
        }
    }
    tets.clear();
    for t in &tri.tets {
        if t.alive {
            tets.push(t.v);
        }
    }
    stats
}

/// Try to remove each interior edge of tet `t`; true if one was.
fn try_edges(tri: &mut Tri3, t: u32, stats: &mut ImproveStats) -> bool {
    let v = tri.tets[t as usize].v;
    let mut best: Option<(f64, (u32, u32))> = None;
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for i in 0..4 {
        for j in i + 1..4 {
            edges.push((v[i], v[j]));
        }
    }
    // the edge removal that improves the worst quality the most
    let mut plan: Option<(f64, Vec<u32>, Vec<[u32; 4]>)> = None;
    for (a, b) in edges {
        let Some((fan, ring)) = closed_ring(tri, a, b) else {
            continue;
        };
        let old_min = fan
            .iter()
            .map(|&f| quality(&tri.pts, &tri.tets[f as usize].v))
            .fold(f64::INFINITY, f64::min);
        if let Some((q, new)) = best_triangulation(&tri.pts, a, b, &ring) {
            if q > old_min + 1e-9 && plan.as_ref().is_none_or(|p| q > p.0) {
                plan = Some((q, fan, new));
                best = Some((q, (a, b)));
            }
        }
    }
    let _ = best;
    if let Some((_, fan, new)) = plan {
        let old_v: f64 = fan
            .iter()
            .map(|&f| crate::signed_volume(&tri.pts, &tri.tets[f as usize].v))
            .sum();
        let new_v: f64 = new.iter().map(|t| crate::signed_volume(&tri.pts, t)).sum();
        if (old_v - new_v).abs() > 1e-10 * old_v.abs() {
            return false;
        }
        if std::env::var("KW_IMPROVE_CHECK").is_ok() {
            let vol = |ts: &mut dyn Iterator<Item = [u32; 4]>| -> f64 {
                ts.map(|t| crate::signed_volume(&tri.pts, &t)).sum()
            };
            let old_v = vol(&mut fan.iter().map(|&f| tri.tets[f as usize].v));
            let new_v = vol(&mut new.iter().copied());
            if (old_v - new_v).abs() > 1e-12 {
                eprintln!(
                    "edge removal changes the volume: k={} old {old_v:e} new {new_v:e}",
                    fan.len()
                );
                if fan.len() == 3 {
                    for &f in &fan {
                        let tv = tri.tets[f as usize].v;
                        eprintln!(
                            "  old tet {tv:?} vol {:e}",
                            crate::signed_volume(&tri.pts, &tv)
                        );
                    }
                    for t in &new {
                        eprintln!(
                            "  new tet {t:?} vol {:e}",
                            crate::signed_volume(&tri.pts, t)
                        );
                    }
                }
            }
        }
        tri.retet(&fan, &new);
        stats.edge_removals += 1;
        return true;
    }
    false
}

/// Try the 2-3 flip across each interior face of tet `t`.
fn try_faces(tri: &mut Tri3, t: u32, stats: &mut ImproveStats) -> bool {
    let tet = tri.tets[t as usize];
    let mut plan: Option<(f64, [u32; 2], Vec<[u32; 4]>)> = None;
    for i in 0..4 {
        let nb = tet.n[i];
        if nb == NONE {
            continue;
        }
        let f = tri.face_ids(t, i);
        let d = tet.v[i];
        let Some(e) = tri.tets[nb as usize]
            .v
            .iter()
            .copied()
            .find(|x| !f.contains(x))
        else {
            continue;
        };
        let p = &tri.pts;
        // the segment d–e must pass through the interior of the face: the three tets round it are positive
        let mut new: Vec<[u32; 4]> = Vec::with_capacity(3);
        let mut sign = 0.0f64;
        let mut ok = true;
        for k in 0..3 {
            let (a, b) = (f[k], f[(k + 1) % 3]);
            let o = orient(
                &p[a as usize],
                &p[b as usize],
                &p[d as usize],
                &p[e as usize],
            );
            if o == 0.0 || (sign != 0.0 && o.signum() != sign) {
                ok = false;
                break;
            }
            sign = o.signum();
            new.push(if o > 0.0 { [a, b, d, e] } else { [b, a, d, e] });
        }
        if !ok {
            continue;
        }
        let new_min = new
            .iter()
            .map(|x| quality(p, x))
            .fold(f64::INFINITY, f64::min);
        let old_min = quality(p, &tet.v).min(quality(p, &tri.tets[nb as usize].v));
        if new_min > old_min + 1e-9 && plan.as_ref().is_none_or(|q| new_min > q.0) {
            plan = Some((new_min, [t, nb], new));
        }
    }
    if let Some((_, old, new)) = plan {
        let old_v: f64 = old
            .iter()
            .map(|&f| crate::signed_volume(&tri.pts, &tri.tets[f as usize].v))
            .sum();
        let new_v: f64 = new.iter().map(|t| crate::signed_volume(&tri.pts, t)).sum();
        if (old_v - new_v).abs() > 1e-10 * old_v.abs() {
            return false;
        }
        tri.retet(&old, &new);
        stats.face_removals += 1;
        return true;
    }
    false
}

/// The tets round edge `a–b` when they form a closed ring of at most [`MAX_RING`], with the ring's
/// vertices in cyclic order.
fn closed_ring(tri: &mut Tri3, a: u32, b: u32) -> Option<(Vec<u32>, Vec<u32>)> {
    let fan = tri.fan_of_edge(a, b);
    let k = fan.len();
    if !(3..=MAX_RING).contains(&k) {
        return None;
    }
    // each tet contributes the pair of its other vertices; a closed ring chains them head to tail
    let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(k);
    for &f in &fan {
        let tv = tri.tets[f as usize].v;
        // a boundary edge has a tet face without a neighbour among the ones containing a and b
        for i in 0..4 {
            if tv[i] != a && tv[i] != b && tri.tets[f as usize].n[i] == NONE {
                return None;
            }
        }
        let others: Vec<u32> = tv.iter().copied().filter(|&x| x != a && x != b).collect();
        pairs.push((others[0], others[1]));
    }
    let mut ring = vec![pairs[0].0, pairs[0].1];
    let mut used = vec![false; k];
    used[0] = true;
    for _ in 1..k {
        let last = *ring.last()?;
        let next = (0..k).find(|&i| !used[i] && (pairs[i].0 == last || pairs[i].1 == last))?;
        used[next] = true;
        let other = if pairs[next].0 == last {
            pairs[next].1
        } else {
            pairs[next].0
        };
        ring.push(other);
    }
    // the chain must close on the first vertex
    if ring.len() != k + 1 || ring[k] != ring[0] {
        return None;
    }
    ring.pop();
    Some((fan, ring))
}

/// Best triangulation of the ring polygon: returns the worst quality over the `2(k − 2)` tets (each
/// triangle with `a` and with `b`) and the tets, or `None` if no triangulation gives valid tets.
fn best_triangulation(pts: &[P3], a: u32, b: u32, ring: &[u32]) -> Option<(f64, Vec<[u32; 4]>)> {
    let k = ring.len();
    // q[i][j]: best worst-quality of a triangulation of the polygon ring[i..=j] (chain i, i+1, …, j)
    let mut best = vec![vec![f64::NEG_INFINITY; k]; k];
    let mut choice = vec![vec![usize::MAX; k]; k];
    for i in 0..k - 1 {
        best[i][i + 1] = f64::INFINITY;
    }
    // the tets (a, w_i, w_{i+1}, x) and (b, …) next to a ring edge must lie on the same side of the outer
    // face as the old tet, otherwise the new tets stick out of the old region
    let side = |p: u32, q: u32, apex: u32, r: u32| -> f64 {
        orient(
            &pts[apex as usize],
            &pts[p as usize],
            &pts[q as usize],
            &pts[r as usize],
        )
    };
    // for a ring edge (p, q) in ring order, the old tet's fourth vertex is the opposite end of the edge
    let edge_ok = |p: u32, q: u32, r: u32| -> bool {
        // the old tets (a, b, p, q) lie on the side of face (a, p, q) where b is, and on the side of
        // (b, p, q) where a is
        let (sa, sb) = (side(p, q, a, b), side(p, q, b, a));
        let (na, nb) = (side(p, q, a, r), side(p, q, b, r));
        sa * na > 0.0 && sb * nb > 0.0
    };
    let tri_quality = |i: usize, m: usize, j: usize| -> Option<f64> {
        let (p, q, r) = (ring[i], ring[m], ring[j]);
        let ta = positive(pts, [p, q, r, a])?;
        let tb = positive(pts, [p, q, r, b])?;
        // the triangle must separate a from b
        let oa = orient(
            &pts[p as usize],
            &pts[q as usize],
            &pts[r as usize],
            &pts[a as usize],
        );
        let ob = orient(
            &pts[p as usize],
            &pts[q as usize],
            &pts[r as usize],
            &pts[b as usize],
        );
        if oa * ob >= 0.0 {
            return None;
        }
        // the sides of the triangle that are edges of the ring polygon
        if m == i + 1 && !edge_ok(p, q, r) {
            return None;
        }
        if j == m + 1 && !edge_ok(q, r, p) {
            return None;
        }
        if i == 0 && j == k - 1 && !edge_ok(r, p, q) {
            return None;
        }
        Some(quality(pts, &ta).min(quality(pts, &tb)))
    };
    for len in 2..k {
        for i in 0..k - len {
            let j = i + len;
            let mut bq = f64::NEG_INFINITY;
            let mut bm = usize::MAX;
            for m in i + 1..j {
                let Some(q) = tri_quality(i, m, j) else {
                    continue;
                };
                let v = q.min(best[i][m]).min(best[m][j]);
                if v > bq {
                    bq = v;
                    bm = m;
                }
            }
            best[i][j] = bq;
            choice[i][j] = bm;
        }
    }
    let q = best[0][k - 1];
    if !q.is_finite() || q <= 0.0 {
        return None;
    }
    // reconstruct
    let mut out: Vec<[u32; 4]> = Vec::with_capacity(2 * (k - 2));
    let mut stack = vec![(0usize, k - 1)];
    while let Some((i, j)) = stack.pop() {
        if j <= i + 1 {
            continue;
        }
        let m = choice[i][j];
        let (p, qv, r) = (ring[i], ring[m], ring[j]);
        out.push(positive(pts, [p, qv, r, a])?);
        out.push(positive(pts, [p, qv, r, b])?);
        stack.push((i, m));
        stack.push((m, j));
    }
    Some((q, out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::box_tets;
    use crate::smooth::dihedral_range;

    #[test]
    fn flips_keep_the_region_and_never_lower_the_worst_quality() {
        let m = box_tets(2.0, 1.0, 1.0, 4, 2, 2);
        let mut tets = m.tets.clone();
        let vol0: f64 = tets.iter().map(|t| crate::signed_volume(&m.nodes, t)).sum();
        let q0 = tets
            .iter()
            .map(|t| mean_ratio(&m.nodes, t))
            .fold(f64::INFINITY, f64::min);
        let s = improve(&m.nodes, &mut tets, 0.9, 10);
        let vol1: f64 = tets.iter().map(|t| crate::signed_volume(&m.nodes, t)).sum();
        let q1 = tets
            .iter()
            .map(|t| mean_ratio(&m.nodes, t))
            .fold(f64::INFINITY, f64::min);
        assert!((vol0 - vol1).abs() < 1e-12);
        assert!(q1 >= q0 - 1e-12, "{q0} -> {q1} ({s:?})");
        assert!(tets.iter().all(|t| crate::signed_volume(&m.nodes, t) > 0.0));
        let _ = dihedral_range(&m.nodes, &tets);
    }

    #[test]
    fn slivers_of_a_random_delaunay_triangulation_are_improved() {
        // Delaunay triangulations of random points have slivers; the flips must raise the worst quality
        // without changing the region
        let mut seed = 7u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 11) as f64) / ((1u64 << 53) as f64)
        };
        let pts: Vec<P3> = (0..300).map(|_| [rnd(), rnd(), rnd()]).collect();
        let dt = crate::tri3::delaunay(&pts);
        let mut tets: Vec<[u32; 4]> = dt
            .tets
            .iter()
            .filter(|t| t.alive && t.v.iter().all(|&v| (v as usize) < pts.len()))
            .map(|t| t.v)
            .collect();
        let vol0: f64 = tets.iter().map(|t| crate::signed_volume(&pts, t)).sum();
        let worst = |tets: &[[u32; 4]]| {
            tets.iter()
                .map(|t| mean_ratio(&pts, t))
                .fold(f64::INFINITY, f64::min)
        };
        let (q0, n0) = (worst(&tets), tets.len());
        let s = improve(&pts, &mut tets, 0.5, 20);
        let vol1: f64 = tets.iter().map(|t| crate::signed_volume(&pts, t)).sum();
        let q1 = worst(&tets);
        eprintln!(
            "{n0} tets, worst mean ratio {q0:.4} -> {q1:.4}, {} tets after, {s:?}",
            tets.len()
        );
        assert!((vol0 - vol1).abs() < 1e-10 * vol0);
        assert!(tets.iter().all(|t| crate::signed_volume(&pts, t) > 0.0));
        assert!(q1 > q0, "{q0} -> {q1}");
        assert!(s.edge_removals + s.face_removals > 0);
        // adjacency stays consistent
        let tri = Tri3::from_tets(pts.clone(), &tets);
        tri.validate().unwrap();
    }
}
