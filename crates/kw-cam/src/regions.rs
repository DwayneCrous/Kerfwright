//! Machining regions from the model: the outline of a planar face as polygons with islands.

use crate::{P2, P3};
use std::collections::HashMap;

/// The boundary loops of a set of triangles that share a plane: the edges used by exactly one triangle,
/// chained into closed loops. The loop of largest area is the outer boundary; all others are islands.
/// Points are projected to XY. Vertices closer than `1e-6` are the same vertex.
pub fn face_loops(tris: &[[P3; 3]]) -> Option<(Vec<P2>, Vec<Vec<P2>>)> {
    let key = |p: &P3| -> (i64, i64, i64) {
        (
            (p[0] * 1e6).round() as i64,
            (p[1] * 1e6).round() as i64,
            (p[2] * 1e6).round() as i64,
        )
    };
    let mut ids: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut pts: Vec<P2> = Vec::new();
    let mut id_of = |p: &P3| -> usize {
        *ids.entry(key(p)).or_insert_with(|| {
            pts.push([p[0], p[1]]);
            pts.len() - 1
        })
    };
    // directed edge counts: an interior edge is used once in each direction
    let mut edges: HashMap<(usize, usize), i32> = HashMap::new();
    for t in tris {
        let v = [id_of(&t[0]), id_of(&t[1]), id_of(&t[2])];
        for k in 0..3 {
            let (a, b) = (v[k], v[(k + 1) % 3]);
            if a == b {
                continue;
            }
            if edges.remove(&(b, a)).is_none() {
                *edges.entry((a, b)).or_insert(0) += 1;
            }
        }
    }
    let mut next: HashMap<usize, Vec<usize>> = HashMap::new();
    for (a, b) in edges.keys() {
        next.entry(*a).or_default().push(*b);
    }
    let mut loops: Vec<Vec<P2>> = Vec::new();
    while let Some(&start) = next.iter().find(|(_, v)| !v.is_empty()).map(|(k, _)| k) {
        let mut lp = vec![start];
        let mut cur = start;
        while let Some(n) = next.get_mut(&cur).and_then(|v| v.pop()) {
            if n == start {
                break;
            }
            lp.push(n);
            cur = n;
            if lp.len() > pts.len() + 1 {
                break;
            }
        }
        if lp.len() >= 3 {
            loops.push(lp.iter().map(|i| pts[*i]).collect());
        }
    }
    let area = |p: &Vec<P2>| -> f64 {
        let mut a = 0.0;
        for i in 0..p.len() {
            let (q, r) = (p[i], p[(i + 1) % p.len()]);
            a += q[0] * r[1] - r[0] * q[1];
        }
        0.5 * a
    };
    let outer_i = (0..loops.len())
        .max_by(|a, b| area(&loops[*a]).abs().total_cmp(&area(&loops[*b]).abs()))?;
    let outer = loops.remove(outer_i);
    Some((outer, loops))
}

/// Drop collinear vertices (within `tol`) from a closed polygon.
pub fn simplify(p: &[P2], tol: f64) -> Vec<P2> {
    let n = p.len();
    if n < 4 {
        return p.to_vec();
    }
    let mut out = Vec::new();
    for i in 0..n {
        let (a, b, c) = (p[(i + n - 1) % n], p[i], p[(i + 1) % n]);
        let (ux, uy) = (c[0] - a[0], c[1] - a[1]);
        let len = ux.hypot(uy);
        let dist = if len < 1e-12 {
            0.0
        } else {
            ((b[0] - a[0]) * uy - (b[1] - a[1]) * ux).abs() / len
        };
        if dist > tol {
            out.push(b);
        }
    }
    if out.len() < 3 {
        p.to_vec()
    } else {
        out
    }
}
