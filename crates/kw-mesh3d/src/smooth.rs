//! Interior-vertex smoothing and dihedral-angle measures.

use crate::{cross, dot, norm, sub, P3};

fn volume6(p: &[P3], t: &[u32; 4]) -> f64 {
    let (a, b, c, d) = (
        p[t[0] as usize],
        p[t[1] as usize],
        p[t[2] as usize],
        p[t[3] as usize],
    );
    dot(cross(sub(b, a), sub(c, a)), sub(d, a))
}

/// Mean-ratio quality in (0, 1]: 1 for a regular tetrahedron, 0 for a flat one, negative if inverted.
pub fn mean_ratio(p: &[P3], t: &[u32; 4]) -> f64 {
    let v6 = volume6(p, t);
    let mut s = 0.0;
    for i in 0..4 {
        for j in i + 1..4 {
            let e = sub(p[t[i] as usize], p[t[j] as usize]);
            s += dot(e, e);
        }
    }
    if s <= 0.0 {
        return 0.0;
    }
    // 12 (3V)^{2/3} / Σ l²  with V = v6 / 6
    let v = v6 / 6.0;
    12.0 * (3.0 * v.abs()).powf(2.0 / 3.0) * v.signum() / s
}

/// Smallest and largest dihedral angle (degrees) over all tets.
pub fn dihedral_range(p: &[P3], tets: &[[u32; 4]]) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = 0.0f64;
    for t in tets {
        let c = [
            p[t[0] as usize],
            p[t[1] as usize],
            p[t[2] as usize],
            p[t[3] as usize],
        ];
        // outward normals of the face opposite each vertex
        let mut n = [[0.0; 3]; 4];
        for i in 0..4 {
            let f: Vec<usize> = (0..4).filter(|&k| k != i).collect();
            let mut nn = cross(sub(c[f[1]], c[f[0]]), sub(c[f[2]], c[f[0]]));
            if dot(nn, sub(c[i], c[f[0]])) > 0.0 {
                nn = [-nn[0], -nn[1], -nn[2]];
            }
            let l = norm(nn);
            n[i] = if l > 0.0 {
                [nn[0] / l, nn[1] / l, nn[2] / l]
            } else {
                nn
            };
        }
        for i in 0..4 {
            for j in i + 1..4 {
                let a = (-dot(n[i], n[j])).clamp(-1.0, 1.0).acos().to_degrees();
                lo = lo.min(a);
                hi = hi.max(a);
            }
        }
    }
    (lo, hi)
}

/// Smart Laplacian smoothing: every non-boundary vertex moves to the centroid of its neighbours when that
/// improves the worst mean-ratio quality of the tets around it (and keeps them all positive).
pub fn smooth(nodes: &mut [P3], tets: &[[u32; 4]], boundary: &[bool], passes: usize) {
    if passes == 0 {
        return;
    }
    let n = nodes.len();
    let mut count = vec![0usize; n + 1];
    for t in tets {
        for &v in t {
            count[v as usize + 1] += 1;
        }
    }
    for i in 0..n {
        count[i + 1] += count[i];
    }
    let mut fill = count.clone();
    let mut star = vec![0u32; count[n]];
    for (k, t) in tets.iter().enumerate() {
        for &v in t {
            star[fill[v as usize]] = k as u32;
            fill[v as usize] += 1;
        }
    }
    for _ in 0..passes {
        let mut moved = 0usize;
        for v in 0..n {
            if boundary[v] || count[v] == count[v + 1] {
                continue;
            }
            let ts = &star[count[v]..count[v + 1]];
            let mut sum = [0.0; 3];
            let mut cnt = 0.0;
            for &k in ts {
                for &u in &tets[k as usize] {
                    if u as usize != v {
                        for a in 0..3 {
                            sum[a] += nodes[u as usize][a];
                        }
                        cnt += 1.0;
                    }
                }
            }
            let cand = [sum[0] / cnt, sum[1] / cnt, sum[2] / cnt];
            let worst = |nodes: &[P3]| {
                ts.iter()
                    .map(|&k| mean_ratio(nodes, &tets[k as usize]))
                    .fold(f64::INFINITY, f64::min)
            };
            let before = worst(nodes);
            let old = nodes[v];
            nodes[v] = cand;
            let after = worst(nodes);
            if after > before && after > 0.0 {
                moved += 1;
            } else {
                nodes[v] = old;
            }
        }
        if moved == 0 {
            break;
        }
    }
}
