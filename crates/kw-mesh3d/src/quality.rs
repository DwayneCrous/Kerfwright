use crate::{cross, dot, norm, sub, P3};

/// Signed volume of the tet (positive when `d` lies on the side of `abc` its right-hand normal points to).
pub fn signed_volume(nodes: &[P3], t: &[u32; 4]) -> f64 {
    let (a, b, c, d) = (
        nodes[t[0] as usize],
        nodes[t[1] as usize],
        nodes[t[2] as usize],
        nodes[t[3] as usize],
    );
    dot(cross(sub(b, a), sub(c, a)), sub(d, a)) / 6.0
}

pub fn shortest_edge(nodes: &[P3], t: &[u32; 4]) -> f64 {
    let mut m = f64::INFINITY;
    for i in 0..4 {
        for j in i + 1..4 {
            m = m.min(norm(sub(nodes[t[i] as usize], nodes[t[j] as usize])));
        }
    }
    m
}

pub fn circumradius(nodes: &[P3], t: &[u32; 4]) -> f64 {
    let a = nodes[t[0] as usize];
    let (b, c, d) = (
        sub(nodes[t[1] as usize], a),
        sub(nodes[t[2] as usize], a),
        sub(nodes[t[3] as usize], a),
    );
    let det = dot(b, cross(c, d));
    if det.abs() < 1e-300 {
        return f64::INFINITY;
    }
    let num = [cross(c, d), cross(d, b), cross(b, c)];
    let (b2, c2, d2) = (dot(b, b), dot(c, c), dot(d, d));
    let center = [
        (b2 * num[0][0] + c2 * num[1][0] + d2 * num[2][0]) / (2.0 * det),
        (b2 * num[0][1] + c2 * num[1][1] + d2 * num[2][1]) / (2.0 * det),
        (b2 * num[0][2] + c2 * num[1][2] + d2 * num[2][2]) / (2.0 * det),
    ];
    norm(center)
}

/// Circumradius over shortest edge: 0.612 for the regular tetrahedron, large for slivers.
pub fn radius_edge_ratio(nodes: &[P3], t: &[u32; 4]) -> f64 {
    circumradius(nodes, t) / shortest_edge(nodes, t)
}

#[derive(Debug, Clone, Copy)]
pub struct QualityReport {
    pub elements: usize,
    pub inverted: usize,
    pub worst_radius_edge: f64,
    /// Fraction of elements with radius-edge ratio ≤ 2.
    pub fraction_within_2: f64,
    pub min_volume: f64,
}

impl QualityReport {
    pub fn of(nodes: &[P3], tets: &[[u32; 4]]) -> QualityReport {
        let mut inverted = 0;
        let mut worst = 0.0f64;
        let mut good = 0usize;
        let mut vmin = f64::INFINITY;
        for t in tets {
            let v = signed_volume(nodes, t);
            vmin = vmin.min(v);
            if v <= 0.0 {
                inverted += 1;
                continue;
            }
            let q = radius_edge_ratio(nodes, t);
            worst = worst.max(q);
            if q <= 2.0 {
                good += 1;
            }
        }
        QualityReport {
            elements: tets.len(),
            inverted,
            worst_radius_edge: worst,
            fraction_within_2: good as f64 / tets.len().max(1) as f64,
            min_volume: vmin,
        }
    }
}
