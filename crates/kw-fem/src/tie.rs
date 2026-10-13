//! Bonded contact between bodies: two separately meshed bodies that touch are joined by tying the nodes of
//! one surface (the *slave* side, the one with more interface nodes) to the quadratic facets of the other
//! (the *master* side).
//!
//! For a slave node `s` lying (within a gap tolerance) on the master facet `m` at barycentric coordinates
//! `(l1, l2, l3)` the tie is `u_s = Σ N_i(l) u_{m_i}` with the six quadratic shape functions of the facet,
//! on every component. The constraint is enforced by a penalty `α (u_s − Σ N_i u_{m_i})²` added to the
//! matrix, which keeps it symmetric positive definite, so the Cholesky factorisation and the conjugate
//! gradient solver work unchanged. It serves any number of unknowns per node: the displacements of a
//! structural model and the temperature of a thermal one.

use crate::sparse::Csr;
use kw_mesh3d::{MergedMesh, P3};
use rstar::primitives::{GeomWithData, Rectangle};
use rstar::{RTree, AABB};
use std::collections::HashMap;

/// One slave node tied to a master facet.
#[derive(Debug, Clone, PartialEq)]
pub struct Tie {
    pub slave: u32,
    /// The six nodes of the master facet (corners, then the mid-nodes of edges 01, 12, 20).
    pub master: [u32; 6],
    /// Their shape functions at the slave node (they sum to one).
    pub weights: [f64; 6],
    /// Distance from the slave node to the master facet before tying.
    pub gap: f64,
}

/// What was found between one pair of bodies.
#[derive(Debug, Clone, PartialEq)]
pub struct PairReport {
    pub a: usize,
    pub b: usize,
    /// The body whose nodes were tied to the other's facets.
    pub slave: usize,
    pub tied: usize,
    pub max_gap: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct TieReport {
    pub pairs: Vec<PairReport>,
}

impl TieReport {
    pub fn tied(&self) -> usize {
        self.pairs.iter().map(|p| p.tied).sum()
    }
}

/// How to look for contacts.
#[derive(Debug, Clone)]
pub struct TieOptions {
    /// Largest gap between two surfaces that still counts as touching (mm).
    pub tolerance: f64,
    /// Pairs of bodies (indices, any order) that are not bonded.
    pub skip: Vec<(usize, usize)>,
}

impl Default for TieOptions {
    fn default() -> Self {
        TieOptions {
            tolerance: 1e-3,
            skip: Vec::new(),
        }
    }
}

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The point of triangle `abc` closest to `p` and its barycentric coordinates (Ericson, *Real-Time
/// Collision Detection*).
pub fn closest_on_triangle(p: P3, a: P3, b: P3, c: P3) -> (P3, [f64; 3]) {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, [1.0, 0.0, 0.0]);
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return (b, [0.0, 1.0, 0.0]);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (
            [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]],
            [1.0 - v, v, 0.0],
        );
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return (c, [0.0, 0.0, 1.0]);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (
            [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]],
            [1.0 - w, 0.0, w],
        );
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (
            [
                b[0] + w * (c[0] - b[0]),
                b[1] + w * (c[1] - b[1]),
                b[2] + w * (c[2] - b[2]),
            ],
            [0.0, 1.0 - w, w],
        );
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    (
        [
            a[0] + ab[0] * v + ac[0] * w,
            a[1] + ab[1] * v + ac[1] * w,
            a[2] + ab[2] * v + ac[2] * w,
        ],
        [1.0 - v - w, v, w],
    )
}

/// The quadratic triangle shape functions at barycentric coordinates `l` (corners, then edges 01, 12, 20).
pub fn tri6_shape(l: [f64; 3]) -> [f64; 6] {
    let (l1, l2, l3) = (l[0], l[1], l[2]);
    [
        l1 * (2.0 * l1 - 1.0),
        l2 * (2.0 * l2 - 1.0),
        l3 * (2.0 * l3 - 1.0),
        4.0 * l1 * l2,
        4.0 * l2 * l3,
        4.0 * l3 * l1,
    ]
}

type Item = GeomWithData<Rectangle<[f64; 3]>, usize>;

fn aabb_of(mesh: &kw_mesh3d::Tet10Mesh, v: &[u32; 6], pad: f64) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &n in v {
        let p = mesh.nodes[n as usize];
        for a in 0..3 {
            lo[a] = lo[a].min(p[a] - pad);
            hi[a] = hi[a].max(p[a] + pad);
        }
    }
    (lo, hi)
}

/// Find the ties between every pair of bodies of `merged` whose surfaces touch.
pub fn find_ties(merged: &MergedMesh, opts: &TieOptions) -> (Vec<Tie>, TieReport) {
    let mesh = &merged.mesh;
    let bodies = merged.node_start.len().saturating_sub(1);
    let mut by_body: Vec<Vec<usize>> = vec![Vec::new(); bodies];
    for (i, s) in mesh.surface.iter().enumerate() {
        by_body[merged.body_of_node(s.v[0])].push(i);
    }
    let corner = |ti: usize| -> [P3; 3] {
        let v = &mesh.surface[ti].v;
        [
            mesh.nodes[v[0] as usize],
            mesh.nodes[v[1] as usize],
            mesh.nodes[v[2] as usize],
        ]
    };
    let normal = |ti: usize| -> P3 {
        let [a, b, c] = corner(ti);
        let n = cross(sub(b, a), sub(c, a));
        let l = dot(n, n).sqrt().max(1e-300);
        [n[0] / l, n[1] / l, n[2] / l]
    };
    let mut ties = Vec::new();
    let mut report = TieReport::default();
    let tol = opts.tolerance;
    for a in 0..bodies {
        for b in a + 1..bodies {
            if opts
                .skip
                .iter()
                .any(|&(x, y)| (x == a && y == b) || (x == b && y == a))
            {
                continue;
            }
            // facets of b in a tree; facets of a look for partners
            let tree = RTree::bulk_load(
                by_body[b]
                    .iter()
                    .map(|&ti| {
                        let (lo, hi) = aabb_of(mesh, &mesh.surface[ti].v, tol);
                        GeomWithData::new(Rectangle::from_corners(lo, hi), ti)
                    })
                    .collect::<Vec<Item>>(),
            );
            // partner facets per facet of a (opposing normals, closer than the tolerance)
            let mut partners: Vec<(usize, Vec<usize>)> = Vec::new();
            for &ta in &by_body[a] {
                let (lo, hi) = aabb_of(mesh, &mesh.surface[ta].v, tol);
                let na = normal(ta);
                let mut found = Vec::new();
                for item in tree.locate_in_envelope_intersecting(&AABB::from_corners(lo, hi)) {
                    let tb = item.data;
                    if dot(na, normal(tb)) > -0.5 {
                        continue;
                    }
                    // some node of the facet of a must be within the tolerance of the facet of b
                    let [b0, b1, b2] = corner(tb);
                    let near = mesh.surface[ta].v.iter().any(|&n| {
                        let p = mesh.nodes[n as usize];
                        let (q, _) = closest_on_triangle(p, b0, b1, b2);
                        let d = sub(p, q);
                        dot(d, d).sqrt() <= tol
                    });
                    if near {
                        found.push(tb);
                    }
                }
                if !found.is_empty() {
                    partners.push((ta, found));
                }
            }
            if partners.is_empty() {
                continue;
            }
            // the same from b's side, to decide which side is finer (has more nodes on the interface)
            let mut nodes_a: HashMap<u32, Vec<usize>> = HashMap::new();
            let mut nodes_b: HashMap<u32, Vec<usize>> = HashMap::new();
            for (ta, tbs) in &partners {
                for &n in &mesh.surface[*ta].v {
                    nodes_a.entry(n).or_default().extend(tbs.iter().copied());
                }
                for &tb in tbs {
                    for &n in &mesh.surface[tb].v {
                        nodes_b.entry(n).or_default().push(*ta);
                    }
                }
            }
            let (slave_body, slave_nodes, master_body) = if nodes_a.len() > nodes_b.len() {
                (a, nodes_a, b)
            } else {
                (b, nodes_b, a)
            };
            let _ = master_body;
            let mut tied = 0usize;
            let mut max_gap = 0.0f64;
            let mut keys: Vec<u32> = slave_nodes.keys().copied().collect();
            keys.sort_unstable();
            for n in keys {
                let p = mesh.nodes[n as usize];
                let mut best: Option<(f64, usize, [f64; 3])> = None;
                let mut cands = slave_nodes[&n].clone();
                cands.sort_unstable();
                cands.dedup();
                for tm in cands {
                    let [c0, c1, c2] = corner(tm);
                    let (q, bary) = closest_on_triangle(p, c0, c1, c2);
                    let d = sub(p, q);
                    let dist = dot(d, d).sqrt();
                    if best.is_none_or(|(bd, _, _)| dist < bd) {
                        best = Some((dist, tm, bary));
                    }
                }
                if let Some((dist, tm, bary)) = best {
                    if dist <= tol {
                        ties.push(Tie {
                            slave: n,
                            master: mesh.surface[tm].v,
                            weights: tri6_shape(bary),
                            gap: dist,
                        });
                        tied += 1;
                        max_gap = max_gap.max(dist);
                    }
                }
            }
            report.pairs.push(PairReport {
                a,
                b,
                slave: slave_body,
                tied,
                max_gap,
            });
        }
    }
    (ties, report)
}

/// The nodes each tie couples (the slave and its six masters), to widen the sparsity pattern.
pub fn tie_groups(ties: &[Tie]) -> Vec<Vec<u32>> {
    ties.iter()
        .map(|t| {
            let mut g = vec![t.slave];
            g.extend_from_slice(&t.master);
            g
        })
        .collect()
}

/// Add the penalty terms of `ties` to `k` (`nd` unknowns per node). `scale` times the largest diagonal entry
/// among the tied nodes is the penalty stiffness: large enough to hold the nodes together to about 1/scale,
/// small enough not to ruin the conditioning.
pub fn apply_penalty(k: &mut Csr, nd: usize, ties: &[Tie], scale: f64) {
    // the strength comes from the matrix without any penalty, so ties that share nodes do not compound
    let diag = k.diagonal();
    for t in ties {
        for a in 0..nd {
            let mut dofs: Vec<(usize, f64)> = vec![(nd * t.slave as usize + a, 1.0)];
            for (m, w) in t.master.iter().zip(&t.weights) {
                dofs.push((nd * *m as usize + a, -*w));
            }
            let big = dofs.iter().map(|(d, _)| diag[*d]).fold(0.0f64, f64::max);
            let alpha = scale * big;
            for &(p, cp) in &dofs {
                for &(q, cq) in &dofs {
                    k.add(p, q, alpha * cp * cq);
                }
            }
        }
    }
}
