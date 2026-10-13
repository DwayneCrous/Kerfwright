//! One subdivision level: positions, polygons, edge sharpness, and the Catmull–Clark step with the
//! DeRose crease rules.

use crate::mesh::SubdMesh;
use std::collections::HashMap;

pub type P3 = [f64; 3];

pub const INFINITE: f64 = 1e9;

fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: P3, s: f64) -> P3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn lerp(a: P3, b: P3, t: f64) -> P3 {
    add(scale(a, 1.0 - t), scale(b, t))
}

fn key(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

#[derive(Debug, Clone)]
pub struct Level {
    pub verts: Vec<P3>,
    pub faces: Vec<Vec<u32>>,
    /// Sharpness of the edges that are sharp (positive); boundary edges are infinitely sharp.
    pub sharp: HashMap<(u32, u32), f64>,
    /// First child (in the next level) of every face; children of face `f` are
    /// `child_start[f] .. child_start[f + 1]`. Empty until the level has been subdivided.
    pub child_start: Vec<usize>,
}

struct Topology {
    /// Edges in first-use order.
    edges: Vec<(u32, u32)>,
    edge_of: HashMap<(u32, u32), usize>,
    edge_faces: Vec<Vec<usize>>,
    vert_edges: Vec<Vec<usize>>,
    vert_faces: Vec<Vec<usize>>,
}

impl Level {
    pub fn from_mesh(m: &SubdMesh) -> Level {
        let mut l = Level {
            verts: m.verts.clone(),
            faces: m.faces.clone(),
            sharp: HashMap::new(),
            child_start: Vec::new(),
        };
        let topo = l.topology();
        for c in &m.creases {
            let k = key(c.a, c.b);
            if topo.edge_of.contains_key(&k) && c.sharpness > 0.0 {
                l.sharp.insert(k, c.sharpness.min(INFINITE));
            }
        }
        for (e, faces) in topo.edge_faces.iter().enumerate() {
            if faces.len() == 1 {
                l.sharp.insert(topo.edges[e], INFINITE);
            }
        }
        l
    }

    fn topology(&self) -> Topology {
        let mut edges = Vec::new();
        let mut edge_of = HashMap::new();
        let mut edge_faces: Vec<Vec<usize>> = Vec::new();
        let mut vert_edges = vec![Vec::new(); self.verts.len()];
        let mut vert_faces = vec![Vec::new(); self.verts.len()];
        for (fi, f) in self.faces.iter().enumerate() {
            for k in 0..f.len() {
                let (a, b) = (f[k], f[(k + 1) % f.len()]);
                let kk = key(a, b);
                let e = *edge_of.entry(kk).or_insert_with(|| {
                    edges.push(kk);
                    edge_faces.push(Vec::new());
                    vert_edges[a as usize].push(edges.len() - 1);
                    vert_edges[b as usize].push(edges.len() - 1);
                    edges.len() - 1
                });
                edge_faces[e].push(fi);
                vert_faces[a as usize].push(fi);
            }
        }
        Topology {
            edges,
            edge_of,
            edge_faces,
            vert_edges,
            vert_faces,
        }
    }

    pub fn is_all_quads(&self) -> bool {
        self.faces.iter().all(|f| f.len() == 4)
    }

    pub fn sharpness(&self, a: u32, b: u32) -> f64 {
        self.sharp.get(&key(a, b)).copied().unwrap_or(0.0)
    }

    /// One Catmull–Clark step. Child `c` of a quad `(v0 v1 v2 v3)` keeps the parent's orientation:
    /// `c0 = (v0 e01 f e30)`, `c1 = (e01 v1 e12 f)`, `c2 = (f e12 v2 e23)`, `c3 = (e30 f e23 v3)`.
    pub fn subdivide(&mut self) -> Level {
        let t = self.topology();
        let nv = self.verts.len();
        let ne = t.edges.len();
        let nf = self.faces.len();
        // face points
        let fp: Vec<P3> = self
            .faces
            .iter()
            .map(|f| {
                let mut s = [0.0; 3];
                for &v in f {
                    s = add(s, self.verts[v as usize]);
                }
                scale(s, 1.0 / f.len() as f64)
            })
            .collect();
        let mut out = vec![[0.0; 3]; nv + ne + nf];
        // edge points
        for (e, &(a, b)) in t.edges.iter().enumerate() {
            let (pa, pb) = (self.verts[a as usize], self.verts[b as usize]);
            let mid = scale(add(pa, pb), 0.5);
            let s = self.sharpness(a, b);
            let smooth = if t.edge_faces[e].len() == 2 {
                let (f0, f1) = (fp[t.edge_faces[e][0]], fp[t.edge_faces[e][1]]);
                scale(add(add(pa, pb), add(f0, f1)), 0.25)
            } else {
                mid
            };
            out[nv + e] = if s >= 1.0 {
                mid
            } else if s > 0.0 {
                lerp(smooth, mid, s)
            } else {
                smooth
            };
        }
        for (f, p) in fp.iter().enumerate() {
            out[nv + ne + f] = *p;
        }
        // vertex points
        for v in 0..nv {
            let p = self.verts[v];
            let edges = &t.vert_edges[v];
            let mut sharp_nb: Vec<(u32, f64)> = Vec::new();
            for &e in edges {
                let (a, b) = t.edges[e];
                let other = if a as usize == v { b } else { a };
                let s = self.sharpness(a, b);
                if s > 0.0 {
                    sharp_nb.push((other, s));
                }
            }
            let n = edges.len() as f64;
            let smooth = {
                let mut f_sum = [0.0; 3];
                for &f in &t.vert_faces[v] {
                    f_sum = add(f_sum, fp[f]);
                }
                let f_avg = scale(f_sum, 1.0 / t.vert_faces[v].len() as f64);
                let mut r_sum = [0.0; 3];
                for &e in edges {
                    let (a, b) = t.edges[e];
                    r_sum = add(
                        r_sum,
                        scale(add(self.verts[a as usize], self.verts[b as usize]), 0.5),
                    );
                }
                let r_avg = scale(r_sum, 1.0 / n);
                // (F + 2R + (n-3)P) / n
                scale(
                    add(add(f_avg, scale(r_avg, 2.0)), scale(p, n - 3.0)),
                    1.0 / n,
                )
            };
            out[v] = match sharp_nb.len() {
                0 | 1 => smooth,
                2 => {
                    let crease = scale(
                        add(
                            scale(p, 6.0),
                            add(
                                self.verts[sharp_nb[0].0 as usize],
                                self.verts[sharp_nb[1].0 as usize],
                            ),
                        ),
                        1.0 / 8.0,
                    );
                    let s = 0.5 * (sharp_nb[0].1 + sharp_nb[1].1);
                    if s >= 1.0 {
                        crease
                    } else {
                        lerp(smooth, crease, s)
                    }
                }
                _ => p,
            };
        }
        // faces
        let edge_pt = |a: u32, b: u32| (nv + t.edge_of[&key(a, b)]) as u32;
        let mut faces: Vec<Vec<u32>> = Vec::new();
        let mut child_start = Vec::with_capacity(nf + 1);
        for (fi, f) in self.faces.iter().enumerate() {
            child_start.push(faces.len());
            let fpi = (nv + ne + fi) as u32;
            let k = f.len();
            if k == 4 {
                let (v0, v1, v2, v3) = (f[0], f[1], f[2], f[3]);
                let (e01, e12, e23, e30) = (
                    edge_pt(v0, v1),
                    edge_pt(v1, v2),
                    edge_pt(v2, v3),
                    edge_pt(v3, v0),
                );
                faces.push(vec![v0, e01, fpi, e30]);
                faces.push(vec![e01, v1, e12, fpi]);
                faces.push(vec![fpi, e12, v2, e23]);
                faces.push(vec![e30, fpi, e23, v3]);
            } else {
                for i in 0..k {
                    let (a, b, z) = (f[i], f[(i + 1) % k], f[(i + k - 1) % k]);
                    faces.push(vec![a, edge_pt(a, b), fpi, edge_pt(z, a)]);
                }
            }
        }
        child_start.push(faces.len());
        self.child_start = child_start;
        // sharpness
        let mut sharp = HashMap::new();
        for (e, &(a, b)) in t.edges.iter().enumerate() {
            let s = self.sharpness(a, b);
            if s > 0.0 {
                let s2 = if s >= INFINITE {
                    INFINITE
                } else {
                    (s - 1.0).max(0.0)
                };
                if s2 > 0.0 {
                    let m = (nv + e) as u32;
                    sharp.insert(key(a, m), s2);
                    sharp.insert(key(m, b), s2);
                }
            }
        }
        Level {
            verts: out,
            faces,
            sharp,
            child_start: Vec::new(),
        }
    }

    /// Positions of the vertices on the limit surface (the level must consist of quads).
    pub fn limit(&self) -> Vec<P3> {
        debug_assert!(self.is_all_quads());
        let t = self.topology();
        let mut out = Vec::with_capacity(self.verts.len());
        for v in 0..self.verts.len() {
            let p = self.verts[v];
            let mut sharp_nb: Vec<u32> = Vec::new();
            let mut e_sum = [0.0; 3];
            for &e in &t.vert_edges[v] {
                let (a, b) = t.edges[e];
                let other = if a as usize == v { b } else { a };
                e_sum = add(e_sum, self.verts[other as usize]);
                if self.sharpness(a, b) > 0.0 {
                    sharp_nb.push(other);
                }
            }
            out.push(match sharp_nb.len() {
                0 | 1 => {
                    let n = t.vert_edges[v].len() as f64;
                    let mut f_sum = [0.0; 3];
                    for &f in &t.vert_faces[v] {
                        let q = &self.faces[f];
                        let k = q.iter().position(|x| *x as usize == v).unwrap();
                        f_sum = add(f_sum, self.verts[q[(k + 2) % 4] as usize]);
                    }
                    scale(
                        add(add(scale(p, n * n), scale(e_sum, 4.0)), f_sum),
                        1.0 / (n * (n + 5.0)),
                    )
                }
                2 => scale(
                    add(
                        scale(p, 4.0),
                        add(
                            self.verts[sharp_nb[0] as usize],
                            self.verts[sharp_nb[1] as usize],
                        ),
                    ),
                    1.0 / 6.0,
                ),
                _ => p,
            });
        }
        out
    }

    /// Valence of every vertex.
    pub fn valences(&self) -> Vec<usize> {
        self.topology().vert_edges.iter().map(|e| e.len()).collect()
    }
}

/// The `(2^m + 1)²` fine-level vertices below face `face` of `levels[k]`, indexed `[i][j]` with `i` along
/// the face's first edge and `j` along its last (`v0→v1` and `v0→v3`).
pub fn face_grid(levels: &[Level], k: usize, face: usize, m: usize) -> Vec<Vec<u32>> {
    let f = &levels[k].faces[face];
    debug_assert_eq!(f.len(), 4);
    let mut grid: Vec<Vec<u32>> = vec![vec![f[0], f[3]], vec![f[1], f[2]]];
    let mut cells: Vec<Vec<usize>> = vec![vec![face]];
    for step in 0..m {
        let lvl = k + step;
        let n = cells.len();
        let size = 2 * n + 1;
        let mut ng = vec![vec![0u32; size]; size];
        let mut nc = vec![vec![0usize; 2 * n]; 2 * n];
        for a in 0..n {
            for b in 0..n {
                let cf = cells[a][b];
                let first = levels[lvl].child_start[cf];
                let child = |c: usize| &levels[lvl + 1].faces[first + c];
                let (c0, c1, c2, c3) = (child(0), child(1), child(2), child(3));
                // c0 at (2a, 2b)
                ng[2 * a][2 * b] = c0[0];
                ng[2 * a + 1][2 * b] = c0[1];
                ng[2 * a + 1][2 * b + 1] = c0[2];
                ng[2 * a][2 * b + 1] = c0[3];
                // c1 at (2a+1, 2b)
                ng[2 * a + 2][2 * b] = c1[1];
                ng[2 * a + 2][2 * b + 1] = c1[2];
                // c2 at (2a+1, 2b+1)
                ng[2 * a + 2][2 * b + 2] = c2[2];
                ng[2 * a + 1][2 * b + 2] = c2[3];
                // c3 at (2a, 2b+1)
                ng[2 * a][2 * b + 2] = c3[3];
                nc[2 * a][2 * b] = first;
                nc[2 * a + 1][2 * b] = first + 1;
                nc[2 * a + 1][2 * b + 1] = first + 2;
                nc[2 * a][2 * b + 1] = first + 3;
            }
        }
        grid = ng;
        cells = nc;
    }
    grid
}
