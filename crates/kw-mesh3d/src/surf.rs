//! Boundary surface of a solid as an editable, closed, outward-oriented triangle mesh, and builders for
//! simple test surfaces.

use crate::{cross, dot, norm, sub, P3};
use std::collections::HashMap;

/// A triangle mesh (closed and 2-manifold for solids; imported meshes may be open), counter-clockwise seen from outside, with a tag per triangle (the
/// identifier of the geometry face it approximates).
#[derive(Debug, Clone, Default)]
pub struct SurfaceMesh {
    pub nodes: Vec<P3>,
    pub tris: Vec<[u32; 3]>,
    pub tags: Vec<u32>,
}

impl SurfaceMesh {
    /// Build from a B-rep tessellation: vertices with equal `weld` ids are merged, triangles collapsed by
    /// the merge are dropped, and the dense face index is the tag.
    pub fn from_tess(mesh: &kw_tess::Mesh) -> SurfaceMesh {
        let mut index: HashMap<u32, u32> = HashMap::new();
        let mut out = SurfaceMesh::default();
        for (k, tri) in mesh.indices.as_chunks::<3>().0.iter().enumerate() {
            let mut t = [0u32; 3];
            for i in 0..3 {
                let v = tri[i] as usize;
                let w = mesh.weld[v];
                t[i] = *index.entry(w).or_insert_with(|| {
                    let p = mesh.positions[v];
                    out.nodes.push([p.x, p.y, p.z]);
                    (out.nodes.len() - 1) as u32
                });
            }
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                continue;
            }
            out.tris.push(t);
            out.tags.push(mesh.tri_face.get(k).copied().unwrap_or(0));
        }
        out
    }

    /// Build from a triangle soup: corners closer than `tol` become one node (a hash grid, so the node order
    /// follows the first appearance in `tris` and is the same on every run), triangles that collapse or have
    /// no area are dropped and counted in the second result.
    pub fn from_soup(tris: &[[P3; 3]], tol: f64) -> (SurfaceMesh, usize) {
        let cell = tol.max(1e-300) * 2.0;
        let key = |p: &P3| {
            [
                (p[0] / cell).floor() as i64,
                (p[1] / cell).floor() as i64,
                (p[2] / cell).floor() as i64,
            ]
        };
        let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
        let mut out = SurfaceMesh::default();
        let mut dropped = 0;
        for tri in tris {
            let mut t = [0u32; 3];
            for (i, p) in tri.iter().enumerate() {
                let k = key(p);
                let mut found = None;
                'search: for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            if let Some(list) = grid.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                                for &n in list {
                                    if norm(sub(out.nodes[n as usize], *p)) <= tol {
                                        found = Some(n);
                                        break 'search;
                                    }
                                }
                            }
                        }
                    }
                }
                t[i] = found.unwrap_or_else(|| {
                    out.nodes.push(*p);
                    let n = (out.nodes.len() - 1) as u32;
                    grid.entry(k).or_default().push(n);
                    n
                });
            }
            let (a, b, c) = (
                out.nodes[t[0] as usize],
                out.nodes[t[1] as usize],
                out.nodes[t[2] as usize],
            );
            if t[0] == t[1]
                || t[1] == t[2]
                || t[0] == t[2]
                || norm(cross(sub(b, a), sub(c, a))) <= 1e-12 * tol.max(1e-300)
            {
                dropped += 1;
                continue;
            }
            out.tris.push(t);
            out.tags.push(0);
        }
        // nodes that only the dropped triangles used
        let mut used = vec![false; out.nodes.len()];
        for t in &out.tris {
            for &v in t {
                used[v as usize] = true;
            }
        }
        if used.iter().any(|u| !u) {
            let mut map = vec![u32::MAX; out.nodes.len()];
            let mut nodes = Vec::new();
            for (i, p) in out.nodes.iter().enumerate() {
                if used[i] {
                    map[i] = nodes.len() as u32;
                    nodes.push(*p);
                }
            }
            for t in out.tris.iter_mut() {
                for v in t.iter_mut() {
                    *v = map[*v as usize];
                }
            }
            out.nodes = nodes;
        }
        (out, dropped)
    }

    /// Make the winding of neighbouring triangles agree (flooding from the lowest-numbered triangle of each
    /// connected piece) and turn every piece with a negative volume inside out. Returns how many triangles
    /// were flipped. Triangles on a non-manifold edge are left as they are.
    pub fn orient(&mut self) -> usize {
        let n = self.tris.len();
        let mut edges: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
        for (k, t) in self.tris.iter().enumerate() {
            for i in 0..3 {
                let (a, b) = (t[i], t[(i + 1) % 3]);
                edges
                    .entry((a.min(b), a.max(b)))
                    .or_default()
                    .push(k as u32);
            }
        }
        let mut flip = vec![false; n];
        let mut seen = vec![false; n];
        let mut flipped = 0;
        for start in 0..n {
            if seen[start] {
                continue;
            }
            let mut piece = vec![start as u32];
            seen[start] = true;
            let mut head = 0;
            while head < piece.len() {
                let k = piece[head] as usize;
                head += 1;
                let t = self.tris[k];
                for i in 0..3 {
                    let (a, b) = (t[i], t[(i + 1) % 3]);
                    let list = &edges[&(a.min(b), a.max(b))];
                    if list.len() != 2 {
                        continue;
                    }
                    let o = if list[0] as usize == k {
                        list[1]
                    } else {
                        list[0]
                    } as usize;
                    if seen[o] {
                        continue;
                    }
                    seen[o] = true;
                    // the neighbour must run the shared edge the other way (taking flips into account)
                    let u = self.tris[o];
                    let same = (0..3).any(|j| u[j] == a && u[(j + 1) % 3] == b);
                    // k runs a→b unless it is flipped; o has to run the other way
                    flip[o] = if flip[k] { !same } else { same };
                    piece.push(o as u32);
                }
            }
            let vol: f64 = piece
                .iter()
                .map(|&k| {
                    let t = self.tris[k as usize];
                    let t = if flip[k as usize] {
                        [t[0], t[2], t[1]]
                    } else {
                        t
                    };
                    dot(
                        self.nodes[t[0] as usize],
                        cross(self.nodes[t[1] as usize], self.nodes[t[2] as usize]),
                    ) / 6.0
                })
                .sum();
            if vol < 0.0 {
                for &k in &piece {
                    flip[k as usize] = !flip[k as usize];
                }
            }
        }
        for (k, f) in flip.iter().enumerate() {
            if *f {
                self.tris[k].swap(1, 2);
                flipped += 1;
            }
        }
        flipped
    }

    /// Counts that say whether the mesh is a closed solid.
    pub fn stats(&self) -> MeshStats {
        let mut edges: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
        for t in &self.tris {
            for i in 0..3 {
                let (a, b) = (t[i], t[(i + 1) % 3]);
                let e = edges.entry((a.min(b), a.max(b))).or_default();
                if a < b {
                    e.0 += 1;
                } else {
                    e.1 += 1;
                }
            }
        }
        let (mut open, mut non_manifold, mut inconsistent) = (0, 0, 0);
        for &(f, r) in edges.values() {
            match (f, r) {
                (1, 1) => {}
                (1, 0) | (0, 1) => open += 1,
                (2, 0) | (0, 2) => inconsistent += 1,
                _ => non_manifold += 1,
            }
        }
        MeshStats {
            triangles: self.tris.len(),
            vertices: self.nodes.len(),
            open_edges: open,
            non_manifold_edges: non_manifold,
            inconsistent_edges: inconsistent,
            watertight: open == 0
                && non_manifold == 0
                && inconsistent == 0
                && !self.tris.is_empty(),
            volume: self.volume(),
            area: self.area(),
        }
    }

    /// Signed volume enclosed (positive for outward orientation).
    pub fn volume(&self) -> f64 {
        self.tris
            .iter()
            .map(|t| {
                let (a, b, c) = (
                    self.nodes[t[0] as usize],
                    self.nodes[t[1] as usize],
                    self.nodes[t[2] as usize],
                );
                dot(a, cross(b, c)) / 6.0
            })
            .sum()
    }

    pub fn area(&self) -> f64 {
        self.tris
            .iter()
            .map(|t| {
                let (a, b, c) = (
                    self.nodes[t[0] as usize],
                    self.nodes[t[1] as usize],
                    self.nodes[t[2] as usize],
                );
                0.5 * norm(cross(sub(b, a), sub(c, a)))
            })
            .sum()
    }

    /// Every directed edge must occur exactly once and be matched by its reverse.
    pub fn check_closed(&self) -> Result<(), String> {
        let mut he: HashMap<(u32, u32), u32> = HashMap::new();
        for (k, t) in self.tris.iter().enumerate() {
            for i in 0..3 {
                let e = (t[i], t[(i + 1) % 3]);
                if he.insert(e, k as u32).is_some() {
                    return Err(format!("directed edge {e:?} used twice"));
                }
            }
        }
        for (&(a, b), &k) in &he {
            if !he.contains_key(&(b, a)) {
                return Err(format!(
                    "edge ({a},{b}) of triangle {k} has no opposite half-edge (open or inconsistent surface)"
                ));
            }
        }
        Ok(())
    }

    pub fn bounds(&self) -> (P3, P3) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in &self.nodes {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        (lo, hi)
    }

    /// Axis-aligned box `[0,lx]×[0,ly]×[0,lz]`; tags 0–5 are x−, x+, y−, y+, z−, z+.
    pub fn cuboid(lx: f64, ly: f64, lz: f64) -> SurfaceMesh {
        let nodes: Vec<P3> = (0..8)
            .map(|i| {
                [
                    if i & 1 == 1 { lx } else { 0.0 },
                    if i & 2 == 2 { ly } else { 0.0 },
                    if i & 4 == 4 { lz } else { 0.0 },
                ]
            })
            .collect();
        // quads listed counter-clockwise seen from outside
        let quads: [([u32; 4], u32); 6] = [
            ([0, 4, 6, 2], 0),
            ([1, 3, 7, 5], 1),
            ([0, 1, 5, 4], 2),
            ([2, 6, 7, 3], 3),
            ([0, 2, 3, 1], 4),
            ([4, 5, 7, 6], 5),
        ];
        let mut s = SurfaceMesh {
            nodes,
            ..Default::default()
        };
        for (q, tag) in quads {
            s.tris.push([q[0], q[1], q[2]]);
            s.tags.push(tag);
            s.tris.push([q[0], q[2], q[3]]);
            s.tags.push(tag);
        }
        s
    }

    /// Icosphere of radius `r` centred at `c` after `level` subdivisions (all one tag).
    pub fn sphere(c: P3, r: f64, level: usize) -> SurfaceMesh {
        let t = (1.0 + 5.0f64.sqrt()) / 2.0;
        let mut nodes: Vec<P3> = vec![
            [-1.0, t, 0.0],
            [1.0, t, 0.0],
            [-1.0, -t, 0.0],
            [1.0, -t, 0.0],
            [0.0, -1.0, t],
            [0.0, 1.0, t],
            [0.0, -1.0, -t],
            [0.0, 1.0, -t],
            [t, 0.0, -1.0],
            [t, 0.0, 1.0],
            [-t, 0.0, -1.0],
            [-t, 0.0, 1.0],
        ];
        let mut tris: Vec<[u32; 3]> = vec![
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        let unit = |p: P3| {
            let n = norm(p);
            [p[0] / n, p[1] / n, p[2] / n]
        };
        for n in nodes.iter_mut() {
            *n = unit(*n);
        }
        for _ in 0..level {
            let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
            let mut next = Vec::with_capacity(tris.len() * 4);
            for t in &tris {
                let mut m = [0u32; 3];
                for i in 0..3 {
                    let (a, b) = (t[i], t[(i + 1) % 3]);
                    let key = (a.min(b), a.max(b));
                    m[i] = *mid.entry(key).or_insert_with(|| {
                        let (p, q) = (nodes[a as usize], nodes[b as usize]);
                        nodes.push(unit([
                            0.5 * (p[0] + q[0]),
                            0.5 * (p[1] + q[1]),
                            0.5 * (p[2] + q[2]),
                        ]));
                        (nodes.len() - 1) as u32
                    });
                }
                next.push([t[0], m[0], m[2]]);
                next.push([t[1], m[1], m[0]]);
                next.push([t[2], m[2], m[1]]);
                next.push([m[0], m[1], m[2]]);
            }
            tris = next;
        }
        let nodes: Vec<P3> = nodes
            .iter()
            .map(|p| [c[0] + r * p[0], c[1] + r * p[1], c[2] + r * p[2]])
            .collect();
        let tags = vec![0; tris.len()];
        SurfaceMesh { nodes, tris, tags }
    }

    /// Cylinder along z from 0 to `h` with `n` segments round and `rows` along the side; tags 0 side,
    /// 1 bottom, 2 top. The caps are fans round the axis point.
    pub fn cylinder(r: f64, h: f64, n: usize, rows: usize) -> SurfaceMesh {
        let mut s = SurfaceMesh::default();
        for j in 0..=rows {
            for i in 0..n {
                let a = std::f64::consts::TAU * i as f64 / n as f64;
                s.nodes
                    .push([r * a.cos(), r * a.sin(), h * j as f64 / rows as f64]);
            }
        }
        let ring = |j: usize, i: usize| (j * n + i % n) as u32;
        for j in 0..rows {
            for i in 0..n {
                let (a, b, c, d) = (
                    ring(j, i),
                    ring(j, i + 1),
                    ring(j + 1, i + 1),
                    ring(j + 1, i),
                );
                s.tris.push([a, b, c]);
                s.tags.push(0);
                s.tris.push([a, c, d]);
                s.tags.push(0);
            }
        }
        let bottom = s.nodes.len() as u32;
        s.nodes.push([0.0, 0.0, 0.0]);
        let top = s.nodes.len() as u32;
        s.nodes.push([0.0, 0.0, h]);
        for i in 0..n {
            s.tris.push([bottom, ring(0, i + 1), ring(0, i)]);
            s.tags.push(1);
            s.tris.push([top, ring(rows, i), ring(rows, i + 1)]);
            s.tags.push(2);
        }
        s
    }
}

/// Circumscribed sphere of a triangle in 3-D: centre in the plane of the triangle. Degenerate (zero-area)
/// triangles give an infinite radius.
pub fn tri_ball(a: P3, b: P3, c: P3) -> (P3, f64) {
    let u = sub(b, a);
    let v = sub(c, a);
    let w = cross(u, v);
    let w2 = dot(w, w);
    if w2 <= 1e-300 {
        return (
            [
                (a[0] + b[0] + c[0]) / 3.0,
                (a[1] + b[1] + c[1]) / 3.0,
                (a[2] + b[2] + c[2]) / 3.0,
            ],
            f64::INFINITY,
        );
    }
    let (u2, v2) = (dot(u, u), dot(v, v));
    let vw = cross(v, w);
    let wu = cross(w, u);
    let x = [
        (u2 * vw[0] + v2 * wu[0]) / (2.0 * w2),
        (u2 * vw[1] + v2 * wu[1]) / (2.0 * w2),
        (u2 * vw[2] + v2 * wu[2]) / (2.0 * w2),
    ];
    ([a[0] + x[0], a[1] + x[1], a[2] + x[2]], norm(x))
}

/// The surface under refinement: triangles with half-edge lookup.
#[derive(Debug, Clone, Default)]
pub struct Surf {
    pub nodes: Vec<P3>,
    pub tris: Vec<[u32; 3]>,
    pub tag: Vec<u32>,
    pub alive: Vec<bool>,
    he: HashMap<(u32, u32), u32>,
    by_key: HashMap<[u32; 3], u32>,
    pub n_alive: usize,
}

pub fn sorted3(t: [u32; 3]) -> [u32; 3] {
    let mut k = t;
    k.sort_unstable();
    k
}

impl Surf {
    pub fn new(s: &SurfaceMesh) -> Surf {
        let mut out = Surf {
            nodes: s.nodes.clone(),
            ..Default::default()
        };
        for (t, &tag) in s.tris.iter().zip(&s.tags) {
            out.add_tri(*t, tag);
        }
        out
    }

    pub fn add_tri(&mut self, t: [u32; 3], tag: u32) -> u32 {
        let id = self.tris.len() as u32;
        self.tris.push(t);
        self.tag.push(tag);
        self.alive.push(true);
        for i in 0..3 {
            self.he.insert((t[i], t[(i + 1) % 3]), id);
        }
        self.by_key.insert(sorted3(t), id);
        self.n_alive += 1;
        id
    }

    fn kill(&mut self, id: u32) {
        let t = self.tris[id as usize];
        for i in 0..3 {
            self.he.remove(&(t[i], t[(i + 1) % 3]));
        }
        self.by_key.remove(&sorted3(t));
        self.alive[id as usize] = false;
        self.n_alive -= 1;
    }

    pub fn tri_by_key(&self, key: &[u32; 3]) -> Option<u32> {
        self.by_key.get(key).copied()
    }

    /// The triangles on the two sides of the edge: `(containing u→v, containing v→u)`.
    pub fn edge_tris(&self, u: u32, v: u32) -> Option<(u32, u32)> {
        Some((*self.he.get(&(u, v))?, *self.he.get(&(v, u))?))
    }

    pub fn pos(&self, v: u32) -> P3 {
        self.nodes[v as usize]
    }

    pub fn corners(&self, t: u32) -> [P3; 3] {
        let tr = self.tris[t as usize];
        [self.pos(tr[0]), self.pos(tr[1]), self.pos(tr[2])]
    }

    pub fn ball(&self, t: u32) -> (P3, f64) {
        let c = self.corners(t);
        tri_ball(c[0], c[1], c[2])
    }

    /// Longest edge `(u, v, length)` of triangle `t`.
    pub fn longest_edge(&self, t: u32) -> (u32, u32, f64) {
        let tr = self.tris[t as usize];
        let mut best = (tr[0], tr[1], -1.0);
        for i in 0..3 {
            let (a, b) = (tr[i], tr[(i + 1) % 3]);
            let l = norm(sub(self.pos(a), self.pos(b)));
            if l > best.2 {
                best = (a, b, l);
            }
        }
        best
    }

    /// Split edge `u–v` at the new node `m` (already appended to `nodes`). Returns the removed and the
    /// created triangle ids.
    pub fn bisect(&mut self, u: u32, v: u32, m: u32) -> ([u32; 2], [u32; 4]) {
        let (t1, t2) = self.edge_tris(u, v).expect("edge of a closed surface");
        let tag1 = self.tag[t1 as usize];
        let tag2 = self.tag[t2 as usize];
        let w1 = apex(self.tris[t1 as usize], u, v);
        let w2 = apex(self.tris[t2 as usize], v, u);
        self.kill(t1);
        self.kill(t2);
        let n = [
            self.add_tri([u, m, w1], tag1),
            self.add_tri([m, v, w1], tag1),
            self.add_tri([v, m, w2], tag2),
            self.add_tri([m, u, w2], tag2),
        ];
        ([t1, t2], n)
    }

    /// Split triangle `t` at the new node `m` (already appended) into three. Returns the removed id and the
    /// created ids.
    pub fn split_face(&mut self, t: u32, m: u32) -> (u32, [u32; 3]) {
        let tr = self.tris[t as usize];
        let tag = self.tag[t as usize];
        self.kill(t);
        let n = [
            self.add_tri([tr[0], tr[1], m], tag),
            self.add_tri([tr[1], tr[2], m], tag),
            self.add_tri([tr[2], tr[0], m], tag),
        ];
        (t, n)
    }

    /// Flip the edge `a→b` shared by two triangles: the pair `(a,b,c)`, `(b,a,d)` becomes `(a,d,c)`,
    /// `(d,b,c)`. Returns `(killed, created)`. The caller checks that the quad is convex.
    pub fn flip(&mut self, a: u32, b: u32) -> Option<([u32; 2], [u32; 2])> {
        let (t, u) = self.edge_tris(a, b)?;
        let c = apex(self.tris[t as usize], a, b);
        let d = apex(self.tris[u as usize], b, a);
        let (tag_t, tag_u) = (self.tag[t as usize], self.tag[u as usize]);
        if c == d {
            return None;
        }
        self.kill(t);
        self.kill(u);
        let n1 = self.add_tri([a, d, c], tag_t);
        let n2 = self.add_tri([d, b, c], tag_u);
        Some(([t, u], [n1, n2]))
    }

    /// The vertex of live triangle `t` opposite the directed edge `a→b`.
    pub fn apex_of(&self, t: u32, a: u32, b: u32) -> u32 {
        apex(self.tris[t as usize], a, b)
    }

    pub fn alive_tris(&self) -> impl Iterator<Item = u32> + '_ {
        (0..self.tris.len() as u32).filter(|&t| self.alive[t as usize])
    }

    /// Outward orientation of a live triangle given by its sorted key.
    pub fn oriented(&self, key: &[u32; 3]) -> Option<([u32; 3], u32)> {
        let id = self.tri_by_key(key)?;
        Some((self.tris[id as usize], self.tag[id as usize]))
    }
}

/// The vertex of triangle `t` that is neither `a` nor `b` (the triangle contains the directed edge a→b).
fn apex(t: [u32; 3], a: u32, b: u32) -> u32 {
    for i in 0..3 {
        if t[i] == a && t[(i + 1) % 3] == b {
            return t[(i + 2) % 3];
        }
    }
    unreachable!("triangle {t:?} does not contain the edge {a}→{b}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn soup_of(m: &SurfaceMesh) -> Vec<[P3; 3]> {
        m.tris
            .iter()
            .map(|t| {
                [
                    m.nodes[t[0] as usize],
                    m.nodes[t[1] as usize],
                    m.nodes[t[2] as usize],
                ]
            })
            .collect()
    }

    #[test]
    fn a_soup_is_welded_and_degenerate_triangles_are_dropped() {
        let b = SurfaceMesh::cuboid(2.0, 3.0, 4.0);
        let mut soup = soup_of(&b);
        // jitter below the tolerance
        for (i, t) in soup.iter_mut().enumerate() {
            for p in t.iter_mut() {
                p[0] += 1e-9 * (i % 3) as f64;
            }
        }
        // a sliver and a repeated corner
        soup.push([[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
        soup.push([[5.0; 3], [5.0; 3], [6.0, 5.0, 5.0]]);
        let (m, dropped) = SurfaceMesh::from_soup(&soup, 1e-6);
        assert_eq!(dropped, 2);
        assert_eq!((m.nodes.len(), m.tris.len()), (8, 12));
        let st = m.stats();
        assert!(st.watertight, "{st:?}");
        assert!((st.volume - 24.0).abs() < 1e-6);
        // the order of the nodes is the order of first appearance, run after run
        let (again, _) = SurfaceMesh::from_soup(&soup, 1e-6);
        assert_eq!(m.nodes, again.nodes);
        assert_eq!(m.tris, again.tris);
    }

    #[test]
    fn orient_makes_mixed_and_inverted_windings_outward() {
        let b = SurfaceMesh::cuboid(2.0, 3.0, 4.0);
        // every triangle reversed: inside out
        let mut inv = b.clone();
        for t in inv.tris.iter_mut() {
            t.swap(1, 2);
        }
        assert!(inv.stats().volume < 0.0);
        assert_eq!(inv.orient(), 12);
        assert!((inv.volume() - 24.0).abs() < 1e-9 && inv.stats().watertight);
        // some triangles reversed
        let mut mixed = b.clone();
        for k in [0, 3, 4, 9] {
            mixed.tris[k].swap(1, 2);
        }
        assert!(!mixed.stats().watertight);
        mixed.orient();
        assert!((mixed.volume() - 24.0).abs() < 1e-9 && mixed.stats().watertight);
        // two separate pieces, one of them inside out
        let mut two = b.clone();
        let n = two.nodes.len() as u32;
        for p in &b.nodes {
            two.nodes.push([p[0] + 10.0, p[1], p[2]]);
        }
        for t in &b.tris {
            two.tris.push([t[0] + n, t[2] + n, t[1] + n]);
            two.tags.push(0);
        }
        two.orient();
        assert!((two.volume() - 48.0).abs() < 1e-9);
    }

    #[test]
    fn stats_tell_open_and_non_manifold_meshes_apart() {
        let b = SurfaceMesh::cuboid(1.0, 1.0, 1.0);
        let st = b.stats();
        assert!(st.watertight);
        assert_eq!((st.open_edges, st.non_manifold_edges), (0, 0));
        let mut open = b.clone();
        open.tris.pop();
        open.tags.pop();
        let st = open.stats();
        assert!(!st.watertight);
        assert_eq!(st.open_edges, 3);
        // a third triangle on one edge
        let mut fin = b.clone();
        fin.nodes.push([0.5, 0.5, 5.0]);
        let (a, c) = (fin.tris[0][0], fin.tris[0][1]);
        fin.tris.push([a, c, 8]);
        fin.tags.push(0);
        let st = fin.stats();
        assert!(st.non_manifold_edges >= 1 && !st.watertight);
    }

    #[test]
    fn builders_are_closed_and_outward() {
        let b = SurfaceMesh::cuboid(2.0, 3.0, 4.0);
        b.check_closed().unwrap();
        assert!((b.volume() - 24.0).abs() < 1e-12);
        let s = SurfaceMesh::sphere([1.0, 2.0, 3.0], 1.5, 2);
        s.check_closed().unwrap();
        let ideal = 4.0 / 3.0 * std::f64::consts::PI * 1.5f64.powi(3);
        assert!(s.volume() > 0.0 && (s.volume() / ideal - 1.0).abs() < 0.05);
        let c = SurfaceMesh::cylinder(1.0, 2.0, 24, 3);
        c.check_closed().unwrap();
        assert!(c.volume() > 0.0);
    }

    #[test]
    fn circumball_is_equidistant_from_the_corners() {
        let (a, b, c) = ([0.1, 0.2, 0.3], [2.0, 0.4, -0.5], [0.7, 1.9, 1.1]);
        let (o, r) = tri_ball(a, b, c);
        for p in [a, b, c] {
            assert!((norm(sub(p, o)) - r).abs() < 1e-12);
        }
        let n = cross(sub(b, a), sub(c, a));
        assert!(dot(sub(o, a), n).abs() < 1e-12);
    }

    #[test]
    fn bisection_keeps_the_surface_closed() {
        let s = SurfaceMesh::cuboid(1.0, 1.0, 1.0);
        let mut surf = Surf::new(&s);
        let t0 = surf.alive_tris().next().unwrap();
        let tr = surf.tris[t0 as usize];
        let m = surf.nodes.len() as u32;
        let (p, q) = (surf.pos(tr[0]), surf.pos(tr[1]));
        surf.nodes.push([
            0.5 * (p[0] + q[0]),
            0.5 * (p[1] + q[1]),
            0.5 * (p[2] + q[2]),
        ]);
        let ([a, b], new) = surf.bisect(tr[0], tr[1], m);
        assert!(!surf.alive[a as usize] && !surf.alive[b as usize]);
        assert!(new.iter().all(|&t| surf.alive[t as usize]));
        let out = SurfaceMesh {
            nodes: surf.nodes.clone(),
            tris: surf.alive_tris().map(|t| surf.tris[t as usize]).collect(),
            tags: surf.alive_tris().map(|t| surf.tag[t as usize]).collect(),
        };
        out.check_closed().unwrap();
        assert!((out.volume() - 1.0).abs() < 1e-12);
    }
}

/// What [`SurfaceMesh::stats`] counts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshStats {
    pub triangles: usize,
    pub vertices: usize,
    /// Edges used by one triangle only.
    pub open_edges: usize,
    /// Edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Edges two triangles run in the same direction (a flipped neighbour).
    pub inconsistent_edges: usize,
    /// Every edge has exactly two triangles that run it in opposite directions.
    pub watertight: bool,
    pub volume: f64,
    pub area: f64,
}
