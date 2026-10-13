//! Tetrahedral triangulation with adjacency, exact predicates and Bowyer–Watson cavity insertion.
//!
//! A face whose neighbour is [`NONE`] is a boundary: of the super-tetrahedron while the Delaunay
//! triangulation of the surface vertices is built, and of the solid afterwards. Cavities never cross a
//! boundary face, so boundary faces can never be destroyed by an insertion.

use crate::P3;
use robust::{insphere, orient3d, Coord3D};
use std::collections::{HashMap, HashSet};

pub const NONE: u32 = u32::MAX;

/// Vertices (as local indices) of the face opposite local vertex `i`, ordered so that the right-hand
/// normal points towards vertex `i` (a positively oriented tet has `orient(face, v_i) > 0`).
pub const FACE: [[usize; 3]; 4] = [[1, 3, 2], [0, 2, 3], [0, 3, 1], [0, 1, 2]];

#[inline]
fn co(p: &P3) -> Coord3D<f64> {
    Coord3D {
        x: p[0],
        y: p[1],
        z: p[2],
    }
}

/// Exact sign of the signed volume of `abcd`: positive when `d` lies on the side of `abc` its
/// right-hand normal points to.
#[inline]
pub fn orient(a: &P3, b: &P3, c: &P3, d: &P3) -> f64 {
    -orient3d(co(a), co(b), co(c), co(d))
}

/// Positive when `e` lies strictly inside the circumsphere of the positively oriented tet `abcd`.
#[inline]
pub fn in_sphere(a: &P3, b: &P3, c: &P3, d: &P3, e: &P3) -> f64 {
    -insphere(co(a), co(b), co(c), co(d), co(e))
}

#[derive(Debug, Clone, Copy)]
pub struct Tet {
    pub v: [u32; 4],
    /// `n[i]` is the tet across the face opposite `v[i]` ([`NONE`] on the boundary).
    pub n: [u32; 4],
    pub alive: bool,
    mark: u32,
    rej: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertError {
    /// The point coincides with an existing vertex.
    Duplicate,
    /// The point lies outside the triangulation.
    Outside,
    /// A forced cavity tet is not strictly visible from the point (the point is not where it claims to be).
    NotStarShaped,
    /// Some vertex of the cavity would have been swallowed.
    LostVertex,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Loc {
    Inside(u32),
    OnFace(u32, usize),
    /// On the edge between the two local vertices.
    OnEdge(u32, [usize; 2]),
    OnVertex(u32, usize),
    /// The walk towards the point ran into a boundary face (tet, local face).
    Blocked(u32, usize),
    /// Not inside any tet (only reported by the exhaustive search).
    Outside,
}

#[derive(Debug, Clone)]
pub struct Tri3 {
    pub pts: Vec<P3>,
    pub tets: Vec<Tet>,
    free: Vec<u32>,
    vtet: Vec<u32>,
    hint: u32,
    stamp: u32,
    rng: u64,
    live: usize,
}

impl Tri3 {
    fn empty(pts: Vec<P3>) -> Tri3 {
        let n = pts.len();
        Tri3 {
            pts,
            tets: Vec::new(),
            free: Vec::new(),
            vtet: vec![NONE; n],
            hint: 0,
            stamp: 0,
            rng: 0x9E37_79B9_7F4A_7C15,
            live: 0,
        }
    }

    /// Triangulation of `pts` inside a huge super-tetrahedron whose four vertices are appended after the
    /// points (indices `pts.len() .. pts.len()+4`). Nothing is inserted yet.
    pub fn with_super(pts: &[P3]) -> Tri3 {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in pts {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        if pts.is_empty() {
            lo = [0.0; 3];
            hi = [1.0; 3];
        }
        let h = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max).max(1e-9);
        let o = [lo[0] - 2.0 * h, lo[1] - 2.0 * h, lo[2] - 2.0 * h];
        let s = 1000.0 * h;
        let mut all = pts.to_vec();
        let base = all.len() as u32;
        all.push(o);
        all.push([o[0] + s, o[1], o[2]]);
        all.push([o[0], o[1] + s, o[2]]);
        all.push([o[0], o[1], o[2] + s]);
        let mut t = Tri3::empty(all);
        let id = t.alloc(Tet {
            v: [base, base + 1, base + 2, base + 3],
            n: [NONE; 4],
            alive: true,
            mark: 0,
            rej: 0,
        });
        for k in 0..4 {
            t.vtet[(base + k) as usize] = id;
        }
        t.hint = id;
        t
    }

    /// A triangulation with the given points and tets (neighbours are derived from shared faces; unmatched
    /// faces become boundary).
    pub fn from_tets(pts: Vec<P3>, tets: &[[u32; 4]]) -> Tri3 {
        let mut t = Tri3::empty(pts);
        let mut faces: HashMap<[u32; 3], (u32, usize)> = HashMap::with_capacity(tets.len() * 2);
        for tv in tets {
            let id = t.alloc(Tet {
                v: *tv,
                n: [NONE; 4],
                alive: true,
                mark: 0,
                rej: 0,
            });
            for i in 0..4 {
                t.vtet[tv[i] as usize] = id;
                let f = FACE[i];
                let mut key = [tv[f[0]], tv[f[1]], tv[f[2]]];
                key.sort_unstable();
                if let Some((o, oi)) = faces.remove(&key) {
                    t.tets[id as usize].n[i] = o;
                    t.tets[o as usize].n[oi] = id;
                } else {
                    faces.insert(key, (id, i));
                }
            }
        }
        t.hint = 0;
        t
    }

    pub fn len(&self) -> usize {
        self.live
    }
    pub fn is_empty(&self) -> bool {
        self.live == 0
    }

    pub fn alloc(&mut self, tet: Tet) -> u32 {
        self.live += 1;
        if let Some(id) = self.free.pop() {
            self.tets[id as usize] = tet;
            id
        } else {
            self.tets.push(tet);
            (self.tets.len() - 1) as u32
        }
    }

    fn release(&mut self, id: u32) {
        self.tets[id as usize].alive = false;
        self.free.push(id);
        self.live -= 1;
    }

    /// Append a point; it belongs to no tet yet.
    pub fn add_point(&mut self, p: P3) -> u32 {
        self.pts.push(p);
        self.vtet.push(NONE);
        (self.pts.len() - 1) as u32
    }

    /// Remove the most recently added point (it must not be in any tet).
    pub fn drop_last_point(&mut self) {
        self.pts.pop();
        self.vtet.pop();
    }

    pub fn hint(&self) -> u32 {
        self.hint
    }

    pub fn vertex_tet(&self, v: u32) -> u32 {
        self.vtet[v as usize]
    }

    #[inline]
    pub fn corner(&self, t: u32, i: usize) -> &P3 {
        &self.pts[self.tets[t as usize].v[i] as usize]
    }

    /// The three vertex ids of face `i` of tet `t` (normal towards `v[i]`).
    #[inline]
    pub fn face_ids(&self, t: u32, i: usize) -> [u32; 3] {
        let tv = &self.tets[t as usize].v;
        let f = FACE[i];
        [tv[f[0]], tv[f[1]], tv[f[2]]]
    }

    fn next_rand(&mut self) -> u64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        self.rng
    }

    fn sphere_test(&self, t: u32, p: &P3) -> f64 {
        let v = &self.tets[t as usize].v;
        in_sphere(
            &self.pts[v[0] as usize],
            &self.pts[v[1] as usize],
            &self.pts[v[2] as usize],
            &self.pts[v[3] as usize],
            p,
        )
    }

    /// Is `p` strictly on the inner side of face `i` of `t`, and by enough that the tet formed by the face
    /// and `p` is not numerically flat (`flat_tol`, see [`cavity_insert_guarded`])?
    fn visible(&self, t: u32, i: usize, p: &P3, flat_tol: f64) -> bool {
        let d = self.face_side(t, i, p);
        if d <= 0.0 {
            return false;
        }
        if flat_tol <= 0.0 {
            return true;
        }
        let f = self.face_ids(t, i);
        let q = [
            &self.pts[f[0] as usize],
            &self.pts[f[1] as usize],
            &self.pts[f[2] as usize],
            p,
        ];
        let mut e2 = 0.0f64;
        for a in 0..4 {
            for b in a + 1..4 {
                let (x, y) = (q[a], q[b]);
                let l2 = (x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2);
                e2 = e2.max(l2);
            }
        }
        d >= flat_tol * e2 * e2.sqrt()
    }

    fn face_side(&self, t: u32, i: usize, p: &P3) -> f64 {
        let f = self.face_ids(t, i);
        orient(
            &self.pts[f[0] as usize],
            &self.pts[f[1] as usize],
            &self.pts[f[2] as usize],
            p,
        )
    }

    /// Walk from `start` towards `p`. Never leaves the triangulation: a boundary face in the way is
    /// reported as [`Loc::Blocked`].
    pub fn locate(&mut self, p: &P3, start: u32) -> Loc {
        let mut cur = if (start as usize) < self.tets.len() && self.tets[start as usize].alive {
            start
        } else if self.tets[self.hint as usize].alive {
            self.hint
        } else {
            match self.tets.iter().position(|t| t.alive) {
                Some(i) => i as u32,
                None => return Loc::Outside,
            }
        };
        let cap = 8 * self.tets.len() + 100;
        for _ in 0..cap {
            let mut s = [0.0f64; 4];
            let mut cand = [0usize; 4];
            let mut k = 0;
            for i in 0..4 {
                s[i] = self.face_side(cur, i, p);
                if s[i] < 0.0 {
                    cand[k] = i;
                    k += 1;
                }
            }
            if k == 0 {
                self.hint = cur;
                return classify(cur, &s);
            }
            let tet = self.tets[cur as usize];
            let mut open = [0usize; 4];
            let mut m = 0;
            for &i in &cand[..k] {
                if tet.n[i] != NONE {
                    open[m] = i;
                    m += 1;
                }
            }
            if m == 0 {
                return Loc::Blocked(cur, cand[0]);
            }
            let pick = open[(self.next_rand() % m as u64) as usize];
            cur = tet.n[pick];
        }
        self.locate_scan(p)
    }

    /// Exhaustive point location (fallback).
    pub fn locate_scan(&self, p: &P3) -> Loc {
        for (id, t) in self.tets.iter().enumerate() {
            if !t.alive {
                continue;
            }
            let mut s = [0.0f64; 4];
            let mut ok = true;
            for i in 0..4 {
                s[i] = self.face_side(id as u32, i, p);
                if s[i] < 0.0 {
                    ok = false;
                    break;
                }
            }
            if ok {
                return classify(id as u32, &s);
            }
        }
        Loc::Outside
    }

    /// All tets containing both vertices of local edge `(a, b)` of tet `t`, found by walking round the edge.
    pub fn edge_fan(&mut self, t: u32, ids: [u32; 2]) -> Vec<u32> {
        self.stamp += 1;
        let st = self.stamp;
        let mut out = vec![t];
        self.tets[t as usize].mark = st;
        let mut head = 0;
        while head < out.len() {
            let cur = out[head];
            head += 1;
            let tet = self.tets[cur as usize];
            for i in 0..4 {
                // faces containing both ids are the ones opposite the other two vertices
                if tet.v[i] == ids[0] || tet.v[i] == ids[1] {
                    continue;
                }
                let nb = tet.n[i];
                if nb != NONE && self.tets[nb as usize].mark != st {
                    self.tets[nb as usize].mark = st;
                    out.push(nb);
                }
            }
        }
        out
    }

    /// All tets incident to vertex `v`.
    pub fn star(&mut self, v: u32) -> Vec<u32> {
        let start = self.vtet[v as usize];
        if start == NONE {
            return Vec::new();
        }
        self.stamp += 1;
        let st = self.stamp;
        let mut out = vec![start];
        self.tets[start as usize].mark = st;
        let mut head = 0;
        while head < out.len() {
            let cur = out[head];
            head += 1;
            let tet = self.tets[cur as usize];
            for i in 0..4 {
                if tet.v[i] == v {
                    continue;
                }
                let nb = tet.n[i];
                if nb != NONE && self.tets[nb as usize].mark != st {
                    self.tets[nb as usize].mark = st;
                    out.push(nb);
                }
            }
        }
        out
    }

    /// Tets that contain both `u` and `v` (the fan round the edge), if the edge exists.
    pub fn fan_of_edge(&mut self, u: u32, v: u32) -> Vec<u32> {
        let s = self.star(u);
        let first = s
            .into_iter()
            .find(|&t| self.tets[t as usize].v.contains(&v));
        match first {
            Some(t) => self.edge_fan(t, [u, v]),
            None => Vec::new(),
        }
    }

    /// Insert vertex `vid` (already added with [`add_point`]) wherever it falls.
    pub fn insert_vertex(&mut self, vid: u32) -> Result<Vec<u32>, InsertError> {
        let p = self.pts[vid as usize];
        let loc = self.locate(&p, self.hint);
        self.insert_at(vid, loc)
    }

    pub fn insert_at(&mut self, vid: u32, loc: Loc) -> Result<Vec<u32>, InsertError> {
        self.insert_at_guarded(vid, loc, 0.0)
    }

    pub fn insert_at_guarded(
        &mut self,
        vid: u32,
        loc: Loc,
        flat_tol: f64,
    ) -> Result<Vec<u32>, InsertError> {
        let seeds: Vec<u32> = match loc {
            Loc::Inside(t) => vec![t],
            Loc::OnFace(t, i) => {
                let nb = self.tets[t as usize].n[i];
                if nb == NONE {
                    // on the boundary face: the single tet is the cavity
                    vec![t]
                } else {
                    vec![t, nb]
                }
            }
            Loc::OnEdge(t, [a, b]) => {
                let ids = [self.tets[t as usize].v[a], self.tets[t as usize].v[b]];
                self.edge_fan(t, ids)
            }
            Loc::OnVertex(..) => return Err(InsertError::Duplicate),
            Loc::Blocked(..) | Loc::Outside => return Err(InsertError::Outside),
        };
        self.cavity_insert_guarded(vid, &seeds, &[], flat_tol)
    }

    /// Bowyer–Watson insertion of vertex `vid` with the given forced seed tets. The cavity grows over
    /// non-boundary faces to neighbours whose circumsphere contains the point, is then trimmed until every
    /// cavity boundary face is strictly visible from the point (faces containing all of `skip` on the domain boundary
    /// are exempt: the point lies on that edge or face), and is re-triangulated by connecting the point to it.
    pub fn cavity_insert(
        &mut self,
        vid: u32,
        seeds: &[u32],
        skip: &[u32],
    ) -> Result<Vec<u32>, InsertError> {
        self.cavity_insert_guarded(vid, seeds, skip, 0.0)
    }

    /// As [`cavity_insert`], but every new tet must have `6·volume ≥ flat_tol · (longest edge of its
    /// base face)³`, which keeps numerically flat tets from being created.
    pub fn cavity_insert_guarded(
        &mut self,
        vid: u32,
        seeds: &[u32],
        skip: &[u32],
        flat_tol: f64,
    ) -> Result<Vec<u32>, InsertError> {
        let p = self.pts[vid as usize];
        self.stamp += 1;
        let st = self.stamp;
        let mut cav: Vec<u32> = Vec::new();
        for &s in seeds {
            if self.tets[s as usize].mark != st {
                self.tets[s as usize].mark = st;
                cav.push(s);
            }
        }
        let forced: HashSet<u32> = cav.iter().copied().collect();
        let mut head = 0;
        while head < cav.len() {
            let t = cav[head];
            head += 1;
            for i in 0..4 {
                let nb = self.tets[t as usize].n[i];
                if nb == NONE {
                    continue;
                }
                let nt = self.tets[nb as usize];
                if nt.mark == st || nt.rej == st {
                    continue;
                }
                if self.sphere_test(nb, &p) > 0.0 {
                    self.tets[nb as usize].mark = st;
                    cav.push(nb);
                } else {
                    self.tets[nb as usize].rej = st;
                }
            }
        }
        self.finish_cavity(vid, cav, forced, skip, flat_tol, st)
    }

    /// Boundary faces containing every id of `skip` are exempt from the visibility test and get no tet: the
    /// point lies on that edge or face of the domain boundary, which the insertion replaces.
    fn is_skipped(&self, t: u32, i: usize, skip: &[u32]) -> bool {
        if skip.is_empty() {
            return false;
        }
        let f = self.face_ids(t, i);
        skip.iter().all(|s| f.contains(s))
    }

    fn finish_cavity(
        &mut self,
        vid: u32,
        mut cav: Vec<u32>,
        forced: HashSet<u32>,
        skip: &[u32],
        flat_tol: f64,
        mut st: u32,
    ) -> Result<Vec<u32>, InsertError> {
        let p = self.pts[vid as usize];
        // trim until every boundary face is strictly visible from the point (pruning may repeat the trim)
        loop {
            let mut remove: Vec<u32> = Vec::new();
            for &t in &cav {
                if self.tets[t as usize].mark != st {
                    continue;
                }
                for i in 0..4 {
                    let nb = self.tets[t as usize].n[i];
                    if nb != NONE && self.tets[nb as usize].mark == st {
                        continue;
                    }
                    if nb == NONE && self.is_skipped(t, i, skip) {
                        continue;
                    }
                    if !self.visible(t, i, &p, flat_tol) {
                        if forced.contains(&t) {
                            return Err(InsertError::NotStarShaped);
                        }
                        remove.push(t);
                        break;
                    }
                }
            }
            if remove.is_empty() {
                break;
            }
            for t in remove {
                self.tets[t as usize].mark = 0;
            }
            self.stamp += 1;
            let st2 = self.stamp;
            let mut keep: Vec<u32> = Vec::new();
            for &t in &cav {
                if forced.contains(&t) && self.tets[t as usize].mark == st {
                    self.tets[t as usize].mark = st2;
                    keep.push(t);
                }
            }
            let mut h = 0;
            while h < keep.len() {
                let t = keep[h];
                h += 1;
                for i in 0..4 {
                    let nb = self.tets[t as usize].n[i];
                    if nb != NONE && self.tets[nb as usize].mark == st {
                        self.tets[nb as usize].mark = st2;
                        keep.push(nb);
                    }
                }
            }
            for &t in &cav {
                if self.tets[t as usize].mark == st {
                    self.tets[t as usize].mark = 0;
                }
            }
            cav = keep;
            st = st2;
        }
        // boundary faces of the cavity
        struct Bf {
            t: u32,
            i: usize,
            nb: u32,
        }
        let mut bfs: Vec<Bf> = Vec::new();
        let mut seen_vertices: HashSet<u32> = HashSet::new();
        let mut cavity_vertices: HashSet<u32> = HashSet::new();
        for &t in &cav {
            for k in 0..4 {
                cavity_vertices.insert(self.tets[t as usize].v[k]);
            }
            for i in 0..4 {
                let nb = self.tets[t as usize].n[i];
                if nb != NONE && self.tets[nb as usize].mark == st {
                    continue;
                }
                for x in self.face_ids(t, i) {
                    seen_vertices.insert(x);
                }
                if nb == NONE && self.is_skipped(t, i, skip) {
                    continue;
                }
                bfs.push(Bf { t, i, nb });
            }
        }
        if cavity_vertices.iter().any(|v| !seen_vertices.contains(v)) {
            return Err(InsertError::LostVertex);
        }
        // commit
        let mut new_ids: Vec<u32> = Vec::with_capacity(bfs.len());
        let mut glue: HashMap<(u32, u32), (u32, usize)> = HashMap::with_capacity(bfs.len() * 2);
        for b in &bfs {
            let f = self.face_ids(b.t, b.i);
            let tv = [f[0], f[1], f[2], vid];
            let id = self.alloc(Tet {
                v: tv,
                n: [NONE, NONE, NONE, b.nb],
                alive: true,
                mark: 0,
                rej: 0,
            });
            new_ids.push(id);
            if b.nb != NONE {
                let nbt = &mut self.tets[b.nb as usize];
                for j in 0..4 {
                    if nbt.n[j] == b.t {
                        nbt.n[j] = id;
                        break;
                    }
                }
            }
            for k in 0..3 {
                let (a, c) = (tv[(k + 1) % 3], tv[(k + 2) % 3]);
                let key = (a.min(c), a.max(c));
                if let Some((o, oi)) = glue.remove(&key) {
                    self.tets[id as usize].n[k] = o;
                    self.tets[o as usize].n[oi] = id;
                } else {
                    glue.insert(key, (id, k));
                }
            }
        }
        for &id in &new_ids {
            let tv = self.tets[id as usize].v;
            for k in 0..4 {
                self.vtet[tv[k] as usize] = id;
            }
        }
        for &t in &cav {
            self.release(t);
        }
        if let Some(&last) = new_ids.last() {
            self.hint = last;
        }
        Ok(new_ids)
    }

    /// Replace the connected set of tets `old` by `new` (positively oriented tets that fill exactly the
    /// same region, so the faces on the border of the region are unchanged): the new tets are glued to
    /// each other and to the neighbours of the old ones. Returns the new tet ids.
    pub fn retet(&mut self, old: &[u32], new: &[[u32; 4]]) -> Vec<u32> {
        self.stamp += 1;
        let st = self.stamp;
        for &t in old {
            self.tets[t as usize].mark = st;
        }
        // faces on the border of the region and what lies beyond them
        let mut border: HashMap<[u32; 3], u32> = HashMap::new();
        for &t in old {
            for i in 0..4 {
                let nb = self.tets[t as usize].n[i];
                if nb == NONE || self.tets[nb as usize].mark != st {
                    let mut k = self.face_ids(t, i);
                    k.sort_unstable();
                    border.insert(k, nb);
                }
            }
        }
        let mut ids = Vec::with_capacity(new.len());
        let mut inner: HashMap<[u32; 3], (u32, usize)> = HashMap::new();
        for tv in new {
            let id = self.alloc(Tet {
                v: *tv,
                n: [NONE; 4],
                alive: true,
                mark: 0,
                rej: 0,
            });
            ids.push(id);
            for i in 0..4 {
                let mut k = self.face_ids(id, i);
                k.sort_unstable();
                if let Some(&nb) = border.get(&k) {
                    self.tets[id as usize].n[i] = nb;
                    if nb != NONE {
                        for j in 0..4 {
                            let mut kj = self.face_ids(nb, j);
                            kj.sort_unstable();
                            if kj == k {
                                self.tets[nb as usize].n[j] = id;
                            }
                        }
                    }
                } else if let Some((o, oi)) = inner.remove(&k) {
                    self.tets[id as usize].n[i] = o;
                    self.tets[o as usize].n[oi] = id;
                } else {
                    inner.insert(k, (id, i));
                }
            }
        }
        for &id in &ids {
            let tv = self.tets[id as usize].v;
            for k in 0..4 {
                self.vtet[tv[k] as usize] = id;
            }
        }
        for &t in old {
            self.release(t);
        }
        if let Some(&last) = ids.last() {
            self.hint = last;
        }
        ids
    }

    /// Check adjacency symmetry and that all tets are positively oriented (debugging aid).
    pub fn validate(&self) -> Result<(), String> {
        for (id, t) in self.tets.iter().enumerate() {
            if !t.alive {
                continue;
            }
            let id = id as u32;
            let v = |k: usize| &self.pts[t.v[k] as usize];
            if orient(v(0), v(1), v(2), v(3)) <= 0.0 {
                return Err(format!("tet {id} is not positively oriented"));
            }
            for i in 0..4 {
                let nb = t.n[i];
                if nb == NONE {
                    continue;
                }
                let nt = &self.tets[nb as usize];
                if !nt.alive {
                    return Err(format!("tet {id} neighbours dead tet {nb}"));
                }
                let Some(j) = nt.n.iter().position(|&x| x == id) else {
                    return Err(format!("tet {nb} does not point back to {id}"));
                };
                let mut a = self.face_ids(id, i);
                let mut b = self.face_ids(nb, j);
                a.sort_unstable();
                b.sort_unstable();
                if a != b {
                    return Err(format!("tets {id} and {nb} do not share a face"));
                }
            }
        }
        Ok(())
    }
}

fn classify(t: u32, s: &[f64; 4]) -> Loc {
    let zeros: Vec<usize> = (0..4).filter(|&i| s[i] == 0.0).collect();
    match zeros.len() {
        0 => Loc::Inside(t),
        1 => Loc::OnFace(t, zeros[0]),
        2 => {
            let rest: Vec<usize> = (0..4).filter(|i| !zeros.contains(i)).collect();
            Loc::OnEdge(t, [rest[0], rest[1]])
        }
        _ => {
            let v = (0..4).find(|i| !zeros.contains(i)).unwrap_or(0);
            Loc::OnVertex(t, v)
        }
    }
}

/// Delaunay triangulation of `pts` (plus four far-away super vertices at the end). Points are inserted
/// along a Morton curve to keep the walk short. Duplicate points are skipped.
pub fn delaunay(pts: &[P3]) -> Tri3 {
    let mut t = Tri3::with_super(pts);
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in pts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let ext = (0..3)
        .map(|k| hi[k] - lo[k])
        .fold(0.0, f64::max)
        .max(1e-300);
    let code = |p: &P3| -> u64 {
        let mut q = [0u64; 3];
        for k in 0..3 {
            q[k] = (((p[k] - lo[k]) / ext) * 2_097_151.0).clamp(0.0, 2_097_151.0) as u64;
        }
        let spread = |mut x: u64| {
            x &= 0x1f_ffff;
            x = (x | (x << 32)) & 0x1f00000000ffff;
            x = (x | (x << 16)) & 0x1f0000ff0000ff;
            x = (x | (x << 8)) & 0x100f00f00f00f00f;
            x = (x | (x << 4)) & 0x10c30c30c30c30c3;
            x = (x | (x << 2)) & 0x1249249249249249;
            x
        };
        spread(q[0]) | (spread(q[1]) << 1) | (spread(q[2]) << 2)
    };
    let mut order: Vec<u32> = (0..pts.len() as u32).collect();
    order.sort_by_key(|&i| code(&pts[i as usize]));
    for i in order {
        let _ = t.insert_vertex(i);
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 11) as f64) / ((1u64 << 53) as f64)
    }

    #[test]
    fn predicate_conventions() {
        let a = [0.0, 0.0, 0.0];
        let b = [1.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0];
        let d = [0.0, 0.0, 1.0];
        assert!(orient(&a, &b, &c, &d) > 0.0);
        // centre of the circumsphere is inside, a far point is outside
        assert!(in_sphere(&a, &b, &c, &d, &[0.5, 0.5, 0.5]) > 0.0);
        assert!(in_sphere(&a, &b, &c, &d, &[5.0, 5.0, 5.0]) < 0.0);
        // cospherical: (1,1,1) lies on the sphere through the unit corners
        assert_eq!(in_sphere(&a, &b, &c, &d, &[1.0, 1.0, 0.0]), 0.0);
    }

    #[test]
    fn delaunay_has_the_empty_sphere_property() {
        let mut seed = 42;
        let pts: Vec<P3> = (0..400)
            .map(|_| [lcg(&mut seed), lcg(&mut seed), lcg(&mut seed)])
            .collect();
        let t = delaunay(&pts);
        t.validate().unwrap();
        let mut count = 0;
        for tet in t.tets.iter().filter(|t| t.alive) {
            // only tets of real points are checked for emptiness against real points
            if tet.v.iter().any(|&v| v as usize >= pts.len()) {
                continue;
            }
            count += 1;
            let a = &t.pts[tet.v[0] as usize];
            let b = &t.pts[tet.v[1] as usize];
            let c = &t.pts[tet.v[2] as usize];
            let d = &t.pts[tet.v[3] as usize];
            for (i, p) in pts.iter().enumerate() {
                if tet.v.contains(&(i as u32)) {
                    continue;
                }
                assert!(
                    in_sphere(a, b, c, d, p) <= 0.0,
                    "point {i} inside a circumsphere"
                );
            }
        }
        assert!(count > 1000, "{count}");
        // every real vertex is present in the triangulation
        let mut used = vec![false; pts.len()];
        for tet in t.tets.iter().filter(|t| t.alive) {
            for &v in &tet.v {
                if (v as usize) < pts.len() {
                    used[v as usize] = true;
                }
            }
        }
        assert!(used.iter().all(|&u| u));
    }

    use proptest::prelude::*;

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        /// Empty-sphere property on point sets that are deliberately degenerate (quantised coordinates make
        /// many coplanar and cospherical quadruples) and contain duplicates.
        #[test]
        fn delaunay_of_degenerate_point_sets(raw in proptest::collection::vec((0u8..6, 0u8..6, 0u8..6), 6..70)) {
            let pts: Vec<P3> = raw.iter().map(|&(x, y, z)| [x as f64, y as f64, z as f64]).collect();
            let t = delaunay(&pts);
            prop_assert!(t.validate().is_ok());
            for tet in t.tets.iter().filter(|t| t.alive) {
                if tet.v.iter().any(|&v| v as usize >= pts.len()) {
                    continue;
                }
                let c = |k: usize| &t.pts[tet.v[k] as usize];
                for (i, p) in pts.iter().enumerate() {
                    if tet.v.iter().any(|&v| t.pts[v as usize] == *p) {
                        continue;
                    }
                    prop_assert!(in_sphere(c(0), c(1), c(2), c(3), p) <= 0.0, "point {i} inside a circumsphere");
                }
            }
        }
    }

    #[test]
    fn grid_points_with_many_cospherical_sets_still_triangulate() {
        let mut pts = Vec::new();
        for i in 0..6 {
            for j in 0..6 {
                for k in 0..6 {
                    pts.push([i as f64, j as f64, k as f64]);
                }
            }
        }
        let t = delaunay(&pts);
        t.validate().unwrap();
        let vol: f64 = t
            .tets
            .iter()
            .filter(|t| t.alive && t.v.iter().all(|&v| (v as usize) < pts.len()))
            .map(|tet| {
                let v = |k: usize| t.pts[tet.v[k] as usize];
                let (a, b, c, d) = (v(0), v(1), v(2), v(3));
                crate::dot(
                    crate::cross(crate::sub(b, a), crate::sub(c, a)),
                    crate::sub(d, a),
                ) / 6.0
            })
            .sum();
        assert!((vol - 125.0).abs() < 1e-9, "{vol}");
    }
}
