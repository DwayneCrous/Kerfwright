//! Isotropic surface remeshing (Botsch & Kobbelt, "A remeshing approach to multiresolution modeling"):
//! edges are split, collapsed and flipped towards a target length and tangential smoothing relaxes the
//! vertices, which are projected back onto the original surface. Face borders and sharp edges are kept.
//!
//! The tessellation of a B-rep is made for display: pole fans, long thin strips and triangles with tiny
//! angles are normal in it, and no amount of refinement removes a small angle. A volume mesh whose
//! boundary faces have a radius–edge ratio above 2 cannot have good tets either, so the volume mesher
//! starts from a remeshed surface.

use crate::surf::SurfaceMesh;
use crate::{cross, dot, norm, sub, P3};
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

pub type SizeFn = Arc<dyn Fn(P3) -> f64 + Send + Sync>;
pub type ProjectFn = Arc<dyn Fn(u32, P3) -> P3 + Send + Sync>;
/// Projection of a new vertex on the sharp edge between two faces (tags `a`, `b`) onto the true curve.
pub type ProjectEdgeFn = Arc<dyn Fn(u32, u32, P3) -> P3 + Send + Sync>;

#[derive(Clone)]
pub struct RemeshParams {
    /// Target edge length.
    pub target: f64,
    /// Spatially varying target (the smaller of this and `target` applies).
    pub size: Option<SizeFn>,
    /// Edges between triangles of one tag whose normals differ by more than this (degrees) are kept as
    /// sharp edges; all edges that separate two tags are kept.
    pub feature_angle_deg: f64,
    pub iterations: usize,
    /// Projection onto the true surface of a vertex with the given tag; without it vertices are
    /// projected onto the input triangles.
    pub project: Option<ProjectFn>,
    /// Projection of vertices created on sharp edges (otherwise they stay on the chord).
    pub project_edge: Option<ProjectEdgeFn>,
}

impl RemeshParams {
    pub fn new(target: f64) -> RemeshParams {
        RemeshParams {
            target,
            size: None,
            feature_angle_deg: 60.0,
            iterations: 8,
            project: None,
            project_edge: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct OrigTri {
    id: u32,
    a: P3,
    b: P3,
    c: P3,
    tag: u32,
}
impl PartialEq for OrigTri {
    fn eq(&self, o: &Self) -> bool {
        self.id == o.id
    }
}
impl RTreeObject for OrigTri {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        let mut lo = self.a;
        let mut hi = self.a;
        for p in [self.b, self.c] {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        AABB::from_corners(lo, hi)
    }
}
impl PointDistance for OrigTri {
    fn distance_2(&self, p: &[f64; 3]) -> f64 {
        let q = closest_on_triangle(*p, self.a, self.b, self.c);
        let d = sub(*p, q);
        dot(d, d)
    }
}

/// Closest point of triangle `abc` to `p` (Ericson, *Real-Time Collision Detection* §5.1.5).
pub fn closest_on_triangle(p: P3, a: P3, b: P3, c: P3) -> P3 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]];
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]];
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let bc = sub(c, b);
        return [b[0] + w * bc[0], b[1] + w * bc[1], b[2] + w * bc[2]];
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    [
        a[0] + ab[0] * v + ac[0] * w,
        a[1] + ab[1] * v + ac[1] * w,
        a[2] + ab[2] * v + ac[2] * w,
    ]
}

fn key2(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

fn tri_normal(a: P3, b: P3, c: P3) -> P3 {
    cross(sub(b, a), sub(c, a))
}

struct RMesh {
    nodes: Vec<P3>,
    vt: Vec<Vec<u32>>,
    vdead: Vec<bool>,
    /// 0: smooth, 1: on exactly two sharp edges (slides along the curve), 2: corner (fixed)
    vkind: Vec<u8>,
    tris: Vec<[u32; 3]>,
    tag: Vec<u32>,
    alive: Vec<bool>,
    he: HashMap<(u32, u32), u32>,
    feat: HashSet<(u32, u32)>,
}

impl RMesh {
    fn new(s: &SurfaceMesh, feature_angle_deg: f64) -> RMesh {
        let n = s.nodes.len();
        let mut m = RMesh {
            nodes: s.nodes.clone(),
            vt: vec![Vec::new(); n],
            vdead: vec![false; n],
            vkind: vec![0; n],
            tris: Vec::new(),
            tag: Vec::new(),
            alive: Vec::new(),
            he: HashMap::new(),
            feat: HashSet::new(),
        };
        for (t, &tag) in s.tris.iter().zip(&s.tags) {
            m.add_tri(*t, tag);
        }
        let cosf = feature_angle_deg.to_radians().cos();
        let edges: Vec<(u32, u32)> = m.he.keys().filter(|(a, b)| a < b).copied().collect();
        for (a, b) in edges {
            let (t1, t2) = (m.he[&(a, b)], m.he[&(b, a)]);
            let sharp = m.tag[t1 as usize] != m.tag[t2 as usize] || {
                let n1 = m.normal(t1);
                let n2 = m.normal(t2);
                let (l1, l2) = (norm(n1), norm(n2));
                l1 > 0.0 && l2 > 0.0 && dot(n1, n2) / (l1 * l2) < cosf
            };
            if sharp {
                m.feat.insert((a, b));
            }
        }
        m.classify_vertices();
        m
    }

    fn classify_vertices(&mut self) {
        let mut count = vec![0u32; self.nodes.len()];
        let mut dirs: Vec<Vec<P3>> = vec![Vec::new(); self.nodes.len()];
        for &(a, b) in &self.feat {
            count[a as usize] += 1;
            count[b as usize] += 1;
            let d = sub(self.nodes[b as usize], self.nodes[a as usize]);
            dirs[a as usize].push(d);
            dirs[b as usize].push([-d[0], -d[1], -d[2]]);
        }
        for v in 0..self.nodes.len() {
            self.vkind[v] = match count[v] {
                0 => 0,
                2 => {
                    // a bend sharper than ~60° makes a corner
                    let (d0, d1) = (dirs[v][0], dirs[v][1]);
                    let c = -dot(d0, d1) / (norm(d0) * norm(d1)).max(1e-300);
                    if c < 0.5 {
                        2
                    } else {
                        1
                    }
                }
                _ => 2,
            };
        }
    }

    fn add_tri(&mut self, t: [u32; 3], tag: u32) -> u32 {
        let id = self.tris.len() as u32;
        self.tris.push(t);
        self.tag.push(tag);
        self.alive.push(true);
        for i in 0..3 {
            self.he.insert((t[i], t[(i + 1) % 3]), id);
            self.vt[t[i] as usize].push(id);
        }
        id
    }

    fn kill_tri(&mut self, id: u32) {
        let t = self.tris[id as usize];
        for i in 0..3 {
            self.he.remove(&(t[i], t[(i + 1) % 3]));
            let v = &mut self.vt[t[i] as usize];
            if let Some(p) = v.iter().position(|&x| x == id) {
                v.swap_remove(p);
            }
        }
        self.alive[id as usize] = false;
    }

    fn pos(&self, v: u32) -> P3 {
        self.nodes[v as usize]
    }

    fn normal(&self, t: u32) -> P3 {
        let tr = self.tris[t as usize];
        tri_normal(self.pos(tr[0]), self.pos(tr[1]), self.pos(tr[2]))
    }

    fn is_feat(&self, a: u32, b: u32) -> bool {
        self.feat.contains(&key2(a, b))
    }

    fn apex(&self, t: u32, a: u32, b: u32) -> u32 {
        let tr = self.tris[t as usize];
        for i in 0..3 {
            if tr[i] == a && tr[(i + 1) % 3] == b {
                return tr[(i + 2) % 3];
            }
        }
        u32::MAX
    }

    fn valence(&self, v: u32) -> usize {
        self.vt[v as usize].len()
    }

    /// Vertices adjacent to `v`.
    fn neighbours(&self, v: u32) -> Vec<u32> {
        let mut out = Vec::with_capacity(8);
        for &t in &self.vt[v as usize] {
            for &w in &self.tris[t as usize] {
                if w != v && !out.contains(&w) {
                    out.push(w);
                }
            }
        }
        out
    }

    fn all_edges(&self) -> Vec<(u32, u32)> {
        let mut e: Vec<(u32, u32)> = self.he.keys().filter(|(a, b)| a < b).copied().collect();
        e.sort_unstable();
        e
    }
}

struct Ctx<'a> {
    p: &'a RemeshParams,
    orig: RTree<OrigTri>,
}

impl Ctx<'_> {
    fn target_at(&self, p: &P3) -> f64 {
        let mut h = self.p.target;
        if let Some(f) = &self.p.size {
            h = h.min(f(*p));
        }
        h
    }

    fn project(&self, tag: u32, p: P3) -> P3 {
        if let Some(pr) = &self.p.project {
            let q = pr(tag, p);
            if q.iter().all(|x| x.is_finite()) {
                return q;
            }
        }
        let mut best: Option<(f64, P3)> = None;
        for (k, t) in self.orig.nearest_neighbor_iter(&p).enumerate() {
            if t.tag == tag {
                let q = closest_on_triangle(p, t.a, t.b, t.c);
                best = Some((norm(sub(p, q)), q));
                break;
            }
            if k > 64 {
                break;
            }
        }
        best.map_or(p, |(_, q)| q)
    }
}

/// Remesh `input` towards the target edge length. Returns a closed, outward-oriented triangle mesh.
pub fn remesh(input: &SurfaceMesh, params: &RemeshParams) -> SurfaceMesh {
    let orig: Vec<OrigTri> = input
        .tris
        .iter()
        .enumerate()
        .map(|(i, t)| OrigTri {
            id: i as u32,
            a: input.nodes[t[0] as usize],
            b: input.nodes[t[1] as usize],
            c: input.nodes[t[2] as usize],
            tag: input.tags[i],
        })
        .collect();
    let ctx = Ctx {
        p: params,
        orig: RTree::bulk_load(orig),
    };
    let mut m = RMesh::new(input, params.feature_angle_deg);
    for _ in 0..params.iterations {
        split_long(&mut m, &ctx);
        collapse_short(&mut m, &ctx);
        flip_valence(&mut m);
        smooth(&mut m, &ctx);
    }
    // a last pass of splits and collapses fixes what the smoothing moved
    split_long(&mut m, &ctx);
    collapse_short(&mut m, &ctx);
    smooth(&mut m, &ctx);
    // compact
    let mut remap = vec![u32::MAX; m.nodes.len()];
    let mut out = SurfaceMesh::default();
    for t in 0..m.tris.len() {
        if !m.alive[t] {
            continue;
        }
        let mut nt = [0u32; 3];
        for i in 0..3 {
            let v = m.tris[t][i] as usize;
            if remap[v] == u32::MAX {
                remap[v] = out.nodes.len() as u32;
                out.nodes.push(m.nodes[v]);
            }
            nt[i] = remap[v];
        }
        out.tris.push(nt);
        out.tags.push(m.tag[t]);
    }
    out
}

fn edge_len(m: &RMesh, a: u32, b: u32) -> f64 {
    norm(sub(m.pos(a), m.pos(b)))
}

fn mid(m: &RMesh, a: u32, b: u32) -> P3 {
    let (p, q) = (m.pos(a), m.pos(b));
    [
        0.5 * (p[0] + q[0]),
        0.5 * (p[1] + q[1]),
        0.5 * (p[2] + q[2]),
    ]
}

fn split_long(m: &mut RMesh, ctx: &Ctx) {
    for _pass in 0..6 {
        let mut long: Vec<(f64, u32, u32)> = Vec::new();
        for (a, b) in m.all_edges() {
            let l = edge_len(m, a, b);
            let h = ctx.target_at(&mid(m, a, b));
            if l > 4.0 / 3.0 * h {
                long.push((l, a, b));
            }
        }
        if long.is_empty() {
            return;
        }
        long.sort_by(|x, y| y.0.total_cmp(&x.0));
        for (_, a, b) in long {
            let (Some(&t1), Some(&t2)) = (m.he.get(&(a, b)), m.he.get(&(b, a))) else {
                continue;
            };
            let feature = m.is_feat(a, b);
            let c = m.apex(t1, a, b);
            let d = m.apex(t2, b, a);
            let tag1 = m.tag[t1 as usize];
            let tag2 = m.tag[t2 as usize];
            let mut p = mid(m, a, b);
            if feature {
                if let Some(pe) = &ctx.p.project_edge {
                    let q = pe(tag1, tag2, p);
                    if q.iter().all(|x| x.is_finite()) {
                        p = q;
                    }
                }
            } else if tag1 == tag2 {
                p = ctx.project(tag1, p);
            }
            let id = m.nodes.len() as u32;
            m.nodes.push(p);
            m.vt.push(Vec::new());
            m.vdead.push(false);
            m.vkind.push(if feature { 1 } else { 0 });
            m.kill_tri(t1);
            m.kill_tri(t2);
            m.add_tri([a, id, c], tag1);
            m.add_tri([id, b, c], tag1);
            m.add_tri([b, id, d], tag2);
            m.add_tri([id, a, d], tag2);
            if feature {
                m.feat.remove(&key2(a, b));
                m.feat.insert(key2(a, id));
                m.feat.insert(key2(id, b));
            }
        }
    }
}

/// Can edge `u→v` be collapsed (removing `u`)?
fn collapse_ok(m: &RMesh, ctx: &Ctx, u: u32, v: u32) -> bool {
    if m.vkind[u as usize] == 2 {
        return false;
    }
    let feature = m.is_feat(u, v);
    if m.vkind[u as usize] == 1 && !feature {
        return false;
    }
    if feature && m.vkind[u as usize] == 1 {
        // along the curve: the other sharp edge of u must not turn sharply away from v
    } else if m.vkind[u as usize] == 0 && m.vkind[v as usize] != 0 && !feature {
        // smooth vertex collapsing onto a feature vertex is fine
    }
    let (Some(&t1), Some(&t2)) = (m.he.get(&(u, v)), m.he.get(&(v, u))) else {
        return false;
    };
    let c = m.apex(t1, u, v);
    let d = m.apex(t2, v, u);
    // link condition: the common neighbours of u and v are exactly c and d
    let nu = m.neighbours(u);
    let nv = m.neighbours(v);
    for w in &nu {
        if nv.contains(w) && *w != c && *w != d {
            return false;
        }
    }
    // a vertex of valence 3 on c or d would be left with two triangles
    if m.valence(c) <= 3 || m.valence(d) <= 3 {
        return false;
    }
    // geometry: no flipped triangles, no long edges
    let pv = m.pos(v);
    let hmax = 4.0 / 3.0 * ctx.target_at(&pv);
    for &t in &m.vt[u as usize] {
        if t == t1 || t == t2 {
            continue;
        }
        let tr = m.tris[t as usize];
        let old = tri_normal(m.pos(tr[0]), m.pos(tr[1]), m.pos(tr[2]));
        let mut q = [m.pos(tr[0]), m.pos(tr[1]), m.pos(tr[2])];
        for i in 0..3 {
            if tr[i] == u {
                q[i] = pv;
            }
        }
        let new = tri_normal(q[0], q[1], q[2]);
        let (lo, ln) = (norm(old), norm(new));
        if ln <= 1e-12 * hmax * hmax || dot(old, new) <= 0.3 * lo * ln {
            return false;
        }
        for &w in &tr {
            if w != u && w != v && norm(sub(m.pos(w), pv)) > hmax {
                return false;
            }
        }
    }
    true
}

fn collapse(m: &mut RMesh, u: u32, v: u32) {
    let t1 = m.he[&(u, v)];
    let t2 = m.he[&(v, u)];
    let ring: Vec<u32> = m.vt[u as usize].clone();
    m.kill_tri(t1);
    m.kill_tri(t2);
    for t in ring {
        if t == t1 || t == t2 || !m.alive[t as usize] {
            continue;
        }
        let tr = m.tris[t as usize];
        let tag = m.tag[t as usize];
        m.kill_tri(t);
        let mut nt = tr;
        for x in &mut nt {
            if *x == u {
                *x = v;
            }
        }
        m.add_tri(nt, tag);
    }
    // sharp edges at u move to v
    let moved: Vec<(u32, u32)> = m
        .feat
        .iter()
        .copied()
        .filter(|&(a, b)| a == u || b == u)
        .collect();
    for (a, b) in moved {
        m.feat.remove(&(a, b));
        let w = if a == u { b } else { a };
        if w != v {
            m.feat.insert(key2(v, w));
        }
    }
    m.vdead[u as usize] = true;
}

fn collapse_short(m: &mut RMesh, ctx: &Ctx) {
    let mut short: Vec<(f64, u32, u32)> = Vec::new();
    for (a, b) in m.all_edges() {
        let l = edge_len(m, a, b);
        let h = ctx.target_at(&mid(m, a, b));
        if l < 0.8 * h {
            short.push((l, a, b));
        }
    }
    short.sort_by(|x, y| x.0.total_cmp(&y.0));
    for (_, a, b) in short {
        if m.vdead[a as usize] || m.vdead[b as usize] {
            continue;
        }
        if !m.he.contains_key(&(a, b)) || !m.he.contains_key(&(b, a)) {
            continue;
        }
        // try removing either end; prefer keeping the more constrained vertex
        let order = if m.vkind[a as usize] > m.vkind[b as usize] {
            [(b, a), (a, b)]
        } else {
            [(a, b), (b, a)]
        };
        for (u, v) in order {
            if collapse_ok(m, ctx, u, v) {
                collapse(m, u, v);
                break;
            }
        }
    }
}

fn flip_valence(m: &mut RMesh) {
    for (a, b) in m.all_edges() {
        let (Some(&t1), Some(&t2)) = (m.he.get(&(a, b)), m.he.get(&(b, a))) else {
            continue;
        };
        if m.is_feat(a, b) || m.tag[t1 as usize] != m.tag[t2 as usize] {
            continue;
        }
        let c = m.apex(t1, a, b);
        let d = m.apex(t2, b, a);
        if c == d || m.he.contains_key(&(c, d)) || m.he.contains_key(&(d, c)) {
            continue;
        }
        let target = |v: u32| if m.vkind[v as usize] == 0 { 6i32 } else { 4 };
        let val = |v: u32| m.valence(v) as i32;
        let dev = |va: i32, vb: i32, vc: i32, vd: i32| {
            (va - target(a)).pow(2)
                + (vb - target(b)).pow(2)
                + (vc - target(c)).pow(2)
                + (vd - target(d)).pow(2)
        };
        let before = dev(val(a), val(b), val(c), val(d));
        let after = dev(val(a) - 1, val(b) - 1, val(c) + 1, val(d) + 1);
        if after >= before || val(a) <= 3 || val(b) <= 3 {
            continue;
        }
        // geometry: both new triangles keep the orientation of the old ones and are not slivers
        let (pa, pb, pc, pd) = (m.pos(a), m.pos(b), m.pos(c), m.pos(d));
        let n_old = tri_normal(pa, pb, pc);
        let n_old2 = tri_normal(pb, pa, pd);
        let n1 = tri_normal(pa, pd, pc);
        let n2 = tri_normal(pd, pb, pc);
        let ok = |n: P3, o: P3| dot(n, o) > 0.5 * norm(n) * norm(o) && norm(n) > 0.0;
        if !(ok(n1, n_old) && ok(n2, n_old) && ok(n1, n_old2) && ok(n2, n_old2)) {
            continue;
        }
        // do not make the diagonal much longer than the edge it replaces
        if norm(sub(pc, pd)) > 1.5 * norm(sub(pa, pb)) {
            continue;
        }
        let tag = m.tag[t1 as usize];
        m.kill_tri(t1);
        m.kill_tri(t2);
        m.add_tri([a, d, c], tag);
        m.add_tri([d, b, c], tag);
    }
}

fn smooth(m: &mut RMesh, ctx: &Ctx) {
    let n = m.nodes.len();
    let mut moves: Vec<(u32, P3)> = Vec::new();
    for v in 0..n as u32 {
        if m.vdead[v as usize] || m.vt[v as usize].is_empty() || m.vkind[v as usize] != 0 {
            continue;
        }
        let nb = m.neighbours(v);
        if nb.is_empty() {
            continue;
        }
        let p = m.pos(v);
        let mut c = [0.0; 3];
        for &w in &nb {
            let q = m.pos(w);
            for k in 0..3 {
                c[k] += q[k];
            }
        }
        for k in 0..3 {
            c[k] /= nb.len() as f64;
        }
        // area-weighted vertex normal
        let mut nrm = [0.0; 3];
        for &t in &m.vt[v as usize] {
            let tn = m.normal(t);
            for k in 0..3 {
                nrm[k] += tn[k];
            }
        }
        let nl = norm(nrm);
        if nl <= 0.0 {
            continue;
        }
        let nrm = [nrm[0] / nl, nrm[1] / nl, nrm[2] / nl];
        let mut d = sub(c, p);
        let dn = dot(d, nrm);
        d = [d[0] - dn * nrm[0], d[1] - dn * nrm[1], d[2] - dn * nrm[2]];
        let tag = m.tag[m.vt[v as usize][0] as usize];
        let q = ctx.project(tag, [p[0] + d[0], p[1] + d[1], p[2] + d[2]]);
        moves.push((v, q));
    }
    for (v, q) in moves {
        // keep the move only if no incident triangle flips or collapses
        let old = m.pos(v);
        m.nodes[v as usize] = q;
        let mut good = true;
        for &t in &m.vt[v as usize] {
            let tr = m.tris[t as usize];
            let new = tri_normal(m.pos(tr[0]), m.pos(tr[1]), m.pos(tr[2]));
            let mut was = [m.pos(tr[0]), m.pos(tr[1]), m.pos(tr[2])];
            for i in 0..3 {
                if tr[i] == v {
                    was[i] = old;
                }
            }
            let prev = tri_normal(was[0], was[1], was[2]);
            if norm(new) <= 0.0 || dot(new, prev) <= 0.2 * norm(new) * norm(prev) {
                good = false;
                break;
            }
        }
        if !good {
            m.nodes[v as usize] = old;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn min_angle_deg(s: &SurfaceMesh) -> f64 {
        let mut worst = 180.0f64;
        for t in &s.tris {
            let p = [
                s.nodes[t[0] as usize],
                s.nodes[t[1] as usize],
                s.nodes[t[2] as usize],
            ];
            for i in 0..3 {
                let a = sub(p[(i + 1) % 3], p[i]);
                let b = sub(p[(i + 2) % 3], p[i]);
                let c = (dot(a, b) / (norm(a) * norm(b))).clamp(-1.0, 1.0);
                worst = worst.min(c.acos().to_degrees());
            }
        }
        worst
    }

    #[test]
    fn remeshing_a_sphere_gives_uniform_well_shaped_triangles() {
        let s = SurfaceMesh::sphere([0.0; 3], 1.0, 2);
        let out = remesh(&s, &RemeshParams::new(0.2));
        out.check_closed().unwrap();
        // the remeshed surface stays on the input polyhedron
        assert!(
            (out.volume() / s.volume() - 1.0).abs() < 0.02,
            "{} vs {}",
            out.volume(),
            s.volume()
        );
        assert!(min_angle_deg(&out) > 25.0, "{}", min_angle_deg(&out));
        let mut lens: Vec<f64> = Vec::new();
        for t in &out.tris {
            for i in 0..3 {
                lens.push(norm(sub(
                    out.nodes[t[i] as usize],
                    out.nodes[t[(i + 1) % 3] as usize],
                )));
            }
        }
        let mean = lens.iter().sum::<f64>() / lens.len() as f64;
        assert!(mean > 0.15 && mean < 0.27, "{mean}");
    }

    #[test]
    fn sharp_edges_of_a_box_survive() {
        let s = SurfaceMesh::cuboid(2.0, 1.0, 1.0);
        let out = remesh(&s, &RemeshParams::new(0.3));
        out.check_closed().unwrap();
        assert!((out.volume() - 2.0).abs() < 1e-9, "{}", out.volume());
        // every vertex lies on the surface of the box
        for p in &out.nodes {
            let on = |x: f64, lo: f64, hi: f64| (x - lo).abs() < 1e-9 || (x - hi).abs() < 1e-9;
            assert!(
                on(p[0], 0.0, 2.0) || on(p[1], 0.0, 1.0) || on(p[2], 0.0, 1.0),
                "{p:?}"
            );
        }
        // the 8 corners are still vertices
        for c in 0..8 {
            let corner = [
                if c & 1 == 1 { 2.0 } else { 0.0 },
                if c & 2 == 2 { 1.0 } else { 0.0 },
                if c & 4 == 4 { 1.0 } else { 0.0 },
            ];
            assert!(
                out.nodes.iter().any(|p| norm(sub(*p, corner)) < 1e-12),
                "{corner:?}"
            );
        }
        assert!(min_angle_deg(&out) > 20.0, "{}", min_angle_deg(&out));
    }
}
