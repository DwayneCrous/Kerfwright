//! Deliberate corruption of a [`BRep`], used by the fuzz tests to prove that
//! `validate()` never lets a semantically changed body through.

use crate::ids::*;
use crate::store::BRep;
use crate::topology::*;
use kw_math::Vector3;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

/// One corruption. Index fields are reduced modulo the relevant arena size.
#[derive(Debug, Clone, Copy)]
pub enum Mutation {
    FlipCoedgeSense(usize),
    SetCoedgeNext(usize, usize),
    SetCoedgePrev(usize, usize),
    SetCoedgeRadialNext(usize, usize),
    SetCoedgeEdge(usize, usize),
    SetCoedgeLoop(usize, usize),
    SwapPcurves(usize, usize),
    RemoveCoedge(usize),
    RetargetEdgeStart(usize, usize),
    RetargetEdgeEnd(usize, usize),
    SetEdgeCurve(usize, usize),
    ScaleEdgeRange(usize),
    SetEdgeFirstCoedge(usize, usize),
    RemoveEdge(usize),
    RemoveVertex(usize),
    MoveVertex(usize, [i8; 3]),
    FlipFaceSense(usize),
    SetFaceSurface(usize, usize),
    RemoveFace(usize),
    DropFaceFromShell(usize),
    DuplicateFaceInShell(usize),
    FlipLoopKind(usize),
    AddBogusLoop(usize, usize),
}

fn nth<K: slotmap::Key, V>(m: &slotmap::SlotMap<K, V>, i: usize) -> Option<K> {
    if m.is_empty() {
        None
    } else {
        m.keys().nth(i % m.len())
    }
}

fn hash_of<T: std::fmt::Debug>(x: &T) -> u64 {
    let mut h = DefaultHasher::new();
    format!("{x:?}").hash(&mut h);
    h.finish()
}

/// Canonical fingerprint of everything semantically meaningful in the body.
/// Geometry is compared by content, references by key. `first_coedge`
/// pointers and radial-ring *order* are deliberately excluded (rotations are
/// not semantic changes).
pub fn signature(b: &BRep) -> u64 {
    let mut h = DefaultHasher::new();
    for (k, v) in &b.vertices {
        (
            k,
            v.point.x.to_bits(),
            v.point.y.to_bits(),
            v.point.z.to_bits(),
            v.tolerance.to_bits(),
        )
            .hash(&mut h);
    }
    for (k, e) in &b.edges {
        let curve = e.curve.and_then(|c| b.curves.get(c)).map(hash_of);
        let mut ring: Vec<_> = b.radial_ring(k);
        ring.sort();
        (
            k,
            e.start,
            e.end,
            curve,
            e.curve.is_some(),
            e.param_range.0.to_bits(),
            e.param_range.1.to_bits(),
            e.tolerance.to_bits(),
            ring,
        )
            .hash(&mut h);
    }
    for (k, c) in &b.coedges {
        let pc = c.pcurve.and_then(|p| b.curves.get(p)).map(hash_of);
        (
            k,
            c.edge,
            c.owner_loop,
            c.next,
            c.prev,
            c.sense == Sense::Forward,
            pc,
        )
            .hash(&mut h);
    }
    for (k, l) in &b.loops {
        let mut ring = b.loop_coedges(k);
        // rotate so the smallest key leads: `first_coedge` is not semantic
        if let Some(pos) = ring
            .iter()
            .enumerate()
            .min_by_key(|(_, c)| **c)
            .map(|(i, _)| i)
        {
            ring.rotate_left(pos);
        }
        (k, l.face, l.kind == LoopKind::Outer, ring).hash(&mut h);
    }
    for (k, f) in &b.faces {
        let surf = b.surfaces.get(f.surface).map(hash_of);
        let mut loops = f.loops.clone();
        loops.sort();
        (k, f.shell, f.sense == Sense::Forward, surf, loops).hash(&mut h);
    }
    for (k, s) in &b.shells {
        let mut faces = s.faces.clone();
        faces.sort();
        (k, s.lump, faces).hash(&mut h);
    }
    for (k, l) in &b.lumps {
        (k, l.body, l.shells.clone()).hash(&mut h);
    }
    for (k, bd) in &b.bodies {
        (k, bd.lumps.clone(), bd.kind == BodyKind::Solid).hash(&mut h);
    }
    h.finish()
}

/// Apply `m`. Returns `false` when the mutation is not applicable (nothing
/// was changed).
pub fn apply(b: &mut BRep, m: Mutation) -> bool {
    match m {
        Mutation::FlipCoedgeSense(i) => {
            let Some(k) = nth(&b.coedges, i) else {
                return false;
            };
            b.coedges[k].sense = b.coedges[k].sense.flip();
            true
        }
        Mutation::SetCoedgeNext(i, j) => set_coedge_link(b, i, j, |c, t| c.next = t, |c| c.next),
        Mutation::SetCoedgePrev(i, j) => set_coedge_link(b, i, j, |c, t| c.prev = t, |c| c.prev),
        Mutation::SetCoedgeRadialNext(i, j) => {
            set_coedge_link(b, i, j, |c, t| c.radial_next = t, |c| c.radial_next)
        }
        Mutation::SetCoedgeEdge(i, j) => {
            let (Some(k), Some(e)) = (nth(&b.coedges, i), nth(&b.edges, j)) else {
                return false;
            };
            if b.coedges[k].edge == e {
                return false;
            }
            b.coedges[k].edge = e;
            true
        }
        Mutation::SetCoedgeLoop(i, j) => {
            let (Some(k), Some(l)) = (nth(&b.coedges, i), nth(&b.loops, j)) else {
                return false;
            };
            if b.coedges[k].owner_loop == l {
                return false;
            }
            b.coedges[k].owner_loop = l;
            true
        }
        Mutation::SwapPcurves(i, j) => {
            let (Some(a), Some(c)) = (nth(&b.coedges, i), nth(&b.coedges, j)) else {
                return false;
            };
            let (pa, pc) = (b.coedges[a].pcurve, b.coedges[c].pcurve);
            let content = |p: Option<CurveId>| p.and_then(|p| b.curves.get(p)).map(hash_of);
            if content(pa) == content(pc) {
                return false;
            }
            b.coedges[a].pcurve = pc;
            b.coedges[c].pcurve = pa;
            true
        }
        Mutation::RemoveCoedge(i) => {
            let Some(k) = nth(&b.coedges, i) else {
                return false;
            };
            b.coedges.remove(k).is_some()
        }
        Mutation::RetargetEdgeStart(i, j) => retarget(b, i, j, true),
        Mutation::RetargetEdgeEnd(i, j) => retarget(b, i, j, false),
        Mutation::SetEdgeCurve(i, j) => {
            let (Some(e), Some(c)) = (nth(&b.edges, i), nth(&b.curves, j)) else {
                return false;
            };
            let content = |c: Option<CurveId>| c.and_then(|c| b.curves.get(c)).map(hash_of);
            if content(b.edges[e].curve) == content(Some(c)) {
                return false;
            }
            b.edges[e].curve = Some(c);
            true
        }
        Mutation::ScaleEdgeRange(i) => {
            let Some(e) = nth(&b.edges, i) else {
                return false;
            };
            let r = b.edges[e].param_range;
            b.edges[e].param_range = (r.0, r.0 + (r.1 - r.0) * 0.9);
            true
        }
        Mutation::SetEdgeFirstCoedge(i, j) => {
            let (Some(e), Some(c)) = (nth(&b.edges, i), nth(&b.coedges, j)) else {
                return false;
            };
            if b.edges[e].first_coedge == Some(c) || b.coedges[c].edge == e {
                return false; // rotating within the true ring is not semantic
            }
            b.edges[e].first_coedge = Some(c);
            true
        }
        Mutation::RemoveEdge(i) => {
            let Some(k) = nth(&b.edges, i) else {
                return false;
            };
            b.edges.remove(k).is_some()
        }
        Mutation::RemoveVertex(i) => {
            let Some(k) = nth(&b.vertices, i) else {
                return false;
            };
            b.vertices.remove(k).is_some()
        }
        Mutation::MoveVertex(i, d) => {
            let Some(k) = nth(&b.vertices, i) else {
                return false;
            };
            let v = Vector3::new(d[0] as f64, d[1] as f64, d[2] as f64);
            if v.norm() == 0.0 {
                return false;
            }
            b.vertices[k].point += v * 1e-3;
            true
        }
        Mutation::FlipFaceSense(i) => {
            let Some(k) = nth(&b.faces, i) else {
                return false;
            };
            b.faces[k].sense = b.faces[k].sense.flip();
            true
        }
        Mutation::SetFaceSurface(i, j) => {
            let (Some(f), Some(s)) = (nth(&b.faces, i), nth(&b.surfaces, j)) else {
                return false;
            };
            let content = |s: SurfaceId| b.surfaces.get(s).map(hash_of);
            if content(b.faces[f].surface) == content(s) {
                return false;
            }
            b.faces[f].surface = s;
            true
        }
        Mutation::RemoveFace(i) => {
            let Some(k) = nth(&b.faces, i) else {
                return false;
            };
            b.faces.remove(k).is_some()
        }
        Mutation::DropFaceFromShell(i) => {
            let Some(k) = nth(&b.faces, i) else {
                return false;
            };
            let sh = b.faces[k].shell;
            let before = b.shells[sh].faces.len();
            b.shells[sh].faces.retain(|f| *f != k);
            b.shells[sh].faces.len() != before
        }
        Mutation::DuplicateFaceInShell(i) => {
            let Some(k) = nth(&b.faces, i) else {
                return false;
            };
            let sh = b.faces[k].shell;
            b.shells[sh].faces.push(k);
            true
        }
        Mutation::FlipLoopKind(i) => {
            let Some(k) = nth(&b.loops, i) else {
                return false;
            };
            b.loops[k].kind = match b.loops[k].kind {
                LoopKind::Outer => LoopKind::Inner,
                LoopKind::Inner => LoopKind::Outer,
            };
            true
        }
        Mutation::AddBogusLoop(i, j) => {
            let (Some(f), Some(c)) = (nth(&b.faces, i), nth(&b.coedges, j)) else {
                return false;
            };
            let l = b.loops.insert(Loop {
                face: f,
                kind: LoopKind::Inner,
                first_coedge: c,
            });
            b.faces[f].loops.push(l);
            true
        }
    }
}

fn set_coedge_link(
    b: &mut BRep,
    i: usize,
    j: usize,
    set: impl Fn(&mut Coedge, CoedgeId),
    get: impl Fn(&Coedge) -> CoedgeId,
) -> bool {
    let (Some(k), Some(t)) = (nth(&b.coedges, i), nth(&b.coedges, j)) else {
        return false;
    };
    if get(&b.coedges[k]) == t {
        return false;
    }
    set(&mut b.coedges[k], t);
    true
}

fn retarget(b: &mut BRep, i: usize, j: usize, start: bool) -> bool {
    let (Some(e), Some(v)) = (nth(&b.edges, i), nth(&b.vertices, j)) else {
        return false;
    };
    let cur = if start {
        b.edges[e].start
    } else {
        b.edges[e].end
    };
    if cur == v {
        return false;
    }
    // A vertex at the same position would be a semantically neutral change.
    if let (Some(a), Some(c)) = (b.vertices.get(cur), b.vertices.get(v)) {
        if (a.point - c.point).norm() < 1e-9 {
            return false;
        }
    }
    if start {
        b.edges[e].start = v;
    } else {
        b.edges[e].end = v;
    }
    true
}
