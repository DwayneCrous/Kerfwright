//! Phase 5: build the result body from the selected fragments, check it, and copy it into the caller's B-rep.

use crate::arrangement::{FEdge, Fragment, He};
use crate::model::*;
use crate::select::Selected;
use crate::{BoolOptions, BooleanError, BooleanResult, Op};
use kw_brep::{
    BRep, BodyId, BodyKind, CoedgeSpec, CopyOptions, EdgeId, EntityRef, FaceId, LoopKind, Origin,
    Sense, ShapeHistory, ShellId, SolidIndex, VertexId,
};
use kw_nurbs::pcurve::pcurve_on;
use kw_nurbs::{Curve3, CurveGeom};
use std::collections::{BTreeMap, HashMap};

fn oriented(fr: &Fragment, sel: &Selected) -> (Vec<He>, Vec<Vec<He>>) {
    let flip = |c: &Vec<He>| -> Vec<He> {
        if sel.flip {
            c.iter()
                .rev()
                .map(|h| He {
                    fe: h.fe,
                    fwd: !h.fwd,
                })
                .collect()
        } else {
            c.clone()
        }
    };
    (flip(&fr.outer), fr.holes.iter().map(flip).collect())
}

/// A halfedge of an output face: the canonical piece, its direction, and where its pcurve data comes from
/// (selected fragment `si`, fragment edge `fe`).
#[derive(Clone, Copy)]
struct OutHe {
    piece: usize,
    fwd: bool,
    si: usize,
    fe: usize,
}

/// A face of the result: the selected fragment that lends it its surface, its loops and whether it was
/// formed by merging fragments.
struct OutFace {
    base: usize,
    outer: Vec<OutHe>,
    holes: Vec<Vec<OutHe>>,
    merged: bool,
}

fn out_cycle(w: &Work, selected: &[Selected], si: usize, cyc: &[He]) -> Vec<OutHe> {
    let fr = &w.fragments[selected[si].fragment];
    cyc.iter()
        .map(|h| OutHe {
            piece: fr.fedges[h.fe].piece,
            fwd: h.fwd,
            si,
            fe: h.fe,
        })
        .collect()
}

/// Are two selected fragments parts of one geometric face? The same input face (cut apart by an imprint
/// that did not separate the result) or two coplanar planar faces with the same outward normal.
fn same_domain(w: &Work, selected: &[Selected], a: usize, b: usize, tol: f64) -> bool {
    let (fa, fb) = (
        &w.faces[w.fragments[selected[a].fragment].face],
        &w.faces[w.fragments[selected[b].fragment].face],
    );
    if selected[a].flip != selected[b].flip {
        return false;
    }
    if std::ptr::eq(fa, fb) {
        return true;
    }
    use kw_nurbs::SurfaceGeom;
    let (SurfaceGeom::Plane(pa), SurfaceGeom::Plane(pb)) = (&fa.surface, &fb.surface) else {
        return false;
    };
    let sg = |f: &FaceRec| if f.sense == Sense::Forward { 1.0 } else { -1.0 };
    let (na, nb) = (pa.frame.z * sg(fa), pb.frame.z * sg(fb));
    na.dot(&nb) > 1.0 - 1e-12 && (pb.frame.origin - pa.frame.origin).dot(&na).abs() <= 10.0 * tol
}

/// Merge the selected fragments that are neighbours across a piece and lie in one geometric face into
/// single faces. A group whose remaining boundary does not form simple loops (or, across different input
/// faces, more than one loop) is left as it is.
fn merge_fragments(w: &Work, selected: &[Selected]) -> Vec<OutFace> {
    let n = selected.len();
    let mut cycles: Vec<(Vec<OutHe>, Vec<Vec<OutHe>>)> = selected
        .iter()
        .enumerate()
        .map(|(si, s)| {
            let fr = &w.fragments[s.fragment];
            let one = |cyc: &[He]| -> Vec<OutHe> {
                let mk = |h: &He| OutHe {
                    piece: fr.fedges[h.fe].piece,
                    fwd: h.fwd != s.flip,
                    si,
                    fe: h.fe,
                };
                if s.flip {
                    cyc.iter().rev().map(mk).collect()
                } else {
                    cyc.iter().map(mk).collect()
                }
            };
            (one(&fr.outer), fr.holes.iter().map(|h| one(h)).collect())
        })
        .collect();
    // piece → its uses
    let mut uses: HashMap<usize, Vec<(usize, bool)>> = HashMap::new();
    for (si, (outer, holes)) in cycles.iter().enumerate() {
        for cyc in std::iter::once(outer).chain(holes.iter()) {
            for h in cyc {
                uses.entry(h.piece).or_default().push((si, h.fwd));
            }
        }
    }
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while p[r] != r {
            r = p[r];
        }
        let mut c = x;
        while p[c] != r {
            let nx = p[c];
            p[c] = r;
            c = nx;
        }
        r
    }
    for u in uses.values() {
        if let [(a, fa), (b, fb)] = u[..] {
            if a != b && fa != fb && same_domain(w, selected, a, b, w.tol) {
                let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                parent[ra] = rb;
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for si in 0..n {
        let r = find(&mut parent, si);
        groups.entry(r).or_default().push(si);
    }
    let mut out: Vec<OutFace> = Vec::new();
    for (_, members) in groups {
        let plain = |out: &mut Vec<OutFace>, cycles: &mut Vec<(Vec<OutHe>, Vec<Vec<OutHe>>)>| {
            for &si in &members {
                let (outer, holes) = std::mem::take(&mut cycles[si]);
                out.push(OutFace {
                    base: si,
                    outer,
                    holes,
                    merged: false,
                });
            }
        };
        if members.len() == 1 {
            plain(&mut out, &mut cycles);
            continue;
        }
        let in_group = |si: usize| members.contains(&si);
        // halfedges that remain: pieces used inside the group by two of its fragments vanish
        let mut rest: Vec<OutHe> = Vec::new();
        for &si in &members {
            for cyc in std::iter::once(&cycles[si].0).chain(cycles[si].1.iter()) {
                for h in cyc {
                    let inner = uses[&h.piece].len() == 2
                        && uses[&h.piece].iter().all(|(s2, _)| in_group(*s2));
                    if !inner {
                        rest.push(*h);
                    }
                }
            }
        }
        let ends = |h: &OutHe| -> (usize, usize) {
            let fe = &w.fragments[selected[h.si].fragment].fedges[h.fe];
            if h.fwd {
                (fe.v0, fe.v1)
            } else {
                (fe.v1, fe.v0)
            }
        };
        let mut by_start: HashMap<usize, usize> = HashMap::new();
        let mut ok = true;
        for (k, h) in rest.iter().enumerate() {
            if by_start.insert(ends(h).0, k).is_some() {
                ok = false;
            }
        }
        let mut loops: Vec<Vec<OutHe>> = Vec::new();
        if ok {
            let mut used = vec![false; rest.len()];
            for k0 in 0..rest.len() {
                if used[k0] {
                    continue;
                }
                let mut cyc = Vec::new();
                let mut k = k0;
                loop {
                    if used[k] {
                        ok = false;
                        break;
                    }
                    used[k] = true;
                    cyc.push(rest[k]);
                    match by_start.get(&ends(&rest[k]).1) {
                        Some(&nk) => k = nk,
                        None => {
                            ok = false;
                            break;
                        }
                    }
                    if k == k0 {
                        break;
                    }
                }
                if !ok {
                    break;
                }
                loops.push(cyc);
            }
        }
        let base = members[0];
        let base_face = w.fragments[selected[base].fragment].face;
        let one_face = members
            .iter()
            .all(|&si| w.fragments[selected[si].fragment].face == base_face);
        if !ok || loops.is_empty() || (loops.len() > 1 && !one_face) {
            plain(&mut out, &mut cycles);
            continue;
        }
        let (outer, holes) = if loops.len() == 1 {
            (loops.pop().unwrap(), Vec::new())
        } else {
            // one input face: the loops live in its internal parameter plane, where the outer boundary
            // runs counter-clockwise and holes clockwise (the orientation flips for a flipped face)
            let area = |cyc: &[OutHe]| -> f64 {
                let fr = &w.fragments[selected[cyc[0].si].fragment];
                let hes: Vec<He> = cyc
                    .iter()
                    .map(|h| He {
                        fe: h.fe,
                        fwd: h.fwd,
                    })
                    .collect();
                let poly = crate::arrangement::cycle_poly_pub(&fr.fedges, &hes);
                let mut a = 0.0;
                for i in 0..poly.len() {
                    let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
                    a += p.0 * q.1 - q.0 * p.1;
                }
                if selected[cyc[0].si].flip {
                    -a
                } else {
                    a
                }
            };
            let areas: Vec<f64> = loops.iter().map(|l| area(l)).collect();
            let positive: Vec<usize> = (0..loops.len()).filter(|&i| areas[i] > 0.0).collect();
            if positive.len() != 1 {
                plain(&mut out, &mut cycles);
                continue;
            }
            let o = positive[0];
            let mut holes = Vec::new();
            let mut outer = Vec::new();
            for (i, l) in loops.into_iter().enumerate() {
                if i == o {
                    outer = l;
                } else {
                    holes.push(l);
                }
            }
            (outer, holes)
        };
        out.push(OutFace {
            base,
            outer,
            holes,
            merged: true,
        });
    }
    out
}

pub fn assemble(
    rep: &mut BRep,
    w: Work,
    selected: Vec<Selected>,
    op: Op,
    opts: &BoolOptions,
) -> Result<BooleanResult, BooleanError> {
    if opts.merge_faces {
        // merging is cosmetic: whatever it cannot do, the plain assembly can
        if let Ok(r) = assemble_with(rep, &w, &selected, op, opts, true) {
            return Ok(r);
        }
    }
    assemble_with(rep, &w, &selected, op, opts, false)
}

fn assemble_with(
    rep: &mut BRep,
    w: &Work,
    selected: &[Selected],
    _op: Op,
    opts: &BoolOptions,
    merge: bool,
) -> Result<BooleanResult, BooleanError> {
    let mut scratch = BRep::new();
    let mut history = ShapeHistory::default();
    let out_faces: Vec<OutFace> = if merge {
        merge_fragments(w, selected)
    } else {
        selected
            .iter()
            .enumerate()
            .map(|(si, s)| {
                let (outer, holes) = oriented(&w.fragments[s.fragment], s);
                OutFace {
                    base: si,
                    outer: out_cycle(w, selected, si, &outer),
                    holes: holes
                        .iter()
                        .map(|h| out_cycle(w, selected, si, h))
                        .collect(),
                    merged: false,
                }
            })
            .collect()
    };

    // ---- piece usage check
    let mut usage: BTreeMap<usize, (u32, u32)> = BTreeMap::new(); // piece → (forward uses, backward uses)
    for of in &out_faces {
        for cyc in std::iter::once(&of.outer).chain(of.holes.iter()) {
            for h in cyc {
                let e = usage.entry(h.piece).or_insert((0, 0));
                if h.fwd {
                    e.0 += 1;
                } else {
                    e.1 += 1;
                }
            }
        }
    }
    for (pi, (f, b)) in &usage {
        // degenerate edges (sphere poles, cone apex) are single-use by construction
        let degenerate = w.gedges[w.pieces[*pi].gedge].curve.is_none();
        if !degenerate && (*f != 1 || *b != 1) {
            let piece = &w.pieces[*pi];
            let g = &w.gedges[piece.gedge];
            let who: Vec<String> = piece
                .uses
                .iter()
                .map(|u| {
                    format!(
                        "face{}(body{}, sel={})",
                        u.face,
                        w.faces[u.face].body,
                        selected
                            .iter()
                            .any(|s| w.fragments[s.fragment].face == u.face)
                    )
                })
                .collect();
            let frs: Vec<String> = w
                .fragments
                .iter()
                .enumerate()
                .filter(|(_, fr)| {
                    fr.outer
                        .iter()
                        .chain(fr.holes.iter().flatten())
                        .any(|h| fr.fedges[h.fe].piece == *pi)
                })
                .map(|(i, fr)| {
                    format!(
                        "frag{}(face{}, selected={})",
                        i,
                        fr.face,
                        selected.iter().any(|s| s.fragment == i)
                    )
                })
                .collect();
            if std::env::var("KW_BOOL_DEBUG").is_ok() {
                for (name, v) in [("v0", piece.v0), ("v1", piece.v1)] {
                    let inc: Vec<String> = w
                        .pieces
                        .iter()
                        .enumerate()
                        .filter(|(i, q)| q.canon == *i && (q.v0 == v || q.v1 == v))
                        .map(|(i, q)| {
                            format!(
                                "piece{} {:?} faces {:?}",
                                i,
                                w.gedges[q.gedge].src,
                                q.uses.iter().map(|u| u.face).collect::<Vec<_>>()
                            )
                        })
                        .collect();
                    eprintln!("  {name} {:?}: {inc:?}", w.reg.pts[v]);
                }
            }
            return Err(BooleanError::Topology(format!(
                "edge piece {pi} ({:?}, {:?}->{:?}) used {f} forward / {b} backward times; uses {who:?}; fragments {frs:?}",
                g.src, w.reg.pts[piece.v0], w.reg.pts[piece.v1]
            )));
        }
    }

    // ---- vertices, curves, edges
    let mut vmap: HashMap<usize, VertexId> = HashMap::new();
    let mut cmap: HashMap<usize, Option<kw_brep::CurveId>> = HashMap::new();
    let mut emap: HashMap<usize, EdgeId> = HashMap::new();
    for &pi in usage.keys() {
        let p = &w.pieces[pi];
        let g = &w.gedges[p.gedge];
        for v in [p.v0, p.v1] {
            vmap.entry(v)
                .or_insert_with(|| scratch.add_vertex(w.reg.pts[v]));
        }
        let curve = *cmap
            .entry(p.gedge)
            .or_insert_with(|| g.curve.clone().map(|c| scratch.add_curve(c)));
        let e = scratch.add_edge(vmap[&p.v0], vmap[&p.v1], curve, p.range);
        emap.insert(pi, e);
        let whole_edge =
            (p.range.0 - g.range.0).abs() < 1e-12 && (p.range.1 - g.range.1).abs() < 1e-12;
        let origin = match g.src {
            GSrc::Orig(src) if whole_edge => {
                Origin::Preserved(EntityRef::Edge(src_edge_id(w, src)))
            }
            GSrc::Orig(src) => Origin::SplitFrom {
                parent: EntityRef::Edge(src_edge_id(w, src)),
                ordinal: 0,
            },
            GSrc::Cut => Origin::New {
                role: "intersection".into(),
            },
        };
        history.record(EntityRef::Edge(e), origin);
    }

    // ---- shells by connectivity (union-find on faces sharing pieces)
    let mut parent: Vec<usize> = (0..out_faces.len()).collect();
    fn find(p: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while p[r] != r {
            r = p[r];
        }
        let mut c = x;
        while p[c] != r {
            let n = p[c];
            p[c] = r;
            c = n;
        }
        r
    }
    let mut owner: HashMap<usize, usize> = HashMap::new();
    for (oi, of) in out_faces.iter().enumerate() {
        for cyc in std::iter::once(&of.outer).chain(of.holes.iter()) {
            for h in cyc {
                match owner.get(&h.piece) {
                    Some(&o) => {
                        let (a, b) = (find(&mut parent, o), find(&mut parent, oi));
                        parent[a] = b;
                    }
                    None => {
                        owner.insert(h.piece, oi);
                    }
                }
            }
        }
    }
    let mut comps: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for oi in 0..out_faces.len() {
        let r = find(&mut parent, oi);
        comps.entry(r).or_default().push(oi);
    }

    // ---- faces
    let body = scratch.add_body(BodyKind::Solid);
    let mut shells: Vec<ShellId> = Vec::new();
    let lump0 = scratch.add_lump(body);
    let mut surface_of: HashMap<usize, kw_brep::SurfaceId> = HashMap::new();
    let mut frag_count: HashMap<usize, u32> = HashMap::new();
    for of in &out_faces {
        *frag_count
            .entry(w.fragments[selected[of.base].fragment].face)
            .or_default() += 1;
    }
    let mut ordinal: HashMap<usize, u32> = HashMap::new();
    for (_, members) in comps {
        let shell = scratch.add_shell(lump0);
        shells.push(shell);
        for oi in members {
            let of = &out_faces[oi];
            let s = &selected[of.base];
            let fr = &w.fragments[s.fragment];
            let face = &w.faces[fr.face];
            let surf = *surface_of
                .entry(fr.face)
                .or_insert_with(|| scratch.add_surface(face.surface.clone()));
            let sense = if s.flip {
                face.sense.flip()
            } else {
                face.sense
            };
            let nf = scratch.add_face(shell, surf, sense);
            for (kind, cyc) in std::iter::once((LoopKind::Outer, &of.outer))
                .chain(of.holes.iter().map(|h| (LoopKind::Inner, h)))
            {
                let mut specs = Vec::new();
                for h in cyc {
                    let src = &w.fragments[selected[h.si].fragment];
                    let fe = &src.fedges[h.fe];
                    let piece = &w.pieces[h.piece];
                    let pc = if src.face == fr.face {
                        pcurve_for_use(w, fe, piece, face)?
                    } else {
                        pcurve_on_face(w, piece, face)?
                    };
                    let pcid = scratch.add_curve(pc);
                    specs.push(CoedgeSpec {
                        edge: emap[&h.piece],
                        sense: if h.fwd {
                            Sense::Forward
                        } else {
                            Sense::Reversed
                        },
                        pcurve: Some(pcid),
                    });
                }
                scratch.add_loop(nf, kind, &specs);
            }
            let origin = if of.merged {
                Origin::Modified(EntityRef::Face(face.id))
            } else if fr.whole && !s.flip {
                Origin::Preserved(EntityRef::Face(face.id))
            } else if frag_count[&fr.face] > 1 {
                let o = ordinal.entry(fr.face).or_default();
                let r = Origin::SplitFrom {
                    parent: EntityRef::Face(face.id),
                    ordinal: *o,
                };
                *o += 1;
                r
            } else {
                Origin::Modified(EntityRef::Face(face.id))
            };
            history.record(EntityRef::Face(nf), origin);
        }
    }
    // inputs that did not survive
    let kept: std::collections::HashSet<usize> = selected
        .iter()
        .map(|s| w.fragments[s.fragment].face)
        .collect();
    for (i, f) in w.faces.iter().enumerate() {
        if !kept.contains(&i) {
            history.mark_deleted(EntityRef::Face(f.id));
        }
    }

    // ---- lumps: positive shells are lumps; void shells go into the lump that contains them
    let total_volume = arrange_lumps(&mut scratch, body, lump0, &shells)?;

    // ---- verification
    if opts.verify {
        if let Err(errs) = scratch.validate().into_result() {
            return Err(BooleanError::Invalid(format!(
                "{:?}",
                &errs[..errs.len().min(3)]
            )));
        }
        // (the shell volumes were integrated once, by the lump arrangement)
        let vol = total_volume;
        if vol.is_nan() || vol <= 0.0 {
            return Err(BooleanError::Invalid(format!("non-positive volume {vol}")));
        }
    }

    // ---- move into the caller's B-rep and remap the history
    let (new_body, map) = rep.copy_body_from(&scratch, body, &CopyOptions::default());
    let mut out_hist = ShapeHistory::default();
    for (k, v) in history.origins {
        let key = match k {
            EntityRef::Face(f) => map.faces.get(&f).map(|n| EntityRef::Face(*n)),
            EntityRef::Edge(e) => map.edges.get(&e).map(|n| EntityRef::Edge(*n)),
            EntityRef::Vertex(x) => map.vertices.get(&x).map(|n| EntityRef::Vertex(*n)),
        };
        if let Some(key) = key {
            out_hist.record(key, v);
        }
    }
    for d in history.deleted_inputs {
        out_hist.mark_deleted(d);
    }
    Ok(BooleanResult {
        body: new_body,
        history: out_hist,
    })
}

fn src_edge_id(_w: &Work, e: EdgeId) -> EdgeId {
    e
}

/// pcurve of a use, expressed against the canonical piece's own curve parameterisation.
fn pcurve_for_use(
    w: &Work,
    fe: &FEdge,
    piece: &Piece,
    face: &FaceRec,
) -> Result<CurveGeom, BooleanError> {
    let g = &w.gedges[piece.gedge];
    let u = &w.pieces[fe.piece].uses[fe.use_idx];
    if u.defined_on == piece.gedge {
        return Ok(fe.pcurve.clone());
    }
    let curve = g
        .curve
        .as_ref()
        .ok_or_else(|| BooleanError::Topology("merged degenerate piece".into()))?;
    // hint: where the old pcurve starts (canonical start)
    let t_hint = if fe.reversed {
        fe.prange.1
    } else {
        fe.prange.0
    };
    let q = fe.pcurve.point_at(t_hint);
    // the canonical curve was fitted on another surface pair: it lies on this surface only to within
    // the fitting tolerance of that intersection, so aim the pcurve fit at the deviation actually present
    let dev = (0..=16)
        .map(|i| {
            let t = piece.range.0 + (piece.range.1 - piece.range.0) * i as f64 / 16.0;
            kw_nurbs::proj::surface_param_of(&face.surface, &curve.point_at(t), Some((q.x, q.y)))
                .distance
        })
        .fold(0.0f64, f64::max);
    let tol = (1e-9 * (1.0 + w.scale)).max(3.0 * dev);
    if tol > 1e-6 * (1.0 + w.scale) {
        return Err(BooleanError::Unsupported(
            "merged edge does not lie on the face surface",
        ));
    }
    pcurve_on(&face.surface, curve, piece.range, Some((q.x, q.y)), tol).ok_or(
        BooleanError::Unsupported("could not re-express a pcurve on a merged edge"),
    )
}

/// pcurve of a canonical piece on `face`, which is a different input face than the one the piece was
/// traced on (two coplanar faces merged into one).
fn pcurve_on_face(w: &Work, piece: &Piece, face: &FaceRec) -> Result<CurveGeom, BooleanError> {
    let g = &w.gedges[piece.gedge];
    let curve = g
        .curve
        .as_ref()
        .ok_or_else(|| BooleanError::Topology("merged degenerate piece".into()))?;
    let q = kw_nurbs::proj::surface_param_of(&face.surface, &curve.point_at(piece.range.0), None);
    let dev = (0..=16)
        .map(|i| {
            let t = piece.range.0 + (piece.range.1 - piece.range.0) * i as f64 / 16.0;
            kw_nurbs::proj::surface_param_of(&face.surface, &curve.point_at(t), Some((q.u, q.v)))
                .distance
        })
        .fold(0.0f64, f64::max);
    let tol = (1e-9 * (1.0 + w.scale)).max(3.0 * dev);
    if tol > 1e-6 * (1.0 + w.scale) {
        return Err(BooleanError::Unsupported(
            "merged edge does not lie on the face surface",
        ));
    }
    pcurve_on(&face.surface, curve, piece.range, Some((q.u, q.v)), tol).ok_or(
        BooleanError::Unsupported("could not express a pcurve on a merged face"),
    )
}

fn arrange_lumps(
    rep: &mut BRep,
    body: BodyId,
    lump0: kw_brep::LumpId,
    shells: &[ShellId],
) -> Result<f64, BooleanError> {
    let vols: Vec<f64> = shells
        .iter()
        .map(|s| {
            rep.shell_volume(*s)
                .ok_or_else(|| BooleanError::Invalid("shell volume".into()))
        })
        .collect::<Result<_, _>>()?;
    let outer: Vec<usize> = (0..shells.len()).filter(|&i| vols[i] > 0.0).collect();
    let voids: Vec<usize> = (0..shells.len()).filter(|&i| vols[i] < 0.0).collect();
    if outer.is_empty() {
        return Err(BooleanError::Invalid("no outer shell".into()));
    }
    // each outer shell gets its own lump (the first reuses lump0)
    let mut lump_of: HashMap<usize, kw_brep::LumpId> = HashMap::new();
    for (k, &i) in outer.iter().enumerate() {
        let l = if k == 0 { lump0 } else { rep.add_lump(body) };
        lump_of.insert(i, l);
    }
    // rebuild lump → shells lists
    for l in rep.bodies[body].lumps.clone() {
        rep.lumps[l].shells.clear();
    }
    for (&i, &l) in &lump_of {
        rep.lumps[l].shells.push(shells[i]);
        rep.shells[shells[i]].lump = l;
    }
    for &v in &voids {
        // a point of the void shell tested against every outer shell
        let face = rep.shells[shells[v]].faces[0];
        let (lo, hi) = rep
            .face_uv_bbox(face)
            .ok_or_else(|| BooleanError::Invalid("void face".into()))?;
        let surf = rep.surfaces[rep.faces[face].surface].clone();
        let _ = (lo, hi);
        let probe = rep
            .face_bbox(face)
            .map(|b| b.center())
            .ok_or_else(|| BooleanError::Invalid("void face".into()))?;
        let _ = surf;
        let mut best: Option<(f64, usize)> = None;
        for &o in &outer {
            let idx = SolidIndex::from_faces(rep, &rep.shells[shells[o]].faces.clone())
                .ok_or_else(|| BooleanError::Invalid("shell index".into()))?;
            if matches!(idx.classify(&probe, 1e-9), Ok(kw_brep::PointClass::In))
                && best.is_none_or(|b| vols[o] < b.0)
            {
                best = Some((vols[o], o));
            }
        }
        let Some((_, o)) = best else {
            return Err(BooleanError::Invalid(
                "void shell outside every outer shell".into(),
            ));
        };
        let l = lump_of[&o];
        rep.lumps[l].shells.push(shells[v]);
        rep.shells[shells[v]].lump = l;
    }
    Ok(vols.iter().sum())
}

#[allow(dead_code)]
fn _face_id(_: FaceId) {}
