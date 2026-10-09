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

pub fn assemble(
    rep: &mut BRep,
    w: Work,
    selected: Vec<Selected>,
    _op: Op,
    opts: &BoolOptions,
) -> Result<BooleanResult, BooleanError> {
    let mut scratch = BRep::new();
    let mut history = ShapeHistory::default();

    // ---- piece usage check
    let mut usage: BTreeMap<usize, (u32, u32)> = BTreeMap::new(); // piece → (forward uses, backward uses)
    for s in &selected {
        let fr = &w.fragments[s.fragment];
        let (outer, holes) = oriented(fr, s);
        for cyc in std::iter::once(&outer).chain(holes.iter()) {
            for h in cyc {
                let pi = fr.fedges[h.fe].piece;
                let e = usage.entry(pi).or_insert((0, 0));
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
                Origin::Preserved(EntityRef::Edge(src_edge_id(&w, src)))
            }
            GSrc::Orig(src) => Origin::SplitFrom {
                parent: EntityRef::Edge(src_edge_id(&w, src)),
                ordinal: 0,
            },
            GSrc::Cut => Origin::New {
                role: "intersection".into(),
            },
        };
        history.record(EntityRef::Edge(e), origin);
    }

    // ---- shells by connectivity (union-find on fragments sharing pieces)
    let mut parent: Vec<usize> = (0..selected.len()).collect();
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
    for (si, s) in selected.iter().enumerate() {
        let fr = &w.fragments[s.fragment];
        for cyc in std::iter::once(&fr.outer).chain(fr.holes.iter()) {
            for h in cyc {
                let pi = fr.fedges[h.fe].piece;
                match owner.get(&pi) {
                    Some(&o) => {
                        let (a, b) = (find(&mut parent, o), find(&mut parent, si));
                        parent[a] = b;
                    }
                    None => {
                        owner.insert(pi, si);
                    }
                }
            }
        }
    }
    let mut comps: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for si in 0..selected.len() {
        let r = find(&mut parent, si);
        comps.entry(r).or_default().push(si);
    }

    // ---- faces
    let body = scratch.add_body(BodyKind::Solid);
    let mut shells: Vec<ShellId> = Vec::new();
    let lump0 = scratch.add_lump(body);
    let mut surface_of: HashMap<usize, kw_brep::SurfaceId> = HashMap::new();
    let mut frag_count: HashMap<usize, u32> = HashMap::new();
    for s in &selected {
        *frag_count.entry(w.fragments[s.fragment].face).or_default() += 1;
    }
    let mut ordinal: HashMap<usize, u32> = HashMap::new();
    for (_, members) in comps {
        let shell = scratch.add_shell(lump0);
        shells.push(shell);
        for si in members {
            let s = &selected[si];
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
            let (outer, holes) = oriented(fr, s);
            for (kind, cyc) in std::iter::once((LoopKind::Outer, &outer))
                .chain(holes.iter().map(|h| (LoopKind::Inner, h)))
            {
                let mut specs = Vec::new();
                for h in cyc {
                    let fe = &fr.fedges[h.fe];
                    let pi = fe.piece;
                    let piece = &w.pieces[pi];
                    let pc = pcurve_for_use(&w, fe, piece, face)?;
                    let pcid = scratch.add_curve(pc);
                    specs.push(CoedgeSpec {
                        edge: emap[&pi],
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
            let origin = if fr.whole && !s.flip {
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
    arrange_lumps(&mut scratch, body, lump0, &shells)?;

    // ---- verification
    if opts.verify {
        if let Err(errs) = scratch.validate().into_result() {
            return Err(BooleanError::Invalid(format!(
                "{:?}",
                &errs[..errs.len().min(3)]
            )));
        }
        let vol = scratch
            .volume(body)
            .ok_or_else(|| BooleanError::Invalid("volume not computable".into()))?;
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

fn arrange_lumps(
    rep: &mut BRep,
    body: BodyId,
    lump0: kw_brep::LumpId,
    shells: &[ShellId],
) -> Result<(), BooleanError> {
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
    Ok(())
}

#[allow(dead_code)]
fn _face_id(_: FaceId) {}
