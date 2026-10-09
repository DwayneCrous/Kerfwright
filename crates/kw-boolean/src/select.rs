//! Phase 4: classify every fragment against the other operand and select by operation.

use crate::model::*;
use crate::{BooleanError, Op};
use kw_brep::{BRep, PointClass, PreparedFace, SolidIndex};
use kw_nurbs::Surface;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FragClass {
    In,
    Out,
    OnSame,
    OnOpposite,
}

#[derive(Debug, Clone, Copy)]
pub struct Selected {
    pub fragment: usize,
    pub flip: bool,
}

pub fn classify_fragments(_rep: &BRep, w: &Work) -> Result<Vec<FragClass>, BooleanError> {
    // classification indexes are built from the parameter-space data the Boolean already holds
    let solid_of = |body: usize| -> Result<SolidIndex, BooleanError> {
        let prepared: Vec<PreparedFace> = w
            .faces
            .iter()
            .filter(|f| f.body == body)
            .map(|f| PreparedFace {
                surface: f.surface.clone(),
                sense: f.sense,
                index: f.loop_index.clone(),
                uv_lo: f.uv_lo,
                uv_hi: f.uv_hi,
                bbox: f.bbox,
            })
            .collect();
        if prepared.is_empty() {
            return Err(BooleanError::BadInput("operand without geometry"));
        }
        Ok(SolidIndex::from_prepared(prepared))
    };
    let idx = [solid_of(0)?, solid_of(1)?];
    // Fragments of one body that touch along a piece which is not part of an intersection (no use from the
    // other body, no cut / imprint use) lie on the same side of the other body: classify one per component.
    let nfr = w.fragments.len();
    let mut parent: Vec<usize> = (0..nfr).collect();
    fn root(p: &mut [usize], x: usize) -> usize {
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
    let is_cut_piece = |pi: usize| -> bool {
        let uses = &w.pieces[pi].uses;
        uses.iter().any(|u| u.boundary_fwd.is_none())
            || uses
                .iter()
                .any(|u| w.faces[u.face].body != w.faces[uses[0].face].body)
    };
    let mut first_user: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    for (fi, fr) in w.fragments.iter().enumerate() {
        for cyc in std::iter::once(&fr.outer).chain(fr.holes.iter()) {
            for h in cyc {
                let pi = fr.fedges[h.fe].piece;
                if is_cut_piece(pi) {
                    continue;
                }
                match first_user.get(&pi) {
                    Some(&o) if w.faces[w.fragments[o].face].body == w.faces[fr.face].body => {
                        let (a, b) = (root(&mut parent, o), root(&mut parent, fi));
                        parent[a] = b;
                    }
                    Some(_) => {}
                    None => {
                        first_user.insert(pi, fi);
                    }
                }
            }
        }
    }
    let mut members: std::collections::BTreeMap<usize, Vec<usize>> =
        std::collections::BTreeMap::new();
    for fi in 0..nfr {
        let r = root(&mut parent, fi);
        members.entry(r).or_default().push(fi);
    }
    let classify_one = |fi: usize| -> Result<FragClass, BooleanError> {
        let fr = &w.fragments[fi];
        let face = &w.faces[fr.face];
        let other = 1 - face.body;
        let p = face.point_at(fr.sample_uv);
        if !idx[other].bbox.contains(&p, w.tol) {
            return Ok(FragClass::Out);
        }
        let class = idx[other].classify(&p, w.tol).map_err(|_| {
            BooleanError::Classification("ray casting was ambiguous for a fragment")
        })?;
        Ok(match class {
            PointClass::In => FragClass::In,
            PointClass::Out => FragClass::Out,
            PointClass::On => {
                let normals = idx[other].on_normals(&p, 4.0 * w.tol);
                let n = face.surface.normal_at(fr.sample_uv.0, fr.sample_uv.1) * face.sgn;
                let (mut same, mut opp) = (false, false);
                for m in normals {
                    if n.dot(&m) > 0.0 {
                        same = true;
                    } else {
                        opp = true;
                    }
                }
                match (same, opp) {
                    (true, false) => FragClass::OnSame,
                    (false, true) => FragClass::OnOpposite,
                    _ => {
                        return Err(BooleanError::Classification(
                            "fragment lies on an edge of the other operand",
                        ))
                    }
                }
            }
        })
    };
    let mut out = vec![FragClass::Out; nfr];
    for (_, group) in members {
        // a component with an ON member is classified fragment by fragment (it cannot span a coincidence)
        let mut result: Option<FragClass> = None;
        let mut last_err = None;
        for &fi in &group {
            match classify_one(fi) {
                Ok(c) => {
                    result = Some(c);
                    break;
                }
                Err(e) => last_err = Some(e),
            }
        }
        let Some(c) = result else {
            return Err(last_err.unwrap());
        };
        if matches!(c, FragClass::OnSame | FragClass::OnOpposite) {
            for &fi in &group {
                out[fi] = classify_one(fi)?;
            }
        } else {
            for &fi in &group {
                out[fi] = c;
            }
        }
    }
    Ok(out)
}

pub fn select(rep: &BRep, w: &mut Work, op: Op) -> Result<Vec<Selected>, BooleanError> {
    let classes = classify_fragments(rep, w)?;
    if std::env::var("KW_BOOL_DEBUG").is_ok() {
        for (i, fr) in w.fragments.iter().enumerate() {
            let f = &w.faces[fr.face];
            eprintln!(
                "frag {i}: face {} body {} class {:?} whole {} sample_uv {:?} p {:?} outer_len {} holes {}",
                fr.face,
                f.body,
                classes[i],
                fr.whole,
                fr.sample_uv,
                f.point_at(fr.sample_uv),
                fr.outer.len(),
                fr.holes.len()
            );
        }
    }
    let mut sel = Vec::new();
    for (i, fr) in w.fragments.iter().enumerate() {
        let body = w.faces[fr.face].body;
        let c = classes[i];
        let (keep, flip) = match (op, body, c) {
            (Op::Union, 0, FragClass::Out | FragClass::OnSame) => (true, false),
            (Op::Union, 1, FragClass::Out) => (true, false),
            (Op::Intersect, 0, FragClass::In | FragClass::OnSame) => (true, false),
            (Op::Intersect, 1, FragClass::In) => (true, false),
            (Op::Subtract, 0, FragClass::Out | FragClass::OnOpposite) => (true, false),
            (Op::Subtract, 1, FragClass::In) => (true, true),
            _ => (false, false),
        };
        if keep {
            sel.push(Selected { fragment: i, flip });
        }
    }
    if sel.is_empty() {
        return Err(BooleanError::Empty);
    }
    Ok(sel)
}
