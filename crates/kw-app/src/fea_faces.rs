//! Faces of a study that find themselves again. A support or a load is put on a face by its tag, which
//! counts the faces of the included bodies in order and so changes when the model gains or loses a face or a
//! body. The study also keeps what the face looked like (its body, the centre of its surface, its area and
//! normal); when the model changes the face with that signature gets the tag it has now, and a face that is
//! gone is marked missing instead of letting its tag land on some other face.

use crate::model_source::Combined;
use kw_store::{BodyKey, FaceSig, FeaStudy};
use kw_tess::{tessellate_body, TessOptions};

/// Tags from here up mean "a face that is gone": no face has such a number, and a study that holds one is
/// not run.
pub const MISSING_BASE: u32 = 0xFFF0_0000;

pub fn is_missing(tag: u32) -> bool {
    tag >= MISSING_BASE
}

/// The signature of every face of the combined bodies, with the tag the face has now.
pub fn face_signatures(c: &Combined) -> Vec<(u32, FaceSig)> {
    let mut out = Vec::new();
    let mut offset = 0u32;
    for (key, id) in &c.bodies {
        let Ok(t) = tessellate_body(
            &c.brep,
            *id,
            &TessOptions {
                chord_tol: 0.02,
                angle_tol: 0.3,
            },
        ) else {
            continue;
        };
        let n = t.face_keys.len();
        let mut area = vec![0.0f64; n];
        let mut centre = vec![[0.0f64; 3]; n];
        let mut normal = vec![[0.0f64; 3]; n];
        for (k, tri) in t.mesh.indices.as_chunks::<3>().0.iter().enumerate() {
            let f = t.mesh.tri_face[k] as usize;
            let p = |i: u32| {
                let q = t.mesh.positions[i as usize];
                [q.x, q.y, q.z]
            };
            let (a, b, cc) = (p(tri[0]), p(tri[1]), p(tri[2]));
            let (u, v) = (
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]],
            );
            let cr = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let w = 0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
            area[f] += w;
            for k in 0..3 {
                centre[f][k] += w * (a[k] + b[k] + cc[k]) / 3.0;
                normal[f][k] += 0.5 * cr[k];
            }
        }
        let scale = c.brep.body_bbox(*id).diagonal().max(1e-9);
        for f in 0..n {
            let w = area[f].max(1e-300);
            out.push((
                offset + f as u32,
                FaceSig {
                    body: key.clone(),
                    centre: [centre[f][0] / w, centre[f][1] / w, centre[f][2] / w],
                    area: area[f],
                    normal: normal[f],
                    scale,
                },
            ));
        }
        offset += n as u32;
    }
    out
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n < 1e-12 {
        [0.0; 3]
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

/// How the tolerances of a match are set: the centre may have moved by this fraction of the body's size, the
/// area may differ by this fraction.
const CENTRE_TOL: f64 = 0.02;
const AREA_TOL: f64 = 0.05;

/// The tag now of the face with signature `sig`, if there is one: the same body, a similar area and
/// direction, and the nearest centre within the tolerance.
pub fn find_face(current: &[(u32, FaceSig)], sig: &FaceSig) -> Option<u32> {
    let scale = if sig.scale > 0.0 { sig.scale } else { 1.0 };
    let sn = unit(sig.normal);
    let mut best: Option<(f64, u32)> = None;
    for (tag, c) in current {
        if c.body != sig.body {
            continue;
        }
        let d = ((c.centre[0] - sig.centre[0]).powi(2)
            + (c.centre[1] - sig.centre[1]).powi(2)
            + (c.centre[2] - sig.centre[2]).powi(2))
        .sqrt();
        if d > CENTRE_TOL * scale {
            continue;
        }
        if (c.area - sig.area).abs() > AREA_TOL * sig.area.max(1e-9) {
            continue;
        }
        // flat faces keep their direction; a face curved all round has a normal that cancels
        let cn = unit(c.normal);
        let dot = cn[0] * sn[0] + cn[1] * sn[1] + cn[2] * sn[2];
        if (sn != [0.0; 3] && cn != [0.0; 3]) && dot < 0.98 {
            continue;
        }
        if best.is_none_or(|b| d < b.0) {
            best = Some((d, *tag));
        }
    }
    best.map(|b| b.1)
}

/// What a reconciliation did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Reconciled {
    /// Tags that changed because the faces moved in the numbering.
    pub moved: usize,
    /// Faces that are gone.
    pub missing: usize,
    /// Faces that were missing and are back.
    pub found: usize,
}

/// Bring the study's faces up to date with the model whose faces are `current`: a tag without a signature
/// takes the signature of the face it names now (the first time a study is seen); every signature is looked
/// up again, and tags are rewritten, all at once. A face that cannot be found gets a missing tag; one that
/// was missing and can be found again gets its face back.
pub fn reconcile(study: &mut FeaStudy, current: &[(u32, FaceSig)]) -> Reconciled {
    let mut r = Reconciled::default();
    for tag in study.face_tags() {
        if is_missing(tag) || study.face_sigs.iter().any(|(t, _)| *t == tag) {
            continue;
        }
        if let Some((_, s)) = current.iter().find(|(t, _)| *t == tag) {
            study.face_sigs.push((tag, s.clone()));
        }
    }
    let mut map: Vec<(u32, u32)> = Vec::new();
    let mut next_missing = study
        .face_sigs
        .iter()
        .filter(|(t, _)| is_missing(*t))
        .map(|(t, _)| t - MISSING_BASE + 1)
        .max()
        .unwrap_or(0);
    for (tag, sig) in &study.face_sigs {
        match find_face(current, sig) {
            Some(new) => {
                if is_missing(*tag) {
                    r.found += 1;
                }
                if new != *tag {
                    if !is_missing(*tag) {
                        r.moved += 1;
                    }
                    map.push((*tag, new));
                }
            }
            None if !is_missing(*tag) => {
                r.missing += 1;
                map.push((*tag, MISSING_BASE + next_missing));
                next_missing += 1;
            }
            None => {}
        }
    }
    study.remap_faces(&map);
    r
}

/// A short description of the body of a signature for messages.
pub fn describe(sig: &FaceSig) -> String {
    let body = match &sig.body {
        BodyKey::Design(id) => format!("{id}"),
        BodyKey::Assembly(n) => n.clone(),
    };
    format!(
        "face of {body} near ({:.1}, {:.1}, {:.1}), {:.0} mm²",
        sig.centre[0], sig.centre[1], sig.centre[2], sig.area
    )
}
