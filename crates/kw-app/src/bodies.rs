//! The bodies of a document as the app sees them: every head of the feature timeline, with a name, a colour
//! and one display B-rep to draw and pick from.
//!
//! With a single body the display B-rep *is* that body's B-rep, so face and edge ids mean the same thing
//! everywhere. With several, the bodies are copied into one merged B-rep and `origin` translates a picked
//! face or edge back to the body (and the ids inside that body's own output) it came from.

use crate::viewport::Pick;
use kw_brep::{BRep, CopyOptions, EdgeId, FaceId};
use kw_feature::{Document, EvalOutput, Evaluation, FeatureId};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

pub const DEFAULT_COLOUR: [u8; 3] = [196, 200, 208];

/// A body as listed in the browser: no geometry needed.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyInfo {
    pub head: FeatureId,
    pub root: FeatureId,
    pub name: String,
    pub colour: [u8; 3],
    pub material: Option<String>,
}

/// The bodies of `doc` (all of them, hidden or not), in the order they were started.
pub fn summaries(doc: &Document) -> Vec<BodyInfo> {
    let heads = doc.heads();
    let mut roots: Vec<FeatureId> = heads.iter().map(|h| doc.body_root(*h)).collect();
    roots.sort_by_key(|r| doc.index_of(*r));
    let mut out: Vec<BodyInfo> = heads
        .into_iter()
        .map(|head| {
            let root = doc.body_root(head);
            let ap = doc.appearance_of(root);
            let rank = roots.iter().position(|r| *r == root).unwrap_or(0) + 1;
            BodyInfo {
                head,
                root,
                name: ap
                    .and_then(|a| a.name.clone())
                    .unwrap_or_else(|| format!("Body{rank}")),
                colour: ap.and_then(|a| a.colour).unwrap_or(DEFAULT_COLOUR),
                material: ap.and_then(|a| a.material.clone()),
            }
        })
        .collect();
    out.sort_by_key(|b| doc.index_of(b.root));
    out
}

pub struct Body {
    /// The feature whose output is this body.
    pub head: FeatureId,
    /// The feature that started the body: its name, colour and material are stored under this id.
    pub root: FeatureId,
    pub name: String,
    pub colour: [u8; 3],
    pub material: Option<String>,
    pub output: Arc<EvalOutput>,
}

pub struct Bodies {
    pub list: Vec<Body>,
    /// What is drawn and picked: the body's own B-rep when there is one body, a merged copy otherwise.
    pub brep: Arc<BRep>,
    merged: bool,
    faces: HashMap<FaceId, (usize, FaceId)>,
    edges: HashMap<EdgeId, (usize, EdgeId)>,
    /// Changes whenever any body's geometry, name or colour changes.
    pub key: u128,
}

impl Bodies {
    /// The bodies of `doc` that evaluated, except those whose root is in `hidden`. `None` without any.
    pub fn new(doc: &Document, ev: &Evaluation, hidden: &HashSet<FeatureId>) -> Option<Bodies> {
        let heads = doc.heads();
        // numbering follows the order the bodies were started in
        let mut roots: Vec<FeatureId> = heads.iter().map(|h| doc.body_root(*h)).collect();
        roots.sort_by_key(|r| doc.index_of(*r));
        let mut list: Vec<Body> = Vec::new();
        for head in heads {
            let root = doc.body_root(head);
            let Some(output) = ev.output(head).filter(|o| o.brep().is_some()).cloned() else {
                continue;
            };
            if hidden.contains(&root) {
                continue;
            }
            let ap = doc.appearance_of(root);
            let rank = roots.iter().position(|r| *r == root).unwrap_or(0) + 1;
            list.push(Body {
                head,
                root,
                name: ap
                    .and_then(|a| a.name.clone())
                    .unwrap_or_else(|| format!("Body{rank}")),
                colour: ap.and_then(|a| a.colour).unwrap_or(DEFAULT_COLOUR),
                material: ap.and_then(|a| a.material.clone()),
                output,
            });
        }
        if list.is_empty() {
            return None;
        }
        let mut h = kw_feature::Hasher128::new();
        for b in &list {
            h.u128(b.output.geom_hash).u128(b.output.names_hash);
            h.bytes(&b.colour);
            h.bytes(b.name.as_bytes());
        }
        let key = h.finish();
        if list.len() == 1 {
            let brep = list[0].output.brep().cloned()?;
            return Some(Bodies {
                list,
                brep,
                merged: false,
                faces: HashMap::new(),
                edges: HashMap::new(),
                key,
            });
        }
        let mut merged = BRep::new();
        let (mut faces, mut edges) = (HashMap::new(), HashMap::new());
        for (i, b) in list.iter().enumerate() {
            let (rep, body) = (b.output.brep()?, b.output.body()?);
            let (_, map) = merged.copy_body_from(rep, body, &CopyOptions::default());
            for (old, new) in map.faces {
                faces.insert(new, (i, old));
            }
            for (old, new) in map.edges {
                edges.insert(new, (i, old));
            }
        }
        Some(Bodies {
            list,
            brep: Arc::new(merged),
            merged: true,
            faces,
            edges,
            key,
        })
    }

    pub fn is_merged(&self) -> bool {
        self.merged
    }

    /// Colours in the order of the display B-rep's bodies.
    pub fn colours(&self) -> Vec<[u8; 3]> {
        self.list.iter().map(|b| b.colour).collect()
    }

    /// The body a picked face or edge belongs to, and the pick in that body's own output.
    pub fn origin(&self, p: Pick) -> Option<(usize, Pick)> {
        if !self.merged {
            return Some((0, p));
        }
        match p {
            Pick::Face(f) => self.faces.get(&f).map(|(i, o)| (*i, Pick::Face(*o))),
            Pick::Edge(e) => self.edges.get(&e).map(|(i, o)| (*i, Pick::Edge(*o))),
        }
    }

    pub fn index_of_root(&self, root: FeatureId) -> Option<usize> {
        self.list.iter().position(|b| b.root == root)
    }

    pub fn index_of_head(&self, head: FeatureId) -> Option<usize> {
        self.list.iter().position(|b| b.head == head)
    }
}

impl std::fmt::Debug for Bodies {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Bodies({:?})",
            self.list
                .iter()
                .map(|b| b.name.as_str())
                .collect::<Vec<_>>()
        )
    }
}

/// Two bodies that share volume.
#[derive(Debug, Clone, PartialEq)]
pub struct Overlap {
    pub a: usize,
    pub b: usize,
    /// Volume of the common part, mm³.
    pub volume: Option<f64>,
    /// Why it could not be worked out (the Boolean refused the pair).
    pub error: Option<String>,
}

/// Every pair of bodies whose bounding boxes touch, with the volume the two share. Pairs that do not
/// overlap are left out.
pub fn interference(bodies: &Bodies) -> Vec<Overlap> {
    let mut out = Vec::new();
    for i in 0..bodies.list.len() {
        for j in i + 1..bodies.list.len() {
            let (oa, ob) = (&bodies.list[i].output, &bodies.list[j].output);
            // a surface encloses no volume to overlap
            if oa.is_sheet() || ob.is_sheet() {
                continue;
            }
            let (Some((ra, ba)), Some((rb, bb))) =
                (oa.brep().zip(oa.body()), ob.brep().zip(ob.body()))
            else {
                continue;
            };
            if !ra.body_bbox(ba).intersects(&rb.body_bbox(bb)) {
                continue;
            }
            let mut rep = BRep::new();
            let (a, _) = rep.copy_body_from(ra, ba, &CopyOptions::default());
            let (b, _) = rep.copy_body_from(rb, bb, &CopyOptions::default());
            match kw_boolean::intersect(&mut rep, a, b) {
                Ok(r) => {
                    let v = rep.mass_properties(r.body).map(|m| m.volume).unwrap_or(0.0);
                    if v > 1e-9 {
                        out.push(Overlap {
                            a: i,
                            b: j,
                            volume: Some(v),
                            error: None,
                        });
                    }
                }
                Err(e) => {
                    let text = format!("{e:?}");
                    // a pair with nothing in common is the normal case, not a problem
                    if !text.contains("Empty") {
                        out.push(Overlap {
                            a: i,
                            b: j,
                            volume: None,
                            error: Some(text),
                        });
                    }
                }
            }
        }
    }
    out
}
