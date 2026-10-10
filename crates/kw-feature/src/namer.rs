//! Turns a kernel [`ShapeHistory`] into persistent [`TopoName`]s.

use crate::id::FeatureId;
use crate::naming::{NameOrigin, TopoName};
use kw_brep::{BRep, BodyId, EntityRef, Origin, ShapeHistory};
use std::collections::{HashMap, HashSet};

pub struct Namer {
    pub feature: FeatureId,
    counts: HashMap<NameOrigin, u32>,
}

fn tag(n: &TopoName) -> String {
    format!("{:032x}", n.stable_hash())
}

impl Namer {
    pub fn new(feature: FeatureId) -> Namer {
        Namer {
            feature,
            counts: HashMap::new(),
        }
    }

    pub fn unique(&mut self, origin: NameOrigin) -> TopoName {
        let c = self.counts.entry(origin.clone()).or_insert(0);
        let t = TopoName {
            feature: self.feature,
            origin,
            ordinal: *c,
        };
        *c += 1;
        t
    }

    /// Names of the faces of `body`, in face order. Faces that survive an operation keep their name;
    /// new faces get names from how they were made.
    pub fn name_result(
        &mut self,
        rep: &BRep,
        body: BodyId,
        history: &ShapeHistory,
        in_names: &HashMap<EntityRef, TopoName>,
        new_origin: &dyn Fn(&str) -> NameOrigin,
    ) -> HashMap<EntityRef, TopoName> {
        let mut out: HashMap<EntityRef, TopoName> = HashMap::new();
        let mut taken: HashSet<TopoName> = HashSet::new();
        for f in rep.body_faces(body) {
            let key = EntityRef::Face(f);
            let origin = history.origins.get(&key).cloned().unwrap_or(Origin::New {
                role: "face".into(),
            });
            let mut name = self.derive(rep, &origin, in_names, new_origin);
            if taken.contains(&name) {
                // two result faces claim one name (a face cut in two without a split record)
                let parent = name.clone();
                let mut k = 0;
                loop {
                    let cand = self.unique(NameOrigin::Split {
                        parent: Box::new(parent.clone()),
                        ordinal: 1000 + k,
                    });
                    if !taken.contains(&cand) {
                        name = cand;
                        break;
                    }
                    k += 1;
                }
            }
            taken.insert(name.clone());
            out.insert(key, name);
        }
        out
    }

    fn derive(
        &mut self,
        rep: &BRep,
        origin: &Origin,
        in_names: &HashMap<EntityRef, TopoName>,
        new_origin: &dyn Fn(&str) -> NameOrigin,
    ) -> TopoName {
        match origin {
            Origin::Preserved(src) | Origin::Modified(src) => match in_names.get(src) {
                Some(n) => n.clone(),
                None => self.unique(NameOrigin::Primitive {
                    role: "unnamed-input".into(),
                }),
            },
            Origin::New { role } => {
                let o = new_origin(role);
                self.unique(o)
            }
            Origin::Generated { from, role } => {
                let adj: Vec<TopoName> = match from {
                    EntityRef::Edge(e) => {
                        let mut v: Vec<TopoName> = rep
                            .faces_around_edge(*e)
                            .into_iter()
                            .filter_map(|f| in_names.get(&EntityRef::Face(f)).cloned())
                            .collect();
                        v.sort();
                        v.dedup();
                        v
                    }
                    other => in_names.get(other).cloned().into_iter().collect(),
                };
                match adj.split_first() {
                    Some((first, rest)) => {
                        let tail: Vec<String> = rest.iter().map(tag).collect();
                        self.unique(NameOrigin::Generated {
                            from: Box::new(first.clone()),
                            role: format!("{role}|{}", tail.join("|")),
                        })
                    }
                    None => self.unique(NameOrigin::Primitive {
                        role: role.to_string(),
                    }),
                }
            }
            Origin::SplitFrom { parent, ordinal } => match in_names.get(parent) {
                Some(p) => self.unique(NameOrigin::Split {
                    parent: Box::new(p.clone()),
                    ordinal: *ordinal,
                }),
                None => self.unique(NameOrigin::Primitive {
                    role: format!("split-{ordinal}"),
                }),
            },
            Origin::MergedFrom(v) => {
                let mut ns: Vec<TopoName> =
                    v.iter().filter_map(|r| in_names.get(r).cloned()).collect();
                ns.sort();
                ns.dedup();
                if ns.len() == 1 {
                    return ns.pop().unwrap();
                }
                self.unique(NameOrigin::Merge(ns))
            }
        }
    }
}
