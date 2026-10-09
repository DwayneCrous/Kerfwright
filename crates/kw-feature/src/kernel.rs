//! Kernel-backed feature evaluation: maps feature parameters to `kw-model` / `kw-boolean` /
//! `kw-blend` operations and turns their [`ShapeHistory`] into persistent [`TopoName`]s.
//!
//! This is the thin adapter layer of Phase 3; the parametric engine around it (expressions,
//! incremental evaluation, repair) belongs to Phase 4.

use crate::dag::{DagNode, NodeId};
use crate::eval::{EvalError, EvalOutput, FeatureEvaluator};
use crate::naming::{NameMap, NameOrigin, TopoName};
use kw_blend::{chamfer, fillet, fillet_law, RadiusLaw};
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{BRep, BodyId, CopyOptions, EntityRef, FaceId, Origin, ShapeHistory};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{
    draft_faces, extrude, loft, offset_body, revolve, shell, sweep, ExtrudeOptions, Profile,
    ProfileEdge, ProfileLoop, SweepOptions,
};
use kw_nurbs::CurveGeom;
use kw_sketch::{EntityKind, Sketch};
use std::collections::HashMap;
use std::sync::Arc;

/// Pair of faces (by persistent name) whose common edge is selected.
#[derive(Debug, Clone)]
pub struct EdgeSel(pub TopoName, pub TopoName);

#[derive(Debug, Clone)]
pub enum FeatureOp {
    Extrude {
        profile: Profile,
        vector: Vector3<f64>,
        symmetric: bool,
    },
    Revolve {
        profile: Profile,
        axis_origin: Point3<f64>,
        axis: Vector3<f64>,
        angle: f64,
    },
    Loft {
        profiles: Vec<Profile>,
    },
    Sweep {
        profile: Profile,
        path: CurveGeom,
    },
    /// `inputs[0] ∘ inputs[1]`
    Boolean {
        op: Op,
    },
    Fillet {
        edges: Vec<EdgeSel>,
        radius: f64,
    },
    Chamfer {
        edges: Vec<EdgeSel>,
        d1: f64,
        d2: f64,
    },
    /// One edge rounded with a radius that follows `law` along it (straight edge between planes /
    /// cylinders, or a closed circular rim).
    VariableFillet {
        edge: EdgeSel,
        law: RadiusLaw,
    },
    Shell {
        thickness: f64,
        open: Vec<TopoName>,
    },
    Draft {
        faces: Vec<TopoName>,
        neutral_origin: Point3<f64>,
        pull: Vector3<f64>,
        angle: f64,
    },
    Offset {
        distance: f64,
    },
}

/// Evaluator for the geometry-producing node kinds; each node's parameters are registered by `NodeId`.
#[derive(Default)]
pub struct KernelEvaluator {
    pub ops: HashMap<NodeId, FeatureOp>,
}

impl KernelEvaluator {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn register(&mut self, id: NodeId, op: FeatureOp) {
        self.ops.insert(id, op);
    }
}

fn kernel<E: std::fmt::Display>(what: &str, e: E) -> EvalError {
    EvalError::Kernel(format!("{what}: {e}"))
}

fn sole_body(rep: &BRep) -> Result<BodyId, EvalError> {
    rep.bodies
        .keys()
        .next()
        .ok_or_else(|| EvalError::Kernel("input has no body".into()))
}

fn key(n: &TopoName) -> String {
    format!("{n:?}")
}

fn reverse_names(names: &NameMap) -> HashMap<EntityRef, TopoName> {
    names.by_name.iter().map(|(n, e)| (*e, n.clone())).collect()
}

/// Face of `rep` that carries `name`.
fn face_named(map: &NameMap, name: &TopoName) -> Result<FaceId, EvalError> {
    match map.by_name.get(name) {
        Some(EntityRef::Face(f)) => Ok(*f),
        _ => Err(EvalError::Reference(format!(
            "{name:?} does not resolve to a face"
        ))),
    }
}

fn edge_between(rep: &BRep, a: FaceId, b: FaceId) -> Result<kw_brep::EdgeId, EvalError> {
    for lp in &rep.faces[a].loops {
        for co in rep.loop_coedges(*lp) {
            let e = rep.coedges[co].edge;
            if rep.faces_around_edge(e).contains(&b) {
                return Ok(e);
            }
        }
    }
    Err(EvalError::Reference(
        "the selected faces do not share an edge".into(),
    ))
}

impl FeatureEvaluator for KernelEvaluator {
    fn evaluate(
        &self,
        _node: &DagNode,
        id: NodeId,
        inputs: &[Arc<EvalOutput>],
    ) -> Result<EvalOutput, EvalError> {
        let op = self
            .ops
            .get(&id)
            .ok_or_else(|| EvalError::Kernel("no operation registered for this node".into()))?;
        // work on a copy of the first input; names of inputs are carried along by entity
        let mut work = inputs
            .first()
            .map(|i| (*i.brep).clone())
            .unwrap_or_default();
        let mut in_names: HashMap<EntityRef, TopoName> = inputs
            .first()
            .map(|i| reverse_names(&i.names))
            .unwrap_or_default();
        let mut name_map: NameMap = inputs.first().map(|i| i.names.clone()).unwrap_or_default();
        let primary = if inputs.is_empty() {
            None
        } else {
            Some(sole_body(&work)?)
        };

        let result = match op {
            FeatureOp::Extrude {
                profile,
                vector,
                symmetric,
            } => extrude(
                &mut work,
                profile,
                *vector,
                &ExtrudeOptions {
                    symmetric: *symmetric,
                },
            )
            .map_err(|e| kernel("extrude", e))?,
            FeatureOp::Revolve {
                profile,
                axis_origin,
                axis,
                angle,
            } => revolve(&mut work, profile, *axis_origin, *axis, *angle)
                .map_err(|e| kernel("revolve", e))?,
            FeatureOp::Loft { profiles } => {
                loft(&mut work, profiles).map_err(|e| kernel("loft", e))?
            }
            FeatureOp::Sweep { profile, path } => {
                sweep(&mut work, profile, path, &SweepOptions::default())
                    .map_err(|e| kernel("sweep", e))?
            }
            FeatureOp::Boolean { op } => {
                let a =
                    primary.ok_or_else(|| EvalError::Kernel("boolean needs two inputs".into()))?;
                let second = inputs
                    .get(1)
                    .ok_or_else(|| EvalError::Kernel("boolean needs two inputs".into()))?;
                let (b, map) = work.copy_body_from(
                    &second.brep,
                    sole_body(&second.brep)?,
                    &CopyOptions::default(),
                );
                for (n, e) in &second.names.by_name {
                    let mapped = match e {
                        EntityRef::Face(f) => map.faces.get(f).map(|x| EntityRef::Face(*x)),
                        EntityRef::Edge(x) => map.edges.get(x).map(|y| EntityRef::Edge(*y)),
                        EntityRef::Vertex(v) => map.vertices.get(v).map(|y| EntityRef::Vertex(*y)),
                    };
                    if let Some(m) = mapped {
                        in_names.insert(m, n.clone());
                        name_map.by_name.insert(n.clone(), m);
                    }
                }
                let r = boolean(&mut work, a, b, *op, &BoolOptions::default())
                    .map_err(|e| kernel("boolean", e))?;
                kw_brep::BuildResult {
                    body: r.body,
                    history: r.history,
                }
            }
            FeatureOp::Fillet { edges, radius } => {
                let a = primary
                    .ok_or_else(|| EvalError::Kernel("fillet needs an input body".into()))?;
                let es = resolve_edges(&work, &name_map, edges)?;
                fillet(&mut work, a, &es, *radius).map_err(|e| kernel("fillet", e))?
            }
            FeatureOp::VariableFillet { edge, law } => {
                let a = primary
                    .ok_or_else(|| EvalError::Kernel("fillet needs an input body".into()))?;
                let es = resolve_edges(&work, &name_map, std::slice::from_ref(edge))?;
                let e = *es.first().ok_or_else(|| {
                    EvalError::Kernel(
                        "variable fillet: the edge selection resolved to nothing".into(),
                    )
                })?;
                fillet_law(&mut work, a, e, law).map_err(|e| kernel("variable fillet", e))?
            }
            FeatureOp::Chamfer { edges, d1, d2 } => {
                let a = primary
                    .ok_or_else(|| EvalError::Kernel("chamfer needs an input body".into()))?;
                let es = resolve_edges(&work, &name_map, edges)?;
                chamfer(&mut work, a, &es, *d1, *d2).map_err(|e| kernel("chamfer", e))?
            }
            FeatureOp::Shell { thickness, open } => {
                let a =
                    primary.ok_or_else(|| EvalError::Kernel("shell needs an input body".into()))?;
                let fs: Vec<FaceId> = open
                    .iter()
                    .map(|n| face_named(&name_map, n))
                    .collect::<Result<_, _>>()?;
                shell(&mut work, a, *thickness, &fs).map_err(|e| kernel("shell", e))?
            }
            FeatureOp::Draft {
                faces,
                neutral_origin,
                pull,
                angle,
            } => {
                let a =
                    primary.ok_or_else(|| EvalError::Kernel("draft needs an input body".into()))?;
                let fs: Vec<FaceId> = faces
                    .iter()
                    .map(|n| face_named(&name_map, n))
                    .collect::<Result<_, _>>()?;
                draft_faces(&mut work, a, &fs, *neutral_origin, *pull, *angle)
                    .map_err(|e| kernel("draft", e))?
            }
            FeatureOp::Offset { distance } => {
                let a = primary
                    .ok_or_else(|| EvalError::Kernel("offset needs an input body".into()))?;
                offset_body(&mut work, a, *distance).map_err(|e| kernel("offset", e))?
            }
        };

        // ---- the output B-rep holds the result body only
        let mut out = BRep::new();
        let (nb, map) = out.copy_body_from(&work, result.body, &CopyOptions::default());
        let inv: HashMap<FaceId, FaceId> = map.faces.iter().map(|(o, n)| (*n, *o)).collect();
        let _ = inv;

        // ---- persistent names from the history
        let mut names = NameMap::default();
        let mut counts: HashMap<NameOrigin, u32> = HashMap::new();
        let mut new_faces: Vec<(FaceId, FaceId)> = result
            .history
            .origins
            .keys()
            .filter_map(|k| {
                if let EntityRef::Face(f) = k {
                    map.faces.get(f).map(|n| (*f, *n))
                } else {
                    None
                }
            })
            .collect();
        // deterministic order: by the order of faces in the output body
        let order: HashMap<FaceId, usize> = out
            .body_faces(nb)
            .into_iter()
            .enumerate()
            .map(|(i, f)| (f, i))
            .collect();
        new_faces.sort_by_key(|(_, n)| order.get(n).copied().unwrap_or(usize::MAX));
        for (old, new) in new_faces {
            let origin = result
                .history
                .origins
                .get(&EntityRef::Face(old))
                .cloned()
                .unwrap_or(Origin::New {
                    role: "face".into(),
                });
            let name = name_for(id, &origin, &in_names, &work, &mut counts);
            names.by_name.insert(name, EntityRef::Face(new));
        }
        let output_hash = hash_output(&out, nb, &names);
        let _ = ShapeHistory::default();
        // history re-keyed to the output B-rep
        let mut history = ShapeHistory::default();
        for (k, o) in &result.history.origins {
            if let EntityRef::Face(f) = k {
                if let Some(n) = map.faces.get(f) {
                    history.record(EntityRef::Face(*n), o.clone());
                }
            }
        }
        history.deleted_inputs = result.history.deleted_inputs.clone();
        Ok(EvalOutput {
            brep: Arc::new(out),
            names,
            history,
            output_hash,
        })
    }
}

fn resolve_edges(
    work: &BRep,
    names: &NameMap,
    sel: &[EdgeSel],
) -> Result<Vec<kw_brep::EdgeId>, EvalError> {
    sel.iter()
        .map(|EdgeSel(a, b)| {
            let (fa, fb) = (face_named(names, a)?, face_named(names, b)?);
            edge_between(work, fa, fb)
        })
        .collect()
}

fn name_for(
    feature: NodeId,
    origin: &Origin,
    in_names: &HashMap<EntityRef, TopoName>,
    work: &BRep,
    counts: &mut HashMap<NameOrigin, u32>,
) -> TopoName {
    let mut unique = |o: NameOrigin| -> TopoName {
        let c = counts.entry(o.clone()).or_insert(0);
        let t = TopoName {
            feature,
            origin: o,
            ordinal: *c,
        };
        *c += 1;
        t
    };
    match origin {
        Origin::Preserved(src) | Origin::Modified(src) => match in_names.get(src) {
            Some(n) => n.clone(),
            None => unique(NameOrigin::Primitive {
                role: "unnamed-input".into(),
            }),
        },
        Origin::New { role } => unique(NameOrigin::Primitive {
            role: role.to_string(),
        }),
        Origin::Generated { from, role } => {
            // an edge is named through its adjacent faces
            let adj: Vec<TopoName> = match from {
                EntityRef::Edge(e) => {
                    let mut v: Vec<TopoName> = work
                        .faces_around_edge(*e)
                        .into_iter()
                        .filter_map(|f| in_names.get(&EntityRef::Face(f)).cloned())
                        .collect();
                    v.sort_by_key(key);
                    v
                }
                other => in_names.get(other).cloned().into_iter().collect(),
            };
            match adj.split_first() {
                Some((first, rest)) => {
                    let tail: Vec<String> = rest.iter().map(key).collect();
                    unique(NameOrigin::Generated {
                        from: Box::new(first.clone()),
                        role: format!("{role}|{}", tail.join("|")),
                    })
                }
                None => unique(NameOrigin::Primitive {
                    role: role.to_string(),
                }),
            }
        }
        Origin::SplitFrom { parent, ordinal } => match in_names.get(parent) {
            Some(p) => unique(NameOrigin::Split {
                parent: Box::new(p.clone()),
                ordinal: *ordinal,
            }),
            None => unique(NameOrigin::Primitive {
                role: format!("split-{ordinal}"),
            }),
        },
        Origin::MergedFrom(v) => {
            let mut ns: Vec<TopoName> = v.iter().filter_map(|r| in_names.get(r).cloned()).collect();
            ns.sort_by_key(key);
            unique(NameOrigin::Merge(ns))
        }
    }
}

/// FNV-1a over geometry summary and names (deterministic across runs and thread counts).
fn hash_output(rep: &BRep, body: BodyId, names: &NameMap) -> u128 {
    let mut h: u128 = 0x6c62272e07bb014262b821756295c58d;
    let mut feed = |bytes: &[u8]| {
        for b in bytes {
            h ^= *b as u128;
            h = h.wrapping_mul(0x0000000001000000000000000000013b);
        }
    };
    feed(&(rep.body_faces(body).len() as u64).to_le_bytes());
    feed(&rep.volume(body).unwrap_or(0.0).to_bits().to_le_bytes());
    feed(&rep.area(body).unwrap_or(0.0).to_bits().to_le_bytes());
    let mut ks: Vec<String> = names.by_name.keys().map(key).collect();
    ks.sort();
    for k in ks {
        feed(k.as_bytes());
    }
    h
}

/// Turn the closed curves of a sketch into a planar [`Profile`]: lines, arcs and circles that are not
/// construction geometry are chained into loops; the loop with the largest area is the outer one.
pub fn sketch_profile(sk: &Sketch, plane: Frame3) -> Result<Profile, EvalError> {
    use kw_sketch::geom::arc_params;
    let map = |p: (f64, f64)| plane.origin + plane.x * p.0 + plane.y * p.1;
    struct Seg {
        a: (f64, f64),
        b: (f64, f64),
        edge: ProfileEdge,
    }
    let mut segs: Vec<Seg> = Vec::new();
    let mut circles: Vec<ProfileLoop> = Vec::new();
    for (id, e) in &sk.entities {
        if e.construction {
            continue;
        }
        match &e.kind {
            EntityKind::Line { p1, p2 } => {
                let (a, b) = (
                    sk.point(*p1).map_err(|e| kernel("sketch", e))?,
                    sk.point(*p2).map_err(|e| kernel("sketch", e))?,
                );
                segs.push(Seg {
                    a,
                    b,
                    edge: ProfileEdge::line(map(a), map(b)),
                });
            }
            EntityKind::Arc { .. } => {
                let (c, r, a0, a1) = arc_params(sk, id).map_err(|e| kernel("sketch", e))?;
                let frame = Frame3 {
                    origin: map(c),
                    ..plane
                };
                let (pa, pb) = (
                    (c.0 + r * a0.cos(), c.1 + r * a0.sin()),
                    (c.0 + r * a1.cos(), c.1 + r * a1.sin()),
                );
                segs.push(Seg {
                    a: pa,
                    b: pb,
                    edge: ProfileEdge::arc(&frame, r, a0, a1),
                });
            }
            EntityKind::Circle { center, radius } => {
                let c = sk.point(*center).map_err(|e| kernel("sketch", e))?;
                circles.push(ProfileLoop::circle(
                    &Frame3 {
                        origin: map(c),
                        ..plane
                    },
                    sk.value(*radius),
                ));
            }
            // points are the end points of curves (or free points, which carry no area)
            EntityKind::Point(_) => {}
            _ => {
                return Err(EvalError::Kernel(
                    "only lines, arcs and circles can form a profile".into(),
                ))
            }
        }
    }
    let tol = 1e-7;
    let near = |p: (f64, f64), q: (f64, f64)| (p.0 - q.0).hypot(p.1 - q.1) <= tol;
    let mut loops: Vec<ProfileLoop> = Vec::new();
    while let Some(first) = segs.pop() {
        let mut chain = vec![first.edge];
        let mut end = first.b;
        let start = first.a;
        while !near(end, start) {
            let Some(i) = segs.iter().position(|s| near(s.a, end) || near(s.b, end)) else {
                return Err(EvalError::Kernel("sketch profile is not closed".into()));
            };
            let s = segs.remove(i);
            if near(s.a, end) {
                end = s.b;
                chain.push(s.edge);
            } else {
                let mut e = s.edge;
                e.reversed = true;
                end = s.a;
                chain.push(e);
            }
        }
        loops.push(ProfileLoop::new(chain));
    }
    loops.extend(circles);
    if loops.is_empty() {
        return Err(EvalError::Kernel("the sketch has no closed profile".into()));
    }
    // area of each loop about the plane normal; outer = largest, oriented CCW; holes CW
    let area = |l: &ProfileLoop| l.check(&plane.z, 1e-6).unwrap_or(0.0);
    let mut idx: Vec<usize> = (0..loops.len()).collect();
    idx.sort_by(|a, b| {
        area(&loops[*b])
            .abs()
            .partial_cmp(&area(&loops[*a]).abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let reverse = |l: &ProfileLoop| -> ProfileLoop {
        let edges: Vec<ProfileEdge> = l
            .edges
            .iter()
            .rev()
            .map(|e| ProfileEdge {
                reversed: !e.reversed,
                ..e.clone()
            })
            .collect();
        ProfileLoop::new(edges)
    };
    let mut ordered: Vec<ProfileLoop> = Vec::new();
    for (k, i) in idx.into_iter().enumerate() {
        let l = &loops[i];
        let ccw = area(l) > 0.0;
        // a full circle's orientation comes from its frame; reversing a single circle edge is not representable, so
        // rebuild it mirrored
        let want_ccw = k == 0;
        if ccw == want_ccw {
            ordered.push(l.clone());
        } else if l.edges.len() == 1 {
            if let CurveGeom::Circle(c) = &l.edges[0].curve {
                let f = Frame3 {
                    origin: c.frame.origin,
                    x: c.frame.x,
                    y: -c.frame.y,
                    z: -c.frame.z,
                };
                ordered.push(ProfileLoop::circle(&f, c.radius));
            } else {
                ordered.push(reverse(l));
            }
        } else {
            ordered.push(reverse(l));
        }
    }
    let mut it = ordered.into_iter();
    let mut p = Profile::new(plane, it.next().unwrap());
    p.holes = it.collect();
    Ok(p)
}
