//! Sketch → planar [`Profile`] through the planar arrangement of the sketch, remembering which sketch entity
//! every profile edge came from.
//!
//! The curves of a sketch are cut where they cross or touch (`kw_sketch::arrangement`); the faces of that
//! arrangement are the regions. A profile is one face (its outline and its holes) or the outline of several
//! faces merged. Free-standing open curves belong to no face; [`sketch_wires`] returns them (and the closed
//! outlines) as ordered chains of edges, for the surface features.

use crate::eval::EvalError;
use kw_math::Frame3;
use kw_model::{Profile, ProfileEdge, ProfileLoop};
use kw_nurbs::geom::Ellipse;
use kw_nurbs::{CurveGeom, NurbsCurve};
use kw_sketch::arrangement::{Arrangement, Half, Prim};
use kw_sketch::{EntityId, Sketch};
use std::collections::HashMap;

/// A profile together with, for every loop and edge, the label of the sketch entity behind it.
#[derive(Debug, Clone)]
pub struct LabeledProfile {
    pub profile: Profile,
    /// `labels[loop][edge]`; loop 0 is the outer loop, the rest are holes, in profile order.
    pub labels: Vec<Vec<u32>>,
}

/// An ordered chain of sketch curves on the sketch plane: each edge starts where the one before ends. Open
/// chains are the curves a surface is swept or lofted from; a closed chain is an outline.
#[derive(Debug, Clone)]
pub struct Wire {
    pub plane: Frame3,
    pub edges: Vec<ProfileEdge>,
    /// The label of the sketch entity behind each edge.
    pub labels: Vec<u32>,
    pub closed: bool,
}

impl Wire {
    pub fn start(&self) -> kw_math::Point3<f64> {
        self.edges[0].start()
    }
    pub fn end(&self) -> kw_math::Point3<f64> {
        self.edges[self.edges.len() - 1].end()
    }
}

/// The arrangement of a sketch on its plane, with what is needed to turn faces into kernel profiles.
pub struct FaceSet {
    pub arr: Arrangement,
    pub plane: Frame3,
    labels: HashMap<EntityId, u32>,
    /// The splines of the arrangement lifted onto the plane (by curve index).
    splines: HashMap<usize, NurbsCurve>,
}

/// Build the faces of a sketch. `entities[i]` is the entity with label `i`.
pub fn sketch_faces(sk: &Sketch, entities: &[EntityId], plane: Frame3) -> FaceSet {
    let arr = Arrangement::build(sk);
    let labels: HashMap<EntityId, u32> = entities
        .iter()
        .enumerate()
        .map(|(i, e)| (*e, i as u32))
        .collect();
    let map = |p: (f64, f64)| plane.origin + plane.x * p.0 + plane.y * p.1;
    let mut splines = HashMap::new();
    for (i, c) in arr.curves.iter().enumerate() {
        if let Prim::Spline(flat) = &c.prim {
            let pts: Vec<_> = flat.points().iter().map(|p| map((p.x, p.y))).collect();
            if let Ok(curve) = NurbsCurve::new(flat.knots().clone(), pts, flat.weights().to_vec()) {
                splines.insert(i, curve);
            }
        }
    }
    FaceSet {
        arr,
        plane,
        labels,
        splines,
    }
}

impl FaceSet {
    /// The label of the entity a piece belongs to (the smallest when several overlap).
    pub fn label_of(&self, sources: &[EntityId]) -> u32 {
        sources
            .iter()
            .map(|e| self.labels.get(e).copied().unwrap_or(u32::MAX))
            .min()
            .unwrap_or(u32::MAX)
    }

    /// The sorted, distinct labels of every entity on the boundary of a face.
    pub fn face_labels(&self, face: usize) -> Vec<u32> {
        let mut v: Vec<u32> = self.arr.faces[face]
            .sources
            .iter()
            .map(|e| self.labels.get(e).copied().unwrap_or(u32::MAX))
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// The faces that are extruded when none is chosen: those not enclosed by an odd number of other
    /// shapes (a plate with a hole is the plate; what lies inside the hole is another region).
    pub fn default_faces(&self) -> Vec<usize> {
        (0..self.arr.faces.len())
            .filter(|f| self.arr.faces[*f].depth.is_multiple_of(2))
            .collect()
    }

    fn edge(&self, h: Half) -> ProfileEdge {
        let e = &self.arr.edges[h.edge];
        let c = &self.arr.curves[e.curve];
        let plane = &self.plane;
        let map = |p: (f64, f64)| plane.origin + plane.x * p.0 + plane.y * p.1;
        let mut edge = match &c.prim {
            Prim::Line { .. } => {
                let (a, b) = (c.prim.point(e.t0), c.prim.point(e.t1));
                return if h.forward {
                    ProfileEdge::line(map(a), map(b))
                } else {
                    ProfileEdge::line(map(b), map(a))
                };
            }
            Prim::Arc { c: ctr, r, .. } | Prim::Circle { c: ctr, r } => {
                let frame = Frame3 {
                    origin: map(*ctr),
                    ..*plane
                };
                ProfileEdge::arc(&frame, *r, e.t0, e.t1)
            }
            Prim::Ellipse {
                c: ctr,
                u,
                major,
                minor,
            } => {
                let x = plane.x * u.0 + plane.y * u.1;
                let frame = Frame3 {
                    origin: map(*ctr),
                    x,
                    y: plane.z.cross(&x),
                    z: plane.z,
                };
                ProfileEdge::new(
                    CurveGeom::Ellipse(Ellipse {
                        frame,
                        major: *major,
                        minor: *minor,
                    }),
                    (e.t0, e.t1),
                )
            }
            Prim::Spline(_) => match self.splines.get(&e.curve) {
                Some(curve) => ProfileEdge::new(CurveGeom::Nurbs(curve.clone()), (e.t0, e.t1)),
                None => ProfileEdge::line(map(c.prim.point(e.t0)), map(c.prim.point(e.t1))),
            },
        };
        edge.reversed = !h.forward;
        edge
    }

    /// Every curve of the sketch as chains of consecutive edges: from each free end (or from a place where
    /// three or more curves meet) up to the next such place; whatever is left are closed outlines. Chains
    /// are ordered by their smallest entity label and start at the end that makes their first edge run
    /// along its curve, so the names made from them do not depend on the order the curves were found in.
    pub fn wires(&self) -> Vec<Wire> {
        let n = self.arr.vertices.len();
        let mut around: Vec<Vec<Half>> = vec![Vec::new(); n];
        for (i, e) in self.arr.edges.iter().enumerate() {
            if e.from == e.to {
                continue; // a closed curve on its own is handled below
            }
            around[e.from].push(Half {
                edge: i,
                forward: true,
            });
            around[e.to].push(Half {
                edge: i,
                forward: false,
            });
        }
        let end_of = |h: Half| {
            let e = &self.arr.edges[h.edge];
            if h.forward {
                e.to
            } else {
                e.from
            }
        };
        let mut used = vec![false; self.arr.edges.len()];
        let mut chains: Vec<(Vec<Half>, bool)> = Vec::new();
        let walk = |start: Half, used: &mut Vec<bool>| -> (Vec<Half>, bool) {
            let mut halves = vec![start];
            used[start.edge] = true;
            let first_from = if start.forward {
                self.arr.edges[start.edge].from
            } else {
                self.arr.edges[start.edge].to
            };
            let mut at = end_of(start);
            loop {
                if at == first_from {
                    return (halves, true);
                }
                if around[at].len() != 2 {
                    return (halves, false);
                }
                let Some(next) = around[at].iter().copied().find(|h| !used[h.edge]) else {
                    return (halves, false);
                };
                used[next.edge] = true;
                halves.push(next);
                at = end_of(next);
            }
        };
        // open ends and junctions first
        for v in 0..n {
            if around[v].len() == 2 {
                continue;
            }
            for h in around[v].clone() {
                if !used[h.edge] {
                    let (c, closed) = walk(h, &mut used);
                    chains.push((c, closed));
                }
            }
        }
        // then outlines (every vertex of degree two)
        for (i, e) in self.arr.edges.iter().enumerate() {
            if used[i] {
                continue;
            }
            let (c, closed) = if e.from == e.to {
                used[i] = true;
                (
                    vec![Half {
                        edge: i,
                        forward: true,
                    }],
                    true,
                )
            } else {
                walk(
                    Half {
                        edge: i,
                        forward: true,
                    },
                    &mut used,
                )
            };
            chains.push((c, closed));
        }
        let mut wires: Vec<(u32, Wire)> = chains
            .into_iter()
            .map(|(halves, closed)| {
                let rev: Vec<Half> = halves
                    .iter()
                    .rev()
                    .map(|h| Half {
                        edge: h.edge,
                        forward: !h.forward,
                    })
                    .collect();
                // the orientation whose first edge has the smaller label, then the one that runs along its curve
                let key = |hs: &[Half]| {
                    let h = hs[0];
                    let e = &self.arr.edges[h.edge];
                    (self.label_of(&e.sources), !h.forward, e.t0.to_bits())
                };
                let halves = if closed || key(&halves) <= key(&rev) {
                    halves
                } else {
                    rev
                };
                let labels: Vec<u32> = halves
                    .iter()
                    .map(|h| self.label_of(&self.arr.edges[h.edge].sources))
                    .collect();
                let first = labels.iter().copied().min().unwrap_or(u32::MAX);
                // a closed outline starts at its smallest label like a profile loop does
                let (halves, labels) = if closed {
                    let (l, lab) = self.loop_of(&halves);
                    return (
                        first,
                        Wire {
                            plane: self.plane,
                            edges: l.edges,
                            labels: lab,
                            closed,
                        },
                    );
                } else {
                    (halves, labels)
                };
                (
                    first,
                    Wire {
                        plane: self.plane,
                        edges: halves.iter().map(|h| self.edge(*h)).collect(),
                        labels,
                        closed,
                    },
                )
            })
            .collect();
        wires.sort_by_key(|(first, w)| (*first, w.edges.len()));
        wires.into_iter().map(|(_, w)| w).collect()
    }

    /// A loop of half-edges as a kernel loop with the label of every edge. The loop starts at its edge with
    /// the smallest label (then the smallest parameter), so that the names of the faces made from it do not
    /// depend on where the traversal happened to begin.
    pub fn loop_of(&self, halves: &[Half]) -> (ProfileLoop, Vec<u32>) {
        let key = |h: &Half| {
            let e = &self.arr.edges[h.edge];
            (self.label_of(&e.sources), e.t0.to_bits(), h.forward)
        };
        let start = (0..halves.len())
            .min_by_key(|i| key(&halves[*i]))
            .unwrap_or(0);
        let rotated: Vec<Half> = halves[start..]
            .iter()
            .chain(halves[..start].iter())
            .copied()
            .collect();
        let labels: Vec<u32> = rotated
            .iter()
            .map(|h| self.label_of(&self.arr.edges[h.edge].sources))
            .collect();
        // a closed curve on its own, run backwards: its orientation comes from its frame
        if let [h] = rotated.as_slice() {
            if !h.forward {
                let e = &self.arr.edges[h.edge];
                let prim = &self.arr.curves[e.curve].prim;
                if let Some(l) = self.mirrored_closed(prim) {
                    return (l, labels);
                }
            }
        }
        (
            ProfileLoop::new(rotated.iter().map(|h| self.edge(*h)).collect()),
            labels,
        )
    }

    fn mirrored_closed(&self, prim: &Prim) -> Option<ProfileLoop> {
        let plane = &self.plane;
        let map = |p: (f64, f64)| plane.origin + plane.x * p.0 + plane.y * p.1;
        match prim {
            Prim::Circle { c, r } => {
                let f = Frame3 {
                    origin: map(*c),
                    x: plane.x,
                    y: -plane.y,
                    z: -plane.z,
                };
                Some(ProfileLoop::circle(&f, *r))
            }
            Prim::Ellipse { c, u, major, minor } => {
                let x = plane.x * u.0 + plane.y * u.1;
                let f = Frame3 {
                    origin: map(*c),
                    x,
                    y: -(plane.z.cross(&x)),
                    z: -plane.z,
                };
                Some(ProfileLoop::new(vec![ProfileEdge::new(
                    CurveGeom::Ellipse(Ellipse {
                        frame: f,
                        major: *major,
                        minor: *minor,
                    }),
                    (0.0, std::f64::consts::TAU),
                )]))
            }
            _ => None,
        }
    }

    /// The profile of an outline and its holes.
    pub fn profile_of(&self, outer: &[Half], holes: &[Vec<Half>]) -> LabeledProfile {
        let (o, ol) = self.loop_of(outer);
        let mut profile = Profile::new(self.plane, o);
        let mut labels = vec![ol];
        for h in holes {
            let (l, lab) = self.loop_of(h);
            profile.holes.push(l);
            labels.push(lab);
        }
        LabeledProfile { profile, labels }
    }
}

/// The profile of a sketch: its largest region that is not inside another shape, with the shapes directly
/// inside it as holes. (Revolve, Loft and Sweep use this; Extrude chooses its regions.)
pub fn sketch_profile_labeled(
    sk: &Sketch,
    entities: &[EntityId],
    plane: Frame3,
) -> Result<LabeledProfile, EvalError> {
    let fs = sketch_faces(sk, entities, plane);
    let best = (0..fs.arr.faces.len())
        .filter(|f| fs.arr.faces[*f].depth == 0)
        .max_by(|a, b| {
            fs.arr.faces[*a]
                .area
                .total_cmp(&fs.arr.faces[*b].area)
                .then(b.cmp(a))
        })
        .ok_or_else(|| EvalError::Sketch("the sketch has no closed profile".into()))?;
    let f = &fs.arr.faces[best];
    Ok(fs.profile_of(&f.outer, &f.holes))
}

/// As [`sketch_profile_labeled`] with entity labels taken from the sketch's slot order.
pub fn sketch_profile(sk: &Sketch, plane: Frame3) -> Result<Profile, EvalError> {
    let ents: Vec<EntityId> = sk.entities.keys().collect();
    Ok(sketch_profile_labeled(sk, &ents, plane)?.profile)
}

/// The chains of curves of a sketch: open ones (what a surface is made from) and closed ones.
pub fn sketch_wires(sk: &Sketch, entities: &[EntityId], plane: Frame3) -> Vec<Wire> {
    sketch_faces(sk, entities, plane).wires()
}
