//! Sketch → planar [`Profile`] through the planar arrangement of the sketch, remembering which sketch entity
//! every profile edge came from.
//!
//! The curves of a sketch are cut where they cross or touch (`kw_sketch::arrangement`); the faces of that
//! arrangement are the regions. A profile is one face (its outline and its holes) or the outline of several
//! faces merged. Free-standing open curves belong to no face and are ignored.

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
