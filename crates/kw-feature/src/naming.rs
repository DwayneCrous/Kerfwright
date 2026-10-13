//! Persistent topological names, geometric fingerprints and reference resolution.
//!
//! A [`TopoName`] says *how a face came to be* (which sketch entity it came from, which face it was
//! split from, which faces were merged into it, which pattern instance it is), not where it is, so it
//! survives dimension changes and upstream edits. When a name no longer exists, resolution walks the
//! lineage (descendants of the named face), and only then falls back to a geometric
//! [`GeomFingerprint`] match that must beat a threshold *and* the runner-up by a margin. A reference that
//! cannot be resolved with that confidence is reported as [`ResolveOutcome::Ambiguous`] or
//! [`ResolveOutcome::Broken`]; it is never silently guessed.

use crate::id::{FeatureId, Hasher128};
use kw_brep::{BRep, EntityRef, FaceId};
use kw_nurbs::{Surface, SurfaceGeom};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum NameOrigin {
    /// A face swept from entity `entity` of the sketch feature `sketch` (`role`: "side").
    SketchEdge {
        sketch: FeatureId,
        entity: u32,
        role: String,
    },
    /// A face the feature creates by itself (caps, primitive faces).
    Primitive { role: String },
    /// Created from an existing face or from the faces around an edge (fillet and chamfer faces).
    Generated { from: Box<TopoName>, role: String },
    /// The `ordinal`-th fragment of a split face.
    Split { parent: Box<TopoName>, ordinal: u32 },
    /// Several faces merged into one.
    Merge(Vec<TopoName>),
    /// The `index`-th copy of a face made by a pattern or mirror.
    Instance { source: Box<TopoName>, index: u32 },
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct TopoName {
    /// The feature that created the name.
    pub feature: FeatureId,
    pub origin: NameOrigin,
    /// Disambiguates names with an identical origin.
    pub ordinal: u32,
}

impl TopoName {
    pub fn primitive(feature: FeatureId, role: &str, ordinal: u32) -> TopoName {
        TopoName {
            feature,
            origin: NameOrigin::Primitive { role: role.into() },
            ordinal,
        }
    }

    fn write(&self, h: &mut Hasher128) {
        h.u64(self.feature.0);
        match &self.origin {
            NameOrigin::SketchEdge {
                sketch,
                entity,
                role,
            } => {
                h.u64(1).u64(sketch.0).u64(*entity as u64).str(role);
            }
            NameOrigin::Primitive { role } => {
                h.u64(2).str(role);
            }
            NameOrigin::Generated { from, role } => {
                h.u64(3).str(role);
                from.write(h);
            }
            NameOrigin::Split { parent, ordinal } => {
                h.u64(4).u64(*ordinal as u64);
                parent.write(h);
            }
            NameOrigin::Merge(v) => {
                h.u64(5).u64(v.len() as u64);
                for n in v {
                    n.write(h);
                }
            }
            NameOrigin::Instance { source, index } => {
                h.u64(6).u64(*index as u64);
                source.write(h);
            }
        }
        h.u64(self.ordinal as u64);
    }

    /// A 128-bit hash that depends only on the name's content (never on memory addresses, graph
    /// indices or formatting), identical across runs, machines and thread counts.
    pub fn stable_hash(&self) -> u128 {
        let mut h = Hasher128::new();
        self.write(&mut h);
        h.finish()
    }

    /// Direct ancestors named inside this name.
    pub fn ancestors(&self) -> Vec<&TopoName> {
        match &self.origin {
            NameOrigin::Generated { from, .. } => vec![from],
            NameOrigin::Split { parent, .. } => vec![parent],
            NameOrigin::Merge(v) => v.iter().collect(),
            NameOrigin::Instance { source, .. } => vec![source],
            _ => Vec::new(),
        }
    }

    /// Does this name descend (through split, merge, generation or instancing) from `other`?
    pub fn derives_from(&self, other: &TopoName) -> bool {
        self.ancestors()
            .into_iter()
            .any(|a| a == other || a.derives_from(other))
    }

    /// Is the face with this name a piece of the face called `other` (it was split from it, or merged
    /// with it)? A fillet face generated from `other`, or an instance copied from it, is not.
    pub fn is_fragment_of(&self, other: &TopoName) -> bool {
        match &self.origin {
            NameOrigin::Split { parent, .. } => **parent == *other || parent.is_fragment_of(other),
            NameOrigin::Merge(v) => v.iter().any(|n| n == other || n.is_fragment_of(other)),
            _ => false,
        }
    }
}

/// Names of the faces of one body.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NameMap {
    pub by_name: BTreeMap<TopoName, EntityRef>,
}

impl NameMap {
    pub fn face(&self, name: &TopoName) -> Option<FaceId> {
        match self.by_name.get(name) {
            Some(EntityRef::Face(f)) => Some(*f),
            _ => None,
        }
    }

    pub fn reverse(&self) -> HashMap<EntityRef, TopoName> {
        self.by_name.iter().map(|(n, e)| (*e, n.clone())).collect()
    }

    /// Hash of the set of names (order-independent by construction: the map is ordered).
    pub fn set_hash(&self) -> u128 {
        let mut h = Hasher128::new();
        h.u64(self.by_name.len() as u64);
        for n in self.by_name.keys() {
            h.u128(n.stable_hash());
        }
        h.finish()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfKind {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Nurbs,
    Extrusion,
    Revolution,
    Other,
}

/// Geometry of a face, compared when a name cannot be found.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeomFingerprint {
    pub kind: SurfKind,
    /// Outward normal (planes) or axis direction with a canonical sign (cylinders, cones, tori).
    pub dir: Option<[f64; 3]>,
    /// Centre of the face's bounding box.
    pub center: [f64; 3],
    /// Diagonal of the face's bounding box.
    pub extent: f64,
    /// Radius of cylinders, cones (at the base), spheres, tori (major); 0 otherwise.
    pub radius: f64,
    /// Hash of the kinds and directions of the neighbouring faces.
    pub adjacency: u64,
    /// Surface area (0 when unknown, as in files written before it was recorded).
    #[serde(default)]
    pub area: f64,
}

fn canonical_axis(v: [f64; 3]) -> [f64; 3] {
    let flip = v
        .iter()
        .find(|c| c.abs() > 1e-9)
        .map(|c| *c < 0.0)
        .unwrap_or(false);
    if flip {
        [-v[0], -v[1], -v[2]]
    } else {
        v
    }
}

fn surf_summary(rep: &BRep, f: FaceId) -> (SurfKind, Option<[f64; 3]>, f64) {
    let face = &rep.faces[f];
    let s = &rep.surfaces[face.surface];
    let sign = if face.sense == kw_brep::Sense::Forward {
        1.0
    } else {
        -1.0
    };
    let arr = |v: kw_math::Vector3<f64>| [v.x, v.y, v.z];
    match s {
        SurfaceGeom::Plane(p) => (SurfKind::Plane, Some(arr(p.frame.z * sign)), 0.0),
        SurfaceGeom::Cylinder(c) => (
            SurfKind::Cylinder,
            Some(canonical_axis(arr(c.frame.z))),
            c.radius,
        ),
        SurfaceGeom::Cone(c) => (
            SurfKind::Cone,
            Some(canonical_axis(arr(c.frame.z))),
            c.radius_at_origin,
        ),
        SurfaceGeom::Sphere(c) => (SurfKind::Sphere, None, c.radius),
        SurfaceGeom::Torus(t) => (
            SurfKind::Torus,
            Some(canonical_axis(arr(t.frame.z))),
            t.major_radius,
        ),
        SurfaceGeom::Nurbs(_) => (SurfKind::Nurbs, None, 0.0),
        SurfaceGeom::Extrusion { .. } => (SurfKind::Extrusion, None, 0.0),
        SurfaceGeom::Revolution { .. } => (SurfKind::Revolution, None, 0.0),
        SurfaceGeom::Procedural(_) => (SurfKind::Other, None, 0.0),
    }
}

impl GeomFingerprint {
    pub fn of(rep: &BRep, f: FaceId) -> GeomFingerprint {
        let (kind, dir, radius) = surf_summary(rep, f);
        let bb = rep.face_bbox(f).unwrap_or(kw_math::Aabb::EMPTY);
        let c = bb.center();
        let extent = if bb.min.x <= bb.max.x {
            bb.diagonal()
        } else {
            0.0
        };
        // neighbours
        let mut neigh: Vec<(u8, [i32; 3])> = Vec::new();
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                for g in rep.faces_around_edge(e) {
                    if g != f {
                        let (k, d, _) = surf_summary(rep, g);
                        let q = d
                            .map(|d| {
                                [
                                    (d[0] * 10.0).round() as i32,
                                    (d[1] * 10.0).round() as i32,
                                    (d[2] * 10.0).round() as i32,
                                ]
                            })
                            .unwrap_or([0; 3]);
                        neigh.push((k as u8, q));
                    }
                }
            }
        }
        neigh.sort_unstable();
        neigh.dedup();
        let mut h = Hasher128::new();
        for (k, q) in &neigh {
            h.u64(*k as u64);
            for c in q {
                h.u64(*c as i64 as u64);
            }
        }
        GeomFingerprint {
            kind,
            dir,
            center: [c.x, c.y, c.z],
            extent,
            radius,
            adjacency: h.finish() as u64,
            area: rep.face_area(f).unwrap_or(0.0),
        }
    }

    /// Distance-like measure to `other` (0 = identical); `None` if the surface kinds differ.
    pub fn distance(&self, other: &GeomFingerprint, scale: f64) -> Option<f64> {
        if self.kind != other.kind {
            return None;
        }
        let mut d = 0.0;
        if let (Some(a), Some(b)) = (self.dir, other.dir) {
            let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            let dot = if self.kind == SurfKind::Plane {
                dot
            } else {
                dot.abs()
            };
            d += dot.clamp(-1.0, 1.0).acos() / 0.05;
        }
        let rmax = self.radius.max(other.radius);
        if rmax > 0.0 {
            d += (self.radius - other.radius).abs() / (0.05 * rmax + 1e-9 * scale);
        }
        let dc = ((self.center[0] - other.center[0]).powi(2)
            + (self.center[1] - other.center[1]).powi(2)
            + (self.center[2] - other.center[2]).powi(2))
        .sqrt();
        // displacement is measured against the size of the face itself: a face that moved by more
        // than a fraction of its own size is not the same face, however small the model is next to it
        let emax = self.extent.max(other.extent).max(1e-9 * scale);
        d += dc / (0.3 * emax);
        d += (self.extent - other.extent).abs() / (0.3 * emax);
        if self.adjacency != other.adjacency {
            d += 0.7;
        }
        Some(d)
    }

    pub fn score(&self, other: &GeomFingerprint, scale: f64) -> f64 {
        match self.distance(other, scale) {
            Some(d) => 1.0 / (1.0 + d),
            None => 0.0,
        }
    }
}

/// A persistent reference to a face: its name plus what it looked like when the reference was made.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Reference {
    pub name: TopoName,
    pub fingerprint: GeomFingerprint,
    /// Size of the model when the reference was made (normalises the fingerprint distances).
    pub scale: f64,
}

/// Minimum score for a fingerprint match and the margin it must have over the runner-up.
pub const MATCH_SCORE: f64 = 0.45;
pub const MATCH_MARGIN: f64 = 0.15;
/// When a face is split, a piece with at least this many times the area of every other piece keeps the
/// identity; with pieces closer in size the reference is ambiguous.
pub const DOMINANT_AREA: f64 = 1.5;

#[derive(Debug, Clone, PartialEq)]
pub enum ResolveOutcome {
    /// The name exists.
    Exact(EntityRef),
    /// The name is gone; these faces descend from it (it was split, merged, or generated onward).
    Descendants(Vec<EntityRef>),
    /// Found by geometry, with a score above the threshold and a margin over the runner-up.
    Matched { entity: EntityRef, score: f64 },
    /// More than one plausible face; the user must choose.
    Ambiguous { candidates: Vec<(EntityRef, f64)> },
    /// Nothing plausible.
    Broken,
}

/// How a reference resolved, for callers that need one face.
#[derive(Debug, Clone, PartialEq)]
pub enum Resolved {
    Face(FaceId, ResolveKind),
    Ambiguous(Vec<(FaceId, f64)>),
    Broken,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResolveKind {
    Exact,
    Lineage,
    Fingerprint,
}

impl NameMap {
    /// Resolution order: the name itself, its descendants, then geometry.
    pub fn resolve(&self, rep: &BRep, r: &Reference) -> ResolveOutcome {
        if let Some(e) = self.by_name.get(&r.name) {
            return ResolveOutcome::Exact(*e);
        }
        let desc: Vec<EntityRef> = self
            .by_name
            .iter()
            .filter(|(n, _)| n.is_fragment_of(&r.name))
            .map(|(_, e)| *e)
            .collect();
        if !desc.is_empty() {
            return ResolveOutcome::Descendants(desc);
        }
        // the face was a fragment of a face that exists whole again (the cut that split it is gone)
        let mut cur = &r.name;
        while let NameOrigin::Split { parent, .. } = &cur.origin {
            if let Some(e) = self.by_name.get(parent.as_ref()) {
                return ResolveOutcome::Descendants(vec![*e]);
            }
            cur = parent;
        }
        self.by_geometry(
            rep,
            r,
            self.by_name.values().copied().collect(),
            MATCH_SCORE,
            MATCH_MARGIN,
        )
    }

    fn by_geometry(
        &self,
        rep: &BRep,
        r: &Reference,
        pool: Vec<EntityRef>,
        min_score: f64,
        margin: f64,
    ) -> ResolveOutcome {
        let mut scored: Vec<(EntityRef, f64)> = pool
            .into_iter()
            .filter_map(|e| match e {
                EntityRef::Face(f) => {
                    let s = r.fingerprint.score(&GeomFingerprint::of(rep, f), r.scale);
                    (s > 0.0).then_some((e, s))
                }
                _ => None,
            })
            .collect();
        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        match scored.as_slice() {
            [] => ResolveOutcome::Broken,
            [(e, s)] => {
                if *s >= min_score {
                    ResolveOutcome::Matched {
                        entity: *e,
                        score: *s,
                    }
                } else {
                    ResolveOutcome::Broken
                }
            }
            [(e, s), (_, s2), ..] => {
                if *s >= min_score && *s - *s2 >= margin {
                    ResolveOutcome::Matched {
                        entity: *e,
                        score: *s,
                    }
                } else if *s >= min_score * 0.6 {
                    ResolveOutcome::Ambiguous {
                        candidates: scored.into_iter().take(4).collect(),
                    }
                } else {
                    ResolveOutcome::Broken
                }
            }
        }
    }

    /// Resolve to one face; several descendants are told apart by geometry (with the same margin rule).
    pub fn resolve_face(&self, rep: &BRep, r: &Reference) -> Resolved {
        match self.resolve(rep, r) {
            ResolveOutcome::Exact(EntityRef::Face(f)) => Resolved::Face(f, ResolveKind::Exact),
            ResolveOutcome::Descendants(v) => {
                if let [EntityRef::Face(f)] = v.as_slice() {
                    return Resolved::Face(*f, ResolveKind::Lineage);
                }
                // a face split into pieces: the reference follows a piece that clearly dominates in area
                let mut areas: Vec<(FaceId, f64)> = v
                    .iter()
                    .filter_map(|e| match e {
                        EntityRef::Face(f) => Some((*f, rep.face_area(*f).unwrap_or(0.0))),
                        _ => None,
                    })
                    .collect();
                areas.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
                if let [(f, a), (_, b), ..] = areas.as_slice() {
                    if *a >= DOMINANT_AREA * *b && *a > 0.0 {
                        return Resolved::Face(*f, ResolveKind::Lineage);
                    }
                }
                // otherwise the pieces are told apart by the user, never guessed
                let mut scored: Vec<(FaceId, f64)> = v
                    .iter()
                    .filter_map(|e| match e {
                        EntityRef::Face(f) => Some((
                            *f,
                            r.fingerprint.score(&GeomFingerprint::of(rep, *f), r.scale),
                        )),
                        _ => None,
                    })
                    .collect();
                scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
                Resolved::Ambiguous(scored)
            }
            ResolveOutcome::Matched {
                entity: EntityRef::Face(f),
                ..
            } => Resolved::Face(f, ResolveKind::Fingerprint),
            ResolveOutcome::Ambiguous { candidates } => Resolved::Ambiguous(
                candidates
                    .into_iter()
                    .filter_map(|(e, s)| match e {
                        EntityRef::Face(f) => Some((f, s)),
                        _ => None,
                    })
                    .collect(),
            ),
            _ => Resolved::Broken,
        }
    }

    /// A reference to `face` of the body this map names, for storing in a feature.
    pub fn reference_to(&self, rep: &BRep, face: FaceId) -> Option<Reference> {
        let (name, _) = self
            .by_name
            .iter()
            .find(|(_, e)| **e == EntityRef::Face(face))?;
        let bb = rep.body_bbox(rep.bodies.keys().next()?);
        Some(Reference {
            name: name.clone(),
            fingerprint: GeomFingerprint::of(rep, face),
            scale: bb.diagonal().max(1e-9),
        })
    }
}

#[allow(dead_code)]
fn _unused(_: &dyn Surface) {}
