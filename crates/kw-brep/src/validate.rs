//! Structural and geometric validation of a [`BRep`].
//!
//! Checks (all collected, none short-circuit):
//! * reference integrity and parent/child back-pointers
//! * loop closure, `next`/`prev` inverse, vertex continuity around loops
//! * radial rings, manifold edge pairing and orientation (solid bodies)
//! * Euler–Poincaré per shell
//! * edge curve ↔ vertex agreement, pcurve ↔ surface ↔ curve agreement
//! * uv continuity of loops and loop orientation vs. face sense

use crate::ids::*;
use crate::store::BRep;
use crate::topology::*;
use kw_nurbs::{Curve3, Surface};
use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone, PartialEq)]
pub enum ValidationError {
    Dangling {
        what: &'static str,
        from: String,
    },
    BackPointer {
        what: &'static str,
        id: String,
    },
    LoopNotClosed(LoopId),
    LoopBadOuterCount {
        face: FaceId,
        outers: usize,
    },
    LoopDiscontinuous {
        loop_id: LoopId,
        at: CoedgeId,
    },
    OrphanCoedge(CoedgeId),
    RadialRing {
        edge: EdgeId,
        reason: &'static str,
    },
    Orientation {
        edge: EdgeId,
    },
    EulerPoincare {
        shell: ShellId,
        lhs: i64,
    },
    Geometry {
        what: &'static str,
        id: String,
        deviation: f64,
    },
    MissingPcurve(CoedgeId),
    LoopOrientation {
        loop_id: LoopId,
        area: f64,
    },
    NonPositiveVolume(f64),
}

#[derive(Debug, Clone, Default)]
pub struct ValidationReport {
    pub errors: Vec<ValidationError>,
    /// Genus per shell (when Euler–Poincaré could be evaluated).
    pub shell_genus: Vec<(ShellId, i64)>,
}

impl ValidationReport {
    pub fn is_ok(&self) -> bool {
        self.errors.is_empty()
    }
    pub fn into_result(self) -> Result<ValidationReport, Vec<ValidationError>> {
        if self.errors.is_empty() {
            Ok(self)
        } else {
            Err(self.errors)
        }
    }
}

/// A parameter-space gap is harmless if it is small and the two surface points it separates are
/// within the 3D tolerance (the parameterisation can be very contracted, e.g. near a cone apex).
fn uv_gap_is_geometrically_closed(
    surface: &kw_nurbs::SurfaceGeom,
    a: (f64, f64),
    b: (f64, f64),
    uv_gap: f64,
    tol: f64,
) -> bool {
    use kw_nurbs::Surface;
    uv_gap < 1e-4 && (surface.point_at(a.0, a.1) - surface.point_at(b.0, b.1)).norm() <= tol
}

impl BRep {
    /// Full validation: topology followed by geometry.
    pub fn validate(&self) -> ValidationReport {
        let mut rep = self.validate_topology();
        self.validate_geometry(&mut rep);
        rep
    }

    /// Convenience: `Err` with all problems, or `Ok(())`.
    pub fn check(&self) -> Result<(), Vec<ValidationError>> {
        self.validate().into_result().map(|_| ())
    }

    pub fn validate_topology(&self) -> ValidationReport {
        let mut rep = ValidationReport::default();
        let errs = &mut rep.errors;
        let dangling = |what: &'static str, from: String| ValidationError::Dangling { what, from };
        let back = |what: &'static str, id: String| ValidationError::BackPointer { what, id };

        // --- hierarchy: bodies → lumps → shells → faces → loops --------------
        for (bid, b) in &self.bodies {
            for l in &b.lumps {
                match self.lumps.get(*l) {
                    None => errs.push(dangling("lump", format!("{bid:?}"))),
                    Some(lump) if lump.body != bid => {
                        errs.push(back("lump.body", format!("{l:?}")))
                    }
                    _ => {}
                }
            }
        }
        for (lid, l) in &self.lumps {
            if self
                .bodies
                .get(l.body)
                .is_none_or(|b| b.lumps.iter().filter(|x| **x == lid).count() != 1)
            {
                errs.push(back("lump not listed once in body", format!("{lid:?}")));
            }
            for s in &l.shells {
                match self.shells.get(*s) {
                    None => errs.push(dangling("shell", format!("{lid:?}"))),
                    Some(sh) if sh.lump != lid => errs.push(back("shell.lump", format!("{s:?}"))),
                    _ => {}
                }
            }
        }
        for (sid, s) in &self.shells {
            if self
                .lumps
                .get(s.lump)
                .is_none_or(|l| l.shells.iter().filter(|x| **x == sid).count() != 1)
            {
                errs.push(back("shell not listed once in lump", format!("{sid:?}")));
            }
            for f in &s.faces {
                match self.faces.get(*f) {
                    None => errs.push(dangling("face", format!("{sid:?}"))),
                    Some(face) if face.shell != sid => {
                        errs.push(back("face.shell", format!("{f:?}")))
                    }
                    _ => {}
                }
            }
        }
        for (fid, f) in &self.faces {
            if self
                .shells
                .get(f.shell)
                .is_none_or(|s| s.faces.iter().filter(|x| **x == fid).count() != 1)
            {
                errs.push(back("face not listed once in shell", format!("{fid:?}")));
            }
            if !self.surfaces.contains_key(f.surface) {
                errs.push(dangling("surface", format!("{fid:?}")));
            }
            let mut outers = 0;
            for l in &f.loops {
                match self.loops.get(*l) {
                    None => errs.push(dangling("loop", format!("{fid:?}"))),
                    Some(lp) => {
                        if lp.face != fid {
                            errs.push(back("loop.face", format!("{l:?}")));
                        }
                        if lp.kind == LoopKind::Outer {
                            outers += 1;
                        }
                    }
                }
            }
            if outers != 1 {
                errs.push(ValidationError::LoopBadOuterCount { face: fid, outers });
            }
        }
        for (lid, l) in &self.loops {
            if self
                .faces
                .get(l.face)
                .is_none_or(|f| f.loops.iter().filter(|x| **x == lid).count() != 1)
            {
                errs.push(back("loop not listed once in face", format!("{lid:?}")));
            }
        }

        // --- loops and coedges ------------------------------------------------
        let mut visited: HashSet<CoedgeId> = HashSet::new();
        for (lid, l) in &self.loops {
            if !self.coedges.contains_key(l.first_coedge) {
                errs.push(dangling("loop.first_coedge", format!("{lid:?}")));
                continue;
            }
            let ring = self.loop_coedges(lid);
            let closed = ring
                .last()
                .and_then(|c| self.coedges.get(*c))
                .map(|c| c.next)
                == Some(l.first_coedge);
            if !closed {
                errs.push(ValidationError::LoopNotClosed(lid));
                continue;
            }
            for (i, &cid) in ring.iter().enumerate() {
                visited.insert(cid);
                let c = &self.coedges[cid];
                if c.owner_loop != lid {
                    errs.push(back("coedge.owner_loop", format!("{cid:?}")));
                }
                let next = ring[(i + 1) % ring.len()];
                match self.coedges.get(c.prev) {
                    Some(p) if p.next == cid => {}
                    _ => errs.push(back("coedge.prev", format!("{cid:?}"))),
                }
                if !self.edges.contains_key(c.edge) {
                    errs.push(dangling("coedge.edge", format!("{cid:?}")));
                    continue;
                }
                if self.coedge_end(cid) != self.coedge_start(next) || self.coedge_end(cid).is_none()
                {
                    errs.push(ValidationError::LoopDiscontinuous {
                        loop_id: lid,
                        at: cid,
                    });
                }
                if let Some(pc) = c.pcurve {
                    if !self.curves.contains_key(pc) {
                        errs.push(dangling("coedge.pcurve", format!("{cid:?}")));
                    }
                }
            }
        }
        for (cid, c) in &self.coedges {
            if !visited.contains(&cid) {
                errs.push(ValidationError::OrphanCoedge(cid));
            }
            if !self.coedges.contains_key(c.radial_next) {
                errs.push(dangling("coedge.radial_next", format!("{cid:?}")));
            }
        }

        // --- edges and radial rings ------------------------------------------
        // closedness is a property of each body: the edges of a sheet (or wire) body may be free, those of
        // a solid must pair up exactly
        let open_edges: HashSet<EdgeId> = self
            .bodies
            .iter()
            .filter(|(_, b)| b.kind != BodyKind::Solid)
            .flat_map(|(id, _)| self.body_faces(id))
            .filter_map(|f| self.faces.get(f))
            .flat_map(|f| f.loops.iter())
            .flat_map(|l| self.loop_coedges(*l))
            .filter_map(|c| self.coedges.get(c).map(|c| c.edge))
            .collect();
        let mut uses: HashMap<EdgeId, usize> = HashMap::new();
        for c in self.coedges.values() {
            *uses.entry(c.edge).or_default() += 1;
        }
        for (eid, e) in &self.edges {
            if !self.vertices.contains_key(e.start) || !self.vertices.contains_key(e.end) {
                errs.push(dangling("edge.vertex", format!("{eid:?}")));
            }
            if let Some(c) = e.curve {
                if !self.curves.contains_key(c) {
                    errs.push(dangling("edge.curve", format!("{eid:?}")));
                }
            } else if e.start != e.end {
                errs.push(ValidationError::RadialRing {
                    edge: eid,
                    reason: "degenerate edge with distinct endpoints",
                });
            }
            let ring = self.radial_ring(eid);
            if ring.is_empty() {
                errs.push(ValidationError::RadialRing {
                    edge: eid,
                    reason: "no coedges",
                });
                continue;
            }
            let closed = self
                .coedges
                .get(*ring.last().unwrap())
                .map(|c| c.radial_next)
                == Some(ring[0]);
            if !closed {
                errs.push(ValidationError::RadialRing {
                    edge: eid,
                    reason: "radial ring not closed",
                });
                continue;
            }
            if ring.iter().any(|c| self.coedges[*c].edge != eid) {
                errs.push(ValidationError::RadialRing {
                    edge: eid,
                    reason: "ring member references another edge",
                });
            }
            if ring.len() != uses.get(&eid).copied().unwrap_or(0) {
                errs.push(ValidationError::RadialRing {
                    edge: eid,
                    reason: "ring misses coedges that use the edge",
                });
            }
            if !open_edges.contains(&eid) && !e.is_degenerate() {
                if ring.len() != 2 {
                    errs.push(ValidationError::RadialRing {
                        edge: eid,
                        reason: "solid edge must have exactly two coedges",
                    });
                } else if self.coedges[ring[0]].sense == self.coedges[ring[1]].sense {
                    errs.push(ValidationError::Orientation { edge: eid });
                }
            }
        }

        // --- Euler–Poincaré per shell -----------------------------------------
        {
            for (sid, s) in &self.shells {
                // only a solid's shells are closed surfaces
                let solid = self
                    .lumps
                    .get(s.lump)
                    .and_then(|l| self.bodies.get(l.body))
                    .is_some_and(|b| b.kind == BodyKind::Solid);
                if !solid {
                    continue;
                }
                let mut verts: HashSet<VertexId> = HashSet::new();
                let mut edges: HashSet<EdgeId> = HashSet::new();
                let mut inner = 0i64;
                let mut faces = 0i64;
                let mut ok = true;
                for f in &s.faces {
                    let Some(face) = self.faces.get(*f) else {
                        ok = false;
                        continue;
                    };
                    faces += 1;
                    for l in &face.loops {
                        let Some(lp) = self.loops.get(*l) else {
                            ok = false;
                            continue;
                        };
                        if lp.kind == LoopKind::Inner {
                            inner += 1;
                        }
                        for c in self.loop_coedges(*l) {
                            if let Some(co) = self.coedges.get(c) {
                                if let Some(e) = self.edges.get(co.edge) {
                                    verts.insert(e.start);
                                    verts.insert(e.end);
                                    if !e.is_degenerate() {
                                        edges.insert(co.edge);
                                    }
                                }
                            }
                        }
                    }
                }
                if !ok {
                    continue;
                }
                let lhs = verts.len() as i64 - edges.len() as i64 + faces - inner;
                // V − E + F − R = 2(1 − H) for a single shell, H ≥ 0 integer.
                if lhs % 2 != 0 || lhs > 2 {
                    errs.push(ValidationError::EulerPoincare { shell: sid, lhs });
                } else {
                    rep.shell_genus.push((sid, 1 - lhs / 2));
                }
            }
        }
        rep
    }

    /// Geometric consistency; appends to `rep.errors`.
    pub fn validate_geometry(&self, rep: &mut ValidationReport) {
        let scale = self
            .vertices
            .values()
            .map(|v| v.point.coords.amax())
            .fold(1.0_f64, f64::max);
        let tol = 1e-7 * (1.0 + scale);

        // edge curve ends vs vertices
        for (eid, e) in &self.edges {
            let (Some(vs), Some(ve)) = (self.vertices.get(e.start), self.vertices.get(e.end))
            else {
                continue;
            };
            if let Some(cid) = e.curve {
                let Some(c) = self.curves.get(cid) else {
                    continue;
                };
                let d0 = (c.point_at(e.param_range.0) - vs.point).norm();
                let d1 = (c.point_at(e.param_range.1) - ve.point).norm();
                let d = d0.max(d1);
                if d > tol.max(e.tolerance) {
                    rep.errors.push(ValidationError::Geometry {
                        what: "edge curve vs vertex",
                        id: format!("{eid:?}"),
                        deviation: d,
                    });
                }
            } else {
                let d = (vs.point - ve.point).norm();
                if d > tol {
                    rep.errors.push(ValidationError::Geometry {
                        what: "degenerate edge endpoints",
                        id: format!("{eid:?}"),
                        deviation: d,
                    });
                }
            }
        }

        // pcurves, uv continuity and loop orientation
        for (lid, l) in &self.loops {
            let Some(face) = self.faces.get(l.face) else {
                continue;
            };
            let Some(surface) = self.surfaces.get(face.surface) else {
                continue;
            };
            let ring = self.loop_coedges(lid);
            let closed = ring
                .last()
                .and_then(|c| self.coedges.get(*c))
                .map(|c| c.next)
                == Some(l.first_coedge);
            if !closed {
                continue;
            }
            let mut poly: Vec<(f64, f64)> = Vec::new();
            let mut prev_end: Option<(f64, f64)> = None;
            let mut first_start: Option<(f64, f64)> = None;
            let mut all_pcurves = true;
            for &cid in &ring {
                let co = &self.coedges[cid];
                let (Some(e), Some(range)) =
                    (self.edges.get(co.edge), self.coedge_param_range(cid))
                else {
                    all_pcurves = false;
                    continue;
                };
                let Some(pc) = co.pcurve.and_then(|p| self.curves.get(p)) else {
                    rep.errors.push(ValidationError::MissingPcurve(cid));
                    all_pcurves = false;
                    continue;
                };
                let uv = |t: f64| {
                    let p = pc.point_at(t);
                    (p.x, p.y)
                };
                let start = uv(range.0);
                let end = uv(range.1);
                if let Some(pe) = prev_end {
                    let d = ((pe.0 - start.0).powi(2) + (pe.1 - start.1).powi(2)).sqrt();
                    if d > 1e-8 && !uv_gap_is_geometrically_closed(surface, pe, start, d, tol) {
                        rep.errors.push(ValidationError::Geometry {
                            what: "uv discontinuity in loop",
                            id: format!("{cid:?}"),
                            deviation: d,
                        });
                    }
                }
                if first_start.is_none() {
                    first_start = Some(start);
                }
                prev_end = Some(end);
                // pcurve ↔ surface ↔ 3D curve at samples
                const N: usize = 8;
                for i in 0..=N {
                    let t = range.0 + (range.1 - range.0) * i as f64 / N as f64;
                    let (u, v) = uv(t);
                    let sp = surface.point_at(u, v);
                    let target = match e.curve.and_then(|c| self.curves.get(c)) {
                        Some(c) => c.point_at(t),
                        None => match self.vertices.get(e.start) {
                            Some(vx) => vx.point,
                            None => continue,
                        },
                    };
                    let d = (sp - target).norm();
                    if d > tol.max(e.tolerance) {
                        rep.errors.push(ValidationError::Geometry {
                            what: "pcurve vs edge curve",
                            id: format!("{cid:?}"),
                            deviation: d,
                        });
                        break;
                    }
                    if i < N {
                        poly.push((u, v));
                    }
                }
            }
            if let (Some(pe), Some(fs)) = (prev_end, first_start) {
                let d = ((pe.0 - fs.0).powi(2) + (pe.1 - fs.1).powi(2)).sqrt();
                if d > 1e-8 && !uv_gap_is_geometrically_closed(surface, pe, fs, d, tol) {
                    rep.errors.push(ValidationError::Geometry {
                        what: "uv loop not closed",
                        id: format!("{lid:?}"),
                        deviation: d,
                    });
                }
            }
            if all_pcurves && poly.len() >= 3 {
                let mut area = 0.0;
                for i in 0..poly.len() {
                    let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                    area += a.0 * b.1 - b.0 * a.1;
                }
                area *= 0.5;
                // Outer loops run CCW about the face's effective normal.
                let expect_positive = (l.kind == LoopKind::Outer) == (face.sense == Sense::Forward);
                if area == 0.0 || (area > 0.0) != expect_positive {
                    rep.errors
                        .push(ValidationError::LoopOrientation { loop_id: lid, area });
                }
            }
        }
    }
}
