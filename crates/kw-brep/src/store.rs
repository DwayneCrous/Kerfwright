use crate::ids::*;
use crate::topology::*;
use kw_math::Point3;
use kw_nurbs::{CurveGeom, SurfaceGeom};
use slotmap::SlotMap;

/// One document body's topology and geometry arenas.
#[derive(Debug, Default, Clone)]
pub struct BRep {
    pub vertices: SlotMap<VertexId, Vertex>,
    pub edges: SlotMap<EdgeId, Edge>,
    pub coedges: SlotMap<CoedgeId, Coedge>,
    pub loops: SlotMap<LoopId, Loop>,
    pub faces: SlotMap<FaceId, Face>,
    pub shells: SlotMap<ShellId, Shell>,
    pub lumps: SlotMap<LumpId, Lump>,
    pub bodies: SlotMap<BodyId, Body>,
    pub curves: SlotMap<CurveId, CurveGeom>,
    pub surfaces: SlotMap<SurfaceId, SurfaceGeom>,
}

/// Description of one oriented edge use when building a loop.
#[derive(Debug, Clone, Copy)]
pub struct CoedgeSpec {
    pub edge: EdgeId,
    pub sense: Sense,
    /// Parametric curve in the face's `(u, v)` space (stored as a 3D curve
    /// with `z = 0`), sharing the edge's parameterisation.
    pub pcurve: Option<CurveId>,
}

impl BRep {
    pub fn new() -> Self {
        Self::default()
    }

    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    pub fn add_vertex(&mut self, point: Point3<f64>) -> VertexId {
        self.vertices.insert(Vertex {
            point,
            tolerance: 0.0,
        })
    }

    pub fn add_curve(&mut self, c: CurveGeom) -> CurveId {
        self.curves.insert(c)
    }

    pub fn add_surface(&mut self, s: SurfaceGeom) -> SurfaceId {
        self.surfaces.insert(s)
    }

    pub fn add_edge(
        &mut self,
        start: VertexId,
        end: VertexId,
        curve: Option<CurveId>,
        param_range: (f64, f64),
    ) -> EdgeId {
        self.edges.insert(Edge {
            start,
            end,
            curve,
            param_range,
            tolerance: 0.0,
            first_coedge: None,
        })
    }

    pub fn add_body(&mut self, kind: BodyKind) -> BodyId {
        self.bodies.insert(Body {
            kind,
            lumps: Vec::new(),
        })
    }

    pub fn add_lump(&mut self, body: BodyId) -> LumpId {
        let id = self.lumps.insert(Lump {
            body,
            shells: Vec::new(),
        });
        self.bodies[body].lumps.push(id);
        id
    }

    pub fn add_shell(&mut self, lump: LumpId) -> ShellId {
        let id = self.shells.insert(Shell {
            lump,
            faces: Vec::new(),
        });
        self.lumps[lump].shells.push(id);
        id
    }

    pub fn add_face(&mut self, shell: ShellId, surface: SurfaceId, sense: Sense) -> FaceId {
        let id = self.faces.insert(Face {
            shell,
            surface,
            sense,
            loops: Vec::new(),
        });
        self.shells[shell].faces.push(id);
        id
    }

    /// Create a loop on `face` from an ordered list of coedge uses. Links
    /// `next`/`prev` cyclically and inserts each coedge in its edge's radial ring.
    pub fn add_loop(&mut self, face: FaceId, kind: LoopKind, uses: &[CoedgeSpec]) -> LoopId {
        assert!(!uses.is_empty(), "a loop needs at least one coedge");
        let loop_id = self.loops.insert_with_key(|_| Loop {
            face,
            kind,
            // patched below once the first coedge exists
            first_coedge: CoedgeId::default(),
        });
        let mut ids: Vec<CoedgeId> = Vec::with_capacity(uses.len());
        for spec in uses {
            let id = self.coedges.insert_with_key(|k| Coedge {
                edge: spec.edge,
                owner_loop: loop_id,
                next: k,
                prev: k,
                radial_next: k,
                sense: spec.sense,
                pcurve: spec.pcurve,
            });
            ids.push(id);
        }
        let n = ids.len();
        for i in 0..n {
            self.coedges[ids[i]].next = ids[(i + 1) % n];
            self.coedges[ids[i]].prev = ids[(i + n - 1) % n];
        }
        self.loops[loop_id].first_coedge = ids[0];
        for (id, spec) in ids.iter().zip(uses) {
            match self.edges[spec.edge].first_coedge {
                None => {
                    self.edges[spec.edge].first_coedge = Some(*id);
                }
                Some(first) => {
                    let after = self.coedges[first].radial_next;
                    self.coedges[*id].radial_next = after;
                    self.coedges[first].radial_next = *id;
                }
            }
        }
        self.faces[face].loops.push(loop_id);
        loop_id
    }

    // ------------------------------------------------------------------
    // Traversal
    // ------------------------------------------------------------------

    /// Coedges of `edge`'s radial ring, starting at `first_coedge`. Stops if
    /// the ring is broken (never loops forever).
    pub fn radial_ring(&self, edge: EdgeId) -> Vec<CoedgeId> {
        let mut out = Vec::new();
        let Some(start) = self.edges.get(edge).and_then(|e| e.first_coedge) else {
            return out;
        };
        let mut cur = start;
        loop {
            out.push(cur);
            match self.coedges.get(cur) {
                Some(c) => cur = c.radial_next,
                None => break,
            }
            if cur == start || out.len() > self.coedges.len() {
                break;
            }
        }
        out
    }

    /// Faces adjacent to `edge` (one per coedge in the radial ring).
    pub fn faces_around_edge(&self, edge: EdgeId) -> Vec<FaceId> {
        self.radial_ring(edge)
            .into_iter()
            .filter_map(|c| self.coedges.get(c))
            .filter_map(|c| self.loops.get(c.owner_loop))
            .map(|l| l.face)
            .collect()
    }

    /// Coedges of a loop in order. Stops if the ring is broken.
    pub fn loop_coedges(&self, lp: LoopId) -> Vec<CoedgeId> {
        let mut out = Vec::new();
        let Some(start) = self.loops.get(lp).map(|l| l.first_coedge) else {
            return out;
        };
        let mut cur = start;
        loop {
            out.push(cur);
            match self.coedges.get(cur) {
                Some(c) => cur = c.next,
                None => break,
            }
            if cur == start || out.len() > self.coedges.len() {
                break;
            }
        }
        out
    }

    /// Start vertex of a coedge in traversal direction.
    pub fn coedge_start(&self, c: CoedgeId) -> Option<VertexId> {
        let co = self.coedges.get(c)?;
        let e = self.edges.get(co.edge)?;
        Some(match co.sense {
            Sense::Forward => e.start,
            Sense::Reversed => e.end,
        })
    }

    /// End vertex of a coedge in traversal direction.
    pub fn coedge_end(&self, c: CoedgeId) -> Option<VertexId> {
        let co = self.coedges.get(c)?;
        let e = self.edges.get(co.edge)?;
        Some(match co.sense {
            Sense::Forward => e.end,
            Sense::Reversed => e.start,
        })
    }

    /// Parameter range traversed by a coedge, in traversal order.
    pub fn coedge_param_range(&self, c: CoedgeId) -> Option<(f64, f64)> {
        let co = self.coedges.get(c)?;
        let e = self.edges.get(co.edge)?;
        Some(match co.sense {
            Sense::Forward => e.param_range,
            Sense::Reversed => (e.param_range.1, e.param_range.0),
        })
    }

    /// Faces of a body (through lumps and shells).
    pub fn body_faces(&self, body: BodyId) -> Vec<FaceId> {
        let mut out = Vec::new();
        if let Some(b) = self.bodies.get(body) {
            for l in &b.lumps {
                if let Some(l) = self.lumps.get(*l) {
                    for s in &l.shells {
                        if let Some(s) = self.shells.get(*s) {
                            out.extend(s.faces.iter().copied());
                        }
                    }
                }
            }
        }
        out
    }
}

impl BRep {
    /// Remove edges that no loop uses, then vertices that no remaining edge references, and curves
    /// that nothing references. Returns the number of edges removed.
    pub fn prune_orphans(&mut self) -> usize {
        let orphan_edges: Vec<EdgeId> = self
            .edges
            .iter()
            .filter(|(_, e)| e.first_coedge.is_none())
            .map(|(id, _)| id)
            .collect();
        for e in &orphan_edges {
            self.edges.remove(*e);
        }
        let mut used_v = std::collections::HashSet::new();
        let mut used_c = std::collections::HashSet::new();
        for e in self.edges.values() {
            used_v.insert(e.start);
            used_v.insert(e.end);
            if let Some(c) = e.curve {
                used_c.insert(c);
            }
        }
        for c in self.coedges.values() {
            if let Some(p) = c.pcurve {
                used_c.insert(p);
            }
        }
        let dead_v: Vec<VertexId> = self
            .vertices
            .keys()
            .filter(|v| !used_v.contains(v))
            .collect();
        for v in dead_v {
            self.vertices.remove(v);
        }
        let dead_c: Vec<CurveId> = self.curves.keys().filter(|c| !used_c.contains(c)).collect();
        for c in dead_c {
            self.curves.remove(c);
        }
        orphan_edges.len()
    }
}

impl BRep {
    /// Move the lumps of `others` into `target` (which then represents their disjoint union) and
    /// delete the emptied bodies. The caller guarantees the bodies do not overlap.
    pub fn merge_bodies(&mut self, target: BodyId, others: &[BodyId]) {
        for &o in others {
            if o == target {
                continue;
            }
            let lumps = std::mem::take(&mut self.bodies[o].lumps);
            for l in lumps {
                self.lumps[l].body = target;
                self.bodies[target].lumps.push(l);
            }
            self.bodies.remove(o);
        }
    }
}
