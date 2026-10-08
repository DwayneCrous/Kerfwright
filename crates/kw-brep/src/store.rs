use crate::ids::*;
use crate::topology::*;
use kw_math::{CurveGeom, SurfaceGeom};
use slotmap::SlotMap;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationError {
    DanglingCoedge(CoedgeId),
    LoopNotClosed(LoopId),
    RadialRingBroken(EdgeId),
    EulerPoincare { lhs: i64, rhs: i64 },
}

/// One document body's topology and geometry arenas.
#[derive(Debug, Default)]
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

impl BRep {
    pub fn new() -> Self {
        Self::default()
    }

    /// Coedges of `edge`'s radial ring, starting at `first_coedge`.
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

    /// Coedges of a loop in order.
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

    /// Structural validation for a single closed manifold solid shell set:
    /// pointer integrity, loop closure, radial rings, and Euler–Poincaré
    /// `V - E + F - R = 2(S - H)` with `H` supplied by the caller (genus).
    pub fn validate_solid(&self, genus: i64) -> Result<(), ValidationError> {
        for (cid, c) in &self.coedges {
            if !self.edges.contains_key(c.edge)
                || !self.coedges.contains_key(c.next)
                || !self.coedges.contains_key(c.prev)
                || !self.coedges.contains_key(c.radial_next)
            {
                return Err(ValidationError::DanglingCoedge(cid));
            }
        }
        for (lid, _) in &self.loops {
            let ring = self.loop_coedges(lid);
            let closed = ring.last().and_then(|l| self.coedges.get(*l)).map(|c| c.next)
                == ring.first().copied();
            if ring.is_empty() || !closed {
                return Err(ValidationError::LoopNotClosed(lid));
            }
        }
        for (eid, _) in &self.edges {
            if self.radial_ring(eid).is_empty() {
                return Err(ValidationError::RadialRingBroken(eid));
            }
        }
        let v = self.vertices.len() as i64;
        let e = self.edges.len() as i64;
        let f = self.faces.len() as i64;
        let r = self
            .loops
            .values()
            .filter(|l| l.kind == LoopKind::Inner)
            .count() as i64;
        let s = self.shells.len() as i64;
        let lhs = v - e + f - r;
        let rhs = 2 * (s - genus);
        if lhs != rhs {
            return Err(ValidationError::EulerPoincare { lhs, rhs });
        }
        Ok(())
    }
}
