use crate::ids::*;
use kw_math::Point3;

/// Orientation of a use relative to the underlying geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sense {
    Forward,
    Reversed,
}

impl Sense {
    pub fn flip(self) -> Self {
        match self {
            Sense::Forward => Sense::Reversed,
            Sense::Reversed => Sense::Forward,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Vertex {
    pub point: Point3<f64>,
    /// Per-entity tolerance; grows when healing merges nearby geometry.
    pub tolerance: f64,
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub start: VertexId,
    pub end: VertexId,
    /// `None` for degenerate edges (e.g. sphere poles).
    pub curve: Option<CurveId>,
    pub param_range: (f64, f64),
    pub tolerance: f64,
    /// One coedge in the radial ring; iterate with `BRep::radial_ring`.
    /// 1 coedge = sheet/free boundary, 2 = manifold, >=3 = non-manifold.
    pub first_coedge: Option<CoedgeId>,
}

impl Edge {
    /// Degenerate edges (sphere poles, cone apex) collapse to a point and have
    /// no 3D curve; they do not count towards Euler–Poincaré.
    pub fn is_degenerate(&self) -> bool {
        self.curve.is_none()
    }
}

/// Oriented use of an edge within a face loop. Plays the role of the
/// half-edge (ACIS "coedge", Parasolid "fin").
#[derive(Debug, Clone)]
pub struct Coedge {
    pub edge: EdgeId,
    pub owner_loop: LoopId,
    /// Next/previous coedge around the loop.
    pub next: CoedgeId,
    pub prev: CoedgeId,
    /// Next coedge in the edge's radial ring (cycles back to the first).
    pub radial_next: CoedgeId,
    pub sense: Sense,
    /// 2D parametric curve on the face's surface, if stored.
    pub pcurve: Option<CurveId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopKind {
    Outer,
    Inner,
}

#[derive(Debug, Clone)]
pub struct Loop {
    pub face: FaceId,
    pub kind: LoopKind,
    pub first_coedge: CoedgeId,
}

#[derive(Debug, Clone)]
pub struct Face {
    pub shell: ShellId,
    pub surface: SurfaceId,
    /// `Reversed` means the face normal is opposite to the surface normal.
    pub sense: Sense,
    pub loops: Vec<LoopId>,
}

#[derive(Debug, Clone)]
pub struct Shell {
    pub lump: LumpId,
    pub faces: Vec<FaceId>,
}

/// Region of space bounded by shells: solid material (outer shell + void
/// shells) or sheet/wire material.
#[derive(Debug, Clone)]
pub struct Lump {
    pub body: BodyId,
    pub shells: Vec<ShellId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyKind {
    Solid,
    Sheet,
    Wire,
    /// Mixed / non-manifold.
    General,
}

#[derive(Debug, Clone)]
pub struct Body {
    pub kind: BodyKind,
    pub lumps: Vec<LumpId>,
}
