//! Working data of one Boolean: face/edge records copied out of the B-rep, intersection cuts,
//! the global edge/piece/vertex structures and per-face fragments.

use crate::{BoolOptions, BooleanError};
use kw_brep::uvgeom::{UvLoop, UV};
use kw_brep::{BRep, BodyId, BodyKind, CoedgeId, EdgeId, FaceId, LoopKind, Sense, VertexId};
use kw_math::{Aabb, Point3};
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};
use std::collections::HashMap;

#[derive(Debug, Clone)]
#[allow(dead_code)] // retained for diagnostics
pub struct CoedgeRec {
    pub id: CoedgeId,
    pub edge: EdgeId,
    /// Traversed along increasing edge parameter.
    pub fwd: bool,
    pub pcurve: CurveGeom,
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // retained for diagnostics
pub struct LoopRec {
    pub kind: LoopKind,
    pub coedges: Vec<CoedgeRec>,
}

#[derive(Debug, Clone)]
pub struct FaceRec {
    pub id: FaceId,
    /// 0 = first operand, 1 = second.
    pub body: usize,
    pub surface: SurfaceGeom,
    pub sense: Sense,
    /// +1 for forward faces, −1 for reversed: the internal parameter plane is `(u, sgn·v)` in
    /// which every face has counter-clockwise outer loops.
    pub sgn: f64,
    pub loops: Vec<LoopRec>,
    pub uv_loops: Vec<UvLoop>,
    pub loop_index: std::sync::Arc<kw_brep::uvgeom::LoopIndex>,
    pub uv_lo: UV,
    pub uv_hi: UV,
    pub bbox: Aabb,
    pub involved: bool,
    /// Samples of a B-spline surface over the face's parameter box, made when first needed.
    pub grid: std::sync::OnceLock<Option<kw_nurbs::SurfaceGrid>>,
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // retained for diagnostics
pub struct EdgeRec {
    pub id: EdgeId,
    pub curve: Option<CurveGeom>,
    pub range: (f64, f64),
    pub v0: VertexId,
    pub v1: VertexId,
    pub p0: Point3<f64>,
    pub p1: Point3<f64>,
    pub bbox: Aabb,
}

/// Where a global edge came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GSrc {
    Orig(EdgeId),
    Cut,
}

#[derive(Debug, Clone)]
pub struct GUse {
    pub face: usize,
    /// Index of the global edge whose curve parameterisation `pcurve` is defined against.
    pub defined_on: usize,
    /// Parameter-space curve on the face, defined over the *gedge's* curve parameter.
    pub pcurve: CurveGeom,
    /// Parameter range over which `pcurve` is valid (the piece's range on `defined_on`).
    pub range: (f64, f64),
    /// Canonical forward travel of the piece runs this pcurve's parameter backwards.
    pub reversed: bool,
    /// `Some(fwd)` for a boundary use (an original coedge), `None` for a cut / imprint.
    pub boundary_fwd: Option<bool>,
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // retained for diagnostics
pub struct GEdge {
    pub curve: Option<CurveGeom>,
    pub range: (f64, f64),
    pub closed: bool,
    pub src: GSrc,
    pub uses: Vec<GUse>,
    pub bbox: Aabb,
    /// Coarse polyline of the curve and its largest segment length (quick rejection of point-on-curve tests).
    pub coarse: Vec<Point3<f64>>,
    pub coarse_gap: f64,
    pub v_start: usize,
    pub v_end: usize,
    /// Interior split points `(param, vertex)`.
    pub splits: Vec<(f64, usize)>,
    /// A cut along which the two surfaces only touch.
    pub tangent: bool,
}

#[derive(Debug, Clone)]
pub struct Piece {
    pub gedge: usize,
    pub range: (f64, f64),
    pub v0: usize,
    pub v1: usize,
    pub uses: Vec<GUse>,
    /// Canonical piece this one was merged into (itself if none), and whether the orientation is reversed.
    pub canon: usize,
    pub flipped: bool,
}

/// A cut produced by intersecting face `fa` with face `fb`.
#[derive(Debug, Clone)]
#[allow(dead_code)] // retained for diagnostics
pub struct CutSeg {
    pub curve: CurveGeom,
    pub range: (f64, f64),
    pub closed: bool,
    pub fa: usize,
    pub fb: usize,
    pub pcurve_a: CurveGeom,
    pub pcurve_b: CurveGeom,
    /// The two surfaces touch (rather than cross) along this curve.
    pub tangent: bool,
}

#[derive(Debug, Default)]
pub struct Registry {
    pub pts: Vec<Point3<f64>>,
    pub tol: f64,
    grid: HashMap<(i64, i64, i64), Vec<usize>>,
}

impl Registry {
    pub fn new(tol: f64) -> Self {
        Registry {
            pts: Vec::new(),
            tol,
            grid: HashMap::new(),
        }
    }

    fn cell(&self, p: &Point3<f64>) -> (i64, i64, i64) {
        let s = 8.0 * self.tol;
        (
            (p.x / s).floor() as i64,
            (p.y / s).floor() as i64,
            (p.z / s).floor() as i64,
        )
    }

    pub fn find(&self, p: &Point3<f64>) -> Option<usize> {
        // cells are 8·tol wide: a registered point within `tol` can only sit in a neighbouring cell on
        // an axis where `p` is within `tol` of the cell boundary
        let s = 8.0 * self.tol;
        let offsets = |x: f64| -> ([i64; 3], usize, i64) {
            let q = x / s;
            let c = q.floor();
            let frac = q - c;
            let mut o = [0i64; 3];
            let mut n = 1;
            if frac < 0.125 + 1e-9 {
                o[n] = -1;
                n += 1;
            }
            if frac > 0.875 - 1e-9 {
                o[n] = 1;
                n += 1;
            }
            (o, n, c as i64)
        };
        let (ox, nx, cx) = offsets(p.x);
        let (oy, ny, cy) = offsets(p.y);
        let (oz, nz, cz) = offsets(p.z);
        for dx in &ox[..nx] {
            for dy in &oy[..ny] {
                for dz in &oz[..nz] {
                    if let Some(list) = self.grid.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &i in list {
                            if (self.pts[i] - p).norm() <= self.tol {
                                return Some(i);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    pub fn add(&mut self, p: Point3<f64>) -> usize {
        if let Some(i) = self.find(&p) {
            return i;
        }
        let i = self.pts.len();
        self.pts.push(p);
        let c = self.cell(&p);
        self.grid.entry(c).or_default().push(i);
        i
    }
}

#[allow(dead_code)] // retained for diagnostics
pub struct Work {
    pub bodies: [BodyId; 2],
    pub tol: f64,
    pub scale: f64,
    pub faces: Vec<FaceRec>,
    pub edges: HashMap<EdgeId, EdgeRec>,
    pub face_index: HashMap<FaceId, usize>,
    /// Distinct edges of every face (in loop order), computed once.
    pub face_edge_ids: Vec<Vec<EdgeId>>,
    /// Spatial index of each face's edge boxes (for faces with many edges).
    pub face_edge_grid: Vec<Option<crate::spatial::GridIndex>>,
    /// `(piece, use index)` for every use of every canonical piece, grouped by face.
    pub pieces_of_face: Vec<Vec<(usize, usize)>>,
    /// Global edge index of every original edge.
    pub gedge_of_edge: HashMap<EdgeId, usize>,
    pub pairs: Vec<(usize, usize)>,
    pub cuts: Vec<CutSeg>,
    /// Pairs of faces lying on the same surface: `(fa, fb, same_normal)`.
    pub coincident: Vec<(usize, usize, bool)>,
    pub reg: Registry,
    pub gedges: Vec<GEdge>,
    pub pieces: Vec<Piece>,
    pub fragments: Vec<crate::arrangement::Fragment>,
    /// The cut network looks incomplete (a slit, an odd junction inside a face): the result must be
    /// verified independently before it is trusted.
    pub suspect: bool,
    /// Points just beside the intersection curves and near suspicious junctions, used to verify a
    /// suspect result by ray casting.
    pub probes: Vec<Point3<f64>>,
}

impl Work {
    pub fn new(rep: &BRep, a: BodyId, b: BodyId, opts: &BoolOptions) -> Result<Work, BooleanError> {
        use rayon::prelude::*;
        let mut order: Vec<(FaceId, usize)> = Vec::new();
        for (bi, body) in [a, b].into_iter().enumerate() {
            let bd = rep
                .bodies
                .get(body)
                .ok_or(BooleanError::BadInput("unknown body"))?;
            if bd.kind != BodyKind::Solid {
                return Err(BooleanError::BadInput("operands must be solid bodies"));
            }
            for f in rep.body_faces(body) {
                order.push((f, bi));
            }
        }
        // faces are converted independently (results come back in face order)
        let built: Vec<Result<FaceRec, BooleanError>> = order
            .par_iter()
            .map(|&(f, bi)| -> Result<FaceRec, BooleanError> {
                let face = &rep.faces[f];
                let surface = rep
                    .surfaces
                    .get(face.surface)
                    .ok_or(BooleanError::BadInput("missing surface"))?
                    .clone();
                let mut loops = Vec::new();
                for &lp in &face.loops {
                    let mut cos = Vec::new();
                    for co in rep.loop_coedges(lp) {
                        let c = &rep.coedges[co];
                        let pc = c
                            .pcurve
                            .and_then(|p| rep.curves.get(p))
                            .ok_or(BooleanError::BadInput("coedge without pcurve"))?
                            .clone();
                        cos.push(CoedgeRec {
                            id: co,
                            edge: c.edge,
                            fwd: c.sense == Sense::Forward,
                            pcurve: pc,
                        });
                    }
                    loops.push(LoopRec {
                        kind: rep.loops[lp].kind,
                        coedges: cos,
                    });
                }
                // the loops of a face with many holes are sampled in parallel
                let uv_loops: Vec<kw_brep::uvgeom::UvLoop> = if face.loops.len() > 8 {
                    face.loops
                        .par_iter()
                        .map(|l| rep.loop_uv(*l, 3))
                        .collect::<Option<Vec<_>>>()
                } else {
                    rep.face_uv_loops(f, 3)
                }
                .ok_or(BooleanError::BadInput("face without trim"))?;
                let (uv_lo, uv_hi) = kw_brep::uvgeom::loops_bbox(&uv_loops)
                    .ok_or(BooleanError::BadInput("face without trim"))?;
                let loop_index = std::sync::Arc::new(kw_brep::uvgeom::LoopIndex::new(&uv_loops));
                let bbox = rep
                    .face_bbox_with(f, &uv_loops, &loop_index, uv_lo, uv_hi)
                    .ok_or(BooleanError::BadInput("face without bounds"))?;
                Ok(FaceRec {
                    id: f,
                    body: bi,
                    surface,
                    sense: face.sense,
                    sgn: if face.sense == Sense::Forward {
                        1.0
                    } else {
                        -1.0
                    },
                    loops,
                    loop_index,
                    uv_loops,
                    uv_lo,
                    uv_hi,
                    bbox,
                    involved: false,
                    grid: std::sync::OnceLock::new(),
                })
            })
            .collect();
        let faces: Vec<FaceRec> = built.into_iter().collect::<Result<_, _>>()?;
        let mut face_index = HashMap::new();
        for (i, f) in faces.iter().enumerate() {
            face_index.insert(f.id, i);
        }
        // distinct edges, in first-use order
        let mut edge_ids: Vec<EdgeId> = Vec::new();
        {
            let mut seen = std::collections::HashSet::new();
            for f in &faces {
                for l in &f.loops {
                    for c in &l.coedges {
                        if seen.insert(c.edge) {
                            edge_ids.push(c.edge);
                        }
                    }
                }
            }
        }
        let recs: Vec<EdgeRec> = edge_ids
            .par_iter()
            .map(|&eid| {
                let e = &rep.edges[eid];
                let (p0, p1) = (rep.vertices[e.start].point, rep.vertices[e.end].point);
                let curve = e.curve.and_then(|c| rep.curves.get(c)).cloned();
                let bbox = match &curve {
                    Some(cv) => {
                        let n = if matches!(cv, kw_nurbs::CurveGeom::Line(_)) {
                            1
                        } else {
                            32
                        };
                        let pts: Vec<Point3<f64>> = (0..=n)
                            .map(|i| {
                                cv.point_at(
                                    e.param_range.0
                                        + (e.param_range.1 - e.param_range.0) * i as f64 / n as f64,
                                )
                            })
                            .collect();
                        let mut b = Aabb::from_points(&pts);
                        let pad = 0.03 * b.diagonal() + 1e-9;
                        b.min -= kw_math::Vector3::repeat(pad);
                        b.max += kw_math::Vector3::repeat(pad);
                        b
                    }
                    None => Aabb::from_points(&[p0]),
                };
                EdgeRec {
                    id: eid,
                    curve,
                    range: e.param_range,
                    v0: e.start,
                    v1: e.end,
                    p0,
                    p1,
                    bbox,
                }
            })
            .collect();
        let mut scale = 1.0f64;
        for r in &recs {
            scale = scale.max(r.p0.coords.amax()).max(r.p1.coords.amax());
        }
        let edges: HashMap<EdgeId, EdgeRec> = recs.into_iter().map(|r| (r.id, r)).collect();
        let tol = opts.tol.max(1e-12) * (1.0 + scale);
        let face_edge_ids: Vec<Vec<EdgeId>> = faces
            .iter()
            .map(|f| {
                let mut seen = std::collections::HashSet::new();
                let mut v = Vec::new();
                for l in &f.loops {
                    for c in &l.coedges {
                        if seen.insert(c.edge) {
                            v.push(c.edge);
                        }
                    }
                }
                v
            })
            .collect();
        Ok(Work {
            bodies: [a, b],
            tol,
            scale,
            faces,
            edges,
            face_index,
            face_edge_ids,
            face_edge_grid: Vec::new(),
            pieces_of_face: Vec::new(),
            gedge_of_edge: HashMap::new(),
            pairs: Vec::new(),
            cuts: Vec::new(),
            coincident: Vec::new(),
            reg: Registry::new(tol),
            gedges: Vec::new(),
            pieces: Vec::new(),
            fragments: Vec::new(),
            suspect: false,
            probes: Vec::new(),
        })
    }
}

#[allow(dead_code)]
impl CutSeg {
    pub fn curve_start(&self) -> Point3<f64> {
        self.curve.point_at(self.range.0)
    }
    pub fn curve_end(&self) -> Point3<f64> {
        self.curve.point_at(self.range.1)
    }
}

impl FaceRec {
    pub fn centre(&self) -> UV {
        (
            0.5 * (self.uv_lo.0 + self.uv_hi.0),
            0.5 * (self.uv_lo.1 + self.uv_hi.1),
        )
    }

    /// Parameters of `p` on this face's surface, on the branch nearest the face.
    pub fn uv_of(&self, p: &Point3<f64>) -> UV {
        // a B-spline surface is searched from the nearest of a grid of samples: a search of the whole
        // surface costs milliseconds on a surface with many spans, and a cut asks for thousands of points
        if let SurfaceGeom::Nurbs(ns) = &self.surface {
            let grid = self.grid.get_or_init(|| {
                let w = |lo: f64, hi: f64| {
                    let m = 0.05 * (hi - lo).max(1e-6);
                    (lo - m, hi + m)
                };
                Some(ns.sample_grid(
                    w(self.uv_lo.0, self.uv_hi.0),
                    w(self.uv_lo.1, self.uv_hi.1),
                    40,
                ))
            });
            if let Some(g) = grid {
                let r = ns.project_in_grid(g, p);
                return kw_brep::uvgeom::unwrap_uv(&self.surface, (r.u, r.v), self.centre());
            }
        }
        let r = kw_nurbs::proj::surface_param_of(&self.surface, p, Some(self.centre()));
        kw_brep::uvgeom::unwrap_uv(&self.surface, (r.u, r.v), self.centre())
    }

    pub fn point_at(&self, uv: UV) -> Point3<f64> {
        use kw_nurbs::Surface;
        self.surface.point_at(uv.0, uv.1)
    }
}

#[allow(dead_code)]
pub fn curve_point(c: &CurveGeom, t: f64) -> Point3<f64> {
    c.point_at(t)
}
