//! Delaunay-refinement tetrahedral mesher for a closed triangulated surface.
//!
//! 1. *Surface*: boundary triangles are refined (longest-edge bisection with Rivara's propagation, plus
//!    Lawson flips inside coplanar patches) until every triangle's diametral ball — the smallest sphere
//!    through its corners — is empty of vertices and small enough for the size field, and every *feature
//!    segment* (a sharp edge, or the border between two geometry faces) has an empty diametral ball.
//!    Such triangles and segments are faces and edges of every Delaunay triangulation, so the surface is
//!    recovered by a plain Delaunay triangulation of its vertices. Encroached segments are split before
//!    triangles: across a 90° edge a vertex of one face that encroaches a triangle of the other is the
//!    same thing as encroaching the shared segment, and splitting the triangle instead feeds a cascade.
//! 2. *Domain*: the tets on the inner side of the surface are kept; the surface faces become the
//!    boundary of the triangulation, which no later insertion can cross.
//! 3. *Interior*: tets with a radius–edge ratio above the bound (or too big for the size field) get a
//!    point inserted on the segment from their centroid to their circumcentre, as far out as the solid
//!    and the diametral balls of the boundary segments and triangles allow. The point is inside the
//!    circumball, so the tet disappears. When even points beside the centroid encroach, the boundary
//!    entity in the way is split instead.
//! 4. Smart-Laplacian smoothing of interior vertices.

use crate::quality::QualityReport;
use crate::surf::{sorted3, Surf, SurfaceMesh};
use crate::tri3::{delaunay, Loc, Tri3, NONE};
use crate::{cross, dot, norm, sub, SurfaceTri, TetMesh, P3};
use rstar::primitives::GeomWithData;
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet, VecDeque};
use std::sync::Arc;
use std::time::Instant;

pub type SizeFn = Arc<dyn Fn(P3) -> f64 + Send + Sync>;
pub type ProjectFn = Arc<dyn Fn(u32, P3) -> P3 + Send + Sync>;
pub use crate::remesh::ProjectEdgeFn;

#[derive(Clone)]
pub struct MeshParams {
    /// Radius–edge bound for interior tets.
    pub radius_edge: f64,
    /// Uniform target edge length (0 = none).
    pub max_size: f64,
    /// Spatially varying target edge length; the smaller of this and `max_size` applies.
    pub size: Option<SizeFn>,
    /// Edges shorter than twice this are never split (0 = 1e-4 × the bounding-box diagonal).
    pub min_size: f64,
    pub max_vertices: usize,
    /// Moves a boundary vertex (given the face tag) onto the true surface. Meshing works on the polyhedral
    /// input surface, which the Delaunay machinery is robust on; the vertices are snapped to the true
    /// surface at the end, each move only if the tets round the vertex stay well shaped.
    pub project: Option<ProjectFn>,
    /// Moves a vertex on a sharp edge between two faces onto the true curve (as `project`).
    pub project_edge: Option<ProjectEdgeFn>,
    /// Edges between triangles of one tag whose normals differ by more than this (degrees) are feature
    /// segments as well as all edges that separate two tags (face borders of the B-rep); within one
    /// tag the surface is smooth, and the tessellator's pole fans have normal jumps of 25° or more.
    pub feature_angle_deg: f64,
    /// Remesh the input surface towards the target size first (needs `max_size` or `size`): the
    /// tessellation of a B-rep is made for display and has pole fans and slivers that no refinement
    /// removes.
    pub remesh: bool,
    pub smooth_passes: usize,
    pub cancel: Option<kw_job::CancelFlag>,
}

impl Default for MeshParams {
    fn default() -> Self {
        MeshParams {
            radius_edge: 2.0,
            max_size: 0.0,
            size: None,
            min_size: 0.0,
            max_vertices: 2_000_000,
            project: None,
            project_edge: None,
            feature_angle_deg: 60.0,
            remesh: true,
            smooth_passes: 8,
            cancel: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum MeshError {
    NotClosed(String),
    Inverted,
    Conformity(String),
    Cancelled,
}

impl std::fmt::Display for MeshError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for MeshError {}

#[derive(Debug, Clone, Default)]
pub struct MeshReport {
    pub vertices: usize,
    pub tets: usize,
    pub input_vertices: usize,
    pub surface_splits: usize,
    pub segment_splits: usize,
    pub interior_points: usize,
    pub flips: usize,
    /// Boundary vertices moved onto the true surface.
    pub snapped: usize,
    /// Boundary splits by cause: forced segment, encroached segment, forced triangle, size, encroached.
    pub splits_by: [usize; 5],
    /// Edge and face removals of the volume mesh improvement.
    pub flips_volume: usize,
    /// Splits that were not possible because the edge was shorter than twice `min_size`.
    pub floor_hits: usize,
    pub failed_inserts: usize,
    /// Bad tets left alone in the polishing stage because fixing them needs the boundary to change.
    pub unfixed: usize,
    /// Numerically flat tets left alone.
    pub degenerate: usize,
    pub budget_hit: bool,
    pub quality: Option<QualityReport>,
    pub min_dihedral_deg: f64,
    pub max_dihedral_deg: f64,
    pub millis: f64,
}

#[derive(Debug, Clone, Copy)]
struct Ball {
    id: u32,
    c: P3,
    r: f64,
}
impl PartialEq for Ball {
    fn eq(&self, o: &Self) -> bool {
        self.id == o.id
    }
}
fn ball_envelope(c: &P3, r: f64) -> AABB<[f64; 3]> {
    AABB::from_corners(
        [c[0] - r, c[1] - r, c[2] - r],
        [c[0] + r, c[1] + r, c[2] + r],
    )
}
impl RTreeObject for Ball {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        ball_envelope(&self.c, self.r)
    }
}
impl PointDistance for Ball {
    fn distance_2(&self, p: &[f64; 3]) -> f64 {
        let d = sub(*p, self.c);
        dot(d, d)
    }
}

/// Diametral ball of a feature segment `u < v`.
#[derive(Debug, Clone, Copy)]
struct SegBall {
    u: u32,
    v: u32,
    c: P3,
    r: f64,
}
impl PartialEq for SegBall {
    fn eq(&self, o: &Self) -> bool {
        self.u == o.u && self.v == o.v
    }
}
impl RTreeObject for SegBall {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        ball_envelope(&self.c, self.r)
    }
}

#[derive(Debug, Clone, Copy)]
struct HeapItem {
    key: f64,
    t: u32,
}
impl PartialEq for HeapItem {
    fn eq(&self, o: &Self) -> bool {
        self.key.total_cmp(&o.key) == Ordering::Equal && self.t == o.t
    }
}
impl Eq for HeapItem {}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapItem {
    fn cmp(&self, o: &Self) -> Ordering {
        self.key.total_cmp(&o.key).then(self.t.cmp(&o.t))
    }
}

/// A vertex counts as encroaching only if it is inside the diametral ball by more than this relative
/// margin: cocircular vertices (regular grids on planar faces) must not trigger endless refinement; the
/// rare face that a tie keeps out of the Delaunay triangulation is found and flipped or split afterwards.
const ENCROACH_TOL: f64 = 1e-9;

/// Delaunay refinement is only guaranteed to terminate for radius–edge bounds above 2 (and cascades at
/// feature edges when run with exactly 2), so the boundary is refined against this bound and the target
/// bound is approached afterwards with the boundary frozen.
const SAFE_RADIUS_EDGE: f64 = 3.0;

/// Once the solid exists, quality-driven refinement does not go below this fraction of the target size:
/// features that small are not resolved, and chasing them only feeds cascades at sharp curved corners.
const LOCAL_FLOOR: f64 = 0.1;

/// Vertices within this fraction of a triangle's circumradius of its plane count as lying in it.
const SLAB: f64 = 0.35;
/// Neighbouring triangles of one smooth patch may be flipped when their normals differ by less than this.
const FLIP_COS: f64 = 0.9396926; // cos 20°

/// New interior tets must have `6·volume` of at least this fraction of (longest edge)³.
const FLAT_TOL: f64 = 1e-7;

enum DomainError {
    Missing(Vec<u32>),
    Leak(String),
}

fn mid_point(p: P3, q: P3) -> P3 {
    [
        0.5 * (p[0] + q[0]),
        0.5 * (p[1] + q[1]),
        0.5 * (p[2] + q[2]),
    ]
}

fn key2(u: u32, v: u32) -> (u32, u32) {
    (u.min(v), u.max(v))
}

struct Mesher {
    p: MeshParams,
    surf: Surf,
    vidx: RTree<GeomWithData<[f64; 3], u32>>,
    /// Interior vertices only (those that are not on the boundary), for the volume phase.
    vint: RTree<GeomWithData<[f64; 3], u32>>,
    tball: RTree<Ball>,
    balls: Vec<Ball>,
    feat: HashSet<(u32, u32)>,
    sball: RTree<SegBall>,
    /// Feature segments to test for encroachment.
    segq: VecDeque<(u32, u32)>,
    /// Feature segments to split regardless.
    forced_seg: VecDeque<(u32, u32)>,
    /// Triangles to test for encroachment or size.
    sq: VecDeque<u32>,
    /// Triangles to split regardless.
    forced: VecDeque<u32>,
    flip_stack: Vec<(u32, u32)>,
    dom: Option<Tri3>,
    heap: BinaryHeap<HeapItem>,
    rep: MeshReport,
    min_size: f64,
    big: f64,
    iter: usize,
    trace: bool,
    /// Current radius–edge bound: a safe one while the boundary is still being refined, the target one
    /// after the boundary is frozen.
    bound: f64,
    frozen: bool,
}

pub fn mesh_surface(
    input: &SurfaceMesh,
    params: &MeshParams,
) -> Result<(TetMesh, MeshReport), MeshError> {
    let t0 = Instant::now();
    input.check_closed().map_err(MeshError::NotClosed)?;
    if input.volume() <= 0.0 {
        return Err(MeshError::Inverted);
    }
    let remeshed;
    let input = if params.remesh && (params.max_size > 0.0 || params.size.is_some()) {
        let mut rp = crate::remesh::RemeshParams::new(if params.max_size > 0.0 {
            0.9 * params.max_size
        } else {
            1e300
        });
        if let Some(f) = &params.size {
            let f = f.clone();
            rp.size = Some(Arc::new(move |p| 0.9 * f(p)));
        }
        rp.feature_angle_deg = params.feature_angle_deg;
        let out = crate::remesh::remesh(input, &rp);
        if out.check_closed().is_ok() && out.volume() > 0.0 {
            remeshed = out;
            &remeshed
        } else {
            input
        }
    } else {
        input
    };
    let (lo, hi) = input.bounds();
    let diag = norm(sub(hi, lo));
    // Snap the vertices to a dyadic grid (unit ≈ 6e-8 of the size of the body). Edge midpoints are then
    // exact, so points created on a flat facet stay *exactly* coplanar with it; otherwise rounding noise
    // makes the exact predicates see tets of volume 1e-20 that can be neither split nor removed.
    let unit = 2f64.powi(diag.log2().floor() as i32 - 24);
    let mut snapped = input.clone();
    for p in &mut snapped.nodes {
        for k in 0..3 {
            p[k] = (p[k] / unit).round() * unit;
        }
    }
    snapped.check_closed().map_err(MeshError::NotClosed)?;
    if snapped.volume() <= 0.0
        || snapped
            .tris
            .iter()
            .any(|t| t[0] == t[1] || t[1] == t[2] || t[0] == t[2])
    {
        return Err(MeshError::Inverted);
    }
    let input = &snapped;
    let mut m = Mesher {
        p: params.clone(),
        surf: Surf::new(input),
        vidx: RTree::new(),
        vint: RTree::new(),
        tball: RTree::new(),
        balls: Vec::new(),
        feat: HashSet::new(),
        sball: RTree::new(),
        segq: VecDeque::new(),
        forced_seg: VecDeque::new(),
        sq: VecDeque::new(),
        forced: VecDeque::new(),
        flip_stack: Vec::new(),
        dom: None,
        heap: BinaryHeap::new(),
        rep: MeshReport {
            input_vertices: input.nodes.len(),
            ..Default::default()
        },
        min_size: if params.min_size > 0.0 {
            params.min_size
        } else {
            1e-4 * diag
        },
        big: 1e3 * diag,
        iter: 0,
        trace: std::env::var("KW_MESH_TRACE").is_ok(),
        bound: params.radius_edge.max(SAFE_RADIUS_EDGE),
        frozen: false,
    };
    for (i, p) in m.surf.nodes.clone().iter().enumerate() {
        m.vidx.insert(GeomWithData::new(*p, i as u32));
    }
    let ids: Vec<u32> = m.surf.alive_tris().collect();
    for &t in &ids {
        m.register_tri(t);
        m.sq.push_back(t);
        let tr = m.surf.tris[t as usize];
        for i in 0..3 {
            let (a, b) = (tr[i], tr[(i + 1) % 3]);
            m.flip_stack.push((a, b));
            if a < b && m.is_feature_edge(a, b) {
                m.register_seg(a, b);
                m.segq.push_back((a, b));
            }
        }
    }
    m.run_surface()?;
    if m.trace {
        eprintln!(
            "surface done: {} vertices, {} triangles, {} feature segments",
            m.surf.nodes.len(),
            m.surf.n_alive,
            m.feat.len()
        );
    }
    m.build_domain()?;
    if m.trace {
        eprintln!(
            "domain built: {} tets",
            m.dom.as_ref().map_or(0, |d| d.len())
        );
    }
    m.run_volume()?;
    let (mesh, mut rep) = m.finish();
    rep.millis = t0.elapsed().as_secs_f64() * 1e3;
    Ok((mesh, rep))
}

impl Mesher {
    fn h_at(&self, p: &P3) -> f64 {
        let mut h = if self.p.max_size > 0.0 {
            self.p.max_size
        } else {
            f64::INFINITY
        };
        if let Some(f) = &self.p.size {
            h = h.min(f(*p));
        }
        h
    }

    fn check_stop(&mut self) -> Result<bool, MeshError> {
        self.iter += 1;
        if self.trace && self.iter.is_multiple_of(20000) {
            eprintln!(
                "iter {} verts {} tris {} dom {} queues: segq {} fseg {} sq {} forced {} heap {} | splits {} seg {} interior {} floor {} failed {}",
                self.iter,
                self.surf.nodes.len(),
                self.surf.n_alive,
                self.dom.is_some(),
                self.segq.len(),
                self.forced_seg.len(),
                self.sq.len(),
                self.forced.len(),
                self.heap.len(),
                self.rep.surface_splits,
                self.rep.segment_splits,
                self.rep.interior_points,
                self.rep.floor_hits,
                self.rep.failed_inserts
            );
        }
        if self.iter.is_multiple_of(256) {
            if let Some(c) = &self.p.cancel {
                if c.is_cancelled() {
                    return Err(MeshError::Cancelled);
                }
            }
        }
        if self.surf.nodes.len() >= self.p.max_vertices
            || self.iter > 100 * self.p.max_vertices + 1_000_000
        {
            self.rep.budget_hit = true;
            return Ok(true);
        }
        Ok(false)
    }

    // ---- triangle and segment balls ------------------------------------------------------------

    fn register_tri(&mut self, t: u32) {
        let (c, r) = self.surf.ball(t);
        let r = if r.is_finite() {
            r.min(self.big)
        } else {
            self.big
        };
        let b = Ball { id: t, c, r };
        if self.balls.len() <= t as usize {
            self.balls.resize(
                t as usize + 1,
                Ball {
                    id: u32::MAX,
                    c: [0.0; 3],
                    r: 0.0,
                },
            );
        }
        self.balls[t as usize] = b;
        self.tball.insert(b);
    }

    fn unregister_tri(&mut self, t: u32) {
        let b = self.balls[t as usize];
        self.tball.remove(&b);
    }

    fn seg_ball(&self, u: u32, v: u32) -> SegBall {
        let (u, v) = key2(u, v);
        let (pu, pv) = (self.surf.pos(u), self.surf.pos(v));
        SegBall {
            u,
            v,
            c: [
                0.5 * (pu[0] + pv[0]),
                0.5 * (pu[1] + pv[1]),
                0.5 * (pu[2] + pv[2]),
            ],
            r: 0.5 * norm(sub(pu, pv)),
        }
    }

    fn register_seg(&mut self, u: u32, v: u32) {
        if self.feat.insert(key2(u, v)) {
            let b = self.seg_ball(u, v);
            self.sball.insert(b);
        }
    }

    fn unregister_seg(&mut self, u: u32, v: u32) {
        if self.feat.remove(&key2(u, v)) {
            let b = self.seg_ball(u, v);
            self.sball.remove(&b);
        }
    }

    fn is_feature_edge(&self, u: u32, v: u32) -> bool {
        let Some((t1, t2)) = self.surf.edge_tris(u, v) else {
            return false;
        };
        if self.surf.tag[t1 as usize] != self.surf.tag[t2 as usize] {
            return true;
        }
        let n = |t: u32| {
            let c = self.surf.corners(t);
            cross(sub(c[1], c[0]), sub(c[2], c[0]))
        };
        let (n1, n2) = (n(t1), n(t2));
        let (l1, l2) = (norm(n1), norm(n2));
        if l1 == 0.0 || l2 == 0.0 {
            return false;
        }
        dot(n1, n2) / (l1 * l2) < self.p.feature_angle_deg.to_radians().cos()
    }

    /// Live triangles whose diametral ball strictly contains `p`.
    fn balls_containing(&self, p: &P3) -> Vec<u32> {
        self.tball
            .locate_in_envelope_intersecting(&AABB::from_point(*p))
            .filter(|b| {
                let d = sub(*p, b.c);
                dot(d, d) < b.r * b.r * (1.0 - ENCROACH_TOL)
            })
            .map(|b| b.id)
            .collect()
    }

    /// Feature segments whose diametral ball strictly contains `p`.
    fn segs_containing(&self, p: &P3) -> Vec<(u32, u32)> {
        self.sball
            .locate_in_envelope_intersecting(&AABB::from_point(*p))
            .filter(|b| {
                let d = sub(*p, b.c);
                dot(d, d) < b.r * b.r * (1.0 - ENCROACH_TOL)
            })
            .map(|b| (b.u, b.v))
            .collect()
    }

    fn seg_encroached(&self, u: u32, v: u32) -> bool {
        let b = self.seg_ball(u, v);
        self.vidx
            .locate_within_distance(b.c, b.r * b.r * (1.0 - ENCROACH_TOL))
            .any(|g| g.data != u && g.data != v)
    }

    /// Is some vertex inside the diametral ball of triangle `t`?
    ///
    /// While the surface is being made Delaunay (before the solid exists) every vertex counts: vertices on
    /// the sphere count when they are off the plane of the triangle (a tie between faces would otherwise
    /// leave the triangle out of the Delaunay triangulation), coplanar cocircular ones do not (regular
    /// grids on planar faces) and coplanar ones strictly inside are cured by Lawson flips.
    /// Afterwards boundary faces are protected by construction, so only *interior* vertices count: they
    /// are what makes flat tets against the boundary. Vertices of neighbouring boundary facets would
    /// otherwise force refinement of any curved surface far below the size it needs.
    fn encroached(&self, t: u32) -> bool {
        let b = self.balls[t as usize];
        let tr = self.surf.tris[t as usize];
        if self.dom.is_some() {
            return self
                .vint
                .locate_within_distance(b.c, b.r * b.r * (1.0 - ENCROACH_TOL))
                .next()
                .is_some();
        }
        let c = self.surf.corners(t);
        let n = cross(sub(c[1], c[0]), sub(c[2], c[0]));
        let nl = norm(n);
        // vertices of neighbouring facets of a smooth surface (and of a surface whose new vertices are
        // projected onto the true curved geometry) lie a little off the plane of the triangle: they belong
        // to the same patch and are the business of the flips, not of encroachment
        let slab = SLAB * b.r;
        self.vidx
            .locate_within_distance(b.c, b.r * b.r * (1.0 + ENCROACH_TOL))
            .any(|g| {
                if tr.contains(&g.data) {
                    return false;
                }
                let q = *g.geom();
                let d = sub(q, b.c);
                let inside = dot(d, d) < b.r * b.r * (1.0 - ENCROACH_TOL);
                let off_plane = nl > 0.0 && dot(n, sub(q, c[0])).abs() / nl > slab;
                off_plane || inside
            })
    }

    fn tri_is_bad(&self, t: u32) -> bool {
        let b = self.balls[t as usize];
        let c = self.surf.corners(t);
        let centroid = [
            (c[0][0] + c[1][0] + c[2][0]) / 3.0,
            (c[0][1] + c[1][1] + c[2][1]) / 3.0,
            (c[0][2] + c[1][2] + c[2][2]) / 3.0,
        ];
        let h = self.h_at(&centroid);
        if h.is_finite() && b.r > 0.75 * h {
            return true;
        }
        self.encroached(t)
    }

    /// Smallest edge quality-driven refinement may create near `p`.
    fn floor_at(&self, p: &P3) -> f64 {
        if self.dom.is_some() {
            let h = self.h_at(p);
            if h.is_finite() {
                return self.min_size.max(LOCAL_FLOOR * h);
            }
        }
        self.min_size
    }

    fn splittable(&self, t: u32) -> bool {
        let (u, v, l) = self.surf.longest_edge(t);
        l >= 2.0 * self.floor_at(&mid_point(self.surf.pos(u), self.surf.pos(v)))
    }

    fn seg_splittable(&self, u: u32, v: u32) -> bool {
        let (pu, pv) = (self.surf.pos(u), self.surf.pos(v));
        norm(sub(pu, pv)) >= 2.0 * self.floor_at(&mid_point(pu, pv))
    }

    /// Lawson flips: edges between coplanar triangles of one face whose opposite vertex lies inside the
    /// other triangle's circumcircle. Only before the solid exists.
    fn lawson(&mut self) {
        if self.dom.is_some() {
            self.flip_stack.clear();
            return;
        }
        while let Some((a, b)) = self.flip_stack.pop() {
            if self.feat.contains(&key2(a, b)) {
                continue;
            }
            let Some((t, u)) = self.surf.edge_tris(a, b) else {
                continue;
            };
            if self.surf.tag[t as usize] != self.surf.tag[u as usize] {
                continue;
            }
            let c = self.surf.apex_of(t, a, b);
            let d = self.surf.apex_of(u, b, a);
            let (pa, pb, pc, pd) = (
                self.surf.pos(a),
                self.surf.pos(b),
                self.surf.pos(c),
                self.surf.pos(d),
            );
            let n1 = cross(sub(pb, pa), sub(pc, pa));
            let n2 = cross(sub(pa, pb), sub(pd, pb));
            let (l1, l2) = (norm(n1), norm(n2));
            if l1 == 0.0 || l2 == 0.0 || dot(n1, n2) < FLIP_COS * l1 * l2 {
                continue;
            }
            let ball = self.balls[t as usize];
            let dd = sub(pd, ball.c);
            if dot(dd, dd) >= ball.r * ball.r * (1.0 - ENCROACH_TOL) {
                continue;
            }
            // the quad must be strictly convex and the flipped pair must keep the orientation of the old one
            let (m1, m2) = (
                cross(sub(pd, pa), sub(pc, pa)),
                cross(sub(pb, pd), sub(pc, pd)),
            );
            let (a1, a2) = (norm(m1), norm(m2));
            if a1 == 0.0
                || a2 == 0.0
                || dot(m1, n1) <= FLIP_COS * a1 * l1
                || dot(m2, n1) <= FLIP_COS * a2 * l1
                || dot(m1, n2) <= FLIP_COS * a1 * l2
                || dot(m2, n2) <= FLIP_COS * a2 * l2
            {
                continue;
            }
            if let Some((old, new)) = self.surf.flip(a, b) {
                for o in old {
                    self.unregister_tri(o);
                }
                for n in new {
                    self.register_tri(n);
                    self.sq.push_back(n);
                }
                for e in [(a, d), (d, b), (b, c), (c, a)] {
                    self.flip_stack.push(e);
                }
                self.rep.flips += 1;
            }
        }
    }

    // ---- splitting -----------------------------------------------------------------------------

    /// Split triangle `t` along its longest edge, first splitting a neighbour whose own longest edge is
    /// longer (Rivara's longest-edge propagation), which keeps the angles of the refined mesh bounded.
    fn split_tri(&mut self, t: u32) -> bool {
        let mut cur = t;
        for _ in 0..64 {
            let (u, v, l) = self.surf.longest_edge(cur);
            let Some((t1, t2)) = self.surf.edge_tris(u, v) else {
                return false;
            };
            let other = if t1 == cur { t2 } else { t1 };
            let lo = self.surf.longest_edge(other).2;
            if lo > l * (1.0 + 1e-9) && self.splittable(other) {
                cur = other;
                continue;
            }
            return self.split_edge(u, v);
        }
        let (u, v, _) = self.surf.longest_edge(cur);
        self.split_edge(u, v)
    }

    fn split_edge(&mut self, u: u32, v: u32) -> bool {
        let (pu, pv) = (self.surf.pos(u), self.surf.pos(v));
        let len = norm(sub(pu, pv));
        if len < 2.0 * self.floor_at(&mid_point(pu, pv)) {
            self.rep.floor_hits += 1;
            return false;
        }
        let Some((t1, t2)) = self.surf.edge_tris(u, v) else {
            return false;
        };
        let mid = [
            0.5 * (pu[0] + pv[0]),
            0.5 * (pu[1] + pv[1]),
            0.5 * (pu[2] + pv[2]),
        ];
        let mut cands: Vec<P3> = Vec::with_capacity(1);
        let (tag1, tag2) = (self.surf.tag[t1 as usize], self.surf.tag[t2 as usize]);
        let _ = (tag1, tag2);
        cands.push(mid);
        let m_id = self.surf.nodes.len() as u32;
        let mut chosen: Option<(P3, Vec<u32>)> = None;
        for q in cands {
            match &mut self.dom {
                Some(dom) => {
                    let vid = dom.add_point(q);
                    debug_assert_eq!(vid, m_id);
                    let fan = dom.fan_of_edge(u, v);
                    if fan.is_empty() {
                        dom.drop_last_point();
                        continue;
                    }
                    match dom.cavity_insert(vid, &fan, &[u, v]) {
                        Ok(new) => {
                            chosen = Some((q, new));
                            break;
                        }
                        Err(_) => dom.drop_last_point(),
                    }
                }
                None => {
                    chosen = Some((q, Vec::new()));
                    break;
                }
            }
        }
        let Some((q, new_tets)) = chosen else {
            self.rep.failed_inserts += 1;
            return false;
        };
        let was_feature = self.feat.contains(&key2(u, v));
        self.surf.nodes.push(q);
        let (old, new) = self.surf.bisect(u, v, m_id);
        for t in old {
            self.unregister_tri(t);
        }
        self.vidx.insert(GeomWithData::new(q, m_id));
        if was_feature {
            self.unregister_seg(u, v);
            self.register_seg(u, m_id);
            self.register_seg(m_id, v);
            self.segq.push_back((u, m_id));
            self.segq.push_back((m_id, v));
            self.rep.segment_splits += 1;
        }
        for t in new {
            self.register_tri(t);
            self.sq.push_back(t);
            let tr = self.surf.tris[t as usize];
            for i in 0..3 {
                self.flip_stack.push((tr[i], tr[(i + 1) % 3]));
            }
        }
        for id in self.balls_containing(&q) {
            self.sq.push_back(id);
        }
        for s in self.segs_containing(&q) {
            self.segq.push_back(s);
        }
        for t in new_tets {
            self.push_tet(t);
        }
        self.rep.surface_splits += 1;
        true
    }

    /// Run the queues of boundary work; returns false when the budget or cancellation stops it.
    fn drain_boundary(&mut self) -> Result<bool, MeshError> {
        loop {
            if self.check_stop()? {
                return Ok(false);
            }
            if let Some((u, v)) = self.forced_seg.pop_front() {
                if self.feat.contains(&key2(u, v)) && self.surf.edge_tris(u, v).is_some() {
                    if self.split_edge(u, v) {
                        self.rep.splits_by[0] += 1;
                    }
                    self.lawson();
                }
                continue;
            }
            if let Some((u, v)) = self.segq.pop_front() {
                if self.feat.contains(&key2(u, v))
                    && self.surf.edge_tris(u, v).is_some()
                    && self.seg_encroached(u, v)
                {
                    if self.split_edge(u, v) {
                        self.rep.splits_by[1] += 1;
                    }
                    self.lawson();
                }
                continue;
            }
            if let Some(t) = self.forced.pop_front() {
                if self.surf.alive[t as usize] {
                    if self.split_tri(t) {
                        self.rep.splits_by[2] += 1;
                    }
                    self.lawson();
                }
                continue;
            }
            if let Some(t) = self.sq.pop_front() {
                if self.surf.alive[t as usize] && self.tri_is_bad(t) {
                    let size_bad = {
                        let b = self.balls[t as usize];
                        let c = self.surf.corners(t);
                        let cen = [
                            (c[0][0] + c[1][0] + c[2][0]) / 3.0,
                            (c[0][1] + c[1][1] + c[2][1]) / 3.0,
                            (c[0][2] + c[1][2] + c[2][2]) / 3.0,
                        ];
                        let h = self.h_at(&cen);
                        h.is_finite() && b.r > 0.75 * h
                    };
                    if self.split_tri(t) {
                        self.rep.splits_by[if size_bad { 3 } else { 4 }] += 1;
                    }
                    self.lawson();
                }
                continue;
            }
            return Ok(true);
        }
    }

    // ---- phase 1: surface only -----------------------------------------------------------------

    fn run_surface(&mut self) -> Result<(), MeshError> {
        self.lawson();
        self.drain_boundary()?;
        Ok(())
    }

    // ---- domain --------------------------------------------------------------------------------

    fn build_domain(&mut self) -> Result<(), MeshError> {
        for attempt in 0..40 {
            let dt = delaunay(&self.surf.nodes);
            let faces = dt_face_set(&dt);
            let flips = self.flip_to_match(&faces);
            if flips > 0 {
                if self.trace {
                    eprintln!("attempt {attempt}: {flips} edge flips to match the triangulation");
                }
                self.run_surface()?;
                if self.surf.nodes.len() + 4 != dt.pts.len() {
                    // flipped triangles had to be split: the triangulation is stale
                    continue;
                }
            }
            match make_domain(&self.surf, &dt) {
                Ok(dom) => {
                    self.dom = Some(dom);
                    return Ok(());
                }
                Err(DomainError::Missing(list)) => {
                    if self.trace {
                        eprintln!(
                            "attempt {attempt}: {} missing faces of {}",
                            list.len(),
                            self.surf.n_alive
                        );
                        for &t in list.iter().take(3) {
                            let c = self.surf.corners(t);
                            let (u, v, l) = self.surf.longest_edge(t);
                            let n = cross(sub(c[1], c[0]), sub(c[2], c[0]));
                            eprintln!(
                                "   missing tri {t}: {:?} longest edge {l:.3e} ({u}-{v}) area {:.3e} tag {} feature edges {:?} ball r {:.3e}",
                                c,
                                0.5 * norm(n),
                                self.surf.tag[t as usize],
                                (0..3)
                                    .map(|i| self.feat.contains(&key2(self.surf.tris[t as usize][i], self.surf.tris[t as usize][(i + 1) % 3])))
                                    .collect::<Vec<_>>(),
                                self.balls[t as usize].r
                            );
                        }
                    }
                    if attempt == 39 {
                        return Err(MeshError::Conformity(format!(
                            "{} surface triangles are not faces of the Delaunay triangulation",
                            list.len()
                        )));
                    }
                    let mut any = false;
                    for t in list {
                        if self.surf.alive[t as usize] {
                            any |= self.split_tri(t);
                        }
                    }
                    if !any {
                        return Err(MeshError::Conformity(
                            "surface triangles missing from the triangulation and too small to split".into(),
                        ));
                    }
                    self.run_surface()?;
                }
                Err(DomainError::Leak(s)) => return Err(MeshError::Conformity(s)),
            }
        }
        unreachable!()
    }

    /// Flip coplanar edges whose current diagonal is not in the Delaunay triangulation (a tie between
    /// the two diagonals of a cocircular quad) when the other diagonal is. Returns the number of flips.
    fn flip_to_match(&mut self, faces: &HashSet<[u32; 3]>) -> usize {
        let mut total = 0;
        loop {
            let mut did = 0;
            let ids: Vec<u32> = self.surf.alive_tris().collect();
            for t in ids {
                if !self.surf.alive[t as usize]
                    || faces.contains(&sorted3(self.surf.tris[t as usize]))
                {
                    continue;
                }
                let tr = self.surf.tris[t as usize];
                for i in 0..3 {
                    let (a, b) = (tr[i], tr[(i + 1) % 3]);
                    if self.feat.contains(&key2(a, b)) {
                        continue;
                    }
                    let Some((_, u)) = self.surf.edge_tris(a, b) else {
                        continue;
                    };
                    if self.surf.tag[u as usize] != self.surf.tag[t as usize] {
                        continue;
                    }
                    let c = tr[(i + 2) % 3];
                    let d = self.surf.apex_of(u, b, a);
                    // coplanar quad: the normals of the two triangles agree
                    let (pa, pb, pc, pd) = (
                        self.surf.pos(a),
                        self.surf.pos(b),
                        self.surf.pos(c),
                        self.surf.pos(d),
                    );
                    let n1 = cross(sub(pb, pa), sub(pc, pa));
                    let n2 = cross(sub(pa, pb), sub(pd, pb));
                    let (l1, l2) = (norm(n1), norm(n2));
                    if l1 == 0.0 || l2 == 0.0 || dot(n1, n2) / (l1 * l2) < FLIP_COS {
                        continue;
                    }
                    if !faces.contains(&sorted3([a, d, c])) || !faces.contains(&sorted3([d, b, c]))
                    {
                        continue;
                    }
                    if let Some((old, new)) = self.surf.flip(a, b) {
                        for o in old {
                            self.unregister_tri(o);
                        }
                        for n in new {
                            self.register_tri(n);
                            self.sq.push_back(n);
                        }
                        did += 1;
                        break;
                    }
                }
            }
            total += did;
            if did == 0 {
                return total;
            }
        }
    }

    // ---- phase 2: interior ---------------------------------------------------------------------

    fn tet_geom(&self, t: u32) -> (P3, f64, f64, P3) {
        let dom = self.dom.as_ref().expect("domain");
        let v = |k: usize| *dom.corner(t, k);
        let (a, b, c, d) = (v(0), v(1), v(2), v(3));
        let (center, r) = circumsphere(a, b, c, d);
        let mut short = f64::INFINITY;
        for (p, q) in [(a, b), (a, c), (a, d), (b, c), (b, d), (c, d)] {
            short = short.min(norm(sub(p, q)));
        }
        let centroid = [
            0.25 * (a[0] + b[0] + c[0] + d[0]),
            0.25 * (a[1] + b[1] + c[1] + d[1]),
            0.25 * (a[2] + b[2] + c[2] + d[2]),
        ];
        (center, r, short, centroid)
    }

    fn tet_key(&self, r: f64, short: f64, centroid: &P3) -> f64 {
        let mut key = r / short / self.bound;
        let h = self.h_at(centroid);
        if h.is_finite() {
            key = key.max(r / (0.75 * h));
        }
        key
    }

    fn push_tet(&mut self, t: u32) {
        let (_, r, short, cen) = self.tet_geom(t);
        let key = self.tet_key(r, short, &cen);
        if key > 1.0 && r.is_finite() {
            self.heap.push(HeapItem { key, t });
        }
    }

    fn run_volume(&mut self) -> Result<(), MeshError> {
        self.requeue_all_tets();
        loop {
            if !self.drain_boundary()? {
                return Ok(());
            }
            let Some(item) = self.heap.pop() else { break };
            self.process_tet(item.t);
        }
        if self.p.radius_edge < self.bound {
            // stage B: approach the target bound with the boundary frozen
            if self.trace {
                eprintln!(
                    "stage A done: {} vertices; polishing towards radius-edge {}",
                    self.surf.nodes.len(),
                    self.p.radius_edge
                );
            }
            self.bound = self.p.radius_edge;
            self.frozen = true;
            self.requeue_all_tets();
            loop {
                if self.check_stop()? {
                    break;
                }
                let Some(item) = self.heap.pop() else { break };
                self.process_tet(item.t);
            }
        }
        Ok(())
    }

    fn requeue_all_tets(&mut self) {
        self.heap.clear();
        let ids: Vec<u32> = {
            let dom = self.dom.as_ref().expect("domain");
            (0..dom.tets.len() as u32)
                .filter(|&t| dom.tets[t as usize].alive)
                .collect()
        };
        for t in ids {
            self.push_tet(t);
        }
    }

    fn process_tet(&mut self, t: u32) {
        if !self.dom.as_ref().expect("domain").tets[t as usize].alive {
            return;
        }
        let (center, r, short, cen) = self.tet_geom(t);
        let key = self.tet_key(r, short, &cen);
        if key <= 1.0 || !r.is_finite() {
            return;
        }
        if short < self.floor_at(&cen) || r < self.floor_at(&cen) {
            self.rep.floor_hits += 1;
            return;
        }
        if r > 1e6 * short {
            // numerically flat: nothing sensible can be done with it
            self.rep.degenerate += 1;
            return;
        }
        // The circumcentre, pulled back towards the centroid (which is inside the tet and inside the
        // circumball, so the tet is still removed) until it is inside the solid, out of every boundary
        // segment's and triangle's diametral ball, and inserted without creating flat tets.
        let limit = match self.first_crossing(&cen, &center) {
            Some(s) => 0.9 * s,
            None => 1.0,
        };
        let at = |s: f64| {
            [
                cen[0] + s * (center[0] - cen[0]),
                cen[1] + s * (center[1] - cen[1]),
                cen[2] + s * (center[2] - cen[2]),
            ]
        };
        let mut s = limit;
        let mut encroaching: Option<(Vec<u32>, Vec<(u32, u32)>)> = None;
        for _ in 0..16 {
            let p = at(s);
            let tris = self.balls_containing(&p);
            let segs = self.segs_containing(&p);
            if !tris.is_empty() || !segs.is_empty() {
                encroaching = Some((tris, segs));
                s *= 0.7;
                continue;
            }
            encroaching = None;
            let loc = self.dom.as_mut().expect("domain").locate(&p, t);
            match loc {
                Loc::Inside(_) | Loc::OnFace(..) | Loc::OnEdge(..) => {
                    let dom = self.dom.as_mut().expect("domain");
                    let vid = dom.add_point(p);
                    match dom.insert_at_guarded(vid, loc, FLAT_TOL) {
                        Ok(new) => {
                            debug_assert_eq!(vid as usize, self.surf.nodes.len());
                            self.surf.nodes.push(p);
                            self.vidx.insert(GeomWithData::new(p, vid));
                            self.vint.insert(GeomWithData::new(p, vid));
                            self.rep.interior_points += 1;
                            for n in new {
                                self.push_tet(n);
                            }
                            return;
                        }
                        Err(_) => {
                            dom.drop_last_point();
                            s *= 0.6;
                        }
                    }
                }
                Loc::Blocked(..) | Loc::OnVertex(..) | Loc::Outside => s *= 0.6,
            }
        }
        match encroaching {
            Some((tris, segs)) if !self.frozen => {
                // A point that lies *in* the plane of a boundary triangle (or on a segment) is inside
                // that triangle's ball however small the triangle gets, and splitting it again and again
                // never helps: circumcentres of symmetric configurations land exactly there. Only
                // genuine encroachment is answered by refining the boundary.
                let genuine_tris: Vec<u32> = tris
                    .into_iter()
                    .filter(|&f| !self.on_plane_of(f, &at(s)))
                    .collect();
                let genuine_segs: Vec<(u32, u32)> = segs
                    .into_iter()
                    .filter(|&(u, v)| !self.on_segment(u, v, &at(s)))
                    .collect();
                let mut any = false;
                for (u, v) in genuine_segs {
                    if self.seg_splittable(u, v) {
                        self.forced_seg.push_back((u, v));
                        any = true;
                    }
                }
                if !any {
                    for f in genuine_tris {
                        if self.splittable(f) {
                            self.forced.push_back(f);
                            any = true;
                        }
                    }
                }
                if any {
                    self.heap.push(HeapItem { key, t });
                } else {
                    self.rep.degenerate += 1;
                }
            }
            Some(_) => self.rep.unfixed += 1,
            None => self.rep.failed_inserts += 1,
        }
    }

    /// Does `p` lie in the slab round the plane of triangle `f`? Then it is on the surface as far as
    /// this triangle is concerned (a cap tet of a curved wall has its centroid within the sagitta of the
    /// wall) and no refinement of the triangle can take it out of the ball.
    fn on_plane_of(&self, f: u32, p: &P3) -> bool {
        let c = self.surf.corners(f);
        let n = cross(sub(c[1], c[0]), sub(c[2], c[0]));
        let nl = norm(n);
        nl > 0.0 && dot(n, sub(*p, c[0])).abs() / nl <= SLAB * self.balls[f as usize].r
    }

    /// Does `p` lie on the line of segment `u`–`v` (to within rounding)?
    fn on_segment(&self, u: u32, v: u32, p: &P3) -> bool {
        let (pu, pv) = (self.surf.pos(u), self.surf.pos(v));
        let d = sub(pv, pu);
        let l = norm(d);
        if l == 0.0 {
            return true;
        }
        let w = sub(*p, pu);
        norm(cross(d, w)) / l <= 1e-7 * 0.5 * l
    }

    /// Parameter at which the segment `a`→`b` first crosses a boundary triangle.
    fn first_crossing(&self, a: &P3, b: &P3) -> Option<f64> {
        let lo = [a[0].min(b[0]), a[1].min(b[1]), a[2].min(b[2])];
        let hi = [a[0].max(b[0]), a[1].max(b[1]), a[2].max(b[2])];
        let dir = sub(*b, *a);
        let mut best: Option<f64> = None;
        for ball in self
            .tball
            .locate_in_envelope_intersecting(&AABB::from_corners(lo, hi))
        {
            let c = self.surf.corners(ball.id);
            if let Some(s) = segment_triangle(a, &dir, &c) {
                if best.is_none_or(|bs| s < bs) {
                    best = Some(s);
                }
            }
        }
        best
    }

    // ---- output --------------------------------------------------------------------------------

    fn finish(mut self) -> (TetMesh, MeshReport) {
        let dom = self.dom.take().expect("domain");
        let mut tets: Vec<[u32; 4]> = Vec::with_capacity(dom.len());
        let mut surface: Vec<SurfaceTri> = Vec::new();
        for (id, t) in dom.tets.iter().enumerate() {
            if !t.alive {
                continue;
            }
            tets.push(t.v);
            for i in 0..4 {
                if t.n[i] == NONE {
                    let f = dom.face_ids(id as u32, i);
                    let tag = self
                        .surf
                        .tri_by_key(&sorted3(f))
                        .map_or(0, |s| self.surf.tag[s as usize]);
                    // the face normal points into the solid; flip it
                    surface.push(SurfaceTri {
                        v: [f[0], f[2], f[1]],
                        tag,
                    });
                }
            }
        }
        let mut nodes = dom.pts.clone();
        let mut boundary = vec![false; nodes.len()];
        for s in &surface {
            for &v in &s.v {
                boundary[v as usize] = true;
            }
        }
        // flips and smoothing take turns: a flip changes which vertices pull on each other, a move
        // changes which flips pay
        for round in 0..3 {
            if round == 1 {
                self.rep.snapped = crate::snap::snap_boundary(
                    &mut nodes,
                    &tets,
                    &surface,
                    self.p.project.as_deref(),
                    self.p.project_edge.as_deref(),
                );
            }
            let st = crate::improve::improve(&nodes, &mut tets, 0.6, 8);
            self.rep.flips_volume += st.edge_removals + st.face_removals;
            crate::smooth::smooth(&mut nodes, &tets, &boundary, self.p.smooth_passes);
        }
        let q = QualityReport::of(&nodes, &tets);
        let (dmin, dmax) = crate::smooth::dihedral_range(&nodes, &tets);
        self.rep.vertices = nodes.len();
        self.rep.tets = tets.len();
        self.rep.quality = Some(q);
        self.rep.min_dihedral_deg = dmin;
        self.rep.max_dihedral_deg = dmax;
        (
            TetMesh {
                nodes,
                tets,
                surface,
            },
            self.rep,
        )
    }
}

/// Circumsphere of a tetrahedron (centre, radius); infinite radius for a flat tet.
pub fn circumsphere(a: P3, b: P3, c: P3, d: P3) -> (P3, f64) {
    let (u, v, w) = (sub(b, a), sub(c, a), sub(d, a));
    let det = dot(u, cross(v, w));
    if det.abs() < 1e-300 {
        return (a, f64::INFINITY);
    }
    let (u2, v2, w2) = (dot(u, u), dot(v, v), dot(w, w));
    let (vw, wu, uv) = (cross(v, w), cross(w, u), cross(u, v));
    let x = [
        (u2 * vw[0] + v2 * wu[0] + w2 * uv[0]) / (2.0 * det),
        (u2 * vw[1] + v2 * wu[1] + w2 * uv[1]) / (2.0 * det),
        (u2 * vw[2] + v2 * wu[2] + w2 * uv[2]) / (2.0 * det),
    ];
    ([a[0] + x[0], a[1] + x[1], a[2] + x[2]], norm(x))
}

/// Parameter `s ∈ [0,1]` at which the segment `a + s·dir` crosses the triangle, if it does.
fn segment_triangle(a: &P3, dir: &P3, c: &[P3; 3]) -> Option<f64> {
    let e1 = sub(c[1], c[0]);
    let e2 = sub(c[2], c[0]);
    let p = cross(*dir, e2);
    let det = dot(e1, p);
    if det.abs() < 1e-300 {
        return None;
    }
    let inv = 1.0 / det;
    let s = sub(*a, c[0]);
    let u = dot(s, p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = dot(*dir, q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = dot(e2, q) * inv;
    (0.0..=1.0).contains(&t).then_some(t)
}

fn cyclic_same(f: [u32; 3], s: [u32; 3]) -> bool {
    f == s || f == [s[1], s[2], s[0]] || f == [s[2], s[0], s[1]]
}

/// Sorted vertex triples of every face of the triangulation.
fn dt_face_set(dt: &Tri3) -> HashSet<[u32; 3]> {
    let mut out = HashSet::with_capacity(dt.len() * 2);
    for (id, t) in dt.tets.iter().enumerate() {
        if t.alive {
            for i in 0..4 {
                out.insert(sorted3(dt.face_ids(id as u32, i)));
            }
        }
    }
    out
}

/// Cut the Delaunay triangulation of the surface vertices down to the solid.
fn make_domain(surf: &Surf, dt: &Tri3) -> Result<Tri3, DomainError> {
    let n = surf.nodes.len();
    let mut found = vec![false; surf.tris.len()];
    let mut seeds: Vec<u32> = Vec::new();
    for (id, t) in dt.tets.iter().enumerate() {
        if !t.alive {
            continue;
        }
        for i in 0..4 {
            let f = dt.face_ids(id as u32, i);
            if let Some(tri) = surf.tri_by_key(&sorted3(f)) {
                found[tri as usize] = true;
                if !cyclic_same(f, surf.tris[tri as usize]) {
                    seeds.push(id as u32);
                }
            }
        }
    }
    let missing: Vec<u32> = surf.alive_tris().filter(|&t| !found[t as usize]).collect();
    if !missing.is_empty() {
        return Err(DomainError::Missing(missing));
    }
    let mut inside = vec![false; dt.tets.len()];
    let mut stack: Vec<u32> = Vec::new();
    for s in seeds {
        if !inside[s as usize] {
            inside[s as usize] = true;
            stack.push(s);
        }
    }
    while let Some(t) = stack.pop() {
        let tet = dt.tets[t as usize];
        if tet.v.iter().any(|&v| v as usize >= n) {
            return Err(DomainError::Leak(
                "the inside of the surface connects to the outside (surface not watertight)".into(),
            ));
        }
        for i in 0..4 {
            let nb = tet.n[i];
            if nb == NONE || inside[nb as usize] {
                continue;
            }
            if surf.tri_by_key(&sorted3(dt.face_ids(t, i))).is_some() {
                continue;
            }
            inside[nb as usize] = true;
            stack.push(nb);
        }
    }
    let tets: Vec<[u32; 4]> = dt
        .tets
        .iter()
        .enumerate()
        .filter(|(i, t)| t.alive && inside[*i])
        .map(|(_, t)| t.v)
        .collect();
    let dom = Tri3::from_tets(surf.nodes.clone(), &tets);
    let boundary = dom
        .tets
        .iter()
        .filter(|t| t.alive)
        .map(|t| t.n.iter().filter(|&&x| x == NONE).count())
        .sum::<usize>();
    if boundary != surf.n_alive {
        return Err(DomainError::Leak(format!(
            "the cut-out solid has {boundary} boundary faces but the surface has {}",
            surf.n_alive
        )));
    }
    Ok(dom)
}
