//! Parameter-space geometry of faces: loop polylines, bounding boxes, containment.

use crate::ids::*;
use crate::store::BRep;
use crate::topology::*;
use kw_math::Aabb;
use kw_nurbs::{Curve3, CurveGeom, Surface, SurfaceGeom};

pub type UV = (f64, f64);

#[derive(Debug, Clone)]
pub struct UvLoop {
    pub kind: LoopKind,
    pub points: Vec<UV>,
}

/// Number of samples for a pcurve segment.
fn samples_for(c: &CurveGeom, a: f64, b: f64) -> usize {
    match c {
        CurveGeom::Line(_) => 2,
        CurveGeom::Circle(_) | CurveGeom::Ellipse(_) => ((b - a).abs() / 0.1).ceil() as usize + 2,
        CurveGeom::Nurbs(n) => (n.knots().distinct().len().max(2) - 1) * 6 + 2,
        CurveGeom::Procedural(_) => 32,
    }
}

impl BRep {
    /// Points of the pcurve of `co` in traversal order, `n_extra` times denser than the default.
    pub fn coedge_uv_points(&self, co: CoedgeId, density: usize) -> Option<Vec<UV>> {
        let c = self.coedges.get(co)?;
        let pc = self.curves.get(c.pcurve?)?;
        let (t0, t1) = self.coedge_param_range(co)?;
        let n = samples_for(pc, t0, t1) * density.max(1);
        Some(
            (0..=n)
                .map(|i| {
                    let p = pc.point_at(t0 + (t1 - t0) * i as f64 / n as f64);
                    (p.x, p.y)
                })
                .collect(),
        )
    }

    /// The loop as a closed polyline in uv (last point not repeated).
    pub fn loop_uv(&self, lp: LoopId, density: usize) -> Option<UvLoop> {
        let kind = self.loops.get(lp)?.kind;
        let mut points: Vec<UV> = Vec::new();
        for co in self.loop_coedges(lp) {
            let pts = self.coedge_uv_points(co, density)?;
            let n = pts.len();
            points.extend(pts.into_iter().take(n - 1));
        }
        Some(UvLoop { kind, points })
    }

    pub fn face_uv_loops(&self, f: FaceId, density: usize) -> Option<Vec<UvLoop>> {
        self.faces
            .get(f)?
            .loops
            .iter()
            .map(|l| self.loop_uv(*l, density))
            .collect()
    }

    /// Parameter bounding box of all loops of the face.
    pub fn face_uv_bbox(&self, f: FaceId) -> Option<(UV, UV)> {
        loops_bbox(&self.face_uv_loops(f, 2)?)
    }

    /// Conservative 3D bounding box of a face: boundary samples plus interior grid, inflated by the
    /// sampling error bound.
    pub fn face_bbox(&self, f: FaceId) -> Option<Aabb> {
        let loops = self.face_uv_loops(f, 3)?;
        let (lo, hi) = self.face_uv_bbox(f)?;
        let index = LoopIndex::new(&loops);
        self.face_bbox_with(f, &loops, &index, lo, hi)
    }

    /// [`face_bbox`](Self::face_bbox) from already computed parameter loops, their index and bounding box.
    pub fn face_bbox_with(
        &self,
        f: FaceId,
        loops: &[UvLoop],
        index: &LoopIndex,
        lo: UV,
        hi: UV,
    ) -> Option<Aabb> {
        let face = self.faces.get(f)?;
        let surf = self.surfaces.get(face.surface)?;
        let mut bb = Aabb::EMPTY;
        for l in loops {
            for p in &l.points {
                bb.extend(&surf.point_at(p.0, p.1));
            }
        }
        // a plane is bounded by its boundary: no interior samples needed
        if matches!(surf, SurfaceGeom::Plane(_)) {
            let pad = 1e-9 + 1e-9 * bb.diagonal();
            bb.min -= kw_math::Vector3::repeat(pad);
            bb.max += kw_math::Vector3::repeat(pad);
            return Some(bb);
        }
        // interior samples
        let n = 24;
        for i in 0..=n {
            for j in 0..=n {
                let u = lo.0 + (hi.0 - lo.0) * i as f64 / n as f64;
                let v = lo.1 + (hi.1 - lo.1) * j as f64 / n as f64;
                if index.contains((u, v)) {
                    bb.extend(&surf.point_at(u, v));
                }
            }
        }
        // inflate for curvature between samples
        let diag = bb.diagonal();
        let pad = 0.02 * diag + 1e-9;
        bb.min -= kw_math::Vector3::repeat(pad);
        bb.max += kw_math::Vector3::repeat(pad);
        Some(bb)
    }

    pub fn body_bbox(&self, body: BodyId) -> Aabb {
        let mut bb = Aabb::EMPTY;
        for f in self.body_faces(body) {
            if let Some(b) = self.face_bbox(f) {
                bb = bb.union(&b);
            }
        }
        bb
    }
}

/// Parameter bounding box of a set of loops.
pub fn loops_bbox(loops: &[UvLoop]) -> Option<(UV, UV)> {
    let mut lo = (f64::INFINITY, f64::INFINITY);
    let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for l in loops {
        for p in &l.points {
            lo = (lo.0.min(p.0), lo.1.min(p.1));
            hi = (hi.0.max(p.0), hi.1.max(p.1));
        }
    }
    (lo.0.is_finite()).then_some((lo, hi))
}

/// Winding number of `p` with respect to a closed polyline.
pub fn winding(p: UV, poly: &[UV]) -> i32 {
    let mut w = 0;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if a.1 <= p.1 {
            if b.1 > p.1 && cross(a, b, p) > 0.0 {
                w += 1;
            }
        } else if b.1 <= p.1 && cross(a, b, p) < 0.0 {
            w -= 1;
        }
    }
    w
}

fn cross(a: UV, b: UV, p: UV) -> f64 {
    (b.0 - a.0) * (p.1 - a.1) - (p.0 - a.0) * (b.1 - a.1)
}

/// Is `p` inside the region bounded by the loops (non-zero total winding)?
pub fn point_in_loops(p: UV, loops: &[UvLoop]) -> bool {
    loops.iter().map(|l| winding(p, &l.points)).sum::<i32>() != 0
}

/// Distance from `p` to the nearest loop segment (parameter space).
pub fn distance_to_loops(p: UV, loops: &[UvLoop]) -> f64 {
    let mut best = f64::INFINITY;
    for l in loops {
        let n = l.points.len();
        for i in 0..n {
            let (a, b) = (l.points[i], l.points[(i + 1) % n]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            let t = if len2 > 0.0 {
                (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let q = (a.0 + t * dx, a.1 + t * dy);
            best = best.min((p.0 - q.0).hypot(p.1 - q.1));
        }
    }
    best
}

/// Shift `uv` by whole periods of the surface so it is as close as possible to `near`.
pub fn unwrap_uv(surface: &kw_nurbs::SurfaceGeom, uv: UV, near: UV) -> UV {
    let mut r = uv;
    if let Some(p) = surface.period_u() {
        r.0 = kw_nurbs::proj::wrap_near(uv.0, near.0, p);
    }
    if let Some(p) = surface.period_v() {
        r.1 = kw_nurbs::proj::wrap_near(uv.1, near.1, p);
    }
    r
}

/// Spatial index over the segments of a set of uv loops: fast nearest-boundary distance and
/// point-in-region tests for faces with very long boundaries.
#[derive(Debug, Clone)]
pub struct LoopIndex {
    segs: Vec<(UV, UV)>,
    lo: UV,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
    /// y-bins for ray casting
    ybin: f64,
    ny_bins: usize,
    rows: Vec<Vec<u32>>,
}

impl LoopIndex {
    pub fn new(loops: &[UvLoop]) -> Self {
        let mut segs: Vec<(UV, UV)> = Vec::new();
        for l in loops {
            let n = l.points.len();
            for i in 0..n {
                segs.push((l.points[i], l.points[(i + 1) % n]));
            }
        }
        let mut lo = (f64::INFINITY, f64::INFINITY);
        let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for s in &segs {
            for p in [s.0, s.1] {
                lo = (lo.0.min(p.0), lo.1.min(p.1));
                hi = (hi.0.max(p.0), hi.1.max(p.1));
            }
        }
        if segs.is_empty() {
            return Self {
                segs,
                lo: (0.0, 0.0),
                cell: 1.0,
                nx: 1,
                ny: 1,
                cells: vec![Vec::new()],
                ybin: 1.0,
                ny_bins: 1,
                rows: vec![Vec::new()],
            };
        }
        let (w, h) = ((hi.0 - lo.0).max(1e-12), (hi.1 - lo.1).max(1e-12));
        let target = (segs.len() as f64).sqrt().ceil().max(1.0);
        let cell = (w.max(h) / target).max(1e-12);
        let (nx, ny) = (
            ((w / cell).ceil() as usize).clamp(1, 4096),
            ((h / cell).ceil() as usize).clamp(1, 4096),
        );
        let mut cells = vec![Vec::new(); nx * ny];
        for (si, s) in segs.iter().enumerate() {
            let (x0, x1) = (s.0 .0.min(s.1 .0), s.0 .0.max(s.1 .0));
            let (y0, y1) = (s.0 .1.min(s.1 .1), s.0 .1.max(s.1 .1));
            let (i0, i1) = (
                (((x0 - lo.0) / cell) as usize).min(nx - 1),
                (((x1 - lo.0) / cell) as usize).min(nx - 1),
            );
            let (j0, j1) = (
                (((y0 - lo.1) / cell) as usize).min(ny - 1),
                (((y1 - lo.1) / cell) as usize).min(ny - 1),
            );
            for j in j0..=j1 {
                for i in i0..=i1 {
                    cells[j * nx + i].push(si as u32);
                }
            }
        }
        let ny_bins = (segs.len() / 4).clamp(1, 1 << 16);
        let ybin = (h / ny_bins as f64).max(1e-12);
        let mut rows = vec![Vec::new(); ny_bins];
        for (si, s) in segs.iter().enumerate() {
            let (y0, y1) = (s.0 .1.min(s.1 .1), s.0 .1.max(s.1 .1));
            let (j0, j1) = (
                (((y0 - lo.1) / ybin) as usize).min(ny_bins - 1),
                (((y1 - lo.1) / ybin) as usize).min(ny_bins - 1),
            );
            for j in j0..=j1 {
                rows[j].push(si as u32);
            }
        }
        Self {
            segs,
            lo,
            cell,
            nx,
            ny,
            cells,
            ybin,
            ny_bins,
            rows,
        }
    }

    /// Total winding number of `p` about all loops.
    pub fn winding(&self, p: UV) -> i32 {
        if self.segs.is_empty() {
            return 0;
        }
        let j = (((p.1 - self.lo.1) / self.ybin).floor()).clamp(0.0, (self.ny_bins - 1) as f64)
            as usize;
        let mut w = 0;
        for &si in &self.rows[j] {
            let (a, b) = self.segs[si as usize];
            if a.1 <= p.1 {
                if b.1 > p.1 && cross(a, b, p) > 0.0 {
                    w += 1;
                }
            } else if b.1 <= p.1 && cross(a, b, p) < 0.0 {
                w -= 1;
            }
        }
        w
    }

    pub fn contains(&self, p: UV) -> bool {
        self.winding(p) != 0
    }

    /// Is `p` within `r` of the boundary? Cheap for points far from it.
    pub fn near(&self, p: UV, r: f64) -> bool {
        if self.segs.is_empty() {
            return false;
        }
        let hi = (
            self.lo.0 + self.nx as f64 * self.cell,
            self.lo.1 + self.ny as f64 * self.cell,
        );
        if p.0 < self.lo.0 - r || p.0 > hi.0 + r || p.1 < self.lo.1 - r || p.1 > hi.1 + r {
            return false;
        }
        let rings = ((r / self.cell).ceil() as i64 + 1).min(self.nx.max(self.ny) as i64);
        let ci = (((p.0 - self.lo.0) / self.cell).floor()).clamp(0.0, (self.nx - 1) as f64) as i64;
        let cj = (((p.1 - self.lo.1) / self.cell).floor()).clamp(0.0, (self.ny - 1) as f64) as i64;
        for j in (cj - rings).max(0)..=(cj + rings).min(self.ny as i64 - 1) {
            for i in (ci - rings).max(0)..=(ci + rings).min(self.nx as i64 - 1) {
                for &si in &self.cells[(j as usize) * self.nx + i as usize] {
                    let (a, b) = self.segs[si as usize];
                    if seg_distance(p, a, b) <= r {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Distance from `p` to the nearest boundary segment.
    pub fn distance(&self, p: UV) -> f64 {
        if self.segs.is_empty() {
            return f64::INFINITY;
        }
        let ci = (((p.0 - self.lo.0) / self.cell).floor()).clamp(0.0, (self.nx - 1) as f64) as i64;
        let cj = (((p.1 - self.lo.1) / self.cell).floor()).clamp(0.0, (self.ny - 1) as f64) as i64;
        let mut best = f64::INFINITY;
        let max_ring = self.nx.max(self.ny) as i64;
        let visit = |i: i64, j: i64, best: &mut f64| {
            if i < 0 || j < 0 || i >= self.nx as i64 || j >= self.ny as i64 {
                return;
            }
            for &si in &self.cells[(j as usize) * self.nx + i as usize] {
                let (a, b) = self.segs[si as usize];
                *best = best.min(seg_distance(p, a, b));
            }
        };
        for ring in 0..=max_ring {
            // every cell of this ring is at least (ring - 1) cells away
            if best.is_finite() && (ring as f64 - 1.0) * self.cell > best {
                break;
            }
            if ring == 0 {
                visit(ci, cj, &mut best);
                continue;
            }
            // perimeter only
            for i in (ci - ring)..=(ci + ring) {
                visit(i, cj - ring, &mut best);
                visit(i, cj + ring, &mut best);
            }
            for j in (cj - ring + 1)..=(cj + ring - 1) {
                visit(ci - ring, j, &mut best);
                visit(ci + ring, j, &mut best);
            }
        }
        best
    }
}

fn seg_distance(p: UV, a: UV, b: UV) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy)
}
