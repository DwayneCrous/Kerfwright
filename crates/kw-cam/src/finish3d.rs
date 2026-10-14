//! Three-axis finishing by drop-cutter, and the gouge and holder-collision checks for 3-D toolpaths.

use crate::bvh::{Aabb, TriMesh};
use crate::dropcutter::{cylinder_hits_mesh, drop_cutter, z_extent_in_disc};
use crate::path::{Event, Toolpath};
use crate::tool::Tool;
use crate::P3;

#[derive(Debug, Clone)]
pub struct RasterParams {
    /// Tool-centre region of the raster (x0, y0, x1, y1).
    pub region: [f64; 4],
    pub stepover: f64,
    /// Initial sample spacing along a line.
    pub step: f64,
    /// Largest allowed chord sag into the surface between samples.
    pub tolerance: f64,
    pub min_segment: f64,
    /// Tip height used where nothing is within reach (the bottom of the part).
    pub floor: f64,
    pub safe_z: f64,
    pub feed: f64,
    pub plunge_feed: f64,
}

fn height(mesh: &TriMesh, tool: &Tool, x: f64, y: f64, floor: f64) -> f64 {
    drop_cutter(mesh, tool, x, y).unwrap_or(floor).max(floor)
}

/// Sample the line from `(xa, y)` to `(xb, y)` so that no chord of the result cuts into the surface by more
/// than the tolerance.
fn line_points(mesh: &TriMesh, tool: &Tool, xa: f64, xb: f64, y: f64, p: &RasterParams) -> Vec<P3> {
    let n = (((xb - xa).abs() / p.step).ceil() as usize).max(1);
    let mut pts: Vec<P3> = (0..=n)
        .map(|i| {
            let x = xa + (xb - xa) * i as f64 / n as f64;
            [x, y, height(mesh, tool, x, y, p.floor)]
        })
        .collect();
    // refine every chord that cuts into the surface: probe interior points and split at the worst
    let mut i = 0;
    let mut guard = 0;
    while i + 1 < pts.len() && guard < 400_000 {
        guard += 1;
        let (a, b) = (pts[i], pts[i + 1]);
        if (b[0] - a[0]).abs() > p.min_segment {
            let mut worst: Option<(f64, [f64; 3])> = None;
            // probe at about 40 µm spacing (at least seven points)
            let probes = (((b[0] - a[0]).abs() / 0.04).ceil() as usize).clamp(8, 26);
            for k in 1..probes {
                let t = k as f64 / probes as f64;
                let x = a[0] + (b[0] - a[0]) * t;
                let z = height(mesh, tool, x, y, p.floor);
                let sag = z - (a[2] + (b[2] - a[2]) * t);
                if sag > 0.5 * p.tolerance && worst.is_none_or(|w| sag > w.0) {
                    worst = Some((sag, [x, y, z]));
                }
            }
            if let Some((_, q)) = worst {
                pts.insert(i + 1, q);
                continue;
            }
        }
        i += 1;
    }
    pts
}

/// Zig-zag raster finishing along x.
pub fn raster_finish(mesh: &TriMesh, tool: &Tool, p: &RasterParams) -> Toolpath {
    raster_finish_masked(mesh, tool, p, None)
}

/// Where a finishing pass is needed: a grid of cells over a region.
#[derive(Debug, Clone)]
pub struct Mask {
    pub origin: [f64; 2],
    pub cell: f64,
    pub n: [usize; 2],
    bits: Vec<bool>,
}

impl Mask {
    pub fn contains(&self, x: f64, y: f64) -> bool {
        let i = ((x - self.origin[0]) / self.cell).floor();
        let j = ((y - self.origin[1]) / self.cell).floor();
        if i < 0.0 || j < 0.0 || i >= self.n[0] as f64 || j >= self.n[1] as f64 {
            return false;
        }
        self.bits[i as usize * self.n[1] + j as usize]
    }

    /// Number of cells set.
    pub fn count(&self) -> usize {
        self.bits.iter().filter(|b| **b).count()
    }
}

/// The height of the mesh's upper surface at `(x, y)`: the highest point of any triangle over it.
pub fn mesh_height(mesh: &TriMesh, x: f64, y: f64) -> Option<f64> {
    let q = Aabb {
        min: [x, y, f64::NEG_INFINITY],
        max: [x, y, f64::INFINITY],
    };
    let mut best: Option<f64> = None;
    mesh.query(&q, |i| {
        let t = &mesh.tris[i];
        let area =
            (t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[1][1] - t[0][1]) * (t[2][0] - t[0][0]);
        if area.abs() < 1e-14 {
            return;
        }
        let w0 = ((t[1][0] - x) * (t[2][1] - y) - (t[1][1] - y) * (t[2][0] - x)) / area;
        let w1 = ((t[2][0] - x) * (t[0][1] - y) - (t[2][1] - y) * (t[0][0] - x)) / area;
        let w2 = 1.0 - w0 - w1;
        if w0 >= -1e-9 && w1 >= -1e-9 && w2 >= -1e-9 {
            let z = w0 * t[0][2] + w1 * t[1][2] + w2 * t[2][2];
            if best.is_none_or(|b| z > b) {
                best = Some(z);
            }
        }
    });
    best
}

/// Where the stock still stands more than `threshold` above the part, widened by the tool's radius (the
/// tool cuts that far around its centre). `stock` is the stock after the earlier operations.
pub fn remaining_mask(
    stock: &crate::dexel::TriDexel,
    mesh: &TriMesh,
    tool: &Tool,
    threshold: f64,
    region: [f64; 4],
) -> Mask {
    let cell = stock.h;
    let n = [
        (((region[2] - region[0]) / cell).ceil() as usize).max(1),
        (((region[3] - region[1]) / cell).ceil() as usize).max(1),
    ];
    let floor = mesh.bounds().min[2];
    let mut raw = vec![false; n[0] * n[1]];
    let cells = stock.cells();
    for i in 0..n[0] {
        for j in 0..n[1] {
            let (x, y) = (
                region[0] + (i as f64 + 0.5) * cell,
                region[1] + (j as f64 + 0.5) * cell,
            );
            let (si, sj) = (
                ((x - stock.min[0]) / stock.h).floor(),
                ((y - stock.min[1]) / stock.h).floor(),
            );
            if si < 0.0 || sj < 0.0 || si >= cells[0] as f64 || sj >= cells[1] as f64 {
                continue;
            }
            let Some(top) = stock.top_height(si as usize, sj as usize) else {
                continue;
            };
            let part = mesh_height(mesh, x, y).unwrap_or(floor);
            raw[i * n[1] + j] = top - part > threshold;
        }
    }
    // widen by the tool radius
    let reach = (tool.radius() / cell).ceil() as i64;
    let mut bits = vec![false; n[0] * n[1]];
    for i in 0..n[0] as i64 {
        for j in 0..n[1] as i64 {
            if !raw[i as usize * n[1] + j as usize] {
                continue;
            }
            for di in -reach..=reach {
                for dj in -reach..=reach {
                    if (di * di + dj * dj) as f64 > (reach * reach) as f64 {
                        continue;
                    }
                    let (a, b) = (i + di, j + dj);
                    if a >= 0 && b >= 0 && a < n[0] as i64 && b < n[1] as i64 {
                        bits[a as usize * n[1] + b as usize] = true;
                    }
                }
            }
        }
    }
    Mask {
        origin: [region[0], region[1]],
        cell,
        n,
        bits,
    }
}

/// As [`raster_finish`], keeping only the stretches of the lines that lie in `mask`.
pub fn raster_finish_masked(
    mesh: &TriMesh,
    tool: &Tool,
    p: &RasterParams,
    mask: Option<&Mask>,
) -> Toolpath {
    let mut tp = Toolpath::new("raster finish");
    tp.push(Event::ToolChange {
        number: tool.number,
        name: tool.name.clone(),
        diameter: tool.diameter,
        rpm: 9000.0,
    });
    tp.push(Event::SpindleOn {
        rpm: 9000.0,
        clockwise: true,
    });
    let [x0, y0, x1, y1] = p.region;
    let rows = (((y1 - y0) / p.stepover).ceil() as usize).max(1);
    for r in 0..=rows {
        let y = y0 + (y1 - y0) * r as f64 / rows as f64;
        let (xa, xb) = if r % 2 == 0 { (x0, x1) } else { (x1, x0) };
        let pts = line_points(mesh, tool, xa, xb, y, p);
        // the stretches of the line that need cutting
        let runs: Vec<&[P3]> = match mask {
            None => vec![&pts[..]],
            Some(m) => {
                let mut runs = Vec::new();
                let mut start: Option<usize> = None;
                for (k, q) in pts.iter().enumerate() {
                    let on = m.contains(q[0], q[1]);
                    match (on, start) {
                        (true, None) => start = Some(k.saturating_sub(1)),
                        (false, Some(a)) => {
                            runs.push(&pts[a..(k + 1).min(pts.len())]);
                            start = None;
                        }
                        _ => {}
                    }
                }
                if let Some(a) = start {
                    runs.push(&pts[a..]);
                }
                runs
            }
        };
        for run in runs {
            if run.len() < 2 {
                continue;
            }
            let first = run[0];
            tp.push(Event::Rapid {
                p: [first[0], first[1], p.safe_z],
            });
            tp.push(Event::Feed {
                p: first,
                feed: p.plunge_feed,
            });
            for q in &run[1..] {
                tp.push(Event::Feed {
                    p: *q,
                    feed: p.feed,
                });
            }
            let last = *run.last().expect("a point");
            tp.push(Event::Rapid {
                p: [last[0], last[1], p.safe_z],
            });
        }
    }
    tp.push(Event::SpindleOff);
    tp.push(Event::ProgramEnd);
    tp
}

/// All straight moves of a toolpath (rapids included) as `(from, to)`; arcs are chorded.
pub fn moves(tp: &Toolpath) -> Vec<(P3, P3)> {
    let mut out = Vec::new();
    let mut pos: Option<P3> = None;
    for e in &tp.events {
        match e {
            Event::Rapid { p } | Event::Feed { p, .. } | Event::Pose { p, .. } => {
                if let Some(q) = pos {
                    out.push((q, *p));
                }
                pos = Some(*p);
            }
            Event::Arc { p, centre, ccw, .. } => {
                if let Some(q) = pos {
                    let r = (q[0] - centre[0]).hypot(q[1] - centre[1]);
                    let a0 = (q[1] - centre[1]).atan2(q[0] - centre[0]);
                    let a1 = (p[1] - centre[1]).atan2(p[0] - centre[0]);
                    let mut d = a1 - a0;
                    if *ccw {
                        while d <= 1e-12 {
                            d += std::f64::consts::TAU;
                        }
                    } else {
                        while d >= -1e-12 {
                            d -= std::f64::consts::TAU;
                        }
                    }
                    let n = ((d.abs() * r / 0.2).ceil() as usize).max(2);
                    let mut prev = q;
                    for k in 1..=n {
                        let t = k as f64 / n as f64;
                        let a = a0 + d * t;
                        let c = [
                            centre[0] + r * a.cos(),
                            centre[1] + r * a.sin(),
                            q[2] + (p[2] - q[2]) * t,
                        ];
                        out.push((prev, c));
                        prev = c;
                    }
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    out
}

/// Poses along the moves at spacing at most `spacing`.
pub fn sample_poses(mv: &[(P3, P3)], spacing: f64) -> Vec<P3> {
    let mut out = Vec::new();
    for (a, b) in mv {
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
        let n = ((len / spacing).ceil() as usize).max(1);
        for i in 0..=n {
            let t = i as f64 / n as f64;
            out.push([
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ]);
        }
    }
    out
}

#[derive(Debug, Clone, Default)]
pub struct Report {
    pub poses: usize,
    /// Poses where the tip is below the non-cutting height by more than the tolerance.
    pub gouges: usize,
    pub max_gouge: f64,
    pub shank_collisions: usize,
    pub holder_collisions: usize,
}

/// The stack of cylinders above the cutting part: `(radius, from, to)` heights above the tip.
pub fn sections(tool: &Tool) -> Vec<(f64, f64, f64)> {
    let mut v = Vec::new();
    let mut z = tool.flute_length;
    v.push((tool.shank_radius, z, z + tool.shank_length));
    z += tool.shank_length;
    for (r, l) in &tool.holder.sections {
        v.push((*r, z, z + *l));
        z += *l;
    }
    v
}

/// Check poses against a mesh: gouge (tip below the drop-cutter height by more than `tol`) and the
/// shank/holder cylinders against the mesh.
pub fn check_poses(mesh: &TriMesh, tool: &Tool, poses: &[P3], tol: f64) -> Report {
    let mut rep = Report {
        poses: poses.len(),
        ..Report::default()
    };
    let secs = sections(tool);
    for p in poses {
        if let Some(h) = drop_cutter(mesh, tool, p[0], p[1]) {
            let g = h - p[2];
            if g > tol {
                rep.gouges += 1;
            }
            rep.max_gouge = rep.max_gouge.max(g);
        }
        for (k, (r, a, b)) in secs.iter().enumerate() {
            if cylinder_hits_mesh(mesh, [p[0], p[1]], *r, p[2] + a, p[2] + b, 1e-6) {
                if k == 0 {
                    rep.shank_collisions += 1;
                } else {
                    rep.holder_collisions += 1;
                }
            }
        }
    }
    rep
}

/// The shortest distance from the tip to the start of a holder of radius `holder_radius` that keeps it clear
/// of the mesh along all poses (the stick-out the assembly needs).
pub fn required_stickout(mesh: &TriMesh, holder_radius: f64, poses: &[P3]) -> f64 {
    let mut need = f64::NEG_INFINITY;
    for p in poses {
        let q = Aabb {
            min: [
                p[0] - holder_radius,
                p[1] - holder_radius,
                f64::NEG_INFINITY,
            ],
            max: [p[0] + holder_radius, p[1] + holder_radius, f64::INFINITY],
        };
        mesh.query(&q, |i| {
            if let Some((_, hi)) = z_extent_in_disc(&mesh.tris[i], [p[0], p[1]], holder_radius) {
                need = need.max(hi - p[2]);
            }
        });
    }
    need
}

/// Settings of a waterline (constant-z) finishing pass.
#[derive(Debug, Clone)]
pub struct WaterlineParams {
    /// Tool-centre region (x0, y0, x1, y1).
    pub region: [f64; 4],
    pub top: f64,
    pub bottom: f64,
    /// Distance between levels.
    pub step_down: f64,
    /// Spacing of the cutter-location grid.
    pub grid: f64,
    pub safe_z: f64,
    pub feed: f64,
    pub plunge_feed: f64,
}

type Pt2 = [f64; 2];

/// Douglas–Peucker simplification of an open polyline.
fn simplify_line(pts: &[Pt2], tol: f64) -> Vec<Pt2> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = dx.hypot(dy);
    let (mut worst, mut at) = (0.0, 0);
    for (i, p) in pts.iter().enumerate().take(pts.len() - 1).skip(1) {
        let d = if len < 1e-12 {
            (p[0] - a[0]).hypot(p[1] - a[1])
        } else {
            ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx).abs() / len
        };
        if d > worst {
            (worst, at) = (d, i);
        }
    }
    if worst <= tol {
        return vec![a, b];
    }
    let mut left = simplify_line(&pts[..=at], tol);
    let right = simplify_line(&pts[at..], tol);
    left.pop();
    left.extend(right);
    left
}

/// Move the vertices of a polyline that sag into the surface: where a segment's probe points have the tool
/// tip more than `tol / 2` below the cutter-location height `z` would allow, add a vertex found on the
/// level curve by bisecting across the segment, and test the two halves again.
fn refine_line(pts: &[Pt2], cl: &dyn Fn(f64, f64) -> f64, z: f64, tol: f64) -> Vec<Pt2> {
    fn seg(
        a: Pt2,
        b: Pt2,
        depth: usize,
        cl: &dyn Fn(f64, f64) -> f64,
        z: f64,
        tol: f64,
        out: &mut Vec<Pt2>,
    ) {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy);
        if len < 0.08 || depth > 7 {
            out.push(b);
            return;
        }
        // the worst of three probes
        let mut worst: Option<(f64, Pt2)> = None;
        for t in [0.25, 0.5, 0.75] {
            let m = [a[0] + dx * t, a[1] + dy * t];
            let over = cl(m[0], m[1]) - z;
            if over > 0.5 * tol && worst.is_none_or(|w| over > w.0) {
                worst = Some((over, m));
            }
        }
        let Some((_, m)) = worst else {
            out.push(b);
            return;
        };
        // look across the segment for the free side, nearest first
        let n = [-dy / len, dx / len];
        let mut found = None;
        for k in 1..=8 {
            let off = 0.05 * len * k as f64;
            for sgn in [1.0, -1.0] {
                let q = [m[0] + sgn * n[0] * off, m[1] + sgn * n[1] * off];
                if cl(q[0], q[1]) <= z && found.is_none() {
                    found = Some(q);
                }
            }
            if found.is_some() {
                break;
            }
        }
        let Some(mut free) = found else {
            out.push(b);
            return;
        };
        let mut hard = m;
        for _ in 0..22 {
            let mid = [0.5 * (free[0] + hard[0]), 0.5 * (free[1] + hard[1])];
            if cl(mid[0], mid[1]) > z {
                hard = mid;
            } else {
                free = mid;
            }
        }
        seg(a, free, depth + 1, cl, z, tol, out);
        seg(free, b, depth + 1, cl, z, tol, out);
    }
    let mut out = vec![pts[0]];
    for w in pts.windows(2) {
        seg(w[0], w[1], 0, cl, z, tol, &mut out);
    }
    out
}

/// Constant-z finishing: at each level the tool centre follows the curve where the cutter-location surface
/// (the drop-cutter height of the tool) crosses that level, so the tool touches the part along a contour at
/// that height. The surface is sampled on a grid and contoured by marching squares; every crossing point is
/// then moved to the exact crossing on its grid edge by bisection on the true drop-cutter height. Surfaces
/// are single-valued in z: an undercut is not seen.
pub fn waterline(mesh: &TriMesh, tool: &Tool, p: &WaterlineParams) -> Toolpath {
    use rayon::prelude::*;
    use std::collections::HashMap;
    let mut tp = Toolpath::new("waterline");
    tp.push(Event::ToolChange {
        number: tool.number,
        name: tool.name.clone(),
        diameter: tool.diameter,
        rpm: 9000.0,
    });
    tp.push(Event::SpindleOn {
        rpm: 9000.0,
        clockwise: true,
    });
    let [x0, y0, x1, y1] = p.region;
    let g = p.grid.max(0.02);
    let (nx, ny) = (
        (((x1 - x0) / g).ceil() as usize + 1).max(2),
        (((y1 - y0) / g).ceil() as usize + 1).max(2),
    );
    let sx = (x1 - x0) / (nx - 1) as f64;
    let sy = (y1 - y0) / (ny - 1) as f64;
    let at = |i: usize, j: usize| (x0 + i as f64 * sx, y0 + j as f64 * sy);
    let cl = |x: f64, y: f64| drop_cutter(mesh, tool, x, y).unwrap_or(f64::NEG_INFINITY);
    let field: Vec<f64> = (0..nx * ny)
        .into_par_iter()
        .map(|k| {
            let (x, y) = at(k / ny, k % ny);
            cl(x, y)
        })
        .collect();
    let f = |i: usize, j: usize| field[i * ny + j];
    let depth = (p.top - p.bottom).max(0.0);
    let levels = ((depth / p.step_down.max(0.05)).ceil() as usize).max(1);
    let mut pos: Option<Pt2> = None;
    for lv in 1..=levels {
        let z = p.top - depth * lv as f64 / levels as f64;
        let blocked = |i: usize, j: usize| f(i, j) > z;
        // crossing points on grid edges, found when first needed
        let mut ids: HashMap<(u8, usize, usize), usize> = HashMap::new();
        let mut points: Vec<Pt2> = Vec::new();
        let mut cross = |kind: u8, i: usize, j: usize| -> usize {
            if let Some(id) = ids.get(&(kind, i, j)) {
                return *id;
            }
            let (a, b) = if kind == 0 {
                ((i, j), (i + 1, j))
            } else {
                ((i, j), (i, j + 1))
            };
            let (pa, pb) = (at(a.0, a.1), at(b.0, b.1));
            // free end and blocked end of the edge
            let (mut free, mut hard) = if blocked(a.0, a.1) {
                (pb, pa)
            } else {
                (pa, pb)
            };
            for _ in 0..22 {
                let m = [0.5 * (free.0 + hard.0), 0.5 * (free.1 + hard.1)];
                if cl(m[0], m[1]) > z {
                    hard = (m[0], m[1]);
                } else {
                    free = (m[0], m[1]);
                }
            }
            points.push([free.0, free.1]);
            ids.insert((kind, i, j), points.len() - 1);
            points.len() - 1
        };
        let mut segs: Vec<(usize, usize)> = Vec::new();
        for i in 0..nx - 1 {
            for j in 0..ny - 1 {
                let (c0, c1, c2, c3) = (
                    blocked(i, j),
                    blocked(i + 1, j),
                    blocked(i + 1, j + 1),
                    blocked(i, j + 1),
                );
                let case = c0 as u8 | (c1 as u8) << 1 | (c2 as u8) << 2 | (c3 as u8) << 3;
                if case == 0 || case == 15 {
                    continue;
                }
                // edges: 0 bottom (i,j)-(i+1,j), 1 right (i+1,j)-(i+1,j+1), 2 top (i,j+1)-(i+1,j+1), 3 left
                let mut e = |k: u8| match k {
                    0 => cross(0, i, j),
                    1 => cross(1, i + 1, j),
                    2 => cross(0, i, j + 1),
                    _ => cross(1, i, j),
                };
                let pairs: &[(u8, u8)] = match case {
                    1 | 14 => &[(3, 0)],
                    2 | 13 => &[(0, 1)],
                    3 | 12 => &[(3, 1)],
                    4 | 11 => &[(1, 2)],
                    6 | 9 => &[(0, 2)],
                    7 | 8 => &[(3, 2)],
                    5 | 10 => {
                        // a saddle: join the corners through the centre when it is blocked
                        let centre_blocked =
                            0.25 * (f(i, j) + f(i + 1, j) + f(i + 1, j + 1) + f(i, j + 1)) > z;
                        match (case, centre_blocked) {
                            (5, true) | (10, false) => &[(3, 2), (0, 1)],
                            _ => &[(3, 0), (1, 2)],
                        }
                    }
                    _ => &[],
                };
                for (a, b) in pairs {
                    let (ia, ib) = (e(*a), e(*b));
                    if ia != ib {
                        segs.push((ia, ib));
                    }
                }
            }
        }
        // chain the segments into polylines
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for (a, b) in &segs {
            adj.entry(*a).or_default().push(*b);
            adj.entry(*b).or_default().push(*a);
        }
        let mut used: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
        let key = |a: usize, b: usize| (a.min(b), a.max(b));
        let mut lines: Vec<(Vec<Pt2>, bool)> = Vec::new();
        let mut starts: Vec<usize> = adj
            .iter()
            .filter(|(_, v)| v.len() == 1)
            .map(|(k, _)| *k)
            .collect();
        starts.sort_unstable();
        let mut rest: Vec<usize> = adj.keys().copied().collect();
        rest.sort_unstable();
        for start in starts.into_iter().chain(rest) {
            loop {
                let next = adj[&start]
                    .iter()
                    .copied()
                    .find(|n| !used.contains(&key(start, *n)));
                let Some(mut cur) = next else {
                    break;
                };
                let mut line = vec![start];
                used.insert(key(start, cur));
                loop {
                    line.push(cur);
                    let Some(n) = adj[&cur]
                        .iter()
                        .copied()
                        .find(|n| !used.contains(&key(cur, *n)))
                    else {
                        break;
                    };
                    used.insert(key(cur, n));
                    cur = n;
                }
                let closed = line.first() == line.last() || {
                    let (a, b) = (line[0], line[line.len() - 1]);
                    adj[&b].contains(&a) && line.len() > 2
                };
                let mut pts: Vec<Pt2> = line.iter().map(|i| points[*i]).collect();
                if closed && pts.first() != pts.last() {
                    pts.push(pts[0]);
                }
                let pts = simplify_line(&pts, 0.004);
                lines.push((refine_line(&pts, &cl, z, 0.01), closed));
            }
        }
        // visit the nearest first
        let mut todo = lines;
        while !todo.is_empty() {
            let from = pos.unwrap_or([x0, y0]);
            let d2 = |q: Pt2| (q[0] - from[0]).powi(2) + (q[1] - from[1]).powi(2);
            let (k, rev) = todo
                .iter()
                .enumerate()
                .flat_map(|(k, (l, _))| [(k, false, d2(l[0])), (k, true, d2(l[l.len() - 1]))])
                .min_by(|a, b| a.2.total_cmp(&b.2))
                .map(|(k, r, _)| (k, r))
                .expect("a line");
            let (mut line, _) = todo.swap_remove(k);
            if rev {
                line.reverse();
            }
            if line.len() < 2 {
                continue;
            }
            tp.push(Event::Rapid {
                p: [line[0][0], line[0][1], p.safe_z],
            });
            tp.push(Event::Feed {
                p: [line[0][0], line[0][1], z],
                feed: p.plunge_feed,
            });
            for q in &line[1..] {
                tp.push(Event::Feed {
                    p: [q[0], q[1], z],
                    feed: p.feed,
                });
            }
            let last = line[line.len() - 1];
            tp.push(Event::Rapid {
                p: [last[0], last[1], p.safe_z],
            });
            pos = Some(last);
        }
    }
    tp.push(Event::SpindleOff);
    tp.push(Event::ProgramEnd);
    tp
}
