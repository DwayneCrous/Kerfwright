//! Tri-dexel stock simulation. The stock is stored as three families of rays (along x, y and z), each ray
//! holding the sorted, disjoint intervals of material along it. A vertical cutter, posed along the toolpath at
//! a spacing of at most half a cell, removes from every ray the interval its body covers. Tilted setups (3+2)
//! are simulated in the frame of the setup where the cutter is vertical; continuous five-axis motion is not
//! simulated here.

use crate::bvh::{Aabb, TriMesh};
use crate::dropcutter::Profile;
use crate::path::{Event, Toolpath};
use crate::tool::Tool;
use crate::P3;

type Seg = [f64; 2];

/// Remove `[a, b]` from a sorted list of disjoint intervals; returns the length removed.
fn subtract(v: &mut Vec<Seg>, a: f64, b: f64) -> f64 {
    if b <= a {
        return 0.0;
    }
    let mut removed = 0.0;
    let mut i = 0;
    while i < v.len() {
        let [s0, s1] = v[i];
        if s1 <= a {
            i += 1;
            continue;
        }
        if s0 >= b {
            break;
        }
        let lo = s0.max(a);
        let hi = s1.min(b);
        removed += hi - lo;
        match (s0 < a, s1 > b) {
            (false, false) => {
                v.remove(i);
            }
            (true, false) => {
                v[i][1] = a;
                i += 1;
            }
            (false, true) => {
                v[i][0] = b;
                i += 1;
            }
            (true, true) => {
                v[i][1] = a;
                v.insert(i + 1, [b, s1]);
                i += 2;
            }
        }
    }
    removed
}

/// The parts of `a` not covered by `b` (both sorted and disjoint).
pub fn difference(a: &[Seg], b: &[Seg]) -> Vec<Seg> {
    let mut out = Vec::new();
    for &[s0, s1] in a {
        let mut lo = s0;
        for &[t0, t1] in b {
            if t1 <= lo {
                continue;
            }
            if t0 >= s1 {
                break;
            }
            if t0 > lo {
                out.push([lo, t0]);
            }
            lo = lo.max(t1);
            if lo >= s1 {
                break;
            }
        }
        if lo < s1 {
            out.push([lo, s1]);
        }
    }
    out
}

fn length(v: &[Seg]) -> f64 {
    v.iter().map(|s| s[1] - s[0]).sum()
}

/// One triangle of the surface of the stock with its outward normal (an axis direction).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Facet {
    pub tri: [P3; 3],
    pub normal: P3,
}

/// Where the surfaces of two dexel stocks (or a stock and a part) differ, in volume.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Comparison {
    /// Material of the stock the part does not contain: still to be removed.
    pub remaining: f64,
    /// Material of the part the stock lacks: cut away that should not have been.
    pub overcut: f64,
}

#[derive(Debug, Clone)]
pub struct TriDexel {
    pub min: P3,
    pub max: P3,
    pub h: f64,
    n: [usize; 3],
    /// Rays along z indexed `[i * ny + j]` (x, y), along x indexed `[j * nz + k]` (y, z), along y `[i * nz + k]`.
    zr: Vec<Vec<Seg>>,
    xr: Vec<Vec<Seg>>,
    yr: Vec<Vec<Seg>>,
    pos: Option<P3>,
    /// Direction of the tool axis at `pos` (tip towards spindle).
    axis: P3,
    /// Volume removed by rapid moves (a non-zero value means a rapid cut into stock).
    pub rapid_removed: f64,
}

impl TriDexel {
    pub fn block(min: P3, max: P3, h: f64) -> TriDexel {
        let n = [
            ((max[0] - min[0]) / h).round().max(1.0) as usize,
            ((max[1] - min[1]) / h).round().max(1.0) as usize,
            ((max[2] - min[2]) / h).round().max(1.0) as usize,
        ];
        // adjust so cells tile the block exactly
        let h3 = [
            (max[0] - min[0]) / n[0] as f64,
            (max[1] - min[1]) / n[1] as f64,
            (max[2] - min[2]) / n[2] as f64,
        ];
        assert!(
            (h3[0] - h3[1]).abs() < 1e-9 * h && (h3[1] - h3[2]).abs() < 1e-9 * h,
            "the block must be a whole number of cells in each direction (cell {h}, extents {h3:?})"
        );
        let h = h3[0];
        TriDexel {
            min,
            max,
            h,
            n,
            zr: vec![vec![[min[2], max[2]]]; n[0] * n[1]],
            xr: vec![vec![[min[0], max[0]]]; n[1] * n[2]],
            yr: vec![vec![[min[1], max[1]]]; n[0] * n[2]],
            pos: None,
            axis: [0.0, 0.0, 1.0],
            rapid_removed: 0.0,
        }
    }

    pub fn cells(&self) -> [usize; 3] {
        self.n
    }

    /// A grid of cubic cells of about `h` covering `[min, max]` (the upper corner moves up to a whole number
    /// of cells), holding no material.
    pub fn empty_grid(min: P3, max: P3, h: f64) -> TriDexel {
        let n = [
            ((max[0] - min[0]) / h).ceil().max(1.0) as usize,
            ((max[1] - min[1]) / h).ceil().max(1.0) as usize,
            ((max[2] - min[2]) / h).ceil().max(1.0) as usize,
        ];
        TriDexel {
            min,
            max: [
                min[0] + n[0] as f64 * h,
                min[1] + n[1] as f64 * h,
                min[2] + n[2] as f64 * h,
            ],
            h,
            n,
            zr: vec![Vec::new(); n[0] * n[1]],
            xr: vec![Vec::new(); n[1] * n[2]],
            yr: vec![Vec::new(); n[0] * n[2]],
            pos: None,
            axis: [0.0, 0.0, 1.0],
            rapid_removed: 0.0,
        }
    }

    /// The material inside a closed triangle mesh, sampled on the grid of `empty_grid(min, max, h)`. Each ray
    /// pairs up the places where it crosses the surface; a mesh with holes gives rays with a dropped
    /// crossing, so the mesh should be closed.
    pub fn from_mesh(mesh: &TriMesh, min: P3, max: P3, h: f64) -> TriDexel {
        let mut d = TriDexel::empty_grid(min, max, h);
        let n = d.n;
        let ext = mesh.bounds();
        // a ray along axis `a` through the cell centres of the other two axes `p`, `q`; the origin is nudged
        // by an amount no vertex or edge is likely to share
        let nudge = [1.234567e-3 * h, 2.345679e-3 * h, 3.456791e-3 * h];
        let fill = |a: usize, p: usize, q: usize, rays: &mut Vec<Vec<Seg>>| {
            for ip in 0..n[p] {
                let pc = d.min[p] + (ip as f64 + 0.5) * h + nudge[p];
                for iq in 0..n[q] {
                    let qc = d.min[q] + (iq as f64 + 0.5) * h + nudge[q];
                    let mut bb = Aabb {
                        min: [0.0; 3],
                        max: [0.0; 3],
                    };
                    bb.min[p] = pc;
                    bb.max[p] = pc;
                    bb.min[q] = qc;
                    bb.max[q] = qc;
                    bb.min[a] = ext.min[a] - 1.0;
                    bb.max[a] = ext.max[a] + 1.0;
                    let mut hits: Vec<f64> = Vec::new();
                    mesh.query(&bb, |ti| {
                        let t = &mesh.tris[ti];
                        let (p0, p1, p2) =
                            ([t[0][p], t[0][q]], [t[1][p], t[1][q]], [t[2][p], t[2][q]]);
                        let area =
                            (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
                        if area.abs() < 1e-14 {
                            return;
                        }
                        let w0 = ((p1[0] - pc) * (p2[1] - qc) - (p1[1] - qc) * (p2[0] - pc)) / area;
                        let w1 = ((p2[0] - pc) * (p0[1] - qc) - (p2[1] - qc) * (p0[0] - pc)) / area;
                        let w2 = 1.0 - w0 - w1;
                        if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 {
                            hits.push(w0 * t[0][a] + w1 * t[1][a] + w2 * t[2][a]);
                        }
                    });
                    hits.sort_by(f64::total_cmp);
                    if hits.len() % 2 == 1 {
                        hits.pop();
                    }
                    let (lo, hi) = (d.min[a], d.max[a]);
                    let ray = &mut rays[ip * n[q] + iq];
                    for pair in hits.as_chunks::<2>().0 {
                        let (s0, s1) = (pair[0].max(lo), pair[1].min(hi));
                        if s1 > s0 {
                            ray.push([s0, s1]);
                        }
                    }
                }
            }
        };
        let (mut zr, mut xr, mut yr) = (
            std::mem::take(&mut d.zr),
            std::mem::take(&mut d.xr),
            std::mem::take(&mut d.yr),
        );
        fill(2, 0, 1, &mut zr);
        fill(0, 1, 2, &mut xr);
        fill(1, 0, 2, &mut yr);
        d.zr = zr;
        d.xr = xr;
        d.yr = yr;
        d
    }

    /// Forget where the tool was: the next rapid does not cut a line from the end of the previous
    /// operation (the operation before it left the spindle at a tool change).
    pub fn lift_tool(&mut self) {
        self.pos = None;
    }

    /// The triangles of the surface of the material. Heights are exact (the ends of the z intervals), the
    /// walls are cell-wide steps: the surface is as accurate as the cell size sideways.
    pub fn facets(&self) -> Vec<Facet> {
        let (nx, ny) = (self.n[0], self.n[1]);
        let h = self.h;
        let mut out = Vec::new();
        let quad = |out: &mut Vec<Facet>, c: [P3; 4], normal: P3| {
            out.push(Facet {
                tri: [c[0], c[1], c[2]],
                normal,
            });
            out.push(Facet {
                tri: [c[0], c[2], c[3]],
                normal,
            });
        };
        let none: Vec<Seg> = Vec::new();
        for i in 0..nx {
            let (x0, x1) = (self.min[0] + i as f64 * h, self.min[0] + (i + 1) as f64 * h);
            for j in 0..ny {
                let (y0, y1) = (self.min[1] + j as f64 * h, self.min[1] + (j + 1) as f64 * h);
                let me = &self.zr[i * ny + j];
                for s in me {
                    quad(
                        &mut out,
                        [
                            [x0, y0, s[1]],
                            [x1, y0, s[1]],
                            [x1, y1, s[1]],
                            [x0, y1, s[1]],
                        ],
                        [0.0, 0.0, 1.0],
                    );
                    quad(
                        &mut out,
                        [
                            [x0, y0, s[0]],
                            [x0, y1, s[0]],
                            [x1, y1, s[0]],
                            [x1, y0, s[0]],
                        ],
                        [0.0, 0.0, -1.0],
                    );
                }
                // the walls: what this cell shows beyond each of its four neighbours
                let east = if i + 1 < nx {
                    &self.zr[(i + 1) * ny + j]
                } else {
                    &none
                };
                let north = if j + 1 < ny {
                    &self.zr[i * ny + j + 1]
                } else {
                    &none
                };
                let west = if i > 0 {
                    &self.zr[(i - 1) * ny + j]
                } else {
                    &none
                };
                let south = if j > 0 {
                    &self.zr[i * ny + j - 1]
                } else {
                    &none
                };
                for s in difference(me, east) {
                    quad(
                        &mut out,
                        [
                            [x1, y0, s[0]],
                            [x1, y1, s[0]],
                            [x1, y1, s[1]],
                            [x1, y0, s[1]],
                        ],
                        [1.0, 0.0, 0.0],
                    );
                }
                for s in difference(me, west) {
                    quad(
                        &mut out,
                        [
                            [x0, y0, s[0]],
                            [x0, y0, s[1]],
                            [x0, y1, s[1]],
                            [x0, y1, s[0]],
                        ],
                        [-1.0, 0.0, 0.0],
                    );
                }
                for s in difference(me, north) {
                    quad(
                        &mut out,
                        [
                            [x0, y1, s[0]],
                            [x0, y1, s[1]],
                            [x1, y1, s[1]],
                            [x1, y1, s[0]],
                        ],
                        [0.0, 1.0, 0.0],
                    );
                }
                for s in difference(me, south) {
                    quad(
                        &mut out,
                        [
                            [x0, y0, s[0]],
                            [x1, y0, s[0]],
                            [x1, y0, s[1]],
                            [x0, y0, s[1]],
                        ],
                        [0.0, -1.0, 0.0],
                    );
                }
            }
        }
        out
    }

    /// The surface as a triangle mesh (for another dexel grid, or for display).
    pub fn to_mesh(&self) -> TriMesh {
        TriMesh::new(self.facets().into_iter().map(|f| f.tri).collect())
    }

    fn ray_of(&self, axis: usize, c: P3) -> (&Vec<Seg>, f64) {
        let cell = |k: usize| {
            (((c[k] - self.min[k]) / self.h).floor().max(0.0) as usize).min(self.n[k] - 1)
        };
        match axis {
            2 => (&self.zr[cell(0) * self.n[1] + cell(1)], c[2]),
            0 => (&self.xr[cell(1) * self.n[2] + cell(2)], c[0]),
            _ => (&self.yr[cell(0) * self.n[2] + cell(2)], c[1]),
        }
    }

    /// How far the surface facet `f` of this stock is from `part` along its normal, measured on the dexel
    /// ray through the facet: positive where this stock still has material beyond the part, negative where
    /// the part has material the stock lacks. `part` must be on the same grid.
    pub fn facet_deviation(&self, part: &TriDexel, f: &Facet) -> f64 {
        let axis = (0..3).find(|k| f.normal[*k] != 0.0).unwrap_or(2);
        let sign = f.normal[axis].signum();
        let c = [
            (f.tri[0][0] + f.tri[1][0] + f.tri[2][0]) / 3.0,
            (f.tri[0][1] + f.tri[1][1] + f.tri[2][1]) / 3.0,
            (f.tri[0][2] + f.tri[1][2] + f.tri[2][2]) / 3.0,
        ];
        // the facet is on the boundary of a cell: take the cell inside the material
        let mut inside = c;
        inside[axis] -= sign * 0.5 * self.h;
        let (ray, _) = part.ray_of(axis, inside);
        let s = c[axis];
        let eps = 1e-9;
        if let Some(seg) = ray.iter().find(|g| g[0] + eps < s && s < g[1] - eps) {
            // the part continues beyond the stock's surface
            return -if sign > 0.0 { seg[1] - s } else { s - seg[0] };
        }
        let behind = if sign > 0.0 {
            ray.iter().rev().find(|g| g[1] <= s + eps).map(|g| s - g[1])
        } else {
            ray.iter().find(|g| g[0] >= s - eps).map(|g| g[0] - s)
        };
        behind.unwrap_or(f64::INFINITY)
    }

    /// Volumes of the stock outside `part` and of `part` outside the stock (measured on the z rays).
    pub fn compare(&self, part: &TriDexel) -> Comparison {
        let h2 = self.h * self.h;
        let mut c = Comparison::default();
        for (a, b) in self.zr.iter().zip(&part.zr) {
            c.remaining += length(&difference(a, b)) * h2;
            c.overcut += length(&difference(b, a)) * h2;
        }
        c
    }

    fn centre(&self, axis: usize, i: usize) -> f64 {
        self.min[axis] + (i as f64 + 0.5) * self.h
    }

    /// Index range of cell centres within `[lo, hi]` along an axis.
    fn range(&self, axis: usize, lo: f64, hi: f64) -> std::ops::Range<usize> {
        let a = ((lo - self.min[axis]) / self.h - 0.5).ceil().max(0.0) as usize;
        let b = (((hi - self.min[axis]) / self.h - 0.5).floor() + 1.0).max(0.0) as usize;
        a.min(self.n[axis])..b.min(self.n[axis])
    }

    /// Remove the volume of the cutter with its tip at `tip`; returns the volume removed (measured on the
    /// z rays).
    pub fn cut_pose(&mut self, tool: &Tool, tip: P3) -> f64 {
        let prof = Profile::of(tool);
        let big = prof.big;
        let top = tip[2] + tool.flute_length;
        let h2 = self.h * self.h;
        let mut removed = 0.0;
        let (nx, ny, nz) = (self.n[0], self.n[1], self.n[2]);
        let _ = nx;
        for i in self.range(0, tip[0] - big, tip[0] + big) {
            let dx = self.centre(0, i) - tip[0];
            for j in self.range(1, tip[1] - big, tip[1] + big) {
                let d = dx.hypot(self.centre(1, j) - tip[1]);
                if d <= big {
                    removed += subtract(&mut self.zr[i * ny + j], tip[2] + prof.lift(d), top) * h2;
                }
            }
        }
        let reach = |hh: f64| prof.reach(hh);
        for k in self.range(2, tip[2], top) {
            let hh = self.centre(2, k) - tip[2];
            let dm = reach(hh);
            for j in self.range(1, tip[1] - dm, tip[1] + dm) {
                let dy = self.centre(1, j) - tip[1];
                if dy.abs() < dm {
                    let w = (dm * dm - dy * dy).sqrt();
                    subtract(&mut self.xr[j * nz + k], tip[0] - w, tip[0] + w);
                }
            }
            for i in self.range(0, tip[0] - dm, tip[0] + dm) {
                let dx = self.centre(0, i) - tip[0];
                if dx.abs() < dm {
                    let w = (dm * dm - dx * dx).sqrt();
                    subtract(&mut self.yr[i * nz + k], tip[1] - w, tip[1] + w);
                }
            }
        }
        removed
    }

    /// Remove the volume of the cutter with its tip at `tip` and its axis along `axis` (a unit vector from
    /// the tip towards the spindle; only the flutes cut). Each dexel ray is intersected with the cutter's
    /// body of revolution, which is convex, so the part of the ray inside it is one interval. Returns the
    /// volume removed (measured on the z rays).
    pub fn cut_pose_axis(&mut self, tool: &Tool, tip: P3, axis: P3) -> f64 {
        let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let a = [axis[0] / n, axis[1] / n, axis[2] / n];
        let prof = Profile::of(tool);
        let len = tool.flute_length;
        let big = prof.big;
        // bounding box of the body: the axis segment grown by the radius (the sphere of a tilted axis is
        // bounded by the radius times the sine of its angle to each coordinate axis)
        let mut lo = [0.0; 3];
        let mut hi = [0.0; 3];
        for k in 0..3 {
            let grow = big * (1.0 - a[k] * a[k]).max(0.0).sqrt();
            lo[k] = tip[k] + (a[k] * len).min(0.0) - grow;
            hi[k] = tip[k] + (a[k] * len).max(0.0) + grow;
        }
        let h2 = self.h * self.h;
        let mut removed = 0.0;
        let nn = self.n;
        // the three families: ray axis, the two others, and where the rays live
        for fam in 0..3usize {
            let (ax, p, q) = match fam {
                0 => (2usize, 0usize, 1usize),
                1 => (0, 1, 2),
                _ => (1, 0, 2),
            };
            let ranges = (self.range(p, lo[p], hi[p]), self.range(q, lo[q], hi[q]));
            for ip in ranges.0 {
                let pc = self.centre(p, ip);
                for iq in ranges.1.clone() {
                    let qc = self.centre(q, iq);
                    // a point on the ray at the lower end of the body's box, direction e = unit(ax)
                    let mut o = [0.0; 3];
                    o[p] = pc;
                    o[q] = qc;
                    o[ax] = lo[ax];
                    let w = [o[0] - tip[0], o[1] - tip[1], o[2] - tip[2]];
                    let ea = a[ax];
                    let wa = w[0] * a[0] + w[1] * a[1] + w[2] * a[2];
                    // s(t) = wa + t ea in [0, len]
                    let (mut t0, mut t1) = (0.0f64, hi[ax] - lo[ax]);
                    if ea.abs() > 1e-12 {
                        let (u, v) = ((0.0 - wa) / ea, (len - wa) / ea);
                        t0 = t0.max(u.min(v));
                        t1 = t1.min(u.max(v));
                    } else if wa < 0.0 || wa > len {
                        continue;
                    }
                    // rho^2(t) = |w|^2 + 2 t w_e + t^2 - s(t)^2 <= big^2
                    let ww = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
                    let qa = 1.0 - ea * ea;
                    let qb = 2.0 * (w[ax] - wa * ea);
                    let qc2 = ww - wa * wa - big * big;
                    if qa > 1e-12 {
                        let disc = qb * qb - 4.0 * qa * qc2;
                        if disc < 0.0 {
                            continue;
                        }
                        let sq = disc.sqrt();
                        t0 = t0.max((-qb - sq) / (2.0 * qa));
                        t1 = t1.min((-qb + sq) / (2.0 * qa));
                    } else if qc2 > 0.0 {
                        continue;
                    }
                    if t1 <= t0 {
                        continue;
                    }
                    let inside = |t: f64| -> f64 {
                        let s = wa + t * ea;
                        let r2 = ww + 2.0 * t * w[ax] + t * t - s * s;
                        let reach = prof.reach(s.clamp(0.0, len));
                        reach * reach - r2.max(0.0)
                    };
                    // the body is convex: look for a point inside, then bisect to both boundaries
                    let steps = 16;
                    let mut found = None;
                    for i in 0..=steps {
                        let t = t0 + (t1 - t0) * i as f64 / steps as f64;
                        if inside(t) >= 0.0 {
                            found = Some(t);
                            break;
                        }
                    }
                    let Some(tin) = found else {
                        continue;
                    };
                    let edge = |mut good: f64, mut bad: f64| -> f64 {
                        for _ in 0..24 {
                            let m = 0.5 * (good + bad);
                            if inside(m) >= 0.0 {
                                good = m;
                            } else {
                                bad = m;
                            }
                        }
                        good
                    };
                    let a0 = if inside(t0) >= 0.0 { t0 } else { edge(tin, t0) };
                    let a1 = if inside(t1) >= 0.0 { t1 } else { edge(tin, t1) };
                    let (c0, c1) = (o[ax] + a0, o[ax] + a1);
                    let ray = match fam {
                        0 => &mut self.zr[ip * nn[1] + iq],
                        1 => &mut self.xr[ip * nn[2] + iq],
                        _ => &mut self.yr[ip * nn[2] + iq],
                    };
                    let r = subtract(ray, c0, c1);
                    if fam == 0 {
                        removed += r * h2;
                    }
                }
            }
        }
        removed
    }

    /// Cut along a straight move of a tilted tool: tip and axis are interpolated (the axis along the great
    /// circle) in steps of at most h/2 of tip travel or a quarter-cell of the tool's reach in rotation.
    pub fn cut_line_axis(&mut self, tool: &Tool, a: (P3, P3), b: (P3, P3)) -> f64 {
        let dist =
            ((b.0[0] - a.0[0]).powi(2) + (b.0[1] - a.0[1]).powi(2) + (b.0[2] - a.0[2]).powi(2))
                .sqrt();
        let dot = (a.1[0] * b.1[0] + a.1[1] * b.1[1] + a.1[2] * b.1[2]).clamp(-1.0, 1.0);
        let ang = dot.acos();
        // the far end of the flutes moves by angle · length
        let swing = ang * tool.flute_length;
        let n = (((dist.max(swing)) / (0.5 * self.h)).ceil() as usize).max(1);
        let mut removed = 0.0;
        for s in 1..=n {
            let t = s as f64 / n as f64;
            let tip = [
                a.0[0] + (b.0[0] - a.0[0]) * t,
                a.0[1] + (b.0[1] - a.0[1]) * t,
                a.0[2] + (b.0[2] - a.0[2]) * t,
            ];
            let axis = if ang < 1e-9 {
                a.1
            } else {
                let (sa, sb) = (
                    ((1.0 - t) * ang).sin() / ang.sin(),
                    (t * ang).sin() / ang.sin(),
                );
                [
                    sa * a.1[0] + sb * b.1[0],
                    sa * a.1[1] + sb * b.1[1],
                    sa * a.1[2] + sb * b.1[2],
                ]
            };
            removed += self.cut_pose_axis(tool, tip, axis);
        }
        removed
    }

    /// Cut along a straight segment, posing the cutter at spacing ≤ h/2.
    pub fn cut_line(&mut self, tool: &Tool, a: P3, b: P3) -> f64 {
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
        let n = (len / (0.5 * self.h)).ceil().max(1.0) as usize;
        let mut removed = 0.0;
        for s in 1..=n {
            let t = s as f64 / n as f64;
            removed += self.cut_pose(
                tool,
                [
                    a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t,
                ],
            );
        }
        removed
    }

    /// Cut along an arc about `centre` (helical when the heights differ).
    pub fn cut_arc(&mut self, tool: &Tool, a: P3, b: P3, centre: [f64; 2], ccw: bool) -> f64 {
        let (ra, rb) = (
            (a[0] - centre[0]).hypot(a[1] - centre[1]),
            (b[0] - centre[0]).hypot(b[1] - centre[1]),
        );
        let a0 = (a[1] - centre[1]).atan2(a[0] - centre[0]);
        let a1 = (b[1] - centre[1]).atan2(b[0] - centre[0]);
        let mut sweep = a1 - a0;
        if ccw {
            while sweep <= 1e-12 {
                sweep += std::f64::consts::TAU;
            }
        } else {
            while sweep >= -1e-12 {
                sweep -= std::f64::consts::TAU;
            }
        }
        let r = ra.max(rb);
        let len = (sweep.abs() * r).hypot(b[2] - a[2]);
        let n = (len / (0.5 * self.h)).ceil().max(1.0) as usize;
        let mut removed = 0.0;
        for s in 1..=n {
            let t = s as f64 / n as f64;
            let th = a0 + sweep * t;
            let rr = ra + (rb - ra) * t;
            removed += self.cut_pose(
                tool,
                [
                    centre[0] + rr * th.cos(),
                    centre[1] + rr * th.sin(),
                    a[2] + (b[2] - a[2]) * t,
                ],
            );
        }
        removed
    }

    /// Move the tool to `p` with its axis along `axis`, cutting on the way; a vertical-to-vertical move takes
    /// the fast path. Returns the volume removed.
    fn advance(&mut self, tool: &Tool, p: P3, axis: P3, rapid: bool) -> f64 {
        let mut r = 0.0;
        if let Some(q) = self.pos {
            let vertical = |a: P3| a[0].abs() < 1e-9 && a[1].abs() < 1e-9 && a[2] > 0.0;
            r = if vertical(self.axis) && vertical(axis) {
                self.cut_line(tool, q, p)
            } else {
                self.cut_line_axis(tool, (q, self.axis), (p, axis))
            };
            if rapid {
                self.rapid_removed += r;
            }
        }
        self.pos = Some(p);
        self.axis = axis;
        r
    }

    /// Run a toolpath. Rapids also cut (a rapid through material is a collision), and what they remove is
    /// recorded in `rapid_removed`. Returns the volume removed by the whole path.
    pub fn run(&mut self, tool: &Tool, path: &Toolpath) -> f64 {
        let mut total = 0.0;
        let mut drill: Option<(f64, f64, f64)> = None; // depth, retract, surface
        for e in &path.events {
            match e {
                Event::Rapid { p } => total += self.advance(tool, *p, [0.0, 0.0, 1.0], true),
                Event::Feed { p, .. } => total += self.advance(tool, *p, [0.0, 0.0, 1.0], false),
                Event::Pose { p, axis, feed } => {
                    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
                    let a = [axis[0] / n, axis[1] / n, axis[2] / n];
                    total += self.advance(tool, *p, a, feed.is_none());
                }
                Event::Arc { p, centre, ccw, .. } => {
                    if let Some(q) = self.pos {
                        total += self.cut_arc(tool, q, *p, *centre, *ccw);
                    }
                    self.pos = Some(*p);
                    self.axis = [0.0, 0.0, 1.0];
                }
                Event::DrillStart {
                    depth,
                    retract,
                    surface,
                    ..
                } => drill = Some((*depth, *retract, *surface)),
                Event::DrillAt { x, y } => {
                    if let Some((depth, retract, _)) = drill {
                        let top = [*x, *y, retract];
                        if let Some(q) = self.pos {
                            let r = self.cut_line(tool, q, top);
                            self.rapid_removed += r;
                            total += r;
                        }
                        total += self.cut_line(tool, top, [*x, *y, depth]);
                        total += self.cut_line(tool, [*x, *y, depth], top);
                        self.pos = Some(top);
                    }
                }
                _ => {}
            }
        }
        total
    }

    /// Material volume as measured on the z, x and y rays.
    pub fn volume(&self) -> [f64; 3] {
        let h2 = self.h * self.h;
        let sum = |v: &Vec<Vec<Seg>>| {
            v.iter()
                .flat_map(|r| r.iter())
                .map(|s| s[1] - s[0])
                .sum::<f64>()
                * h2
        };
        [sum(&self.zr), sum(&self.xr), sum(&self.yr)]
    }

    pub fn initial_volume(&self) -> f64 {
        (self.max[0] - self.min[0]) * (self.max[1] - self.min[1]) * (self.max[2] - self.min[2])
    }

    /// Top surface height of the stock at cell `(i, j)` (the highest material on the z ray), if any.
    pub fn top_height(&self, i: usize, j: usize) -> Option<f64> {
        self.zr[i * self.n[1] + j].last().map(|s| s[1])
    }
}
