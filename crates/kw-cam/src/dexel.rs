//! Tri-dexel stock simulation. The stock is stored as three families of rays (along x, y and z), each ray
//! holding the sorted, disjoint intervals of material along it. A vertical cutter, posed along the toolpath at
//! a spacing of at most half a cell, removes from every ray the interval its body covers. Tilted setups (3+2)
//! are simulated in the frame of the setup where the cutter is vertical; continuous five-axis motion is not
//! simulated here.

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
            rapid_removed: 0.0,
        }
    }

    pub fn cells(&self) -> [usize; 3] {
        self.n
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
        let reach = |hh: f64| -> f64 {
            if prof.r == 0.0 || hh >= prof.r {
                big
            } else {
                prof.rho + (2.0 * prof.r * hh - hh * hh).max(0.0).sqrt()
            }
        };
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

    /// Run a toolpath. Rapids also cut (a rapid through material is a collision), and what they remove is
    /// recorded in `rapid_removed`. Returns the volume removed by the whole path.
    pub fn run(&mut self, tool: &Tool, path: &Toolpath) -> f64 {
        let mut total = 0.0;
        let mut drill: Option<(f64, f64, f64)> = None; // depth, retract, surface
        for e in &path.events {
            match e {
                Event::Rapid { p } => {
                    if let Some(q) = self.pos {
                        let r = self.cut_line(tool, q, *p);
                        self.rapid_removed += r;
                        total += r;
                    }
                    self.pos = Some(*p);
                }
                Event::Feed { p, .. } => {
                    if let Some(q) = self.pos {
                        total += self.cut_line(tool, q, *p);
                    }
                    self.pos = Some(*p);
                }
                Event::Arc { p, centre, ccw, .. } => {
                    if let Some(q) = self.pos {
                        total += self.cut_arc(tool, q, *p, *centre, *ccw);
                    }
                    self.pos = Some(*p);
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
