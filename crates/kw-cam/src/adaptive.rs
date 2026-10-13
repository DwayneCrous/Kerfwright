//! Adaptive (constant-engagement) clearing of a pocket.
//!
//! The tool is steered so that the angle over which its flutes are buried in uncut material stays at a
//! target value (set by the radial depth of cut, `a_e = R(1 − cos φ)`), instead of following concentric
//! offsets whose engagement spikes at corners and when loops merge.
//!
//! * The stock and the swept area live on a raster with bilinear sub-cell interpolation: the *material*
//!   field is the signed distance to the pocket boundary (islands included), the *cleared* field the
//!   maximum over all moves of `R − distance to the move`. The engagement of a tool centre is the length
//!   of the arc of its circle that lies in uncut material (material and not cleared), found with the
//!   zero crossings of the interpolated field.
//! * A controller picks the heading of each short step: the heading whose engagement at the new position
//!   equals the target (bisection between the fan of candidate headings that bracket it), within the
//!   feasible region of tool centres. Turning towards the uncut side raises the engagement, turning away
//!   lowers it, so the tool follows the boundary of the cleared region — from the entry helix outwards as a
//!   spiral, round islands, along walls.
//! * When a pass can go no further (no material in reach, or a corner where even the extreme turn leaves the
//!   engagement above the target) the next pass starts where the tool can step from the cleared area into
//!   the uncut material at the target engagement. Links stay down inside the cleared area, otherwise the tool
//!   retracts.
//!
//! Clean-room note: this follows the textbook idea of engagement-angle control; no code or patent text was
//! used. A freedom-to-operate review of adaptive clearing (several patents cover constant-engagement
//! toolpaths) has not been done — see `LICENSES/patent-watch.md`.

use crate::geom2d::Region;
use crate::ops2d::{end_program, start_program, Cut};
use crate::path::{Event, Toolpath};
use crate::tool::Tool;
use crate::P2;
use cavalier_contours::core::math::Vector2;
use cavalier_contours::polyline::PlineSource;
use rstar::{RTree, RTreeObject, AABB};

#[derive(Debug, Clone)]
pub struct AdaptiveParams {
    /// Target engagement angle (radians); `None` derives it from the cut's stepover: `arccos(1 − a_e/R)`.
    pub target: Option<f64>,
    /// Length of a controller step as a fraction of the tool radius.
    pub step: f64,
    /// Raster cell as a fraction of the tool radius.
    pub cell: f64,
    /// Largest change of heading per step (radians).
    pub max_turn: f64,
    /// Smallest uncut area (in units of `R · a_e`) worth another pass.
    pub min_leftover: f64,
    /// Radius of the entry helix as a fraction of the tool radius.
    pub helix: f64,
    /// Add a finishing contour at the final depth.
    pub finish: bool,
    /// Tolerance of the polyline simplification (mm).
    pub simplify: f64,
}

impl Default for AdaptiveParams {
    fn default() -> Self {
        AdaptiveParams {
            target: None,
            step: 0.1,
            cell: 0.1,
            max_turn: 25f64.to_radians(),
            min_leftover: 1.0,
            helix: 0.7,
            finish: true,
            simplify: 0.004,
        }
    }
}

/// One continuous cut of tool-centre positions (a polyline), with how it was produced.
#[derive(Debug, Clone)]
pub struct Pass {
    pub points: Vec<P2>,
    /// Why the pass ended (diagnostics).
    pub ended: &'static str,
    /// The pass ended because even the extreme turn left the engagement above the target (a corner).
    pub corner_limited: bool,
    /// The tool can stay down on the way here from the end of the previous pass: the straight link lies
    /// where the tool touches no uncut material.
    pub link_down: bool,
}

/// The planar plan of one depth level.
#[derive(Debug, Clone)]
pub struct Plan2d {
    pub entry: P2,
    pub helix_radius: f64,
    /// The radial move out of the helix to the first contact with the stock (part of the entry).
    pub seed: Vec<P2>,
    pub passes: Vec<Pass>,
    /// Target engagement angle used (radians).
    pub target: f64,
    /// Reachable material left uncut (mm²).
    pub leftover_area: f64,
    /// Reachable material at the start (mm²).
    pub reachable_area: f64,
}

/// A swept move for the exact engagement check.
struct SegBox {
    a: P2,
    b: P2,
    r: f64,
}

impl SegBox {
    fn dist(&self, q: P2) -> f64 {
        let (dx, dy) = (self.b[0] - self.a[0], self.b[1] - self.a[1]);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 {
            (((q[0] - self.a[0]) * dx + (q[1] - self.a[1]) * dy) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (q[0] - (self.a[0] + t * dx)).hypot(q[1] - (self.a[1] + t * dy))
    }
}

impl RTreeObject for SegBox {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [
                self.a[0].min(self.b[0]) - self.r,
                self.a[1].min(self.b[1]) - self.r,
            ],
            [
                self.a[0].max(self.b[0]) + self.r,
                self.a[1].max(self.b[1]) + self.r,
            ],
        )
    }
}

struct Grid {
    region: Region,
    tree: RTree<SegBox>,
    x0: f64,
    y0: f64,
    h: f64,
    nx: usize,
    ny: usize,
    /// Signed distance to the boundary of the region (positive inside).
    mat: Vec<f64>,
    /// `mat − (R + allowance)`: positive where the tool centre may be.
    free: Vec<f64>,
    /// Cleared field: positive where the tool has been.
    clr: Vec<f64>,
    r: f64,
}

impl Grid {
    fn new(region: &Region, r: f64, allowance: f64, h: f64) -> Grid {
        let [x0, y0, x1, y1] = region.bounds().unwrap_or([0.0, 0.0, 1.0, 1.0]);
        let margin = r + 3.0 * h;
        let (x0, y0) = (x0 - margin, y0 - margin);
        let nx = ((x1 + margin - x0) / h).ceil() as usize + 2;
        let ny = ((y1 + margin - y0) / h).ceil() as usize + 2;
        let loops = region.loops();
        let mut mat = vec![0.0; nx * ny];
        for j in 0..ny {
            for i in 0..nx {
                let (x, y) = (x0 + i as f64 * h, y0 + j as f64 * h);
                let mut d = f64::INFINITY;
                for l in &loops {
                    if let Some(c) = l.closest_point(Vector2::new(x, y), 1e-9) {
                        d = d.min(c.distance);
                    }
                }
                mat[j * nx + i] = if region.contains([x, y]) { d } else { -d };
            }
        }
        let free = mat.iter().map(|m| m - (r + allowance)).collect();
        Grid {
            region: region.clone(),
            tree: RTree::new(),
            x0,
            y0,
            h,
            nx,
            ny,
            mat,
            free,
            clr: vec![-2.0 * r; nx * ny],
            r,
        }
    }

    fn bil(&self, f: &[f64], x: f64, y: f64) -> f64 {
        let gx = ((x - self.x0) / self.h).clamp(0.0, (self.nx - 1) as f64 - 1e-9);
        let gy = ((y - self.y0) / self.h).clamp(0.0, (self.ny - 1) as f64 - 1e-9);
        let (i, j) = (gx.floor() as usize, gy.floor() as usize);
        let (fx, fy) = (gx - i as f64, gy - j as f64);
        let at = |a: usize, b: usize| f[b * self.nx + a];
        at(i, j) * (1.0 - fx) * (1.0 - fy)
            + at(i + 1, j) * fx * (1.0 - fy)
            + at(i, j + 1) * (1.0 - fx) * fy
            + at(i + 1, j + 1) * fx * fy
    }

    /// Positive where there is material that has not been cut.
    fn uncut(&self, x: f64, y: f64) -> f64 {
        self.bil(&self.mat, x, y).min(-self.bil(&self.clr, x, y))
    }

    fn feasible(&self, p: P2) -> bool {
        // a few microns of margin: the interpolated field is not exact at the corners of the boundary
        self.bil(&self.free, p[0], p[1]) > 0.002 * self.r
    }

    /// Record the sweep of the tool along the segment `a`–`b`.
    fn sweep(&mut self, a: P2, b: P2) {
        let r = self.r;
        self.tree.insert(SegBox { a, b, r });
        let (xl, xh) = (a[0].min(b[0]) - r - self.h, a[0].max(b[0]) + r + self.h);
        let (yl, yh) = (a[1].min(b[1]) - r - self.h, a[1].max(b[1]) + r + self.h);
        let i0 = (((xl - self.x0) / self.h).floor().max(0.0)) as usize;
        let i1 = (((xh - self.x0) / self.h).ceil() as usize).min(self.nx - 1);
        let j0 = (((yl - self.y0) / self.h).floor().max(0.0)) as usize;
        let j1 = (((yh - self.y0) / self.h).ceil() as usize).min(self.ny - 1);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        for j in j0..=j1 {
            for i in i0..=i1 {
                let (x, y) = (self.x0 + i as f64 * self.h, self.y0 + j as f64 * self.h);
                let t = if l2 > 0.0 {
                    (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let d = (x - (a[0] + t * dx)).hypot(y - (a[1] + t * dy));
                let v = r - d;
                let k = j * self.nx + i;
                if v > self.clr[k] {
                    self.clr[k] = v;
                }
            }
        }
    }

    /// Engagement angle at tool centre `p`: the angular measure of the tool circle in uncut material,
    /// and the direction towards the middle of the engaged arcs.
    fn engagement(&self, p: P2, n: usize) -> (f64, Option<P2>) {
        let r = self.r;
        let dth = std::f64::consts::TAU / n as f64;
        let mut prev = {
            let (x, y) = (p[0] + r, p[1]);
            self.uncut(x, y)
        };
        let first = prev;
        let mut total = 0.0;
        let (mut sx, mut sy) = (0.0, 0.0);
        for k in 1..=n {
            let th = if k == n { 0.0 } else { k as f64 * dth };
            let cur = if k == n {
                first
            } else {
                self.uncut(p[0] + r * th.cos(), p[1] + r * th.sin())
            };
            let (a, b) = (prev, cur);
            // fraction of the interval [θ_{k−1}, θ_k] with positive value (linear interpolation)
            let frac = if a > 0.0 && b > 0.0 {
                1.0
            } else if a <= 0.0 && b <= 0.0 {
                0.0
            } else if a > 0.0 {
                a / (a - b)
            } else {
                b / (b - a)
            };
            if frac > 0.0 {
                let mid = (k as f64 - 0.5) * dth;
                total += frac * dth;
                sx += frac * dth * mid.cos();
                sy += frac * dth * mid.sin();
            }
            prev = cur;
        }
        let dir = if total > 0.0 {
            let l = sx.hypot(sy);
            if l > 1e-9 {
                Some([sx / l, sy / l])
            } else {
                None
            }
        } else {
            None
        };
        (total, dir)
    }

    /// Engagement at `p` from the exact geometry (stadium distances to every move so far): the circle
    /// sampled at 0.05 R, the ends of each arc refined by bisection. The raster steers; this decides.
    fn exact_engagement(&self, p: P2) -> f64 {
        let r = self.r;
        let at = |th: f64| {
            let q = [p[0] + r * th.cos(), p[1] + r * th.sin()];
            if !self.region.contains(q) {
                return false;
            }
            !self
                .tree
                .locate_in_envelope_intersecting(&AABB::from_point(q))
                .any(|s| s.dist(q) < r * (1.0 - 1e-9))
        };
        let n = 126;
        let dth = std::f64::consts::TAU / n as f64;
        let flags: Vec<bool> = (0..n).map(|k| at(k as f64 * dth)).collect();
        if flags.iter().all(|f| !*f) {
            return 0.0;
        }
        let mut total = 0.0;
        for k in 0..n {
            let (fa, fb) = (flags[k], flags[(k + 1) % n]);
            let th0 = k as f64 * dth;
            match (fa, fb) {
                (true, true) => total += dth,
                (false, false) => {}
                _ => {
                    let (mut lo, mut hi) = (th0, th0 + dth);
                    for _ in 0..30 {
                        let mid = 0.5 * (lo + hi);
                        if at(mid) == fa {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    let c = 0.5 * (lo + hi);
                    total += if fa { c - th0 } else { th0 + dth - c };
                }
            }
        }
        total
    }

    /// Uncut material within reach of the tool (mm²), counted on the raster.
    fn reachable_uncut(&self) -> f64 {
        let mut n = 0usize;
        for k in 0..self.mat.len() {
            // material that some feasible centre is within R of: the free field is at least −R there
            if self.mat[k] > 0.0 && self.free[k] >= -self.r * 0.999 && self.clr[k] <= 0.0 {
                n += 1;
            }
        }
        n as f64 * self.h * self.h
    }
}

enum Step {
    /// Moved to `p` with heading `psi`, engagement `e`.
    Moved { p: P2, psi: f64, e: f64 },
    /// Nowhere to go: the extreme turn leaves the engagement above the target.
    Trapped,
    /// No feasible heading at all.
    Blocked,
}

fn advance(p: P2, psi: f64, s: f64) -> P2 {
    [p[0] + s * psi.cos(), p[1] + s * psi.sin()]
}

/// Choose the heading of the next step so that the engagement at the new position is the target.
fn controller(g: &Grid, p: P2, psi: f64, s: f64, target: f64, max_turn: f64) -> Step {
    // arriving at a wall head-on leaves no feasible heading within the normal turn: widen it so that the
    // tool can turn to follow the wall
    for turn in [
        max_turn,
        3.0 * max_turn,
        std::f64::consts::FRAC_PI_2,
        0.9 * std::f64::consts::PI,
    ] {
        match controller_in(g, p, psi, s, target, turn) {
            Step::Blocked => continue,
            other => return other,
        }
    }
    Step::Blocked
}

fn controller_in(g: &Grid, p: P2, psi: f64, s: f64, target: f64, max_turn: f64) -> Step {
    const K: usize = 32;
    let mut cand: Vec<(f64, bool, f64)> = Vec::with_capacity(K + 1); // (δ, feasible, engagement)
    for k in 0..=K {
        let d = -max_turn + 2.0 * max_turn * k as f64 / K as f64;
        let q = advance(p, psi + d, s);
        if g.feasible(q) {
            cand.push((d, true, g.engagement(q, 360).0));
        } else {
            cand.push((d, false, f64::NAN));
        }
    }
    if !cand.iter().any(|c| c.1) {
        return Step::Blocked;
    }
    // bracket a crossing of the target between neighbouring feasible candidates; the smallest turn wins
    let mut best: Option<(f64, f64, f64)> = None; // (|δ|, δ_a, δ_b)
    for w in cand.windows(2) {
        if w[0].1 && w[1].1 && (w[0].2 - target) * (w[1].2 - target) <= 0.0 {
            let mid = 0.5 * (w[0].0 + w[1].0);
            if best.is_none_or(|b| mid.abs() < b.0) {
                best = Some((mid.abs(), w[0].0, w[1].0));
            }
        }
    }
    let delta = if let Some((_, mut a, mut b)) = best {
        let mut ea = g.engagement(advance(p, psi + a, s), 720).0 - target;
        for _ in 0..24 {
            let m = 0.5 * (a + b);
            let q = advance(p, psi + m, s);
            if !g.feasible(q) {
                break;
            }
            let em = g.engagement(q, 720).0 - target;
            if (em < 0.0) == (ea < 0.0) {
                a = m;
                ea = em;
            } else {
                b = m;
            }
        }
        0.5 * (a + b)
    } else {
        // no crossing: closest to the target among the feasible candidates, preferring not to exceed it
        let feas: Vec<&(f64, bool, f64)> = cand.iter().filter(|c| c.1).collect();
        if feas.iter().all(|c| c.2 > target) {
            let c = feas
                .iter()
                .min_by(|a, b| a.2.total_cmp(&b.2))
                .expect("a feasible candidate");
            if c.2 > 1.05 * target {
                return Step::Trapped;
            }
            c.0
        } else {
            feas.iter()
                .max_by(|a, b| a.2.total_cmp(&b.2))
                .expect("a feasible candidate")
                .0
        }
    };
    let q = advance(p, psi + delta, s);
    if !g.feasible(q) {
        return Step::Blocked;
    }
    let mut delta = delta;
    // the engagement of a step is the larger of the values at its end and at its middle: on a turn the
    // middle can be the worse of the two
    let step_e = |d: f64| -> f64 {
        g.exact_engagement(advance(p, psi + d, s))
            .max(g.exact_engagement(advance(p, psi + d, 0.5 * s)))
    };
    let mut e = step_e(delta);
    if (e - target).abs() > 0.02 * target && e > 0.0 || e > 1.03 * target {
        // the raster was off here: find the heading on the exact function, around the raster's choice
        let span = 4.0 * max_turn / K as f64;
        let eval = |d: f64| -> Option<f64> {
            let q = advance(p, psi + d, s);
            g.feasible(q).then(|| step_e(d))
        };
        let (mut lo, mut hi) = ((delta - span).max(-max_turn), (delta + span).min(max_turn));
        // widen until the target is bracketed (engagement rises with the turn towards the material)
        let (mut elo, mut ehi) = (eval(lo), eval(hi));
        for _ in 0..4 {
            if let (Some(a), Some(b)) = (elo, ehi) {
                if (a - target) * (b - target) <= 0.0 {
                    break;
                }
                // not bracketed: look further in the direction that moves the engagement towards the target
                let towards_hi = (b - target).abs() < (a - target).abs();
                if towards_hi {
                    lo = hi;
                    hi = (hi + 2.0 * span).min(max_turn);
                    elo = ehi;
                    ehi = eval(hi);
                } else {
                    hi = lo;
                    lo = (lo - 2.0 * span).max(-max_turn);
                    ehi = elo;
                    elo = eval(lo);
                }
            } else {
                break;
            }
        }
        if let (Some(a), Some(b)) = (elo, ehi) {
            if (a - target) * (b - target) <= 0.0 {
                let (mut x, mut y, mut ex) = (lo, hi, a - target);
                for _ in 0..22 {
                    let m = 0.5 * (x + y);
                    let Some(em) = eval(m) else { break };
                    if (em - target < 0.0) == (ex < 0.0) {
                        x = m;
                        ex = em - target;
                    } else {
                        y = m;
                    }
                }
                delta = 0.5 * (x + y);
                let q2 = advance(p, psi + delta, s);
                if g.feasible(q2) {
                    e = g.exact_engagement(q2);
                    return Step::Moved {
                        p: q2,
                        psi: psi + delta,
                        e,
                    };
                }
            }
        }
        if e > 1.04 * target {
            return Step::Trapped;
        }
    }
    Step::Moved {
        p: q,
        psi: psi + delta,
        e,
    }
}

/// Run one pass from `start` with heading `psi0` until the tool has nothing left to cut or is trapped.
fn run_pass(
    g: &mut Grid,
    start: P2,
    psi0: f64,
    s: f64,
    target: f64,
    max_turn: f64,
    max_steps: usize,
) -> Pass {
    let mut pts = vec![start];
    let (mut p, mut psi) = (start, psi0);
    let mut idle = 0usize;
    let mut corner = false;
    let mut ended = "step limit";
    for _ in 0..max_steps {
        match controller(g, p, psi, s, target, max_turn) {
            Step::Moved { p: q, psi: nps, e } => {
                if e < 0.15 * target {
                    idle += 1;
                    // no material in reach for about a tool diameter of travel: the pass is over
                    if idle as f64 * s > 1.0 * g.r {
                        ended = "no material in reach";
                        break;
                    }
                } else {
                    idle = 0;
                }
                g.sweep(p, q);
                pts.push(q);
                p = q;
                psi = nps;
            }
            Step::Trapped => {
                corner = true;
                ended = "trapped";
                break;
            }
            Step::Blocked => {
                ended = "blocked";
                break;
            }
        }
    }
    // (the idle tail stays: its sweep is already in the stock model, so it is part of the path)
    Pass {
        points: pts,
        ended,
        corner_limited: corner,
        link_down: false,
    }
}

/// A point where the tool, inside the cleared area, touches uncut material at the target engagement,
/// nearest to `from`, with the heading that keeps the uncut side on the right.
fn find_entry(g: &Grid, from: P2, full_target: f64, nogo: &[(P2, f64)]) -> Option<(P2, f64)> {
    // enter a little below the target engagement: the controller then raises it, so that the transient of
    // the first steps cannot overshoot
    let target = 0.92 * full_target;
    // uncut material within reach, as a summed-area table, so that entries next to big uncut regions win
    let w = g.nx + 1;
    let mut sat = vec![0u32; w * (g.ny + 1)];
    for j in 0..g.ny {
        let mut row = 0u32;
        for i in 0..g.nx {
            let k = j * g.nx + i;
            if g.mat[k] > 0.0 && g.clr[k] <= 0.0 && g.free[k] >= -g.r * 0.999 {
                row += 1;
            }
            sat[(j + 1) * w + i + 1] = sat[j * w + i + 1] + row;
        }
    }
    let mass = |p: P2| -> f64 {
        let rad = (2.0 * g.r / g.h).ceil() as isize;
        let ci = ((p[0] - g.x0) / g.h).round() as isize;
        let cj = ((p[1] - g.y0) / g.h).round() as isize;
        let (i0, i1) = (
            (ci - rad).max(0) as usize,
            ((ci + rad + 1).min(g.nx as isize)) as usize,
        );
        let (j0, j1) = (
            (cj - rad).max(0) as usize,
            ((cj + rad + 1).min(g.ny as isize)) as usize,
        );
        let v = sat[j1 * w + i1] + sat[j0 * w + i0] - sat[j0 * w + i1] - sat[j1 * w + i0];
        v as f64 * g.h * g.h
    };
    let mut best: Option<(f64, P2, P2)> = None; // (score, centre, direction); larger is better
    for j in 0..g.ny {
        for i in 0..g.nx {
            let k = j * g.nx + i;
            if g.free[k] <= 1e-9 || g.clr[k] <= 0.0 {
                continue;
            }
            let p = [g.x0 + i as f64 * g.h, g.y0 + j as f64 * g.h];
            if nogo
                .iter()
                .any(|(c, rr)| (p[0] - c[0]).hypot(p[1] - c[1]) < *rr)
            {
                continue;
            }
            let d = (p[0] - from[0]).hypot(p[1] - from[1]);
            let score = mass(p) / (1.0 + d / (6.0 * g.r));
            if best.is_some_and(|b| score <= b.0) {
                continue;
            }
            // quick reject: no uncut material within reach of the circle
            let probe = (0..16).any(|q| {
                let th = q as f64 * std::f64::consts::TAU / 16.0;
                g.uncut(p[0] + g.r * th.cos(), p[1] + g.r * th.sin()) > 0.0
            });
            if !probe {
                continue;
            }
            let (e, dir) = g.engagement(p, 360);
            if e <= 0.0 {
                continue;
            }
            let Some(m) = dir else { continue };
            // slide along the direction to the uncut side until the engagement is the target
            let eval = |t: f64| {
                let q = [p[0] + t * m[0], p[1] + t * m[1]];
                if g.feasible(q) && g.bil(&g.clr, q[0], q[1]) > 0.0 {
                    Some((g.engagement(q, 720).0, q))
                } else {
                    None
                }
            };
            let (mut lo, mut hi) = (-3.0 * g.h, 3.0 * g.h);
            let (Some((elo, _)), Some((ehi, _))) = (eval(lo), eval(hi)) else {
                continue;
            };
            if !(elo <= target && ehi >= target) {
                // engagement is not monotone here, or the target is out of reach: try the node as it is
                if e >= 0.9 * target && e <= 1.02 * target {
                    best = Some((score, p, m));
                }
                continue;
            }
            for _ in 0..30 {
                let mid = 0.5 * (lo + hi);
                match eval(mid) {
                    Some((em, _)) if em < target => lo = mid,
                    Some(_) => hi = mid,
                    None => break,
                }
            }
            let t = 0.5 * (lo + hi);
            if let Some((e2, q)) = eval(t) {
                if (e2 - target).abs() <= 0.03 * target {
                    best = Some((score, q, m));
                }
            }
        }
    }
    best.map(|(_, p, m)| {
        let (_, dir) = g.engagement(p, 720);
        let m = dir.unwrap_or(m);
        // settle the start on the exact engagement, sliding along the direction to the uncut side
        let exact = |t: f64| {
            let q = [p[0] + t * m[0], p[1] + t * m[1]];
            (g.feasible(q) && g.bil(&g.clr, q[0], q[1]) > 0.0).then(|| (g.exact_engagement(q), q))
        };
        let mut p = p;
        let (mut lo, mut hi) = (-2.5 * g.h, 2.5 * g.h);
        if let (Some((elo, _)), Some((ehi, _))) = (exact(lo), exact(hi)) {
            if elo <= target && ehi >= target {
                for _ in 0..28 {
                    let mid = 0.5 * (lo + hi);
                    match exact(mid) {
                        Some((em, _)) if em < target => lo = mid,
                        Some(_) => hi = mid,
                        None => break,
                    }
                }
                if let Some((_, q)) = exact(0.5 * (lo + hi)) {
                    p = q;
                }
            }
        }
        // uncut material on the right: heading is the direction to it turned by +90°
        (p, m[1].atan2(m[0]) + std::f64::consts::FRAC_PI_2)
    })
}

/// Plan the tool-centre moves of one depth level.
pub fn plan(
    region: &Region,
    tool_radius: f64,
    allowance: f64,
    stepover: f64,
    par: &AdaptiveParams,
) -> Plan2d {
    let r = tool_radius;
    let target = par
        .target
        .unwrap_or_else(|| (1.0 - stepover / r).clamp(-1.0, 1.0).acos());
    let h = par.cell * r;
    let s = par.step * r;
    let mut g = Grid::new(region, r, allowance, h);
    let reachable = g.reachable_uncut();
    // entry: the deepest point of the feasible region
    let mut best = (f64::MIN, [0.0, 0.0]);
    for j in 0..g.ny {
        for i in 0..g.nx {
            let v = g.free[j * g.nx + i];
            if v > best.0 {
                best = (v, [g.x0 + i as f64 * g.h, g.y0 + j as f64 * g.h]);
            }
        }
    }
    let mut plan = Plan2d {
        entry: best.1,
        helix_radius: 0.0,
        seed: Vec::new(),
        passes: Vec::new(),
        target,
        leftover_area: reachable,
        reachable_area: reachable,
    };
    if best.0 <= 0.0 {
        return plan; // the tool does not fit
    }
    let entry = best.1;
    let helix_r = (par.helix * r).min(0.9 * best.0).max(0.0);
    plan.helix_radius = helix_r;
    // the helix clears a disc of radius helix_r + R about the entry point
    let n_h = 48;
    for k in 0..n_h {
        let a0 = std::f64::consts::TAU * k as f64 / n_h as f64;
        let a1 = std::f64::consts::TAU * (k + 1) as f64 / n_h as f64;
        g.sweep(
            [entry[0] + helix_r * a0.cos(), entry[1] + helix_r * a0.sin()],
            [entry[0] + helix_r * a1.cos(), entry[1] + helix_r * a1.sin()],
        );
    }
    // seed: straight out from the end of the helix until the target engagement is reached
    let mut p = [entry[0] + helix_r, entry[1]];
    let mut seed = vec![p];
    for _ in 0..((4.0 * r / s) as usize + 4) {
        let q = [p[0] + s, p[1]];
        if !g.feasible(q) {
            break;
        }
        g.sweep(p, q);
        seed.push(q);
        p = q;
        if g.exact_engagement(p) >= 0.97 * target {
            break;
        }
    }
    // steer a little below the target so that the slack of the discretisation stays under it
    let aim = 0.97 * target;
    let max_steps =
        ((region.area().abs() / (s * stepover.max(1e-3))) as usize * 6).clamp(2000, 400_000);
    let first = run_pass(
        &mut g,
        p,
        std::f64::consts::FRAC_PI_2,
        s,
        aim,
        par.max_turn,
        max_steps,
    );
    plan.seed = seed;
    plan.passes.push(first);
    let min_area = par.min_leftover * r * stepover;
    let mut from = *plan.passes[0].points.last().expect("a point");
    let mut nogo: Vec<(P2, f64)> = Vec::new();
    for _ in 0..4000 {
        if g.reachable_uncut() < min_area {
            break;
        }
        let Some((start, psi)) = find_entry(&g, from, target, &nogo) else {
            break;
        };
        let link_down = link_is_clear(&g, from, start);
        let before = g.reachable_uncut();
        let mut pass = run_pass(&mut g, start, psi, s, aim, par.max_turn, max_steps);
        if before - g.reachable_uncut() < 0.5 * min_area || pass.corner_limited {
            // a sliver or a wedge the target engagement cannot reach: do not start near there again
            nogo.push((start, 0.6 * r));
        }
        if pass.points.len() < 2 {
            continue;
        }
        pass.link_down = link_down;
        from = *pass.points.last().expect("a point");
        plan.passes.push(pass);
    }
    plan.leftover_area = g.reachable_uncut();
    plan
}

fn dp(points: &[P2], tol: f64) -> Vec<P2> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0usize, points.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let (pa, pb) = (points[a], points[b]);
        let (dx, dy) = (pb[0] - pa[0], pb[1] - pa[1]);
        let l = dx.hypot(dy).max(1e-12);
        let (mut worst, mut wi) = (0.0, a);
        for i in a + 1..b {
            let d = ((points[i][0] - pa[0]) * dy - (points[i][1] - pa[1]) * dx).abs() / l;
            if d > worst {
                worst = d;
                wi = i;
            }
        }
        if worst > tol {
            keep[wi] = true;
            stack.push((a, wi));
            stack.push((wi, b));
        }
    }
    points
        .iter()
        .zip(&keep)
        .filter(|(_, k)| **k)
        .map(|(p, _)| *p)
        .collect()
}

/// Comment markers: moves between `EXCLUDE_BEGIN` and `EXCLUDE_END` are not adaptive cut moves (they are
/// entries, links and finishing passes, or segments the controller could not hold to the target).
pub const EXCLUDE_BEGIN: &str = "adaptive: not counted (begin)";
pub const EXCLUDE_END: &str = "adaptive: not counted (end)";
pub const LEVEL: &str = "adaptive: level";

/// Adaptive clearing of `region` at every depth level of `cut`, with `cut.stepover` as the radial depth of
/// cut (so the target engagement is `arccos(1 − stepover/R)`), then an optional finishing contour.
pub fn adaptive(
    region: &Region,
    tool: &Tool,
    cut: &Cut,
    par: &AdaptiveParams,
) -> (Toolpath, Plan2d) {
    let r = tool.radius();
    let pl = plan(region, r, cut.wall_allowance, cut.stepover, par);
    let mut tp = Toolpath::new("adaptive clearing");
    start_program(&mut tp, tool, cut);
    let passes: Vec<Vec<P2>> = pl
        .passes
        .iter()
        .map(|p| dp(&p.points, par.simplify))
        .collect();
    let mut z_prev = cut.top;
    let safe = cut.top + cut.clearance;
    let near = |z: f64| z + 1.0_f64.min(cut.clearance);
    for z in cut.levels() {
        tp.push(Event::Comment(format!("{LEVEL} z={z:.4}")));
        if passes.is_empty() {
            break;
        }
        tp.push(Event::Comment(EXCLUDE_BEGIN.into()));
        // entry: helix about the entry point down to the level
        let e = pl.entry;
        let hr = pl.helix_radius;
        let start = [e[0] + hr, e[1]];
        tp.push(Event::Rapid {
            p: [start[0], start[1], safe],
        });
        tp.push(Event::Rapid {
            p: [start[0], start[1], near(z_prev)],
        });
        let dz = near(z_prev) - z;
        if hr > 0.05 * r {
            let per_turn = (std::f64::consts::TAU * hr * 3f64.to_radians().tan()).max(1e-6);
            let turns = (dz / per_turn).ceil().max(1.0) as usize;
            let halves = 2 * turns;
            for k in 0..halves {
                let ang = std::f64::consts::PI * (k + 1) as f64;
                let ze = near(z_prev) - dz * (k + 1) as f64 / halves as f64;
                tp.push(Event::Arc {
                    p: [e[0] + hr * ang.cos(), e[1] + hr * ang.sin(), ze],
                    centre: e,
                    ccw: true,
                    feed: cut.plunge_feed.min(cut.feed),
                });
            }
            // a flat lap at the final depth cleans the ramp
            for k in 0..2 {
                let ang = std::f64::consts::PI * (k + 1) as f64;
                tp.push(Event::Arc {
                    p: [e[0] + hr * ang.cos(), e[1] + hr * ang.sin(), z],
                    centre: e,
                    ccw: true,
                    feed: cut.feed,
                });
            }
        } else {
            tp.push(Event::Feed {
                p: [start[0], start[1], z],
                feed: cut.plunge_feed,
            });
        }
        for q in pl.seed.iter().skip(1) {
            tp.push(Event::Feed {
                p: [q[0], q[1], z],
                feed: cut.feed,
            });
        }
        tp.push(Event::Comment(EXCLUDE_END.into()));
        let mut pos = pl.seed.last().copied().unwrap_or(start);
        for (k, pts) in passes.iter().enumerate() {
            if pts.len() < 2 {
                continue;
            }
            let first = pts[0];
            if k > 0 || first != pos {
                // link: straight and down inside the cleared area when possible, otherwise retract
                tp.push(Event::Comment(EXCLUDE_BEGIN.into()));
                if pl.passes[k].link_down {
                    tp.push(Event::Feed {
                        p: [first[0], first[1], z],
                        feed: cut.feed,
                    });
                } else {
                    tp.push(Event::Rapid {
                        p: [pos[0], pos[1], safe],
                    });
                    tp.push(Event::Rapid {
                        p: [first[0], first[1], safe],
                    });
                    tp.push(Event::Rapid {
                        p: [first[0], first[1], near(z)],
                    });
                    tp.push(Event::Feed {
                        p: [first[0], first[1], z],
                        feed: cut.plunge_feed,
                    });
                }
                tp.push(Event::Comment(EXCLUDE_END.into()));
            }
            if pl.passes[k].corner_limited && k + 1 == passes.len() {
                tp.push(Event::Comment(EXCLUDE_BEGIN.into()));
            }
            for q in &pts[1..] {
                tp.push(Event::Feed {
                    p: [q[0], q[1], z],
                    feed: cut.feed,
                });
            }
            if pl.passes[k].corner_limited && k + 1 == passes.len() {
                tp.push(Event::Comment(EXCLUDE_END.into()));
            }
            pos = *pts.last().expect("a point");
        }
        z_prev = z;
    }
    if par.finish {
        // finishing contour at the final depth along the pocket walls
        let mut fin = cut.clone();
        fin.top = cut.bottom + cut.step_down.min(cut.top - cut.bottom);
        fin.wall_allowance = 0.0;
        let c = crate::ops2d::contour(region, tool, crate::ops2d::Side::Inside, &fin);
        tp.push(Event::Comment(EXCLUDE_BEGIN.into()));
        for ev in c.events {
            match ev {
                Event::ToolChange { .. }
                | Event::SpindleOn { .. }
                | Event::CoolantOn
                | Event::CoolantOff
                | Event::SpindleOff
                | Event::ProgramEnd => {}
                e => tp.push(e),
            }
        }
        tp.push(Event::Comment(EXCLUDE_END.into()));
    }
    end_program(&mut tp, cut);
    (tp, pl)
}

/// Can the tool go from `a` to `b` without cutting: every sample of the straight line is a feasible tool
/// centre whose circle touches no uncut material.
fn link_is_clear(g: &Grid, a: P2, b: P2) -> bool {
    let l = (b[0] - a[0]).hypot(b[1] - a[1]);
    let n = ((l / (0.25 * g.r)).ceil() as usize).max(1);
    (0..=n).all(|i| {
        let t = i as f64 / n as f64;
        let q = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        g.feasible(q) && g.exact_engagement(q) <= 1e-6
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tool::Tool;

    #[test]
    fn a_rectangle_is_cleared_at_constant_engagement() {
        let region = Region::rect(0.0, 0.0, 40.0, 30.0);
        let tool = Tool::flat(1, 6.0, 20.0);
        let mut cut = Cut::new(0.0, -3.0);
        cut.stepover = 1.2;
        cut.step_down = 3.0;
        let (tp, pl) = adaptive(&region, &tool, &cut, &AdaptiveParams::default());
        eprintln!(
            "passes {}, target {:.1}°, leftover {:.1} of {:.1} mm², cutting length {:.0} mm",
            pl.passes.len(),
            pl.target.to_degrees(),
            pl.leftover_area,
            pl.reachable_area,
            tp.cutting_length()
        );
        for (k, p) in pl.passes.iter().enumerate() {
            eprintln!(
                "  pass {k}: {} points from {:.2?} to {:.2?}, ended: {}",
                p.points.len(),
                p.points[0],
                p.points[p.points.len() - 1],
                p.ended
            );
        }
        assert!(!pl.passes.is_empty());
        assert!(
            pl.leftover_area < 0.10 * pl.reachable_area,
            "{} of {}",
            pl.leftover_area,
            pl.reachable_area
        );
    }
}
