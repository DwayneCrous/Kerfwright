//! Independent verification of 2.5-D clearing toolpaths: engagement angle of every cut, gouging of the
//! walls, and how much of the reachable material was removed.
//!
//! Nothing here uses the planner's raster. The tool is swept as a *stadium* (the set of points within the
//! tool radius of the path), so the stock before a position is `region ∖ ⋃ stadiums of the earlier moves`,
//! and a point of the tool circle is uncut exactly when it lies in the region and farther than `R` from
//! every earlier move. The engagement is the length of the arc of uncut circle points, with the ends of
//! every arc refined by bisection on that exact predicate, and the circle sampled at a spacing of at most
//! `0.05 R` so that no arc is missed. Distances to segments are analytic.
//!
//! The same author wrote the planner and this checker (a third-party simulator was not available), so the
//! check is independent of the planner's *method*, not of its author.

use crate::geom2d::Region;
use crate::path::Event;
use crate::P2;
use cavalier_contours::core::math::Vector2;
use cavalier_contours::polyline::PlineSource;
use rstar::{RTree, RTreeObject, AABB};

#[derive(Debug, Clone)]
pub struct VerifyParams {
    pub tool_radius: f64,
    pub allowance: f64,
    /// Target engagement angle (radians).
    pub target: f64,
    /// Sampling distance along the path as a fraction of `R`.
    pub path_step: f64,
    /// Number of seeded random points used for the coverage estimate.
    pub coverage_samples: usize,
}

impl VerifyParams {
    pub fn new(tool_radius: f64, allowance: f64, target: f64) -> VerifyParams {
        VerifyParams {
            tool_radius,
            allowance,
            target,
            path_step: 0.1,
            coverage_samples: 20_000,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Report {
    /// Largest engagement of a counted move divided by the target.
    pub max_engagement_ratio: f64,
    pub max_engagement_deg: f64,
    pub mean_engagement_deg: f64,
    /// Where the largest engagement occurred.
    pub worst_at: P2,
    pub counted_length: f64,
    pub excluded_length: f64,
    /// Excluded length over total cutting length.
    pub excluded_fraction: f64,
    /// Deepest penetration of the tool into the walls (mm), over every cutting position.
    pub gouge: f64,
    /// Fraction of the reachable material that the path removed (seeded random sample).
    pub coverage: f64,
    pub samples: usize,
    pub levels: usize,
    /// The ten largest engagements: (degrees, position, index of the move in its level, sample in the move).
    pub top: Vec<(f64, P2, usize, usize)>,
    /// The points of the move (and the one before it) holding the largest engagement.
    pub worst_move: Vec<P2>,
    pub prev_move: Vec<P2>,
    /// Index in the event list of the move holding the largest engagement.
    pub worst_event: usize,
}

#[derive(Clone, Copy)]
struct Seg {
    a: P2,
    b: P2,
}

impl Seg {
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

struct Boxed {
    seg: Seg,
    r: f64,
}

impl RTreeObject for Boxed {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        let (a, b) = (self.seg.a, self.seg.b);
        AABB::from_corners(
            [a[0].min(b[0]) - self.r, a[1].min(b[1]) - self.r],
            [a[0].max(b[0]) + self.r, a[1].max(b[1]) + self.r],
        )
    }
}

impl PartialEq for Boxed {
    fn eq(&self, o: &Self) -> bool {
        self.seg.a == o.seg.a && self.seg.b == o.seg.b
    }
}

/// One move of the tool in the plane, with whether it counts as an adaptive cut move.
struct Mv {
    pts: Vec<P2>,
    counted: bool,
    ev: usize,
}

fn arc_points(from: P2, to: P2, c: P2, ccw: bool, max_chord: f64) -> Vec<P2> {
    let r = (from[0] - c[0]).hypot(from[1] - c[1]);
    let a0 = (from[1] - c[1]).atan2(from[0] - c[0]);
    let a1 = (to[1] - c[1]).atan2(to[0] - c[0]);
    let mut d = a1 - a0;
    if ccw {
        while d <= 1e-9 {
            d += std::f64::consts::TAU;
        }
    } else {
        while d >= -1e-9 {
            d -= std::f64::consts::TAU;
        }
    }
    let n = ((d.abs() * r / max_chord).ceil() as usize).max(1);
    (0..=n)
        .map(|i| {
            let a = a0 + d * i as f64 / n as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}

/// Split a path of events into levels, each a list of moves; rapid moves only reposition.
fn parse(events: &[Event], marker_level: &str, begin: &str, end: &str, r: f64) -> Vec<Vec<Mv>> {
    let mut levels: Vec<Vec<Mv>> = Vec::new();
    let mut pos: Option<[f64; 3]> = None;
    let mut counted = true;
    for (ei, e) in events.iter().enumerate() {
        match e {
            Event::Comment(c) if c.starts_with(marker_level) => levels.push(Vec::new()),
            Event::Comment(c) if c == begin => counted = false,
            Event::Comment(c) if c == end => counted = true,
            Event::Rapid { p } => pos = Some(*p),
            Event::Feed { p, .. } => {
                if let (Some(q), Some(l)) = (pos, levels.last_mut()) {
                    // a move with a height change is a plunge or ramp, not a cut at the level
                    let flat = (p[2] - q[2]).abs() < 1e-9;
                    l.push(Mv {
                        pts: vec![[q[0], q[1]], [p[0], p[1]]],
                        counted: counted && flat,
                        ev: ei,
                    });
                }
                pos = Some(*p);
            }
            Event::Arc { p, centre, ccw, .. } => {
                if let (Some(q), Some(l)) = (pos, levels.last_mut()) {
                    let flat = (p[2] - q[2]).abs() < 1e-9;
                    l.push(Mv {
                        pts: arc_points([q[0], q[1]], [p[0], p[1]], *centre, *ccw, 0.05 * r),
                        counted: counted && flat,
                        ev: ei,
                    });
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    levels
}

fn dist_to_boundary(region: &Region, q: P2) -> f64 {
    region
        .loops()
        .iter()
        .filter_map(|l| l.closest_point(Vector2::new(q[0], q[1]), 1e-10))
        .map(|c| c.distance)
        .fold(f64::INFINITY, f64::min)
}

/// Verify the cutting at every level of `events` (levels are introduced by `Comment` events starting with
/// `level_marker`; moves between `begin` and `end` comments are not counted for the engagement limit).
pub fn verify(
    region: &Region,
    events: &[Event],
    par: &VerifyParams,
    level_marker: &str,
    begin: &str,
    end: &str,
) -> Report {
    let r = par.tool_radius;
    let levels = parse(events, level_marker, begin, end, r);
    let loops = region.loops();
    let _ = &loops;
    let mut rep = Report {
        levels: levels.len(),
        ..Default::default()
    };
    let mut sum_e = 0.0;
    let mut count_e = 0usize;
    let ncirc = (std::f64::consts::TAU / 0.05).ceil() as usize; // spacing 0.05 R along the circle
    let ds = par.path_step * r;
    for (li, level) in levels.iter().enumerate() {
        let mut tree: RTree<Boxed> = RTree::new();
        let uncut = |tree: &RTree<Boxed>, q: P2| -> bool {
            if !region.contains(q) {
                return false;
            }
            let probe = AABB::from_point([q[0], q[1]]);
            !tree
                .locate_in_envelope_intersecting(&probe)
                .any(|b| b.seg.dist(q) < r * (1.0 - 1e-9))
        };
        let engagement = |tree: &RTree<Boxed>, p: P2| -> f64 {
            let at = |th: f64| uncut(tree, [p[0] + r * th.cos(), p[1] + r * th.sin()]);
            let flags: Vec<bool> = (0..ncirc)
                .map(|k| at(std::f64::consts::TAU * k as f64 / ncirc as f64))
                .collect();
            if flags.iter().all(|f| !*f) {
                return 0.0;
            }
            let dth = std::f64::consts::TAU / ncirc as f64;
            let mut total = 0.0;
            for k in 0..ncirc {
                let (fa, fb) = (flags[k], flags[(k + 1) % ncirc]);
                let th0 = k as f64 * dth;
                match (fa, fb) {
                    (true, true) => total += dth,
                    (false, false) => {}
                    _ => {
                        // bisect the crossing on the exact predicate
                        let (mut lo, mut hi) = (th0, th0 + dth);
                        let lo_val = fa;
                        for _ in 0..40 {
                            let mid = 0.5 * (lo + hi);
                            if at(mid) == lo_val {
                                lo = mid;
                            } else {
                                hi = mid;
                            }
                        }
                        let cross = 0.5 * (lo + hi);
                        total += if fa { cross - th0 } else { th0 + dth - cross };
                    }
                }
            }
            total
        };
        for (mi, mv) in level.iter().enumerate() {
            let mut prev = mv.pts[0];
            let mut si = 0usize;
            // sample the move at spacing ds (and at its ends)
            for w in mv.pts.windows(2) {
                let (a, b) = (w[0], w[1]);
                let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                let n = ((l / ds).ceil() as usize).max(1);
                for i in 1..=n {
                    let t = i as f64 / n as f64;
                    let q = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
                    // gouge: the tool must stay r + allowance away from the walls, inside the region
                    let d = dist_to_boundary(region, q);
                    let pen = if region.contains(q) {
                        (r + par.allowance) - d
                    } else {
                        r + par.allowance + d
                    };
                    rep.gouge = rep.gouge.max(pen);
                    let seg_len = (q[0] - prev[0]).hypot(q[1] - prev[1]);
                    if mv.counted {
                        let e = engagement(&tree, q);
                        rep.samples += 1;
                        sum_e += e;
                        count_e += 1;
                        si += 1;
                        if rep.top.len() < 10
                            || rep.top.last().is_some_and(|t| e.to_degrees() > t.0)
                        {
                            rep.top.push((e.to_degrees(), q, mi, si));
                            rep.top.sort_by(|a, b| b.0.total_cmp(&a.0));
                            rep.top.truncate(10);
                        }
                        let ratio = e / par.target;
                        if ratio > rep.max_engagement_ratio {
                            rep.max_engagement_ratio = ratio;
                            rep.max_engagement_deg = e.to_degrees();
                            rep.worst_at = q;
                            rep.worst_move = mv.pts.clone();
                            rep.worst_event = mv.ev;
                            rep.prev_move = if mi > 0 {
                                level[mi - 1].pts.clone()
                            } else {
                                Vec::new()
                            };
                        }
                        rep.counted_length += seg_len;
                    } else {
                        rep.excluded_length += seg_len;
                    }
                    if seg_len > 0.0 {
                        tree.insert(Boxed {
                            seg: Seg { a: prev, b: q },
                            r,
                        });
                    }
                    prev = q;
                }
            }
            if mv.pts.len() == 1 {
                tree.insert(Boxed {
                    seg: Seg {
                        a: mv.pts[0],
                        b: mv.pts[0],
                    },
                    r,
                });
            }
        }
        // coverage on the last level (all levels repeat the same plan)
        if li + 1 == levels.len() {
            rep.coverage = coverage(region, &tree, par);
        }
    }
    if count_e > 0 {
        rep.mean_engagement_deg = (sum_e / count_e as f64).to_degrees();
    }
    let total = rep.counted_length + rep.excluded_length;
    rep.excluded_fraction = if total > 0.0 {
        rep.excluded_length / total
    } else {
        0.0
    };
    rep
}

/// Fraction of the reachable material (within `R` of the feasible tool centres) that lies within `R` of
/// the path, on a seeded random sample.
fn coverage(region: &Region, tree: &RTree<Boxed>, par: &VerifyParams) -> f64 {
    let Some(b) = region.bounds() else { return 0.0 };
    let r = par.tool_radius;
    let free = region.offset(r + par.allowance);
    let free_loops = free.loops();
    let mut seed = 0x9E3779B97F4A7C15u64;
    let mut rnd = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let (mut reach, mut done) = (0usize, 0usize);
    let mut tried = 0usize;
    while reach < par.coverage_samples && tried < 40 * par.coverage_samples {
        tried += 1;
        let q = [b[0] + rnd() * (b[2] - b[0]), b[1] + rnd() * (b[3] - b[1])];
        if !region.contains(q) {
            continue;
        }
        // reachable: some feasible centre within R (the point is inside the free region or within R of it)
        let reachable = free.contains(q)
            || free_loops
                .iter()
                .filter_map(|l| l.closest_point(Vector2::new(q[0], q[1]), 1e-10))
                .any(|c| c.distance <= r);
        if !reachable {
            continue;
        }
        reach += 1;
        let probe = AABB::from_point([q[0], q[1]]);
        if tree
            .locate_in_envelope_intersecting(&probe)
            .any(|s| s.seg.dist(q) <= r)
        {
            done += 1;
        }
    }
    if reach == 0 {
        0.0
    } else {
        done as f64 / reach as f64
    }
}
