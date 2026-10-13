//! Deterministic sketch generators with known ground truth, shared by the
//! classification / branch-flip tests and the benchmarks.
#![allow(
    clippy::type_complexity,
    clippy::needless_late_init,
    clippy::manual_inspect
)]

use crate::constraint::*;
use crate::entity::*;
use crate::sketch::Sketch;
use crate::system::System;

pub struct Rng(pub u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut r = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) ^ 0xD1B54A32D192ED03);
        r.unit();
        r
    }
    pub fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unit()
    }
    pub fn int(&mut self, n: usize) -> usize {
        ((self.unit() * n as f64) as usize).min(n - 1)
    }
    pub fn flip(&mut self) -> bool {
        self.unit() < 0.5
    }
    pub fn sign(&mut self) -> f64 {
        if self.flip() {
            1.0
        } else {
            -1.0
        }
    }
}

/// Move every variable not pinned by a `Fix` or dimension by up to `±amp`.
pub fn perturb(sk: &mut Sketch, rng: &mut Rng, amp: f64) {
    let sys = System::build(sk).expect("lowerable");
    for v in 0..sk.vars.values.len() {
        if !sys.fixed[v] {
            sk.vars.values[v] += amp * (rng.unit() - 0.5) * 2.0;
        }
    }
}

// ---------------------------------------------------------------------------
// classification suite
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub enum Truth {
    Under {
        dof: usize,
    },
    Fully,
    /// Consistent but dependent constraints; `flag` must be reported (at least one of them).
    Over {
        flag: Vec<ConstraintId>,
    },
    /// Planted minimal conflicting set.
    Conflict {
        minimal: Vec<ConstraintId>,
    },
}

pub struct ClassCase {
    pub name: String,
    pub sketch: Sketch,
    pub truth: Truth,
}

struct Chain {
    sk: Sketch,
    pts: Vec<EntityId>,
    lines: Vec<EntityId>,
    cons: Vec<ConstraintId>,
    dof: usize,
}

fn rotate(v: (f64, f64), a: f64) -> (f64, f64) {
    (v.0 * a.cos() - v.1 * a.sin(), v.0 * a.sin() + v.1 * a.cos())
}

/// Polyline anchored at a fixed first point; each new point gets an optional
/// length dimension and an optional direction constraint (H, V or an angle to
/// the previous segment), so the degrees of freedom are known exactly.
fn build_chain(rng: &mut Rng, nseg: usize, full: bool) -> Chain {
    let mut sk = Sketch::new();
    let mut pos = (rng.range(-20.0, 20.0), rng.range(-20.0, 20.0));
    let p0 = sk.add_point(pos.0, pos.1);
    let mut ch = Chain {
        sk: Sketch::new(),
        pts: vec![p0],
        lines: vec![],
        cons: vec![],
        dof: 0,
    };
    let c = sk.fix(p0).unwrap();
    ch.cons.push(c);
    let mut prev_dir: Option<(f64, f64)> = None;
    for i in 1..=nseg {
        let len = rng.range(2.0, 10.0);
        let dir_kind = if full { rng.int(3) } else { rng.int(4) }; // 0 H, 1 V, 2 angle (or H), 3 none
        let dir_kind = if i == 1 && dir_kind == 2 { 0 } else { dir_kind };
        let dir = match dir_kind {
            0 => (rng.sign(), 0.0),
            1 => (0.0, rng.sign()),
            2 => {
                let ang = rng.sign() * rng.range(0.35, 2.7);
                rotate(prev_dir.unwrap(), ang)
            }
            _ => {
                let a = rng.range(0.0, std::f64::consts::TAU);
                (a.cos(), a.sin())
            }
        };
        let n = (dir.0 * dir.0 + dir.1 * dir.1).sqrt();
        let dir = (dir.0 / n, dir.1 / n);
        pos = (pos.0 + dir.0 * len, pos.1 + dir.1 * len);
        let pi = sk.add_point(pos.0, pos.1);
        let line = sk.add_line(*ch.pts.last().unwrap(), pi);
        let has_len = full || rng.unit() < 0.7;
        let mut m = 0;
        if has_len {
            ch.cons
                .push(sk.dim_distance(*ch.pts.last().unwrap(), pi).unwrap());
            m += 1;
        }
        match dir_kind {
            0 => {
                ch.cons.push(sk.horizontal(line).unwrap());
                m += 1;
            }
            1 => {
                ch.cons.push(sk.vertical(line).unwrap());
                m += 1;
            }
            2 => {
                ch.cons
                    .push(sk.dim_angle(*ch.lines.last().unwrap(), line).unwrap());
                m += 1;
            }
            _ => {}
        }
        ch.dof += 2 - m;
        ch.pts.push(pi);
        ch.lines.push(line);
        prev_dir = Some(dir);
    }
    ch.sk = sk;
    ch
}

fn finish(name: String, mut sk: Sketch, truth: Truth, rng: &mut Rng) -> ClassCase {
    perturb(&mut sk, rng, 0.25);
    ClassCase {
        name,
        sketch: sk,
        truth,
    }
}

fn chain_cases(out: &mut Vec<ClassCase>, rng: &mut Rng, count: usize) {
    for i in 0..count {
        let nseg = 2 + rng.int(5);
        let ch = build_chain(rng, nseg, i % 3 == 0);
        let truth = if ch.dof == 0 {
            Truth::Fully
        } else {
            Truth::Under { dof: ch.dof }
        };
        out.push(finish(
            format!("chain#{i} nseg={nseg} dof={}", ch.dof),
            ch.sk,
            truth,
            rng,
        ));
    }
}

/// New-line templates with known independent equation counts. Returns the dof added.
fn add_template(
    sk: &mut Sketch,
    rng: &mut Rng,
    t: usize,
    anchor: (EntityId, EntityId, EntityId, EntityId),
) -> (usize, Vec<ConstraintId>) {
    // anchor = (reference line R, fixed point Q, fixed circle, fixed second line R2)
    let (r, q, circ, _r2) = anchor;
    let (rp1, rp2) = sk.line_points(r).unwrap();
    let (a, b) = (sk.point(rp1).unwrap(), sk.point(rp2).unwrap());
    let rdir = (b.0 - a.0, b.1 - a.1);
    let rl = (rdir.0 * rdir.0 + rdir.1 * rdir.1).sqrt();
    let u = (rdir.0 / rl, rdir.1 / rl);
    let nrm = (-u.1, u.0);
    let qp = sk.point(q).unwrap();
    let mut cons = Vec::new();
    let place = |sk: &mut Sketch, p1: (f64, f64), p2: (f64, f64)| {
        let l = sk.add_line_xy(p1.0, p1.1, p2.0, p2.1);
        let (x, y) = sk.line_points(l).unwrap();
        (l, x, y)
    };
    let dof;
    match t {
        // parallel to R
        0 => {
            let o = rng.range(-6.0, 6.0);
            let s = rng.range(-5.0, 5.0);
            let len = rng.range(2.0, 8.0);
            let p1 = (a.0 + u.0 * s + nrm.0 * o, a.1 + u.1 * s + nrm.1 * o);
            let (l, _, _) = place(sk, p1, (p1.0 + u.0 * len, p1.1 + u.1 * len));
            cons.push(sk.parallel(l, r).unwrap());
            dof = 3;
        }
        // perpendicular and equal length
        1 => {
            let p1 = (a.0 + rng.range(-5.0, 5.0), a.1 + rng.range(-5.0, 5.0));
            let (l, _, _) = place(sk, p1, (p1.0 + nrm.0 * rl, p1.1 + nrm.1 * rl));
            cons.push(sk.perpendicular(l, r).unwrap());
            cons.push(sk.equal_length(l, r).unwrap());
            dof = 2;
        }
        // collinear + equal length
        2 => {
            let s = rng.range(-12.0, 12.0);
            let p1 = (a.0 + u.0 * s, a.1 + u.1 * s);
            let (l, _, _) = place(sk, p1, (p1.0 + u.0 * rl, p1.1 + u.1 * rl));
            cons.push(sk.collinear(l, r).unwrap());
            cons.push(sk.equal_length(l, r).unwrap());
            dof = 1;
        }
        // midpoint at Q, parallel to R (+ equal length half the time)
        3 => {
            let len = rl * rng.range(0.5, 1.5);
            let p1 = (qp.0 - u.0 * len / 2.0, qp.1 - u.1 * len / 2.0);
            let (l, _, _) = place(sk, p1, (p1.0 + u.0 * len, p1.1 + u.1 * len));
            cons.push(sk.midpoint(q, l).unwrap());
            cons.push(sk.parallel(l, r).unwrap());
            if rng.flip() {
                cons.push(sk.equal_length(l, r).unwrap());
                dof = 0;
            } else {
                dof = 1;
            }
        }
        // starts at Q, perpendicular to R, equal length
        4 => {
            let (l, s, _) = place(sk, qp, (qp.0 + nrm.0 * rl, qp.1 + nrm.1 * rl));
            cons.push(sk.coincident(s, q).unwrap());
            cons.push(sk.perpendicular(l, r).unwrap());
            cons.push(sk.equal_length(l, r).unwrap());
            dof = 0;
        }
        // chord of the fixed circle parallel to R (+ length)
        5 => {
            let ((cx, cy), rad) = sk.circle_geometry(circ).unwrap();
            let off = rng.range(-0.6, 0.6) * rad;
            let half = (rad * rad - off * off).sqrt();
            let p1 = (cx + nrm.0 * off - u.0 * half, cy + nrm.1 * off - u.1 * half);
            let p2 = (cx + nrm.0 * off + u.0 * half, cy + nrm.1 * off + u.1 * half);
            let (l, x, y) = place(sk, p1, p2);
            cons.push(sk.point_on(x, circ).unwrap());
            cons.push(sk.point_on(y, circ).unwrap());
            cons.push(sk.parallel(l, r).unwrap());
            if rng.flip() {
                cons.push(sk.dim_distance(x, y).unwrap());
                dof = 0;
            } else {
                dof = 1;
            }
        }
        // angle to R and distances of both ends to Q
        _ => {
            let ang = rng.range(0.4, 2.6);
            let d1 = rng.range(3.0, 7.0);
            let d2 = rng.range(7.5, 11.0);
            let w = rotate(u, ang);
            // Q-centred construction keeps the three points far from collinear
            let t0 = rng.range(0.6, 1.2);
            let p1 = (qp.0 + d1 * (t0.cos()), qp.1 + d1 * (t0.sin()));
            let p2 = (p1.0 + w.0 * (d2 - d1 * 0.4), p1.1 + w.1 * (d2 - d1 * 0.4));
            let (l, x, y) = place(sk, p1, p2);
            cons.push(sk.dim_angle(r, l).unwrap());
            cons.push(sk.dim_distance(x, q).unwrap());
            cons.push(sk.dim_distance(y, q).unwrap());
            dof = 1;
        }
    }
    (dof, cons)
}

fn anchor_scene(rng: &mut Rng) -> (Sketch, (EntityId, EntityId, EntityId, EntityId)) {
    let mut sk = Sketch::new();
    let ang = rng.range(0.0, std::f64::consts::TAU);
    let len = rng.range(4.0, 10.0);
    let r = sk.add_line_xy(0.0, 0.0, len * ang.cos(), len * ang.sin());
    let q = sk.add_point(rng.range(8.0, 14.0), rng.range(-6.0, 6.0));
    let circ = sk.add_circle_xy(
        rng.range(-10.0, -4.0),
        rng.range(-8.0, 8.0),
        rng.range(3.0, 6.0),
    );
    let r2 = sk.add_line_xy(-5.0, 14.0, 5.0, 16.0);
    for e in [r, q, circ, r2] {
        sk.fix(e).unwrap();
    }
    (sk, (r, q, circ, r2))
}

fn template_cases(out: &mut Vec<ClassCase>, rng: &mut Rng, count: usize) {
    for i in 0..count {
        let (mut sk, anchor) = anchor_scene(rng);
        let k = 1 + rng.int(3);
        let mut dof = 0;
        let mut names = Vec::new();
        for _ in 0..k {
            let t = rng.int(7);
            names.push(t);
            dof += add_template(&mut sk, rng, t, anchor).0;
        }
        let truth = if dof == 0 {
            Truth::Fully
        } else {
            Truth::Under { dof }
        };
        out.push(finish(
            format!("template#{i} {names:?} dof={dof}"),
            sk,
            truth,
            rng,
        ));
    }
}

/// Fully/under-constrained chain plus a constraint that is dependent but consistent.
fn redundant_cases(out: &mut Vec<ClassCase>, rng: &mut Rng, count: usize) {
    for i in 0..count {
        let nseg = 3 + rng.int(4);
        let mut ch = build_chain(rng, nseg, i % 2 == 0);
        let kind = rng.int(3);
        let added = match kind {
            // duplicate length dimension of an existing segment
            0 => {
                let k = rng.int(nseg);
                // find a segment that has a length dimension: add a fresh one (consistent with geometry)
                let s = ch.sk.dim_distance(ch.pts[k], ch.pts[k + 1]).unwrap();
                let existing = ch.cons.iter().any(|c| {
                    matches!(&ch.sk.constraints[*c].kind, ConstraintKind::Distance { a, b, .. } if *a == ch.pts[k] && *b == ch.pts[k + 1])
                });
                if existing {
                    Some(s)
                } else {
                    // length was free: the dimension is independent, not redundant
                    ch.dof -= 1;
                    None
                }
            }
            // distance between two non-adjacent points of a fully determined chain
            1 if ch.dof == 0 && nseg >= 3 => {
                Some(ch.sk.dim_distance(ch.pts[0], ch.pts[nseg]).unwrap())
            }
            // duplicate fix on the anchor
            _ => ch.sk.fix(ch.pts[0]).ok().map(|c| {
                ch.cons.push(c);
                c
            }),
        };
        // a Fix on an already fixed point adds no rows: that is not "redundant" in the
        // rank sense, so only keep cases where a real dependent row exists
        let Some(added) = added else {
            let truth = if ch.dof == 0 {
                Truth::Fully
            } else {
                Truth::Under { dof: ch.dof }
            };
            out.push(finish(
                format!("redundant#{i} (independent dim)"),
                ch.sk,
                truth,
                rng,
            ));
            continue;
        };
        if matches!(ch.sk.constraints[added].kind, ConstraintKind::Fix(_)) {
            let truth = if ch.dof == 0 {
                Truth::Fully
            } else {
                Truth::Under { dof: ch.dof }
            };
            out.push(finish(
                format!("redundant#{i} (duplicate fix)"),
                ch.sk,
                truth,
                rng,
            ));
            continue;
        }
        out.push(finish(
            format!("redundant#{i} chain dof={}", ch.dof),
            ch.sk,
            Truth::Over { flag: vec![added] },
            rng,
        ));
    }
}

fn conflict_cases(out: &mut Vec<ClassCase>, rng: &mut Rng, count: usize) {
    for i in 0..count {
        let family = i % 8;
        // context: a chain whose last point we attach the conflict to
        let ctx_len = 1 + rng.int(3);
        let mut ch = build_chain(rng, ctx_len, false);
        let a = *ch.pts.last().unwrap();
        let ap = ch.sk.point(a).unwrap();
        let sk = &mut ch.sk;
        let minimal: Vec<ConstraintId>;
        let ang = rng.range(0.0, std::f64::consts::TAU);
        let len = rng.range(3.0, 9.0);
        match family {
            // C1: two different distances between the same pair
            0 => {
                let b = sk.add_point(ap.0 + len * ang.cos(), ap.1 + len * ang.sin());
                let d1 = sk.dim_distance(a, b).unwrap();
                let d2 = sk.dim_distance(a, b).unwrap();
                sk.set_dimension(d2, len * rng.range(1.2, 1.8)).unwrap();
                minimal = vec![d1, d2];
            }
            // C2: horizontal + vertical + length on one line
            1 => {
                let b = sk.add_point(ap.0 + len, ap.1 + 0.3);
                let l = sk.add_line(a, b);
                let h = sk.horizontal(l).unwrap();
                let v = sk.vertical(l).unwrap();
                let d = sk.dim_distance(a, b).unwrap();
                minimal = vec![h, v, d];
            }
            // C3: two different radii for one circle
            2 => {
                let c = sk.add_circle(a, len);
                let r1 = sk.dim_radius(c).unwrap();
                let r2 = sk.dim_radius(c).unwrap();
                sk.set_dimension(r2, len * rng.range(1.3, 2.0)).unwrap();
                minimal = vec![r1, r2];
            }
            // C4: tangent line/circle with radius R but centre distance D ≠ R
            3 => {
                let l = sk.add_line_xy(ap.0 - 15.0, ap.1 - 20.0, ap.0 + 15.0, ap.1 - 20.0);
                let ctr = sk.add_point(ap.0 + 2.0, ap.1 - 20.0 + len);
                let c = sk.add_circle(ctr, len);
                let t = sk.tangent(l, c).unwrap();
                let r = sk.dim_radius(c).unwrap();
                let d = sk.dim_distance(ctr, l).unwrap();
                sk.set_dimension(d, len * rng.range(1.3, 1.9)).unwrap();
                sk.fix(l).unwrap(); // context only: the conflict does not need it
                minimal = vec![t, r, d];
            }
            // C5: two fixed points with a wrong distance
            4 => {
                let b = sk.add_point(ap.0 + len * ang.cos() + 30.0, ap.1 + len * ang.sin());
                let fa = sk.fix(b).unwrap();
                let (p, q) = (
                    sk.add_point(ap.0 + 50.0, ap.1),
                    sk.add_point(ap.0 + 50.0 + len, ap.1),
                );
                let f1 = sk.fix(p).unwrap();
                let f2 = sk.fix(q).unwrap();
                let d = sk.dim_distance(p, q).unwrap();
                sk.set_dimension(d, len * rng.range(1.3, 1.9)).unwrap();
                // keep `b` constrained to the context component too (satisfiable)
                let _ = fa;
                minimal = vec![f1, f2, d];
            }
            // C6: parallel and perpendicular on the same pair of lines
            5 => {
                let l1 = sk.add_line_xy(ap.0, ap.1, ap.0 + len, ap.1 + 1.0);
                let l2 = sk.add_line_xy(ap.0, ap.1 + 4.0, ap.0 + len, ap.1 + 5.5);
                let p = sk.parallel(l1, l2).unwrap();
                let q = sk.perpendicular(l1, l2).unwrap();
                minimal = vec![p, q];
            }
            // C7: two different angle dimensions between the same lines
            6 => {
                let l1 = sk.add_line_xy(ap.0, ap.1, ap.0 + len, ap.1);
                let l2 = sk.add_line_xy(ap.0, ap.1, ap.0 + len * 0.5, ap.1 + len * 0.8);
                let a1 = sk.dim_angle(l1, l2).unwrap();
                let a2 = sk.dim_angle(l1, l2).unwrap();
                let v = sk.dimension_value(a2).unwrap();
                sk.set_dimension(a2, v + rng.range(0.3, 0.8)).unwrap();
                minimal = vec![a1, a2];
            }
            // C8: triangle inequality violated (3, 4, 10)
            _ => {
                let b = sk.add_point(ap.0 + 3.0, ap.1);
                let c = sk.add_point(ap.0 + 3.0, ap.1 + 4.0);
                let d1 = sk.dim_distance(a, b).unwrap();
                let d2 = sk.dim_distance(b, c).unwrap();
                let d3 = sk.dim_distance(a, c).unwrap();
                sk.set_dimension(d1, 3.0).unwrap();
                sk.set_dimension(d2, 4.0).unwrap();
                sk.set_dimension(d3, 10.0).unwrap();
                minimal = vec![d1, d2, d3];
            }
        }
        let mut minimal = minimal;
        minimal.sort();
        out.push(finish(
            format!("conflict#{i} family=C{}", family + 1),
            ch.sk,
            Truth::Conflict { minimal },
            rng,
        ));
    }
}

/// Hand-built special-position cases (generic counting and numeric rank differ).
fn special_cases(out: &mut Vec<ClassCase>, rng: &mut Rng) {
    // 1. parallelogram: opposite sides parallel ⇒ equal-length is redundant
    for k in 0..6 {
        let mut sk = Sketch::new();
        let w = rng.range(4.0, 8.0);
        let h = rng.range(2.0, 5.0);
        let sh = rng.range(0.5, 3.0);
        let p = [
            sk.add_point(0.0, 0.0),
            sk.add_point(w, 0.0),
            sk.add_point(w + sh, h),
            sk.add_point(sh, h),
        ];
        let l: Vec<_> = (0..4).map(|i| sk.add_line(p[i], p[(i + 1) % 4])).collect();
        sk.fix(p[0]).unwrap();
        sk.horizontal(l[0]).unwrap();
        sk.parallel(l[0], l[2]).unwrap();
        sk.parallel(l[1], l[3]).unwrap();
        let _ = sk.dim_distance(p[0], p[1]).unwrap();
        let _ = sk.dim_distance(p[1], p[2]).unwrap();
        let _ = sk.dim_angle(l[0], l[1]).unwrap();
        let red = sk.equal_length(l[0], l[2]).unwrap();
        out.push(finish(
            format!("special parallelogram#{k}"),
            sk,
            Truth::Over { flag: vec![red] },
            rng,
        ));
    }
    // 2. rectangle with both pairs H/V and a perpendicular (dependent)
    for k in 0..6 {
        let mut sk = Sketch::new();
        let (w, h) = (rng.range(3.0, 9.0), rng.range(2.0, 6.0));
        let p = [
            sk.add_point(0.0, 0.0),
            sk.add_point(w, 0.0),
            sk.add_point(w, h),
            sk.add_point(0.0, h),
        ];
        let l: Vec<_> = (0..4).map(|i| sk.add_line(p[i], p[(i + 1) % 4])).collect();
        sk.fix(p[0]).unwrap();
        sk.horizontal(l[0]).unwrap();
        sk.horizontal(l[2]).unwrap();
        sk.vertical(l[1]).unwrap();
        sk.vertical(l[3]).unwrap();
        sk.dim_distance(p[0], p[1]).unwrap();
        sk.dim_distance(p[1], p[2]).unwrap();
        let red = sk.perpendicular(l[0], l[1]).unwrap();
        out.push(finish(
            format!("special rectangle-perp#{k}"),
            sk,
            Truth::Over { flag: vec![red] },
            rng,
        ));
    }
    // 3. three angle dimensions between three lines (sum of directed angles is fixed)
    for k in 0..6 {
        let mut sk = Sketch::new();
        let dirs = [
            rng.range(0.0, 1.0),
            rng.range(1.5, 2.5),
            rng.range(3.0, 4.5),
        ];
        let l: Vec<_> = dirs
            .iter()
            .enumerate()
            .map(|(i, a)| {
                sk.add_line_xy(
                    i as f64 * 7.0,
                    0.0,
                    i as f64 * 7.0 + 5.0 * a.cos(),
                    5.0 * a.sin(),
                )
            })
            .collect();
        for e in &l {
            let (p1, _) = sk.line_points(*e).unwrap();
            sk.fix(p1).unwrap();
        }
        sk.dim_angle(l[0], l[1]).unwrap();
        sk.dim_angle(l[1], l[2]).unwrap();
        let red = sk.dim_angle(l[0], l[2]).unwrap();
        // the three lines' far endpoints stay free: 3 free directions - 2 independent angles = 1 DOF + 3 lengths
        out.push(finish(
            format!("special angle-triangle#{k}"),
            sk,
            Truth::Over { flag: vec![red] },
            rng,
        ));
    }
    // 4. collinear triple: point on line twice (second is the same constraint)
    for k in 0..6 {
        let mut sk = Sketch::new();
        let l = sk.add_line_xy(0.0, 0.0, 10.0, rng.range(-3.0, 3.0));
        sk.fix(l).unwrap();
        let (x, y) = (rng.range(2.0, 8.0), 0.0);
        let (lp1, _) = sk.line_points(l).unwrap();
        let _ = lp1;
        let p = sk.add_point(x, y + 0.01);
        // place the point exactly on the line
        let (a, b) = {
            let (u, v) = sk.line_points(l).unwrap();
            (sk.point(u).unwrap(), sk.point(v).unwrap())
        };
        let t = x / 10.0;
        sk.set_point(p, a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
            .unwrap();
        sk.point_on(p, l).unwrap();
        let red = sk.point_on(p, l).unwrap();
        out.push(finish(
            format!("special duplicate-point-on-line#{k}"),
            sk,
            Truth::Over { flag: vec![red] },
            rng,
        ));
    }
    // 5. concentric circles + coincident centres (duplicate) with equal radius dims
    for k in 0..6 {
        let mut sk = Sketch::new();
        let c1 = sk.add_circle_xy(2.0, 3.0, rng.range(2.0, 4.0));
        let ctr = match sk.kind(c1).unwrap() {
            EntityKind::Circle { center, .. } => *center,
            _ => unreachable!(),
        };
        let c2 = sk.add_circle(ctr, rng.range(5.0, 7.0));
        sk.fix(ctr).unwrap();
        sk.concentric(c1, c2).unwrap();
        sk.dim_radius(c1).unwrap();
        sk.dim_radius(c2).unwrap();
        let red = sk.concentric(c1, c2).unwrap();
        out.push(finish(
            format!("special concentric-dup#{k}"),
            sk,
            Truth::Over { flag: vec![red] },
            rng,
        ));
    }
}

/// The full ground-truth suite (≥ 300 cases).
pub fn classification_suite() -> Vec<ClassCase> {
    let mut rng = Rng::new(20240607);
    let mut out = Vec::new();
    chain_cases(&mut out, &mut rng, 130);
    template_cases(&mut out, &mut rng, 70);
    redundant_cases(&mut out, &mut rng, 50);
    conflict_cases(&mut out, &mut rng, 64);
    special_cases(&mut out, &mut rng);
    out
}

// ---------------------------------------------------------------------------
// branch-flip suite: tangent / angle / offset heavy sketches with free DOF
// ---------------------------------------------------------------------------

pub struct FlipCase {
    pub name: String,
    pub sketch: Sketch,
    /// Characteristic size of the sketch.
    pub scale: f64,
}

fn pt(sk: &mut Sketch, x: f64, y: f64) -> EntityId {
    sk.add_point(x, y)
}

/// Rectangle with rounded corners: 4 lines, 4 arcs, 8 G1 joints.
fn rounded_rect(sk: &mut Sketch, rng: &mut Rng, ox: f64, oy: f64) -> f64 {
    let w = rng.range(14.0, 26.0);
    let h = rng.range(9.0, 16.0);
    let r = rng.range(1.5, (w.min(h) / 2.0 - 1.0).min(4.5));
    let p = [
        pt(sk, ox + r, oy),
        pt(sk, ox + w - r, oy),
        pt(sk, ox + w, oy + r),
        pt(sk, ox + w, oy + h - r),
        pt(sk, ox + w - r, oy + h),
        pt(sk, ox + r, oy + h),
        pt(sk, ox, oy + h - r),
        pt(sk, ox, oy + r),
    ];
    let c = [
        pt(sk, ox + w - r, oy + r),
        pt(sk, ox + w - r, oy + h - r),
        pt(sk, ox + r, oy + h - r),
        pt(sk, ox + r, oy + r),
    ];
    let bottom = sk.add_line(p[0], p[1]);
    let right = sk.add_line(p[2], p[3]);
    let top = sk.add_line(p[4], p[5]);
    let left = sk.add_line(p[6], p[7]);
    let a_br = sk.add_arc(c[0], p[1], p[2]);
    let a_tr = sk.add_arc(c[1], p[3], p[4]);
    let a_tl = sk.add_arc(c[2], p[5], p[6]);
    let a_bl = sk.add_arc(c[3], p[7], p[0]);
    sk.fix(p[0]).unwrap();
    sk.horizontal(bottom).unwrap();
    sk.horizontal(top).unwrap();
    sk.vertical(right).unwrap();
    sk.vertical(left).unwrap();
    use CurveEnd::*;
    let g1 = Continuity::G1;
    sk.joint(bottom, End, a_br, Start, g1).unwrap();
    sk.joint(a_br, End, right, Start, g1).unwrap();
    sk.joint(right, End, a_tr, Start, g1).unwrap();
    sk.joint(a_tr, End, top, Start, g1).unwrap();
    sk.joint(top, End, a_tl, Start, g1).unwrap();
    sk.joint(a_tl, End, left, Start, g1).unwrap();
    sk.joint(left, End, a_bl, Start, g1).unwrap();
    sk.joint(a_bl, End, bottom, Start, g1).unwrap();
    sk.equal_radius(a_br, a_tr).unwrap();
    sk.equal_radius(a_tr, a_tl).unwrap();
    sk.equal_radius(a_tl, a_bl).unwrap();
    sk.dim_radius(a_br).unwrap();
    w.max(h)
}

/// Chain of circles tangent to a fixed base line and to each other.
fn circle_chain(sk: &mut Sketch, rng: &mut Rng) -> f64 {
    let base = sk.add_line_xy(-5.0, 0.0, 60.0, 0.0);
    sk.fix(base).unwrap();
    let n = 2 + rng.int(3);
    let mut x = 0.0;
    let mut prev: Option<(EntityId, f64)> = None;
    for _ in 0..n {
        let r = rng.range(1.5, 4.0);
        let cx = match prev {
            None => x + r,
            Some((_, pr)) => x + 2.0 * (pr * r).sqrt() + rng.range(0.0, 0.0),
        };
        let c = sk.add_circle_xy(cx, r, r);
        sk.tangent(base, c).unwrap();
        sk.dim_radius(c).unwrap();
        if let Some((pc, _)) = prev {
            // exact tangency requires dist = r1 + r2; place accordingly
            let (pcen, pr) = sk.circle_geometry(pc).unwrap();
            let dx = 2.0 * (pr * r).sqrt();
            sk.set_value(
                match sk.kind(c).unwrap() {
                    EntityKind::Circle { center, .. } => sk.point_vars(*center).unwrap().x,
                    _ => unreachable!(),
                },
                pcen.0 + dx,
            );
            sk.tangent(pc, c).unwrap();
        }
        x = sk.circle_geometry(c).unwrap().0 .0 + r;
        prev = Some((c, r));
    }
    // leave the first circle free along the line, fix nothing else
    60.0
}

/// Base polyline with offset copies at fixed distances and angle dimensions.
fn offset_polyline(sk: &mut Sketch, rng: &mut Rng) -> f64 {
    let n = 3 + rng.int(2);
    let mut p = vec![pt(sk, 0.0, 0.0)];
    let mut pos = (0.0, 0.0);
    let mut dir = (1.0, 0.0);
    let mut lines = Vec::new();
    let mut angles: Vec<(EntityId, EntityId)> = Vec::new();
    for i in 0..n {
        if i > 0 {
            dir = rotate(dir, rng.sign() * rng.range(0.4, 1.2));
        }
        let len = rng.range(6.0, 11.0);
        pos = (pos.0 + dir.0 * len, pos.1 + dir.1 * len);
        let q = pt(sk, pos.0, pos.1);
        let l = sk.add_line(*p.last().unwrap(), q);
        if let Some(&prev) = lines.last() {
            angles.push((prev, l));
        }
        lines.push(l);
        p.push(q);
    }
    sk.fix(p[0]).unwrap();
    sk.horizontal(lines[0]).unwrap();
    for (a, b) in angles {
        sk.dim_angle(a, b).unwrap();
    }
    // offset copies
    for &l in &lines {
        let (a, b) = sk.line_points(l).unwrap();
        let (pa, pb) = (sk.point(a).unwrap(), sk.point(b).unwrap());
        let d = (pb.0 - pa.0, pb.1 - pa.1);
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        let nrm = (-d.1 / len, d.0 / len);
        let off = rng.range(1.5, 3.5) * rng.sign();
        let m = sk.add_line_xy(
            pa.0 + nrm.0 * off,
            pa.1 + nrm.1 * off,
            pb.0 + nrm.0 * off,
            pb.1 + nrm.1 * off,
        );
        sk.dim_distance(m, l).unwrap();
        sk.equal_length(m, l).unwrap();
    }
    n as f64 * 9.0
}

/// Line → spline → arc with G1/G2 joints.
fn spline_chain(sk: &mut Sketch, rng: &mut Rng) -> f64 {
    let line = sk.add_line_xy(-8.0, 0.0, 0.0, 0.0);
    let (l0, l1) = sk.line_points(line).unwrap();
    sk.fix(l0).unwrap();
    sk.horizontal(line).unwrap();
    // spline starts at the line's end, leaves tangentially
    let mut ctrl = vec![l1];
    let steps = [
        (2.5, 0.0),
        (2.5, rng.range(1.0, 2.5)),
        (2.0, rng.range(2.5, 4.0)),
        (1.5, rng.range(2.5, 4.5)),
    ];
    let mut pos = sk.point(l1).unwrap();
    for s in steps {
        pos = (pos.0 + s.0, pos.1 + s.1);
        ctrl.push(sk.add_point(pos.0, pos.1));
    }
    let spl = sk.add_spline(3, ctrl.clone()).unwrap();
    // arc continuing from the spline's end
    let last = *ctrl.last().unwrap();
    let lp = sk.point(last).unwrap();
    let prev = sk.point(ctrl[ctrl.len() - 2]).unwrap();
    let t = (lp.0 - prev.0, lp.1 - prev.1);
    let tl = (t.0 * t.0 + t.1 * t.1).sqrt();
    let (tx, ty) = (t.0 / tl, t.1 / tl);
    let rad = rng.range(3.0, 5.0);
    // centre to the left of the travel direction
    let c = (lp.0 - ty * rad, lp.1 + tx * rad);
    let a0 = (lp.1 - c.1).atan2(lp.0 - c.0);
    let arc = sk.add_arc_xy(c.0, c.1, rad, a0, a0 + 1.0);
    let EntityKind::Arc { start, .. } = sk.kind(arc).unwrap().clone() else {
        unreachable!()
    };
    sk.set_point(start, lp.0, lp.1).unwrap();
    sk.coincident(start, last).unwrap();
    sk.joint(line, CurveEnd::End, spl, CurveEnd::Start, Continuity::G1)
        .unwrap();
    sk.joint(spl, CurveEnd::End, arc, CurveEnd::Start, Continuity::G1)
        .unwrap();
    sk.dim_radius(arc).unwrap();
    30.0
}

/// Lines fanning out of a fixed pivot with angle dimensions between neighbours.
fn pivot_fan(sk: &mut Sketch, rng: &mut Rng) -> f64 {
    let o = pt(sk, 0.0, 0.0);
    sk.fix(o).unwrap();
    let n = 3 + rng.int(3);
    let mut a = rng.range(0.0, 1.0);
    let mut lines: Vec<EntityId> = Vec::new();
    let mut first_len = 0.0;
    for i in 0..n {
        let mut len = rng.range(6.0, 12.0);
        if i == 0 {
            first_len = len;
        } else if i == 1 {
            len = first_len; // so that the EqualLength added below holds initially
        }
        let q = pt(sk, len * a.cos(), len * a.sin());
        let l = sk.add_line(o, q);
        if let Some(&prev) = lines.last() {
            sk.dim_angle(prev, l).unwrap();
        }
        lines.push(l);
        a += rng.range(0.35, 0.8);
    }
    if n > 1 {
        sk.equal_length(lines[0], lines[1]).unwrap();
    }
    first_len.max(12.0)
}

/// 100 reference sketches (tangent / angle / offset heavy) with free degrees of freedom.
pub fn flip_suite() -> Vec<FlipCase> {
    let mut out = Vec::new();
    for i in 0..100 {
        let mut rng = Rng::new(7000 + i as u64);
        let mut sk = Sketch::new();
        let (kind, scale) = match i % 10 {
            0..=2 => ("rounded-rect", rounded_rect(&mut sk, &mut rng, 0.0, 0.0)),
            3 | 4 => ("circle-chain", circle_chain(&mut sk, &mut rng)),
            5 | 6 => ("offset-polyline", offset_polyline(&mut sk, &mut rng)),
            7 | 8 => ("spline-chain", spline_chain(&mut sk, &mut rng)),
            _ => ("pivot-fan", pivot_fan(&mut sk, &mut rng)),
        };
        out.push(FlipCase {
            name: format!("{kind}#{i}"),
            sketch: sk,
            scale,
        });
    }
    out
}

// ---------------------------------------------------------------------------
// benchmark sketches
// ---------------------------------------------------------------------------

/// One bracket-like feature (~17 entities): dimensioned rectangle, a hole
/// located by two offsets, a second circle tangent to the base line and the
/// hole, an angled arm with an offset parallel (which keeps one free DOF).
/// Returns `(corner p0, corner p1, number of entities added)`.
fn bracket(sk: &mut Sketch, rng: &mut Rng, ox: f64, oy: f64, anchor: bool) -> (EntityId, EntityId) {
    let w = rng.range(18.0, 26.0);
    let h = rng.range(10.0, 16.0);
    let p0 = pt(sk, ox, oy);
    let p1 = pt(sk, ox + w, oy);
    let p2 = pt(sk, ox + w, oy + h);
    let p3 = pt(sk, ox, oy + h);
    let l0 = sk.add_line(p0, p1);
    let l1 = sk.add_line(p1, p2);
    let l2 = sk.add_line(p2, p3);
    let l3 = sk.add_line(p3, p0);
    if anchor {
        sk.fix(p0).unwrap();
    }
    sk.horizontal(l0).unwrap();
    sk.horizontal(l2).unwrap();
    sk.vertical(l1).unwrap();
    sk.vertical(l3).unwrap();
    sk.dim_distance(p0, p1).unwrap(); // width
                                      // height stays free (one DOF)
                                      // hole located by distances to the left and bottom edges; the second circle
                                      // (tangent to the base line and the hole) must be geometrically possible
    let hr = rng.range(2.0, 3.0);
    let r2 = rng.range(1.5, 2.5);
    let dbot = hr + rng.range(0.8, 1.6) * r2;
    let hole = sk.add_circle_xy(ox + w * 0.3, oy + dbot, hr);
    let hc = match sk.kind(hole).unwrap() {
        EntityKind::Circle { center, .. } => *center,
        _ => unreachable!(),
    };
    sk.dim_radius(hole).unwrap();
    sk.dim_distance(hc, l3).unwrap();
    sk.dim_distance(hc, l0).unwrap();
    // second circle tangent to the base line and externally to the hole
    let ddy = r2 - dbot;
    let need = ((hr + r2).powi(2) - ddy * ddy).max(0.01).sqrt();
    let c2 = sk.add_circle_xy(ox + w * 0.3 + need, oy + r2, r2);
    sk.tangent(l0, c2).unwrap();
    sk.tangent(hole, c2).unwrap();
    sk.dim_radius(c2).unwrap();
    // angled arm off the top-right corner with an offset parallel
    let ang = rng.range(0.5, 1.0);
    let arm_len = rng.range(6.0, 9.0);
    let q = pt(
        sk,
        ox + w + arm_len * ang.cos(),
        oy + h + arm_len * ang.sin(),
    );
    let arm = sk.add_line(p2, q);
    sk.dim_angle(l1, arm).unwrap();
    sk.dim_distance(p2, q).unwrap();
    let off = 2.5;
    let nx = (-ang.sin() * off, ang.cos() * off);
    let a = pt(sk, ox + w + nx.0, oy + h + nx.1);
    let b = pt(
        sk,
        ox + w + nx.0 + arm_len * ang.cos(),
        oy + h + nx.1 + arm_len * ang.sin(),
    );
    let par = sk.add_line(a, b);
    sk.dim_distance(par, arm).unwrap(); // includes the parallel constraint
    sk.equal_length(par, arm).unwrap();
    (p0, p1)
}

/// `n_features` brackets laid out on a grid. With `coupled` every feature's
/// anchor is tied to its left neighbour (horizontal gap), so each row is one
/// long dependency chain and the whole row is a single connected component;
/// otherwise each feature is fixed in place (many small components).
pub fn feature_grid(n_features: usize, coupled: bool, seed: u64) -> Sketch {
    let mut rng = Rng::new(seed);
    let mut sk = Sketch::new();
    let cols = 16usize;
    let mut prev_p1: Option<EntityId> = None;
    for k in 0..n_features {
        let (row, col) = (k / cols, k % cols);
        let anchor = !coupled || col == 0;
        let (ox, oy) = if coupled && col > 0 {
            // start 20 to the right of the previous feature's p1, on the same level
            let (px, py) = sk.point(prev_p1.unwrap()).unwrap();
            (px + 20.0, py)
        } else {
            (col as f64 * 70.0, row as f64 * 60.0)
        };
        let (p0, p1) = bracket(&mut sk, &mut rng, ox, oy, anchor);
        if coupled && col > 0 {
            let pp = prev_p1.unwrap();
            sk.add_constraint(ConstraintKind::HorizontalPts { a: pp, b: p0 })
                .unwrap();
            sk.dim_distance(pp, p0).unwrap();
        }
        prev_p1 = Some(p1);
    }
    sk
}

/// A single polyline of `n` segments: every point determined by the previous one.
pub fn long_chain(n: usize, seed: u64) -> Sketch {
    let mut rng = Rng::new(seed);
    build_chain(&mut rng, n, true).sk
}

/// Rigid truss (ladder with diagonals) of `2·m` points: one irreducible block of ~4m unknowns.
pub fn truss(m: usize, seed: u64) -> Sketch {
    let mut rng = Rng::new(seed);
    let mut sk = Sketch::new();
    let top: Vec<EntityId> = (0..m)
        .map(|i| {
            pt(
                &mut sk,
                i as f64 * 4.0 + rng.range(-0.2, 0.2),
                4.0 + rng.range(-0.2, 0.2),
            )
        })
        .collect();
    let bot: Vec<EntityId> = (0..m)
        .map(|i| {
            pt(
                &mut sk,
                i as f64 * 4.0 + rng.range(-0.2, 0.2),
                rng.range(-0.2, 0.2),
            )
        })
        .collect();
    let ex = |sk: &mut Sketch, a: EntityId, b: EntityId| {
        sk.dim_distance(a, b).unwrap();
    };
    for i in 0..m {
        ex(&mut sk, top[i], bot[i]);
        if i + 1 < m {
            ex(&mut sk, top[i], top[i + 1]);
            ex(&mut sk, bot[i], bot[i + 1]);
            ex(&mut sk, top[i], bot[i + 1]);
        }
    }
    sk.fix(bot[0]).unwrap();
    let l = sk.add_line(bot[0], bot[1]);
    sk.horizontal(l).unwrap();
    sk
}

pub fn entity_count(sk: &Sketch) -> usize {
    sk.entities.len()
}
