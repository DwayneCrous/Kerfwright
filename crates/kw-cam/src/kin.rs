//! Five-axis machine kinematics: forward and inverse kinematics of machines described as chains of linear
//! and rotary joints on the tool side and on the workpiece side, singularity handling and a per-block
//! joint step cap with re-solved tool-tip position (RTCP).
//!
//! A machine moves the tool tip relative to the workpiece. With `T_tool(q)` the product of the tool-side
//! joint transforms (outermost first) and `T_work(q)` that of the workpiece side, a workpiece point `w`
//! sits at `T_work(q) w` in the machine base frame, and the tool tip at `T_tool(q) · tip0`. The tip in
//! workpiece coordinates is therefore `T_work⁻¹ T_tool tip0` and the tool axis `R_work⁻¹ R_tool k0`.
//!
//! Inverse kinematics is analytic. The two rotary angles follow from the tool axis alone: whatever the
//! layout (both rotaries on the table, one on the head and one on the table, both on the head) the
//! orientation condition has the form `R_a(θ_a) x = R_b(θ_b) y` for two fixed axes and two fixed vectors,
//! which gives `θ_b` from one equation `A cos θ + B sin θ = C` (two solutions, the two *branches*) and
//! `θ_a` by an `atan2`. The linear axes then solve a 3×3 linear system, because the tip position is affine in
//! them once the rotary angles are known.

use serde::{Deserialize, Serialize};

pub type V3 = [f64; 3];
pub type M3 = [[f64; 3]; 3];

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: V3) -> V3 {
    let n = norm(a);
    scale(a, 1.0 / n)
}

/// Rodrigues rotation of `v` about the unit axis `u` by `th`.
pub fn rotate(u: V3, th: f64, v: V3) -> V3 {
    let (s, c) = th.sin_cos();
    add(
        add(scale(v, c), scale(cross(u, v), s)),
        scale(u, dot(u, v) * (1.0 - c)),
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
    Linear,
    Rotary,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub kind: JointKind,
    /// Direction of the translation, or axis of the rotation (unit), in the machine base frame at zero.
    pub axis: V3,
    /// A point on the rotation axis (rotary joints).
    pub pivot: V3,
    /// Which entry of the axis vector this joint takes.
    pub index: usize,
}

impl Joint {
    pub fn linear(index: usize, axis: V3) -> Joint {
        Joint {
            kind: JointKind::Linear,
            axis: unit(axis),
            pivot: [0.0; 3],
            index,
        }
    }
    pub fn rotary(index: usize, axis: V3, pivot: V3) -> Joint {
        Joint {
            kind: JointKind::Rotary,
            axis: unit(axis),
            pivot,
            index,
        }
    }
    fn apply(&self, q: &[f64], x: V3) -> V3 {
        match self.kind {
            JointKind::Linear => add(x, scale(self.axis, q[self.index])),
            JointKind::Rotary => add(
                self.pivot,
                rotate(self.axis, q[self.index], sub(x, self.pivot)),
            ),
        }
    }
    fn apply_dir(&self, q: &[f64], v: V3) -> V3 {
        match self.kind {
            JointKind::Linear => v,
            JointKind::Rotary => rotate(self.axis, q[self.index], v),
        }
    }
    fn apply_inv(&self, q: &[f64], x: V3) -> V3 {
        match self.kind {
            JointKind::Linear => sub(x, scale(self.axis, q[self.index])),
            JointKind::Rotary => add(
                self.pivot,
                rotate(self.axis, -q[self.index], sub(x, self.pivot)),
            ),
        }
    }
    fn apply_dir_inv(&self, q: &[f64], v: V3) -> V3 {
        match self.kind {
            JointKind::Linear => v,
            JointKind::Rotary => rotate(self.axis, -q[self.index], v),
        }
    }
}

/// Pose of the tool in workpiece coordinates: the tip and the unit axis pointing from the tip up the tool.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub tip: V3,
    pub axis: V3,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Machine {
    pub name: String,
    /// Joints carrying the tool, outermost (nearest the machine base) first.
    pub tool_chain: Vec<Joint>,
    /// Joints carrying the workpiece, outermost first.
    pub work_chain: Vec<Joint>,
    /// Tool tip in the machine base frame at zero.
    pub tip0: V3,
    /// Tool axis (tip towards spindle) at zero.
    pub axis0: V3,
    /// Names of the axes in the order of the axis vector (`X`, `Y`, `Z`, then the rotaries).
    pub axis_names: Vec<String>,
    /// Travel limits per axis (rotaries in radians).
    pub limits: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IkSolution {
    pub q: Vec<f64>,
    /// The orientation is singular (one rotary angle is free): the free angle was set from the hint.
    pub singular: bool,
}

impl Machine {
    pub fn axis_count(&self) -> usize {
        self.axis_names.len()
    }

    fn rotaries(&self) -> Vec<(bool, Joint)> {
        // (on the tool side, joint)
        self.tool_chain
            .iter()
            .filter(|j| j.kind == JointKind::Rotary)
            .map(|j| (true, *j))
            .chain(
                self.work_chain
                    .iter()
                    .filter(|j| j.kind == JointKind::Rotary)
                    .map(|j| (false, *j)),
            )
            .collect()
    }

    /// Indices of the rotary axes in the axis vector.
    pub fn rotary_indices(&self) -> Vec<usize> {
        self.rotaries().iter().map(|r| r.1.index).collect()
    }

    fn tool_point(&self, q: &[f64]) -> V3 {
        // the innermost joint acts first on the point
        self.tool_chain
            .iter()
            .rev()
            .fold(self.tip0, |x, j| j.apply(q, x))
    }

    fn tool_dir(&self, q: &[f64]) -> V3 {
        self.tool_chain
            .iter()
            .rev()
            .fold(self.axis0, |v, j| j.apply_dir(q, v))
    }

    /// Workpiece point `w` expressed in the machine base frame.
    fn work_point(&self, q: &[f64], w: V3) -> V3 {
        self.work_chain.iter().rev().fold(w, |x, j| j.apply(q, x))
    }

    /// Machine base frame point expressed in workpiece coordinates.
    fn to_work(&self, q: &[f64], x: V3) -> V3 {
        self.work_chain.iter().fold(x, |x, j| j.apply_inv(q, x))
    }

    fn dir_to_work(&self, q: &[f64], v: V3) -> V3 {
        self.work_chain.iter().fold(v, |v, j| j.apply_dir_inv(q, v))
    }

    pub fn forward(&self, q: &[f64]) -> Pose {
        Pose {
            tip: self.to_work(q, self.tool_point(q)),
            axis: unit(self.dir_to_work(q, self.tool_dir(q))),
        }
    }

    /// Solve the orientation for the rotary angles. `hint` supplies the free angle at a singularity
    /// (by axis index) and breaks ties. Returns the angles per rotary joint in the order of
    /// [`rotary_indices`], one entry per solution, and whether the solution is singular.
    pub fn orientation(&self, k: V3, hint: &[f64]) -> Vec<(Vec<f64>, bool)> {
        let k = unit(k);
        let rot = self.rotaries();
        match rot.len() {
            0 => vec![(Vec::new(), false)],
            1 => {
                let (tool_side, j) = rot[0];
                // R_t k0 = R_w k  with one joint
                // single rotary: the angle that turns the tool axis onto k
                let (from, to) = if tool_side {
                    (self.axis0, k)
                } else {
                    (k, self.axis0)
                };
                vec![(vec![angle_about(j.axis, from, to)], false)]
            }
            2 => self.orientation2(&rot, k, hint),
            _ => Vec::new(),
        }
    }

    fn orientation2(&self, rot: &[(bool, Joint)], k: V3, hint: &[f64]) -> Vec<(Vec<f64>, bool)> {
        let (s0, j0) = rot[0];
        let (s1, j1) = rot[1];
        // the equation R_a(θ_a) x = R_b(θ_b) y and how its unknowns map to the joints
        // case on the same side: tool side:  R1 R2 z0 = k_w-ish ... see below
        // both on the tool side:   R_t1(θ1) R_t2(θ2) a0 = k             → R_t2 a0 = R_t1⁻¹ k
        // both on the work side:   R_w1(θ1) R_w2(θ2) k = a0             → R_w2 k  = R_w1⁻¹ a0
        // tool side + work side:   R_t(θt) a0 = R_w(θw) k
        let a0 = self.axis0;
        // (axis_a, x, axis_b, y, sign_b, joint order: whether a is rot[0]) with R_a x = R_b^{sign}(θ) y
        struct Eq {
            ua: V3,
            x: V3,
            ub: V3,
            y: V3,
            sign_b: f64,
            a_is_first: bool,
        }
        let eq = if s0 && s1 {
            // R_t2(θ2) a0 = R_t1(−θ1) k
            Eq {
                ua: j1.axis,
                x: a0,
                ub: j0.axis,
                y: k,
                sign_b: -1.0,
                a_is_first: false,
            }
        } else if !s0 && !s1 {
            // R_w2(θ2) k = R_w1(−θ1) a0
            Eq {
                ua: j1.axis,
                x: k,
                ub: j0.axis,
                y: a0,
                sign_b: -1.0,
                a_is_first: false,
            }
        } else if s0 && !s1 {
            // tool rotary first, work rotary second: R_t(θt) a0 = R_w(θw) k
            Eq {
                ua: j0.axis,
                x: a0,
                ub: j1.axis,
                y: k,
                sign_b: 1.0,
                a_is_first: true,
            }
        } else {
            // work rotary first, tool rotary second
            Eq {
                ua: j1.axis,
                x: a0,
                ub: j0.axis,
                y: k,
                sign_b: 1.0,
                a_is_first: false,
            }
        };
        let (ua, ub) = (eq.ua, eq.ub);
        // R_b(σθ) y = y cosθ + σ(ub×y) sinθ + ub(ub·y)(1−cosθ);   ua·x = ua·R_b y
        let (a_coef, b_coef, c_coef) = {
            let uu = dot(ua, ub);
            let uby = dot(ub, eq.y);
            (
                dot(ua, eq.y) - uu * uby,
                eq.sign_b * dot(ua, cross(ub, eq.y)),
                dot(ua, eq.x) - uu * uby,
            )
        };
        let r = a_coef.hypot(b_coef);
        let mut sols: Vec<(Vec<f64>, bool)> = Vec::new();
        let hint_b = hint_for(self, if eq.a_is_first { j1.index } else { j0.index }, hint);
        let hint_a = hint_for(self, if eq.a_is_first { j0.index } else { j1.index }, hint);
        let ths: Vec<(f64, bool)> = if r < 1e-12 {
            // the axes are aligned with the vector: θ_b is free (singular), keep the hint
            if c_coef.abs() < 1e-9 {
                vec![(hint_b, true)]
            } else {
                Vec::new()
            }
        } else if c_coef.abs() > r + 1e-12 {
            Vec::new()
        } else {
            let phi = b_coef.atan2(a_coef);
            let d = (c_coef / r).clamp(-1.0, 1.0).acos();
            let mut v = vec![(phi + d, false)];
            if d > 1e-12 && (std::f64::consts::PI - d) > 1e-12 {
                v.push((phi - d, false));
            }
            v
        };
        for (th_b, singular_b) in ths {
            let w = rotate(ub, eq.sign_b * th_b, eq.y);
            // θ_a rotates x onto w about ua; if x is parallel to ua the angle is free
            let (th_a, singular_a) = {
                let xp = sub(eq.x, scale(ua, dot(ua, eq.x)));
                let wp = sub(w, scale(ua, dot(ua, w)));
                if norm(xp) < 1e-9 || norm(wp) < 1e-9 {
                    (hint_a, true)
                } else {
                    (angle_about(ua, xp, wp), false)
                }
            };
            let (t0, t1) = if eq.a_is_first {
                (th_a, th_b)
            } else {
                (th_b, th_a)
            };
            // R_b(σθ) y with σ = −1 on the same-side chains makes θ_b the joint angle itself in every case
            let ang = vec![t0, t1];
            sols.push((ang, singular_a || singular_b));
        }
        sols
    }

    /// All inverse-kinematics solutions for a pose; `hint` is the previous axis vector (used for the free
    /// angle at a singularity).
    pub fn inverse(&self, tip: V3, axis: V3, hint: &[f64]) -> Vec<IkSolution> {
        let rot_idx = self.rotary_indices();
        let mut out = Vec::new();
        for (angles, singular) in self.orientation(axis, hint) {
            let mut q = vec![0.0; self.axis_count()];
            for (i, a) in rot_idx.iter().zip(&angles) {
                q[*i] = *a;
            }
            // linear axes: the tip is affine in them
            let lin: Vec<usize> = self
                .tool_chain
                .iter()
                .chain(&self.work_chain)
                .filter(|j| j.kind == JointKind::Linear)
                .map(|j| j.index)
                .collect();
            let resid = |qv: &[f64]| sub(self.work_point(qv, tip), self.tool_point(qv));
            let r0 = resid(&q);
            let mut jm = [[0.0; 3]; 3];
            for (c, &li) in lin.iter().enumerate().take(3) {
                let mut qq = q.clone();
                qq[li] = 1.0;
                let r1 = resid(&qq);
                for row in 0..3 {
                    jm[row][c] = r1[row] - r0[row];
                }
            }
            let Some(sol) = solve3(jm, [-r0[0], -r0[1], -r0[2]]) else {
                continue;
            };
            for (c, &li) in lin.iter().enumerate().take(3) {
                q[li] = sol[c];
            }
            out.push(IkSolution { q, singular });
        }
        out
    }
}

fn hint_for(m: &Machine, index: usize, hint: &[f64]) -> f64 {
    let _ = m;
    hint.get(index).copied().unwrap_or(0.0)
}

/// Angle of the rotation about unit `u` that takes `from` onto `to` (both projected perpendicular to `u`).
fn angle_about(u: V3, from: V3, to: V3) -> f64 {
    let f = sub(from, scale(u, dot(u, from)));
    let t = sub(to, scale(u, dot(u, to)));
    dot(u, cross(f, t)).atan2(dot(f, t))
}

fn solve3(a: M3, b: V3) -> Option<V3> {
    let det = dot(a[0], cross(a[1], a[2]));
    if det.abs() < 1e-12 {
        return None;
    }
    // Cramer's rule on the rows
    let col = |c: usize| [a[0][c], a[1][c], a[2][c]];
    let (c0, c1, c2) = (col(0), col(1), col(2));
    let d = dot(c0, cross(c1, c2));
    Some([
        dot(b, cross(c1, c2)) / d,
        dot(c0, cross(b, c2)) / d,
        dot(c0, cross(c1, b)) / d,
    ])
}

/// Angle between two unit vectors, accurate for tiny angles.
pub fn angle_between(a: V3, b: V3) -> f64 {
    norm(cross(a, b)).atan2(dot(a, b))
}

fn slerp(a: V3, b: V3, t: f64) -> V3 {
    let c = dot(a, b).clamp(-1.0, 1.0);
    let th = c.acos();
    if th < 1e-9 {
        return unit(add(scale(a, 1.0 - t), scale(b, t)));
    }
    if std::f64::consts::PI - th < 1e-9 {
        // opposite axes: turn about any perpendicular
        let p = if a[0].abs() < 0.9 {
            cross(a, [1.0, 0.0, 0.0])
        } else {
            cross(a, [0.0, 1.0, 0.0])
        };
        return rotate(unit(p), std::f64::consts::PI * t, a);
    }
    let s = th.sin();
    add(
        scale(a, ((1.0 - t) * th).sin() / s),
        scale(b, (t * th).sin() / s),
    )
}

fn wrap(a: f64) -> f64 {
    let mut x = a % std::f64::consts::TAU;
    if x > std::f64::consts::PI {
        x -= std::f64::consts::TAU;
    } else if x < -std::f64::consts::PI {
        x += std::f64::consts::TAU;
    }
    x
}

/// The five test machines of the acceptance suite.
pub fn test_machines() -> Vec<Machine> {
    let z = [0.0, 0.0, 1.0];
    let x = [1.0, 0.0, 0.0];
    let y = [0.0, 1.0, 0.0];
    let names = |r: [&str; 2]| vec!["X".into(), "Y".into(), "Z".into(), r[0].into(), r[1].into()];
    let lim = |a: f64, b: f64| vec![(-1e4, 1e4), (-1e4, 1e4), (-1e4, 1e4), (-a, a), (-b, b)];
    let pi = std::f64::consts::PI;
    // 1. table-table, A (about X, on the cradle) and C (about Z, on the table), with a pivot offset
    let m1 = Machine {
        name: "AC trunnion table".into(),
        tool_chain: vec![Joint::linear(2, z)],
        work_chain: vec![
            Joint::linear(0, x),
            Joint::linear(1, y),
            Joint::rotary(3, x, [0.0, 12.5, -30.0]),
            Joint::rotary(4, z, [3.0, 0.0, 0.0]),
        ],
        tip0: [0.0, 0.0, 100.0],
        axis0: z,
        axis_names: names(["A", "C"]),
        limits: lim(110f64.to_radians(), 1e4),
    };
    // 2. table-table, B (about Y) and C (about Z)
    let m2 = Machine {
        name: "BC table".into(),
        tool_chain: vec![Joint::linear(2, z)],
        work_chain: vec![
            Joint::linear(0, x),
            Joint::linear(1, y),
            Joint::rotary(3, y, [-5.0, 0.0, -20.0]),
            Joint::rotary(4, z, [0.0, 0.0, 0.0]),
        ],
        tip0: [0.0, 0.0, 80.0],
        axis0: z,
        axis_names: names(["B", "C"]),
        limits: lim(100f64.to_radians(), 1e4),
    };
    // 3. head-table: B tilts the head (about Y), C turns the table
    let m3 = Machine {
        name: "B head, C table".into(),
        tool_chain: vec![Joint::linear(2, z), Joint::rotary(3, y, [0.0, 0.0, 150.0])],
        work_chain: vec![
            Joint::linear(0, x),
            Joint::linear(1, y),
            Joint::rotary(4, z, [0.0, 0.0, 0.0]),
        ],
        tip0: [0.0, 0.0, 100.0],
        axis0: z,
        axis_names: names(["B", "C"]),
        limits: lim(110f64.to_radians(), 1e4),
    };
    // 4. head-head: C about Z, B about Y on the head
    let m4 = Machine {
        name: "C-B swivel head".into(),
        tool_chain: vec![
            Joint::linear(0, x),
            Joint::linear(1, y),
            Joint::linear(2, z),
            Joint::rotary(4, z, [0.0, 0.0, 200.0]),
            Joint::rotary(3, y, [0.0, 0.0, 150.0]),
        ],
        work_chain: vec![],
        tip0: [0.0, 0.0, 100.0],
        axis0: z,
        axis_names: names(["B", "C"]),
        limits: lim(100f64.to_radians(), 1e4),
    };
    // 5. nutating head: C about Z, A about an axis 45° from Z
    let tilt = unit([0.0, (pi / 4.0).sin(), (pi / 4.0).cos()]);
    let m5 = Machine {
        name: "nutating head (45°)".into(),
        tool_chain: vec![
            Joint::linear(0, x),
            Joint::linear(1, y),
            Joint::linear(2, z),
            Joint::rotary(4, z, [0.0, 0.0, 200.0]),
            Joint::rotary(3, tilt, [0.0, 0.0, 160.0]),
        ],
        work_chain: vec![],
        tip0: [0.0, 0.0, 90.0],
        axis0: [0.0, 0.0, 1.0],
        axis_names: names(["A", "C"]),
        limits: lim(2.0 * pi, 1e4),
    };
    vec![m1, m2, m3, m4, m5]
}

/// How a sequence of tool poses is converted to axis positions.
#[derive(Debug, Clone)]
pub struct IkOptions {
    /// Largest change of any rotary axis within one block (radians); longer moves are split.
    pub max_joint_step: f64,
    /// Half-angle of the singular cone about the singular direction within which the free angle is held.
    pub singular_cone: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions {
            max_joint_step: 5f64.to_radians(),
            singular_cone: 2f64.to_radians(),
        }
    }
}

/// One output block: axis values and the orientation error actually achieved.
#[derive(Debug, Clone)]
pub struct AxisBlock {
    pub q: Vec<f64>,
    /// The tool pose this block stands for (tip on the straight line, axis on the great circle).
    pub want_tip: V3,
    pub want_axis: V3,
    /// Angle between the requested tool axis and the one the axes produce (radians).
    pub orientation_error: f64,
    /// Distance between the requested tip and the one the axes produce (mm); 0 with RTCP re-solve.
    pub tip_error: f64,
}

/// Convert a sequence of poses to axis blocks: pick the branch nearest to the previous block, hold the free
/// angle inside the singular cone (and interpolate the rotaries across it), re-solve the linear axes for the
/// tool-tip position (RTCP), and split any move whose rotary change exceeds the step cap.
pub fn path_to_axes(m: &Machine, poses: &[Pose], start: &[f64], opt: &IkOptions) -> Vec<AxisBlock> {
    let rot = m.rotary_indices();
    let mut out: Vec<AxisBlock> = Vec::new();
    let mut prev = start.to_vec();
    for pose in poses {
        let sols = m.inverse(pose.tip, pose.axis, &prev);
        if sols.is_empty() {
            continue;
        }
        // nearest branch (unwrapped distance of the rotaries to the previous block)
        let dist = |q: &[f64]| -> f64 {
            rot.iter()
                .map(|&i| wrap(q[i] - prev[i]).abs())
                .fold(0.0, f64::max)
        };
        let mut best = sols
            .into_iter()
            .min_by(|a, b| dist(&a.q).total_cmp(&dist(&b.q)))
            .expect("a solution");
        // unwrap to the previous angles
        for &i in &rot {
            best.q[i] = prev[i] + wrap(best.q[i] - prev[i]);
        }
        let target = best.q.clone();
        // a move whose rotary change exceeds the cap is split into equal parts (axes interpolated, position
        // re-solved so that the tool tip follows the straight line)
        let jump = rot
            .iter()
            .map(|&i| (target[i] - prev[i]).abs())
            .fold(0.0, f64::max);
        let n = ((jump / opt.max_joint_step).ceil() as usize).max(1);
        let from = prev.clone();
        let p0 = m.forward(&from);
        for s in 1..=n {
            let t = s as f64 / n as f64;
            let mut q = target.clone();
            for &i in &rot {
                q[i] = from[i] + (target[i] - from[i]) * t;
            }
            // intermediate tip: on the straight line from the previous tip to the new one
            let (tip, axis) = if n == 1 {
                (pose.tip, pose.axis)
            } else {
                let tip = add(p0.tip, scale(sub(pose.tip, p0.tip), t));
                (tip, slerp(p0.axis, unit(pose.axis), t))
            };
            // re-solve the linear axes for the tip with the rotaries as set (RTCP)
            let lin: Vec<usize> = m
                .tool_chain
                .iter()
                .chain(&m.work_chain)
                .filter(|j| j.kind == JointKind::Linear)
                .map(|j| j.index)
                .collect();
            let resid = |qv: &[f64]| sub(m.work_point(qv, tip), m.tool_point(qv));
            let r0 = resid(&q);
            let mut jm = [[0.0; 3]; 3];
            for (c, &li) in lin.iter().enumerate().take(3) {
                let mut qq = q.clone();
                qq[li] += 1.0;
                let r1 = resid(&qq);
                for row in 0..3 {
                    jm[row][c] = r1[row] - r0[row];
                }
            }
            if let Some(sol) = solve3(jm, [-r0[0], -r0[1], -r0[2]]) {
                for (c, &li) in lin.iter().enumerate().take(3) {
                    q[li] += sol[c];
                }
            }
            let got = m.forward(&q);
            out.push(AxisBlock {
                q: q.clone(),
                want_tip: tip,
                want_axis: unit(axis),
                orientation_error: angle_between(got.axis, unit(axis)),
                tip_error: norm(sub(got.tip, tip)),
            });
            prev = q;
        }
    }
    out
}

/// Turn tool poses into a five-axis toolpath for a post. With `tcp` the controller does the kinematics: X Y Z
/// are the tool tip in workpiece coordinates and only the rotary angles come from the inverse kinematics.
/// Without it the post writes the machine's linear axes, resolved here (RTCP in the post).
pub fn to_toolpath(
    m: &Machine,
    poses: &[Pose],
    start: &[f64],
    opt: &IkOptions,
    tcp: bool,
    feed: f64,
    tool: &crate::Tool,
) -> (crate::path::Toolpath, Vec<AxisBlock>) {
    use crate::path::{Event, Toolpath};
    let blocks = path_to_axes(m, poses, start, opt);
    let rot = m.rotary_indices();
    let names: [char; 2] = [
        m.axis_names[rot[0]].chars().next().unwrap_or('A'),
        m.axis_names[rot[1]].chars().next().unwrap_or('C'),
    ];
    let mut tp = Toolpath::new(&m.name);
    tp.push(Event::ToolChange {
        number: tool.number,
        name: tool.name.clone(),
        diameter: tool.diameter,
        rpm: 8000.0,
    });
    tp.push(Event::SpindleOn {
        rpm: 8000.0,
        clockwise: true,
    });
    if tcp {
        tp.push(Event::Tcp { on: true });
    }
    for (i, b) in blocks.iter().enumerate() {
        let p = if tcp {
            b.want_tip
        } else {
            [b.q[0], b.q[1], b.q[2]]
        };
        let deg = [b.q[rot[0]].to_degrees(), b.q[rot[1]].to_degrees()];
        // with TCP the intermediate tip of a split block is the commanded one; the rotaries carry the
        // orientation
        tp.push(if i == 0 {
            Event::Rapid5 { p, rot: deg, names }
        } else {
            Event::Feed5 {
                p,
                rot: deg,
                names,
                feed,
            }
        });
    }
    if tcp {
        tp.push(Event::Tcp { on: false });
    }
    tp.push(Event::SpindleOff);
    tp.push(Event::ProgramEnd);
    (tp, blocks)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rnd(seed: &mut u64) -> f64 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        (*seed >> 11) as f64 / (1u64 << 53) as f64
    }

    #[test]
    fn forward_of_the_zero_configuration_is_the_identity_pose() {
        for m in test_machines() {
            let p = m.forward(&[0.0; 5]);
            // the tool axis points along z and the tip is where tip0 says (up to the pivot offsets, which do
            // not move anything at zero)
            assert!(norm(sub(p.axis, m.axis0)) < 1e-12, "{}", m.name);
            assert!(norm(sub(p.tip, m.tip0)) < 1e-12, "{}", m.name);
        }
    }

    #[test]
    fn inverse_round_trips_on_every_machine_and_both_branches() {
        let mut seed = 0x1234_5678_9ABC_DEF1u64;
        for m in test_machines() {
            let mut worst = 0.0f64;
            let mut both = 0;
            for _ in 0..2000 {
                let mut q = vec![0.0; 5];
                for i in 0..3 {
                    q[i] = rnd(&mut seed) * 200.0 - 100.0;
                }
                for &i in &m.rotary_indices() {
                    let lim = m.limits[i].1.min(std::f64::consts::PI);
                    q[i] = (rnd(&mut seed) * 2.0 - 1.0) * lim * 0.95;
                }
                let pose = m.forward(&q);
                let sols = m.inverse(pose.tip, pose.axis, &q);
                assert!(!sols.is_empty(), "{}: no solution for {q:?}", m.name);
                if sols.len() == 2 {
                    both += 1;
                }
                for s in &sols {
                    let g = m.forward(&s.q);
                    let e = norm(sub(g.tip, pose.tip)).max(norm(sub(g.axis, pose.axis)));
                    worst = worst.max(e);
                }
                // the original configuration is among the solutions
                let found = sols.iter().any(|s| {
                    (0..5).all(|i| {
                        let d = if m.rotary_indices().contains(&i) {
                            wrap(s.q[i] - q[i]).abs()
                        } else {
                            (s.q[i] - q[i]).abs()
                        };
                        d < 1e-7
                    })
                });
                assert!(
                    found,
                    "{}: the original axes {q:?} are not among {:?}",
                    m.name,
                    sols.iter().map(|s| s.q.clone()).collect::<Vec<_>>()
                );
            }
            eprintln!(
                "{}: worst FK(IK) error {worst:.2e}, two branches in {both}/2000 poses",
                m.name
            );
            assert!(worst < 1e-9, "{}: {worst}", m.name);
        }
    }

    #[test]
    fn a_path_through_the_singular_cone_never_jumps_more_than_the_cap() {
        let opt = IkOptions::default();
        for m in test_machines() {
            let rot = m.rotary_indices();
            // tool axis sweeping past the pole of the first rotary at a small offset: the second rotary
            // swings through nearly half a turn over a tiny change of orientation
            let n = 400;
            let off = 0.05f64.to_radians();
            let poses: Vec<Pose> = (0..=n)
                .map(|i| {
                    let t = (-8.0 + 16.0 * i as f64 / n as f64).to_radians();
                    let a = unit([t.sin(), off.sin(), t.cos()]);
                    Pose {
                        tip: [i as f64 * 0.1, 5.0, 2.0],
                        axis: a,
                    }
                })
                .collect();
            let start = m.inverse(poses[0].tip, poses[0].axis, &[0.0; 5])[0]
                .q
                .clone();
            let blocks = path_to_axes(&m, &poses, &start, &opt);
            assert!(blocks.len() >= poses.len(), "{}", m.name);
            let mut prev = start.clone();
            let (mut jump, mut oerr, mut terr) = (0.0f64, 0.0f64, 0.0f64);
            for b in &blocks {
                for &i in &rot {
                    jump = jump.max((b.q[i] - prev[i]).abs());
                }
                oerr = oerr.max(b.orientation_error);
                terr = terr.max(b.tip_error);
                prev = b.q.clone();
            }
            eprintln!(
                "{}: {} poses -> {} blocks, largest rotary step {:.3}°, worst intermediate orientation error {:.3}°, tip error {:.1e}",
                m.name,
                poses.len(),
                blocks.len(),
                jump.to_degrees(),
                oerr.to_degrees(),
                terr
            );
            assert!(jump <= opt.max_joint_step + 1e-9, "{}: step {jump}", m.name);
            assert!(terr < 1e-9, "{}: tip error {terr}", m.name);
            assert!(
                oerr.to_degrees() < 0.1,
                "{}: orientation error {oerr}",
                m.name
            );
            // the last block lands exactly on the requested final pose
            let last = m.forward(&blocks.last().unwrap().q);
            assert!(dot(last.axis, poses[n].axis) > 1.0 - 1e-12, "{}", m.name);
        }
    }
}
