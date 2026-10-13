//! Placing the components and closing kinematic loops.
//!
//! The joints are first walked as a tree from the grounded components (a spanning tree: each joint that
//! reaches an unplaced component places it). A joint whose two ends are both placed already **closes a
//! loop**; it is a constraint, not a placement: the pose it implies must equal the pose the tree gave.
//! The values of the joints marked `solved` are the unknowns; they are found by a damped Gauss-Newton
//! (Levenberg-Marquardt) iteration on the six-component gap of every closing joint, starting from the
//! values the joints hold, so a mechanism moved a little at a time stays on the branch it is on.

use super::*;
use crate::numeric::{rank, solve_small};

/// Rotation gaps are counted as this many mm per radian next to translation gaps.
const ROT_SCALE: f64 = 100.0;
/// A loop is closed when no component of its gap exceeds this (mm).
const CLOSED: f64 = 1e-9;
/// The best gap the solver may stop at and still be taken as closed.
const ACCEPT: f64 = 1e-6;

/// One placement pass over the joint tree.
pub(crate) struct TreePass {
    pub placed: Vec<Transform3>,
    pub via: Vec<Option<(usize, usize)>>,
    /// Joints whose two ends were both placed by others: loop closures.
    pub closing: Vec<usize>,
}

/// A `Param` list for the values of a joint of `kind` (radians / mm).
pub(crate) fn lit_values(kind: JointKind, vals: &[f64]) -> Vec<Param> {
    vals.iter()
        .zip(kind.dims())
        .map(|(x, dim)| {
            if *dim == Dim::ANGLE {
                Param::Lit(Quantity::angle(*x))
            } else {
                Param::length(*x)
            }
        })
        .collect()
}

fn inf_norm(r: &[f64]) -> f64 {
    r.iter().fold(0.0f64, |a, x| a.max(x.abs()))
}

/// What a motion sweep found at one value of the input.
#[derive(Debug, Clone)]
pub struct SweepStep {
    /// The input value (radians / mm).
    pub value: f64,
    pub placed: Vec<Transform3>,
}

/// The mobility of an assembly: how many independent inputs its mechanisms need.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mobility {
    /// Degrees of freedom: the joint values that can still change with the loops closed.
    pub dof: usize,
    /// All joint values.
    pub variables: usize,
    /// Closure equations (six per loop-closing joint).
    pub equations: usize,
    /// How many of the equations are independent.
    pub rank: usize,
    /// Equations that repeat others (a planar loop has three of six); the loop is still sound.
    pub redundant: usize,
    /// Values of driven joints.
    pub driven: usize,
    /// Values of joints marked `solved`.
    pub solved: usize,
}

impl Assembly {
    /// The placement of every instance, with the values of the solved joints (which a loop solve found).
    pub(crate) fn solve_loops(&self) -> Result<(TreePass, Vec<(usize, Vec<f64>)>), AssemblyError> {
        let scope = self.scope()?;
        let none: Vec<Option<Vec<f64>>> = Vec::new();
        let pass = self.place_tree(&scope, &none)?;
        let unknown: Vec<usize> = (0..self.joints.len())
            .filter(|i| self.joints[*i].solved && self.joints[*i].kind.arity() > 0)
            .collect();
        // stored values of the solved joints, held inside their limits
        let mut stored: Vec<Vec<f64>> = Vec::new();
        for &ji in &unknown {
            let j = &self.joints[ji];
            let mut v = self.joint_values_with(j, &scope, false)?;
            for (x, lim) in v.iter_mut().zip(&j.limits) {
                if let Some((lo, hi)) = lim {
                    *x = x.clamp(*lo, *hi);
                }
            }
            stored.push(v);
        }
        let as_values = |u: &[Vec<f64>]| -> Vec<(usize, Vec<f64>)> {
            unknown.iter().copied().zip(u.iter().cloned()).collect()
        };
        if pass.closing.is_empty() {
            return Ok((pass, as_values(&stored)));
        }
        let free: usize = stored.iter().map(|v| v.len()).sum();
        // a loop that is already closed at the stored values needs nothing
        let ov = self.overrides(&unknown, &stored);
        let pass = self.place_tree(&scope, &ov)?;
        let (r0, worst0) = self.closure_residual(&pass, &scope, &ov)?;
        if inf_norm(&r0) < CLOSED {
            return Ok((pass, as_values(&stored)));
        }
        if free == 0 {
            return Err(AssemblyError::LoopNotClosed {
                joint: self.joints[worst0].name.clone(),
                residual: inf_norm(&r0),
                free: 0,
            });
        }
        let (u, pass) = self.close_loops(&scope, &unknown, stored, pass, r0)?;
        Ok((pass, as_values(&u)))
    }

    /// As `solve_loops`, only the placement.
    pub(crate) fn solve_pass(&self) -> Result<TreePass, AssemblyError> {
        self.solve_loops().map(|(p, _)| p)
    }

    fn overrides(&self, unknown: &[usize], u: &[Vec<f64>]) -> Vec<Option<Vec<f64>>> {
        let mut ov: Vec<Option<Vec<f64>>> = vec![None; self.joints.len()];
        for (ji, v) in unknown.iter().zip(u) {
            ov[*ji] = Some(v.clone());
        }
        ov
    }

    /// Levenberg-Marquardt on the gaps of the closing joints. Returns the solved joints' values and the
    /// placement they give.
    fn close_loops(
        &self,
        scope: &HashMap<String, Quantity>,
        unknown: &[usize],
        start: Vec<Vec<f64>>,
        pass0: TreePass,
        r_start: Vec<f64>,
    ) -> Result<(Vec<Vec<f64>>, TreePass), AssemblyError> {
        // flat views of the unknowns
        let lens: Vec<usize> = start.iter().map(|v| v.len()).collect();
        let mut dims: Vec<Dim> = Vec::new();
        let mut lims: Vec<Option<(f64, f64)>> = Vec::new();
        for &ji in unknown {
            let j = &self.joints[ji];
            dims.extend(j.kind.dims().iter().copied());
            for k in 0..j.kind.arity() {
                lims.push(j.limits.get(k).copied().flatten());
            }
        }
        let k: usize = lens.iter().sum();
        let split = |flat: &[f64]| -> Vec<Vec<f64>> {
            let mut out = Vec::new();
            let mut at = 0;
            for l in &lens {
                out.push(flat[at..at + l].to_vec());
                at += l;
            }
            out
        };
        let clamp = |u: &mut [f64]| {
            for (x, lim) in u.iter_mut().zip(&lims) {
                if let Some((lo, hi)) = lim {
                    *x = x.clamp(*lo, *hi);
                }
            }
        };
        let eval = |u: &[f64]| -> Result<(TreePass, Vec<f64>, usize), AssemblyError> {
            let ov = self.overrides(unknown, &split(u));
            let pass = self.place_tree(scope, &ov)?;
            let (r, worst) = self.closure_residual(&pass, scope, &ov)?;
            Ok((pass, r, worst))
        };
        let mut u: Vec<f64> = start.into_iter().flatten().collect();
        let mut pass = pass0;
        let mut r = r_start;
        let mut worst = {
            // the worst joint of the starting gap, for the error message
            let ov = self.overrides(unknown, &split(&u));
            self.closure_residual(&pass, scope, &ov)?.1
        };
        let mut lambda = 1e-3;
        for _ in 0..200 {
            if inf_norm(&r) < CLOSED {
                break;
            }
            // forward-difference Jacobian: one column per unknown
            let m = r.len();
            let mut jac = vec![vec![0.0f64; m]; k];
            for c in 0..k {
                let eps = 1e-6;
                let mut up = u.clone();
                up[c] += eps;
                clamp(&mut up);
                if (up[c] - u[c]).abs() < 0.5 * eps {
                    up[c] = u[c] - eps;
                    clamp(&mut up);
                }
                let h = up[c] - u[c];
                if h.abs() < 1e-300 {
                    continue;
                }
                let (_, r1, _) = eval(&up)?;
                for e in 0..m {
                    jac[c][e] = (r1[e] - r[e]) / h;
                }
            }
            let mut a = vec![vec![0.0f64; k]; k];
            let mut g = vec![0.0f64; k];
            for p in 0..k {
                for q in 0..k {
                    a[p][q] = (0..m).map(|e| jac[p][e] * jac[q][e]).sum();
                }
                g[p] = -(0..m).map(|e| jac[p][e] * r[e]).sum::<f64>();
            }
            let f = r.iter().map(|x| x * x).sum::<f64>();
            let mut improved = false;
            for _ in 0..14 {
                let mut damped = a.clone();
                for p in 0..k {
                    damped[p][p] += lambda * (a[p][p] + 1e-9);
                }
                let Some(delta) = solve_small(damped, g.clone()) else {
                    lambda *= 10.0;
                    continue;
                };
                // keep a step from jumping across the circle
                let mut step = 1.0f64;
                for c in 0..k {
                    let cap = if dims[c] == Dim::ANGLE { 0.5 } else { 25.0 };
                    step = step.min(cap / delta[c].abs().max(1e-300));
                }
                let mut un = u.clone();
                for c in 0..k {
                    un[c] += delta[c] * step;
                }
                clamp(&mut un);
                let (pn, rn, wn) = eval(&un)?;
                if rn.iter().map(|x| x * x).sum::<f64>() < f {
                    u = un;
                    pass = pn;
                    r = rn;
                    worst = wn;
                    lambda = (lambda / 3.0).max(1e-12);
                    improved = true;
                    break;
                }
                lambda *= 4.0;
            }
            if !improved {
                break;
            }
        }
        if inf_norm(&r) > ACCEPT {
            return Err(AssemblyError::LoopNotClosed {
                joint: self.joints[worst].name.clone(),
                residual: inf_norm(&r),
                free: k,
            });
        }
        Ok((split(&u), pass))
    }

    /// The gap of every closing joint (a translation in mm and a rotation vector times [`ROT_SCALE`]), and
    /// the closing joint with the largest gap. `ov` replaces joint values by index.
    fn closure_residual(
        &self,
        pass: &TreePass,
        scope: &HashMap<String, Quantity>,
        ov: &[Option<Vec<f64>>],
    ) -> Result<(Vec<f64>, usize), AssemblyError> {
        let mut out = Vec::new();
        let mut worst = (-1.0f64, pass.closing.first().copied().unwrap_or(0));
        for &ji in &pass.closing {
            let j = &self.joints[ji];
            let vals = match ov.get(ji) {
                Some(Some(v)) => v.clone(),
                _ => self.joint_values_with(j, scope, !j.solved)?,
            };
            let fa = self.frame_of(&j.parent)?;
            let fb = self.frame_of(&j.child)?;
            let motion = Self::joint_transform(j, &vals);
            // where the joint says the child must be, given where the parent is
            let expected = world_of(&fb)
                .inverse()
                .then(&motion)
                .then(&world_of(&fa))
                .then(&pass.placed[j.parent.instance]);
            let actual = &pass.placed[j.child.instance];
            let d = actual.then(&expected.inverse());
            let t = d.0.translation.vector;
            let w = d.0.rotation.scaled_axis() * ROT_SCALE;
            let gap = [t.x, t.y, t.z, w.x, w.y, w.z];
            let size = inf_norm(&gap);
            if size > worst.0 {
                worst = (size, ji);
            }
            out.extend(gap);
        }
        Ok((out, worst.1))
    }

    /// Place the components from the joint tree. `ov` replaces the values of joints by index (empty:
    /// none are replaced). A joint that reaches two placed components closes a loop and is listed in
    /// `closing`; two grounded components joined together are still refused.
    pub(crate) fn place_tree(
        &self,
        scope: &HashMap<String, Quantity>,
        ov: &[Option<Vec<f64>>],
    ) -> Result<TreePass, AssemblyError> {
        if self.instances.is_empty() {
            return Ok(TreePass {
                placed: Vec::new(),
                via: Vec::new(),
                closing: Vec::new(),
            });
        }
        let n = self.instances.len();
        if let Some(g) = &self.grounds {
            if let Some(bad) = g.iter().find(|i| **i >= n) {
                return Err(AssemblyError::UnknownInstance(*bad));
            }
        } else if self.ground >= n {
            return Err(AssemblyError::UnknownInstance(self.ground));
        }
        let grounded = self.grounded();
        // a joint between two members of one rigid group cannot move anything
        for j in &self.joints {
            if let (Some(a), Some(b)) = (
                self.group_of(j.parent.instance),
                self.group_of(j.child.instance),
            ) {
                if a == b {
                    return Err(AssemblyError::Redundant {
                        joint: j.name.clone(),
                        group: self.groups[a].name.clone(),
                    });
                }
            }
        }
        // anything a joint or a rigid group touches is placed by them; the rest floats where it is
        let mut tied = vec![false; n];
        for j in &self.joints {
            for e in [&j.parent, &j.child] {
                if e.instance < n {
                    tied[e.instance] = true;
                }
            }
        }
        // a group that nothing else places floats as one: its first member stays where it is
        let mut group_seeds: Vec<usize> = Vec::new();
        for g in &self.groups {
            let anchored = g
                .members
                .iter()
                .any(|m| *m < n && (tied[*m] || grounded.contains(m)));
            if let Some(&a) = g.members.first() {
                if !anchored && a < n {
                    group_seeds.push(a);
                }
            }
            for m in &g.members {
                if *m < n {
                    tied[*m] = true;
                }
            }
        }
        let own = |i: usize| self.instances[i].at.to_transform();
        let mut placed: Vec<Option<Transform3>> = vec![None; n];
        let mut seeded = vec![false; n];
        let mut via: Vec<Option<(usize, usize)>> = vec![None; n];
        for i in 0..n {
            if grounded.contains(&i) || !tied[i] || group_seeds.contains(&i) {
                placed[i] = Some(own(i));
                seeded[i] = true;
            }
        }
        let mut done = vec![false; self.joints.len()];
        let mut closing: Vec<usize> = Vec::new();
        loop {
            let mut progress = false;
            // rigid groups: one placed member places the rest
            for g in &self.groups {
                let Some(&anchor) = g.members.first() else {
                    continue;
                };
                if anchor >= n || g.rel.len() != g.members.len() {
                    continue;
                }
                let rel = |k: usize| g.rel[k].to_transform();
                let known = g
                    .members
                    .iter()
                    .position(|m| *m < n && placed[*m].is_some());
                let Some(k) = known else { continue };
                let anchor_placed = if k == 0 {
                    placed[anchor].unwrap()
                } else {
                    rel(k).inverse().then(&placed[g.members[k]].unwrap())
                };
                for (idx, m) in g.members.iter().enumerate() {
                    if *m < n && placed[*m].is_none() {
                        placed[*m] = Some(rel(idx).then(&anchor_placed));
                        progress = true;
                    }
                }
            }
            for (ji, j) in self.joints.iter().enumerate() {
                if done[ji] {
                    continue;
                }
                let (p, c) = (j.parent.instance, j.child.instance);
                if p >= n {
                    return Err(AssemblyError::UnknownInstance(p));
                }
                if c >= n {
                    return Err(AssemblyError::UnknownInstance(c));
                }
                // joints may be given in either direction: orient towards the unplaced instance
                let (from, to, flipped) = match (placed[p].is_some(), placed[c].is_some()) {
                    (true, false) => (&j.parent, &j.child, false),
                    (false, true) => (&j.child, &j.parent, true),
                    (true, true) => {
                        if seeded[p] && seeded[c] {
                            return Err(AssemblyError::BothGrounded {
                                joint: j.name.clone(),
                            });
                        }
                        // both ends are placed by other joints: this one closes a loop
                        closing.push(ji);
                        done[ji] = true;
                        continue;
                    }
                    (false, false) => continue,
                };
                let vals = match ov.get(ji) {
                    Some(Some(v)) => v.clone(),
                    _ => self.joint_values_with(j, scope, !j.solved)?,
                };
                let fa = self.frame_of(from)?;
                let fb = self.frame_of(to)?;
                let mut motion = Self::joint_transform(j, &vals);
                if flipped {
                    motion = motion.inverse();
                }
                // child coordinates → child frame → motion → parent frame → parent coordinates → world
                let t = world_of(&fb)
                    .inverse()
                    .then(&motion)
                    .then(&world_of(&fa))
                    .then(placed[from.instance].as_ref().unwrap());
                placed[to.instance] = Some(t);
                via[to.instance] = Some((ji, from.instance));
                done[ji] = true;
                progress = true;
            }
            if !progress {
                break;
            }
        }
        for (i, p) in placed.iter().enumerate() {
            if p.is_none() {
                return Err(AssemblyError::Disconnected(self.instances[i].name.clone()));
            }
        }
        for (ji, j) in self.joints.iter().enumerate() {
            if !done[ji] {
                return Err(AssemblyError::Loop {
                    joint: j.name.clone(),
                    instance: self.instances[j.child.instance].name.clone(),
                });
            }
        }
        Ok(TreePass {
            placed: placed.into_iter().map(|p| p.unwrap()).collect(),
            via,
            closing,
        })
    }

    /// Solve the assembly and keep the values the loop solve found in the solved joints, so that the next
    /// solve starts from them and a mechanism moved step by step stays on its branch. Returns the
    /// placements; on an error nothing is changed.
    pub fn settle(&mut self) -> Result<Vec<Transform3>, AssemblyError> {
        let (pass, values) = self.solve_loops()?;
        for (ji, v) in values {
            let kind = self.joints[ji].kind;
            self.joints[ji].values = lit_values(kind, &v);
        }
        Ok(pass.placed)
    }

    /// The degrees of freedom the mechanisms have with their loops closed: all joint values, minus the
    /// independent closure equations (found by the rank of the numerical Jacobian, so planar and
    /// over-constrained linkages are counted right).
    pub fn mobility(&self) -> Result<Mobility, AssemblyError> {
        let scope = self.scope()?;
        let (pass, solved) = self.solve_loops()?;
        // every joint's current values
        let mut cur: Vec<Vec<f64>> = Vec::new();
        for (ji, j) in self.joints.iter().enumerate() {
            let v = match solved.iter().find(|(k, _)| *k == ji) {
                Some((_, v)) => v.clone(),
                None => self.joint_values_with(j, &scope, false)?,
            };
            cur.push(v);
        }
        let variables: usize = cur.iter().map(|v| v.len()).sum();
        let equations = 6 * pass.closing.len();
        let driven: usize = self
            .joints
            .iter()
            .filter(|j| !j.solved)
            .map(|j| j.kind.arity())
            .sum();
        let solved_n = variables - driven;
        let mut cols: Vec<Vec<f64>> = Vec::new();
        if equations > 0 {
            for ji in 0..self.joints.len() {
                for c in 0..cur[ji].len() {
                    let eps = 1e-6;
                    let mut cols_r = Vec::new();
                    for sign in [1.0, -1.0] {
                        let mut ov: Vec<Option<Vec<f64>>> =
                            cur.iter().map(|v| Some(v.clone())).collect();
                        ov[ji].as_mut().unwrap()[c] += sign * eps;
                        let p = self.place_tree(&scope, &ov)?;
                        cols_r.push(self.closure_residual(&p, &scope, &ov)?.0);
                    }
                    let col: Vec<f64> = cols_r[0]
                        .iter()
                        .zip(&cols_r[1])
                        .map(|(a, b)| (a - b) / (2.0 * eps))
                        .collect();
                    let norm = col.iter().map(|x| x * x).sum::<f64>().sqrt();
                    cols.push(if norm > 1e-7 {
                        col.iter().map(|x| x / norm).collect()
                    } else {
                        vec![0.0; equations]
                    });
                }
            }
        }
        // rows = equations, columns = variables
        let rows: Vec<Vec<f64>> = (0..equations)
            .map(|e| cols.iter().map(|c| c[e]).collect())
            .collect();
        let rk = rank(rows, 1e-5);
        Ok(Mobility {
            dof: variables - rk,
            variables,
            equations,
            rank: rk,
            redundant: equations - rk,
            driven,
            solved: solved_n,
        })
    }

    /// Move the value `index` of the driven joint `joint` from `from` to `to` (radians / mm) in `steps`
    /// equal steps (`steps + 1` poses), solving the loops at each, each step starting from the one before.
    /// Stops at the first value the mechanism cannot reach, and says why. The joint values are put back
    /// as they were.
    pub fn sweep(
        &mut self,
        joint: usize,
        index: usize,
        from: f64,
        to: f64,
        steps: usize,
    ) -> Result<(Vec<SweepStep>, Option<AssemblyError>), AssemblyError> {
        let j = self
            .joints
            .get(joint)
            .ok_or_else(|| AssemblyError::Invalid(format!("there is no joint {joint}")))?;
        if j.solved {
            return Err(AssemblyError::Invalid(format!(
                "joint '{}' is solved by the loops: sweep a driven joint",
                j.name
            )));
        }
        if index >= j.kind.arity() {
            return Err(AssemblyError::Invalid(format!(
                "joint '{}' has no value {index}",
                j.name
            )));
        }
        let kind = j.kind;
        let saved = self.joints.clone();
        let mut out = Vec::new();
        let mut stopped = None;
        for s in 0..=steps.max(1) {
            let x = from + (to - from) * s as f64 / steps.max(1) as f64;
            let mut vals = match self.joint_values_with(&self.joints[joint], &self.scope()?, false)
            {
                Ok(v) => v,
                Err(e) => {
                    stopped = Some(e);
                    break;
                }
            };
            vals[index] = x;
            self.joints[joint].values = lit_values(kind, &vals);
            match self.settle() {
                Ok(placed) => out.push(SweepStep { value: x, placed }),
                Err(e) => {
                    stopped = Some(e);
                    break;
                }
            }
        }
        // put back the inputs (the settled solved values go with them)
        self.joints = saved;
        Ok((out, stopped))
    }
}
