//! Degree-of-freedom, redundancy and conflict analysis.
//!
//! Rank is computed from the Jacobian at the solution (dense SVD per
//! component; for very large components, per Dulmage–Mendelsohn block, which
//! is then a lower bound on the rank). A system whose residual cannot be
//! driven to zero is *conflicting*; a minimal conflicting subset of
//! constraints is found with a deletion filter that re-solves subsystems from
//! several starts (so a stuck local minimum is not mistaken for a conflict).

use crate::constraint::ConstraintId;
use crate::entity::{EntityId, EntityKind, VarId};
use crate::graph::Component;
use crate::residual::{Jac, MAXR, MAXV};
use crate::sketch::{Sketch, SketchError};
use crate::solve::{solve_system, SolveStatus, Solver};
use crate::system::System;
use nalgebra::DMatrix;
use std::collections::HashSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchStatus {
    /// Solvable with remaining freedom (`dof > 0`), no redundancy.
    UnderConstrained,
    /// Solvable, no freedom left, no redundancy.
    FullyConstrained,
    /// Solvable, but some constraints are dependent on others (consistent).
    OverConstrained,
    /// The constraints contradict each other.
    Conflicting,
}

/// Result of the Jacobian rank analysis at the solution.
#[derive(Debug, Clone)]
pub struct DofReport {
    pub status: SketchStatus,
    /// Remaining degrees of freedom (`unknowns − rank`).
    pub dof: usize,
    /// Variables that can still move.
    pub free_vars: Vec<VarId>,
    /// Entities with at least one free variable.
    pub free_entities: Vec<EntityId>,
    /// Constraints that are linearly dependent on earlier ones (consistent).
    pub redundant: Vec<ConstraintId>,
    /// A minimal set of constraints that cannot be satisfied together.
    pub conflicting: Vec<ConstraintId>,
    /// Largest dimensionless residual after solving.
    pub residual: f64,
    /// `false` if some component was too large for a dense SVD and a
    /// block-wise lower bound on the rank was used.
    pub exact: bool,
}

const EXACT_LIMIT: usize = 350;

/// Dense weighted Jacobian of `rows` with respect to `cols` (global ids).
fn dense_jacobian(sys: &System, rows: &[u32], cols: &[u32], x: &[f64]) -> DMatrix<f64> {
    let mut col_of = std::collections::HashMap::with_capacity(cols.len());
    for (c, &g) in cols.iter().enumerate() {
        col_of.insert(g, c);
    }
    let mut j = DMatrix::<f64>::zeros(rows.len(), cols.len());
    let mut row_index = std::collections::HashMap::with_capacity(rows.len());
    for (r, &g) in rows.iter().enumerate() {
        row_index.insert(g, r);
    }
    let mut jl: Jac = [[0.0; MAXV]; MAXR];
    let mut seen = HashSet::new();
    for &gr in rows {
        let ri = sys.row_res[gr as usize] as usize;
        if !seen.insert(ri) {
            continue;
        }
        sys.jac_res(ri, x, &mut jl);
        for eq in 0..sys.res[ri].kind.arity() {
            let grow = (sys.row_start[ri] + eq) as u32;
            let Some(&r) = row_index.get(&grow) else {
                continue;
            };
            let w = sys.row_w[grow as usize];
            for (slot, &g) in sys.res[ri].vars.iter().enumerate() {
                if let Some(&c) = col_of.get(&g) {
                    j[(r, c)] += jl[eq][slot] * w;
                }
            }
        }
    }
    j
}

/// Numerical rank, nullspace indicator per column, and the set of rows that
/// are dependent on earlier rows.
struct RankInfo {
    rank: usize,
    /// For each column, the norm of its row in the nullspace basis.
    null_weight: Vec<f64>,
    dependent_rows: Vec<usize>,
}

fn rank_info(j: &DMatrix<f64>) -> RankInfo {
    let (m, n) = j.shape();
    if n == 0 {
        // no unknowns: every equation row is "dependent" (a pure check)
        return RankInfo {
            rank: 0,
            null_weight: Vec::new(),
            dependent_rows: (0..m).collect(),
        };
    }
    // pad to at least n rows so the SVD returns the full right-singular basis
    let padded = if m < n {
        let mut p = DMatrix::<f64>::zeros(n, n);
        p.view_mut((0, 0), (m, n)).copy_from(j);
        p
    } else {
        j.clone()
    };
    let svd = padded.svd(false, true);
    let vt = svd.v_t.expect("requested V^T");
    let smax = svd.singular_values.iter().cloned().fold(0.0f64, f64::max);
    let tol = (1e-8 * smax).max(1e-12);
    let rank = svd.singular_values.iter().filter(|s| **s > tol).count();
    let mut null_weight = vec![0.0; n];
    for (k, s) in svd.singular_values.iter().enumerate() {
        if *s <= tol {
            for c in 0..n {
                null_weight[c] += vt[(k, c)] * vt[(k, c)];
            }
        }
    }
    // dependent rows by greedy modified Gram–Schmidt in row order
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut dependent = Vec::new();
    for r in 0..m {
        let mut v: Vec<f64> = (0..n).map(|c| j[(r, c)]).collect();
        let norm0 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        for _ in 0..2 {
            for b in &basis {
                let d: f64 = v.iter().zip(b).map(|(x, y)| x * y).sum();
                for (x, y) in v.iter_mut().zip(b) {
                    *x -= d * y;
                }
            }
        }
        let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm <= tol.max(1e-9 * norm0.max(1.0)) || norm0 == 0.0 {
            dependent.push(r);
        } else {
            for x in v.iter_mut() {
                *x /= norm;
            }
            basis.push(v);
        }
    }
    RankInfo {
        rank,
        null_weight,
        dependent_rows: dependent,
    }
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64) - 0.5
    }
}

impl Solver {
    /// Solve, then analyse degrees of freedom, redundancy and conflicts.
    pub fn analyze(&mut self, sk: &mut Sketch) -> Result<DofReport, SketchError> {
        let x0 = sk.vars.values.clone();
        let mut outcome = self.solve(sk)?;
        let opts = self.opts.clone();
        if outcome.status == SolveStatus::Failed {
            // A stuck local minimum is not proof of a conflict: retry from a few
            // deterministic perturbations before declaring one.
            let sys = &self.cache().expect("prepared by solve").sys;
            let mut lcg = Lcg(0x9E3779B97F4A7C15);
            for _ in 0..6 {
                let mut x = x0.clone();
                for (v, xv) in x.iter_mut().enumerate() {
                    if !sys.fixed[v] {
                        *xv += 0.08 * sys.scale * lcg.next();
                    }
                }
                let o = solve_system(sys, &mut x, &opts);
                if o.status == SolveStatus::Converged {
                    sk.vars.values.copy_from_slice(&x);
                    outcome = o;
                    break;
                }
            }
        }
        let cache = self.cache().expect("prepared by solve");
        let sys = &cache.sys;
        let x = sk.vars.values.clone();

        let mut dof = 0usize;
        let mut free_vars: Vec<VarId> = Vec::new();
        let mut redundant: Vec<ConstraintId> = Vec::new();
        let mut exact = true;
        let mut touched = vec![false; sys.n_vars];

        for (ci, comp) in cache.comps.iter().enumerate() {
            for &c in &comp.cols {
                touched[c as usize] = true;
            }
            let cols: Vec<u32> = comp
                .cols
                .iter()
                .copied()
                .filter(|&c| !sys.fixed[c as usize])
                .collect();
            if cols.len() <= EXACT_LIMIT && comp.rows.len() <= 3 * EXACT_LIMIT {
                let j = dense_jacobian(sys, &comp.rows, &cols, &x);
                let info = rank_info(&j);
                dof += cols.len() - info.rank.min(cols.len());
                for (c, w) in info.null_weight.iter().enumerate() {
                    if *w > 1e-9 {
                        free_vars.push(VarId(cols[c] as usize));
                    }
                }
                for r in info.dependent_rows {
                    if let Some(id) = sys.row_constraint(comp.rows[r] as usize) {
                        redundant.push(id);
                    }
                }
            } else {
                exact = false;
                let dec = &cache.decomps[ci];
                for &g in &dec.free_cols {
                    dof += 1;
                    free_vars.push(VarId(g as usize));
                }
                for blk in &dec.blocks {
                    let j = dense_jacobian(sys, &blk.rows, &blk.cols, &x);
                    let info = rank_info(&j);
                    dof += blk.cols.len() - info.rank.min(blk.cols.len());
                    for (c, w) in info.null_weight.iter().enumerate() {
                        if *w > 1e-9 {
                            free_vars.push(VarId(blk.cols[c] as usize));
                        }
                    }
                    for r in info.dependent_rows {
                        if let Some(id) = sys.row_constraint(blk.rows[r] as usize) {
                            redundant.push(id);
                        }
                    }
                }
            }
        }
        // unknown variables that no equation touches are entirely free
        for (v, t) in touched.iter().enumerate() {
            if !t && !sys.fixed[v] && self.var_in_entity(sk, v) {
                dof += 1;
                free_vars.push(VarId(v));
            }
        }
        free_vars.sort();
        free_vars.dedup();
        redundant.sort();
        redundant.dedup();

        let mut conflicting = Vec::new();
        let status = if outcome.status == SolveStatus::Failed {
            conflicting =
                self.minimal_conflict(sk, &x0, &outcome_failed_components(self, sk)?, &opts)?;
            SketchStatus::Conflicting
        } else if !redundant.is_empty() {
            SketchStatus::OverConstrained
        } else if dof > 0 {
            SketchStatus::UnderConstrained
        } else {
            SketchStatus::FullyConstrained
        };
        if status == SketchStatus::Conflicting {
            redundant.clear();
        }

        let free_set: HashSet<usize> = free_vars.iter().map(|v| v.0).collect();
        let mut free_entities = Vec::new();
        for (id, e) in &sk.entities {
            let mut vars = Vec::new();
            if let EntityKind::Point(p) = &e.kind {
                vars.extend([p.x.0, p.y.0]);
            } else if let EntityKind::Circle { radius, .. } = &e.kind {
                vars.push(radius.0);
            } else if let EntityKind::Ellipse { minor_radius, .. } = &e.kind {
                vars.push(minor_radius.0);
            }
            if vars.iter().any(|v| free_set.contains(v)) {
                free_entities.push(id);
            }
        }
        Ok(DofReport {
            status,
            dof,
            free_vars,
            free_entities,
            redundant,
            conflicting,
            residual: outcome.residual,
            exact,
        })
    }

    fn var_in_entity(&self, sk: &Sketch, v: usize) -> bool {
        // dimension values and other bookkeeping variables are fixed already;
        // any unfixed variable belongs to an entity
        let _ = (sk, v);
        true
    }

    /// Deletion filter over the constraints of the failing components.
    fn minimal_conflict(
        &mut self,
        sk: &Sketch,
        x0: &[f64],
        failing: &[Component],
        opts: &crate::solve::SolveOptions,
    ) -> Result<Vec<ConstraintId>, SketchError> {
        let sys = &self.cache().expect("prepared").sys;
        // candidate constraints: owners of the failing components' rows, plus
        // Fix constraints touching their variables
        let mut cand: Vec<ConstraintId> = Vec::new();
        let mut vars: HashSet<usize> = HashSet::new();
        for comp in failing {
            for &r in &comp.rows {
                if let Some(id) = sys.row_constraint(r as usize) {
                    cand.push(id);
                }
                // every variable the row reads, including pinned ones (Fix candidates)
                vars.extend(
                    sys.res[sys.row_res[r as usize] as usize]
                        .vars
                        .iter()
                        .map(|&g| g as usize),
                );
            }
            vars.extend(comp.cols.iter().map(|&c| c as usize));
        }
        for (id, c) in &sk.constraints {
            if let crate::constraint::ConstraintKind::Fix(e) = &c.kind {
                if c.enabled && sk.entity_vars(*e)?.iter().any(|v| vars.contains(&v.0)) {
                    cand.push(id);
                }
            }
        }
        // dimension constraints whose value variable is shared are handled as normal rows
        cand.sort();
        cand.dedup();
        let scale = sys.scale;
        let conflicting = |set: &HashSet<ConstraintId>| -> Result<bool, SketchError> {
            let sub = System::build_filtered(sk, &|id| set.contains(&id))?;
            let mut lcg = Lcg(0x2545F4914F6CDD1D);
            for start in 0..4 {
                let mut x = x0.to_vec();
                if start > 0 {
                    for (v, xv) in x.iter_mut().enumerate() {
                        if !sub.fixed[v] {
                            *xv += 0.05 * scale * lcg.next();
                        }
                    }
                }
                if solve_system(&sub, &mut x, opts).status == SolveStatus::Converged {
                    return Ok(false);
                }
            }
            Ok(true)
        };
        let mut set: HashSet<ConstraintId> = cand.iter().copied().collect();
        if cand.len() > 200 || !conflicting(&set)? {
            return Ok(cand); // too many to filter, or the subsystem is not conflicting on its own
        }
        for c in cand.iter().rev() {
            set.remove(c);
            if !conflicting(&set)? {
                set.insert(*c);
            }
        }
        let mut out: Vec<ConstraintId> = set.into_iter().collect();
        out.sort();
        Ok(out)
    }
}

fn outcome_failed_components(solver: &Solver, sk: &Sketch) -> Result<Vec<Component>, SketchError> {
    let cache = solver.cache().expect("prepared");
    let sys = &cache.sys;
    let x = sk.vars.values.as_slice();
    let mut out = Vec::new();
    let mut tmp = [0.0; MAXR];
    for comp in &cache.comps {
        let mut worst = 0.0f64;
        for &row in &comp.rows {
            let ri = sys.row_res[row as usize] as usize;
            sys.eval_res(ri, x, &mut tmp);
            let eq = row as usize - sys.row_start[ri];
            worst = worst.max((tmp[eq] * sys.row_w[row as usize]).abs());
        }
        let mut last = usize::MAX;
        let degenerate = comp.rows.iter().any(|&row| {
            let ri = sys.row_res[row as usize] as usize;
            let bad = ri != last && !sys.residual_valid(ri, x);
            last = ri;
            bad
        });
        if worst > solver.opts.tol || degenerate {
            out.push(comp.clone());
        }
    }
    Ok(out)
}
