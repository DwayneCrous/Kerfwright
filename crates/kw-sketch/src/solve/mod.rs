//! Clustered nonlinear solver.
//!
//! The compiled system is split into connected components; each component is
//! put into block-triangular form by a Dulmage–Mendelsohn decomposition and
//! solved block by block (damped Gauss–Newton/LM, dog-leg and BFGS fallbacks).
//! If a block fails, the whole component is re-solved as one least-squares
//! problem with all of its variables free (minimum-norm step).
//!
//! Components write disjoint variables, so they are solved in parallel with
//! no cross-thread floating-point reductions; results are bit-identical for
//! any thread count.

mod dense;
mod drag;
mod lm;
mod sparse;

pub use drag::DragOutcome;
pub use lm::{bfgs, dogleg, lm, LmOpts, LmResult, LmWs, Problem};

use crate::graph::{self, make_block, Block, Component, Decomp};
use crate::residual::{Jac, MAXR};
use crate::sketch::{Sketch, SketchError};
use crate::system::{System, XAccess};
use rayon::prelude::*;
use std::cell::RefCell;

#[derive(Clone, Debug)]
pub struct SolveOptions {
    /// Converged when the largest dimensionless residual is below this.
    pub tol: f64,
    pub max_iter: usize,
    /// Solve independent components on the rayon pool.
    pub parallel: bool,
    /// Minimum number of rows before parallelism is used.
    pub par_min_rows: usize,
    /// Largest component (unknowns) re-solved densely when a block fails.
    pub max_dense: usize,
    /// Refuse (bisect) drag moves that flip a tangent side, joint direction or radius sign.
    pub prevent_flips: bool,
}

impl Default for SolveOptions {
    fn default() -> Self {
        Self {
            tol: 1e-11,
            max_iter: 60,
            parallel: true,
            par_min_rows: 4000,
            max_dense: 200,
            prevent_flips: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolveStatus {
    Converged,
    /// Some rows could not be satisfied (conflicting constraints or a stuck local minimum).
    Failed,
}

#[derive(Clone, Debug)]
pub struct SolveOutcome {
    pub status: SolveStatus,
    /// Largest dimensionless (weighted) residual after solving.
    pub residual: f64,
    pub components: usize,
    pub blocks: usize,
    pub failed_blocks: usize,
    /// Components that needed the dense whole-component fallback.
    pub escalated: usize,
    pub iterations: usize,
}

/// Shared, disjointly-written view of the variable vector.
///
/// SAFETY contract: while an `XRef` is shared between threads, each thread
/// only *writes* variables owned by its component and only *reads* variables of
/// its own component or fixed variables (which nobody writes). Components share
/// no unknown variables by construction.
#[derive(Clone, Copy)]
pub struct XRef {
    ptr: *mut f64,
    len: usize,
}

unsafe impl Send for XRef {}
unsafe impl Sync for XRef {}

impl XRef {
    pub fn new(x: &mut [f64]) -> Self {
        Self {
            ptr: x.as_mut_ptr(),
            len: x.len(),
        }
    }
    #[inline(always)]
    pub fn set(&self, i: usize, v: f64) {
        debug_assert!(i < self.len);
        unsafe { *self.ptr.add(i) = v }
    }
}

impl XAccess for XRef {
    #[inline(always)]
    fn get(&self, i: usize) -> f64 {
        debug_assert!(i < self.len);
        unsafe { *self.ptr.add(i) }
    }
}

/// Per-thread scratch space.
#[derive(Default)]
pub struct Scratch {
    col_of: Vec<i32>,
    xl: Vec<f64>,
    x0: Vec<f64>,
    best: Vec<f64>,
    ws: LmWs,
}

thread_local! {
    static SCRATCH: RefCell<Scratch> = RefCell::new(Scratch::default());
}

fn with_scratch<R>(n_vars: usize, f: impl FnOnce(&mut Scratch) -> R) -> R {
    SCRATCH.with(|s| {
        let mut s = s.borrow_mut();
        if s.col_of.len() < n_vars {
            s.col_of.resize(n_vars, -1);
        }
        f(&mut s)
    })
}

struct BlockProblem<'a> {
    sys: &'a System,
    blk: &'a Block,
    x: XRef,
    col_of: &'a mut [i32],
}

impl BlockProblem<'_> {
    fn scatter(&self, xl: &[f64]) {
        for (c, &g) in self.blk.cols.iter().enumerate() {
            self.x.set(g as usize, xl[c]);
        }
    }
}

impl Problem for BlockProblem<'_> {
    fn feasible(&mut self) -> bool {
        self.blk
            .res_list
            .iter()
            .all(|&ri| self.sys.residual_valid(ri as usize, &self.x))
    }
    fn m(&self) -> usize {
        self.blk.rows.len()
    }
    fn n(&self) -> usize {
        self.blk.cols.len()
    }
    fn eval(&mut self, xl: &[f64], r: &mut [f64]) {
        self.scatter(xl);
        let mut out = [0.0; MAXR];
        for (pos, &ri) in self.blk.res_list.iter().enumerate() {
            self.sys.eval_res(ri as usize, &self.x, &mut out);
            for &(row, eq) in &self.blk.res_rows[pos] {
                r[row as usize] =
                    out[eq as usize] * self.sys.row_w[self.blk.rows[row as usize] as usize];
            }
        }
    }
    fn jac(&mut self, xl: &[f64], j: &mut [f64]) {
        self.scatter(xl);
        let n = self.blk.cols.len();
        for v in j.iter_mut() {
            *v = 0.0;
        }
        for (c, &g) in self.blk.cols.iter().enumerate() {
            self.col_of[g as usize] = c as i32;
        }
        let mut jl: Jac = [[0.0; crate::residual::MAXV]; MAXR];
        for (pos, &ri) in self.blk.res_list.iter().enumerate() {
            self.sys.jac_res(ri as usize, &self.x, &mut jl);
            let vars = &self.sys.res[ri as usize].vars;
            for (slot, &g) in vars.iter().enumerate() {
                let c = self.col_of[g as usize];
                if c < 0 {
                    continue;
                }
                for &(row, eq) in &self.blk.res_rows[pos] {
                    j[row as usize * n + c as usize] += jl[eq as usize][slot]
                        * self.sys.row_w[self.blk.rows[row as usize] as usize];
                }
            }
        }
        for &g in &self.blk.cols {
            self.col_of[g as usize] = -1;
        }
    }
}

pub(crate) struct BlockResult {
    pub converged: bool,
    pub iters: usize,
}

/// Solve one block in place (LM → dog-leg → BFGS); leaves the best point in `x`.
pub(crate) fn solve_block(
    sys: &System,
    blk: &Block,
    x: XRef,
    sc: &mut Scratch,
    opts: &SolveOptions,
    max_step: f64,
    min_norm: bool,
) -> BlockResult {
    let n = blk.cols.len();
    let mut xl = std::mem::take(&mut sc.xl);
    let mut x0 = std::mem::take(&mut sc.x0);
    let mut best = std::mem::take(&mut sc.best);
    xl.clear();
    xl.extend(blk.cols.iter().map(|&g| x.get(g as usize)));
    x0.clear();
    x0.extend_from_slice(&xl);
    best.clear();
    best.extend_from_slice(&xl);
    let lo = LmOpts {
        tol: opts.tol,
        max_iter: opts.max_iter,
        max_step,
        lambda0: 1e-12,
        unit_scaling: min_norm,
    };
    let mut total = 0;
    let mut best_r = f64::INFINITY;
    let mut converged = false;
    {
        let mut prob = BlockProblem {
            sys,
            blk,
            x,
            col_of: &mut sc.col_of,
        };
        for attempt in 0..3 {
            if attempt > 0 {
                xl.copy_from_slice(&x0);
            }
            if (attempt == 2 && n > 64) || (attempt >= 1 && n > 120) {
                break;
            }
            let res = match attempt {
                0 => lm(&mut prob, &mut xl, &lo, &mut sc.ws),
                1 => dogleg(&mut prob, &mut xl, &lo, &mut sc.ws),
                _ => bfgs(&mut prob, &mut xl, &lo, &mut sc.ws),
            };
            total += res.iters;
            if res.rinf < best_r {
                best_r = res.rinf;
                best.copy_from_slice(&xl);
            }
            if res.converged {
                converged = true;
                break;
            }
        }
        prob.scatter(&best);
    }
    if converged
        && !blk
            .res_list
            .iter()
            .all(|&ri| sys.residual_valid(ri as usize, &x))
    {
        // satisfied only by a degenerate configuration (zero-length line, radius ≤ 0)
        converged = false;
    }
    sc.xl = xl;
    sc.x0 = x0;
    sc.best = best;
    BlockResult {
        converged,
        iters: total,
    }
}

pub(crate) struct CompResult {
    pub failed_blocks: usize,
    pub escalated: bool,
    pub iterations: usize,
    pub residual: f64,
}

fn component_residual(sys: &System, comp: &Component, x: &XRef) -> f64 {
    let mut worst = 0.0f64;
    let mut out = [0.0; MAXR];
    let mut last = u32::MAX;
    for &row in &comp.rows {
        let ri = sys.row_res[row as usize];
        if ri != last {
            sys.eval_res(ri as usize, x, &mut out);
            last = ri;
        }
        let eq = row as usize - sys.row_start[ri as usize];
        worst = worst.max((out[eq] * sys.row_w[row as usize]).abs());
    }
    worst
}

/// Solve one component with its decomposition.
pub(crate) fn solve_component(
    sys: &System,
    comp: &Component,
    dec: &Decomp,
    cluster: &sparse::Cluster,
    x: XRef,
    sc: &mut Scratch,
    opts: &SolveOptions,
) -> CompResult {
    let mut res = CompResult {
        failed_blocks: 0,
        escalated: false,
        iterations: 0,
        residual: 0.0,
    };
    // warm path: already satisfied
    let pre = component_residual(sys, comp, &x);
    if pre <= opts.tol {
        res.residual = pre;
        return res;
    }
    let max_step = 0.5 * sys.scale.max(1.0);
    let snapshot: Vec<f64> = comp.cols.iter().map(|&g| x.get(g as usize)).collect();

    // Primary: sparse damped Gauss–Newton over the whole component. It finds the
    // nearest solution and needs no choice of "free" variables.
    let sr = sparse::sparse_lm(sys, cluster, x, sc, opts, max_step, None, opts.max_iter);
    res.iterations += sr.iters;
    if sr.converged {
        res.residual = component_residual(sys, comp, &x);
        let valid = {
            let mut last = u32::MAX;
            comp.rows.iter().all(|&r| {
                let ri = sys.row_res[r as usize];
                let ok = ri == last || sys.residual_valid(ri as usize, &x);
                last = ri;
                ok
            })
        };
        if valid && res.residual <= opts.tol {
            return res;
        }
    }
    for (&g, &v) in comp.cols.iter().zip(&snapshot) {
        x.set(g as usize, v);
    }

    // Fallback: block-triangular solve over the Dulmage–Mendelsohn decomposition,
    // cluster merges, numeric re-matching, dense whole-component solve.
    let run_blocks = |dec: &Decomp, sc: &mut Scratch, res: &mut CompResult| -> Vec<usize> {
        let mut failed = Vec::new();
        for (bi, blk) in dec.blocks.iter().enumerate() {
            let r = solve_block(sys, blk, x, sc, opts, max_step, false);
            res.iterations += r.iters;
            if !r.converged {
                failed.push(bi);
            }
        }
        failed
    };
    let mut failed = run_blocks(dec, sc, &mut res);
    // A failed block usually means its matched variable is a poor lever. Merge it
    // with the blocks that produce the variables it reads and solve the cluster
    // jointly (minimum-norm), then redo the sequence.
    for _round in 0..2 {
        if failed.is_empty() {
            break;
        }
        for &f in &failed {
            let mut members: Vec<u32> = vec![f as u32];
            for _depth in 0..3 {
                let mut next = members.clone();
                for &m in &members {
                    next.extend(dec.deps[m as usize].iter().copied());
                }
                next.sort_unstable();
                next.dedup();
                if next.len() == members.len() {
                    break;
                }
                members = next;
                let cols: usize = members
                    .iter()
                    .map(|&m| dec.blocks[m as usize].cols.len())
                    .sum();
                if cols > 150 {
                    break;
                }
                let mut rows: Vec<u32> = members
                    .iter()
                    .flat_map(|&m| dec.blocks[m as usize].rows.iter().copied())
                    .collect();
                let mut cs: Vec<u32> = members
                    .iter()
                    .flat_map(|&m| dec.blocks[m as usize].cols.iter().copied())
                    .collect();
                rows.sort_unstable();
                cs.sort_unstable();
                let blk = make_block(sys, rows, cs, false);
                let r = solve_block(sys, &blk, x, sc, opts, max_step, true);
                res.iterations += r.iters;
                if r.converged {
                    break;
                }
            }
        }
        failed = run_blocks(dec, sc, &mut res);
    }
    res.failed_blocks = failed.len();
    if res.failed_blocks > 0 {
        // A matched variable may be numerically irrelevant at this geometry; re-match
        // using the numeric Jacobian pattern and continue from where we are.
        let snapshot_x: Vec<f64> = (0..sys.n_vars).map(|i| x.get(i)).collect();
        let dec2 = graph::decompose_numeric(sys, comp, &[], &snapshot_x);
        let mut failed = 0;
        for blk in &dec2.blocks {
            let r = solve_block(sys, blk, x, sc, opts, max_step, false);
            res.iterations += r.iters;
            if !r.converged {
                failed += 1;
            }
        }
        res.failed_blocks = failed;
        res.residual = component_residual(sys, comp, &x);
    }
    let invalid = |x: &XRef| {
        let mut last = u32::MAX;
        comp.rows.iter().any(|&r| {
            let ri = sys.row_res[r as usize];
            let bad = ri != last && !sys.residual_valid(ri as usize, x);
            last = ri;
            bad
        })
    };
    if res.residual <= opts.tol && invalid(&x) {
        // satisfied only by a degenerate configuration: treat as unsolved
        res.residual = res.residual.max(1.0);
    }
    if res.residual > opts.tol && comp.cols.len() <= opts.max_dense {
        // dense fallback: everything in the component free, min-norm LM
        res.escalated = true;
        let cols: Vec<u32> = comp
            .cols
            .iter()
            .copied()
            .filter(|&g| !sys.fixed[g as usize])
            .collect();
        let over = comp.rows.len() > cols.len();
        let blk = make_block(sys, comp.rows.clone(), cols, over);
        let before = res.residual;
        let r = solve_block(sys, &blk, x, sc, opts, max_step, true);
        res.iterations += r.iters;
        let mut after = component_residual(sys, comp, &x);
        if after <= opts.tol && invalid(&x) {
            after = after.max(1.0);
        }
        if after > before {
            // the fallback made things worse: restore the block-wise result? use snapshot of start
            for (&g, &v) in comp.cols.iter().zip(&snapshot) {
                x.set(g as usize, v);
            }
            res.residual = component_residual(sys, comp, &x);
        } else {
            res.residual = after;
        }
        if res.residual > opts.tol {
            res.failed_blocks = res.failed_blocks.max(1);
        } else {
            res.failed_blocks = 0;
        }
    }
    res
}

pub(crate) fn component_cluster(sys: &System, comp: &Component) -> sparse::Cluster {
    let cols: Vec<u32> = comp
        .cols
        .iter()
        .copied()
        .filter(|&g| !sys.fixed[g as usize])
        .collect();
    sparse::Cluster::new(sys, comp.rows.clone(), cols)
}

/// Everything derived from the sketch topology.
pub struct Cache {
    pub version: u64,
    pub sys: System,
    pub comps: Vec<Component>,
    pub decomps: Vec<Decomp>,
    /// One sparse cluster (symbolic factorisation) per component.
    pub(crate) clusters: Vec<sparse::Cluster>,
    /// Component index of each variable (`graph::NONE` if no equation touches it).
    pub var_comp: Vec<u32>,
}

impl Cache {
    fn build(sk: &Sketch) -> Result<Cache, SketchError> {
        let sys = System::build(sk)?;
        let comps = graph::components(&sys);
        let decomps: Vec<Decomp> = comps
            .par_iter()
            .map(|c| graph::decompose(&sys, c, &[], &sk.vars.values))
            .collect();
        let clusters: Vec<sparse::Cluster> = comps
            .par_iter()
            .map(|c| component_cluster(&sys, c))
            .collect();
        let mut var_comp = vec![graph::NONE; sys.n_vars];
        for (ci, c) in comps.iter().enumerate() {
            for &v in &c.cols {
                var_comp[v as usize] = ci as u32;
            }
        }
        Ok(Cache {
            version: sk.topology_version(),
            sys,
            comps,
            decomps,
            clusters,
            var_comp,
        })
    }
}

/// Solve a freshly compiled system on a scratch variable vector (no caching,
/// sequential). Used by analysis, which probes many constraint subsets.
pub fn solve_system(sys: &System, x: &mut [f64], opts: &SolveOptions) -> SolveOutcome {
    let comps = graph::components(sys);
    let decomps: Vec<Decomp> = comps
        .iter()
        .map(|c| graph::decompose(sys, c, &[], x))
        .collect();
    let xr = XRef::new(x);
    let clusters: Vec<sparse::Cluster> = comps.iter().map(|c| component_cluster(sys, c)).collect();
    let results: Vec<CompResult> = with_scratch(sys.n_vars, |sc| {
        comps
            .iter()
            .zip(&decomps)
            .zip(&clusters)
            .map(|((c, d), cl)| solve_component(sys, c, d, cl, xr, sc, opts))
            .collect()
    });
    let residual = results.iter().fold(0.0f64, |m, r| m.max(r.residual));
    SolveOutcome {
        status: if residual <= opts.tol {
            SolveStatus::Converged
        } else {
            SolveStatus::Failed
        },
        residual,
        components: comps.len(),
        blocks: decomps.iter().map(|d| d.blocks.len()).sum(),
        failed_blocks: results.iter().map(|r| r.failed_blocks).sum(),
        escalated: results.iter().filter(|r| r.escalated).count(),
        iterations: results.iter().map(|r| r.iterations).sum(),
    }
}

/// The sketch solver. Holds compiled/decomposed state keyed by
/// [`Sketch::topology_version`], so repeated solves and drags are cheap.
pub struct Solver {
    pub opts: SolveOptions,
    cache: Option<Cache>,
    drag: Option<drag::DragCache>,
}

impl Default for Solver {
    fn default() -> Self {
        Self::new(SolveOptions::default())
    }
}

impl Solver {
    pub fn new(opts: SolveOptions) -> Self {
        Self {
            opts,
            cache: None,
            drag: None,
        }
    }

    /// Compile and decompose if the sketch topology changed.
    pub fn prepare(&mut self, sk: &Sketch) -> Result<&Cache, SketchError> {
        if self
            .cache
            .as_ref()
            .is_none_or(|c| c.version != sk.topology_version())
        {
            self.cache = Some(Cache::build(sk)?);
            self.drag = None;
        }
        Ok(self.cache.as_ref().unwrap())
    }

    pub fn cache(&self) -> Option<&Cache> {
        self.cache.as_ref()
    }

    /// Solve all constraints, updating `sk.vars.values`.
    pub fn solve(&mut self, sk: &mut Sketch) -> Result<SolveOutcome, SketchError> {
        self.prepare(sk)?;
        let opts = self.opts.clone();
        let cache = self.cache.as_ref().unwrap();
        let sys = &cache.sys;
        let n_vars = sys.n_vars;
        let x = XRef::new(&mut sk.vars.values);
        let results: Vec<CompResult> =
            if opts.parallel && sys.n_rows >= opts.par_min_rows && cache.comps.len() > 1 {
                cache
                    .comps
                    .par_iter()
                    .zip(cache.decomps.par_iter())
                    .zip(cache.clusters.par_iter())
                    .map(|((c, d), cl)| {
                        with_scratch(n_vars, |sc| solve_component(sys, c, d, cl, x, sc, &opts))
                    })
                    .collect()
            } else {
                with_scratch(n_vars, |sc| {
                    cache
                        .comps
                        .iter()
                        .zip(&cache.decomps)
                        .zip(&cache.clusters)
                        .map(|((c, d), cl)| solve_component(sys, c, d, cl, x, sc, &opts))
                        .collect()
                })
            };
        let residual = results.iter().fold(0.0f64, |m, r| m.max(r.residual));
        let failed: usize = results.iter().map(|r| r.failed_blocks).sum();
        Ok(SolveOutcome {
            status: if residual <= opts.tol {
                SolveStatus::Converged
            } else {
                SolveStatus::Failed
            },
            residual,
            components: cache.comps.len(),
            blocks: cache.decomps.iter().map(|d| d.blocks.len()).sum(),
            failed_blocks: failed,
            escalated: results.iter().filter(|r| r.escalated).count(),
            iterations: results.iter().map(|r| r.iterations).sum(),
        })
    }

    /// Largest dimensionless residual of the current geometry.
    pub fn residual(&mut self, sk: &Sketch) -> Result<f64, SketchError> {
        self.prepare(sk)?;
        let sys = &self.cache.as_ref().unwrap().sys;
        let mut worst = 0.0f64;
        let mut out = [0.0; MAXR];
        for i in 0..sys.res.len() {
            sys.eval_res(i, sk.vars.values.as_slice(), &mut out);
            for k in 0..sys.res[i].kind.arity() {
                worst = worst.max((out[k] * sys.row_w[sys.row_start[i] + k]).abs());
            }
        }
        Ok(worst)
    }
}
