//! Sparse damped Gauss–Newton (Levenberg–Marquardt) on a *cluster* of rows and
//! unknowns, using a sparse Cholesky factorisation of `JᵀJ + λI` from `faer`.
//!
//! Identity damping makes every rank-deficient step the minimum-Euclidean-norm
//! change, so under-determined sketches move as little as possible, and solving
//! the whole connected cluster at once avoids the fragility of pinning
//! "free" variables chosen by a structural matching. The symbolic
//! factorisation depends only on the cluster's sparsity and is computed once.

use super::{Scratch, SolveOptions, XRef};
use crate::graph::{make_block, Block};
use crate::residual::{Jac, MAXR, MAXV};
use crate::system::{System, XAccess};
use faer::linalg::solvers::SolveCore;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMatRef};
use faer::{Conj, MatMut, Side};

pub(crate) struct Cluster {
    pub block: Block,
    n: usize,
    /// Per row: sorted local columns, flattened with offsets.
    row_off: Vec<u32>,
    row_cols: Vec<u32>,
    /// CSC (lower triangle) of `JᵀJ`.
    col_ptr: Vec<u32>,
    row_idx: Vec<u32>,
    /// For each row and each pair `a ≥ b` of its columns: position in `values`.
    pair_pos: Vec<u32>,
    diag_pos: Vec<u32>,
    symbolic: Option<SymbolicLlt<u32>>,
}

impl Cluster {
    pub fn new(sys: &System, rows: Vec<u32>, cols: Vec<u32>) -> Cluster {
        let block = make_block(sys, rows, cols, false);
        let n = block.cols.len();
        let mut row_off = vec![0u32];
        let mut row_cols: Vec<u32> = Vec::new();
        let mut pat = Vec::new();
        for &r in &block.rows {
            sys.row_pattern(r as usize, &mut pat);
            let mut loc: Vec<u32> = pat
                .iter()
                .filter_map(|g| block.cols.binary_search(g).ok().map(|i| i as u32))
                .collect();
            loc.sort_unstable();
            loc.dedup();
            row_cols.extend_from_slice(&loc);
            row_off.push(row_cols.len() as u32);
        }
        // lower-triangle pattern
        let mut cols_lists: Vec<Vec<u32>> = (0..n).map(|j| vec![j as u32]).collect();
        for i in 0..block.rows.len() {
            let l = &row_cols[row_off[i] as usize..row_off[i + 1] as usize];
            for a in 0..l.len() {
                for b in 0..a {
                    // entry (row=l[a], col=l[b]) with l[a] > l[b]
                    cols_lists[l[b] as usize].push(l[a]);
                }
            }
        }
        let mut col_ptr = Vec::with_capacity(n + 1);
        let mut row_idx: Vec<u32> = Vec::new();
        col_ptr.push(0u32);
        for l in cols_lists.iter_mut() {
            l.sort_unstable();
            l.dedup();
            row_idx.extend_from_slice(l);
            col_ptr.push(row_idx.len() as u32);
        }
        let pos = |row: u32, col: u32| -> u32 {
            let (s, e) = (
                col_ptr[col as usize] as usize,
                col_ptr[col as usize + 1] as usize,
            );
            (s + row_idx[s..e].binary_search(&row).expect("entry in pattern")) as u32
        };
        let mut pair_pos = Vec::new();
        for i in 0..block.rows.len() {
            let l = &row_cols[row_off[i] as usize..row_off[i + 1] as usize];
            for a in 0..l.len() {
                for b in 0..=a {
                    pair_pos.push(pos(l[a], l[b]));
                }
            }
        }
        let diag_pos: Vec<u32> = (0..n as u32).map(|j| pos(j, j)).collect();
        let symbolic = if n > 0 {
            let sym = SymbolicSparseColMatRef::new_checked(n, n, &col_ptr[..], None, &row_idx[..]);
            SymbolicLlt::try_new(sym, Side::Lower).ok()
        } else {
            None
        };
        Cluster {
            block,
            n,
            row_off,
            row_cols,
            col_ptr,
            row_idx,
            pair_pos,
            diag_pos,
            symbolic,
        }
    }
}

/// Soft pull `w·(x_v − t)/scale` on selected variables (slide-to-closest drag).
pub(crate) struct Soft<'a> {
    pub targets: &'a [(u32, f64)],
    pub w: f64,
}

pub(crate) struct SparseResult {
    pub converged: bool,
    pub iters: usize,
}

fn eval_rows(sys: &System, cl: &Cluster, x: &XRef, r: &mut [f64]) {
    let mut out = [0.0; MAXR];
    for (pos, &ri) in cl.block.res_list.iter().enumerate() {
        sys.eval_res(ri as usize, x, &mut out);
        for &(row, eq) in &cl.block.res_rows[pos] {
            r[row as usize] = out[eq as usize] * sys.row_w[cl.block.rows[row as usize] as usize];
        }
    }
}

fn feasible(sys: &System, cl: &Cluster, x: &XRef) -> bool {
    cl.block
        .res_list
        .iter()
        .all(|&ri| sys.residual_valid(ri as usize, x))
}

fn rinf(r: &[f64]) -> f64 {
    r.iter().fold(0.0f64, |m, v| m.max(v.abs()))
}

/// LM on the cluster. `x` is read and updated in place (restored to the best point).
pub(crate) fn sparse_lm(
    sys: &System,
    cl: &Cluster,
    x: XRef,
    sc: &mut Scratch,
    opts: &SolveOptions,
    max_step: f64,
    soft: Option<&Soft<'_>>,
    max_iter: usize,
) -> SparseResult {
    let n = cl.n;
    let m = cl.block.rows.len();
    let blk = &cl.block;
    let Some(symbolic) = &cl.symbolic else {
        let mut r = vec![0.0; m];
        eval_rows(sys, cl, &x, &mut r);
        let ri = rinf(&r);
        return SparseResult {
            converged: ri <= opts.tol,
            iters: 0,
        };
    };
    let scale = sys.scale;
    let soft_col: Vec<usize> = soft
        .map(|s| {
            s.targets
                .iter()
                .map(|(g, _)| blk.cols.binary_search(g).expect("soft variable in cluster"))
                .collect()
        })
        .unwrap_or_default();
    let soft_w = soft.map_or(0.0, |s| s.w / scale);

    let mut r = vec![0.0; m];
    let mut rt = vec![0.0; m];
    let mut vals = vec![0.0; cl.row_idx.len()];
    let mut g = vec![0.0; n];
    let mut p = vec![0.0; n];
    let mut jv = vec![0.0; cl.row_cols.len()];
    let cols = &blk.cols;
    let get = |c: usize| x.get(cols[c] as usize);

    let cost = |r: &[f64], x: &XRef| -> f64 {
        let mut f: f64 = r.iter().map(|v| v * v).sum();
        if let Some(s) = soft {
            for (k, (gid, t)) in s.targets.iter().enumerate() {
                let d = soft_w * (x.get(*gid as usize) - t);
                f += d * d;
                let _ = k;
            }
        }
        f
    };

    eval_rows(sys, cl, &x, &mut r);
    let mut f = cost(&r, &x);
    let mut ri = rinf(&r);
    if soft.is_none() && ri <= opts.tol {
        return SparseResult {
            converged: true,
            iters: 0,
        };
    }
    let mut lambda_rel = 1e-9;
    let mut iters = 0;
    let mut converged = false;
    let col_of = &mut sc.col_of;
    let mut jl: Jac = [[0.0; MAXV]; MAXR];
    let mut best_x: Vec<f64> = (0..n).map(get).collect();
    let mut stalled = 0;
    while iters < max_iter {
        iters += 1;
        // ---- assemble H = JᵀJ and g = Jᵀ r ---------------------------------
        for v in vals.iter_mut() {
            *v = 0.0;
        }
        for v in g.iter_mut() {
            *v = 0.0;
        }
        for v in jv.iter_mut() {
            *v = 0.0;
        }
        for (c, &gid) in cols.iter().enumerate() {
            col_of[gid as usize] = c as i32;
        }
        for (pos, &ri_) in blk.res_list.iter().enumerate() {
            sys.jac_res(ri_ as usize, &x, &mut jl);
            let vars = &sys.res[ri_ as usize].vars;
            for (slot, &gid) in vars.iter().enumerate() {
                let c = col_of[gid as usize];
                if c < 0 {
                    continue;
                }
                for &(row, eq) in &blk.res_rows[pos] {
                    let (lo, hi) = (
                        cl.row_off[row as usize] as usize,
                        cl.row_off[row as usize + 1] as usize,
                    );
                    // entries outside the row's structural pattern are exactly zero
                    if let Ok(k) = cl.row_cols[lo..hi].binary_search(&(c as u32)) {
                        jv[lo + k] +=
                            jl[eq as usize][slot] * sys.row_w[blk.rows[row as usize] as usize];
                    }
                }
            }
        }
        for &gid in cols.iter() {
            col_of[gid as usize] = -1;
        }
        let mut pp = 0usize;
        for i in 0..m {
            let (lo, hi) = (cl.row_off[i] as usize, cl.row_off[i + 1] as usize);
            let l = &cl.row_cols[lo..hi];
            let jr = &jv[lo..hi];
            for a in 0..l.len() {
                g[l[a] as usize] += jr[a] * r[i];
                for b in 0..=a {
                    vals[cl.pair_pos[pp] as usize] += jr[a] * jr[b];
                    pp += 1;
                }
            }
        }
        if soft_w > 0.0 {
            let s = soft.unwrap();
            for (k, (gid, t)) in s.targets.iter().enumerate() {
                let c = soft_col[k];
                g[c] += soft_w * soft_w * (x.get(*gid as usize) - t);
                vals[cl.diag_pos[c] as usize] += soft_w * soft_w;
            }
        }
        let diag_mean =
            cl.diag_pos.iter().map(|&d| vals[d as usize]).sum::<f64>() / n.max(1) as f64;
        let diag_mean = diag_mean.max(1e-300);

        // ---- damped step with accept/reject --------------------------------
        let base = vals.clone();
        let mut accepted = false;
        let mut pinf = 0.0;
        for _ in 0..10 {
            vals.copy_from_slice(&base);
            let lam = lambda_rel * diag_mean;
            for &d in &cl.diag_pos {
                vals[d as usize] += lam;
            }
            let sym =
                SymbolicSparseColMatRef::new_checked(n, n, &cl.col_ptr[..], None, &cl.row_idx[..]);
            let mat = SparseColMatRef::new(sym, &vals[..]);
            let llt = match Llt::try_new_with_symbolic(symbolic.clone(), mat, Side::Lower) {
                Ok(l) => l,
                Err(_) => {
                    lambda_rel = (lambda_rel * 100.0).max(1e-9);
                    continue;
                }
            };
            for c in 0..n {
                p[c] = -g[c];
            }
            llt.solve_in_place_with_conj(
                Conj::No,
                MatMut::from_column_major_slice_mut(&mut p[..], n, 1),
            );
            pinf = p.iter().fold(0.0f64, |mx, v| mx.max(v.abs()));
            if !pinf.is_finite() {
                lambda_rel = (lambda_rel * 100.0).max(1e-9);
                continue;
            }
            let k = if pinf > max_step {
                max_step / pinf
            } else {
                1.0
            };
            for c in 0..n {
                x.set(cols[c] as usize, best_x[c] + k * p[c]);
            }
            eval_rows(sys, cl, &x, &mut rt);
            let ft = cost(&rt, &x);
            if ft.is_finite() && ft < f && feasible(sys, cl, &x) {
                f = ft;
                std::mem::swap(&mut r, &mut rt);
                for c in 0..n {
                    best_x[c] = x.get(cols[c] as usize);
                }
                lambda_rel = (lambda_rel * 0.1).max(1e-12);
                accepted = true;
                pinf *= k;
                break;
            }
            lambda_rel = (lambda_rel * 10.0).max(1e-9);
        }
        // restore the best point (a rejected trial may have been left in x)
        for c in 0..n {
            x.set(cols[c] as usize, best_x[c]);
        }
        if !accepted {
            break;
        }
        ri = rinf(&r);
        if soft.is_none() && ri <= opts.tol {
            converged = true;
            break;
        }
        let xinf = best_x.iter().fold(0.0f64, |mx, v| mx.max(v.abs()));
        if pinf <= 1e-14 * (1.0 + xinf) {
            stalled += 1;
            if stalled >= 2 {
                break;
            }
        } else {
            stalled = 0;
        }
    }
    SparseResult { converged, iters }
}
