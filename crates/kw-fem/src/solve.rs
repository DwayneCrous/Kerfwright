use crate::sparse::{dot, norm, Csr};
use faer::linalg::solvers::SolveCore;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMatRef};
use faer::{Conj, MatMut, Side};
use rayon::prelude::*;

/// A sparse Cholesky factorisation (AMD ordering) of a symmetric positive definite matrix, kept for
/// repeated solves.
pub struct Factor {
    n: usize,
    llt: Llt<u32, f64>,
}

impl Factor {
    pub fn new(a: &Csr) -> Result<Factor, String> {
        let n = a.n;
        // lower triangle in CSC = upper triangle (including the diagonal) of each CSR row
        let mut col_ptr: Vec<u32> = Vec::with_capacity(n + 1);
        let mut row_idx: Vec<u32> = Vec::new();
        let mut vals: Vec<f64> = Vec::new();
        col_ptr.push(0);
        for i in 0..n {
            let (s, e) = (a.row_ptr[i], a.row_ptr[i + 1]);
            let start = s + a.col[s..e].partition_point(|c| (*c as usize) < i);
            row_idx.extend_from_slice(&a.col[start..e]);
            vals.extend_from_slice(&a.val[start..e]);
            col_ptr.push(row_idx.len() as u32);
        }
        let sym = SymbolicSparseColMatRef::new_checked(n, n, &col_ptr[..], None, &row_idx[..]);
        let symbolic =
            SymbolicLlt::try_new(sym, Side::Lower).map_err(|e| format!("symbolic: {e:?}"))?;
        let mat = SparseColMatRef::new(sym, &vals[..]);
        let llt = Llt::try_new_with_symbolic(symbolic, mat, Side::Lower)
            .map_err(|e| format!("factorisation: {e:?}"))?;
        Ok(Factor { n, llt })
    }

    pub fn solve_in_place(&self, x: &mut [f64]) {
        self.llt
            .solve_in_place_with_conj(Conj::No, MatMut::from_column_major_slice_mut(x, self.n, 1));
    }

    pub fn solve(&self, b: &[f64]) -> Vec<f64> {
        let mut x = b.to_vec();
        self.solve_in_place(&mut x);
        x
    }
}

/// Solve `A x = b` for symmetric positive definite `A` by sparse Cholesky (AMD ordering).
pub fn solve_direct(a: &Csr, b: &[f64]) -> Result<Vec<f64>, String> {
    Ok(Factor::new(a)?.solve(b))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Precond {
    Jacobi,
    /// 3×3 nodal blocks of the diagonal.
    BlockJacobi,
}

#[derive(Debug, Clone)]
pub struct PcgReport {
    pub iterations: usize,
    /// ‖b − A x‖ / ‖b‖ at exit (recomputed, not the recurrence).
    pub residual: f64,
    pub converged: bool,
}

enum Pre {
    Diag(Vec<f64>),
    Block(Vec<[f64; 9]>),
}

impl Pre {
    fn new(a: &Csr, kind: Precond) -> Pre {
        match kind {
            Precond::Jacobi => Pre::Diag(a.diagonal().into_iter().map(|d| 1.0 / d).collect()),
            Precond::BlockJacobi => {
                let nn = a.n / 3;
                Pre::Block(
                    (0..nn)
                        .into_par_iter()
                        .map(|i| {
                            let mut m = [0.0; 9];
                            for r in 0..3 {
                                for c in 0..3 {
                                    m[3 * r + c] = a.get(3 * i + r, 3 * i + c);
                                }
                            }
                            inv3(&m)
                        })
                        .collect(),
                )
            }
        }
    }

    fn apply(&self, r: &[f64], z: &mut [f64]) {
        match self {
            Pre::Diag(d) => z
                .par_iter_mut()
                .zip(r.par_iter().zip(d.par_iter()))
                .for_each(|(z, (r, d))| *z = r * d),
            Pre::Block(b) => z
                .par_chunks_mut(3)
                .zip(r.par_chunks(3).zip(b.par_iter()))
                .for_each(|(z, (r, m))| {
                    for i in 0..3 {
                        z[i] = m[3 * i] * r[0] + m[3 * i + 1] * r[1] + m[3 * i + 2] * r[2];
                    }
                }),
        }
    }
}

fn inv3(m: &[f64; 9]) -> [f64; 9] {
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);
    let id = 1.0 / det;
    [
        (m[4] * m[8] - m[5] * m[7]) * id,
        (m[2] * m[7] - m[1] * m[8]) * id,
        (m[1] * m[5] - m[2] * m[4]) * id,
        (m[5] * m[6] - m[3] * m[8]) * id,
        (m[0] * m[8] - m[2] * m[6]) * id,
        (m[2] * m[3] - m[0] * m[5]) * id,
        (m[3] * m[7] - m[4] * m[6]) * id,
        (m[1] * m[6] - m[0] * m[7]) * id,
        (m[0] * m[4] - m[1] * m[3]) * id,
    ]
}

/// Preconditioned conjugate gradients on `A x = b` from `x = 0`, stopping at `‖r‖/‖b‖ ≤ tol`.
pub fn pcg(
    a: &Csr,
    b: &[f64],
    kind: Precond,
    tol: f64,
    max_iter: usize,
    cancel: Option<&kw_job::CancelFlag>,
) -> (Vec<f64>, PcgReport) {
    let n = a.n;
    let pre = Pre::new(a, kind);
    let bn = norm(b).max(1e-300);
    let mut x = vec![0.0; n];
    let mut r = b.to_vec();
    let mut z = vec![0.0; n];
    pre.apply(&r, &mut z);
    let mut p = z.clone();
    let mut ap = vec![0.0; n];
    let mut rz = dot(&r, &z);
    let mut iters = 0;
    while iters < max_iter {
        if norm(&r) / bn <= tol {
            break;
        }
        if cancel.is_some_and(|c| c.is_cancelled()) {
            break;
        }
        a.spmv(&p, &mut ap);
        let alpha = rz / dot(&p, &ap);
        x.par_iter_mut()
            .zip(p.par_iter())
            .for_each(|(x, p)| *x += alpha * p);
        r.par_iter_mut()
            .zip(ap.par_iter())
            .for_each(|(r, a)| *r -= alpha * a);
        pre.apply(&r, &mut z);
        let rz_new = dot(&r, &z);
        let beta = rz_new / rz;
        rz = rz_new;
        p.par_iter_mut()
            .zip(z.par_iter())
            .for_each(|(p, z)| *p = z + beta * *p);
        iters += 1;
    }
    // true residual
    a.spmv(&x, &mut ap);
    let res: f64 = r
        .par_iter()
        .zip(b.par_iter().zip(ap.par_iter()))
        .map(|(_, (b, a))| (b - a) * (b - a))
        .sum::<f64>()
        .sqrt()
        / bn;
    (
        x,
        PcgReport {
            iterations: iters,
            residual: res,
            converged: res <= tol * 1.5,
        },
    )
}
