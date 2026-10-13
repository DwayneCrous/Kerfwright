//! Modal analysis: the lowest eigenpairs of `K φ = ω² M φ` by shift-and-invert subspace iteration with
//! Rayleigh–Ritz projection. The factorisation of `K + σM` is computed once and reused.

use crate::assemble::{assemble_mass, assemble_stiffness};
use crate::model::Model;
use crate::solve::Factor;
use crate::sparse::{dot, Csr};

#[derive(Debug, Clone)]
pub struct Modes {
    /// Natural frequencies in Hz, ascending.
    pub frequencies: Vec<f64>,
    /// Eigenvalues `ω²`.
    pub omega2: Vec<f64>,
    /// Mode shapes (M-orthonormal), one vector of `3 × nodes` per mode.
    pub shapes: Vec<Vec<f64>>,
    pub iterations: usize,
}

/// Lowest `count` modes of the model's free vibration with its fixed supports (prescribed values are
/// ignored). `shift` ≥ 0 allows free bodies (rigid-body modes then appear at `ω² = 0`).
pub fn modal(model: &Model, count: usize, shift: f64) -> Result<Modes, String> {
    let (mut k, pattern) = assemble_stiffness(&model.mesh, model.material.e, model.material.nu);
    let mut m = assemble_mass(&model.mesh, model.material.rho, &pattern);
    let n = k.n;
    let mut fixed = vec![false; n];
    for c in &model.fixed {
        fixed[3 * c.node as usize + c.axis] = true;
    }
    // fixed dofs: identity in K and zero in M, so they never take part in an eigenvector
    let mut dummy = vec![0.0; n];
    let list: Vec<(usize, f64)> = (0..n).filter(|&i| fixed[i]).map(|i| (i, 0.0)).collect();
    k.apply_dirichlet(&mut dummy, &list);
    zero_dofs(&mut m, &fixed);
    let mut a = k.clone();
    if shift != 0.0 {
        for i in 0..a.n {
            for kk in a.row_ptr[i]..a.row_ptr[i + 1] {
                a.val[kk] += shift * m.val[kk];
            }
        }
    }
    let factor = Factor::new(&a)?;

    let p = (2 * count).max(count + 8).min(n - list.len());
    // deterministic starting block
    let mut x: Vec<Vec<f64>> = (0..p)
        .map(|j| {
            (0..n)
                .map(|i| {
                    if fixed[i] {
                        0.0
                    } else {
                        let t = (i * 7919 + j * 104729 + 13) as f64;
                        (t * 0.618_033_988_749_894_8).fract() - 0.5
                    }
                })
                .collect()
        })
        .collect();
    let mut y = vec![0.0; n];
    let mut prev: Vec<f64> = vec![f64::INFINITY; count];
    let mut lambda: Vec<f64> = vec![0.0; p];
    let mut iterations = 0;
    for it in 0..200 {
        iterations = it + 1;
        // Y = (K + σM)⁻¹ M X
        for xj in x.iter_mut() {
            m.spmv(xj, &mut y);
            let mut z = y.clone();
            factor.solve_in_place(&mut z);
            *xj = z;
        }
        // Rayleigh–Ritz in span(X): small p×p problem  (XᵀKX) q = λ (XᵀMX) q
        let mut kx: Vec<Vec<f64>> = Vec::with_capacity(p);
        let mut mx: Vec<Vec<f64>> = Vec::with_capacity(p);
        for xj in &x {
            let mut t = vec![0.0; n];
            k.spmv(xj, &mut t);
            kx.push(t);
            let mut t = vec![0.0; n];
            m.spmv(xj, &mut t);
            mx.push(t);
        }
        let mut kr = vec![0.0; p * p];
        let mut mr = vec![0.0; p * p];
        for i in 0..p {
            for j in 0..=i {
                let a = dot(&x[i], &kx[j]);
                let b = dot(&x[i], &mx[j]);
                kr[i * p + j] = a;
                kr[j * p + i] = a;
                mr[i * p + j] = b;
                mr[j * p + i] = b;
            }
        }
        let (vals, vecs) = generalized_eigen(&kr, &mr, p)?;
        // rotate X into the Ritz vectors
        let mut nx: Vec<Vec<f64>> = vec![vec![0.0; n]; p];
        for (c, nxc) in nx.iter_mut().enumerate() {
            for (j, xj) in x.iter().enumerate() {
                let q = vecs[j * p + c];
                if q != 0.0 {
                    for i in 0..n {
                        nxc[i] += q * xj[i];
                    }
                }
            }
        }
        x = nx;
        lambda = vals;
        let done =
            (0..count).all(|i| (lambda[i] - prev[i]).abs() <= 1e-10 * lambda[i].abs().max(1e-300));
        prev[..count].copy_from_slice(&lambda[..count]);
        if done && it > 1 {
            break;
        }
    }
    let omega2: Vec<f64> = lambda[..count].to_vec();
    let frequencies = omega2
        .iter()
        .map(|w| w.max(0.0).sqrt() / (2.0 * std::f64::consts::PI))
        .collect();
    Ok(Modes {
        frequencies,
        omega2,
        shapes: x.into_iter().take(count).collect(),
        iterations,
    })
}

fn zero_dofs(m: &mut Csr, fixed: &[bool]) {
    for i in 0..m.n {
        for kk in m.row_ptr[i]..m.row_ptr[i + 1] {
            let c = m.col[kk] as usize;
            if fixed[i] || fixed[c] {
                m.val[kk] = 0.0;
            }
        }
    }
}

/// Eigenvalues (ascending) and M-orthonormal eigenvectors (columns, row-major `p×p`) of the dense
/// symmetric pencil `(A, B)`, `B` positive definite: Cholesky reduction and cyclic Jacobi rotations.
fn generalized_eigen(a: &[f64], b: &[f64], p: usize) -> Result<(Vec<f64>, Vec<f64>), String> {
    // B = L Lᵀ
    let mut l = vec![0.0; p * p];
    for i in 0..p {
        for j in 0..=i {
            let mut s = b[i * p + j];
            for k in 0..j {
                s -= l[i * p + k] * l[j * p + k];
            }
            if i == j {
                if s <= 0.0 {
                    return Err("subspace mass matrix is not positive definite".into());
                }
                l[i * p + i] = s.sqrt();
            } else {
                l[i * p + j] = s / l[j * p + j];
            }
        }
    }
    // C = L⁻¹ A L⁻ᵀ
    let mut c = a.to_vec();
    // forward substitution on columns, then rows
    for col in 0..p {
        for i in 0..p {
            let mut s = c[i * p + col];
            for k in 0..i {
                s -= l[i * p + k] * c[k * p + col];
            }
            c[i * p + col] = s / l[i * p + i];
        }
    }
    for row in 0..p {
        for j in 0..p {
            let mut s = c[row * p + j];
            for k in 0..j {
                s -= l[j * p + k] * c[row * p + k];
            }
            c[row * p + j] = s / l[j * p + j];
        }
    }
    // symmetrise
    for i in 0..p {
        for j in 0..i {
            let v = 0.5 * (c[i * p + j] + c[j * p + i]);
            c[i * p + j] = v;
            c[j * p + i] = v;
        }
    }
    let (vals, z) = jacobi_eigen(c, p);
    // eigenvectors of the pencil: Lᵀ q = z  →  q = L⁻ᵀ z
    let mut q = vec![0.0; p * p];
    for col in 0..p {
        for i in (0..p).rev() {
            let mut s = z[i * p + col];
            for k in i + 1..p {
                s -= l[k * p + i] * q[k * p + col];
            }
            q[i * p + col] = s / l[i * p + i];
        }
    }
    // sort ascending
    let mut order: Vec<usize> = (0..p).collect();
    order.sort_by(|&x, &y| vals[x].total_cmp(&vals[y]));
    let sorted_vals: Vec<f64> = order.iter().map(|&i| vals[i]).collect();
    let mut sorted_q = vec![0.0; p * p];
    for (nc, &oc) in order.iter().enumerate() {
        for i in 0..p {
            sorted_q[i * p + nc] = q[i * p + oc];
        }
    }
    Ok((sorted_vals, sorted_q))
}

/// Cyclic Jacobi for a dense symmetric matrix; returns eigenvalues and the eigenvector matrix (columns).
fn jacobi_eigen(mut a: Vec<f64>, p: usize) -> (Vec<f64>, Vec<f64>) {
    let mut v = vec![0.0; p * p];
    for i in 0..p {
        v[i * p + i] = 1.0;
    }
    for _sweep in 0..100 {
        let mut off = 0.0;
        for i in 0..p {
            for j in 0..i {
                off += a[i * p + j] * a[i * p + j];
            }
        }
        let diag: f64 = (0..p).map(|i| a[i * p + i] * a[i * p + i]).sum();
        if off <= 1e-28 * diag.max(1e-300) {
            break;
        }
        for pp in 0..p {
            for qq in pp + 1..p {
                let apq = a[pp * p + qq];
                if apq.abs() < 1e-300 {
                    continue;
                }
                let theta = (a[qq * p + qq] - a[pp * p + pp]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..p {
                    let (akp, akq) = (a[k * p + pp], a[k * p + qq]);
                    a[k * p + pp] = c * akp - s * akq;
                    a[k * p + qq] = s * akp + c * akq;
                }
                for k in 0..p {
                    let (apk, aqk) = (a[pp * p + k], a[qq * p + k]);
                    a[pp * p + k] = c * apk - s * aqk;
                    a[qq * p + k] = s * apk + c * aqk;
                }
                for k in 0..p {
                    let (vkp, vkq) = (v[k * p + pp], v[k * p + qq]);
                    v[k * p + pp] = c * vkp - s * vkq;
                    v[k * p + qq] = s * vkp + c * vkq;
                }
            }
        }
    }
    ((0..p).map(|i| a[i * p + i]).collect(), v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_generalized_problem() {
        // A = diag(1,4,9), B = diag(1,1,1) mixed by a rotation: eigenvalues 1, 4, 9
        let a = vec![2.0, 1.0, 0.0, 1.0, 3.0, 1.0, 0.0, 1.0, 2.0];
        let b = vec![2.0, 0.5, 0.0, 0.5, 1.0, 0.2, 0.0, 0.2, 1.5];
        let (vals, q) = generalized_eigen(&a, &b, 3).unwrap();
        for c in 0..3 {
            // A q = λ B q
            for i in 0..3 {
                let aq: f64 = (0..3).map(|j| a[i * 3 + j] * q[j * 3 + c]).sum();
                let bq: f64 = (0..3).map(|j| b[i * 3 + j] * q[j * 3 + c]).sum();
                assert!(
                    (aq - vals[c] * bq).abs() < 1e-10,
                    "{aq} vs {}",
                    vals[c] * bq
                );
            }
        }
        assert!(vals[0] <= vals[1] && vals[1] <= vals[2]);
    }
}
