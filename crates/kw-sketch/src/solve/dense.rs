//! Small dense linear algebra for block solves (Householder QR).

/// Least-squares solve `min ‖A p − b‖` for a column-major `A` with leading
/// dimension `ld` using its first `m` rows and `n` columns. `A` and `b` are
/// overwritten. Returns the number of numerically deficient pivots.
pub fn qr_solve(
    a: &mut [f64],
    ld: usize,
    m: usize,
    n: usize,
    b: &mut [f64],
    p: &mut [f64],
) -> usize {
    let mut deficient = 0;
    let mut maxdiag = 0.0f64;
    for k in 0..n.min(m) {
        let mut norm2 = 0.0;
        for i in k..m {
            norm2 += a[k * ld + i] * a[k * ld + i];
        }
        let norm = norm2.sqrt();
        if norm == 0.0 {
            continue;
        }
        let akk = a[k * ld + k];
        let alpha = if akk > 0.0 { -norm } else { norm };
        let vk = akk - alpha;
        let vtv = vk * vk + (norm2 - akk * akk);
        if vtv > 0.0 {
            let f = 2.0 / vtv;
            for j in (k + 1)..n {
                let mut s = vk * a[j * ld + k];
                for i in (k + 1)..m {
                    s += a[k * ld + i] * a[j * ld + i];
                }
                s *= f;
                a[j * ld + k] -= s * vk;
                for i in (k + 1)..m {
                    a[j * ld + i] -= s * a[k * ld + i];
                }
            }
            let mut s = vk * b[k];
            for i in (k + 1)..m {
                s += a[k * ld + i] * b[i];
            }
            s *= f;
            b[k] -= s * vk;
            for i in (k + 1)..m {
                b[i] -= s * a[k * ld + i];
            }
        }
        a[k * ld + k] = alpha;
        maxdiag = maxdiag.max(alpha.abs());
    }
    // back substitution on R (upper triangle of the first n rows)
    for k in (0..n).rev() {
        if k >= m {
            p[k] = 0.0;
            continue;
        }
        let rkk = a[k * ld + k];
        if rkk.abs() <= 1e-13 * maxdiag.max(1e-300) {
            deficient += 1;
            p[k] = 0.0;
            continue;
        }
        let mut s = b[k];
        for j in (k + 1)..n {
            s -= a[j * ld + k] * p[j];
        }
        p[k] = s / rkk;
    }
    deficient
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_overdetermined_system() {
        // A = [[1,0],[0,1],[1,1]], b = [1,2,3.1] → normal-equation solution
        let (m, n) = (3, 2);
        let mut a = vec![1.0, 0.0, 1.0, 0.0, 1.0, 1.0]; // column-major
        let mut b = vec![1.0, 2.0, 3.1];
        let mut p = vec![0.0; n];
        let d = qr_solve(&mut a, m, m, n, &mut b, &mut p);
        assert_eq!(d, 0);
        // (AᵀA)p = Aᵀb → [[2,1],[1,2]]p = [4.1, 5.1]
        let det = 3.0;
        let px = (2.0 * 4.1 - 5.1) / det;
        let py = (2.0 * 5.1 - 4.1) / det;
        assert!((p[0] - px).abs() < 1e-12 && (p[1] - py).abs() < 1e-12);
    }

    #[test]
    fn reports_rank_deficiency() {
        let (m, n) = (2, 2);
        let mut a = vec![1.0, 2.0, 2.0, 4.0]; // columns proportional
        let mut b = vec![1.0, 2.0];
        let mut p = vec![0.0; n];
        assert_eq!(qr_solve(&mut a, m, m, n, &mut b, &mut p), 1);
    }
}
