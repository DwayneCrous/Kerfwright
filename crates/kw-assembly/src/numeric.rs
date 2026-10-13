//! Small dense linear algebra shared by the drag and the loop solver.

/// Solve a small dense system by Gaussian elimination with partial pivoting.
pub(crate) fn solve_small(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let piv = (col..n).max_by(|x, y| a[*x][col].abs().total_cmp(&a[*y][col].abs()))?;
        if a[piv][col].abs() < 1e-300 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..n {
            let f = a[r][col] / a[col][col];
            for c in col..n {
                let v = a[col][c];
                a[r][c] -= f * v;
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = (r + 1..n).map(|c| a[r][c] * x[c]).sum();
        x[r] = (b[r] - s) / a[r][r];
    }
    Some(x)
}

/// The numerical rank of a matrix given as rows, by elimination with full pivoting. Entries below
/// `tol` times the largest are treated as zero.
pub(crate) fn rank(mut m: Vec<Vec<f64>>, tol: f64) -> usize {
    let rows = m.len();
    let cols = m.first().map_or(0, |r| r.len());
    let big = m
        .iter()
        .flat_map(|r| r.iter())
        .fold(0.0f64, |a, x| a.max(x.abs()));
    if big == 0.0 {
        return 0;
    }
    let eps = tol * big;
    let mut used = vec![false; cols];
    let mut rank = 0;
    while rank < rows {
        let mut best = (0.0, 0, 0);
        for r in rank..rows {
            for c in (0..cols).filter(|c| !used[*c]) {
                if m[r][c].abs() > best.0 {
                    best = (m[r][c].abs(), r, c);
                }
            }
        }
        if best.0 <= eps {
            break;
        }
        let (_, pr, pc) = best;
        m.swap(rank, pr);
        used[pc] = true;
        for r in rank + 1..rows {
            let f = m[r][pc] / m[rank][pc];
            for c in 0..cols {
                let v = m[rank][c];
                m[r][c] -= f * v;
            }
        }
        rank += 1;
    }
    rank
}
