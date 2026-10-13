use rayon::prelude::*;

/// Compressed sparse row matrix (square, f64).
#[derive(Debug, Clone)]
pub struct Csr {
    pub n: usize,
    pub row_ptr: Vec<usize>,
    pub col: Vec<u32>,
    pub val: Vec<f64>,
}

impl Csr {
    pub fn nnz(&self) -> usize {
        self.val.len()
    }

    /// `y = A x`, rows in parallel.
    pub fn spmv(&self, x: &[f64], y: &mut [f64]) {
        debug_assert_eq!(x.len(), self.n);
        y.par_iter_mut().enumerate().for_each(|(i, yi)| {
            let (s, e) = (self.row_ptr[i], self.row_ptr[i + 1]);
            let mut acc = 0.0;
            for k in s..e {
                acc += self.val[k] * x[self.col[k] as usize];
            }
            *yi = acc;
        });
    }

    pub fn diagonal(&self) -> Vec<f64> {
        (0..self.n)
            .into_par_iter()
            .map(|i| {
                let (s, e) = (self.row_ptr[i], self.row_ptr[i + 1]);
                match self.col[s..e].binary_search(&(i as u32)) {
                    Ok(k) => self.val[s + k],
                    Err(_) => 0.0,
                }
            })
            .collect()
    }

    /// Add `v` to the stored entry (i, j). The entry must be in the pattern.
    pub fn add(&mut self, i: usize, j: usize, v: f64) {
        let (s, e) = (self.row_ptr[i], self.row_ptr[i + 1]);
        let k = self.col[s..e]
            .binary_search(&(j as u32))
            .expect("entry is in the pattern");
        self.val[s + k] += v;
    }

    /// Entry (i, j), 0 if not stored.
    pub fn get(&self, i: usize, j: usize) -> f64 {
        let (s, e) = (self.row_ptr[i], self.row_ptr[i + 1]);
        match self.col[s..e].binary_search(&(j as u32)) {
            Ok(k) => self.val[s + k],
            Err(_) => 0.0,
        }
    }

    /// Impose Dirichlet conditions symmetrically: fixed rows and columns become the identity, and the
    /// right-hand side is corrected for the prescribed values.
    pub fn apply_dirichlet(&mut self, b: &mut [f64], fixed: &[(usize, f64)]) {
        let mut is_fixed = vec![false; self.n];
        let mut value = vec![0.0; self.n];
        for (d, v) in fixed {
            is_fixed[*d] = true;
            value[*d] = *v;
        }
        for i in 0..self.n {
            let (s, e) = (self.row_ptr[i], self.row_ptr[i + 1]);
            if is_fixed[i] {
                for k in s..e {
                    self.val[k] = if self.col[k] as usize == i { 1.0 } else { 0.0 };
                }
                b[i] = value[i];
            } else {
                for k in s..e {
                    let c = self.col[k] as usize;
                    if is_fixed[c] {
                        b[i] -= self.val[k] * value[c];
                        self.val[k] = 0.0;
                    }
                }
            }
        }
    }
}

pub fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.par_iter().zip(b.par_iter()).map(|(x, y)| x * y).sum()
}

pub fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}
