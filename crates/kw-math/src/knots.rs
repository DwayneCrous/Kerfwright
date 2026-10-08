/// Non-decreasing knot vector with degree `p`. Number of control points is
/// `knots.len() - p - 1`.
#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector {
    knots: Vec<f64>,
    degree: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum KnotError {
    TooShort { len: usize, degree: usize },
    NotNonDecreasing { index: usize },
    NonFinite { index: usize },
    TooFewControlPoints { num_ctrl: usize, degree: usize },
}

impl KnotVector {
    pub fn new(degree: usize, knots: Vec<f64>) -> Result<Self, KnotError> {
        if knots.len() < 2 * (degree + 1) {
            return Err(KnotError::TooShort { len: knots.len(), degree });
        }
        for (i, k) in knots.iter().enumerate() {
            if !k.is_finite() {
                return Err(KnotError::NonFinite { index: i });
            }
            if i > 0 && *k < knots[i - 1] {
                return Err(KnotError::NotNonDecreasing { index: i });
            }
        }
        Ok(Self { knots, degree })
    }

    /// Clamped knot vector with uniformly spaced interior knots on [0, 1].
    pub fn clamped_uniform(degree: usize, num_ctrl: usize) -> Result<Self, KnotError> {
        if num_ctrl < degree + 1 {
            return Err(KnotError::TooFewControlPoints { num_ctrl, degree });
        }
        let interior = num_ctrl - degree - 1;
        let mut k = Vec::with_capacity(num_ctrl + degree + 1);
        k.extend(std::iter::repeat(0.0).take(degree + 1));
        for j in 1..=interior {
            k.push(j as f64 / (interior + 1) as f64);
        }
        k.extend(std::iter::repeat(1.0).take(degree + 1));
        Self::new(degree, k)
    }

    pub fn degree(&self) -> usize {
        self.degree
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.knots
    }

    pub fn num_ctrl(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    /// Valid parameter interval `[U[p], U[n+1]]`.
    pub fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.num_ctrl()])
    }

    /// Knot span index `i` with `U[i] <= u < U[i+1]` (Piegl & Tiller A2.1).
    /// `u` at or beyond the end of the domain maps to the last non-empty span.
    pub fn find_span(&self, u: f64) -> usize {
        let p = self.degree;
        let n = self.num_ctrl() - 1;
        let k = &self.knots;
        if u >= k[n + 1] {
            return n;
        }
        if u <= k[p] {
            return p;
        }
        let (mut lo, mut hi) = (p, n + 1);
        let mut mid = (lo + hi) / 2;
        while u < k[mid] || u >= k[mid + 1] {
            if u < k[mid] {
                hi = mid;
            } else {
                lo = mid;
            }
            mid = (lo + hi) / 2;
        }
        mid
    }

    /// Non-zero basis functions `N[span-p ..= span]` at `u` (Piegl & Tiller A2.2).
    pub fn basis_funs(&self, span: usize, u: f64) -> Vec<f64> {
        let p = self.degree;
        let k = &self.knots;
        let mut n = vec![0.0; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        n[0] = 1.0;
        for j in 1..=p {
            left[j] = u - k[span + 1 - j];
            right[j] = k[span + j] - u;
            let mut saved = 0.0;
            for r in 0..j {
                let temp = n[r] / (right[r + 1] + left[j - r]);
                n[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n[j] = saved;
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partition_of_unity() {
        for degree in 1..=5usize {
            for num_ctrl in (degree + 1)..(degree + 9) {
                let kv = KnotVector::clamped_uniform(degree, num_ctrl).unwrap();
                let (a, b) = kv.domain();
                for s in 0..=100 {
                    let u = a + (b - a) * (s as f64) / 100.0;
                    let span = kv.find_span(u);
                    let n = kv.basis_funs(span, u);
                    let sum: f64 = n.iter().sum();
                    assert!((sum - 1.0).abs() < 1e-12, "p={degree} n={num_ctrl} u={u}");
                    assert!(n.iter().all(|x| *x >= -1e-12));
                }
            }
        }
    }

    #[test]
    fn rejects_decreasing() {
        let r = KnotVector::new(1, vec![0.0, 0.0, 1.0, 0.5]);
        assert!(matches!(r, Err(KnotError::NotNonDecreasing { .. })));
    }
}
