//! Knot vectors and B-spline basis functions (Piegl & Tiller, ch. 2).

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
    NotClamped,
}

impl KnotVector {
    pub fn new(degree: usize, knots: Vec<f64>) -> Result<Self, KnotError> {
        if degree == 0 || knots.len() < 2 * (degree + 1) {
            return Err(KnotError::TooShort {
                len: knots.len(),
                degree,
            });
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

    /// Like [`new`](Self::new) but also requires end knots of multiplicity `p + 1`.
    pub fn new_clamped(degree: usize, knots: Vec<f64>) -> Result<Self, KnotError> {
        let kv = Self::new(degree, knots)?;
        let k = &kv.knots;
        let n = k.len();
        let ok = (0..=degree).all(|i| k[i] == k[0] && k[n - 1 - i] == k[n - 1]) && k[0] < k[n - 1];
        if ok {
            Ok(kv)
        } else {
            Err(KnotError::NotClamped)
        }
    }

    /// Clamped knot vector with uniformly spaced interior knots on [0, 1].
    pub fn clamped_uniform(degree: usize, num_ctrl: usize) -> Result<Self, KnotError> {
        if num_ctrl < degree + 1 {
            return Err(KnotError::TooFewControlPoints { num_ctrl, degree });
        }
        let interior = num_ctrl - degree - 1;
        let mut k = Vec::with_capacity(num_ctrl + degree + 1);
        k.extend(std::iter::repeat_n(0.0, degree + 1));
        for j in 1..=interior {
            k.push(j as f64 / (interior + 1) as f64);
        }
        k.extend(std::iter::repeat_n(1.0, degree + 1));
        Self::new(degree, k)
    }

    pub fn degree(&self) -> usize {
        self.degree
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.knots
    }

    pub fn len(&self) -> usize {
        self.knots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.knots.is_empty()
    }

    pub fn num_ctrl(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    /// Valid parameter interval `[U[p], U[n+1]]`.
    pub fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.num_ctrl()])
    }

    /// Number of knots exactly equal to `u`.
    pub fn multiplicity(&self, u: f64) -> usize {
        self.knots.iter().filter(|k| **k == u).count()
    }

    /// Distinct knot values with multiplicities, in order.
    pub fn distinct(&self) -> Vec<(f64, usize)> {
        let mut out: Vec<(f64, usize)> = Vec::new();
        for &k in &self.knots {
            match out.last_mut() {
                Some((v, m)) if *v == k => *m += 1,
                _ => out.push((k, 1)),
            }
        }
        out
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

    /// Basis functions and their derivatives up to order `n` (A2.3).
    /// `result[k][j]` is the k-th derivative of `N[span-p+j]`; orders above
    /// the degree are identically zero.
    pub fn ders_basis_funs(&self, span: usize, u: f64, n: usize) -> Vec<Vec<f64>> {
        let p = self.degree;
        let k = &self.knots;
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        ndu[0][0] = 1.0;
        for j in 1..=p {
            left[j] = u - k[span + 1 - j];
            right[j] = k[span + j] - u;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }
        let mut ders = vec![vec![0.0; p + 1]; n + 1];
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }
        let nn = n.min(p);
        let mut a = [vec![0.0; p + 1], vec![0.0; p + 1]];
        for r in 0..=p as isize {
            let (mut s1, mut s2) = (0usize, 1usize);
            a[0][0] = 1.0;
            for kk in 1..=nn as isize {
                let mut d = 0.0;
                let rk = r - kk;
                let pk = p as isize - kk;
                if r >= kk {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }
                let j1 = if rk >= -1 { 1 } else { -rk };
                let j2 = if r - 1 <= pk { kk - 1 } else { p as isize - r };
                for j in j1..=j2 {
                    a[s2][j as usize] = (a[s1][j as usize] - a[s1][(j - 1) as usize])
                        / ndu[(pk + 1) as usize][(rk + j) as usize];
                    d += a[s2][j as usize] * ndu[(rk + j) as usize][pk as usize];
                }
                if r <= pk {
                    a[s2][kk as usize] =
                        -a[s1][(kk - 1) as usize] / ndu[(pk + 1) as usize][r as usize];
                    d += a[s2][kk as usize] * ndu[r as usize][pk as usize];
                }
                ders[kk as usize][r as usize] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }
        let mut fac = p as f64;
        for kk in 1..=nn {
            for j in 0..=p {
                ders[kk][j] *= fac;
            }
            fac *= (p - kk) as f64;
        }
        ders
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

    /// Piegl & Tiller, Example 2.3: U = {0,0,0,1,2,3,4,4,5,5,5}, p = 2, u = 5/2
    /// lies in span 4 and N_{2,2}, N_{3,2}, N_{4,2} = 1/8, 3/4, 1/8.
    #[test]
    fn piegl_tiller_example_2_3() {
        let kv = KnotVector::new(2, vec![0., 0., 0., 1., 2., 3., 4., 4., 5., 5., 5.]).unwrap();
        let span = kv.find_span(2.5);
        assert_eq!(span, 4);
        let n = kv.basis_funs(span, 2.5);
        assert!((n[0] - 0.125).abs() < 1e-15);
        assert!((n[1] - 0.75).abs() < 1e-15);
        assert!((n[2] - 0.125).abs() < 1e-15);
    }

    /// Same example, derivatives: N'_{2,2}=-1/2, N'_{3,2}=0, N'_{4,2}=1/2 and
    /// N''_{2,2}=1, N''_{3,2}=-2, N''_{4,2}=1 (computed by hand from Eq. 2.7).
    #[test]
    fn derivatives_of_example_2_3() {
        let kv = KnotVector::new(2, vec![0., 0., 0., 1., 2., 3., 4., 4., 5., 5., 5.]).unwrap();
        let d = kv.ders_basis_funs(4, 2.5, 3);
        let exp1 = [-0.5, 0.0, 0.5];
        let exp2 = [1.0, -2.0, 1.0];
        for j in 0..3 {
            assert!((d[1][j] - exp1[j]).abs() < 1e-14, "d1[{j}]={}", d[1][j]);
            assert!((d[2][j] - exp2[j]).abs() < 1e-14, "d2[{j}]={}", d[2][j]);
            assert_eq!(d[3][j], 0.0);
        }
    }

    #[test]
    fn multiplicity_and_distinct() {
        let kv = KnotVector::new(2, vec![0., 0., 0., 1., 1., 2., 2., 2.]).unwrap();
        assert_eq!(kv.multiplicity(1.0), 2);
        assert_eq!(kv.distinct(), vec![(0.0, 3), (1.0, 2), (2.0, 3)]);
    }
}
