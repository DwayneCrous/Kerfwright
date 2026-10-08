//! Minimal forward-mode automatic differentiation.
//!
//! Residuals are written once, generically over [`Scalar`]; evaluating them with
//! [`Dual`] numbers gives an independent derivative that the tests compare
//! against the hand-coded analytic Jacobians.

use std::ops::{Add, Div, Mul, Neg, Sub};

/// Number of derivative slots carried by a [`Dual`].
pub const NDER: usize = 16;

pub trait Scalar:
    Copy
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
{
    fn c(x: f64) -> Self;
    fn sqrt(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn val(self) -> f64;
}

impl Scalar for f64 {
    #[inline(always)]
    fn c(x: f64) -> Self {
        x
    }
    #[inline(always)]
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
    #[inline(always)]
    fn sin(self) -> Self {
        f64::sin(self)
    }
    #[inline(always)]
    fn cos(self) -> Self {
        f64::cos(self)
    }
    #[inline(always)]
    fn val(self) -> f64 {
        self
    }
}

/// Value plus gradient with respect to up to [`NDER`] independent variables.
#[derive(Clone, Copy, Debug)]
pub struct Dual {
    pub v: f64,
    pub d: [f64; NDER],
}

impl Dual {
    pub fn variable(v: f64, index: usize) -> Self {
        let mut d = [0.0; NDER];
        d[index] = 1.0;
        Self { v, d }
    }
}

impl Add for Dual {
    type Output = Dual;
    fn add(self, o: Dual) -> Dual {
        let mut d = self.d;
        for (a, b) in d.iter_mut().zip(o.d.iter()) {
            *a += b;
        }
        Dual { v: self.v + o.v, d }
    }
}

impl Sub for Dual {
    type Output = Dual;
    fn sub(self, o: Dual) -> Dual {
        let mut d = self.d;
        for (a, b) in d.iter_mut().zip(o.d.iter()) {
            *a -= b;
        }
        Dual { v: self.v - o.v, d }
    }
}

impl Mul for Dual {
    type Output = Dual;
    fn mul(self, o: Dual) -> Dual {
        let mut d = [0.0; NDER];
        for i in 0..NDER {
            d[i] = self.d[i] * o.v + self.v * o.d[i];
        }
        Dual { v: self.v * o.v, d }
    }
}

impl Div for Dual {
    type Output = Dual;
    fn div(self, o: Dual) -> Dual {
        let inv = 1.0 / o.v;
        let q = self.v * inv;
        let mut d = [0.0; NDER];
        for i in 0..NDER {
            d[i] = (self.d[i] - q * o.d[i]) * inv;
        }
        Dual { v: q, d }
    }
}

impl Neg for Dual {
    type Output = Dual;
    fn neg(self) -> Dual {
        let mut d = self.d;
        for a in d.iter_mut() {
            *a = -*a;
        }
        Dual { v: -self.v, d }
    }
}

impl Scalar for Dual {
    fn c(x: f64) -> Self {
        Dual {
            v: x,
            d: [0.0; NDER],
        }
    }
    fn sqrt(self) -> Self {
        let s = self.v.sqrt();
        let k = 0.5 / s;
        let mut d = self.d;
        for a in d.iter_mut() {
            *a *= k;
        }
        Dual { v: s, d }
    }
    fn sin(self) -> Self {
        let (s, c) = self.v.sin_cos();
        let mut d = self.d;
        for a in d.iter_mut() {
            *a *= c;
        }
        Dual { v: s, d }
    }
    fn cos(self) -> Self {
        let (s, c) = self.v.sin_cos();
        let mut d = self.d;
        for a in d.iter_mut() {
            *a *= -s;
        }
        Dual { v: c, d }
    }
    fn val(self) -> f64 {
        self.v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dual_matches_calculus() {
        let x = Dual::variable(0.7, 0);
        let y = Dual::variable(-1.3, 1);
        let f = (x * y + x.sin()) / (x * x + y * y).sqrt();
        // numerical check
        let g = |x: f64, y: f64| (x * y + x.sin()) / (x * x + y * y).sqrt();
        let h = 1e-6;
        let dx = (g(0.7 + h, -1.3) - g(0.7 - h, -1.3)) / (2.0 * h);
        let dy = (g(0.7, -1.3 + h) - g(0.7, -1.3 - h)) / (2.0 * h);
        assert!((f.v - g(0.7, -1.3)).abs() < 1e-14);
        assert!((f.d[0] - dx).abs() < 1e-8 && (f.d[1] - dy).abs() < 1e-8);
    }
}
