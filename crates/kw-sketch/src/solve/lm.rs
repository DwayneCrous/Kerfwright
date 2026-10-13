//! Nonlinear least-squares drivers on small dense problems: damped
//! Gauss–Newton / Levenberg–Marquardt (primary), Powell dog-leg and BFGS
//! (fallbacks).

use super::dense::qr_solve;

/// A nonlinear least-squares problem `min ½‖r(x)‖²` with dimensionless
/// (weighted) residuals.
pub trait Problem {
    fn m(&self) -> usize;
    fn n(&self) -> usize;
    fn eval(&mut self, x: &[f64], r: &mut [f64]);
    /// Whether the point most recently passed to `eval` is a valid
    /// configuration (no degenerate geometry). Steps to invalid points are
    /// rejected like steps that increase the residual.
    fn feasible(&mut self) -> bool {
        true
    }
    /// Row-major `m × n` Jacobian.
    fn jac(&mut self, x: &[f64], j: &mut [f64]);
}

#[derive(Clone, Debug)]
pub struct LmOpts {
    /// Converged when `max|r| ≤ tol`.
    pub tol: f64,
    pub max_iter: usize,
    /// Largest allowed `max|Δx|` per iteration.
    pub max_step: f64,
    pub lambda0: f64,
    /// Damp with the identity instead of Marquardt column scaling, so
    /// rank-deficient steps are the minimum *Euclidean* norm change (the least
    /// movement of the geometry) rather than one skewed toward insensitive variables.
    pub unit_scaling: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LmResult {
    pub converged: bool,
    pub iters: usize,
    pub rinf: f64,
}

/// Reusable buffers.
#[derive(Default)]
pub struct LmWs {
    jac: Vec<f64>,
    r: Vec<f64>,
    rt: Vec<f64>,
    xt: Vec<f64>,
    p: Vec<f64>,
    d: Vec<f64>,
    a: Vec<f64>,
    b: Vec<f64>,
    g: Vec<f64>,
    aux: Vec<f64>,
}

impl LmWs {
    fn ensure(&mut self, m: usize, n: usize) {
        let grow = |v: &mut Vec<f64>, len: usize| {
            if v.len() < len {
                v.resize(len, 0.0);
            }
        };
        grow(&mut self.jac, m * n);
        grow(&mut self.r, m);
        grow(&mut self.rt, m);
        grow(&mut self.xt, n);
        grow(&mut self.p, n);
        grow(&mut self.d, n);
        grow(&mut self.g, n);
        grow(&mut self.aux, n.max(m));
        grow(&mut self.a, (m + n) * n);
        grow(&mut self.b, m + n);
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn inf_norm(a: &[f64]) -> f64 {
    a.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

/// Solve `(JᵀJ + λ D²) p = −Jᵀ r` through Householder QR of `[J; √λ D]`.
fn damped_step(ws: &mut LmWs, m: usize, n: usize, lambda: f64) {
    let ld = m + n;
    let sl = lambda.sqrt();
    for j in 0..n {
        let col = &mut ws.a[j * ld..j * ld + ld];
        for i in 0..m {
            col[i] = ws.jac[i * n + j];
        }
        for i in 0..n {
            col[m + i] = 0.0;
        }
        col[m + j] = sl * ws.d[j];
    }
    for i in 0..m {
        ws.b[i] = -ws.r[i];
    }
    for i in 0..n {
        ws.b[m + i] = 0.0;
    }
    let (a, b, p) = (&mut ws.a, &mut ws.b, &mut ws.p);
    qr_solve(a, ld, m + n, n, b, p);
}

/// Levenberg–Marquardt with Marquardt column scaling and a step limit.
pub fn lm<P: Problem>(p: &mut P, x: &mut [f64], o: &LmOpts, ws: &mut LmWs) -> LmResult {
    let (m, n) = (p.m(), p.n());
    ws.ensure(m, n);
    p.eval(x, &mut ws.r[..m]);
    let mut f = dot(&ws.r[..m], &ws.r[..m]);
    let mut rinf = inf_norm(&ws.r[..m]);
    if rinf <= o.tol {
        return LmResult {
            converged: true,
            iters: 0,
            rinf,
        };
    }
    if n == 0 {
        return LmResult {
            converged: false,
            iters: 0,
            rinf,
        };
    }
    for j in 0..n {
        ws.d[j] = 0.0;
    }
    let mut lambda = o.lambda0;
    let mut iters = 0;
    while iters < o.max_iter {
        iters += 1;
        p.jac(x, &mut ws.jac[..m * n]);
        for j in 0..n {
            let mut s = 0.0;
            for i in 0..m {
                let v = ws.jac[i * n + j];
                s += v * v;
            }
            ws.d[j] = if o.unit_scaling {
                1.0
            } else {
                ws.d[j].max(s.sqrt()).max(1e-8)
            };
        }
        let mut accepted = false;
        let mut pinf = 0.0;
        for _ in 0..16 {
            damped_step(ws, m, n, lambda);
            pinf = inf_norm(&ws.p[..n]);
            let k = if pinf > o.max_step {
                o.max_step / pinf
            } else {
                1.0
            };
            for i in 0..n {
                ws.xt[i] = x[i] + k * ws.p[i];
            }
            p.eval(&ws.xt[..n], &mut ws.rt[..m]);
            let ft = dot(&ws.rt[..m], &ws.rt[..m]);
            if ft.is_finite() && ft < f && p.feasible() {
                x[..n].copy_from_slice(&ws.xt[..n]);
                ws.r[..m].copy_from_slice(&ws.rt[..m]);
                f = ft;
                lambda = (lambda * 0.1).max(1e-14);
                accepted = true;
                pinf *= k;
                break;
            }
            lambda = (lambda * 10.0).max(1e-8);
        }
        if !accepted {
            break;
        }
        rinf = inf_norm(&ws.r[..m]);
        if rinf <= o.tol {
            return LmResult {
                converged: true,
                iters,
                rinf,
            };
        }
        let xinf = inf_norm(&x[..n]);
        if pinf <= 1e-15 * (1.0 + xinf) {
            break;
        }
    }
    LmResult {
        converged: false,
        iters,
        rinf,
    }
}

/// Powell's dog-leg trust-region method.
pub fn dogleg<P: Problem>(p: &mut P, x: &mut [f64], o: &LmOpts, ws: &mut LmWs) -> LmResult {
    let (m, n) = (p.m(), p.n());
    ws.ensure(m, n);
    p.eval(x, &mut ws.r[..m]);
    let mut f = 0.5 * dot(&ws.r[..m], &ws.r[..m]);
    let mut rinf = inf_norm(&ws.r[..m]);
    if rinf <= o.tol {
        return LmResult {
            converged: true,
            iters: 0,
            rinf,
        };
    }
    if n == 0 {
        return LmResult {
            converged: false,
            iters: 0,
            rinf,
        };
    }
    let mut delta = o.max_step.clamp(1e-3, 1.0);
    let mut iters = 0;
    while iters < o.max_iter * 2 {
        iters += 1;
        p.jac(x, &mut ws.jac[..m * n]);
        // gradient g = Jᵀ r
        for j in 0..n {
            let mut s = 0.0;
            for i in 0..m {
                s += ws.jac[i * n + j] * ws.r[i];
            }
            ws.g[j] = s;
        }
        for j in 0..n {
            ws.d[j] = 1.0;
        }
        damped_step(ws, m, n, 1e-18); // Gauss–Newton step in ws.p
        let gn: Vec<f64> = ws.p[..n].to_vec();
        let gn_norm = dot(&gn, &gn).sqrt();
        let g_norm2 = dot(&ws.g[..n], &ws.g[..n]);
        // Cauchy step along −g
        let mut jg2 = 0.0;
        for i in 0..m {
            let mut s = 0.0;
            for j in 0..n {
                s += ws.jac[i * n + j] * ws.g[j];
            }
            jg2 += s * s;
        }
        let alpha = if jg2 > 0.0 { g_norm2 / jg2 } else { 0.0 };
        let cauchy: Vec<f64> = ws.g[..n].iter().map(|g| -alpha * g).collect();
        let c_norm = dot(&cauchy, &cauchy).sqrt();
        let step: Vec<f64> = if gn_norm <= delta {
            gn.clone()
        } else if c_norm >= delta {
            let k = delta / g_norm2.sqrt().max(1e-300);
            ws.g[..n].iter().map(|g| -k * g).collect()
        } else {
            // p = c + τ (gn − c), |p| = Δ
            let dlt: Vec<f64> = gn.iter().zip(&cauchy).map(|(a, b)| a - b).collect();
            let (a, b, c) = (
                dot(&dlt, &dlt),
                2.0 * dot(&cauchy, &dlt),
                dot(&cauchy, &cauchy) - delta * delta,
            );
            let tau = (-b + (b * b - 4.0 * a * c).max(0.0).sqrt()) / (2.0 * a);
            cauchy
                .iter()
                .zip(&dlt)
                .map(|(c0, d)| c0 + tau * d)
                .collect()
        };
        for i in 0..n {
            ws.xt[i] = x[i] + step[i];
        }
        p.eval(&ws.xt[..n], &mut ws.rt[..m]);
        let mut ft = 0.5 * dot(&ws.rt[..m], &ws.rt[..m]);
        if !p.feasible() {
            ft = f64::INFINITY;
        }
        // predicted reduction ½(‖r‖² − ‖r + J s‖²)
        let mut pred = 0.0;
        for i in 0..m {
            let mut js = 0.0;
            for j in 0..n {
                js += ws.jac[i * n + j] * step[j];
            }
            let v = ws.r[i] + js;
            pred += ws.r[i] * ws.r[i] - v * v;
        }
        pred *= 0.5;
        let rho = if pred > 0.0 { (f - ft) / pred } else { -1.0 };
        if ft.is_finite() && rho > 1e-4 {
            x[..n].copy_from_slice(&ws.xt[..n]);
            ws.r[..m].copy_from_slice(&ws.rt[..m]);
            f = ft;
            rinf = inf_norm(&ws.r[..m]);
            if rinf <= o.tol {
                return LmResult {
                    converged: true,
                    iters,
                    rinf,
                };
            }
        }
        if rho < 0.25 {
            delta *= 0.25;
        } else if rho > 0.75 && (dot(&step, &step).sqrt() >= 0.99 * delta) {
            delta = (delta * 2.0).min(1e6);
        }
        if delta < 1e-15 {
            break;
        }
    }
    LmResult {
        converged: false,
        iters,
        rinf,
    }
}

/// BFGS on `½‖r‖²` with Armijo backtracking (last resort).
pub fn bfgs<P: Problem>(p: &mut P, x: &mut [f64], o: &LmOpts, ws: &mut LmWs) -> LmResult {
    let (m, n) = (p.m(), p.n());
    ws.ensure(m, n);
    p.eval(x, &mut ws.r[..m]);
    let mut f = 0.5 * dot(&ws.r[..m], &ws.r[..m]);
    let mut rinf = inf_norm(&ws.r[..m]);
    if rinf <= o.tol || n == 0 {
        return LmResult {
            converged: rinf <= o.tol,
            iters: 0,
            rinf,
        };
    }
    let grad = |p: &mut P, x: &[f64], ws: &mut LmWs, out: &mut [f64]| {
        p.jac(x, &mut ws.jac[..m * n]);
        for j in 0..n {
            let mut s = 0.0;
            for i in 0..m {
                s += ws.jac[i * n + j] * ws.r[i];
            }
            out[j] = s;
        }
    };
    let mut g = vec![0.0; n];
    grad(p, x, ws, &mut g);
    let mut h = vec![0.0; n * n]; // inverse Hessian approximation
    for i in 0..n {
        h[i * n + i] = 1.0;
    }
    let mut iters = 0;
    let max_iter = o.max_iter * 20;
    while iters < max_iter {
        iters += 1;
        // direction d = −H g
        let mut d = vec![0.0; n];
        for i in 0..n {
            d[i] = -(0..n).map(|j| h[i * n + j] * g[j]).sum::<f64>();
        }
        let mut slope = dot(&g, &d);
        if slope >= 0.0 {
            // reset to steepest descent
            for i in 0..n {
                for j in 0..n {
                    h[i * n + j] = if i == j { 1.0 } else { 0.0 };
                }
                d[i] = -g[i];
            }
            slope = -dot(&g, &g);
        }
        let dinf = inf_norm(&d);
        let mut t = if dinf > o.max_step {
            o.max_step / dinf
        } else {
            1.0
        };
        let mut ok = false;
        for _ in 0..40 {
            for i in 0..n {
                ws.xt[i] = x[i] + t * d[i];
            }
            p.eval(&ws.xt[..n], &mut ws.rt[..m]);
            let ft = 0.5 * dot(&ws.rt[..m], &ws.rt[..m]);
            if ft.is_finite() && ft <= f + 1e-4 * t * slope && p.feasible() {
                ok = true;
                break;
            }
            t *= 0.5;
        }
        if !ok {
            break;
        }
        let s: Vec<f64> = (0..n).map(|i| ws.xt[i] - x[i]).collect();
        x[..n].copy_from_slice(&ws.xt[..n]);
        ws.r[..m].copy_from_slice(&ws.rt[..m]);
        f = 0.5 * dot(&ws.r[..m], &ws.r[..m]);
        rinf = inf_norm(&ws.r[..m]);
        if rinf <= o.tol {
            return LmResult {
                converged: true,
                iters,
                rinf,
            };
        }
        let mut gn = vec![0.0; n];
        grad(p, x, ws, &mut gn);
        let y: Vec<f64> = (0..n).map(|i| gn[i] - g[i]).collect();
        let sy = dot(&s, &y);
        if sy > 1e-14 {
            // H ← (I − ρ s yᵀ) H (I − ρ y sᵀ) + ρ s sᵀ
            let rho = 1.0 / sy;
            let mut hy = vec![0.0; n];
            for i in 0..n {
                hy[i] = (0..n).map(|j| h[i * n + j] * y[j]).sum();
            }
            let yhy = dot(&y, &hy);
            for i in 0..n {
                for j in 0..n {
                    h[i * n + j] +=
                        (1.0 + rho * yhy) * rho * s[i] * s[j] - rho * (hy[i] * s[j] + s[i] * hy[j]);
                }
            }
        }
        g = gn;
        if inf_norm(&g) < 1e-300 {
            break;
        }
    }
    LmResult {
        converged: false,
        iters,
        rinf,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Intersection of a circle and a line, plus an extra consistent equation.
    struct Toy;
    impl Problem for Toy {
        fn m(&self) -> usize {
            3
        }
        fn n(&self) -> usize {
            2
        }
        fn eval(&mut self, x: &[f64], r: &mut [f64]) {
            r[0] = x[0] * x[0] + x[1] * x[1] - 25.0;
            r[1] = x[0] - x[1] - 1.0;
            r[2] = 3.0 * x[0] - 3.0 * x[1] - 3.0; // redundant with r[1]
        }
        fn jac(&mut self, x: &[f64], j: &mut [f64]) {
            j.copy_from_slice(&[2.0 * x[0], 2.0 * x[1], 1.0, -1.0, 3.0, -3.0]);
        }
    }

    fn opts() -> LmOpts {
        LmOpts {
            tol: 1e-12,
            max_iter: 100,
            max_step: 10.0,
            lambda0: 1e-12,
            unit_scaling: false,
        }
    }

    #[test]
    fn all_three_solvers_solve_a_redundant_system() {
        for which in 0..3 {
            let mut x = [3.0, 1.0];
            let mut ws = LmWs::default();
            let r = match which {
                0 => lm(&mut Toy, &mut x, &opts(), &mut ws),
                1 => dogleg(&mut Toy, &mut x, &opts(), &mut ws),
                _ => bfgs(&mut Toy, &mut x, &opts(), &mut ws),
            };
            assert!(r.converged, "solver {which}: {r:?}");
            assert!(
                (x[0] - 4.0).abs() < 1e-8 && (x[1] - 3.0).abs() < 1e-8,
                "solver {which}: {x:?}"
            );
        }
    }
}
