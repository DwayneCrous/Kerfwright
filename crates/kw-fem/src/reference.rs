//! Closed-form reference solutions the finite-element results are checked against: the end-loaded
//! cantilever of Timoshenko & Goodier (*Theory of Elasticity*, §21), Kirsch's circular hole in a plate
//! under tension, Lamé's thick-walled cylinder and sphere under internal pressure, and the Euler–Bernoulli
//! beam frequencies. Each has a unit test that verifies the formulas against the field equations, so
//! that the FE benchmarks compare against a checked reference rather than a transcription.

use kw_mesh3d::P3;

/// Plane-strain constants `(E', ν')` of a material `(E, ν)`: the plane-stress formulas with these are the
/// plane-strain solution.
pub fn plane_strain(e: f64, nu: f64) -> (f64, f64) {
    (e / (1.0 - nu * nu), nu / (1.0 - nu))
}

/// End-loaded cantilever: length `l` along x from the clamp at `x = 0`, depth `h` along y (centred),
/// shear force `p` per unit thickness acting in `−y` at `x = l`, parabolic shear distribution.
/// The displacement field satisfies the clamp conditions `u = v = ∂u/∂y = 0` at the centroid of the
/// clamped section, and is the exact elasticity solution (plane strain when built with
/// [`plane_strain`] constants).
#[derive(Debug, Clone, Copy)]
pub struct Cantilever {
    pub l: f64,
    pub h: f64,
    pub p: f64,
    /// Plane-strain modulus and Poisson ratio.
    pub e: f64,
    pub nu: f64,
    /// Shear modulus of the original material.
    pub g: f64,
}

impl Cantilever {
    /// Cantilever of a material `(e, nu)` in plane strain.
    pub fn plane_strain(l: f64, h: f64, p: f64, e: f64, nu: f64) -> Cantilever {
        let (ep, nup) = plane_strain(e, nu);
        Cantilever {
            l,
            h,
            p,
            e: ep,
            nu: nup,
            g: e / (2.0 * (1.0 + nu)),
        }
    }

    fn c(&self) -> f64 {
        0.5 * self.h
    }
    fn i(&self) -> f64 {
        2.0 * self.c().powi(3) / 3.0
    }

    /// Displacements `(u, v)` at `(x, y)`.
    pub fn displacement(&self, x: f64, y: f64) -> (f64, f64) {
        let (l, p, e, nu, g, c, i) = (self.l, self.p, self.e, self.nu, self.g, self.c(), self.i());
        let k3 = p * l * l / (2.0 * e * i) + p * c * c / (2.0 * i * g);
        let k1 = -k3;
        let k2 = p * l.powi(3) / (6.0 * e * i);
        let f = -nu * p * y.powi(3) / (6.0 * e * i) + p * y.powi(3) / (6.0 * i * g)
            - p * c * c * y / (2.0 * i * g)
            + k3 * y;
        let gx = -p * (l - x).powi(3) / (6.0 * e * i) + k1 * x + k2;
        let u = -p * y * (l - x).powi(2) / (2.0 * e * i) + f;
        let v = -nu * p * (l - x) * y * y / (2.0 * e * i) + gx;
        (u, v)
    }

    /// In-plane stresses `(σxx, σyy, τxy)`.
    pub fn stress(&self, x: f64, y: f64) -> (f64, f64, f64) {
        let (l, p, c, i) = (self.l, self.p, self.c(), self.i());
        (p * (l - x) * y / i, 0.0, p * (y * y - c * c) / (2.0 * i))
    }

    /// Deflection of the centroid at the free end: `−P L³/(3E'I) − P c² L/(2 I G)`.
    pub fn tip_deflection(&self) -> f64 {
        self.displacement(self.l, 0.0).1
    }
}

/// Kirsch solution for a circular hole of radius `a` in an infinite plate under uniaxial tension `sigma`
/// along x. Returns `(σxx, σyy, τxy)` at `(x, y)` relative to the hole centre.
pub fn kirsch(a: f64, sigma: f64, x: f64, y: f64) -> (f64, f64, f64) {
    let r2 = x * x + y * y;
    let r = r2.sqrt();
    let th = y.atan2(x);
    let (s, c) = th.sin_cos();
    let a2 = a * a / r2;
    let a4 = a2 * a2;
    let cos2 = (2.0 * th).cos();
    let sin2 = (2.0 * th).sin();
    let _ = r;
    let sr = 0.5 * sigma * (1.0 - a2) + 0.5 * sigma * (1.0 - 4.0 * a2 + 3.0 * a4) * cos2;
    let st = 0.5 * sigma * (1.0 + a2) - 0.5 * sigma * (1.0 + 3.0 * a4) * cos2;
    let trt = -0.5 * sigma * (1.0 + 2.0 * a2 - 3.0 * a4) * sin2;
    let sxx = sr * c * c + st * s * s - 2.0 * trt * s * c;
    let syy = sr * s * s + st * c * c + 2.0 * trt * s * c;
    let txy = (sr - st) * s * c + trt * (c * c - s * s);
    (sxx, syy, txy)
}

/// Lamé thick-walled cylinder (inner radius `a`, outer `b`) under internal pressure `p`, plane strain
/// (`ε_z = 0`). Returns `(σr, σθ, σz, u_r)` at radius `r`.
pub fn lame_cylinder(a: f64, b: f64, p: f64, e: f64, nu: f64, r: f64) -> (f64, f64, f64, f64) {
    let k = p * a * a / (b * b - a * a);
    let sr = k * (1.0 - b * b / (r * r));
    let st = k * (1.0 + b * b / (r * r));
    let sz = nu * (sr + st);
    let ur = (1.0 + nu) / e * k * ((1.0 - 2.0 * nu) * r + b * b / r);
    (sr, st, sz, ur)
}

/// Lamé thick-walled sphere under internal pressure. Returns `(σr, σθ, u_r)`.
pub fn lame_sphere(a: f64, b: f64, p: f64, e: f64, nu: f64, r: f64) -> (f64, f64, f64) {
    let (a3, b3, r3) = (a.powi(3), b.powi(3), r.powi(3));
    let k = p * a3 / (b3 - a3);
    let sr = k * (1.0 - b3 / r3);
    let st = k * (1.0 + b3 / (2.0 * r3));
    let ur = k * r / e * ((1.0 - 2.0 * nu) + (1.0 + nu) * b3 / (2.0 * r3));
    (sr, st, ur)
}

/// Roots `β_n L` of the clamped–free beam equation `cos βL cosh βL = −1`.
pub const CANTILEVER_BETA_L: [f64; 5] = [
    1.875_104_07,
    4.694_091_13,
    7.854_757_44,
    10.995_540_73,
    14.137_168_39,
];

/// Euler–Bernoulli natural frequencies (Hz) of a uniform clamped–free beam: mode `n` (0-based).
pub fn cantilever_frequency(n: usize, l: f64, e: f64, i: f64, rho: f64, area: f64) -> f64 {
    let bl = CANTILEVER_BETA_L[n];
    bl * bl / (2.0 * std::f64::consts::PI) * (e * i / (rho * area * l.powi(4))).sqrt()
}

pub fn dist(a: P3, b: P3) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(f: impl Fn(f64) -> f64, x: f64) -> f64 {
        let h = 1e-5 * (1.0 + x.abs());
        (f(x + h) - f(x - h)) / (2.0 * h)
    }

    #[test]
    fn cantilever_satisfies_the_field_equations() {
        let c = Cantilever::plane_strain(10.0, 2.0, 3.0, 210e3, 0.3);
        for (x, y) in [(1.0, 0.3), (4.0, -0.7), (9.0, 0.9), (0.2, 0.0)] {
            let (sxx, syy, txy) = c.stress(x, y);
            // equilibrium (no body force)
            let eq1 = d(|t| c.stress(t, y).0, x) + d(|t| c.stress(x, t).2, y);
            let eq2 = d(|t| c.stress(t, y).2, x) + d(|t| c.stress(x, t).1, y);
            assert!(eq1.abs() < 1e-6 && eq2.abs() < 1e-6, "{eq1} {eq2}");
            // Hooke (plane stress with the plane-strain constants) against the displacement gradients
            let exx = d(|t| c.displacement(t, y).0, x);
            let eyy = d(|t| c.displacement(x, t).1, y);
            let gxy = d(|t| c.displacement(x, t).0, y) + d(|t| c.displacement(t, y).1, x);
            assert!((exx - (sxx - c.nu * syy) / c.e).abs() < 1e-9, "exx");
            assert!((eyy - (syy - c.nu * sxx) / c.e).abs() < 1e-9, "eyy");
            assert!((gxy - txy / c.g).abs() < 1e-9, "gxy");
        }
        // tractions: free top and bottom, total shear P at the end
        assert!(c.stress(3.0, 1.0).2.abs() < 1e-12 && c.stress(3.0, -1.0).2.abs() < 1e-12);
        let n = 2000;
        let shear: f64 = (0..n)
            .map(|k| c.stress(10.0, -1.0 + 2.0 * (k as f64 + 0.5) / n as f64).2 * 2.0 / n as f64)
            .sum();
        assert!((shear + 3.0).abs() < 1e-6, "{shear}");
        // clamp conditions at the centroid of the clamped section
        let (u0, v0) = c.displacement(0.0, 0.0);
        assert!(u0.abs() < 1e-15 && v0.abs() < 1e-15);
        assert!(d(|t| c.displacement(0.0, t).0, 0.0).abs() < 1e-9);
        // tip deflection formula
        let (e, g, i, cc) = (c.e, c.g, 2.0 / 3.0, 1.0);
        let tip = -3.0 * 1000.0 / (3.0 * e * i) - 3.0 * cc * cc * 10.0 / (2.0 * i * g);
        assert!(
            (c.tip_deflection() - tip).abs() < 1e-12 * tip.abs(),
            "{} vs {tip}",
            c.tip_deflection()
        );
    }

    #[test]
    fn kirsch_satisfies_equilibrium_and_the_boundary_conditions() {
        let (a, s) = (1.0, 100.0);
        for (x, y) in [(1.7, 0.4), (-0.9, 2.2), (0.3, 1.5)] {
            let eq1 = d(|t| kirsch(a, s, t, y).0, x) + d(|t| kirsch(a, s, x, t).2, y);
            let eq2 = d(|t| kirsch(a, s, t, y).2, x) + d(|t| kirsch(a, s, x, t).1, y);
            assert!(eq1.abs() < 1e-5 && eq2.abs() < 1e-5, "{eq1} {eq2}");
        }
        // traction-free hole: σr = τrθ = 0 at r = a; hoop stress 3σ at θ = 90°, −σ at θ = 0
        for th in [0.3f64, 1.1, 2.0] {
            let (x, y) = (a * th.cos(), a * th.sin());
            let (sxx, syy, txy) = kirsch(a, s, x, y);
            let (c, sn) = (th.cos(), th.sin());
            let sr = sxx * c * c + syy * sn * sn + 2.0 * txy * sn * c;
            let trt = (syy - sxx) * sn * c + txy * (c * c - sn * sn);
            assert!(sr.abs() < 1e-9 && trt.abs() < 1e-9, "{sr} {trt}");
        }
        // at the top of the hole the hoop direction is x
        let (sxx, _, _) = kirsch(a, s, 0.0, a);
        assert!((sxx - 3.0 * s).abs() < 1e-9, "{sxx}");
        let (_, syy, _) = kirsch(a, s, a, 0.0);
        assert!((syy + s).abs() < 1e-9, "{syy}");
        // far field
        let (sxx, syy, txy) = kirsch(a, s, 400.0, 150.0);
        assert!((sxx - s).abs() < 1e-3 * s && syy.abs() < 1e-3 * s && txy.abs() < 1e-3 * s);
    }

    #[test]
    fn lame_solutions_satisfy_equilibrium_and_the_boundary_conditions() {
        let (a, b, p, e, nu) = (10.0, 25.0, 50.0, 210e3, 0.3);
        let (sr, _, _, _) = lame_cylinder(a, b, p, e, nu, a);
        assert!((sr + p).abs() < 1e-9);
        assert!(lame_cylinder(a, b, p, e, nu, b).0.abs() < 1e-9);
        // radial equilibrium  dσr/dr + (σr − σθ)/r = 0, and compatibility ε_θ = u/r
        for r in [11.0, 15.0, 22.0] {
            let eq = d(|t| lame_cylinder(a, b, p, e, nu, t).0, r)
                + (lame_cylinder(a, b, p, e, nu, r).0 - lame_cylinder(a, b, p, e, nu, r).1) / r;
            assert!(eq.abs() < 1e-6, "{eq}");
            let (sr, st, sz, ur) = lame_cylinder(a, b, p, e, nu, r);
            assert!((ur / r - (st - nu * (sr + sz)) / e).abs() < 1e-12);
            let err = d(|t| lame_cylinder(a, b, p, e, nu, t).3, r);
            let er = (sr - nu * (st + sz)) / e;
            assert!((err - er).abs() < 1e-9, "{err} {er}");
        }
        let (sr, _, _) = lame_sphere(a, b, p, e, nu, a);
        assert!((sr + p).abs() < 1e-9);
        assert!(lame_sphere(a, b, p, e, nu, b).0.abs() < 1e-9);
        for r in [11.0, 15.0, 22.0] {
            let eq = d(|t| lame_sphere(a, b, p, e, nu, t).0, r)
                + 2.0 * (lame_sphere(a, b, p, e, nu, r).0 - lame_sphere(a, b, p, e, nu, r).1) / r;
            assert!(eq.abs() < 1e-6, "{eq}");
            let (sr, st, ur) = lame_sphere(a, b, p, e, nu, r);
            assert!((ur / r - (st - nu * (sr + st)) / e).abs() < 1e-12);
            let err = d(|t| lame_sphere(a, b, p, e, nu, t).2, r);
            assert!((err - (sr - 2.0 * nu * st) / e).abs() < 1e-9);
        }
    }

    #[test]
    fn beam_frequencies_use_the_roots_of_the_characteristic_equation() {
        for bl in CANTILEVER_BETA_L {
            assert!(
                (bl.cos() * bl.cosh() + 1.0).abs() / bl.cosh() < 1e-7,
                "{bl}"
            );
        }
    }
}
