//! The quadratic tetrahedron (TET10): shape functions, quadrature, element stiffness and mass.

use kw_mesh3d::{P3, TET10_EDGES};

/// Isotropic elastic constitutive matrix (Voigt: xx yy zz yz zx xy, engineering shear strains).
pub fn elasticity(e: f64, nu: f64) -> [[f64; 6]; 6] {
    let c = e / ((1.0 + nu) * (1.0 - 2.0 * nu));
    let (a, b, g) = (c * (1.0 - nu), c * nu, c * (1.0 - 2.0 * nu) / 2.0);
    let mut d = [[0.0; 6]; 6];
    for i in 0..3 {
        for j in 0..3 {
            d[i][j] = if i == j { a } else { b };
        }
        d[3 + i][3 + i] = g;
    }
    d
}

/// A quadrature rule on the unit tetrahedron: barycentric points and weights (summing to 1/6).
pub struct Rule {
    pub points: Vec<[f64; 4]>,
    pub weights: Vec<f64>,
}

/// The 4-point rule of degree 2 (exact for the stiffness of straight-edged TET10).
pub fn rule4() -> Rule {
    let (a, b) = (0.138_196_601_125_010_5, 0.585_410_196_624_968_5);
    Rule {
        points: vec![[b, a, a, a], [a, b, a, a], [a, a, b, a], [a, a, a, b]],
        weights: vec![1.0 / 24.0; 4],
    }
}

/// Gauss–Legendre nodes and weights on [0, 1].
pub fn gauss01(n: usize) -> (Vec<f64>, Vec<f64>) {
    // Newton iteration on the Legendre polynomial
    let mut x = vec![0.0; n];
    let mut w = vec![0.0; n];
    for i in 0..n {
        let mut z = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, z);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * z * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            let dp = n as f64 * (z * p1 - p0) / (z * z - 1.0);
            let dz = p1 / dp;
            z -= dz;
            if dz.abs() < 1e-15 {
                break;
            }
        }
        let (mut p0, mut p1) = (1.0, z);
        for k in 2..=n {
            let p2 = ((2 * k - 1) as f64 * z * p1 - (k - 1) as f64 * p0) / k as f64;
            p0 = p1;
            p1 = p2;
        }
        let dp = n as f64 * (z * p1 - p0) / (z * z - 1.0);
        x[i] = 0.5 * (1.0 - z);
        w[i] = 1.0 / ((1.0 - z * z) * dp * dp);
    }
    (x, w)
}

/// Collapsed (Duffy) tensor rule with `n³` points, exact for polynomials of degree `2n − 2` or less.
pub fn duffy(n: usize) -> Rule {
    let (g, gw) = gauss01(n);
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for a in 0..n {
        for b in 0..n {
            for c in 0..n {
                let (u, v, w) = (g[a], g[b], g[c]);
                let xi = u;
                let eta = (1.0 - u) * v;
                let zeta = (1.0 - u) * (1.0 - v) * w;
                points.push([1.0 - xi - eta - zeta, xi, eta, zeta]);
                weights.push(gw[a] * gw[b] * gw[c] * (1.0 - u) * (1.0 - u) * (1.0 - v));
            }
        }
    }
    Rule { points, weights }
}

pub fn shape(l: &[f64; 4]) -> [f64; 10] {
    let mut n = [0.0; 10];
    for i in 0..4 {
        n[i] = l[i] * (2.0 * l[i] - 1.0);
    }
    for (k, (a, b)) in TET10_EDGES.iter().enumerate() {
        n[4 + k] = 4.0 * l[*a] * l[*b];
    }
    n
}

/// Gradients of the shape functions with respect to physical coordinates, and `det J`.
pub fn grad(x: &[P3; 10], l: &[f64; 4]) -> ([[f64; 3]; 10], f64) {
    let dn = kw_mesh3d::tet10_dn_dl(l);
    // dN/dxi_k = dN/dL_{k+1} - dN/dL_0
    let mut dxi = [[0.0; 3]; 10];
    for i in 0..10 {
        for k in 0..3 {
            dxi[i][k] = dn[i][k + 1] - dn[i][0];
        }
    }
    let mut j = [[0.0; 3]; 3]; // j[a][k] = dx_a / dxi_k
    for i in 0..10 {
        for a in 0..3 {
            for k in 0..3 {
                j[a][k] += x[i][a] * dxi[i][k];
            }
        }
    }
    let det = j[0][0] * (j[1][1] * j[2][2] - j[1][2] * j[2][1])
        - j[0][1] * (j[1][0] * j[2][2] - j[1][2] * j[2][0])
        + j[0][2] * (j[1][0] * j[2][1] - j[1][1] * j[2][0]);
    let inv_det = 1.0 / det;
    // inverse of j
    let ji = [
        [
            (j[1][1] * j[2][2] - j[1][2] * j[2][1]) * inv_det,
            (j[0][2] * j[2][1] - j[0][1] * j[2][2]) * inv_det,
            (j[0][1] * j[1][2] - j[0][2] * j[1][1]) * inv_det,
        ],
        [
            (j[1][2] * j[2][0] - j[1][0] * j[2][2]) * inv_det,
            (j[0][0] * j[2][2] - j[0][2] * j[2][0]) * inv_det,
            (j[0][2] * j[1][0] - j[0][0] * j[1][2]) * inv_det,
        ],
        [
            (j[1][0] * j[2][1] - j[1][1] * j[2][0]) * inv_det,
            (j[0][1] * j[2][0] - j[0][0] * j[2][1]) * inv_det,
            (j[0][0] * j[1][1] - j[0][1] * j[1][0]) * inv_det,
        ],
    ];
    // dN/dx_a = sum_k dN/dxi_k * dxi_k/dx_a = sum_k dxi[i][k] * ji[k][a]
    let mut g = [[0.0; 3]; 10];
    for i in 0..10 {
        for a in 0..3 {
            for k in 0..3 {
                g[i][a] += dxi[i][k] * ji[k][a];
            }
        }
    }
    (g, det)
}

/// Strain-displacement matrix B (6 × 30) from shape gradients.
pub fn bmatrix(g: &[[f64; 3]; 10]) -> [[f64; 30]; 6] {
    let mut b = [[0.0; 30]; 6];
    for i in 0..10 {
        let (dx, dy, dz) = (g[i][0], g[i][1], g[i][2]);
        let c = 3 * i;
        b[0][c] = dx;
        b[1][c + 1] = dy;
        b[2][c + 2] = dz;
        b[3][c + 1] = dz;
        b[3][c + 2] = dy;
        b[4][c] = dz;
        b[4][c + 2] = dx;
        b[5][c] = dy;
        b[5][c + 1] = dx;
    }
    b
}

/// Element stiffness (30 × 30, row-major) with the 4-point rule.
pub fn stiffness(x: &[P3; 10], d: &[[f64; 6]; 6], rule: &Rule) -> Vec<f64> {
    let mut ke = vec![0.0; 900];
    for (l, w) in rule.points.iter().zip(&rule.weights) {
        let (g, det) = grad(x, l);
        let b = bmatrix(&g);
        // DB (6 × 30)
        let mut db = [[0.0; 30]; 6];
        for i in 0..6 {
            for j in 0..30 {
                let mut s = 0.0;
                for k in 0..6 {
                    s += d[i][k] * b[k][j];
                }
                db[i][j] = s;
            }
        }
        let f = w * det.abs();
        for i in 0..30 {
            for j in 0..30 {
                let mut s = 0.0;
                for k in 0..6 {
                    s += b[k][i] * db[k][j];
                }
                ke[i * 30 + j] += f * s;
            }
        }
    }
    ke
}

/// Consistent element mass (30 × 30) with density `rho`.
pub fn mass(x: &[P3; 10], rho: f64, rule: &Rule) -> Vec<f64> {
    let mut me = vec![0.0; 900];
    for (l, w) in rule.points.iter().zip(&rule.weights) {
        let (_, det) = grad(x, l);
        let n = shape(l);
        let f = rho * w * det.abs();
        for a in 0..10 {
            for b in 0..10 {
                let v = f * n[a] * n[b];
                for c in 0..3 {
                    me[(3 * a + c) * 30 + 3 * b + c] += v;
                }
            }
        }
    }
    me
}

/// Stress `D B u` (Voigt) at barycentric point `l` for element nodal displacements `ue` (30).
pub fn stress_at(x: &[P3; 10], ue: &[f64; 30], d: &[[f64; 6]; 6], l: &[f64; 4]) -> [f64; 6] {
    let (g, _) = grad(x, l);
    let b = bmatrix(&g);
    let mut eps = [0.0; 6];
    for i in 0..6 {
        for j in 0..30 {
            eps[i] += b[i][j] * ue[j];
        }
    }
    let mut s = [0.0; 6];
    for i in 0..6 {
        for k in 0..6 {
            s[i] += d[i][k] * eps[k];
        }
    }
    s
}

pub fn von_mises(s: &[f64; 6]) -> f64 {
    let (xx, yy, zz, yz, zx, xy) = (s[0], s[1], s[2], s[3], s[4], s[5]);
    (0.5 * ((xx - yy).powi(2) + (yy - zz).powi(2) + (zz - xx).powi(2))
        + 3.0 * (yz * yz + zx * zx + xy * xy))
        .sqrt()
}

/// The coordinates of the ten nodes of an element.
pub fn element_coords_of(mesh: &kw_mesh3d::Tet10Mesh, e: &[u32; 10]) -> [P3; 10] {
    let mut x = [[0.0; 3]; 10];
    for i in 0..10 {
        x[i] = mesh.nodes[e[i] as usize];
    }
    x
}
