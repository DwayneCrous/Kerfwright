//! SIMP topology optimisation of a 2-D compliance problem: a transcription of the 88-line MATLAB code of
//! Andreassen, Clausen, Schevenels, Lazarov & Sigmund, "Efficient topology optimization in MATLAB using 88
//! lines of code", *Struct Multidisc Optim* 43 (2011) 1–16 — modified SIMP, sensitivity (`ft = 1`) or
//! density (`ft = 2`) filter, optimality-criteria update with bisection on the Lagrange multiplier, stop
//! when the largest change of a design variable drops below 1 %. The load case is the half MBB beam of
//! the paper's figure 1: unit downward force at the top-left corner, symmetry on the left edge, roller at
//! the bottom-right corner.
//!
//! Element numbering, node numbering and the element stiffness matrix follow the MATLAB code, so that
//! results can be compared with it directly.

use crate::solve::solve_direct;
use crate::sparse::Csr;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Filter {
    /// `ft = 1`: sensitivity filter.
    Sensitivity,
    /// `ft = 2`: density filter.
    Density,
}

#[derive(Debug, Clone)]
pub struct SimpParams {
    pub nelx: usize,
    pub nely: usize,
    pub volfrac: f64,
    pub penal: f64,
    pub rmin: f64,
    pub filter: Filter,
    pub max_iterations: usize,
}

impl SimpParams {
    /// The MBB beam of the paper: `top88(nelx, nely, volfrac, penal, rmin, ft)`.
    pub fn mbb(
        nelx: usize,
        nely: usize,
        volfrac: f64,
        penal: f64,
        rmin: f64,
        filter: Filter,
    ) -> SimpParams {
        SimpParams {
            nelx,
            nely,
            volfrac,
            penal,
            rmin,
            filter,
            max_iterations: 1000,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SimpResult {
    /// Compliance of the last iteration (what the MATLAB code prints as `Obj.`).
    pub compliance: f64,
    pub iterations: usize,
    pub history: Vec<f64>,
    /// Physical densities, element `(ey, ex)` at `ex * nely + ey`.
    pub density: Vec<f64>,
    pub volume_fraction: f64,
}

const E0: f64 = 1.0;
const EMIN: f64 = 1e-9;
const NU: f64 = 0.3;

/// Element stiffness matrix of the bilinear plane-stress square for `E = 1`, as in the MATLAB code
/// (lines 8–12). Local degrees of freedom: bottom-left, bottom-right, top-right, top-left nodes.
pub fn element_stiffness() -> [[f64; 8]; 8] {
    let a11: [[f64; 4]; 4] = [
        [12.0, 3.0, -6.0, -3.0],
        [3.0, 12.0, 3.0, 0.0],
        [-6.0, 3.0, 12.0, -3.0],
        [-3.0, 0.0, -3.0, 12.0],
    ];
    let a12: [[f64; 4]; 4] = [
        [-6.0, -3.0, 0.0, 3.0],
        [-3.0, -6.0, -3.0, -6.0],
        [0.0, -3.0, -6.0, 3.0],
        [3.0, -6.0, 3.0, -6.0],
    ];
    let b11: [[f64; 4]; 4] = [
        [-4.0, 3.0, -2.0, 9.0],
        [3.0, -4.0, -9.0, 4.0],
        [-2.0, -9.0, -4.0, -3.0],
        [9.0, 4.0, -3.0, -4.0],
    ];
    let b12: [[f64; 4]; 4] = [
        [2.0, -3.0, 4.0, -9.0],
        [-3.0, 2.0, 9.0, -2.0],
        [4.0, 9.0, 2.0, 3.0],
        [-9.0, -2.0, 3.0, 2.0],
    ];
    let mut ke = [[0.0; 8]; 8];
    let f = 1.0 / (1.0 - NU * NU) / 24.0;
    for i in 0..4 {
        for j in 0..4 {
            ke[i][j] = f * (a11[i][j] + NU * b11[i][j]);
            ke[i][j + 4] = f * (a12[i][j] + NU * b12[i][j]);
            ke[i + 4][j] = f * (a12[j][i] + NU * b12[j][i]);
            ke[i + 4][j + 4] = f * (a11[i][j] + NU * b11[i][j]);
        }
    }
    ke
}

pub fn simp(p: &SimpParams) -> Result<SimpResult, String> {
    let (nelx, nely) = (p.nelx, p.nely);
    let nel = nelx * nely;
    let ndof = 2 * (nelx + 1) * (nely + 1);
    let ke = element_stiffness();

    // element degrees of freedom (0-based): local nodes BL, BR, TR, TL; node (row i, column j) is
    // numbered j*(nely+1)+i, the dofs of node n are 2n (x) and 2n+1 (y)
    let node = |i: usize, j: usize| j * (nely + 1) + i;
    let edof: Vec<[usize; 8]> = (0..nel)
        .map(|e| {
            let (ex, ey) = (e / nely, e % nely);
            let n = [
                node(ey + 1, ex),
                node(ey + 1, ex + 1),
                node(ey, ex + 1),
                node(ey, ex),
            ];
            let mut d = [0; 8];
            for k in 0..4 {
                d[2 * k] = 2 * n[k];
                d[2 * k + 1] = 2 * n[k] + 1;
            }
            d
        })
        .collect();

    // loads and supports (half MBB beam)
    let mut f = vec![0.0; ndof];
    f[1] = -1.0;
    let mut fixed = vec![false; ndof];
    for i in 0..=nely {
        fixed[2 * node(i, 0)] = true;
    }
    fixed[ndof - 1] = true;
    let fixed_list: Vec<(usize, f64)> = (0..ndof).filter(|&d| fixed[d]).map(|d| (d, 0.0)).collect();

    // filter: weights max(0, rmin − distance) within the window of ceil(rmin) − 1 elements
    let rc = p.rmin.ceil() as isize - 1;
    let mut h: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nel];
    let mut hs = vec![0.0; nel];
    for i1 in 0..nelx as isize {
        for j1 in 0..nely as isize {
            let e1 = (i1 as usize) * nely + j1 as usize;
            for i2 in (i1 - rc).max(0)..=(i1 + rc).min(nelx as isize - 1) {
                for j2 in (j1 - rc).max(0)..=(j1 + rc).min(nely as isize - 1) {
                    let e2 = (i2 as usize) * nely + j2 as usize;
                    let w =
                        (p.rmin - (((i1 - i2).pow(2) + (j1 - j2).pow(2)) as f64).sqrt()).max(0.0);
                    if w > 0.0 {
                        h[e1].push((e2, w));
                        hs[e1] += w;
                    }
                }
            }
        }
    }
    let apply_h = |v: &[f64]| -> Vec<f64> {
        (0..nel)
            .map(|e| h[e].iter().map(|&(k, w)| w * v[k]).sum())
            .collect()
    };

    let mut x = vec![p.volfrac; nel];
    let mut x_phys = x.clone();
    let mut change = 1.0;
    let mut history = Vec::new();
    let mut loop_count = 0;
    let mut c = 0.0;
    while change > 0.01 && loop_count < p.max_iterations {
        loop_count += 1;
        // finite-element analysis
        let mut trip: Vec<std::collections::BTreeMap<u32, f64>> = vec![Default::default(); ndof];
        for e in 0..nel {
            let ee = EMIN + x_phys[e].powf(p.penal) * (E0 - EMIN);
            for a in 0..8 {
                for b in 0..8 {
                    *trip[edof[e][a]].entry(edof[e][b] as u32).or_insert(0.0) += ee * ke[a][b];
                }
            }
        }
        let mut row_ptr = vec![0usize; ndof + 1];
        let mut col = Vec::new();
        let mut val = Vec::new();
        for (r, row) in trip.iter().enumerate() {
            for (&cc, &v) in row {
                col.push(cc);
                val.push(v);
            }
            row_ptr[r + 1] = col.len();
        }
        let mut k = Csr {
            n: ndof,
            row_ptr,
            col,
            val,
        };
        let mut rhs = f.clone();
        k.apply_dirichlet(&mut rhs, &fixed_list);
        let u = solve_direct(&k, &rhs)?;
        // objective and sensitivities
        let mut ce = vec![0.0; nel];
        for e in 0..nel {
            let ue: [f64; 8] = std::array::from_fn(|a| u[edof[e][a]]);
            let mut s = 0.0;
            for a in 0..8 {
                for b in 0..8 {
                    s += ue[a] * ke[a][b] * ue[b];
                }
            }
            ce[e] = s;
        }
        c = (0..nel)
            .map(|e| (EMIN + x_phys[e].powf(p.penal) * (E0 - EMIN)) * ce[e])
            .sum();
        let mut dc: Vec<f64> = (0..nel)
            .map(|e| -p.penal * (E0 - EMIN) * x_phys[e].powf(p.penal - 1.0) * ce[e])
            .collect();
        let mut dv = vec![1.0; nel];
        // filtering / modification of sensitivities
        match p.filter {
            Filter::Sensitivity => {
                let t: Vec<f64> = (0..nel).map(|e| x[e] * dc[e]).collect();
                let ht = apply_h(&t);
                for e in 0..nel {
                    dc[e] = ht[e] / hs[e] / x[e].max(1e-3);
                }
            }
            Filter::Density => {
                let t: Vec<f64> = (0..nel).map(|e| dc[e] / hs[e]).collect();
                dc = apply_h(&t);
                let t: Vec<f64> = (0..nel).map(|e| dv[e] / hs[e]).collect();
                dv = apply_h(&t);
            }
        }
        // optimality criteria update
        let (mut l1, mut l2, mv) = (0.0f64, 1e9f64, 0.2f64);
        let mut xnew = x.clone();
        while (l2 - l1) / (l1 + l2) > 1e-3 {
            let lmid = 0.5 * (l2 + l1);
            for e in 0..nel {
                let b = (-dc[e] / dv[e] / lmid).sqrt();
                xnew[e] = 0.0f64.max((x[e] - mv).max(1.0f64.min((x[e] + mv).min(x[e] * b))));
            }
            x_phys = match p.filter {
                Filter::Sensitivity => xnew.clone(),
                Filter::Density => {
                    let hx = apply_h(&xnew);
                    (0..nel).map(|e| hx[e] / hs[e]).collect()
                }
            };
            if x_phys.iter().sum::<f64>() > p.volfrac * nel as f64 {
                l1 = lmid;
            } else {
                l2 = lmid;
            }
        }
        change = (0..nel).map(|e| (xnew[e] - x[e]).abs()).fold(0.0, f64::max);
        x = xnew;
        history.push(c);
    }
    Ok(SimpResult {
        compliance: c,
        iterations: loop_count,
        history,
        volume_fraction: x_phys.iter().sum::<f64>() / nel as f64,
        density: x_phys,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn element_stiffness_is_symmetric_with_rigid_body_modes() {
        let ke = element_stiffness();
        for i in 0..8 {
            for j in 0..8 {
                assert!((ke[i][j] - ke[j][i]).abs() < 1e-12);
            }
        }
        // translations are in the null space
        for axis in 0..2 {
            let mut u = [0.0; 8];
            for k in 0..4 {
                u[2 * k + axis] = 1.0;
            }
            for i in 0..8 {
                let s: f64 = (0..8).map(|j| ke[i][j] * u[j]).sum();
                assert!(s.abs() < 1e-12, "{s}");
            }
        }
        // a rotation too
        let xy = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut u = [0.0; 8];
        for (k, (x, y)) in xy.iter().enumerate() {
            u[2 * k] = -y;
            u[2 * k + 1] = *x;
        }
        for i in 0..8 {
            let s: f64 = (0..8).map(|j| ke[i][j] * u[j]).sum();
            assert!(s.abs() < 1e-12, "{s}");
        }
        // the unit square carries its Young's modulus: uniform strain energy of ε_x = 1 is E/(1−ν²) /2
        let mut u = [0.0; 8];
        for (k, (x, _)) in xy.iter().enumerate() {
            u[2 * k] = *x;
        }
        let e: f64 = (0..8)
            .map(|i| (0..8).map(|j| u[i] * ke[i][j] * u[j]).sum::<f64>())
            .sum();
        assert!((e - 1.0 / (1.0 - NU * NU)).abs() < 1e-12, "{e}");
    }

    #[test]
    fn a_small_run_converges_and_keeps_the_volume() {
        let r = simp(&SimpParams::mbb(30, 10, 0.5, 3.0, 1.2, Filter::Sensitivity)).unwrap();
        assert!(r.iterations > 5 && r.iterations < 400);
        assert!(
            (r.volume_fraction - 0.5).abs() < 2e-3,
            "{}",
            r.volume_fraction
        );
        // compliance falls
        assert!(r.history.last().unwrap() < &r.history[0]);
    }
}
