//! Nonlinear static analysis: large displacements and rotations (total Lagrangian, St. Venant–Kirchhoff
//! material) and von Mises plasticity with linear isotropic hardening, solved by Newton iterations with
//! load stepping.
//!
//! Scope: the strain stays moderate (a few percent, the plastic part additive in Green–Lagrange strain);
//! loads are dead loads (they keep their direction and magnitude as the body moves). A step that does not
//! converge is cut in half; a tangent that is not positive definite (a limit point, buckling) ends the run
//! with a message instead of a wrong answer.

use crate::assemble::{element_coords, empty_csr, scatter, Structure};
use crate::elem::{self, Rule};
use crate::model::Model;
use crate::solve::solve_direct;
use crate::sparse::{norm, Csr};
use kw_job::{CancelFlag, Progress};
use rayon::prelude::*;

/// What the nonlinear run needs besides the [`Model`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nonlinear {
    /// Yield stress (0: the material stays elastic).
    pub yield_stress: f64,
    /// Slope of the stress against the total strain after yielding (`E_t`), below `E`.
    pub tangent_modulus: f64,
    /// Large displacements. Off: the strain is the small-strain one (plasticity only).
    pub geometric: bool,
    /// Number of load steps to start with.
    pub steps: usize,
    pub max_iter: usize,
    /// Relative residual to stop at.
    pub tol: f64,
}

impl Default for Nonlinear {
    fn default() -> Self {
        Nonlinear {
            yield_stress: 0.0,
            tangent_modulus: 0.0,
            geometric: true,
            steps: 10,
            max_iter: 30,
            tol: 1e-8,
        }
    }
}

/// The history of one integration point.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GaussState {
    /// Plastic strain (Voigt, engineering shear).
    pub ep: [f64; 6],
    /// Accumulated equivalent plastic strain.
    pub alpha: f64,
}

/// Material constants for the point routines.
#[derive(Debug, Clone, Copy)]
struct Mat {
    d: [[f64; 6]; 6],
    g: f64,
    bulk: f64,
    sy: f64,
    /// Plastic modulus `H = E E_t / (E − E_t)`.
    h: f64,
}

impl Mat {
    fn new(e: f64, nu: f64, sy: f64, et: f64) -> Mat {
        let h = if sy > 0.0 && et > 0.0 && et < e {
            e * et / (e - et)
        } else {
            0.0
        };
        Mat {
            d: elem::elasticity(e, nu),
            g: e / (2.0 * (1.0 + nu)),
            bulk: e / (3.0 * (1.0 - 2.0 * nu)),
            sy,
            h,
        }
    }
}

/// Stress, consistent tangent and new state at a point with total strain `eps` (Voigt, engineering shear)
/// and the committed state `st`.
fn point(m: &Mat, eps: &[f64; 6], st: &GaussState) -> ([f64; 6], [[f64; 6]; 6], GaussState) {
    let mut ee = [0.0; 6];
    for i in 0..6 {
        ee[i] = eps[i] - st.ep[i];
    }
    let mut tr = [0.0; 6];
    for i in 0..6 {
        for j in 0..6 {
            tr[i] += m.d[i][j] * ee[j];
        }
    }
    if m.sy <= 0.0 {
        return (tr, m.d, *st);
    }
    let p = (tr[0] + tr[1] + tr[2]) / 3.0;
    let s = [tr[0] - p, tr[1] - p, tr[2] - p, tr[3], tr[4], tr[5]];
    let ss =
        s[0] * s[0] + s[1] * s[1] + s[2] * s[2] + 2.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
    let q = (1.5 * ss).sqrt();
    let yield_now = m.sy + m.h * st.alpha;
    if q <= yield_now * (1.0 + 1e-12) {
        return (tr, m.d, *st);
    }
    let dg = (q - yield_now) / (3.0 * m.g + m.h);
    let theta = 1.0 - 3.0 * m.g * dg / q;
    let mut sigma = [0.0; 6];
    for i in 0..3 {
        sigma[i] = s[i] * theta + p;
    }
    for i in 3..6 {
        sigma[i] = s[i] * theta;
    }
    let mut ep = st.ep;
    for i in 0..3 {
        ep[i] += dg * 1.5 * s[i] / q;
    }
    for i in 3..6 {
        ep[i] += 2.0 * dg * 1.5 * s[i] / q;
    }
    // consistent tangent: volumetric part + θ × deviatoric part − 2G θ̄ n nᵀ
    let sn = ss.sqrt();
    let n = [
        s[0] / sn,
        s[1] / sn,
        s[2] / sn,
        s[3] / sn,
        s[4] / sn,
        s[5] / sn,
    ];
    let tb = 3.0 * m.g / (3.0 * m.g + m.h) - (1.0 - theta);
    let mut c = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            let vol = if i < 3 && j < 3 { m.bulk } else { 0.0 };
            c[i][j] = vol + theta * (m.d[i][j] - vol) - 2.0 * m.g * tb * n[i] * n[j];
        }
    }
    (
        sigma,
        c,
        GaussState {
            ep,
            alpha: st.alpha + dg,
        },
    )
}

/// What one element gives at displacements `ue`.
pub struct ElementOut {
    /// Tangent stiffness (`30 × 30`, row-major); empty unless asked for.
    pub ke: Vec<f64>,
    /// Internal force.
    pub fe: [f64; 30],
    pub states: Vec<GaussState>,
    /// Von Mises stress of the Cauchy stress at every integration point.
    pub vm: Vec<f64>,
}

fn gauss_rule() -> Rule {
    elem::duffy(3)
}

/// Internal force (and, if `want_k`, the tangent) of one element. `states` are the committed histories of
/// its integration points.
pub fn element_response(
    x: &[[f64; 3]; 10],
    ue: &[f64; 30],
    m: &NlMat,
    geometric: bool,
    states: &[GaussState],
    want_k: bool,
) -> ElementOut {
    let rule = gauss_rule();
    let mut fe = [0.0; 30];
    let mut ke = if want_k { vec![0.0; 900] } else { Vec::new() };
    let mut new_states = Vec::with_capacity(rule.points.len());
    let mut vm = Vec::with_capacity(rule.points.len());
    for (gp, (l, w)) in rule.points.iter().zip(&rule.weights).enumerate() {
        let (g, det) = elem::grad(x, l);
        let wd = w * det.abs();
        // deformation gradient
        let mut f = [[0.0; 3]; 3];
        for k in 0..3 {
            f[k][k] = 1.0;
        }
        for i in 0..10 {
            for k in 0..3 {
                for a in 0..3 {
                    f[k][a] += ue[3 * i + k] * g[i][a];
                }
            }
        }
        let (eps, ff) = if geometric {
            let mut c = [[0.0; 3]; 3];
            for a in 0..3 {
                for b in 0..3 {
                    for k in 0..3 {
                        c[a][b] += f[k][a] * f[k][b];
                    }
                }
            }
            (
                [
                    0.5 * (c[0][0] - 1.0),
                    0.5 * (c[1][1] - 1.0),
                    0.5 * (c[2][2] - 1.0),
                    c[1][2],
                    c[2][0],
                    c[0][1],
                ],
                f,
            )
        } else {
            // small strain: the symmetric part of the displacement gradient
            let h = [
                [f[0][0] - 1.0, f[0][1], f[0][2]],
                [f[1][0], f[1][1] - 1.0, f[1][2]],
                [f[2][0], f[2][1], f[2][2] - 1.0],
            ];
            (
                [
                    h[0][0],
                    h[1][1],
                    h[2][2],
                    h[1][2] + h[2][1],
                    h[2][0] + h[0][2],
                    h[0][1] + h[1][0],
                ],
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            )
        };
        let (sigma, ctan, ns) = point(&m.0, &eps, &states[gp]);
        new_states.push(ns);
        // Cauchy stress for the report: σ = F S Fᵀ / J
        let s = [
            [sigma[0], sigma[5], sigma[4]],
            [sigma[5], sigma[1], sigma[3]],
            [sigma[4], sigma[3], sigma[2]],
        ];
        let j = ff[0][0] * (ff[1][1] * ff[2][2] - ff[1][2] * ff[2][1])
            - ff[0][1] * (ff[1][0] * ff[2][2] - ff[1][2] * ff[2][0])
            + ff[0][2] * (ff[1][0] * ff[2][1] - ff[1][1] * ff[2][0]);
        let mut fs = [[0.0; 3]; 3];
        for a in 0..3 {
            for b in 0..3 {
                for c in 0..3 {
                    fs[a][b] += ff[a][c] * s[c][b];
                }
            }
        }
        let mut cs = [[0.0; 3]; 3];
        for a in 0..3 {
            for b in 0..3 {
                for c in 0..3 {
                    cs[a][b] += fs[a][c] * ff[b][c];
                }
                cs[a][b] /= j.max(1e-12);
            }
        }
        vm.push(elem::von_mises(&[
            cs[0][0], cs[1][1], cs[2][2], cs[1][2], cs[2][0], cs[0][1],
        ]));
        // B matrix at this point
        let mut b = [[0.0; 30]; 6];
        for i in 0..10 {
            for k in 0..3 {
                let col = 3 * i + k;
                b[0][col] = ff[k][0] * g[i][0];
                b[1][col] = ff[k][1] * g[i][1];
                b[2][col] = ff[k][2] * g[i][2];
                b[3][col] = ff[k][1] * g[i][2] + ff[k][2] * g[i][1];
                b[4][col] = ff[k][2] * g[i][0] + ff[k][0] * g[i][2];
                b[5][col] = ff[k][0] * g[i][1] + ff[k][1] * g[i][0];
            }
        }
        for col in 0..30 {
            let mut acc = 0.0;
            for r in 0..6 {
                acc += b[r][col] * sigma[r];
            }
            fe[col] += wd * acc;
        }
        if want_k {
            // BᵀCB
            let mut cb = [[0.0; 30]; 6];
            for r in 0..6 {
                for col in 0..30 {
                    let mut acc = 0.0;
                    for q in 0..6 {
                        acc += ctan[r][q] * b[q][col];
                    }
                    cb[r][col] = acc;
                }
            }
            for i in 0..30 {
                for j2 in 0..30 {
                    let mut acc = 0.0;
                    for r in 0..6 {
                        acc += b[r][i] * cb[r][j2];
                    }
                    ke[i * 30 + j2] += wd * acc;
                }
            }
            if geometric {
                for i in 0..10 {
                    for j2 in 0..10 {
                        let mut sij = 0.0;
                        for a in 0..3 {
                            for c in 0..3 {
                                sij += g[i][a] * s[a][c] * g[j2][c];
                            }
                        }
                        for k in 0..3 {
                            ke[(3 * i + k) * 30 + 3 * j2 + k] += wd * sij;
                        }
                    }
                }
            }
        }
    }
    ElementOut {
        ke,
        fe,
        states: new_states,
        vm,
    }
}

/// Elastic and plastic constants of a material, for [`element_response`].
#[derive(Debug, Clone, Copy)]
pub struct NlMat(Mat);

impl NlMat {
    pub fn new(e: f64, nu: f64, yield_stress: f64, tangent_modulus: f64) -> NlMat {
        NlMat(Mat::new(e, nu, yield_stress, tangent_modulus))
    }
}

/// The result of a nonlinear run.
#[derive(Debug, Clone)]
pub struct NlSolution {
    /// Displacements at the end, 3 per node.
    pub u: Vec<f64>,
    /// The load factor reached after every converged step, and the monitored displacement at it.
    pub load_factors: Vec<f64>,
    pub monitor: Vec<f64>,
    /// Newton iterations of every step.
    pub iterations: Vec<usize>,
    /// Von Mises stress (of the Cauchy stress) and equivalent plastic strain at the nodes.
    pub von_mises: Vec<f64>,
    pub plastic_strain: Vec<f64>,
}

/// Solve `model` under its loads and prescribed displacements, scaled from 0 to 1 in steps. `monitor` is
/// the `(node, axis)` whose displacement is recorded after each step.
pub fn solve_nonlinear(
    model: &Model,
    nl: &Nonlinear,
    monitor: Option<(u32, usize)>,
    cancel: Option<&CancelFlag>,
    progress: Option<&Progress>,
) -> Result<NlSolution, String> {
    let mesh = &model.mesh;
    let n = 3 * mesh.nodes.len();
    let ne = mesh.elems.len();
    let rule_n = gauss_rule().points.len();
    let mats: Vec<NlMat> = if model.materials.is_empty() {
        vec![NlMat::new(
            model.material.e,
            model.material.nu,
            nl.yield_stress,
            nl.tangent_modulus,
        )]
    } else {
        model
            .materials
            .iter()
            .map(|m| NlMat::new(m.e, m.nu, nl.yield_stress, nl.tangent_modulus))
            .collect()
    };
    let mat_of = |ei: usize| -> &NlMat {
        match model.elem_material.get(ei) {
            Some(&k) if mats.len() > 1 => &mats[k as usize],
            _ => &mats[0],
        }
    };
    let st = model.structure();
    let f_ref = model.load_vector();
    let fixed: Vec<(usize, f64)> = model
        .fixed
        .iter()
        .map(|c| (3 * c.node as usize + c.axis, c.value))
        .collect();
    let mut is_fixed = vec![false; n];
    for (d, _) in &fixed {
        is_fixed[*d] = true;
    }
    let mut committed = vec![GaussState::default(); ne * rule_n];
    let mut u = vec![0.0; n];
    let mut lambda = 0.0f64;
    let mut dl = 1.0 / nl.steps.max(1) as f64;
    let min_dl = dl / 256.0;
    let mut out = NlSolution {
        u: Vec::new(),
        load_factors: Vec::new(),
        monitor: Vec::new(),
        iterations: Vec::new(),
        von_mises: Vec::new(),
        plastic_strain: Vec::new(),
    };
    let mut last_vm: Vec<f64> = Vec::new();
    let mut last_states = committed.clone();
    let assemble = |u: &[f64], committed: &[GaussState], want_k: bool, st: &Structure| {
        let outs: Vec<ElementOut> = (0..ne)
            .into_par_iter()
            .map(|ei| {
                let e = &mesh.elems[ei];
                let x = element_coords(mesh, e);
                let mut ue = [0.0; 30];
                for i in 0..10 {
                    for a in 0..3 {
                        ue[3 * i + a] = u[3 * e[i] as usize + a];
                    }
                }
                element_response(
                    &x,
                    &ue,
                    mat_of(ei),
                    nl.geometric,
                    &committed[ei * rule_n..(ei + 1) * rule_n],
                    want_k,
                )
            })
            .collect();
        let mut fint = vec![0.0; n];
        for (e, o) in mesh.elems.iter().zip(&outs) {
            for i in 0..10 {
                for a in 0..3 {
                    fint[3 * e[i] as usize + a] += o.fe[3 * i + a];
                }
            }
        }
        let k: Option<Csr> = want_k.then(|| {
            let mut k = empty_csr(mesh.nodes.len(), &st.pattern, 3);
            scatter(&mut k, &st.pattern, &st.colours, &mesh.elems, 3, &|ei| {
                outs[ei].ke.clone()
            });
            k
        });
        (fint, k, outs)
    };
    while lambda < 1.0 - 1e-12 {
        if cancel.is_some_and(|c| c.is_cancelled()) {
            return Err("cancelled".into());
        }
        let target = (lambda + dl).min(1.0);
        let mut trial = u.clone();
        for (d, v) in &fixed {
            trial[*d] = v * target;
        }
        let mut converged = false;
        let mut iters = 0;
        let mut failure = String::new();
        let mut step_states = committed.clone();
        let mut step_vm: Vec<f64> = Vec::new();
        for it in 0..=nl.max_iter {
            let (mut fint, k, outs) = assemble(&trial, &committed, true, &st);
            // bonded contact: the penalty is part of the tangent, so its force is part of the residual
            let mut k = k;
            if !model.ties.is_empty() {
                let kk = k.as_mut().expect("tangent assembled");
                let alphas = crate::tie::tie_alphas(kk, 3, &model.ties, model.tie_penalty);
                crate::tie::apply_penalty_with(kk, 3, &model.ties, &alphas);
                crate::tie::penalty_force(3, &model.ties, &alphas, &trial, &mut fint);
            }
            for (ei, o) in outs.iter().enumerate() {
                step_states[ei * rule_n..(ei + 1) * rule_n].copy_from_slice(&o.states);
            }
            step_vm = outs.iter().flat_map(|o| o.vm.iter().copied()).collect();
            let mut r: Vec<f64> = (0..n).map(|i| target * f_ref[i] - fint[i]).collect();
            for i in 0..n {
                if is_fixed[i] {
                    r[i] = 0.0;
                }
            }
            let rn = norm(&r);
            let scale = norm(&fint).max(norm(&f_ref) * target).max(1e-30);
            if rn <= nl.tol * scale {
                converged = true;
                iters = it;
                break;
            }
            if it == nl.max_iter {
                failure = format!(
                    "no convergence in {} iterations (residual {:.2e})",
                    it,
                    rn / scale
                );
                break;
            }
            let mut k = k.expect("tangent assembled");
            let zero: Vec<(usize, f64)> = fixed.iter().map(|(d, _)| (*d, 0.0)).collect();
            k.apply_dirichlet(&mut r, &zero);
            match solve_direct(&k, &r) {
                Ok(du) => {
                    if du.iter().any(|v| !v.is_finite()) {
                        failure = "the iteration diverged".into();
                        break;
                    }
                    for i in 0..n {
                        trial[i] += du[i];
                    }
                }
                Err(_) => {
                    failure = "the tangent stiffness is not positive definite (a limit point or buckling)"
                        .into();
                    break;
                }
            }
        }
        if converged {
            u = trial;
            committed = step_states;
            last_states = committed.clone();
            last_vm = step_vm;
            lambda = target;
            out.load_factors.push(lambda);
            out.iterations.push(iters);
            out.monitor
                .push(monitor.map_or(0.0, |(node, axis)| u[3 * node as usize + axis]));
            if let Some(p) = progress {
                p.set(lambda);
            }
            if iters <= 4 {
                dl = (dl * 1.5).min(1.0 / nl.steps.max(1) as f64 * 2.0);
            }
        } else {
            dl *= 0.5;
            if dl < min_dl {
                return Err(format!("stopped at load factor {:.4}: {failure}", lambda));
            }
        }
    }
    // nodal values: integration point values spread with the shape functions and a lumped weight
    let rule = gauss_rule();
    let nn = mesh.nodes.len();
    let (mut vm_n, mut ep_n, mut w_n) = (vec![0.0; nn], vec![0.0; nn], vec![0.0; nn]);
    for (ei, e) in mesh.elems.iter().enumerate() {
        let x = element_coords(mesh, e);
        for (gp, (l, w)) in rule.points.iter().zip(&rule.weights).enumerate() {
            let (_, det) = elem::grad(&x, l);
            let shape = elem::shape(l);
            let wd = w * det.abs();
            let vm = last_vm.get(ei * rule_n + gp).copied().unwrap_or(0.0);
            let al = last_states[ei * rule_n + gp].alpha;
            for i in 0..10 {
                let wi = wd * shape[i].abs();
                vm_n[e[i] as usize] += wi * vm;
                ep_n[e[i] as usize] += wi * al;
                w_n[e[i] as usize] += wi;
            }
        }
    }
    for i in 0..nn {
        if w_n[i] > 0.0 {
            vm_n[i] /= w_n[i];
            ep_n[i] /= w_n[i];
        }
    }
    out.u = u;
    out.von_mises = vm_n;
    out.plastic_strain = ep_n;
    Ok(out)
}
