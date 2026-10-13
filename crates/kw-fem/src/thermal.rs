//! Steady-state heat conduction on the quadratic tetrahedra: `∇·(k ∇T) + Q = 0` with prescribed
//! temperatures, heat flux, convection to an ambient temperature and volumetric heat generation. One
//! temperature per node; bodies that touch are tied exactly as in the structural model.
//!
//! Units follow the structural ones (mm, N, tonne, s): conductivity in mW/(mm·K) (numerically W/(m·K)), flux
//! in mW/mm², convection in mW/(mm²·K), generation in mW/mm³ (`kw-materials` converts from the usual units).

use crate::assemble::{assemble_scalar, element_coords, Structure};
use crate::elem;
use crate::model::SolverKind;
use crate::solve::{pcg, solve_direct, Precond};
use crate::sparse::Csr;
use crate::tie::{apply_penalty, tie_groups, tri6_shape, Tie};
use kw_mesh3d::{Tet10Mesh, P3};

#[derive(Debug, Clone)]
pub struct ThermalModel {
    pub mesh: Tet10Mesh,
    /// Conductivity used by elements without an entry in `conductivities`.
    pub conductivity: f64,
    /// Conductivity per body (indexed by `elem_material`).
    pub conductivities: Vec<f64>,
    pub elem_material: Vec<u16>,
    /// Volumetric heat generation of every element (mW/mm³) and, per body, of the elements of that body.
    pub generation: f64,
    pub generations: Vec<f64>,
    /// Prescribed temperatures `(node, T)`.
    pub fixed: Vec<(u32, f64)>,
    /// Heat flowing *into* the body through the faces with this tag (mW/mm²).
    pub flux: Vec<(u32, f64)>,
    /// Convection on the faces with this tag: `(tag, h, ambient temperature)`.
    pub convection: Vec<(u32, f64, f64)>,
    pub ties: Vec<Tie>,
    pub tie_penalty: f64,
}

impl ThermalModel {
    pub fn new(mesh: Tet10Mesh, conductivity: f64) -> ThermalModel {
        ThermalModel {
            mesh,
            conductivity,
            conductivities: Vec::new(),
            elem_material: Vec::new(),
            generation: 0.0,
            generations: Vec::new(),
            fixed: Vec::new(),
            flux: Vec::new(),
            convection: Vec::new(),
            ties: Vec::new(),
            tie_penalty: crate::model::DEFAULT_TIE_PENALTY,
        }
    }

    /// Prescribe `t` at every node for which `pred` holds.
    pub fn fix_where(&mut self, t: f64, pred: &dyn Fn(P3) -> bool) {
        for (i, p) in self.mesh.nodes.iter().enumerate() {
            if pred(*p) {
                self.fixed.push((i as u32, t));
            }
        }
    }

    fn k_of(&self, ei: usize) -> f64 {
        match self.elem_material.get(ei) {
            Some(&m) if !self.conductivities.is_empty() => self.conductivities[m as usize],
            _ => self.conductivity,
        }
    }

    fn q_of(&self, ei: usize) -> f64 {
        match self.elem_material.get(ei) {
            Some(&m) if (m as usize) < self.generations.len() => self.generations[m as usize],
            _ => self.generation,
        }
    }

    pub fn structure(&self) -> Structure {
        if self.ties.is_empty() {
            Structure::new(&self.mesh)
        } else {
            Structure::with_couplings(&self.mesh, &tie_groups(&self.ties))
        }
    }

    /// Conduction matrix plus the convection terms, and the right-hand side (flux, convection, generation).
    pub fn assemble(&self) -> (Csr, Vec<f64>) {
        let mesh = &self.mesh;
        let st = self.structure();
        let rule = elem::rule4();
        let mut k = assemble_scalar(mesh, &st, &|ei| {
            let x = element_coords(mesh, &mesh.elems[ei]);
            let kk = self.k_of(ei);
            let mut ke = vec![0.0; 100];
            for (l, w) in rule.points.iter().zip(&rule.weights) {
                let (g, det) = elem::grad(&x, l);
                let wd = w * det.abs() * kk;
                for i in 0..10 {
                    for j in 0..10 {
                        ke[i * 10 + j] +=
                            wd * (g[i][0] * g[j][0] + g[i][1] * g[j][1] + g[i][2] * g[j][2]);
                    }
                }
            }
            ke
        });
        apply_penalty(&mut k, 1, &self.ties, self.tie_penalty);
        let mut f = vec![0.0; mesh.nodes.len()];
        // volumetric generation: ∫ N_i Q dV
        let duffy = elem::duffy(3);
        for (ei, e) in mesh.elems.iter().enumerate() {
            let q = self.q_of(ei);
            if q == 0.0 {
                continue;
            }
            let x = element_coords(mesh, e);
            for (l, w) in duffy.points.iter().zip(&duffy.weights) {
                let (_, det) = elem::grad(&x, l);
                let n = elem::shape(l);
                for i in 0..10 {
                    f[e[i] as usize] += w * det.abs() * q * n[i];
                }
            }
        }
        // faces: flux into the body, and convection h (T - T_ambient)
        for s in &mesh.surface {
            let flux: f64 = self
                .flux
                .iter()
                .filter(|(t, _)| *t == s.tag)
                .map(|(_, q)| q)
                .sum();
            let conv: Vec<&(u32, f64, f64)> = self
                .convection
                .iter()
                .filter(|(t, _, _)| *t == s.tag)
                .collect();
            if flux == 0.0 && conv.is_empty() {
                continue;
            }
            let x: Vec<P3> = s.v.iter().map(|n| mesh.nodes[*n as usize]).collect();
            for (l, w) in TRI_RULE {
                let n = tri6_shape(l);
                let area = face_area_element(&x, l);
                for i in 0..6 {
                    f[s.v[i] as usize] += 0.5 * w * area * n[i] * flux;
                }
                for (_, h, t_inf) in &conv {
                    for i in 0..6 {
                        f[s.v[i] as usize] += 0.5 * w * area * n[i] * h * t_inf;
                        for j in 0..6 {
                            k.add(
                                s.v[i] as usize,
                                s.v[j] as usize,
                                0.5 * w * area * n[i] * n[j] * h,
                            );
                        }
                    }
                }
            }
        }
        (k, f)
    }

    /// The temperature of every node.
    pub fn solve(&self, kind: SolverKind) -> Result<Vec<f64>, String> {
        if self.fixed.is_empty() && self.convection.is_empty() {
            return Err(
                "a thermal study needs a prescribed temperature or a convection face to be determinate"
                    .into(),
            );
        }
        let (mut k, mut f) = self.assemble();
        let fixed: Vec<(usize, f64)> = self.fixed.iter().map(|(n, t)| (*n as usize, *t)).collect();
        k.apply_dirichlet(&mut f, &fixed);
        match kind {
            SolverKind::Direct => solve_direct(&k, &f),
            SolverKind::Pcg(_) => {
                let (t, rep) = pcg(&k, &f, Precond::Jacobi, 1e-10, 200_000, None);
                if rep.converged {
                    Ok(t)
                } else {
                    Err(format!(
                        "the conjugate gradient solver stopped at {:.2e}",
                        rep.residual
                    ))
                }
            }
        }
    }
}

/// Barycentric points and weights (summing to one) of the 7-point degree-5 rule on a triangle.
const TRI_RULE: [([f64; 3], f64); 7] = {
    // a = (6 + √15)/21, b = (9 − 2√15)/21, c = (6 − √15)/21, d = (9 + 2√15)/21
    const S15: f64 = 3.872_983_346_207_417;
    const A: f64 = (6.0 + S15) / 21.0;
    const B: f64 = (9.0 - 2.0 * S15) / 21.0;
    const C: f64 = (6.0 - S15) / 21.0;
    const D: f64 = (9.0 + 2.0 * S15) / 21.0;
    const W1: f64 = (155.0 + S15) / 1200.0;
    const W2: f64 = (155.0 - S15) / 1200.0;
    [
        ([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0], 9.0 / 40.0),
        ([A, A, B], W1),
        ([A, B, A], W1),
        ([B, A, A], W1),
        ([C, C, D], W2),
        ([C, D, C], W2),
        ([D, C, C], W2),
    ]
};

/// `|∂x/∂ξ × ∂x/∂η|` of a six-node face at barycentric coordinates `l` (twice the area of the reference
/// triangle's image, so an integral over the face is `½ Σ w f |…|`).
fn face_area_element(x: &[P3], l: [f64; 3]) -> f64 {
    let (l1, l2, l3) = (l[0], l[1], l[2]);
    let d_l = [
        [4.0 * l1 - 1.0, 0.0, 0.0],
        [0.0, 4.0 * l2 - 1.0, 0.0],
        [0.0, 0.0, 4.0 * l3 - 1.0],
        [4.0 * l2, 4.0 * l1, 0.0],
        [0.0, 4.0 * l3, 4.0 * l2],
        [4.0 * l3, 0.0, 4.0 * l1],
    ];
    let (mut tx, mut ty) = ([0.0; 3], [0.0; 3]);
    for i in 0..6 {
        for a in 0..3 {
            tx[a] += x[i][a] * (d_l[i][1] - d_l[i][0]);
            ty[a] += x[i][a] * (d_l[i][2] - d_l[i][0]);
        }
    }
    let n = [
        tx[1] * ty[2] - tx[2] * ty[1],
        tx[2] * ty[0] - tx[0] * ty[2],
        tx[0] * ty[1] - tx[1] * ty[0],
    ];
    (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
}
