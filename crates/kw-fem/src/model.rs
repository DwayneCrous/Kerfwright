use crate::assemble::{assemble_mass_with, assemble_stiffness_with, Pattern, Structure};
use crate::elem;
use crate::solve::{pcg, solve_direct, PcgReport, Precond};
use crate::sparse::Csr;
use crate::tie::{apply_penalty, tie_groups, Tie};
use kw_mesh3d::{Tet10Mesh, P3};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Instant;

/// A traction (force per unit area) that depends on the position and the outward unit normal.
#[derive(Clone)]
pub struct TractionFn(pub Arc<dyn Fn(P3, P3) -> P3 + Send + Sync>);

impl std::fmt::Debug for TractionFn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TractionFn")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// Young's modulus.
    pub e: f64,
    pub nu: f64,
    /// Density.
    pub rho: f64,
}

/// A prescribed displacement component.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fixed {
    pub node: u32,
    pub axis: usize,
    pub value: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverKind {
    Direct,
    Pcg(Precond),
}

#[derive(Debug, Clone)]
pub struct Model {
    pub mesh: Tet10Mesh,
    pub material: Material,
    pub fixed: Vec<Fixed>,
    /// Point loads on nodes.
    pub forces: Vec<(u32, P3)>,
    /// Pressure (positive = pushing into the body) on the boundary triangles with the given tag.
    pub pressure: Vec<(u32, f64)>,
    /// Traction vector per unit area on the boundary triangles with the given tag.
    pub traction: Vec<(u32, P3)>,
    /// Position- and normal-dependent traction on the boundary triangles with the given tag.
    pub traction_fn: Vec<(u32, TractionFn)>,
    /// Gravity acceleration (body force `ρ g`).
    pub gravity: Option<P3>,
    /// The materials of a model with several (one per body). Empty: `material` everywhere.
    pub materials: Vec<Material>,
    /// Index into `materials` for every element (empty: all elements are `material`).
    pub elem_material: Vec<u16>,
    /// Bonded contacts between bodies (see [`crate::tie`]).
    pub ties: Vec<Tie>,
    /// Penalty stiffness of the ties relative to the largest diagonal entry of the tied nodes.
    pub tie_penalty: f64,
}

/// Default relative penalty of the ties: holds tied nodes together to about one part in 1e5 while keeping
/// the matrix well enough conditioned for the direct solver.
pub const DEFAULT_TIE_PENALTY: f64 = 1e5;

#[derive(Debug, Clone)]
pub struct Timings {
    pub assembly: f64,
    pub bcs: f64,
    pub solve: f64,
}

#[derive(Debug, Clone)]
pub struct Solution {
    /// Nodal displacements, 3 per node.
    pub u: Vec<f64>,
    pub timings: Timings,
    pub pcg: Option<PcgReport>,
}

impl Model {
    pub fn new(mesh: Tet10Mesh, material: Material) -> Model {
        Model {
            mesh,
            material,
            fixed: Vec::new(),
            forces: Vec::new(),
            pressure: Vec::new(),
            traction: Vec::new(),
            traction_fn: Vec::new(),
            gravity: None,
            materials: Vec::new(),
            elem_material: Vec::new(),
            ties: Vec::new(),
            tie_penalty: DEFAULT_TIE_PENALTY,
        }
    }

    /// The material of element `ei`.
    pub fn material_of(&self, ei: usize) -> &Material {
        match self.elem_material.get(ei) {
            Some(&k) if !self.materials.is_empty() => &self.materials[k as usize],
            _ => &self.material,
        }
    }

    /// The elasticity matrix of every distinct material (indexed like `materials`, or just `material`).
    pub fn elasticity_table(&self) -> Vec<[[f64; 6]; 6]> {
        if self.materials.is_empty() {
            vec![elem::elasticity(self.material.e, self.material.nu)]
        } else {
            self.materials
                .iter()
                .map(|m| elem::elasticity(m.e, m.nu))
                .collect()
        }
    }

    /// The elasticity matrix of element `ei` from a table made by [`elasticity_table`](Self::elasticity_table).
    pub fn elasticity_of(&self, table: &[[[f64; 6]; 6]], ei: usize) -> [[f64; 6]; 6] {
        match self.elem_material.get(ei) {
            Some(&k) if table.len() > 1 || !self.materials.is_empty() => table[k as usize],
            _ => table[0],
        }
    }

    /// The sparsity pattern and colouring, widened by the ties.
    pub fn structure(&self) -> Structure {
        if self.ties.is_empty() {
            Structure::new(&self.mesh)
        } else {
            Structure::with_couplings(&self.mesh, &tie_groups(&self.ties))
        }
    }

    /// Stiffness matrix of the whole model with the ties in (no boundary conditions applied).
    pub fn stiffness(&self, st: &Structure) -> Csr {
        let table = self.elasticity_table();
        let mut k = assemble_stiffness_with(&self.mesh, st, &|ei| self.elasticity_of(&table, ei));
        apply_penalty(&mut k, 3, &self.ties, self.tie_penalty);
        k
    }

    /// Consistent mass matrix of the whole model.
    pub fn mass_matrix(&self, st: &Structure) -> Csr {
        assemble_mass_with(&self.mesh, st, &|ei| self.material_of(ei).rho)
    }

    /// Fix the listed components of every node for which `pred` holds.
    pub fn fix_where(&mut self, axes: [bool; 3], pred: &dyn Fn(P3) -> bool) {
        for (i, p) in self.mesh.nodes.iter().enumerate() {
            if pred(*p) {
                for a in 0..3 {
                    if axes[a] {
                        self.fixed.push(Fixed {
                            node: i as u32,
                            axis: a,
                            value: 0.0,
                        });
                    }
                }
            }
        }
    }

    /// Load vector from nodal forces, boundary pressure/traction and gravity.
    pub fn load_vector(&self) -> Vec<f64> {
        let mesh = &self.mesh;
        let mut f = vec![0.0; 3 * mesh.nodes.len()];
        for (n, v) in &self.forces {
            for a in 0..3 {
                f[3 * *n as usize + a] += v[a];
            }
        }
        // surface loads: 6-node triangle, 3-point rule at the edge midpoints (exact for the quadratic
        // shape functions times a constant Jacobian, and good for curved faces)
        let pts: [[f64; 3]; 3] = [[0.5, 0.5, 0.0], [0.0, 0.5, 0.5], [0.5, 0.0, 0.5]]; // barycentric
        for s in &mesh.surface {
            let tfn = self
                .traction_fn
                .iter()
                .find(|(t, _)| *t == s.tag)
                .map(|(_, f)| f);
            let load: Option<(P3, bool)> = self
                .pressure
                .iter()
                .find(|(t, _)| *t == s.tag)
                .map(|(_, p)| ([*p, 0.0, 0.0], true))
                .or_else(|| {
                    self.traction
                        .iter()
                        .find(|(t, _)| *t == s.tag)
                        .map(|(_, t)| (*t, false))
                })
                .or_else(|| tfn.map(|_| ([0.0; 3], false)));
            let Some((val, is_pressure)) = load else {
                continue;
            };
            let x: Vec<P3> = s.v.iter().map(|n| mesh.nodes[*n as usize]).collect();
            for l in pts {
                // quadratic triangle shape functions and derivatives in (ξ=L2, η=L3)
                let (l1, l2, l3) = (l[0], l[1], l[2]);
                let n = [
                    l1 * (2.0 * l1 - 1.0),
                    l2 * (2.0 * l2 - 1.0),
                    l3 * (2.0 * l3 - 1.0),
                    4.0 * l1 * l2,
                    4.0 * l2 * l3,
                    4.0 * l3 * l1,
                ];
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
                // outward normal times area element (reference triangle area 1/2, weights 1/6 each)
                let nrm = [
                    tx[1] * ty[2] - tx[2] * ty[1],
                    tx[2] * ty[0] - tx[0] * ty[2],
                    tx[0] * ty[1] - tx[1] * ty[0],
                ];
                let w = 1.0 / 6.0;
                let area2 = (nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]).sqrt();
                let t_here = match tfn {
                    Some(TractionFn(g)) if area2 > 0.0 => {
                        let mut xq = [0.0; 3];
                        for i in 0..6 {
                            for a in 0..3 {
                                xq[a] += n[i] * x[i][a];
                            }
                        }
                        g(xq, [nrm[0] / area2, nrm[1] / area2, nrm[2] / area2])
                    }
                    _ => val,
                };
                for i in 0..6 {
                    for a in 0..3 {
                        let q = if is_pressure {
                            -val[0] * nrm[a]
                        } else {
                            t_here[a] * area2
                        };
                        f[3 * s.v[i] as usize + a] += w * n[i] * q;
                    }
                }
            }
        }
        if let Some(g) = self.gravity {
            let rule = elem::duffy(3);
            for (ei, e) in mesh.elems.iter().enumerate() {
                let rho = self.material_of(ei).rho;
                let mut x = [[0.0; 3]; 10];
                for i in 0..10 {
                    x[i] = mesh.nodes[e[i] as usize];
                }
                for (l, w) in rule.points.iter().zip(&rule.weights) {
                    let (_, det) = elem::grad(&x, l);
                    let n = elem::shape(l);
                    for i in 0..10 {
                        for a in 0..3 {
                            f[3 * e[i] as usize + a] +=
                                w * det.abs() * rho * g[a] * n[i];
                        }
                    }
                }
            }
        }
        f
    }

    pub fn assemble(&self) -> (Csr, Pattern, Vec<f64>) {
        let st = self.structure();
        let k = self.stiffness(&st);
        (k, st.pattern, self.load_vector())
    }

    pub fn solve(&self, kind: SolverKind) -> Result<Solution, String> {
        self.solve_with(kind, None)
    }

    /// As [`solve`](Self::solve), stopping early with `Err("cancelled")` when the flag is raised (the
    /// iterative solver checks it every iteration).
    pub fn solve_with(
        &self,
        kind: SolverKind,
        cancel: Option<&kw_job::CancelFlag>,
    ) -> Result<Solution, String> {
        let t0 = Instant::now();
        let (mut k, _p, mut f) = self.assemble();
        let t1 = Instant::now();
        let fixed: Vec<(usize, f64)> = self
            .fixed
            .iter()
            .map(|c| (3 * c.node as usize + c.axis, c.value))
            .collect();
        k.apply_dirichlet(&mut f, &fixed);
        let t2 = Instant::now();
        let (u, rep) = match kind {
            SolverKind::Direct => (solve_direct(&k, &f)?, None),
            SolverKind::Pcg(pre) => {
                // stop a little early: the recurrence residual drifts from the true one by rounding, and the
                // contract is ‖b − A x‖/‖b‖ ≤ 1e-8
                let (u, r) = pcg(&k, &f, pre, 9e-9, 200_000, cancel);
                if cancel.is_some_and(|c| c.is_cancelled()) {
                    return Err("cancelled".into());
                }
                (u, Some(r))
            }
        };
        let t3 = Instant::now();
        Ok(Solution {
            u,
            timings: Timings {
                assembly: (t1 - t0).as_secs_f64(),
                bcs: (t2 - t1).as_secs_f64(),
                solve: (t3 - t2).as_secs_f64(),
            },
            pcg: rep,
        })
    }
}
