//! Stress recovery: the stress of the displacement solution is discontinuous between elements; its global
//! L2 projection onto the continuous quadratic space gives smooth nodal values (Hinton & Campbell).

use crate::assemble::{assemble_mass, build_pattern};
use crate::elem;
use crate::model::Model;
use crate::solve::{pcg, Precond};
use kw_mesh3d::P3;

/// Voigt stress `[xx, yy, zz, yz, zx, xy]` at every node: the L2 projection of the element stresses.
pub fn nodal_stress(model: &Model, u: &[f64]) -> Vec<[f64; 6]> {
    let mesh = &model.mesh;
    let n = mesh.nodes.len();
    let table = model.elasticity_table();
    let rule = elem::duffy(3);
    // right-hand sides: b[node][component] = ∫ N_i σ_c dV
    let mut b = vec![[0.0f64; 6]; n];
    for (ei, e) in mesh.elems.iter().enumerate() {
        let d = model.elasticity_of(&table, ei);
        let mut x = [[0.0; 3]; 10];
        let mut ue = [0.0; 30];
        for i in 0..10 {
            x[i] = mesh.nodes[e[i] as usize];
            for a in 0..3 {
                ue[3 * i + a] = u[3 * e[i] as usize + a];
            }
        }
        for (l, w) in rule.points.iter().zip(&rule.weights) {
            let (_, det) = elem::grad(&x, l);
            let s = elem::stress_at(&x, &ue, &d, l);
            let nn = elem::shape(l);
            let wd = w * det.abs();
            for i in 0..10 {
                let bi = &mut b[e[i] as usize];
                for c in 0..6 {
                    bi[c] += wd * nn[i] * s[c];
                }
            }
        }
    }
    // the vector mass matrix is M ⊗ I₃, so one solve handles three components
    let pattern = build_pattern(n, &mesh.elems);
    let m = assemble_mass(mesh, 1.0, &pattern);
    let mut out = vec![[0.0f64; 6]; n];
    for half in 0..2 {
        let mut rhs = vec![0.0; 3 * n];
        for i in 0..n {
            for a in 0..3 {
                rhs[3 * i + a] = b[i][3 * half + a];
            }
        }
        let (sol, rep) = pcg(&m, &rhs, Precond::Jacobi, 1e-12, 2000, None);
        debug_assert!(rep.converged || rep.residual < 1e-9);
        for i in 0..n {
            for a in 0..3 {
                out[i][3 * half + a] = sol[3 * i + a];
            }
        }
    }
    out
}

/// Von Mises equivalent stress of nodal Voigt stresses.
pub fn von_mises_nodal(s: &[[f64; 6]]) -> Vec<f64> {
    s.iter().map(elem::von_mises).collect()
}

/// Stress at an arbitrary point of element `ei` with barycentric coordinates `l`, interpolated from the
/// projected nodal values.
pub fn interpolate(model: &Model, nodal: &[[f64; 6]], ei: usize, l: &[f64; 4]) -> [f64; 6] {
    let e = &model.mesh.elems[ei];
    let n = elem::shape(l);
    let mut s = [0.0; 6];
    for i in 0..10 {
        for c in 0..6 {
            s[c] += n[i] * nodal[e[i] as usize][c];
        }
    }
    s
}

/// Position of node `i`.
pub fn node(model: &Model, i: usize) -> P3 {
    model.mesh.nodes[i]
}
