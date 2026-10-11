//! Parallel assembly of element matrices into a CSR matrix.
//!
//! The sparsity pattern is built once from the node connectivity (3 scalar rows per node share one
//! column pattern). Elements are greedily coloured so that two elements of a colour share no node; the
//! elements of one colour then write to disjoint rows and can scatter in parallel without locks.

use crate::elem::{self, Rule};
use crate::sparse::Csr;
use kw_mesh3d::{Tet10Mesh, P3};
use rayon::prelude::*;

/// Block pattern: for node `i`, neighbouring nodes `nbr[nbr_ptr[i]..nbr_ptr[i+1]]` (sorted, with `i`).
pub struct Pattern {
    pub nbr_ptr: Vec<usize>,
    pub nbr: Vec<u32>,
}

pub fn build_pattern(nnodes: usize, elems: &[[u32; 10]]) -> Pattern {
    // node → incident elements, then neighbours by merging
    let mut counts = vec![0usize; nnodes + 1];
    for e in elems {
        for &n in e {
            counts[n as usize + 1] += 1;
        }
    }
    for i in 0..nnodes {
        counts[i + 1] += counts[i];
    }
    let mut inc = vec![0u32; counts[nnodes]];
    let mut fill = counts.clone();
    for (ei, e) in elems.iter().enumerate() {
        for &n in e {
            inc[fill[n as usize]] = ei as u32;
            fill[n as usize] += 1;
        }
    }
    let lists: Vec<Vec<u32>> = (0..nnodes)
        .into_par_iter()
        .map(|i| {
            let mut v: Vec<u32> = Vec::new();
            for &ei in &inc[counts[i]..counts[i + 1]] {
                v.extend_from_slice(&elems[ei as usize]);
            }
            v.sort_unstable();
            v.dedup();
            v
        })
        .collect();
    let mut nbr_ptr = Vec::with_capacity(nnodes + 1);
    nbr_ptr.push(0);
    let mut nbr = Vec::new();
    for l in lists {
        nbr.extend_from_slice(&l);
        nbr_ptr.push(nbr.len());
    }
    Pattern { nbr_ptr, nbr }
}

/// Greedy element colouring (up to 128 colours).
pub fn colour_elements(nnodes: usize, elems: &[[u32; 10]]) -> Vec<Vec<u32>> {
    let mut node_mask = vec![0u128; nnodes];
    let mut colours: Vec<Vec<u32>> = Vec::new();
    for (ei, e) in elems.iter().enumerate() {
        let mut used = 0u128;
        for &n in e {
            used |= node_mask[n as usize];
        }
        let c = (!used).trailing_zeros() as usize;
        assert!(c < 128, "element colouring needs more than 128 colours");
        for &n in e {
            node_mask[n as usize] |= 1u128 << c;
        }
        if colours.len() <= c {
            colours.resize(c + 1, Vec::new());
        }
        colours[c].push(ei as u32);
    }
    colours
}

struct SyncPtr(*mut f64);
unsafe impl Send for SyncPtr {}
unsafe impl Sync for SyncPtr {}

fn empty_csr(nnodes: usize, p: &Pattern) -> Csr {
    let n = 3 * nnodes;
    let mut row_ptr = Vec::with_capacity(n + 1);
    row_ptr.push(0usize);
    let mut col: Vec<u32> = Vec::with_capacity(9 * p.nbr.len());
    for i in 0..nnodes {
        let nb = &p.nbr[p.nbr_ptr[i]..p.nbr_ptr[i + 1]];
        for _c in 0..3 {
            for &j in nb {
                for cb in 0..3 {
                    col.push(3 * j + cb);
                }
            }
            row_ptr.push(col.len());
        }
    }
    let val = vec![0.0; col.len()];
    Csr {
        n,
        row_ptr,
        col,
        val,
    }
}

fn scatter(
    a: &mut Csr,
    p: &Pattern,
    colours: &[Vec<u32>],
    elems: &[[u32; 10]],
    element_matrix: &(dyn Fn(usize) -> Vec<f64> + Sync),
) {
    let ptr = SyncPtr(a.val.as_mut_ptr());
    let ptr = &ptr;
    let row_ptr = &a.row_ptr;
    for group in colours {
        group.par_iter().for_each(|&ei| {
            let e = &elems[ei as usize];
            let ke = element_matrix(ei as usize);
            // position of every (local node a, local node b) block start within the row of node a
            for la in 0..10 {
                let ni = e[la] as usize;
                let nb = &p.nbr[p.nbr_ptr[ni]..p.nbr_ptr[ni + 1]];
                let deg = nb.len();
                for lb in 0..10 {
                    let nj = e[lb];
                    let rank = nb.binary_search(&nj).expect("pattern contains the edge");
                    for ca in 0..3 {
                        let row = 3 * ni + ca;
                        let base = row_ptr[row] + 3 * rank;
                        let _ = deg;
                        for cb in 0..3 {
                            // SAFETY: elements of one colour share no node, hence write disjoint rows.
                            unsafe {
                                *ptr.0.add(base + cb) += ke[(3 * la + ca) * 30 + 3 * lb + cb];
                            }
                        }
                    }
                }
            }
        });
    }
}

fn element_coords(mesh: &Tet10Mesh, e: &[u32; 10]) -> [P3; 10] {
    let mut x = [[0.0; 3]; 10];
    for i in 0..10 {
        x[i] = mesh.nodes[e[i] as usize];
    }
    x
}

/// Stiffness matrix of the whole mesh (no boundary conditions applied).
pub fn assemble_stiffness(mesh: &Tet10Mesh, e: f64, nu: f64) -> (Csr, Pattern) {
    let p = build_pattern(mesh.nodes.len(), &mesh.elems);
    let colours = colour_elements(mesh.nodes.len(), &mesh.elems);
    let mut a = empty_csr(mesh.nodes.len(), &p);
    let d = elem::elasticity(e, nu);
    let rule: Rule = elem::rule4();
    scatter(&mut a, &p, &colours, &mesh.elems, &|ei| {
        let x = element_coords(mesh, &mesh.elems[ei]);
        elem::stiffness(&x, &d, &rule)
    });
    (a, p)
}

/// Consistent mass matrix with the pattern of `p`.
pub fn assemble_mass(mesh: &Tet10Mesh, rho: f64, p: &Pattern) -> Csr {
    let colours = colour_elements(mesh.nodes.len(), &mesh.elems);
    let mut a = empty_csr(mesh.nodes.len(), p);
    let rule = elem::duffy(4);
    scatter(&mut a, p, &colours, &mesh.elems, &|ei| {
        let x = element_coords(mesh, &mesh.elems[ei]);
        elem::mass(&x, rho, &rule)
    });
    a
}
