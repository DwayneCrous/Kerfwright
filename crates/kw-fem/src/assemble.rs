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
#[derive(Debug, Clone)]
pub struct Pattern {
    pub nbr_ptr: Vec<usize>,
    pub nbr: Vec<u32>,
}

impl Pattern {
    /// Make every pair of nodes in `group` neighbours (for the couplings a tie constraint adds).
    pub fn add_couplings(&mut self, groups: &[Vec<u32>]) {
        let n = self.nbr_ptr.len() - 1;
        let mut extra: Vec<Vec<u32>> = vec![Vec::new(); n];
        for g in groups {
            for &a in g {
                extra[a as usize].extend_from_slice(g);
            }
        }
        let mut nbr_ptr = Vec::with_capacity(n + 1);
        nbr_ptr.push(0);
        let mut nbr = Vec::with_capacity(self.nbr.len());
        for i in 0..n {
            let mut row: Vec<u32> = self.nbr[self.nbr_ptr[i]..self.nbr_ptr[i + 1]].to_vec();
            row.extend_from_slice(&extra[i]);
            row.sort_unstable();
            row.dedup();
            nbr.extend_from_slice(&row);
            nbr_ptr.push(nbr.len());
        }
        self.nbr_ptr = nbr_ptr;
        self.nbr = nbr;
    }
}

/// The sparsity pattern and element colouring of a mesh, built once and reused by every assembly (stiffness,
/// mass, a Newton iteration's tangent).
pub struct Structure {
    pub pattern: Pattern,
    pub colours: Vec<Vec<u32>>,
}

impl Structure {
    pub fn new(mesh: &Tet10Mesh) -> Structure {
        Structure {
            pattern: build_pattern(mesh.nodes.len(), &mesh.elems),
            colours: colour_elements(mesh.nodes.len(), &mesh.elems),
        }
    }

    /// As [`new`](Self::new), also coupling the nodes of each group (tie constraints).
    pub fn with_couplings(mesh: &Tet10Mesh, groups: &[Vec<u32>]) -> Structure {
        let mut s = Structure::new(mesh);
        s.pattern.add_couplings(groups);
        s
    }
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

/// Greedy element colouring. A node with many elements around it (a vertex where thousands of slivers meet
/// is rare, but a badly graded mesh has some) needs as many colours as it has elements, so the colour sets
/// are kept in as many 128-bit words as the mesh turns out to need.
pub fn colour_elements(nnodes: usize, elems: &[[u32; 10]]) -> Vec<Vec<u32>> {
    let mut words = 1usize;
    loop {
        if let Some(c) = try_colour(nnodes, elems, words) {
            return c;
        }
        words *= 2;
    }
}

fn try_colour(nnodes: usize, elems: &[[u32; 10]], words: usize) -> Option<Vec<Vec<u32>>> {
    let mut node_mask = vec![0u128; nnodes * words];
    let mut colours: Vec<Vec<u32>> = Vec::new();
    let mut used = vec![0u128; words];
    for (ei, e) in elems.iter().enumerate() {
        used.iter_mut().for_each(|u| *u = 0);
        for &n in e {
            for (w, u) in used.iter_mut().enumerate() {
                *u |= node_mask[n as usize * words + w];
            }
        }
        let c = used.iter().enumerate().find_map(|(w, u)| {
            (*u != u128::MAX).then(|| w * 128 + (!*u).trailing_zeros() as usize)
        })?;
        for &n in e {
            node_mask[n as usize * words + c / 128] |= 1u128 << (c % 128);
        }
        if colours.len() <= c {
            colours.resize(c + 1, Vec::new());
        }
        colours[c].push(ei as u32);
    }
    Some(colours)
}

struct SyncPtr(*mut f64);
unsafe impl Send for SyncPtr {}
unsafe impl Sync for SyncPtr {}

/// An all-zero matrix with the pattern of `p` and `nd` unknowns per node.
pub fn empty_csr(nnodes: usize, p: &Pattern, nd: usize) -> Csr {
    let n = nd * nnodes;
    let mut row_ptr = Vec::with_capacity(n + 1);
    row_ptr.push(0usize);
    let mut col: Vec<u32> = Vec::with_capacity(nd * nd * p.nbr.len());
    for i in 0..nnodes {
        let nb = &p.nbr[p.nbr_ptr[i]..p.nbr_ptr[i + 1]];
        for _c in 0..nd {
            for &j in nb {
                for cb in 0..nd {
                    col.push(nd as u32 * j + cb as u32);
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

/// Add the element matrices (`10 nd` square, row-major) into `a`, element groups of one colour in parallel.
pub fn scatter(
    a: &mut Csr,
    p: &Pattern,
    colours: &[Vec<u32>],
    elems: &[[u32; 10]],
    nd: usize,
    element_matrix: &(dyn Fn(usize) -> Vec<f64> + Sync),
) {
    let ptr = SyncPtr(a.val.as_mut_ptr());
    let ptr = &ptr;
    let row_ptr = &a.row_ptr;
    let width = 10 * nd;
    for group in colours {
        group.par_iter().for_each(|&ei| {
            let e = &elems[ei as usize];
            let ke = element_matrix(ei as usize);
            // position of every (local node a, local node b) block start within the row of node a
            for la in 0..10 {
                let ni = e[la] as usize;
                let nb = &p.nbr[p.nbr_ptr[ni]..p.nbr_ptr[ni + 1]];
                for lb in 0..10 {
                    let nj = e[lb];
                    let rank = nb.binary_search(&nj).expect("pattern contains the edge");
                    for ca in 0..nd {
                        let row = nd * ni + ca;
                        let base = row_ptr[row] + nd * rank;
                        for cb in 0..nd {
                            // SAFETY: elements of one colour share no node, hence write disjoint rows.
                            unsafe {
                                *ptr.0.add(base + cb) += ke[(nd * la + ca) * width + nd * lb + cb];
                            }
                        }
                    }
                }
            }
        });
    }
}

pub fn element_coords(mesh: &Tet10Mesh, e: &[u32; 10]) -> [P3; 10] {
    let mut x = [[0.0; 3]; 10];
    for i in 0..10 {
        x[i] = mesh.nodes[e[i] as usize];
    }
    x
}

/// Stiffness matrix of the whole mesh (no boundary conditions applied).
pub fn assemble_stiffness(mesh: &Tet10Mesh, e: f64, nu: f64) -> (Csr, Pattern) {
    let st = Structure::new(mesh);
    let d = elem::elasticity(e, nu);
    let a = assemble_stiffness_with(mesh, &st, &|_| d);
    (a, st.pattern)
}

/// Stiffness matrix with the elasticity matrix `d_of(element)` (a different material per element).
pub fn assemble_stiffness_with(
    mesh: &Tet10Mesh,
    st: &Structure,
    d_of: &(dyn Fn(usize) -> [[f64; 6]; 6] + Sync),
) -> Csr {
    let mut a = empty_csr(mesh.nodes.len(), &st.pattern, 3);
    let rule: Rule = elem::rule4();
    scatter(&mut a, &st.pattern, &st.colours, &mesh.elems, 3, &|ei| {
        let x = element_coords(mesh, &mesh.elems[ei]);
        elem::stiffness(&x, &d_of(ei), &rule)
    });
    a
}

/// Consistent mass matrix with the pattern of `p`.
pub fn assemble_mass(mesh: &Tet10Mesh, rho: f64, p: &Pattern) -> Csr {
    let colours = colour_elements(mesh.nodes.len(), &mesh.elems);
    let st = Structure {
        pattern: p.clone(),
        colours,
    };
    assemble_mass_with(mesh, &st, &|_| rho)
}

/// Consistent mass matrix with the density `rho_of(element)`.
pub fn assemble_mass_with(
    mesh: &Tet10Mesh,
    st: &Structure,
    rho_of: &(dyn Fn(usize) -> f64 + Sync),
) -> Csr {
    let mut a = empty_csr(mesh.nodes.len(), &st.pattern, 3);
    let rule = elem::duffy(4);
    scatter(&mut a, &st.pattern, &st.colours, &mesh.elems, 3, &|ei| {
        let x = element_coords(mesh, &mesh.elems[ei]);
        elem::mass(&x, rho_of(ei), &rule)
    });
    a
}

/// A matrix with one unknown per node from the `10 x 10` element matrices of `element_matrix` (thermal
/// conduction, a scalar mass).
pub fn assemble_scalar(
    mesh: &Tet10Mesh,
    st: &Structure,
    element_matrix: &(dyn Fn(usize) -> Vec<f64> + Sync),
) -> Csr {
    let mut a = empty_csr(mesh.nodes.len(), &st.pattern, 1);
    scatter(
        &mut a,
        &st.pattern,
        &st.colours,
        &mesh.elems,
        1,
        element_matrix,
    );
    a
}
