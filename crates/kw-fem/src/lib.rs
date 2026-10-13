//! Linear finite elements: quadratic tetrahedra (TET10), parallel assembly into CSR, boundary conditions,
//! a sparse direct solver (faer Cholesky) and a preconditioned conjugate-gradient solver, stress recovery
//! and modal analysis.

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod assemble;
pub mod elem;
pub mod modal;
pub mod model;
pub mod nonlinear;
pub mod reference;
pub mod simp;
pub mod solve;
pub mod sparse;
pub mod stress;
pub mod thermal;
pub mod tie;

pub use assemble::{assemble_mass, assemble_stiffness, Structure};
pub use model::{Fixed, Material, Model, Solution, SolverKind, ThermalLoad, TractionFn};
pub use solve::{pcg, solve_direct, Factor, PcgReport, Precond};
pub use sparse::Csr;
pub use thermal::ThermalModel;
