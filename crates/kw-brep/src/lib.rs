//! B-rep topology store: ACIS-style hierarchy with coedges and a radial ring
//! per edge (manifold + non-manifold). Geometry lives in separate arenas.
//! See docs/01-research/01-brep-kernel.md.

// The algorithms follow the index notation of the cited literature (Piegl & Tiller etc.);
// rewriting those loops as iterators would obscure the correspondence.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments)]

pub mod builders;
pub mod history;
pub mod ids;
pub mod mutate;
pub mod store;
pub mod topology;
pub mod validate;

pub use builders::{make_box, make_cone, make_cylinder, make_sphere, make_torus, BuildResult};
pub use history::{EntityRef, Origin, ShapeHistory};
pub use ids::*;
pub use store::{BRep, CoedgeSpec};
pub use topology::*;
pub use validate::{ValidationError, ValidationReport};
