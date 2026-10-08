//! B-rep topology store: ACIS-style hierarchy with coedges and a radial ring
//! per edge (manifold + non-manifold). Geometry lives in separate arenas.
//! See docs/01-research/01-brep-kernel.md.

pub mod history;
pub mod ids;
pub mod store;
pub mod topology;

pub use history::{EntityRef, Origin, ShapeHistory};
pub use ids::*;
pub use store::{BRep, ValidationError};
pub use topology::*;
