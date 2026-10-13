//! B-rep topology store: ACIS-style hierarchy with coedges and a radial ring
//! per edge (manifold + non-manifold). Geometry lives in separate arenas.
//! See docs/01-research/01-brep-kernel.md.

// The algorithms follow the index notation of the cited literature (Piegl & Tiller etc.);
// rewriting those loops as iterators would obscure the correspondence.
#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod builders;
pub mod classify;
pub mod copy;
pub mod history;
pub mod ids;
pub mod mass;
pub mod mutate;
pub mod sew;
pub mod store;
pub mod topology;
pub mod uvgeom;
pub mod validate;

pub use builders::{
    add_face_auto, make_box, make_cone, make_cylinder, make_sphere, make_torus, BuildResult,
    LoopSpec,
};
pub use classify::{classify_point, ClassifyError, PointClass, PreparedFace, SolidIndex};
pub use copy::{CopyMap, CopyOptions};
pub use history::{EntityRef, Origin, Role, ShapeHistory};
pub use ids::*;
pub use mass::MassProps;
pub use store::{BRep, CoedgeSpec};
pub use topology::*;
pub use validate::{ValidationError, ValidationReport};
