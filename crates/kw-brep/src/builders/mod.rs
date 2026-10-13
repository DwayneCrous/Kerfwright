//! Primitive solid builders. Each creates real topology (seams, poles,
//! degenerate edges) with exact analytic geometry and per-coedge pcurves,
//! and records `Origin::New` in the returned [`ShapeHistory`].
//!
//! Topology models (V, E = non-degenerate edges, F, R, S, genus):
//!
//! | primitive | V | E | F | R | S | genus |
//! |---|---|---|---|---|---|---|
//! | box | 8 | 12 | 6 | 0 | 1 | 0 |
//! | cylinder | 2 | 3 | 3 | 0 | 1 | 0 |
//! | cone (frustum) | 2 | 3 | 3 | 0 | 1 | 0 |
//! | cone (apex) | 2 | 2 | 2 | 0 | 1 | 0 |
//! | sphere | 2 | 1 | 1 | 0 | 1 | 0 |
//! | torus | 1 | 2 | 1 | 0 | 1 | 1 |

mod auto;
mod primitives;

pub use auto::{add_face_auto, LoopSpec};
pub use primitives::{
    make_box, make_cone, make_cylinder, make_plane_sheet, make_sphere, make_torus,
};

use crate::history::ShapeHistory;
use crate::ids::BodyId;

/// Result of a primitive builder.
#[derive(Debug, Clone)]
pub struct BuildResult {
    pub body: BodyId,
    pub history: ShapeHistory,
}
