//! Subdivision surfaces: Catmull–Clark with creases, and *Finish*, which turns the limit surface into a
//! B-rep of bicubic patches.
//!
//! * [`SubdMesh`] is the control cage (polygons of any size, optional semi-sharp creases).
//! * [`Level`] subdivides it (Catmull–Clark with the DeRose crease rules) and gives exact limit points.
//! * [`finish`] subdivides until the patches are small enough, fits one bicubic Bézier patch per face of
//!   that level to exact limit points (regular patches are reproduced exactly, patches around
//!   extraordinary vertices are fitted), shares one cubic edge curve between neighbours, measures the
//!   deviation from the limit surface and the tangent-plane continuity (G1) across smooth edges, and builds
//!   a validated closed B-rep.

// Numerical code indexes small fixed-size arrays; `!(x > 0.0)` is deliberate (it also rejects NaN).
#![allow(
    clippy::needless_range_loop,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod finish;
mod level;
mod mesh;
mod preview;

pub use finish::{finish, FinishOptions, Finished, G1Report};
pub use level::Level;
pub use mesh::{Crease, SubdError, SubdMesh};
pub use preview::{fast_mesh, levels_for, PreviewMesh};
