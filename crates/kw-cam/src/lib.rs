//! Computer-aided manufacturing: 2.5-D toolpaths (contour, pocket, drill, adaptive clearing), 3-D finishing
//! (drop-cutter), tri-dexel stock simulation, five-axis kinematics, and the post-processor that turns
//! cutter-location events into the G-code of a controller.

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod adaptive;
pub mod bvh;
pub mod dexel;
pub mod dropcutter;
pub mod finish3d;
pub mod geom2d;
pub mod kin;
pub mod multiaxis;
pub mod ops2d;
pub mod path;
pub mod post;
pub mod suite;
pub mod suite3d;
pub mod tool;
pub mod verify;

pub use geom2d::Region;
pub use path::{Event, Toolpath};
pub use tool::{Holder, Tool, ToolKind};

pub type P2 = [f64; 2];
pub type P3 = [f64; 3];
