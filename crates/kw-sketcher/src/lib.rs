//! Interactive 2-D sketcher built on `kw-sketch` (solver + [`Editor`](kw_sketch::editor::Editor))
//! and `kw-dxf`, with an egui front end.
//!
//! All sketching *logic* lives in `kw-sketch` and is tested without a GUI; this crate only
//! translates egui input into [`Input`](kw_sketch::editor::Input) events and draws the result.

mod app;
mod view;

pub use app::SketcherApp;
pub use view::View;
