//! ASCII DXF (AC1015 / R2000) reader and writer for 2-D geometry, plus
//! adapters to and from [`kw_sketch::Sketch`].
//!
//! Supported entities: `POINT`, `LINE`, `CIRCLE`, `ARC`, `ELLIPSE`,
//! `LWPOLYLINE` (with bulges), `SPLINE`, `TEXT`. Everything else is skipped on read.
//! Doubles are written with Rust's shortest round-trip formatting, so a
//! write → read cycle is bit-exact.

mod format;
mod sketch;

pub use format::{parse, write, write_with_layers, DxfDoc, DxfEntity, DxfError, LayerDef};
pub use sketch::{export_sketch, import_sketch, ImportOptions};
