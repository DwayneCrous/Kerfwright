//! STEP (ISO 10303-21) import and export of B-rep solids.
//!
//! * [`parse`] — a bounded, never-panicking Part 21 parser (header + data section, simple and
//!   complex entity instances).
//! * [`import`] — AP203 / AP214 / AP242 `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS` into a
//!   [`BRep`](kw_brep::BRep): planes, cylinders, cones, spheres, tori, (rational) B-spline surfaces,
//!   surfaces of revolution / linear extrusion; lines, circles, ellipses, (rational) B-spline curves.
//!   Pcurves are computed, loops oriented and the result validated and reported.
//! * [`export`] — writes solids as `MANIFOLD_SOLID_BREP` (AP214-style product structure).

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod export;
mod import;
mod parser;

pub use export::{export, ExportOptions};
pub use import::{import, ImportOptions, ImportReport, StepError};
pub use parser::{parse, Instance, Limits, Param, ParseError, StepFile};
