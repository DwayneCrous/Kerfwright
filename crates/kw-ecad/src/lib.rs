//! Printed circuit boards: the board model, footprints, placement, a grid router, design-rule checking
//! and fabrication outputs (Gerber RS-274X, Excellon, IDF 3.0, STEP, pick-and-place).

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod bench_gen;
pub mod board;
pub mod drc;
pub mod export;
pub mod footprints;
pub mod place;
pub mod pour;
pub mod route;
pub mod shape;

pub use board::{Board, Footprint, Pad, PadShape, Part, Rules, Side, Track, Via, Zone};
pub use drc::{drc, open_nets, Violation, ViolationKind};
pub use shape::{Item, ItemKind, Shape};

pub type P2 = [f64; 2];
