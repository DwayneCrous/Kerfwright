//! An independent G-code interpreter for checking the output of the post-processors.
//!
//! It knows nothing about `kw-cam`: it reads program *text* for one dialect, rejects what that dialect does
//! not define (unknown words, unknown G/M numbers, malformed numbers, arcs with inconsistent radii,
//! motion before a feed rate or a tool, missing program end), tracks the modal state, and returns the
//! cutter motion it executes. Tests compare that motion with the cutter-location data the program was
//! posted from.
//!
//! It was written from the public descriptions of the dialects (Fanuc/Haas macro-B style codes, the
//! RS-274/NGC language of LinuxCNC, Siemens SINUMERIK 840D ISO/standard cycles, Heidenhain TNC
//! conversational programming) by the same author as the post-processors, so it is an independent
//! *implementation*, not an independent *authority*.

#![allow(clippy::too_many_arguments)]

mod gcode;
mod heidenhain;

pub use gcode::run_gcode;
pub use heidenhain::run_heidenhain;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dialect {
    Fanuc,
    Haas,
    Siemens840D,
    Heidenhain,
    LinuxCnc,
}

pub type P3 = [f64; 3];

#[derive(Debug, Clone, PartialEq)]
pub enum MotionKind {
    Rapid,
    Feed,
    ArcCw,
    ArcCcw,
}

/// One executed motion block.
#[derive(Debug, Clone, PartialEq)]
pub struct Motion {
    pub kind: MotionKind,
    pub from: P3,
    pub to: P3,
    /// Arc centre (XY plane) for arcs.
    pub centre: Option<[f64; 2]>,
    pub feed: f64,
    pub tool: u32,
    pub spindle_rpm: f64,
    pub line: usize,
    /// Rotary axes A, B, C (degrees) at the end of the motion; axes that were never programmed stay 0.
    pub rot: [f64; 3],
    /// Tool-centre-point control (RTCP) active: X Y Z are then tool-tip coordinates in the workpiece frame.
    pub tcp: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SimError {
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for SimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}
impl std::error::Error for SimError {}

#[derive(Debug, Clone, Default)]
pub struct Run {
    pub motions: Vec<Motion>,
    pub tools_used: Vec<u32>,
    pub program_end_seen: bool,
    pub lines: usize,
    /// Drilling cycles executed: (x, y, z_depth, r_plane).
    pub drills: Vec<(f64, f64, f64, f64)>,
}

/// Execute `text` as a program of `dialect`.
pub fn run(dialect: Dialect, text: &str) -> Result<Run, SimError> {
    match dialect {
        Dialect::Heidenhain => run_heidenhain(text),
        d => run_gcode(d, text),
    }
}
