//! Boolean operations on B-rep solids.
//!
//! Pipeline: broad phase → surface/surface intersection → clip cuts to both faces →
//! vertex registry and edge splitting → per-face planar arrangement in parameter space →
//! fragment classification (ray casting / coincidence) → selection per operation →
//! assembly into a new body → verification. The verification step is a hard post-condition:
//! a result that is not a closed, consistently oriented, valid solid is never returned.

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod arrangement;
mod assemble;
mod cuts;
mod error;
mod model;
mod select;
mod spatial;
mod vertices;

pub use error::BooleanError;
use kw_brep::{BRep, BodyId, ShapeHistory};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Union,
    Intersect,
    Subtract,
}

#[derive(Debug, Clone)]
pub struct BoolOptions {
    /// Linear tolerance used to merge vertices and classify coincidence (model units).
    pub tol: f64,
    /// Verify the result (validate + positive volume + manifold edges) before returning.
    pub verify: bool,
}

impl Default for BoolOptions {
    fn default() -> Self {
        Self {
            tol: 1e-8,
            verify: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct BooleanResult {
    pub body: BodyId,
    pub history: ShapeHistory,
}

/// `a ∘ b` for two solid bodies stored in `rep`. The inputs are left untouched; the result is a
/// new body in the same `BRep`.
/// Cumulative wall time per phase in nanoseconds (all threads), for profiling:
/// `[setup, cuts, vertices, arrangement, classify, assemble]`.
pub static PHASE_NANOS: [std::sync::atomic::AtomicU64; 6] = [
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
];

pub fn boolean(
    rep: &mut BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    opts: &BoolOptions,
) -> Result<BooleanResult, BooleanError> {
    use std::sync::atomic::Ordering::Relaxed;
    let mut t = std::time::Instant::now();
    let mut lap = |i: usize| {
        PHASE_NANOS[i].fetch_add(t.elapsed().as_nanos() as u64, Relaxed);
        t = std::time::Instant::now();
    };
    let mut work = model::Work::new(rep, a, b, opts)?;
    lap(0);
    cuts::compute(&mut work)?;
    lap(1);
    vertices::build(&mut work)?;
    lap(2);
    arrangement::build(&mut work)?;
    lap(3);
    if std::env::var("KW_BOOL_DEBUG").is_ok() {
        eprintln!(
            "faces {} pairs {} cuts {} pieces {} coincident {}",
            work.faces.len(),
            work.pairs.len(),
            work.cuts.len(),
            work.pieces.len(),
            work.coincident.len()
        );
    }
    let selected = select::select(rep, &mut work, op)?;
    lap(4);
    let r = assemble::assemble(rep, work, selected, op, opts);
    lap(5);
    r
}

pub fn union(rep: &mut BRep, a: BodyId, b: BodyId) -> Result<BooleanResult, BooleanError> {
    boolean(rep, a, b, Op::Union, &BoolOptions::default())
}
pub fn intersect(rep: &mut BRep, a: BodyId, b: BodyId) -> Result<BooleanResult, BooleanError> {
    boolean(rep, a, b, Op::Intersect, &BoolOptions::default())
}
pub fn subtract(rep: &mut BRep, a: BodyId, b: BodyId) -> Result<BooleanResult, BooleanError> {
    boolean(rep, a, b, Op::Subtract, &BoolOptions::default())
}
