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
mod verify;
mod vertices;

pub use error::BooleanError;
use kw_brep::{BRep, BodyId, ShapeHistory};
use kw_ssi::SsiError;

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
    /// Re-run a failed operation at other merge tolerances (see [`boolean`]).
    pub heal: bool,
    /// Merge neighbouring result faces that lie in one surface (coplanar faces of the two operands, or
    /// pieces of one face that an imprint cut apart) into a single face.
    pub merge_faces: bool,
}

impl Default for BoolOptions {
    fn default() -> Self {
        Self {
            tol: 1e-8,
            verify: true,
            heal: true,
            merge_faces: true,
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

/// Does either operand have a face that is not a plane or a cylinder? Those intersect in exact lines,
/// circles and ellipses and their arrangements close up (an open one is flagged as suspect); every other
/// surface goes through tracing, where a missing or misplaced piece can pass every topological check.
fn has_traced_surfaces(rep: &BRep, a: BodyId, b: BodyId) -> bool {
    [a, b].iter().any(|body| {
        rep.body_faces(*body).iter().any(|f| {
            !matches!(
                rep.surfaces[rep.faces[*f].surface],
                kw_nurbs::SurfaceGeom::Plane(_) | kw_nurbs::SurfaceGeom::Cylinder(_)
            )
        })
    })
}

/// `a ∘ b` for two solid bodies stored in `rep`. The inputs are left untouched; the result is a
/// new body in the same `BRep`.
///
/// Checks and healing (all in the failure or suspicion path, so the common case costs nothing):
/// * an arrangement whose intersection curves do not close up (a curve that ends inside a face, a
///   fragment with a slit) is *suspect*: the result is accepted only if ray casting agrees with the
///   operation at points beside every cut and at random points;
/// * a numerical/topological failure, a refused suspect result, or a doubtful "no material" is *healed
///   by re-running* at other merge tolerances, where vertices and coincidences are decided differently;
///   a retried result must pass inclusion–exclusion with the complementary operation and the same
///   sampling check;
/// * only a verified solid is returned; otherwise the original error stands.
pub fn boolean(
    rep: &mut BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    opts: &BoolOptions,
) -> Result<BooleanResult, BooleanError> {
    let first = boolean_flagged(rep, a, b, op, opts);
    // bodies up to this many faces are always checked against ray casting at random points (a few
    // milliseconds); larger ones only when the arrangement itself looks suspect
    const ALWAYS_CHECK_FACES: usize = 4000;
    let mut why: BooleanError = match &first {
        Ok((r, None)) => {
            let hard = has_traced_surfaces(rep, a, b);
            let small =
                hard && rep.body_faces(a).len() + rep.body_faces(b).len() <= ALWAYS_CHECK_FACES;
            if !opts.heal
                || !opts.verify
                || !small
                || verify::sample_verify(rep, a, b, op, Some(r.body), &[])
            {
                return first.map(|(r, _)| r);
            }
            BooleanError::Topology("the result disagrees with ray casting".into())
        }
        Ok((r, Some(probes))) => {
            // the probes sit beside the cuts; random points add little for exact (plane / cylinder) operands
            let random = if has_traced_surfaces(rep, a, b) {
                600
            } else {
                60
            };
            if !opts.heal
                || !opts.verify
                || verify::sample_verify_n(rep, a, b, op, Some(r.body), probes, random)
            {
                return first.map(|(r, _)| r);
            }
            BooleanError::Topology(
                "the arrangement is incomplete and the result disagrees with ray casting".into(),
            )
        }
        Err(e) => e.clone(),
    };
    if !opts.heal {
        return first.map(|(r, _)| r);
    }
    let retryable = |e: &BooleanError| {
        matches!(
            e,
            BooleanError::Topology(_)
                | BooleanError::Classification(_)
                | BooleanError::Invalid(_)
                | BooleanError::Ssi(SsiError::Failed(_))
        )
    };
    // "no material" is only believed when the operands cannot overlap, or when the complementary
    // union says so (V(A∪B) = V(A) + V(B) for a disjoint intersection, = V(B) for A ⊂ B)
    let empty_confirmed = |rep: &mut BRep| -> bool {
        // a union contains both operands: it is never empty
        if op == Op::Union {
            return false;
        }
        if !rep.body_bbox(a).intersects(&rep.body_bbox(b)) {
            return true;
        }
        let (Some(va), Some(vb)) = (rep.volume(a), rep.volume(b)) else {
            return true;
        };
        let by_volume = match boolean_once(rep, a, b, Op::Union, opts) {
            Ok(u) => rep.volume(u.body).is_some_and(|vu| {
                let want = if op == Op::Intersect { va + vb } else { vb };
                (vu - want).abs() <= 1e-9 * (va + vb)
            }),
            Err(_) => false,
        };
        // the union can be wrong in the same way as the operation: ray casting has the last word
        by_volume && verify::sample_verify_n(rep, a, b, op, None, &[], 3000)
    };
    let worth_retrying = match &why {
        BooleanError::Empty => !empty_confirmed(rep),
        e => retryable(e),
    };
    if !worth_retrying {
        return first.map(|(r, _)| r);
    }
    for factor in [10.0, 0.1, 100.0, 0.01] {
        let o = BoolOptions {
            tol: opts.tol * factor,
            ..opts.clone()
        };
        match boolean_flagged(rep, a, b, op, &o) {
            Ok((r, probes)) => {
                let sampled =
                    verify::sample_verify(rep, a, b, op, Some(r.body), &probes.unwrap_or_default());
                if sampled && healed_result_is_consistent(rep, a, b, op, &o, &r) {
                    return Ok(r);
                }
            }
            Err(BooleanError::Empty) if why == BooleanError::Empty && op != Op::Union => {
                // consistently empty at other tolerances too: believe it
                if verify::sample_verify_n(rep, a, b, op, None, &[], 3000) {
                    return Err(BooleanError::Empty);
                }
            }
            Err(_) => {}
        }
    }
    // nothing verified: a refused result stays refused, and an "empty" that could not be confirmed is
    // reported as the failure it is rather than as a result
    if first.is_ok() {
        why = BooleanError::Topology("no verified result".into());
    }
    Err(match first {
        Err(BooleanError::Empty) => BooleanError::Topology(
            "the result could not be determined (empty, but not confirmed)".into(),
        ),
        Err(e) => e,
        Ok(_) => why,
    })
}

/// A result obtained by retrying is only trusted if it satisfies inclusion–exclusion with the
/// complementary operation computed at the same tolerance:
/// `V(A∪B) + V(A∩B) = V(A) + V(B)` and `V(A∖B) + V(A∩B) = V(A)`.
fn healed_result_is_consistent(
    rep: &mut BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    opts: &BoolOptions,
    result: &BooleanResult,
) -> bool {
    let (Some(va), Some(vb), Some(vr)) = (rep.volume(a), rep.volume(b), rep.volume(result.body))
    else {
        return false;
    };
    // the complementary operation: the intersection, or the union when the result is the intersection
    let other = if op == Op::Intersect {
        Op::Union
    } else {
        Op::Intersect
    };
    let vo = match boolean_once(rep, a, b, other, opts) {
        Ok(r) => match rep.volume(r.body) {
            Some(v) => v,
            None => return false,
        },
        Err(BooleanError::Empty) => 0.0,
        Err(_) => return false,
    };
    let (lhs, rhs) = match op {
        Op::Union => (vr + vo, va + vb),
        Op::Intersect => (vr + vo, va + vb),
        Op::Subtract => (vr + vo, va),
    };
    (lhs - rhs).abs() <= 1e-9 * (va + vb)
}

fn boolean_once(
    rep: &mut BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    opts: &BoolOptions,
) -> Result<BooleanResult, BooleanError> {
    boolean_flagged(rep, a, b, op, opts).map(|(r, _)| r)
}

/// As [`boolean_once`], also returning probe points when the arrangement was suspect.
fn boolean_flagged(
    rep: &mut BRep,
    a: BodyId,
    b: BodyId,
    op: Op,
    opts: &BoolOptions,
) -> Result<(BooleanResult, Option<Vec<kw_math::Point3<f64>>>), BooleanError> {
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
    let probes = work.suspect.then(|| {
        let mut p = verify::cut_probes(&work, 40);
        p.extend(work.probes.iter().copied());
        // a few hundred well-spread probes find a missing piece as surely as thousands do, and each probe is
        // three point classifications
        const MAX_PROBES: usize = 300;
        if p.len() > MAX_PROBES {
            let step = p.len().div_ceil(MAX_PROBES);
            p = p.into_iter().step_by(step).collect();
        }
        p
    });
    let r = assemble::assemble(rep, work, selected, op, opts);
    lap(5);
    r.map(|r| (r, probes))
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
