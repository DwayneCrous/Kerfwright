//! Linear end evaluators for curve joints.
//!
//! At the end of a line, arc or B-spline the position `P`, first derivative
//! `T` and second derivative `A` (with respect to the curve's own parameter)
//! are all *linear* in the control geometry: each is `Σ M_k · pt_k` with a
//! constant 2×2 matrix `M_k` per point. That makes the joint Jacobians cheap
//! and exact.

use crate::ad::Scalar;
use crate::residual::MAXV;
use kw_nurbs::KnotVector;

/// One point's contribution: `P += p·pt`, `T += t·pt`, `A += a·pt`
/// (matrices row-major `[m00 m01 m10 m11]`; the point's x slot is `slot`).
#[derive(Clone, Copy, Debug)]
pub struct EndTerm {
    pub slot: u8,
    pub p: [f64; 4],
    pub t: [f64; 4],
    pub a: [f64; 4],
}

#[derive(Clone, Debug, Default)]
pub struct EndLin {
    pub terms: Vec<EndTerm>,
}

const ZERO: [f64; 4] = [0.0; 4];

fn scalar(k: f64) -> [f64; 4] {
    [k, 0.0, 0.0, k]
}

/// Rotation by +90° scaled by `k`.
fn rot90(k: f64) -> [f64; 4] {
    [0.0, -k, k, 0.0]
}

impl EndLin {
    /// `(P, T, A)` of this end.
    #[allow(clippy::type_complexity)]
    pub fn eval<S: Scalar>(&self, v: &[S; MAXV]) -> ((S, S), (S, S), (S, S)) {
        let mut p = (S::c(0.0), S::c(0.0));
        let mut t = (S::c(0.0), S::c(0.0));
        let mut a = (S::c(0.0), S::c(0.0));
        for term in &self.terms {
            let (x, y) = (v[term.slot as usize], v[term.slot as usize + 1]);
            let apply = |m: &[f64; 4], acc: &mut (S, S)| {
                if m.iter().any(|c| *c != 0.0) {
                    acc.0 = acc.0 + S::c(m[0]) * x + S::c(m[1]) * y;
                    acc.1 = acc.1 + S::c(m[2]) * x + S::c(m[3]) * y;
                }
            };
            apply(&term.p, &mut p);
            apply(&term.t, &mut t);
            apply(&term.a, &mut a);
        }
        (p, t, a)
    }
}

/// Line from the point at `start_slot` to the point at `end_slot`.
/// `at_end` selects which end the joint is on.
pub fn line_end(start_slot: u8, end_slot: u8, at_end: bool) -> EndLin {
    let (p_slot, p_other) = if at_end {
        (end_slot, start_slot)
    } else {
        (start_slot, end_slot)
    };
    // T = end − start
    let (t_end, t_start) = (scalar(1.0), scalar(-1.0));
    let mut terms = vec![
        EndTerm {
            slot: p_slot,
            p: scalar(1.0),
            t: if at_end { t_end } else { t_start },
            a: ZERO,
        },
        EndTerm {
            slot: p_other,
            p: ZERO,
            t: if at_end { t_start } else { t_end },
            a: ZERO,
        },
    ];
    terms.sort_by_key(|t| t.slot);
    EndLin { terms }
}

/// Counter-clockwise arc about `c_slot` from `s_slot` to `e_slot`, parametrised
/// by angle: `T = perp(P − c)`, `A = −(P − c)`.
pub fn arc_end(c_slot: u8, s_slot: u8, e_slot: u8, at_end: bool) -> EndLin {
    let p_slot = if at_end { e_slot } else { s_slot };
    let mut terms = vec![
        EndTerm {
            slot: p_slot,
            p: scalar(1.0),
            t: rot90(1.0),
            a: scalar(-1.0),
        },
        EndTerm {
            slot: c_slot,
            p: ZERO,
            t: rot90(-1.0),
            a: scalar(1.0),
        },
    ];
    terms.sort_by_key(|t| t.slot);
    EndLin { terms }
}

/// Clamped uniform B-spline of `degree` with `n` control points. `alloc(i)` is
/// called for each control point `i` that influences the end's position or
/// first two derivatives (at most three) and must return that point's slot.
pub fn spline_end(
    degree: usize,
    n: usize,
    at_end: bool,
    alloc: &mut dyn FnMut(usize) -> u8,
) -> EndLin {
    let kv = KnotVector::clamped_uniform(degree, n).expect("valid spline");
    let u = if at_end { 1.0 } else { 0.0 };
    let span = kv.find_span(u);
    let d = kv.ders_basis_funs(span, u, 2);
    let mut terms = Vec::new();
    for j in 0..=degree {
        let idx = span - degree + j;
        let (c0, c1, c2) = (d[0][j], d[1][j], d[2][j]);
        if c0.abs() > 1e-14 || c1.abs() > 1e-14 || c2.abs() > 1e-14 {
            terms.push(EndTerm {
                slot: alloc(idx),
                p: scalar(c0),
                t: scalar(c1),
                a: scalar(c2),
            });
        }
    }
    terms.sort_by_key(|t| t.slot);
    EndLin { terms }
}
