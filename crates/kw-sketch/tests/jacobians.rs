//! Phase 2 acceptance: analytic Jacobians match finite-difference *and*
//! forward-mode AD oracles to 1e-7 for every constraint residual type over
//! randomised configurations.

#![allow(clippy::needless_range_loop)]

use kw_sketch::ad::{Dual, Scalar};
use kw_sketch::residual::*;
use proptest::prelude::*;

fn cases() -> u32 {
    std::env::var("PROPTEST_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(20_000)
}

/// A residual kind plus how to draw valid slot values for it.
struct Spec {
    name: &'static str,
    kind: Kind,
    nv: usize,
    /// slots that must be strictly positive (radii, distances, minor axes)
    positive: Vec<usize>,
    /// (start, end) point slot pairs that must not coincide (line directions)
    lines: Vec<(usize, usize)>,
}

fn specs() -> Vec<Spec> {
    use Rad::*;
    let end_line = |a, b, at_end| line_end(a, b, at_end);
    let joint = |a: EndLin, b: EndLin, sa: f64, sb: f64, level: u8| {
        Kind::Joint(Box::new(JointSpec {
            a,
            b,
            sign_a: sa,
            sign_b: sb,
            level,
            g0: true,
            branch: 1.0,
        }))
    };
    let joint_no_g0 = |a: EndLin, b: EndLin, sa: f64, sb: f64, level: u8| {
        Kind::Joint(Box::new(JointSpec {
            a,
            b,
            sign_a: sa,
            sign_b: sb,
            level,
            g0: false,
            branch: 1.0,
        }))
    };
    let s = |name, kind, nv, positive: Vec<usize>, lines: Vec<(usize, usize)>| Spec {
        name,
        kind,
        nv,
        positive,
        lines,
    };
    vec![
        s("pt2pt", Kind::Pt2Pt, 4, vec![], vec![]),
        s("on_line", Kind::OnLine, 6, vec![], vec![(2, 4)]),
        s("on_circle_var", Kind::OnCircle(Var(4)), 5, vec![4], vec![]),
        s(
            "on_circle_pt",
            Kind::OnCircle(Pt { c: 2, s: 4 }),
            6,
            vec![],
            vec![(2, 4)],
        ),
        s("on_ellipse", Kind::OnEllipse, 7, vec![6], vec![(2, 4)]),
        s("diff", Kind::Diff, 2, vec![], vec![]),
        s(
            "rad_val_var",
            Kind::RadVal {
                rad: Var(0),
                scale: 1.0,
                v: 1,
            },
            2,
            vec![0],
            vec![],
        ),
        s(
            "diam_val_var",
            Kind::RadVal {
                rad: Var(0),
                scale: 2.0,
                v: 1,
            },
            2,
            vec![0],
            vec![],
        ),
        s(
            "rad_val_pt",
            Kind::RadVal {
                rad: Pt { c: 0, s: 2 },
                scale: 1.0,
                v: 4,
            },
            5,
            vec![],
            vec![(0, 2)],
        ),
        s("parallel", Kind::Parallel, 8, vec![], vec![(0, 2), (4, 6)]),
        s("perp", Kind::Perp, 8, vec![], vec![(0, 2), (4, 6)]),
        s("eq_len", Kind::EqLen, 8, vec![], vec![]),
        s(
            "eq_rad_var_var",
            Kind::EqRad(Var(0), Var(1)),
            2,
            vec![0, 1],
            vec![],
        ),
        s(
            "eq_rad_var_pt",
            Kind::EqRad(Var(0), Pt { c: 1, s: 3 }),
            5,
            vec![0],
            vec![(1, 3)],
        ),
        s(
            "eq_rad_pt_pt",
            Kind::EqRad(Pt { c: 0, s: 2 }, Pt { c: 4, s: 6 }),
            8,
            vec![],
            vec![(0, 2), (4, 6)],
        ),
        s("midpoint", Kind::Midpoint, 6, vec![], vec![]),
        s("symmetric", Kind::Symmetric, 8, vec![], vec![(4, 6)]),
        s(
            "tan_line_circ_var",
            Kind::TanLineCirc { s: 1.0, r: Var(6) },
            7,
            vec![6],
            vec![(0, 2)],
        ),
        s(
            "tan_line_circ_pt",
            Kind::TanLineCirc {
                s: -1.0,
                r: Pt { c: 4, s: 6 },
            },
            8,
            vec![],
            vec![(0, 2), (4, 6)],
        ),
        s(
            "tan_circ_circ_vv",
            Kind::TanCircCirc {
                s: 1.0,
                r1: Var(4),
                r2: Var(5),
            },
            6,
            vec![4, 5],
            vec![],
        ),
        s(
            "tan_circ_circ_pp",
            Kind::TanCircCirc {
                s: -1.0,
                r1: Pt { c: 0, s: 4 },
                r2: Pt { c: 2, s: 6 },
            },
            8,
            vec![],
            vec![(0, 4), (2, 6)],
        ),
        s(
            "tan_circ_circ_vp",
            Kind::TanCircCirc {
                s: 1.0,
                r1: Var(4),
                r2: Pt { c: 2, s: 5 },
            },
            7,
            vec![4],
            vec![(2, 5)],
        ),
        s("dist_pp", Kind::DistPP, 5, vec![4], vec![]),
        s("dist_pl", Kind::DistPL { s: 1.0 }, 7, vec![6], vec![(2, 4)]),
        s(
            "dist_pl_neg",
            Kind::DistPL { s: -1.0 },
            7,
            vec![6],
            vec![(2, 4)],
        ),
        s("angle", Kind::Angle, 9, vec![], vec![(0, 2), (4, 6)]),
        s("arc_cons", Kind::ArcCons, 6, vec![], vec![]),
        // joints: line/arc/spline combinations at every end pairing and level
        s(
            "joint_line_line_g0",
            joint(end_line(0, 2, true), end_line(4, 6, false), 1.0, -1.0, 0),
            8,
            vec![],
            vec![(0, 2), (4, 6)],
        ),
        s(
            "joint_line_line_g1",
            joint(end_line(0, 2, true), end_line(4, 6, false), 1.0, -1.0, 1),
            8,
            vec![],
            vec![(0, 2), (4, 6)],
        ),
        s(
            "joint_line_line_g2",
            joint(end_line(0, 2, false), end_line(4, 6, true), -1.0, 1.0, 2),
            8,
            vec![],
            vec![(0, 2), (4, 6)],
        ),
        s(
            "joint_nog0_line_arc_g1",
            joint_no_g0(end_line(0, 2, true), arc_end(4, 6, 8, false), 1.0, -1.0, 1),
            10,
            vec![],
            vec![(0, 2), (4, 6), (4, 8)],
        ),
        s(
            "joint_nog0_line_arc_g2",
            joint_no_g0(end_line(0, 2, true), arc_end(4, 6, 8, false), 1.0, -1.0, 2),
            10,
            vec![],
            vec![(0, 2), (4, 6), (4, 8)],
        ),
        s(
            "joint_line_arc_g2",
            joint(end_line(0, 2, true), arc_end(4, 6, 8, false), 1.0, -1.0, 2),
            10,
            vec![],
            vec![(0, 2), (4, 6), (4, 8)],
        ),
        s(
            "joint_arc_arc_g2",
            joint(arc_end(0, 2, 4, true), arc_end(6, 8, 10, true), 1.0, 1.0, 2),
            12,
            vec![],
            vec![(0, 2), (0, 4), (6, 8), (6, 10)],
        ),
        s(
            "joint_spline_line_g2",
            joint(
                spline_end(3, 5, true, &mut |i| [0u8, 2, 4, 6, 8][i]),
                end_line(10, 12, false),
                1.0,
                -1.0,
                2,
            ),
            14,
            vec![],
            vec![(10, 12), (6, 8), (4, 6)],
        ),
        s(
            "joint_spline_spline_g2",
            joint(
                spline_end(3, 4, false, &mut |i| [0u8, 2, 4, 6][i]),
                spline_end(2, 3, true, &mut |i| [8u8, 10, 12][i]),
                -1.0,
                1.0,
                2,
            ),
            14,
            vec![],
            vec![(0, 2), (2, 4), (8, 10), (10, 12)],
        ),
        s(
            "joint_spline_arc_g1",
            joint(
                spline_end(2, 3, true, &mut |i| [0u8, 2, 4][i]),
                arc_end(6, 8, 10, true),
                1.0,
                1.0,
                1,
            ),
            12,
            vec![],
            vec![(6, 8), (6, 10), (0, 2), (2, 4)],
        ),
    ]
}

fn draw(spec: &Spec, raw: &[f64]) -> [f64; MAXV] {
    let mut v = [0.0; MAXV];
    for i in 0..spec.nv {
        v[i] = raw[i] * 5.0;
    }
    for &p in &spec.positive {
        v[p] = v[p].abs() + 0.6;
    }
    // push line endpoints apart so directions are well defined
    for &(a, b) in &spec.lines {
        let (dx, dy) = (v[b] - v[a], v[b + 1] - v[a + 1]);
        if (dx * dx + dy * dy).sqrt() < 0.8 {
            v[b] = v[a] + 0.8 + dx.abs();
            v[b + 1] = v[a + 1] + dy;
        }
    }
    v
}

fn analytic(spec: &Spec, v: &[f64; MAXV]) -> Jac {
    let mut j = [[0.0; MAXV]; MAXR];
    jac(&spec.kind, v, &mut j);
    j
}

fn finite_difference(spec: &Spec, v: &[f64; MAXV]) -> Jac {
    let mut j = [[0.0; MAXV]; MAXR];
    let n = spec.kind.arity();
    for c in 0..spec.nv {
        let h = 1e-6 * (1.0 + v[c].abs());
        let (mut vp, mut vm) = (*v, *v);
        vp[c] += h;
        vm[c] -= h;
        let (mut op, mut om) = ([0.0; MAXR], [0.0; MAXR]);
        eval(&spec.kind, &vp, &mut op);
        eval(&spec.kind, &vm, &mut om);
        for r in 0..n {
            j[r][c] = (op[r] - om[r]) / (2.0 * h);
        }
    }
    j
}

fn automatic(spec: &Spec, v: &[f64; MAXV]) -> Jac {
    let mut j = [[0.0; MAXV]; MAXR];
    let dv: [Dual; MAXV] = std::array::from_fn(|i| {
        if i < spec.nv {
            Dual::variable(v[i], i)
        } else {
            Dual::c(v[i])
        }
    });
    let mut out = [Dual::c(0.0); MAXR];
    eval(&spec.kind, &dv, &mut out);
    for r in 0..spec.kind.arity() {
        for c in 0..spec.nv {
            j[r][c] = out[r].d[c];
        }
    }
    j
}

fn max_rel_err(spec: &Spec, a: &Jac, b: &Jac) -> f64 {
    let mut worst = 0.0f64;
    for r in 0..spec.kind.arity() {
        let scale = (0..spec.nv)
            .map(|c| a[r][c].abs().max(b[r][c].abs()))
            .fold(1.0, f64::max);
        for c in 0..spec.nv {
            worst = worst.max((a[r][c] - b[r][c]).abs() / scale);
        }
    }
    worst
}

#[test]
fn every_kind_is_covered() {
    // arity bookkeeping: eval must only write the first `arity` outputs
    for spec in specs() {
        let raw: Vec<f64> = (0..MAXV)
            .map(|i| ((i * 7 + 3) % 11) as f64 / 11.0 - 0.4)
            .collect();
        let v = draw(&spec, &raw);
        let mut out = [f64::NAN; MAXR];
        eval(&spec.kind, &v, &mut out);
        for (i, o) in out.iter().enumerate() {
            assert_eq!(
                o.is_nan(),
                i >= spec.kind.arity(),
                "{}: output {i}",
                spec.name
            );
        }
    }
}

#[test]
fn pattern_covers_all_nonzeros() {
    for spec in specs() {
        for seed in 0..50u64 {
            let raw: Vec<f64> = (0..MAXV)
                .map(|i| (((seed * 31 + i as u64 * 17 + 5) % 97) as f64 / 97.0) * 2.0 - 1.0)
                .collect();
            let v = draw(&spec, &raw);
            let j = analytic(&spec, &v);
            let mut pat = Vec::new();
            for eq in 0..spec.kind.arity() {
                spec.kind.pattern(eq, spec.nv, &mut pat);
                for c in 0..spec.nv {
                    if j[eq][c].abs() > 1e-12 {
                        assert!(
                            pat.contains(&(c as u8)),
                            "{}: eq {eq} nonzero at slot {c} missing from pattern",
                            spec.name
                        );
                    }
                }
            }
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: cases(), ..ProptestConfig::default() })]

    #[test]
    fn analytic_jacobian_matches_fd_and_ad(
        which in 0usize..64,
        raw in prop::collection::vec(-1.0f64..1.0, MAXV),
    ) {
        let all = specs();
        let spec = &all[which % all.len()];
        let v = draw(spec, &raw);
        let a = analytic(spec, &v);
        let ad = automatic(spec, &v);
        let fd = finite_difference(spec, &v);
        let e_ad = max_rel_err(spec, &a, &ad);
        let e_fd = max_rel_err(spec, &a, &fd);
        prop_assert!(e_ad < 1e-7, "{}: analytic vs AD {e_ad:e}", spec.name);
        prop_assert!(e_fd < 1e-7, "{}: analytic vs FD {e_fd:e}", spec.name);
    }
}
