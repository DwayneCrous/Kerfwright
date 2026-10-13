//! Property tests for the Phase 1 acceptance criteria:
//! * partition of unity ≤ 1e-14 over randomised knot vectors
//! * ∂ⁿC (n ≤ 3) agrees with a finite-difference oracle to 1e-6 relative
//! * knot insertion / removal / degree elevation preserve geometry ≤ 1e-10
//!
//! Case count: `PROPTEST_CASES` (default 20 000 here; CI/release uses 1 000 000).

use kw_nurbs::{KnotVector, NurbsCurve, NurbsSurface};
use nalgebra::Point3;
use proptest::prelude::*;

fn cases() -> u32 {
    std::env::var("PROPTEST_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(20_000)
}

fn config() -> ProptestConfig {
    ProptestConfig {
        cases: cases(),
        max_shrink_iters: 64,
        ..ProptestConfig::default()
    }
}

/// (degree, clamped knot vector with interior knots of multiplicity <= degree)
fn knot_strategy() -> impl Strategy<Value = KnotVector> {
    (
        1usize..=6,
        prop::collection::vec((0.05f64..0.95, 1usize..=3), 0..5),
    )
        .prop_map(|(p, raw)| {
            let mut interior: Vec<(f64, usize)> = Vec::new();
            let mut raw = raw;
            raw.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            for (u, m) in raw {
                if interior.last().is_none_or(|(v, _)| u - v > 0.04) {
                    interior.push((u, m.min(p)));
                }
            }
            let mut k = vec![0.0; p + 1];
            for (u, m) in interior {
                k.extend(std::iter::repeat_n(u, m));
            }
            k.extend(std::iter::repeat_n(1.0, p + 1));
            KnotVector::new(p, k).unwrap()
        })
}

fn curve_strategy() -> impl Strategy<Value = NurbsCurve> {
    knot_strategy().prop_flat_map(|kv| {
        let n = kv.num_ctrl();
        (
            Just(kv),
            prop::collection::vec((-10.0f64..10.0, -10.0f64..10.0, -10.0f64..10.0), n),
            prop::collection::vec(0.3f64..3.0, n),
        )
            .prop_map(|(kv, pts, w)| {
                NurbsCurve::new(
                    kv,
                    pts.into_iter()
                        .map(|(x, y, z)| Point3::new(x, y, z))
                        .collect(),
                    w,
                )
                .unwrap()
            })
    })
}

fn max_dev(a: &NurbsCurve, b: &NurbsCurve) -> f64 {
    (0..=200)
        .map(|i| {
            let u = i as f64 / 200.0;
            (a.point_at(u) - b.point_at(u)).norm()
        })
        .fold(0.0, f64::max)
}

/// Nudge `u` so it is at least `margin` from every knot (keeps the
/// finite-difference oracle on a single polynomial piece).
fn push_away(kv: &KnotVector, mut u: f64, margin: f64) -> f64 {
    for _ in 0..8 {
        match kv.as_slice().iter().find(|k| (**k - u).abs() < margin) {
            Some(k) => u = k + margin * 1.01,
            None => break,
        }
    }
    u.min(1.0 - margin)
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn basis_partition_of_unity(kv in knot_strategy(), t in 0.0f64..=1.0, at_knot in prop::bool::weighted(0.1)) {
        let u = if at_knot { kv.as_slice()[kv.degree() + kv.len() / 3 % (kv.num_ctrl() - kv.degree()).max(1)] } else { t };
        let span = kv.find_span(u);
        let n = kv.basis_funs(span, u);
        let sum: f64 = n.iter().sum();
        prop_assert!((sum - 1.0).abs() <= 1e-14, "sum={sum} u={u}");
        prop_assert!(n.iter().all(|x| *x >= -1e-15));
    }

    #[allow(clippy::needless_range_loop)]
    #[test]
    fn basis_derivatives_sum_to_zero(kv in knot_strategy(), t in 0.0f64..=1.0) {
        let span = kv.find_span(t);
        let d = kv.ders_basis_funs(span, t, 3);
        for k in 1..=3 {
            let s: f64 = d[k].iter().sum();
            prop_assert!(s.abs() < 1e-8 * (1.0 + d[k].iter().map(|x| x.abs()).fold(0.0, f64::max)), "k={k} s={s}");
        }
    }

    #[test]
    fn derivatives_match_finite_differences(c in curve_strategy(), t in 0.02f64..0.98) {
        let kv = c.knots();
        let h = 1e-4;
        let t = push_away(kv, t, 0.02);
        let d = c.derivatives(t, 3);
        // Fourth-order central differences (Richardson) so the oracle's own
        // truncation error is far below the 1e-6 acceptance threshold.
        let cd = |f: &dyn Fn(f64) -> nalgebra::Vector3<f64>| {
            (f(t + h) - f(t - h)) * (8.0 / (12.0 * h)) - (f(t + 2.0 * h) - f(t - 2.0 * h)) * (1.0 / (12.0 * h))
        };
        let fd1 = cd(&|x| c.point_at(x).coords);
        let fd2 = cd(&|x| c.derivatives(x, 1)[1]);
        let fd3 = cd(&|x| c.derivatives(x, 2)[2]);
        for (k, fd) in [(1, fd1), (2, fd2), (3, fd3)] {
            let scale = d[k].norm().max(1.0);
            prop_assert!((d[k] - fd).norm() / scale < 1e-6, "k={k} analytic={:?} fd={:?}", d[k], fd);
        }
    }

    #[test]
    fn insertion_preserves_geometry(c in curve_strategy(), u in 0.01f64..0.99, times in 1usize..4) {
        let c2 = c.insert_knot(u, times);
        prop_assert!(max_dev(&c, &c2) <= 1e-10);
    }

    #[test]
    fn insert_then_remove_is_identity(c in curve_strategy(), u in 0.1f64..0.9) {
        // Removal divides by alpha = (u - U_i)/(U_{i+p} - U_i): if `u` sits within
        // a hair of an existing knot the problem is ill-conditioned by
        // construction, so keep the oracle in the well-conditioned regime.
        let u = push_away(c.knots(), u, 0.03);
        prop_assume!(c.knots().multiplicity(u) == 0);
        let p = c.degree();
        let times = p.min(2);
        let ins = c.insert_knot(u, times);
        let (rem, n) = ins.remove_knot(u, times, 1e-7);
        prop_assert_eq!(n, times);
        prop_assert_eq!(rem.points().len(), c.points().len());
        prop_assert!(max_dev(&c, &rem) <= 1e-10, "dev={} u={u} times={times} p={p}", max_dev(&c, &rem));
    }

    #[test]
    fn removal_never_exceeds_tolerance(c in curve_strategy(), dist in 1e-6f64..1e-2) {
        // Try to remove every interior knot; whatever is removed must keep the
        // curve within `dist` (+ the slack of repeated removals).
        let mut cur = c.clone();
        for (u, m) in c.knots().distinct() {
            if u > 0.0 && u < 1.0 {
                let (next, _) = cur.remove_knot(u, m, dist);
                cur = next;
            }
        }
        prop_assert!(max_dev(&c, &cur) <= dist * 50.0 + 1e-9);
    }

    #[test]
    fn elevation_preserves_geometry(c in curve_strategy(), t in 1usize..3) {
        let e = c.elevate_degree(t);
        prop_assert_eq!(e.degree(), c.degree() + t);
        prop_assert!(max_dev(&c, &e) <= 1e-10, "dev={}", max_dev(&c, &e));
    }

    #[test]
    fn bezier_decomposition_reproduces_curve(c in curve_strategy()) {
        for seg in c.bezier_segments() {
            let (a, b) = seg.domain();
            for i in 0..=8 {
                let u = a + (b - a) * i as f64 / 8.0;
                prop_assert!((seg.point_at(u) - c.point_at(u)).norm() <= 1e-10);
            }
        }
    }

    #[test]
    fn surface_insertion_preserves_geometry(
        pu in 2usize..4, pv in 2usize..4, nu in 4usize..7, nv in 4usize..7,
        u in 0.05f64..0.95, v in 0.05f64..0.95, seed in 0u64..1000
    ) {
        let ku = KnotVector::clamped_uniform(pu, nu).unwrap();
        let kv = KnotVector::clamped_uniform(pv, nv).unwrap();
        let mut pts = Vec::new();
        let mut w = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let s = (seed as f64) * 0.013 + (i * 7 + j * 13) as f64;
                pts.push(Point3::new(i as f64, j as f64, (s.sin()) * 2.0));
                w.push(1.0 + 0.5 * (s.cos()).abs());
            }
        }
        let s = NurbsSurface::new(ku, kv, pts, w).unwrap();
        let t = s.insert_knot_u(u, 1).insert_knot_v(v, 1);
        for i in 0..=8 {
            for j in 0..=8 {
                let (a, b) = (i as f64 / 8.0, j as f64 / 8.0);
                prop_assert!((s.point_at(a, b) - t.point_at(a, b)).norm() <= 1e-10);
            }
        }
    }
}
