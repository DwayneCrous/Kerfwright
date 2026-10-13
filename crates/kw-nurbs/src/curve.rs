use crate::hom::{binom, insert_knot_once, remove_knot, Hom};
use crate::knots::KnotVector;
use nalgebra::{DMatrix, Point3, Vector3};

#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError {
    ControlPointCount { expected: usize, got: usize },
    WeightCount { expected: usize, got: usize },
    NonPositiveWeight { index: usize },
    NotEnoughPoints { need: usize, got: usize },
    SingularSystem,
    InvalidArgument(&'static str),
}

/// Result of a closest-point query on a curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveProjection {
    pub u: f64,
    pub point: Point3<f64>,
    pub distance: f64,
}

/// Rational B-spline curve `C(u) = Σ Nᵢ,ₚ(u) wᵢ Pᵢ / Σ Nᵢ,ₚ(u) wᵢ`.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsCurve {
    knots: KnotVector,
    points: Vec<Point3<f64>>,
    weights: Vec<f64>,
}

impl NurbsCurve {
    pub fn new(
        knots: KnotVector,
        points: Vec<Point3<f64>>,
        weights: Vec<f64>,
    ) -> Result<Self, NurbsError> {
        let expected = knots.num_ctrl();
        if points.len() != expected {
            return Err(NurbsError::ControlPointCount {
                expected,
                got: points.len(),
            });
        }
        if weights.len() != expected {
            return Err(NurbsError::WeightCount {
                expected,
                got: weights.len(),
            });
        }
        if let Some(i) = weights.iter().position(|w| w.is_nan() || *w <= 0.0) {
            return Err(NurbsError::NonPositiveWeight { index: i });
        }
        Ok(Self {
            knots,
            points,
            weights,
        })
    }

    /// Non-rational convenience constructor (all weights 1).
    pub fn new_polynomial(knots: KnotVector, points: Vec<Point3<f64>>) -> Result<Self, NurbsError> {
        let w = vec![1.0; points.len()];
        Self::new(knots, points, w)
    }

    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }
    pub fn points(&self) -> &[Point3<f64>] {
        &self.points
    }
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }
    pub fn degree(&self) -> usize {
        self.knots.degree()
    }
    pub fn domain(&self) -> (f64, f64) {
        self.knots.domain()
    }
    pub fn is_rational(&self) -> bool {
        self.weights.iter().any(|w| *w != 1.0)
    }

    /// Homogeneous control points `(w·P, w)`.
    pub fn homogeneous(&self) -> Vec<Hom> {
        self.points
            .iter()
            .zip(&self.weights)
            .map(|(p, w)| Hom::new(p.x * w, p.y * w, p.z * w, *w))
            .collect()
    }

    pub fn from_homogeneous(knots: KnotVector, hom: &[Hom]) -> Result<Self, NurbsError> {
        let points = hom
            .iter()
            .map(|h| Point3::new(h.x / h.w, h.y / h.w, h.z / h.w))
            .collect();
        let weights = hom.iter().map(|h| h.w).collect();
        Self::new(knots, points, weights)
    }

    /// Evaluate at `u` (clamped to the domain).
    pub fn point_at(&self, u: f64) -> Point3<f64> {
        let (a, b) = self.knots.domain();
        let u = u.clamp(a, b);
        let p = self.knots.degree();
        let span = self.knots.find_span(u);
        let basis = self.knots.basis_funs(span, u);
        let mut num = Vector3::zeros();
        let mut den = 0.0;
        for (k, nk) in basis.iter().enumerate() {
            let i = span - p + k;
            let w = self.weights[i] * nk;
            num += self.points[i].coords * w;
            den += w;
        }
        Point3::from(num / den)
    }

    /// Derivatives `C⁽⁰⁾ … C⁽ᵈ⁾` at `u` (Piegl & Tiller A3.2 + A4.2).
    pub fn derivatives(&self, u: f64, d: usize) -> Vec<Vector3<f64>> {
        let (a, b) = self.knots.domain();
        let u = u.clamp(a, b);
        let p = self.knots.degree();
        let span = self.knots.find_span(u);
        let nders = self.knots.ders_basis_funs(span, u, d);
        let mut aders = vec![Vector3::zeros(); d + 1];
        let mut wders = vec![0.0; d + 1];
        for k in 0..=d.min(p) {
            for j in 0..=p {
                let i = span - p + j;
                let w = self.weights[i];
                aders[k] += self.points[i].coords * (w * nders[k][j]);
                wders[k] += w * nders[k][j];
            }
        }
        let mut ck: Vec<Vector3<f64>> = vec![Vector3::zeros(); d + 1];
        for k in 0..=d {
            let mut v = aders[k];
            for i in 1..=k {
                v -= ck[k - i] * (binom(k, i) * wders[i]);
            }
            ck[k] = v / wders[0];
        }
        ck
    }

    pub fn reversed(&self) -> NurbsCurve {
        let (a, b) = self.domain();
        let new_knots: Vec<f64> = self
            .knots
            .as_slice()
            .iter()
            .rev()
            .map(|k| a + b - k)
            .collect();
        NurbsCurve {
            knots: KnotVector::new(self.degree(), new_knots).unwrap(),
            points: self.points.iter().rev().cloned().collect(),
            weights: self.weights.iter().rev().cloned().collect(),
        }
    }

    // ------------------------------------------------------------------
    // Refinement
    // ------------------------------------------------------------------

    /// Insert knot `u` up to `times` times (stops at multiplicity `p`).
    pub fn insert_knot(&self, u: f64, times: usize) -> NurbsCurve {
        let mut kv = self.knots.clone();
        let mut pw = self.homogeneous();
        for _ in 0..times {
            match insert_knot_once(&kv, &pw, u) {
                Some((k, q)) => {
                    kv = k;
                    pw = q;
                }
                None => break,
            }
        }
        NurbsCurve::from_homogeneous(kv, &pw).expect("insertion preserves validity")
    }

    /// Insert every knot in `knots` (repeats insert multiple times).
    pub fn refine_knots(&self, knots: &[f64]) -> NurbsCurve {
        let mut c = self.clone();
        for &u in knots {
            c = c.insert_knot(u, 1);
        }
        c
    }

    /// Remove knot `u` up to `times` times if the curve stays within `dist`
    /// (geometric tolerance, converted to homogeneous space per P&T Eq. 5.30).
    /// Returns the new curve and the number of removals actually performed.
    pub fn remove_knot(&self, u: f64, times: usize, dist: f64) -> (NurbsCurve, usize) {
        let pw = self.homogeneous();
        let wmin = self.weights.iter().cloned().fold(f64::INFINITY, f64::min);
        let pmax = self
            .points
            .iter()
            .map(|p| p.coords.norm())
            .fold(0.0, f64::max);
        let tol = dist * wmin / (1.0 + pmax);
        let (kv, q, removed) = remove_knot(&self.knots, &pw, u, times, tol);
        (
            NurbsCurve::from_homogeneous(kv, &q).expect("removal preserves validity"),
            removed,
        )
    }

    /// Split into Bézier segments (Piegl & Tiller A5.6): every interior knot is
    /// raised to multiplicity `p`, then control points are sliced per span.
    pub fn bezier_segments(&self) -> Vec<NurbsCurve> {
        let p = self.degree();
        let mut c = self.clone();
        for (u, _) in self.knots.distinct() {
            let (a, b) = self.domain();
            if u > a && u < b {
                c = c.insert_knot(u, p);
            }
        }
        let breaks: Vec<f64> = {
            let mut v = vec![self.domain().0];
            v.extend(
                c.knots
                    .distinct()
                    .into_iter()
                    .map(|(k, _)| k)
                    .filter(|k| *k > self.domain().0 && *k < self.domain().1),
            );
            v.push(self.domain().1);
            v
        };
        let pw = c.homogeneous();
        let mut out = Vec::new();
        for (s, w) in breaks.windows(2).enumerate() {
            let seg = &pw[s * p..s * p + p + 1];
            let mut k = vec![w[0]; p + 1];
            k.extend(std::iter::repeat_n(w[1], p + 1));
            out.push(
                NurbsCurve::from_homogeneous(KnotVector::new(p, k).unwrap(), seg)
                    .expect("valid Bezier segment"),
            );
        }
        out
    }

    /// Raise the degree by `t` without changing the shape: Bézier-decompose,
    /// elevate each segment, reassemble, then remove the surplus knots (they
    /// are exactly removable). Equivalent in result to Piegl & Tiller A5.9.
    pub fn elevate_degree(&self, t: usize) -> NurbsCurve {
        if t == 0 {
            return self.clone();
        }
        let p = self.degree();
        let big_p = p + t;
        let segs = self.bezier_segments();
        let mut pts: Vec<Hom> = Vec::new();
        let mut knots = vec![self.domain().0; big_p + 1];
        let orig_mult: Vec<(f64, usize)> = self.knots.distinct();
        for (si, seg) in segs.iter().enumerate() {
            let mut b = seg.homogeneous();
            for _ in 0..t {
                b = elevate_bezier_once(&b);
            }
            if si == 0 {
                pts.extend(b);
            } else {
                pts.extend(b.into_iter().skip(1));
                let brk = seg.domain().0;
                knots.extend(std::iter::repeat_n(brk, big_p));
            }
        }
        knots.extend(std::iter::repeat_n(self.domain().1, big_p + 1));
        let mut kv = KnotVector::new(big_p, knots).expect("valid elevated knots");
        let scale = 1.0 + pts.iter().map(|h| h.norm()).fold(0.0, f64::max);
        let tol = 1e-9 * scale;
        let (a, b) = self.domain();
        for (u, m) in orig_mult {
            if u > a && u < b {
                let to_remove = big_p - (m + t);
                if to_remove > 0 {
                    let (k2, q2, _) = remove_knot(&kv, &pts, u, to_remove, tol);
                    kv = k2;
                    pts = q2;
                }
            }
        }
        NurbsCurve::from_homogeneous(kv, &pts).expect("elevation preserves validity")
    }

    // ------------------------------------------------------------------
    // Fitting
    // ------------------------------------------------------------------

    /// Global interpolation through `pts` with a degree-`p` clamped B-spline
    /// (chord-length parameters, averaged knots; Piegl & Tiller A9.1).
    pub fn interpolate(pts: &[Point3<f64>], degree: usize) -> Result<NurbsCurve, NurbsError> {
        let n = pts.len();
        if n < degree + 1 || degree == 0 {
            return Err(NurbsError::NotEnoughPoints {
                need: degree + 1,
                got: n,
            });
        }
        let params = chord_length_params(pts);
        let knots = averaged_knots(&params, degree);
        let kv =
            KnotVector::new(degree, knots).map_err(|_| NurbsError::InvalidArgument("knots"))?;
        let mut a = DMatrix::<f64>::zeros(n, n);
        for (k, &u) in params.iter().enumerate() {
            let span = kv.find_span(u);
            let nb = kv.basis_funs(span, u);
            for (j, v) in nb.iter().enumerate() {
                a[(k, span - degree + j)] = *v;
            }
        }
        let rhs = DMatrix::from_fn(n, 3, |i, c| pts[i][c]);
        let sol = a.lu().solve(&rhs).ok_or(NurbsError::SingularSystem)?;
        let ctrl = (0..n)
            .map(|i| Point3::new(sol[(i, 0)], sol[(i, 1)], sol[(i, 2)]))
            .collect();
        NurbsCurve::new_polynomial(kv, ctrl)
    }

    /// The curve on the clamped `knots` that passes through `f` at the Greville abscissae of the
    /// knots (collocation): a fit that keeps the parameters of `f`, with a break wherever the knot
    /// vector has one.
    pub fn collocate(
        knots: KnotVector,
        f: &dyn Fn(f64) -> Point3<f64>,
    ) -> Result<NurbsCurve, NurbsError> {
        let p = knots.degree();
        let n = knots.num_ctrl();
        let g = greville(&knots);
        let mut a = DMatrix::<f64>::zeros(n, n);
        for (r, &t) in g.iter().enumerate() {
            let span = knots.find_span(t);
            for (j, b) in knots.basis_funs(span, t).iter().enumerate() {
                a[(r, span - p + j)] = *b;
            }
        }
        let pts: Vec<Point3<f64>> = g.iter().map(|t| f(*t)).collect();
        let rhs = DMatrix::from_fn(n, 3, |i, c| pts[i][c]);
        let sol = a.lu().solve(&rhs).ok_or(NurbsError::SingularSystem)?;
        let ctrl = (0..n)
            .map(|i| Point3::new(sol[(i, 0)], sol[(i, 1)], sol[(i, 2)]))
            .collect();
        NurbsCurve::new_polynomial(knots, ctrl)
    }

    /// Least-squares fit with `num_ctrl` control points; end points are
    /// interpolated exactly (Piegl & Tiller A9.7, no end derivatives).
    pub fn approximate(
        pts: &[Point3<f64>],
        degree: usize,
        num_ctrl: usize,
    ) -> Result<NurbsCurve, NurbsError> {
        let m = pts.len() - 1;
        let n = num_ctrl.wrapping_sub(1);
        if num_ctrl < degree + 1 || num_ctrl > pts.len() {
            return Err(NurbsError::InvalidArgument(
                "need degree+1 <= num_ctrl <= points",
            ));
        }
        let params = chord_length_params(pts);
        // Knot placement, P&T Eqs. 9.68–9.69.
        let mut knots = vec![0.0; num_ctrl + degree + 1];
        for k in knots.iter_mut().skip(num_ctrl) {
            *k = 1.0;
        }
        let d = (m + 1) as f64 / (n - degree + 1) as f64;
        for j in 1..=(n - degree) {
            let i = (j as f64 * d).floor() as usize;
            let alpha = j as f64 * d - i as f64;
            knots[degree + j] = (1.0 - alpha) * params[i - 1] + alpha * params[i];
        }
        let kv =
            KnotVector::new(degree, knots).map_err(|_| NurbsError::InvalidArgument("knots"))?;
        if n < 2 {
            return NurbsCurve::new_polynomial(kv, vec![pts[0], pts[m]]);
        }
        // Basis matrix rows for interior parameters, columns 0..=n.
        let mut nmat = DMatrix::<f64>::zeros(m + 1, n + 1);
        for (k, &u) in params.iter().enumerate() {
            let span = kv.find_span(u);
            let nb = kv.basis_funs(span, u);
            for (j, v) in nb.iter().enumerate() {
                nmat[(k, span - degree + j)] = *v;
            }
        }
        let q0 = pts[0].coords;
        let qm = pts[m].coords;
        let mut rhs = DMatrix::<f64>::zeros(n - 1, 3);
        let mut inner = DMatrix::<f64>::zeros(m - 1, n - 1);
        let mut r = DMatrix::<f64>::zeros(m - 1, 3);
        for k in 1..m {
            let rk = pts[k].coords - q0 * nmat[(k, 0)] - qm * nmat[(k, n)];
            for c in 0..3 {
                r[(k - 1, c)] = rk[c];
            }
            for j in 1..n {
                inner[(k - 1, j - 1)] = nmat[(k, j)];
            }
        }
        rhs.copy_from(&(inner.transpose() * &r));
        let ntn = inner.transpose() * &inner;
        let sol = ntn.lu().solve(&rhs).ok_or(NurbsError::SingularSystem)?;
        let mut ctrl = vec![pts[0]];
        for i in 0..n - 1 {
            ctrl.push(Point3::new(sol[(i, 0)], sol[(i, 1)], sol[(i, 2)]));
        }
        ctrl.push(pts[m]);
        NurbsCurve::new_polynomial(kv, ctrl)
    }

    // ------------------------------------------------------------------
    // Projection
    // ------------------------------------------------------------------

    /// Closest point on the curve to `target`: dense sampling per knot span to
    /// find candidate basins, then Newton iteration on `C'(u)·(C(u)−P) = 0`.
    pub fn closest_point(&self, target: &Point3<f64>) -> CurveProjection {
        let (a, b) = self.domain();
        let p = self.degree();
        let spans: Vec<f64> = self
            .knots
            .distinct()
            .into_iter()
            .map(|(k, _)| k)
            .filter(|k| *k >= a && *k <= b)
            .collect();
        // dense Hermite-like curves have hundreds of spans: fewer samples per span suffice there
        let nspans = spans.len().saturating_sub(1);
        let per_span = if nspans > 48 { 2 } else { 4 * (p + 1) };
        let mut samples: Vec<(f64, f64)> = Vec::new();
        for w in spans.windows(2) {
            for s in 0..per_span {
                let u = w[0] + (w[1] - w[0]) * s as f64 / per_span as f64;
                samples.push((u, (self.point_at(u) - target).norm_squared()));
            }
        }
        samples.push((b, (self.point_at(b) - target).norm_squared()));
        // Local minima of the sampled distance (plus the global best) seed Newton.
        let mut seeds: Vec<(f64, f64)> = Vec::new();
        for i in 0..samples.len() {
            let lo = if i == 0 {
                f64::INFINITY
            } else {
                samples[i - 1].1
            };
            let hi = if i + 1 == samples.len() {
                f64::INFINITY
            } else {
                samples[i + 1].1
            };
            if samples[i].1 <= lo && samples[i].1 <= hi {
                seeds.push(samples[i]);
            }
        }
        seeds.sort_by(|x, y| x.1.partial_cmp(&y.1).unwrap_or(std::cmp::Ordering::Equal));
        seeds.truncate(6);
        let mut best = CurveProjection {
            u: a,
            point: self.point_at(a),
            distance: f64::INFINITY,
        };
        for (u0, _) in seeds {
            let mut u = u0;
            for _ in 0..50 {
                let d = self.derivatives(u, 2);
                let r = d[0] - target.coords;
                let f = d[1].dot(&r);
                let fp = d[2].dot(&r) + d[1].norm_squared();
                if fp.abs() < 1e-300 {
                    break;
                }
                let mut step = -f / fp;
                if !step.is_finite() {
                    break;
                }
                let un = (u + step).clamp(a, b);
                step = un - u;
                u = un;
                if step.abs() <= 1e-15 * (b - a).max(1.0) {
                    break;
                }
            }
            let pt = self.point_at(u);
            let dist = (pt - target).norm();
            if dist < best.distance {
                best = CurveProjection {
                    u,
                    point: pt,
                    distance: dist,
                };
            }
        }
        best
    }
}

fn elevate_bezier_once(b: &[Hom]) -> Vec<Hom> {
    let p = b.len() - 1;
    let mut q = Vec::with_capacity(p + 2);
    for i in 0..=p + 1 {
        let a = i as f64 / (p + 1) as f64;
        let prev = if i > 0 { b[i - 1] * a } else { Hom::zeros() };
        let cur = if i <= p {
            b[i] * (1.0 - a)
        } else {
            Hom::zeros()
        };
        q.push(prev + cur);
    }
    q
}

/// The Greville abscissae of a knot vector: the average of `degree` consecutive knots.
pub(crate) fn greville(k: &KnotVector) -> Vec<f64> {
    let p = k.degree();
    let ks = k.as_slice();
    (0..k.num_ctrl())
        .map(|i| ks[i + 1..=i + p].iter().sum::<f64>() / p as f64)
        .collect()
}

pub(crate) fn chord_length_params(pts: &[Point3<f64>]) -> Vec<f64> {
    let mut u = vec![0.0; pts.len()];
    let total: f64 = pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
    if total == 0.0 {
        for (i, v) in u.iter_mut().enumerate() {
            *v = i as f64 / (pts.len() - 1).max(1) as f64;
        }
        return u;
    }
    let mut acc = 0.0;
    for i in 1..pts.len() {
        acc += (pts[i] - pts[i - 1]).norm();
        u[i] = acc / total;
    }
    *u.last_mut().unwrap() = 1.0;
    u
}

pub(crate) fn averaged_knots(params: &[f64], p: usize) -> Vec<f64> {
    let n = params.len() - 1;
    let mut k = vec![0.0; n + p + 2];
    for v in k.iter_mut().skip(n + 1) {
        *v = 1.0;
    }
    for j in 1..=(n - p) {
        k[j + p] = params[j..j + p].iter().sum::<f64>() / p as f64;
    }
    k
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_curve() -> NurbsCurve {
        let kv = KnotVector::new(3, vec![0., 0., 0., 0., 0.3, 0.7, 0.7, 1., 1., 1., 1.]).unwrap();
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.5),
            Point3::new(2.5, 1.0, -0.3),
            Point3::new(3.0, 3.0, 1.0),
            Point3::new(4.0, 0.0, 0.2),
            Point3::new(5.0, 1.0, 0.0),
            Point3::new(6.0, -1.0, 0.7),
        ];
        let w = vec![1.0, 0.8, 1.3, 1.0, 2.0, 0.9, 1.0];
        NurbsCurve::new(kv, pts, w).unwrap()
    }

    fn max_dev(a: &NurbsCurve, b: &NurbsCurve) -> f64 {
        let (lo, hi) = a.domain();
        (0..=400)
            .map(|i| {
                let u = lo + (hi - lo) * i as f64 / 400.0;
                (a.point_at(u) - b.point_at(u)).norm()
            })
            .fold(0.0, f64::max)
    }

    #[test]
    fn bezier_line_is_linear() {
        let kv = KnotVector::clamped_uniform(1, 2).unwrap();
        let c = NurbsCurve::new_polynomial(
            kv,
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        )
        .unwrap();
        assert!((c.point_at(0.5).x - 1.0).abs() < 1e-12);
    }

    #[test]
    fn insertion_preserves_geometry() {
        let c = sample_curve();
        let c2 = c.insert_knot(0.5, 2).insert_knot(0.3, 1);
        assert_eq!(c2.points().len(), c.points().len() + 3);
        assert!(max_dev(&c, &c2) < 1e-12);
    }

    #[test]
    fn insert_then_remove_roundtrip() {
        let c = sample_curve();
        let ins = c.insert_knot(0.5, 2);
        let (rem, n) = ins.remove_knot(0.5, 2, 1e-10);
        assert_eq!(n, 2);
        assert_eq!(rem.points().len(), c.points().len());
        assert!(max_dev(&c, &rem) < 1e-11);
        // existing knot 0.7 (mult 2) cannot be removed without changing shape
        let (_, n) = c.remove_knot(0.7, 1, 1e-10);
        assert_eq!(n, 0);
    }

    #[test]
    fn degree_elevation_preserves_geometry() {
        let c = sample_curve();
        for t in 1..=2 {
            let e = c.elevate_degree(t);
            assert_eq!(e.degree(), c.degree() + t);
            assert!(max_dev(&c, &e) < 1e-10, "t={t}: {}", max_dev(&c, &e));
        }
    }

    #[test]
    fn bezier_segments_match() {
        let c = sample_curve();
        let segs = c.bezier_segments();
        assert_eq!(segs.len(), 3);
        for s in &segs {
            let (a, b) = s.domain();
            for i in 0..=10 {
                let u = a + (b - a) * i as f64 / 10.0;
                assert!((s.point_at(u) - c.point_at(u)).norm() < 1e-12);
            }
        }
    }

    #[test]
    fn derivatives_match_finite_differences() {
        let c = sample_curve();
        let h = 1e-4;
        for &u in &[0.1, 0.45, 0.8] {
            let d = c.derivatives(u, 3);
            let fd1 = (c.point_at(u + h).coords - c.point_at(u - h).coords) / (2.0 * h);
            assert!((d[1] - fd1).norm() / d[1].norm() < 1e-6);
            let d_p = c.derivatives(u + h, 1)[1];
            let d_m = c.derivatives(u - h, 1)[1];
            let fd2 = (d_p - d_m) / (2.0 * h);
            assert!((d[2] - fd2).norm() / d[2].norm() < 1e-6);
            let d2_p = c.derivatives(u + h, 2)[2];
            let d2_m = c.derivatives(u - h, 2)[2];
            let fd3 = (d2_p - d2_m) / (2.0 * h);
            assert!((d[3] - fd3).norm() / d[3].norm() < 1e-5);
        }
    }

    #[test]
    fn interpolation_hits_points() {
        let pts: Vec<_> = (0..9)
            .map(|i| {
                let t = i as f64 * 0.4;
                Point3::new(t, t.sin(), 0.1 * t * t)
            })
            .collect();
        let c = NurbsCurve::interpolate(&pts, 3).unwrap();
        let params = chord_length_params(&pts);
        for (p, u) in pts.iter().zip(params) {
            assert!((c.point_at(u) - p).norm() < 1e-10);
        }
    }

    #[test]
    fn approximation_is_close_and_interpolates_ends() {
        let pts: Vec<_> = (0..40)
            .map(|i| {
                let t = i as f64 * 0.1;
                Point3::new(t, t.sin(), 0.0)
            })
            .collect();
        let c = NurbsCurve::approximate(&pts, 3, 10).unwrap();
        assert!((c.point_at(0.0) - pts[0]).norm() < 1e-12);
        assert!((c.point_at(1.0) - pts[39]).norm() < 1e-12);
        let params = chord_length_params(&pts);
        let worst = pts
            .iter()
            .zip(params)
            .map(|(p, u)| (c.point_at(u) - p).norm())
            .fold(0.0, f64::max);
        assert!(worst < 5e-3, "worst={worst}");
    }

    #[test]
    fn closest_point_finds_global_minimum() {
        let c = sample_curve();
        let target = Point3::new(2.9, 3.4, 0.9);
        let r = c.closest_point(&target);
        let brute = (0..=20000)
            .map(|i| (c.point_at(i as f64 / 20000.0) - target).norm())
            .fold(f64::INFINITY, f64::min);
        assert!(r.distance <= brute + 1e-12, "{} vs {}", r.distance, brute);
        assert!((brute - r.distance).abs() < 1e-6);
    }

    #[test]
    fn reversed_curve_traces_same_points() {
        let c = sample_curve();
        let r = c.reversed();
        for i in 0..=10 {
            let u = i as f64 / 10.0;
            assert!((c.point_at(u) - r.point_at(1.0 - u)).norm() < 1e-12);
        }
    }
}
