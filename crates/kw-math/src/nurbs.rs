use crate::knots::KnotVector;
use nalgebra::{Point3, Vector3};

#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError {
    ControlPointCount { expected: usize, got: usize },
    WeightCount { expected: usize, got: usize },
    NonPositiveWeight { index: usize },
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
            return Err(NurbsError::ControlPointCount { expected, got: points.len() });
        }
        if weights.len() != expected {
            return Err(NurbsError::WeightCount { expected, got: weights.len() });
        }
        if let Some(i) = weights.iter().position(|w| !(*w > 0.0)) {
            return Err(NurbsError::NonPositiveWeight { index: i });
        }
        Ok(Self { knots, points, weights })
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
    pub fn domain(&self) -> (f64, f64) {
        self.knots.domain()
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
}

/// Tensor-product rational B-spline surface. Control net is row-major:
/// index `(i, j) -> i * nv + j`, with `i` along `u` and `j` along `v`.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsSurface {
    knots_u: KnotVector,
    knots_v: KnotVector,
    points: Vec<Point3<f64>>,
    weights: Vec<f64>,
}

impl NurbsSurface {
    pub fn new(
        knots_u: KnotVector,
        knots_v: KnotVector,
        points: Vec<Point3<f64>>,
        weights: Vec<f64>,
    ) -> Result<Self, NurbsError> {
        let expected = knots_u.num_ctrl() * knots_v.num_ctrl();
        if points.len() != expected {
            return Err(NurbsError::ControlPointCount { expected, got: points.len() });
        }
        if weights.len() != expected {
            return Err(NurbsError::WeightCount { expected, got: weights.len() });
        }
        if let Some(i) = weights.iter().position(|w| !(*w > 0.0)) {
            return Err(NurbsError::NonPositiveWeight { index: i });
        }
        Ok(Self { knots_u, knots_v, points, weights })
    }

    pub fn knots_u(&self) -> &KnotVector {
        &self.knots_u
    }
    pub fn knots_v(&self) -> &KnotVector {
        &self.knots_v
    }
    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (self.knots_u.domain(), self.knots_v.domain())
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3<f64> {
        let ((ua, ub), (va, vb)) = self.domain();
        let (u, v) = (u.clamp(ua, ub), v.clamp(va, vb));
        let (p, q) = (self.knots_u.degree(), self.knots_v.degree());
        let nv = self.knots_v.num_ctrl();
        let su = self.knots_u.find_span(u);
        let sv = self.knots_v.find_span(v);
        let bu = self.knots_u.basis_funs(su, u);
        let bv = self.knots_v.basis_funs(sv, v);
        let mut num = Vector3::zeros();
        let mut den = 0.0;
        for (a, na) in bu.iter().enumerate() {
            for (b, nb) in bv.iter().enumerate() {
                let idx = (su - p + a) * nv + (sv - q + b);
                let w = self.weights[idx] * na * nb;
                num += self.points[idx].coords * w;
                den += w;
            }
        }
        Point3::from(num / den)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bezier_line_is_linear() {
        let kv = KnotVector::clamped_uniform(1, 2).unwrap();
        let c = NurbsCurve::new(
            kv,
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let m = c.point_at(0.5);
        assert!((m.x - 1.0).abs() < 1e-12);
    }

    #[test]
    fn quarter_circle_is_exact() {
        // Rational quadratic quarter circle, radius 1, middle weight cos(45°).
        let kv = KnotVector::clamped_uniform(2, 3).unwrap();
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let c = NurbsCurve::new(
            kv,
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        )
        .unwrap();
        for s in 0..=20 {
            let p = c.point_at(s as f64 / 20.0);
            assert!((p.coords.norm() - 1.0).abs() < 1e-12);
        }
    }
}
