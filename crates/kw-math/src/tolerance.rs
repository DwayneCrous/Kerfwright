use nalgebra::Point3;

/// Document-scale tolerances. Per-entity tolerances (tolerant edges/vertices)
/// live on B-rep entities and may exceed these after healing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerance {
    /// Distance below which two points are considered coincident (model units).
    pub linear: f64,
    /// Angle below which two directions are considered parallel (radians).
    pub angular: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { linear: 1e-6, angular: 1e-10 }
    }
}

impl Tolerance {
    pub fn points_coincide(&self, a: &Point3<f64>, b: &Point3<f64>) -> bool {
        (a - b).norm() <= self.linear
    }

    pub fn is_zero_length(&self, x: f64) -> bool {
        x.abs() <= self.linear
    }

    pub fn is_zero_angle(&self, x: f64) -> bool {
        x.abs() <= self.angular
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coincidence() {
        let t = Tolerance::default();
        let a = Point3::new(0.0, 0.0, 0.0);
        assert!(t.points_coincide(&a, &Point3::new(1e-7, 0.0, 0.0)));
        assert!(!t.points_coincide(&a, &Point3::new(1e-3, 0.0, 0.0)));
    }
}
