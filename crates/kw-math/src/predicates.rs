//! Robust (exact-sign) geometric predicates — Shewchuk's adaptive-precision
//! orient/incircle via the `robust` crate, returning a typed sign.

use nalgebra::{Point2, Point3};
use robust::{Coord, Coord3D};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sign {
    Negative,
    Zero,
    Positive,
}

impl Sign {
    fn of(x: f64) -> Sign {
        if x > 0.0 {
            Sign::Positive
        } else if x < 0.0 {
            Sign::Negative
        } else {
            Sign::Zero
        }
    }
}

fn c2(p: &Point2<f64>) -> Coord<f64> {
    Coord { x: p.x, y: p.y }
}

fn c3(p: &Point3<f64>) -> Coord3D<f64> {
    Coord3D {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

/// Positive if `a, b, c` are in counter-clockwise order.
pub fn orient2d(a: &Point2<f64>, b: &Point2<f64>, c: &Point2<f64>) -> Sign {
    Sign::of(robust::orient2d(c2(a), c2(b), c2(c)))
}

/// Positive if `d` lies below the plane of `a, b, c` (which are CCW when
/// viewed from above) — the sign convention of `robust::orient3d`.
pub fn orient3d(a: &Point3<f64>, b: &Point3<f64>, c: &Point3<f64>, d: &Point3<f64>) -> Sign {
    Sign::of(robust::orient3d(c3(a), c3(b), c3(c), c3(d)))
}

/// Positive if `d` is strictly inside the circle through CCW `a, b, c`.
pub fn incircle(a: &Point2<f64>, b: &Point2<f64>, c: &Point2<f64>, d: &Point2<f64>) -> Sign {
    Sign::of(robust::incircle(c2(a), c2(b), c2(c), c2(d)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orient2d_basic() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        assert_eq!(orient2d(&a, &b, &Point2::new(0.0, 1.0)), Sign::Positive);
        assert_eq!(orient2d(&a, &b, &Point2::new(0.0, -1.0)), Sign::Negative);
        assert_eq!(orient2d(&a, &b, &Point2::new(2.0, 0.0)), Sign::Zero);
    }

    #[test]
    fn orient2d_is_exact_on_near_degenerate_input() {
        // Classic failure case for naive floating point: nearly collinear points.
        let a = Point2::new(0.5, 0.5);
        let b = Point2::new(12.0, 12.0);
        let c = Point2::new(24.0, 24.0);
        assert_eq!(orient2d(&a, &b, &c), Sign::Zero);
        let c2 = Point2::new(24.0, 24.0 + f64::EPSILON * 16.0);
        assert_eq!(orient2d(&a, &b, &c2), Sign::Positive);
    }

    #[test]
    fn incircle_basic() {
        let a = Point2::new(1.0, 0.0);
        let b = Point2::new(0.0, 1.0);
        let c = Point2::new(-1.0, 0.0);
        assert_eq!(incircle(&a, &b, &c, &Point2::new(0.0, 0.0)), Sign::Positive);
        assert_eq!(incircle(&a, &b, &c, &Point2::new(3.0, 0.0)), Sign::Negative);
        assert_eq!(incircle(&a, &b, &c, &Point2::new(0.0, -1.0)), Sign::Zero);
    }

    #[test]
    fn orient3d_basic() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);
        assert_eq!(
            orient3d(&a, &b, &c, &Point3::new(0.0, 0.0, 1.0)),
            Sign::Negative
        );
        assert_eq!(
            orient3d(&a, &b, &c, &Point3::new(0.0, 0.0, -1.0)),
            Sign::Positive
        );
        assert_eq!(
            orient3d(&a, &b, &c, &Point3::new(5.0, 5.0, 0.0)),
            Sign::Zero
        );
    }
}
