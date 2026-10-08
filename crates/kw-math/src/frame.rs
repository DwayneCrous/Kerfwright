use nalgebra::{Point3, Vector3};

/// Right-handed orthonormal frame. Invariant (not enforced by the type):
/// `x`, `y`, `z` are unit length, mutually orthogonal, and `x × y = z`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame3 {
    pub origin: Point3<f64>,
    pub x: Vector3<f64>,
    pub y: Vector3<f64>,
    pub z: Vector3<f64>,
}

impl Frame3 {
    pub fn world() -> Self {
        Self {
            origin: Point3::origin(),
            x: Vector3::x(),
            y: Vector3::y(),
            z: Vector3::z(),
        }
    }

    /// Frame at `origin` with normal `z` (normalised); `x` is any perpendicular.
    pub fn from_origin_normal(origin: Point3<f64>, normal: Vector3<f64>) -> Self {
        let z = normal.normalize();
        let helper = if z.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let x = helper.cross(&z).normalize();
        let y = z.cross(&x);
        Self { origin, x, y, z }
    }
}
