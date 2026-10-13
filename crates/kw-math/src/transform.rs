use crate::frame::Frame3;
use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

/// Rigid transform (rotation + translation). Uniform scale is deliberately
/// excluded: scaling a B-rep changes tolerances and is handled explicitly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3(pub Isometry3<f64>);

impl Transform3 {
    pub fn identity() -> Self {
        Self(Isometry3::identity())
    }

    pub fn translation(v: Vector3<f64>) -> Self {
        Self(Isometry3::from_parts(
            Translation3::from(v),
            UnitQuaternion::identity(),
        ))
    }

    pub fn rotation(axis: Vector3<f64>, angle: f64) -> Self {
        let q = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle);
        Self(Isometry3::from_parts(Translation3::identity(), q))
    }

    /// Transform taking world coordinates to coordinates local to `frame`.
    pub fn world_to_frame(frame: &Frame3) -> Self {
        Self::frame_to_world(frame).inverse()
    }

    /// Transform taking coordinates local to `frame` into world coordinates.
    pub fn frame_to_world(frame: &Frame3) -> Self {
        let rot = Rotation3::from_basis_unchecked(&[frame.x, frame.y, frame.z]);
        Self(Isometry3::from_parts(
            Translation3::from(frame.origin.coords),
            UnitQuaternion::from_rotation_matrix(&rot),
        ))
    }

    pub fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }

    pub fn then(&self, next: &Transform3) -> Self {
        Self(next.0 * self.0)
    }

    pub fn apply_point(&self, p: &Point3<f64>) -> Point3<f64> {
        self.0.transform_point(p)
    }

    pub fn apply_vector(&self, v: &Vector3<f64>) -> Vector3<f64> {
        self.0.transform_vector(v)
    }

    pub fn to_matrix(&self) -> Matrix4<f64> {
        self.0.to_homogeneous()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_round_trip() {
        let f = Frame3::from_origin_normal(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, 1.0, 0.5));
        let t = Transform3::frame_to_world(&f);
        let p = Point3::new(0.3, -0.2, 0.9);
        let q = t.inverse().apply_point(&t.apply_point(&p));
        assert!((p - q).norm() < 1e-14);
        // local z maps onto frame z
        let z = t.apply_vector(&Vector3::z());
        assert!((z - f.z).norm() < 1e-14);
    }

    #[test]
    fn rotation_quarter_turn() {
        let r = Transform3::rotation(Vector3::z(), std::f64::consts::FRAC_PI_2);
        let p = r.apply_point(&Point3::new(1.0, 0.0, 0.0));
        assert!((p - Point3::new(0.0, 1.0, 0.0)).norm() < 1e-15);
    }
}
