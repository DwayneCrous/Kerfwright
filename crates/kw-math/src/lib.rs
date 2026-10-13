//! Math foundation: tolerance model, frames, transforms, bounding boxes and
//! robust geometric predicates. NURBS and analytic geometry live in `kw-nurbs`.
//! See docs/01-research/01-brep-kernel.md.

pub mod aabb;
pub mod frame;
pub mod predicates;
pub mod tolerance;
pub mod transform;

pub use nalgebra::{Matrix3, Matrix4, Point2, Point3, Unit, Vector2, Vector3, Vector4};

pub use aabb::Aabb;
pub use frame::Frame3;
pub use tolerance::Tolerance;
pub use transform::Transform3;
