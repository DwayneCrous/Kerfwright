//! Mesh exporters: STL (binary + ASCII), OBJ + MTL, glTF 2.0 / GLB.
//!
//! CAD data is Z-up; glTF is Y-up, so glTF output is rotated (x, y, z) →
//! (x, z, −y). STL and OBJ keep CAD axes unless [`Axes::YUp`] is requested.

mod gltf_out;
mod obj;
mod stl;

pub use gltf_out::{glb_bytes, gltf_json_and_bin, write_gltf_files, GltfOptions};
pub use obj::{write_mtl, write_obj, ObjOptions};
pub use stl::{read_stl, write_stl_ascii, write_stl_binary, StlError, StlSoup};

use nalgebra::{Point3, Vector3};

/// Which axis is "up" in the exported coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Axes {
    /// Keep CAD coordinates (z up).
    #[default]
    ZUp,
    /// Rotate to y-up: (x, y, z) → (x, z, −y).
    YUp,
}

impl Axes {
    pub fn point(self, p: &Point3<f64>) -> [f64; 3] {
        match self {
            Axes::ZUp => [p.x, p.y, p.z],
            Axes::YUp => [p.x, p.z, -p.y],
        }
    }
    pub fn vector(self, v: &Vector3<f64>) -> [f64; 3] {
        match self {
            Axes::ZUp => [v.x, v.y, v.z],
            Axes::YUp => [v.x, v.z, -v.y],
        }
    }
}
