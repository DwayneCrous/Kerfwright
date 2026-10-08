//! Kerfwright viewer: `wgpu` renderer with PBR shading, topological edge
//! lines, and an ID buffer for face/edge picking; windowed (winit) and
//! headless (offscreen) front ends.

pub mod app;
pub mod camera;
pub mod gpu;
pub mod headless;
pub mod renderer;
pub mod scene;

pub use camera::OrbitCamera;
pub use gpu::Gpu;
pub use renderer::{RenderSettings, Renderer};
pub use scene::{PickTarget, Scene};
