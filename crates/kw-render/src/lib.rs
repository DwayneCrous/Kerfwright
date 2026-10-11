//! Advanced rendering on wgpu: the realtime frame (opaque PBR, weighted blended OIT, GTAO, edges) and the
//! progressive path tracer with its denoiser and the CPU reference it is validated against.

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod frame;
pub mod gpu;
pub mod pt;

pub use frame::{Frame, FrameOptions, GpuTimes};
pub use gpu::RenderGpu;

/// Read an `Rgba8UnormSrgb` texture back as RGBA bytes.
pub fn read_rgba(gpu: &RenderGpu, tex: &wgpu::Texture, size: (u32, u32)) -> Vec<u8> {
    kw_viewer::renderer::read_texture(&gpu.device, &gpu.queue, tex, 0, 0, size.0, size.1, 4)
}

/// An offscreen `Rgba8UnormSrgb` colour target.
pub fn offscreen(device: &wgpu::Device, size: (u32, u32)) -> (wgpu::Texture, wgpu::TextureView) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("offscreen"),
        size: wgpu::Extent3d {
            width: size.0,
            height: size.1,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: frame::OUT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::COPY_SRC
            | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
    (tex, view)
}
