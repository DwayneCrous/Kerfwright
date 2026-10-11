use crate::camera::OrbitCamera;
use crate::scene::{Scene, Segment, Vertex};
use bytemuck::{Pod, Zeroable};
use std::sync::mpsc;
use wgpu::util::DeviceExt;

pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
pub const ID_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R32Uint;

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    cam_pos: [f32; 4],
    light_dir: [f32; 4],
    viewport: [f32; 4],
    params: [f32; 4],
    ids: [u32; 4],
}

#[derive(Debug, Clone)]
pub struct RenderSettings {
    pub metallic: f32,
    pub roughness: f32,
    pub exposure: f32,
    pub ambient: f32,
    pub line_px: f32,
    pub show_edges: bool,
    /// Draw the shaded faces (off = wireframe of the edges only).
    pub show_mesh: bool,
    pub background: [f64; 3],
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            metallic: 0.15,
            roughness: 0.45,
            exposure: 1.0,
            ambient: 1.0,
            line_px: 1.6,
            show_edges: true,
            show_mesh: true,
            background: [0.10, 0.11, 0.13],
        }
    }
}

pub struct Renderer {
    pub color_format: wgpu::TextureFormat,
    mesh_pipeline: wgpu::RenderPipeline,
    edge_pipeline: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    vertex_buf: Option<wgpu::Buffer>,
    index_buf: Option<wgpu::Buffer>,
    segment_buf: Option<wgpu::Buffer>,
    index_count: u32,
    segment_count: u32,
    depth_view: wgpu::TextureView,
    id_tex: wgpu::Texture,
    id_view: wgpu::TextureView,
    size: (u32, u32),
    pub settings: RenderSettings,
    pub hovered: u32,
    pub selected: u32,
}

fn targets(color: wgpu::TextureFormat) -> [Option<wgpu::ColorTargetState>; 2] {
    [
        Some(wgpu::ColorTargetState {
            format: color,
            blend: None,
            write_mask: wgpu::ColorWrites::ALL,
        }),
        Some(wgpu::ColorTargetState {
            format: ID_FORMAT,
            blend: None,
            write_mask: wgpu::ColorWrites::ALL,
        }),
    ]
}

impl Renderer {
    pub fn new(device: &wgpu::Device, color_format: wgpu::TextureFormat, size: (u32, u32)) -> Self {
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("globals_bg"),
            layout: &bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals.as_entire_binding(),
            }],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pl"),
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: 0,
        });

        let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mesh"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/mesh.wgsl").into()),
        });
        let edge_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("edges"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/edges.wgsl").into()),
        });
        let color_targets = targets(color_format);
        // Faces are pushed back with a slope-scaled bias so topological edges
        // (drawn without bias) win even at grazing angles.
        let depth = |write: bool, compare: wgpu::CompareFunction, bias: wgpu::DepthBiasState| {
            wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(write),
                depth_compare: Some(compare),
                stencil: wgpu::StencilState::default(),
                bias,
            }
        };
        let face_bias = wgpu::DepthBiasState {
            constant: 2,
            slope_scale: 2.0,
            clamp: 0.0,
        };
        let mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh_pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &mesh_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3, 1 => Float32x3, 2 => Uint32, 3 => Unorm8x4
                    ],
                })],
            },
            fragment: Some(wgpu::FragmentState {
                module: &mesh_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &color_targets,
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(depth(true, wgpu::CompareFunction::Less, face_bias)),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let edge_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("edge_pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &edge_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Segment>() as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Uint32],
                })],
            },
            fragment: Some(wgpu::FragmentState {
                module: &edge_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &color_targets,
            }),
            primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::TriangleList, ..Default::default() },
            depth_stencil: Some(depth(true, wgpu::CompareFunction::LessEqual, wgpu::DepthBiasState::default())),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let (depth_view, id_tex, id_view) = Self::make_targets(device, size);
        Self {
            color_format,
            mesh_pipeline,
            edge_pipeline,
            globals,
            bind_group,
            vertex_buf: None,
            index_buf: None,
            segment_buf: None,
            index_count: 0,
            segment_count: 0,
            depth_view,
            id_tex,
            id_view,
            size,
            settings: RenderSettings::default(),
            hovered: 0,
            selected: 0,
        }
    }

    fn make_targets(
        device: &wgpu::Device,
        size: (u32, u32),
    ) -> (wgpu::TextureView, wgpu::Texture, wgpu::TextureView) {
        let extent = wgpu::Extent3d {
            width: size.0.max(1),
            height: size.1.max(1),
            depth_or_array_layers: 1,
        };
        let mk = |format, label| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: extent,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
                view_formats: &[],
            })
        };
        let depth = mk(DEPTH_FORMAT, "depth");
        let id = mk(ID_FORMAT, "id");
        let id_view = id.create_view(&wgpu::TextureViewDescriptor::default());
        (
            depth.create_view(&wgpu::TextureViewDescriptor::default()),
            id,
            id_view,
        )
    }

    pub fn size(&self) -> (u32, u32) {
        self.size
    }

    pub fn resize(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        let size = (size.0.max(1), size.1.max(1));
        if size == self.size {
            return;
        }
        self.size = size;
        let (d, t, v) = Self::make_targets(device, size);
        self.depth_view = d;
        self.id_tex = t;
        self.id_view = v;
    }

    pub fn upload(&mut self, device: &wgpu::Device, scene: &Scene) {
        let mk = |label, data: &[u8], usage| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: data,
                usage,
            })
        };
        self.vertex_buf = Some(mk(
            "vertices",
            bytemuck::cast_slice(&scene.vertices),
            wgpu::BufferUsages::VERTEX,
        ));
        self.index_buf = Some(mk(
            "indices",
            bytemuck::cast_slice(&scene.indices),
            wgpu::BufferUsages::INDEX,
        ));
        self.segment_buf = if scene.segments.is_empty() {
            None
        } else {
            Some(mk(
                "segments",
                bytemuck::cast_slice(&scene.segments),
                wgpu::BufferUsages::VERTEX,
            ))
        };
        self.index_count = scene.indices.len() as u32;
        self.segment_count = scene.segments.len() as u32;
    }

    fn write_globals(&self, queue: &wgpu::Queue, cam: &OrbitCamera) {
        let eye = cam.eye();
        let s = &self.settings;
        let g = Globals {
            view_proj: cam.view_proj_f32(),
            cam_pos: [eye.x as f32, eye.y as f32, eye.z as f32, 1.0],
            light_dir: [0.45, -0.35, 0.82, 0.0],
            viewport: [self.size.0 as f32, self.size.1 as f32, s.line_px, 1.0e-4],
            params: [s.metallic, s.roughness, s.exposure, s.ambient],
            ids: [self.hovered, self.selected, 0, 0],
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&g));
    }

    /// Encode one frame into `color` (plus the internal depth and id targets).
    pub fn render(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        cam: &OrbitCamera,
        color: &wgpu::TextureView,
    ) {
        self.write_globals(queue, cam);
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("frame"),
        });
        {
            let bg = self.settings.background;
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: color,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color {
                                r: bg[0],
                                g: bg[1],
                                b: bg[2],
                                a: 1.0,
                            }),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &self.id_view,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if let (Some(vb), Some(ib), true) =
                (&self.vertex_buf, &self.index_buf, self.settings.show_mesh && self.index_count > 0)
            {
                pass.set_pipeline(&self.mesh_pipeline);
                pass.set_bind_group(0, &self.bind_group, &[]);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..self.index_count, 0, 0..1);
            }
            if self.settings.show_edges && self.segment_count > 0 {
                if let Some(sb) = &self.segment_buf {
                    pass.set_pipeline(&self.edge_pipeline);
                    pass.set_bind_group(0, &self.bind_group, &[]);
                    pass.set_vertex_buffer(0, sb.slice(..));
                    pass.draw(0..6, 0..self.segment_count);
                }
            }
        }
        queue.submit(Some(enc.finish()));
    }

    /// Read back a `w × h` region of the pick-id buffer (blocking).
    pub fn read_ids(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        x: u32,
        y: u32,
        w: u32,
        h: u32,
    ) -> Vec<u32> {
        let bytes = read_texture(device, queue, &self.id_tex, x, y, w, h, 4);
        bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }
}

/// Copy a texture region into a tightly packed `Vec<u8>` (blocking).
#[allow(clippy::too_many_arguments)]
pub fn read_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    tex: &wgpu::Texture,
    x: u32,
    y: u32,
    w: u32,
    h: u32,
    bytes_per_pixel: u32,
) -> Vec<u8> {
    let unpadded = w * bytes_per_pixel;
    let padded =
        unpadded.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (padded * h) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("readback"),
    });
    enc.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: tex,
            mip_level: 0,
            origin: wgpu::Origin3d { x, y, z: 0 },
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buf,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(h),
            },
        },
        wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
    );
    queue.submit(Some(enc.finish()));
    let (tx, rx) = mpsc::channel();
    buf.slice(..).map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .expect("poll");
    rx.recv().expect("map callback").expect("map ok");
    let data = buf.slice(..).get_mapped_range().expect("mapped range");
    let mut out = Vec::with_capacity((unpadded * h) as usize);
    for row in 0..h {
        let s = (row * padded) as usize;
        out.extend_from_slice(&data[s..s + unpadded as usize]);
    }
    drop(data);
    buf.unmap();
    out
}
