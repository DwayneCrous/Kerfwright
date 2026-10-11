//! The realtime frame: opaque PBR pass, weighted blended OIT, GTAO, composite, topological edges.

use bytemuck::{Pod, Zeroable};
use kw_viewer::scene::{Scene, Segment, Vertex};
use kw_viewer::OrbitCamera;
use wgpu::util::DeviceExt;

pub const HDR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
pub const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
pub const OUT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    view: [[f32; 4]; 4],
    cam_pos: [f32; 4],
    light_dir: [f32; 4],
    viewport: [f32; 4],
    params: [f32; 4],
    proj: [f32; 4],
    extra: [f32; 4],
}

#[derive(Debug, Clone)]
pub struct FrameOptions {
    pub oit: bool,
    pub ao: bool,
    pub edges: bool,
    pub ao_radius: f32,
    pub ao_strength: f32,
    pub line_px: f32,
    pub background: [f64; 3],
}

impl Default for FrameOptions {
    fn default() -> Self {
        FrameOptions {
            oit: true,
            ao: true,
            edges: true,
            ao_radius: 0.5,
            ao_strength: 1.0,
            line_px: 1.4,
            background: [0.10, 0.11, 0.13],
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GpuTimes {
    pub opaque_ms: f64,
    pub oit_ms: f64,
    pub ao_ms: f64,
    pub composite_ms: f64,
    pub edges_ms: f64,
    pub total_ms: f64,
}

struct Targets {
    size: (u32, u32),
    depth: wgpu::TextureView,
    hdr: wgpu::TextureView,
    nd: wgpu::TextureView,
    accum: wgpu::TextureView,
    reveal: wgpu::TextureView,
    ao: wgpu::TextureView,
    ao_tex: wgpu::Texture,
    hdr_tex: wgpu::Texture,
    bind: wgpu::BindGroup,
    bind_nd: wgpu::BindGroup,
}

pub struct Frame {
    opaque: wgpu::RenderPipeline,
    oit: wgpu::RenderPipeline,
    gtao: wgpu::RenderPipeline,
    bgl_nd: wgpu::BindGroupLayout,
    composite: wgpu::RenderPipeline,
    edge: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    bg0: wgpu::BindGroup,
    bgl1: wgpu::BindGroupLayout,
    t: Targets,
    vertices: Option<wgpu::Buffer>,
    opaque_idx: Option<wgpu::Buffer>,
    trans_idx: Option<wgpu::Buffer>,
    segments: Option<wgpu::Buffer>,
    n_opaque: u32,
    n_trans: u32,
    n_seg: u32,
    query: Option<(wgpu::QuerySet, wgpu::Buffer, wgpu::Buffer)>,
    period: f32,
    pub triangles: usize,
}

fn tex(
    device: &wgpu::Device,
    size: (u32, u32),
    format: wgpu::TextureFormat,
    label: &str,
) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width: size.0.max(1),
            height: size.1.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    })
}

fn view(t: &wgpu::Texture) -> wgpu::TextureView {
    t.create_view(&wgpu::TextureViewDescriptor::default())
}

fn make_targets(
    device: &wgpu::Device,
    bgl1: &wgpu::BindGroupLayout,
    bgl_nd: &wgpu::BindGroupLayout,
    size: (u32, u32),
) -> Targets {
    let half = (size.0.div_ceil(2), size.1.div_ceil(2));
    let depth = view(&tex(device, size, DEPTH, "depth"));
    let hdr_tex = tex(device, size, HDR, "hdr");
    let nd_tex = tex(device, size, HDR, "nd");
    let accum_tex = tex(device, size, HDR, "accum");
    let reveal_tex = tex(device, size, wgpu::TextureFormat::R8Unorm, "reveal");
    let ao_tex = tex(device, half, wgpu::TextureFormat::R8Unorm, "ao");
    let (hdr, nd, accum, reveal, ao) = (
        view(&hdr_tex),
        view(&nd_tex),
        view(&accum_tex),
        view(&reveal_tex),
        view(&ao_tex),
    );
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("frame_textures"),
        layout: bgl1,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&nd),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&hdr),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::TextureView(&accum),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(&reveal),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(&ao),
            },
        ],
    });
    let bind_nd = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("frame_nd"),
        layout: bgl_nd,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::TextureView(&nd),
        }],
    });
    Targets {
        size,
        depth,
        hdr,
        nd,
        accum,
        reveal,
        ao,
        ao_tex,
        hdr_tex,
        bind,
        bind_nd,
    }
}

fn mat(m: nalgebra::Matrix4<f64>) -> [[f32; 4]; 4] {
    let mut out = [[0.0f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = m[(r, c)] as f32;
        }
    }
    out
}

impl Frame {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        size: (u32, u32),
        timestamps: bool,
    ) -> Frame {
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bgl0 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("bgl0"),
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
        let tex_entry = |binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable: false },
                view_dimension: wgpu::TextureViewDimension::D2,
                multisampled: false,
            },
            count: None,
        };
        let bgl1 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("bgl1"),
            entries: &[
                tex_entry(0),
                tex_entry(1),
                tex_entry(2),
                tex_entry(3),
                tex_entry(4),
            ],
        });
        let bgl_nd = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("bgl_nd"),
            entries: &[tex_entry(0)],
        });
        let layout0nd = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pl0nd"),
            bind_group_layouts: &[Some(&bgl0), Some(&bgl_nd)],
            immediate_size: 0,
        });
        let bg0 = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("bg0"),
            layout: &bgl0,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals.as_entire_binding(),
            }],
        });
        let layout0 = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pl0"),
            bind_group_layouts: &[Some(&bgl0)],
            immediate_size: 0,
        });
        let layout01 = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pl01"),
            bind_group_layouts: &[Some(&bgl0), Some(&bgl1)],
            immediate_size: 0,
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("frame"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/frame.wgsl").into()),
        });
        let vbuf = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Uint32, 3 => Unorm8x4],
        };
        let write_all = wgpu::ColorWrites::ALL;
        let no_blend = |f| {
            Some(wgpu::ColorTargetState {
                format: f,
                blend: None,
                write_mask: write_all,
            })
        };
        let depth_state = |write: bool, cmp: wgpu::CompareFunction, bias: wgpu::DepthBiasState| {
            wgpu::DepthStencilState {
                format: DEPTH,
                depth_write_enabled: Some(write),
                depth_compare: Some(cmp),
                stencil: wgpu::StencilState::default(),
                bias,
            }
        };
        let face_bias = wgpu::DepthBiasState {
            constant: 2,
            slope_scale: 2.0,
            clamp: 0.0,
        };
        let opaque_targets = [no_blend(HDR), no_blend(HDR)];
        let opaque = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("opaque"),
            layout: Some(&layout0),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(vbuf.clone())],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_opaque"),
                compilation_options: Default::default(),
                targets: &opaque_targets,
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(depth_state(true, wgpu::CompareFunction::Less, face_bias)),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let add = wgpu::BlendComponent {
            src_factor: wgpu::BlendFactor::One,
            dst_factor: wgpu::BlendFactor::One,
            operation: wgpu::BlendOperation::Add,
        };
        let reveal_blend = wgpu::BlendComponent {
            src_factor: wgpu::BlendFactor::Zero,
            dst_factor: wgpu::BlendFactor::OneMinusSrc,
            operation: wgpu::BlendOperation::Add,
        };
        let oit_targets = [
            Some(wgpu::ColorTargetState {
                format: HDR,
                blend: Some(wgpu::BlendState {
                    color: add,
                    alpha: add,
                }),
                write_mask: write_all,
            }),
            Some(wgpu::ColorTargetState {
                format: wgpu::TextureFormat::R8Unorm,
                blend: Some(wgpu::BlendState {
                    color: reveal_blend,
                    alpha: reveal_blend,
                }),
                write_mask: write_all,
            }),
        ];
        let oit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("oit"),
            layout: Some(&layout0),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[Some(vbuf)],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_oit"),
                compilation_options: Default::default(),
                targets: &oit_targets,
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(depth_state(
                false,
                wgpu::CompareFunction::LessEqual,
                face_bias,
            )),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let full = |label: &'static str,
                    entry: &'static str,
                    format: wgpu::TextureFormat,
                    layout: &wgpu::PipelineLayout| {
            let targets = [no_blend(format)];
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_full"),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(entry),
                    compilation_options: Default::default(),
                    targets: &targets,
                }),
                primitive: wgpu::PrimitiveState::default(),
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                multiview_mask: None,
                cache: None,
            })
        };
        let gtao = full("gtao", "fs_gtao", wgpu::TextureFormat::R8Unorm, &layout0nd);
        let composite = full("composite", "fs_composite", OUT, &layout01);
        let edge_targets = [no_blend(OUT)];
        let edge = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("edge"),
            layout: Some(&layout0),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_edge"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Segment>() as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Uint32],
                })],
            },
            fragment: Some(wgpu::FragmentState { module: &shader, entry_point: Some("fs_edge"), compilation_options: Default::default(), targets: &edge_targets }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: Some(depth_state(false, wgpu::CompareFunction::LessEqual, wgpu::DepthBiasState::default())),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let t = make_targets(device, &bgl1, &bgl_nd, size);
        let query = timestamps.then(|| {
            let qs = device.create_query_set(&wgpu::QuerySetDescriptor {
                label: Some("ts"),
                ty: wgpu::QueryType::Timestamp,
                count: 10,
            });
            let resolve = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("ts_resolve"),
                size: 80,
                usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            let read = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("ts_read"),
                size: 80,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            (qs, resolve, read)
        });
        Frame {
            opaque,
            oit,
            gtao,
            bgl_nd,
            composite,
            edge,
            globals,
            bg0,
            bgl1,
            t,
            vertices: None,
            opaque_idx: None,
            trans_idx: None,
            segments: None,
            n_opaque: 0,
            n_trans: 0,
            n_seg: 0,
            query,
            period: queue.get_timestamp_period(),
            triangles: 0,
        }
    }

    pub fn size(&self) -> (u32, u32) {
        self.t.size
    }

    pub fn resize(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        let size = (size.0.max(1), size.1.max(1));
        if size != self.t.size {
            self.t = make_targets(device, &self.bgl1, &self.bgl_nd, size);
        }
    }

    /// Upload a scene; triangles whose first vertex has colour alpha below 255 go to the transparent set.
    pub fn upload(&mut self, device: &wgpu::Device, scene: &Scene) {
        let mut opaque: Vec<u32> = Vec::with_capacity(scene.indices.len());
        let mut trans: Vec<u32> = Vec::new();
        for t in scene.indices.as_chunks::<3>().0 {
            if scene.vertices[t[0] as usize].color[3] < 255 {
                trans.extend_from_slice(t);
            } else {
                opaque.extend_from_slice(t);
            }
        }
        let mk = |label, data: &[u8], usage| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: data,
                usage,
            })
        };
        self.vertices = Some(mk(
            "vertices",
            bytemuck::cast_slice(&scene.vertices),
            wgpu::BufferUsages::VERTEX,
        ));
        self.opaque_idx = (!opaque.is_empty()).then(|| {
            mk(
                "opaque_idx",
                bytemuck::cast_slice(&opaque),
                wgpu::BufferUsages::INDEX,
            )
        });
        self.trans_idx = (!trans.is_empty()).then(|| {
            mk(
                "trans_idx",
                bytemuck::cast_slice(&trans),
                wgpu::BufferUsages::INDEX,
            )
        });
        self.segments = (!scene.segments.is_empty()).then(|| {
            mk(
                "segments",
                bytemuck::cast_slice(&scene.segments),
                wgpu::BufferUsages::VERTEX,
            )
        });
        self.n_opaque = opaque.len() as u32;
        self.n_trans = trans.len() as u32;
        self.n_seg = scene.segments.len() as u32;
        self.triangles = scene.triangle_count();
    }

    fn write_globals(&self, queue: &wgpu::Queue, cam: &OrbitCamera, o: &FrameOptions) {
        let eye = cam.eye();
        let ty = (cam.fov_y * 0.5).tan();
        let g = Globals {
            view_proj: cam.view_proj_f32(),
            view: mat(cam.view()),
            cam_pos: [eye.x as f32, eye.y as f32, eye.z as f32, 1.0],
            light_dir: [0.45, -0.35, 0.82, 0.0],
            viewport: [
                self.t.size.0 as f32,
                self.t.size.1 as f32,
                o.line_px,
                1.0e-4,
            ],
            params: [0.15, 0.45, 1.0, 1.0],
            proj: [
                cam.near as f32,
                cam.far as f32,
                (ty * cam.aspect) as f32,
                ty as f32,
            ],
            extra: [
                o.ao_radius,
                o.ao_strength,
                1.0,
                if o.ao { 1.0 } else { 0.0 },
            ],
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&g));
    }

    fn ts(&self, begin: u32) -> Option<wgpu::RenderPassTimestampWrites<'_>> {
        self.query
            .as_ref()
            .map(|(qs, _, _)| wgpu::RenderPassTimestampWrites {
                query_set: qs,
                beginning_of_pass_write_index: Some(begin),
                end_of_pass_write_index: Some(begin + 1),
            })
    }

    /// Render one frame into `out` (an `Rgba8UnormSrgb` texture view of the frame's size).
    pub fn render(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        cam: &OrbitCamera,
        out: &wgpu::TextureView,
        o: &FrameOptions,
    ) {
        self.write_globals(queue, cam, o);
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("frame"),
        });
        let bg = o.background;
        // 1. opaque
        {
            let lin = |c: f64| c.powf(2.2);
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("opaque"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: &self.t.hdr,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color {
                                r: lin(bg[0]),
                                g: lin(bg[1]),
                                b: lin(bg[2]),
                                a: 1.0,
                            }),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &self.t.nd,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.t.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: self.ts(0),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if let (Some(vb), Some(ib), true) =
                (&self.vertices, &self.opaque_idx, self.n_opaque > 0)
            {
                pass.set_pipeline(&self.opaque);
                pass.set_bind_group(0, &self.bg0, &[]);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..self.n_opaque, 0, 0..1);
            }
        }
        // 2. weighted blended OIT
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("oit"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: &self.t.accum,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &self.t.reveal,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::WHITE),
                            store: wgpu::StoreOp::Store,
                        },
                    }),
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.t.depth,
                    depth_ops: None,
                    stencil_ops: None,
                }),
                timestamp_writes: self.ts(2),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if o.oit {
                if let (Some(vb), Some(ib), true) =
                    (&self.vertices, &self.trans_idx, self.n_trans > 0)
                {
                    pass.set_pipeline(&self.oit);
                    pass.set_bind_group(0, &self.bg0, &[]);
                    pass.set_vertex_buffer(0, vb.slice(..));
                    pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..self.n_trans, 0, 0..1);
                }
            }
        }
        // 3. GTAO (half resolution)
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("gtao"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.t.ao,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::WHITE),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: self.ts(4),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if o.ao {
                pass.set_pipeline(&self.gtao);
                pass.set_bind_group(0, &self.bg0, &[]);
                pass.set_bind_group(1, &self.t.bind_nd, &[]);
                pass.draw(0..3, 0..1);
            }
        }
        // 4. composite
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("composite"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: out,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: self.ts(6),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.composite);
            pass.set_bind_group(0, &self.bg0, &[]);
            pass.set_bind_group(1, &self.t.bind, &[]);
            pass.draw(0..3, 0..1);
        }
        // 5. edges, depth-tested against the opaque depth
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("edges"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: out,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.t.depth,
                    depth_ops: None,
                    stencil_ops: None,
                }),
                timestamp_writes: self.ts(8),
                occlusion_query_set: None,
                multiview_mask: None,
            });
            if o.edges {
                if let (Some(sb), true) = (&self.segments, self.n_seg > 0) {
                    pass.set_pipeline(&self.edge);
                    pass.set_bind_group(0, &self.bg0, &[]);
                    pass.set_vertex_buffer(0, sb.slice(..));
                    pass.draw(0..6, 0..self.n_seg);
                }
            }
        }
        if let Some((qs, resolve, read)) = &self.query {
            enc.resolve_query_set(qs, 0..10, resolve, 0);
            enc.copy_buffer_to_buffer(resolve, 0, read, 0, 80);
        }
        queue.submit(Some(enc.finish()));
    }

    /// GPU times of the last submitted frame (blocks until the GPU is done); `None` without timestamps.
    pub fn read_times(&self, device: &wgpu::Device) -> Option<GpuTimes> {
        let (_, _, read) = self.query.as_ref()?;
        let (tx, rx) = std::sync::mpsc::channel();
        read.slice(..).map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        device.poll(wgpu::PollType::wait_indefinitely()).ok()?;
        rx.recv().ok()?.ok()?;
        let data = read.slice(..).get_mapped_range().ok()?;
        let v: Vec<u64> = data
            .as_chunks::<8>()
            .0
            .iter()
            .map(|c| u64::from_le_bytes(*c))
            .collect();
        drop(data);
        read.unmap();
        let ms =
            |i: usize| (v[2 * i + 1].saturating_sub(v[2 * i])) as f64 * self.period as f64 * 1e-6;
        let (a, b, c, d, e) = (ms(0), ms(1), ms(2), ms(3), ms(4));
        Some(GpuTimes {
            opaque_ms: a,
            oit_ms: b,
            ao_ms: c,
            composite_ms: d,
            edges_ms: e,
            total_ms: (v[9].saturating_sub(v[0])) as f64 * self.period as f64 * 1e-6,
        })
    }

    /// The half-resolution ambient-occlusion buffer (`R8`), row-major.
    pub fn read_ao(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> (Vec<u8>, (u32, u32)) {
        let sz = (self.t.size.0.div_ceil(2), self.t.size.1.div_ceil(2));
        (
            kw_viewer::renderer::read_texture(device, queue, &self.t.ao_tex, 0, 0, sz.0, sz.1, 1),
            sz,
        )
    }

    /// The HDR scene colour before composition (`Rgba16Float`, as raw half floats).
    pub fn read_hdr(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Vec<u8> {
        kw_viewer::renderer::read_texture(
            device,
            queue,
            &self.t.hdr_tex,
            0,
            0,
            self.t.size.0,
            self.t.size.1,
            8,
        )
    }
}
