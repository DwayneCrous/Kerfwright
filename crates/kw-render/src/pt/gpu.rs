//! The GPU path tracer: LBVH built on the CPU, traversal, shading and accumulation in a WGSL compute shader.

use super::lbvh::{Lbvh, LEAF};
use super::{PtCamera, PtScene};
use crate::RenderGpu;
use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Params {
    origin: [f32; 4],
    fwd: [f32; 4],
    right: [f32; 4],
    up: [f32; 4],
    env: [f32; 4],
    width: u32,
    height: u32,
    frame: u32,
    seed: u32,
    n_lights: u32,
    max_depth: u32,
    spp: u32,
    p0: u32,
    total_area: f32,
    q: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GpuMat {
    albedo: [f32; 3],
    rough: f32,
    emission: [f32; 3],
    metallic: f32,
    specular: f32,
    p: [f32; 3],
}

pub struct GpuTracer {
    pipeline: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    params: wgpu::Buffer,
    accum: wgpu::Buffer,
    readback: wgpu::Buffer,
    size: (u32, u32),
    cam: PtCamera,
    env: [f32; 4],
    n_lights: u32,
    total_area: f32,
    pub max_depth: u32,
    pub samples: u32,
    pub nodes: usize,
}

fn storage<T: Pod>(device: &wgpu::Device, data: &[T], label: &str) -> wgpu::Buffer {
    let bytes: &[u8] = if data.is_empty() {
        &[0u8; 16]
    } else {
        bytemuck::cast_slice(data)
    };
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: bytes,
        usage: wgpu::BufferUsages::STORAGE,
    })
}

pub fn shader_source() -> &'static str {
    include_str!("../shaders/pt.wgsl")
}

impl GpuTracer {
    pub fn new(g: &RenderGpu, scene: &PtScene, cam: PtCamera) -> GpuTracer {
        let device = &g.device;
        let bvh = Lbvh::build(scene);
        // triangles in leaf order
        let (mut v0, mut e1, mut e2) = (Vec::new(), Vec::new(), Vec::new());
        for &i in &bvh.order {
            let t = &scene.tris[i as usize];
            v0.push([
                t[0][0],
                t[0][1],
                t[0][2],
                f32::from_bits(scene.mat[i as usize]),
            ]);
            e1.push([t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2], 0.0]);
            e2.push([t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2], 0.0]);
        }
        let mats: Vec<GpuMat> = scene
            .materials
            .iter()
            .map(|m| GpuMat {
                albedo: m.albedo,
                rough: m.roughness,
                emission: m.emission,
                metallic: m.metallic,
                specular: m.specular,
                p: [0.0; 3],
            })
            .collect();
        let mut lights: Vec<[f32; 4]> = Vec::new();
        let mut acc = 0.0f32;
        for k in 0..bvh.order.len() {
            let m = &scene.materials[scene.mat[bvh.order[k] as usize] as usize];
            if m.is_emissive() {
                let (a, b) = (e1[k], e2[k]);
                let c = [
                    a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0],
                ];
                acc += 0.5 * (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
                lights.push([f32::from_bits(k as u32), acc, 0.0, 0.0]);
            }
        }
        let nodes = storage(device, &bvh.nodes, "pt_nodes");
        let b_v0 = storage(device, &v0, "pt_v0");
        let b_e1 = storage(device, &e1, "pt_e1");
        let b_e2 = storage(device, &e2, "pt_e2");
        let b_m = storage(device, &mats, "pt_mats");
        let b_l = storage(device, &lights, "pt_lights");
        let n_px = (cam.size.0 * cam.size.1) as u64;
        let accum = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pt_accum"),
            size: n_px * 16,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pt_readback"),
            size: n_px * 16,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("pt_params"),
            size: std::mem::size_of::<Params>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("pt"),
            source: wgpu::ShaderSource::Wgsl(shader_source().into()),
        });
        let entry = |binding: u32, ro: bool| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: ro },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("pt_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                entry(1, true),
                entry(2, true),
                entry(3, true),
                entry(4, true),
                entry(5, true),
                entry(6, true),
                entry(7, false),
            ],
        });
        // group 1 (the BRDF test harness) is declared by the module; give the main pipeline a layout for it too
        let bgl1 = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("pt_bgl1"),
            entries: &[entry(0, true), entry(1, false)],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("pt_layout"),
            bind_group_layouts: &[Some(&bgl), Some(&bgl1)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("pt"),
            layout: Some(&layout),
            module: &module,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("pt_bind"),
            layout: &bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: nodes.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: b_v0.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: b_e1.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: b_e2.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: b_m.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: b_l.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: accum.as_entire_binding(),
                },
            ],
        });
        let _ = LEAF;
        GpuTracer {
            pipeline,
            bind,
            params,
            accum,
            readback,
            size: cam.size,
            cam,
            env: [scene.env[0], scene.env[1], scene.env[2], 0.0],
            n_lights: lights.len() as u32,
            total_area: acc,
            max_depth: 8,
            samples: 0,
            nodes: bvh.nodes.len(),
        }
    }

    /// Add `spp` samples per pixel (in one dispatch per call).
    pub fn accumulate(&mut self, g: &RenderGpu, spp: u32, seed: u32) {
        let cam = &self.cam;
        let f = super::normalize(cam.forward);
        let r = super::normalize(super::cross(f, cam.up));
        let u = super::cross(r, f);
        let aspect = self.size.0 as f32 / self.size.1 as f32;
        let t = (cam.fov / 2.0).tan();
        let p = Params {
            origin: [cam.origin[0], cam.origin[1], cam.origin[2], 0.0],
            fwd: [f[0], f[1], f[2], 0.0],
            right: [r[0] * t * aspect, r[1] * t * aspect, r[2] * t * aspect, 0.0],
            up: [u[0] * t, u[1] * t, u[2] * t, 0.0],
            env: self.env,
            width: self.size.0,
            height: self.size.1,
            frame: self.samples,
            seed,
            n_lights: self.n_lights,
            max_depth: self.max_depth,
            spp,
            p0: 0,
            total_area: self.total_area,
            q: [0.0; 3],
        };
        g.queue
            .write_buffer(&self.params, 0, bytemuck::bytes_of(&p));
        let dummy = dummy_group1(g, &self.pipeline);
        let mut enc = g.device.create_command_encoder(&Default::default());
        {
            let mut pass = enc.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind, &[]);
            pass.set_bind_group(1, &dummy, &[]);
            pass.dispatch_workgroups(self.size.0.div_ceil(8), self.size.1.div_ceil(8), 1);
        }
        g.queue.submit([enc.finish()]);
        g.wait();
        self.samples += spp;
    }

    /// Mean radiance per pixel.
    pub fn image(&self, g: &RenderGpu) -> Vec<[f64; 3]> {
        let n = (self.size.0 * self.size.1) as u64;
        let mut enc = g.device.create_command_encoder(&Default::default());
        enc.copy_buffer_to_buffer(&self.accum, 0, &self.readback, 0, n * 16);
        g.queue.submit([enc.finish()]);
        let slice = self.readback.slice(..);
        slice.map_async(wgpu::MapMode::Read, |r| r.expect("map"));
        g.wait();
        let data: Vec<[f32; 4]> =
            bytemuck::cast_slice(&slice.get_mapped_range().expect("range")).to_vec();
        self.readback.unmap();
        let s = self.samples.max(1) as f64;
        data.iter()
            .map(|p| [p[0] as f64 / s, p[1] as f64 / s, p[2] as f64 / s])
            .collect()
    }

    pub fn reset(&mut self, g: &RenderGpu) {
        let n = (self.size.0 * self.size.1) as u64;
        g.queue
            .write_buffer(&self.accum, 0, &vec![0u8; (n * 16) as usize]);
        self.samples = 0;
    }
}

/// The module declares group 1 for the BRDF harness; the main entry point never touches it but the layout
/// needs a bind group.
fn dummy_group1(g: &RenderGpu, pipeline: &wgpu::ComputePipeline) -> wgpu::BindGroup {
    let a = g.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("dummy_in"),
        size: 64,
        usage: wgpu::BufferUsages::STORAGE,
        mapped_at_creation: false,
    });
    let b = g.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("dummy_out"),
        size: 64,
        usage: wgpu::BufferUsages::STORAGE,
        mapped_at_creation: false,
    });
    g.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("dummy1"),
        layout: &pipeline.get_bind_group_layout(1),
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: a.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: b.as_entire_binding(),
            },
        ],
    })
}

/// Evaluate the WGSL BRDF and pdf for a table of inputs `(n, wo, wi, albedo+roughness, metallic+specular)`.
pub fn brdf_on_gpu(g: &RenderGpu, inputs: &[[[f32; 4]; 5]]) -> Vec<[f32; 4]> {
    let device = &g.device;
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("pt_brdf"),
        source: wgpu::ShaderSource::Wgsl(shader_source().into()),
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("brdf_test"),
        layout: None,
        module: &module,
        entry_point: Some("brdf_test"),
        compilation_options: Default::default(),
        cache: None,
    });
    let bgl1 = pipeline.get_bind_group_layout(1);
    let input = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: None,
        contents: bytemuck::cast_slice(inputs),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let out_size = (inputs.len() * 16) as u64;
    let out = device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: out_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let read = device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: out_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bind1 = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &bgl1,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: input.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: out.as_entire_binding(),
            },
        ],
    });
    let mut enc = device.create_command_encoder(&Default::default());
    {
        let mut pass = enc.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(1, &bind1, &[]);
        pass.dispatch_workgroups((inputs.len() as u32).div_ceil(64), 1, 1);
    }
    enc.copy_buffer_to_buffer(&out, 0, &read, 0, out_size);
    g.queue.submit([enc.finish()]);
    let slice = read.slice(..);
    slice.map_async(wgpu::MapMode::Read, |r| r.expect("map"));
    g.wait();
    let v: Vec<[f32; 4]> = bytemuck::cast_slice(&slice.get_mapped_range().expect("range")).to_vec();
    read.unmap();
    v
}
