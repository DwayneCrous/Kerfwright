//! Offscreen rendering: frame-time benchmark, screenshots, and the scripted
//! pick-buffer correctness test.

use crate::camera::OrbitCamera;
use crate::gpu::Gpu;
use crate::renderer::{read_texture, Renderer};
use crate::scene::{self, PickTarget, Scene};
use nalgebra::Vector3;
use rayon::prelude::*;
use std::path::Path;
use std::time::Instant;

pub const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

pub struct Offscreen {
    pub tex: wgpu::Texture,
    pub view: wgpu::TextureView,
}

impl Offscreen {
    pub fn new(device: &wgpu::Device, w: u32, h: u32) -> Self {
        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("offscreen"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        Self { tex, view }
    }
}

#[derive(Debug)]
pub struct FrameStats {
    pub frames: usize,
    pub mean_ms: f64,
    pub p50_ms: f64,
    pub p95_ms: f64,
    pub p99_ms: f64,
    pub max_ms: f64,
    pub fps_mean: f64,
}

pub fn stats(mut ms: Vec<f64>) -> FrameStats {
    ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = ms.len();
    let pct = |p: f64| ms[((n as f64 * p).ceil() as usize).clamp(1, n) - 1];
    let mean = ms.iter().sum::<f64>() / n as f64;
    FrameStats {
        frames: n,
        mean_ms: mean,
        p50_ms: pct(0.5),
        p95_ms: pct(0.95),
        p99_ms: pct(0.99),
        max_ms: ms[n - 1],
        fps_mean: 1000.0 / mean,
    }
}

pub fn make_scene(name: &str, million_tris: usize) -> Scene {
    match name {
        "million" => scene::million_scene(million_tris),
        "primitives" => scene::primitives_scene(0.005),
        other => panic!("unknown scene {other:?} (use `primitives` or `million`)"),
    }
}

/// Render `frames` frames with a slowly orbiting camera and report GPU-inclusive
/// wall-clock frame times (each frame waits for GPU completion).
pub fn bench_frames(gpu: &Gpu, scene: &Scene, size: (u32, u32), frames: usize) -> FrameStats {
    let mut r = Renderer::new(&gpu.device, COLOR_FORMAT, size);
    r.upload(&gpu.device, scene);
    let off = Offscreen::new(&gpu.device, size.0, size.1);
    let mut cam = OrbitCamera::new(scene.radius, size.0 as f64 / size.1 as f64);
    for _ in 0..10 {
        r.render(&gpu.device, &gpu.queue, &cam, &off.view);
    }
    gpu.device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    let mut times = Vec::with_capacity(frames);
    for _ in 0..frames {
        cam.yaw += 0.01;
        let t = Instant::now();
        r.render(&gpu.device, &gpu.queue, &cam, &off.view);
        gpu.device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        times.push(t.elapsed().as_secs_f64() * 1000.0);
    }
    stats(times)
}

pub fn save_png(path: &Path, rgba: &[u8], w: u32, h: u32) {
    let file = std::fs::File::create(path).expect("create png");
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w, h);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header().unwrap().write_image_data(rgba).unwrap();
}

pub fn screenshot(gpu: &Gpu, scene: &Scene, size: (u32, u32), path: &Path) {
    let mut r = Renderer::new(&gpu.device, COLOR_FORMAT, size);
    r.upload(&gpu.device, scene);
    let off = Offscreen::new(&gpu.device, size.0, size.1);
    let cam = OrbitCamera::new(scene.radius, size.0 as f64 / size.1 as f64);
    r.render(&gpu.device, &gpu.queue, &cam, &off.view);
    let px = read_texture(&gpu.device, &gpu.queue, &off.tex, 0, 0, size.0, size.1, 4);
    save_png(path, &px, size.0, size.1);
}

#[derive(Debug, Default)]
pub struct PickReport {
    pub face_samples: usize,
    /// Of `face_samples`, how many landed on a surface (not background).
    pub face_surface_samples: usize,
    pub face_ambiguous: usize,
    pub face_mismatches: Vec<String>,
    pub edge_samples: usize,
    pub edge_skipped: usize,
    pub edge_mismatches: Vec<String>,
}

impl PickReport {
    pub fn passed(&self) -> bool {
        self.face_mismatches.is_empty()
            && self.edge_mismatches.is_empty()
            && self.face_samples > 0
            && self.edge_samples > 0
    }
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

fn cpu_id(scene: &Scene, cam: &OrbitCamera, px: f64, py: f64, w: f64, h: f64) -> u32 {
    let (o, d) = cam.pixel_ray(px, py, w, h);
    scene.raycast(o, d).map_or(0, |hit| hit.id)
}

/// The scripted click test, run from several camera views.
///
/// Face check: pixels sampled across the whole frame *and* at jittered
/// projections of random triangle centroids (so surfaces, not just
/// background, are exercised). The CPU ray-cast id must equal the ID buffer
/// wherever the answer is unambiguous (the four ±0.45 px neighbours agree
/// with the centre, i.e. not on a silhouette or face boundary).
///
/// Edge check: for several points along every edge, skip if occluded or
/// within 6 px of another edge, and require the ID buffer to hold that
/// edge's id at the projected pixel.
pub fn pick_test(gpu: &Gpu, scene: &Scene, size: (u32, u32), samples: usize) -> PickReport {
    let (w, h) = (size.0 as f64, size.1 as f64);
    let mut r = Renderer::new(&gpu.device, COLOR_FORMAT, size);
    r.settings.line_px = 3.0;
    r.upload(&gpu.device, scene);
    let off = Offscreen::new(&gpu.device, size.0, size.1);
    let mut report = PickReport::default();
    let views = [
        (-0.8, 0.45),
        (0.6, 0.9),
        (2.2, 0.3),
        (3.9, 0.6),
        (-2.2, 1.2),
    ];
    for (vi, (yaw, pitch)) in views.iter().enumerate() {
        let mut cam = OrbitCamera::new(scene.radius, w / h);
        cam.yaw = *yaw;
        cam.pitch = *pitch;
        cam.distance *= 0.62;
        cam.update_clip(scene.radius);
        face_check(
            gpu,
            &mut r,
            &off,
            scene,
            &cam,
            size,
            samples / views.len(),
            0x9E3779B97F4A7C15 ^ vi as u64,
            &mut report,
        );
        edge_check(gpu, &mut r, &off, scene, &cam, size, &mut report);
    }
    report
}

#[allow(clippy::too_many_arguments)]
fn face_check(
    gpu: &Gpu,
    r: &mut Renderer,
    off: &Offscreen,
    scene: &Scene,
    cam: &OrbitCamera,
    size: (u32, u32),
    samples: usize,
    seed: u64,
    report: &mut PickReport,
) {
    let (w, h) = (size.0 as f64, size.1 as f64);
    r.settings.show_edges = false;
    r.render(&gpu.device, &gpu.queue, cam, &off.view);
    let ids = r.read_ids(&gpu.device, &gpu.queue, 0, 0, size.0, size.1);
    let mut rng = Lcg(seed);
    let mut pts: Vec<(u32, u32)> = Vec::with_capacity(samples);
    while pts.len() < samples {
        if pts.len().is_multiple_of(2) {
            pts.push(((rng.next() * w) as u32, (rng.next() * h) as u32));
        } else {
            let t = (rng.next() * scene.triangle_count() as f64) as usize % scene.triangle_count();
            let c = (0..3)
                .map(|k| {
                    let p = scene.vertices[scene.indices[3 * t + k] as usize].pos;
                    Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64)
                })
                .sum::<Vector3<f64>>()
                / 3.0;
            if let Some((px, py)) = cam.project(&c, w, h) {
                let (x, y) = (px + (rng.next() - 0.5) * 3.0, py + (rng.next() - 0.5) * 3.0);
                if x >= 0.0 && y >= 0.0 && x < w && y < h {
                    pts.push((x as u32, y as u32));
                }
            }
        }
    }
    let results: Vec<(bool, bool, Option<String>)> = pts
        .par_iter()
        .map(|&(x, y)| {
            let (px, py) = (x as f64, y as f64);
            let centre = cpu_id(scene, cam, px, py, w, h);
            let ambiguous = [(0.45, 0.0), (-0.45, 0.0), (0.0, 0.45), (0.0, -0.45)]
                .iter()
                .any(|(dx, dy)| cpu_id(scene, cam, px + dx, py + dy, w, h) != centre);
            if ambiguous {
                return (true, false, None);
            }
            let gpu_id = ids[(y * size.0 + x) as usize];
            let err = (gpu_id != centre)
                .then(|| format!("pixel ({x},{y}): cpu id {centre}, gpu id {gpu_id}"));
            (false, centre != 0, err)
        })
        .collect();
    for (amb, on_surface, err) in results {
        if amb {
            report.face_ambiguous += 1;
        } else {
            report.face_samples += 1;
            if on_surface {
                report.face_surface_samples += 1;
            }
        }
        if let Some(e) = err {
            report.face_mismatches.push(e);
        }
    }
}

fn edge_check(
    gpu: &Gpu,
    r: &mut Renderer,
    off: &Offscreen,
    scene: &Scene,
    cam: &OrbitCamera,
    size: (u32, u32),
    report: &mut PickReport,
) {
    let (w, h) = (size.0 as f64, size.1 as f64);
    r.settings.show_edges = true;
    r.render(&gpu.device, &gpu.queue, cam, &off.view);
    let ids = r.read_ids(&gpu.device, &gpu.queue, 0, 0, size.0, size.1);
    let v3 = |p: [f32; 3]| Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64);
    type ScreenSegment = ((f64, f64), (f64, f64), u32);
    let seg_screen: Vec<Option<ScreenSegment>> = scene
        .segments
        .iter()
        .map(|s| {
            Some((
                cam.project(&v3(s.p0), w, h)?,
                cam.project(&v3(s.p1), w, h)?,
                s.id,
            ))
        })
        .collect();
    let mut by_edge: std::collections::BTreeMap<u32, Vec<usize>> = Default::default();
    for (i, s) in scene.segments.iter().enumerate() {
        by_edge.entry(s.id).or_default().push(i);
    }
    let dist_to_seg = |p: (f64, f64), a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        ((p.0 - a.0 - t * dx).powi(2) + (p.1 - a.1 - t * dy).powi(2)).sqrt()
    };
    let eye = cam.eye();
    for (id, segs) in by_edge {
        for frac in [0.25, 0.5, 0.75] {
            let s =
                &scene.segments[segs[((segs.len() as f64 * frac) as usize).min(segs.len() - 1)]];
            let mid = (v3(s.p0) + v3(s.p1)) * 0.5;
            let Some(pix) = cam.project(&mid, w, h) else {
                report.edge_skipped += 1;
                continue;
            };
            if pix.0 < 3.0 || pix.1 < 3.0 || pix.0 > w - 4.0 || pix.1 > h - 4.0 {
                report.edge_skipped += 1;
                continue;
            }
            let to = mid - eye;
            let dist = to.norm();
            // a front face clearly nearer than the edge point hides it
            let occluded = scene
                .raycast(eye, to / dist)
                .is_some_and(|hit| hit.t < dist - 0.003 * scene.radius);
            // silhouette-grazing edges are unstable targets: require the nearest
            // surface hit to be at the edge (within tolerance) or absent
            let crowded = seg_screen
                .iter()
                .flatten()
                .any(|(a, b, sid)| *sid != id && dist_to_seg(pix, *a, *b) < 6.0);
            if occluded || crowded {
                report.edge_skipped += 1;
                continue;
            }
            let (x, y) = (pix.0.round() as u32, pix.1.round() as u32);
            let got = ids[(y * size.0 + x) as usize];
            report.edge_samples += 1;
            if got != id {
                report
                    .edge_mismatches
                    .push(format!("edge id {id} at pixel ({x},{y}): gpu id {got}"));
            }
        }
    }
    for &v in ids.iter().step_by(97) {
        if v != 0 {
            assert!((v as usize) <= scene.pick.len(), "id {v} out of range");
            assert!(matches!(
                scene.pick[v as usize - 1],
                PickTarget::Face { .. } | PickTarget::Edge { .. }
            ));
        }
    }
}
