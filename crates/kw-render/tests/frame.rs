//! GPU tests: skipped (with a message) when the machine has no adapter. Never run these under `ulimit -v`.

use kw_render::{offscreen, read_rgba, Frame, FrameOptions, RenderGpu};
use kw_viewer::scene::{Scene, Vertex};
use kw_viewer::OrbitCamera;
use nalgebra::Vector3;

fn gpu() -> Option<RenderGpu> {
    let g = RenderGpu::try_new();
    if g.is_none() {
        eprintln!("no GPU adapter: test skipped");
    }
    g
}

fn quad(
    scene: &mut Scene,
    a: [f32; 3],
    b: [f32; 3],
    c: [f32; 3],
    d: [f32; 3],
    n: [f32; 3],
    color: [u8; 4],
) {
    let base = scene.vertices.len() as u32;
    for p in [a, b, c, d] {
        scene.vertices.push(Vertex {
            pos: p,
            normal: n,
            id: 1,
            color,
        });
    }
    scene
        .indices
        .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

fn empty_scene(radius: f64) -> Scene {
    Scene {
        vertices: vec![],
        indices: vec![],
        segments: vec![],
        pick: vec![],
        center: Vector3::zeros(),
        radius,
    }
}

fn f16(h: u16) -> f32 {
    let (s, e, m) = ((h >> 15) & 1, (h >> 10) & 0x1f, h & 0x3ff);
    let v = match e {
        0 => (m as f32) * 2f32.powi(-24),
        31 => f32::INFINITY,
        _ => (1.0 + m as f32 / 1024.0) * 2f32.powi(e as i32 - 15),
    };
    if s == 1 {
        -v
    } else {
        v
    }
}

fn hdr_at(px: &[u8], w: u32, x: u32, y: u32) -> [f32; 3] {
    let o = ((y * w + x) * 8) as usize;
    let h = |k: usize| f16(u16::from_le_bytes([px[o + 2 * k], px[o + 2 * k + 1]]));
    [h(0), h(1), h(2)]
}

fn aces(x: f32) -> f32 {
    ((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)).clamp(0.0, 1.0)
}
fn srgb(l: f32) -> f32 {
    if l <= 0.0031308 {
        12.92 * l
    } else {
        1.055 * l.powf(1.0 / 2.4) - 0.055
    }
}

fn render_to_rgba(
    g: &RenderGpu,
    frame: &Frame,
    cam: &OrbitCamera,
    size: (u32, u32),
    o: &FrameOptions,
) -> Vec<u8> {
    let (tex, view) = offscreen(&g.device, size);
    frame.render(&g.device, &g.queue, cam, &view, o);
    read_rgba(g, &tex, size)
}

#[test]
fn weighted_blended_oit_matches_the_blend_equation() {
    let Some(g) = gpu() else { return };
    let size = (128, 128);
    let cam = {
        let mut c = OrbitCamera::new(3.0, 1.0);
        c.yaw = std::f64::consts::FRAC_PI_2 * 3.0;
        c.pitch = 0.0;
        c.update_clip(3.0);
        c
    };
    let mk = |alpha: u8| {
        let mut s = empty_scene(3.0);
        // a quad in the x–z plane facing the camera, which looks along +y from y = -distance
        quad(
            &mut s,
            [-1.0, 0.0, -1.0],
            [1.0, 0.0, -1.0],
            [1.0, 0.0, 1.0],
            [-1.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [220, 60, 40, alpha],
        );
        s
    };
    let o = FrameOptions {
        ao: false,
        edges: false,
        ..Default::default()
    };
    let mut f = Frame::new(&g.device, &g.queue, size, g.timestamps);
    f.upload(&g.device, &mk(255));
    let (tex, view) = offscreen(&g.device, size);
    f.render(&g.device, &g.queue, &cam, &view, &o);
    let hdr = f.read_hdr(&g.device, &g.queue);
    let c = hdr_at(&hdr, size.0, 64, 64);
    let bgl = hdr_at(&hdr, size.0, 2, 2);
    assert!(c[0] > 0.05, "the opaque quad is lit: {c:?}");
    drop(tex);
    // the same quad at alpha 128
    f.upload(&g.device, &mk(128));
    let px = render_to_rgba(&g, &f, &cam, size, &o);
    let a = 128.0 / 255.0;
    let p = &px[((64 * size.0 + 64) * 4) as usize..][..3];
    for k in 0..3 {
        let expect = srgb(aces(a * c[k] + (1.0 - a) * bgl[k])) * 255.0;
        assert!(
            (p[k] as f32 - expect).abs() < 3.0,
            "channel {k}: got {} expected {expect:.1}",
            p[k]
        );
    }
    // outside the quad nothing changes: still the background
    let corner = &px[((2 * size.0 + 2) * 4) as usize..][..3];
    for k in 0..3 {
        let expect = srgb(aces(bgl[k])) * 255.0;
        assert!((corner[k] as f32 - expect).abs() < 2.0);
    }
}

#[test]
fn two_layers_blend_close_to_sorted_over() {
    let Some(g) = gpu() else { return };
    let size = (96, 96);
    let mut cam = OrbitCamera::new(3.0, 1.0);
    cam.yaw = std::f64::consts::FRAC_PI_2 * 3.0;
    cam.pitch = 0.0;
    cam.update_clip(3.0);
    let o = FrameOptions {
        ao: false,
        edges: false,
        ..Default::default()
    };
    let mut s = empty_scene(3.0);
    quad(
        &mut s,
        [-1.0, 0.3, -1.0],
        [1.0, 0.3, -1.0],
        [1.0, 0.3, 1.0],
        [-1.0, 0.3, 1.0],
        [0.0, -1.0, 0.0],
        [40, 80, 220, 128],
    );
    quad(
        &mut s,
        [-1.0, -0.3, -1.0],
        [1.0, -0.3, -1.0],
        [1.0, -0.3, 1.0],
        [-1.0, -0.3, 1.0],
        [0.0, -1.0, 0.0],
        [220, 60, 40, 128],
    );
    let mut f = Frame::new(&g.device, &g.queue, size, g.timestamps);
    f.upload(&g.device, &s);
    let px = render_to_rgba(&g, &f, &cam, size, &o);
    let hdr_pixel = &px[((48 * size.0 + 48) * 4) as usize..][..3];
    // single layers for the reference "over" operator in linear light
    let single = |back: bool| {
        let mut s1 = empty_scene(3.0);
        let (y, col) = if back {
            (0.3, [40u8, 80, 220, 255])
        } else {
            (-0.3, [220u8, 60, 40, 255])
        };
        quad(
            &mut s1,
            [-1.0, y, -1.0],
            [1.0, y, -1.0],
            [1.0, y, 1.0],
            [-1.0, y, 1.0],
            [0.0, -1.0, 0.0],
            col,
        );
        let mut f1 = Frame::new(&g.device, &g.queue, size, g.timestamps);
        f1.upload(&g.device, &s1);
        let (_t, v) = offscreen(&g.device, size);
        f1.render(&g.device, &g.queue, &cam, &v, &o);
        let h = f1.read_hdr(&g.device, &g.queue);
        (hdr_at(&h, size.0, 48, 48), hdr_at(&h, size.0, 2, 2))
    };
    let (cb, bgc) = single(true);
    let (cf, _) = single(false);
    let a = 128.0 / 255.0;
    let mut worst: f32 = 0.0;
    for k in 0..3 {
        let behind = a * cb[k] + (1.0 - a) * bgc[k];
        let over = a * cf[k] + (1.0 - a) * behind;
        let expect = srgb(aces(over)) * 255.0;
        worst = worst.max((hdr_pixel[k] as f32 - expect).abs());
    }
    eprintln!("WBOIT vs exact sorted blend, two layers: worst channel difference {worst:.1} / 255");
    assert!(worst < 32.0, "{worst}");
}

fn ao_at(ao: &[u8], size: (u32, u32), cam: &OrbitCamera, full: (u32, u32), p: [f64; 3]) -> f32 {
    let (x, y) = cam
        .project(
            &Vector3::new(p[0], p[1], p[2]),
            full.0 as f64,
            full.1 as f64,
        )
        .expect("visible");
    let (hx, hy) = (
        ((x / 2.0) as u32).min(size.0 - 1),
        ((y / 2.0) as u32).min(size.1 - 1),
    );
    ao[(hy * size.0 + hx) as usize] as f32 / 255.0
}

#[test]
fn gtao_darkens_a_concave_corner_and_leaves_flat_ground_alone() {
    let Some(g) = gpu() else { return };
    let full = (256, 256);
    let mut s = empty_scene(8.0);
    // floor z = 0 and a wall x = 0 (facing +x)
    quad(
        &mut s,
        [-8.0, -8.0, 0.0],
        [8.0, -8.0, 0.0],
        [8.0, 8.0, 0.0],
        [-8.0, 8.0, 0.0],
        [0.0, 0.0, 1.0],
        [180, 180, 180, 255],
    );
    quad(
        &mut s,
        [0.0, -8.0, 0.0],
        [0.0, 8.0, 0.0],
        [0.0, 8.0, 6.0],
        [0.0, -8.0, 6.0],
        [1.0, 0.0, 0.0],
        [180, 180, 180, 255],
    );
    let mut cam = OrbitCamera::new(8.0, 1.0);
    cam.yaw = 0.7;
    cam.pitch = 0.6;
    cam.distance *= 0.8;
    cam.update_clip(8.0);
    let mut f = Frame::new(&g.device, &g.queue, full, g.timestamps);
    f.upload(&g.device, &s);
    let o = FrameOptions {
        ao_radius: 1.5,
        edges: false,
        ..Default::default()
    };
    let (_t, v) = offscreen(&g.device, full);
    f.render(&g.device, &g.queue, &cam, &v, &o);
    let (ao, half) = f.read_ao(&g.device, &g.queue);
    let corner = ao_at(&ao, half, &cam, full, [0.25, 0.0, 0.0]);
    let wall_low = ao_at(&ao, half, &cam, full, [0.0, 0.0, 0.25]);
    let far_floor = ao_at(&ao, half, &cam, full, [5.0, 0.0, 0.0]);
    let high_wall = ao_at(&ao, half, &cam, full, [0.0, 0.0, 5.0]);
    eprintln!("AO: floor at the wall {corner:.3}, wall at the floor {wall_low:.3}, far floor {far_floor:.3}, high wall {high_wall:.3}");
    assert!(
        corner < 0.8 && wall_low < 0.85,
        "the corner must be occluded"
    );
    assert!(
        far_floor > 0.97 && high_wall > 0.9,
        "open surfaces stay open"
    );
    // a plane alone: AO = 1 everywhere it is visible
    let mut flat = empty_scene(8.0);
    quad(
        &mut flat,
        [-8.0, -8.0, 0.0],
        [8.0, -8.0, 0.0],
        [8.0, 8.0, 0.0],
        [-8.0, 8.0, 0.0],
        [0.0, 0.0, 1.0],
        [180, 180, 180, 255],
    );
    f.upload(&g.device, &flat);
    f.render(&g.device, &g.queue, &cam, &v, &o);
    let (ao, half) = f.read_ao(&g.device, &g.queue);
    let mut minv = 255u8;
    for y in (half.1 / 4)..(3 * half.1 / 4) {
        for x in (half.0 / 4)..(3 * half.0 / 4) {
            minv = minv.min(ao[(y * half.0 + x) as usize]);
        }
    }
    eprintln!("flat plane: minimum AO {:.3}", minv as f32 / 255.0);
    assert!(minv as f32 / 255.0 > 0.97);
}

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

/// Frame time at 1M triangles with edges, GTAO and OIT. Prints the numbers; asserts only that the frame runs
/// (the 60 FPS criterion is judged from the printed p95 GPU time in the report).
#[test]
fn frame_time_at_one_million_triangles() {
    let Some(g) = gpu() else { return };
    if !g.timestamps {
        eprintln!("no timestamp queries on this adapter: skipped");
        return;
    }
    let tris = env_usize("KW_RENDER_TRIS", 1_000_000);
    let frames = env_usize("KW_RENDER_FRAMES", 600);
    let mut scene = kw_viewer::headless::make_scene("million", tris);
    // about 30 % of the triangles transparent
    for (i, v) in scene.vertices.iter_mut().enumerate() {
        if (i / 1000) % 10 < 3 {
            v.color[3] = 110;
        }
    }
    eprintln!("adapter: {}", g.describe());
    for size in [(1920u32, 1080u32), (1280, 720)] {
        let mut f = Frame::new(&g.device, &g.queue, size, true);
        f.upload(&g.device, &scene);
        let (_t, v) = offscreen(&g.device, size);
        let mut cam = OrbitCamera::new(scene.radius, size.0 as f64 / size.1 as f64);
        cam.distance *= 0.7;
        cam.update_clip(scene.radius);
        let o = FrameOptions::default();
        for _ in 0..10 {
            f.render(&g.device, &g.queue, &cam, &v, &o);
        }
        g.wait();
        let mut totals = Vec::new();
        let mut sum = [0.0f64; 5];
        for _ in 0..frames {
            cam.yaw += 0.01;
            f.render(&g.device, &g.queue, &cam, &v, &o);
            let t = f.read_times(&g.device).expect("timestamps");
            totals.push(t.total_ms);
            for (s, x) in
                sum.iter_mut()
                    .zip([t.opaque_ms, t.oit_ms, t.ao_ms, t.composite_ms, t.edges_ms])
            {
                *s += x;
            }
        }
        totals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p =
            |q: f64| totals[((totals.len() as f64 * q).ceil() as usize).clamp(1, totals.len()) - 1];
        let n = frames as f64;
        eprintln!(
            "{}x{} {} tris: GPU frame p50 {:.2} ms, p95 {:.2} ms, max {:.2} ms (mean per pass: opaque {:.2}, oit {:.2}, gtao {:.2}, composite {:.2}, edges {:.2})",
            size.0, size.1, f.triangles, p(0.5), p(0.95), totals[totals.len() - 1], sum[0] / n, sum[1] / n, sum[2] / n, sum[3] / n, sum[4] / n
        );
    }
}
