//! GPU test of the section plane: skipped (with a message) when the machine has no adapter.
//! Never run under `ulimit -v`.

use kw_viewer::headless::{Offscreen, COLOR_FORMAT};
use kw_viewer::renderer::read_texture;
use kw_viewer::scene::{Scene, Vertex};
use kw_viewer::{Gpu, OrbitCamera, Renderer};
use nalgebra::Vector3;

fn cube(half: f32) -> Scene {
    let mut scene = Scene {
        vertices: vec![],
        indices: vec![],
        segments: vec![],
        pick: vec![],
        center: Vector3::zeros(),
        radius: (half * 1.8) as f64,
    };
    // (normal, u, v) with u x v = normal: counter-clockwise seen from outside
    let faces = [
        ([0., 0., 1.], [1., 0., 0.], [0., 1., 0.]),
        ([0., 0., -1.], [0., 1., 0.], [1., 0., 0.]),
        ([1., 0., 0.], [0., 1., 0.], [0., 0., 1.]),
        ([-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]),
        ([0., 1., 0.], [0., 0., 1.], [1., 0., 0.]),
        ([0., -1., 0.], [1., 0., 0.], [0., 0., 1.]),
    ];
    for (n, u, v) in faces {
        let base = scene.vertices.len() as u32;
        for (a, b) in [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)] {
            let pos = [
                (n[0] + a * u[0] + b * v[0]) * half,
                (n[1] + a * u[1] + b * v[1]) * half,
                (n[2] + a * u[2] + b * v[2]) * half,
            ];
            scene.vertices.push(Vertex {
                pos,
                normal: n,
                id: 1,
                color: [190, 195, 205, 255],
            });
        }
        scene
            .indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    scene
}

/// Pixels that are the cap colour (strongly orange).
fn caps(px: &[u8]) -> usize {
    px.chunks(4)
        .filter(|p| p[0] > 200 && p[1] > 90 && p[1] < 190 && p[2] < 110)
        .count()
}

fn render(gpu: &Gpu, scene: &Scene, section: Option<([f64; 3], f64)>, pitch: f64) -> Vec<u8> {
    let size = (320, 240);
    let mut r = Renderer::new(&gpu.device, COLOR_FORMAT, size);
    r.upload(&gpu.device, scene);
    r.settings.section = section;
    let off = Offscreen::new(&gpu.device, size.0, size.1);
    let mut cam = OrbitCamera::new(scene.radius, size.0 as f64 / size.1 as f64);
    cam.pitch = pitch;
    r.render(&gpu.device, &gpu.queue, &cam, &off.view);
    read_texture(&gpu.device, &gpu.queue, &off.tex, 0, 0, size.0, size.1, 4)
}

#[test]
fn a_section_cuts_the_solid_and_shows_the_inside_as_a_cap() {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let Some(_) =
        pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).ok()
    else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let gpu = Gpu::headless();
    let scene = cube(5.0);
    // no section: a closed cube shows no cap, whichever way it is seen
    for pitch in [0.9, -0.9] {
        assert_eq!(caps(&render(&gpu, &scene, None, pitch)), 0, "pitch {pitch}");
    }
    // the top half (z > 0) cut away: from above the inside of the box is a cap …
    let cut = Some(([0.0, 0.0, 1.0], 0.0));
    let above = caps(&render(&gpu, &scene, cut, 0.9));
    assert!(above > 3000, "{above}");
    // … and from below the underside is plain
    // (perspective lets a sliver of the far walls' insides show at the silhouette corners)
    let below = caps(&render(&gpu, &scene, cut, -0.9));
    assert!(below < 200, "{below}");
    // cutting everything away leaves nothing of the cube
    let all = render(&gpu, &scene, Some(([0.0, 0.0, 1.0], -100.0)), 0.9);
    assert_eq!(caps(&all), 0);
    // a plane beyond the cube cuts nothing: the same picture as no section, up to the silhouette
    let none = render(&gpu, &scene, Some(([0.0, 0.0, 1.0], 100.0)), 0.9);
    let plain = render(&gpu, &scene, None, 0.9);
    let differing = none
        .chunks(4)
        .zip(plain.chunks(4))
        .filter(|(a, b)| a != b)
        .count();
    // only the one-pixel fringe along the silhouette differs (a back face owns some pixels of a shared
    // silhouette edge; the edge lines are drawn over it in the application)
    assert!(differing < 800, "{differing} pixels differ");
}
