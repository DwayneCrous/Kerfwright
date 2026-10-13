//! GPU tests of the analysis overlays (zebra, scalar ramps, draft): skipped (with a message) when the
//! machine has no adapter. Never run under `ulimit -v`.

use kw_analysis::ramp;
use kw_viewer::headless::{Offscreen, COLOR_FORMAT};
use kw_viewer::renderer::{read_texture, Analysis, AnalysisMode};
use kw_viewer::scene::{Scene, Vertex};
use kw_viewer::{Gpu, OrbitCamera, Renderer};
use nalgebra::Vector3;

const SIZE: (u32, u32) = (320, 240);

fn empty(radius: f64) -> Scene {
    Scene {
        vertices: vec![],
        indices: vec![],
        segments: vec![],
        pick: vec![],
        center: Vector3::zeros(),
        radius,
    }
}

fn cube(half: f32) -> Scene {
    let mut scene = empty((half * 1.8) as f64);
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

/// A unit-ish UV sphere with exact normals.
fn sphere(r: f32) -> Scene {
    let mut scene = empty(r as f64 * 1.6);
    let (nu, nv) = (64u32, 32u32);
    for j in 0..=nv {
        let th = std::f32::consts::PI * j as f32 / nv as f32;
        for i in 0..=nu {
            let ph = std::f32::consts::TAU * i as f32 / nu as f32;
            let n = [th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()];
            scene.vertices.push(Vertex {
                pos: [n[0] * r, n[1] * r, n[2] * r],
                normal: n,
                id: 1,
                color: [190, 195, 205, 255],
            });
        }
    }
    for j in 0..nv {
        for i in 0..nu {
            let a = j * (nu + 1) + i;
            let b = a + nu + 1;
            // counter-clockwise seen from outside
            scene
                .indices
                .extend_from_slice(&[a, a + 1, b, a + 1, b + 1, b]);
        }
    }
    scene
}

/// A square in the xy plane facing +z whose vertex scalars are given (corner order -x-y, +x-y, +x+y, -x+y).
fn quad(half: f32) -> Scene {
    let mut scene = empty(half as f64 * 1.6);
    for (a, b) in [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)] {
        scene.vertices.push(Vertex {
            pos: [a * half, b * half, 0.0],
            normal: [0., 0., 1.],
            id: 1,
            color: [190, 195, 205, 255],
        });
    }
    scene.indices = vec![0, 1, 2, 0, 2, 3];
    scene
}

fn gpu() -> Option<Gpu> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).ok()?;
    Some(Gpu::headless())
}

struct Shot {
    px: Vec<u8>,
}

impl Shot {
    fn at(&self, x: u32, y: u32) -> [u8; 3] {
        let i = ((y * SIZE.0 + x) * 4) as usize;
        [self.px[i], self.px[i + 1], self.px[i + 2]]
    }
}

fn render(
    gpu: &Gpu,
    scene: &Scene,
    scalars: Option<&[f32]>,
    analysis: Analysis,
    section: Option<([f64; 3], f64)>,
    pitch: f64,
) -> Shot {
    let mut r = Renderer::new(&gpu.device, COLOR_FORMAT, SIZE);
    r.upload(&gpu.device, scene);
    if let Some(s) = scalars {
        r.upload_scalars(&gpu.device, s, scene.vertices.len());
    }
    r.settings.analysis = analysis;
    r.settings.section = section;
    r.settings.show_edges = false;
    let off = Offscreen::new(&gpu.device, SIZE.0, SIZE.1);
    let mut cam = OrbitCamera::new(scene.radius, SIZE.0 as f64 / SIZE.1 as f64);
    cam.pitch = pitch;
    r.render(&gpu.device, &gpu.queue, &cam, &off.view);
    Shot {
        px: read_texture(&gpu.device, &gpu.queue, &off.tex, 0, 0, SIZE.0, SIZE.1, 4),
    }
}

fn to_u8(c: [f32; 3]) -> [i32; 3] {
    [
        (c[0] * 255.0).round() as i32,
        (c[1] * 255.0).round() as i32,
        (c[2] * 255.0).round() as i32,
    ]
}

fn near(a: [u8; 3], b: [i32; 3], tol: i32) -> bool {
    (0..3).all(|k| (i32::from(a[k]) - b[k]).abs() <= tol)
}

#[test]
fn a_scalar_ramp_shows_exactly_the_cpu_colour_at_the_interpolated_value() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let q = quad(4.0);
    // 0 on the left, 1 on the right: the middle of the quad is 0.5
    let sc = [0.0, 1.0, 1.0, 0.0];
    let a = Analysis {
        mode: AnalysisMode::Scalar,
        min: 0.0,
        max: 1.0,
        ..Analysis::default()
    };
    let shot = render(&gpu, &q, Some(&sc), a, None, std::f64::consts::FRAC_PI_2);
    let mid = shot.at(SIZE.0 / 2, SIZE.1 / 2);
    assert!(
        near(mid, to_u8(ramp::sequential(0.5)), 4),
        "{mid:?} vs {:?}",
        to_u8(ramp::sequential(0.5))
    );
    // a quarter of the way in: scale the range so the same pixel is a quarter along the ramp
    let a = Analysis {
        mode: AnalysisMode::Scalar,
        min: -0.5,
        max: 1.5,
        ..Analysis::default()
    };
    let shot = render(&gpu, &q, Some(&sc), a, None, std::f64::consts::FRAC_PI_2);
    let mid = shot.at(SIZE.0 / 2, SIZE.1 / 2);
    assert!(near(mid, to_u8(ramp::sequential(0.5)), 4), "{mid:?}");
    // signed ramp: zero is the neutral middle, a negative value leans blue
    let a = Analysis {
        mode: AnalysisMode::Signed,
        max: 1.0,
        ..Analysis::default()
    };
    let neutral = render(
        &gpu,
        &q,
        Some(&[-1.0, 1.0, 1.0, -1.0]),
        a,
        None,
        std::f64::consts::FRAC_PI_2,
    )
    .at(SIZE.0 / 2, SIZE.1 / 2);
    assert!(near(neutral, to_u8(ramp::diverging(0.0)), 4), "{neutral:?}");
    let left = render(
        &gpu,
        &q,
        Some(&[-1.0, 1.0, 1.0, -1.0]),
        a,
        None,
        std::f64::consts::FRAC_PI_2,
    )
    .at(SIZE.0 / 2 - 40, SIZE.1 / 2);
    assert!(left[2] > left[0], "negative leans blue: {left:?}");
    // a vertex without a value (NaN) makes grey, not garbage
    let grey = render(
        &gpu,
        &q,
        Some(&[f32::NAN; 4]),
        Analysis {
            mode: AnalysisMode::Scalar,
            ..Analysis::default()
        },
        None,
        std::f64::consts::FRAC_PI_2,
    )
    .at(SIZE.0 / 2, SIZE.1 / 2);
    assert!(near(grey, [128, 128, 128], 4), "{grey:?}");
}

/// How many times the colour flips between white and black along a row of the picture.
fn flips(shot: &Shot, y: u32, x0: u32, x1: u32) -> usize {
    let mut last = None;
    let mut n = 0;
    for x in x0..x1 {
        let c = shot.at(x, y);
        let state = if c[0] > 200 && c[1] > 200 {
            Some(true)
        } else if c[0] < 60 && c[1] < 60 {
            Some(false)
        } else {
            None
        };
        if let Some(s) = state {
            if last.is_some_and(|l| l != s) {
                n += 1;
            }
            last = Some(s);
        }
    }
    n
}

#[test]
fn zebra_stripes_break_up_a_sphere_and_barely_touch_a_flat_face() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let a = Analysis {
        mode: AnalysisMode::Zebra,
        axis: [1.0, 0.0, 0.0],
        zebra_freq: 4.0,
        duty: 0.5,
        ..Analysis::default()
    };
    let s = render(&gpu, &sphere(5.0), None, a, None, 0.0);
    let on_sphere = flips(&s, SIZE.1 / 2, 60, SIZE.0 - 60);
    assert!(on_sphere >= 6, "{on_sphere} stripes edges across a sphere");
    // a flat face seen from above: the reflection hardly changes across it
    let c = render(
        &gpu,
        &cube(5.0),
        None,
        Analysis {
            zebra_freq: 1.0,
            axis: [0.0, 0.0, 1.0],
            ..a
        },
        None,
        1.5,
    );
    let on_flat = flips(&c, SIZE.1 / 2, 110, 210);
    assert!(on_flat <= 1, "{on_flat}");
    // off: the ordinary lit picture, not stripes
    let plain = render(&gpu, &sphere(5.0), None, Analysis::default(), None, 0.0);
    assert!(flips(&plain, SIZE.1 / 2, 60, SIZE.0 - 60) <= 1);
}

#[test]
fn draft_colours_walls_red_and_floors_by_which_way_they_open() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let a = Analysis {
        mode: AnalysisMode::Draft,
        axis: [0.0, 0.0, 1.0],
        max: 3.0,
        ..Analysis::default()
    };
    // from above (pitch 0.9): the top face (+90 deg, opens along the pull) and two walls (0 deg)
    let shot = render(&gpu, &cube(5.0), None, a, None, 0.9);
    let want_wall = to_u8(ramp::draft(0.0, 3.0));
    let want_top = to_u8(ramp::draft(90.0, 3.0));
    let (mut walls, mut tops) = (0, 0);
    for y in 0..SIZE.1 {
        for x in 0..SIZE.0 {
            let c = shot.at(x, y);
            if near(c, want_wall, 3) {
                walls += 1;
            }
            if near(c, want_top, 3) {
                tops += 1;
            }
        }
    }
    assert!(walls > 1500 && tops > 1500, "walls {walls}, top {tops}");
    // pulling the other way turns the top face into the wrong-way (bluish) colour
    let back = Analysis {
        axis: [0.0, 0.0, -1.0],
        ..a
    };
    let shot = render(&gpu, &cube(5.0), None, back, None, 0.9);
    let want_wrong = to_u8(ramp::draft(-90.0, 3.0));
    let wrong = shot
        .px
        .chunks(4)
        .filter(|p| near([p[0], p[1], p[2]], want_wrong, 3))
        .count();
    assert!(wrong > 1500, "{wrong}");
}

#[test]
fn the_section_cap_stays_orange_under_analysis() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let cut = Some(([0.0, 0.0, 1.0], 0.0));
    let a = Analysis {
        mode: AnalysisMode::Zebra,
        ..Analysis::default()
    };
    let shot = render(&gpu, &cube(5.0), None, a, cut, 0.9);
    let caps = shot
        .px
        .chunks(4)
        .filter(|p| p[0] > 200 && p[1] > 90 && p[1] < 190 && p[2] < 110)
        .count();
    assert!(caps > 3000, "{caps}");
}
