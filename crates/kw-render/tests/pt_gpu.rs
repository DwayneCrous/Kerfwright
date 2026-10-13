#![allow(clippy::type_complexity)]
//! GPU path tracer against the CPU reference and the closed forms. Skipped without an adapter; never run these
//! under `ulimit -v`.

use kw_render::pt::cpu_ref::{brdf, pdf, Mat, Reference};
use kw_render::pt::denoise::{denoise, DenoiseParams, Guides};
use kw_render::pt::gpu::{brdf_on_gpu, GpuTracer};
use kw_render::pt::metrics::{psnr_tonemapped, relative_mse};
use kw_render::pt::{normalize, scenes, Material, PtCamera, PtScene};
use kw_render::RenderGpu;

fn gpu() -> Option<RenderGpu> {
    let g = RenderGpu::try_new();
    if g.is_none() {
        eprintln!("no GPU adapter: test skipped");
    }
    g
}

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn rnd(s: &mut u64) -> f64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    (*s >> 11) as f64 / (1u64 << 53) as f64
}

fn mean(img: &[[f64; 3]]) -> [f64; 3] {
    let n = img.len() as f64;
    let mut m = [0.0; 3];
    for p in img {
        for k in 0..3 {
            m[k] += p[k] / n;
        }
    }
    m
}

fn run(g: &RenderGpu, s: &PtScene, cam: PtCamera, spp: u32) -> Vec<[f64; 3]> {
    let mut t = GpuTracer::new(g, s, cam);
    let mut done = 0;
    while done < spp {
        let n = 64.min(spp - done);
        t.accumulate(g, n, 7 + done);
        done += n;
    }
    t.image(g)
}

#[test]
fn wgsl_brdf_matches_the_f64_brdf() {
    let Some(g) = gpu() else { return };
    let mut seed = 5u64;
    let mut inputs: Vec<[[f32; 4]; 5]> = Vec::new();
    let mut cases: Vec<(Mat, [f64; 3], [f64; 3], [f64; 3])> = Vec::new();
    while inputs.len() < 4096 {
        let dir = |s: &mut u64| {
            let v = normalize([
                (rnd(s) - 0.5) as f32,
                (rnd(s) - 0.5) as f32,
                (rnd(s) - 0.5) as f32,
            ]);
            [v[0], v[1], v[2]]
        };
        let n = dir(&mut seed);
        let (wo, wi) = (dir(&mut seed), dir(&mut seed));
        let dotp = |a: [f32; 3], b: [f32; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        // keep clearly on the lit side (the grazing limit is where f32 and f64 may differ in sign)
        if dotp(n, wo) < 0.05 || dotp(n, wi) < 0.05 {
            continue;
        }
        let m = Material {
            albedo: [
                rnd(&mut seed) as f32,
                rnd(&mut seed) as f32,
                rnd(&mut seed) as f32,
            ],
            roughness: (0.05 + 0.95 * rnd(&mut seed)) as f32,
            metallic: if rnd(&mut seed) < 0.3 {
                1.0
            } else {
                rnd(&mut seed) as f32
            },
            specular: if rnd(&mut seed) < 0.2 { 0.0 } else { 1.0 },
            emission: [0.0; 3],
        };
        inputs.push([
            [n[0], n[1], n[2], 0.0],
            [wo[0], wo[1], wo[2], 0.0],
            [wi[0], wi[1], wi[2], 0.0],
            [m.albedo[0], m.albedo[1], m.albedo[2], m.roughness],
            [m.metallic, m.specular, 0.0, 0.0],
        ]);
        let up = |a: [f32; 3]| [a[0] as f64, a[1] as f64, a[2] as f64];
        cases.push((Mat::from(&m), up(n), up(wo), up(wi)));
    }
    let out = brdf_on_gpu(&g, &inputs);
    let mut worst = 0.0f64;
    for (o, (m, n, wo, wi)) in out.iter().zip(&cases) {
        let f = brdf(m, *n, *wo, *wi);
        let p = pdf(m, *n, *wo, *wi);
        for k in 0..3 {
            worst = worst.max((o[k] as f64 - f[k]).abs() / f[k].abs().max(1e-2));
        }
        worst = worst.max((o[3] as f64 - p).abs() / p.abs().max(1e-2));
    }
    eprintln!(
        "WGSL vs f64 BRDF and pdf over {} cases: worst relative difference {worst:.2e}",
        cases.len()
    );
    assert!(worst < 2e-3, "{worst}");
}

#[test]
fn gpu_furnace_and_disc_light_match_the_closed_forms() {
    let Some(g) = gpu() else { return };
    // Lambert furnace
    let mut s = PtScene::new([1.0; 3]);
    let m = s.add_material(Material::lambert([1.0; 3]));
    s.add_sphere([0.0; 3], 1.0, 3, m);
    let cam = PtCamera {
        origin: [0.0, -4.0, 0.0],
        forward: [0.0, 1.0, 0.0],
        up: [0.0, 0.0, 1.0],
        fov: 0.2,
        size: (24, 24),
    };
    let v = mean(&run(&g, &s, cam, 512));
    eprintln!("GPU Lambert furnace: {v:.5?}");
    assert!((v[0] - 1.0).abs() < 3e-3);
    // GGX on a plane: directional albedo, GPU against the CPU reference (itself checked against quadrature)
    for (rough, metallic) in [(0.2f32, 1.0f32), (0.5, 1.0), (1.0, 1.0), (0.4, 0.0)] {
        let mut s = PtScene::new([1.0; 3]);
        let m = s.add_material(Material::glossy([1.0; 3], rough, metallic));
        s.add_quad(
            [-1e3, -1e3, 0.0],
            [1e3, -1e3, 0.0],
            [1e3, 1e3, 0.0],
            [-1e3, 1e3, 0.0],
            m,
        );
        let cam = PtCamera {
            origin: [0.0, 0.0, 3.0],
            forward: [0.0, 0.0, -1.0],
            up: [0.0, 1.0, 0.0],
            fov: 0.02,
            size: (8, 8),
        };
        let gpu_v = mean(&run(&g, &s, cam, 4096));
        let cpu_v = mean(&Reference::new(&s).render(&cam, 4000, 5));
        eprintln!(
            "GGX rough {rough} metallic {metallic}: GPU {:.4}, CPU reference {:.4}",
            gpu_v[0], cpu_v[0]
        );
        assert!((gpu_v[0] - cpu_v[0]).abs() < 0.01 * cpu_v[0].max(0.2));
        assert!(gpu_v[0] <= 1.005);
    }
    // disc light
    let (rho, l, big_r, h) = (0.6f32, 5.0f32, 1.3f32, 1.7f32);
    let mut s = PtScene::new([0.0; 3]);
    let plane = s.add_material(Material::lambert([rho; 3]));
    let lamp = s.add_material(Material::light([l; 3]));
    s.add_quad(
        [-1e3, -1e3, 0.0],
        [1e3, -1e3, 0.0],
        [1e3, 1e3, 0.0],
        [-1e3, 1e3, 0.0],
        plane,
    );
    s.add_disc([0.0, 0.0, h], [0.0, 0.0, -1.0], big_r, 512, lamp);
    let cam = PtCamera {
        origin: [3.5, 0.0, 0.9],
        forward: normalize([-3.5, 0.0, -0.9]),
        up: [0.0, 0.0, 1.0],
        fov: 0.004,
        size: (6, 6),
    };
    let v = mean(&run(&g, &s, cam, 2048));
    let want = (rho * l * big_r * big_r / (h * h + big_r * big_r)) as f64;
    eprintln!("GPU disc light: {:.5}, closed form {want:.5}", v[0]);
    assert!((v[0] - want).abs() / want < 0.01);
}

#[test]
fn cornell_box_converges_to_the_reference_and_the_denoiser_reaches_30_db() {
    let Some(g) = gpu() else { return };
    let (scene, cam) = scenes::cornell();
    let ref_spp = env_usize("KW_PT_REF_SPP", 4096);
    let reference = Reference::new(&scene);
    let t = std::time::Instant::now();
    let rimg = reference.render(&cam, ref_spp, 11);
    eprintln!(
        "CPU reference: {} spp at {}x{} in {:.1} s",
        ref_spp,
        cam.size.0,
        cam.size.1,
        t.elapsed().as_secs_f64()
    );
    // GPU convergence curve
    let mut tracer = GpuTracer::new(&g, &scene, cam);
    let mut curve: Vec<(u32, f64)> = Vec::new();
    let mut done = 0u32;
    let mut first_ok: Option<u32> = None;
    for target in [16u32, 64, 256, 1024, 4096] {
        while done < target {
            let n = 32.min(target - done);
            tracer.accumulate(&g, n, 100 + done);
            done += n;
        }
        let e = relative_mse(&tracer.image(&g), &rimg);
        curve.push((target, e));
        if e <= 0.01 && first_ok.is_none() {
            first_ok = Some(target);
        }
    }
    for (n, e) in &curve {
        eprintln!("GPU {n:>5} spp: relative MSE vs reference {:.3e}", e);
    }
    let n_ok = first_ok.expect("the GPU tracer never reached 1 % relative MSE");
    eprintln!(
        "relative MSE <= 1 % reached at N = {n_ok} spp (reference noise floor: {ref_spp} spp)"
    );
    // denoiser: a low-spp GPU image against the reference, after tonemapping
    let guides_raw = reference.guides(&cam);
    let guides = Guides {
        albedo: guides_raw.iter().map(|g| g.0).collect(),
        normal: guides_raw.iter().map(|g| g.1).collect(),
        depth: guides_raw.iter().map(|g| g.2).collect(),
    };
    let mut best: Option<(u32, f64, f64)> = None;
    for spp in [16u32, 32, 64, 128] {
        let mut t = GpuTracer::new(&g, &scene, cam);
        t.accumulate(&g, spp, 900);
        let noisy = t.image(&g);
        let den = denoise(&noisy, &guides, cam.size, &DenoiseParams::default());
        let (p0, p1) = (psnr_tonemapped(&noisy, &rimg), psnr_tonemapped(&den, &rimg));
        eprintln!("{spp:>4} spp: PSNR noisy {p0:.2} dB -> denoised {p1:.2} dB");
        if best.is_none() && p1 >= 30.0 {
            best = Some((spp, p0, p1));
        }
    }
    let (spp, p0, p1) = best.expect("the denoiser never reached 30 dB");
    eprintln!("denoiser reaches {p1:.2} dB (noisy {p0:.2} dB) at {spp} spp");
}
