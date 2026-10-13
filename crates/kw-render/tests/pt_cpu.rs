//! CPU-side checks of the path tracer: LBVH against brute force, and the reference tracer against closed forms.

use kw_render::pt::cpu_ref::{brdf, Mat, Reference};
use kw_render::pt::lbvh::{tri_hit, Lbvh};
use kw_render::pt::{normalize, scenes, Material, PtCamera, PtScene};

fn rnd(s: &mut u64) -> f32 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    (*s >> 40) as f32 / (1u64 << 24) as f32
}

#[test]
fn lbvh_agrees_with_brute_force() {
    let (scene, _) = scenes::cornell();
    let bvh = Lbvh::build(&scene);
    assert_eq!(bvh.nodes.len(), scene.tris.len() - 1);
    let mut seed = 99u64;
    let (mut hits, mut total) = (0, 0);
    for _ in 0..20_000 {
        let o = [
            rnd(&mut seed) * 1.8 - 0.9,
            rnd(&mut seed) * 1.8 - 0.9,
            rnd(&mut seed) * 1.8 + 0.1,
        ];
        let d = normalize([
            rnd(&mut seed) - 0.5,
            rnd(&mut seed) - 0.5,
            rnd(&mut seed) - 0.5,
        ]);
        let fast = bvh.intersect(&scene, o, d, f32::MAX);
        let mut slow: Option<(f32, usize)> = None;
        for (i, t) in scene.tris.iter().enumerate() {
            if let Some(tt) = tri_hit(t, o, d, slow.map_or(f32::MAX, |s| s.0)) {
                slow = Some((tt, i));
            }
        }
        total += 1;
        match (fast, slow) {
            (None, None) => {}
            (Some(a), Some(b)) => {
                hits += 1;
                assert!((a.0 - b.0).abs() < 1e-5, "{a:?} vs {b:?}");
            }
            (a, b) => panic!("{a:?} vs {b:?}"),
        }
    }
    eprintln!(
        "LBVH: {hits}/{total} rays hit, all identical to brute force ({} nodes)",
        bvh.nodes.len()
    );
    assert!(hits > 10_000);
}

fn plane_scene(env: f32, mat: Material) -> (PtScene, PtCamera) {
    let mut s = PtScene::new([env; 3]);
    let m = s.add_material(mat);
    // a huge upward-facing plane
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
    (s, cam)
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

#[test]
fn white_furnace_lambert_albedo_one_returns_the_environment() {
    // a convex albedo-1 Lambert ball in uniform radiance 1 reflects exactly radiance 1
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
    let r = Reference::new(&s);
    let img = r.render(&cam, 2000, 1);
    let v = mean(&img);
    eprintln!("Lambert furnace (CPU reference): {v:.5?}");
    for k in 0..3 {
        assert!((v[k] - 1.0).abs() < 3e-3, "{v:?}");
    }
}

/// Directional albedo ∫ f cos dωi by quadrature (f64, midpoint rule in θ and φ).
fn albedo_quadrature(m: &Mat, n: [f64; 3], wo: [f64; 3]) -> [f64; 3] {
    let (nt, np) = (600, 600);
    let mut acc = [0.0; 3];
    for i in 0..nt {
        let th = std::f64::consts::FRAC_PI_2 * (i as f64 + 0.5) / nt as f64;
        for j in 0..np {
            let ph = std::f64::consts::TAU * (j as f64 + 0.5) / np as f64;
            let wi = [th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()];
            let f = brdf(m, n, wo, wi);
            let w = th.sin()
                * th.cos()
                * (std::f64::consts::FRAC_PI_2 / nt as f64)
                * (std::f64::consts::TAU / np as f64);
            for k in 0..3 {
                acc[k] += f[k] * w;
            }
        }
    }
    acc
}

#[test]
fn ggx_furnace_matches_the_directional_albedo_and_never_exceeds_one() {
    for (rough, metallic) in [(0.2f32, 1.0f32), (0.5, 1.0), (1.0, 1.0), (0.4, 0.0)] {
        let mat = Material::glossy([1.0, 1.0, 1.0], rough, metallic);
        let (s, cam) = plane_scene(1.0, mat);
        let r = Reference::new(&s);
        let v = mean(&r.render(&cam, 6000, 2));
        let q = albedo_quadrature(&Mat::from(&mat), [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]);
        eprintln!(
            "GGX rough {rough} metallic {metallic}: traced {:.4}, quadrature {:.4}",
            v[0], q[0]
        );
        assert!(
            (v[0] - q[0]).abs() < 0.01 * q[0].max(0.2),
            "traced {} quadrature {}",
            v[0],
            q[0]
        );
        assert!(q[0] <= 1.0 + 1e-3, "energy gain {}", q[0]);
    }
}

#[test]
fn disc_light_irradiance_matches_the_closed_form() {
    // Lambert plane (albedo ρ) lit by a disc of radiance L, radius R, at height h facing down:
    // E = π L R² / (h² + R²) at the point below its centre, outgoing radiance ρ E / π.
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
    // an oblique camera looking at the origin from the side, below the disc
    let cam = PtCamera {
        origin: [3.5, 0.0, 0.9],
        forward: normalize([-3.5, 0.0, -0.9]),
        up: [0.0, 0.0, 1.0],
        fov: 0.004,
        size: (6, 6),
    };
    let r = Reference::new(&s);
    let v = mean(&r.render(&cam, 3000, 3));
    let want = (rho * l * big_r * big_r / (h * h + big_r * big_r)) as f64;
    eprintln!("disc light: traced {:.5}, closed form {want:.5}", v[0]);
    assert!((v[0] - want).abs() / want < 0.01, "{} vs {want}", v[0]);
}

#[test]
#[ignore]
fn denoiser_parameter_sweep() {
    use kw_render::pt::denoise::{denoise, DenoiseParams, Guides};
    use kw_render::pt::metrics::psnr_tonemapped;
    let (scene, cam) = scenes::cornell();
    let r = Reference::new(&scene);
    let reference = r.render(&cam, 4096, 11);
    let g = r.guides(&cam);
    let guides = Guides {
        albedo: g.iter().map(|x| x.0).collect(),
        normal: g.iter().map(|x| x.1).collect(),
        depth: g.iter().map(|x| x.2).collect(),
    };
    for spp in [32usize, 64] {
        let noisy = r.render(&cam, spp, 900);
        eprintln!(
            "{spp} spp noisy {:.2} dB",
            psnr_tonemapped(&noisy, &reference)
        );
        for it in [3usize, 4, 5] {
            for sl in [1.0, 2.0, 4.0, 8.0] {
                for sn in [16.0, 64.0] {
                    let p = DenoiseParams {
                        iterations: it,
                        sigma_l: sl,
                        sigma_n: sn,
                        sigma_z: 1.0,
                    };
                    let d = denoise(&noisy, &guides, cam.size, &p);
                    eprintln!(
                        "   it {it} sigma_l {sl} sigma_n {sn}: {:.2} dB",
                        psnr_tonemapped(&d, &reference)
                    );
                }
            }
        }
    }
}
