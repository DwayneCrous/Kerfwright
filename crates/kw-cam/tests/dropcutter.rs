//! Drop-cutter against closed forms and against the brute-force sampler.

use kw_cam::bvh::{Tri, TriMesh};
use kw_cam::dropcutter::{cylinder_hits_mesh, drop_cutter, drop_cutter_brute};
use kw_cam::Tool;

fn rnd(seed: &mut u64) -> f64 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 7;
    *seed ^= *seed << 17;
    (*seed >> 11) as f64 / (1u64 << 53) as f64
}

fn plane(alpha: f64) -> TriMesh {
    // z = x tan(alpha) over a wide square
    let t = alpha.tan();
    let c = |x: f64, y: f64| [x, y, x * t];
    TriMesh::new(vec![
        [c(-50.0, -50.0), c(50.0, -50.0), c(50.0, 50.0)],
        [c(-50.0, -50.0), c(50.0, 50.0), c(-50.0, 50.0)],
    ])
}

#[test]
fn closed_forms_on_a_sloped_plane() {
    for deg in [0.0f64, 10.0, 30.0, 60.0] {
        let a = deg.to_radians();
        let m = plane(a);
        let r = 5.0;
        // flat: tip = R tan(alpha)
        let flat = drop_cutter(&m, &Tool::flat(1, 2.0 * r, 20.0), 0.0, 0.0).unwrap();
        assert!((flat - r * a.tan()).abs() < 1e-9, "flat {deg}: {flat}");
        // ball: centre on the offset plane, tip = R/cos(alpha) − R
        let ball = drop_cutter(&m, &Tool::ball(1, 2.0 * r, 20.0), 0.0, 0.0).unwrap();
        assert!(
            (ball - (r / a.cos() - r)).abs() < 1e-9,
            "ball {deg}: {ball}"
        );
        // bull (r = 2): corner circle centre sits at distance (corner radius) from the plane, at rho = 3
        let cr = 2.0;
        let bull = drop_cutter(&m, &Tool::bull(1, 2.0 * r, cr, 20.0), 0.0, 0.0).unwrap();
        // the corner circle centre sits rho towards +x at height tip + cr and at distance cr from the plane:
        // tip + cr = rho tan(alpha) + cr / cos(alpha)
        let rho = r - cr;
        let expect = rho * a.tan() + cr / a.cos() - cr;
        assert!(
            (bull - expect).abs() < 1e-9,
            "bull {deg}: {bull} vs {expect}"
        );
    }
}

#[test]
fn ball_on_a_sphere_and_flat_on_a_cylinder_edge() {
    // an inscribed pyramid-ish fan approximating a sphere is awkward; use a single apex triangle fan instead:
    // ball of radius 3 over a vertical-edged box: tip rests on the top face
    let b = TriMesh::from_box([-10.0, -10.0, 0.0], [10.0, 10.0, 4.0]);
    let h = drop_cutter(&b, &Tool::ball(1, 6.0, 20.0), 0.0, 0.0).unwrap();
    assert!((h - 4.0).abs() < 1e-12);
    // beside the box by less than the radius: the ball touches the top edge, height = 4 − 3 + sqrt(9 − d²)
    let d = 2.0;
    let h = drop_cutter(&b, &Tool::ball(1, 6.0, 20.0), 10.0 + d, 0.0).unwrap();
    assert!(
        (h - (4.0 - 3.0 + (9.0f64 - d * d).sqrt())).abs() < 1e-9,
        "{h}"
    );
    // flat tool beside the edge: the top face is within reach as soon as d < R, then tip = 4
    let h = drop_cutter(&b, &Tool::flat(1, 6.0, 20.0), 12.9, 0.0).unwrap();
    assert!((h - 4.0).abs() < 1e-12);
    assert!(drop_cutter(&b, &Tool::flat(1, 6.0, 20.0), 13.1, 0.0).is_none());
}

#[test]
fn agrees_with_the_brute_force_sampler_on_random_triangles() {
    let mut seed = 0xDEADBEEF12345u64;
    let tools = [
        Tool::flat(1, 6.0, 20.0),
        Tool::ball(2, 6.0, 20.0),
        Tool::bull(3, 6.0, 1.5, 20.0),
    ];
    let mut worst = 0.0f64;
    let mut cases = 0;
    for _ in 0..300 {
        let tris: Vec<Tri> = (0..8)
            .map(|_| {
                let o = [
                    rnd(&mut seed) * 12.0 - 6.0,
                    rnd(&mut seed) * 12.0 - 6.0,
                    rnd(&mut seed) * 6.0,
                ];
                let mut v = || {
                    [
                        o[0] + rnd(&mut seed) * 8.0 - 4.0,
                        o[1] + rnd(&mut seed) * 8.0 - 4.0,
                        o[2] + rnd(&mut seed) * 4.0 - 2.0,
                    ]
                };
                [v(), v(), v()]
            })
            .collect();
        let mesh = TriMesh::new(tris.clone());
        for tool in &tools {
            let (x, y) = (rnd(&mut seed) * 6.0 - 3.0, rnd(&mut seed) * 6.0 - 3.0);
            let fast = drop_cutter(&mesh, tool, x, y);
            let slow = drop_cutter_brute(&tris, tool, x, y, 90);
            match (fast, slow) {
                (None, None) => {}
                (Some(f), Some(s)) => {
                    // the sampler never exceeds the exact maximum, and falls short by sampling error only
                    assert!(f >= s - 1e-9, "exact {f} below sample {s}");
                    worst = worst.max(f - s);
                    cases += 1;
                }
                (f, s) => {
                    // a triangle grazing the disc boundary can be seen by one and missed by the other only when
                    // it just touches; the sampler may then miss it
                    assert!(f.is_some() && s.is_none(), "{f:?} {s:?}");
                }
            }
        }
    }
    eprintln!("drop-cutter vs brute force: {cases} cases, exact − sampled up to {worst:.4} mm");
    assert!(worst < 0.15, "{worst}");
}

#[test]
fn holder_collision_is_exact_against_a_box() {
    let b = TriMesh::from_box([-5.0, -5.0, 0.0], [5.0, 5.0, 10.0]);
    // a cylinder beside the box, radius 3 with its axis 4.5 from the box face: clear. At 2.9: hit.
    assert!(!cylinder_hits_mesh(&b, [8.5, 0.0], 3.0, 2.0, 30.0, 1e-6));
    assert!(cylinder_hits_mesh(&b, [7.9, 0.0], 3.0, 2.0, 30.0, 1e-6));
    // above the box top: clear whatever the radius
    assert!(!cylinder_hits_mesh(&b, [0.0, 0.0], 3.0, 10.5, 30.0, 1e-6));
    assert!(cylinder_hits_mesh(&b, [0.0, 0.0], 3.0, 9.5, 30.0, 1e-6));
}
