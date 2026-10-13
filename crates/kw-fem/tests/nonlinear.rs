//! Large displacements and plasticity against closed forms and finite differences.
#![allow(clippy::needless_range_loop)]
use kw_fem::nonlinear::*;
use kw_fem::*;
use kw_mesh3d::box_tets;

fn one_element() -> [[f64; 3]; 10] {
    // a distorted quadratic tet: the corners of a unit tet moved about, mid-nodes near the midpoints
    let m = box_tets(1.0, 1.0, 1.0, 1, 1, 1).to_tet10(None);
    let e = m.elems[0];
    let mut x = [[0.0; 3]; 10];
    for i in 0..10 {
        x[i] = m.nodes[e[i] as usize];
        for a in 0..3 {
            x[i][a] += 0.03 * (((i * 7 + a * 3) % 5) as f64 - 2.0);
        }
    }
    x
}

#[test]
fn the_tangent_is_the_derivative_of_the_internal_force() {
    let x = one_element();
    // yields at a strain of 1e-3; the displacement below is several times that
    let mat = NlMat::new(1000.0, 0.3, 1.0, 100.0);
    let mut ue = [0.0; 30];
    for i in 0..30 {
        ue[i] = 0.01 * (((i * 13 + 5) % 11) as f64 - 5.0) / 5.0;
    }
    for geometric in [true, false] {
        let states = vec![GaussState::default(); 27];
        let base = element_response(&x, &ue, &mat, geometric, &states, true);
        assert!(
            base.states.iter().any(|s| s.alpha > 0.0),
            "the test point must be plastic"
        );
        let scale = base.ke.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let h = 1e-6;
        let mut worst = 0.0f64;
        for j in 0..30 {
            let (mut up, mut um) = (ue, ue);
            up[j] += h;
            um[j] -= h;
            let fp = element_response(&x, &up, &mat, geometric, &states, false).fe;
            let fm = element_response(&x, &um, &mat, geometric, &states, false).fe;
            for i in 0..30 {
                let fd = (fp[i] - fm[i]) / (2.0 * h);
                worst = worst.max((fd - base.ke[i * 30 + j]).abs() / scale);
            }
        }
        assert!(worst < 1e-5, "geometric={geometric}: tangent error {worst}");
    }
}

#[test]
fn a_large_rigid_rotation_costs_no_force() {
    let x = one_element();
    let mat = NlMat::new(210e3, 0.3, 250.0, 2100.0);
    let (s, c) = 90f64.to_radians().sin_cos();
    // rotation about z by 90 degrees: u = (R - I) X
    let mut ue = [0.0; 30];
    for i in 0..10 {
        ue[3 * i] = (c - 1.0) * x[i][0] - s * x[i][1];
        ue[3 * i + 1] = s * x[i][0] + (c - 1.0) * x[i][1];
    }
    let states = vec![GaussState::default(); 27];
    let out = element_response(&x, &ue, &mat, true, &states, false);
    let f = out.fe.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(f < 1e-8 * 210e3, "force {f}");
    assert!(out.states.iter().all(|s| s.alpha == 0.0));
    assert!(out.vm.iter().all(|v| *v < 1e-6));
    // the linear theory, by contrast, would find a huge stress here
    let lin = element_response(&x, &ue, &mat, false, &states, false);
    assert!(lin.fe.iter().fold(0.0f64, |m, v| m.max(v.abs())) > 1e3);
}

#[test]
fn a_bar_stretched_past_yield_follows_the_bilinear_stress_strain_curve() {
    let (e, sy, et) = (1000.0, 1.0, 100.0);
    let mesh = box_tets(10.0, 2.0, 2.0, 5, 2, 2).to_tet10(None);
    let mut model = Model::new(
        mesh,
        Material {
            e,
            nu: 0.0,
            rho: 1e-9,
        },
    );
    model.fix_where([true, false, false], &|p| p[0] < 1e-9);
    model.fix_where([false, true, false], &|p| p[1] < 1e-9);
    model.fix_where([false, false, true], &|p| p[2] < 1e-9);
    let strain = 0.01;
    for (i, p) in model.mesh.nodes.clone().iter().enumerate() {
        if (p[0] - 10.0).abs() < 1e-9 {
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 0,
                value: strain * 10.0,
            });
        }
    }
    let nl = Nonlinear {
        yield_stress: sy,
        tangent_modulus: et,
        geometric: false,
        steps: 5,
        ..Default::default()
    };
    let r = solve_nonlinear(&model, &nl, None, None, None).unwrap();
    let want = sy + et * (strain - sy / e);
    let plastic = strain - want / e;
    for (i, p) in model.mesh.nodes.iter().enumerate() {
        assert!((r.u[3 * i] - strain * p[0]).abs() < 1e-9);
        assert!(
            (r.von_mises[i] - want).abs() < 0.005 * want,
            "{} vs {want}",
            r.von_mises[i]
        );
        assert!((r.plastic_strain[i] - plastic).abs() < 0.01 * plastic);
    }
    assert!((r.load_factors.last().unwrap() - 1.0).abs() < 1e-12);
    // below yield nothing is plastic
    let mut elastic = model.clone();
    for c in &mut elastic.fixed {
        if c.value != 0.0 {
            c.value = 0.0005 * 10.0;
        }
    }
    let r = solve_nonlinear(&elastic, &nl, None, None, None).unwrap();
    assert!(r.plastic_strain.iter().all(|a| *a == 0.0));
    assert!((r.von_mises[0] - 0.5).abs() < 0.005);
}

fn cantilever(p: f64, geometric: bool, steps: usize) -> (f64, usize) {
    // L = 20, 2 x 2 section: EI = 2.8e5; P = alpha EI / L^2
    let (l, h) = (20.0, 2.0);
    let mesh = box_tets(l, h, h, 40, 2, 2).to_tet10(None);
    let mut model = Model::new(
        mesh,
        Material {
            e: 210e3,
            nu: 0.3,
            rho: 7.8e-9,
        },
    );
    model.fix_where([true; 3], &|x| x[0] < 1e-9);
    model.traction.push((1, [0.0, 0.0, -p / (h * h)]));
    let nl = Nonlinear {
        geometric,
        steps,
        ..Default::default()
    };
    let r = solve_nonlinear(&model, &nl, None, None, None).unwrap();
    let (mut sum, mut cnt) = (0.0, 0);
    for (i, x) in model.mesh.nodes.iter().enumerate() {
        if (x[0] - l).abs() < 1e-9 {
            sum += r.u[3 * i + 2];
            cnt += 1;
        }
    }
    (-sum / cnt as f64, r.iterations.iter().sum())
}

#[test]
fn a_cantilever_under_a_large_tip_load_follows_the_elastica() {
    // P L² / EI = 1: the exact tip deflection is 0.3017 L (the linear theory says 0.3333 L)
    let p = 1.0 * 2.8e5 / 400.0;
    let (v, _) = cantilever(p, true, 5);
    let l = 20.0;
    assert!(
        (v / l - 0.3017).abs() < 0.012,
        "tip deflection {} L, the elastica gives 0.3017 L",
        v / l
    );
    // and it is clearly not the linear answer
    let (lin, _) = cantilever(p, false, 1);
    assert!((lin / l - 0.3333).abs() < 0.015, "{}", lin / l);
    assert!(v < 0.93 * lin);
}

#[test]
fn small_loads_give_the_linear_answer() {
    let (nonlin, _) = cantilever(7.0, true, 2);
    let (lin, _) = cantilever(7.0, false, 1);
    assert!((nonlin - lin).abs() < 2e-3 * lin, "{nonlin} vs {lin}");
}

#[test]
fn a_cancelled_run_stops() {
    let mesh = box_tets(2.0, 1.0, 1.0, 2, 1, 1).to_tet10(None);
    let mut model = Model::new(
        mesh,
        Material {
            e: 1000.0,
            nu: 0.3,
            rho: 1e-9,
        },
    );
    model.fix_where([true; 3], &|x| x[0] < 1e-9);
    model.traction.push((1, [0.0, 0.0, -1.0]));
    let cancel = kw_job::CancelFlag::new();
    cancel.cancel();
    let r = solve_nonlinear(&model, &Nonlinear::default(), None, Some(&cancel), None);
    assert_eq!(r.unwrap_err(), "cancelled");
}
