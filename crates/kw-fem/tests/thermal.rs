//! Steady-state conduction and thermal stress against closed forms.
#![allow(clippy::needless_range_loop)]
use kw_fem::tie::{find_ties, TieOptions};
use kw_fem::*;
use kw_mesh3d::{box_tets, Tet10Mesh};

fn bar(origin: f64, l: f64, cells: [usize; 3]) -> Tet10Mesh {
    let mut m = box_tets(l, 2.0, 2.0, cells[0], cells[1], cells[2]).to_tet10(None);
    for p in &mut m.nodes {
        p[0] += origin;
    }
    m
}

#[test]
fn a_slab_with_fixed_end_temperatures_has_a_linear_profile() {
    let mut m = ThermalModel::new(bar(0.0, 10.0, [5, 2, 2]), 50.0);
    m.fix_where(100.0, &|p| p[0] < 1e-9);
    m.fix_where(20.0, &|p| (p[0] - 10.0).abs() < 1e-9);
    let t = m.solve(SolverKind::Direct).unwrap();
    for (i, p) in m.mesh.nodes.iter().enumerate() {
        assert!(
            (t[i] - (100.0 - 8.0 * p[0])).abs() < 1e-8,
            "{} at {}",
            t[i],
            p[0]
        );
    }
    // the conjugate gradient solver agrees
    let t2 = m.solve(SolverKind::Pcg(Precond::Jacobi)).unwrap();
    assert!(t.iter().zip(&t2).all(|(a, b)| (a - b).abs() < 1e-6));
}

#[test]
fn convection_at_the_far_end_follows_the_series_resistance() {
    // T = 100 at x = 0, convection (h, 20 °C) at x = L: q = ΔT / (L/k + 1/h), T_end = 20 + q/h
    let (l, k, h) = (10.0, 50.0, 0.05);
    let mut m = ThermalModel::new(bar(0.0, l, [5, 2, 2]), k);
    m.fix_where(100.0, &|p| p[0] < 1e-9);
    m.convection.push((1, h, 20.0)); // tag 1: x max of the box
    let t = m.solve(SolverKind::Direct).unwrap();
    let q = 80.0 / (l / k + 1.0 / h);
    let end = 20.0 + q / h;
    for (i, p) in m.mesh.nodes.iter().enumerate() {
        if (p[0] - l).abs() < 1e-9 {
            assert!((t[i] - end).abs() < 1e-6 * end, "{} vs {end}", t[i]);
        }
        let want = 100.0 - q / k * p[0];
        assert!(
            (t[i] - want).abs() < 1e-6 * 100.0,
            "{} vs {want} at {}",
            t[i],
            p[0]
        );
    }
}

#[test]
fn an_applied_flux_heats_the_slab_by_q_l_over_k() {
    // flux q into the face x = L, temperature fixed at x = 0: T(L) - T(0) = q L / k
    let (l, k, q) = (10.0, 20.0, 4.0);
    let mut m = ThermalModel::new(bar(0.0, l, [5, 2, 2]), k);
    m.fix_where(30.0, &|p| p[0] < 1e-9);
    m.flux.push((1, q));
    let t = m.solve(SolverKind::Direct).unwrap();
    for (i, p) in m.mesh.nodes.iter().enumerate() {
        assert!((t[i] - (30.0 + q / k * p[0])).abs() < 1e-8);
    }
}

#[test]
fn heat_generation_gives_the_parabolic_profile() {
    // T = 0 at both ends, Q everywhere: T(x) = Q x (L - x) / (2 k)
    let (l, k, q) = (10.0, 25.0, 3.0);
    let mut m = ThermalModel::new(bar(0.0, l, [6, 2, 2]), k);
    m.generation = q;
    m.fix_where(0.0, &|p| p[0] < 1e-9 || (p[0] - l).abs() < 1e-9);
    let t = m.solve(SolverKind::Direct).unwrap();
    for (i, p) in m.mesh.nodes.iter().enumerate() {
        let want = q * p[0] * (l - p[0]) / (2.0 * k);
        assert!(
            (t[i] - want).abs() < 1e-8 + 1e-6 * want,
            "{} vs {want}",
            t[i]
        );
    }
}

#[test]
fn heat_flows_across_a_bonded_interface_like_resistances_in_series() {
    let (k1, k2, l1, l2) = (50.0, 5.0, 6.0, 4.0);
    let merged = Tet10Mesh::concat(&[bar(0.0, l1, [3, 2, 2]), bar(l1, l2, [4, 3, 3])]);
    let (ties, report) = find_ties(
        &merged,
        &TieOptions {
            tolerance: 1e-6,
            skip: vec![],
        },
    );
    assert_eq!(report.pairs.len(), 1);
    let mut m = ThermalModel::new(merged.mesh.clone(), k1);
    m.conductivities = vec![k1, k2];
    m.elem_material = merged.elem_body.clone();
    m.ties = ties;
    m.fix_where(100.0, &|p| p[0] < 1e-9);
    m.fix_where(20.0, &|p| (p[0] - 10.0).abs() < 1e-9);
    let t = m.solve(SolverKind::Direct).unwrap();
    let (r1, r2) = (l1 / k1, l2 / k2);
    let at_interface = 100.0 - 80.0 * r1 / (r1 + r2);
    let mut seen = 0;
    for (i, p) in m.mesh.nodes.iter().enumerate() {
        if (p[0] - l1).abs() < 1e-9 {
            seen += 1;
            assert!(
                (t[i] - at_interface).abs() < 0.01 * at_interface,
                "{} vs {at_interface}",
                t[i]
            );
        }
    }
    assert!(seen > 20);
}

#[test]
fn an_underdetermined_thermal_study_is_refused() {
    let m = ThermalModel::new(bar(0.0, 10.0, [2, 2, 2]), 50.0);
    assert!(m
        .solve(SolverKind::Direct)
        .unwrap_err()
        .contains("determinate"));
}

fn steel() -> Material {
    Material {
        e: 210e3,
        nu: 0.3,
        rho: 7.85e-9,
    }
}

/// A bar heated uniformly to `t` (reference 20 °C), supported on the symmetry planes, with the far end
/// either held in x or free.
fn heated_bar(held: bool, t: f64) -> (Model, Vec<f64>) {
    let mesh = bar(0.0, 10.0, [5, 2, 2]);
    let n = mesh.nodes.len();
    let mut model = Model::new(mesh, steel());
    model.fix_where([true, false, false], &|p| p[0] < 1e-9);
    model.fix_where([false, true, false], &|p| p[1] < 1e-9);
    model.fix_where([false, false, true], &|p| p[2] < 1e-9);
    if held {
        model.fix_where([true, false, false], &|p| (p[0] - 10.0).abs() < 1e-9);
    }
    model.thermal = Some(ThermalLoad {
        temperature: vec![t; n],
        expansion: vec![12e-6],
        t_ref: 20.0,
    });
    let sol = model.solve(SolverKind::Direct).unwrap();
    (model, sol.u)
}

#[test]
fn a_bar_held_at_both_ends_is_stressed_by_e_alpha_dt() {
    let (model, u) = heated_bar(true, 120.0);
    let nodal = kw_fem::stress::nodal_stress(&model, &u);
    let want = -210e3 * 12e-6 * 100.0; // -252 MPa
    for s in &nodal {
        assert!(
            (s[0] - want).abs() < 0.01 * want.abs(),
            "{} vs {want}",
            s[0]
        );
        assert!(s[1].abs() < 0.01 * want.abs() && s[2].abs() < 0.01 * want.abs());
    }
}

#[test]
fn a_bar_free_to_expand_grows_by_alpha_dt_l_and_has_no_stress() {
    let (model, u) = heated_bar(false, 120.0);
    let nodal = kw_fem::stress::nodal_stress(&model, &u);
    for s in &nodal {
        for c in s {
            assert!(c.abs() < 1e-3, "stress {c}");
        }
    }
    let grow = 12e-6 * 100.0 * 10.0;
    for (i, p) in model.mesh.nodes.iter().enumerate() {
        if (p[0] - 10.0).abs() < 1e-9 {
            assert!((u[3 * i] - grow).abs() < 1e-9, "{} vs {grow}", u[3 * i]);
        }
    }
}
