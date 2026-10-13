#![allow(clippy::needless_range_loop)]
use kw_fem::*;
use kw_mesh3d::box_tets;

fn steel() -> Material {
    Material {
        e: 210e3,
        nu: 0.3,
        rho: 7.8e-9,
    }
}

#[test]
fn constant_strain_patch_test_is_exact() {
    // a box whose boundary nodes carry a linear displacement field: the interior nodes must reproduce it
    let m = box_tets(2.0, 1.0, 1.5, 3, 2, 2).to_tet10(None);
    let field = |p: [f64; 3]| -> [f64; 3] {
        [
            1e-3 * p[0] + 2e-4 * p[1] - 3e-4 * p[2] + 1e-4,
            -5e-4 * p[0] + 7e-4 * p[1] + 1e-4 * p[2] - 2e-4,
            2e-4 * p[0] + 3e-4 * p[1] + 9e-4 * p[2] + 5e-5,
        ]
    };
    let on_boundary: std::collections::HashSet<u32> =
        m.surface.iter().flat_map(|s| s.v.iter().copied()).collect();
    let mut model = Model::new(m, steel());
    for n in 0..model.mesh.nodes.len() as u32 {
        if on_boundary.contains(&n) {
            let u = field(model.mesh.nodes[n as usize]);
            for a in 0..3 {
                model.fixed.push(Fixed {
                    node: n,
                    axis: a,
                    value: u[a],
                });
            }
        }
    }
    for kind in [SolverKind::Direct, SolverKind::Pcg(Precond::BlockJacobi)] {
        let sol = model.solve(kind).unwrap();
        let mut worst = 0.0f64;
        for (i, p) in model.mesh.nodes.iter().enumerate() {
            let u = field(*p);
            for a in 0..3 {
                worst = worst.max((sol.u[3 * i + a] - u[a]).abs());
            }
        }
        assert!(worst < 1e-9, "{kind:?}: patch test error {worst}");
    }
}

#[test]
fn free_stiffness_has_exactly_the_rigid_body_null_space() {
    let m = box_tets(1.0, 1.0, 1.0, 2, 2, 2).to_tet10(None);
    let model = Model::new(m, steel());
    let (k, _, _) = model.assemble();
    let n = model.mesh.nodes.len();
    let mut y = vec![0.0; 3 * n];
    for mode in 0..6 {
        let mut x = vec![0.0; 3 * n];
        for (i, p) in model.mesh.nodes.iter().enumerate() {
            let v = match mode {
                0 => [1.0, 0.0, 0.0],
                1 => [0.0, 1.0, 0.0],
                2 => [0.0, 0.0, 1.0],
                3 => [0.0, -p[2], p[1]],
                4 => [p[2], 0.0, -p[0]],
                _ => [-p[1], p[0], 0.0],
            };
            x[3 * i..3 * i + 3].copy_from_slice(&v);
        }
        k.spmv(&x, &mut y);
        let ny = y.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        assert!(ny < 1e-6, "mode {mode}: |K x| = {ny}");
    }
    for i in 0..k.n.min(60) {
        for kk in k.row_ptr[i]..k.row_ptr[i + 1] {
            let j = k.col[kk] as usize;
            assert!((k.val[kk] - k.get(j, i)).abs() < 1e-6 * (1.0 + k.val[kk].abs()));
        }
    }
}

#[test]
fn clamped_beam_tip_deflection_is_close_to_beam_theory() {
    // L = 20, h = b = 2 (slender), tip shear load P: delta = P L^3 / (3 E I)
    let (l, h, p) = (20.0, 2.0, 100.0);
    let m = box_tets(l, h, h, 20, 2, 2).to_tet10(None);
    let mut model = Model::new(m, steel());
    model.fix_where([true; 3], &|x| x[0] < 1e-9);
    model.traction.push((1, [0.0, 0.0, -p / (h * h)]));
    let sol = model.solve(SolverKind::Direct).unwrap();
    let (mut sum, mut cnt) = (0.0, 0);
    for (i, x) in model.mesh.nodes.iter().enumerate() {
        if (x[0] - l).abs() < 1e-9 {
            sum += sol.u[3 * i + 2];
            cnt += 1;
        }
    }
    let tip = sum / cnt as f64;
    let i_mom = h * h * h * h / 12.0;
    let euler = p * l * l * l / (3.0 * 210e3 * i_mom);
    assert!(
        -tip > 0.9 * euler && -tip < 1.06 * euler,
        "tip {tip} vs Euler {euler}"
    );
}

#[test]
fn a_raised_cancel_flag_stops_the_iterative_solver() {
    let mesh = box_tets(4.0, 1.0, 1.0, 4, 2, 2);
    let mut model = Model::new(
        mesh.to_tet10(None),
        Material {
            e: 1000.0,
            nu: 0.3,
            rho: 1e-9,
        },
    );
    model.fix_where([true; 3], &|p| p[0] < 1e-9);
    model
        .forces
        .push((model.mesh.nodes.len() as u32 - 1, [0.0, -1.0, 0.0]));
    let cancel = kw_job::CancelFlag::new();
    cancel.cancel();
    let kind = SolverKind::Pcg(Precond::Jacobi);
    assert_eq!(
        model.solve_with(kind, Some(&cancel)).unwrap_err(),
        "cancelled"
    );
    // not raised: it solves
    assert!(model
        .solve_with(kind, Some(&kw_job::CancelFlag::new()))
        .is_ok());
}
