#![allow(clippy::needless_range_loop)]
//! Timing of the solvers on box-shaped cantilevers (structured tet meshes). Sizes via environment:
//! KW_FEM_DIRECT_N (cells along the short side for the direct run, default 16 ≈ 100k DOF) and
//! KW_FEM_PCG_N (default 22 ≈ 1.07M DOF, aspect 4:1:1).

use kw_fem::*;
use kw_mesh3d::box_tets;
use std::sync::Mutex;
use std::time::Instant;

/// The timings are only meaningful when the two runs do not compete for the cores and the memory bus.
static ONE_AT_A_TIME: Mutex<()> = Mutex::new(());

fn setup(n: usize, aspect: usize) -> (Model, usize) {
    let (nx, ny, nz) = (aspect * n, n, n);
    let h = 1.0;
    let m = box_tets(aspect as f64 * h, h, h, nx, ny, nz).to_tet10(None);
    let dofs = 3 * m.nodes.len();
    let mut model = Model::new(
        m,
        Material {
            e: 210e3,
            nu: 0.3,
            rho: 7.8e-9,
        },
    );
    model.fix_where([true; 3], &|x| x[0] < 1e-9);
    model.traction.push((1, [0.0, 0.0, -1.0]));
    (model, dofs)
}

fn env(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

#[test]
fn direct_100k_dofs() {
    let _guard = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let n = env("KW_FEM_DIRECT_N", 16);
    let t = Instant::now();
    let (model, dofs) = setup(n, 1);
    let t_mesh = t.elapsed().as_secs_f64();
    let sol = model.solve(SolverKind::Direct).unwrap();
    eprintln!(
        "direct: {dofs} DOF, mesh {:.2} s, assembly {:.2} s, BCs {:.2} s, factor+solve {:.2} s",
        t_mesh, sol.timings.assembly, sol.timings.bcs, sol.timings.solve
    );
    // the criterion: a static solve of 100k DOF takes at most 10 s (assembly, boundary conditions,
    // factorisation and solve all counted; meshing is not)
    if n == 16 {
        assert!(dofs >= 100_000, "{dofs}");
        assert!(sol.timings.assembly + sol.timings.bcs + sol.timings.solve <= 10.0);
    }
}

#[test]
fn pcg_one_million_dofs() {
    let _guard = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let n = env("KW_FEM_PCG_N", 22);
    let (model, dofs) = setup(n, 4);
    for pre in [Precond::Jacobi, Precond::BlockJacobi] {
        let t = Instant::now();
        let sol = model.solve(SolverKind::Pcg(pre)).unwrap();
        let rep = sol.pcg.unwrap();
        eprintln!(
            "pcg {pre:?}: {dofs} DOF, assembly {:.1} s, solve {:.1} s, {} iterations, residual {:.2e}, total {:.1} s",
            sol.timings.assembly, sol.timings.solve, rep.iterations, rep.residual, t.elapsed().as_secs_f64()
        );
        // the criterion: residual 1e-8 on 1M DOF within 120 s (assembly counted)
        if n == 22 {
            assert!(dofs >= 1_000_000, "{dofs}");
            assert!(rep.converged && rep.residual <= 1e-8, "{rep:?}");
            assert!(sol.timings.assembly + sol.timings.bcs + sol.timings.solve <= 120.0);
        }
        if std::env::var("KW_FEM_BOTH").is_err() {
            break;
        }
    }
}
