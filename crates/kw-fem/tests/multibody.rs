//! Several bodies in one model: per-body materials, bonded contact across non-matching meshes.
#![allow(clippy::needless_range_loop)]
use kw_fem::tie::{find_ties, TieOptions};
use kw_fem::*;
use kw_mesh3d::{box_tets, MergedMesh, Tet10Mesh};

fn mat(e: f64, nu: f64, rho: f64) -> Material {
    Material { e, nu, rho }
}

/// One block `[x0, x0+lx] × [0, ly] × [z0, z0+lz]` meshed with `cells` and made of `m`.
struct Block {
    origin: [f64; 3],
    size: [f64; 3],
    cells: [usize; 3],
    material: Material,
}

fn build(blocks: &[Block], tol: f64) -> (Model, MergedMesh) {
    let meshes: Vec<Tet10Mesh> = blocks
        .iter()
        .map(|b| {
            let mut m = box_tets(
                b.size[0], b.size[1], b.size[2], b.cells[0], b.cells[1], b.cells[2],
            )
            .to_tet10(None);
            for p in &mut m.nodes {
                for a in 0..3 {
                    p[a] += b.origin[a];
                }
            }
            m
        })
        .collect();
    let merged = Tet10Mesh::concat(&meshes);
    let mut model = Model::new(merged.mesh.clone(), blocks[0].material);
    model.materials = blocks.iter().map(|b| b.material).collect();
    model.elem_material = merged.elem_body.clone();
    let (ties, _) = find_ties(
        &merged,
        &TieOptions {
            tolerance: tol,
            skip: vec![],
        },
    );
    model.ties = ties;
    (model, merged)
}

fn symmetry_planes(model: &mut Model) {
    model.fix_where([true, false, false], &|p| p[0] < 1e-9);
    model.fix_where([false, true, false], &|p| p[1] < 1e-9);
    model.fix_where([false, false, true], &|p| p[2] < 1e-9);
}

fn prescribe_end(model: &mut Model, x: f64, delta: f64) {
    for (i, p) in model.mesh.nodes.clone().iter().enumerate() {
        if (p[0] - x).abs() < 1e-9 {
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 0,
                value: delta,
            });
        }
    }
}

#[test]
fn two_bars_with_different_meshes_bonded_end_to_end_stretch_like_one() {
    // 6 mm and 4 mm long, 2 x 2 section, meshed with unrelated grids; the interface is non-conforming
    let m = mat(210e3, 0.3, 7.8e-9);
    let blocks = [
        Block {
            origin: [0.0, 0.0, 0.0],
            size: [6.0, 2.0, 2.0],
            cells: [3, 2, 2],
            material: m,
        },
        Block {
            origin: [6.0, 0.0, 0.0],
            size: [4.0, 2.0, 2.0],
            cells: [4, 3, 3],
            material: m,
        },
    ];
    let (mut model, merged) = build(&blocks, 1e-6);
    assert!(!model.ties.is_empty(), "the interface is found");
    assert_eq!(merged.node_start.len(), 3);
    symmetry_planes(&mut model);
    let delta = 0.01;
    prescribe_end(&mut model, 10.0, delta);
    let sol = model.solve(SolverKind::Direct).unwrap();
    // uniform strain: u_x = δ x / L everywhere (both sides of the interface), u_y = -ν ε y
    let eps = delta / 10.0;
    let mut worst = 0.0f64;
    for (i, p) in model.mesh.nodes.iter().enumerate() {
        worst = worst.max((sol.u[3 * i] - eps * p[0]).abs() / delta);
        worst = worst.max((sol.u[3 * i + 1] + 0.3 * eps * p[1]).abs() / delta);
    }
    // node-to-surface ties transfer the interface force at points, not consistently (a mortar scheme would),
    // so a patch test across unrelated meshes is met to about a percent, not exactly
    assert!(worst < 0.015, "patch test across the interface: {worst}");
    // the ties hold: slave and master displacements agree to the penalty's accuracy
    for t in &model.ties {
        for a in 0..3 {
            let want: f64 = t
                .master
                .iter()
                .zip(&t.weights)
                .map(|(m, w)| w * sol.u[3 * *m as usize + a])
                .sum();
            assert!((sol.u[3 * t.slave as usize + a] - want).abs() < 2e-4 * delta);
        }
    }
}

#[test]
fn bonded_bars_of_different_materials_share_the_stretch_like_springs_in_series() {
    // ν = 0 so that each bar stays uniaxial; stiffnesses in series: u at the interface follows from the
    // compliances L/E
    let (e1, e2) = (210e3, 70e3);
    let blocks = [
        Block {
            origin: [0.0; 3],
            size: [6.0, 2.0, 2.0],
            cells: [3, 2, 2],
            material: mat(e1, 0.0, 7.8e-9),
        },
        Block {
            origin: [6.0, 0.0, 0.0],
            size: [4.0, 2.0, 2.0],
            cells: [4, 3, 3],
            material: mat(e2, 0.0, 2.7e-9),
        },
    ];
    let (mut model, _) = build(&blocks, 1e-6);
    symmetry_planes(&mut model);
    let delta = 0.01;
    prescribe_end(&mut model, 10.0, delta);
    let sol = model.solve(SolverKind::Direct).unwrap();
    let (c1, c2) = (6.0 / e1, 4.0 / e2);
    let at_interface = delta * c1 / (c1 + c2);
    let mut seen = 0;
    for (i, p) in model.mesh.nodes.iter().enumerate() {
        if (p[0] - 6.0).abs() < 1e-9 {
            seen += 1;
            assert!(
                (sol.u[3 * i] - at_interface).abs() < 0.02 * at_interface,
                "{} vs {at_interface}",
                sol.u[3 * i]
            );
        }
    }
    assert!(seen > 20);
    // the stress is the same in both bars: σ = δ / (c1 + c2)
    let nodal = kw_fem::stress::nodal_stress(&model, &sol.u);
    let want = delta / (c1 + c2);
    for (i, p) in model.mesh.nodes.iter().enumerate() {
        if (p[0] - 3.0).abs() < 1e-9 || (p[0] - 8.0).abs() < 1e-9 {
            assert!(
                (nodal[i][0] - want).abs() < 0.04 * want,
                "{} vs {want}",
                nodal[i][0]
            );
        }
    }
}

#[test]
fn gravity_uses_the_density_of_each_body() {
    let blocks = [
        Block {
            origin: [0.0; 3],
            size: [2.0, 2.0, 2.0],
            cells: [2, 2, 2],
            material: mat(210e3, 0.3, 7.85e-9),
        },
        Block {
            origin: [5.0, 0.0, 0.0],
            size: [3.0, 2.0, 2.0],
            cells: [3, 2, 2],
            material: mat(70e3, 0.33, 2.7e-9),
        },
    ];
    let (mut model, _) = build(&blocks, 1e-6);
    let g = 9810.0;
    model.gravity = Some([0.0, 0.0, -g]);
    let f = model.load_vector();
    let total: f64 = (0..model.mesh.nodes.len()).map(|i| f[3 * i + 2]).sum();
    let mass = 7.85e-9 * 8.0 + 2.7e-9 * 12.0;
    assert!(
        (total + g * mass).abs() < 1e-9 * g * mass,
        "{total} vs {}",
        -g * mass
    );
}

#[test]
fn contacts_are_found_only_between_bodies_that_touch_and_are_not_skipped() {
    let m = mat(1000.0, 0.3, 1e-9);
    let touching = [
        Block {
            origin: [0.0; 3],
            size: [2.0, 2.0, 2.0],
            cells: [2, 2, 2],
            material: m,
        },
        Block {
            origin: [2.0, 0.0, 0.0],
            size: [2.0, 2.0, 2.0],
            cells: [4, 4, 4],
            material: m,
        },
    ];
    let meshes: Vec<Tet10Mesh> = touching
        .iter()
        .map(|b| {
            let mut t = box_tets(
                b.size[0], b.size[1], b.size[2], b.cells[0], b.cells[1], b.cells[2],
            )
            .to_tet10(None);
            for p in &mut t.nodes {
                p[0] += b.origin[0];
            }
            t
        })
        .collect();
    let merged = Tet10Mesh::concat(&meshes);
    let (ties, report) = find_ties(&merged, &TieOptions::default());
    assert!(!ties.is_empty());
    assert_eq!(report.pairs.len(), 1);
    // the finer side (body 1) carries the nodes
    assert_eq!(report.pairs[0].slave, 1);
    assert!(report.pairs[0].max_gap < 1e-9);
    assert!(ties.iter().all(|t| merged.body_of_node(t.slave) == 1));
    assert!(ties
        .iter()
        .all(|t| t.master.iter().all(|n| merged.body_of_node(*n) == 0)));
    // weights of a tie sum to one
    for t in &ties {
        assert!((t.weights.iter().sum::<f64>() - 1.0).abs() < 1e-12);
    }
    // the pair is left unbonded on request
    let (none, report) = find_ties(
        &merged,
        &TieOptions {
            tolerance: 1e-3,
            skip: vec![(1, 0)],
        },
    );
    assert!(none.is_empty() && report.pairs.is_empty());
    // a gap larger than the tolerance is no contact
    let mut apart = meshes.clone();
    for p in &mut apart[1].nodes {
        p[0] += 0.5;
    }
    let (none, _) = find_ties(&Tet10Mesh::concat(&apart), &TieOptions::default());
    assert!(none.is_empty());
    // ...unless the tolerance reaches over it
    let (some, report) = find_ties(
        &Tet10Mesh::concat(&apart),
        &TieOptions {
            tolerance: 0.6,
            skip: vec![],
        },
    );
    assert!(!some.is_empty());
    assert!((report.pairs[0].max_gap - 0.5).abs() < 1e-9);
}

#[test]
fn a_bonded_two_material_cantilever_matches_the_transformed_section() {
    // two layers stacked in z, steel below aluminium, clamped at x = 0, loaded at the tip
    let (l, b, h1, h2, p) = (20.0, 2.0, 1.0, 1.0, 100.0);
    let (e1, e2) = (210e3, 70e3);
    let blocks = [
        Block {
            origin: [0.0; 3],
            size: [l, b, h1],
            cells: [40, 4, 2],
            material: mat(e1, 0.3, 7.85e-9),
        },
        Block {
            origin: [0.0, 0.0, h1],
            size: [l, b, h2],
            cells: [30, 3, 2],
            material: mat(e2, 0.3, 2.7e-9),
        },
    ];
    let (mut model, merged) = build(&blocks, 1e-6);
    assert!(!model.ties.is_empty());
    model.fix_where([true; 3], &|x| x[0] < 1e-9);
    // the end faces of both layers (local tag 1 = x max) carry the shear load
    let tip_tags: Vec<u32> = (0..2).map(|bi| merged.tag_offset[bi] + 1).collect();
    for t in &tip_tags {
        model.traction.push((*t, [0.0, 0.0, -p / (b * (h1 + h2))]));
    }
    let sol = model.solve(SolverKind::Direct).unwrap();
    let (mut sum, mut cnt) = (0.0, 0);
    for (i, x) in model.mesh.nodes.iter().enumerate() {
        if (x[0] - l).abs() < 1e-9 {
            sum += sol.u[3 * i + 2];
            cnt += 1;
        }
    }
    let tip = -sum / cnt as f64;
    // transformed section: neutral axis from the bottom, then ΣE_i I_i about it
    let zbar = (e1 * h1 * h1 / 2.0 + e2 * h2 * (h1 + h2 / 2.0)) / (e1 * h1 + e2 * h2);
    let ei = e1 * (b * h1.powi(3) / 12.0 + b * h1 * (h1 / 2.0 - zbar).powi(2))
        + e2 * (b * h2.powi(3) / 12.0 + b * h2 * (h1 + h2 / 2.0 - zbar).powi(2));
    let beam = p * l.powi(3) / (3.0 * ei);
    assert!(
        tip > 0.92 * beam && tip < 1.08 * beam,
        "tip {tip} vs transformed-section beam {beam}"
    );
}
