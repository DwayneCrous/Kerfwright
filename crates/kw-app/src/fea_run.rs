//! Running a study: mesh every body, join them (bonded contact where they touch), apply the supports and
//! loads by face tag, and solve what the study asks for. Runs on a background thread.
//!
//! Face tags of a study count the faces of the included bodies in order, so a tag picked on the second body
//! is the number of faces of the first plus its own index.

use crate::fea::{FeaResult, Mode, NamedField};
use kw_brep::{BRep, BodyId};
use kw_fem::modal::modal;
use kw_fem::nonlinear::{solve_nonlinear, Nonlinear};
use kw_fem::tie::{find_ties, TieOptions};
use kw_fem::{Fixed, Material, Model, SolverKind, ThermalLoad, ThermalModel};
use kw_job::{CancelFlag, Progress};
use kw_mesh3d::refine::{mesh_surface, MeshParams};
use kw_mesh3d::{MergedMesh, SurfaceMesh, Tet10Mesh};
use kw_store::{BodyKey, FeaLoad, FeaStudy, StudyKind, StudyMaterial, ThermalBc};
use kw_tess::{tessellate_body, TessOptions};
use std::collections::{HashMap, HashSet};

/// The geometry a study runs on: the bodies in one B-rep, with a key and a material each.
pub struct StudyModel<'a> {
    pub brep: &'a BRep,
    pub bodies: Vec<(BodyKey, BodyId)>,
    pub materials: Vec<StudyMaterial>,
}

/// The structural constants of a study material (MPa, tonne/mm³).
pub fn fem_of(m: &StudyMaterial) -> Material {
    Material {
        e: m.e,
        nu: m.nu,
        rho: m.density * 1e-12,
    }
}

/// A study material from a library material.
pub fn study_material(m: &kw_materials::MaterialDef) -> StudyMaterial {
    StudyMaterial {
        name: m.name.clone(),
        e: m.e,
        nu: m.nu,
        density: m.density,
        yield_strength: m.yield_strength,
        hardening_modulus: m.hardening_modulus,
        conductivity: m.conductivity,
        expansion: m.expansion,
        ref_temperature: m.ref_temperature,
    }
}

/// Pairs of bodies that share volume (more than `min_volume`), with the volume.
pub fn overlapping_bodies(
    brep: &BRep,
    bodies: &[BodyId],
    min_volume: f64,
) -> Vec<(usize, usize, f64)> {
    let mut out = Vec::new();
    for i in 0..bodies.len() {
        for j in i + 1..bodies.len() {
            if !brep
                .body_bbox(bodies[i])
                .intersects(&brep.body_bbox(bodies[j]))
            {
                continue;
            }
            let mut rep = BRep::new();
            let (a, _) = rep.copy_body_from(brep, bodies[i], &kw_brep::CopyOptions::default());
            let (b, _) = rep.copy_body_from(brep, bodies[j], &kw_brep::CopyOptions::default());
            if let Ok(r) = kw_boolean::intersect(&mut rep, a, b) {
                let v = rep.mass_properties(r.body).map(|m| m.volume).unwrap_or(0.0);
                if v > min_volume {
                    out.push((i, j, v));
                }
            }
        }
    }
    out
}

fn magnitude(u: &[f64], i: usize) -> f64 {
    (u[3 * i].powi(2) + u[3 * i + 1].powi(2) + u[3 * i + 2].powi(2)).sqrt()
}

fn field(name: &str, unit: &str, values: Vec<f64>) -> NamedField {
    NamedField {
        name: name.into(),
        unit: unit.into(),
        values,
    }
}

/// Run `study` on `m`. Errors say what to fix; `Err("cancelled")` when cancelled.
pub fn run_study(
    m: &StudyModel,
    study: &FeaStudy,
    cancel: &CancelFlag,
    progress: &Progress,
) -> Result<FeaResult, String> {
    let t0 = std::time::Instant::now();
    let check = || -> Result<(), String> {
        if cancel.is_cancelled() {
            Err("cancelled".into())
        } else {
            Ok(())
        }
    };
    if m.bodies.is_empty() {
        return Err("the model has no solid".into());
    }
    let kind = study.kind;
    let has_supports = !study.fixed_tags.is_empty() || !study.supports.is_empty();
    let needs_support = matches!(
        kind,
        StudyKind::Static | StudyKind::ThermalStress | StudyKind::Nonlinear
    );
    if needs_support && !has_supports {
        return Err("fix at least one face first (pick a face, then \"Fix selected face\")".into());
    }
    let mechanical = matches!(kind, StudyKind::Static | StudyKind::Nonlinear);
    if mechanical && study.loads.is_empty() {
        return Err(
            "add a load first (pick a face, then \"Add force\" or \"Add pressure\")".into(),
        );
    }
    if matches!(kind, StudyKind::Thermal | StudyKind::ThermalStress) && study.thermal.is_empty() {
        return Err(
            "add a thermal condition first (a temperature, a heat flux or convection on a picked face)"
                .into(),
        );
    }
    progress.set(0.02);
    // bodies that share volume cannot be bonded: say which
    let ids: Vec<BodyId> = m.bodies.iter().map(|b| b.1).collect();
    if ids.len() > 1 {
        if let Some((i, j, v)) = overlapping_bodies(m.brep, &ids, 1e-6).first() {
            return Err(format!(
                "bodies {} and {} overlap by {:.3} mm³: move them apart or remove the overlap",
                i + 1,
                j + 1,
                v
            ));
        }
    }
    check()?;
    // mesh every body on its own
    let mut meshes: Vec<Tet10Mesh> = Vec::new();
    let mut offsets: Vec<u32> = Vec::new();
    let mut surfs: Vec<SurfaceMesh> = Vec::new();
    let mut base = 0u32;
    let mut quality = 0.0;
    for (k, (_, body)) in m.bodies.iter().enumerate() {
        let tess = tessellate_body(
            m.brep,
            *body,
            &TessOptions {
                chord_tol: 0.005,
                angle_tol: 12f64.to_radians(),
            },
        )
        .map_err(|e| format!("{e:?}"))?;
        let surf = SurfaceMesh::from_tess(&tess.mesh);
        offsets.push(base);
        base += tess.face_keys.len() as u32;
        let params = MeshParams {
            max_size: study.max_size,
            radius_edge: study.radius_edge,
            cancel: Some(cancel.clone()),
            ..MeshParams::default()
        };
        let (tets, _) = mesh_surface(&surf, &params)
            .map_err(|e| format!("meshing body {} failed: {e}", k + 1))?;
        let good = tets
            .tets
            .iter()
            .filter(|t| kw_mesh3d::radius_edge_ratio(&tets.nodes, t) <= 2.0)
            .count();
        quality += good as f64 / tets.tets.len().max(1) as f64;
        meshes.push(tets.to_tet10(None));
        surfs.push(surf);
        progress.set(0.05 + 0.35 * (k + 1) as f64 / m.bodies.len() as f64);
        check()?;
    }
    quality /= m.bodies.len() as f64;
    let merged: MergedMesh = Tet10Mesh::concat_with_tags(&meshes, &offsets);
    let mesh = &merged.mesh;
    // bonded contact between bodies that touch
    let skip: Vec<(usize, usize)> = study
        .unbonded
        .iter()
        .filter_map(|(a, b)| {
            let ia = m.bodies.iter().position(|x| &x.0 == a)?;
            let ib = m.bodies.iter().position(|x| &x.0 == b)?;
            Some((ia, ib))
        })
        .collect();
    let (ties, _) = if m.bodies.len() > 1 {
        find_ties(
            &merged,
            &TieOptions {
                tolerance: if study.bond_tolerance > 0.0 {
                    study.bond_tolerance
                } else {
                    1e-3
                },
                skip,
            },
        )
    } else {
        (Vec::new(), Default::default())
    };
    let n_ties = ties.len();
    let nn = mesh.nodes.len();
    let mats: Vec<Material> = m.materials.iter().map(fem_of).collect();
    // supports and loads by (global) face tag
    let tag_nodes = |tags: &HashSet<u32>| -> Vec<u32> {
        let mut v: Vec<u32> = mesh
            .surface
            .iter()
            .filter(|s| tags.contains(&s.tag))
            .flat_map(|s| s.v)
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    let fixed_nodes = tag_nodes(&study.fixed_tags.iter().copied().collect());
    let mut area: HashMap<u32, f64> = HashMap::new();
    for (s, off) in surfs.iter().zip(&offsets) {
        for (t, tag) in s.tris.iter().zip(&s.tags) {
            let (a, b, c) = (
                s.nodes[t[0] as usize],
                s.nodes[t[1] as usize],
                s.nodes[t[2] as usize],
            );
            let (u, v) = (
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
            );
            let cr = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            *area.entry(tag + off).or_default() +=
                0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
        }
    }
    let structural = |loads: bool| -> Result<Model, String> {
        let mut model = Model::new(mesh.clone(), mats[0]);
        model.materials = mats.clone();
        model.elem_material = merged.elem_body.clone();
        model.ties = ties.clone();
        for &n in &fixed_nodes {
            for axis in 0..3 {
                model.fixed.push(Fixed {
                    node: n,
                    axis,
                    value: 0.0,
                });
            }
        }
        for sup in &study.supports {
            let nodes = tag_nodes(&[sup.tag].into_iter().collect());
            for n in nodes {
                for axis in 0..3 {
                    if sup.axes[axis] {
                        model.fixed.push(Fixed {
                            node: n,
                            axis,
                            value: sup.value[axis],
                        });
                    }
                }
            }
        }
        if loads {
            for l in &study.loads {
                match l {
                    FeaLoad::Force { tag, force } => {
                        let a = area.get(tag).copied().unwrap_or(0.0);
                        if a <= 0.0 {
                            return Err(format!("face {tag} has no area"));
                        }
                        model
                            .traction
                            .push((*tag, [force[0] / a, force[1] / a, force[2] / a]));
                    }
                    FeaLoad::Pressure { tag, pressure } => model.pressure.push((*tag, *pressure)),
                    FeaLoad::Gravity { g } => model.gravity = Some(*g),
                }
            }
        }
        Ok(model)
    };
    let dofs = 3 * nn;
    let kind_solver = if dofs <= 250_000 {
        SolverKind::Direct
    } else {
        SolverKind::Pcg(kw_fem::Precond::BlockJacobi)
    };
    progress.set(0.45);
    check()?;
    let mut fields: Vec<NamedField> = Vec::new();
    let mut modes: Vec<Mode> = Vec::new();
    let mut curve: Vec<[f64; 2]> = Vec::new();
    let mut u = vec![0.0; dofs];
    let mut vm: Vec<f64> = vec![0.0; nn];
    // the temperature of a thermal study (also the first half of thermal stress)
    let temperature = |cancel_check: &dyn Fn() -> Result<(), String>| -> Result<Vec<f64>, String> {
        let conduct: Vec<f64> = m.materials.iter().map(|x| x.conductivity).collect();
        let mut tm = ThermalModel::new(mesh.clone(), conduct.first().copied().unwrap_or(0.0));
        tm.conductivities = conduct;
        tm.elem_material = merged.elem_body.clone();
        tm.ties = ties.clone();
        for bc in &study.thermal {
            match bc {
                ThermalBc::Temperature { tag, t } => {
                    for n in tag_nodes(&[*tag].into_iter().collect()) {
                        tm.fixed.push((n, *t));
                    }
                }
                ThermalBc::Flux { tag, q } => tm.flux.push((*tag, q * 1e-3)),
                ThermalBc::Convection { tag, h, t_inf } => {
                    tm.convection.push((*tag, h * 1e-3, *t_inf))
                }
                ThermalBc::Generation { q } => {
                    tm.generation = q * 1e-6;
                    tm.generations = vec![q * 1e-6; m.materials.len()];
                }
            }
        }
        cancel_check()?;
        tm.solve(kind_solver)
    };
    match kind {
        StudyKind::Static | StudyKind::ThermalStress => {
            let mut model = structural(true)?;
            if kind == StudyKind::ThermalStress {
                let t = temperature(&check)?;
                progress.set(0.6);
                model.thermal = Some(ThermalLoad {
                    temperature: t.clone(),
                    expansion: m.materials.iter().map(|x| x.expansion).collect(),
                    t_ref: m.materials.first().map_or(20.0, |x| x.ref_temperature),
                });
                fields.push(field("Temperature", "°C", t));
            }
            let sol = model.solve_with(kind_solver, Some(cancel))?;
            progress.set(0.85);
            check()?;
            let nodal = kw_fem::stress::nodal_stress(&model, &sol.u);
            vm = kw_fem::stress::von_mises_nodal(&nodal);
            u = sol.u;
            fields.push(field(
                "Displacement",
                "mm",
                (0..nn).map(|i| magnitude(&u, i)).collect(),
            ));
            fields.push(field("Von Mises", "MPa", vm.clone()));
            if m.materials.iter().any(|x| x.yield_strength > 0.0) {
                let sf = (0..nn)
                    .map(|i| {
                        let y = m.materials[merged.body_of_node(i as u32)].yield_strength;
                        if y > 0.0 && vm[i] > 1e-12 {
                            (y / vm[i]).min(100.0)
                        } else {
                            100.0
                        }
                    })
                    .collect();
                fields.push(field("Safety factor", "", sf));
            }
        }
        StudyKind::Modal => {
            let model = structural(false)?;
            let count = if study.modes == 0 { 6 } else { study.modes };
            let shift = if has_supports { 0.0 } else { 1.0 };
            let r = modal(&model, count, shift)?;
            progress.set(0.9);
            for (k, (f, shape)) in r.frequencies.iter().zip(&r.shapes).enumerate() {
                let top = (0..nn)
                    .map(|i| magnitude(shape, i))
                    .fold(0.0, f64::max)
                    .max(1e-30);
                fields.push(field(
                    &format!("Mode {} ({:.2} Hz)", k + 1, f),
                    "",
                    (0..nn).map(|i| magnitude(shape, i) / top).collect(),
                ));
                modes.push(Mode {
                    frequency: *f,
                    shape: shape.clone(),
                });
            }
            if let Some(first) = modes.first() {
                u = first.shape.clone();
            }
        }
        StudyKind::Thermal => {
            let t = temperature(&check)?;
            progress.set(0.9);
            fields.push(field("Temperature", "°C", t));
        }
        StudyKind::Nonlinear => {
            let model = structural(true)?;
            let first = &m.materials[0];
            let nl = Nonlinear {
                yield_stress: first.yield_strength,
                tangent_modulus: first.hardening_modulus,
                geometric: study.nl_geometric,
                steps: if study.nl_steps == 0 {
                    10
                } else {
                    study.nl_steps
                },
                ..Nonlinear::default()
            };
            // follow the displacement of a node of the first loaded face, along the larger force component
            let monitor = study.loads.iter().find_map(|l| match l {
                FeaLoad::Force { tag, force } => {
                    let axis = (0..3)
                        .max_by(|a, b| force[*a].abs().total_cmp(&force[*b].abs()))
                        .unwrap_or(0);
                    tag_nodes(&[*tag].into_iter().collect())
                        .first()
                        .map(|n| (*n, axis))
                }
                _ => None,
            });
            let r = solve_nonlinear(&model, &nl, monitor, Some(cancel), Some(progress))?;
            u = r.u;
            vm = r.von_mises.clone();
            curve = r
                .load_factors
                .iter()
                .zip(&r.monitor)
                .map(|(l, d)| [*d, *l])
                .collect();
            fields.push(field(
                "Displacement",
                "mm",
                (0..nn).map(|i| magnitude(&u, i)).collect(),
            ));
            fields.push(field("Von Mises", "MPa", r.von_mises));
            fields.push(field("Plastic strain", "", r.plastic_strain));
        }
    }
    let max_displacement = (0..nn).map(|i| magnitude(&u, i)).fold(0.0, f64::max);
    let max_von_mises = vm.iter().cloned().fold(0.0, f64::max);
    Ok(FeaResult {
        nodes: mesh.nodes.clone(),
        surface: mesh.surface.iter().map(|s| s.v).collect(),
        u,
        von_mises: vm,
        max_displacement,
        max_von_mises,
        dofs,
        elements: mesh.elems.len(),
        seconds: t0.elapsed().as_secs_f64(),
        radius_edge_ok: quality,
        kind,
        fields,
        modes,
        curve,
        ties: n_ties,
        body_of_node: (0..nn)
            .map(|i| merged.body_of_node(i as u32) as u16)
            .collect(),
    })
}
