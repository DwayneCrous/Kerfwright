//! The FEA workspace: material, mesh size, supports and loads on faces picked in the 3-D view, a static
//! solve with quadratic tetrahedra on a background thread, and the stress / displacement shown on the model.

use crate::job::Job;
use crate::project::Project;
use crate::viewport::{ModelScene, Pick, Viewport3d};
use eframe::egui;
use kw_fem::{Fixed, Material, Model, SolverKind};
use kw_mesh3d::refine::{mesh_surface, MeshParams};
use kw_mesh3d::SurfaceMesh;
use kw_store::{FeaLoad, FeaStudy};
use kw_tess::{tessellate_body, TessOptions};
use kw_viewer::scene::{Scene, Vertex};
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    Model,
    VonMises,
    Displacement,
}

/// The outcome of a solve, kept for display.
#[derive(Debug, Clone)]
pub struct FeaResult {
    pub nodes: Vec<[f64; 3]>,
    /// Boundary triangles (corners, then mid-nodes of edges 01, 12, 20).
    pub surface: Vec<[u32; 6]>,
    pub u: Vec<f64>,
    pub von_mises: Vec<f64>,
    pub max_displacement: f64,
    pub max_von_mises: f64,
    pub dofs: usize,
    pub elements: usize,
    pub seconds: f64,
    pub radius_edge_ok: f64,
}

/// Material presets (E in MPa with lengths in mm and forces in N; density in t/mm³).
pub const MATERIALS: [(&str, Material); 4] = [
    (
        "Steel",
        Material {
            e: 210e3,
            nu: 0.30,
            rho: 7.85e-9,
        },
    ),
    (
        "Aluminium",
        Material {
            e: 70e3,
            nu: 0.33,
            rho: 2.70e-9,
        },
    ),
    (
        "Titanium",
        Material {
            e: 110e3,
            nu: 0.34,
            rho: 4.43e-9,
        },
    ),
    (
        "ABS plastic",
        Material {
            e: 2.3e3,
            nu: 0.35,
            rho: 1.05e-9,
        },
    ),
];

/// Mesh, apply the study's boundary conditions and solve (blocking; run it on a job).
pub fn solve_study(brep: &kw_brep::BRep, study: &FeaStudy) -> Result<FeaResult, String> {
    let t0 = std::time::Instant::now();
    let body = brep.bodies.keys().next().ok_or("the model has no solid")?;
    if study.fixed_tags.is_empty() {
        return Err("fix at least one face first (pick a face, then \"Fix selected face\")".into());
    }
    let tess = tessellate_body(
        brep,
        body,
        &TessOptions {
            chord_tol: 0.005,
            angle_tol: 12f64.to_radians(),
        },
    )
    .map_err(|e| format!("{e:?}"))?;
    let surf = SurfaceMesh::from_tess(&tess.mesh);
    let params = MeshParams {
        max_size: study.max_size,
        radius_edge: study.radius_edge,
        ..MeshParams::default()
    };
    let (mesh, rep) = mesh_surface(&surf, &params).map_err(|e| format!("meshing failed: {e}"))?;
    let q = mesh.to_tet10(None);
    let quality = {
        let good = mesh
            .tets
            .iter()
            .filter(|t| kw_mesh3d::radius_edge_ratio(&mesh.nodes, t) <= 2.0)
            .count();
        good as f64 / mesh.tets.len().max(1) as f64
    };
    let _ = rep;
    let mut model = Model::new(q, study.material);
    // supports: every node of a fixed face
    let mut fixed_nodes: Vec<u32> = Vec::new();
    for s in &model.mesh.surface {
        if study.fixed_tags.contains(&s.tag) {
            fixed_nodes.extend_from_slice(&s.v);
        }
    }
    fixed_nodes.sort_unstable();
    fixed_nodes.dedup();
    for n in fixed_nodes {
        for axis in 0..3 {
            model.fixed.push(Fixed {
                node: n,
                axis,
                value: 0.0,
            });
        }
    }
    // loads
    let area_of = |tag: u32| -> f64 {
        surf.tris
            .iter()
            .zip(&surf.tags)
            .filter(|(_, t)| **t == tag)
            .map(|(t, _)| {
                let (a, b, c) = (
                    surf.nodes[t[0] as usize],
                    surf.nodes[t[1] as usize],
                    surf.nodes[t[2] as usize],
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
                0.5 * (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt()
            })
            .sum()
    };
    if study.loads.is_empty() {
        return Err(
            "add a load first (pick a face, then \"Add force\" or \"Add pressure\")".into(),
        );
    }
    for l in &study.loads {
        match l {
            FeaLoad::Force { tag, force } => {
                let a = area_of(*tag);
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
    let dofs = 3 * model.mesh.nodes.len();
    let kind = if dofs <= 250_000 {
        SolverKind::Direct
    } else {
        SolverKind::Pcg(kw_fem::Precond::BlockJacobi)
    };
    let sol = model.solve(kind)?;
    let nodal = kw_fem::stress::nodal_stress(&model, &sol.u);
    let vm = kw_fem::stress::von_mises_nodal(&nodal);
    let n = model.mesh.nodes.len();
    let max_displacement = (0..n)
        .map(|i| {
            (sol.u[3 * i].powi(2) + sol.u[3 * i + 1].powi(2) + sol.u[3 * i + 2].powi(2)).sqrt()
        })
        .fold(0.0, f64::max);
    let max_von_mises = vm.iter().cloned().fold(0.0, f64::max);
    Ok(FeaResult {
        nodes: model.mesh.nodes.clone(),
        surface: model.mesh.surface.iter().map(|s| s.v).collect(),
        u: sol.u,
        von_mises: vm,
        max_displacement,
        max_von_mises,
        dofs,
        elements: model.mesh.elems.len(),
        seconds: t0.elapsed().as_secs_f64(),
        radius_edge_ok: quality,
    })
}

/// Blue → cyan → green → yellow → red.
pub fn colour_map(t: f64) -> [u8; 4] {
    let t = t.clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.25 {
        (0.0, 4.0 * t, 1.0)
    } else if t < 0.5 {
        (0.0, 1.0, 1.0 - 4.0 * (t - 0.25))
    } else if t < 0.75 {
        (4.0 * (t - 0.5), 1.0, 0.0)
    } else {
        (1.0, 1.0 - 4.0 * (t - 0.75), 0.0)
    };
    [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8, 255]
}

/// The boundary of the result as a coloured scene. `scale` multiplies the displacements.
pub fn result_scene(r: &FeaResult, field: Field, scale: f64) -> ModelScene {
    let pos = |i: u32| -> [f64; 3] {
        let k = i as usize;
        [
            r.nodes[k][0] + scale * r.u[3 * k],
            r.nodes[k][1] + scale * r.u[3 * k + 1],
            r.nodes[k][2] + scale * r.u[3 * k + 2],
        ]
    };
    let value = |i: u32| -> f64 {
        match field {
            Field::VonMises => r.von_mises[i as usize] / r.max_von_mises.max(1e-30),
            Field::Displacement => {
                let k = i as usize;
                (r.u[3 * k].powi(2) + r.u[3 * k + 1].powi(2) + r.u[3 * k + 2].powi(2)).sqrt()
                    / r.max_displacement.max(1e-30)
            }
            Field::Model => 0.5,
        }
    };
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for i in 0..r.nodes.len() as u32 {
        let p = pos(i);
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let centre = [
        (lo[0] + hi[0]) / 2.0,
        (lo[1] + hi[1]) / 2.0,
        (lo[2] + hi[2]) / 2.0,
    ];
    let radius =
        0.5 * ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for s in &r.surface {
        // four sub-triangles of the six-node triangle
        for sub in [[0usize, 3, 5], [3, 1, 4], [5, 4, 2], [3, 4, 5]] {
            let p: Vec<[f64; 3]> = sub.iter().map(|k| pos(s[*k])).collect();
            let (u, v) = (
                [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]],
                [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]],
            );
            let mut n = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-30);
            n = [n[0] / len, n[1] / len, n[2] / len];
            for (j, k) in sub.iter().enumerate() {
                indices.push(vertices.len() as u32);
                vertices.push(Vertex {
                    pos: [
                        (p[j][0] - centre[0]) as f32,
                        (p[j][1] - centre[1]) as f32,
                        (p[j][2] - centre[2]) as f32,
                    ],
                    normal: [n[0] as f32, n[1] as f32, n[2] as f32],
                    // not a pickable id (0 would read as "selected" while nothing is selected)
                    id: u32::MAX,
                    color: colour_map(value(s[*k])),
                });
            }
        }
    }
    ModelScene::custom(Scene {
        vertices,
        indices,
        segments: Vec::new(),
        pick: Vec::new(),
        center: nalgebra::Vector3::new(centre[0], centre[1], centre[2]),
        radius,
    })
}

pub struct FeaWorkspace {
    pub viewport: Viewport3d,
    pub study: FeaStudy,
    pub field: Field,
    pub scale: f64,
    pub result: Option<FeaResult>,
    pub message: String,
    pub force: [f64; 3],
    pub pressure: f64,
    picked: Option<(kw_brep::FaceId, u32)>,
    job: Option<Job<FeaResult>>,
    version: u64,
    initialised: bool,
}

impl Default for FeaWorkspace {
    fn default() -> Self {
        FeaWorkspace {
            viewport: Viewport3d::default(),
            study: FeaStudy {
                name: "Static study".into(),
                body: String::new(),
                material: MATERIALS[0].1,
                max_size: 5.0,
                radius_edge: 2.0,
                fixed_tags: Vec::new(),
                loads: Vec::new(),
            },
            field: Field::Model,
            scale: 1.0,
            result: None,
            message: String::new(),
            force: [0.0, 0.0, -100.0],
            pressure: 1.0,
            picked: None,
            job: None,
            version: 0,
            initialised: false,
        }
    }
}

impl FeaWorkspace {
    fn store(&self, project: &mut Project) {
        project.fea = Some(self.study.clone());
        project.mark_extras_dirty();
    }

    pub fn fix_face(&mut self, project: &mut Project, tag: u32) {
        if !self.study.fixed_tags.contains(&tag) {
            self.study.fixed_tags.push(tag);
            self.store(project);
        }
    }

    pub fn add_force(&mut self, project: &mut Project, tag: u32, force: [f64; 3]) {
        self.study.loads.push(FeaLoad::Force { tag, force });
        self.store(project);
    }

    pub fn add_pressure(&mut self, project: &mut Project, tag: u32, pressure: f64) {
        self.study.loads.push(FeaLoad::Pressure { tag, pressure });
        self.store(project);
    }

    pub fn start(&mut self, brep: &Arc<kw_brep::BRep>) {
        let (b, study) = (brep.clone(), self.study.clone());
        self.job = Some(Job::spawn("static solve", move || solve_study(&b, &study)));
        self.message = "meshing and solving…".into();
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        let ev = project.evaluation();
        let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
        let model_key = ev
            .final_hashes()
            .map(|h| h.0 ^ h.1.rotate_left(1))
            .unwrap_or(0);
        if !self.initialised {
            if let Some(s) = &project.fea {
                self.study = s.clone();
            } else if let Some(b) = &brep {
                if let Some(body) = b.bodies.keys().next() {
                    let d = b.body_bbox(body).diagonal();
                    self.study.max_size = (d / 12.0 * 10.0).round() / 10.0;
                }
            }
            self.initialised = true;
        }
        if let Some(job) = &self.job {
            match job.poll() {
                Some(Ok(r)) => {
                    self.message = format!("solved: {} elements, {} DOF in {:.1} s; max displacement {:.4} mm, max von Mises {:.2} MPa; {:.1} % of elements have radius–edge ≤ 2", r.elements, r.dofs, r.seconds, r.max_displacement, r.max_von_mises, 100.0 * r.radius_edge_ok);
                    self.result = Some(r);
                    self.field = Field::VonMises;
                    // a sensible default exaggeration: the largest displacement shown as 5 % of the model size
                    if let (Some(b), Some(r)) = (&brep, &self.result) {
                        if let Some(body) = b.bodies.keys().next() {
                            let d = b.body_bbox(body).diagonal();
                            self.scale = (0.05 * d / r.max_displacement.max(1e-30))
                                .clamp(1.0, 1e5)
                                .round();
                        }
                    }
                    self.version += 1;
                    self.job = None;
                }
                Some(Err(e)) => {
                    self.message = e;
                    self.job = None;
                }
                None => ui.ctx().request_repaint(),
            }
        }
        egui::Panel::left("fea_side")
            .default_size(320.0)
            .show(ui, |ui| self.side(ui, project, brep.as_ref()));
        egui::CentralPanel::default().show(ui, |ui| {
            let showing_result = self.result.is_some() && self.field != Field::Model;
            let key = if showing_result {
                0x5eed
                    ^ ((self.version as u128) << 64)
                    ^ (self.field as u128 + 1)
                    ^ (self.scale.to_bits() as u128) << 8
            } else {
                model_key
            };
            self.viewport.wireframe = false;
            let (res, field, scale, b) =
                (self.result.clone(), self.field, self.scale, brep.clone());
            let highlight = self.picked.map(|(f, _)| Pick::Face(f));
            let hit = self.viewport.show_with(
                ui,
                rs,
                key,
                move || match (&res, field) {
                    (Some(r), f) if f != Field::Model => result_scene(r, f, scale),
                    _ => match &b {
                        Some(b) => ModelScene::build(b),
                        None => ModelScene::empty(),
                    },
                },
                highlight,
            );
            if let Some(Some(Pick::Face(f))) = hit {
                if let Some(tag) = self.viewport.model().and_then(|m| m.face_tag(f)) {
                    self.picked = Some((f, tag));
                }
            }
            if showing_result {
                let rect = ui.max_rect();
                let (lo, hi) = (
                    rect.left_bottom() + egui::vec2(12.0, -170.0),
                    rect.left_bottom() + egui::vec2(32.0, -12.0),
                );
                let bar = egui::Rect::from_two_pos(lo, hi);
                for i in 0..64 {
                    let t0 = i as f32 / 64.0;
                    let y1 = bar.bottom() - bar.height() * t0;
                    let y0 = bar.bottom() - bar.height() * (t0 + 1.0 / 64.0);
                    let c = colour_map(((t0 + 0.5 / 64.0) as f64).clamp(0.0, 1.0));
                    ui.painter().rect_filled(
                        egui::Rect::from_min_max(
                            egui::pos2(bar.left(), y0),
                            egui::pos2(bar.right(), y1),
                        ),
                        0.0,
                        egui::Color32::from_rgb(c[0], c[1], c[2]),
                    );
                }
                if let Some(r) = &self.result {
                    let (max, unit) = if self.field == Field::VonMises {
                        (r.max_von_mises, "MPa")
                    } else {
                        (r.max_displacement, "mm")
                    };
                    ui.painter().text(
                        egui::pos2(bar.right() + 6.0, bar.top()),
                        egui::Align2::LEFT_TOP,
                        format!("{max:.3} {unit}"),
                        egui::FontId::proportional(13.0),
                        egui::Color32::WHITE,
                    );
                    ui.painter().text(
                        egui::pos2(bar.right() + 6.0, bar.bottom()),
                        egui::Align2::LEFT_BOTTOM,
                        format!("0 {unit}"),
                        egui::FontId::proportional(13.0),
                        egui::Color32::WHITE,
                    );
                }
            }
        });
    }

    fn side(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        brep: Option<&Arc<kw_brep::BRep>>,
    ) {
        let busy = self.job.is_some();
        let mut changed = false;
        ui.heading("Study");
        egui::ComboBox::from_id_salt("fea_material")
            .selected_text(
                MATERIALS
                    .iter()
                    .find(|(_, m)| *m == self.study.material)
                    .map(|(n, _)| *n)
                    .unwrap_or("Custom"),
            )
            .show_ui(ui, |ui| {
                for (name, m) in MATERIALS {
                    if ui
                        .selectable_label(self.study.material == m, name)
                        .clicked()
                    {
                        self.study.material = m;
                        changed = true;
                    }
                }
            });
        ui.horizontal(|ui| {
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.study.material.e)
                        .speed(100.0)
                        .prefix("E ")
                        .suffix(" MPa"),
                )
                .changed();
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.study.material.nu)
                        .speed(0.005)
                        .range(0.0..=0.49)
                        .prefix("ν "),
                )
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("Element size");
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.study.max_size)
                        .speed(0.1)
                        .range(0.2..=1e4)
                        .suffix(" mm"),
                )
                .changed();
        });
        ui.separator();
        ui.heading("Supports and loads");
        match self.picked {
            Some((_, tag)) => ui.label(format!("Selected face: {tag}")),
            None => ui.weak("Click a face in the 3-D view."),
        };
        let tag = self.picked.map(|(_, t)| t);
        if ui
            .add_enabled(
                tag.is_some() && !busy,
                egui::Button::new("Fix selected face"),
            )
            .clicked()
        {
            if let Some(t) = tag {
                self.fix_face(project, t);
            }
        }
        ui.horizontal(|ui| {
            ui.add(
                egui::DragValue::new(&mut self.force[0])
                    .speed(1.0)
                    .prefix("Fx "),
            );
            ui.add(
                egui::DragValue::new(&mut self.force[1])
                    .speed(1.0)
                    .prefix("Fy "),
            );
            ui.add(
                egui::DragValue::new(&mut self.force[2])
                    .speed(1.0)
                    .prefix("Fz "),
            );
        });
        if ui
            .add_enabled(
                tag.is_some() && !busy,
                egui::Button::new("Add force (N) on selected face"),
            )
            .clicked()
        {
            if let Some(t) = tag {
                let f = self.force;
                self.add_force(project, t, f);
            }
        }
        ui.horizontal(|ui| {
            ui.add(
                egui::DragValue::new(&mut self.pressure)
                    .speed(0.1)
                    .prefix("p ")
                    .suffix(" MPa"),
            );
            if ui
                .add_enabled(tag.is_some() && !busy, egui::Button::new("Add pressure"))
                .clicked()
            {
                if let Some(t) = tag {
                    let p = self.pressure;
                    self.add_pressure(project, t, p);
                }
            }
        });
        let mut remove_fixed = None;
        for (i, t) in self.study.fixed_tags.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("Fixed: face {t}"));
                if ui.small_button("Remove").clicked() {
                    remove_fixed = Some(i);
                }
            });
        }
        if let Some(i) = remove_fixed {
            self.study.fixed_tags.remove(i);
            changed = true;
        }
        let mut remove_load = None;
        for (i, l) in self.study.loads.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(match l {
                    FeaLoad::Force { tag, force } => format!(
                        "Force on face {tag}: ({:.0}, {:.0}, {:.0}) N",
                        force[0], force[1], force[2]
                    ),
                    FeaLoad::Pressure { tag, pressure } => {
                        format!("Pressure on face {tag}: {pressure} MPa")
                    }
                    FeaLoad::Gravity { g } => {
                        format!("Gravity ({:.0}, {:.0}, {:.0})", g[0], g[1], g[2])
                    }
                });
                if ui.small_button("Remove").clicked() {
                    remove_load = Some(i);
                }
            });
        }
        if let Some(i) = remove_load {
            self.study.loads.remove(i);
            changed = true;
        }
        if changed {
            self.store(project);
        }
        ui.separator();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!busy && brep.is_some(), egui::Button::new("Solve"))
                .clicked()
            {
                if let Some(b) = brep {
                    self.start(b);
                }
            }
            if busy {
                ui.spinner();
            }
        });
        if self.result.is_some() {
            ui.separator();
            ui.heading("Results");
            ui.horizontal(|ui| {
                for (f, name) in [
                    (Field::Model, "Model"),
                    (Field::VonMises, "Von Mises"),
                    (Field::Displacement, "Displacement"),
                ] {
                    ui.selectable_value(&mut self.field, f, name);
                }
            });
            ui.add(
                egui::Slider::new(&mut self.scale, 1.0..=2000.0)
                    .logarithmic(true)
                    .text("Deformation ×"),
            );
        }
        if !self.message.is_empty() {
            ui.separator();
            ui.label(&self.message);
        }
    }
}
