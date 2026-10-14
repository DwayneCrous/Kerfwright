//! The FEA workspace: material, mesh size, supports and loads on faces picked in the 3-D view, a static
//! solve with quadratic tetrahedra on a background thread, and the stress / displacement shown on the model.

use crate::fea_run::{run_study, study_material, StudyModel};
use crate::job::Job;
use crate::model_source::{Combined, ModelSource};
use crate::project::Project;
use crate::viewport::{ModelScene, Pick, Viewport3d};
use eframe::egui;
use kw_fem::Material;
use kw_store::{
    BodyKey, FeaLoad, FeaStudy, SourceKind, StudyBody, StudyKind, StudyMaterial, Support, ThermalBc,
};
use kw_viewer::scene::{Scene, Vertex};
use std::collections::HashSet;
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Field {
    Model,
    VonMises,
    Displacement,
    /// The result field of this index in [`FeaResult::fields`].
    Named(usize),
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
    pub kind: StudyKind,
    /// The results a study offers to colour the model by (per node).
    pub fields: Vec<NamedField>,
    /// Natural frequencies and mode shapes of a modal study.
    pub modes: Vec<Mode>,
    /// Load-displacement points `[monitored displacement, load factor]` of a nonlinear study.
    pub curve: Vec<[f64; 2]>,
    /// How many nodes were tied to other bodies.
    pub ties: usize,
    pub body_of_node: Vec<u16>,
}

/// A scalar result at every node.
#[derive(Debug, Clone)]
pub struct NamedField {
    pub name: String,
    pub unit: String,
    pub values: Vec<f64>,
}

/// A natural frequency (Hz) and its mode shape (3 per node).
#[derive(Debug, Clone)]
pub struct Mode {
    pub frequency: f64,
    pub shape: Vec<f64>,
}

/// The FEA constants of a library material (E in MPa with lengths in mm and forces in N; density in t/mm³).
pub fn fem_material(m: &kw_materials::MaterialDef) -> Material {
    Material {
        e: m.e,
        nu: m.nu,
        rho: m.rho_fem(),
    }
}

/// The material a new study starts with.
fn default_material() -> Material {
    let lib = kw_materials::global();
    fem_material(lib.find("Steel").unwrap_or(&lib.all()[0]))
}

/// Mesh, apply the study's boundary conditions and solve (blocking; run it on a job).
pub fn solve_study(brep: &kw_brep::BRep, study: &FeaStudy) -> Result<FeaResult, String> {
    solve_study_with(
        brep,
        study,
        &kw_job::CancelFlag::new(),
        &kw_job::Progress::new(),
    )
}

/// As [`solve_study`], stopping with `Err("cancelled")` when `cancel` is raised and reporting how far it is.
/// Works on the first body of `brep` with the study's own material.
pub fn solve_study_with(
    brep: &kw_brep::BRep,
    study: &FeaStudy,
    cancel: &kw_job::CancelFlag,
    progress: &kw_job::Progress,
) -> Result<FeaResult, String> {
    let body = brep.bodies.keys().next().ok_or("the model has no solid")?;
    let material = kw_store::StudyMaterial {
        name: "Study material".into(),
        e: study.material.e,
        nu: study.material.nu,
        density: study.material.rho * 1e12,
        yield_strength: 0.0,
        hardening_modulus: 0.0,
        conductivity: 0.0,
        expansion: 0.0,
        ref_temperature: 20.0,
    };
    let model = crate::fea_run::StudyModel {
        brep,
        bodies: vec![(kw_store::BodyKey::Assembly(String::new()), body)],
        materials: vec![material],
    };
    crate::fea_run::run_study(&model, study, cancel, progress)
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

/// The values of `field` at every node and the range they are coloured over, with the unit.
pub fn field_range(r: &FeaResult, field: Field) -> (f64, f64, String) {
    let span = |v: &dyn Fn(usize) -> f64| {
        let n = r.nodes.len();
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        for i in 0..n {
            lo = lo.min(v(i));
            hi = hi.max(v(i));
        }
        (lo, hi)
    };
    match field {
        Field::Model => (0.0, 1.0, String::new()),
        Field::VonMises => (0.0, r.max_von_mises, "MPa".into()),
        Field::Displacement => (0.0, r.max_displacement, "mm".into()),
        Field::Named(k) => match r.fields.get(k) {
            Some(f) => {
                let (lo, hi) = span(&|i| f.values[i]);
                // stresses and displacements start at zero; a temperature or a ratio does not
                let from_zero = lo >= 0.0 && matches!(f.unit.as_str(), "MPa" | "mm" | "");
                (if from_zero { 0.0 } else { lo }, hi, f.unit.clone())
            }
            None => (0.0, 1.0, String::new()),
        },
    }
}

/// The boundary of the result as a coloured scene. `scale` multiplies the displacements.
pub fn result_scene(r: &FeaResult, field: Field, scale: f64) -> ModelScene {
    result_scene_with(r, field, scale, None)
}

/// As [`result_scene`], moving the nodes by `displacement` (3 per node) instead of the solution's (a mode
/// shape seen at some instant of its oscillation).
pub fn result_scene_with(
    r: &FeaResult,
    field: Field,
    scale: f64,
    displacement: Option<&[f64]>,
) -> ModelScene {
    let u: &[f64] = displacement.unwrap_or(&r.u);
    let pos = |i: u32| -> [f64; 3] {
        let k = i as usize;
        [
            r.nodes[k][0] + scale * u[3 * k],
            r.nodes[k][1] + scale * u[3 * k + 1],
            r.nodes[k][2] + scale * u[3 * k + 2],
        ]
    };
    let (lo_v, hi_v, _) = field_range(r, field);
    let value = |i: u32| -> f64 {
        let k = i as usize;
        let raw = match field {
            Field::VonMises => r.von_mises[k],
            Field::Displacement => {
                (r.u[3 * k].powi(2) + r.u[3 * k + 1].powi(2) + r.u[3 * k + 2].powi(2)).sqrt()
            }
            Field::Named(f) => r.fields.get(f).map_or(0.0, |f| f.values[k]),
            Field::Model => return 0.5,
        };
        (raw - lo_v) / (hi_v - lo_v).max(1e-30)
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

/// The bodies of the model as the workspace last built them.
struct Cached {
    key: u128,
    combined: Arc<Combined>,
    names: Vec<String>,
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
    /// Inputs of the "add" buttons.
    pub gravity: [f64; 3],
    pub temperature: f64,
    pub flux: f64,
    pub convection: [f64; 2],
    pub generation: f64,
    pub roller_axes: [bool; 3],
    pub roller_value: [f64; 3],
    picked: Option<(kw_brep::FaceId, u32)>,
    job: Option<Job<FeaResult>>,
    version: u64,
    initialised: bool,
    cached: Option<Cached>,
    pub requests: Vec<&'static str>,
}

impl Default for FeaWorkspace {
    fn default() -> Self {
        FeaWorkspace {
            viewport: Viewport3d::default(),
            study: FeaStudy {
                material: default_material(),
                ..FeaStudy::default()
            },
            field: Field::Model,
            scale: 1.0,
            result: None,
            message: String::new(),
            force: [0.0, 0.0, -100.0],
            pressure: 1.0,
            gravity: [0.0, 0.0, -9810.0],
            temperature: 100.0,
            flux: 1000.0,
            convection: [25.0, 20.0],
            generation: 1e5,
            roller_axes: [true, false, false],
            roller_value: [0.0; 3],
            picked: None,
            job: None,
            version: 0,
            initialised: false,
            cached: None,
            requests: Vec::new(),
        }
    }
}

/// The library material called `name`, else steel.
fn material_named(name: Option<&str>) -> StudyMaterial {
    let lib = kw_materials::global();
    let m = name
        .and_then(|n| lib.find(n))
        .or_else(|| lib.find("Steel"))
        .unwrap_or(&lib.all()[0]);
    study_material(m)
}

impl FeaWorkspace {
    /// The ribbon of the Simulation workspace.
    pub fn ribbon(&mut self, ui: &mut egui::Ui) {
        use crate::chrome::{ribbon as rb, Icon};
        let mut cur = 0usize;
        rb::tabs(ui, &[("SIMULATION", true)], &mut cur);
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            rb::group(ui, "STUDY TYPE", |ui| {
                for k in StudyKind::ALL {
                    if rb::button(
                        ui,
                        Icon::Fea,
                        k.name(),
                        self.job_idle(),
                        self.study.kind == k,
                        "Choose what this study computes",
                    )
                    .clicked()
                    {
                        self.study.kind = k;
                        self.requests.push("kind");
                    }
                }
            });
            rb::group(ui, "STUDY", |ui| {
                if rb::button(
                    ui,
                    Icon::Play,
                    "Solve",
                    self.job_idle(),
                    false,
                    "Mesh and solve the study",
                )
                .clicked()
                {
                    self.requests.push("solve");
                }
            });
            rb::group(ui, "RESULTS", |ui| {
                let has = self.result.is_some();
                if rb::button(
                    ui,
                    Icon::Fea,
                    "Model",
                    has,
                    self.field == Field::Model,
                    "Show the model without results",
                )
                .clicked()
                {
                    self.field = Field::Model;
                }
                let names: Vec<String> = self
                    .result
                    .as_ref()
                    .map(|r| r.fields.iter().map(|f| f.name.clone()).collect())
                    .unwrap_or_default();
                for (i, name) in names.iter().take(4).enumerate() {
                    if rb::button(
                        ui,
                        Icon::Fea,
                        name,
                        has,
                        self.field == Field::Named(i),
                        "Colour the model by this result",
                    )
                    .clicked()
                    {
                        self.field = Field::Named(i);
                    }
                }
            });
        });
    }

    fn job_idle(&self) -> bool {
        self.job.is_none()
    }

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

    /// Run the study on one B-rep (its first body, with the study's own material).
    pub fn start(&mut self, brep: &Arc<kw_brep::BRep>) {
        let (b, study) = (brep.clone(), self.study.clone());
        self.job = Some(Job::spawn_with("static solve", move |cancel, progress| {
            solve_study_with(&b, &study, cancel, progress)
        }));
        self.message = "meshing and solving…".into();
    }

    /// Run the study on the bodies it includes, each with its own material.
    pub fn start_study(&mut self) {
        let Some(cached) = &self.cached else {
            self.message = "there is no solid body to run a study on".into();
            return;
        };
        let combined = cached.combined.clone();
        let study = self.study.clone();
        let materials: Vec<StudyMaterial> = study
            .bodies
            .iter()
            .filter(|b| b.included)
            .map(|b| b.material.clone())
            .collect();
        let label = study.kind.name();
        self.job = Some(Job::spawn_with(label, move |cancel, progress| {
            let model = StudyModel {
                brep: &combined.brep,
                bodies: combined.bodies.clone(),
                materials,
            };
            run_study(&model, &study, cancel, progress)
        }));
        self.message = "meshing and solving…".into();
    }

    /// Bring the bodies of the model up to date (and the study's list of them with it).
    fn refresh_model(&mut self, project: &mut Project) {
        let kind = if self.study.source == SourceKind::Assembly && project.assembly.is_some() {
            SourceKind::Assembly
        } else {
            SourceKind::Design
        };
        self.study.source = kind;
        let mut h = kw_feature::Hasher128::new();
        h.u128(project.document().content_hash());
        if kind == SourceKind::Assembly {
            if let Some(a) = &project.assembly {
                h.bytes(&a.to_bytes());
            }
        }
        for b in &self.study.bodies {
            h.u64(b.included as u64);
        }
        let key = h.finish();
        if self.cached.as_ref().is_some_and(|c| c.key == key) {
            return;
        }
        let Ok(source) = ModelSource::of(project, kind, &Default::default()) else {
            self.cached = None;
            return;
        };
        // the study's list follows the model: new bodies join (included), vanished ones leave
        let legacy_material =
            (self.study.bodies.is_empty() && source.bodies.len() == 1).then(|| {
                let m = &self.study.material;
                StudyMaterial {
                    name: "Study material".into(),
                    e: m.e,
                    nu: m.nu,
                    density: m.rho * 1e12,
                    ..material_named(source.bodies[0].material.as_deref())
                }
            });
        let mut bodies: Vec<StudyBody> = Vec::new();
        for (i, b) in source.bodies.iter().enumerate() {
            let old = self.study.bodies.iter().find(|x| x.key == b.key);
            bodies.push(match old {
                Some(o) => o.clone(),
                None => StudyBody {
                    key: b.key.clone(),
                    included: true,
                    material: if i == 0 && legacy_material.is_some() {
                        legacy_material.clone().unwrap()
                    } else {
                        material_named(b.material.as_deref())
                    },
                },
            });
        }
        self.study.bodies = bodies;
        let included: HashSet<BodyKey> = self
            .study
            .bodies
            .iter()
            .filter(|b| b.included)
            .map(|b| b.key.clone())
            .collect();
        let combined = Arc::new(source.combined(&|k| included.contains(k)));
        self.cached = Some(Cached {
            key,
            names: combined
                .source_index
                .iter()
                .map(|i| source.bodies[*i].name.clone())
                .collect(),
            combined,
        });
        if !self.initialised || self.study.max_size <= 0.0 {
            if let Some(c) = &self.cached {
                if let Some((_, body)) = c.combined.bodies.first() {
                    let d = c.combined.brep.body_bbox(*body).diagonal();
                    self.study.max_size = (d / 12.0 * 10.0).round() / 10.0;
                }
            }
        }
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        if !self.initialised {
            if let Some(s) = &project.fea {
                self.study = s.clone();
            }
        }
        self.refresh_model(project);
        self.initialised = true;
        let brep = self.cached.as_ref().map(|c| c.combined.brep.clone());
        let model_key = self.cached.as_ref().map_or(0, |c| c.key);
        if let Some(job) = &self.job {
            match job.poll() {
                Some(Ok(r)) => {
                    self.message = summary(&r);
                    self.field = default_field(&r);
                    // a sensible default exaggeration: the largest displacement shown as 5 % of the model size
                    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
                    for p in &r.nodes {
                        for k in 0..3 {
                            lo[k] = lo[k].min(p[k]);
                            hi[k] = hi[k].max(p[k]);
                        }
                    }
                    let d = ((hi[0] - lo[0]).powi(2)
                        + (hi[1] - lo[1]).powi(2)
                        + (hi[2] - lo[2]).powi(2))
                    .sqrt();
                    let top = if r.max_displacement > 0.0 {
                        r.max_displacement
                    } else {
                        1.0
                    };
                    self.scale = (0.05 * d / top).clamp(1.0, 1e5).round();
                    self.result = Some(r);
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
        for r in std::mem::take(&mut self.requests) {
            match r {
                "solve" if self.job.is_none() => self.start_study(),
                "kind" => {
                    self.result = None;
                    self.field = Field::Model;
                    self.store(project);
                }
                _ => {}
            }
        }
        egui::Panel::left("fea_side")
            .default_size(340.0)
            .show(ui, |ui| self.side(ui, project));
        egui::CentralPanel::default().show(ui, |ui| {
            let showing_result = self.result.is_some() && self.field != Field::Model;
            // a mode shape swings back and forth
            let mode = match (&self.result, self.field) {
                (Some(r), Field::Named(i)) if r.kind == StudyKind::Modal && i < r.modes.len() => {
                    Some(i)
                }
                _ => None,
            };
            let phase = if mode.is_some() {
                ui.ctx().request_repaint();
                (ui.input(|i| i.time) * std::f64::consts::TAU * 0.7).sin()
            } else {
                1.0
            };
            let key = if showing_result {
                let mut h = kw_feature::Hasher128::new();
                h.u64(0x5eed ^ self.version);
                h.u64(match self.field {
                    Field::Model => 0,
                    Field::VonMises => 1,
                    Field::Displacement => 2,
                    Field::Named(i) => 3 + i as u64,
                });
                h.u64(self.scale.to_bits());
                h.u64((phase * 1000.0) as i64 as u64);
                h.finish()
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
                    (Some(r), f) if f != Field::Model => {
                        let shape: Option<Vec<f64>> = mode.map(|i| r.modes[i].shape.clone());
                        result_scene_with(r, f, scale * phase, shape.as_deref().or(Some(&r.u)))
                    }
                    _ => match &b {
                        Some(b) => ModelScene::build(b),
                        None => ModelScene::empty(),
                    },
                },
                highlight,
                crate::viewport::ViewOpts::default(),
            );
            if let Some(Some(Pick::Face(f))) = hit.click {
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
                    let (lo_v, hi_v, unit) = field_range(r, self.field);
                    let unit = if unit.is_empty() {
                        String::new()
                    } else {
                        format!(" {unit}")
                    };
                    ui.painter().text(
                        egui::pos2(bar.right() + 6.0, bar.top()),
                        egui::Align2::LEFT_TOP,
                        format!("{hi_v:.3}{unit}"),
                        egui::FontId::proportional(13.0),
                        egui::Color32::WHITE,
                    );
                    ui.painter().text(
                        egui::pos2(bar.right() + 6.0, bar.bottom()),
                        egui::Align2::LEFT_BOTTOM,
                        format!("{lo_v:.3}{unit}"),
                        egui::FontId::proportional(13.0),
                        egui::Color32::WHITE,
                    );
                }
            }
        });
    }

    fn side(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let busy = self.job.is_some();
        let mut changed = false;
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Study");
            ui.label(self.study.kind.name());
            if project.assembly.is_some() {
                ui.horizontal(|ui| {
                    ui.label("Bodies from");
                    for (k, name) in [(SourceKind::Design, "Design"), (SourceKind::Assembly, "Assembly")] {
                        if ui.selectable_label(self.study.source == k, name).clicked() {
                            self.study.source = k;
                            self.study.bodies.clear();
                            self.result = None;
                            self.field = Field::Model;
                            self.picked = None;
                            changed = true;
                        }
                    }
                });
            }
            // the bodies, each with its material
            let names: Vec<String> = self
                .cached
                .as_ref()
                .map(|c| c.names.clone())
                .unwrap_or_default();
            let all_names: Vec<String> = self.study.bodies.iter().map(|b| match &b.key {
                BodyKey::Design(id) => format!("{id}"),
                BodyKey::Assembly(n) => n.clone(),
            }).collect();
            let lib = kw_materials::global();
            let multi = self.study.bodies.len() > 1;
            for (i, b) in self.study.bodies.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    if multi {
                        changed |= ui.checkbox(&mut b.included, "").changed();
                    }
                    let shown = if b.included {
                        names
                            .iter()
                            .zip(self.cached.as_ref().map(|c| &c.combined.bodies).into_iter().flatten())
                            .find(|(_, (k, _))| *k == b.key)
                            .map(|(n, _)| n.clone())
                            .unwrap_or_else(|| all_names[i].clone())
                    } else {
                        all_names[i].clone()
                    };
                    ui.label(shown);
                    egui::ComboBox::from_id_salt(("fea_body_material", i))
                        .selected_text(b.material.name.clone())
                        .show_ui(ui, |ui| {
                            for m in lib.all() {
                                if ui
                                    .selectable_label(b.material.name == m.name, &m.name)
                                    .clicked()
                                {
                                    b.material = study_material(m);
                                    changed = true;
                                }
                            }
                        });
                });
            }
            if self.study.bodies.len() == 1 {
                let m = &mut self.study.bodies[0].material;
                ui.horizontal(|ui| {
                    changed |= ui
                        .add(egui::DragValue::new(&mut m.e).speed(100.0).prefix("E ").suffix(" MPa"))
                        .changed();
                    changed |= ui
                        .add(egui::DragValue::new(&mut m.nu).speed(0.005).range(0.0..=0.49).prefix("ν "))
                        .changed();
                });
            }
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
            if multi {
                ui.horizontal(|ui| {
                    ui.label("Bond gap");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut self.study.bond_tolerance)
                                .speed(0.001)
                                .range(0.0..=10.0)
                                .suffix(" mm"),
                        )
                        .on_hover_text("Surfaces closer than this are bonded (0: a thousandth of a millimetre)")
                        .changed();
                });
            }
            match self.study.kind {
                StudyKind::Modal => {
                    ui.horizontal(|ui| {
                        ui.label("Modes");
                        let mut n = if self.study.modes == 0 { 6 } else { self.study.modes };
                        if ui.add(egui::DragValue::new(&mut n).range(1..=30)).changed() {
                            self.study.modes = n;
                            changed = true;
                        }
                    });
                }
                StudyKind::Nonlinear => {
                    ui.horizontal(|ui| {
                        ui.label("Load steps");
                        let mut n = if self.study.nl_steps == 0 { 10 } else { self.study.nl_steps };
                        if ui.add(egui::DragValue::new(&mut n).range(1..=200)).changed() {
                            self.study.nl_steps = n;
                            changed = true;
                        }
                        changed |= ui
                            .checkbox(&mut self.study.nl_geometric, "Large displacements")
                            .changed();
                    });
                    ui.weak("Plasticity uses the yield strength and tangent modulus of the first body's material.");
                }
                _ => {}
            }
            ui.separator();
            ui.heading("Supports and loads");
            match self.picked {
                Some((_, tag)) => ui.label(format!("Selected face: {tag}")),
                None => ui.weak("Click a face in the 3-D view."),
            };
            let tag = self.picked.map(|(_, t)| t);
            let can = tag.is_some() && !busy;
            if ui.add_enabled(can, egui::Button::new("Fix selected face")).clicked() {
                if let Some(t) = tag {
                    self.fix_face(project, t);
                }
            }
            ui.horizontal(|ui| {
                for (k, l) in ["x", "y", "z"].iter().enumerate() {
                    ui.checkbox(&mut self.roller_axes[k], *l);
                }
                for k in 0..3 {
                    ui.add(egui::DragValue::new(&mut self.roller_value[k]).speed(0.01).suffix(" mm"));
                }
            });
            if ui
                .add_enabled(can, egui::Button::new("Support selected face (ticked axes)"))
                .on_hover_text("Holds the ticked components at the values given: a roller, a symmetry plane, a prescribed displacement")
                .clicked()
            {
                if let Some(t) = tag {
                    self.study.supports.push(Support {
                        tag: t,
                        axes: self.roller_axes,
                        value: self.roller_value,
                    });
                    changed = true;
                }
            }
            if matches!(self.study.kind, StudyKind::Static | StudyKind::Nonlinear | StudyKind::ThermalStress) {
                ui.horizontal(|ui| {
                    for (k, l) in ["Fx ", "Fy ", "Fz "].iter().enumerate() {
                        ui.add(egui::DragValue::new(&mut self.force[k]).speed(1.0).prefix(*l));
                    }
                });
                if ui.add_enabled(can, egui::Button::new("Add force (N) on selected face")).clicked() {
                    if let Some(t) = tag {
                        let f = self.force;
                        self.add_force(project, t, f);
                    }
                }
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.pressure).speed(0.1).prefix("p ").suffix(" MPa"));
                    if ui.add_enabled(can, egui::Button::new("Add pressure")).clicked() {
                        if let Some(t) = tag {
                            let p = self.pressure;
                            self.add_pressure(project, t, p);
                        }
                    }
                });
                ui.horizontal(|ui| {
                    for (k, l) in ["gx ", "gy ", "gz "].iter().enumerate() {
                        ui.add(egui::DragValue::new(&mut self.gravity[k]).speed(10.0).prefix(*l));
                    }
                });
                if ui
                    .add_enabled(!busy, egui::Button::new("Add gravity (mm/s²)"))
                    .clicked()
                {
                    self.study.loads.push(FeaLoad::Gravity { g: self.gravity });
                    changed = true;
                }
            }
            if matches!(self.study.kind, StudyKind::Thermal | StudyKind::ThermalStress) {
                ui.separator();
                ui.heading("Thermal conditions");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.temperature).speed(1.0).suffix(" °C"));
                    if ui.add_enabled(can, egui::Button::new("Hold selected face at")).clicked() {
                        if let Some(t) = tag {
                            self.study.thermal.push(ThermalBc::Temperature { tag: t, t: self.temperature });
                            changed = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.flux).speed(10.0).suffix(" W/m²"));
                    if ui.add_enabled(can, egui::Button::new("Heat flux into face")).clicked() {
                        if let Some(t) = tag {
                            self.study.thermal.push(ThermalBc::Flux { tag: t, q: self.flux });
                            changed = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.convection[0]).speed(1.0).prefix("h ").suffix(" W/m²K"));
                    ui.add(egui::DragValue::new(&mut self.convection[1]).speed(1.0).prefix("T∞ ").suffix(" °C"));
                    if ui.add_enabled(can, egui::Button::new("Convection")).clicked() {
                        if let Some(t) = tag {
                            self.study.thermal.push(ThermalBc::Convection {
                                tag: t,
                                h: self.convection[0],
                                t_inf: self.convection[1],
                            });
                            changed = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.generation).speed(1000.0).suffix(" W/m³"));
                    if ui.add_enabled(!busy, egui::Button::new("Heat generation")).clicked() {
                        self.study.thermal.push(ThermalBc::Generation { q: self.generation });
                        changed = true;
                    }
                });
            }
            // what has been set
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
            let mut remove_support = None;
            for (i, s) in self.study.supports.iter().enumerate() {
                ui.horizontal(|ui| {
                    let axes: String = ["x", "y", "z"]
                        .iter()
                        .zip(s.axes)
                        .filter(|(_, on)| *on)
                        .map(|(l, _)| *l)
                        .collect();
                    ui.label(format!("Support face {} ({axes})", s.tag));
                    if ui.small_button("Remove").clicked() {
                        remove_support = Some(i);
                    }
                });
            }
            if let Some(i) = remove_support {
                self.study.supports.remove(i);
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
            let mut remove_bc = None;
            for (i, bc) in self.study.thermal.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(match bc {
                        ThermalBc::Temperature { tag, t } => format!("Face {tag} held at {t} °C"),
                        ThermalBc::Flux { tag, q } => format!("Heat flux {q} W/m² into face {tag}"),
                        ThermalBc::Convection { tag, h, t_inf } => {
                            format!("Convection on face {tag}: h {h} W/m²K, {t_inf} °C")
                        }
                        ThermalBc::Generation { q } => format!("Heat generation {q} W/m³"),
                    });
                    if ui.small_button("Remove").clicked() {
                        remove_bc = Some(i);
                    }
                });
            }
            if let Some(i) = remove_bc {
                self.study.thermal.remove(i);
                changed = true;
            }
            if changed {
                self.store(project);
            }
            ui.separator();
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!busy && self.cached.is_some(), egui::Button::new("Solve"))
                    .clicked()
                {
                    self.start_study();
                }
                if let Some(job) = &self.job {
                    job.ui(ui);
                }
            });
            if let Some(r) = &self.result {
                ui.separator();
                ui.heading("Results");
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(&mut self.field, Field::Model, "Model");
                    for (i, f) in r.fields.iter().enumerate() {
                        ui.selectable_value(&mut self.field, Field::Named(i), &f.name);
                    }
                });
                if r.kind != StudyKind::Thermal {
                    ui.add(
                        egui::Slider::new(&mut self.scale, 1.0..=2000.0)
                            .logarithmic(true)
                            .text("Deformation ×"),
                    );
                }
                if !r.modes.is_empty() {
                    for (k, m) in r.modes.iter().enumerate() {
                        ui.label(format!("Mode {}: {:.2} Hz", k + 1, m.frequency));
                    }
                }
                if r.curve.len() > 1 {
                    let (rect, _) = ui.allocate_exact_size(egui::vec2(260.0, 110.0), egui::Sense::hover());
                    ui.painter().rect_stroke(rect, 0.0, egui::Stroke::new(1.0, egui::Color32::GRAY), egui::StrokeKind::Inside);
                    let top_x = r.curve.iter().map(|p| p[0].abs()).fold(1e-30, f64::max);
                    let pts: Vec<egui::Pos2> = r
                        .curve
                        .iter()
                        .map(|p| {
                            egui::pos2(
                                rect.left() + (p[0].abs() / top_x) as f32 * rect.width(),
                                rect.bottom() - p[1] as f32 * rect.height(),
                            )
                        })
                        .collect();
                    ui.painter().add(egui::Shape::line(pts, egui::Stroke::new(1.5, egui::Color32::LIGHT_BLUE)));
                    ui.weak("Load factor against the displacement of the loaded face");
                }
            }
            if !self.message.is_empty() {
                ui.separator();
                ui.label(&self.message);
            }
        });
    }
}

/// The result to show first.
fn default_field(r: &FeaResult) -> Field {
    match r.kind {
        StudyKind::Thermal | StudyKind::Modal => Field::Named(0),
        _ => r
            .fields
            .iter()
            .position(|f| f.name == "Von Mises")
            .map_or(Field::VonMises, Field::Named),
    }
}

/// One line about a finished solve.
fn summary(r: &FeaResult) -> String {
    let ties = if r.ties > 0 {
        format!("; {} nodes bonded between bodies", r.ties)
    } else {
        String::new()
    };
    match r.kind {
        StudyKind::Modal => format!(
            "solved: lowest frequency {:.2} Hz; {} elements, {} DOF in {:.1} s{ties}",
            r.modes.first().map_or(0.0, |m| m.frequency),
            r.elements,
            r.dofs,
            r.seconds
        ),
        StudyKind::Thermal => {
            let t = r.fields.first().map(|f| &f.values);
            let (lo, hi) = t.map_or((0.0, 0.0), |v| {
                (
                    v.iter().cloned().fold(f64::MAX, f64::min),
                    v.iter().cloned().fold(f64::MIN, f64::max),
                )
            });
            format!(
                "solved: {lo:.1} to {hi:.1} °C; {} elements in {:.1} s{ties}",
                r.elements, r.seconds
            )
        }
        _ => format!(
            "solved: {} elements, {} DOF in {:.1} s; max displacement {:.4} mm, max von Mises {:.2} MPa; {:.1} % of elements have radius–edge ≤ 2{ties}",
            r.elements, r.dofs, r.seconds, r.max_displacement, r.max_von_mises, 100.0 * r.radius_edge_ok
        ),
    }
}
