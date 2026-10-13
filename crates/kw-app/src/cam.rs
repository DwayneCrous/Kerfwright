//! The CAM workspace: tool and operation settings, toolpath generation (pocket, adaptive clearing, contour,
//! 3-D finishing) on a background thread, a stock-removal check, and posting to five controller dialects.

use crate::dialogs::Dialogs;
use crate::job::Job;
use crate::project::Project;
use crate::viewport::{ModelScene, Viewport3d};
use eframe::egui;
use kw_cam::adaptive::{adaptive, AdaptiveParams};
use kw_cam::bvh::TriMesh;
use kw_cam::dexel::TriDexel;
use kw_cam::finish3d::{moves, raster_finish, RasterParams};
use kw_cam::ops2d::{contour, pocket, Cut, Side};
use kw_cam::path::{Event, Toolpath};
use kw_cam::post::{render, PostConfig};
use kw_cam::{Region, Tool};
use kw_store::{CamOperation, CamSetup};
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpKind {
    Pocket,
    Adaptive,
    ContourOutside,
    Finish3d,
}

impl OpKind {
    pub const ALL: [OpKind; 4] = [
        OpKind::Pocket,
        OpKind::Adaptive,
        OpKind::ContourOutside,
        OpKind::Finish3d,
    ];
    pub fn label(self) -> &'static str {
        match self {
            OpKind::Pocket => "Pocket",
            OpKind::Adaptive => "Adaptive clearing",
            OpKind::ContourOutside => "Contour (outside)",
            OpKind::Finish3d => "3-D finish (ball)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolChoice {
    Flat,
    Ball,
    Bull,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dialect {
    Fanuc,
    Haas,
    Siemens,
    Heidenhain,
    LinuxCnc,
}

impl Dialect {
    pub const ALL: [Dialect; 5] = [
        Dialect::Fanuc,
        Dialect::Haas,
        Dialect::Siemens,
        Dialect::Heidenhain,
        Dialect::LinuxCnc,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Dialect::Fanuc => "Fanuc",
            Dialect::Haas => "Haas",
            Dialect::Siemens => "Siemens 840D",
            Dialect::Heidenhain => "Heidenhain TNC",
            Dialect::LinuxCnc => "LinuxCNC",
        }
    }
    pub fn config(self) -> PostConfig {
        match self {
            Dialect::Fanuc => PostConfig::fanuc(),
            Dialect::Haas => PostConfig::haas(),
            Dialect::Siemens => PostConfig::siemens(),
            Dialect::Heidenhain => PostConfig::heidenhain(),
            Dialect::LinuxCnc => PostConfig::linuxcnc(),
        }
    }
    pub fn extension(self) -> &'static str {
        match self {
            Dialect::Heidenhain => "h",
            Dialect::Siemens => "mpf",
            Dialect::LinuxCnc => "ngc",
            _ => "nc",
        }
    }
}

/// All the settings of one operation, as text fields.
#[derive(Debug, Clone)]
pub struct Settings {
    pub op: OpKind,
    pub tool: ToolChoice,
    pub diameter: f64,
    pub corner: f64,
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
    pub top: f64,
    pub bottom: f64,
    pub step_down: f64,
    pub stepover: f64,
    pub feed: f64,
    pub rpm: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            op: OpKind::Pocket,
            tool: ToolChoice::Flat,
            diameter: 6.0,
            corner: 1.0,
            x0: 0.0,
            y0: 0.0,
            x1: 40.0,
            y1: 25.0,
            top: 10.0,
            bottom: 7.0,
            step_down: 3.0,
            stepover: 2.0,
            feed: 800.0,
            rpm: 8000.0,
        }
    }
}

impl Settings {
    pub fn tool(&self) -> Tool {
        match self.tool {
            ToolChoice::Flat => Tool::flat(1, self.diameter, 25.0),
            ToolChoice::Ball => Tool::ball(1, self.diameter, 25.0),
            ToolChoice::Bull => {
                Tool::bull(1, self.diameter, self.corner.min(self.diameter / 2.0), 25.0)
            }
        }
    }

    pub fn cut(&self) -> Cut {
        let mut c = Cut::new(self.top, self.bottom);
        c.step_down = self.step_down.max(0.1);
        c.stepover = self.stepover.max(0.05);
        c.feed = self.feed;
        c.rpm = self.rpm;
        c
    }
}

/// Generate the toolpath for `s` (blocking; run it on a job). `mesh` is only used by 3-D finishing.
pub fn generate(s: &Settings, mesh: Option<&TriMesh>) -> Result<Toolpath, String> {
    let tool = s.tool();
    let cut = s.cut();
    if s.top <= s.bottom {
        return Err("the top must be above the bottom".into());
    }
    let (w, h) = (s.x1 - s.x0, s.y1 - s.y0);
    match s.op {
        OpKind::Pocket | OpKind::Adaptive => {
            if w <= tool.diameter || h <= tool.diameter {
                return Err(format!(
                    "the region {w:.1} × {h:.1} mm is too small for a {} mm tool",
                    tool.diameter
                ));
            }
            let region = Region::rect(s.x0, s.y0, s.x1, s.y1);
            Ok(if s.op == OpKind::Pocket {
                pocket(&region, &tool, &cut)
            } else {
                adaptive(&region, &tool, &cut, &AdaptiveParams::default()).0
            })
        }
        OpKind::ContourOutside => {
            let region = Region::rect(s.x0, s.y0, s.x1, s.y1);
            Ok(contour(&region, &tool, Side::Outside, &cut))
        }
        OpKind::Finish3d => {
            let Some(mesh) = mesh else {
                return Err("there is no solid to finish".into());
            };
            if s.tool != ToolChoice::Ball {
                return Err("3-D finishing here uses a ball-nose tool".into());
            }
            let p = RasterParams {
                region: [s.x0, s.y0, s.x1, s.y1],
                stepover: s.stepover.max(0.2),
                step: 1.0,
                tolerance: 0.01,
                min_segment: 0.05,
                floor: s.bottom.min(s.top) - 1.0,
                safe_z: s.top + 10.0,
                feed: s.feed,
                plunge_feed: (s.feed * 0.4).max(50.0),
            };
            Ok(raster_finish(mesh, &tool, &p))
        }
    }
}

/// Cutting moves of a toolpath (arcs chorded) for display.
pub fn cut_segments(tp: &Toolpath) -> Vec<([f64; 3], [f64; 3])> {
    let mut out = Vec::new();
    let mut pos: Option<[f64; 3]> = None;
    for e in &tp.events {
        match e {
            Event::Rapid { p } => pos = Some(*p),
            Event::Feed { p, .. } => {
                if let Some(q) = pos {
                    out.push((q, *p));
                }
                pos = Some(*p);
            }
            Event::Arc { p, .. } => {
                if let Some(q) = pos {
                    let mut mini = Toolpath::new("arc");
                    mini.push(Event::Rapid { p: q });
                    mini.push(e.clone());
                    out.extend(moves(&mini));
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    out
}

fn mesh_of(brep: &kw_brep::BRep) -> Option<TriMesh> {
    let mut tris = Vec::new();
    for body in brep.bodies.keys() {
        let t = kw_tess::tessellate_body(
            brep,
            body,
            &kw_tess::TessOptions {
                chord_tol: 0.01,
                angle_tol: 0.15,
            },
        )
        .ok()?;
        for c in t.mesh.indices.as_chunks::<3>().0 {
            let p = |i: u32| {
                let q = t.mesh.positions[i as usize];
                [q.x, q.y, q.z]
            };
            tris.push([p(c[0]), p(c[1]), p(c[2])]);
        }
    }
    (!tris.is_empty()).then(|| TriMesh::new(tris))
}

pub struct CamWorkspace {
    pub viewport: Viewport3d,
    pub settings: Settings,
    pub dialect: Dialect,
    pub selected: Option<usize>,
    pub message: String,
    job: Option<Job<Toolpath>>,
    sim_job: Option<Job<String>>,
    /// Increases whenever the displayed toolpath changes (the viewport rebuilds its scene).
    shown: u64,
    initialised_for: Option<u128>,
    /// Ribbon buttons ask for things here (`generate`, `simulate`); `show` carries them out.
    pub requests: Vec<&'static str>,
}

impl Default for CamWorkspace {
    fn default() -> Self {
        CamWorkspace {
            viewport: Viewport3d::default(),
            settings: Settings::default(),
            dialect: Dialect::Fanuc,
            selected: None,
            message: String::new(),
            job: None,
            sim_job: None,
            shown: 0,
            initialised_for: None,
            requests: Vec::new(),
        }
    }
}

impl CamWorkspace {
    /// The ribbon of the Manufacture workspace.
    pub fn ribbon(&mut self, ui: &mut egui::Ui) {
        use crate::chrome::{ribbon as rb, Icon};
        let mut cur = 0usize;
        rb::tabs(
            ui,
            &[
                ("MILLING", true),
                ("TURNING", false),
                ("ADDITIVE", false),
                ("INSPECTION", false),
            ],
            &mut cur,
        );
        ui.add_space(2.0);
        let op = self.settings.op;
        let mut req: Vec<&'static str> = Vec::new();
        let mut set: Option<OpKind> = None;
        ui.horizontal(|ui| {
            rb::group(ui, "2D", |ui| {
                for (k, icon, label) in [
                    (OpKind::Pocket, Icon::Rectangle, "Pocket"),
                    (OpKind::Adaptive, Icon::Offset, "Adaptive Clearing"),
                    (OpKind::ContourOutside, Icon::Spline, "2D Contour"),
                ] {
                    if rb::button(
                        ui,
                        icon,
                        label,
                        true,
                        op == k,
                        "Choose the operation and generate its toolpath",
                    )
                    .clicked()
                    {
                        set = Some(k);
                        req.push("generate");
                    }
                }
            });
            rb::group(ui, "3D", |ui| {
                if rb::button(
                    ui,
                    Icon::Cam,
                    "3D Finish",
                    true,
                    op == OpKind::Finish3d,
                    "Ball-nose raster finishing of the model",
                )
                .clicked()
                {
                    set = Some(OpKind::Finish3d);
                    req.push("generate");
                }
            });
            rb::group(ui, "ACTIONS", |ui| {
                if rb::button(
                    ui,
                    Icon::Play,
                    "Simulate",
                    self.selected.is_some(),
                    false,
                    "Remove stock with the selected operation",
                )
                .clicked()
                {
                    req.push("simulate");
                }
            });
        });
        if let Some(k) = set {
            self.settings.op = k;
        }
        self.requests.extend(req);
    }

    /// Take the region and heights from the model's bounding box.
    pub fn fit_to_model(&mut self, brep: &kw_brep::BRep) {
        if let Some(body) = brep.bodies.keys().next() {
            let bb = brep.body_bbox(body);
            let s = &mut self.settings;
            s.x0 = bb.min.x;
            s.y0 = bb.min.y;
            s.x1 = bb.max.x;
            s.y1 = bb.max.y;
            s.top = bb.max.z;
            s.bottom = (bb.max.z - 3.0).max(bb.min.z);
        }
    }

    /// Generate and record an operation now, on this thread (tests and demos).
    pub fn run_now(
        &mut self,
        project: &mut Project,
        brep: Option<&Arc<kw_brep::BRep>>,
    ) -> Result<(), String> {
        let mesh = if self.settings.op == OpKind::Finish3d {
            brep.and_then(|b| mesh_of(b))
        } else {
            None
        };
        let tp = generate(&self.settings, mesh.as_ref())?;
        self.finished(project, brep, tp);
        Ok(())
    }

    pub fn start(&mut self, brep: Option<&Arc<kw_brep::BRep>>) {
        let s = self.settings.clone();
        let mesh = if s.op == OpKind::Finish3d {
            brep.and_then(|b| mesh_of(b))
        } else {
            None
        };
        self.job = Some(Job::spawn(s.op.label(), move || {
            generate(&s, mesh.as_ref())
        }));
        self.message = format!("generating {}…", self.settings.op.label());
    }

    fn setup<'a>(project: &'a mut Project, brep: Option<&Arc<kw_brep::BRep>>) -> &'a mut CamSetup {
        if project.cam.is_none() {
            let (min, max) = brep
                .and_then(|b| b.bodies.keys().next().map(|k| b.body_bbox(k)))
                .map(|bb| {
                    (
                        [bb.min.x, bb.min.y, bb.min.z],
                        [bb.max.x, bb.max.y, bb.max.z],
                    )
                })
                .unwrap_or(([0.0; 3], [100.0, 100.0, 20.0]));
            project.cam = Some(CamSetup {
                machine: "3-axis mill".into(),
                stock_min: min,
                stock_max: max,
                tools: Vec::new(),
                operations: Vec::new(),
            });
        }
        project.cam.as_mut().expect("cam setup")
    }

    fn finished(&mut self, project: &mut Project, brep: Option<&Arc<kw_brep::BRep>>, tp: Toolpath) {
        let tool = self.settings.tool();
        let len = tp.cutting_length();
        let time = len / self.settings.feed.max(1.0) * 60.0;
        self.message = format!(
            "{}: {} events, {:.0} mm of cutting, about {:.0} s at {:.0} mm/min",
            self.settings.op.label(),
            tp.events.len(),
            len,
            time,
            self.settings.feed
        );
        let setup = Self::setup(project, brep);
        if !setup.tools.iter().any(|t| t.name == tool.name) {
            setup.tools.push(tool.clone());
        }
        let name = format!(
            "{} {}",
            self.settings.op.label(),
            setup.operations.len() + 1
        );
        setup.operations.push(CamOperation {
            name,
            tool: tool.number,
            path: tp,
        });
        self.selected = Some(setup.operations.len() - 1);
        project.mark_extras_dirty();
        self.shown += 1;
    }

    pub fn post_text(&self, project: &Project) -> Option<Result<String, String>> {
        let op = project.cam.as_ref()?.operations.get(self.selected?)?;
        Some(render(&self.dialect.config(), &op.path).map_err(|e| e.to_string()))
    }

    /// Remove the stock with the selected operation and report how much material it took.
    pub fn start_sim(&mut self, project: &Project, brep: Option<&Arc<kw_brep::BRep>>) {
        let (Some(cam), Some(sel)) = (project.cam.as_ref(), self.selected) else {
            return;
        };
        let Some(op) = cam.operations.get(sel) else {
            return;
        };
        let path = op.path.clone();
        let tool = self.settings.tool();
        let (min, max) = (cam.stock_min, cam.stock_max);
        let _ = brep;
        self.sim_job = Some(Job::spawn("stock simulation", move || {
            let pad = tool.diameter;
            let ext = [
                max[0] - min[0] + 2.0 * pad,
                max[1] - min[1] + 2.0 * pad,
                (max[2] - min[2]).max(1.0),
            ];
            let vol = ext[0] * ext[1] * ext[2];
            let h = (vol / 2.0e6).cbrt().max(0.25);
            let n = [
                (ext[0] / h).ceil(),
                (ext[1] / h).ceil(),
                (ext[2] / h).ceil(),
            ];
            let lo = [min[0] - pad, min[1] - pad, min[2]];
            let hi = [lo[0] + n[0] * h, lo[1] + n[1] * h, lo[2] + n[2] * h];
            let mut d = TriDexel::block(lo, hi, h);
            let before = d.initial_volume();
            d.run(&tool, &path);
            let after = d.volume()[0];
            Ok(format!("stock {:.0} mm³ → {:.0} mm³: {:.0} mm³ removed ({:.1} %), cell {:.2} mm; rapids through material: {:.1} mm³", before, after, before - after, 100.0 * (before - after) / before, h, d.rapid_removed))
        }));
        self.message = "simulating stock removal…".into();
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
        dialogs: &mut dyn Dialogs,
    ) {
        let ev = project.evaluation();
        let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
        let model_key = ev
            .final_hashes()
            .map(|h| h.0 ^ h.1.rotate_left(1))
            .unwrap_or(0);
        if self.initialised_for != Some(model_key) {
            if let Some(b) = &brep {
                self.fit_to_model(b);
            }
            self.initialised_for = Some(model_key);
        }
        // finished jobs
        if let Some(job) = &self.job {
            match job.poll() {
                Some(Ok(tp)) => {
                    self.job = None;
                    self.finished(project, brep.as_ref(), tp);
                }
                Some(Err(e)) => {
                    self.message = e;
                    self.job = None;
                }
                None => ui.ctx().request_repaint(),
            }
        }
        if let Some(job) = &self.sim_job {
            match job.poll() {
                Some(Ok(m)) => {
                    self.message = m;
                    self.sim_job = None;
                }
                Some(Err(e)) => {
                    self.message = e;
                    self.sim_job = None;
                }
                None => ui.ctx().request_repaint(),
            }
        }
        for r in std::mem::take(&mut self.requests) {
            match r {
                "generate" => {
                    if self.job.is_none() {
                        self.start(brep.as_ref());
                    }
                }
                "simulate" => self.start_sim(project, brep.as_ref()),
                _ => {}
            }
        }
        egui::Panel::left("cam_side")
            .default_size(320.0)
            .show(ui, |ui| self.side(ui, project, brep.as_ref(), dialogs));
        egui::CentralPanel::default().show(ui, |ui| {
            let segs: Vec<([f64; 3], [f64; 3])> = self
                .selected
                .and_then(|i| project.cam.as_ref()?.operations.get(i))
                .map(|o| cut_segments(&o.path))
                .unwrap_or_default();
            let key = model_key
                ^ ((self.shown as u128) << 64)
                ^ (self.selected.map(|s| s as u128 + 1).unwrap_or(0) << 100)
                ^ segs.len() as u128;
            let b = brep.clone();
            self.viewport.wireframe = true;
            self.viewport.show_with(
                ui,
                rs,
                key,
                move || {
                    let mut m = match &b {
                        Some(b) => ModelScene::build(b),
                        None => ModelScene::empty(),
                    };
                    m.add_segments(&segs);
                    m
                },
                None,
                crate::viewport::ViewOpts::default(),
            );
        });
    }

    fn side(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        brep: Option<&Arc<kw_brep::BRep>>,
        dialogs: &mut dyn Dialogs,
    ) {
        let busy = self.job.is_some() || self.sim_job.is_some();
        ui.heading("Operation");
        egui::ComboBox::from_id_salt("cam_op")
            .selected_text(self.settings.op.label())
            .show_ui(ui, |ui| {
                for k in OpKind::ALL {
                    ui.selectable_value(&mut self.settings.op, k, k.label());
                }
            });
        if self.settings.op == OpKind::Finish3d {
            self.settings.tool = ToolChoice::Ball;
        }
        ui.horizontal(|ui| {
            ui.label("Tool");
            ui.selectable_value(&mut self.settings.tool, ToolChoice::Flat, "Flat");
            ui.selectable_value(&mut self.settings.tool, ToolChoice::Ball, "Ball");
            ui.selectable_value(&mut self.settings.tool, ToolChoice::Bull, "Bull");
        });
        let s = &mut self.settings;
        let drag = |ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64| {
            ui.horizontal(|ui| {
                ui.label(label);
                ui.add(egui::DragValue::new(v).speed(speed));
            });
        };
        drag(ui, "Diameter (mm)", &mut s.diameter, 0.1);
        if s.tool == ToolChoice::Bull {
            drag(ui, "Corner radius", &mut s.corner, 0.05);
        }
        ui.separator();
        ui.strong("Region (XY, mm)");
        ui.horizontal(|ui| {
            ui.add(egui::DragValue::new(&mut s.x0).speed(0.5).prefix("x0 "));
            ui.add(egui::DragValue::new(&mut s.x1).speed(0.5).prefix("x1 "));
        });
        ui.horizontal(|ui| {
            ui.add(egui::DragValue::new(&mut s.y0).speed(0.5).prefix("y0 "));
            ui.add(egui::DragValue::new(&mut s.y1).speed(0.5).prefix("y1 "));
        });
        if ui.button("Use the model's footprint").clicked() {
            if let Some(b) = brep {
                self.fit_to_model(b);
            }
        }
        let s = &mut self.settings;
        drag(ui, "Top z", &mut s.top, 0.5);
        drag(ui, "Bottom z", &mut s.bottom, 0.5);
        drag(ui, "Step down", &mut s.step_down, 0.1);
        drag(ui, "Stepover", &mut s.stepover, 0.1);
        drag(ui, "Feed (mm/min)", &mut s.feed, 10.0);
        drag(ui, "Spindle (rpm)", &mut s.rpm, 100.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!busy, egui::Button::new("Generate toolpath"))
                .clicked()
            {
                self.start(brep);
            }
            if busy {
                ui.spinner();
            }
        });
        ui.separator();
        ui.heading("Operations");
        let ops: Vec<String> = project
            .cam
            .as_ref()
            .map(|c| c.operations.iter().map(|o| o.name.clone()).collect())
            .unwrap_or_default();
        if ops.is_empty() {
            ui.weak("None yet.");
        }
        let mut remove = None;
        for (i, name) in ops.iter().enumerate() {
            ui.horizontal(|ui| {
                if ui
                    .selectable_label(self.selected == Some(i), name)
                    .clicked()
                {
                    self.selected = Some(i);
                    self.shown += 1;
                }
                if ui.small_button("Delete").clicked() {
                    remove = Some(i);
                }
            });
        }
        if let Some(i) = remove {
            if let Some(c) = project.cam.as_mut() {
                c.operations.remove(i);
            }
            project.mark_extras_dirty();
            self.selected = None;
            self.shown += 1;
        }
        if self.selected.is_some() {
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!busy, egui::Button::new("Simulate stock"))
                    .clicked()
                {
                    self.start_sim(project, brep);
                }
            });
            ui.separator();
            ui.heading("Post");
            egui::ComboBox::from_id_salt("cam_dialect")
                .selected_text(self.dialect.label())
                .show_ui(ui, |ui| {
                    for d in Dialect::ALL {
                        ui.selectable_value(&mut self.dialect, d, d.label());
                    }
                });
            match self.post_text(project) {
                Some(Ok(text)) => {
                    ui.label(format!("{} lines", text.lines().count()));
                    if ui.button("Save G-code…").clicked() {
                        let name = format!("{}.{}", project.name(), self.dialect.extension());
                        if let Some(p) =
                            dialogs.save_file(&name, "G-code", &[self.dialect.extension()])
                        {
                            self.message = match std::fs::write(&p, &text) {
                                Ok(()) => format!("wrote {}", p.display()),
                                Err(e) => format!("could not write: {e}"),
                            };
                        }
                    }
                    egui::ScrollArea::vertical()
                        .max_height(180.0)
                        .show(ui, |ui| {
                            ui.add(
                                egui::Label::new(
                                    egui::RichText::new(
                                        text.lines().take(40).collect::<Vec<_>>().join("\n"),
                                    )
                                    .monospace()
                                    .size(10.0),
                                )
                                .wrap_mode(egui::TextWrapMode::Extend),
                            );
                        });
                }
                Some(Err(e)) => {
                    ui.colored_label(egui::Color32::from_rgb(230, 110, 90), e);
                }
                None => {}
            }
        }
        if !self.message.is_empty() {
            ui.separator();
            ui.label(&self.message);
        }
    }
}
