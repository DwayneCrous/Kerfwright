//! The application: a welcome page and, once a project is open, the project view with its workspaces.

use crate::cam::CamWorkspace;
use crate::dialogs::{Dialogs, NativeDialogs};
use crate::ecad::EcadWorkspace;
use crate::fea::FeaWorkspace;
use crate::model::ModelWorkspace;
use crate::project::Project;
use crate::recent::RecentList;
use crate::render::RenderWorkspace;
use crate::sketch::SketchWorkspace;
use eframe::egui;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Workspace {
    Sketch,
    Model,
    Cam,
    Fea,
    Ecad,
    Render,
}

impl Workspace {
    pub const ALL: [Workspace; 6] = [
        Workspace::Sketch,
        Workspace::Model,
        Workspace::Cam,
        Workspace::Fea,
        Workspace::Ecad,
        Workspace::Render,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Workspace::Sketch => "Sketch",
            Workspace::Model => "Model",
            Workspace::Cam => "CAM",
            Workspace::Fea => "FEA",
            Workspace::Ecad => "ECAD",
            Workspace::Render => "Render",
        }
    }

    pub fn blurb(self) -> &'static str {
        match self {
            Workspace::Sketch => "Constrained 2-D sketches that become features.",
            Workspace::Model => "The feature timeline and the 3-D model.",
            Workspace::Cam => "Toolpaths, stock simulation and G-code.",
            Workspace::Fea => "Meshing and static stress analysis.",
            Workspace::Ecad => "Circuit board placement, routing and fabrication outputs.",
            Workspace::Render => "Realtime and path-traced images.",
        }
    }
}

/// What the user asked for that has to wait for a decision about unsaved changes.
#[derive(Debug, Clone, PartialEq)]
pub enum Pending {
    CloseProject,
    Open(PathBuf),
    OpenDemo,
    Quit,
}

pub enum Screen {
    Welcome,
    Project {
        project: Box<Project>,
        workspace: Workspace,
    },
}

pub struct KerfwrightApp {
    pub screen: Screen,
    pub recents: RecentList,
    recents_file: Option<PathBuf>,
    dialogs: Box<dyn Dialogs>,
    pub pending: Option<Pending>,
    pub model: ModelWorkspace,
    pub sketch: SketchWorkspace,
    pub ecad: EcadWorkspace,
    pub cam: CamWorkspace,
    pub fea: FeaWorkspace,
    pub render: RenderWorkspace,
    pub message: String,
    pub new_name: String,
    pub new_folder: Option<PathBuf>,
    pub frames: u64,
    pub exit_after_frames: Option<u64>,
    /// Set when the user confirmed quitting; the window may close.
    pub quit_ok: bool,
    /// Save a screenshot of the window to this file after `screenshot_after` frames, then quit.
    pub screenshot: Option<(PathBuf, u64)>,
}

impl Default for KerfwrightApp {
    fn default() -> Self {
        let recents_file = RecentList::default_file();
        let recents = recents_file
            .as_deref()
            .map(RecentList::load)
            .unwrap_or_default();
        KerfwrightApp::with(recents, recents_file, Box::new(NativeDialogs))
    }
}

impl KerfwrightApp {
    pub fn with(
        recents: RecentList,
        recents_file: Option<PathBuf>,
        dialogs: Box<dyn Dialogs>,
    ) -> Self {
        KerfwrightApp {
            screen: Screen::Welcome,
            recents,
            recents_file,
            dialogs,
            pending: None,
            model: ModelWorkspace::default(),
            sketch: SketchWorkspace::default(),
            ecad: EcadWorkspace::default(),
            cam: CamWorkspace::default(),
            fea: FeaWorkspace::default(),
            render: RenderWorkspace::default(),
            message: String::new(),
            new_name: "Untitled".into(),
            new_folder: std::env::var_os("HOME").map(PathBuf::from),
            frames: 0,
            exit_after_frames: None,
            quit_ok: false,
            screenshot: None,
        }
    }

    pub fn project(&self) -> Option<&Project> {
        match &self.screen {
            Screen::Project { project, .. } => Some(project),
            Screen::Welcome => None,
        }
    }

    pub fn project_mut(&mut self) -> Option<&mut Project> {
        match &mut self.screen {
            Screen::Project { project, .. } => Some(project),
            Screen::Welcome => None,
        }
    }

    pub fn workspace(&self) -> Option<Workspace> {
        match &self.screen {
            Screen::Project { workspace, .. } => Some(*workspace),
            Screen::Welcome => None,
        }
    }

    /// Open the Sketch workspace on an existing sketch feature.
    pub fn edit_sketch(&mut self, id: kw_feature::FeatureId) -> Result<(), String> {
        let Screen::Project { project, .. } = &mut self.screen else {
            return Err("no project is open".into());
        };
        self.sketch.begin_edit(project, id)?;
        self.set_workspace(Workspace::Sketch);
        Ok(())
    }

    /// A project with a generated board, routed, in the ECAD workspace (for demos and screenshots).
    pub fn ecad_demo(&mut self, index: usize) {
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            self.ecad.generate_sample(project, index);
            if let Some(b) = project.board.take() {
                let mut r = kw_ecad::route::Router::new(b, Default::default());
                r.route_all();
                project.board = Some(r.board);
            }
            self.ecad.check(project);
        }
        self.set_workspace(Workspace::Ecad);
    }

    /// The demo part with a pocket planned in the CAM workspace (for demos and screenshots).
    pub fn cam_demo(&mut self) {
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            let ev = project.evaluation();
            let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
            if let Some(b) = &brep {
                self.cam.fit_to_model(b);
            }
            self.cam.settings.x0 += 4.0;
            self.cam.settings.y0 += 4.0;
            self.cam.settings.x1 -= 4.0;
            self.cam.settings.y1 -= 4.0;
            self.cam.settings.bottom = self.cam.settings.top - 4.0;
            let _ = self.cam.run_now(project, brep.as_ref());
        }
        self.set_workspace(Workspace::Cam);
    }

    /// The demo block clamped at one end and loaded at the other, solved, in the FEA workspace (demos and
    /// screenshots).
    pub fn fea_demo(&mut self) {
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            let ev = project.evaluation();
            if let Some(brep) = ev.final_solid().and_then(|(_, o)| o.brep().cloned()) {
                let Some(body) = brep.bodies.keys().next() else {
                    return;
                };
                let bb = brep.body_bbox(body);
                let Ok(t) = kw_tess::tessellate_body(
                    &brep,
                    body,
                    &kw_tess::TessOptions {
                        chord_tol: 0.005,
                        angle_tol: 12f64.to_radians(),
                    },
                ) else {
                    return;
                };
                let face_at = |x: f64| {
                    t.mesh
                        .indices
                        .as_chunks::<3>()
                        .0
                        .iter()
                        .enumerate()
                        .find(|(_, tri)| {
                            tri.iter()
                                .all(|i| (t.mesh.positions[*i as usize].x - x).abs() < 1e-6)
                        })
                        .map(|(k, _)| t.mesh.tri_face[k])
                };
                if let (Some(a), Some(b)) = (face_at(bb.min.x + 0.0), face_at(bb.max.x)) {
                    self.fea.study.max_size = 3.0;
                    self.fea.fix_face(project, a);
                    self.fea.add_force(project, b, [0.0, 0.0, -2000.0]);
                    match crate::fea::solve_study(&brep, &self.fea.study) {
                        Ok(r) => {
                            self.fea.scale = (0.05 * bb.diagonal() / r.max_displacement.max(1e-30))
                                .round()
                                .max(1.0);
                            self.fea.result = Some(r);
                            self.fea.field = crate::fea::Field::VonMises;
                        }
                        Err(e) => self.message = e,
                    }
                }
            }
        }
        self.set_workspace(Workspace::Fea);
    }

    pub fn set_workspace(&mut self, w: Workspace) {
        if let Screen::Project { workspace, .. } = &mut self.screen {
            *workspace = w;
        }
    }

    fn remember(&mut self, path: &Path) {
        self.recents.touch(path.to_path_buf());
        if let Some(f) = &self.recents_file {
            if let Err(e) = self.recents.save(f) {
                self.message = format!("could not update the recent list: {e}");
            }
        }
    }

    fn enter(&mut self, project: Project) {
        self.model = ModelWorkspace::default();
        self.sketch = SketchWorkspace::default();
        self.ecad = EcadWorkspace::default();
        self.cam = CamWorkspace::default();
        self.fea = FeaWorkspace::default();
        self.render = RenderWorkspace::default();
        self.screen = Screen::Project {
            project: Box::new(project),
            workspace: Workspace::Model,
        };
    }

    // ----------------------------------------------------------------------------- project actions

    /// A new project `folder/name.kwf`, saved at once so that it exists on disk and in the recent list.
    pub fn create_project(&mut self, name: &str, folder: &Path) -> Result<(), String> {
        let name = name.trim();
        if name.is_empty() || name.contains(['/', '\\']) {
            return Err("give the project a name without slashes".into());
        }
        if !folder.is_dir() {
            return Err(format!("{} is not a folder", folder.display()));
        }
        let path = folder.join(format!("{name}.kwf"));
        if path.exists() {
            return Err(format!("{} already exists", path.display()));
        }
        let mut p = Project::empty();
        p.save_as(&path)?;
        self.remember(&path);
        self.enter(p);
        self.message.clear();
        Ok(())
    }

    pub fn open_path(&mut self, path: &Path) -> Result<(), String> {
        match Project::open(path) {
            Ok(p) => {
                self.remember(path);
                self.enter(p);
                self.message.clear();
                Ok(())
            }
            Err(e) => {
                self.message = format!("cannot open {}: {e}", path.display());
                Err(e)
            }
        }
    }

    pub fn open_demo(&mut self) {
        self.enter(Project::demo());
        self.message.clear();
    }

    fn dirty(&self) -> bool {
        self.project().is_some_and(|p| p.is_dirty())
    }

    /// Run `what` now, or ask about unsaved changes first.
    pub fn request(&mut self, what: Pending) {
        if self.dirty() && !matches!(what, Pending::OpenDemo if self.project().is_none()) {
            self.pending = Some(what);
        } else {
            self.perform(what);
        }
    }

    fn perform(&mut self, what: Pending) {
        self.pending = None;
        match what {
            Pending::CloseProject => self.screen = Screen::Welcome,
            Pending::Open(p) => {
                let _ = self.open_path(&p);
            }
            Pending::OpenDemo => self.open_demo(),
            Pending::Quit => self.quit_ok = true,
        }
    }

    /// Save, asking for a file name when the project has none. Returns whether the project was saved.
    pub fn save(&mut self) -> bool {
        let Some(p) = self.project_mut() else {
            return false;
        };
        if p.path.is_none() {
            return self.save_as();
        }
        match p.save() {
            Ok(()) => {
                self.message = "saved".into();
                true
            }
            Err(e) => {
                self.message = format!("save failed: {e}");
                false
            }
        }
    }

    pub fn save_as(&mut self) -> bool {
        let suggested = format!(
            "{}.kwf",
            self.project().map(|p| p.name()).unwrap_or_default()
        );
        let Some(path) = self.dialogs.save_project(&suggested) else {
            return false;
        };
        let path = if path.extension().is_none() {
            path.with_extension("kwf")
        } else {
            path
        };
        let Some(p) = self.project_mut() else {
            return false;
        };
        match p.save_as(&path) {
            Ok(()) => {
                self.remember(&path);
                self.message = "saved".into();
                true
            }
            Err(e) => {
                self.message = format!("save failed: {e}");
                false
            }
        }
    }

    /// The answer to the unsaved-changes question.
    pub fn resolve_pending(&mut self, save_first: bool) {
        let Some(what) = self.pending.clone() else {
            return;
        };
        if save_first && !self.save() {
            return; // cancelled or failed: stay where we are
        }
        self.perform(what);
    }

    pub fn cancel_pending(&mut self) {
        self.pending = None;
    }

    /// The window asked to close. Returns true if it may.
    pub fn close_requested(&mut self) -> bool {
        if self.quit_ok {
            return true;
        }
        self.request(Pending::Quit);
        self.quit_ok
    }

    // --------------------------------------------------------------------------------------- drawing

    pub fn show(&mut self, ui: &mut egui::Ui) {
        self.show_with(ui, None);
    }

    /// Draw a frame; `rs` is the GPU state the 3-D views draw with (`None` in headless tests).
    pub fn show_with(&mut self, ui: &mut egui::Ui, rs: Option<&eframe::egui_wgpu::RenderState>) {
        self.frames += 1;
        match &self.screen {
            Screen::Welcome => self.welcome_ui(ui),
            Screen::Project { .. } => self.project_ui(ui, rs),
        }
        self.pending_ui(ui.ctx());
        self.screenshot_step(ui.ctx());
        if let Some(n) = self.exit_after_frames {
            if self.frames >= n {
                self.quit_ok = true;
                ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
            } else {
                ui.ctx().request_repaint();
            }
        }
    }

    fn screenshot_step(&mut self, ctx: &egui::Context) {
        let Some((path, after)) = self.screenshot.clone() else {
            return;
        };
        if self.frames == after {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
        }
        let shot = ctx.input(|i| {
            i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            let [w, h] = img.size;
            let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
            let result = std::fs::File::create(&path)
                .map_err(|e| e.to_string())
                .and_then(|f| {
                    let mut enc = png::Encoder::new(std::io::BufWriter::new(f), w as u32, h as u32);
                    enc.set_color(png::ColorType::Rgba);
                    enc.set_depth(png::BitDepth::Eight);
                    enc.write_header()
                        .map_err(|e| e.to_string())?
                        .write_image_data(&bytes)
                        .map_err(|e| e.to_string())
                });
            match result {
                Ok(()) => eprintln!("screenshot written to {}", path.display()),
                Err(e) => eprintln!("screenshot failed: {e}"),
            }
            self.screenshot = None;
            self.quit_ok = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        } else {
            ctx.request_repaint();
        }
    }

    fn welcome_ui(&mut self, ui: &mut egui::Ui) {
        egui::CentralPanel::default().show(ui, |ui| {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                ui.heading(egui::RichText::new("Kerfwright").size(40.0));
                ui.label("Sketch, model, machine, analyse and design boards in one place.");
            });
            ui.add_space(24.0);
            ui.columns(2, |cols| {
                // ---- left: start something
                cols[0].group(|ui| {
                    ui.heading("Start");
                    ui.add_space(6.0);
                    ui.label("New project");
                    ui.horizontal(|ui| {
                        ui.label("Name");
                        ui.text_edit_singleline(&mut self.new_name);
                    });
                    ui.horizontal(|ui| {
                        ui.label("Folder");
                        let shown = self
                            .new_folder
                            .as_ref()
                            .map(|p| p.display().to_string())
                            .unwrap_or_else(|| "(none chosen)".into());
                        ui.monospace(shown);
                        if ui.button("Browse…").clicked() {
                            if let Some(f) = self.dialogs.choose_folder() {
                                self.new_folder = Some(f);
                            }
                        }
                    });
                    if ui.button("Create project").clicked() {
                        let name = self.new_name.clone();
                        match self.new_folder.clone() {
                            Some(f) => {
                                if let Err(e) = self.create_project(&name, &f) {
                                    self.message = e;
                                }
                            }
                            None => self.message = "choose a folder first".into(),
                        }
                    }
                    ui.separator();
                    if ui.button("Open project…").clicked() {
                        if let Some(p) = self.dialogs.open_project() {
                            let _ = self.open_path(&p);
                        }
                    }
                    if ui.button("Open the demo part").clicked() {
                        self.open_demo();
                    }
                });
                // ---- right: recent projects
                cols[1].group(|ui| {
                    ui.heading("Recent projects");
                    ui.add_space(6.0);
                    if self.recents.paths.is_empty() {
                        ui.weak("Nothing yet — create or open a project.");
                    }
                    let mut open: Option<PathBuf> = None;
                    let mut forget: Option<PathBuf> = None;
                    for p in &self.recents.paths {
                        let exists = p.exists();
                        ui.horizontal(|ui| {
                            let name = p
                                .file_stem()
                                .map(|s| s.to_string_lossy().into_owned())
                                .unwrap_or_default();
                            let text = egui::RichText::new(name);
                            if exists {
                                if ui
                                    .link(text)
                                    .on_hover_text(p.display().to_string())
                                    .clicked()
                                {
                                    open = Some(p.clone());
                                }
                            } else {
                                ui.weak(text.strikethrough());
                                ui.weak("(missing)");
                                if ui.small_button("forget").clicked() {
                                    forget = Some(p.clone());
                                }
                            }
                            ui.weak(
                                p.parent()
                                    .map(|d| d.display().to_string())
                                    .unwrap_or_default(),
                            );
                        });
                    }
                    if let Some(p) = forget {
                        self.recents.remove(&p);
                        if let Some(f) = &self.recents_file {
                            let _ = self.recents.save(f);
                        }
                    }
                    if let Some(p) = open {
                        let _ = self.open_path(&p);
                    }
                });
            });
            ui.add_space(12.0);
            if !self.message.is_empty() {
                ui.colored_label(egui::Color32::from_rgb(220, 120, 80), &self.message);
            }
        });
    }

    fn project_ui(&mut self, ui: &mut egui::Ui, rs: Option<&eframe::egui_wgpu::RenderState>) {
        // ---- top bar
        egui::Panel::top("app_top").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Save   Ctrl+S").clicked() {
                        self.save();
                        ui.close();
                    }
                    if ui.button("Save as…").clicked() {
                        self.save_as();
                        ui.close();
                    }
                    ui.separator();
                    if ui.button("Open project…").clicked() {
                        if let Some(p) = self.dialogs.open_project() {
                            self.request(Pending::Open(p));
                        }
                        ui.close();
                    }
                    if ui.button("Close project").clicked() {
                        self.request(Pending::CloseProject);
                        ui.close();
                    }
                });
                ui.menu_button("Edit", |ui| {
                    let (can_undo, can_redo) = self
                        .project()
                        .map(|p| (p.history.can_undo(), p.history.can_redo()))
                        .unwrap_or((false, false));
                    if ui
                        .add_enabled(can_undo, egui::Button::new("Undo   Ctrl+Z"))
                        .clicked()
                    {
                        if let Some(p) = self.project_mut() {
                            p.history.undo();
                        }
                        ui.close();
                    }
                    if ui
                        .add_enabled(can_redo, egui::Button::new("Redo   Ctrl+Y"))
                        .clicked()
                    {
                        if let Some(p) = self.project_mut() {
                            p.history.redo();
                        }
                        ui.close();
                    }
                });
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(p) = self.project() {
                        let dirty = if p.is_dirty() { " •" } else { "" };
                        ui.strong(format!("{}{}", p.name(), dirty));
                    }
                });
            });
        });
        // keyboard shortcuts
        let (save, undo, redo) = ui.ctx().input(|i| {
            let ctrl = i.modifiers.command;
            (
                ctrl && i.key_pressed(egui::Key::S),
                ctrl && i.key_pressed(egui::Key::Z) && !i.modifiers.shift,
                ctrl && (i.key_pressed(egui::Key::Y)
                    || (i.key_pressed(egui::Key::Z) && i.modifiers.shift)),
            )
        });
        if save {
            self.save();
        }
        if (undo || redo) && self.workspace() != Some(Workspace::Sketch) {
            if let Some(p) = self.project_mut() {
                if undo {
                    p.history.undo();
                } else {
                    p.history.redo();
                }
            }
        }
        // ---- workspace rail
        egui::Panel::left("app_rail")
            .exact_size(92.0)
            .show(ui, |ui| {
                ui.add_space(6.0);
                let current = self.workspace();
                for w in Workspace::ALL {
                    let sel = current == Some(w);
                    let b = egui::Button::new(egui::RichText::new(w.label()).size(15.0))
                        .selected(sel)
                        .min_size(egui::vec2(80.0, 40.0));
                    if ui.add(b).on_hover_text(w.blurb()).clicked() {
                        self.set_workspace(w);
                    }
                    ui.add_space(2.0);
                }
            });
        // ---- status
        egui::Panel::bottom("app_status").show(ui, |ui| {
            ui.horizontal(|ui| {
                let path = self
                    .project()
                    .and_then(|p| p.path.as_ref())
                    .map(|p| p.display().to_string())
                    .unwrap_or_else(|| "not saved yet".into());
                ui.weak(path);
                if !self.message.is_empty() {
                    ui.separator();
                    ui.label(&self.message);
                }
            });
        });
        // ---- the workspace
        let w = self.workspace().unwrap_or(Workspace::Model);
        match w {
            Workspace::Model => {
                let Screen::Project { project, .. } = &mut self.screen else {
                    return;
                };
                egui::CentralPanel::default().show(ui, |ui| self.model.show(ui, project, rs));
            }
            Workspace::Sketch => {
                let Screen::Project { project, .. } = &mut self.screen else {
                    return;
                };
                let mut done = None;
                egui::CentralPanel::default().show(ui, |ui| done = self.sketch.show(ui, project));
                if let Some(id) = done {
                    self.model.selected = Some(id);
                    self.set_workspace(Workspace::Model);
                }
            }
            Workspace::Cam => {
                let Screen::Project { project, .. } = &mut self.screen else {
                    return;
                };
                let (cam, dialogs) = (&mut self.cam, &mut self.dialogs);
                egui::CentralPanel::default()
                    .show(ui, |ui| cam.show(ui, project, rs, dialogs.as_mut()));
            }
            Workspace::Render => {
                let Screen::Project { project, .. } = &mut self.screen else {
                    return;
                };
                let (render, dialogs) = (&mut self.render, &mut self.dialogs);
                egui::CentralPanel::default()
                    .show(ui, |ui| render.show(ui, project, dialogs.as_mut()));
            }
            Workspace::Fea => {
                let Screen::Project { project, .. } = &mut self.screen else {
                    return;
                };
                egui::CentralPanel::default().show(ui, |ui| self.fea.show(ui, project, rs));
            }
            Workspace::Ecad => {
                let Screen::Project { project, .. } = &mut self.screen else {
                    return;
                };
                let (ecad, dialogs) = (&mut self.ecad, &mut self.dialogs);
                egui::CentralPanel::default()
                    .show(ui, |ui| ecad.show(ui, project, dialogs.as_mut()));
            }
        }
    }

    fn pending_ui(&mut self, ctx: &egui::Context) {
        let Some(what) = self.pending.clone() else {
            return;
        };
        let name = self.project().map(|p| p.name()).unwrap_or_default();
        let action = match what {
            Pending::CloseProject => "close the project",
            Pending::Open(_) => "open another project",
            Pending::OpenDemo => "open the demo",
            Pending::Quit => "quit",
        };
        egui::Window::new("Unsaved changes")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.label(format!(
                    "“{name}” has changes that are not saved. Save them before you {action}?"
                ));
                ui.horizontal(|ui| {
                    if ui.button("Save").clicked() {
                        self.resolve_pending(true);
                    }
                    if ui.button("Don't save").clicked() {
                        self.resolve_pending(false);
                    }
                    if ui.button("Cancel").clicked() {
                        self.cancel_pending();
                    }
                });
            });
    }
}

impl eframe::App for KerfwrightApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        if ui.ctx().input(|i| i.viewport().close_requested()) && !self.close_requested() {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
        }
        self.show_with(ui, frame.wgpu_render_state());
    }
}
