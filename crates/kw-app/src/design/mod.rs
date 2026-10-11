//! The Design workspace: Sketch and Model as one thing, like Fusion's Design environment. A ribbon on top,
//! the browser at the left, the timeline at the bottom and the 3-D view in the middle; *Create Sketch* picks
//! a plane or a planar face, the view turns into the flat sketch canvas, *Finish Sketch* drops straight into
//! the Extrude dialog with a live preview.

pub mod browser;
pub mod geom;
pub mod overlay;
pub mod ribbon;
pub mod timeline;
pub mod tools;

use crate::chrome::theme;
use crate::model::ModelWorkspace;
use crate::project::Project;
use crate::sketch::SketchWorkspace;
use crate::viewport::{ModelScene, Pick, ViewOpts, Viewport3d};
use eframe::egui;
use geom::PlaneFrame;
use kw_brep::BRep;
use kw_feature::{BodyOp, Evaluation, FeatureId, FeatureSpec, PlaneSpec};
use std::collections::HashSet;
use std::sync::Arc;
use tools::{BuildCtx, Dialog, Outcome, Tool};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Idle,
    /// Waiting for a plane or planar face to sketch on.
    PickPlane,
    Sketching,
}

/// The live preview of the open dialog.
#[derive(Debug, Clone, Default)]
pub struct Preview {
    pub key: String,
    pub brep: Option<Arc<BRep>>,
    pub error: Option<String>,
}

/// Things the panels ask for while they are being drawn; carried out afterwards.
#[derive(Debug, Clone)]
pub enum Action {
    Select(FeatureId),
    Edit(FeatureId),
    Delete(FeatureId),
    Suppress(FeatureId, bool),
    RollbackTo(FeatureId),
    ClearRollback,
    Move(FeatureId, usize),
    Open(Tool),
    StartSketch,
    FinishSketch,
    CancelSketch,
    SketchOn(PlaneSpec, PlaneFrame),
    ToggleSketch(FeatureId),
}

pub struct DesignWorkspace {
    /// The editing commands (add / replace features, parameters, selection, error text).
    pub model: ModelWorkspace,
    pub sketch: SketchWorkspace,
    pub viewport: Viewport3d,
    pub mode: Mode,
    pub dialog: Option<Dialog>,
    pub tab: usize,
    pub hidden_sketches: HashSet<FeatureId>,
    /// Origin planes shown in the 3-D view (XY, XZ, YZ).
    pub plane_visible: [bool; 3],
    pub hover_plane: Option<usize>,
    pub preview: Option<Preview>,
    pub status: String,
    pub hole_plane: Option<PlaneSpec>,
    pub timeline: timeline::State,
    pub params_open: bool,
    pub actions: Vec<Action>,
    /// Timeline position of the viewport's last click (used for double-click handling).
    pub shaded: bool,
}

impl Default for DesignWorkspace {
    fn default() -> Self {
        DesignWorkspace {
            model: ModelWorkspace::default(),
            sketch: SketchWorkspace::default(),
            viewport: Viewport3d::default(),
            mode: Mode::Idle,
            dialog: None,
            tab: 0,
            hidden_sketches: HashSet::new(),
            plane_visible: [false; 3],
            hover_plane: None,
            preview: None,
            status: String::new(),
            hole_plane: None,
            timeline: timeline::State::default(),
            params_open: false,
            actions: Vec::new(),
            shaded: true,
        }
    }
}

impl DesignWorkspace {
    // ------------------------------------------------------------------------------ queries

    /// A one-line hint for the status bar.
    pub fn status_line(&self) -> Option<String> {
        if !self.model.error.is_empty() {
            return Some(self.model.error.clone());
        }
        (!self.status.is_empty()).then(|| self.status.clone())
    }

    pub fn picks(&self) -> &[Pick] {
        &self.model.picks
    }

    fn base_outputs(ev: &Evaluation) -> Option<(&Arc<BRep>, &kw_feature::NameMap)> {
        let (_, o) = ev.final_solid()?;
        Some((o.brep()?, &o.names))
    }

    // ------------------------------------------------------------------------------- sketching

    /// Enter the sketch mode on `plane` (frame `frame`): a flat canvas with the body's outline as reference.
    pub fn begin_sketch(&mut self, project: &mut Project, plane: PlaneSpec, frame: PlaneFrame) {
        self.sketch.begin_new_on(plane);
        self.sketch.app.reference = Self::reference_geometry(project, &frame);
        self.enter_sketching();
    }

    /// The model's edges lying in the sketch plane, as faint reference lines.
    fn reference_geometry(project: &mut Project, frame: &PlaneFrame) -> Vec<Vec<(f64, f64)>> {
        let ev = project.evaluation();
        match ev.final_solid().and_then(|(_, o)| o.brep().cloned()) {
            Some(b) => geom::project_edges(&ModelScene::build(&b), frame),
            None => Vec::new(),
        }
    }

    fn enter_sketching(&mut self) {
        self.mode = Mode::Sketching;
        self.tab = 2;
        self.dialog = None;
        self.preview = None;
        self.model.picks.clear();
        self.status = "Sketch: draw with the tools in the ribbon, then Finish Sketch.".into();
    }

    pub fn edit_sketch(&mut self, project: &mut Project, id: FeatureId) {
        match self.sketch.begin_edit(project, id) {
            Ok(()) => {
                let ev = project.evaluation();
                let frame = match ev.output(id).map(|o| &o.product) {
                    Some(kw_feature::Product::Sketch(sp)) => Some(PlaneFrame {
                        origin: [sp.plane.origin.x, sp.plane.origin.y, sp.plane.origin.z],
                        x: [sp.plane.x.x, sp.plane.x.y, sp.plane.x.z],
                        y: [sp.plane.y.x, sp.plane.y.y, sp.plane.y.z],
                        n: [sp.plane.z.x, sp.plane.z.y, sp.plane.z.z],
                    }),
                    _ => None,
                };
                self.sketch.app.reference = match frame {
                    Some(f) => Self::reference_geometry(project, &f),
                    None => Vec::new(),
                };
                self.enter_sketching();
            }
            Err(e) => self.model.error = e,
        }
    }

    pub fn finish_sketch(&mut self, project: &mut Project) {
        let was_new = matches!(self.sketch.editing, crate::sketch::Editing::New(_));
        if let Some(id) = self.sketch.finish(project) {
            self.mode = Mode::Idle;
            self.tab = 0;
            self.model.selected = Some(id);
            self.status.clear();
            if was_new {
                // straight into Extrude, as in Fusion
                let has_body = project.evaluation().final_solid().is_some();
                self.open_tool(
                    project,
                    Tool::extrude(Some(id), if has_body { BodyOp::Join } else { BodyOp::New }),
                );
            }
        } else if !self.sketch.error.is_empty() {
            self.model.error = self.sketch.error.clone();
        }
    }

    pub fn cancel_sketch(&mut self) {
        self.sketch.cancel();
        self.mode = Mode::Idle;
        self.tab = 0;
        self.status.clear();
    }

    // ------------------------------------------------------------------------------------ tools

    pub fn open_tool(&mut self, project: &mut Project, tool: Tool) {
        let doc = project.document();
        if doc.rollback.is_some() {
            self.model.error =
                "Roll the timeline forward to its end before adding features.".into();
            return;
        }
        let mut tool = tool;
        let last_tool_feature = doc
            .features
            .iter()
            .rev()
            .find(|f| {
                matches!(
                    f.spec,
                    FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
                )
            })
            .map(|f| f.id);
        let sel_tool = self.model.selected.filter(|s| {
            doc.get(*s).is_some_and(|f| {
                matches!(
                    f.spec,
                    FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
                )
            })
        });
        match &mut tool {
            Tool::Pattern { source, .. }
            | Tool::CircularPattern { source, .. }
            | Tool::Mirror { source, .. } => *source = sel_tool.or(last_tool_feature),
            Tool::Combine {
                target, tool: t, ..
            } => {
                *target = last_tool_feature;
                *t = None;
            }
            Tool::Extrude { sketch, .. } | Tool::Revolve { sketch, .. } if sketch.is_none() => {
                let sel = self.model.selected.filter(|s| {
                    doc.get(*s)
                        .is_some_and(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                });
                *sketch = sel.or_else(|| {
                    doc.features
                        .iter()
                        .rev()
                        .find(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                        .map(|f| f.id)
                });
            }
            _ => {}
        }
        self.model.picks.clear();
        self.hole_plane = None;
        self.preview = None;
        self.mode = Mode::Idle;
        self.dialog = Some(Dialog {
            tool,
            editing: None,
        });
        self.model.error.clear();
    }

    pub fn open_edit(&mut self, project: &mut Project, id: FeatureId) {
        let doc = project.document();
        let Some(f) = doc.get(id) else { return };
        if matches!(f.spec, FeatureSpec::Sketch(_)) {
            self.edit_sketch(project, id);
            return;
        }
        match Tool::for_feature(&f.spec) {
            Some(tool) => {
                self.model.picks.clear();
                self.hole_plane = None;
                self.preview = None;
                self.dialog = Some(Dialog {
                    tool,
                    editing: Some((id, f.spec.clone())),
                });
                self.model.selected = Some(id);
            }
            None => self.model.error = format!("{} has no editor yet", f.spec.kind_name()),
        }
    }

    /// Build the dialog's feature spec against the current document.
    fn build_spec(&self, project: &mut Project, dlg: &Dialog) -> Result<FeatureSpec, String> {
        let doc = project.document();
        let ev = project.evaluation();
        let base = Self::base_outputs(&ev);
        let ctx = BuildCtx {
            doc: &doc,
            base,
            picks: &self.model.picks,
            hole_plane: self.hole_plane.clone(),
        };
        dlg.tool.spec(&ctx, dlg.editing.as_ref().map(|(_, s)| s))
    }

    /// Recompute the preview when the dialog's inputs changed.
    pub fn update_preview(&mut self, project: &mut Project) {
        let Some(dlg) = self.dialog.clone() else {
            self.preview = None;
            return;
        };
        let doc = project.document();
        let key = format!(
            "{:?}|{}|{:?}|{:?}",
            dlg.tool,
            doc.content_hash(),
            self.model.picks,
            self.hole_plane
        );
        if self.preview.as_ref().is_some_and(|p| p.key == key) {
            return;
        }
        let mut pv = Preview {
            key,
            brep: None,
            error: None,
        };
        match self.build_spec(project, &dlg) {
            Err(e) => pv.error = Some(e),
            Ok(spec) => {
                let mut tmp = (*doc).clone();
                let name = crate::model::unique_name(&doc, dlg.tool.name_base());
                let added = match &dlg.editing {
                    Some((id, _)) => tmp.replace_spec(*id, spec).map(|_| *id),
                    None => tmp.add(&name, spec),
                };
                match added {
                    Err(e) => pv.error = Some(e.to_string()),
                    Ok(id) => {
                        let ev = project.engine.evaluate(&tmp);
                        if let Some(e) = ev.error(id) {
                            pv.error = Some(e.to_string());
                        } else if let Some((_, o)) = ev.final_solid() {
                            pv.brep = o.brep().cloned();
                        }
                    }
                }
            }
        }
        self.preview = Some(pv);
    }

    pub fn commit_dialog(&mut self, project: &mut Project) -> bool {
        let Some(dlg) = self.dialog.clone() else {
            return false;
        };
        let spec = match self.build_spec(project, &dlg) {
            Ok(s) => s,
            Err(e) => {
                self.model.error = e;
                return false;
            }
        };
        let doc = project.document();
        let name = crate::model::unique_name(&doc, dlg.tool.name_base());
        let ok = match &dlg.editing {
            Some((id, _)) => {
                let id = *id;
                self.model
                    .edit(project, |d| d.replace_spec(id, spec).map(|_| id))
            }
            None => self.model.edit(project, |d| d.add(&name, spec)),
        };
        match ok {
            Some(id) => {
                self.model.selected = Some(id);
                self.dialog = None;
                self.preview = None;
                self.model.picks.clear();
                self.hole_plane = None;
                true
            }
            None => false,
        }
    }

    pub fn close_dialog(&mut self) {
        self.dialog = None;
        self.preview = None;
        self.model.picks.clear();
        self.hole_plane = None;
        self.model.error.clear();
    }

    // --------------------------------------------------------------------------------- actions

    pub fn run_actions(&mut self, project: &mut Project) {
        let actions = std::mem::take(&mut self.actions);
        for a in actions {
            match a {
                Action::Select(id) => self.model.selected = Some(id),
                Action::Edit(id) => self.open_edit(project, id),
                Action::Delete(id) => {
                    if self
                        .model
                        .edit(project, |d| d.remove(id).map(|_| ()))
                        .is_some()
                        && self.model.selected == Some(id)
                    {
                        self.model.selected = None;
                    }
                }
                Action::Suppress(id, s) => {
                    self.model.edit(project, |d| d.set_suppressed(id, s));
                }
                Action::RollbackTo(id) => {
                    self.model.edit(project, |d| d.rollback_to(id));
                }
                Action::ClearRollback => {
                    self.model.edit(project, |d| {
                        d.clear_rollback();
                        Ok(())
                    });
                }
                Action::Move(id, idx) => {
                    self.model.edit(project, |d| d.move_feature(id, idx));
                }
                Action::Open(t) => self.open_tool(project, t),
                Action::StartSketch => {
                    if project.document().rollback.is_some() {
                        self.model.error =
                            "Roll the timeline forward to its end before adding features.".into();
                        continue;
                    }
                    self.mode = Mode::PickPlane;
                    self.dialog = None;
                    self.preview = None;
                    self.model.picks.clear();
                    self.status = "Select a sketch plane or a planar face.".into();
                }
                Action::FinishSketch => self.finish_sketch(project),
                Action::CancelSketch => self.cancel_sketch(),
                Action::SketchOn(plane, frame) => self.begin_sketch(project, plane, frame),
                Action::ToggleSketch(id) => {
                    if !self.hidden_sketches.remove(&id) {
                        self.hidden_sketches.insert(id);
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------------------------- UI

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        // ---- bottom: timeline (always), left: browser
        self.timeline_ui(ui, project);
        egui::Panel::left("design_browser")
            .default_size(250.0)
            .show(ui, |ui| self.browser_ui(ui, project));
        if self.mode == Mode::Sketching {
            egui::Panel::right("sketch_palette")
                .default_size(250.0)
                .show(ui, |ui| self.palette_ui(ui));
            egui::CentralPanel::default().show(ui, |ui| {
                self.sketch.app.shortcuts(ui.ctx());
                self.sketch.app.canvas_ui(ui);
            });
            self.sketch.app.dialogs(ui.ctx());
        } else {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ui, |ui| self.viewport_ui(ui, project, rs));
        }
        self.run_actions(project);
    }

    fn palette_ui(&mut self, ui: &mut egui::Ui) {
        ui.add_space(4.0);
        ui.label(egui::RichText::new("SKETCH PALETTE").size(11.0).strong());
        ui.separator();
        ui.label(egui::RichText::new("Solver").color(theme::TEXT_WEAK));
        self.sketch.app.inspector(ui);
        if !self.sketch.error.is_empty() {
            ui.colored_label(theme::BAD, &self.sketch.error);
        }
    }

    fn viewport_ui(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        let ev = project.evaluation();
        let doc = project.document();
        self.update_preview(project);
        let (shown, base_key) = match self.preview.as_ref().and_then(|p| p.brep.clone()) {
            Some(b) => (Some(b), 0xB0B0_u128),
            None => (ev.final_solid().and_then(|(_, o)| o.brep().cloned()), 0),
        };
        let geom_key = match (&self.preview, ev.final_hashes()) {
            (Some(p), _) if p.brep.is_some() => {
                let mut h = kw_feature::Hasher128::new();
                h.bytes(p.key.as_bytes());
                h.finish()
            }
            (_, Some(h)) => h.0 ^ h.1.rotate_left(1),
            _ => 0,
        };
        let segs = geom::sketch_segments(&ev, &doc, &self.hidden_sketches);
        let key = geom_key
            ^ base_key
            ^ ((segs.len() as u128) << 96)
            ^ ((self.hidden_sketches.len() as u128) << 90);
        let highlight = self.model.picks.last().copied();
        let opts = ViewOpts {
            hover: true,
            navigate: true,
        };
        self.viewport.wireframe = !self.shaded;
        let b = shown.clone();
        let out = self.viewport.show_with(
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
            highlight,
            opts,
        );
        self.overlay_ui(ui, project, &ev, &out);
        self.handle_clicks(ui, project, &ev, &out);
        // dialog
        if let Some(mut dlg) = self.dialog.take() {
            let picks = self.model.picks.clone();
            let err = self
                .preview
                .as_ref()
                .and_then(|p| p.error.clone())
                .or_else(|| (!self.model.error.is_empty()).then(|| self.model.error.clone()));
            let (outcome, _) = tools::panel(ui.ctx(), &mut dlg, &doc, &picks, err.as_deref());
            self.dialog = Some(dlg);
            match outcome {
                Outcome::Ok => {
                    if let Some(e) = self.preview.as_ref().and_then(|p| p.error.clone()) {
                        self.model.error = e;
                    } else {
                        self.commit_dialog(project);
                    }
                }
                Outcome::Cancel => self.close_dialog(),
                Outcome::Open => {}
            }
        }
        // status line inside the view
        if !self.status.is_empty() && self.mode != Mode::Idle {
            let p = ui.painter();
            let r = egui::Rect::from_min_size(
                out.rect.left_top() + egui::vec2(10.0, 8.0),
                egui::vec2(420.0, 24.0),
            );
            p.rect_filled(
                r,
                3.0,
                egui::Color32::from_rgba_unmultiplied(255, 255, 255, 220),
            );
            p.text(
                r.left_center() + egui::vec2(8.0, 0.0),
                egui::Align2::LEFT_CENTER,
                &self.status,
                egui::FontId::proportional(12.0),
                theme::TEXT,
            );
        }
        if ui.ctx().input(|i| i.key_pressed(egui::Key::Escape)) {
            match self.mode {
                Mode::PickPlane => {
                    self.mode = Mode::Idle;
                    self.status.clear();
                }
                _ if self.dialog.is_some() => self.close_dialog(),
                _ => self.model.picks.clear(),
            }
        }
    }

    pub fn handle_clicks(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        ev: &Arc<Evaluation>,
        out: &crate::viewport::ViewOut,
    ) {
        let shift = ui.ctx().input(|i| i.modifiers.shift);
        if let Some(click) = &out.click {
            match self.mode {
                Mode::PickPlane => {
                    let face = match click {
                        Some(Pick::Face(f)) => Some(*f),
                        _ => None,
                    };
                    let brep = Self::base_outputs(ev);
                    if let (Some(f), Some((b, names))) = (face, brep) {
                        match (geom::face_frame(b, f), names.reference_to(b, f)) {
                            (Some(frame), Some(r)) => {
                                self.actions
                                    .push(Action::SketchOn(PlaneSpec::Face(r), frame));
                                return;
                            }
                            (None, _) => {
                                self.model.error = "that face is not planar".into();
                                self.status =
                                    "That face is not planar - pick a plane or a flat face.".into();
                                return;
                            }
                            _ => {}
                        }
                    }
                    if let Some(i) = self.hover_plane {
                        self.actions.push(Action::SketchOn(
                            geom::origin_spec(geom::ORIGIN_PLANES[i].1),
                            geom::origin_frame(geom::ORIGIN_PLANES[i].1),
                        ));
                    }
                }
                Mode::Idle => {
                    // tools that take picks, otherwise plain selection
                    let (edges, faces, hole) = self
                        .dialog
                        .as_ref()
                        .map(|d| {
                            (
                                d.tool.picks_edges(),
                                d.tool.picks_faces(),
                                matches!(d.tool, Tool::Hole { .. }),
                            )
                        })
                        .unwrap_or((false, false, false));
                    match click {
                        Some(p) => {
                            let accept = if self.dialog.is_none() {
                                true
                            } else {
                                (edges && matches!(p, Pick::Edge(_)))
                                    || (faces && matches!(p, Pick::Face(_)))
                            };
                            if accept {
                                if hole {
                                    self.pick_hole_face(ev, *p, out);
                                } else if shift || edges || (faces && !hole) {
                                    if let Some(i) = self.model.picks.iter().position(|q| q == p) {
                                        self.model.picks.remove(i);
                                    } else {
                                        self.model.picks.push(*p);
                                    }
                                } else {
                                    self.model.picks = vec![*p];
                                }
                            }
                        }
                        None => {
                            if !shift && self.dialog.is_none() {
                                self.model.picks.clear();
                            }
                        }
                    }
                }
                Mode::Sketching => {}
            }
        }
        if let Some(Some(p)) = &out.double_click {
            // double-click a face while idle starts nothing; reserved for edit-in-place of features later
            let _ = p;
        }
        let _ = project;
    }

    /// A click on a planar face for the Hole tool: remember the plane and put the click's position in the fields.
    fn pick_hole_face(&mut self, ev: &Arc<Evaluation>, p: Pick, out: &crate::viewport::ViewOut) {
        let Pick::Face(f) = p else { return };
        let Some((brep, names)) = Self::base_outputs(ev) else {
            return;
        };
        let (Some(frame), Some(r)) = (geom::face_frame(brep, f), names.reference_to(brep, f))
        else {
            self.model.error = "that face is not planar".into();
            return;
        };
        self.hole_plane = Some(PlaneSpec::Face(r));
        self.model.picks = vec![p];
        if let (Some(pos), Some(m)) = (out.click_pos, self.viewport.model()) {
            let (o, d) = self.viewport.ray(pos);
            let c = m.scene.center;
            let (o, d) = ([o.x + c.x, o.y + c.y, o.z + c.z], [d.x, d.y, d.z]);
            let denom = d[0] * frame.n[0] + d[1] * frame.n[1] + d[2] * frame.n[2];
            if denom.abs() > 1e-9 {
                let t = ((frame.origin[0] - o[0]) * frame.n[0]
                    + (frame.origin[1] - o[1]) * frame.n[1]
                    + (frame.origin[2] - o[2]) * frame.n[2])
                    / denom;
                let hit = [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
                let (u, v, _) = frame.to_plane(hit);
                if let Some(Dialog {
                    tool: Tool::Hole { x, y, .. },
                    ..
                }) = self.dialog.as_mut()
                {
                    *x = format!("{u:.3} mm");
                    *y = format!("{v:.3} mm");
                }
            }
        }
    }
}
