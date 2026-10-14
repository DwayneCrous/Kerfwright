//! The Form dialog's cage in the viewport: the pointer and keys that pick and drag its vertices, the window
//! with the lock / snap / symmetry / crease controls, the quick preview of the surface, and the local undo.
//! The maths is in [`super::form_edit`].

use super::form_edit::{self, Hit, Lock, View, P3};
use super::tools::Tool;
use super::DesignWorkspace;
use eframe::egui;
use kw_subd::SubdMesh;

/// The colour of the preview surface.
pub const PREVIEW_COLOUR: [u8; 3] = [150, 190, 225];

/// What the viewport draws for a Form being edited: the cage and its scale.
pub struct FormPreview {
    pub cage: SubdMesh,
    pub scale: f64,
}

impl FormPreview {
    /// The surface of the cage as a mesh (None for a cage that cannot be subdivided).
    pub fn mesh(&self) -> Option<kw_mesh3d::SurfaceMesh> {
        let p = kw_subd::fast_mesh(&self.cage, self.scale, kw_subd::levels_for(&self.cage)).ok()?;
        Some(kw_mesh3d::SurfaceMesh {
            tags: vec![0; p.tris.len()],
            nodes: p.positions,
            tris: p.tris,
        })
    }

    /// A key that changes with the cage and its scale.
    pub fn key(&self) -> u128 {
        let mut h = kw_feature::Hasher128::new();
        h.u128(kw_feature::id::hash_serde(&self.cage))
            .u64(self.scale.to_bits());
        h.finish()
    }
}

impl DesignWorkspace {
    /// Is the Form dialog open?
    pub fn form_dialog_open(&self) -> bool {
        matches!(
            self.dialog.as_ref().map(|d| &d.tool),
            Some(Tool::Form { .. })
        )
    }

    /// The cage and scale of the open Form dialog, when the scale is a plain number.
    pub fn form_preview(&self) -> Option<FormPreview> {
        match self.dialog.as_ref().map(|d| &d.tool) {
            Some(Tool::Form { cage, scale, .. }) => Some(FormPreview {
                cage: cage.clone(),
                scale: scale.trim().parse::<f64>().ok().filter(|k| *k > 0.0)?,
            }),
            _ => None,
        }
    }

    /// Ctrl+Z / Ctrl+Y while the Form dialog is open step through the cages made in it.
    pub fn form_undo_redo(&mut self, undo: bool) {
        let Some(Tool::Form { cage, kind, .. }) = self.dialog.as_mut().map(|d| &mut d.tool) else {
            return;
        };
        let back = if undo {
            self.form_edit.undo(cage)
        } else {
            self.form_edit.redo(cage)
        };
        match back {
            Some(c) => {
                *cage = c;
                *kind = 255;
                self.form_edit
                    .selected
                    .retain(|v| (*v as usize) < cage.verts.len());
            }
            None => {
                self.status = if undo {
                    "nothing to undo in this form".into()
                } else {
                    "nothing to redo in this form".into()
                }
            }
        }
    }

    /// Called every frame before the viewport handles the pointer: picks and drags cage vertices. Returns
    /// whether a drag or a box is in progress (the view must not orbit).
    pub(super) fn form_drag_frame(&mut self, ui: &egui::Ui) -> bool {
        if !self.form_dialog_open() || self.mode == super::Mode::Sketching {
            if self.form_edit.undo.len() + self.form_edit.redo.len() + self.form_edit.selected.len()
                > 0
                || self.form_edit.frozen_centre.is_some()
            {
                self.form_edit.clear();
            }
            return false;
        }
        // hold the scene centre for the whole session: the preview must not shift the view
        if self.form_edit.frozen_centre.is_none() {
            let c = self
                .viewport
                .model()
                .map(|m| [m.scene.center.x, m.scene.center.y, m.scene.center.z])
                .unwrap_or([0.0; 3]);
            self.form_edit.frozen_centre = Some(c);
        }
        let Some(k) = self.form_preview().map(|p| p.scale) else {
            return false;
        };
        let (pressed, down, pos, shift, keys) = ui.input(|i| {
            (
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.pointer.latest_pos(),
                i.modifiers.shift,
                [
                    i.key_pressed(egui::Key::X),
                    i.key_pressed(egui::Key::Y),
                    i.key_pressed(egui::Key::Z) && !i.modifiers.command,
                ],
            )
        });
        let rect = self.viewport.rect();
        let cube = egui::Rect::from_min_size(
            rect.right_top() + egui::vec2(-146.0, 8.0),
            egui::vec2(136.0, 148.0),
        );
        let over_view = pos.is_some_and(|p| rect.contains(p) && !cube.contains(p));
        // X / Y / Z lock the drag to an axis, with Shift to the plane across it
        if over_view && !ui.ctx().egui_wants_keyboard_input() {
            for (axis, pressed) in keys.iter().enumerate() {
                if *pressed {
                    self.form_edit.key_lock(axis as u8, shift);
                }
            }
        }
        let off = self.viewport.origin_in_scene();
        let vp = &self.viewport;
        let project = |p: P3| vp.project([p[0] + off.x, p[1] + off.y, p[2] + off.z]);
        let ray = |pos: egui::Pos2| -> (P3, P3) {
            let (o, d) = vp.ray(pos);
            ([o.x - off.x, o.y - off.y, o.z - off.z], [d.x, d.y, d.z])
        };
        let view = View {
            project: &project,
            ray: &ray,
        };
        let cam = &vp.camera;
        let fwd = (cam.target - cam.eye()).normalize();
        let forward = [fwd.x, fwd.y, fwd.z];
        let released = !down;
        let Some(Tool::Form {
            cage,
            kind,
            vertex,
            built,
            ..
        }) = self.dialog.as_mut().map(|d| &mut d.tool)
        else {
            return false;
        };
        let fe = &mut self.form_edit;
        fe.selected.retain(|v| (*v as usize) < cage.verts.len());
        // ---- a drag in progress
        if fe.drag.is_some() {
            if released {
                fe.end_drag();
                return false;
            }
            if let Some(pos) = pos {
                fe.update_drag(cage, k, ray(pos));
            }
            return true;
        }
        // ---- a box in progress
        if let Some((a, _)) = fe.boxsel {
            if let Some(pos) = pos {
                fe.boxsel = Some((a, pos));
            }
            if released {
                if let Some((a, b)) = fe.boxsel.take() {
                    let inside =
                        form_edit::box_select(cage, k, &view, egui::Rect::from_two_pos(a, b));
                    if !shift {
                        fe.selected.clear();
                    }
                    fe.selected.extend(inside);
                }
                return false;
            }
            return true;
        }
        // ---- a click on empty space that clears the selection
        if let Some(at) = fe.clear_on_release {
            if released {
                fe.clear_on_release = None;
                if pos.is_none_or(|p| p.distance(at) < 4.0) {
                    fe.selected.clear();
                }
            }
        }
        // ---- hover
        fe.hover = pos
            .filter(|_| over_view)
            .and_then(|p| form_edit::hit_test(cage, k, &view, p));
        // ---- a press
        if pressed && over_view {
            let pos = pos.expect("over the view");
            match fe.hover {
                Some(hit) => {
                    if shift {
                        fe.select(cage, hit, true);
                    } else {
                        let vs = form_edit::vertices_of(cage, hit);
                        if !vs.iter().all(|v| fe.selected.contains(v)) {
                            fe.select(cage, hit, false);
                        }
                        // the grabbed vertex: the hit one, or the nearest of an edge's / face's
                        let primary = vs
                            .iter()
                            .copied()
                            .min_by(|a, b| {
                                let d = |v: &u32| {
                                    project(form_edit::world(cage, k, *v))
                                        .map_or(f32::MAX, |p| p.distance(pos))
                                };
                                d(a).total_cmp(&d(b))
                            })
                            .unwrap_or(vs[0]);
                        *vertex = primary as usize;
                        *kind = 255; // a dragged cage is no longer the primitive of its size
                        *built = (255, built.1, built.2);
                        fe.begin_drag(cage, primary, form_edit::world(cage, k, primary), forward);
                        // grab where the pointer is, so that the vertex does not jump to it
                        if let Some(d) = fe.drag.as_mut() {
                            if let Some(delta) = form_edit::drag_delta(d, Lock::Free, ray(pos)) {
                                for (g, dl) in d.grab.iter_mut().zip(delta) {
                                    *g += dl;
                                }
                            }
                        }
                    }
                    if let Hit::Vertex(v) = hit {
                        *vertex = v as usize;
                    }
                }
                None => {
                    if shift {
                        fe.boxsel = Some((pos, pos));
                    } else {
                        fe.clear_on_release = Some(pos);
                    }
                }
            }
        }
        fe.dragging()
    }

    /// The window beside the cage: how a drag moves, snapping, symmetry, creases, undo.
    pub fn form_edit_window(&mut self, ctx: &egui::Context) {
        if !self.form_dialog_open() {
            return;
        }
        let Some(Tool::Form {
            cage, kind, built, ..
        }) = self.dialog.as_mut().map(|d| &mut d.tool)
        else {
            return;
        };
        // an edit that is not a change of primitive makes the cage custom
        let rebuilt = self.form_edit.last_built != Some(*built);
        self.form_edit.last_built = Some(*built);
        if self.form_edit.sync(cage) && !rebuilt && (*kind as usize) < super::form::KINDS.len() {
            *kind = 255;
            *built = (255, built.1, built.2);
            self.form_edit.last_built = Some(*built);
        }
        let fe = &mut self.form_edit;
        egui::Window::new(egui::RichText::new("EDIT CAGE").size(11.5).strong())
            .default_pos(egui::pos2(24.0, 180.0))
            .collapsible(true)
            .resizable(false)
            .default_width(250.0)
            .show(ctx, |ui| {
                let weak = egui::Color32::from_gray(120);
                ui.label(
                    egui::RichText::new(
                        "Click a vertex, edge or face to select it; drag to move. Shift adds; \
                         Shift-drag in empty space boxes. X / Y / Z lock the drag to an axis, \
                         Shift+X / Y / Z to the plane across it. Ctrl+Z undoes in the cage.",
                    )
                    .size(10.5)
                    .color(weak),
                );
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label(format!("{} selected", fe.selected.len()));
                    if ui.small_button("All").clicked() {
                        fe.selected = (0..cage.verts.len() as u32).collect();
                    }
                    if ui.small_button("None").clicked() {
                        fe.selected.clear();
                    }
                });
                ui.separator();
                ui.label(egui::RichText::new("Move").color(weak));
                ui.horizontal_wrapped(|ui| {
                    for l in [
                        Lock::Free,
                        Lock::Axis(0),
                        Lock::Axis(1),
                        Lock::Axis(2),
                        Lock::Plane(2),
                        Lock::Plane(0),
                        Lock::Plane(1),
                    ] {
                        ui.selectable_value(&mut fe.lock, l, l.label());
                    }
                });
                ui.horizontal(|ui| {
                    let mut on = fe.snap.is_some();
                    if ui.checkbox(&mut on, "Snap to grid").changed() {
                        fe.snap = on.then_some(1.0);
                    }
                    if let Some(g) = fe.snap.as_mut() {
                        ui.add(
                            egui::DragValue::new(g)
                                .speed(0.1)
                                .range(0.01..=1000.0)
                                .suffix(" mm"),
                        );
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Symmetry");
                    egui::ComboBox::from_id_salt("form_mirror")
                        .selected_text(match fe.mirror {
                            None => "Off",
                            Some(0) => "Across YZ (x → −x)",
                            Some(1) => "Across XZ (y → −y)",
                            Some(_) => "Across XY (z → −z)",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut fe.mirror, None, "Off");
                            ui.selectable_value(&mut fe.mirror, Some(0), "Across YZ (x → −x)");
                            ui.selectable_value(&mut fe.mirror, Some(1), "Across XZ (y → −y)");
                            ui.selectable_value(&mut fe.mirror, Some(2), "Across XY (z → −z)");
                        });
                });
                ui.separator();
                let edges = form_edit::selected_edges(cage, &fe.selected);
                ui.label(
                    egui::RichText::new(format!(
                        "Edges between the selected vertices: {}",
                        edges.len()
                    ))
                    .color(weak),
                );
                ui.horizontal(|ui| {
                    let sharp = edges.iter().any(|(a, b)| cage.crease_of(*a, *b) <= 0.0);
                    if ui
                        .add_enabled(!edges.is_empty(), egui::Button::new("Sharp"))
                        .on_hover_text("Crease the edges between the selected vertices")
                        .clicked()
                    {
                        for (a, b) in &edges {
                            cage.set_crease(*a, *b, 100.0);
                        }
                    }
                    if ui
                        .add_enabled(!edges.is_empty(), egui::Button::new("Smooth"))
                        .on_hover_text(
                            "Remove the creases of the edges between the selected vertices",
                        )
                        .clicked()
                    {
                        for (a, b) in &edges {
                            cage.set_crease(*a, *b, 0.0);
                        }
                    }
                    if ui
                        .add_enabled(!edges.is_empty(), egui::Button::new("Toggle"))
                        .on_hover_text(
                            "Smooth the edges if they are all sharp, else make them sharp",
                        )
                        .clicked()
                    {
                        for (a, b) in &edges {
                            cage.set_crease(*a, *b, if sharp { 100.0 } else { 0.0 });
                        }
                    }
                });
                ui.separator();
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!fe.undo.is_empty(), egui::Button::new("Undo"))
                        .clicked()
                    {
                        if let Some(c) = fe.undo(cage) {
                            *cage = c;
                            *kind = 255;
                        }
                    }
                    if ui
                        .add_enabled(!fe.redo.is_empty(), egui::Button::new("Redo"))
                        .clicked()
                    {
                        if let Some(c) = fe.redo(cage) {
                            *cage = c;
                            *kind = 255;
                        }
                    }
                    ui.label(egui::RichText::new(format!("{} steps", fe.undo.len())).color(weak));
                });
            });
    }
}
