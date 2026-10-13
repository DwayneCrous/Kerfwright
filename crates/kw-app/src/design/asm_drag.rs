//! Dragging components in the posed assembly view, and the numeric Move command.

use super::assembly::AsmAction;
use super::{Action, DesignWorkspace};
use crate::chrome::theme::BAD;
use crate::project::Project;
use eframe::egui;
use kw_assembly::{Assembly, Drag, Placement, Ray};
use kw_expr::Dim;
use kw_math::{Transform3, Vector3};

/// The Move command's fields: a translation along the world axes (mm) and a turn about them (degrees).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MoveForm {
    pub t: [String; 3],
    pub r: [String; 3],
    pub error: String,
}

impl MoveForm {
    pub fn new() -> MoveForm {
        MoveForm {
            t: ["0".into(), "0".into(), "0".into()],
            r: ["0".into(), "0".into(), "0".into()],
            error: String::new(),
        }
    }

    /// The turn (about the world axes) and the shift the fields describe.
    pub fn transform(&self) -> Result<(Transform3, Vector3<f64>), String> {
        let num = |s: &str, d: Dim| -> Result<f64, String> {
            if s.trim().is_empty() {
                return Ok(0.0);
            }
            super::asm_joint::number(s, d)
        };
        let t = [
            num(&self.t[0], Dim::LENGTH)?,
            num(&self.t[1], Dim::LENGTH)?,
            num(&self.t[2], Dim::LENGTH)?,
        ];
        let r = [
            num(&self.r[0], Dim::ANGLE)?,
            num(&self.r[1], Dim::ANGLE)?,
            num(&self.r[2], Dim::ANGLE)?,
        ];
        Ok((
            Transform3::rotation(Vector3::x(), r[0])
                .then(&Transform3::rotation(Vector3::y(), r[1]))
                .then(&Transform3::rotation(Vector3::z(), r[2])),
            Vector3::new(t[0], t[1], t[2]),
        ))
    }
}

/// Move a free component (or its rigid group): turn it by `turn` about its own origin, then shift it.
pub fn move_free(
    asm: &mut Assembly,
    instance: usize,
    turn: &Transform3,
    shift: Vector3<f64>,
) -> Result<(), String> {
    if !asm.is_free(instance) {
        return Err(format!(
            "'{}' is placed by a joint or a ground: change the joint's value, or release it first",
            asm.instances.get(instance).map_or("?", |i| i.name.as_str())
        ));
    }
    let placed = asm.solve().map_err(|e| e.to_string())?;
    let anchor = asm.float_anchor(instance);
    let now = placed[anchor];
    let origin = now.apply_point(&kw_math::Point3::origin()).coords;
    let motion = Transform3::translation(-origin)
        .then(turn)
        .then(&Transform3::translation(origin + shift));
    asm.instances[anchor].at = Placement::from_transform(&now.then(&motion));
    Ok(())
}

impl DesignWorkspace {
    /// A world-space ray (for the engine) through a screen position.
    pub fn asm_world_ray(&self, pos: egui::Pos2) -> Ray {
        let (o, d) = self.viewport.ray(pos);
        let s = self.viewport.origin_in_scene();
        Ray {
            origin: [o.x - s.x, o.y - s.y, o.z - s.z],
            dir: [d.x, d.y, d.z],
        }
    }

    /// Called every frame of the posed view before the viewport handles the pointer: starts, continues or
    /// ends the drag of a component. Returns whether a drag is in progress (the view must not orbit).
    pub(super) fn asm_drag_frame(&mut self, ui: &egui::Ui, project: &mut Project) -> bool {
        let (pressed, down, pos) = ui.input(|i| {
            (
                i.pointer.primary_pressed(),
                i.pointer.primary_down(),
                i.pointer.latest_pos(),
            )
        });
        if !self.asm.view
            || self.asm.dialog.is_some()
            || self.asm.rigid.is_some()
            || project.editing().is_some()
        {
            self.asm.drag = None;
            return false;
        }
        let Some(pos) = pos else {
            return self.asm.drag.is_some();
        };
        if self.asm.drag.is_none() && pressed {
            let rect = self.viewport.rect();
            let cube = egui::Rect::from_min_size(
                rect.right_top() + egui::vec2(-146.0, 8.0),
                egui::vec2(136.0, 148.0),
            );
            if rect.contains(pos) && !cube.contains(pos) {
                if let Some(top) = self.asm.view_hover {
                    self.asm_drag_begin(project, top, pos);
                }
            }
        }
        if self.asm.drag.is_some() {
            if !down {
                self.asm.drag = None;
                project.asm_drop_checkpoint_if_unchanged();
                if self.status.starts_with("Dragging") {
                    self.status.clear();
                }
                return false;
            }
            let ray = self.asm_world_ray(pos);
            if let (Some(drag), Some(asm)) = (self.asm.drag.as_mut(), project.assembly.as_mut()) {
                match drag.update(asm, &ray) {
                    Ok(()) => project.mark_extras_dirty(),
                    Err(e) => {
                        self.status = format!("Dragging stopped: {e}");
                        self.asm.drag = None;
                        return false;
                    }
                }
            }
            return true;
        }
        false
    }

    fn asm_drag_begin(&mut self, project: &mut Project, top: usize, pos: egui::Pos2) {
        let Some(asm) = project.assembly.as_ref() else {
            return;
        };
        let (o, d) = self.viewport.ray(pos);
        let Some(m) = self.viewport.model() else {
            return;
        };
        // the point of the model under the pointer
        let Some(hit) = m.scene.raycast(o, d) else {
            return;
        };
        let p = o + d * hit.t;
        let s = self.viewport.origin_in_scene();
        let grab = [p.x - s.x, p.y - s.y, p.z - s.z];
        // free components slide in the plane facing the camera (not perpendicular to this one ray)
        let cam = &self.viewport.camera;
        let fwd = (cam.target - cam.eye()).normalize();
        match Drag::begin(asm, top, grab, [fwd.x, fwd.y, fwd.z]) {
            Ok(drag) => {
                self.status = match drag.joint() {
                    Some(j) => format!(
                        "Dragging {} (joint {})",
                        asm.instances[top].name, asm.joints[j].name
                    ),
                    None => format!("Dragging {}", asm.instances[top].name),
                };
                self.asm.drag = Some(drag);
                project.asm_checkpoint();
                if !self.asm.selected.contains(&top) {
                    self.actions
                        .push(Action::Asm(AsmAction::Select(top, false)));
                }
            }
            Err(e) => self.status = e.to_string(),
        }
    }

    /// The Move command: type a shift and a turn for the selected component.
    pub fn asm_move_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(mut form) = self.asm.move_form.take() else {
            return;
        };
        let target = self.asm.selected.first().copied();
        let name = project
            .assembly
            .as_ref()
            .and_then(|a| target.and_then(|t| a.instances.get(t)))
            .map(|i| i.name.clone());
        let mut apply = false;
        let mut close = false;
        egui::Window::new(egui::RichText::new("MOVE COMPONENT").size(11.5).strong())
            .anchor(egui::Align2::RIGHT_TOP, [-12.0, 312.0])
            .collapsible(false)
            .default_width(250.0)
            .show(ctx, |ui| {
                match &name {
                    Some(n) => ui.label(egui::RichText::new(n).strong()),
                    None => ui.label("Select a component first."),
                };
                ui.label(egui::RichText::new("Move (mm), along the world axes").size(11.0));
                ui.horizontal(|ui| {
                    for (i, l) in ["X", "Y", "Z"].iter().enumerate() {
                        ui.label(*l);
                        ui.add(egui::TextEdit::singleline(&mut form.t[i]).desired_width(52.0));
                    }
                });
                ui.label(egui::RichText::new("Turn (degrees) about its own origin").size(11.0));
                ui.horizontal(|ui| {
                    for (i, l) in ["X", "Y", "Z"].iter().enumerate() {
                        ui.label(*l);
                        ui.add(egui::TextEdit::singleline(&mut form.r[i]).desired_width(52.0));
                    }
                });
                if !form.error.is_empty() {
                    ui.label(egui::RichText::new(&form.error).color(BAD).size(11.0));
                }
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(name.is_some(), egui::Button::new("Apply"))
                        .clicked()
                    {
                        apply = true;
                    }
                    if ui.button("Close").clicked() {
                        close = true;
                    }
                });
            });
        if apply {
            if let (Some(t), Some(asm)) = (target, project.assembly.as_mut()) {
                let before = asm.to_bytes();
                let result = form
                    .transform()
                    .and_then(|(turn, shift)| move_free(asm, t, &turn, shift))
                    .and_then(|_| asm.solve().map(|_| ()).map_err(|e| e.to_string()));
                match result {
                    Ok(()) => {
                        form.error.clear();
                        project.asm_checkpoint_with(Some(before));
                        form = MoveForm {
                            error: String::new(),
                            ..MoveForm::new()
                        };
                        project.mark_extras_dirty();
                        self.asm.overlaps = None;
                    }
                    Err(e) => form.error = e,
                }
            }
        }
        if !close {
            self.asm.move_form = Some(form);
        }
    }
}
