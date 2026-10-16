//! Joints made by clicking geometry (the Joint and As-Built Joint commands), the properties window of a
//! joint, and picking components in the assembly view.

use super::asm_snap::{self, Origin};
use super::assembly::AsmAction;
use super::{Action, DesignWorkspace};
use crate::chrome::theme::{ACCENT, BAD, TEXT_WEAK};
use crate::model::{param_text, parse_param};
use crate::project::Project;
use crate::viewport::{Pick, ViewOut};
use eframe::egui::{self, Color32, Stroke};
use kw_assembly::{Assembly, Joint, JointEnd, JointKind};
use kw_expr::Dim;
use kw_math::{Frame3, Point3, Transform3, Vector3};
use std::collections::HashMap;

/// A snap on a component: where on it, in world coordinates.
#[derive(Debug, Clone, PartialEq)]
pub struct Anchor {
    /// Index of the top-level instance the point belongs to.
    pub instance: usize,
    pub origin: Origin,
}

/// The Joint / As-Built Joint command while it is open.
#[derive(Debug, Clone)]
pub struct JointDialog {
    pub as_built: bool,
    pub kind: JointKind,
    /// The first snap (its component moves) and the second.
    pub first: Option<Anchor>,
    pub second: Option<Anchor>,
    /// Which snap the next click sets: 0 the first, 1 the second.
    pub active: u8,
    /// As-built: which axis the joint turns about (0 the picked direction, 1 X, 2 Y, 3 Z of the world).
    pub axis: usize,
    pub flip: bool,
    pub offset: [String; 3],
    pub angle: String,
    pub limit_on: [bool; 3],
    pub limit_lo: [String; 3],
    pub limit_hi: [String; 3],
    pub error: String,
    /// The assembly with the joint added (shown while the dialog is open).
    pub preview: Option<Assembly>,
    dirty: bool,
}

impl JointDialog {
    pub fn new(as_built: bool) -> JointDialog {
        JointDialog {
            as_built,
            kind: if as_built {
                JointKind::Rigid
            } else {
                JointKind::Revolute
            },
            first: None,
            second: None,
            active: 0,
            axis: 0,
            flip: false,
            offset: ["0".into(), "0".into(), "0".into()],
            angle: "0".into(),
            limit_on: [false; 3],
            limit_lo: ["0".into(), "0".into(), "0".into()],
            limit_hi: ["0".into(), "0".into(), "0".into()],
            error: String::new(),
            preview: None,
            dirty: true,
        }
    }

    /// Set the snap the next click is for.
    pub fn set_anchor(&mut self, a: Anchor) {
        match self.active {
            0 => {
                self.first = Some(a);
                self.active = 1;
            }
            _ => self.second = Some(a),
        }
        self.dirty = true;
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    fn limits(&self) -> Result<Vec<Option<(f64, f64)>>, String> {
        let dims = self.kind.dims();
        let mut out = Vec::new();
        for (i, d) in dims.iter().enumerate() {
            if self.limit_on[i] {
                let lo = number(&self.limit_lo[i], *d)?;
                let hi = number(&self.limit_hi[i], *d)?;
                if lo > hi {
                    return Err("a limit's minimum is above its maximum".into());
                }
                out.push(Some((lo, hi)));
            } else {
                out.push(None);
            }
        }
        Ok(out)
    }

    /// (parent, child): the first snap's component moves to the second, unless it is the grounded one.
    fn ends<'a>(&'a self, asm: &Assembly) -> Result<(&'a Anchor, &'a Anchor), String> {
        let (Some(a), Some(b)) = (&self.first, &self.second) else {
            return Err("click a point on each of the two components".into());
        };
        if a.instance == b.instance {
            return Err("a joint connects two different components".into());
        }
        Ok(
            if asm.is_grounded(a.instance) && !asm.is_grounded(b.instance) {
                (a, b)
            } else {
                (b, a)
            },
        )
    }

    /// The assembly with the joint made from the snaps added (its index is the last).
    pub fn build(&self, asm: &Assembly) -> Result<Assembly, String> {
        let (parent, child) = self.ends(asm)?;
        let mut out = asm.clone();
        if self.as_built {
            let (o, axis) = (self.second.as_ref().unwrap().origin.clone(), self.axis);
            let z = match axis {
                1 => [1.0, 0.0, 0.0],
                2 => [0.0, 1.0, 0.0],
                3 => [0.0, 0.0, 1.0],
                _ => o.z,
            };
            let name = format!("As-built {} {}", self.kind.name(), out.joints.len() + 1);
            let k = out
                .as_built_joint(
                    parent.instance,
                    child.instance,
                    self.kind,
                    o.point,
                    z,
                    &name,
                )
                .map_err(|e| e.to_string())?;
            out.joints[k].limits = self.limits()?;
            out.joints[k].flip = self.flip;
            out.solve().map_err(|e| e.to_string())?;
            return Ok(out);
        }
        let placed = out.solve().map_err(|e| e.to_string())?;
        let frame = |a: &Anchor, flip_z: bool| -> Frame3 {
            let o = &a.origin;
            let z = if flip_z {
                Vector3::new(-o.z[0], -o.z[1], -o.z[2])
            } else {
                Vector3::new(o.z[0], o.z[1], o.z[2])
            };
            let x = Vector3::new(o.x[0], o.x[1], o.x[2]);
            let world = Frame3 {
                origin: Point3::new(o.point[0], o.point[1], o.point[2]),
                x,
                y: z.cross(&x),
                z,
            };
            let inv = placed[a.instance].inverse();
            Frame3 {
                origin: inv.apply_point(&world.origin),
                x: inv.apply_vector(&world.x),
                y: inv.apply_vector(&world.y),
                z: inv.apply_vector(&world.z),
            }
        };
        // the child's frame points the other way, so that two outward faces meet
        let fp = frame(parent, false);
        let fc = frame(child, true);
        let n = out.joints.len() + 1;
        let name = format!("{} {n}", self.kind.name());
        let pf = out
            .add_frame_to_instance(parent.instance, &format!("{name} origin"), &fp)
            .map_err(|e| e.to_string())?;
        let cf = out
            .add_frame_to_instance(child.instance, &format!("{name} origin"), &fc)
            .map_err(|e| e.to_string())?;
        let mut j = Joint::new(
            &name,
            self.kind,
            JointEnd {
                instance: parent.instance,
                frame: pf,
            },
            JointEnd {
                instance: child.instance,
                frame: cf,
            },
        );
        j.limits = self.limits()?;
        j.flip = self.flip;
        j.offset = [
            number(&self.offset[0], Dim::LENGTH)?,
            number(&self.offset[1], Dim::LENGTH)?,
            number(&self.offset[2], Dim::LENGTH)?,
        ];
        j.angle = number(&self.angle, Dim::ANGLE)?;
        out.add_joint(j);
        out.solve().map_err(|e| e.to_string())?;
        Ok(out)
    }
}

/// A number from a field: a plain number is mm / degrees, anything else an expression of units.
pub fn number(text: &str, dim: Dim) -> Result<f64, String> {
    let t = text.trim();
    if t.is_empty() {
        return Ok(0.0);
    }
    let p = parse_param(t, dim)?;
    let q = p
        .eval(&HashMap::<String, kw_expr::Quantity>::new())
        .map_err(|e| e.to_string())?;
    q.expect(dim).map_err(|e| e.to_string())
}

impl DesignWorkspace {
    // --------------------------------------------------------------------------------- opening

    /// Start the Joint (or As-Built Joint) command; the view shows the assembly so its components can be
    /// clicked.
    pub fn asm_open_joint(&mut self, project: &Project, as_built: bool) -> Result<(), String> {
        let asm = project.assembly.as_ref().ok_or("add components first")?;
        if asm.instances.len() < 2 {
            return Err("a joint needs two components".into());
        }
        self.asm.dialog = Some(JointDialog::new(as_built));
        self.asm.view = true;
        self.status = if as_built {
            "As-Built Joint: click a point on the first component, then on the second.".into()
        } else {
            "Joint: click the joint origin on the first component, then on the second.".into()
        };
        Ok(())
    }

    // -------------------------------------------------------------------------------- picking

    /// Which top-level instance (and body of the pose) a picked face or edge of the posed view belongs to.
    pub fn asm_owner(&self, pick: Pick) -> Option<(usize, usize)> {
        let (top, body) = self.asm_pick_owner(pick)?;
        Some((top, body))
    }

    /// Clicks in the posed view: select components, or feed the joint command.
    pub fn asm_view_clicks(&mut self, ui: &mut egui::Ui, project: &mut Project, out: &ViewOut) {
        let shift = ui.ctx().input(|i| i.modifiers.shift || i.modifiers.command);
        self.asm.view_hover = out.hover.and_then(|p| self.asm_owner(p)).map(|o| o.0);
        if let Some(Some(pick)) = &out.double_click {
            if self.asm.dialog.is_none() {
                if let Some((top, _)) = self.asm_owner(*pick) {
                    let part = project
                        .assembly
                        .as_ref()
                        .and_then(|a| a.instances.get(top))
                        .map(|i| i.part);
                    let is_sub = project
                        .assembly
                        .as_ref()
                        .and_then(|a| part.and_then(|p| a.parts.get(p)))
                        .is_some_and(|p| p.sub.is_some());
                    if let (Some(p), false) = (part, is_sub) {
                        self.actions.push(Action::ActivateComponent(p));
                        return;
                    }
                }
            }
        }
        let Some(click) = &out.click else {
            return;
        };
        match click {
            Some(pick) => {
                let Some((top, _)) = self.asm_owner(*pick) else {
                    return;
                };
                if self.asm.dialog.is_some() {
                    self.asm_dialog_pick(*pick, top, out.click_pos);
                } else {
                    self.actions
                        .push(Action::Asm(AsmAction::Select(top, shift)));
                }
            }
            None => {
                if !shift && self.asm.dialog.is_none() {
                    self.asm.selected.clear();
                    self.asm.selected_joint = None;
                }
            }
        }
    }

    /// A click on geometry while the Joint command is open.
    pub fn asm_dialog_pick(&mut self, pick: Pick, top: usize, click_pos: Option<egui::Pos2>) {
        let Some(rep) = self.asm.display_rep() else {
            return;
        };
        let cands = asm_snap::candidates(&rep, pick);
        // the candidate nearest the pointer on screen, within a few pixels of it
        let target = click_pos.map(|p| {
            let o = self.viewport.origin_in_scene();
            let mut best: Option<(f32, usize)> = None;
            for (i, c) in cands.iter().enumerate() {
                if let Some(s) =
                    self.viewport
                        .project([c.point[0] + o.x, c.point[1] + o.y, c.point[2] + o.z])
                {
                    let d = (s - p).length();
                    if best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, i));
                    }
                }
            }
            best
        });
        let origin = match target.flatten() {
            Some((d, i)) if d <= 14.0 => Some(cands[i].clone()),
            _ => asm_snap::choose(&cands, None, 0.0),
        };
        let Some(origin) = origin else {
            return;
        };
        if let Some(dlg) = self.asm.dialog.as_mut() {
            dlg.set_anchor(Anchor {
                instance: top,
                origin,
            });
        }
    }

    // ----------------------------------------------------------------------------- the dialog

    pub fn asm_joint_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(mut dlg) = self.asm.dialog.take() else {
            return;
        };
        let Some(asm) = project.assembly.as_ref() else {
            return;
        };
        let names: Vec<String> = asm.instances.iter().map(|i| i.name.clone()).collect();
        let mut close = false;
        let mut apply = false;
        egui::Window::new(
            egui::RichText::new(if dlg.as_built {
                "AS-BUILT JOINT"
            } else {
                "JOINT"
            })
            .size(11.5)
            .strong(),
        )
        .anchor(egui::Align2::RIGHT_TOP, [-12.0, 312.0])
        .default_width(260.0)
        .collapsible(false)
        .show(ctx, |ui| {
            let snap_line = |ui: &mut egui::Ui, n: u8, a: &Option<Anchor>, active: bool| -> bool {
                let mut clicked = false;
                ui.horizontal(|ui| {
                    let t = match a {
                        Some(a) => format!(
                            "Snap {n}: {} ({})",
                            names.get(a.instance).map_or("?", |s| s.as_str()),
                            a.origin.what
                        ),
                        None => format!("Snap {n}: click on a component"),
                    };
                    let mut r = egui::RichText::new(t).size(11.5);
                    if active {
                        r = r.strong().color(ACCENT);
                    }
                    clicked = ui.selectable_label(active, r).clicked();
                });
                clicked
            };
            if snap_line(ui, 1, &dlg.first, dlg.active == 0) {
                dlg.active = 0;
            }
            if snap_line(ui, 2, &dlg.second, dlg.active == 1) {
                dlg.active = 1;
            }
            ui.separator();
            let before = dlg.kind;
            egui::ComboBox::from_label("Type")
                .selected_text(dlg.kind.name())
                .show_ui(ui, |ui| {
                    for k in JointKind::ALL {
                        ui.selectable_value(&mut dlg.kind, k, k.name());
                    }
                });
            if before != dlg.kind {
                dlg.limit_on = [false; 3];
                dlg.dirty = true;
            }
            if dlg.as_built {
                ui.horizontal(|ui| {
                    ui.label("Axis");
                    for (i, t) in ["Picked", "X", "Y", "Z"].iter().enumerate() {
                        if ui.selectable_label(dlg.axis == i, *t).clicked() {
                            dlg.axis = i;
                            dlg.dirty = true;
                        }
                    }
                });
            } else {
                if ui.checkbox(&mut dlg.flip, "Flip").changed() {
                    dlg.dirty = true;
                }
                ui.label(
                    egui::RichText::new("Offset (mm) and angle")
                        .size(11.0)
                        .color(TEXT_WEAK),
                );
                ui.horizontal(|ui| {
                    for (i, l) in ["X", "Y", "Z"].iter().enumerate() {
                        ui.label(*l);
                        if ui
                            .add(egui::TextEdit::singleline(&mut dlg.offset[i]).desired_width(48.0))
                            .changed()
                        {
                            dlg.dirty = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Angle");
                    if ui
                        .add(egui::TextEdit::singleline(&mut dlg.angle).desired_width(60.0))
                        .changed()
                    {
                        dlg.dirty = true;
                    }
                });
            }
            let dims = dlg.kind.dims();
            if !dims.is_empty() {
                ui.separator();
                ui.label(
                    egui::RichText::new("Motion limits")
                        .size(11.0)
                        .color(TEXT_WEAK),
                );
                for (i, d) in dims.iter().enumerate() {
                    let unit = if *d == Dim::ANGLE { "°" } else { "mm" };
                    ui.horizontal(|ui| {
                        if ui
                            .checkbox(&mut dlg.limit_on[i], format!("#{} ", i + 1))
                            .changed()
                        {
                            dlg.dirty = true;
                        }
                        ui.add_enabled_ui(dlg.limit_on[i], |ui| {
                            for (field, hint) in
                                [(&mut dlg.limit_lo[i], "min"), (&mut dlg.limit_hi[i], "max")]
                            {
                                if ui
                                    .add(
                                        egui::TextEdit::singleline(field)
                                            .hint_text(hint)
                                            .desired_width(52.0),
                                    )
                                    .changed()
                                {
                                    dlg.dirty = true;
                                }
                            }
                            ui.label(unit);
                        });
                    });
                }
            }
            ui.separator();
            // the preview, rebuilt when something changed
            if dlg.dirty {
                dlg.dirty = false;
                dlg.error.clear();
                dlg.preview = None;
                if dlg.first.is_some() && dlg.second.is_some() && !dlg.as_built {
                    match dlg.build(asm) {
                        Ok(a) => dlg.preview = Some(a),
                        Err(e) => dlg.error = e,
                    }
                }
            }
            if !dlg.error.is_empty() {
                ui.label(egui::RichText::new(&dlg.error).size(11.0).color(BAD));
            }
            ui.horizontal(|ui| {
                let ready = dlg.first.is_some() && dlg.second.is_some();
                if ui.add_enabled(ready, egui::Button::new("OK")).clicked() {
                    apply = true;
                }
                if ui.button("Cancel").clicked() {
                    close = true;
                }
            });
        });
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            close = true;
        }
        if apply {
            match self.asm_joint_commit(project, &dlg) {
                Ok(()) => return,
                Err(e) => dlg.error = e,
            }
        }
        if close {
            self.status.clear();
            return;
        }
        self.asm.dialog = Some(dlg);
    }

    /// Make the joint of the dialog (OK).
    pub fn asm_joint_commit(
        &mut self,
        project: &mut Project,
        dlg: &JointDialog,
    ) -> Result<(), String> {
        let asm = project.assembly.as_ref().ok_or("there is no assembly")?;
        let a = dlg.build(asm)?;
        let name = a.joints.last().map(|j| j.name.clone()).unwrap_or_default();
        self.asm.selected_joint = Some(a.joints.len() - 1);
        self.asm.selected.clear();
        project.asm_checkpoint();
        project.assembly = Some(a);
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        self.asm.dialog = None;
        self.status = format!("{name} made.");
        Ok(())
    }

    /// Markers for the snaps of the open Joint command and for the selected joint.
    pub(super) fn draw_joint_markers(
        &self,
        painter: &egui::Painter,
        project: &Project,
        scene_origin: [f64; 3],
        size: f64,
    ) {
        let at = |p: [f64; 3]| {
            self.viewport.project([
                p[0] + scene_origin[0],
                p[1] + scene_origin[1],
                p[2] + scene_origin[2],
            ])
        };
        let triad = |o: &Origin, label: &str, colour: Color32| {
            let tip = |d: [f64; 3], k: f64| {
                at([
                    o.point[0] + d[0] * k,
                    o.point[1] + d[1] * k,
                    o.point[2] + d[2] * k,
                ])
            };
            let Some(c) = at(o.point) else {
                return;
            };
            painter.circle_filled(c, 4.5, colour);
            if let Some(t) = tip(o.z, size) {
                painter.line_segment([c, t], Stroke::new(2.0, Color32::from_rgb(40, 90, 220)));
            }
            if let Some(t) = tip(o.x, size * 0.6) {
                painter.line_segment([c, t], Stroke::new(2.0, Color32::from_rgb(220, 60, 50)));
            }
            painter.text(
                c + egui::vec2(8.0, -8.0),
                egui::Align2::LEFT_BOTTOM,
                label,
                egui::FontId::proportional(11.0),
                colour,
            );
        };
        if let Some(dlg) = &self.asm.dialog {
            match (&dlg.first, &dlg.second) {
                // with the joint previewed the two snaps coincide: the first component has moved to the second
                (Some(_), Some(b)) if dlg.preview.is_some() => {
                    triad(&b.origin, "1 = 2", Color32::from_rgb(214, 150, 40));
                }
                (a, b) => {
                    if let Some(a) = a {
                        triad(&a.origin, "1", ACCENT);
                    }
                    if let Some(b) = b {
                        triad(&b.origin, "2", Color32::from_rgb(214, 150, 40));
                    }
                }
            }
        }
        // every joint's origin as a small dot, the selected one with its axes
        if let Some(asm) = project.assembly.as_ref() {
            if let Ok(placed) = asm.solve() {
                for j in 0..asm.joints.len() {
                    let Some((o, name)) = joint_origin_in(asm, &placed, j) else {
                        continue;
                    };
                    if self.asm.selected_joint == Some(j) {
                        triad(&o, &name, ACCENT);
                    } else if let Some(c) = at(o.point) {
                        painter.circle_filled(c, 3.0, Color32::from_rgb(214, 150, 40));
                        painter.circle_stroke(c, 3.0, Stroke::new(1.0, Color32::WHITE));
                    }
                }
            }
        }
    }
}

/// The world position and axes of a joint (its parent frame).
pub fn joint_world_origin(asm: &Assembly, j: usize) -> Option<(Origin, String)> {
    let placed = asm.solve().ok()?;
    joint_origin_in(asm, &placed, j)
}

/// As [`joint_world_origin`] for placements already solved.
pub fn joint_origin_in(
    asm: &Assembly,
    placed: &[Transform3],
    j: usize,
) -> Option<(Origin, String)> {
    let joint = asm.joints.get(j)?;
    let part = asm
        .parts
        .get(asm.instances.get(joint.parent.instance)?.part)?;
    let f = part.frames.iter().find(|f| f.name == joint.parent.frame)?;
    let t: &Transform3 = placed.get(joint.parent.instance)?;
    let p = t.apply_point(&Point3::new(f.origin[0], f.origin[1], f.origin[2]));
    let z = t.apply_vector(&Vector3::new(f.z[0], f.z[1], f.z[2]));
    let x = t.apply_vector(&Vector3::new(f.x[0], f.x[1], f.x[2]));
    Some((
        Origin {
            point: [p.x, p.y, p.z],
            z: [z.x, z.y, z.z],
            x: [x.x, x.y, x.z],
            what: "joint",
        },
        joint.name.clone(),
    ))
}

// ---------------------------------------------------------------------- joint properties window

/// Text boxes of a joint's values and limits are kept between frames.
#[derive(Debug, Clone, Default)]
pub struct JointEdit {
    pub joint: usize,
    pub values: Vec<String>,
    pub offset: [String; 3],
    pub angle: String,
    pub limit_on: [bool; 3],
    pub limit_lo: [String; 3],
    pub limit_hi: [String; 3],
    pub error: String,
}

impl JointEdit {
    pub fn load(asm: &Assembly, j: usize) -> JointEdit {
        let joint = &asm.joints[j];
        let dims = joint.kind.dims();
        let mut e = JointEdit {
            joint: j,
            values: joint.values.iter().map(param_text).collect(),
            offset: [
                fmt(joint.offset[0]),
                fmt(joint.offset[1]),
                fmt(joint.offset[2]),
            ],
            angle: fmt(joint.angle.to_degrees()),
            ..Default::default()
        };
        for (i, d) in dims.iter().enumerate() {
            if let Some(Some((lo, hi))) = joint.limits.get(i) {
                let s = |v: f64| {
                    if *d == Dim::ANGLE {
                        fmt(v.to_degrees())
                    } else {
                        fmt(v)
                    }
                };
                e.limit_on[i] = true;
                e.limit_lo[i] = s(*lo);
                e.limit_hi[i] = s(*hi);
            } else {
                e.limit_lo[i] = "0".into();
                e.limit_hi[i] = if *d == Dim::ANGLE {
                    "180".into()
                } else {
                    "100".into()
                };
            }
        }
        e
    }

    /// Write the fields into the joint. Nothing changes when a field cannot be read.
    pub fn apply(&mut self, asm: &mut Assembly) -> Result<(), String> {
        let joint = asm.joints.get(self.joint).ok_or("the joint is gone")?;
        let dims = joint.kind.dims();
        let mut values = Vec::new();
        for (t, d) in self.values.iter().zip(dims) {
            values.push(parse_param(t, *d)?);
        }
        let mut limits = Vec::new();
        for (i, d) in dims.iter().enumerate() {
            limits.push(if self.limit_on[i] {
                let lo = number(&self.limit_lo[i], *d)?;
                let hi = number(&self.limit_hi[i], *d)?;
                if lo > hi {
                    return Err("a limit's minimum is above its maximum".into());
                }
                Some((lo, hi))
            } else {
                None
            });
        }
        let offset = [
            number(&self.offset[0], Dim::LENGTH)?,
            number(&self.offset[1], Dim::LENGTH)?,
            number(&self.offset[2], Dim::LENGTH)?,
        ];
        let angle = number(&self.angle, Dim::ANGLE)?;
        let j = &mut asm.joints[self.joint];
        j.values = values;
        j.limits = limits;
        j.offset = offset;
        j.angle = angle;
        Ok(())
    }
}

fn fmt(v: f64) -> String {
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-" {
        "0".into()
    } else {
        s.into()
    }
}

impl DesignWorkspace {
    /// The properties of the joint chosen in the tree: kind, value, limits, flip, offset and angle. Every
    /// change is applied at once (and taken back, with the reason, when the assembly would not solve).
    pub fn asm_joint_props_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(j) = self.asm.edit_joint else {
            return;
        };
        let Some(asm) = project.assembly.as_ref() else {
            self.asm.edit_joint = None;
            return;
        };
        if j >= asm.joints.len() {
            self.asm.edit_joint = None;
            return;
        }
        let mut edit = match self.asm.edit.take() {
            Some(e) if e.joint == j => e,
            _ => JointEdit::load(asm, j),
        };
        let joint = &asm.joints[j];
        let (name, kind, flip, as_built, solved) = (
            joint.name.clone(),
            joint.kind,
            joint.flip,
            joint.as_built,
            joint.solved,
        );
        let pair = {
            let n = |i: usize| {
                asm.instances
                    .get(i)
                    .map_or("?", |x| x.name.as_str())
                    .to_string()
            };
            format!(
                "{} to {}",
                n(joint.parent.instance),
                n(joint.child.instance)
            )
        };
        let dims = kind.dims();
        let names = kind.value_names();
        let mut changed = false;
        let mut new_kind = None;
        let mut flip_now = flip;
        let mut solved_now = solved;
        let mut close = false;
        let mut delete = false;
        let mut open = true;
        egui::Window::new(
            egui::RichText::new(format!("JOINT {name}"))
                .size(11.5)
                .strong(),
        )
        .open(&mut open)
        .default_pos(egui::pos2(330.0, 190.0))
        .default_width(300.0)
        .show(ctx, |ui| {
            ui.label(
                egui::RichText::new(format!(
                    "{pair}{}",
                    if as_built { "  (as-built)" } else { "" }
                ))
                .size(11.0)
                .color(TEXT_WEAK),
            );
            let mut k = kind;
            egui::ComboBox::from_label("Type")
                .selected_text(k.name())
                .show_ui(ui, |ui| {
                    for c in JointKind::ALL {
                        ui.selectable_value(&mut k, c, c.name());
                    }
                });
            if k != kind {
                new_kind = Some(k);
            }
            if ui.checkbox(&mut flip_now, "Flip").changed() {
                changed = true;
            }
            if kind.arity() > 0
                && ui
                    .checkbox(&mut solved_now, "Solved by a loop")
                    .on_hover_text(
                        "Its values are found so that the kinematic loops close, instead of being set by hand. Leave one joint of every loop driven: that is the input.",
                    )
                    .changed()
            {
                changed = true;
            }
            ui.horizontal(|ui| {
                ui.label("Offset");
                for (i, l) in ["X", "Y", "Z"].iter().enumerate() {
                    ui.label(*l);
                    if ui
                        .add(egui::TextEdit::singleline(&mut edit.offset[i]).desired_width(44.0))
                        .lost_focus()
                    {
                        changed = true;
                    }
                }
                ui.label("Angle");
                if ui
                    .add(egui::TextEdit::singleline(&mut edit.angle).desired_width(44.0))
                    .lost_focus()
                {
                    changed = true;
                }
            });
            if !dims.is_empty() {
                ui.separator();
            }
            for (i, d) in dims.iter().enumerate() {
                let angle = *d == Dim::ANGLE;
                let unit = if angle { "°" } else { "mm" };
                ui.label(egui::RichText::new(names[i]).strong().size(11.5));
                ui.horizontal(|ui| {
                    let r =
                        ui.add(egui::TextEdit::singleline(&mut edit.values[i]).desired_width(80.0));
                    if r.lost_focus() {
                        changed = true;
                    }
                    // a slider over the limits (or a sensible span)
                    let (lo, hi) = if edit.limit_on[i] {
                        (
                            edit.limit_lo[i].trim().parse::<f64>().unwrap_or(-180.0),
                            edit.limit_hi[i].trim().parse::<f64>().unwrap_or(180.0),
                        )
                    } else if angle {
                        (-180.0, 180.0)
                    } else {
                        (-100.0, 100.0)
                    };
                    if let Ok(mut v) = edit.values[i].trim().parse::<f64>() {
                        if hi > lo
                            && ui
                                .add(egui::Slider::new(&mut v, lo..=hi).show_value(false))
                                .changed()
                        {
                            edit.values[i] = fmt(v);
                            changed = true;
                        }
                    }
                    ui.label(unit);
                });
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut edit.limit_on[i], "Limit").changed() {
                        changed = true;
                    }
                    ui.add_enabled_ui(edit.limit_on[i], |ui| {
                        for (field, hint) in [
                            (&mut edit.limit_lo[i], "min"),
                            (&mut edit.limit_hi[i], "max"),
                        ] {
                            if ui
                                .add(
                                    egui::TextEdit::singleline(field)
                                        .hint_text(hint)
                                        .desired_width(56.0),
                                )
                                .lost_focus()
                            {
                                changed = true;
                            }
                        }
                        ui.label(unit);
                    });
                });
            }
            if !edit.error.is_empty() {
                ui.label(egui::RichText::new(&edit.error).size(11.0).color(BAD));
            }
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Delete joint").clicked() {
                    delete = true;
                }
                if ui.button("Close").clicked() {
                    close = true;
                }
            });
        });
        if !open || close {
            self.asm.edit_joint = None;
            self.asm.edit = None;
            return;
        }
        if delete {
            self.asm.edit = None;
            self.actions.push(Action::Asm(AsmAction::DeleteJoint(j)));
            return;
        }
        if new_kind.is_some() || flip_now != flip || solved_now != solved || changed {
            let asm = project.assembly.as_mut().unwrap();
            let backup = asm.joints[j].clone();
            if let Some(k) = new_kind {
                let jt = &mut asm.joints[j];
                jt.kind = k;
                jt.values = kw_assembly::zero_values(k);
                jt.limits = vec![None; k.arity()];
            }
            asm.joints[j].flip = flip_now;
            asm.joints[j].solved = solved_now;
            let result = if new_kind.is_some() {
                Ok(())
            } else {
                edit.apply(asm)
            }
            // the joints of the loops follow the value that was set (and a change that leaves a loop open is refused)
            .and_then(|_| asm.settle().map(|_| ()).map_err(|e| e.to_string()));
            match result {
                Ok(()) => {
                    edit.error.clear();
                    self.asm_commit_snapshot(project);
                    project.mark_extras_dirty();
                    self.asm.overlaps = None;
                    if new_kind.is_some() {
                        edit = JointEdit::load(project.assembly.as_ref().unwrap(), j);
                    }
                }
                Err(e) => {
                    asm.joints[j] = backup;
                    edit.error = e;
                }
            }
        }
        self.asm.edit = Some(edit);
    }
}
