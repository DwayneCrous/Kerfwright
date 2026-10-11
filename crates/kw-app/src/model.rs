//! The Model workspace: the feature timeline, an inspector for the selected feature, the toolbar that adds
//! features, and the 3-D viewport.

use crate::project::Project;
use crate::viewport::{Pick, Viewport3d};
use eframe::egui;
use kw_expr::{Dim, Param};
use kw_feature::{BodyOp, Document, Evaluation, FeatureId, FeatureSpec, FeatureState, PlaneSpec};
use std::collections::HashMap;

/// Text of a parameter as the user would type it.
pub fn param_text(p: &Param) -> String {
    let trim = |v: f64| {
        let s = format!("{v:.6}");
        let s = s.trim_end_matches('0').trim_end_matches('.');
        if s.is_empty() || s == "-" {
            "0".to_string()
        } else {
            s.to_string()
        }
    };
    match p {
        Param::Lit(q) if q.dim == Dim::LENGTH => format!("{} mm", trim(q.value)),
        Param::Lit(q) if q.dim == Dim::ANGLE => format!("{} deg", trim(q.value.to_degrees())),
        Param::Lit(q) => trim(q.value),
        Param::Expr(e) => e.source().to_string(),
    }
}

/// A plain number means millimetres / degrees / a count (by `dim`); anything else is an expression.
pub fn parse_param(text: &str, dim: Dim) -> Result<Param, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("enter a value".into());
    }
    if let Ok(v) = t.parse::<f64>() {
        return Ok(if dim == Dim::LENGTH {
            Param::length(v)
        } else if dim == Dim::ANGLE {
            Param::angle_deg(v)
        } else {
            Param::scalar(v)
        });
    }
    Param::expr(t).map_err(|e| e.to_string())
}

fn unique_name(doc: &Document, base: &str) -> String {
    (1..)
        .map(|n| format!("{base} {n}"))
        .find(|n| doc.find_by_name(n).is_none())
        .expect("a free name")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlaneChoice {
    Xy,
    Yz,
    Xz,
}

enum Dialog {
    RectSketch {
        plane: PlaneChoice,
        x0: String,
        y0: String,
        w: String,
        h: String,
    },
    CircleSketch {
        plane: PlaneChoice,
        cx: String,
        cy: String,
        d: String,
    },
    Extrude {
        sketch: Option<FeatureId>,
        depth: String,
        op: BodyOp,
    },
    Fillet {
        radius: String,
    },
    Pattern {
        source: Option<FeatureId>,
        dx: String,
        dy: String,
        dz: String,
        count: String,
    },
    Parameter {
        name: String,
        value: String,
    },
}

#[derive(Default)]
pub struct ModelWorkspace {
    pub viewport: Viewport3d,
    pub selected: Option<FeatureId>,
    /// Faces and edges picked in the viewport (the last one is highlighted).
    pub picks: Vec<Pick>,
    dialog: Option<Dialog>,
    pub error: String,
    bufs: HashMap<String, String>,
}

fn plane_spec(p: PlaneChoice) -> PlaneSpec {
    match p {
        PlaneChoice::Xy => PlaneSpec::Xy(Param::length(0.0)),
        PlaneChoice::Yz => PlaneSpec::Yz(Param::length(0.0)),
        PlaneChoice::Xz => PlaneSpec::Xz(Param::length(0.0)),
    }
}

fn plane_combo(ui: &mut egui::Ui, id: &str, p: &mut PlaneChoice) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(match p {
            PlaneChoice::Xy => "XY plane",
            PlaneChoice::Yz => "YZ plane",
            PlaneChoice::Xz => "XZ plane",
        })
        .show_ui(ui, |ui| {
            ui.selectable_value(p, PlaneChoice::Xy, "XY plane");
            ui.selectable_value(p, PlaneChoice::Yz, "YZ plane");
            ui.selectable_value(p, PlaneChoice::Xz, "XZ plane");
        });
}

fn field(ui: &mut egui::Ui, label: &str, text: &mut String) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::TextEdit::singleline(text).desired_width(90.0));
    });
}

impl ModelWorkspace {
    // ---------------------------------------------------------------------------------- edits

    fn edit<T>(
        &mut self,
        project: &mut Project,
        f: impl FnOnce(&mut Document) -> Result<T, kw_feature::TimelineError>,
    ) -> Option<T> {
        match project.history.edit(f) {
            Ok(v) => {
                self.error.clear();
                Some(v)
            }
            Err(e) => {
                self.error = e.to_string();
                None
            }
        }
    }

    pub fn add_rect_sketch(
        &mut self,
        project: &mut Project,
        plane: &str,
        x0: &str,
        y0: &str,
        w: &str,
        h: &str,
    ) -> Option<FeatureId> {
        let p = |t: &str| parse_param(t, Dim::LENGTH);
        let (x0, y0, w, h) = match (p(x0), p(y0), p(w), p(h)) {
            (Ok(a), Ok(b), Ok(c), Ok(d)) => (a, b, c, d),
            (a, b, c, d) => {
                self.error = [a.err(), b.err(), c.err(), d.err()]
                    .into_iter()
                    .flatten()
                    .next()
                    .unwrap_or_default();
                return None;
            }
        };
        let plane = match plane {
            "yz" => PlaneChoice::Yz,
            "xz" => PlaneChoice::Xz,
            _ => PlaneChoice::Xy,
        };
        let name = unique_name(&project.document(), "Sketch");
        let id = self.edit(project, |d| {
            d.add_rect_sketch(&name, plane_spec(plane), x0, y0, w, h)
        })?;
        self.selected = Some(id);
        Some(id)
    }

    pub fn add_extrude(
        &mut self,
        project: &mut Project,
        sketch: FeatureId,
        depth: &str,
        op: BodyOp,
    ) -> Option<FeatureId> {
        let depth = match parse_param(depth, Dim::LENGTH) {
            Ok(d) => d,
            Err(e) => {
                self.error = e;
                return None;
            }
        };
        let name = unique_name(&project.document(), "Extrude");
        let id = self.edit(project, |d| d.add_extrude(&name, sketch, depth, op))?;
        self.selected = Some(id);
        Some(id)
    }

    pub fn add_parameter(
        &mut self,
        project: &mut Project,
        name: &str,
        value: &str,
    ) -> Option<FeatureId> {
        let v = match parse_param(value, Dim::LENGTH) {
            Ok(v) => v,
            Err(e) => {
                self.error = e;
                return None;
            }
        };
        let name = name.trim().to_string();
        let id = self.edit(project, |d| d.add_parameter(&name, v))?;
        self.selected = Some(id);
        Some(id)
    }

    /// Fillet the picked edges. Each edge needs the two faces it joins, named so the fillet survives edits.
    pub fn add_fillet(&mut self, project: &mut Project, radius: &str) -> Option<FeatureId> {
        let r = match parse_param(radius, Dim::LENGTH) {
            Ok(r) => r,
            Err(e) => {
                self.error = e;
                return None;
            }
        };
        let ev = project.evaluation();
        let Some((_, out)) = ev.final_solid() else {
            self.error = "there is no solid to fillet".into();
            return None;
        };
        let (Some(brep), names) = (out.brep(), &out.names) else {
            return None;
        };
        let mut edges = Vec::new();
        for p in &self.picks {
            if let Pick::Edge(e) = p {
                let faces = brep.faces_around_edge(*e);
                let refs: Vec<_> = faces
                    .iter()
                    .filter_map(|f| names.reference_to(brep, *f))
                    .collect();
                if refs.len() == 2 {
                    edges.push([refs[0].clone(), refs[1].clone()]);
                } else {
                    self.error = "an edge must join exactly two named faces".into();
                    return None;
                }
            }
        }
        if edges.is_empty() {
            self.error =
                "pick one or more edges first (click edges in the view, Shift adds)".into();
            return None;
        }
        let name = unique_name(&project.document(), "Fillet");
        let id = self.edit(project, |d| d.add_fillet(&name, edges, r))?;
        self.picks.clear();
        self.selected = Some(id);
        Some(id)
    }

    pub fn add_pattern(
        &mut self,
        project: &mut Project,
        source: FeatureId,
        spacing: [&str; 3],
        count: &str,
    ) -> Option<FeatureId> {
        let sp: Result<Vec<Param>, String> = spacing
            .iter()
            .map(|t| parse_param(t, Dim::LENGTH))
            .collect();
        let (sp, c) = match (sp, parse_param(count, Dim::NONE)) {
            (Ok(s), Ok(c)) => (s, c),
            (Err(e), _) | (_, Err(e)) => {
                self.error = e;
                return None;
            }
        };
        let name = unique_name(&project.document(), "Pattern");
        let id = self.edit(project, |d| {
            d.add_linear_pattern(
                &name,
                source,
                [sp[0].clone(), sp[1].clone(), sp[2].clone()],
                c,
            )
        })?;
        self.selected = Some(id);
        Some(id)
    }

    // ------------------------------------------------------------------------------------ UI

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        let ev = project.evaluation();
        self.toolbar(ui, project, &ev);
        egui::Panel::left("timeline")
            .default_size(300.0)
            .show(ui, |ui| self.timeline(ui, project, &ev));
        egui::CentralPanel::default().show(ui, |ui| {
            let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
            let key = ev
                .final_hashes()
                .map(|h| h.0 ^ h.1.rotate_left(1))
                .unwrap_or(0);
            let shift = ui.ctx().input(|i| i.modifiers.shift);
            let highlight = self.picks.last().copied();
            if let Some(hit) = self.viewport.show(ui, rs, key, brep.as_ref(), highlight) {
                match hit {
                    Some(p) => {
                        if shift {
                            if let Some(i) = self.picks.iter().position(|q| *q == p) {
                                self.picks.remove(i);
                            } else {
                                self.picks.push(p);
                            }
                        } else {
                            self.picks = vec![p];
                        }
                    }
                    None => {
                        if !shift {
                            self.picks.clear();
                        }
                    }
                }
            }
        });
        self.dialogs(ui.ctx(), project);
    }

    fn toolbar(&mut self, ui: &mut egui::Ui, project: &mut Project, ev: &Evaluation) {
        egui::Panel::top("model_tools").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Rectangle sketch").clicked() {
                    self.dialog = Some(Dialog::RectSketch {
                        plane: PlaneChoice::Xy,
                        x0: "0".into(),
                        y0: "0".into(),
                        w: "40".into(),
                        h: "25".into(),
                    });
                }
                if ui.button("Circle sketch").clicked() {
                    self.dialog = Some(Dialog::CircleSketch {
                        plane: PlaneChoice::Xy,
                        cx: "0".into(),
                        cy: "0".into(),
                        d: "20".into(),
                    });
                }
                ui.separator();
                let doc = project.document();
                let sketch = doc
                    .features
                    .iter()
                    .rev()
                    .find(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                    .map(|f| f.id);
                if ui
                    .add_enabled(sketch.is_some(), egui::Button::new("Extrude"))
                    .clicked()
                {
                    let sel = self
                        .selected
                        .filter(|s| {
                            doc.get(*s)
                                .is_some_and(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                        })
                        .or(sketch);
                    self.dialog = Some(Dialog::Extrude {
                        sketch: sel,
                        depth: "10".into(),
                        op: if ev.final_solid().is_some() {
                            BodyOp::Join
                        } else {
                            BodyOp::New
                        },
                    });
                }
                let edges = self
                    .picks
                    .iter()
                    .filter(|p| matches!(p, Pick::Edge(_)))
                    .count();
                if ui
                    .add_enabled(
                        edges > 0,
                        egui::Button::new(format!(
                            "Fillet ({edges} edge{})",
                            if edges == 1 { "" } else { "s" }
                        )),
                    )
                    .on_hover_text("Click edges in the 3-D view (Shift adds more)")
                    .clicked()
                {
                    self.dialog = Some(Dialog::Fillet { radius: "2".into() });
                }
                if ui
                    .add_enabled(!doc.features.is_empty(), egui::Button::new("Pattern"))
                    .clicked()
                {
                    let src = self.selected.or(doc.features.last().map(|f| f.id));
                    self.dialog = Some(Dialog::Pattern {
                        source: src,
                        dx: "50".into(),
                        dy: "0".into(),
                        dz: "0".into(),
                        count: "3".into(),
                    });
                }
                if ui.button("Parameter").clicked() {
                    self.dialog = Some(Dialog::Parameter {
                        name: "width".into(),
                        value: "20".into(),
                    });
                }
                ui.separator();
                if doc.rollback.is_some() && ui.button("Roll forward").clicked() {
                    self.edit(project, |d| {
                        d.clear_rollback();
                        Ok(())
                    });
                }
            });
            if !self.error.is_empty() {
                ui.colored_label(egui::Color32::from_rgb(230, 110, 90), &self.error);
            }
        });
    }

    fn timeline(&mut self, ui: &mut egui::Ui, project: &mut Project, ev: &Evaluation) {
        ui.heading("Features");
        let doc = project.document();
        let mut remove: Option<FeatureId> = None;
        let mut suppress: Option<(FeatureId, bool)> = None;
        let mut rollback: Option<FeatureId> = None;
        egui::ScrollArea::vertical()
            .max_height(ui.available_height() * 0.55)
            .show(ui, |ui| {
                if doc.features.is_empty() {
                    ui.weak("No features yet. Add a sketch, then extrude it.");
                }
                for (i, f) in doc.features.iter().enumerate() {
                    let beyond = doc.rollback.is_some_and(|r| i >= r);
                    ui.horizontal(|ui| {
                        let mut on = !f.suppressed;
                        if ui
                            .checkbox(&mut on, "")
                            .on_hover_text("Suppress / unsuppress")
                            .changed()
                        {
                            suppress = Some((f.id, !on));
                        }
                        let (mark, color) = match ev.state(f.id) {
                            Some(FeatureState::Ok(_)) => {
                                ("OK", egui::Color32::from_rgb(110, 190, 120))
                            }
                            Some(FeatureState::Failed { .. }) => {
                                ("ERR", egui::Color32::from_rgb(230, 100, 90))
                            }
                            _ => ("--", egui::Color32::GRAY),
                        };
                        let tip = ev.error(f.id).map(|e| e.to_string()).unwrap_or_default();
                        ui.colored_label(color, mark).on_hover_text(tip);
                        let mut text =
                            egui::RichText::new(format!("{}  {}", f.spec.kind_name(), f.name));
                        if f.suppressed || beyond {
                            text = text.weak().strikethrough();
                        }
                        if ui
                            .selectable_label(self.selected == Some(f.id), text)
                            .clicked()
                        {
                            self.selected = Some(f.id);
                        }
                        if ui
                            .small_button("Stop here")
                            .on_hover_text("Roll back to this feature")
                            .clicked()
                        {
                            rollback = Some(f.id);
                        }
                        if ui.small_button("Delete").clicked() {
                            remove = Some(f.id);
                        }
                    });
                }
            });
        if let Some((id, s)) = suppress {
            self.edit(project, |d| d.set_suppressed(id, s));
        }
        if let Some(id) = rollback {
            self.edit(project, |d| d.rollback_to(id));
        }
        if let Some(id) = remove {
            if self.edit(project, |d| d.remove(id).map(|_| ())).is_some()
                && self.selected == Some(id)
            {
                self.selected = None;
            }
        }
        ui.separator();
        self.inspector(ui, project, ev);
    }

    fn inspector(&mut self, ui: &mut egui::Ui, project: &mut Project, ev: &Evaluation) {
        ui.heading("Properties");
        let doc = project.document();
        let Some(f) = self.selected.and_then(|id| doc.get(id)).cloned() else {
            ui.weak("Select a feature.");
            return;
        };
        ui.strong(format!("{} — {}", f.spec.kind_name(), f.name));
        if let Some(e) = ev.error(f.id) {
            ui.colored_label(egui::Color32::from_rgb(230, 110, 90), e.to_string());
        }
        let mut new_spec: Option<FeatureSpec> = None;
        match &f.spec {
            FeatureSpec::Parameter { name, value } => {
                if let Some(p) = self.param_field(ui, f.id, "value", "Value", value, Dim::LENGTH) {
                    new_spec = Some(FeatureSpec::Parameter {
                        name: name.clone(),
                        value: p,
                    });
                }
            }
            FeatureSpec::Extrude(x) => {
                let mut x = x.clone();
                let mut changed = false;
                if let Some(p) = self.param_field(ui, f.id, "depth", "Depth", &x.depth, Dim::LENGTH)
                {
                    x.depth = p;
                    changed = true;
                }
                changed |= ui.checkbox(&mut x.symmetric, "Symmetric").changed();
                changed |= ui.checkbox(&mut x.flip, "Flip direction").changed();
                egui::ComboBox::from_id_salt("extrude_op")
                    .selected_text(format!("{:?}", x.op))
                    .show_ui(ui, |ui| {
                        for op in [BodyOp::New, BodyOp::Join, BodyOp::Cut, BodyOp::Intersect] {
                            changed |= ui
                                .selectable_value(&mut x.op, op, format!("{op:?}"))
                                .changed();
                        }
                    });
                if changed {
                    new_spec = Some(FeatureSpec::Extrude(x));
                }
            }
            FeatureSpec::Fillet(x) => {
                if let Some(p) =
                    self.param_field(ui, f.id, "radius", "Radius", &x.radius, Dim::LENGTH)
                {
                    let mut x = x.clone();
                    x.radius = p;
                    new_spec = Some(FeatureSpec::Fillet(x));
                }
                ui.weak(format!("{} edge(s)", x.edges.len()));
            }
            FeatureSpec::LinearPattern(x) => {
                let mut x = x.clone();
                let mut changed = false;
                for (k, label) in ["Spacing X", "Spacing Y", "Spacing Z"].iter().enumerate() {
                    if let Some(p) =
                        self.param_field(ui, f.id, label, label, &x.spacing[k].clone(), Dim::LENGTH)
                    {
                        x.spacing[k] = p;
                        changed = true;
                    }
                }
                if let Some(p) =
                    self.param_field(ui, f.id, "count", "Count", &x.count.clone(), Dim::NONE)
                {
                    x.count = p;
                    changed = true;
                }
                if changed {
                    new_spec = Some(FeatureSpec::LinearPattern(x));
                }
            }
            FeatureSpec::Sketch(_) => {
                ui.weak("Edit sketch geometry in the Sketch workspace.");
            }
            other => {
                ui.weak(format!("{} has no editor here yet.", other.kind_name()));
            }
        }
        if let Some(spec) = new_spec {
            let id = f.id;
            self.edit(project, |d| d.replace_spec(id, spec));
        }
    }

    /// A text field for a parameter; returns the new value when the user commits a valid one.
    fn param_field(
        &mut self,
        ui: &mut egui::Ui,
        id: FeatureId,
        key: &str,
        label: &str,
        cur: &Param,
        dim: Dim,
    ) -> Option<Param> {
        let k = format!("{}:{}", id.0, key);
        let shown = param_text(cur);
        let mut text = self.bufs.get(&k).cloned().unwrap_or_else(|| shown.clone());
        let mut out = None;
        ui.horizontal(|ui| {
            ui.label(label);
            let r = ui.add(egui::TextEdit::singleline(&mut text).desired_width(120.0));
            if r.changed() {
                self.bufs.insert(k.clone(), text.clone());
            }
            if r.lost_focus() {
                self.bufs.remove(&k);
                if text.trim() != shown {
                    match parse_param(&text, dim) {
                        Ok(p) => out = Some(p),
                        Err(e) => self.error = e,
                    }
                }
            }
        });
        out
    }

    fn dialogs(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(mut d) = self.dialog.take() else {
            return;
        };
        let mut done = false;
        let mut create = false;
        let title = match &d {
            Dialog::RectSketch { .. } => "New rectangle sketch",
            Dialog::CircleSketch { .. } => "New circle sketch",
            Dialog::Extrude { .. } => "Extrude",
            Dialog::Fillet { .. } => "Fillet",
            Dialog::Pattern { .. } => "Linear pattern",
            Dialog::Parameter { .. } => "New parameter",
        };
        let doc = project.document();
        egui::Window::new(title)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                match &mut d {
                    Dialog::RectSketch {
                        plane,
                        x0,
                        y0,
                        w,
                        h,
                    } => {
                        plane_combo(ui, "rect_plane", plane);
                        field(ui, "Corner x", x0);
                        field(ui, "Corner y", y0);
                        field(ui, "Width", w);
                        field(ui, "Height", h);
                    }
                    Dialog::CircleSketch { plane, cx, cy, d } => {
                        plane_combo(ui, "circ_plane", plane);
                        field(ui, "Centre x", cx);
                        field(ui, "Centre y", cy);
                        field(ui, "Diameter", d);
                    }
                    Dialog::Extrude { sketch, depth, op } => {
                        egui::ComboBox::from_id_salt("ex_sketch")
                            .selected_text(
                                sketch
                                    .and_then(|s| doc.get(s))
                                    .map(|f| f.name.clone())
                                    .unwrap_or_else(|| "choose a sketch".into()),
                            )
                            .show_ui(ui, |ui| {
                                for f in doc
                                    .features
                                    .iter()
                                    .filter(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                                {
                                    ui.selectable_value(sketch, Some(f.id), &f.name);
                                }
                            });
                        field(ui, "Depth", depth);
                        egui::ComboBox::from_id_salt("ex_op")
                            .selected_text(format!("{op:?}"))
                            .show_ui(ui, |ui| {
                                for o in [BodyOp::New, BodyOp::Join, BodyOp::Cut, BodyOp::Intersect]
                                {
                                    ui.selectable_value(op, o, format!("{o:?}"));
                                }
                            });
                    }
                    Dialog::Fillet { radius } => field(ui, "Radius", radius),
                    Dialog::Pattern {
                        source,
                        dx,
                        dy,
                        dz,
                        count,
                    } => {
                        egui::ComboBox::from_id_salt("pat_src")
                            .selected_text(
                                source
                                    .and_then(|s| doc.get(s))
                                    .map(|f| f.name.clone())
                                    .unwrap_or_else(|| "choose a feature".into()),
                            )
                            .show_ui(ui, |ui| {
                                for f in doc
                                    .features
                                    .iter()
                                    .filter(|f| !matches!(f.spec, FeatureSpec::Parameter { .. }))
                                {
                                    ui.selectable_value(source, Some(f.id), &f.name);
                                }
                            });
                        field(ui, "Spacing X", dx);
                        field(ui, "Spacing Y", dy);
                        field(ui, "Spacing Z", dz);
                        field(ui, "Count", count);
                    }
                    Dialog::Parameter { name, value } => {
                        field(ui, "Name", name);
                        field(ui, "Value", value);
                    }
                }
                ui.horizontal(|ui| {
                    if ui.button("Create").clicked() {
                        create = true;
                    }
                    if ui.button("Cancel").clicked() {
                        done = true;
                    }
                });
            });
        if create {
            let ok = match &d {
                Dialog::RectSketch {
                    plane,
                    x0,
                    y0,
                    w,
                    h,
                } => {
                    let pl = match plane {
                        PlaneChoice::Xy => "xy",
                        PlaneChoice::Yz => "yz",
                        PlaneChoice::Xz => "xz",
                    };
                    self.add_rect_sketch(project, pl, x0, y0, w, h).is_some()
                }
                Dialog::CircleSketch { plane, cx, cy, d } => {
                    let p = |t: &str| parse_param(t, Dim::LENGTH);
                    match (p(cx), p(cy), p(d)) {
                        (Ok(cx), Ok(cy), Ok(dd)) => {
                            let name = unique_name(&project.document(), "Sketch");
                            let pl = plane_spec(*plane);
                            match self
                                .edit(project, |doc| doc.add_circle_sketch(&name, pl, cx, cy, dd))
                            {
                                Some(id) => {
                                    self.selected = Some(id);
                                    true
                                }
                                None => false,
                            }
                        }
                        (a, b, c) => {
                            self.error = [a.err(), b.err(), c.err()]
                                .into_iter()
                                .flatten()
                                .next()
                                .unwrap_or_default();
                            false
                        }
                    }
                }
                Dialog::Extrude { sketch, depth, op } => match sketch {
                    Some(s) => self.add_extrude(project, *s, depth, *op).is_some(),
                    None => {
                        self.error = "choose a sketch".into();
                        false
                    }
                },
                Dialog::Fillet { radius } => self.add_fillet(project, radius).is_some(),
                Dialog::Pattern {
                    source,
                    dx,
                    dy,
                    dz,
                    count,
                } => match source {
                    Some(s) => self.add_pattern(project, *s, [dx, dy, dz], count).is_some(),
                    None => {
                        self.error = "choose a feature".into();
                        false
                    }
                },
                Dialog::Parameter { name, value } => {
                    self.add_parameter(project, name, value).is_some()
                }
            };
            done = ok;
        }
        if !done {
            self.dialog = Some(d);
        }
    }
}
