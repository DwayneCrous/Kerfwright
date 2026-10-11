//! Things drawn over the 3-D view: origin planes, the view cube, the navigation bar, the Extrude drag arrow,
//! and the parameters window.

use super::tools::Tool;
use super::{geom, DesignWorkspace, Mode};
use crate::chrome::theme::{ACCENT, ACCENT_SOFT, BORDER, CHROME, TEXT};
use crate::chrome::{ribbon as rb, viewcube, Icon};
use crate::model::{param_text, parse_param};
use crate::project::Project;
use crate::viewport::ViewOut;
use eframe::egui::{self, pos2, vec2, Color32, Shape, Stroke, Ui};
use kw_expr::{Dim, Param};
use kw_feature::{Evaluation, FeatureSpec, Product};
use std::sync::Arc;

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

impl DesignWorkspace {
    pub(crate) fn overlay_ui(
        &mut self,
        ui: &mut Ui,
        project: &mut Project,
        ev: &Arc<Evaluation>,
        out: &ViewOut,
    ) {
        let rect = out.rect;
        let painter = ui.painter_at(rect);
        let picking = self.mode == Mode::PickPlane;
        let radius = self
            .viewport
            .model()
            .map(|m| m.scene.radius)
            .unwrap_or(40.0);
        let s = geom::plane_size(radius);
        let o = self.viewport.origin_in_scene();
        let o3 = [o.x, o.y, o.z];
        // ---- which origin plane is under the pointer (only while picking)
        self.hover_plane = None;
        if picking {
            if let Some(pos) = ui.ctx().pointer_hover_pos().filter(|p| rect.contains(*p)) {
                let (ro, rd) = self.viewport.ray(pos);
                let (ro, rd) = ([ro.x, ro.y, ro.z], [rd.x, rd.y, rd.z]);
                let mut best: Option<(f64, usize)> = None;
                for (k, (_, idx)) in geom::ORIGIN_PLANES.iter().enumerate() {
                    let f = geom::origin_frame(*idx);
                    let denom = dot(rd, f.n);
                    if denom.abs() < 1e-9 {
                        continue;
                    }
                    let t = dot([o3[0] - ro[0], o3[1] - ro[1], o3[2] - ro[2]], f.n) / denom;
                    if t <= 0.0 {
                        continue;
                    }
                    let hit = [
                        ro[0] + t * rd[0] - o3[0],
                        ro[1] + t * rd[1] - o3[1],
                        ro[2] + t * rd[2] - o3[2],
                    ];
                    if dot(hit, f.x).abs() <= s
                        && dot(hit, f.y).abs() <= s
                        && best.is_none_or(|b| t < b.0)
                    {
                        best = Some((t, k));
                    }
                }
                self.hover_plane = best.map(|b| b.1);
            }
        }
        // a hovered planar face takes precedence over a plane behind it
        if picking && matches!(out.hover, Some(crate::viewport::Pick::Face(_))) {
            self.hover_plane = None;
        }
        // ---- the planes
        for (k, (name, idx)) in geom::ORIGIN_PLANES.iter().enumerate() {
            if !(picking || self.plane_visible[k]) {
                continue;
            }
            let f = geom::origin_frame(*idx);
            let pts: Vec<egui::Pos2> = [(-s, -s), (s, -s), (s, s), (-s, s)]
                .iter()
                .filter_map(|(u, v)| {
                    let w = f.to_world(*u, *v);
                    self.viewport
                        .project([w[0] + o3[0], w[1] + o3[1], w[2] + o3[2]])
                })
                .collect();
            if pts.len() == 4 {
                let hot = self.hover_plane == Some(k);
                painter.add(Shape::convex_polygon(
                    pts.clone(),
                    Color32::from_rgba_unmultiplied(6, 150, 215, if hot { 110 } else { 38 }),
                    Stroke::new(if hot { 2.0 } else { 1.0 }, ACCENT),
                ));
                painter.text(
                    pts[2] + vec2(-4.0, -4.0),
                    egui::Align2::RIGHT_BOTTOM,
                    *name,
                    egui::FontId::proportional(11.0),
                    ACCENT,
                );
            }
        }
        // ---- view cube
        let cube =
            egui::Rect::from_min_size(rect.right_top() + vec2(-100.0, 12.0), vec2(88.0, 88.0));
        if let Some((yaw, pitch)) = viewcube::show(ui, cube, &self.viewport.camera) {
            let now = ui.ctx().input(|i| i.time);
            self.viewport.look(yaw, pitch, now);
        }
        // ---- navigation bar
        let nav = egui::Rect::from_center_size(
            pos2(rect.center().x, rect.bottom() - 22.0),
            vec2(176.0, 30.0),
        );
        ui.scope_builder(egui::UiBuilder::new().max_rect(nav), |ui| {
            egui::Frame::new()
                .fill(CHROME)
                .stroke(Stroke::new(1.0, BORDER))
                .corner_radius(4)
                .inner_margin(2)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 2.0;
                        if rb::small(ui, Icon::Home, "Home view", false).clicked() {
                            let now = ui.ctx().input(|i| i.time);
                            self.viewport.look(-0.8, 0.45, now);
                            self.viewport.fit();
                        }
                        if rb::small(ui, Icon::Fit, "Fit the model in the view", false).clicked() {
                            self.viewport.fit();
                        }
                        if rb::small(ui, Icon::Body, "Shaded / wireframe", !self.shaded).clicked() {
                            self.shaded = !self.shaded;
                        }
                        if rb::small(ui, Icon::Line, "Show edges", self.viewport.show_edges)
                            .clicked()
                        {
                            self.viewport.show_edges = !self.viewport.show_edges;
                        }
                        let any = self.plane_visible.iter().any(|v| *v);
                        if rb::small(ui, Icon::Plane, "Origin planes", any).clicked() {
                            self.plane_visible = [!any; 3];
                        }
                    });
                });
        });
        self.extrude_handle(ui, ev, out);
        self.params_window(ui.ctx(), project);
    }

    /// The arrow that drags the Extrude distance.
    fn extrude_handle(&mut self, ui: &mut Ui, ev: &Arc<Evaluation>, out: &ViewOut) {
        let Some(dlg) = self.dialog.as_mut() else {
            return;
        };
        let Tool::Extrude {
            sketch: Some(sk),
            depth,
            symmetric,
            flip,
            ..
        } = &mut dlg.tool
        else {
            return;
        };
        let Some(Product::Sketch(sp)) = ev.output(*sk).map(|o| &o.product) else {
            return;
        };
        let Some((lo, hi)) = sp.sketch.bounds() else {
            return;
        };
        let Some(d) = literal_mm(depth) else { return };
        let f = &sp.plane;
        let (cu, cv) = ((lo.0 + hi.0) / 2.0, (lo.1 + hi.1) / 2.0);
        let base = [
            f.origin.x + cu * f.x.x + cv * f.y.x,
            f.origin.y + cu * f.x.y + cv * f.y.y,
            f.origin.z + cu * f.x.z + cv * f.y.z,
        ];
        let n = [f.z.x, f.z.y, f.z.z];
        let sign = if *flip { -1.0 } else { 1.0 };
        let reach = if *symmetric { d / 2.0 } else { d };
        let o = self.viewport.origin_in_scene();
        let to_scene = |p: [f64; 3]| [p[0] + o.x, p[1] + o.y, p[2] + o.z];
        let tip_w = [
            base[0] + sign * n[0] * reach,
            base[1] + sign * n[1] * reach,
            base[2] + sign * n[2] * reach,
        ];
        let (Some(a), Some(b)) = (
            self.viewport.project(to_scene(base)),
            self.viewport.project(to_scene(tip_w)),
        ) else {
            return;
        };
        let painter = ui.painter_at(out.rect);
        painter.line_segment([a, b], Stroke::new(2.0, ACCENT));
        let handle = egui::Rect::from_center_size(b, vec2(18.0, 18.0));
        let resp = ui.interact(handle, egui::Id::new("extrude_handle"), egui::Sense::drag());
        painter.circle_filled(
            b,
            7.0,
            if resp.hovered() || resp.dragged() {
                ACCENT
            } else {
                ACCENT_SOFT
            },
        );
        painter.circle_stroke(b, 7.0, Stroke::new(1.5, ACCENT));
        if resp.dragged() {
            let dir = b - a;
            let len = dir.length();
            if len > 2.0 {
                let u = dir / len;
                let px_per_mm = len as f64 / reach.max(1e-6);
                let delta = resp.drag_delta().dot(u) as f64 / px_per_mm
                    * if *symmetric { 2.0 } else { 1.0 };
                let nd = (d + delta).max(0.01);
                *depth = format!("{nd:.2} mm");
            }
        }
    }

    /// The Change Parameters window: the Parameter features, editable.
    fn params_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        if !self.params_open {
            return;
        }
        let doc = project.document();
        let params: Vec<(kw_feature::FeatureId, String, Param)> = doc
            .features
            .iter()
            .filter_map(|f| match &f.spec {
                FeatureSpec::Parameter { value, .. } => Some((f.id, f.name.clone(), value.clone())),
                _ => None,
            })
            .collect();
        let mut open = true;
        let mut edit: Option<(kw_feature::FeatureId, String, Param)> = None;
        egui::Window::new(egui::RichText::new("CHANGE PARAMETERS").size(11.5).strong()).open(&mut open).default_pos(egui::pos2(330.0, 190.0)).collapsible(false).default_width(340.0).show(ctx, |ui| {
            if params.is_empty() {
                ui.label(egui::RichText::new("No parameters yet. Add one below, then use its name in any distance (for example width / 2).").size(11.0).color(crate::chrome::theme::TEXT_WEAK));
            }
            egui::Grid::new("param_grid").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                for (id, name, value) in &params {
                    ui.label(egui::RichText::new(name).color(TEXT));
                    let key = egui::Id::new(("param_edit", id.0));
                    let mut text: String = ui.data_mut(|d| d.get_temp(key)).unwrap_or_else(|| param_text(value));
                    let r = ui.add(egui::TextEdit::singleline(&mut text).desired_width(150.0));
                    if r.changed() {
                        ui.data_mut(|d| d.insert_temp(key, text.clone()));
                    }
                    if r.lost_focus() {
                        ui.data_mut(|d| d.remove_temp::<String>(key));
                        if text.trim() != param_text(value) {
                            match parse_param(&text, Dim::LENGTH) {
                                Ok(p) => edit = Some((*id, name.clone(), p)),
                                Err(e) => self.model.error = e,
                            }
                        }
                    }
                    ui.end_row();
                }
            });
            ui.separator();
            let key = egui::Id::new("new_param");
            let (mut name, mut val): (String, String) = ui.data_mut(|d| d.get_temp(key)).unwrap_or_else(|| ("width".into(), "20 mm".into()));
            ui.horizontal(|ui| {
                ui.add(egui::TextEdit::singleline(&mut name).desired_width(90.0).hint_text("name"));
                ui.add(egui::TextEdit::singleline(&mut val).desired_width(110.0).hint_text("value"));
                if ui.button("Add").clicked() {
                    self.model.add_parameter(project, &name, &val);
                }
            });
            ui.data_mut(|d| d.insert_temp(key, (name, val)));
            if !self.model.error.is_empty() {
                ui.colored_label(crate::chrome::theme::BAD, &self.model.error);
            }
        });
        if let Some((id, name, p)) = edit {
            self.model.edit(project, |d| {
                d.replace_spec(id, FeatureSpec::Parameter { name, value: p })
            });
        }
        self.params_open = open;
    }
}

/// The value of a plain length like "10", "10 mm" or "2.5mm" (None for expressions).
pub fn literal_mm(text: &str) -> Option<f64> {
    let t = text.trim();
    let t = t.strip_suffix("mm").unwrap_or(t).trim();
    t.parse::<f64>().ok()
}
