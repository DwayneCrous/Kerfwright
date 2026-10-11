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

/// Fill the area inside `polys` (even-odd: holes stay empty) with thin horizontal strips.
fn fill_polys(painter: &egui::Painter, polys: &[Vec<egui::Pos2>], colour: Color32) {
    let (mut y0, mut y1) = (f32::INFINITY, f32::NEG_INFINITY);
    for p in polys.iter().flatten() {
        y0 = y0.min(p.y);
        y1 = y1.max(p.y);
    }
    if !y0.is_finite() {
        return;
    }
    let mut y = (y0 / 2.0).floor() * 2.0 + 1.0;
    while y < y1 {
        let mut xs: Vec<f32> = Vec::new();
        for poly in polys {
            let n = poly.len();
            for i in 0..n {
                let (a, b) = (poly[i], poly[(i + 1) % n]);
                if (a.y <= y) != (b.y <= y) {
                    xs.push(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
        }
        xs.sort_by(f32::total_cmp);
        for pair in xs.as_chunks::<2>().0 {
            painter.rect_filled(
                egui::Rect::from_min_max(pos2(pair[0], y - 1.0), pos2(pair[1], y + 1.0)),
                0.0,
                colour,
            );
        }
        y += 2.0;
    }
}

impl DesignWorkspace {
    /// The regions of the open Extrude dialog's sketch drawn on the sketch plane: chosen ones strongly, the
    /// others faintly, the one under the pointer highlighted. Clicking one chooses it.
    fn draw_regions(&self, ui: &Ui, painter: &egui::Painter, rect: egui::Rect, o3: [f64; 3]) {
        let Some(rv) = self.region_view.as_ref().filter(|r| r.regions().len() > 1) else {
            return;
        };
        let Some(Tool::Extrude { more, .. }) = self.dialog.as_ref().map(|d| &d.tool) else {
            return;
        };
        let hover = ui
            .ctx()
            .pointer_hover_pos()
            .filter(|p| rect.contains(*p))
            .and_then(|p| self.region_at(p));
        let f = &rv.plane;
        let chosen_of = rv.matching(&more.regions);
        for (ri, r) in rv.regions().iter().enumerate() {
            let polys: Vec<Vec<egui::Pos2>> = r
                .shapes
                .iter()
                .flatten()
                .map(|contour| {
                    contour
                        .iter()
                        .filter_map(|p| {
                            let w = f.origin + f.x * p[0] + f.y * p[1];
                            self.viewport
                                .project([w.x + o3[0], w.y + o3[1], w.z + o3[2]])
                        })
                        .collect::<Vec<_>>()
                })
                .filter(|v| v.len() >= 3)
                .collect();
            let chosen = !chosen_of[ri].is_empty();
            let hot = hover == Some(ri);
            let colour = match (chosen, hot) {
                (true, true) => Color32::from_rgba_unmultiplied(6, 150, 215, 150),
                (true, false) => Color32::from_rgba_unmultiplied(6, 150, 215, 85),
                (false, true) => Color32::from_rgba_unmultiplied(120, 120, 120, 120),
                (false, false) => Color32::from_rgba_unmultiplied(120, 120, 120, 30),
            };
            fill_polys(painter, &polys, colour);
            for poly in &polys {
                painter.add(Shape::closed_line(
                    poly.clone(),
                    Stroke::new(
                        if hot { 2.0 } else { 1.0 },
                        if chosen {
                            ACCENT
                        } else {
                            Color32::from_gray(130)
                        },
                    ),
                ));
            }
        }
    }

    /// The control cage of the Form dialog: its edges, creased edges thicker, the edited vertex marked.
    fn draw_cage(&self, painter: &egui::Painter) {
        let Some(Tool::Form {
            cage,
            scale,
            vertex,
            ..
        }) = self.dialog.as_ref().map(|d| &d.tool)
        else {
            return;
        };
        let k = scale.trim().parse::<f64>().unwrap_or(1.0);
        let at = |v: u32| {
            let p = cage.verts.get(v as usize)?;
            self.viewport.project([p[0] * k, p[1] * k, p[2] * k])
        };
        let creased = |a: u32, b: u32| {
            cage.creases
                .iter()
                .any(|c| (c.a == a && c.b == b) || (c.a == b && c.b == a))
        };
        let colour = Color32::from_rgb(40, 150, 90);
        for f in &cage.faces {
            for i in 0..f.len() {
                let (a, b) = (f[i], f[(i + 1) % f.len()]);
                if let (Some(pa), Some(pb)) = (at(a), at(b)) {
                    painter.line_segment(
                        [pa, pb],
                        Stroke::new(if creased(a, b) { 2.2 } else { 1.0 }, colour),
                    );
                }
            }
        }
        for v in 0..cage.verts.len() as u32 {
            if let Some(p) = at(v) {
                let on = v as usize == *vertex;
                painter.circle_filled(
                    p,
                    if on { 5.0 } else { 2.5 },
                    if on { ACCENT } else { colour },
                );
            }
        }
    }

    /// Construction planes (quads), axes (long lines) and points (crosses) of the document.
    fn draw_datums(&self, painter: &egui::Painter, ev: &Evaluation, project: &Project, size: f64) {
        use kw_feature::Product;
        let doc = project.document();
        for f in &doc.features {
            let kind = match f.spec {
                FeatureSpec::DatumPlane(_) => 0,
                FeatureSpec::DatumAxis(_) => 1,
                FeatureSpec::DatumPoint(_) => 2,
                _ => continue,
            };
            if f.suppressed || self.hidden_datums.contains(&f.id) {
                continue;
            }
            let Some(Product::Datum(fr)) = ev.output(f.id).map(|o| &o.product) else {
                continue;
            };
            let chosen = self.model.selected == Some(f.id);
            let colour = if chosen {
                ACCENT
            } else {
                Color32::from_rgb(214, 150, 40)
            };
            let at = |a: f64, b: f64, c: f64| {
                let w = fr.origin + fr.x * a + fr.y * b + fr.z * c;
                self.viewport.project([w.x, w.y, w.z])
            };
            match kind {
                0 => {
                    let pts: Vec<egui::Pos2> =
                        [(-size, -size), (size, -size), (size, size), (-size, size)]
                            .iter()
                            .filter_map(|(u, v)| at(*u, *v, 0.0))
                            .collect();
                    if pts.len() == 4 {
                        painter.add(Shape::convex_polygon(
                            pts.clone(),
                            colour.gamma_multiply(0.14),
                            Stroke::new(1.0, colour),
                        ));
                        painter.text(
                            pts[2] + vec2(-4.0, -4.0),
                            egui::Align2::RIGHT_BOTTOM,
                            &f.name,
                            egui::FontId::proportional(11.0),
                            colour,
                        );
                    }
                }
                1 => {
                    if let (Some(a), Some(b)) =
                        (at(0.0, 0.0, -size * 1.4), at(0.0, 0.0, size * 1.4))
                    {
                        painter.line_segment([a, b], Stroke::new(1.5, colour));
                        painter.text(
                            b,
                            egui::Align2::LEFT_BOTTOM,
                            &f.name,
                            egui::FontId::proportional(11.0),
                            colour,
                        );
                    }
                }
                _ => {
                    if let Some(c) = at(0.0, 0.0, 0.0) {
                        let stroke = Stroke::new(1.5, colour);
                        painter.line_segment([c + vec2(-5.0, 0.0), c + vec2(5.0, 0.0)], stroke);
                        painter.line_segment([c + vec2(0.0, -5.0), c + vec2(0.0, 5.0)], stroke);
                        painter.circle_stroke(c, 3.0, stroke);
                        painter.text(
                            c + vec2(7.0, -3.0),
                            egui::Align2::LEFT_BOTTOM,
                            &f.name,
                            egui::FontId::proportional(11.0),
                            colour,
                        );
                    }
                }
            }
        }
    }

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
        // ---- construction planes, axes and points
        self.draw_datums(&painter, ev, project, s);
        // ---- the cage of the Form being edited
        self.draw_cage(&painter);
        // ---- the regions of the Extrude dialog's sketch
        self.draw_regions(ui, &painter, rect, o3);
        // ---- view cube
        let cube =
            egui::Rect::from_min_size(rect.right_top() + vec2(-146.0, 8.0), vec2(136.0, 148.0));
        let cube_out = viewcube::show(ui, cube, &self.viewport.camera);
        let now = ui.ctx().input(|i| i.time);
        if let Some((yaw, pitch)) = cube_out.look {
            self.viewport.look(yaw, pitch, now);
        }
        match cube_out.menu {
            Some(viewcube::CubeMenu::Isometric) => {
                self.viewport.look(-0.8, 0.45, now);
                self.viewport.fit();
            }
            Some(viewcube::CubeMenu::Fit) => self.viewport.fit(),
            Some(viewcube::CubeMenu::Previous) => self.viewport.previous_view(now),
            Some(viewcube::CubeMenu::Shaded) => self.shaded = !self.shaded,
            Some(viewcube::CubeMenu::Edges) => self.viewport.show_edges = !self.viewport.show_edges,
            Some(viewcube::CubeMenu::Section) => {
                self.section = match self.section {
                    Some(_) => None,
                    None => Some(super::section::SectionState::new()),
                };
            }
            None => {}
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
        self.measure_window(ui.ctx(), project);
        self.interference_window(ui.ctx());
        self.section_window(ui.ctx(), project);
        self.assembly_window(ui.ctx(), project);
        self.flat_window(ui.ctx());
        self.body_props_window(ui.ctx(), project);
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
