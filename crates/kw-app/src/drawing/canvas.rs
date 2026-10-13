//! Painting the sheet and handling the pointer: pan, zoom, select and drag views.

use super::{Drag, DrawingWorkspace, Tool};
use crate::chrome::theme::ACCENT;
use crate::project::Project;
use eframe::egui::{self, epaint::TextShape, Color32, FontId, Pos2, Rect, Shape as Sh, Stroke};
use kw_drawing::scene::{flatten, Anchor, Layer, LineStyle, Scene2d, Shape};
use kw_drawing::section::to_sheet;
use kw_drawing::{Align, Sheet, View, ViewKind};

fn colour(l: Layer) -> Color32 {
    let [r, g, b] = l.rgb();
    Color32::from_rgb(r, g, b)
}

/// Paint a scene; `to` maps sheet mm to screen points, `px` is screen pixels per mm.
pub fn paint_scene(
    painter: &egui::Painter,
    scene: &Scene2d,
    to: &dyn Fn([f64; 2]) -> Pos2,
    px: f64,
) {
    for pr in &scene.prims {
        let col = colour(pr.layer);
        let w = (pr.layer.weight() * px).clamp(0.6, 4.0) as f32;
        let stroke = Stroke::new(w, col);
        let style = pr.layer.style();
        let line = |pts: Vec<Pos2>| {
            if style == LineStyle::Continuous || pts.len() < 2 {
                painter.add(Sh::line(pts, stroke));
            } else {
                let d = style.dashes();
                let (dash, gap) = (d[0] * px, d[1] * px);
                painter.add(Sh::dashed_line(
                    &pts,
                    stroke,
                    dash.max(2.0) as f32,
                    gap.max(1.5) as f32,
                ));
            }
        };
        match &pr.shape {
            Shape::Circle { c, r } if style == LineStyle::Continuous => {
                painter.circle_stroke(to(*c), (*r * px) as f32, stroke);
            }
            Shape::Fill { pts, shade } => {
                let g = *shade;
                painter.add(Sh::convex_polygon(
                    pts.iter().map(|p| to(*p)).collect(),
                    Color32::from_gray(g),
                    Stroke::NONE,
                ));
            }
            Shape::Text {
                p,
                height,
                angle,
                anchor,
                text,
            } => {
                let size = (*height * px / 0.718) as f32;
                if size < 3.5 {
                    continue;
                }
                let galley = painter.layout_no_wrap(text.clone(), FontId::proportional(size), col);
                let width = galley.size().x;
                let shift = match anchor {
                    Anchor::Left => 0.0,
                    Anchor::Center => width / 2.0,
                    Anchor::Right => width,
                };
                // top-left of the text box: back along the baseline and up by the ascent
                let (s, c) = (angle.sin() as f32, angle.cos() as f32);
                let ascent = galley.size().y * 0.78;
                let base = to(*p);
                let pos = Pos2::new(
                    base.x - shift * c + ascent * -s,
                    base.y + shift * s + ascent * -c,
                );
                painter.add(Sh::Text(
                    TextShape::new(pos, galley, col).with_angle(-*angle as f32),
                ));
            }
            shape => {
                let pts: Vec<Pos2> = flatten(shape, 0.25 / px.max(1e-6))
                    .iter()
                    .map(|p| to(*p))
                    .collect();
                line(pts);
            }
        }
    }
}

fn view_rect(v: &View) -> Option<([f64; 2], [f64; 2])> {
    let (lo, hi) = v.cache.as_ref()?.scene.bounds()?;
    Some((
        [lo[0] + v.pos[0], lo[1] + v.pos[1]],
        [hi[0] + v.pos[0], hi[1] + v.pos[1]],
    ))
}

/// The views to move along with `id`: those projected from it, and theirs.
fn children(sheet: &Sheet, id: u64) -> Vec<u64> {
    let mut out = vec![];
    let mut frontier = vec![id];
    while let Some(p) = frontier.pop() {
        for v in &sheet.views {
            if matches!(v.kind, ViewKind::Projected { parent, .. } | ViewKind::Section { parent, .. } if parent == p)
            {
                out.push(v.id);
                frontier.push(v.id);
            }
        }
    }
    out
}

impl DrawingWorkspace {
    pub(super) fn canvas_ui(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let (rect, resp) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        self.rect = rect;
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(120, 126, 136));
        let Some(sheet) = project.drawings.sheets.get(self.sheet) else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Make a drawing with New Drawing",
                FontId::proportional(15.0),
                Color32::from_gray(225),
            );
            return;
        };
        let (pw, ph) = sheet.format.size.dims();
        // frame the sheet on first sight of it (and again when the window changes a lot)
        let key = (self.sheet, rect.width().to_bits() as u64);
        if self.fit_for != Some(key) {
            self.view.fit(rect, (0.0, 0.0), (pw, ph));
            self.fit_for = Some(key);
        }
        // zoom and pan
        if resp.hovered() {
            let (scroll, zoom) = ui
                .ctx()
                .input(|i| (i.smooth_scroll_delta.y, i.zoom_delta()));
            if let Some(pos) = ui.ctx().pointer_hover_pos() {
                if scroll != 0.0 {
                    self.view
                        .zoom_at(rect, pos, (1.0 + scroll as f64 * 0.002).max(0.1));
                }
                if zoom != 1.0 {
                    self.view.zoom_at(rect, pos, zoom as f64);
                }
            }
        }
        let panning = ui
            .ctx()
            .input(|i| i.pointer.middle_down() || i.pointer.secondary_down());
        if panning && resp.dragged() {
            let d = resp.drag_delta();
            self.view.pan(d.x, d.y);
        }

        let view = self.view;
        let px = view.scale;
        let to = |p: [f64; 2]| view.to_screen(rect, (p[0], p[1]));
        // paper with a shadow
        let paper = Rect::from_two_pos(to([0.0, 0.0]), to([pw, ph]));
        painter.rect_filled(
            paper.translate(egui::vec2(4.0, 4.0)),
            0.0,
            Color32::from_black_alpha(70),
        );
        painter.rect_filled(paper, 0.0, Color32::WHITE);
        let sheet_no = self.sheet + 1;
        let count = project.drawings.sheets.len();
        // the frame and title block, without the views (those are drawn one by one to show a selection)
        let sheet = &project.drawings.sheets[self.sheet];
        let frame = sheet.format.scene(&sheet.title, sheet_no, count);
        paint_scene(&painter, &frame, &to, px);
        for v in &sheet.views {
            let Some(cache) = &v.cache else { continue };
            let placed = cache.scene.placed(1.0, v.pos);
            paint_scene(&painter, &placed, &to, px);
            if let Some(l) = kw_drawing::render::view_label(v, &placed) {
                paint_scene(&painter, &l, &to, px);
            }
        }
        paint_scene(&painter, &sheet.annotations(), &to, px);
        if let Some(id) = self.selected {
            if let Some((lo, hi)) = sheet.view(id).and_then(view_rect) {
                let r = Rect::from_two_pos(
                    to([lo[0] - 3.0, lo[1] - 3.0]),
                    to([hi[0] + 3.0, hi[1] + 3.0]),
                );
                painter.add(Sh::dashed_line(
                    &[
                        r.left_top(),
                        r.right_top(),
                        r.right_bottom(),
                        r.left_bottom(),
                        r.left_top(),
                    ],
                    Stroke::new(1.2, ACCENT),
                    5.0,
                    3.0,
                ));
            }
        }
        if self.busy() {
            painter.text(
                rect.left_top() + egui::vec2(10.0, 10.0),
                egui::Align2::LEFT_TOP,
                "Generating views...",
                FontId::proportional(12.0),
                Color32::WHITE,
            );
        }

        // select and drag views
        let pointer = ui.ctx().pointer_interact_pos();
        let world = |p: Pos2| {
            let w = view.to_world(rect, p);
            [w.0, w.1]
        };
        let inside = |v: &View, p: [f64; 2]| {
            view_rect(v).is_some_and(|(lo, hi)| {
                p[0] >= lo[0] - 3.0
                    && p[0] <= hi[0] + 3.0
                    && p[1] >= lo[1] - 3.0
                    && p[1] <= hi[1] + 3.0
            })
        };
        // a section or detail tool takes the clicks
        if let Some(tool) = self.tool.clone() {
            if ui.ctx().input(|i| i.key_pressed(egui::Key::Escape)) {
                self.tool = None;
                self.status.clear();
                return;
            }
            let uv_of = |v: &View, p: [f64; 2]| -> Option<[f64; 2]> {
                let c = v.cache.as_ref()?;
                Some([
                    (p[0] - v.pos[0] + c.centre[0]) / v.scale,
                    (p[1] - v.pos[1] + c.centre[1]) / v.scale,
                ])
            };
            let hover = ui
                .ctx()
                .pointer_hover_pos()
                .filter(|p| rect.contains(*p))
                .map(world);
            let sheet = &project.drawings.sheets[self.sheet];
            let (first, is_section) = match &tool {
                Tool::Section { first } => (*first, true),
                Tool::Detail { first } => (*first, false),
            };
            // where the first point sits on the sheet, and the second as it would be placed
            let parent = first.as_ref().and_then(|(id, _)| sheet.view(*id));
            let end_uv = |p: [f64; 2]| -> Option<[f64; 2]> {
                let (pv, (_, a)) = (parent?, first.as_ref()?);
                let mut b = uv_of(pv, p)?;
                if is_section {
                    // square to the axes when within a few degrees
                    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                    let k = 4.0_f64.to_radians().tan();
                    if dy.abs() < k * dx.abs() {
                        b[1] = a[1];
                    } else if dx.abs() < k * dy.abs() {
                        b[0] = a[0];
                    }
                }
                Some(b)
            };
            if let (Some(pv), Some((_, a)), Some(h)) = (parent, first.as_ref(), hover) {
                if let (Some(c), Some(b)) = (pv.cache.as_ref(), end_uv(h)) {
                    let at = |uv: [f64; 2]| to(to_sheet(pv.pos, pv.scale, c.centre, uv));
                    let stroke = Stroke::new(1.5, ACCENT);
                    if is_section {
                        painter.line_segment([at(*a), at(b)], stroke);
                    } else {
                        let r = (b[0] - a[0]).hypot(b[1] - a[1]) * pv.scale * px;
                        painter.circle_stroke(at(*a), r as f32, stroke);
                    }
                }
            }
            if resp.clicked_by(egui::PointerButton::Primary) {
                if let Some(p) = pointer.map(world) {
                    match first {
                        None => {
                            let hit = sheet.views.iter().rev().find(|v| inside(v, p));
                            if let Some((v, uv)) = hit.and_then(|v| Some((v, uv_of(v, p)?))) {
                                let first = Some((v.id, uv));
                                self.tool = Some(if is_section {
                                    Tool::Section { first }
                                } else {
                                    Tool::Detail { first }
                                });
                                self.selected = Some(v.id);
                            }
                        }
                        Some((pid, a)) => {
                            if let Some(b) = end_uv(p) {
                                self.tool = None;
                                self.status.clear();
                                let res = if is_section {
                                    self.add_section(project, pid, a, b, false)
                                } else {
                                    let r = (b[0] - a[0]).hypot(b[1] - a[1]);
                                    self.add_detail(project, pid, a, r, None)
                                };
                                if let Err(e) = res {
                                    self.status = e;
                                }
                            }
                        }
                    }
                }
            }
            return;
        }
        if resp.drag_started_by(egui::PointerButton::Primary) {
            if let Some(p) = pointer.map(world) {
                let hit = project.drawings.sheets[self.sheet]
                    .views
                    .iter()
                    .rev()
                    .find(|v| inside(v, p))
                    .map(|v| v.id);
                if let Some(id) = hit {
                    self.selected = Some(id);
                    project.dw_checkpoint();
                    self.drag = Some(Drag { view: id });
                }
            }
        }
        if let Some(d) = &self.drag {
            if resp.dragged_by(egui::PointerButton::Primary) {
                let delta = resp.drag_delta();
                let (mut dx, mut dy) = (delta.x as f64 / px, -delta.y as f64 / px);
                let sheet = &mut project.drawings.sheets[self.sheet];
                // a projected view slides along the line through its parent
                match sheet.view(d.view).map(|v| &v.kind) {
                    Some(kw_drawing::ViewKind::Projected {
                        align: Align::Horizontal,
                        ..
                    }) => dy = 0.0,
                    Some(kw_drawing::ViewKind::Projected {
                        align: Align::Vertical,
                        ..
                    }) => dx = 0.0,
                    _ => {}
                }
                let mut moving = children(sheet, d.view);
                moving.push(d.view);
                for id in moving {
                    if let Some(v) = sheet.view_mut(id) {
                        v.pos[0] += dx;
                        v.pos[1] += dy;
                    }
                }
                project.mark_extras_dirty();
            }
        }
        if resp.drag_stopped() && self.drag.take().is_some() {
            project.dw_drop_checkpoint_if_unchanged();
        }
        if resp.clicked_by(egui::PointerButton::Primary) {
            if let Some(p) = pointer.map(world) {
                self.selected = project.drawings.sheets[self.sheet]
                    .views
                    .iter()
                    .rev()
                    .find(|v| inside(v, p))
                    .map(|v| v.id);
            }
        }
    }
}
