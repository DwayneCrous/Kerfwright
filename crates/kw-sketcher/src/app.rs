use crate::view::View;
use eframe::egui::{self, Color32, Key, PointerButton, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use kw_dxf::{export_sketch, import_sketch, parse, write, ImportOptions};
use kw_sketch::editor::{ConstraintTool, Editor, EntityState, Input, Snap, Tool};
use kw_sketch::{geom, ConstraintId, ConstraintKind, EntityId, EntityKind, Sketch, SketchStatus};

type P2 = (f64, f64);

const TOOLS: &[(Tool, &str, &str)] = &[
    (Tool::Select, "Select", "Esc"),
    (Tool::Point, "Point", "P"),
    (Tool::Line, "Line", "L"),
    (Tool::Rectangle, "Rectangle", "R"),
    (Tool::Circle, "Circle", "C"),
    (Tool::Arc, "Arc", "A"),
    (Tool::Spline, "Spline", "S"),
    (Tool::Dimension, "Dimension", "D"),
    (Tool::Trim, "Trim", "T"),
    (Tool::Extend, "Extend", "E"),
    (Tool::Offset, "Offset", "O"),
];

const CONSTRAINTS: &[(ConstraintTool, &str)] = &[
    (ConstraintTool::Coincident, "Coincident"),
    (ConstraintTool::Horizontal, "Horizontal"),
    (ConstraintTool::Vertical, "Vertical"),
    (ConstraintTool::Parallel, "Parallel"),
    (ConstraintTool::Perpendicular, "Perpendicular"),
    (ConstraintTool::Tangent, "Tangent"),
    (ConstraintTool::Equal, "Equal"),
    (ConstraintTool::Fix, "Fix"),
    (ConstraintTool::Midpoint, "Midpoint"),
    (ConstraintTool::Concentric, "Concentric"),
    (ConstraintTool::Collinear, "Collinear"),
    (ConstraintTool::PointOnCurve, "Point on curve"),
    (ConstraintTool::Symmetric, "Symmetric"),
];

fn state_color(s: EntityState) -> Color32 {
    match s {
        EntityState::Constrained => Color32::from_rgb(70, 190, 110),
        EntityState::Free => Color32::from_rgb(90, 160, 255),
        EntityState::Redundant => Color32::from_rgb(235, 200, 60),
        EntityState::Conflicting => Color32::from_rgb(235, 70, 70),
    }
}

fn constraint_label(k: &ConstraintKind) -> &'static str {
    use ConstraintKind::*;
    match k {
        Coincident { .. } => "Coincident",
        PointOnCurve { .. } => "Point on curve",
        Horizontal(_) => "Horizontal",
        Vertical(_) => "Vertical",
        HorizontalPts { .. } => "Horizontal points",
        VerticalPts { .. } => "Vertical points",
        Parallel { .. } => "Parallel",
        Perpendicular { .. } => "Perpendicular",
        Collinear { .. } => "Collinear",
        Concentric { .. } => "Concentric",
        EqualLength { .. } => "Equal length",
        EqualRadius { .. } => "Equal radius",
        Midpoint { .. } => "Midpoint",
        Symmetric { .. } => "Symmetric",
        Fix(_) => "Fix",
        Tangent { .. } => "Tangent",
        Joint { .. } => "Joint",
        Distance { .. } => "Distance",
        Angle { .. } => "Angle",
        Radius { .. } => "Radius",
        Diameter { .. } => "Diameter",
    }
}

pub struct SketcherApp {
    pub editor: Editor,
    pub view: View,
    /// Quit after this many frames (smoke test).
    pub exit_after_frames: Option<u64>,
    pub frames: u64,
    canvas: Rect,
    first_frame: bool,
    show_grid: bool,
    file_open: bool,
    file_path: String,
    file_msg: String,
    /// Dimension value being edited (opened right after the dimension tool creates one).
    dim_edit: Option<(ConstraintId, String)>,
    /// A drag of sketch geometry is in progress.
    grabbing: bool,
}

impl Default for SketcherApp {
    fn default() -> Self {
        Self {
            editor: Editor::default(),
            view: View::default(),
            exit_after_frames: None,
            frames: 0,
            canvas: Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0)),
            first_frame: true,
            show_grid: true,
            file_open: false,
            file_path: "sketch.dxf".into(),
            file_msg: String::new(),
            dim_edit: None,
            grabbing: false,
        }
    }
}

impl SketcherApp {
    /// A 40 × 25 rectangle with a hole, partly dimensioned, to show the colouring.
    pub fn load_demo(&mut self) {
        let mut sk = Sketch::new();
        let pts: Vec<_> = [(0.0, 0.0), (40.0, 0.0), (40.0, 25.0), (0.0, 25.0)]
            .iter()
            .map(|p| sk.add_point(p.0, p.1))
            .collect();
        let l: Vec<_> = (0..4)
            .map(|i| sk.add_line(pts[i], pts[(i + 1) % 4]))
            .collect();
        let _ = sk.horizontal(l[0]);
        let _ = sk.horizontal(l[2]);
        let _ = sk.vertical(l[1]);
        let _ = sk.vertical(l[3]);
        let _ = sk.fix(pts[0]);
        let _ = sk.dim_distance(l[0], l[0]);
        let c = sk.add_point(20.0, 12.0);
        sk.add_circle(c, 5.0);
        self.editor.replace_sketch(sk);
        self.first_frame = true;
    }

    pub fn open_dxf(&mut self, path: &str) -> Result<(), String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        let doc = parse(&text).map_err(|e| e.to_string())?;
        let sk = import_sketch(&doc, &ImportOptions::default());
        let n = sk.entities.len();
        self.editor.replace_sketch(sk);
        self.editor.status = format!("imported {n} entities from {path}");
        self.first_frame = true;
        Ok(())
    }

    pub fn save_dxf(&mut self, path: &str) -> Result<(), String> {
        let text = write(&export_sketch(&self.editor.sketch));
        std::fs::write(path, text).map_err(|e| e.to_string())?;
        self.editor.status = format!("exported to {path}");
        Ok(())
    }

    /// The canvas rectangle of the last frame (screen pixels).
    pub fn canvas_rect(&self) -> Rect {
        self.canvas
    }

    // ---------------------------------------------------------------- layout

    pub fn show(&mut self, ui: &mut egui::Ui) {
        self.frames += 1;
        self.shortcuts(ui.ctx());
        egui::Panel::top("menu").show(ui, |ui| self.menu(ui));
        egui::Panel::left("tools")
            .default_size(150.0)
            .show(ui, |ui| self.tool_panel(ui));
        egui::Panel::right("inspector")
            .default_size(270.0)
            .show(ui, |ui| self.inspector(ui));
        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(&self.editor.status);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let c = self.editor.cursor;
                    ui.monospace(format!(
                        "x {:>9.3}  y {:>9.3}   zoom {:.2} px/unit",
                        c.0, c.1, self.view.scale
                    ));
                });
            });
        });
        egui::CentralPanel::default().show(ui, |ui| self.canvas_ui(ui));
        self.dialogs(ui.ctx());

        if let Some(n) = self.exit_after_frames {
            if self.frames >= n {
                ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
            } else {
                ui.ctx().request_repaint();
            }
        }
    }

    fn menu(&mut self, ui: &mut egui::Ui) {
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("New").clicked() {
                    self.editor.replace_sketch(Sketch::new());
                    ui.close();
                }
                if ui.button("Import / export DXF…").clicked() {
                    self.file_open = true;
                    ui.close();
                }
                if ui.button("Load demo").clicked() {
                    self.load_demo();
                    ui.close();
                }
            });
            ui.menu_button("Edit", |ui| {
                if ui
                    .add_enabled(self.editor.can_undo(), egui::Button::new("Undo   Ctrl+Z"))
                    .clicked()
                {
                    self.editor.undo();
                    ui.close();
                }
                if ui
                    .add_enabled(
                        self.editor.can_redo(),
                        egui::Button::new("Redo   Ctrl+Shift+Z"),
                    )
                    .clicked()
                {
                    self.editor.redo();
                    ui.close();
                }
                if ui.button("Toggle construction").clicked() {
                    self.editor.toggle_construction();
                    ui.close();
                }
                if ui.button("Delete selection   Del").clicked() {
                    self.editor.delete_selection();
                    ui.close();
                }
            });
            ui.menu_button("View", |ui| {
                ui.checkbox(&mut self.show_grid, "Grid");
                if ui.button("Zoom to fit   F").clicked() {
                    self.first_frame = true;
                    ui.close();
                }
            });
        });
    }

    fn tool_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Tools");
        for (tool, name, key) in TOOLS {
            let selected = self.editor.tool == *tool;
            if ui
                .selectable_label(selected, format!("{name}  ({key})"))
                .clicked()
            {
                self.editor.set_tool(*tool);
            }
        }
        ui.horizontal(|ui| {
            ui.label("Offset");
            ui.add(
                egui::DragValue::new(&mut self.editor.offset_distance)
                    .speed(0.1)
                    .range(1e-6..=1e6),
            );
        });
        if ui.button("Finish   (Enter)").clicked() {
            self.editor.handle(Input::Finish);
        }
        ui.separator();
        ui.heading("Constraints");
        ui.horizontal_wrapped(|ui| {
            for (c, name) in CONSTRAINTS {
                let tool = Tool::Constrain(*c);
                if ui
                    .selectable_label(self.editor.tool == tool, *name)
                    .clicked()
                {
                    self.editor.set_tool(tool);
                }
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(self.editor.can_undo(), egui::Button::new("Undo"))
                .clicked()
            {
                self.editor.undo();
            }
            if ui
                .add_enabled(self.editor.can_redo(), egui::Button::new("Redo"))
                .clicked()
            {
                self.editor.redo();
            }
        });
    }

    fn inspector(&mut self, ui: &mut egui::Ui) {
        ui.heading("Solver");
        match &self.editor.report {
            None => {
                ui.label("(not solved yet)");
            }
            Some(r) => {
                let (text, color) = match r.status {
                    SketchStatus::UnderConstrained => {
                        ("Under-constrained", state_color(EntityState::Free))
                    }
                    SketchStatus::FullyConstrained => {
                        ("Fully constrained", state_color(EntityState::Constrained))
                    }
                    SketchStatus::OverConstrained => (
                        "Over-constrained (redundant)",
                        state_color(EntityState::Redundant),
                    ),
                    SketchStatus::Conflicting => (
                        "Conflicting constraints",
                        state_color(EntityState::Conflicting),
                    ),
                };
                ui.colored_label(color, text);
                ui.label(format!(
                    "DOF: {}{}",
                    r.dof,
                    if r.exact { "" } else { " (lower bound)" }
                ));
                ui.label(format!("residual: {:.1e}", r.residual));
                if !r.conflicting.is_empty() {
                    ui.colored_label(
                        state_color(EntityState::Conflicting),
                        format!("{} conflicting", r.conflicting.len()),
                    );
                }
                if !r.redundant.is_empty() {
                    ui.colored_label(
                        state_color(EntityState::Redundant),
                        format!("{} redundant", r.redundant.len()),
                    );
                }
            }
        }
        ui.separator();
        ui.heading("Constraints");
        let ids: Vec<ConstraintId> = self.editor.sketch.constraints.keys().collect();
        let (red, con) = self
            .editor
            .report
            .as_ref()
            .map(|r| (r.redundant.clone(), r.conflicting.clone()))
            .unwrap_or_default();
        let mut remove = None;
        let mut change = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            for id in ids {
                let Some(c) = self.editor.sketch.constraints.get(id) else {
                    continue;
                };
                // internal arc-consistency rows are not user constraints
                let label = constraint_label(&c.kind);
                let color = if con.contains(&id) {
                    Some(state_color(EntityState::Conflicting))
                } else if red.contains(&id) {
                    Some(state_color(EntityState::Redundant))
                } else {
                    None
                };
                ui.horizontal(|ui| {
                    match color {
                        Some(col) => ui.colored_label(col, label),
                        None => ui.label(label),
                    };
                    if let Some(mut v) = self.editor.sketch.dimension_value(id) {
                        let angle = matches!(c.kind, ConstraintKind::Angle { .. });
                        if angle {
                            v = v.to_degrees();
                        }
                        let before = v;
                        ui.add(egui::DragValue::new(&mut v).speed(0.1).suffix(if angle {
                            "°"
                        } else {
                            ""
                        }));
                        if v != before {
                            change = Some((id, if angle { v.to_radians() } else { v }));
                        }
                    }
                    if ui
                        .small_button("✕")
                        .on_hover_text("remove constraint")
                        .clicked()
                    {
                        remove = Some(id);
                    }
                });
            }
        });
        if let Some((id, v)) = change {
            if !self.editor.edit_dimension(id, v) {
                self.editor.status = "that value cannot be satisfied".into();
            }
        }
        if let Some(id) = remove {
            self.editor.remove_constraint(id);
        }
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        if let Some(c) = self.editor.last_dimension.take() {
            if let Some(v) = self.editor.sketch.dimension_value(c) {
                let v = if matches!(
                    self.editor.sketch.constraints.get(c).map(|c| &c.kind),
                    Some(ConstraintKind::Angle { .. })
                ) {
                    v.to_degrees()
                } else {
                    v
                };
                self.dim_edit = Some((c, format!("{v:.4}")));
            }
        }
        let mut close = false;
        let mut apply = None;
        if let Some((c, text)) = &mut self.dim_edit {
            egui::Window::new("Dimension")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    let r = ui.text_edit_singleline(text);
                    r.request_focus();
                    ui.horizontal(|ui| {
                        let enter = r.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
                        if ui.button("OK").clicked() || enter {
                            apply = Some((*c, text.clone()));
                        }
                        if ui.button("Cancel").clicked() {
                            close = true;
                        }
                    });
                });
        }
        if let Some((c, text)) = apply {
            match text.trim().parse::<f64>() {
                Ok(mut v) => {
                    if matches!(
                        self.editor.sketch.constraints.get(c).map(|c| &c.kind),
                        Some(ConstraintKind::Angle { .. })
                    ) {
                        v = v.to_radians();
                    }
                    if !self.editor.edit_dimension(c, v) {
                        self.editor.status = "that value cannot be satisfied".into();
                    }
                    close = true;
                }
                Err(_) => self.editor.status = "not a number".into(),
            }
        }
        if close {
            self.dim_edit = None;
        }

        if self.file_open {
            let mut open = true;
            let mut action = None;
            egui::Window::new("DXF")
                .open(&mut open)
                .collapsible(false)
                .show(ctx, |ui| {
                    ui.text_edit_singleline(&mut self.file_path);
                    ui.horizontal(|ui| {
                        if ui.button("Import").clicked() {
                            action = Some(false);
                        }
                        if ui.button("Export").clicked() {
                            action = Some(true);
                        }
                    });
                    if !self.file_msg.is_empty() {
                        ui.label(&self.file_msg);
                    }
                });
            if let Some(export) = action {
                let path = self.file_path.clone();
                let r = if export {
                    self.save_dxf(&path)
                } else {
                    self.open_dxf(&path)
                };
                self.file_msg = match r {
                    Ok(()) => self.editor.status.clone(),
                    Err(e) => format!("error: {e}"),
                };
            }
            self.file_open = open;
        }
    }

    fn shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (undo, redo, del, enter, esc, fit, letter, ctrl) = ctx.input(|i| {
            let ctrl = i.modifiers.command;
            let letters = [
                (Key::P, Tool::Point),
                (Key::L, Tool::Line),
                (Key::R, Tool::Rectangle),
                (Key::C, Tool::Circle),
                (Key::A, Tool::Arc),
                (Key::S, Tool::Spline),
                (Key::D, Tool::Dimension),
                (Key::T, Tool::Trim),
                (Key::E, Tool::Extend),
                (Key::O, Tool::Offset),
            ];
            (
                ctrl && i.key_pressed(Key::Z) && !i.modifiers.shift,
                ctrl && (i.key_pressed(Key::Y) || (i.key_pressed(Key::Z) && i.modifiers.shift)),
                i.key_pressed(Key::Delete) || i.key_pressed(Key::Backspace),
                i.key_pressed(Key::Enter),
                i.key_pressed(Key::Escape),
                i.key_pressed(Key::F),
                letters
                    .iter()
                    .find(|(k, _)| i.key_pressed(*k))
                    .map(|(_, t)| *t),
                ctrl,
            )
        });
        if undo {
            self.editor.undo();
        } else if redo {
            self.editor.redo();
        } else if del {
            self.editor.handle(Input::Delete);
        } else if enter {
            self.editor.handle(Input::Finish);
        } else if esc {
            if self.editor.tool == Tool::Select {
                self.editor.handle(Input::Escape);
            } else {
                self.editor.handle(Input::Escape);
                self.editor.set_tool(Tool::Select);
            }
        } else if fit {
            self.first_frame = true;
        } else if let (Some(t), false) = (letter, ctrl) {
            self.editor.set_tool(t);
        }
    }

    // ---------------------------------------------------------------- canvas

    fn canvas_ui(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        self.canvas = rect;
        if self.first_frame {
            self.first_frame = false;
            match self.editor.sketch.bounds() {
                Some((lo, hi)) => self.view.fit(rect, lo, hi),
                None => self.view = View::default(),
            }
        }

        // ---- view navigation
        if response.hovered() {
            let (scroll, pinch, pos) = ui.input(|i| {
                (
                    i.smooth_scroll_delta.y,
                    i.zoom_delta(),
                    i.pointer.hover_pos(),
                )
            });
            if let Some(pos) = pos {
                if scroll != 0.0 {
                    self.view.zoom_at(rect, pos, (scroll as f64 * 0.0025).exp());
                }
                if pinch != 1.0 {
                    self.view.zoom_at(rect, pos, pinch as f64);
                }
            }
        }
        let pan_drag = response.dragged_by(PointerButton::Middle)
            || response.dragged_by(PointerButton::Secondary);
        if pan_drag {
            let d = response.drag_delta();
            self.view.pan(d.x, d.y);
        }

        // ---- editor input
        self.editor.multi_select = ui.input(|i| i.modifiers.command || i.modifiers.shift);
        self.editor.tolerance = 8.0 / self.view.scale;
        if let Some(pos) = response.hover_pos().or(response.interact_pointer_pos()) {
            let w = self.view.to_world(rect, pos);
            self.editor.handle(Input::Move(w));
        }
        let select = self.editor.tool == Tool::Select;
        if response.drag_started_by(PointerButton::Primary) && select {
            // grab where the button went down, not where egui's drag threshold fired
            if let Some(pos) = ui
                .input(|i| i.pointer.press_origin())
                .or(response.interact_pointer_pos())
            {
                let w = self.view.to_world(rect, pos);
                if let Some(e) = geom::pick(&self.editor.sketch, w, self.editor.tolerance) {
                    if !self.editor.selection.contains(&e) {
                        self.editor.handle(Input::Click(w));
                    }
                    self.editor.begin_drag(e, w);
                    self.grabbing = self.editor.is_dragging();
                }
            }
        }
        if self.grabbing {
            if let Some(pos) = response.interact_pointer_pos() {
                self.editor.drag_to(self.view.to_world(rect, pos));
            }
            if response.drag_stopped() {
                self.editor.end_drag();
                self.grabbing = false;
            }
        } else if select && response.dragged_by(PointerButton::Primary) {
            // dragging empty space pans
            let d = response.drag_delta();
            self.view.pan(d.x, d.y);
        }
        if response.clicked_by(PointerButton::Primary) {
            if let Some(pos) = response.interact_pointer_pos() {
                let w = self.view.to_world(rect, pos);
                self.editor.handle(Input::Click(w));
            }
        }
        if response.double_clicked_by(PointerButton::Primary)
            && matches!(self.editor.tool, Tool::Spline)
        {
            self.editor.handle(Input::Finish);
        }

        self.paint(&painter, rect, response.hover_pos());
    }

    fn paint(&self, painter: &egui::Painter, rect: Rect, hover: Option<Pos2>) {
        let ctx_dark = painter.ctx().global_style().visuals.dark_mode;
        painter.rect_filled(
            rect,
            0.0,
            if ctx_dark {
                Color32::from_gray(22)
            } else {
                Color32::from_gray(245)
            },
        );
        let painter = painter.with_clip_rect(rect);
        let v = &self.view;

        if self.show_grid {
            let step = v.grid_step(24.0);
            let lo = v.to_world(rect, rect.left_bottom());
            let hi = v.to_world(rect, rect.right_top());
            let grid = Stroke::new(1.0, Color32::from_gray(if ctx_dark { 42 } else { 222 }));
            let mut x = (lo.0 / step).floor() * step;
            while x <= hi.0 {
                let sx = v.to_screen(rect, (x, 0.0)).x;
                painter.line_segment(
                    [Pos2::new(sx, rect.top()), Pos2::new(sx, rect.bottom())],
                    grid,
                );
                x += step;
            }
            let mut y = (lo.1 / step).floor() * step;
            while y <= hi.1 {
                let sy = v.to_screen(rect, (0.0, y)).y;
                painter.line_segment(
                    [Pos2::new(rect.left(), sy), Pos2::new(rect.right(), sy)],
                    grid,
                );
                y += step;
            }
        }
        let o = v.to_screen(rect, (0.0, 0.0));
        painter.line_segment(
            [Pos2::new(rect.left(), o.y), Pos2::new(rect.right(), o.y)],
            Stroke::new(1.0, Color32::from_rgb(160, 70, 70)),
        );
        painter.line_segment(
            [Pos2::new(o.x, rect.top()), Pos2::new(o.x, rect.bottom())],
            Stroke::new(1.0, Color32::from_rgb(70, 150, 70)),
        );

        let sk = &self.editor.sketch;
        let tol = 0.75 / v.scale;
        let selected: Vec<EntityId> = self.editor.selection.clone();
        // curves first, points on top
        for (id, ent) in &sk.entities {
            if matches!(ent.kind, EntityKind::Point(_)) {
                continue;
            }
            let Ok(poly) = geom::polyline(sk, id, tol) else {
                continue;
            };
            let mut color = state_color(self.editor.entity_state(id));
            let mut width = 2.0;
            if selected.contains(&id) {
                color = Color32::from_rgb(255, 150, 30);
                width = 3.0;
            }
            let pts: Vec<Pos2> = poly.iter().map(|p| v.to_screen(rect, *p)).collect();
            if ent.construction {
                painter.extend(Shape::dashed_line(
                    &pts,
                    Stroke::new(1.0, color.gamma_multiply(0.8)),
                    6.0,
                    4.0,
                ));
            } else {
                painter.add(Shape::line(pts, Stroke::new(width, color)));
            }
            if let EntityKind::Spline { control, .. } = &ent.kind {
                let hull: Vec<Pos2> = control
                    .iter()
                    .filter_map(|c| sk.point(*c).ok())
                    .map(|p| v.to_screen(rect, p))
                    .collect();
                painter.extend(Shape::dashed_line(
                    &hull,
                    Stroke::new(1.0, Color32::from_gray(120)),
                    3.0,
                    3.0,
                ));
            }
        }
        for (id, ent) in &sk.entities {
            if !matches!(ent.kind, EntityKind::Point(_)) {
                continue;
            }
            let Ok(p) = sk.point(id) else { continue };
            let mut color = state_color(self.editor.entity_state(id));
            let mut r = 3.0;
            if selected.contains(&id) {
                color = Color32::from_rgb(255, 150, 30);
                r = 4.5;
            }
            painter.circle_filled(v.to_screen(rect, p), r, color);
        }

        // dimension labels
        for (cid, c) in &sk.constraints {
            let Some(val) = sk.dimension_value(cid) else {
                continue;
            };
            let pts: Vec<P2> = c
                .kind
                .entities()
                .iter()
                .flat_map(|e| sk.children(*e).into_iter().chain([*e]))
                .filter_map(|e| sk.point(e).ok())
                .collect();
            if pts.is_empty() {
                continue;
            }
            let n = pts.len() as f64;
            let at = (
                pts.iter().map(|p| p.0).sum::<f64>() / n,
                pts.iter().map(|p| p.1).sum::<f64>() / n,
            );
            let text = match c.kind {
                ConstraintKind::Angle { .. } => format!("{:.2}°", val.to_degrees()),
                ConstraintKind::Radius { .. } => format!("R{val:.3}"),
                ConstraintKind::Diameter { .. } => format!("⌀{val:.3}"),
                _ => format!("{val:.3}"),
            };
            painter.text(
                v.to_screen(rect, at) + Vec2::new(0.0, -10.0),
                egui::Align2::CENTER_BOTTOM,
                text,
                egui::FontId::proportional(13.0),
                Color32::from_rgb(120, 190, 255),
            );
        }

        // rubber band + snap marker
        if self.editor.tool != Tool::Select {
            if let Some(pos) = hover {
                let cursor = self.editor.snap(v.to_world(rect, pos));
                let a = self.editor.anchors();
                let rub = Stroke::new(1.0, Color32::from_gray(150));
                if let Some(last) = a.last() {
                    painter.extend(Shape::dashed_line(
                        &[
                            v.to_screen(rect, *last),
                            v.to_screen(rect, cursor.position()),
                        ],
                        rub,
                        4.0,
                        3.0,
                    ));
                }
                for p in &a {
                    painter.circle_stroke(v.to_screen(rect, *p), 4.0, rub);
                }
                let sp = v.to_screen(rect, cursor.position());
                match cursor {
                    Snap::Point(..) => {
                        painter.rect_stroke(
                            Rect::from_center_size(sp, Vec2::splat(11.0)),
                            0.0,
                            Stroke::new(1.5, Color32::YELLOW),
                            egui::StrokeKind::Middle,
                        );
                    }
                    Snap::OnCurve(..) => {
                        painter.circle_stroke(sp, 6.0, Stroke::new(1.5, Color32::YELLOW));
                    }
                    Snap::Free(_) => {
                        painter.circle_filled(sp, 2.0, Color32::from_gray(180));
                    }
                }
            }
        }
    }
}

impl eframe::App for SketcherApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.show(ui);
    }
}
