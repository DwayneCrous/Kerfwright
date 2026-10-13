//! Sketch commands that need the model or a small panel: Project, Intersect, patterns and text.

use super::{sketch_project, DesignWorkspace};
use crate::bodies::Bodies;
use crate::chrome::theme::TEXT_WEAK;
use crate::project::Project;
use eframe::egui;
use kw_sketch::text::{text_curves, TextStyle};

/// The fonts text can be set in: the ones the application already carries.
pub const FONTS: [(&str, &[u8]); 2] = [
    ("Ubuntu Light", epaint_default_fonts::UBUNTU_LIGHT),
    ("Hack (monospace)", epaint_default_fonts::HACK_REGULAR),
];

/// The panel over the sketch canvas for a command with numbers to enter.
#[derive(Debug, Clone, PartialEq)]
pub enum SketchPanel {
    RectPattern {
        nx: u32,
        ny: u32,
        dx: f64,
        dy: f64,
    },
    CircPattern {
        count: u32,
        angle: f64,
        cx: f64,
        cy: f64,
    },
    Text {
        text: String,
        height: f64,
        spacing: f64,
        x: f64,
        y: f64,
        angle: f64,
        font: usize,
    },
}

impl SketchPanel {
    pub fn rect_pattern() -> Self {
        SketchPanel::RectPattern {
            nx: 3,
            ny: 1,
            dx: 20.0,
            dy: 20.0,
        }
    }
    pub fn circ_pattern() -> Self {
        SketchPanel::CircPattern {
            count: 6,
            angle: 360.0,
            cx: 0.0,
            cy: 0.0,
        }
    }
    pub fn text() -> Self {
        SketchPanel::Text {
            text: "Text".into(),
            height: 10.0,
            spacing: 0.0,
            x: 0.0,
            y: 0.0,
            angle: 0.0,
            font: 0,
        }
    }
    fn title(&self) -> &'static str {
        match self {
            SketchPanel::RectPattern { .. } => "RECTANGULAR PATTERN",
            SketchPanel::CircPattern { .. } => "CIRCULAR PATTERN",
            SketchPanel::Text { .. } => "TEXT",
        }
    }
}

fn num(ui: &mut egui::Ui, label: &str, v: &mut f64, suffix: &str) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(label).color(TEXT_WEAK));
        ui.add(egui::DragValue::new(v).speed(0.1).suffix(suffix));
    });
}

impl DesignWorkspace {
    /// Project (`intersect == false`) or Intersect: the model's edges, or its cut by the sketch plane,
    /// become locked sketch curves.
    pub fn sketch_project(&mut self, project: &mut Project, intersect: bool) {
        let Some(frame) = self.sketch_frame else {
            return;
        };
        let ev = project.evaluation();
        let doc = project.document();
        let Some(bodies) = Bodies::new(&doc, &ev, &self.hidden_bodies) else {
            self.sketch.error = "there is no solid to project yet".into();
            return;
        };
        let curves = if intersect {
            sketch_project::intersect(&bodies.brep, &frame)
        } else {
            sketch_project::project(&bodies.brep, &frame)
        };
        let n = self.sketch.app.editor.insert_curves(&curves, false);
        if n == 0 {
            self.status = if intersect {
                "The sketch plane does not cut the model.".into()
            } else {
                "Nothing to project.".into()
            };
        } else {
            self.sketch.app.editor.pin_selection();
            self.status = format!(
                "{n} curves {} (locked in place).",
                if intersect {
                    "intersected"
                } else {
                    "projected"
                }
            );
        }
    }

    /// Do what the panel says to the sketch; returns the number of curves added.
    pub fn apply_sketch_panel(&mut self, panel: &SketchPanel) -> Result<usize, String> {
        let ed = &mut self.sketch.app.editor;
        match panel {
            SketchPanel::RectPattern { nx, ny, dx, dy } => {
                ed.pattern_rect(*nx as usize, *ny as usize, (*dx, 0.0), (0.0, *dy))
            }
            SketchPanel::CircPattern {
                count,
                angle,
                cx,
                cy,
            } => ed.pattern_circular((*cx, *cy), *count as usize, *angle),
            SketchPanel::Text {
                text,
                height,
                spacing,
                x,
                y,
                angle,
                font,
            } => text_curves(
                FONTS[(*font).min(FONTS.len() - 1)].1,
                text,
                &TextStyle {
                    height: *height,
                    spacing: *spacing,
                    origin: (*x, *y),
                    angle_deg: *angle,
                },
            )
            .map(|(curves, _)| ed.insert_curves(&curves, false)),
        }
    }

    /// The floating panel of the pattern and text commands.
    pub fn sketch_panel_window(&mut self, ctx: &egui::Context) {
        let Some(mut panel) = self.sketch_panel.take() else {
            return;
        };
        let mut open = true;
        let mut apply = false;
        let mut error = None;
        egui::Window::new(egui::RichText::new(panel.title()).size(11.5).strong())
            .open(&mut open)
            .collapsible(false)
            .default_pos(egui::pos2(330.0, 190.0))
            .show(ctx, |ui| {
                match &mut panel {
                    SketchPanel::RectPattern { nx, ny, dx, dy } => {
                        ui.label(
                            egui::RichText::new(
                                "Repeats the selected curves; counts include the original.",
                            )
                            .size(10.5)
                            .color(TEXT_WEAK),
                        );
                        ui.horizontal(|ui| {
                            ui.label("Columns");
                            ui.add(egui::DragValue::new(nx).range(1..=50));
                            ui.label("Rows");
                            ui.add(egui::DragValue::new(ny).range(1..=50));
                        });
                        num(ui, "Column spacing (x)", dx, " mm");
                        num(ui, "Row spacing (y)", dy, " mm");
                    }
                    SketchPanel::CircPattern {
                        count,
                        angle,
                        cx,
                        cy,
                    } => {
                        ui.label(
                            egui::RichText::new(
                                "Repeats the selected curves about a centre; 360° spaces them evenly.",
                            )
                            .size(10.5)
                            .color(TEXT_WEAK),
                        );
                        ui.horizontal(|ui| {
                            ui.label("Count");
                            ui.add(egui::DragValue::new(count).range(1..=360));
                        });
                        num(ui, "Angle", angle, "°");
                        num(ui, "Centre x", cx, " mm");
                        num(ui, "Centre y", cy, " mm");
                    }
                    SketchPanel::Text {
                        text,
                        height,
                        spacing,
                        x,
                        y,
                        angle,
                        font,
                    } => {
                        ui.add(
                            egui::TextEdit::multiline(text)
                                .desired_rows(2)
                                .desired_width(220.0),
                        );
                        egui::ComboBox::from_id_salt("text_font")
                            .selected_text(FONTS[(*font).min(FONTS.len() - 1)].0)
                            .show_ui(ui, |ui| {
                                for (i, (name, _)) in FONTS.iter().enumerate() {
                                    ui.selectable_value(font, i, *name);
                                }
                            });
                        num(ui, "Height (capitals)", height, " mm");
                        num(ui, "Letter spacing", spacing, "");
                        num(ui, "Position x", x, " mm");
                        num(ui, "Position y", y, " mm");
                        num(ui, "Angle", angle, "°");
                    }
                }
                if let Some(e) = &error {
                    ui.colored_label(crate::chrome::theme::BAD, e);
                }
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        apply = true;
                    }
                });
            });
        if apply {
            match self.apply_sketch_panel(&panel) {
                Ok(n) => {
                    self.status = format!("{n} curves added.");
                    open = false;
                }
                Err(e) => error = Some(e),
            }
        }
        if let Some(e) = error {
            self.status = e;
        }
        if open {
            self.sketch_panel = Some(panel);
        }
    }
}
