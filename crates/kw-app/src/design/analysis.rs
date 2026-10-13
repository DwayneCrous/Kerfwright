//! Surface analysis in the Design workspace: zebra stripes, curvature maps and draft analysis shown on
//! the model, with a colour-bar legend and a window to set them up.

use super::DesignWorkspace;
use crate::chrome::theme::TEXT_WEAK;
use crate::viewport::{AnalysisKind, AnalysisView};
use eframe::egui;
use kw_analysis::{ramp, CurvKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AKind {
    Zebra,
    Gaussian,
    Mean,
    Max,
    Min,
    Draft,
}

impl AKind {
    pub const ALL: [AKind; 6] = [
        AKind::Zebra,
        AKind::Gaussian,
        AKind::Mean,
        AKind::Max,
        AKind::Min,
        AKind::Draft,
    ];

    pub fn name(self) -> &'static str {
        match self {
            AKind::Zebra => "Zebra stripes",
            AKind::Gaussian => "Gaussian curvature",
            AKind::Mean => "Mean curvature",
            AKind::Max => "Maximum curvature",
            AKind::Min => "Minimum curvature",
            AKind::Draft => "Draft analysis",
        }
    }

    pub fn is_curvature(self) -> bool {
        matches!(
            self,
            AKind::Gaussian | AKind::Mean | AKind::Max | AKind::Min
        )
    }

    /// The curvature this kind shows, if it is one.
    pub fn curvature(self) -> Option<CurvKind> {
        match self {
            AKind::Gaussian => Some(CurvKind::Gaussian),
            AKind::Mean => Some(CurvKind::Mean),
            AKind::Max => Some(CurvKind::Max),
            AKind::Min => Some(CurvKind::Min),
            _ => None,
        }
    }
}

/// Directions offered for the zebra stripes and the draft pull direction.
pub const DIRS: [(&str, [f64; 3]); 6] = [
    ("+X", [1.0, 0.0, 0.0]),
    ("-X", [-1.0, 0.0, 0.0]),
    ("+Y", [0.0, 1.0, 0.0]),
    ("-Y", [0.0, -1.0, 0.0]),
    ("+Z", [0.0, 0.0, 1.0]),
    ("-Z", [0.0, 0.0, -1.0]),
];

#[derive(Debug, Clone, PartialEq)]
pub struct AnalysisState {
    pub kind: AKind,
    /// Choose the curvature range from the model.
    pub auto: bool,
    /// Curvature at the ends of the ramp (1/mm) when not `auto`.
    pub limit: f32,
    /// Index into [`DIRS`]: the stripe axis or the pull direction.
    pub dir: usize,
    pub freq: f32,
    /// Draft angle (degrees) below which a face is flagged.
    pub draft_limit: f32,
}

impl AnalysisState {
    pub fn new(kind: AKind) -> Self {
        AnalysisState {
            kind,
            auto: true,
            limit: 0.1,
            dir: if kind == AKind::Zebra { 0 } else { 4 },
            freq: 12.0,
            draft_limit: 3.0,
        }
    }
}

impl DesignWorkspace {
    /// What the viewport should show for the current analysis, if any.
    pub fn analysis_view(&self) -> Option<AnalysisView> {
        let a = self.analysis.as_ref()?;
        Some(AnalysisView {
            kind: match a.kind {
                AKind::Zebra => AnalysisKind::Zebra,
                AKind::Draft => AnalysisKind::Draft,
                k => AnalysisKind::Curvature(k.curvature()?),
            },
            auto: a.auto,
            limit: a.limit,
            axis: DIRS[a.dir.min(5)].1,
            zebra_freq: a.freq,
            draft_limit: a.draft_limit,
        })
    }

    /// The ribbon's Zebra / Curvature / Draft buttons: switch the analysis on, or off when it is already showing.
    pub fn toggle_analysis(&mut self, kind: AKind) {
        let same_family = |a: AKind| match kind {
            AKind::Zebra | AKind::Draft => a == kind,
            _ => a.is_curvature(),
        };
        self.analysis = match &self.analysis {
            Some(a) if same_family(a.kind) => None,
            _ => Some(AnalysisState::new(kind)),
        };
    }

    /// The window that sets the analysis up.
    pub fn analysis_window(&mut self, ctx: &egui::Context) {
        let Some(mut a) = self.analysis.clone() else {
            return;
        };
        let mut open = true;
        egui::Window::new(egui::RichText::new("SURFACE ANALYSIS").size(11.5).strong())
            .open(&mut open)
            .anchor(egui::Align2::LEFT_BOTTOM, [270.0, -90.0])
            .default_width(260.0)
            .show(ctx, |ui| {
                egui::Grid::new("analysis_grid").num_columns(2).show(ui, |ui| {
                    ui.label("Analysis");
                    egui::ComboBox::from_id_salt("analysis_kind")
                        .selected_text(a.kind.name())
                        .show_ui(ui, |ui| {
                            for k in AKind::ALL {
                                if ui.selectable_value(&mut a.kind, k, k.name()).clicked() {
                                    // each kind has its own sensible direction
                                    if k == AKind::Zebra && a.dir > 1 {
                                        a.dir = 0;
                                    }
                                }
                            }
                        });
                    ui.end_row();
                    match a.kind {
                        AKind::Zebra => {
                            ui.label("Stripes along");
                            dir_combo(ui, "zebra_dir", &mut a.dir);
                            ui.end_row();
                            ui.label("Stripe density");
                            ui.add(egui::Slider::new(&mut a.freq, 2.0..=40.0));
                            ui.end_row();
                        }
                        AKind::Draft => {
                            ui.label("Pull direction");
                            dir_combo(ui, "draft_dir", &mut a.dir);
                            ui.end_row();
                            ui.label("Needs draft below");
                            ui.add(egui::Slider::new(&mut a.draft_limit, 0.0..=15.0).suffix(" deg"));
                            ui.end_row();
                        }
                        _ => {
                            ui.label("Range");
                            ui.checkbox(&mut a.auto, "from the model");
                            ui.end_row();
                            if !a.auto {
                                ui.label("Limit (1/mm)");
                                ui.add(
                                    egui::DragValue::new(&mut a.limit)
                                        .speed(0.001)
                                        .range(1e-5..=100.0)
                                        .max_decimals(5),
                                );
                                ui.end_row();
                            }
                        }
                    }
                });
                ui.label(
                    egui::RichText::new(match a.kind {
                        AKind::Zebra => "Stripes that run on unbroken across a join mean the surfaces are tangent there.",
                        AKind::Draft => "Red faces have too little draft for the pull direction.",
                        _ => "Blue is concave, red convex; white is flat.",
                    })
                    .size(10.5)
                    .color(TEXT_WEAK),
                );
            });
        self.analysis = if open { Some(a) } else { None };
    }

    /// The colour bar of the current analysis in a corner of the viewport.
    pub fn analysis_legend(&self, painter: &egui::Painter, rect: egui::Rect) {
        let Some(a) = &self.analysis else { return };
        if a.kind == AKind::Zebra {
            return;
        }
        let bar = egui::Rect::from_min_size(
            egui::pos2(rect.left() + 28.0, rect.top() + 64.0),
            egui::vec2(16.0, 180.0),
        );
        painter.rect_filled(bar.expand(6.0), 4.0, egui::Color32::from_black_alpha(110));
        let steps = 60;
        let limit = self.viewport.curvature_limit();
        for k in 0..steps {
            // top of the bar is the high end
            let t = 1.0 - (k as f32 + 0.5) / steps as f32;
            let c = if a.kind == AKind::Draft {
                ramp::draft(-90.0 + 180.0 * t, a.draft_limit)
            } else {
                ramp::diverging(2.0 * t - 1.0)
            };
            let y0 = bar.top() + bar.height() * k as f32 / steps as f32;
            let y1 = bar.top() + bar.height() * (k + 1) as f32 / steps as f32;
            painter.rect_filled(
                egui::Rect::from_min_max(
                    egui::pos2(bar.left(), y0),
                    egui::pos2(bar.right(), y1 + 0.5),
                ),
                0.0,
                egui::Color32::from_rgb(
                    (c[0] * 255.0) as u8,
                    (c[1] * 255.0) as u8,
                    (c[2] * 255.0) as u8,
                ),
            );
        }
        let label = |y: f32, text: String| {
            painter.text(
                egui::pos2(bar.right() + 5.0, y),
                egui::Align2::LEFT_CENTER,
                text,
                egui::FontId::proportional(10.5),
                egui::Color32::WHITE,
            );
        };
        if a.kind == AKind::Draft {
            label(bar.top(), "+90°".into());
            label(bar.center().y, "0°".into());
            label(bar.bottom(), "-90°".into());
        } else {
            label(bar.top(), format!("{limit:.3}"));
            label(bar.center().y, "0".into());
            label(bar.bottom(), format!("-{limit:.3}"));
            painter.text(
                egui::pos2(bar.center().x, bar.top() - 12.0),
                egui::Align2::CENTER_CENTER,
                "1/mm",
                egui::FontId::proportional(10.5),
                egui::Color32::WHITE,
            );
        }
    }
}

fn dir_combo(ui: &mut egui::Ui, id: &str, dir: &mut usize) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(DIRS[(*dir).min(5)].0)
        .show_ui(ui, |ui| {
            for (i, (name, _)) in DIRS.iter().enumerate() {
                ui.selectable_value(dir, i, *name);
            }
        });
}
