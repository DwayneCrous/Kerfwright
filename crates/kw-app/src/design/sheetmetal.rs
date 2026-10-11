//! Sheet metal: the flat pattern of a strip (or of a recognised folded body), its bend table and DXF.

use super::assembly::FileRequest;
use super::tools::{SheetRow, Tool};
use super::DesignWorkspace;
use crate::chrome::theme::TEXT_WEAK;
use crate::model::param_text;
use crate::project::Project;
use eframe::egui;
use kw_feature::{FeatureId, FeatureSpec};
use kw_sheet::{FlatItem, FlatPattern, SheetStrip};

/// K-factor assumed when a folded body is recognised (its recipe is not known).
pub const RECOGNISED_K: f64 = 0.44;

pub struct FlatState {
    pub title: String,
    pub pattern: FlatPattern,
}

/// A dialog that rebuilds `strip`.
pub fn strip_tool(strip: &SheetStrip) -> Tool {
    let mm = |v: f64| param_text(&kw_expr::Param::length(v));
    Tool::SheetMetal {
        thickness: mm(strip.spec.thickness),
        k_factor: format!("{}", strip.spec.k_factor),
        radius: mm(strip.spec.default_radius),
        width: mm(strip.width),
        rows: strip
            .segs
            .iter()
            .map(|g| match g {
                kw_sheet::Seg::Flat { length } => SheetRow {
                    bend: false,
                    a: mm(*length),
                    r: String::new(),
                },
                kw_sheet::Seg::Bend { angle, radius } => SheetRow {
                    bend: true,
                    a: param_text(&kw_expr::Param::angle_deg(angle.to_degrees())),
                    r: radius.map(mm).unwrap_or_default(),
                },
            })
            .collect(),
    }
}

/// Starting strips of the SHEET METAL tab.
pub fn preset(kind: &str) -> Tool {
    let mut t = strip_tool(
        &SheetStrip::new(
            kw_sheet::SheetSpec {
                thickness: 2.0,
                k_factor: RECOGNISED_K,
                default_radius: 2.0,
            },
            30.0,
        )
        .flat(40.0),
    );
    if let Tool::SheetMetal { rows, .. } = &mut t {
        match kind {
            // an L bracket
            "l" => {
                rows.push(SheetRow::bend("90 deg"));
                rows.push(SheetRow::flat("25 mm"));
            }
            // a U channel
            "u" => {
                rows.push(SheetRow::bend("90 deg"));
                rows.push(SheetRow::flat("30 mm"));
                rows.push(SheetRow::bend("90 deg"));
                rows.push(SheetRow::flat("40 mm"));
            }
            // a Z bracket
            "z" => {
                rows.push(SheetRow::bend("90 deg"));
                rows.push(SheetRow::flat("25 mm"));
                rows.push(SheetRow::bend("-90 deg"));
                rows.push(SheetRow::flat("30 mm"));
            }
            _ => {}
        }
    }
    t
}

impl DesignWorkspace {
    /// Work out the flat pattern of the selected (or last) sheet-metal feature; of the solid itself when
    /// there is none and it can be read as a bent strip.
    pub fn sheet_flat_pattern(&mut self, project: &mut Project) -> Result<(), String> {
        let ev = project.evaluation();
        let doc = project.document();
        let sheet: Option<FeatureId> = self
            .model
            .selected
            .filter(|s| {
                doc.get(*s)
                    .is_some_and(|f| matches!(f.spec, FeatureSpec::SheetMetal(_)))
            })
            .or_else(|| {
                doc.features
                    .iter()
                    .rev()
                    .find(|f| matches!(f.spec, FeatureSpec::SheetMetal(_)) && !f.suppressed)
                    .map(|f| f.id)
            });
        let (title, pattern) = match sheet {
            Some(id) => (
                doc.name_of(id),
                ev.flat_pattern(&doc, id).map_err(|e| e.to_string())?,
            ),
            None => {
                let (_, out) = ev
                    .final_solid()
                    .ok_or("there is no sheet-metal part or solid to unfold")?;
                let (rep, body) = (out.brep().ok_or("no body")?, out.body().ok_or("no body")?);
                let strip = kw_sheet::recognize(rep, body, RECOGNISED_K)
                    .map_err(|e| format!("the solid is not a bent sheet: {e}"))?;
                (
                    "recognised body".into(),
                    kw_sheet::unfold(&strip).map_err(|e| e.to_string())?,
                )
            }
        };
        self.flat = Some(FlatState { title, pattern });
        Ok(())
    }

    /// The flat pattern window: sizes, the bend table, DXF export and refolding.
    pub fn flat_window(&mut self, ctx: &egui::Context) {
        let Some(state) = self.flat.as_ref() else {
            return;
        };
        let mut open = true;
        let mut export = false;
        let mut refold = false;
        egui::Window::new(egui::RichText::new("FLAT PATTERN").size(11.5).strong())
            .open(&mut open)
            .collapsible(false)
            .default_pos(egui::pos2(330.0, 190.0))
            .default_width(320.0)
            .show(ctx, |ui| {
                let p = &state.pattern;
                ui.label(egui::RichText::new(&state.title).strong());
                egui::Grid::new("flat_info")
                    .num_columns(2)
                    .spacing([14.0, 3.0])
                    .show(ui, |ui| {
                        let mut row = |k: &str, v: String| {
                            ui.label(egui::RichText::new(k).color(TEXT_WEAK));
                            ui.label(v);
                            ui.end_row();
                        };
                        row("Length", format!("{:.3} mm", p.length()));
                        row("Width", format!("{:.3} mm", p.width));
                        row("Thickness", format!("{:.3} mm", p.spec.thickness));
                        row("K-factor", format!("{:.3}", p.spec.k_factor));
                        row("Blank area", format!("{:.2} mm²", p.length() * p.width));
                    });
                ui.add_space(4.0);
                ui.label(egui::RichText::new("Bends").strong());
                let bends = p.bends();
                if bends.is_empty() {
                    ui.label(egui::RichText::new("none").color(TEXT_WEAK));
                }
                egui::Grid::new("flat_bends").num_columns(5).show(ui, |ui| {
                    for h in ["#", "at", "angle", "radius", "allowance"] {
                        ui.label(egui::RichText::new(h).color(TEXT_WEAK).size(10.5));
                    }
                    ui.end_row();
                    for (i, (s, e, a, r)) in bends.iter().enumerate() {
                        ui.label(format!("{}", i + 1));
                        ui.label(format!("{:.2}", (s + e) / 2.0));
                        ui.label(format!("{:.1}°", a.to_degrees()));
                        ui.label(format!("{r:.2}"));
                        ui.label(format!("{:.3}", e - s));
                        ui.end_row();
                    }
                });
                // a sketch of the blank with its bend lines
                let (rect, _) = ui.allocate_exact_size(
                    egui::vec2(ui.available_width().min(300.0), 60.0),
                    egui::Sense::hover(),
                );
                let painter = ui.painter_at(rect);
                let scale = ((rect.width() - 8.0) as f64 / p.length().max(1e-9))
                    .min((rect.height() - 8.0) as f64 / p.width.max(1e-9));
                let o = rect.left_top() + egui::vec2(4.0, 4.0);
                let pt = |x: f64, y: f64| o + egui::vec2((x * scale) as f32, (y * scale) as f32);
                painter.rect_stroke(
                    egui::Rect::from_two_pos(pt(0.0, 0.0), pt(p.length(), p.width)),
                    0.0,
                    egui::Stroke::new(1.5, crate::chrome::theme::TEXT),
                    egui::StrokeKind::Inside,
                );
                for it in &p.items {
                    if let FlatItem::Bend { start, end, .. } = it {
                        painter.rect_filled(
                            egui::Rect::from_two_pos(pt(*start, 0.0), pt(*end, p.width)),
                            0.0,
                            crate::chrome::theme::ACCENT_SOFT,
                        );
                    }
                }
                ui.horizontal(|ui| {
                    if ui.button("Export DXF…").clicked() {
                        export = true;
                    }
                    if ui
                        .button("Refold")
                        .on_hover_text("Open the strip that folds back from this pattern")
                        .clicked()
                    {
                        refold = true;
                    }
                });
            });
        if export {
            self.asm.request = Some(FileRequest::ExportDxf);
        }
        if refold {
            if let Some(s) = &self.flat {
                let strip = kw_sheet::refold(&s.pattern);
                self.actions.push(super::Action::Open(strip_tool(&strip)));
            }
        }
        if !open {
            self.flat = None;
        }
    }
}
