//! Section analysis: cut the model with a plane to look inside.

use super::geom;
use super::DesignWorkspace;
use crate::chrome::theme::TEXT_WEAK;
use crate::project::Project;
use eframe::egui;
use kw_feature::{FeatureId, FeatureSpec, Product};

/// The plane the model is cut by: an origin plane (0 = XY, 1 = YZ, 2 = XZ) or a construction plane,
/// moved `offset` along its normal; `flip` keeps the other side.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionState {
    pub which: u8,
    pub datum: Option<FeatureId>,
    pub offset: f64,
    pub flip: bool,
}

impl SectionState {
    pub fn new() -> Self {
        SectionState {
            which: 2,
            datum: None,
            offset: 0.0,
            flip: false,
        }
    }
}

impl Default for SectionState {
    fn default() -> Self {
        Self::new()
    }
}

impl DesignWorkspace {
    /// The cutting plane in world coordinates as `(unit normal, offset)`: what lies on the side the
    /// normal points to (`dot(n, p) > offset`) is cut away.
    pub fn section_plane(&self, project: &mut Project) -> Option<([f64; 3], f64)> {
        let s = self.section.as_ref()?;
        let (n, o) = match s.datum {
            Some(d) => {
                let ev = project.evaluation();
                match &ev.output(d)?.product {
                    Product::Datum(f) => {
                        ([f.z.x, f.z.y, f.z.z], [f.origin.x, f.origin.y, f.origin.z])
                    }
                    _ => return None,
                }
            }
            None => (geom::origin_frame(s.which as usize).n, [0.0; 3]),
        };
        let base = n[0] * o[0] + n[1] * o[1] + n[2] * o[2] + s.offset;
        Some(if s.flip {
            ([-n[0], -n[1], -n[2]], -base)
        } else {
            (n, base)
        })
    }

    /// The Section Analysis window.
    pub fn section_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        if self.section.is_none() {
            return;
        }
        let radius = self
            .viewport
            .model()
            .map(|m| m.scene.radius)
            .unwrap_or(40.0)
            * 1.2;
        let planes: Vec<(FeatureId, String)> = project
            .document()
            .features
            .iter()
            .filter(|f| matches!(f.spec, FeatureSpec::DatumPlane(_)) && !f.suppressed)
            .map(|f| (f.id, f.name.clone()))
            .collect();
        let mut open = true;
        let mut s = self.section.clone().unwrap();
        egui::Window::new(egui::RichText::new("SECTION ANALYSIS").size(11.5).strong())
            .open(&mut open)
            .collapsible(false)
            .default_pos(egui::pos2(330.0, 190.0))
            .default_width(280.0)
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(
                        "Cuts the model to look inside; the cut is shown as a cap.",
                    )
                    .size(10.5)
                    .color(TEXT_WEAK),
                );
                ui.horizontal(|ui| {
                    ui.label("Plane");
                    for (i, name) in ["XY", "YZ", "XZ"].iter().enumerate() {
                        if ui
                            .selectable_label(s.datum.is_none() && s.which == i as u8, *name)
                            .clicked()
                        {
                            s.which = i as u8;
                            s.datum = None;
                        }
                    }
                });
                if !planes.is_empty() {
                    let name = planes
                        .iter()
                        .find(|(id, _)| Some(*id) == s.datum)
                        .map(|(_, n)| n.clone())
                        .unwrap_or_else(|| "construction plane".into());
                    egui::ComboBox::from_id_salt("section_plane")
                        .selected_text(name)
                        .show_ui(ui, |ui| {
                            for (id, n) in &planes {
                                ui.selectable_value(&mut s.datum, Some(*id), n);
                            }
                        });
                }
                ui.add(
                    egui::Slider::new(&mut s.offset, -radius..=radius)
                        .text("Offset")
                        .suffix(" mm"),
                );
                ui.checkbox(&mut s.flip, "Flip (keep the other side)");
            });
        if open {
            self.section = Some(s);
        } else {
            self.section = None;
        }
    }
}
