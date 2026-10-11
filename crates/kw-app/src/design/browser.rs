//! The browser tree at the left: document settings, origin, sketches, bodies and features.

use super::{geom, Action, DesignWorkspace};
use crate::chrome::theme::{ACCENT_SOFT, BAD, TEXT, TEXT_WEAK};
use crate::chrome::{icons, Icon};
use crate::project::Project;
use eframe::egui::{self, vec2, Sense, Ui};
use kw_feature::{FeatureId, FeatureSpec, FeatureState};

/// One tree row: eye toggle (optional), icon, label. Returns (row response, eye clicked).
fn row(
    ui: &mut Ui,
    icon: Icon,
    label: &str,
    selected: bool,
    eye: Option<bool>,
    error: bool,
    weak: bool,
) -> (egui::Response, bool) {
    let h = 20.0;
    let (rect, resp) = ui.allocate_at_least(vec2(ui.available_width(), h), Sense::click());
    if selected {
        ui.painter().rect_filled(rect, 2.0, ACCENT_SOFT);
    } else if resp.hovered() {
        ui.painter()
            .rect_filled(rect, 2.0, egui::Color32::from_black_alpha(10));
    }
    let mut eye_clicked = false;
    let mut x = rect.left() + 2.0;
    if let Some(on) = eye {
        let er = egui::Rect::from_min_size(egui::pos2(x, rect.top() + 2.0), vec2(16.0, 16.0));
        let er_resp = ui.interact(er, resp.id.with("eye"), Sense::click());
        icons::paint(ui.painter(), er, Icon::Eye, on);
        if !on {
            ui.painter().line_segment(
                [er.left_bottom(), er.right_top()],
                egui::Stroke::new(1.5, TEXT_WEAK),
            );
        }
        eye_clicked = er_resp.clicked();
        x += 20.0;
    }
    icons::paint(
        ui.painter(),
        egui::Rect::from_min_size(egui::pos2(x, rect.top() + 2.0), vec2(16.0, 16.0)),
        icon,
        true,
    );
    x += 20.0;
    let color = if error {
        BAD
    } else if weak {
        TEXT_WEAK
    } else {
        TEXT
    };
    ui.painter().text(
        egui::pos2(x, rect.center().y),
        egui::Align2::LEFT_CENTER,
        label,
        egui::FontId::proportional(12.0),
        color,
    );
    (resp, eye_clicked)
}

impl DesignWorkspace {
    pub fn browser_ui(&mut self, ui: &mut Ui, project: &mut Project) {
        let doc = project.document();
        let ev = project.evaluation();
        ui.add_space(3.0);
        ui.label(
            egui::RichText::new("BROWSER")
                .size(11.0)
                .strong()
                .color(TEXT_WEAK),
        );
        ui.separator();
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::CollapsingHeader::new(egui::RichText::new("Document Settings").size(12.0))
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new("Units: Millimeter")
                                .size(11.5)
                                .color(TEXT_WEAK),
                        );
                    });
                egui::CollapsingHeader::new(egui::RichText::new("Origin").size(12.0))
                    .default_open(false)
                    .show(ui, |ui| {
                        for (k, (name, _)) in geom::ORIGIN_PLANES.iter().enumerate() {
                            let (_, eye) = row(
                                ui,
                                Icon::Plane,
                                name,
                                false,
                                Some(self.plane_visible[k]),
                                false,
                                false,
                            );
                            if eye {
                                self.plane_visible[k] = !self.plane_visible[k];
                            }
                        }
                    });
                let sketches: Vec<(FeatureId, String, bool)> = doc
                    .features
                    .iter()
                    .filter(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                    .map(|f| (f.id, f.name.clone(), f.suppressed))
                    .collect();
                egui::CollapsingHeader::new(
                    egui::RichText::new(format!("Sketches ({})", sketches.len())).size(12.0),
                )
                .default_open(true)
                .show(ui, |ui| {
                    for (id, name, sup) in &sketches {
                        let (resp, eye) = row(
                            ui,
                            Icon::Sketch,
                            name,
                            self.model.selected == Some(*id),
                            Some(!self.hidden_sketches.contains(id)),
                            ev.error(*id).is_some(),
                            *sup,
                        );
                        if eye {
                            self.actions.push(Action::ToggleSketch(*id));
                        }
                        self.row_interactions(&resp, *id, *sup);
                    }
                });
                let datums: Vec<(FeatureId, String, Icon, bool)> = doc
                    .features
                    .iter()
                    .filter_map(|f| {
                        let icon = match f.spec {
                            FeatureSpec::DatumPlane(_) => Icon::Plane,
                            FeatureSpec::DatumAxis(_) => Icon::Axis,
                            FeatureSpec::DatumPoint(_) => Icon::PointDatum,
                            _ => return None,
                        };
                        Some((f.id, f.name.clone(), icon, f.suppressed))
                    })
                    .collect();
                if !datums.is_empty() {
                    egui::CollapsingHeader::new(
                        egui::RichText::new(format!("Construction ({})", datums.len())).size(12.0),
                    )
                    .default_open(true)
                    .show(ui, |ui| {
                        for (id, name, icon, sup) in &datums {
                            let (resp, eye) = row(
                                ui,
                                *icon,
                                name,
                                self.model.selected == Some(*id),
                                Some(!self.hidden_datums.contains(id)),
                                ev.error(*id).is_some(),
                                *sup,
                            );
                            if eye && !self.hidden_datums.remove(id) {
                                self.hidden_datums.insert(*id);
                            }
                            self.row_interactions(&resp, *id, *sup);
                        }
                    });
                }
                let infos = crate::bodies::summaries(&doc);
                egui::CollapsingHeader::new(
                    egui::RichText::new(format!("Bodies ({})", infos.len())).size(12.0),
                )
                .default_open(true)
                .show(ui, |ui| {
                    for info in &infos {
                        let hidden = self.hidden_bodies.contains(&info.root);
                        let (resp, eye) = row(
                            ui,
                            Icon::Body,
                            &info.name,
                            self.active_body == Some(info.root),
                            Some(!hidden),
                            false,
                            hidden,
                        );
                        // the body's colour next to its name
                        let sw = egui::Rect::from_center_size(
                            egui::pos2(resp.rect.right() - 12.0, resp.rect.center().y),
                            vec2(10.0, 10.0),
                        );
                        ui.painter().rect_filled(
                            sw,
                            2.0,
                            egui::Color32::from_rgb(info.colour[0], info.colour[1], info.colour[2]),
                        );
                        if eye && !self.hidden_bodies.remove(&info.root) {
                            self.hidden_bodies.insert(info.root);
                        }
                        if resp.clicked() {
                            self.active_body = Some(info.root);
                        }
                        if resp.double_clicked() {
                            self.body_props = Some(info.root);
                        }
                        let root = info.root;
                        resp.context_menu(|ui| {
                            if ui.button("Properties…").clicked() {
                                self.body_props = Some(root);
                                ui.close();
                            }
                            if ui.button("Make active").clicked() {
                                self.active_body = Some(root);
                                ui.close();
                            }
                            if ui.button("Isolate").clicked() {
                                self.hidden_bodies = infos
                                    .iter()
                                    .map(|b| b.root)
                                    .filter(|r| *r != root)
                                    .collect();
                                ui.close();
                            }
                            if ui.button("Show all").clicked() {
                                self.hidden_bodies.clear();
                                ui.close();
                            }
                        });
                    }
                });
                let features: Vec<(FeatureId, String, &'static str, bool)> = doc
                    .features
                    .iter()
                    .filter(|f| !matches!(f.spec, FeatureSpec::Sketch(_)))
                    .map(|f| (f.id, f.name.clone(), f.spec.kind_name(), f.suppressed))
                    .collect();
                egui::CollapsingHeader::new(
                    egui::RichText::new(format!("Features ({})", features.len())).size(12.0),
                )
                .default_open(true)
                .show(ui, |ui| {
                    for (id, name, kind, sup) in &features {
                        let failed = matches!(ev.state(*id), Some(FeatureState::Failed { .. }));
                        let icon = super::timeline::icon_for(kind);
                        let (resp, _) = row(
                            ui,
                            icon,
                            name,
                            self.model.selected == Some(*id),
                            None,
                            failed,
                            *sup,
                        );
                        self.row_interactions(&resp, *id, *sup);
                    }
                });
            });
    }

    /// Click selects, double-click edits, right-click offers the common commands.
    pub(crate) fn row_interactions(
        &mut self,
        resp: &egui::Response,
        id: FeatureId,
        suppressed: bool,
    ) {
        if resp.clicked() {
            self.actions.push(Action::Select(id));
        }
        if resp.double_clicked() {
            self.actions.push(Action::Edit(id));
        }
        resp.context_menu(|ui| {
            if ui.button("Edit").clicked() {
                self.actions.push(Action::Edit(id));
                ui.close();
            }
            if ui
                .button(if suppressed { "Unsuppress" } else { "Suppress" })
                .clicked()
            {
                self.actions.push(Action::Suppress(id, !suppressed));
                ui.close();
            }
            if ui.button("Roll back to here").clicked() {
                self.actions.push(Action::RollbackTo(id));
                ui.close();
            }
            let n = self.dep_counts.get(&id).copied().unwrap_or(0);
            let label = match n {
                0 => "Delete".to_string(),
                1 => "Delete (and 1 dependent step)".to_string(),
                n => format!("Delete (and {n} dependent steps)"),
            };
            if ui.button(label).clicked() {
                self.actions.push(Action::Delete(id));
                ui.close();
            }
        });
    }
}
