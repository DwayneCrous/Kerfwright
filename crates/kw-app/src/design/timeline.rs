//! The timeline along the bottom: one chip per feature, drag to reorder, a draggable rollback marker.

use super::{Action, DesignWorkspace};
use crate::chrome::theme::{ACCENT, ACCENT_SOFT, BAD, BORDER, CHROME, TEXT_WEAK};
use crate::chrome::{icons, Icon};
use crate::project::Project;
use eframe::egui::{self, vec2, Sense, Ui};
use kw_feature::FeatureState;

#[derive(Default)]
pub struct State {
    /// The chip being dragged.
    pub drag: Option<usize>,
}

pub fn icon_for(kind: &str) -> Icon {
    match kind {
        "Sketch" => Icon::Sketch,
        "Extrude" => Icon::Extrude,
        "Revolve" => Icon::Revolve,
        "Hole" => Icon::Hole,
        "Fillet" | "VariableFillet" => Icon::Fillet,
        "Chamfer" => Icon::Chamfer,
        "Shell" => Icon::Shell,
        "LinearPattern" => Icon::Pattern,
        "CircularPattern" => Icon::CircularPattern,
        "Mirror" => Icon::Mirror,
        "Combine" => Icon::Combine,
        "Parameter" => Icon::Parameters,
        "DatumPlane" => Icon::Plane,
        "DatumAxis" => Icon::Axis,
        "DatumPoint" => Icon::PointDatum,
        "Sweep" => Icon::Sweep,
        "Loft" | "Ruled" => Icon::Loft,
        "Patch" => Icon::Plane,
        "Draft" => Icon::Draft,
        "Thread" => Icon::Thread,
        "Offset" => Icon::OffsetBody,
        "MoveBody" => Icon::Move,
        "CopyBody" => Icon::Component,
        "ScaleBody" => Icon::Scale,
        "SplitBody" => Icon::Split,
        "Import" => Icon::Folder,
        "SheetMetal" => Icon::Flange,
        "Subd" => Icon::FormBox,
        _ => Icon::Generic,
    }
}

impl DesignWorkspace {
    pub fn timeline_ui(&mut self, ui: &mut Ui, project: &mut Project) {
        let doc = project.document();
        let ev = project.evaluation();
        let n = doc.features.len();
        egui::Panel::bottom("design_timeline")
            .exact_size(52.0)
            .frame(
                egui::Frame::new()
                    .fill(CHROME)
                    .stroke(egui::Stroke::new(1.0, BORDER)),
            )
            .show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing.x = 3.0;
                    // playback-style controls
                    let ids: Vec<_> = doc.features.iter().map(|f| f.id).collect();
                    if ui
                        .small_button("|<")
                        .on_hover_text("Roll back to the start")
                        .clicked()
                    {
                        if let Some(first) = ids.first() {
                            self.actions.push(Action::RollbackTo(*first));
                        }
                    }
                    if ui.small_button("<").on_hover_text("Step back").clicked() {
                        let active = doc.rollback.unwrap_or(n);
                        if active > 1 {
                            self.actions.push(Action::RollbackTo(ids[active - 2]));
                        }
                    }
                    if ui.small_button(">").on_hover_text("Step forward").clicked() {
                        match doc.rollback {
                            Some(r) if r + 1 < n => self.actions.push(Action::RollbackTo(ids[r])),
                            Some(_) => self.actions.push(Action::ClearRollback),
                            None => {}
                        }
                    }
                    if ui
                        .small_button(">|")
                        .on_hover_text("Roll forward to the end")
                        .clicked()
                    {
                        self.actions.push(Action::ClearRollback);
                    }
                    ui.add_space(8.0);
                    egui::ScrollArea::horizontal()
                        .id_salt("timeline_scroll")
                        .auto_shrink([false, true])
                        .show(ui, |ui| {
                            ui.horizontal_centered(|ui| {
                                ui.spacing_mut().item_spacing.x = 3.0;
                                let mut rects: Vec<egui::Rect> = Vec::with_capacity(n);
                                let mut marker_drop: Option<egui::Pos2> = None;
                                let marker_at = doc.rollback.unwrap_or(n);
                                for (i, f) in doc.features.iter().enumerate() {
                                    if i == marker_at {
                                        marker_drop = marker_drop.or(self.marker(ui, &doc));
                                    }
                                    let (rect, resp) = ui.allocate_exact_size(
                                        vec2(30.0, 30.0),
                                        Sense::click_and_drag(),
                                    );
                                    rects.push(rect);
                                    let failed =
                                        matches!(ev.state(f.id), Some(FeatureState::Failed { .. }));
                                    let beyond = doc.rollback.is_some_and(|r| i >= r);
                                    let sel = self.model.selected == Some(f.id);
                                    let p = ui.painter();
                                    let dragging = self.timeline.drag == Some(i);
                                    p.rect(
                                        rect,
                                        3.0,
                                        if sel {
                                            ACCENT_SOFT
                                        } else {
                                            egui::Color32::WHITE
                                        },
                                        egui::Stroke::new(
                                            if failed || sel { 1.6 } else { 1.0 },
                                            if failed {
                                                BAD
                                            } else if sel {
                                                ACCENT
                                            } else {
                                                BORDER
                                            },
                                        ),
                                        egui::StrokeKind::Inside,
                                    );
                                    icons::paint(
                                        p,
                                        rect.shrink(3.0),
                                        icon_for(f.spec.kind_name()),
                                        !(f.suppressed || beyond) && !dragging,
                                    );
                                    let tip = format!(
                                        "{} - {}{}",
                                        f.spec.kind_name(),
                                        f.name,
                                        ev.error(f.id)
                                            .map(|e| format!("\n{e}"))
                                            .unwrap_or_default()
                                    );
                                    let resp = resp.on_hover_text(tip);
                                    if resp.drag_started() {
                                        self.timeline.drag = Some(i);
                                    }
                                    self.row_interactions(&resp, f.id, f.suppressed);
                                }
                                if marker_at >= n {
                                    marker_drop = marker_drop.or(self.marker(ui, &doc));
                                }
                                if let Some(pos) = marker_drop {
                                    // chips to the left of the pointer stay active
                                    let keep =
                                        rects.iter().filter(|r| r.center().x < pos.x).count();
                                    if keep >= n {
                                        self.actions.push(Action::ClearRollback);
                                    } else {
                                        self.actions.push(Action::RollbackTo(
                                            doc.features[keep.max(1) - 1].id,
                                        ));
                                    }
                                }
                                // drop target while dragging
                                if let Some(from) = self.timeline.drag {
                                    if let Some(pos) = ui.ctx().pointer_latest_pos() {
                                        // the other chips left of the pointer decide the final position
                                        let others: Vec<egui::Rect> = rects
                                            .iter()
                                            .enumerate()
                                            .filter(|(i, _)| *i != from)
                                            .map(|(_, r)| *r)
                                            .collect();
                                        let to =
                                            others.iter().filter(|r| r.center().x < pos.x).count();
                                        let x = match others.get(to) {
                                            Some(r) => r.left() - 1.5,
                                            None => others
                                                .last()
                                                .map(|r| r.right() + 1.5)
                                                .unwrap_or(0.0),
                                        };
                                        if let Some(r) = rects.first() {
                                            ui.painter().line_segment(
                                                [
                                                    egui::pos2(x, r.top() - 2.0),
                                                    egui::pos2(x, r.bottom() + 2.0),
                                                ],
                                                egui::Stroke::new(2.0, ACCENT),
                                            );
                                        }
                                        if ui.ctx().input(|i| i.pointer.any_released()) {
                                            if to != from {
                                                self.actions
                                                    .push(Action::Move(doc.features[from].id, to));
                                            }
                                            self.timeline.drag = None;
                                        }
                                    } else {
                                        self.timeline.drag = None;
                                    }
                                }
                            });
                        });
                });
            });
    }

    /// The rollback marker: a bar the user can drag to roll the model back and forward. Returns where it was
    /// dropped.
    fn marker(&mut self, ui: &mut Ui, doc: &kw_feature::Document) -> Option<egui::Pos2> {
        let (rect, resp) = ui.allocate_exact_size(vec2(9.0, 38.0), Sense::click_and_drag());
        let c = if doc.rolled_back() {
            ACCENT
        } else {
            TEXT_WEAK.gamma_multiply(0.7)
        };
        ui.painter().rect_filled(
            egui::Rect::from_center_size(rect.center(), vec2(3.0, 32.0)),
            1.5,
            c,
        );
        ui.painter().rect_filled(
            egui::Rect::from_center_size(rect.center_top() + vec2(0.0, 4.0), vec2(9.0, 8.0)),
            2.0,
            c,
        );
        let resp = resp.on_hover_text("Drag to roll the model back or forward");
        if resp.drag_stopped() {
            return ui.ctx().pointer_latest_pos();
        }
        if resp.clicked() && doc.rollback.is_some() {
            self.actions.push(Action::ClearRollback);
        }
        None
    }
}
