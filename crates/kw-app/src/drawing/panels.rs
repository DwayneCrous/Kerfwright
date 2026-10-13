//! The browser, the properties panel, the sheet tabs and the dialogs of the Drawing workspace.

use super::{orientation_name, style_name, Dialog, DrawingWorkspace, ViewForm, ORIENTATIONS};
use crate::chrome::theme::{BAD, TEXT_WEAK};
use crate::project::Project;
use eframe::egui;
use kw_drawing::update::STANDARD_SCALES;
use kw_drawing::{scale_label, PaperSize, Standard, ViewKind, ViewStyle};

fn view_name(v: &kw_drawing::View) -> String {
    let kind = match &v.kind {
        ViewKind::Base => "Base",
        ViewKind::Projected { .. } => "Projected",
        ViewKind::Section { .. } => "Section",
        ViewKind::Detail { .. } => "Detail",
    };
    format!("{kind} - {}", orientation_name(v.orientation))
}

impl DrawingWorkspace {
    pub(super) fn browser_ui(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.add_space(4.0);
        ui.label(egui::RichText::new("DRAWINGS").size(11.5).strong());
        ui.separator();
        if project.drawings.is_empty() {
            ui.label(
                egui::RichText::new("No drawings yet. Use New Drawing in the ribbon.")
                    .color(TEXT_WEAK)
                    .size(11.5),
            );
            return;
        }
        let mut delete_sheet = None;
        for (i, s) in project.drawings.sheets.iter().enumerate() {
            let open =
                egui::CollapsingHeader::new(format!("{} ({})", s.name, s.format.size.name()))
                    .id_salt(("dw_sheet", i))
                    .default_open(true)
                    .show(ui, |ui| {
                        for v in &s.views {
                            let sel = self.selected == Some(v.id);
                            let pending = if v.cache.is_none() { "  ..." } else { "" };
                            if ui
                                .selectable_label(sel, format!("{}{pending}", view_name(v)))
                                .clicked()
                            {
                                self.selected = Some(v.id);
                                self.sheet = i;
                            }
                        }
                    });
            open.header_response.context_menu(|ui| {
                if ui.button("Delete sheet").clicked() {
                    delete_sheet = Some(i);
                    ui.close();
                }
            });
            if open.header_response.clicked() {
                self.sheet = i;
            }
        }
        if let Some(i) = delete_sheet {
            self.delete_sheet(project, i);
        }
    }

    pub(super) fn tabs_ui(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.horizontal(|ui| {
            let mut pick = None;
            for (i, s) in project.drawings.sheets.iter().enumerate() {
                if ui.selectable_label(self.sheet == i, &s.name).clicked() {
                    pick = Some(i);
                }
            }
            if let Some(i) = pick {
                self.sheet = i;
                self.selected = None;
            }
            if !self.status.is_empty() {
                ui.separator();
                ui.label(
                    egui::RichText::new(&self.status)
                        .size(11.5)
                        .color(if self.busy() { TEXT_WEAK } else { BAD }),
                );
            }
        });
    }

    pub(super) fn props_ui(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.add_space(4.0);
        let sheet = self.sheet;
        if project.drawings.sheets.get(sheet).is_none() {
            return;
        }
        ui.label(egui::RichText::new("TITLE BLOCK").size(11.5).strong());
        ui.separator();
        let mut changed = false;
        let mut start_edit = false;
        {
            let t = &mut project.drawings.sheets[sheet].title;
            egui::Grid::new("dw_title").num_columns(2).show(ui, |ui| {
                for (label, field) in [
                    ("Title", &mut t.title),
                    ("Number", &mut t.number),
                    ("Revision", &mut t.revision),
                    ("Drawn by", &mut t.author),
                    ("Company", &mut t.company),
                    ("Date", &mut t.date),
                ] {
                    ui.label(label);
                    let r = ui.add(egui::TextEdit::singleline(field).desired_width(140.0));
                    start_edit |= r.gained_focus();
                    changed |= r.changed();
                    ui.end_row();
                }
            });
        }
        if start_edit {
            // focus arrives before the first keystroke: remember the state before the edit
            project.dw_checkpoint();
        }
        if changed {
            project.mark_extras_dirty();
        }
        ui.add_space(8.0);
        let Some(id) = self.selected else {
            return;
        };
        let Some(v) = project.drawings.sheets[sheet].view(id).cloned() else {
            return;
        };
        ui.label(egui::RichText::new("VIEW").size(11.5).strong());
        ui.separator();
        ui.label(view_name(&v));
        let mut scale_i = STANDARD_SCALES
            .iter()
            .position(|s| (s - v.scale).abs() < 1e-9);
        let mut style = v.style;
        let mut orient = v.orientation;
        let mut label = v.label.clone();
        let mut regen = false;
        egui::Grid::new("dw_view").num_columns(2).show(ui, |ui| {
            ui.label("Scale");
            egui::ComboBox::from_id_salt("dw_scale")
                .selected_text(scale_label(v.scale))
                .show_ui(ui, |ui| {
                    for (i, s) in STANDARD_SCALES.iter().enumerate() {
                        if ui
                            .selectable_label(scale_i == Some(i), scale_label(*s))
                            .clicked()
                        {
                            scale_i = Some(i);
                        }
                    }
                });
            ui.end_row();
            ui.label("Style");
            egui::ComboBox::from_id_salt("dw_style")
                .selected_text(style_name(style))
                .show_ui(ui, |ui| {
                    for s in [
                        ViewStyle::Visible,
                        ViewStyle::VisibleHidden,
                        ViewStyle::Wireframe,
                        ViewStyle::Shaded,
                    ] {
                        ui.selectable_value(&mut style, s, style_name(s));
                    }
                });
            ui.end_row();
            if matches!(v.kind, ViewKind::Base) {
                ui.label("Direction");
                egui::ComboBox::from_id_salt("dw_orient")
                    .selected_text(orientation_name(orient))
                    .show_ui(ui, |ui| {
                        for (o, n) in ORIENTATIONS {
                            ui.selectable_value(&mut orient, o, n);
                        }
                    });
                ui.end_row();
            }
            ui.label("Label");
            ui.add(egui::TextEdit::singleline(&mut label).desired_width(110.0));
            ui.end_row();
        });
        let new_scale = scale_i.map_or(v.scale, |i| STANDARD_SCALES[i]);
        if (new_scale - v.scale).abs() > 1e-12 || style != v.style || orient != v.orientation {
            regen = true;
        }
        if regen || label != v.label {
            project.dw_checkpoint();
            if let Some(vm) = project.drawings.sheets[sheet].view_mut(id) {
                vm.scale = new_scale;
                vm.style = style;
                vm.orientation = orient;
                vm.label = label;
            }
            project.mark_extras_dirty();
            if regen {
                self.generate(project, vec![id]);
            }
        }
    }

    pub(super) fn dialogs_ui(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(dialog) = self.dialog.clone() else {
            return;
        };
        let mut keep = true;
        let mut apply = false;
        let mut dialog = dialog;
        let title = match &dialog {
            Dialog::NewDrawing(_) => "NEW DRAWING",
            Dialog::BaseView(_) => "BASE VIEW",
        };
        egui::Window::new(egui::RichText::new(title).size(11.5).strong())
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                match &mut dialog {
                    Dialog::NewDrawing(f) => {
                        egui::Grid::new("dw_new").num_columns(2).show(ui, |ui| {
                            ui.label("Of");
                            ui.horizontal(|ui| {
                                ui.selectable_value(&mut f.assembly, false, "Design");
                                ui.add_enabled_ui(project.assembly.is_some(), |ui| {
                                    ui.selectable_value(&mut f.assembly, true, "Assembly");
                                });
                            });
                            ui.end_row();
                            ui.label("Standard");
                            ui.horizontal(|ui| {
                                ui.selectable_value(&mut f.standard, Standard::Iso, "ISO");
                                ui.selectable_value(&mut f.standard, Standard::Asme, "ASME");
                            });
                            ui.end_row();
                            ui.label("Sheet size");
                            egui::ComboBox::from_id_salt("dw_size")
                                .selected_text(f.size.name())
                                .show_ui(ui, |ui| {
                                    for s in PaperSize::STANDARD {
                                        ui.selectable_value(&mut f.size, s, s.name());
                                    }
                                });
                            ui.end_row();
                            ui.label("Title");
                            ui.text_edit_singleline(&mut f.title);
                            ui.end_row();
                            ui.label("Number");
                            ui.text_edit_singleline(&mut f.number);
                            ui.end_row();
                            ui.label("Drawn by");
                            ui.text_edit_singleline(&mut f.author);
                            ui.end_row();
                        });
                    }
                    Dialog::BaseView(f) => {
                        egui::Grid::new("dw_base").num_columns(2).show(ui, |ui| {
                            ui.label("Direction");
                            egui::ComboBox::from_id_salt("dw_base_o")
                                .selected_text(orientation_name(f.orientation))
                                .show_ui(ui, |ui| {
                                    for (o, n) in ORIENTATIONS {
                                        ui.selectable_value(&mut f.orientation, o, n);
                                    }
                                });
                            ui.end_row();
                            ui.label("Style");
                            egui::ComboBox::from_id_salt("dw_base_s")
                                .selected_text(style_name(f.style))
                                .show_ui(ui, |ui| {
                                    for s in [
                                        ViewStyle::Visible,
                                        ViewStyle::VisibleHidden,
                                        ViewStyle::Wireframe,
                                        ViewStyle::Shaded,
                                    ] {
                                        ui.selectable_value(&mut f.style, s, style_name(s));
                                    }
                                });
                            ui.end_row();
                            ui.label("Scale");
                            egui::ComboBox::from_id_salt("dw_base_sc")
                                .selected_text(
                                    f.scale.map_or("Fit".to_string(), |i| {
                                        scale_label(STANDARD_SCALES[i])
                                    }),
                                )
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut f.scale, None, "Fit");
                                    for (i, s) in STANDARD_SCALES.iter().enumerate() {
                                        ui.selectable_value(&mut f.scale, Some(i), scale_label(*s));
                                    }
                                });
                            ui.end_row();
                            ui.label("Of");
                            ui.horizontal(|ui| {
                                ui.selectable_value(&mut f.assembly, false, "Design");
                                ui.add_enabled_ui(project.assembly.is_some(), |ui| {
                                    ui.selectable_value(&mut f.assembly, true, "Assembly");
                                });
                            });
                            ui.end_row();
                        });
                    }
                }
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        apply = true;
                        keep = false;
                    }
                    if ui.button("Cancel").clicked() {
                        keep = false;
                    }
                });
            });
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            keep = false;
            apply = false;
        }
        if apply {
            let res = match &dialog {
                Dialog::NewDrawing(f) => self.new_drawing(project, f),
                Dialog::BaseView(f) => {
                    let f: ViewForm = f.clone();
                    self.add_base_view(project, &f, true).map(|_| ())
                }
            };
            if let Err(e) = res {
                self.status = e;
            }
        }
        self.dialog = if keep { Some(dialog) } else { None };
    }
}
