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

/// How an annotation is listed, and whether it needs attention.
fn annot_name(a: &kw_drawing::Annotation, units: kw_drawing::Units) -> (String, bool) {
    use kw_drawing::AnnotKind as K;
    match &a.kind {
        K::Dimension(d) => {
            let t = kw_drawing::dim::dim_text(d, units);
            let shown = if t.main.is_empty() {
                t.upper.unwrap_or_default()
            } else {
                t.main
            };
            let bad = matches!(d.status, kw_drawing::DimStatus::Broken(_));
            (
                format!("Dimension {shown}{}", if bad { "  (broken)" } else { "" }),
                bad,
            )
        }
        K::Note { text, .. } => (format!("Note {}", text.lines().next().unwrap_or("")), false),
        K::Datum { letter, .. } => (format!("Datum {letter}"), false),
        K::Surface { value, .. } => (format!("Finish {value}"), false),
        K::Fcf { symbol, .. } => (format!("Frame - {}", symbol.name()), false),
        K::Bom { rows, .. } => (format!("Parts list ({} parts)", rows.len()), false),
        K::Balloon {
            item, part, status, ..
        } => {
            let bad = matches!(status, kw_drawing::DimStatus::Broken(_));
            (
                format!(
                    "Balloon {item} - {part}{}",
                    if bad { "  (broken)" } else { "" }
                ),
                bad,
            )
        }
    }
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
                        for a in &s.annots {
                            let sel = self.selected_annot == Some(a.id);
                            let (name, bad) = annot_name(a, s.format.units);
                            let text = egui::RichText::new(name).color(if bad {
                                BAD
                            } else {
                                ui.visuals().text_color()
                            });
                            if ui.selectable_label(sel, text).clicked() {
                                self.selected_annot = Some(a.id);
                                self.selected = None;
                                self.sheet = i;
                            }
                        }
                        for v in &s.views {
                            let sel = self.selected == Some(v.id);
                            let pending = if v.cache.is_none() { "  ..." } else { "" };
                            if ui
                                .selectable_label(sel, format!("{}{pending}", view_name(v)))
                                .clicked()
                            {
                                self.selected = Some(v.id);
                                self.selected_annot = None;
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
        if let Some(aid) = self.selected_annot {
            self.annot_props_ui(ui, project, aid);
            return;
        }
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
        if matches!(v.kind, ViewKind::Section { .. })
            && ui
                .button("Flip direction")
                .on_hover_text("Look at the other side of the cutting line")
                .clicked()
        {
            if let Err(e) = self.flip_section(project, id) {
                self.status = e;
            }
        }
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

impl DrawingWorkspace {
    /// The editor of the selected dimension, note or symbol.
    fn annot_props_ui(&mut self, ui: &mut egui::Ui, project: &mut Project, id: u64) {
        use kw_drawing::{AnnotKind as K, Gdt, Tolerance};
        let sheet = self.sheet;
        let Some(a) = project.drawings.sheets[sheet]
            .annots
            .iter()
            .find(|a| a.id == id)
            .cloned()
        else {
            self.selected_annot = None;
            return;
        };
        let units = project.drawings.sheets[sheet].format.units;
        let mut kind = a.kind.clone();
        let mut start_edit = false;
        let mut changed = false;
        let mut delete = false;
        let mut refresh = false;
        let text_field =
            |ui: &mut egui::Ui, v: &mut String, start: &mut bool, ch: &mut bool, w: f32| {
                let r = ui.add(egui::TextEdit::singleline(v).desired_width(w));
                *start |= r.gained_focus();
                *ch |= r.changed();
            };
        match &mut kind {
            K::Dimension(d) => {
                ui.label(egui::RichText::new("DIMENSION").size(11.5).strong());
                ui.separator();
                match &d.status {
                    kw_drawing::DimStatus::Ok => {
                        ui.label(format!(
                            "Value {}",
                            kw_drawing::dim::format_value(d.value, d.kind, units, d.precision)
                        ));
                    }
                    kw_drawing::DimStatus::Broken(why) => {
                        ui.label(
                            egui::RichText::new(format!("Lost its model: {why}"))
                                .color(BAD)
                                .size(11.5),
                        );
                        ui.label(
                            egui::RichText::new(
                                "It shows the last value. Delete it and dimension again.",
                            )
                            .color(TEXT_WEAK)
                            .size(11.0),
                        );
                    }
                }
                egui::Grid::new("dw_dim").num_columns(2).show(ui, |ui| {
                    ui.label("Text");
                    text_field(ui, &mut d.text, &mut start_edit, &mut changed, 120.0);
                    ui.end_row();
                    ui.label("Before");
                    text_field(ui, &mut d.prefix, &mut start_edit, &mut changed, 120.0);
                    ui.end_row();
                    ui.label("After");
                    text_field(ui, &mut d.suffix, &mut start_edit, &mut changed, 120.0);
                    ui.end_row();
                    ui.label("Decimals");
                    let cur = d.precision;
                    egui::ComboBox::from_id_salt("dw_prec")
                        .selected_text(cur.map_or("Auto".to_string(), |p| p.to_string()))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut d.precision, None, "Auto");
                            for p in 0..=4u8 {
                                ui.selectable_value(&mut d.precision, Some(p), p.to_string());
                            }
                        });
                    changed |= cur != d.precision;
                    ui.end_row();
                    ui.label("Tolerance");
                    let name = |t: &Tolerance| match t {
                        Tolerance::None => "None",
                        Tolerance::Symmetric(_) => "Symmetric ±",
                        Tolerance::Bilateral { .. } => "Bilateral +/-",
                        Tolerance::Limits { .. } => "Limits",
                    };
                    let before = d.tolerance;
                    egui::ComboBox::from_id_salt("dw_tol")
                        .selected_text(name(&d.tolerance))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut d.tolerance, Tolerance::None, "None");
                            ui.selectable_value(
                                &mut d.tolerance,
                                Tolerance::Symmetric(0.1),
                                "Symmetric ±",
                            );
                            ui.selectable_value(
                                &mut d.tolerance,
                                Tolerance::Bilateral {
                                    upper: 0.1,
                                    lower: 0.1,
                                },
                                "Bilateral +/-",
                            );
                            ui.selectable_value(
                                &mut d.tolerance,
                                Tolerance::Limits {
                                    upper: 0.1,
                                    lower: 0.1,
                                },
                                "Limits",
                            );
                        });
                    changed |=
                        std::mem::discriminant(&before) != std::mem::discriminant(&d.tolerance);
                    ui.end_row();
                });
                let drag = |ui: &mut egui::Ui, v: &mut f64, label: &str, ch: &mut bool| {
                    ui.horizontal(|ui| {
                        ui.label(label);
                        *ch |= ui
                            .add(egui::DragValue::new(v).speed(0.01).range(0.0..=1000.0))
                            .changed();
                    });
                };
                match &mut d.tolerance {
                    Tolerance::None => {}
                    Tolerance::Symmetric(t) => drag(ui, t, "±", &mut changed),
                    Tolerance::Bilateral { upper, lower } | Tolerance::Limits { upper, lower } => {
                        drag(ui, upper, "+", &mut changed);
                        drag(ui, lower, "-", &mut changed);
                    }
                }
            }
            K::Note { text, height, .. } => {
                ui.label(egui::RichText::new("NOTE").size(11.5).strong());
                ui.separator();
                let r = ui.add(
                    egui::TextEdit::multiline(text)
                        .desired_width(200.0)
                        .desired_rows(3),
                );
                start_edit |= r.gained_focus();
                changed |= r.changed();
                ui.horizontal(|ui| {
                    ui.label("Height");
                    changed |= ui
                        .add(egui::DragValue::new(height).speed(0.1).range(1.5..=20.0))
                        .changed();
                });
            }
            K::Datum { letter, .. } => {
                ui.label(egui::RichText::new("DATUM").size(11.5).strong());
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Letter");
                    text_field(ui, letter, &mut start_edit, &mut changed, 40.0);
                });
            }
            K::Surface { value, .. } => {
                ui.label(egui::RichText::new("SURFACE FINISH").size(11.5).strong());
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Roughness");
                    text_field(ui, value, &mut start_edit, &mut changed, 100.0);
                });
            }
            K::Bom { columns, rows, .. } => {
                ui.label(egui::RichText::new("PARTS LIST").size(11.5).strong());
                ui.separator();
                ui.label(format!(
                    "{} parts, {} components",
                    rows.len(),
                    rows.iter().map(|r| r.count).sum::<u32>()
                ));
                for c in kw_drawing::BomColumn::ALL {
                    let mut on = columns.contains(&c);
                    if ui.checkbox(&mut on, c.name()).changed() {
                        if on {
                            // keep the columns in their usual order
                            columns.push(c);
                            columns.sort_by_key(|x| {
                                kw_drawing::BomColumn::ALL.iter().position(|y| y == x)
                            });
                        } else {
                            columns.retain(|x| *x != c);
                        }
                        changed = true;
                    }
                }
                if ui.button("Read from the model again").clicked() {
                    refresh = true;
                }
            }
            K::Balloon {
                item, part, status, ..
            } => {
                ui.label(egui::RichText::new("BALLOON").size(11.5).strong());
                ui.separator();
                match status {
                    kw_drawing::DimStatus::Ok => {
                        ui.label(format!("Item {item}: {part}"));
                    }
                    kw_drawing::DimStatus::Broken(why) => {
                        ui.label(egui::RichText::new(why.clone()).color(BAD).size(11.5));
                    }
                }
            }
            K::Fcf {
                symbol,
                tol,
                datums,
                ..
            } => {
                ui.label(
                    egui::RichText::new("FEATURE CONTROL FRAME")
                        .size(11.5)
                        .strong(),
                );
                ui.separator();
                egui::Grid::new("dw_fcf").num_columns(2).show(ui, |ui| {
                    ui.label("Characteristic");
                    let cur = *symbol;
                    egui::ComboBox::from_id_salt("dw_gdt")
                        .selected_text(symbol.name())
                        .show_ui(ui, |ui| {
                            for g in Gdt::ALL {
                                ui.selectable_value(symbol, g, g.name());
                            }
                        });
                    changed |= cur != *symbol;
                    ui.end_row();
                    ui.label("Tolerance");
                    text_field(ui, tol, &mut start_edit, &mut changed, 100.0);
                    ui.end_row();
                    ui.label("Datums");
                    let mut joined = datums.join(",");
                    let r = ui.add(egui::TextEdit::singleline(&mut joined).desired_width(100.0));
                    start_edit |= r.gained_focus();
                    if r.changed() {
                        *datums = joined
                            .split(',')
                            .map(|t| t.trim().to_string())
                            .filter(|t| !t.is_empty())
                            .collect();
                        changed = true;
                    }
                    ui.end_row();
                });
            }
        }
        ui.add_space(6.0);
        if ui.button("Delete").clicked() {
            delete = true;
        }
        if delete {
            let _ = self.delete_annot(project, id);
            return;
        }
        if refresh {
            self.refresh_bom(project);
            return;
        }
        if start_edit {
            project.dw_checkpoint();
        }
        if changed {
            if !start_edit && !matches!(kind, K::Note { .. } | K::Datum { .. } | K::Surface { .. })
            {
                // a click in a combo or drag value is a step of its own
                project.dw_checkpoint();
            }
            if let Some(m) = project.drawings.sheets[sheet]
                .annots
                .iter_mut()
                .find(|x| x.id == id)
            {
                m.kind = kind;
            }
            project.mark_extras_dirty();
        }
    }
}
