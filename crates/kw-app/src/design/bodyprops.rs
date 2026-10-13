//! The body properties window: name, colour and material of a body, stored in the document.

use super::measure::materials;
use super::DesignWorkspace;
use crate::bodies::{summaries, DEFAULT_COLOUR};
use crate::project::Project;
use eframe::egui;
use kw_feature::Appearance;

impl DesignWorkspace {
    pub fn body_props_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(root) = self.body_props else {
            return;
        };
        let doc = project.document();
        let Some(info) = summaries(&doc).into_iter().find(|b| b.root == root) else {
            self.body_props = None;
            return;
        };
        // the edited values live in the window until Apply
        let key = egui::Id::new(("body_props", root.0));
        let (mut name, mut colour, mut material): (String, [u8; 3], Option<String>) = ctx
            .data_mut(|d| d.get_temp(key))
            .unwrap_or((info.name.clone(), info.colour, info.material.clone()));
        let mut open = true;
        let mut apply = false;
        let mut reset = false;
        egui::Window::new(egui::RichText::new("BODY PROPERTIES").size(11.5).strong())
            .open(&mut open)
            .collapsible(false)
            .default_pos(egui::pos2(330.0, 190.0))
            .show(ctx, |ui| {
                egui::Grid::new("body_props_grid")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("Name");
                        ui.text_edit_singleline(&mut name);
                        ui.end_row();
                        ui.label("Colour");
                        ui.color_edit_button_srgb(&mut colour);
                        ui.end_row();
                        ui.label("Material");
                        egui::ComboBox::from_id_salt("body_material")
                            .selected_text(material.clone().unwrap_or_else(|| "(none)".into()))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut material, None, "(none)");
                                for m in materials() {
                                    ui.selectable_value(
                                        &mut material,
                                        Some(m.name.clone()),
                                        format!("{} ({} kg/m³)", m.name, m.density),
                                    );
                                }
                            });
                        ui.end_row();
                    });
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui.button("Apply").clicked() {
                        apply = true;
                    }
                    if ui.button("Reset").clicked() {
                        reset = true;
                    }
                });
            });
        if apply || reset {
            let ap = if reset {
                Appearance::default()
            } else {
                Appearance {
                    name: (!name.trim().is_empty()).then(|| name.trim().to_string()),
                    colour: (colour != DEFAULT_COLOUR).then_some(colour),
                    material: material.clone(),
                }
            };
            self.model.edit(project, |d| {
                d.set_appearance(root, ap);
                Ok(())
            });
            ctx.data_mut(|d| d.remove_temp::<(String, [u8; 3], Option<String>)>(key));
        } else {
            ctx.data_mut(|d| d.insert_temp(key, (name, colour, material)));
        }
        if !open {
            self.body_props = None;
            ctx.data_mut(|d| d.remove_temp::<(String, [u8; 3], Option<String>)>(key));
        }
    }
}
