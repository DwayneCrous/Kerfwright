//! Inspecting an assembly: the bill of materials (and its CSV), and interference with the overlaps drawn.

use super::assembly::AsmAction;
use super::{Action, DesignWorkspace};
use crate::chrome::theme::{BAD, TEXT_WEAK};
use crate::project::Project;
use eframe::egui::{self, Color32, Stroke};
use kw_assembly::{BomRow, Pose};
use kw_brep::{BRep, BodyId};
use kw_nurbs::Curve3;

/// Density (kg/m³) of a material by name; 0 when the part has none (its mass is then not known).
pub fn density_of(material: Option<&str>) -> f64 {
    kw_materials::global().density_of(material)
}

/// The bill of materials of a posed assembly.
pub fn bom_rows(pose: &Pose) -> Vec<BomRow> {
    pose.bom(&density_of)
}

fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// The bill of materials as CSV (one line per part, then the totals).
pub fn bom_csv(rows: &[BomRow]) -> String {
    let mut out = String::from(
        "Item,Part,Quantity,Material,Volume (mm3),Mass each (g),Mass total (g),Size X (mm),Size Y (mm),Size Z (mm)\n",
    );
    let (mut qty, mut mass) = (0usize, 0.0);
    for (i, r) in rows.iter().enumerate() {
        qty += r.count;
        mass += r.mass_g * r.count as f64;
        out.push_str(&format!(
            "{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}\n",
            i + 1,
            csv_field(&r.part),
            r.count,
            csv_field(r.material.as_deref().unwrap_or("")),
            r.volume,
            r.mass_g,
            r.mass_g * r.count as f64,
            r.size[0],
            r.size[1],
            r.size[2],
        ));
    }
    out.push_str(&format!(",Total,{qty},,,,{mass:.3},,,\n"));
    out
}

/// The edges of a body as polylines (world coordinates), for drawing.
pub fn body_edge_polylines(rep: &BRep, body: BodyId) -> Vec<Vec<[f64; 3]>> {
    let mut seen = Vec::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        let Some(face) = rep.faces.get(f) else {
            continue;
        };
        for lp in &face.loops {
            for c in rep.loop_coedges(*lp) {
                let Some(co) = rep.coedges.get(c) else {
                    continue;
                };
                if seen.contains(&co.edge) {
                    continue;
                }
                seen.push(co.edge);
                let Some(edge) = rep.edges.get(co.edge) else {
                    continue;
                };
                let Some(ci) = edge.curve else {
                    continue;
                };
                let (t0, t1) = edge.param_range;
                let n = 24;
                out.push(
                    (0..=n)
                        .map(|k| {
                            let p = rep.curves[ci].point_at(t0 + (t1 - t0) * k as f64 / n as f64);
                            [p.x, p.y, p.z]
                        })
                        .collect(),
                );
            }
        }
    }
    out
}

impl DesignWorkspace {
    /// Check the posed assembly for overlaps and keep the outlines of the overlaps for drawing.
    pub fn asm_run_interference(&mut self, project: &Project) {
        self.asm_check_interference(project);
        let mut edges = Vec::new();
        if let Some(Ok(hits)) = &self.asm.overlaps {
            if let Some(Ok(p)) = self.asm.pose_ref() {
                for h in hits {
                    if let Some(b) = h.body {
                        edges.extend(body_edge_polylines(&p.rep, b));
                    }
                }
            }
        }
        self.asm.overlap_edges = edges;
        self.asm.interference_open = true;
        self.asm.view = true;
    }

    /// The window listing overlaps (click a line to select the two components).
    pub fn asm_interference_window(&mut self, ctx: &egui::Context, project: &Project) {
        if !self.asm.interference_open {
            return;
        }
        let Some(asm) = project.assembly.as_ref() else {
            self.asm.interference_open = false;
            return;
        };
        let mut open = true;
        let mut recheck = false;
        let mut select: Option<(String, String)> = None;
        egui::Window::new(egui::RichText::new("INTERFERENCE").size(11.5).strong())
            .open(&mut open)
            .anchor(egui::Align2::LEFT_BOTTOM, [270.0, -90.0])
            .default_width(260.0)
            .show(ctx, |ui| {
                match &self.asm.overlaps {
                    None => {
                        ui.label("Not checked yet.");
                    }
                    Some(Err(e)) => {
                        ui.label(egui::RichText::new(e).color(BAD).size(11.0));
                    }
                    Some(Ok(v)) if v.is_empty() => {
                        ui.label("No components overlap.");
                    }
                    Some(Ok(v)) => {
                        ui.label(
                            egui::RichText::new(format!(
                                "{} overlap{}",
                                v.len(),
                                if v.len() == 1 { "" } else { "s" }
                            ))
                            .strong()
                            .color(BAD),
                        );
                        for h in v {
                            if ui
                                .selectable_label(
                                    false,
                                    format!("{} × {}: {:.3} mm³", h.a, h.b, h.volume),
                                )
                                .clicked()
                            {
                                select = Some((h.a.clone(), h.b.clone()));
                            }
                        }
                    }
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Check again").clicked() {
                        recheck = true;
                    }
                    ui.checkbox(&mut self.asm.live_check, "Check after every change")
                        .on_hover_text("Re-checks whenever a joint or component moves");
                });
            });
        if let Some((a, b)) = select {
            // names of leaves are `Outer/Inner`: the top-level instance is the first part of the name
            let top = |n: &str| {
                let first = n.split('/').next().unwrap_or(n);
                asm.instances.iter().position(|i| i.name == first)
            };
            self.asm.selected = [top(&a), top(&b)].into_iter().flatten().collect();
            self.asm.selected.dedup();
        }
        if recheck {
            self.actions.push(Action::Asm(AsmAction::CheckInterference));
        }
        if !open {
            self.asm.interference_open = false;
            self.asm.overlap_edges.clear();
        }
    }

    /// The outlines of the overlaps, in red over the view.
    pub(super) fn draw_overlaps(&self, painter: &egui::Painter, scene_origin: [f64; 3]) {
        let stroke = Stroke::new(2.2, Color32::from_rgb(235, 40, 40));
        for poly in &self.asm.overlap_edges {
            let pts: Vec<egui::Pos2> = poly
                .iter()
                .filter_map(|p| {
                    self.viewport.project([
                        p[0] + scene_origin[0],
                        p[1] + scene_origin[1],
                        p[2] + scene_origin[2],
                    ])
                })
                .collect();
            if pts.len() >= 2 {
                painter.add(egui::Shape::line(pts, stroke));
            }
        }
    }

    /// The bill of materials: one line per part with the number of components made from it.
    pub fn asm_bom_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        if !self.asm.bom_open {
            return;
        }
        self.asm_refresh_pub(project);
        let rows = match self.asm.pose_ref() {
            Some(Ok(p)) => Ok(bom_rows(p)),
            Some(Err(e)) => Err(e.clone()),
            None => Err("there is no assembly yet".to_string()),
        };
        let mut open = true;
        let mut export = false;
        let mut set_material: Option<(String, Option<String>)> = None;
        egui::Window::new(egui::RichText::new("BILL OF MATERIALS").size(11.5).strong())
            .open(&mut open)
            .default_pos(egui::pos2(300.0, 190.0))
            .default_width(560.0)
            .show(ctx, |ui| match &rows {
                Err(e) => {
                    ui.label(egui::RichText::new(e).color(BAD).size(11.0));
                }
                Ok(rows) => {
                    egui::Grid::new("bom_grid")
                        .striped(true)
                        .spacing([14.0, 4.0])
                        .show(ui, |ui| {
                            for h in [
                                "#", "Part", "Qty", "Material", "Volume mm³", "Mass g", "Size mm",
                            ] {
                                ui.label(egui::RichText::new(h).strong().size(11.0));
                            }
                            ui.end_row();
                            let (mut qty, mut mass) = (0usize, 0.0);
                            for (i, r) in rows.iter().enumerate() {
                                qty += r.count;
                                mass += r.mass_g * r.count as f64;
                                ui.label(format!("{}", i + 1));
                                ui.label(&r.part);
                                ui.label(format!("{}", r.count));
                                let mut cur = r.material.clone();
                                egui::ComboBox::from_id_salt(("bom_mat", i))
                                    .selected_text(cur.clone().unwrap_or_else(|| "—".into()))
                                    .width(110.0)
                                    .show_ui(ui, |ui| {
                                        if ui.selectable_label(cur.is_none(), "—").clicked() {
                                            cur = None;
                                            set_material = Some((r.part.clone(), None));
                                        }
                                        for m in kw_materials::global().all() {
                                            if ui
                                                .selectable_label(
                                                    cur.as_deref() == Some(m.name.as_str()),
                                                    &m.name,
                                                )
                                                .clicked()
                                            {
                                                set_material = Some((
                                                    r.part.clone(),
                                                    Some(m.name.clone()),
                                                ));
                                            }
                                        }
                                    });
                                ui.label(format!("{:.1}", r.volume));
                                ui.label(if r.mass_g > 0.0 {
                                    format!("{:.2}", r.mass_g * r.count as f64)
                                } else {
                                    "—".into()
                                });
                                ui.label(format!(
                                    "{:.1} × {:.1} × {:.1}",
                                    r.size[0], r.size[1], r.size[2]
                                ));
                                ui.end_row();
                            }
                            ui.label("");
                            ui.label(egui::RichText::new("Total").strong());
                            ui.label(egui::RichText::new(format!("{qty}")).strong());
                            ui.label("");
                            ui.label("");
                            ui.label(egui::RichText::new(format!("{mass:.2}")).strong());
                            ui.label("");
                            ui.end_row();
                        });
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new(
                            "Mass is volume × the density of the material; a part without a material has no mass.",
                        )
                        .size(10.5)
                        .color(TEXT_WEAK),
                    );
                    if ui.button("Export CSV…").clicked() {
                        export = true;
                    }
                }
            });
        if let Some((part, mat)) = set_material {
            if let Some(a) = project.assembly.as_mut() {
                set_part_material(a, &part, mat);
                project.mark_extras_dirty();
            }
        }
        if export {
            self.asm.request = Some(super::assembly::FileRequest::ExportBom);
        }
        if !open {
            self.asm.bom_open = false;
        }
    }
}

/// Set the material of the part called `name`, wherever it is in the tree of sub-assemblies.
pub fn set_part_material(asm: &mut kw_assembly::Assembly, name: &str, material: Option<String>) {
    for p in &mut asm.parts {
        if p.name == name {
            p.material = material.clone();
        }
        if let Some(sub) = p.sub.as_mut() {
            set_part_material(sub, name, material.clone());
        }
    }
}
