//! The ECAD workspace: a board in 2-D, with placement, routing, design-rule checks, copper pour and the
//! fabrication outputs. The board is stored in the project (`board.cbor`).

use crate::dialogs::Dialogs;
use crate::job::Job;
use crate::project::Project;
use eframe::egui;
use kw_ecad::bench_gen::{benchmark_specs, generate};
use kw_ecad::drc::{drc, open_nets, Violation};
use kw_ecad::export::gerber::{gerber, GerberLayer};
use kw_ecad::export::{excellon, idf, pnp, step};
use kw_ecad::place::{place, PlaceParams};
use kw_ecad::route::{RouteReport, Router, RouterParams};
use kw_ecad::shape::{copper_items, ItemKind, Shape};
use kw_ecad::{Board, Side};
use std::path::Path;

/// Pan / zoom of the board canvas.
struct View {
    /// Pixels per millimetre.
    scale: f32,
    /// Board point at the canvas centre.
    centre: [f32; 2],
    fitted_for: Option<(u64, u64)>,
}

pub struct EcadWorkspace {
    view: View,
    sample: usize,
    pub message: String,
    pub violations: Vec<Violation>,
    pub opens: usize,
    layer_on: Vec<bool>,
    pour_net: usize,
    route_job: Option<Job<(Board, RouteReport)>>,
    pub requests: Vec<&'static str>,
}

impl Default for EcadWorkspace {
    fn default() -> Self {
        EcadWorkspace {
            view: View {
                scale: 8.0,
                centre: [0.0, 0.0],
                fitted_for: None,
            },
            sample: 0,
            message: String::new(),
            violations: Vec::new(),
            opens: 0,
            layer_on: Vec::new(),
            pour_net: 0,
            route_job: None,
            requests: Vec::new(),
        }
    }
}

fn layer_colour(l: usize, layers: usize) -> egui::Color32 {
    if l == 0 {
        egui::Color32::from_rgb(210, 70, 60)
    } else if l == layers - 1 {
        egui::Color32::from_rgb(70, 120, 220)
    } else if l == 1 {
        egui::Color32::from_rgb(200, 170, 60)
    } else {
        egui::Color32::from_rgb(90, 180, 110)
    }
}

impl EcadWorkspace {
    /// The ribbon of the Electronics workspace.
    pub fn ribbon(&mut self, ui: &mut egui::Ui) {
        use crate::chrome::{ribbon as rb, Icon};
        let mut cur = 0usize;
        rb::tabs(ui, &[("BOARD", true), ("SCHEMATIC", false)], &mut cur);
        ui.add_space(2.0);
        let busy = self.route_job.is_some();
        let mut req: Vec<&'static str> = Vec::new();
        ui.horizontal(|ui| {
            rb::group(ui, "BOARD", |ui| {
                if rb::button(
                    ui,
                    Icon::Board,
                    "Sample Board",
                    !busy,
                    false,
                    "Generate the selected sample board",
                )
                .clicked()
                {
                    req.push("generate");
                }
            });
            rb::group(ui, "PLACE & ROUTE", |ui| {
                if rb::button(
                    ui,
                    Icon::Pattern,
                    "Place",
                    !busy,
                    false,
                    "Place all parts again",
                )
                .clicked()
                {
                    req.push("place");
                }
                if rb::button(ui, Icon::Line, "Route All", !busy, busy, "Route every net").clicked()
                {
                    req.push("route");
                }
                if rb::button(
                    ui,
                    Icon::Finish,
                    "Check (DRC)",
                    !busy,
                    false,
                    "Design-rule check and connectivity",
                )
                .clicked()
                {
                    req.push("check");
                }
                if rb::button(
                    ui,
                    Icon::Plane,
                    "Pour",
                    !busy,
                    false,
                    "Pour the chosen net on the bottom layer",
                )
                .clicked()
                {
                    req.push("pour");
                }
            });
            rb::group(ui, "OUTPUT", |ui| {
                if rb::button(
                    ui,
                    Icon::Folder,
                    "Fabrication Files",
                    true,
                    false,
                    "Gerber, drill, IDF and pick-and-place into a folder",
                )
                .clicked()
                {
                    req.push("fab");
                }
                if rb::button(
                    ui,
                    Icon::Body,
                    "STEP",
                    true,
                    false,
                    "The board with its components as a STEP file",
                )
                .clicked()
                {
                    req.push("step");
                }
            });
        });
        self.requests.extend(req);
    }

    // ------------------------------------------------------------------------------ actions

    pub fn generate_sample(&mut self, project: &mut Project, index: usize) {
        let specs = benchmark_specs();
        let spec = &specs[index.min(specs.len() - 1)];
        let (b, rep) = generate(spec);
        self.message = format!(
            "generated {} ({} parts, {} nets), placement legal: {}, wirelength {:.0} → {:.0} mm",
            b.name,
            b.parts.len(),
            b.nets.len(),
            rep.legal,
            rep.hpwl_initial,
            rep.hpwl_final
        );
        self.set_board(project, b);
    }

    fn set_board(&mut self, project: &mut Project, b: Board) {
        self.layer_on = vec![true; b.layers];
        self.violations.clear();
        self.opens = 0;
        self.view.fitted_for = None;
        self.pour_net = b.nets.iter().position(|n| n == "GND").unwrap_or(0);
        project.board = Some(b);
        project.mark_extras_dirty();
    }

    pub fn replace(&mut self, project: &mut Project) {
        let Some(b) = project.board.as_mut() else {
            return;
        };
        b.tracks.clear();
        b.vias.clear();
        b.zones.clear();
        let rep = place(
            b,
            &PlaceParams {
                seed: 1 + b.parts.len() as u64 * 7,
                ..PlaceParams::default()
            },
        );
        self.message = format!(
            "placed: legal {}, wirelength {:.0} → {:.0} mm",
            rep.legal, rep.hpwl_initial, rep.hpwl_final
        );
        self.violations.clear();
        project.mark_extras_dirty();
    }

    pub fn start_route(&mut self, project: &Project) {
        let Some(b) = project.board.clone() else {
            return;
        };
        self.route_job = Some(Job::spawn("routing", move || {
            let mut b = b;
            b.tracks.clear();
            b.vias.clear();
            b.zones.clear();
            let mut r = Router::new(b, RouterParams::default());
            let rep = r.route_all();
            Ok((r.board, rep))
        }));
        self.message = "routing…".into();
    }

    pub fn check(&mut self, project: &Project) {
        let Some(b) = &project.board else { return };
        self.violations = drc(b);
        self.opens = open_nets(b).len();
        self.message = format!(
            "{} design-rule violations, {} unconnected nets",
            self.violations.len(),
            self.opens
        );
    }

    pub fn pour(&mut self, project: &mut Project) {
        let (net, layer) = match &project.board {
            Some(b) => (
                self.pour_net.min(b.nets.len().saturating_sub(1)),
                b.layers - 1,
            ),
            None => return,
        };
        if let Some(b) = project.board.as_mut() {
            let rep = kw_ecad::pour::pour(b, net, layer);
            self.message = format!(
                "poured {} on layer {}: {:.0} mm², {} island(s) dropped",
                b.nets[net],
                layer + 1,
                rep.area,
                rep.islands_removed
            );
        }
        project.mark_extras_dirty();
    }

    /// Write Gerber, Excellon, IDF and pick-and-place files into `dir`.
    pub fn export_fabrication(&mut self, board: &Board, dir: &Path) -> Result<usize, String> {
        let mut n = 0;
        let mut put = |name: String, text: String| -> Result<(), String> {
            std::fs::write(dir.join(name), text).map_err(|e| e.to_string())?;
            n += 1;
            Ok(())
        };
        let mut layers: Vec<GerberLayer> = (0..board.layers).map(GerberLayer::Copper).collect();
        layers.extend([
            GerberLayer::Mask(Side::Top),
            GerberLayer::Mask(Side::Bottom),
            GerberLayer::Profile,
        ]);
        for l in layers {
            put(l.file_name(board), gerber(board, l))?;
        }
        put(format!("{}-PTH.drl", board.name), excellon::pth(board))?;
        put(format!("{}-NPTH.drl", board.name), excellon::npth(board))?;
        put(format!("{}.emn", board.name), idf::emn(board))?;
        put(format!("{}.emp", board.name), idf::emp(board))?;
        put(
            format!("{}-pos.csv", board.name),
            pnp::pick_and_place(board),
        )?;
        Ok(n)
    }

    // ----------------------------------------------------------------------------------- UI

    pub fn show(&mut self, ui: &mut egui::Ui, project: &mut Project, dialogs: &mut dyn Dialogs) {
        // a finished routing job
        if let Some(job) = &self.route_job {
            match job.poll() {
                Some(Ok((b, rep))) => {
                    self.message = format!(
                        "routed {}/{} nets ({:.1} %), {} tracks, {} vias in {:.1} s",
                        rep.nets_routed,
                        rep.nets_total,
                        100.0 * rep.completion(),
                        rep.tracks,
                        rep.vias,
                        job.elapsed()
                    );
                    let layers_on = self.layer_on.clone();
                    project.board = Some(b);
                    self.layer_on = layers_on;
                    project.mark_extras_dirty();
                    self.route_job = None;
                    self.violations.clear();
                }
                Some(Err(e)) => {
                    self.message = format!("routing failed: {e}");
                    self.route_job = None;
                }
                None => ui.ctx().request_repaint(),
            }
        }
        for r in std::mem::take(&mut self.requests) {
            match r {
                "generate" => {
                    let i = self.sample;
                    self.generate_sample(project, i);
                }
                "place" => self.replace(project),
                "route" => self.start_route(project),
                "check" => self.check(project),
                "pour" => self.pour(project),
                "fab" => {
                    if let Some(b) = project.board.clone() {
                        if let Some(dir) = dialogs.choose_folder() {
                            self.message = match self.export_fabrication(&b, &dir) {
                                Ok(n) => format!("wrote {n} files to {}", dir.display()),
                                Err(e) => format!("export failed: {e}"),
                            };
                        }
                    }
                }
                "step" => {
                    if let Some(b) = project.board.clone() {
                        if let Some(path) =
                            dialogs.save_file(&format!("{}.step", b.name), "STEP", &["step", "stp"])
                        {
                            self.message = match std::fs::write(&path, step::step(&b)) {
                                Ok(()) => format!("wrote {}", path.display()),
                                Err(e) => format!("export failed: {e}"),
                            };
                        }
                    }
                }
                _ => {}
            }
        }
        egui::Panel::right("ecad_side")
            .default_size(300.0)
            .show(ui, |ui| self.side(ui, project, dialogs));
        egui::CentralPanel::default().show(ui, |ui| self.canvas(ui, project));
    }

    fn side(&mut self, ui: &mut egui::Ui, project: &mut Project, dialogs: &mut dyn Dialogs) {
        ui.heading("Board");
        let busy = self.route_job.is_some();
        ui.horizontal(|ui| {
            let specs = benchmark_specs();
            egui::ComboBox::from_id_salt("ecad_sample")
                .selected_text(format!("sample {}", self.sample + 1))
                .show_ui(ui, |ui| {
                    for (i, s) in specs.iter().enumerate() {
                        ui.selectable_value(
                            &mut self.sample,
                            i,
                            format!(
                                "{}: {}p {}L {}×{}",
                                i + 1,
                                s.passives + s.soic + s.qfp + s.headers,
                                s.layers,
                                s.width,
                                s.height
                            ),
                        );
                    }
                });
            if ui
                .add_enabled(!busy, egui::Button::new("Generate"))
                .on_hover_text("A seeded sample board with a netlist, placed")
                .clicked()
            {
                let i = self.sample;
                self.generate_sample(project, i);
            }
        });
        let Some(b) = &project.board else {
            ui.weak("No board in this project. Generate a sample board to begin.");
            if !self.message.is_empty() {
                ui.label(&self.message);
            }
            return;
        };
        ui.label(format!(
            "{} — {}×{} mm, {} layers",
            b.name, b.width, b.height, b.layers
        ));
        ui.label(format!(
            "{} parts, {} nets, {} tracks, {} vias, {} pours",
            b.parts.len(),
            b.nets.len(),
            b.tracks.len(),
            b.vias.len(),
            b.zones.len()
        ));
        let nets = b.nets.clone();
        let layers = b.layers;
        ui.separator();
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!busy, egui::Button::new("Place"))
                .on_hover_text("Re-place all parts (clears routing)")
                .clicked()
            {
                self.replace(project);
            }
            if ui
                .add_enabled(!busy, egui::Button::new("Route all"))
                .clicked()
            {
                self.start_route(project);
            }
            if ui
                .add_enabled(!busy, egui::Button::new("Check (DRC)"))
                .clicked()
            {
                self.check(project);
            }
        });
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("pour_net")
                .selected_text(nets.get(self.pour_net).cloned().unwrap_or_default())
                .show_ui(ui, |ui| {
                    for (i, n) in nets.iter().enumerate() {
                        ui.selectable_value(&mut self.pour_net, i, n);
                    }
                });
            if ui
                .add_enabled(!busy, egui::Button::new("Pour bottom layer"))
                .clicked()
            {
                self.pour(project);
            }
        });
        if busy {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("routing…");
            });
        }
        ui.separator();
        ui.strong("Layers");
        if self.layer_on.len() != layers {
            self.layer_on = vec![true; layers];
        }
        ui.horizontal_wrapped(|ui| {
            for l in 0..layers {
                let name = if l == 0 {
                    "Top".to_string()
                } else if l == layers - 1 {
                    "Bottom".to_string()
                } else {
                    format!("Inner {l}")
                };
                ui.checkbox(
                    &mut self.layer_on[l],
                    egui::RichText::new(name).color(layer_colour(l, layers)),
                );
            }
        });
        ui.separator();
        ui.strong("Export");
        let board = project.board.clone().expect("board");
        ui.horizontal_wrapped(|ui| {
            if ui
                .button("Gerber / drill / IDF / CSV…")
                .on_hover_text("Choose a folder")
                .clicked()
            {
                if let Some(dir) = dialogs.choose_folder() {
                    self.message = match self.export_fabrication(&board, &dir) {
                        Ok(n) => format!("wrote {n} files to {}", dir.display()),
                        Err(e) => format!("export failed: {e}"),
                    };
                }
            }
            if ui.button("STEP…").clicked() {
                if let Some(path) =
                    dialogs.save_file(&format!("{}.step", board.name), "STEP", &["step", "stp"])
                {
                    self.message = match std::fs::write(&path, step::step(&board)) {
                        Ok(()) => format!("wrote {}", path.display()),
                        Err(e) => format!("export failed: {e}"),
                    };
                }
            }
        });
        if !self.message.is_empty() {
            ui.separator();
            ui.label(&self.message);
        }
        if !self.violations.is_empty() {
            ui.separator();
            ui.strong(format!("Violations ({})", self.violations.len()));
            egui::ScrollArea::vertical()
                .max_height(200.0)
                .show(ui, |ui| {
                    for v in self.violations.iter().take(200) {
                        ui.label(format!(
                            "{:?}: {} / {} at ({:.1}, {:.1})",
                            v.kind, v.a, v.b, v.at[0], v.at[1]
                        ));
                    }
                });
        }
    }

    fn canvas(&mut self, ui: &mut egui::Ui, project: &Project) {
        let (rect, resp) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(18, 40, 28));
        let Some(b) = &project.board else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No board",
                egui::FontId::proportional(18.0),
                egui::Color32::GRAY,
            );
            return;
        };
        let key = (b.width.to_bits(), b.height.to_bits());
        if self.view.fitted_for != Some(key) {
            self.view.scale =
                ((rect.width() / b.width as f32).min(rect.height() / b.height as f32) * 0.9)
                    .max(0.5);
            self.view.centre = [b.width as f32 / 2.0, b.height as f32 / 2.0];
            self.view.fitted_for = Some(key);
        }
        if resp.dragged() {
            let d = resp.drag_delta();
            self.view.centre[0] -= d.x / self.view.scale;
            self.view.centre[1] += d.y / self.view.scale;
        }
        if resp.hovered() {
            let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.view.scale = (self.view.scale * (1.0 + scroll * 0.002)).clamp(0.5, 400.0);
            }
        }
        let v = &self.view;
        let to = |p: [f64; 2]| {
            egui::pos2(
                rect.center().x + (p[0] as f32 - v.centre[0]) * v.scale,
                rect.center().y - (p[1] as f32 - v.centre[1]) * v.scale,
            )
        };
        // board
        painter.rect_filled(
            egui::Rect::from_two_pos(to([0.0, 0.0]), to([b.width, b.height])),
            0.0,
            egui::Color32::from_rgb(28, 70, 46),
        );
        painter.rect_stroke(
            egui::Rect::from_two_pos(to([0.0, 0.0]), to([b.width, b.height])),
            0.0,
            egui::Stroke::new(1.5, egui::Color32::from_rgb(230, 220, 90)),
            egui::StrokeKind::Middle,
        );
        // copper, bottom layers first so the top shows over them
        let items = copper_items(b);
        let on = |l: usize| self.layer_on.get(l).copied().unwrap_or(true);
        for l in (0..b.layers).rev() {
            if !on(l) {
                continue;
            }
            let col = layer_colour(l, b.layers).gamma_multiply(0.75);
            for it in items.iter().filter(|i| i.layer == Some(l)) {
                match &it.shape {
                    Shape::Cap { a, b: e, r } => {
                        let w = (2.0 * *r as f32 * v.scale).max(1.0);
                        if a == e {
                            painter.circle_filled(to(*a), w / 2.0, col);
                        } else {
                            painter.line_segment([to(*a), to(*e)], egui::Stroke::new(w, col));
                            painter.circle_filled(to(*a), w / 2.0, col);
                            painter.circle_filled(to(*e), w / 2.0, col);
                        }
                    }
                    Shape::Poly(p) => {
                        painter.add(egui::Shape::convex_polygon(
                            p.iter().map(|q| to(*q)).collect(),
                            col,
                            egui::Stroke::NONE,
                        ));
                    }
                    Shape::Region(contours) => {
                        for c in contours {
                            painter.add(egui::Shape::closed_line(
                                c.iter().map(|q| to(*q)).collect(),
                                egui::Stroke::new(1.0, col),
                            ));
                        }
                    }
                }
            }
        }
        // through items (vias, through-hole pads): a copper ring and the hole
        for it in items
            .iter()
            .filter(|i| i.layer.is_none() && i.kind != ItemKind::Zone)
        {
            if let Shape::Cap { a, r, .. } = &it.shape {
                painter.circle_filled(
                    to(*a),
                    (*r as f32 * v.scale).max(1.5),
                    egui::Color32::from_rgb(220, 200, 150),
                );
                painter.circle_filled(
                    to(*a),
                    (*r as f32 * v.scale * 0.45).max(0.8),
                    egui::Color32::from_rgb(18, 40, 28),
                );
            } else if let Shape::Poly(p) = &it.shape {
                painter.add(egui::Shape::convex_polygon(
                    p.iter().map(|q| to(*q)).collect(),
                    egui::Color32::from_rgb(220, 200, 150),
                    egui::Stroke::NONE,
                ));
            }
        }
        // reference designators
        if v.scale > 6.0 {
            for p in &b.parts {
                painter.text(
                    to(p.pos),
                    egui::Align2::CENTER_CENTER,
                    &p.refdes,
                    egui::FontId::monospace(10.0),
                    egui::Color32::from_rgb(240, 240, 240),
                );
            }
        }
        for viol in &self.violations {
            painter.circle_stroke(
                to(viol.at),
                8.0,
                egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 60, 200)),
            );
        }
    }
}
