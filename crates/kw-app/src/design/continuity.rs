//! Edge continuity in the Design workspace: every edge shared by two faces is drawn in the colour of
//! how smoothly the faces meet (gap, crease, tangent, curvature continuous), with a window that
//! counts them and lists the worst.

use super::DesignWorkspace;
use crate::chrome::theme::TEXT_WEAK;
use eframe::egui;
use kw_analysis::{body_continuity, ContClass, EdgeContinuity, Tolerances};

#[derive(Debug, Clone, PartialEq)]
pub struct ContinuityState {
    /// Faces whose normals differ by less than this (degrees) are tangent.
    pub angle_deg: f32,
}

impl Default for ContinuityState {
    fn default() -> Self {
        ContinuityState { angle_deg: 0.5 }
    }
}

/// The colour an edge of this class is drawn in.
pub fn class_colour(c: ContClass) -> egui::Color32 {
    match c {
        ContClass::Broken => egui::Color32::from_rgb(220, 50, 50),
        ContClass::G0 => egui::Color32::from_rgb(235, 190, 40),
        ContClass::G1 => egui::Color32::from_rgb(50, 120, 235),
        ContClass::G2 => egui::Color32::from_rgb(40, 180, 90),
        ContClass::Open => egui::Color32::from_rgb(230, 60, 200),
    }
}

/// The classes shown in the window, best last.
pub const CLASSES: [ContClass; 5] = [
    ContClass::Broken,
    ContClass::G0,
    ContClass::G1,
    ContClass::G2,
    ContClass::Open,
];

/// The edges of every body with their continuity, kept while the model and the tolerance stay put.
#[derive(Debug, Clone)]
pub struct ContinuityCache {
    pub key: u128,
    pub angle_bits: u32,
    pub edges: Vec<EdgeContinuity>,
}

impl DesignWorkspace {
    /// Switch the edge continuity display on or off.
    pub fn toggle_continuity(&mut self) {
        self.continuity = match self.continuity {
            Some(_) => None,
            None => Some(ContinuityState::default()),
        };
    }

    /// The continuity of the edges of the bodies shown, worked out again only when the model changes.
    pub fn continuity_edges(&mut self) -> &[EdgeContinuity] {
        let (Some(state), Some(bodies)) = (self.continuity.as_ref(), self.bodies.as_ref()) else {
            return &[];
        };
        let (key, bits) = (bodies.key, state.angle_deg.to_bits());
        let stale = self
            .continuity_cache
            .as_ref()
            .is_none_or(|c| c.key != key || c.angle_bits != bits);
        if stale {
            let mut edges = Vec::new();
            for b in &bodies.list {
                let (Some(rep), Some(body)) = (b.output.brep(), b.output.body()) else {
                    continue;
                };
                let size = (rep.body_bbox(body).max - rep.body_bbox(body).min).norm();
                let mut tol = Tolerances::for_size(size);
                tol.angle = (state.angle_deg as f64).to_radians();
                edges.extend(body_continuity(rep, body, &tol));
            }
            self.continuity_cache = Some(ContinuityCache {
                key,
                angle_bits: bits,
                edges,
            });
        }
        &self.continuity_cache.as_ref().expect("just made").edges
    }

    /// Draw the edges in their colours over the viewport.
    pub fn draw_continuity(&mut self, painter: &egui::Painter) {
        if self.continuity.is_none() {
            return;
        }
        let centre = self
            .viewport
            .model()
            .map(|m| m.scene.center)
            .unwrap_or_else(nalgebra::Vector3::zeros);
        let edges: Vec<(ContClass, Vec<[f64; 3]>)> = self
            .continuity_edges()
            .iter()
            .filter(|e| e.class != ContClass::Open)
            .map(|e| {
                (
                    e.class,
                    e.points
                        .iter()
                        .map(|p| [p.x - centre.x, p.y - centre.y, p.z - centre.z])
                        .collect(),
                )
            })
            .collect();
        for (class, pts) in edges {
            let screen: Vec<Option<egui::Pos2>> =
                pts.iter().map(|p| self.viewport.project(*p)).collect();
            let stroke = egui::Stroke::new(3.0, class_colour(class));
            for w in screen.windows(2) {
                if let [Some(a), Some(b)] = w {
                    painter.line_segment([*a, *b], stroke);
                }
            }
        }
    }

    /// The window that counts the classes and lists the worst edges.
    pub fn continuity_window(&mut self, ctx: &egui::Context) {
        let Some(mut state) = self.continuity.clone() else {
            return;
        };
        let counts: Vec<(ContClass, usize)> = {
            let all = self.continuity_edges();
            CLASSES
                .iter()
                .map(|c| (*c, all.iter().filter(|e| e.class == *c).count()))
                .collect()
        };
        let worst: Vec<(ContClass, f64, f64, Option<f64>)> = {
            let mut v: Vec<&EdgeContinuity> = self
                .continuity_edges()
                .iter()
                .filter(|e| e.class != ContClass::Open && e.class != ContClass::G2)
                .collect();
            v.sort_by(|a, b| {
                b.gap
                    .max(0.0)
                    .partial_cmp(&a.gap)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(
                        b.angle
                            .partial_cmp(&a.angle)
                            .unwrap_or(std::cmp::Ordering::Equal),
                    )
            });
            v.iter()
                .take(6)
                .map(|e| (e.class, e.gap, e.angle.to_degrees(), e.curvature))
                .collect()
        };
        let mut open = true;
        egui::Window::new(egui::RichText::new("EDGE CONTINUITY").size(11.5).strong())
            .open(&mut open)
            .anchor(egui::Align2::LEFT_BOTTOM, [270.0, -90.0])
            .default_width(280.0)
            .show(ctx, |ui| {
                egui::Grid::new("continuity_grid")
                    .num_columns(3)
                    .spacing([10.0, 3.0])
                    .show(ui, |ui| {
                        for (c, n) in &counts {
                            let (r, _) =
                                ui.allocate_exact_size(egui::vec2(18.0, 8.0), egui::Sense::hover());
                            ui.painter().rect_filled(r, 2.0, class_colour(*c));
                            ui.label(c.name());
                            ui.label(n.to_string());
                            ui.end_row();
                        }
                    });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label("Tangent within");
                    ui.add(egui::Slider::new(&mut state.angle_deg, 0.1..=5.0).suffix(" deg"));
                });
                if !worst.is_empty() {
                    ui.separator();
                    ui.label(egui::RichText::new("Worst joins").color(TEXT_WEAK).size(10.5));
                    for (c, gap, angle, curv) in &worst {
                        let mut text = format!("{}: ", c.name());
                        if *c == ContClass::Broken {
                            text += &format!("gap {gap:.4} mm, ");
                        }
                        text += &format!("{angle:.2}°");
                        if let Some(k) = curv {
                            text += &format!(", curvature differs by {k:.4}/mm");
                        }
                        ui.label(egui::RichText::new(text).size(10.5));
                    }
                }
                ui.label(
                    egui::RichText::new(
                        "Joins are judged along the middle of each edge. Stitch surfaces first: only an edge shared by two faces has a continuity.",
                    )
                    .size(10.5)
                    .color(TEXT_WEAK),
                );
            });
        self.continuity = if open { Some(state) } else { None };
    }
}
