//! Measuring: properties of the body (volume, area, mass in a chosen material, centre of mass, inertia) and
//! of picked faces and edges (area, length, radius, distance between two picks, angle between two planes).
//! Pure functions on a B-rep, so the numbers can be checked against closed forms.

use super::geom;
use crate::viewport::Pick;
use kw_brep::{BRep, BodyId, EdgeId, FaceId};
use kw_nurbs::{Curve3, CurveGeom, SurfaceGeom};

/// A material with a density in kg/m³ (the model is in millimetres).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    pub name: &'static str,
    pub density: f64,
}

pub const MATERIALS: &[Material] = &[
    Material {
        name: "Steel",
        density: 7850.0,
    },
    Material {
        name: "Stainless steel",
        density: 8000.0,
    },
    Material {
        name: "Aluminium",
        density: 2700.0,
    },
    Material {
        name: "Brass",
        density: 8500.0,
    },
    Material {
        name: "Copper",
        density: 8960.0,
    },
    Material {
        name: "Titanium",
        density: 4500.0,
    },
    Material {
        name: "ABS plastic",
        density: 1050.0,
    },
    Material {
        name: "PLA",
        density: 1240.0,
    },
    Material {
        name: "Nylon",
        density: 1150.0,
    },
    Material {
        name: "Oak wood",
        density: 750.0,
    },
];

#[derive(Debug, Clone, PartialEq)]
pub struct BodyProps {
    /// mm³
    pub volume: f64,
    /// mm²
    pub area: f64,
    pub centroid: [f64; 3],
    pub bbox_min: [f64; 3],
    pub bbox_max: [f64; 3],
    /// Inertia tensor about the centroid for unit density (mm⁵), row-major.
    pub inertia: [[f64; 3]; 3],
}

impl BodyProps {
    pub fn size(&self) -> [f64; 3] {
        [
            self.bbox_max[0] - self.bbox_min[0],
            self.bbox_max[1] - self.bbox_min[1],
            self.bbox_max[2] - self.bbox_min[2],
        ]
    }

    /// Mass in grams for a material of density `kg_m3`.
    pub fn mass_g(&self, kg_m3: f64) -> f64 {
        // mm³ -> m³ is 1e-9; kg -> g is 1e3
        self.volume * 1e-9 * kg_m3 * 1e3
    }

    /// Inertia tensor about the centroid in g·mm² for a material.
    pub fn inertia_g_mm2(&self, kg_m3: f64) -> [[f64; 3]; 3] {
        // unit-density inertia is mm⁵; times density kg/m³ = 1e-6 g/mm³
        let k = kg_m3 * 1e-6;
        let mut m = self.inertia;
        for r in &mut m {
            for c in r {
                *c *= k;
            }
        }
        m
    }
}

pub fn body_props(brep: &BRep, body: BodyId) -> Option<BodyProps> {
    let mp = brep.mass_properties(body)?;
    let bb = brep.body_bbox(body);
    let i = mp.inertia_centroid();
    Some(BodyProps {
        volume: mp.volume,
        area: mp.area,
        centroid: [mp.centroid.x, mp.centroid.y, mp.centroid.z],
        bbox_min: [bb.min.x, bb.min.y, bb.min.z],
        bbox_max: [bb.max.x, bb.max.y, bb.max.z],
        inertia: [
            [i[(0, 0)], i[(0, 1)], i[(0, 2)]],
            [i[(1, 0)], i[(1, 1)], i[(1, 2)]],
            [i[(2, 0)], i[(2, 1)], i[(2, 2)]],
        ],
    })
}

/// What an edge is, with its length.
#[derive(Debug, Clone, PartialEq)]
pub enum EdgeKind {
    Line { length: f64, direction: [f64; 3] },
    Circle { radius: f64, length: f64 },
    Curve { length: f64 },
}

pub fn edge_kind(brep: &BRep, e: EdgeId) -> Option<EdgeKind> {
    let edge = brep.edges.get(e)?;
    let curve = &brep.curves[edge.curve?];
    let (t0, t1) = edge.param_range;
    Some(match curve {
        CurveGeom::Line(l) => EdgeKind::Line {
            length: l.dir.norm() * (t1 - t0).abs(),
            direction: {
                let d = l.dir.normalize();
                [d.x, d.y, d.z]
            },
        },
        CurveGeom::Circle(c) => EdgeKind::Circle {
            radius: c.radius,
            length: c.radius * (t1 - t0).abs(),
        },
        other => {
            let n = 400;
            let pts: Vec<_> = (0..=n)
                .map(|i| other.point_at(t0 + (t1 - t0) * i as f64 / n as f64))
                .collect();
            EdgeKind::Curve {
                length: pts.windows(2).map(|w| (w[1] - w[0]).norm()).sum(),
            }
        }
    })
}

/// A face: its area and a description of its surface.
#[derive(Debug, Clone, PartialEq)]
pub enum FaceKind {
    Plane { normal: [f64; 3] },
    Cylinder { radius: f64 },
    Cone,
    Sphere { radius: f64 },
    Torus { major: f64, minor: f64 },
    FreeForm,
}

pub fn face_kind(brep: &BRep, f: FaceId) -> Option<(FaceKind, f64)> {
    let face = brep.faces.get(f)?;
    let area = brep.face_area(f)?;
    let kind = match &brep.surfaces[face.surface] {
        SurfaceGeom::Plane(_) => FaceKind::Plane {
            normal: geom::face_frame(brep, f)?.n,
        },
        SurfaceGeom::Cylinder(c) => FaceKind::Cylinder { radius: c.radius },
        SurfaceGeom::Cone(_) => FaceKind::Cone,
        SurfaceGeom::Sphere(s) => FaceKind::Sphere { radius: s.radius },
        SurfaceGeom::Torus(t) => FaceKind::Torus {
            major: t.major_radius,
            minor: t.minor_radius,
        },
        _ => FaceKind::FreeForm,
    };
    Some((kind, area))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Sampled points of an edge (for distances between edges).
fn edge_points(brep: &BRep, e: EdgeId) -> Vec<[f64; 3]> {
    let Some(edge) = brep.edges.get(e) else {
        return Vec::new();
    };
    let Some(c) = edge.curve else {
        return Vec::new();
    };
    let curve = &brep.curves[c];
    let (t0, t1) = edge.param_range;
    (0..=200)
        .map(|i| {
            let p = curve.point_at(t0 + (t1 - t0) * i as f64 / 200.0);
            [p.x, p.y, p.z]
        })
        .collect()
}

/// Minimum distance between two polylines, by densely sampling both (points and segment interiors).
fn polyline_distance(a: &[[f64; 3]], b: &[[f64; 3]]) -> f64 {
    let seg_point = |p: [f64; 3], s0: [f64; 3], s1: [f64; 3]| -> f64 {
        let d = [s1[0] - s0[0], s1[1] - s0[1], s1[2] - s0[2]];
        let l2 = dot(d, d);
        let t = if l2 > 0.0 {
            (dot([p[0] - s0[0], p[1] - s0[1], p[2] - s0[2]], d) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        dist(p, [s0[0] + t * d[0], s0[1] + t * d[1], s0[2] + t * d[2]])
    };
    let mut best = f64::INFINITY;
    for p in a {
        for w in b.windows(2) {
            best = best.min(seg_point(*p, w[0], w[1]));
        }
    }
    for p in b {
        for w in a.windows(2) {
            best = best.min(seg_point(*p, w[0], w[1]));
        }
    }
    best
}

fn face_centre(brep: &BRep, f: FaceId) -> Option<[f64; 3]> {
    let bb = brep.face_bbox(f)?;
    Some([
        0.5 * (bb.min.x + bb.max.x),
        0.5 * (bb.min.y + bb.max.y),
        0.5 * (bb.min.z + bb.max.z),
    ])
}

/// Lines of text describing the picks: one pick alone, or two compared. `None` picks give no lines.
pub fn describe(brep: &BRep, picks: &[Pick]) -> Vec<String> {
    let mut out = Vec::new();
    let one = |p: &Pick, out: &mut Vec<String>| match p {
        Pick::Face(f) => {
            if let Some((k, area)) = face_kind(brep, *f) {
                let what = match k {
                    FaceKind::Plane { normal } => format!(
                        "planar face, normal ({:.3}, {:.3}, {:.3})",
                        normal[0], normal[1], normal[2]
                    ),
                    FaceKind::Cylinder { radius } => {
                        format!("cylindrical face, radius {radius:.4} mm")
                    }
                    FaceKind::Cone => "conical face".into(),
                    FaceKind::Sphere { radius } => format!("spherical face, radius {radius:.4} mm"),
                    FaceKind::Torus { major, minor } => {
                        format!("toroidal face, radii {major:.4} / {minor:.4} mm")
                    }
                    FaceKind::FreeForm => "free-form face".into(),
                };
                out.push(format!("{what}: area {area:.4} mm²"));
            }
        }
        Pick::Edge(e) => {
            if let Some(k) = edge_kind(brep, *e) {
                out.push(match k {
                    EdgeKind::Line { length, .. } => {
                        format!("straight edge: length {length:.4} mm")
                    }
                    EdgeKind::Circle { radius, length } => {
                        format!("circular edge: radius {radius:.4} mm, length {length:.4} mm")
                    }
                    EdgeKind::Curve { length } => format!("curved edge: length {length:.4} mm"),
                });
            }
        }
    };
    match picks {
        [] => {}
        [p] => one(p, &mut out),
        [a, b, ..] => {
            one(a, &mut out);
            one(b, &mut out);
            match (a, b) {
                (Pick::Face(fa), Pick::Face(fb)) => {
                    let (ka, kb) = (face_kind(brep, *fa), face_kind(brep, *fb));
                    if let (
                        Some((FaceKind::Plane { normal: na }, _)),
                        Some((FaceKind::Plane { normal: nb }, _)),
                    ) = (&ka, &kb)
                    {
                        let c = dot(*na, *nb).clamp(-1.0, 1.0);
                        let angle = c.acos().to_degrees();
                        if (c.abs() - 1.0).abs() < 1e-9 {
                            // parallel planes: distance along the normal between their points
                            if let (Some(pa), Some(pb), Some(fra), Some(_)) = (
                                face_centre(brep, *fa),
                                face_centre(brep, *fb),
                                geom::face_frame(brep, *fa),
                                geom::face_frame(brep, *fb),
                            ) {
                                let d =
                                    dot([pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]], fra.n).abs();
                                out.push(format!("parallel planes {d:.4} mm apart"));
                            }
                        } else {
                            out.push(format!("angle between the planes: {angle:.4}°"));
                        }
                    } else if let (Some(pa), Some(pb)) =
                        (face_centre(brep, *fa), face_centre(brep, *fb))
                    {
                        out.push(format!(
                            "distance between the bounding-box centres: {:.4} mm",
                            dist(pa, pb)
                        ));
                    }
                }
                (Pick::Edge(ea), Pick::Edge(eb)) => {
                    let d = polyline_distance(&edge_points(brep, *ea), &edge_points(brep, *eb));
                    out.push(format!("minimum distance between the edges: {d:.4} mm"));
                    if let (
                        Some(EdgeKind::Line { direction: da, .. }),
                        Some(EdgeKind::Line { direction: db, .. }),
                    ) = (edge_kind(brep, *ea), edge_kind(brep, *eb))
                    {
                        let c = dot(da, db).abs().clamp(0.0, 1.0);
                        out.push(format!(
                            "angle between the lines: {:.4}°",
                            c.acos().to_degrees()
                        ));
                    }
                }
                (Pick::Face(f), Pick::Edge(e)) | (Pick::Edge(e), Pick::Face(f)) => {
                    if let (Some((FaceKind::Plane { normal }, _)), Some(fr), pts) = (
                        face_kind(brep, *f),
                        geom::face_frame(brep, *f),
                        edge_points(brep, *e),
                    ) {
                        if let Some(p) = pts.first() {
                            let d = dot(
                                [
                                    p[0] - fr.origin[0],
                                    p[1] - fr.origin[1],
                                    p[2] - fr.origin[2],
                                ],
                                normal,
                            );
                            out.push(format!("edge start is {:.4} mm from the plane", d.abs()));
                        }
                    }
                }
            }
        }
    }
    out
}

// ------------------------------------------------------------------------------------------------- panel

use super::DesignWorkspace;
use crate::chrome::theme::TEXT_WEAK;
use crate::project::Project;
use eframe::egui;

impl DesignWorkspace {
    /// The Measure window: the body's properties and a description of the picked faces and edges.
    pub fn measure_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        if !self.measure_open {
            return;
        }
        let threads = {
            let ev = project.evaluation();
            let doc = project.document();
            kw_feature::threads::report(&doc, &ev.scope(&doc))
        };
        let mut open = true;
        let picks = self.model.picks.clone();
        let bodies = self.bodies.clone();
        let target = self.target_index();
        egui::Window::new(egui::RichText::new("MEASURE").size(11.5).strong())
            .open(&mut open)
            .default_pos(egui::pos2(330.0, 190.0))
            .collapsible(false)
            .default_width(330.0)
            .show(ctx, |ui| {
                let Some(bodies) = bodies else {
                    ui.label("There is no solid to measure yet.");
                    return;
                };
                ui.label(
                    egui::RichText::new(
                        "Click faces and edges in the view (two picks are compared).",
                    )
                    .size(10.5)
                    .color(TEXT_WEAK),
                );
                ui.add_space(4.0);
                // picks are ids of the drawn B-rep, which holds every body: distances between bodies work too
                let lines = describe(&bodies.brep, &picks);
                if lines.is_empty() {
                    ui.label(egui::RichText::new("Nothing picked.").color(TEXT_WEAK));
                }
                for l in &lines {
                    ui.label(l);
                }
                if !picks.is_empty() && ui.small_button("Clear picks").clicked() {
                    self.model.picks.clear();
                }
                ui.separator();
                let idx = target.unwrap_or(0).min(bodies.list.len() - 1);
                let body = &bodies.list[idx];
                ui.label(egui::RichText::new(format!("Body: {}", body.name)).strong());
                let Some((brep, id)) = body.output.brep().zip(body.output.body()) else {
                    return;
                };
                if let Some(p) = body_props(brep, id) {
                    let size = p.size();
                    let own = body
                        .material
                        .as_deref()
                        .and_then(|n| MATERIALS.iter().position(|m| m.name == n));
                    if let Some(i) = own {
                        self.material = i;
                        ui.label(
                            egui::RichText::new("material set on the body")
                                .size(10.5)
                                .color(TEXT_WEAK),
                        );
                    }
                    egui::ComboBox::from_id_salt("material")
                        .selected_text(MATERIALS[self.material.min(MATERIALS.len() - 1)].name)
                        .show_ui(ui, |ui| {
                            for (i, m) in MATERIALS.iter().enumerate() {
                                ui.selectable_value(
                                    &mut self.material,
                                    i,
                                    format!("{} ({} kg/m³)", m.name, m.density),
                                );
                            }
                        });
                    let density = MATERIALS[self.material.min(MATERIALS.len() - 1)].density;
                    egui::Grid::new("body_props")
                        .num_columns(2)
                        .spacing([14.0, 3.0])
                        .show(ui, |ui| {
                            let mut row = |k: &str, v: String| {
                                ui.label(egui::RichText::new(k).color(TEXT_WEAK));
                                ui.label(v);
                                ui.end_row();
                            };
                            row("Volume", format!("{:.3} mm³", p.volume));
                            row("Surface area", format!("{:.3} mm²", p.area));
                            row("Mass", format!("{:.4} g", p.mass_g(density)));
                            row(
                                "Centre of mass",
                                format!(
                                    "({:.3}, {:.3}, {:.3}) mm",
                                    p.centroid[0], p.centroid[1], p.centroid[2]
                                ),
                            );
                            row(
                                "Bounding box",
                                format!("{:.3} × {:.3} × {:.3} mm", size[0], size[1], size[2]),
                            );
                            let i = p.inertia_g_mm2(density);
                            row(
                                "Inertia Ixx, Iyy, Izz",
                                format!("{:.2}, {:.2}, {:.2} g·mm²", i[0][0], i[1][1], i[2][2]),
                            );
                        });
                }
                if !threads.is_empty() {
                    ui.separator();
                    ui.label(egui::RichText::new(format!("Threads ({})", threads.len())).strong());
                    for t in &threads {
                        ui.label(egui::RichText::new(t.text()).size(10.5));
                    }
                }
            });
        if !open {
            self.measure_open = false;
            self.model.picks.clear();
        }
    }
}

impl DesignWorkspace {
    /// The Interference window: which bodies share volume, and how much.
    pub fn interference_window(&mut self, ctx: &egui::Context) {
        if !self.interference_open {
            return;
        }
        let bodies = self.bodies.clone();
        let mut open = true;
        egui::Window::new(egui::RichText::new("INTERFERENCE").size(11.5).strong())
            .open(&mut open)
            .default_pos(egui::pos2(330.0, 190.0))
            .collapsible(false)
            .default_width(320.0)
            .show(ctx, |ui| {
                let Some(bodies) = bodies else {
                    ui.label("There are no bodies yet.");
                    return;
                };
                if bodies.list.len() < 2 {
                    ui.label("Interference compares two or more bodies; there is one.");
                    return;
                }
                let key = egui::Id::new(("interference", bodies.key));
                let found: Vec<crate::bodies::Overlap> =
                    ctx.data_mut(|d| d.get_temp(key)).unwrap_or_else(|| {
                        let f = crate::bodies::interference(&bodies);
                        ctx.data_mut(|d| d.insert_temp(key, f.clone()));
                        f
                    });
                if found.is_empty() {
                    ui.colored_label(
                        crate::chrome::theme::GOOD,
                        "No two bodies share any volume.",
                    );
                }
                for o in &found {
                    let (a, b) = (&bodies.list[o.a].name, &bodies.list[o.b].name);
                    match (&o.volume, &o.error) {
                        (Some(v), _) => {
                            ui.colored_label(
                                crate::chrome::theme::BAD,
                                format!("{a} and {b} overlap: {v:.3} mm³"),
                            );
                        }
                        (None, Some(e)) => {
                            ui.label(format!("{a} and {b}: could not be checked ({e})"));
                        }
                        _ => {}
                    }
                }
            });
        if !open {
            self.interference_open = false;
        }
    }
}
