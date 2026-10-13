//! The click tools for dimensions, notes and symbols.

use super::source::source_for;
use super::{DrawingWorkspace, Tool};
use crate::chrome::theme::ACCENT;
use crate::project::Project;
use eframe::egui::{self, Color32, Pos2, Rect, Stroke};
use kw_drawing::dim::{self, Dimension, Resolved};
use kw_drawing::projection::ViewFrame;
use kw_drawing::snap::{distance, Cand, CandKind};
use kw_drawing::{DimKind, Gdt, ViewKind};

use super::canvas::paint_scene;

/// Pick radius, screen pixels.
const SNAP_PX: f64 = 9.0;

fn resolved_of(c: &Cand) -> Resolved {
    match c.kind {
        CandKind::Point(_) => Resolved::Point(c.p3),
        CandKind::Circle => {
            let (centre, radius, axis) = c.circle.unwrap_or((c.p3, 0.0, [0.0, 0.0, 1.0]));
            Resolved::Circle {
                centre,
                radius,
                axis,
            }
        }
        CandKind::Line => {
            let (a, b) = c.line.unwrap_or((c.p3, c.p3));
            Resolved::Line { a, b }
        }
    }
}

/// A dimension of the picks, not yet in the drawing: for the preview and for working out where
/// it goes.
pub fn sketch_dim(view: u64, kind: DimKind, picks: &[Cand], frame: &ViewFrame) -> Dimension {
    let r: Vec<Resolved> = picks.iter().map(resolved_of).collect();
    let mut d = Dimension::new(
        view,
        kind,
        r.iter()
            .cloned()
            .map(kw_drawing::dim::Anchor::Fixed)
            .collect(),
    );
    d.resolved = r;
    d.value = dim::measure(&d, frame).unwrap_or(0.0);
    d
}

impl DrawingWorkspace {
    pub(super) fn annot_tool_ui(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rect: Rect,
        painter: &egui::Painter,
        resp: &egui::Response,
        tool: Tool,
    ) {
        let view2d = self.view;
        let px = view2d.scale;
        let to = |p: [f64; 2]| view2d.to_screen(rect, (p[0], p[1]));
        let world = |p: Pos2| {
            let w = view2d.to_world(rect, p);
            [w.0, w.1]
        };
        let hover = ui
            .ctx()
            .pointer_hover_pos()
            .filter(|p| rect.contains(*p))
            .map(world);
        let click = if resp.clicked_by(egui::PointerButton::Primary) {
            ui.ctx().pointer_interact_pos().map(world)
        } else {
            None
        };
        let stroke = Stroke::new(1.5, ACCENT);
        let done = |me: &mut Self, res: Result<u64, String>| {
            me.tool = None;
            me.status = res.err().unwrap_or_default();
        };
        match tool {
            Tool::Note { leader, target } => {
                if let (Some(t), Some(h)) = (target, hover) {
                    painter.line_segment([to(t), to(h)], stroke);
                }
                if let Some(c) = click {
                    match (leader, target) {
                        (true, None) => {
                            self.tool = Some(Tool::Note {
                                leader,
                                target: Some(c),
                            })
                        }
                        (_, t) => {
                            let r = self.add_note(project, c, "NOTE", t);
                            done(self, r);
                        }
                    }
                }
            }
            Tool::Datum { target } => {
                if let (Some(t), Some(h)) = (target, hover) {
                    painter.line_segment([to(t), to(h)], stroke);
                }
                if let Some(c) = click {
                    match target {
                        None => self.tool = Some(Tool::Datum { target: Some(c) }),
                        Some(t) => {
                            let r = self.add_datum(project, c, t);
                            done(self, r);
                        }
                    }
                }
            }
            Tool::Surface => {
                if let Some(c) = click {
                    let r = self.add_surface(project, c, "Ra 3.2");
                    done(self, r);
                }
            }
            Tool::Balloon { first } => {
                // the first click picks the component, the second places the balloon
                let sheet_i = self.sheet;
                let under = |p: [f64; 2]| {
                    project.drawings.sheets[sheet_i]
                        .views
                        .iter()
                        .rev()
                        .find(|v| {
                            v.cache.as_ref().and_then(|c| c.scene.bounds()).is_some_and(
                                |(lo, hi)| {
                                    p[0] >= v.pos[0] + lo[0]
                                        && p[0] <= v.pos[0] + hi[0]
                                        && p[1] >= v.pos[1] + lo[1]
                                        && p[1] <= v.pos[1] + hi[1]
                                },
                            )
                        })
                        .cloned()
                };
                if let (Some((vid, uv)), Some(h)) = (first, hover) {
                    if let Some(v) = project.drawings.sheets[sheet_i].view(vid) {
                        if let Some(c) = &v.cache {
                            let at = [
                                v.pos[0] + uv[0] * v.scale - c.centre[0],
                                v.pos[1] + uv[1] * v.scale - c.centre[1],
                            ];
                            painter.line_segment([to(at), to(h)], stroke);
                        }
                    }
                }
                if let Some(c) = click {
                    match first {
                        None => {
                            if let Some(v) = under(c) {
                                let cc = v.cache.as_ref().map(|x| x.centre).unwrap_or([0.0; 2]);
                                let uv = [
                                    (c[0] - v.pos[0] + cc[0]) / v.scale,
                                    (c[1] - v.pos[1] + cc[1]) / v.scale,
                                ];
                                self.tool = Some(Tool::Balloon {
                                    first: Some((v.id, uv)),
                                });
                            }
                        }
                        Some((vid, uv)) => {
                            let r = self.add_balloon(project, vid, uv, c);
                            done(self, r);
                        }
                    }
                }
            }
            Tool::Fcf { target } => {
                if let (Some(t), Some(h)) = (target, hover) {
                    painter.line_segment([to(t), to(h)], stroke);
                }
                if let Some(c) = click {
                    match target {
                        None => self.tool = Some(Tool::Fcf { target: Some(c) }),
                        Some(t) => {
                            let r = self.add_fcf(project, c, Gdt::Position, "Ø0.1", &[], Some(t));
                            done(self, r);
                        }
                    }
                }
            }
            Tool::Dim {
                kind,
                smart,
                view,
                picks,
            } => self.dim_tool(
                ui, project, painter, &to, px, hover, click, kind, smart, view, picks,
            ),
            Tool::Section { .. } | Tool::Detail { .. } => {}
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn dim_tool(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        painter: &egui::Painter,
        to: &dyn Fn([f64; 2]) -> Pos2,
        px: f64,
        hover: Option<[f64; 2]>,
        click: Option<[f64; 2]>,
        kind: DimKind,
        smart: bool,
        view: Option<u64>,
        picks: Vec<Cand>,
    ) {
        let _ = ui;
        let want = match kind {
            DimKind::Diameter | DimKind::Radius => 1,
            _ => 2,
        };
        let sheet_i = self.sheet;
        // the view being dimensioned: the one chosen by the first click, else the one under the pointer
        let under = |p: [f64; 2]| -> Option<u64> {
            project.drawings.sheets[sheet_i]
                .views
                .iter()
                .rev()
                .find(|v| {
                    let Some(c) = &v.cache else { return false };
                    c.scene.bounds().is_some_and(|(lo, hi)| {
                        p[0] >= v.pos[0] + lo[0] - 10.0
                            && p[0] <= v.pos[0] + hi[0] + 10.0
                            && p[1] >= v.pos[1] + lo[1] - 10.0
                            && p[1] <= v.pos[1] + hi[1] + 10.0
                    })
                })
                .map(|v| v.id)
        };
        let vid = view.or_else(|| hover.or(click).and_then(under));
        let Some(vid) = vid else { return };
        let Some(v) = project.drawings.sheets[sheet_i].view(vid).cloned() else {
            return;
        };
        let Some(cache) = v.cache.clone() else { return };
        if matches!(v.kind, ViewKind::Section { .. } | ViewKind::Detail { .. }) {
            self.tool = None;
            self.status = "dimensions on section and detail views are not available yet".into();
            return;
        }
        if self.cands_view != Some(vid) {
            match self.candidates_for(project, vid) {
                Ok(c) => {
                    self.cands = c;
                    self.cands_view = Some(vid);
                }
                Err(e) => {
                    self.tool = None;
                    self.status = e;
                    return;
                }
            }
        }
        let frame = ViewFrame::of(v.orientation);
        let local = |p: [f64; 2]| [p[0] - v.pos[0], p[1] - v.pos[1]];
        let allowed = |c: &Cand| super::dims::fits(kind, c.kind);
        let nearest = |p: [f64; 2]| -> Option<&Cand> {
            let l = local(p);
            self.cands
                .iter()
                .filter(|c| allowed(c) && !picks.contains(c))
                .map(|c| (c, distance(c, l, v.scale, &frame)))
                .filter(|(_, d)| *d <= SNAP_PX / px)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(c, _)| c)
        };
        let tint = Stroke::new(2.0, ACCENT);
        let sheet_of = |l: [f64; 2]| to([v.pos[0] + l[0], v.pos[1] + l[1]]);
        // what is already picked
        for c in &picks {
            mark(painter, &sheet_of, c, v.scale, px, tint);
        }
        if picks.len() < want {
            if let Some(c) = hover.and_then(nearest) {
                mark(
                    painter,
                    &sheet_of,
                    c,
                    v.scale,
                    px,
                    Stroke::new(2.0, Color32::from_rgb(230, 140, 20)),
                );
            }
            if let Some(p) = click {
                if let Some(c) = nearest(p).cloned() {
                    let mut picks = picks;
                    picks.push(c);
                    self.tool = Some(Tool::Dim {
                        kind,
                        smart,
                        view: view_if(&picks, vid),
                        picks,
                    });
                }
            } else {
                self.tool = Some(Tool::Dim {
                    kind,
                    smart,
                    view: view_if(&picks, vid),
                    picks,
                });
            }
            return;
        }
        // all picked: the dimension follows the pointer until it is placed
        let mut k = kind;
        let mid = |picks: &[Cand]| {
            let n = picks.len() as f64;
            [
                picks.iter().map(|c| c.p2[0]).sum::<f64>() / n,
                picks.iter().map(|c| c.p2[1]).sum::<f64>() / n,
            ]
        };
        let decide = |p: [f64; 2]| -> DimKind {
            if !smart {
                return kind;
            }
            let (m, l) = (mid(&picks), local(p));
            if (l[1] - m[1]).abs() >= (l[0] - m[0]).abs() {
                DimKind::Horizontal
            } else {
                DimKind::Vertical
            }
        };
        if let Some(h) = hover.or(click) {
            k = decide(h);
            let mut d = sketch_dim(vid, k, &picks, &frame);
            dim::place(&mut d, &v, &cache, local(h));
            let scene = dim::layout(
                &d,
                &v,
                &cache,
                project.drawings.sheets[sheet_i].format.units,
            );
            paint_scene(painter, &scene, to, px, Some(ACCENT));
        }
        if let Some(p) = click {
            k = decide(p);
            let mut d = sketch_dim(vid, k, &picks, &frame);
            dim::place(&mut d, &v, &cache, local(p));
            let r = self.add_dimension(project, vid, k, &picks, d.line_pos, d.angle);
            self.tool = None;
            self.status = r.err().unwrap_or_default();
        } else {
            self.tool = Some(Tool::Dim {
                kind,
                smart,
                view: view_if(&picks, vid),
                picks,
            });
        }
        let _ = (source_for, k);
    }
}

/// The view a dimension tool is tied to: fixed by the first pick, free while nothing is picked.
fn view_if(picks: &[Cand], vid: u64) -> Option<u64> {
    (!picks.is_empty()).then_some(vid)
}

/// A marker on a pickable point or edge.
fn mark(
    painter: &egui::Painter,
    sheet_of: &dyn Fn([f64; 2]) -> Pos2,
    c: &Cand,
    scale: f64,
    px: f64,
    stroke: Stroke,
) {
    match c.kind {
        CandKind::Point(_) => {
            let p = sheet_of(c.p2);
            painter.rect_stroke(
                Rect::from_center_size(p, egui::vec2(9.0, 9.0)),
                0.0,
                stroke,
                egui::StrokeKind::Middle,
            );
        }
        CandKind::Circle => {
            let r = c.circle.map_or(0.0, |(_, r, _)| r * scale * px);
            painter.circle_stroke(sheet_of(c.p2), r as f32, stroke);
        }
        CandKind::Line => {
            if let Some((a, b)) = c.line2 {
                painter.line_segment([sheet_of(a), sheet_of(b)], stroke);
            }
        }
    }
}
