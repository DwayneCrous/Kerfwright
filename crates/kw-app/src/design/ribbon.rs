//! The Design ribbon: SOLID tab (Create / Modify / Construct / Inspect / Select) and the contextual SKETCH tab.

use super::tools::Tool;
use super::{Action, DesignWorkspace, Mode};
use crate::chrome::{ribbon as rb, Icon};
use crate::project::Project;
use eframe::egui;
use kw_feature::{BodyOp, FeatureSpec};
use kw_sketch::editor::{ConstraintTool, Tool as SkTool};

const CONSTRAINTS: [(ConstraintTool, &str); 13] = [
    (ConstraintTool::Coincident, "Coincident"),
    (ConstraintTool::Horizontal, "Horizontal"),
    (ConstraintTool::Vertical, "Vertical"),
    (ConstraintTool::Parallel, "Parallel"),
    (ConstraintTool::Perpendicular, "Perpendicular"),
    (ConstraintTool::Tangent, "Tangent"),
    (ConstraintTool::Equal, "Equal"),
    (ConstraintTool::Fix, "Fix"),
    (ConstraintTool::Midpoint, "Midpoint"),
    (ConstraintTool::Concentric, "Concentric"),
    (ConstraintTool::Collinear, "Collinear"),
    (ConstraintTool::PointOnCurve, "On curve"),
    (ConstraintTool::Symmetric, "Symmetric"),
];

impl DesignWorkspace {
    pub fn ribbon(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let sketching = self.mode == Mode::Sketching;
        let mut cur = self.tab;
        let labels = [
            ("SOLID", !sketching),
            ("SURFACE", false),
            ("SKETCH", sketching),
            ("UTILITIES", false),
        ];
        if rb::tabs(ui, &labels, &mut cur) {
            self.tab = cur;
        }
        if sketching {
            self.tab = 2;
        } else if self.tab == 2 {
            self.tab = 0;
        }
        ui.add_space(2.0);
        ui.horizontal(|ui| match self.tab {
            2 => self.sketch_tab(ui),
            _ => self.solid_tab(ui, project),
        });
    }

    fn solid_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let doc = project.document();
        let has_sketch = doc
            .features
            .iter()
            .any(|f| matches!(f.spec, FeatureSpec::Sketch(_)));
        let doc_sketch_count = doc
            .features
            .iter()
            .filter(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
            .count();
        let has_solid = project.evaluation().final_solid().is_some();
        let has_tool_feature = doc.features.iter().any(|f| {
            matches!(
                f.spec,
                FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
            )
        });
        let picking = self.mode == Mode::PickPlane;
        let dlg = |t: &str| {
            self.dialog
                .as_ref()
                .is_some_and(|d| d.tool.title().starts_with(t))
        };
        let mut act: Vec<Action> = Vec::new();
        let (ex, rv, ho, fi, ch, sh, pa, cp, mi, co) = (
            dlg("EXTRUDE"),
            dlg("REVOLVE"),
            dlg("HOLE"),
            dlg("FILLET"),
            dlg("CHAMFER"),
            dlg("SHELL"),
            dlg("RECTANGULAR"),
            dlg("CIRCULAR"),
            dlg("MIRROR"),
            dlg("COMBINE"),
        );
        rb::group(ui, "CREATE", |ui| {
            if rb::button(
                ui,
                Icon::Sketch,
                "Create Sketch",
                true,
                picking,
                "Pick a plane or a planar face, then sketch on it",
            )
            .clicked()
            {
                act.push(Action::StartSketch);
            }
            if rb::button(
                ui,
                Icon::Extrude,
                "Extrude",
                has_sketch,
                ex,
                "Pull a sketch profile into a solid (E)",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::extrude(
                    None,
                    if has_solid { BodyOp::Join } else { BodyOp::New },
                )));
            }
            if rb::button(
                ui,
                Icon::Revolve,
                "Revolve",
                has_sketch,
                rv,
                "Spin a sketch profile around its axis",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::Revolve {
                    sketch: None,
                    angle: "360 deg".into(),
                    axis_y: false,
                    op: if has_solid { BodyOp::Join } else { BodyOp::New },
                }));
            }
            if rb::button(
                ui,
                Icon::Hole,
                "Hole",
                has_solid,
                ho,
                "Drill a hole into a planar face",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::default_hole()));
            }
            if rb::button(
                ui,
                Icon::Pattern,
                "Rect. Pattern",
                has_tool_feature,
                pa,
                "Repeat a feature along a direction",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::Pattern {
                    source: None,
                    dx: "20 mm".into(),
                    dy: "0 mm".into(),
                    dz: "0 mm".into(),
                    count: "3".into(),
                }));
            }
            if rb::button(
                ui,
                Icon::CircularPattern,
                "Circ. Pattern",
                has_tool_feature,
                cp,
                "Repeat a feature around an axis",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::CircularPattern {
                    source: None,
                    count: "6".into(),
                    angle: "360 deg".into(),
                    axis: 2,
                }));
            }
            if rb::button(
                ui,
                Icon::Mirror,
                "Mirror",
                has_tool_feature,
                mi,
                "Mirror a feature across a plane",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::Mirror {
                    source: None,
                    plane: 1,
                }));
            }
        });
        rb::group(ui, "MODIFY", |ui| {
            if rb::button(
                ui,
                Icon::Fillet,
                "Fillet",
                has_solid,
                fi,
                "Round edges (click edges in the view)",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::Fillet {
                    radius: "2 mm".into(),
                }));
            }
            if rb::button(ui, Icon::Chamfer, "Chamfer", has_solid, ch, "Bevel edges").clicked() {
                act.push(Action::Open(Tool::Chamfer {
                    d1: "1 mm".into(),
                    d2: "1 mm".into(),
                }));
            }
            if rb::button(
                ui,
                Icon::Shell,
                "Shell",
                has_solid,
                sh,
                "Hollow the solid, removing the faces you pick",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::Shell {
                    thickness: "2 mm".into(),
                }));
            }
            if rb::button(
                ui,
                Icon::Combine,
                "Combine",
                has_tool_feature,
                co,
                "Join, cut or intersect two tool bodies",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::Combine {
                    target: None,
                    tool: None,
                    op: BodyOp::Join,
                }));
            }
            if rb::button(
                ui,
                Icon::Parameters,
                "Parameters",
                true,
                self.params_open,
                "Named values that features can use in expressions",
            )
            .clicked()
            {
                self.params_open = !self.params_open;
            }
        });
        rb::group(ui, "MORE", |ui| {
            let sketches = doc_sketch_count;
            let join = if has_solid { BodyOp::Join } else { BodyOp::New };
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(3.0, 3.0);
                let small = |ui: &mut egui::Ui, label: &str, enabled: bool, tip: &str| {
                    ui.add_enabled(
                        enabled,
                        egui::Button::new(egui::RichText::new(label).size(10.5)),
                    )
                    .on_hover_text(tip)
                    .clicked()
                };
                ui.horizontal(|ui| {
                    if small(
                        ui,
                        "Loft",
                        sketches >= 2,
                        "Blend between two or more profile sketches",
                    ) {
                        act.push(Action::Open(Tool::Loft {
                            sketches: vec![None, None],
                            op: join,
                        }));
                    }
                    if small(
                        ui,
                        "Sweep",
                        sketches >= 2,
                        "Sweep a profile sketch along a path sketch",
                    ) {
                        act.push(Action::Open(Tool::Sweep {
                            profile: None,
                            path: None,
                            op: join,
                        }));
                    }
                });
                ui.horizontal(|ui| {
                    if small(
                        ui,
                        "Draft",
                        has_solid,
                        "Tilt faces for moulding or casting (click faces)",
                    ) {
                        act.push(Action::Open(Tool::Draft {
                            angle: "3 deg".into(),
                            axis: 2,
                            position: "0 mm".into(),
                        }));
                    }
                    if small(
                        ui,
                        "Offset body",
                        has_solid,
                        "Grow or shrink the whole body",
                    ) {
                        act.push(Action::Open(Tool::Offset {
                            distance: "1 mm".into(),
                        }));
                    }
                });
                ui.horizontal(|ui| {
                    if small(
                        ui,
                        "Var. fillet",
                        has_solid,
                        "Fillet whose radius changes along one edge",
                    ) {
                        act.push(Action::Open(Tool::VariableFillet {
                            start: "1 mm".into(),
                            end: "3 mm".into(),
                        }));
                    }
                });
            });
        });
        rb::group(ui, "CONSTRUCT", |ui| {
            if rb::button(
                ui,
                Icon::Plane,
                "Plane",
                true,
                false,
                "Construction plane: an origin plane moved and tilted",
            )
            .clicked()
            {
                act.push(Action::Open(Tool::ConstructionPlane {
                    base: 0,
                    offset: "10 mm".into(),
                    tilt: "0 deg".into(),
                }));
            }
            let any = self.plane_visible.iter().any(|v| *v);
            if rb::button(
                ui,
                Icon::Plane,
                "Origin Planes",
                true,
                any,
                "Show or hide the origin planes",
            )
            .clicked()
            {
                self.plane_visible = [!any; 3];
            }
        });
        rb::group(ui, "INSPECT", |ui| {
            if rb::button(
                ui,
                Icon::Measure,
                "Measure",
                has_solid,
                self.measure_open,
                "Distances, angles, areas, mass and centre of mass",
            )
            .clicked()
            {
                self.measure_open = !self.measure_open;
                self.model.picks.clear();
            }
        });
        rb::group(ui, "SELECT", |ui| {
            rb::button(
                ui,
                Icon::Select,
                "Select",
                true,
                self.mode == Mode::Idle && self.dialog.is_none(),
                "Click faces and edges in the view",
            );
        });
        self.actions.extend(act);
    }

    fn sketch_tab(&mut self, ui: &mut egui::Ui) {
        let cur = self.sketch.app.editor.tool;
        let set: std::cell::Cell<Option<SkTool>> = std::cell::Cell::new(None);
        let poly_sides = std::cell::Cell::new(self.sketch.app.editor.polygon_sides);
        let corner = std::cell::Cell::new(self.sketch.app.editor.corner_size);
        let mut finish = false;
        let mut cancel = false;
        let tool_btn = |ui: &mut egui::Ui, icon: Icon, label: &str, tool: SkTool, tip: &str| {
            if rb::button(ui, icon, label, true, cur == tool, tip).clicked() {
                set.set(Some(tool));
            }
        };
        rb::group(ui, "CREATE", |ui| {
            tool_btn(ui, Icon::Line, "Line", SkTool::Line, "Line (L)");
            tool_btn(
                ui,
                Icon::Rectangle,
                "Rectangle",
                SkTool::Rectangle,
                "Rectangle (R)",
            );
            tool_btn(ui, Icon::Circle, "Circle", SkTool::Circle, "Circle (C)");
            tool_btn(ui, Icon::Arc, "Arc", SkTool::Arc, "Arc (A)");
            tool_btn(
                ui,
                Icon::Spline,
                "Spline",
                SkTool::Spline,
                "Spline (S), double-click to finish",
            );
            tool_btn(ui, Icon::Point, "Point", SkTool::Point, "Point (P)");
        });
        rb::group(ui, "SHAPES", |ui| {
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(3.0, 3.0);
                let more: [(&str, SkTool, &str); 6] = [
                    (
                        "Polygon",
                        SkTool::Polygon,
                        "Regular polygon (G): click the centre, then a vertex",
                    ),
                    (
                        "Slot",
                        SkTool::Slot,
                        "Slot (U): two centres, then the width",
                    ),
                    (
                        "Ellipse",
                        SkTool::Ellipse,
                        "Ellipse (H): centre, major axis end, minor axis point",
                    ),
                    (
                        "3-pt arc",
                        SkTool::Arc3,
                        "Arc through three points (W): start, end, a point on it",
                    ),
                    (
                        "3-pt circle",
                        SkTool::Circle3,
                        "Circle through three points (X)",
                    ),
                    (
                        "Fit spline",
                        SkTool::FitSpline,
                        "Spline through its points (K); double-click or Enter to finish",
                    ),
                ];
                for chunk in more.chunks(3) {
                    ui.horizontal(|ui| {
                        for (name, t, tip) in chunk {
                            if ui
                                .add(
                                    egui::Button::new(egui::RichText::new(*name).size(10.5))
                                        .selected(cur == *t),
                                )
                                .on_hover_text(*tip)
                                .clicked()
                            {
                                set.set(Some(*t));
                            }
                        }
                    });
                }
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("sides").size(10.5));
                    let mut n = poly_sides.get();
                    if ui.add(egui::DragValue::new(&mut n).range(3..=64)).changed() {
                        poly_sides.set(n);
                    }
                });
            });
        });
        rb::group(ui, "MODIFY", |ui| {
            tool_btn(ui, Icon::Trim, "Trim", SkTool::Trim, "Trim (T)");
            tool_btn(ui, Icon::Extend, "Extend", SkTool::Extend, "Extend (E)");
            tool_btn(ui, Icon::Offset, "Offset", SkTool::Offset, "Offset (O)");
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(3.0, 3.0);
                let more: [(&str, SkTool, &str); 3] = [
                    (
                        "Fillet",
                        SkTool::Fillet,
                        "Round a corner (J): click the point where two lines meet",
                    ),
                    (
                        "Chamfer",
                        SkTool::Chamfer,
                        "Bevel a corner (B): click the point where two lines meet",
                    ),
                    (
                        "Mirror",
                        SkTool::Mirror,
                        "Mirror (M): click the entities, press Enter, click the mirror line",
                    ),
                ];
                for (name, t, tip) in more {
                    if ui
                        .add(
                            egui::Button::new(egui::RichText::new(name).size(10.5))
                                .selected(cur == t),
                        )
                        .on_hover_text(tip)
                        .clicked()
                    {
                        set.set(Some(t));
                    }
                }
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("size").size(10.5));
                    let mut n = corner.get();
                    if ui
                        .add(egui::DragValue::new(&mut n).speed(0.1).range(1e-6..=1e6))
                        .changed()
                    {
                        corner.set(n);
                    }
                });
            });
        });
        rb::group(ui, "CONSTRAINTS", |ui| {
            tool_btn(
                ui,
                Icon::Dimension,
                "Dimension",
                SkTool::Dimension,
                "Sketch dimension (D)",
            );
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(3.0, 3.0);
                for chunk in CONSTRAINTS.chunks(5) {
                    ui.horizontal(|ui| {
                        for (c, name) in chunk {
                            let t = SkTool::Constrain(*c);
                            if ui
                                .add(
                                    egui::Button::new(egui::RichText::new(*name).size(10.5))
                                        .selected(cur == t),
                                )
                                .clicked()
                            {
                                set.set(Some(t));
                            }
                        }
                    });
                }
            });
        });
        rb::group(ui, "SNAPS", |ui| {
            ui.menu_button("Snaps  v", |ui| self.sketch.app.snaps_ui(ui))
                .response
                .on_hover_text("What the cursor snaps to; hold Alt to ignore snaps");
        });
        rb::group(ui, "FINISH", |ui| {
            if rb::button(
                ui,
                Icon::Finish,
                "Finish Sketch",
                true,
                false,
                "Keep the sketch and continue",
            )
            .clicked()
            {
                finish = true;
            }
            if rb::button(
                ui,
                Icon::Generic,
                "Discard",
                true,
                false,
                "Leave without saving the sketch",
            )
            .clicked()
            {
                cancel = true;
            }
        });
        self.sketch.app.editor.polygon_sides = poly_sides.get();
        self.sketch.app.editor.corner_size = corner.get();
        if let Some(t) = set.get() {
            self.sketch.app.editor.set_tool(t);
        }
        if finish {
            self.actions.push(Action::FinishSketch);
        }
        if cancel {
            self.actions.push(Action::CancelSketch);
        }
    }
}
