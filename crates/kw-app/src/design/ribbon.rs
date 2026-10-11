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
        rb::group(ui, "CONSTRUCT", |ui| {
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
            rb::button(ui, Icon::Measure, "Measure", false, false, "Not built yet");
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
        rb::group(ui, "MODIFY", |ui| {
            tool_btn(ui, Icon::Trim, "Trim", SkTool::Trim, "Trim (T)");
            tool_btn(ui, Icon::Extend, "Extend", SkTool::Extend, "Extend (E)");
            tool_btn(ui, Icon::Offset, "Offset", SkTool::Offset, "Offset (O)");
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
