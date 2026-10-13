//! The application: a welcome page and, once a project is open, the project view with its workspaces.

use crate::cam::CamWorkspace;
use crate::chrome::{icons, ribbon as rb, theme, Icon};
use crate::design::DesignWorkspace;
use crate::dialogs::{Dialogs, NativeDialogs};
use crate::drawing::DrawingWorkspace;
use crate::ecad::EcadWorkspace;
use crate::fea::FeaWorkspace;
use crate::project::Project;
use crate::recent::RecentList;
use crate::render::RenderWorkspace;
use eframe::egui;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Workspace {
    /// Sketch and Model together, like Fusion's Design environment.
    Design,
    Cam,
    Fea,
    Ecad,
    Render,
    /// 2-D drawing sheets of the design.
    Drawing,
}

impl Workspace {
    pub const ALL: [Workspace; 6] = [
        Workspace::Design,
        Workspace::Cam,
        Workspace::Fea,
        Workspace::Ecad,
        Workspace::Render,
        Workspace::Drawing,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Workspace::Design => "DESIGN",
            Workspace::Cam => "MANUFACTURE",
            Workspace::Fea => "SIMULATION",
            Workspace::Ecad => "ELECTRONICS",
            Workspace::Render => "RENDER",
            Workspace::Drawing => "DRAWING",
        }
    }

    pub fn blurb(self) -> &'static str {
        match self {
            Workspace::Design => "Sketches and the 3-D model with its timeline.",
            Workspace::Cam => "Toolpaths, stock simulation and G-code.",
            Workspace::Fea => "Meshing and static stress analysis.",
            Workspace::Ecad => "Circuit board placement, routing and fabrication outputs.",
            Workspace::Render => "Realtime and path-traced images.",
            Workspace::Drawing => "Dimensioned drawing sheets with views of the design.",
        }
    }

    fn icon(self) -> Icon {
        match self {
            Workspace::Design => Icon::Extrude,
            Workspace::Cam => Icon::Cam,
            Workspace::Fea => Icon::Fea,
            Workspace::Ecad => Icon::Board,
            Workspace::Render => Icon::Render,
            Workspace::Drawing => Icon::Drawing,
        }
    }
}

/// What the user asked for that has to wait for a decision about unsaved changes.
#[derive(Debug, Clone, PartialEq)]
pub enum Pending {
    CloseProject,
    Open(PathBuf),
    OpenDemo,
    Quit,
}

pub enum Screen {
    Welcome,
    Project {
        project: Box<Project>,
        workspace: Workspace,
    },
}

pub struct KerfwrightApp {
    pub screen: Screen,
    pub recents: RecentList,
    recents_file: Option<PathBuf>,
    dialogs: Box<dyn Dialogs>,
    pub pending: Option<Pending>,
    pub design: DesignWorkspace,
    pub ecad: EcadWorkspace,
    pub cam: CamWorkspace,
    pub fea: FeaWorkspace,
    pub render: RenderWorkspace,
    pub drawing: DrawingWorkspace,
    pub message: String,
    pub new_name: String,
    pub new_folder: Option<PathBuf>,
    pub frames: u64,
    pub exit_after_frames: Option<u64>,
    /// Set when the user confirmed quitting; the window may close.
    pub quit_ok: bool,
    /// Quitting was confirmed in the unsaved-changes dialog: close the window on the next chance.
    pub close_now: bool,
    /// Save a screenshot of the window to this file after `screenshot_after` frames, then quit.
    pub screenshot: Option<(PathBuf, u64)>,
    themed: bool,
    /// Hold the ribbon dropdown of this group open (screenshots).
    pub force_menu: Option<String>,
}

impl Default for KerfwrightApp {
    fn default() -> Self {
        let recents_file = RecentList::default_file();
        let recents = recents_file
            .as_deref()
            .map(RecentList::load)
            .unwrap_or_default();
        KerfwrightApp::with(recents, recents_file, Box::new(NativeDialogs))
    }
}

impl KerfwrightApp {
    pub fn with(
        recents: RecentList,
        recents_file: Option<PathBuf>,
        dialogs: Box<dyn Dialogs>,
    ) -> Self {
        KerfwrightApp {
            screen: Screen::Welcome,
            recents,
            recents_file,
            dialogs,
            pending: None,
            design: DesignWorkspace::default(),
            themed: false,
            force_menu: None,
            ecad: EcadWorkspace::default(),
            cam: CamWorkspace::default(),
            fea: FeaWorkspace::default(),
            render: RenderWorkspace::default(),
            drawing: DrawingWorkspace::default(),
            message: String::new(),
            new_name: "Untitled".into(),
            new_folder: std::env::var_os("HOME").map(PathBuf::from),
            frames: 0,
            exit_after_frames: None,
            quit_ok: false,
            close_now: false,
            screenshot: None,
        }
    }

    pub fn project(&self) -> Option<&Project> {
        match &self.screen {
            Screen::Project { project, .. } => Some(project),
            Screen::Welcome => None,
        }
    }

    pub fn project_mut(&mut self) -> Option<&mut Project> {
        match &mut self.screen {
            Screen::Project { project, .. } => Some(project),
            Screen::Welcome => None,
        }
    }

    pub fn workspace(&self) -> Option<Workspace> {
        match &self.screen {
            Screen::Project { workspace, .. } => Some(*workspace),
            Screen::Welcome => None,
        }
    }

    /// Open the Design workspace editing an existing sketch feature.
    pub fn edit_sketch(&mut self, id: kw_feature::FeatureId) -> Result<(), String> {
        let Screen::Project { project, .. } = &mut self.screen else {
            return Err("no project is open".into());
        };
        self.design.edit_sketch(project, id);
        if !self.design.model.error.is_empty() {
            return Err(self.design.model.error.clone());
        }
        self.set_workspace(Workspace::Design);
        Ok(())
    }

    /// A project with a generated board, routed, in the ECAD workspace (for demos and screenshots).
    /// The demo part with a drawing sheet: front, top and right views and an isometric one.
    pub fn drawing_demo(&mut self) {
        use kw_drawing::projection::Side;
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            let form = self.drawing.new_drawing_form(project);
            if self.drawing.new_drawing(project, &form).is_ok() {
                self.drawing.wait(project);
                if let Some(base) = self.drawing.selected {
                    for side in [Side::Right, Side::Above] {
                        let _ = self.drawing.add_projected(project, base, side);
                        self.drawing.wait(project);
                    }
                    let iso = crate::drawing::ViewForm {
                        orientation: kw_drawing::Orientation::Iso,
                        style: kw_drawing::ViewStyle::VisibleHidden,
                        assembly: false,
                        scale: None,
                    };
                    if self.drawing.add_base_view(project, &iso, false).is_ok() {
                        self.drawing.wait(project);
                    }
                    // a section down the middle of the front view and a detail of its corner
                    let _ =
                        self.drawing
                            .add_section(project, base, [20.0, -2.0], [20.0, 12.0], false);
                    self.drawing.wait(project);
                    let _ = self
                        .drawing
                        .add_detail(project, base, [0.0, 0.0], 6.0, None);
                    self.drawing.wait(project);
                    Self::demo_dimensions(&mut self.drawing, project, base);
                }
            }
            self.drawing.selected = None;
            self.drawing.selected_annot = None;
        }
        self.set_workspace(Workspace::Drawing);
    }

    /// Dimensions, a note and a datum on the demo's front view.
    fn demo_dimensions(drawing: &mut DrawingWorkspace, project: &mut Project, base: u64) {
        use kw_drawing::snap::CandKind;
        let Ok(cands) = drawing.candidates_for(project, base) else {
            return;
        };
        let corner = |x: f64, z: f64| {
            cands
                .iter()
                .find(|c| {
                    matches!(c.kind, CandKind::Point(_))
                        && (c.p3[0] - x).abs() < 1e-9
                        && (c.p3[2] - z).abs() < 1e-9
                        && c.p3[1].abs() < 1e-9
                })
                .cloned()
        };
        let Some(v) = project
            .drawings
            .sheets
            .first()
            .and_then(|s| s.view(base))
            .cloned()
        else {
            return;
        };
        let size = v.size();
        if let (Some(a), Some(b), Some(c)) =
            (corner(0.0, 0.0), corner(40.0, 0.0), corner(0.0, 10.0))
        {
            let _ = drawing.add_dimension(
                project,
                base,
                kw_drawing::DimKind::Horizontal,
                &[a.clone(), b],
                -(size[1] / 2.0 + 12.0),
                0.0,
            );
            let _ = drawing.add_dimension(
                project,
                base,
                kw_drawing::DimKind::Vertical,
                &[a, c],
                -(size[0] / 2.0 + 12.0),
                0.0,
            );
        }
        let _ = drawing.add_note(
            project,
            [v.pos[0] + size[0] / 2.0 + 130.0, v.pos[1] + 40.0],
            "DEBURR ALL EDGES\n0.5 MAX",
            None,
        );
    }

    pub fn ecad_demo(&mut self, index: usize) {
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            self.ecad.generate_sample(project, index);
            if let Some(b) = project.board.take() {
                let mut r = kw_ecad::route::Router::new(b, Default::default());
                r.route_all();
                project.board = Some(r.board);
            }
            self.ecad.check(project);
        }
        self.set_workspace(Workspace::Ecad);
    }

    /// Put the Design workspace into a state for screenshots and demos: `idle`, `pick`, `sketch`, `extrude`
    /// (rectangle sketched on XY, Extrude dialog open), `face` (sketch on the top face of the demo block),
    /// `faceextrude`, `hole`, `fillet`, `params`.
    pub fn design_demo(&mut self, what: &str) {
        use crate::design::{geom, tools::Tool, Action, Mode};
        self.open_demo();
        self.set_workspace(Workspace::Design);
        let Screen::Project { project, .. } = &mut self.screen else {
            return;
        };
        let rect = |w: f64, h: f64| {
            let mut sk = kw_sketch::Sketch::new();
            let pts: Vec<_> = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
                .iter()
                .map(|p| sk.add_point(p.0, p.1))
                .collect();
            let l: Vec<_> = (0..4)
                .map(|i| sk.add_line(pts[i], pts[(i + 1) % 4]))
                .collect();
            let _ = sk.horizontal(l[0]);
            let _ = sk.horizontal(l[2]);
            let _ = sk.vertical(l[1]);
            let _ = sk.vertical(l[3]);
            let _ = sk.fix(pts[0]);
            sk
        };
        let top_face = |project: &mut Project| {
            let ev = project.evaluation();
            let (_, out) = ev.final_solid()?;
            let brep = out.brep()?.clone();
            let names = out.names.clone();
            let mut best = None;
            for f in brep.faces.keys() {
                if let (Some(fr), Some(bb)) = (geom::face_frame(&brep, f), brep.face_bbox(f)) {
                    if fr.n[2] > 0.99
                        && (best.is_none()
                            || bb.max.z
                                > best
                                    .map(|b: (kw_brep::FaceId, f64)| b.1)
                                    .unwrap_or(f64::MIN))
                    {
                        best = Some((f, bb.max.z));
                    }
                }
            }
            let (f, _) = best?;
            Some((
                geom::face_frame(&brep, f)?,
                names.reference_to(&brep, f)?,
                f,
            ))
        };
        match what {
            "pick" => self.design.actions.push(Action::StartSketch),
            "sketch" | "extrude" | "sketch-done" => {
                self.design
                    .begin_sketch(project, geom::origin_spec(0), geom::origin_frame(0));
                self.design
                    .sketch
                    .app
                    .editor
                    .replace_sketch(rect(30.0, 20.0));
                self.design.sketch.app.zoom_to_fit();
                if what == "extrude" || what == "sketch-done" {
                    self.design.finish_sketch(project);
                }
                if what == "sketch-done" {
                    // Cancel the Extrude dialog: the sketch stays, un-extruded
                    self.design.close_dialog();
                }
            }
            "snap" | "snap-mid" | "snap-cross" | "snap-origin" | "snap-point" => {
                self.design
                    .begin_sketch(project, geom::origin_spec(0), geom::origin_frame(0));
                let app = &mut self.design.sketch.app;
                let mut sk = rect(30.0, 20.0);
                let far = sk.add_point(40.0, 12.0);
                // a diagonal across the rectangle crossing the loose point's level
                sk.add_line_xy(5.0, 18.0, 20.0, -2.0);
                app.editor.replace_sketch(sk);
                app.editor.set_tool(kw_sketch::editor::Tool::Line);
                app.demo_cursor = Some(match what {
                    "snap-mid" => (15.0, 0.1),
                    "snap-cross" => (18.55, 0.05),
                    "snap-origin" => (0.1, 0.1),
                    "snap-point" => (30.05, 20.05),
                    _ => {
                        // start on the middle of the bottom edge, remember the loose point, hover
                        // where the 60 degree ray from the start crosses the level of that point
                        app.editor
                            .handle(kw_sketch::editor::Input::Click((15.0, 0.02)));
                        app.editor.acquire(far);
                        (15.0 + 12.0 / 60f64.to_radians().tan(), 12.05)
                    }
                });
                app.zoom_to_fit();
            }
            "regions" => {
                self.design
                    .begin_sketch(project, geom::origin_spec(0), geom::origin_frame(0));
                let mut sk = rect(30.0, 20.0);
                sk.add_circle_xy(30.0, 10.0, 8.0);
                self.design.sketch.app.editor.replace_sketch(sk);
                self.design.finish_sketch(project);
                if let Some(crate::design::tools::Dialog {
                    tool:
                        Tool::Extrude {
                            more, depth, op, ..
                        },
                    ..
                }) = self.design.dialog.as_mut()
                {
                    *depth = "8 mm".into();
                    *op = kw_feature::BodyOp::Join;
                    more.thin = String::new();
                }
            }
            "chord" | "chord-sketch" => {
                // a line across a circle, and two bars crossing: every enclosed area is a region
                self.design
                    .begin_sketch(project, geom::origin_spec(0), geom::origin_frame(0));
                let mut sk = kw_sketch::Sketch::new();
                sk.add_circle_xy(10.0, 12.0, 8.0);
                sk.add_line_xy(0.0, 15.0, 20.0, 15.0);
                for (x0, y0, x1, y1) in [(30.0, 8.0, 50.0, 14.0), (36.0, 2.0, 44.0, 20.0)] {
                    let p: Vec<_> = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                        .iter()
                        .map(|q| sk.add_point(q.0, q.1))
                        .collect();
                    for i in 0..4 {
                        sk.add_line(p[i], p[(i + 1) % 4]);
                    }
                }
                self.design.sketch.app.editor.replace_sketch(sk);
                self.design.sketch.app.zoom_to_fit();
                if what == "chord" {
                    self.design.finish_sketch(project);
                    if let Some(crate::design::tools::Dialog {
                        tool: Tool::Extrude { depth, op, .. },
                        ..
                    }) = self.design.dialog.as_mut()
                    {
                        *depth = "6 mm".into();
                        *op = kw_feature::BodyOp::New;
                    }
                }
            }
            "bodies" => {
                // the demo block, a moved copy of it, a third body made by splitting a second copy
                let _ = project.history.edit(|d| {
                    let a = d.heads()[0];
                    let spec = |dx: f64, copy: bool| {
                        kw_feature::FeatureSpec::MoveBody(kw_feature::MoveBodySpec {
                            base: Some(a),
                            delta: [
                                kw_expr::Param::length(dx),
                                kw_expr::Param::length(0.0),
                                kw_expr::Param::length(0.0),
                            ],
                            axis: 2,
                            angle: kw_expr::Param::angle_deg(0.0),
                            copy,
                        })
                    };
                    let c1 = d.add("Copy", spec(45.0, true))?;
                    d.set_appearance(
                        c1,
                        kw_feature::Appearance {
                            name: Some("Bracket".into()),
                            colour: Some([214, 96, 60]),
                            material: None,
                        },
                    );
                    d.set_appearance(
                        a,
                        kw_feature::Appearance {
                            name: Some("Base plate".into()),
                            colour: Some([96, 140, 200]),
                            material: None,
                        },
                    );
                    Ok::<_, kw_feature::TimelineError>(())
                });
                self.design.body_props = None;
            }
            "shapes" => {
                use kw_sketch::editor::{Input, Tool as T};
                self.design
                    .begin_sketch(project, geom::origin_spec(0), geom::origin_frame(0));
                let app = &mut self.design.sketch.app;
                app.editor.polygon_sides = 5;
                let ed = &mut app.editor;
                let mut go = |tool: T, pts: &[(f64, f64)]| {
                    ed.set_tool(tool);
                    for p in pts {
                        ed.handle(Input::Click(*p));
                    }
                    ed.handle(Input::Finish);
                    ed.handle(Input::Escape);
                };
                go(T::Polygon, &[(-30.0, 20.0), (-30.0, 30.0)]);
                go(T::Slot, &[(0.0, 25.0), (25.0, 25.0), (12.0, 31.0)]);
                go(T::Ellipse, &[(-25.0, -5.0), (-5.0, -5.0), (-25.0, 3.0)]);
                go(T::Arc3, &[(8.0, -2.0), (28.0, -2.0), (18.0, 8.0)]);
                go(
                    T::Circle3,
                    &[(-30.0, -30.0), (-18.0, -30.0), (-24.0, -20.0)],
                );
                go(
                    T::FitSpline,
                    &[(0.0, -30.0), (8.0, -20.0), (16.0, -32.0), (28.0, -22.0)],
                );
                go(T::Slot, &[]);
                ed.set_tool(T::Slot);
                ed.handle(Input::Click((0.0, 8.0)));
                ed.handle(Input::Click((22.0, 8.0)));
                app.demo_cursor = Some((11.0, 12.0));
                app.zoom_to_fit();
            }
            "face" | "faceextrude" => {
                if let Some((frame, reference, _)) = top_face(project) {
                    self.design.begin_sketch(
                        project,
                        kw_feature::PlaneSpec::Face(reference),
                        frame,
                    );
                    let mut sk = kw_sketch::Sketch::new();
                    let c = sk.add_point(20.0, 12.0);
                    sk.add_circle(c, 5.0);
                    self.design.sketch.app.editor.replace_sketch(sk);
                    self.design.sketch.app.zoom_to_fit();
                    if what == "faceextrude" {
                        self.design.finish_sketch(project);
                        if let Some(d) = self.design.dialog.as_mut() {
                            if let Tool::Extrude {
                                op, flip, depth, ..
                            } = &mut d.tool
                            {
                                *op = kw_feature::BodyOp::Cut;
                                *flip = true;
                                *depth = "4 mm".into();
                            }
                        }
                    }
                }
            }
            "hole" => self.design.open_tool(project, Tool::default_hole()),
            "fillet" | "fillet-picked" => {
                self.design.open_tool(
                    project,
                    Tool::Fillet {
                        radius: "2 mm".into(),
                    },
                );
                if what == "fillet-picked" {
                    // the two long edges of the top face are picked (blue)
                    let ev = project.evaluation();
                    if let Some((_, o)) = ev.final_solid() {
                        if let Some(b) = o.brep() {
                            let zmax = b
                                .vertices
                                .values()
                                .map(|v| v.point.z)
                                .fold(f64::MIN, f64::max);
                            self.design.model.picks = b
                                .edges
                                .iter()
                                .filter(|(_, ed)| {
                                    (b.vertices[ed.start].point.z - zmax).abs() < 1e-9
                                        && (b.vertices[ed.end].point.z - zmax).abs() < 1e-9
                                })
                                .take(2)
                                .map(|(e, _)| crate::viewport::Pick::Edge(e))
                                .collect();
                        }
                    }
                }
            }
            "zebra" | "curvature" | "draft" => {
                // round the top edges so there is something curved to look at
                let ev = project.evaluation();
                let picks: Vec<crate::viewport::Pick> = ev
                    .final_solid()
                    .and_then(|(_, o)| o.brep().map(|b| (b.clone(), o.body())))
                    .map(|(b, _)| {
                        let zmax = b
                            .vertices
                            .values()
                            .map(|v| v.point.z)
                            .fold(f64::MIN, f64::max);
                        b.edges
                            .iter()
                            .filter(|(_, ed)| {
                                (b.vertices[ed.start].point.z - zmax).abs() < 1e-9
                                    && (b.vertices[ed.end].point.z - zmax).abs() < 1e-9
                            })
                            .map(|(e, _)| crate::viewport::Pick::Edge(e))
                            .collect()
                    })
                    .unwrap_or_default();
                self.design.model.picks = picks;
                let _ = self.design.model.add_fillet(project, "3 mm");
                self.design.model.picks.clear();
                use crate::design::analysis::{AKind, AnalysisState};
                self.design.analysis = Some(AnalysisState::new(match what {
                    "zebra" => AKind::Zebra,
                    "curvature" => AKind::Mean,
                    _ => AKind::Draft,
                }));
            }
            "surface" | "surface-zebra" => {
                // a vase: a smooth surface through three rings, its open edges marked
                use kw_expr::Param;
                use kw_feature::{BodyOp, FeatureSpec, LoftSpec, PlaneSpec};
                let mm = Param::length;
                let _ = self.design.model.edit(project, |d| {
                    let mut rings = Vec::new();
                    for (z, dia) in [(0.0, 26.0), (16.0, 14.0), (34.0, 22.0)] {
                        rings.push(d.add_circle_sketch(
                            &format!("Ring {z}"),
                            PlaneSpec::Xy(mm(z)),
                            mm(0.0),
                            mm(0.0),
                            mm(dia),
                        )?);
                    }
                    d.add(
                        "Vase",
                        FeatureSpec::Loft(LoftSpec {
                            sketches: rings,
                            op: BodyOp::New,
                            base: None,
                            surface: true,
                            ruled: false,
                        }),
                    )
                });
                self.design.ribbon_tab = 1;
                if what == "surface-zebra" {
                    use crate::design::analysis::{AKind, AnalysisState};
                    self.design.analysis = Some(AnalysisState::new(AKind::Zebra));
                }
            }
            "construct" => {
                use crate::design::tools::{PlaneInput, PointInput};
                let pt = |x: &str, y: &str, z: &str| PointInput {
                    datum: None,
                    x: x.into(),
                    y: y.into(),
                    z: z.into(),
                };
                for tool in [
                    Tool::Axis {
                        mode: 0,
                        pts: [
                            pt("20 mm", "12.5 mm", "-5 mm"),
                            pt("20 mm", "12.5 mm", "20 mm"),
                        ],
                        at_u: "0 mm".into(),
                        at_v: "0 mm".into(),
                        planes: Default::default(),
                    },
                    Tool::ConstructionPlane {
                        base: 0,
                        offset: "16 mm".into(),
                        tilt: "0 deg".into(),
                        more: Default::default(),
                    },
                    Tool::DatumPoint {
                        mode: 0,
                        coords: pt("30 mm", "6 mm", "16 mm"),
                        axis: None,
                        plane: PlaneInput::default(),
                    },
                ] {
                    self.design.open_tool(project, tool);
                    self.design.commit_dialog(project);
                }
            }
            "section" => {
                self.design.section = Some(crate::design::section::SectionState {
                    which: 2,
                    datum: None,
                    offset: 12.5,
                    // keep the back half: the camera looks into the cut
                    flip: true,
                });
            }
            "assembly" | "assembly-joint" | "assembly-bom" => {
                let _ = self.design.asm_demo(project, what);
            }
            "sheet" => {
                self.design.ribbon_tab = 2;
                self.design
                    .open_tool(project, crate::design::sheetmetal::preset("u"));
                self.design.commit_dialog(project);
                let _ = self.design.sheet_flat_pattern(project);
            }
            "form" => {
                self.design.ribbon_tab = 3;
                let (size, sides) = crate::design::form::defaults(1);
                self.design.open_tool(
                    project,
                    Tool::Form {
                        kind: 1,
                        size,
                        sides,
                        cage: crate::design::form::primitive(1, size, sides),
                        scale: "1".into(),
                        tolerance: "0.2 mm".into(),
                        vertex: 2,
                        built: (1, size, sides),
                    },
                );
            }
            "tools" => self.design.ribbon_tab = 4,
            "text" => {
                self.design
                    .begin_sketch(project, geom::origin_spec(0), geom::origin_frame(0));
                let _ =
                    self.design
                        .apply_sketch_panel(&crate::design::sketch_ops::SketchPanel::Text {
                            text: "Kerf".into(),
                            height: 12.0,
                            spacing: 0.05,
                            x: 0.0,
                            y: 0.0,
                            angle: 0.0,
                            font: 0,
                        });
                self.design.sketch.app.zoom_to_fit();
            }
            "project" => {
                self.design
                    .begin_sketch(project, geom::origin_spec(2), geom::origin_frame(2));
                self.design.sketch_project(project, false);
                self.design.sketch.app.zoom_to_fit();
            }
            "params" => {
                self.design.model.add_parameter(project, "width", "30");
                self.design.params_open = true;
            }
            _ => {}
        }
        let _ = Mode::Idle;
    }

    /// The demo part with a pocket planned in the CAM workspace (for demos and screenshots).
    pub fn cam_demo(&mut self) {
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            let ev = project.evaluation();
            let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
            if let Some(b) = &brep {
                self.cam.fit_to_model(b);
            }
            self.cam.settings.x0 += 4.0;
            self.cam.settings.y0 += 4.0;
            self.cam.settings.x1 -= 4.0;
            self.cam.settings.y1 -= 4.0;
            self.cam.settings.bottom = self.cam.settings.top - 4.0;
            let _ = self.cam.run_now(project, brep.as_ref());
        }
        self.set_workspace(Workspace::Cam);
    }

    /// The demo block clamped at one end and loaded at the other, solved, in the FEA workspace (demos and
    /// screenshots).
    pub fn fea_demo(&mut self) {
        self.open_demo();
        if let Screen::Project { project, .. } = &mut self.screen {
            let ev = project.evaluation();
            if let Some(brep) = ev.final_solid().and_then(|(_, o)| o.brep().cloned()) {
                let Some(body) = brep.bodies.keys().next() else {
                    return;
                };
                let bb = brep.body_bbox(body);
                let Ok(t) = kw_tess::tessellate_body(
                    &brep,
                    body,
                    &kw_tess::TessOptions {
                        chord_tol: 0.005,
                        angle_tol: 12f64.to_radians(),
                    },
                ) else {
                    return;
                };
                let face_at = |x: f64| {
                    t.mesh
                        .indices
                        .as_chunks::<3>()
                        .0
                        .iter()
                        .enumerate()
                        .find(|(_, tri)| {
                            tri.iter()
                                .all(|i| (t.mesh.positions[*i as usize].x - x).abs() < 1e-6)
                        })
                        .map(|(k, _)| t.mesh.tri_face[k])
                };
                if let (Some(a), Some(b)) = (face_at(bb.min.x + 0.0), face_at(bb.max.x)) {
                    self.fea.study.max_size = 3.0;
                    self.fea.fix_face(project, a);
                    self.fea.add_force(project, b, [0.0, 0.0, -2000.0]);
                    match crate::fea::solve_study(&brep, &self.fea.study) {
                        Ok(r) => {
                            self.fea.scale = (0.05 * bb.diagonal() / r.max_displacement.max(1e-30))
                                .round()
                                .max(1.0);
                            self.fea.result = Some(r);
                            self.fea.field = crate::fea::Field::VonMises;
                        }
                        Err(e) => self.message = e,
                    }
                }
            }
        }
        self.set_workspace(Workspace::Fea);
    }

    pub fn set_workspace(&mut self, w: Workspace) {
        if let Screen::Project { workspace, .. } = &mut self.screen {
            *workspace = w;
        }
    }

    fn remember(&mut self, path: &Path) {
        self.recents.touch(path.to_path_buf());
        if let Some(f) = &self.recents_file {
            if let Err(e) = self.recents.save(f) {
                self.message = format!("could not update the recent list: {e}");
            }
        }
    }

    fn enter(&mut self, project: Project) {
        self.design = DesignWorkspace::default();
        self.ecad = EcadWorkspace::default();
        self.cam = CamWorkspace::default();
        self.fea = FeaWorkspace::default();
        self.render = RenderWorkspace::default();
        self.drawing = DrawingWorkspace::default();
        self.screen = Screen::Project {
            project: Box::new(project),
            workspace: Workspace::Design,
        };
    }

    // ----------------------------------------------------------------------------- project actions

    /// A new project `folder/name.kwf`, saved at once so that it exists on disk and in the recent list.
    pub fn create_project(&mut self, name: &str, folder: &Path) -> Result<(), String> {
        let name = name.trim();
        if name.is_empty() || name.contains(['/', '\\']) {
            return Err("give the project a name without slashes".into());
        }
        if !folder.is_dir() {
            return Err(format!("{} is not a folder", folder.display()));
        }
        let path = folder.join(format!("{name}.kwf"));
        if path.exists() {
            return Err(format!("{} already exists", path.display()));
        }
        let mut p = Project::empty();
        p.save_as(&path)?;
        self.remember(&path);
        self.enter(p);
        self.message.clear();
        Ok(())
    }

    pub fn open_path(&mut self, path: &Path) -> Result<(), String> {
        match Project::open(path) {
            Ok(p) => {
                self.remember(path);
                self.enter(p);
                self.message.clear();
                Ok(())
            }
            Err(e) => {
                self.message = format!("cannot open {}: {e}", path.display());
                Err(e)
            }
        }
    }

    pub fn open_demo(&mut self) {
        self.enter(Project::demo());
        self.message.clear();
    }

    fn dirty(&self) -> bool {
        self.project().is_some_and(|p| p.is_dirty())
    }

    /// Run `what` now, or ask about unsaved changes first.
    pub fn request(&mut self, what: Pending) {
        if self.dirty() && !matches!(what, Pending::OpenDemo if self.project().is_none()) {
            self.pending = Some(what);
        } else {
            self.perform(what);
        }
    }

    fn perform(&mut self, what: Pending) {
        self.pending = None;
        match what {
            Pending::CloseProject => self.screen = Screen::Welcome,
            Pending::Open(p) => {
                let _ = self.open_path(&p);
            }
            Pending::OpenDemo => self.open_demo(),
            Pending::Quit => {
                self.quit_ok = true;
                self.close_now = true;
            }
        }
    }

    /// Save, asking for a file name when the project has none. Returns whether the project was saved.
    pub fn save(&mut self) -> bool {
        let Some(p) = self.project_mut() else {
            return false;
        };
        if p.path.is_none() {
            return self.save_as();
        }
        match p.save() {
            Ok(()) => {
                self.message = "saved".into();
                true
            }
            Err(e) => {
                self.message = format!("save failed: {e}");
                false
            }
        }
    }

    /// Dialogs the Design workspace asked for (inserting a component, exporting the assembly).
    pub fn service_design_requests(&mut self) {
        use crate::design::assembly::FileRequest;
        let Some(req) = self.design.asm.request.take() else {
            return;
        };
        match req {
            FileRequest::Insert => {
                let Some(path) = self.dialogs.open_file("Kerfwright project", &["kwf"]) else {
                    return;
                };
                self.insert_component_path(&path);
            }
            FileRequest::Import => self.import_file_dialog(),
            FileRequest::Export(f) => self.export_file_dialog(f),
            FileRequest::ExportDxf => {
                let Some(text) = self
                    .design
                    .flat
                    .as_ref()
                    .map(|f| kw_dxf::write(&f.pattern.to_dxf()))
                else {
                    return;
                };
                let name = self.project().map(|p| p.name()).unwrap_or_default();
                let Some(path) =
                    self.dialogs
                        .save_file(&format!("{name}-flat.dxf"), "DXF", &["dxf"])
                else {
                    return;
                };
                self.message = match std::fs::write(&path, text) {
                    Ok(()) => format!("wrote {}", path.display()),
                    Err(e) => format!("export failed: {e}"),
                };
            }
            FileRequest::ExportBom => {
                let name = self.project().map(|p| p.name()).unwrap_or_default();
                let Some(path) =
                    self.dialogs
                        .save_file(&format!("{name}-bom.csv"), "CSV", &["csv"])
                else {
                    return;
                };
                self.export_bom_path(&path);
            }
            FileRequest::ExportStep => {
                let name = self.project().map(|p| p.name()).unwrap_or_default();
                let Some(path) = self.dialogs.save_file(
                    &format!("{name}-assembly.step"),
                    "STEP",
                    &["step", "stp"],
                ) else {
                    return;
                };
                self.export_assembly_path(&path);
            }
        }
    }

    /// In the Design workspace with the assembly shown (and no component open) Undo and Redo work on the
    /// assembly; everywhere else on the design.
    pub fn undo_is_for_assembly(&self) -> bool {
        self.workspace() == Some(Workspace::Design)
            && self.design.asm.view
            && self.project().is_some_and(|p| p.editing().is_none())
    }

    /// In the Drawing workspace Undo and Redo work on the drawings.
    pub fn undo_is_for_drawing(&self) -> bool {
        self.workspace() == Some(Workspace::Drawing)
    }

    pub fn undo_redo(&mut self, undo: bool) {
        let on_assembly = self.undo_is_for_assembly();
        let on_drawing = self.undo_is_for_drawing();
        let Some(p) = self.project_mut() else {
            return;
        };
        if on_drawing {
            if undo {
                p.dw_undo();
            } else {
                p.dw_redo();
            }
            self.drawing.selected = None;
        } else if on_assembly {
            if undo {
                p.asm_undo();
            } else {
                p.asm_redo();
            }
            // the selection and any open joint windows referred to the old state
            self.design.asm.selected.clear();
            self.design.asm.selected_joint = None;
            self.design.asm.edit_joint = None;
            self.design.asm.edit = None;
        } else if undo {
            p.history.undo();
        } else {
            p.history.redo();
        }
    }

    pub fn insert_component_path(&mut self, path: &Path) {
        let Screen::Project { project, .. } = &mut self.screen else {
            return;
        };
        self.message = match self.design.asm_insert_path(project, path) {
            Ok(_) => format!("inserted {}", path.display()),
            Err(e) => format!("insert failed: {e}"),
        };
    }

    /// Write the bill of materials as CSV.
    pub fn export_bom_path(&mut self, path: &Path) {
        let Screen::Project { project, .. } = &mut self.screen else {
            return;
        };
        self.design.asm_refresh_pub(project);
        self.message = match self.design.asm.pose_ref() {
            Some(Ok(p)) => {
                let csv =
                    crate::design::asm_inspect::bom_csv(&crate::design::asm_inspect::bom_rows(p));
                match std::fs::write(path, csv) {
                    Ok(()) => format!("wrote {}", path.display()),
                    Err(e) => format!("export failed: {e}"),
                }
            }
            Some(Err(e)) => format!("export failed: {e}"),
            None => "export failed: there is no assembly yet".into(),
        };
    }

    pub fn export_assembly_path(&mut self, path: &Path) {
        let Screen::Project { project, .. } = &mut self.screen else {
            return;
        };
        self.message = match self
            .design
            .asm_step(project)
            .and_then(|s| std::fs::write(path, s).map_err(|e| e.to_string()))
        {
            Ok(()) => format!("wrote {}", path.display()),
            Err(e) => format!("export failed: {e}"),
        };
    }

    /// File ▸ Import: ask for a STEP / IGES file and add its solids to the project.
    pub fn import_file_dialog(&mut self) {
        let Some(path) = self
            .dialogs
            .open_file("STEP / IGES", &["step", "stp", "iges", "igs"])
        else {
            return;
        };
        self.import_path(&path);
    }

    pub fn import_path(&mut self, path: &Path) {
        let Some(p) = self.project_mut() else {
            return;
        };
        self.message = match crate::exchange::import_file(p, path) {
            Ok(m) => m,
            Err(e) => format!("import failed: {e}"),
        };
    }

    /// File ▸ Export: ask where to write the model's solid and write it.
    pub fn export_file_dialog(&mut self, format: crate::exchange::ExportFormat) {
        let suggested = format!(
            "{}.{}",
            self.project().map(|p| p.name()).unwrap_or_default(),
            format.extensions()[0]
        );
        let Some(path) = self
            .dialogs
            .save_file(&suggested, format.filter(), format.extensions())
        else {
            return;
        };
        let path = if path.extension().is_none() {
            path.with_extension(format.extensions()[0])
        } else {
            path
        };
        let Some(p) = self.project_mut() else {
            return;
        };
        self.message = match crate::exchange::export_file(p, format, &path) {
            Ok(m) => m,
            Err(e) => format!("export failed: {e}"),
        };
    }

    pub fn save_as(&mut self) -> bool {
        let suggested = format!(
            "{}.kwf",
            self.project().map(|p| p.name()).unwrap_or_default()
        );
        let Some(path) = self.dialogs.save_project(&suggested) else {
            return false;
        };
        let path = if path.extension().is_none() {
            path.with_extension("kwf")
        } else {
            path
        };
        let Some(p) = self.project_mut() else {
            return false;
        };
        match p.save_as(&path) {
            Ok(()) => {
                self.remember(&path);
                self.message = "saved".into();
                true
            }
            Err(e) => {
                self.message = format!("save failed: {e}");
                false
            }
        }
    }

    /// The answer to the unsaved-changes question.
    pub fn resolve_pending(&mut self, save_first: bool) {
        let Some(what) = self.pending.clone() else {
            return;
        };
        if save_first && !self.save() {
            return; // cancelled or failed: stay where we are
        }
        self.perform(what);
    }

    pub fn cancel_pending(&mut self) {
        self.pending = None;
    }

    /// The window asked to close. Returns true if it may.
    pub fn close_requested(&mut self) -> bool {
        if self.quit_ok {
            return true;
        }
        self.request(Pending::Quit);
        self.quit_ok
    }

    // --------------------------------------------------------------------------------------- drawing

    pub fn show(&mut self, ui: &mut egui::Ui) {
        self.show_with(ui, None);
    }

    /// Draw a frame; `rs` is the GPU state the 3-D views draw with (`None` in headless tests).
    pub fn show_with(&mut self, ui: &mut egui::Ui, rs: Option<&eframe::egui_wgpu::RenderState>) {
        self.frames += 1;
        if !self.themed {
            theme::apply(ui.ctx());
            self.themed = true;
        }
        if let Some(t) = &self.force_menu {
            let t = t.clone();
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("force_open_menu"), t));
        }
        match &self.screen {
            Screen::Welcome => self.welcome_ui(ui),
            Screen::Project { .. } => self.project_ui(ui, rs),
        }
        self.pending_ui(ui.ctx());
        if self.close_now {
            // the answer to “save before quitting?” was given: close, no second click on the button
            self.close_now = false;
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        self.screenshot_step(ui.ctx());
        if let Some(n) = self.exit_after_frames {
            if self.frames >= n {
                self.quit_ok = true;
                ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
            } else {
                ui.ctx().request_repaint();
            }
        }
    }

    fn screenshot_step(&mut self, ctx: &egui::Context) {
        let Some((path, after)) = self.screenshot.clone() else {
            return;
        };
        if self.frames == after {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
        }
        let shot = ctx.input(|i| {
            i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            let [w, h] = img.size;
            let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
            let result = std::fs::File::create(&path)
                .map_err(|e| e.to_string())
                .and_then(|f| {
                    let mut enc = png::Encoder::new(std::io::BufWriter::new(f), w as u32, h as u32);
                    enc.set_color(png::ColorType::Rgba);
                    enc.set_depth(png::BitDepth::Eight);
                    enc.write_header()
                        .map_err(|e| e.to_string())?
                        .write_image_data(&bytes)
                        .map_err(|e| e.to_string())
                });
            match result {
                Ok(()) => eprintln!("screenshot written to {}", path.display()),
                Err(e) => eprintln!("screenshot failed: {e}"),
            }
            self.screenshot = None;
            self.quit_ok = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        } else {
            ctx.request_repaint();
        }
    }

    fn welcome_ui(&mut self, ui: &mut egui::Ui) {
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(theme::CHROME))
            .show(ui, |ui| {
                let width = ui.available_width().min(980.0);
                let margin = ((ui.available_width() - width) / 2.0).max(0.0);
                ui.add_space(36.0);
                ui.horizontal(|ui| {
                    ui.add_space(margin);
                    icons::paint(
                        ui.painter(),
                        egui::Rect::from_min_size(
                            ui.cursor().min + egui::vec2(0.0, 2.0),
                            egui::vec2(44.0, 44.0),
                        ),
                        Icon::Extrude,
                        true,
                    );
                    ui.add_space(54.0);
                    ui.vertical(|ui| {
                        ui.label(
                            egui::RichText::new("Kerfwright")
                                .size(34.0)
                                .strong()
                                .color(theme::TEXT),
                        );
                        ui.label(
                            egui::RichText::new(
                                "Sketch, model, machine, analyse and design boards in one place.",
                            )
                            .size(13.0)
                            .color(theme::TEXT_WEAK),
                        );
                    });
                });
                ui.add_space(26.0);
                ui.horizontal_top(|ui| {
                    ui.add_space(margin);
                    // ---- start
                    egui::Frame::new()
                        .fill(egui::Color32::WHITE)
                        .stroke(egui::Stroke::new(1.0, theme::BORDER))
                        .corner_radius(4)
                        .inner_margin(18)
                        .show(ui, |ui| {
                            ui.set_width(width * 0.42);
                            ui.vertical(|ui| {
                                ui.label(
                                    egui::RichText::new("NEW DESIGN")
                                        .size(11.5)
                                        .strong()
                                        .color(theme::TEXT_WEAK),
                                );
                                ui.add_space(8.0);
                                ui.horizontal(|ui| {
                                    ui.add_sized([60.0, 22.0], egui::Label::new("Name"));
                                    ui.add(
                                        egui::TextEdit::singleline(&mut self.new_name)
                                            .desired_width(width * 0.42 - 90.0),
                                    );
                                });
                                ui.horizontal(|ui| {
                                    ui.add_sized([60.0, 22.0], egui::Label::new("Folder"));
                                    let shown = self
                                        .new_folder
                                        .as_ref()
                                        .map(|p| p.display().to_string())
                                        .unwrap_or_else(|| "(none chosen)".into());
                                    ui.label(egui::RichText::new(shown).monospace().size(11.5));
                                    if ui.button("Browse…").clicked() {
                                        if let Some(f) = self.dialogs.choose_folder() {
                                            self.new_folder = Some(f);
                                        }
                                    }
                                });
                                ui.add_space(8.0);
                                if ui
                                    .add_sized(
                                        [width * 0.42 - 12.0, 30.0],
                                        egui::Button::new(
                                            egui::RichText::new("Create project")
                                                .color(egui::Color32::WHITE)
                                                .strong(),
                                        )
                                        .fill(theme::ACCENT),
                                    )
                                    .clicked()
                                {
                                    let name = self.new_name.clone();
                                    match self.new_folder.clone() {
                                        Some(f) => {
                                            if let Err(e) = self.create_project(&name, &f) {
                                                self.message = e;
                                            }
                                        }
                                        None => self.message = "choose a folder first".into(),
                                    }
                                }
                                ui.add_space(14.0);
                                ui.separator();
                                ui.add_space(8.0);
                                if ui
                                    .add_sized(
                                        [width * 0.42 - 12.0, 28.0],
                                        egui::Button::new("Open project…"),
                                    )
                                    .clicked()
                                {
                                    if let Some(p) = self.dialogs.open_project() {
                                        let _ = self.open_path(&p);
                                    }
                                }
                                ui.add_space(4.0);
                                if ui
                                    .add_sized(
                                        [width * 0.42 - 12.0, 28.0],
                                        egui::Button::new("Open the demo part"),
                                    )
                                    .clicked()
                                {
                                    self.open_demo();
                                }
                                if !self.message.is_empty() {
                                    ui.add_space(8.0);
                                    ui.colored_label(theme::BAD, &self.message);
                                }
                            });
                        });
                    ui.add_space(18.0);
                    // ---- recent
                    egui::Frame::new()
                        .fill(egui::Color32::WHITE)
                        .stroke(egui::Stroke::new(1.0, theme::BORDER))
                        .corner_radius(4)
                        .inner_margin(18)
                        .show(ui, |ui| {
                            ui.set_width(width * 0.58 - 60.0);
                            ui.vertical(|ui| {
                                ui.label(
                                    egui::RichText::new("RECENT PROJECTS")
                                        .size(11.5)
                                        .strong()
                                        .color(theme::TEXT_WEAK),
                                );
                                ui.add_space(8.0);
                                if self.recents.paths.is_empty() {
                                    ui.label(
                                        egui::RichText::new(
                                            "Nothing yet - create or open a project.",
                                        )
                                        .color(theme::TEXT_WEAK),
                                    );
                                }
                                let mut open: Option<PathBuf> = None;
                                let mut forget: Option<PathBuf> = None;
                                for p in &self.recents.paths {
                                    let exists = p.exists();
                                    let (rect, resp) = ui.allocate_exact_size(
                                        egui::vec2(ui.available_width(), 38.0),
                                        if exists {
                                            egui::Sense::click()
                                        } else {
                                            egui::Sense::hover()
                                        },
                                    );
                                    if resp.hovered() && exists {
                                        ui.painter().rect_filled(rect, 3.0, theme::ACCENT_SOFT);
                                    }
                                    icons::paint(
                                        ui.painter(),
                                        egui::Rect::from_min_size(
                                            rect.min + egui::vec2(6.0, 5.0),
                                            egui::vec2(28.0, 28.0),
                                        ),
                                        Icon::Body,
                                        exists,
                                    );
                                    let name = p
                                        .file_stem()
                                        .map(|s| s.to_string_lossy().into_owned())
                                        .unwrap_or_default();
                                    ui.painter().text(
                                        rect.min + egui::vec2(44.0, 11.0),
                                        egui::Align2::LEFT_CENTER,
                                        name,
                                        egui::FontId::proportional(13.5),
                                        if exists {
                                            theme::TEXT
                                        } else {
                                            theme::TEXT_WEAK
                                        },
                                    );
                                    let dir = p
                                        .parent()
                                        .map(|d| d.display().to_string())
                                        .unwrap_or_default();
                                    ui.painter().text(
                                        rect.min + egui::vec2(44.0, 27.0),
                                        egui::Align2::LEFT_CENTER,
                                        if exists {
                                            dir
                                        } else {
                                            format!("{dir} - missing")
                                        },
                                        egui::FontId::proportional(10.5),
                                        theme::TEXT_WEAK,
                                    );
                                    if exists && resp.clicked() {
                                        open = Some(p.clone());
                                    }
                                    if !exists {
                                        let r = egui::Rect::from_min_size(
                                            rect.right_top() + egui::vec2(-60.0, 8.0),
                                            egui::vec2(54.0, 22.0),
                                        );
                                        if ui.put(r, egui::Button::new("Forget")).clicked() {
                                            forget = Some(p.clone());
                                        }
                                    }
                                }
                                if let Some(p) = forget {
                                    self.recents.remove(&p);
                                    if let Some(f) = &self.recents_file {
                                        let _ = self.recents.save(f);
                                    }
                                }
                                if let Some(p) = open {
                                    let _ = self.open_path(&p);
                                }
                            });
                        });
                });
            });
    }

    fn project_ui(&mut self, ui: &mut egui::Ui, rs: Option<&eframe::egui_wgpu::RenderState>) {
        // ---- keyboard shortcuts
        let (save, undo, redo) = ui.ctx().input(|i| {
            let ctrl = i.modifiers.command;
            (
                ctrl && i.key_pressed(egui::Key::S),
                ctrl && i.key_pressed(egui::Key::Z) && !i.modifiers.shift,
                ctrl && (i.key_pressed(egui::Key::Y)
                    || (i.key_pressed(egui::Key::Z) && i.modifiers.shift)),
            )
        });
        if save {
            self.save();
        }
        let sketching = self.design.mode == crate::design::Mode::Sketching
            && self.workspace() == Some(Workspace::Design);
        if (undo || redo) && !sketching {
            self.undo_redo(undo);
        }
        // ---- application bar
        let frame = egui::Frame::new()
            .fill(theme::CHROME)
            .inner_margin(egui::Margin::symmetric(6, 3));
        egui::Panel::top("app_bar").frame(frame).show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                if rb::small(ui, Icon::Home, "Back to the welcome page", false).clicked() {
                    self.request(Pending::CloseProject);
                }
                ui.menu_button("File", |ui| {
                    if ui.button("Save   Ctrl+S").clicked() {
                        self.save();
                        ui.close();
                    }
                    if ui.button("Save as…").clicked() {
                        self.save_as();
                        ui.close();
                    }
                    ui.separator();
                    if self.workspace().is_some() {
                        if ui.button("Import STEP / IGES…").clicked() {
                            self.import_file_dialog();
                            ui.close();
                        }
                        ui.menu_button("Export", |ui| {
                            for f in crate::exchange::ExportFormat::ALL {
                                if ui.button(f.label()).clicked() {
                                    self.export_file_dialog(f);
                                    ui.close();
                                }
                            }
                        });
                        ui.separator();
                    }
                    if ui.button("Open project…").clicked() {
                        if let Some(p) = self.dialogs.open_project() {
                            self.request(Pending::Open(p));
                        }
                        ui.close();
                    }
                    if ui.button("Close project").clicked() {
                        self.request(Pending::CloseProject);
                        ui.close();
                    }
                });
                let on_assembly = self.undo_is_for_assembly();
                let on_drawing = self.undo_is_for_drawing();
                let (can_undo, can_redo) = self
                    .project()
                    .map(|p| {
                        if on_drawing {
                            (p.dw_can_undo(), p.dw_can_redo())
                        } else if on_assembly {
                            (p.asm_can_undo(), p.asm_can_redo())
                        } else {
                            (p.history.can_undo(), p.history.can_redo())
                        }
                    })
                    .unwrap_or((false, false));
                let tip = if on_assembly {
                    "the assembly view: undoes changes to components, joints and motion"
                } else {
                    "Ctrl+Z"
                };
                if ui
                    .add_enabled(can_undo, egui::Button::new("Undo"))
                    .on_hover_text(tip)
                    .clicked()
                {
                    self.undo_redo(true);
                }
                if ui
                    .add_enabled(can_redo, egui::Button::new("Redo"))
                    .on_hover_text(if on_assembly { tip } else { "Ctrl+Y" })
                    .clicked()
                {
                    self.undo_redo(false);
                }
                if ui.button("Save").on_hover_text("Ctrl+S").clicked() {
                    self.save();
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let Some(p) = self.project() {
                        let dirty = if p.is_dirty() { "*" } else { "" };
                        ui.label(
                            egui::RichText::new(format!("{}{}", p.name(), dirty))
                                .strong()
                                .color(theme::TEXT),
                        );
                        icons::paint(
                            ui.painter(),
                            egui::Rect::from_center_size(
                                ui.cursor().right_center() + egui::vec2(-18.0, 0.0),
                                egui::vec2(16.0, 16.0),
                            ),
                            Icon::Body,
                            true,
                        );
                    }
                });
            });
        });
        // ---- ribbon
        let current = self.workspace().unwrap_or(Workspace::Design);
        let mut picked: Option<Workspace> = None;
        let ribbon_frame = egui::Frame::new()
            .fill(theme::CHROME)
            .stroke(egui::Stroke::new(1.0, theme::BORDER))
            .inner_margin(egui::Margin::symmetric(8, 4));
        egui::Panel::top("ribbon")
            .frame(ribbon_frame)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    picked = workspace_menu(ui, current);
                    ui.add_space(6.0);
                    ui.vertical(|ui| match (&mut self.screen, current) {
                        (Screen::Project { project, .. }, Workspace::Design) => {
                            self.design.ribbon(ui, project)
                        }
                        (_, Workspace::Cam) => self.cam.ribbon(ui),
                        (_, Workspace::Fea) => self.fea.ribbon(ui),
                        (_, Workspace::Ecad) => self.ecad.ribbon(ui),
                        (_, Workspace::Render) => self.render.ribbon(ui),
                        (Screen::Project { project, .. }, Workspace::Drawing) => {
                            self.drawing.ribbon(ui, project)
                        }
                        (_, Workspace::Design | Workspace::Drawing) => {}
                    });
                });
            });
        if let Some(w) = picked {
            self.set_workspace(w);
        }
        // ---- status
        egui::Panel::bottom("app_status")
            .frame(
                egui::Frame::new()
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(8, 2)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let path = self
                        .project()
                        .and_then(|p| p.path.as_ref())
                        .map(|p| p.display().to_string())
                        .unwrap_or_else(|| "not saved yet".into());
                    ui.label(egui::RichText::new(path).size(11.0).color(theme::TEXT_WEAK));
                    if !self.message.is_empty() {
                        ui.separator();
                        ui.label(egui::RichText::new(&self.message).size(11.0));
                    }
                    if let Some(msg) = self.design.status_line() {
                        ui.separator();
                        ui.label(egui::RichText::new(msg).size(11.0));
                    }
                });
            });
        // ---- the workspace
        let Screen::Project { project, .. } = &mut self.screen else {
            return;
        };
        match current {
            Workspace::Design => {
                egui::CentralPanel::default()
                    .frame(egui::Frame::NONE)
                    .show(ui, |ui| self.design.show(ui, project, rs));
                self.service_design_requests();
            }
            Workspace::Cam => {
                let (cam, dialogs) = (&mut self.cam, &mut self.dialogs);
                egui::CentralPanel::default()
                    .show(ui, |ui| cam.show(ui, project, rs, dialogs.as_mut()));
            }
            Workspace::Render => {
                let (render, dialogs) = (&mut self.render, &mut self.dialogs);
                egui::CentralPanel::default()
                    .show(ui, |ui| render.show(ui, project, dialogs.as_mut()));
            }
            Workspace::Fea => {
                egui::CentralPanel::default().show(ui, |ui| self.fea.show(ui, project, rs));
            }
            Workspace::Ecad => {
                let (ecad, dialogs) = (&mut self.ecad, &mut self.dialogs);
                egui::CentralPanel::default()
                    .show(ui, |ui| ecad.show(ui, project, dialogs.as_mut()));
            }
            Workspace::Drawing => {
                let (drawing, dialogs) = (&mut self.drawing, &mut self.dialogs);
                egui::CentralPanel::default()
                    .frame(egui::Frame::NONE)
                    .show(ui, |ui| drawing.show(ui, project, dialogs.as_mut()));
            }
        }
    }

    fn pending_ui(&mut self, ctx: &egui::Context) {
        let Some(what) = self.pending.clone() else {
            return;
        };
        let name = self.project().map(|p| p.name()).unwrap_or_default();
        let action = match what {
            Pending::CloseProject => "close the project",
            Pending::Open(_) => "open another project",
            Pending::OpenDemo => "open the demo",
            Pending::Quit => "quit",
        };
        egui::Window::new("Unsaved changes")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.label(format!(
                    "“{name}” has changes that are not saved. Save them before you {action}?"
                ));
                ui.horizontal(|ui| {
                    if ui.button("Save").clicked() {
                        self.resolve_pending(true);
                    }
                    if ui.button("Don't save").clicked() {
                        self.resolve_pending(false);
                    }
                    if ui.button("Cancel").clicked() {
                        self.cancel_pending();
                    }
                });
            });
    }
}

/// The workspace drop-down at the left of the ribbon ("DESIGN v"). Returns the workspace picked.
fn workspace_menu(ui: &mut egui::Ui, current: Workspace) -> Option<Workspace> {
    let mut picked = None;
    ui.scope(|ui| {
        ui.spacing_mut().button_padding = egui::vec2(10.0, 18.0);
        ui.visuals_mut().widgets.inactive.weak_bg_fill = egui::Color32::WHITE;
        ui.menu_button(
            egui::RichText::new(format!("{} v", current.label()))
                .size(12.5)
                .strong(),
            |ui| {
                ui.set_min_width(190.0);
                for w in Workspace::ALL {
                    ui.horizontal(|ui| {
                        icons::paint(
                            ui.painter(),
                            egui::Rect::from_min_size(
                                ui.cursor().min + egui::vec2(0.0, 2.0),
                                egui::vec2(18.0, 18.0),
                            ),
                            w.icon(),
                            true,
                        );
                        ui.add_space(22.0);
                        if ui
                            .selectable_label(w == current, w.label())
                            .on_hover_text(w.blurb())
                            .clicked()
                        {
                            picked = Some(w);
                            ui.close();
                        }
                    });
                }
            },
        );
    });
    picked
}

impl eframe::App for KerfwrightApp {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        if ui.ctx().input(|i| i.viewport().close_requested()) && !self.close_requested() {
            ui.ctx()
                .send_viewport_cmd(egui::ViewportCommand::CancelClose);
        }
        self.show_with(ui, frame.wgpu_render_state());
    }
}
