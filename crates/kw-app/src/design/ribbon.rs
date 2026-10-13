//! The Design ribbon, laid out like Fusion 360's: icon buttons for the common commands and, under every group
//! title, a dropdown (`CREATE ▾`) listing all of the group's commands under category headings. The SOLID tab
//! is always there; the SKETCH tab appears while a sketch is being drawn.

use super::assembly::AsmAction;
use super::tools::Tool;
use super::{Action, DesignWorkspace, Mode};
use crate::chrome::ribbon::{Item, Prim, Section};
use crate::chrome::{ribbon as rb, Icon};
use crate::project::Project;
use eframe::egui;
use kw_feature::{BodyOp, FeatureSpec};
use kw_sketch::editor::{ConstraintTool, Tool as SkTool};

/// The constraint commands: id, tool, icon, name.
pub const CONSTRAINTS: [(&str, ConstraintTool, Icon, &str); 13] = [
    (
        "c_coincident",
        ConstraintTool::Coincident,
        Icon::CCoincident,
        "Coincident",
    ),
    (
        "c_horizontal",
        ConstraintTool::Horizontal,
        Icon::CHorizontal,
        "Horizontal",
    ),
    (
        "c_vertical",
        ConstraintTool::Vertical,
        Icon::CVertical,
        "Vertical",
    ),
    (
        "c_parallel",
        ConstraintTool::Parallel,
        Icon::CParallel,
        "Parallel",
    ),
    (
        "c_perpendicular",
        ConstraintTool::Perpendicular,
        Icon::CPerpendicular,
        "Perpendicular",
    ),
    (
        "c_tangent",
        ConstraintTool::Tangent,
        Icon::CTangent,
        "Tangent",
    ),
    ("c_equal", ConstraintTool::Equal, Icon::CEqual, "Equal"),
    ("c_fix", ConstraintTool::Fix, Icon::CFix, "Fix"),
    (
        "c_midpoint",
        ConstraintTool::Midpoint,
        Icon::CMidpoint,
        "Midpoint",
    ),
    (
        "c_concentric",
        ConstraintTool::Concentric,
        Icon::CConcentric,
        "Concentric",
    ),
    (
        "c_collinear",
        ConstraintTool::Collinear,
        Icon::CCollinear,
        "Collinear",
    ),
    (
        "c_on_curve",
        ConstraintTool::PointOnCurve,
        Icon::COnCurve,
        "Point on curve",
    ),
    (
        "c_symmetric",
        ConstraintTool::Symmetric,
        Icon::CSymmetric,
        "Symmetric",
    ),
];

/// Everything the ribbon needs to know about the document to enable or grey its commands.
struct Ctx {
    has_sketch: bool,
    sketches: usize,
    has_solid: bool,
    /// Whether there is any body at all, a surface included.
    has_body: bool,
    /// How many of the bodies are surfaces.
    surfaces: usize,
    has_tool_feature: bool,
    /// How many bodies there are (Combine needs two).
    bodies: usize,
    /// How many components the assembly has, and whether one is open for editing.
    components: usize,
    editing: bool,
}

fn it(id: &'static str, icon: Icon, label: &'static str, tip: &str) -> Item {
    Item::new(id, icon, label).tip(tip)
}

impl DesignWorkspace {
    pub fn ribbon(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let sketching = self.mode == Mode::Sketching;
        // while sketching the one tab is SKETCH; otherwise the tabs that work: SOLID, SURFACE, SHEET METAL, FORM, TOOLS
        let labels: Vec<(&str, bool)> = if sketching {
            vec![("SKETCH", true)]
        } else {
            vec![
                ("SOLID", true),
                ("SURFACE", true),
                ("SHEET METAL", true),
                ("FORM", true),
                ("TOOLS", true),
            ]
        };
        let mut cur = if sketching { 0 } else { self.ribbon_tab };
        rb::tabs(ui, &labels, &mut cur);
        if !sketching {
            self.ribbon_tab = cur;
        }
        self.tab = usize::from(sketching);
        ui.add_space(2.0);
        // a narrow window scrolls the groups sideways instead of cutting them off
        egui::ScrollArea::horizontal()
            .id_salt("ribbon_scroll")
            .auto_shrink([false, true])
            .show(ui, |ui| {
                ui.horizontal(|ui| match (sketching, self.ribbon_tab) {
                    (true, _) => self.sketch_tab(ui, project),
                    (false, 1) => self.surface_tab(ui, project),
                    (false, 2) => self.sheet_tab(ui, project),
                    (false, 3) => self.form_tab(ui, project),
                    (false, 4) => self.tools_tab(ui, project),
                    (false, _) => self.solid_tab(ui, project),
                });
            });
    }

    /// SURFACE: bodies without volume, and how good they are.
    fn surface_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let cx = Self::context(project);
        let dlg = |t: &str| {
            self.dialog
                .as_ref()
                .is_some_and(|d| d.tool.title().starts_with(t))
        };
        let need_sketch = "draw a sketch first";
        let need_body = "there is nothing to analyse yet";
        let mut picked: Vec<&'static str> = Vec::new();
        let extrude = it(
            "surf_extrude",
            Icon::Extrude,
            "Extrude",
            "Pull the curves of a sketch into a surface",
        )
        .active(dlg("EXTRUDE"))
        .enabled_if(cx.has_sketch, need_sketch);
        let revolve = it(
            "surf_revolve",
            Icon::Revolve,
            "Revolve",
            "Spin the curves of a sketch around an axis into a surface",
        )
        .active(dlg("REVOLVE"))
        .enabled_if(cx.has_sketch, need_sketch);
        let sweep = it(
            "surf_sweep",
            Icon::Sweep,
            "Sweep",
            "Carry the curves of a sketch along a path, with a twist or a guide rail",
        )
        .active(dlg("SWEEP"))
        .enabled_if(
            cx.sketches >= 2,
            "it needs a profile sketch and a path sketch",
        );
        let loft = it(
            "surf_loft",
            Icon::Loft,
            "Loft",
            "A surface through two or more sketches, smooth or ruled",
        )
        .active(dlg("LOFT"))
        .enabled_if(cx.sketches >= 2, "it needs at least two profile sketches");
        let patch = it(
            "surf_patch",
            Icon::Plane,
            "Patch",
            "Fill a hole: pick the edges around it, optionally tangent to the surfaces they belong to",
        )
        .active(dlg("PATCH"))
        .enabled_if(cx.has_body, "there are no edges to fill between yet");
        let ruled = it(
            "surf_ruled",
            Icon::Loft,
            "Ruled Surface",
            "Straight lines between two picked edges",
        )
        .active(dlg("RULED"))
        .enabled_if(cx.has_body, "there are no edges to join yet");
        if let Some(id) = rb::menu_group(
            ui,
            "CREATE",
            &[
                extrude.clone().into(),
                revolve.clone().into(),
                sweep.clone().into(),
                loft.clone().into(),
                patch.clone().into(),
                ruled.clone().into(),
            ],
            &[
                Section::new("FROM CURVES", vec![extrude, revolve, sweep, loft]),
                Section::new("FROM EDGES", vec![patch, ruled]),
            ],
        ) {
            picked.push(id);
        }
        let need_surface = "there is no surface yet";
        let stitch = it(
            "surf_stitch",
            Icon::Combine,
            "Stitch",
            "Join surfaces along edges that lie on one another; a closed group becomes a solid",
        )
        .active(dlg("STITCH"))
        .enabled_if(cx.surfaces >= 1, need_surface);
        let unstitch = it(
            "surf_unstitch",
            Icon::Split,
            "Unstitch",
            "Split a body into its faces",
        )
        .active(dlg("UNSTITCH"))
        .enabled_if(cx.has_body, need_body);
        let thicken = it(
            "surf_thicken",
            Icon::Shell,
            "Thicken",
            "Give an open, smooth surface thickness: a solid with walls along its open edges",
        )
        .active(dlg("THICKEN"))
        .enabled_if(cx.surfaces >= 1, need_surface);
        let offset = it(
            "surf_offset",
            Icon::OffsetBody,
            "Offset Surface",
            "Move a smooth surface along its normal",
        )
        .active(dlg("OFFSET SURFACE"))
        .enabled_if(cx.surfaces >= 1, need_surface);
        if let Some(id) = rb::menu_group(
            ui,
            "MODIFY",
            &[
                stitch.clone().into(),
                unstitch.clone().into(),
                thicken.clone().into(),
                offset.clone().into(),
            ],
            &[Section::new(
                "SURFACES",
                vec![stitch, unstitch, thicken, offset],
            )],
        ) {
            picked.push(id);
        }
        let zebra = it(
            "zebra",
            Icon::Zebra,
            "Zebra Analysis",
            "Reflected stripes: they flow across tangent joins and break at creases",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind == super::analysis::AKind::Zebra),
        )
        .enabled_if(cx.has_body, need_body);
        let curvature = it(
            "curvature",
            Icon::Curvature,
            "Curvature Analysis",
            "Colour the surfaces by Gaussian, mean or principal curvature",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind.is_curvature()),
        )
        .enabled_if(cx.has_body, need_body);
        let draft = it(
            "draft_analysis",
            Icon::DraftAnalysis,
            "Draft Analysis",
            "Faces with too little draft for a pull direction",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind == super::analysis::AKind::Draft),
        )
        .enabled_if(cx.has_body, need_body);
        if let Some(id) = rb::menu_group(
            ui,
            "INSPECT",
            &[
                zebra.clone().into(),
                curvature.clone().into(),
                draft.clone().into(),
            ],
            &[Section::new("ANALYSIS", vec![zebra, curvature, draft])],
        ) {
            picked.push(id);
        }
        for id in picked {
            self.run_solid_command(id, &cx);
        }
    }

    /// SHEET METAL: strips and their flat pattern.
    fn sheet_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let cx = Self::context(project);
        let dlg = |t: &str| {
            self.dialog
                .as_ref()
                .is_some_and(|d| d.tool.title().starts_with(t))
        };
        let has_sheet = project
            .document()
            .features
            .iter()
            .any(|f| matches!(f.spec, FeatureSpec::SheetMetal(_)));
        let mut picked: Vec<&'static str> = Vec::new();
        let l = it(
            "sm_l",
            Icon::Flange,
            "L Bracket",
            "A flat with one 90° flange",
        )
        .active(dlg("SHEET"));
        let u = it(
            "sm_u",
            Icon::Bend,
            "U Channel",
            "A flat with two 90° flanges",
        );
        let z = it(
            "sm_z",
            Icon::Bend,
            "Z Bracket",
            "Two flanges bent in opposite directions",
        );
        let custom = it(
            "sm_custom",
            Icon::Flange,
            "Custom Strip",
            "Start from one flat and add flats and bends",
        );
        if let Some(id) = rb::menu_group(
            ui,
            "CREATE",
            &[l.clone().into(), u.clone().into(), z.clone().into()],
            &[Section::new("STRIPS", vec![l, u, z, custom])],
        ) {
            picked.push(id);
        }
        let flat = it(
            "sm_flat",
            Icon::Unfold,
            "Flat Pattern",
            "Unfold the strip (or a bent body) with its bend table; export it as DXF",
        )
        .active(self.flat.is_some())
        .enabled_if(cx.has_solid, "there is no sheet-metal part yet");
        let edit = it(
            "sm_edit",
            Icon::Flange,
            "Edit Strip",
            "Change thickness, K-factor, flats and bends",
        )
        .enabled_if(has_sheet, "there is no sheet-metal feature");
        if let Some(id) = rb::menu_group(
            ui,
            "MODIFY",
            &[flat.clone().into(), edit.clone().into()],
            &[Section::new("FLAT PATTERN", vec![flat, edit])],
        ) {
            picked.push(id);
        }
        for id in picked {
            self.run_solid_command(id, &cx);
        }
    }

    /// FORM: subdivision cages that become bodies.
    fn form_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let cx = Self::context(project);
        let has_form = project
            .document()
            .features
            .iter()
            .any(|f| matches!(f.spec, FeatureSpec::Subd(_)));
        let mut picked: Vec<&'static str> = Vec::new();
        let bx = it(
            "form_box",
            Icon::FormBox,
            "Box",
            "A box cage with sharp edges",
        );
        let cy = it(
            "form_cyl",
            Icon::FormCylinder,
            "Cylinder",
            "A prism cage: sharp caps, smooth sides",
        );
        let sp = it(
            "form_sphere",
            Icon::FormSphere,
            "Sphere",
            "A cube cage that subdivides to a ball",
        );
        let to = it("form_torus", Icon::FormTorus, "Torus", "A ring cage");
        if let Some(id) = rb::menu_group(
            ui,
            "CREATE",
            &[
                bx.clone().into(),
                cy.clone().into(),
                sp.clone().into(),
                to.clone().into(),
            ],
            &[Section::new("PRIMITIVES", vec![bx, cy, sp, to])],
        ) {
            picked.push(id);
        }
        let edit = it(
            "form_edit",
            Icon::FormBox,
            "Edit Form",
            "Move the cage vertices, change creases; the body follows",
        )
        .enabled_if(has_form, "there is no form yet");
        if let Some(id) = rb::menu_group(
            ui,
            "MODIFY",
            &[edit.clone().into()],
            &[Section::new("CAGE", vec![edit])],
        ) {
            picked.push(id);
        }
        for id in picked {
            self.run_solid_command(id, &cx);
        }
    }

    /// TOOLS: inspection, appearance and exchange.
    fn tools_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let cx = Self::context(project);
        let need_solid = "there is no solid yet";
        let mut picked: Vec<&'static str> = Vec::new();
        let measure = it(
            "measure",
            Icon::Measure,
            "Measure",
            "Distances, angles, areas, mass and centre of mass",
        )
        .active(self.measure_open)
        .enabled_if(cx.has_solid, need_solid);
        let interference = it(
            "interference",
            Icon::Interference,
            "Interference",
            "Which bodies overlap, and by how much",
        )
        .active(self.interference_open)
        .enabled_if(cx.bodies >= 2, "it needs two bodies");
        let section = it(
            "section",
            Icon::Section,
            "Section Analysis",
            "Cut the model with a plane to look inside",
        )
        .active(self.section.is_some())
        .enabled_if(cx.has_solid, need_solid);
        let zebra = it(
            "zebra",
            Icon::Zebra,
            "Zebra Analysis",
            "Reflected stripes: they flow across tangent joins and break at creases",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind == super::analysis::AKind::Zebra),
        )
        .enabled_if(cx.has_solid, need_solid);
        let curvature = it(
            "curvature",
            Icon::Curvature,
            "Curvature Analysis",
            "Colour the surfaces by Gaussian, mean or principal curvature",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind.is_curvature()),
        )
        .enabled_if(cx.has_solid, need_solid);
        let draft = it(
            "draft_analysis",
            Icon::DraftAnalysis,
            "Draft Analysis",
            "Faces with too little draft for a pull direction",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind == super::analysis::AKind::Draft),
        )
        .enabled_if(cx.has_solid, need_solid);
        if let Some(id) = rb::menu_group(
            ui,
            "INSPECT",
            &[measure.clone().into(), section.clone().into()],
            &[Section::new(
                "ANALYSIS",
                vec![measure, interference, section, zebra, curvature, draft],
            )],
        ) {
            picked.push(id);
        }
        let appearance = it(
            "appearance",
            Icon::Appearance,
            "Appearance",
            "Name, colour and material of the active body",
        )
        .enabled_if(cx.has_solid, need_solid);
        let params = it(
            "params",
            Icon::Parameters,
            "Parameters",
            "Named values and expressions of the design",
        );
        if let Some(id) = rb::menu_group(
            ui,
            "DESIGN DATA",
            &[appearance.clone().into(), params.clone().into()],
            &[Section::new("LOOK AND VALUES", vec![appearance, params])],
        ) {
            picked.push(id);
        }
        let import = it(
            "file_import",
            Icon::Folder,
            "Import",
            "Add the solids of a STEP or IGES file",
        );
        let exports: Vec<Item> = crate::exchange::ExportFormat::ALL
            .iter()
            .map(|f| {
                it(
                    match f {
                        crate::exchange::ExportFormat::Step => "file_export_step",
                        crate::exchange::ExportFormat::Stl => "file_export_stl",
                        crate::exchange::ExportFormat::Obj => "file_export_obj",
                        crate::exchange::ExportFormat::Glb => "file_export_glb",
                    },
                    Icon::Folder,
                    f.label(),
                    "Write the model to a file",
                )
                .enabled_if(cx.has_solid, need_solid)
            })
            .collect();
        let mut sections = vec![Section::new("IMPORT", vec![import.clone()])];
        sections.push(Section::new("EXPORT", exports));
        if let Some(id) = rb::menu_group(ui, "EXCHANGE", &[import.into()], &sections) {
            picked.push(id);
        }
        for id in picked {
            self.run_solid_command(id, &cx);
        }
    }

    fn context(project: &mut Project) -> Ctx {
        let doc = project.document();
        let sketches = doc
            .features
            .iter()
            .filter(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
            .count();
        Ctx {
            has_sketch: sketches > 0,
            sketches,
            has_solid: project.evaluation().final_solid().is_some(),
            has_body: project.evaluation().final_body().is_some(),
            surfaces: {
                let ev = project.evaluation();
                doc.heads()
                    .iter()
                    .filter(|h| ev.output(**h).is_some_and(|o| o.is_sheet()))
                    .count()
            },
            bodies: doc.heads().len(),
            components: project.assembly.as_ref().map_or(0, |a| a.instances.len()),
            editing: project.editing().is_some(),
            has_tool_feature: doc.features.iter().any(|f| {
                matches!(
                    f.spec,
                    FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
                )
            }),
        }
    }

    fn solid_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let cx = Self::context(project);
        let dlg = |t: &str| {
            self.dialog
                .as_ref()
                .is_some_and(|d| d.tool.title().starts_with(t))
        };
        let need_sketch = "draw a sketch first";
        let need_solid = "there is no solid yet";
        let need_feature = "there is no extrude, revolve or hole to repeat";
        let mut picked: Vec<&'static str> = Vec::new();

        // ------------------------------------------------------------------ CREATE
        let create_sketch = it(
            "create_sketch",
            Icon::Sketch,
            "Create Sketch",
            "Pick a plane or a planar face, then sketch on it",
        )
        .active(self.mode == Mode::PickPlane);
        let extrude = it(
            "extrude",
            Icon::Extrude,
            "Extrude",
            "Pull a sketch profile into a solid",
        )
        .key("E")
        .active(dlg("EXTRUDE"))
        .enabled_if(cx.has_sketch, need_sketch);
        let revolve = it(
            "revolve",
            Icon::Revolve,
            "Revolve",
            "Spin a sketch profile around its axis",
        )
        .active(dlg("REVOLVE"))
        .enabled_if(cx.has_sketch, need_sketch);
        let sweep = it(
            "sweep",
            Icon::Sweep,
            "Sweep",
            "Sweep a profile sketch along a path sketch",
        )
        .active(dlg("SWEEP"))
        .enabled_if(
            cx.sketches >= 2,
            "it needs a profile sketch and a path sketch",
        );
        let loft = it(
            "loft",
            Icon::Loft,
            "Loft",
            "Blend between two or more profile sketches",
        )
        .active(dlg("LOFT"))
        .enabled_if(cx.sketches >= 2, "it needs at least two profile sketches");
        let rib = it(
            "rib",
            Icon::Rib,
            "Rib",
            "Thicken the lines and arcs of a sketch into walls",
        )
        .active(dlg("RIB"))
        .enabled_if(cx.has_sketch, need_sketch);
        let pipe = it("pipe", Icon::Pipe, "Pipe", "A tube along a path sketch")
            .active(dlg("PIPE"))
            .enabled_if(cx.has_sketch, need_sketch);
        let hole = it(
            "hole",
            Icon::Hole,
            "Hole",
            "Drill a hole into a planar face",
        )
        .active(dlg("HOLE"))
        .enabled_if(cx.has_solid, need_solid);
        let hole_tapped = it(
            "hole_tapped",
            Icon::Hole,
            "Tapped Hole",
            "A hole drilled to the tap-drill size with a threaded section (M, UNC, UNF)",
        )
        .active(dlg("HOLE"))
        .enabled_if(cx.has_solid, need_solid);
        let thread = it(
            "thread",
            Icon::Thread,
            "Thread",
            "Cosmetic thread on a hole wall or a pin (click the cylindrical face)",
        )
        .active(dlg("THREAD"))
        .enabled_if(cx.has_solid, need_solid);
        let rect_pat = it(
            "rect_pattern",
            Icon::Pattern,
            "Rectangular Pattern",
            "Repeat a feature along a direction",
        )
        .active(dlg("RECTANGULAR"))
        .enabled_if(cx.has_tool_feature, need_feature);
        let circ_pat = it(
            "circ_pattern",
            Icon::CircularPattern,
            "Circular Pattern",
            "Repeat a feature around an axis",
        )
        .active(dlg("CIRCULAR"))
        .enabled_if(cx.has_tool_feature, need_feature);
        let mirror = it(
            "mirror",
            Icon::Mirror,
            "Mirror",
            "Mirror a feature across a plane",
        )
        .active(dlg("MIRROR"))
        .enabled_if(cx.has_tool_feature, need_feature);
        if let Some(id) = rb::menu_group(
            ui,
            "CREATE",
            &[
                create_sketch.clone().into(),
                extrude.clone().into(),
                revolve.clone().into(),
                sweep.clone().into(),
                loft.clone().into(),
                hole.clone().into(),
            ],
            &[
                Section::new(
                    "SKETCH-BASED",
                    vec![create_sketch, extrude, revolve, sweep, loft],
                ),
                Section::new("FEATURES", vec![hole, hole_tapped, thread, rib, pipe]),
                Section::new("PATTERN", vec![rect_pat, circ_pat, mirror]),
            ],
        ) {
            picked.push(id);
        }

        // ------------------------------------------------------------------ MODIFY
        let fillet = it(
            "fillet",
            Icon::Fillet,
            "Fillet",
            "Round edges (click edges in the view)",
        )
        .active(dlg("FILLET"))
        .enabled_if(cx.has_solid, need_solid);
        let var_fillet = it(
            "var_fillet",
            Icon::VarFillet,
            "Variable Fillet",
            "Fillet whose radius changes along one edge",
        )
        .active(dlg("VARIABLE"))
        .enabled_if(cx.has_solid, need_solid);
        let chamfer = it("chamfer", Icon::Chamfer, "Chamfer", "Bevel edges")
            .active(dlg("CHAMFER"))
            .enabled_if(cx.has_solid, need_solid);
        let shell = it(
            "shell",
            Icon::Shell,
            "Shell",
            "Hollow the solid, removing the faces you pick",
        )
        .active(dlg("SHELL"))
        .enabled_if(cx.has_solid, need_solid);
        let draft = it(
            "draft",
            Icon::Draft,
            "Draft",
            "Tilt faces for moulding or casting (click faces)",
        )
        .active(dlg("DRAFT"))
        .enabled_if(cx.has_solid, need_solid);
        let offset_body = it(
            "offset_body",
            Icon::OffsetBody,
            "Offset Body",
            "Grow or shrink the whole body",
        )
        .active(dlg("OFFSET"))
        .enabled_if(cx.has_solid, need_solid);
        let combine = it(
            "combine",
            Icon::Combine,
            "Combine",
            "Join, cut or intersect two tool bodies",
        )
        .active(dlg("COMBINE"))
        .enabled_if(cx.has_tool_feature, need_feature);
        let params = it(
            "parameters",
            Icon::Parameters,
            "Change Parameters",
            "Named values that features can use in expressions",
        )
        .active(self.params_open);
        if let Some(id) = rb::menu_group(
            ui,
            "MODIFY",
            &[
                fillet.clone().into(),
                chamfer.clone().into(),
                shell.clone().into(),
                combine.clone().into(),
                params.clone().into(),
            ],
            &[
                Section::new(
                    "EDGES AND FACES",
                    vec![fillet, var_fillet, chamfer, shell, draft],
                ),
                Section::new("BODIES", vec![offset_body, combine]),
                Section::new("PARAMETERS", vec![params]),
            ],
        ) {
            picked.push(id);
        }

        // --------------------------------------------------------------- CONSTRUCT
        let any_plane = self.plane_visible.iter().any(|v| *v);
        let plane = it(
            "plane",
            Icon::Plane,
            "Plane",
            "Construction plane: an origin plane moved and tilted",
        )
        .active(dlg("CONSTRUCTION"));
        let origin_planes = it(
            "origin_planes",
            Icon::Plane,
            "Origin Planes",
            "Show or hide the origin planes",
        )
        .active(any_plane);
        let plane_mid = it(
            "plane_mid",
            Icon::PlaneMid,
            "Midplane",
            "Plane halfway between the XY/YZ/XZ plane and another plane",
        )
        .active(dlg("CONSTRUCTION"));
        let plane_3pt = it(
            "plane_3pt",
            Icon::Plane3Pt,
            "Through 3 Points",
            "Plane through three points or construction points",
        );
        let axis_2pt = it(
            "axis_2pt",
            Icon::Axis,
            "Axis Through 2 Points",
            "Construction axis through two points",
        );
        let axis_edge = it(
            "axis_edge",
            Icon::Axis,
            "Axis Along Edge",
            "Construction axis along a straight edge",
        );
        let axis_face = it(
            "axis_face",
            Icon::Axis,
            "Axis Normal to Face",
            "Construction axis perpendicular to a planar face",
        );
        let axis_planes = it(
            "axis_planes",
            Icon::Axis,
            "Axis at 2 Planes",
            "Construction axis where two planes meet",
        );
        let point_xyz = it(
            "point_xyz",
            Icon::PointDatum,
            "Point at Coordinates",
            "Construction point at x, y, z",
        );
        let point_edge = it(
            "point_edge",
            Icon::PointDatum,
            "Point at Edge Middle",
            "Construction point at the middle of an edge",
        );
        let point_axis = it(
            "point_axis",
            Icon::PointDatum,
            "Point at Axis and Plane",
            "Construction point where an axis crosses a plane",
        );
        if let Some(id) = rb::menu_group(
            ui,
            "CONSTRUCT",
            &[
                plane.clone().into(),
                Item {
                    label: "Axis",
                    ..axis_2pt.clone()
                }
                .into(),
                Item {
                    label: "Point",
                    ..point_xyz.clone()
                }
                .into(),
            ],
            &[
                Section::new("PLANE", vec![plane, plane_mid, plane_3pt, origin_planes]),
                Section::new("AXIS", vec![axis_2pt, axis_edge, axis_face, axis_planes]),
                Section::new("POINT", vec![point_xyz, point_edge, point_axis]),
            ],
        ) {
            picked.push(id);
        }

        // ----------------------------------------------------------------- INSPECT
        let measure = it(
            "measure",
            Icon::Measure,
            "Measure",
            "Distances, angles, areas, mass and centre of mass",
        )
        .active(self.measure_open)
        .enabled_if(cx.has_solid, need_solid);
        let interference = it(
            "interference",
            Icon::Interference,
            "Interference",
            "Which bodies overlap, and by how much",
        )
        .active(self.interference_open)
        .enabled_if(cx.bodies >= 2, "it needs two bodies");
        let section = it(
            "section",
            Icon::Section,
            "Section Analysis",
            "Cut the model with a plane to look inside",
        )
        .active(self.section.is_some())
        .enabled_if(cx.has_solid, need_solid);
        let zebra = it(
            "zebra",
            Icon::Zebra,
            "Zebra Analysis",
            "Reflected stripes: they flow across tangent joins and break at creases",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind == super::analysis::AKind::Zebra),
        )
        .enabled_if(cx.has_solid, need_solid);
        let curvature = it(
            "curvature",
            Icon::Curvature,
            "Curvature Analysis",
            "Colour the surfaces by Gaussian, mean or principal curvature",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind.is_curvature()),
        )
        .enabled_if(cx.has_solid, need_solid);
        let draft = it(
            "draft_analysis",
            Icon::DraftAnalysis,
            "Draft Analysis",
            "Faces with too little draft for a pull direction",
        )
        .active(
            self.analysis
                .as_ref()
                .is_some_and(|a| a.kind == super::analysis::AKind::Draft),
        )
        .enabled_if(cx.has_solid, need_solid);
        if let Some(id) = rb::menu_group(
            ui,
            "INSPECT",
            &[
                measure.clone().into(),
                interference.clone().into(),
                section.clone().into(),
            ],
            &[Section::new(
                "ANALYSIS",
                vec![measure, interference, section, zebra, curvature, draft],
            )],
        ) {
            picked.push(id);
        }

        // ---------------------------------------------------------------- ASSEMBLE
        let need_two = "the assembly needs two components";
        let asm_new = it(
            "asm_new",
            Icon::Component,
            "New Component",
            "A new, empty component, opened for editing",
        );
        let asm_body = it(
            "asm_body",
            Icon::Body,
            "Component from Body",
            "Make the active body (or the only one) a component",
        )
        .enabled_if(
            cx.bodies > 0 && !cx.editing,
            "there is no body to make a component of",
        );
        let asm_design = it(
            "asm_design",
            Icon::Component,
            "Component from Design",
            "Make the whole design a component of the assembly",
        )
        .enabled_if(cx.has_solid && !cx.editing, "there is no solid yet");
        let asm_insert = it(
            "asm_insert",
            Icon::Folder,
            "Insert Component",
            "Insert the design of another .kwf file as a component",
        );
        let asm_activate = if cx.editing {
            it(
                "asm_finish",
                Icon::Finish,
                "Finish Component",
                "Close the component and return to the assembly",
            )
        } else {
            it(
                "asm_activate",
                Icon::Select,
                "Activate Component",
                "Edit the selected component in place",
            )
            .enabled_if(self.asm.selected.len() == 1, "select one component first")
        };
        let asm_joint = it(
            "asm_joint",
            Icon::Joint,
            "Joint",
            "Click a point on each of two components: rigid, revolute, slider, cylindrical, planar or ball",
        )
        .enabled_if(cx.components >= 2, need_two)
        .active(self.asm.dialog.as_ref().is_some_and(|d| !d.as_built));
        let asm_as_built = it(
            "asm_as_built",
            Icon::AsBuiltJoint,
            "As-Built Joint",
            "Join two components where they are now, without moving them",
        )
        .enabled_if(cx.components >= 2, need_two)
        .active(self.asm.dialog.as_ref().is_some_and(|d| d.as_built));
        let asm_rigid = it(
            "asm_rigid",
            Icon::RigidGroup,
            "Rigid Group",
            "Make the chosen components move as one",
        )
        .enabled_if(cx.components >= 2, need_two);
        let asm_ground = it(
            "asm_ground",
            Icon::Ground,
            "Ground",
            "Ground or release the selected components",
        )
        .enabled_if(!self.asm.selected.is_empty(), "select a component first");
        let asm_move = it(
            "asm_move",
            Icon::Move,
            "Move Component",
            "Type a shift and a turn for the selected free component (or drag it in the view)",
        )
        .enabled_if(!self.asm.selected.is_empty(), "select a component first");
        let asm_view = it(
            "asm_view",
            Icon::AssemblyView,
            "Assembly View",
            "Show the components at their solved positions",
        )
        .enabled_if(cx.components > 0, "there are no components yet")
        .active(self.asm.view);
        let asm_motion = it(
            "asm_motion",
            Icon::Motion,
            "Drive Joints",
            "Move the joints with sliders and watch the assembly",
        )
        .enabled_if(cx.components > 0, "there are no components yet")
        .active(self.asm.open);
        let asm_check = it(
            "asm_check",
            Icon::Interference,
            "Interference",
            "Find components that overlap and show the overlap",
        )
        .enabled_if(cx.components >= 2, need_two);
        let asm_bom = it(
            "asm_bom",
            Icon::Bom,
            "Bill of Materials",
            "Parts, how many of each, volume, material and mass",
        )
        .enabled_if(cx.components > 0, "there are no components yet");
        if let Some(id) = rb::menu_group(
            ui,
            "ASSEMBLE",
            &[
                asm_new.clone().into(),
                asm_joint.clone().into(),
                asm_as_built.clone().into(),
            ],
            &[
                Section::new(
                    "COMPONENT",
                    vec![asm_new, asm_body, asm_design, asm_insert, asm_activate],
                ),
                Section::new(
                    "JOINTS",
                    vec![asm_joint, asm_as_built, asm_rigid, asm_ground],
                ),
                Section::new("MOTION", vec![asm_view, asm_move, asm_motion]),
                Section::new("INSPECT", vec![asm_check, asm_bom]),
            ],
        ) {
            picked.push(id);
        }

        // ------------------------------------------------------------------ SELECT
        let select = it(
            "select",
            Icon::Select,
            "Select",
            "Click faces and edges in the view",
        )
        .active(self.mode == Mode::Idle && self.dialog.is_none() && !self.measure_open);
        if let Some(id) = rb::menu_group(ui, "SELECT", &[select.into()], &[]) {
            picked.push(id);
        }

        for id in picked {
            self.run_solid_command(id, &cx);
        }
    }

    /// What a SOLID-tab command does.
    fn run_solid_command(&mut self, id: &str, cx: &Ctx) {
        let join = if cx.has_solid {
            BodyOp::Join
        } else {
            BodyOp::New
        };
        let tool = match id {
            "create_sketch" => {
                self.actions.push(Action::StartSketch);
                return;
            }
            "extrude" => Tool::extrude(None, join),
            "surf_extrude" => {
                let mut t = Tool::extrude(None, BodyOp::New);
                if let Tool::Extrude { more, .. } = &mut t {
                    more.surface = true;
                }
                t
            }
            "surf_revolve" => Tool::Revolve {
                sketch: None,
                angle: "360 deg".into(),
                axis_y: false,
                axis_edge: false,
                axis_datum: None,
                op: BodyOp::New,
                surface: true,
            },
            "surf_sweep" => Tool::Sweep {
                profile: None,
                path: None,
                op: BodyOp::New,
                surface: true,
                twist: String::new(),
                guide: None,
            },
            "surf_stitch" => Tool::Stitch {
                tolerance: "0.01 mm".into(),
            },
            "surf_unstitch" => Tool::Unstitch,
            "surf_thicken" => Tool::Thicken {
                thickness: "1 mm".into(),
                side: 0,
            },
            "surf_offset" => Tool::OffsetSurface {
                distance: "1 mm".into(),
            },
            "surf_patch" => Tool::Patch { tangent: false },
            "surf_ruled" => Tool::Ruled,
            "surf_loft" => Tool::Loft {
                sketches: vec![None, None],
                op: BodyOp::New,
                surface: true,
                ruled: false,
            },
            "revolve" => Tool::Revolve {
                sketch: None,
                angle: "360 deg".into(),
                axis_y: false,
                axis_edge: false,
                axis_datum: None,
                op: join,
                surface: false,
            },
            "rib" => Tool::Rib {
                sketch: None,
                thickness: "2 mm".into(),
                depth: "10 mm".into(),
                symmetric: false,
                flip: false,
                op: join,
            },
            "pipe" => Tool::Pipe {
                path: None,
                diameter: "10 mm".into(),
                wall: "1 mm".into(),
                op: join,
            },
            "sweep" => Tool::Sweep {
                profile: None,
                path: None,
                op: join,
                surface: false,
                twist: String::new(),
                guide: None,
            },
            "loft" => Tool::Loft {
                sketches: vec![None, None],
                op: join,
                surface: false,
                ruled: false,
            },
            "hole" => Tool::default_hole(),
            "hole_tapped" => match Tool::default_hole() {
                Tool::Hole {
                    x,
                    y,
                    diameter,
                    depth,
                    cb_diameter,
                    cb_depth,
                    cs_diameter,
                    cs_angle,
                    through_all,
                    thread,
                    thread_length,
                    ..
                } => Tool::Hole {
                    x,
                    y,
                    diameter,
                    depth,
                    kind: 3,
                    cb_diameter,
                    cb_depth,
                    cs_diameter,
                    cs_angle,
                    through_all,
                    thread,
                    thread_length,
                },
                t => t,
            },
            "thread" => Tool::Thread {
                designation: "M6".into(),
                length: "8 mm".into(),
                internal: true,
            },
            "rect_pattern" => Tool::Pattern {
                source: None,
                dx: "20 mm".into(),
                dy: "0 mm".into(),
                dz: "0 mm".into(),
                count: "3".into(),
            },
            "circ_pattern" => Tool::CircularPattern {
                source: None,
                count: "6".into(),
                angle: "360 deg".into(),
                axis: 2,
                datum: None,
            },
            "mirror" => Tool::Mirror {
                source: None,
                plane: 1,
                datum: None,
            },
            "fillet" => Tool::Fillet {
                radius: "2 mm".into(),
            },
            "var_fillet" => Tool::VariableFillet {
                start: "1 mm".into(),
                end: "3 mm".into(),
            },
            "chamfer" => Tool::Chamfer {
                d1: "1 mm".into(),
                d2: "1 mm".into(),
            },
            "shell" => Tool::Shell {
                thickness: "2 mm".into(),
            },
            "draft" => Tool::Draft {
                angle: "3 deg".into(),
                axis: 2,
                position: "0 mm".into(),
            },
            "offset_body" => Tool::Offset {
                distance: "1 mm".into(),
            },
            "combine" => Tool::Combine {
                target: None,
                tool: None,
                op: BodyOp::Join,
                keep_tool: false,
            },
            "move" | "copy" => Tool::MoveBody {
                copy: id == "copy",
                dx: if id == "copy" { "30 mm" } else { "10 mm" }.into(),
                dy: "0 mm".into(),
                dz: "0 mm".into(),
                axis: 2,
                angle: "0 deg".into(),
            },
            "scale" => Tool::ScaleBody {
                factor: "1.5".into(),
                cx: "0 mm".into(),
                cy: "0 mm".into(),
                cz: "0 mm".into(),
            },
            "split" => Tool::SplitBody {
                plane: 1,
                offset: "0 mm".into(),
            },
            "plane" | "plane_mid" | "plane_3pt" => Tool::ConstructionPlane {
                base: 0,
                offset: if id == "plane_mid" { "0 mm" } else { "10 mm" }.into(),
                tilt: "0 deg".into(),
                more: super::tools::PlaneMore {
                    mode: match id {
                        "plane_mid" => 1,
                        "plane_3pt" => 2,
                        _ => 0,
                    },
                    other: super::tools::PlaneInput {
                        offset: "20 mm".into(),
                        ..Default::default()
                    },
                    pts: [
                        super::tools::PointInput::default(),
                        super::tools::PointInput {
                            x: "10 mm".into(),
                            ..Default::default()
                        },
                        super::tools::PointInput {
                            y: "10 mm".into(),
                            ..Default::default()
                        },
                    ],
                },
            },
            "axis_2pt" | "axis_edge" | "axis_face" | "axis_planes" => Tool::Axis {
                mode: match id {
                    "axis_2pt" => 0,
                    "axis_edge" => 1,
                    "axis_face" => 2,
                    _ => 3,
                },
                pts: [
                    super::tools::PointInput::default(),
                    super::tools::PointInput {
                        z: "10 mm".into(),
                        ..Default::default()
                    },
                ],
                at_u: "0 mm".into(),
                at_v: "0 mm".into(),
                planes: [
                    super::tools::PlaneInput {
                        which: 1,
                        ..Default::default()
                    },
                    super::tools::PlaneInput {
                        which: 2,
                        ..Default::default()
                    },
                ],
            },
            "point_xyz" | "point_edge" | "point_axis" => Tool::DatumPoint {
                mode: match id {
                    "point_xyz" => 0,
                    "point_edge" => 1,
                    _ => 2,
                },
                coords: super::tools::PointInput::default(),
                axis: None,
                plane: super::tools::PlaneInput::default(),
            },
            "parameters" => {
                self.params_open = !self.params_open;
                return;
            }
            "origin_planes" => {
                let any = self.plane_visible.iter().any(|v| *v);
                self.plane_visible = [!any; 3];
                return;
            }
            "measure" => {
                self.measure_open = !self.measure_open;
                self.model.picks.clear();
                return;
            }
            "interference" => {
                self.interference_open = !self.interference_open;
                return;
            }
            "sm_l" | "sm_u" | "sm_z" | "sm_custom" => {
                self.actions
                    .push(Action::Open(super::sheetmetal::preset(match id {
                        "sm_l" => "l",
                        "sm_u" => "u",
                        "sm_z" => "z",
                        _ => "",
                    })));
                return;
            }
            "sm_flat" => {
                self.actions.push(Action::FlatPattern);
                return;
            }
            "sm_edit" => {
                self.actions.push(Action::EditLastOf("SheetMetal"));
                return;
            }
            "form_edit" => {
                self.actions.push(Action::EditLastOf("Subd"));
                return;
            }
            "form_box" | "form_cyl" | "form_sphere" | "form_torus" => {
                let kind: u8 = match id {
                    "form_box" => 0,
                    "form_cyl" => 1,
                    "form_sphere" => 2,
                    _ => 3,
                };
                let (size, sides) = super::form::defaults(kind);
                self.actions.push(Action::Open(Tool::Form {
                    kind,
                    size,
                    sides,
                    cage: super::form::primitive(kind, size, sides),
                    scale: "1".into(),
                    tolerance: "0.2 mm".into(),
                    vertex: 0,
                    built: (kind, size, sides),
                }));
                return;
            }
            "appearance" => {
                self.actions.push(Action::OpenAppearance);
                return;
            }
            "file_import" => {
                self.asm.request = Some(super::assembly::FileRequest::Import);
                return;
            }
            "file_export_step" | "file_export_stl" | "file_export_obj" | "file_export_glb" => {
                use crate::exchange::ExportFormat as F;
                self.asm.request = Some(super::assembly::FileRequest::Export(match id {
                    "file_export_step" => F::Step,
                    "file_export_stl" => F::Stl,
                    "file_export_obj" => F::Obj,
                    _ => F::Glb,
                }));
                return;
            }
            "asm_new" => {
                self.actions.push(Action::NewEmptyComponent);
                return;
            }
            "asm_body" => {
                self.actions
                    .push(Action::Asm(AsmAction::ComponentFromActiveBody));
                return;
            }
            "asm_design" => {
                self.actions.push(Action::NewComponent);
                return;
            }
            "asm_insert" => {
                self.asm.request = Some(super::assembly::FileRequest::Insert);
                return;
            }
            "asm_activate" => {
                self.actions.push(Action::Asm(AsmAction::ActivateSelected));
                return;
            }
            "asm_finish" => {
                self.actions.push(Action::FinishComponent);
                return;
            }
            "asm_joint" => {
                self.actions.push(Action::Asm(AsmAction::OpenJoint(false)));
                return;
            }
            "asm_as_built" => {
                self.actions.push(Action::Asm(AsmAction::OpenJoint(true)));
                return;
            }
            "asm_rigid" => {
                self.actions.push(Action::Asm(AsmAction::OpenRigid));
                return;
            }
            "asm_ground" => {
                self.actions.push(Action::Asm(AsmAction::GroundSelected));
                return;
            }
            "asm_view" => {
                self.asm.view = !self.asm.view;
                return;
            }
            "asm_move" => {
                self.asm.view = true;
                self.asm.move_form = Some(super::asm_drag::MoveForm::new());
                return;
            }
            "asm_motion" => {
                self.asm.open = !self.asm.open;
                return;
            }
            "asm_check" => {
                self.actions.push(Action::Asm(AsmAction::CheckInterference));
                return;
            }
            "asm_bom" => {
                self.actions.push(Action::Asm(AsmAction::OpenBom));
                return;
            }
            "zebra" => {
                self.toggle_analysis(super::analysis::AKind::Zebra);
                return;
            }
            "curvature" => {
                self.toggle_analysis(super::analysis::AKind::Mean);
                return;
            }
            "draft_analysis" => {
                self.toggle_analysis(super::analysis::AKind::Draft);
                return;
            }
            "section" => {
                self.section = match self.section {
                    Some(_) => None,
                    None => Some(super::section::SectionState::new()),
                };
                return;
            }
            "select" => {
                self.measure_open = false;
                self.dialog = None;
                self.preview = None;
                self.model.picks.clear();
                return;
            }
            _ => return,
        };
        self.actions.push(Action::Open(tool));
    }

    fn sketch_tab(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        let cur = self.sketch.app.editor.tool;
        let on = |t: SkTool| cur == t;
        let mut picked: Vec<&'static str> = Vec::new();

        // ------------------------------------------------------------------ CREATE
        let line = it(
            "sk_line",
            Icon::Line,
            "Line",
            "Line (L): click points; Escape ends the chain",
        )
        .key("L")
        .active(on(SkTool::Line));
        let rect = it(
            "sk_rect",
            Icon::Rectangle,
            "Rectangle",
            "Two-corner rectangle (R)",
        )
        .key("R")
        .active(on(SkTool::Rectangle));
        let rect_c = it(
            "sk_rect_c",
            Icon::Rectangle,
            "Centre Rectangle",
            "Centre, then a corner (Shift+R)",
        )
        .active(on(SkTool::CenterRectangle));
        let rect_a = it(
            "sk_rect_a",
            Icon::Rectangle,
            "3-Point Rectangle",
            "One side, then the width: a rectangle at any angle",
        )
        .active(on(SkTool::AlignedRectangle));
        let circle = it(
            "sk_circle",
            Icon::Circle,
            "Centre Circle",
            "Centre, then a point on the circle (C)",
        )
        .key("C")
        .active(on(SkTool::Circle));
        let circle3 = it(
            "sk_circle3",
            Icon::Circle3,
            "3-Point Circle",
            "Three points on the circle (X)",
        )
        .key("X")
        .active(on(SkTool::Circle3));
        let arc = it("sk_arc", Icon::Arc, "Centre Arc", "Centre, start, end (A)")
            .key("A")
            .active(on(SkTool::Arc));
        let arc3 = it(
            "sk_arc3",
            Icon::Arc3,
            "3-Point Arc",
            "Start, end, a point on the arc (W)",
        )
        .key("W")
        .active(on(SkTool::Arc3));
        let spline = it(
            "sk_spline",
            Icon::Spline,
            "Control Spline",
            "Spline by control points (S); double-click or Enter to finish",
        )
        .key("S")
        .active(on(SkTool::Spline));
        let fit = it(
            "sk_fit",
            Icon::FitSpline,
            "Fit Spline",
            "Spline through its points (K); double-click or Enter to finish",
        )
        .key("K")
        .active(on(SkTool::FitSpline));
        let polygon = it(
            "sk_polygon",
            Icon::Polygon,
            "Polygon",
            "Regular polygon (G): centre, then a vertex",
        )
        .key("G")
        .active(on(SkTool::Polygon));
        let slot = it(
            "sk_slot",
            Icon::Slot,
            "Slot",
            "Slot (U): two centres, then the width",
        )
        .key("U")
        .active(on(SkTool::Slot));
        let ellipse = it(
            "sk_ellipse",
            Icon::Ellipse,
            "Ellipse",
            "Ellipse (H): centre, major axis end, minor axis point",
        )
        .key("H")
        .active(on(SkTool::Ellipse));
        let point = it("sk_point", Icon::Point, "Point", "Point (P)")
            .key("P")
            .active(on(SkTool::Point));
        let text = it(
            "sk_text",
            Icon::Text,
            "Text",
            "Letters as sketch curves that can be extruded or cut",
        );
        let prim: Vec<Prim> = vec![
            line.clone().into(),
            Prim::Split(
                Item {
                    label: "Rectangle",
                    ..rect.clone()
                },
                vec![rect.clone(), rect_c.clone(), rect_a.clone()],
            ),
            Prim::Split(
                Item {
                    label: "Circle",
                    ..circle.clone()
                },
                vec![circle.clone(), circle3.clone()],
            ),
            Prim::Split(
                Item {
                    label: "Arc",
                    ..arc.clone()
                },
                vec![arc.clone(), arc3.clone()],
            ),
            Prim::Split(
                Item {
                    label: "Spline",
                    ..spline.clone()
                },
                vec![spline.clone(), fit.clone()],
            ),
        ];
        if let Some(id) = rb::menu_group(
            ui,
            "CREATE",
            &prim,
            &[
                Section::new(
                    "LINES AND SHAPES",
                    vec![line, rect, rect_c, rect_a, polygon, slot, ellipse],
                ),
                Section::new("CIRCLES AND ARCS", vec![circle, circle3, arc, arc3]),
                Section::new("CURVES AND POINTS", vec![spline, fit, point]),
                Section::new("TEXT", vec![text]),
            ],
        ) {
            picked.push(id);
        }

        // ------------------------------------------------------------------ MODIFY
        let trim = it(
            "sk_trim",
            Icon::Trim,
            "Trim",
            "Trim (T): click the part to remove",
        )
        .key("T")
        .active(on(SkTool::Trim));
        let extend = it(
            "sk_extend",
            Icon::Extend,
            "Extend",
            "Extend (E) a curve to the next one",
        )
        .key("E")
        .active(on(SkTool::Extend));
        let offset = it(
            "sk_offset",
            Icon::Offset,
            "Offset",
            "Offset (O): click curves, Enter, then the side",
        )
        .key("O")
        .active(on(SkTool::Offset));
        let fillet = it(
            "sk_fillet",
            Icon::SkFillet,
            "Fillet",
            "Round a corner (J): click the point where two lines meet",
        )
        .key("J")
        .active(on(SkTool::Fillet));
        let chamfer = it(
            "sk_chamfer",
            Icon::SkChamfer,
            "Chamfer",
            "Bevel a corner (B): click the point where two lines meet",
        )
        .key("B")
        .active(on(SkTool::Chamfer));
        let mirror = it(
            "sk_mirror",
            Icon::SkMirror,
            "Mirror",
            "Mirror (M): click the curves, press Enter, click the mirror line",
        )
        .key("M")
        .active(on(SkTool::Mirror));
        let pat_rect = it(
            "sk_pat_rect",
            Icon::SkPattern,
            "Rectangular Pattern",
            "Repeat the selected curves on a grid (select curves first)",
        );
        let pat_circ = it(
            "sk_pat_circ",
            Icon::CircularPattern,
            "Circular Pattern",
            "Repeat the selected curves about a centre (select curves first)",
        );
        let sk_project = it(
            "sk_project",
            Icon::Project,
            "Project",
            "Project the model's edges onto the sketch plane (locked curves)",
        );
        let sk_intersect = it(
            "sk_intersect",
            Icon::Project,
            "Intersect",
            "Where the sketch plane cuts the model (locked curves)",
        );
        if let Some(id) = rb::menu_group(
            ui,
            "MODIFY",
            &[
                trim.clone().into(),
                extend.clone().into(),
                offset.clone().into(),
                fillet.clone().into(),
            ],
            &[
                Section::new("EDIT", vec![trim, extend, offset]),
                Section::new("CORNERS", vec![fillet, chamfer]),
                Section::new("COPY", vec![mirror, pat_rect, pat_circ]),
            ],
        ) {
            picked.push(id);
        }
        if let Some(id) = rb::menu_group(
            ui,
            "PROJECT",
            &[sk_project.clone().into()],
            &[Section::new(
                "FROM THE MODEL",
                vec![sk_project, sk_intersect],
            )],
        ) {
            picked.push(id);
        }

        // ------------------------------------------------------------- CONSTRAINTS
        let dim = it(
            "sk_dimension",
            Icon::Dimension,
            "Dimension",
            "Sketch dimension (D)",
        )
        .key("D")
        .active(on(SkTool::Dimension));
        let grid: Vec<Item> = CONSTRAINTS
            .iter()
            .map(|(id, t, icon, name)| Item::new(id, *icon, name).active(on(SkTool::Constrain(*t))))
            .collect();
        rb::group(ui, "CONSTRAINTS", |ui| {
            if rb::button(ui, dim.icon, dim.label, true, dim.active, &dim.tip).clicked() {
                picked.push("sk_dimension");
            }
            if let Some(id) = rb::icon_grid(ui, &grid, 7) {
                picked.push(id);
            }
        });

        // ------------------------------------------------------------------ SNAPS
        rb::group(ui, "OPTIONS", |ui| {
            rb::popup_button(
                ui,
                Icon::Gear,
                "Snaps",
                "What the cursor snaps to; hold Alt to ignore snaps",
                |ui| self.sketch.app.snaps_ui(ui),
            );
        });

        // ------------------------------------------------------------------ FINISH
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
                picked.push("sk_finish");
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
                picked.push("sk_discard");
            }
        });

        for id in picked {
            self.run_sketch_command(id, project);
        }
    }

    /// What a SKETCH-tab command does.
    fn run_sketch_command(&mut self, id: &str, project: &mut Project) {
        let tool = match id {
            "sk_project" | "sk_intersect" => {
                self.sketch_project(project, id == "sk_intersect");
                return;
            }
            "sk_pat_rect" => {
                self.sketch_panel = Some(super::sketch_ops::SketchPanel::rect_pattern());
                return;
            }
            "sk_pat_circ" => {
                self.sketch_panel = Some(super::sketch_ops::SketchPanel::circ_pattern());
                return;
            }
            "sk_text" => {
                self.sketch_panel = Some(super::sketch_ops::SketchPanel::text());
                return;
            }
            "sk_line" => SkTool::Line,
            "sk_rect" => SkTool::Rectangle,
            "sk_rect_c" => SkTool::CenterRectangle,
            "sk_rect_a" => SkTool::AlignedRectangle,
            "sk_circle" => SkTool::Circle,
            "sk_circle3" => SkTool::Circle3,
            "sk_arc" => SkTool::Arc,
            "sk_arc3" => SkTool::Arc3,
            "sk_spline" => SkTool::Spline,
            "sk_fit" => SkTool::FitSpline,
            "sk_polygon" => SkTool::Polygon,
            "sk_slot" => SkTool::Slot,
            "sk_ellipse" => SkTool::Ellipse,
            "sk_point" => SkTool::Point,
            "sk_trim" => SkTool::Trim,
            "sk_extend" => SkTool::Extend,
            "sk_offset" => SkTool::Offset,
            "sk_fillet" => SkTool::Fillet,
            "sk_chamfer" => SkTool::Chamfer,
            "sk_mirror" => SkTool::Mirror,
            "sk_dimension" => SkTool::Dimension,
            "sk_finish" => {
                self.actions.push(Action::FinishSketch);
                return;
            }
            "sk_discard" => {
                self.actions.push(Action::CancelSketch);
                return;
            }
            other => match CONSTRAINTS.iter().find(|(cid, ..)| *cid == other) {
                Some((_, t, ..)) => SkTool::Constrain(*t),
                None => return,
            },
        };
        self.sketch.app.editor.set_tool(tool);
    }

    /// Run a ribbon command by id (what a click on that button does); used by tests.
    pub fn command(&mut self, id: &str, project: &mut Project) {
        if id.starts_with("sk_") || id.starts_with("c_") {
            self.run_sketch_command(id, project);
        } else {
            let cx = Self::context(project);
            self.run_solid_command(id, &cx);
        }
    }
}
