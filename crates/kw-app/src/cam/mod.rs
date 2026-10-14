//! The CAM workspace: setups (work coordinate system, stock, workpiece and fixture bodies), operations with
//! their settings, toolpath generation on a background thread, a stock simulation of the whole setup with a
//! comparison against the part and a check against the fixtures, and posting to five controller dialects.

pub mod fiveaxis;
pub mod library_ui;
mod panels;
mod settings;
pub mod setup;
pub mod sheet;
pub mod sim;
mod view;

use crate::dialogs::Dialogs;
use crate::job::Job;
use crate::model_source::{Combined, ModelSource};
use crate::project::Project;
use crate::viewport::{ModelScene, Viewport3d};
use eframe::egui;
use kw_cam::bvh::TriMesh;
use kw_cam::finish3d::moves;
use kw_cam::library::{speeds_and_feeds, with_holder, Library};
use kw_cam::path::{Event, Toolpath};
use kw_cam::post::{render, PostConfig};
use kw_cam::{OpParams, RegionSpec, StockSpec, Tool, ToolKind};
use kw_store::{BodyKey, BodyRole, CamSetup, SourceKind, Wcs};
use setup::{frame_of, geometry, geometry_key, op_hash, tool_number, tool_of, SetupGeometry};
use sim::SimResult;
use std::sync::Arc;

pub use kw_cam::OpKind;
pub use settings::{Settings, ToolChoice};
pub use view::{cut_segments, deviation_colour, stock_scene};

/// The post-processors that ship with the program, with the dialect of the independent interpreter that
/// replays their output.
pub const BUILTIN_POSTS: [&str; 5] = ["Fanuc", "Haas", "Siemens840D", "Heidenhain", "LinuxCNC"];

/// The extension of the programs a post writes (by its id).
pub fn program_extension(post: &str) -> &'static str {
    match post {
        "Heidenhain" => "h",
        "Siemens840D" => "mpf",
        "LinuxCNC" => "ngc",
        _ => "nc",
    }
}

/// One toolpath to make (or remake).
struct GenReq {
    /// The operation this replaces, else a new one is added.
    replace: Option<usize>,
    params: OpParams,
    tool: Tool,
}

struct GenOut {
    req: GenReq,
    path: Result<Toolpath, String>,
}

struct Generated {
    setup: usize,
    gkey: u128,
    outs: Vec<GenOut>,
}

/// The workspace's view of the bodies: the model and the roles of its bodies in the current setup.
struct SceneBodies {
    key: u128,
    combined: Arc<Combined>,
    colours: Vec<[u8; 3]>,
}

pub struct CamWorkspace {
    pub viewport: Viewport3d,
    pub settings: Settings,
    /// The tool, holder, machine and post libraries (the built-in ones with the user's laid over them).
    pub library: Library,
    /// Where the user's library files are kept.
    pub library_dir: Option<std::path::PathBuf>,
    pub library_open: bool,
    lib_ui: library_ui::State,
    /// The kind of operation the panel last adapted to.
    panel_kind: Option<OpKind>,
    /// The selected operation of the current setup.
    pub selected: Option<usize>,
    /// The setup being worked on.
    pub current: usize,
    pub message: String,
    /// Show the simulated stock instead of the model.
    pub view_stock: bool,
    gen_job: Option<Job<Generated>>,
    sim_job: Option<Job<(usize, u128, SimResult)>>,
    /// The simulation of setup `.0`, made from the state with key `.1`.
    pub sim: Option<(usize, u128, Arc<SimResult>)>,
    /// Increases whenever the displayed toolpath changes (the viewport rebuilds its scene).
    shown: u64,
    initialised_for: Option<(usize, u128)>,
    source: Option<(u128, Arc<ModelSource>)>,
    geo: Option<(u128, Arc<SetupGeometry>)>,
    scene_bodies: Option<SceneBodies>,
    /// Ribbon buttons ask for things here (`generate`, `simulate`); `show` carries them out.
    pub requests: Vec<&'static str>,
}

impl Default for CamWorkspace {
    fn default() -> Self {
        CamWorkspace {
            viewport: Viewport3d::default(),
            settings: Settings::default(),
            library: Library::builtin(),
            library_dir: kw_cam::library::config_dir(),
            library_open: false,
            lib_ui: library_ui::State::default(),
            panel_kind: None,
            selected: None,
            current: 0,
            message: String::new(),
            view_stock: false,
            gen_job: None,
            sim_job: None,
            sim: None,
            shown: 0,
            initialised_for: None,
            source: None,
            geo: None,
            scene_bodies: None,
            requests: Vec::new(),
        }
    }
}

/// What operation `index` of the setup depends on: the geometry key, and for an operation that works on
/// what the earlier operations left (rest finishing), those operations too.
fn op_key(setup: &CamSetup, index: usize, gkey: u128) -> u128 {
    let needs = setup.operations[index]
        .params
        .as_ref()
        .is_some_and(|p| p.kind.needs_stock());
    if !needs {
        return gkey;
    }
    let mut h = kw_feature::Hasher128::new();
    h.u128(gkey);
    for o in &setup.operations[..index] {
        h.u64(o.input_hash);
        h.u64(o.path.events.len() as u64);
        h.u64(o.path.cutting_length().to_bits());
    }
    h.finish()
}

fn model_key(project: &mut Project, kind: SourceKind) -> u128 {
    let mut h = kw_feature::Hasher128::new();
    h.u128(project.document().content_hash());
    if kind == SourceKind::Assembly {
        if let Some(a) = &project.assembly {
            h.bytes(&a.to_bytes());
        }
    }
    h.finish()
}

/// The six presets for the direction the tool approaches from, as (label, z axis, x axis).
pub const ORIENTATIONS: [(&str, [f64; 3], [f64; 3]); 6] = [
    ("Top", [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
    ("Bottom", [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]),
    ("Front", [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]),
    ("Back", [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]),
    ("Left", [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
    ("Right", [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
];

impl CamWorkspace {
    /// The ribbon of the Manufacture workspace.
    pub fn ribbon(&mut self, ui: &mut egui::Ui) {
        use crate::chrome::{ribbon as rb, Icon};
        let mut cur = 0usize;
        rb::tabs(
            ui,
            &[
                ("MILLING", true),
                ("TURNING", false),
                ("ADDITIVE", false),
                ("INSPECTION", false),
            ],
            &mut cur,
        );
        ui.add_space(2.0);
        let op = self.settings.op;
        let mut req: Vec<&'static str> = Vec::new();
        let mut set: Option<OpKind> = None;
        ui.horizontal(|ui| {
            rb::group(ui, "SETUP", |ui| {
                if rb::button(
                    ui,
                    Icon::Cam,
                    "New Setup",
                    true,
                    false,
                    "Add a setup: another way of holding the workpiece, with its own work coordinates and stock",
                )
                .clicked()
                {
                    req.push("new_setup");
                }
                if rb::button(
                    ui,
                    Icon::Cam,
                    "Libraries",
                    true,
                    self.library_open,
                    "Tools, holders, machines and post-processors",
                )
                .clicked()
                {
                    req.push("library");
                }
            });
            for (group, items) in [
                (
                    "2D",
                    &[
                        (OpKind::Face, Icon::Plane, "Face"),
                        (OpKind::Pocket, Icon::Rectangle, "Pocket"),
                        (OpKind::Adaptive, Icon::Offset, "Adaptive Clearing"),
                        (OpKind::ContourOutside, Icon::Spline, "2D Contour"),
                        (OpKind::ContourInside, Icon::Trim, "Inside Contour"),
                        (OpKind::Chamfer, Icon::Chamfer, "Chamfer"),
                    ][..],
                ),
                (
                    "HOLES",
                    &[
                        (OpKind::Drill, Icon::Hole, "Drill"),
                        (OpKind::Bore, Icon::Circle, "Bore"),
                    ][..],
                ),
                (
                    "3D",
                    &[
                        (OpKind::Finish3d, Icon::Cam, "3D Finish"),
                        (OpKind::Waterline, Icon::Curvature, "Waterline"),
                        (OpKind::RestFinish, Icon::Finish, "Rest Finish"),
                    ][..],
                ),
                (
                    "MULTI-AXIS",
                    &[
                        (OpKind::Swarf, Icon::Ruled, "Swarf"),
                        (OpKind::TiltedContour, Icon::Draft, "Tilted Contour"),
                    ][..],
                ),
            ] {
                rb::group(ui, group, |ui| {
                    for (k, icon, label) in items {
                        if rb::button(
                            ui,
                            *icon,
                            label,
                            true,
                            op == *k,
                            if k.uses_points() {
                                "Choose the operation; pick its holes in the panel, then add it"
                            } else {
                                "Choose the operation and generate its toolpath"
                            },
                        )
                        .clicked()
                        {
                            set = Some(*k);
                            // an operation that needs its holes chosen is set up first
                            if !k.uses_points() {
                                req.push("generate");
                            }
                        }
                    }
                });
            }
            rb::group(ui, "ACTIONS", |ui| {
                if rb::button(
                    ui,
                    Icon::Play,
                    "Simulate",
                    true,
                    self.view_stock,
                    "Cut the stock with every operation of the setup and compare it with the part",
                )
                .clicked()
                {
                    req.push("simulate");
                }
            });
        });
        if let Some(k) = set {
            self.settings.op = k;
        }
        self.requests.extend(req);
    }

    // ------------------------------------------------------------------------------------- the model

    /// The setup being worked on, made if the project has none.
    fn current_setup<'a>(&mut self, project: &'a mut Project) -> &'a mut CamSetup {
        if project.cam.is_empty() {
            project.cam.push(CamSetup::default());
        }
        self.current = self.current.min(project.cam.len() - 1);
        let i = self.current;
        let s = &mut project.cam[i];
        if s.name.is_empty() {
            s.name = format!("Setup {}", i + 1);
        }
        s
    }

    /// The bodies the current setup works on (cached until the model changes).
    pub fn source(&mut self, project: &mut Project) -> Result<Arc<ModelSource>, String> {
        let wanted = project
            .cam
            .get(self.current)
            .map_or(SourceKind::Design, |s| s.source);
        let kind = if wanted == SourceKind::Assembly && project.assembly.is_some() {
            SourceKind::Assembly
        } else {
            SourceKind::Design
        };
        let key = model_key(project, kind);
        if let Some((k, s)) = &self.source {
            if *k == key && s.kind == kind {
                return Ok(s.clone());
            }
        }
        let s = Arc::new(ModelSource::of(project, kind, &Default::default())?);
        self.source = Some((key, s.clone()));
        self.geo = None;
        self.scene_bodies = None;
        Ok(s)
    }

    /// The bodies of the current setup tessellated in its work coordinates (cached).
    pub fn geometry(&mut self, project: &mut Project) -> Result<Arc<SetupGeometry>, String> {
        let source = self.source(project)?;
        let setup = self.current_setup(project).clone();
        let key = geometry_key(&source, &setup);
        if let Some((k, g)) = &self.geo {
            if *k == key {
                return Ok(g.clone());
            }
        }
        let g = Arc::new(geometry(&source, &setup)?);
        self.geo = Some((key, g.clone()));
        Ok(g)
    }

    /// The box of the workpiece bodies of the setup in its work coordinates, from the bounding boxes of
    /// their B-reps.
    pub fn workpiece_bounds(
        source: &ModelSource,
        setup: &CamSetup,
    ) -> Option<([f64; 3], [f64; 3])> {
        let frame = frame_of(&setup.wcs);
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        for b in &source.bodies {
            if setup.role_of(&b.key) != BodyRole::Workpiece {
                continue;
            }
            let bb = b.brep.body_bbox(b.body);
            for i in 0..8 {
                let p = frame.to_frame([
                    if i & 1 == 0 { bb.min.x } else { bb.max.x },
                    if i & 2 == 0 { bb.min.y } else { bb.max.y },
                    if i & 4 == 0 { bb.min.z } else { bb.max.z },
                ]);
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
        (lo[0] <= hi[0]).then_some((lo, hi))
    }

    /// Take the region and heights from the workpiece (and the top of the stock).
    pub fn fit_to_workpiece(&mut self, project: &mut Project) {
        let Ok(source) = self.source(project) else {
            return;
        };
        let setup = self.current_setup(project).clone();
        let Some((lo, hi)) = Self::workpiece_bounds(&source, &setup) else {
            return;
        };
        let top = match setup.stock {
            StockSpec::Box { top, .. } => hi[2] + top,
            StockSpec::Cylinder { axis: 2, ends, .. } => hi[2] + ends,
            StockSpec::Explicit { max, .. } => max[2],
            _ => hi[2],
        };
        let s = &mut self.settings;
        (s.x0, s.y0, s.x1, s.y1) = (lo[0], lo[1], hi[0], hi[1]);
        s.region = None;
        s.top = top;
        s.bottom = (top - 3.0).max(lo[2]);
    }

    /// Take the region and heights from the first body of a B-rep (the work system is the world).
    pub fn fit_to_model(&mut self, brep: &kw_brep::BRep) {
        if let Some(body) = brep.bodies.keys().next() {
            let bb = brep.body_bbox(body);
            let s = &mut self.settings;
            s.x0 = bb.min.x;
            s.y0 = bb.min.y;
            s.x1 = bb.max.x;
            s.y1 = bb.max.y;
            s.region = None;
            s.top = bb.max.z;
            s.bottom = (bb.max.z - 3.0).max(bb.min.z);
        }
    }

    /// Put the origin of the work system at the top corner (lowest x and y) of the workpiece as seen in the
    /// current orientation.
    pub fn place_origin(&mut self, project: &mut Project) {
        let Ok(source) = self.source(project) else {
            return;
        };
        let mut setup = self.current_setup(project).clone();
        setup.wcs.origin = [0.0; 3];
        let Some((lo, hi)) = Self::workpiece_bounds(&source, &setup) else {
            return;
        };
        let frame = frame_of(&setup.wcs);
        let origin = frame.to_work([lo[0], lo[1], hi[2]]);
        self.current_setup(project).wcs.origin = origin;
        project.mark_extras_dirty();
        self.shown += 1;
    }

    /// Set the direction the tool approaches from (one of [`ORIENTATIONS`]) and put the origin on the top
    /// corner of the workpiece.
    pub fn set_orientation(&mut self, project: &mut Project, which: usize) {
        let (_, z, x) = ORIENTATIONS[which.min(ORIENTATIONS.len() - 1)];
        let s = self.current_setup(project);
        s.wcs.z_axis = z;
        s.wcs.x_axis = x;
        self.place_origin(project);
        self.sim = None;
    }

    /// Add a setup (the next work offset), looking from the top.
    pub fn new_setup(&mut self, project: &mut Project) {
        self.current_setup(project);
        let n = project.cam.len();
        let prev = &project.cam[self.current];
        let new = CamSetup {
            name: format!("Setup {}", n + 1),
            machine: prev.machine.clone(),
            source: prev.source,
            bodies: prev.bodies.clone(),
            wcs: Wcs {
                offset: (n as u8 % 6) + 1,
                ..Wcs::default()
            },
            ..CamSetup::default()
        };
        project.cam.push(new);
        self.current = n;
        self.selected = None;
        self.sim = None;
        project.mark_extras_dirty();
        self.fit_to_workpiece(project);
        self.shown += 1;
    }

    pub fn delete_setup(&mut self, project: &mut Project, i: usize) {
        if i < project.cam.len() {
            project.cam.remove(i);
            self.current = self.current.min(project.cam.len().saturating_sub(1));
            self.selected = None;
            self.sim = None;
            self.geo = None;
            project.mark_extras_dirty();
            self.shown += 1;
        }
    }

    // ------------------------------------------------------------------------------- operations

    /// Is the operation out of date with the model, its tool, the stock and the work system?
    pub fn is_stale(setup: &CamSetup, index: usize, gkey: u128) -> bool {
        let op = &setup.operations[index];
        match (&op.params, tool_of(setup, op.tool)) {
            (Some(p), Some(t)) => op_hash(p, t, op_key(setup, index, gkey)) != op.input_hash,
            (Some(_), None) => true,
            _ => false,
        }
    }

    /// The operations of the current setup that no longer match it.
    pub fn stale_ops(&mut self, project: &mut Project) -> Vec<usize> {
        let Ok(source) = self.source(project) else {
            return Vec::new();
        };
        let setup = self.current_setup(project);
        let gkey = geometry_key(&source, setup);
        (0..setup.operations.len())
            .filter(|i| Self::is_stale(setup, *i, gkey))
            .collect()
    }

    fn new_request(&self) -> Vec<GenReq> {
        vec![GenReq {
            replace: None,
            params: self.settings.params(),
            tool: self.settings.tool(),
        }]
    }

    fn make_job(
        &mut self,
        project: &mut Project,
        reqs: Vec<GenReq>,
        label: &str,
    ) -> Result<Job<Generated>, String> {
        let source = self.source(project)?;
        let setup_index = self.current;
        let setup = self.current_setup(project).clone();
        let setups: Vec<CamSetup> = project.cam[..=setup_index].to_vec();
        let gkey = geometry_key(&source, &setup);
        let needs_geometry = reqs
            .iter()
            .any(|r| r.params.kind.needs_model() || r.params.kind.needs_stock());
        // the machine cannot run faster than it can
        let mut reqs = reqs;
        let machine = self.machine_of(&setup);
        if let Some(m) = &machine {
            for r in &mut reqs {
                if m.spindle_min <= m.spindle_max {
                    r.params.rpm = r.params.rpm.clamp(m.spindle_min, m.spindle_max);
                }
                r.params.feed = r.params.feed.min(m.max_feed);
            }
        }
        for r in &reqs {
            if r.params.kind.is_five_axis() && machine.as_ref().is_none_or(|m| m.axes() < 5) {
                return Err(format!(
                    "{} tilts the tool: choose a machine with rotary axes first",
                    r.params.kind.label()
                ));
            }
        }
        let ops_now = setup.operations.len();
        Ok(Job::spawn_with(label, move |cancel, progress| {
            let geo = if needs_geometry {
                Some(geometry(&source, &setup)?)
            } else {
                None
            };
            let mesh = geo.as_ref().map(|g| TriMesh::new(g.workpiece.clone()));
            let n = reqs.len().max(1);
            let mut outs = Vec::new();
            for (k, req) in reqs.into_iter().enumerate() {
                if cancel.is_cancelled() {
                    return Err("cancelled".into());
                }
                progress.set(k as f64 / n as f64);
                // the stock the operations before this one leave
                let stock = if req.params.kind.needs_stock() {
                    let prior = req.replace.unwrap_or(ops_now).min(ops_now);
                    let previous = sim::previous_surface(&source, &setups, setup_index, cancel)?;
                    Some(sim::stock_after(
                        &setup,
                        geo.as_ref().expect("geometry"),
                        previous.as_ref(),
                        prior,
                    )?)
                } else {
                    None
                };
                let path = req.params.generate_with(
                    &req.tool,
                    &kw_cam::ops::GenContext {
                        mesh: mesh.as_ref(),
                        stock: stock.as_ref(),
                    },
                );
                outs.push(GenOut { req, path });
            }
            Ok(Generated {
                setup: setup_index,
                gkey,
                outs,
            })
        }))
    }

    /// Generate and record an operation now, on this thread (tests and demos).
    pub fn run_now(&mut self, project: &mut Project) -> Result<(), String> {
        let reqs = self.new_request();
        let job = self.make_job(project, reqs, "operation")?;
        let out = loop {
            if let Some(r) = job.poll() {
                break r?;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        };
        self.apply(project, out)
    }

    /// Start generating the operation described by the settings.
    pub fn start(&mut self, project: &mut Project) {
        let reqs = self.new_request();
        match self.make_job(project, reqs, self.settings.op.label()) {
            Ok(job) => {
                self.gen_job = Some(job);
                self.message = format!("generating {}…", self.settings.op.label());
            }
            Err(e) => self.message = e,
        }
    }

    /// Start remaking the selected operation from the settings in the panel.
    pub fn start_update(&mut self, project: &mut Project) {
        let Some(sel) = self.selected else {
            return;
        };
        let reqs = vec![GenReq {
            replace: Some(sel),
            params: self.settings.params(),
            tool: self.settings.tool(),
        }];
        match self.make_job(project, reqs, "update operation") {
            Ok(job) => {
                self.gen_job = Some(job);
                self.message = "updating the operation…".into();
            }
            Err(e) => self.message = e,
        }
    }

    /// Start remaking the stale operations of the current setup with the settings they were made with.
    pub fn start_regenerate(&mut self, project: &mut Project) {
        let stale = self.stale_ops(project);
        if stale.is_empty() {
            self.message = "every operation is up to date".into();
            return;
        }
        let setup = self.current_setup(project).clone();
        let mut reqs = Vec::new();
        for i in stale {
            let op = &setup.operations[i];
            if let (Some(p), Some(t)) = (&op.params, tool_of(&setup, op.tool)) {
                reqs.push(GenReq {
                    replace: Some(i),
                    params: p.clone(),
                    tool: t.clone(),
                });
            }
        }
        match self.make_job(project, reqs, "regenerate") {
            Ok(job) => {
                self.gen_job = Some(job);
                self.message = "regenerating the stale operations…".into();
            }
            Err(e) => self.message = e,
        }
    }

    /// Record the toolpaths a job made.
    fn apply(&mut self, project: &mut Project, g: Generated) -> Result<(), String> {
        let mut errors = Vec::new();
        let mut summary = String::new();
        for out in g.outs {
            let path = match out.path {
                Ok(p) => p,
                Err(e) => {
                    errors.push(e);
                    continue;
                }
            };
            if g.setup >= project.cam.len() {
                continue;
            }
            let mut params = out.req.params;
            let mut tool = out.req.tool;
            tool.number = tool_number(&mut project.cam, g.setup, &tool);
            let setup = &mut project.cam[g.setup];
            params.tool = tool.number;
            // the path was made with the tool as the settings named it; it is called by its number in the setup
            let mut path = path;
            for e in &mut path.events {
                if let Event::ToolChange { number, .. } = e {
                    *number = tool.number;
                }
            }
            let target = match out.req.replace {
                Some(i) if i < setup.operations.len() => i,
                _ => setup.operations.len(),
            };
            // (the operation's own entry may not exist yet: the key only reads the ones before it)
            let key = {
                let needs = params.kind.needs_stock();
                if needs {
                    let mut probe = setup.clone();
                    if target >= probe.operations.len() {
                        probe.operations.push(kw_store::CamOperation {
                            name: String::new(),
                            tool: tool.number,
                            path: Toolpath::new(""),
                            params: Some(params.clone()),
                            input_hash: 0,
                        });
                    } else {
                        probe.operations[target].params = Some(params.clone());
                    }
                    op_key(&probe, target, g.gkey)
                } else {
                    g.gkey
                }
            };
            let hash = op_hash(&params, &tool, key);
            let len = path.cutting_length();
            let time = setup::estimate_seconds(&path);
            summary = format!(
                "{}: {} events, {:.0} mm of cutting, about {:.0} s",
                params.kind.label(),
                path.events.len(),
                len,
                time
            );
            match out.req.replace {
                Some(i) if i < setup.operations.len() => {
                    let op = &mut setup.operations[i];
                    op.path = path;
                    op.tool = tool.number;
                    op.params = Some(params);
                    op.input_hash = hash;
                    self.selected = Some(i);
                }
                _ => {
                    let name = format!("{} {}", params.kind.label(), setup.operations.len() + 1);
                    setup.operations.push(kw_store::CamOperation {
                        name,
                        tool: tool.number,
                        path,
                        params: Some(params),
                        input_hash: hash,
                    });
                    self.selected = Some(setup.operations.len() - 1);
                }
            }
        }
        project.mark_extras_dirty();
        self.shown += 1;
        self.sim = None;
        if errors.is_empty() {
            self.message = summary;
            Ok(())
        } else {
            self.message = errors.join("; ");
            Err(self.message.clone())
        }
    }

    /// The post-processor of a setup: its embedded copy, else the one its machine names, else Fanuc.
    pub fn post_config_of(&self, setup: &CamSetup) -> Result<PostConfig, String> {
        if let Some(p) = &setup.post {
            return PostConfig::parse(&p.text).map_err(|e| e.to_string());
        }
        let id = setup
            .machine_def
            .as_ref()
            .map(|m| m.post.as_str())
            .or_else(|| {
                self.library
                    .machine(&setup.machine)
                    .map(|m| m.post.as_str())
            })
            .unwrap_or("Fanuc");
        self.library.post_config(id)
    }

    /// The id of the post-processor of a setup.
    pub fn post_id_of(&self, setup: &CamSetup) -> String {
        self.post_config_of(setup)
            .map(|c| c.name)
            .unwrap_or_else(|_| "Fanuc".into())
    }

    /// Use the library machine `name` in the current setup (its post with it), embedding copies of both.
    pub fn set_machine(&mut self, project: &mut Project, name: &str) -> Result<(), String> {
        let m = self
            .library
            .machine(name)
            .cloned()
            .ok_or_else(|| format!("there is no machine '{name}' in the library"))?;
        let post = self.library.post(&m.post).cloned();
        let s = self.current_setup(project);
        s.machine = m.name.clone();
        s.machine_def = Some(m);
        if post.is_some() {
            s.post = post;
        }
        project.mark_extras_dirty();
        Ok(())
    }

    /// Use the library post-processor `id` in the current setup, embedding a copy.
    pub fn set_post(&mut self, project: &mut Project, id: &str) -> Result<(), String> {
        let p = self
            .library
            .post(id)
            .cloned()
            .ok_or_else(|| format!("there is no post-processor '{id}' in the library"))?;
        self.current_setup(project).post = Some(p);
        project.mark_extras_dirty();
        Ok(())
    }

    /// Replace the embedded machine and post of every setup by the library's current ones (those the library
    /// has; a setup keeps a copy the library no longer holds).
    pub fn refresh_from_library(&mut self, project: &mut Project) -> usize {
        let mut changed = 0;
        for s in project.cam.iter_mut() {
            if let Some(m) = self.library.machine(&s.machine).cloned() {
                if s.machine_def.as_ref() != Some(&m) {
                    s.machine_def = Some(m);
                    changed += 1;
                }
            }
            if let Some(id) = s.post.as_ref().map(|p| p.id.clone()) {
                if let Some(p) = self.library.post(&id).cloned() {
                    if s.post.as_ref() != Some(&p) {
                        s.post = Some(p);
                        changed += 1;
                    }
                }
            }
        }
        if changed > 0 {
            project.mark_extras_dirty();
        }
        changed
    }

    /// Read the user's library files over the built-in libraries; problems are reported in the message.
    pub fn load_library(&mut self) {
        let Some(dir) = self.library_dir.clone() else {
            return;
        };
        let (lib, warnings) = Library::load_dir(&dir);
        self.library = lib;
        if !warnings.is_empty() {
            self.message = format!("library: {}", warnings.join("; "));
        }
    }

    /// Write the user's library entries to the library folder.
    pub fn save_library(&mut self) -> Result<(), String> {
        let dir = self
            .library_dir
            .clone()
            .ok_or("there is no folder for the library")?;
        self.library.save_dir(&dir)
    }

    /// The machine of the current setup: its embedded copy, else the library's.
    pub fn machine_of(&self, setup: &CamSetup) -> Option<kw_cam::library::MachineDef> {
        setup
            .machine_def
            .clone()
            .or_else(|| self.library.machine(&setup.machine).cloned())
    }

    /// Set the spindle speed and feed of the settings from the workpiece material and the tool, held to the
    /// machine's limits. The text says what was used.
    pub fn feeds_from_material(&mut self, project: &mut Project) -> Result<String, String> {
        let source = self.source(project)?;
        let setup = self.current_setup(project).clone();
        let body = source
            .bodies
            .iter()
            .find(|b| setup.role_of(&b.key) == BodyRole::Workpiece)
            .ok_or("the setup has no workpiece")?;
        let name = body
            .material
            .clone()
            .ok_or_else(|| format!("{} has no material; set one on the body", body.name))?;
        let mat = kw_materials::global()
            .find(&name)
            .ok_or_else(|| format!("there is no material '{name}' in the materials library"))?;
        let tool = self.settings.tool();
        let group = mat.cam.group.name();
        let own = self
            .library
            .tool(&tool.name)
            .and_then(|d| d.cutting.iter().find(|c| c.group == group));
        let (vc, fz, from) = match own {
            Some(c) => (c.vc, c.fz, "the tool's own data"),
            None => (
                mat.cam.nominal_speed(),
                mat.cam.chip_load_factor * tool.diameter,
                "the material's data",
            ),
        };
        let machine = self.machine_of(&setup);
        let sp = speeds_and_feeds(&tool, vc, fz, machine.as_ref());
        self.settings.rpm = sp.rpm.round();
        self.settings.feed = sp.feed.round();
        Ok(format!(
            "{name} ({group}): {vc:.0} m/min and {fz:.3} mm/tooth from {from} give {:.0} rpm and {:.0} mm/min{}",
            sp.rpm,
            sp.feed,
            if sp.limited {
                ", held to the machine's limits"
            } else {
                ""
            }
        ))
    }

    /// The program of a setup for its machine, with the five-axis report when rotary axes were used.
    pub fn program_of(
        &self,
        setup: &CamSetup,
    ) -> Result<(Toolpath, fiveaxis::FiveAxisReport), String> {
        let machine = self
            .machine_of(setup)
            .or_else(|| self.library.machine("3-axis mill").cloned())
            .ok_or("there is no machine")?;
        fiveaxis::program(setup, &machine)
    }

    /// The text of the selected operation alone, under its work offset.
    pub fn post_text(&self, project: &Project) -> Option<Result<String, String>> {
        let setup = project.cam.get(self.current)?;
        let op = setup.operations.get(self.selected?)?;
        let mut one = setup.clone();
        one.operations = vec![op.clone()];
        Some(self.post_setup(&one))
    }

    /// The program of a setup: all its operations under its work offset.
    pub fn setup_post_text(
        &self,
        project: &Project,
        index: usize,
    ) -> Option<Result<String, String>> {
        let setup = project.cam.get(index)?;
        if setup.operations.is_empty() {
            return None;
        }
        Some(self.post_setup(setup))
    }

    fn post_setup(&self, setup: &CamSetup) -> Result<String, String> {
        let (tp, _) = self.program_of(setup)?;
        let cfg = self.post_config_of(setup)?;
        render(&cfg, &tp).map_err(|e| e.to_string())
    }

    // ------------------------------------------------------------------------------- simulation

    /// Simulate the current setup (and the earlier ones it starts from) in the background.
    pub fn start_sim(&mut self, project: &mut Project) {
        let source = match self.source(project) {
            Ok(s) => s,
            Err(e) => {
                self.message = e;
                return;
            }
        };
        self.current_setup(project);
        let current = self.current;
        let setups: Vec<CamSetup> = project.cam[..=current].to_vec();
        let key = self.sim_key(&source, &setups);
        let mut first = current;
        while first > 0 && setups[first].stock == StockSpec::Previous {
            first -= 1;
        }
        self.sim_job = Some(Job::spawn_with(
            "stock simulation",
            move |cancel, progress| {
                let mut previous: Option<setup::Tris> = None;
                let mut last = None;
                for (k, setup) in setups.iter().enumerate().skip(first) {
                    let geo = geometry(&source, setup)?;
                    let r = sim::simulate_setup(
                        setup,
                        &geo,
                        previous.as_ref(),
                        cancel,
                        &kw_job::Progress::new(),
                    )?;
                    progress.set((k - first + 1) as f64 / (current - first + 1) as f64);
                    previous = Some(r.world.clone());
                    last = Some(r);
                }
                Ok((current, key, last.expect("a setup")))
            },
        ));
        self.message = "simulating stock removal…".into();
    }

    /// What a simulation of setups `..=current` depends on.
    fn sim_key(&self, source: &ModelSource, setups: &[CamSetup]) -> u128 {
        let mut h = kw_feature::Hasher128::new();
        h.u128(source.key);
        for s in setups {
            h.bytes(format!("{:?}", s).as_bytes());
        }
        h.finish()
    }

    fn sim_message(r: &SimResult) -> String {
        let removed = r.stock_volume - r.final_volume;
        let collisions = r.ops.iter().filter(|o| o.collides()).count();
        format!(
            "{}: stock {:.0} mm³ → {:.0} mm³: {:.0} mm³ removed ({:.1} %), cell {:.2} mm; still outside the part {:.0} mm³, cut into the part {:.0} mm³; rapids through material: {:.1} mm³; {} operation(s) with a collision",
            r.setup,
            r.stock_volume,
            r.final_volume,
            removed,
            100.0 * removed / r.stock_volume.max(1e-9),
            r.cell,
            r.remaining,
            r.overcut,
            r.ops.iter().map(|o| o.rapid_cut).sum::<f64>(),
            collisions
        )
    }

    // -------------------------------------------------------------------------------------- the UI

    fn scene_bodies(&mut self, project: &mut Project) -> Option<(Arc<Combined>, Vec<[u8; 3]>)> {
        let source = self.source(project).ok()?;
        let setup = self.current_setup(project).clone();
        let mut h = kw_feature::Hasher128::new();
        h.u128(source.key);
        h.bytes(format!("{:?}", setup.bodies).as_bytes());
        let key = h.finish();
        if let Some(s) = &self.scene_bodies {
            if s.key == key {
                return Some((s.combined.clone(), s.colours.clone()));
            }
        }
        let combined = Arc::new(source.combined(&|k| setup.role_of(k) != BodyRole::Ignore));
        let colours = combined
            .source_index
            .iter()
            .map(|i| match setup.role_of(&source.bodies[*i].key) {
                BodyRole::Fixture => [214, 150, 70],
                _ => source.bodies[*i].colour,
            })
            .collect::<Vec<_>>();
        self.scene_bodies = Some(SceneBodies {
            key,
            combined: combined.clone(),
            colours: colours.clone(),
        });
        Some((combined, colours))
    }

    /// Take the results of finished jobs; true while one is still running.
    pub fn poll_jobs(&mut self, project: &mut Project) -> bool {
        let mut pending = false;
        if let Some(job) = &self.gen_job {
            match job.poll() {
                Some(Ok(g)) => {
                    self.gen_job = None;
                    let _ = self.apply(project, g);
                }
                Some(Err(e)) => {
                    self.message = e;
                    self.gen_job = None;
                }
                None => pending = true,
            }
        }
        if let Some(job) = &self.sim_job {
            match job.poll() {
                Some(Ok((setup, key, r))) => {
                    self.message = Self::sim_message(&r);
                    self.sim = Some((setup, key, Arc::new(r)));
                    self.view_stock = true;
                    self.shown += 1;
                    self.sim_job = None;
                }
                Some(Err(e)) => {
                    self.message = e;
                    self.sim_job = None;
                }
                None => pending = true,
            }
        }
        pending
    }

    /// Wait for the toolpath job to finish (tests and demos).
    pub fn wait_generation(&mut self, project: &mut Project) {
        while self.poll_jobs(project) {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
        dialogs: &mut dyn Dialogs,
    ) {
        self.current_setup(project);
        let source = self.source(project);
        let skey = source.as_ref().map(|s| s.key).unwrap_or(0);
        if self.initialised_for != Some((self.current, skey)) {
            if source.is_ok() {
                self.fit_to_workpiece(project);
            }
            self.initialised_for = Some((self.current, skey));
        }
        if self.poll_jobs(project) {
            ui.ctx().request_repaint();
        }
        for r in std::mem::take(&mut self.requests) {
            match r {
                "generate" => {
                    if self.gen_job.is_none() {
                        self.start(project);
                    }
                }
                "simulate" => {
                    if self.sim_job.is_none() {
                        self.start_sim(project);
                    }
                }
                "new_setup" => self.new_setup(project),
                "library" => self.library_open = !self.library_open,
                _ => {}
            }
        }
        self.library_window(ui.ctx(), project, dialogs);
        egui::Panel::left("cam_side")
            .default_size(340.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.side(ui, project, dialogs))
            });
        let setup = self.current_setup(project).clone();
        let frame = frame_of(&setup.wcs);
        let sim_now = self
            .sim
            .as_ref()
            .filter(|(i, _, _)| *i == self.current)
            .map(|(_, _, r)| r.clone());
        let bodies = self.scene_bodies(project);
        egui::CentralPanel::default().show(ui, |ui| {
            let segs: Vec<([f64; 3], [f64; 3])> = self
                .selected
                .and_then(|i| setup.operations.get(i))
                .map(|o| {
                    cut_segments(&o.path)
                        .into_iter()
                        .map(|(a, b)| (frame.to_work(a), frame.to_work(b)))
                        .collect()
                })
                .unwrap_or_default();
            // the axes of the work coordinate system
            let len = 10.0;
            let o = frame.origin;
            let axes = [frame.u, frame.v, frame.w]
                .map(|d| (o, [o[0] + len * d[0], o[1] + len * d[1], o[2] + len * d[2]]));
            let show_stock = self.view_stock && sim_now.is_some();
            let key = skey
                ^ ((self.shown as u128) << 64)
                ^ (self.selected.map(|s| s as u128 + 1).unwrap_or(0) << 100)
                ^ ((self.current as u128) << 108)
                ^ ((show_stock as u128) << 120)
                ^ segs.len() as u128;
            self.viewport.wireframe = !show_stock;
            self.viewport.show_with(
                ui,
                rs,
                key,
                move || {
                    let mut m = if let (true, Some(s)) = (show_stock, &sim_now) {
                        stock_scene(s)
                    } else {
                        match &bodies {
                            Some((c, colours)) => ModelScene::build_coloured(&c.brep, colours),
                            None => ModelScene::empty(),
                        }
                    };
                    m.add_segments(&segs);
                    m.add_segments(&axes);
                    m
                },
                None,
                crate::viewport::ViewOpts::default(),
            );
        });
    }

    /// Write one program per setup into `dir`; the message says what happened.
    pub fn write_all(&self, project: &Project, dir: &std::path::Path) -> String {
        let mut wrote = 0;
        for (i, s) in project.cam.iter().enumerate() {
            let ext = program_extension(&self.post_id_of(s));
            let Some(Ok(text)) = self.setup_post_text(project, i) else {
                continue;
            };
            let file = dir.join(format!(
                "{}_{}_{}.{ext}",
                project.name(),
                i + 1,
                s.name.replace(' ', "_")
            ));
            if let Err(e) = std::fs::write(&file, text) {
                return format!("could not write {}: {e}", file.display());
            }
            wrote += 1;
        }
        format!("wrote {wrote} program(s) to {}", dir.display())
    }

    /// Write the setup sheet as a web page and a CSV file next to it.
    pub fn write_sheet(&mut self, project: &mut Project, path: &std::path::Path) -> String {
        let names: std::collections::HashMap<BodyKey, String> = self
            .source(project)
            .map(|s| {
                s.bodies
                    .iter()
                    .map(|b| (b.key.clone(), b.name.clone()))
                    .collect()
            })
            .unwrap_or_default();
        let name_of = |k: &BodyKey| names.get(k).cloned().unwrap_or_else(|| format!("{k:?}"));
        let html = sheet::html(&project.name(), &project.cam, &name_of);
        let csv = sheet::csv(&project.cam);
        let csv_path = path.with_extension("csv");
        match std::fs::write(path, html).and_then(|()| std::fs::write(&csv_path, csv)) {
            Ok(()) => format!("wrote {} and {}", path.display(), csv_path.display()),
            Err(e) => format!("could not write: {e}"),
        }
    }
}
