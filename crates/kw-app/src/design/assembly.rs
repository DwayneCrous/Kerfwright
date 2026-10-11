//! Assemblies in the Design workspace: components (parts with named connection frames), joints between
//! them, motion, a check for interference, and a view of the posed assembly.
//!
//! The assembly is stored in the project file (`assembly.cbor`); the window below edits it.

use super::{geom, DesignWorkspace};
use crate::chrome::theme::{BAD, TEXT_WEAK};
use crate::model::{param_text, parse_param};
use crate::project::Project;
use eframe::egui;
use kw_assembly::{Assembly, Frame, Interference, Joint, JointEnd, JointKind, Part, Pose};
use kw_brep::BRep;
use kw_expr::{Dim, Param};
use kw_feature::{Engine, Hasher128};
use std::sync::Arc;

/// Colours of the components in the posed view.
const PALETTE: [[u8; 3]; 8] = [
    [196, 200, 208],
    [120, 170, 220],
    [220, 160, 110],
    [140, 200, 150],
    [210, 130, 160],
    [190, 180, 100],
    [150, 140, 210],
    [110, 190, 190],
];

/// A file the assembly wants the application to ask for (the dialogs live in the application).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FileRequest {
    /// Choose a `.kwf` to insert as a component.
    Insert,
    /// Choose where to write the assembly as STEP.
    ExportStep,
    /// Choose where to write the flat pattern as DXF.
    ExportDxf,
    /// File ▸ Import.
    Import,
    /// File ▸ Export in this format.
    Export(crate::exchange::ExportFormat),
}

/// The joint being defined in the window.
#[derive(Debug, Clone, PartialEq)]
pub struct JointForm {
    pub kind: JointKind,
    pub parent: usize,
    pub parent_frame: usize,
    pub child: usize,
    pub child_frame: usize,
}

impl Default for JointForm {
    fn default() -> Self {
        JointForm {
            kind: JointKind::Rigid,
            parent: 0,
            parent_frame: 0,
            child: 1,
            child_frame: 0,
        }
    }
}

struct PoseCache {
    key: u128,
    /// The posed assembly, or why it could not be posed.
    pose: Result<Pose, String>,
    rep: Arc<BRep>,
    colours: Vec<[u8; 3]>,
}

pub struct AssemblyUi {
    pub open: bool,
    /// Show the posed assembly in the viewport instead of the design.
    pub view: bool,
    engine: Engine,
    cache: Option<PoseCache>,
    pub new_name: String,
    /// Frames for the next new component besides its origin.
    pub new_frames: Vec<Frame>,
    pub joint: JointForm,
    pub request: Option<FileRequest>,
    pub overlaps: Option<Result<Vec<Interference>, String>>,
    pub message: String,
}

impl Default for AssemblyUi {
    fn default() -> Self {
        AssemblyUi {
            open: false,
            view: false,
            engine: Engine::new(),
            cache: None,
            new_name: "Part".into(),
            new_frames: Vec::new(),
            joint: JointForm::default(),
            request: None,
            overlaps: None,
            message: String::new(),
        }
    }
}

fn kind_name(k: JointKind) -> &'static str {
    match k {
        JointKind::Rigid => "Rigid",
        JointKind::Revolute => "Revolute",
        JointKind::Slider => "Slider",
        JointKind::Cylindrical => "Cylindrical",
        JointKind::Planar => "Planar",
    }
}

const KINDS: [JointKind; 5] = [
    JointKind::Rigid,
    JointKind::Revolute,
    JointKind::Slider,
    JointKind::Cylindrical,
    JointKind::Planar,
];

/// Default values of a new joint: zero of the right dimension for each degree of freedom.
fn zero_values(k: JointKind) -> Vec<Param> {
    match k {
        JointKind::Rigid => vec![],
        JointKind::Revolute => vec![Param::angle_deg(0.0)],
        JointKind::Slider => vec![Param::length(0.0)],
        JointKind::Cylindrical => vec![Param::angle_deg(0.0), Param::length(0.0)],
        JointKind::Planar => vec![
            Param::length(0.0),
            Param::length(0.0),
            Param::angle_deg(0.0),
        ],
    }
}

fn dims(k: JointKind) -> &'static [Dim] {
    match k {
        JointKind::Rigid => &[],
        JointKind::Revolute => &[Dim::ANGLE],
        JointKind::Slider => &[Dim::LENGTH],
        JointKind::Cylindrical => &[Dim::ANGLE, Dim::LENGTH],
        JointKind::Planar => &[Dim::LENGTH, Dim::LENGTH, Dim::ANGLE],
    }
}

/// Take instance `i` out of the assembly with the joints that touch it; later instances move up.
pub fn remove_instance(asm: &mut Assembly, i: usize) {
    if i >= asm.instances.len() {
        return;
    }
    asm.joints
        .retain(|j| j.parent.instance != i && j.child.instance != i);
    for j in &mut asm.joints {
        for e in [&mut j.parent, &mut j.child] {
            if e.instance > i {
                e.instance -= 1;
            }
        }
    }
    asm.instances.remove(i);
    if asm.ground > i || asm.ground >= asm.instances.len() {
        asm.ground = asm.ground.saturating_sub(1);
    }
}

/// A connection frame on a planar face: at the middle of the face's bounding box, z along the outward
/// normal.
pub fn face_connection_frame(name: &str, brep: &BRep, face: kw_brep::FaceId) -> Option<Frame> {
    let f = geom::face_frame(brep, face)?;
    let bb = brep.face_bbox(face)?;
    let c = [
        (bb.min.x + bb.max.x) / 2.0,
        (bb.min.y + bb.max.y) / 2.0,
        (bb.min.z + bb.max.z) / 2.0,
    ];
    // the middle of the box, moved onto the face's plane
    let d = (c[0] - f.origin[0]) * f.n[0]
        + (c[1] - f.origin[1]) * f.n[1]
        + (c[2] - f.origin[2]) * f.n[2];
    let o = [c[0] - d * f.n[0], c[1] - d * f.n[1], c[2] - d * f.n[2]];
    Some(Frame::new(name, o, f.n, f.x))
}

impl DesignWorkspace {
    // ------------------------------------------------------------------------------- editing

    /// Make the current design a component (a new instance of a new part) of the project's assembly.
    pub fn asm_new_component(&mut self, project: &mut Project) -> Result<usize, String> {
        let doc = (*project.document()).clone();
        if project.evaluation().final_solid().is_none() {
            return Err("the design has no solid to make a component of".into());
        }
        let name = self.asm.new_name.trim().to_string();
        let name = if name.is_empty() { "Part".into() } else { name };
        let mut frames = vec![Frame::new(
            "origin",
            [0.0; 3],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        )];
        frames.append(&mut self.asm.new_frames);
        let asm = project
            .assembly
            .get_or_insert_with(|| Assembly::new(&project_name(&doc)));
        let part = asm.add_part(Part {
            name: name.clone(),
            document: doc,
            frames,
        });
        let inst = asm.add_instance(&name, part);
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        Ok(inst)
    }

    /// Insert the design in the `.kwf` file at `path` as a component.
    pub fn asm_insert_path(
        &mut self,
        project: &mut Project,
        path: &std::path::Path,
    ) -> Result<usize, String> {
        let file = kw_feature::kwf::KwfFile::load(path).map_err(|e| e.to_string())?;
        let name = path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Part".into());
        let frames = vec![Frame::new(
            "origin",
            [0.0; 3],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        )];
        let asm = project.assembly.get_or_insert_with(|| Assembly::new(&name));
        let part = asm.add_part(Part {
            name: name.clone(),
            document: file.document,
            frames,
        });
        let inst = asm.add_instance(&name, part);
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        Ok(inst)
    }

    /// The part names and frame names the joint form can choose from.
    fn instance_frames(asm: &Assembly, inst: usize) -> Vec<String> {
        asm.instances
            .get(inst)
            .and_then(|i| asm.parts.get(i.part))
            .map(|p| p.frames.iter().map(|f| f.name.clone()).collect())
            .unwrap_or_default()
    }

    /// Add the joint described by the form.
    pub fn asm_add_joint(&mut self, project: &mut Project) -> Result<(), String> {
        let form = self.asm.joint.clone();
        let asm = project.assembly.as_mut().ok_or("add components first")?;
        if form.parent == form.child {
            return Err("a joint connects two different components".into());
        }
        let pf = Self::instance_frames(asm, form.parent);
        let cf = Self::instance_frames(asm, form.child);
        let (Some(pfn), Some(cfn)) = (pf.get(form.parent_frame), cf.get(form.child_frame)) else {
            return Err("choose a frame on each component".into());
        };
        let n = asm.joints.len() + 1;
        let kind = form.kind;
        asm.add_joint(Joint {
            name: format!("{} {n}", kind_name(kind)),
            kind,
            parent: JointEnd {
                instance: form.parent,
                frame: pfn.clone(),
            },
            child: JointEnd {
                instance: form.child,
                frame: cfn.clone(),
            },
            values: zero_values(kind),
            limits: vec![None; kind_dof(kind)],
        });
        // the form must still produce a solvable tree: take the joint back if it does not
        if let Err(e) = asm.solve() {
            asm.joints.pop();
            return Err(e.to_string());
        }
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        Ok(())
    }

    // ----------------------------------------------------------------------------- posing

    fn asm_refresh(&mut self, project: &Project) {
        let Some(asm) = project.assembly.as_ref() else {
            self.asm.cache = None;
            return;
        };
        let key = {
            let mut h = Hasher128::new();
            h.bytes(&asm.to_bytes());
            h.finish()
        };
        if self.asm.cache.as_ref().is_some_and(|c| c.key == key) {
            return;
        }
        let pose = asm.pose(&mut self.asm.engine).map_err(|e| e.to_string());
        let (rep, colours) = match &pose {
            Ok(p) => (
                Arc::new(p.rep.clone()),
                (0..p.bodies.len())
                    .map(|i| PALETTE[i % PALETTE.len()])
                    .collect(),
            ),
            Err(_) => (Arc::new(BRep::new()), Vec::new()),
        };
        self.asm.cache = Some(PoseCache {
            key,
            pose,
            rep,
            colours,
        });
    }

    /// What the viewport draws while the assembly view is on: `(key, bodies, colours)`.
    pub fn asm_display(&mut self, project: &Project) -> Option<(u128, Arc<BRep>, Vec<[u8; 3]>)> {
        if !self.asm.view {
            return None;
        }
        self.asm_refresh(project);
        let c = self.asm.cache.as_ref()?;
        c.pose.as_ref().ok()?;
        Some((c.key ^ 0xA55E_0000, c.rep.clone(), c.colours.clone()))
    }

    /// Check the posed assembly for overlapping components.
    pub fn asm_check_interference(&mut self, project: &Project) {
        self.asm_refresh(project);
        self.asm.overlaps = Some(match self.asm.cache.as_mut().map(|c| &mut c.pose) {
            Some(Ok(p)) => p.interference(1e-6).map_err(|e| e.to_string()),
            Some(Err(e)) => Err(e.clone()),
            None => Err("there is no assembly yet".into()),
        });
    }

    /// The posed assembly as STEP text (what Export writes).
    pub fn asm_step(&mut self, project: &Project) -> Result<String, String> {
        self.asm_refresh(project);
        match self.asm.cache.as_ref().map(|c| &c.pose) {
            Some(Ok(p)) => Ok(p.to_step()),
            Some(Err(e)) => Err(e.clone()),
            None => Err("there is no assembly yet".into()),
        }
    }

    // ------------------------------------------------------------------------------- window

    pub fn assembly_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        if !self.asm.open {
            return;
        }
        let mut open = true;
        let mut asm = project.assembly.clone();
        let before = asm.clone();
        // faces picked in the view become frames of the next new component
        let picked_face = self.model.picks.iter().rev().find_map(|p| match p {
            crate::viewport::Pick::Face(f) => self
                .bodies
                .as_ref()
                .and_then(|b| face_connection_frame("F", &b.brep, *f)),
            _ => None,
        });
        let mut do_new = false;
        let mut do_joint = false;
        let mut do_check = false;
        egui::Window::new(egui::RichText::new("ASSEMBLY").size(11.5).strong())
            .open(&mut open)
            .default_pos(egui::pos2(330.0, 120.0))
            .default_width(340.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().max_height(560.0).show(ui, |ui| {
                    ui.checkbox(&mut self.asm.view, "Show the assembly in the view")
                        .on_hover_text("Draws every component at its solved position");
                    ui.separator();
                    // ---- components
                    ui.label(egui::RichText::new("Components").strong());
                    match asm.as_mut() {
                        None => {
                            ui.label(
                                egui::RichText::new("None yet: make the design a component, or insert a file.")
                                    .color(TEXT_WEAK)
                                    .size(10.5),
                            );
                        }
                        Some(a) => {
                            let mut remove = None;
                            let mut copy = None;
                            for i in 0..a.instances.len() {
                                ui.horizontal(|ui| {
                                    if ui.radio(a.ground == i, "").on_hover_text("Grounded: it does not move").clicked() {
                                        a.ground = i;
                                    }
                                    ui.add(
                                        egui::TextEdit::singleline(&mut a.instances[i].name)
                                            .desired_width(130.0),
                                    );
                                    ui.label(
                                        egui::RichText::new(
                                            a.parts
                                                .get(a.instances[i].part)
                                                .map(|p| p.name.as_str())
                                                .unwrap_or("?"),
                                        )
                                        .color(TEXT_WEAK)
                                        .size(10.5),
                                    );
                                    if ui.small_button("Copy").on_hover_text("Another instance of the same part").clicked() {
                                        copy = Some(i);
                                    }
                                    if ui.small_button("x").on_hover_text("Remove with its joints").clicked() {
                                        remove = Some(i);
                                    }
                                });
                            }
                            if let Some(i) = copy {
                                let (n, p) = (a.instances[i].name.clone(), a.instances[i].part);
                                a.add_instance(&format!("{n} copy"), p);
                            }
                            if let Some(i) = remove {
                                remove_instance(a, i);
                            }
                        }
                    }
                    ui.horizontal(|ui| {
                        ui.add(egui::TextEdit::singleline(&mut self.asm.new_name).desired_width(110.0));
                        if ui
                            .button("Make this design a component")
                            .on_hover_text("The solid you are designing becomes a part; frames you add below are its connection points")
                            .clicked()
                        {
                            do_new = true;
                        }
                    });
                    ui.horizontal(|ui| {
                        let can = picked_face.is_some();
                        if ui
                            .add_enabled(can, egui::Button::new("Add frame from picked face"))
                            .on_hover_text("A connection frame on the planar face you clicked (its z axis points out)")
                            .clicked()
                        {
                            if let Some(f) = &picked_face {
                                let n = self.asm.new_frames.len() + 1;
                                let mut f = f.clone();
                                f.name = format!("F{n}");
                                self.asm.new_frames.push(f);
                            }
                        }
                        ui.label(
                            egui::RichText::new(format!(
                                "{} frame{} + origin",
                                self.asm.new_frames.len(),
                                if self.asm.new_frames.len() == 1 { "" } else { "s" }
                            ))
                            .color(TEXT_WEAK)
                            .size(10.5),
                        );
                    });
                    if ui.button("Insert a component from a file…").clicked() {
                        self.asm.request = Some(FileRequest::Insert);
                    }
                    // ---- frames of every part
                    if let Some(a) = asm.as_mut() {
                        egui::CollapsingHeader::new("Frames of the parts").show(ui, |ui| {
                            for (pi, p) in a.parts.iter_mut().enumerate() {
                                ui.label(egui::RichText::new(&p.name).strong().size(11.0));
                                let mut remove = None;
                                for (fi, f) in p.frames.iter_mut().enumerate() {
                                    ui.horizontal(|ui| {
                                        ui.add(egui::TextEdit::singleline(&mut f.name).desired_width(70.0));
                                        for (k, c) in f.origin.iter_mut().enumerate() {
                                            ui.add(egui::DragValue::new(c).speed(0.1).prefix(["x ", "y ", "z "][k]));
                                        }
                                        if fi > 0 && ui.small_button("x").clicked() {
                                            remove = Some(fi);
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.add_space(78.0);
                                        for (k, c) in f.z.iter_mut().enumerate() {
                                            ui.add(egui::DragValue::new(c).speed(0.01).prefix(["zx ", "zy ", "zz "][k]));
                                        }
                                    });
                                    ui.push_id((pi, fi), |_| {});
                                }
                                if let Some(fi) = remove {
                                    p.frames.remove(fi);
                                }
                            }
                        });
                    }
                    ui.separator();
                    // ---- joints
                    ui.label(egui::RichText::new("Joints").strong());
                    if let Some(a) = asm.as_mut() {
                        let mut remove = None;
                        for ji in 0..a.joints.len() {
                            let (pn, cn) = (
                                a.instances.get(a.joints[ji].parent.instance).map(|i| i.name.clone()).unwrap_or_default(),
                                a.instances.get(a.joints[ji].child.instance).map(|i| i.name.clone()).unwrap_or_default(),
                            );
                            let j = &mut a.joints[ji];
                            ui.horizontal(|ui| {
                                ui.add(egui::TextEdit::singleline(&mut j.name).desired_width(110.0));
                                ui.label(
                                    egui::RichText::new(format!("{pn}.{} to {cn}.{}", j.parent.frame, j.child.frame))
                                        .color(TEXT_WEAK)
                                        .size(10.5),
                                );
                                if ui.small_button("x").clicked() {
                                    remove = Some(ji);
                                }
                            });
                            let ds = dims(j.kind);
                            for (vi, dim) in ds.iter().enumerate() {
                                let id = ui.make_persistent_id(("joint_value", ji, vi));
                                let mut text: String = ui
                                    .data_mut(|d| d.get_temp::<String>(id))
                                    .unwrap_or_else(|| param_text(&j.values[vi]));
                                ui.horizontal(|ui| {
                                    ui.add_space(14.0);
                                    ui.label(
                                        egui::RichText::new(match (j.kind, vi) {
                                            (JointKind::Revolute, _) => "angle",
                                            (JointKind::Slider, _) => "distance",
                                            (JointKind::Cylindrical, 0) => "angle",
                                            (JointKind::Cylindrical, _) => "distance",
                                            (JointKind::Planar, 0) => "x",
                                            (JointKind::Planar, 1) => "y",
                                            _ => "angle",
                                        })
                                        .color(TEXT_WEAK)
                                        .size(10.5),
                                    );
                                    let r = ui.add(egui::TextEdit::singleline(&mut text).desired_width(80.0));
                                    if r.changed() {
                                        if let Ok(p) = parse_param(&text, *dim) {
                                            j.values[vi] = p;
                                        }
                                    }
                                    if r.has_focus() {
                                        ui.data_mut(|d| d.insert_temp(id, text.clone()));
                                    } else {
                                        ui.data_mut(|d| d.remove::<String>(id));
                                    }
                                    // a slider over the limit (or a turn / 100 mm) moves the joint
                                    if let Param::Lit(q) = &j.values[vi] {
                                        let (lo, hi) = j
                                            .limits
                                            .get(vi)
                                            .copied()
                                            .flatten()
                                            .unwrap_or(if *dim == Dim::ANGLE {
                                                (-std::f64::consts::PI, std::f64::consts::PI)
                                            } else {
                                                (-100.0, 100.0)
                                            });
                                        let mut v = q.value;
                                        let changed = if *dim == Dim::ANGLE {
                                            let mut deg = v.to_degrees();
                                            let c = ui
                                                .add(egui::Slider::new(&mut deg, lo.to_degrees()..=hi.to_degrees()).show_value(false))
                                                .changed();
                                            v = deg.to_radians();
                                            c
                                        } else {
                                            ui.add(egui::Slider::new(&mut v, lo..=hi).show_value(false))
                                                .changed()
                                        };
                                        if changed {
                                            j.values[vi] = if *dim == Dim::ANGLE {
                                                Param::angle_deg(v.to_degrees())
                                            } else {
                                                Param::length(v)
                                            };
                                            ui.data_mut(|d| d.remove::<String>(id));
                                        }
                                    }
                                });
                            }
                        }
                        if let Some(ji) = remove {
                            a.joints.remove(ji);
                        }
                        // ---- new joint
                        ui.add_space(4.0);
                        let n = a.instances.len();
                        if n >= 2 {
                            let f = &mut self.asm.joint;
                            f.parent = f.parent.min(n - 1);
                            f.child = f.child.min(n - 1);
                            ui.horizontal(|ui| {
                                egui::ComboBox::from_id_salt("joint_kind")
                                    .selected_text(kind_name(f.kind))
                                    .show_ui(ui, |ui| {
                                        for k in KINDS {
                                            ui.selectable_value(&mut f.kind, k, kind_name(k));
                                        }
                                    });
                            });
                            for (label, inst, fr) in [
                                ("Parent", &mut f.parent, &mut f.parent_frame),
                                ("Child", &mut f.child, &mut f.child_frame),
                            ] {
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new(label).color(TEXT_WEAK).size(10.5));
                                    egui::ComboBox::from_id_salt(("joint_inst", label))
                                        .selected_text(a.instances[*inst].name.clone())
                                        .show_ui(ui, |ui| {
                                            for (i, ins) in a.instances.iter().enumerate() {
                                                ui.selectable_value(inst, i, &ins.name);
                                            }
                                        });
                                    let frames = Self::instance_frames(a, *inst);
                                    *fr = (*fr).min(frames.len().saturating_sub(1));
                                    egui::ComboBox::from_id_salt(("joint_frame", label))
                                        .selected_text(frames.get(*fr).cloned().unwrap_or_default())
                                        .show_ui(ui, |ui| {
                                            for (i, n) in frames.iter().enumerate() {
                                                ui.selectable_value(fr, i, n);
                                            }
                                        });
                                });
                            }
                            if ui.button("Add joint").clicked() {
                                do_joint = true;
                            }
                        } else {
                            ui.label(
                                egui::RichText::new("Joints need two components.")
                                    .color(TEXT_WEAK)
                                    .size(10.5),
                            );
                        }
                    }
                    ui.separator();
                    // ---- checks and export
                    ui.horizontal(|ui| {
                        if ui.button("Check interference").clicked() {
                            do_check = true;
                        }
                        if ui.button("Export STEP…").clicked() {
                            self.asm.request = Some(FileRequest::ExportStep);
                        }
                    });
                    match &self.asm.overlaps {
                        Some(Ok(v)) if v.is_empty() => {
                            ui.label("No components overlap.");
                        }
                        Some(Ok(v)) => {
                            for o in v {
                                ui.label(format!("{} and {} share {:.3} mm³", o.a, o.b, o.volume));
                            }
                        }
                        Some(Err(e)) => {
                            ui.colored_label(BAD, e);
                        }
                        None => {}
                    }
                    if let Some(c) = &self.asm.cache {
                        match &c.pose {
                            Ok(p) => {
                                if let Some((v, cm)) = p.mass() {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Volume {:.2} mm³, centre ({:.2}, {:.2}, {:.2})",
                                            v, cm.x, cm.y, cm.z
                                        ))
                                        .color(TEXT_WEAK)
                                        .size(10.5),
                                    );
                                }
                            }
                            Err(e) => {
                                ui.colored_label(BAD, e);
                            }
                        }
                    }
                    if !self.asm.message.is_empty() {
                        ui.colored_label(BAD, &self.asm.message);
                    }
                });
            });
        if asm != before {
            project.assembly = asm;
            project.mark_extras_dirty();
            self.asm.overlaps = None;
        }
        self.asm.message.clear();
        if do_new {
            if let Err(e) = self.asm_new_component(project) {
                self.asm.message = e;
            }
        }
        if do_joint {
            if let Err(e) = self.asm_add_joint(project) {
                self.asm.message = e;
            }
        }
        if do_check {
            self.asm_check_interference(project);
        }
        // the pose is refreshed for the readout (cheap when nothing changed)
        self.asm_refresh(project);
        if !open {
            self.asm.open = false;
        }
    }
}

fn kind_dof(k: JointKind) -> usize {
    dims(k).len()
}

fn project_name(doc: &kw_feature::Document) -> String {
    let _ = doc;
    "Assembly".into()
}
