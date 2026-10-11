//! Assemblies in the Design workspace: components (parts with named connection frames), joints between
//! them, motion, a check for interference, and a view of the posed assembly.
//!
//! The assembly is stored in the project file (`assembly.cbor`); the window below edits it.

use super::{geom, DesignWorkspace};
use crate::chrome::theme::{BAD, TEXT_WEAK};
use crate::model::param_text;
use crate::project::Project;
use eframe::egui;
use kw_assembly::{Assembly, Frame, Interference, Joint, JointEnd, JointKind, Part, Pose};
use kw_brep::BRep;
use kw_expr::{Dim, Param};
use kw_feature::{Engine, Hasher128};
use std::collections::HashMap;
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

fn mix(a: [u8; 3], b: [u8; 3], t: f32) -> [u8; 3] {
    let m = |x: u8, y: u8| {
        (x as f32 + (y as f32 - x as f32) * t)
            .round()
            .clamp(0.0, 255.0) as u8
    };
    [m(a[0], b[0]), m(a[1], b[1]), m(a[2], b[2])]
}

/// A file the assembly wants the application to ask for (the dialogs live in the application).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FileRequest {
    /// Choose a `.kwf` to insert as a component.
    Insert,
    /// Choose where to write the assembly as STEP.
    ExportStep,
    /// Choose where to write the flat pattern as DXF.
    ExportDxf,
    /// Choose where to write the bill of materials as CSV.
    ExportBom,
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
    /// Which top-level instance each body of `rep` belongs to.
    tops: Vec<usize>,
    /// Which body (index into the pose) every face / edge of `rep` belongs to.
    face_body: HashMap<kw_brep::FaceId, usize>,
    edge_body: HashMap<kw_brep::EdgeId, usize>,
    /// Interference has been checked for this pose.
    checked: bool,
}

/// A command on the assembly from the browser tree, a menu or a dialog.
#[derive(Debug, Clone, PartialEq)]
pub enum AsmAction {
    /// Select an instance; `true` adds it to (or takes it from) the selection.
    Select(usize, bool),
    SelectJoint(usize),
    ToggleHidden(usize),
    /// Show only this component.
    Isolate(usize),
    ShowAll,
    ToggleGround(usize),
    /// A new instance of the same part, next to the first.
    Copy(usize),
    Delete(usize),
    DeleteJoint(usize),
    DeleteGroup(usize),
    RigidGroupFromSelection,
    SubAssemblyFromSelection,
    Rename(usize, String),
    RenameJoint(usize, String),
    /// The Joint command (`true`: As-Built).
    OpenJoint(bool),
    OpenRigid,
    OpenBom,
    CheckInterference,
    /// The active (or only) body of the design becomes a component.
    ComponentFromActiveBody,
    /// Open the selected component for editing.
    ActivateSelected,
    /// Ground the selected components, or release them when they are all grounded.
    GroundSelected,
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
    /// The selected instances (indices of the assembly's instances) and joint.
    pub selected: Vec<usize>,
    pub selected_joint: Option<usize>,
    /// The instance the pointer is over (in the tree or in the view).
    pub hover: Option<usize>,
    /// Renaming an instance (index, text so far) or a joint.
    pub rename: Option<(usize, String)>,
    pub rename_joint: Option<(usize, String)>,
    /// The joint whose properties window is open, and the text boxes of it.
    pub edit_joint: Option<usize>,
    pub edit: Option<super::asm_joint::JointEdit>,
    /// The Joint / As-Built Joint command, while it is open.
    pub dialog: Option<super::asm_joint::JointDialog>,
    /// The Rigid Group command: which components are ticked.
    pub rigid: Option<Vec<bool>>,
    /// The interference window, the outlines of the overlaps (world coordinates), and whether to re-check
    /// after every change.
    pub interference_open: bool,
    pub overlap_edges: Vec<Vec<[f64; 3]>>,
    pub live_check: bool,
    pub bom_open: bool,
    /// The component drag in progress, and the component under the pointer in the view.
    pub drag: Option<kw_assembly::Drag>,
    pub view_hover: Option<usize>,
    /// The Move command, while it is open.
    pub move_form: Option<super::asm_drag::MoveForm>,
    /// The assembly as it was when the pointer last went down (what Undo returns to after a slider or a
    /// field was changed).
    pub press_snapshot: Option<Option<Vec<u8>>>,
}

impl AssemblyUi {
    /// The posed assembly of the last refresh, or why it could not be posed.
    pub fn pose_ref(&self) -> Option<&Result<Pose, String>> {
        self.cache.as_ref().map(|c| &c.pose)
    }

    /// The B-rep the view shows (the posed assembly), if there is one.
    pub fn display_rep(&self) -> Option<Arc<BRep>> {
        self.cache
            .as_ref()
            .filter(|c| c.pose.is_ok())
            .map(|c| c.rep.clone())
    }
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
            selected: Vec::new(),
            selected_joint: None,
            hover: None,
            rename: None,
            rename_joint: None,
            edit_joint: None,
            edit: None,
            dialog: None,
            rigid: None,
            interference_open: false,
            overlap_edges: Vec::new(),
            live_check: false,
            bom_open: false,
            drag: None,
            view_hover: None,
            move_form: None,
            press_snapshot: None,
        }
    }
}

fn kind_name(k: JointKind) -> &'static str {
    k.name()
}

/// Take instance `i` out of the assembly with the joints that touch it; later instances move up.
pub fn remove_instance(asm: &mut Assembly, i: usize) {
    asm.remove_instance(i);
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
        project.asm_checkpoint();
        let mut frames = vec![Frame::new(
            "origin",
            [0.0; 3],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        )];
        frames.append(&mut self.asm.new_frames);
        let asm = Self::assembly_of(project);
        let part = asm.add_part_unique(Part::new(&name, doc, frames));
        let inst_name = asm.unique_instance_name(&asm.parts[part].name.clone());
        let inst = asm.add_instance(&inst_name, part);
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        Ok(inst)
    }

    /// Note the assembly when the pointer goes down, so a change made by a slider or a field can be undone.
    pub(super) fn asm_note_press(&mut self, ctx: &egui::Context, project: &Project) {
        if ctx.input(|i| i.pointer.primary_pressed()) {
            self.asm.press_snapshot = Some(project.assembly.as_ref().map(|a| a.to_bytes()));
        }
    }

    /// Make the state noted at the last press the one Undo returns to.
    pub(super) fn asm_commit_snapshot(&mut self, project: &mut Project) {
        if let Some(before) = self.asm.press_snapshot.take() {
            project.asm_checkpoint_with(before);
        }
    }

    /// The assembly of the project, made when the first component is.
    fn assembly_of(project: &mut Project) -> &mut Assembly {
        let name = project.name();
        project.assembly.get_or_insert_with(|| Assembly::new(&name))
    }

    fn origin_frame() -> Frame {
        Frame::new("origin", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    }

    /// A new component with nothing in it, opened for editing (Fusion's New Component).
    pub fn asm_new_empty_component(&mut self, project: &mut Project) -> Result<usize, String> {
        let name = {
            let n = self.asm.new_name.trim();
            if n.is_empty() {
                "Component".to_string()
            } else {
                n.to_string()
            }
        };
        project.asm_checkpoint();
        let asm = Self::assembly_of(project);
        let part = asm.add_part_unique(Part::new(
            &name,
            kw_feature::Document::new(),
            vec![Self::origin_frame()],
        ));
        let part_name = asm.parts[part].name.clone();
        let inst_name = asm.unique_instance_name(&part_name);
        let inst = asm.add_instance(&inst_name, part);
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        self.activate_component(project, part)?;
        Ok(inst)
    }

    /// The body ending at feature `head` becomes a component: a copy of the features that build it. The
    /// body is hidden in the design so it is not drawn twice.
    pub fn asm_component_from_body(
        &mut self,
        project: &mut Project,
        head: kw_feature::FeatureId,
    ) -> Result<usize, String> {
        if project.editing().is_some() {
            return Err("finish editing the component first".into());
        }
        let doc = project.document();
        let part_doc = doc.extract_body(head).map_err(|e| e.to_string())?;
        let root = doc.body_root(head);
        let name = crate::bodies::summaries(&doc)
            .into_iter()
            .find(|b| b.head == head)
            .map(|b| b.name)
            .unwrap_or_else(|| doc.name_of(head));
        project.asm_checkpoint();
        let asm = Self::assembly_of(project);
        let part = asm.add_part_unique(Part::new(&name, part_doc, vec![Self::origin_frame()]));
        let part_name = asm.parts[part].name.clone();
        let inst_name = asm.unique_instance_name(&part_name);
        let inst = asm.add_instance(&inst_name, part);
        self.hidden_bodies.insert(root);
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        self.status = format!("'{name}' is now a component.");
        Ok(inst)
    }

    /// Build the scene of a screenshot: the design's body as a component, a copy of it, and (by `stage`)
    /// a joint made from two clicked faces, the Joint command half-way, or the bill of materials.
    pub fn asm_demo(&mut self, project: &mut Project, stage: &str) -> Result<(), String> {
        use super::asm_joint::{Anchor, JointDialog};
        let doc = project.document();
        let head = crate::bodies::summaries(&doc)
            .first()
            .map(|b| b.head)
            .ok_or("the design has no body")?;
        self.asm_component_from_body(project, head)?;
        self.asm_action(project, AsmAction::Copy(0));
        self.asm.view = true;
        self.asm_refresh(project);
        let rep = self.asm.display_rep().ok_or("the assembly does not pose")?;
        // a flat face of body `i` at height `z` facing up or down
        let face_of = |i: usize, z: f64, up: bool| -> Option<crate::viewport::Pick> {
            let body = rep.bodies.keys().nth(i)?;
            rep.body_faces(body)
                .into_iter()
                .find(|f| {
                    geom::face_frame(&rep, *f)
                        .is_some_and(|fr| fr.n[2].abs() > 0.99 && (fr.n[2] > 0.0) == up)
                        && rep.face_bbox(*f).is_some_and(|bb| {
                            (bb.min.z - z).abs() < 1e-6 && (bb.max.z - z).abs() < 1e-6
                        })
                })
                .map(crate::viewport::Pick::Face)
        };
        let top = rep
            .bodies
            .keys()
            .next()
            .and_then(|b| {
                rep.body_faces(b)
                    .into_iter()
                    .filter_map(|f| rep.face_bbox(f))
                    .map(|bb| bb.max.z)
                    .reduce(f64::max)
            })
            .ok_or("no height")?;
        let (bottom_copy, top_first) = (
            face_of(1, 0.0, false).ok_or("no underside")?,
            face_of(0, top, true).ok_or("no top face")?,
        );
        let mut dlg = JointDialog::new(false);
        dlg.kind = JointKind::Revolute;
        for (inst, pick) in [(1, bottom_copy), (0, top_first)] {
            let origin = super::asm_snap::candidates(&rep, pick)
                .into_iter()
                .next()
                .ok_or("no joint origin")?;
            dlg.set_anchor(Anchor {
                instance: inst,
                origin,
            });
        }
        match stage {
            "assembly-joint" => {
                self.asm.dialog = Some(dlg);
            }
            _ => {
                self.asm_joint_commit(project, &dlg)?;
                if let Some(j) = project.assembly.as_mut().and_then(|a| a.joints.first_mut()) {
                    j.values[0] = Param::angle_deg(35.0);
                }
                self.asm_action(project, AsmAction::Copy(0));
                self.asm.selected_joint = Some(0);
                self.asm.selected.clear();
                if stage == "assembly-bom" {
                    self.asm.bom_open = true;
                } else {
                    self.asm_run_interference(project);
                }
            }
        }
        Ok(())
    }

    /// Open component `part` for editing in the Design workspace.
    pub fn activate_component(&mut self, project: &mut Project, part: usize) -> Result<(), String> {
        project.activate_component(part)?;
        self.switch_document();
        // the component is edited in the design view; the assembly comes back when it is finished
        self.asm.view = false;
        let name = project
            .assembly
            .as_ref()
            .and_then(|a| a.parts.get(part))
            .map(|p| p.name.clone())
            .unwrap_or_default();
        self.status = format!("Editing {name}: Finish Component returns to the assembly.");
        Ok(())
    }

    /// Close the component being edited.
    pub fn finish_component(&mut self, project: &mut Project) -> Result<(), String> {
        let part = project.finish_component()?;
        self.switch_document();
        self.asm.view = true;
        let name = project
            .assembly
            .as_ref()
            .and_then(|a| a.parts.get(part))
            .map(|p| p.name.clone())
            .unwrap_or_default();
        self.status = format!("{name} saved in the assembly.");
        self.asm.overlaps = None;
        Ok(())
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
        project.asm_checkpoint();
        let asm = project.assembly.get_or_insert_with(|| Assembly::new(&name));
        let part = asm.add_part_unique(Part::new(&name, file.document, frames));
        let inst_name = asm.unique_instance_name(&asm.parts[part].name.clone());
        let inst = asm.add_instance(&inst_name, part);
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
        let before = asm.to_bytes();
        asm.add_joint(Joint::new(
            &format!("{} {n}", kind_name(kind)),
            kind,
            JointEnd {
                instance: form.parent,
                frame: pfn.clone(),
            },
            JointEnd {
                instance: form.child,
                frame: cfn.clone(),
            },
        ));
        // the form must still produce a solvable tree: take the joint back if it does not
        if let Err(e) = asm.solve() {
            asm.joints.pop();
            return Err(e.to_string());
        }
        project.asm_checkpoint_with(Some(before));
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        Ok(())
    }

    // ----------------------------------------------------------------------------- posing

    pub fn asm_refresh_pub(&mut self, project: &Project) {
        self.asm_refresh(project);
    }

    fn asm_refresh(&mut self, project: &Project) {
        // while the Joint command shows its result, that assembly is the one posed
        let source = self
            .asm
            .dialog
            .as_ref()
            .and_then(|d| d.preview.as_ref())
            .or(project.assembly.as_ref());
        let Some(asm) = source else {
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
        let (rep, tops, face_body, edge_body) = match &pose {
            Ok(p) => {
                let mut fb = HashMap::new();
                let mut eb = HashMap::new();
                for (i, map) in p.maps.iter().enumerate() {
                    for f in map.faces.values() {
                        fb.insert(*f, i);
                    }
                    for e in map.edges.values() {
                        eb.insert(*e, i);
                    }
                }
                (Arc::new(p.rep.clone()), p.tops.clone(), fb, eb)
            }
            Err(_) => (
                Arc::new(BRep::new()),
                Vec::new(),
                HashMap::new(),
                HashMap::new(),
            ),
        };
        self.asm.cache = Some(PoseCache {
            key,
            pose,
            rep,
            tops,
            face_body,
            edge_body,
            checked: false,
        });
        self.asm.overlap_edges.clear();
        // the numbers belonged to the old pose (live checking makes them again)
        self.asm.overlaps = None;
    }

    /// The top-level instance and body (index into the pose) a face or edge of the posed view belongs to.
    pub fn asm_pick_owner(&self, pick: crate::viewport::Pick) -> Option<(usize, usize)> {
        let c = self.asm.cache.as_ref()?;
        let body = match pick {
            crate::viewport::Pick::Face(f) => *c.face_body.get(&f)?,
            crate::viewport::Pick::Edge(e) => *c.edge_body.get(&e)?,
        };
        Some((*c.tops.get(body)?, body))
    }

    /// What the viewport draws while the assembly view is on: `(key, bodies, colours)`.
    pub fn asm_display(&mut self, project: &Project) -> Option<(u128, Arc<BRep>, Vec<[u8; 3]>)> {
        if !self.asm.view {
            return None;
        }
        self.asm_refresh(project);
        if self.asm.live_check && self.asm.cache.as_ref().is_some_and(|c| !c.checked) {
            self.asm_run_interference(project);
        }
        let c = self.asm.cache.as_ref()?;
        c.pose.as_ref().ok()?;
        // components keep one colour each; the selected ones turn blue
        let colours: Vec<[u8; 3]> = c
            .tops
            .iter()
            .map(|t| {
                let base = PALETTE[t % PALETTE.len()];
                if self.asm.selected.contains(t) {
                    mix(base, [6, 150, 215], 0.6)
                } else {
                    base
                }
            })
            .collect();
        let mut h = Hasher128::new();
        h.bytes(&c.key.to_le_bytes());
        for col in &colours {
            h.bytes(col);
        }
        Some((h.finish() ^ 0xA55E_0000, c.rep.clone(), colours))
    }

    /// Check the posed assembly for overlapping components.
    pub fn asm_check_interference(&mut self, project: &Project) {
        self.asm_refresh(project);
        if let Some(c) = self.asm.cache.as_mut() {
            c.checked = true;
        }
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

    /// The window behind "Drive Joints": a slider for every degree of freedom, the connection frames of the
    /// parts, and the joint form that works from named frames. Edits are made in place.
    pub fn assembly_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        if !self.asm.open {
            return;
        }
        let mut open = true;
        let mut changed = false;
        let mut do_joint = false;
        let mut edit_joint = None;
        egui::Window::new(egui::RichText::new("DRIVE JOINTS").size(11.5).strong())
            .open(&mut open)
            .default_pos(egui::pos2(330.0, 190.0))
            .default_width(330.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .max_height(560.0)
                    .show(ui, |ui| {
                        ui.checkbox(&mut self.asm.view, "Show the assembly in the view")
                            .on_hover_text("Draws every component at its solved position");
                        ui.separator();
                        let Some(a) = project.assembly.as_mut() else {
                            ui.label(
                                egui::RichText::new(
                                    "No components yet: use ASSEMBLE > New Component.",
                                )
                                .color(TEXT_WEAK)
                                .size(10.5),
                            );
                            return;
                        };
                        // ---- the joints' values
                        if a.joints.is_empty() {
                            ui.label(
                                egui::RichText::new("No joints yet: use ASSEMBLE > Joint.")
                                    .color(TEXT_WEAK)
                                    .size(10.5),
                            );
                        }
                        for ji in 0..a.joints.len() {
                            let (pn, cn) = (
                                a.instances
                                    .get(a.joints[ji].parent.instance)
                                    .map(|i| i.name.clone())
                                    .unwrap_or_default(),
                                a.instances
                                    .get(a.joints[ji].child.instance)
                                    .map(|i| i.name.clone())
                                    .unwrap_or_default(),
                            );
                            let j = &mut a.joints[ji];
                            ui.horizontal(|ui| {
                                ui.label(egui::RichText::new(&j.name).strong().size(11.5));
                                ui.label(
                                    egui::RichText::new(format!("{pn} to {cn}"))
                                        .color(TEXT_WEAK)
                                        .size(10.5),
                                );
                                if ui.small_button("Edit…").clicked() {
                                    edit_joint = Some(ji);
                                }
                            });
                            let names = j.kind.value_names();
                            for (vi, dim) in j.kind.dims().iter().enumerate() {
                                let angle = *dim == Dim::ANGLE;
                                ui.horizontal(|ui| {
                                    ui.add_space(14.0);
                                    ui.label(
                                        egui::RichText::new(names[vi]).color(TEXT_WEAK).size(10.5),
                                    );
                                    let Param::Lit(q) = &j.values[vi] else {
                                        ui.label(
                                            egui::RichText::new(format!(
                                                "= {}",
                                                param_text(&j.values[vi])
                                            ))
                                            .size(10.5),
                                        );
                                        return;
                                    };
                                    let (lo, hi) =
                                        j.limits.get(vi).copied().flatten().unwrap_or(if angle {
                                            (-std::f64::consts::PI, std::f64::consts::PI)
                                        } else {
                                            (-100.0, 100.0)
                                        });
                                    let mut v = q.value;
                                    let moved = if angle {
                                        let mut deg = v.to_degrees();
                                        let c = ui
                                            .add(
                                                egui::Slider::new(
                                                    &mut deg,
                                                    lo.to_degrees()..=hi.to_degrees(),
                                                )
                                                .suffix("°"),
                                            )
                                            .changed();
                                        v = deg.to_radians();
                                        c
                                    } else {
                                        ui.add(egui::Slider::new(&mut v, lo..=hi).suffix(" mm"))
                                            .changed()
                                    };
                                    if moved {
                                        j.values[vi] = if angle {
                                            Param::Lit(kw_expr::Quantity::angle(v))
                                        } else {
                                            Param::length(v)
                                        };
                                        changed = true;
                                    }
                                });
                            }
                        }
                        ui.separator();
                        ui.horizontal(|ui| {
                            if ui.button("Insert component from a file…").clicked() {
                                self.asm.request = Some(FileRequest::Insert);
                            }
                            if ui.button("Export STEP…").clicked() {
                                self.asm.request = Some(FileRequest::ExportStep);
                            }
                        });
                        // ---- connection frames of every part
                        egui::CollapsingHeader::new("Frames of the parts").show(ui, |ui| {
                            for (pi, p) in a.parts.iter_mut().enumerate() {
                                ui.label(egui::RichText::new(&p.name).strong().size(11.0));
                                let mut remove = None;
                                for (fi, f) in p.frames.iter_mut().enumerate() {
                                    ui.horizontal(|ui| {
                                        changed |= ui
                                            .add(
                                                egui::TextEdit::singleline(&mut f.name)
                                                    .desired_width(70.0),
                                            )
                                            .changed();
                                        for (k, c) in f.origin.iter_mut().enumerate() {
                                            changed |= ui
                                                .add(
                                                    egui::DragValue::new(c)
                                                        .speed(0.1)
                                                        .prefix(["x ", "y ", "z "][k]),
                                                )
                                                .changed();
                                        }
                                        if fi > 0 && ui.small_button("x").clicked() {
                                            remove = Some(fi);
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.add_space(78.0);
                                        for (k, c) in f.z.iter_mut().enumerate() {
                                            changed |= ui
                                                .add(
                                                    egui::DragValue::new(c)
                                                        .speed(0.01)
                                                        .prefix(["zx ", "zy ", "zz "][k]),
                                                )
                                                .changed();
                                        }
                                    });
                                    ui.push_id((pi, fi), |_| {});
                                }
                                if let Some(fi) = remove {
                                    p.frames.remove(fi);
                                    changed = true;
                                }
                            }
                        });
                        // ---- a joint between named frames
                        egui::CollapsingHeader::new("Joint between named frames").show(ui, |ui| {
                            let n = a.instances.len();
                            if n < 2 {
                                ui.label(
                                    egui::RichText::new("Joints need two components.")
                                        .color(TEXT_WEAK)
                                        .size(10.5),
                                );
                                return;
                            }
                            let f = &mut self.asm.joint;
                            f.parent = f.parent.min(n - 1);
                            f.child = f.child.min(n - 1);
                            egui::ComboBox::from_id_salt("joint_kind")
                                .selected_text(kind_name(f.kind))
                                .show_ui(ui, |ui| {
                                    for k in JointKind::ALL {
                                        ui.selectable_value(&mut f.kind, k, kind_name(k));
                                    }
                                });
                            for (label, inst, fr) in [
                                ("Parent", &mut f.parent, &mut f.parent_frame),
                                ("Child", &mut f.child, &mut f.child_frame),
                            ] {
                                ui.horizontal(|ui| {
                                    ui.label(
                                        egui::RichText::new(label).color(TEXT_WEAK).size(10.5),
                                    );
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
                        });
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
        if changed {
            self.asm_commit_snapshot(project);
            project.mark_extras_dirty();
            self.asm.overlaps = None;
        }
        if let Some(j) = edit_joint {
            self.asm.edit_joint = Some(j);
        }
        self.asm.message.clear();
        if do_joint {
            if let Err(e) = self.asm_add_joint(project) {
                self.asm.message = e;
            }
        }
        // the pose is refreshed for the readout (cheap when nothing changed)
        self.asm_refresh(project);
        if !open {
            self.asm.open = false;
        }
    }
}
