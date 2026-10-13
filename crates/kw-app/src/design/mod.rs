//! The Design workspace: Sketch and Model as one thing, like Fusion's Design environment. A ribbon on top,
//! the browser at the left, the timeline at the bottom and the 3-D view in the middle; *Create Sketch* picks
//! a plane or a planar face, the view turns into the flat sketch canvas, *Finish Sketch* drops straight into
//! the Extrude dialog with a live preview.

pub mod analysis;
pub mod asm_drag;
pub mod asm_inspect;
pub mod asm_joint;
pub mod asm_link;
pub mod asm_snap;
pub mod asm_tree;
pub mod assembly;
pub mod bodyprops;
pub mod browser;
pub mod continuity;
pub mod form;
pub mod geom;
pub mod measure;
pub mod overlay;
pub mod ribbon;
pub mod section;
pub mod sheetmetal;
pub mod sketch_ops;
pub mod sketch_project;
pub mod timeline;
pub mod tools;

use crate::bodies::Bodies;
use crate::chrome::theme;
use crate::model::ModelWorkspace;
use crate::project::Project;
use crate::sketch::SketchWorkspace;
use crate::viewport::{ModelScene, Pick, ViewOpts, Viewport3d};
use eframe::egui;
use geom::PlaneFrame;
use kw_brep::BRep;
use kw_feature::{BodyOp, Evaluation, FeatureId, FeatureSpec, PlaneSpec};
use std::collections::HashSet;
use std::sync::Arc;
use tools::{BuildCtx, Dialog, Outcome, Tool};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Idle,
    /// Waiting for a plane or planar face to sketch on.
    PickPlane,
    Sketching,
}

/// The regions of the sketch the open Extrude dialog works on: where they are, for drawing and picking them.
pub struct RegionView {
    pub sketch: FeatureId,
    /// Identifies the sketch geometry the regions were computed for.
    pub key: u128,
    pub plane: kw_math::Frame3,
    /// The regions of the sketch (faces of its planar arrangement).
    pub sr: kw_feature::regions::SketchRegions,
    /// The sketch they were made from (to find regions again from saved picks).
    pub product: Arc<kw_feature::SketchProduct>,
}

impl RegionView {
    pub fn regions(&self) -> &[kw_feature::regions::Region] {
        &self.sr.regions
    }

    /// The region a pick names.
    pub fn index_of(&self, pick: &kw_feature::RegionPick) -> Option<usize> {
        self.sr
            .resolve(pick, &self.product.sketch, &self.product.entities)
    }

    /// The picks that choose each region (several picks can name one region).
    pub fn matching(&self, picks: &[kw_feature::RegionPick]) -> Vec<Vec<usize>> {
        let mut out = vec![Vec::new(); self.sr.regions.len()];
        for (k, p) in picks.iter().enumerate() {
            if let Some(i) = self.index_of(p) {
                out[i].push(k);
            }
        }
        out
    }

    /// The picks of the regions that are extruded by default.
    pub fn default_picks(&self) -> Vec<kw_feature::RegionPick> {
        self.sr
            .default_regions()
            .into_iter()
            .map(|i| self.sr.regions[i].pick.clone())
            .collect()
    }
}

/// The live preview of the open dialog.
#[derive(Debug, Clone, Default)]
pub struct Preview {
    pub key: String,
    /// What the preview draws: the bodies of the document with the feature applied.
    pub bodies: Option<Arc<Bodies>>,
    pub brep: Option<Arc<BRep>>,
    pub error: Option<String>,
}

/// Things the panels ask for while they are being drawn; carried out afterwards.
#[derive(Debug, Clone)]
pub enum Action {
    Select(FeatureId),
    Edit(FeatureId),
    Delete(FeatureId),
    Suppress(FeatureId, bool),
    RollbackTo(FeatureId),
    ClearRollback,
    Move(FeatureId, usize),
    Open(Tool),
    StartSketch,
    FinishSketch,
    CancelSketch,
    SketchOn(PlaneSpec, PlaneFrame),
    ToggleSketch(FeatureId),
    /// Make the design a component of the assembly.
    NewComponent,
    /// A new, empty component, opened for editing.
    NewEmptyComponent,
    /// The body ending at this feature becomes a component.
    ComponentFromBody(FeatureId),
    /// Open this part of the assembly for editing in place.
    ActivateComponent(usize),
    /// Close the component being edited.
    FinishComponent,
    /// A command on the assembly (tree, menus, dialogs).
    Asm(assembly::AsmAction),
    /// The flat pattern of the sheet-metal part.
    FlatPattern,
    /// Edit the last feature of this kind (`FeatureSpec::kind_name`).
    EditLastOf(&'static str),
    /// The appearance window of the active (or first) body.
    OpenAppearance,
}

pub struct DesignWorkspace {
    /// The editing commands (add / replace features, parameters, selection, error text).
    pub model: ModelWorkspace,
    pub sketch: SketchWorkspace,
    pub viewport: Viewport3d,
    pub mode: Mode,
    pub dialog: Option<Dialog>,
    pub tab: usize,
    pub hidden_sketches: HashSet<FeatureId>,
    /// Origin planes shown in the 3-D view (XY, XZ, YZ).
    pub plane_visible: [bool; 3],
    pub hover_plane: Option<usize>,
    /// The construction plane under the pointer while picking a sketch plane, with its frame.
    pub hover_datum: Option<(FeatureId, geom::PlaneFrame)>,
    pub preview: Option<Preview>,
    pub status: String,
    pub hole_plane: Option<PlaneSpec>,
    pub timeline: timeline::State,
    pub params_open: bool,
    /// The Measure window is open (picks are compared instead of used by a tool).
    pub measure_open: bool,
    /// The Interference window is open.
    pub interference_open: bool,
    /// For every feature, how many others depend on it (what deleting it takes along).
    pub dep_counts: std::collections::HashMap<FeatureId, usize>,
    /// The flat pattern window (Sheet Metal ▸ Flat Pattern).
    pub flat: Option<sheetmetal::FlatState>,
    /// Which ribbon tab is shown (0 SOLID, 1 SHEET METAL, 2 FORM, 3 TOOLS).
    pub ribbon_tab: usize,
    /// The assembly window and the posed view.
    pub asm: assembly::AssemblyUi,
    /// The section plane (Section Analysis), when it is on.
    pub section: Option<section::SectionState>,
    /// The analysis overlay (zebra, curvature, draft), if one is showing.
    pub analysis: Option<analysis::AnalysisState>,
    /// The edge continuity display, and the edges it last worked out.
    pub continuity: Option<continuity::ContinuityState>,
    pub continuity_cache: Option<continuity::ContinuityCache>,
    /// Index into `measure::materials()`.
    pub material: usize,
    /// The bodies drawn in the view (refreshed every frame), and the roots of the bodies hidden from it.
    pub bodies: Option<Arc<Bodies>>,
    pub hidden_bodies: HashSet<FeatureId>,
    /// Construction planes, axes and points hidden from the view.
    pub hidden_datums: HashSet<FeatureId>,
    /// The plane of the sketch being edited, and the pattern / text panel open over it.
    pub sketch_frame: Option<PlaneFrame>,
    pub sketch_panel: Option<sketch_ops::SketchPanel>,
    /// The body new features go to when no pick says otherwise (a body root).
    pub active_body: Option<FeatureId>,
    /// The body whose properties window (name, colour, material) is open.
    pub body_props: Option<FeatureId>,
    /// The body a sketch about to be started on a face belongs to.
    pub next_sketch_base: Option<FeatureId>,
    /// The regions of the Extrude dialog's sketch (only kept while there is more than one).
    pub region_view: Option<RegionView>,
    pub actions: Vec<Action>,
    /// Timeline position of the viewport's last click (used for double-click handling).
    pub shaded: bool,
}

impl Default for DesignWorkspace {
    fn default() -> Self {
        DesignWorkspace {
            model: ModelWorkspace::default(),
            sketch: SketchWorkspace::default(),
            viewport: Viewport3d::default(),
            mode: Mode::Idle,
            dialog: None,
            tab: 0,
            hidden_sketches: HashSet::new(),
            plane_visible: [false; 3],
            hover_plane: None,
            hover_datum: None,
            preview: None,
            status: String::new(),
            hole_plane: None,
            timeline: timeline::State::default(),
            params_open: false,
            measure_open: false,
            interference_open: false,
            dep_counts: Default::default(),
            flat: None,
            ribbon_tab: 0,
            asm: Default::default(),
            section: None,
            analysis: None,
            continuity: None,
            continuity_cache: None,
            material: 2,
            bodies: None,
            hidden_bodies: HashSet::new(),
            hidden_datums: HashSet::new(),
            sketch_frame: None,
            sketch_panel: None,
            active_body: None,
            body_props: None,
            next_sketch_base: None,
            region_view: None,
            actions: Vec::new(),
            shaded: true,
        }
    }
}

impl DesignWorkspace {
    // ------------------------------------------------------------------------------ queries

    /// A one-line hint for the status bar.
    pub fn status_line(&self) -> Option<String> {
        if !self.model.error.is_empty() {
            return Some(self.model.error.clone());
        }
        (!self.status.is_empty()).then(|| self.status.clone())
    }

    pub fn picks(&self) -> &[Pick] {
        &self.model.picks
    }

    /// The body the next feature works on: the one `active_body` names, else the last one.
    pub(crate) fn target_index(&self) -> Option<usize> {
        let b = self.bodies.as_ref()?;
        self.active_body
            .and_then(|r| b.index_of_root(r))
            .or(Some(b.list.len().checked_sub(1)?))
    }

    // ------------------------------------------------------------------------------- sketching

    /// Enter the sketch mode on `plane` (frame `frame`): a flat canvas with the body's outline as reference.
    pub fn begin_sketch(&mut self, project: &mut Project, plane: PlaneSpec, frame: PlaneFrame) {
        self.sketch.begin_new_on(plane);
        self.sketch.new_base = self.next_sketch_base.take();
        self.sketch.app.reference = Self::reference_geometry(project, &frame);
        self.sketch_frame = Some(frame);
        self.enter_sketching();
    }

    /// The model's edges lying in the sketch plane, as faint reference lines.
    fn reference_geometry(project: &mut Project, frame: &PlaneFrame) -> Vec<Vec<(f64, f64)>> {
        let ev = project.evaluation();
        let doc = project.document();
        match Bodies::new(&doc, &ev, &HashSet::new()) {
            Some(b) => geom::project_edges(&ModelScene::build(&b.brep), frame),
            None => Vec::new(),
        }
    }

    fn enter_sketching(&mut self) {
        self.sketch_panel = None;
        self.mode = Mode::Sketching;
        self.tab = 2;
        self.dialog = None;
        self.preview = None;
        self.model.picks.clear();
        self.status = "Sketch: draw with the tools in the ribbon, then Finish Sketch.".into();
    }

    pub fn edit_sketch(&mut self, project: &mut Project, id: FeatureId) {
        match self.sketch.begin_edit(project, id) {
            Ok(()) => {
                let ev = project.evaluation();
                let frame = match ev.output(id).map(|o| &o.product) {
                    Some(kw_feature::Product::Sketch(sp)) => Some(PlaneFrame {
                        origin: [sp.plane.origin.x, sp.plane.origin.y, sp.plane.origin.z],
                        x: [sp.plane.x.x, sp.plane.x.y, sp.plane.x.z],
                        y: [sp.plane.y.x, sp.plane.y.y, sp.plane.y.z],
                        n: [sp.plane.z.x, sp.plane.z.y, sp.plane.z.z],
                    }),
                    _ => None,
                };
                self.sketch.app.reference = match &frame {
                    Some(f) => Self::reference_geometry(project, f),
                    None => Vec::new(),
                };
                self.sketch_frame = frame;
                self.enter_sketching();
            }
            Err(e) => self.model.error = e,
        }
    }

    pub fn finish_sketch(&mut self, project: &mut Project) {
        let was_new = matches!(self.sketch.editing, crate::sketch::Editing::New(_));
        if let Some(id) = self.sketch.finish(project) {
            self.mode = Mode::Idle;
            self.tab = 0;
            self.model.selected = Some(id);
            self.status.clear();
            if was_new {
                // straight into Extrude, as in Fusion
                let has_body = project.evaluation().final_solid().is_some();
                self.open_tool(
                    project,
                    Tool::extrude(Some(id), if has_body { BodyOp::Join } else { BodyOp::New }),
                );
            }
        } else if !self.sketch.error.is_empty() {
            self.model.error = self.sketch.error.clone();
        }
    }

    pub fn cancel_sketch(&mut self) {
        self.sketch.cancel();
        self.mode = Mode::Idle;
        self.tab = 0;
        self.status.clear();
    }

    // ------------------------------------------------------------------------------------ tools

    pub fn open_tool(&mut self, project: &mut Project, tool: Tool) {
        let doc = project.document();
        if doc.rolled_back() {
            self.model.error =
                "Roll the timeline forward to its end before adding features.".into();
            return;
        }
        let mut tool = tool;
        let last_tool_feature = doc
            .features
            .iter()
            .rev()
            .find(|f| {
                matches!(
                    f.spec,
                    FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
                )
            })
            .map(|f| f.id);
        let sel_tool = self.model.selected.filter(|s| {
            doc.get(*s).is_some_and(|f| {
                matches!(
                    f.spec,
                    FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
                )
            })
        });
        match &mut tool {
            Tool::Pattern { source, .. }
            | Tool::CircularPattern { source, .. }
            | Tool::Mirror { source, .. } => *source = sel_tool.or(last_tool_feature),
            Tool::Combine {
                target, tool: t, ..
            } => {
                // the active body (else the last one) is the target; the tool is left to the user
                let heads = doc.heads();
                *target = self
                    .active_body
                    .and_then(|r| heads.iter().copied().find(|h| doc.body_root(*h) == r))
                    .or(heads.last().copied());
                *t = None;
            }
            Tool::Loft { sketches, .. } => {
                let ids: Vec<FeatureId> = doc
                    .features
                    .iter()
                    .filter(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                    .map(|f| f.id)
                    .collect();
                let from = ids.len().saturating_sub(2);
                *sketches = ids[from..].iter().map(|i| Some(*i)).collect();
                while sketches.len() < 2 {
                    sketches.push(None);
                }
            }
            Tool::Sweep { profile, path, .. } => {
                let ids: Vec<FeatureId> = doc
                    .features
                    .iter()
                    .filter(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                    .map(|f| f.id)
                    .collect();
                if ids.len() >= 2 {
                    *profile = Some(ids[ids.len() - 2]);
                    *path = Some(ids[ids.len() - 1]);
                }
            }
            Tool::Extrude { sketch, .. }
            | Tool::Revolve { sketch, .. }
            | Tool::Rib { sketch, .. }
            | Tool::Pipe { path: sketch, .. }
                if sketch.is_none() =>
            {
                let sel = self.model.selected.filter(|s| {
                    doc.get(*s)
                        .is_some_and(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                });
                *sketch = sel.or_else(|| {
                    doc.features
                        .iter()
                        .rev()
                        .find(|f| matches!(f.spec, FeatureSpec::Sketch(_)))
                        .map(|f| f.id)
                });
            }
            _ => {}
        }
        self.model.picks.clear();
        self.hole_plane = None;
        self.preview = None;
        self.mode = Mode::Idle;
        self.dialog = Some(Dialog {
            tool,
            editing: None,
        });
        self.model.error.clear();
    }

    pub fn open_edit(&mut self, project: &mut Project, id: FeatureId) {
        let doc = project.document();
        let Some(f) = doc.get(id) else { return };
        if matches!(f.spec, FeatureSpec::Sketch(_)) {
            self.edit_sketch(project, id);
            return;
        }
        match Tool::for_feature(&f.spec) {
            Some(tool) => {
                self.model.picks.clear();
                self.hole_plane = None;
                self.preview = None;
                self.dialog = Some(Dialog {
                    tool,
                    editing: Some((id, f.spec.clone())),
                });
                self.model.selected = Some(id);
            }
            None => self.model.error = format!("{} has no editor yet", f.spec.kind_name()),
        }
    }

    /// Build the dialog's feature spec against the current document. The picks decide which body it works
    /// on (all picks must lie on one body); without picks it is the active body, else the last one.
    fn build_spec(&self, project: &mut Project, dlg: &Dialog) -> Result<FeatureSpec, String> {
        // a sketch with several regions needs some of them chosen (the dialog starts with the usual ones)
        if let (Some(rv), Tool::Extrude { more, sketch, .. }) = (&self.region_view, &dlg.tool) {
            if Some(rv.sketch) == *sketch
                && rv.regions().len() > 1
                && !more.to_face
                && more.regions.iter().all(|p| rv.index_of(p).is_none())
            {
                return Err("click the regions of the sketch to extrude".into());
            }
        }
        let doc = project.document();
        let ev = project.evaluation();
        let fallback;
        let bodies: &Bodies = match &self.bodies {
            Some(b) => b,
            None => {
                fallback = Bodies::new(&doc, &ev, &HashSet::new());
                match &fallback {
                    Some(b) => b,
                    None => {
                        let ctx = BuildCtx {
                            doc: &doc,
                            base: None,
                            picks: &[],
                            boundary: &[],
                            picked_bodies: &[],
                            faces: &[],
                            surfaces: &[],
                            hole_plane: self.hole_plane.clone(),
                        };
                        return dlg.tool.spec(&ctx, dlg.editing.as_ref().map(|(_, s)| s));
                    }
                }
            }
        };
        // a surface bounded by edges may take them from several bodies
        let multi = matches!(
            dlg.tool,
            Tool::Patch { .. }
                | Tool::Ruled
                | Tool::Stitch { .. }
                | Tool::Match { .. }
                | Tool::Trim { use_face: true, .. }
        );
        let wants_edges = matches!(
            dlg.tool,
            Tool::Patch { .. } | Tool::Ruled | Tool::Extend { .. } | Tool::Match { .. }
        );
        let wants_faces = matches!(dlg.tool, Tool::Match { .. } | Tool::Trim { .. });
        let mut picked_faces: Vec<(FeatureId, kw_feature::Reference)> = Vec::new();
        let mut picked_heads: Vec<FeatureId> = Vec::new();
        let mut boundary: Vec<(FeatureId, kw_feature::BoundaryRef)> = Vec::new();
        let mut picked_body: Option<usize> = None;
        let mut local: Vec<Pick> = Vec::new();
        for p in &self.model.picks {
            if let Some((i, lp)) = bodies.origin(*p) {
                let body = &bodies.list[i];
                if let (Pick::Edge(e), true) = (lp, wants_edges) {
                    boundary.push((body.head, kw_feature::ops::boundary_ref(&body.output, e)?));
                    // a match works on the body its edge is on
                    if matches!(dlg.tool, Tool::Match { .. }) {
                        picked_body = Some(i);
                    }
                }
                if let (Pick::Face(f), true) = (lp, wants_faces) {
                    if let Some(r) = body
                        .output
                        .brep()
                        .and_then(|rep| body.output.names.reference_to(rep, f))
                    {
                        picked_faces.push((body.head, r));
                    }
                }
                if multi {
                    let head = body.head;
                    if !picked_heads.contains(&head) {
                        picked_heads.push(head);
                    }
                    continue;
                }
                if picked_body.is_some_and(|b| b != i) {
                    return Err("pick faces and edges of one body only".into());
                }
                picked_body = Some(i);
                local.push(lp);
            }
        }
        let sheet_heads: Vec<FeatureId> = bodies
            .list
            .iter()
            .filter(|b| b.output.is_sheet())
            .map(|b| b.head)
            .collect();
        let idx = picked_body
            .or_else(|| self.target_index())
            .unwrap_or(bodies.list.len() - 1);
        let body = &bodies.list[idx];
        let ctx = BuildCtx {
            doc: &doc,
            base: body.output.brep().map(|b| (b, &body.output.names)),
            picks: &local,
            boundary: &boundary,
            picked_bodies: &picked_heads,
            faces: &picked_faces,
            surfaces: &sheet_heads,
            hole_plane: self.hole_plane.clone(),
        };
        let orig = dlg.editing.as_ref().map(|(_, s)| s);
        let mut spec = dlg.tool.spec(&ctx, orig)?;
        // (Split Body always names its body: its two halves must both start from the same one)
        if (bodies.list.len() > 1 || matches!(dlg.tool, Tool::SplitBody { .. }))
            && spec.needs_body()
        {
            // several bodies: say which one the feature works on (an edited feature keeps its own, unless
            // new picks name another)
            match (orig, picked_body) {
                (Some(o), None) => spec.set_base(o.explicit_base()),
                _ => spec.set_base(Some(body.head)),
            }
        }
        Ok(spec)
    }

    /// Recompute the regions of the Extrude dialog's sketch when it changed.
    fn refresh_regions(&mut self, ev: &Evaluation) {
        let sk = match &self.dialog {
            Some(Dialog {
                tool: Tool::Extrude {
                    sketch: Some(s), ..
                },
                ..
            }) => *s,
            _ => {
                self.region_view = None;
                return;
            }
        };
        let Some(o) = ev.output(sk) else {
            self.region_view = None;
            return;
        };
        let key = o.geom_hash;
        if self
            .region_view
            .as_ref()
            .is_some_and(|r| r.sketch == sk && r.key == key)
        {
            return;
        }
        let Some(kw_feature::Product::Sketch(sp)) = Some(&o.product) else {
            self.region_view = None;
            return;
        };
        let sr = kw_feature::regions::sketch_regions(&sp.sketch, &sp.entities, sp.plane);
        let view = RegionView {
            sketch: sk,
            key,
            plane: sp.plane,
            sr,
            product: sp.clone(),
        };
        // the pick is explicit: a dialog that has chosen nothing starts with the default regions chosen
        if view.regions().len() > 1 {
            if let Some(Dialog {
                tool: Tool::Extrude { more, .. },
                ..
            }) = self.dialog.as_mut()
            {
                if more.regions.is_empty() {
                    more.regions = view.default_picks();
                }
            }
        }
        self.region_view = Some(view);
    }

    /// The index of the sketch region under a screen position, if the position is over one.
    fn region_at(&self, pos: egui::Pos2) -> Option<usize> {
        let rv = self
            .region_view
            .as_ref()
            .filter(|r| r.regions().len() > 1)?;
        let c = self.viewport.model()?.scene.center;
        let (o, d) = self.viewport.ray(pos);
        let (o, d) = (
            kw_math::Vector3::new(o.x + c.x, o.y + c.y, o.z + c.z),
            kw_math::Vector3::new(d.x, d.y, d.z),
        );
        let f = &rv.plane;
        let denom = d.dot(&f.z);
        if denom.abs() < 1e-9 {
            return None;
        }
        let t = (f.origin.coords - o).dot(&f.z) / denom;
        let hit = o + d * t - f.origin.coords;
        rv.sr.region_at([hit.dot(&f.x), hit.dot(&f.y)])
    }

    /// The features a dialog adds: one, except Split Body, which adds a feature per half.
    fn specs_for(dlg: &Dialog, spec: FeatureSpec) -> Vec<FeatureSpec> {
        match (&dlg.tool, &spec) {
            (Tool::SplitBody { .. }, FeatureSpec::SplitBody(sb)) if dlg.editing.is_none() => {
                let (mut a, mut b) = (sb.clone(), sb.clone());
                a.keep = kw_feature::SplitSide::Positive;
                b.keep = kw_feature::SplitSide::Negative;
                vec![FeatureSpec::SplitBody(a), FeatureSpec::SplitBody(b)]
            }
            _ => vec![spec],
        }
    }

    /// Recompute the preview when the dialog's inputs changed.
    pub fn update_preview(&mut self, project: &mut Project) {
        let Some(dlg) = self.dialog.clone() else {
            self.preview = None;
            return;
        };
        let doc = project.document();
        let key = format!(
            "{:?}|{}|{:?}|{:?}|{:?}",
            dlg.tool,
            doc.content_hash(),
            self.model.picks,
            self.hole_plane,
            // the regions of the sketch decide whether an Extrude may go ahead
            self.region_view
                .as_ref()
                .map(|r| (r.sketch, r.key, r.regions().len()))
        );
        if self.preview.as_ref().is_some_and(|p| p.key == key) {
            return;
        }
        let mut pv = Preview {
            key,
            bodies: None,
            brep: None,
            error: None,
        };
        match self.build_spec(project, &dlg) {
            Err(e) => pv.error = Some(e),
            // a form is finished into a body only at OK: the surface fit takes seconds, the cage is drawn
            Ok(_) if matches!(dlg.tool, Tool::Form { .. }) => {}
            Ok(spec) => {
                let mut tmp = (*doc).clone();
                let added: Result<Vec<FeatureId>, kw_feature::TimelineError> = match &dlg.editing {
                    Some((id, _)) => tmp.replace_spec(*id, spec).map(|_| vec![*id]),
                    None => {
                        let mut ids = Vec::new();
                        let mut res = Ok(());
                        for sp in Self::specs_for(&dlg, spec) {
                            let name = crate::model::unique_name(&tmp, dlg.tool.name_base());
                            match tmp.add(&name, sp) {
                                Ok(id) => ids.push(id),
                                Err(e) => {
                                    res = Err(e);
                                    break;
                                }
                            }
                        }
                        res.map(|_| ids)
                    }
                };
                match added {
                    Err(e) => pv.error = Some(e.to_string()),
                    Ok(ids) => {
                        let ev = project.engine.evaluate(&tmp);
                        if let Some(e) = ids.iter().find_map(|id| ev.error(*id)) {
                            pv.error = Some(e.to_string());
                        } else if let Some(b) = Bodies::new(&tmp, &ev, &self.hidden_bodies) {
                            pv.brep = Some(b.brep.clone());
                            pv.bodies = Some(Arc::new(b));
                        }
                    }
                }
            }
        }
        self.preview = Some(pv);
    }

    pub fn commit_dialog(&mut self, project: &mut Project) -> bool {
        let Some(dlg) = self.dialog.clone() else {
            return false;
        };
        let spec = match self.build_spec(project, &dlg) {
            Ok(s) => s,
            Err(e) => {
                self.model.error = e;
                return false;
            }
        };
        let base_name = dlg.tool.name_base();
        let ok = match &dlg.editing {
            Some((id, _)) => {
                let id = *id;
                self.model
                    .edit(project, |d| d.replace_spec(id, spec).map(|_| id))
            }
            None => {
                let specs = Self::specs_for(&dlg, spec);
                self.model.edit(project, |d| {
                    let mut last = None;
                    for sp in specs {
                        let name = crate::model::unique_name(d, base_name);
                        last = Some(d.add(&name, sp)?);
                    }
                    last.ok_or(kw_feature::TimelineError::BadIndex { index: 0, len: 0 })
                })
            }
        };
        match ok {
            Some(id) => {
                // as in Onshape, a sketch that has just been used is hidden (its eye in the browser brings
                // it back, to make another feature from other regions of it)
                if dlg.editing.is_none() {
                    self.hidden_sketches.extend(dlg.tool.sketches());
                }
                self.model.selected = Some(id);
                self.dialog = None;
                self.preview = None;
                self.model.picks.clear();
                self.hole_plane = None;
                true
            }
            None => false,
        }
    }

    pub fn close_dialog(&mut self) {
        self.dialog = None;
        self.preview = None;
        self.model.picks.clear();
        self.hole_plane = None;
        self.model.error.clear();
    }

    // --------------------------------------------------------------------------------- actions

    pub fn run_actions(&mut self, project: &mut Project) {
        let actions = std::mem::take(&mut self.actions);
        for a in actions {
            match a {
                Action::FlatPattern => {
                    if let Err(e) = self.sheet_flat_pattern(project) {
                        self.model.error = e;
                    }
                }
                Action::EditLastOf(kind) => {
                    let id = project
                        .document()
                        .features
                        .iter()
                        .rev()
                        .find(|f| f.spec.kind_name() == kind)
                        .map(|f| f.id);
                    if let Some(id) = id {
                        self.open_edit(project, id);
                    }
                }
                Action::OpenAppearance => {
                    let doc = project.document();
                    self.body_props = self
                        .active_body
                        .or_else(|| crate::bodies::summaries(&doc).first().map(|b| b.root));
                }
                Action::NewComponent => {
                    if let Err(e) = self.asm_new_component(project) {
                        self.model.error = e;
                    }
                }
                Action::NewEmptyComponent => {
                    if let Err(e) = self.asm_new_empty_component(project) {
                        self.model.error = e;
                    }
                }
                Action::ComponentFromBody(head) => {
                    if let Err(e) = self.asm_component_from_body(project, head) {
                        self.model.error = e;
                    }
                }
                Action::ActivateComponent(p) => {
                    if let Err(e) = self.activate_component(project, p) {
                        self.model.error = e;
                    }
                }
                Action::FinishComponent => {
                    if let Err(e) = self.finish_component(project) {
                        self.model.error = e;
                    }
                }
                Action::Asm(a) => self.asm_action(project, a),
                Action::Select(id) => self.model.selected = Some(id),
                Action::Edit(id) => self.open_edit(project, id),
                Action::Delete(id) => {
                    // the feature and everything that depends on it go together (one undo step)
                    let removed = self.model.edit(project, |d| d.remove_with_dependents(id));
                    if let Some(removed) = removed {
                        if self
                            .model
                            .selected
                            .is_some_and(|s| removed.iter().any(|f| f.id == s))
                        {
                            self.model.selected = None;
                        }
                        self.status = match removed.len() {
                            0 | 1 => String::new(),
                            n => format!(
                                "Deleted {} and the {} step{} that depended on it.",
                                removed[0].name,
                                n - 1,
                                if n == 2 { "" } else { "s" }
                            ),
                        };
                    }
                }
                Action::Suppress(id, s) => {
                    self.model.edit(project, |d| d.set_suppressed(id, s));
                }
                Action::RollbackTo(id) => {
                    self.model.edit(project, |d| d.rollback_to(id));
                }
                Action::ClearRollback => {
                    self.model.edit(project, |d| {
                        d.clear_rollback();
                        Ok(())
                    });
                }
                Action::Move(id, idx) => {
                    self.model.edit(project, |d| d.move_feature(id, idx));
                }
                Action::Open(t) => self.open_tool(project, t),
                Action::StartSketch => {
                    if project.document().rolled_back() {
                        self.model.error =
                            "Roll the timeline forward to its end before adding features.".into();
                        continue;
                    }
                    self.mode = Mode::PickPlane;
                    self.dialog = None;
                    self.preview = None;
                    self.model.picks.clear();
                    self.status = "Select a sketch plane or a planar face.".into();
                }
                Action::FinishSketch => self.finish_sketch(project),
                Action::CancelSketch => self.cancel_sketch(),
                Action::SketchOn(plane, frame) => self.begin_sketch(project, plane, frame),
                Action::ToggleSketch(id) => {
                    if !self.hidden_sketches.remove(&id) {
                        self.hidden_sketches.insert(id);
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------------------------- UI

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        {
            let doc = project.document();
            self.dep_counts = doc
                .features
                .iter()
                .map(|f| (f.id, doc.all_dependents_of(f.id).len()))
                .collect();
        }
        // ---- bottom: timeline (always), left: browser
        self.timeline_ui(ui, project);
        egui::Panel::left("design_browser")
            .default_size(250.0)
            .show(ui, |ui| self.browser_ui(ui, project));
        if self.mode == Mode::Sketching {
            egui::Panel::right("sketch_palette")
                .default_size(250.0)
                .show(ui, |ui| self.palette_ui(ui));
            egui::CentralPanel::default().show(ui, |ui| {
                // formulas in dimensions may use the document's parameters
                let doc = project.document();
                self.sketch.app.scope = project.evaluation().scope(&doc);
                self.sketch.app.shortcuts(ui.ctx());
                self.sketch.app.canvas_ui(ui);
            });
            self.sketch.app.dialogs(ui.ctx());
            self.sketch_panel_window(ui.ctx());
        } else {
            egui::CentralPanel::default()
                .frame(egui::Frame::NONE)
                .show(ui, |ui| self.viewport_ui(ui, project, rs));
        }
        self.asm_links_tick(ui.ctx(), project);
        self.run_actions(project);
        // a component being edited in place is copied into the assembly after every change
        project.sync_component();
    }

    /// Forget everything that refers to the document on screen (picks, selections, open dialogs) because
    /// another document replaces it (a component is opened, or closed). The camera, the ribbon tab and the
    /// assembly state stay.
    pub fn switch_document(&mut self) {
        let asm = std::mem::take(&mut self.asm);
        let viewport = std::mem::take(&mut self.viewport);
        let (tab, shaded) = (self.ribbon_tab, self.shaded);
        *self = DesignWorkspace::default();
        self.asm = asm;
        self.viewport = viewport;
        self.ribbon_tab = tab;
        self.shaded = shaded;
    }

    fn palette_ui(&mut self, ui: &mut egui::Ui) {
        ui.add_space(4.0);
        ui.label(egui::RichText::new("SKETCH PALETTE").size(11.0).strong());
        ui.separator();
        ui.label(egui::RichText::new("Solver").color(theme::TEXT_WEAK));
        self.sketch.app.inspector(ui);
        if !self.sketch.error.is_empty() {
            ui.colored_label(theme::BAD, &self.sketch.error);
        }
    }

    fn viewport_ui(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        rs: Option<&eframe::egui_wgpu::RenderState>,
    ) {
        let ev = project.evaluation();
        let doc = project.document();
        self.viewport.section = self.section_plane(project);
        self.viewport.analysis = self.analysis_view();
        self.update_preview(project);
        // While a tool is collecting picks (edges for Fillet / Chamfer, faces for Shell / Hole) the view must
        // show the body the picks refer to: the preview solid has other edges and faces, so picking on it
        // would store ids that mean something else. Those tools show their result after OK.
        let collecting = self
            .dialog
            .as_ref()
            .is_some_and(|d| d.tool.picks_edges() || d.tool.picks_faces());
        self.bodies = Bodies::new(&doc, &ev, &self.hidden_bodies).map(Arc::new);
        if self.active_body.is_some_and(|r| {
            self.bodies
                .as_ref()
                .is_none_or(|b| b.index_of_root(r).is_none())
        }) {
            self.active_body = None;
        }
        self.refresh_regions(&ev);
        let preview = self.preview.as_ref().filter(|_| !collecting);
        let preview_bodies = preview.and_then(|p| p.bodies.clone());
        let shown: Option<Arc<Bodies>> = preview_bodies.clone().or_else(|| self.bodies.clone());
        let geom_key = match (&preview_bodies, &shown) {
            (Some(pb), _) => {
                let mut h = kw_feature::Hasher128::new();
                h.bytes(preview.map(|p| p.key.as_bytes()).unwrap_or_default());
                h.u128(pb.key);
                h.finish()
            }
            (None, Some(sb)) => sb.key,
            _ => 0,
        };
        let base_key = if preview_bodies.is_some() {
            0xB0B0_u128
        } else {
            0
        };
        // the sketch a tool is working on is shown while the tool is open, even when it was hidden
        let mut hidden_now = self.hidden_sketches.clone();
        if let Some(d) = &self.dialog {
            for sk in d.tool.sketches() {
                hidden_now.remove(&sk);
            }
        }
        let segs = geom::sketch_segments(&ev, &doc, &hidden_now);
        // a curvature map is baked into the scene: another kind needs another build
        let curvature = self.analysis.as_ref().and_then(|a| a.kind.curvature());
        let curv_code: u128 = match curvature {
            None => 0,
            Some(kw_analysis::CurvKind::Gaussian) => 1,
            Some(kw_analysis::CurvKind::Mean) => 2,
            Some(kw_analysis::CurvKind::Max) => 3,
            Some(kw_analysis::CurvKind::Min) => 4,
        };
        let key = geom_key
            ^ base_key
            ^ ((segs.len() as u128) << 96)
            ^ ((hidden_now.len() as u128) << 90)
            ^ (curv_code << 80);
        let highlight = self.model.picks.last().copied();
        // all picked faces / edges show as selected, and edges are easy to hit while a tool wants them
        self.viewport.extra_selected = self.model.picks.clone();
        self.viewport.edge_pick_px = match (&self.dialog, self.mode) {
            (_, Mode::PickPlane) => 0,
            (Some(d), _) if d.tool.picks_edges() => 8,
            (Some(_), _) => 0,
            (None, _) if self.measure_open => 8,
            (None, _) => 4,
        };
        // dragging a component of the posed assembly takes the pointer from the camera
        let dragging = self.asm_drag_frame(ui, project);
        let opts = ViewOpts {
            hover: true,
            navigate: !dragging,
        };
        self.viewport.wireframe = !self.shaded;
        let b = shown.clone();
        // while the assembly is shown the view draws the posed components (nothing in it can be picked)
        let asm_view = self.asm_display(project);
        let key = asm_view.as_ref().map_or(key, |a| a.0);
        let out = self.viewport.show_with(
            ui,
            rs,
            key,
            move || {
                let mut m = match (&asm_view, &b) {
                    (Some((_, rep, cols)), _) => ModelScene::build_analysed(rep, cols, curvature),
                    (None, Some(b)) => ModelScene::build_analysed(&b.brep, &b.colours(), curvature),
                    (None, None) => ModelScene::empty(),
                };
                m.add_segments(&segs);
                m
            },
            highlight,
            opts,
        );
        self.overlay_ui(ui, project, &ev, &out);
        if self.asm.view {
            self.asm_view_clicks(ui, project, &out);
        } else {
            self.handle_clicks(ui, project, &ev, &out);
        }
        // dialog
        if let Some(mut dlg) = self.dialog.take() {
            let picks = self.model.picks.clone();
            let err = self
                .preview
                .as_ref()
                .and_then(|p| p.error.clone())
                .or_else(|| (!self.model.error.is_empty()).then(|| self.model.error.clone()));
            let matching: Vec<Vec<usize>> = match (&self.region_view, &dlg.tool) {
                (Some(rv), Tool::Extrude { more, .. }) => rv.matching(&more.regions),
                _ => Vec::new(),
            };
            let infos: Vec<tools::RegionInfo> = self
                .region_view
                .as_ref()
                .filter(|_| !matching.is_empty())
                .map(|r| {
                    r.regions()
                        .iter()
                        .enumerate()
                        .map(|(i, g)| tools::RegionInfo {
                            pick: g.pick.clone(),
                            area: g.area,
                            matching: matching[i].clone(),
                            default: g.depth.is_multiple_of(2),
                        })
                        .collect()
                })
                .unwrap_or_default();
            let (outcome, _) =
                tools::panel(ui.ctx(), &mut dlg, &doc, &picks, &infos, err.as_deref());
            self.dialog = Some(dlg);
            match outcome {
                Outcome::Ok => {
                    if let Some(e) = self.preview.as_ref().and_then(|p| p.error.clone()) {
                        self.model.error = e;
                    } else {
                        self.commit_dialog(project);
                    }
                }
                Outcome::Cancel => self.close_dialog(),
                Outcome::Open => {}
            }
        }
        // status line inside the view
        if !self.status.is_empty() && self.mode != Mode::Idle {
            let p = ui.painter();
            let r = egui::Rect::from_min_size(
                out.rect.left_top() + egui::vec2(10.0, 8.0),
                egui::vec2(420.0, 24.0),
            );
            p.rect_filled(
                r,
                3.0,
                egui::Color32::from_rgba_unmultiplied(255, 255, 255, 220),
            );
            p.text(
                r.left_center() + egui::vec2(8.0, 0.0),
                egui::Align2::LEFT_CENTER,
                &self.status,
                egui::FontId::proportional(12.0),
                theme::TEXT,
            );
        }
        if ui.ctx().input(|i| i.key_pressed(egui::Key::Escape)) {
            match self.mode {
                Mode::PickPlane => {
                    self.mode = Mode::Idle;
                    self.status.clear();
                }
                _ if self.dialog.is_some() => self.close_dialog(),
                _ => self.model.picks.clear(),
            }
        }
    }

    pub fn handle_clicks(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        ev: &Arc<Evaluation>,
        out: &crate::viewport::ViewOut,
    ) {
        let shift = ui.ctx().input(|i| i.modifiers.shift);
        if let Some(click) = &out.click {
            match self.mode {
                Mode::PickPlane => {
                    let face = match click {
                        Some(Pick::Face(f)) => Some(*f),
                        _ => None,
                    };
                    if let (Some(f), Some(bodies)) = (face, self.bodies.clone()) {
                        let located = bodies.origin(Pick::Face(f));
                        let brep = &bodies.brep;
                        let reference = located.and_then(|(i, lp)| match lp {
                            Pick::Face(of) => {
                                let o = &bodies.list[i].output;
                                o.names.reference_to(o.brep()?, of).map(|r| (i, r))
                            }
                            _ => None,
                        });
                        match (geom::face_frame(brep, f), reference) {
                            (Some(frame), Some((i, r))) => {
                                if bodies.list.len() > 1 {
                                    self.next_sketch_base = Some(bodies.list[i].head);
                                    self.active_body = Some(bodies.list[i].root);
                                }
                                self.actions
                                    .push(Action::SketchOn(PlaneSpec::Face(r), frame));
                                return;
                            }
                            (None, _) => {
                                self.model.error = "that face is not planar".into();
                                self.status =
                                    "That face is not planar - pick a plane or a flat face.".into();
                                return;
                            }
                            _ => {}
                        }
                    }
                    if let Some((id, frame)) = self.hover_datum {
                        self.actions
                            .push(Action::SketchOn(PlaneSpec::Datum(id), frame));
                    } else if let Some(i) = self.hover_plane {
                        self.actions.push(Action::SketchOn(
                            geom::origin_spec(geom::ORIGIN_PLANES[i].1),
                            geom::origin_frame(geom::ORIGIN_PLANES[i].1),
                        ));
                    }
                }
                Mode::Idle => {
                    if self.try_toggle_region(out) {
                        return;
                    }
                    // tools that take picks, otherwise plain selection
                    let (edges, faces, hole) = self
                        .dialog
                        .as_ref()
                        .map(|d| {
                            (
                                d.tool.picks_edges(),
                                d.tool.picks_faces(),
                                matches!(d.tool, Tool::Hole { .. }),
                            )
                        })
                        .unwrap_or((false, false, false));
                    match click {
                        Some(p) => {
                            let accept = if self.dialog.is_none() {
                                true
                            } else {
                                (edges && matches!(p, Pick::Edge(_)))
                                    || (faces && matches!(p, Pick::Face(_)))
                            };
                            if accept {
                                if hole {
                                    self.pick_hole_face(ev, *p, out);
                                } else if shift || edges || (faces && !hole) || self.measure_open {
                                    if let Some(i) = self.model.picks.iter().position(|q| q == p) {
                                        self.model.picks.remove(i);
                                    } else {
                                        self.model.picks.push(*p);
                                    }
                                } else {
                                    self.model.picks = vec![*p];
                                }
                            }
                        }
                        None => {
                            if !shift && self.dialog.is_none() {
                                self.model.picks.clear();
                            }
                        }
                    }
                }
                Mode::Sketching => {}
            }
        }
        if let Some(Some(p)) = &out.double_click {
            // double-click a face while idle starts nothing; reserved for edit-in-place of features later
            let _ = p;
        }
        let _ = project;
    }

    /// A click on a sketch region while the Extrude dialog is open chooses (or drops) that region.
    fn try_toggle_region(&mut self, out: &crate::viewport::ViewOut) -> bool {
        let Some(pos) = out.click_pos else {
            return false;
        };
        let Some(Dialog {
            tool: Tool::Extrude { more, .. },
            ..
        }) = &self.dialog
        else {
            return false;
        };
        if more.to_face {
            return false; // clicks choose the face to stop at
        }
        let Some(hit) = self.region_at(pos) else {
            return false;
        };
        let Some(rv) = self.region_view.as_ref() else {
            return false;
        };
        let new_pick = rv.regions()[hit].pick.clone();
        let existing = match self.dialog.as_ref().map(|d| &d.tool) {
            Some(Tool::Extrude { more, .. }) => rv.matching(&more.regions)[hit].clone(),
            _ => return false,
        };
        if let Some(Dialog {
            tool: Tool::Extrude { more, .. },
            ..
        }) = self.dialog.as_mut()
        {
            if existing.is_empty() {
                more.regions.push(new_pick);
            } else {
                for k in existing.into_iter().rev() {
                    more.regions.remove(k);
                }
            }
        }
        true
    }

    /// A click on a planar face for the Hole tool: remember the plane and put the click's position in the fields.
    fn pick_hole_face(&mut self, _ev: &Arc<Evaluation>, p: Pick, out: &crate::viewport::ViewOut) {
        let Pick::Face(f) = p else { return };
        let Some(bodies) = self.bodies.clone() else {
            return;
        };
        let reference = bodies.origin(p).and_then(|(i, lp)| match lp {
            Pick::Face(of) => {
                let o = &bodies.list[i].output;
                o.names.reference_to(o.brep()?, of)
            }
            _ => None,
        });
        let (Some(frame), Some(r)) = (geom::face_frame(&bodies.brep, f), reference) else {
            self.model.error = "that face is not planar".into();
            return;
        };
        self.hole_plane = Some(PlaneSpec::Face(r));
        self.model.picks = vec![p];
        if let (Some(pos), Some(m)) = (out.click_pos, self.viewport.model()) {
            let (o, d) = self.viewport.ray(pos);
            let c = m.scene.center;
            let (o, d) = ([o.x + c.x, o.y + c.y, o.z + c.z], [d.x, d.y, d.z]);
            let denom = d[0] * frame.n[0] + d[1] * frame.n[1] + d[2] * frame.n[2];
            if denom.abs() > 1e-9 {
                let t = ((frame.origin[0] - o[0]) * frame.n[0]
                    + (frame.origin[1] - o[1]) * frame.n[1]
                    + (frame.origin[2] - o[2]) * frame.n[2])
                    / denom;
                let hit = [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
                let (u, v, _) = frame.to_plane(hit);
                if let Some(Dialog {
                    tool: Tool::Hole { x, y, .. },
                    ..
                }) = self.dialog.as_mut()
                {
                    *x = format!("{u:.3} mm");
                    *y = format!("{v:.3} mm");
                }
            }
        }
    }
}
