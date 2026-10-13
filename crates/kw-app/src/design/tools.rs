//! The modelling tools of the Design workspace as dialogs: each holds the user's inputs as text, builds a
//! feature spec from them, and draws its own panel. The same dialog creates a feature and edits one.

use crate::chrome::theme::{ACCENT, BAD, TEXT_WEAK};
use crate::model::{param_text, parse_param};
use crate::viewport::Pick;
use eframe::egui;
use kw_brep::BRep;
use kw_expr::{Dim, Param};
use kw_feature::{
    AxisSpec, BodyOp, ChamferSpec, CircularPatternSpec, CombineSpec, DatumPlaneSpec, Document,
    DraftSpec, EdgeRef, ExtrudeSpec, FeatureId, FeatureSpec, FilletSpec, HoleKind, HoleSpec,
    LinearPatternSpec, LoftSpec, MirrorPlane, MirrorSpec, MoveBodySpec, NameMap, OffsetSpec,
    PlaneSpec, RevolveSpec, ScaleBodySpec, ShellSpec, SplitBodySpec, SplitSide, SweepSpec,
};
use std::sync::Arc;

/// What a tool may look at while building its spec.
pub struct BuildCtx<'a> {
    pub doc: &'a Document,
    /// The solid the new feature will modify, with its face names.
    pub base: Option<(&'a Arc<BRep>, &'a NameMap)>,
    pub picks: &'a [Pick],
    /// The edges picked for a surface (Patch, Ruled), each with the feature whose body it is on: they may
    /// belong to different bodies.
    pub boundary: &'a [(FeatureId, kw_feature::BoundaryRef)],
    /// The bodies the picked faces and edges belong to, and every surface body of the document.
    pub picked_bodies: &'a [FeatureId],
    pub surfaces: &'a [FeatureId],
    /// Plane picked for a hole, and the spot on it (plane coordinates).
    pub hole_plane: Option<PlaneSpec>,
}

/// A plane chosen in a dialog: an origin plane (moved by `offset`) or a construction plane feature.
#[derive(Debug, Clone, PartialEq)]
pub struct PlaneInput {
    /// 0 = XY, 1 = YZ, 2 = XZ, 3 = a construction plane.
    pub which: u8,
    pub offset: String,
    pub datum: Option<FeatureId>,
}

impl Default for PlaneInput {
    fn default() -> Self {
        PlaneInput {
            which: 0,
            offset: "0 mm".into(),
            datum: None,
        }
    }
}

impl PlaneInput {
    pub fn spec(&self) -> Result<PlaneSpec, String> {
        let o = || parse_param(&self.offset, Dim::LENGTH);
        Ok(match self.which {
            0 => PlaneSpec::Xy(o()?),
            1 => PlaneSpec::Yz(o()?),
            2 => PlaneSpec::Xz(o()?),
            _ => PlaneSpec::Datum(self.datum.ok_or("choose a construction plane")?),
        })
    }

    fn from_spec(p: &PlaneSpec) -> PlaneInput {
        match p {
            PlaneSpec::Xy(o) => PlaneInput {
                which: 0,
                offset: param_text(o),
                datum: None,
            },
            PlaneSpec::Yz(o) => PlaneInput {
                which: 1,
                offset: param_text(o),
                datum: None,
            },
            PlaneSpec::Xz(o) => PlaneInput {
                which: 2,
                offset: param_text(o),
                datum: None,
            },
            PlaneSpec::Datum(d) => PlaneInput {
                which: 3,
                offset: "0 mm".into(),
                datum: Some(*d),
            },
            PlaneSpec::Face(_) => PlaneInput::default(),
        }
    }
}

/// A point chosen in a dialog: coordinates, or a construction point feature.
#[derive(Debug, Clone, PartialEq)]
pub struct PointInput {
    pub datum: Option<FeatureId>,
    pub x: String,
    pub y: String,
    pub z: String,
}

impl Default for PointInput {
    fn default() -> Self {
        PointInput {
            datum: None,
            x: "0 mm".into(),
            y: "0 mm".into(),
            z: "0 mm".into(),
        }
    }
}

impl PointInput {
    pub fn src(&self) -> Result<kw_feature::PointSrc, String> {
        match self.datum {
            Some(d) => Ok(kw_feature::PointSrc::Datum(d)),
            None => Ok(kw_feature::PointSrc::Coords([
                parse_param(&self.x, Dim::LENGTH)?,
                parse_param(&self.y, Dim::LENGTH)?,
                parse_param(&self.z, Dim::LENGTH)?,
            ])),
        }
    }

    fn from_src(p: &kw_feature::PointSrc) -> PointInput {
        match p {
            kw_feature::PointSrc::Datum(d) => PointInput {
                datum: Some(*d),
                ..Default::default()
            },
            kw_feature::PointSrc::Coords(c) => PointInput {
                datum: None,
                x: param_text(&c[0]),
                y: param_text(&c[1]),
                z: param_text(&c[2]),
            },
        }
    }
}

/// The extra inputs of a construction plane that is not a plain offset.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlaneMore {
    /// 0 = offset from a plane, 1 = midplane between two planes, 2 = through three points.
    pub mode: u8,
    pub other: PlaneInput,
    pub pts: [PointInput; 3],
}

/// One flat or bend of a sheet-metal strip as typed: a flat has a `length`, a bend an `angle` (degrees)
/// and an optional inner `radius` (empty = the strip's default).
#[derive(Debug, Clone, PartialEq)]
pub struct SheetRow {
    pub bend: bool,
    pub a: String,
    pub r: String,
}

impl SheetRow {
    pub fn flat(length: &str) -> Self {
        SheetRow {
            bend: false,
            a: length.into(),
            r: String::new(),
        }
    }
    pub fn bend(angle: &str) -> Self {
        SheetRow {
            bend: true,
            a: angle.into(),
            r: String::new(),
        }
    }
}

/// The less common Extrude inputs.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExtrudeMore {
    /// Stop at a planar face (picked in the view) instead of after a distance.
    pub to_face: bool,
    /// Wall thickness; empty = a solid.
    pub thin: String,
    /// 0 = inside the profile, 1 = outside, 2 = centred.
    pub thin_side: u8,
    /// Distance from the sketch plane to where the extrusion starts; empty = at the plane.
    pub start: String,
    /// Chosen regions of the sketch (empty = everything).
    pub regions: Vec<kw_feature::RegionPick>,
    /// Make a surface from the curves of the sketch instead of a solid.
    pub surface: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Tool {
    Extrude {
        sketch: Option<FeatureId>,
        depth: String,
        op: BodyOp,
        symmetric: bool,
        flip: bool,
        /// Through all of the body (the distance is ignored).
        through_all: bool,
        /// Distance on the other side of the sketch plane; empty = none.
        second: String,
        /// Taper angle; empty = none.
        taper: String,
        more: ExtrudeMore,
    },
    Revolve {
        sketch: Option<FeatureId>,
        angle: String,
        axis_y: bool,
        /// Turn about a straight edge of the body (picked in the view) instead of a sketch axis.
        axis_edge: bool,
        /// Turn about this construction axis (takes precedence over the sketch axes).
        axis_datum: Option<FeatureId>,
        op: BodyOp,
        /// Make a surface from the curves of the sketch instead of a solid.
        surface: bool,
    },
    /// A wall standing on a sketch: lines and arcs thickened in the sketch plane, extruded.
    Rib {
        sketch: Option<FeatureId>,
        thickness: String,
        depth: String,
        symmetric: bool,
        flip: bool,
        op: BodyOp,
    },
    /// A tube along the path of a sketch.
    Pipe {
        path: Option<FeatureId>,
        diameter: String,
        wall: String,
        op: BodyOp,
    },
    Hole {
        x: String,
        y: String,
        diameter: String,
        depth: String,
        kind: u8,
        cb_diameter: String,
        cb_depth: String,
        cs_diameter: String,
        cs_angle: String,
        /// Drill through the whole body (the depth is ignored).
        through_all: bool,
        /// For a tapped hole (`kind == 3`): the thread size and the length of the thread.
        thread: String,
        thread_length: String,
    },
    Fillet {
        radius: String,
    },
    Chamfer {
        d1: String,
        d2: String,
    },
    /// Fillet whose radius changes linearly along one picked edge.
    VariableFillet {
        start: String,
        end: String,
    },
    Shell {
        thickness: String,
    },
    Pattern {
        source: Option<FeatureId>,
        dx: String,
        dy: String,
        dz: String,
        count: String,
    },
    CircularPattern {
        source: Option<FeatureId>,
        count: String,
        angle: String,
        /// 0 = X, 1 = Y, 2 = Z, 3 = the construction axis in `datum`.
        axis: u8,
        datum: Option<FeatureId>,
    },
    Mirror {
        source: Option<FeatureId>,
        /// 0 = XY, 1 = YZ, 2 = XZ, 3 = the construction plane in `datum`, 4 = the picked planar face.
        plane: u8,
        datum: Option<FeatureId>,
    },
    Combine {
        target: Option<FeatureId>,
        tool: Option<FeatureId>,
        op: BodyOp,
        /// Keep the tool body as a body of its own.
        keep_tool: bool,
    },
    /// Blend between two or more profile sketches, in order.
    Loft {
        sketches: Vec<Option<FeatureId>>,
        op: BodyOp,
        /// Make a surface instead of a solid.
        surface: bool,
        /// A surface straight between neighbouring profiles.
        ruled: bool,
    },
    /// A profile sketch swept along the first line, arc or spline of a path sketch.
    Sweep {
        profile: Option<FeatureId>,
        path: Option<FeatureId>,
        op: BodyOp,
        /// Make a surface instead of a solid.
        surface: bool,
        /// Total turn of the profile along the path; empty = none.
        twist: String,
        /// A sketch whose first curve guides the profile's orientation and scale.
        guide: Option<FeatureId>,
    },
    /// A surface filling the loop of edges picked in the view.
    Patch {
        /// Leave each open edge tangent to the surface it belongs to.
        tangent: bool,
    },
    /// A surface of straight lines between two picked edges.
    Ruled,
    /// Join the picked surfaces (all of them if none is picked) along edges closer than `tolerance`.
    Stitch {
        tolerance: String,
    },
    /// Split a body into its faces.
    Unstitch,
    /// Give a surface thickness: 0 = along its normal, 1 = against it, 2 = half each way.
    Thicken {
        thickness: String,
        side: u8,
    },
    /// Move a surface along its normal.
    OffsetSurface {
        distance: String,
    },
    /// Tilt the picked faces about the neutral plane `axis = position`; `axis` is the pull direction.
    Draft {
        angle: String,
        axis: u8,
        position: String,
    },
    /// A cosmetic thread on a picked cylindrical face.
    Thread {
        designation: String,
        length: String,
        internal: bool,
    },
    /// A sheet-metal strip: thickness, K-factor, bend radius, width and the flats and bends along it.
    SheetMetal {
        thickness: String,
        k_factor: String,
        radius: String,
        width: String,
        rows: Vec<SheetRow>,
    },
    /// A subdivision cage (a primitive whose vertices can be moved) finished into a body.
    Form {
        /// An index into [`super::form::KINDS`]; 255 = the cage is custom (edited from an existing form).
        kind: u8,
        size: [f64; 3],
        sides: u32,
        cage: kw_subd::SubdMesh,
        scale: String,
        tolerance: String,
        /// The vertex being edited.
        vertex: usize,
        /// `(kind, size, sides)` the cage was last built from: a change rebuilds it.
        built: (u8, [f64; 3], u32),
    },
    /// Grow (positive) or shrink the whole body.
    Offset {
        distance: String,
    },
    /// A construction plane: an origin plane moved along its normal and tilted about its x axis.
    ConstructionPlane {
        base: u8,
        offset: String,
        tilt: String,
        more: PlaneMore,
    },
    /// A construction axis: 0 = through two points, 1 = along an edge (click), 2 = normal to a face (click),
    /// 3 = where two planes meet.
    Axis {
        mode: u8,
        pts: [PointInput; 2],
        at_u: String,
        at_v: String,
        planes: [PlaneInput; 2],
    },
    /// A construction point: 0 = coordinates, 1 = middle of an edge (click), 2 = axis meets plane.
    DatumPoint {
        mode: u8,
        coords: PointInput,
        axis: Option<FeatureId>,
        plane: PlaneInput,
    },
    /// Move a body, or add a moved copy of it.
    MoveBody {
        copy: bool,
        dx: String,
        dy: String,
        dz: String,
        axis: u8,
        angle: String,
    },
    /// Scale a body uniformly about a point.
    ScaleBody {
        factor: String,
        cx: String,
        cy: String,
        cz: String,
    },
    /// Cut a body in two with a plane: `plane` 0 = XY, 1 = YZ, 2 = XZ at `offset`, 3 = the picked planar face.
    SplitBody {
        plane: u8,
        offset: String,
    },
}

/// A region of the sketch the dialog's profile can be cut into (for the Extrude region list).
#[derive(Debug, Clone, PartialEq)]
pub struct RegionInfo {
    pub pick: kw_feature::RegionPick,
    pub area: f64,
    /// The indices (in the dialog's picks) of the picks that name this region; empty = not chosen.
    pub matching: Vec<usize>,
    /// Is it one of the regions extruded by default?
    pub default: bool,
}

/// A tool dialog: the tool, and the feature it edits (if any) with that feature's original spec.
#[derive(Debug, Clone, PartialEq)]
pub struct Dialog {
    pub tool: Tool,
    pub editing: Option<(FeatureId, FeatureSpec)>,
}

pub enum Outcome {
    Open,
    Ok,
    Cancel,
}

fn op_label(op: BodyOp) -> &'static str {
    match op {
        BodyOp::New => "New Body",
        BodyOp::Join => "Join",
        BodyOp::Cut => "Cut",
        BodyOp::Intersect => "Intersect",
    }
}

fn row(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui.horizontal(|ui| {
        ui.add_sized(
            [86.0, 20.0],
            egui::Label::new(egui::RichText::new(label).color(TEXT_WEAK)),
        );
        add(ui);
    });
}

fn text_field(ui: &mut egui::Ui, label: &str, v: &mut String) {
    row(ui, label, |ui| {
        ui.add(egui::TextEdit::singleline(v).desired_width(110.0));
    });
}

fn feature_combo(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    doc: &Document,
    sel: &mut Option<FeatureId>,
    filter: impl Fn(&FeatureSpec) -> bool,
) {
    row(ui, label, |ui| {
        egui::ComboBox::from_id_salt(id)
            .selected_text(
                sel.and_then(|s| doc.get(s))
                    .map(|f| f.name.clone())
                    .unwrap_or_else(|| "Select…".into()),
            )
            .width(150.0)
            .show_ui(ui, |ui| {
                for f in doc.features.iter().filter(|f| filter(&f.spec)) {
                    ui.selectable_value(sel, Some(f.id), &f.name);
                }
            });
    });
}

/// The construction features of the document picked by `pred`.
fn datum_ids(doc: &Document, pred: impl Fn(&FeatureSpec) -> bool) -> Vec<FeatureId> {
    doc.features
        .iter()
        .filter(|f| pred(&f.spec))
        .map(|f| f.id)
        .collect()
}

fn datum_combo(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    doc: &Document,
    sel: &mut Option<FeatureId>,
    pred: impl Fn(&FeatureSpec) -> bool,
) {
    feature_combo(ui, id, label, doc, sel, pred);
}

/// An origin plane (with an offset) or a construction plane.
fn plane_input_ui(ui: &mut egui::Ui, id: &str, label: &str, doc: &Document, p: &mut PlaneInput) {
    row(ui, label, |ui| {
        ui.selectable_value(&mut p.which, 0, "XY");
        ui.selectable_value(&mut p.which, 1, "YZ");
        ui.selectable_value(&mut p.which, 2, "XZ");
        ui.selectable_value(&mut p.which, 3, "Plane");
    });
    if p.which == 3 {
        if p.datum.is_none() {
            p.datum = datum_ids(doc, |s| matches!(s, FeatureSpec::DatumPlane(_)))
                .first()
                .copied();
        }
        datum_combo(ui, id, "", doc, &mut p.datum, |s| {
            matches!(s, FeatureSpec::DatumPlane(_))
        });
    } else {
        text_field(ui, "  offset", &mut p.offset);
    }
}

/// A point by coordinates, or a construction point.
fn point_input_ui(ui: &mut egui::Ui, id: &str, label: &str, doc: &Document, p: &mut PointInput) {
    let points = datum_ids(doc, |s| matches!(s, FeatureSpec::DatumPoint(_)));
    row(ui, label, |ui| {
        let mut use_datum = p.datum.is_some();
        ui.selectable_value(&mut use_datum, false, "Coordinates");
        if !points.is_empty() {
            ui.selectable_value(&mut use_datum, true, "Point");
        }
        if use_datum && p.datum.is_none() {
            p.datum = points.first().copied();
        }
        if !use_datum {
            p.datum = None;
        }
    });
    if p.datum.is_some() {
        datum_combo(ui, id, "", doc, &mut p.datum, |s| {
            matches!(s, FeatureSpec::DatumPoint(_))
        });
    } else {
        text_field(ui, "  x", &mut p.x);
        text_field(ui, "  y", &mut p.y);
        text_field(ui, "  z", &mut p.z);
    }
}

/// A drop-down of the standard thread sizes, grouped by series.
fn thread_size_ui(ui: &mut egui::Ui, id: &str, designation: &mut String) {
    use kw_feature::threads::{self, ThreadFamily};
    row(ui, "Size", |ui| {
        egui::ComboBox::from_id_salt(id)
            .selected_text(designation.as_str())
            .show_ui(ui, |ui| {
                for fam in [
                    ThreadFamily::MetricCoarse,
                    ThreadFamily::MetricFine,
                    ThreadFamily::Unc,
                    ThreadFamily::Unf,
                ] {
                    ui.label(egui::RichText::new(fam.label()).color(TEXT_WEAK).size(10.5));
                    for s in threads::family(fam) {
                        ui.selectable_value(designation, s.designation.to_string(), s.designation);
                    }
                }
            });
    });
}

fn op_combo(ui: &mut egui::Ui, id: &str, op: &mut BodyOp) {
    row(ui, "Operation", |ui| {
        egui::ComboBox::from_id_salt(id)
            .selected_text(op_label(*op))
            .width(150.0)
            .show_ui(ui, |ui| {
                for o in [BodyOp::New, BodyOp::Join, BodyOp::Cut, BodyOp::Intersect] {
                    ui.selectable_value(op, o, op_label(o));
                }
            });
    });
}

/// Solid or surface: a surface is always a new body, so the operation row is not needed.
fn output_row(ui: &mut egui::Ui, surface: &mut bool, op: &mut BodyOp) {
    row(ui, "Output", |ui| {
        ui.selectable_value(surface, false, "Solid");
        ui.selectable_value(surface, true, "Surface");
    });
    if *surface {
        *op = BodyOp::New;
    }
}

fn is_sketch(s: &FeatureSpec) -> bool {
    matches!(s, FeatureSpec::Sketch(_))
}

fn is_tool_feature(s: &FeatureSpec) -> bool {
    matches!(
        s,
        FeatureSpec::Extrude(_) | FeatureSpec::Revolve(_) | FeatureSpec::Hole(_)
    )
}

impl Tool {
    pub fn extrude(sketch: Option<FeatureId>, op: BodyOp) -> Tool {
        Tool::Extrude {
            sketch,
            depth: "10 mm".into(),
            op,
            symmetric: false,
            flip: false,
            through_all: false,
            second: String::new(),
            taper: String::new(),
            more: ExtrudeMore::default(),
        }
    }

    pub fn title(&self) -> &'static str {
        match self {
            Tool::Extrude { .. } => "EXTRUDE",
            Tool::Revolve { .. } => "REVOLVE",
            Tool::Rib { .. } => "RIB",
            Tool::Pipe { .. } => "PIPE",
            Tool::Hole { .. } => "HOLE",
            Tool::Fillet { .. } => "FILLET",
            Tool::Chamfer { .. } => "CHAMFER",
            Tool::VariableFillet { .. } => "VARIABLE FILLET",
            Tool::Shell { .. } => "SHELL",
            Tool::Pattern { .. } => "RECTANGULAR PATTERN",
            Tool::CircularPattern { .. } => "CIRCULAR PATTERN",
            Tool::Mirror { .. } => "MIRROR",
            Tool::Combine { .. } => "COMBINE",
            Tool::Loft { .. } => "LOFT",
            Tool::Sweep { .. } => "SWEEP",
            Tool::Patch { .. } => "PATCH",
            Tool::Ruled => "RULED SURFACE",
            Tool::Stitch { .. } => "STITCH",
            Tool::Unstitch => "UNSTITCH",
            Tool::Thicken { .. } => "THICKEN",
            Tool::OffsetSurface { .. } => "OFFSET SURFACE",
            Tool::Draft { .. } => "DRAFT",
            Tool::Thread { .. } => "THREAD",
            Tool::SheetMetal { .. } => "SHEET METAL",
            Tool::Form { .. } => "FORM",
            Tool::Offset { .. } => "OFFSET BODY",
            Tool::ConstructionPlane { .. } => "CONSTRUCTION PLANE",
            Tool::Axis { .. } => "CONSTRUCTION AXIS",
            Tool::DatumPoint { .. } => "CONSTRUCTION POINT",
            Tool::MoveBody { copy: true, .. } => "COPY BODY",
            Tool::MoveBody { .. } => "MOVE BODY",
            Tool::ScaleBody { .. } => "SCALE BODY",
            Tool::SplitBody { .. } => "SPLIT BODY",
        }
    }

    /// Base of the feature's name ("Extrude" → "Extrude 3").
    pub fn name_base(&self) -> &'static str {
        match self {
            Tool::Extrude { .. } => "Extrude",
            Tool::Revolve { .. } => "Revolve",
            Tool::Rib { .. } => "Rib",
            Tool::Pipe { .. } => "Pipe",
            Tool::Hole { .. } => "Hole",
            Tool::Fillet { .. } => "Fillet",
            Tool::Chamfer { .. } => "Chamfer",
            Tool::VariableFillet { .. } => "Variable Fillet",
            Tool::Shell { .. } => "Shell",
            Tool::Pattern { .. } => "Pattern",
            Tool::CircularPattern { .. } => "Circular Pattern",
            Tool::Mirror { .. } => "Mirror",
            Tool::Combine { .. } => "Combine",
            Tool::Loft { .. } => "Loft",
            Tool::Sweep { .. } => "Sweep",
            Tool::Patch { .. } => "Patch",
            Tool::Ruled => "Ruled",
            Tool::Stitch { .. } => "Stitch",
            Tool::Unstitch => "Unstitch",
            Tool::Thicken { .. } => "Thicken",
            Tool::OffsetSurface { .. } => "Offset Surface",
            Tool::Draft { .. } => "Draft",
            Tool::Thread { .. } => "Thread",
            Tool::SheetMetal { .. } => "Sheet Metal",
            Tool::Form { .. } => "Form",
            Tool::Offset { .. } => "Offset",
            Tool::ConstructionPlane { .. } => "Plane",
            Tool::Axis { .. } => "Axis",
            Tool::DatumPoint { .. } => "Point",
            Tool::MoveBody { copy: true, .. } => "Copy",
            Tool::MoveBody { .. } => "Move",
            Tool::ScaleBody { .. } => "Scale",
            Tool::SplitBody { .. } => "Split",
        }
    }

    /// The sketches this tool builds from (what disappears from the view once the feature is made).
    pub fn sketches(&self) -> Vec<FeatureId> {
        match self {
            Tool::Extrude { sketch, .. }
            | Tool::Revolve { sketch, .. }
            | Tool::Rib { sketch, .. } => sketch.iter().copied().collect(),
            Tool::Pipe { path, .. } => path.iter().copied().collect(),
            Tool::Loft { sketches, .. } => sketches.iter().flatten().copied().collect(),
            Tool::Sweep {
                profile,
                path,
                guide,
                ..
            } => profile
                .iter()
                .chain(path.iter())
                .chain(guide.iter())
                .copied()
                .collect(),
            _ => Vec::new(),
        }
    }

    /// Does this tool take faces or edges from the viewport?
    pub fn picks_edges(&self) -> bool {
        matches!(
            self,
            Tool::Fillet { .. }
                | Tool::Chamfer { .. }
                | Tool::VariableFillet { .. }
                | Tool::Patch { .. }
                | Tool::Ruled
                | Tool::Revolve {
                    axis_edge: true,
                    ..
                }
                | Tool::Axis { mode: 1, .. }
                | Tool::DatumPoint { mode: 1, .. }
        )
    }
    pub fn picks_faces(&self) -> bool {
        matches!(
            self,
            Tool::Shell { .. }
                | Tool::Hole { .. }
                | Tool::Draft { .. }
                | Tool::Thread { .. }
                | Tool::SplitBody { plane: 3, .. }
                | Tool::Mirror { plane: 4, .. }
                | Tool::Axis { mode: 2, .. }
                | Tool::Extrude {
                    more: ExtrudeMore { to_face: true, .. },
                    ..
                }
        )
    }

    /// The dialog that edits an existing feature (`None` for kinds without one).
    pub fn for_feature(spec: &FeatureSpec) -> Option<Tool> {
        Some(match spec {
            FeatureSpec::Extrude(x) => Tool::Extrude {
                sketch: Some(x.sketch),
                depth: param_text(&x.depth),
                op: x.op,
                symmetric: x.symmetric,
                flip: x.flip,
                through_all: x.extent.through_all,
                second: x
                    .extent
                    .second_side
                    .as_ref()
                    .map(param_text)
                    .unwrap_or_default(),
                taper: x.extent.taper.as_ref().map(param_text).unwrap_or_default(),
                more: ExtrudeMore {
                    to_face: x.extent.to_face.is_some(),
                    thin: x
                        .extent
                        .thin
                        .as_ref()
                        .map(|t| param_text(&t.thickness))
                        .unwrap_or_default(),
                    thin_side: match x.extent.thin.as_ref().map(|t| t.side) {
                        Some(kw_feature::ThinSide::Outside) => 1,
                        Some(kw_feature::ThinSide::Centered) => 2,
                        _ => 0,
                    },
                    start: x
                        .extent
                        .start_offset
                        .as_ref()
                        .map(param_text)
                        .unwrap_or_default(),
                    regions: x.extent.regions.clone(),
                    surface: x.surface,
                },
            },
            FeatureSpec::Revolve(x) => Tool::Revolve {
                sketch: Some(x.sketch),
                angle: param_text(&x.angle),
                axis_y: matches!(x.axis, AxisSpec::SketchY),
                axis_edge: matches!(x.axis, AxisSpec::Edge(_)),
                axis_datum: match x.axis {
                    AxisSpec::Datum(d) => Some(d),
                    _ => None,
                },
                op: x.op,
                surface: x.surface,
            },
            FeatureSpec::Rib(x) => Tool::Rib {
                sketch: Some(x.sketch),
                thickness: param_text(&x.thickness),
                depth: param_text(&x.depth),
                symmetric: x.symmetric,
                flip: x.flip,
                op: x.op,
            },
            FeatureSpec::Pipe(x) => Tool::Pipe {
                path: Some(x.path),
                diameter: param_text(&x.diameter),
                wall: param_text(&x.wall),
                op: x.op,
            },
            FeatureSpec::Fillet(x) => Tool::Fillet {
                radius: param_text(&x.radius),
            },
            FeatureSpec::Chamfer(x) => Tool::Chamfer {
                d1: param_text(&x.d1),
                d2: param_text(&x.d2),
            },
            FeatureSpec::VariableFillet(x) => Tool::VariableFillet {
                start: param_text(&x.start),
                end: param_text(&x.end),
            },
            FeatureSpec::Shell(x) => Tool::Shell {
                thickness: param_text(&x.thickness),
            },
            FeatureSpec::LinearPattern(x) => Tool::Pattern {
                source: Some(x.source),
                dx: param_text(&x.spacing[0]),
                dy: param_text(&x.spacing[1]),
                dz: param_text(&x.spacing[2]),
                count: param_text(&x.count),
            },
            FeatureSpec::CircularPattern(x) => {
                let axis = if x.axis_feature.is_some() {
                    3
                } else if x.axis_dir[0].abs() > 0.5 {
                    0
                } else if x.axis_dir[1].abs() > 0.5 {
                    1
                } else {
                    2
                };
                Tool::CircularPattern {
                    source: Some(x.source),
                    count: param_text(&x.count),
                    angle: param_text(&x.total_angle),
                    axis,
                    datum: x.axis_feature,
                }
            }
            FeatureSpec::Mirror(x) => Tool::Mirror {
                source: Some(x.source),
                plane: match x.plane {
                    MirrorPlane::Xy(_) => 0,
                    MirrorPlane::Yz(_) => 1,
                    MirrorPlane::Xz(_) => 2,
                    MirrorPlane::Datum(_) => 3,
                    MirrorPlane::Face(_) => 4,
                },
                datum: match x.plane {
                    MirrorPlane::Datum(d) => Some(d),
                    _ => None,
                },
            },
            FeatureSpec::Combine(x) => Tool::Combine {
                target: Some(x.target),
                tool: Some(x.tool),
                op: x.op,
                keep_tool: x.keep_tool,
            },
            FeatureSpec::Loft(x) => Tool::Loft {
                sketches: x.sketches.iter().map(|s| Some(*s)).collect(),
                op: x.op,
                surface: x.surface,
                ruled: x.ruled,
            },
            FeatureSpec::Patch(x) => Tool::Patch {
                tangent: x.edges.iter().any(|e| e.tangent),
            },
            FeatureSpec::Ruled(_) => Tool::Ruled,
            FeatureSpec::Stitch(x) => Tool::Stitch {
                tolerance: param_text(&x.tolerance),
            },
            FeatureSpec::Unstitch(_) => Tool::Unstitch,
            FeatureSpec::Thicken(x) => Tool::Thicken {
                thickness: param_text(&x.thickness),
                side: match x.side {
                    kw_feature::ThickenDir::Normal => 0,
                    kw_feature::ThickenDir::Reverse => 1,
                    kw_feature::ThickenDir::Both => 2,
                },
            },
            FeatureSpec::OffsetSurface(x) => Tool::OffsetSurface {
                distance: param_text(&x.distance),
            },
            FeatureSpec::Sweep(x) => Tool::Sweep {
                profile: Some(x.profile),
                path: Some(x.path),
                op: x.op,
                surface: x.surface,
                twist: x.twist.as_ref().map(param_text).unwrap_or_default(),
                guide: x.guide,
            },
            FeatureSpec::Draft(x) => {
                let axis = (0..3)
                    .max_by(|a, b| x.pull[*a].abs().total_cmp(&x.pull[*b].abs()))
                    .unwrap_or(2) as u8;
                Tool::Draft {
                    angle: param_text(&x.angle),
                    axis,
                    position: param_text(&x.neutral_origin[axis as usize]),
                }
            }
            FeatureSpec::MoveBody(x) => Tool::MoveBody {
                copy: x.copy,
                dx: param_text(&x.delta[0]),
                dy: param_text(&x.delta[1]),
                dz: param_text(&x.delta[2]),
                axis: x.axis,
                angle: param_text(&x.angle),
            },
            FeatureSpec::ScaleBody(x) => Tool::ScaleBody {
                factor: param_text(&x.factor),
                cx: param_text(&x.centre[0]),
                cy: param_text(&x.centre[1]),
                cz: param_text(&x.centre[2]),
            },
            FeatureSpec::SplitBody(x) => match &x.plane {
                PlaneSpec::Xy(o) => Tool::SplitBody {
                    plane: 0,
                    offset: param_text(o),
                },
                PlaneSpec::Yz(o) => Tool::SplitBody {
                    plane: 1,
                    offset: param_text(o),
                },
                PlaneSpec::Xz(o) => Tool::SplitBody {
                    plane: 2,
                    offset: param_text(o),
                },
                _ => Tool::SplitBody {
                    plane: 3,
                    offset: "0 mm".into(),
                },
            },
            FeatureSpec::Offset(x) => Tool::Offset {
                distance: param_text(&x.distance),
            },
            FeatureSpec::DatumPlane(x) => Tool::ConstructionPlane {
                base: match x.base {
                    PlaneSpec::Yz(_) => 1,
                    PlaneSpec::Xz(_) => 2,
                    _ => 0,
                },
                offset: param_text(&x.offset),
                tilt: param_text(&x.tilt),
                more: match &x.mode {
                    kw_feature::PlaneMode::Offset => PlaneMore::default(),
                    kw_feature::PlaneMode::Midplane(o) => PlaneMore {
                        mode: 1,
                        other: PlaneInput::from_spec(o),
                        ..Default::default()
                    },
                    kw_feature::PlaneMode::ThreePoints(p) => PlaneMore {
                        mode: 2,
                        pts: [
                            PointInput::from_src(&p[0]),
                            PointInput::from_src(&p[1]),
                            PointInput::from_src(&p[2]),
                        ],
                        ..Default::default()
                    },
                },
            },
            FeatureSpec::DatumAxis(x) => match &x.kind {
                kw_feature::AxisKind::TwoPoints(p) => Tool::Axis {
                    mode: 0,
                    pts: [PointInput::from_src(&p[0]), PointInput::from_src(&p[1])],
                    at_u: "0 mm".into(),
                    at_v: "0 mm".into(),
                    planes: Default::default(),
                },
                kw_feature::AxisKind::Edge(_) => Tool::Axis {
                    mode: 1,
                    pts: Default::default(),
                    at_u: "0 mm".into(),
                    at_v: "0 mm".into(),
                    planes: Default::default(),
                },
                kw_feature::AxisKind::NormalToFace { at, .. } => Tool::Axis {
                    mode: 2,
                    pts: Default::default(),
                    at_u: param_text(&at[0]),
                    at_v: param_text(&at[1]),
                    planes: Default::default(),
                },
                kw_feature::AxisKind::PlanePlane(pp) => Tool::Axis {
                    mode: 3,
                    pts: Default::default(),
                    at_u: "0 mm".into(),
                    at_v: "0 mm".into(),
                    planes: [PlaneInput::from_spec(&pp.0), PlaneInput::from_spec(&pp.1)],
                },
            },
            FeatureSpec::DatumPoint(x) => match &x.kind {
                kw_feature::PointKind::Coords(c) => Tool::DatumPoint {
                    mode: 0,
                    coords: PointInput {
                        datum: None,
                        x: param_text(&c[0]),
                        y: param_text(&c[1]),
                        z: param_text(&c[2]),
                    },
                    axis: None,
                    plane: Default::default(),
                },
                kw_feature::PointKind::EdgeMidpoint(_) => Tool::DatumPoint {
                    mode: 1,
                    coords: Default::default(),
                    axis: None,
                    plane: Default::default(),
                },
                kw_feature::PointKind::AxisPlane { axis, plane } => Tool::DatumPoint {
                    mode: 2,
                    coords: Default::default(),
                    axis: Some(*axis),
                    plane: PlaneInput::from_spec(plane),
                },
            },
            FeatureSpec::Hole(x) => {
                let (kind, d, cbd, cbh, csd, csa) = match &x.kind {
                    HoleKind::Simple { diameter } => (
                        0,
                        param_text(diameter),
                        "16 mm".into(),
                        "3 mm".into(),
                        "16 mm".into(),
                        "90 deg".into(),
                    ),
                    HoleKind::Counterbore {
                        diameter,
                        cb_diameter,
                        cb_depth,
                    } => (
                        1,
                        param_text(diameter),
                        param_text(cb_diameter),
                        param_text(cb_depth),
                        "16 mm".into(),
                        "90 deg".into(),
                    ),
                    HoleKind::Countersink {
                        diameter,
                        cs_diameter,
                        cs_angle,
                    } => (
                        2,
                        param_text(diameter),
                        "16 mm".into(),
                        "3 mm".into(),
                        param_text(cs_diameter),
                        param_text(cs_angle),
                    ),
                    HoleKind::Tapped { .. } => (
                        3,
                        "6 mm".into(),
                        "16 mm".into(),
                        "3 mm".into(),
                        "16 mm".into(),
                        "90 deg".into(),
                    ),
                };
                let (thread, thread_length) = match &x.kind {
                    HoleKind::Tapped {
                        designation,
                        thread_length,
                    } => (designation.clone(), param_text(thread_length)),
                    _ => ("M6".into(), "8 mm".into()),
                };
                let (px, py) = x
                    .points
                    .first()
                    .map(|p| (param_text(&p[0]), param_text(&p[1])))
                    .unwrap_or(("0 mm".into(), "0 mm".into()));
                Tool::Hole {
                    x: px,
                    y: py,
                    diameter: d,
                    depth: param_text(&x.depth),
                    kind,
                    cb_diameter: cbd,
                    cb_depth: cbh,
                    cs_diameter: csd,
                    cs_angle: csa,
                    through_all: x.through_all,
                    thread,
                    thread_length,
                }
            }
            FeatureSpec::SheetMetal(x) => Tool::SheetMetal {
                thickness: param_text(&x.thickness),
                k_factor: param_text(&x.k_factor),
                radius: param_text(&x.default_radius),
                width: param_text(&x.width),
                rows: x
                    .segs
                    .iter()
                    .map(|g| match g {
                        kw_feature::SheetSegSpec::Flat { length } => SheetRow {
                            bend: false,
                            a: param_text(length),
                            r: String::new(),
                        },
                        kw_feature::SheetSegSpec::Bend { angle, radius } => SheetRow {
                            bend: true,
                            a: param_text(angle),
                            r: radius.as_ref().map(param_text).unwrap_or_default(),
                        },
                    })
                    .collect(),
            },
            FeatureSpec::Subd(x) => Tool::Form {
                kind: 255,
                size: [20.0; 3],
                sides: 8,
                cage: x.cage.clone(),
                scale: param_text(&x.scale),
                tolerance: param_text(&x.tolerance),
                vertex: 0,
                built: (255, [20.0; 3], 8),
            },
            FeatureSpec::Thread(x) => Tool::Thread {
                designation: x.designation.clone(),
                length: param_text(&x.length),
                internal: x.internal,
            },
            _ => return None,
        })
    }

    pub fn default_hole() -> Tool {
        Tool::Hole {
            x: "0 mm".into(),
            y: "0 mm".into(),
            diameter: "6 mm".into(),
            depth: "10 mm".into(),
            kind: 0,
            cb_diameter: "12 mm".into(),
            cb_depth: "3 mm".into(),
            cs_diameter: "12 mm".into(),
            cs_angle: "90 deg".into(),
            through_all: false,
            thread: "M6".into(),
            thread_length: "8 mm".into(),
        }
    }

    /// Build the feature spec. When editing, the parts that came from picks keep their original values.
    pub fn spec(&self, ctx: &BuildCtx, orig: Option<&FeatureSpec>) -> Result<FeatureSpec, String> {
        let len = |t: &str| parse_param(t, Dim::LENGTH);
        let none = |t: &str| parse_param(t, Dim::NONE);
        let ang = |t: &str| parse_param(t, Dim::ANGLE);
        let need = |f: &Option<FeatureId>, what: &str| f.ok_or(format!("choose {what}"));
        let face_refs = |ctx: &BuildCtx| -> Result<Vec<kw_feature::Reference>, String> {
            let (brep, names) = ctx.base.ok_or("there is no solid to work on")?;
            let faces: Vec<kw_brep::FaceId> = ctx
                .picks
                .iter()
                .filter_map(|p| {
                    if let Pick::Face(f) = p {
                        Some(*f)
                    } else {
                        None
                    }
                })
                .collect();
            faces
                .iter()
                .map(|f| {
                    names
                        .reference_to(brep, *f)
                        .ok_or_else(|| "a picked face has no persistent name".to_string())
                })
                .collect()
        };
        Ok(match self {
            Tool::Extrude {
                sketch,
                depth,
                op,
                symmetric,
                flip,
                through_all,
                second,
                taper,
                more,
            } => {
                let base = match orig {
                    Some(FeatureSpec::Extrude(o)) => o.base,
                    _ => None,
                };
                let opt = |t: &str, dim| -> Result<Option<kw_expr::Param>, String> {
                    if t.trim().is_empty() {
                        Ok(None)
                    } else {
                        parse_param(t, dim).map(Some)
                    }
                };
                let to_face = if more.to_face {
                    Some(match orig {
                        Some(FeatureSpec::Extrude(o))
                            if ctx.picks.is_empty() && o.extent.to_face.is_some() =>
                        {
                            o.extent.to_face.clone().unwrap()
                        }
                        _ => face_refs(ctx)?
                            .into_iter()
                            .next()
                            .ok_or("click the planar face to stop at")?,
                    })
                } else {
                    None
                };
                let thin = opt(&more.thin, Dim::LENGTH)?.map(|t| kw_feature::ThinWall {
                    thickness: t,
                    side: match more.thin_side {
                        1 => kw_feature::ThinSide::Outside,
                        2 => kw_feature::ThinSide::Centered,
                        _ => kw_feature::ThinSide::Inside,
                    },
                });
                if more.surface {
                    return Ok(FeatureSpec::Extrude(ExtrudeSpec {
                        surface: true,
                        sketch: need(sketch, "a sketch")?,
                        depth: len(depth)?,
                        symmetric: *symmetric,
                        flip: *flip,
                        op: BodyOp::New,
                        base,
                        extent: Default::default(),
                    }));
                }
                FeatureSpec::Extrude(ExtrudeSpec {
                    surface: false,
                    sketch: need(sketch, "a sketch")?,
                    depth: if *through_all || more.to_face {
                        // not used: the tool is as long as the body (or the face) needs
                        len("1 mm")?
                    } else {
                        len(depth)?
                    },
                    symmetric: *symmetric,
                    flip: *flip,
                    op: *op,
                    base,
                    extent: kw_feature::ExtrudeExtent {
                        through_all: *through_all,
                        second_side: opt(second, Dim::LENGTH)?,
                        taper: opt(taper, Dim::ANGLE)?,
                        thin,
                        to_face,
                        start_offset: opt(&more.start, Dim::LENGTH)?,
                        regions: more.regions.clone(),
                    },
                })
            }
            Tool::Rib {
                sketch,
                thickness,
                depth,
                symmetric,
                flip,
                op,
            } => FeatureSpec::Rib(kw_feature::RibSpec {
                sketch: need(sketch, "a sketch")?,
                thickness: len(thickness)?,
                depth: len(depth)?,
                symmetric: *symmetric,
                flip: *flip,
                op: *op,
                base: None,
            }),
            Tool::Pipe {
                path,
                diameter,
                wall,
                op,
            } => FeatureSpec::Pipe(kw_feature::PipeSpec {
                path: need(path, "a path sketch")?,
                diameter: len(diameter)?,
                wall: len(wall)?,
                op: *op,
                base: None,
            }),
            Tool::Revolve {
                sketch,
                angle,
                axis_y,
                axis_edge,
                axis_datum,
                op,
                surface,
            } => FeatureSpec::Revolve(RevolveSpec {
                surface: *surface,
                sketch: need(sketch, "a sketch")?,
                axis: if let Some(d) = axis_datum {
                    AxisSpec::Datum(*d)
                } else if *axis_edge {
                    match orig {
                        Some(FeatureSpec::Revolve(RevolveSpec {
                            axis: AxisSpec::Edge(e),
                            ..
                        })) if ctx.picks.is_empty() => AxisSpec::Edge(e.clone()),
                        _ => {
                            let mut e = edge_refs(ctx)?;
                            if e.len() != 1 {
                                return Err("pick exactly one straight edge as the axis".into());
                            }
                            AxisSpec::Edge(Box::new(e.remove(0)))
                        }
                    }
                } else if *axis_y {
                    AxisSpec::SketchY
                } else {
                    AxisSpec::SketchX
                },
                angle: ang(angle)?,
                op: if *surface { BodyOp::New } else { *op },
                base: match orig {
                    Some(FeatureSpec::Revolve(o)) => o.base,
                    _ => None,
                },
            }),
            Tool::Fillet { radius } => {
                let edges = match orig {
                    Some(FeatureSpec::Fillet(o)) if ctx.picks.is_empty() => o.edges.clone(),
                    _ => edge_refs(ctx)?,
                };
                FeatureSpec::Fillet(FilletSpec {
                    edges,
                    radius: len(radius)?,
                    base: None,
                })
            }
            Tool::Chamfer { d1, d2 } => {
                let edges = match orig {
                    Some(FeatureSpec::Chamfer(o)) if ctx.picks.is_empty() => o.edges.clone(),
                    _ => edge_refs(ctx)?,
                };
                FeatureSpec::Chamfer(ChamferSpec {
                    edges,
                    d1: len(d1)?,
                    d2: len(d2)?,
                    base: None,
                })
            }
            Tool::VariableFillet { start, end } => {
                let edge = match orig {
                    Some(FeatureSpec::VariableFillet(o)) if ctx.picks.is_empty() => o.edge.clone(),
                    _ => {
                        let mut e = edge_refs(ctx)?;
                        if e.len() != 1 {
                            return Err("a variable fillet takes exactly one edge".into());
                        }
                        e.remove(0)
                    }
                };
                FeatureSpec::VariableFillet(kw_feature::VariableFilletSpec {
                    edge,
                    start: len(start)?,
                    end: len(end)?,
                    base: None,
                })
            }
            Tool::Shell { thickness } => {
                let open = match orig {
                    Some(FeatureSpec::Shell(o)) if ctx.picks.is_empty() => o.open.clone(),
                    _ => face_refs(ctx)?,
                };
                FeatureSpec::Shell(ShellSpec {
                    thickness: len(thickness)?,
                    open,
                    base: None,
                })
            }
            Tool::Hole {
                x,
                y,
                diameter,
                depth,
                kind,
                cb_diameter,
                cb_depth,
                cs_diameter,
                cs_angle,
                through_all,
                thread,
                thread_length,
            } => {
                let plane = match (orig, &ctx.hole_plane) {
                    (_, Some(p)) => p.clone(),
                    (Some(FeatureSpec::Hole(o)), None) => o.plane.clone(),
                    _ => return Err("click a planar face to place the hole on".into()),
                };
                // a tapped hole takes its diameter from the thread table
                let d = if *kind == 3 {
                    len("6 mm")?
                } else {
                    len(diameter)?
                };
                let kind = match kind {
                    1 => HoleKind::Counterbore {
                        diameter: d,
                        cb_diameter: len(cb_diameter)?,
                        cb_depth: len(cb_depth)?,
                    },
                    2 => HoleKind::Countersink {
                        diameter: d,
                        cs_diameter: len(cs_diameter)?,
                        cs_angle: ang(cs_angle)?,
                    },
                    3 => HoleKind::Tapped {
                        designation: thread.clone(),
                        thread_length: len(thread_length)?,
                    },
                    _ => HoleKind::Simple { diameter: d },
                };
                FeatureSpec::Hole(HoleSpec {
                    plane,
                    points: vec![[len(x)?, len(y)?]],
                    kind,
                    depth: len(depth)?,
                    base: None,
                    through_all: *through_all,
                })
            }
            Tool::Pattern {
                source,
                dx,
                dy,
                dz,
                count,
            } => FeatureSpec::LinearPattern(LinearPatternSpec {
                source: need(source, "a feature to pattern")?,
                spacing: [len(dx)?, len(dy)?, len(dz)?],
                count: none(count)?,
                base: None,
            }),
            Tool::CircularPattern {
                source,
                count,
                angle,
                axis,
                datum,
            } => {
                let mut dir = [0.0; 3];
                dir[(*axis as usize).min(2)] = 1.0;
                FeatureSpec::CircularPattern(CircularPatternSpec {
                    source: need(source, "a feature to pattern")?,
                    axis_origin: [Param::length(0.0), Param::length(0.0), Param::length(0.0)],
                    axis_dir: dir,
                    count: none(count)?,
                    total_angle: ang(angle)?,
                    base: None,
                    axis_feature: if *axis == 3 {
                        Some(datum.ok_or("choose a construction axis")?)
                    } else {
                        None
                    },
                })
            }
            Tool::Mirror {
                source,
                plane,
                datum,
            } => {
                let z = Param::length(0.0);
                FeatureSpec::Mirror(MirrorSpec {
                    source: need(source, "a feature to mirror")?,
                    plane: match plane {
                        0 => MirrorPlane::Xy(z),
                        1 => MirrorPlane::Yz(z),
                        2 => MirrorPlane::Xz(z),
                        3 => MirrorPlane::Datum(datum.ok_or("choose a construction plane")?),
                        _ => MirrorPlane::Face(match orig {
                            Some(FeatureSpec::Mirror(MirrorSpec {
                                plane: MirrorPlane::Face(r),
                                ..
                            })) if ctx.picks.is_empty() => r.clone(),
                            _ => face_refs(ctx)?
                                .into_iter()
                                .next()
                                .ok_or("click the planar face to mirror across")?,
                        }),
                    },
                    base: None,
                })
            }
            Tool::Combine {
                target,
                tool,
                op,
                keep_tool,
            } => FeatureSpec::Combine(CombineSpec {
                target: need(target, "a target body")?,
                tool: need(tool, "a tool body")?,
                op: *op,
                keep_tool: *keep_tool,
            }),
            Tool::Loft {
                sketches,
                op,
                surface,
                ruled,
            } => {
                let picked: Vec<FeatureId> = sketches.iter().flatten().copied().collect();
                if picked.len() < 2 {
                    return Err("a loft needs at least two profile sketches".into());
                }
                FeatureSpec::Loft(LoftSpec {
                    surface: *surface,
                    ruled: *surface && *ruled,
                    sketches: picked,
                    op: if *surface { BodyOp::New } else { *op },
                    base: None,
                })
            }
            Tool::Sweep {
                profile,
                path,
                op,
                surface,
                twist,
                guide,
            } => FeatureSpec::Sweep(SweepSpec {
                surface: *surface,
                twist: if twist.trim().is_empty() {
                    None
                } else {
                    Some(ang(twist)?)
                },
                guide: *guide,
                profile: need(profile, "a profile sketch")?,
                path: need(path, "a path sketch")?,
                op: if *surface { BodyOp::New } else { *op },
                base: None,
            }),
            Tool::Patch { tangent } => {
                let make =
                    |source: FeatureId, edge: &kw_feature::BoundaryRef| kw_feature::BoundaryEdge {
                        source,
                        edge: edge.clone(),
                        tangent: *tangent && matches!(edge, kw_feature::BoundaryRef::Free { .. }),
                    };
                let edges: Vec<_> = if ctx.boundary.is_empty() {
                    match orig {
                        Some(FeatureSpec::Patch(o)) => {
                            o.edges.iter().map(|e| make(e.source, &e.edge)).collect()
                        }
                        _ => Vec::new(),
                    }
                } else {
                    ctx.boundary.iter().map(|(s, e)| make(*s, e)).collect()
                };
                if edges.len() < 3 {
                    return Err(
                        "click the edges that surround the hole: a patch needs at least three"
                            .into(),
                    );
                }
                FeatureSpec::Patch(kw_feature::PatchSpec { edges })
            }
            Tool::Stitch { tolerance } => {
                let bodies: Vec<FeatureId> = if !ctx.picked_bodies.is_empty() {
                    ctx.picked_bodies.to_vec()
                } else if let (Some(FeatureSpec::Stitch(o)), true) = (orig, ctx.surfaces.is_empty())
                {
                    o.bodies.clone()
                } else {
                    ctx.surfaces.to_vec()
                };
                if bodies.is_empty() {
                    return Err("there are no surfaces to stitch".into());
                }
                FeatureSpec::Stitch(kw_feature::StitchSpec {
                    bodies,
                    tolerance: len(tolerance)?,
                })
            }
            Tool::Unstitch => FeatureSpec::Unstitch(kw_feature::UnstitchSpec { base: None }),
            Tool::Thicken { thickness, side } => FeatureSpec::Thicken(kw_feature::ThickenSpec {
                thickness: len(thickness)?,
                side: match side {
                    1 => kw_feature::ThickenDir::Reverse,
                    2 => kw_feature::ThickenDir::Both,
                    _ => kw_feature::ThickenDir::Normal,
                },
                base: None,
            }),
            Tool::OffsetSurface { distance } => {
                FeatureSpec::OffsetSurface(kw_feature::OffsetSurfaceSpec {
                    distance: len(distance)?,
                    base: None,
                })
            }
            Tool::Ruled => {
                let pair: Vec<kw_feature::BoundaryEdge> = if ctx.boundary.is_empty() {
                    match orig {
                        Some(FeatureSpec::Ruled(o)) => vec![o.a.clone(), o.b.clone()],
                        _ => Vec::new(),
                    }
                } else {
                    ctx.boundary
                        .iter()
                        .map(|(source, edge)| kw_feature::BoundaryEdge {
                            source: *source,
                            edge: edge.clone(),
                            tangent: false,
                        })
                        .collect()
                };
                let [a, b] = <[kw_feature::BoundaryEdge; 2]>::try_from(pair)
                    .map_err(|_| "click exactly two edges".to_string())?;
                FeatureSpec::Ruled(kw_feature::RuledSpec { a, b })
            }
            Tool::Draft {
                angle,
                axis,
                position,
            } => {
                let faces = match orig {
                    Some(FeatureSpec::Draft(o)) if ctx.picks.is_empty() => o.faces.clone(),
                    _ => face_refs(ctx)?,
                };
                let axis = (*axis as usize).min(2);
                let mut pull = [0.0; 3];
                pull[axis] = 1.0;
                let mut origin = [Param::length(0.0), Param::length(0.0), Param::length(0.0)];
                origin[axis] = len(position)?;
                FeatureSpec::Draft(DraftSpec {
                    faces,
                    neutral_origin: origin,
                    pull,
                    angle: ang(angle)?,
                    base: None,
                })
            }
            Tool::SheetMetal {
                thickness,
                k_factor,
                radius,
                width,
                rows,
            } => {
                if rows.is_empty() {
                    return Err("add at least one flat or bend".into());
                }
                let mut segs = Vec::new();
                for r in rows {
                    segs.push(if r.bend {
                        kw_feature::SheetSegSpec::Bend {
                            angle: ang(&r.a)?,
                            radius: if r.r.trim().is_empty() {
                                None
                            } else {
                                Some(len(&r.r)?)
                            },
                        }
                    } else {
                        kw_feature::SheetSegSpec::Flat { length: len(&r.a)? }
                    });
                }
                FeatureSpec::SheetMetal(kw_feature::SheetMetalSpec {
                    thickness: len(thickness)?,
                    k_factor: none(k_factor)?,
                    default_radius: len(radius)?,
                    width: len(width)?,
                    segs,
                })
            }
            Tool::Form {
                cage,
                scale,
                tolerance,
                ..
            } => FeatureSpec::Subd(kw_feature::SubdSpec {
                cage: cage.clone(),
                scale: none(scale)?,
                tolerance: len(tolerance)?,
            }),
            Tool::Thread {
                designation,
                length,
                internal,
            } => FeatureSpec::Thread(kw_feature::ThreadSpec {
                face: match orig {
                    Some(FeatureSpec::Thread(o)) if ctx.picks.is_empty() => o.face.clone(),
                    _ => face_refs(ctx)?
                        .into_iter()
                        .next()
                        .ok_or("click the cylindrical face to thread")?,
                },
                designation: designation.clone(),
                length: len(length)?,
                internal: *internal,
                base: None,
            }),
            Tool::MoveBody {
                copy,
                dx,
                dy,
                dz,
                axis,
                angle,
            } => FeatureSpec::MoveBody(MoveBodySpec {
                base: None,
                delta: [len(dx)?, len(dy)?, len(dz)?],
                axis: (*axis).min(2),
                angle: ang(angle)?,
                copy: *copy,
            }),
            Tool::ScaleBody { factor, cx, cy, cz } => FeatureSpec::ScaleBody(ScaleBodySpec {
                base: None,
                factor: none(factor)?,
                centre: [len(cx)?, len(cy)?, len(cz)?],
            }),
            Tool::SplitBody { plane, offset } => {
                let spec = match plane {
                    0 => PlaneSpec::Xy(len(offset)?),
                    1 => PlaneSpec::Yz(len(offset)?),
                    2 => PlaneSpec::Xz(len(offset)?),
                    _ => match orig {
                        Some(FeatureSpec::SplitBody(o)) if ctx.picks.is_empty() => o.plane.clone(),
                        _ => PlaneSpec::Face(
                            face_refs(ctx)?
                                .into_iter()
                                .next()
                                .ok_or("click a planar face to cut along")?,
                        ),
                    },
                };
                FeatureSpec::SplitBody(SplitBodySpec {
                    base: None,
                    plane: spec,
                    keep: match orig {
                        Some(FeatureSpec::SplitBody(o)) => o.keep,
                        _ => SplitSide::Positive,
                    },
                })
            }
            Tool::Offset { distance } => FeatureSpec::Offset(OffsetSpec {
                distance: len(distance)?,
                base: None,
            }),
            Tool::ConstructionPlane {
                base,
                offset,
                tilt,
                more,
            } => {
                let zero = Param::length(0.0);
                let base_spec = match base {
                    1 => PlaneSpec::Yz(zero),
                    2 => PlaneSpec::Xz(zero),
                    _ => PlaneSpec::Xy(zero),
                };
                let mode = match more.mode {
                    1 => kw_feature::PlaneMode::Midplane(more.other.spec()?),
                    2 => kw_feature::PlaneMode::ThreePoints(Box::new([
                        more.pts[0].src()?,
                        more.pts[1].src()?,
                        more.pts[2].src()?,
                    ])),
                    _ => kw_feature::PlaneMode::Offset,
                };
                // the base of the first plane of a midplane is its own offset too
                let base_spec = if more.mode == 1 {
                    match base_spec {
                        PlaneSpec::Xy(_) => PlaneSpec::Xy(len(offset)?),
                        PlaneSpec::Yz(_) => PlaneSpec::Yz(len(offset)?),
                        other => match other {
                            PlaneSpec::Xz(_) => PlaneSpec::Xz(len(offset)?),
                            o => o,
                        },
                    }
                } else {
                    base_spec
                };
                FeatureSpec::DatumPlane(DatumPlaneSpec {
                    base: base_spec,
                    offset: if more.mode == 0 {
                        len(offset)?
                    } else {
                        len("0 mm")?
                    },
                    tilt: if more.mode == 0 {
                        ang(tilt)?
                    } else {
                        ang("0 deg")?
                    },
                    mode,
                })
            }
            Tool::Axis {
                mode,
                pts,
                at_u,
                at_v,
                planes,
            } => FeatureSpec::DatumAxis(kw_feature::DatumAxisSpec {
                kind: match mode {
                    0 => kw_feature::AxisKind::TwoPoints(Box::new([pts[0].src()?, pts[1].src()?])),
                    1 => match orig {
                        Some(FeatureSpec::DatumAxis(kw_feature::DatumAxisSpec {
                            kind: kw_feature::AxisKind::Edge(e),
                        })) if ctx.picks.is_empty() => kw_feature::AxisKind::Edge(e.clone()),
                        _ => {
                            let mut e = edge_refs(ctx)?;
                            if e.len() != 1 {
                                return Err("pick exactly one straight edge".into());
                            }
                            kw_feature::AxisKind::Edge(Box::new(e.remove(0)))
                        }
                    },
                    2 => kw_feature::AxisKind::NormalToFace {
                        face: match orig {
                            Some(FeatureSpec::DatumAxis(kw_feature::DatumAxisSpec {
                                kind: kw_feature::AxisKind::NormalToFace { face, .. },
                            })) if ctx.picks.is_empty() => face.clone(),
                            _ => face_refs(ctx)?
                                .into_iter()
                                .next()
                                .ok_or("click a planar face")?,
                        },
                        at: [len(at_u)?, len(at_v)?],
                    },
                    _ => kw_feature::AxisKind::PlanePlane(Box::new((
                        planes[0].spec()?,
                        planes[1].spec()?,
                    ))),
                },
            }),
            Tool::DatumPoint {
                mode,
                coords,
                axis,
                plane,
            } => FeatureSpec::DatumPoint(kw_feature::DatumPointSpec {
                kind: match mode {
                    0 => kw_feature::PointKind::Coords([
                        len(&coords.x)?,
                        len(&coords.y)?,
                        len(&coords.z)?,
                    ]),
                    1 => match orig {
                        Some(FeatureSpec::DatumPoint(kw_feature::DatumPointSpec {
                            kind: kw_feature::PointKind::EdgeMidpoint(e),
                        })) if ctx.picks.is_empty() => {
                            kw_feature::PointKind::EdgeMidpoint(e.clone())
                        }
                        _ => {
                            let mut e = edge_refs(ctx)?;
                            if e.len() != 1 {
                                return Err("pick exactly one edge".into());
                            }
                            kw_feature::PointKind::EdgeMidpoint(Box::new(e.remove(0)))
                        }
                    },
                    _ => kw_feature::PointKind::AxisPlane {
                        axis: axis.ok_or("choose a construction axis")?,
                        plane: plane.spec()?,
                    },
                },
            }),
        })
    }

    /// Draw the fields; returns whether anything changed.
    pub fn fields(
        &mut self,
        ui: &mut egui::Ui,
        doc: &Document,
        picks: &[Pick],
        regions: &[RegionInfo],
    ) -> bool {
        let before = self.clone();
        match self {
            Tool::Extrude {
                sketch,
                depth,
                op,
                symmetric,
                flip,
                through_all,
                second,
                taper,
                more,
            } => {
                feature_combo(ui, "ex_sk", "Profile", doc, sketch, is_sketch);
                output_row(ui, &mut more.surface, op);
                if more.surface {
                    row(ui, "Direction", |ui| {
                        ui.selectable_value(symmetric, false, "One side");
                        ui.selectable_value(symmetric, true, "Symmetric");
                    });
                    text_field(ui, "Distance", depth);
                    row(ui, "", |ui| {
                        ui.checkbox(flip, "Flip direction");
                    });
                    ui.label(
                        egui::RichText::new(
                            "Every curve of the sketch, open or closed, becomes part of the surface.",
                        )
                        .color(TEXT_WEAK)
                        .size(10.5),
                    );
                    return before != *self;
                }
                if regions.len() > 1 {
                    // the pieces the sketch's curves cut the plane into: click them in the view or tick them here
                    ui.add_space(2.0);
                    row(ui, "Regions", |ui| {
                        let chosen = regions.iter().filter(|r| !r.matching.is_empty()).count();
                        ui.label(format!("{chosen} of {} chosen", regions.len()));
                        if ui.small_button("Default").clicked() {
                            more.regions = regions
                                .iter()
                                .filter(|r| r.default)
                                .map(|r| r.pick.clone())
                                .collect();
                        }
                        if ui.small_button("All").clicked() {
                            more.regions = regions.iter().map(|r| r.pick.clone()).collect();
                        }
                        if ui.small_button("None").clicked() {
                            more.regions.clear();
                        }
                    });
                    for (i, r) in regions.iter().enumerate() {
                        let mut on = !r.matching.is_empty();
                        let before_on = on;
                        ui.horizontal(|ui| {
                            ui.add_space(92.0);
                            ui.checkbox(&mut on, format!("Region {} ({:.2} mm²)", i + 1, r.area));
                        });
                        if on != before_on {
                            if on {
                                more.regions.push(r.pick.clone());
                            } else {
                                for k in r.matching.iter().rev() {
                                    if *k < more.regions.len() {
                                        more.regions.remove(*k);
                                    }
                                }
                            }
                        }
                    }
                    if regions.iter().all(|r| r.matching.is_empty()) {
                        ui.label(
                            egui::RichText::new("Click the regions to extrude.")
                                .color(BAD)
                                .size(10.5),
                        );
                    }
                    ui.add_space(2.0);
                }
                row(ui, "Extent", |ui| {
                    let mut e = if *through_all {
                        1
                    } else if more.to_face {
                        2
                    } else {
                        0
                    };
                    ui.selectable_value(&mut e, 0, "Distance");
                    ui.selectable_value(&mut e, 1, "Through all");
                    ui.selectable_value(&mut e, 2, "To face");
                    *through_all = e == 1;
                    more.to_face = e == 2;
                });
                if more.to_face {
                    let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                    row(ui, "Face", |ui| {
                        ui.label(if n > 0 {
                            "face selected"
                        } else {
                            "click a planar face"
                        });
                    });
                }
                row(ui, "Direction", |ui| {
                    ui.selectable_value(symmetric, false, "One side");
                    ui.selectable_value(symmetric, true, "Symmetric");
                });
                if !*through_all && !more.to_face {
                    text_field(ui, "Distance", depth);
                }
                text_field(ui, "Start offset", &mut more.start);
                if !*symmetric {
                    text_field(ui, "Second side", second);
                }
                text_field(ui, "Taper angle", taper);
                text_field(ui, "Wall thickness", &mut more.thin);
                if !more.thin.trim().is_empty() {
                    row(ui, "Wall on", |ui| {
                        ui.selectable_value(&mut more.thin_side, 0, "Inside");
                        ui.selectable_value(&mut more.thin_side, 1, "Outside");
                        ui.selectable_value(&mut more.thin_side, 2, "Centred");
                    });
                }
                row(ui, "", |ui| {
                    ui.checkbox(flip, "Flip direction");
                });
                op_combo(ui, "ex_op", op);
            }
            Tool::Rib {
                sketch,
                thickness,
                depth,
                symmetric,
                flip,
                op,
            } => {
                feature_combo(ui, "rb_sk", "Profile", doc, sketch, is_sketch);
                text_field(ui, "Thickness", thickness);
                text_field(ui, "Height", depth);
                row(ui, "Direction", |ui| {
                    ui.selectable_value(symmetric, false, "One side");
                    ui.selectable_value(symmetric, true, "Symmetric");
                });
                row(ui, "", |ui| {
                    ui.checkbox(flip, "Flip direction");
                });
                op_combo(ui, "rb_op", op);
                ui.label(
                    egui::RichText::new(
                        "Every line and arc of the sketch becomes a wall: its thickness lies in the sketch plane, its height along the normal.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Pipe {
                path,
                diameter,
                wall,
                op,
            } => {
                feature_combo(ui, "pp_sk", "Path", doc, path, is_sketch);
                text_field(ui, "Outer diameter", diameter);
                text_field(ui, "Wall", wall);
                op_combo(ui, "pp_op", op);
                ui.label(
                    egui::RichText::new("The path is the first line, arc or spline of its sketch. A wall of half the diameter or more makes a solid rod.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Revolve {
                sketch,
                angle,
                axis_y,
                axis_edge,
                axis_datum,
                op,
                surface,
            } => {
                feature_combo(ui, "rv_sk", "Profile", doc, sketch, is_sketch);
                output_row(ui, surface, op);
                row(ui, "Axis", |ui| {
                    let mut a = if axis_datum.is_some() {
                        3
                    } else if *axis_edge {
                        2
                    } else if *axis_y {
                        1
                    } else {
                        0
                    };
                    ui.selectable_value(&mut a, 0, "Sketch X");
                    ui.selectable_value(&mut a, 1, "Sketch Y");
                    ui.selectable_value(&mut a, 2, "Edge");
                    ui.selectable_value(&mut a, 3, "Axis");
                    *axis_edge = a == 2;
                    *axis_y = a == 1;
                    if a == 3 {
                        if axis_datum.is_none() {
                            *axis_datum =
                                datum_ids(doc, |s| matches!(s, FeatureSpec::DatumAxis(_)))
                                    .first()
                                    .copied();
                        }
                    } else {
                        *axis_datum = None;
                    }
                });
                if axis_datum.is_some() {
                    datum_combo(ui, "rv_axis", "Construction axis", doc, axis_datum, |s| {
                        matches!(s, FeatureSpec::DatumAxis(_))
                    });
                }
                if *axis_edge {
                    let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                    row(ui, "Edge", |ui| {
                        ui.label(if n > 0 {
                            "edge selected"
                        } else {
                            "click a straight edge"
                        });
                    });
                }
                text_field(ui, "Angle", angle);
                if !*surface {
                    op_combo(ui, "rv_op", op);
                }
            }
            Tool::Hole {
                x,
                y,
                diameter,
                depth,
                kind,
                cb_diameter,
                cb_depth,
                cs_diameter,
                cs_angle,
                through_all,
                thread,
                thread_length,
            } => {
                row(ui, "Placement", |ui| {
                    ui.label(if picks.iter().any(|p| matches!(p, Pick::Face(_))) {
                        "face selected"
                    } else {
                        "click a planar face"
                    });
                });
                text_field(ui, "Position x", x);
                text_field(ui, "Position y", y);
                row(ui, "Type", |ui| {
                    ui.selectable_value(kind, 0, "Simple");
                    ui.selectable_value(kind, 1, "Counterbore");
                    ui.selectable_value(kind, 2, "Countersink");
                    ui.selectable_value(kind, 3, "Tapped");
                });
                if *kind == 3 {
                    thread_size_ui(ui, "hole_thread", thread);
                    if let Some(size) = kw_feature::threads::lookup(thread) {
                        ui.label(
                            egui::RichText::new(format!(
                                "Tap drill dia {:.2} mm, pitch {:.3} mm",
                                size.tap_drill, size.pitch
                            ))
                            .color(TEXT_WEAK)
                            .size(10.5),
                        );
                    }
                    text_field(ui, "Thread length", thread_length);
                } else {
                    text_field(ui, "Diameter", diameter);
                }
                row(ui, "Extent", |ui| {
                    ui.selectable_value(through_all, false, "Depth");
                    ui.selectable_value(through_all, true, "Through all");
                });
                if !*through_all {
                    text_field(ui, "Depth", depth);
                }
                if *kind == 1 {
                    text_field(ui, "CB diameter", cb_diameter);
                    text_field(ui, "CB depth", cb_depth);
                }
                if *kind == 2 {
                    text_field(ui, "CS diameter", cs_diameter);
                    text_field(ui, "CS angle", cs_angle);
                }
            }
            Tool::Fillet { radius } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                row(ui, "Edges", |ui| {
                    ui.label(format!("{n} selected"));
                });
                text_field(ui, "Radius", radius);
                ui.label(
                    egui::RichText::new("Click edges in the view (Shift adds).")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Chamfer { d1, d2 } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                row(ui, "Edges", |ui| {
                    ui.label(format!("{n} selected"));
                });
                text_field(ui, "Distance 1", d1);
                text_field(ui, "Distance 2", d2);
            }
            Tool::VariableFillet { start, end } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                row(ui, "Edge", |ui| {
                    ui.label(format!("{n} selected"));
                });
                text_field(ui, "Start radius", start);
                text_field(ui, "End radius", end);
                ui.label(
                    egui::RichText::new("Click one edge; the radius changes along it.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Shell { thickness } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                row(ui, "Open faces", |ui| {
                    ui.label(format!("{n} selected"));
                });
                text_field(ui, "Thickness", thickness);
                ui.label(
                    egui::RichText::new("Click the faces to remove (Shift adds).")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Pattern {
                source,
                dx,
                dy,
                dz,
                count,
            } => {
                feature_combo(ui, "pt_src", "Feature", doc, source, is_tool_feature);
                text_field(ui, "Spacing X", dx);
                text_field(ui, "Spacing Y", dy);
                text_field(ui, "Spacing Z", dz);
                text_field(ui, "Quantity", count);
            }
            Tool::CircularPattern {
                source,
                count,
                angle,
                axis,
                datum,
            } => {
                feature_combo(ui, "cp_src", "Feature", doc, source, is_tool_feature);
                row(ui, "Axis", |ui| {
                    ui.selectable_value(axis, 0, "X");
                    ui.selectable_value(axis, 1, "Y");
                    ui.selectable_value(axis, 2, "Z");
                    ui.selectable_value(axis, 3, "Axis");
                });
                if *axis == 3 {
                    if datum.is_none() {
                        *datum = datum_ids(doc, |s| matches!(s, FeatureSpec::DatumAxis(_)))
                            .first()
                            .copied();
                    }
                    datum_combo(ui, "cp_axis", "Construction axis", doc, datum, |s| {
                        matches!(s, FeatureSpec::DatumAxis(_))
                    });
                }
                text_field(ui, "Quantity", count);
                text_field(ui, "Angle", angle);
            }
            Tool::Mirror {
                source,
                plane,
                datum,
            } => {
                feature_combo(ui, "mr_src", "Feature", doc, source, is_tool_feature);
                row(ui, "Mirror plane", |ui| {
                    ui.selectable_value(plane, 0, "XY");
                    ui.selectable_value(plane, 1, "YZ");
                    ui.selectable_value(plane, 2, "XZ");
                    ui.selectable_value(plane, 3, "Plane");
                    ui.selectable_value(plane, 4, "Face");
                });
                if *plane == 3 {
                    if datum.is_none() {
                        *datum = datum_ids(doc, |s| matches!(s, FeatureSpec::DatumPlane(_)))
                            .first()
                            .copied();
                    }
                    datum_combo(ui, "mr_plane", "Construction plane", doc, datum, |s| {
                        matches!(s, FeatureSpec::DatumPlane(_))
                    });
                }
                if *plane == 4 {
                    let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                    row(ui, "Face", |ui| {
                        ui.label(if n > 0 {
                            "face selected"
                        } else {
                            "click a planar face"
                        });
                    });
                }
            }
            Tool::Combine {
                target,
                tool,
                op,
                keep_tool,
            } => {
                // bodies, not features: the targets are the bodies there are (plus the one being edited)
                let heads = doc.heads();
                let names: std::collections::HashMap<FeatureId, String> = heads
                    .iter()
                    .map(|h| {
                        let root = doc.body_root(*h);
                        let n = doc
                            .appearance_of(root)
                            .and_then(|a| a.name.clone())
                            .or_else(|| doc.get(root).map(|f| f.name.clone()))
                            .unwrap_or_default();
                        (*h, n)
                    })
                    .collect();
                for (label, id, sel) in [
                    ("Target body", "cb_t", &mut *target),
                    ("Tool body", "cb_k", &mut *tool),
                ] {
                    row(ui, label, |ui| {
                        egui::ComboBox::from_id_salt(id)
                            .selected_text(
                                sel.and_then(|s| {
                                    names
                                        .get(&s)
                                        .cloned()
                                        .or_else(|| doc.get(s).map(|f| f.name.clone()))
                                })
                                .unwrap_or_else(|| "Select…".into()),
                            )
                            .width(150.0)
                            .show_ui(ui, |ui| {
                                for h in &heads {
                                    ui.selectable_value(sel, Some(*h), names[h].clone());
                                }
                            });
                    });
                }
                op_combo(ui, "cb_op", op);
                row(ui, "", |ui| {
                    ui.checkbox(keep_tool, "Keep tool body");
                });
            }
            Tool::Loft {
                sketches,
                op,
                surface,
                ruled,
            } => {
                output_row(ui, surface, op);
                if *surface {
                    row(ui, "Shape", |ui| {
                        ui.selectable_value(ruled, false, "Smooth");
                        ui.selectable_value(ruled, true, "Ruled");
                    });
                }
                let mut remove = None;
                for (i, s) in sketches.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        feature_combo(
                            ui,
                            &format!("lf_{i}"),
                            &format!("Profile {}", i + 1),
                            doc,
                            s,
                            is_sketch,
                        );
                        if i >= 2
                            && ui
                                .small_button("x")
                                .on_hover_text("Remove this profile")
                                .clicked()
                        {
                            remove = Some(i);
                        }
                    });
                }
                if let Some(i) = remove {
                    sketches.remove(i);
                }
                row(ui, "", |ui| {
                    if ui.small_button("+ Add profile").clicked() {
                        sketches.push(None);
                    }
                });
                if !*surface {
                    op_combo(ui, "lf_op", op);
                }
                ui.label(
                    egui::RichText::new("Profiles are blended in the order listed.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Sweep {
                profile,
                path,
                op,
                surface,
                twist,
                guide,
            } => {
                feature_combo(ui, "sw_p", "Profile", doc, profile, is_sketch);
                feature_combo(ui, "sw_w", "Path", doc, path, is_sketch);
                output_row(ui, surface, op);
                text_field(ui, "Twist", twist);
                row(ui, "Guide rail", |ui| {
                    let mut on = guide.is_some();
                    if ui.checkbox(&mut on, "").changed() {
                        *guide = if on {
                            doc.features
                                .iter()
                                .rev()
                                .find(|f| is_sketch(&f.spec))
                                .map(|f| f.id)
                        } else {
                            None
                        };
                    }
                });
                if guide.is_some() {
                    feature_combo(ui, "sw_g", "Guide sketch", doc, guide, is_sketch);
                }
                if !*surface {
                    op_combo(ui, "sw_op", op);
                }
                ui.label(
                    egui::RichText::new("The path is the first line, arc or spline of its sketch.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Patch { tangent } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                row(ui, "Edges", |ui| {
                    ui.label(format!("{n} selected"));
                });
                row(ui, "", |ui| {
                    ui.checkbox(tangent, "Tangent to the surfaces they belong to");
                });
                ui.label(
                    egui::RichText::new(
                        "Click the edges around the hole (Shift adds): three or more, closing a loop. Four edges give one smooth patch; other counts are split about the centre. Only open edges of a surface can be tangent.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Stitch { tolerance } => {
                let n = picks.len();
                row(ui, "Surfaces", |ui| {
                    ui.label(if n == 0 {
                        "all surfaces".to_string()
                    } else {
                        format!("{n} picked")
                    });
                });
                text_field(ui, "Tolerance", tolerance);
                ui.label(
                    egui::RichText::new(
                        "Open edges that lie on one another within the tolerance are joined; a group that closes becomes a solid. Click faces to stitch only some surfaces.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Unstitch => {
                ui.label(
                    egui::RichText::new(
                        "The body falls apart into its faces, which share no edges. Click a face to choose the body.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Thicken { thickness, side } => {
                text_field(ui, "Thickness", thickness);
                row(ui, "Direction", |ui| {
                    ui.selectable_value(side, 0, "Normal");
                    ui.selectable_value(side, 1, "Reverse");
                    ui.selectable_value(side, 2, "Both");
                });
                ui.label(
                    egui::RichText::new(
                        "An open, smooth surface becomes a solid with walls along its open edges.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::OffsetSurface { distance } => {
                text_field(ui, "Distance", distance);
                ui.label(
                    egui::RichText::new(
                        "Moves the surface along its normal; a negative distance goes the other way. The surface must be smooth.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Ruled => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                row(ui, "Edges", |ui| {
                    ui.label(format!("{n} of 2 selected"));
                });
                ui.label(
                    egui::RichText::new(
                        "Click two edges: straight lines join their corresponding points.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Draft {
                angle,
                axis,
                position,
            } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                row(ui, "Faces", |ui| {
                    ui.label(format!("{n} selected"));
                });
                row(ui, "Pull direction", |ui| {
                    ui.selectable_value(axis, 0, "X");
                    ui.selectable_value(axis, 1, "Y");
                    ui.selectable_value(axis, 2, "Z");
                });
                text_field(ui, "Neutral plane", position);
                text_field(ui, "Angle", angle);
                ui.label(
                    egui::RichText::new(
                        "Click the faces to tilt (Shift adds). The neutral plane stays put.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::SheetMetal {
                thickness,
                k_factor,
                radius,
                width,
                rows,
            } => {
                text_field(ui, "Thickness", thickness);
                text_field(ui, "K-factor", k_factor);
                text_field(ui, "Bend radius", radius);
                text_field(ui, "Width", width);
                ui.separator();
                let mut remove = None;
                for (i, r) in rows.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(if r.bend { "Bend" } else { "Flat" })
                                .color(TEXT_WEAK)
                                .size(10.5),
                        );
                        ui.add(egui::TextEdit::singleline(&mut r.a).desired_width(70.0));
                        if r.bend {
                            ui.label(egui::RichText::new("radius").color(TEXT_WEAK).size(10.5));
                            ui.add(egui::TextEdit::singleline(&mut r.r).desired_width(50.0));
                        }
                        if ui.small_button("x").clicked() {
                            remove = Some(i);
                        }
                    });
                }
                if let Some(i) = remove {
                    rows.remove(i);
                }
                ui.horizontal(|ui| {
                    if ui.small_button("+ Flat").clicked() {
                        rows.push(SheetRow::flat("20 mm"));
                    }
                    if ui.small_button("+ Bend").clicked() {
                        rows.push(SheetRow::bend("90 deg"));
                    }
                    if ui.small_button("+ Flange").clicked() {
                        rows.push(SheetRow::bend("90 deg"));
                        rows.push(SheetRow::flat("15 mm"));
                    }
                });
                ui.label(
                    egui::RichText::new(
                        "The strip runs along x: flats are straight, bends turn it about its width (positive up).",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::Form {
                kind,
                size,
                sides,
                cage,
                scale,
                tolerance,
                vertex,
                built,
            } => {
                row(ui, "Shape", |ui| {
                    egui::ComboBox::from_id_salt("form_kind")
                        .selected_text(
                            super::form::KINDS
                                .get(*kind as usize)
                                .copied()
                                .unwrap_or("Custom cage"),
                        )
                        .show_ui(ui, |ui| {
                            for (i, n) in super::form::KINDS.iter().enumerate() {
                                if ui.selectable_value(kind, i as u8, *n).clicked() {
                                    let (s, n) = super::form::defaults(i as u8);
                                    *size = s;
                                    *sides = n;
                                }
                            }
                        });
                });
                if (*kind as usize) < super::form::KINDS.len() {
                    for (i, l) in ["Width", "Depth", "Height"].iter().enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new(*l).color(TEXT_WEAK));
                            ui.add(
                                egui::DragValue::new(&mut size[i])
                                    .speed(0.2)
                                    .range(0.1..=1e5)
                                    .suffix(" mm"),
                            );
                        });
                    }
                    if matches!(*kind, 1 | 3) {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("Sides").color(TEXT_WEAK));
                            ui.add(egui::DragValue::new(sides).range(3..=48));
                        });
                    }
                    if (*kind, *size, *sides) != *built {
                        *cage = super::form::primitive(*kind, *size, *sides);
                        *built = (*kind, *size, *sides);
                        *vertex = (*vertex).min(cage.verts.len().saturating_sub(1));
                    }
                }
                text_field(ui, "Scale", scale);
                text_field(ui, "Tolerance", tolerance);
                ui.separator();
                ui.label(
                    egui::RichText::new(format!(
                        "Cage: {} vertices, {} faces, {} creases",
                        cage.verts.len(),
                        cage.faces.len(),
                        cage.creases.len()
                    ))
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
                if !cage.verts.is_empty() {
                    *vertex = (*vertex).min(cage.verts.len() - 1);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Vertex").color(TEXT_WEAK));
                        ui.add(egui::DragValue::new(vertex).range(0..=cage.verts.len() - 1));
                    });
                    ui.horizontal(|ui| {
                        for (k, p) in cage.verts[*vertex].iter_mut().enumerate() {
                            ui.add(
                                egui::DragValue::new(p)
                                    .speed(0.1)
                                    .prefix(["x ", "y ", "z "][k]),
                            );
                        }
                    });
                }
                ui.horizontal(|ui| {
                    if ui.small_button("Sharp edges").clicked() {
                        *cage = cage.clone().crease_all(100.0);
                    }
                    if ui.small_button("Smooth edges").clicked() {
                        cage.creases.clear();
                    }
                });
                ui.label(
                    egui::RichText::new("Move the highlighted cage vertex with the numbers; the result is the smooth surface the cage describes (the body updates as you edit).")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Thread {
                designation,
                length,
                internal,
            } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                row(ui, "Face", |ui| {
                    ui.label(if n > 0 {
                        "face selected"
                    } else {
                        "click a hole wall or a pin"
                    });
                });
                row(ui, "Kind", |ui| {
                    ui.selectable_value(internal, true, "Internal (tapped)");
                    ui.selectable_value(internal, false, "External (bolt)");
                });
                thread_size_ui(ui, "thread_size", designation);
                if let Some(size) = kw_feature::threads::lookup(designation) {
                    ui.label(
                        egui::RichText::new(kw_feature::threads::describe(size, 0.0, *internal))
                            .color(TEXT_WEAK)
                            .size(10.5),
                    );
                }
                text_field(ui, "Length", length);
                ui.label(
                    egui::RichText::new(
                        "Cosmetic: the model is not changed; the thread is listed in Measure and the feature list.",
                    )
                    .color(TEXT_WEAK)
                    .size(10.5),
                );
            }
            Tool::MoveBody {
                copy,
                dx,
                dy,
                dz,
                axis,
                angle,
            } => {
                let _ = copy;
                text_field(ui, "Move X", dx);
                text_field(ui, "Move Y", dy);
                text_field(ui, "Move Z", dz);
                row(ui, "Rotate about", |ui| {
                    ui.selectable_value(axis, 0, "X");
                    ui.selectable_value(axis, 1, "Y");
                    ui.selectable_value(axis, 2, "Z");
                });
                text_field(ui, "Angle", angle);
            }
            Tool::ScaleBody { factor, cx, cy, cz } => {
                text_field(ui, "Scale factor", factor);
                text_field(ui, "About X", cx);
                text_field(ui, "About Y", cy);
                text_field(ui, "About Z", cz);
            }
            Tool::SplitBody { plane, offset } => {
                let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                row(ui, "Split with", |ui| {
                    ui.selectable_value(plane, 0, "XY");
                    ui.selectable_value(plane, 1, "YZ");
                    ui.selectable_value(plane, 2, "XZ");
                    ui.selectable_value(plane, 3, "Face");
                });
                if *plane == 3 {
                    row(ui, "Face", |ui| {
                        ui.label(format!("{n} selected"));
                    });
                } else {
                    text_field(ui, "Offset", offset);
                }
                ui.label(
                    egui::RichText::new("The body becomes two bodies, one on each side.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Offset { distance } => {
                text_field(ui, "Distance", distance);
                ui.label(
                    egui::RichText::new("Positive grows the body, negative shrinks it.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::ConstructionPlane {
                base,
                offset,
                tilt,
                more,
            } => {
                row(ui, "Type", |ui| {
                    ui.selectable_value(&mut more.mode, 0, "Offset");
                    ui.selectable_value(&mut more.mode, 1, "Midplane");
                    ui.selectable_value(&mut more.mode, 2, "3 points");
                });
                if more.mode < 2 {
                    row(ui, "Based on", |ui| {
                        ui.selectable_value(base, 0, "XY");
                        ui.selectable_value(base, 1, "YZ");
                        ui.selectable_value(base, 2, "XZ");
                    });
                    text_field(ui, "Offset", offset);
                }
                match more.mode {
                    0 => text_field(ui, "Angle", tilt),
                    1 => plane_input_ui(ui, "mid_other", "Other plane", doc, &mut more.other),
                    _ => {
                        for (i, p) in more.pts.iter_mut().enumerate() {
                            point_input_ui(
                                ui,
                                &format!("pl_pt{i}"),
                                &format!("Point {}", i + 1),
                                doc,
                                p,
                            );
                        }
                    }
                }
            }
            Tool::Axis {
                mode,
                pts,
                at_u,
                at_v,
                planes,
            } => {
                row(ui, "Type", |ui| {
                    ui.selectable_value(mode, 0, "2 points");
                    ui.selectable_value(mode, 1, "Edge");
                    ui.selectable_value(mode, 2, "Face normal");
                    ui.selectable_value(mode, 3, "2 planes");
                });
                match *mode {
                    0 => {
                        for (i, p) in pts.iter_mut().enumerate() {
                            point_input_ui(
                                ui,
                                &format!("ax_pt{i}"),
                                &format!("Point {}", i + 1),
                                doc,
                                p,
                            );
                        }
                    }
                    1 => {
                        let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                        row(ui, "Edge", |ui| {
                            ui.label(if n > 0 {
                                "edge selected"
                            } else {
                                "click a straight edge"
                            });
                        });
                    }
                    2 => {
                        let n = picks.iter().filter(|p| matches!(p, Pick::Face(_))).count();
                        row(ui, "Face", |ui| {
                            ui.label(if n > 0 {
                                "face selected"
                            } else {
                                "click a planar face"
                            });
                        });
                        text_field(ui, "Through u", at_u);
                        text_field(ui, "Through v", at_v);
                    }
                    _ => {
                        for (i, p) in planes.iter_mut().enumerate() {
                            plane_input_ui(
                                ui,
                                &format!("ax_pl{i}"),
                                &format!("Plane {}", i + 1),
                                doc,
                                p,
                            );
                        }
                    }
                }
            }
            Tool::DatumPoint {
                mode,
                coords,
                axis,
                plane,
            } => {
                row(ui, "Type", |ui| {
                    ui.selectable_value(mode, 0, "Coordinates");
                    ui.selectable_value(mode, 1, "Edge middle");
                    ui.selectable_value(mode, 2, "Axis and plane");
                });
                match *mode {
                    0 => {
                        text_field(ui, "X", &mut coords.x);
                        text_field(ui, "Y", &mut coords.y);
                        text_field(ui, "Z", &mut coords.z);
                    }
                    1 => {
                        let n = picks.iter().filter(|p| matches!(p, Pick::Edge(_))).count();
                        row(ui, "Edge", |ui| {
                            ui.label(if n > 0 {
                                "edge selected"
                            } else {
                                "click an edge"
                            });
                        });
                    }
                    _ => {
                        datum_combo(ui, "pt_axis", "Axis", doc, axis, |s| {
                            matches!(s, FeatureSpec::DatumAxis(_))
                        });
                        plane_input_ui(ui, "pt_plane", "Plane", doc, plane);
                    }
                }
            }
        }
        *self != before
    }
}

fn edge_refs(ctx: &BuildCtx) -> Result<Vec<EdgeRef>, String> {
    let (brep, names) = ctx.base.ok_or("there is no solid to work on")?;
    let mut edges = Vec::new();
    for p in ctx.picks {
        if let Pick::Edge(e) = p {
            let faces = brep.faces_around_edge(*e);
            let refs: Vec<_> = faces
                .iter()
                .filter_map(|f| names.reference_to(brep, *f))
                .collect();
            if refs.len() != 2 {
                return Err("an edge must join exactly two named faces".into());
            }
            edges.push(EdgeRef {
                faces: [refs[0].clone(), refs[1].clone()],
            });
        }
    }
    if edges.is_empty() {
        return Err("click one or more edges in the view first".into());
    }
    Ok(edges)
}

/// The floating panel around a tool's fields (title bar, OK / Cancel, an error line).
pub fn panel(
    ctx: &egui::Context,
    dlg: &mut Dialog,
    doc: &Document,
    picks: &[Pick],
    regions: &[RegionInfo],
    error: Option<&str>,
) -> (Outcome, bool) {
    let mut outcome = Outcome::Open;
    let mut changed = false;
    let title = if dlg.editing.is_some() {
        format!("EDIT {}", dlg.tool.title())
    } else {
        dlg.tool.title().to_string()
    };
    egui::Window::new(egui::RichText::new(title).size(11.5).strong())
        .id(egui::Id::new("tool_dialog"))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::RIGHT_TOP, [-14.0, 150.0])
        .min_width(260.0)
        .show(ctx, |ui| {
            changed = dlg.tool.fields(ui, doc, picks, regions);
            if let Some(e) = error {
                ui.add_space(4.0);
                ui.label(egui::RichText::new(e).color(BAD).size(11.0));
            }
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("  OK  ").color(egui::Color32::WHITE),
                            )
                            .fill(ACCENT),
                        )
                        .clicked()
                    {
                        outcome = Outcome::Ok;
                    }
                    if ui.button("Cancel").clicked() {
                        outcome = Outcome::Cancel;
                    }
                });
            });
        });
    (outcome, changed)
}
