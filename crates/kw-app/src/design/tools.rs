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
    LinearPatternSpec, LoftSpec, MirrorPlane, MirrorSpec, NameMap, OffsetSpec, PlaneSpec,
    RevolveSpec, ShellSpec, SweepSpec,
};
use std::sync::Arc;

/// What a tool may look at while building its spec.
pub struct BuildCtx<'a> {
    pub doc: &'a Document,
    /// The solid the new feature will modify, with its face names.
    pub base: Option<(&'a Arc<BRep>, &'a NameMap)>,
    pub picks: &'a [Pick],
    /// Plane picked for a hole, and the spot on it (plane coordinates).
    pub hole_plane: Option<PlaneSpec>,
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
    },
    Revolve {
        sketch: Option<FeatureId>,
        angle: String,
        axis_y: bool,
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
        axis: u8,
    },
    Mirror {
        source: Option<FeatureId>,
        plane: u8,
    },
    Combine {
        target: Option<FeatureId>,
        tool: Option<FeatureId>,
        op: BodyOp,
    },
    /// Blend between two or more profile sketches, in order.
    Loft {
        sketches: Vec<Option<FeatureId>>,
        op: BodyOp,
    },
    /// A profile sketch swept along the first line, arc or spline of a path sketch.
    Sweep {
        profile: Option<FeatureId>,
        path: Option<FeatureId>,
        op: BodyOp,
    },
    /// Tilt the picked faces about the neutral plane `axis = position`; `axis` is the pull direction.
    Draft {
        angle: String,
        axis: u8,
        position: String,
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
    },
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
        }
    }

    pub fn title(&self) -> &'static str {
        match self {
            Tool::Extrude { .. } => "EXTRUDE",
            Tool::Revolve { .. } => "REVOLVE",
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
            Tool::Draft { .. } => "DRAFT",
            Tool::Offset { .. } => "OFFSET BODY",
            Tool::ConstructionPlane { .. } => "CONSTRUCTION PLANE",
        }
    }

    /// Base of the feature's name ("Extrude" → "Extrude 3").
    pub fn name_base(&self) -> &'static str {
        match self {
            Tool::Extrude { .. } => "Extrude",
            Tool::Revolve { .. } => "Revolve",
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
            Tool::Draft { .. } => "Draft",
            Tool::Offset { .. } => "Offset",
            Tool::ConstructionPlane { .. } => "Plane",
        }
    }

    /// Does this tool take faces or edges from the viewport?
    pub fn picks_edges(&self) -> bool {
        matches!(
            self,
            Tool::Fillet { .. } | Tool::Chamfer { .. } | Tool::VariableFillet { .. }
        )
    }
    pub fn picks_faces(&self) -> bool {
        matches!(
            self,
            Tool::Shell { .. } | Tool::Hole { .. } | Tool::Draft { .. }
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
            },
            FeatureSpec::Revolve(x) => Tool::Revolve {
                sketch: Some(x.sketch),
                angle: param_text(&x.angle),
                axis_y: matches!(x.axis, AxisSpec::SketchY),
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
                let axis = if x.axis_dir[0].abs() > 0.5 {
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
                }
            }
            FeatureSpec::Mirror(x) => Tool::Mirror {
                source: Some(x.source),
                plane: match x.plane {
                    MirrorPlane::Xy(_) => 0,
                    MirrorPlane::Yz(_) => 1,
                    MirrorPlane::Xz(_) => 2,
                },
            },
            FeatureSpec::Combine(x) => Tool::Combine {
                target: Some(x.target),
                tool: Some(x.tool),
                op: x.op,
            },
            FeatureSpec::Loft(x) => Tool::Loft {
                sketches: x.sketches.iter().map(|s| Some(*s)).collect(),
                op: x.op,
            },
            FeatureSpec::Sweep(x) => Tool::Sweep {
                profile: Some(x.profile),
                path: Some(x.path),
                op: x.op,
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
                }
            }
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
                FeatureSpec::Extrude(ExtrudeSpec {
                    sketch: need(sketch, "a sketch")?,
                    depth: if *through_all {
                        // not used: the tool is as long as the body needs
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
                    },
                })
            }
            Tool::Revolve {
                sketch,
                angle,
                axis_y,
                op,
            } => FeatureSpec::Revolve(RevolveSpec {
                sketch: need(sketch, "a sketch")?,
                axis: if *axis_y {
                    AxisSpec::SketchY
                } else {
                    AxisSpec::SketchX
                },
                angle: ang(angle)?,
                op: *op,
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
            } => {
                let plane = match (orig, &ctx.hole_plane) {
                    (_, Some(p)) => p.clone(),
                    (Some(FeatureSpec::Hole(o)), None) => o.plane.clone(),
                    _ => return Err("click a planar face to place the hole on".into()),
                };
                let d = len(diameter)?;
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
                })
            }
            Tool::Mirror { source, plane } => {
                let z = Param::length(0.0);
                FeatureSpec::Mirror(MirrorSpec {
                    source: need(source, "a feature to mirror")?,
                    plane: match plane {
                        0 => MirrorPlane::Xy(z),
                        1 => MirrorPlane::Yz(z),
                        _ => MirrorPlane::Xz(z),
                    },
                    base: None,
                })
            }
            Tool::Combine { target, tool, op } => FeatureSpec::Combine(CombineSpec {
                target: need(target, "a target")?,
                tool: need(tool, "a tool")?,
                op: *op,
            }),
            Tool::Loft { sketches, op } => {
                let picked: Vec<FeatureId> = sketches.iter().flatten().copied().collect();
                if picked.len() < 2 {
                    return Err("a loft needs at least two profile sketches".into());
                }
                FeatureSpec::Loft(LoftSpec {
                    sketches: picked,
                    op: *op,
                    base: None,
                })
            }
            Tool::Sweep { profile, path, op } => FeatureSpec::Sweep(SweepSpec {
                profile: need(profile, "a profile sketch")?,
                path: need(path, "a path sketch")?,
                op: *op,
                base: None,
            }),
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
            Tool::Offset { distance } => FeatureSpec::Offset(OffsetSpec {
                distance: len(distance)?,
                base: None,
            }),
            Tool::ConstructionPlane { base, offset, tilt } => {
                let zero = Param::length(0.0);
                FeatureSpec::DatumPlane(DatumPlaneSpec {
                    base: match base {
                        1 => PlaneSpec::Yz(zero),
                        2 => PlaneSpec::Xz(zero),
                        _ => PlaneSpec::Xy(zero),
                    },
                    offset: len(offset)?,
                    tilt: ang(tilt)?,
                })
            }
        })
    }

    /// Draw the fields; returns whether anything changed.
    pub fn fields(&mut self, ui: &mut egui::Ui, doc: &Document, picks: &[Pick]) -> bool {
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
            } => {
                feature_combo(ui, "ex_sk", "Profile", doc, sketch, is_sketch);
                row(ui, "Extent", |ui| {
                    ui.selectable_value(through_all, false, "Distance");
                    ui.selectable_value(through_all, true, "Through all");
                });
                row(ui, "Direction", |ui| {
                    ui.selectable_value(symmetric, false, "One side");
                    ui.selectable_value(symmetric, true, "Symmetric");
                });
                if !*through_all {
                    text_field(ui, "Distance", depth);
                }
                if !*symmetric {
                    text_field(ui, "Second side", second);
                }
                text_field(ui, "Taper angle", taper);
                row(ui, "", |ui| {
                    ui.checkbox(flip, "Flip direction");
                });
                op_combo(ui, "ex_op", op);
            }
            Tool::Revolve {
                sketch,
                angle,
                axis_y,
                op,
            } => {
                feature_combo(ui, "rv_sk", "Profile", doc, sketch, is_sketch);
                row(ui, "Axis", |ui| {
                    ui.selectable_value(axis_y, false, "Sketch X");
                    ui.selectable_value(axis_y, true, "Sketch Y");
                });
                text_field(ui, "Angle", angle);
                op_combo(ui, "rv_op", op);
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
                });
                text_field(ui, "Diameter", diameter);
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
            } => {
                feature_combo(ui, "cp_src", "Feature", doc, source, is_tool_feature);
                row(ui, "Axis", |ui| {
                    ui.selectable_value(axis, 0, "X");
                    ui.selectable_value(axis, 1, "Y");
                    ui.selectable_value(axis, 2, "Z");
                });
                text_field(ui, "Quantity", count);
                text_field(ui, "Angle", angle);
            }
            Tool::Mirror { source, plane } => {
                feature_combo(ui, "mr_src", "Feature", doc, source, is_tool_feature);
                row(ui, "Mirror plane", |ui| {
                    ui.selectable_value(plane, 0, "XY");
                    ui.selectable_value(plane, 1, "YZ");
                    ui.selectable_value(plane, 2, "XZ");
                });
            }
            Tool::Combine { target, tool, op } => {
                feature_combo(ui, "cb_t", "Target", doc, target, |s| {
                    is_tool_feature(s) || matches!(s, FeatureSpec::Combine(_))
                });
                feature_combo(ui, "cb_k", "Tool", doc, tool, |s| {
                    is_tool_feature(s) || matches!(s, FeatureSpec::Combine(_))
                });
                op_combo(ui, "cb_op", op);
            }
            Tool::Loft { sketches, op } => {
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
                op_combo(ui, "lf_op", op);
                ui.label(
                    egui::RichText::new("Profiles are blended in the order listed.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::Sweep { profile, path, op } => {
                feature_combo(ui, "sw_p", "Profile", doc, profile, is_sketch);
                feature_combo(ui, "sw_w", "Path", doc, path, is_sketch);
                op_combo(ui, "sw_op", op);
                ui.label(
                    egui::RichText::new("The path is the first line, arc or spline of its sketch.")
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
            Tool::Offset { distance } => {
                text_field(ui, "Distance", distance);
                ui.label(
                    egui::RichText::new("Positive grows the body, negative shrinks it.")
                        .color(TEXT_WEAK)
                        .size(10.5),
                );
            }
            Tool::ConstructionPlane { base, offset, tilt } => {
                row(ui, "Based on", |ui| {
                    ui.selectable_value(base, 0, "XY");
                    ui.selectable_value(base, 1, "YZ");
                    ui.selectable_value(base, 2, "XZ");
                });
                text_field(ui, "Offset", offset);
                text_field(ui, "Angle", tilt);
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
            changed = dlg.tool.fields(ui, doc, picks);
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
