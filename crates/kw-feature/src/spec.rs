//! What a feature *is*: a serialisable description whose numbers are literals or expressions and whose
//! references to other geometry are persistent [`Reference`]s or feature ids. Nothing here holds a
//! graph index or a slotmap key, so a [`FeatureSpec`] can be written to a `.kwf` file and read back.

use crate::id::FeatureId;
use crate::naming::Reference;
use crate::recipe::SketchRecipe;
use kw_expr::{EvalError, Param, Scope};
use serde::{Deserialize, Serialize};

/// What a solid-producing feature does with the body so far.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BodyOp {
    /// Start a new, separate body.
    New,
    Join,
    Cut,
    Intersect,
}

/// Where a sketch or hole lies.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PlaneSpec {
    /// Plane z = offset, x → X, y → Y.
    Xy(Param),
    /// Plane x = offset, x → Y, y → Z.
    Yz(Param),
    /// Plane y = offset, x → Z, y → X.
    Xz(Param),
    /// A datum plane feature.
    Datum(FeatureId),
    /// A planar face of the body so far.
    Face(Reference),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchSpec {
    pub plane: PlaneSpec,
    pub recipe: SketchRecipe,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatumPlaneSpec {
    pub base: PlaneSpec,
    /// Offset along the base plane's normal.
    pub offset: Param,
    /// Rotation about the base plane's x axis.
    pub tilt: Param,
}

/// How far an extrude reaches, beyond `depth` along one side of the sketch plane.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ExtrudeExtent {
    /// Through all of the body: the distance is ignored and the tool is long enough to clear it.
    #[serde(default)]
    pub through_all: bool,
    /// Distance on the other side of the sketch plane (two-sided extrude with different distances).
    #[serde(default)]
    pub second_side: Option<Param>,
    /// Taper angle: positive narrows the profile along the extrusion direction.
    #[serde(default)]
    pub taper: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExtrudeSpec {
    /// Extent options beyond the plain distance; the default is an ordinary one-sided extrude.
    #[serde(default)]
    pub extent: ExtrudeExtent,
    pub sketch: FeatureId,
    pub depth: Param,
    pub symmetric: bool,
    /// Extrude against the sketch plane's normal.
    pub flip: bool,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AxisSpec {
    SketchX,
    SketchY,
    /// A line entity of the profile sketch (by label).
    SketchLine(u32),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RevolveSpec {
    pub sketch: FeatureId,
    pub axis: AxisSpec,
    pub angle: Param,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoftSpec {
    pub sketches: Vec<FeatureId>,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SweepSpec {
    pub profile: FeatureId,
    /// A sketch whose first non-construction line, arc or spline is the path.
    pub path: FeatureId,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
}

/// The edges shared by two faces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EdgeRef {
    pub faces: [Reference; 2],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FilletSpec {
    pub edges: Vec<EdgeRef>,
    pub radius: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChamferSpec {
    pub edges: Vec<EdgeRef>,
    pub d1: Param,
    pub d2: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VariableFilletSpec {
    pub edge: EdgeRef,
    pub start: Param,
    pub end: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShellSpec {
    pub thickness: Param,
    pub open: Vec<Reference>,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DraftSpec {
    pub faces: Vec<Reference>,
    pub neutral_origin: [Param; 3],
    pub pull: [f64; 3],
    pub angle: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffsetSpec {
    pub distance: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CombineSpec {
    pub target: FeatureId,
    pub tool: FeatureId,
    pub op: BodyOp,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HoleKind {
    Simple {
        diameter: Param,
    },
    Counterbore {
        diameter: Param,
        cb_diameter: Param,
        cb_depth: Param,
    },
    Countersink {
        diameter: Param,
        cs_diameter: Param,
        /// Full included angle of the countersink cone.
        cs_angle: Param,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoleSpec {
    pub plane: PlaneSpec,
    /// Hole centres in the plane's (x, y).
    pub points: Vec<[Param; 2]>,
    /// Stored as `style`: `kind` is the tag of [`FeatureSpec`], and a second `kind` key made every saved
    /// document with a hole unreadable.
    #[serde(rename = "style")]
    pub kind: HoleKind,
    pub depth: Param,
    /// Drill through the whole body: `depth` is ignored (a counterbore still uses its own depth).
    #[serde(default)]
    pub through_all: bool,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinearPatternSpec {
    pub source: FeatureId,
    /// Offset between consecutive instances.
    pub spacing: [Param; 3],
    /// Total number of instances including the source (an integer ≥ 1).
    pub count: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CircularPatternSpec {
    pub source: FeatureId,
    pub axis_origin: [Param; 3],
    pub axis_dir: [f64; 3],
    pub count: Param,
    /// Angle spanned by the instances (a full turn spaces them evenly around).
    pub total_angle: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MirrorPlane {
    Xy(Param),
    Yz(Param),
    Xz(Param),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MirrorSpec {
    pub source: FeatureId,
    pub plane: MirrorPlane,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SheetSegSpec {
    Flat { length: Param },
    Bend { angle: Param, radius: Option<Param> },
}

/// A sheet-metal strip: a thickness, a K-factor, a width and the flats and bends along the path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SheetMetalSpec {
    pub thickness: Param,
    pub k_factor: Param,
    pub default_radius: Param,
    pub width: Param,
    pub segs: Vec<SheetSegSpec>,
}

/// A subdivision-surface body: a control cage turned into a B-rep of bicubic patches by *Finish*.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SubdSpec {
    pub cage: kw_subd::SubdMesh,
    /// Uniform scale applied to the cage.
    pub scale: Param,
    /// Largest distance from the patches to the limit surface.
    pub tolerance: Param,
}

/// File format of an imported body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImportFormat {
    Step,
    Iges,
}

/// A body read from another CAD system's file; the file's text is kept in the document so the part stays
/// self-contained. The imported solids (united when there are several) become the body of this feature.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImportSpec {
    /// Name of the file it came from (for display).
    pub source: String,
    pub format: ImportFormat,
    pub data: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum FeatureSpec {
    Parameter { name: String, value: Param },
    Sketch(SketchSpec),
    DatumPlane(DatumPlaneSpec),
    Extrude(ExtrudeSpec),
    Revolve(RevolveSpec),
    Loft(LoftSpec),
    Sweep(SweepSpec),
    Fillet(FilletSpec),
    Chamfer(ChamferSpec),
    VariableFillet(VariableFilletSpec),
    Shell(ShellSpec),
    Draft(DraftSpec),
    Offset(OffsetSpec),
    Combine(CombineSpec),
    Hole(HoleSpec),
    LinearPattern(LinearPatternSpec),
    CircularPattern(CircularPatternSpec),
    Mirror(MirrorSpec),
    SheetMetal(SheetMetalSpec),
    Subd(SubdSpec),
    Import(ImportSpec),
}

fn plane_params(p: &mut PlaneSpec, f: &mut dyn FnMut(&mut Param)) {
    match p {
        PlaneSpec::Xy(o) | PlaneSpec::Yz(o) | PlaneSpec::Xz(o) => f(o),
        _ => {}
    }
}

fn plane_refs(p: &PlaneSpec, out: &mut Vec<FeatureId>) {
    match p {
        PlaneSpec::Datum(d) => out.push(*d),
        PlaneSpec::Face(r) => out.push(r.name.feature),
        _ => {}
    }
}

impl FeatureSpec {
    /// Short human-readable kind.
    pub fn kind_name(&self) -> &'static str {
        match self {
            FeatureSpec::Parameter { .. } => "Parameter",
            FeatureSpec::Sketch(_) => "Sketch",
            FeatureSpec::DatumPlane(_) => "DatumPlane",
            FeatureSpec::Extrude(_) => "Extrude",
            FeatureSpec::Revolve(_) => "Revolve",
            FeatureSpec::Loft(_) => "Loft",
            FeatureSpec::Sweep(_) => "Sweep",
            FeatureSpec::Fillet(_) => "Fillet",
            FeatureSpec::Chamfer(_) => "Chamfer",
            FeatureSpec::VariableFillet(_) => "VariableFillet",
            FeatureSpec::Shell(_) => "Shell",
            FeatureSpec::Draft(_) => "Draft",
            FeatureSpec::Offset(_) => "Offset",
            FeatureSpec::Combine(_) => "Combine",
            FeatureSpec::Hole(_) => "Hole",
            FeatureSpec::LinearPattern(_) => "LinearPattern",
            FeatureSpec::CircularPattern(_) => "CircularPattern",
            FeatureSpec::Mirror(_) => "Mirror",
            FeatureSpec::SheetMetal(_) => "SheetMetal",
            FeatureSpec::Subd(_) => "Subd",
            FeatureSpec::Import(_) => "Import",
        }
    }

    /// Does the feature produce a solid body?
    pub fn produces_solid(&self) -> bool {
        !matches!(
            self,
            FeatureSpec::Parameter { .. } | FeatureSpec::Sketch(_) | FeatureSpec::DatumPlane(_)
        )
    }

    /// Visit every parameter, in a fixed order.
    pub fn for_each_param_mut(&mut self, f: &mut dyn FnMut(&mut Param)) {
        match self {
            FeatureSpec::Parameter { value, .. } => f(value),
            FeatureSpec::Sketch(s) => {
                plane_params(&mut s.plane, &mut |p| f(p));
                for b in &mut s.recipe.bindings {
                    f(&mut b.value);
                }
            }
            FeatureSpec::DatumPlane(d) => {
                plane_params(&mut d.base, &mut |p| f(p));
                f(&mut d.offset);
                f(&mut d.tilt);
            }
            FeatureSpec::Extrude(e) => {
                f(&mut e.depth);
                if let Some(p) = &mut e.extent.second_side {
                    f(p);
                }
                if let Some(p) = &mut e.extent.taper {
                    f(p);
                }
            }
            FeatureSpec::Revolve(r) => f(&mut r.angle),
            FeatureSpec::Loft(_) | FeatureSpec::Sweep(_) | FeatureSpec::Combine(_) => {}
            FeatureSpec::Fillet(x) => f(&mut x.radius),
            FeatureSpec::Chamfer(x) => {
                f(&mut x.d1);
                f(&mut x.d2);
            }
            FeatureSpec::VariableFillet(x) => {
                f(&mut x.start);
                f(&mut x.end);
            }
            FeatureSpec::Shell(x) => f(&mut x.thickness),
            FeatureSpec::Draft(x) => {
                for p in &mut x.neutral_origin {
                    f(p);
                }
                f(&mut x.angle);
            }
            FeatureSpec::Offset(x) => f(&mut x.distance),
            FeatureSpec::Hole(h) => {
                plane_params(&mut h.plane, &mut |p| f(p));
                for pt in &mut h.points {
                    f(&mut pt[0]);
                    f(&mut pt[1]);
                }
                match &mut h.kind {
                    HoleKind::Simple { diameter } => f(diameter),
                    HoleKind::Counterbore {
                        diameter,
                        cb_diameter,
                        cb_depth,
                    } => {
                        f(diameter);
                        f(cb_diameter);
                        f(cb_depth);
                    }
                    HoleKind::Countersink {
                        diameter,
                        cs_diameter,
                        cs_angle,
                    } => {
                        f(diameter);
                        f(cs_diameter);
                        f(cs_angle);
                    }
                }
                f(&mut h.depth);
            }
            FeatureSpec::LinearPattern(p) => {
                for s in &mut p.spacing {
                    f(s);
                }
                f(&mut p.count);
            }
            FeatureSpec::CircularPattern(p) => {
                for s in &mut p.axis_origin {
                    f(s);
                }
                f(&mut p.count);
                f(&mut p.total_angle);
            }
            FeatureSpec::Mirror(m) => match &mut m.plane {
                MirrorPlane::Xy(o) | MirrorPlane::Yz(o) | MirrorPlane::Xz(o) => f(o),
            },
            FeatureSpec::Import(_) => {}
            FeatureSpec::Subd(s) => {
                f(&mut s.scale);
                f(&mut s.tolerance);
            }
            FeatureSpec::SheetMetal(s) => {
                f(&mut s.thickness);
                f(&mut s.k_factor);
                f(&mut s.default_radius);
                f(&mut s.width);
                for g in &mut s.segs {
                    match g {
                        SheetSegSpec::Flat { length } => f(length),
                        SheetSegSpec::Bend { angle, radius } => {
                            f(angle);
                            if let Some(r) = radius {
                                f(r);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Visit every persistent reference the feature holds.
    pub fn for_each_reference_mut(&mut self, f: &mut dyn FnMut(&mut Reference)) {
        fn plane(p: &mut PlaneSpec, f: &mut dyn FnMut(&mut Reference)) {
            if let PlaneSpec::Face(r) = p {
                f(r);
            }
        }
        match self {
            FeatureSpec::Sketch(s) => plane(&mut s.plane, f),
            FeatureSpec::DatumPlane(d) => plane(&mut d.base, f),
            FeatureSpec::Hole(h) => plane(&mut h.plane, f),
            FeatureSpec::Fillet(x) => {
                for e in &mut x.edges {
                    e.faces.iter_mut().for_each(&mut *f);
                }
            }
            FeatureSpec::Chamfer(x) => {
                for e in &mut x.edges {
                    e.faces.iter_mut().for_each(&mut *f);
                }
            }
            FeatureSpec::VariableFillet(x) => x.edge.faces.iter_mut().for_each(&mut *f),
            FeatureSpec::Shell(x) => x.open.iter_mut().for_each(&mut *f),
            FeatureSpec::Draft(x) => x.faces.iter_mut().for_each(&mut *f),
            _ => {}
        }
    }

    /// Variable names read by the feature's expressions.
    pub fn free_vars(&self) -> Vec<String> {
        let mut c = self.clone();
        let mut v: Vec<String> = Vec::new();
        c.for_each_param_mut(&mut |p| v.extend(p.free_vars()));
        v.sort();
        v.dedup();
        v
    }

    /// A copy whose parameters are all literals.
    pub fn resolved(&self, scope: &dyn Scope) -> Result<FeatureSpec, EvalError> {
        let mut c = self.clone();
        let mut err = None;
        c.for_each_param_mut(&mut |p| {
            if err.is_some() {
                return;
            }
            match p.eval(scope) {
                Ok(q) => *p = Param::Lit(q),
                Err(e) => err = Some(e),
            }
        });
        match err {
            Some(e) => Err(e),
            None => Ok(c),
        }
    }

    /// The body the feature modifies, if it names one explicitly.
    pub fn explicit_base(&self) -> Option<FeatureId> {
        match self {
            FeatureSpec::Extrude(x) => x.base,
            FeatureSpec::Revolve(x) => x.base,
            FeatureSpec::Loft(x) => x.base,
            FeatureSpec::Sweep(x) => x.base,
            FeatureSpec::Fillet(x) => x.base,
            FeatureSpec::Chamfer(x) => x.base,
            FeatureSpec::VariableFillet(x) => x.base,
            FeatureSpec::Shell(x) => x.base,
            FeatureSpec::Draft(x) => x.base,
            FeatureSpec::Offset(x) => x.base,
            FeatureSpec::Hole(x) => x.base,
            FeatureSpec::LinearPattern(x) => x.base,
            FeatureSpec::CircularPattern(x) => x.base,
            FeatureSpec::Mirror(x) => x.base,
            _ => None,
        }
    }

    /// Does the feature take the body so far as an input (the previous solid unless `base` names one)?
    pub fn needs_body(&self) -> bool {
        match self {
            FeatureSpec::Parameter { .. }
            | FeatureSpec::DatumPlane(_)
            | FeatureSpec::Combine(_)
            | FeatureSpec::SheetMetal(_)
            | FeatureSpec::Import(_)
            | FeatureSpec::Subd(_) => false,
            FeatureSpec::Sketch(s) => matches!(s.plane, PlaneSpec::Face(_)),
            FeatureSpec::Extrude(x) => x.op != BodyOp::New,
            FeatureSpec::Revolve(x) => x.op != BodyOp::New,
            FeatureSpec::Loft(x) => x.op != BodyOp::New,
            FeatureSpec::Sweep(x) => x.op != BodyOp::New,
            _ => true,
        }
    }

    /// Features (other than parameters and the implicit previous body) this feature reads.
    pub fn explicit_dependencies(&self) -> Vec<FeatureId> {
        let mut v = Vec::new();
        let refs = |rs: &[Reference], v: &mut Vec<FeatureId>| {
            for r in rs {
                v.push(r.name.feature);
            }
        };
        match self {
            FeatureSpec::Parameter { .. } => {}
            FeatureSpec::Sketch(s) => plane_refs(&s.plane, &mut v),
            FeatureSpec::DatumPlane(d) => plane_refs(&d.base, &mut v),
            FeatureSpec::Extrude(x) => v.push(x.sketch),
            FeatureSpec::Revolve(x) => v.push(x.sketch),
            FeatureSpec::Loft(x) => v.extend(x.sketches.iter().copied()),
            FeatureSpec::Sweep(x) => {
                v.push(x.profile);
                v.push(x.path);
            }
            FeatureSpec::Fillet(x) => {
                for e in &x.edges {
                    refs(&e.faces, &mut v);
                }
            }
            FeatureSpec::Chamfer(x) => {
                for e in &x.edges {
                    refs(&e.faces, &mut v);
                }
            }
            FeatureSpec::VariableFillet(x) => refs(&x.edge.faces, &mut v),
            FeatureSpec::Shell(x) => refs(&x.open, &mut v),
            FeatureSpec::Draft(x) => refs(&x.faces, &mut v),
            FeatureSpec::Offset(_) => {}
            FeatureSpec::Combine(x) => {
                v.push(x.target);
                v.push(x.tool);
            }
            FeatureSpec::Hole(x) => plane_refs(&x.plane, &mut v),
            FeatureSpec::LinearPattern(x) => v.push(x.source),
            FeatureSpec::CircularPattern(x) => v.push(x.source),
            FeatureSpec::Mirror(x) => v.push(x.source),
            FeatureSpec::SheetMetal(_) | FeatureSpec::Subd(_) | FeatureSpec::Import(_) => {}
        }
        if let Some(b) = self.explicit_base() {
            v.push(b);
        }
        v.sort();
        v.dedup();
        v
    }
}
