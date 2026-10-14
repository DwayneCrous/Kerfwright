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
    /// For a sketch on a face: the body the face belongs to (`None` = the body so far).
    #[serde(default)]
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatumPlaneSpec {
    /// How the plane is made; the base, offset and tilt below are used by `PlaneMode::Offset`.
    #[serde(default)]
    pub mode: PlaneMode,
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
    /// Make a wall of this thickness along the profile instead of a solid.
    #[serde(default)]
    pub thin: Option<ThinWall>,
    /// Stop at this planar face of the body instead of after a distance.
    #[serde(default)]
    pub to_face: Option<Reference>,
    /// Start the extrusion this far from the sketch plane (along the extrusion direction).
    #[serde(default)]
    pub start_offset: Option<Param>,
    /// Extrude only these regions of the sketch (faces of its planar arrangement). Empty = the default
    /// regions: everything the shapes enclose, with the shapes inside them as holes.
    #[serde(default)]
    pub regions: Vec<RegionPick>,
}

/// A region of a sketch, named so that it can be found again after the sketch changes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum RegionPick {
    /// The region whose boundary is made of the entities with these labels (sorted, without repeats) and
    /// which contains the point `at` (several regions can share the same boundary entities: the halves of a
    /// circle cut by a line).
    Face { labels: Vec<u32>, at: [f64; 2] },
    /// A choice saved before sketches were split at their intersections: the set of closed loops (by the
    /// smallest label of each loop's entities) the region lay inside. It is turned into a position when used.
    Legacy(Vec<u32>),
}

/// Which side of the profile curve a thin extrusion's wall lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThinSide {
    /// Inside the profile (the profile is the outer face of the wall).
    Inside,
    /// Outside the profile (the profile is the inner face of the wall).
    Outside,
    /// Half on each side.
    Centered,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThinWall {
    pub thickness: Param,
    pub side: ThinSide,
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
    /// Make a surface (a sheet body) from the curves of the sketch instead of a solid from its regions.
    #[serde(default)]
    pub surface: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AxisSpec {
    SketchX,
    SketchY,
    /// A line entity of the profile sketch (by label).
    SketchLine(u32),
    /// A straight edge of the body, named by the two faces that meet there.
    Edge(Box<EdgeRef>),
    /// A construction axis feature.
    Datum(FeatureId),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RevolveSpec {
    pub sketch: FeatureId,
    pub axis: AxisSpec,
    pub angle: Param,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
    /// Make a surface from the curves of the sketch instead of a solid from its regions.
    #[serde(default)]
    pub surface: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoftSpec {
    pub sketches: Vec<FeatureId>,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
    /// Make a surface from the curves of the sketches instead of a solid from their regions.
    #[serde(default)]
    pub surface: bool,
    /// A surface straight between neighbouring sketches instead of a smooth one.
    #[serde(default)]
    pub ruled: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SweepSpec {
    pub profile: FeatureId,
    /// A sketch whose first non-construction line, arc or spline is the path.
    pub path: FeatureId,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
    /// Make a surface from the curves of the profile sketch instead of a solid from its regions.
    #[serde(default)]
    pub surface: bool,
    /// The total turn of the profile along the path.
    #[serde(default)]
    pub twist: Option<Param>,
    /// A sketch whose first arc, spline or line guides the profile's orientation and scale.
    #[serde(default)]
    pub guide: Option<FeatureId>,
}

/// Which way a thickened surface grows from the direction its faces look.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThickenDir {
    Normal,
    Reverse,
    Both,
}

/// What a trim cuts with: a plane, or the surface of a face (carried on past the face's edges).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TrimToolSpec {
    Plane(PlaneSpec),
    Face { source: FeatureId, face: Reference },
}

/// Cut a surface with a plane or a surface and keep one side.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimSpec {
    pub tool: TrimToolSpec,
    /// Keep the side the tool's normal points to (else the other).
    pub keep_positive: bool,
    pub base: Option<FeatureId>,
}

/// Carry a surface on past its open edges.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExtendSpec {
    pub edges: Vec<BoundaryRef>,
    pub length: Param,
    pub base: Option<FeatureId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MatchLevel {
    G1,
    G2,
}

/// Reshape a NURBS patch near an open edge to meet a neighbouring face.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MatchSpec {
    pub edge: BoundaryRef,
    /// The face to meet, and the feature whose body it is on.
    pub neighbour_source: FeatureId,
    pub neighbour_face: Reference,
    pub level: MatchLevel,
    pub base: Option<FeatureId>,
}

/// Join surface bodies along coincident edges; a group that closes becomes a solid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StitchSpec {
    /// The features whose bodies are joined (they are used up).
    pub bodies: Vec<FeatureId>,
    /// Open edges closer than this are joined.
    pub tolerance: Param,
}

/// Split a body into its faces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct UnstitchSpec {
    pub base: Option<FeatureId>,
}

/// Give a surface thickness.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThickenSpec {
    pub thickness: Param,
    pub side: ThickenDir,
    pub base: Option<FeatureId>,
}

/// Move a surface along its normal.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffsetSurfaceSpec {
    pub distance: Param,
    pub base: Option<FeatureId>,
}

/// An edge picked as the boundary of a surface: one between two named faces, or an open edge of a
/// surface, named by its face and its place among that face's open edges.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum BoundaryRef {
    Shared(EdgeRef),
    Free { face: Reference, index: u32 },
}

/// A boundary edge of a patch or ruled surface, with the feature whose body it is on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoundaryEdge {
    /// The feature that made the body the edge is on (the body keeps living; the surface is a new one).
    pub source: FeatureId,
    pub edge: BoundaryRef,
    /// Make the surface tangent to the face this open edge belongs to.
    #[serde(default)]
    pub tangent: bool,
}

/// A surface filling a closed loop of edges.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PatchSpec {
    pub edges: Vec<BoundaryEdge>,
}

/// A surface of straight lines between two edges.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuledSpec {
    pub a: BoundaryEdge,
    pub b: BoundaryEdge,
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

/// A thread on a cylindrical face. Cosmetic by default: the geometry is not changed and the designation,
/// length and hand are recorded for drawings, the feature list and export. A `modelled` thread cuts the
/// helical groove into the body (see [`crate::thread_geom`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThreadSpec {
    pub face: Reference,
    pub designation: String,
    pub length: Param,
    /// A tapped hole (true) or a bolt (false).
    pub internal: bool,
    pub base: Option<FeatureId>,
    /// Cut the thread into the body instead of only recording it.
    #[serde(default)]
    pub modelled: bool,
    /// Right-handed (the usual) or left-handed.
    #[serde(default = "right_handed")]
    pub right_hand: bool,
}

fn right_handed() -> bool {
    true
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
    /// Keep the tool body (it stays a body of its own) instead of consuming it.
    #[serde(default)]
    pub keep_tool: bool,
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
    /// Drilled to the tap-drill diameter of a standard size (see [`crate::threads`]) with an internal
    /// thread of `thread_length` (the rest of the hole stays plain). The thread is cosmetic unless
    /// `modelled`; then the hole is drilled to the thread's minor diameter and the helical groove is cut.
    Tapped {
        designation: String,
        thread_length: Param,
        #[serde(default)]
        modelled: bool,
        #[serde(default = "right_handed")]
        right_hand: bool,
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
    /// A construction axis to turn about; when set, `axis_origin` and `axis_dir` are not used.
    #[serde(default)]
    pub axis_feature: Option<FeatureId>,
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
    /// A construction plane feature.
    Datum(FeatureId),
    /// A planar face of the body.
    Face(Reference),
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

/// Move (translate, then rotate about a world axis through the origin) a body, or put a moved copy next to it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MoveBodySpec {
    pub base: Option<FeatureId>,
    pub delta: [Param; 3],
    /// World axis of the rotation: 0 = X, 1 = Y, 2 = Z.
    pub axis: u8,
    pub angle: Param,
    /// Leave the original in place and add the moved body as a second body.
    pub copy: bool,
}

/// Uniform scale of a body about a point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScaleBodySpec {
    pub base: Option<FeatureId>,
    pub factor: Param,
    pub centre: [Param; 3],
}

/// Which side of the splitting plane a split keeps (relative to the plane's normal).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SplitSide {
    Positive,
    Negative,
}

/// One half of a body cut by a plane; a Split Body command adds one feature per half.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SplitBodySpec {
    pub base: Option<FeatureId>,
    pub plane: PlaneSpec,
    pub keep: SplitSide,
}

/// A wall standing on the sketch plane: every line and arc of the sketch is thickened (in the plane) and the
/// result is extruded.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RibSpec {
    pub sketch: FeatureId,
    pub thickness: Param,
    pub depth: Param,
    pub symmetric: bool,
    pub flip: bool,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
}

/// A tube of the given outer diameter (and wall thickness) along the path of a sketch.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PipeSpec {
    pub path: FeatureId,
    pub diameter: Param,
    /// Wall thickness; at least half the diameter makes a solid rod.
    pub wall: Param,
    pub op: BodyOp,
    pub base: Option<FeatureId>,
}

/// File format of an imported body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImportFormat {
    Step,
    Iges,
}

/// A point given by coordinates (mm) or by a datum point feature.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(clippy::large_enum_variant)] // a handful per document, built once
pub enum PointSrc {
    Coords([Param; 3]),
    Datum(FeatureId),
}

/// How a construction plane is made.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub enum PlaneMode {
    /// The base plane moved along its normal and tilted about its x axis.
    #[default]
    Offset,
    /// Halfway between the base plane and another (parallel) plane.
    Midplane(PlaneSpec),
    /// Through three points.
    ThreePoints(Box<[PointSrc; 3]>),
}

/// How a construction axis is made.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(clippy::large_enum_variant)] // a handful per document, built once
pub enum AxisKind {
    /// Through two points.
    TwoPoints(Box<[PointSrc; 2]>),
    /// Along a straight edge of the body.
    Edge(Box<EdgeRef>),
    /// Through the point `at` (plane coordinates of the face) normal to a planar face of the body.
    NormalToFace { face: Reference, at: [Param; 2] },
    /// Where two planes meet.
    PlanePlane(Box<(PlaneSpec, PlaneSpec)>),
}

/// A construction axis: a line the Revolve, Circular Pattern and sketches can use.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatumAxisSpec {
    pub kind: AxisKind,
}

/// How a construction point is made.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PointKind {
    Coords([Param; 3]),
    /// The middle of an edge of the body.
    EdgeMidpoint(Box<EdgeRef>),
    /// Where an axis pierces a plane.
    AxisPlane {
        axis: FeatureId,
        plane: PlaneSpec,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatumPointSpec {
    pub kind: PointKind,
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

/// A triangle mesh read from a file (STL): a body that is not a solid. It can be moved, scaled, measured and
/// exported; "convert to solid" ([`MeshToSolidSpec`]) turns it into a faceted B-rep.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImportMeshSpec {
    /// Name of the file it came from (for display).
    pub source: String,
    /// The unit the file is written in.
    pub units: crate::mesh_body::MeshUnits,
    /// The file has y up (the model has z up).
    pub y_up: bool,
    pub data: crate::mesh_body::MeshBlob,
}

/// Turn a closed mesh body into a solid of planar faces.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshToSolidSpec {
    pub base: Option<FeatureId>,
    /// Make one face of the triangles that lie in one plane.
    pub merge_coplanar: bool,
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
    Patch(PatchSpec),
    Ruled(RuledSpec),
    Trim(TrimSpec),
    Extend(ExtendSpec),
    Match(MatchSpec),
    Stitch(StitchSpec),
    Unstitch(UnstitchSpec),
    Thicken(ThickenSpec),
    OffsetSurface(OffsetSurfaceSpec),
    Fillet(FilletSpec),
    Chamfer(ChamferSpec),
    VariableFillet(VariableFilletSpec),
    Shell(ShellSpec),
    Draft(DraftSpec),
    Thread(ThreadSpec),
    Offset(OffsetSpec),
    Combine(CombineSpec),
    Hole(HoleSpec),
    LinearPattern(LinearPatternSpec),
    CircularPattern(CircularPatternSpec),
    Mirror(MirrorSpec),
    SheetMetal(SheetMetalSpec),
    Subd(SubdSpec),
    Import(ImportSpec),
    ImportMesh(ImportMeshSpec),
    MeshToSolid(MeshToSolidSpec),
    MoveBody(MoveBodySpec),
    ScaleBody(ScaleBodySpec),
    SplitBody(SplitBodySpec),
    Rib(RibSpec),
    Pipe(PipeSpec),
    DatumAxis(DatumAxisSpec),
    DatumPoint(DatumPointSpec),
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
            FeatureSpec::DatumAxis(_) => "DatumAxis",
            FeatureSpec::DatumPoint(_) => "DatumPoint",
            FeatureSpec::Extrude(_) => "Extrude",
            FeatureSpec::Revolve(_) => "Revolve",
            FeatureSpec::Loft(_) => "Loft",
            FeatureSpec::Sweep(_) => "Sweep",
            FeatureSpec::Patch(_) => "Patch",
            FeatureSpec::Ruled(_) => "Ruled",
            FeatureSpec::Trim(_) => "Trim",
            FeatureSpec::Extend(_) => "Extend",
            FeatureSpec::Match(_) => "Match",
            FeatureSpec::Stitch(_) => "Stitch",
            FeatureSpec::Unstitch(_) => "Unstitch",
            FeatureSpec::Thicken(_) => "Thicken",
            FeatureSpec::OffsetSurface(_) => "OffsetSurface",
            FeatureSpec::Fillet(_) => "Fillet",
            FeatureSpec::Chamfer(_) => "Chamfer",
            FeatureSpec::VariableFillet(_) => "VariableFillet",
            FeatureSpec::Shell(_) => "Shell",
            FeatureSpec::Draft(_) => "Draft",
            FeatureSpec::Thread(_) => "Thread",
            FeatureSpec::Offset(_) => "Offset",
            FeatureSpec::Combine(_) => "Combine",
            FeatureSpec::Hole(_) => "Hole",
            FeatureSpec::LinearPattern(_) => "LinearPattern",
            FeatureSpec::CircularPattern(_) => "CircularPattern",
            FeatureSpec::Mirror(_) => "Mirror",
            FeatureSpec::SheetMetal(_) => "SheetMetal",
            FeatureSpec::Subd(_) => "Subd",
            FeatureSpec::Import(_) => "Import",
            FeatureSpec::ImportMesh(_) => "ImportMesh",
            FeatureSpec::MeshToSolid(_) => "MeshToSolid",
            FeatureSpec::MoveBody(m) => {
                if m.copy {
                    "CopyBody"
                } else {
                    "MoveBody"
                }
            }
            FeatureSpec::ScaleBody(_) => "ScaleBody",
            FeatureSpec::SplitBody(_) => "SplitBody",
            FeatureSpec::Rib(_) => "Rib",
            FeatureSpec::Pipe(_) => "Pipe",
        }
    }

    /// Does the feature produce a solid body?
    pub fn produces_solid(&self) -> bool {
        !matches!(
            self,
            FeatureSpec::Parameter { .. }
                | FeatureSpec::Sketch(_)
                | FeatureSpec::DatumPlane(_)
                | FeatureSpec::DatumAxis(_)
                | FeatureSpec::DatumPoint(_)
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
                match &mut d.mode {
                    PlaneMode::Offset => {}
                    PlaneMode::Midplane(o) => plane_params(o, &mut |p| f(p)),
                    PlaneMode::ThreePoints(pts) => {
                        for pt in pts.iter_mut() {
                            if let PointSrc::Coords(c) = pt {
                                for p in c.iter_mut() {
                                    f(p);
                                }
                            }
                        }
                    }
                }
            }
            FeatureSpec::DatumAxis(a) => match &mut a.kind {
                AxisKind::TwoPoints(pts) => {
                    for pt in pts.iter_mut() {
                        if let PointSrc::Coords(c) = pt {
                            for p in c.iter_mut() {
                                f(p);
                            }
                        }
                    }
                }
                AxisKind::NormalToFace { at, .. } => {
                    f(&mut at[0]);
                    f(&mut at[1]);
                }
                AxisKind::PlanePlane(pp) => {
                    plane_params(&mut pp.0, &mut |p| f(p));
                    plane_params(&mut pp.1, &mut |p| f(p));
                }
                AxisKind::Edge(_) => {}
            },
            FeatureSpec::DatumPoint(d) => match &mut d.kind {
                PointKind::Coords(c) => {
                    for p in c.iter_mut() {
                        f(p);
                    }
                }
                PointKind::AxisPlane { plane, .. } => plane_params(plane, &mut |p| f(p)),
                PointKind::EdgeMidpoint(_) => {}
            },
            FeatureSpec::Extrude(e) => {
                f(&mut e.depth);
                if let Some(p) = &mut e.extent.second_side {
                    f(p);
                }
                if let Some(p) = &mut e.extent.taper {
                    f(p);
                }
                if let Some(t) = &mut e.extent.thin {
                    f(&mut t.thickness);
                }
                if let Some(p) = &mut e.extent.start_offset {
                    f(p);
                }
            }
            FeatureSpec::Revolve(r) => f(&mut r.angle),
            FeatureSpec::Sweep(w) => {
                if let Some(p) = &mut w.twist {
                    f(p);
                }
            }
            FeatureSpec::Loft(_)
            | FeatureSpec::Combine(_)
            | FeatureSpec::Patch(_)
            | FeatureSpec::Ruled(_)
            | FeatureSpec::Unstitch(_)
            | FeatureSpec::Match(_) => {}
            FeatureSpec::Trim(t) => {
                if let TrimToolSpec::Plane(pl) = &mut t.tool {
                    plane_params(pl, &mut |p| f(p));
                }
            }
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
            FeatureSpec::Thread(x) => f(&mut x.length),
            FeatureSpec::Offset(x) => f(&mut x.distance),
            FeatureSpec::Thicken(x) => f(&mut x.thickness),
            FeatureSpec::Extend(x) => f(&mut x.length),
            FeatureSpec::OffsetSurface(x) => f(&mut x.distance),
            FeatureSpec::Stitch(x) => f(&mut x.tolerance),
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
                    HoleKind::Tapped { thread_length, .. } => f(thread_length),
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
                MirrorPlane::Datum(_) | MirrorPlane::Face(_) => {}
            },
            FeatureSpec::Import(_) | FeatureSpec::ImportMesh(_) | FeatureSpec::MeshToSolid(_) => {}
            FeatureSpec::MoveBody(m) => {
                for p in &mut m.delta {
                    f(p);
                }
                f(&mut m.angle);
            }
            FeatureSpec::ScaleBody(m) => {
                f(&mut m.factor);
                for p in &mut m.centre {
                    f(p);
                }
            }
            FeatureSpec::SplitBody(m) => plane_params(&mut m.plane, &mut |p| f(p)),
            FeatureSpec::Rib(r) => {
                f(&mut r.thickness);
                f(&mut r.depth);
            }
            FeatureSpec::Pipe(p) => {
                f(&mut p.diameter);
                f(&mut p.wall);
            }
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
            FeatureSpec::Thread(x) => f(&mut x.face),
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
            FeatureSpec::Thread(x) => x.base,
            FeatureSpec::Offset(x) => x.base,
            FeatureSpec::Unstitch(x) => x.base,
            FeatureSpec::Trim(x) => x.base,
            FeatureSpec::Extend(x) => x.base,
            FeatureSpec::Match(x) => x.base,
            FeatureSpec::Thicken(x) => x.base,
            FeatureSpec::OffsetSurface(x) => x.base,
            FeatureSpec::Hole(x) => x.base,
            FeatureSpec::LinearPattern(x) => x.base,
            FeatureSpec::CircularPattern(x) => x.base,
            FeatureSpec::Mirror(x) => x.base,
            FeatureSpec::Sketch(x) => x.base,
            FeatureSpec::MoveBody(x) => x.base,
            FeatureSpec::ScaleBody(x) => x.base,
            FeatureSpec::SplitBody(x) => x.base,
            FeatureSpec::Rib(x) => x.base,
            FeatureSpec::Pipe(x) => x.base,
            FeatureSpec::MeshToSolid(x) => x.base,
            _ => None,
        }
    }

    /// Point the feature at another body (`None` = the body so far). Features that take no body ignore it.
    pub fn set_base(&mut self, b: Option<FeatureId>) {
        match self {
            FeatureSpec::Extrude(x) => x.base = b,
            FeatureSpec::Revolve(x) => x.base = b,
            FeatureSpec::Loft(x) => x.base = b,
            FeatureSpec::Sweep(x) => x.base = b,
            FeatureSpec::Fillet(x) => x.base = b,
            FeatureSpec::Chamfer(x) => x.base = b,
            FeatureSpec::VariableFillet(x) => x.base = b,
            FeatureSpec::Shell(x) => x.base = b,
            FeatureSpec::Draft(x) => x.base = b,
            FeatureSpec::Thread(x) => x.base = b,
            FeatureSpec::Offset(x) => x.base = b,
            FeatureSpec::Unstitch(x) => x.base = b,
            FeatureSpec::Trim(x) => x.base = b,
            FeatureSpec::Extend(x) => x.base = b,
            FeatureSpec::Match(x) => x.base = b,
            FeatureSpec::Thicken(x) => x.base = b,
            FeatureSpec::OffsetSurface(x) => x.base = b,
            FeatureSpec::Hole(x) => x.base = b,
            FeatureSpec::LinearPattern(x) => x.base = b,
            FeatureSpec::CircularPattern(x) => x.base = b,
            FeatureSpec::Mirror(x) => x.base = b,
            FeatureSpec::Sketch(x) => x.base = b,
            FeatureSpec::MoveBody(x) => x.base = b,
            FeatureSpec::ScaleBody(x) => x.base = b,
            FeatureSpec::SplitBody(x) => x.base = b,
            FeatureSpec::Rib(x) => x.base = b,
            FeatureSpec::Pipe(x) => x.base = b,
            FeatureSpec::MeshToSolid(x) => x.base = b,
            _ => {}
        }
    }

    /// Does the feature use up the body it reads? A copy leaves its source alive as a body of its own.
    pub fn consumes_body(&self) -> bool {
        !matches!(self, FeatureSpec::MoveBody(m) if m.copy)
    }

    /// Does the feature take the body so far as an input (the previous solid unless `base` names one)?
    pub fn needs_body(&self) -> bool {
        match self {
            FeatureSpec::Parameter { .. }
            | FeatureSpec::DatumPlane(_)
            | FeatureSpec::Combine(_)
            | FeatureSpec::Patch(_)
            | FeatureSpec::Ruled(_)
            | FeatureSpec::Stitch(_)
            | FeatureSpec::SheetMetal(_)
            | FeatureSpec::Import(_)
            | FeatureSpec::ImportMesh(_)
            | FeatureSpec::Subd(_) => false,
            FeatureSpec::Sketch(s) => matches!(s.plane, PlaneSpec::Face(_)),
            FeatureSpec::Extrude(x) => x.op != BodyOp::New,
            FeatureSpec::Revolve(x) => x.op != BodyOp::New,
            FeatureSpec::Loft(x) => x.op != BodyOp::New,
            FeatureSpec::Sweep(x) => x.op != BodyOp::New,
            FeatureSpec::Rib(x) => x.op != BodyOp::New,
            FeatureSpec::Pipe(x) => x.op != BodyOp::New,
            FeatureSpec::DatumAxis(a) => {
                matches!(a.kind, AxisKind::Edge(_) | AxisKind::NormalToFace { .. })
            }
            FeatureSpec::DatumPoint(p) => matches!(p.kind, PointKind::EdgeMidpoint(_)),
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
            FeatureSpec::DatumPlane(d) => {
                plane_refs(&d.base, &mut v);
                match &d.mode {
                    PlaneMode::Offset => {}
                    PlaneMode::Midplane(o) => plane_refs(o, &mut v),
                    PlaneMode::ThreePoints(pts) => {
                        for p in pts.iter() {
                            if let PointSrc::Datum(id) = p {
                                v.push(*id);
                            }
                        }
                    }
                }
            }
            FeatureSpec::DatumAxis(a) => match &a.kind {
                AxisKind::TwoPoints(pts) => {
                    for p in pts.iter() {
                        if let PointSrc::Datum(id) = p {
                            v.push(*id);
                        }
                    }
                }
                AxisKind::Edge(e) => refs(&e.faces, &mut v),
                AxisKind::NormalToFace { face, .. } => v.push(face.name.feature),
                AxisKind::PlanePlane(pp) => {
                    plane_refs(&pp.0, &mut v);
                    plane_refs(&pp.1, &mut v);
                }
            },
            FeatureSpec::DatumPoint(p) => match &p.kind {
                PointKind::Coords(_) => {}
                PointKind::EdgeMidpoint(e) => refs(&e.faces, &mut v),
                PointKind::AxisPlane { axis, plane } => {
                    v.push(*axis);
                    plane_refs(plane, &mut v);
                }
            },
            FeatureSpec::Extrude(x) => {
                v.push(x.sketch);
                if let Some(r) = &x.extent.to_face {
                    v.push(r.name.feature);
                }
            }
            FeatureSpec::Rib(x) => v.push(x.sketch),
            FeatureSpec::Pipe(x) => v.push(x.path),
            FeatureSpec::Revolve(x) => {
                v.push(x.sketch);
                match &x.axis {
                    AxisSpec::Edge(e) => refs(&e.faces, &mut v),
                    AxisSpec::Datum(a) => v.push(*a),
                    _ => {}
                }
            }
            FeatureSpec::Loft(x) => v.extend(x.sketches.iter().copied()),
            FeatureSpec::Sweep(x) => {
                v.push(x.profile);
                v.push(x.path);
                v.extend(x.guide);
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
            FeatureSpec::Thread(x) => refs(std::slice::from_ref(&x.face), &mut v),
            FeatureSpec::Offset(_)
            | FeatureSpec::Unstitch(_)
            | FeatureSpec::Thicken(_)
            | FeatureSpec::OffsetSurface(_) => {}
            FeatureSpec::Combine(x) => {
                v.push(x.target);
                v.push(x.tool);
            }
            FeatureSpec::Trim(x) => match &x.tool {
                TrimToolSpec::Plane(pl) => plane_refs(pl, &mut v),
                TrimToolSpec::Face { source, face } => {
                    v.push(*source);
                    v.push(face.name.feature);
                }
            },
            FeatureSpec::Extend(x) => {
                for e in &x.edges {
                    match e {
                        BoundaryRef::Shared(r) => refs(&r.faces, &mut v),
                        BoundaryRef::Free { face, .. } => v.push(face.name.feature),
                    }
                }
            }
            FeatureSpec::Match(x) => {
                v.push(x.neighbour_source);
                v.push(x.neighbour_face.name.feature);
                match &x.edge {
                    BoundaryRef::Shared(r) => refs(&r.faces, &mut v),
                    BoundaryRef::Free { face, .. } => v.push(face.name.feature),
                }
            }
            FeatureSpec::Stitch(x) => v.extend(x.bodies.iter().copied()),
            FeatureSpec::Patch(x) => v.extend(x.edges.iter().map(|e| e.source)),
            FeatureSpec::Ruled(x) => {
                v.push(x.a.source);
                v.push(x.b.source);
            }
            FeatureSpec::Hole(x) => plane_refs(&x.plane, &mut v),
            FeatureSpec::LinearPattern(x) => v.push(x.source),
            FeatureSpec::CircularPattern(x) => {
                v.push(x.source);
                if let Some(a) = x.axis_feature {
                    v.push(a);
                }
            }
            FeatureSpec::Mirror(x) => {
                v.push(x.source);
                match &x.plane {
                    MirrorPlane::Datum(d) => v.push(*d),
                    MirrorPlane::Face(r) => v.push(r.name.feature),
                    _ => {}
                }
            }
            FeatureSpec::SplitBody(x) => plane_refs(&x.plane, &mut v),
            FeatureSpec::SheetMetal(_)
            | FeatureSpec::Subd(_)
            | FeatureSpec::Import(_)
            | FeatureSpec::ImportMesh(_)
            | FeatureSpec::MeshToSolid(_)
            | FeatureSpec::MoveBody(_)
            | FeatureSpec::ScaleBody(_) => {}
        }
        if let Some(b) = self.explicit_base() {
            v.push(b);
        }
        v.sort();
        v.dedup();
        v
    }
}
