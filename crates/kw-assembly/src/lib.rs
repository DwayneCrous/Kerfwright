//! Assemblies: instances of parts placed by joints.
//!
//! * A [`Part`] is a feature [`Document`] plus named connection frames (origin, z axis, x axis in the
//!   part's own coordinates); or a whole sub-[`Assembly`] (`Part::sub`), whose inner frames are named
//!   `Instance.frame`.
//! * A [`Joint`] ties a frame of one instance to a frame of another with a motion: rigid, revolute (about z),
//!   slider (along z), cylindrical, planar or ball, optionally flipped and offset. Joint values are
//!   parameters or expressions; limits are enforced.
//! * Instances may be **grounded** (several allowed; each sits at its own placement `at`), **floating**
//!   (no joint: it stays at `at`) or placed by joints. Instances can be tied together in a **rigid group**
//!   that keeps the relative positions they had when it was made, and an **as-built joint** takes the
//!   current relative position as the joint's zero.
//! * The joints form a kinematic tree rooted at the grounded instances; [`Assembly::solve`] walks it and
//!   returns every instance's placement. A joint that closes a **kinematic loop** (a four-bar, a
//!   slider-crank) is a loop-closure constraint: the values of the joints marked `solved` are found
//!   numerically so that the loop closes, and the others are the inputs that drive the mechanism
//!   ([`Assembly::settle`], [`Assembly::mobility`], [`Assembly::sweep`]).
//! * [`Assembly::pose`] builds the posed bodies in one B-rep; [`Pose::interference`] finds overlaps with the
//!   kernel's Boolean; [`Pose::bom`] lists the parts; [`Pose::to_step`] writes one STEP file.
//!
//! An assembly is stored next to the model in a `.kwf` file as the entry `assembly.cbor`. Every field added
//! after the first version has a serde default, so older files still read.

// Numerical code indexes small fixed-size arrays; `!(x > 0.0)` is deliberate (it also rejects NaN).
#![allow(
    clippy::needless_range_loop,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod drag;
pub use drag::{Drag, Ray};
mod link;
pub use link::{relative_path, LinkKind, LinkStatus, PartLink, UpdateReport};
mod loops;
pub use loops::{Mobility, SweepStep};
pub(crate) mod numeric;

use kw_boolean::{boolean, BoolOptions, BooleanError, Op};
use kw_brep::{BRep, BodyId, CopyMap, CopyOptions};
use kw_expr::{Dim, Param, Quantity};
use kw_feature::kwf::KwfFile;
use kw_feature::{Document, Engine};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub name: String,
    pub origin: [f64; 3],
    pub z: [f64; 3],
    pub x: [f64; 3],
}

impl Frame {
    pub fn new(name: &str, origin: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
        Frame {
            name: name.into(),
            origin,
            z,
            x,
        }
    }

    fn to_frame3(&self) -> Result<Frame3, AssemblyError> {
        let z = Vector3::new(self.z[0], self.z[1], self.z[2]);
        let x0 = Vector3::new(self.x[0], self.x[1], self.x[2]);
        if z.norm() < 1e-12 || x0.norm() < 1e-12 {
            return Err(AssemblyError::BadFrame(self.name.clone()));
        }
        let z = z.normalize();
        let x = (x0 - z * x0.dot(&z)).try_normalize(1e-12).ok_or_else(|| {
            AssemblyError::BadFrame(format!("{} (its x axis is parallel to z)", self.name))
        })?;
        let y = z.cross(&x);
        Ok(Frame3 {
            origin: Point3::new(self.origin[0], self.origin[1], self.origin[2]),
            x,
            y,
            z,
        })
    }

    /// A frame from a `Frame3` (axes need not be unit length).
    pub fn from_frame3(name: &str, f: &Frame3) -> Frame {
        Frame::new(
            name,
            [f.origin.x, f.origin.y, f.origin.z],
            [f.z.x, f.z.y, f.z.z],
            [f.x.x, f.x.y, f.x.z],
        )
    }
}

/// Where an instance sits when nothing else decides it (a grounded or floating component): a rotation
/// about `axis` by `angle` (radians), then a translation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    pub t: [f64; 3],
    pub axis: [f64; 3],
    pub angle: f64,
}

impl Default for Placement {
    fn default() -> Self {
        Placement {
            t: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            angle: 0.0,
        }
    }
}

impl Placement {
    pub fn to_transform(&self) -> Transform3 {
        let axis = Vector3::new(self.axis[0], self.axis[1], self.axis[2]);
        let rot = if axis.norm() < 1e-12 || self.angle == 0.0 {
            Transform3::identity()
        } else {
            Transform3::rotation(axis, self.angle)
        };
        rot.then(&Transform3::translation(Vector3::new(
            self.t[0], self.t[1], self.t[2],
        )))
    }

    pub fn from_transform(t: &Transform3) -> Placement {
        let (axis, angle) =
            t.0.rotation
                .axis_angle()
                .map(|(a, ang)| ([a.x, a.y, a.z], ang))
                .unwrap_or(([0.0, 0.0, 1.0], 0.0));
        let tr = t.0.translation.vector;
        Placement {
            t: [tr.x, tr.y, tr.z],
            axis,
            angle,
        }
    }

    pub fn is_identity(&self) -> bool {
        self.t == [0.0; 3] && self.angle == 0.0
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Part {
    pub name: String,
    pub document: Document,
    pub frames: Vec<Frame>,
    /// A sub-assembly: this part is made of other instances (its document is empty). Joints reach its inner
    /// frames as `"Instance.frame"`.
    #[serde(default)]
    pub sub: Option<Box<Assembly>>,
    /// Material name for the bill of materials (a name the application knows).
    #[serde(default)]
    pub material: Option<String>,
    /// The file this part's model comes from, if it is linked (`document` is then a cached copy of it).
    #[serde(default)]
    pub link: Option<PartLink>,
}

impl Part {
    pub fn new(name: &str, document: Document, frames: Vec<Frame>) -> Part {
        Part {
            name: name.into(),
            document,
            frames,
            sub: None,
            material: None,
            link: None,
        }
    }

    /// The material for the bill of materials: the one set on the part, else the one of its first body.
    pub fn effective_material(&self) -> Option<String> {
        self.material.clone().or_else(|| {
            let d = &self.document;
            let head = *d.heads().first()?;
            d.appearance_of(d.body_root(head))?.material.clone()
        })
    }

    /// The connection frames a joint can use on this part: its own, or `Instance.frame` of a sub-assembly.
    pub fn frame_names(&self) -> Vec<String> {
        match &self.sub {
            None => self.frames.iter().map(|f| f.name.clone()).collect(),
            Some(sub) => {
                let mut v: Vec<String> = self.frames.iter().map(|f| f.name.clone()).collect();
                for inst in &sub.instances {
                    if let Some(p) = sub.parts.get(inst.part) {
                        for f in p.frame_names() {
                            v.push(format!("{}.{f}", inst.name));
                        }
                    }
                }
                v
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Instance {
    pub name: String,
    pub part: usize,
    /// Grounded: it stays at its placement and everything else moves relative to it.
    #[serde(default)]
    pub grounded: bool,
    /// Where it sits when it is grounded, or floating (not in any joint).
    #[serde(default)]
    pub at: Placement,
    /// Not drawn (and not part of interference or the bill of materials).
    #[serde(default)]
    pub hidden: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointKind {
    /// No motion.
    Rigid,
    /// Rotation about the joint's z axis: `[angle]`.
    Revolute,
    /// Translation along z: `[distance]`.
    Slider,
    /// Rotation about z and translation along it: `[angle, distance]`.
    Cylindrical,
    /// Translation in the joint's xy plane and rotation about z: `[x, y, angle]`.
    Planar,
    /// Rotation about the joint's x, then y, then z axis: `[about x, about y, about z]`.
    Ball,
}

impl JointKind {
    pub const ALL: [JointKind; 6] = [
        JointKind::Rigid,
        JointKind::Revolute,
        JointKind::Slider,
        JointKind::Cylindrical,
        JointKind::Planar,
        JointKind::Ball,
    ];

    pub fn name(self) -> &'static str {
        match self {
            JointKind::Rigid => "Rigid",
            JointKind::Revolute => "Revolute",
            JointKind::Slider => "Slider",
            JointKind::Cylindrical => "Cylindrical",
            JointKind::Planar => "Planar",
            JointKind::Ball => "Ball",
        }
    }

    /// How many values (degrees of freedom) the joint has.
    pub fn arity(self) -> usize {
        match self {
            JointKind::Rigid => 0,
            JointKind::Revolute | JointKind::Slider => 1,
            JointKind::Cylindrical => 2,
            JointKind::Planar | JointKind::Ball => 3,
        }
    }

    /// What each value is called.
    pub fn value_names(self) -> &'static [&'static str] {
        match self {
            JointKind::Rigid => &[],
            JointKind::Revolute => &["Angle"],
            JointKind::Slider => &["Distance"],
            JointKind::Cylindrical => &["Angle", "Distance"],
            JointKind::Planar => &["X", "Y", "Angle"],
            JointKind::Ball => &["About X", "About Y", "About Z"],
        }
    }

    /// The dimension of each value (length or angle).
    pub fn dims(self) -> &'static [Dim] {
        const L: Dim = Dim::LENGTH;
        const A: Dim = Dim::ANGLE;
        match self {
            JointKind::Rigid => &[],
            JointKind::Revolute => &[A],
            JointKind::Slider => &[L],
            JointKind::Cylindrical => &[A, L],
            JointKind::Planar => &[L, L, A],
            JointKind::Ball => &[A, A, A],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointEnd {
    pub instance: usize,
    pub frame: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    /// The side nearer the grounded instance.
    pub parent: JointEnd,
    pub child: JointEnd,
    pub values: Vec<Param>,
    /// Allowed range (SI units: mm or radians) of every value, if limited.
    pub limits: Vec<Option<(f64, f64)>>,
    /// Turn the joint's z axis round (a half turn about x).
    #[serde(default)]
    pub flip: bool,
    /// Offset of the child along the joint's x, y, z (mm), before the motion.
    #[serde(default)]
    pub offset: [f64; 3],
    /// Turn of the child about the joint's z (radians), before the motion.
    #[serde(default)]
    pub angle: f64,
    /// Made from the positions the components already had (zero motion reproduces them).
    #[serde(default)]
    pub as_built: bool,
    /// The values are unknowns of a kinematic loop: found by the loop solve so that the loops close,
    /// rather than given. A driven joint (`false`) keeps the values it has.
    #[serde(default)]
    pub solved: bool,
}

impl Joint {
    /// A joint with zero values, no limits and no offset.
    pub fn new(name: &str, kind: JointKind, parent: JointEnd, child: JointEnd) -> Joint {
        Joint {
            name: name.into(),
            kind,
            parent,
            child,
            values: zero_values(kind),
            limits: vec![None; kind.arity()],
            flip: false,
            offset: [0.0; 3],
            angle: 0.0,
            as_built: false,
            solved: false,
        }
    }
}

/// Zero of the right dimension for each degree of freedom of a joint.
pub fn zero_values(k: JointKind) -> Vec<Param> {
    k.dims()
        .iter()
        .map(|d| {
            if *d == Dim::ANGLE {
                Param::angle_deg(0.0)
            } else {
                Param::length(0.0)
            }
        })
        .collect()
}

/// Components that move as one, at the relative positions they had when the group was made.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RigidGroup {
    pub name: String,
    /// Instance indices; the first is the anchor the others are placed against.
    pub members: Vec<usize>,
    /// `rel[k]`: member `k`'s coordinates → the anchor's coordinates.
    pub rel: Vec<Placement>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Assembly {
    pub name: String,
    pub parts: Vec<Part>,
    pub instances: Vec<Instance>,
    /// The grounded instance of the first version of the file format. Used only while `grounds` is `None`.
    pub ground: usize,
    pub joints: Vec<Joint>,
    /// Named values the joint expressions may use, evaluated in order.
    pub params: Vec<(String, Param)>,
    /// Which instances are grounded (`None`: the single legacy `ground` index plus any instance flagged
    /// `grounded`).
    #[serde(default)]
    pub grounds: Option<Vec<usize>>,
    #[serde(default)]
    pub groups: Vec<RigidGroup>,
    /// Take the model of a linked part again as soon as its file changes (otherwise the application only
    /// says that the part is out of date).
    #[serde(default)]
    pub auto_update_links: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AssemblyError {
    UnknownInstance(usize),
    UnknownPart(usize),
    UnknownFrame {
        part: String,
        frame: String,
    },
    BadFrame(String),
    /// The joint would close a kinematic loop (or attach an instance that is already placed).
    Loop {
        joint: String,
        instance: String,
    },
    /// The kinematic loop through this joint does not close at the given inputs: the mechanism cannot be
    /// assembled there (out of reach, locked up, or over-constrained). `residual` is the gap in mm
    /// (rotation counted as 100 mm per radian); `free` is how many values the loop solve may change.
    LoopNotClosed {
        joint: String,
        residual: f64,
        free: usize,
    },
    /// The instance is not connected to a grounded one by any joint.
    Disconnected(String),
    OutOfLimits {
        joint: String,
        value: f64,
        low: f64,
        high: f64,
    },
    WrongValues {
        joint: String,
        expected: usize,
        got: usize,
    },
    Expression {
        joint: String,
        message: String,
    },
    /// A part does not evaluate to a solid.
    Part {
        part: String,
        message: String,
    },
    /// A joint between two grounded components (they cannot move against each other).
    BothGrounded {
        joint: String,
    },
    /// A joint between two members of one rigid group (they cannot move against each other).
    Redundant {
        joint: String,
        group: String,
    },
    /// An operation that cannot be done (and why).
    Invalid(String),
    Kernel(String),
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AssemblyError::UnknownInstance(i) => write!(f, "there is no instance {i}"),
            AssemblyError::UnknownPart(i) => write!(f, "there is no part {i}"),
            AssemblyError::UnknownFrame { part, frame } => {
                write!(f, "part '{part}' has no frame called '{frame}'")
            }
            AssemblyError::BadFrame(n) => write!(f, "the frame {n} is degenerate"),
            AssemblyError::Loop { joint, instance } => write!(
                f,
                "joint '{joint}' would close a loop: '{instance}' is already placed by another chain of joints, and assemblies are solved as a tree"
            ),
            AssemblyError::LoopNotClosed {
                joint,
                residual,
                free,
            } => {
                if *free == 0 {
                    write!(
                        f,
                        "joint '{joint}' closes a kinematic loop that does not close at these values (gap {residual:.4}); mark a joint of the loop as solved so its value can be found"
                    )
                } else {
                    write!(
                        f,
                        "the kinematic loop through joint '{joint}' does not close at these inputs (gap {residual:.4}): the mechanism cannot reach this position, is locked up, or is over-constrained"
                    )
                }
            }
            AssemblyError::Disconnected(n) => {
                write!(f, "'{n}' is not connected to a grounded component by any joint")
            }
            AssemblyError::OutOfLimits {
                joint,
                value,
                low,
                high,
            } => write!(f, "joint '{joint}': {value} is outside its limits [{low}, {high}]"),
            AssemblyError::WrongValues {
                joint,
                expected,
                got,
            } => write!(f, "joint '{joint}' takes {expected} value(s), {got} given"),
            AssemblyError::Expression { joint, message } => write!(f, "joint '{joint}': {message}"),
            AssemblyError::Part { part, message } => write!(f, "part '{part}': {message}"),
            AssemblyError::BothGrounded { joint } => write!(
                f,
                "joint '{joint}' connects two grounded components, which cannot move against each other; release one of them"
            ),
            AssemblyError::Redundant { joint, group } => write!(
                f,
                "joint '{joint}' joins two members of the rigid group '{group}', which cannot move against each other"
            ),
            AssemblyError::Invalid(m) => write!(f, "{m}"),
            AssemblyError::Kernel(m) => write!(f, "{m}"),
        }
    }
}
impl std::error::Error for AssemblyError {}

fn world_of(f: &Frame3) -> Transform3 {
    Transform3::frame_to_world(f)
}

/// One component at the bottom of the tree: a part with its placement in the top-level world.
#[derive(Debug, Clone)]
pub struct Leaf {
    /// `Outer/Inner` for components of sub-assemblies.
    pub name: String,
    pub part: Part,
    pub transform: Transform3,
    /// Index of the instance of the top-level assembly it belongs to.
    pub top: usize,
    pub hidden: bool,
}

impl Assembly {
    pub fn new(name: &str) -> Assembly {
        Assembly {
            name: name.into(),
            parts: Vec::new(),
            instances: Vec::new(),
            ground: 0,
            joints: Vec::new(),
            params: Vec::new(),
            grounds: None,
            groups: Vec::new(),
            auto_update_links: false,
        }
    }

    pub fn add_part(&mut self, part: Part) -> usize {
        self.parts.push(part);
        self.parts.len() - 1
    }

    /// As [`add_part`](Self::add_part), renaming the part if another has its name.
    pub fn add_part_unique(&mut self, mut part: Part) -> usize {
        let base = part.name.clone();
        let mut k = 2;
        while self.parts.iter().any(|p| p.name == part.name) {
            part.name = format!("{base} {k}");
            k += 1;
        }
        self.add_part(part)
    }

    pub fn add_instance(&mut self, name: &str, part: usize) -> usize {
        self.instances.push(Instance {
            name: name.into(),
            part,
            grounded: false,
            at: Placement::default(),
            hidden: false,
        });
        self.instances.len() - 1
    }

    /// Add a joint. One that closes a kinematic loop gets its values solved (see
    /// [`mark_loop_closures`](Self::mark_loop_closures)).
    pub fn add_joint(&mut self, joint: Joint) -> usize {
        self.joints.push(joint);
        self.mark_loop_closures();
        self.joints.len() - 1
    }

    /// Add a joint, and take it back (with the loop marks it made) if the assembly can no longer be
    /// solved. Returns the new joint's index, or why it was refused.
    pub fn try_add_joint(&mut self, joint: Joint) -> Result<usize, AssemblyError> {
        let flags: Vec<bool> = self.joints.iter().map(|j| j.solved).collect();
        let k = self.add_joint(joint);
        match self.solve() {
            Ok(_) => Ok(k),
            Err(e) => {
                self.joints.pop();
                for (j, f) in self.joints.iter_mut().zip(flags) {
                    j.solved = f;
                }
                Err(e)
            }
        }
    }

    /// Make the loops of the assembly solvable. For every joint that closes a kinematic loop, the joints
    /// of the loop are marked `solved` (their values are found so that the loop closes) except one **input**,
    /// the one next to the top of the loop, which stays driven and moves the mechanism. A joint that is
    /// already driven and is the only one left keeps that role. Returns whether anything was marked. Does
    /// nothing if the tree cannot be walked (the caller finds out when it solves).
    pub fn mark_loop_closures(&mut self) -> bool {
        let Ok(scope) = self.scope() else {
            return false;
        };
        let Ok(pass) = self.place_tree(&scope, &[]) else {
            return false;
        };
        // the joints from instance `x` up to the instance that was placed first: (joint, instance above)
        let chain = |mut x: usize| -> Vec<(usize, usize)> {
            let mut out = Vec::new();
            while let Some((j, up)) = pass.via[x] {
                out.push((j, up));
                x = up;
                if out.len() > self.instances.len() {
                    break;
                }
            }
            out
        };
        let mut any = false;
        for &ci in &pass.closing {
            let (p, c) = (
                self.joints[ci].parent.instance,
                self.joints[ci].child.instance,
            );
            let (mut up_p, mut up_c) = (chain(p), chain(c));
            // cut both chains where they meet
            let anc_p: Vec<usize> = std::iter::once(p)
                .chain(up_p.iter().map(|(_, i)| *i))
                .collect();
            let meet = std::iter::once(c)
                .chain(up_c.iter().map(|(_, i)| *i))
                .find(|i| anc_p.contains(i));
            if let Some(m) = meet {
                let cut = |v: &mut Vec<(usize, usize)>, start: usize| {
                    let mut at = start;
                    let mut keep = 0;
                    while at != m && keep < v.len() {
                        at = v[keep].1;
                        keep += 1;
                    }
                    v.truncate(keep);
                };
                cut(&mut up_p, p);
                cut(&mut up_c, c);
            }
            // the loop's joints, the one next to the top first
            let mut ring: Vec<usize> = up_p.iter().rev().map(|(j, _)| *j).collect();
            ring.extend(up_c.iter().rev().map(|(j, _)| *j));
            let movable = |j: &usize| self.joints[*j].kind.arity() > 0;
            let input = ring
                .iter()
                .copied()
                .filter(movable)
                .find(|j| !self.joints[*j].solved);
            for j in ring.into_iter().chain(std::iter::once(ci)) {
                if Some(j) == input {
                    continue;
                }
                let jt = &mut self.joints[j];
                if !jt.solved && jt.kind.arity() > 0 {
                    jt.solved = true;
                    any = true;
                }
            }
        }
        any
    }

    // ------------------------------------------------------------------------------ grounding

    /// The grounded instances.
    pub fn grounded(&self) -> Vec<usize> {
        match &self.grounds {
            Some(g) => g
                .iter()
                .copied()
                .filter(|i| *i < self.instances.len())
                .collect(),
            None => {
                let mut v: Vec<usize> = Vec::new();
                if !self.instances.is_empty() {
                    v.push(self.ground.min(self.instances.len() - 1));
                }
                for (i, inst) in self.instances.iter().enumerate() {
                    if inst.grounded && !v.contains(&i) {
                        v.push(i);
                    }
                }
                v
            }
        }
    }

    pub fn is_grounded(&self, i: usize) -> bool {
        self.grounded().contains(&i)
    }

    /// Store every instance's current position as its own placement, so that whatever stops placing it
    /// (a deleted joint, a released ground) leaves it where it is. Does nothing if the assembly cannot be
    /// solved.
    pub fn snapshot_placements(&mut self) {
        if let Ok(placed) = self.solve() {
            for (inst, t) in self.instances.iter_mut().zip(placed) {
                inst.at = Placement::from_transform(&t);
            }
        }
    }

    /// Delete joint `j`; the components it placed stay where they are.
    pub fn remove_joint(&mut self, j: usize) {
        if j < self.joints.len() {
            self.snapshot_placements();
            self.joints.remove(j);
        }
    }

    /// Delete rigid group `g`; its members stay where they are.
    pub fn remove_group(&mut self, g: usize) {
        if g < self.groups.len() {
            self.snapshot_placements();
            self.groups.remove(g);
        }
    }

    /// Ground or release an instance. It stays where it is.
    pub fn set_grounded(&mut self, i: usize, on: bool) {
        self.snapshot_placements();
        let mut g = self.grounded();
        g.retain(|x| *x != i);
        if on {
            g.push(i);
        }
        for (k, inst) in self.instances.iter_mut().enumerate() {
            inst.grounded = g.contains(&k);
        }
        if let Some(first) = g.first() {
            self.ground = *first;
        }
        self.grounds = Some(g);
    }

    // ------------------------------------------------------------------------------- frames

    fn part_of(&self, instance: usize) -> Result<&Part, AssemblyError> {
        let inst = self
            .instances
            .get(instance)
            .ok_or(AssemblyError::UnknownInstance(instance))?;
        self.parts
            .get(inst.part)
            .ok_or(AssemblyError::UnknownPart(inst.part))
    }

    /// A frame of an instance's part in the part's own coordinates (for a sub-assembly, an `Inner.frame`
    /// is carried through the inner placements).
    fn frame_in_part(part: &Part, name: &str) -> Result<Frame3, AssemblyError> {
        if let Some(f) = part.frames.iter().find(|f| f.name == name) {
            return f.to_frame3();
        }
        if let (Some(sub), Some((inner, rest))) = (&part.sub, name.split_once('.')) {
            let k = sub
                .instances
                .iter()
                .position(|i| i.name == inner)
                .ok_or_else(|| AssemblyError::UnknownFrame {
                    part: part.name.clone(),
                    frame: name.to_string(),
                })?;
            let placed = sub.solve()?[k];
            let f = Self::frame_in_part(sub.part_of(k)?, rest)?;
            let world = placed.then(&Transform3::identity());
            return Ok(Frame3 {
                origin: world.apply_point(&f.origin),
                x: world.apply_vector(&f.x),
                y: world.apply_vector(&f.y),
                z: world.apply_vector(&f.z),
            });
        }
        Err(AssemblyError::UnknownFrame {
            part: part.name.clone(),
            frame: name.to_string(),
        })
    }

    fn frame_of(&self, end: &JointEnd) -> Result<Frame3, AssemblyError> {
        Self::frame_in_part(self.part_of(end.instance)?, &end.frame)
    }

    /// Values of the named parameters.
    pub fn scope(&self) -> Result<HashMap<String, Quantity>, AssemblyError> {
        let mut scope: HashMap<String, Quantity> = HashMap::new();
        for (n, p) in &self.params {
            let q = p.eval(&scope).map_err(|e| AssemblyError::Expression {
                joint: format!("parameter {n}"),
                message: e.to_string(),
            })?;
            scope.insert(n.clone(), q);
        }
        Ok(scope)
    }

    /// The evaluated values of a joint (radians / mm), limits enforced.
    pub fn joint_values(
        &self,
        j: &Joint,
        scope: &HashMap<String, Quantity>,
    ) -> Result<Vec<f64>, AssemblyError> {
        self.joint_values_with(j, scope, true)
    }

    /// The evaluated values of a joint, with the limits enforced or not (a loop solve starts from values
    /// that may lie outside them and clamps).
    pub(crate) fn joint_values_with(
        &self,
        j: &Joint,
        scope: &HashMap<String, Quantity>,
        check_limits: bool,
    ) -> Result<Vec<f64>, AssemblyError> {
        if j.values.len() != j.kind.arity() {
            return Err(AssemblyError::WrongValues {
                joint: j.name.clone(),
                expected: j.kind.arity(),
                got: j.values.len(),
            });
        }
        let mut out = Vec::new();
        for (i, (p, dim)) in j.values.iter().zip(j.kind.dims()).enumerate() {
            let q = p.eval(scope).map_err(|e| AssemblyError::Expression {
                joint: j.name.clone(),
                message: e.to_string(),
            })?;
            let v = q.expect(*dim).map_err(|e| AssemblyError::Expression {
                joint: j.name.clone(),
                message: e.to_string(),
            })?;
            if let (true, Some(Some((lo, hi)))) = (check_limits, j.limits.get(i)) {
                if v < *lo - 1e-12 || v > *hi + 1e-12 {
                    return Err(AssemblyError::OutOfLimits {
                        joint: j.name.clone(),
                        value: v,
                        low: *lo,
                        high: *hi,
                    });
                }
            }
            out.push(v);
        }
        Ok(out)
    }

    /// The motion of a joint (child frame → parent frame) for given values, with its flip and offset.
    pub fn joint_transform(j: &Joint, vals: &[f64]) -> Transform3 {
        let mut pre = Transform3::identity();
        if j.flip {
            pre = Transform3::rotation(Vector3::x(), std::f64::consts::PI);
        }
        pre =
            pre.then(&Transform3::rotation(Vector3::z(), j.angle))
                .then(&Transform3::translation(Vector3::new(
                    j.offset[0],
                    j.offset[1],
                    j.offset[2],
                )));
        let motion = match j.kind {
            JointKind::Rigid => Transform3::identity(),
            JointKind::Revolute => Transform3::rotation(Vector3::z(), vals[0]),
            JointKind::Slider => Transform3::translation(Vector3::z() * vals[0]),
            JointKind::Cylindrical => Transform3::rotation(Vector3::z(), vals[0])
                .then(&Transform3::translation(Vector3::z() * vals[1])),
            JointKind::Planar => Transform3::rotation(Vector3::z(), vals[2]).then(
                &Transform3::translation(Vector3::new(vals[0], vals[1], 0.0)),
            ),
            JointKind::Ball => Transform3::rotation(Vector3::x(), vals[0])
                .then(&Transform3::rotation(Vector3::y(), vals[1]))
                .then(&Transform3::rotation(Vector3::z(), vals[2])),
        };
        pre.then(&motion)
    }

    // ------------------------------------------------------------------------------- solving

    /// The group (if any) an instance belongs to.
    pub fn group_of(&self, instance: usize) -> Option<usize> {
        self.groups
            .iter()
            .position(|g| g.members.contains(&instance))
    }

    /// The placement of every instance (part coordinates → world).
    pub fn solve(&self) -> Result<Vec<Transform3>, AssemblyError> {
        self.solve_traced().map(|(t, _)| t)
    }

    /// As [`solve`](Self::solve), also saying for each instance which joint placed it and from which
    /// instance (`None` for grounded, floating and rigid-group-placed instances).
    pub fn solve_traced(
        &self,
    ) -> Result<(Vec<Transform3>, Vec<Option<(usize, usize)>>), AssemblyError> {
        let pass = self.solve_pass()?;
        Ok((pass.placed, pass.via))
    }

    /// The joint that moves instance `i` when it is dragged: the nearest driven joint towards the grounded
    /// components, looking through rigid joints (which cannot move), joints solved by a loop (the pointer
    /// drives the inputs, not them) and across rigid groups. When only solved joints lie that way (the
    /// rocker of a four-bar), the driven joint that moves the instance is found by trying them. `None` when
    /// the instance is grounded or held rigidly to a grounded one, or floats (then it moves freely).
    pub fn drive_joint(&self, i: usize) -> Option<usize> {
        let (_, via) = self.solve_traced().ok()?;
        let mut x = i;
        let mut seen: Vec<usize> = Vec::new();
        let mut through_solved = false;
        'walk: loop {
            if seen.contains(&x) {
                break;
            }
            seen.push(x);
            let mates: Vec<usize> = match self.group_of(x) {
                Some(g) => self.groups[g].members.clone(),
                None => vec![x],
            };
            for m in mates {
                if let Some((j, from)) = via.get(m).copied().flatten() {
                    let jt = &self.joints[j];
                    if jt.kind.arity() > 0 && !jt.solved {
                        return Some(j);
                    }
                    through_solved |= jt.solved && jt.kind.arity() > 0;
                    x = from;
                    continue 'walk;
                }
            }
            break;
        }
        if through_solved {
            self.driven_joint_moving(i)
        } else {
            None
        }
    }

    /// The first driven joint whose value, nudged, moves instance `i` (loops are re-solved each time).
    fn driven_joint_moving(&self, i: usize) -> Option<usize> {
        let base = self.solve().ok()?.get(i)?.to_matrix();
        let scope = self.scope().ok()?;
        let mut probe = self.clone();
        for (ji, j) in self.joints.iter().enumerate() {
            if j.solved || j.kind.arity() == 0 {
                continue;
            }
            let vals = self.joint_values_with(j, &scope, false).ok()?;
            for (c, dim) in j.kind.dims().iter().enumerate() {
                let eps = if *dim == Dim::ANGLE { 1e-4 } else { 1e-3 };
                for sign in [1.0, -1.0] {
                    let mut v = vals.clone();
                    v[c] += sign * eps;
                    if let Some(Some((lo, hi))) = j.limits.get(c) {
                        v[c] = v[c].clamp(*lo, *hi);
                    }
                    probe.joints = self.joints.clone();
                    probe.joints[ji].values = loops::lit_values(j.kind, &v);
                    if let Ok(p) = probe.settle() {
                        if (p[i].to_matrix() - base).norm() > 1e-9 {
                            return Some(ji);
                        }
                    }
                }
            }
        }
        None
    }

    /// Whether instance `i` moves as a free body: not grounded, no joint above it, not held by a ground.
    /// For a member of a floating rigid group the answer is for the group.
    pub fn is_free(&self, i: usize) -> bool {
        let Ok((_, via)) = self.solve_traced() else {
            return false;
        };
        let mates: Vec<usize> = match self.group_of(i) {
            Some(g) => self.groups[g].members.clone(),
            None => vec![i],
        };
        let grounded = self.grounded();
        mates
            .iter()
            .all(|m| !grounded.contains(m) && via.get(*m).copied().flatten().is_none())
            && mates.iter().all(|m| {
                !self
                    .joints
                    .iter()
                    .any(|j| j.parent.instance == *m || j.child.instance == *m)
            })
    }

    /// The instance whose own placement (`at`) positions instance `i` when it floats: `i` itself, or the
    /// first member of its rigid group.
    pub fn float_anchor(&self, i: usize) -> usize {
        self.group_of(i)
            .and_then(|g| self.groups[g].members.first().copied())
            .unwrap_or(i)
    }

    // ------------------------------------------------------------------------------- editing

    /// `base`, or `base 2`, `base 3`, … when the name is taken.
    pub fn unique_instance_name(&self, base: &str) -> String {
        if !self.instances.iter().any(|i| i.name == base) {
            return base.to_string();
        }
        (2..)
            .map(|k| format!("{base} {k}"))
            .find(|n| !self.instances.iter().any(|i| &i.name == n))
            .expect("a free name")
    }

    /// Take instance `i` out with the joints that touch it; later instances move up. A rigid group that
    /// loses a member keeps the others (and disappears when fewer than two are left).
    pub fn remove_instance(&mut self, i: usize) {
        if i >= self.instances.len() {
            return;
        }
        self.snapshot_placements();
        self.joints
            .retain(|j| j.parent.instance != i && j.child.instance != i);
        let fix = |x: usize| if x > i { x - 1 } else { x };
        for j in &mut self.joints {
            j.parent.instance = fix(j.parent.instance);
            j.child.instance = fix(j.child.instance);
        }
        for g in &mut self.groups {
            if let Some(k) = g.members.iter().position(|m| *m == i) {
                g.members.remove(k);
                g.rel.remove(k);
                // the first member is the anchor: re-express the others against the new anchor
                if k == 0 && !g.members.is_empty() {
                    let a = g.rel[0].to_transform();
                    for r in g.rel.iter_mut() {
                        *r = Placement::from_transform(&r.to_transform().then(&a.inverse()));
                    }
                }
            }
            for m in &mut g.members {
                *m = fix(*m);
            }
        }
        self.groups.retain(|g| g.members.len() >= 2);
        let grounded = self.grounded();
        self.instances.remove(i);
        let mut g: Vec<usize> = grounded.into_iter().filter(|x| *x != i).map(fix).collect();
        g.sort_unstable();
        for (k, inst) in self.instances.iter_mut().enumerate() {
            inst.grounded = g.contains(&k);
        }
        self.ground = g.first().copied().unwrap_or(0);
        self.grounds = Some(g);
    }

    /// Drop parts no instance uses (after deleting components) and renumber the instances' part indices.
    /// Returns how many parts went. Indices of parts held elsewhere become stale: do this only when none is.
    pub fn prune_unused_parts(&mut self) -> usize {
        let mut gone = 0;
        let mut p = 0;
        while p < self.parts.len() {
            if self.instances.iter().any(|i| i.part == p) {
                p += 1;
                continue;
            }
            self.parts.remove(p);
            for i in &mut self.instances {
                if i.part > p {
                    i.part -= 1;
                }
            }
            gone += 1;
        }
        gone
    }

    /// Tie `members` (two or more instances) into a rigid group at their current relative positions.
    pub fn make_rigid_group(
        &mut self,
        name: &str,
        members: &[usize],
    ) -> Result<usize, AssemblyError> {
        let mut members: Vec<usize> = members.to_vec();
        members.sort_unstable();
        members.dedup();
        if members.len() < 2 {
            return Err(AssemblyError::Invalid(
                "a rigid group needs at least two components".into(),
            ));
        }
        for m in &members {
            if *m >= self.instances.len() {
                return Err(AssemblyError::UnknownInstance(*m));
            }
            if let Some(g) = self.group_of(*m) {
                return Err(AssemblyError::Invalid(format!(
                    "'{}' already belongs to the rigid group '{}'",
                    self.instances[*m].name, self.groups[g].name
                )));
            }
        }
        let placed = self.solve()?;
        let anchor = placed[members[0]];
        let rel = members
            .iter()
            .map(|m| Placement::from_transform(&placed[*m].then(&anchor.inverse())))
            .collect();
        self.groups.push(RigidGroup {
            name: name.into(),
            members,
            rel,
        });
        // the group must still solve (it would not if two members were also joined to each other)
        if let Err(e) = self.solve() {
            self.groups.pop();
            return Err(e);
        }
        Ok(self.groups.len() - 1)
    }

    /// Make a joint of `kind` between `parent` and `child` from where they are now: the joint's origin is the
    /// world point `origin` and its z axis the world direction `axis`, and zero motion keeps everything
    /// where it is. The two frames are added to the parts.
    pub fn as_built_joint(
        &mut self,
        parent: usize,
        child: usize,
        kind: JointKind,
        origin: [f64; 3],
        axis: [f64; 3],
        name: &str,
    ) -> Result<usize, AssemblyError> {
        if parent == child {
            return Err(AssemblyError::Invalid(
                "a joint connects two different components".into(),
            ));
        }
        let placed = self.solve()?;
        let (pp, pc) = (
            placed
                .get(parent)
                .ok_or(AssemblyError::UnknownInstance(parent))?,
            placed
                .get(child)
                .ok_or(AssemblyError::UnknownInstance(child))?,
        );
        let z = Vector3::new(axis[0], axis[1], axis[2])
            .try_normalize(1e-12)
            .ok_or_else(|| AssemblyError::Invalid("the joint axis has no direction".into()))?;
        let helper = if z.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let x = (helper - z * helper.dot(&z)).normalize();
        let world = Frame3 {
            origin: Point3::new(origin[0], origin[1], origin[2]),
            x,
            y: z.cross(&x),
            z,
        };
        let local = |t: &Transform3| {
            let inv = t.inverse();
            Frame3 {
                origin: inv.apply_point(&world.origin),
                x: inv.apply_vector(&world.x),
                y: inv.apply_vector(&world.y),
                z: inv.apply_vector(&world.z),
            }
        };
        let (fp, fc) = (local(pp), local(pc));
        let tag = format!("{name} origin");
        let pname = self.add_frame_to_instance(parent, &format!("{tag} (parent)"), &fp)?;
        let cname = self.add_frame_to_instance(child, &format!("{tag} (child)"), &fc)?;
        let mut j = Joint::new(
            name,
            kind,
            JointEnd {
                instance: parent,
                frame: pname,
            },
            JointEnd {
                instance: child,
                frame: cname,
            },
        );
        j.as_built = true;
        let flags: Vec<bool> = self.joints.iter().map(|j| j.solved).collect();
        let undo = |a: &mut Assembly| {
            a.joints.pop();
            for (j, f) in a.joints.iter_mut().zip(&flags) {
                j.solved = *f;
            }
        };
        self.joints.push(j);
        self.mark_loop_closures();
        // the child must still be placed where it was: the joint it gets is the one that places it
        match self.solve() {
            Ok(after) => {
                let moved = (after[child].to_matrix() - pc.to_matrix()).norm();
                if moved > 1e-6 {
                    undo(self);
                    return Err(AssemblyError::Invalid(
                        "the as-built joint would move the component".into(),
                    ));
                }
            }
            Err(e) => {
                undo(self);
                return Err(e);
            }
        }
        Ok(self.joints.len() - 1)
    }

    /// Add a connection frame (given in the part's coordinates) to the part of `instance` under a name made
    /// unique; returns the name.
    pub fn add_frame_to_instance(
        &mut self,
        instance: usize,
        name: &str,
        frame: &Frame3,
    ) -> Result<String, AssemblyError> {
        let part = self
            .instances
            .get(instance)
            .ok_or(AssemblyError::UnknownInstance(instance))?
            .part;
        let p = self
            .parts
            .get_mut(part)
            .ok_or(AssemblyError::UnknownPart(part))?;
        let mut unique = name.to_string();
        let mut k = 2;
        while p.frames.iter().any(|f| f.name == unique) {
            unique = format!("{name} {k}");
            k += 1;
        }
        p.frames.push(Frame::from_frame3(&unique, frame));
        Ok(unique)
    }

    /// Move `instances` (and the joints among them, and rigid groups wholly inside) into a new sub-assembly
    /// that becomes one component. Joints that connected them to the rest now reach the new component by
    /// `Inner.frame`. The sub-assembly keeps every component where it is.
    pub fn group_into_subassembly(
        &mut self,
        instances: &[usize],
        name: &str,
    ) -> Result<usize, AssemblyError> {
        let mut chosen: Vec<usize> = instances.to_vec();
        chosen.sort_unstable();
        chosen.dedup();
        if chosen.len() < 2 {
            return Err(AssemblyError::Invalid(
                "a sub-assembly needs at least two components".into(),
            ));
        }
        for c in &chosen {
            if *c >= self.instances.len() {
                return Err(AssemblyError::UnknownInstance(*c));
            }
        }
        for g in &self.groups {
            let inside = g.members.iter().filter(|m| chosen.contains(m)).count();
            if inside != 0 && inside != g.members.len() {
                return Err(AssemblyError::Invalid(format!(
                    "the rigid group '{}' is only partly selected",
                    g.name
                )));
            }
        }
        let placed = self.solve()?;
        let was_grounded: Vec<usize> = self
            .grounded()
            .into_iter()
            .filter(|g| chosen.contains(g))
            .collect();
        // the new assembly: its parts, instances (each kept at its current placement), joints and groups
        let mut sub = Assembly::new(name);
        let mut part_map: HashMap<usize, usize> = HashMap::new();
        let mut inst_map: HashMap<usize, usize> = HashMap::new();
        for c in &chosen {
            let old = &self.instances[*c];
            let np = *part_map.entry(old.part).or_insert_with(|| {
                sub.parts.push(self.parts[old.part].clone());
                sub.parts.len() - 1
            });
            let ni = sub.add_instance(&old.name, np);
            sub.instances[ni].at = Placement::from_transform(&placed[*c]);
            sub.instances[ni].hidden = old.hidden;
            inst_map.insert(*c, ni);
        }
        let mut crossing: Vec<usize> = Vec::new();
        let mut kept_joints: Vec<Joint> = Vec::new();
        for (ji, j) in self.joints.iter().enumerate() {
            let (a, b) = (
                inst_map.get(&j.parent.instance),
                inst_map.get(&j.child.instance),
            );
            match (a, b) {
                (Some(a), Some(b)) => {
                    let mut nj = j.clone();
                    nj.parent.instance = *a;
                    nj.child.instance = *b;
                    sub.joints.push(nj);
                }
                (None, None) => kept_joints.push(j.clone()),
                _ => {
                    crossing.push(kept_joints.len());
                    kept_joints.push(j.clone());
                    let _ = ji;
                }
            }
        }
        for g in &self.groups {
            if g.members.iter().all(|m| inst_map.contains_key(m)) {
                sub.groups.push(RigidGroup {
                    name: g.name.clone(),
                    members: g.members.iter().map(|m| inst_map[m]).collect(),
                    rel: g.rel.clone(),
                });
            }
        }
        // the inner anchor: what was grounded, else the first component that nothing inside places
        let mut inner_grounds: Vec<usize> = was_grounded.iter().map(|g| inst_map[g]).collect();
        if inner_grounds.is_empty() {
            let jointed_child: Vec<usize> = sub.joints.iter().map(|j| j.child.instance).collect();
            inner_grounds.push(
                (0..sub.instances.len())
                    .find(|i| !jointed_child.contains(i))
                    .unwrap_or(0),
            );
        }
        sub.grounds = Some(inner_grounds.clone());
        sub.ground = inner_grounds[0];
        for (i, inst) in sub.instances.iter_mut().enumerate() {
            inst.grounded = inner_grounds.contains(&i);
        }
        sub.solve()?;
        // the outer side
        let sub_name = self.unique_instance_name(name);
        let mut part = Part::new(name, Document::new(), Vec::new());
        part.sub = Some(Box::new(sub));
        let inner_names: Vec<String> = chosen
            .iter()
            .map(|c| self.instances[*c].name.clone())
            .collect();
        // joints that crossed the boundary keep their frames, now under `Inner.frame` on the new component
        let new_index_of_sub = self.instances.len() - chosen.len();
        let mut outer_joints = kept_joints;
        for ci in crossing {
            let j = &mut outer_joints[ci];
            for e in [&mut j.parent, &mut j.child] {
                if let Some(k) = chosen.iter().position(|c| *c == e.instance) {
                    e.frame = format!("{}.{}", inner_names[k], e.frame);
                    e.instance = usize::MAX; // the new component, fixed up below
                }
            }
        }
        // remove the chosen instances and renumber
        let remap = |x: usize| -> usize {
            if x == usize::MAX {
                new_index_of_sub
            } else {
                x - chosen.iter().filter(|c| **c < x).count()
            }
        };
        let old_groups: Vec<RigidGroup> = self
            .groups
            .iter()
            .filter(|g| !g.members.iter().all(|m| chosen.contains(m)))
            .cloned()
            .collect();
        let old_grounded: Vec<usize> = self
            .grounded()
            .into_iter()
            .filter(|g| !chosen.contains(g))
            .collect();
        let remaining: Vec<Instance> = self
            .instances
            .iter()
            .enumerate()
            .filter(|(i, _)| !chosen.contains(i))
            .map(|(_, inst)| inst.clone())
            .collect();
        self.instances = remaining;
        self.parts.push(part);
        let part_index = self.parts.len() - 1;
        self.instances.push(Instance {
            name: sub_name,
            part: part_index,
            grounded: !was_grounded.is_empty(),
            at: Placement::default(),
            hidden: false,
        });
        for j in &mut outer_joints {
            j.parent.instance = remap(j.parent.instance);
            j.child.instance = remap(j.child.instance);
        }
        self.joints = outer_joints;
        self.groups = old_groups
            .into_iter()
            .map(|mut g| {
                g.members = g.members.iter().map(|m| remap(*m)).collect();
                g
            })
            .collect();
        let mut grounds: Vec<usize> = old_grounded.iter().map(|g| remap(*g)).collect();
        if !was_grounded.is_empty() {
            grounds.push(new_index_of_sub);
        }
        grounds.sort_unstable();
        for (k, inst) in self.instances.iter_mut().enumerate() {
            inst.grounded = grounds.contains(&k);
        }
        self.ground = grounds.first().copied().unwrap_or(0);
        self.grounds = Some(grounds);
        // the new component is placed by its joints; one that nothing joins stays where it is (identity:
        // the inner placements already carry the world positions)
        self.solve()?;
        Ok(new_index_of_sub)
    }

    // ------------------------------------------------------------------------------- leaves

    /// Every component at the bottom of the tree with its world placement (sub-assemblies flattened).
    pub fn leaves(&self) -> Result<Vec<Leaf>, AssemblyError> {
        let placed = self.solve()?;
        let mut out = Vec::new();
        for (i, inst) in self.instances.iter().enumerate() {
            let part = self
                .parts
                .get(inst.part)
                .ok_or(AssemblyError::UnknownPart(inst.part))?;
            self.collect_leaves(part, &inst.name, placed[i], i, inst.hidden, &mut out)?;
        }
        Ok(out)
    }

    fn collect_leaves(
        &self,
        part: &Part,
        name: &str,
        transform: Transform3,
        top: usize,
        hidden: bool,
        out: &mut Vec<Leaf>,
    ) -> Result<(), AssemblyError> {
        match &part.sub {
            None => out.push(Leaf {
                name: name.to_string(),
                part: part.clone(),
                transform,
                top,
                hidden,
            }),
            Some(sub) => {
                let inner = sub.solve()?;
                for (k, inst) in sub.instances.iter().enumerate() {
                    let p = sub
                        .parts
                        .get(inst.part)
                        .ok_or(AssemblyError::UnknownPart(inst.part))?;
                    sub.collect_leaves(
                        p,
                        &format!("{name}/{}", inst.name),
                        inner[k].then(&transform),
                        top,
                        hidden || inst.hidden,
                        out,
                    )?;
                }
            }
        }
        Ok(())
    }

    /// Evaluate every part and place the bodies. `engine` caches the parts between calls, so moving a
    /// joint re-evaluates nothing. Hidden components are left out. A part with several solid bodies places
    /// all of them (named `Component/Body`); the material of a body is the part's, else the body's own.
    pub fn pose(&self, engine: &mut Engine) -> Result<Pose, AssemblyError> {
        /// One solid body of an evaluated part.
        struct Solid {
            brep: std::sync::Arc<BRep>,
            body: BodyId,
            /// The body's own name, when the part has more than one.
            name: Option<String>,
            material: Option<String>,
            size: [f64; 3],
            volume: f64,
        }
        let leaves = self.leaves()?;
        let mut solids: HashMap<String, Vec<Solid>> = HashMap::new();
        let mut rep = BRep::new();
        let mut bodies = Vec::new();
        let mut maps = Vec::new();
        let mut names = Vec::new();
        let mut placements = Vec::new();
        let mut tops = Vec::new();
        let mut parts = Vec::new();
        let mut materials = Vec::new();
        let mut sizes = Vec::new();
        let mut volumes = Vec::new();
        for leaf in &leaves {
            if leaf.hidden {
                continue;
            }
            let key = format!("{}#{:x}", leaf.part.name, leaf.part.document.content_hash());
            if let std::collections::hash_map::Entry::Vacant(slot) = solids.entry(key.clone()) {
                let ev = engine.evaluate(&leaf.part.document);
                if let Some((id, e)) = ev.failures().first() {
                    return Err(AssemblyError::Part {
                        part: leaf.part.name.clone(),
                        message: format!("'{}' fails: {e}", leaf.part.document.name_of(*id)),
                    });
                }
                let doc = &leaf.part.document;
                let heads = doc.heads();
                let mut found: Vec<Solid> = Vec::new();
                for (rank, head) in heads.iter().enumerate() {
                    let Some(out) = ev.output(*head).filter(|o| !o.is_sheet()) else {
                        continue;
                    };
                    let (Some(brep), Some(body)) = (out.brep().cloned(), out.body()) else {
                        continue;
                    };
                    let root = doc.body_root(*head);
                    let ap = doc.appearance_of(root);
                    let bb = brep.body_bbox(body);
                    found.push(Solid {
                        name: Some(
                            ap.and_then(|a| a.name.clone())
                                .unwrap_or_else(|| format!("Body{}", rank + 1)),
                        ),
                        material: ap.and_then(|a| a.material.clone()),
                        size: [
                            bb.max.x - bb.min.x,
                            bb.max.y - bb.min.y,
                            bb.max.z - bb.min.z,
                        ],
                        volume: brep.volume(body).unwrap_or(0.0),
                        brep,
                        body,
                    });
                }
                if found.is_empty() {
                    return Err(AssemblyError::Part {
                        part: leaf.part.name.clone(),
                        message: "it has no solid".into(),
                    });
                }
                // a part of one body keeps the name of its component
                if found.len() == 1 {
                    found[0].name = None;
                }
                slot.insert(found);
            }
            for solid in &solids[&key] {
                let (b, map) = rep.copy_body_from(
                    &solid.brep,
                    solid.body,
                    &CopyOptions {
                        transform: Some(leaf.transform),
                        flip: false,
                        ..Default::default()
                    },
                );
                bodies.push(b);
                maps.push(map);
                names.push(match &solid.name {
                    Some(n) => format!("{}/{n}", leaf.name),
                    None => leaf.name.clone(),
                });
                placements.push(leaf.transform);
                tops.push(leaf.top);
                parts.push(leaf.part.name.clone());
                materials.push(
                    leaf.part
                        .material
                        .clone()
                        .or_else(|| solid.material.clone()),
                );
                sizes.push(solid.size);
                volumes.push(solid.volume);
            }
        }
        Ok(Pose {
            rep,
            bodies,
            placements,
            names,
            maps,
            tops,
            parts,
            materials,
            sizes,
            volumes,
            name: self.name.clone(),
        })
    }

    // ------------------------------------------------------------------------------ storage

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        ciborium::into_writer(self, &mut out).expect("assembly serialises");
        out
    }

    pub fn from_bytes(b: &[u8]) -> Result<Assembly, String> {
        ciborium::from_reader(b).map_err(|e| e.to_string())
    }

    /// Store the assembly in a `.kwf` file (entry `assembly.cbor`).
    pub fn attach(&self, file: &mut KwfFile) {
        file.extra_entries
            .insert("assembly.cbor".into(), self.to_bytes());
    }

    pub fn from_file(file: &KwfFile) -> Option<Result<Assembly, String>> {
        file.extra_entries
            .get("assembly.cbor")
            .map(|b| Assembly::from_bytes(b))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Interference {
    pub a: String,
    pub b: String,
    /// Volume the two bodies share (mm³).
    pub volume: f64,
    /// The overlap as a solid of the pose's B-rep (so it can be drawn).
    pub body: Option<BodyId>,
}

/// A line of the bill of materials: a part and how many of it there are.
#[derive(Debug, Clone, PartialEq)]
pub struct BomRow {
    pub part: String,
    pub count: usize,
    /// Volume of one (mm³).
    pub volume: f64,
    pub material: Option<String>,
    /// Mass of one (g) with the density the caller supplied for the material.
    pub mass_g: f64,
    /// Size of its bounding box (mm).
    pub size: [f64; 3],
}

pub struct Pose {
    pub rep: BRep,
    pub bodies: Vec<BodyId>,
    pub placements: Vec<Transform3>,
    pub names: Vec<String>,
    /// Display face / edge / vertex → the part's own, for every body (same order as `bodies`).
    pub maps: Vec<CopyMap>,
    /// Which top-level instance each body belongs to.
    pub tops: Vec<usize>,
    /// The part name of each body.
    pub parts: Vec<String>,
    pub materials: Vec<Option<String>>,
    /// Bounding-box size of each body's part (mm), before it was placed.
    pub sizes: Vec<[f64; 3]>,
    /// Volume of each body (mm³).
    pub volumes: Vec<f64>,
    name: String,
}

impl Pose {
    /// The index (in `bodies`) of the body a face of the posed B-rep belongs to.
    pub fn body_of_face(&self, face: kw_brep::FaceId) -> Option<usize> {
        self.bodies
            .iter()
            .position(|b| self.rep.body_faces(*b).contains(&face))
    }

    /// Pairs of components whose bodies overlap by more than `min_volume` (mm³); the overlaps are kept as
    /// solids of the pose's B-rep.
    pub fn interference(&mut self, min_volume: f64) -> Result<Vec<Interference>, AssemblyError> {
        let mut found = Vec::new();
        let n = self.bodies.len();
        for i in 0..n {
            for j in i + 1..n {
                let (a, b) = (self.bodies[i], self.bodies[j]);
                if !self.rep.body_bbox(a).intersects(&self.rep.body_bbox(b)) {
                    continue;
                }
                match boolean(&mut self.rep, a, b, Op::Intersect, &BoolOptions::default()) {
                    Ok(r) => {
                        let v = self.rep.volume(r.body).unwrap_or(0.0);
                        if v > min_volume {
                            found.push(Interference {
                                a: self.names[i].clone(),
                                b: self.names[j].clone(),
                                volume: v,
                                body: Some(r.body),
                            });
                        }
                    }
                    Err(BooleanError::Empty) => {}
                    Err(e) => {
                        return Err(AssemblyError::Kernel(format!(
                            "interference of '{}' and '{}': {e}",
                            self.names[i], self.names[j]
                        )))
                    }
                }
            }
        }
        Ok(found)
    }

    /// Total volume and centre of mass of all instances (unit density).
    pub fn mass(&self) -> Option<(f64, Point3<f64>)> {
        let mut vol = 0.0;
        let mut moment = Vector3::zeros();
        for b in &self.bodies {
            let m = self.rep.mass_properties(*b)?;
            vol += m.volume;
            moment += m.centroid.coords * m.volume;
        }
        (vol > 0.0).then(|| (vol, Point3::from(moment / vol)))
    }

    /// The bill of materials: one row per part with the number of components made from it. `density` gives
    /// kg/m³ for a material name (`None` when the part has no material).
    pub fn bom(&self, density: &dyn Fn(Option<&str>) -> f64) -> Vec<BomRow> {
        let mut rows: Vec<BomRow> = Vec::new();
        for i in 0..self.bodies.len() {
            let name = &self.parts[i];
            if let Some(r) = rows.iter_mut().find(|r| &r.part == name) {
                r.count += 1;
                continue;
            }
            let d = density(self.materials[i].as_deref());
            let vol = self.volumes[i];
            rows.push(BomRow {
                part: name.clone(),
                count: 1,
                volume: vol,
                material: self.materials[i].clone(),
                // mm³ × kg/m³ = 1e-9 m³ × kg/m³ × 1000 g/kg
                mass_g: vol * d * 1e-6,
                size: self.sizes[i],
            });
        }
        rows
    }

    /// The posed bodies as one STEP file.
    pub fn to_step(&self) -> String {
        kw_step::export(
            &self.rep,
            &self.bodies,
            &kw_step::ExportOptions {
                product_name: self.name.clone(),
                ..Default::default()
            },
        )
    }
}
