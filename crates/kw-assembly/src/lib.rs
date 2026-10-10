//! Assemblies: instances of parts placed by joints.
//!
//! * A [`Part`] is a feature [`Document`] plus named connection frames (origin, z axis, x axis in the
//!   part's own coordinates).
//! * A [`Joint`] ties a frame of one instance to a frame of another with a motion: rigid, revolute
//!   (about z), slider (along z), cylindrical or planar. Joint values are parameters or expressions.
//! * The joints form a kinematic tree rooted at the grounded instance; [`Assembly::solve`] walks it and
//!   returns every instance's placement. A joint that would close a loop is refused with an explanation.
//! * [`Assembly::pose`] builds the posed bodies in one B-rep; [`Pose::interference`] finds overlaps with
//!   the kernel's Boolean; [`Pose::to_step`] writes them as one STEP file.
//!
//! An assembly is stored next to the model in a `.kwf` file as the entry `assembly.cbor`.

// Numerical code indexes small fixed-size arrays; `!(x > 0.0)` is deliberate (it also rejects NaN).
#![allow(
    clippy::needless_range_loop,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

use kw_boolean::{boolean, BoolOptions, BooleanError, Op};
use kw_brep::{BRep, BodyId, CopyOptions};
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
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Part {
    pub name: String,
    pub document: Document,
    pub frames: Vec<Frame>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Instance {
    pub name: String,
    pub part: usize,
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
}

impl JointKind {
    fn arity(self) -> usize {
        match self {
            JointKind::Rigid => 0,
            JointKind::Revolute | JointKind::Slider => 1,
            JointKind::Cylindrical => 2,
            JointKind::Planar => 3,
        }
    }
    fn dims(self) -> &'static [Dim] {
        const L: Dim = Dim::LENGTH;
        const A: Dim = Dim::ANGLE;
        match self {
            JointKind::Rigid => &[],
            JointKind::Revolute => &[A],
            JointKind::Slider => &[L],
            JointKind::Cylindrical => &[A, L],
            JointKind::Planar => &[L, L, A],
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
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Assembly {
    pub name: String,
    pub parts: Vec<Part>,
    pub instances: Vec<Instance>,
    pub ground: usize,
    pub joints: Vec<Joint>,
    /// Named values the joint expressions may use, evaluated in order.
    pub params: Vec<(String, Param)>,
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
    /// The instance is not connected to the grounded one by any joint.
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
            AssemblyError::Disconnected(n) => {
                write!(f, "'{n}' is not connected to the grounded instance by any joint")
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
            AssemblyError::Kernel(m) => write!(f, "{m}"),
        }
    }
}
impl std::error::Error for AssemblyError {}

fn world_of(f: &Frame3) -> Transform3 {
    Transform3::frame_to_world(f)
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
        }
    }

    pub fn add_part(&mut self, part: Part) -> usize {
        self.parts.push(part);
        self.parts.len() - 1
    }

    pub fn add_instance(&mut self, name: &str, part: usize) -> usize {
        self.instances.push(Instance {
            name: name.into(),
            part,
        });
        self.instances.len() - 1
    }

    pub fn add_joint(&mut self, joint: Joint) -> usize {
        self.joints.push(joint);
        self.joints.len() - 1
    }

    fn frame_of(&self, end: &JointEnd) -> Result<Frame3, AssemblyError> {
        let inst = self
            .instances
            .get(end.instance)
            .ok_or(AssemblyError::UnknownInstance(end.instance))?;
        let part = self
            .parts
            .get(inst.part)
            .ok_or(AssemblyError::UnknownPart(inst.part))?;
        let fr = part
            .frames
            .iter()
            .find(|f| f.name == end.frame)
            .ok_or_else(|| AssemblyError::UnknownFrame {
                part: part.name.clone(),
                frame: end.frame.clone(),
            })?;
        fr.to_frame3()
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

    fn joint_values(
        &self,
        j: &Joint,
        scope: &HashMap<String, Quantity>,
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
            if let Some(Some((lo, hi))) = j.limits.get(i) {
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

    /// The placement of every instance (part coordinates → world).
    pub fn solve(&self) -> Result<Vec<Transform3>, AssemblyError> {
        if self.instances.is_empty() {
            return Ok(Vec::new());
        }
        if self.ground >= self.instances.len() {
            return Err(AssemblyError::UnknownInstance(self.ground));
        }
        let scope = self.scope()?;
        let mut placed: Vec<Option<Transform3>> = vec![None; self.instances.len()];
        placed[self.ground] = Some(Transform3::identity());
        let mut done = vec![false; self.joints.len()];
        loop {
            let mut progress = false;
            for (ji, j) in self.joints.iter().enumerate() {
                if done[ji] {
                    continue;
                }
                let (p, c) = (j.parent.instance, j.child.instance);
                if p >= self.instances.len() {
                    return Err(AssemblyError::UnknownInstance(p));
                }
                if c >= self.instances.len() {
                    return Err(AssemblyError::UnknownInstance(c));
                }
                // joints may be given in either direction: orient towards the unplaced instance
                let (from, to, flipped) = match (placed[p].is_some(), placed[c].is_some()) {
                    (true, false) => (&j.parent, &j.child, false),
                    (false, true) => (&j.child, &j.parent, true),
                    (true, true) => {
                        return Err(AssemblyError::Loop {
                            joint: j.name.clone(),
                            instance: self.instances[c].name.clone(),
                        })
                    }
                    (false, false) => continue,
                };
                let vals = self.joint_values(j, &scope)?;
                let fa = self.frame_of(from)?;
                let fb = self.frame_of(to)?;
                let mut motion = Transform3::identity();
                match j.kind {
                    JointKind::Rigid => {}
                    JointKind::Revolute => motion = Transform3::rotation(Vector3::z(), vals[0]),
                    JointKind::Slider => motion = Transform3::translation(Vector3::z() * vals[0]),
                    JointKind::Cylindrical => {
                        motion = Transform3::rotation(Vector3::z(), vals[0])
                            .then(&Transform3::translation(Vector3::z() * vals[1]))
                    }
                    JointKind::Planar => {
                        motion = Transform3::rotation(Vector3::z(), vals[2]).then(
                            &Transform3::translation(Vector3::new(vals[0], vals[1], 0.0)),
                        )
                    }
                }
                if flipped {
                    motion = motion.inverse();
                }
                // child coordinates → child frame → motion → parent frame → parent coordinates → world
                let t = world_of(&fb)
                    .inverse()
                    .then(&motion)
                    .then(&world_of(&fa))
                    .then(placed[from.instance].as_ref().unwrap());
                placed[to.instance] = Some(t);
                done[ji] = true;
                progress = true;
            }
            if !progress {
                break;
            }
        }
        for (i, p) in placed.iter().enumerate() {
            if p.is_none() {
                return Err(AssemblyError::Disconnected(self.instances[i].name.clone()));
            }
        }
        for (ji, j) in self.joints.iter().enumerate() {
            if !done[ji] {
                return Err(AssemblyError::Loop {
                    joint: j.name.clone(),
                    instance: self.instances[j.child.instance].name.clone(),
                });
            }
        }
        Ok(placed.into_iter().map(|p| p.unwrap()).collect())
    }

    /// Evaluate every part and place the bodies. `engine` caches the parts between calls, so moving a
    /// joint re-evaluates nothing.
    pub fn pose(&self, engine: &mut Engine) -> Result<Pose, AssemblyError> {
        let placements = self.solve()?;
        let mut solids: HashMap<usize, (std::sync::Arc<BRep>, BodyId)> = HashMap::new();
        let mut rep = BRep::new();
        let mut bodies = Vec::new();
        for (i, inst) in self.instances.iter().enumerate() {
            let part = self
                .parts
                .get(inst.part)
                .ok_or(AssemblyError::UnknownPart(inst.part))?;
            if let std::collections::hash_map::Entry::Vacant(slot) = solids.entry(inst.part) {
                let ev = engine.evaluate(&part.document);
                if let Some((id, e)) = ev.failures().first() {
                    return Err(AssemblyError::Part {
                        part: part.name.clone(),
                        message: format!("'{}' fails: {e}", part.document.name_of(*id)),
                    });
                }
                let (_, out) = ev.final_solid().ok_or_else(|| AssemblyError::Part {
                    part: part.name.clone(),
                    message: "it has no solid".into(),
                })?;
                let (brep, body) = (out.brep().unwrap().clone(), out.body().unwrap());
                slot.insert((brep, body));
            }
            let (brep, body) = &solids[&inst.part];
            let (b, _) = rep.copy_body_from(
                brep,
                *body,
                &CopyOptions {
                    transform: Some(placements[i]),
                    flip: false,
                },
            );
            bodies.push(b);
        }
        Ok(Pose {
            rep,
            bodies,
            placements,
            names: self.instances.iter().map(|i| i.name.clone()).collect(),
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
}

pub struct Pose {
    pub rep: BRep,
    pub bodies: Vec<BodyId>,
    pub placements: Vec<Transform3>,
    pub names: Vec<String>,
    name: String,
}

impl Pose {
    /// Pairs of instances whose bodies overlap by more than `min_volume` (mm³).
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
