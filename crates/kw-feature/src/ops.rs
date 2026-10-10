//! Executes one feature: parameters are already literals, inputs are the outputs of the features it
//! reads. Kernel calls are wrapped so that every failure is an [`EvalError`] that names what went wrong.

use crate::eval::{
    EvalError, EvalOutput, Product, Resolution, SketchProduct, ToolProduct, Unresolved,
};
use crate::id::FeatureId;
use crate::namer::Namer;
use crate::naming::{NameMap, NameOrigin, Reference, ResolveKind, Resolved, TopoName};
use crate::profile::sketch_profile_labeled;
use crate::spec::*;
use crate::tools::{instance_names, BuiltTool, Place, ToolGen, ToolShape};
use kw_blend::{chamfer, fillet, fillet_law, RadiusLaw};
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{BRep, BodyId, BuildResult, CopyOptions, EdgeId, EntityRef, FaceId, ShapeHistory};
use kw_expr::{Dim, Param, Quantity};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::{draft_faces, loft, offset_body, shell, sweep, SweepOptions};
use kw_nurbs::CurveGeom;
use kw_sketch::{EntityKind, Solver};
use std::collections::HashMap;
use std::sync::Arc;

pub struct Inputs<'a> {
    pub deps: &'a HashMap<FeatureId, Arc<EvalOutput>>,
    /// The feature whose body this one modifies.
    pub body: Option<FeatureId>,
}

impl<'a> Inputs<'a> {
    fn get(&self, id: FeatureId) -> Result<&'a Arc<EvalOutput>, EvalError> {
        self.deps
            .get(&id)
            .ok_or_else(|| EvalError::Dependency(format!("the output of {id} is not available")))
    }
    fn body_out(&self) -> Option<&'a Arc<EvalOutput>> {
        self.body.and_then(|b| self.deps.get(&b))
    }
    fn sketch(&self, id: FeatureId) -> Result<Arc<SketchProduct>, EvalError> {
        match &self.get(id)?.product {
            Product::Sketch(s) => Ok(s.clone()),
            _ => Err(EvalError::Dependency(format!("{id} is not a sketch"))),
        }
    }
    fn solid(&self, id: FeatureId) -> Result<&'a Arc<EvalOutput>, EvalError> {
        let o = self.get(id)?;
        match &o.product {
            Product::Solid(_) => Ok(o),
            _ => Err(EvalError::Dependency(format!(
                "{id} does not produce a solid"
            ))),
        }
    }
}

fn lit(p: &Param) -> Result<Quantity, EvalError> {
    match p {
        Param::Lit(q) => Ok(*q),
        Param::Expr(e) => Err(EvalError::Expression(format!(
            "'{}' was not evaluated",
            e.source()
        ))),
    }
}

/// A length in mm (a bare number is taken as mm).
fn length(p: &Param) -> Result<f64, EvalError> {
    let q = lit(p)?;
    if q.dim == Dim::LENGTH || q.dim == Dim::NONE {
        Ok(q.value)
    } else {
        Err(EvalError::Expression(format!(
            "expected a length, got {}",
            q.dim
        )))
    }
}
fn angle(p: &Param) -> Result<f64, EvalError> {
    lit(p)?
        .expect(Dim::ANGLE)
        .map_err(|e| EvalError::Expression(e.to_string()))
}
fn number(p: &Param) -> Result<f64, EvalError> {
    lit(p)?
        .expect(Dim::NONE)
        .map_err(|e| EvalError::Expression(e.to_string()))
}
fn count(p: &Param, what: &str) -> Result<u32, EvalError> {
    let v = number(p)?;
    if v < 1.0 || (v - v.round()).abs() > 1e-9 || v > 10_000.0 {
        return Err(EvalError::Expression(format!(
            "{what} must be a whole number of at least 1, got {v}"
        )));
    }
    Ok(v.round() as u32)
}
fn vec3(p: &[Param; 3]) -> Result<Vector3<f64>, EvalError> {
    Ok(Vector3::new(length(&p[0])?, length(&p[1])?, length(&p[2])?))
}

fn kernel<E: std::fmt::Display>(what: &str, e: E) -> EvalError {
    EvalError::Kernel(format!("{what}: {e}"))
}

// ---------------------------------------------------------------------------------------------
// planes

fn std_frame(origin: Point3<f64>, x: Vector3<f64>, y: Vector3<f64>, z: Vector3<f64>) -> Frame3 {
    Frame3 { origin, x, y, z }
}

/// A frame for a plane of unit normal `n` through `p0`, independent of how the face is parametrised:
/// the origin is the projection of the world origin, x points along the next world axis after the
/// normal's dominant one (so planes parallel to XY, YZ, XZ get the same frames as the standard planes).
pub fn canonical_plane_frame(n: Vector3<f64>, p0: Point3<f64>) -> Frame3 {
    let n = n.normalize();
    let a = [n.x.abs(), n.y.abs(), n.z.abs()];
    let dom = if a[0] >= a[1] && a[0] >= a[2] {
        0
    } else if a[1] >= a[2] {
        1
    } else {
        2
    };
    let mut r = Vector3::zeros();
    r[(dom + 1) % 3] = 1.0;
    let x = (r - n * r.dot(&n)).normalize();
    let y = n.cross(&x);
    let origin = Point3::from(n * n.dot(&p0.coords));
    std_frame(origin, x, y, n)
}

fn plane_frame(
    p: &PlaneSpec,
    inp: &Inputs,
    notes: &mut Vec<Resolution>,
) -> Result<Frame3, EvalError> {
    let o = |q: &Param| length(q);
    Ok(match p {
        PlaneSpec::Xy(off) => std_frame(
            Point3::new(0.0, 0.0, o(off)?),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
        ),
        PlaneSpec::Yz(off) => std_frame(
            Point3::new(o(off)?, 0.0, 0.0),
            Vector3::y(),
            Vector3::z(),
            Vector3::x(),
        ),
        PlaneSpec::Xz(off) => std_frame(
            Point3::new(0.0, o(off)?, 0.0),
            Vector3::z(),
            Vector3::x(),
            Vector3::y(),
        ),
        PlaneSpec::Datum(id) => match &inp.get(*id)?.product {
            Product::Datum(f) => *f,
            _ => return Err(EvalError::Dependency(format!("{id} is not a datum plane"))),
        },
        PlaneSpec::Face(r) => {
            let b = inp.body_out().ok_or_else(|| {
                EvalError::Dependency("a sketch on a face needs a body before it".into())
            })?;
            let rep = b.brep().unwrap();
            let f = resolve_face(b, r, notes)?;
            let face = &rep.faces[f];
            match &rep.surfaces[face.surface] {
                kw_nurbs::SurfaceGeom::Plane(pl) => {
                    let s = if face.sense == kw_brep::Sense::Forward {
                        1.0
                    } else {
                        -1.0
                    };
                    canonical_plane_frame(pl.frame.z * s, pl.frame.origin)
                }
                _ => return Err(EvalError::Reference("the face is not planar".into())),
            }
        }
    })
}

// ---------------------------------------------------------------------------------------------
// references

fn resolve_face(
    body: &EvalOutput,
    r: &Reference,
    notes: &mut Vec<Resolution>,
) -> Result<FaceId, EvalError> {
    let rep = body
        .brep()
        .ok_or_else(|| EvalError::Dependency("the input has no body".into()))?;
    let rev = body.names.reverse();
    let name_of = |f: FaceId| rev.get(&EntityRef::Face(f)).cloned();
    match body.names.resolve_face(rep, r) {
        Resolved::Face(f, kind) => {
            if kind != ResolveKind::Exact {
                if let Some(to) = name_of(f) {
                    notes.push(Resolution {
                        reference: r.clone(),
                        to,
                        kind,
                    });
                }
            }
            Ok(f)
        }
        Resolved::Ambiguous(c) => Err(EvalError::Unresolved(Box::new(Unresolved {
            reference: r.clone(),
            candidates: c
                .into_iter()
                .filter_map(|(f, s)| name_of(f).map(|n| (n, s)))
                .collect(),
            ambiguous: true,
        }))),
        Resolved::Broken => Err(EvalError::Unresolved(Box::new(Unresolved {
            reference: r.clone(),
            candidates: Vec::new(),
            ambiguous: false,
        }))),
    }
}

fn edge_between(rep: &BRep, a: FaceId, b: FaceId) -> Result<EdgeId, EvalError> {
    for lp in &rep.faces[a].loops {
        for co in rep.loop_coedges(*lp) {
            let e = rep.coedges[co].edge;
            if rep.faces_around_edge(e).contains(&b) {
                return Ok(e);
            }
        }
    }
    Err(EvalError::Reference(
        "the selected faces do not share an edge".into(),
    ))
}

fn resolve_edges(
    body: &EvalOutput,
    edges: &[EdgeRef],
    notes: &mut Vec<Resolution>,
) -> Result<Vec<EdgeId>, EvalError> {
    let rep = body.brep().unwrap();
    let mut out: Vec<EdgeId> = Vec::new();
    for e in edges {
        let fa = resolve_face(body, &e.faces[0], notes)?;
        let fb = resolve_face(body, &e.faces[1], notes)?;
        let id = edge_between(rep, fa, fb)?;
        if !out.contains(&id) {
            out.push(id);
        }
    }
    if out.is_empty() {
        return Err(EvalError::Reference("no edges are selected".into()));
    }
    Ok(out)
}

// ---------------------------------------------------------------------------------------------
// a working copy of the body

struct Session {
    rep: BRep,
    body: Option<BodyId>,
    names: HashMap<EntityRef, TopoName>,
    history: ShapeHistory,
    namer: Namer,
}

fn kernel_op(op: BodyOp) -> Op {
    match op {
        BodyOp::Join | BodyOp::New => Op::Union,
        BodyOp::Cut => Op::Subtract,
        BodyOp::Intersect => Op::Intersect,
    }
}

impl Session {
    fn new(feature: FeatureId, input: Option<&EvalOutput>) -> Session {
        let mut s = Session {
            rep: BRep::new(),
            body: None,
            names: HashMap::new(),
            history: ShapeHistory::default(),
            namer: Namer::new(feature),
        };
        if let Some(i) = input {
            if let (Some(brep), Some(b)) = (i.brep(), i.body()) {
                s.rep = (**brep).clone();
                s.body = Some(b);
                s.names = i
                    .names
                    .by_name
                    .iter()
                    .map(|(n, e)| (*e, n.clone()))
                    .collect();
            }
        }
        s
    }

    fn input_body(&self, what: &str) -> Result<BodyId, EvalError> {
        self.body
            .ok_or_else(|| EvalError::Dependency(format!("{what} needs a body before it")))
    }

    /// Record the result of an in-place kernel operation.
    fn adopt(&mut self, r: BuildResult) {
        let named = self
            .namer
            .name_result(&self.rep, r.body, &r.history, &self.names, &|role| {
                NameOrigin::Primitive { role: role.into() }
            });
        self.names = named;
        self.body = Some(r.body);
        self.history = r.history;
    }

    /// Combine a built tool with the body (or make it the body).
    fn apply(
        &mut self,
        tool: BuiltTool,
        tool_names: HashMap<FaceId, TopoName>,
        op: BodyOp,
    ) -> Result<(), EvalError> {
        match (self.body, op) {
            (None, _) | (_, BodyOp::New) => {
                self.rep = tool.rep;
                self.body = Some(tool.body);
                self.names = tool_names
                    .into_iter()
                    .map(|(f, n)| (EntityRef::Face(f), n))
                    .collect();
                self.history = tool.history;
                Ok(())
            }
            (Some(a), _) => {
                if matches!(op, BodyOp::Cut | BodyOp::Intersect)
                    && !self
                        .rep
                        .body_bbox(a)
                        .intersects(&tool.rep.body_bbox(tool.body))
                {
                    return Err(EvalError::Kernel(
                        "the tool does not touch the body, so the operation would change nothing"
                            .into(),
                    ));
                }
                let (b, map) =
                    self.rep
                        .copy_body_from(&tool.rep, tool.body, &CopyOptions::default());
                for (f, n) in tool_names {
                    if let Some(m) = map.faces.get(&f) {
                        self.names.insert(EntityRef::Face(*m), n);
                    }
                }
                let r = boolean(&mut self.rep, a, b, kernel_op(op), &BoolOptions::default())
                    .map_err(|e| kernel("boolean", e))?;
                self.adopt(BuildResult {
                    body: r.body,
                    history: r.history,
                });
                Ok(())
            }
        }
    }

    fn finish(
        self,
        tool: Option<ToolProduct>,
        notes: Vec<Resolution>,
    ) -> Result<EvalOutput, EvalError> {
        let body = self
            .body
            .ok_or_else(|| EvalError::Kernel("the feature produced no body".into()))?;
        let mut out = BRep::new();
        let (nb, map) = out.copy_body_from(&self.rep, body, &CopyOptions::default());
        let mut names = NameMap::default();
        for (e, n) in &self.names {
            if let EntityRef::Face(f) = e {
                if let Some(nf) = map.faces.get(f) {
                    names.by_name.insert(n.clone(), EntityRef::Face(*nf));
                }
            }
        }
        let mut history = ShapeHistory::default();
        for (k, o) in &self.history.origins {
            if let EntityRef::Face(f) = k {
                if let Some(n) = map.faces.get(f) {
                    history.record(EntityRef::Face(*n), o.clone());
                }
            }
        }
        Ok(EvalOutput::solid(
            out,
            nb,
            names,
            history,
            tool.map(Arc::new),
            notes,
        ))
    }
}

// ---------------------------------------------------------------------------------------------
// execution

pub fn execute(id: FeatureId, spec: &FeatureSpec, inp: &Inputs) -> Result<EvalOutput, EvalError> {
    let mut notes: Vec<Resolution> = Vec::new();
    match spec {
        FeatureSpec::Parameter { value, .. } => Ok(EvalOutput::param(lit(value)?)),
        FeatureSpec::DatumPlane(d) => {
            let base = plane_frame(&d.base, inp, &mut notes)?;
            let off = length(&d.offset)?;
            let tilt = angle(&d.tilt)?;
            let t = Transform3::rotation(base.x, tilt);
            let z = t.apply_vector(&base.z);
            let y = t.apply_vector(&base.y);
            let f = Frame3 {
                origin: base.origin + z * off,
                x: base.x,
                y,
                z,
            };
            Ok(EvalOutput::datum(f, notes))
        }
        FeatureSpec::Sketch(s) => {
            let plane = plane_frame(&s.plane, inp, &mut notes)?;
            let (mut sk, ents) = s
                .recipe
                .to_sketch()
                .map_err(|e| EvalError::Sketch(e.to_string()))?;
            // Bound values (dimensions, positions) are applied gradually, solving after every step, so
            // that the sketch moves continuously from the shape it was drawn in and keeps its
            // configuration (a rectangle stays on the same side of its corner, whatever the numbers).
            let mut targets: Vec<(kw_sketch::VarId, f64, f64)> = Vec::new();
            for b in &s.recipe.bindings {
                let v = lit(&b.value)?.value;
                if (b.var as usize) >= sk.vars.values.len() {
                    return Err(EvalError::Sketch(format!(
                        "a binding refers to the variable {}, which does not exist",
                        b.var
                    )));
                }
                let var = kw_sketch::VarId(b.var as usize);
                targets.push((var, sk.value(var), v));
                sk.set_fixed(var, true);
            }
            // The first bound x and y coordinate of a point are the sketch's anchor: the whole sketch is
            // moved there rigidly (a rectangle drawn at the origin is placed at x = 200 without any
            // solving), so that only the shape-changing bindings need to be walked gradually.
            let (mut moved_x, mut moved_y) = (false, false);
            let point_vars: Vec<kw_sketch::PointVars> = sk
                .entities
                .values()
                .filter_map(|e| match &e.kind {
                    EntityKind::Point(p) => Some(*p),
                    _ => None,
                })
                .collect();
            for t in targets.iter_mut() {
                let is_x = point_vars.iter().any(|p| p.x == t.0);
                let is_y = point_vars.iter().any(|p| p.y == t.0);
                if (is_x && !moved_x) || (is_y && !moved_y) {
                    let d = t.2 - t.1;
                    for p in &point_vars {
                        let v = if is_x { p.x } else { p.y };
                        let cur = sk.value(v);
                        sk.set_value(v, cur + d);
                    }
                    if is_x {
                        moved_x = true;
                    } else {
                        moved_y = true;
                    }
                    t.1 = t.2;
                }
            }
            for t in targets.iter_mut() {
                if t.1 != t.2 {
                    t.1 = sk.value(t.0);
                }
            }
            let mut solver = Solver::default();
            let solve =
                |solver: &mut Solver, sk: &mut kw_sketch::Sketch| -> Result<(), EvalError> {
                    if sk.constraints.is_empty() {
                        return Ok(());
                    }
                    let out = solver
                        .solve(sk)
                        .map_err(|e| EvalError::Sketch(e.to_string()))?;
                    if out.status != kw_sketch::SolveStatus::Converged {
                        return Err(EvalError::Sketch(
                            "the constraints could not be satisfied".into(),
                        ));
                    }
                    Ok(())
                };
            let scale = sk
                .bounds()
                .map(|((x0, y0), (x1, y1))| (x1 - x0).max(y1 - y0))
                .unwrap_or(1.0)
                .max(1e-6);
            let biggest = targets
                .iter()
                .map(|(_, a, b)| (b - a).abs())
                .fold(0.0, f64::max);
            let steps = ((biggest / (0.2 * scale)).ceil() as usize).clamp(1, 40);
            for k in 1..=steps {
                let t = k as f64 / steps as f64;
                for (var, a, b) in &targets {
                    sk.set_value(*var, if k == steps { *b } else { a + (b - a) * t });
                }
                solve(&mut solver, &mut sk)?;
            }
            if targets.is_empty() {
                solve(&mut solver, &mut sk)?;
            }
            Ok(EvalOutput::sketch(
                SketchProduct {
                    sketch: sk,
                    plane,
                    entities: ents,
                },
                notes,
            ))
        }
        FeatureSpec::Extrude(x) => {
            let sk = inp.sketch(x.sketch)?;
            let depth = length(&x.depth)?;
            if !(depth > 0.0) {
                return Err(EvalError::Kernel(
                    "the extrusion depth must be positive".into(),
                ));
            }
            let n = sk.plane.z;
            let dir = if x.flip { -n } else { n };
            let gen = ToolGen {
                tag: String::new(),
                shape: ToolShape::Extrude {
                    plane: sk.plane,
                    sketch: sk,
                    sketch_id: x.sketch,
                    vector: dir * depth,
                    symmetric: x.symmetric,
                },
            };
            from_gens(id, inp, vec![gen], x.op, notes)
        }
        FeatureSpec::Revolve(r) => {
            let sk = inp.sketch(r.sketch)?;
            let a = angle(&r.angle)?;
            let (axis_origin, axis) = match &r.axis {
                AxisSpec::SketchX => (sk.plane.origin, sk.plane.x),
                AxisSpec::SketchY => (sk.plane.origin, sk.plane.y),
                AxisSpec::SketchLine(label) => {
                    let e = *sk.entities.get(*label as usize).ok_or_else(|| {
                        EvalError::Sketch(format!("the sketch has no entity {label}"))
                    })?;
                    let (p1, p2) = sk
                        .sketch
                        .line_points(e)
                        .map_err(|e| EvalError::Sketch(e.to_string()))?;
                    let (a, b) = (
                        sk.sketch
                            .point(p1)
                            .map_err(|e| EvalError::Sketch(e.to_string()))?,
                        sk.sketch
                            .point(p2)
                            .map_err(|e| EvalError::Sketch(e.to_string()))?,
                    );
                    let w = |p: (f64, f64)| sk.plane.origin + sk.plane.x * p.0 + sk.plane.y * p.1;
                    (w(a), w(b) - w(a))
                }
            };
            if axis.norm() < 1e-12 {
                return Err(EvalError::Sketch("the revolve axis has no length".into()));
            }
            let gen = ToolGen {
                tag: String::new(),
                shape: ToolShape::Revolve {
                    plane: sk.plane,
                    sketch: sk,
                    sketch_id: r.sketch,
                    axis_origin,
                    axis: axis.normalize(),
                    angle: a,
                },
            };
            from_gens(id, inp, vec![gen], r.op, notes)
        }
        FeatureSpec::Loft(l) => {
            let mut profiles = Vec::new();
            for s in &l.sketches {
                let sk = inp.sketch(*s)?;
                profiles.push(sketch_profile_labeled(&sk.sketch, &sk.entities, sk.plane)?.profile);
            }
            let mut sess = Session::new(id, None);
            let r = loft(&mut sess.rep, &profiles).map_err(|e| kernel("loft", e))?;
            sess.adopt(r);
            finish_with_op(id, inp, sess, l.op, notes)
        }
        FeatureSpec::Sweep(w) => {
            let prof = inp.sketch(w.profile)?;
            let path = inp.sketch(w.path)?;
            let profile = sketch_profile_labeled(&prof.sketch, &prof.entities, prof.plane)?.profile;
            let curve = path_curve(&path)?;
            let mut sess = Session::new(id, None);
            let r = sweep(&mut sess.rep, &profile, &curve, &SweepOptions::default())
                .map_err(|e| kernel("sweep", e))?;
            sess.adopt(r);
            finish_with_op(id, inp, sess, w.op, notes)
        }
        FeatureSpec::Fillet(f) => {
            let b = body_of(inp, "fillet")?;
            let radius = length(&f.radius)?;
            if !(radius > 0.0) {
                return Err(EvalError::Kernel(
                    "the fillet radius must be positive".into(),
                ));
            }
            let edges = resolve_edges(b, &f.edges, &mut notes)?;
            let mut sess = Session::new(id, Some(b));
            let body = sess.input_body("fillet")?;
            let r = fillet(&mut sess.rep, body, &edges, radius).map_err(|e| kernel("fillet", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::Chamfer(c) => {
            let b = body_of(inp, "chamfer")?;
            let (d1, d2) = (length(&c.d1)?, length(&c.d2)?);
            if !(d1 > 0.0 && d2 > 0.0) {
                return Err(EvalError::Kernel(
                    "the chamfer distances must be positive".into(),
                ));
            }
            let edges = resolve_edges(b, &c.edges, &mut notes)?;
            let mut sess = Session::new(id, Some(b));
            let body = sess.input_body("chamfer")?;
            let r =
                chamfer(&mut sess.rep, body, &edges, d1, d2).map_err(|e| kernel("chamfer", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::VariableFillet(v) => {
            let b = body_of(inp, "variable fillet")?;
            let (s, e) = (length(&v.start)?, length(&v.end)?);
            if !(s > 0.0 && e > 0.0) {
                return Err(EvalError::Kernel(
                    "the fillet radii must be positive".into(),
                ));
            }
            let edges = resolve_edges(b, std::slice::from_ref(&v.edge), &mut notes)?;
            let mut sess = Session::new(id, Some(b));
            let body = sess.input_body("variable fillet")?;
            let r = fillet_law(
                &mut sess.rep,
                body,
                edges[0],
                &RadiusLaw::Linear { start: s, end: e },
            )
            .map_err(|e| kernel("variable fillet", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::Shell(s) => {
            let b = body_of(inp, "shell")?;
            let t = length(&s.thickness)?;
            let mut faces = Vec::new();
            for r in &s.open {
                faces.push(resolve_face(b, r, &mut notes)?);
            }
            let mut sess = Session::new(id, Some(b));
            let body = sess.input_body("shell")?;
            let r = shell(&mut sess.rep, body, t, &faces).map_err(|e| kernel("shell", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::Draft(d) => {
            let b = body_of(inp, "draft")?;
            let a = angle(&d.angle)?;
            let origin = Point3::from(vec3(&d.neutral_origin)?);
            let pull = Vector3::new(d.pull[0], d.pull[1], d.pull[2]);
            if pull.norm() < 1e-12 {
                return Err(EvalError::Kernel("the pull direction has no length".into()));
            }
            let mut faces = Vec::new();
            for r in &d.faces {
                faces.push(resolve_face(b, r, &mut notes)?);
            }
            let mut sess = Session::new(id, Some(b));
            let body = sess.input_body("draft")?;
            let r = draft_faces(&mut sess.rep, body, &faces, origin, pull, a)
                .map_err(|e| kernel("draft", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::Offset(o) => {
            let b = body_of(inp, "offset")?;
            let d = length(&o.distance)?;
            let mut sess = Session::new(id, Some(b));
            let body = sess.input_body("offset")?;
            let r = offset_body(&mut sess.rep, body, d).map_err(|e| kernel("offset", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::Combine(c) => {
            let t = inp.solid(c.target)?;
            let tool = inp.solid(c.tool)?;
            let mut sess = Session::new(id, Some(t));
            let a = sess.input_body("combine")?;
            let (tb, tbody) = (tool.brep().unwrap(), tool.body().unwrap());
            let (b, map) = sess.rep.copy_body_from(tb, tbody, &CopyOptions::default());
            for (n, e) in &tool.names.by_name {
                if let EntityRef::Face(f) = e {
                    if let Some(m) = map.faces.get(f) {
                        sess.names.insert(EntityRef::Face(*m), n.clone());
                    }
                }
            }
            let r = boolean(
                &mut sess.rep,
                a,
                b,
                kernel_op(c.op),
                &BoolOptions::default(),
            )
            .map_err(|e| kernel("combine", e))?;
            sess.adopt(BuildResult {
                body: r.body,
                history: r.history,
            });
            sess.finish(None, notes)
        }
        FeatureSpec::Hole(h) => {
            let b = body_of(inp, "hole")?;
            let plane = plane_frame(&h.plane, inp, &mut notes)?;
            let depth = length(&h.depth)?;
            if !(depth > 0.0) {
                return Err(EvalError::Kernel("the hole depth must be positive".into()));
            }
            let mut gens: Vec<ToolGen> = Vec::new();
            for (k, pt) in h.points.iter().enumerate() {
                let (u, v) = (length(&pt[0])?, length(&pt[1])?);
                let origin = plane.origin + plane.x * u + plane.y * v;
                // drills into the material: against the plane's normal
                let frame = Frame3 {
                    origin,
                    x: plane.x,
                    y: -plane.y, // right-handed with z = -normal
                    z: -plane.z,
                };
                // the tool starts 1 mm above the plane so that its end cap never lies in the entry face
                let lead = 1.0_f64;
                let up = |f: &Frame3, d: f64| Frame3 {
                    origin: f.origin - f.z * d,
                    ..*f
                };
                let cyl = |tag: String, d: f64, depth: f64| ToolGen {
                    tag,
                    shape: ToolShape::Cylinder {
                        frame: up(&frame, lead),
                        radius: d / 2.0,
                        height: depth + lead,
                    },
                };
                match &h.kind {
                    HoleKind::Simple { diameter } => {
                        let d = length(diameter)?;
                        if !(d > 0.0) {
                            return Err(EvalError::Kernel(
                                "the hole diameter must be positive".into(),
                            ));
                        }
                        gens.push(cyl(format!("p{k}-"), d, depth));
                    }
                    HoleKind::Counterbore {
                        diameter,
                        cb_diameter,
                        cb_depth,
                    } => {
                        let (d, cd, cdp) =
                            (length(diameter)?, length(cb_diameter)?, length(cb_depth)?);
                        if !(d > 0.0 && cd > d && cdp > 0.0 && cdp < depth) {
                            return Err(EvalError::Kernel(
                                "a counterbore needs 0 < diameter < counterbore diameter and 0 < counterbore depth < depth".into(),
                            ));
                        }
                        gens.push(cyl(format!("p{k}-bore-"), d, depth));
                        gens.push(cyl(format!("p{k}-cb-"), cd, cdp));
                    }
                    HoleKind::Countersink {
                        diameter,
                        cs_diameter,
                        cs_angle,
                    } => {
                        let (d, cd, ang) =
                            (length(diameter)?, length(cs_diameter)?, angle(cs_angle)?);
                        if !(d > 0.0 && cd > d && ang > 0.0 && ang < std::f64::consts::PI) {
                            return Err(EvalError::Kernel(
                                "a countersink needs 0 < diameter < countersink diameter and an angle between 0 and 180 degrees".into(),
                            ));
                        }
                        let h_cone = (cd - d) / 2.0 / (ang / 2.0).tan();
                        if h_cone >= depth {
                            return Err(EvalError::Kernel(
                                "the countersink is deeper than the hole".into(),
                            ));
                        }
                        gens.push(cyl(format!("p{k}-bore-"), d, depth));
                        gens.push(ToolGen {
                            tag: format!("p{k}-cs-"),
                            shape: ToolShape::Cone {
                                frame,
                                r0: cd / 2.0,
                                r1: d / 2.0,
                                height: h_cone,
                            },
                        });
                    }
                }
            }
            if gens.is_empty() {
                return Err(EvalError::Kernel(
                    "the hole feature has no positions".into(),
                ));
            }
            let _ = b;
            from_gens(id, inp, gens, BodyOp::Cut, notes)
        }
        FeatureSpec::SheetMetal(m) => {
            let strip = sheet_strip(m)?;
            let mut sess = Session::new(id, None);
            let r = kw_sheet::build(&mut sess.rep, &strip).map_err(|e| kernel("sheet metal", e))?;
            let n = kw_sheet::piece_count(&strip);
            let named = sess.namer.name_result(
                &sess.rep,
                r.body,
                &r.history,
                &HashMap::new(),
                &|role: &str| {
                    let role = match role
                        .strip_prefix("side-0-")
                        .and_then(|k| k.parse::<usize>().ok())
                    {
                        Some(k) => kw_sheet::side_face_role(k, n),
                        None => role.to_string(),
                    };
                    NameOrigin::Primitive { role }
                },
            );
            sess.names = named;
            sess.body = Some(r.body);
            sess.history = r.history;
            sess.finish(None, notes)
        }
        FeatureSpec::Subd(sd) => {
            let scale = number(&sd.scale)?;
            let tol = length(&sd.tolerance)?;
            if !(scale > 0.0 && tol > 0.0) {
                return Err(EvalError::Kernel(
                    "the subdivision scale and tolerance must be positive".into(),
                ));
            }
            let mut cage = sd.cage.clone();
            for v in &mut cage.verts {
                for c in v.iter_mut() {
                    *c *= scale;
                }
            }
            let fin = kw_subd::finish(
                &cage,
                &kw_subd::FinishOptions {
                    tolerance: tol,
                    ..Default::default()
                },
            )
            .map_err(|e| kernel("subdivision surface", e))?;
            let mut sess = Session::new(id, None);
            sess.rep = fin.rep;
            sess.body = Some(fin.body);
            let mut names: HashMap<EntityRef, TopoName> = HashMap::new();
            for (f, (cf, path)) in sess
                .rep
                .body_faces(fin.body)
                .into_iter()
                .zip(&fin.patch_paths)
            {
                names.insert(
                    EntityRef::Face(f),
                    TopoName {
                        feature: id,
                        origin: NameOrigin::Primitive {
                            role: format!("cage-{cf}-{path}"),
                        },
                        ordinal: 0,
                    },
                );
            }
            sess.names = names;
            sess.finish(None, notes)
        }
        FeatureSpec::LinearPattern(p) => {
            let n = count(&p.count, "the pattern count")?;
            let step = vec3(&p.spacing)?;
            let places: Vec<(u32, Place)> = (1..n)
                .map(|i| (i, Place::Rigid(Transform3::translation(step * i as f64))))
                .collect();
            pattern(id, inp, p.source, places, notes)
        }
        FeatureSpec::CircularPattern(p) => {
            let n = count(&p.count, "the pattern count")?;
            let origin = Point3::from(vec3(&p.axis_origin)?);
            let axis = Vector3::new(p.axis_dir[0], p.axis_dir[1], p.axis_dir[2]);
            if axis.norm() < 1e-12 {
                return Err(EvalError::Kernel("the pattern axis has no length".into()));
            }
            let total = angle(&p.total_angle)?;
            let full = (total - std::f64::consts::TAU).abs() < 1e-9;
            let step = if full || n == 1 {
                total / n as f64
            } else {
                total / (n - 1) as f64
            };
            let to_origin = Transform3::translation(-origin.coords);
            let back = Transform3::translation(origin.coords);
            let places: Vec<(u32, Place)> = (1..n)
                .map(|i| {
                    let rot = Transform3::rotation(axis, step * i as f64);
                    (i, Place::Rigid(to_origin.then(&rot).then(&back)))
                })
                .collect();
            pattern(id, inp, p.source, places, notes)
        }
        FeatureSpec::Mirror(m) => {
            let (point, normal) = match &m.plane {
                MirrorPlane::Xy(o) => (Point3::new(0.0, 0.0, length(o)?), Vector3::z()),
                MirrorPlane::Yz(o) => (Point3::new(length(o)?, 0.0, 0.0), Vector3::x()),
                MirrorPlane::Xz(o) => (Point3::new(0.0, length(o)?, 0.0), Vector3::y()),
            };
            pattern(
                id,
                inp,
                m.source,
                vec![(1, Place::Mirror { point, normal })],
                notes,
            )
        }
    }
}

/// The strip a (resolved) sheet-metal feature describes.
pub fn sheet_strip(m: &SheetMetalSpec) -> Result<kw_sheet::SheetStrip, EvalError> {
    let mut segs = Vec::new();
    for g in &m.segs {
        segs.push(match g {
            SheetSegSpec::Flat { length } => kw_sheet::Seg::Flat {
                length: length_of(length)?,
            },
            SheetSegSpec::Bend { angle: a, radius } => kw_sheet::Seg::Bend {
                angle: angle(a)?,
                radius: match radius {
                    Some(r) => Some(length_of(r)?),
                    None => None,
                },
            },
        });
    }
    let strip = kw_sheet::SheetStrip {
        spec: kw_sheet::SheetSpec {
            thickness: length_of(&m.thickness)?,
            k_factor: number(&m.k_factor)?,
            default_radius: length_of(&m.default_radius)?,
        },
        width: length_of(&m.width)?,
        segs,
    };
    strip.validate().map_err(|e| kernel("sheet metal", e))?;
    Ok(strip)
}

fn length_of(p: &Param) -> Result<f64, EvalError> {
    length(p)
}

fn body_of<'a>(inp: &Inputs<'a>, what: &str) -> Result<&'a Arc<EvalOutput>, EvalError> {
    let b = inp
        .body_out()
        .ok_or_else(|| EvalError::Dependency(format!("{what} needs a body before it")))?;
    if b.brep().is_none() {
        return Err(EvalError::Dependency(format!(
            "{what}: the input is not a solid"
        )));
    }
    Ok(b)
}

/// Build every tool, apply them in turn, and remember them if they were added to a body.
fn from_gens(
    id: FeatureId,
    inp: &Inputs,
    gens: Vec<ToolGen>,
    op: BodyOp,
    notes: Vec<Resolution>,
) -> Result<EvalOutput, EvalError> {
    let mut sess = Session::new(id, inp.body_out().map(|b| &**b));
    let had_body = sess.body.is_some() && op != BodyOp::New;
    for g in &gens {
        let built = g.build(id)?;
        let names = built.names.clone();
        sess.apply(built, names, op)?;
    }
    let tool = had_body.then(|| ToolProduct {
        gens,
        op,
        source: id,
    });
    sess.finish(tool, notes)
}

/// A solid made from scratch inside `sess` (loft, sweep) joined to or cut from the body.
fn finish_with_op(
    id: FeatureId,
    inp: &Inputs,
    sess: Session,
    op: BodyOp,
    notes: Vec<Resolution>,
) -> Result<EvalOutput, EvalError> {
    let body_in = if op == BodyOp::New {
        None
    } else {
        inp.body_out()
    };
    let Some(prev) = body_in else {
        return sess.finish(None, notes);
    };
    // the new solid becomes a tool: name it, then combine
    let body = sess.body.unwrap();
    let mut tool_names: HashMap<FaceId, TopoName> = HashMap::new();
    for (e, n) in &sess.names {
        if let EntityRef::Face(f) = e {
            tool_names.insert(*f, n.clone());
        }
    }
    let built = BuiltTool {
        rep: sess.rep,
        body,
        history: sess.history,
        names: tool_names.clone(),
    };
    let mut s2 = Session::new(id, Some(prev));
    s2.apply(built, tool_names, op)?;
    s2.finish(None, notes)
}

fn path_curve(path: &SketchProduct) -> Result<CurveGeom, EvalError> {
    for (id, e) in &path.sketch.entities {
        if e.construction {
            continue;
        }
        let w = |p: (f64, f64)| path.plane.origin + path.plane.x * p.0 + path.plane.y * p.1;
        match &e.kind {
            EntityKind::Arc { .. } => {
                let (c, r, a0, a1) = kw_sketch::geom::arc_params(&path.sketch, id)
                    .map_err(|e| EvalError::Sketch(e.to_string()))?;
                let frame = Frame3 {
                    origin: w(c),
                    ..path.plane
                };
                let n = kw_nurbs::build::make_arc(&frame, r, a0, a1 - a0)
                    .map_err(|e| EvalError::Sketch(format!("{e:?}")))?;
                return Ok(CurveGeom::Nurbs(n));
            }
            EntityKind::Spline { .. } => {
                let n = kw_sketch::geom::spline_curve(&path.sketch, id)
                    .map_err(|e| EvalError::Sketch(e.to_string()))?;
                let t = Transform3::frame_to_world(&path.plane);
                return Ok(CurveGeom::Nurbs(kw_nurbs::fit::transform_nurbs_curve(
                    &n, &t,
                )));
            }
            _ => {}
        }
    }
    Err(EvalError::Sketch(
        "a sweep path must contain an arc or a spline".into(),
    ))
}

/// Repeat the tools of `source` at each place and apply them to the current body.
fn pattern(
    id: FeatureId,
    inp: &Inputs,
    source: FeatureId,
    places: Vec<(u32, Place)>,
    notes: Vec<Resolution>,
) -> Result<EvalOutput, EvalError> {
    let b = body_of(inp, "a pattern")?;
    let src = inp.solid(source)?;
    let tool = src
        .solid_product()
        .and_then(|s| s.tool.clone())
        .ok_or_else(|| {
            EvalError::Dependency(
                "the source feature adds or removes material from an existing body in a way that can be repeated only if it is an extrude, revolve or hole".into(),
            )
        })?;
    let mut sess = Session::new(id, Some(b));
    for (index, place) in &places {
        for g in &tool.gens {
            let placed = g.placed(place);
            // names as the source feature gave them, wrapped as instances of this feature
            let original = g.build(tool.source)?;
            let built = placed.build(tool.source)?;
            if built.names.len() != original.names.len() {
                return Err(EvalError::Kernel(
                    "the repeated tool has a different face structure than the original".into(),
                ));
            }
            // pair faces by their original name
            let by_name: HashMap<&TopoName, FaceId> =
                built.names.iter().map(|(f, n)| (n, *f)).collect();
            let mut inst: HashMap<FaceId, TopoName> = HashMap::new();
            for (n, _) in original.names.iter().map(|(f, n)| (n, f)) {
                let Some(&bf) = by_name.get(n) else {
                    return Err(EvalError::Kernel(
                        "the repeated tool does not match the original".into(),
                    ));
                };
                inst.insert(
                    bf,
                    TopoName {
                        feature: id,
                        origin: NameOrigin::Instance {
                            source: Box::new(n.clone()),
                            index: *index,
                        },
                        ordinal: 0,
                    },
                );
            }
            let _ = instance_names;
            sess.apply(built, inst, tool.op).map_err(|e| match e {
                EvalError::Kernel(m) => {
                    EvalError::Kernel(format!("instance {index} of the pattern: {m}"))
                }
                o => o,
            })?;
        }
    }
    sess.finish(None, notes)
}
