//! Executes one feature: parameters are already literals, inputs are the outputs of the features it
//! reads. Kernel calls are wrapped so that every failure is an [`EvalError`] that names what went wrong.

use crate::eval::{
    EvalError, EvalOutput, Product, Resolution, SketchProduct, ToolProduct, Unresolved,
};
use crate::id::FeatureId;
use crate::namer::Namer;
use crate::naming::{NameMap, NameOrigin, Reference, ResolveKind, Resolved, TopoName};
use crate::profile::{sketch_profile_labeled, sketch_wires};
use crate::spec::*;
use crate::tools::{instance_names, BuiltTool, Place, ToolGen, ToolShape};
use kw_blend::{chamfer, fillet, fillet_law, RadiusLaw};
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{BRep, BodyId, BuildResult, CopyOptions, EdgeId, EntityRef, FaceId, ShapeHistory};
use kw_expr::{Dim, Param, Quantity};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::{
    draft_faces, extrude, extrude_sheet, loft, loft_sheet, offset_body, revolve_sheet, shell,
    sweep, sweep_sheet, Chain, Profile, ProfileLoop, SweepOptions,
};
use kw_nurbs::Curve3;
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
            Product::Solid(_) if o.is_sheet() => Err(EvalError::Dependency(format!(
                "{id} is a surface, not a solid: stitch it closed or thicken it first"
            ))),
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

/// A point given by coordinates or by a datum point feature.
fn point_src(p: &PointSrc, inp: &Inputs) -> Result<Point3<f64>, EvalError> {
    match p {
        PointSrc::Coords(c) => Ok(Point3::from(vec3(c)?)),
        PointSrc::Datum(id) => match &inp.get(*id)?.product {
            Product::Datum(f) => Ok(f.origin),
            _ => Err(EvalError::Dependency(format!("{id} is not a point"))),
        },
    }
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
        FeatureSpec::DatumPlane(d) => match &d.mode {
            PlaneMode::Offset => {
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
            PlaneMode::Midplane(other) => {
                let a = plane_frame(&d.base, inp, &mut notes)?;
                let b = plane_frame(other, inp, &mut notes)?;
                if a.z.cross(&b.z).norm() > 1e-9 {
                    return Err(EvalError::Kernel(
                        "a midplane needs two parallel planes".into(),
                    ));
                }
                let f = Frame3 {
                    origin: Point3::from((a.origin.coords + b.origin.coords) * 0.5),
                    ..a
                };
                // keep the point on the plane: move the origin onto it
                let f = Frame3 {
                    origin: f.origin - f.z * (f.origin - a.origin).dot(&f.z)
                        + f.z * (((b.origin - a.origin).dot(&a.z)) * 0.5),
                    ..f
                };
                Ok(EvalOutput::datum(f, notes))
            }
            PlaneMode::ThreePoints(pts) => {
                let p0 = point_src(&pts[0], inp)?;
                let p1 = point_src(&pts[1], inp)?;
                let p2 = point_src(&pts[2], inp)?;
                let (u, v) = (p1 - p0, p2 - p0);
                let z = u.cross(&v);
                if z.norm() < 1e-9 * (1.0 + u.norm() * v.norm()) {
                    return Err(EvalError::Kernel("the three points lie on one line".into()));
                }
                let z = z.normalize();
                let x = u.normalize();
                Ok(EvalOutput::datum(
                    Frame3 {
                        origin: p0,
                        x,
                        y: z.cross(&x),
                        z,
                    },
                    notes,
                ))
            }
        },
        FeatureSpec::DatumAxis(a) => {
            let (origin, dir) = match &a.kind {
                AxisKind::TwoPoints(pts) => {
                    let (p0, p1) = (point_src(&pts[0], inp)?, point_src(&pts[1], inp)?);
                    (p0, p1 - p0)
                }
                AxisKind::Edge(e) => {
                    let b = body_of(inp, "an axis along an edge")?;
                    let edges = resolve_edges(b, std::slice::from_ref(&**e), &mut notes)?;
                    let rep = b.brep().unwrap();
                    let ed = &rep.edges[edges[0]];
                    match ed.curve.map(|c| &rep.curves[c]) {
                        Some(CurveGeom::Line(l)) => (l.origin, l.dir),
                        _ => {
                            return Err(EvalError::Kernel(
                                "an axis can only follow a straight edge".into(),
                            ))
                        }
                    }
                }
                AxisKind::NormalToFace { face, at } => {
                    let f = plane_frame(&PlaneSpec::Face(face.clone()), inp, &mut notes)?;
                    (
                        f.origin + f.x * length(&at[0])? + f.y * length(&at[1])?,
                        f.z,
                    )
                }
                AxisKind::PlanePlane(pp) => {
                    let a = plane_frame(&pp.0, inp, &mut notes)?;
                    let b = plane_frame(&pp.1, inp, &mut notes)?;
                    let d = a.z.cross(&b.z);
                    if d.norm() < 1e-9 {
                        return Err(EvalError::Kernel(
                            "the two planes are parallel: they have no common line".into(),
                        ));
                    }
                    // a point on both planes: solve n1.p = d1, n2.p = d2, d.p = 0
                    let (d1, d2) = (a.z.dot(&a.origin.coords), b.z.dot(&b.origin.coords));
                    let m = kw_math::Matrix3::new(
                        a.z.x, a.z.y, a.z.z, b.z.x, b.z.y, b.z.z, d.x, d.y, d.z,
                    );
                    let p = m
                        .try_inverse()
                        .ok_or_else(|| EvalError::Kernel("the planes do not meet".into()))?
                        * Vector3::new(d1, d2, 0.0);
                    (Point3::from(p), d)
                }
            };
            if dir.norm() < 1e-12 {
                return Err(EvalError::Kernel("the axis has no direction".into()));
            }
            let z = dir.normalize();
            let helper = if z.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let x = (helper - z * helper.dot(&z)).normalize();
            Ok(EvalOutput::datum(
                Frame3 {
                    origin,
                    x,
                    y: z.cross(&x),
                    z,
                },
                notes,
            ))
        }
        FeatureSpec::DatumPoint(p) => {
            let origin = match &p.kind {
                PointKind::Coords(c) => Point3::from(vec3(c)?),
                PointKind::EdgeMidpoint(e) => {
                    let b = body_of(inp, "a point at an edge")?;
                    let edges = resolve_edges(b, std::slice::from_ref(&**e), &mut notes)?;
                    let rep = b.brep().unwrap();
                    let ed = &rep.edges[edges[0]];
                    match ed.curve.map(|c| &rep.curves[c]) {
                        Some(c) => c.point_at((ed.param_range.0 + ed.param_range.1) / 2.0),
                        None => return Err(EvalError::Kernel("that edge has no curve".into())),
                    }
                }
                PointKind::AxisPlane { axis, plane } => {
                    let a = match &inp.get(*axis)?.product {
                        Product::Datum(f) => *f,
                        _ => return Err(EvalError::Dependency(format!("{axis} is not an axis"))),
                    };
                    let pl = plane_frame(plane, inp, &mut notes)?;
                    let denom = a.z.dot(&pl.z);
                    if denom.abs() < 1e-9 {
                        return Err(EvalError::Kernel(
                            "the axis is parallel to the plane".into(),
                        ));
                    }
                    a.origin + a.z * ((pl.origin - a.origin).dot(&pl.z) / denom)
                }
            };
            Ok(EvalOutput::datum(
                Frame3 {
                    origin,
                    x: Vector3::x(),
                    y: Vector3::y(),
                    z: Vector3::z(),
                },
                notes,
            ))
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
        FeatureSpec::Extrude(x) if x.surface => {
            surface_only_new(x.op, "an extruded surface")?;
            if x.extent != ExtrudeExtent::default() {
                return Err(EvalError::Kernel(
                    "a surface extrude has only a distance (no taper, thin wall, second side or end condition)".into(),
                ));
            }
            let sk = inp.sketch(x.sketch)?;
            let chains = chains_of(&sk)?;
            let d = length(&x.depth)?;
            if !(d > 0.0) {
                return Err(EvalError::Kernel(
                    "the extrusion depth must be positive".into(),
                ));
            }
            let dir = if x.flip { -sk.plane.z } else { sk.plane.z };
            let mut sess = Session::new(id, None);
            let r = extrude_sheet(
                &mut sess.rep,
                &chains,
                dir * d,
                &kw_model::ExtrudeOptions {
                    symmetric: x.symmetric,
                },
            )
            .map_err(|e| kernel("extrude", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
        }
        FeatureSpec::Extrude(x) => {
            let sk = inp.sketch(x.sketch)?;
            let ext = &x.extent;
            let n = sk.plane.z;
            let dir = if x.flip { -n } else { n };
            // the tool may start away from the sketch plane
            let mut start = sk.plane;
            if let Some(p) = &ext.start_offset {
                start.origin += dir * length(p)?;
            }
            let d1 = if ext.through_all {
                // long enough to clear the whole body from wherever the sketch plane is
                let b = body_of(inp, "a through-all extrude")?;
                let bb = b.brep().unwrap().body_bbox(b.body().unwrap());
                let centre = (bb.min.coords + bb.max.coords) * 0.5;
                2.0 * (bb.max - bb.min).norm() + (start.origin.coords - centre).norm() + 1.0
            } else if let Some(face) = &ext.to_face {
                // up to a planar face of the body
                let b = body_of(inp, "an extrude up to a face")?;
                let f = resolve_face(b, face, &mut notes)?;
                let rep = b.brep().unwrap();
                let fr = match &rep.surfaces[rep.faces[f].surface] {
                    kw_nurbs::SurfaceGeom::Plane(p) => p.frame,
                    _ => {
                        return Err(EvalError::Kernel(
                            "an extrude can only stop at a planar face".into(),
                        ))
                    }
                };
                let denom = dir.dot(&fr.z);
                if denom.abs() < 1e-9 {
                    return Err(EvalError::Kernel(
                        "that face is parallel to the extrusion direction".into(),
                    ));
                }
                let t = (fr.origin - start.origin).dot(&fr.z) / denom;
                if !(t > 1e-9) {
                    return Err(EvalError::Kernel(
                        "that face lies behind the sketch plane (flip the direction)".into(),
                    ));
                }
                t
            } else {
                length(&x.depth)?
            };
            if !(d1 > 0.0) {
                return Err(EvalError::Kernel(
                    "the extrusion depth must be positive".into(),
                ));
            }
            let d2 = match &ext.second_side {
                Some(p) => length(p)?,
                None => 0.0,
            };
            if d2 < 0.0 || (d2 > 0.0 && x.symmetric) {
                return Err(EvalError::Kernel(
                    "the second side needs a positive distance and excludes a symmetric extent"
                        .into(),
                ));
            }
            let taper = match &ext.taper {
                Some(p) => angle(p)?,
                None => 0.0,
            };
            if taper.abs() >= std::f64::consts::FRAC_PI_2 - 1e-6 {
                return Err(EvalError::Kernel(
                    "the taper angle must be under 90 degrees".into(),
                ));
            }
            if taper != 0.0 && d2 > 0.0 {
                return Err(EvalError::Kernel(
                    "a taper cannot be combined with a second side".into(),
                ));
            }
            let thin = match &ext.thin {
                Some(t) => Some((length(&t.thickness)?, t.side)),
                None => None,
            };
            // a second side starts the tool that far behind the start plane
            let mut plane = start;
            plane.origin -= dir * d2;
            let gen = ToolGen {
                tag: String::new(),
                shape: ToolShape::Extrude {
                    plane,
                    sketch: sk,
                    sketch_id: x.sketch,
                    vector: dir * (d1 + d2),
                    symmetric: x.symmetric,
                    taper,
                    regions: ext.regions.clone(),
                    thin,
                },
            };
            from_gens(id, inp, vec![gen], x.op, notes)
        }
        FeatureSpec::Rib(r) => {
            let sk = inp.sketch(r.sketch)?;
            let depth = length(&r.depth)?;
            if !(depth > 0.0) {
                return Err(EvalError::Kernel("the rib height must be positive".into()));
            }
            let n = sk.plane.z;
            let dir = if r.flip { -n } else { n };
            let gen = ToolGen {
                tag: String::new(),
                shape: ToolShape::Rib {
                    plane: sk.plane,
                    sketch: sk,
                    sketch_id: r.sketch,
                    vector: dir * depth,
                    thickness: length(&r.thickness)?,
                    symmetric: r.symmetric,
                },
            };
            from_gens(id, inp, vec![gen], r.op, notes)
        }
        FeatureSpec::Pipe(pp) => {
            let path = inp.sketch(pp.path)?;
            let diameter = length(&pp.diameter)?;
            let wall = length(&pp.wall)?;
            if !(diameter > 0.0 && wall > 0.0) {
                return Err(EvalError::Kernel(
                    "the pipe diameter and wall must be positive".into(),
                ));
            }
            let (curve, line) = match path_curve(&path) {
                Ok(c) => (Some(c), None),
                Err(e) => match path_line(&path) {
                    Some(l) => (None, Some(l)),
                    None => return Err(e),
                },
            };
            let (p0, tangent) = match (&curve, &line) {
                (Some(c), _) => {
                    let (t0, _) = c.domain();
                    (c.point_at(t0), c.derivative(t0))
                }
                (_, Some((a, b))) => (*a, b - a),
                _ => unreachable!(),
            };
            let z = tangent.normalize();
            let helper = if z.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let xv = (helper - z * helper.dot(&z)).normalize();
            let frame = Frame3 {
                origin: p0,
                x: xv,
                y: z.cross(&xv),
                z,
            };
            let ro = diameter / 2.0;
            let mut profile = Profile::new(frame, ProfileLoop::circle(&frame, ro));
            if wall < ro - 1e-9 {
                let flipped = Frame3 {
                    origin: p0,
                    x: xv,
                    y: -frame.y,
                    z: -z,
                };
                profile.holes.push(ProfileLoop::circle(&flipped, ro - wall));
            }
            let mut sess = Session::new(id, None);
            let r = match (&curve, &line) {
                (Some(c), _) => sweep(&mut sess.rep, &profile, c, &SweepOptions::default())
                    .map_err(|e| kernel("pipe", e))?,
                (_, Some((a, b))) => extrude(
                    &mut sess.rep,
                    &profile,
                    b - a,
                    &kw_model::ExtrudeOptions::default(),
                )
                .map_err(|e| kernel("pipe", e))?,
                _ => unreachable!(),
            };
            sess.adopt(r);
            finish_with_op(id, inp, sess, pp.op, notes)
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
                AxisSpec::Datum(a) => match &inp.get(*a)?.product {
                    Product::Datum(f) => (f.origin, f.z),
                    _ => return Err(EvalError::Dependency(format!("{a} is not an axis"))),
                },
                AxisSpec::Edge(e) => {
                    let b = body_of(inp, "a revolve about an edge")?;
                    let edges = resolve_edges(b, std::slice::from_ref(&**e), &mut notes)?;
                    let rep = b.brep().unwrap();
                    let ed = &rep.edges[edges[0]];
                    match ed.curve.map(|c| &rep.curves[c]) {
                        Some(CurveGeom::Line(l)) => (l.origin, l.dir),
                        _ => {
                            return Err(EvalError::Kernel(
                                "a revolve can only turn about a straight edge".into(),
                            ))
                        }
                    }
                }
            };
            if axis.norm() < 1e-12 {
                return Err(EvalError::Sketch("the revolve axis has no length".into()));
            }
            if r.surface {
                surface_only_new(r.op, "a revolved surface")?;
                let chains = chains_of(&sk)?;
                let mut sess = Session::new(id, None);
                let built = revolve_sheet(&mut sess.rep, &chains, axis_origin, axis.normalize(), a)
                    .map_err(|e| kernel("revolve", e))?;
                sess.adopt(built);
                return sess.finish(None, notes);
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
        FeatureSpec::Loft(l) if l.surface => {
            surface_only_new(l.op, "a lofted surface")?;
            let mut sections = Vec::new();
            for s in &l.sketches {
                sections.push(chains_of(&*inp.sketch(*s)?)?);
            }
            let mut sess = Session::new(id, None);
            let r = loft_sheet(&mut sess.rep, &sections, l.ruled).map_err(|e| kernel("loft", e))?;
            sess.adopt(r);
            sess.finish(None, notes)
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
            let twist = match &w.twist {
                Some(p) => angle(p)?,
                None => 0.0,
            };
            let guide = match w.guide {
                Some(g) => Some(curve_of(&*inp.sketch(g)?, "a guide rail")?),
                None => None,
            };
            // a twist or a guide rail needs the full sweep even along a straight path
            let shaped = twist != 0.0 || guide.is_some();
            let opts = SweepOptions {
                twist,
                guide,
                ..SweepOptions::default()
            };
            let mut sess = Session::new(id, None);
            if w.surface {
                surface_only_new(w.op, "a swept surface")?;
                let chains = chains_of(&prof)?;
                let curve = curve_of(&path, "a sweep path")?;
                let r = sweep_sheet(&mut sess.rep, &chains, &curve, &opts)
                    .map_err(|e| kernel("sweep", e))?;
                sess.adopt(r);
                return sess.finish(None, notes);
            }
            let profile = sketch_profile_labeled(&prof.sketch, &prof.entities, prof.plane)?.profile;
            let r = match path_curve(&path) {
                Ok(curve) => {
                    sweep(&mut sess.rep, &profile, &curve, &opts).map_err(|e| kernel("sweep", e))?
                }
                Err(e) if shaped => match curve_of(&path, "a sweep path") {
                    Ok(curve) => sweep(&mut sess.rep, &profile, &curve, &opts)
                        .map_err(|e| kernel("sweep", e))?,
                    Err(_) => return Err(e),
                },
                // a straight path is an extrusion along it
                Err(e) => match path_line(&path) {
                    Some((a, b)) => extrude(
                        &mut sess.rep,
                        &profile,
                        b - a,
                        &kw_model::ExtrudeOptions::default(),
                    )
                    .map_err(|e| kernel("sweep", e))?,
                    None => return Err(e),
                },
            };
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
        FeatureSpec::Thread(t) => {
            let b = body_of(inp, "thread")?;
            let size = crate::threads::lookup(&t.designation).ok_or_else(|| {
                EvalError::Kernel(format!("unknown thread size \"{}\"", t.designation))
            })?;
            let len = length(&t.length)?;
            if !(len > 0.0) {
                return Err(EvalError::Kernel(
                    "the thread length must be positive".into(),
                ));
            }
            let f = resolve_face(b, &t.face, &mut notes)?;
            let rep = b.brep().unwrap();
            let d = match &rep.surfaces[rep.faces[f].surface] {
                kw_nurbs::SurfaceGeom::Cylinder(c) => 2.0 * c.radius,
                _ => {
                    return Err(EvalError::Kernel(
                        "a thread goes on a cylindrical face (a hole wall or a pin)".into(),
                    ))
                }
            };
            let ok = if t.internal {
                size.fits_hole(d)
            } else {
                size.fits_shaft(d)
            };
            if !ok {
                return Err(EvalError::Kernel(format!(
                    "{} needs a {} of about dia {:.2} mm; that face is dia {:.2} mm",
                    size.designation,
                    if t.internal { "hole" } else { "pin" },
                    if t.internal {
                        size.tap_drill
                    } else {
                        size.major
                    },
                    d
                )));
            }
            copy_with(id, b, &CopyOptions::default(), notes)
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
            let depth = if h.through_all {
                // as long as it takes to clear the body from the plane the hole starts on
                let bb = b.brep().unwrap().body_bbox(b.body().unwrap());
                let centre = (bb.min.coords + bb.max.coords) * 0.5;
                2.0 * (bb.max - bb.min).norm() + (plane.origin.coords - centre).norm() + 1.0
            } else {
                length(&h.depth)?
            };
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
                    HoleKind::Tapped {
                        designation,
                        thread_length,
                    } => {
                        let size = crate::threads::lookup(designation).ok_or_else(|| {
                            EvalError::Kernel(format!("unknown thread size \"{designation}\""))
                        })?;
                        let tl = length(thread_length)?;
                        if !(tl > 0.0) || (!h.through_all && tl > depth + 1e-9) {
                            return Err(EvalError::Kernel(
                                "the thread length must be positive and at most the hole depth"
                                    .into(),
                            ));
                        }
                        gens.push(cyl(format!("p{k}-"), size.tap_drill, depth));
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
        FeatureSpec::Import(im) => {
            let mut sess = Session::new(id, None);
            let bodies: Vec<BodyId> = match im.format {
                ImportFormat::Step => {
                    kw_step::import(&mut sess.rep, &im.data, &kw_step::ImportOptions::default())
                        .map_err(|e| kernel("STEP import", e))?
                        .bodies
                }
                ImportFormat::Iges => {
                    kw_iges::import(&mut sess.rep, &im.data)
                        .map_err(|e| kernel("IGES import", e))?
                        .bodies
                }
            };
            let Some((&first, rest)) = bodies.split_first() else {
                return Err(EvalError::Kernel(format!(
                    "{} holds no solid body",
                    im.source
                )));
            };
            let mut body = first;
            for b in rest {
                let r = boolean(&mut sess.rep, body, *b, Op::Union, &BoolOptions::default())
                    .map_err(|e| kernel("joining the imported solids", e))?;
                body = r.body;
            }
            sess.body = Some(body);
            sess.names = sess
                .rep
                .body_faces(body)
                .into_iter()
                .enumerate()
                .map(|(k, f)| {
                    (
                        EntityRef::Face(f),
                        TopoName {
                            feature: id,
                            origin: NameOrigin::Primitive {
                                role: format!("face-{k}"),
                            },
                            ordinal: 0,
                        },
                    )
                })
                .collect();
            sess.finish(None, notes)
        }
        FeatureSpec::MoveBody(m) => {
            let b = body_of(inp, "move")?;
            let delta = vec3(&m.delta)?;
            let ang = angle(&m.angle)?;
            let axis = match m.axis {
                0 => Vector3::x(),
                1 => Vector3::y(),
                _ => Vector3::z(),
            };
            let t = Transform3::rotation(axis, ang).then(&Transform3::translation(delta));
            copy_with(
                id,
                b,
                &CopyOptions {
                    transform: Some(t),
                    ..Default::default()
                },
                notes,
            )
        }
        FeatureSpec::ScaleBody(m) => {
            let b = body_of(inp, "scale")?;
            let k = number(&m.factor)?;
            if !(k > 1e-6 && k.is_finite()) {
                return Err(EvalError::Kernel(
                    "the scale factor must be a positive number".into(),
                ));
            }
            let c = Point3::from(vec3(&m.centre)?);
            copy_with(
                id,
                b,
                &CopyOptions {
                    scale: Some((k, c)),
                    ..Default::default()
                },
                notes,
            )
        }
        FeatureSpec::SplitBody(sp) => {
            let b = body_of(inp, "split")?;
            let plane = plane_frame(&sp.plane, inp, &mut notes)?;
            let bb = b.brep().unwrap().body_bbox(b.body().unwrap());
            let size = 4.0 * (bb.max - bb.min).norm()
                + (plane.origin.coords - (bb.min.coords + bb.max.coords) * 0.5).norm()
                + 10.0;
            let dir = match sp.keep {
                SplitSide::Positive => plane.z,
                SplitSide::Negative => -plane.z,
            };
            // the kept half-space: a huge slab standing on the plane
            let mut sess = Session::new(id, Some(b));
            let a = sess.input_body("split")?;
            let slab = kw_model::extrude(
                &mut sess.rep,
                &kw_model::Profile::rectangle(plane, size, size),
                dir * size,
                &kw_model::ExtrudeOptions::default(),
            )
            .map_err(|e| kernel("split", e))?;
            let r = boolean(
                &mut sess.rep,
                a,
                slab.body,
                Op::Intersect,
                &BoolOptions::default(),
            )
            .map_err(|e| kernel("split", e))?;
            sess.adopt(BuildResult {
                body: r.body,
                history: r.history,
            });
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
            let (origin, axis) = match p.axis_feature {
                Some(a) => match &inp.get(a)?.product {
                    Product::Datum(f) => (f.origin, f.z),
                    _ => return Err(EvalError::Dependency(format!("{a} is not an axis"))),
                },
                None => (
                    Point3::from(vec3(&p.axis_origin)?),
                    Vector3::new(p.axis_dir[0], p.axis_dir[1], p.axis_dir[2]),
                ),
            };
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
                MirrorPlane::Datum(d) => match &inp.get(*d)?.product {
                    Product::Datum(f) => (f.origin, f.z),
                    _ => return Err(EvalError::Dependency(format!("{d} is not a plane"))),
                },
                MirrorPlane::Face(r) => {
                    let f = plane_frame(&PlaneSpec::Face(r.clone()), inp, &mut notes)?;
                    (f.origin, f.z)
                }
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
    if b.is_sheet() {
        return Err(EvalError::Dependency(format!(
            "{what} needs a solid body, but the body before it is a surface: stitch it closed or thicken it first"
        )));
    }
    Ok(b)
}

/// A copy of the input body with its geometry moved or scaled; every face keeps its name.
fn copy_with(
    id: FeatureId,
    b: &Arc<EvalOutput>,
    opts: &CopyOptions,
    notes: Vec<Resolution>,
) -> Result<EvalOutput, EvalError> {
    let (src, body) = (b.brep().unwrap(), b.body().unwrap());
    let mut rep = BRep::new();
    let (nb, map) = rep.copy_body_from(src, body, opts);
    let mut names = NameMap::default();
    for (n, e) in &b.names.by_name {
        if let EntityRef::Face(f) = e {
            if let Some(m) = map.faces.get(f) {
                names.by_name.insert(n.clone(), EntityRef::Face(*m));
            }
        }
    }
    let _ = id;
    Ok(EvalOutput::solid(
        rep,
        nb,
        names,
        ShapeHistory::default(),
        None,
        notes,
    ))
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
    if prev.is_sheet() {
        return Err(EvalError::Dependency(
            "a solid cannot be joined to or cut from a surface: stitch the surface closed or thicken it first".into(),
        ));
    }
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

/// A surface is always a body of its own: it cannot be joined to or cut from a solid.
fn surface_only_new(op: BodyOp, what: &str) -> Result<(), EvalError> {
    if op == BodyOp::New {
        Ok(())
    } else {
        Err(EvalError::Kernel(format!(
            "{what} is a body of its own: it cannot be joined, cut or intersected"
        )))
    }
}

/// The chains of curves of a sketch, open and closed, as the surface builders take them.
fn chains_of(sk: &SketchProduct) -> Result<Vec<Chain>, EvalError> {
    let wires = sketch_wires(&sk.sketch, &sk.entities, sk.plane);
    if wires.is_empty() {
        return Err(EvalError::Sketch(
            "the sketch has no curves to make a surface from".into(),
        ));
    }
    Ok(wires
        .into_iter()
        .map(|w| Chain::new(w.plane, w.edges, w.closed))
        .collect())
}

/// The first arc, spline or straight line of a sketch as a curve with a finite domain.
fn curve_of(sk: &SketchProduct, what: &str) -> Result<CurveGeom, EvalError> {
    if let Ok(c) = path_curve(sk) {
        return Ok(c);
    }
    match path_line(sk) {
        Some((a, b)) => Ok(CurveGeom::Nurbs(
            kw_nurbs::NurbsCurve::new_polynomial(
                kw_nurbs::KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).expect("a line"),
                vec![a, b],
            )
            .expect("a line"),
        )),
        None => Err(EvalError::Sketch(format!(
            "{what} must contain a line, an arc or a spline"
        ))),
    }
}

/// The end points of the first straight line of a path sketch (the path of a straight sweep or pipe).
fn path_line(path: &SketchProduct) -> Option<(Point3<f64>, Point3<f64>)> {
    for (_, e) in &path.sketch.entities {
        if e.construction {
            continue;
        }
        if let EntityKind::Line { p1, p2 } = &e.kind {
            let w = |p: (f64, f64)| path.plane.origin + path.plane.x * p.0 + path.plane.y * p.1;
            let (a, b) = (path.sketch.point(*p1).ok()?, path.sketch.point(*p2).ok()?);
            return Some((w(a), w(b)));
        }
    }
    None
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

#[cfg(test)]
mod sheet_guards {
    use super::*;
    use kw_brep::builders::{make_box, make_plane_sheet};
    use kw_math::Frame3;

    fn out(rep: BRep, body: kw_brep::BodyId) -> Arc<EvalOutput> {
        Arc::new(EvalOutput::solid(
            rep,
            body,
            NameMap::default(),
            ShapeHistory::default(),
            None,
            vec![],
        ))
    }

    #[test]
    fn a_surface_is_a_body_but_not_a_solid_for_solid_only_features() {
        let mut rep = BRep::new();
        let sheet = make_plane_sheet(&mut rep, &Frame3::world(), 4.0, 4.0).body;
        let surface = out(rep, sheet);
        let mut rep = BRep::new();
        let b = make_box(&mut rep, &Frame3::world(), 2.0, 2.0, 2.0).body;
        let solid = out(rep, b);
        assert!(surface.is_sheet() && !solid.is_sheet());
        let (sid, bid) = (FeatureId(1), FeatureId(2));
        let mut deps = HashMap::new();
        deps.insert(sid, surface);
        deps.insert(bid, solid);
        // a feature that works on "the body before it"
        let on_sheet = Inputs {
            deps: &deps,
            body: Some(sid),
        };
        let err = body_of(&on_sheet, "Fillet").expect_err("refused");
        assert!(err.to_string().contains("surface"), "{err}");
        let on_solid = Inputs {
            deps: &deps,
            body: Some(bid),
        };
        assert!(body_of(&on_solid, "Fillet").is_ok());
        // a feature that names a body explicitly
        assert!(on_sheet.solid(sid).is_err());
        assert!(on_sheet.solid(bid).is_ok());
    }
}
