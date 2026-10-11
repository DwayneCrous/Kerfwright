//! Tool bodies: the solids that an Extrude, Revolve or Hole adds to or removes from the body, described
//! by how they are built so that a pattern or mirror can rebuild them somewhere else and name their
//! faces after the faces of the original.

use crate::eval::{EvalError, SketchProduct};
use crate::id::FeatureId;
use crate::namer::Namer;
use crate::naming::{NameOrigin, TopoName};
use crate::profile::sketch_profile_labeled;
use crate::spec::ThinSide;
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::{make_cone, BRep, BodyId, EntityRef, FaceId, ShapeHistory};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::{
    draft_faces, extrude, offset_body, revolve, shell, ExtrudeOptions, Profile, ProfileLoop,
};
use std::collections::HashMap;
use std::sync::Arc;

#[derive(Debug, Clone)]
pub enum ToolShape {
    Extrude {
        sketch: Arc<SketchProduct>,
        sketch_id: FeatureId,
        plane: Frame3,
        vector: Vector3<f64>,
        symmetric: bool,
        /// Taper angle (radians); positive narrows the profile along `vector`. Zero = straight.
        taper: f64,
        /// Regions to extrude (see `regions.rs`); empty = all loops.
        regions: Vec<Vec<u32>>,
        /// A wall of this thickness along the profile instead of a solid.
        thin: Option<(f64, ThinSide)>,
    },
    /// Every line and arc of the sketch thickened in its plane, then extruded.
    Rib {
        sketch: Arc<SketchProduct>,
        sketch_id: FeatureId,
        plane: Frame3,
        vector: Vector3<f64>,
        thickness: f64,
        symmetric: bool,
    },
    Revolve {
        sketch: Arc<SketchProduct>,
        sketch_id: FeatureId,
        plane: Frame3,
        axis_origin: Point3<f64>,
        axis: Vector3<f64>,
        angle: f64,
    },
    Cylinder {
        frame: Frame3,
        radius: f64,
        height: f64,
    },
    Cone {
        frame: Frame3,
        r0: f64,
        r1: f64,
        height: f64,
    },
}

#[derive(Debug, Clone)]
pub struct ToolGen {
    /// Prefix of the roles of the faces this tool creates (tells the tools of one feature apart).
    pub tag: String,
    pub shape: ToolShape,
}

/// Where a tool is rebuilt.
#[derive(Debug, Clone, Copy)]
pub enum Place {
    Rigid(Transform3),
    /// Reflection in the plane through `point` with unit `normal`.
    Mirror {
        point: Point3<f64>,
        normal: Vector3<f64>,
    },
}

impl Place {
    fn point(&self, p: &Point3<f64>) -> Point3<f64> {
        match self {
            Place::Rigid(t) => t.apply_point(p),
            Place::Mirror { point, normal } => p - normal * (2.0 * (p - point).dot(normal)),
        }
    }
    fn vector(&self, v: &Vector3<f64>) -> Vector3<f64> {
        match self {
            Place::Rigid(t) => t.apply_vector(v),
            Place::Mirror { normal, .. } => v - normal * (2.0 * v.dot(normal)),
        }
    }
    fn mirrored(&self) -> bool {
        matches!(self, Place::Mirror { .. })
    }
    /// A right-handed frame whose x and y are the images of the originals (mirrored frames flip z).
    fn frame(&self, f: &Frame3) -> Frame3 {
        let x = self.vector(&f.x);
        let y = self.vector(&f.y);
        let z = if self.mirrored() {
            x.cross(&y)
        } else {
            self.vector(&f.z)
        };
        Frame3 {
            origin: self.point(&f.origin),
            x,
            y,
            z,
        }
    }
    /// Frame for a tool built along its z axis (the axis maps to the image of z, the handedness is
    /// restored through y).
    fn axis_frame(&self, f: &Frame3) -> Frame3 {
        let z = self.vector(&f.z);
        let x = self.vector(&f.x);
        let y = z.cross(&x);
        Frame3 {
            origin: self.point(&f.origin),
            x,
            y,
            z,
        }
    }
}

pub struct BuiltTool {
    pub rep: BRep,
    pub body: BodyId,
    pub history: ShapeHistory,
    pub names: HashMap<FaceId, TopoName>,
}

impl ToolGen {
    pub fn placed(&self, place: &Place) -> ToolGen {
        let shape = match &self.shape {
            ToolShape::Extrude {
                sketch,
                sketch_id,
                plane,
                vector,
                symmetric,
                taper,
                regions,
                thin,
            } => ToolShape::Extrude {
                sketch: sketch.clone(),
                sketch_id: *sketch_id,
                plane: place.frame(plane),
                vector: place.vector(vector),
                symmetric: *symmetric,
                taper: *taper,
                regions: regions.clone(),
                thin: *thin,
            },
            ToolShape::Rib {
                sketch,
                sketch_id,
                plane,
                vector,
                thickness,
                symmetric,
            } => ToolShape::Rib {
                sketch: sketch.clone(),
                sketch_id: *sketch_id,
                plane: place.frame(plane),
                vector: place.vector(vector),
                thickness: *thickness,
                symmetric: *symmetric,
            },
            ToolShape::Revolve {
                sketch,
                sketch_id,
                plane,
                axis_origin,
                axis,
                angle,
            } => ToolShape::Revolve {
                sketch: sketch.clone(),
                sketch_id: *sketch_id,
                plane: place.frame(plane),
                axis_origin: place.point(axis_origin),
                axis: if place.mirrored() {
                    -place.vector(axis)
                } else {
                    place.vector(axis)
                },
                angle: *angle,
            },
            ToolShape::Cylinder {
                frame,
                radius,
                height,
            } => ToolShape::Cylinder {
                frame: place.axis_frame(frame),
                radius: *radius,
                height: *height,
            },
            ToolShape::Cone {
                frame,
                r0,
                r1,
                height,
            } => ToolShape::Cone {
                frame: place.axis_frame(frame),
                r0: *r0,
                r1: *r1,
                height: *height,
            },
        };
        ToolGen {
            tag: self.tag.clone(),
            shape,
        }
    }

    /// Build the solid in a fresh B-rep and name its faces as `feature` would.
    pub fn build(&self, feature: FeatureId) -> Result<BuiltTool, EvalError> {
        let mut rep = BRep::new();
        let tag = self.tag.clone();
        let kernel =
            |what: &str, e: &dyn std::fmt::Display| EvalError::Kernel(format!("{what}: {e}"));
        let mut sketch_id = feature;
        let mut swept_role = "side";
        // every kernel result of the build, in order, with the entity labels its face roles refer to; the
        // last one is the tool, the earlier ones are the intermediate solids it was made from
        let mut steps: Vec<(kw_brep::BuildResult, Vec<Vec<u32>>)> = Vec::new();
        match &self.shape {
            ToolShape::Extrude {
                sketch,
                sketch_id: sid,
                plane,
                vector,
                symmetric,
                taper,
                regions,
                thin,
            } => {
                sketch_id = *sid;
                extrude_steps(
                    &mut rep, sketch, *plane, *vector, *symmetric, *taper, regions, *thin,
                    &mut steps,
                )?;
            }
            ToolShape::Rib {
                sketch,
                sketch_id: sid,
                plane,
                vector,
                thickness,
                symmetric,
            } => {
                sketch_id = *sid;
                rib_steps(
                    &mut rep, sketch, *plane, *vector, *thickness, *symmetric, &mut steps,
                )?;
            }
            ToolShape::Revolve {
                sketch,
                sketch_id: sid,
                plane,
                axis_origin,
                axis,
                angle,
            } => {
                sketch_id = *sid;
                swept_role = "sweep";
                let lp = sketch_profile_labeled(&sketch.sketch, &sketch.entities, *plane)?;
                let r = revolve(&mut rep, &lp.profile, *axis_origin, *axis, *angle)
                    .map_err(|e| kernel("revolve", &e))?;
                steps.push((r, lp.labels));
            }
            ToolShape::Cylinder {
                frame,
                radius,
                height,
            } => {
                if !(*radius > 0.0 && *height > 0.0) {
                    return Err(EvalError::Kernel(
                        "a cylinder needs a positive radius and height".into(),
                    ));
                }
                // built as the extrusion of a circle (like a sketch feature's tool): its caps then
                // meet coincident faces of the body the way the Boolean expects
                let profile = Profile::new(*frame, ProfileLoop::circle(frame, *radius));
                let r = extrude(
                    &mut rep,
                    &profile,
                    frame.z * *height,
                    &ExtrudeOptions::default(),
                )
                .map_err(|e| kernel("extrude", &e))?;
                steps.push((r, Vec::new()));
            }
            ToolShape::Cone {
                frame,
                r0,
                r1,
                height,
            } => {
                if !(*r0 > 0.0 && *r1 >= 0.0 && *height > 0.0) {
                    return Err(EvalError::Kernel(
                        "a cone needs a positive base radius and height".into(),
                    ));
                }
                steps.push((make_cone(&mut rep, frame, *r0, *r1, *height), Vec::new()));
            }
        }
        let mut namer = Namer::new(feature);
        let mut named: HashMap<EntityRef, TopoName> = HashMap::new();
        for (res, labels) in &steps {
            let origin = |role: &str| -> NameOrigin {
                // "side-L-E" / "sweep-L-E": the face swept from entity labels[L][E]
                let prefix = format!("{swept_role}-");
                if let Some(rest) = role.strip_prefix(prefix.as_str()) {
                    let mut it = rest.split('-');
                    if let (Some(l), Some(e)) = (it.next(), it.next()) {
                        if let (Ok(l), Ok(e)) = (l.parse::<usize>(), e.parse::<usize>()) {
                            if let Some(&entity) = labels.get(l).and_then(|v| v.get(e)) {
                                return NameOrigin::SketchEdge {
                                    sketch: sketch_id,
                                    entity,
                                    role: format!("{tag}{swept_role}"),
                                };
                            }
                        }
                    }
                }
                NameOrigin::Primitive {
                    role: format!("{tag}{role}"),
                }
            };
            let n = namer.name_result(&rep, res.body, &res.history, &named, &origin);
            named.extend(n);
        }
        let result = steps
            .into_iter()
            .last()
            .map(|(r, _)| r)
            .ok_or_else(|| EvalError::Kernel("the tool has no shape".into()))?;
        let names = named
            .into_iter()
            .filter_map(|(k, v)| match k {
                EntityRef::Face(f) => Some((f, v)),
                _ => None,
            })
            .collect();
        Ok(BuiltTool {
            rep,
            body: result.body,
            history: result.history,
            names,
        })
    }
}

fn is_cap(rep: &BRep, f: FaceId, pull: &Vector3<f64>) -> bool {
    matches!(&rep.surfaces[rep.faces[f].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.z.dot(pull).abs() > 1.0 - 1e-9)
}

/// The oriented outer loop (counter-clockwise about the plane normal) of `l`.
fn as_outer(l: &ProfileLoop, plane: &Frame3) -> ProfileLoop {
    let area = l.check(&plane.z, 1e-6).unwrap_or(1.0);
    if area >= 0.0 {
        return l.clone();
    }
    if l.edges.len() == 1 {
        if let kw_nurbs::CurveGeom::Circle(c) = &l.edges[0].curve {
            let f = Frame3 {
                origin: c.frame.origin,
                x: c.frame.x,
                y: -c.frame.y,
                z: -c.frame.z,
            };
            return ProfileLoop::circle(&f, c.radius);
        }
    }
    ProfileLoop::new(
        l.edges
            .iter()
            .rev()
            .map(|e| kw_model::ProfileEdge {
                reversed: !e.reversed,
                ..e.clone()
            })
            .collect(),
    )
}

/// Extrude a sketch, optionally only some regions of it, tapered, or as a thin wall.
#[allow(clippy::too_many_arguments)]
fn extrude_steps(
    rep: &mut BRep,
    sketch: &SketchProduct,
    plane: Frame3,
    vector: Vector3<f64>,
    symmetric: bool,
    taper: f64,
    regions: &[Vec<u32>],
    thin: Option<(f64, ThinSide)>,
    steps: &mut Vec<(kw_brep::BuildResult, Vec<Vec<u32>>)>,
) -> Result<(), EvalError> {
    let kernel = |what: &str, e: &dyn std::fmt::Display| EvalError::Kernel(format!("{what}: {e}"));
    let opts = ExtrudeOptions { symmetric };
    let pull = vector.normalize();
    if regions.is_empty() {
        let lp = sketch_profile_labeled(&sketch.sketch, &sketch.entities, plane)?;
        let straight =
            extrude(rep, &lp.profile, vector, &opts).map_err(|e| kernel("extrude", &e))?;
        steps.push((straight.clone(), lp.labels));
        if taper != 0.0 {
            if symmetric {
                return Err(EvalError::Kernel(
                    "a taper cannot be combined with a symmetric extent".into(),
                ));
            }
            if thin.is_some() {
                return Err(EvalError::Kernel(
                    "a taper cannot be combined with a thin wall".into(),
                ));
            }
            // every side face leans; the two caps (perpendicular to the extrusion) stay put
            let sides: Vec<FaceId> = rep
                .body_faces(straight.body)
                .into_iter()
                .filter(|f| !is_cap(rep, *f, &pull))
                .collect();
            let drafted = draft_faces(rep, straight.body, &sides, plane.origin, pull, taper)
                .map_err(|e| {
                    kernel(
                        "taper (only straight and circular walls can be tapered)",
                        &e,
                    )
                })?;
            steps.push((drafted, Vec::new()));
        }
    } else {
        if taper != 0.0 {
            return Err(EvalError::Kernel(
                "a taper cannot be combined with chosen regions".into(),
            ));
        }
        regions_steps(rep, sketch, plane, vector, &opts, regions, steps)?;
    }
    if let Some((t, side)) = thin {
        thin_steps(rep, plane, vector, &opts, t, side, steps)?;
    }
    Ok(())
}

/// The chosen regions of the sketch as solids: each region is the intersection of the extrusions of the
/// loops it lies inside, minus those of the loops that cut into it; the regions are then united.
fn regions_steps(
    rep: &mut BRep,
    sketch: &SketchProduct,
    plane: Frame3,
    vector: Vector3<f64>,
    opts: &ExtrudeOptions,
    wanted: &[Vec<u32>],
    steps: &mut Vec<(kw_brep::BuildResult, Vec<Vec<u32>>)>,
) -> Result<(), EvalError> {
    use crate::regions::{region_of, sketch_loops};
    let kernel = |what: &str, e: &dyn std::fmt::Display| EvalError::Kernel(format!("{what}: {e}"));
    let loops = sketch_loops(&sketch.sketch, &sketch.entities, plane)?;
    // one prism per loop that any wanted region uses
    let mut needed: Vec<usize> = Vec::new();
    let mut plan: Vec<(Vec<usize>, Vec<usize>)> = Vec::new();
    for sig in wanted {
        let (_, members, cut) = region_of(&loops, sig).ok_or_else(|| {
            EvalError::Kernel(
                "a chosen region no longer exists: the sketch changed, choose the regions again"
                    .into(),
            )
        })?;
        for i in members.iter().chain(cut.iter()) {
            if !needed.contains(i) {
                needed.push(*i);
            }
        }
        plan.push((members, cut));
    }
    let mut prism: HashMap<usize, BodyId> = HashMap::new();
    for i in &needed {
        let l = as_outer(&loops[*i].profile, &plane);
        let r = extrude(rep, &Profile::new(plane, l), vector, opts)
            .map_err(|e| kernel("extrude", &e))?;
        prism.insert(*i, r.body);
        steps.push((r, vec![loops[*i].labels.clone()]));
    }
    let bo = BoolOptions::default();
    let mut total: Option<BodyId> = None;
    for (members, cut) in plan {
        let mut acc = prism[&members[0]];
        let single = members.len() == 1 && cut.is_empty();
        for m in &members[1..] {
            let r = boolean(rep, acc, prism[m], Op::Intersect, &bo)
                .map_err(|e| kernel("region", &e))?;
            acc = r.body;
            steps.push((
                kw_brep::BuildResult {
                    body: r.body,
                    history: r.history,
                },
                Vec::new(),
            ));
        }
        for c in cut {
            let r = boolean(rep, acc, prism[&c], Op::Subtract, &bo)
                .map_err(|e| kernel("region", &e))?;
            acc = r.body;
            steps.push((
                kw_brep::BuildResult {
                    body: r.body,
                    history: r.history,
                },
                Vec::new(),
            ));
        }
        let _ = single;
        total = Some(match total {
            None => acc,
            Some(t) => {
                let r = boolean(rep, t, acc, Op::Union, &bo).map_err(|e| kernel("regions", &e))?;
                steps.push((
                    kw_brep::BuildResult {
                        body: r.body,
                        history: r.history,
                    },
                    Vec::new(),
                ));
                r.body
            }
        });
    }
    // the last step must be the tool: if no Boolean ran (one loop on its own), it is that loop's prism
    if let Some(t) = total {
        let last_is_total = steps.last().is_some_and(|(r, _)| r.body == t);
        if !last_is_total {
            let i = steps
                .iter()
                .position(|(r, _)| r.body == t)
                .ok_or_else(|| EvalError::Kernel("region bookkeeping failed".into()))?;
            let step = steps.remove(i);
            steps.push(step);
        }
    }
    Ok(())
}

/// Hollow the last solid into a wall along its profile: the caps are opened with `shell`; a wall outside
/// the profile (or centred on it) is the shell of the body grown by the wall (or half of it), cut back to
/// the original height.
fn thin_steps(
    rep: &mut BRep,
    plane: Frame3,
    vector: Vector3<f64>,
    opts: &ExtrudeOptions,
    t: f64,
    side: ThinSide,
    steps: &mut Vec<(kw_brep::BuildResult, Vec<Vec<u32>>)>,
) -> Result<(), EvalError> {
    let kernel = |what: &str, e: &dyn std::fmt::Display| EvalError::Kernel(format!("{what}: {e}"));
    if !(t > 0.0) {
        return Err(EvalError::Kernel(
            "the wall thickness must be positive".into(),
        ));
    }
    let pull = vector.normalize();
    let body = steps
        .last()
        .map(|(r, _)| r.body)
        .ok_or_else(|| EvalError::Kernel("nothing to thin".into()))?;
    let caps_of = |rep: &BRep, b: BodyId| -> Vec<FaceId> {
        rep.body_faces(b)
            .into_iter()
            .filter(|f| is_cap(rep, *f, &pull))
            .collect()
    };
    match side {
        ThinSide::Inside => {
            let caps = caps_of(rep, body);
            let r = shell(rep, body, t, &caps).map_err(|e| kernel("thin wall", &e))?;
            steps.push((r, Vec::new()));
        }
        ThinSide::Outside | ThinSide::Centered => {
            let grow = if side == ThinSide::Outside {
                t
            } else {
                t / 2.0
            };
            let big = offset_body(rep, body, grow).map_err(|e| kernel("thin wall", &e))?;
            let big_body = big.body;
            steps.push((big, Vec::new()));
            let caps = caps_of(rep, big_body);
            let wall = shell(rep, big_body, t, &caps).map_err(|e| kernel("thin wall", &e))?;
            let wall_body = wall.body;
            steps.push((wall, Vec::new()));
            // growing the body also grew it along the extrusion: cut that back with a slab of the original height
            let bb = rep.body_bbox(wall_body);
            let size = 4.0 * (bb.max - bb.min).norm() + 10.0;
            let slab = extrude(rep, &Profile::rectangle(plane, size, size), vector, opts)
                .map_err(|e| kernel("thin wall", &e))?;
            let slab_body = slab.body;
            steps.push((slab, Vec::new()));
            let r = boolean(
                rep,
                wall_body,
                slab_body,
                Op::Intersect,
                &BoolOptions::default(),
            )
            .map_err(|e| kernel("thin wall", &e))?;
            steps.push((
                kw_brep::BuildResult {
                    body: r.body,
                    history: r.history,
                },
                Vec::new(),
            ));
        }
    }
    Ok(())
}

/// A rib: every line and arc of the sketch becomes a strip `thickness` wide (in the sketch plane), and
/// the strips are extruded and united.
fn rib_steps(
    rep: &mut BRep,
    sketch: &SketchProduct,
    plane: Frame3,
    vector: Vector3<f64>,
    thickness: f64,
    symmetric: bool,
    steps: &mut Vec<(kw_brep::BuildResult, Vec<Vec<u32>>)>,
) -> Result<(), EvalError> {
    use kw_model::ProfileEdge;
    use kw_sketch::EntityKind;
    let kernel = |what: &str, e: &dyn std::fmt::Display| EvalError::Kernel(format!("{what}: {e}"));
    if !(thickness > 0.0) {
        return Err(EvalError::Kernel(
            "the rib thickness must be positive".into(),
        ));
    }
    let opts = ExtrudeOptions { symmetric };
    let w = |p: (f64, f64)| plane.origin + plane.x * p.0 + plane.y * p.1;
    let labels: HashMap<kw_sketch::EntityId, u32> = sketch
        .entities
        .iter()
        .enumerate()
        .map(|(i, e)| (*e, i as u32))
        .collect();
    let h = thickness / 2.0;
    let mut bodies: Vec<BodyId> = Vec::new();
    for (id, e) in &sketch.sketch.entities {
        if e.construction {
            continue;
        }
        let label = labels.get(&id).copied().unwrap_or(u32::MAX);
        let strip: Option<ProfileLoop> = match &e.kind {
            EntityKind::Line { p1, p2 } => {
                let (a, b) = (
                    sketch
                        .sketch
                        .point(*p1)
                        .map_err(|e| EvalError::Sketch(e.to_string()))?,
                    sketch
                        .sketch
                        .point(*p2)
                        .map_err(|e| EvalError::Sketch(e.to_string()))?,
                );
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let l = dx.hypot(dy);
                if l < 1e-9 {
                    None
                } else {
                    let n = (-dy / l * h, dx / l * h);
                    Some(ProfileLoop::polygon(&[
                        w((a.0 + n.0, a.1 + n.1)),
                        w((b.0 + n.0, b.1 + n.1)),
                        w((b.0 - n.0, b.1 - n.1)),
                        w((a.0 - n.0, a.1 - n.1)),
                    ]))
                }
            }
            EntityKind::Arc { .. } => {
                let (c, r, a0, a1) = kw_sketch::geom::arc_params(&sketch.sketch, id)
                    .map_err(|e| EvalError::Sketch(e.to_string()))?;
                if r <= h {
                    return Err(EvalError::Kernel(
                        "an arc of the rib is smaller than half its thickness".into(),
                    ));
                }
                let f = Frame3 {
                    origin: w(c),
                    ..plane
                };
                let pt = |rr: f64, a: f64| w((c.0 + rr * a.cos(), c.1 + rr * a.sin()));
                let mut inner = ProfileEdge::arc(&f, r - h, a0, a1);
                inner.reversed = true;
                Some(ProfileLoop::new(vec![
                    ProfileEdge::arc(&f, r + h, a0, a1),
                    ProfileEdge::line(pt(r + h, a1), pt(r - h, a1)),
                    inner,
                    ProfileEdge::line(pt(r - h, a0), pt(r + h, a0)),
                ]))
            }
            _ => None,
        };
        if let Some(l) = strip {
            let l = as_outer(&l, &plane);
            let n_edges = l.edges.len();
            let r = extrude(rep, &Profile::new(plane, l), vector, &opts)
                .map_err(|e| kernel("rib", &e))?;
            bodies.push(r.body);
            steps.push((r, vec![vec![label; n_edges]]));
        }
    }
    let Some(first) = bodies.first().copied() else {
        return Err(EvalError::Sketch(
            "a rib needs lines or arcs in its sketch".into(),
        ));
    };
    let mut acc = first;
    for b in &bodies[1..] {
        let r = boolean(rep, acc, *b, Op::Union, &BoolOptions::default())
            .map_err(|e| kernel("rib", &e))?;
        acc = r.body;
        steps.push((
            kw_brep::BuildResult {
                body: r.body,
                history: r.history,
            },
            Vec::new(),
        ));
    }
    // a single strip is already the last step; several end with their union
    let _ = acc;
    Ok(())
}

/// The names a tool's faces get in instance `index` of a pattern or mirror made by `feature`.
pub fn instance_names(
    built: &BuiltTool,
    feature: FeatureId,
    index: u32,
) -> HashMap<FaceId, TopoName> {
    built
        .names
        .iter()
        .map(|(f, n)| {
            (
                *f,
                TopoName {
                    feature,
                    origin: NameOrigin::Instance {
                        source: Box::new(n.clone()),
                        index,
                    },
                    ordinal: 0,
                },
            )
        })
        .collect()
}
