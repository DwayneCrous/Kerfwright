//! Tool bodies: the solids that an Extrude, Revolve or Hole adds to or removes from the body, described
//! by how they are built so that a pattern or mirror can rebuild them somewhere else and name their
//! faces after the faces of the original.

use crate::eval::{EvalError, SketchProduct};
use crate::id::FeatureId;
use crate::namer::Namer;
use crate::naming::{NameOrigin, TopoName};
use crate::profile::sketch_profile_labeled;
use kw_brep::{make_cone, BRep, BodyId, EntityRef, FaceId, ShapeHistory};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::{draft_faces, extrude, revolve, ExtrudeOptions, Profile, ProfileLoop};
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
            } => ToolShape::Extrude {
                sketch: sketch.clone(),
                sketch_id: *sketch_id,
                plane: place.frame(plane),
                vector: place.vector(vector),
                symmetric: *symmetric,
                taper: *taper,
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
        let mut labels: Vec<Vec<u32>> = Vec::new();
        let mut sketch_id = feature;
        let mut swept_role = "side";
        let mut first: Option<kw_brep::BuildResult> = None;
        let result = match &self.shape {
            ToolShape::Extrude {
                sketch,
                sketch_id: sid,
                plane,
                vector,
                symmetric,
                taper,
            } => {
                sketch_id = *sid;
                let lp = sketch_profile_labeled(&sketch.sketch, &sketch.entities, *plane)?;
                labels = lp.labels;
                let straight = extrude(
                    &mut rep,
                    &lp.profile,
                    *vector,
                    &ExtrudeOptions {
                        symmetric: *symmetric,
                    },
                )
                .map_err(|e| kernel("extrude", &e))?;
                if *taper != 0.0 {
                    if *symmetric {
                        return Err(EvalError::Kernel(
                            "a taper cannot be combined with a symmetric extent".into(),
                        ));
                    }
                    let pull = vector.normalize();
                    // every side face leans; the two caps (perpendicular to the extrusion) stay put
                    let sides: Vec<FaceId> = rep
                        .body_faces(straight.body)
                        .into_iter()
                        .filter(|f| match &rep.surfaces[rep.faces[*f].surface] {
                            kw_nurbs::SurfaceGeom::Plane(p) => {
                                p.frame.z.dot(&pull).abs() < 1.0 - 1e-9
                            }
                            _ => true,
                        })
                        .collect();
                    let drafted =
                        draft_faces(&mut rep, straight.body, &sides, plane.origin, pull, *taper)
                            .map_err(|e| {
                                kernel(
                                    "taper (only straight and circular walls can be tapered)",
                                    &e,
                                )
                            })?;
                    first = Some(straight);
                    drafted
                } else {
                    straight
                }
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
                labels = lp.labels;
                revolve(&mut rep, &lp.profile, *axis_origin, *axis, *angle)
                    .map_err(|e| kernel("revolve", &e))?
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
                extrude(
                    &mut rep,
                    &profile,
                    frame.z * *height,
                    &ExtrudeOptions::default(),
                )
                .map_err(|e| kernel("extrude", &e))?
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
                make_cone(&mut rep, frame, *r0, *r1, *height)
            }
        };
        let new_origin = |role: &str| -> NameOrigin {
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
        let mut namer = Namer::new(feature);
        let named = match first {
            // a tapered extrude: name the straight prism, then carry its names through the taper
            Some(f) => {
                let n1 = namer.name_result(&rep, f.body, &f.history, &HashMap::new(), &new_origin);
                namer.name_result(&rep, result.body, &result.history, &n1, &new_origin)
            }
            None => namer.name_result(
                &rep,
                result.body,
                &result.history,
                &HashMap::new(),
                &new_origin,
            ),
        };
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
