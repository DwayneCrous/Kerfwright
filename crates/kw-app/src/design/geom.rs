//! Plane geometry for sketching: frames of origin planes and planar faces, projection of body edges into a
//! plane, and sketches as 3-D polylines.

use crate::viewport::ModelScene;
use kw_brep::{BRep, FaceId};
use kw_expr::Param;
use kw_feature::{Document, Evaluation, FeatureId, FeatureSpec, PlaneSpec, Product};
use std::collections::HashSet;

/// An orthonormal plane frame: points are `origin + u x + v y`, `n = x × y`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneFrame {
    pub origin: [f64; 3],
    pub x: [f64; 3],
    pub y: [f64; 3],
    pub n: [f64; 3],
}

pub const ORIGIN_PLANES: [(&str, usize); 3] = [("XY", 0), ("XZ", 2), ("YZ", 1)];

/// The standard planes, as the engine defines them (`PlaneSpec::Xy/Yz/Xz`).
pub fn origin_frame(which: usize) -> PlaneFrame {
    match which {
        0 => PlaneFrame {
            origin: [0.0; 3],
            x: [1.0, 0.0, 0.0],
            y: [0.0, 1.0, 0.0],
            n: [0.0, 0.0, 1.0],
        },
        1 => PlaneFrame {
            origin: [0.0; 3],
            x: [0.0, 1.0, 0.0],
            y: [0.0, 0.0, 1.0],
            n: [1.0, 0.0, 0.0],
        },
        _ => PlaneFrame {
            origin: [0.0; 3],
            x: [0.0, 0.0, 1.0],
            y: [1.0, 0.0, 0.0],
            n: [0.0, 1.0, 0.0],
        },
    }
}

pub fn origin_spec(which: usize) -> PlaneSpec {
    match which {
        0 => PlaneSpec::Xy(Param::length(0.0)),
        1 => PlaneSpec::Yz(Param::length(0.0)),
        _ => PlaneSpec::Xz(Param::length(0.0)),
    }
}

/// The frame the engine gives a sketch on this planar face (`None` when the face is not planar).
pub fn face_frame(brep: &BRep, face: FaceId) -> Option<PlaneFrame> {
    let f = brep.faces.get(face)?;
    match &brep.surfaces[f.surface] {
        kw_nurbs::SurfaceGeom::Plane(pl) => {
            let s = if f.sense == kw_brep::Sense::Forward {
                1.0
            } else {
                -1.0
            };
            let fr = kw_feature::ops::canonical_plane_frame(pl.frame.z * s, pl.frame.origin);
            Some(PlaneFrame {
                origin: [fr.origin.x, fr.origin.y, fr.origin.z],
                x: [fr.x.x, fr.x.y, fr.x.z],
                y: [fr.y.x, fr.y.y, fr.y.z],
                n: [fr.z.x, fr.z.y, fr.z.z],
            })
        }
        _ => None,
    }
}

impl PlaneFrame {
    pub fn to_world(&self, u: f64, v: f64) -> [f64; 3] {
        [
            self.origin[0] + u * self.x[0] + v * self.y[0],
            self.origin[1] + u * self.x[1] + v * self.y[1],
            self.origin[2] + u * self.x[2] + v * self.y[2],
        ]
    }

    pub fn to_plane(&self, p: [f64; 3]) -> (f64, f64, f64) {
        let d = [
            p[0] - self.origin[0],
            p[1] - self.origin[1],
            p[2] - self.origin[2],
        ];
        let dot = |a: [f64; 3]| a[0] * d[0] + a[1] * d[1] + a[2] * d[2];
        (dot(self.x), dot(self.y), dot(self.n))
    }
}

/// Edges of the model lying in the plane, as 2-D polylines in plane coordinates (reference geometry).
pub fn project_edges(model: &ModelScene, pf: &PlaneFrame) -> Vec<Vec<(f64, f64)>> {
    model
        .edge_polylines()
        .into_iter()
        .filter_map(|poly| {
            let pts: Vec<(f64, f64, f64)> = poly.iter().map(|p| pf.to_plane(*p)).collect();
            if pts.iter().all(|p| p.2.abs() < 1e-6) {
                Some(pts.iter().map(|p| (p.0, p.1)).collect())
            } else {
                None
            }
        })
        .collect()
}

/// The visible sketches as 3-D line segments (world coordinates).
pub fn sketch_segments(
    ev: &Evaluation,
    doc: &Document,
    hidden: &HashSet<FeatureId>,
) -> Vec<([f64; 3], [f64; 3])> {
    let mut out = Vec::new();
    for f in &doc.features {
        if !matches!(f.spec, FeatureSpec::Sketch(_)) || hidden.contains(&f.id) {
            continue;
        }
        let Some(o) = ev.output(f.id) else { continue };
        let Product::Sketch(sp) = &o.product else {
            continue;
        };
        let fr = &sp.plane;
        let w = |p: (f64, f64)| -> [f64; 3] {
            [
                fr.origin.x + p.0 * fr.x.x + p.1 * fr.y.x,
                fr.origin.y + p.0 * fr.x.y + p.1 * fr.y.y,
                fr.origin.z + p.0 * fr.x.z + p.1 * fr.y.z,
            ]
        };
        for (id, ent) in &sp.sketch.entities {
            // points and construction geometry (the diagonals of a centre rectangle) are not drawn in 3-D
            if ent.construction || matches!(ent.kind, kw_sketch::EntityKind::Point(_)) {
                continue;
            }
            if let Ok(poly) = kw_sketch::geom::polyline(&sp.sketch, id, 0.05) {
                for pair in poly.windows(2) {
                    out.push((w(pair[0]), w(pair[1])));
                }
            }
        }
    }
    out
}

/// A rough size for the origin planes: a bit larger than the model.
pub fn plane_size(radius: f64) -> f64 {
    (radius * 1.1).max(20.0)
}
