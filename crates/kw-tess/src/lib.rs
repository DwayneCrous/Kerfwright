//! Edge-first, crack-free B-rep tessellation.
//!
//! Every B-rep edge is discretised exactly once; every face reuses those
//! samples for its boundary, so neighbouring faces agree bit-for-bit on
//! shared edges. Face interiors are triangulated in parameter space with a
//! constrained Delaunay triangulation (`spade`), honouring seams, poles
//! (degenerate edges) and inner loops.

// The algorithms follow the index notation of the cited literature (Piegl & Tiller etc.);
// rewriting those loops as iterators would obscure the correspondence.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments)]

mod edges;
mod face;
mod mesh;

pub use mesh::{signed_volume, EdgePolylines, Mesh, Tessellation};

use kw_brep::{BRep, BodyId, EdgeId, FaceId, ValidationError};

#[derive(Debug, Clone)]
pub struct TessOptions {
    /// Maximum distance between the mesh and the true geometry (model units).
    pub chord_tol: f64,
    /// Maximum turning angle of the surface normal across one triangle edge /
    /// of the tangent across one polyline segment (radians).
    pub angle_tol: f64,
}

impl Default for TessOptions {
    fn default() -> Self {
        Self {
            chord_tol: 0.01,
            angle_tol: 20.0_f64.to_radians(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum TessError {
    MissingPcurve(String),
    BadTopology(String),
    Triangulation(String),
}

impl std::fmt::Display for TessError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for TessError {}

/// Tessellate every face of `body` (and the body's edges).
pub fn tessellate_body(
    brep: &BRep,
    body: BodyId,
    opts: &TessOptions,
) -> Result<Tessellation, TessError> {
    let faces = brep.body_faces(body);
    tessellate_faces(brep, &faces, opts)
}

/// Tessellate every body in the B-rep.
pub fn tessellate_brep(brep: &BRep, opts: &TessOptions) -> Result<Tessellation, TessError> {
    let faces: Vec<FaceId> = brep.faces.keys().collect();
    tessellate_faces(brep, &faces, opts)
}

fn tessellate_faces(
    brep: &BRep,
    faces: &[FaceId],
    opts: &TessOptions,
) -> Result<Tessellation, TessError> {
    use rayon::prelude::*;
    let edge_keys: Vec<EdgeId> = {
        let mut seen = std::collections::HashSet::new();
        let mut out = Vec::new();
        for f in faces {
            for l in &brep.faces[*f].loops {
                for c in brep.loop_coedges(*l) {
                    let e = brep.coedges[c].edge;
                    if seen.insert(e) {
                        out.push(e);
                    }
                }
            }
        }
        out
    };
    let ctx = edges::EdgeContext::build(brep, &edge_keys, opts)?;
    let face_meshes: Vec<Result<face::FaceMesh, TessError>> = faces
        .par_iter()
        .map(|f| face::mesh_face(brep, &ctx, *f, opts))
        .collect();
    let mut tess = Tessellation::new(
        faces.to_vec(),
        ctx.polylines(&edge_keys),
        edge_keys,
        ctx.next_weld,
    );
    for (i, fm) in face_meshes.into_iter().enumerate() {
        tess.mesh.append_face(fm?, i as u32);
    }
    Ok(tess)
}

/// Full validation of a solid: B-rep topology + geometry, tessellation,
/// positive enclosed volume, and watertightness of the mesh.
pub fn validate_solid_full(
    brep: &BRep,
    body: BodyId,
    opts: &TessOptions,
) -> Result<Tessellation, Vec<ValidationError>> {
    let mut errors = match brep.validate().into_result() {
        Ok(_) => Vec::new(),
        Err(e) => e,
    };
    match tessellate_body(brep, body, opts) {
        Ok(t) => {
            let vol = t.mesh.signed_volume();
            if vol.is_nan() || vol <= 0.0 {
                errors.push(ValidationError::NonPositiveVolume(vol));
            }
            if let Some(bad) = t.mesh.first_unmatched_edge() {
                errors.push(ValidationError::Geometry {
                    what: "mesh edge not shared by exactly two triangles",
                    id: format!("{bad:?}"),
                    deviation: 0.0,
                });
            }
            if errors.is_empty() {
                return Ok(t);
            }
        }
        Err(e) => errors.push(ValidationError::Geometry {
            what: "tessellation failed",
            id: e.to_string(),
            deviation: 0.0,
        }),
    }
    Err(errors)
}
