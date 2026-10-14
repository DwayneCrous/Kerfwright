//! The bodies a simulation or a machining setup works on: the design's solid bodies, or the posed
//! assembly. Both the FEA and CAM workspaces take their input from here instead of the last solid.
//!
//! Every body is addressed by a [`BodyKey`] that survives edits (see `kw-store`), carries its material
//! name and colour, and says which B-rep and body to look at. Faces of the bodies are addressed as
//! `(body index, FaceId of that B-rep)`.

use crate::bodies::Bodies;
use crate::project::Project;
use kw_brep::{BRep, BodyId, CopyOptions, FaceId};
use kw_feature::FeatureId;
use kw_math::Transform3;
use kw_store::{BodyKey, SourceKind};
use std::collections::HashSet;
use std::sync::Arc;

/// One body of the model.
#[derive(Debug, Clone)]
pub struct SourceBody {
    pub key: BodyKey,
    pub name: String,
    /// The B-rep that holds the body (shared by all bodies of an assembly pose).
    pub brep: Arc<BRep>,
    pub body: BodyId,
    /// Where the body was placed by the assembly (identity for a design body). The B-rep of an assembly
    /// body is already in the world frame; this says how it got there.
    pub placement: Transform3,
    pub material: Option<String>,
    pub colour: [u8; 3],
    /// mm³
    pub volume: f64,
}

#[derive(Debug, Clone)]
pub struct ModelSource {
    pub kind: SourceKind,
    pub bodies: Vec<SourceBody>,
    /// Changes whenever any body's geometry, name or material changes.
    pub key: u128,
}

const DEFAULT_COLOUR: [u8; 3] = [196, 200, 208];

/// The chosen bodies copied into one B-rep, in the order of the source (the order that face tags follow).
pub struct Combined {
    pub brep: Arc<BRep>,
    pub bodies: Vec<(BodyKey, BodyId)>,
    /// Index into the source of every body.
    pub source_index: Vec<usize>,
}

impl ModelSource {
    /// The model of `project` from the chosen source. `hidden` lists design bodies (by root feature) that
    /// are left out. `Err` says why there is nothing to work on.
    pub fn of(
        project: &mut Project,
        kind: SourceKind,
        hidden: &HashSet<FeatureId>,
    ) -> Result<ModelSource, String> {
        match kind {
            SourceKind::Design => ModelSource::design(project, hidden),
            SourceKind::Assembly => ModelSource::assembly(project),
        }
    }

    /// The solid bodies of the design (surfaces are not simulated or machined).
    pub fn design(
        project: &mut Project,
        hidden: &HashSet<FeatureId>,
    ) -> Result<ModelSource, String> {
        let doc = project.document();
        let ev = project.evaluation();
        let bodies = Bodies::new(&doc, &ev, hidden).ok_or("the design has no solid body")?;
        let mut out = Vec::new();
        for b in &bodies.list {
            if b.output.is_sheet() {
                continue;
            }
            let (Some(brep), Some(body)) = (b.output.brep().cloned(), b.output.body()) else {
                continue;
            };
            let volume = brep.volume(body).unwrap_or(0.0);
            out.push(SourceBody {
                key: BodyKey::Design(b.root),
                name: b.name.clone(),
                brep,
                body,
                placement: Transform3::identity(),
                material: b.material.clone(),
                colour: b.colour,
                volume,
            });
        }
        if out.is_empty() {
            return Err("the design has no solid body (surfaces are not used)".into());
        }
        Ok(ModelSource {
            kind: SourceKind::Design,
            bodies: out,
            key: bodies.key,
        })
    }

    /// The bodies of the posed assembly, one per solid body of every visible component.
    pub fn assembly(project: &mut Project) -> Result<ModelSource, String> {
        let asm = project
            .assembly
            .as_ref()
            .ok_or("the project has no assembly")?;
        let mut h = kw_feature::Hasher128::new();
        h.bytes(&asm.to_bytes());
        let key = h.finish();
        let pose = asm
            .pose(&mut project.engine)
            .map_err(|e| format!("the assembly could not be posed: {e}"))?;
        if pose.bodies.is_empty() {
            return Err("the assembly has no visible component".into());
        }
        let rep = Arc::new(pose.rep);
        let bodies = pose
            .bodies
            .iter()
            .enumerate()
            .map(|(i, body)| SourceBody {
                key: BodyKey::Assembly(pose.names[i].clone()),
                name: pose.names[i].clone(),
                brep: rep.clone(),
                body: *body,
                placement: pose.placements[i],
                material: pose.materials[i].clone(),
                colour: DEFAULT_COLOUR,
                volume: pose.volumes[i],
            })
            .collect();
        Ok(ModelSource {
            kind: SourceKind::Assembly,
            bodies,
            key,
        })
    }

    pub fn index_of(&self, key: &BodyKey) -> Option<usize> {
        self.bodies.iter().position(|b| &b.key == key)
    }

    pub fn find(&self, key: &BodyKey) -> Option<&SourceBody> {
        self.index_of(key).map(|i| &self.bodies[i])
    }

    /// The body that owns `face` of the B-rep `brep` (compared by identity of the shared B-rep).
    pub fn body_of_face(&self, brep: &Arc<BRep>, face: FaceId) -> Option<usize> {
        self.bodies
            .iter()
            .position(|b| Arc::ptr_eq(&b.brep, brep) && b.brep.body_faces(b.body).contains(&face))
    }

    /// The faces of body `index`.
    pub fn faces_of(&self, index: usize) -> Vec<FaceId> {
        let b = &self.bodies[index];
        b.brep.body_faces(b.body)
    }

    /// The bodies for which `include` holds, copied into one B-rep. A single body is used as it is.
    pub fn combined(&self, include: &dyn Fn(&BodyKey) -> bool) -> Combined {
        let chosen: Vec<usize> = (0..self.bodies.len())
            .filter(|i| include(&self.bodies[*i].key))
            .collect();
        if let [only] = chosen[..] {
            let b = &self.bodies[only];
            // a body that shares its B-rep with others (an assembly pose) is copied out of it
            if b.brep.bodies.len() == 1 {
                return Combined {
                    brep: b.brep.clone(),
                    bodies: vec![(b.key.clone(), b.body)],
                    source_index: vec![only],
                };
            }
        }
        let mut rep = BRep::new();
        let mut bodies = Vec::new();
        for &i in &chosen {
            let b = &self.bodies[i];
            let (id, _) = rep.copy_body_from(&b.brep, b.body, &CopyOptions::default());
            bodies.push((b.key.clone(), id));
        }
        Combined {
            brep: Arc::new(rep),
            bodies,
            source_index: chosen,
        }
    }
}
