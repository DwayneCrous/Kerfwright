//! What a drawing view looks at: the design's solid bodies, or the posed assembly.

use crate::bodies::Bodies;
use crate::project::Project;
use kw_brep::{BRep, BodyId};
use kw_drawing::update::{fold, hash_bytes};
use kw_drawing::ViewSource;
use std::collections::HashSet;
use std::sync::Arc;

pub struct Source {
    /// For a design: its bodies with their persistent names, which dimensions attach through.
    pub design: Option<Arc<Bodies>>,
    pub brep: Arc<BRep>,
    pub bodies: Vec<BodyId>,
    /// Changes when the model the views are made from changes.
    pub hash: u64,
}

pub fn source_for(project: &mut Project, which: &ViewSource) -> Result<Source, String> {
    match which {
        ViewSource::Design => {
            let doc = project.document();
            let ev = project.evaluation();
            let bodies = Bodies::new(&doc, &ev, &HashSet::new())
                .ok_or("the design has no solid body to draw")?;
            let ids: Vec<BodyId> = bodies.brep.bodies.keys().collect();
            Ok(Source {
                brep: bodies.brep.clone(),
                bodies: ids,
                hash: fold(doc.content_hash()),
                design: Some(Arc::new(bodies)),
            })
        }
        ViewSource::Assembly => {
            let asm = project
                .assembly
                .as_ref()
                .ok_or("the project has no assembly")?;
            let hash = hash_bytes(&asm.to_bytes());
            let pose = asm
                .pose(&mut project.engine)
                .map_err(|e| format!("the assembly could not be posed: {e:?}"))?;
            Ok(Source {
                design: None,
                bodies: pose.bodies.clone(),
                brep: Arc::new(pose.rep),
                hash,
            })
        }
        ViewSource::Component(_) => Err("drawing a single component is not available yet".into()),
    }
}

/// Is the model different from what these views were last made from?
pub fn current_hash(project: &mut Project, which: &ViewSource) -> Option<u64> {
    match which {
        ViewSource::Design => Some(fold(project.document().content_hash())),
        ViewSource::Assembly => project.assembly.as_ref().map(|a| hash_bytes(&a.to_bytes())),
        ViewSource::Component(_) => None,
    }
}
