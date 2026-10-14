//! A quick look at the limit surface of a cage: a few Catmull–Clark steps, the exact limit positions of the
//! vertices of the last step, and two triangles per quad. Fast enough to follow a vertex dragged in the
//! viewport (it does not fit patches: that is what [`crate::finish`] does when the form is accepted).

use crate::{Level, SubdError, SubdMesh};

/// A triangle mesh of the surface, counter-clockwise seen from outside.
#[derive(Debug, Clone, Default)]
pub struct PreviewMesh {
    pub positions: Vec<[f64; 3]>,
    pub tris: Vec<[u32; 3]>,
}

/// The surface of `cage` (positions multiplied by `scale`) after `levels` subdivision steps (at least one:
/// the first step turns every face into quads). A cage that faces inwards is turned the right way round.
pub fn fast_mesh(cage: &SubdMesh, scale: f64, levels: usize) -> Result<PreviewMesh, SubdError> {
    cage.validate()?;
    let cage = if cage.signed_volume() < 0.0 && cage.is_closed() {
        cage.flipped()
    } else {
        cage.clone()
    };
    let mut level = Level::from_mesh(&cage);
    for _ in 0..levels.max(1) {
        level = level.subdivide();
    }
    let positions: Vec<[f64; 3]> = level
        .limit()
        .into_iter()
        .map(|p| [p[0] * scale, p[1] * scale, p[2] * scale])
        .collect();
    let mut tris = Vec::with_capacity(level.faces.len() * 2);
    for q in &level.faces {
        tris.push([q[0], q[1], q[2]]);
        tris.push([q[0], q[2], q[3]]);
    }
    Ok(PreviewMesh { positions, tris })
}

/// How many subdivision steps keep the preview light: about 20 000 triangles at most.
pub fn levels_for(cage: &SubdMesh) -> usize {
    let mut quads = cage.faces.iter().map(|f| f.len()).sum::<usize>().max(1);
    let mut levels = 1;
    while levels < 4 && quads * 4 <= 10_000 {
        quads *= 4;
        levels += 1;
    }
    levels
}
