//! Which body of the model shows at a point of a view.

use crate::projection::{ViewFrame, V3};
use crate::scene::P;
use kw_brep::{BRep, BodyId};
use kw_tess::{tessellate_body, TessOptions};

/// The index (in `bodies`) of the body whose surface is nearest the eye at the point `uv` of the
/// view (model units, as `f.project` gives them), with that point of its surface.
pub fn body_at(brep: &BRep, bodies: &[BodyId], f: &ViewFrame, uv: P) -> Option<(usize, V3)> {
    let mut best: Option<(usize, f64)> = None;
    let mut diag = 0.0_f64;
    for b in bodies {
        let bb = brep.body_bbox(*b);
        diag = diag.max((bb.max - bb.min).norm());
    }
    let opts = TessOptions {
        chord_tol: (diag * 1e-3).max(1e-6),
        angle_tol: 0.2,
    };
    for (bi, b) in bodies.iter().enumerate() {
        let Ok(t) = tessellate_body(brep, *b, &opts) else {
            continue;
        };
        for tri in 0..t.mesh.triangle_count() {
            let c = t.mesh.triangle(tri);
            let p: Vec<([f64; 2], f64)> = c.iter().map(|q| f.project([q.x, q.y, q.z])).collect();
            let area = (p[1].0[0] - p[0].0[0]) * (p[2].0[1] - p[0].0[1])
                - (p[2].0[0] - p[0].0[0]) * (p[1].0[1] - p[0].0[1]);
            if area.abs() < 1e-14 {
                continue;
            }
            let w0 = ((p[1].0[0] - uv[0]) * (p[2].0[1] - uv[1])
                - (p[2].0[0] - uv[0]) * (p[1].0[1] - uv[1]))
                / area;
            let w1 = ((p[2].0[0] - uv[0]) * (p[0].0[1] - uv[1])
                - (p[0].0[0] - uv[0]) * (p[2].0[1] - uv[1]))
                / area;
            let w2 = 1.0 - w0 - w1;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let depth = w0 * p[0].1 + w1 * p[1].1 + w2 * p[2].1;
            if best.is_none_or(|(_, d)| depth < d) {
                best = Some((bi, depth));
            }
        }
    }
    best.map(|(bi, depth)| {
        let world = [
            f.right[0] * uv[0] + f.up[0] * uv[1] + f.dir[0] * depth,
            f.right[1] * uv[0] + f.up[1] * uv[1] + f.dir[1] * depth,
            f.right[2] * uv[0] + f.up[2] * uv[1] + f.dir[2] * depth,
        ];
        (bi, world)
    })
}
