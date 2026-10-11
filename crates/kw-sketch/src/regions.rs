//! Closed regions of a sketch, for shading: the loops that close up, and which of them are holes.

use crate::curves::chain_segments;
use crate::entity::EntityKind;
use crate::geom;
use crate::sketch::Sketch;

pub type P2 = (f64, f64);

/// A shaded area: its outline and the outlines of the holes cut out of it.
#[derive(Debug, Clone, PartialEq)]
pub struct FillRegion {
    pub outer: Vec<P2>,
    pub holes: Vec<Vec<P2>>,
}

/// The closed outlines of the sketch (construction geometry and open curves are left out). `tol` is how
/// finely curves are approximated.
pub fn closed_loops(sk: &Sketch, tol: f64) -> Vec<Vec<P2>> {
    let mut loops: Vec<Vec<P2>> = Vec::new();
    let mut segs: Vec<(P2, P2)> = Vec::new();
    for (id, e) in sk.entities.iter() {
        if e.construction || matches!(e.kind, EntityKind::Point(_)) {
            continue;
        }
        let Ok(poly) = geom::polyline(sk, id, tol) else {
            continue;
        };
        if poly.len() < 2 {
            continue;
        }
        let (a, z) = (poly[0], poly[poly.len() - 1]);
        // a circle or an ellipse is one loop by itself
        if poly.len() > 3 && (a.0 - z.0).hypot(a.1 - z.1) < 1e-9 {
            loops.push(poly[..poly.len() - 1].to_vec());
        } else {
            segs.extend(poly.windows(2).map(|w| (w[0], w[1])));
        }
    }
    for chain in chain_segments(&segs, 1e-6) {
        let n = chain.len();
        if n > 3 && (chain[0].0 - chain[n - 1].0).hypot(chain[0].1 - chain[n - 1].1) < 1e-6 {
            loops.push(chain[..n - 1].to_vec());
        }
    }
    loops
}

/// The shaded areas: every region (face of the planar arrangement) of the sketch. Curves that cross cut each
/// other, so overlapping shapes and lines across a shape give several regions; a shape inside another is a hole
/// of it and a region of its own.
pub fn fill_regions(sk: &Sketch, tol: f64) -> Vec<FillRegion> {
    let arr = crate::arrangement::Arrangement::build(sk);
    arr.faces
        .iter()
        .map(|f| {
            let mut polys = arr.face_polygons(f, tol);
            let outer = polys.remove(0);
            FillRegion {
                outer,
                holes: polys,
            }
        })
        .collect()
}
