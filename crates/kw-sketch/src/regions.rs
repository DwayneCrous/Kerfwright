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

fn inside(poly: &[P2], p: P2) -> bool {
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        j = i;
    }
    inside
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

/// The shaded areas: every closed loop that is not inside an odd number of others, with the loops
/// directly inside it as holes. Loops that overlap without containing one another are both shaded.
pub fn fill_regions(sk: &Sketch, tol: f64) -> Vec<FillRegion> {
    let loops = closed_loops(sk, tol);
    let n = loops.len();
    // loop i is inside loop j when a vertex of i is
    let within = |i: usize, j: usize| {
        i != j && inside(&loops[j], loops[i][0]) && inside(&loops[j], loops[i][loops[i].len() / 2])
    };
    let depth: Vec<usize> = (0..n)
        .map(|i| (0..n).filter(|j| within(i, *j)).count())
        .collect();
    let mut out = Vec::new();
    for i in 0..n {
        if depth[i] % 2 == 1 {
            continue;
        }
        let holes = (0..n)
            .filter(|j| depth[*j] == depth[i] + 1 && within(*j, i))
            .map(|j| loops[j].clone())
            .collect();
        out.push(FillRegion {
            outer: loops[i].clone(),
            holes,
        });
    }
    out
}
