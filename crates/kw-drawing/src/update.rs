//! Making and refreshing the geometry of views: what to project, at what scale, and a hash of
//! the source so a drawing can tell when the model has moved on.

use crate::hlr::{project_bodies, HlrOptions, ViewGeometry};
use crate::model::ViewStyle;
use crate::projection::ViewFrame;
use crate::scene::P;
use kw_brep::{BRep, BodyId};

/// What to draw for one view.
#[derive(Debug, Clone)]
pub struct GenSpec {
    pub id: u64,
    pub frame: ViewFrame,
    pub scale: f64,
    pub style: ViewStyle,
}

/// The drawn geometry of each spec, in order. Fails with the first error.
pub fn generate(
    brep: &BRep,
    bodies: &[BodyId],
    specs: &[GenSpec],
) -> Result<Vec<(u64, ViewGeometry)>, String> {
    specs
        .iter()
        .map(|s| {
            project_bodies(
                brep,
                bodies,
                &HlrOptions {
                    frame: s.frame,
                    scale: s.scale,
                    style: s.style,
                },
            )
            .map(|g| (s.id, g))
        })
        .collect()
}

/// Width and height of the bodies' bounding box seen along `frame`, in model units. This is the
/// box, so a little more than the outline of a rounded part, which is fine for choosing a scale.
pub fn extent(brep: &BRep, bodies: &[BodyId], frame: &ViewFrame) -> P {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for b in bodies {
        let bb = brep.body_bbox(*b);
        for k in 0..8 {
            let p = [
                if k & 1 == 0 { bb.min.x } else { bb.max.x },
                if k & 2 == 0 { bb.min.y } else { bb.max.y },
                if k & 4 == 0 { bb.min.z } else { bb.max.z },
            ];
            let (q, _) = frame.project(p);
            for i in 0..2 {
                lo[i] = lo[i].min(q[i]);
                hi[i] = hi[i].max(q[i]);
            }
        }
    }
    if lo[0].is_finite() {
        [hi[0] - lo[0], hi[1] - lo[1]]
    } else {
        [0.0, 0.0]
    }
}

/// The preferred drawing scales (sheet mm per model unit), largest last.
pub const STANDARD_SCALES: [f64; 17] = [
    1.0 / 100.0,
    1.0 / 50.0,
    1.0 / 20.0,
    1.0 / 10.0,
    1.0 / 5.0,
    1.0 / 2.0,
    1.0,
    2.0,
    5.0,
    10.0,
    20.0,
    50.0,
    100.0,
    200.0,
    500.0,
    1000.0,
    10000.0,
];

/// The largest standard scale at which `extent` still fits in `avail`.
pub fn auto_scale(extent: P, avail: P) -> f64 {
    let fits = |s: f64| extent[0] * s <= avail[0] && extent[1] * s <= avail[1];
    let mut best = STANDARD_SCALES[0];
    for s in STANDARD_SCALES {
        if fits(s) {
            best = s;
        }
    }
    best
}

/// A stable hash of bytes, for the source hash of a view.
pub fn hash_bytes(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Fold a 128-bit hash (as the feature document gives) to 64 bits.
pub fn fold(h: u128) -> u64 {
    (h as u64) ^ ((h >> 64) as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_scale_picks_the_largest_standard_scale_that_fits() {
        assert_eq!(auto_scale([100.0, 50.0], [150.0, 120.0]), 1.0);
        assert_eq!(auto_scale([100.0, 50.0], [60.0, 60.0]), 0.5);
        assert_eq!(auto_scale([10.0, 10.0], [100.0, 100.0]), 10.0);
        assert_eq!(auto_scale([1e9, 1e9], [10.0, 10.0]), 0.01);
    }
}
