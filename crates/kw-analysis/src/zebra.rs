//! Zebra stripes: what a striped environment looks like reflected in the surface. Where the
//! surface is tangent-continuous the stripes run on across the join; where it is only
//! position-continuous they break.
//!
//! The viewer evaluates exactly this in its fragment shader.

/// Reflect the view direction `v` (from the eye toward the surface) in the unit normal `n`.
pub fn reflect(v: [f32; 3], n: [f32; 3]) -> [f32; 3] {
    let d = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
    [
        v[0] - 2.0 * d * n[0],
        v[1] - 2.0 * d * n[1],
        v[2] - 2.0 * d * n[2],
    ]
}

/// 1 (white) or 0 (black) for a reflection direction `r`: the environment is striped along `axis`
/// (a unit vector) with `freq` stripe pairs across the unit range, white for the fraction `duty`.
pub fn stripe(r: [f32; 3], axis: [f32; 3], freq: f32, duty: f32) -> f32 {
    let c = r[0] * axis[0] + r[1] * axis[1] + r[2] * axis[2];
    let f = c * freq;
    let frac = f - f.floor();
    if frac < duty {
        1.0
    } else {
        0.0
    }
}

/// The stripe at a surface point with unit normal `n` seen along `view`.
pub fn zebra(view: [f32; 3], n: [f32; 3], axis: [f32; 3], freq: f32, duty: f32) -> f32 {
    stripe(reflect(view, n), axis, freq, duty)
}
