//! Colour ramps for the scalar overlays. Piecewise linear in sRGB with a handful of stops, so the
//! shader repeats them exactly.

/// Stops of the sequential ramp (low to high): blue, cyan, green, yellow, red.
pub const SEQUENTIAL: [[f32; 3]; 5] = [
    [0.17, 0.23, 0.62],
    [0.18, 0.66, 0.88],
    [0.31, 0.78, 0.40],
    [0.95, 0.82, 0.24],
    [0.84, 0.21, 0.17],
];

/// Stops of the diverging ramp (negative, zero, positive): blue, white, red.
pub const DIVERGING: [[f32; 3]; 3] = [[0.18, 0.37, 0.83], [0.95, 0.95, 0.95], [0.83, 0.20, 0.18]];

fn along(stops: &[[f32; 3]], t: f32) -> [f32; 3] {
    let t = t.clamp(0.0, 1.0) * (stops.len() - 1) as f32;
    let i = (t.floor() as usize).min(stops.len() - 2);
    let f = t - i as f32;
    let (a, b) = (stops[i], stops[i + 1]);
    [
        a[0] + (b[0] - a[0]) * f,
        a[1] + (b[1] - a[1]) * f,
        a[2] + (b[2] - a[2]) * f,
    ]
}

/// The sequential ramp at `t` in 0..=1.
pub fn sequential(t: f32) -> [f32; 3] {
    along(&SEQUENTIAL, t)
}

/// The diverging ramp at `t` in -1..=1 (0 is the neutral middle).
pub fn diverging(t: f32) -> [f32; 3] {
    along(&DIVERGING, 0.5 + 0.5 * t.clamp(-1.0, 1.0))
}

/// Draft analysis colours: faces with too little draft (|angle| below `limit`) are red, faces
/// that open the right way green, the wrong way blue, and walls just short of the limit orange.
pub fn draft(angle_deg: f32, limit_deg: f32) -> [f32; 3] {
    let a = angle_deg;
    if a.abs() < limit_deg {
        [0.84, 0.21, 0.17]
    } else if a > 0.0 {
        let t = ((a - limit_deg) / (90.0 - limit_deg).max(1e-3)).clamp(0.0, 1.0);
        [0.30 - 0.12 * t, 0.72 - 0.10 * t, 0.40 + 0.2 * t]
    } else {
        let t = ((-a - limit_deg) / (90.0 - limit_deg).max(1e-3)).clamp(0.0, 1.0);
        [0.30 - 0.12 * t, 0.42 - 0.1 * t, 0.85 - 0.2 * t]
    }
}
