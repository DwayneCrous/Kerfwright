//! Draft analysis: the angle between a face and the direction a mould opens in.

use kw_tess::Tessellation;

/// The draft angle in degrees of a surface with unit normal `n` for the pull direction `pull`:
/// 0 for a wall parallel to the pull (it needs draft), +90 for a face looking along the pull and
/// -90 for one looking against it. Positive angles open in the pull direction.
pub fn draft_degrees(n: [f64; 3], pull: [f64; 3]) -> f64 {
    let pl = (pull[0] * pull[0] + pull[1] * pull[1] + pull[2] * pull[2]).sqrt();
    let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if pl < 1e-300 || nl < 1e-300 {
        return 0.0;
    }
    let c = (n[0] * pull[0] + n[1] * pull[1] + n[2] * pull[2]) / (pl * nl);
    c.clamp(-1.0, 1.0).asin().to_degrees()
}

/// The draft angle at every vertex of a tessellation, from the vertex normals.
pub fn mesh_draft(t: &Tessellation, pull: [f64; 3]) -> Vec<f32> {
    t.mesh
        .normals
        .iter()
        .map(|n| draft_degrees([n.x, n.y, n.z], pull) as f32)
        .collect()
}
