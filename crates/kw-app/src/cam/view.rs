//! What the CAM viewport draws: toolpath segments and the simulated stock coloured by its distance from the part.

use super::*;

/// Cutting moves of a toolpath (arcs chorded) for display.
pub fn cut_segments(tp: &Toolpath) -> Vec<([f64; 3], [f64; 3])> {
    let mut out = Vec::new();
    let mut pos: Option<[f64; 3]> = None;
    for e in &tp.events {
        match e {
            Event::Rapid { p } | Event::Pose { p, feed: None, .. } => pos = Some(*p),
            Event::Feed { p, .. }
            | Event::Pose {
                p, feed: Some(_), ..
            } => {
                if let Some(q) = pos {
                    out.push((q, *p));
                }
                pos = Some(*p);
            }
            Event::Arc { p, .. } => {
                if let Some(q) = pos {
                    let mut mini = Toolpath::new("arc");
                    mini.push(Event::Rapid { p: q });
                    mini.push(e.clone());
                    out.extend(moves(&mini));
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    out
}

/// Colour of a stock facet by its distance from the part: green where it is on the part, orange to red where
/// material is left, blue where the part is cut too deep.
pub fn deviation_colour(d: f32) -> [u8; 4] {
    let lerp = |a: [f32; 3], b: [f32; 3], t: f32| {
        let t = t.clamp(0.0, 1.0);
        [
            (a[0] + (b[0] - a[0]) * t) as u8,
            (a[1] + (b[1] - a[1]) * t) as u8,
            (a[2] + (b[2] - a[2]) * t) as u8,
            255,
        ]
    };
    if d.abs() <= 0.05 {
        [70, 180, 100, 255]
    } else if d > 0.0 {
        lerp([240.0, 190.0, 70.0], [200.0, 40.0, 40.0], d / 2.0)
    } else {
        lerp([100.0, 160.0, 230.0], [30.0, 40.0, 150.0], -d)
    }
}

/// The scene of a simulated stock in world coordinates.
pub fn stock_scene(sim: &SimResult) -> ModelScene {
    use kw_viewer::scene::{Scene, Vertex};
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for t in &sim.world {
        for p in t {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    if sim.world.is_empty() {
        return ModelScene::empty();
    }
    let c = [
        (lo[0] + hi[0]) / 2.0,
        (lo[1] + hi[1]) / 2.0,
        (lo[2] + hi[2]) / 2.0,
    ];
    let radius =
        0.5 * ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let mut vertices = Vec::with_capacity(sim.world.len() * 3);
    let mut indices = Vec::with_capacity(sim.world.len() * 3);
    for (t, (_, dev)) in sim.world.iter().zip(&sim.surface) {
        let (u, v) = (
            [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]],
            [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]],
        );
        let mut n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-30);
        n = [n[0] / len, n[1] / len, n[2] / len];
        let colour = deviation_colour(*dev);
        for p in t {
            indices.push(vertices.len() as u32);
            vertices.push(Vertex {
                pos: [
                    (p[0] - c[0]) as f32,
                    (p[1] - c[1]) as f32,
                    (p[2] - c[2]) as f32,
                ],
                normal: [n[0] as f32, n[1] as f32, n[2] as f32],
                id: u32::MAX,
                color: colour,
            });
        }
    }
    ModelScene::custom(Scene {
        vertices,
        indices,
        segments: Vec::new(),
        pick: Vec::new(),
        center: nalgebra::Vector3::new(c[0], c[1], c[2]),
        radius,
    })
}
