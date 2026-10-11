//! Multi-axis operations: 3+2 setups (a fixed tilted tool axis) and swarf (flank) milling of ruled surfaces.

use crate::kin::{Pose, V3};
use crate::path::{Event, Toolpath};

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(a: V3) -> V3 {
    let n = dot(a, a).sqrt();
    scale(a, 1.0 / n)
}

/// A frame for a 3+2 setup: origin and orthonormal axes (u, v, w) in workpiece coordinates, with the tool
/// pointing along `w`.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub origin: V3,
    pub u: V3,
    pub v: V3,
    pub w: V3,
}

impl Frame {
    /// The frame whose tool axis is `w`, with `u` as close as possible to the workpiece x axis.
    pub fn from_axis(origin: V3, w: V3) -> Frame {
        let w = unit(w);
        let helper = if w[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let u = unit(sub(helper, scale(w, dot(helper, w))));
        let v = cross(w, u);
        Frame { origin, u, v, w }
    }

    pub fn to_work(&self, p: V3) -> V3 {
        add(
            self.origin,
            add(
                add(scale(self.u, p[0]), scale(self.v, p[1])),
                scale(self.w, p[2]),
            ),
        )
    }

    pub fn to_frame(&self, p: V3) -> V3 {
        let d = sub(p, self.origin);
        [dot(d, self.u), dot(d, self.v), dot(d, self.w)]
    }
}

/// Poses of a toolpath written in the setup frame: the tip positions mapped to the workpiece, the tool axis
/// held at `w`. Arcs are chorded (a tilted arc is not an arc of the machine's XY plane).
pub fn setup_poses(tp: &Toolpath, frame: &Frame) -> Vec<Pose> {
    let mut out = Vec::new();
    let mut pos: Option<V3> = None;
    for e in &tp.events {
        match e {
            Event::Rapid { p } | Event::Feed { p, .. } => {
                out.push(Pose {
                    tip: frame.to_work(*p),
                    axis: frame.w,
                });
                pos = Some(*p);
            }
            Event::Arc { p, centre, ccw, .. } => {
                if let Some(q) = pos {
                    let r = (q[0] - centre[0]).hypot(q[1] - centre[1]);
                    let a0 = (q[1] - centre[1]).atan2(q[0] - centre[0]);
                    let a1 = (p[1] - centre[1]).atan2(p[0] - centre[0]);
                    let mut d = a1 - a0;
                    if *ccw {
                        while d <= 1e-12 {
                            d += std::f64::consts::TAU;
                        }
                    } else {
                        while d >= -1e-12 {
                            d -= std::f64::consts::TAU;
                        }
                    }
                    let n = ((d.abs() * r / 0.1).ceil() as usize).max(2);
                    for k in 1..=n {
                        let t = k as f64 / n as f64;
                        let a = a0 + d * t;
                        out.push(Pose {
                            tip: frame.to_work([
                                centre[0] + r * a.cos(),
                                centre[1] + r * a.sin(),
                                q[2] + (p[2] - q[2]) * t,
                            ]),
                            axis: frame.w,
                        });
                    }
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    out
}

/// Which side of the surface the tool stands on, relative to `ruling × tangent`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Side {
    Left,
    Right,
}

/// Swarf poses for a ruled surface given by matching points on a bottom and a top curve (the rulings are the
/// segments between them). The tool is a cylinder of radius `r`: its axis is the ruling shifted by `r` along
/// the surface normal, the tip lies on the bottom curve's offset. Exact for developable surfaces; for the
/// others the flank deviates between rulings (measure it with [`flank_deviation`]).
pub fn swarf_poses(bottom: &[V3], top: &[V3], r: f64, side: Side) -> Vec<Pose> {
    assert_eq!(bottom.len(), top.len());
    let n = bottom.len();
    (0..n)
        .map(|i| {
            let (a, b) = (bottom[i], top[i]);
            let ruling = unit(sub(b, a));
            let tangent = if i + 1 < n {
                sub(bottom[i + 1], bottom[i])
            } else {
                sub(bottom[i], bottom[i - 1])
            };
            let mut nrm = unit(cross(ruling, tangent));
            if side == Side::Right {
                nrm = scale(nrm, -1.0);
            }
            Pose {
                tip: add(a, scale(nrm, r)),
                axis: ruling,
            }
        })
        .collect()
}

/// Deviation of the machined flank from the design surface: for sample points of the ruled surface, the
/// distance to the nearest tool axis minus the tool radius. Zero means the swept envelope touches the surface
/// there; negative values are gouges, positive values leave material. Returns the largest magnitude.
pub fn flank_deviation(bottom: &[V3], top: &[V3], poses: &[Pose], r: f64, samples: usize) -> f64 {
    let window = 6usize;
    let mut worst = 0.0f64;
    for i in 0..poses.len() - 1 {
        for k in 0..=samples {
            let t = k as f64 / samples as f64;
            let pb = add(scale(bottom[i], 1.0 - t), scale(bottom[i + 1], t));
            let pt = add(scale(top[i], 1.0 - t), scale(top[i + 1], t));
            for m in 0..=4 {
                let s = m as f64 / 4.0;
                let q = add(scale(pb, 1.0 - s), scale(pt, s));
                let lo = i.saturating_sub(window);
                let hi = (i + window + 1).min(poses.len());
                let nearest = (lo..hi)
                    .map(|j| {
                        let d = sub(q, poses[j].tip);
                        let a = unit(poses[j].axis);
                        let perp = sub(d, scale(a, dot(d, a)));
                        dot(perp, perp).sqrt()
                    })
                    .fold(f64::INFINITY, f64::min);
                worst = worst.max((nearest - r).abs());
            }
        }
    }
    worst
}
