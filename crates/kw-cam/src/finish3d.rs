//! Three-axis finishing by drop-cutter, and the gouge and holder-collision checks for 3-D toolpaths.

use crate::bvh::{Aabb, TriMesh};
use crate::dropcutter::{cylinder_hits_mesh, drop_cutter, z_extent_in_disc};
use crate::path::{Event, Toolpath};
use crate::tool::Tool;
use crate::P3;

#[derive(Debug, Clone)]
pub struct RasterParams {
    /// Tool-centre region of the raster (x0, y0, x1, y1).
    pub region: [f64; 4],
    pub stepover: f64,
    /// Initial sample spacing along a line.
    pub step: f64,
    /// Largest allowed chord sag into the surface between samples.
    pub tolerance: f64,
    pub min_segment: f64,
    /// Tip height used where nothing is within reach (the bottom of the part).
    pub floor: f64,
    pub safe_z: f64,
    pub feed: f64,
    pub plunge_feed: f64,
}

fn height(mesh: &TriMesh, tool: &Tool, x: f64, y: f64, floor: f64) -> f64 {
    drop_cutter(mesh, tool, x, y).unwrap_or(floor).max(floor)
}

/// Sample the line from `(xa, y)` to `(xb, y)` so that no chord of the result cuts into the surface by more
/// than the tolerance.
fn line_points(mesh: &TriMesh, tool: &Tool, xa: f64, xb: f64, y: f64, p: &RasterParams) -> Vec<P3> {
    let n = (((xb - xa).abs() / p.step).ceil() as usize).max(1);
    let mut pts: Vec<P3> = (0..=n)
        .map(|i| {
            let x = xa + (xb - xa) * i as f64 / n as f64;
            [x, y, height(mesh, tool, x, y, p.floor)]
        })
        .collect();
    // refine every chord that cuts into the surface: probe interior points and split at the worst
    let mut i = 0;
    let mut guard = 0;
    while i + 1 < pts.len() && guard < 400_000 {
        guard += 1;
        let (a, b) = (pts[i], pts[i + 1]);
        if (b[0] - a[0]).abs() > p.min_segment {
            let mut worst: Option<(f64, [f64; 3])> = None;
            // probe at about 40 µm spacing (at least seven points)
            let probes = (((b[0] - a[0]).abs() / 0.04).ceil() as usize).clamp(8, 26);
            for k in 1..probes {
                let t = k as f64 / probes as f64;
                let x = a[0] + (b[0] - a[0]) * t;
                let z = height(mesh, tool, x, y, p.floor);
                let sag = z - (a[2] + (b[2] - a[2]) * t);
                if sag > 0.5 * p.tolerance && worst.is_none_or(|w| sag > w.0) {
                    worst = Some((sag, [x, y, z]));
                }
            }
            if let Some((_, q)) = worst {
                pts.insert(i + 1, q);
                continue;
            }
        }
        i += 1;
    }
    pts
}

/// Zig-zag raster finishing along x.
pub fn raster_finish(mesh: &TriMesh, tool: &Tool, p: &RasterParams) -> Toolpath {
    let mut tp = Toolpath::new("raster finish");
    tp.push(Event::ToolChange {
        number: tool.number,
        name: tool.name.clone(),
        diameter: tool.diameter,
        rpm: 9000.0,
    });
    tp.push(Event::SpindleOn {
        rpm: 9000.0,
        clockwise: true,
    });
    let [x0, y0, x1, y1] = p.region;
    let rows = (((y1 - y0) / p.stepover).ceil() as usize).max(1);
    for r in 0..=rows {
        let y = y0 + (y1 - y0) * r as f64 / rows as f64;
        let (xa, xb) = if r % 2 == 0 { (x0, x1) } else { (x1, x0) };
        let pts = line_points(mesh, tool, xa, xb, y, p);
        let first = pts[0];
        tp.push(Event::Rapid {
            p: [first[0], first[1], p.safe_z],
        });
        tp.push(Event::Feed {
            p: first,
            feed: p.plunge_feed,
        });
        for q in &pts[1..] {
            tp.push(Event::Feed {
                p: *q,
                feed: p.feed,
            });
        }
        let last = *pts.last().expect("a point");
        tp.push(Event::Rapid {
            p: [last[0], last[1], p.safe_z],
        });
    }
    tp.push(Event::SpindleOff);
    tp.push(Event::ProgramEnd);
    tp
}

/// All straight moves of a toolpath (rapids included) as `(from, to)`; arcs are chorded.
pub fn moves(tp: &Toolpath) -> Vec<(P3, P3)> {
    let mut out = Vec::new();
    let mut pos: Option<P3> = None;
    for e in &tp.events {
        match e {
            Event::Rapid { p } | Event::Feed { p, .. } => {
                if let Some(q) = pos {
                    out.push((q, *p));
                }
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
                    let n = ((d.abs() * r / 0.2).ceil() as usize).max(2);
                    let mut prev = q;
                    for k in 1..=n {
                        let t = k as f64 / n as f64;
                        let a = a0 + d * t;
                        let c = [
                            centre[0] + r * a.cos(),
                            centre[1] + r * a.sin(),
                            q[2] + (p[2] - q[2]) * t,
                        ];
                        out.push((prev, c));
                        prev = c;
                    }
                }
                pos = Some(*p);
            }
            _ => {}
        }
    }
    out
}

/// Poses along the moves at spacing at most `spacing`.
pub fn sample_poses(mv: &[(P3, P3)], spacing: f64) -> Vec<P3> {
    let mut out = Vec::new();
    for (a, b) in mv {
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
        let n = ((len / spacing).ceil() as usize).max(1);
        for i in 0..=n {
            let t = i as f64 / n as f64;
            out.push([
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ]);
        }
    }
    out
}

#[derive(Debug, Clone, Default)]
pub struct Report {
    pub poses: usize,
    /// Poses where the tip is below the non-cutting height by more than the tolerance.
    pub gouges: usize,
    pub max_gouge: f64,
    pub shank_collisions: usize,
    pub holder_collisions: usize,
}

/// The stack of cylinders above the cutting part: `(radius, from, to)` heights above the tip.
pub fn sections(tool: &Tool) -> Vec<(f64, f64, f64)> {
    let mut v = Vec::new();
    let mut z = tool.flute_length;
    v.push((tool.shank_radius, z, z + tool.shank_length));
    z += tool.shank_length;
    for (r, l) in &tool.holder.sections {
        v.push((*r, z, z + *l));
        z += *l;
    }
    v
}

/// Check poses against a mesh: gouge (tip below the drop-cutter height by more than `tol`) and the
/// shank/holder cylinders against the mesh.
pub fn check_poses(mesh: &TriMesh, tool: &Tool, poses: &[P3], tol: f64) -> Report {
    let mut rep = Report {
        poses: poses.len(),
        ..Report::default()
    };
    let secs = sections(tool);
    for p in poses {
        if let Some(h) = drop_cutter(mesh, tool, p[0], p[1]) {
            let g = h - p[2];
            if g > tol {
                rep.gouges += 1;
            }
            rep.max_gouge = rep.max_gouge.max(g);
        }
        for (k, (r, a, b)) in secs.iter().enumerate() {
            if cylinder_hits_mesh(mesh, [p[0], p[1]], *r, p[2] + a, p[2] + b, 1e-6) {
                if k == 0 {
                    rep.shank_collisions += 1;
                } else {
                    rep.holder_collisions += 1;
                }
            }
        }
    }
    rep
}

/// The shortest distance from the tip to the start of a holder of radius `holder_radius` that keeps it clear
/// of the mesh along all poses (the stick-out the assembly needs).
pub fn required_stickout(mesh: &TriMesh, holder_radius: f64, poses: &[P3]) -> f64 {
    let mut need = f64::NEG_INFINITY;
    for p in poses {
        let q = Aabb {
            min: [
                p[0] - holder_radius,
                p[1] - holder_radius,
                f64::NEG_INFINITY,
            ],
            max: [p[0] + holder_radius, p[1] + holder_radius, f64::INFINITY],
        };
        mesh.query(&q, |i| {
            if let Some((_, hi)) = z_extent_in_disc(&mesh.tris[i], [p[0], p[1]], holder_radius) {
                need = need.max(hi - p[2]);
            }
        });
    }
    need
}
