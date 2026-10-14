//! 2.5-D operations: contour, pocket (inside-out offset loops with islands), drilling.

use crate::geom2d::{nearest_vertex, perimeter, start_point, trace_loop, Region};
use crate::path::{Event, Toolpath};
use crate::tool::Tool;
use crate::{P2, P3};
use cavalier_contours::polyline::Polyline;

/// Cutting parameters shared by the 2.5-D operations (all lengths in mm, z up).
#[derive(Debug, Clone)]
pub struct Cut {
    /// Top of the stock.
    pub top: f64,
    /// Final depth of the cut (z of the tool tip), below `top`.
    pub bottom: f64,
    /// Maximum depth per pass.
    pub step_down: f64,
    /// Distance between neighbouring offset loops.
    pub stepover: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    pub rpm: f64,
    /// Height above `top` for rapid moves.
    pub clearance: f64,
    /// Stock left on the walls for a finishing pass.
    pub wall_allowance: f64,
}

impl Cut {
    pub fn new(top: f64, bottom: f64) -> Cut {
        Cut {
            top,
            bottom,
            step_down: 3.0,
            stepover: 3.0,
            feed: 800.0,
            plunge_feed: 300.0,
            rpm: 6000.0,
            clearance: 5.0,
            wall_allowance: 0.0,
        }
    }

    /// Depth levels from the first pass to the final depth.
    pub fn levels(&self) -> Vec<f64> {
        let depth = self.top - self.bottom;
        if depth <= 0.0 {
            return Vec::new();
        }
        let n = (depth / self.step_down).ceil().max(1.0) as usize;
        (1..=n)
            .map(|i| self.top - depth * i as f64 / n as f64)
            .collect()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// The tool stays inside the region (walls of a pocket, outside faces of an island).
    Inside,
    /// The tool stays outside the region (the profile of a part).
    Outside,
}

pub fn start_program(tp: &mut Toolpath, tool: &Tool, cut: &Cut) {
    tp.push(Event::ToolChange {
        number: tool.number,
        name: tool.name.clone(),
        diameter: tool.diameter,
        rpm: cut.rpm,
    });
    tp.push(Event::SpindleOn {
        rpm: cut.rpm,
        clockwise: true,
    });
    tp.push(Event::CoolantOn);
}

pub fn end_program(tp: &mut Toolpath, cut: &Cut) {
    tp.push(Event::Rapid {
        p: [last_xy(tp).0, last_xy(tp).1, cut.top + cut.clearance],
    });
    tp.push(Event::CoolantOff);
    tp.push(Event::SpindleOff);
    tp.push(Event::ProgramEnd);
}

fn last_xy(tp: &Toolpath) -> (f64, f64) {
    for e in tp.events.iter().rev() {
        match e {
            Event::Rapid { p } | Event::Feed { p, .. } | Event::Arc { p, .. } => {
                return (p[0], p[1])
            }
            _ => {}
        }
    }
    (0.0, 0.0)
}

/// Follow the offset loops of `region` at the tool radius (plus the wall allowance) on `side`, one pass per
/// depth level.
pub fn contour(region: &Region, tool: &Tool, side: Side, cut: &Cut) -> Toolpath {
    let mut tp = Toolpath::new("contour");
    start_program(&mut tp, tool, cut);
    let d = tool.radius() + cut.wall_allowance;
    let path_region = match side {
        Side::Inside => region.offset(d),
        Side::Outside => region.offset(-d),
    };
    let loops = path_region.loops();
    let mut pos: Option<P3> = None;
    let mut z_prev = cut.top;
    for z in cut.levels() {
        for pl in &loops {
            let s = nearest_vertex(pl, pos.map_or([0.0, 0.0], |p| [p[0], p[1]]));
            let sp = start_point(pl, s);
            tp.push(Event::Rapid {
                p: [sp[0], sp[1], cut.top + cut.clearance],
            });
            // the level above is cut along the same loops: come down to just above its floor
            tp.push(Event::Rapid {
                p: [sp[0], sp[1], z_prev + 1.0_f64.min(cut.clearance)],
            });
            tp.push(Event::Feed {
                p: [sp[0], sp[1], z],
                feed: cut.plunge_feed,
            });
            let mut evs = Vec::new();
            trace_loop(&mut evs, pl, s, z, z, cut.feed);
            tp.events.extend(evs);
            pos = Some([sp[0], sp[1], z]);
        }
        z_prev = z;
    }
    end_program(&mut tp, cut);
    tp
}

/// The tool-centre loops of a pocket from the innermost to the wall pass.
pub fn pocket_loops(region: &Region, tool: &Tool, cut: &Cut) -> Vec<Vec<Polyline<f64>>> {
    let r = tool.radius() + cut.wall_allowance;
    let mut rings: Vec<Vec<Polyline<f64>>> = Vec::new();
    let mut k = 0usize;
    loop {
        let off = region.offset(r + k as f64 * cut.stepover);
        if off.is_empty() {
            break;
        }
        rings.push(off.loops());
        k += 1;
        if k > 10_000 {
            break;
        }
    }
    rings.reverse(); // innermost first
    rings
}

/// Clear the region (islands excluded) with concentric loops from the inside out, ramping into each
/// depth level along the innermost loop.
pub fn pocket(region: &Region, tool: &Tool, cut: &Cut) -> Toolpath {
    let mut tp = Toolpath::new("pocket");
    start_program(&mut tp, tool, cut);
    let rings = pocket_loops(region, tool, cut);
    if rings.is_empty() {
        end_program(&mut tp, cut);
        return tp;
    }
    // where the tool may be at all (to decide whether a link can stay down)
    let free = region.offset(tool.radius() + cut.wall_allowance);
    let mut z_prev = cut.top;
    let mut pos: Option<P2> = None;
    for z in cut.levels() {
        let mut first = true;
        for ring in &rings {
            for pl in ring {
                let s = nearest_vertex(pl, pos.unwrap_or([0.0, 0.0]));
                let sp = start_point(pl, s);
                if first {
                    // rapid above the start, then ramp down along the loop
                    tp.push(Event::Rapid {
                        p: [sp[0], sp[1], cut.top + cut.clearance],
                    });
                    tp.push(Event::Rapid {
                        p: [sp[0], sp[1], z_prev + 1.0_f64.min(cut.clearance)],
                    });
                    let per = perimeter(pl).max(1e-9);
                    let dz = z_prev + 1.0_f64.min(cut.clearance) - z;
                    // at most about 5 degrees of ramp angle
                    let laps = ((dz / (per * 0.087)).ceil() as usize).max(1);
                    let mut zs = z_prev + 1.0_f64.min(cut.clearance);
                    for lap in 0..laps {
                        let ze = zs - dz / laps as f64;
                        let mut evs = Vec::new();
                        trace_loop(&mut evs, pl, s, zs, ze, cut.plunge_feed.min(cut.feed));
                        tp.events.extend(evs);
                        zs = ze;
                        let _ = lap;
                    }
                    // clean the ramp: one flat lap
                    let mut evs = Vec::new();
                    trace_loop(&mut evs, pl, s, z, z, cut.feed);
                    tp.events.extend(evs);
                    first = false;
                } else {
                    // link: stay down when the straight line to the next start is inside the free region
                    let from = pos.unwrap();
                    let stay = (1..=8).all(|i| {
                        let t = i as f64 / 8.0;
                        free.contains([
                            from[0] + (sp[0] - from[0]) * t,
                            from[1] + (sp[1] - from[1]) * t,
                        ])
                    });
                    if stay {
                        tp.push(Event::Feed {
                            p: [sp[0], sp[1], z],
                            feed: cut.feed,
                        });
                    } else {
                        tp.push(Event::Rapid {
                            p: [from[0], from[1], cut.top + cut.clearance],
                        });
                        tp.push(Event::Rapid {
                            p: [sp[0], sp[1], cut.top + cut.clearance],
                        });
                        // the level above is cleared everywhere: come down to just above it
                        tp.push(Event::Rapid {
                            p: [sp[0], sp[1], z_prev + 1.0_f64.min(cut.clearance)],
                        });
                        tp.push(Event::Feed {
                            p: [sp[0], sp[1], z],
                            feed: cut.plunge_feed,
                        });
                    }
                    let mut evs = Vec::new();
                    trace_loop(&mut evs, pl, s, z, z, cut.feed);
                    tp.events.extend(evs);
                }
                pos = Some(sp);
            }
        }
        z_prev = z;
    }
    end_program(&mut tp, cut);
    tp
}

/// Drill holes (nearest-neighbour order) to depth `bottom`; peck when `peck` is given.
pub fn drill(points: &[P2], tool: &Tool, cut: &Cut, peck: Option<f64>) -> Toolpath {
    let mut tp = Toolpath::new("drill");
    start_program(&mut tp, tool, cut);
    let mut left: Vec<P2> = points.to_vec();
    let mut order: Vec<P2> = Vec::new();
    let mut cur = [0.0, 0.0];
    while !left.is_empty() {
        let i = (0..left.len())
            .min_by(|a, b| {
                let da = (left[*a][0] - cur[0]).hypot(left[*a][1] - cur[1]);
                let db = (left[*b][0] - cur[0]).hypot(left[*b][1] - cur[1]);
                da.partial_cmp(&db).unwrap()
            })
            .unwrap();
        cur = left.remove(i);
        order.push(cur);
    }
    tp.push(Event::Rapid {
        p: [0.0, 0.0, cut.top + cut.clearance],
    });
    tp.push(Event::DrillStart {
        depth: cut.bottom,
        retract: cut.top + 2.0_f64.min(cut.clearance),
        surface: cut.top,
        peck,
        feed: cut.plunge_feed,
    });
    for p in order {
        tp.push(Event::DrillAt { x: p[0], y: p[1] });
    }
    tp.push(Event::DrillEnd);
    end_program(&mut tp, cut);
    tp
}

/// Bore a hole of `hole_diameter` centred at `centre` with a smaller tool on a helix: down in turns of
/// `step_down` from `cut.top` to `cut.bottom`, a flat lap at the bottom, then out through the middle.
/// `None` when the tool is not smaller than the hole.
pub fn bore(centre: P2, hole_diameter: f64, tool: &Tool, cut: &Cut) -> Option<Toolpath> {
    let r = 0.5 * (hole_diameter - tool.diameter) - cut.wall_allowance;
    if r <= 1e-6 || cut.top <= cut.bottom {
        return None;
    }
    let mut tp = Toolpath::new("bore");
    start_program(&mut tp, tool, cut);
    let start = [centre[0] + r, centre[1]];
    let safe = cut.top + cut.clearance;
    let near = cut.top + 1.0_f64.min(cut.clearance);
    tp.push(Event::Rapid {
        p: [centre[0], centre[1], safe],
    });
    tp.push(Event::Rapid {
        p: [start[0], start[1], near],
    });
    let depth = near - cut.bottom;
    let turns = ((depth / cut.step_down.max(0.05)).ceil() as usize).max(1);
    // a turn is two half circles; the height falls evenly over them
    let halves = 2 * turns;
    for k in 0..halves {
        let ang = std::f64::consts::PI * (k + 1) as f64;
        let z = near - depth * (k + 1) as f64 / halves as f64;
        tp.push(Event::Arc {
            p: [centre[0] + r * ang.cos(), centre[1] + r * ang.sin(), z],
            centre,
            ccw: true,
            feed: cut.plunge_feed.min(cut.feed),
        });
    }
    // a flat lap at the bottom takes off the helix
    for k in 0..2 {
        let ang = std::f64::consts::PI * (k + 1) as f64;
        tp.push(Event::Arc {
            p: [
                centre[0] + r * ang.cos(),
                centre[1] + r * ang.sin(),
                cut.bottom,
            ],
            centre,
            ccw: true,
            feed: cut.feed,
        });
    }
    tp.push(Event::Feed {
        p: [centre[0], centre[1], cut.bottom],
        feed: cut.feed,
    });
    tp.push(Event::Rapid {
        p: [centre[0], centre[1], safe],
    });
    tp.push(Event::CoolantOff);
    tp.push(Event::SpindleOff);
    tp.push(Event::ProgramEnd);
    Some(tp)
}
