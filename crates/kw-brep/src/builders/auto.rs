//! Face construction from bare edge chains: computes pcurves (with periodic-branch continuity,
//! pole/apex degenerate edges included), chooses the face sense from an outward direction, and
//! orients every loop so that it satisfies the B-rep conventions.

use crate::ids::*;
use crate::store::{BRep, CoedgeSpec};
use crate::topology::*;
use kw_math::{Point3, Vector3};
use kw_nurbs::pcurve::{pcurve_error, pcurve_on_anchored};
use kw_nurbs::proj::surface_param_of;
use kw_nurbs::{Curve3, CurveGeom, Line, Surface};

#[derive(Debug, Clone)]
pub struct LoopSpec {
    pub kind: LoopKind,
    /// Ordered, consecutive edge uses (each use's end vertex is the next use's start vertex).
    pub chain: Vec<(EdgeId, Sense)>,
}

fn xy(u: f64, v: f64) -> Point3<f64> {
    Point3::new(u, v, 0.0)
}

/// Tolerance for fitted pcurves (absolute, scaled by model size).
fn pc_tol(rep: &BRep) -> f64 {
    let scale = rep
        .vertices
        .values()
        .map(|v| v.point.coords.amax())
        .fold(1.0f64, f64::max);
    1e-9 * (1.0 + scale)
}

/// Add a face on `surface` bounded by `loops`. `outward` gives the desired outward normal at a
/// point of the face boundary (used to choose `Forward`/`Reversed`).
pub fn add_face_auto(
    rep: &mut BRep,
    shell: ShellId,
    surface: SurfaceId,
    outward: &dyn Fn(&Point3<f64>) -> Vector3<f64>,
    loops: &[LoopSpec],
) -> Result<FaceId, String> {
    let surf = rep.surfaces[surface].clone();
    let tol = pc_tol(rep);
    // --- sense from a boundary sample
    let sample = loops
        .iter()
        .flat_map(|l| l.chain.iter())
        .find_map(|(e, _)| {
            let edge = &rep.edges[*e];
            let c = rep.curves.get(edge.curve?)?;
            Some(c.point_at(0.5 * (edge.param_range.0 + edge.param_range.1)))
        })
        .ok_or("face has no non-degenerate edge")?;
    let r = surface_param_of(&surf, &sample, None);
    let n = surf.normal_at(r.u, r.v);
    let sense = if n.dot(&outward(&sample)) >= 0.0 {
        Sense::Forward
    } else {
        Sense::Reversed
    };

    // --- pcurves per loop
    #[derive(Clone)]
    struct Placed {
        pc: Option<CurveGeom>,
        /// uv at traversal start / end
        start: Option<(f64, f64)>,
        end: Option<(f64, f64)>,
        /// deviation of the pcurve image from the edge curve (0 for exact forms)
        err: f64,
    }
    let mut all: Vec<(LoopKind, Vec<(EdgeId, Sense)>, Vec<CurveGeom>)> = Vec::new();
    let mut edge_err: Vec<(EdgeId, f64)> = Vec::new();
    for ls in loops {
        let m = ls.chain.len();
        let degenerate = |k: usize| rep.edges[ls.chain[k].0].curve.is_none();
        let seed = (0..m)
            .find(|k| !degenerate(*k))
            .ok_or("loop of degenerate edges")?;
        // place coedge k with an anchor given as (param, uv)
        let place = |k: usize,
                     anchor: Option<(f64, (f64, f64))>,
                     placed: &mut Vec<Placed>|
         -> Result<(), String> {
            let (eid, sn) = ls.chain[k];
            let edge = &rep.edges[eid];
            let curve = rep
                .curves
                .get(edge.curve.unwrap())
                .ok_or("edge without curve")?;
            // Curves read from files only lie on their surfaces up to the file's tolerance: loosen the
            // fit tolerance step by step and let the edge carry the deviation that was needed.
            // Start from the deviation the curve actually has from the surface: a fit tighter than
            // that can never succeed and is expensive to give up on.
            let dev = (0..=16)
                .map(|i| {
                    let (a, b) = edge.param_range;
                    let p = curve.point_at(a + (b - a) * i as f64 / 16.0);
                    surface_param_of(&surf, &p, None).distance
                })
                .fold(0.0f64, f64::max);
            let base = tol.max(2.0 * dev);
            let mut found = None;
            for mult in [1.0, 10.0, 100.0, 1e3] {
                if let Some(pc) =
                    pcurve_on_anchored(&surf, curve, edge.param_range, anchor, base * mult)
                {
                    let err = pcurve_error(&surf, curve, &pc, edge.param_range, 64);
                    found = Some((pc, err));
                    break;
                }
            }
            let (pc, err) = found.ok_or_else(|| {
                let kind = |d: String| d.split(['(', ' ', '{']).next().unwrap_or("").to_string();
                format!(
                    "could not build a pcurve for edge {eid:?} ({} on {}, range {:?}, anchor {:?})",
                    kind(format!("{curve:?}")),
                    kind(format!("{surf:?}")),
                    edge.param_range,
                    anchor
                )
            })?;
            let (a, b) = match sn {
                Sense::Forward => (edge.param_range.0, edge.param_range.1),
                Sense::Reversed => (edge.param_range.1, edge.param_range.0),
            };
            let (s, e) = (pc.point_at(a), pc.point_at(b));
            placed[k] = Placed {
                pc: Some(pc),
                start: Some((s.x, s.y)),
                end: Some((e.x, e.y)),
                err,
            };
            Ok(())
        };
        // Place coedges until none is left: a coedge is anchored to a placed non-degenerate neighbour
        // (its predecessor's end or its successor's start). Runs of coedges separated by degenerate
        // edges (poles / apexes) are only related by a whole number of periods: the first coedge of
        // run `r` is moved by `ms[r]` periods, and the combination whose parameter-space polygon winds
        // the way the loop must (counter-clockwise about the face normal for outer loops) while wrapping
        // the surface at most once is kept.
        let pu = surf.period_u().unwrap_or(0.0);
        let solve = |ms: &[i32], placed: &mut Vec<Placed>| -> Result<usize, String> {
            let mut done = vec![false; m];
            let mut runs = 0usize;
            place(seed, None, placed)?;
            done[seed] = true;
            loop {
                loop {
                    let mut progress = false;
                    for step in 0..m {
                        let k = (seed + step) % m;
                        if done[k] || degenerate(k) {
                            continue;
                        }
                        let (prev, next) = ((k + m - 1) % m, (k + 1) % m);
                        let (eid, sn) = ls.chain[k];
                        let edge = &rep.edges[eid];
                        let (start_param, end_param) = if sn == Sense::Forward {
                            (edge.param_range.0, edge.param_range.1)
                        } else {
                            (edge.param_range.1, edge.param_range.0)
                        };
                        if !degenerate(prev) && done[prev] {
                            let uv = placed[prev].end.unwrap();
                            place(k, Some((start_param, uv)), placed)?;
                        } else if !degenerate(next) && done[next] {
                            let uv = placed[next].start.unwrap();
                            place(k, Some((end_param, uv)), placed)?;
                        } else {
                            continue;
                        }
                        done[k] = true;
                        progress = true;
                    }
                    if !progress {
                        break;
                    }
                }
                // a run behind a degenerate edge
                let Some(k) = (0..m)
                    .map(|st| (seed + st) % m)
                    .find(|&k| !done[k] && !degenerate(k))
                else {
                    break;
                };
                let (eid, sn) = ls.chain[k];
                let edge = &rep.edges[eid];
                let start_param = if sn == Sense::Forward {
                    edge.param_range.0
                } else {
                    edge.param_range.1
                };
                let mut j = (k + m - 1) % m;
                while degenerate(j) && j != k {
                    j = (j + m - 1) % m;
                }
                let Some(q) = placed[j].end.filter(|_| done[j]) else {
                    return Err("could not place a coedge".into());
                };
                place(k, Some((start_param, q)), placed)?;
                let mult = ms.get(runs).copied().unwrap_or(0);
                if mult != 0 && pu > 0.0 {
                    let du = f64::from(mult) * pu;
                    let pl = &mut placed[k];
                    pl.pc = pl
                        .pc
                        .take()
                        .map(|pc| kw_nurbs::fit::translate_pcurve(&pc, du, 0.0));
                    pl.start = pl.start.map(|x| (x.0 + du, x.1));
                    pl.end = pl.end.map(|x| (x.0 + du, x.1));
                }
                runs += 1;
                done[k] = true;
            }
            Ok(runs)
        };
        let fresh = || -> Vec<Placed> {
            (0..m)
                .map(|_| Placed {
                    pc: None,
                    start: None,
                    end: None,
                    err: 0.0,
                })
                .collect()
        };
        // degenerate edges: straight line between the neighbours' uv
        let fill = |placed: &mut Vec<Placed>| -> Result<(), String> {
            for k in 0..m {
                if !degenerate(k) {
                    continue;
                }
                let prev = (k + m - 1) % m;
                let next = (k + 1) % m;
                let (Some(a), Some(b)) = (placed[prev].end, placed[next].start) else {
                    return Err("degenerate edge between degenerate edges".into());
                };
                let sn = ls.chain[k].1;
                let (o, d) = match sn {
                    Sense::Forward => (a, (b.0 - a.0, b.1 - a.1)),
                    Sense::Reversed => (b, (a.0 - b.0, a.1 - b.1)),
                };
                placed[k].pc = Some(CurveGeom::Line(Line {
                    origin: xy(o.0, o.1),
                    dir: Vector3::new(d.0, d.1, 0.0),
                }));
            }
            Ok(())
        };
        // (signed polygon area, u extent) of a completed placement
        let measure = |pl: &[Placed]| -> (f64, f64) {
            let mut poly: Vec<(f64, f64)> = Vec::new();
            for (k, p) in pl.iter().enumerate() {
                let (eid, sn) = ls.chain[k];
                let edge = &rep.edges[eid];
                let (a, b) = match sn {
                    Sense::Forward => (edge.param_range.0, edge.param_range.1),
                    Sense::Reversed => (edge.param_range.1, edge.param_range.0),
                };
                let pc = p.pc.as_ref().unwrap();
                for i in 0..8 {
                    let q = pc.point_at(a + (b - a) * i as f64 / 8.0);
                    poly.push((q.x, q.y));
                }
            }
            let mut area = 0.0;
            for i in 0..poly.len() {
                let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
                area += p.0 * q.1 - q.0 * p.1;
            }
            let lo = poly.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
            let hi = poly.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
            (area, hi - lo)
        };
        let want_positive = (ls.kind == LoopKind::Outer) == (sense == Sense::Forward);
        let mut placed = fresh();
        let runs = solve(&[], &mut placed)?;
        fill(&mut placed)?;
        if runs > 0 && pu > 0.0 {
            // candidates: every combination of {-1, 0, +1} periods per run (at most 3 runs), ranked by
            // (wrong winding, wraps more than once, u extent)
            let score = |pl: &[Placed]| -> (bool, bool, f64) {
                let (area, ext) = measure(pl);
                (
                    area.abs() < 1e-9 || (area > 0.0) != want_positive,
                    ext > pu * (1.0 + 1e-6),
                    ext,
                )
            };
            let mut best = score(&placed);
            let combos = if runs <= 3 {
                3usize.pow(runs as u32)
            } else {
                0
            };
            for c in 0..combos {
                let ms: Vec<i32> = (0..runs)
                    .map(|r| ((c / 3usize.pow(r as u32)) % 3) as i32 - 1)
                    .collect();
                let mut cand = fresh();
                if solve(&ms, &mut cand).is_err() || fill(&mut cand).is_err() {
                    continue;
                }
                let sc = score(&cand);
                if sc < best {
                    best = sc;
                    placed = cand;
                }
            }
        }
        for (k, p) in placed.iter().enumerate() {
            if p.err > tol {
                edge_err.push((ls.chain[k].0, p.err));
            }
        }
        let pcs: Vec<CurveGeom> = placed.into_iter().map(|p| p.pc.unwrap()).collect();
        all.push((ls.kind, ls.chain.clone(), pcs));
    }

    // --- orientation of each loop from its parameter-space polygon
    for (e, err) in edge_err {
        let t = &mut rep.edges[e].tolerance;
        *t = t.max(err * 1.5);
    }
    let face = rep.add_face(shell, surface, sense);
    for (kind, mut chain, mut pcs) in all {
        let mut poly: Vec<(f64, f64)> = Vec::new();
        for (k, (eid, sn)) in chain.iter().enumerate() {
            let edge = &rep.edges[*eid];
            let (a, b) = match sn {
                Sense::Forward => (edge.param_range.0, edge.param_range.1),
                Sense::Reversed => (edge.param_range.1, edge.param_range.0),
            };
            for i in 0..8 {
                let p = pcs[k].point_at(a + (b - a) * i as f64 / 8.0);
                poly.push((p.x, p.y));
            }
        }
        let mut area = 0.0;
        for i in 0..poly.len() {
            let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
            area += p.0 * q.1 - q.0 * p.1;
        }
        let want_positive = (kind == LoopKind::Outer) == (sense == Sense::Forward);
        if (area > 0.0) != want_positive {
            chain.reverse();
            pcs.reverse();
            for c in &mut chain {
                c.1 = c.1.flip();
            }
        }
        let specs: Vec<CoedgeSpec> = chain
            .iter()
            .zip(pcs)
            .map(|((e, s), pc)| CoedgeSpec {
                edge: *e,
                sense: *s,
                pcurve: Some(rep.add_curve(pc)),
            })
            .collect();
        rep.add_loop(face, kind, &specs);
    }
    Ok(face)
}
