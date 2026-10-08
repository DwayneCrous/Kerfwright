//! Interactive dragging.
//!
//! The dragged variables are pinned to their targets and only the part of the
//! sketch that depends on them is re-solved, warm-started from the current
//! geometry with the sparse minimum-norm LM solver:
//!
//! * the affected *region* is found from the Dulmage–Mendelsohn decomposition of
//!   the pinned system: the blocks that read a pinned variable, everything
//!   downstream of them, and (when free variables are involved) the blocks that
//!   share those free variables;
//! * if the target is unreachable (the constraints forbid that position) the
//!   dragged variables slide to the closest feasible position: a soft pull
//!   followed by an exact polish;
//! * moves that would flip a branch (tangent side, joint direction, radius
//!   sign) are refused/shortened by bisection.

use super::sparse::{sparse_lm, Cluster, Soft};
use super::*;
use crate::entity::VarId;
use crate::graph::{decompose, NONE};

pub(crate) struct PinnedComp {
    /// Region unknowns (pins held constant).
    region: Cluster,
    /// Same rows with the pins as unknowns (soft-target fallback).
    soft: Cluster,
    /// Residuals checked for branch flips.
    check: Vec<u32>,
}

pub(crate) struct DragCache {
    version: u64,
    pins: Vec<u32>,
    comps: Vec<PinnedComp>,
    /// The last frame could not reach its target exactly: try the slide
    /// (soft-target) solve first, it also reaches feasible targets.
    prefer_soft: std::cell::Cell<bool>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DragOutcome {
    /// The geometry changed.
    pub moved: bool,
    /// The dragged variables reached their targets exactly (otherwise they
    /// slid to the closest feasible position, or the move was shortened).
    pub reached: bool,
    /// Fraction of the requested displacement achieved along its direction (1 = fully).
    pub fraction: f64,
    /// All affected equations are satisfied after the move.
    pub converged: bool,
    /// Part of the move was refused because it would flip a branch.
    pub branch_limited: bool,
}

const MAX_REGION_COLS: usize = 4000;

fn build_pinned(sys: &System, cache: &Cache, comp: usize, pins: &[u32], x: &[f64]) -> PinnedComp {
    let c = &cache.comps[comp];
    let comp_pins: Vec<u32> = pins
        .iter()
        .copied()
        .filter(|p| cache.var_comp[*p as usize] == comp as u32)
        .collect();
    let dec = decompose(sys, c, &comp_pins, x);
    let nb = dec.blocks.len();
    let mut block_of_row = std::collections::HashMap::new();
    for (b, blk) in dec.blocks.iter().enumerate() {
        for &r in &blk.rows {
            block_of_row.insert(r, b as u32);
        }
    }
    let mut touched = vec![false; nb];
    let mut pat = Vec::new();
    for &r in &c.rows {
        sys.row_pattern(r as usize, &mut pat);
        if pat.iter().any(|g| comp_pins.contains(g)) {
            if let Some(&b) = block_of_row.get(&r) {
                touched[b as usize] = true;
            }
        }
    }
    let close = |flag: &mut Vec<bool>| {
        for b in 0..nb {
            if !flag[b] && dec.deps[b].iter().any(|&d| flag[d as usize]) {
                flag[b] = true;
            }
        }
    };
    let mut region = touched.clone();
    close(&mut region);
    let reach_flags = region.clone();

    // Free variables read by the region couple it to other blocks; pull those in too.
    let mut used_free: Vec<u32> = Vec::new();
    loop {
        let mut added = false;
        used_free.clear();
        for (b, blk) in dec.blocks.iter().enumerate() {
            if !region[b] {
                continue;
            }
            for &r in &blk.rows {
                sys.row_pattern(r as usize, &mut pat);
                for &g in &pat {
                    if dec.free_cols.binary_search(&g).is_ok() {
                        used_free.push(g);
                    }
                }
            }
        }
        used_free.sort_unstable();
        used_free.dedup();
        if used_free.is_empty() {
            break;
        }
        for (b, blk) in dec.blocks.iter().enumerate() {
            if region[b] {
                continue;
            }
            let reads_free = blk.rows.iter().any(|&r| {
                sys.row_pattern(r as usize, &mut pat);
                pat.iter().any(|g| used_free.binary_search(g).is_ok())
            });
            if reads_free {
                region[b] = true;
                added = true;
            }
        }
        if added {
            close(&mut region);
        } else {
            break;
        }
    }
    let mut chosen: &Vec<bool> = &region;
    let size = |flags: &Vec<bool>| -> usize {
        dec.blocks
            .iter()
            .enumerate()
            .filter(|(b, _)| flags[*b])
            .map(|(_, blk)| blk.cols.len())
            .sum::<usize>()
    };
    if size(&region) + used_free.len() > MAX_REGION_COLS {
        chosen = &reach_flags;
        used_free.clear();
    }

    let mut rows: Vec<u32> = Vec::new();
    let mut cols: Vec<u32> = Vec::new();
    let mut check: Vec<u32> = Vec::new();
    for (b, blk) in dec.blocks.iter().enumerate() {
        if chosen[b] {
            rows.extend(blk.rows.iter().copied());
            cols.extend(blk.cols.iter().copied());
            check.extend(blk.res_list.iter().copied());
        }
    }
    cols.extend(used_free.iter().copied());
    rows.sort_unstable();
    rows.dedup();
    cols.sort_unstable();
    cols.dedup();
    check.sort_unstable();
    check.dedup();
    let mut soft_cols = cols.clone();
    soft_cols.extend(comp_pins.iter().copied());
    soft_cols.sort_unstable();
    soft_cols.dedup();
    PinnedComp {
        region: Cluster::new(sys, rows.clone(), cols),
        soft: Cluster::new(sys, rows, soft_cols),
        check,
    }
}

impl Solver {
    /// Drag the given variables to `targets` (one frame). The sketch should
    /// already be solved. Returns what was achieved.
    pub fn drag(
        &mut self,
        sk: &mut Sketch,
        targets: &[(VarId, f64)],
    ) -> Result<DragOutcome, SketchError> {
        self.prepare(sk)?;
        let mut pins: Vec<u32> = targets
            .iter()
            .filter(|(v, _)| !self.cache.as_ref().unwrap().sys.fixed[v.0])
            .map(|(v, _)| v.0 as u32)
            .collect();
        pins.sort_unstable();
        pins.dedup();
        if pins.is_empty() {
            return Ok(DragOutcome {
                moved: false,
                reached: true,
                fraction: 0.0,
                converged: true,
                branch_limited: false,
            });
        }
        let version = sk.topology_version();
        let rebuild = self
            .drag
            .as_ref()
            .is_none_or(|d| d.version != version || d.pins != pins);
        if rebuild {
            let cache = self.cache.as_ref().unwrap();
            let mut comp_ids: Vec<usize> = pins
                .iter()
                .filter_map(|p| {
                    (cache.var_comp[*p as usize] != NONE)
                        .then_some(cache.var_comp[*p as usize] as usize)
                })
                .collect();
            comp_ids.sort_unstable();
            comp_ids.dedup();
            let comps = comp_ids
                .iter()
                .map(|&c| build_pinned(&cache.sys, cache, c, &pins, &sk.vars.values))
                .collect();
            self.drag = Some(DragCache {
                version,
                pins: pins.clone(),
                comps,
                prefer_soft: std::cell::Cell::new(false),
            });
        }
        let cache = self.cache.as_ref().unwrap();
        let dc = self.drag.as_ref().unwrap();
        let sys = &cache.sys;
        let opts = self.opts.clone();
        let x = XRef::new(&mut sk.vars.values);
        let step = 0.25 * sys.scale;

        let goal: Vec<(usize, f64)> = targets
            .iter()
            .filter(|(v, _)| pins.binary_search(&(v.0 as u32)).is_ok())
            .map(|(v, t)| (v.0, *t))
            .collect();
        let old: Vec<(usize, f64)> = goal.iter().map(|(v, _)| (*v, x.get(*v))).collect();
        let snapshot = |dc: &DragCache| -> Vec<(u32, f64)> {
            dc.comps
                .iter()
                .flat_map(|p| p.soft.block.cols.iter().map(|&g| (g, x.get(g as usize))))
                .collect()
        };
        let restore = |snap: &[(u32, f64)]| {
            for (g, v) in snap {
                x.set(*g as usize, *v);
            }
            for (v, o) in &old {
                x.set(*v, *o);
            }
        };
        let branch_ok = |p: &PinnedComp| {
            !opts.prevent_flips
                || p.check
                    .iter()
                    .all(|&ri| sys.residual_branch_ok(ri as usize, &x))
        };

        // one attempt with the pins at old + alpha·(goal − old); restores on failure
        let trial = |alpha: f64, sc: &mut Scratch, snap: &[(u32, f64)]| -> (bool, bool) {
            for ((v, o), (_, t)) in old.iter().zip(&goal) {
                x.set(*v, o + alpha * (t - o));
            }
            let mut ok = true;
            let mut flipped = false;
            for p in &dc.comps {
                let r = sparse_lm(sys, &p.region, x, sc, &opts, step, None, 15);
                if !r.converged
                    || !p
                        .region
                        .block
                        .res_list
                        .iter()
                        .all(|&ri| sys.residual_valid(ri as usize, &x))
                {
                    ok = false;
                    break;
                }
                if !branch_ok(p) {
                    ok = false;
                    flipped = true;
                    break;
                }
            }
            if !ok {
                restore(snap);
            }
            (ok, flipped)
        };

        let outcome = with_scratch(sys.n_vars, |sc| {
            let snap = snapshot(dc);
            let mut limited = false;
            if !dc.prefer_soft.get() {
                let (ok, flipped) = trial(1.0, sc, &snap);
                if ok {
                    return DragOutcome {
                        moved: true,
                        reached: true,
                        fraction: 1.0,
                        converged: true,
                        branch_limited: false,
                    };
                }
                limited = flipped;
            }

            // Soft stage: slide to the closest feasible position.
            let pin_goal: Vec<(u32, f64)> = goal.iter().map(|(v, t)| (*v as u32, *t)).collect();
            let mut soft_ok = true;
            for p in &dc.comps {
                let tg: Vec<(u32, f64)> = pin_goal
                    .iter()
                    .copied()
                    .filter(|(g, _)| p.soft.block.cols.binary_search(g).is_ok())
                    .collect();
                if !tg.is_empty() {
                    let soft = Soft {
                        targets: &tg,
                        w: 1e-3,
                    };
                    let _ = sparse_lm(sys, &p.soft, x, sc, &opts, step, Some(&soft), 10);
                }
            }
            for p in &dc.comps {
                let r = sparse_lm(sys, &p.soft, x, sc, &opts, step, None, 25);
                if !r.converged
                    || !p
                        .soft
                        .block
                        .res_list
                        .iter()
                        .all(|&ri| sys.residual_valid(ri as usize, &x))
                {
                    soft_ok = false;
                    break;
                }
                if !branch_ok(p) {
                    soft_ok = false;
                    limited = true;
                }
            }
            if soft_ok {
                let (mut dot, mut tt, mut miss) = (0.0f64, 0.0f64, 0.0f64);
                for ((v, t), (_, o)) in goal.iter().zip(&old) {
                    let (ach, want) = (x.get(*v) - o, t - o);
                    dot += ach * want;
                    tt += want * want;
                    miss = miss.max((x.get(*v) - t).abs());
                }
                let fraction = if tt > 1e-24 {
                    (dot / tt).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                let reached = miss <= 1e-7 * sys.scale.max(1.0);
                dc.prefer_soft.set(!reached);
                let moved = old.iter().any(|(v, o)| (x.get(*v) - o).abs() > 1e-12);
                return DragOutcome {
                    moved,
                    reached,
                    fraction,
                    converged: true,
                    branch_limited: limited,
                };
            }
            restore(&snap);
            dc.prefer_soft.set(false);

            // Last resort: shorten the move by bisection.
            let (mut lo, mut hi) = (0.0f64, 1.0f64);
            for _ in 0..10 {
                let mid = 0.5 * (lo + hi);
                restore(&snap);
                let (ok, flipped) = trial(mid, sc, &snap);
                limited |= flipped;
                if ok {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            restore(&snap);
            if lo > 0.0 {
                let (ok, _) = trial(lo, sc, &snap);
                if ok {
                    return DragOutcome {
                        moved: true,
                        reached: false,
                        fraction: lo,
                        converged: true,
                        branch_limited: limited,
                    };
                }
            }
            restore(&snap);
            DragOutcome {
                moved: false,
                reached: false,
                fraction: 0.0,
                converged: true,
                branch_limited: limited,
            }
        });
        Ok(outcome)
    }
}
