//! Component placement: simulated annealing on half-perimeter wirelength with hard non-overlap constraints
//! (courtyards plus a gap, board margin), with moves guided towards the centroid of a part's nets (the
//! force-directed element), shifts, swaps, quarter turns and side flips of surface-mount parts.

use crate::board::{Board, Side};
use crate::P2;

#[derive(Debug, Clone)]
pub struct PlaceParams {
    pub seed: u64,
    /// Minimum gap between courtyards (mm).
    pub gap: f64,
    /// Keep-out from the board edge for courtyards (mm).
    pub margin: f64,
    /// Annealing moves per part.
    pub moves_per_part: usize,
    /// Allow flipping surface-mount parts to the other side.
    pub allow_flip: bool,
    /// Wirelength weight of the bottom-top mix (penalty per net with pins on both sides is not used).
    pub t_start: f64,
    pub t_end: f64,
}

impl Default for PlaceParams {
    fn default() -> Self {
        PlaceParams {
            seed: 1,
            gap: 0.8,
            margin: 1.5,
            moves_per_part: 3000,
            allow_flip: false,
            t_start: 4.0,
            t_end: 0.02,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PlaceReport {
    pub hpwl_initial: f64,
    pub hpwl_final: f64,
    pub accepted: usize,
    pub legal: bool,
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    fn below(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n.saturating_sub(1))
    }
}

/// Half sizes of the courtyard of part `i` in board orientation.
fn half(board: &Board, i: usize) -> P2 {
    let p = &board.parts[i];
    let c = board.footprints[p.footprint].courtyard;
    let q = (p.rot / 90.0).round() as i64;
    if q.rem_euclid(2) == 1 {
        [c[1], c[0]]
    } else {
        c
    }
}

fn through_hole(board: &Board, i: usize) -> bool {
    board.footprints[board.parts[i].footprint]
        .pads
        .iter()
        .any(|p| p.drill.is_some())
}

/// Do two parts conflict (courtyards closer than `gap`, on a shared side)?
fn conflict(board: &Board, i: usize, j: usize, gap: f64) -> bool {
    let (a, b) = (&board.parts[i], &board.parts[j]);
    let shares = a.side == b.side || through_hole(board, i) || through_hole(board, j);
    if !shares {
        return false;
    }
    let (ha, hb) = (half(board, i), half(board, j));
    (a.pos[0] - b.pos[0]).abs() < ha[0] + hb[0] + gap
        && (a.pos[1] - b.pos[1]).abs() < ha[1] + hb[1] + gap
}

fn inside(board: &Board, i: usize, margin: f64) -> bool {
    let p = &board.parts[i];
    let h = half(board, i);
    p.pos[0] - h[0] >= margin
        && p.pos[0] + h[0] <= board.width - margin
        && p.pos[1] - h[1] >= margin
        && p.pos[1] + h[1] <= board.height - margin
}

fn fits(board: &Board, i: usize, p: &PlaceParams) -> bool {
    inside(board, i, p.margin)
        && (0..board.parts.len()).all(|j| j == i || !conflict(board, i, j, p.gap))
}

/// Pin positions per net (board coordinates), restricted to the nets a part touches.
fn net_hpwl(board: &Board, net: usize) -> f64 {
    let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    let mut n = 0;
    for part in &board.parts {
        let fp = &board.footprints[part.footprint];
        for (k, pad) in fp.pads.iter().enumerate() {
            if part.nets[k] == Some(net) {
                let q = board.to_board(part, pad.pos);
                x0 = x0.min(q[0]);
                x1 = x1.max(q[0]);
                y0 = y0.min(q[1]);
                y1 = y1.max(q[1]);
                n += 1;
            }
        }
    }
    if n < 2 {
        0.0
    } else {
        (x1 - x0) + (y1 - y0)
    }
}

pub fn total_hpwl(board: &Board) -> f64 {
    (0..board.nets.len()).map(|n| net_hpwl(board, n)).sum()
}

fn nets_of(board: &Board, i: usize) -> Vec<usize> {
    let mut v: Vec<usize> = board.parts[i].nets.iter().flatten().copied().collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Centroid of the other pins of the nets of part `i` (None if it has no connections).
fn centroid(board: &Board, i: usize, signal_only: &[bool]) -> Option<P2> {
    let (mut sx, mut sy, mut n) = (0.0, 0.0, 0.0);
    for net in nets_of(board, i) {
        if !signal_only[net] {
            continue;
        }
        for (j, part) in board.parts.iter().enumerate() {
            if j == i {
                continue;
            }
            for k in 0..part.nets.len() {
                if part.nets[k] == Some(net) {
                    sx += part.pos[0];
                    sy += part.pos[1];
                    n += 1.0;
                    break;
                }
            }
        }
    }
    (n > 0.0).then(|| [sx / n, sy / n])
}

pub fn place(board: &mut Board, p: &PlaceParams) -> PlaceReport {
    let mut rng = Rng(p.seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let n = board.parts.len();
    // 1. a legal start: biggest first, random spots
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|a, b| {
        let (ha, hb) = (half(board, *a), half(board, *b));
        (hb[0] * hb[1]).partial_cmp(&(ha[0] * ha[1])).unwrap()
    });
    let mut placed = vec![false; n];
    let mut legal = true;
    for &i in &order {
        let mut ok = false;
        for _ in 0..20_000 {
            let h = half(board, i);
            if board.width - 2.0 * p.margin < 2.0 * h[0]
                || board.height - 2.0 * p.margin < 2.0 * h[1]
            {
                break;
            }
            board.parts[i].pos = [
                rng.range(p.margin + h[0], board.width - p.margin - h[0]),
                rng.range(p.margin + h[1], board.height - p.margin - h[1]),
            ];
            if (0..n).all(|j| j == i || !placed[j] || !conflict(board, i, j, p.gap))
                && inside(board, i, p.margin)
            {
                ok = true;
                break;
            }
        }
        placed[i] = true;
        if !ok {
            legal = false;
        }
    }
    let hpwl0 = total_hpwl(board);
    if !legal {
        return PlaceReport {
            hpwl_initial: hpwl0,
            hpwl_final: hpwl0,
            accepted: 0,
            legal: false,
        };
    }
    // nets with at least two pins and not huge (power nets are poured or routed wide: they do not pull)
    let mut pins = vec![0usize; board.nets.len()];
    for part in &board.parts {
        for net in part.nets.iter().flatten() {
            pins[*net] += 1;
        }
    }
    let pulling: Vec<bool> = pins.iter().map(|c| *c >= 2 && *c <= 8).collect();
    // 2. annealing
    let total_moves = p.moves_per_part * n.max(1);
    let cool = (p.t_end / p.t_start).powf(1.0 / total_moves.max(1) as f64);
    let mut t = p.t_start;
    let mut cost = total_hpwl(board);
    let mut accepted = 0;
    for _ in 0..total_moves {
        t *= cool;
        let i = rng.below(n);
        let old = board.parts[i].clone();
        let affected = nets_of(board, i);
        let before: f64 = affected.iter().map(|net| net_hpwl(board, *net)).sum();
        let kind = rng.next();
        if kind < 0.45 {
            // pull towards the centroid of the connected parts, with noise
            if let Some(c) = centroid(board, i, &pulling) {
                let k = rng.next();
                let s = 2.0 * t / p.t_start * 6.0 + 0.3;
                board.parts[i].pos = [
                    old.pos[0] + (c[0] - old.pos[0]) * k * 0.5 + rng.range(-s, s),
                    old.pos[1] + (c[1] - old.pos[1]) * k * 0.5 + rng.range(-s, s),
                ];
            }
        } else if kind < 0.75 {
            let s = 1.0 + 12.0 * t / p.t_start;
            board.parts[i].pos = [old.pos[0] + rng.range(-s, s), old.pos[1] + rng.range(-s, s)];
        } else if kind < 0.85 {
            board.parts[i].rot = (old.rot + 90.0).rem_euclid(360.0);
        } else if kind < 0.95 && n > 1 {
            let j = rng.below(n);
            if j != i {
                let oj = board.parts[j].clone();
                let mut all = affected.clone();
                all.extend(nets_of(board, j));
                all.sort_unstable();
                all.dedup();
                let before_all: f64 = all.iter().map(|net| net_hpwl(board, *net)).sum();
                board.parts[i].pos = oj.pos;
                board.parts[j].pos = old.pos;
                if !(fits(board, i, p) && fits(board, j, p)) {
                    board.parts[i] = old;
                    board.parts[j] = oj;
                    continue;
                }
                let after: f64 = all.iter().map(|net| net_hpwl(board, *net)).sum();
                let d = after - before_all;
                if d <= 0.0 || rng.next() < (-d / t).exp() {
                    cost += d;
                    accepted += 1;
                } else {
                    board.parts[i] = old;
                    board.parts[j] = oj;
                }
                continue;
            }
        } else if p.allow_flip && !through_hole(board, i) {
            board.parts[i].side = if old.side == Side::Top {
                Side::Bottom
            } else {
                Side::Top
            };
        }
        if !fits(board, i, p) {
            board.parts[i] = old;
            continue;
        }
        let after: f64 = affected.iter().map(|net| net_hpwl(board, *net)).sum();
        let d = after - before;
        if d <= 0.0 || rng.next() < (-d / t).exp() {
            cost += d;
            accepted += 1;
        } else {
            board.parts[i] = old;
        }
    }
    let _ = cost;
    // snap to a 0.05 mm grid so the exports stay tidy, keeping legality
    for i in 0..n {
        let keep = board.parts[i].pos;
        board.parts[i].pos = [
            (keep[0] / 0.05).round() * 0.05,
            (keep[1] / 0.05).round() * 0.05,
        ];
        if !fits(board, i, p) {
            board.parts[i].pos = keep;
        }
    }
    let legal = (0..n).all(|i| fits(board, i, p));
    PlaceReport {
        hpwl_initial: hpwl0,
        hpwl_final: total_hpwl(board),
        accepted,
        legal,
    }
}
