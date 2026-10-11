//! Seeded benchmark boards: footprints, a netlist with locality, and a placement.

use crate::board::{Board, Side};
use crate::footprints::*;
use crate::place::{place, PlaceParams, PlaceReport};

/// Density parameters of a benchmark board; all are printed with the results.
#[derive(Debug, Clone)]
pub struct BoardSpec {
    pub seed: u64,
    pub layers: usize,
    pub width: f64,
    pub height: f64,
    pub passives: usize,
    pub soic: usize,
    pub qfp: usize,
    pub headers: usize,
    /// Fraction of chip pads that carry a signal net (the rest: power, ground or unconnected).
    pub signal_pin_fraction: f64,
    /// Clearance rule override.
    pub clearance: f64,
    pub track_width: f64,
    pub bottom_passives: bool,
}

impl BoardSpec {
    pub fn describe(&self) -> String {
        format!(
            "seed {} {}L {}x{} mm, {} passives {} SOIC8 {} QFP32 {} headers, signal pins {:.0} %, clearance {} track {}{}",
            self.seed,
            self.layers,
            self.width,
            self.height,
            self.passives,
            self.soic,
            self.qfp,
            self.headers,
            100.0 * self.signal_pin_fraction,
            self.clearance,
            self.track_width,
            if self.bottom_passives { ", passives on both sides" } else { "" }
        )
    }
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn below(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n.saturating_sub(1))
    }
}

/// Generate a board: parts, nets (GND, VCC and signal nets of 2-5 pins formed among parts that are near each
/// other in a logical ordering), then placement.
pub fn generate(spec: &BoardSpec) -> (Board, PlaceReport) {
    let mut rng = Rng(spec.seed.wrapping_mul(0x2545_F491_4F6C_DD1D) | 1);
    let mut b = Board::new(
        &format!("bench{}", spec.seed),
        spec.width,
        spec.height,
        spec.layers,
    );
    b.rules.clearance = spec.clearance;
    b.rules.track_width = spec.track_width;
    let chip = b.add_footprint(chip_0603());
    let soic = b.add_footprint(soic8());
    let qfp = b.add_footprint(qfp32());
    let hdr = b.add_footprint(header(6));
    // parts in logical order: chips are interleaved with the passives that belong to them
    let mut parts: Vec<usize> = Vec::new();
    for i in 0..spec.qfp {
        parts.push(b.place(&format!("U{}", i + 1), qfp, [0.0, 0.0], 0.0, Side::Top));
    }
    for i in 0..spec.soic {
        parts.push(b.place(
            &format!("U{}", spec.qfp + i + 1),
            soic,
            [0.0, 0.0],
            0.0,
            Side::Top,
        ));
    }
    for i in 0..spec.headers {
        parts.push(b.place(&format!("J{}", i + 1), hdr, [0.0, 0.0], 0.0, Side::Top));
    }
    for i in 0..spec.passives {
        let side = if spec.bottom_passives && rng.next() < 0.3 {
            Side::Bottom
        } else {
            Side::Top
        };
        parts.push(b.place(&format!("R{}", i + 1), chip, [0.0, 0.0], 0.0, side));
    }
    let gnd = b.net("GND");
    let vcc = b.net("VCC");
    // pad pool: (part, pad name)
    let mut pool: Vec<(usize, String)> = Vec::new();
    for &pi in &parts {
        let fp = &b.footprints[b.parts[pi].footprint];
        let is_chip = fp.pads.len() >= 8;
        let names: Vec<String> = fp.pads.iter().map(|p| p.name.clone()).collect();
        let n = names.len();
        for (k, name) in names.iter().enumerate() {
            if is_chip && n >= 8 {
                // power and ground pins at fixed positions
                if k == 0 {
                    b.connect(pi, name, gnd);
                    continue;
                }
                if k == n / 2 {
                    b.connect(pi, name, vcc);
                    continue;
                }
                if n > 8 && (k % 8 == 4) {
                    b.connect(pi, name, if k % 16 == 4 { gnd } else { vcc });
                    continue;
                }
                if rng.next() < spec.signal_pin_fraction {
                    pool.push((pi, name.clone()));
                }
            } else if n == 2 {
                // passive: one side to ground or power, the other a signal
                if k == 0 {
                    if rng.next() < 0.5 {
                        b.connect(pi, name, gnd);
                    } else {
                        pool.push((pi, name.clone()));
                    }
                } else {
                    pool.push((pi, name.clone()));
                }
            } else {
                // header: first pin ground, second power
                if k == 0 {
                    b.connect(pi, name, gnd);
                } else if k == 1 {
                    b.connect(pi, name, vcc);
                } else {
                    pool.push((pi, name.clone()));
                }
            }
        }
    }
    // signal nets: an anchor pad plus 1-4 pads from parts that are near in the logical order
    let mut taken = vec![false; pool.len()];
    let order_of = |part: usize| parts.iter().position(|p| *p == part).unwrap();
    let mut net_id = 0;
    for start in 0..pool.len() {
        if taken[start] {
            continue;
        }
        let size = {
            let r = rng.next();
            if r < 0.68 {
                2
            } else if r < 0.88 {
                3
            } else if r < 0.96 {
                4
            } else {
                5
            }
        };
        taken[start] = true;
        let anchor = order_of(pool[start].0);
        let mut members = vec![start];
        let mut tries = 0;
        while members.len() < size && tries < 400 {
            tries += 1;
            let span = 3 + tries / 40;
            let target = (anchor as i64 + rng.below(2 * span + 1) as i64 - span as i64)
                .clamp(0, parts.len() as i64 - 1) as usize;
            let cand: Vec<usize> = (0..pool.len())
                .filter(|k| {
                    !taken[*k]
                        && order_of(pool[*k].0) == target
                        && !members.iter().any(|m| pool[*m].0 == pool[*k].0)
                })
                .collect();
            if cand.is_empty() {
                continue;
            }
            let k = cand[rng.below(cand.len())];
            taken[k] = true;
            members.push(k);
        }
        if members.len() >= 2 {
            let n = b.net(&format!("N{net_id}"));
            net_id += 1;
            for m in members {
                let (pi, name) = pool[m].clone();
                b.connect(pi, &name, n);
            }
        }
        // single leftover pads stay unconnected
    }
    let rep = place(
        &mut b,
        &PlaceParams {
            seed: spec.seed,
            allow_flip: false,
            ..PlaceParams::default()
        },
    );
    (b, rep)
}

/// The 20 benchmark boards: part counts and the share of the board the parts occupy (including gaps) rise
/// with the index, from 28 % to 46 %.
pub fn benchmark_specs() -> Vec<BoardSpec> {
    (0..20)
        .map(|i| {
            let f = i as f64 / 19.0;
            let layers = if i % 4 == 3 { 4 } else { 2 };
            let passives = 24 + (f * 56.0) as usize;
            let soic = 3 + (f * 4.0) as usize;
            let qfp = [0, 1, 1, 2][i % 4];
            let headers = 1 + (i % 3);
            // courtyard areas with the 0.8 mm placement gap: 0603 9.1, SOIC8 56.8, QFP32 144, 6-pin header 62
            let parts_area = passives as f64 * 9.1
                + soic as f64 * 56.8
                + qfp as f64 * 144.0
                + headers as f64 * 62.0;
            let occupancy = 0.28 + 0.18 * f;
            let area = parts_area / occupancy;
            let aspect = 1.3 + 0.2 * ((i % 5) as f64) / 4.0;
            let w = (area * aspect).sqrt().round();
            let h = (area / aspect).sqrt().round();
            BoardSpec {
                seed: 1000 + i as u64,
                layers,
                width: w,
                height: h,
                passives,
                soic,
                qfp,
                headers,
                signal_pin_fraction: 0.55 + 0.25 * f,
                clearance: if i % 5 == 4 { 0.15 } else { 0.2 },
                track_width: if i % 5 == 4 { 0.15 } else { 0.2 },
                bottom_passives: i % 3 == 2,
            }
        })
        .collect()
}
