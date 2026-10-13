//! The pocket suite the adaptive-clearing acceptance is run on: 50 seeded pockets in four families
//! (convex, concave, with islands, slots), documented by their parameters. The suite is generated, not
//! taken from a standard: the roadmap names a "50-pocket suite" without defining it.

use crate::geom2d::{circle_loop, Region};
use crate::P2;

#[derive(Debug, Clone)]
pub struct PocketCase {
    pub name: String,
    pub family: &'static str,
    pub region: Region,
    /// Tool diameter and radial depth of cut (mm).
    pub tool_diameter: f64,
    pub stepover: f64,
    /// Human-readable parameters.
    pub params: String,
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
    fn pick<T: Copy>(&mut self, v: &[T]) -> T {
        v[(self.next() * v.len() as f64) as usize % v.len()]
    }
}

fn poly(pts: &[P2]) -> Region {
    Region::polygon(pts)
}

fn regular(n: usize, r: f64, rot: f64, cx: f64, cy: f64) -> Vec<P2> {
    (0..n)
        .map(|i| {
            let a = rot + std::f64::consts::TAU * i as f64 / n as f64;
            [cx + r * a.cos(), cy + r * a.sin()]
        })
        .collect()
}

fn star(n: usize, r_out: f64, r_in: f64) -> Vec<P2> {
    (0..2 * n)
        .map(|i| {
            let a = std::f64::consts::PI * i as f64 / n as f64;
            let r = if i % 2 == 0 { r_out } else { r_in };
            [r * a.cos(), r * a.sin()]
        })
        .collect()
}

/// Rectangle with rounded corners as a loop with bulges.
fn rounded_rect(w: f64, h: f64, r: f64) -> Vec<[f64; 3]> {
    let b = (std::f64::consts::FRAC_PI_4 / 2.0).tan() * 2.0 / 2.0 * 1.0; // tan(θ/4) for a quarter circle
    let b = b.max((std::f64::consts::FRAC_PI_2 / 4.0).tan());
    vec![
        [r, 0.0, 0.0],
        [w - r, 0.0, b],
        [w, r, 0.0],
        [w, h - r, b],
        [w - r, h, 0.0],
        [r, h, b],
        [0.0, h - r, 0.0],
        [0.0, r, b],
    ]
}

pub fn pocket_suite(seed: u64) -> Vec<PocketCase> {
    let mut rng = Rng(seed ^ 0x9E37_79B9_7F4A_7C15);
    for _ in 0..8 {
        rng.next();
    }
    let mut out: Vec<PocketCase> = Vec::new();
    let push = |out: &mut Vec<PocketCase>,
                family: &'static str,
                name: String,
                region: Region,
                d: f64,
                ae: f64,
                params: String| {
        out.push(PocketCase {
            name,
            family,
            region,
            tool_diameter: d,
            stepover: ae,
            params,
        });
    };
    // 12 convex pockets
    for i in 0..12 {
        let d = rng.pick(&[6.0, 8.0, 10.0]);
        let ae = d * rng.range(0.14, 0.24);
        match i % 4 {
            0 | 1 => {
                let (w, h) = (rng.range(30.0, 70.0), rng.range(22.0, 50.0));
                push(
                    &mut out,
                    "convex",
                    format!("convex-{i} rectangle"),
                    Region::rect(0.0, 0.0, w, h),
                    d,
                    ae,
                    format!("{w:.1} x {h:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
            2 => {
                let n = 5 + (rng.next() * 4.0) as usize;
                let r = rng.range(18.0, 32.0);
                let rot = rng.range(0.0, 1.0);
                push(
                    &mut out,
                    "convex",
                    format!("convex-{i} {n}-gon"),
                    poly(&regular(n, r, rot, 0.0, 0.0)),
                    d,
                    ae,
                    format!("{n}-gon radius {r:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
            _ => {
                let (w, h) = (rng.range(36.0, 66.0), rng.range(26.0, 46.0));
                let cr = rng.range(5.0, 10.0).min(h / 3.0);
                push(
                    &mut out,
                    "convex",
                    format!("convex-{i} rounded rectangle"),
                    Region::from_loops(&[rounded_rect(w, h, cr)]),
                    d,
                    ae,
                    format!("{w:.1} x {h:.1} corner radius {cr:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
        }
    }
    // 12 concave pockets
    for i in 0..12 {
        let d = rng.pick(&[6.0, 8.0]);
        let ae = d * rng.range(0.14, 0.24);
        let t = rng.range(2.6, 3.2) * d; // arm width
        match i % 4 {
            0 => {
                let a = rng.range(40.0, 60.0).max(t + 2.0 * d);
                let b = rng.range(36.0, 52.0).max(t + 2.0 * d);
                let pts = [[0.0, 0.0], [a, 0.0], [a, t], [t, t], [t, b], [0.0, b]];
                push(
                    &mut out,
                    "concave",
                    format!("concave-{i} L"),
                    poly(&pts),
                    d,
                    ae,
                    format!("L {a:.1} x {b:.1} arm {t:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
            1 => {
                let a = rng.range(44.0, 64.0).max(t + 4.0 * d);
                let b = rng.range(30.0, 44.0).max(t + 2.0 * d);
                let c = (a - t) / 2.0;
                let pts = [
                    [c, 0.0],
                    [c + t, 0.0],
                    [c + t, b - t],
                    [a, b - t],
                    [a, b],
                    [0.0, b],
                    [0.0, b - t],
                    [c, b - t],
                ];
                push(
                    &mut out,
                    "concave",
                    format!("concave-{i} T"),
                    poly(&pts),
                    d,
                    ae,
                    format!("T {a:.1} x {b:.1} arm {t:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
            2 => {
                let r = rng.range(26.0, 36.0);
                let k = 5 + (rng.next() * 2.0) as usize;
                push(
                    &mut out,
                    "concave",
                    format!("concave-{i} {k}-point star"),
                    poly(&star(k, r, 0.45 * r)),
                    d,
                    ae,
                    format!("{k}-point star {r:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
            _ => {
                // the gap between the arms must hold the tool with room to spare
                let a = rng.range(46.0, 62.0).max(2.0 * t + 3.0 * d);
                let b = rng.range(34.0, 48.0).max(t + 2.0 * d);
                let pts = [
                    [0.0, 0.0],
                    [a, 0.0],
                    [a, b],
                    [a - t, b],
                    [a - t, t],
                    [t, t],
                    [t, b],
                    [0.0, b],
                ];
                push(
                    &mut out,
                    "concave",
                    format!("concave-{i} U"),
                    poly(&pts),
                    d,
                    ae,
                    format!("U {a:.1} x {b:.1} arm {t:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
        }
    }
    // 14 pockets with islands
    for i in 0..14 {
        let d = rng.pick(&[6.0, 8.0]);
        let ae = d * rng.range(0.14, 0.24);
        let (w, h) = (rng.range(50.0, 80.0), rng.range(40.0, 60.0));
        let mut region = Region::rect(0.0, 0.0, w, h);
        let n = 1 + i % 3;
        let mut desc = format!("{w:.1} x {h:.1}, tool {d}, a_e {ae:.2}, {n} island(s):");
        for k in 0..n {
            // islands on a coarse grid so that they do not touch each other or the walls
            let cx = w * (k as f64 + 1.0) / (n as f64 + 1.0);
            let cy = h * rng.range(0.35, 0.65);
            if (i + k) % 2 == 0 {
                let r = rng.range(4.0, 8.0);
                region = region.with_island(&circle_loop(cx, cy, r));
                desc += &format!(" circle r {r:.1} at ({cx:.1},{cy:.1});");
            } else {
                let (sx, sy) = (rng.range(6.0, 12.0), rng.range(6.0, 14.0));
                region = region.with_island(&[
                    [cx - sx / 2.0, cy - sy / 2.0, 0.0],
                    [cx + sx / 2.0, cy - sy / 2.0, 0.0],
                    [cx + sx / 2.0, cy + sy / 2.0, 0.0],
                    [cx - sx / 2.0, cy + sy / 2.0, 0.0],
                ]);
                desc += &format!(" rect {sx:.1} x {sy:.1} at ({cx:.1},{cy:.1});");
            }
        }
        push(
            &mut out,
            "islands",
            format!("islands-{i}"),
            region,
            d,
            ae,
            desc,
        );
    }
    // 12 slots: width 2.6 to 4.2 tool diameters
    for i in 0..12 {
        let d = rng.pick(&[6.0, 8.0]);
        let ae = d * rng.range(0.14, 0.22);
        let w = rng.range(2.6, 4.2) * d;
        let l = rng.range(40.0, 90.0);
        match i % 3 {
            0 => push(
                &mut out,
                "slots",
                format!("slot-{i} straight"),
                Region::rect(0.0, 0.0, l, w),
                d,
                ae,
                format!("straight slot {l:.1} x {w:.1}, tool {d}, a_e {ae:.2}"),
            ),
            1 => {
                let r = w / 2.0;
                let loopv = vec![
                    [r, 0.0, 0.0],
                    [l - r, 0.0, 1.0],
                    [l - r, w, 0.0],
                    [r, w, 1.0],
                ];
                push(
                    &mut out,
                    "slots",
                    format!("slot-{i} stadium"),
                    Region::from_loops(&[loopv]),
                    d,
                    ae,
                    format!("stadium slot {l:.1} x {w:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
            _ => {
                let a = rng.range(30.0, 50.0);
                let b = rng.range(25.0, 45.0);
                let pts = [
                    [0.0, 0.0],
                    [a, 0.0],
                    [a, b],
                    [a - w, b],
                    [a - w, w],
                    [0.0, w],
                ];
                push(
                    &mut out,
                    "slots",
                    format!("slot-{i} bent"),
                    poly(&pts),
                    d,
                    ae,
                    format!("bent slot {a:.1} x {b:.1} width {w:.1}, tool {d}, a_e {ae:.2}"),
                );
            }
        }
    }
    out
}
