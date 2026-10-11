//! The 3-D verification suite: closed, seeded parts (terrain, domes and cones, terraces) with fixtures beside
//! them. Parts are height fields over a 60 × 60 mm footprint with a flat closed base, so a point-in-solid test
//! by ray parity is exact for them.

use crate::bvh::{Tri, TriMesh};
use crate::P3;

pub const SIZE: f64 = 60.0;
pub const CELLS: usize = 40;
pub const BASE: f64 = 0.0;

pub struct Part {
    pub name: String,
    pub tris: Vec<Tri>,
    pub fixture: Vec<Tri>,
    pub zmax: f64,
}

impl Part {
    pub fn part_mesh(&self) -> TriMesh {
        TriMesh::new(self.tris.clone())
    }

    /// Part and fixtures together, as the tool must avoid both.
    pub fn scene(&self) -> TriMesh {
        let mut t = self.tris.clone();
        t.extend(self.fixture.iter().copied());
        TriMesh::new(t)
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
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
}

/// Closed solid under a height function: top grid, flat bottom grid, four side walls.
pub fn heightfield_solid(f: impl Fn(f64, f64) -> f64, n: usize, size: f64, base: f64) -> Vec<Tri> {
    let h = size / n as f64;
    let top =
        |i: usize, j: usize| -> P3 { [i as f64 * h, j as f64 * h, f(i as f64 * h, j as f64 * h)] };
    let bot = |i: usize, j: usize| -> P3 { [i as f64 * h, j as f64 * h, base] };
    let mut t: Vec<Tri> = Vec::new();
    for i in 0..n {
        for j in 0..n {
            // top, counter-clockwise seen from above
            t.push([top(i, j), top(i + 1, j), top(i + 1, j + 1)]);
            t.push([top(i, j), top(i + 1, j + 1), top(i, j + 1)]);
            // bottom, facing down
            t.push([bot(i, j), bot(i + 1, j + 1), bot(i + 1, j)]);
            t.push([bot(i, j), bot(i, j + 1), bot(i + 1, j + 1)]);
        }
    }
    for k in 0..n {
        // y = 0 wall (normal −y), y = size wall (+y), x = 0 wall (−x), x = size wall (+x)
        t.push([bot(k, 0), bot(k + 1, 0), top(k + 1, 0)]);
        t.push([bot(k, 0), top(k + 1, 0), top(k, 0)]);
        t.push([bot(k, n), top(k + 1, n), bot(k + 1, n)]);
        t.push([bot(k, n), top(k, n), top(k + 1, n)]);
        t.push([bot(0, k), top(0, k + 1), bot(0, k + 1)]);
        t.push([bot(0, k), top(0, k), top(0, k + 1)]);
        t.push([bot(n, k), bot(n, k + 1), top(n, k + 1)]);
        t.push([bot(n, k), top(n, k + 1), top(n, k)]);
    }
    t
}

fn box_tris(min: P3, max: P3) -> Vec<Tri> {
    TriMesh::from_box(min, max).tris
}

pub fn suite(count: usize, seed: u64) -> Vec<Part> {
    let mut rng = Rng(seed.max(1) | 1);
    (0..count)
        .map(|k| {
            let kind = k % 3;
            let (name, f): (String, Box<dyn Fn(f64, f64) -> f64>) = match kind {
                0 => {
                    let g: Vec<[f64; 4]> = (0..5)
                        .map(|_| {
                            [
                                rng.range(5.0, 55.0),
                                rng.range(5.0, 55.0),
                                rng.range(4.0, 10.0),
                                rng.range(3.0, 12.0),
                            ]
                        })
                        .collect();
                    (
                        format!("terrain {k}"),
                        Box::new(move |x, y| {
                            2.0 + g
                                .iter()
                                .map(|p| {
                                    p[3] * (-((x - p[0]).powi(2) + (y - p[1]).powi(2))
                                        / (2.0 * p[2] * p[2]))
                                        .exp()
                                })
                                .sum::<f64>()
                        }),
                    )
                }
                1 => {
                    // domes and cones: the upper envelope of a few round solids
                    let s: Vec<[f64; 5]> = (0..3)
                        .map(|_| {
                            [
                                rng.range(10.0, 50.0),
                                rng.range(10.0, 50.0),
                                rng.range(8.0, 18.0),
                                rng.range(0.0, 1.0),
                                rng.range(0.6, 1.4),
                            ]
                        })
                        .collect();
                    (
                        format!("domes and cones {k}"),
                        Box::new(move |x, y| {
                            s.iter()
                                .map(|p| {
                                    let r = ((x - p[0]).powi(2) + (y - p[1]).powi(2)).sqrt();
                                    if p[3] < 0.5 {
                                        (p[4] * (p[2] * p[2] - r * r).max(0.0).sqrt()).max(0.0)
                                    } else {
                                        (p[4] * (p[2] - r)).max(0.0)
                                    }
                                })
                                .fold(0.0, f64::max)
                                + 2.0
                        }),
                    )
                }
                _ => {
                    let g: Vec<[f64; 4]> = (0..4)
                        .map(|_| {
                            [
                                rng.range(5.0, 55.0),
                                rng.range(5.0, 55.0),
                                rng.range(6.0, 12.0),
                                rng.range(6.0, 16.0),
                            ]
                        })
                        .collect();
                    (
                        format!("terraces {k}"),
                        Box::new(move |x, y| {
                            let v: f64 = g
                                .iter()
                                .map(|p| {
                                    p[3] * (-((x - p[0]).powi(2) + (y - p[1]).powi(2))
                                        / (2.0 * p[2] * p[2]))
                                        .exp()
                                })
                                .sum();
                            // 3 mm terraces with a one-cell blend between levels
                            let l = (v / 3.0).floor();
                            let frac = v / 3.0 - l;
                            let w = (frac / 0.35).min(1.0);
                            2.0 + 3.0 * (l + w * w * (3.0 - 2.0 * w))
                        }),
                    )
                }
            };
            let tris = heightfield_solid(&f, CELLS, SIZE, BASE);
            let zmax = tris
                .iter()
                .flat_map(|t| t.iter())
                .map(|p| p[2])
                .fold(f64::MIN, f64::max);
            // a vise-jaw-like fixture on each side, at a random height below the part's top
            let jaw = rng.range(6.0, 14.0);
            let mut fixture = box_tris([-30.0, -10.0, BASE], [-5.0, SIZE + 10.0, jaw]);
            fixture.extend(box_tris(
                [SIZE + 5.0, -10.0, BASE],
                [SIZE + 30.0, SIZE + 10.0, jaw],
            ));
            Part {
                name,
                tris,
                fixture,
                zmax,
            }
        })
        .collect()
}
