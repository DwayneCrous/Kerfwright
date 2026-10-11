//! Exact 2-D shapes of board copper and the distances between them.

use crate::board::{Board, PadShape};
use crate::P2;

#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// Stadium: the points within `r` of the segment `a`–`b` (a disc when `a == b`).
    Cap { a: P2, b: P2, r: f64 },
    /// Convex polygon, counter-clockwise.
    Poly(Vec<P2>),
    /// Filled region with holes (outer contours counter-clockwise, holes clockwise).
    Region(Vec<Vec<P2>>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ItemKind {
    Pad,
    Track,
    Via,
    Zone,
}

#[derive(Debug, Clone)]
pub struct Item {
    pub kind: ItemKind,
    pub net: Option<usize>,
    /// Copper layer, or `None` for an item present on every layer (through-hole pads, vias).
    pub layer: Option<usize>,
    pub shape: Shape,
    /// Human-readable owner, e.g. `R3.1` or `track`.
    pub owner: String,
}

impl Shape {
    pub fn bounds(&self) -> [f64; 4] {
        match self {
            Shape::Cap { a, b, r } => [
                a[0].min(b[0]) - r,
                a[1].min(b[1]) - r,
                a[0].max(b[0]) + r,
                a[1].max(b[1]) + r,
            ],
            Shape::Poly(p) => pts_bounds(p.iter()),
            Shape::Region(c) => pts_bounds(c.iter().flatten()),
        }
    }
}

fn pts_bounds<'a>(it: impl Iterator<Item = &'a P2>) -> [f64; 4] {
    let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
    for p in it {
        b[0] = b[0].min(p[0]);
        b[1] = b[1].min(p[1]);
        b[2] = b[2].max(p[0]);
        b[3] = b[3].max(p[1]);
    }
    b
}

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub fn point_seg(p: P2, a: P2, b: P2) -> f64 {
    let ab = sub(b, a);
    let l2 = dot(ab, ab);
    let t = if l2 < 1e-300 {
        0.0
    } else {
        (dot(sub(p, a), ab) / l2).clamp(0.0, 1.0)
    };
    let q = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    (p[0] - q[0]).hypot(p[1] - q[1])
}

fn segs_intersect(a: P2, b: P2, c: P2, d: P2) -> bool {
    let (d1, d2) = (cross(sub(b, a), sub(c, a)), cross(sub(b, a), sub(d, a)));
    let (d3, d4) = (cross(sub(d, c), sub(a, c)), cross(sub(d, c), sub(b, c)));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    let on = |p: P2, q: P2, r: P2| {
        cross(sub(q, p), sub(r, p)).abs() < 1e-12
            && r[0] >= p[0].min(q[0]) - 1e-12
            && r[0] <= p[0].max(q[0]) + 1e-12
            && r[1] >= p[1].min(q[1]) - 1e-12
            && r[1] <= p[1].max(q[1]) + 1e-12
    };
    on(a, b, c) || on(a, b, d) || on(c, d, a) || on(c, d, b)
}

pub fn seg_seg(a: P2, b: P2, c: P2, d: P2) -> f64 {
    if segs_intersect(a, b, c, d) {
        return 0.0;
    }
    point_seg(a, c, d)
        .min(point_seg(b, c, d))
        .min(point_seg(c, a, b))
        .min(point_seg(d, a, b))
}

fn in_convex(p: P2, poly: &[P2]) -> bool {
    let n = poly.len();
    (0..n).all(|i| cross(sub(poly[(i + 1) % n], poly[i]), sub(p, poly[i])) >= -1e-12)
}

/// Winding-number containment in a set of contours (holes have opposite orientation).
pub fn in_region(p: P2, contours: &[Vec<P2>]) -> bool {
    let mut w = 0i32;
    for c in contours {
        let n = c.len();
        for i in 0..n {
            let (a, b) = (c[i], c[(i + 1) % n]);
            if a[1] <= p[1] {
                if b[1] > p[1] && cross(sub(b, a), sub(p, a)) > 0.0 {
                    w += 1;
                }
            } else if b[1] <= p[1] && cross(sub(b, a), sub(p, a)) < 0.0 {
                w -= 1;
            }
        }
    }
    w != 0
}

fn edges(poly: &[P2]) -> impl Iterator<Item = (P2, P2)> + '_ {
    let n = poly.len();
    (0..n).map(move |i| (poly[i], poly[(i + 1) % n]))
}

/// Distance between two shapes (0 when they touch or overlap).
pub fn distance(a: &Shape, b: &Shape) -> f64 {
    match (a, b) {
        (
            Shape::Cap {
                a: a1,
                b: b1,
                r: r1,
            },
            Shape::Cap {
                a: a2,
                b: b2,
                r: r2,
            },
        ) => (seg_seg(*a1, *b1, *a2, *b2) - r1 - r2).max(0.0),
        (Shape::Cap { a: a1, b: b1, r }, Shape::Poly(p))
        | (Shape::Poly(p), Shape::Cap { a: a1, b: b1, r }) => {
            if in_convex(*a1, p) || in_convex(*b1, p) {
                return 0.0;
            }
            (edges(p)
                .map(|(c, d)| seg_seg(*a1, *b1, c, d))
                .fold(f64::MAX, f64::min)
                - r)
                .max(0.0)
        }
        (Shape::Cap { a: a1, b: b1, r }, Shape::Region(c))
        | (Shape::Region(c), Shape::Cap { a: a1, b: b1, r }) => {
            if in_region(*a1, c) || in_region(*b1, c) {
                return 0.0;
            }
            let mut m = f64::MAX;
            for ct in c {
                for (p, q) in edges(ct) {
                    m = m.min(seg_seg(*a1, *b1, p, q));
                }
            }
            (m - r).max(0.0)
        }
        (Shape::Poly(p), Shape::Poly(q)) => {
            if p.iter().any(|v| in_convex(*v, q)) || q.iter().any(|v| in_convex(*v, p)) {
                return 0.0;
            }
            let mut m = f64::MAX;
            for (a1, b1) in edges(p) {
                for (c, d) in edges(q) {
                    m = m.min(seg_seg(a1, b1, c, d));
                }
            }
            m
        }
        (Shape::Poly(p), Shape::Region(c)) | (Shape::Region(c), Shape::Poly(p)) => {
            if p.iter().any(|v| in_region(*v, c)) {
                return 0.0;
            }
            let mut m = f64::MAX;
            for ct in c {
                if ct.iter().any(|v| in_convex(*v, p)) {
                    return 0.0;
                }
                for (a1, b1) in edges(p) {
                    for (c1, d1) in edges(ct) {
                        m = m.min(seg_seg(a1, b1, c1, d1));
                    }
                }
            }
            m
        }
        (Shape::Region(_), Shape::Region(_)) => f64::MAX,
    }
}

/// The shape of a pad centred at `c`, rotated by `angle` degrees.
pub fn pad_shape(shape: PadShape, c: P2, angle: f64) -> Shape {
    let (s, co) = angle.to_radians().sin_cos();
    let rot = |x: f64, y: f64| -> P2 { [c[0] + co * x - s * y, c[1] + s * x + co * y] };
    match shape {
        PadShape::Circle { d } => Shape::Cap {
            a: c,
            b: c,
            r: d / 2.0,
        },
        PadShape::Rect { w, h } => Shape::Poly(vec![
            rot(-w / 2.0, -h / 2.0),
            rot(w / 2.0, -h / 2.0),
            rot(w / 2.0, h / 2.0),
            rot(-w / 2.0, h / 2.0),
        ]),
        PadShape::Oval { w, h } => {
            if w >= h {
                let k = (w - h) / 2.0;
                Shape::Cap {
                    a: rot(-k, 0.0),
                    b: rot(k, 0.0),
                    r: h / 2.0,
                }
            } else {
                let k = (h - w) / 2.0;
                Shape::Cap {
                    a: rot(0.0, -k),
                    b: rot(0.0, k),
                    r: w / 2.0,
                }
            }
        }
    }
}

/// All copper of the board as items (pads, tracks, vias). Zones are added by the pour code.
pub fn copper_items(board: &Board) -> Vec<Item> {
    let mut out = Vec::new();
    for p in board.pads() {
        let part = &board.parts[p.part];
        out.push(Item {
            kind: ItemKind::Pad,
            net: p.net,
            layer: p.layer,
            shape: pad_shape(p.shape, p.pos, p.angle),
            owner: format!(
                "{}.{}",
                part.refdes, board.footprints[part.footprint].pads[p.pad].name
            ),
        });
    }
    for t in &board.tracks {
        out.push(Item {
            kind: ItemKind::Track,
            net: Some(t.net),
            layer: Some(t.layer),
            shape: Shape::Cap {
                a: t.a,
                b: t.b,
                r: t.width / 2.0,
            },
            owner: format!("track@({:.2},{:.2})", t.a[0], t.a[1]),
        });
    }
    for v in &board.vias {
        out.push(Item {
            kind: ItemKind::Via,
            net: Some(v.net),
            layer: None,
            shape: Shape::Cap {
                a: v.pos,
                b: v.pos,
                r: v.dia / 2.0,
            },
            owner: format!("via@({:.2},{:.2})", v.pos[0], v.pos[1]),
        });
    }
    for z in &board.zones {
        for (k, sh) in z.shapes.iter().enumerate() {
            out.push(Item {
                kind: ItemKind::Zone,
                net: Some(z.net),
                layer: Some(z.layer),
                shape: Shape::Region(sh.clone()),
                owner: format!("zone{}.{}#{k}", z.net, z.layer),
            });
        }
    }
    out
}
