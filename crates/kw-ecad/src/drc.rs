//! Design-rule checking. Independent of the router: it works on the exact shapes of the finished board
//! (stadiums, convex polygons), with an R-tree for the broad phase.

use crate::board::Board;
use crate::shape::{copper_items, distance, Item, ItemKind, Shape};
use rstar::primitives::{GeomWithData, Rectangle};
use rstar::{RTree, AABB};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViolationKind {
    Clearance,
    TrackWidth,
    AnnularRing,
    DrillSize,
    EdgeClearance,
    HoleToHole,
    Courtyard,
}

#[derive(Debug, Clone)]
pub struct Violation {
    pub kind: ViolationKind,
    pub a: String,
    pub b: String,
    pub at: [f64; 2],
    pub value: f64,
    pub required: f64,
}

const EPS: f64 = 1e-6;

fn item_layers(i: &Item, layers: usize) -> Vec<usize> {
    match i.layer {
        Some(l) => vec![l],
        None => (0..layers).collect(),
    }
}

fn centre(s: &Shape) -> [f64; 2] {
    let b = s.bounds();
    [(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0]
}

pub fn drc(board: &Board) -> Vec<Violation> {
    let r = &board.rules;
    let mut out = Vec::new();
    let items = copper_items(board);
    // ---- clearance between different nets on a shared layer
    let tree: RTree<GeomWithData<Rectangle<[f64; 2]>, usize>> = RTree::bulk_load(
        items
            .iter()
            .enumerate()
            .map(|(i, it)| {
                let b = it.shape.bounds();
                GeomWithData::new(Rectangle::from_corners([b[0], b[1]], [b[2], b[3]]), i)
            })
            .collect(),
    );
    for (i, a) in items.iter().enumerate() {
        let b = a.shape.bounds();
        let env = AABB::from_corners(
            [b[0] - r.clearance, b[1] - r.clearance],
            [b[2] + r.clearance, b[3] + r.clearance],
        );
        let la = item_layers(a, board.layers);
        for hit in tree.locate_in_envelope_intersecting(&env) {
            let j = hit.data;
            if j <= i {
                continue;
            }
            let c = &items[j];
            // same net: connected by design; unconnected pads (None) count as different nets
            if a.net.is_some() && a.net == c.net {
                continue;
            }
            let lc = item_layers(c, board.layers);
            if !la.iter().any(|l| lc.contains(l)) {
                continue;
            }
            let d = distance(&a.shape, &c.shape);
            if d < r.clearance - EPS {
                out.push(Violation {
                    kind: ViolationKind::Clearance,
                    a: a.owner.clone(),
                    b: c.owner.clone(),
                    at: centre(&a.shape),
                    value: d,
                    required: r.clearance,
                });
            }
        }
    }
    // ---- track width, edge clearance
    for it in &items {
        if it.kind == ItemKind::Track {
            if let Shape::Cap { r: rad, .. } = it.shape {
                if rad * 2.0 < r.min_track - EPS {
                    out.push(Violation {
                        kind: ViolationKind::TrackWidth,
                        a: it.owner.clone(),
                        b: String::new(),
                        at: centre(&it.shape),
                        value: rad * 2.0,
                        required: r.min_track,
                    });
                }
            }
        }
        let b = it.shape.bounds();
        let edge = b[0]
            .min(b[1])
            .min(board.width - b[2])
            .min(board.height - b[3]);
        if edge < r.edge_clearance - EPS {
            out.push(Violation {
                kind: ViolationKind::EdgeClearance,
                a: it.owner.clone(),
                b: "board edge".into(),
                at: centre(&it.shape),
                value: edge,
                required: r.edge_clearance,
            });
        }
    }
    // ---- drills: size, annular ring, hole to hole
    struct Hole {
        owner: String,
        c: [f64; 2],
        drill: f64,
    }
    let mut holes: Vec<Hole> = Vec::new();
    for p in board.pads() {
        if let Some(d) = p.drill {
            let part = &board.parts[p.part];
            let (hx, hy) = p.shape.half_extents();
            let ring = (hx.min(hy) * 2.0 - d) / 2.0;
            let owner = format!(
                "{}.{}",
                part.refdes, board.footprints[part.footprint].pads[p.pad].name
            );
            if ring < r.min_annular - EPS {
                out.push(Violation {
                    kind: ViolationKind::AnnularRing,
                    a: owner.clone(),
                    b: String::new(),
                    at: p.pos,
                    value: ring,
                    required: r.min_annular,
                });
            }
            holes.push(Hole {
                owner,
                c: p.pos,
                drill: d,
            });
        }
    }
    for v in &board.vias {
        let ring = (v.dia - v.drill) / 2.0;
        let owner = format!("via@({:.2},{:.2})", v.pos[0], v.pos[1]);
        if ring < r.min_annular - EPS {
            out.push(Violation {
                kind: ViolationKind::AnnularRing,
                a: owner.clone(),
                b: String::new(),
                at: v.pos,
                value: ring,
                required: r.min_annular,
            });
        }
        holes.push(Hole {
            owner,
            c: v.pos,
            drill: v.drill,
        });
    }
    for (k, (c, d)) in board.holes.iter().enumerate() {
        holes.push(Hole {
            owner: format!("hole{k}"),
            c: *c,
            drill: *d,
        });
    }
    for h in &holes {
        if h.drill < r.min_drill - EPS {
            out.push(Violation {
                kind: ViolationKind::DrillSize,
                a: h.owner.clone(),
                b: String::new(),
                at: h.c,
                value: h.drill,
                required: r.min_drill,
            });
        }
    }
    let hole_gap = 0.25;
    for i in 0..holes.len() {
        for j in i + 1..holes.len() {
            let d = (holes[i].c[0] - holes[j].c[0]).hypot(holes[i].c[1] - holes[j].c[1])
                - (holes[i].drill + holes[j].drill) / 2.0;
            if d < hole_gap - EPS {
                out.push(Violation {
                    kind: ViolationKind::HoleToHole,
                    a: holes[i].owner.clone(),
                    b: holes[j].owner.clone(),
                    at: holes[i].c,
                    value: d,
                    required: hole_gap,
                });
            }
        }
    }
    // ---- courtyards (same side)
    for i in 0..board.parts.len() {
        for j in i + 1..board.parts.len() {
            let (a, b) = (&board.parts[i], &board.parts[j]);
            if a.side != b.side {
                continue;
            }
            let ca = courtyard(board, i);
            let cb = courtyard(board, j);
            if distance(&Shape::Poly(ca), &Shape::Poly(cb)) <= 0.0 {
                out.push(Violation {
                    kind: ViolationKind::Courtyard,
                    a: a.refdes.clone(),
                    b: b.refdes.clone(),
                    at: a.pos,
                    value: 0.0,
                    required: 0.0,
                });
            }
        }
    }
    out
}

/// The courtyard rectangle of a part in board coordinates.
pub fn courtyard(board: &Board, part: usize) -> Vec<[f64; 2]> {
    let p = &board.parts[part];
    let c = board.footprints[p.footprint].courtyard;
    [[-c[0], -c[1]], [c[0], -c[1]], [c[0], c[1]], [-c[0], c[1]]]
        .iter()
        .map(|q| board.to_board(p, *q))
        .collect::<Vec<_>>()
        .into_iter()
        .collect::<Vec<_>>()
        .pipe_ccw()
}

trait Ccw {
    fn pipe_ccw(self) -> Vec<[f64; 2]>;
}
impl Ccw for Vec<[f64; 2]> {
    fn pipe_ccw(mut self) -> Vec<[f64; 2]> {
        let n = self.len();
        let area: f64 = (0..n)
            .map(|i| self[i][0] * self[(i + 1) % n][1] - self[(i + 1) % n][0] * self[i][1])
            .sum();
        if area < 0.0 {
            self.reverse();
        }
        self
    }
}

/// Nets whose copper does not connect all their pads, found from the finished geometry alone (items of a net
/// are joined when their shapes touch on a shared layer). Returns `(net, number of separate pieces holding
/// pads)`; independent of what the router believes.
pub fn open_nets(board: &Board) -> Vec<(usize, usize)> {
    let items = copper_items(board);
    let mut out = Vec::new();
    for net in 0..board.nets.len() {
        let mine: Vec<&Item> = items.iter().filter(|i| i.net == Some(net)).collect();
        let pads: Vec<usize> = mine
            .iter()
            .enumerate()
            .filter(|(_, i)| i.kind == ItemKind::Pad)
            .map(|(k, _)| k)
            .collect();
        if pads.len() < 2 {
            continue;
        }
        let n = mine.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(p: &mut [usize], x: usize) -> usize {
            let mut r = x;
            while p[r] != r {
                r = p[r];
            }
            let mut c = x;
            while p[c] != r {
                let nx = p[c];
                p[c] = r;
                c = nx;
            }
            r
        }
        for i in 0..n {
            let bi = mine[i].shape.bounds();
            for j in i + 1..n {
                let bj = mine[j].shape.bounds();
                if bi[0] > bj[2] + 1e-6
                    || bj[0] > bi[2] + 1e-6
                    || bi[1] > bj[3] + 1e-6
                    || bj[1] > bi[3] + 1e-6
                {
                    continue;
                }
                let share = match (mine[i].layer, mine[j].layer) {
                    (Some(a), Some(b)) => a == b,
                    _ => true,
                };
                if share && distance(&mine[i].shape, &mine[j].shape) <= 1e-6 {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    parent[a] = b;
                }
            }
        }
        let mut roots: Vec<usize> = pads.iter().map(|k| find(&mut parent, *k)).collect();
        roots.sort_unstable();
        roots.dedup();
        if roots.len() > 1 {
            out.push((net, roots.len()));
        }
    }
    out
}
