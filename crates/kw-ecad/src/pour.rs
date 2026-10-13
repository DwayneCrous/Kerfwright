//! Copper pours (zones): the board outline pulled in by the edge clearance, minus every foreign copper item of
//! the layer grown by the clearance. Islands that touch no copper of the pour's net are removed. The net's
//! own pads, tracks and vias stay as they are (a direct connection, no thermal spokes); the pour adds area to
//! them, so the net's connectivity is that of its routed copper.

use crate::board::{Board, Zone};
use crate::shape::{copper_items, distance, Item, ItemKind, Shape};
use crate::P2;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;

type Shapes = Vec<Vec<Vec<P2>>>;

/// Counter-clockwise capsule polygon that circumscribes the stadium (a, b, r).
fn capsule(a: P2, b: P2, r: f64) -> Vec<P2> {
    let m = 10usize;
    let rr = r / (std::f64::consts::PI / (2.0 * m as f64)).cos();
    let d = [b[0] - a[0], b[1] - a[1]];
    let len = d[0].hypot(d[1]);
    let phi = if len < 1e-12 { 0.0 } else { d[1].atan2(d[0]) };
    let mut v = Vec::new();
    if len < 1e-12 {
        for k in 0..2 * m {
            let t = std::f64::consts::PI * k as f64 / m as f64
                + std::f64::consts::PI / (2.0 * m as f64);
            v.push([a[0] + rr * t.cos(), a[1] + rr * t.sin()]);
        }
        return v;
    }
    // around b from phi - 90 deg to phi + 90 deg, then around a
    for k in 0..=m {
        let t = phi - std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / m as f64;
        v.push([b[0] + rr * t.cos(), b[1] + rr * t.sin()]);
    }
    for k in 0..=m {
        let t = phi + std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / m as f64;
        v.push([a[0] + rr * t.cos(), a[1] + rr * t.sin()]);
    }
    v
}

fn signed_area(c: &[P2]) -> f64 {
    let n = c.len();
    0.5 * (0..n)
        .map(|i| c[i][0] * c[(i + 1) % n][1] - c[(i + 1) % n][0] * c[i][1])
        .sum::<f64>()
}

fn ccw(mut c: Vec<P2>) -> Vec<P2> {
    if signed_area(&c) < 0.0 {
        c.reverse();
    }
    c
}

/// Contours (to be united) covering everything within `g` of the shape.
fn grown(shape: &Shape, g: f64) -> Vec<Vec<P2>> {
    match shape {
        Shape::Cap { a, b, r } => vec![capsule(*a, *b, r + g)],
        Shape::Poly(p) => {
            let mut v = vec![ccw(p.clone())];
            for i in 0..p.len() {
                v.push(capsule(p[i], p[(i + 1) % p.len()], g));
            }
            v
        }
        Shape::Region(_) => Vec::new(),
    }
}

#[derive(Debug, Clone)]
pub struct PourReport {
    pub area: f64,
    pub shapes: usize,
    pub islands_removed: usize,
}

/// Fill `layer` with copper of `net`.
pub fn pour(board: &mut Board, net: usize, layer: usize) -> PourReport {
    board.zones.retain(|z| !(z.net == net && z.layer == layer));
    let rules = board.rules.clone();
    let g = rules.clearance + 0.01;
    let items: Vec<Item> = copper_items(board);
    let on_layer = |it: &Item| it.layer.is_none() || it.layer == Some(layer);
    let mut foreign: Vec<Vec<P2>> = Vec::new();
    for it in &items {
        if on_layer(it) && it.net != Some(net) && it.kind != ItemKind::Zone {
            foreign.extend(grown(&it.shape, g));
        }
    }
    // mounting holes and the drills of through pads of other nets are covered by their pads; holes need a ring
    for (c, d) in &board.holes {
        foreign.push(capsule(*c, *c, d / 2.0 + g));
    }
    let e = rules.edge_clearance + 0.01;
    let outline = vec![vec![
        [e, e],
        [board.width - e, e],
        [board.width - e, board.height - e],
        [e, board.height - e],
    ]];
    let blocked: Shapes = foreign.overlay(
        &Vec::<Vec<P2>>::new(),
        OverlayRule::Union,
        FillRule::NonZero,
    );
    let flat: Vec<Vec<P2>> = blocked.into_iter().flatten().collect();
    let filled: Shapes = outline.overlay(&flat, OverlayRule::Difference, FillRule::NonZero);
    // keep what touches copper of the net
    let own: Vec<&Item> = items
        .iter()
        .filter(|it| on_layer(it) && it.net == Some(net) && it.kind != ItemKind::Zone)
        .collect();
    let mut kept: Shapes = Vec::new();
    let mut removed = 0;
    for sh in filled {
        let contours: Vec<Vec<P2>> = sh
            .into_iter()
            .enumerate()
            .map(|(i, c)| {
                let c = ccw(c);
                if i == 0 {
                    c
                } else {
                    c.into_iter().rev().collect()
                }
            })
            .collect();
        let region = Shape::Region(contours.clone());
        if own.iter().any(|it| distance(&it.shape, &region) <= 1e-9) {
            kept.push(contours);
        } else {
            removed += 1;
        }
    }
    let area: f64 = kept
        .iter()
        .map(|s| {
            s.iter()
                .enumerate()
                .map(|(i, c)| {
                    if i == 0 {
                        signed_area(c).abs()
                    } else {
                        -signed_area(c).abs()
                    }
                })
                .sum::<f64>()
        })
        .sum();
    let rep = PourReport {
        area,
        shapes: kept.len(),
        islands_removed: removed,
    };
    board.zones.push(Zone {
        net,
        layer,
        shapes: kept,
    });
    rep
}
