//! The 20-board benchmark: seeded placement, routing, independent DRC, and the fabrication outputs read back
//! by two viewers.

use kw_ecad::bench_gen::{benchmark_specs, generate};
use kw_ecad::drc::drc;
use kw_ecad::export::excellon;
use kw_ecad::export::gerber::{gerber, GerberLayer};
use kw_ecad::route::{Router, RouterParams};
use kw_ecad::shape::{copper_items, Shape};
use kw_ecad::Side;
use kw_fabcheck::{excellon::parse_excellon, poly, raster};

/// All boards in an optimised build; the first six in a debug build (the router is 10-20x slower there).
fn count() -> usize {
    if cfg!(debug_assertions) {
        6
    } else {
        20
    }
}

#[test]
fn boards_place_legally_and_route_with_no_violations() {
    let (mut routed, mut total, mut viol) = (0usize, 0usize, 0usize);
    let mut worst_board = 1.0f64;
    for spec in benchmark_specs().iter().take(count()) {
        let (b, pr) = generate(spec);
        assert!(pr.legal, "{}", spec.describe());
        assert!(pr.hpwl_final < pr.hpwl_initial);
        let t = std::time::Instant::now();
        let mut r = Router::new(b, RouterParams::default());
        let rep = r.route_all();
        let v = drc(&r.board);
        eprintln!(
            "{} | {} parts {} nets: {}/{} routed ({:.1} %), {} tracks {} vias, HPWL {:.0} -> {:.0}, DRC {}, {:.1} s",
            spec.describe(),
            r.board.parts.len(),
            r.board.nets.len(),
            rep.nets_routed,
            rep.nets_total,
            100.0 * rep.completion(),
            rep.tracks,
            rep.vias,
            pr.hpwl_initial,
            pr.hpwl_final,
            v.len(),
            t.elapsed().as_secs_f64()
        );
        assert!(v.is_empty(), "{}: {:?}", spec.describe(), v.first());
        // connectivity from the finished geometry alone must agree with the router's own count
        let opens = kw_ecad::open_nets(&r.board);
        assert_eq!(
            opens.len(),
            rep.nets_total - rep.nets_routed,
            "{}: {:?} vs failed {:?}",
            spec.describe(),
            opens,
            rep.failed
        );
        routed += rep.nets_routed;
        total += rep.nets_total;
        viol += v.len();
        worst_board = worst_board.min(rep.completion());
    }
    eprintln!(
        "total: {routed}/{total} nets ({:.2} %), worst board {:.1} %, {viol} DRC violations",
        100.0 * routed as f64 / total as f64,
        100.0 * worst_board
    );
    assert!(routed as f64 >= 0.98 * total as f64);
    assert!(worst_board >= 0.98);
    assert_eq!(viol, 0);
}

#[test]
fn routed_benchmark_boards_export_gerber_and_drill_files_that_two_viewers_read() {
    for spec in benchmark_specs()
        .iter()
        .take(if cfg!(debug_assertions) { 2 } else { 6 })
    {
        let (b, _) = generate(spec);
        let mut r = Router::new(b, RouterParams::default());
        r.route_all();
        let board = r.board;
        let (x0, y0, x1, y1) = (-1.0, -1.0, board.width + 1.0, board.height + 1.0);
        let mut layers: Vec<GerberLayer> = (0..board.layers).map(GerberLayer::Copper).collect();
        layers.push(GerberLayer::Mask(Side::Top));
        layers.push(GerberLayer::Profile);
        for layer in layers {
            let text = gerber(&board, layer);
            let a = poly::parse(&text).unwrap_or_else(|e| panic!("{} {layer:?}: {e}", board.name));
            let b = raster::render(&text, x0, y0, x1, y1, 0.02)
                .unwrap_or_else(|e| panic!("{} {layer:?}: {e}", board.name));
            let rel = (a.area() - b.area()).abs() / a.area().max(1e-9);
            assert!(
                rel < 0.01,
                "{} {layer:?}: polygon viewer {} mm2 vs raster viewer {} mm2",
                board.name,
                a.area(),
                b.area()
            );
            if let GerberLayer::Copper(l) = layer {
                let mut native = raster::Canvas::new(x0, y0, x1, y1, 0.02);
                for it in copper_items(&board) {
                    if it.layer.is_none() || it.layer == Some(l) {
                        match &it.shape {
                            Shape::Cap { a, b, r } => native.paint_cap(*a, *b, *r, 1),
                            Shape::Poly(p) => native.paint_poly(p, 1),
                            Shape::Region(_) => {}
                        }
                    }
                }
                let iou = b.iou(&native);
                assert!(iou > 0.995, "{} layer {l}: IoU {iou}", board.name);
            }
        }
        let holes = parse_excellon(&excellon::pth(&board)).unwrap();
        let pads = board.pads().iter().filter(|p| p.drill.is_some()).count();
        assert_eq!(holes.len(), pads + board.vias.len(), "{}", board.name);
    }
}

#[test]
fn a_ground_pour_keeps_the_clearances_and_both_viewers_see_it() {
    for idx in [0usize, 3] {
        let spec = &benchmark_specs()[idx];
        let (b, _) = generate(spec);
        let mut r = Router::new(b, RouterParams::default());
        r.route_all();
        let mut board = r.board;
        let gnd = board.nets.iter().position(|n| n == "GND").unwrap();
        let bottom = board.layers - 1;
        let before = gerber(&board, GerberLayer::Copper(bottom));
        let area_before = poly::parse(&before).unwrap().area();
        let rep = kw_ecad::pour::pour(&mut board, gnd, bottom);
        let v = drc(&board);
        assert!(
            v.is_empty(),
            "{} violations after the pour, first {:?}",
            v.len(),
            v.first()
        );
        let text = gerber(&board, GerberLayer::Copper(bottom));
        let a = poly::parse(&text).unwrap();
        let rb = raster::render(
            &text,
            -1.0,
            -1.0,
            board.width + 1.0,
            board.height + 1.0,
            0.02,
        )
        .unwrap();
        let rel = (a.area() - rb.area()).abs() / a.area();
        eprintln!("{}: pour {:.0} mm2 in {} shapes ({} islands dropped); bottom copper {:.0} -> {:.0} mm2 (viewer A), viewer B {:.0} mm2 ({:.2} % apart)", board.name, rep.area, rep.shapes, rep.islands_removed, area_before, a.area(), rb.area(), 100.0 * rel);
        assert!(rep.area > 0.0 && a.area() > area_before + 0.5 * rep.area);
        assert!(rel < 0.01);
    }
}
