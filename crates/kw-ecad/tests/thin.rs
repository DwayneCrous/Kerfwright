use kw_ecad::board::{Board, Side};
use kw_ecad::drc::drc;
use kw_ecad::export::excellon;
use kw_ecad::export::gerber::{gerber, GerberLayer};
use kw_ecad::footprints::*;
use kw_ecad::route::{Router, RouterParams};
use kw_ecad::shape::{copper_items, Shape};
use kw_fabcheck::{excellon::parse_excellon, poly, raster};

fn small_board() -> Board {
    let mut b = Board::new("thin", 40.0, 30.0, 2);
    let r = b.add_footprint(chip_0603());
    let u = b.add_footprint(soic8());
    let h = b.add_footprint(header(4));
    let (vcc, gnd, sig, sig2) = (b.net("VCC"), b.net("GND"), b.net("SIG"), b.net("SIG2"));
    let u1 = b.place("U1", u, [20.0, 15.0], 0.0, Side::Top);
    let r1 = b.place("R1", r, [10.0, 20.0], 0.0, Side::Top);
    let r2 = b.place("R2", r, [30.0, 10.0], 90.0, Side::Top);
    let j1 = b.place("J1", h, [5.0, 15.0], 0.0, Side::Top);
    b.connect(u1, "8", vcc);
    b.connect(u1, "4", gnd);
    b.connect(u1, "2", sig);
    b.connect(u1, "6", sig2);
    b.connect(r1, "1", vcc);
    b.connect(r1, "2", sig);
    b.connect(r2, "1", gnd);
    b.connect(r2, "2", sig2);
    b.connect(j1, "1", vcc);
    b.connect(j1, "2", gnd);
    b.connect(j1, "3", sig);
    b.connect(j1, "4", sig2);
    b
}

#[test]
fn route_check_export_and_view() {
    let board = small_board();
    // placement is clean
    assert!(drc(&board).is_empty(), "{:?}", drc(&board));
    let mut router = Router::new(board, RouterParams::default());
    let rep = router.route_all();
    assert_eq!(rep.nets_routed, rep.nets_total, "{rep:?}");
    let board = router.board;
    let v = drc(&board);
    assert!(
        v.is_empty(),
        "{} violations, first: {:?}",
        v.len(),
        v.first()
    );

    // Gerber: both viewers read every layer; they agree with each other and with the board
    for layer in [
        GerberLayer::Copper(0),
        GerberLayer::Copper(1),
        GerberLayer::Mask(Side::Top),
        GerberLayer::Profile,
    ] {
        let text = gerber(&board, layer);
        let a = poly::parse(&text).unwrap_or_else(|e| panic!("{layer:?}: {e}"));
        let b = raster::render(
            &text,
            -1.0,
            -1.0,
            board.width + 1.0,
            board.height + 1.0,
            0.02,
        )
        .unwrap_or_else(|e| panic!("{layer:?}: {e}"));
        let rel = (a.area() - b.area()).abs() / a.area().max(1e-9);
        assert!(
            rel < 0.01,
            "{layer:?}: polygon area {} vs raster area {}",
            a.area(),
            b.area()
        );
        if let GerberLayer::Copper(l) = layer {
            // native copper on layer l, painted with the same raster code
            let mut native =
                raster::Canvas::new(-1.0, -1.0, board.width + 1.0, board.height + 1.0, 0.02);
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
            assert!(iou > 0.995, "layer {l}: IoU {iou}");
            let bb = a.bounds().unwrap();
            assert!(bb[0] > 0.0 && bb[2] < board.width && bb[1] > 0.0 && bb[3] < board.height);
        }
    }
    // drill file
    let holes = parse_excellon(&excellon::pth(&board)).unwrap();
    let pads = board.pads().iter().filter(|p| p.drill.is_some()).count();
    assert_eq!(holes.len(), pads + board.vias.len());
    assert!(parse_excellon(&excellon::npth(&board)).unwrap().is_empty());
}

#[test]
fn malformed_gerber_is_rejected_by_both_viewers() {
    let board = small_board();
    let good = gerber(&board, GerberLayer::Copper(0));
    assert!(poly::parse(&good).is_ok());
    let cases: Vec<(&str, String)> = vec![
        ("no M02", good.replace("M02*\n", "")),
        ("no MO", good.replace("%MOMM*%\n", "")),
        ("no FS", good.replace("%FSLAX46Y46*%\n", "")),
        ("undefined aperture", good.replace("D10*", "D99*")),
        (
            "unterminated region",
            format!(
                "{}G36*\nX1000000Y1000000D02*\nM02*\n",
                good.replace("M02*\n", "")
            ),
        ),
        ("junk", good.replace("G01*", "Q77*")),
        ("trailing content", format!("{good}X1Y1D03*\n")),
        ("missing star", good.replace("%MOMM*%", "%MOMM%")),
    ];
    for (what, text) in cases {
        assert!(poly::parse(&text).is_err(), "viewer A accepted: {what}");
        assert!(
            raster::render(&text, 0.0, 0.0, 10.0, 10.0, 0.1).is_err(),
            "viewer B accepted: {what}"
        );
    }
    // drill file
    let d = excellon::pth(&board);
    assert!(parse_excellon(&d.replace("M30\n", "")).is_err());
    assert!(parse_excellon(&d.replace("T1\n", "T9\n")).is_err());
}
