//! The board in the MCAD workspace: the STEP file re-imported by the kernel must put every component body
//! and its pin-1 marker where the board model says (bottom-side mirroring included); IDF and pick-and-place
//! files read back to the same placement.

use kw_brep::BRep;
use kw_ecad::bench_gen::{benchmark_specs, generate};
use kw_ecad::export::idf::{emn, emp};
use kw_ecad::export::pnp::pick_and_place;
use kw_ecad::export::step::{step, MARKER, MARKER_INSET};
use kw_ecad::Side;

#[test]
fn step_import_places_every_body_where_the_board_model_says() {
    let specs = benchmark_specs();
    let mut worst = 0.0f64;
    let mut bodies_checked = 0;
    for spec in [&specs[2], &specs[5], &specs[11]] {
        let (board, _) = generate(spec);
        assert!(board.parts.iter().any(|p| p.side == Side::Bottom) || spec.seed != 1002);
        let text = step(&board);
        let mut rep = BRep::new();
        let r = kw_step::import(&mut rep, &text, &kw_step::ImportOptions::default()).unwrap();
        assert!(r.failed.is_empty(), "{:?}", r.failed);
        assert_eq!(r.bodies.len(), 1 + 2 * board.parts.len());
        let check = |rep: &BRep, id, lo: [f64; 3], hi: [f64; 3], what: &str| {
            let bb = rep.body_bbox(id);
            let mp = rep.mass_properties(id).unwrap();
            let mut e = 0.0f64;
            for k in 0..3 {
                e = e
                    .max((bb.min[k] - lo[k]).abs())
                    .max((bb.max[k] - hi[k]).abs());
                e = e.max((mp.centroid[k] - 0.5 * (lo[k] + hi[k])).abs());
            }
            assert!(e < 1e-6, "{what}: error {e}");
            e
        };
        worst = worst.max(check(
            &rep,
            r.bodies[0],
            [0.0, 0.0, -board.thickness],
            [board.width, board.height, 0.0],
            "board",
        ));
        for (i, p) in board.parts.iter().enumerate() {
            let fp = &board.footprints[p.footprint];
            let [bx, by, bh] = fp.body;
            // expected extents from the board model's own mapping of footprint points
            let corners: Vec<[f64; 2]> = [
                [-bx / 2.0, -by / 2.0],
                [bx / 2.0, -by / 2.0],
                [bx / 2.0, by / 2.0],
                [-bx / 2.0, by / 2.0],
            ]
            .iter()
            .map(|c| board.to_board(p, *c))
            .collect();
            let (x0, x1) = (
                corners.iter().map(|c| c[0]).fold(f64::MAX, f64::min),
                corners.iter().map(|c| c[0]).fold(f64::MIN, f64::max),
            );
            let (y0, y1) = (
                corners.iter().map(|c| c[1]).fold(f64::MAX, f64::min),
                corners.iter().map(|c| c[1]).fold(f64::MIN, f64::max),
            );
            let (z0, z1) = if p.side == Side::Top {
                (0.0, bh)
            } else {
                (-board.thickness - bh, -board.thickness)
            };
            worst = worst.max(check(
                &rep,
                r.bodies[1 + 2 * i],
                [x0, y0, z0],
                [x1, y1, z1],
                &p.refdes,
            ));
            // pin-1 marker
            let m = board.to_board(p, [-bx / 2.0 + MARKER_INSET, by / 2.0 - MARKER_INSET]);
            let mc = [m[0], m[1]];
            let (rx, ry) = {
                // marker extents after the 0/90 degree turn
                let q = (p.rot / 90.0).round() as i64;
                if q.rem_euclid(2) == 1 {
                    (MARKER[1] / 2.0, MARKER[0] / 2.0)
                } else {
                    (MARKER[0] / 2.0, MARKER[1] / 2.0)
                }
            };
            let zc = if p.side == Side::Top {
                bh + MARKER[2] / 2.0
            } else {
                -board.thickness - bh - MARKER[2] / 2.0
            };
            worst = worst.max(check(
                &rep,
                r.bodies[2 + 2 * i],
                [mc[0] - rx, mc[1] - ry, zc - MARKER[2] / 2.0],
                [mc[0] + rx, mc[1] + ry, zc + MARKER[2] / 2.0],
                &format!("{} pin 1", p.refdes),
            ));
            bodies_checked += 2;
        }
    }
    eprintln!(
        "{bodies_checked} component bodies re-imported, worst placement error {worst:.2e} mm"
    );
}

#[test]
fn idf_and_pick_and_place_read_back_to_the_same_placement() {
    let (board, _) = generate(&benchmark_specs()[2]);
    let n = emn(&board);
    let lib = emp(&board);
    // minimal independent IDF reader
    let sec = |text: &str, name: &str| -> Vec<String> {
        let mut on = false;
        let mut out = Vec::new();
        for l in text.lines() {
            if l.starts_with(&format!(".END_{name}")) {
                on = false;
            }
            if on {
                out.push(l.to_string());
            }
            if l.starts_with(&format!(".{name}")) && !l.starts_with(".END") {
                on = true;
            }
        }
        out
    };
    let pl = sec(&n, "PLACEMENT");
    assert_eq!(pl.len(), 2 * board.parts.len());
    for (k, p) in board.parts.iter().enumerate() {
        let name: Vec<&str> = pl[2 * k].split_whitespace().collect();
        assert_eq!(name[2], p.refdes);
        let v: Vec<&str> = pl[2 * k + 1].split_whitespace().collect();
        let (x, y, rot): (f64, f64, f64) = (
            v[0].parse().unwrap(),
            v[1].parse().unwrap(),
            v[3].parse().unwrap(),
        );
        assert!(
            (x - p.pos[0]).abs() < 1e-4
                && (y - p.pos[1]).abs() < 1e-4
                && (rot - p.rot.rem_euclid(360.0)).abs() < 1e-4
        );
        assert_eq!(v[4], if p.side == Side::Top { "TOP" } else { "BOTTOM" });
        // every placed package has a library entry with its height
        assert!(
            lib.contains(&format!(".ELECTRICAL\n{} {}", name[0], name[1])),
            "{}",
            name[0]
        );
    }
    let outline = sec(&n, "BOARD_OUTLINE");
    assert_eq!(outline.len(), 1 + 5);
    let holes = sec(&n, "DRILLED_HOLES");
    assert_eq!(
        holes.len(),
        board.pads().iter().filter(|p| p.drill.is_some()).count() + board.holes.len()
    );
    // pick and place
    let csv = pick_and_place(&board);
    let rows: Vec<Vec<&str>> = csv
        .lines()
        .skip(1)
        .map(|l| l.split(',').collect())
        .collect();
    assert_eq!(rows.len(), board.parts.len());
    for (r, p) in rows.iter().zip(&board.parts) {
        assert_eq!(r[0], p.refdes);
        assert!((r[3].parse::<f64>().unwrap() - p.pos[0]).abs() < 1e-4);
        assert_eq!(r[6], if p.side == Side::Top { "top" } else { "bottom" });
    }
}
