//! IDF 3.0: the board file (`.emn`: outline, holes, placement) and the library file (`.emp`: component
//! outlines and heights).
//!
//! Convention used for the bottom side (written from the published format description as remembered, and not
//! checked against a third-party IDF reader): the library outline is mirrored about the y axis, then turned
//! counter-clockwise by the written angle as seen from the top of the board.

use crate::board::{Board, Side};

fn header(kind: &str, board: &Board) -> String {
    format!(
        ".HEADER\n{kind} 3.0 \"Kerfwright\" 2026/10/11.00:00:00 1\n\"{}\" MM\n.END_HEADER\n",
        board.name
    )
}

pub fn emn(board: &Board) -> String {
    let mut s = header("BOARD_FILE", board);
    s.push_str(".BOARD_OUTLINE UNOWNED\n");
    s.push_str(&format!("{:.4}\n", board.thickness));
    let o = board.outline();
    for (i, p) in o.iter().enumerate() {
        s.push_str(&format!("0 {:.4} {:.4} 0\n", p[0], p[1]));
        if i == 3 {
            s.push_str(&format!("0 {:.4} {:.4} 0\n", o[0][0], o[0][1]));
        }
    }
    s.push_str(".END_BOARD_OUTLINE\n");
    s.push_str(".DRILLED_HOLES\n");
    for p in board.pads() {
        if let Some(d) = p.drill {
            s.push_str(&format!(
                "{:.4} {:.4} {:.4} PTH {} PIN {}\n",
                d, p.pos[0], p.pos[1], "UNOWNED", board.parts[p.part].refdes
            ));
        }
    }
    for v in &board.vias {
        s.push_str(&format!(
            "{:.4} {:.4} {:.4} PTH UNOWNED VIA UNOWNED\n",
            v.drill, v.pos[0], v.pos[1]
        ));
    }
    for (c, d) in &board.holes {
        s.push_str(&format!(
            "{:.4} {:.4} {:.4} NPTH UNOWNED MTG UNOWNED\n",
            d, c[0], c[1]
        ));
    }
    s.push_str(".END_DRILLED_HOLES\n");
    s.push_str(".PLACEMENT\n");
    for p in &board.parts {
        let fp = &board.footprints[p.footprint];
        s.push_str(&format!("\"{}\" \"{}\" {}\n", fp.name, fp.name, p.refdes));
        s.push_str(&format!(
            "{:.4} {:.4} 0.0000 {:.4} {} PLACED\n",
            p.pos[0],
            p.pos[1],
            p.rot.rem_euclid(360.0),
            if p.side == Side::Top { "TOP" } else { "BOTTOM" }
        ));
    }
    s.push_str(".END_PLACEMENT\n");
    s
}

pub fn emp(board: &Board) -> String {
    let mut s = header("LIBRARY_FILE", board);
    for fp in &board.footprints {
        s.push_str(&format!(
            ".ELECTRICAL\n\"{}\" \"{}\" MM {:.4}\n",
            fp.name, fp.name, fp.body[2]
        ));
        let (hx, hy) = (fp.body[0] / 2.0, fp.body[1] / 2.0);
        for (i, (x, y)) in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)]
            .iter()
            .enumerate()
        {
            let _ = i;
            s.push_str(&format!("0 {x:.4} {y:.4} 0\n"));
        }
        s.push_str(".END_ELECTRICAL\n");
    }
    s
}
