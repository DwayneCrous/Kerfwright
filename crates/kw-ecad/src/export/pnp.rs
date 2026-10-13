//! Pick-and-place file (CSV): reference, value, package, centre, rotation, side.

use crate::board::{Board, Side};

pub fn pick_and_place(board: &Board) -> String {
    let mut s = String::from("Ref,Val,Package,PosX,PosY,Rot,Side\n");
    for p in &board.parts {
        s.push_str(&format!(
            "{},{},{},{:.4},{:.4},{:.2},{}\n",
            p.refdes,
            if p.value.is_empty() { "~" } else { &p.value },
            board.footprints[p.footprint].name,
            p.pos[0],
            p.pos[1],
            p.rot.rem_euclid(360.0),
            if p.side == Side::Top { "top" } else { "bottom" }
        ));
    }
    s
}
