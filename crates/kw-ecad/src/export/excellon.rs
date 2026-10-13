//! Excellon drill files.

use crate::board::Board;
use std::collections::BTreeMap;

fn fmt(v: f64) -> String {
    format!("{:.3}", v)
}

/// Plated holes (through-hole pad drills and vias).
pub fn pth(board: &Board) -> String {
    let mut holes: Vec<(f64, [f64; 2])> = Vec::new();
    for p in board.pads() {
        if let Some(d) = p.drill {
            holes.push((d, p.pos));
        }
    }
    for v in &board.vias {
        holes.push((v.drill, v.pos));
    }
    write(board, &holes, "Plated,1,4,PTH")
}

/// Non-plated holes (mounting holes).
pub fn npth(board: &Board) -> String {
    let holes: Vec<(f64, [f64; 2])> = board.holes.iter().map(|(p, d)| (*d, *p)).collect();
    write(board, &holes, "NonPlated,1,4,NPTH")
}

fn write(board: &Board, holes: &[(f64, [f64; 2])], function: &str) -> String {
    let mut tools: BTreeMap<i64, Vec<[f64; 2]>> = BTreeMap::new();
    for (d, p) in holes {
        tools
            .entry((d * 1000.0).round() as i64)
            .or_default()
            .push(*p);
    }
    let mut s = String::new();
    s.push_str("M48\n");
    s.push_str(&format!("; DRILL file {{Kerfwright}} for {}\n", board.name));
    s.push_str(&format!("; #@! TF.FileFunction,{function}\n"));
    s.push_str("; #@! TF.FilePolarity,Positive\nFMAT,2\nMETRIC,TZ\n");
    for (i, (d, _)) in tools.iter().enumerate() {
        s.push_str(&format!("T{}C{:.3}\n", i + 1, *d as f64 / 1000.0));
    }
    s.push_str("%\nG90\nG05\n");
    for (i, (_, pts)) in tools.iter().enumerate() {
        s.push_str(&format!("T{}\n", i + 1));
        for p in pts {
            s.push_str(&format!("X{}Y{}\n", fmt(p[0]), fmt(p[1])));
        }
    }
    s.push_str("T0\nM30\n");
    s
}
