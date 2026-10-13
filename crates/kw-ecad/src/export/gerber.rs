//! Gerber RS-274X (with X2 attributes) writers for copper, solder mask and the board profile.

use crate::board::{Board, PadShape, Side};
use crate::shape::{pad_shape, Shape};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GerberLayer {
    /// Copper layer index (0 = top).
    Copper(usize),
    Mask(Side),
    Profile,
}

impl GerberLayer {
    pub fn file_name(&self, board: &Board) -> String {
        match self {
            GerberLayer::Copper(0) => format!("{}-F_Cu.gtl", board.name),
            GerberLayer::Copper(l) if *l == board.layers - 1 => format!("{}-B_Cu.gbl", board.name),
            GerberLayer::Copper(l) => format!("{}-In{}_Cu.g{}", board.name, l, l + 1),
            GerberLayer::Mask(Side::Top) => format!("{}-F_Mask.gts", board.name),
            GerberLayer::Mask(Side::Bottom) => format!("{}-B_Mask.gbs", board.name),
            GerberLayer::Profile => format!("{}-Edge_Cuts.gko", board.name),
        }
    }
}

/// Coordinates are written as integers in nanometres (`FSLAX46Y46` = 4 integer, 6 decimal digits of mm).
fn c(v: f64) -> i64 {
    (v * 1e6).round() as i64
}

#[derive(Debug, Clone, PartialEq)]
enum Aperture {
    Circle(i64),
    Rect(i64, i64),
    Obround(i64, i64),
}

struct Writer {
    body: String,
    apertures: BTreeMap<String, (u32, String)>,
    next: u32,
    current: Option<u32>,
}

impl Writer {
    fn new() -> Writer {
        Writer {
            body: String::new(),
            apertures: BTreeMap::new(),
            next: 10,
            current: None,
        }
    }

    fn select(&mut self, a: &Aperture) {
        let (key, def) = match a {
            Aperture::Circle(d) => (format!("C{d}"), format!("C,{:.6}", *d as f64 / 1e6)),
            Aperture::Rect(w, h) => (
                format!("R{w}x{h}"),
                format!("R,{:.6}X{:.6}", *w as f64 / 1e6, *h as f64 / 1e6),
            ),
            Aperture::Obround(w, h) => (
                format!("O{w}x{h}"),
                format!("O,{:.6}X{:.6}", *w as f64 / 1e6, *h as f64 / 1e6),
            ),
        };
        let next = self.next;
        let entry = self.apertures.entry(key).or_insert((next, def));
        if entry.0 == next {
            self.next += 1;
        }
        let code = entry.0;
        if self.current != Some(code) {
            self.body.push_str(&format!("D{code}*\n"));
            self.current = Some(code);
        }
    }

    fn flash(&mut self, a: &Aperture, p: [f64; 2]) {
        self.select(a);
        self.body
            .push_str(&format!("X{}Y{}D03*\n", c(p[0]), c(p[1])));
    }

    fn line(&mut self, w: f64, a: [f64; 2], b: [f64; 2]) {
        self.select(&Aperture::Circle(c(w)));
        self.body.push_str(&format!(
            "X{}Y{}D02*\nX{}Y{}D01*\n",
            c(a[0]),
            c(a[1]),
            c(b[0]),
            c(b[1])
        ));
    }

    fn region(&mut self, contour: &[[f64; 2]]) {
        self.body.push_str("G36*\n");
        for (i, p) in contour.iter().enumerate() {
            self.body.push_str(&format!(
                "X{}Y{}D0{}*\n",
                c(p[0]),
                c(p[1]),
                if i == 0 { 2 } else { 1 }
            ));
        }
        let p = contour[0];
        self.body
            .push_str(&format!("X{}Y{}D01*\nG37*\n", c(p[0]), c(p[1])));
    }

    fn finish(self, function: &str, polarity: &str) -> String {
        let mut s = String::new();
        s.push_str("%TF.GenerationSoftware,Kerfwright,kw-ecad,0.0.1*%\n");
        s.push_str(&format!("%TF.FileFunction,{function}*%\n"));
        s.push_str(&format!("%TF.FilePolarity,{polarity}*%\n"));
        s.push_str("%TF.SameCoordinates,Original*%\n");
        s.push_str("%FSLAX46Y46*%\n%MOMM*%\n");
        s.push_str("G01*\n");
        let mut defs: Vec<(u32, String)> = self.apertures.values().cloned().collect();
        defs.sort();
        for (code, def) in defs {
            s.push_str(&format!("%ADD{code}{def}*%\n"));
        }
        s.push_str("%LPD*%\n");
        s.push_str(&self.body);
        s.push_str("M02*\n");
        s
    }
}

fn flash_pad(w: &mut Writer, shape: PadShape, pos: [f64; 2], angle: f64, grow: f64) {
    let quarter = ((angle / 90.0).round() as i64).rem_euclid(4);
    let aligned = (angle / 90.0 - (angle / 90.0).round()).abs() < 1e-9;
    let swap = quarter % 2 == 1;
    match shape {
        PadShape::Circle { d } => w.flash(&Aperture::Circle(c(d + grow)), pos),
        PadShape::Rect { w: pw, h: ph } => {
            if aligned {
                let (a, b) = if swap { (ph, pw) } else { (pw, ph) };
                w.flash(&Aperture::Rect(c(a + grow), c(b + grow)), pos);
            } else if let Shape::Poly(p) = pad_shape(
                PadShape::Rect {
                    w: pw + grow,
                    h: ph + grow,
                },
                pos,
                angle,
            ) {
                w.region(&p);
            }
        }
        PadShape::Oval { w: pw, h: ph } => {
            if aligned {
                let (a, b) = if swap { (ph, pw) } else { (pw, ph) };
                w.flash(&Aperture::Obround(c(a + grow), c(b + grow)), pos);
            } else if let Shape::Cap { a, b, r } = pad_shape(shape, pos, angle) {
                w.line(2.0 * r + grow, a, b);
            }
        }
    }
}

pub fn gerber(board: &Board, layer: GerberLayer) -> String {
    let mut w = Writer::new();
    match layer {
        GerberLayer::Copper(l) => {
            // pours first: their clearance holes are cut with clear polarity, then the copper goes on top
            for z in board.zones.iter().filter(|z| z.layer == l) {
                for sh in &z.shapes {
                    w.region(&sh[0]);
                    if sh.len() > 1 {
                        w.body.push_str("%LPC*%\n");
                        for hole in &sh[1..] {
                            w.region(hole);
                        }
                        w.body.push_str("%LPD*%\n");
                    }
                }
            }
            for p in board.pads() {
                if p.layer.is_none() || p.layer == Some(l) {
                    flash_pad(&mut w, p.shape, p.pos, p.angle, 0.0);
                }
            }
            for v in &board.vias {
                w.flash(&Aperture::Circle(c(v.dia)), v.pos);
            }
            for t in board.tracks.iter().filter(|t| t.layer == l) {
                w.line(t.width, t.a, t.b);
            }
            let function = if l == 0 {
                "Copper,L1,Top".to_string()
            } else if l == board.layers - 1 {
                format!("Copper,L{},Bot", l + 1)
            } else {
                format!("Copper,L{},Inr", l + 1)
            };
            w.finish(&function, "Positive")
        }
        GerberLayer::Mask(side) => {
            let l = match side {
                Side::Top => 0,
                Side::Bottom => board.layers - 1,
            };
            for p in board.pads() {
                if p.layer.is_none() || p.layer == Some(l) {
                    flash_pad(&mut w, p.shape, p.pos, p.angle, 0.1);
                }
            }
            w.finish(
                &format!(
                    "Soldermask,{}",
                    if side == Side::Top { "Top" } else { "Bot" }
                ),
                "Negative",
            )
        }
        GerberLayer::Profile => {
            let o = board.outline();
            for i in 0..4 {
                w.line(0.1, o[i], o[(i + 1) % 4]);
            }
            w.finish("Profile,NP", "Positive")
        }
    }
}
