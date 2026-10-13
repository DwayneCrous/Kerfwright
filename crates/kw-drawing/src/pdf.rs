//! A small PDF 1.4 writer for drawing sheets (ISO 32000-1 file structure, written from the
//! public standard). One page per sheet; line work is vector, text uses the built-in Helvetica
//! font (WinAnsi), so no font is embedded. Streams are not compressed, which keeps the output
//! byte-for-byte reproducible and easy to diff.

use crate::scene::{flatten, text_width, Anchor, Layer, Scene2d, Shape, P};
use std::f64::consts::{FRAC_PI_2, TAU};
use std::fmt::Write as _;

/// One page: the sheet size in mm and what to draw on it.
pub struct Page<'a> {
    pub size: P,
    pub scene: &'a Scene2d,
}

const MM: f64 = 72.0 / 25.4;
/// Cap height of Helvetica as a fraction of the em.
const CAP: f64 = 0.718;

/// Format a coordinate: at most 4 decimals, no trailing zeros, never `-0`.
fn n(v: f64) -> String {
    let s = format!("{:.4}", if v.abs() < 5e-5 { 0.0 } else { v });
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.to_string()
}

fn winansi(c: char) -> u8 {
    match c {
        ' '..='~' => c as u8,
        '°' => 0xB0,
        '±' => 0xB1,
        '²' => 0xB2,
        '³' => 0xB3,
        'µ' => 0xB5,
        '×' => 0xD7,
        'Ø' | '⌀' => 0xD8,
        '−' | '–' => b'-',
        _ => b'?',
    }
}

fn hex_text(t: &str) -> String {
    let mut s = String::from("<");
    for c in t.chars() {
        let _ = write!(s, "{:02X}", winansi(c));
    }
    s.push('>');
    s
}

fn utf16_hex(t: &str) -> String {
    let mut s = String::from("<FEFF");
    for u in t.encode_utf16() {
        let _ = write!(s, "{u:04X}");
    }
    s.push('>');
    s
}

fn arc_segments(out: &mut String, c: P, r: f64, a0: f64, sweep: f64, move_to: bool) {
    let segs = (sweep.abs() / FRAC_PI_2).ceil().max(1.0) as usize;
    let da = sweep / segs as f64;
    let k = 4.0 / 3.0 * (da / 4.0).tan();
    let pt = |a: f64| [c[0] + r * a.cos(), c[1] + r * a.sin()];
    if move_to {
        let p = pt(a0);
        let _ = writeln!(out, "{} {} m", n(p[0]), n(p[1]));
    }
    for i in 0..segs {
        let a = a0 + da * i as f64;
        let b = a + da;
        let (p, q) = (pt(a), pt(b));
        let c1 = [p[0] - k * r * a.sin(), p[1] + k * r * a.cos()];
        let c2 = [q[0] + k * r * b.sin(), q[1] - k * r * b.cos()];
        let _ = writeln!(
            out,
            "{} {} {} {} {} {} c",
            n(c1[0]),
            n(c1[1]),
            n(c2[0]),
            n(c2[1]),
            n(q[0]),
            n(q[1])
        );
    }
}

fn set_layer(out: &mut String, l: Layer) {
    let [r, g, b] = l.rgb();
    let c = |v: u8| n(f64::from(v) / 255.0);
    let _ = writeln!(
        out,
        "{} {} {} RG {} {} {} rg",
        c(r),
        c(g),
        c(b),
        c(r),
        c(g),
        c(b)
    );
    let _ = writeln!(out, "{} w", n(l.weight()));
    let d = l.style().dashes();
    if d.is_empty() {
        out.push_str("[] 0 d\n");
    } else {
        let list: Vec<String> = d.iter().map(|v| n(*v)).collect();
        let _ = writeln!(out, "[{}] 0 d", list.join(" "));
    }
}

fn content(scene: &Scene2d) -> String {
    let mut o = String::new();
    let _ = writeln!(o, "{} 0 0 {} 0 0 cm", n(MM), n(MM));
    o.push_str("0 J 1 j\n");
    let mut cur: Option<Layer> = None;
    for pr in &scene.prims {
        if cur != Some(pr.layer) {
            set_layer(&mut o, pr.layer);
            cur = Some(pr.layer);
        }
        match &pr.shape {
            Shape::Line { a, b } => {
                let _ = writeln!(o, "{} {} m {} {} l S", n(a[0]), n(a[1]), n(b[0]), n(b[1]));
            }
            Shape::Circle { c, r } => {
                arc_segments(&mut o, *c, *r, 0.0, TAU, true);
                o.push_str("h S\n");
            }
            Shape::Arc { c, r, a0, a1 } => {
                let mut sweep = a1 - a0;
                if sweep <= 0.0 {
                    sweep += TAU;
                }
                arc_segments(&mut o, *c, *r, *a0, sweep, true);
                o.push_str("S\n");
            }
            Shape::Ellipse { .. } => path(&mut o, &flatten(&pr.shape, 0.005), false, "S"),
            Shape::Polyline { pts, closed } => path(&mut o, pts, *closed, "S"),
            Shape::Fill { pts, shade } => {
                let _ = writeln!(o, "{} g", n(f64::from(*shade) / 255.0));
                path(&mut o, pts, true, "f");
                // restore the layer's fill colour for text drawn afterwards
                cur = None;
            }
            Shape::Text {
                p,
                height,
                angle,
                anchor,
                text,
            } => {
                let em = height / CAP;
                let w = text_width(text, *height);
                let shift = match anchor {
                    Anchor::Left => 0.0,
                    Anchor::Center => w / 2.0,
                    Anchor::Right => w,
                };
                let (s, c) = angle.sin_cos();
                let _ = writeln!(
                    o,
                    "BT /F1 {} Tf {} {} {} {} {} {} Tm {} Tj ET",
                    n(em),
                    n(c),
                    n(s),
                    n(-s),
                    n(c),
                    n(p[0] - shift * c),
                    n(p[1] - shift * s),
                    hex_text(text)
                );
            }
        }
    }
    o
}

fn path(o: &mut String, pts: &[P], close: bool, paint: &str) {
    for (i, p) in pts.iter().enumerate() {
        let _ = writeln!(
            o,
            "{} {} {}",
            n(p[0]),
            n(p[1]),
            if i == 0 { "m" } else { "l" }
        );
    }
    if close {
        o.push_str("h ");
    }
    let _ = writeln!(o, "{paint}");
}

/// Write a PDF with one page per entry of `pages`.
pub fn write_pdf(title: &str, pages: &[Page]) -> Vec<u8> {
    // object numbers: 1 catalog, 2 page tree, 3 font, 4 info, then page/content pairs
    let page_obj = |i: usize| 5 + 2 * i;
    let mut objs: Vec<String> = Vec::new();
    objs.push("<< /Type /Catalog /Pages 2 0 R >>".into());
    let kids: Vec<String> = (0..pages.len())
        .map(|i| format!("{} 0 R", page_obj(i)))
        .collect();
    objs.push(format!(
        "<< /Type /Pages /Kids [{}] /Count {} >>",
        kids.join(" "),
        pages.len()
    ));
    objs.push(
        "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>".into(),
    );
    objs.push(format!(
        "<< /Title {} /Producer {} >>",
        utf16_hex(title),
        utf16_hex("Kerfwright")
    ));
    for (i, pg) in pages.iter().enumerate() {
        objs.push(format!(
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {} {}] /Resources << /Font << /F1 3 0 R >> >> /Contents {} 0 R >>",
            n(pg.size[0] * MM),
            n(pg.size[1] * MM),
            page_obj(i) + 1
        ));
        let body = content(pg.scene);
        objs.push(format!(
            "<< /Length {} >>\nstream\n{}endstream",
            body.len(),
            body
        ));
    }

    let mut out: Vec<u8> = Vec::new();
    out.extend_from_slice(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n");
    let mut offsets = Vec::with_capacity(objs.len());
    for (i, o) in objs.iter().enumerate() {
        offsets.push(out.len());
        out.extend_from_slice(format!("{} 0 obj\n{}\nendobj\n", i + 1, o).as_bytes());
    }
    let xref = out.len();
    out.extend_from_slice(format!("xref\n0 {}\n", objs.len() + 1).as_bytes());
    out.extend_from_slice(b"0000000000 65535 f \n");
    for off in &offsets {
        out.extend_from_slice(format!("{off:010} 00000 n \n").as_bytes());
    }
    out.extend_from_slice(
        format!(
            "trailer\n<< /Size {} /Root 1 0 R /Info 4 0 R >>\nstartxref\n{}\n%%EOF\n",
            objs.len() + 1,
            xref
        )
        .as_bytes(),
    );
    out
}
