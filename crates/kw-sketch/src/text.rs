//! Text as sketch geometry: the outlines of a font's glyphs as lines and Bézier splines, so that
//! letters are profiles like any other and can be extruded or cut.

use crate::curves::{Curve2, P2};
use ab_glyph::{Font, FontRef, OutlineCurve};

/// Where the text goes and how big it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TextStyle {
    /// Height of a capital letter, mm.
    pub height: f64,
    /// Extra space between letters, as a fraction of the height.
    pub spacing: f64,
    /// Lower-left corner of the first letter's baseline.
    pub origin: P2,
    /// Turn the whole text counter-clockwise by this many degrees about `origin`.
    pub angle_deg: f64,
}

impl Default for TextStyle {
    fn default() -> Self {
        TextStyle {
            height: 10.0,
            spacing: 0.0,
            origin: (0.0, 0.0),
            angle_deg: 0.0,
        }
    }
}

/// The outline curves of `text` set in `font` (TrueType / OpenType data). Line breaks start a new line
/// 1.5 capital heights lower. Returns the curves and the width of the longest line.
pub fn text_curves(
    font: &[u8],
    text: &str,
    style: &TextStyle,
) -> Result<(Vec<Curve2>, f64), String> {
    let font = FontRef::try_from_slice(font).map_err(|e| format!("cannot read the font: {e}"))?;
    if !(style.height > 0.0 && style.height.is_finite()) {
        return Err("the text height must be positive".into());
    }
    // scale so that a capital H is `height` tall (ab_glyph's y axis points down)
    let cap = font
        .outline(font.glyph_id('H'))
        .map(|o| (o.bounds.max.y.abs().max(o.bounds.min.y.abs())) as f64)
        .filter(|c| *c > 0.0)
        .ok_or("the font has no capital H to measure")?;
    let up = font
        .outline(font.glyph_id('H'))
        .map(|o| if o.bounds.min.y < 0.0 { -1.0 } else { 1.0 })
        .unwrap_or(-1.0);
    let k = style.height / cap;
    let (sin, cos) = style.angle_deg.to_radians().sin_cos();
    let place = |x: f64, y: f64| -> P2 {
        (
            style.origin.0 + x * cos - y * sin,
            style.origin.1 + x * sin + y * cos,
        )
    };
    let mut curves = Vec::new();
    let mut widest = 0.0_f64;
    for (row, line) in text.split('\n').enumerate() {
        let base_y = -(row as f64) * 1.5 * style.height;
        let mut pen = 0.0_f64;
        let mut prev = None;
        for ch in line.chars() {
            let id = font.glyph_id(ch);
            if let Some(p) = prev {
                pen += font.kern_unscaled(p, id) as f64 * k;
            }
            if let Some(outline) = font.outline(id) {
                let pt =
                    |p: ab_glyph::Point| place(pen + p.x as f64 * k, base_y + up * p.y as f64 * k);
                for c in &outline.curves {
                    curves.push(match c {
                        OutlineCurve::Line(a, b) => {
                            let (a, b) = (pt(*a), pt(*b));
                            if (a.0 - b.0).hypot(a.1 - b.1) < 1e-9 {
                                continue;
                            }
                            Curve2::Line(a, b)
                        }
                        OutlineCurve::Quad(a, b, c) => Curve2::Bezier(vec![pt(*a), pt(*b), pt(*c)]),
                        OutlineCurve::Cubic(a, b, c, d) => {
                            Curve2::Bezier(vec![pt(*a), pt(*b), pt(*c), pt(*d)])
                        }
                    });
                }
            }
            pen += font.h_advance_unscaled(id) as f64 * k + style.spacing * style.height;
            prev = Some(id);
        }
        widest = widest.max(pen);
    }
    Ok((curves, widest))
}
