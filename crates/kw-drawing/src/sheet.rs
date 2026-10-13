//! Sheet formats: paper sizes, border, centring marks and the title block.
//!
//! The layout follows the general arrangement of ISO 5457 (trimmed sheet, 20 mm filing margin
//! on the left, title block in the lower-right corner) and ASME Y14.1 (uniform border, title
//! block in the lower-right corner). The title block is our own design.

use crate::scene::{Anchor, Layer, Scene2d, Shape, P};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PaperSize {
    A0,
    A1,
    A2,
    A3,
    A4,
    /// ANSI / ASME A, 8.5 x 11 in.
    AnsiA,
    AnsiB,
    AnsiC,
    AnsiD,
    AnsiE,
    /// Width and height in mm (landscape: width >= height).
    Custom {
        w: f64,
        h: f64,
    },
}

impl PaperSize {
    pub const STANDARD: [PaperSize; 10] = [
        PaperSize::A0,
        PaperSize::A1,
        PaperSize::A2,
        PaperSize::A3,
        PaperSize::A4,
        PaperSize::AnsiA,
        PaperSize::AnsiB,
        PaperSize::AnsiC,
        PaperSize::AnsiD,
        PaperSize::AnsiE,
    ];

    /// Landscape width and height in mm.
    pub fn dims(self) -> (f64, f64) {
        const IN: f64 = 25.4;
        match self {
            PaperSize::A0 => (1189.0, 841.0),
            PaperSize::A1 => (841.0, 594.0),
            PaperSize::A2 => (594.0, 420.0),
            PaperSize::A3 => (420.0, 297.0),
            PaperSize::A4 => (297.0, 210.0),
            PaperSize::AnsiA => (11.0 * IN, 8.5 * IN),
            PaperSize::AnsiB => (17.0 * IN, 11.0 * IN),
            PaperSize::AnsiC => (22.0 * IN, 17.0 * IN),
            PaperSize::AnsiD => (34.0 * IN, 22.0 * IN),
            PaperSize::AnsiE => (44.0 * IN, 34.0 * IN),
            PaperSize::Custom { w, h } => (w.max(h), w.min(h)),
        }
    }

    pub fn name(self) -> String {
        match self {
            PaperSize::A0 => "A0".into(),
            PaperSize::A1 => "A1".into(),
            PaperSize::A2 => "A2".into(),
            PaperSize::A3 => "A3".into(),
            PaperSize::A4 => "A4".into(),
            PaperSize::AnsiA => "ANSI A".into(),
            PaperSize::AnsiB => "ANSI B".into(),
            PaperSize::AnsiC => "ANSI C".into(),
            PaperSize::AnsiD => "ANSI D".into(),
            PaperSize::AnsiE => "ANSI E".into(),
            PaperSize::Custom { w, h } => format!("{w:.0} x {h:.0} mm"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Standard {
    Iso,
    Asme,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Units {
    Mm,
    Inch,
}

impl Units {
    pub fn label(self) -> &'static str {
        match self {
            Units::Mm => "mm",
            Units::Inch => "in",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProjectionAngle {
    Third,
    First,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SheetFormat {
    pub size: PaperSize,
    pub standard: Standard,
    pub units: Units,
    pub projection: ProjectionAngle,
}

impl SheetFormat {
    pub fn iso(size: PaperSize) -> Self {
        SheetFormat {
            size,
            standard: Standard::Iso,
            units: Units::Mm,
            projection: ProjectionAngle::Third,
        }
    }

    pub fn asme(size: PaperSize) -> Self {
        SheetFormat {
            size,
            standard: Standard::Asme,
            units: Units::Inch,
            projection: ProjectionAngle::Third,
        }
    }
}

impl Default for SheetFormat {
    fn default() -> Self {
        SheetFormat::iso(PaperSize::A3)
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TitleBlock {
    pub title: String,
    pub number: String,
    pub revision: String,
    pub author: String,
    pub company: String,
    pub date: String,
    pub scale: String,
}

/// Where things sit on a sheet, in sheet mm with the origin at the lower-left corner of the paper.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SheetFrame {
    pub paper: P,
    /// Inner border rectangle `(lo, hi)`.
    pub border: (P, P),
    /// Title block rectangle `(lo, hi)`, in the lower-right corner of the border.
    pub title: (P, P),
}

pub const TITLE_W: f64 = 180.0;
pub const TITLE_H: f64 = 36.0;

impl SheetFormat {
    pub fn frame(&self) -> SheetFrame {
        let (w, h) = self.size.dims();
        let (left, other) = match self.standard {
            Standard::Iso => (
                20.0,
                if matches!(self.size, PaperSize::A0 | PaperSize::A1) {
                    20.0
                } else {
                    10.0
                },
            ),
            Standard::Asme => (12.7, 12.7),
        };
        let lo = [left, other];
        let hi = [w - other, h - other];
        // keep the title block inside very small custom sheets
        let tw = TITLE_W.min((hi[0] - lo[0]) * 0.9);
        let th = TITLE_H.min((hi[1] - lo[1]) * 0.5);
        SheetFrame {
            paper: [w, h],
            border: (lo, hi),
            title: ([hi[0] - tw, lo[1]], [hi[0], lo[1] + th]),
        }
    }

    /// Border, centring marks and title block with the given contents.
    pub fn scene(&self, tb: &TitleBlock, sheet_no: usize, sheet_count: usize) -> Scene2d {
        let f = self.frame();
        let mut s = Scene2d::new();
        let (lo, hi) = f.border;
        s.rect(Layer::Border, lo, hi);

        // centring marks: from the border out to the paper edge, at the middle of each side
        let mid = [f.paper[0] / 2.0, f.paper[1] / 2.0];
        s.line(Layer::TitleBlock, [mid[0], 0.0], [mid[0], lo[1] + 5.0]);
        s.line(
            Layer::TitleBlock,
            [mid[0], f.paper[1]],
            [mid[0], hi[1] - 5.0],
        );
        s.line(Layer::TitleBlock, [0.0, mid[1]], [lo[0] + 5.0, mid[1]]);
        s.line(
            Layer::TitleBlock,
            [f.paper[0], mid[1]],
            [hi[0] - 5.0, mid[1]],
        );

        self.title_block(&mut s, f, tb, sheet_no, sheet_count);
        s
    }

    fn title_block(
        &self,
        s: &mut Scene2d,
        f: SheetFrame,
        tb: &TitleBlock,
        sheet_no: usize,
        sheet_count: usize,
    ) {
        let (lo, hi) = f.title;
        let w = hi[0] - lo[0];
        let h = hi[1] - lo[1];
        let k = w / TITLE_W; // all coordinates below are for the 180 x 36 block, scaled to fit
        let row = h / 3.0;
        let x = |u: f64| lo[0] + u * k;
        let y = |r: f64| lo[1] + r * row;
        s.rect(Layer::TitleBlock, lo, hi);

        // rows (bottom, middle, top)
        s.line(Layer::TitleBlock, [lo[0], y(1.0)], [hi[0], y(1.0)]);
        s.line(Layer::TitleBlock, [lo[0], y(2.0)], [hi[0], y(2.0)]);
        // top row: title | number
        s.line(Layer::TitleBlock, [x(110.0), y(2.0)], [x(110.0), hi[1]]);
        // middle row: author | date | scale | rev | sheet
        for u in [40.0, 75.0, 110.0, 135.0] {
            s.line(Layer::TitleBlock, [x(u), y(1.0)], [x(u), y(2.0)]);
        }
        // bottom row: company | projection | units
        s.line(Layer::TitleBlock, [x(110.0), lo[1]], [x(110.0), y(1.0)]);
        s.line(Layer::TitleBlock, [x(150.0), lo[1]], [x(150.0), y(1.0)]);

        let cell = |s: &mut Scene2d, u: f64, r: f64, label: &str, value: &str, size: f64| {
            let top = y(r + 1.0);
            s.text(
                Layer::TitleBlock,
                [x(u) + 1.0, top - 3.2],
                1.8,
                Anchor::Left,
                label,
            );
            s.text(
                Layer::Text,
                [x(u) + 1.5, y(r) + 2.0],
                size,
                Anchor::Left,
                value,
            );
        };
        let (no_label, dwg_label) = match self.standard {
            Standard::Iso => ("TITLE", "DRAWING NUMBER"),
            Standard::Asme => ("TITLE", "DWG NO."),
        };
        cell(s, 0.0, 2.0, no_label, &tb.title, 5.0);
        cell(s, 110.0, 2.0, dwg_label, &tb.number, 4.0);
        cell(s, 0.0, 1.0, "DRAWN BY", &tb.author, 3.2);
        cell(s, 40.0, 1.0, "DATE", &tb.date, 3.2);
        cell(s, 75.0, 1.0, "SCALE", &tb.scale, 3.2);
        cell(s, 110.0, 1.0, "REV", &tb.revision, 3.2);
        cell(
            s,
            135.0,
            1.0,
            "SHEET",
            &format!("{sheet_no} / {sheet_count}"),
            3.2,
        );
        cell(s, 0.0, 0.0, "COMPANY", &tb.company, 3.2);
        let std_name = match self.standard {
            Standard::Iso => "ISO",
            Standard::Asme => "ASME",
        };
        cell(
            s,
            150.0,
            0.0,
            "STD / UNITS",
            &format!("{std_name} {}", self.units.label()),
            3.0,
        );
        self.projection_symbol(s, [x(130.0), y(0.5)], row * 0.8);
    }

    /// The projection-angle symbol: a truncated cone seen from the side next to its end view.
    /// The end view sits to the right of the side view for third-angle and to the left for first.
    fn projection_symbol(&self, s: &mut Scene2d, c: P, h: f64) {
        let l = Layer::Symbol;
        let half = h / 2.0;
        let (cone_x, ring_x) = match self.projection {
            ProjectionAngle::Third => (c[0] - h * 0.9, c[0] + h * 0.8),
            ProjectionAngle::First => (c[0] + h * 0.2, c[0] - h * 1.4),
        };
        // side view: a trapezoid, tall end on the left
        s.polyline(
            l,
            vec![
                [cone_x, c[1] - half],
                [cone_x + h * 1.2, c[1] - half * 0.5],
                [cone_x + h * 1.2, c[1] + half * 0.5],
                [cone_x, c[1] + half],
            ],
            true,
        );
        s.push(
            Layer::Center,
            Shape::Line {
                a: [cone_x - 1.0, c[1]],
                b: [cone_x + h * 1.2 + 1.0, c[1]],
            },
        );
        // end view: two concentric circles
        s.circle(l, [ring_x + half, c[1]], half);
        s.circle(l, [ring_x + half, c[1]], half * 0.5);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iso_a3_is_420_by_297_with_a_20_mm_filing_margin() {
        let f = SheetFormat::iso(PaperSize::A3).frame();
        assert_eq!(f.paper, [420.0, 297.0]);
        assert_eq!(f.border, ([20.0, 10.0], [410.0, 287.0]));
        assert_eq!(f.title.1, [410.0, 46.0]);
        assert_eq!(f.title.0, [230.0, 10.0]);
    }

    #[test]
    fn ansi_sizes_are_in_inches() {
        let (w, h) = PaperSize::AnsiB.dims();
        assert!((w - 431.8).abs() < 1e-9 && (h - 279.4).abs() < 1e-9);
    }
}
