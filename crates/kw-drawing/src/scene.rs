//! The flat 2-D scene every output shares: the canvas paints it, the PDF and DXF writers
//! serialise it, and tests assert on it. Coordinates are sheet millimetres, y up.

use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

pub type P = [f64; 2];

/// Drawing layers. A layer fixes the line style, the line weight and the colour, as drawing
/// standards do (thick visible outline, thin dimension and hatch lines, dashed hidden lines).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Layer {
    Border,
    TitleBlock,
    Visible,
    Hidden,
    Center,
    Section,
    Hatch,
    Dimension,
    Text,
    Symbol,
    Table,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LineStyle {
    Continuous,
    Dashed,
    /// Long dash, short dash (centre lines).
    Chain,
    /// Long dash, two short dashes (cutting planes).
    Phantom,
}

impl LineStyle {
    /// Dash pattern in millimetres (dash, gap, dash, gap, ...); empty for a solid line.
    pub fn dashes(self) -> &'static [f64] {
        match self {
            LineStyle::Continuous => &[],
            LineStyle::Dashed => &[3.0, 1.5],
            LineStyle::Chain => &[12.0, 2.0, 2.0, 2.0],
            LineStyle::Phantom => &[12.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        }
    }

    /// The DXF linetype name.
    pub fn dxf_name(self) -> &'static str {
        match self {
            LineStyle::Continuous => "CONTINUOUS",
            LineStyle::Dashed => "HIDDEN",
            LineStyle::Chain => "CENTER",
            LineStyle::Phantom => "PHANTOM",
        }
    }
}

impl Layer {
    pub const ALL: [Layer; 11] = [
        Layer::Border,
        Layer::TitleBlock,
        Layer::Visible,
        Layer::Hidden,
        Layer::Center,
        Layer::Section,
        Layer::Hatch,
        Layer::Dimension,
        Layer::Text,
        Layer::Symbol,
        Layer::Table,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Layer::Border => "BORDER",
            Layer::TitleBlock => "TITLE_BLOCK",
            Layer::Visible => "VISIBLE",
            Layer::Hidden => "HIDDEN",
            Layer::Center => "CENTER",
            Layer::Section => "SECTION",
            Layer::Hatch => "HATCH",
            Layer::Dimension => "DIMENSION",
            Layer::Text => "TEXT",
            Layer::Symbol => "SYMBOL",
            Layer::Table => "TABLE",
        }
    }

    pub fn style(self) -> LineStyle {
        match self {
            Layer::Hidden => LineStyle::Dashed,
            Layer::Center => LineStyle::Chain,
            Layer::Section => LineStyle::Phantom,
            _ => LineStyle::Continuous,
        }
    }

    /// Line weight in millimetres. Thick and thin lines keep the 2:1 ratio of ISO 128.
    pub fn weight(self) -> f64 {
        match self {
            Layer::Border => 0.7,
            Layer::TitleBlock | Layer::Symbol | Layer::Table => 0.35,
            Layer::Visible | Layer::Section => 0.5,
            Layer::Hidden | Layer::Center | Layer::Dimension | Layer::Text => 0.25,
            Layer::Hatch => 0.18,
        }
    }

    /// AutoCAD colour index used for the DXF layer.
    pub fn aci(self) -> i32 {
        match self {
            Layer::Hidden | Layer::Hatch => 8,
            Layer::Center => 1,
            Layer::Dimension => 3,
            Layer::Section => 5,
            _ => 7,
        }
    }

    /// Display / print colour.
    pub fn rgb(self) -> [u8; 3] {
        match self {
            Layer::Hidden | Layer::Hatch => [110, 110, 110],
            Layer::Center => [190, 40, 40],
            Layer::Dimension => [20, 110, 40],
            Layer::Section => [40, 70, 190],
            _ => [0, 0, 0],
        }
    }
}

/// Horizontal text anchor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Anchor {
    Left,
    Center,
    Right,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Shape {
    Line {
        a: P,
        b: P,
    },
    Circle {
        c: P,
        r: f64,
    },
    /// Counter-clockwise from `a0` to `a1`, in radians.
    Arc {
        c: P,
        r: f64,
        a0: f64,
        a1: f64,
    },
    /// `major` is the major-axis end point relative to the centre, `ratio` = minor / major,
    /// `t0`..`t1` the parameter range in radians.
    Ellipse {
        c: P,
        major: P,
        ratio: f64,
        t0: f64,
        t1: f64,
    },
    Polyline {
        pts: Vec<P>,
        closed: bool,
    },
    /// `p` is the baseline point at the anchor; `height` is the cap height in mm.
    Text {
        p: P,
        height: f64,
        angle: f64,
        anchor: Anchor,
        text: String,
    },
    /// A filled polygon; `shade` 0 = black, 255 = white.
    Fill {
        pts: Vec<P>,
        shade: u8,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Prim {
    pub layer: Layer,
    pub shape: Shape,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Scene2d {
    pub prims: Vec<Prim>,
}

/// Advance widths (1/1000 em) of Helvetica for ASCII 32..=126, from the standard AFM.
const HELV: [u16; 95] = [
    278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278, 556, 556, 556,
    556, 556, 556, 556, 556, 556, 556, 278, 278, 584, 584, 584, 556, 1015, 667, 667, 722, 722, 667,
    611, 778, 722, 278, 500, 667, 556, 833, 722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667,
    667, 611, 278, 278, 278, 469, 556, 333, 556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500,
    222, 833, 556, 556, 556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, 334, 260, 334, 584,
];

/// Width of one character in 1/1000 em.
pub fn char_width(c: char) -> f64 {
    match c {
        ' '..='~' => f64::from(HELV[c as usize - 32]),
        '°' => 400.0,
        '±' => 584.0,
        '×' => 584.0,
        'Ø' | '⌀' => 778.0,
        _ => 556.0,
    }
}

/// Width in mm of `text` set in Helvetica with the given em size (the text `height` below is the
/// cap height; an em is `height / 0.718`).
pub fn text_width(text: &str, height: f64) -> f64 {
    let em = height / 0.718;
    text.chars().map(char_width).sum::<f64>() * em / 1000.0
}

impl Scene2d {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, layer: Layer, shape: Shape) {
        self.prims.push(Prim { layer, shape });
    }

    pub fn line(&mut self, layer: Layer, a: P, b: P) {
        self.push(layer, Shape::Line { a, b });
    }

    pub fn circle(&mut self, layer: Layer, c: P, r: f64) {
        self.push(layer, Shape::Circle { c, r });
    }

    pub fn polyline(&mut self, layer: Layer, pts: Vec<P>, closed: bool) {
        self.push(layer, Shape::Polyline { pts, closed });
    }

    /// An axis-aligned rectangle from corner `lo` to corner `hi`.
    pub fn rect(&mut self, layer: Layer, lo: P, hi: P) {
        self.polyline(layer, vec![lo, [hi[0], lo[1]], hi, [lo[0], hi[1]]], true);
    }

    pub fn text(&mut self, layer: Layer, p: P, height: f64, anchor: Anchor, text: &str) {
        self.text_at(layer, p, height, 0.0, anchor, text);
    }

    pub fn text_at(
        &mut self,
        layer: Layer,
        p: P,
        height: f64,
        angle: f64,
        anchor: Anchor,
        text: &str,
    ) {
        if text.is_empty() {
            return;
        }
        self.push(
            layer,
            Shape::Text {
                p,
                height,
                angle,
                anchor,
                text: text.to_string(),
            },
        );
    }

    pub fn extend(&mut self, other: Scene2d) {
        self.prims.extend(other.prims);
    }

    /// The scene scaled uniformly by `scale` about the origin, then moved by `offset`.
    /// Text keeps its height (it is annotation, not geometry).
    pub fn placed(&self, scale: f64, offset: P) -> Scene2d {
        let pt = |p: &P| [p[0] * scale + offset[0], p[1] * scale + offset[1]];
        let prims = self
            .prims
            .iter()
            .map(|pr| Prim {
                layer: pr.layer,
                shape: match &pr.shape {
                    Shape::Line { a, b } => Shape::Line { a: pt(a), b: pt(b) },
                    Shape::Circle { c, r } => Shape::Circle {
                        c: pt(c),
                        r: r * scale,
                    },
                    Shape::Arc { c, r, a0, a1 } => Shape::Arc {
                        c: pt(c),
                        r: r * scale,
                        a0: *a0,
                        a1: *a1,
                    },
                    Shape::Ellipse {
                        c,
                        major,
                        ratio,
                        t0,
                        t1,
                    } => Shape::Ellipse {
                        c: pt(c),
                        major: [major[0] * scale, major[1] * scale],
                        ratio: *ratio,
                        t0: *t0,
                        t1: *t1,
                    },
                    Shape::Polyline { pts, closed } => Shape::Polyline {
                        pts: pts.iter().map(pt).collect(),
                        closed: *closed,
                    },
                    Shape::Text {
                        p,
                        height,
                        angle,
                        anchor,
                        text,
                    } => Shape::Text {
                        p: pt(p),
                        height: *height,
                        angle: *angle,
                        anchor: *anchor,
                        text: text.clone(),
                    },
                    Shape::Fill { pts, shade } => Shape::Fill {
                        pts: pts.iter().map(pt).collect(),
                        shade: *shade,
                    },
                },
            })
            .collect();
        Scene2d { prims }
    }

    /// Bounding box `(lo, hi)` of the geometry, or `None` for an empty scene.
    pub fn bounds(&self) -> Option<(P, P)> {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        let mut add = |p: P, r: f64| {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k] - r);
                hi[k] = hi[k].max(p[k] + r);
            }
        };
        for pr in &self.prims {
            match &pr.shape {
                Shape::Line { a, b } => {
                    add(*a, 0.0);
                    add(*b, 0.0);
                }
                Shape::Circle { c, r } | Shape::Arc { c, r, .. } => add(*c, *r),
                Shape::Ellipse { c, major, .. } => add(*c, major[0].hypot(major[1])),
                Shape::Polyline { pts, .. } | Shape::Fill { pts, .. } => {
                    for p in pts {
                        add(*p, 0.0);
                    }
                }
                Shape::Text {
                    p,
                    height,
                    angle,
                    anchor,
                    text,
                } => {
                    let w = text_width(text, *height);
                    let (x0, x1) = match anchor {
                        Anchor::Left => (0.0, w),
                        Anchor::Center => (-w / 2.0, w / 2.0),
                        Anchor::Right => (-w, 0.0),
                    };
                    let (s, c) = angle.sin_cos();
                    for (x, y) in [(x0, 0.0), (x1, 0.0), (x0, *height), (x1, *height)] {
                        add([p[0] + x * c - y * s, p[1] + x * s + y * c], 0.0);
                    }
                }
            }
        }
        (lo[0].is_finite()).then_some((lo, hi))
    }
}

/// Points along an arc / ellipse / circle for display and for outputs without those primitives.
/// `tol` is the largest allowed distance between the polyline and the curve, in mm.
pub fn flatten(shape: &Shape, tol: f64) -> Vec<P> {
    let steps = |r: f64, sweep: f64| -> usize {
        let r = r.abs().max(1e-9);
        let da = if tol >= r {
            std::f64::consts::FRAC_PI_2
        } else {
            2.0 * (1.0 - tol / r).acos()
        };
        ((sweep.abs() / da.max(1e-3)).ceil() as usize).clamp(2, 4096)
    };
    match shape {
        Shape::Line { a, b } => vec![*a, *b],
        Shape::Circle { c, r } => {
            let n = steps(*r, TAU);
            (0..=n)
                .map(|i| {
                    let t = TAU * i as f64 / n as f64;
                    [c[0] + r * t.cos(), c[1] + r * t.sin()]
                })
                .collect()
        }
        Shape::Arc { c, r, a0, a1 } => {
            let mut sweep = a1 - a0;
            if sweep <= 0.0 {
                sweep += TAU;
            }
            let n = steps(*r, sweep);
            (0..=n)
                .map(|i| {
                    let t = a0 + sweep * i as f64 / n as f64;
                    [c[0] + r * t.cos(), c[1] + r * t.sin()]
                })
                .collect()
        }
        Shape::Ellipse {
            c,
            major,
            ratio,
            t0,
            t1,
        } => {
            let a = major[0].hypot(major[1]);
            let mut sweep = t1 - t0;
            if sweep <= 0.0 {
                sweep += TAU;
            }
            let n = steps(a, sweep);
            let minor = [-major[1] * ratio, major[0] * ratio];
            (0..=n)
                .map(|i| {
                    let t = t0 + sweep * i as f64 / n as f64;
                    [
                        c[0] + major[0] * t.cos() + minor[0] * t.sin(),
                        c[1] + major[1] * t.cos() + minor[1] * t.sin(),
                    ]
                })
                .collect()
        }
        Shape::Polyline { pts, closed } => {
            let mut v = pts.clone();
            if *closed {
                if let Some(f) = pts.first() {
                    v.push(*f);
                }
            }
            v
        }
        Shape::Fill { pts, .. } => pts.clone(),
        Shape::Text { .. } => Vec::new(),
    }
}
