//! Annotations other than dimensions: notes with leaders, datum feature symbols, surface finish
//! symbols, feature control frames (with the characteristic symbols drawn from lines and arcs,
//! since fonts do not carry them), and the centre marks of circles and cylinders.

use crate::bom::{BomColumn, BomRowData};
use crate::dim::{DimStatus, Dimension};
use crate::model::View;
use crate::projection::V3;
use crate::scene::{text_width, Anchor, Layer, Scene2d, Shape, P};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Annotation {
    pub id: u64,
    pub kind: AnnotKind,
}

/// Geometric characteristic symbols of a feature control frame (ISO 1101 / ASME Y14.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Gdt {
    Straightness,
    Flatness,
    Circularity,
    Cylindricity,
    Perpendicularity,
    Parallelism,
    Angularity,
    Position,
    Concentricity,
    Symmetry,
    CircularRunout,
    TotalRunout,
    ProfileLine,
    ProfileSurface,
}

impl Gdt {
    pub const ALL: [Gdt; 14] = [
        Gdt::Straightness,
        Gdt::Flatness,
        Gdt::Circularity,
        Gdt::Cylindricity,
        Gdt::Perpendicularity,
        Gdt::Parallelism,
        Gdt::Angularity,
        Gdt::Position,
        Gdt::Concentricity,
        Gdt::Symmetry,
        Gdt::CircularRunout,
        Gdt::TotalRunout,
        Gdt::ProfileLine,
        Gdt::ProfileSurface,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Gdt::Straightness => "Straightness",
            Gdt::Flatness => "Flatness",
            Gdt::Circularity => "Circularity",
            Gdt::Cylindricity => "Cylindricity",
            Gdt::Perpendicularity => "Perpendicularity",
            Gdt::Parallelism => "Parallelism",
            Gdt::Angularity => "Angularity",
            Gdt::Position => "Position",
            Gdt::Concentricity => "Concentricity",
            Gdt::Symmetry => "Symmetry",
            Gdt::CircularRunout => "Circular runout",
            Gdt::TotalRunout => "Total runout",
            Gdt::ProfileLine => "Profile of a line",
            Gdt::ProfileSurface => "Profile of a surface",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AnnotKind {
    Dimension(Dimension),
    /// Text at `pos` (the baseline of its first line), several lines separated by `\n`; with a
    /// leader, an arrow runs from the text to `leader`.
    Note {
        pos: P,
        text: String,
        height: f64,
        leader: Option<P>,
    },
    /// A datum feature symbol: a boxed letter at `pos`, joined to a triangle on the feature at `target`.
    Datum {
        pos: P,
        letter: String,
        target: P,
    },
    /// A surface finish symbol with its apex at `pos` and the roughness above it.
    Surface {
        pos: P,
        value: String,
    },
    /// The parts list, its lower-right corner at `pos`; `rows` is what it was last made from.
    Bom {
        pos: P,
        columns: Vec<BomColumn>,
        rows: Vec<BomRowData>,
    },
    /// A balloon with the item number of `part` at `pos` (on the sheet), its leader ending on a
    /// point of the part. `instance` says which of several like components (in the order of the
    /// assembly's bodies); `local` is the point in the component's own frame and `world` where it
    /// was last found, in the assembly.
    Balloon {
        view: u64,
        part: String,
        instance: u32,
        local: V3,
        world: V3,
        pos: P,
        item: u32,
        status: DimStatus,
    },
    /// A feature control frame with its lower-left corner at `pos`, optionally with a leader.
    Fcf {
        pos: P,
        symbol: Gdt,
        tol: String,
        datums: Vec<String>,
        target: Option<P>,
    },
}

/// Height of the boxes of datum symbols and control frames, mm.
pub const BOX_H: f64 = 7.0;
const TEXT_H: f64 = 3.5;

fn arrow_head(s: &mut Scene2d, tip: P, back: P, layer: Layer) {
    let d = [back[0] - tip[0], back[1] - tip[1]];
    let l = d[0].hypot(d[1]).max(1e-9);
    let t = [d[0] / l, d[1] / l];
    let n = [-t[1], t[0]];
    let base = [tip[0] + t[0] * 3.0, tip[1] + t[1] * 3.0];
    s.push(
        layer,
        Shape::Fill {
            pts: vec![
                tip,
                [base[0] + n[0] * 0.9, base[1] + n[1] * 0.9],
                [base[0] - n[0] * 0.9, base[1] - n[1] * 0.9],
            ],
            shade: 0,
        },
    );
}

/// The characteristic symbol drawn about `c`, about 5 mm across.
pub fn gdt_glyph(s: &mut Scene2d, g: Gdt, c: P) {
    let l = Layer::Symbol;
    let p = |x: f64, y: f64| [c[0] + x, c[1] + y];
    let line = |s: &mut Scene2d, a: P, b: P| s.line(l, a, b);
    match g {
        Gdt::Straightness => line(s, p(-2.5, 0.0), p(2.5, 0.0)),
        Gdt::Flatness => s.polyline(
            l,
            vec![p(-2.5, -1.2), p(1.5, -1.2), p(2.5, 1.2), p(-1.5, 1.2)],
            true,
        ),
        Gdt::Circularity => s.circle(l, c, 2.2),
        Gdt::Cylindricity => {
            s.circle(l, c, 1.8);
            line(s, p(-3.6, -1.6), p(-2.2, 1.6));
            line(s, p(2.2, -1.6), p(3.6, 1.6));
        }
        Gdt::Perpendicularity => {
            line(s, p(-2.5, -2.5), p(2.5, -2.5));
            line(s, p(0.0, -2.5), p(0.0, 2.5));
        }
        Gdt::Parallelism => {
            line(s, p(-2.2, -2.2), p(-0.4, 2.2));
            line(s, p(0.4, -2.2), p(2.2, 2.2));
        }
        Gdt::Angularity => {
            line(s, p(-2.5, -2.2), p(2.5, -2.2));
            line(s, p(-2.5, -2.2), p(1.4, 2.4));
        }
        Gdt::Position => {
            s.circle(l, c, 2.2);
            line(s, p(-3.3, 0.0), p(3.3, 0.0));
            line(s, p(0.0, -3.3), p(0.0, 3.3));
        }
        Gdt::Concentricity => {
            s.circle(l, c, 1.2);
            s.circle(l, c, 2.4);
        }
        Gdt::Symmetry => {
            line(s, p(-2.5, 1.5), p(2.5, 1.5));
            line(s, p(-3.0, 0.0), p(3.0, 0.0));
            line(s, p(-2.5, -1.5), p(2.5, -1.5));
        }
        Gdt::CircularRunout => {
            line(s, p(-2.2, -2.4), p(1.4, 1.2));
            arrow_head(s, p(2.4, 2.2), p(1.2, 1.0), l);
        }
        Gdt::TotalRunout => {
            line(s, p(-2.8, -2.4), p(0.2, 0.6));
            line(s, p(-0.6, -2.4), p(2.4, 0.6));
            arrow_head(s, p(1.0, 1.8), p(0.0, 0.8), l);
            arrow_head(s, p(3.2, 1.8), p(2.2, 0.8), l);
        }
        Gdt::ProfileLine => s.push(
            l,
            Shape::Arc {
                c: p(0.0, -1.6),
                r: 3.4,
                a0: std::f64::consts::FRAC_PI_2 - 1.0,
                a1: std::f64::consts::FRAC_PI_2 + 1.0,
            },
        ),
        Gdt::ProfileSurface => {
            s.push(
                l,
                Shape::Arc {
                    c: p(0.0, -1.6),
                    r: 3.4,
                    a0: std::f64::consts::FRAC_PI_2 - 1.0,
                    a1: std::f64::consts::FRAC_PI_2 + 1.0,
                },
            );
            let (hx, hy) = (3.4 * 1.0_f64.sin(), -1.6 + 3.4 * 1.0_f64.cos());
            line(s, p(-hx, hy), p(hx, hy));
        }
    }
}

/// A leader: a line from `from` to `target` with an arrowhead at the target.
fn leader(s: &mut Scene2d, from: P, target: P) {
    s.line(Layer::Text, from, target);
    arrow_head(s, target, from, Layer::Text);
}

/// Draw the notes and symbols (not dimensions, which need their view).
pub fn layout(a: &Annotation) -> Scene2d {
    let mut s = Scene2d::new();
    match &a.kind {
        // these need their view (balloons) or are drawn by the sheet
        AnnotKind::Dimension(_) | AnnotKind::Balloon { .. } => {}
        AnnotKind::Bom { pos, columns, rows } => {
            s.extend(crate::bom::table_scene(*pos, columns, rows))
        }
        AnnotKind::Note {
            pos,
            text,
            height,
            leader: target,
        } => {
            let lines: Vec<&str> = text.split('\n').collect();
            let step = height * 1.45;
            let mut w: f64 = 0.0;
            for (i, t) in lines.iter().enumerate() {
                s.text(
                    Layer::Text,
                    [pos[0], pos[1] - step * i as f64],
                    *height,
                    Anchor::Left,
                    t,
                );
                w = w.max(text_width(t, *height));
            }
            if let Some(tg) = target {
                // from the middle of the side of the text nearer the target
                let mid_y = pos[1] + height / 2.0 - step * (lines.len() as f64 - 1.0) / 2.0;
                let from = if tg[0] < pos[0] + w / 2.0 {
                    [pos[0] - 1.0, mid_y]
                } else {
                    [pos[0] + w + 1.0, mid_y]
                };
                leader(&mut s, from, *tg);
            }
        }
        AnnotKind::Datum {
            pos,
            letter,
            target,
        } => {
            let half = BOX_H / 2.0 - 0.5;
            s.rect(
                Layer::Symbol,
                [pos[0] - half, pos[1] - half],
                [pos[0] + half, pos[1] + half],
            );
            s.text(
                Layer::Symbol,
                [pos[0], pos[1] - TEXT_H / 2.0],
                TEXT_H,
                Anchor::Center,
                letter,
            );
            // line from the box toward the feature, ending in a filled triangle on it
            let d = [target[0] - pos[0], target[1] - pos[1]];
            let l = d[0].hypot(d[1]).max(1e-9);
            let t = [d[0] / l, d[1] / l];
            let k = (half / t[0].abs().max(1e-9)).min(half / t[1].abs().max(1e-9));
            let edge = [pos[0] + t[0] * k, pos[1] + t[1] * k];
            let apex = [target[0] - t[0] * 3.0, target[1] - t[1] * 3.0];
            s.line(Layer::Symbol, edge, apex);
            let n = [-t[1], t[0]];
            s.push(
                Layer::Symbol,
                Shape::Fill {
                    pts: vec![
                        apex,
                        [target[0] + n[0] * 1.8, target[1] + n[1] * 1.8],
                        [target[0] - n[0] * 1.8, target[1] - n[1] * 1.8],
                    ],
                    shade: 0,
                },
            );
        }
        AnnotKind::Surface { pos, value } => {
            let w = text_width(value, TEXT_H) + 2.0;
            let (ax, ay) = (pos[0], pos[1]);
            s.polyline(
                Layer::Symbol,
                vec![
                    [ax - 3.0, ay + 4.5],
                    [ax, ay],
                    [ax + 5.5, ay + 10.0],
                    [ax + 5.5 + w, ay + 10.0],
                ],
                false,
            );
            s.text(
                Layer::Symbol,
                [ax + 6.5, ay + 10.8],
                TEXT_H,
                Anchor::Left,
                value,
            );
        }
        AnnotKind::Fcf {
            pos,
            symbol,
            tol,
            datums,
            target,
        } => {
            let h = BOX_H;
            let tol_w = text_width(tol, TEXT_H) + 4.0;
            let mut x = pos[0];
            let mut cells: Vec<(f64, f64)> = vec![(h, 0.0), (tol_w, 0.0)];
            for _ in datums {
                cells.push((h, 0.0));
            }
            let total: f64 = cells.iter().map(|c| c.0).sum();
            s.rect(Layer::Symbol, *pos, [pos[0] + total, pos[1] + h]);
            for (i, (w, _)) in cells.iter().enumerate() {
                if i > 0 {
                    s.line(Layer::Symbol, [x, pos[1]], [x, pos[1] + h]);
                }
                let cx = x + w / 2.0;
                let cy = pos[1] + h / 2.0;
                match i {
                    0 => gdt_glyph(&mut s, *symbol, [cx, cy]),
                    1 => s.text(
                        Layer::Symbol,
                        [cx, cy - TEXT_H / 2.0],
                        TEXT_H,
                        Anchor::Center,
                        tol,
                    ),
                    n => s.text(
                        Layer::Symbol,
                        [cx, cy - TEXT_H / 2.0],
                        TEXT_H,
                        Anchor::Center,
                        &datums[n - 2],
                    ),
                }
                x += w;
            }
            if let Some(tg) = target {
                let from = if tg[0] < pos[0] + total / 2.0 {
                    [pos[0], pos[1] + h / 2.0]
                } else {
                    [pos[0] + total, pos[1] + h / 2.0]
                };
                leader(&mut s, from, *tg);
            }
        }
    }
    s
}

/// Centre marks (crosses on circles seen face-on) and centre lines (along cylinders seen from
/// the side) of a view, on the sheet.
pub fn center_marks(v: &View) -> Scene2d {
    let mut s = Scene2d::new();
    let Some(c) = &v.cache else { return s };
    if !v.center_marks {
        return s;
    }
    let at = |p: P| [v.pos[0] + p[0], v.pos[1] + p[1]];
    for (ctr, r) in &c.marks {
        if *r < 0.3 {
            continue;
        }
        let p = at(*ctr);
        let e = r + 3.0;
        s.line(Layer::Center, [p[0] - e, p[1]], [p[0] + e, p[1]]);
        s.line(Layer::Center, [p[0], p[1] - e], [p[0], p[1] + e]);
    }
    for (a, b) in &c.axes {
        let (a, b) = (at(*a), at(*b));
        let d = [b[0] - a[0], b[1] - a[1]];
        let l = d[0].hypot(d[1]);
        if l < 1.0 {
            continue;
        }
        let t = [d[0] / l, d[1] / l];
        s.line(
            Layer::Center,
            [a[0] - t[0] * 3.0, a[1] - t[1] * 3.0],
            [b[0] + t[0] * 3.0, b[1] + t[1] * 3.0],
        );
    }
    s
}
