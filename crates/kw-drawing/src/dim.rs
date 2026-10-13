//! Dimensions: what they are tied to in the model, how their value is measured from it, and how
//! they are drawn (extension lines, dimension lines, arrowheads, text with tolerances), following
//! the usual conventions of ISO 129-1 and ASME Y14.5.
//!
//! A dimension is tied to *model* geometry, not to lines on the sheet: its anchors name the edges
//! (by the two faces that meet there) and say which point of the edge. After the model changes
//! the anchors are found again; if that fails the dimension says so instead of guessing. The
//! value is always measured from the 3-D geometry, so it does not depend on the view's scale.

use crate::model::{View, ViewCache};
use crate::projection::{dot, ViewFrame, V3};
use crate::scene::{text_width, Anchor as TextAnchor, Layer, Scene2d, Shape, P};
use crate::sheet::Units;
use kw_feature::EdgeRef;
use serde::{Deserialize, Serialize};

/// Which point of an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PointKind {
    Start,
    End,
    Mid,
    /// The centre of a circular or elliptical edge.
    Centre,
}

/// What a dimension is attached to.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Anchor {
    /// A point of an edge of body `body` (its index among the design's bodies), found through
    /// the two faces that meet at the edge.
    EdgePoint {
        body: u32,
        edge: EdgeRef,
        at: PointKind,
    },
    /// A circular edge: the dimension is its diameter or radius.
    Circle { body: u32, edge: EdgeRef },
    /// A straight edge (for angles).
    Line { body: u32, edge: EdgeRef },
    /// Not tied to the model: fixed geometry in space (assemblies, imported bodies).
    Fixed(Resolved),
}

/// An anchor found in the model.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Resolved {
    Point(V3),
    Circle { centre: V3, radius: f64, axis: V3 },
    Line { a: V3, b: V3 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DimKind {
    Horizontal,
    Vertical,
    /// Along the line between the two points, as projected into the view.
    Aligned,
    Diameter,
    Radius,
    /// The angle between two straight edges.
    Angle,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Tolerance {
    None,
    /// `value ± t`
    Symmetric(f64),
    /// `value +upper / -lower` (stacked)
    Bilateral {
        upper: f64,
        lower: f64,
    },
    /// the two limits, stacked: `value + upper` over `value - lower`
    Limits {
        upper: f64,
        lower: f64,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DimStatus {
    Ok,
    /// An anchor could not be found again; the text says why.
    Broken(String),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Dimension {
    /// The view whose projection the dimension is drawn in.
    pub view: u64,
    pub kind: DimKind,
    pub anchors: Vec<Anchor>,
    /// The anchors as last found in the model.
    pub resolved: Vec<Resolved>,
    /// Where the dimension line is: for horizontal / vertical, its y / x relative to the view
    /// origin; for aligned, its distance from the measured line; for angles, the arc's radius.
    pub line_pos: f64,
    /// Direction (radians) of the leader of a diameter or radius.
    pub angle: f64,
    /// Text before and after the value (`4×`, `THRU`).
    #[serde(default)]
    pub prefix: String,
    #[serde(default)]
    pub suffix: String,
    pub tolerance: Tolerance,
    /// Replaces the measured text when not empty.
    #[serde(default)]
    pub text: String,
    /// Decimals shown; `None` trims trailing zeros.
    #[serde(default)]
    pub precision: Option<u8>,
    /// The value as last measured: model units for lengths, degrees for angles.
    pub value: f64,
    pub status: DimStatus,
}

impl Dimension {
    pub fn new(view: u64, kind: DimKind, anchors: Vec<Anchor>) -> Dimension {
        Dimension {
            view,
            kind,
            anchors,
            resolved: Vec::new(),
            line_pos: 0.0,
            angle: std::f64::consts::FRAC_PI_4,
            prefix: String::new(),
            suffix: String::new(),
            tolerance: Tolerance::None,
            text: String::new(),
            precision: None,
            value: 0.0,
            status: DimStatus::Ok,
        }
    }
}

// ---------------------------------------------------------------------------------- measuring

fn uv(f: &ViewFrame, p: V3) -> P {
    f.project(p).0
}

/// The two ends of a straight resolved anchor, or the single point twice.
fn ends(r: &Resolved) -> (V3, V3) {
    match r {
        Resolved::Point(p) => (*p, *p),
        Resolved::Line { a, b } => (*a, *b),
        Resolved::Circle { centre, .. } => (*centre, *centre),
    }
}

/// The value of a dimension from its resolved anchors, seen along `f`: lengths in model units
/// (measured in the view plane), angles in degrees, circles from their true radius.
pub fn measure(d: &Dimension, f: &ViewFrame) -> Option<f64> {
    let r = &d.resolved;
    match d.kind {
        DimKind::Horizontal | DimKind::Vertical | DimKind::Aligned => {
            let (a, b) = (uv(f, ends(r.first()?).0), uv(f, ends(r.get(1)?).0));
            Some(match d.kind {
                DimKind::Horizontal => (b[0] - a[0]).abs(),
                DimKind::Vertical => (b[1] - a[1]).abs(),
                _ => (b[0] - a[0]).hypot(b[1] - a[1]),
            })
        }
        DimKind::Diameter | DimKind::Radius => match r.first()? {
            Resolved::Circle { radius, .. } => Some(if d.kind == DimKind::Diameter {
                2.0 * radius
            } else {
                *radius
            }),
            _ => None,
        },
        DimKind::Angle => {
            let l = |x: &Resolved| {
                let (a, b) = ends(x);
                let (a, b) = (uv(f, a), uv(f, b));
                [b[0] - a[0], b[1] - a[1]]
            };
            let (u, v) = (l(r.first()?), l(r.get(1)?));
            let (lu, lv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
            if lu < 1e-12 || lv < 1e-12 {
                return None;
            }
            // the angle between the lines, not the rays: at most 90 degrees apart from 180
            let c = ((u[0] * v[0] + u[1] * v[1]) / (lu * lv)).clamp(-1.0, 1.0);
            Some(c.acos().to_degrees())
        }
    }
}

// ---------------------------------------------------------------------------------- text

fn trim(v: f64, decimals: usize) -> String {
    let s = format!("{v:.decimals$}");
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        s
    }
}

/// The value as written: millimetres with up to three decimals, inches likewise; angles in degrees.
pub fn format_value(value: f64, kind: DimKind, units: Units, precision: Option<u8>) -> String {
    let (v, max) = match (kind, units) {
        (DimKind::Angle, _) => (value, 2),
        (_, Units::Mm) => (value, 3),
        (_, Units::Inch) => (value / 25.4, 4),
    };
    match precision {
        Some(p) => format!("{v:.p$}", p = p as usize),
        None => trim(v, max),
    }
}

/// The text of a dimension: the main line and, for stacked tolerances, the upper and lower ones.
#[derive(Debug, Clone, PartialEq)]
pub struct DimText {
    pub main: String,
    pub upper: Option<String>,
    pub lower: Option<String>,
}

pub fn dim_text(d: &Dimension, units: Units) -> DimText {
    let sym = match d.kind {
        DimKind::Diameter => "Ø",
        DimKind::Radius => "R",
        _ => "",
    };
    let unit_k = |v: f64| match (d.kind, units) {
        (DimKind::Angle, _) | (_, Units::Mm) => v,
        (_, Units::Inch) => v / 25.4,
    };
    let fmt = |v: f64| format_value(v, d.kind, units, d.precision);
    let deg = if d.kind == DimKind::Angle { "°" } else { "" };
    let wrap = |core: String| format!("{}{}{core}{}", d.prefix, sym, d.suffix);
    if !d.text.is_empty() {
        return DimText {
            main: wrap(d.text.clone()),
            upper: None,
            lower: None,
        };
    }
    let signed = |v: f64| {
        let s = fmt(unit_k(v).abs());
        format!("{}{s}", if v < 0.0 { "-" } else { "+" })
    };
    match d.tolerance {
        Tolerance::None => DimText {
            main: wrap(format!("{}{deg}", fmt(d.value))),
            upper: None,
            lower: None,
        },
        Tolerance::Symmetric(t) => DimText {
            main: wrap(format!("{}{deg} ±{}", fmt(d.value), fmt(unit_k(t).abs()))),
            upper: None,
            lower: None,
        },
        Tolerance::Bilateral { upper, lower } => DimText {
            main: wrap(format!("{}{deg}", fmt(d.value))),
            upper: Some(signed(upper)),
            lower: Some(signed(-lower.abs())),
        },
        Tolerance::Limits { upper, lower } => DimText {
            main: String::new(),
            upper: Some(wrap(format!("{}{deg}", fmt(d.value + upper)))),
            lower: Some(wrap(format!("{}{deg}", fmt(d.value - lower.abs())))),
        },
    }
}

// ---------------------------------------------------------------------------------- drawing

/// Text height, arrow size and gaps (mm on the sheet).
pub const TEXT_H: f64 = 3.5;
const TOL_H: f64 = 2.5;
const ARROW_L: f64 = 3.0;
const ARROW_W: f64 = 0.9;
/// Gap between the feature and an extension line, and how far the line runs past the dimension line.
const EXT_GAP: f64 = 1.0;
const EXT_OVER: f64 = 2.0;
/// Length of the leader of a diameter or radius outside its circle.
const LEAD: f64 = 7.0;

fn arrow(s: &mut Scene2d, tip: P, dir: P) {
    // `dir` points from the tip back along the line
    let base = [tip[0] + dir[0] * ARROW_L, tip[1] + dir[1] * ARROW_L];
    let n = [-dir[1], dir[0]];
    s.push(
        Layer::Dimension,
        Shape::Fill {
            pts: vec![
                tip,
                [base[0] + n[0] * ARROW_W, base[1] + n[1] * ARROW_W],
                [base[0] - n[0] * ARROW_W, base[1] - n[1] * ARROW_W],
            ],
            shade: 0,
        },
    );
}

fn unit(v: P) -> P {
    let l = v[0].hypot(v[1]).max(1e-300);
    [v[0] / l, v[1] / l]
}

/// Text centred on `at` along direction `t` (unit), its glyphs rising toward `up`. A stacked
/// tolerance goes right of the main text.
fn put_text(s: &mut Scene2d, at: P, t: P, text: &DimText) {
    let angle = t[1].atan2(t[0]);
    let up = [-t[1], t[0]];
    let w_main = text_width(&text.main, TEXT_H);
    let w_tol = text
        .upper
        .iter()
        .chain(text.lower.iter())
        .map(|x| text_width(x, if text.main.is_empty() { TEXT_H } else { TOL_H }))
        .fold(0.0, f64::max);
    if text.main.is_empty() {
        // two limits one over the other
        let mid = |dy: f64| [at[0] + up[0] * dy, at[1] + up[1] * dy];
        if let Some(u) = &text.upper {
            s.text_at(
                Layer::Dimension,
                mid(0.3),
                TEXT_H,
                angle,
                TextAnchor::Center,
                u,
            );
        }
        if let Some(l) = &text.lower {
            s.text_at(
                Layer::Dimension,
                mid(-TEXT_H - 0.8),
                TEXT_H,
                angle,
                TextAnchor::Center,
                l,
            );
        }
        return;
    }
    let total = w_main + if w_tol > 0.0 { w_tol + 0.8 } else { 0.0 };
    let start = [at[0] - t[0] * total / 2.0, at[1] - t[1] * total / 2.0];
    s.text_at(
        Layer::Dimension,
        start,
        TEXT_H,
        angle,
        TextAnchor::Left,
        &text.main,
    );
    let tol_at = |dy: f64| {
        let x = w_main + 0.8;
        [
            start[0] + t[0] * x + up[0] * dy,
            start[1] + t[1] * x + up[1] * dy,
        ]
    };
    if let Some(u) = &text.upper {
        s.text_at(
            Layer::Dimension,
            tol_at(TEXT_H - TOL_H + 0.2),
            TOL_H,
            angle,
            TextAnchor::Left,
            u,
        );
    }
    if let Some(l) = &text.lower {
        s.text_at(
            Layer::Dimension,
            tol_at(-0.1),
            TOL_H,
            angle,
            TextAnchor::Left,
            l,
        );
    }
}

fn text_len(t: &DimText) -> f64 {
    let m = text_width(&t.main, TEXT_H);
    let tol = t
        .upper
        .iter()
        .chain(t.lower.iter())
        .map(|x| text_width(x, if t.main.is_empty() { TEXT_H } else { TOL_H }))
        .fold(0.0, f64::max);
    m + if tol > 0.0 { tol + 0.8 } else { 0.0 }
}

/// A dimension line from `a` to `b` with arrows at both ends; the arrows go outside when the
/// line is too short to hold them.
fn dim_line(s: &mut Scene2d, a: P, b: P) {
    let len = (b[0] - a[0]).hypot(b[1] - a[1]);
    if len < 1e-9 {
        return;
    }
    let t = unit([b[0] - a[0], b[1] - a[1]]);
    if len >= 2.5 * ARROW_L {
        s.line(Layer::Dimension, a, b);
        arrow(s, a, t);
        arrow(s, b, [-t[0], -t[1]]);
    } else {
        // arrows outside, pointing in, with a stub of line behind each
        let out = ARROW_L + 2.0;
        s.line(Layer::Dimension, a, b);
        s.line(Layer::Dimension, a, [a[0] - t[0] * out, a[1] - t[1] * out]);
        s.line(Layer::Dimension, b, [b[0] + t[0] * out, b[1] + t[1] * out]);
        arrow(s, a, [-t[0], -t[1]]);
        arrow(s, b, t);
    }
}

/// An extension line from `feature` toward a dimension line at `target`, leaving a gap at the
/// feature and running a little past the dimension line.
fn ext_line(s: &mut Scene2d, feature: P, target: P) {
    let d = [target[0] - feature[0], target[1] - feature[1]];
    let l = d[0].hypot(d[1]);
    if l < 1e-9 {
        return;
    }
    let t = [d[0] / l, d[1] / l];
    if l <= EXT_GAP {
        return;
    }
    s.line(
        Layer::Dimension,
        [feature[0] + t[0] * EXT_GAP, feature[1] + t[1] * EXT_GAP],
        [target[0] + t[0] * EXT_OVER, target[1] + t[1] * EXT_OVER],
    );
}

pub fn line_hit(a1: P, d1: P, a2: P, d2: P) -> Option<P> {
    let den = d1[0] * d2[1] - d1[1] * d2[0];
    if den.abs() < 1e-12 {
        return None;
    }
    let t = ((a2[0] - a1[0]) * d2[1] - (a2[1] - a1[1]) * d2[0]) / den;
    Some([a1[0] + d1[0] * t, a1[1] + d1[1] * t])
}

/// Draw a dimension in `view` (whose geometry was made at `cache`). Nothing is drawn for a
/// dimension that cannot be placed (broken anchors) except a marker the caller can add.
pub fn layout(d: &Dimension, view: &View, cache: &ViewCache, units: Units) -> Scene2d {
    let mut s = Scene2d::new();
    let f = ViewFrame::of(view.orientation);
    let sheet = |p: V3| -> P {
        let q = uv(&f, p);
        [
            view.pos[0] + q[0] * view.scale - cache.centre[0],
            view.pos[1] + q[1] * view.scale - cache.centre[1],
        ]
    };
    let text = dim_text(d, units);
    let r = &d.resolved;
    match d.kind {
        DimKind::Horizontal | DimKind::Vertical => {
            let (Some(ra), Some(rb)) = (r.first(), r.get(1)) else {
                return s;
            };
            let (a, b) = (sheet(ends(ra).0), sheet(ends(rb).0));
            if d.kind == DimKind::Horizontal {
                let y = view.pos[1] + d.line_pos;
                ext_line(&mut s, a, [a[0], y]);
                ext_line(&mut s, b, [b[0], y]);
                dim_line(&mut s, [a[0], y], [b[0], y]);
                put_text(&mut s, [(a[0] + b[0]) / 2.0, y + 1.0], [1.0, 0.0], &text);
            } else {
                let x = view.pos[0] + d.line_pos;
                ext_line(&mut s, a, [x, a[1]]);
                ext_line(&mut s, b, [x, b[1]]);
                dim_line(&mut s, [x, a[1]], [x, b[1]]);
                // text reads from the right, left of its line
                put_text(&mut s, [x - 1.0, (a[1] + b[1]) / 2.0], [0.0, 1.0], &text);
            }
        }
        DimKind::Aligned => {
            let (Some(ra), Some(rb)) = (r.first(), r.get(1)) else {
                return s;
            };
            let (a, b) = (sheet(ends(ra).0), sheet(ends(rb).0));
            let mut t = unit([b[0] - a[0], b[1] - a[1]]);
            if t[0] < -1e-12 || (t[0].abs() < 1e-12 && t[1] < 0.0) {
                t = [-t[0], -t[1]];
            }
            let n = [-t[1], t[0]];
            let off = |p: P| [p[0] + n[0] * d.line_pos, p[1] + n[1] * d.line_pos];
            let (a2, b2) = (off(a), off(b));
            ext_line(&mut s, a, a2);
            ext_line(&mut s, b, b2);
            dim_line(&mut s, a2, b2);
            let mid = [
                (a2[0] + b2[0]) / 2.0 + n[0] * 1.0,
                (a2[1] + b2[1]) / 2.0 + n[1] * 1.0,
            ];
            put_text(&mut s, mid, t, &text);
        }
        DimKind::Diameter | DimKind::Radius => {
            let Some(Resolved::Circle { centre, radius, .. }) = r.first() else {
                return s;
            };
            let c = sheet(*centre);
            let rr = radius * view.scale;
            let dir = [d.angle.cos(), d.angle.sin()];
            let far = [c[0] + dir[0] * (rr + LEAD), c[1] + dir[1] * (rr + LEAD)];
            let rim = [c[0] + dir[0] * rr, c[1] + dir[1] * rr];
            if d.kind == DimKind::Diameter {
                let opp = [c[0] - dir[0] * rr, c[1] - dir[1] * rr];
                s.line(Layer::Dimension, opp, far);
                arrow(&mut s, opp, [dir[0], dir[1]]);
            } else {
                s.line(Layer::Dimension, c, far);
            }
            arrow(&mut s, rim, [-dir[0], -dir[1]]);
            // the text sits on a short shelf at the end of the leader
            let w = text_len(&text);
            let right = dir[0] >= 0.0;
            let shelf_end = [far[0] + if right { w + 1.0 } else { -w - 1.0 }, far[1]];
            s.line(Layer::Dimension, far, shelf_end);
            put_text(
                &mut s,
                [(far[0] + shelf_end[0]) / 2.0, far[1] + 1.0],
                [1.0, 0.0],
                &text,
            );
        }
        DimKind::Angle => {
            let (Some(ra), Some(rb)) = (r.first(), r.get(1)) else {
                return s;
            };
            let (l1, l2) = (ends(ra), ends(rb));
            let (a1, b1, a2, b2) = (sheet(l1.0), sheet(l1.1), sheet(l2.0), sheet(l2.1));
            let (d1, d2) = (
                [b1[0] - a1[0], b1[1] - a1[1]],
                [b2[0] - a2[0], b2[1] - a2[1]],
            );
            let Some(v) = line_hit(a1, d1, a2, d2) else {
                return s;
            };
            // the rays leave the vertex toward the farther end of each edge
            let far = |a: P, b: P| {
                let (da, db) = (
                    (a[0] - v[0]).hypot(a[1] - v[1]),
                    (b[0] - v[0]).hypot(b[1] - v[1]),
                );
                if da >= db {
                    a
                } else {
                    b
                }
            };
            let (e1, e2) = (far(a1, b1), far(a2, b2));
            let (u1, u2) = (
                unit([e1[0] - v[0], e1[1] - v[1]]),
                unit([e2[0] - v[0], e2[1] - v[1]]),
            );
            let radius = if d.line_pos.abs() < 1.0 {
                20.0
            } else {
                d.line_pos.abs()
            };
            let (p1, p2) = (u1[1].atan2(u1[0]), u2[1].atan2(u2[0]));
            let cross = u1[0] * u2[1] - u1[1] * u2[0];
            let (a0, a1r) = if cross >= 0.0 { (p1, p2) } else { (p2, p1) };
            s.push(
                Layer::Dimension,
                Shape::Arc {
                    c: v,
                    r: radius,
                    a0,
                    a1: a1r,
                },
            );
            // extension lines where the arc lies beyond the edges
            for (e, u) in [(e1, u1), (e2, u2)] {
                let dist = (e[0] - v[0]).hypot(e[1] - v[1]);
                if dist < radius + 1.0 {
                    ext_line(&mut s, e, [v[0] + u[0] * radius, v[1] + u[1] * radius]);
                }
            }
            let tip = |a: f64| [v[0] + a.cos() * radius, v[1] + a.sin() * radius];
            // arrowheads along the arc's tangent
            let (ta, tb) = (tip(a0), tip(a1r));
            arrow(&mut s, ta, [a0.sin(), -a0.cos()]);
            arrow(&mut s, tb, [-a1r.sin(), a1r.cos()]);
            let mut sweep = a1r - a0;
            if sweep < 0.0 {
                sweep += std::f64::consts::TAU;
            }
            let mid = a0 + sweep / 2.0;
            let at = [
                v[0] + mid.cos() * (radius + 3.5),
                v[1] + mid.sin() * (radius + 3.5),
            ];
            put_text(&mut s, at, [1.0, 0.0], &text);
        }
    }
    s
}

/// Does the plane dimension direction `(du, dv)` of two anchors run mostly across the view?
pub fn mostly_horizontal(a: P, b: P) -> bool {
    (b[0] - a[0]).abs() >= (b[1] - a[1]).abs()
}

/// How far, in model units, the anchors of a dimension have moved from where they were — a quick
/// check that two resolved sets describe the same geometry.
pub fn same_resolution(a: &[Resolved], b: &[Resolved], tol: f64) -> bool {
    let near = |p: V3, q: V3| {
        dot(
            [p[0] - q[0], p[1] - q[1], p[2] - q[2]],
            [p[0] - q[0], p[1] - q[1], p[2] - q[2]],
        )
        .sqrt()
            <= tol
    };
    a.len() == b.len()
        && a.iter().zip(b).all(|(x, y)| match (x, y) {
            (Resolved::Point(p), Resolved::Point(q)) => near(*p, *q),
            (Resolved::Line { a: a1, b: b1 }, Resolved::Line { a: a2, b: b2 }) => {
                near(*a1, *a2) && near(*b1, *b2)
            }
            (
                Resolved::Circle {
                    centre: c1,
                    radius: r1,
                    ..
                },
                Resolved::Circle {
                    centre: c2,
                    radius: r2,
                    ..
                },
            ) => near(*c1, *c2) && (r1 - r2).abs() <= tol,
            _ => false,
        })
}

/// Where the resolved anchors of a dimension are in its view, relative to the view origin (sheet mm).
pub fn local_points(d: &Dimension, view: &View, cache: &ViewCache) -> Vec<(P, P)> {
    let f = ViewFrame::of(view.orientation);
    let at = |p: V3| {
        let q = uv(&f, p);
        [
            q[0] * view.scale - cache.centre[0],
            q[1] * view.scale - cache.centre[1],
        ]
    };
    d.resolved
        .iter()
        .map(|r| match r {
            Resolved::Point(p) => (at(*p), at(*p)),
            Resolved::Line { a, b } => (at(*a), at(*b)),
            Resolved::Circle { centre, .. } => (at(*centre), at(*centre)),
        })
        .collect()
}

/// Place a dimension so its line (or leader, or arc) passes through `p`, a point of the view
/// relative to the view origin (sheet mm). The inverse of what [`layout`] does with `line_pos`
/// and `angle`.
pub fn place(d: &mut Dimension, view: &View, cache: &ViewCache, p: P) {
    let pts = local_points(d, view, cache);
    match d.kind {
        DimKind::Horizontal => d.line_pos = p[1],
        DimKind::Vertical => d.line_pos = p[0],
        DimKind::Aligned => {
            if let (Some(a), Some(b)) = (pts.first(), pts.get(1)) {
                let mut t = unit([b.0[0] - a.0[0], b.0[1] - a.0[1]]);
                if t[0] < -1e-12 || (t[0].abs() < 1e-12 && t[1] < 0.0) {
                    t = [-t[0], -t[1]];
                }
                let n = [-t[1], t[0]];
                d.line_pos = (p[0] - a.0[0]) * n[0] + (p[1] - a.0[1]) * n[1];
            }
        }
        DimKind::Diameter | DimKind::Radius => {
            if let Some(c) = pts.first() {
                d.angle = (p[1] - c.0[1]).atan2(p[0] - c.0[0]);
            }
        }
        DimKind::Angle => {
            if let (Some(l1), Some(l2)) = (pts.first(), pts.get(1)) {
                let (d1, d2) = (
                    [l1.1[0] - l1.0[0], l1.1[1] - l1.0[1]],
                    [l2.1[0] - l2.0[0], l2.1[1] - l2.0[1]],
                );
                if let Some(v) = line_hit(l1.0, d1, l2.0, d2) {
                    d.line_pos = (p[0] - v[0]).hypot(p[1] - v[1]).max(2.0);
                }
            }
        }
    }
}
