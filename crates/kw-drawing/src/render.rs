//! A sheet (or the whole set) as a [`Scene2d`].

use crate::model::{scale_label, Sheet, View, ViewKind};
use crate::scene::{Anchor, Layer, Scene2d, P};
use crate::section::{cut_directions, to_sheet};

/// Arrow length and head size of a cutting-plane line, mm.
const ARROW: f64 = 9.0;
const HEAD: f64 = 2.8;
/// Length of the thick ends of a cutting-plane line, mm.
const END: f64 = 8.0;

impl Sheet {
    /// Border, title block, every view at its place on the sheet, and what refers to views from
    /// others (cutting-plane lines, detail circles). `no` is 1-based.
    pub fn scene(&self, no: usize, count: usize) -> Scene2d {
        let mut s = self.format.scene(&self.title, no, count);
        for v in &self.views {
            let Some(cache) = &v.cache else { continue };
            let placed = cache.scene.placed(1.0, v.pos);
            if let Some(l) = view_label(v, &placed) {
                s.extend(l);
            }
            s.extend(placed);
        }
        s.extend(self.annotations());
        s
    }

    /// The marks on a parent view that tell where a section is cut and what a detail enlarges.
    pub fn annotations(&self) -> Scene2d {
        let mut s = Scene2d::new();
        for v in &self.views {
            match &v.kind {
                ViewKind::Section {
                    parent,
                    a,
                    b,
                    flip,
                    letter,
                    ..
                } => {
                    let Some(p) = self.view(*parent) else {
                        continue;
                    };
                    let Some(pc) = &p.cache else { continue };
                    let at = |uv: P| to_sheet(p.pos, p.scale, pc.centre, uv);
                    let Some((d, n)) = cut_directions(*a, *b, *flip) else {
                        continue;
                    };
                    let (pa, pb) = (at(*a), at(*b));
                    // chain line between thick ends, arrows at both ends
                    s.line(Layer::Section, pa, pb);
                    let len = (pb[0] - pa[0]).hypot(pb[1] - pa[1]);
                    let end = END.min(len / 3.0);
                    s.line(Layer::Visible, pa, [pa[0] + d[0] * end, pa[1] + d[1] * end]);
                    s.line(Layer::Visible, pb, [pb[0] - d[0] * end, pb[1] - d[1] * end]);
                    for e in [pa, pb] {
                        let tip = [e[0] + n[0] * ARROW, e[1] + n[1] * ARROW];
                        s.line(Layer::Visible, e, tip);
                        let back = [tip[0] - n[0] * HEAD * 1.6, tip[1] - n[1] * HEAD * 1.6];
                        s.polyline(
                            Layer::Visible,
                            vec![
                                tip,
                                [back[0] + d[0] * HEAD * 0.5, back[1] + d[1] * HEAD * 0.5],
                                [back[0] - d[0] * HEAD * 0.5, back[1] - d[1] * HEAD * 0.5],
                            ],
                            true,
                        );
                        s.text(
                            Layer::Text,
                            [tip[0] + n[0] * 3.0 - 1.8, tip[1] + n[1] * 3.0 - 2.5],
                            5.0,
                            Anchor::Left,
                            letter,
                        );
                    }
                }
                ViewKind::Detail {
                    parent,
                    center,
                    radius,
                    letter,
                } => {
                    let Some(p) = self.view(*parent) else {
                        continue;
                    };
                    let Some(pc) = &p.cache else { continue };
                    let c = to_sheet(p.pos, p.scale, pc.centre, *center);
                    let r = radius * p.scale;
                    s.circle(Layer::Symbol, c, r);
                    let k = std::f64::consts::FRAC_1_SQRT_2;
                    s.text(
                        Layer::Text,
                        [c[0] + (r + 2.0) * k, c[1] + (r + 2.0) * k],
                        5.0,
                        Anchor::Left,
                        letter,
                    );
                }
                _ => {}
            }
        }
        s
    }
}

/// The caption under a view: its label, section letters and scale.
pub fn view_label(v: &View, placed: &Scene2d) -> Option<Scene2d> {
    let (lo, hi) = placed.bounds()?;
    let text = match &v.kind {
        ViewKind::Section { letter, .. } if v.label.is_empty() => {
            format!("SECTION {letter}-{letter}   SCALE {}", scale_label(v.scale))
        }
        ViewKind::Detail { letter, .. } if v.label.is_empty() => {
            format!("DETAIL {letter}   SCALE {}", scale_label(v.scale))
        }
        _ if v.label.is_empty() => return None,
        _ => format!("{}   SCALE {}", v.label, scale_label(v.scale)),
    };
    let mut s = Scene2d::new();
    s.text(
        Layer::Text,
        [(lo[0] + hi[0]) / 2.0, lo[1] - 7.0],
        3.5,
        Anchor::Center,
        &text,
    );
    Some(s)
}
