//! The second half of the icon set: solid tools, construction, sketch shapes and constraints, sheet metal
//! and Form. Same conventions as `icons.rs` (a 32-unit square, painter-only, tinted when disabled).

use super::icons::Icon;
use super::icons::{arrow_up, iso_box, line, plus, BLUE, BLUE_DARK, GREEN, GREY, INK, ORANGE};
use eframe::egui::{self, pos2, vec2, Color32, Painter, Pos2, Rect, Shape, Stroke};
use std::f32::consts::{PI, TAU};

const SOLID_TOP: Color32 = Color32::from_rgb(160, 205, 240);
const SOLID_SIDE: Color32 = Color32::from_rgb(110, 175, 225);
const PURPLE: Color32 = Color32::from_rgb(150, 110, 205);
const PURPLE_LIGHT: Color32 = Color32::from_rgb(200, 175, 235);
const SHEET: Color32 = Color32::from_rgb(240, 175, 115);
const RED: Color32 = Color32::from_rgb(220, 70, 60);

pub(super) fn paint(p: &Painter, rect: Rect, icon: Icon, enabled: bool) {
    let c = rect.center();
    let s = rect.width().min(rect.height()) / 32.0;
    let q = |x: f32, y: f32| pos2(c.x + x * s, c.y + y * s);
    let tint = |col: Color32| {
        if enabled {
            col
        } else {
            col.gamma_multiply(0.35)
        }
    };
    let (blue, blue_d, orange, green, grey, ink) = (
        tint(BLUE),
        tint(BLUE_DARK),
        tint(ORANGE),
        tint(GREEN),
        tint(GREY),
        tint(INK),
    );
    let (top, side, purple, purple_l, sheet, red) = (
        tint(SOLID_TOP),
        tint(SOLID_SIDE),
        tint(PURPLE),
        tint(PURPLE_LIGHT),
        tint(SHEET),
        tint(RED),
    );
    let st = |col: Color32| Stroke::new(1.6, col);
    let thick = |col: Color32| Stroke::new(2.2, col);
    let curve = |pts: Vec<Pos2>, stroke: Stroke| Shape::line(pts, stroke);
    let arc = |cx: f32, cy: f32, r: f32, a0: f32, a1: f32| -> Vec<Pos2> {
        (0..=24)
            .map(|i| {
                let a = a0 + (a1 - a0) * i as f32 / 24.0;
                q(cx + r * a.cos(), cy + r * a.sin())
            })
            .collect()
    };
    let dot = |x: f32, y: f32, col: Color32| p.circle_filled(q(x, y), 2.2 * s, col);
    let poly = |pts: &[(f32, f32)], fill: Color32, stroke: Stroke| {
        p.add(Shape::convex_polygon(
            pts.iter().map(|(x, y)| q(*x, *y)).collect(),
            fill,
            stroke,
        ));
    };
    match icon {
        // ------------------------------------------------------------------------- solid tools
        Icon::Sweep => {
            p.add(curve(
                arc(-2.0, 12.0, 17.0, -PI * 0.95, -PI * 0.1),
                Stroke::new(7.0, top),
            ));
            p.add(curve(
                arc(-2.0, 12.0, 17.0, -PI * 0.95, -PI * 0.1),
                st(blue_d),
            ));
            p.add(Shape::ellipse_filled(
                q(-18.5, 10.0),
                vec2(2.8 * s, 5.0 * s),
                blue,
            ));
            p.add(Shape::ellipse_stroke(
                q(-18.5, 10.0),
                vec2(2.8 * s, 5.0 * s),
                st(blue_d),
            ));
            p.add(Shape::ellipse_filled(
                q(15.0, 11.0),
                vec2(2.8 * s, 5.0 * s),
                orange,
            ));
        }
        Icon::Loft => {
            poly(
                &[(-12.0, 4.0), (2.0, 4.0), (2.0, 11.0), (-12.0, 11.0)],
                side,
                st(blue_d),
            );
            p.add(Shape::ellipse_filled(
                q(3.0, -8.0),
                vec2(9.0 * s, 4.0 * s),
                top,
            ));
            p.add(Shape::ellipse_stroke(
                q(3.0, -8.0),
                vec2(9.0 * s, 4.0 * s),
                st(blue_d),
            ));
            line(p, q(-12.0, 4.0), q(-5.0, -8.0), blue_d, 1.4);
            line(p, q(2.0, 4.0), q(11.0, -8.0), blue_d, 1.4);
            line(p, q(-12.0, 11.0), q(-6.0, -6.0), blue_d, 1.0);
        }
        Icon::Rib => {
            poly(
                &[(-12.0, 8.0), (12.0, 8.0), (12.0, 12.0), (-12.0, 12.0)],
                side,
                st(blue_d),
            );
            poly(
                &[(-12.0, 8.0), (-12.0, -10.0), (-8.0, -10.0), (-8.0, 8.0)],
                side,
                st(blue_d),
            );
            poly(&[(-8.0, 8.0), (-8.0, -2.0), (4.0, 8.0)], orange, st(blue_d));
        }
        Icon::Thread => {
            poly(
                &[(-7.0, -12.0), (7.0, -12.0), (7.0, 12.0), (-7.0, 12.0)],
                top,
                st(blue_d),
            );
            for i in 0..5 {
                let y = -10.0 + 5.0 * i as f32;
                line(p, q(-7.0, y + 3.0), q(7.0, y - 2.0), blue_d, 1.4);
            }
        }
        Icon::Pipe => {
            p.add(curve(
                arc(-2.0, 12.0, 16.0, -PI * 0.95, -PI * 0.1),
                Stroke::new(8.0, side),
            ));
            p.add(curve(
                arc(-2.0, 12.0, 16.0, -PI * 0.95, -PI * 0.1),
                Stroke::new(3.0, Color32::WHITE),
            ));
            p.add(Shape::ellipse_stroke(
                q(-17.5, 10.0),
                vec2(3.0 * s, 5.5 * s),
                st(blue_d),
            ));
        }
        Icon::Draft => {
            poly(
                &[(-10.0, 12.0), (10.0, 12.0), (5.0, -12.0), (-10.0, -12.0)],
                top,
                st(blue_d),
            );
            p.add(curve(
                arc(10.0, 12.0, 11.0, -PI * 0.62, -PI * 0.5),
                thick(orange),
            ));
            line(p, q(10.0, 12.0), q(10.0, -12.0), ink, 1.0);
        }
        Icon::Scale => {
            p.rect_stroke(
                Rect::from_center_size(q(-4.0, 4.0), vec2(12.0 * s, 12.0 * s)),
                0.0,
                st(blue_d),
                egui::StrokeKind::Middle,
            );
            p.rect_stroke(
                Rect::from_center_size(q(1.0, -1.0), vec2(24.0 * s, 24.0 * s)),
                0.0,
                Stroke::new(1.4, grey),
                egui::StrokeKind::Middle,
            );
            line(p, q(0.0, 0.0), q(11.0, -11.0), orange, 2.2);
            poly(
                &[(12.0, -12.0), (6.0, -11.0), (11.0, -6.0)],
                orange,
                Stroke::NONE,
            );
        }
        Icon::Move => {
            for (dx, dy) in [(0.0, -1.0), (0.0, 1.0), (-1.0, 0.0), (1.0, 0.0)] {
                line(p, q(0.0, 0.0), q(dx * 11.0, dy * 11.0), ink, 2.0);
                let (tx, ty) = (dx * 13.0, dy * 13.0);
                poly(
                    &[
                        (tx, ty),
                        (tx - dx * 5.0 + dy * 3.5, ty - dy * 5.0 + dx * 3.5),
                        (tx - dx * 5.0 - dy * 3.5, ty - dy * 5.0 - dx * 3.5),
                    ],
                    ink,
                    Stroke::NONE,
                );
            }
        }
        Icon::Split => {
            poly(
                &[(-12.0, -2.0), (0.0, -9.0), (12.0, -2.0), (0.0, 5.0)],
                top,
                st(blue_d),
            );
            poly(
                &[
                    (-12.0, 3.0),
                    (0.0, 10.0),
                    (12.0, 3.0),
                    (12.0, 8.0),
                    (0.0, 14.0),
                    (-12.0, 8.0),
                ],
                side,
                st(blue_d),
            );
            p.add(Shape::dashed_line(
                &[q(-14.0, 0.0), q(14.0, 0.0)],
                Stroke::new(1.6, orange),
                3.0,
                2.0,
            ));
        }
        Icon::OffsetBody => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 0.0), vec2(26.0 * s, 24.0 * s)),
                Color32::TRANSPARENT,
                Color32::TRANSPARENT,
                Color32::TRANSPARENT,
            );
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 1.0), vec2(14.0 * s, 13.0 * s)),
                top,
                blue,
                side,
            );
        }
        Icon::VarFillet => {
            poly(
                &[
                    (-12.0, -12.0),
                    (-2.0, -12.0),
                    (3.0, -10.0),
                    (8.0, -4.0),
                    (12.0, 4.0),
                    (12.0, 12.0),
                    (-12.0, 12.0),
                ],
                tint(Color32::from_rgb(190, 215, 235)),
                st(blue_d),
            );
            line(p, q(-8.0, -6.0), q(8.0, 6.0), orange, 1.6);
            dot(-8.0, -6.0, orange);
            dot(8.0, 6.0, orange);
        }
        Icon::Appearance => {
            p.circle_filled(
                q(0.0, 0.0),
                12.0 * s,
                tint(Color32::from_rgb(120, 170, 225)),
            );
            p.circle_filled(q(-4.0, -5.0), 4.0 * s, Color32::from_white_alpha(170));
            p.circle_filled(q(7.0, 8.0), 3.0 * s, orange);
            p.circle_filled(q(10.0, 3.0), 2.5 * s, green);
        }
        Icon::Component => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 2.0), vec2(22.0 * s, 20.0 * s)),
                top,
                blue,
                side,
            );
            plus(p, q(9.0, -9.0), 4.0 * s, green);
        }
        Icon::Joint => {
            poly(
                &[(-12.0, 0.0), (-2.0, 0.0), (-2.0, 12.0), (-12.0, 12.0)],
                side,
                st(blue_d),
            );
            poly(
                &[(2.0, -12.0), (12.0, -12.0), (12.0, 0.0), (2.0, 0.0)],
                top,
                st(blue_d),
            );
            p.circle_filled(q(0.0, 0.0), 3.4 * s, orange);
            p.circle_stroke(q(0.0, 0.0), 3.4 * s, st(ink));
        }
        Icon::Motion => {
            p.add(curve(
                arc(0.0, 0.0, 10.0, -PI * 0.2, PI * 1.35),
                thick(blue_d),
            ));
            poly(
                &[(-9.0, 9.0), (-3.0, 10.0), (-8.0, 3.0)],
                blue_d,
                Stroke::NONE,
            );
            dot(0.0, 0.0, orange);
        }
        // ------------------------------------------------------------------------- construction
        Icon::PlaneAngle => {
            poly(
                &[(-12.0, 8.0), (-4.0, 2.0), (12.0, 2.0), (4.0, 8.0)],
                tint(Color32::from_rgba_unmultiplied(6, 150, 215, 100)),
                st(blue_d),
            );
            poly(
                &[(-8.0, 10.0), (-1.0, -12.0), (12.0, -10.0), (6.0, 6.0)],
                tint(Color32::from_rgba_unmultiplied(235, 150, 70, 130)),
                st(orange),
            );
        }
        Icon::PlaneMid => {
            for dy in [-8.0, 0.0, 8.0] {
                poly(
                    &[
                        (-12.0, dy + 3.0),
                        (-5.0, dy - 3.0),
                        (12.0, dy - 3.0),
                        (5.0, dy + 3.0),
                    ],
                    tint(Color32::from_rgba_unmultiplied(
                        6,
                        150,
                        215,
                        if dy == 0.0 { 140 } else { 70 },
                    )),
                    st(if dy == 0.0 { orange } else { blue_d }),
                );
            }
        }
        Icon::Plane3Pt => {
            poly(
                &[(-12.0, 6.0), (-4.0, -8.0), (12.0, -8.0), (4.0, 6.0)],
                tint(Color32::from_rgba_unmultiplied(6, 150, 215, 100)),
                st(blue_d),
            );
            dot(-8.0, 4.0, ink);
            dot(8.0, -5.0, ink);
            dot(2.0, 1.0, ink);
        }
        Icon::Axis => {
            p.add(Shape::dashed_line(
                &[q(-12.0, 11.0), q(10.0, -9.0)],
                Stroke::new(2.0, orange),
                5.0,
                3.0,
            ));
            poly(
                &[(13.0, -12.0), (5.0, -10.0), (10.0, -4.0)],
                orange,
                Stroke::NONE,
            );
        }
        Icon::PointDatum => {
            poly(
                &[(-12.0, 8.0), (-4.0, -6.0), (12.0, -6.0), (4.0, 8.0)],
                tint(Color32::from_rgba_unmultiplied(6, 150, 215, 70)),
                st(blue_d),
            );
            p.circle_filled(q(1.0, 1.0), 3.6 * s, orange);
            p.circle_stroke(q(1.0, 1.0), 6.5 * s, Stroke::new(1.0, grey));
        }
        // ------------------------------------------------------------------------------- inspect
        Icon::Section => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 2.0), vec2(24.0 * s, 22.0 * s)),
                top,
                blue,
                side,
            );
            poly(
                &[(-13.0, -2.0), (0.0, -9.0), (13.0, -2.0), (0.0, 5.0)],
                tint(Color32::from_rgba_unmultiplied(235, 80, 60, 150)),
                st(red),
            );
        }
        Icon::Interference => {
            p.rect(
                Rect::from_center_size(q(-4.0, -3.0), vec2(16.0 * s, 16.0 * s)),
                0.0,
                tint(Color32::from_rgba_unmultiplied(76, 160, 224, 200)),
                st(blue_d),
                egui::StrokeKind::Middle,
            );
            p.rect(
                Rect::from_center_size(q(5.0, 5.0), vec2(16.0 * s, 16.0 * s)),
                0.0,
                tint(Color32::from_rgba_unmultiplied(235, 150, 70, 200)),
                st(blue_d),
                egui::StrokeKind::Middle,
            );
            p.rect_filled(Rect::from_min_max(q(-1.0, 2.0), q(4.0, 7.0)), 0.0, red);
        }
        // ------------------------------------------------------------------------ sketch shapes
        Icon::Polygon => {
            let pts: Vec<Pos2> = (0..6)
                .map(|i| {
                    let a = TAU * i as f32 / 6.0;
                    q(11.0 * a.cos(), 11.0 * a.sin())
                })
                .collect();
            p.add(Shape::closed_line(pts, Stroke::new(2.0, blue_d)));
            dot(0.0, 0.0, ink);
        }
        Icon::Slot => {
            p.add(curve(
                [
                    arc(-8.0, 0.0, 6.0, PI * 0.5, PI * 1.5),
                    arc(8.0, 0.0, 6.0, -PI * 0.5, PI * 0.5),
                ]
                .concat(),
                Stroke::new(2.0, blue_d),
            ));
            line(p, q(-8.0, -6.0), q(8.0, -6.0), blue_d, 2.0);
            line(p, q(-8.0, 6.0), q(8.0, 6.0), blue_d, 2.0);
            dot(-8.0, 0.0, ink);
            dot(8.0, 0.0, ink);
        }
        Icon::Ellipse => {
            p.add(Shape::ellipse_stroke(
                c,
                vec2(13.0 * s, 7.5 * s),
                Stroke::new(2.0, blue_d),
            ));
            dot(0.0, 0.0, ink);
        }
        Icon::Arc3 => {
            p.add(curve(
                arc(0.0, 6.0, 12.0, -PI * 0.95, -PI * 0.05),
                Stroke::new(2.0, blue_d),
            ));
            dot(-11.5, 5.0, ink);
            dot(11.5, 5.0, ink);
            dot(0.0, -6.0, orange);
        }
        Icon::Circle3 => {
            p.circle_stroke(c, 11.0 * s, Stroke::new(2.0, blue_d));
            for a in [-PI * 0.5, PI * 0.1, PI * 0.9] {
                dot(11.0 * a.cos(), 11.0 * a.sin(), ink);
            }
        }
        Icon::FitSpline => {
            let pts: Vec<Pos2> = (0..=20)
                .map(|i| {
                    let t = i as f32 / 20.0;
                    q(-12.0 + 24.0 * t, 7.0 * (t * TAU).sin())
                })
                .collect();
            p.add(Shape::line(pts, Stroke::new(2.0, blue_d)));
            for t in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
                dot(-12.0 + 24.0 * t, 7.0 * (t * TAU).sin(), ink);
            }
        }
        Icon::SkFillet => {
            line(p, q(-12.0, 12.0), q(-12.0, -2.0), blue_d, 2.2);
            p.add(curve(
                arc(-4.0, -2.0, 8.0, PI, PI * 1.5),
                Stroke::new(2.2, orange),
            ));
            line(p, q(-4.0, -10.0), q(12.0, -10.0), blue_d, 2.2);
        }
        Icon::SkChamfer => {
            line(p, q(-12.0, 12.0), q(-12.0, -2.0), blue_d, 2.2);
            line(p, q(-12.0, -2.0), q(-4.0, -10.0), orange, 2.2);
            line(p, q(-4.0, -10.0), q(12.0, -10.0), blue_d, 2.2);
        }
        Icon::SkMirror => {
            poly(
                &[(-4.0, -10.0), (-12.0, 8.0), (-4.0, 8.0)],
                tint(Color32::TRANSPARENT),
                st(blue_d),
            );
            poly(
                &[(4.0, -10.0), (12.0, 8.0), (4.0, 8.0)],
                tint(Color32::TRANSPARENT),
                st(orange),
            );
            p.add(Shape::dashed_line(
                &[q(0.0, -13.0), q(0.0, 12.0)],
                st(ink),
                3.0,
                2.0,
            ));
        }
        Icon::SkPattern => {
            for r in 0..2 {
                for k in 0..3 {
                    p.circle_stroke(
                        q(-9.0 + 9.0 * k as f32, -5.0 + 11.0 * r as f32),
                        3.4 * s,
                        st(blue_d),
                    );
                }
            }
        }
        Icon::Project => {
            line(p, q(-12.0, 11.0), q(12.0, 11.0), blue_d, 2.4);
            p.add(Shape::dashed_line(
                &[q(0.0, -10.0), q(0.0, 9.0)],
                Stroke::new(1.6, ink),
                3.0,
                2.0,
            ));
            poly(&[(0.0, 11.0), (-3.5, 5.0), (3.5, 5.0)], ink, Stroke::NONE);
            p.add(Shape::ellipse_stroke(
                q(0.0, -10.0),
                vec2(8.0 * s, 3.0 * s),
                st(orange),
            ));
        }
        Icon::Text => {
            p.text(
                c,
                egui::Align2::CENTER_CENTER,
                "A",
                egui::FontId::proportional(24.0 * s),
                blue_d,
            );
            line(p, q(-10.0, 12.0), q(10.0, 12.0), orange, 1.8);
        }
        Icon::Dxf => {
            poly(
                &[
                    (-8.0, -12.0),
                    (4.0, -12.0),
                    (10.0, -6.0),
                    (10.0, 12.0),
                    (-8.0, 12.0),
                ],
                Color32::WHITE,
                st(grey),
            );
            p.text(
                q(0.0, 3.0),
                egui::Align2::CENTER_CENTER,
                "DXF",
                egui::FontId::proportional(7.5 * s),
                blue_d,
            );
        }
        // ------------------------------------------------------------------- sketch constraints
        Icon::CCoincident => {
            p.circle_stroke(q(0.0, 0.0), 8.0 * s, Stroke::new(1.6, blue_d));
            dot(0.0, 0.0, ink);
            line(p, q(-12.0, 10.0), q(0.0, 0.0), grey, 1.6);
            line(p, q(12.0, 10.0), q(0.0, 0.0), grey, 1.6);
        }
        Icon::CHorizontal => {
            line(p, q(-12.0, 0.0), q(12.0, 0.0), blue_d, 2.4);
            dot(-12.0, 0.0, ink);
            dot(12.0, 0.0, ink);
            line(p, q(-5.0, 7.0), q(5.0, 7.0), orange, 2.0);
        }
        Icon::CVertical => {
            line(p, q(0.0, -12.0), q(0.0, 12.0), blue_d, 2.4);
            dot(0.0, -12.0, ink);
            dot(0.0, 12.0, ink);
            line(p, q(7.0, -5.0), q(7.0, 5.0), orange, 2.0);
        }
        Icon::CParallel => {
            line(p, q(-12.0, 4.0), q(4.0, -12.0), blue_d, 2.4);
            line(p, q(-4.0, 12.0), q(12.0, -4.0), blue_d, 2.4);
            p.text(
                q(2.0, 2.0),
                egui::Align2::CENTER_CENTER,
                "//",
                egui::FontId::proportional(9.0 * s),
                orange,
            );
        }
        Icon::CPerpendicular => {
            line(p, q(-10.0, 11.0), q(10.0, 11.0), blue_d, 2.4);
            line(p, q(-2.0, 11.0), q(-2.0, -11.0), blue_d, 2.4);
            p.add(Shape::line(
                vec![q(-2.0, 5.0), q(4.0, 5.0), q(4.0, 11.0)],
                st(orange),
            ));
        }
        Icon::CTangent => {
            p.circle_stroke(q(0.0, 4.0), 8.0 * s, Stroke::new(2.0, blue_d));
            line(p, q(-12.0, -4.0), q(12.0, -4.0), orange, 2.2);
        }
        Icon::CEqual => {
            line(p, q(-9.0, -4.0), q(9.0, -4.0), blue_d, 3.0);
            line(p, q(-9.0, 4.0), q(9.0, 4.0), blue_d, 3.0);
        }
        Icon::CFix => {
            p.circle_stroke(q(0.0, -3.0), 5.0 * s, Stroke::new(2.4, blue_d));
            poly(
                &[(-8.0, 0.0), (8.0, 0.0), (8.0, 11.0), (-8.0, 11.0)],
                orange,
                st(ink),
            );
        }
        Icon::CMidpoint => {
            line(p, q(-12.0, 4.0), q(12.0, 4.0), blue_d, 2.2);
            poly(&[(0.0, -9.0), (-5.0, 0.0), (5.0, 0.0)], orange, st(ink));
            dot(-12.0, 4.0, ink);
            dot(12.0, 4.0, ink);
        }
        Icon::CConcentric => {
            p.circle_stroke(c, 11.0 * s, Stroke::new(2.0, blue_d));
            p.circle_stroke(c, 5.0 * s, Stroke::new(2.0, orange));
            dot(0.0, 0.0, ink);
        }
        Icon::CCollinear => {
            line(p, q(-12.0, 5.0), q(-3.0, 5.0), blue_d, 2.4);
            line(p, q(3.0, 5.0), q(12.0, 5.0), blue_d, 2.4);
            p.add(Shape::dashed_line(
                &[q(-13.0, 5.0), q(13.0, 5.0)],
                Stroke::new(1.0, orange),
                2.0,
                2.0,
            ));
            line(p, q(-6.0, -5.0), q(6.0, -5.0), grey, 1.4);
        }
        Icon::COnCurve => {
            p.add(curve(
                arc(0.0, 10.0, 13.0, -PI * 0.85, -PI * 0.15),
                Stroke::new(2.2, blue_d),
            ));
            dot(0.0, -3.0, orange);
        }
        Icon::CSymmetric => {
            p.add(Shape::dashed_line(
                &[q(0.0, -13.0), q(0.0, 12.0)],
                st(ink),
                3.0,
                2.0,
            ));
            dot(-8.0, -4.0, blue_d);
            dot(8.0, -4.0, blue_d);
            dot(-5.0, 7.0, orange);
            dot(5.0, 7.0, orange);
        }
        // --------------------------------------------------------------- sheet metal and Form
        Icon::Flange => {
            poly(
                &[
                    (-12.0, 8.0),
                    (4.0, 8.0),
                    (4.0, -10.0),
                    (8.0, -10.0),
                    (8.0, 12.0),
                    (-12.0, 12.0),
                ],
                sheet,
                st(blue_d),
            );
        }
        Icon::Bend => {
            p.add(curve(
                arc(-2.0, 12.0, 14.0, -PI * 0.95, -PI * 0.05),
                Stroke::new(6.0, sheet),
            ));
            p.add(curve(
                arc(-2.0, 12.0, 14.0, -PI * 0.95, -PI * 0.05),
                st(blue_d),
            ));
        }
        Icon::Unfold => {
            poly(
                &[(-13.0, 2.0), (13.0, 2.0), (13.0, 8.0), (-13.0, 8.0)],
                sheet,
                st(blue_d),
            );
            for x in [-5.0, 5.0] {
                p.add(Shape::dashed_line(
                    &[q(x, 0.0), q(x, 10.0)],
                    st(ink),
                    2.0,
                    1.5,
                ));
            }
            line(p, q(-12.0, -6.0), q(12.0, -6.0), ink, 1.6);
            poly(&[(14.0, -6.0), (9.0, -9.0), (9.0, -3.0)], ink, Stroke::NONE);
        }
        Icon::Refold => {
            poly(
                &[
                    (-12.0, 8.0),
                    (4.0, 8.0),
                    (4.0, -10.0),
                    (8.0, -10.0),
                    (8.0, 12.0),
                    (-12.0, 12.0),
                ],
                sheet,
                st(blue_d),
            );
            p.add(curve(arc(0.0, 0.0, 7.0, PI * 1.1, PI * 1.9), st(ink)));
        }
        Icon::FormBox => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 2.0), vec2(24.0 * s, 22.0 * s)),
                purple_l,
                purple,
                tint(Color32::from_rgb(120, 85, 175)),
            );
            line(
                p,
                q(0.0, -9.0),
                q(0.0, 13.0),
                Color32::from_white_alpha(120),
                1.0,
            );
        }
        Icon::FormCylinder => {
            p.add(Shape::ellipse_filled(
                q(0.0, 8.0),
                vec2(10.0 * s, 4.0 * s),
                purple,
            ));
            p.rect_filled(
                Rect::from_min_max(q(-10.0, -8.0), q(10.0, 8.0)),
                0.0,
                purple,
            );
            p.add(Shape::ellipse_filled(
                q(0.0, -8.0),
                vec2(10.0 * s, 4.0 * s),
                purple_l,
            ));
            for x in [-5.0, 0.0, 5.0] {
                line(
                    p,
                    q(x, -8.0),
                    q(x, 8.0),
                    Color32::from_white_alpha(110),
                    1.0,
                );
            }
        }
        Icon::FormSphere => {
            p.circle_filled(c, 12.0 * s, purple);
            p.add(Shape::ellipse_stroke(
                c,
                vec2(12.0 * s, 4.0 * s),
                Stroke::new(1.0, Color32::from_white_alpha(130)),
            ));
            p.add(Shape::ellipse_stroke(
                c,
                vec2(4.0 * s, 12.0 * s),
                Stroke::new(1.0, Color32::from_white_alpha(130)),
            ));
        }
        Icon::FormTorus => {
            p.add(Shape::ellipse_stroke(
                c,
                vec2(11.0 * s, 8.0 * s),
                Stroke::new(6.0, purple),
            ));
            p.add(Shape::ellipse_stroke(
                c,
                vec2(11.0 * s, 8.0 * s),
                Stroke::new(1.0, Color32::from_white_alpha(120)),
            ));
        }
        Icon::FormPlane => {
            poly(
                &[(-12.0, 6.0), (-4.0, -8.0), (12.0, -8.0), (4.0, 6.0)],
                purple_l,
                st(purple),
            );
            line(p, q(-8.0, -1.0), q(8.0, -1.0), purple, 1.0);
            line(p, q(0.0, -8.0), q(-4.0, 6.0), purple, 1.0);
        }
        Icon::FormFinish => {
            p.circle_filled(c, 13.0 * s, purple);
            p.add(Shape::line(
                vec![q(-6.0, 0.0), q(-1.5, 5.0), q(7.0, -5.0)],
                Stroke::new(3.0, Color32::WHITE),
            ));
        }
        Icon::Crease => {
            poly(
                &[(-12.0, 8.0), (0.0, -8.0), (12.0, 8.0)],
                purple_l,
                st(purple),
            );
            line(p, q(0.0, -8.0), q(0.0, 8.0), orange, 2.4);
        }
        Icon::Chevron => {
            p.add(Shape::line(
                vec![q(-5.0, -2.0), q(0.0, 3.0), q(5.0, -2.0)],
                Stroke::new(1.8, ink),
            ));
        }
        _ => {
            let _ = (arrow_up, green);
            p.rect_stroke(
                Rect::from_center_size(c, vec2(18.0 * s, 18.0 * s)),
                2.0,
                Stroke::new(1.6, grey),
                egui::StrokeKind::Middle,
            );
        }
    }
}
