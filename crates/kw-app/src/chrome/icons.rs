//! Icons drawn with the egui painter (no image assets).

use super::theme::{ACCENT, TEXT_WEAK};
use eframe::egui::{self, pos2, vec2, Color32, Painter, Pos2, Rect, Shape, Stroke};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    Sketch,
    Extrude,
    Revolve,
    Hole,
    Fillet,
    Chamfer,
    Shell,
    Pattern,
    CircularPattern,
    Mirror,
    Combine,
    Plane,
    Parameters,
    Line,
    Rectangle,
    Circle,
    Arc,
    Spline,
    Point,
    Trim,
    Extend,
    Offset,
    Dimension,
    Constraint,
    Finish,
    Select,
    Orbit,
    Pan,
    Zoom,
    Fit,
    Home,
    Eye,
    Folder,
    Body,
    Gear,
    Measure,
    Play,
    Cam,
    Fea,
    Board,
    Render,
    Generic,
}

const BLUE: Color32 = Color32::from_rgb(76, 160, 224);
const BLUE_DARK: Color32 = Color32::from_rgb(40, 110, 170);
const ORANGE: Color32 = Color32::from_rgb(235, 150, 70);
const GREEN: Color32 = Color32::from_rgb(70, 170, 90);
const GREY: Color32 = Color32::from_rgb(150, 150, 150);
const INK: Color32 = Color32::from_rgb(70, 70, 70);

fn line(p: &Painter, a: Pos2, b: Pos2, c: Color32, w: f32) {
    p.line_segment([a, b], Stroke::new(w, c));
}

fn plus(p: &Painter, at: Pos2, r: f32, c: Color32) {
    line(p, at - vec2(r, 0.0), at + vec2(r, 0.0), c, 2.2);
    line(p, at - vec2(0.0, r), at + vec2(0.0, r), c, 2.2);
}

fn arrow_up(p: &Painter, base: Pos2, h: f32, c: Color32) {
    line(p, base, base - vec2(0.0, h), c, 2.0);
    p.add(Shape::convex_polygon(
        vec![
            base - vec2(0.0, h + 4.0),
            base - vec2(-4.0, h - 2.0),
            base - vec2(4.0, h - 2.0),
        ],
        c,
        Stroke::NONE,
    ));
}

/// An isometric box in `r` (top face lighter).
fn iso_box(p: &Painter, r: Rect, top: Color32, left: Color32, right: Color32) {
    let (c, w, h) = (r.center(), r.width() / 2.0, r.height() / 2.0);
    let a = c + vec2(0.0, -h);
    let b = c + vec2(w, -h / 2.0);
    let d = c + vec2(0.0, 0.0);
    let e = c + vec2(-w, -h / 2.0);
    let f = c + vec2(-w, h / 2.0);
    let g = c + vec2(0.0, h);
    let hh = c + vec2(w, h / 2.0);
    p.add(Shape::convex_polygon(
        vec![a, b, d, e],
        top,
        Stroke::new(1.0, BLUE_DARK),
    ));
    p.add(Shape::convex_polygon(
        vec![e, d, g, f],
        left,
        Stroke::new(1.0, BLUE_DARK),
    ));
    p.add(Shape::convex_polygon(
        vec![d, b, hh, g],
        right,
        Stroke::new(1.0, BLUE_DARK),
    ));
}

/// Draw `icon` filling `rect` (about 32 px square).
pub fn paint(p: &Painter, rect: Rect, icon: Icon, enabled: bool) {
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
    let stroke = |col: Color32| Stroke::new(1.6, col);
    match icon {
        Icon::Sketch => {
            p.add(Shape::dashed_line(
                &[
                    q(-11.0, -8.0),
                    q(8.0, -8.0),
                    q(8.0, 9.0),
                    q(-11.0, 9.0),
                    q(-11.0, -8.0),
                ],
                stroke(ink),
                4.0,
                3.0,
            ));
            p.add(Shape::line(
                vec![q(-9.0, 5.0), q(-3.0, -4.0), q(4.0, 3.0)],
                stroke(blue_d),
            ));
            plus(p, q(9.0, 9.0), 5.0 * s, green);
        }
        Icon::Extrude => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 4.0), vec2(22.0 * s, 20.0 * s)),
                tint(Color32::from_rgb(160, 205, 240)),
                blue,
                tint(Color32::from_rgb(110, 175, 225)),
            );
            arrow_up(p, q(9.0, -4.0), 8.0 * s, ink);
        }
        Icon::Revolve => {
            p.add(Shape::line(
                (0..=20)
                    .map(|i| {
                        let a = std::f32::consts::PI * (0.15 + 1.5 * i as f32 / 20.0);
                        q(11.0 * a.cos(), 3.0 + 6.0 * a.sin())
                    })
                    .collect(),
                stroke(blue_d),
            ));
            p.add(Shape::convex_polygon(
                vec![q(-2.0, -12.0), q(8.0, -12.0), q(8.0, 8.0), q(-2.0, 8.0)],
                tint(Color32::from_rgb(160, 205, 240)),
                stroke(blue_d),
            ));
            line(p, q(-2.0, -14.0), q(-2.0, 12.0), ink, 1.4);
        }
        Icon::Hole => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 2.0), vec2(26.0 * s, 24.0 * s)),
                tint(Color32::from_rgb(190, 215, 235)),
                blue,
                tint(Color32::from_rgb(110, 175, 225)),
            );
            p.add(Shape::ellipse_filled(
                q(0.0, -5.0),
                vec2(6.0 * s, 3.0 * s),
                ink,
            ));
        }
        Icon::Fillet => {
            p.add(Shape::convex_polygon(
                vec![
                    q(-12.0, -12.0),
                    q(0.0, -12.0),
                    q(4.0, -10.0),
                    q(8.0, -6.0),
                    q(12.0, 0.0),
                    q(12.0, 12.0),
                    q(-12.0, 12.0),
                ],
                tint(Color32::from_rgb(190, 215, 235)),
                stroke(blue_d),
            ));
        }
        Icon::Chamfer => {
            p.add(Shape::convex_polygon(
                vec![
                    q(-12.0, -12.0),
                    q(0.0, -12.0),
                    q(12.0, 0.0),
                    q(12.0, 12.0),
                    q(-12.0, 12.0),
                ],
                tint(Color32::from_rgb(190, 215, 235)),
                stroke(blue_d),
            ));
        }
        Icon::Shell => {
            iso_box(
                p,
                Rect::from_center_size(q(0.0, 0.0), vec2(26.0 * s, 24.0 * s)),
                tint(Color32::from_rgb(220, 232, 244)),
                blue,
                tint(Color32::from_rgb(110, 175, 225)),
            );
            iso_box(
                p,
                Rect::from_center_size(q(0.0, -2.0), vec2(12.0 * s, 11.0 * s)),
                tint(Color32::WHITE),
                tint(Color32::from_rgb(210, 220, 230)),
                tint(Color32::from_rgb(190, 205, 220)),
            );
        }
        Icon::Pattern => {
            for i in 0..3 {
                let x = -11.0 + 9.0 * i as f32;
                p.rect(
                    Rect::from_center_size(q(x + 4.0, 0.0), vec2(7.0 * s, 14.0 * s)),
                    0.0,
                    blue,
                    stroke(blue_d),
                    egui::StrokeKind::Middle,
                );
            }
        }
        Icon::CircularPattern => {
            for i in 0..5 {
                let a = std::f32::consts::TAU * i as f32 / 5.0;
                p.circle_filled(q(10.0 * a.cos(), 10.0 * a.sin()), 3.2 * s, blue);
            }
            p.circle_stroke(q(0.0, 0.0), 10.0 * s, Stroke::new(1.0, grey));
        }
        Icon::Mirror => {
            p.add(Shape::convex_polygon(
                vec![q(-4.0, -10.0), q(-12.0, 8.0), q(-4.0, 8.0)],
                blue,
                stroke(blue_d),
            ));
            p.add(Shape::convex_polygon(
                vec![q(4.0, -10.0), q(12.0, 8.0), q(4.0, 8.0)],
                orange,
                stroke(blue_d),
            ));
            p.add(Shape::dashed_line(
                &[q(0.0, -13.0), q(0.0, 12.0)],
                stroke(ink),
                3.0,
                2.0,
            ));
        }
        Icon::Combine => {
            p.rect(
                Rect::from_center_size(q(-4.0, -3.0), vec2(16.0 * s, 16.0 * s)),
                0.0,
                blue,
                stroke(blue_d),
                egui::StrokeKind::Middle,
            );
            p.rect(
                Rect::from_center_size(q(5.0, 5.0), vec2(16.0 * s, 16.0 * s)),
                0.0,
                tint(Color32::from_rgba_unmultiplied(235, 150, 70, 200)),
                stroke(blue_d),
                egui::StrokeKind::Middle,
            );
        }
        Icon::Plane => {
            p.add(Shape::convex_polygon(
                vec![q(-12.0, 6.0), q(-4.0, -8.0), q(12.0, -8.0), q(4.0, 6.0)],
                tint(Color32::from_rgba_unmultiplied(6, 150, 215, 110)),
                stroke(blue_d),
            ));
        }
        Icon::Parameters => {
            p.text(
                c,
                egui::Align2::CENTER_CENTER,
                "fx",
                egui::FontId::proportional(16.0 * s),
                blue_d,
            );
        }
        Icon::Line => {
            line(p, q(-11.0, 10.0), q(11.0, -10.0), blue_d, 2.4);
            p.circle_filled(q(-11.0, 10.0), 2.6 * s, ink);
            p.circle_filled(q(11.0, -10.0), 2.6 * s, ink);
        }
        Icon::Rectangle => {
            p.rect_stroke(
                Rect::from_center_size(c, vec2(22.0 * s, 15.0 * s)),
                0.0,
                Stroke::new(2.0, blue_d),
                egui::StrokeKind::Middle,
            );
            p.circle_filled(q(-11.0, -7.5), 2.2 * s, ink);
            p.circle_filled(q(11.0, 7.5), 2.2 * s, ink);
        }
        Icon::Circle => {
            p.circle_stroke(c, 11.0 * s, Stroke::new(2.0, blue_d));
            p.circle_filled(c, 1.8 * s, ink);
        }
        Icon::Arc => {
            p.add(Shape::line(
                (0..=16)
                    .map(|i| {
                        let a = std::f32::consts::PI * (1.0 + i as f32 / 16.0);
                        q(11.0 * a.cos(), 5.0 + 11.0 * a.sin())
                    })
                    .collect(),
                Stroke::new(2.0, blue_d),
            ));
            p.circle_filled(q(-11.0, 5.0), 2.2 * s, ink);
            p.circle_filled(q(11.0, 5.0), 2.2 * s, ink);
        }
        Icon::Spline => {
            p.add(Shape::line(
                (0..=20)
                    .map(|i| {
                        let t = i as f32 / 20.0;
                        q(-12.0 + 24.0 * t, 7.0 * (t * std::f32::consts::TAU).sin())
                    })
                    .collect(),
                Stroke::new(2.0, blue_d),
            ));
        }
        Icon::Point => {
            p.circle_filled(c, 3.5 * s, ink);
            p.circle_stroke(c, 7.0 * s, Stroke::new(1.0, grey));
        }
        Icon::Trim => {
            line(p, q(-11.0, 0.0), q(11.0, 0.0), grey, 2.0);
            line(p, q(0.0, -11.0), q(0.0, 11.0), blue_d, 2.4);
            line(p, q(-5.0, -5.0), q(5.0, 5.0), orange, 2.0);
        }
        Icon::Extend => {
            line(p, q(-11.0, 6.0), q(2.0, 6.0), blue_d, 2.4);
            p.add(Shape::dashed_line(
                &[q(2.0, 6.0), q(11.0, 6.0)],
                Stroke::new(1.6, blue_d),
                3.0,
                2.0,
            ));
            line(p, q(11.0, -10.0), q(11.0, 12.0), ink, 2.0);
        }
        Icon::Offset => {
            p.rect_stroke(
                Rect::from_center_size(c, vec2(10.0 * s, 10.0 * s)),
                0.0,
                Stroke::new(1.8, blue_d),
                egui::StrokeKind::Middle,
            );
            p.rect_stroke(
                Rect::from_center_size(c, vec2(22.0 * s, 22.0 * s)),
                0.0,
                Stroke::new(1.8, orange),
                egui::StrokeKind::Middle,
            );
        }
        Icon::Dimension => {
            line(p, q(-12.0, 4.0), q(12.0, 4.0), ink, 1.6);
            line(p, q(-12.0, -2.0), q(-12.0, 10.0), ink, 1.6);
            line(p, q(12.0, -2.0), q(12.0, 10.0), ink, 1.6);
            p.text(
                q(0.0, -5.0),
                egui::Align2::CENTER_CENTER,
                "12",
                egui::FontId::proportional(11.0 * s),
                blue_d,
            );
        }
        Icon::Constraint => {
            line(p, q(-10.0, -6.0), q(10.0, -6.0), blue_d, 2.0);
            line(p, q(-10.0, 6.0), q(10.0, 6.0), blue_d, 2.0);
            p.text(
                c,
                egui::Align2::CENTER_CENTER,
                "//",
                egui::FontId::proportional(10.0 * s),
                ink,
            );
        }
        Icon::Finish => {
            p.circle_filled(c, 13.0 * s, tint(GREEN));
            p.add(Shape::line(
                vec![q(-6.0, 0.0), q(-1.5, 5.0), q(7.0, -5.0)],
                Stroke::new(3.0, Color32::WHITE),
            ));
        }
        Icon::Select => {
            p.add(Shape::dashed_line(
                &[
                    q(-12.0, -12.0),
                    q(8.0, -12.0),
                    q(8.0, 8.0),
                    q(-12.0, 8.0),
                    q(-12.0, -12.0),
                ],
                Stroke::new(1.4, GREEN),
                3.0,
                2.0,
            ));
            p.add(Shape::convex_polygon(
                vec![
                    q(-2.0, -2.0),
                    q(-2.0, 13.0),
                    q(2.0, 9.0),
                    q(5.0, 14.0),
                    q(8.0, 12.0),
                    q(5.0, 8.0),
                    q(10.0, 7.0),
                ],
                ink,
                Stroke::NONE,
            ));
        }
        Icon::Orbit => {
            p.add(Shape::ellipse_stroke(
                c,
                vec2(11.0 * s, 6.0 * s),
                Stroke::new(1.8, ink),
            ));
            p.circle_filled(c, 3.0 * s, blue);
        }
        Icon::Pan => {
            line(p, q(-11.0, 0.0), q(11.0, 0.0), ink, 1.8);
            line(p, q(0.0, -11.0), q(0.0, 11.0), ink, 1.8);
            p.circle_filled(c, 2.5 * s, blue);
        }
        Icon::Zoom => {
            p.circle_stroke(q(-2.0, -2.0), 8.0 * s, Stroke::new(2.0, ink));
            line(p, q(4.0, 4.0), q(11.0, 11.0), ink, 3.0);
        }
        Icon::Fit => {
            p.rect_stroke(
                Rect::from_center_size(c, vec2(20.0 * s, 20.0 * s)),
                0.0,
                Stroke::new(1.8, ink),
                egui::StrokeKind::Middle,
            );
            p.rect_filled(Rect::from_center_size(c, vec2(9.0 * s, 9.0 * s)), 0.0, blue);
        }
        Icon::Home => {
            p.add(Shape::convex_polygon(
                vec![
                    q(-11.0, 0.0),
                    q(0.0, -10.0),
                    q(11.0, 0.0),
                    q(7.0, 0.0),
                    q(7.0, 10.0),
                    q(-7.0, 10.0),
                    q(-7.0, 0.0),
                ],
                tint(Color32::from_rgb(190, 190, 190)),
                Stroke::new(1.2, ink),
            ));
        }
        Icon::Eye => {
            p.add(Shape::ellipse_stroke(
                c,
                vec2(10.0 * s, 5.5 * s),
                Stroke::new(1.6, ink),
            ));
            p.circle_filled(c, 3.0 * s, ink);
        }
        Icon::Folder => {
            p.add(Shape::convex_polygon(
                vec![
                    q(-11.0, -7.0),
                    q(-3.0, -7.0),
                    q(0.0, -4.0),
                    q(11.0, -4.0),
                    q(11.0, 8.0),
                    q(-11.0, 8.0),
                ],
                tint(Color32::from_rgb(220, 190, 110)),
                Stroke::new(1.0, grey),
            ));
        }
        Icon::Body => {
            iso_box(
                p,
                Rect::from_center_size(c, vec2(20.0 * s, 20.0 * s)),
                tint(Color32::from_rgb(225, 225, 230)),
                tint(Color32::from_rgb(190, 190, 200)),
                tint(Color32::from_rgb(160, 160, 172)),
            );
        }
        Icon::Gear => {
            p.circle_stroke(c, 8.0 * s, Stroke::new(3.0, grey));
            for i in 0..8 {
                let a = std::f32::consts::TAU * i as f32 / 8.0;
                line(
                    p,
                    q(8.0 * a.cos(), 8.0 * a.sin()),
                    q(12.0 * a.cos(), 12.0 * a.sin()),
                    grey,
                    3.0,
                );
            }
        }
        Icon::Measure => {
            p.rect_filled(
                Rect::from_center_size(c, vec2(26.0 * s, 8.0 * s)),
                1.0,
                tint(Color32::from_rgb(235, 200, 90)),
            );
            for i in 0..6 {
                let x = -11.0 + 4.4 * i as f32;
                line(
                    p,
                    q(x, -4.0),
                    q(x, if i % 2 == 0 { 0.0 } else { -1.0 }),
                    ink,
                    1.0,
                );
            }
        }
        Icon::Play => {
            p.add(Shape::convex_polygon(
                vec![q(-5.0, -8.0), q(8.0, 0.0), q(-5.0, 8.0)],
                ink,
                Stroke::NONE,
            ));
        }
        Icon::Cam => {
            p.add(Shape::line(
                vec![
                    q(-11.0, 8.0),
                    q(-11.0, -6.0),
                    q(0.0, -6.0),
                    q(0.0, 4.0),
                    q(11.0, 4.0),
                    q(11.0, -8.0),
                ],
                Stroke::new(2.0, orange),
            ));
            p.circle_filled(q(11.0, -8.0), 2.6 * s, ink);
        }
        Icon::Fea => {
            for (i, col) in [
                Color32::from_rgb(40, 80, 230),
                Color32::from_rgb(60, 200, 120),
                Color32::from_rgb(240, 200, 40),
                Color32::from_rgb(230, 60, 40),
            ]
            .into_iter()
            .enumerate()
            {
                p.rect_filled(
                    Rect::from_min_size(q(-12.0 + 6.0 * i as f32, -8.0), vec2(6.0 * s, 16.0 * s)),
                    0.0,
                    tint(col),
                );
            }
        }
        Icon::Board => {
            p.rect_filled(
                Rect::from_center_size(c, vec2(26.0 * s, 20.0 * s)),
                2.0,
                tint(Color32::from_rgb(40, 120, 70)),
            );
            line(
                p,
                q(-9.0, -4.0),
                q(2.0, -4.0),
                tint(Color32::from_rgb(230, 200, 90)),
                1.6,
            );
            line(
                p,
                q(2.0, -4.0),
                q(9.0, 5.0),
                tint(Color32::from_rgb(230, 200, 90)),
                1.6,
            );
            p.circle_filled(q(9.0, 5.0), 2.2 * s, tint(Color32::from_rgb(230, 200, 90)));
        }
        Icon::Render => {
            p.circle_filled(c, 11.0 * s, tint(Color32::from_rgb(210, 214, 225)));
            p.circle_filled(q(-3.5, -3.5), 4.0 * s, Color32::WHITE);
        }
        Icon::Generic => {
            p.rect_stroke(
                Rect::from_center_size(c, vec2(18.0 * s, 18.0 * s)),
                2.0,
                Stroke::new(1.6, grey),
                egui::StrokeKind::Middle,
            );
        }
    }
    let _ = (ACCENT, TEXT_WEAK);
}
