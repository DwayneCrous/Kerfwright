//! The orientation widget: a small cube that shows the camera orientation; clicking a face turns the view to it.

use super::theme::{ACCENT, BORDER, TEXT};
use eframe::egui::{
    self, pos2, vec2, Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, Ui, Vec2,
};
use kw_viewer::OrbitCamera;

/// (label, outward normal in world axes, yaw, pitch of a camera looking at that face)
const FACES: [(&str, [f64; 3], f64, f64); 6] = [
    ("TOP", [0.0, 0.0, 1.0], -std::f64::consts::FRAC_PI_2, 1.55),
    (
        "BOTTOM",
        [0.0, 0.0, -1.0],
        -std::f64::consts::FRAC_PI_2,
        -1.55,
    ),
    ("FRONT", [0.0, -1.0, 0.0], -std::f64::consts::FRAC_PI_2, 0.0),
    ("BACK", [0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2, 0.0),
    ("RIGHT", [1.0, 0.0, 0.0], 0.0, 0.0),
    ("LEFT", [-1.0, 0.0, 0.0], std::f64::consts::PI, 0.0),
];

/// Camera direction (yaw, pitch) to turn to when `face` is clicked.
pub fn face_view(label: &str) -> Option<(f64, f64)> {
    FACES.iter().find(|f| f.0 == label).map(|f| (f.2, f.3))
}

/// The camera direction (yaw, pitch) that looks at the cube from the side `d` points to: a face
/// (one non-zero component), an edge (two) or a corner (three).
pub fn direction_view(d: [i8; 3]) -> (f64, f64) {
    let v = [d[0] as f64, d[1] as f64, d[2] as f64];
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-12);
    if d[0] == 0 && d[1] == 0 {
        // straight above or below: keep the usual yaw
        return (-std::f64::consts::FRAC_PI_2, 1.55 * v[2].signum());
    }
    (v[1].atan2(v[0]), (v[2] / len).asin().clamp(-1.55, 1.55))
}

/// What a click on the cube's surroundings asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CubeMenu {
    Isometric,
    Fit,
    Previous,
    Shaded,
    Edges,
    Section,
}

/// What the orientation widget reports for a frame.
#[derive(Debug, Default)]
pub struct CubeOut {
    /// The (yaw, pitch) to animate to: a face, an edge, a corner or an arrow was clicked.
    pub look: Option<(f64, f64)>,
    pub menu: Option<CubeMenu>,
}

/// How far the bevel cuts into each face (fraction of the half edge).
const BEVEL: f64 = 0.34;

/// One clickable piece of the cube: a face, an edge or a corner.
struct Piece {
    dir: [i8; 3],
    label: &'static str,
    pts: Vec<[f64; 3]>,
}

fn pieces() -> Vec<Piece> {
    let k = 1.0 - BEVEL;
    let name = |d: [i8; 3]| -> &'static str {
        match d {
            [0, 0, 1] => "TOP",
            [0, 0, -1] => "BOTTOM",
            [0, -1, 0] => "FRONT",
            [0, 1, 0] => "BACK",
            [1, 0, 0] => "RIGHT",
            [-1, 0, 0] => "LEFT",
            _ => "",
        }
    };
    let mut out = Vec::new();
    for x in -1i8..=1 {
        for y in -1i8..=1 {
            for z in -1i8..=1 {
                let d = [x, y, z];
                let nz: Vec<usize> = (0..3).filter(|i| d[*i] != 0).collect();
                let at = |c: [f64; 3]| c;
                let pts: Vec<[f64; 3]> = match nz.len() {
                    1 => {
                        let a = nz[0];
                        let (t1, t2) = ((a + 1) % 3, (a + 2) % 3);
                        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
                            .iter()
                            .map(|(u, v)| {
                                let mut p = [0.0; 3];
                                p[a] = d[a] as f64;
                                p[t1] = u * k;
                                p[t2] = v * k;
                                at(p)
                            })
                            .collect()
                    }
                    2 => {
                        let (a1, a2) = (nz[0], nz[1]);
                        let f = 3 - a1 - a2;
                        let (s1, s2) = (d[a1] as f64, d[a2] as f64);
                        let mk = |outer1: bool, t: f64| {
                            let mut p = [0.0; 3];
                            p[a1] = s1 * if outer1 { 1.0 } else { k };
                            p[a2] = s2 * if outer1 { k } else { 1.0 };
                            p[f] = t;
                            p
                        };
                        vec![mk(true, -k), mk(true, k), mk(false, k), mk(false, -k)]
                    }
                    3 => {
                        let (s1, s2, s3) = (d[0] as f64, d[1] as f64, d[2] as f64);
                        vec![
                            [s1, s2 * k, s3 * k],
                            [s1 * k, s2, s3 * k],
                            [s1 * k, s2 * k, s3],
                        ]
                    }
                    _ => continue,
                };
                out.push(Piece {
                    dir: d,
                    label: name(d),
                    pts,
                });
            }
        }
    }
    out
}

/// Draw the cube and its surroundings in `rect` (about 130 px square): the bevelled cube (click a face, an
/// edge or a corner to look from there), turn arrows, and a menu button.
pub fn show(ui: &mut Ui, rect: Rect, cam: &OrbitCamera) -> CubeOut {
    let mut out = CubeOut::default();
    let view = cam.view();
    let rot = |p: [f64; 3]| -> (f32, f32, f64) {
        let x = view[(0, 0)] * p[0] + view[(0, 1)] * p[1] + view[(0, 2)] * p[2];
        let y = view[(1, 0)] * p[0] + view[(1, 1)] * p[1] + view[(1, 2)] * p[2];
        let z = view[(2, 0)] * p[0] + view[(2, 1)] * p[1] + view[(2, 2)] * p[2];
        (x as f32, y as f32, z)
    };
    let c = rect.center() + vec2(0.0, 2.0);
    let s = rect.width().min(rect.height()) * 0.23;
    let to_screen = |p: (f32, f32, f64)| -> Pos2 { pos2(c.x + p.0 * s, c.y - p.1 * s) };
    let cube_rect = Rect::from_center_size(c, vec2(rect.width() * 0.62, rect.height() * 0.62));
    let resp = ui.interact(
        cube_rect,
        ui.id().with("orientation_widget"),
        Sense::click(),
    );
    let painter = ui.painter_at(rect.expand(4.0));
    painter.circle_filled(
        c,
        rect.width() * 0.36,
        Color32::from_rgba_unmultiplied(255, 255, 255, 80),
    );
    // the pieces facing the viewer, far to near
    let mut vis: Vec<(f64, Piece, Vec<Pos2>)> = pieces()
        .into_iter()
        .filter_map(|p| {
            let n = [p.dir[0] as f64, p.dir[1] as f64, p.dir[2] as f64];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            let r = rot([n[0] / len, n[1] / len, n[2] / len]);
            if r.2 <= 0.12 {
                return None;
            }
            let pts = p.pts.iter().map(|q| to_screen(rot(*q))).collect();
            Some((r.2, p, pts))
        })
        .collect();
    vis.sort_by(|a, b| a.0.total_cmp(&b.0));
    let hover = resp.hover_pos();
    // faces win over the edges and corners next to them where they overlap on screen
    let hit = vis
        .iter()
        .rev()
        .find(|(_, _, pts)| hover.is_some_and(|h| point_in_poly(h, pts)))
        .map(|(_, p, _)| p.dir);
    for (_, p, pts) in &vis {
        let kind = p.dir.iter().filter(|v| **v != 0).count();
        let on = hit == Some(p.dir);
        let fill = if on {
            Color32::from_rgb(160, 210, 240)
        } else if kind == 1 {
            Color32::from_rgb(246, 246, 248)
        } else {
            Color32::from_rgb(214, 218, 224)
        };
        painter.add(Shape::convex_polygon(
            pts.clone(),
            fill,
            Stroke::new(1.0, BORDER),
        ));
        if kind == 1 {
            let mid = pts.iter().fold(vec2(0.0, 0.0), |a, q| a + q.to_vec2()) / pts.len() as f32;
            painter.text(
                mid.to_pos2(),
                Align2::CENTER_CENTER,
                p.label,
                FontId::proportional(7.5),
                if on { ACCENT } else { TEXT },
            );
        }
    }
    if let (Some(d), true) = (hit, resp.clicked()) {
        out.look = Some(direction_view(d));
    }
    // ---- turn arrows: a quarter turn about the vertical, or 45 degrees up and down
    let arrow = |ui: &mut Ui, at: Pos2, dir: Vec2, tip: &str| -> bool {
        let r = Rect::from_center_size(at, vec2(18.0, 18.0));
        let resp = ui
            .interact(r, ui.id().with(tip), Sense::click())
            .on_hover_text(tip);
        let col = if resp.hovered() {
            ACCENT
        } else {
            Color32::from_gray(110)
        };
        let n = vec2(-dir.y, dir.x);
        painter.add(Shape::convex_polygon(
            vec![
                at + dir * 6.0,
                at - dir * 4.0 + n * 6.0,
                at - dir * 4.0 - n * 6.0,
            ],
            col,
            Stroke::NONE,
        ));
        resp.clicked()
    };
    let (yaw_q, pitch) = (std::f64::consts::FRAC_PI_2, cam.pitch);
    let snapped = (cam.yaw / yaw_q).round() * yaw_q;
    let m = rect.width() * 0.46;
    if arrow(ui, c + vec2(-m, 0.0), vec2(-1.0, 0.0), "Turn left") {
        out.look = Some((snapped + yaw_q, pitch));
    }
    if arrow(ui, c + vec2(m, 0.0), vec2(1.0, 0.0), "Turn right") {
        out.look = Some((snapped - yaw_q, pitch));
    }
    if arrow(ui, c + vec2(0.0, -m), vec2(0.0, -1.0), "Look from higher") {
        out.look = Some((cam.yaw, (pitch + std::f64::consts::FRAC_PI_4).min(1.55)));
    }
    if arrow(ui, c + vec2(0.0, m), vec2(0.0, 1.0), "Look from lower") {
        out.look = Some((cam.yaw, (pitch - std::f64::consts::FRAC_PI_4).max(-1.55)));
    }
    // ---- the menu button under the cube
    let mb = Rect::from_center_size(rect.right_bottom() - vec2(14.0, 6.0), vec2(26.0, 16.0));
    let mresp = ui.interact(mb, ui.id().with("orientation_widget_menu"), Sense::click());
    painter.rect(
        mb,
        3.0,
        if mresp.hovered() {
            Color32::from_rgb(220, 235, 245)
        } else {
            Color32::from_rgb(246, 246, 248)
        },
        Stroke::new(1.0, BORDER),
        egui::StrokeKind::Inside,
    );
    painter.add(Shape::convex_polygon(
        vec![
            mb.center() + vec2(-9.0, -3.0),
            mb.center() + vec2(-3.0, -3.0),
            mb.center() + vec2(-6.0, 2.5),
        ],
        TEXT,
        Stroke::NONE,
    ));
    painter.rect_stroke(
        Rect::from_center_size(mb.center() + vec2(5.0, 0.0), vec2(8.0, 8.0)),
        1.0,
        Stroke::new(1.2, TEXT),
        egui::StrokeKind::Inside,
    );
    egui::Popup::menu(&mresp).show(|ui| {
        ui.set_min_width(170.0);
        for (label, action) in [
            ("Isometric", CubeMenu::Isometric),
            ("Zoom to fit", CubeMenu::Fit),
            ("Previous view", CubeMenu::Previous),
            ("Shaded / wireframe", CubeMenu::Shaded),
            ("Show edges", CubeMenu::Edges),
            ("Section view", CubeMenu::Section),
        ] {
            if ui.button(label).clicked() {
                out.menu = Some(action);
                ui.close();
            }
        }
    });
    out
}

fn point_in_poly(p: Pos2, poly: &[Pos2]) -> bool {
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Interpolate yaw (shortest way round) and pitch.
pub fn lerp_view(from: (f64, f64), to: (f64, f64), t: f64) -> (f64, f64) {
    let mut dy = (to.0 - from.0) % std::f64::consts::TAU;
    if dy > std::f64::consts::PI {
        dy -= std::f64::consts::TAU;
    } else if dy < -std::f64::consts::PI {
        dy += std::f64::consts::TAU;
    }
    let e = t * t * (3.0 - 2.0 * t);
    (from.0 + dy * e, from.1 + (to.1 - from.1) * e)
}

#[allow(dead_code)]
fn _unused(_: &egui::Context) {}
