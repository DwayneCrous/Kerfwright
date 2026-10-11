//! The view cube: a small cube that shows the camera orientation; clicking a face turns the view to it.

use super::theme::{ACCENT, BORDER, TEXT};
use eframe::egui::{
    self, pos2, vec2, Align2, Color32, FontId, Pos2, Rect, Sense, Shape, Stroke, Ui,
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

fn face_corners(n: [f64; 3]) -> [[f64; 3]; 4] {
    // two tangent axes
    let (a, b) = if n[2].abs() > 0.5 {
        (0, 1)
    } else if n[1].abs() > 0.5 {
        (0, 2)
    } else {
        (1, 2)
    };
    let _ = (a, b);
    let mut ta = [0.0; 3];
    let mut tb = [0.0; 3];
    ta[a] = 1.0;
    tb[b] = 1.0;
    let corner = |sa: f64, sb: f64| {
        [
            n[0] + sa * ta[0] + sb * tb[0],
            n[1] + sa * ta[1] + sb * tb[1],
            n[2] + sa * ta[2] + sb * tb[2],
        ]
    };
    [
        corner(-1.0, -1.0),
        corner(1.0, -1.0),
        corner(1.0, 1.0),
        corner(-1.0, 1.0),
    ]
}

/// Draw the cube in `rect`; returns the (yaw, pitch) to animate to when a face was clicked.
pub fn show(ui: &mut Ui, rect: Rect, cam: &OrbitCamera) -> Option<(f64, f64)> {
    let view = cam.view();
    let rot = |p: [f64; 3]| -> (f32, f32, f64) {
        // view-space x right, y up, z toward the viewer is -z in a right-handed look-at
        let x = view[(0, 0)] * p[0] + view[(0, 1)] * p[1] + view[(0, 2)] * p[2];
        let y = view[(1, 0)] * p[0] + view[(1, 1)] * p[1] + view[(1, 2)] * p[2];
        let z = view[(2, 0)] * p[0] + view[(2, 1)] * p[1] + view[(2, 2)] * p[2];
        (x as f32, y as f32, z)
    };
    let c = rect.center();
    let s = rect.width().min(rect.height()) * 0.27;
    let to_screen = |p: (f32, f32, f64)| -> Pos2 { pos2(c.x + p.0 * s, c.y - p.1 * s) };
    let resp = ui.allocate_rect(rect, Sense::click());
    let painter = ui.painter_at(rect.expand(4.0));
    painter.circle_filled(
        c,
        rect.width() * 0.47,
        Color32::from_rgba_unmultiplied(255, 255, 255, 90),
    );
    // faces sorted far to near
    let mut faces: Vec<(f64, usize, Vec<Pos2>)> = FACES
        .iter()
        .enumerate()
        .filter_map(|(i, f)| {
            let n = rot(f.1);
            if n.2 <= 0.0 {
                return None; // faces away from the viewer
            }
            let pts: Vec<Pos2> = face_corners(f.1)
                .iter()
                .map(|q| to_screen(rot(*q)))
                .collect();
            Some((n.2, i, pts))
        })
        .collect();
    faces.sort_by(|a, b| a.0.total_cmp(&b.0));
    let hover = resp.hover_pos();
    let mut clicked = None;
    for (_, i, pts) in &faces {
        let inside = hover.is_some_and(|h| point_in_poly(h, pts));
        let fill = if inside {
            Color32::from_rgb(190, 225, 245)
        } else {
            Color32::from_rgb(246, 246, 248)
        };
        painter.add(Shape::convex_polygon(
            pts.clone(),
            fill,
            Stroke::new(1.0, BORDER),
        ));
        let mid = pts.iter().fold(vec2(0.0, 0.0), |a, p| a + p.to_vec2()) / 4.0;
        painter.text(
            mid.to_pos2(),
            Align2::CENTER_CENTER,
            FACES[*i].0,
            FontId::proportional(8.5),
            if inside { ACCENT } else { TEXT },
        );
        if inside && resp.clicked() {
            clicked = Some(*i);
        }
    }
    clicked.map(|i| (FACES[i].2, FACES[i].3))
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
