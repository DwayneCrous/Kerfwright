//! The light theme.

use eframe::egui::{self, Color32, CornerRadius, Stroke};

pub const ACCENT: Color32 = Color32::from_rgb(6, 150, 215);
pub const ACCENT_SOFT: Color32 = Color32::from_rgb(207, 232, 246);
pub const CHROME: Color32 = Color32::from_rgb(243, 243, 243);
pub const PANEL: Color32 = Color32::from_rgb(236, 236, 236);
pub const PANEL_DARK: Color32 = Color32::from_rgb(222, 222, 222);
pub const BORDER: Color32 = Color32::from_rgb(196, 196, 196);
pub const TEXT: Color32 = Color32::from_rgb(45, 45, 45);
pub const TEXT_WEAK: Color32 = Color32::from_rgb(120, 120, 120);
pub const GOOD: Color32 = Color32::from_rgb(70, 160, 80);
pub const BAD: Color32 = Color32::from_rgb(210, 70, 60);
/// Viewport clear colour (linear 0–1).
pub const VIEWPORT: [f64; 3] = [0.80, 0.84, 0.90];

pub fn apply(ctx: &egui::Context) {
    let mut v = egui::Visuals::light();
    v.panel_fill = PANEL;
    v.window_fill = CHROME;
    v.extreme_bg_color = Color32::WHITE;
    v.faint_bg_color = Color32::from_rgb(246, 246, 246);
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = ACCENT_SOFT;
    v.selection.stroke = Stroke::new(1.0, ACCENT);
    v.hyperlink_color = ACCENT;
    v.window_stroke = Stroke::new(1.0, BORDER);
    v.window_corner_radius = CornerRadius::same(3);
    let r = CornerRadius::same(2);
    for w in [
        &mut v.widgets.noninteractive,
        &mut v.widgets.inactive,
        &mut v.widgets.hovered,
        &mut v.widgets.active,
        &mut v.widgets.open,
    ] {
        w.corner_radius = r;
    }
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
    v.widgets.inactive.bg_fill = Color32::from_rgb(250, 250, 250);
    v.widgets.inactive.weak_bg_fill = Color32::from_rgb(250, 250, 250);
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, BORDER);
    v.widgets.hovered.bg_fill = ACCENT_SOFT;
    v.widgets.hovered.weak_bg_fill = ACCENT_SOFT;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT);
    v.widgets.active.bg_fill = Color32::from_rgb(180, 218, 240);
    v.widgets.active.weak_bg_fill = Color32::from_rgb(180, 218, 240);
    v.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    ctx.set_visuals(v);
}
