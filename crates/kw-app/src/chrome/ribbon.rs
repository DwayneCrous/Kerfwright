//! Ribbon widgets: tab row, icon buttons and groups.

use super::icons::{self, Icon};
use super::theme::{ACCENT, ACCENT_SOFT, BORDER, PANEL_DARK, TEXT, TEXT_WEAK};
use eframe::egui::{self, vec2, FontId, Response, Sense, Stroke, Ui};

pub const BUTTON_W: f32 = 60.0;
pub const BUTTON_H: f32 = 66.0;

/// A big icon button with a label under the icon.
pub fn button(
    ui: &mut Ui,
    icon: Icon,
    label: &str,
    enabled: bool,
    active: bool,
    tip: &str,
) -> Response {
    let (rect, resp) = ui.allocate_exact_size(
        vec2(BUTTON_W, BUTTON_H),
        if enabled {
            Sense::click()
        } else {
            Sense::hover()
        },
    );
    let p = ui.painter();
    if active {
        p.rect(
            rect.shrink(1.0),
            3.0,
            ACCENT_SOFT,
            Stroke::new(1.0, ACCENT),
            egui::StrokeKind::Inside,
        );
    } else if resp.hovered() && enabled {
        p.rect(
            rect.shrink(1.0),
            3.0,
            ACCENT_SOFT.gamma_multiply(0.6),
            Stroke::new(1.0, ACCENT.gamma_multiply(0.6)),
            egui::StrokeKind::Inside,
        );
    }
    icons::paint(
        p,
        egui::Rect::from_center_size(rect.center_top() + vec2(0.0, 21.0), vec2(34.0, 34.0)),
        icon,
        enabled,
    );
    let mut job = egui::text::LayoutJob::single_section(
        label.to_string(),
        egui::TextFormat::simple(
            FontId::proportional(10.5),
            if enabled { TEXT } else { TEXT_WEAK },
        ),
    );
    job.wrap.max_width = BUTTON_W - 4.0;
    job.halign = egui::Align::Center;
    let galley = p.layout_job(job);
    p.galley(
        rect.center_top() + vec2(0.0, 41.0) - vec2(0.0, 0.0),
        galley,
        TEXT,
    );
    if !tip.is_empty() {
        resp.on_hover_text(tip)
    } else {
        resp
    }
}

/// A small icon-only button (used in the navigation bar and timeline).
pub fn small(ui: &mut Ui, icon: Icon, tip: &str, active: bool) -> Response {
    let (rect, resp) = ui.allocate_exact_size(vec2(26.0, 26.0), Sense::click());
    let p = ui.painter();
    if active {
        p.rect(
            rect.shrink(1.0),
            2.0,
            ACCENT_SOFT,
            Stroke::new(1.0, ACCENT),
            egui::StrokeKind::Inside,
        );
    } else if resp.hovered() {
        p.rect_filled(rect.shrink(1.0), 2.0, PANEL_DARK);
    }
    icons::paint(p, rect.shrink(2.0), icon, true);
    resp.on_hover_text(tip)
}

/// A group of buttons with a title under it and a separator to the right.
pub fn group(ui: &mut Ui, title: &str, add: impl FnOnce(&mut Ui)) {
    let r = ui
        .vertical(|ui| {
            ui.spacing_mut().item_spacing = vec2(2.0, 0.0);
            let row = ui.horizontal(|ui| add(ui));
            let w = row.response.rect.width().max(40.0);
            ui.add_sized(
                [w, 12.0],
                egui::Label::new(egui::RichText::new(title).size(10.0).color(TEXT_WEAK)),
            );
        })
        .response
        .rect;
    let x = r.right() + 5.0;
    ui.painter().line_segment(
        [
            egui::pos2(x, r.top() + 6.0),
            egui::pos2(x, r.bottom() - 6.0),
        ],
        Stroke::new(1.0, BORDER),
    );
    ui.add_space(10.0);
}

/// The tab row; returns true when `current` changed. A tab can be disabled (shown greyed).
pub fn tabs(ui: &mut Ui, labels: &[(&str, bool)], current: &mut usize) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 18.0;
        for (i, (label, enabled)) in labels.iter().enumerate() {
            let text = egui::RichText::new(*label).size(11.5).color(if *enabled {
                TEXT
            } else {
                TEXT_WEAK
            });
            let r = ui.add(egui::Label::new(text).sense(if *enabled {
                Sense::click()
            } else {
                Sense::hover()
            }));
            if *current == i {
                let y = r.rect.bottom() + 2.0;
                ui.painter().line_segment(
                    [egui::pos2(r.rect.left(), y), egui::pos2(r.rect.right(), y)],
                    Stroke::new(2.0, ACCENT),
                );
            }
            if r.clicked() && *enabled && *current != i {
                *current = i;
                changed = true;
            }
        }
    });
    changed
}
