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
    // a long word (Interference) gets a smaller font instead of breaking in the middle
    let longest = label
        .split_whitespace()
        .map(|w| w.chars().count())
        .max()
        .unwrap_or(0);
    let size = if longest > 10 {
        9.0
    } else if longest > 9 {
        9.5
    } else {
        10.5
    };
    let mut job = egui::text::LayoutJob::single_section(
        label.to_string(),
        egui::TextFormat::simple(
            FontId::proportional(size),
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

// ------------------------------------------------------------------------------ menu groups

/// One command of the ribbon: an icon, a name, and the id the caller matches on.
#[derive(Clone)]
pub struct Item {
    pub id: &'static str,
    pub icon: Icon,
    pub label: &'static str,
    pub tip: String,
    pub key: &'static str,
    pub enabled: bool,
    pub active: bool,
}

impl Item {
    pub fn new(id: &'static str, icon: Icon, label: &'static str) -> Item {
        Item {
            id,
            icon,
            label,
            tip: String::new(),
            key: "",
            enabled: true,
            active: false,
        }
    }
    pub fn tip(mut self, tip: &str) -> Item {
        self.tip = tip.to_string();
        self
    }
    pub fn key(mut self, key: &'static str) -> Item {
        self.key = key;
        self
    }
    pub fn active(mut self, on: bool) -> Item {
        self.active = on;
        self
    }
    /// Enabled when `cond` holds; otherwise greyed, and `why` is added to the tooltip.
    pub fn enabled_if(mut self, cond: bool, why: &str) -> Item {
        self.enabled = cond;
        if !cond {
            if !self.tip.is_empty() {
                self.tip.push_str("\n\n");
            }
            self.tip.push_str(&format!("Unavailable: {why}"));
        }
        self
    }
}

/// What a group shows as big buttons: a plain command, or a command with a dropdown of variants.
pub enum Prim {
    One(Item),
    Split(Item, Vec<Item>),
}

impl From<Item> for Prim {
    fn from(i: Item) -> Prim {
        Prim::One(i)
    }
}

/// A headed list of items in a dropdown.
pub struct Section {
    pub heading: &'static str,
    pub items: Vec<Item>,
}

impl Section {
    pub fn new(heading: &'static str, items: Vec<Item>) -> Section {
        Section { heading, items }
    }
}

const MENU_W: f32 = 250.0;
const ROW_H: f32 = 26.0;

/// The rows of a dropdown; returns the id clicked.
fn menu_rows(ui: &mut Ui, sections: &[Section]) -> Option<&'static str> {
    let mut picked = None;
    ui.set_min_width(MENU_W);
    for (n, sec) in sections.iter().enumerate() {
        if n > 0 {
            ui.separator();
        }
        if !sec.heading.is_empty() {
            ui.label(
                egui::RichText::new(sec.heading)
                    .size(9.5)
                    .strong()
                    .color(TEXT_WEAK),
            );
        }
        for it in &sec.items {
            let (rect, resp) = ui.allocate_exact_size(
                vec2(MENU_W, ROW_H),
                if it.enabled {
                    Sense::click()
                } else {
                    Sense::hover()
                },
            );
            let p = ui.painter();
            if resp.hovered() && it.enabled {
                p.rect_filled(rect, 2.0, ACCENT_SOFT);
            } else if it.active {
                p.rect_filled(rect, 2.0, ACCENT_SOFT.gamma_multiply(0.6));
            }
            icons::paint(
                p,
                egui::Rect::from_center_size(
                    rect.left_center() + vec2(16.0, 0.0),
                    vec2(22.0, 22.0),
                ),
                it.icon,
                it.enabled,
            );
            p.text(
                rect.left_center() + vec2(36.0, 0.0),
                egui::Align2::LEFT_CENTER,
                it.label,
                FontId::proportional(12.0),
                if it.enabled { TEXT } else { TEXT_WEAK },
            );
            if !it.key.is_empty() {
                p.text(
                    rect.right_center() - vec2(8.0, 0.0),
                    egui::Align2::RIGHT_CENTER,
                    it.key,
                    FontId::proportional(10.5),
                    TEXT_WEAK,
                );
            }
            let resp = if it.tip.is_empty() {
                resp
            } else {
                resp.on_hover_text(&it.tip)
            };
            if resp.clicked() && it.enabled {
                picked = Some(it.id);
                ui.close();
            }
        }
    }
    picked
}

/// A group like Fusion's: big icon buttons for the commonly used commands, and the group title as a
/// `TITLE ▾` button that opens a dropdown with every command of the group, under category headings.
/// Returns the id of whatever was clicked, in the buttons or in the dropdown.
pub fn menu_group(
    ui: &mut Ui,
    title: &str,
    primary: &[Prim],
    sections: &[Section],
) -> Option<&'static str> {
    let mut picked: Option<&'static str> = None;
    let r = ui
        .vertical(|ui| {
            ui.spacing_mut().item_spacing = vec2(2.0, 0.0);
            let row = ui.horizontal(|ui| {
                for prim in primary {
                    match prim {
                        Prim::One(it) => {
                            if button(ui, it.icon, it.label, it.enabled, it.active, &it.tip)
                                .clicked()
                                && it.enabled
                            {
                                picked = Some(it.id);
                            }
                        }
                        Prim::Split(main, variants) => {
                            if let Some(id) = split_button(ui, main, variants) {
                                picked = Some(id);
                            }
                        }
                    }
                }
            });
            let w = row.response.rect.width().max(58.0);
            let (rect, resp) = ui.allocate_exact_size(vec2(w, 14.0), Sense::click());
            let p = ui.painter();
            if resp.hovered() {
                p.rect_filled(rect, 2.0, PANEL_DARK);
            }
            let galley = p.layout_no_wrap(title.to_string(), FontId::proportional(10.0), TEXT_WEAK);
            let total = galley.size().x + 12.0;
            let x0 = rect.center().x - total / 2.0;
            p.galley(
                egui::pos2(x0, rect.center().y - galley.size().y / 2.0),
                galley,
                TEXT_WEAK,
            );
            icons::paint(
                p,
                egui::Rect::from_center_size(
                    egui::pos2(x0 + total - 4.0, rect.center().y + 1.0),
                    vec2(12.0, 12.0),
                ),
                Icon::Chevron,
                true,
            );
            if !sections.is_empty() {
                // screenshots and tests can hold one dropdown open by naming its group
                let forced = ui
                    .ctx()
                    .data(|d| d.get_temp::<String>(egui::Id::new("force_open_menu")))
                    .is_some_and(|t| t == title);
                let mut always_open = true;
                let popup = if forced {
                    egui::Popup::menu(&resp).open_bool(&mut always_open)
                } else {
                    egui::Popup::menu(&resp)
                };
                if let Some(inner) = popup.show(|ui| menu_rows(ui, sections)) {
                    if let Some(id) = inner.inner {
                        picked = Some(id);
                    }
                }
            }
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
    picked
}

/// A big button whose lower strip opens a dropdown of variants (Circle ▾: centre, three points …).
/// Clicking the icon runs `main`; clicking the strip lists `variants`.
pub fn split_button(ui: &mut Ui, main: &Item, variants: &[Item]) -> Option<&'static str> {
    let mut picked = None;
    let (rect, resp) = ui.allocate_exact_size(vec2(BUTTON_W, BUTTON_H), Sense::click());
    let strip = egui::Rect::from_min_max(egui::pos2(rect.left(), rect.bottom() - 17.0), rect.max);
    let chev = ui.interact(strip, resp.id.with("variants"), Sense::click());
    let p = ui.painter();
    if main.active {
        p.rect(
            rect.shrink(1.0),
            3.0,
            ACCENT_SOFT,
            Stroke::new(1.0, ACCENT),
            egui::StrokeKind::Inside,
        );
    } else if resp.hovered() {
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
        egui::Rect::from_center_size(rect.center_top() + vec2(0.0, 20.0), vec2(34.0, 34.0)),
        main.icon,
        main.enabled,
    );
    let galley = p.layout_no_wrap(
        main.label.to_string(),
        FontId::proportional(10.5),
        if main.enabled { TEXT } else { TEXT_WEAK },
    );
    let total = galley.size().x + 10.0;
    let x0 = rect.center().x - total / 2.0;
    let y = rect.bottom() - 8.5;
    p.galley(egui::pos2(x0, y - galley.size().y / 2.0), galley, TEXT);
    icons::paint(
        p,
        egui::Rect::from_center_size(egui::pos2(x0 + total - 3.0, y + 1.0), vec2(11.0, 11.0)),
        Icon::Chevron,
        true,
    );
    if !main.tip.is_empty() {
        resp.clone().on_hover_text(&main.tip);
    }
    if chev.clicked() {
        // handled by the popup below
    } else if resp.clicked() && main.enabled {
        picked = Some(main.id);
    }
    let section = [Section {
        heading: "",
        items: variants.to_vec(),
    }];
    if let Some(inner) = egui::Popup::menu(&chev).show(|ui| menu_rows(ui, &section)) {
        if let Some(id) = inner.inner {
            picked = Some(id);
        }
    }
    picked
}

/// Compact icon-only buttons in `columns` columns (the sketch constraints); the tooltip names each.
pub fn icon_grid(ui: &mut Ui, items: &[Item], columns: usize) -> Option<&'static str> {
    let mut picked = None;
    ui.vertical(|ui| {
        ui.spacing_mut().item_spacing = vec2(2.0, 2.0);
        for chunk in items.chunks(columns.max(1)) {
            ui.horizontal(|ui| {
                for it in chunk {
                    let (rect, resp) = ui.allocate_exact_size(
                        vec2(30.0, 30.0),
                        if it.enabled {
                            Sense::click()
                        } else {
                            Sense::hover()
                        },
                    );
                    let p = ui.painter();
                    if it.active {
                        p.rect(
                            rect.shrink(1.0),
                            2.0,
                            ACCENT_SOFT,
                            Stroke::new(1.0, ACCENT),
                            egui::StrokeKind::Inside,
                        );
                    } else if resp.hovered() && it.enabled {
                        p.rect_filled(rect.shrink(1.0), 2.0, ACCENT_SOFT.gamma_multiply(0.7));
                    }
                    icons::paint(p, rect.shrink(3.0), it.icon, it.enabled);
                    let tip = if it.tip.is_empty() {
                        it.label.to_string()
                    } else {
                        format!("{}\n{}", it.label, it.tip)
                    };
                    if resp.on_hover_text(tip).clicked() && it.enabled {
                        picked = Some(it.id);
                    }
                }
            });
        }
    });
    picked
}

/// A big button that opens a popup with arbitrary content (options such as the sketch snaps).
pub fn popup_button(
    ui: &mut Ui,
    icon: Icon,
    label: &str,
    tip: &str,
    content: impl FnOnce(&mut Ui),
) {
    let r = button(ui, icon, label, true, false, tip);
    egui::Popup::menu(&r)
        .close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside)
        .show(|ui| {
            ui.set_min_width(220.0);
            content(ui);
        });
}
