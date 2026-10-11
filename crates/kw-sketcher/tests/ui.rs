//! Drives the egui front end headlessly: synthetic pointer / keyboard events are fed through
//! `egui::Context::run_ui`, so the real panel, canvas and shortcut code paths execute.

use eframe::egui::{self, Event, Key, Modifiers, PointerButton, Pos2, RawInput, Rect};
use kw_sketch::editor::{Ray, Snap, Tool};
use kw_sketch::{EntityKind, SketchStatus};
use kw_sketcher::SketcherApp;

struct Harness {
    ctx: egui::Context,
    app: SketcherApp,
    t: f64,
}

impl Harness {
    fn new(app: SketcherApp) -> Self {
        let mut h = Self {
            ctx: egui::Context::default(),
            app,
            t: 0.0,
        };
        h.frame(vec![]);
        h.frame(vec![]);
        h
    }

    fn frame(&mut self, events: Vec<Event>) {
        self.frame_with(events, Modifiers::NONE);
    }

    fn frame_with(&mut self, mut events: Vec<Event>, modifiers: Modifiers) {
        // modifiers reach egui as an event, and persist until changed
        events.insert(0, Event::ModifiersChanged(modifiers));
        let raw = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1280.0, 800.0))),
            events,
            time: Some(self.t),
            ..Default::default()
        };
        self.t += 1.0 / 60.0;
        let app = &mut self.app;
        let mut out = self.ctx.run_ui(raw, |ui| app.show(ui));
        out.textures_delta.clear(); // no renderer consumes them
    }

    fn screen(&self, p: (f64, f64)) -> Pos2 {
        self.app.view.to_screen(self.app.canvas_rect(), p)
    }

    fn press(&mut self, at: Pos2, pressed: bool) {
        self.frame(vec![Event::PointerButton {
            pos: at,
            button: PointerButton::Primary,
            pressed,
            modifiers: Modifiers::NONE,
        }]);
    }

    fn click(&mut self, p: (f64, f64)) {
        let at = self.screen(p);
        self.frame(vec![Event::PointerMoved(at)]);
        self.press(at, true);
        self.press(at, false);
        self.frame(vec![]);
    }

    fn drag(&mut self, from: (f64, f64), to: (f64, f64)) {
        let (a, b) = (self.screen(from), self.screen(to));
        self.frame(vec![Event::PointerMoved(a)]);
        self.press(a, true);
        for i in 1..=8 {
            let t = i as f32 / 8.0;
            self.frame(vec![Event::PointerMoved(a + (b - a) * t)]);
        }
        self.press(b, false);
        self.frame(vec![]);
    }

    fn key(&mut self, key: Key, modifiers: Modifiers) {
        for pressed in [true, false] {
            self.frame_with(
                vec![Event::Key {
                    key,
                    physical_key: Some(key),
                    pressed,
                    repeat: false,
                    modifiers,
                }],
                modifiers,
            );
        }
    }
}

fn count(h: &Harness, f: impl Fn(&EntityKind) -> bool) -> usize {
    h.app
        .editor
        .sketch
        .entities
        .values()
        .filter(|e| f(&e.kind))
        .count()
}

#[test]
fn empty_and_demo_frames_render() {
    let mut h = Harness::new(SketcherApp::default());
    for _ in 0..5 {
        h.frame(vec![]);
    }
    assert!(h.app.frames >= 7);
    let mut app = SketcherApp::default();
    app.load_demo();
    let mut h = Harness::new(app);
    for _ in 0..5 {
        h.frame(vec![]);
    }
    assert!(h.app.editor.sketch.entities.len() > 5);
    assert!(h.app.editor.report.is_some());
    // demo is framed: all points land inside the canvas
    let (lo, hi) = h.app.editor.sketch.bounds().unwrap();
    let c = h.app.canvas_rect();
    assert!(c.contains(h.screen(lo)) && c.contains(h.screen(hi)));
}

#[test]
fn exit_after_frames_requests_close() {
    let mut app = SketcherApp::default();
    app.exit_after_frames = Some(3);
    let mut h = Harness::new(app);
    let mut closed = false;
    for _ in 0..4 {
        let raw = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1280.0, 800.0))),
            time: Some(h.t),
            ..Default::default()
        };
        h.t += 0.016;
        let app = &mut h.app;
        let mut out = h.ctx.run_ui(raw, |ui| app.show(ui));
        out.textures_delta.clear();
        closed |= out.viewport_output.values().any(|v| {
            v.commands
                .iter()
                .any(|c| matches!(c, egui::ViewportCommand::Close))
        });
    }
    assert!(closed, "the smoke-test exit never fired");
}

#[test]
fn clicking_with_the_line_tool_draws_a_chain() {
    let mut h = Harness::new(SketcherApp::default());
    h.key(Key::L, Modifiers::NONE);
    assert_eq!(h.app.editor.tool, Tool::Line);
    h.click((0.0, 0.0));
    h.click((10.0, 0.0));
    h.click((10.0, 6.0));
    h.key(Key::Escape, Modifiers::NONE);
    assert_eq!(count(&h, |k| matches!(k, EntityKind::Line { .. })), 2);
    assert_eq!(h.app.editor.tool, Tool::Select);
    // horizontal inference made the first line exactly horizontal
    let sk = &h.app.editor.sketch;
    let (_, l) = sk
        .entities
        .iter()
        .find(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .unwrap();
    let EntityKind::Line { p1, p2 } = l.kind else {
        unreachable!()
    };
    assert!((sk.point(p1).unwrap().1 - sk.point(p2).unwrap().1).abs() < 1e-9);
}

#[test]
fn rectangle_then_undo_redo_by_keyboard() {
    let mut h = Harness::new(SketcherApp::default());
    h.key(Key::R, Modifiers::NONE);
    h.click((-5.0, -3.0));
    h.click((7.0, 4.0));
    assert_eq!(count(&h, |k| matches!(k, EntityKind::Line { .. })), 4);
    h.key(Key::Z, Modifiers::COMMAND);
    assert_eq!(count(&h, |k| matches!(k, EntityKind::Line { .. })), 0);
    h.key(Key::Z, Modifiers::COMMAND | Modifiers::SHIFT);
    assert_eq!(count(&h, |k| matches!(k, EntityKind::Line { .. })), 4);
}

#[test]
fn dragging_a_free_corner_moves_it_and_keeps_the_rectangle_rectangular() {
    let mut h = Harness::new(SketcherApp::default());
    h.key(Key::R, Modifiers::NONE);
    h.click((0.0, 0.0));
    h.click((10.0, 6.0));
    h.key(Key::Escape, Modifiers::NONE);
    let report = h.app.editor.report.clone().unwrap();
    assert_eq!(report.status, SketchStatus::UnderConstrained);

    h.drag((10.0, 6.0), (14.0, 9.0));
    let sk = &h.app.editor.sketch;
    let corners: Vec<_> = sk
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
        .map(|(id, _)| sk.point(id).unwrap())
        .collect();
    let max = corners
        .iter()
        .fold((f64::MIN, f64::MIN), |m, p| (m.0.max(p.0), m.1.max(p.1)));
    assert!(
        (max.0 - 14.0).abs() < 1e-2 && (max.1 - 9.0).abs() < 1e-2,
        "dragged corner ended at {max:?}"
    );
    // still a rectangle: every line horizontal or vertical
    for (id, e) in &sk.entities {
        if let EntityKind::Line { p1, p2 } = e.kind {
            let (a, b) = (sk.point(p1).unwrap(), sk.point(p2).unwrap());
            assert!(
                (a.0 - b.0).abs() < 1e-6 || (a.1 - b.1).abs() < 1e-6,
                "{id:?} not axis aligned: {a:?} {b:?}"
            );
        }
    }
    // the drag is a single undo step
    h.key(Key::Z, Modifiers::COMMAND);
    let sk = &h.app.editor.sketch;
    let max = sk
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
        .map(|(id, _)| sk.point(id).unwrap().0)
        .fold(f64::MIN, f64::max);
    assert!((max - 10.0).abs() < 1e-6);
}

#[test]
fn dimension_tool_opens_the_value_editor_and_applies_it() {
    let mut h = Harness::new(SketcherApp::default());
    h.key(Key::L, Modifiers::NONE);
    h.click((0.0, 0.0));
    h.click((10.0, 0.0));
    h.key(Key::Escape, Modifiers::NONE);
    h.key(Key::D, Modifiers::NONE);
    h.click((5.0, 0.0)); // pick the line
    h.frame(vec![]);
    assert!(h
        .app
        .editor
        .sketch
        .constraints
        .values()
        .any(|c| matches!(c.kind, kw_sketch::ConstraintKind::Distance { .. })));
    // the same thing a user does in the dialog
    let id = h
        .app
        .editor
        .sketch
        .constraints
        .iter()
        .find(|(_, c)| matches!(c.kind, kw_sketch::ConstraintKind::Distance { .. }))
        .unwrap()
        .0;
    assert!(h.app.editor.edit_dimension(id, 25.0));
    h.frame(vec![]);
    let sk = &h.app.editor.sketch;
    let pts: Vec<_> = sk
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
        .map(|(i, _)| sk.point(i).unwrap())
        .collect();
    let d = (pts[0].0 - pts[1].0).hypot(pts[0].1 - pts[1].1);
    assert!((d - 25.0).abs() < 1e-6, "length is {d}");
}

#[test]
fn dxf_export_then_import_through_the_app() {
    let mut app = SketcherApp::default();
    app.load_demo();
    let n = app.editor.sketch.entities.len();
    let path = std::env::temp_dir().join(format!("kw_sketcher_ui_test_{}.dxf", std::process::id()));
    let p = path.to_str().unwrap().to_string();
    app.save_dxf(&p).unwrap();
    let mut fresh = SketcherApp::default();
    fresh.open_dxf(&p).unwrap();
    std::fs::remove_file(&path).ok();
    // points are welded, so the entity count matches up to the free points DXF cannot carry
    assert!(!fresh.editor.sketch.entities.is_empty());
    let lines = |a: &SketcherApp| {
        a.editor
            .sketch
            .entities
            .values()
            .filter(|e| matches!(e.kind, EntityKind::Line { .. }))
            .count()
    };
    assert_eq!(lines(&fresh), lines(&app));
    assert!(n > 0);
    assert!(fresh
        .open_dxf("/nonexistent/definitely/missing.dxf")
        .is_err());
}

impl Harness {
    fn click_with(&mut self, p: (f64, f64), modifiers: Modifiers) {
        let at = self.screen(p);
        self.frame_with(vec![Event::PointerMoved(at)], modifiers);
        for pressed in [true, false] {
            self.frame_with(
                vec![Event::PointerButton {
                    pos: at,
                    button: PointerButton::Primary,
                    pressed,
                    modifiers,
                }],
                modifiers,
            );
        }
        self.frame_with(vec![], modifiers);
    }

    fn hover(&mut self, p: (f64, f64), frames: usize) {
        let at = self.screen(p);
        self.frame(vec![Event::PointerMoved(at)]);
        for _ in 0..frames {
            self.frame(vec![]);
        }
    }
}

#[test]
fn resting_on_a_point_acquires_it_and_offers_an_alignment() {
    let mut h = Harness::new(SketcherApp::default());
    let a = h.app.editor.sketch.add_point(0.0, 10.0);
    h.app.editor.refresh();
    h.key(Key::L, Modifiers::NONE);
    // passing over the point is not enough ...
    h.hover((0.0, 10.0), 3);
    assert!(h.app.editor.acquired_points().is_empty());
    // ... resting on it is
    h.hover((0.0, 10.0), 30);
    assert_eq!(h.app.editor.acquired_points(), vec![(0.0, 10.0)]);
    let _ = a;
    // moving away keeps it; the cursor level with it now snaps onto the dashed line
    h.hover((14.0, 10.03), 2);
    let w = h.app.editor.cursor;
    let snap = h.app.editor.snap(w);
    assert!(
        matches!(snap, Snap::Track(q, Ray::Horizontal { anchor: false, .. }, None) if (q.1 - 10.0).abs() < 1e-9),
        "{snap:?}"
    );
    // clicking places the point exactly on that line; the memory is cleared
    h.click((14.0, 10.03));
    assert!(h.app.editor.sketch.entities.keys().any(|id| h
        .app
        .editor
        .sketch
        .point(id)
        .is_ok_and(|p| (p.1 - 10.0).abs() < 1e-12 && p.0 > 5.0)));
    assert!(h.app.editor.acquired_points().is_empty());
}

#[test]
fn alt_places_exactly_where_the_pointer_is() {
    let mut h = Harness::new(SketcherApp::default());
    h.key(Key::P, Modifiers::NONE);
    let target = (3.37, 2.21);
    h.click_with(target, Modifiers::ALT);
    h.click((13.37, 7.21));
    let pts: Vec<(f64, f64)> = h
        .app
        .editor
        .sketch
        .entities
        .keys()
        .filter_map(|id| h.app.editor.sketch.point(id).ok())
        .collect();
    assert_eq!(pts.len(), 2, "{pts:?}");
    let off_grid = |p: &(f64, f64)| {
        let step = h.app.view.grid_step(24.0);
        ((p.0 / step).round() * step - p.0).abs() > 1e-6
    };
    let raw: Vec<&(f64, f64)> = pts.iter().filter(|p| off_grid(p)).collect();
    let snapped: Vec<&(f64, f64)> = pts.iter().filter(|p| !off_grid(p)).collect();
    assert_eq!((raw.len(), snapped.len()), (1, 1), "{pts:?}");
    assert!((raw[0].0 - target.0).hypot(raw[0].1 - target.1) < 0.05);
}

#[test]
fn every_snap_marker_draws_and_the_snaps_palette_renders() {
    let mut app = SketcherApp::default();
    {
        let sk = &mut app.editor.sketch;
        sk.add_line_xy(2.0, 1.0, 10.0, 9.0);
        sk.add_line_xy(0.0, 12.0, 20.0, 2.0);
        sk.add_circle_xy(30.0, 10.0, 5.0);
        sk.add_point(-20.0, 5.0);
    }
    app.editor.refresh();
    let mut h = Harness::new(app);
    h.key(Key::L, Modifiers::NONE);
    let probes = [
        ((-0.05, 0.05), "Origin"),
        ((6.05, 5.05), "Midpoint"),
        ((8.7, 7.65), "Intersection"),
        ((-20.0, 5.05), "Point"),
        ((34.95, 10.05), "Point or curve"),
        ((17.3, -12.1), "grid or free"),
    ];
    for (p, what) in probes {
        h.app.demo_cursor = Some(p);
        h.frame(vec![]);
        h.frame(vec![]);
        let s = h.app.editor.snap(p);
        match what {
            "Origin" => assert!(matches!(s, Snap::Origin), "{s:?}"),
            "Midpoint" => assert!(matches!(s, Snap::Midpoint(..)), "{s:?}"),
            "Intersection" => assert!(matches!(s, Snap::Intersection(..)), "{s:?}"),
            "Point" => assert!(matches!(s, Snap::Point(..)), "{s:?}"),
            _ => {}
        }
    }
    // drawing a line from the origin: the 30 degree ray appears as a tracking snap
    h.click((0.0, 0.0));
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let probe = (15.0 * c, 15.0 * s + 0.1);
    h.app.demo_cursor = Some(probe);
    h.frame(vec![]);
    assert!(matches!(
        h.app.editor.snap(probe),
        Snap::Track(_, Ray::Angle { .. }, _)
    ));
    // the palette is part of the inspector
    h.app.demo_cursor = None;
    for _ in 0..3 {
        h.frame(vec![]);
    }
}
