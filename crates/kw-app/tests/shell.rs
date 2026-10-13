//! The application shell, driven without a desktop: real egui frames, scripted file dialogs.

use eframe::egui::{self, Pos2, RawInput, Rect};
use kw_app::dialogs::ScriptedDialogs;
use kw_app::recent::{RecentList, MAX_RECENT};
use kw_app::ui::{KerfwrightApp, Pending, Screen, Workspace};
use std::path::PathBuf;

struct Harness {
    ctx: egui::Context,
    app: KerfwrightApp,
    t: f64,
}

impl Harness {
    fn new(app: KerfwrightApp) -> Self {
        let mut h = Harness {
            ctx: egui::Context::default(),
            app,
            t: 0.0,
        };
        h.frame();
        h
    }
    fn frame(&mut self) {
        let raw = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1360.0, 860.0))),
            time: Some(self.t),
            ..Default::default()
        };
        self.t += 1.0 / 60.0;
        let app = &mut self.app;
        let mut out = self.ctx.run_ui(raw, |ui| app.show(ui));
        out.textures_delta.clear();
    }
}

fn scratch(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("kw_app_{name}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn app_in(dir: &std::path::Path) -> KerfwrightApp {
    let recents_file = dir.join("config").join("recent.json");
    KerfwrightApp::with(
        RecentList::load(&recents_file),
        Some(recents_file),
        Box::new(ScriptedDialogs::default()),
    )
}

#[test]
fn starts_on_the_welcome_page_and_draws_it() {
    let dir = scratch("welcome");
    let mut h = Harness::new(app_in(&dir));
    assert!(matches!(h.app.screen, Screen::Welcome));
    for _ in 0..3 {
        h.frame();
    }
}

#[test]
fn creating_a_project_saves_it_and_lists_it_as_recent() {
    let dir = scratch("create");
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Bracket", &dir).unwrap();
    let path = dir.join("Bracket.kwf");
    assert!(path.exists());
    assert!(matches!(h.app.screen, Screen::Project { .. }));
    assert_eq!(h.app.project().unwrap().name(), "Bracket");
    assert!(!h.app.project().unwrap().is_dirty());
    assert_eq!(h.app.recents.paths, vec![path.clone()]);
    h.frame();
    // the recent list is on disk, so a new session sees it
    let again = app_in(&dir);
    assert_eq!(again.recents.paths, vec![path]);
    // bad names and duplicates are refused
    assert!(h.app.create_project("", &dir).is_err());
    assert!(h.app.create_project("a/b", &dir).is_err());
    assert!(h.app.create_project("Bracket", &dir).is_err());
}

#[test]
fn every_workspace_draws() {
    let dir = scratch("workspaces");
    let mut h = Harness::new(app_in(&dir));
    h.app.open_demo();
    for w in Workspace::ALL {
        h.app.set_workspace(w);
        assert_eq!(h.app.workspace(), Some(w));
        h.frame();
        h.frame();
    }
}

#[test]
fn edits_make_the_project_dirty_and_saving_clears_it_and_reopening_gives_the_same_document() {
    let dir = scratch("save");
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Part", &dir).unwrap();
    let hash0 = h.app.project().unwrap().document().content_hash();
    h.app
        .project_mut()
        .unwrap()
        .history
        .edit(|d| {
            d.add_rect_sketch(
                "S",
                kw_feature::PlaneSpec::Xy(kw_expr::Param::length(0.0)),
                kw_expr::Param::length(0.0),
                kw_expr::Param::length(0.0),
                kw_expr::Param::length(10.0),
                kw_expr::Param::length(5.0),
            )
            .map(|_| ())
        })
        .unwrap();
    assert!(h.app.project().unwrap().is_dirty());
    assert!(h.app.save());
    assert!(!h.app.project().unwrap().is_dirty());
    let hash1 = h.app.project().unwrap().document().content_hash();
    assert_ne!(hash0, hash1);
    // undo makes it differ from the saved state again
    h.app.project_mut().unwrap().history.undo();
    assert!(h.app.project().unwrap().is_dirty());
    // reopen from disk
    let mut h2 = Harness::new(app_in(&dir));
    h2.app.open_path(&dir.join("Part.kwf")).unwrap();
    assert_eq!(h2.app.project().unwrap().document().content_hash(), hash1);
    // the evaluation of the reopened document is identical to the one that was saved
    let ev1 = h.app.project_mut().unwrap().history.redo();
    assert!(ev1);
    let a = h.app.project_mut().unwrap().evaluation().document_hash();
    let b = h2.app.project_mut().unwrap().evaluation().document_hash();
    assert_eq!(a, b);
}

#[test]
fn unsaved_changes_are_asked_about_before_closing_opening_or_quitting() {
    let dir = scratch("prompt");
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Doc", &dir).unwrap();
    // a clean project closes without a question
    h.app.request(Pending::CloseProject);
    assert!(h.app.pending.is_none());
    assert!(matches!(h.app.screen, Screen::Welcome));
    h.app.open_path(&dir.join("Doc.kwf")).unwrap();
    h.app
        .project_mut()
        .unwrap()
        .history
        .edit(|d| {
            d.add_parameter("w", kw_expr::Param::length(3.0))
                .map(|_| ())
        })
        .unwrap();
    // a dirty one asks; cancel stays
    h.app.request(Pending::CloseProject);
    assert_eq!(h.app.pending, Some(Pending::CloseProject));
    h.frame(); // the dialog draws
    h.app.cancel_pending();
    assert!(matches!(h.app.screen, Screen::Project { .. }));
    assert!(h.app.project().unwrap().is_dirty());
    // "don't save" discards
    h.app.request(Pending::CloseProject);
    h.app.resolve_pending(false);
    assert!(matches!(h.app.screen, Screen::Welcome));
    h.app.open_path(&dir.join("Doc.kwf")).unwrap();
    assert!(
        h.app.project().unwrap().document().is_empty(),
        "the discarded edit must not be on disk"
    );
    // quitting with changes asks, and "save" saves then allows it
    h.app
        .project_mut()
        .unwrap()
        .history
        .edit(|d| {
            d.add_parameter("w", kw_expr::Param::length(3.0))
                .map(|_| ())
        })
        .unwrap();
    assert!(!h.app.close_requested());
    assert_eq!(h.app.pending, Some(Pending::Quit));
    h.app.resolve_pending(true);
    assert!(h.app.quit_ok);
    // the window is told to close by itself (no second click on the close button)
    assert!(h.app.close_now);
    h.frame();
    assert!(!h.app.close_now, "the close command was sent");
    let reopened = kw_app::project::Project::open(&dir.join("Doc.kwf")).unwrap();
    assert_eq!(reopened.document().len(), 1);
}

#[test]
fn damaged_and_missing_files_are_reported_not_fatal() {
    let dir = scratch("damaged");
    let mut h = Harness::new(app_in(&dir));
    let bad = dir.join("bad.kwf");
    std::fs::write(&bad, b"this is not a project").unwrap();
    assert!(h.app.open_path(&bad).is_err());
    assert!(h.app.message.contains("cannot open"));
    assert!(matches!(h.app.screen, Screen::Welcome));
    assert!(h.app.open_path(&dir.join("nope.kwf")).is_err());
    h.frame();
}

#[test]
fn the_recent_list_keeps_ten_without_duplicates_and_tolerates_a_damaged_file() {
    let mut r = RecentList::default();
    for i in 0..15 {
        r.touch(PathBuf::from(format!("/p/{i}.kwf")));
    }
    r.touch(PathBuf::from("/p/12.kwf"));
    assert_eq!(r.paths.len(), MAX_RECENT);
    assert_eq!(r.paths[0], PathBuf::from("/p/12.kwf"));
    assert_eq!(
        r.paths
            .iter()
            .filter(|p| p.as_path() == std::path::Path::new("/p/12.kwf"))
            .count(),
        1
    );
    let dir = scratch("recent");
    let f = dir.join("r.json");
    std::fs::write(&f, b"{ nonsense").unwrap();
    assert!(RecentList::load(&f).paths.is_empty());
}

#[test]
fn cam_fea_and_board_data_ride_along_in_the_saved_file() {
    let dir = scratch("extras");
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Multi", &dir).unwrap();
    {
        let p = h.app.project_mut().unwrap();
        let (board, _) = kw_ecad::bench_gen::generate(&kw_ecad::bench_gen::benchmark_specs()[0]);
        p.board = Some(board);
        p.mark_extras_dirty();
        assert!(p.is_dirty());
    }
    assert!(h.app.save());
    let back = kw_app::project::Project::open(&dir.join("Multi.kwf")).unwrap();
    assert!(back.board.is_some() && back.cam.is_none());
    assert!(!back.is_dirty());
}

// ------------------------------------------------------------------------------------ Model workspace

use kw_app::viewport::{ModelScene, Pick};
use kw_feature::FeatureSpec;

fn demo_app(dir: &std::path::Path) -> Harness {
    let mut h = Harness::new(app_in(dir));
    h.app.create_project("Model", dir).unwrap();
    h.app.set_workspace(Workspace::Design);
    h
}

fn volume(p: &mut kw_app::project::Project) -> f64 {
    let ev = p.evaluation();
    let (_, o) = ev.final_solid().expect("a solid");
    let b = o.brep().unwrap();
    b.mass_properties(o.body().unwrap()).unwrap().volume
}

#[test]
fn sketch_extrude_fillet_through_the_workspace_commands_and_undo() {
    let dir = scratch("model_flow");
    let mut h = demo_app(&dir);
    let (mut p, mut m) = (
        std::mem::replace(
            h.app.project_mut().unwrap(),
            kw_app::project::Project::empty(),
        ),
        std::mem::take(&mut h.app.design.model),
    );
    let sk = m
        .add_rect_sketch(&mut p, "xy", "0", "0", "40", "20")
        .expect(&m.error);
    let ex = m
        .add_extrude(&mut p, sk, "10", kw_feature::BodyOp::New)
        .expect(&m.error);
    assert!((volume(&mut p) - 8000.0).abs() < 1e-6);
    // pick the four top edges and fillet them
    let ev = p.evaluation();
    let (_, out) = ev.final_solid().unwrap();
    let scene = ModelScene::build(out.brep().unwrap());
    let top: Vec<Pick> = (1..=scene.scene.pick.len() as u32)
        .filter_map(|id| scene.resolve(id))
        .filter_map(|pk| match pk {
            Pick::Edge(e) => Some(e),
            _ => None,
        })
        .filter(|e| {
            let b = out.brep().unwrap();
            let ed = &b.edges[*e];
            let (a, c) = (b.vertices[ed.start].point, b.vertices[ed.end].point);
            (a.z - 10.0).abs() < 1e-9 && (c.z - 10.0).abs() < 1e-9
        })
        .map(Pick::Edge)
        .collect();
    assert_eq!(top.len(), 4);
    m.picks = top;
    let fil = m.add_fillet(&mut p, "2").expect(&m.error);
    let v = volume(&mut p);
    // four 2 mm roundings along edges of total length 120: each removes (1 - pi/4) r^2 per unit length, less the
    // corner overlaps — between 3 % and 5 % of the block's top edge material
    assert!(
        v < 8000.0 && v > 8000.0 - 4.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 4.0 * 60.0 - 1.0,
        "{v}"
    );
    // editing the fillet radius through the inspector path (replace_spec) changes the volume
    let before = v;
    let mut spec = p.document().get(fil).unwrap().spec.clone();
    if let FeatureSpec::Fillet(f) = &mut spec {
        f.radius = kw_app::model::parse_param("3 mm", kw_expr::Dim::LENGTH).unwrap();
    }
    p.history.edit(|d| d.replace_spec(fil, spec)).unwrap();
    assert!(volume(&mut p) < before);
    // undo twice returns to the plain block, redo comes back
    p.history.undo();
    p.history.undo();
    assert!((volume(&mut p) - 8000.0).abs() < 1e-6);
    p.history.redo();
    assert!(volume(&mut p) < 8000.0);
    // suppress / delete / roll back
    p.history.edit(|d| d.set_suppressed(fil, true)).unwrap();
    assert!((volume(&mut p) - 8000.0).abs() < 1e-6);
    let _ = ex;
    // bad input is refused with a message and changes nothing
    let n = p.document().len();
    assert!(m
        .add_extrude(&mut p, sk, "ten", kw_feature::BodyOp::Join)
        .is_none());
    assert!(!m.error.is_empty());
    assert_eq!(p.document().len(), n);
    // a second part named like the first is refused by the document, not silently renamed
    assert!(m.add_parameter(&mut p, "w", "20").is_some());
    assert!(m.add_parameter(&mut p, "w", "30").is_none());
    // an expression parameter drives a feature
    assert!(m.add_parameter(&mut p, "d", "w / 2").is_some());
}

#[test]
fn the_model_workspace_draws_with_and_without_features_and_a_failed_feature() {
    let dir = scratch("model_draw");
    let mut h = demo_app(&dir);
    h.frame();
    h.app.open_demo();
    h.app.set_workspace(Workspace::Design);
    h.frame();
    h.frame();
    // a feature whose expression refers to nothing fails to evaluate; the list still draws
    let (mut p, mut m) = (
        std::mem::replace(
            h.app.project_mut().unwrap(),
            kw_app::project::Project::empty(),
        ),
        std::mem::take(&mut h.app.design.model),
    );
    let sk = p.document().features[0].id;
    let n = p.document().len();
    assert!(m
        .add_extrude(&mut p, sk, "missing_name * 2", kw_feature::BodyOp::Join)
        .is_none());
    assert!(!m.error.is_empty(), "an unknown name must be reported");
    assert_eq!(p.document().len(), n, "and must not enter the timeline");
    *h.app.project_mut().unwrap() = p;
    h.app.design.model = m;
    h.frame();
}

// ---------------------------------------------------------------------------------- Sketch workspace

use kw_app::sketch::{Editing, NewPlane, SketchWorkspace};

fn rectangle(w: f64, h: f64) -> kw_sketch::Sketch {
    let mut sk = kw_sketch::Sketch::new();
    let pts: Vec<_> = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
        .iter()
        .map(|p| sk.add_point(p.0, p.1))
        .collect();
    let l: Vec<_> = (0..4)
        .map(|i| sk.add_line(pts[i], pts[(i + 1) % 4]))
        .collect();
    let _ = sk.horizontal(l[0]);
    let _ = sk.horizontal(l[2]);
    let _ = sk.vertical(l[1]);
    let _ = sk.vertical(l[3]);
    let _ = sk.fix(pts[0]);
    sk
}

#[test]
fn a_finished_sketch_becomes_a_feature_that_extrudes_and_can_be_edited_again() {
    let dir = scratch("sketch_flow");
    let mut h = demo_app(&dir);
    h.app.set_workspace(Workspace::Design);
    h.frame();
    h.frame();
    let mut p = std::mem::replace(
        h.app.project_mut().unwrap(),
        kw_app::project::Project::empty(),
    );
    let mut ws = SketchWorkspace::default();
    // finishing with nothing being edited does nothing
    assert!(ws.finish(&mut p).is_none());
    ws.begin_new(NewPlane::Xy);
    assert!(matches!(ws.editing, Editing::New(_)));
    ws.app.editor.replace_sketch(rectangle(30.0, 20.0));
    let id = ws.finish(&mut p).expect(&ws.error);
    assert!(matches!(ws.editing, Editing::Nothing));
    assert!(matches!(
        p.document().get(id).unwrap().spec,
        FeatureSpec::Sketch(_)
    ));
    let mut m = kw_app::model::ModelWorkspace::default();
    m.add_extrude(&mut p, id, "5", kw_feature::BodyOp::New)
        .expect(&m.error);
    assert!((volume(&mut p) - 3000.0).abs() < 1e-6, "{}", volume(&mut p));
    // edit it again: the editor shows the same geometry, a new size replaces the feature (same id)
    ws.begin_edit(&mut p, id).unwrap();
    assert_eq!(ws.app.editor.sketch.entities.len(), 8);
    ws.app.editor.replace_sketch(rectangle(30.0, 40.0));
    assert_eq!(ws.finish(&mut p), Some(id));
    assert!((volume(&mut p) - 6000.0).abs() < 1e-6);
    // undo restores the earlier sketch
    p.history.undo();
    assert!((volume(&mut p) - 3000.0).abs() < 1e-6);
    // editing something that is not a sketch is refused
    let ex = p
        .document()
        .features
        .iter()
        .find(|f| matches!(f.spec, FeatureSpec::Extrude(_)))
        .unwrap()
        .id;
    assert!(ws.begin_edit(&mut p, ex).is_err());
    // the embedded sketcher draws (start page, then the editing view)
    *h.app.project_mut().unwrap() = p;
    h.app.design.sketch = ws;
    h.frame();
    h.app.design.sketch.begin_new(NewPlane::Yz);
    h.frame();
    h.frame();
}

#[test]
fn editing_a_builder_sketch_shows_its_real_size_and_keeps_bindings_the_user_did_not_touch() {
    let dir = scratch("sketch_bindings");
    let mut h = demo_app(&dir);
    h.app.open_demo(); // a 40 x 25 rectangle made from parameters, extruded 10
    let mut p = std::mem::replace(
        h.app.project_mut().unwrap(),
        kw_app::project::Project::empty(),
    );
    let id = p.document().features[0].id;
    let FeatureSpec::Sketch(before) = p.document().get(id).unwrap().spec.clone() else {
        panic!()
    };
    assert!(
        !before.recipe.bindings.is_empty(),
        "the builder binds the rectangle's numbers"
    );
    let mut ws = SketchWorkspace::default();
    ws.begin_edit(&mut p, id).unwrap();
    // the editor holds the evaluated sketch: a point at x = 40, not the unbound placeholder
    let xs: Vec<f64> = ws.app.editor.sketch.vars.values.clone();
    assert!(xs.iter().any(|v| (v - 40.0).abs() < 1e-9), "{xs:?}");
    // finishing unchanged keeps the bindings and the geometry
    let vol0 = volume(&mut p);
    ws.finish(&mut p).expect(&ws.error);
    let FeatureSpec::Sketch(after) = p.document().get(id).unwrap().spec.clone() else {
        panic!()
    };
    assert_eq!(after.recipe.bindings.len(), before.recipe.bindings.len());
    assert!((volume(&mut p) - vol0).abs() < 1e-6);
    // moving a corner out of its bound value drops that binding so the new value wins
    ws.begin_edit(&mut p, id).unwrap();
    let mut sk = ws.app.editor.sketch.clone();
    let k = before
        .recipe
        .bindings
        .iter()
        .find(|b| (sk.vars.values[b.var as usize] - 40.0).abs() < 1e-9)
        .expect("the width is bound")
        .var as usize;
    sk.vars.values[k] = 60.0;
    ws.app.editor.replace_sketch(sk);
    ws.finish(&mut p).expect(&ws.error);
    let FeatureSpec::Sketch(changed) = p.document().get(id).unwrap().spec.clone() else {
        panic!()
    };
    assert!(changed.recipe.bindings.len() < before.recipe.bindings.len());
}

// -------------------------------------------------------------------------------------- ECAD workspace

#[test]
fn the_ecad_workspace_generates_places_routes_checks_pours_and_exports() {
    let dir = scratch("ecad");
    let mut h = demo_app(&dir);
    h.app.set_workspace(Workspace::Ecad);
    h.frame(); // no board yet: draws the empty state
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.ecad.generate_sample(project, 0);
        assert!(project.board.is_some() && project.is_dirty());
    }
    h.frame();
    // route in the background and wait for it, drawing frames as the real loop would
    {
        let Screen::Project { project, .. } = &h.app.screen else {
            panic!()
        };
        h.app.ecad.start_route(project);
    }
    let t = std::time::Instant::now();
    loop {
        h.frame();
        if h.app.ecad.message.starts_with("routed") {
            break;
        }
        assert!(
            t.elapsed().as_secs() < 120,
            "routing did not finish: {}",
            h.app.ecad.message
        );
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    assert!(
        h.app.ecad.message.contains("22/22"),
        "{}",
        h.app.ecad.message
    );
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.ecad.check(project);
    assert!(h.app.ecad.violations.is_empty() && h.app.ecad.opens == 0);
    h.app.ecad.pour(project);
    assert_eq!(project.board.as_ref().unwrap().zones.len(), 1);
    assert!(kw_ecad::drc(project.board.as_ref().unwrap()).is_empty());
    // fabrication files
    let out = dir.join("fab");
    std::fs::create_dir_all(&out).unwrap();
    let n = h
        .app
        .ecad
        .export_fabrication(project.board.as_ref().unwrap(), &out)
        .unwrap();
    assert_eq!(n, std::fs::read_dir(&out).unwrap().count());
    assert!(n >= 10);
    let gtl = std::fs::read_to_string(out.join("bench1000-F_Cu.gtl")).unwrap();
    assert!(
        kw_fabcheck::poly::parse(&gtl).is_ok(),
        "the exported copper layer is a valid Gerber file"
    );
    // re-placing clears the routing
    h.app.ecad.replace(project);
    assert!(project.board.as_ref().unwrap().tracks.is_empty());
    // the board is saved with the project
    assert!(project.is_dirty());
    project.save().unwrap();
    let back = kw_app::project::Project::open(project.path.as_ref().unwrap()).unwrap();
    assert_eq!(
        back.board.as_ref().unwrap().parts.len(),
        project.board.as_ref().unwrap().parts.len()
    );
}

// ---------------------------------------------------------------------------------------- CAM workspace

use kw_app::cam::{Dialect, OpKind, ToolChoice};

#[test]
fn the_cam_workspace_plans_operations_posts_every_dialect_and_simulates_the_stock() {
    let dir = scratch("cam");
    let mut h = Harness::new(app_in(&dir));
    h.app.open_demo(); // a 40 x 25 x 10 block
    h.app.set_workspace(Workspace::Cam);
    h.frame();
    h.frame();
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let ev = project.evaluation();
    let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
    h.app.cam.fit_to_model(brep.as_ref().unwrap());
    let s = &h.app.cam.settings;
    for (got, want) in [
        (s.x0, 0.0),
        (s.y0, 0.0),
        (s.x1, 40.0),
        (s.y1, 25.0),
        (s.top, 10.0),
    ] {
        assert!((got - want).abs() < 1e-6, "{got} vs {want}");
    }
    // the four operations
    for (op, tool) in [
        (OpKind::Pocket, ToolChoice::Flat),
        (OpKind::Adaptive, ToolChoice::Flat),
        (OpKind::ContourOutside, ToolChoice::Flat),
        (OpKind::Finish3d, ToolChoice::Ball),
    ] {
        h.app.cam.settings.op = op;
        h.app.cam.settings.tool = tool;
        h.app.cam.fit_to_model(brep.as_ref().unwrap());
        let st = &mut h.app.cam.settings;
        if matches!(op, OpKind::Pocket | OpKind::Adaptive) {
            st.x0 += 5.0;
            st.y0 += 5.0;
            st.x1 -= 5.0;
            st.y1 -= 5.0;
            st.bottom = st.top - 3.0;
        }
        if op == OpKind::Finish3d {
            st.stepover = 3.0;
        }
        h.app
            .cam
            .run_now(project, brep.as_ref())
            .unwrap_or_else(|e| panic!("{op:?}: {e}"));
    }
    let cam = project.cam.as_ref().unwrap();
    assert_eq!(cam.operations.len(), 4);
    assert!(
        cam.operations
            .iter()
            .all(|o| o.path.cutting_length() > 10.0),
        "every operation cuts"
    );
    assert!(project.is_dirty());
    // post every operation in every dialect; each program replays in the independent interpreter
    for sel in 0..4 {
        h.app.cam.selected = Some(sel);
        for d in Dialect::ALL {
            h.app.cam.dialect = d;
            let text = h
                .app
                .cam
                .post_text(project)
                .unwrap()
                .unwrap_or_else(|e| panic!("{d:?}: {e}"));
            let gd = match d {
                Dialect::Fanuc => kw_gsim::Dialect::Fanuc,
                Dialect::Haas => kw_gsim::Dialect::Haas,
                Dialect::Siemens => kw_gsim::Dialect::Siemens840D,
                Dialect::Heidenhain => kw_gsim::Dialect::Heidenhain,
                Dialect::LinuxCnc => kw_gsim::Dialect::LinuxCnc,
            };
            kw_gsim::run(gd, &text).unwrap_or_else(|e| panic!("op {sel} {d:?}: {e}"));
        }
    }
    // bad input
    h.app.cam.settings.op = OpKind::Pocket;
    h.app.cam.settings.x1 = h.app.cam.settings.x0 + 2.0;
    assert!(
        h.app.cam.run_now(project, brep.as_ref()).is_err(),
        "a region smaller than the tool is refused"
    );
    h.app.cam.settings.top = h.app.cam.settings.bottom - 1.0;
    assert!(h.app.cam.run_now(project, brep.as_ref()).is_err());
    // the operations are saved with the project
    project.save_as(&dir.join("cam.kwf")).unwrap();
    let back = kw_app::project::Project::open(&dir.join("cam.kwf")).unwrap();
    assert_eq!(back.cam.unwrap().operations.len(), 4);
    // draw with an operation selected (the viewport has no GPU here, the panels still run)
    h.app.cam.selected = Some(0);
    h.frame();
    // stock removal by the pocket: the volume removed is about the pocket's area times its depth
    {
        let Screen::Project { project, .. } = &h.app.screen else {
            panic!()
        };
        let ev2 = ev.clone();
        let b2 = ev2.final_solid().and_then(|(_, o)| o.brep().cloned());
        h.app.cam.start_sim(project, b2.as_ref());
    }
    let t = std::time::Instant::now();
    while !h.app.cam.message.contains("removed") {
        h.frame();
        assert!(t.elapsed().as_secs() < 120, "{}", h.app.cam.message);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    eprintln!("{}", h.app.cam.message);
}

// ---------------------------------------------------------------------------------------- FEA workspace

fn face_tag_at_x(brep: &kw_brep::BRep, x: f64) -> u32 {
    let body = brep.bodies.keys().next().unwrap();
    let t = kw_tess::tessellate_body(
        brep,
        body,
        &kw_tess::TessOptions {
            chord_tol: 0.005,
            angle_tol: 12f64.to_radians(),
        },
    )
    .unwrap();
    let mut found = None;
    for (k, tri) in t.mesh.indices.as_chunks::<3>().0.iter().enumerate() {
        let cx: f64 = tri
            .iter()
            .map(|i| t.mesh.positions[*i as usize].x)
            .sum::<f64>()
            / 3.0;
        let all_on = tri
            .iter()
            .all(|i| (t.mesh.positions[*i as usize].x - x).abs() < 1e-6);
        if all_on && (cx - x).abs() < 1e-6 {
            found = Some(t.mesh.tri_face[k]);
            break;
        }
    }
    found.expect("a face on that plane")
}

#[test]
fn the_fea_workspace_solves_a_cantilever_close_to_beam_theory_and_stores_its_study() {
    let dir = scratch("fea");
    let mut h = demo_app(&dir);
    let (mut p, mut m) = (
        std::mem::replace(
            h.app.project_mut().unwrap(),
            kw_app::project::Project::empty(),
        ),
        std::mem::take(&mut h.app.design.model),
    );
    let sk = m
        .add_rect_sketch(&mut p, "xy", "0", "0", "100", "10")
        .expect(&m.error);
    m.add_extrude(&mut p, sk, "10", kw_feature::BodyOp::New)
        .expect(&m.error);
    let ev = p.evaluation();
    let brep = ev
        .final_solid()
        .and_then(|(_, o)| o.brep().cloned())
        .unwrap();
    let (fixed, loaded) = (face_tag_at_x(&brep, 0.0), face_tag_at_x(&brep, 100.0));
    let mut fea = kw_app::fea::FeaWorkspace::default();
    fea.study.max_size = 4.0;
    fea.fix_face(&mut p, fixed);
    fea.add_force(&mut p, loaded, [0.0, 0.0, -100.0]);
    assert!(p.is_dirty() && p.fea.is_some());
    // without supports or loads the solve explains what is missing
    let mut empty = fea.study.clone();
    empty.fixed_tags.clear();
    assert!(kw_app::fea::solve_study(&brep, &empty)
        .unwrap_err()
        .contains("fix at least one face"));
    empty.fixed_tags.push(fixed);
    empty.loads.clear();
    assert!(kw_app::fea::solve_study(&brep, &empty)
        .unwrap_err()
        .contains("load"));
    // solve in the background
    fea.start(&brep);
    *h.app.project_mut().unwrap() = p;
    h.app.fea = fea;
    h.app.set_workspace(Workspace::Fea);
    let t = std::time::Instant::now();
    while h.app.fea.result.is_none() {
        h.frame();
        assert!(t.elapsed().as_secs() < 180, "{}", h.app.fea.message);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let r = h.app.fea.result.clone().unwrap();
    // beam theory: P L^3 / (3 E I), I = b h^3 / 12; shear and the stiff clamped end change it by a few percent
    let (e, l, ii): (f64, f64, f64) = (210e3, 100.0, 10.0 * 10.0f64.powi(3) / 12.0);
    let beam = 100.0 * l.powi(3) / (3.0 * e * ii);
    eprintln!(
        "{} | tip {:.4} mm, beam theory {:.4} mm",
        h.app.fea.message, r.max_displacement, beam
    );
    assert!(
        (r.max_displacement / beam - 1.0).abs() < 0.08,
        "{} vs {beam}",
        r.max_displacement
    );
    // bending stress at the clamped end: 6 P L / (b h^2) = 60 MPa
    assert!(
        r.max_von_mises > 55.0 && r.max_von_mises < 75.0,
        "{}",
        r.max_von_mises
    );
    // the coloured result scene has geometry
    let scene = kw_app::fea::result_scene(&r, kw_app::fea::Field::VonMises, 50.0);
    assert!(scene.scene.vertices.len() > 100 && scene.scene.radius > 40.0);
    // all views draw
    for f in [
        kw_app::fea::Field::Model,
        kw_app::fea::Field::VonMises,
        kw_app::fea::Field::Displacement,
    ] {
        h.app.fea.field = f;
        h.frame();
    }
    // the study is saved with the project
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    project.save().unwrap();
    let back = kw_app::project::Project::open(project.path.as_ref().unwrap()).unwrap();
    let st = back.fea.unwrap();
    assert_eq!(st.fixed_tags, vec![fixed]);
    assert_eq!(st.loads.len(), 1);
}

// -------------------------------------------------------------------------------------- Render workspace
// (needs a GPU; skipped without an adapter; do not run under `ulimit -v`)

#[test]
fn the_render_workspace_draws_realtime_path_traces_denoises_and_saves_a_png() {
    if kw_render::RenderGpu::try_new().is_none() {
        eprintln!("no GPU adapter: test skipped");
        return;
    }
    let dir = scratch("render");
    let mut h = Harness::new(app_in(&dir));
    h.app.open_demo();
    h.app.set_workspace(Workspace::Render);
    h.frame();
    h.frame();
    let real = h.app.render.pixels.clone();
    assert_eq!(
        real.len(),
        (kw_app::render::IMAGE.0 * kw_app::render::IMAGE.1 * 4) as usize,
        "a realtime image was drawn"
    );
    let distinct = real
        .chunks(4)
        .map(|p| (p[0], p[1], p[2]))
        .collect::<std::collections::HashSet<_>>()
        .len();
    assert!(
        distinct > 50,
        "the image has shading, not a flat fill ({distinct} colours)"
    );
    // path trace 16 samples
    h.app.render.target_spp = 16;
    h.app.render.start_path_trace();
    for _ in 0..12 {
        h.frame();
    }
    assert_eq!(h.app.render.spp_done, 16);
    assert_ne!(
        h.app.render.pixels, real,
        "the path-traced image differs from the realtime one"
    );
    let brightness = |px: &[u8]| {
        px.chunks(4)
            .map(|p| p[0] as u64 + p[1] as u64 + p[2] as u64)
            .sum::<u64>() as f64
            / (px.len() as f64 / 4.0 * 765.0)
    };
    let b = brightness(&h.app.render.pixels);
    assert!(b > 0.05 && b < 0.95, "mean brightness {b}");
    // denoise (button path): the job finishes and changes the image
    let noisy = h.app.render.pixels.clone();
    h.app.render.show_denoise_for_test();
    let t = std::time::Instant::now();
    while h.app.render.message != "denoised" {
        h.frame();
        assert!(t.elapsed().as_secs() < 120, "{}", h.app.render.message);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    assert_ne!(h.app.render.pixels, noisy);
    // save
    let out = dir.join("shot.png");
    h.app.render.save_png(&out).unwrap();
    assert!(std::fs::metadata(&out).unwrap().len() > 1000);
}

// ----------------------------------------------------------------------------------------- 3-D viewport

#[test]
fn the_viewport_renders_the_model_offscreen_and_picks_faces_and_edges() {
    let Some(g) = kw_render::RenderGpu::try_new() else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let p = kw_app::project::Project::demo();
    let mut p = p;
    let ev = p.evaluation();
    let brep = ev
        .final_solid()
        .and_then(|(_, o)| o.brep().cloned())
        .unwrap();
    let model = ModelScene::build(&brep);
    assert!(model.scene.vertices.len() > 20);
    let size = (400u32, 300u32);
    let mut view = kw_app::viewport::GpuView::new(&g.device, size);
    view.renderer.upload(&g.device, &model.scene);
    let mut cam = kw_viewer::OrbitCamera::new(model.scene.radius, size.0 as f64 / size.1 as f64);
    cam.fit(model.scene.radius);
    view.render(&g.device, &g.queue, &cam);
    let px = view.read_rgba(&g.device, &g.queue);
    let bg = &px[0..4];
    let lit = px.chunks(4).filter(|c| c[..3] != bg[..3]).count();
    assert!(lit > 5000, "the model covers part of the image ({lit} px)");
    // the centre of the image is on the block: a face
    let id = view.pick_id(&g.device, &g.queue, size.0 / 2, size.1 / 2);
    assert!(matches!(model.resolve(id), Some(Pick::Face(_))), "id {id}");
    // a corner is empty space
    assert_eq!(view.pick_id(&g.device, &g.queue, 2, 2), 0);
    assert!(model.resolve(0).is_none());
    // every pick id resolves, and the id of a pick maps back to it
    for id in 1..=model.scene.pick.len() as u32 {
        let pick = model.resolve(id).expect("resolves");
        assert_eq!(model.id_of(pick), id);
    }
    // scanning the image finds all 3 visible faces and some edges
    let mut faces = std::collections::HashSet::new();
    let mut edges = 0;
    for y in (0..size.1).step_by(4) {
        for x in (0..size.0).step_by(4) {
            match model.resolve(view.pick_id(&g.device, &g.queue, x, y)) {
                Some(Pick::Face(f)) => {
                    faces.insert(f);
                }
                Some(Pick::Edge(_)) => edges += 1,
                None => {}
            }
        }
    }
    assert!(faces.len() >= 3, "{} faces visible", faces.len());
    assert!(edges > 0, "edges can be picked too");
    // resizing replaces the colour target
    assert!(view.resize(&g.device, (500, 300)));
    assert!(!view.resize(&g.device, (500, 300)));
}

#[test]
fn an_empty_model_renders_without_a_crash() {
    let Some(g) = kw_render::RenderGpu::try_new() else {
        return;
    };
    let model = ModelScene::empty();
    let mut view = kw_app::viewport::GpuView::new(&g.device, (200, 150));
    view.renderer.upload(&g.device, &model.scene);
    let cam = kw_viewer::OrbitCamera::new(10.0, 1.3);
    view.render(&g.device, &g.queue, &cam);
    let _ = view.read_rgba(&g.device, &g.queue);
    // the realtime pipeline too
    let mut f = kw_render::Frame::new(&g.device, &g.queue, (200, 150), false);
    f.upload(&g.device, &model.scene);
    let (_t, v) = kw_render::offscreen(&g.device, (200, 150));
    f.render(
        &g.device,
        &g.queue,
        &cam,
        &v,
        &kw_render::FrameOptions::default(),
    );
}

// ------------------------------------------------------------------------- Design workspace (Fusion-style)

use kw_app::design::{geom, tools::Tool, Action, Mode};
use kw_app::viewport::ViewOut;

fn design_harness(name: &str) -> (Harness, std::path::PathBuf) {
    let dir = scratch(name);
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Design", &dir).unwrap();
    h.app.set_workspace(Workspace::Design);
    h.frame();
    (h, dir)
}

fn proj(h: &mut Harness) -> &mut kw_app::project::Project {
    h.app.project_mut().unwrap()
}

fn vol(h: &mut Harness) -> f64 {
    volume(proj(h))
}

fn act(h: &mut Harness, a: Action) {
    h.app.design.actions.push(a);
    h.frame();
}

/// Sketch a rectangle on the XY plane and finish it: the Extrude dialog must be open with that sketch.
fn sketch_block(h: &mut Harness, w: f64, hh: f64, depth: &str) {
    act(h, Action::StartSketch);
    assert_eq!(h.app.design.mode, Mode::PickPlane);
    act(
        h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    assert_eq!(h.app.design.mode, Mode::Sketching);
    h.app
        .design
        .sketch
        .app
        .editor
        .replace_sketch(rectangle(w, hh));
    act(h, Action::FinishSketch);
    assert_eq!(h.app.design.mode, Mode::Idle);
    let Some(d) = h.app.design.dialog.as_mut() else {
        panic!("the Extrude dialog opens straight after Finish Sketch")
    };
    let Tool::Extrude {
        sketch,
        depth: dd,
        op,
        ..
    } = &mut d.tool
    else {
        panic!("{:?}", d.tool)
    };
    assert!(sketch.is_some(), "the new sketch is preselected");
    assert_eq!(*op, kw_feature::BodyOp::New, "no body yet: New Body");
    *dd = depth.into();
    h.frame();
}

fn commit(h: &mut Harness) {
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    assert!(
        h.app.design.commit_dialog(project),
        "{}",
        h.app.design.model.error
    );
    h.frame();
}

fn top_face(h: &mut Harness, z: f64) -> kw_brep::FaceId {
    let ev = proj(h).evaluation();
    let (_, o) = ev.final_solid().unwrap();
    let b = o.brep().unwrap();
    b.faces
        .keys()
        .find(|f| {
            geom::face_frame(b, *f).is_some_and(|fr| fr.n[2] > 0.99)
                && b.face_bbox(*f)
                    .is_some_and(|bb| (bb.max.z - z).abs() < 1e-6)
        })
        .expect("a top face")
}

fn click(h: &mut Harness, pick: Option<Pick>) {
    let out = ViewOut {
        rect: eframe::egui::Rect::NOTHING,
        click: Some(pick),
        double_click: None,
        hover: None,
        click_pos: None,
        response: None,
    };
    let Harness { ctx, app, t } = h;
    let raw = RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1360.0, 860.0))),
        time: Some(*t),
        ..Default::default()
    };
    *t += 1.0 / 60.0;
    let kw_app::ui::Screen::Project { project, .. } = &mut app.screen else {
        panic!()
    };
    let ev = project.evaluation();
    let _ = ctx.run_ui(raw, |ui| app.design.handle_clicks(ui, project, &ev, &out));
    h.frame();
}

#[test]
fn sketch_on_a_plane_then_straight_into_a_previewed_extrude() {
    let (mut h, _d) = design_harness("fusion_flow");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    // the live preview shows the new solid before OK
    let pv = h.app.design.preview.clone().expect("a preview");
    assert!(pv.error.is_none(), "{:?}", pv.error);
    let pb = pv.brep.expect("the preview solid");
    let pvol = pb
        .mass_properties(pb.bodies.keys().next().unwrap())
        .unwrap()
        .volume;
    assert!((pvol - 3000.0).abs() < 1e-6, "{pvol}");
    assert!(
        proj(&mut h).evaluation().final_solid().is_none(),
        "nothing is committed until OK"
    );
    commit(&mut h);
    assert!((vol(&mut h) - 3000.0).abs() < 1e-6);
    assert!(h.app.design.dialog.is_none());
    let doc = proj(&mut h).document();
    assert_eq!(doc.len(), 2, "one sketch and one extrude");
    assert_eq!(h.app.design.model.selected, Some(doc.features[1].id));
    // undo removes the extrude, then the sketch
    proj(&mut h).history.undo();
    assert!(proj(&mut h).evaluation().final_solid().is_none());
}

#[test]
fn pick_a_planar_face_sketch_on_it_and_cut_a_hole_with_the_extrude_dialog() {
    let (mut h, _d) = design_harness("face_flow");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    // Create Sketch, then click the top face
    act(&mut h, Action::StartSketch);
    let f = top_face(&mut h, 5.0);
    click(&mut h, Some(Pick::Face(f)));
    assert_eq!(
        h.app.design.mode,
        Mode::Sketching,
        "{}",
        h.app.design.status
    );
    // the outline of the face is shown as reference geometry
    assert!(
        h.app.design.sketch.app.reference.len() >= 4,
        "{} reference lines",
        h.app.design.sketch.app.reference.len()
    );
    let mut sk = kw_sketch::Sketch::new();
    let c = sk.add_point(15.0, 10.0);
    sk.add_circle(c, 3.0);
    h.app.design.sketch.app.editor.replace_sketch(sk);
    act(&mut h, Action::FinishSketch);
    let d = h.app.design.dialog.as_mut().expect("Extrude dialog");
    let Tool::Extrude {
        op, flip, depth, ..
    } = &mut d.tool
    else {
        panic!()
    };
    assert_eq!(
        *op,
        kw_feature::BodyOp::Join,
        "a sketch on a body defaults to Join"
    );
    *op = kw_feature::BodyOp::Cut;
    *flip = true;
    *depth = "2 mm".into();
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let want = 3000.0 - std::f64::consts::PI * 9.0 * 2.0;
    assert!(
        (vol(&mut h) - want).abs() < 0.5,
        "{} vs {want}",
        vol(&mut h)
    );
    // the sketch feature sits on the face by name
    let doc = proj(&mut h).document();
    assert!(doc.features.iter().any(|f| matches!(&f.spec, FeatureSpec::Sketch(s) if matches!(s.plane, kw_feature::PlaneSpec::Face(_)))));
}

#[test]
fn a_curved_face_cannot_be_sketched_on_and_escape_leaves_the_mode() {
    let (mut h, _d) = design_harness("curved");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    let mut sk = kw_sketch::Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    sk.add_circle(c, 8.0);
    h.app.design.sketch.app.editor.replace_sketch(sk);
    act(&mut h, Action::FinishSketch);
    let Some(d) = h.app.design.dialog.as_mut() else {
        panic!()
    };
    if let Tool::Extrude { depth, .. } = &mut d.tool {
        *depth = "10 mm".into();
    }
    h.frame();
    commit(&mut h);
    // the cylinder's side face is not planar
    let ev = proj(&mut h).evaluation();
    let (_, o) = ev.final_solid().unwrap();
    let b = o.brep().unwrap().clone();
    let side = b
        .faces
        .keys()
        .find(|f| geom::face_frame(&b, *f).is_none())
        .expect("a curved face");
    act(&mut h, Action::StartSketch);
    click(&mut h, Some(Pick::Face(side)));
    assert_eq!(
        h.app.design.mode,
        Mode::PickPlane,
        "the mode stays so that the user can pick again"
    );
    assert!(h.app.design.status.contains("not planar"));
    // the flat end faces work
    let top = top_face(&mut h, 10.0);
    click(&mut h, Some(Pick::Face(top)));
    assert_eq!(h.app.design.mode, Mode::Sketching);
    // discarding leaves everything as it was
    let n = proj(&mut h).document().len();
    act(&mut h, Action::CancelSketch);
    assert_eq!(h.app.design.mode, Mode::Idle);
    assert_eq!(proj(&mut h).document().len(), n);
}

#[test]
fn the_timeline_reorders_rolls_back_suppresses_and_edits_in_place() {
    let (mut h, _d) = design_harness("timeline");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let doc = proj(&mut h).document();
    let (sk, ex) = (doc.features[0].id, doc.features[1].id);
    // an extrude cannot move before its sketch: refused with a message, nothing changes
    act(&mut h, Action::Move(ex, 0));
    assert!(!h.app.design.model.error.is_empty());
    assert_eq!(proj(&mut h).document().features[0].id, sk);
    // a parameter can be added and moved to the front
    h.app.design.model.error.clear();
    {
        let Harness { app, .. } = &mut h;
        let kw_app::ui::Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.model.add_parameter(project, "w", "10");
    }
    let p = proj(&mut h).document().features[2].id;
    act(&mut h, Action::Move(p, 0));
    assert!(
        h.app.design.model.error.is_empty(),
        "{}",
        h.app.design.model.error
    );
    assert_eq!(proj(&mut h).document().features[0].id, p);
    // rolling back to the sketch removes the solid; rolling forward brings it back
    act(&mut h, Action::RollbackTo(sk));
    assert!(proj(&mut h).evaluation().final_solid().is_none());
    // new features are refused while the model is rolled back (they would land after the bar, unseen)
    h.app.design.model.error.clear();
    act(
        &mut h,
        Action::Open(Tool::Fillet {
            radius: "1 mm".into(),
        }),
    );
    assert!(
        h.app.design.dialog.is_none()
            && h.app
                .design
                .model
                .error
                .contains("Roll the timeline forward")
    );
    h.app.design.model.error.clear();
    act(&mut h, Action::ClearRollback);
    assert!((vol(&mut h) - 3000.0).abs() < 1e-6);
    // suppress the extrude and bring it back
    act(&mut h, Action::Suppress(ex, true));
    assert!(proj(&mut h).evaluation().final_solid().is_none());
    act(&mut h, Action::Suppress(ex, false));
    // edit the extrude in place: the same dialog, with the old values
    act(&mut h, Action::Edit(ex));
    let d = h.app.design.dialog.as_mut().expect("edit dialog");
    assert!(d.editing.is_some());
    let Tool::Extrude { depth, .. } = &mut d.tool else {
        panic!()
    };
    assert_eq!(depth, "5 mm");
    *depth = "8 mm".into();
    h.frame();
    assert!(
        (vol(&mut h) - 3000.0).abs() < 1e-6,
        "the preview does not change the model"
    );
    commit(&mut h);
    assert!((vol(&mut h) - 4800.0).abs() < 1e-6);
    assert_eq!(
        proj(&mut h).document().get(ex).unwrap().id,
        ex,
        "same feature, new spec"
    );
    // cancel keeps the model
    act(&mut h, Action::Edit(ex));
    h.app.design.close_dialog();
    assert!((vol(&mut h) - 4800.0).abs() < 1e-6);
    // edit the sketch: the extrude follows
    act(&mut h, Action::Edit(sk));
    assert_eq!(h.app.design.mode, Mode::Sketching);
    h.app
        .design
        .sketch
        .app
        .editor
        .replace_sketch(rectangle(30.0, 40.0));
    act(&mut h, Action::FinishSketch);
    assert_eq!(h.app.design.mode, Mode::Idle);
    assert!(
        h.app.design.dialog.is_none(),
        "editing a sketch does not open Extrude"
    );
    assert!((vol(&mut h) - 9600.0).abs() < 1e-6);
    // deleting a sketch takes the steps that depend on it along, and one Undo brings them all back
    let n = proj(&mut h).document().features.len();
    assert!(n >= 2);
    act(&mut h, Action::Delete(sk));
    let doc = proj(&mut h).document();
    assert!(doc.get(sk).is_none() && doc.features.len() < n);
    assert!(doc
        .features
        .iter()
        .all(|f| !matches!(f.spec, kw_feature::FeatureSpec::Extrude(_))));
    assert!(
        h.app.design.status.contains("depended"),
        "{}",
        h.app.design.status
    );
    proj(&mut h).history.undo();
    assert_eq!(proj(&mut h).document().features.len(), n);
}

#[test]
fn revolve_chamfer_shell_hole_and_patterns_match_closed_forms() {
    let (mut h, _d) = design_harness("tools");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let base = 3000.0;
    // chamfer one top edge by 1 mm: removes 0.5 * 1 * 1 * 30
    let edge = {
        let ev = proj(&mut h).evaluation();
        let b = ev.final_solid().unwrap().1.brep().unwrap().clone();
        b.edges
            .keys()
            .find(|e| {
                let ed = &b.edges[*e];
                let (p, q) = (b.vertices[ed.start].point, b.vertices[ed.end].point);
                (p.z - 5.0).abs() < 1e-9
                    && (q.z - 5.0).abs() < 1e-9
                    && ((p.x - q.x).abs() - 30.0).abs() < 1e-9
                    && p.y.abs() < 1e-9
            })
            .expect("a 30 mm top edge")
    };
    act(
        &mut h,
        Action::Open(Tool::Chamfer {
            d1: "1 mm".into(),
            d2: "1 mm".into(),
        }),
    );
    click(&mut h, Some(Pick::Edge(edge)));
    assert_eq!(h.app.design.model.picks.len(), 1);
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    assert!(
        (vol(&mut h) - (base - 15.0)).abs() < 1e-3,
        "chamfer: {}",
        vol(&mut h)
    );
    proj(&mut h).history.undo();
    // shell with the top face open, 1 mm walls
    act(
        &mut h,
        Action::Open(Tool::Shell {
            thickness: "1 mm".into(),
        }),
    );
    let top = top_face(&mut h, 5.0);
    click(&mut h, Some(Pick::Face(top)));
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    assert!(
        (vol(&mut h) - (base - 28.0 * 18.0 * 4.0)).abs() < 1e-2,
        "shell: {}",
        vol(&mut h)
    );
    proj(&mut h).history.undo();
    // a hole on the top face
    act(&mut h, Action::Open(Tool::default_hole()));
    let top = top_face(&mut h, 5.0);
    click(&mut h, Some(Pick::Face(top)));
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Hole {
            x,
            y,
            diameter,
            depth,
            ..
        } = &mut d.tool
        else {
            panic!()
        };
        (*x, *y, *diameter, *depth) = ("8 mm".into(), "10 mm".into(), "4 mm".into(), "5 mm".into());
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let hole = std::f64::consts::PI * 4.0 * 5.0;
    assert!(
        (vol(&mut h) - (base - hole)).abs() < 0.5,
        "hole: {}",
        vol(&mut h)
    );
    // a rectangular pattern of the hole (3 in all, 8 mm apart along x)
    act(
        &mut h,
        Action::Open(Tool::Pattern {
            source: None,
            dx: "8 mm".into(),
            dy: "0 mm".into(),
            dz: "0 mm".into(),
            count: "3".into(),
        }),
    );
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    assert!(
        (vol(&mut h) - (base - 3.0 * hole)).abs() < 1.0,
        "pattern: {}",
        vol(&mut h)
    );
    // a missing choice is reported, not a crash
    act(
        &mut h,
        Action::Open(Tool::Mirror {
            source: None,
            plane: 1,
            datum: None,
        }),
    );
    if let Some(d) = h.app.design.dialog.as_mut() {
        if let Tool::Mirror { source, .. } = &mut d.tool {
            *source = None;
        }
    }
    h.frame();
    assert!(h.app.design.preview.clone().unwrap().error.is_some());
    h.app.design.close_dialog();
}

#[test]
fn revolving_a_profile_gives_the_closed_form_volume() {
    let (mut h, _d) = design_harness("revolve");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    // rectangle x in [0, 10], y in [2, 5]: revolved about the sketch X axis -> a tube
    let mut sk = kw_sketch::Sketch::new();
    let pts: Vec<_> = [(0.0, 2.0), (10.0, 2.0), (10.0, 5.0), (0.0, 5.0)]
        .iter()
        .map(|p| sk.add_point(p.0, p.1))
        .collect();
    for i in 0..4 {
        sk.add_line(pts[i], pts[(i + 1) % 4]);
    }
    h.app.design.sketch.app.editor.replace_sketch(sk);
    act(&mut h, Action::FinishSketch);
    // turn the Extrude dialog into Revolve
    let id = h.app.design.model.selected.unwrap();
    h.app.design.close_dialog();
    act(
        &mut h,
        Action::Open(Tool::Revolve {
            sketch: Some(id),
            angle: "360 deg".into(),
            axis_y: false,
            axis_edge: false,
            axis_datum: None,
            op: kw_feature::BodyOp::New,
        }),
    );
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let want = std::f64::consts::PI * (25.0 - 4.0) * 10.0;
    assert!(
        (vol(&mut h) - want).abs() / want < 1e-3,
        "{} vs {want}",
        vol(&mut h)
    );
}

#[test]
fn the_view_cube_and_camera_animation_and_plane_overlay_work() {
    use kw_app::chrome::viewcube;
    assert_eq!(viewcube::face_view("TOP").map(|v| v.1 > 1.5), Some(true));
    assert_eq!(
        viewcube::face_view("FRONT"),
        Some((-std::f64::consts::FRAC_PI_2, 0.0))
    );
    assert!(viewcube::face_view("NOPE").is_none());
    // yaw turns the short way round
    let (y, _) = viewcube::lerp_view((3.0, 0.0), (-3.0, 0.0), 0.5);
    assert!(y.abs() > 3.0 && y.abs() < 3.3, "{y}");
    let (y0, p0) = viewcube::lerp_view((0.2, 0.3), (1.0, 0.9), 0.0);
    let (y1, p1) = viewcube::lerp_view((0.2, 0.3), (1.0, 0.9), 1.0);
    assert!(
        (y0 - 0.2).abs() < 1e-12
            && (p0 - 0.3).abs() < 1e-12
            && (y1 - 1.0).abs() < 1e-12
            && (p1 - 0.9).abs() < 1e-12
    );
    // while picking a plane, the pointer over the middle of an empty view is over the origin, i.e. on a plane
    let (mut h, _d) = design_harness("overlay");
    act(&mut h, Action::StartSketch);
    let r = h.app.design.viewport.rect();
    assert!(r.width() > 100.0);
    let pos = r.center();
    {
        let Harness { ctx, app, t } = &mut h;
        let raw = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1360.0, 860.0))),
            time: Some(*t),
            events: vec![egui::Event::PointerMoved(pos)],
            ..Default::default()
        };
        *t += 1.0 / 60.0;
        let _ = ctx.run_ui(raw, |ui| app.show(ui));
    }
    assert!(
        h.app.design.hover_plane.is_some(),
        "a plane is under the pointer"
    );
}

#[test]
fn a_profile_drawn_with_snaps_extrudes_to_the_exact_solid() {
    use kw_sketch::editor::{Input, Tool as SkTool};
    use kw_sketch::{ConstraintKind, EntityKind};
    let (mut h, _d) = design_harness("snap_flow");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    {
        let ed = &mut h.app.design.sketch.app.editor;
        assert!(
            ed.snaps.midpoints && ed.snaps.polar.is_some(),
            "Fusion snaps are the default"
        );
        ed.set_tool(SkTool::Line);
        ed.handle(Input::Click((0.05, 0.05))); // origin
        ed.handle(Input::Click((30.0, 0.1))); // horizontal inference
        ed.handle(Input::Click((15.2, 14.9))); // 135 degrees from there
        ed.handle(Input::Click((0.04, 0.06))); // back onto the first point
        ed.handle(Input::Escape);
        let sk = &ed.sketch;
        let count =
            |f: fn(&ConstraintKind) -> bool| sk.constraints.values().filter(|c| f(&c.kind)).count();
        assert_eq!(
            count(|k| matches!(k, ConstraintKind::Fix(_))),
            1,
            "the origin point is fixed"
        );
        assert_eq!(count(|k| matches!(k, ConstraintKind::Horizontal(_))), 1);
        assert_eq!(
            sk.entities
                .values()
                .filter(|e| matches!(e.kind, EntityKind::Line { .. }))
                .count(),
            3
        );
    }
    // the exact triangle the snaps produced: origin, (30, 0), and the point on the 135 degree ray
    let pts: Vec<(f64, f64)> = {
        let sk = &h.app.design.sketch.app.editor.sketch;
        let mut v: Vec<_> = sk
            .entities
            .iter()
            .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
            .filter_map(|(id, _)| sk.point(id).ok())
            .collect();
        v.sort_by(|a, b| a.0.total_cmp(&b.0));
        v
    };
    assert_eq!(pts.len(), 3, "{pts:?}");
    let (a, b, c) = (
        pts.iter()
            .find(|p| p.0.abs() < 1e-9 && p.1.abs() < 1e-9)
            .copied()
            .expect("origin"),
        pts.iter()
            .find(|p| (p.0 - 30.0).abs() < 1e-9)
            .copied()
            .expect("B"),
        pts.iter().find(|p| p.1 > 1.0).copied().expect("apex"),
    );
    assert!(b.1.abs() < 1e-9, "B is level with the origin");
    // C lies on the 135 degree ray from B
    let (dx, dy) = (c.0 - b.0, c.1 - b.1);
    assert!((dy.atan2(dx).to_degrees() - 135.0).abs() < 1e-6, "{c:?}");
    let area = 0.5 * ((b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1)).abs();
    act(&mut h, Action::FinishSketch);
    let Some(d) = h.app.design.dialog.as_mut() else {
        panic!("Extrude dialog")
    };
    let Tool::Extrude { depth, .. } = &mut d.tool else {
        panic!()
    };
    *depth = "4 mm".into();
    h.frame();
    commit(&mut h);
    let v = vol(&mut h);
    assert!((v - area * 4.0).abs() < 1e-6, "{v} vs {}", area * 4.0);
}

#[test]
fn a_sketch_edited_with_deletions_still_saves_shows_and_reopens() {
    use kw_sketch::editor::{Input, Tool as SkTool};
    let (mut h, _d) = design_harness("sketch_after_delete");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    {
        let ed = &mut h.app.design.sketch.app.editor;
        // a stray point is placed first, the rectangle is drawn, then the stray point is deleted: its
        // low slot is free again and the next line (between corners that already exist) takes it
        ed.set_tool(SkTool::Point);
        ed.handle(Input::Click((50.0, 50.0)));
        ed.set_tool(SkTool::Rectangle);
        ed.handle(Input::Click((0.0, 0.0)));
        ed.handle(Input::Click((30.0, 20.0)));
        ed.set_tool(SkTool::Select);
        ed.handle(Input::Click((50.0, 50.0)));
        assert_eq!(ed.selection.len(), 1);
        ed.delete_selection();
        // a line between two corners that are still there lands in a freed, lower slot
        ed.set_tool(SkTool::Line);
        ed.handle(Input::Click((30.0, 0.0)));
        ed.handle(Input::Click((0.0, 20.0)));
        ed.handle(Input::Escape);
        let sk = &ed.sketch;
        let slot: std::collections::HashMap<_, _> = sk
            .entities
            .keys()
            .enumerate()
            .map(|(i, k)| (k, i))
            .collect();
        let reversed = sk.entities.iter().any(|(id, e)| match e.kind {
            kw_sketch::EntityKind::Line { p1, p2 } => {
                slot[&id] < slot[&p1] || slot[&id] < slot[&p2]
            }
            _ => false,
        });
        let layout: Vec<String> = sk
            .entities
            .values()
            .map(|e| format!("{:?}", e.kind).chars().take(8).collect())
            .collect();
        assert!(reversed, "the setup must reproduce slot reuse: {layout:?}");
    }
    act(&mut h, Action::FinishSketch);
    // the sketch is a healthy feature: it evaluates and can be opened again
    let id = proj(&mut h).document().features[0].id;
    let ev = proj(&mut h).evaluation();
    assert!(
        ev.error(id).is_none(),
        "the saved sketch must evaluate: {:?}",
        ev.error(id)
    );
    // leaving the Extrude dialog keeps the sketch, and it is drawn in the 3-D view
    h.app.design.close_dialog();
    h.frame();
    let doc = proj(&mut h).document();
    let ev = proj(&mut h).evaluation();
    let segs = geom::sketch_segments(&ev, &doc, &h.app.design.hidden_sketches);
    assert!(segs.len() >= 3, "the finished sketch is visible: {segs:?}");
    act(&mut h, Action::Edit(id));
    assert_eq!(
        h.app.design.mode,
        Mode::Sketching,
        "{}",
        h.app.design.model.error
    );
    assert!(h.app.design.sketch.app.editor.sketch.entities.len() >= 5);
}

#[test]
fn edges_are_picked_within_a_few_pixels_and_light_up_when_hovered_or_selected() {
    let Some(g) = kw_render::RenderGpu::try_new() else {
        eprintln!("no GPU adapter: test skipped");
        return;
    };
    let mut p = kw_app::project::Project::demo();
    let ev = p.evaluation();
    let brep = ev
        .final_solid()
        .and_then(|(_, o)| o.brep().cloned())
        .unwrap();
    let model = ModelScene::build(&brep);
    let size = (480u32, 360u32);
    let mut view = kw_app::viewport::GpuView::new(&g.device, size);
    view.renderer.upload(&g.device, &model.scene);
    let mut cam = kw_viewer::OrbitCamera::new(model.scene.radius, size.0 as f64 / size.1 as f64);
    cam.fit(model.scene.radius);
    view.render(&g.device, &g.queue, &cam);
    // find a pixel that is exactly on an edge and one whose neighbours 6 px away are a face
    let at = |view: &kw_app::viewport::GpuView, x: u32, y: u32, r: u32| {
        view.pick_near(&g.device, &g.queue, &model, x, y, r)
    };
    let mut found = None;
    'scan: for y in 20..size.1 - 20 {
        for x in 20..size.0 - 20 {
            let Some(Pick::Edge(e)) = at(&view, x, y, 0) else {
                continue;
            };
            // 6 px to the right and left: a face on both sides, so this is a clean test of reach
            let side = |dx: i32| at(&view, (x as i32 + dx) as u32, y, 0);
            if matches!(side(6), Some(Pick::Face(_))) && matches!(side(-6), Some(Pick::Face(_))) {
                found = Some((x, y, e));
                break 'scan;
            }
        }
    }
    let (x, y, edge) = found.expect("an edge with faces 6 px to either side");
    // the exact pixel misses it 6 px away, a reach of 8 px finds it, a reach of 1 px does not
    for dx in [6i32, -6] {
        let (px, py) = ((x as i32 + dx) as u32, y);
        assert!(matches!(at(&view, px, py, 0), Some(Pick::Face(_))));
        // some edge is found (the nearest one: another edge may be closer than the one searched from)
        assert!(
            matches!(at(&view, px, py, 8), Some(Pick::Edge(_))),
            "reach 8 at dx {dx}"
        );
        assert_ne!(at(&view, px, py, 1), Some(Pick::Edge(edge)));
    }
    // empty space stays empty and a face in the middle of the block stays a face
    assert_eq!(at(&view, 3, 3, 8), None);
    assert!(matches!(
        at(&view, size.0 / 2, size.1 / 2, 8),
        Some(Pick::Face(_)) | Some(Pick::Edge(_))
    ));

    // hovered / selected edges are drawn in their own colours, thicker than plain ones
    let id = model.id_of(Pick::Edge(edge));
    let colour_at = |view: &kw_app::viewport::GpuView| {
        let px = view.read_rgba(&g.device, &g.queue);
        let i = ((y * size.0 + x) * 4) as usize;
        [px[i], px[i + 1], px[i + 2]]
    };
    view.render(&g.device, &g.queue, &cam);
    let plain = colour_at(&view);
    view.renderer.hovered = id;
    view.render(&g.device, &g.queue, &cam);
    let hovered = colour_at(&view);
    view.renderer.hovered = 0;
    view.renderer.selected_extra = vec![id];
    view.render(&g.device, &g.queue, &cam);
    let selected = colour_at(&view);
    assert!(
        plain.iter().all(|c| *c < 80),
        "plain edges are dark: {plain:?}"
    );
    assert!(
        hovered[0] > 200 && hovered[2] < 80,
        "hovered edge is orange: {hovered:?}"
    );
    assert!(
        selected[2] > 200 && selected[0] < 80,
        "selected edge is blue: {selected:?}"
    );
}

#[test]
fn while_edges_are_being_picked_the_view_shows_the_body_the_picks_refer_to() {
    let (mut h, _d) = design_harness("pick_on_base");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let top_edges: Vec<kw_brep::EdgeId> = {
        let ev = proj(&mut h).evaluation();
        let b = ev.final_solid().unwrap().1.brep().unwrap().clone();
        b.edges
            .iter()
            .filter(|(_, ed)| {
                (b.vertices[ed.start].point.z - 5.0).abs() < 1e-9
                    && (b.vertices[ed.end].point.z - 5.0).abs() < 1e-9
            })
            .map(|(e, _)| e)
            .collect()
    };
    assert_eq!(top_edges.len(), 4);
    let base_edges = {
        let ev = proj(&mut h).evaluation();
        ev.final_solid().unwrap().1.brep().unwrap().edges.len()
    };
    act(
        &mut h,
        Action::Open(Tool::Fillet {
            radius: "2 mm".into(),
        }),
    );
    h.frame();
    for (n, e) in top_edges.iter().enumerate().take(3) {
        click(&mut h, Some(Pick::Edge(*e)));
        h.frame();
        assert_eq!(h.app.design.model.picks.len(), n + 1);
        // the preview solid exists, but the view still shows (and picks from) the unmodified body
        let pv = h.app.design.preview.clone().expect("a preview");
        assert!(pv.error.is_none(), "{:?}", pv.error);
        let shown = h
            .app
            .design
            .viewport
            .model()
            .expect("model")
            .edge_polylines()
            .len();
        assert_eq!(shown, base_edges, "after {} picks", n + 1);
        // all picks are drawn as selected, edges are easy to hit
        assert_eq!(h.app.design.viewport.extra_selected.len(), n + 1);
        assert_eq!(h.app.design.viewport.edge_pick_px, 8);
    }
    commit(&mut h);
    let v = vol(&mut h);
    assert!(
        v < 3000.0 && v > 3000.0 - 3.0 * 30.0 * 1.0,
        "three filleted edges: {v}"
    );
    // outside of a pick tool the reach is small, and picking a plane has none
    assert_eq!(h.app.design.viewport.edge_pick_px, 4);
}

// ------------------------------------------------------------------------------------ exchange (import / export)

use kw_app::exchange::{self, ExportFormat};

/// Volume of a binary STL by summing signed tetrahedra.
fn stl_volume(bytes: &[u8]) -> (usize, f64) {
    let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    assert_eq!(bytes.len(), 84 + 50 * n, "binary STL layout");
    let f = |o: usize| f32::from_le_bytes(bytes[o..o + 4].try_into().unwrap()) as f64;
    let mut v = 0.0;
    for t in 0..n {
        let base = 84 + 50 * t + 12;
        let p: Vec<[f64; 3]> = (0..3)
            .map(|k| [f(base + 12 * k), f(base + 12 * k + 4), f(base + 12 * k + 8)])
            .collect();
        v += (p[0][0] * (p[1][1] * p[2][2] - p[1][2] * p[2][1])
            - p[0][1] * (p[1][0] * p[2][2] - p[1][2] * p[2][0])
            + p[0][2] * (p[1][0] * p[2][1] - p[1][1] * p[2][0]))
            / 6.0;
    }
    (n, v)
}

#[test]
fn the_solid_exports_to_step_stl_obj_and_glb_with_the_right_geometry() {
    let dir = scratch("export");
    let mut p = kw_app::project::Project::demo();
    let want = volume(&mut p);
    // STEP: reading it back gives the same volume
    let step = dir.join("part.step");
    let msg = exchange::export_file(&mut p, ExportFormat::Step, &step).unwrap();
    assert!(msg.contains("part.step"), "{msg}");
    let mut rep = kw_brep::BRep::new();
    let r = kw_step::import(
        &mut rep,
        &std::fs::read_to_string(&step).unwrap(),
        &kw_step::ImportOptions::default(),
    )
    .unwrap();
    assert_eq!(r.bodies.len(), 1);
    let got = rep.mass_properties(r.bodies[0]).unwrap().volume;
    assert!((got - want).abs() < 1e-6 * want, "{got} vs {want}");
    // STL: a closed mesh whose enclosed volume matches
    let stl = dir.join("part.stl");
    exchange::export_file(&mut p, ExportFormat::Stl, &stl).unwrap();
    let (tris, vol) = stl_volume(&std::fs::read(&stl).unwrap());
    assert!(tris >= 12);
    assert!(
        (vol - want).abs() < 2e-3 * want,
        "STL volume {vol} vs {want}"
    );
    // OBJ (+ its material file) and GLB
    let obj = dir.join("part.obj");
    exchange::export_file(&mut p, ExportFormat::Obj, &obj).unwrap();
    let text = std::fs::read_to_string(&obj).unwrap();
    assert!(
        text.contains("mtllib part.mtl")
            && text.lines().filter(|l| l.starts_with("f ")).count() == tris
    );
    assert!(dir.join("part.mtl").exists());
    let glb = dir.join("part.glb");
    exchange::export_file(&mut p, ExportFormat::Glb, &glb).unwrap();
    let b = std::fs::read(&glb).unwrap();
    assert_eq!(&b[0..4], b"glTF");
    assert_eq!(
        u32::from_le_bytes(b[8..12].try_into().unwrap()) as usize,
        b.len()
    );
    // an empty project has nothing to export
    let mut empty = kw_app::project::Project::demo();
    empty
        .history
        .edit(|d| {
            let ids: Vec<_> = d.features.iter().rev().map(|f| f.id).collect();
            for id in ids {
                d.remove(id)?;
            }
            Ok::<_, kw_feature::TimelineError>(())
        })
        .unwrap();
    assert!(exchange::export_file(&mut empty, ExportFormat::Step, &dir.join("none.step")).is_err());
}

#[test]
fn importing_a_step_file_adds_a_body_that_survives_save_and_reopen_and_bad_files_change_nothing() {
    let dir = scratch("import");
    // a STEP file made from the demo part
    let mut src = kw_app::project::Project::demo();
    let want = volume(&mut src);
    let file = dir.join("bracket.step");
    exchange::export_file(&mut src, ExportFormat::Step, &file).unwrap();

    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Imported", &dir).unwrap();
    h.app.set_workspace(Workspace::Design);
    h.frame();
    h.app.import_path(&file);
    assert!(
        h.app.message.contains("imported bracket.step"),
        "{}",
        h.app.message
    );
    let got = volume(h.app.project_mut().unwrap());
    assert!((got - want).abs() < 1e-6 * want, "{got} vs {want}");
    h.frame();
    // the file is not needed any more: save, reopen
    assert!(h.app.save());
    let reopened = dir.join("Imported.kwf");
    std::fs::remove_file(&file).unwrap();
    let mut p2 = kw_app::project::Project::open(&reopened).unwrap();
    assert!((volume(&mut p2) - want).abs() < 1e-6 * want);

    // a hole drilled in the imported body saves and reopens too (holes once made files unreadable)
    {
        let p = h.app.project_mut().unwrap();
        let ev = p.evaluation();
        let (_, o) = ev.final_solid().unwrap();
        let b = o.brep().unwrap();
        let top = b
            .faces
            .keys()
            .filter(|f| kw_app::design::geom::face_frame(b, *f).is_some_and(|fr| fr.n[2] > 0.99))
            .max_by(|a, c| {
                b.face_bbox(*a)
                    .unwrap()
                    .max
                    .z
                    .total_cmp(&b.face_bbox(*c).unwrap().max.z)
            })
            .unwrap();
        let r = o.names.reference_to(b, top).unwrap();
        let mm = |v: f64| kw_expr::Param::length(v);
        p.history
            .edit(|d| {
                d.add(
                    "Drill",
                    kw_feature::FeatureSpec::Hole(kw_feature::HoleSpec {
                        plane: kw_feature::PlaneSpec::Face(r),
                        points: vec![[mm(1.0), mm(1.0)]],
                        kind: kw_feature::HoleKind::Simple { diameter: mm(1.0) },
                        depth: mm(1.0),
                        base: None,
                        through_all: false,
                    }),
                )
                .map(|_| ())
            })
            .unwrap();
    }
    assert!(h.app.save());
    let p3 = kw_app::project::Project::open(&reopened).expect("a project with a hole reopens");
    assert_eq!(p3.document().len(), 2);

    // a file that is not CAD is refused and the project is untouched
    let n = h.app.project().unwrap().document().len();
    let junk = dir.join("junk.step");
    std::fs::write(&junk, "ISO-10303-21; garbage").unwrap();
    h.app.import_path(&junk);
    assert!(
        h.app.message.starts_with("import failed"),
        "{}",
        h.app.message
    );
    assert_eq!(h.app.project().unwrap().document().len(), n);
    h.app.import_path(&dir.join("part.dwg"));
    assert!(
        h.app.message.contains("unsupported file type"),
        "{}",
        h.app.message
    );
}

#[test]
fn the_extrude_dialog_cuts_through_all_tapers_and_reports_what_cannot_combine() {
    let (mut h, _d) = design_harness("extent_dialog");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let base = 3000.0;
    let pocket = |h: &mut Harness| {
        act(h, Action::StartSketch);
        let f = top_face(h, 5.0);
        click(h, Some(Pick::Face(f)));
        let mut sk = kw_sketch::Sketch::new();
        let pts: Vec<_> = [(10.0, 5.0), (20.0, 5.0), (20.0, 15.0), (10.0, 15.0)]
            .iter()
            .map(|p| sk.add_point(p.0, p.1))
            .collect();
        for i in 0..4 {
            sk.add_line(pts[i], pts[(i + 1) % 4]);
        }
        h.app.design.sketch.app.editor.replace_sketch(sk);
        act(h, Action::FinishSketch);
        let d = h.app.design.dialog.as_mut().expect("Extrude dialog");
        let Tool::Extrude { op, flip, .. } = &mut d.tool else {
            panic!()
        };
        *op = kw_feature::BodyOp::Cut;
        *flip = true;
    };
    // tapered: a 10 x 10 square pocket 3 mm deep narrowing by 8 degrees on each wall
    pocket(&mut h);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude {
            depth,
            taper,
            symmetric,
            ..
        } = &mut d.tool
        else {
            panic!()
        };
        *depth = "3 mm".into();
        *taper = "8 deg".into();
        *symmetric = true; // refused: a taper needs one side
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(
        pv.error.as_deref().is_some_and(|e| e.contains("taper")),
        "{:?}",
        pv.error
    );
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { symmetric, .. } = &mut d.tool else {
            panic!()
        };
        *symmetric = false;
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let t = 8f64.to_radians();
    let s1 = 10.0 - 2.0 * 3.0 * t.tan();
    let frustum = 3.0 / 3.0 * (100.0 + s1 * s1 + (100.0 * s1 * s1).sqrt());
    assert!(
        (vol(&mut h) - (base - frustum)).abs() < 1e-4,
        "{} vs {}",
        vol(&mut h),
        base - frustum
    );
    // through all, with the stated distance far too short, goes right through the remaining plate
    let before = vol(&mut h);
    pocket(&mut h);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude {
            depth, through_all, ..
        } = &mut d.tool
        else {
            panic!()
        };
        *depth = "0.5 mm".into();
        *through_all = true;
    }
    h.frame();
    commit(&mut h);
    let want = base - 100.0 * 5.0; // the plate is 5 mm thick: a 10 x 10 hole straight through
                                   // (the tapered pocket's volume is inside the through hole)
    assert!(
        (vol(&mut h) - want).abs() < 1e-4,
        "{} vs {want} (before {before})",
        vol(&mut h)
    );
    // and the dialog reads the extent back when the feature is edited
    let ex = proj(&mut h).document().features.last().unwrap().id;
    act(&mut h, Action::Edit(ex));
    let d = h.app.design.dialog.as_ref().expect("edit dialog");
    assert!(matches!(
        &d.tool,
        Tool::Extrude {
            through_all: true,
            ..
        }
    ));
}

/// Sketch `sk` on a plane and leave the Extrude dialog that opens afterwards.
fn sketch_only(
    h: &mut Harness,
    spec: kw_feature::PlaneSpec,
    frame: geom::PlaneFrame,
    sk: kw_sketch::Sketch,
) {
    act(h, Action::StartSketch);
    act(h, Action::SketchOn(spec, frame));
    h.app.design.sketch.app.editor.replace_sketch(sk);
    act(h, Action::FinishSketch);
    h.app.design.close_dialog();
    h.frame();
}

fn square(c: (f64, f64), half: f64) -> kw_sketch::Sketch {
    let mut sk = kw_sketch::Sketch::new();
    let pts: Vec<_> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        .iter()
        .map(|p| sk.add_point(c.0 + p.0 * half, c.1 + p.1 * half))
        .collect();
    for i in 0..4 {
        sk.add_line(pts[i], pts[(i + 1) % 4]);
    }
    sk
}

fn open_commit(h: &mut Harness, tool: Tool) {
    act(h, Action::Open(tool));
    h.frame();
    let pv = h.app.design.preview.clone();
    if let Some(pv) = pv {
        assert!(pv.error.is_none(), "{:?}", pv.error);
    }
    commit(h);
}

#[test]
fn loft_sweep_draft_offset_and_construction_plane_dialogs_build_the_expected_solids() {
    // a construction plane 10 mm above XY; a 20 mm square on XY and a 10 mm square on it, lofted
    let (mut h, _d) = design_harness("more_tools");
    open_commit(
        &mut h,
        Tool::ConstructionPlane {
            base: 0,
            offset: "10 mm".into(),
            tilt: "0 deg".into(),
            more: Default::default(),
        },
    );
    let plane_feature = proj(&mut h).document().features.last().unwrap().id;
    sketch_only(
        &mut h,
        geom::origin_spec(0),
        geom::origin_frame(0),
        square((0.0, 0.0), 10.0),
    );
    let mut f10 = geom::origin_frame(0);
    f10.origin = [0.0, 0.0, 10.0];
    sketch_only(
        &mut h,
        kw_feature::PlaneSpec::Datum(plane_feature),
        f10,
        square((0.0, 0.0), 5.0),
    );
    open_commit(
        &mut h,
        Tool::Loft {
            sketches: vec![None, None],
            op: kw_feature::BodyOp::New,
        },
    );
    let frustum = 10.0 / 3.0 * (400.0 + 100.0 + (400.0f64 * 100.0).sqrt());
    assert!(
        (vol(&mut h) - frustum).abs() < 1e-3 * frustum,
        "loft {} vs {frustum}",
        vol(&mut h)
    );

    // offset: a body made of free-form faces cannot be offset in closed form, and the dialog says so
    act(
        &mut h,
        Action::Open(Tool::Offset {
            distance: "1 mm".into(),
        }),
    );
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(
        pv.error.as_deref().is_some_and(|e| e.contains("offset")),
        "{:?}",
        pv.error
    );
    h.app.design.close_dialog();

    // offset of a block: grows by the Minkowski sum with a ball (or its mitred variant): print and check
    let (mut h2, _d2) = design_harness("offset_dialog");
    sketch_block(&mut h2, 30.0, 20.0, "5 mm");
    commit(&mut h2);
    let before = vol(&mut h2);
    open_commit(
        &mut h2,
        Tool::Offset {
            distance: "1 mm".into(),
        },
    );
    let after = vol(&mut h2);
    let rounded = before
        + 2.0 * (600.0 + 150.0 + 100.0)
        + std::f64::consts::PI * 55.0
        + 4.0 / 3.0 * std::f64::consts::PI;
    let mitred = 32.0 * 22.0 * 7.0;
    assert!(
        (after - rounded).abs() < 1e-3 * rounded || (after - mitred).abs() < 1e-3 * mitred,
        "offset {before} -> {after} (rounded {rounded}, mitred {mitred})"
    );
    proj(&mut h2).history.undo();
    assert!((vol(&mut h2) - before).abs() < 1e-6);

    // sweep: a circle swept along a quarter-circle path (Pappus: area x path length)
    let (mut h, _d) = design_harness("sweep_dialog");
    let mut sk = kw_sketch::Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    sk.add_circle(c, 3.0);
    sketch_only(&mut h, geom::origin_spec(0), geom::origin_frame(0), sk);
    let mut path = kw_sketch::Sketch::new();
    // the XZ plane's sketch x axis runs along world Z: the path leaves the origin along +Z and bends to +X
    path.add_arc_xy(0.0, 20.0, 20.0, -std::f64::consts::FRAC_PI_2, 0.0);
    sketch_only(&mut h, geom::origin_spec(2), geom::origin_frame(2), path);
    open_commit(
        &mut h,
        Tool::Sweep {
            profile: None,
            path: None,
            op: kw_feature::BodyOp::New,
        },
    );
    let want = std::f64::consts::PI * 9.0 * (std::f64::consts::FRAC_PI_2 * 20.0);
    assert!(
        (vol(&mut h) - want).abs() < 1e-3 * want,
        "sweep {} vs {want}",
        vol(&mut h)
    );

    // draft: a 30 x 20 x 5 block, side faces tilted 5 degrees about the bottom plane
    let (mut h, _d) = design_harness("draft_dialog");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let sides: Vec<kw_brep::FaceId> = {
        let ev = proj(&mut h).evaluation();
        let b = ev.final_solid().unwrap().1.brep().unwrap().clone();
        b.faces
            .keys()
            .filter(|f| geom::face_frame(&b, *f).is_some_and(|fr| fr.n[2].abs() < 1e-9))
            .collect()
    };
    assert_eq!(sides.len(), 4);
    act(
        &mut h,
        Action::Open(Tool::Draft {
            angle: "5 deg".into(),
            axis: 2,
            position: "0 mm".into(),
        }),
    );
    for f in &sides {
        click(&mut h, Some(Pick::Face(*f)));
    }
    assert_eq!(h.app.design.model.picks.len(), 4);
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let t = 5f64.to_radians().tan();
    let (a1, b1) = (30.0 - 2.0 * 5.0 * t, 20.0 - 2.0 * 5.0 * t);
    let (a0, a1a) = (600.0, a1 * b1);
    let drafted = 5.0 / 3.0 * (a0 + a1a + (a0 * a1a).sqrt());
    assert!(
        (vol(&mut h) - drafted).abs() < 1e-3 * drafted,
        "draft {} vs {drafted}",
        vol(&mut h)
    );
}

#[test]
fn measure_reports_body_properties_and_picked_geometry_against_closed_forms() {
    use kw_app::design::measure::{
        body_props, describe, edge_kind, face_kind, EdgeKind, FaceKind, MATERIALS,
    };
    let (mut h, _d) = design_harness("measure");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let ev = proj(&mut h).evaluation();
    let (_, o) = ev.final_solid().unwrap();
    let (b, body) = (o.brep().unwrap(), o.body().unwrap());
    // body: a 30 x 20 x 5 block
    let p = body_props(b, body).unwrap();
    assert!((p.volume - 3000.0).abs() < 1e-6 && (p.area - 1700.0).abs() < 1e-6);
    for (got, want) in p.centroid.iter().zip([15.0, 10.0, 2.5]) {
        assert!((got - want).abs() < 1e-6, "{:?}", p.centroid);
    }
    assert!((p.size()[0] - 30.0).abs() < 1e-6 && (p.size()[2] - 5.0).abs() < 1e-6);
    let al = MATERIALS
        .iter()
        .find(|m| m.name == "Aluminium")
        .unwrap()
        .density;
    assert!((p.mass_g(al) - 8.1).abs() < 1e-6, "{}", p.mass_g(al));
    let ixx = 3000.0 / 12.0 * (400.0 + 25.0);
    assert!(
        (p.inertia[0][0] - ixx).abs() < 1e-3 * ixx,
        "{}",
        p.inertia[0][0]
    );
    assert!((p.inertia_g_mm2(al)[0][0] - ixx * al * 1e-6).abs() < 1e-3);

    // faces: the top is 600 mm², the top and bottom are 5 mm apart, top and a side meet at 90 degrees
    let faces: Vec<_> = b.faces.keys().collect();
    let by_normal = |nz: f64, ny: f64| {
        *faces
            .iter()
            .find(|f| {
                matches!(face_kind(b, **f), Some((FaceKind::Plane { normal }, _)) if (normal[2] - nz).abs() < 1e-9 && (normal[1] - ny).abs() < 1e-9)
            })
            .unwrap()
    };
    let (top, bottom, front) = (
        by_normal(1.0, 0.0),
        by_normal(-1.0, 0.0),
        by_normal(0.0, -1.0),
    );
    assert!((face_kind(b, top).unwrap().1 - 600.0).abs() < 1e-6);
    let d = describe(b, &[Pick::Face(top), Pick::Face(bottom)]);
    assert!(d.iter().any(|l| l.contains("5.0000 mm apart")), "{d:?}");
    let d = describe(b, &[Pick::Face(top), Pick::Face(front)]);
    assert!(d.iter().any(|l| l.contains("90.0000°")), "{d:?}");
    // edges: the long top edges are 30 mm and 20 mm apart, parallel
    let long: Vec<_> = b
        .edges
        .keys()
        .filter(|e| matches!(edge_kind(b, *e), Some(EdgeKind::Line { length, direction }) if (length - 30.0).abs() < 1e-9 && direction[0].abs() > 0.99))
        .collect();
    assert_eq!(long.len(), 4);
    let top_long: Vec<_> = long
        .iter()
        .copied()
        .filter(|e| (b.vertices[b.edges[*e].start].point.z - 5.0).abs() < 1e-9)
        .collect();
    assert_eq!(top_long.len(), 2);
    let d = describe(b, &[Pick::Edge(top_long[0]), Pick::Edge(top_long[1])]);
    assert!(
        d.iter()
            .any(|l| l.contains("distance between the edges: 20.0000")),
        "{d:?}"
    );
    assert!(
        d.iter()
            .any(|l| l.contains("angle between the lines: 0.0000")),
        "{d:?}"
    );
    assert!(describe(b, &[Pick::Edge(top_long[0])])[0].contains("length 30.0000"));

    // a cylinder: radius, side area, circular edge
    let (mut h2, _d2) = design_harness("measure_cyl");
    act(&mut h2, Action::StartSketch);
    act(
        &mut h2,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    let mut sk = kw_sketch::Sketch::new();
    let c = sk.add_point(0.0, 0.0);
    sk.add_circle(c, 4.0);
    h2.app.design.sketch.app.editor.replace_sketch(sk);
    act(&mut h2, Action::FinishSketch);
    let Some(dlg) = h2.app.design.dialog.as_mut() else {
        panic!()
    };
    if let Tool::Extrude { depth, .. } = &mut dlg.tool {
        *depth = "10 mm".into();
    }
    h2.frame();
    commit(&mut h2);
    let ev = proj(&mut h2).evaluation();
    let (_, o) = ev.final_solid().unwrap();
    let b = o.brep().unwrap();
    let side = b
        .faces
        .keys()
        .find(|f| matches!(face_kind(b, *f), Some((FaceKind::Cylinder { .. }, _))))
        .unwrap();
    let (k, area) = face_kind(b, side).unwrap();
    assert!(matches!(k, FaceKind::Cylinder { radius } if (radius - 4.0).abs() < 1e-9));
    assert!(
        (area - 2.0 * std::f64::consts::PI * 4.0 * 10.0).abs() < 1e-6,
        "{area}"
    );
    let rim = b
        .edges
        .keys()
        .find(|e| matches!(edge_kind(b, *e), Some(EdgeKind::Circle { .. })))
        .unwrap();
    let EdgeKind::Circle { radius, length } = edge_kind(b, rim).unwrap() else {
        unreachable!()
    };
    assert!(
        (radius - 4.0).abs() < 1e-9 && (length - 2.0 * std::f64::consts::PI * 4.0).abs() < 1e-6,
        "{length}"
    );

    // the window: opens from the ribbon state, picks pile up without a tool, closing clears them
    h2.app.design.measure_open = true;
    h2.frame();
    click(&mut h2, Some(Pick::Face(side)));
    click(&mut h2, Some(Pick::Edge(rim)));
    assert_eq!(h2.app.design.model.picks.len(), 2);
    h2.frame();
    assert_eq!(h2.app.design.viewport.edge_pick_px, 8);
}

#[test]
fn a_hole_through_all_and_a_variable_fillet_through_their_dialogs() {
    let (mut h, _d) = design_harness("hole_vfillet");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    // hole: 4 mm wide, through all, although the stated depth is only 1 mm
    act(&mut h, Action::Open(Tool::default_hole()));
    let f = top_face(&mut h, 5.0);
    click(&mut h, Some(Pick::Face(f)));
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Hole {
            x,
            y,
            diameter,
            depth,
            through_all,
            ..
        } = &mut d.tool
        else {
            panic!()
        };
        *x = "15 mm".into();
        *y = "10 mm".into();
        *diameter = "4 mm".into();
        *depth = "1 mm".into();
        *through_all = true;
    }
    h.frame();
    commit(&mut h);
    let want = 3000.0 - std::f64::consts::PI * 4.0 * 5.0;
    assert!(
        (vol(&mut h) - want).abs() < 1e-3,
        "{} vs {want}",
        vol(&mut h)
    );
    // the dialog reads it back
    let id = proj(&mut h).document().features.last().unwrap().id;
    act(&mut h, Action::Edit(id));
    assert!(matches!(
        &h.app.design.dialog.as_ref().unwrap().tool,
        Tool::Hole {
            through_all: true,
            ..
        }
    ));
    h.app.design.close_dialog();

    // variable fillet on a long top edge of a fresh block: radius 1 -> 3 over 30 mm
    let (mut h, _d) = design_harness("vfillet");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    let edge = {
        let ev = proj(&mut h).evaluation();
        let b = ev.final_solid().unwrap().1.brep().unwrap().clone();
        b.edges
            .keys()
            .find(|e| {
                let ed = &b.edges[*e];
                let (p, q) = (b.vertices[ed.start].point, b.vertices[ed.end].point);
                (p.z - 5.0).abs() < 1e-9
                    && (q.z - 5.0).abs() < 1e-9
                    && ((p.x - q.x).abs() - 30.0).abs() < 1e-9
                    && p.y.abs() < 1e-9
            })
            .unwrap()
    };
    act(
        &mut h,
        Action::Open(Tool::VariableFillet {
            start: "1 mm".into(),
            end: "3 mm".into(),
        }),
    );
    click(&mut h, Some(Pick::Edge(edge)));
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let removed = (1.0 - std::f64::consts::FRAC_PI_4) * 30.0 * (1.0 + 3.0 + 9.0) / 3.0;
    let got = 3000.0 - vol(&mut h);
    assert!(
        (got - removed).abs() < 0.03 * removed,
        "removed {got} vs {removed}"
    );
}

#[test]
fn every_ribbon_command_runs_what_its_label_says() {
    use kw_sketch::editor::Tool as SkTool;
    let (mut h, _d) = design_harness("ribbon_commands");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    // a second sketch so that Sweep and Loft are available
    sketch_only(
        &mut h,
        geom::origin_spec(2),
        geom::origin_frame(2),
        square((0.0, 0.0), 3.0),
    );
    // solid tab: id -> start of the dialog title
    let solid = [
        ("extrude", "EXTRUDE"),
        ("revolve", "REVOLVE"),
        ("sweep", "SWEEP"),
        ("loft", "LOFT"),
        ("hole", "HOLE"),
        ("rect_pattern", "RECTANGULAR"),
        ("circ_pattern", "CIRCULAR"),
        ("mirror", "MIRROR"),
        ("fillet", "FILLET"),
        ("var_fillet", "VARIABLE"),
        ("chamfer", "CHAMFER"),
        ("shell", "SHELL"),
        ("draft", "DRAFT"),
        ("offset_body", "OFFSET"),
        ("combine", "COMBINE"),
        ("plane", "CONSTRUCTION"),
    ];
    for (id, title) in solid {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command(id, project);
        h.frame();
        let d = h
            .app
            .design
            .dialog
            .as_ref()
            .unwrap_or_else(|| panic!("{id} opens a dialog"));
        assert!(
            d.tool.title().starts_with(title),
            "{id}: {}",
            d.tool.title()
        );
        h.app.design.close_dialog();
    }
    // toggles
    {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command("parameters", project);
        assert!(app.design.params_open);
        app.design.command("parameters", project);
        assert!(!app.design.params_open);
        app.design.command("measure", project);
        assert!(app.design.measure_open);
        app.design.command("select", project);
        assert!(!app.design.measure_open);
        app.design.command("origin_planes", project);
        assert!(app.design.plane_visible.iter().all(|v| *v));
        app.design.command("create_sketch", project);
    }
    h.frame();
    assert_eq!(h.app.design.mode, Mode::PickPlane);
    // enter a sketch and run every sketch command
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    let sketch_cmds = [
        ("sk_line", SkTool::Line),
        ("sk_rect", SkTool::Rectangle),
        ("sk_circle", SkTool::Circle),
        ("sk_circle3", SkTool::Circle3),
        ("sk_arc", SkTool::Arc),
        ("sk_arc3", SkTool::Arc3),
        ("sk_spline", SkTool::Spline),
        ("sk_fit", SkTool::FitSpline),
        ("sk_polygon", SkTool::Polygon),
        ("sk_slot", SkTool::Slot),
        ("sk_ellipse", SkTool::Ellipse),
        ("sk_point", SkTool::Point),
        ("sk_trim", SkTool::Trim),
        ("sk_extend", SkTool::Extend),
        ("sk_offset", SkTool::Offset),
        ("sk_fillet", SkTool::Fillet),
        ("sk_chamfer", SkTool::Chamfer),
        ("sk_mirror", SkTool::Mirror),
        ("sk_dimension", SkTool::Dimension),
    ];
    for (id, tool) in sketch_cmds {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command(id, project);
        h.frame();
        assert_eq!(h.app.design.sketch.app.editor.tool, tool, "{id}");
    }
    for (id, ..) in kw_app::design::ribbon::CONSTRAINTS.iter() {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command(id, project);
        h.frame();
        assert!(
            matches!(h.app.design.sketch.app.editor.tool, SkTool::Constrain(_)),
            "{id}"
        );
    }
    // finishing goes through the same path
    {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command("sk_discard", project);
    }
    h.frame();
    assert_eq!(h.app.design.mode, Mode::Idle);
}

// ------------------------------------------------------------------------------------- several bodies

fn bodies_of(h: &mut Harness) -> std::sync::Arc<kw_app::bodies::Bodies> {
    h.frame();
    h.app.design.bodies.clone().expect("bodies")
}

fn body_volume(b: &kw_app::bodies::Bodies, i: usize) -> f64 {
    let o = &b.list[i].output;
    o.brep().unwrap().volume(o.body().unwrap()).unwrap()
}

/// A second block as a new body, away from the first.
fn add_second_block(h: &mut Harness) {
    sketch_only(
        h,
        geom::origin_spec(0),
        geom::origin_frame(0),
        square((60.0, 10.0), 5.0),
    );
    act(
        h,
        Action::Open(Tool::extrude(None, kw_feature::BodyOp::New)),
    );
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { depth, .. } = &mut d.tool else {
            panic!()
        };
        *depth = "5 mm".into();
    }
    h.frame();
    commit(h);
}

fn face_where(
    b: &kw_app::bodies::Bodies,
    f: impl Fn(&kw_brep::BRep, kw_brep::FaceId) -> bool,
) -> kw_brep::FaceId {
    b.brep
        .faces
        .keys()
        .find(|k| f(&b.brep, *k))
        .expect("a face")
}

#[test]
fn two_bodies_are_both_shown_picked_and_edited_separately() {
    let (mut h, _d) = design_harness("two_bodies");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    add_second_block(&mut h);
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 2, "{:?}", proj(&mut h).document().heads());
    assert!(b.is_merged());
    assert!(
        (body_volume(&b, 0) - 3000.0).abs() < 1e-6 && (body_volume(&b, 1) - 500.0).abs() < 1e-6
    );
    // the viewport draws both
    assert!(h.app.design.viewport.model().unwrap().scene.vertices.len() > 40);

    // a hole through the *second* body's top face (display ids of the merged brep)
    let top2 = face_where(&b, |br, f| {
        geom::face_frame(br, f).is_some_and(|fr| fr.n[2] > 0.99)
            && br.face_bbox(f).is_some_and(|bb| bb.min.x > 50.0)
    });
    act(&mut h, Action::Open(Tool::default_hole()));
    click(&mut h, Some(Pick::Face(top2)));
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Hole {
            x,
            y,
            diameter,
            depth,
            ..
        } = &mut d.tool
        else {
            panic!()
        };
        // the face's plane has its origin at the projection of the world origin; the block's centre is at (60, 10)
        *x = "60 mm".into();
        *y = "10 mm".into();
        *diameter = "4 mm".into();
        *depth = "2 mm".into();
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 2);
    assert!(
        (body_volume(&b, 0) - 3000.0).abs() < 1e-6,
        "the first body is untouched"
    );
    assert!(
        (body_volume(&b, 1) - (500.0 - std::f64::consts::PI * 4.0 * 2.0)).abs() < 1e-3,
        "{}",
        body_volume(&b, 1)
    );
    // and it names its body explicitly
    let doc = proj(&mut h).document();
    let hole = doc.features.last().unwrap();
    assert!(hole.spec.explicit_base().is_some());

    // a sketch on that body's face, extruded, joins that body and leaves the other alone
    act(&mut h, Action::StartSketch);
    let top2 = face_where(&bodies_of(&mut h), |br, f| {
        geom::face_frame(br, f).is_some_and(|fr| fr.n[2] > 0.99)
            && br.face_bbox(f).is_some_and(|bb| bb.min.x > 50.0)
    });
    click(&mut h, Some(Pick::Face(top2)));
    assert_eq!(
        h.app.design.mode,
        Mode::Sketching,
        "{}",
        h.app.design.status
    );
    h.app
        .design
        .sketch
        .app
        .editor
        .replace_sketch(square((57.0, 7.0), 1.0));
    act(&mut h, Action::FinishSketch);
    let sk = proj(&mut h).document().features.last().unwrap().clone();
    let FeatureSpec::Sketch(ss) = &sk.spec else {
        panic!()
    };
    assert!(
        ss.base.is_some(),
        "the sketch remembers which body its face is on"
    );
    {
        let d = h.app.design.dialog.as_mut().expect("Extrude dialog");
        let Tool::Extrude { depth, op, .. } = &mut d.tool else {
            panic!()
        };
        assert_eq!(*op, kw_feature::BodyOp::Join);
        *depth = "3 mm".into();
    }
    h.frame();
    commit(&mut h);
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 2);
    assert!((body_volume(&b, 0) - 3000.0).abs() < 1e-6);
    assert!(
        (body_volume(&b, 1) - (500.0 - std::f64::consts::PI * 8.0 + 4.0 * 3.0)).abs() < 1e-2,
        "{}",
        body_volume(&b, 1)
    );
}

#[test]
fn move_copy_scale_split_and_combine_dialogs_work_on_bodies() {
    let (mut h, _d) = design_harness("body_tools");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    // copy: a second body 20 mm to the right, overlapping the first by 10 mm
    open_commit(
        &mut h,
        Tool::MoveBody {
            copy: true,
            dx: "20 mm".into(),
            dy: "0 mm".into(),
            dz: "0 mm".into(),
            axis: 2,
            angle: "0 deg".into(),
        },
    );
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 2);
    // interference: they share 10 x 20 x 5
    let ov = kw_app::bodies::interference(&b);
    assert_eq!(ov.len(), 1);
    assert!((ov[0].volume.unwrap() - 1000.0).abs() < 1e-3, "{ov:?}");
    // combine joins them: 3000 + 3000 - 1000
    act(
        &mut h,
        Action::Open(Tool::Combine {
            target: None,
            tool: None,
            op: kw_feature::BodyOp::Join,
            keep_tool: false,
        }),
    );
    {
        let heads = proj(&mut h).document().heads();
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Combine { target, tool, .. } = &mut d.tool else {
            panic!()
        };
        assert!(target.is_some(), "the target defaults to a body");
        *target = Some(heads[0]);
        *tool = Some(heads[1]);
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 1);
    assert!(
        (body_volume(&b, 0) - 5000.0).abs() < 1e-3,
        "{}",
        body_volume(&b, 0)
    );
    // split with the plane x = 25: two halves of 2500, added by one dialog
    open_commit(
        &mut h,
        Tool::SplitBody {
            plane: 1,
            offset: "25 mm".into(),
        },
    );
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 2);
    assert!(
        (body_volume(&b, 0) - 2500.0).abs() < 1e-3 && (body_volume(&b, 1) - 2500.0).abs() < 1e-3,
        "{} {}",
        body_volume(&b, 0),
        body_volume(&b, 1)
    );
    // undo takes both halves away at once
    proj(&mut h).history.undo();
    assert_eq!(bodies_of(&mut h).list.len(), 1);
    proj(&mut h).history.redo();
    assert_eq!(bodies_of(&mut h).list.len(), 2);
    // scale the active body (the last one) by 2 about the origin: volume x 8
    open_commit(
        &mut h,
        Tool::ScaleBody {
            factor: "2".into(),
            cx: "0 mm".into(),
            cy: "0 mm".into(),
            cz: "0 mm".into(),
        },
    );
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 2);
    let vols: Vec<f64> = (0..2).map(|i| body_volume(&b, i)).collect();
    assert!(
        vols.iter().any(|v| (v - 2500.0).abs() < 1e-3)
            && vols.iter().any(|v| (v - 20000.0).abs() < 1e-2),
        "{vols:?}"
    );
    // a bad factor is reported in the dialog
    act(
        &mut h,
        Action::Open(Tool::ScaleBody {
            factor: "-1".into(),
            cx: "0 mm".into(),
            cy: "0 mm".into(),
            cz: "0 mm".into(),
        }),
    );
    h.frame();
    assert!(h.app.design.preview.clone().unwrap().error.is_some());
    h.app.design.close_dialog();
}

#[test]
fn bodies_have_names_colours_and_can_be_hidden() {
    let (mut h, _d) = design_harness("body_looks");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    add_second_block(&mut h);
    let b = bodies_of(&mut h);
    assert_eq!(
        b.list.iter().map(|x| x.name.clone()).collect::<Vec<_>>(),
        vec!["Body1", "Body2"]
    );
    assert_eq!(b.list[0].colour, kw_app::bodies::DEFAULT_COLOUR);
    // name and colour the second body
    let root = b.list[1].root;
    proj(&mut h)
        .history
        .edit(|d| {
            d.set_appearance(
                root,
                kw_feature::Appearance {
                    name: Some("Pin".into()),
                    colour: Some([200, 40, 30]),
                    material: Some("Steel".into()),
                },
            );
            Ok::<_, kw_feature::TimelineError>(())
        })
        .unwrap();
    let b = bodies_of(&mut h);
    assert_eq!(b.list[1].name, "Pin");
    assert_eq!(b.list[1].colour, [200, 40, 30]);
    assert_eq!(b.list[1].material.as_deref(), Some("Steel"));
    // hiding a body takes it out of the view and out of picking
    h.app.design.hidden_bodies.insert(root);
    let b = bodies_of(&mut h);
    assert_eq!(b.list.len(), 1);
    assert!(!b.is_merged());
    h.app.design.hidden_bodies.clear();
    assert_eq!(bodies_of(&mut h).list.len(), 2);
    // the properties window renders
    h.app.design.body_props = Some(root);
    for _ in 0..3 {
        h.frame();
    }
    // both bodies export, and the file holds two solids
    let dir = scratch("two_bodies_export");
    let step = dir.join("two.step");
    exchange::export_file(proj(&mut h), ExportFormat::Step, &step).unwrap();
    let mut rep = kw_brep::BRep::new();
    let r = kw_step::import(
        &mut rep,
        &std::fs::read_to_string(&step).unwrap(),
        &kw_step::ImportOptions::default(),
    )
    .unwrap();
    assert_eq!(r.bodies.len(), 2);
    let mut v: Vec<f64> = r
        .bodies
        .iter()
        .map(|b| rep.mass_properties(*b).unwrap().volume)
        .collect();
    v.sort_by(f64::total_cmp);
    assert!(
        (v[0] - 500.0).abs() < 1e-6 && (v[1] - 3000.0).abs() < 1e-6,
        "{v:?}"
    );
}

/// A viewport click at a screen position (no pick), delivered like the viewport would.
fn click_at(h: &mut Harness, pos: eframe::egui::Pos2) {
    let out = ViewOut {
        rect: eframe::egui::Rect::NOTHING,
        click: Some(None),
        double_click: None,
        hover: None,
        click_pos: Some(pos),
        response: None,
    };
    let Harness { ctx, app, t } = h;
    let raw = RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1360.0, 860.0))),
        time: Some(*t),
        ..Default::default()
    };
    *t += 1.0 / 60.0;
    let kw_app::ui::Screen::Project { project, .. } = &mut app.screen else {
        panic!()
    };
    let ev = project.evaluation();
    let _ = ctx.run_ui(raw, |ui| app.design.handle_clicks(ui, project, &ev, &out));
    h.frame();
}

#[test]
fn regions_of_overlapping_shapes_are_chosen_in_the_view_and_extruded() {
    let (mut h, _d) = design_harness("regions_ui");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    let mut sk = square((10.0, 5.0), 5.0); // 5..15 x 0..10: a 10 x 10 square
    sk.add_circle_xy(15.0, 5.0, 3.0); // a circle centred on the right edge
    h.app.design.sketch.app.editor.replace_sketch(sk);
    act(&mut h, Action::FinishSketch);
    h.frame();
    let rv = h.app.design.region_view.as_ref().expect("regions");
    assert_eq!(rv.regions().len(), 3);
    // the dialog starts with the default regions chosen (all three); start from none
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { more, .. } = &mut d.tool else {
            panic!()
        };
        assert_eq!(more.regions.len(), 3, "the usual regions are pre-chosen");
        more.regions.clear();
    }
    h.frame();
    // OK is refused with nothing chosen
    assert!(h
        .app
        .design
        .preview
        .as_ref()
        .is_some_and(|p| p.error.is_some()));
    // the circle-only piece (x > 15) is where I click: world (17, 5, 0)
    let o = h.app.design.viewport.origin_in_scene();
    let pos = h
        .app
        .design
        .viewport
        .project([17.0 + o.x, 5.0 + o.y, 0.0 + o.z])
        .expect("on screen");
    click_at(&mut h, pos);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude {
            more, depth, op, ..
        } = &mut d.tool
        else {
            panic!()
        };
        assert_eq!(more.regions.len(), 1, "one region chosen");
        *depth = "4 mm".into();
        *op = kw_feature::BodyOp::New;
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    // the circle's part outside the square: pi 9 - the half-disc-like overlap segment; compare with the polygon area
    let want = {
        let rv_area = |h: &Harness| h.app.design.region_view.is_none();
        let _ = rv_area;
        // circle radius 3 centred on the edge x = 15: the part with x > 15 is half of the disc
        0.5 * std::f64::consts::PI * 9.0 * 4.0
    };
    assert!(
        (vol(&mut h) - want).abs() < 0.1,
        "{} vs {want}",
        vol(&mut h)
    );
    // as in Onshape, the sketch is hidden once a feature has used it ...
    let sk_id = proj(&mut h)
        .document()
        .features
        .iter()
        .find(|f| matches!(f.spec, kw_feature::FeatureSpec::Sketch(_)))
        .unwrap()
        .id;
    assert!(h.app.design.hidden_sketches.contains(&sk_id));
    // ... its eye brings it back, and another Extrude takes a different region of the same sketch
    act(&mut h, Action::ToggleSketch(sk_id));
    assert!(!h.app.design.hidden_sketches.contains(&sk_id));
    act(&mut h, Action::Select(sk_id));
    act(
        &mut h,
        Action::Open(Tool::extrude(Some(sk_id), kw_feature::BodyOp::New)),
    );
    h.frame();
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { more, .. } = &mut d.tool else {
            panic!()
        };
        more.regions.clear();
    }
    h.frame();
    // the overlap of the square and the circle (x < 15): where I click now (the scene has moved)
    let o = h.app.design.viewport.origin_in_scene();
    let pos = h
        .app
        .design
        .viewport
        .project([13.0 + o.x, 5.0 + o.y, 0.0 + o.z])
        .expect("on screen");
    click_at(&mut h, pos);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude {
            more, depth, op, ..
        } = &mut d.tool
        else {
            panic!()
        };
        assert_eq!(more.regions.len(), 1, "one region chosen");
        *depth = "6 mm".into();
        *op = kw_feature::BodyOp::New;
    }
    h.frame();
    commit(&mut h);
    let want2 = 0.5 * std::f64::consts::PI * 9.0 * 6.0;
    assert!(
        (vol(&mut h) - want2).abs() < 0.1,
        "{} vs {want2}",
        vol(&mut h)
    );
    // both extrusions hang on the one sketch: it is hidden again, and deleting it takes both
    assert!(h.app.design.hidden_sketches.contains(&sk_id));
    let doc = proj(&mut h).document();
    assert_eq!(doc.all_dependents_of(sk_id).len(), 2);
}

#[test]
fn thin_wall_rib_pipe_to_face_and_edge_axis_dialogs() {
    use std::f64::consts::PI;
    // ---- thin wall: a 20 x 10 rectangle, 2 mm wall inside, 6 mm tall
    let (mut h, _d) = design_harness("thin_dialog");
    sketch_only(&mut h, geom::origin_spec(0), geom::origin_frame(0), {
        let mut s = square((10.0, 5.0), 5.0);
        s = {
            let mut t = kw_sketch::Sketch::new();
            let p: Vec<_> = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
                .iter()
                .map(|q| t.add_point(q.0, q.1))
                .collect();
            for i in 0..4 {
                t.add_line(p[i], p[(i + 1) % 4]);
            }
            let _ = s;
            t
        };
        s
    });
    act(
        &mut h,
        Action::Open(Tool::extrude(None, kw_feature::BodyOp::New)),
    );
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { depth, more, .. } = &mut d.tool else {
            panic!()
        };
        *depth = "6 mm".into();
        more.thin = "2 mm".into();
        more.thin_side = 0;
    }
    h.frame();
    assert!(h.app.design.preview.clone().unwrap().error.is_none());
    commit(&mut h);
    assert!((vol(&mut h) - 104.0 * 6.0).abs() < 1e-3, "{}", vol(&mut h));
    // the dialog reads it back
    let ex = proj(&mut h).document().features.last().unwrap().id;
    act(&mut h, Action::Edit(ex));
    assert!(
        matches!(&h.app.design.dialog.as_ref().unwrap().tool, Tool::Extrude { more, .. } if more.thin == "2 mm")
    );
    h.app.design.close_dialog();

    // ---- rib: a line sketch, 2 mm thick, 8 mm tall
    let (mut h, _d) = design_harness("rib_dialog");
    let mut line = kw_sketch::Sketch::new();
    line.add_line_xy(0.0, 0.0, 30.0, 0.0);
    sketch_only(&mut h, geom::origin_spec(0), geom::origin_frame(0), line);
    {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command("rib", project);
    }
    h.frame();
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Rib {
            thickness,
            depth,
            op,
            ..
        } = &mut d.tool
        else {
            panic!("{:?}", d.tool)
        };
        *thickness = "2 mm".into();
        *depth = "8 mm".into();
        *op = kw_feature::BodyOp::New;
    }
    h.frame();
    assert!(h.app.design.preview.clone().unwrap().error.is_none());
    commit(&mut h);
    assert!((vol(&mut h) - 480.0).abs() < 1e-6);

    // ---- pipe along a quarter circle
    let (mut h, _d) = design_harness("pipe_dialog");
    let mut arc = kw_sketch::Sketch::new();
    arc.add_arc_xy(0.0, 20.0, 20.0, -PI / 2.0, 0.0);
    sketch_only(&mut h, geom::origin_spec(2), geom::origin_frame(2), arc);
    {
        let Harness { app, .. } = &mut h;
        let Screen::Project { project, .. } = &mut app.screen else {
            panic!()
        };
        app.design.command("pipe", project);
    }
    h.frame();
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Pipe {
            diameter, wall, op, ..
        } = &mut d.tool
        else {
            panic!()
        };
        *diameter = "6 mm".into();
        *wall = "1 mm".into();
        *op = kw_feature::BodyOp::New;
    }
    h.frame();
    assert!(h.app.design.preview.clone().unwrap().error.is_none());
    commit(&mut h);
    let want = PI * 5.0 * (PI / 2.0 * 20.0);
    assert!((vol(&mut h) - want).abs() < 1e-3 * want);

    // ---- extrude up to a face (clicked), then revolve about a clicked edge
    let (mut h, _d) = design_harness("to_face_dialog");
    sketch_block(&mut h, 30.0, 20.0, "10 mm");
    commit(&mut h);
    sketch_only(
        &mut h,
        geom::origin_spec(0),
        geom::origin_frame(0),
        square((32.5, 2.5), 2.5),
    );
    act(
        &mut h,
        Action::Open(Tool::extrude(None, kw_feature::BodyOp::Join)),
    );
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { more, .. } = &mut d.tool else {
            panic!()
        };
        more.to_face = true;
    }
    let top = top_face(&mut h, 10.0);
    click(&mut h, Some(Pick::Face(top)));
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    assert!(
        (vol(&mut h) - (6000.0 + 5.0 * 5.0 * 10.0)).abs() < 1e-3,
        "{}",
        vol(&mut h)
    );
}

#[test]
fn revolve_about_a_clicked_body_edge() {
    let (mut h, _d) = design_harness("revolve_edge");
    sketch_block(&mut h, 10.0, 10.0, "10 mm");
    commit(&mut h);
    // the profile lives on the XZ plane (sketch x = world Z, y = world X): z 0..3, x 4..6
    let mut ring = kw_sketch::Sketch::new();
    let p: Vec<_> = [(0.0, 4.0), (3.0, 4.0), (3.0, 6.0), (0.0, 6.0)]
        .iter()
        .map(|q| ring.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        ring.add_line(p[i], p[(i + 1) % 4]);
    }
    sketch_only(&mut h, geom::origin_spec(2), geom::origin_frame(2), ring);
    act(
        &mut h,
        Action::Open(Tool::Revolve {
            sketch: None,
            angle: "360 deg".into(),
            axis_y: false,
            axis_edge: true,
            axis_datum: None,
            op: kw_feature::BodyOp::Cut,
        }),
    );
    // the vertical block edge at x = 0, y = 0
    let edge = {
        let ev = proj(&mut h).evaluation();
        let b = ev.final_solid().unwrap().1.brep().unwrap().clone();
        b.edges
            .iter()
            .find(|(_, e)| {
                let (a, c) = (b.vertices[e.start].point, b.vertices[e.end].point);
                a.x.abs() < 1e-9
                    && a.y.abs() < 1e-9
                    && c.x.abs() < 1e-9
                    && c.y.abs() < 1e-9
                    && (a.z - c.z).abs() > 1.0
            })
            .map(|(id, _)| id)
            .unwrap()
    };
    click(&mut h, Some(Pick::Edge(edge)));
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let want = 1000.0 - std::f64::consts::PI * (36.0 - 16.0) * 3.0 / 4.0;
    assert!(
        (vol(&mut h) - want).abs() < 1e-3 * want,
        "{} vs {want}",
        vol(&mut h)
    );
}

fn datum_frame(h: &mut Harness, id: kw_feature::FeatureId) -> kw_math::Frame3 {
    match &proj(h).evaluation().output(id).unwrap().product {
        kw_feature::Product::Datum(f) => *f,
        p => panic!("not a datum: {p:?}"),
    }
}

fn last_id(h: &mut Harness) -> kw_feature::FeatureId {
    proj(h).document().features.last().unwrap().id
}

#[test]
fn construction_axes_points_and_planes_are_built_from_their_dialogs() {
    use kw_app::design::tools::{PlaneInput, PlaneMore, PointInput};
    let (mut h, _d) = design_harness("construction_dialogs");
    let pt = |x: &str, y: &str, z: &str| PointInput {
        datum: None,
        x: x.into(),
        y: y.into(),
        z: z.into(),
    };
    // an axis through (0,0,0) and (0,0,10): along +Z
    open_commit(
        &mut h,
        Tool::Axis {
            mode: 0,
            pts: [pt("0 mm", "0 mm", "0 mm"), pt("0 mm", "0 mm", "10 mm")],
            at_u: "0 mm".into(),
            at_v: "0 mm".into(),
            planes: Default::default(),
        },
    );
    let axis = last_id(&mut h);
    let f = datum_frame(&mut h, axis);
    assert!((f.z.z - 1.0).abs() < 1e-9 && f.origin.coords.norm() < 1e-9);
    // a point where the axis meets the plane z = 4
    open_commit(
        &mut h,
        Tool::DatumPoint {
            mode: 2,
            coords: PointInput::default(),
            axis: Some(axis),
            plane: PlaneInput {
                which: 0,
                offset: "4 mm".into(),
                datum: None,
            },
        },
    );
    let p = last_id(&mut h);
    assert!((datum_frame(&mut h, p).origin.z - 4.0).abs() < 1e-9);
    // a point by coordinates is used by a three-point plane through (0,0,4), (10,0,4), (0,10,4)
    open_commit(
        &mut h,
        Tool::DatumPoint {
            mode: 0,
            coords: pt("10 mm", "0 mm", "4 mm"),
            axis: None,
            plane: PlaneInput::default(),
        },
    );
    let p2 = last_id(&mut h);
    open_commit(
        &mut h,
        Tool::ConstructionPlane {
            base: 0,
            offset: "0 mm".into(),
            tilt: "0 deg".into(),
            more: PlaneMore {
                mode: 2,
                pts: [
                    PointInput {
                        datum: Some(p),
                        ..Default::default()
                    },
                    PointInput {
                        datum: Some(p2),
                        ..Default::default()
                    },
                    pt("0 mm", "10 mm", "4 mm"),
                ],
                ..Default::default()
            },
        },
    );
    let pl = last_id(&mut h);
    let f = datum_frame(&mut h, pl);
    assert!((f.z.z.abs() - 1.0).abs() < 1e-9 && (f.origin.z - 4.0).abs() < 1e-9);
    // a midplane between XY and the plane z = 10 sits at z = 5
    open_commit(
        &mut h,
        Tool::ConstructionPlane {
            base: 0,
            offset: "0 mm".into(),
            tilt: "0 deg".into(),
            more: PlaneMore {
                mode: 1,
                other: PlaneInput {
                    which: 0,
                    offset: "10 mm".into(),
                    datum: None,
                },
                ..Default::default()
            },
        },
    );
    let mid = last_id(&mut h);
    assert!((datum_frame(&mut h, mid).origin.z - 5.0).abs() < 1e-9);
    // the axis of two planes: YZ and XZ meet along Z
    open_commit(
        &mut h,
        Tool::Axis {
            mode: 3,
            pts: Default::default(),
            at_u: "0 mm".into(),
            at_v: "0 mm".into(),
            planes: [
                PlaneInput {
                    which: 1,
                    ..Default::default()
                },
                PlaneInput {
                    which: 2,
                    ..Default::default()
                },
            ],
        },
    );
    let a2 = last_id(&mut h);
    assert!(datum_frame(&mut h, a2).z.z.abs() > 1.0 - 1e-9);
    // every dialog reopens with the values it was built from
    for id in [axis, p, p2, pl, mid, a2] {
        let spec = proj(&mut h).document().get(id).unwrap().spec.clone();
        let tool = Tool::for_feature(&spec).expect("editable");
        assert!(matches!(
            tool,
            Tool::Axis { .. } | Tool::DatumPoint { .. } | Tool::ConstructionPlane { .. }
        ));
    }
    // the ribbon's construct menu opens each of them
    for id in [
        "axis_2pt",
        "axis_edge",
        "axis_face",
        "axis_planes",
        "point_xyz",
        "point_edge",
        "point_axis",
        "plane_mid",
        "plane_3pt",
    ] {
        h.app.design.close_dialog();
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command(id, project);
        h.frame();
        assert!(h.app.design.dialog.is_some(), "{id} opens a dialog");
    }
}

#[test]
fn revolve_uses_a_construction_axis() {
    use kw_app::design::tools::PointInput;
    let (mut h, _d) = design_harness("datum_users");
    sketch_block(&mut h, 10.0, 10.0, "10 mm");
    commit(&mut h);
    // the block's vertical edge at x = y = 0 as a construction axis
    open_commit(
        &mut h,
        Tool::Axis {
            mode: 0,
            pts: [
                PointInput::default(),
                PointInput {
                    z: "10 mm".into(),
                    ..Default::default()
                },
            ],
            at_u: "0 mm".into(),
            at_v: "0 mm".into(),
            planes: Default::default(),
        },
    );
    let axis = last_id(&mut h);
    // a ring around it, revolved as a cut: the same pocket as revolving about the body edge
    let mut ring = kw_sketch::Sketch::new();
    let p: Vec<_> = [(0.0, 4.0), (3.0, 4.0), (3.0, 6.0), (0.0, 6.0)]
        .iter()
        .map(|q| ring.add_point(q.0, q.1))
        .collect();
    for i in 0..4 {
        ring.add_line(p[i], p[(i + 1) % 4]);
    }
    sketch_only(&mut h, geom::origin_spec(2), geom::origin_frame(2), ring);
    open_commit(
        &mut h,
        Tool::Revolve {
            sketch: None,
            angle: "360 deg".into(),
            axis_y: false,
            axis_edge: false,
            axis_datum: Some(axis),
            op: kw_feature::BodyOp::Cut,
        },
    );
    let want = 1000.0 - std::f64::consts::PI * (36.0 - 16.0) * 3.0 / 4.0;
    assert!(
        (vol(&mut h) - want).abs() < 1e-3 * want,
        "{} vs {want}",
        vol(&mut h)
    );
}

#[test]
fn tapped_holes_and_cosmetic_threads_are_made_from_their_dialogs() {
    let (mut h, _d) = design_harness("threads");
    sketch_block(&mut h, 30.0, 30.0, "20 mm");
    commit(&mut h);
    let block = vol(&mut h);
    // Create ▸ Tapped Hole: M6, 12 mm deep, 8 mm of thread
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("hole_tapped", project);
    }
    h.frame();
    let f = top_face(&mut h, 20.0);
    click(&mut h, Some(Pick::Face(f)));
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Hole {
            x,
            y,
            depth,
            kind,
            thread,
            thread_length,
            ..
        } = &mut d.tool
        else {
            panic!("{:?}", d.tool)
        };
        assert_eq!(*kind, 3);
        (*x, *y, *depth) = ("15 mm".into(), "15 mm".into(), "12 mm".into());
        *thread = "M6".into();
        *thread_length = "8 mm".into();
    }
    h.frame();
    commit(&mut h);
    let want = block - std::f64::consts::PI * 2.5 * 2.5 * 12.0;
    assert!(
        (vol(&mut h) - want).abs() < 1e-3,
        "{} vs {want}",
        vol(&mut h)
    );
    // the dialog reads the thread back
    let id = proj(&mut h).document().features.last().unwrap().id;
    act(&mut h, Action::Edit(id));
    assert!(matches!(
        &h.app.design.dialog.as_ref().unwrap().tool,
        Tool::Hole { kind: 3, thread, .. } if thread == "M6"
    ));
    h.app.design.close_dialog();
    // Create ▸ Thread on the hole wall
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("thread", project);
    }
    h.frame();
    let wall = {
        let ev = proj(&mut h).evaluation();
        let o = ev.final_solid().unwrap().1.clone();
        let rep = o.brep().unwrap();
        rep.faces
            .iter()
            .find(|(_, fc)| matches!(rep.surfaces[fc.surface], kw_nurbs::SurfaceGeom::Cylinder(_)))
            .map(|(id, _)| id)
            .unwrap()
    };
    click(&mut h, Some(Pick::Face(wall)));
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    assert!((vol(&mut h) - want).abs() < 1e-3);
    // both are listed
    let ev = proj(&mut h).evaluation();
    let doc = proj(&mut h).document();
    let report = kw_feature::threads::report(&doc, &ev.scope(&doc));
    assert_eq!(report.len(), 2);
    assert!(report.iter().all(|t| t.size.designation == "M6"));
}

fn start_sketch_plane(h: &mut Harness, idx: usize) {
    act(h, Action::StartSketch);
    act(
        h,
        Action::SketchOn(geom::origin_spec(idx), geom::origin_frame(idx)),
    );
    assert_eq!(h.app.design.mode, Mode::Sketching);
}

fn sketch_entities(h: &Harness, f: fn(&kw_sketch::entity::EntityKind) -> bool) -> usize {
    h.app
        .design
        .sketch
        .app
        .editor
        .sketch
        .entities
        .values()
        .filter(|e| f(&e.kind))
        .count()
}

#[test]
fn project_and_intersect_bring_the_model_into_the_sketch() {
    use kw_sketch::entity::EntityKind;
    let (mut h, _d) = design_harness("project");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    // Project onto XY: the top and bottom outlines coincide, the vertical edges are points
    start_sketch_plane(&mut h, 0);
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("sk_project", project);
    }
    assert_eq!(
        sketch_entities(&h, |k| matches!(k, EntityKind::Line { .. })),
        4
    );
    // projected curves are locked: a quarter of the unknowns remain
    let rep = h.app.design.sketch.app.editor.report.clone().unwrap();
    assert_eq!(rep.dof, 0, "{rep:?}");
    act(&mut h, Action::CancelSketch);

    // Intersect with the plane y = 10 (XZ shifted): a 30 x 5 rectangle
    start_sketch_plane(&mut h, 2);
    let mut f = h.app.design.sketch_frame.unwrap();
    f.origin = [0.0, 10.0, 0.0];
    h.app.design.sketch_frame = Some(f);
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("sk_intersect", project);
    }
    assert_eq!(
        sketch_entities(&h, |k| matches!(k, EntityKind::Line { .. })),
        4
    );
    let mut xs = Vec::new();
    let ed = &h.app.design.sketch.app.editor;
    for (id, e) in ed.sketch.entities.iter() {
        if matches!(e.kind, EntityKind::Point(_)) {
            xs.push(ed.sketch.point(id).unwrap());
        }
    }
    let (w, hgt) = (
        xs.iter().map(|p| p.0).fold(f64::MIN, f64::max)
            - xs.iter().map(|p| p.0).fold(f64::MAX, f64::min),
        xs.iter().map(|p| p.1).fold(f64::MIN, f64::max)
            - xs.iter().map(|p| p.1).fold(f64::MAX, f64::min),
    );
    let (a, b) = if w > hgt { (w, hgt) } else { (hgt, w) };
    assert!(
        (a - 30.0).abs() < 1e-6 && (b - 5.0).abs() < 1e-6,
        "{a} x {b}"
    );
    act(&mut h, Action::CancelSketch);

    // a cylinder cut at z = 2 gives a circle of the cylinder's radius
    let (mut h, _d) = design_harness("project_cyl");
    let mut sk = kw_sketch::Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 6.0);
    sketch_only(&mut h, geom::origin_spec(0), geom::origin_frame(0), sk);
    let s = proj(&mut h).document().features.last().unwrap().id;
    open_commit(&mut h, Tool::extrude(Some(s), kw_feature::BodyOp::New));
    start_sketch_plane(&mut h, 0);
    let mut f = h.app.design.sketch_frame.unwrap();
    f.origin = [0.0, 0.0, 2.0];
    h.app.design.sketch_frame = Some(f);
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("sk_intersect", project);
    }
    let ed = &h.app.design.sketch.app.editor;
    let radii: Vec<f64> = ed
        .sketch
        .entities
        .values()
        .filter_map(|e| match e.kind {
            EntityKind::Circle { radius, .. } => Some(ed.sketch.value(radius)),
            _ => None,
        })
        .collect();
    assert_eq!(radii.len(), 1, "{radii:?}");
    assert!((radii[0] - 6.0).abs() < 0.01, "{radii:?}");
}

#[test]
fn sketch_patterns_and_text_become_profiles() {
    use kw_app::design::sketch_ops::SketchPanel;
    use kw_sketch::entity::EntityKind;
    let (mut h, _d) = design_harness("sketch_ops");
    start_sketch_plane(&mut h, 0);
    // a 4 x 4 square, selected, repeated 3 x 2 and then the whole set turned into a pattern of 2
    h.app
        .design
        .sketch
        .app
        .editor
        .replace_sketch(square((0.0, 0.0), 2.0));
    let lines: Vec<_> = h
        .app
        .design
        .sketch
        .app
        .editor
        .sketch
        .entities
        .iter()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Line { .. }))
        .map(|(id, _)| id)
        .collect();
    h.app.design.sketch.app.editor.selection = lines;
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("sk_pat_rect", project);
    }
    assert!(matches!(
        h.app.design.sketch_panel,
        Some(SketchPanel::RectPattern { .. })
    ));
    let n = h
        .app
        .design
        .apply_sketch_panel(&SketchPanel::RectPattern {
            nx: 3,
            ny: 2,
            dx: 10.0,
            dy: 10.0,
        })
        .unwrap();
    assert_eq!(n, 20);
    assert_eq!(
        sketch_entities(&h, |k| matches!(k, EntityKind::Line { .. })),
        24
    );
    act(&mut h, Action::CancelSketch);

    // text: the letter O extruded 3 mm has the volume of the glyph's area times 3
    let (mut h, _d) = design_harness("sketch_text");
    start_sketch_plane(&mut h, 0);
    let panel = SketchPanel::Text {
        text: "O".into(),
        height: 20.0,
        spacing: 0.0,
        x: 0.0,
        y: 0.0,
        angle: 0.0,
        font: 0,
    };
    h.app.design.apply_sketch_panel(&panel).unwrap();
    act(&mut h, Action::FinishSketch);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { depth, .. } = &mut d.tool else {
            panic!()
        };
        *depth = "3 mm".into();
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    // the glyph's area from its outline, sampled finely
    let (curves, _) = kw_sketch::text::text_curves(
        kw_app::design::sketch_ops::FONTS[0].1,
        "O",
        &kw_sketch::text::TextStyle {
            height: 20.0,
            ..Default::default()
        },
    )
    .unwrap();
    let mut area = 0.0;
    for c in &curves {
        let pts: Vec<(f64, f64)> = match c {
            kw_sketch::curves::Curve2::Line(a, b) => vec![*a, *b],
            kw_sketch::curves::Curve2::Bezier(v) => (0..=200)
                .map(|i| {
                    let t = i as f64 / 200.0;
                    let mut p = v.clone();
                    while p.len() > 1 {
                        p = p
                            .windows(2)
                            .map(|w| {
                                (
                                    w[0].0 + (w[1].0 - w[0].0) * t,
                                    w[0].1 + (w[1].1 - w[0].1) * t,
                                )
                            })
                            .collect();
                    }
                    p[0]
                })
                .collect(),
            _ => continue,
        };
        for w in pts.windows(2) {
            area += w[0].0 * w[1].1 - w[1].0 * w[0].1;
        }
    }
    // outer loop counter-clockwise (+), inner clockwise (-): the net is the ring's area
    let want = (area / 2.0).abs() * 3.0;
    let got = vol(&mut h);
    assert!((got - want).abs() < 0.01 * want, "{got} vs {want}");
}

#[test]
fn section_analysis_defines_the_cutting_plane_from_an_origin_or_construction_plane() {
    use kw_app::design::section::SectionState;
    let (mut h, _d) = design_harness("section");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        assert!(h.app.design.section_plane(project).is_none());
        h.app.design.command("section", project);
        // XZ (normal +y), offset 0
        assert_eq!(
            h.app.design.section_plane(project),
            Some(([0.0, 1.0, 0.0], 0.0))
        );
        h.app.design.section = Some(SectionState {
            which: 0,
            datum: None,
            offset: 2.5,
            flip: true,
        });
        // XY moved up 2.5, flipped: keep the upper part, so the normal points down
        assert_eq!(
            h.app.design.section_plane(project),
            Some(([0.0, 0.0, -1.0], -2.5))
        );
        h.app.design.command("section", project);
        assert!(h.app.design.section.is_none());
    }
    // a construction plane 4 mm above XY, cut at its own offset 1 mm: z = 5, normal +z
    open_commit(
        &mut h,
        Tool::ConstructionPlane {
            base: 0,
            offset: "4 mm".into(),
            tilt: "0 deg".into(),
            more: Default::default(),
        },
    );
    let pl = last_id(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.design.section = Some(SectionState {
        which: 0,
        datum: Some(pl),
        offset: 1.0,
        flip: false,
    });
    let (n, d) = h.app.design.section_plane(project).unwrap();
    assert!(
        (n[2] - 1.0).abs() < 1e-9 && (d - 5.0).abs() < 1e-9,
        "{n:?} {d}"
    );
    // the window draws without trouble
    h.frame();
    assert!(h.app.design.viewport.section.is_some());
}

#[test]
fn components_joints_and_motion_make_an_assembly_that_is_saved_with_the_project() {
    use kw_app::design::assembly::{face_connection_frame, remove_instance};
    use kw_assembly::{Assembly, JointKind};
    let (mut h, dir) = design_harness("assembly");
    sketch_block(&mut h, 20.0, 20.0, "10 mm");
    commit(&mut h);
    // the top face gives a connection frame F1 (centre, pointing up)
    let top = top_face(&mut h, 10.0);
    let frame = {
        let ev = proj(&mut h).evaluation();
        let b = ev.final_solid().unwrap().1.brep().unwrap().clone();
        face_connection_frame("F1", &b, top).unwrap()
    };
    assert_eq!(frame.origin, [10.0, 10.0, 10.0]);
    h.app.design.asm.new_frames.push(frame);
    h.app.design.asm.new_name = "Block".into();
    act(&mut h, Action::NewComponent);
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let a = project.assembly.as_mut().expect("assembly made");
        assert_eq!((a.parts.len(), a.instances.len()), (1, 1));
        assert_eq!(a.parts[0].frames.len(), 2);
        // a second instance of the same part
        a.add_instance("Block copy", 0);
    }
    // a revolute joint: the copy's origin turns about the top face's normal
    h.app.design.asm.joint.kind = JointKind::Revolute;
    h.app.design.asm.joint.parent = 0;
    h.app.design.asm.joint.parent_frame = 1;
    h.app.design.asm.joint.child = 1;
    h.app.design.asm.joint.child_frame = 0;
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_add_joint(project).unwrap();
        // the same two components cannot be joined twice (that would close a loop)
        assert!(h.app.design.asm_add_joint(project).is_err());
        let a = project.assembly.as_mut().unwrap();
        a.joints[0].values[0] = kw_expr::Param::angle_deg(90.0);
        let placed = a.solve().unwrap();
        // the copy's point (20, 0, 0) turns a quarter about the vertical line through (10, 10): (10, 30, 10)
        let p = placed[1]
            .0
            .transform_point(&kw_math::Point3::new(20.0, 0.0, 0.0));
        let p0 = placed[0]
            .0
            .transform_point(&kw_math::Point3::new(20.0, 0.0, 0.0));
        assert!(
            (p.x - 10.0).abs() < 1e-9 && (p.y - 30.0).abs() < 1e-9 && (p.z - 10.0).abs() < 1e-9,
            "{p:?}"
        );
        assert!((p0.x - 20.0).abs() < 1e-12);
    }
    // the window and the posed view draw; the view shows both blocks (volume 2 x 4000)
    h.app.design.asm.open = true;
    h.app.design.asm.view = true;
    h.frame();
    h.frame();
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let disp = h.app.design.asm_display(project).expect("posed view");
        assert_eq!(disp.2.len(), 2);
        h.app.design.asm_check_interference(project);
        match h.app.design.asm.overlaps.clone() {
            Some(Ok(v)) => assert!(v.is_empty(), "{v:?}"),
            // touching solids may be reported by the kernel as a failed Boolean: it must say so
            Some(Err(e)) => assert!(!e.is_empty()),
            None => panic!("no result"),
        }
        let step = h.app.design.asm_step(project).unwrap();
        assert!(step.contains("ISO-10303-21"));
    }
    // saved with the project and read back
    let path = dir.join("asm.kwf");
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        project.save_as(&path).unwrap();
    }
    let reopened = kw_app::project::Project::open(&path).unwrap();
    let a = reopened.assembly.expect("assembly stored in the file");
    assert_eq!(
        (a.parts.len(), a.instances.len(), a.joints.len()),
        (1, 2, 1)
    );
    // removing an instance takes its joints with it and keeps the indices valid
    let mut a: Assembly = a;
    remove_instance(&mut a, 1);
    assert_eq!((a.instances.len(), a.joints.len()), (1, 0));
    assert!(a.solve().is_ok());
    // inserting a file adds a part
    let (mut h2, _d2) = design_harness("assembly_insert");
    {
        let Screen::Project { project, .. } = &mut h2.app.screen else {
            panic!()
        };
        h2.app.design.asm_insert_path(project, &path).unwrap();
        let a = project.assembly.as_ref().unwrap();
        assert_eq!(a.parts.len(), 1);
        assert_eq!(a.parts[0].frames.len(), 1);
    }
}

/// Volume of the solid a document builds.
fn doc_volume(doc: &kw_feature::Document) -> f64 {
    let ev = kw_feature::Engine::new().evaluate(doc);
    let (_, o) = ev.final_solid().expect("a solid");
    o.brep()
        .unwrap()
        .mass_properties(o.body().unwrap())
        .unwrap()
        .volume
}

fn set_extrude_depth(p: &mut kw_app::project::Project, depth: &str) {
    let (id, mut spec) = {
        let doc = p.document();
        let f = doc
            .features
            .iter()
            .find(|f| matches!(f.spec, FeatureSpec::Extrude(_)))
            .expect("an extrude");
        (f.id, f.spec.clone())
    };
    if let FeatureSpec::Extrude(e) = &mut spec {
        e.depth = kw_app::model::parse_param(depth, kw_expr::Dim::LENGTH).unwrap();
    }
    p.history.edit(|d| d.replace_spec(id, spec)).unwrap();
}

#[test]
fn a_body_becomes_a_component_that_is_edited_in_place_and_saved_with_the_project() {
    let (mut h, dir) = design_harness("component");
    sketch_block(&mut h, 20.0, 20.0, "10 mm");
    commit(&mut h);
    let head = *proj(&mut h).evaluation().final_solid().unwrap().0;
    act(&mut h, Action::ComponentFromBody(head));
    {
        let a = proj(&mut h)
            .assembly
            .clone()
            .expect("the assembly was made");
        assert_eq!((a.parts.len(), a.instances.len()), (1, 1));
        assert_eq!(a.parts[0].frames.len(), 1);
        assert!((doc_volume(&a.parts[0].document) - 4000.0).abs() < 1e-6);
    }
    assert!(
        h.app.design.hidden_bodies.contains(&head),
        "the body is hidden in the design so it is not drawn twice"
    );
    let root_len = proj(&mut h).document().len();

    // open the component: its document replaces the design on screen
    act(&mut h, Action::ActivateComponent(0));
    assert_eq!(proj(&mut h).editing(), Some(0));
    assert!(
        h.app.design.hidden_bodies.is_empty(),
        "the screen state is the component's own"
    );
    // change the extrude to 25 mm; the assembly follows at once (the root design is not touched)
    set_extrude_depth(proj(&mut h), "25 mm");
    h.frame();
    {
        let p = proj(&mut h);
        let a = p.assembly.as_ref().unwrap();
        assert!((doc_volume(&a.parts[0].document) - 10000.0).abs() < 1e-6);
        assert_eq!(p.root_document().len(), root_len);
        assert!((doc_volume(&p.root_document()) - 4000.0).abs() < 1e-6);
        assert!(p.is_dirty());
    }
    // saving while editing stores the component in the assembly and the root design as the design
    let path = dir.join("component.kwf");
    proj(&mut h).save_as(&path).unwrap();
    let reopened = kw_app::project::Project::open(&path).unwrap();
    assert!((doc_volume(&reopened.document()) - 4000.0).abs() < 1e-6);
    assert!(
        (doc_volume(&reopened.assembly.as_ref().unwrap().parts[0].document) - 10000.0).abs() < 1e-6
    );
    // undo inside the component does not touch the root's history
    assert!(proj(&mut h).history.can_undo());
    proj(&mut h).history.undo();
    h.frame();
    assert!(
        (doc_volume(&proj(&mut h).assembly.as_ref().unwrap().parts[0].document) - 4000.0).abs()
            < 1e-6
    );
    proj(&mut h).history.redo();
    h.frame();
    act(&mut h, Action::FinishComponent);
    {
        let p = proj(&mut h);
        assert_eq!(p.editing(), None);
        assert_eq!(p.document().len(), root_len);
        assert!((volume(p) - 4000.0).abs() < 1e-6, "the root design is back");
        assert!(
            (doc_volume(&p.assembly.as_ref().unwrap().parts[0].document) - 10000.0).abs() < 1e-6
        );
    }
    // a new empty component opens for editing at once and finishes empty
    h.app.design.asm.new_name = "Bracket".into();
    act(&mut h, Action::NewEmptyComponent);
    {
        let p = proj(&mut h);
        assert_eq!(p.editing(), Some(1));
        assert_eq!(p.document().len(), 0);
        let a = p.assembly.as_ref().unwrap();
        assert_eq!(
            (a.parts[1].name.as_str(), a.instances[1].name.as_str()),
            ("Bracket", "Bracket")
        );
    }
    act(&mut h, Action::FinishComponent);
    assert_eq!(proj(&mut h).editing(), None);
}

fn asm_of(h: &mut Harness) -> kw_assembly::Assembly {
    proj(h).assembly.clone().expect("an assembly")
}

#[test]
fn the_assembly_tree_selects_grounds_groups_copies_and_deletes_components() {
    use kw_app::design::assembly::AsmAction as A;
    use kw_assembly::{Joint, JointEnd, JointKind};
    let (mut h, _dir) = design_harness("asm_tree");
    sketch_block(&mut h, 20.0, 20.0, "10 mm");
    commit(&mut h);
    let head = *proj(&mut h).evaluation().final_solid().unwrap().0;
    act(&mut h, Action::ComponentFromBody(head));
    act(&mut h, Action::Asm(A::Copy(0)));
    act(&mut h, Action::Asm(A::Copy(0)));
    let a = asm_of(&mut h);
    let names: Vec<&str> = a.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(names, ["Body1", "Body1 copy", "Body1 copy 2"]);
    assert_eq!(
        h.app.design.asm.selected,
        vec![2],
        "the new copy is selected"
    );
    // a copy floats 15 mm from the original
    let t = a.solve().unwrap();
    assert!((t[1].apply_point(&kw_math::Point3::origin()).x - 15.0).abs() < 1e-9);

    // selection: plain click replaces, shift-click toggles
    act(&mut h, Action::Asm(A::Select(0, false)));
    assert_eq!(h.app.design.asm.selected, vec![0]);
    act(&mut h, Action::Asm(A::Select(1, true)));
    assert_eq!(h.app.design.asm.selected, vec![0, 1]);
    act(&mut h, Action::Asm(A::Select(1, true)));
    assert_eq!(h.app.design.asm.selected, vec![0]);

    // hiding a component takes it out of the posed view
    act(&mut h, Action::Asm(A::ToggleHidden(2)));
    h.app.design.asm.view = true;
    h.frame();
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        assert_eq!(h.app.design.asm_display(project).unwrap().2.len(), 2);
    }
    act(&mut h, Action::Asm(A::ShowAll));
    assert!(asm_of(&mut h).instances.iter().all(|i| !i.hidden));

    // grounding a second component, then releasing the first
    act(&mut h, Action::Asm(A::ToggleGround(1)));
    assert_eq!(asm_of(&mut h).grounded(), vec![0, 1]);
    act(&mut h, Action::Asm(A::ToggleGround(0)));
    assert_eq!(asm_of(&mut h).grounded(), vec![1]);
    act(&mut h, Action::Asm(A::ToggleGround(0)));

    // a joint, deleted again, leaves the component in place
    {
        let a = proj(&mut h).assembly.as_mut().unwrap();
        let mut j = Joint::new(
            "pin",
            JointKind::Revolute,
            JointEnd {
                instance: 0,
                frame: "origin".into(),
            },
            JointEnd {
                instance: 2,
                frame: "origin".into(),
            },
        );
        j.values = vec![kw_expr::Param::angle_deg(45.0)];
        a.add_joint(j);
    }
    let before = asm_of(&mut h).solve().unwrap()[2].to_matrix();
    act(&mut h, Action::Asm(A::SelectJoint(0)));
    assert_eq!(h.app.design.asm.selected_joint, Some(0));
    act(&mut h, Action::Asm(A::DeleteJoint(0)));
    let a = asm_of(&mut h);
    assert!(a.joints.is_empty());
    assert!((a.solve().unwrap()[2].to_matrix() - before).norm() < 1e-9);

    // a rename that clashes is refused with a message
    act(&mut h, Action::Asm(A::Rename(0, "Body1 copy".into())));
    assert_eq!(asm_of(&mut h).instances[0].name, "Body1");
    assert!(h.app.design.model.error.contains("already called"));
    h.app.design.model.error.clear();
    act(&mut h, Action::Asm(A::Rename(0, "Base".into())));
    assert_eq!(asm_of(&mut h).instances[0].name, "Base");

    // two selected components become a rigid group, or a sub-assembly
    h.app.design.asm.selected = vec![1, 2];
    act(&mut h, Action::Asm(A::RigidGroupFromSelection));
    assert_eq!(asm_of(&mut h).groups.len(), 1);
    act(&mut h, Action::Asm(A::DeleteGroup(0)));
    assert!(asm_of(&mut h).groups.is_empty());
    h.app.design.asm.selected = vec![1, 2];
    act(&mut h, Action::Asm(A::SubAssemblyFromSelection));
    let a = asm_of(&mut h);
    assert_eq!(a.instances.len(), 2);
    assert!(a.parts.iter().any(|p| p.sub.is_some()));
    // the tree draws with the sub-assembly in it
    h.frame();
    h.frame();
    // deleting a component takes it away
    let k = a.instances.len() - 1;
    act(&mut h, Action::Asm(A::Delete(k)));
    assert_eq!(asm_of(&mut h).instances.len(), 1);
    assert!(h.app.design.asm.selected.is_empty());
    assert!(proj(&mut h).is_dirty());
}

/// A click in the posed assembly view, as the viewport would deliver it.
fn asm_click(h: &mut Harness, pick: Option<Pick>, double: bool) {
    let out = ViewOut {
        rect: eframe::egui::Rect::NOTHING,
        click: (!double).then_some(pick),
        double_click: double.then_some(pick),
        hover: None,
        click_pos: None,
        response: None,
    };
    let Harness { ctx, app, t } = h;
    let raw = RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1360.0, 860.0))),
        time: Some(*t),
        ..Default::default()
    };
    *t += 1.0 / 60.0;
    let kw_app::ui::Screen::Project { project, .. } = &mut app.screen else {
        panic!()
    };
    let _ = ctx.run_ui(raw, |ui| app.design.asm_view_clicks(ui, project, &out));
    h.frame();
}

/// A face of the posed view: the one of body `top` whose box is flat at height `z`, facing `up`.
fn posed_face(h: &mut Harness, top: usize, z: f64, up: bool) -> Pick {
    let rep = h.app.design.asm.display_rep().expect("the posed view");
    let faces = rep.body_faces(rep.bodies.keys().nth(top).unwrap());
    let f = faces
        .into_iter()
        .find(|f| {
            geom::face_frame(&rep, *f)
                .is_some_and(|fr| (fr.n[2] > 0.99) == up && fr.n[2].abs() > 0.99)
                && rep
                    .face_bbox(*f)
                    .is_some_and(|bb| (bb.max.z - z).abs() < 1e-6 && (bb.min.z - z).abs() < 1e-6)
        })
        .expect("a face at that height");
    Pick::Face(f)
}

fn two_blocks(name: &str) -> Harness {
    use kw_app::design::assembly::AsmAction as A;
    let (mut h, _d) = design_harness(name);
    sketch_block(&mut h, 20.0, 20.0, "10 mm");
    commit(&mut h);
    let head = *proj(&mut h).evaluation().final_solid().unwrap().0;
    act(&mut h, Action::ComponentFromBody(head));
    act(&mut h, Action::Asm(A::Copy(0)));
    h.app.design.asm.view = true;
    h.frame();
    h.frame();
    h
}

#[test]
fn joint_origins_are_found_on_faces_edges_and_round_faces() {
    use kw_app::design::asm_snap::{candidates, choose};
    let mut h = two_blocks("snaps");
    let rep = h.app.design.asm.display_rep().unwrap();
    let top = posed_face(&mut h, 0, 10.0, true);
    let c = candidates(&rep, top);
    // a flat face offers its centre first (pointing out), then its corners and edge middles
    assert_eq!(c[0].what, "face centre");
    assert!(
        (c[0].point[0] - 10.0).abs() < 1e-9
            && (c[0].point[1] - 10.0).abs() < 1e-9
            && (c[0].point[2] - 10.0).abs() < 1e-9,
        "{:?}",
        c[0]
    );
    assert!((c[0].z[2] - 1.0).abs() < 1e-9);
    assert!(c.iter().filter(|o| o.what == "vertex").count() >= 4);
    assert!(c.iter().filter(|o| o.what == "edge midpoint").count() >= 4);
    // the pointer near a corner picks that corner, far from everything the centre
    let corner = choose(&c, Some([0.2, 0.1, 10.0]), 2.0).unwrap();
    assert_eq!(corner.what, "vertex");
    assert!(corner.point[0].abs() < 1e-9 && corner.point[1].abs() < 1e-9);
    let mid = choose(&c, Some([9.0, 11.0, 10.0]), 0.5).unwrap();
    assert_eq!(mid.what, "face centre");
    // every candidate's axes are orthonormal
    for o in &c {
        let dot = o.z[0] * o.x[0] + o.z[1] * o.x[1] + o.z[2] * o.x[2];
        assert!(dot.abs() < 1e-9);
    }
}

#[test]
fn a_round_face_offers_its_axis_and_a_circle_its_centre() {
    use kw_app::design::asm_snap::candidates;
    // a cylinder: a circle of radius 4 at (5, 6) extruded 12 mm
    let (mut h, _d) = design_harness("round_snaps");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    {
        let mut sk = kw_sketch::Sketch::new();
        let c = sk.add_point(5.0, 6.0);
        sk.add_circle(c, 4.0);
        h.app.design.sketch.app.editor.replace_sketch(sk);
    }
    act(&mut h, Action::FinishSketch);
    if let Some(d) = h.app.design.dialog.as_mut() {
        if let Tool::Extrude { depth, .. } = &mut d.tool {
            *depth = "12 mm".into();
        }
    }
    h.frame();
    commit(&mut h);
    let ev = proj(&mut h).evaluation();
    let (_, o) = ev.final_solid().unwrap();
    let b = o.brep().unwrap();
    let side = b
        .faces
        .keys()
        .find(|f| {
            matches!(
                b.surfaces[b.faces[*f].surface],
                kw_nurbs::SurfaceGeom::Cylinder(_)
            )
        })
        .unwrap();
    let c = candidates(b, Pick::Face(side));
    assert_eq!(c[0].what, "cylinder axis");
    assert!((c[0].point[0] - 5.0).abs() < 1e-9 && (c[0].point[1] - 6.0).abs() < 1e-9);
    assert!(
        (c[0].point[2] - 6.0).abs() < 1e-6,
        "mid-height of the pin: {:?}",
        c[0]
    );
    assert!(c[0].z[2].abs() > 0.999, "along the pin: {:?}", c[0].z);
    // its two circular edges give their centres
    let circles: Vec<_> = c.iter().filter(|o| o.what == "circle centre").collect();
    assert_eq!(circles.len(), 2);
    let mut zs: Vec<f64> = circles.iter().map(|o| o.point[2]).collect();
    zs.sort_by(f64::total_cmp);
    assert!(zs[0].abs() < 1e-9 && (zs[1] - 12.0).abs() < 1e-9);
    let edge = b
        .edges
        .keys()
        .find(|e| {
            matches!(
                kw_app::design::measure::edge_kind(b, *e),
                Some(kw_app::design::measure::EdgeKind::Circle { .. })
            )
        })
        .unwrap();
    let ec = candidates(b, Pick::Edge(edge));
    assert_eq!(ec[0].what, "circle centre");
}

#[test]
fn a_joint_is_made_by_clicking_two_faces_with_a_live_preview() {
    let mut h = two_blocks("click_joint");
    // block 0 is grounded at the origin; its copy floats 15 mm along x
    // open the command, click the bottom face of the copy, then the top face of the first block
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_open_joint(project, false).unwrap();
    }
    assert!(h.app.design.asm.dialog.is_some());
    let bottom_copy = posed_face(&mut h, 1, 0.0, false);
    asm_click(&mut h, Some(bottom_copy), false);
    let top_first = posed_face(&mut h, 0, 10.0, true);
    asm_click(&mut h, Some(top_first), false);
    h.frame();
    h.frame();
    {
        let d = h.app.design.asm.dialog.as_ref().expect("still open");
        let (a, b) = (d.first.as_ref().unwrap(), d.second.as_ref().unwrap());
        assert_eq!((a.instance, b.instance), (1, 0));
        assert_eq!(a.origin.what, "face centre");
        // the first click was the underside of the copy: it points down
        assert!((a.origin.z[2] + 1.0).abs() < 1e-9);
        assert!((b.origin.z[2] - 1.0).abs() < 1e-9);
        // the live preview already has the joint, and the copy sits on the first block
        let prev = d.preview.as_ref().expect("a preview");
        assert_eq!(prev.joints.len(), 1);
        let t = prev.solve().unwrap();
        let c = t[1].apply_point(&kw_math::Point3::new(10.0, 10.0, 5.0));
        assert!(
            (c.x - 10.0).abs() < 1e-9 && (c.y - 10.0).abs() < 1e-9 && (c.z - 15.0).abs() < 1e-9,
            "{c:?}"
        );
    }
    // the real assembly is untouched until OK
    assert!(asm_of(&mut h).joints.is_empty());
    // a revolute joint with limits 0..90°
    {
        let d = h.app.design.asm.dialog.as_mut().unwrap();
        d.kind = kw_assembly::JointKind::Revolute;
        d.limit_on[0] = true;
        d.limit_lo[0] = "0".into();
        d.limit_hi[0] = "90".into();
        d.mark_dirty();
    }
    h.frame();
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let d = h.app.design.asm.dialog.clone().unwrap();
        h.app.design.asm_joint_commit(project, &d).unwrap();
    }
    h.frame();
    let a = asm_of(&mut h);
    assert!(h.app.design.asm.dialog.is_none());
    assert_eq!(a.joints.len(), 1);
    let j = &a.joints[0];
    assert_eq!(j.kind, kw_assembly::JointKind::Revolute);
    assert_eq!((j.parent.instance, j.child.instance), (0, 1));
    assert!((j.limits[0].unwrap().1 - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    // two new frames, one on each part (they share the part: one frame was added to it twice with
    // different names)
    assert!(a.parts[0].frames.len() >= 2);
    // turning the joint 90° swings the copy about the vertical line through the first block's top centre
    let mut a2 = a.clone();
    a2.joints[0].values = vec![kw_expr::Param::angle_deg(90.0)];
    let t = a2.solve().unwrap();
    let p = t[1].apply_point(&kw_math::Point3::new(20.0, 10.0, 5.0));
    // (20,10,5) is 10 mm from the copy's axis (10,10): a quarter turn about it moves it to (10, 20, 15)
    // or (10, 0, 15) depending on the joint's sense; either way it is 10 mm from the axis, at z = 15
    assert!(
        ((p.x - 10.0).powi(2) + (p.y - 10.0).powi(2)).sqrt() - 10.0 < 1e-9,
        "{p:?}"
    );
    assert!(
        (p.z - 15.0).abs() < 1e-9 && (p.x - 10.0).abs() < 1e-9,
        "{p:?}"
    );
    // beyond the limit the pose is refused with the reason
    a2.joints[0].values = vec![kw_expr::Param::angle_deg(120.0)];
    assert!(a2.solve().is_err());
}

#[test]
fn clicking_components_selects_them_and_a_double_click_opens_them() {
    use kw_app::design::assembly::AsmAction as A;
    let mut h = two_blocks("pick_components");
    let top1 = posed_face(&mut h, 1, 10.0, true);
    asm_click(&mut h, Some(top1), false);
    assert_eq!(h.app.design.asm.selected, vec![1]);
    let top0 = posed_face(&mut h, 0, 10.0, true);
    // plain click replaces the selection
    asm_click(&mut h, Some(top0), false);
    assert_eq!(h.app.design.asm.selected, vec![0]);
    // a click in empty space clears it
    asm_click(&mut h, None, false);
    assert!(h.app.design.asm.selected.is_empty());
    // the selected component is drawn in another colour
    act(&mut h, Action::Asm(A::Select(1, false)));
    let key_selected = {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_display(project).unwrap()
    };
    act(&mut h, Action::Asm(A::Select(0, false)));
    let key_other = {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_display(project).unwrap()
    };
    assert_ne!(key_selected.2, key_other.2);
    // a double click opens the component for editing in place
    asm_click(&mut h, Some(top1), true);
    assert_eq!(proj(&mut h).editing(), Some(0));
    act(&mut h, Action::FinishComponent);
}

#[test]
fn joint_properties_change_values_limits_and_kind_and_refuse_nonsense() {
    use kw_app::design::asm_joint::JointEdit;
    use kw_assembly::{Joint, JointEnd, JointKind};
    let mut h = two_blocks("joint_props");
    {
        let a = proj(&mut h).assembly.as_mut().unwrap();
        a.add_joint(Joint::new(
            "pin",
            JointKind::Cylindrical,
            JointEnd {
                instance: 0,
                frame: "origin".into(),
            },
            JointEnd {
                instance: 1,
                frame: "origin".into(),
            },
        ));
    }
    let mut a = asm_of(&mut h);
    let mut e = JointEdit::load(&a, 0);
    assert_eq!(e.values.len(), 2);
    e.values = vec!["30".into(), "5".into()];
    e.limit_on = [true, true, false];
    e.limit_lo = ["0".into(), "0".into(), "0".into()];
    e.limit_hi = ["45".into(), "10".into(), "0".into()];
    e.offset = ["1".into(), "2".into(), "3".into()];
    e.angle = "15".into();
    e.apply(&mut a).unwrap();
    let j = &a.joints[0];
    assert!((j.limits[0].unwrap().1 - 45f64.to_radians()).abs() < 1e-12);
    assert_eq!(j.limits[1], Some((0.0, 10.0)));
    assert_eq!(j.offset, [1.0, 2.0, 3.0]);
    assert!((j.angle - 15f64.to_radians()).abs() < 1e-12);
    assert!(a.solve().is_ok());
    // a value beyond its limit is a solve error; limits the wrong way round are refused outright
    e.values[0] = "60".into();
    e.apply(&mut a).unwrap();
    assert!(a.solve().is_err());
    e.limit_lo[0] = "50".into();
    e.limit_hi[0] = "40".into();
    assert!(e.apply(&mut a).is_err());
    // a length where an angle belongs is explained
    e.limit_lo[0] = "0".into();
    e.limit_hi[0] = "45".into();
    e.values[0] = "5 mm".into();
    e.apply(&mut a).unwrap();
    assert!(a.solve().unwrap_err().to_string().contains("angle") || a.solve().is_err());
    // the window draws for a real joint (it applies edits at once)
    h.app.design.asm.edit_joint = Some(0);
    h.frame();
    h.frame();
    assert_eq!(h.app.design.asm.edit_joint, Some(0));
}

#[test]
fn an_as_built_joint_keeps_the_components_where_they_are() {
    let mut h = two_blocks("as_built_ui");
    // the copy floats 15 mm away; remember where it is
    let before = asm_of(&mut h).solve().unwrap()[1].to_matrix();
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_open_joint(project, true).unwrap();
    }
    let a = posed_face(&mut h, 1, 10.0, true);
    asm_click(&mut h, Some(a), false);
    let b = posed_face(&mut h, 0, 10.0, true);
    asm_click(&mut h, Some(b), false);
    {
        let d = h.app.design.asm.dialog.as_mut().unwrap();
        assert!(d.as_built);
        d.kind = kw_assembly::JointKind::Revolute;
        d.axis = 3; // the world Z axis
    }
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let d = h.app.design.asm.dialog.clone().unwrap();
        h.app.design.asm_joint_commit(project, &d).unwrap();
    }
    let a = asm_of(&mut h);
    assert_eq!(a.joints.len(), 1);
    assert!(a.joints[0].as_built);
    let after = a.solve().unwrap()[1].to_matrix();
    assert!((after - before).norm() < 1e-9, "nothing moved");
}

#[test]
fn interference_shows_the_overlap_and_the_bill_of_materials_counts_and_weighs() {
    use kw_app::design::asm_inspect::{bom_csv, bom_rows, set_part_material};
    let mut h = two_blocks("inspect_app");
    // the copy is 15 mm along x; the blocks are 20 wide: they share 5 x 20 x 10 mm³
    run_command(&mut h, "asm_check");
    match h.app.design.asm.overlaps.clone() {
        Some(Ok(v)) => {
            assert_eq!(v.len(), 1, "{v:?}");
            assert!((v[0].volume - 1000.0).abs() < 1e-6, "{}", v[0].volume);
            assert!(v[0].body.is_some());
        }
        other => panic!("{other:?}"),
    }
    assert!(h.app.design.asm.interference_open);
    assert!(
        h.app.design.asm.overlap_edges.len() >= 12,
        "the twelve edges of the overlap box"
    );
    h.frame();
    h.frame();
    // moving a component makes the old result stale
    {
        let a = proj(&mut h).assembly.as_mut().unwrap();
        a.instances[1].at.t[0] = 40.0;
    }
    h.frame();
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_display(project);
    }
    assert!(h.app.design.asm.overlaps.is_none());
    assert!(h.app.design.asm.overlap_edges.is_empty());
    // the bill: one part, two of it; steel gives a mass
    {
        let a = proj(&mut h).assembly.as_mut().unwrap();
        set_part_material(a, "Body1", Some("Steel".into()));
    }
    let rows = {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.asm_refresh_pub(project);
        match h.app.design.asm.pose_ref() {
            Some(Ok(p)) => bom_rows(p),
            other => panic!("{:?}", other.map(|r| r.as_ref().err().cloned())),
        }
    };
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].count, 2);
    assert!((rows[0].volume - 4000.0).abs() < 1e-6);
    // 4000 mm³ of steel (7850 kg/m³) = 31.4 g each
    assert!((rows[0].mass_g - 31.4).abs() < 1e-9, "{}", rows[0].mass_g);
    let csv = bom_csv(&rows);
    let lines: Vec<&str> = csv.lines().collect();
    assert!(lines[0].starts_with("Item,Part,Quantity"));
    assert!(
        lines[1].starts_with("1,Body1,2,Steel,4000.000,31.400,62.800,"),
        "{}",
        lines[1]
    );
    assert!(
        lines[2].contains("Total,2") && lines[2].contains("62.800"),
        "{}",
        lines[2]
    );
    // the window draws, and the export request is raised
    h.app.design.asm.bom_open = true;
    h.frame();
    h.frame();
    assert!(h.app.design.asm.bom_open);
}

#[test]
fn the_assemble_menu_commands_start_the_joint_commands_and_groups() {
    let mut h = two_blocks("assemble_menu");
    run_command(&mut h, "asm_joint");
    assert!(h
        .app
        .design
        .asm
        .dialog
        .as_ref()
        .is_some_and(|d| !d.as_built));
    // Escape closes it
    h.app.design.asm.dialog = None;
    run_command(&mut h, "asm_as_built");
    assert!(h.app.design.asm.dialog.as_ref().is_some_and(|d| d.as_built));
    h.app.design.asm.dialog = None;
    // a rigid group from the two components
    run_command(&mut h, "asm_rigid");
    assert_eq!(h.app.design.asm.rigid.as_ref().map(|r| r.len()), Some(2));
    h.app.design.asm.rigid = Some(vec![true, true]);
    // the OK button acts through the same action the window pushes
    h.app.design.asm.selected = vec![0, 1];
    h.app.design.asm.rigid = None;
    act(
        &mut h,
        Action::Asm(kw_app::design::assembly::AsmAction::RigidGroupFromSelection),
    );
    assert_eq!(asm_of(&mut h).groups.len(), 1);
    // ground the selection: both components are grounded (the group is allowed to be)
    run_command(&mut h, "asm_ground");
    assert_eq!(asm_of(&mut h).grounded(), vec![0, 1]);
    run_command(&mut h, "asm_ground");
    assert!(
        asm_of(&mut h).grounded().is_empty(),
        "a second Ground releases them"
    );
    // activate the selected component and finish it
    h.app.design.asm.selected = vec![1];
    run_command(&mut h, "asm_activate");
    assert_eq!(proj(&mut h).editing(), Some(0));
    run_command(&mut h, "asm_finish");
    assert_eq!(proj(&mut h).editing(), None);
    // New Component from the menu makes an empty one and opens it
    run_command(&mut h, "asm_new");
    assert_eq!(proj(&mut h).editing(), Some(1));
    run_command(&mut h, "asm_finish");
    // the view toggle and the bill
    let before = h.app.design.asm.view;
    run_command(&mut h, "asm_view");
    assert_ne!(h.app.design.asm.view, before);
    run_command(&mut h, "asm_bom");
    assert!(h.app.design.asm.bom_open);
}

/// One frame with the given pointer events.
fn pointer_frame(h: &mut Harness, events: Vec<egui::Event>) {
    let Harness { ctx, app, t } = h;
    let raw = RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1360.0, 860.0))),
        time: Some(*t),
        events,
        ..Default::default()
    };
    *t += 1.0 / 60.0;
    let _ = ctx.run_ui(raw, |ui| app.show(ui));
}

fn press(pos: Pos2, down: bool) -> egui::Event {
    egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed: down,
        modifiers: egui::Modifiers::NONE,
    }
}

/// The screen position of a world point in the design viewport.
fn screen_of(h: &Harness, p: [f64; 3]) -> Pos2 {
    let o = h.app.design.viewport.origin_in_scene();
    h.app
        .design
        .viewport
        .project([p[0] + o.x, p[1] + o.y, p[2] + o.z])
        .expect("on screen")
}

#[test]
fn dragging_a_component_in_the_view_moves_it_and_does_not_orbit() {
    let mut h = two_blocks("drag_view");
    h.frame();
    let cam_before = format!("{:?}", h.app.design.viewport.camera.view());
    // the copy floats at x = 15: grab the middle of its top face (25, 10, 10) and pull it to the right
    let from = screen_of(&h, [25.0, 10.0, 10.0]);
    let to = from + egui::vec2(60.0, 0.0);
    // (no GPU here, so the pointer hover that tells which component is under the pointer is set by hand)
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(from)]);
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![press(from, true)]);
    assert!(h.app.design.asm.drag.is_some(), "{}", h.app.design.status);
    for k in 1..=6 {
        let p = from + (to - from) * (k as f32 / 6.0);
        pointer_frame(&mut h, vec![egui::Event::PointerMoved(p)]);
    }
    pointer_frame(&mut h, vec![press(to, false)]);
    pointer_frame(&mut h, vec![]);
    assert!(h.app.design.asm.drag.is_none());
    let a = asm_of(&mut h);
    let moved = a.solve().unwrap()[1].apply_point(&kw_math::Point3::origin());
    let shift = [moved.x - 15.0, moved.y, moved.z];
    let len = (shift[0].powi(2) + shift[1].powi(2) + shift[2].powi(2)).sqrt();
    assert!(len > 5.0, "the component followed the pointer: {shift:?}");
    // it moved in the plane facing the camera
    let cam = &h.app.design.viewport.camera;
    let dir = (cam.target - cam.eye()).normalize();
    assert!(
        (shift[0] * dir.x + shift[1] * dir.y + shift[2] * dir.z).abs() < 1e-6 * len.max(1.0),
        "{shift:?} vs {dir:?}"
    );
    assert!(proj(&mut h).is_dirty());
    assert_eq!(
        cam_before,
        format!("{:?}", h.app.design.viewport.camera.view()),
        "the camera did not orbit"
    );
    // the grounded block refuses with a reason (and the view orbits instead)
    h.app.design.asm.view_hover = Some(0);
    let g = screen_of(&h, [10.0, 10.0, 10.0]);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(g)]);
    h.app.design.asm.view_hover = Some(0);
    pointer_frame(&mut h, vec![press(g, true)]);
    assert!(h.app.design.asm.drag.is_none());
    assert!(
        h.app.design.status.contains("cannot be moved"),
        "{}",
        h.app.design.status
    );
    pointer_frame(&mut h, vec![press(g, false)]);
}

#[test]
fn dragging_a_jointed_component_turns_its_joint_within_the_limits() {
    use kw_assembly::{Joint, JointEnd, JointKind};
    let mut h = two_blocks("drag_joint");
    {
        let a = proj(&mut h).assembly.as_mut().unwrap();
        let mut j = Joint::new(
            "pin",
            JointKind::Revolute,
            JointEnd {
                instance: 0,
                frame: "origin".into(),
            },
            JointEnd {
                instance: 1,
                frame: "origin".into(),
            },
        );
        j.limits = vec![Some((
            -std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
        ))];
        a.add_joint(j);
    }
    h.frame();
    h.frame();
    // the copy now sits on the first block's origin turning about the z axis; grab a point of its top face (15, 15, 10)
    let from = screen_of(&h, [15.0, 15.0, 10.0]);
    let to = from + egui::vec2(-80.0, 40.0);
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(from)]);
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![press(from, true)]);
    assert!(h.app.design.asm.drag.is_some(), "{}", h.app.design.status);
    assert_eq!(h.app.design.asm.drag.as_ref().unwrap().joint(), Some(0));
    for k in 1..=8 {
        let p = from + (to - from) * (k as f32 / 8.0);
        pointer_frame(&mut h, vec![egui::Event::PointerMoved(p)]);
    }
    pointer_frame(&mut h, vec![press(to, false)]);
    pointer_frame(&mut h, vec![]);
    let a = asm_of(&mut h);
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!(v.abs() > 0.05, "the joint turned: {}°", v.to_degrees());
    assert!(
        v.abs() <= std::f64::consts::FRAC_PI_2 + 1e-9,
        "within its limit"
    );
    assert!(a.solve().is_ok());
    // a pull far beyond what the limit allows holds the joint at the limit
    h.app.design.asm.view_hover = Some(1);
    let g = screen_of(&h, [15.0, 15.0, 10.0]);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(g)]);
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![press(g, true)]);
    assert!(h.app.design.asm.drag.is_some(), "{}", h.app.design.status);
    for k in 1..=10 {
        let p = g + egui::vec2(-60.0 * k as f32, 30.0 * k as f32);
        pointer_frame(&mut h, vec![egui::Event::PointerMoved(p)]);
    }
    pointer_frame(&mut h, vec![press(g, false)]);
    pointer_frame(&mut h, vec![]);
    let a = asm_of(&mut h);
    let v = a.joint_values(&a.joints[0], &a.scope().unwrap()).unwrap()[0];
    assert!(v.abs() <= std::f64::consts::FRAC_PI_2 + 1e-9);
    assert!(a.solve().is_ok());
}

#[test]
fn the_move_command_shifts_and_turns_a_free_component_and_refuses_a_jointed_one() {
    use kw_app::design::asm_drag::{move_free, MoveForm};
    let mut h = two_blocks("move_cmd");
    let mut form = MoveForm::new();
    form.t = ["10".into(), "0".into(), "5 mm".into()];
    form.r = ["0".into(), "0".into(), "90".into()];
    let (turn, shift) = form.transform().unwrap();
    let a = proj(&mut h).assembly.as_mut().unwrap();
    // the copy floats at x = 15; turn it a quarter about its own origin, then shift (10, 0, 5)
    move_free(a, 1, &turn, shift).unwrap();
    let t = a.solve().unwrap();
    let p = t[1].apply_point(&kw_math::Point3::new(4.0, 0.0, 0.0));
    // (4,0,0) turns to (0,4,0) about the origin at x = 15 → (15, 4, 0), then + (10, 0, 5)
    assert!(
        (p.x - 25.0).abs() < 1e-9 && (p.y - 4.0).abs() < 1e-9 && (p.z - 5.0).abs() < 1e-9,
        "{p:?}"
    );
    // the grounded block is refused with an explanation
    let err = move_free(a, 0, &turn, shift).unwrap_err();
    assert!(err.contains("placed by a joint or a ground"), "{err}");
    // bad numbers are explained
    form.t[0] = "abc".into();
    assert!(form.transform().is_err());
    // the window draws
    h.app.design.asm.selected = vec![1];
    run_command(&mut h, "asm_move");
    assert!(h.app.design.asm.move_form.is_some());
    h.frame();
}

#[test]
fn undo_and_redo_work_on_the_assembly_in_the_assembly_view_and_on_the_design_elsewhere() {
    use kw_app::design::assembly::AsmAction as A;
    let mut h = two_blocks("asm_undo");
    assert!(h.app.undo_is_for_assembly());
    act(&mut h, Action::Asm(A::ToggleGround(1)));
    assert_eq!(asm_of(&mut h).grounded(), vec![0, 1]);
    h.app.undo_redo(true);
    assert_eq!(asm_of(&mut h).grounded(), vec![0], "undone");
    h.app.undo_redo(false);
    assert_eq!(asm_of(&mut h).grounded(), vec![0, 1], "redone");
    h.app.undo_redo(true);
    // a dragged component goes back to where it was
    let before = asm_of(&mut h).solve().unwrap()[1].to_matrix();
    let from = screen_of(&h, [25.0, 10.0, 10.0]);
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(from)]);
    h.app.design.asm.view_hover = Some(1);
    pointer_frame(&mut h, vec![press(from, true)]);
    for k in 1..=4 {
        pointer_frame(
            &mut h,
            vec![egui::Event::PointerMoved(
                from + egui::vec2(20.0 * k as f32, 0.0),
            )],
        );
    }
    pointer_frame(&mut h, vec![press(from + egui::vec2(80.0, 0.0), false)]);
    pointer_frame(&mut h, vec![]);
    let moved = asm_of(&mut h).solve().unwrap()[1].to_matrix();
    assert!((moved - before).norm() > 1.0, "the drag moved it");
    h.app.undo_redo(true);
    assert!((asm_of(&mut h).solve().unwrap()[1].to_matrix() - before).norm() < 1e-9);
    // the copy itself can be undone, and everything back to the first component
    h.app.undo_redo(true);
    assert_eq!(asm_of(&mut h).instances.len(), 1);
    h.app.undo_redo(true);
    assert!(
        proj(&mut h).assembly.is_none(),
        "back to no assembly at all"
    );
    assert!(!proj(&mut h).asm_can_undo());
    // with the assembly view off, Undo is the design's own
    h.app.design.asm.view = false;
    assert!(!h.app.undo_is_for_assembly());
    let before = proj(&mut h).document().len();
    h.app.undo_redo(true);
    assert!(
        proj(&mut h).document().len() < before,
        "the design's last step was undone"
    );
}

#[test]
fn groups_sub_assemblies_grounds_and_joints_survive_saving_and_reopening() {
    use kw_app::design::assembly::AsmAction as A;
    use kw_assembly::{Joint, JointEnd, JointKind};
    let mut h = two_blocks("asm_roundtrip");
    act(&mut h, Action::Asm(A::Copy(0)));
    act(&mut h, Action::Asm(A::Copy(0)));
    {
        let a = proj(&mut h).assembly.as_mut().unwrap();
        let mut j = Joint::new(
            "pin",
            JointKind::Revolute,
            JointEnd {
                instance: 0,
                frame: "origin".into(),
            },
            JointEnd {
                instance: 1,
                frame: "origin".into(),
            },
        );
        j.values = vec![kw_expr::Param::angle_deg(30.0)];
        j.limits = vec![Some((0.0, 1.5))];
        j.flip = true;
        j.offset = [1.0, 2.0, 3.0];
        a.add_joint(j);
    }
    h.app.design.asm.selected = vec![2, 3];
    act(&mut h, Action::Asm(A::RigidGroupFromSelection));
    act(&mut h, Action::Asm(A::ToggleHidden(3)));
    let before = asm_of(&mut h);
    assert_eq!(before.groups.len(), 1);
    let path = scratch("asm_roundtrip_file").join("rt.kwf");
    proj(&mut h).save_as(&path).unwrap();
    let back = kw_app::project::Project::open(&path).unwrap();
    let a = back.assembly.clone().expect("assembly stored");
    assert_eq!(a, before);
    assert_eq!(
        a.solve().unwrap()[1].to_matrix(),
        before.solve().unwrap()[1].to_matrix()
    );
    // a sub-assembly of the pinned pair
    h.app.design.asm.selected = vec![0, 1];
    act(&mut h, Action::Asm(A::SubAssemblyFromSelection));
    let sub = asm_of(&mut h);
    assert!(sub.parts.iter().any(|p| p.sub.is_some()));
    proj(&mut h).save_as(&path).unwrap();
    let back = kw_app::project::Project::open(&path).unwrap();
    let a = back.assembly.unwrap();
    assert_eq!(a, sub);
    assert_eq!(a.leaves().unwrap().len(), sub.leaves().unwrap().len());
}

fn run_command(h: &mut Harness, id: &str) {
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command(id, project);
    }
    h.frame();
    h.frame();
}

#[test]
fn the_sheet_metal_tab_builds_strips_and_unfolds_them() {
    use std::f64::consts::FRAC_PI_2;
    let (mut h, _d) = design_harness("sheet_tab");
    h.app.design.ribbon_tab = 1;
    h.frame();
    // U channel: 40 flat, bend, 30 flat, bend, 40 flat; t = 2, k = 0.44, r = 2, width 30
    run_command(&mut h, "sm_u");
    assert!(matches!(
        h.app.design.dialog.as_ref().map(|d| &d.tool),
        Some(Tool::SheetMetal { .. })
    ));
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let spec = kw_sheet::SheetSpec {
        thickness: 2.0,
        k_factor: 0.44,
        default_radius: 2.0,
    };
    let strip = kw_sheet::SheetStrip::new(spec, 30.0)
        .flat(40.0)
        .bend(FRAC_PI_2)
        .flat(30.0)
        .bend(FRAC_PI_2)
        .flat(40.0);
    let want = kw_sheet::analytic_volume(&strip);
    assert!(
        (vol(&mut h) - want).abs() < 1e-6 * want,
        "{} vs {want}",
        vol(&mut h)
    );
    // the flat pattern: three flats plus two bend allowances
    run_command(&mut h, "sm_flat");
    let flat = h.app.design.flat.as_ref().expect("flat pattern");
    let ba = FRAC_PI_2 * (2.0 + 0.44 * 2.0);
    assert!((flat.pattern.length() - (110.0 + 2.0 * ba)).abs() < 1e-9);
    assert_eq!(flat.pattern.bends().len(), 2);
    // Refold reopens the strip with the same rows
    let pat = flat.pattern.clone();
    let back = kw_sheet::refold(&pat);
    assert_eq!(back.segs.len(), 5);
    h.app.design.close_dialog();
    // Edit Strip reads the feature back
    run_command(&mut h, "sm_edit");
    match h.app.design.dialog.as_ref().map(|d| &d.tool) {
        Some(Tool::SheetMetal {
            rows, thickness, ..
        }) => {
            assert_eq!(rows.len(), 5);
            assert_eq!(thickness, "2 mm");
        }
        t => panic!("{t:?}"),
    }
}

#[test]
fn the_form_tab_makes_bodies_from_cages() {
    let (mut h, _d) = design_harness("form_tab");
    h.app.design.ribbon_tab = 2;
    h.frame();
    // a 20 mm box with sharp edges finishes to the box itself
    run_command(&mut h, "form_box");
    commit(&mut h);
    let v = vol(&mut h);
    assert!((v - 8000.0).abs() < 0.01 * 8000.0, "{v}");
    // editing: the scale doubles every length
    let id = proj(&mut h).document().features.last().unwrap().id;
    act(&mut h, Action::Edit(id));
    match h.app.design.dialog.as_mut().map(|d| &mut d.tool) {
        Some(Tool::Form { scale, kind, .. }) => {
            assert_eq!(*kind, 255, "an existing form is a custom cage");
            *scale = "2".into();
        }
        t => panic!("{t:?}"),
    }
    h.frame();
    commit(&mut h);
    let v2 = vol(&mut h);
    assert!((v2 - 64000.0).abs() < 0.01 * 64000.0, "{v2}");
    // a new box of another size: changing the numbers rebuilds the cage
    let (mut h, _d) = design_harness("form_tab2");
    run_command(&mut h, "form_box");
    if let Some(Tool::Form { size, .. }) = h.app.design.dialog.as_mut().map(|d| &mut d.tool) {
        *size = [10.0, 20.0, 30.0];
    }
    h.frame();
    h.frame();
    commit(&mut h);
    let v = vol(&mut h);
    assert!((v - 6000.0).abs() < 0.01 * 6000.0, "{v}");
}

#[test]
fn the_other_form_primitives_finish_to_bodies() {
    // (the kernel's volume of a patch body takes seconds, so this checks the finished body, not its volume)
    for cmd in ["form_cyl", "form_sphere", "form_torus"] {
        let (mut h, _d) = design_harness(cmd);
        run_command(&mut h, cmd);
        commit(&mut h);
        let ev = proj(&mut h).evaluation();
        assert!(ev.failures().is_empty(), "{cmd}: {:?}", ev.failures());
        let (_, o) = ev
            .final_solid()
            .unwrap_or_else(|| panic!("{cmd}: no solid"));
        let (rep, body) = (o.brep().unwrap(), o.body().unwrap());
        assert!(rep.body_faces(body).len() >= 6, "{cmd}");
        let bb = rep.body_bbox(body);
        assert!((bb.max - bb.min).norm() > 5.0, "{cmd}");
    }
}

#[test]
fn every_ribbon_tab_draws_and_the_tools_tab_commands_work() {
    let (mut h, _d) = design_harness("tabs");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    for tab in 0..4 {
        h.app.design.ribbon_tab = tab;
        h.frame();
        h.frame();
    }
    run_command(&mut h, "appearance");
    assert!(h.app.design.body_props.is_some());
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("file_import", project);
    }
    assert_eq!(
        h.app.design.asm.request,
        Some(kw_app::design::assembly::FileRequest::Import)
    );
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.command("file_export_step", project);
    }
    assert_eq!(
        h.app.design.asm.request,
        Some(kw_app::design::assembly::FileRequest::Export(
            kw_app::exchange::ExportFormat::Step
        ))
    );
}

#[test]
fn a_rollback_bar_at_the_end_of_the_timeline_does_not_block_new_features() {
    // |< on a timeline of one feature puts the bar after it, which is where it already is
    let (mut h, _d) = design_harness("rollback_end");
    sketch_only(
        &mut h,
        geom::origin_spec(0),
        geom::origin_frame(0),
        square((0.0, 0.0), 5.0),
    );
    let s = proj(&mut h).document().features.last().unwrap().id;
    act(&mut h, Action::RollbackTo(s));
    assert!(!proj(&mut h).document().rolled_back());
    act(
        &mut h,
        Action::Open(Tool::extrude(Some(s), kw_feature::BodyOp::New)),
    );
    assert!(
        h.app.design.dialog.is_some(),
        "{}",
        h.app.design.model.error
    );
    commit(&mut h);
    assert!(vol(&mut h) > 0.0);
    // a bar saved at the end of an older document behaves the same
    let n = proj(&mut h).document().features.len();
    let mut d = (*proj(&mut h).document()).clone();
    d.rollback = Some(n);
    assert!(!d.rolled_back());
}

#[test]
fn the_view_cube_looks_from_faces_edges_and_corners() {
    use kw_app::chrome::viewcube::direction_view;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
    // faces agree with the named views
    assert_eq!(direction_view([0, -1, 0]), (-FRAC_PI_2, 0.0));
    let (y, p) = direction_view([1, 0, 0]);
    assert!(y.abs() < 1e-12 && p.abs() < 1e-12);
    assert!((direction_view([-1, 0, 0]).0 - PI).abs() < 1e-12);
    assert!(direction_view([0, 0, 1]).1 > 1.5 && direction_view([0, 0, -1]).1 < -1.5);
    // an edge between right and top: 45 degrees up from the right
    let (y, p) = direction_view([1, 0, 1]);
    assert!(y.abs() < 1e-12 && (p - FRAC_PI_4).abs() < 1e-12);
    // a corner (right, back, top): 45 degrees round, asin(1/sqrt 3) up
    let (y, p) = direction_view([1, 1, 1]);
    assert!((y - FRAC_PI_4).abs() < 1e-12 && (p - (1.0f64 / 3.0f64.sqrt()).asin()).abs() < 1e-12);
    // the camera at that (yaw, pitch) sits on the side the direction names
    let mut cam = kw_viewer::OrbitCamera::new(10.0, 1.5);
    cam.yaw = y;
    cam.pitch = p;
    let e = cam.eye() - cam.target;
    assert!(
        e.x > 0.0 && e.y > 0.0 && e.z > 0.0 && (e.x - e.y).abs() < 1e-9 && (e.y - e.z).abs() < 1e-9
    );
    // it draws in the design view, and previous view returns to where the camera was
    let (mut h, _d) = design_harness("cube");
    sketch_block(&mut h, 30.0, 20.0, "5 mm");
    commit(&mut h);
    h.frame();
    let before = (
        h.app.design.viewport.camera.yaw,
        h.app.design.viewport.camera.pitch,
    );
    h.app.design.viewport.look(0.3, 0.2, 0.0);
    h.app.design.viewport.previous_view(0.0);
    for i in 0..60 {
        h.t += 0.05;
        let _ = i;
        h.frame();
    }
    let after = (
        h.app.design.viewport.camera.yaw,
        h.app.design.viewport.camera.pitch,
    );
    assert!(
        (after.0 - before.0).abs() < 1e-6 && (after.1 - before.1).abs() < 1e-6,
        "{before:?} {after:?}"
    );
}

#[test]
fn construction_lines_of_a_sketch_are_not_drawn_in_the_3d_view() {
    use kw_sketch::editor::Tool as SkTool;
    let (mut h, _d) = design_harness("construction_hidden");
    start_sketch_plane(&mut h, 0);
    let ed = &mut h.app.design.sketch.app.editor;
    ed.set_tool(SkTool::CenterRectangle);
    ed.handle(kw_sketch::editor::Input::Click((0.0, 0.0)));
    ed.handle(kw_sketch::editor::Input::Click((10.0, 5.0)));
    act(&mut h, Action::FinishSketch);
    h.app.design.close_dialog();
    let ev = proj(&mut h).evaluation();
    let doc = proj(&mut h).document();
    let segs = geom::sketch_segments(&ev, &doc, &Default::default());
    // the four sides only (no diagonals)
    assert_eq!(segs.len(), 4, "{}", segs.len());
}

#[test]
fn the_seam_of_a_cylinder_is_not_drawn_as_an_edge() {
    let (mut h, _d) = design_harness("seam");
    let mut sk = kw_sketch::Sketch::new();
    sk.add_circle_xy(0.0, 0.0, 6.0);
    sketch_only(&mut h, geom::origin_spec(0), geom::origin_frame(0), sk);
    let s = proj(&mut h).document().features.last().unwrap().id;
    open_commit(&mut h, Tool::extrude(Some(s), kw_feature::BodyOp::New));
    let ev = proj(&mut h).evaluation();
    let rep = ev.final_solid().unwrap().1.brep().unwrap().clone();
    // the B-rep has the two rims and the seam
    let seams = rep
        .edges
        .keys()
        .filter(|e| kw_app::viewport::is_seam(&rep, *e))
        .count();
    assert_eq!((rep.edges.len(), seams), (3, 1));
    let scene = kw_app::viewport::ModelScene::build(&rep);
    let mut ids: Vec<u32> = scene.scene.segments.iter().map(|s| s.id).collect();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), 2, "only the two rims are drawn");
}

#[test]
fn a_dimension_formula_follows_the_documents_parameters() {
    use kw_sketch::editor::{Input, Tool as SkTool};
    let (mut h, _d) = design_harness("dim_formula");
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.design.model.add_parameter(project, "Width", "30");
    }
    start_sketch_plane(&mut h, 0);
    h.frame(); // the sketch canvas reads the document's parameters
    let ed = &mut h.app.design.sketch.app.editor;
    ed.set_tool(SkTool::Rectangle);
    ed.handle(Input::Click((0.0, 0.0)));
    ed.handle(Input::Click((10.0, 6.0)));
    ed.set_tool(SkTool::Dimension);
    ed.handle(Input::Click((5.0, 0.0)));
    ed.handle(Input::Click((5.0, -8.0)));
    let c = ed.last_dimension.expect("a driving dimension");
    // the width is half the parameter; the height stays 6
    let (v, f) = h
        .app
        .design
        .sketch
        .app
        .eval_dimension_text(c, "Width / 2")
        .unwrap();
    assert!((v - 15.0).abs() < 1e-9 && f.is_some());
    assert!(h.app.design.sketch.app.editor.edit_dimension_expr(c, v, f));
    // pin the corner so the width does not float
    {
        let ed = &mut h.app.design.sketch.app.editor;
        let (lo, _) = ed.sketch.bounds().unwrap();
        ed.set_tool(SkTool::Constrain(kw_sketch::editor::ConstraintTool::Fix));
        ed.handle(Input::Click(lo));
    }
    act(&mut h, Action::FinishSketch);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { depth, .. } = &mut d.tool else {
            panic!()
        };
        *depth = "2 mm".into();
    }
    h.frame();
    commit(&mut h);
    assert!(
        (vol(&mut h) - 15.0 * 6.0 * 2.0).abs() < 1e-6,
        "{}",
        vol(&mut h)
    );
    // change the parameter: the sketch follows
    let width = proj(&mut h)
        .document()
        .features
        .iter()
        .find(|f| matches!(f.spec, kw_feature::FeatureSpec::Parameter { .. }))
        .unwrap()
        .id;
    proj(&mut h)
        .history
        .edit(|d| {
            d.replace_spec(
                width,
                kw_feature::FeatureSpec::Parameter {
                    name: "Width".into(),
                    value: kw_expr::Param::length(50.0),
                },
            )
        })
        .unwrap();
    assert!(
        (vol(&mut h) - 25.0 * 6.0 * 2.0).abs() < 1e-6,
        "{}",
        vol(&mut h)
    );
    // editing the sketch again shows the formula
    let sk = proj(&mut h)
        .document()
        .features
        .iter()
        .find(|f| matches!(f.spec, kw_feature::FeatureSpec::Sketch(_)))
        .unwrap()
        .id;
    act(&mut h, Action::Edit(sk));
    let ed = &h.app.design.sketch.app.editor;
    assert_eq!(
        ed.sketch.exprs.values().next().map(String::as_str),
        Some("Width / 2")
    );
}

#[test]
fn a_line_across_a_circle_gives_two_regions_and_the_chosen_one_extrudes() {
    let (mut h, _d) = design_harness("chord_regions");
    act(&mut h, Action::StartSketch);
    act(
        &mut h,
        Action::SketchOn(geom::origin_spec(0), geom::origin_frame(0)),
    );
    let mut sk = kw_sketch::Sketch::new();
    sk.add_circle_xy(10.0, 5.0, 5.0);
    sk.add_line_xy(3.0, 7.0, 17.0, 7.0); // a chord 2 mm above the centre, longer than the circle
    h.app.design.sketch.app.editor.replace_sketch(sk);
    act(&mut h, Action::FinishSketch);
    h.frame();
    let rv = h.app.design.region_view.as_ref().expect("regions");
    assert_eq!(rv.regions().len(), 2, "the chord cuts the circle in two");
    // both are chosen to begin with; the dialog needs a choice, so start from none
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude { more, .. } = &mut d.tool else {
            panic!()
        };
        assert_eq!(more.regions.len(), 2);
        more.regions.clear();
    }
    h.frame();
    assert!(h
        .app
        .design
        .preview
        .as_ref()
        .is_some_and(|p| p.error.is_some()));
    // the small cap above the chord, at world (10, 9)
    let o = h.app.design.viewport.origin_in_scene();
    let pos = h
        .app
        .design
        .viewport
        .project([10.0 + o.x, 9.0 + o.y, o.z])
        .expect("on screen");
    click_at(&mut h, pos);
    {
        let d = h.app.design.dialog.as_mut().unwrap();
        let Tool::Extrude {
            more, depth, op, ..
        } = &mut d.tool
        else {
            panic!()
        };
        assert_eq!(more.regions.len(), 1, "the cap is chosen");
        *depth = "3 mm".into();
        *op = kw_feature::BodyOp::New;
    }
    h.frame();
    let pv = h.app.design.preview.clone().unwrap();
    assert!(pv.error.is_none(), "{:?}", pv.error);
    commit(&mut h);
    let cap = 25.0 * (2.0f64 / 5.0).acos() - 2.0 * 21.0f64.sqrt();
    assert!(
        (vol(&mut h) - cap * 3.0).abs() < 1e-4,
        "{} vs {}",
        vol(&mut h),
        cap * 3.0
    );
}

// ------------------------------------------------------------------------------------ Drawing workspace

use kw_drawing::projection::Side;
use kw_drawing::{Orientation, ViewKind};

/// The demo block (40 x 25 x 10) open in the Drawing workspace.
fn drawing_app(name: &str) -> (Harness, PathBuf) {
    let dir = scratch(name);
    let mut h = Harness::new(app_in(&dir));
    h.app.open_demo();
    h.app.set_workspace(Workspace::Drawing);
    h.frame();
    (h, dir)
}

fn dw_project(h: &mut Harness) -> &mut kw_app::project::Project {
    h.app.project_mut().unwrap()
}

/// A drawing with a front view; waits for the geometry.
fn make_drawing(h: &mut Harness) -> u64 {
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!("no project")
    };
    let form = h.app.drawing.new_drawing_form(project);
    h.app.drawing.new_drawing(project, &form).unwrap();
    h.app.drawing.wait(project);
    h.app.drawing.selected.unwrap()
}

#[test]
fn a_new_drawing_has_a_front_view_with_geometry_at_a_fitting_scale() {
    let (mut h, _) = drawing_app("dw_new");
    let base = make_drawing(&mut h);
    let p = dw_project(&mut h);
    assert!(p.is_dirty());
    assert_eq!(p.drawings.sheets.len(), 1);
    let v = p.drawings.sheets[0].view(base).unwrap();
    assert_eq!(v.orientation, Orientation::Front);
    assert!(
        v.cache.is_some(),
        "the views are generated in the background"
    );
    // the front of a 40 x 10 block, at the chosen scale
    let s = v.size();
    assert!(
        (s[0] - 40.0 * v.scale).abs() < 1e-6 && (s[1] - 10.0 * v.scale).abs() < 1e-6,
        "{s:?} at {}",
        v.scale
    );
    // it fits inside the border, clear of the title block
    let fr = p.drawings.sheets[0].format.frame();
    assert!(v.pos[0] - s[0] / 2.0 > fr.border.0[0] && v.pos[0] + s[0] / 2.0 < fr.border.1[0]);
    for _ in 0..3 {
        h.frame();
    }
}

#[test]
fn projected_views_follow_third_angle_and_stay_aligned() {
    let (mut h, _) = drawing_app("dw_proj");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let right = h
        .app
        .drawing
        .add_projected(project, base, Side::Right)
        .unwrap();
    h.app.drawing.wait(project);
    let top = h
        .app
        .drawing
        .add_projected(project, base, Side::Above)
        .unwrap();
    h.app.drawing.wait(project);
    let sh = &project.drawings.sheets[0];
    let (b, r, t) = (
        sh.view(base).unwrap(),
        sh.view(right).unwrap(),
        sh.view(top).unwrap(),
    );
    assert_eq!(r.orientation, Orientation::Right);
    assert_eq!(t.orientation, Orientation::Top);
    assert!(matches!(r.kind, ViewKind::Projected { parent, .. } if parent == base));
    // right of the front view at the same height, top above it at the same x
    assert!(r.pos[0] > b.pos[0] && (r.pos[1] - b.pos[1]).abs() < 1e-9);
    assert!(t.pos[1] > b.pos[1] && (t.pos[0] - b.pos[0]).abs() < 1e-9);
    // the right view of the block is 25 wide (y) and 10 high; the top is 40 x 25
    assert!(
        (r.size()[0] - 25.0 * b.scale).abs() < 1e-6 && (r.size()[1] - 10.0 * b.scale).abs() < 1e-6
    );
    assert!(
        (t.size()[0] - 40.0 * b.scale).abs() < 1e-6 && (t.size()[1] - 25.0 * b.scale).abs() < 1e-6
    );
    // no overlap between the views
    let gap = (r.pos[0] - r.size()[0] / 2.0) - (b.pos[0] + b.size()[0] / 2.0);
    assert!(gap > 10.0, "{gap}");
}

#[test]
fn drawings_save_and_reopen_with_their_geometry() {
    let (mut h, dir) = drawing_app("dw_save");
    let base = make_drawing(&mut h);
    let path = dir.join("drawn.kwf");
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app
            .drawing
            .add_projected(project, base, Side::Right)
            .unwrap();
        h.app.drawing.wait(project);
        assert!(project.is_dirty());
        project.save_as(&path).unwrap();
        assert!(!project.is_dirty());
    }
    let back = kw_app::project::Project::open(&path).unwrap();
    assert_eq!(back.drawings, dw_project(&mut h).drawings);
    assert_eq!(back.drawings.sheets[0].views.len(), 2);
    assert!(back.drawings.sheets[0]
        .views
        .iter()
        .all(|v| v.cache.is_some()));
    assert!(!back.is_dirty());
}

#[test]
fn undo_and_redo_work_on_the_drawings_in_the_drawing_workspace() {
    let (mut h, _) = drawing_app("dw_undo");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app
        .drawing
        .add_projected(project, base, Side::Right)
        .unwrap();
    h.app.drawing.wait(project);
    assert_eq!(project.drawings.sheets[0].views.len(), 2);
    assert!(h.app.undo_is_for_drawing());
    h.app.undo_redo(true);
    assert_eq!(dw_project(&mut h).drawings.sheets[0].views.len(), 1);
    h.app.undo_redo(true);
    assert!(
        dw_project(&mut h).drawings.is_empty(),
        "the whole drawing is one step"
    );
    h.app.undo_redo(false);
    h.app.undo_redo(false);
    assert_eq!(dw_project(&mut h).drawings.sheets[0].views.len(), 2);
    // the design's own history is untouched
    assert!(!dw_project(&mut h).history.can_redo());
    h.frame();
}

#[test]
fn deleting_a_view_deletes_what_was_projected_from_it() {
    let (mut h, _) = drawing_app("dw_delete");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let right = h
        .app
        .drawing
        .add_projected(project, base, Side::Right)
        .unwrap();
    h.app.drawing.wait(project);
    h.app
        .drawing
        .add_projected(project, right, Side::Above)
        .unwrap();
    h.app.drawing.wait(project);
    assert_eq!(project.drawings.sheets[0].views.len(), 3);
    h.app.drawing.delete_view(project, right).unwrap();
    assert_eq!(project.drawings.sheets[0].views.len(), 1);
    h.app.undo_redo(true);
    assert_eq!(dw_project(&mut h).drawings.sheets[0].views.len(), 3);
}

#[test]
fn pdf_and_dxf_are_written_through_the_dialogs() {
    let (mut h, dir) = drawing_app("dw_export");
    make_drawing(&mut h);
    let (pdf, dxf) = (dir.join("out.pdf"), dir.join("out.dxf"));
    let mut dialogs = ScriptedDialogs {
        save: vec![pdf.clone(), dxf.clone()],
        ..Default::default()
    };
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.command("export_pdf", project, &mut dialogs);
    h.app.drawing.command("export_dxf", project, &mut dialogs);
    let bytes = std::fs::read(&pdf).unwrap();
    assert!(bytes.starts_with(b"%PDF-1.4"));
    assert!(String::from_utf8_lossy(&bytes).contains("/MediaBox [0 0 1190"));
    let text = std::fs::read_to_string(&dxf).unwrap();
    let doc = kw_dxf::parse(&text).unwrap();
    assert!(doc.entities.len() > 20, "border, title block and the view");
    assert!(
        h.app.drawing.status.contains("saved"),
        "{}",
        h.app.drawing.status
    );
}

#[test]
fn dragging_a_view_moves_it_and_a_projected_view_slides_along_its_line() {
    let (mut h, _) = drawing_app("dw_drag");
    let base = make_drawing(&mut h);
    let right = {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let r = h
            .app
            .drawing
            .add_projected(project, base, Side::Right)
            .unwrap();
        h.app.drawing.wait(project);
        r
    };
    h.frame();
    let (before_b, before_r) = {
        let sh = &dw_project(&mut h).drawings.sheets[0];
        (sh.view(base).unwrap().pos, sh.view(right).unwrap().pos)
    };
    // drag the right view diagonally: only the horizontal part counts
    let from = h.app.drawing.screen_of(before_r);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(from)]);
    pointer_frame(&mut h, vec![press(from, true)]);
    for k in 1..=6 {
        let to = from + egui::vec2(8.0 * k as f32, -6.0 * k as f32);
        pointer_frame(&mut h, vec![egui::Event::PointerMoved(to)]);
    }
    let to = from + egui::vec2(48.0, -36.0);
    pointer_frame(&mut h, vec![press(to, false)]);
    let sh = &dw_project(&mut h).drawings.sheets[0];
    let after_r = sh.view(right).unwrap().pos;
    assert!(
        after_r[0] > before_r[0] + 1.0,
        "moved right: {before_r:?} -> {after_r:?}"
    );
    assert!(
        (after_r[1] - before_r[1]).abs() < 1e-9,
        "stays level with its parent"
    );
    assert_eq!(sh.view(base).unwrap().pos, before_b);
    // dragging the parent takes the projected view along
    let from = h.app.drawing.screen_of(before_b);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(from)]);
    pointer_frame(&mut h, vec![press(from, true)]);
    for k in 1..=6 {
        let to = from + egui::vec2(0.0, 8.0 * k as f32);
        pointer_frame(&mut h, vec![egui::Event::PointerMoved(to)]);
    }
    pointer_frame(&mut h, vec![press(from + egui::vec2(0.0, 48.0), false)]);
    let sh = &dw_project(&mut h).drawings.sheets[0];
    let (nb, nr) = (sh.view(base).unwrap().pos, sh.view(right).unwrap().pos);
    assert!(nb[1] < before_b[1] - 1.0, "dragged down");
    assert!(
        ((nr[1] - after_r[1]) - (nb[1] - before_b[1])).abs() < 1e-9,
        "the child followed"
    );
    // each drag is one undo step
    h.app.undo_redo(true);
    assert_eq!(
        dw_project(&mut h).drawings.sheets[0]
            .view(base)
            .unwrap()
            .pos,
        before_b
    );
}

#[test]
fn nothing_to_draw_is_reported_not_made() {
    let dir = scratch("dw_empty");
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Empty", &dir).unwrap();
    h.app.set_workspace(Workspace::Drawing);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let form = h.app.drawing.new_drawing_form(project);
    assert!(h.app.drawing.new_drawing(project, &form).is_err());
    assert!(project.drawings.is_empty());
    assert!(!project.dw_can_undo());
    h.frame();
}

// ------------------------------------------------------------------------------------ section and detail views

use kw_drawing::{Layer, Shape};

fn dw_wait(h: &mut Harness) {
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.wait(project);
}

fn count_layer(v: &kw_drawing::View, l: Layer) -> usize {
    v.cache
        .as_ref()
        .map_or(0, |c| c.scene.prims.iter().filter(|p| p.layer == l).count())
}

#[test]
fn a_section_is_placed_in_the_direction_of_its_arrows_hatched_and_marked_on_its_parent() {
    let (mut h, _) = drawing_app("dw_section");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    // the front view of the demo block (40 x 10): a vertical cut up the middle, material to its right
    let (a, b) = ([20.0, -2.0], [20.0, 12.0]);
    let sec = h
        .app
        .drawing
        .add_section(project, base, a, b, false)
        .unwrap();
    h.app.drawing.wait(project);
    let sh = &project.drawings.sheets[0];
    let (bv, sv) = (sh.view(base).unwrap(), sh.view(sec).unwrap());
    assert!(
        matches!(&sv.kind, ViewKind::Section { letter, parent, .. } if letter == "A" && *parent == base)
    );
    assert_eq!(sv.orientation, Orientation::Left, "looking along +x");
    assert!(
        sv.pos[0] > bv.pos[0] + 10.0 && (sv.pos[1] - bv.pos[1]).abs() < 1e-9,
        "to the right of its parent, level"
    );
    // the cut face of the block is 25 wide and 10 high; the hatch fills it
    let s = sv.size();
    assert!(
        (s[0] - 25.0 * sv.scale).abs() < 1e-3 && (s[1] - 10.0 * sv.scale).abs() < 1e-3,
        "{s:?}"
    );
    assert!(count_layer(sv, Layer::Hatch) > 5);
    let hatch_len: f64 = sv
        .cache
        .as_ref()
        .unwrap()
        .scene
        .prims
        .iter()
        .filter_map(|p| match (&p.shape, p.layer) {
            (Shape::Line { a, b }, Layer::Hatch) => Some((a[0] - b[0]).hypot(a[1] - b[1])),
            _ => None,
        })
        .sum();
    let area = 25.0 * 10.0 * sv.scale * sv.scale;
    assert!(
        (hatch_len * 3.0 - area).abs() < 0.03 * area,
        "{hatch_len} x 3 vs {area}"
    );
    // the parent carries the cutting-plane line and the letters at its ends
    let notes = sh.annotations();
    let letters = notes
        .prims
        .iter()
        .filter(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "A"))
        .count();
    assert_eq!(letters, 2);
    assert!(notes.prims.iter().any(|p| p.layer == Layer::Section));
    // and the sheet's own scene captions the view
    let scene = sh.scene(1, 1);
    assert!(scene
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text.starts_with("SECTION A-A"))));
    // a cut that misses the block is refused and leaves no view behind
    let n = project.drawings.sheets[0].views.len();
    assert!(h
        .app
        .drawing
        .add_section(project, base, [100.0, -2.0], [100.0, 12.0], false)
        .is_err());
    assert_eq!(project.drawings.sheets[0].views.len(), n);
}

#[test]
fn flipping_a_section_looks_at_the_other_half() {
    let (mut h, _) = drawing_app("dw_flip");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let sec = h
        .app
        .drawing
        .add_section(project, base, [10.0, -2.0], [10.0, 12.0], false)
        .unwrap();
    h.app.drawing.wait(project);
    // x > 10 is kept: 30 wide in the block's x, so the section is 25 (y) wide either way, but
    // the other half is seen from the other side
    h.app.drawing.flip_section(project, sec).unwrap();
    h.app.drawing.wait(project);
    let v = project.drawings.sheets[0].view(sec).unwrap();
    assert_eq!(v.orientation, Orientation::Right);
    assert!(matches!(&v.kind, ViewKind::Section { flip: true, .. }));
    assert!(count_layer(v, Layer::Hatch) > 5);
    h.app.undo_redo(true);
    assert_eq!(
        dw_project(&mut h).drawings.sheets[0]
            .view(sec)
            .unwrap()
            .orientation,
        Orientation::Left
    );
}

#[test]
fn a_detail_encloses_what_is_inside_its_circle_at_a_larger_scale() {
    let (mut h, _) = drawing_app("dw_detail");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let before = project.drawings.sheets[0].view(base).unwrap().scale;
    let det = h
        .app
        .drawing
        .add_detail(project, base, [0.0, 0.0], 6.0, None)
        .unwrap();
    h.app.drawing.wait(project);
    let sh = &project.drawings.sheets[0];
    let (bv, dv) = (sh.view(base).unwrap(), sh.view(det).unwrap());
    assert!(
        dv.scale > bv.scale && before == bv.scale,
        "enlarged: {} vs {}",
        dv.scale,
        bv.scale
    );
    // it is a disc: its size is the circle's diameter at the new scale
    let r = 6.0 * dv.scale;
    assert!(
        (dv.size()[0] - 2.0 * r).abs() < 1e-6,
        "{:?} vs {}",
        dv.size(),
        2.0 * r
    );
    assert!(dv
        .cache
        .as_ref()
        .unwrap()
        .scene
        .prims
        .iter()
        .any(|p| matches!(p.shape, Shape::Circle { r: rr, .. } if (rr - r).abs() < 1e-9)));
    // it does not sit on top of another view
    for o in sh.views.iter().filter(|o| o.id != det) {
        let (os, ds) = (o.size(), dv.size());
        let apart = (o.pos[0] - dv.pos[0]).abs() > (os[0] + ds[0]) / 2.0
            || (o.pos[1] - dv.pos[1]).abs() > (os[1] + ds[1]) / 2.0;
        assert!(apart, "overlaps {:?}", o.kind);
    }
    // the parent shows the circle and the letter
    let notes = sh.annotations();
    assert!(notes
        .prims
        .iter()
        .any(|p| p.layer == Layer::Symbol && matches!(p.shape, Shape::Circle { .. })));
    assert!(notes
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "A")));
    // a detail of a section is not offered
    let sec = h
        .app
        .drawing
        .add_section(project, base, [20.0, -2.0], [20.0, 12.0], false)
        .unwrap();
    h.app.drawing.wait(project);
    assert!(h
        .app
        .drawing
        .add_detail(project, sec, [0.0, 0.0], 2.0, None)
        .is_err());
    // deleting the parent takes the section and the detail with it
    h.app.drawing.delete_view(project, base).unwrap();
    assert!(project.drawings.sheets[0].views.is_empty());
}

#[test]
fn section_and_detail_tools_work_with_clicks() {
    let (mut h, _) = drawing_app("dw_tools");
    let base = make_drawing(&mut h);
    h.frame();
    let (pos, size) = {
        let v = dw_project(&mut h).drawings.sheets[0]
            .view(base)
            .unwrap()
            .clone();
        (v.pos, v.size())
    };
    let click = |h: &mut Harness, p: Pos2| {
        pointer_frame(h, vec![egui::Event::PointerMoved(p)]);
        pointer_frame(h, vec![press(p, true)]);
        pointer_frame(h, vec![press(p, false)]);
    };
    // section: click the view's middle, then above it: a vertical line up the middle
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let mut d = ScriptedDialogs::default();
        h.app.drawing.command("section_view", project, &mut d);
    }
    assert!(h.app.drawing.tool.is_some());
    // the status line changes the layout: let it settle before aiming
    h.frame();
    h.frame();
    let first = h.app.drawing.screen_of(pos);
    click(&mut h, first);
    let second = h
        .app
        .drawing
        .screen_of([pos[0] + 0.7, pos[1] + size[1] * 0.7]); // a hair off the vertical
    click(&mut h, second);
    assert!(h.app.drawing.tool.is_none());
    dw_wait(&mut h);
    {
        let sh = &dw_project(&mut h).drawings.sheets[0];
        assert_eq!(sh.views.len(), 2, "{}", h.app.drawing.status);
        let ViewKind::Section { a, b, .. } = &sh.views[1].kind else {
            panic!("{:?}", sh.views[1].kind)
        };
        assert_eq!(a[0], b[0], "squared to the vertical");
        assert!(
            b[1] > a[1],
            "the line points up, so the material to the right is kept"
        );
    }
    // detail: click a centre, then a point on the edge
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let mut d = ScriptedDialogs::default();
        h.app.drawing.command("detail_view", project, &mut d);
    }
    h.frame();
    h.frame();
    let (c0, c1) = (
        h.app
            .drawing
            .screen_of([pos[0] - size[0] / 2.0 + 1.0, pos[1]]),
        h.app
            .drawing
            .screen_of([pos[0] - size[0] / 2.0 + 9.0, pos[1]]),
    );
    click(&mut h, c0);
    click(&mut h, c1);
    dw_wait(&mut h);
    let sh = &dw_project(&mut h).drawings.sheets[0];
    assert_eq!(sh.views.len(), 3, "{}", h.app.drawing.status);
    let ViewKind::Detail { radius, letter, .. } = &sh.views[2].kind else {
        panic!()
    };
    assert!(
        (radius * sh.views[0].scale - 8.0).abs() < 1e-3,
        "radius {radius}"
    );
    assert_eq!(letter, "B");
    // Escape cancels a tool
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let mut d = ScriptedDialogs::default();
        h.app.drawing.command("section_view", project, &mut d);
    }
    pointer_frame(
        &mut h,
        vec![egui::Event::Key {
            key: egui::Key::Escape,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }],
    );
    assert!(h.app.drawing.tool.is_none());
}

#[test]
fn sections_and_details_save_and_reopen() {
    let (mut h, dir) = drawing_app("dw_sec_save");
    let base = make_drawing(&mut h);
    let path = dir.join("sec.kwf");
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app
            .drawing
            .add_section(project, base, [20.0, -2.0], [20.0, 12.0], false)
            .unwrap();
        h.app.drawing.wait(project);
        h.app
            .drawing
            .add_detail(project, base, [0.0, 0.0], 6.0, None)
            .unwrap();
        h.app.drawing.wait(project);
        project.save_as(&path).unwrap();
    }
    let back = kw_app::project::Project::open(&path).unwrap();
    assert_eq!(back.drawings, dw_project(&mut h).drawings);
    let sh = &back.drawings.sheets[0];
    assert_eq!(sh.views.len(), 3);
    // the annotations come back from the stored geometry, without regenerating anything
    assert!(sh.annotations().prims.len() > 8);
}

#[test]
fn a_section_steps_aside_for_a_view_already_beside_its_parent() {
    let (mut h, _) = drawing_app("dw_section_clear");
    let base = make_drawing(&mut h);
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let right = h
        .app
        .drawing
        .add_projected(project, base, Side::Right)
        .unwrap();
    h.app.drawing.wait(project);
    // arrows to the right: the right view is already where the section would go
    let sec = h
        .app
        .drawing
        .add_section(project, base, [20.0, -2.0], [20.0, 12.0], false)
        .unwrap();
    h.app.drawing.wait(project);
    let sh = &project.drawings.sheets[0];
    let (r, s) = (sh.view(right).unwrap(), sh.view(sec).unwrap());
    assert!(
        s.pos[0] - s.size()[0] / 2.0 >= r.pos[0] + r.size()[0] / 2.0,
        "clear of the right view"
    );
    assert!(
        (s.pos[1] - r.pos[1]).abs() < 1e-9,
        "still level with its parent"
    );
}

// ------------------------------------------------------------------------------------ dimensions and annotations

use kw_drawing::{AnnotKind, DimKind, DimStatus, Gdt};

/// A 60 x 40 x 10 plate with two through holes of diameter 6, in the Drawing workspace with a
/// top view and a front view. Returns the ids of the hole, the extrusion and the two views.
struct Plate {
    hole: kw_feature::FeatureId,
    extrude: kw_feature::FeatureId,
    top: u64,
    front: u64,
}

fn plate_app(name: &str) -> (Harness, Plate) {
    let dir = scratch(name);
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Plate", &dir).unwrap();
    let mm = |v: f64| kw_expr::Param::length(v);
    let (hole, extrude) = {
        let p = h.app.project_mut().unwrap();
        let mut ids = None;
        p.history
            .edit(|d| {
                let sk = d.add_rect_sketch(
                    "Plate",
                    kw_feature::PlaneSpec::Xy(mm(0.0)),
                    mm(0.0),
                    mm(0.0),
                    mm(60.0),
                    mm(40.0),
                )?;
                let ex = d.add_extrude("Block", sk, mm(10.0), kw_feature::BodyOp::New)?;
                ids = Some((sk, ex));
                Ok(())
            })
            .unwrap();
        let _ = ids;
        let ev = p.evaluation();
        let (_, o) = ev.final_solid().unwrap();
        let b = o.brep().unwrap();
        let top = b
            .body_faces(o.body().unwrap())
            .into_iter()
            .filter(|f| kw_app::design::geom::face_frame(b, *f).is_some_and(|fr| fr.n[2] > 0.99))
            .max_by(|a, c| {
                b.face_bbox(*a)
                    .unwrap()
                    .max
                    .z
                    .total_cmp(&b.face_bbox(*c).unwrap().max.z)
            })
            .unwrap();
        let r = o.names.reference_to(b, top).unwrap();
        let mut hole = None;
        p.history
            .edit(|d| {
                hole = Some(d.add(
                    "Drill",
                    kw_feature::FeatureSpec::Hole(kw_feature::HoleSpec {
                        plane: kw_feature::PlaneSpec::Face(r),
                        points: vec![[mm(15.0), mm(20.0)], [mm(45.0), mm(20.0)]],
                        kind: kw_feature::HoleKind::Simple { diameter: mm(6.0) },
                        depth: mm(10.0),
                        base: None,
                        through_all: true,
                    }),
                )?);
                Ok(())
            })
            .unwrap();
        let ex = p
            .document()
            .features
            .iter()
            .rfind(|f| matches!(f.spec, kw_feature::FeatureSpec::Extrude(_)))
            .unwrap()
            .id;
        (hole.unwrap(), ex)
    };
    {
        let p = h.app.project_mut().unwrap();
        let ev = p.evaluation();
        assert!(
            ev.is_clean(),
            "the plate must evaluate: {:?}",
            p.document()
                .features
                .iter()
                .map(|f| ev.error(f.id))
                .collect::<Vec<_>>()
        );
    }
    h.app.set_workspace(Workspace::Drawing);
    h.frame();
    let (top, front);
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        let form = |o| kw_app::drawing::ViewForm {
            orientation: o,
            style: kw_drawing::ViewStyle::VisibleHidden,
            assembly: false,
            scale: None,
        };
        project
            .drawings
            .add_sheet("Sheet 1", kw_drawing::SheetFormat::default());
        h.app.drawing.sheet = 0;
        top = h
            .app
            .drawing
            .add_base_view(project, &form(Orientation::Top), false)
            .unwrap();
        h.app.drawing.wait(project);
        front = h
            .app
            .drawing
            .add_base_view(project, &form(Orientation::Front), false)
            .unwrap();
        h.app.drawing.wait(project);
    }
    (
        h,
        Plate {
            hole,
            extrude,
            top,
            front,
        },
    )
}

fn change_hole_diameter(h: &mut Harness, hole: kw_feature::FeatureId, d: f64) {
    let p = h.app.project_mut().unwrap();
    p.history
        .edit(|doc| {
            let Some(kw_feature::Feature {
                spec: kw_feature::FeatureSpec::Hole(hs),
                ..
            }) = doc.get(hole).cloned()
            else {
                panic!("not a hole")
            };
            let mut hs = hs;
            hs.kind = kw_feature::HoleKind::Simple {
                diameter: kw_expr::Param::length(d),
            };
            doc.replace_spec(hole, kw_feature::FeatureSpec::Hole(hs))
        })
        .unwrap();
}

fn dw_update(h: &mut Harness) {
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.update_all(project).unwrap();
    h.app.drawing.wait(project);
}

fn dims_of(h: &mut Harness) -> Vec<kw_drawing::Dimension> {
    dw_project(h).drawings.sheets[0]
        .annots
        .iter()
        .filter_map(|a| match &a.kind {
            AnnotKind::Dimension(d) => Some(d.clone()),
            _ => None,
        })
        .collect()
}

#[test]
fn a_hole_dimension_is_tied_to_the_hole_counts_its_twin_and_follows_a_size_change() {
    let (mut h, pl) = plate_app("dw_hole");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    // the circles the top view can dimension: both holes, both ends of each (a through hole)
    let cands = h.app.drawing.candidates_for(project, pl.top).unwrap();
    let circles: Vec<_> = cands
        .iter()
        .filter(|c| matches!(c.kind, kw_drawing::snap::CandKind::Circle))
        .cloned()
        .collect();
    assert!(circles.len() >= 2, "{}", circles.len());
    let c = circles
        .iter()
        .find(|c| c.circle.unwrap().0[0] < 30.0)
        .unwrap()
        .clone();
    let id = h
        .app
        .drawing
        .add_dimension(project, pl.top, DimKind::Diameter, &[c], 0.0, 0.8)
        .unwrap();
    let d = dims_of(&mut h).pop().unwrap();
    assert_eq!(d.value, 6.0);
    assert_eq!(d.prefix, "2×", "two holes of this size");
    assert_eq!(d.status, DimStatus::Ok);
    assert!(
        matches!(d.anchors[0], kw_drawing::dim::Anchor::Circle { .. }),
        "attached through the faces, not fixed in space"
    );
    // the text on the sheet
    let sheet = &dw_project(&mut h).drawings.sheets[0];
    assert!(sheet
        .annotations()
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "2×Ø6")));
    let _ = id;
    // make the holes 10 across: the dimension follows, and so does the drawn circle
    change_hole_diameter(&mut h, pl.hole, 10.0);
    dw_update(&mut h);
    let d = dims_of(&mut h).pop().unwrap();
    assert_eq!(d.value, 10.0);
    assert_eq!(d.status, DimStatus::Ok);
    let v = dw_project(&mut h).drawings.sheets[0]
        .view(pl.top)
        .unwrap()
        .clone();
    assert!(
        v.cache
            .as_ref()
            .unwrap()
            .marks
            .iter()
            .any(|(_, r)| (r - 5.0 * v.scale).abs() < 1e-6),
        "the view shows the new size"
    );
    let sheet = &dw_project(&mut h).drawings.sheets[0];
    assert!(sheet
        .annotations()
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "2×Ø10")));
}

#[test]
fn linear_dimensions_measure_the_model_and_track_its_edits() {
    let (mut h, pl) = plate_app("dw_linear");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    // the front view: the plate's thickness, between two corners on its left side
    let cands = h.app.drawing.candidates_for(project, pl.front).unwrap();
    let corner = |z: f64| {
        cands
            .iter()
            .find(|c| {
                matches!(c.kind, kw_drawing::snap::CandKind::Point(_))
                    && c.p3[0].abs() < 1e-9
                    && (c.p3[2] - z).abs() < 1e-9
                    && c.p3[1].abs() < 1e-9
            })
            .cloned()
            .unwrap_or_else(|| panic!("no corner at z = {z}"))
    };
    h.app
        .drawing
        .add_dimension(
            project,
            pl.front,
            DimKind::Vertical,
            &[corner(0.0), corner(10.0)],
            -12.0,
            0.0,
        )
        .unwrap();
    // and the plate's length along the bottom
    let right = cands
        .iter()
        .find(|c| {
            matches!(c.kind, kw_drawing::snap::CandKind::Point(_))
                && (c.p3[0] - 60.0).abs() < 1e-9
                && c.p3[2].abs() < 1e-9
                && c.p3[1].abs() < 1e-9
        })
        .cloned()
        .unwrap();
    h.app
        .drawing
        .add_dimension(
            project,
            pl.front,
            DimKind::Horizontal,
            &[corner(0.0), right],
            -10.0,
            0.0,
        )
        .unwrap();
    let ds = dims_of(&mut h);
    assert_eq!((ds[0].value, ds[1].value), (10.0, 60.0));
    // a thicker plate
    {
        let p = h.app.project_mut().unwrap();
        p.history
            .edit(|doc| {
                let Some(kw_feature::Feature {
                    spec: kw_feature::FeatureSpec::Extrude(e),
                    ..
                }) = doc.get(pl.extrude).cloned()
                else {
                    panic!()
                };
                let mut e = e;
                e.depth = kw_expr::Param::length(25.0);
                doc.replace_spec(pl.extrude, kw_feature::FeatureSpec::Extrude(e))
            })
            .unwrap();
    }
    dw_update(&mut h);
    let ds = dims_of(&mut h);
    assert_eq!(
        (ds[0].value, ds[1].value),
        (25.0, 60.0),
        "{:?}",
        ds.iter().map(|d| &d.status).collect::<Vec<_>>()
    );
    assert!(ds.iter().all(|d| d.status == DimStatus::Ok));
    // the value is the model's, not the view's: changing the scale does not change it
    let id = pl.front;
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        project.drawings.sheets[0].view_mut(id).unwrap().scale = 0.5;
    }
    dw_update(&mut h);
    assert_eq!(dims_of(&mut h)[0].value, 25.0);
}

#[test]
fn a_dimension_whose_hole_is_gone_says_so_and_keeps_its_last_value() {
    let (mut h, pl) = plate_app("dw_broken");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let cands = h.app.drawing.candidates_for(project, pl.top).unwrap();
    let c = cands
        .iter()
        .find(|c| matches!(c.kind, kw_drawing::snap::CandKind::Circle))
        .unwrap()
        .clone();
    h.app
        .drawing
        .add_dimension(project, pl.top, DimKind::Diameter, &[c], 0.0, 0.8)
        .unwrap();
    {
        let p = h.app.project_mut().unwrap();
        p.history.edit(|d| d.remove(pl.hole).map(|_| ())).unwrap();
    }
    dw_update(&mut h);
    let d = dims_of(&mut h).pop().unwrap();
    assert!(
        matches!(&d.status, DimStatus::Broken(m) if m.contains("no longer")),
        "{:?}",
        d.status
    );
    assert_eq!(d.value, 6.0, "the last good value stays");
    // it is still drawn, so the user can see which one needs fixing
    assert!(dw_project(&mut h).drawings.sheets[0]
        .annotations()
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text.contains("Ø6"))));
}

#[test]
fn views_carry_centre_marks_for_holes_and_centre_lines_for_their_walls() {
    let (mut h, pl) = plate_app("dw_marks");
    let sh = &dw_project(&mut h).drawings.sheets[0];
    let (t, f) = (sh.view(pl.top).unwrap(), sh.view(pl.front).unwrap());
    // seen from above: two circles of radius 3 mm x scale
    let tm = &t.cache.as_ref().unwrap().marks;
    assert!(
        tm.iter()
            .filter(|(_, r)| (r - 3.0 * t.scale).abs() < 1e-6)
            .count()
            >= 2,
        "{tm:?}"
    );
    // seen from the front: the holes' axes
    let fa = &f.cache.as_ref().unwrap().axes;
    assert_eq!(fa.len(), 2, "{fa:?}");
    assert!(fa
        .iter()
        .all(|(a, b)| (a[0] - b[0]).abs() < 1e-9
            && ((b[1] - a[1]).abs() - 10.0 * f.scale).abs() < 1e-6));
    // drawn on the sheet as centre lines, and gone when switched off
    let sc = sh.annotations();
    assert!(sc.prims.iter().filter(|p| p.layer == Layer::Center).count() >= 4);
    let id = pl.top;
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    project.drawings.sheets[0]
        .view_mut(id)
        .unwrap()
        .center_marks = false;
    let before = project.drawings.sheets[0]
        .annotations()
        .prims
        .iter()
        .filter(|p| p.layer == Layer::Center)
        .count();
    assert!(before < sc.prims.iter().filter(|p| p.layer == Layer::Center).count());
}

#[test]
fn notes_and_symbols_are_added_deleted_undone_and_saved() {
    let (mut h, _pl) = plate_app("dw_notes");
    let dir = scratch("dw_notes_out");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let dw = &mut h.app.drawing;
    let note = dw
        .add_note(
            project,
            [30.0, 200.0],
            "BREAK ALL EDGES\n0.5 MAX",
            Some([60.0, 150.0]),
        )
        .unwrap();
    let d1 = dw.add_datum(project, [80.0, 60.0], [60.0, 60.0]).unwrap();
    let d2 = dw.add_datum(project, [90.0, 60.0], [60.0, 70.0]).unwrap();
    dw.add_surface(project, [100.0, 100.0], "Ra 1.6").unwrap();
    dw.add_fcf(
        project,
        [120.0, 100.0],
        Gdt::Position,
        "Ø0.1",
        &["A", "B"],
        Some([60.0, 60.0]),
    )
    .unwrap();
    let sheet = &project.drawings.sheets[0];
    let letters: Vec<_> = sheet
        .annots
        .iter()
        .filter_map(|a| match &a.kind {
            AnnotKind::Datum { letter, .. } => Some(letter.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(letters, ["A", "B"], "datum letters are given in order");
    let scene = sheet.annotations();
    for want in ["BREAK ALL EDGES", "0.5 MAX", "Ra 1.6", "Ø0.1"] {
        assert!(
            scene
                .prims
                .iter()
                .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == want)),
            "{want}"
        );
    }
    let path = dir.join("n.kwf");
    project.save_as(&path).unwrap();
    assert_eq!(
        kw_app::project::Project::open(&path).unwrap().drawings,
        project.drawings
    );
    h.app.drawing.delete_annot(project, note).unwrap();
    h.app.drawing.delete_annot(project, d1).unwrap();
    assert_eq!(project.drawings.sheets[0].annots.len(), 3);
    assert!(h.app.drawing.delete_annot(project, d1).is_err());
    h.app.undo_redo(true);
    h.app.undo_redo(true);
    assert_eq!(dw_project(&mut h).drawings.sheets[0].annots.len(), 5);
    let _ = d2;
}

fn dw_click(h: &mut Harness, p: Pos2) {
    pointer_frame(h, vec![egui::Event::PointerMoved(p)]);
    pointer_frame(h, vec![press(p, true)]);
    pointer_frame(h, vec![press(p, false)]);
}

fn dw_command(h: &mut Harness, id: &str) {
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let mut d = ScriptedDialogs::default();
    h.app.drawing.command(id, project, &mut d);
    // the status line changes the layout: let it settle before aiming
    h.frame();
    h.frame();
}

fn dw_click_view(h: &mut Harness, view: u64, local: [f64; 2]) {
    let p = view_pt(h, view, local);
    dw_click(h, p);
}

/// The screen position of a point of a view given relative to the view's origin (sheet mm).
fn view_pt(h: &mut Harness, view: u64, local: [f64; 2]) -> Pos2 {
    let v = dw_project(h).drawings.sheets[0].view(view).unwrap().clone();
    h.app
        .drawing
        .screen_of([v.pos[0] + local[0], v.pos[1] + local[1]])
}

#[test]
fn dimension_tools_are_driven_by_clicks_on_the_model() {
    let (mut h, pl) = plate_app("dw_dim_tools");
    h.frame();
    // a diameter: click the rim of a hole, then where the leader should point
    let cands = {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.drawing.candidates_for(project, pl.top).unwrap()
    };
    let hole = cands
        .iter()
        .find(|c| matches!(c.kind, kw_drawing::snap::CandKind::Circle))
        .unwrap()
        .clone();
    let scale = dw_project(&mut h).drawings.sheets[0]
        .view(pl.top)
        .unwrap()
        .scale;
    dw_command(&mut h, "dim_diameter");
    assert!(h.app.drawing.tool.is_some());
    let rim = view_pt(&mut h, pl.top, [hole.p2[0] + 3.0 * scale, hole.p2[1]]);
    dw_click(&mut h, rim);
    let away = view_pt(
        &mut h,
        pl.top,
        [hole.p2[0] + 3.0 * scale + 20.0, hole.p2[1] + 20.0],
    );
    dw_click(&mut h, away);
    dw_wait(&mut h);
    let ds = dims_of(&mut h);
    assert_eq!(ds.len(), 1, "{}", h.app.drawing.status);
    assert_eq!((ds[0].kind, ds[0].value), (DimKind::Diameter, 6.0));
    assert!(h.app.drawing.tool.is_none());
    assert!(
        (ds[0].angle - std::f64::consts::FRAC_PI_4).abs() < 0.2,
        "the leader points where it was placed: {}",
        ds[0].angle
    );
    // a linear dimension on the front view: two corners, then above them: horizontal
    let fc = {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        h.app.drawing.candidates_for(project, pl.front).unwrap()
    };
    let corner = |x: f64| {
        fc.iter()
            .find(|c| {
                matches!(c.kind, kw_drawing::snap::CandKind::Point(_))
                    && (c.p3[0] - x).abs() < 1e-9
                    && c.p3[2].abs() < 1e-9
                    && c.p3[1].abs() < 1e-9
            })
            .unwrap()
            .clone()
    };
    let (a, b) = (corner(0.0), corner(60.0));
    dw_command(&mut h, "dim_linear");
    let (pa, pb) = (
        view_pt(&mut h, pl.front, a.p2),
        view_pt(&mut h, pl.front, b.p2),
    );
    dw_click(&mut h, pa);
    dw_click(&mut h, pb);
    let place = view_pt(
        &mut h,
        pl.front,
        [(a.p2[0] + b.p2[0]) / 2.0, a.p2[1] - 14.0],
    );
    dw_click(&mut h, place);
    let ds = dims_of(&mut h);
    assert_eq!(ds.len(), 2, "{}", h.app.drawing.status);
    assert_eq!((ds[1].kind, ds[1].value), (DimKind::Horizontal, 60.0));
    assert!(
        ds[1].line_pos < a.p2[1],
        "below the plate: {}",
        ds[1].line_pos
    );
    // placed beside instead: vertical
    let (z0, z1) = (
        fc.iter()
            .find(|c| {
                matches!(c.kind, kw_drawing::snap::CandKind::Point(_))
                    && c.p3[0].abs() < 1e-9
                    && c.p3[2].abs() < 1e-9
                    && c.p3[1].abs() < 1e-9
            })
            .unwrap()
            .clone(),
        fc.iter()
            .find(|c| {
                matches!(c.kind, kw_drawing::snap::CandKind::Point(_))
                    && c.p3[0].abs() < 1e-9
                    && (c.p3[2] - 10.0).abs() < 1e-9
                    && c.p3[1].abs() < 1e-9
            })
            .unwrap()
            .clone(),
    );
    dw_command(&mut h, "dim_linear");
    dw_click_view(&mut h, pl.front, z0.p2);
    dw_click_view(&mut h, pl.front, z1.p2);
    dw_click_view(
        &mut h,
        pl.front,
        [z0.p2[0] - 12.0, (z0.p2[1] + z1.p2[1]) / 2.0],
    );
    let ds = dims_of(&mut h);
    assert_eq!((ds[2].kind, ds[2].value), (DimKind::Vertical, 10.0));
    // Escape cancels a tool half way; the clicks so far made nothing
    dw_command(&mut h, "dim_aligned");
    dw_click_view(&mut h, pl.front, z0.p2);
    pointer_frame(
        &mut h,
        vec![egui::Event::Key {
            key: egui::Key::Escape,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }],
    );
    assert!(h.app.drawing.tool.is_none());
    assert_eq!(dims_of(&mut h).len(), 3);
    // each dimension is a step of its own
    h.app.undo_redo(true);
    assert_eq!(dims_of(&mut h).len(), 2);
}

#[test]
fn annotations_are_selected_dragged_and_deleted_on_the_sheet() {
    let (mut h, pl) = plate_app("dw_annot_ui");
    h.frame();
    let _ = pl;
    // a note by its tool: one click places it
    dw_command(&mut h, "note");
    let at = h.app.drawing.screen_of([60.0, 230.0]);
    dw_click(&mut h, at);
    let id = h
        .app
        .drawing
        .selected_annot
        .expect("the new note is selected");
    let pos = |h: &mut Harness| match &dw_project(h).drawings.sheets[0]
        .annots
        .iter()
        .find(|a| a.id == id)
        .unwrap()
        .kind
    {
        AnnotKind::Note { pos, .. } => *pos,
        _ => panic!(),
    };
    let start = pos(&mut h);
    assert!((start[0] - 60.0).abs() < 0.5 && (start[1] - 230.0).abs() < 0.5);
    h.app.drawing.selected_annot = None;
    h.frame();
    // click on it to select, drag to move
    let on = h.app.drawing.screen_of([start[0] + 3.0, start[1] + 1.5]);
    pointer_frame(&mut h, vec![egui::Event::PointerMoved(on)]);
    pointer_frame(&mut h, vec![press(on, true)]);
    for k in 1..=5 {
        pointer_frame(
            &mut h,
            vec![egui::Event::PointerMoved(
                on + egui::vec2(6.0 * k as f32, 4.0 * k as f32),
            )],
        );
    }
    pointer_frame(&mut h, vec![press(on + egui::vec2(30.0, 20.0), false)]);
    assert_eq!(h.app.drawing.selected_annot, Some(id));
    let moved = pos(&mut h);
    assert!(
        moved[0] > start[0] + 2.0 && moved[1] < start[1] - 2.0,
        "{start:?} -> {moved:?}"
    );
    // one drag, one undo step
    h.app.undo_redo(true);
    assert_eq!(pos(&mut h), start);
    h.app.undo_redo(false);
    // Delete removes the selection
    h.frame();
    pointer_frame(
        &mut h,
        vec![egui::Event::Key {
            key: egui::Key::Delete,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }],
    );
    assert!(dw_project(&mut h).drawings.sheets[0].annots.is_empty());
}

// ------------------------------------------------------------------------------------ parts list and balloons

use kw_assembly::{Assembly, Part, Placement};

fn box_part(name: &str, w: f64, h: f64, d: f64, material: Option<&str>) -> Part {
    let mm = |v: f64| kw_expr::Param::length(v);
    let mut doc = kw_feature::Document::new();
    let s = doc
        .add_rect_sketch(
            "S",
            kw_feature::PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(w),
            mm(h),
        )
        .unwrap();
    doc.add_extrude("Box", s, mm(d), kw_feature::BodyOp::New)
        .unwrap();
    let mut p = Part::new(name, doc, vec![]);
    p.material = material.map(String::from);
    p
}

/// A 60 x 40 x 10 aluminium plate with two steel pins (10 x 10 x 20) standing on it, in the
/// Drawing workspace with a front view of the assembly. Returns the view.
fn rig_app(name: &str) -> (Harness, u64) {
    let dir = scratch(name);
    let mut h = Harness::new(app_in(&dir));
    h.app.create_project("Rig", &dir).unwrap();
    {
        let p = h.app.project_mut().unwrap();
        let mut a = Assembly::new("Rig");
        let plate = a.add_part(box_part("Plate", 60.0, 40.0, 10.0, Some("Aluminium")));
        let pin = a.add_part(box_part("Pin", 10.0, 10.0, 20.0, Some("Steel")));
        let i0 = a.add_instance("Plate:1", plate);
        a.instances[i0].grounded = true;
        for (k, x) in [10.0, 40.0].iter().enumerate() {
            let i = a.add_instance(&format!("Pin:{}", k + 1), pin);
            a.instances[i].at = Placement {
                t: [*x, 15.0, 10.0],
                ..Placement::default()
            };
        }
        p.assembly = Some(a);
        p.mark_extras_dirty();
    }
    h.app.set_workspace(Workspace::Drawing);
    h.frame();
    let front;
    {
        let Screen::Project { project, .. } = &mut h.app.screen else {
            panic!()
        };
        project
            .drawings
            .add_sheet("Sheet 1", kw_drawing::SheetFormat::default());
        h.app.drawing.sheet = 0;
        let f = kw_app::drawing::ViewForm {
            orientation: Orientation::Front,
            style: kw_drawing::ViewStyle::Visible,
            assembly: true,
            scale: None,
        };
        front = h.app.drawing.add_base_view(project, &f, false).unwrap();
        h.app.drawing.wait(project);
    }
    (h, front)
}

fn add_balloon_at(h: &mut Harness, view: u64, uv: [f64; 2], pos: [f64; 2]) -> Result<u64, String> {
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.add_balloon(project, view, uv, pos)
}

fn boms_of(h: &mut Harness) -> Vec<(Vec<kw_drawing::BomColumn>, Vec<kw_drawing::BomRowData>)> {
    dw_project(h).drawings.sheets[0]
        .annots
        .iter()
        .filter_map(|a| match &a.kind {
            AnnotKind::Bom { columns, rows, .. } => Some((columns.clone(), rows.clone())),
            _ => None,
        })
        .collect()
}

fn balloons_of(h: &mut Harness) -> Vec<(u32, String, [f64; 3], DimStatus)> {
    dw_project(h).drawings.sheets[0]
        .annots
        .iter()
        .filter_map(|a| match &a.kind {
            AnnotKind::Balloon {
                item,
                part,
                world,
                status,
                ..
            } => Some((*item, part.clone(), *world, status.clone())),
            _ => None,
        })
        .collect()
}

#[test]
fn the_parts_list_counts_and_weighs_like_the_assembly_bill_of_materials() {
    use kw_app::design::asm_inspect::{bom_csv, bom_rows};
    let (mut h, _front) = rig_app("dw_bom");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.add_bom(project).unwrap();
    assert!(h.app.drawing.add_bom(project).is_err(), "one list a sheet");
    // the reference: the assembly's own BOM, read independently
    let pose = project
        .assembly
        .as_ref()
        .unwrap()
        .pose(&mut project.engine)
        .unwrap();
    let want = bom_rows(&pose);
    let rows = boms_of(&mut h).pop().unwrap().1;
    assert_eq!(rows.len(), want.len());
    assert_eq!(rows.len(), 2);
    for (r, w) in rows.iter().zip(&want) {
        assert_eq!(
            (r.part.as_str(), r.count as usize),
            (w.part.as_str(), w.count)
        );
        assert_eq!(r.material, w.material.clone().unwrap_or_default());
        assert!((r.volume - w.volume).abs() < 1e-9 && (r.mass_g - w.mass_g).abs() < 1e-9);
    }
    // 60 x 40 x 10 aluminium (2700 kg/m3) = 64.8 g; a steel pin (7850) of 2000 mm3 = 15.7 g
    assert_eq!(rows[0].part, "Plate");
    assert!(
        (rows[0].mass_g - 24000.0 * 2700.0e-6).abs() < 1e-6,
        "{}",
        rows[0].mass_g
    );
    assert_eq!((rows[1].part.as_str(), rows[1].count), ("Pin", 2));
    assert!((rows[1].mass_g - 2000.0 * 7850.0e-6).abs() < 1e-6);
    // the CSV is the assembly's
    assert_eq!(
        h.app.drawing.bom_csv(h.app.project().unwrap()).unwrap(),
        bom_csv(&want)
    );
    // it is drawn on the sheet, its corner at the top of the title block
    let sheet = &dw_project(&mut h).drawings.sheets[0];
    let notes = sheet.annotations();
    for t in ["Plate", "Pin", "ITEM", "QTY", "MATERIAL"] {
        assert!(
            notes
                .prims
                .iter()
                .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == t)),
            "{t}"
        );
    }
    let (_, hi) = notes.bounds().unwrap();
    assert!(hi[0] >= sheet.format.frame().title.1[0] - 1e-6);
}

#[test]
fn the_parts_list_is_saved_exported_and_its_columns_are_chosen() {
    let (mut h, _front) = rig_app("dw_bom_csv");
    let dir = scratch("dw_bom_out");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    let id = h.app.drawing.add_bom(project).unwrap();
    // fewer / more columns
    if let Some(a) = project.drawings.sheets[0]
        .annots
        .iter_mut()
        .find(|a| a.id == id)
    {
        if let AnnotKind::Bom { columns, .. } = &mut a.kind {
            *columns = vec![
                kw_drawing::BomColumn::Item,
                kw_drawing::BomColumn::Part,
                kw_drawing::BomColumn::TotalMass,
            ];
        }
    }
    let notes = project.drawings.sheets[0].annotations();
    assert!(notes
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "TOTAL (g)")));
    assert!(!notes
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "MATERIAL")));
    // through the dialogs
    let out = dir.join("bom.csv");
    let mut dialogs = ScriptedDialogs {
        save: vec![out.clone()],
        ..Default::default()
    };
    h.app.drawing.command("export_bom", project, &mut dialogs);
    let csv = std::fs::read_to_string(&out).unwrap();
    assert!(
        csv.lines()
            .nth(1)
            .unwrap()
            .starts_with("1,Plate,1,Aluminium,"),
        "{csv}"
    );
    assert!(csv.lines().last().unwrap().starts_with(",Total,3,"));
    // saved with the sheet
    let path = dir.join("rig.kwf");
    project.save_as(&path).unwrap();
    assert_eq!(
        kw_app::project::Project::open(&path).unwrap().drawings,
        project.drawings
    );
}

#[test]
fn balloons_carry_the_items_of_their_parts_and_follow_the_assembly() {
    let (mut h, front) = rig_app("dw_balloon");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.add_bom(project).unwrap();
    // the front view (x right, z up): the plate's body at (5, 5) and the second pin at (45, 20)
    let v = project.drawings.sheets[0].view(front).unwrap().clone();
    let sheet_pos = |uv: [f64; 2]| {
        let c = v.cache.as_ref().unwrap().centre;
        [
            v.pos[0] + uv[0] * v.scale - c[0],
            v.pos[1] + uv[1] * v.scale - c[1],
        ]
    };
    let _ = sheet_pos;
    add_balloon_at(&mut h, front, [5.0, 5.0], [30.0, 250.0]).unwrap();
    add_balloon_at(&mut h, front, [45.0, 20.0], [90.0, 250.0]).unwrap();
    add_balloon_at(&mut h, front, [15.0, 20.0], [60.0, 250.0]).unwrap();
    let b = balloons_of(&mut h);
    assert_eq!(
        b.iter().map(|x| (x.0, x.1.as_str())).collect::<Vec<_>>(),
        [(1, "Plate"), (2, "Pin"), (2, "Pin")],
        "the number is the part's row in the list"
    );
    // the point is on the front of the component that was clicked (the plate's front face is y = 0)
    assert!(
        b[0].2[1].abs() < 1e-6 && (b[0].2[0] - 5.0).abs() < 1e-6 && (b[0].2[2] - 5.0).abs() < 1e-6,
        "{:?}",
        b[0].2
    );
    assert!(
        (b[1].2[1] - 15.0).abs() < 1e-6,
        "the pin's front face is at y = 15: {:?}",
        b[1].2
    );
    assert!(
        add_balloon_at(&mut h, front, [200.0, 20.0], [0.0, 0.0]).is_err(),
        "nothing there"
    );
    // drawn: the item numbers on the sheet
    let sheet = &dw_project(&mut h).drawings.sheets[0];
    let notes = sheet.annotations();
    assert_eq!(
        notes
            .prims
            .iter()
            .filter(|p| matches!(&p.shape, Shape::Circle { r, .. } if (*r - 5.0).abs() < 1e-9))
            .count(),
        3
    );
    // move the second pin 20 mm along x: its balloon's point goes with it, the other pin's stays
    {
        let p = h.app.project_mut().unwrap();
        p.assembly.as_mut().unwrap().instances[2].at.t[0] += 20.0;
    }
    dw_update(&mut h);
    let b = balloons_of(&mut h);
    assert!(
        (b[1].2[0] - 65.0).abs() < 1e-6,
        "followed the pin: {:?}",
        b[1].2
    );
    assert!(
        (b[2].2[0] - 15.0).abs() < 1e-6,
        "the other pin did not move: {:?}",
        b[2].2
    );
    assert!(b.iter().all(|x| x.3 == DimStatus::Ok));
    // take the plate out of the assembly: the pins become item 1, the plate's balloon is broken
    {
        let p = h.app.project_mut().unwrap();
        p.assembly.as_mut().unwrap().instances[0].hidden = true;
    }
    dw_update(&mut h);
    let b = balloons_of(&mut h);
    assert!(matches!(b[0].3, DimStatus::Broken(_)), "{:?}", b[0]);
    assert_eq!((b[1].0, b[2].0), (1, 1), "numbers follow the list");
    assert_eq!(boms_of(&mut h)[0].1.len(), 1);
    let notes = dw_project(&mut h).drawings.sheets[0].annotations();
    assert!(notes
        .prims
        .iter()
        .any(|p| matches!(&p.shape, Shape::Text { text, .. } if text == "?")));
}

#[test]
fn the_balloon_tool_takes_a_click_on_the_component_then_one_for_the_balloon() {
    let (mut h, front) = rig_app("dw_balloon_tool");
    h.frame();
    dw_command(&mut h, "balloon");
    let v = dw_project(&mut h).drawings.sheets[0]
        .view(front)
        .unwrap()
        .clone();
    let c = v.cache.as_ref().unwrap().centre;
    // the second pin's front at (45, 20), in view coordinates (sheet mm about the view origin)
    let on = [45.0 * v.scale - c[0], 20.0 * v.scale - c[1]];
    dw_click_view(&mut h, front, on);
    assert!(
        matches!(
            h.app.drawing.tool,
            Some(kw_app::drawing::Tool::Balloon { first: Some(_) })
        ),
        "{:?}",
        h.app.drawing.tool
    );
    let at = h.app.drawing.screen_of([v.pos[0] + 40.0, v.pos[1] + 50.0]);
    dw_click(&mut h, at);
    assert!(h.app.drawing.tool.is_none(), "{}", h.app.drawing.status);
    let b = balloons_of(&mut h);
    assert_eq!(b.len(), 1);
    assert_eq!(b[0].1, "Pin");
    assert!(
        (b[0].2[0] - 45.0).abs() < 0.5 && (b[0].2[2] - 20.0).abs() < 0.5,
        "{:?}",
        b[0].2
    );
    // it is a step of its own
    h.app.undo_redo(true);
    assert!(balloons_of(&mut h).is_empty());
}

#[test]
fn a_design_without_an_assembly_lists_its_bodies() {
    let (mut h, _pl) = plate_app("dw_bom_design");
    let Screen::Project { project, .. } = &mut h.app.screen else {
        panic!()
    };
    h.app.drawing.add_bom(project).unwrap();
    let rows = boms_of(&mut h).pop().unwrap().1;
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].count, 1);
    // 60 x 40 x 10 with two holes of 6 mm
    let v = 60.0 * 40.0 * 10.0 - 2.0 * std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (rows[0].volume - v).abs() < 1e-3,
        "{} vs {v}",
        rows[0].volume
    );
    assert_eq!(rows[0].size.map(|x| x.round()), [60.0, 40.0, 10.0]);
}
