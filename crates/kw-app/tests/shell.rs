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
    // deleting a sketch that has a dependent is refused
    act(&mut h, Action::Delete(sk));
    assert!(!h.app.design.model.error.is_empty());
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
