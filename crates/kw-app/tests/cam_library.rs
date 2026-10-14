//! The CAM libraries in use: tools numbered across the project, a machine's limits, posts and machines
//! embedded in the project, and feeds and speeds from the workpiece material.

use kw_app::cam::{CamWorkspace, OpKind, ToolChoice};
use kw_app::project::Project;
use kw_cam::library::{Library, PostDef};
use kw_cam::Tool;
use kw_store::BodyKey;

fn mm(v: f64) -> kw_expr::Param {
    kw_expr::Param::length(v)
}

fn block(material: Option<&str>) -> Project {
    let mut p = Project::empty();
    p.history
        .edit(|d| {
            let s = d.add_rect_sketch(
                "S",
                kw_feature::PlaneSpec::Xy(mm(0.0)),
                mm(0.0),
                mm(0.0),
                mm(40.0),
                mm(25.0),
            )?;
            let root = d.add_extrude("Block", s, mm(10.0), kw_feature::BodyOp::New)?;
            if let Some(m) = material {
                d.set_appearance(
                    root,
                    kw_feature::Appearance {
                        material: Some(m.to_string()),
                        ..Default::default()
                    },
                );
            }
            Ok(())
        })
        .unwrap();
    p
}

fn pocket(ws: &mut CamWorkspace, diameter: f64) {
    let s = &mut ws.settings;
    s.op = OpKind::Pocket;
    s.tool = ToolChoice::Flat;
    s.preset = None;
    s.diameter = diameter;
    (s.x0, s.y0, s.x1, s.y1) = (5.0, 5.0, 35.0, 20.0);
    s.region = None;
    s.top = 10.0;
    s.bottom = 7.0;
}

#[test]
fn tools_have_one_number_in_the_whole_project() {
    let mut p = block(None);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    pocket(&mut ws, 6.0);
    ws.run_now(&mut p).unwrap();
    pocket(&mut ws, 8.0);
    ws.run_now(&mut p).unwrap();
    ws.new_setup(&mut p);
    // the same 8 mm tool in the second setup is still T2, and a new tool gets the next free number
    pocket(&mut ws, 8.0);
    ws.run_now(&mut p).unwrap();
    pocket(&mut ws, 10.0);
    ws.run_now(&mut p).unwrap();
    pocket(&mut ws, 6.0);
    ws.run_now(&mut p).unwrap();
    let numbers = |i: usize| -> Vec<(u32, f64)> {
        p.cam[i]
            .tools
            .iter()
            .map(|t| (t.number, t.diameter))
            .collect()
    };
    assert_eq!(numbers(0), vec![(1, 6.0), (2, 8.0)]);
    assert_eq!(numbers(1), vec![(2, 8.0), (3, 10.0), (1, 6.0)]);
    // the toolpaths call the tools by those numbers
    let called: Vec<u32> = p.cam[1]
        .operations
        .iter()
        .map(|o| {
            o.path
                .events
                .iter()
                .find_map(|e| match e {
                    kw_cam::Event::ToolChange { number, .. } => Some(*number),
                    _ => None,
                })
                .unwrap()
        })
        .collect();
    assert_eq!(called, vec![2, 3, 1]);
    // no two different tools share a number anywhere
    let mut seen: Vec<(u32, f64)> = Vec::new();
    for s in &p.cam {
        for t in &s.tools {
            match seen.iter().find(|(n, _)| *n == t.number) {
                Some((_, d)) => assert_eq!(*d, t.diameter),
                None => seen.push((t.number, t.diameter)),
            }
        }
    }
}

#[test]
fn a_machine_holds_the_operation_to_its_spindle_and_feed_and_brings_its_post() {
    let mut p = block(None);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    ws.set_machine(&mut p, "Haas VF-2").unwrap();
    assert_eq!(p.cam[0].machine, "Haas VF-2");
    assert_eq!(p.cam[0].post.as_ref().unwrap().id, "Haas");
    assert!(p.cam[0].machine_def.is_some());
    pocket(&mut ws, 6.0);
    ws.settings.rpm = 20_000.0;
    ws.settings.feed = 99_999.0;
    ws.run_now(&mut p).unwrap();
    let params = p.cam[0].operations[0].params.as_ref().unwrap();
    assert_eq!(params.rpm, 8_100.0);
    assert_eq!(params.feed, 16_500.0);
    assert!(ws.set_machine(&mut p, "no such machine").is_err());
    // the program is the Haas post's
    let text = ws.setup_post_text(&p, 0).unwrap().unwrap();
    assert!(text.contains("S8100"), "{text}");
}

#[test]
fn a_drill_does_not_mill_a_pocket() {
    let mut p = block(None);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    pocket(&mut ws, 6.0);
    ws.settings.preset = Some(Tool::drill(1, 6.0, 118.0, 30.0));
    let e = ws.run_now(&mut p).unwrap_err();
    assert!(e.contains("does not mill"), "{e}");
    assert!(p.cam[0].operations.is_empty());
}

#[test]
fn a_project_posts_from_its_embedded_copies_without_the_users_library() {
    let shop = kw_cam::post::FANUC
        .replace("name = Fanuc", "name = Shop")
        .replace("program_start = ", "program_start = @(SHOP HEADER)\\n");
    let mut p = block(None);
    let mut ws = CamWorkspace::default();
    ws.library.import_post(&shop).unwrap();
    ws.fit_to_workpiece(&mut p);
    ws.set_post(&mut p, "Shop").unwrap();
    pocket(&mut ws, 6.0);
    ws.run_now(&mut p).unwrap();
    let text = ws.setup_post_text(&p, 0).unwrap().unwrap();
    assert!(text.contains("(SHOP HEADER)"), "{text}");
    // save and reopen on a "machine" whose library has never heard of the post
    let dir = std::env::temp_dir().join(format!("kw_cam_lib_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    p.save_as(&dir.join("job.kwf")).unwrap();
    let back = Project::open(&dir.join("job.kwf")).unwrap();
    let fresh = CamWorkspace::default();
    assert!(fresh.library.post("Shop").is_none());
    let text2 = fresh.setup_post_text(&back, 0).unwrap().unwrap();
    assert_eq!(text, text2);
    assert_eq!(fresh.post_id_of(&back.cam[0]), "Shop");
    // updating from a library without it keeps the copy; from one that has a newer version, takes it
    let mut back = back;
    let mut fresh = fresh;
    // (the machine, which the library has, is embedded now; the post it lacks is left as it is)
    assert_eq!(fresh.refresh_from_library(&mut back), 1);
    assert!(back.cam[0].machine_def.is_some());
    assert!(fresh
        .setup_post_text(&back, 0)
        .unwrap()
        .unwrap()
        .contains("SHOP HEADER"));
    let newer = shop.replace("SHOP HEADER", "NEW HEADER");
    fresh.library.put_post(PostDef::from_text(&newer).unwrap());
    assert_eq!(fresh.refresh_from_library(&mut back), 1);
    assert!(fresh
        .setup_post_text(&back, 0)
        .unwrap()
        .unwrap()
        .contains("NEW HEADER"));
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn feeds_and_speeds_follow_the_material_the_tool_and_the_machine() {
    let lib = kw_materials::global();
    let alu = lib
        .all()
        .iter()
        .find(|m| m.cam.group == kw_materials::MaterialGroup::Aluminium)
        .expect("an aluminium")
        .name
        .clone();
    let mut p = block(Some(&alu));
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    pocket(&mut ws, 10.0);
    // the 3-axis mill tops out at 12 000 rpm; 450 m/min on 10 mm would want 14 324
    let msg = ws.feeds_from_material(&mut p).unwrap();
    assert!(
        msg.contains("Aluminium") && msg.contains("held to the machine"),
        "{msg}"
    );
    assert_eq!(ws.settings.rpm, 12_000.0);
    // 0.015 · 10 mm per tooth, two flutes, at 12 000 rpm
    assert!(
        (ws.settings.feed - 12_000.0 * 2.0 * 0.15).abs() < 1.0,
        "{}",
        ws.settings.feed
    );
    // a tool with its own data for the group wins
    let mut d = ws.library.tools[0].clone();
    d.name = "special".into();
    d.tool = Tool::flat(1, 10.0, 25.0);
    d.tool.name = "special".into();
    d.cutting = vec![kw_cam::library::CutData {
        group: "Aluminium".into(),
        vc: 200.0,
        fz: 0.05,
    }];
    ws.library.put_tool(d.clone());
    ws.settings.preset = Some(d.tool);
    let msg = ws.feeds_from_material(&mut p).unwrap();
    assert!(msg.contains("tool's own"), "{msg}");
    assert!((ws.settings.rpm - 1000.0 * 200.0 / (std::f64::consts::PI * 10.0)).abs() < 1.0);
    // a body without a material says so
    let mut p2 = block(None);
    let e = ws.feeds_from_material(&mut p2).unwrap_err();
    assert!(e.contains("no material"), "{e}");
    let _ = (BodyKey::Assembly(String::new()), Library::builtin());
}
