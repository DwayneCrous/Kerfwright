//! The CAM libraries: round trips through JSON and the user folder, feeds and speeds against hand
//! calculation, posts from user text, and the cone tools in the stock simulation.

use kw_cam::dexel::TriDexel;
use kw_cam::library::*;
use kw_cam::path::{Event, Toolpath};
use kw_cam::post::{render, PostConfig};
use kw_cam::{Tool, ToolKind};

fn tempdir(name: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("kw_camlib_{name}_{}", std::process::id()));
    std::fs::remove_dir_all(&d).ok();
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn the_built_in_library_is_consistent() {
    let lib = Library::builtin();
    assert!(lib.tools.len() > 30);
    // names are unique, every machine's post exists, every post parses
    for (what, names) in [
        (
            "tool",
            lib.tools.iter().map(|t| t.name.clone()).collect::<Vec<_>>(),
        ),
        (
            "holder",
            lib.holders.iter().map(|t| t.name.clone()).collect(),
        ),
        (
            "machine",
            lib.machines.iter().map(|t| t.name.clone()).collect(),
        ),
        ("post", lib.posts.iter().map(|t| t.id.clone()).collect()),
    ] {
        let mut sorted = names.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(
            sorted.len(),
            names.len(),
            "duplicate {what} names: {names:?}"
        );
    }
    for m in &lib.machines {
        assert!(
            lib.post_config(&m.post).is_ok(),
            "{}: post {}",
            m.name,
            m.post
        );
    }
    assert_eq!(lib.posts.len(), 5);
    assert_eq!(lib.machines.iter().filter(|m| m.axes() == 5).count(), 5);
    assert!(lib.machine("3-axis mill").unwrap().kinematics.is_none());
    // every kind of tool is offered
    let kinds: Vec<&str> = lib.tools.iter().map(|t| t.tool.kind.label()).collect();
    for k in [
        "Flat end mill",
        "Ball end mill",
        "Bull-nose end mill",
        "Drill",
        "Chamfer mill",
        "Face mill",
        "Engraver",
        "Boring bar",
        "Tap",
    ] {
        assert!(kinds.contains(&k), "no {k}");
    }
}

#[test]
fn every_library_type_survives_json() {
    let lib = Library::builtin();
    let tools: Vec<ToolDef> = serde_json::from_str(&lib.export_tools()).unwrap();
    assert_eq!(tools, lib.tools);
    let holders: Vec<HolderDef> = serde_json::from_str(&lib.export_holders()).unwrap();
    assert_eq!(holders, lib.holders);
    for m in &lib.machines {
        let back: MachineDef = serde_json::from_str(&serde_json::to_string(m).unwrap()).unwrap();
        assert_eq!(&back, m);
    }
    for p in &lib.posts {
        let back = PostDef::from_text(&p.text).unwrap();
        assert_eq!(&back, p);
    }
    // a tool with cutting data
    let d = ToolDef {
        name: "my cutter".into(),
        tool: Tool::chamfer(7, 8.0, 60.0, 0.5, 10.0),
        cutting: vec![CutData {
            group: "Aluminium".into(),
            vc: 400.0,
            fz: 0.04,
        }],
    };
    let back: ToolDef = serde_json::from_str(&serde_json::to_string(&d).unwrap()).unwrap();
    assert_eq!(back, d);
    // a tool stored before flutes, material and the rest existed still reads
    let old = r#"{"number":3,"name":"old","kind":"Flat","diameter":6.0,"flute_length":20.0,
        "shank_radius":3.0,"shank_length":20.0,"holder":{"sections":[[20.0,40.0]]}}"#;
    let t: Tool = serde_json::from_str(old).unwrap();
    assert_eq!((t.flutes, t.material.as_str()), (2, ""));
}

#[test]
fn feeds_and_speeds_match_the_hand_calculation() {
    // 10 mm four-flute end mill, 200 m/min, 0.06 mm per tooth: n = 1000 · 200 / (π · 10) = 6366.2 rpm
    let mut t = Tool::flat(1, 10.0, 25.0);
    t.flutes = 4;
    let s = speeds_and_feeds(&t, 200.0, 0.06, None);
    assert!((s.rpm - 6366.198).abs() < 0.01, "{}", s.rpm);
    assert!((s.feed - 6366.198 * 4.0 * 0.06).abs() < 0.01, "{}", s.feed);
    assert!(!s.limited);
    // a spindle that tops out at 5000 rpm cuts the feed back with the speed
    let mut m = Library::builtin().machine("3-axis mill").unwrap().clone();
    m.spindle_max = 5000.0;
    m.max_feed = 10_000.0;
    let s = speeds_and_feeds(&t, 200.0, 0.06, Some(&m));
    assert!(s.limited);
    assert!((s.rpm - 5000.0).abs() < 1e-9 && (s.feed - 5000.0 * 4.0 * 0.06).abs() < 1e-6);
    // and a machine that cannot feed that fast holds the feed
    m.max_feed = 900.0;
    let s = speeds_and_feeds(&t, 200.0, 0.06, Some(&m));
    assert_eq!(s.feed, 900.0);
    // a 3 mm tool in aluminium wants more than a small spindle gives: held at the top speed
    let small = Tool::flat(1, 3.0, 10.0);
    let s = speeds_and_feeds(&small, 400.0, 0.02, Some(&m));
    assert_eq!(s.rpm, 5000.0);
}

#[test]
fn a_user_post_is_checked_when_it_is_read() {
    let ok = PostDef::from_text(&FANUC_TEXT.replace("name = Fanuc", "name = Shop Fanuc")).unwrap();
    assert_eq!(ok.id, "Shop Fanuc");
    let e = PostDef::from_text("name = X\ndecimals = three").unwrap_err();
    assert!(e.contains("line 2") && e.contains("decimals"), "{e}");
    let e = PostDef::from_text("name = X\nbogus_key = 1").unwrap_err();
    assert!(e.contains("line 2") && e.contains("bogus_key"), "{e}");
    let e = PostDef::from_text("decimals = 3").unwrap_err();
    assert!(e.contains("no name"), "{e}");
    let e = PostDef::from_text("name = X\nthis line has no equals sign").unwrap_err();
    assert!(e.contains("line 2"), "{e}");
    // a user post writes programs like any other
    let mut lib = Library::builtin();
    lib.import_post(&ok.text).unwrap();
    let cfg = lib.post_config("Shop Fanuc").unwrap();
    let mut tp = Toolpath::new("t");
    tp.push(Event::WorkOffset { number: 3 });
    tp.push(Event::Rapid { p: [1.0, 2.0, 3.0] });
    tp.push(Event::ProgramEnd);
    let text = render(&cfg, &tp).unwrap();
    assert!(text.contains("G56") && text.contains("G00"), "{text}");
    assert!(lib.post_config("nothing").is_err());
}

const FANUC_TEXT: &str = kw_cam::post::FANUC;

#[test]
fn the_users_entries_are_saved_and_read_back_over_the_built_in_ones() {
    let dir = tempdir("user");
    let (lib, warnings) = Library::load_dir(&dir);
    assert!(warnings.is_empty());
    assert_eq!(lib, Library::builtin());
    let mut lib = lib;
    lib.put_tool(ToolDef {
        name: "shop 6 mm roughing".into(),
        tool: Tool::flat(9, 6.0, 18.0),
        cutting: vec![CutData {
            group: "Steel".into(),
            vc: 120.0,
            fz: 0.03,
        }],
    });
    // changing a built-in entry keeps the user's version
    let mut er32 = lib.holder("ER32 collet chuck").unwrap().clone();
    er32.sections = vec![(24.0, 50.0)];
    lib.put_holder(er32.clone());
    let mut m = lib.machine("Haas VF-2").unwrap().clone();
    m.spindle_max = 10_000.0;
    lib.put_machine(m.clone());
    lib.import_post(&FANUC_TEXT.replace("name = Fanuc", "name = Shop"))
        .unwrap();
    lib.save_dir(&dir).unwrap();
    // only the changes are on disk
    let tools: Vec<ToolDef> =
        serde_json::from_str(&std::fs::read_to_string(dir.join("tools.json")).unwrap()).unwrap();
    assert_eq!(tools.len(), 1);
    assert_eq!(std::fs::read_dir(dir.join("machines")).unwrap().count(), 1);
    assert_eq!(std::fs::read_dir(dir.join("posts")).unwrap().count(), 1);
    let (back, warnings) = Library::load_dir(&dir);
    assert!(warnings.is_empty(), "{warnings:?}");
    assert_eq!(back.tool("shop 6 mm roughing").unwrap().tool.number, 9);
    assert_eq!(back.holder("ER32 collet chuck"), Some(&er32));
    assert_eq!(back.machine("Haas VF-2").unwrap().spindle_max, 10_000.0);
    assert!(back.post("Shop").is_some());
    assert_eq!(back.tools.len(), lib.tools.len());
    // deleting an entry from the library and saving removes its file
    let mut lib2 = back;
    lib2.posts.retain(|p| p.id != "Shop");
    lib2.save_dir(&dir).unwrap();
    assert_eq!(std::fs::read_dir(dir.join("posts")).unwrap().count(), 0);
    // a damaged file is reported and skipped; the rest still loads
    std::fs::write(dir.join("holders.json"), "[ not json").unwrap();
    std::fs::write(dir.join("posts").join("bad.post"), "name = Bad\nzzz = 1").unwrap();
    let (loaded, warnings) = Library::load_dir(&dir);
    assert_eq!(warnings.len(), 2, "{warnings:?}");
    assert!(warnings.iter().any(|w| w.contains("holders.json")));
    assert!(warnings
        .iter()
        .any(|w| w.contains("bad.post") && w.contains("line 2")));
    assert!(loaded.tool("shop 6 mm roughing").is_some());
    assert!(loaded.post("Bad").is_none());
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn imports_replace_by_name_and_report_bad_files() {
    let mut lib = Library::builtin();
    let n = lib.tools.len();
    let json = lib.export_tools();
    assert_eq!(lib.import_tools(&json).unwrap(), n);
    assert_eq!(
        lib.tools.len(),
        n,
        "importing what is there changes nothing"
    );
    assert!(lib.import_tools("{").is_err());
    assert!(lib.import_machine("[]").is_err());
    assert!(lib
        .import_post("name = X\nqq = 1")
        .unwrap_err()
        .contains("line 2"));
    let mut numbers = vec![
        Tool::flat(0, 6.0, 20.0),
        Tool::flat(2, 6.0, 20.0),
        Tool::flat(2, 8.0, 20.0),
        Tool::flat(1, 8.0, 20.0),
    ];
    renumber(&mut numbers);
    let got: Vec<u32> = numbers.iter().map(|t| t.number).collect();
    assert_eq!(got, vec![1, 2, 3, 4]);
}

#[test]
fn a_drill_cuts_a_cylinder_with_a_conical_bottom() {
    let drill = Tool::drill(1, 6.0, 118.0, 30.0);
    assert!(matches!(drill.kind, ToolKind::Drill { .. }));
    let mut d = TriDexel::block([-10.0, -10.0, -14.0], [10.0, 10.0, 0.0], 0.1);
    let mut tp = Toolpath::new("drill");
    tp.push(Event::Rapid { p: [0.0, 0.0, 2.0] });
    tp.push(Event::Feed {
        p: [0.0, 0.0, -10.0],
        feed: 100.0,
    });
    tp.push(Event::Feed {
        p: [0.0, 0.0, 2.0],
        feed: 100.0,
    });
    d.run(&drill, &tp);
    let removed = d.initial_volume() - d.volume()[0];
    // cylinder down to where the cone meets the full diameter, plus the cone
    let half = 59f64.to_radians();
    let cone_h = 3.0 / half.tan();
    let want =
        std::f64::consts::PI * 9.0 * (10.0 - cone_h) + std::f64::consts::PI * 9.0 * cone_h / 3.0;
    assert!((removed - want).abs() < 0.02 * want, "{removed} vs {want}");
    // a chamfer mill leaves a 45 degree wall: a 90 degree tool buried to depth h removes a cone of
    // radius tip_r + h
    let ch = Tool::chamfer(2, 12.0, 90.0, 0.0, 10.0);
    let mut d = TriDexel::block([-10.0, -10.0, -14.0], [10.0, 10.0, 0.0], 0.1);
    let mut tp = Toolpath::new("chamfer");
    tp.push(Event::Rapid { p: [0.0, 0.0, 2.0] });
    tp.push(Event::Feed {
        p: [0.0, 0.0, -4.0],
        feed: 100.0,
    });
    d.run(&ch, &tp);
    let removed = d.initial_volume() - d.volume()[0];
    // the cone: radius 4 at the surface (z = 0), apex at z = -4
    let want = std::f64::consts::PI * 16.0 * 4.0 / 3.0;
    assert!((removed - want).abs() < 0.03 * want, "{removed} vs {want}");
    // and a PostConfig still parses the built-in text (guards the include path)
    PostConfig::parse(FANUC_TEXT).unwrap();
}

// ---- the tilted tool in the stock simulation

use kw_cam::path::Event as Ev;

fn plunge(tool: &Tool, tilt_deg: f64, depth: f64, h: f64) -> (TriDexel, f64) {
    let mut d = TriDexel::block([-20.0, -20.0, -15.0], [20.0, 20.0, 0.0], h);
    let t = tilt_deg.to_radians();
    let axis = [t.sin(), 0.0, t.cos()];
    let mut tp = Toolpath::new("plunge");
    // approach along the axis from outside the stock and go down to the tip depth
    let tip = [0.0, 0.0, -depth];
    let from = [tip[0] + 8.0 * axis[0], tip[1], tip[2] + 8.0 * axis[2]];
    tp.push(Ev::Pose {
        p: from,
        axis,
        feed: None,
    });
    tp.push(Ev::Pose {
        p: tip,
        axis,
        feed: Some(100.0),
    });
    let removed = d.run(tool, &tp);
    (d, removed)
}

#[test]
fn a_tilted_flat_tool_cuts_an_oblique_cylinder() {
    let tool = Tool::flat(1, 6.0, 25.0);
    // an oblique cylinder cut by the plane z = 0 has the volume of the cylinder along its axis to the plane
    // through its centre: pi R² d / cos(tilt)
    for (tilt, depth) in [(0.0, 5.0), (30.0, 5.0), (45.0, 4.0)] {
        let (d, _) = plunge(&tool, tilt, depth, 0.1);
        let removed = d.initial_volume() - d.volume()[0];
        let want = std::f64::consts::PI * 9.0 * depth / tilt.to_radians().cos();
        assert!(
            (removed - want).abs() < 0.03 * want,
            "tilt {tilt}: {removed} vs {want}"
        );
    }
}

#[test]
fn the_three_ray_families_agree_on_a_tilted_cut() {
    let tool = Tool::ball(1, 6.0, 25.0);
    let (d, _) = plunge(&tool, 25.0, 6.0, 0.1);
    let v = d.volume();
    // the z, x and y rays measure the same stock
    assert!((v[0] - v[1]).abs() < 0.01 * v[0], "{v:?}");
    assert!((v[0] - v[2]).abs() < 0.01 * v[0], "{v:?}");
    // a vertical pose through the general routine matches the fast one
    let mut a = TriDexel::block([-10.0, -10.0, -10.0], [10.0, 10.0, 0.0], 0.1);
    let mut b = a.clone();
    a.cut_pose(&tool, [0.0, 0.0, -4.0]);
    b.cut_pose_axis(&tool, [0.0, 0.0, -4.0], [0.0, 0.0, 1.0]);
    let (va, vb) = (a.volume()[0], b.volume()[0]);
    assert!(
        (va - vb).abs() < 0.005 * (a.initial_volume() - va),
        "{va} vs {vb}"
    );
}

#[test]
fn a_tilted_move_with_a_vertical_one_after_it_is_cut_continuously() {
    let tool = Tool::flat(1, 4.0, 20.0);
    let mut d = TriDexel::block([-20.0, -20.0, -10.0], [20.0, 20.0, 0.0], 0.2);
    let mut tp = Toolpath::new("mixed");
    tp.push(Ev::Rapid {
        p: [-10.0, 0.0, 5.0],
    });
    tp.push(Ev::Feed {
        p: [-10.0, 0.0, -3.0],
        feed: 100.0,
    });
    tp.push(Ev::Pose {
        p: [0.0, 0.0, -3.0],
        axis: [0.0, 0.3, 0.953939],
        feed: Some(200.0),
    });
    tp.push(Ev::Feed {
        p: [10.0, 0.0, -3.0],
        feed: 200.0,
    });
    let removed = d.run(&tool, &tp);
    // 20 mm of travel through a 4 mm wide, 3 mm deep slot, plus the plunge at the start and the end caps
    let slot = 20.0 * 4.0 * 3.0 + std::f64::consts::PI * 4.0 * 3.0 * 1.0;
    assert!((removed - slot).abs() < 0.12 * slot, "{removed} vs {slot}");
    assert_eq!(d.rapid_removed, 0.0);
}
