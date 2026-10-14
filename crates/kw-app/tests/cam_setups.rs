//! Setups in the CAM workspace: bodies and their roles, work coordinates, stock from the model, a second
//! setup that starts from what the first left, fixtures that the tools must clear, operations that know
//! when they are out of date, and the per-setup programs and setup sheet.

use kw_app::cam::{setup, sim, CamWorkspace, OpKind, ToolChoice};
use kw_app::model_source::ModelSource;
use kw_app::project::Project;
use kw_cam::{RegionSpec, StockSpec};
use kw_store::{BodyRole, SourceKind};

fn mm(v: f64) -> kw_expr::Param {
    kw_expr::Param::length(v)
}

/// Blocks (x0, x1, y0, y1, z0, z1) as separate bodies, in order.
fn bodies(blocks: &[(f64, f64, f64, f64, f64, f64)]) -> Project {
    let mut p = Project::empty();
    p.history
        .edit(|d| {
            for (k, (x0, x1, y0, y1, z0, z1)) in blocks.iter().enumerate() {
                let s = d.add_rect_sketch(
                    &format!("S{k}"),
                    kw_feature::PlaneSpec::Xy(mm(*z0)),
                    mm(*x0),
                    mm(*y0),
                    mm(x1 - x0),
                    mm(y1 - y0),
                )?;
                d.add_extrude(
                    &format!("Block{k}"),
                    s,
                    mm(z1 - z0),
                    kw_feature::BodyOp::New,
                )?;
            }
            Ok(())
        })
        .unwrap();
    p
}

fn source(p: &mut Project) -> ModelSource {
    ModelSource::of(p, SourceKind::Design, &Default::default()).unwrap()
}

fn bounds(tris: &setup::Tris) -> ([f64; 3], [f64; 3]) {
    let b = kw_cam::stock::bounds_of(tris);
    (b.min, b.max)
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6)
}

fn wait_sim(ws: &mut CamWorkspace, p: &mut Project) {
    let t = std::time::Instant::now();
    ws.start_sim(p);
    // the workspace polls its jobs when it draws; run the same poll here through a headless frame
    while ws.sim.is_none() {
        poll(ws, p);
        assert!(t.elapsed().as_secs() < 300, "{}", ws.message);
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn poll(ws: &mut CamWorkspace, p: &mut Project) {
    ws.poll_jobs(p);
}

#[test]
fn a_setup_machines_the_workpiece_bodies_and_keeps_fixtures_apart() {
    // the part, and a vise jaw beside it
    let mut p = bodies(&[
        (0.0, 40.0, 0.0, 25.0, 0.0, 10.0),
        (-12.0, -2.0, -5.0, 30.0, 0.0, 6.0),
    ]);
    let src = source(&mut p);
    assert_eq!(src.bodies.len(), 2);
    let mut s = kw_store::CamSetup::default();
    // everything is a workpiece until told otherwise
    let g = setup::geometry(&src, &s).unwrap();
    assert!(near(bounds(&g.workpiece).0, [-12.0, -5.0, 0.0]));
    assert!(g.fixtures.is_empty());
    s.set_role(&src.bodies[1].key, BodyRole::Fixture);
    let g = setup::geometry(&src, &s).unwrap();
    let (lo, hi) = bounds(&g.workpiece);
    assert!(
        near(lo, [0.0, 0.0, 0.0]) && near(hi, [40.0, 25.0, 10.0]),
        "{lo:?} {hi:?}"
    );
    let (lo, hi) = bounds(&g.fixtures);
    assert!(near(lo, [-12.0, -5.0, 0.0]) && near(hi, [-2.0, 30.0, 6.0]));
    // the flat faces of the part: its top and its bottom
    let ups: Vec<f64> = g.faces.iter().filter(|f| f.up).map(|f| f.z).collect();
    assert!(ups.contains(&10.0), "{ups:?}");
    // an ignored body is gone
    s.set_role(&src.bodies[1].key, BodyRole::Ignore);
    let g = setup::geometry(&src, &s).unwrap();
    assert!(g.fixtures.is_empty());
    // a setup with nothing to machine is an error, not an empty job
    s.set_role(&src.bodies[0].key, BodyRole::Ignore);
    assert!(setup::geometry(&src, &s).is_err());
}

#[test]
fn work_coordinates_put_the_origin_on_the_top_corner_as_seen_from_each_side() {
    let mut p = bodies(&[(5.0, 45.0, 10.0, 35.0, 2.0, 12.0)]);
    let mut ws = CamWorkspace::default();
    let src = {
        ws.fit_to_workpiece(&mut p);
        ws.source(&mut p).unwrap()
    };
    // from the top: the corner at x 5, y 10, z 12, and the part is 40 x 25 x 10 below it
    ws.place_origin(&mut p);
    let s = &p.cam[0];
    assert!(near(s.wcs.origin, [5.0, 10.0, 12.0]), "{:?}", s.wcs.origin);
    let (lo, hi) = CamWorkspace::workpiece_bounds(&src, s).unwrap();
    assert!(
        near(lo, [0.0, 0.0, -10.0]) && near(hi, [40.0, 25.0, 0.0]),
        "{lo:?} {hi:?}"
    );
    // from the bottom the tool points up the other way round and the part hangs from its top
    ws.set_orientation(&mut p, 1);
    let s = &p.cam[0];
    let (lo, hi) = CamWorkspace::workpiece_bounds(&src, s).unwrap();
    assert!(
        near(lo, [0.0, 0.0, -10.0]) && near(hi, [40.0, 25.0, 0.0]),
        "{lo:?} {hi:?}"
    );
    assert!(near(s.wcs.origin, [5.0, 35.0, 2.0]), "{:?}", s.wcs.origin);
    // every orientation keeps the frame right-handed and the part below the origin plane
    for i in 0..6 {
        ws.set_orientation(&mut p, i);
        let s = &p.cam[0];
        let f = setup::frame_of(&s.wcs);
        let cross = [
            f.u[1] * f.v[2] - f.u[2] * f.v[1],
            f.u[2] * f.v[0] - f.u[0] * f.v[2],
            f.u[0] * f.v[1] - f.u[1] * f.v[0],
        ];
        assert!(near(cross, f.w), "orientation {i}");
        let (_, hi) = CamWorkspace::workpiece_bounds(&src, s).unwrap();
        assert!(hi[2].abs() < 1e-6, "orientation {i}: top at {}", hi[2]);
    }
}

#[test]
fn stock_follows_the_workpiece_and_its_specification() {
    let mut p = bodies(&[(0.0, 40.0, 0.0, 25.0, 0.0, 10.0)]);
    let src = source(&mut p);
    let mut s = kw_store::CamSetup {
        stock: StockSpec::Box {
            side: 2.0,
            top: 3.0,
            bottom: 0.0,
        },
        ..Default::default()
    };
    let g = setup::geometry(&src, &s).unwrap();
    let st = sim::stock_of(&s, &g, None).unwrap();
    assert!(near(st.min, [-2.0, -2.0, 0.0]) && near(st.max, [42.0, 27.0, 13.0]));
    s.stock = StockSpec::Cylinder {
        axis: 2,
        margin: 1.0,
        ends: 0.0,
    };
    let st = sim::stock_of(&s, &g, None).unwrap();
    let r = (20.0f64.powi(2) + 12.5f64.powi(2)).sqrt() + 1.0;
    assert!((st.max[0] - st.min[0] - 2.0 * r).abs() < 1e-6);
    s.stock = StockSpec::Previous;
    assert!(sim::stock_of(&s, &g, None).is_err());
}

fn pocket_settings(ws: &mut CamWorkspace, region: [f64; 4], top: f64, bottom: f64) {
    let s = &mut ws.settings;
    s.op = OpKind::Pocket;
    s.tool = ToolChoice::Flat;
    s.diameter = 6.0;
    (s.x0, s.y0, s.x1, s.y1) = (region[0], region[1], region[2], region[3]);
    s.region = None;
    s.top = top;
    s.bottom = bottom;
    s.step_down = 2.0;
    s.stepover = 4.0;
}

#[test]
fn a_second_setup_starts_from_what_the_first_one_left() {
    // a 40 x 25 x 10 part; the stock is 4 mm too tall on both faces
    let mut p = bodies(&[(0.0, 40.0, 0.0, 25.0, 0.0, 10.0)]);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    p.cam[0].stock = StockSpec::Explicit {
        min: [0.0, 0.0, -4.0],
        max: [40.0, 25.0, 14.0],
    };
    // setup 1: face the top (z 10 to 14)
    pocket_settings(&mut ws, [0.0, 0.0, 40.0, 25.0], 14.0, 10.0);
    ws.run_now(&mut p).unwrap();
    wait_sim(&mut ws, &mut p);
    let (_, _, r1) = ws.sim.clone().unwrap();
    let want = 40.0 * 25.0 * 4.0;
    let removed = r1.stock_volume - r1.final_volume;
    assert!((removed - want).abs() < 0.04 * want, "{removed} vs {want}");
    // 4 mm of waste is still on the bottom
    assert!(
        (r1.remaining - want).abs() < 0.04 * want,
        "{}",
        r1.remaining
    );
    assert!(r1.overcut < 0.01 * want, "{}", r1.overcut);
    assert!(r1.ops.iter().all(|o| !o.collides()), "{:?}", r1.ops);
    // setup 2: flipped, starting from the first setup's result
    ws.new_setup(&mut p);
    ws.set_orientation(&mut p, 1);
    p.cam[1].stock = StockSpec::Previous;
    assert_eq!(p.cam[1].wcs.offset, 2);
    ws.fit_to_workpiece(&mut p);
    // in the flipped frame the part's face is at z 0 and the waste is above it
    pocket_settings(&mut ws, [0.0, 0.0, 40.0, 25.0], 4.0, 0.0);
    ws.run_now(&mut p).unwrap();
    ws.sim = None;
    wait_sim(&mut ws, &mut p);
    let (i, _, r2) = ws.sim.clone().unwrap();
    assert_eq!(i, 1);
    // it began where the first setup ended
    assert!(
        (r2.stock_volume - r1.final_volume).abs() < 0.01 * r1.final_volume,
        "{} vs {}",
        r2.stock_volume,
        r1.final_volume
    );
    let removed2 = r2.stock_volume - r2.final_volume;
    assert!(
        (removed2 - want).abs() < 0.04 * want,
        "{removed2} vs {want}"
    );
    // and what is left is the part
    assert!(r2.remaining < 0.04 * want, "{}", r2.remaining);
    assert!(r2.overcut < 0.01 * want, "{}", r2.overcut);
    assert!(
        (r2.final_volume - 40.0 * 25.0 * 10.0).abs() < 0.02 * 10000.0,
        "{}",
        r2.final_volume
    );
}

#[test]
fn the_tools_are_checked_against_a_fixture_beside_the_part() {
    // a jaw 2 mm from the part's side
    let mut p = bodies(&[
        (0.0, 40.0, 0.0, 25.0, 0.0, 10.0),
        (-12.0, -2.0, -5.0, 30.0, 0.0, 8.0),
    ]);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    let src = ws.source(&mut p).unwrap();
    // contour the part: the tool centre runs 3 mm outside the walls, through the jaw
    ws.settings.op = OpKind::ContourOutside;
    ws.settings.diameter = 6.0;
    ws.settings.step_down = 5.0;
    // without roles both bodies are workpiece: nothing to collide with
    p.cam[0].set_role(&src.bodies[1].key, BodyRole::Fixture);
    ws.fit_to_workpiece(&mut p);
    ws.settings.top = 10.0;
    ws.settings.bottom = 2.0;
    ws.run_now(&mut p).unwrap();
    wait_sim(&mut ws, &mut p);
    let (_, _, r) = ws.sim.clone().unwrap();
    let o = &r.ops[0];
    assert!(o.collides(), "{o:?}");
    assert!(o.fixture_gouges > 0 || o.shank_collisions > 0, "{o:?}");
    // clear of the jaw: ignore it
    p.cam[0].set_role(&src.bodies[1].key, BodyRole::Ignore);
    ws.sim = None;
    wait_sim(&mut ws, &mut p);
    let (_, _, r) = ws.sim.clone().unwrap();
    assert!(!r.ops[0].collides(), "{:?}", r.ops[0]);
}

#[test]
fn an_operation_knows_when_the_model_or_its_tool_changed() {
    let mut p = bodies(&[(0.0, 40.0, 0.0, 25.0, 0.0, 10.0)]);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    pocket_settings(&mut ws, [5.0, 5.0, 35.0, 20.0], 10.0, 7.0);
    ws.run_now(&mut p).unwrap();
    assert!(ws.stale_ops(&mut p).is_empty());
    let first = p.cam[0].operations[0].path.clone();
    // a legacy operation without settings is never stale
    p.cam[0].operations.push(kw_store::CamOperation {
        name: "imported".into(),
        tool: 1,
        path: first.clone(),
        params: None,
        input_hash: 0,
    });
    assert!(ws.stale_ops(&mut p).is_empty());
    // change the model: add a second block
    p.history
        .edit(|d| {
            let s = d.add_rect_sketch(
                "Extra",
                kw_feature::PlaneSpec::Xy(mm(0.0)),
                mm(60.0),
                mm(0.0),
                mm(10.0),
                mm(10.0),
            )?;
            d.add_extrude("Extra block", s, mm(5.0), kw_feature::BodyOp::New)?;
            Ok(())
        })
        .unwrap();
    assert_eq!(ws.stale_ops(&mut p), vec![0]);
    // regenerating brings it up to date and keeps the settings
    ws.start_regenerate(&mut p);
    ws.wait_generation(&mut p);
    assert!(ws.stale_ops(&mut p).is_empty());
    assert_eq!(p.cam[0].operations[0].path, first);
    // a different workpiece role changes the geometry it was made for
    let src = ws.source(&mut p).unwrap();
    p.cam[0].set_role(&src.bodies[1].key, BodyRole::Ignore);
    assert_eq!(ws.stale_ops(&mut p), vec![0]);
    ws.start_regenerate(&mut p);
    ws.wait_generation(&mut p);
    // a changed work offset does not change the toolpath, but a moved origin does
    assert!(ws.stale_ops(&mut p).is_empty());
    p.cam[0].wcs.origin[2] += 1.0;
    assert_eq!(ws.stale_ops(&mut p), vec![0]);
}

#[test]
fn each_setup_posts_under_its_own_work_offset_and_the_sheet_lists_them() {
    let mut p = bodies(&[(0.0, 40.0, 0.0, 25.0, 0.0, 10.0)]);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    pocket_settings(&mut ws, [5.0, 5.0, 35.0, 20.0], 10.0, 7.0);
    ws.run_now(&mut p).unwrap();
    ws.settings.diameter = 8.0;
    ws.run_now(&mut p).unwrap();
    ws.new_setup(&mut p);
    ws.set_orientation(&mut p, 1);
    ws.fit_to_workpiece(&mut p);
    pocket_settings(&mut ws, [5.0, 5.0, 35.0, 20.0], 10.0, 8.0);
    ws.run_now(&mut p).unwrap();
    // two tools in the first setup get different numbers
    let tools: Vec<u32> = p.cam[0].tools.iter().map(|t| t.number).collect();
    assert_eq!(tools, vec![1, 2]);
    let dir = std::env::temp_dir().join(format!("kw_cam_setups_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for d in kw_app::cam::BUILTIN_POSTS {
        for i in 0..2 {
            ws.current = i;
            ws.set_post(&mut p, d).unwrap();
        }
        let gd = match d {
            "Fanuc" => kw_gsim::Dialect::Fanuc,
            "Haas" => kw_gsim::Dialect::Haas,
            "Siemens840D" => kw_gsim::Dialect::Siemens840D,
            "Heidenhain" => kw_gsim::Dialect::Heidenhain,
            _ => kw_gsim::Dialect::LinuxCnc,
        };
        for (i, want) in [(0usize, ["G54", "Q339=1"]), (1, ["G55", "Q339=2"])] {
            let text = ws.setup_post_text(&p, i).unwrap().unwrap();
            let marker = if d == "Heidenhain" { want[1] } else { want[0] };
            assert!(
                text.contains(marker),
                "{d:?} setup {i}: {marker} missing\n{text}"
            );
            let run = kw_gsim::run(gd, &text).unwrap_or_else(|e| panic!("{d:?} setup {i}: {e}"));
            assert!(run.program_end_seen);
            if i == 0 {
                assert_eq!(run.tools_used, vec![1, 2], "{d:?}");
            }
        }
    }
    // the programs go to files, one per setup
    for i in 0..2 {
        ws.current = i;
        ws.set_post(&mut p, "Fanuc").unwrap();
    }
    let msg = ws.write_all(&p, &dir);
    assert!(msg.contains("2 program"), "{msg}");
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 2);
    // the setup sheet
    let sheet = dir.join("sheet.html");
    let msg = ws.write_sheet(&mut p, &sheet);
    assert!(msg.contains("wrote"), "{msg}");
    let html = std::fs::read_to_string(&sheet).unwrap();
    for needle in [
        "Setup 1",
        "Setup 2",
        "G54",
        "G55",
        "T1",
        "T2",
        "6 mm flat end mill",
        "Pocket 1",
    ] {
        assert!(html.contains(needle), "the sheet lacks {needle}");
    }
    let csv = std::fs::read_to_string(sheet.with_extension("csv")).unwrap();
    assert_eq!(csv.lines().count(), 4, "{csv}");
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn regions_come_from_the_outline_of_a_face() {
    // a block with a 20 x 10 pocket 4 mm deep: cut from a sketch on the floor level upwards
    let mut p = bodies(&[(0.0, 40.0, 0.0, 25.0, 0.0, 10.0)]);
    p.history
        .edit(|d| {
            let s = d.add_rect_sketch(
                "Pocket",
                kw_feature::PlaneSpec::Xy(mm(6.0)),
                mm(10.0),
                mm(7.0),
                mm(20.0),
                mm(10.0),
            )?;
            d.add_extrude("Cut", s, mm(4.0), kw_feature::BodyOp::Cut)?;
            Ok(())
        })
        .unwrap();
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    let g = ws.geometry(&mut p).unwrap();
    let floor = g
        .faces
        .iter()
        .find(|f| f.up && (f.z - 6.0).abs() < 1e-6)
        .unwrap_or_else(|| {
            panic!(
                "no floor at z 6: {:?}",
                g.faces.iter().map(|f| f.z).collect::<Vec<_>>()
            )
        });
    assert!((floor.area - 200.0).abs() < 1e-6, "{}", floor.area);
    assert_eq!(floor.outer.len(), 4);
    // machine it: the pocket tool path stays inside the walls
    let spec = RegionSpec::Polygon {
        outer: floor.outer.clone(),
        islands: floor.islands.clone(),
    };
    let b = spec.bounds();
    pocket_settings(&mut ws, b, 10.0, 6.0);
    ws.settings.region = Some(spec);
    ws.run_now(&mut p).unwrap();
    let op = &p.cam[0].operations[0];
    assert!(matches!(
        op.params.as_ref().unwrap().region,
        RegionSpec::Polygon { .. }
    ));
    for (a, c) in kw_app::cam::cut_segments(&op.path) {
        for q in [a, c] {
            assert!(
                q[0] >= 10.0 - 1e-6
                    && q[0] <= 30.0 + 1e-6
                    && q[1] >= 7.0 - 1e-6
                    && q[1] <= 17.0 + 1e-6,
                "{q:?} outside the pocket"
            );
        }
    }
    // the part has the pocket, so cutting it into the stock leaves nothing behind and no overcut
    p.cam[0].stock = StockSpec::Explicit {
        min: [0.0, 0.0, 0.0],
        max: [40.0, 25.0, 10.0],
    };
    wait_sim(&mut ws, &mut p);
    let (_, _, r) = ws.sim.clone().unwrap();
    assert!(r.remaining < 0.06 * 800.0, "{}", r.remaining);
    assert!(r.overcut < 0.06 * 800.0, "{}", r.overcut);
}

#[test]
fn setups_are_saved_and_reopened_with_their_operations_and_roles() {
    let mut p = bodies(&[
        (0.0, 40.0, 0.0, 25.0, 0.0, 10.0),
        (-12.0, -2.0, -5.0, 30.0, 0.0, 6.0),
    ]);
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    let src = ws.source(&mut p).unwrap();
    p.cam[0].set_role(&src.bodies[1].key, BodyRole::Fixture);
    ws.fit_to_workpiece(&mut p);
    pocket_settings(&mut ws, [5.0, 5.0, 35.0, 20.0], 10.0, 7.0);
    ws.run_now(&mut p).unwrap();
    ws.new_setup(&mut p);
    ws.set_orientation(&mut p, 1);
    p.cam[1].stock = StockSpec::Previous;
    let dir = std::env::temp_dir().join(format!("kw_cam_saved_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join("job.kwf");
    p.save_as(&file).unwrap();
    let back = Project::open(&file).unwrap();
    assert_eq!(back.cam, p.cam);
    assert_eq!(back.cam[0].role_of(&src.bodies[1].key), BodyRole::Fixture);
    assert_eq!(back.cam[1].stock, StockSpec::Previous);
    std::fs::remove_dir_all(&dir).ok();
}
