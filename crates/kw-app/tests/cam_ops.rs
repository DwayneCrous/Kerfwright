//! The wider set of operations in the CAM workspace: holes found in the model, drilling and boring them,
//! rest finishing from what earlier operations left, and the rotary axes of a five-axis machine.

use kw_app::cam::{fiveaxis, CamWorkspace, OpKind, ToolChoice};
use kw_app::project::Project;
use kw_cam::{Event, StockSpec, Tool};

fn mm(v: f64) -> kw_expr::Param {
    kw_expr::Param::length(v)
}

/// A 40 x 25 x 10 block with a pocket and holes cut into it:
/// through holes of 5 mm at (10, 12) and (30, 12), a blind 8 mm hole 4 mm deep at (20, 19).
fn block_with_holes() -> Project {
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
            d.add_extrude("Block", s, mm(10.0), kw_feature::BodyOp::New)?;
            for (k, (x, y)) in [(10.0, 12.0), (30.0, 12.0)].iter().enumerate() {
                let c = d.add_circle_sketch(
                    &format!("Hole {k}"),
                    kw_feature::PlaneSpec::Xy(mm(0.0)),
                    mm(*x),
                    mm(*y),
                    mm(5.0),
                )?;
                d.add_extrude(&format!("Cut {k}"), c, mm(10.0), kw_feature::BodyOp::Cut)?;
            }
            let c = d.add_circle_sketch(
                "Blind",
                kw_feature::PlaneSpec::Xy(mm(6.0)),
                mm(20.0),
                mm(19.0),
                mm(8.0),
            )?;
            d.add_extrude("Blind cut", c, mm(4.0), kw_feature::BodyOp::Cut)?;
            Ok(())
        })
        .unwrap();
    p
}

fn wait_sim(ws: &mut CamWorkspace, p: &mut Project) {
    ws.sim = None;
    ws.start_sim(p);
    let t = std::time::Instant::now();
    while ws.sim.is_none() {
        ws.poll_jobs(p);
        assert!(t.elapsed().as_secs() < 300, "{}", ws.message);
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[test]
fn round_holes_are_found_drilled_and_bored_end_to_end() {
    let mut p = block_with_holes();
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    let groups = ws.hole_groups(&mut p);
    assert_eq!(groups.len(), 2, "{groups:?}");
    assert!((groups[0].0 - 5.0).abs() < 0.05 && groups[0].1.len() == 2);
    assert!((groups[1].0 - 8.0).abs() < 0.05 && groups[1].1.len() == 1);
    assert!(groups[0].1.iter().all(|h| h.through) && !groups[1].1[0].through);
    let blind = &groups[1].1[0];
    assert!((blind.bottom - 6.0).abs() < 1e-6 && (blind.top - 10.0).abs() < 1e-6);
    // the stock is the solid block; drilling and boring leave the part
    p.cam[0].stock = StockSpec::Explicit {
        min: [0.0, 0.0, 0.0],
        max: [40.0, 25.0, 10.0],
    };
    // drill the 5 mm holes: the operation picks a drill by itself
    ws.settings.op = OpKind::Drill;
    ws.settings.use_quick_tool();
    ws.use_holes(&mut p, 0);
    // (the panel adapts the tool when it is drawn; here the library drill is chosen by hand)
    let drill = ws
        .library
        .tools
        .iter()
        .find(|d| d.name.starts_with("5 mm drill"))
        .expect("a 5 mm drill")
        .tool
        .clone();
    ws.settings.preset = Some(drill);
    ws.settings.peck = 3.0;
    assert_eq!(ws.settings.points.len(), 2);
    assert!(
        ws.settings.bottom < 0.0,
        "the tip goes through: {}",
        ws.settings.bottom
    );
    ws.run_now(&mut p).unwrap();
    // bore the blind hole with a 6 mm end mill
    ws.settings.op = OpKind::Bore;
    ws.settings.preset = None;
    ws.settings.tool = ToolChoice::Flat;
    ws.settings.diameter = 6.0;
    ws.use_holes(&mut p, 1);
    ws.settings.step_down = 1.5;
    ws.run_now(&mut p).unwrap();
    assert_eq!(p.cam[0].operations.len(), 2);
    wait_sim(&mut ws, &mut p);
    let (_, _, r) = ws.sim.clone().unwrap();
    // what the part has and the stock still holds is the little cone at the tip of the through holes' ends
    // and the floor of the bored hole
    let holes_volume = 2.0 * std::f64::consts::PI * 6.25 * 10.0 + std::f64::consts::PI * 16.0 * 4.0;
    assert!(
        r.remaining < 0.04 * holes_volume,
        "left over {} of {holes_volume}",
        r.remaining
    );
    assert!(
        r.overcut < 0.04 * holes_volume,
        "cut too much {}",
        r.overcut
    );
    assert!(r.ops.iter().all(|o| !o.collides()), "{:?}", r.ops);
    // the programs post in all five dialects and the simulators replay the holes
    for id in kw_app::cam::BUILTIN_POSTS {
        ws.set_post(&mut p, id).unwrap();
        let text = ws
            .setup_post_text(&p, 0)
            .unwrap()
            .unwrap_or_else(|e| panic!("{id}: {e}"));
        let d = match id {
            "Fanuc" => kw_gsim::Dialect::Fanuc,
            "Haas" => kw_gsim::Dialect::Haas,
            "Siemens840D" => kw_gsim::Dialect::Siemens840D,
            "Heidenhain" => kw_gsim::Dialect::Heidenhain,
            _ => kw_gsim::Dialect::LinuxCnc,
        };
        let run = kw_gsim::run(d, &text).unwrap_or_else(|e| panic!("{id}: {e}\n{text}"));
        assert_eq!(run.drills.len(), 2, "{id}");
    }
}

#[test]
fn rest_finishing_cuts_only_where_the_earlier_operations_left_stock() {
    // a block with a 20 x 10 x 4 pocket; the stock is the plain block
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
            d.add_extrude("Block", s, mm(10.0), kw_feature::BodyOp::New)?;
            let c = d.add_rect_sketch(
                "Pocket",
                kw_feature::PlaneSpec::Xy(mm(6.0)),
                mm(10.0),
                mm(7.0),
                mm(20.0),
                mm(10.0),
            )?;
            d.add_extrude("Cut", c, mm(4.0), kw_feature::BodyOp::Cut)?;
            Ok(())
        })
        .unwrap();
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    let rest = |ws: &mut CamWorkspace| {
        let s = &mut ws.settings;
        s.op = OpKind::RestFinish;
        s.preset = None;
        s.tool = ToolChoice::Ball;
        s.diameter = 3.0;
        (s.x0, s.y0, s.x1, s.y1) = (0.0, 0.0, 40.0, 25.0);
        s.region = None;
        s.top = 10.0;
        s.bottom = 5.0;
        s.stepover = 1.0;
        s.threshold = 0.2;
    };
    // nothing has been cut yet: all the pocket's volume is left, and only there
    rest(&mut ws);
    ws.run_now(&mut p).unwrap();
    let first = p.cam[0].operations[0].path.clone();
    for (a, b) in kw_cam::finish3d::moves(&first) {
        for q in [a, b] {
            if q[2] < 9.9 {
                assert!(q[0] > 10.0 - 4.5 && q[0] < 30.0 + 4.5, "{q:?}");
                assert!(q[1] > 7.0 - 4.5 && q[1] < 17.0 + 4.5, "{q:?}");
            }
        }
    }
    assert!(first.cutting_length() > 50.0);
    // pocket it with a flat tool: only the corners the round tool cannot reach are left
    ws.settings.op = OpKind::Pocket;
    ws.settings.use_quick_tool();
    ws.settings.tool = ToolChoice::Flat;
    ws.settings.diameter = 6.0;
    (
        ws.settings.x0,
        ws.settings.y0,
        ws.settings.x1,
        ws.settings.y1,
    ) = (10.0, 7.0, 30.0, 17.0);
    ws.settings.top = 10.0;
    ws.settings.bottom = 6.0;
    ws.settings.step_down = 2.0;
    ws.settings.stepover = 4.0;
    ws.run_now(&mut p).unwrap();
    // the rest pass made before the pocket is now out of date: its key includes the operations before it,
    // and the pocket was added after it, so it is not. Move it behind: remake the rest pass as a new one
    rest(&mut ws);
    ws.run_now(&mut p).unwrap();
    let second = p.cam[0].operations[2].path.clone();
    // (the length that cuts sideways: the plunges from the safe height are most of what is left in the second)
    let sideways = |tp: &kw_cam::Toolpath| -> f64 {
        kw_cam::finish3d::moves(tp)
            .iter()
            .filter(|(a, b)| a[2] < 9.9 && b[2] < 9.9)
            .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]))
            .sum()
    };
    assert!(
        sideways(&second) < 0.4 * sideways(&first),
        "{} vs {}",
        sideways(&second),
        sideways(&first)
    );
    assert!(ws.stale_ops(&mut p).is_empty());
    // taking the pocket away changes what the rest pass was made from
    p.cam[0].operations.remove(1);
    assert_eq!(ws.stale_ops(&mut p), vec![1]);
    ws.start_regenerate(&mut p);
    ws.wait_generation(&mut p);
    assert!(ws.stale_ops(&mut p).is_empty());
    assert!(
        (p.cam[0].operations[1].path.cutting_length() - first.cutting_length()).abs()
            < 0.05 * first.cutting_length()
    );
}

#[test]
fn a_tilted_setup_is_posted_with_the_rotary_axes_of_the_machine() {
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
            d.add_extrude("Block", s, mm(10.0), kw_feature::BodyOp::New)?;
            Ok(())
        })
        .unwrap();
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    // the front face (normal −y), machined on a five-axis machine with a trunnion table
    ws.set_machine(&mut p, "AC trunnion table").unwrap();
    ws.set_orientation(&mut p, 2);
    ws.fit_to_workpiece(&mut p);
    let s = &mut ws.settings;
    s.op = OpKind::Pocket;
    s.preset = None;
    s.tool = ToolChoice::Flat;
    s.diameter = 6.0;
    (s.x0, s.y0, s.x1, s.y1) = (5.0, 2.0, 35.0, 10.0);
    s.region = None;
    s.top = 0.0;
    s.bottom = -2.0;
    s.step_down = 1.0;
    s.stepover = 3.0;
    ws.run_now(&mut p).unwrap();
    let setup = p.cam[0].clone();
    assert!(fiveaxis::needs_rotary(&setup));
    let machine = ws.machine_of(&setup).unwrap();
    let (tp, rep) = fiveaxis::program(&setup, &machine).unwrap();
    assert!(rep.converted && rep.blocks >= rep.moves, "{rep:?}");
    assert!(rep.ok(), "{}", rep.summary());
    assert!(
        rep.max_orientation_error < 1e-3 && rep.max_tip_error < 1e-3,
        "{rep:?}"
    );
    assert!(rep.interpolation_error < 1e-3, "{rep:?}");
    // the table is tilted 90 degrees about x for a face looking along −y
    let a = rep
        .rotary_range
        .iter()
        .find(|r| r.0 == "A")
        .expect("an A axis");
    assert!(
        (a.1.abs() - 90.0).abs() < 1.0 && (a.2.abs() - 90.0).abs() < 1.0,
        "{a:?}"
    );
    assert!(tp
        .events
        .iter()
        .any(|e| matches!(e, Event::Tcp { on: true })));
    assert!(tp.events.iter().any(|e| matches!(e, Event::Feed5 { .. })));
    // every dialect writes it and the simulators run it
    for id in kw_app::cam::BUILTIN_POSTS {
        ws.set_post(&mut p, id).unwrap();
        let text = ws
            .setup_post_text(&p, 0)
            .unwrap()
            .unwrap_or_else(|e| panic!("{id}: {e}"));
        let d = match id {
            "Fanuc" => kw_gsim::Dialect::Fanuc,
            "Haas" => kw_gsim::Dialect::Haas,
            "Siemens840D" => kw_gsim::Dialect::Siemens840D,
            "Heidenhain" => kw_gsim::Dialect::Heidenhain,
            _ => kw_gsim::Dialect::LinuxCnc,
        };
        let run = kw_gsim::run(d, &text).unwrap_or_else(|e| panic!("{id}: {e}\n{text}"));
        assert!(run.program_end_seen, "{id}");
    }
    // a three-axis machine takes the same setup as it is (the part is clamped tilted)
    ws.set_machine(&mut p, "3-axis mill").unwrap();
    let setup = p.cam[0].clone();
    let machine = ws.machine_of(&setup).unwrap();
    let (_, rep) = fiveaxis::program(&setup, &machine).unwrap();
    assert!(!rep.converted);
}

#[test]
fn swarf_and_tilted_contours_need_rotary_axes_and_run_on_them() {
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
            d.add_extrude("Block", s, mm(10.0), kw_feature::BodyOp::New)?;
            Ok(())
        })
        .unwrap();
    let mut ws = CamWorkspace::default();
    ws.fit_to_workpiece(&mut p);
    let s = &mut ws.settings;
    s.op = OpKind::Swarf;
    s.preset = None;
    s.tool = ToolChoice::Flat;
    s.diameter = 8.0;
    (s.x0, s.y0, s.x1, s.y1) = (0.0, 0.0, 40.0, 25.0);
    s.region = None;
    s.top = 10.0;
    s.bottom = 0.0;
    s.taper = 8.0;
    // the three-axis mill is refused
    let e = ws.run_now(&mut p).unwrap_err();
    assert!(e.contains("rotary axes"), "{e}");
    assert!(p.cam[0].operations.is_empty());
    ws.set_machine(&mut p, "AC trunnion table").unwrap();
    // flutes long enough for a 10 mm wall at 8 degrees
    let mut long = Tool::flat(1, 8.0, 25.0);
    long.name = "8 mm long".into();
    ws.settings.preset = Some(long);
    ws.run_now(&mut p).unwrap();
    ws.settings.op = OpKind::TiltedContour;
    ws.settings.tilt = 12.0;
    ws.settings.bottom = 3.0;
    ws.run_now(&mut p).unwrap();
    assert_eq!(p.cam[0].operations.len(), 2);
    assert!(p.cam[0].operations[0]
        .path
        .events
        .iter()
        .any(|e| matches!(e, Event::Pose { axis, .. } if axis[0].hypot(axis[1]) > 0.05)));
    // the program for the machine: rotary axes within their travel, tool axis reproduced
    let setup = p.cam[0].clone();
    let machine = ws.machine_of(&setup).unwrap();
    let (_, rep) = fiveaxis::program(&setup, &machine).unwrap();
    assert!(rep.converted && rep.ok(), "{}", rep.summary());
    assert!(rep.blocks > 200, "{rep:?}");
    // at the sharp corners of the swarf wall the axes swing a few degrees off the great circle
    assert!(rep.interpolation_error < 5.0, "{rep:?}");
    // the stock simulation sweeps the leaning tool without rapids through the stock
    p.cam[0].stock = StockSpec::Box {
        side: 6.0,
        top: 0.0,
        bottom: 0.0,
    };
    wait_sim(&mut ws, &mut p);
    let (_, _, r) = ws.sim.clone().unwrap();
    assert!(
        r.stock_volume - r.final_volume > 100.0,
        "{}",
        r.stock_volume - r.final_volume
    );
    assert!(r.ops.iter().all(|o| o.rapid_cut < 1e-3), "{:?}", r.ops);
    // and the post cannot write the tilted poses for a post-processor of a three-axis machine
    ws.set_machine(&mut p, "3-axis mill").unwrap();
    assert!(ws.setup_post_text(&p, 0).unwrap().is_err());
}
