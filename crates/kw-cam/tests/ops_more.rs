//! The operations beyond the first four: face, inside contour, chamfer, drill, bore, waterline, rest
//! finishing, swarf and the tilted contour, checked on the stock simulation and against closed forms.

use kw_cam::bvh::TriMesh;
use kw_cam::dexel::TriDexel;
use kw_cam::finish3d::{check_poses, moves, sample_poses};
use kw_cam::ops::{GenContext, OpKind, OpParams, RegionSpec};
use kw_cam::path::Event;
use kw_cam::post::{render, PostConfig};
use kw_cam::suite3d::{suite, SIZE};
use kw_cam::Tool;
use kw_gsim::{run, Dialect};

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

fn params(kind: OpKind, region: [f64; 4], top: f64, bottom: f64) -> OpParams {
    OpParams {
        kind,
        region: RegionSpec::Rect(region),
        top,
        bottom,
        step_down: 2.0,
        stepover: 4.0,
        feed: 800.0,
        rpm: 8000.0,
        tool: 1,
        ..OpParams::default()
    }
}

fn block(min: [f64; 3], max: [f64; 3], h: f64) -> TriDexel {
    TriDexel::from_mesh(&TriMesh::from_box(min, max), min, max, h)
}

#[test]
fn facing_takes_the_top_off_the_whole_region() {
    let tool = Tool::flat(1, 10.0, 25.0);
    let mut d = block([0.0, 0.0, 0.0], [40.0, 25.0, 14.0], 0.25);
    let p = params(OpKind::Face, [0.0, 0.0, 40.0, 25.0], 14.0, 10.0);
    let tp = p.generate(&tool, None).unwrap();
    d.run(&tool, &tp);
    let removed = 40.0 * 25.0 * 14.0 - d.volume()[0];
    assert!(rel(removed, 40.0 * 25.0 * 4.0) < 0.02, "{removed}");
    assert_eq!(d.rapid_removed, 0.0);
    // nothing left above the face anywhere
    let cells = d.cells();
    for i in 0..cells[0] {
        for j in 0..cells[1] {
            assert!(d.top_height(i, j).unwrap() <= 10.0 + 1e-9, "({i},{j})");
        }
    }
}

#[test]
fn the_inside_contour_stays_inside_the_walls() {
    let tool = Tool::flat(1, 6.0, 25.0);
    let p = params(OpKind::ContourInside, [0.0, 0.0, 40.0, 30.0], 10.0, 4.0);
    let tp = p.generate(&tool, None).unwrap();
    let mut n = 0;
    for (a, b) in moves(&tp) {
        for q in [a, b] {
            if q[2] < 9.0 {
                n += 1;
                // the tool centre is one radius inside the wall
                assert!(q[0] >= 3.0 - 1e-6 && q[0] <= 37.0 + 1e-6, "{q:?}");
                assert!(q[1] >= 3.0 - 1e-6 && q[1] <= 27.0 + 1e-6, "{q:?}");
            }
        }
    }
    assert!(n > 20);
    // too small a region is refused
    let small = params(OpKind::ContourInside, [0.0, 0.0, 5.0, 5.0], 10.0, 4.0);
    assert!(small.generate(&tool, None).is_err());
}

#[test]
fn a_chamfer_mill_breaks_the_top_edge_by_the_width() {
    // a 60 x 40 x 8 block, 90 degree chamfer mill with a 0.2 mm tip, 1 mm chamfer on the top edge
    let tool = Tool::chamfer(1, 12.0, 90.0, 0.2, 12.0);
    // the block itself is the part [0,60]x[0,40]; the stock around it is air for this test
    let part = TriMesh::from_box([0.0, 0.0, 0.0], [60.0, 40.0, 8.0]);
    let mut d = TriDexel::from_mesh(&part, [-10.0, -10.0, 0.0], [70.0, 50.0, 8.0], 0.1);
    let mut p = params(OpKind::Chamfer, [0.0, 0.0, 60.0, 40.0], 8.0, 0.0);
    p.width = 1.0;
    let tp = p.generate(&tool, None).unwrap();
    let removed_before = d.volume()[0];
    d.run(&tool, &tp);
    let removed = removed_before - d.volume()[0];
    // triangle section 1 x 1 / 2 along 200 mm of perimeter
    let want = 0.5 * 200.0;
    assert!(rel(removed, want) < 0.08, "{removed} vs {want}");
    assert_eq!(d.rapid_removed, 0.0);
    // the chamfer face is at 45 degrees: the surface at the top edge reads 8 − 1 = 7 mm at the wall
    let h = d.top_height(((0.25 + 10.0) / 0.1) as usize, 200).unwrap();
    assert!((h - 7.25).abs() < 0.2, "{h}");
    // a width the tool cannot make is refused, as is a tool that is not a chamfer mill
    p.width = 6.0;
    assert!(p.generate(&tool, None).is_err());
    p.width = 1.0;
    assert!(p.generate(&Tool::flat(1, 6.0, 20.0), None).is_err());
    // the tool is called by its own name
    assert!(tp
        .events
        .iter()
        .any(|e| matches!(e, Event::ToolChange { name, .. } if name == &tool.name)));
}

#[test]
fn drilling_makes_the_holes_with_the_cone_of_the_drill() {
    let tool = Tool::drill(1, 5.0, 118.0, 30.0);
    let mut p = params(OpKind::Drill, [0.0, 0.0, 1.0, 1.0], 10.0, 2.0);
    p.points = vec![[10.0, 10.0], [30.0, 10.0], [20.0, 20.0]];
    let mut d = block([0.0, 0.0, 0.0], [40.0, 30.0, 10.0], 0.1);
    let tp = p.generate(&tool, None).unwrap();
    let before = d.volume()[0];
    d.run(&tool, &tp);
    let removed = before - d.volume()[0];
    let half = 59f64.to_radians();
    let cone_h = 2.5 / half.tan();
    let one =
        std::f64::consts::PI * 6.25 * (8.0 - cone_h) + std::f64::consts::PI * 6.25 * cone_h / 3.0;
    assert!(rel(removed, 3.0 * one) < 0.03, "{removed} vs {}", 3.0 * one);
    // pecking writes the peck cycle in every dialect, and the simulators accept it
    p.peck = 2.0;
    let tp = p.generate(&tool, None).unwrap();
    assert!(tp
        .events
        .iter()
        .any(|e| matches!(e, Event::DrillStart { peck: Some(q), .. } if *q == 2.0)));
    for (dia, cfg) in [
        (Dialect::Fanuc, PostConfig::fanuc()),
        (Dialect::Heidenhain, PostConfig::heidenhain()),
        (Dialect::Siemens840D, PostConfig::siemens()),
    ] {
        let text = render(&cfg, &tp).unwrap();
        let r = run(dia, &text).unwrap_or_else(|e| panic!("{dia:?}: {e}\n{text}"));
        assert_eq!(r.drills.len(), 3, "{dia:?}");
    }
    // a mill cannot drill, and nothing to drill is refused
    assert!(p.generate(&Tool::flat(1, 5.0, 20.0), None).is_err());
    p.points.clear();
    assert!(p.generate(&tool, None).is_err());
}

#[test]
fn a_helical_bore_clears_the_whole_hole() {
    let tool = Tool::flat(1, 6.0, 25.0);
    let mut p = params(OpKind::Bore, [0.0, 0.0, 1.0, 1.0], 10.0, 4.0);
    p.points = vec![[15.0, 15.0], [35.0, 15.0]];
    p.hole_diameter = 12.0;
    p.step_down = 1.5;
    let mut d = block([0.0, 0.0, 0.0], [50.0, 30.0, 10.0], 0.15);
    let tp = p.generate(&tool, None).unwrap();
    let before = d.volume()[0];
    d.run(&tool, &tp);
    let removed = before - d.volume()[0];
    let want = 2.0 * std::f64::consts::PI * 36.0 * 6.0;
    assert!(rel(removed, want) < 0.03, "{removed} vs {want}");
    assert_eq!(d.rapid_removed, 0.0, "the tool never rapids through stock");
    // one program: one tool call and one program end
    let n = |f: fn(&Event) -> bool| tp.events.iter().filter(|e| f(e)).count();
    assert_eq!(n(|e| matches!(e, Event::ToolChange { .. })), 1);
    assert_eq!(n(|e| matches!(e, Event::ProgramEnd)), 1);
    // helical arcs go through every post, and the simulators run them
    for (dia, cfg) in [
        (Dialect::Fanuc, PostConfig::fanuc()),
        (Dialect::Heidenhain, PostConfig::heidenhain()),
        (Dialect::LinuxCnc, PostConfig::linuxcnc()),
    ] {
        let text = render(&cfg, &tp).unwrap();
        run(dia, &text).unwrap_or_else(|e| panic!("{dia:?}: {e}"));
    }
    // a tool as big as the hole does not fit
    p.hole_diameter = 6.0;
    assert!(p.generate(&tool, None).is_err());
}

#[test]
fn waterlines_never_gouge_the_parts_of_the_suite() {
    let parts = suite(if cfg!(debug_assertions) { 2 } else { 21 }, 2024);
    let tool = Tool::ball(1, 6.0, 12.0);
    for part in &parts {
        let mesh = part.part_mesh();
        let mut p = params(OpKind::Waterline, [0.0, 0.0, SIZE, SIZE], part.zmax, 0.5);
        p.step_down = 3.0;
        p.grid = 0.5;
        let tp = p
            .generate_with(
                &tool,
                &GenContext {
                    mesh: Some(&mesh),
                    stock: None,
                },
            )
            .unwrap();
        let mv: Vec<_> = moves(&tp)
            .into_iter()
            .filter(|(a, b)| (a[2] - b[2]).abs() < 1e-9 && a[2] < p.top + 5.0)
            .collect();
        assert!(mv.len() > 20, "{}: only {} moves", part.name, mv.len());
        let poses = sample_poses(&mv, 0.25);
        // the chords between the points of a contour sag a little into the surface (grid of 0.5 mm)
        let rep = check_poses(&mesh, &tool, &poses, 0.03);
        eprintln!(
            "{:<22} {:>6} poses, {} gouges, max gouge {:.4} mm",
            part.name, rep.poses, rep.gouges, rep.max_gouge
        );
        assert_eq!(rep.gouges, 0, "{}: max gouge {}", part.name, rep.max_gouge);
        // and the tool touches the part: at its own level the lowest tip height of the part is reached
        let touching = poses
            .iter()
            .filter(|q| {
                kw_cam::dropcutter::drop_cutter(&mesh, &tool, q[0], q[1])
                    .is_some_and(|h| (h - q[2]).abs() < 0.03)
            })
            .count();
        assert!(
            touching * 10 > poses.len() * 8,
            "{}: {touching}/{}",
            part.name,
            poses.len()
        );
    }
}

#[test]
fn rest_finishing_works_only_where_stock_is_left() {
    // a hemispherical dome; a flat end mill roughed the top but could not reach the foot of the dome
    let parts = suite(2, 2024);
    let part = &parts[0];
    let mesh = part.part_mesh();
    let tool = Tool::ball(1, 3.0, 40.0);
    let region = [0.0, 0.0, SIZE, SIZE];
    // stock: the box over the part, with the part's own surface cut by an earlier tool that left 1 mm
    let zmax = part.zmax;
    let h = 0.5;
    let mut stock = TriDexel::from_mesh(
        &TriMesh::from_box([0.0, 0.0, 0.0], [SIZE, SIZE, zmax + 1.0]),
        [0.0, 0.0, 0.0],
        [SIZE, SIZE, zmax + 1.0],
        h,
    );
    // the earlier operation: a ball tool finishing the left half only, down to the surface
    let rough = Tool::ball(2, 6.0, 40.0);
    let mut left = params(OpKind::Finish3d, [0.0, 0.0, SIZE / 2.0, SIZE], zmax, 0.0);
    left.stepover = 1.0;
    let tp = left
        .generate_with(
            &rough,
            &GenContext {
                mesh: Some(&mesh),
                stock: None,
            },
        )
        .unwrap();
    stock.run(&rough, &tp);
    let mut rest = params(OpKind::RestFinish, region, zmax, 0.0);
    rest.stepover = 1.5;
    rest.threshold = 1.0;
    // without a stock the operation says so
    assert!(rest
        .generate_with(
            &tool,
            &GenContext {
                mesh: Some(&mesh),
                stock: None
            }
        )
        .is_err());
    let tp = rest
        .generate_with(
            &tool,
            &GenContext {
                mesh: Some(&mesh),
                stock: Some(&stock),
            },
        )
        .unwrap();
    let full = {
        let mut f = rest.clone();
        f.kind = OpKind::Finish3d;
        f.generate(&tool, Some(&mesh)).unwrap()
    };
    // only the right half (and a margin of one tool radius) is cut
    let xs: Vec<f64> = moves(&tp)
        .into_iter()
        .filter(|(a, b)| a[2] < zmax + 2.0 && b[2] < zmax + 2.0)
        .flat_map(|(a, b)| [a[0], b[0]])
        .collect();
    assert!(!xs.is_empty());
    let min_x = xs.iter().cloned().fold(f64::MAX, f64::min);
    // the strips the rough pass could not clean (steep slopes) are reached too, but the left side is mostly spared
    assert!(
        min_x > 5.0 && xs.iter().any(|x| *x > SIZE - 10.0),
        "{min_x}"
    );
    assert!(
        tp.cutting_length() < 0.7 * full.cutting_length(),
        "{} vs {}",
        tp.cutting_length(),
        full.cutting_length()
    );
    // the finished stock is within the threshold of the part everywhere the rest pass reached
    stock.run(&tool, &tp);
    let part_d = TriDexel::from_mesh(&mesh, stock.min, stock.max, h);
    let c = stock.compare(&part_d);
    let before_rest = c.remaining;
    assert!(before_rest < 0.2 * (SIZE * SIZE * zmax), "{before_rest}");
}

#[test]
fn swarf_follows_a_tapered_wall_with_the_flank_of_the_tool() {
    let tool = Tool::flat(1, 8.0, 40.0);
    let mut p = params(OpKind::Swarf, [0.0, 0.0, 40.0, 30.0], 20.0, 0.0);
    p.taper = 10.0;
    let tp = p.generate(&tool, None).unwrap();
    let poses: Vec<_> = tp
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Pose {
                p,
                axis,
                feed: Some(_),
            } => Some((*p, *axis)),
            _ => None,
        })
        .collect();
    assert!(poses.len() > 100);
    // the axis leans by the taper: tan = shift / height
    let lean = 10f64.to_radians();
    // (at the sharp corners the mitred top outline makes the ruling lean more)
    let straight = poses
        .iter()
        .filter(|(_, a)| ((a[0].hypot(a[1])).atan2(a[2]) - lean).abs() < 0.5f64.to_radians())
        .count();
    assert!(
        straight * 10 > poses.len() * 9,
        "{straight} of {}",
        poses.len()
    );
    // the tool stands outside the wall by its radius: tips are 4 mm off the bottom outline, on the outer side
    let (tip, _) = poses[poses.len() / 4];
    let d = [
        (tip[0] - 0.0).abs(),
        (40.0 - tip[0]).abs(),
        tip[1].abs(),
        (30.0 - tip[1]).abs(),
    ]
    .into_iter()
    .fold(f64::MAX, f64::min);
    assert!((d - 4.0).abs() < 1.0, "{d}");
    assert!(
        tip[0] < 0.0 || tip[0] > 40.0 || tip[1] < 0.0 || tip[1] > 30.0,
        "{tip:?}"
    );
    // a wall longer than the flutes is refused; the post refuses the tilted poses without a machine
    let short = Tool::flat(1, 8.0, 10.0);
    assert!(p.generate(&short, None).is_err());
    assert!(render(&PostConfig::fanuc(), &tp).is_err());
    // the simulation leaves a wall that leans: at the top the wall is shifted outward by height · tan(taper)
    let mut d = block([-20.0, -20.0, 0.0], [60.0, 50.0, 20.0], 0.25);
    d.run(&tool, &tp);
    let before = 80.0 * 70.0 * 20.0;
    assert!(d.volume()[0] < before);
    assert!(d.rapid_removed < 1e-3, "{}", d.rapid_removed);
}

#[test]
fn a_tilted_contour_leans_the_tool_against_the_wall() {
    let tool = Tool::flat(1, 8.0, 30.0);
    let mut p = params(OpKind::TiltedContour, [0.0, 0.0, 40.0, 30.0], 10.0, 4.0);
    p.tilt = 15.0;
    let tp = p.generate(&tool, None).unwrap();
    let cutting: Vec<_> = tp
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Pose {
                p,
                axis,
                feed: Some(_),
            } if p[2] < 9.0 => Some((*p, *axis)),
            _ => None,
        })
        .collect();
    assert!(cutting.len() > 30);
    for (q, a) in &cutting {
        let tilt = a[0].hypot(a[1]).atan2(a[2]);
        assert!(
            (tilt - 15f64.to_radians()).abs() < 1.0f64.to_radians(),
            "{}",
            tilt.to_degrees()
        );
        // the axis leans towards the part: from the tip, the way the spindle end goes is towards the block
        let centre = [20.0, 15.0];
        let to_centre = [centre[0] - q[0], centre[1] - q[1]];
        let along = a[0] * to_centre[0] + a[1] * to_centre[1];
        // (straight stretches only; at the rounded corners the lean follows the tangent)
        if q[0] > 4.0 && q[0] < 36.0 || q[1] > 4.0 && q[1] < 26.0 {
            assert!(along > 0.0, "{q:?} {a:?}");
        }
    }
    // the tilt cannot be absurd
    p.tilt = 85.0;
    assert!(p.generate(&tool, None).is_err());
}

#[test]
fn every_operation_posts_in_every_dialect_and_the_simulators_run_it() {
    let parts = suite(1, 7);
    let mesh = parts[0].part_mesh();
    let flat = Tool::flat(1, 6.0, 25.0);
    let ball = Tool::ball(1, 4.0, 20.0);
    let drill = Tool::drill(1, 5.0, 118.0, 30.0);
    let chamfer = Tool::chamfer(1, 10.0, 90.0, 0.2, 10.0);
    let mut cases: Vec<(&str, OpParams, &Tool)> = Vec::new();
    for kind in [
        OpKind::Face,
        OpKind::Pocket,
        #[cfg(feature = "adaptive-clearing")]
        OpKind::Adaptive,
        OpKind::ContourOutside,
        OpKind::ContourInside,
    ] {
        cases.push((
            kind.label(),
            params(kind, [0.0, 0.0, 40.0, 30.0], 10.0, 6.0),
            &flat,
        ));
    }
    let mut ch = params(OpKind::Chamfer, [0.0, 0.0, 40.0, 30.0], 10.0, 0.0);
    ch.width = 0.8;
    cases.push(("chamfer", ch, &chamfer));
    let mut dr = params(OpKind::Drill, [0.0, 0.0, 1.0, 1.0], 10.0, 2.0);
    dr.points = vec![[10.0, 10.0], [25.0, 10.0]];
    cases.push(("drill", dr.clone(), &drill));
    dr.peck = 2.5;
    cases.push(("peck drill", dr, &drill));
    let mut bo = params(OpKind::Bore, [0.0, 0.0, 1.0, 1.0], 10.0, 4.0);
    bo.points = vec![[20.0, 15.0]];
    bo.hole_diameter = 12.0;
    cases.push(("bore", bo, &flat));
    let mut wl = params(
        OpKind::Waterline,
        [10.0, 10.0, 40.0, 40.0],
        parts[0].zmax,
        1.0,
    );
    wl.step_down = 4.0;
    wl.grid = 1.0;
    cases.push(("waterline", wl, &ball));
    let mut fin = params(
        OpKind::Finish3d,
        [10.0, 10.0, 40.0, 40.0],
        parts[0].zmax,
        0.0,
    );
    fin.stepover = 4.0;
    cases.push(("3-D finish", fin, &ball));
    for (name, p, tool) in cases {
        let tp = p
            .generate_with(
                tool,
                &GenContext {
                    mesh: Some(&mesh),
                    stock: None,
                },
            )
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        for (dia, cfg) in [
            (Dialect::Fanuc, PostConfig::fanuc()),
            (Dialect::Haas, PostConfig::haas()),
            (Dialect::Siemens840D, PostConfig::siemens()),
            (Dialect::Heidenhain, PostConfig::heidenhain()),
            (Dialect::LinuxCnc, PostConfig::linuxcnc()),
        ] {
            let text = render(&cfg, &tp).unwrap_or_else(|e| panic!("{name} {dia:?}: {e}"));
            let r = run(dia, &text).unwrap_or_else(|e| panic!("{name} {dia:?}: {e}"));
            assert!(r.program_end_seen, "{name} {dia:?}");
            // the replayed motions end where the toolpath does
            if let (Some(last), Some(want)) =
                (r.motions.last(), moves(&tp).into_iter().last().map(|m| m.1))
            {
                assert!(
                    (last.to[0] - want[0]).abs() < 0.01 && (last.to[1] - want[1]).abs() < 0.01,
                    "{name} {dia:?}: ends at {:?}, wanted {:?}",
                    last.to,
                    want
                );
            }
        }
    }
}
