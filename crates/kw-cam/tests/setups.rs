//! The pieces of a multi-setup job: stock shapes, material sampled from a mesh, the stock's surface, the
//! comparison of stock and part, regions from face outlines, operations as data, and the work offset in
//! every post.

use kw_cam::bvh::TriMesh;
use kw_cam::dexel::TriDexel;
use kw_cam::ops::{OpKind, OpParams, RegionSpec};
use kw_cam::ops2d::{pocket, Cut};
use kw_cam::path::{Event, Toolpath};
use kw_cam::post::{render, PostConfig};
use kw_cam::regions::face_loops;
use kw_cam::stock::{cylinder_mesh, StockSpec};
use kw_cam::{Region, Tool};
use kw_gsim::{run, Dialect};

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

#[test]
fn material_sampled_from_a_box_and_a_cylinder_has_the_right_volume() {
    // aligned with the cells: exact sideways, and exact in height
    let b = TriMesh::from_box([1.5, 2.0, 0.0], [21.5, 14.0, 9.5]);
    let d = TriDexel::from_mesh(&b, [0.0, 0.0, -1.0], [24.0, 18.0, 11.0], 0.5);
    let want = 20.0 * 12.0 * 9.5;
    for v in d.volume() {
        assert!(rel(v, want) < 0.002, "box volume {v} vs {want}");
    }
    // off the cells: within the sideways quantisation (half a cell on the outline)
    let b = TriMesh::from_box([1.3, 2.1, 0.0], [21.7, 14.9, 9.2]);
    let d = TriDexel::from_mesh(&b, [0.0, 0.0, -1.0], [24.0, 18.0, 11.0], 0.5);
    let want = 20.4 * 12.8 * 9.2;
    for v in d.volume() {
        assert!(rel(v, want) < 0.06, "box volume {v} vs {want}");
    }
    let c = cylinder_mesh(2, [12.0, 9.0], 8.0, 0.0, 10.0);
    let d = TriDexel::from_mesh(&c, [0.0, 0.0, -1.0], [24.0, 18.0, 11.0], 0.5);
    let area = 0.5 * 96.0 * 64.0 * (std::f64::consts::TAU / 96.0).sin();
    for v in d.volume() {
        assert!(
            rel(v, area * 10.0) < 0.02,
            "cylinder volume {v} vs {}",
            area * 10.0
        );
    }
    // a cylinder along x: its round section is sampled by only 16 rays (the chords end in a square root,
    // which the midpoint rule handles worst), so the volume is looser
    let c = cylinder_mesh(0, [9.0, 5.0], 4.0, 2.0, 20.0);
    let d = TriDexel::from_mesh(&c, [0.0, 0.0, 0.0], [24.0, 18.0, 11.0], 0.5);
    let area = 0.5 * 96.0 * 16.0 * (std::f64::consts::TAU / 96.0).sin();
    for v in d.volume() {
        assert!(
            rel(v, area * 18.0) < 0.05,
            "x cylinder {v} vs {}",
            area * 18.0
        );
    }
}

#[test]
fn the_surface_of_cut_stock_samples_back_to_the_same_material() {
    let stock = TriMesh::from_box([0.0, 0.0, 0.0], [30.0, 24.0, 10.0]);
    let mut d = TriDexel::from_mesh(&stock, [0.0, 0.0, 0.0], [30.0, 24.0, 10.0], 0.5);
    let tool = Tool::flat(1, 6.0, 20.0);
    let cut = Cut::new(10.0, 6.0);
    let tp = pocket(&Region::rect(6.0, 6.0, 24.0, 18.0), &tool, &cut);
    let removed = d.run(&tool, &tp);
    assert!(removed > 700.0, "{removed}");
    let v0 = d.volume()[0];
    let back = TriDexel::from_mesh(&d.to_mesh(), [0.0, 0.0, 0.0], [30.0, 24.0, 10.0], 0.5);
    for v in back.volume() {
        assert!(rel(v, v0) < 0.005, "re-sampled {v} vs {v0}");
    }
    // a different cell size sees the same stock
    let coarse = TriDexel::from_mesh(&d.to_mesh(), [0.0, 0.0, 0.0], [30.0, 24.0, 10.0], 1.0);
    assert!(
        rel(coarse.volume()[0], v0) < 0.03,
        "{} vs {v0}",
        coarse.volume()[0]
    );
}

#[test]
fn stock_against_part_counts_what_is_left_and_what_is_cut_too_deep() {
    let g = ([0.0, 0.0, 0.0], [20.0, 20.0, 12.0]);
    let mk = |zmax: f64| {
        TriDexel::from_mesh(
            &TriMesh::from_box([0.0, 0.0, 0.0], [20.0, 20.0, zmax]),
            g.0,
            g.1,
            0.5,
        )
    };
    let (stock, part) = (mk(8.0), mk(6.0));
    let c = stock.compare(&part);
    assert!(rel(c.remaining, 20.0 * 20.0 * 2.0) < 0.01, "{c:?}");
    assert!(c.overcut < 1e-6);
    let c = part.compare(&stock);
    assert!(rel(c.overcut, 800.0) < 0.01 && c.remaining < 1e-6, "{c:?}");
    // the top facets of the stock are 2 mm above the part
    let top = stock
        .facets()
        .into_iter()
        .find(|f| f.normal == [0.0, 0.0, 1.0])
        .unwrap();
    let dev = stock.facet_deviation(&part, &top);
    assert!((dev - 2.0).abs() < 1e-6, "{dev}");
    // and the part's are 2 mm below where the stock reaches: material missing
    let low = part
        .facets()
        .into_iter()
        .find(|f| f.normal == [0.0, 0.0, 1.0])
        .unwrap();
    let dev = part.facet_deviation(&stock, &low);
    assert!((dev + 2.0).abs() < 1e-6, "{dev}");
}

#[test]
fn stock_shapes_surround_the_workpiece() {
    let part = TriMesh::from_box([2.0, 3.0, 1.0], [12.0, 9.0, 7.0]);
    let s = StockSpec::Box {
        side: 2.0,
        top: 1.0,
        bottom: 0.5,
    }
    .shape(&part)
    .unwrap();
    assert_eq!(s.min, [0.0, 1.0, 0.5]);
    assert_eq!(s.max, [14.0, 11.0, 8.0]);
    let s = StockSpec::Cylinder {
        axis: 2,
        margin: 1.0,
        ends: 2.0,
    }
    .shape(&part)
    .unwrap();
    // the bar's radius reaches the far corner of the footprint
    let r = ((5.0f64).powi(2) + (3.0f64).powi(2)).sqrt() + 1.0;
    assert!((s.max[0] - s.min[0] - 2.0 * r).abs() < 1e-9);
    assert_eq!((s.min[2], s.max[2]), (-1.0, 9.0));
    let d = TriDexel::from_mesh(&s.mesh, s.min, s.max, 0.5);
    let part_d = TriDexel::from_mesh(&part, s.min, s.max, 0.5);
    let c = d.compare(&part_d);
    assert!(c.overcut < 1.0, "the bar must contain the part: {c:?}");
    assert!(StockSpec::Previous.shape(&part).is_none());
    let s = StockSpec::Explicit {
        min: [0.0; 3],
        max: [5.0; 3],
    }
    .shape(&part)
    .unwrap();
    assert_eq!(s.max, [5.0; 3]);
}

#[test]
fn the_outline_of_a_ring_face_is_an_outer_loop_and_an_island() {
    let o = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
    let i = [[3.0, 3.0], [7.0, 3.0], [7.0, 7.0], [3.0, 7.0]];
    let v = |p: [f64; 2]| [p[0], p[1], 4.0];
    let mut tris = Vec::new();
    for k in 0..4 {
        let n = (k + 1) % 4;
        tris.push([v(o[k]), v(o[n]), v(i[n])]);
        tris.push([v(o[k]), v(i[n]), v(i[k])]);
    }
    let (outer, islands) = face_loops(&tris).unwrap();
    assert_eq!(outer.len(), 4);
    assert_eq!(islands.len(), 1);
    assert_eq!(islands[0].len(), 4);
    let spec = RegionSpec::Polygon { outer, islands };
    let region = spec.region();
    assert!(
        (region.area() - (100.0 - 16.0)).abs() < 1e-9,
        "{}",
        region.area()
    );
    assert_eq!(spec.bounds(), [0.0, 0.0, 10.0, 10.0]);
}

fn params(kind: OpKind, region: RegionSpec) -> OpParams {
    OpParams {
        kind,
        region,
        top: 10.0,
        bottom: 7.0,
        step_down: 3.0,
        stepover: 2.0,
        feed: 800.0,
        rpm: 8000.0,
        tool: 1,
        ..OpParams::default()
    }
}

#[test]
fn an_operation_is_regenerated_from_its_parameters() {
    let tool = Tool::flat(1, 6.0, 25.0);
    let p = params(OpKind::Pocket, RegionSpec::Rect([0.0, 0.0, 40.0, 25.0]));
    let a = p.generate(&tool, None).unwrap();
    let b = p.generate(&tool, None).unwrap();
    assert_eq!(a, b);
    assert!(a.cutting_length() > 100.0);
    // a polygon region with the same outline gives a pocket that cuts the same area
    let poly = params(
        OpKind::Pocket,
        RegionSpec::Polygon {
            outer: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 25.0], [0.0, 25.0]],
            islands: vec![],
        },
    );
    let c = poly.generate(&tool, None).unwrap();
    assert!((c.cutting_length() - a.cutting_length()).abs() < 0.02 * a.cutting_length());
    // errors are reported, not panicked
    let small = params(OpKind::Pocket, RegionSpec::Rect([0.0, 0.0, 4.0, 4.0]));
    assert!(small.generate(&tool, None).is_err());
    let mut upside = p.clone();
    upside.top = 5.0;
    assert!(upside.generate(&tool, None).is_err());
    let fin = params(OpKind::Finish3d, RegionSpec::Rect([0.0, 0.0, 40.0, 25.0]));
    assert!(fin.generate(&Tool::ball(1, 6.0, 25.0), None).is_err());
    assert!(fin
        .generate(&tool, Some(&TriMesh::from_box([0.0; 3], [1.0; 3])))
        .is_err());
}

#[test]
fn every_post_writes_the_work_offset_and_the_simulator_accepts_it() {
    let tool = Tool::flat(1, 6.0, 25.0);
    let mut tp = Toolpath::new("two setups");
    tp.push(Event::Comment("setup 1".into()));
    tp.push(Event::WorkOffset { number: 1 });
    let a = pocket(
        &Region::rect(0.0, 0.0, 30.0, 20.0),
        &tool,
        &Cut::new(5.0, 2.0),
    );
    tp.events.extend(
        a.events
            .iter()
            .filter(|e| !matches!(e, Event::ProgramEnd))
            .cloned(),
    );
    tp.push(Event::Comment("setup 2".into()));
    tp.push(Event::WorkOffset { number: 2 });
    tp.events.extend(a.events.iter().cloned());
    for (d, cfg, marker) in [
        (Dialect::Fanuc, PostConfig::fanuc(), ["G54", "G55"]),
        (Dialect::Haas, PostConfig::haas(), ["G54", "G55"]),
        (Dialect::Siemens840D, PostConfig::siemens(), ["G54", "G55"]),
        (
            Dialect::Heidenhain,
            PostConfig::heidenhain(),
            ["Q339=1", "Q339=2"],
        ),
        (Dialect::LinuxCnc, PostConfig::linuxcnc(), ["G54", "G55"]),
    ] {
        let text = render(&cfg, &tp).unwrap();
        for m in marker {
            assert!(text.contains(m), "{d:?} lacks {m}:\n{text}");
        }
        let r = run(d, &text).unwrap_or_else(|e| panic!("{d:?}: {e}\n{text}"));
        assert!(r.program_end_seen, "{d:?}");
        // both pockets ran
        let n1 = run(d, &render(&cfg, &a).unwrap()).unwrap().motions.len();
        assert_eq!(r.motions.len(), 2 * n1, "{d:?}");
    }
    // a post without the template writes a comment instead of nothing
    let bare = PostConfig::parse(
        &kw_cam::post::FANUC
            .lines()
            .filter(|l| !l.starts_with("work_offset"))
            .collect::<Vec<_>>()
            .join("\n"),
    )
    .unwrap();
    let text = render(&bare, &tp).unwrap();
    assert!(text.contains("WORK OFFSET G54"), "{text}");
}
