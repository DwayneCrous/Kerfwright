use kw_brep::{BRep, BodyId};
use kw_drawing::projection::ViewFrame;
use kw_drawing::section::{section_frame, slice_loops};
use kw_drawing::update::{generate, GenKind, GenSpec};
use kw_drawing::{Layer, Orientation, Shape, ViewStyle};
use kw_math::{Frame3, Vector3};
use kw_model::{extrude, ExtrudeOptions, Profile, ProfileEdge, ProfileLoop};

/// A 40 x 40 x 10 plate (z from 0 to 10) with a through hole of radius 5 at its centre.
fn plate() -> (BRep, BodyId) {
    let mut rep = BRep::new();
    let hole = ProfileLoop::new(vec![ProfileEdge::arc(
        &Frame3::world(),
        5.0,
        std::f64::consts::TAU,
        0.0,
    )]);
    let b = extrude(
        &mut rep,
        &Profile::rectangle(Frame3::world(), 40.0, 40.0).with_hole(hole),
        Vector3::new(0.0, 0.0, 10.0),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    (rep, b)
}

fn spec(kind: GenKind, frame: ViewFrame, scale: f64) -> GenSpec {
    GenSpec {
        id: 1,
        frame,
        scale,
        style: ViewStyle::VisibleHidden,
        kind,
    }
}

fn lines(g: &kw_drawing::hlr::ViewGeometry, layer: Layer) -> Vec<([f64; 2], [f64; 2])> {
    g.scene
        .prims
        .iter()
        .filter(|p| p.layer == layer)
        .filter_map(|p| match p.shape {
            Shape::Line { a, b } => Some((a, b)),
            _ => None,
        })
        .collect()
}

#[test]
fn section_frames_look_at_the_kept_side() {
    let front = ViewFrame::of(Orientation::Front);
    // a vertical cut line, material to its right (+x): we look along +x
    let (f, p0) = section_frame(&front, [0.0, -1.0], [0.0, 11.0], false).unwrap();
    assert!((f.dir[0] - 1.0).abs() < 1e-12 && f.up[2] > 0.999);
    assert!(p0[0].abs() < 1e-12);
    let (f, _) = section_frame(&front, [0.0, -1.0], [0.0, 11.0], true).unwrap();
    assert!((f.dir[0] + 1.0).abs() < 1e-12);
    // a horizontal cut line left to right keeps what is below it: a view from above
    let (f, _) = section_frame(&front, [-20.0, 5.0], [20.0, 5.0], false).unwrap();
    assert_eq!(f.orientation(), Orientation::Top);
    let (f, _) = section_frame(&front, [-20.0, 5.0], [20.0, 5.0], true).unwrap();
    assert_eq!(f.orientation(), Orientation::Bottom);
    assert!(section_frame(&front, [1.0, 1.0], [1.0, 1.0], false).is_err());
}

#[test]
fn the_hatch_covers_the_cut_face_and_leaves_the_hole_open() {
    let (rep, b) = plate();
    let front = ViewFrame::of(Orientation::Front);
    let kind = GenKind::Section {
        parent: front,
        a: [0.0, -2.0],
        b: [0.0, 12.0],
        flip: false,
    };
    let out = generate(&rep, &[b], &[spec(kind, front, 1.0)]).unwrap();
    let g = &out[0].1;
    // the cut face is 40 wide and 10 high, less the 10 wide hole
    assert!(
        (g.size[0] - 40.0).abs() < 1e-3 && (g.size[1] - 10.0).abs() < 1e-3,
        "{:?}",
        g.size
    );
    let hatch = lines(g, Layer::Hatch);
    let total: f64 = hatch
        .iter()
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]))
        .sum();
    let area = 40.0 * 10.0 - 10.0 * 10.0;
    assert!(
        (total * 3.0 - area).abs() < 0.04 * area,
        "hatch {total} x 3 vs area {area}"
    );
    // no hatch inside the hole (|x| < 5 about the centre of the view)
    for (a, b) in &hatch {
        for k in 0..=10 {
            let t = k as f64 / 10.0;
            let x = a[0] + (b[0] - a[0]) * t;
            assert!(x.abs() > 5.0 - 0.01, "hatch at x = {x}");
        }
    }
    // 45 degrees
    let (a, b) = hatch[0];
    assert!(((b[1] - a[1]) / (b[0] - a[0])).abs() - 1.0 < 1e-9);
    // the hole shows in the cut face as two vertical edges
    assert!(
        lines(g, Layer::Visible)
            .iter()
            .filter(|(a, b)| (a[0] - b[0]).abs() < 1e-9 && (a[0].abs() - 5.0).abs() < 1e-3)
            .count()
            >= 2
    );
}

fn shoelace(l: &[[f64; 3]], axes: (usize, usize)) -> f64 {
    let n = l.len();
    (0..n)
        .map(|i| {
            let (a, b) = (l[i], l[(i + 1) % n]);
            a[axes.0] * b[axes.1] - b[axes.0] * a[axes.1]
        })
        .sum::<f64>()
        / 2.0
}

#[test]
fn slicing_gives_the_outline_of_the_cut_and_misses_are_errors() {
    let (rep, b) = plate();
    // through the hole: two separate rectangles, 15 x 10 each
    let caps = slice_loops(&rep, &[b], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap();
    assert_eq!(caps.len(), 1);
    assert_eq!(caps[0].len(), 2, "one loop either side of the hole");
    for l in &caps[0] {
        assert!(l.iter().all(|p| p[0].abs() < 1e-9), "in the plane x = 0");
        // the hole's wall is a polygon inscribed in the circle, so a little more is cut
        assert!(
            (shoelace(l, (1, 2)).abs() - 150.0).abs() < 0.2,
            "{}",
            shoelace(l, (1, 2))
        );
    }
    // beside the hole: one 40 x 10 rectangle
    let caps = slice_loops(&rep, &[b], [12.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap();
    assert_eq!(caps[0].len(), 1);
    assert!((shoelace(&caps[0][0], (1, 2)).abs() - 400.0).abs() < 1e-6);
    // across the hole and the plate at an angle: still closed loops
    let caps = slice_loops(&rep, &[b], [0.3, 0.2, 0.0], [1.0, 1.0, 0.0]).unwrap();
    assert!(!caps[0].is_empty());
    // missing the model
    assert!(slice_loops(&rep, &[b], [100.0, 0.0, 0.0], [1.0, 0.0, 0.0]).is_err());
}

#[test]
fn the_half_that_is_cut_away_is_not_drawn() {
    let (rep, b) = plate();
    let front = ViewFrame::of(Orientation::Front);
    // cut at x = 12, keep x > 12: a 8 wide strip of plate remains
    let kind = GenKind::Section {
        parent: front,
        a: [12.0, -2.0],
        b: [12.0, 12.0],
        flip: false,
    };
    let g = &generate(&rep, &[b], &[spec(kind, front, 1.0)]).unwrap()[0].1;
    // the section looks along +x: its width is the plate's depth, 40 mm
    assert!(
        (g.size[0] - 40.0).abs() < 1e-3 && (g.size[1] - 10.0).abs() < 1e-3,
        "{:?}",
        g.size
    );
    // the hole is on the removed side: nothing of it is drawn (no hidden lines, no hole edges)
    assert_eq!(
        lines(g, Layer::Hidden).len(),
        0,
        "{:?}\n{:?}",
        lines(g, Layer::Hidden),
        lines(g, Layer::Visible)
    );
    let hatch = lines(g, Layer::Hatch);
    let total: f64 = hatch
        .iter()
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]))
        .sum();
    assert!((total * 3.0 - 400.0).abs() < 0.03 * 400.0, "{total}");
}

#[test]
fn a_detail_shows_only_the_circle_enlarged() {
    let (rep, b) = plate();
    let front = ViewFrame::of(Orientation::Front);
    // the front-left bottom corner of the plate: (x, z) = (-20, 0)
    let kind = GenKind::Detail {
        center: [-20.0, 0.0],
        radius: 6.0,
    };
    let out = generate(&rep, &[b], &[spec(kind, front, 4.0)]).unwrap();
    let g = &out[0].1;
    let r = 24.0;
    assert!((g.size[0] - 2.0 * r).abs() < 1e-9);
    for p in &g.scene.prims {
        match &p.shape {
            Shape::Line { a, b } => {
                assert!(a[0].hypot(a[1]) <= r + 1e-6 && b[0].hypot(b[1]) <= r + 1e-6);
            }
            Shape::Circle { c, r: rr } => {
                assert!(*rr == r && c[0].hypot(c[1]) < 1e-12 || c[0].hypot(c[1]) + rr <= r + 1e-6)
            }
            _ => {}
        }
    }
    // the two plate edges at the corner meet at a point 24 mm from the centre of the detail only
    // when the corner is at the centre: the corner is the centre of the circle
    let vis = lines(g, Layer::Visible);
    assert!(vis.len() >= 2, "{vis:?}");
    assert!(g.scene.prims.iter().any(
        |p| matches!(p.shape, Shape::Circle { r: rr, .. } if rr == r && p.layer == Layer::Symbol)
    ));
    // the corner sits at the middle of the detail
    assert!(
        vis.iter()
            .any(|(a, b)| a[0].hypot(a[1]) < 1e-6 || b[0].hypot(b[1]) < 1e-6),
        "{vis:?}"
    );
}

#[test]
fn section_and_detail_views_survive_a_save() {
    use kw_drawing::*;
    let mut set = DrawingSet::default();
    let s = set.add_sheet("S", SheetFormat::default());
    for kind in [
        ViewKind::Section {
            parent: 1,
            a: [0.0, 0.0],
            b: [0.0, 10.0],
            flip: true,
            align: Align::Horizontal,
            letter: "A".into(),
        },
        ViewKind::Detail {
            parent: 1,
            center: [1.0, 2.0],
            radius: 3.0,
            letter: "B".into(),
        },
    ] {
        let id = set.fresh_id();
        set.sheets[s].views.push(View {
            id,
            kind,
            source: ViewSource::Design,
            orientation: Orientation::Front,
            scale: 2.0,
            style: ViewStyle::Visible,
            pos: [1.0, 2.0],
            label: String::new(),
            cache: Some(ViewCache {
                source_hash: 3,
                scene: Scene2d::new(),
                centre: [1.5, -2.5],
            }),
        });
    }
    assert_eq!(DrawingSet::from_bytes(&set.to_bytes()).unwrap(), set);
}
