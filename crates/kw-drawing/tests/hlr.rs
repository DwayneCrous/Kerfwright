use kw_brep::{BRep, BodyId};
use kw_drawing::hlr::{project_bodies, HlrOptions, ViewGeometry};
use kw_drawing::projection::ViewFrame;
use kw_drawing::{Layer, Orientation, Shape, ViewStyle};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{extrude, ExtrudeOptions, Profile, ProfileEdge, ProfileLoop};
use std::f64::consts::{FRAC_PI_2, PI};

fn solid(rep: &mut BRep, p: Profile, h: f64) -> BodyId {
    extrude(
        rep,
        &p,
        Vector3::new(0.0, 0.0, h),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body
}

fn view(rep: &BRep, b: BodyId, o: Orientation, style: ViewStyle) -> ViewGeometry {
    project_bodies(
        rep,
        &[b],
        &HlrOptions {
            frame: ViewFrame::of(o),
            scale: 1.0,
            style,
        },
    )
    .unwrap()
}

fn count(g: &ViewGeometry, layer: Layer, f: impl Fn(&Shape) -> bool) -> usize {
    g.scene
        .prims
        .iter()
        .filter(|p| p.layer == layer && f(&p.shape))
        .count()
}
fn is_line(s: &Shape) -> bool {
    matches!(s, Shape::Line { .. })
}

/// One short line per primitive, for failure messages.
fn summary(g: &ViewGeometry) -> String {
    let mut out = String::new();
    for p in g.scene.prims.iter().take(24) {
        let kind = match &p.shape {
            Shape::Line { a, b } => format!("line {a:.2?}-{b:.2?}"),
            Shape::Ellipse { t0, t1, .. } => format!("ellipse {t0:.2}..{t1:.2}"),
            Shape::Circle { r, .. } => format!("circle {r:.2}"),
            _ => "other".into(),
        };
        out += &format!("\n{:?} {kind}", p.layer);
    }
    format!("{} prims{out}", g.scene.prims.len())
}

fn bbox() -> BRep {
    BRep::new()
}

#[test]
fn box_views() {
    let mut rep = bbox();
    let b = solid(
        &mut rep,
        Profile::rectangle(Frame3::world(), 40.0, 20.0),
        10.0,
    );
    // front: a 40 x 10 rectangle; the back edges are hidden behind it and coincide
    let g = view(&rep, b, Orientation::Front, ViewStyle::VisibleHidden);
    assert_eq!(count(&g, Layer::Visible, is_line), 4, "{}", summary(&g));
    assert_eq!(count(&g, Layer::Hidden, |_| true), 0);
    assert!((g.size[0] - 40.0).abs() < 1e-9 && (g.size[1] - 10.0).abs() < 1e-9);
    // top: 40 x 20
    let g = view(&rep, b, Orientation::Top, ViewStyle::VisibleHidden);
    assert_eq!(count(&g, Layer::Visible, is_line), 4);
    assert!((g.size[0] - 40.0).abs() < 1e-9 && (g.size[1] - 20.0).abs() < 1e-9);
    // right: 20 x 10
    let g = view(&rep, b, Orientation::Right, ViewStyle::Visible);
    assert!((g.size[0] - 20.0).abs() < 1e-9 && (g.size[1] - 10.0).abs() < 1e-9);
}

#[test]
fn isometric_box_has_nine_visible_and_three_hidden_edges() {
    let mut rep = bbox();
    let b = solid(
        &mut rep,
        Profile::rectangle(Frame3::world(), 40.0, 20.0),
        10.0,
    );
    let g = view(&rep, b, Orientation::Iso, ViewStyle::VisibleHidden);
    assert_eq!(count(&g, Layer::Visible, is_line), 9);
    assert_eq!(count(&g, Layer::Hidden, is_line), 3);
    // hidden lines are dropped in the plain visible style
    let g = view(&rep, b, Orientation::Iso, ViewStyle::Visible);
    assert_eq!(count(&g, Layer::Hidden, |_| true), 0);
    // wireframe draws all twelve as visible
    let g = view(&rep, b, Orientation::Iso, ViewStyle::Wireframe);
    assert_eq!(count(&g, Layer::Visible, is_line), 12);
}

#[test]
fn cylinder_seen_from_the_side_and_from_above() {
    let mut rep = bbox();
    let b = solid(&mut rep, Profile::disc(Frame3::world(), 10.0), 30.0);
    let g = view(&rep, b, Orientation::Front, ViewStyle::VisibleHidden);
    assert!(
        (g.size[0] - 20.0).abs() < 1e-6 && (g.size[1] - 30.0).abs() < 1e-6,
        "{:?}",
        g.size
    );
    // two silhouette lines, and the two circles seen edge-on
    assert_eq!(count(&g, Layer::Visible, is_line), 4, "{}", summary(&g));
    assert_eq!(count(&g, Layer::Hidden, |_| true), 0);
    let g = view(&rep, b, Orientation::Top, ViewStyle::VisibleHidden);
    let circles: Vec<f64> = g
        .scene
        .prims
        .iter()
        .filter_map(|p| match p.shape {
            Shape::Circle { r, .. } if p.layer == Layer::Visible => Some(r),
            _ => None,
        })
        .collect();
    assert_eq!(circles.len(), 1);
    assert!((circles[0] - 10.0).abs() < 1e-9);
    assert_eq!(
        count(&g, Layer::Hidden, |_| true),
        0,
        "the lower circle coincides"
    );
}

#[test]
fn tilted_cylinder_shows_ellipses_and_a_hidden_back_arc() {
    let mut rep = bbox();
    let b = solid(&mut rep, Profile::disc(Frame3::world(), 10.0), 30.0);
    let g = view(&rep, b, Orientation::Iso, ViewStyle::VisibleHidden);
    let ell = |s: &Shape| matches!(s, Shape::Ellipse { .. });
    assert_eq!(count(&g, Layer::Hidden, ell), 1, "{}", summary(&g));
    assert!(count(&g, Layer::Visible, ell) >= 2);
    // the hidden arc is part of the lower rim: it has the same shape as the visible rest
    let axes: Vec<(f64, f64)> = g
        .scene
        .prims
        .iter()
        .filter_map(|p| match &p.shape {
            Shape::Ellipse { major, ratio, .. } => Some((major[0].hypot(major[1]), *ratio)),
            _ => None,
        })
        .collect();
    for (a, r) in axes {
        assert!((a - 10.0).abs() < 1e-6, "semi-major {a}");
        assert!((r - (1.0_f64 / 3.0_f64.sqrt())).abs() < 1e-6, "ratio {r}");
    }
}

#[test]
fn hole_is_hidden_from_the_side_and_round_from_above() {
    let mut rep = bbox();
    let hole = ProfileLoop::new(vec![ProfileEdge::arc(
        &Frame3::world(),
        5.0,
        std::f64::consts::TAU,
        0.0,
    )]);
    let b = solid(
        &mut rep,
        Profile::rectangle(Frame3::world(), 40.0, 40.0).with_hole(hole),
        10.0,
    );
    let g = view(&rep, b, Orientation::Front, ViewStyle::VisibleHidden);
    let hidden: Vec<&Shape> = g
        .scene
        .prims
        .iter()
        .filter(|p| p.layer == Layer::Hidden)
        .map(|p| &p.shape)
        .collect();
    assert_eq!(hidden.len(), 2, "{hidden:?}");
    for s in hidden {
        let Shape::Line { a, b } = s else {
            panic!("{s:?}")
        };
        assert!((a[0].abs() - 5.0).abs() < 1e-6 && (b[0].abs() - 5.0).abs() < 1e-6);
        assert!(
            ((a[1] - b[1]).abs() - 10.0).abs() < 1e-6,
            "spans the plate thickness"
        );
    }
    assert_eq!(count(&g, Layer::Visible, is_line), 4);
    // dropped in the visible style
    let g = view(&rep, b, Orientation::Front, ViewStyle::Visible);
    assert_eq!(count(&g, Layer::Hidden, |_| true), 0);
    // from above the hole is a circle and nothing is hidden
    let g = view(&rep, b, Orientation::Top, ViewStyle::VisibleHidden);
    assert_eq!(
        count(
            &g,
            Layer::Visible,
            |s| matches!(s, Shape::Circle { r, .. } if (*r - 5.0).abs() < 1e-9)
        ),
        1
    );
    assert_eq!(count(&g, Layer::Hidden, |_| true), 0);
}

fn stadium(rep: &mut BRep) -> BodyId {
    let w = Frame3::world();
    let at = |x: f64| Frame3 {
        origin: Point3::new(x, 0.0, 0.0),
        ..w
    };
    let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    let l = ProfileLoop::new(vec![
        ProfileEdge::line(p(-15.0, -5.0), p(15.0, -5.0)),
        ProfileEdge::arc(&at(15.0), 5.0, -FRAC_PI_2, FRAC_PI_2),
        ProfileEdge::line(p(15.0, 5.0), p(-15.0, 5.0)),
        ProfileEdge::arc(&at(-15.0), 5.0, FRAC_PI_2, 3.0 * FRAC_PI_2),
    ]);
    solid(rep, Profile::new(w, l), 10.0)
}

#[test]
fn tangent_edges_are_not_drawn() {
    let mut rep = bbox();
    let b = stadium(&mut rep);
    // from the front the stadium reads as a 40 x 10 rectangle; the tangent joins at x = +-15
    // are not lines, the round ends' silhouettes at x = +-20 are
    let g = view(&rep, b, Orientation::Front, ViewStyle::VisibleHidden);
    let verticals: Vec<f64> = g
        .scene
        .prims
        .iter()
        .filter_map(|p| match &p.shape {
            Shape::Line { a, b } if p.layer == Layer::Visible && (a[0] - b[0]).abs() < 1e-9 => {
                Some(a[0])
            }
            _ => None,
        })
        .collect();
    assert!(
        !verticals.iter().any(|x| (x.abs() - 15.0).abs() < 1e-6),
        "{verticals:?}"
    );
    assert!(
        verticals.iter().any(|x| (x.abs() - 20.0).abs() < 1e-3),
        "{verticals:?}"
    );
    assert!((g.size[0] - 40.0).abs() < 1e-3);
    let _ = PI;
}

#[test]
fn output_is_deterministic_and_shaded_adds_fills() {
    let mut rep = bbox();
    let b = solid(&mut rep, Profile::disc(Frame3::world(), 10.0), 30.0);
    let a = view(&rep, b, Orientation::Iso, ViewStyle::VisibleHidden);
    let c = view(&rep, b, Orientation::Iso, ViewStyle::VisibleHidden);
    assert_eq!(a.scene, c.scene);
    let s = view(&rep, b, Orientation::Iso, ViewStyle::Shaded);
    assert!(s
        .scene
        .prims
        .iter()
        .any(|p| matches!(p.shape, Shape::Fill { .. })));
    // fills come first so the lines are on top
    assert!(matches!(s.scene.prims[0].shape, Shape::Fill { .. }));
}

#[test]
fn scale_scales_the_view() {
    let mut rep = bbox();
    let b = solid(
        &mut rep,
        Profile::rectangle(Frame3::world(), 40.0, 20.0),
        10.0,
    );
    let g = project_bodies(
        &rep,
        &[b],
        &HlrOptions {
            frame: ViewFrame::of(Orientation::Front),
            scale: 0.5,
            style: ViewStyle::Visible,
        },
    )
    .unwrap();
    assert!((g.size[0] - 20.0).abs() < 1e-9 && (g.size[1] - 5.0).abs() < 1e-9);
}

#[test]
fn a_body_behind_another_is_hidden_by_it() {
    // a small block standing behind a larger one, seen from the front (+y is away from the eye)
    let mut rep = bbox();
    let big = solid(
        &mut rep,
        Profile::rectangle(Frame3::world(), 40.0, 20.0),
        30.0,
    );
    let at = Frame3 {
        origin: Point3::new(0.0, 30.0, 5.0),
        ..Frame3::world()
    };
    let small = solid(&mut rep, Profile::rectangle(at, 10.0, 10.0), 10.0);
    let g = project_bodies(
        &rep,
        &[big, small],
        &HlrOptions {
            frame: ViewFrame::of(Orientation::Front),
            scale: 1.0,
            style: ViewStyle::VisibleHidden,
        },
    )
    .unwrap();
    // it lies wholly inside the outline of the big block: all its lines are hidden
    assert_eq!(count(&g, Layer::Visible, is_line), 4, "{}", summary(&g));
    assert!(count(&g, Layer::Hidden, is_line) >= 4, "{}", summary(&g));
    // and with the small block in front it is drawn visible instead
    let g = project_bodies(
        &rep,
        &[big, small],
        &HlrOptions {
            frame: ViewFrame::of(Orientation::Back),
            scale: 1.0,
            style: ViewStyle::VisibleHidden,
        },
    )
    .unwrap();
    assert_eq!(count(&g, Layer::Visible, is_line), 8, "{}", summary(&g));
}
