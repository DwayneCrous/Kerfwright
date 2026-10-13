use kw_brep::BRep;
use kw_drawing::annot::{self, AnnotKind, Annotation, Gdt};
use kw_drawing::dim::{
    self, dim_text, format_value, layout, measure, Anchor, DimKind, Dimension, Resolved, Tolerance,
};
use kw_drawing::projection::ViewFrame;
use kw_drawing::snap::{candidates, distance, near_visible, CandKind};
use kw_drawing::*;
use kw_math::{Frame3, Vector3};
use kw_model::{extrude, ExtrudeOptions, Profile, ProfileEdge, ProfileLoop};

fn front_view(scale: f64) -> (View, ViewCache) {
    let cache = ViewCache {
        source_hash: 0,
        scene: Scene2d::new(),
        centre: [0.0, 0.0],
        marks: vec![],
        axes: vec![],
    };
    let v = View {
        id: 1,
        kind: ViewKind::Base,
        source: ViewSource::Design,
        orientation: Orientation::Front,
        scale,
        style: ViewStyle::Visible,
        pos: [100.0, 100.0],
        label: String::new(),
        cache: Some(cache.clone()),
        center_marks: true,
    };
    (v, cache)
}

fn p(x: f64, y: f64, z: f64) -> Resolved {
    Resolved::Point([x, y, z])
}

fn dim(kind: DimKind, r: Vec<Resolved>) -> Dimension {
    let mut d = Dimension::new(
        1,
        kind,
        vec![Anchor::Fixed(Resolved::Point([0.0; 3])); r.len()],
    );
    d.resolved = r;
    d.value = measure(&d, &ViewFrame::of(Orientation::Front)).unwrap();
    d
}

fn texts(s: &Scene2d) -> Vec<String> {
    s.prims
        .iter()
        .filter_map(|p| match &p.shape {
            Shape::Text { text, .. } => Some(text.clone()),
            _ => None,
        })
        .collect()
}

#[test]
fn values_are_measured_in_the_view_plane_in_model_units() {
    let f = ViewFrame::of(Orientation::Front); // x to the right, z up, y is depth
    let (a, b) = (p(0.0, 7.0, 0.0), p(40.0, -3.0, 5.0));
    assert_eq!(
        measure(&dim(DimKind::Horizontal, vec![a.clone(), b.clone()]), &f),
        Some(40.0)
    );
    assert_eq!(
        measure(&dim(DimKind::Vertical, vec![a.clone(), b.clone()]), &f),
        Some(5.0)
    );
    let al = measure(&dim(DimKind::Aligned, vec![a, b]), &f).unwrap();
    assert!(
        (al - 40.0_f64.hypot(5.0)).abs() < 1e-12,
        "depth does not count: {al}"
    );
    let c = Resolved::Circle {
        centre: [1.0, 2.0, 3.0],
        radius: 8.0,
        axis: [0.0, 1.0, 0.0],
    };
    assert_eq!(
        measure(&dim(DimKind::Diameter, vec![c.clone()]), &f),
        Some(16.0)
    );
    assert_eq!(measure(&dim(DimKind::Radius, vec![c]), &f), Some(8.0));
    let l1 = Resolved::Line {
        a: [0.0, 0.0, 0.0],
        b: [10.0, 0.0, 0.0],
    };
    let l2 = Resolved::Line {
        a: [0.0, 0.0, 0.0],
        b: [10.0, 0.0, 10.0],
    };
    let ang = measure(&dim(DimKind::Angle, vec![l1, l2]), &f).unwrap();
    assert!((ang - 45.0).abs() < 1e-9, "{ang}");
}

#[test]
fn text_follows_the_conventions_for_units_tolerances_and_hole_callouts() {
    assert_eq!(
        format_value(40.0, DimKind::Horizontal, Units::Mm, None),
        "40"
    );
    assert_eq!(
        format_value(12.5, DimKind::Horizontal, Units::Mm, None),
        "12.5"
    );
    assert_eq!(
        format_value(7.25, DimKind::Horizontal, Units::Mm, Some(3)),
        "7.250"
    );
    assert_eq!(
        format_value(25.4, DimKind::Horizontal, Units::Inch, None),
        "1"
    );
    assert_eq!(
        format_value(12.7, DimKind::Horizontal, Units::Inch, None),
        "0.5"
    );
    let mut d = dim(
        DimKind::Horizontal,
        vec![p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0)],
    );
    assert_eq!(dim_text(&d, Units::Mm).main, "40");
    d.tolerance = Tolerance::Symmetric(0.1);
    assert_eq!(dim_text(&d, Units::Mm).main, "40 ±0.1");
    d.tolerance = Tolerance::Bilateral {
        upper: 0.1,
        lower: 0.05,
    };
    let t = dim_text(&d, Units::Mm);
    assert_eq!(
        (t.main.as_str(), t.upper.as_deref(), t.lower.as_deref()),
        ("40", Some("+0.1"), Some("-0.05"))
    );
    d.tolerance = Tolerance::Limits {
        upper: 0.1,
        lower: 0.2,
    };
    let t = dim_text(&d, Units::Mm);
    assert_eq!(
        (t.upper.as_deref(), t.lower.as_deref()),
        (Some("40.1"), Some("39.8"))
    );
    d.tolerance = Tolerance::None;
    d.text = "40 TYP".into();
    assert_eq!(dim_text(&d, Units::Mm).main, "40 TYP");
    // a hole: 4 x diameter 8 through
    let c = Resolved::Circle {
        centre: [0.0; 3],
        radius: 4.0,
        axis: [0.0, 1.0, 0.0],
    };
    let mut h = dim(DimKind::Diameter, vec![c]);
    h.prefix = "4×".into();
    h.suffix = " THRU".into();
    assert_eq!(dim_text(&h, Units::Mm).main, "4×Ø8 THRU");
    let mut r = dim(
        DimKind::Radius,
        vec![Resolved::Circle {
            centre: [0.0; 3],
            radius: 5.0,
            axis: [0.0, 1.0, 0.0],
        }],
    );
    assert_eq!(dim_text(&r, Units::Mm).main, "R5");
    r.kind = DimKind::Angle;
    r.value = 45.0;
    assert_eq!(dim_text(&r, Units::Mm).main, "45°");
}

fn lines(s: &Scene2d) -> Vec<([f64; 2], [f64; 2])> {
    s.prims
        .iter()
        .filter_map(|p| match p.shape {
            Shape::Line { a, b } => Some((a, b)),
            _ => None,
        })
        .collect()
}

#[test]
fn a_horizontal_dimension_is_drawn_with_extension_lines_arrows_and_text_over_its_line() {
    for scale in [1.0, 2.0, 5.0] {
        let (v, c) = front_view(scale);
        let mut d = dim(
            DimKind::Horizontal,
            vec![p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0)],
        );
        d.line_pos = 15.0;
        let s = layout(&d, &v, &c, Units::Mm);
        let y = 100.0 + 15.0;
        let w = 40.0 * scale;
        let l = lines(&s);
        // the dimension line, between the extension lines' feet
        assert!(
            l.iter().any(|(a, b)| a[1] == y
                && b[1] == y
                && (a[0] - 100.0).abs() < 1e-9
                && (b[0] - (100.0 + w)).abs() < 1e-9),
            "{l:?}"
        );
        // two extension lines, vertical, starting 1 mm off the feature and passing the line by 2 mm
        let ext: Vec<_> = l
            .iter()
            .filter(|(a, b)| a[0] == b[0] && (a[1] - b[1]).abs() > 1.0)
            .collect();
        assert_eq!(ext.len(), 2);
        for (a, b) in ext {
            assert!((a[1] - 101.0).abs() < 1e-9 && (b[1] - (y + 2.0)).abs() < 1e-9);
        }
        // two arrowheads
        assert_eq!(
            s.prims
                .iter()
                .filter(|p| matches!(p.shape, Shape::Fill { .. }))
                .count(),
            2
        );
        // the text is the model's value whatever the scale, centred over the line
        let txt = s.prims.iter().find_map(|p| match &p.shape {
            Shape::Text {
                text, p, anchor, ..
            } if text == "40" => Some((*p, *anchor)),
            _ => None,
        });
        let (pos, _) = txt.expect("text 40");
        assert!(pos[1] > y);
        let bounds = s.bounds().unwrap();
        assert!(bounds.0[0] <= 100.0 && bounds.1[0] >= 100.0 + w);
    }
}

#[test]
fn a_short_dimension_puts_its_arrows_outside() {
    let (v, c) = front_view(1.0);
    let mut d = dim(
        DimKind::Horizontal,
        vec![p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0)],
    );
    d.line_pos = 10.0;
    let s = layout(&d, &v, &c, Units::Mm);
    let l = lines(&s);
    // stubs run outward beyond both ends
    assert!(l
        .iter()
        .any(|(a, b)| a[1] == 110.0 && (a[0].min(b[0]) < 100.0 - 1.0)));
    assert!(l
        .iter()
        .any(|(a, b)| a[1] == 110.0 && (a[0].max(b[0]) > 104.0 + 1.0)));
}

#[test]
fn a_vertical_dimension_reads_from_the_right_and_an_aligned_one_along_its_line() {
    let (v, c) = front_view(1.0);
    let mut d = dim(DimKind::Vertical, vec![p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0)]);
    d.line_pos = -8.0;
    let s = layout(&d, &v, &c, Units::Mm);
    let ang = s.prims.iter().find_map(|p| match &p.shape {
        Shape::Text { text, angle, .. } if text == "10" => Some(*angle),
        _ => None,
    });
    assert!((ang.unwrap() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    let mut a = dim(DimKind::Aligned, vec![p(0.0, 0.0, 0.0), p(30.0, 0.0, 40.0)]);
    a.line_pos = 6.0;
    let s = layout(&a, &v, &c, Units::Mm);
    let (text, ang) = s
        .prims
        .iter()
        .find_map(|p| match &p.shape {
            Shape::Text { text, angle, .. } => Some((text.clone(), *angle)),
            _ => None,
        })
        .unwrap();
    assert_eq!(text, "50");
    assert!(
        (ang - 40.0_f64.atan2(30.0)).abs() < 1e-12,
        "text runs along the line"
    );
}

#[test]
fn diameters_radii_and_angles_have_leaders_arcs_and_arrows() {
    let (v, c) = front_view(2.0);
    let circle = Resolved::Circle {
        centre: [10.0, 0.0, 10.0],
        radius: 4.0,
        axis: [0.0, 1.0, 0.0],
    };
    let mut d = dim(DimKind::Diameter, vec![circle.clone()]);
    d.angle = std::f64::consts::FRAC_PI_4;
    let s = layout(&d, &v, &c, Units::Mm);
    assert_eq!(texts(&s), vec!["Ø8".to_string()]);
    assert_eq!(
        s.prims
            .iter()
            .filter(|p| matches!(p.shape, Shape::Fill { .. }))
            .count(),
        2
    );
    // the leader reaches from the far side of the circle (centre at 120, 120; radius 8 mm)
    let l = lines(&s);
    let far = (120.0 - 8.0 * 0.5_f64.sqrt(), 120.0 - 8.0 * 0.5_f64.sqrt());
    assert!(l.iter().any(
        |(a, b)| (a[0] - far.0).abs() < 1e-9 && (a[1] - far.1).abs() < 1e-9
            || (b[0] - far.0).abs() < 1e-9 && (b[1] - far.1).abs() < 1e-9
    ));
    let r = dim(DimKind::Radius, vec![circle]);
    assert_eq!(
        texts(&layout(&r, &v, &c, Units::Mm)),
        vec!["R4".to_string()]
    );
    // an angle: an arc about the vertex between the two edges
    let l1 = Resolved::Line {
        a: [0.0, 0.0, 0.0],
        b: [10.0, 0.0, 0.0],
    };
    let l2 = Resolved::Line {
        a: [0.0, 0.0, 0.0],
        b: [10.0, 0.0, 10.0],
    };
    let mut an = dim(DimKind::Angle, vec![l1, l2]);
    an.line_pos = 12.0;
    let s = layout(&an, &v, &c, Units::Mm);
    assert_eq!(texts(&s), vec!["45°".to_string()]);
    let arc = s
        .prims
        .iter()
        .find_map(|p| match p.shape {
            Shape::Arc { c, r, a0, a1 } => Some((c, r, a0, a1)),
            _ => None,
        })
        .unwrap();
    assert_eq!(arc.0, [100.0, 100.0]);
    assert_eq!(arc.1, 12.0);
    assert!(((arc.3 - arc.2) - std::f64::consts::FRAC_PI_4).abs() < 1e-9);
}

#[test]
fn notes_and_symbols_draw_inside_their_frames() {
    let note = Annotation {
        id: 1,
        kind: AnnotKind::Note {
            pos: [50.0, 80.0],
            text: "DEBURR\nALL EDGES".into(),
            height: 3.5,
            leader: Some([20.0, 60.0]),
        },
    };
    let s = annot::layout(&note);
    assert_eq!(
        texts(&s),
        vec!["DEBURR".to_string(), "ALL EDGES".to_string()]
    );
    assert!(
        lines(&s)
            .iter()
            .any(|(a, b)| *b == [20.0, 60.0] || *a == [20.0, 60.0]),
        "the leader ends on its target"
    );
    let datum = Annotation {
        id: 2,
        kind: AnnotKind::Datum {
            pos: [30.0, 30.0],
            letter: "A".into(),
            target: [30.0, 10.0],
        },
    };
    let s = annot::layout(&datum);
    assert_eq!(texts(&s), vec!["A".to_string()]);
    assert!(
        s.prims
            .iter()
            .any(|p| matches!(p.shape, Shape::Fill { .. })),
        "the triangle"
    );
    let sf = Annotation {
        id: 3,
        kind: AnnotKind::Surface {
            pos: [10.0, 10.0],
            value: "Ra 1.6".into(),
        },
    };
    assert_eq!(texts(&annot::layout(&sf)), vec!["Ra 1.6".to_string()]);
    // a control frame: symbol cell, tolerance, two datums
    let fcf = Annotation {
        id: 4,
        kind: AnnotKind::Fcf {
            pos: [10.0, 10.0],
            symbol: Gdt::Position,
            tol: "Ø0.1".into(),
            datums: vec!["A".into(), "B".into()],
            target: None,
        },
    };
    let s = annot::layout(&fcf);
    assert_eq!(
        texts(&s),
        vec!["Ø0.1".to_string(), "A".to_string(), "B".to_string()]
    );
    let (lo, hi) = s.bounds().unwrap();
    assert_eq!(lo[1], 10.0);
    assert!(
        (hi[1] - lo[1] - annot::BOX_H).abs() < 1e-9 || hi[1] - lo[1] >= annot::BOX_H,
        "{lo:?} {hi:?}"
    );
    // four cell separators plus the frame: 3 inner dividers
    let dividers = lines(&s)
        .iter()
        .filter(|(a, b)| a[0] == b[0] && (a[1] - b[1]).abs() > 6.9)
        .count();
    assert_eq!(dividers, 3);
    // every characteristic symbol draws something that stays inside its cell
    for g in Gdt::ALL {
        let mut s = Scene2d::new();
        annot::gdt_glyph(&mut s, g, [0.0, 0.0]);
        let (lo, hi) = s.bounds().unwrap_or_else(|| panic!("{g:?} draws nothing"));
        // (the bounds of an arc are those of its whole circle)
        assert!(
            lo[0] >= -3.8 && hi[0] <= 3.8 && lo[1] >= -5.1 && hi[1] <= 3.8,
            "{g:?}: {lo:?} {hi:?}"
        );
    }
}

#[test]
fn centre_marks_follow_the_circles_and_cylinders_of_a_view() {
    let (mut v, mut c) = front_view(1.0);
    c.marks = vec![([10.0, 5.0], 4.0), ([0.0, 0.0], 0.1)];
    c.axes = vec![([-20.0, 0.0], [20.0, 0.0])];
    v.cache = Some(c);
    let s = annot::center_marks(&v);
    let l = lines(&s);
    // a cross reaching 3 mm past the circle (the tiny one gets none) and a centre line 3 mm past each end
    assert!(l.contains(&([100.0 + 10.0 - 7.0, 105.0], [100.0 + 10.0 + 7.0, 105.0])));
    assert!(l.contains(&([110.0, 105.0 - 7.0], [110.0, 105.0 + 7.0])));
    assert!(l.contains(&([100.0 - 23.0, 100.0], [100.0 + 23.0, 100.0])));
    assert_eq!(l.len(), 3);
    assert!(s.prims.iter().all(|p| p.layer == Layer::Center));
    v.center_marks = false;
    assert!(annot::center_marks(&v).prims.is_empty());
}

fn plate() -> (BRep, kw_brep::BodyId) {
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

#[test]
fn snap_candidates_are_the_points_and_edges_of_the_model_seen_in_the_view() {
    let (rep, b) = plate();
    let top = ViewFrame::of(Orientation::Top);
    let cands = candidates(&rep, &[b], &top, 2.0, [0.0, 0.0]);
    // 4 + 4 plate edges (straight) and the hole's two circles
    let circles: Vec<_> = cands
        .iter()
        .filter(|c| matches!(c.kind, CandKind::Circle))
        .collect();
    assert_eq!(circles.len(), 2);
    for c in &circles {
        let (centre, r, axis) = c.circle.unwrap();
        assert_eq!(r, 5.0);
        assert!(axis[2].abs() > 0.999 && centre[0].abs() < 1e-9);
    }
    // a corner of the plate (20, 20) in the top view at scale 2 is at (40, 40)
    assert!(cands.iter().any(|c| matches!(
        c.kind,
        CandKind::Point(PointKind::Start | PointKind::End)
    ) && (c.p2[0] - 40.0).abs() < 1e-9
        && (c.p2[1] - 40.0).abs() < 1e-9));
    // the middle of an edge, and the hole's centre
    assert!(cands
        .iter()
        .any(|c| matches!(c.kind, CandKind::Point(PointKind::Mid))
            && (c.p2[0] - 40.0).abs() < 1e-9
            && c.p2[1].abs() < 1e-9));
    assert!(cands
        .iter()
        .any(|c| matches!(c.kind, CandKind::Point(PointKind::Centre)) && c.p2 == [0.0, 0.0]));
    // picking the circle: near its rim, not near its middle
    let c = circles[0];
    assert!(distance(c, [10.0, 0.0], 2.0, &top) < 1e-9);
    assert!(distance(c, [0.0, 0.0], 2.0, &top) > 9.0);
    // seen from the front a hole's circle cannot be picked
    let front = ViewFrame::of(Orientation::Front);
    assert!(distance(c, [10.0, 0.0], 2.0, &front).is_infinite());
    // straight edges are picked along their length
    let line = cands
        .iter()
        .find(|c| matches!(c.kind, CandKind::Line))
        .unwrap();
    let (a, z) = line.line2.unwrap();
    assert!(distance(line, [(a[0] + z[0]) / 2.0, (a[1] + z[1]) / 2.0], 2.0, &top) < 1e-9);
}

#[test]
fn only_what_is_drawn_visible_is_near() {
    let mut s = Scene2d::new();
    s.line(Layer::Visible, [0.0, 0.0], [10.0, 0.0]);
    s.line(Layer::Hidden, [0.0, 5.0], [10.0, 5.0]);
    s.circle(Layer::Visible, [30.0, 0.0], 5.0);
    assert!(near_visible(&s, [5.0, 0.2], 0.3));
    assert!(
        !near_visible(&s, [5.0, 5.0], 0.3),
        "hidden lines are not offered"
    );
    assert!(near_visible(&s, [35.0, 0.0], 0.3));
    assert!(!near_visible(&s, [30.0, 0.0], 0.3));
}

#[test]
fn dimensions_and_notes_ride_along_in_the_saved_set() {
    let mut set = DrawingSet::default();
    let i = set.add_sheet("S", SheetFormat::default());
    let mut d = dim(
        DimKind::Horizontal,
        vec![p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0)],
    );
    d.anchors = vec![
        Anchor::Fixed(Resolved::Point([0.0; 3])),
        Anchor::Fixed(Resolved::Point([40.0, 0.0, 0.0])),
    ];
    d.tolerance = Tolerance::Bilateral {
        upper: 0.1,
        lower: 0.2,
    };
    d.status = DimStatus::Broken("the face is gone".into());
    let id = set.fresh_id();
    set.sheets[i].annots.push(Annotation {
        id,
        kind: AnnotKind::Dimension(d),
    });
    let id = set.fresh_id();
    set.sheets[i].annots.push(Annotation {
        id,
        kind: AnnotKind::Fcf {
            pos: [1.0, 2.0],
            symbol: Gdt::Flatness,
            tol: "0.05".into(),
            datums: vec![],
            target: Some([5.0, 5.0]),
        },
    });
    assert_eq!(DrawingSet::from_bytes(&set.to_bytes()).unwrap(), set);
    let _ = dim::TEXT_H;
}

#[test]
fn placing_a_dimension_at_a_point_is_what_layout_draws_through_that_point() {
    let (v, c) = front_view(2.0);
    // horizontal / vertical: the line passes through the point
    let mut h = dim(
        DimKind::Horizontal,
        vec![p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0)],
    );
    dim::place(&mut h, &v, &c, [30.0, 17.0]);
    assert_eq!(h.line_pos, 17.0);
    let mut vt = dim(DimKind::Vertical, vec![p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0)]);
    dim::place(&mut vt, &v, &c, [-9.0, 3.0]);
    assert_eq!(vt.line_pos, -9.0);
    // aligned: signed distance from the measured line, on the side of the point
    let mut al = dim(DimKind::Aligned, vec![p(0.0, 0.0, 0.0), p(30.0, 0.0, 40.0)]);
    let n = [-40.0 / 50.0, 30.0 / 50.0];
    let at = [0.0 + n[0] * 6.0 + 10.0 * 0.6, 0.0 + n[1] * 6.0 + 10.0 * 0.8];
    dim::place(&mut al, &v, &c, [at[0] * 1.0, at[1] * 1.0]);
    // the geometry is at scale 2: the points are (0,0) and (60,80) on the sheet
    assert!(al.line_pos.abs() > 0.0);
    let s = layout(&al, &v, &c, Units::Mm);
    let dl = lines(&s);
    // the dimension line passes through the placed point
    let through = dl.iter().any(|(a, b)| {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = dx.hypot(dy);
        l > 20.0 && (((100.0 + at[0]) - a[0]) * dy - ((100.0 + at[1]) - a[1]) * dx).abs() / l < 1e-6
    });
    assert!(through, "{dl:?}");
    // a diameter points its leader at the point; an angle puts its arc through it
    let circle = Resolved::Circle {
        centre: [10.0, 0.0, 10.0],
        radius: 4.0,
        axis: [0.0, 1.0, 0.0],
    };
    let mut d = dim(DimKind::Diameter, vec![circle]);
    dim::place(&mut d, &v, &c, [20.0 + 5.0, 20.0 + 5.0]);
    assert!((d.angle - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
    let l1 = Resolved::Line {
        a: [0.0, 0.0, 0.0],
        b: [10.0, 0.0, 0.0],
    };
    let l2 = Resolved::Line {
        a: [0.0, 0.0, 0.0],
        b: [10.0, 0.0, 10.0],
    };
    let mut an = dim(DimKind::Angle, vec![l1, l2]);
    dim::place(&mut an, &v, &c, [9.0, 12.0]);
    assert!((an.line_pos - 15.0).abs() < 1e-9);
}
