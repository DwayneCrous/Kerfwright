use kw_math::Frame3;
use kw_nurbs::geom::*;
use kw_nurbs::intersect::curve_curve;
use nalgebra::{Point3, Vector3};
use std::f64::consts::{PI, TAU};

fn line(o: [f64; 3], d: [f64; 3]) -> CurveGeom {
    CurveGeom::Line(Line {
        origin: Point3::new(o[0], o[1], o[2]),
        dir: Vector3::new(d[0], d[1], d[2]),
    })
}
fn circle(o: [f64; 3], n: [f64; 3], r: f64) -> CurveGeom {
    CurveGeom::Circle(Circle {
        frame: Frame3::from_origin_normal(
            Point3::new(o[0], o[1], o[2]),
            Vector3::new(n[0], n[1], n[2]),
        ),
        radius: r,
    })
}

#[test]
fn line_line_crossing_and_skew() {
    let h = curve_curve(
        &line([0., 0., 0.], [1., 1., 0.]),
        (0., 2.),
        &line([2., 0., 0.], [-1., 1., 0.]),
        (0., 2.),
        1e-9,
    );
    assert_eq!(h.len(), 1);
    assert!((h[0].point - Point3::new(1., 1., 0.)).norm() < 1e-12 && (h[0].t1 - 1.0).abs() < 1e-12);
    // skew lines do not intersect
    assert!(curve_curve(
        &line([0., 0., 0.], [1., 0., 0.]),
        (0., 2.),
        &line([1., -1., 0.5], [0., 1., 0.]),
        (0., 2.),
        1e-9
    )
    .is_empty());
    // out of range
    assert!(curve_curve(
        &line([0., 0., 0.], [1., 1., 0.]),
        (0., 0.5),
        &line([2., 0., 0.], [-1., 1., 0.]),
        (0., 2.),
        1e-9
    )
    .is_empty());
}

#[test]
fn line_circle_and_circle_circle() {
    let c = circle([0., 0., 0.], [0., 0., 1.], 2.0);
    let h = curve_curve(
        &line([-3., 0.5, 0.], [1., 0., 0.]),
        (0., 6.),
        &c,
        (0., TAU),
        1e-9,
    );
    assert_eq!(h.len(), 2);
    for x in &h {
        assert!(
            (x.point.coords.xy().norm() - 2.0).abs() < 1e-11 && (x.point.y - 0.5).abs() < 1e-11
        );
    }
    // two circles in the same plane meeting at two points
    let c2 = circle([2., 0., 0.], [0., 0., 1.], 2.0);
    let h = curve_curve(&c, (0., TAU), &c2, (0., TAU), 1e-9);
    assert_eq!(h.len(), 2, "{h:?}");
    // circles in perpendicular planes through common points
    let c3 = circle([0., 0., 0.], [1., 0., 0.], 2.0);
    let h = curve_curve(&c, (0., TAU), &c3, (0., TAU), 1e-9);
    assert_eq!(h.len(), 2, "{h:?}");
    // partial range
    let h = curve_curve(&c, (0., PI / 2.), &c3, (0., TAU), 1e-9);
    assert_eq!(h.len(), 1);
    assert!((h[0].point - Point3::new(0., -2., 0.)).norm() < 1e-10);
}
