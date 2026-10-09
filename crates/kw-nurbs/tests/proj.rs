use kw_math::{Frame3, Transform3};
use kw_nurbs::fit::*;
use kw_nurbs::geom::*;
use kw_nurbs::proj::*;
use nalgebra::{Point3, Vector3};
use proptest::prelude::*;

fn frame(seed: f64) -> Frame3 {
    let mut f = Frame3::from_origin_normal(
        Point3::new(0.3 * seed, -1.0, 2.0),
        Vector3::new(0.2 + seed, 0.5, 1.0),
    );
    // rotate x,y about z so frames are not canonical
    let (c, s) = (seed.cos(), seed.sin());
    let (x, y) = (f.x * c + f.y * s, -f.x * s + f.y * c);
    f.x = x;
    f.y = y;
    f
}

fn surfaces(seed: f64) -> Vec<SurfaceGeom> {
    let f = frame(seed);
    vec![
        SurfaceGeom::Plane(Plane { frame: f }),
        SurfaceGeom::Cylinder(Cylinder {
            frame: f,
            radius: 1.5,
        }),
        SurfaceGeom::Cone(Cone {
            frame: f,
            radius_at_origin: 2.0,
            half_angle: 0.3,
        }),
        SurfaceGeom::Cone(Cone {
            frame: f,
            radius_at_origin: 2.0,
            half_angle: -0.4,
        }),
        SurfaceGeom::Sphere(Sphere {
            frame: f,
            radius: 2.0,
        }),
        SurfaceGeom::Torus(Torus {
            frame: f,
            major_radius: 5.0,
            minor_radius: 1.0,
        }),
        SurfaceGeom::Extrusion {
            profile: Box::new(CurveGeom::Circle(Circle {
                frame: f,
                radius: 1.0,
            })),
            dir: f.z * 3.0 + f.x * 0.5,
        },
        SurfaceGeom::Revolution {
            profile: Box::new(CurveGeom::Line(Line {
                origin: Point3::new(2.0, 0.0, 0.0),
                dir: Vector3::new(0.3, 0.0, 1.0),
            })),
            axis_origin: Point3::origin(),
            axis_dir: Vector3::z(),
        },
    ]
}

proptest! {
    #[test]
    fn surface_param_of_inverts_point_at(seed in 0.0f64..6.0, uf in 0.05f64..0.95, vf in 0.1f64..0.9) {
        for s in surfaces(seed) {
            let ((u0, u1), (v0, v1)) = s.domain();
            let clip = |a: f64, b: f64, f: f64| {
                let (a, b) = (if a.is_finite() { a } else { -2.0 }, if b.is_finite() { b } else { 2.0 });
                a + (b - a) * f
            };
            let (u, v) = (clip(u0, u1, uf), clip(v0, v1, vf));
            let p = s.point_at(u, v);
            let r = surface_param_of(&s, &p, Some((u + 0.01, v - 0.01)));
            prop_assert!(r.distance < 1e-9, "{s:?} dist {}", r.distance);
            let q = s.point_at(r.u, r.v);
            prop_assert!((q - p).norm() < 1e-9);
        }
    }

    #[test]
    fn curve_param_of_inverts_point_at(seed in 0.0f64..6.0, tf in 0.02f64..0.98) {
        let f = frame(seed);
        let curves = vec![
            CurveGeom::Line(Line { origin: f.origin, dir: f.x * 2.0 + f.z }),
            CurveGeom::Circle(Circle { frame: f, radius: 2.5 }),
            CurveGeom::Ellipse(Ellipse { frame: f, major: 3.0, minor: 1.2 }),
        ];
        for c in curves {
            let (a, b) = match &c { CurveGeom::Line(_) => (-1.0, 3.0), _ => (0.0, std::f64::consts::TAU) };
            let t = a + (b - a) * tf;
            let p = c.point_at(t);
            let r = curve_param_of(&c, &p, Some(t));
            prop_assert!(r.distance < 1e-9);
            prop_assert!((r.t - t).abs() < 1e-7, "{c:?}: {} vs {t}", r.t);
        }
    }
}

#[test]
fn hermite_fit_reproduces_a_helix_to_tolerance() {
    let mut f = |t: f64| {
        Some((
            Point3::new(t.cos(), t.sin(), 0.2 * t),
            Vector3::new(-t.sin(), t.cos(), 0.2),
        ))
    };
    let c = hermite_fit(&mut f, 0.0, 12.0, 1e-10, 4, 4000).unwrap();
    let mut worst: f64 = 0.0;
    for i in 0..=2000 {
        let t = 12.0 * i as f64 / 2000.0;
        let e = c.point_at(t) - Point3::new(t.cos(), t.sin(), 0.2 * t);
        worst = worst.max(e.norm());
    }
    assert!(worst < 5e-10, "worst {worst}");
    // C1: derivative continuous at interior knots
    let d = c.derivatives(6.0, 1)[1];
    assert!((d - Vector3::new(-(6.0f64).sin(), (6.0f64).cos(), 0.2)).norm() < 1e-6);
}

#[test]
fn transforms_preserve_geometry() {
    let t = Transform3::rotation(Vector3::new(1.0, 2.0, 3.0), 0.7)
        .then(&Transform3::translation(Vector3::new(1.0, -2.0, 0.5)));
    for s in surfaces(1.3) {
        let s2 = transform_surface(&s, &t);
        let ((u0, u1), (v0, v1)) = s.domain();
        let (u, v) = (
            if u0.is_finite() { 0.5 * (u0 + u1) } else { 0.3 },
            if v0.is_finite() { 0.5 * (v0 + v1) } else { 0.4 },
        );
        let d = (s2.point_at(u, v) - t.apply_point(&s.point_at(u, v))).norm();
        assert!(d < 1e-12, "{s:?}");
    }
}

#[test]
fn offsets_are_at_constant_distance() {
    for s in surfaces(0.4) {
        let Some(o) = offset_surface(&s, 0.25) else {
            continue;
        };
        let ((u0, u1), (v0, v1)) = s.domain();
        let (u, v) = (
            if u0.is_finite() {
                u0 + 0.37 * (u1 - u0)
            } else {
                0.3
            },
            if v0.is_finite() {
                v0 + 0.41 * (v1 - v0)
            } else {
                0.4
            },
        );
        // the offset surface passes through p + d n for the same point
        let p = s.point_at(u, v);
        let q = p + s.normal_at(u, v) * 0.25;
        let r = surface_param_of(&o, &q, None);
        assert!(r.distance < 1e-9, "{s:?}: {}", r.distance);
    }
}
