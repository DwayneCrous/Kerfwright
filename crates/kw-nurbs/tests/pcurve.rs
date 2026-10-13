use kw_math::Frame3;
use kw_nurbs::geom::*;
use kw_nurbs::pcurve::*;
use nalgebra::{Point3, Vector3};

fn f0() -> Frame3 {
    Frame3::from_origin_normal(Point3::new(0.5, -1.0, 2.0), Vector3::new(0.3, 0.2, 1.0))
}

fn check(s: &SurfaceGeom, c: &CurveGeom, range: (f64, f64), hint: Option<(f64, f64)>) -> CurveGeom {
    let pc = pcurve_on(s, c, range, hint, 1e-9)
        .unwrap_or_else(|| panic!("no pcurve for {c:?} on {s:?}"));
    let err = pcurve_error(s, c, &pc, range, 400);
    assert!(err < 1e-9, "error {err} for {c:?} on {s:?}");
    // continuity (no jump in uv): step between consecutive samples is small
    let mut prev = pc.point_at(range.0);
    for i in 1..=800 {
        let t = range.0 + (range.1 - range.0) * i as f64 / 800.0;
        let q = pc.point_at(t);
        assert!(
            (q - prev).norm() < 0.2,
            "uv jump at t={t}: {prev:?} -> {q:?}"
        );
        prev = q;
    }
    pc
}

#[test]
fn circle_on_cylinder_is_a_line() {
    let f = f0();
    let cyl = SurfaceGeom::Cylinder(Cylinder {
        frame: f,
        radius: 2.0,
    });
    let mut cf = f;
    cf.origin += f.z * 1.5;
    let c = CurveGeom::Circle(Circle {
        frame: cf,
        radius: 2.0,
    });
    let pc = check(&cyl, &c, (0.0, std::f64::consts::TAU), None);
    assert!(
        matches!(pc, CurveGeom::Line(_)),
        "expected an exact line pcurve"
    );
    // reversed circle
    let cr = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: cf.origin,
            x: f.x,
            y: -f.y,
            z: -f.z,
        },
        radius: 2.0,
    });
    check(&cyl, &cr, (0.0, std::f64::consts::TAU), None);
}

#[test]
fn oblique_plane_section_of_a_cylinder() {
    let f = f0();
    let cyl = SurfaceGeom::Cylinder(Cylinder {
        frame: f,
        radius: 2.0,
    });
    // ellipse = plane through origin tilted about f.y by 30 degrees
    let a = std::f64::consts::FRAC_PI_6;
    let ex = f.x * a.cos() + f.z * a.sin();
    let frame = Frame3 {
        origin: f.origin + f.z * 0.7,
        x: ex,
        y: f.y,
        z: ex.cross(&f.y),
    };
    let e = CurveGeom::Ellipse(Ellipse {
        frame,
        major: 2.0 / a.cos(),
        minor: 2.0,
    });
    check(&cyl, &e, (0.0, std::f64::consts::TAU), None);
    check(&cyl, &e, (0.0, std::f64::consts::TAU), Some((20.0, 0.0)));
}

#[test]
fn curves_on_plane_sphere_torus_cone() {
    let f = f0();
    let plane = SurfaceGeom::Plane(Plane { frame: f });
    let circ = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin + f.x * 0.3,
            ..f
        },
        radius: 1.2,
    });
    let pc = check(&plane, &circ, (0.0, std::f64::consts::TAU), None);
    assert!(matches!(pc, CurveGeom::Circle(_)));

    let sph = SurfaceGeom::Sphere(Sphere {
        frame: f,
        radius: 2.0,
    });
    // latitude circle at height 1
    let r = (4.0_f64 - 1.0).sqrt();
    let lat = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin + f.z,
            ..f
        },
        radius: r,
    });
    let pc = check(&sph, &lat, (0.0, std::f64::consts::TAU), None);
    assert!(matches!(pc, CurveGeom::Line(_)));
    // meridian circle through the poles (generic hermite path)
    let mer = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin,
            x: f.x,
            y: f.z,
            z: -f.y,
        },
        radius: 2.0,
    });
    check(&sph, &mer, (0.2, std::f64::consts::FRAC_PI_2 - 0.2), None); // stops short of the pole: u jumps by π there

    // meridian circle of a torus (v loop) and a parallel
    let tor = SurfaceGeom::Torus(Torus {
        frame: f,
        major_radius: 5.0,
        minor_radius: 1.0,
    });
    let u0 = 0.8_f64;
    let dir = f.x * u0.cos() + f.y * u0.sin();
    let tv = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin + dir * 5.0,
            x: dir,
            y: f.z,
            z: dir.cross(&f.z),
        },
        radius: 1.0,
    });
    check(&tor, &tv, (0.0, std::f64::consts::TAU), None);
    let tu = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin + f.z * 1.0,
            ..f
        },
        radius: 5.0,
    });
    // not on the torus (z=1 -> outer radius): use z = 0.6 and radius 5 + sqrt(1-0.36)
    let _ = tu;
    let r = 5.0 + (1.0_f64 - 0.36).sqrt();
    let tu = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin + f.z * 0.6,
            ..f
        },
        radius: r,
    });
    check(&tor, &tu, (0.0, std::f64::consts::TAU), None);

    // cone: a circle about the axis
    let cone = SurfaceGeom::Cone(Cone {
        frame: f,
        radius_at_origin: 2.0,
        half_angle: 0.3,
    });
    let h = 0.9_f64;
    let cc = CurveGeom::Circle(Circle {
        frame: Frame3 {
            origin: f.origin + f.z * h,
            ..f
        },
        radius: 2.0 + h * 0.3_f64.tan(),
    });
    check(&cone, &cc, (0.0, std::f64::consts::TAU), None);
}
