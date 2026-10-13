use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use std::f64::consts::PI;

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn frame() -> Frame3 {
    Frame3::from_origin_normal(Point3::new(1.0, -2.0, 0.5), Vector3::new(0.3, 0.2, 1.0))
}

#[test]
fn analytic_primitives_have_exact_mass_properties() {
    let f = frame();
    let mut rep = BRep::new();
    let b = make_box(&mut rep, &f, 2.0, 3.0, 4.0).body;
    let p = rep.mass_properties(b).unwrap();
    assert!(rel(p.volume, 24.0) < 1e-12, "{}", p.volume);
    assert!(rel(p.area, 52.0) < 1e-12);
    assert!((p.centroid - f.origin).norm() < 1e-11);
    // box inertia about centroid in its own axes: Ixx = m/12 (y²+z²)
    let i = p.inertia_centroid();
    let axes = [f.x, f.y, f.z];
    let e = [2.0_f64, 3.0, 4.0];
    for k in 0..3 {
        let a = axes[k];
        let exact = 24.0 / 12.0 * (e[(k + 1) % 3].powi(2) + e[(k + 2) % 3].powi(2));
        let got = (a.transpose() * i * a)[(0, 0)];
        assert!(rel(got, exact) < 1e-11, "axis {k}: {got} vs {exact}");
    }

    let cyl = make_cylinder(&mut rep, &f, 1.5, 2.0).body;
    let p = rep.mass_properties(cyl).unwrap();
    assert!(rel(p.volume, PI * 2.25 * 2.0) < 1e-11, "{}", p.volume);
    assert!(rel(p.area, 2.0 * PI * 1.5 * 2.0 + 2.0 * PI * 2.25) < 1e-11);
    assert!((p.centroid - (f.origin + f.z * 1.0)).norm() < 1e-10);

    let cone = make_cone(&mut rep, &f, 2.0, 0.0, 3.0).body;
    let p = rep.mass_properties(cone).unwrap();
    assert!(rel(p.volume, PI * 4.0 * 3.0 / 3.0) < 1e-11, "{}", p.volume);
    let slant = (4.0f64 + 9.0).sqrt();
    assert!(rel(p.area, PI * 2.0 * slant + PI * 4.0) < 1e-11);
    assert!((p.centroid - (f.origin + f.z * 0.75)).norm() < 1e-10);

    let fr = make_cone(&mut rep, &f, 2.0, 1.0, 3.0).body;
    let p = rep.mass_properties(fr).unwrap();
    let vol = PI * 3.0 / 3.0 * (4.0 + 2.0 + 1.0);
    assert!(rel(p.volume, vol) < 1e-11, "{} vs {vol}", p.volume);

    let sph = make_sphere(&mut rep, &f, 1.7).body;
    let p = rep.mass_properties(sph).unwrap();
    assert!(
        rel(p.volume, 4.0 / 3.0 * PI * 1.7f64.powi(3)) < 1e-11,
        "{}",
        p.volume
    );
    assert!(rel(p.area, 4.0 * PI * 1.7 * 1.7) < 1e-11);
    assert!(
        (p.centroid - f.origin).norm() < 1e-10,
        "sphere centroid off by {:e}",
        (p.centroid - f.origin).norm()
    );

    let tor = make_torus(&mut rep, &f, 3.0, 0.8).body;
    let p = rep.mass_properties(tor).unwrap();
    assert!(
        rel(p.volume, 2.0 * PI * PI * 3.0 * 0.64) < 1e-10,
        "{}",
        p.volume
    );
    assert!(rel(p.area, 4.0 * PI * PI * 3.0 * 0.8) < 1e-10);
}
