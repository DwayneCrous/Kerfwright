//! Properties of mesh bodies against closed forms.

use kw_app::design::measure::mesh_props;
use kw_mesh3d::SurfaceMesh;

#[test]
fn a_box_mesh_has_the_volume_centre_and_inertia_of_a_box() {
    let mut m = SurfaceMesh::cuboid(20.0, 30.0, 40.0);
    // far from the origin, so that the centre and the inertia have to be about the centroid
    for p in &mut m.nodes {
        p[0] += 1000.0;
        p[1] -= 500.0;
        p[2] += 25.0;
    }
    let p = mesh_props(&m);
    assert!(!p.surface);
    let v = 24000.0;
    assert!((p.volume - v).abs() < 1e-6 * v, "{}", p.volume);
    assert!((p.area - 2.0 * (600.0 + 800.0 + 1200.0)).abs() < 1e-6);
    for (got, want) in p.centroid.iter().zip([1010.0, -485.0, 45.0]) {
        assert!((got - want).abs() < 1e-6, "{:?}", p.centroid);
    }
    let want = |a: f64, b: f64| v * (a * a + b * b) / 12.0;
    assert!((p.inertia[0][0] - want(30.0, 40.0)).abs() < 1e-6 * want(30.0, 40.0));
    assert!((p.inertia[1][1] - want(20.0, 40.0)).abs() < 1e-6 * want(20.0, 40.0));
    assert!((p.inertia[2][2] - want(20.0, 30.0)).abs() < 1e-6 * want(20.0, 30.0));
    assert!(p.inertia[0][1].abs() < 1e-3 && p.inertia[1][2].abs() < 1e-3);
    assert_eq!(p.size(), [20.0, 30.0, 40.0]);
}

#[test]
fn an_open_mesh_has_an_area_and_no_volume() {
    let mut m = SurfaceMesh::cuboid(10.0, 10.0, 10.0);
    m.tris.truncate(10);
    m.tags.truncate(10);
    let p = mesh_props(&m);
    assert!(p.surface && p.volume == 0.0 && p.area > 0.0);
}
