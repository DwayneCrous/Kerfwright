//! Tensor-product surface interpolation: through the data, and faithful between it.
use kw_nurbs::NurbsSurface;
use nalgebra::Point3;

fn grid(n: usize, f: impl Fn(f64, f64) -> Point3<f64>) -> Vec<Vec<Point3<f64>>> {
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| f(i as f64 / (n - 1) as f64, j as f64 / (n - 1) as f64))
                .collect()
        })
        .collect()
}

#[test]
fn the_uniform_interpolant_passes_through_every_data_point() {
    let f = |u: f64, v: f64| Point3::new(3.0 * u, 2.0 * v, (4.0 * u).sin() * (3.0 * v).cos());
    let g = grid(9, f);
    let s = NurbsSurface::interpolate_uniform(&g, 3, 3).unwrap();
    for (i, row) in g.iter().enumerate() {
        for (j, want) in row.iter().enumerate() {
            let p = s.point_at(i as f64 / 8.0, j as f64 / 8.0);
            assert!((p - want).norm() < 1e-10, "({i},{j})");
        }
    }
}

#[test]
fn the_chord_length_interpolant_passes_through_the_data_too() {
    let f = |u: f64, v: f64| Point3::new(5.0 * u * u, 2.0 * v + u, u * v);
    let g = grid(7, f);
    let s = NurbsSurface::interpolate(&g, 3, 3).unwrap();
    for row in &g {
        for p in row {
            assert!(s.closest_point(p).distance < 1e-8);
        }
    }
}

#[test]
fn a_planar_grid_gives_a_plane_and_a_spherical_one_a_sphere() {
    let plane = grid(6, |u, v| {
        Point3::new(10.0 * u, 7.0 * v, 2.0 * u + 3.0 * v + 1.0)
    });
    let s = NurbsSurface::interpolate_uniform(&plane, 3, 3).unwrap();
    for (u, v) in [(0.13, 0.77), (0.5, 0.5), (0.91, 0.33)] {
        let p = s.point_at(u, v);
        assert!((p.z - (2.0 * p.x / 10.0 + 3.0 * p.y / 7.0 + 1.0)).abs() < 1e-10);
    }
    // a patch of a sphere of radius 5: longitude and latitude within 30 degrees
    let sphere = |u: f64, v: f64| {
        let (a, b) = ((u - 0.5) * 1.0, (v - 0.5) * 1.0);
        Point3::new(
            5.0 * b.cos() * a.sin(),
            5.0 * b.sin(),
            5.0 * b.cos() * a.cos(),
        )
    };
    let s = NurbsSurface::interpolate_uniform(&grid(12, sphere), 3, 3).unwrap();
    for (u, v) in [(0.07, 0.93), (0.41, 0.58), (0.77, 0.12), (0.5, 0.5)] {
        let r = s.point_at(u, v).coords.norm();
        assert!((r - 5.0).abs() < 1e-4, "radius {r}");
    }
}

#[test]
fn bad_grids_are_refused() {
    let ragged = vec![
        vec![Point3::origin(); 4],
        vec![Point3::origin(); 3],
        vec![Point3::origin(); 4],
        vec![Point3::origin(); 4],
    ];
    assert!(NurbsSurface::interpolate_uniform(&ragged, 3, 3).is_err());
    let small = grid(3, |u, v| Point3::new(u, v, 0.0));
    assert!(NurbsSurface::interpolate(&small, 3, 3).is_err());
    assert!(NurbsSurface::interpolate(&small, 0, 2).is_err());
}
