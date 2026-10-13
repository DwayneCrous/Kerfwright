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

use kw_nurbs::{ExtendDir, KnotVector};

/// A bicubic polynomial surface on `[0, 1] x [0, 2]`, with two spans in v.
fn poly_surface() -> NurbsSurface {
    let f = |u: f64, v: f64| {
        Point3::new(
            u + 0.3 * v * v,
            v - 0.2 * u * u * v + 0.1 * v * v * v,
            u * u * u - u * v + 0.5 * v * v,
        )
    };
    let ku = KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).unwrap();
    let kv = KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0]).unwrap();
    // a polynomial of degree 3 is reproduced exactly by collocation (the C2 spline has the same span space)
    let _ = f;
    NurbsSurface::collocate(ku, kv, &f).unwrap()
}

fn f_exact(u: f64, v: f64) -> Point3<f64> {
    Point3::new(
        u + 0.3 * v * v,
        v - 0.2 * u * u * v + 0.1 * v * v * v,
        u * u * u - u * v + 0.5 * v * v,
    )
}

#[test]
fn an_extended_surface_keeps_what_it_was_and_continues_the_polynomial() {
    let s = poly_surface();
    // the collocation really is the polynomial (it has cubic pieces in v only where the data is cubic)
    for (u, v) in [(0.2, 0.4), (0.9, 1.7)] {
        assert!((s.point_at(u, v) - f_exact(u, v)).norm() < 1e-9);
    }
    for (dir, check) in [
        (ExtendDir::VEnd, (0.4, 2.0 + 0.6)),
        (ExtendDir::VStart, (0.4, -0.6)),
        (ExtendDir::UEnd, (1.5, 0.7)),
        (ExtendDir::UStart, (-0.5, 0.7)),
    ] {
        let e = s
            .extended(dir, 0.6)
            .unwrap_or_else(|er| panic!("{dir:?}: {er:?}"));
        let ((u0, u1), (v0, v1)) = e.domain();
        // the new domain is the old one plus 0.6 at the chosen end
        match dir {
            ExtendDir::VEnd => assert!((v1 - 2.6).abs() < 1e-12 && v0 == 0.0 && u1 == 1.0),
            ExtendDir::VStart => assert!((v0 + 0.6).abs() < 1e-12 && v1 == 2.0),
            ExtendDir::UEnd => assert!((u1 - 1.6).abs() < 1e-12 && u0 == 0.0),
            ExtendDir::UStart => assert!((u0 + 0.6).abs() < 1e-12 && u1 == 1.0),
        }
        // where the surface already was it is unchanged, with the same parameters
        for (u, v) in [(0.2, 0.4), (0.9, 1.7), (0.5, 1.0)] {
            assert!(
                (e.point_at(u, v) - s.point_at(u, v)).norm() < 1e-9,
                "{dir:?}"
            );
        }
        // and beyond it, it is the polynomial carried on
        let (u, v) = check;
        let u = if matches!(dir, ExtendDir::VEnd | ExtendDir::VStart) {
            0.4
        } else {
            u
        };
        let v = if matches!(dir, ExtendDir::VEnd | ExtendDir::VStart) {
            v
        } else {
            0.7
        };
        assert!(
            (e.point_at(u, v) - f_exact(u, v)).norm() < 1e-9,
            "{dir:?} at ({u}, {v})"
        );
    }
}

#[test]
fn a_rational_surface_extends_too_and_a_collapsing_weight_is_refused() {
    // a quarter cylinder: rational in u, linear in v
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let pts = vec![
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 1.0),
    ];
    let ws = vec![1.0, 1.0, h, h, 1.0, 1.0];
    let ku = KnotVector::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let kv = KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap();
    let s = NurbsSurface::new(ku, kv, pts, ws).unwrap();
    // along v (a straight generator) the extension is exact
    let e = s.extended(ExtendDir::VEnd, 2.0).unwrap();
    let p = e.point_at(0.3, 2.5);
    let q = s.point_at(0.3, 0.0);
    assert!((p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12 && (p.z - 2.5).abs() < 1e-12);
    // along u the circle's weights run out: far enough, the rational continuation breaks
    assert!(s.extended(ExtendDir::UEnd, 100.0).is_err());
}
