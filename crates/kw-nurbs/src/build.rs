//! Exact rational constructions: circular arcs and surfaces of revolution
//! (Piegl & Tiller A7.1, A8.1).

use crate::curve::{NurbsCurve, NurbsError};
use crate::knots::KnotVector;
use crate::surface::NurbsSurface;
use kw_math::Frame3;
use nalgebra::{Point3, Vector3};

/// Rational quadratic arc of `theta` radians (0 < theta ≤ 2π) of a circle
/// centred at `frame.origin` in the frame's xy-plane, starting at angle
/// `start` from `frame.x`. Parameter domain is [0, 1].
pub fn make_arc(
    frame: &Frame3,
    radius: f64,
    start: f64,
    theta: f64,
) -> Result<NurbsCurve, NurbsError> {
    if theta.is_nan()
        || theta <= 0.0
        || theta > std::f64::consts::TAU + 1e-12
        || radius.is_nan()
        || radius <= 0.0
    {
        return Err(NurbsError::InvalidArgument(
            "arc needs radius > 0 and 0 < theta <= 2pi",
        ));
    }
    let narcs = if theta <= std::f64::consts::FRAC_PI_2 + 1e-12 {
        1
    } else if theta <= std::f64::consts::PI + 1e-12 {
        2
    } else if theta <= 1.5 * std::f64::consts::PI + 1e-12 {
        3
    } else {
        4
    };
    let dtheta = theta / narcs as f64;
    let wm = (dtheta / 2.0).cos();
    let dir = |a: f64| frame.x * a.cos() + frame.y * a.sin();
    let tan = |a: f64| frame.x * (-a.sin()) + frame.y * a.cos();

    let mut pts: Vec<Point3<f64>> = Vec::with_capacity(2 * narcs + 1);
    let mut w: Vec<f64> = Vec::with_capacity(2 * narcs + 1);
    pts.push(frame.origin + dir(start) * radius);
    w.push(1.0);
    let mut angle = start;
    for _ in 0..narcs {
        let p0 = *pts.last().unwrap();
        let t0 = tan(angle);
        angle += dtheta;
        let p2 = frame.origin + dir(angle) * radius;
        // tangent lines at both ends meet at distance r·tan(dθ/2) from p0
        let p1 = p0 + t0 * (radius * (dtheta / 2.0).tan());
        pts.push(p1);
        w.push(wm);
        pts.push(p2);
        w.push(1.0);
    }
    let mut k = vec![0.0; 3];
    for i in 1..narcs {
        let v = i as f64 / narcs as f64;
        k.push(v);
        k.push(v);
    }
    k.extend([1.0, 1.0, 1.0]);
    NurbsCurve::new(
        KnotVector::new(2, k).map_err(|_| NurbsError::InvalidArgument("knots"))?,
        pts,
        w,
    )
}

/// Full circle as a rational quadratic NURBS (9 control points, 4 spans).
pub fn make_circle(frame: &Frame3, radius: f64) -> Result<NurbsCurve, NurbsError> {
    make_arc(frame, radius, 0.0, std::f64::consts::TAU)
}

/// Surface of revolution of `generatrix` about the axis through `axis_origin`
/// with unit direction `axis_dir`, sweeping angle `theta` (0 < theta ≤ 2π).
/// `u` follows the generatrix and `v` the rotation angle (right-handed about
/// the axis, starting at the generatrix position), matching
/// `SurfaceGeom::Revolution`.
pub fn make_revolved_surface(
    generatrix: &NurbsCurve,
    axis_origin: Point3<f64>,
    axis_dir: Vector3<f64>,
    theta: f64,
) -> Result<NurbsSurface, NurbsError> {
    let t = axis_dir.normalize();
    // unit circle in a canonical frame: x-axis start, y-axis ccw
    let unit = make_arc(&Frame3::world(), 1.0, 0.0, theta)?;
    let circ_pts = unit.points();
    let circ_w = unit.weights();
    let nv = circ_pts.len();
    let mut pts = Vec::with_capacity(generatrix.points().len() * nv);
    let mut weights = Vec::with_capacity(pts.capacity());
    for (pj, wj) in generatrix.points().iter().zip(generatrix.weights()) {
        let o = axis_origin + t * (pj - axis_origin).dot(&t);
        let xv = pj - o;
        let r = xv.norm();
        for (c, wc) in circ_pts.iter().zip(circ_w) {
            if r < 1e-300 {
                pts.push(o);
            } else {
                let x = xv / r;
                let y = t.cross(&x);
                pts.push(o + (x * c.x + y * c.y) * r);
            }
            weights.push(wj * wc);
        }
    }
    NurbsSurface::new(
        generatrix.knots().clone(),
        unit.knots().clone(),
        pts,
        weights,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arcs_are_exact_circles() {
        let f =
            Frame3::from_origin_normal(Point3::new(1.0, -2.0, 0.5), Vector3::new(0.3, 0.2, 1.0));
        for &theta in &[0.4, 1.5, 2.9, 4.0, 5.5, std::f64::consts::TAU] {
            let a = make_arc(&f, 2.5, 0.7, theta).unwrap();
            for i in 0..=50 {
                let p = a.point_at(i as f64 / 50.0);
                assert!(((p - f.origin).norm() - 2.5).abs() < 1e-13, "theta={theta}");
                assert!((p - f.origin).dot(&f.z).abs() < 1e-13);
            }
            let end = a.point_at(1.0) - f.origin;
            let ang = 0.7 + theta;
            let want = (f.x * ang.cos() + f.y * ang.sin()) * 2.5;
            assert!((end - want).norm() < 1e-13);
        }
    }

    #[test]
    fn arc_is_parameterised_symmetrically() {
        // The middle of a half circle arc (knot 0.5) is the angle midpoint.
        let a = make_arc(&Frame3::world(), 1.0, 0.0, std::f64::consts::PI).unwrap();
        let p = a.point_at(0.5);
        assert!((p - Point3::new(0.0, 1.0, 0.0)).norm() < 1e-14);
    }

    #[test]
    fn revolved_semicircle_is_a_sphere() {
        let gen = make_arc(
            &Frame3 {
                origin: Point3::origin(),
                x: Vector3::x(),
                y: Vector3::z(),
                z: -Vector3::y(),
            },
            3.0,
            -std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
        )
        .unwrap();
        let s = make_revolved_surface(&gen, Point3::origin(), Vector3::z(), std::f64::consts::TAU)
            .unwrap();
        for i in 0..=20 {
            for j in 0..=40 {
                let p = s.point_at(i as f64 / 20.0, j as f64 / 40.0);
                assert!((p.coords.norm() - 3.0).abs() < 1e-12);
            }
        }
        // normals are radial
        let n = s.normal_at(0.3, 0.2).unwrap();
        let p = s.point_at(0.3, 0.2);
        assert!((n.cross(&p.coords)).norm() < 1e-9);
    }
}
