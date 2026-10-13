//! Shape of a surface at a point: normal curvature along a direction.

use crate::geom::{Surface, SurfaceGeom};
use nalgebra::Vector3;

/// Normal curvature of `s` at `(u, v)` along the unit tangent direction `d`, with respect to the
/// surface normal times `sign` (Weingarten: minus the change of the normal along `d`, dotted with `d`).
pub fn normal_curvature(s: &SurfaceGeom, uv: (f64, f64), d: &Vector3<f64>, sign: f64) -> f64 {
    let (su, sv) = s.derivatives(uv.0, uv.1);
    // d = a su + b sv, in the least-squares sense
    let (e, f, g) = (su.dot(&su), su.dot(&sv), sv.dot(&sv));
    let det = e * g - f * f;
    if det.abs() < 1e-300 {
        return 0.0;
    }
    let (r1, r2) = (d.dot(&su), d.dot(&sv));
    let (a, b) = ((g * r1 - f * r2) / det, (e * r2 - f * r1) / det);
    // a step of h along d is a step of h (a, b) in the parameters
    let h = 1e-4 * (1.0 + s.point_at(uv.0, uv.1).coords.amax());
    let n = |t: f64| s.normal_at(uv.0 + t * h * a, uv.1 + t * h * b) * sign;
    let ((u0, u1), (v0, v1)) = s.domain();
    let inside = |t: f64| {
        let (u, v) = (uv.0 + t * h * a, uv.1 + t * h * b);
        let ok = |x: f64, lo: f64, hi: f64, periodic: bool| periodic || (x >= lo && x <= hi);
        ok(u, u0, u1, s.period_u().is_some()) && ok(v, v0, v1, s.period_v().is_some())
    };
    // central where the surface goes on either side, one-sided where it ends
    let dn = if inside(-2.0) && inside(2.0) {
        (n(1.0) - n(-1.0)) / (2.0 * h)
    } else if inside(2.0) {
        (n(0.0) * -3.0 + n(1.0) * 4.0 - n(2.0)) / (2.0 * h)
    } else {
        (n(0.0) * 3.0 - n(-1.0) * 4.0 + n(-2.0)) / (2.0 * h)
    };
    -dn.dot(d)
}
