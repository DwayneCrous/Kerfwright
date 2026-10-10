//! Surface–surface intersection.
//!
//! * exact closed forms for plane/quadric/torus pairs (lines, circles, ellipses),
//! * certified level-set tracing for any parametric surface against an analytic one,
//! * every curve comes with pcurves on both surfaces sharing its parameter.
//!
//! The functions never panic on degenerate input: unsupported or failed cases return
//! [`SsiError`].

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

pub mod exact;
pub mod field;
pub mod implicit;
mod trace;

use implicit::Implicit;
use kw_nurbs::geom::*;
use kw_nurbs::pcurve::pcurve_on;
use nalgebra::Point3;
use std::f64::consts::FRAC_PI_2;

/// Finite parameter window of a surface (required for unbounded directions; for a periodic
/// direction a window of at least one period means "the whole period").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Window {
    pub u: (f64, f64),
    pub v: (f64, f64),
}

#[derive(Debug, Clone)]
pub struct SsiCurve {
    pub curve: CurveGeom,
    pub range: (f64, f64),
    /// pcurve on the first surface (continuous in `u, v`; periodic coordinates are unwrapped).
    pub pcurve_a: CurveGeom,
    pub pcurve_b: CurveGeom,
    /// The 3D curve is closed (start point equals end point).
    pub closed: bool,
}

#[derive(Debug, Clone)]
pub enum SsiOutcome {
    Empty,
    Curves {
        curves: Vec<SsiCurve>,
        /// The surfaces touch tangentially along these curves.
        tangent: bool,
    },
    /// Isolated tangent contact points.
    Points(Vec<Point3<f64>>),
    /// The surfaces are the same surface; `same_normal` tells whether the normals agree.
    Coincident {
        same_normal: bool,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum SsiError {
    Unsupported(&'static str),
    Failed(&'static str),
}

impl std::fmt::Display for SsiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SsiError::Unsupported(s) => write!(f, "unsupported: {s}"),
            SsiError::Failed(s) => write!(f, "failed: {s}"),
        }
    }
}
impl std::error::Error for SsiError {}

/// Default fitting / pcurve tolerance (model units, scaled by `1 + |x|` internally).
pub const DEFAULT_TOL: f64 = 1e-9;

/// Interval of a line parameter for which the line stays inside the window on a plane or cylinder.
/// `None`: the surface type has no closed form here; `Some(None)`: the line misses the window.
fn line_interval(s: &SurfaceGeom, w: Window, l: &Line) -> Option<Option<(f64, f64)>> {
    let mut lo = f64::NEG_INFINITY;
    let mut hi = f64::INFINITY;
    let mut clamp = |p0: f64, dp: f64, a: f64, b: f64| {
        // a ≤ p0 + dp t ≤ b
        if dp.abs() < 1e-14 {
            if p0 < a - 1e-9 || p0 > b + 1e-9 {
                lo = f64::INFINITY;
            }
            return;
        }
        let (t0, t1) = ((a - p0) / dp, (b - p0) / dp);
        lo = lo.max(t0.min(t1));
        hi = hi.min(t0.max(t1));
    };
    match s {
        SurfaceGeom::Plane(p) => {
            let q = l.origin - p.frame.origin;
            clamp(q.dot(&p.frame.x), l.dir.dot(&p.frame.x), w.u.0, w.u.1);
            clamp(q.dot(&p.frame.y), l.dir.dot(&p.frame.y), w.v.0, w.v.1);
        }
        SurfaceGeom::Cylinder(c) => {
            let q = l.origin - c.frame.origin;
            clamp(q.dot(&c.frame.z), l.dir.dot(&c.frame.z), w.v.0, w.v.1);
        }
        SurfaceGeom::Cone(c) => {
            // the cone's `v` is the axial coordinate
            let q = l.origin - c.frame.origin;
            clamp(q.dot(&c.frame.z), l.dir.dot(&c.frame.z), w.v.0, w.v.1);
        }
        _ => return None,
    }
    Some((lo.is_finite() && hi.is_finite() && lo <= hi).then_some((lo, hi)))
}

/// Does the pcurve visit the window (only non-periodic directions can exclude it)? The curve is
/// sampled finely relative to the window so thin crossings of huge curves are not missed.
fn touches_window(s: &SurfaceGeom, w: Window, pc: &CurveGeom, range: (f64, f64)) -> bool {
    let (pu, pv) = (s.period_u().is_some(), s.period_v().is_some());
    let eps = 1e-9;
    let inside = |q: Point3<f64>| {
        (pu || (q.x >= w.u.0 - eps && q.x <= w.u.1 + eps))
            && (pv || (q.y >= w.v.0 - eps && q.y <= w.v.1 + eps))
    };
    let wsize = ((w.u.1 - w.u.0)
        .abs()
        .min(if pu { f64::INFINITY } else { 1e300 }))
    .min(
        (w.v.1 - w.v.0)
            .abs()
            .min(if pv { f64::INFINITY } else { 1e300 }),
    )
    .max(1e-9);
    let coarse = 256usize;
    let mut len = 0.0;
    let mut prev = pc.point_at(range.0);
    for i in 1..=coarse {
        let q = pc.point_at(range.0 + (range.1 - range.0) * i as f64 / coarse as f64);
        len += (q - prev).norm();
        prev = q;
    }
    let n = ((len / (wsize / 40.0)) as usize).clamp(128, 400_000);
    (0..=n).any(|i| inside(pc.point_at(range.0 + (range.1 - range.0) * i as f64 / n as f64)))
}

fn build_exact(
    a: &SurfaceGeom,
    wa: Window,
    b: &SurfaceGeom,
    wb: Window,
    ex: exact::Exact,
    tol: f64,
) -> Result<SsiOutcome, SsiError> {
    if let Some(same_normal) = ex.coincident {
        return Ok(SsiOutcome::Coincident { same_normal });
    }
    if ex.curves.is_empty() {
        return Ok(if ex.points.is_empty() {
            SsiOutcome::Empty
        } else {
            SsiOutcome::Points(ex.points)
        });
    }
    let mut curves = Vec::new();
    for rc in ex.curves {
        let mut range = rc.range;
        if let CurveGeom::Line(l) = &rc.curve {
            let ia = line_interval(a, wa, l);
            let ib = line_interval(b, wb, l);
            // an infinite line must be bounded by at least one window it understands
            let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
            let mut empty = false;
            let mut bounded = false;
            for i in [ia, ib].into_iter().flatten() {
                match i {
                    Some(r) => {
                        bounded = true;
                        lo = lo.max(r.0);
                        hi = hi.min(r.1);
                    }
                    None => empty = true,
                }
            }
            if empty {
                continue;
            }
            if !bounded || !(lo.is_finite() && hi.is_finite()) {
                return Err(SsiError::Unsupported(
                    "cannot bound an intersection line by the windows",
                ));
            }
            if lo > hi {
                continue;
            }
            range = (lo, hi);
        }
        let ha = Some((0.5 * (wa.u.0 + wa.u.1), 0.5 * (wa.v.0 + wa.v.1)));
        let hb = Some((0.5 * (wb.u.0 + wb.u.1), 0.5 * (wb.v.0 + wb.v.1)));
        let scale = 1.0 + rc.curve.point_at(0.5 * (range.0 + range.1)).coords.amax();
        let pa = pcurve_on(a, &rc.curve, range, ha, tol * scale)
            .ok_or(SsiError::Failed("pcurve on first surface"))?;
        let pb = pcurve_on(b, &rc.curve, range, hb, tol * scale)
            .ok_or(SsiError::Failed("pcurve on second surface"))?;
        if !rc.closed || (touches_window(a, wa, &pa, range) && touches_window(b, wb, &pb, range)) {
            curves.push(SsiCurve {
                curve: rc.curve,
                range,
                pcurve_a: pa,
                pcurve_b: pb,
                closed: rc.closed,
            });
        }
    }
    if curves.is_empty() {
        return Ok(SsiOutcome::Empty);
    }
    Ok(SsiOutcome::Curves {
        curves,
        tangent: ex.tangent,
    })
}

/// Diameter of the image of a window (sampled).
fn window_extent(s: &SurfaceGeom, w: Window) -> f64 {
    let mut pts = Vec::new();
    for i in 0..=4 {
        for j in 0..=4 {
            let u = w.u.0 + (w.u.1 - w.u.0) * i as f64 / 4.0;
            let v = w.v.0 + (w.v.1 - w.v.0) * j as f64 / 4.0;
            pts.push(s.point_at(u, v));
        }
    }
    let bb = kw_math::Aabb::from_points(&pts);
    bb.diagonal().max(1e-9)
}

/// A spline surface that lies on one side of a plane and touches it along a whole boundary iso-curve of
/// its window (the typical contact of a blend or loft face with the face it is tangent to). Tracing
/// cannot follow such a contact (the level set is singular along it), so the iso-curves are returned
/// directly. `None` when there is no such contact or the surface also crosses or lies in the plane.
fn analytic_edge_contact(
    s: &SurfaceGeom,
    w: Window,
    other: &SurfaceGeom,
    tol: f64,
) -> Option<Vec<SsiCurve>> {
    if !matches!(s, SurfaceGeom::Nurbs(_)) {
        return None;
    }
    let imp = Implicit::of(other)?;
    let (u0, u1) = w.u;
    let (v0, v1) = w.v;
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite())
        || u1 <= u0
        || v1 <= v0
    {
        return None;
    }
    let scale = 1.0 + window_extent(s, w);
    let eps = 1e-8 * scale;
    // signed distance (or its analytic stand-in) of the spline surface from the other surface
    let dist = |u: f64, v: f64| imp.eval(&s.point_at(u, v)).0;
    // one side only (interior samples), and not lying in the plane
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    const N: usize = 12;
    for i in 0..=N {
        for j in 0..=N {
            let d = dist(
                u0 + (u1 - u0) * i as f64 / N as f64,
                v0 + (v1 - v0) * j as f64 / N as f64,
            );
            lo = lo.min(d);
            hi = hi.max(d);
        }
    }
    if lo < -eps && hi > eps {
        return None;
    }
    if hi - lo < 100.0 * eps {
        return None;
    }
    let mut out = Vec::new();
    // (fixed u?, fixed parameter value, running range)
    let isos = [
        (true, u0, (v0, v1)),
        (true, u1, (v0, v1)),
        (false, v0, (u0, u1)),
        (false, v1, (u0, u1)),
    ];
    for (fix_u, c, range) in isos {
        let at = |t: f64| if fix_u { (c, t) } else { (t, c) };
        let mut dev: f64 = 0.0;
        let on_plane = (0..=32).all(|k| {
            let t = range.0 + (range.1 - range.0) * k as f64 / 32.0;
            let (u, v) = at(t);
            let d = dist(u, v).abs();
            dev = dev.max(d);
            d <= eps
        });
        if !on_plane {
            continue;
        }
        let mut f = |t: f64| {
            let (u, v) = at(t);
            let (su, sv) = s.derivatives(u, v);
            Some((s.point_at(u, v), if fix_u { sv } else { su }))
        };
        let curve =
            kw_nurbs::fit::hermite_fit(&mut f, range.0, range.1, tol.max(1e-10) * scale, 4, 4000)?;
        let curve = CurveGeom::Nurbs(curve);
        let pcurve_a = if fix_u {
            CurveGeom::Line(Line {
                origin: Point3::new(c, 0.0, 0.0),
                dir: nalgebra::Vector3::new(0.0, 1.0, 0.0),
            })
        } else {
            CurveGeom::Line(Line {
                origin: Point3::new(0.0, c, 0.0),
                dir: nalgebra::Vector3::new(1.0, 0.0, 0.0),
            })
        };
        let pcurve_b = pcurve_on(
            other,
            &curve,
            range,
            None,
            (tol.max(1e-9) * scale).max(3.0 * dev),
        )?;
        out.push(SsiCurve {
            curve,
            range,
            pcurve_a,
            pcurve_b,
            closed: false,
        });
    }
    (!out.is_empty()).then_some(out)
}

/// A cylinder that is the straight continuation of a torus tube: equal radii, its axis tangent to the
/// torus's major circle (in the mid-plane). The surfaces touch along the tube's circle there. This is the
/// junction of a fillet along a line and the fillet around a tangent arc, glued at a common cap. Tracing
/// cannot resolve such a contact, so the circle is returned directly when it is the whole intersection
/// within the windows. `None` unless the geometry and the windows fit.
fn tube_contact(
    a: &SurfaceGeom,
    wa: Window,
    b: &SurfaceGeom,
    wb: Window,
    tol: f64,
) -> Option<Result<SsiOutcome, SsiError>> {
    let (cyl, tor) = match (a, b) {
        (SurfaceGeom::Cylinder(c), SurfaceGeom::Torus(t)) => (c, t),
        (SurfaceGeom::Torus(t), SurfaceGeom::Cylinder(c)) => (c, t),
        _ => return None,
    };
    let scale = 1.0
        + cyl
            .frame
            .origin
            .coords
            .amax()
            .max(tor.frame.origin.coords.amax())
            .max(tor.major_radius);
    let eps = 1e-9 * scale;
    if (cyl.radius - tor.minor_radius).abs() > eps {
        return None;
    }
    let t = cyl.frame.z;
    if t.dot(&tor.frame.z).abs() > 1e-9 {
        return None;
    }
    let c = tor.frame.origin;
    let foot = cyl.frame.origin + t * (c - cyl.frame.origin).dot(&t);
    if ((foot - c).norm() - tor.major_radius).abs() > eps
        || (foot - c).dot(&tor.frame.z).abs() > eps
    {
        return None;
    }
    // Besides the circle the full surfaces cross along two branches on the inner half of the tube (where
    // cos ψ < 0, at axial offset s = ±2 √(R ρ |cos ψ|), growing out of the circle's top and bottom
    // points). If none of them runs through the interior of both windows (shrunk by the widening the
    // caller applied to its face extents), the circle is the whole intersection.
    let radial = (foot - c) / tor.major_radius;
    let up = tor.frame.z;
    let shrunk = |s: &SurfaceGeom, w: Window| -> Option<Window> {
        if !(w.u.0.is_finite() && w.u.1.is_finite() && w.v.0.is_finite() && w.v.1.is_finite()) {
            return None;
        }
        let sh = |lo: f64, hi: f64, per: Option<f64>| match per {
            Some(p) if hi - lo >= p - 1e-9 => (lo, hi),
            _ => {
                let m = 0.0455 * (hi - lo);
                (lo + m, hi - m)
            }
        };
        Some(Window {
            u: sh(w.u.0, w.u.1, s.period_u()),
            v: sh(w.v.0, w.v.1, s.period_v()),
        })
    };
    let inside = |s: &SurfaceGeom, w: &Window, p: &Point3<f64>| -> bool {
        let q = kw_nurbs::proj::surface_param_of(s, p, None);
        let within = |x: f64, lo: f64, hi: f64, per: Option<f64>| match per {
            Some(per) => {
                let x0 = lo + (x - lo).rem_euclid(per);
                x0 <= hi || hi - lo >= per
            }
            None => x >= lo && x <= hi,
        };
        within(q.u, w.u.0, w.u.1, s.period_u()) && within(q.v, w.v.0, w.v.1, s.period_v())
    };
    let (core_a, core_b) = (shrunk(a, wa)?, shrunk(b, wb)?);
    let n = 720;
    for k in 0..=n {
        let psi = FRAC_PI_2 + std::f64::consts::PI * k as f64 / n as f64;
        let off = (radial * psi.cos() + up * psi.sin()) * cyl.radius;
        let s = 2.0 * (tor.major_radius * cyl.radius * (-psi.cos()).max(0.0)).sqrt();
        for sgn in [1.0, -1.0] {
            let p = foot + t * (sgn * s) + off;
            if inside(a, &core_a, &p) && inside(b, &core_b, &p) {
                return None;
            }
        }
    }
    let ex = exact::Exact {
        curves: vec![exact::circle_curve(foot, t, cyl.radius)],
        tangent: true,
        ..Default::default()
    };
    Some(build_exact(a, wa, b, wb, ex, tol))
}

/// Intersect two surfaces within the given parameter windows.
pub fn intersect(
    a: &SurfaceGeom,
    wa: Window,
    b: &SurfaceGeom,
    wb: Window,
    tol: f64,
) -> Result<SsiOutcome, SsiError> {
    if let Some(ex) = exact::exact(a, b) {
        // a closed curve vastly larger than the windows (nearly parallel plane/cylinder) would miss
        // them under sampling; trace those inside the windows instead
        let huge = ex.curves.iter().any(|c| match &c.curve {
            CurveGeom::Ellipse(e) => {
                e.major > 25.0 * window_extent(a, wa).min(window_extent(b, wb))
            }
            CurveGeom::Circle(ci) => {
                ci.radius > 25.0 * window_extent(a, wa).min(window_extent(b, wb))
            }
            _ => false,
        });
        if !huge {
            return build_exact(a, wa, b, wb, ex, tol);
        }
    }
    if let Some(r) = tube_contact(a, wa, b, wb, tol) {
        return r;
    }
    // a spline face tangent to a plane along its boundary
    match (a, b) {
        (SurfaceGeom::Nurbs(_), other) if Implicit::of(other).is_some() => {
            if let Some(curves) = analytic_edge_contact(a, wa, other, tol) {
                return Ok(SsiOutcome::Curves {
                    curves,
                    tangent: true,
                });
            }
        }
        (other, SurfaceGeom::Nurbs(_)) if Implicit::of(other).is_some() => {
            if let Some(curves) = analytic_edge_contact(b, wb, other, tol) {
                let curves = curves
                    .into_iter()
                    .map(|c| SsiCurve {
                        pcurve_a: c.pcurve_b,
                        pcurve_b: c.pcurve_a,
                        ..c
                    })
                    .collect();
                return Ok(SsiOutcome::Curves {
                    curves,
                    tangent: true,
                });
            }
        }
        _ => {}
    }
    // level-set tracing: the surface traced over its parameter grid (A) against an implicit one (B): a
    // closed form for the analytic surfaces, a signed-distance field for the rest
    let (ia, ib) = (Implicit::of(a), Implicit::of(b));
    let (mut traced, swapped) = match (&ia, &ib) {
        (None, None) => {
            // both parametric: trace the smaller window over its grid against the other's field
            let a_smaller = window_extent(a, wa) <= window_extent(b, wb);
            let (pa_, wpa, pb_, wpb) = if a_smaller {
                (a, wa, b, wb)
            } else {
                (b, wb, a, wa)
            };
            let field = Implicit::field(pb_, wpb).ok_or(SsiError::Unsupported(
                "surface has no finite parameter window",
            ))?;
            (trace::trace(pa_, wpa, pb_, wpb, &field, tol), !a_smaller)
        }
        _ => {
            let prefer_b_implicit = match (&ia, &ib) {
                (Some(_), Some(_)) => {
                    !matches!(b, SurfaceGeom::Plane(_)) || matches!(a, SurfaceGeom::Plane(_))
                }
                (None, Some(_)) => true,
                _ => false,
            };
            // when both are analytic and b is chosen implicit, a is parametric (and vice versa)
            if prefer_b_implicit {
                (trace::trace(a, wa, b, wb, ib.as_ref().unwrap(), tol), false)
            } else {
                (trace::trace(b, wb, a, wa, ia.as_ref().unwrap(), tol), true)
            }
        }
    };
    // A curve that grazes the edge of the traced window can defeat the refinement; the window is only a
    // bound the caller chose, so retry once on a wider one (the caller clips the curves anyway).
    if matches!(traced, Err(SsiError::Failed(_))) {
        let widen = |s: &SurfaceGeom, w: Window| -> Window {
            let grow = |lo: f64, hi: f64, per: Option<f64>| -> (f64, f64) {
                if per.is_some_and(|p| hi - lo >= p - 1e-9) || !lo.is_finite() || !hi.is_finite() {
                    (lo, hi)
                } else {
                    let m = 0.25 * (hi - lo).abs().max(1e-6);
                    (lo - m, hi + m)
                }
            };
            Window {
                u: grow(w.u.0, w.u.1, s.period_u()),
                v: grow(w.v.0, w.v.1, s.period_v()),
            }
        };
        let retry = match (&ia, &ib) {
            (None, None) => None,
            _ => {
                let prefer_b_implicit = match (&ia, &ib) {
                    (Some(_), Some(_)) => {
                        !matches!(b, SurfaceGeom::Plane(_)) || matches!(a, SurfaceGeom::Plane(_))
                    }
                    (None, Some(_)) => true,
                    _ => false,
                };
                if prefer_b_implicit {
                    Some(trace::trace(
                        a,
                        widen(a, wa),
                        b,
                        wb,
                        ib.as_ref().unwrap(),
                        tol,
                    ))
                } else {
                    Some(trace::trace(
                        b,
                        widen(b, wb),
                        a,
                        wa,
                        ia.as_ref().unwrap(),
                        tol,
                    ))
                }
            }
        };
        if let Some(Ok(t)) = retry {
            traced = Ok(t);
        }
    }
    if std::env::var("KW_SSI_DEBUG").is_ok() {
        if let Err(e) = &traced {
            eprintln!("SSI FAILED ({e}):\n  A {a:?}\n  wa {wa:?}\n  B {b:?}\n  wb {wb:?}");
        }
    }
    let traced = traced?;
    if traced.is_empty() {
        return Ok(SsiOutcome::Empty);
    }
    let curves = traced
        .into_iter()
        .map(|t| {
            let (pa, pb) = if swapped {
                (t.pcurve_b, t.pcurve_a)
            } else {
                (t.pcurve_a, t.pcurve_b)
            };
            SsiCurve {
                curve: CurveGeom::Nurbs(t.curve),
                range: t.range,
                pcurve_a: CurveGeom::Nurbs(pa),
                pcurve_b: CurveGeom::Nurbs(pb),
                closed: t.closed,
            }
        })
        .collect();
    Ok(SsiOutcome::Curves {
        curves,
        tangent: false,
    })
}
