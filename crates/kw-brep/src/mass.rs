//! Exact (quadrature) mass properties of solid bodies.
//!
//! For a vector field `F`, `∮ F·n dA = Σ_loops ∮ (∫_{u*}^{u} F(S)·(S_u × S_v) ds) dv` by Green's
//! theorem on each face's parameter-space loops; the face/loop orientation conventions make the
//! sum independent of whether the face is `Reversed` (see derivation in `docs/phase3-report.md`).
//! The integrals are composite Gauss–Legendre, exponentially convergent for the smooth analytic
//! surfaces.

use crate::ids::*;
use crate::store::BRep;
use crate::topology::*;
use kw_math::{Matrix3, Point3, Vector3};
use kw_nurbs::{Curve3, Surface, SurfaceGeom};

const GL: [(f64, f64); 8] = [
    (-0.960_289_856_497_536_3, 0.101_228_536_290_376_26),
    (-0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (-0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (-0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (0.960_289_856_497_536_3, 0.101_228_536_290_376_26),
];

fn gauss(a: f64, b: f64, f: &mut dyn FnMut(f64) -> f64) -> f64 {
    let (m, h) = (0.5 * (a + b), 0.5 * (b - a));
    GL.iter().map(|(x, w)| w * f(m + h * x)).sum::<f64>() * h
}

/// `∫_{a}^{b}` with enough panels to resolve `scale`-sized features, split at `breaks`
/// (parameter values where the integrand is only piecewise smooth, e.g. NURBS knots).
fn gauss_composite(
    a: f64,
    b: f64,
    scale: f64,
    breaks: &[f64],
    f: &mut dyn FnMut(f64) -> f64,
) -> f64 {
    let (lo, hi) = (a.min(b), a.max(b));
    let mut stops: Vec<f64> = vec![a];
    let mut inner: Vec<f64> = breaks
        .iter()
        .copied()
        .filter(|k| *k > lo && *k < hi)
        .collect();
    inner.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    if a > b {
        inner.reverse();
    }
    stops.extend(inner);
    stops.push(b);
    let mut s = 0.0;
    for w in stops.windows(2) {
        let (p, q) = (w[0], w[1]);
        let n = (((q - p).abs() / scale).ceil() as usize).clamp(1, 64);
        for i in 0..n {
            let (x0, x1) = (
                p + (q - p) * i as f64 / n as f64,
                p + (q - p) * (i + 1) as f64 / n as f64,
            );
            s += gauss(x0, x1, f);
        }
    }
    s
}

/// Interior `u`-knots of the surface (where integrands along `u` are only piecewise smooth).
fn u_breaks(s: &SurfaceGeom) -> Vec<f64> {
    use kw_nurbs::CurveGeom;
    let curve_knots = |c: &CurveGeom| -> Vec<f64> {
        match c {
            CurveGeom::Nurbs(n) => n.knots().distinct().into_iter().map(|(k, _)| k).collect(),
            _ => Vec::new(),
        }
    };
    match s {
        SurfaceGeom::Nurbs(n) => n.knots_u().distinct().into_iter().map(|(k, _)| k).collect(),
        SurfaceGeom::Procedural(p) => p
            .approximation
            .knots_u()
            .distinct()
            .into_iter()
            .map(|(k, _)| k)
            .collect(),
        SurfaceGeom::Extrusion { profile, .. } | SurfaceGeom::Revolution { profile, .. } => {
            curve_knots(profile)
        }
        _ => Vec::new(),
    }
}

/// Contribution of a straight parameter-space segment `a → b` to the loop integral. Pcurve ends that miss
/// each other by a hair (vertices are merged within a tolerance, pcurves are fitted independently) are
/// bridged; gaps of whole periods are genuine and left alone.
fn bridge(
    surf: &SurfaceGeom,
    g: &mut dyn FnMut(&Point3<f64>, &Vector3<f64>, &Vector3<f64>) -> f64,
    u_ref: f64,
    inner_scale: f64,
    ubreaks: &[f64],
    a: (f64, f64),
    b: (f64, f64),
) -> f64 {
    let (gx, gy) = (b.0 - a.0, b.1 - a.1);
    let m = gx.abs().max(gy.abs());
    if !(m > 0.0 && m < 1e-3 && gy != 0.0) {
        return 0.0;
    }
    gauss(0.0, 1.0, &mut |t| {
        let (u, v) = (a.0 + gx * t, a.1 + gy * t);
        let inner = gauss_composite(u_ref, u, inner_scale, ubreaks, &mut |s| {
            let p = surf.point_at(s, v);
            let (su, sv) = surf.derivatives(s, v);
            g(&p, &su, &sv)
        });
        inner * gy
    })
}

impl BRep {
    /// `Σ_faces ∫ F·n dA` over the faces of `body` (outward normals), or `None` if a coedge lacks a pcurve.
    pub fn flux(&self, body: BodyId, field: &dyn Fn(&Point3<f64>) -> Vector3<f64>) -> Option<f64> {
        self.flux_faces(&self.body_faces(body), field)
    }

    /// Enclosed volume of one shell (positive for an outer shell, negative for a void).
    pub fn shell_volume(&self, shell: ShellId) -> Option<f64> {
        let faces = self.shells.get(shell)?.faces.clone();
        self.faces_volume(&faces)
    }

    pub fn flux_faces(
        &self,
        faces: &[FaceId],
        field: &dyn Fn(&Point3<f64>) -> Vector3<f64>,
    ) -> Option<f64> {
        let mut total = 0.0;
        for &f in faces {
            let face = self.faces.get(f)?;
            let surf = self.surfaces.get(face.surface)?;
            for &lp in &face.loops {
                total +=
                    self.loop_integral(surf, lp, &mut |p, su, sv| field(p).dot(&su.cross(sv)))?;
            }
        }
        Some(total)
    }

    /// `∮ over the loop of (∫_{u*}^{u} g(s, v) ds) dv`.
    fn loop_integral(
        &self,
        surf: &SurfaceGeom,
        lp: LoopId,
        g: &mut dyn FnMut(&Point3<f64>, &Vector3<f64>, &Vector3<f64>) -> f64,
    ) -> Option<f64> {
        let coedges = self.loop_coedges(lp);
        let first = *coedges.first()?;
        let pc0 = self.curves.get(self.coedges.get(first)?.pcurve?)?;
        let (t_start, _) = self.coedge_param_range(first)?;
        let u_ref = pc0.point_at(t_start).x;
        let ubreaks = u_breaks(surf);
        // rational / freeform integrands need finer panels than the analytic quadrics
        let inner_scale = if matches!(surf, SurfaceGeom::Nurbs(_) | SurfaceGeom::Procedural(_)) {
            0.12
        } else if matches!(surf, SurfaceGeom::Plane(_)) {
            // the integrand is a polynomial of low degree in `s` on a plane: one Gauss panel is exact
            f64::INFINITY
        } else {
            // analytic quadrics: trigonometric in `s`, which an 8-point panel integrates to ~1e-20
            // over 1.5 rad
            1.5
        };
        let mut total = 0.0;
        let first_uv = (u_ref, pc0.point_at(t_start).y);
        let mut last_uv = first_uv;
        for co in coedges {
            let c = self.coedges.get(co)?;
            let pc = self.curves.get(c.pcurve?)?;
            let (t0, t1) = self.coedge_param_range(co)?; // traversal order: reversed coedges run backwards
            let (q0, q1) = (pc.point_at(t0), pc.point_at(t1));
            total += bridge(surf, g, u_ref, inner_scale, &ubreaks, last_uv, (q0.x, q0.y));
            last_uv = (q1.x, q1.y);
            // split by pcurve knots for NURBS to keep panels smooth
            // panels: one per knot span for NURBS pcurves (their derivative jumps at the knots),
            // further split so that no panel is longer than ~0.35 in parameter space
            let mut breaks: Vec<f64> = vec![t0];
            if let kw_nurbs::CurveGeom::Nurbs(n) = pc {
                let (lo, hi) = (t0.min(t1), t0.max(t1));
                let mut ks: Vec<f64> = n
                    .knots()
                    .distinct()
                    .into_iter()
                    .map(|(k, _)| k)
                    .filter(|k| *k > lo && *k < hi)
                    .collect();
                if t0 > t1 {
                    ks.reverse();
                }
                breaks.extend(ks);
            }
            breaks.push(t1);
            for w in breaks.windows(2) {
                let (a0, b0) = (w[0], w[1]);
                let extent = (pc.derivative(a0).norm()
                    + pc.derivative(0.5 * (a0 + b0)).norm()
                    + pc.derivative(b0).norm())
                    / 3.0
                    * (b0 - a0).abs();
                let nseg = if matches!(surf, SurfaceGeom::Plane(_))
                    && matches!(pc, kw_nurbs::CurveGeom::Line(_))
                {
                    // polynomial integrand along a straight boundary of a plane: one panel is exact
                    1
                } else {
                    ((extent
                        / if matches!(surf, SurfaceGeom::Nurbs(_) | SurfaceGeom::Procedural(_)) {
                            0.12
                        } else {
                            1.2
                        })
                    .ceil() as usize)
                        .clamp(1, 160)
                };
                for k in 0..nseg {
                    let (a, b) = (
                        a0 + (b0 - a0) * k as f64 / nseg as f64,
                        a0 + (b0 - a0) * (k + 1) as f64 / nseg as f64,
                    );
                    total += gauss(a, b, &mut |t| {
                        let q = pc.point_at(t);
                        let dq = pc.derivative(t);
                        let (u, v) = (q.x, q.y);
                        if dq.y == 0.0 {
                            return 0.0;
                        }
                        let inner = gauss_composite(u_ref, u, inner_scale, &ubreaks, &mut |s| {
                            let p = surf.point_at(s, v);
                            let (su, sv) = surf.derivatives(s, v);
                            g(&p, &su, &sv)
                        });
                        inner * dq.y
                    });
                }
            }
        }
        total += bridge(surf, g, u_ref, inner_scale, &ubreaks, last_uv, first_uv);
        Some(total)
    }

    /// Surface area of one (trimmed) face.
    pub fn face_area(&self, f: FaceId) -> Option<f64> {
        let face = self.faces.get(f)?;
        let surf = self.surfaces.get(face.surface)?;
        let sigma = if face.sense == Sense::Forward {
            1.0
        } else {
            -1.0
        };
        if let SurfaceGeom::Plane(_) = surf {
            return Some(sigma * self.planar_uv_area(face)?);
        }
        let mut face_sum = 0.0;
        for &lp in &face.loops {
            face_sum += self.loop_integral(surf, lp, &mut |_, su, sv| su.cross(sv).norm())?;
        }
        Some(sigma * face_sum)
    }

    /// Surface area of the body's faces.
    pub fn area(&self, body: BodyId) -> Option<f64> {
        let mut total = 0.0;
        for f in self.body_faces(body) {
            total += self.face_area(f)?;
        }
        Some(total)
    }

    /// `½∮(u dv − v du)` over the pcurves of a loop: the signed parameter-space area it encloses.
    fn loop_uv_area(&self, lp: LoopId) -> Option<f64> {
        let mut total = 0.0;
        let mut first: Option<(f64, f64)> = None;
        let mut last = (0.0, 0.0);
        // pcurve ends that miss each other by a hair are bridged by a straight segment
        let bridge = |from: (f64, f64), to: (f64, f64)| -> f64 {
            let m = (to.0 - from.0).abs().max((to.1 - from.1).abs());
            if m > 0.0 && m < 1e-3 {
                0.5 * (from.0 * to.1 - to.0 * from.1)
            } else {
                0.0
            }
        };
        for co in self.loop_coedges(lp) {
            let c = self.coedges.get(co)?;
            let pc = self.curves.get(c.pcurve?)?;
            let (t0, t1) = self.coedge_param_range(co)?;
            let (q0, q1) = (pc.point_at(t0), pc.point_at(t1));
            match first {
                None => first = Some((q0.x, q0.y)),
                Some(_) => total += bridge(last, (q0.x, q0.y)),
            }
            last = (q1.x, q1.y);
            if let kw_nurbs::CurveGeom::Line(_) = pc {
                let (p, q) = (pc.point_at(t0), pc.point_at(t1));
                total += 0.5 * (p.x * q.y - q.x * p.y);
                continue;
            }
            let mut breaks: Vec<f64> = vec![t0];
            if let kw_nurbs::CurveGeom::Nurbs(n) = pc {
                let (lo, hi) = (t0.min(t1), t0.max(t1));
                let mut ks: Vec<f64> = n
                    .knots()
                    .distinct()
                    .into_iter()
                    .map(|(k, _)| k)
                    .filter(|k| *k > lo && *k < hi)
                    .collect();
                if t0 > t1 {
                    ks.reverse();
                }
                breaks.extend(ks);
            }
            breaks.push(t1);
            for w in breaks.windows(2) {
                let (a0, b0) = (w[0], w[1]);
                let extent = (pc.derivative(a0).norm()
                    + pc.derivative(0.5 * (a0 + b0)).norm()
                    + pc.derivative(b0).norm())
                    / 3.0
                    * (b0 - a0).abs();
                let nseg = ((extent / 0.35).ceil() as usize).clamp(1, 160);
                for k in 0..nseg {
                    let (a, b) = (
                        a0 + (b0 - a0) * k as f64 / nseg as f64,
                        a0 + (b0 - a0) * (k + 1) as f64 / nseg as f64,
                    );
                    total += gauss(a, b, &mut |t| {
                        let (q, d) = (pc.point_at(t), pc.derivative(t));
                        0.5 * (q.x * d.y - q.y * d.x)
                    });
                }
            }
        }
        if let Some(f) = first {
            total += bridge(last, f);
        }
        Some(total)
    }

    /// Signed parameter-space area of all loops of a planar face (positive for a `Forward` face).
    fn planar_uv_area(&self, face: &Face) -> Option<f64> {
        let mut s = 0.0;
        for &lp in &face.loops {
            s += self.loop_uv_area(lp)?;
        }
        Some(s)
    }

    pub fn volume(&self, body: BodyId) -> Option<f64> {
        self.faces_volume(&self.body_faces(body))
    }

    /// `⅓ Σ ∫ x·n dA` over the given faces.
    fn faces_volume(&self, faces: &[FaceId]) -> Option<f64> {
        let mut total = 0.0;
        let mut curved: Vec<FaceId> = Vec::new();
        for &f in faces {
            let face = self.faces.get(f)?;
            match self.surfaces.get(face.surface)? {
                // x·n is constant on a plane, so the face contributes (x·n)·area/3 with a 1D boundary integral
                SurfaceGeom::Plane(pl) => {
                    total += pl.frame.z.dot(&pl.frame.origin.coords) / 3.0
                        * self.planar_uv_area(face)?;
                }
                _ => curved.push(f),
            }
        }
        Some(total + self.flux_faces(&curved, &|p| p.coords / 3.0)?)
    }

    pub fn mass_properties(&self, body: BodyId) -> Option<MassProps> {
        let volume = self.volume(body)?;
        let area = self.area(body)?;
        let m = |f: &dyn Fn(&Point3<f64>) -> Vector3<f64>| self.flux(body, f);
        let mx = m(&|p| Vector3::new(0.5 * p.x * p.x, 0.0, 0.0))?;
        let my = m(&|p| Vector3::new(0.0, 0.5 * p.y * p.y, 0.0))?;
        let mz = m(&|p| Vector3::new(0.0, 0.0, 0.5 * p.z * p.z))?;
        // second moments ∫ x² dV etc.
        let xx = m(&|p| Vector3::new(p.x.powi(3) / 3.0, 0.0, 0.0))?;
        let yy = m(&|p| Vector3::new(0.0, p.y.powi(3) / 3.0, 0.0))?;
        let zz = m(&|p| Vector3::new(0.0, 0.0, p.z.powi(3) / 3.0))?;
        let xy = m(&|p| Vector3::new(0.5 * p.x * p.x * p.y, 0.0, 0.0))?;
        let xz = m(&|p| Vector3::new(0.5 * p.x * p.x * p.z, 0.0, 0.0))?;
        let yz = m(&|p| Vector3::new(0.0, 0.5 * p.y * p.y * p.z, 0.0))?;
        let centroid = if volume.abs() > 0.0 {
            Point3::new(mx, my, mz) / volume
        } else {
            Point3::origin()
        };
        // inertia tensor about the origin (unit density)
        let origin = Matrix3::new(yy + zz, -xy, -xz, -xy, xx + zz, -yz, -xz, -yz, xx + yy);
        Some(MassProps {
            volume,
            area,
            centroid,
            inertia_origin: origin,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MassProps {
    pub volume: f64,
    pub area: f64,
    pub centroid: Point3<f64>,
    /// Inertia tensor about the world origin for unit density.
    pub inertia_origin: Matrix3<f64>,
}

impl MassProps {
    /// Inertia tensor about the centroid (parallel-axis theorem).
    pub fn inertia_centroid(&self) -> Matrix3<f64> {
        let c = self.centroid.coords;
        let d2 = c.norm_squared();
        self.inertia_origin - self.volume * (Matrix3::identity() * d2 - c * c.transpose())
    }
}
