//! Principal, Gaussian and mean curvature of a surface.
//!
//! The sign follows the *outward* normal of the face: curvature is positive where the surface bends
//! away from its normal, as on the outside of a sphere or the convex side of a cylinder, so a
//! convex body is positive and a hole or a fillet's concave side is negative. Gaussian curvature
//! does not depend on the orientation.
//!
//! NURBS surfaces use the exact first and second derivatives (the second fundamental form). The
//! other surface kinds use the shape operator from differences of their exact normals.

use kw_brep::{BRep, Sense};
use kw_nurbs::{Surface, SurfaceGeom};
use kw_tess::Tessellation;

/// The two principal curvatures, `k1 >= k2`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Curv {
    pub k1: f64,
    pub k2: f64,
}

impl Curv {
    pub fn gaussian(self) -> f64 {
        self.k1 * self.k2
    }
    pub fn mean(self) -> f64 {
        0.5 * (self.k1 + self.k2)
    }
    /// The same curvature seen from the other side of the surface.
    pub fn flipped(self) -> Curv {
        Curv {
            k1: -self.k2,
            k2: -self.k1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurvKind {
    Gaussian,
    Mean,
    Max,
    Min,
}

impl CurvKind {
    pub fn pick(self, c: Curv) -> f64 {
        match self {
            CurvKind::Gaussian => c.gaussian(),
            CurvKind::Mean => c.mean(),
            CurvKind::Max => c.k1,
            CurvKind::Min => c.k2,
        }
    }
}

fn from_operator(w: [[f64; 2]; 2]) -> Curv {
    let tr = w[0][0] + w[1][1];
    let det = w[0][0] * w[1][1] - w[0][1] * w[1][0];
    let h = tr / 2.0;
    let d = (h * h - det).max(0.0).sqrt();
    Curv {
        k1: h + d,
        k2: h - d,
    }
}

/// W = -a^{-1} II for a NURBS patch: `a` the first and `II` the second fundamental form.
fn nurbs_curv(s: &kw_nurbs::NurbsSurface, u: f64, v: f64) -> Option<Curv> {
    let d = s.derivatives(u, v, 2);
    let (su, sv) = (d[1][0], d[0][1]);
    let (suu, suv, svv) = (d[2][0], d[1][1], d[0][2]);
    let nn = su.cross(&sv);
    let l = nn.norm();
    if l < 1e-12 {
        return None;
    }
    let n = nn / l;
    let (e, f, g) = (su.dot(&su), su.dot(&sv), sv.dot(&sv));
    let det = e * g - f * f;
    if det < 1e-18 * e.max(g).max(1e-300).powi(2) {
        return None;
    }
    let (ll, m, nn2) = (suu.dot(&n), suv.dot(&n), svv.dot(&n));
    // a^{-1} = 1/det [[g, -f], [-f, e]]
    let w = [
        [-(g * ll - f * m) / det, -(g * m - f * nn2) / det],
        [-(-f * ll + e * m) / det, -(-f * m + e * nn2) / det],
    ];
    Some(from_operator(w))
}

/// W from differences of the unit normal (any surface that knows its normal and first derivatives).
fn fd_curv(s: &SurfaceGeom, u: f64, v: f64) -> Option<Curv> {
    let ((ua, ub), (va, vb)) = s.domain();
    let (hu, hv) = (
        1e-5 * (ub - ua).abs().max(1e-9),
        1e-5 * (vb - va).abs().max(1e-9),
    );
    // central differences, one-sided at the edge of the domain
    let diff = |du: f64, dv: f64, h: f64, lo: f64, hi: f64, x: f64| {
        let (a, b) = ((x - h).max(lo), (x + h).min(hi));
        let (pa, pb) = if du != 0.0 {
            (s.normal_at(a, v), s.normal_at(b, v))
        } else {
            (s.normal_at(u, a), s.normal_at(u, b))
        };
        let _ = dv;
        (pb - pa) / (b - a).max(1e-300)
    };
    let nu = diff(1.0, 0.0, hu, ua, ub, u);
    let nv = diff(0.0, 1.0, hv, va, vb, v);
    let (su, sv) = s.derivatives(u, v);
    let (e, f, g) = (su.dot(&su), su.dot(&sv), sv.dot(&sv));
    let det = e * g - f * f;
    if det < 1e-18 * e.max(g).max(1e-300).powi(2) {
        return None;
    }
    // n_u = w11 Su + w21 Sv, n_v = w12 Su + w22 Sv (components in the tangent basis)
    let comps = |n: nalgebra::Vector3<f64>| {
        let (p, q) = (n.dot(&su), n.dot(&sv));
        ((g * p - f * q) / det, (-f * p + e * q) / det)
    };
    let (w11, w21) = comps(nu);
    let (w12, w22) = comps(nv);
    Some(from_operator([[w11, w12], [w21, w22]]))
}

/// The curvature of `surface` at `(u, v)` for the direction of its own normal (`Su x Sv`). At a pole
/// or apex, where the parametrisation degenerates, the point is moved a little into the surface.
pub fn curvature_at(surface: &SurfaceGeom, u: f64, v: f64) -> Option<Curv> {
    let at = |u: f64, v: f64| match surface {
        SurfaceGeom::Nurbs(n) => nurbs_curv(n, u, v),
        SurfaceGeom::Procedural(p) => nurbs_curv(&p.approximation, u, v),
        SurfaceGeom::Plane(_) => Some(Curv { k1: 0.0, k2: 0.0 }),
        other => fd_curv(other, u, v),
    };
    if let Some(c) = at(u, v) {
        return Some(c);
    }
    let ((ua, ub), (va, vb)) = surface.domain();
    let (cu, cv) = (0.5 * (ua + ub), 0.5 * (va + vb));
    for t in [1e-3, 1e-2, 5e-2] {
        let (uu, vv) = (u + (cu - u) * t, v + (cv - v) * t);
        if let Some(c) = at(uu, vv) {
            return Some(c);
        }
    }
    None
}

/// The chosen curvature at every vertex of a tessellation, for the face's outward side. Vertices
/// where it cannot be found (degenerate) get `NaN`.
pub fn mesh_curvature(brep: &BRep, t: &Tessellation, kind: CurvKind) -> Vec<f32> {
    let n = t.mesh.positions.len();
    let mut face_of = vec![u32::MAX; n];
    for (tri, f) in t.mesh.tri_face.iter().enumerate() {
        for k in 0..3 {
            face_of[t.mesh.indices[3 * tri + k] as usize] = *f;
        }
    }
    (0..n)
        .map(|i| {
            let fi = face_of[i];
            let Some(fk) = t.face_keys.get(fi as usize) else {
                return f32::NAN;
            };
            let Some(face) = brep.faces.get(*fk) else {
                return f32::NAN;
            };
            let Some(surf) = brep.surfaces.get(face.surface) else {
                return f32::NAN;
            };
            let (u, v) = t.mesh.uv.get(i).copied().unwrap_or((0.0, 0.0));
            match curvature_at(surf, u, v) {
                Some(c) => {
                    let c = if face.sense == Sense::Reversed {
                        c.flipped()
                    } else {
                        c
                    };
                    kind.pick(c) as f32
                }
                None => f32::NAN,
            }
        })
        .collect()
}
