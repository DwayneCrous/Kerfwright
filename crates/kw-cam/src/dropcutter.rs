//! Drop-cutter: the lowest tool-tip height at which a flat, ball or bull-nose cutter at a given (x, y)
//! touches a triangle mesh without cutting into it, plus the exact collision test for the cylindrical
//! sections above the flutes (shank and holder).
//!
//! The tool is the convex body `disc(ρ) ⊕ ball(r)` below its centre plane; its lowest surface at horizontal
//! distance `d` from the axis sits `f(d)` above the tip (`f = 0` for `d ≤ ρ`, `r − √(r² − (d − ρ)²)` up to
//! `d = R = ρ + r`). A triangle point `p` within `R` of the axis forbids every tip height below
//! `p.z − f(d(p))`, so the answer is the maximum of that function over the triangle clipped to the disc of
//! radius `R`. The function is concave on the triangle (linear minus a convex increasing function of a convex
//! distance), so the maximum is found exactly at: a vertex, the best point of an edge (one-dimensional concave
//! maximisation), or the single interior point where the tool surface is parallel to the facet.

use crate::bvh::{Aabb, Tri, TriMesh};
use crate::tool::{Tool, ToolKind};
use crate::P3;

/// Profile parameters of a cutter: disc radius `rho`, corner radius `r`, total radius `big`.
#[derive(Debug, Clone, Copy)]
pub struct Profile {
    pub rho: f64,
    pub r: f64,
    pub big: f64,
}

impl Profile {
    pub fn of(tool: &Tool) -> Profile {
        let big = tool.radius();
        match tool.kind {
            ToolKind::Flat => Profile {
                rho: big,
                r: 0.0,
                big,
            },
            ToolKind::Ball => Profile {
                rho: 0.0,
                r: big,
                big,
            },
            ToolKind::Bull { corner_radius } => {
                let r = corner_radius.min(big);
                Profile {
                    rho: big - r,
                    r,
                    big,
                }
            }
        }
    }

    /// Height of the tool surface above the tip at distance `d` from the axis (`d ≤ big`).
    pub fn lift(&self, d: f64) -> f64 {
        if d <= self.rho {
            0.0
        } else {
            let e = (d - self.rho).min(self.r);
            self.r - (self.r * self.r - e * e).max(0.0).sqrt()
        }
    }
}

fn dist_xy(p: P3, c: [f64; 2]) -> f64 {
    (p[0] - c[0]).hypot(p[1] - c[1])
}

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Parameter interval `[s0, s1] ⊂ [0, 1]` of the segment `a + s (b − a)` inside the horizontal disc.
fn clip_segment_to_disc(a: P3, b: P3, c: [f64; 2], rad: f64) -> Option<(f64, f64)> {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let (fx, fy) = (a[0] - c[0], a[1] - c[1]);
    let qa = dx * dx + dy * dy;
    let qc = fx * fx + fy * fy - rad * rad;
    if qa < 1e-30 {
        return if qc <= 0.0 { Some((0.0, 1.0)) } else { None };
    }
    let qb = fx * dx + fy * dy;
    let disc = qb * qb - qa * qc;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let (s0, s1) = ((-qb - sq) / qa, (-qb + sq) / qa);
    let (s0, s1) = (s0.max(0.0), s1.min(1.0));
    if s0 <= s1 {
        Some((s0, s1))
    } else {
        None
    }
}

fn lerp(a: P3, b: P3, s: f64) -> P3 {
    [
        a[0] + (b[0] - a[0]) * s,
        a[1] + (b[1] - a[1]) * s,
        a[2] + (b[2] - a[2]) * s,
    ]
}

fn in_triangle_xy(t: &Tri, p: [f64; 2]) -> bool {
    let e = |a: P3, b: P3| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
    let (d0, d1, d2) = (e(t[0], t[1]), e(t[1], t[2]), e(t[2], t[0]));
    (d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0 && d2 <= 0.0)
}

/// Lowest non-cutting tip height over one triangle, or `None` when the triangle is out of reach.
pub fn drop_triangle(t: &Tri, prof: &Profile, c: [f64; 2]) -> Option<f64> {
    let g = |p: P3| p[2] - prof.lift(dist_xy(p, c).min(prof.big));
    let mut best: Option<f64> = None;
    let mut take = |v: f64| {
        best = Some(best.map_or(v, |b: f64| b.max(v)));
    };
    for v in t {
        if dist_xy(*v, c) <= prof.big {
            take(g(*v));
        }
    }
    for k in 0..3 {
        let (a, b) = (t[k], t[(k + 1) % 3]);
        if let Some((mut s0, mut s1)) = clip_segment_to_disc(a, b, c, prof.big) {
            // golden-section maximisation of the concave g along the clipped edge
            let phi = 0.618_033_988_749_894_9;
            let (mut x1, mut x2) = (s1 - phi * (s1 - s0), s0 + phi * (s1 - s0));
            let (mut f1, mut f2) = (g(lerp(a, b, x1)), g(lerp(a, b, x2)));
            for _ in 0..48 {
                if f1 < f2 {
                    s0 = x1;
                    x1 = x2;
                    f1 = f2;
                    x2 = s0 + phi * (s1 - s0);
                    f2 = g(lerp(a, b, x2));
                } else {
                    s1 = x2;
                    x2 = x1;
                    f2 = f1;
                    x1 = s1 - phi * (s1 - s0);
                    f1 = g(lerp(a, b, x1));
                }
            }
            take(g(lerp(a, b, 0.5 * (s0 + s1))));
            take(g(lerp(a, b, s0)));
            take(g(lerp(a, b, s1)));
            // the exact end points of the clipped interval
            if let Some((e0, e1)) = clip_segment_to_disc(a, b, c, prof.big) {
                take(g(lerp(a, b, e0)));
                take(g(lerp(a, b, e1)));
            }
        }
    }
    // facet interior
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    if n[2].abs() > 1e-14 * (n[0].abs() + n[1].abs() + n[2].abs()) {
        let s = [-n[0] / n[2], -n[1] / n[2]];
        let sl = s[0].hypot(s[1]);
        let d = prof.rho + prof.r * sl / (1.0 + sl * sl).sqrt();
        let p = if sl < 1e-15 {
            [c[0], c[1]]
        } else {
            [c[0] + d * s[0] / sl, c[1] + d * s[1] / sl]
        };
        if in_triangle_xy(t, p) {
            let z = t[0][2] + s[0] * (p[0] - t[0][0]) + s[1] * (p[1] - t[0][1]);
            take(g([p[0], p[1], z]));
        }
    }
    best
}

/// Lowest non-cutting tip height at `(x, y)` over a mesh; `None` when nothing is within reach.
pub fn drop_cutter(mesh: &TriMesh, tool: &Tool, x: f64, y: f64) -> Option<f64> {
    let prof = Profile::of(tool);
    let q = Aabb {
        min: [x - prof.big, y - prof.big, f64::NEG_INFINITY],
        max: [x + prof.big, y + prof.big, f64::INFINITY],
    };
    let mut best: Option<f64> = None;
    mesh.query(&q, |i| {
        let t = &mesh.tris[i];
        // g never exceeds the height of the point it is evaluated at: a triangle whose highest vertex lies
        // below the best answer so far cannot improve it
        if best.is_some_and(|b| t[0][2].max(t[1][2]).max(t[2][2]) <= b) {
            return;
        }
        if let Some(v) = drop_triangle(t, &prof, [x, y]) {
            best = Some(best.map_or(v, |b| b.max(v)));
        }
    });
    best
}

/// Extreme heights of a triangle clipped to the horizontal disc (radius `rad` about `c`): exact, since the
/// height is linear and its extremes lie at vertices, edge/circle crossings, or on the circle in the gradient
/// direction.
pub fn z_extent_in_disc(t: &Tri, c: [f64; 2], rad: f64) -> Option<(f64, f64)> {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    let mut take = |z: f64| {
        lo = lo.min(z);
        hi = hi.max(z);
    };
    for v in t {
        if dist_xy(*v, c) <= rad {
            take(v[2]);
        }
    }
    for k in 0..3 {
        let (a, b) = (t[k], t[(k + 1) % 3]);
        if let Some((s0, s1)) = clip_segment_to_disc(a, b, c, rad) {
            take(lerp(a, b, s0)[2]);
            take(lerp(a, b, s1)[2]);
        }
    }
    let n = cross(sub(t[1], t[0]), sub(t[2], t[0]));
    if n[2].abs() > 1e-14 * (n[0].abs() + n[1].abs() + n[2].abs()) {
        let s = [-n[0] / n[2], -n[1] / n[2]];
        let sl = s[0].hypot(s[1]);
        if sl > 1e-15 {
            for sign in [1.0, -1.0] {
                let p = [c[0] + sign * rad * s[0] / sl, c[1] + sign * rad * s[1] / sl];
                if in_triangle_xy(t, p) {
                    take(t[0][2] + s[0] * (p[0] - t[0][0]) + s[1] * (p[1] - t[0][1]));
                }
            }
        } else if in_triangle_xy(t, c) {
            take(t[0][2]);
        }
    }
    if lo.is_finite() {
        Some((lo, hi))
    } else {
        None
    }
}

/// Does the cylinder of radius `rad` about the vertical line through `c`, occupying heights `[z0, z1]`,
/// intersect the mesh (penetrating by more than `tol`)?
pub fn cylinder_hits_mesh(
    mesh: &TriMesh,
    c: [f64; 2],
    rad: f64,
    z0: f64,
    z1: f64,
    tol: f64,
) -> bool {
    let q = Aabb {
        min: [c[0] - rad, c[1] - rad, z0],
        max: [c[0] + rad, c[1] + rad, z1],
    };
    let mut hit = false;
    mesh.query(&q, |i| {
        if hit {
            return;
        }
        if let Some((lo, hi)) = z_extent_in_disc(&mesh.tris[i], c, rad - tol.max(0.0)) {
            if hi > z0 + tol && lo < z1 - tol {
                hit = true;
            }
        }
    });
    hit
}

/// Brute-force reference for the drop-cutter: sample every triangle densely in barycentric coordinates and
/// take the maximum of `p.z − lift(d)` over the samples within reach. O(triangles × samples); an
/// under-estimate by at most the sample spacing times the surface slope.
pub fn drop_cutter_brute(tris: &[Tri], tool: &Tool, x: f64, y: f64, n: usize) -> Option<f64> {
    let prof = Profile::of(tool);
    let mut best: Option<f64> = None;
    for t in tris {
        for i in 0..=n {
            for j in 0..=(n - i) {
                let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                let w = 1.0 - u - v;
                let p = [
                    w * t[0][0] + u * t[1][0] + v * t[2][0],
                    w * t[0][1] + u * t[1][1] + v * t[2][1],
                    w * t[0][2] + u * t[1][2] + v * t[2][2],
                ];
                let d = dist_xy(p, [x, y]);
                if d <= prof.big {
                    let g = p[2] - prof.lift(d);
                    best = Some(best.map_or(g, |b| b.max(g)));
                }
            }
        }
    }
    best
}
