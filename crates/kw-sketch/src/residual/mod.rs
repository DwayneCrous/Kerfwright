//! Scalar residual library. Every constraint lowers to one or more
//! [`Residual`]s: a [`Kind`] plus the global variable indices bound to its
//! local slots.
//!
//! * `eval` is written once, generic over [`Scalar`] (f64 for solving, `Dual`
//!   for the AD oracle);
//! * `jac` is the hand-coded analytic Jacobian used by the solver;
//! * `pattern` gives the structural sparsity per equation (used by the
//!   Dulmage–Mendelsohn decomposition).
//!
//! Slot layouts are documented on each [`Kind`] variant. A "point" occupies two
//! consecutive slots `(x, y)`; the slot index named in a layout is its x slot.

mod curves;

pub use curves::{arc_end, line_end, spline_end, EndLin, EndTerm};

use crate::ad::Scalar;

pub const MAXV: usize = 16;
pub const MAXR: usize = 4;

pub type Jac = [[f64; MAXV]; MAXR];

/// Source of a circle's radius inside a residual.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Rad {
    /// Radius is variable slot `i`.
    Var(u8),
    /// Radius is `|s − c|` for the points at slots `c` and `s` (arcs).
    Pt { c: u8, s: u8 },
}

#[derive(Clone, Debug)]
pub struct JointSpec {
    pub a: EndLin,
    pub b: EndLin,
    /// `+1` for a curve's end, `−1` for its start: orients tangent/curvature
    /// *away from* the joint.
    pub sign_a: f64,
    pub sign_b: f64,
    /// 0 = G0 (position), 1 = G1 (+tangent direction), 2 = G2 (+curvature).
    pub level: u8,
    /// Include the two position equations. `false` when both curves already
    /// share the joint's point variables (the equations would vanish identically).
    pub g0: bool,
    /// Sign of `To_A · To_B` captured at creation (branch-flip diagnostics only).
    pub branch: f64,
}

#[derive(Clone, Debug)]
pub enum Kind {
    /// `[ax ay bx by]` → `a − b` (2 equations).
    Pt2Pt,
    /// `[px py ax ay bx by]` → signed distance of `p` from the line `a→b`
    /// (`cross(b−a, p−a)/|b−a|`; singular at zero length, so degenerate lines are repelled).
    OnLine,
    /// `[px py cx cy ..]` → `|p−c|² − R²`.
    OnCircle(Rad),
    /// `[px py cx cy mx my b]` → ellipse implicit form (`m` = major-axis end, `b` = minor radius).
    OnEllipse,
    /// `[a b]` → `a − b`.
    Diff,
    /// `[.. v]` → `scale·R − v` (radius: scale 1, diameter: 2).
    RadVal { rad: Rad, scale: f64, v: u8 },
    /// `[ax ay bx by cx cy dx dy]` → `sin φ`, the cross product of the unit directions of `a→b` and `c→d`.
    Parallel,
    /// → `cos φ`, the dot product of the unit directions.
    Perp,
    /// → `|b−a|² − |d−c|²`.
    EqLen,
    /// → `R1 − R2`.
    EqRad(Rad, Rad),
    /// `[px py ax ay bx by]` → `2p − a − b` (2 equations).
    Midpoint,
    /// `[ax ay bx by l0x l0y l1x l1y]` → distance of the midpoint from the axis;
    /// component of `b−a` along the axis (2 equations, both lengths).
    Symmetric,
    /// `[ax ay bx by cx cy ..]` → signed distance of the centre from the line minus `s·R`.
    TanLineCirc { s: f64, r: Rad },
    /// `[c1x c1y c2x c2y ..]` → `|c1−c2|² − (R1 + s·R2)²`.
    TanCircCirc { s: f64, r1: Rad, r2: Rad },
    /// `[ax ay bx by d]` → `|b−a|² − d²`.
    DistPP,
    /// `[px py ax ay bx by d]` → signed distance of `p` from the line minus `s·d`.
    DistPL { s: f64 },
    /// `[ax ay bx by cx cy dx dy θ]` → `sin(φ−θ)` with φ the directed angle from `b−a` to `d−c`.
    Angle,
    /// `[cx cy sx sy ex ey]` → `|e−c|² − |s−c|²` (arc has one radius).
    ArcCons,
    /// Continuity joint between two curve ends.
    Joint(Box<JointSpec>),
}

#[derive(Clone, Debug)]
pub struct Residual {
    pub kind: Kind,
    /// Global variable index for each local slot.
    pub vars: Vec<u32>,
}

impl JointSpec {
    pub fn g0_rows(&self) -> usize {
        if self.g0 {
            2
        } else {
            0
        }
    }
}

impl Kind {
    pub fn arity(&self) -> usize {
        match self {
            Kind::Pt2Pt | Kind::Midpoint | Kind::Symmetric => 2,
            Kind::Joint(j) => j.g0_rows() + j.level as usize,
            _ => 1,
        }
    }

    /// Weight that makes equation `eq` dimensionless for a sketch of
    /// characteristic length `scale` (length² equations → `1/scale²`, etc.).
    pub fn row_weight(&self, eq: usize, scale: f64) -> f64 {
        match self {
            Kind::Pt2Pt
            | Kind::Midpoint
            | Kind::Diff
            | Kind::RadVal { .. }
            | Kind::EqRad(..)
            | Kind::OnLine
            | Kind::Symmetric
            | Kind::TanLineCirc { .. }
            | Kind::DistPL { .. } => 1.0 / scale,
            Kind::Angle | Kind::OnEllipse | Kind::Parallel | Kind::Perp => 1.0,
            Kind::OnCircle(_)
            | Kind::EqLen
            | Kind::TanCircCirc { .. }
            | Kind::DistPP
            | Kind::ArcCons => 1.0 / (scale * scale),
            Kind::Joint(j) => {
                let e = eq + 2 - j.g0_rows(); // logical index: 0,1 position, 2 tangent, 3 curvature
                match e {
                    0 | 1 => 1.0 / scale,
                    2 => 1.0,
                    _ => scale,
                }
            }
        }
    }

    /// Local slots that equation `eq` structurally depends on.
    pub fn pattern(&self, eq: usize, nv: usize, out: &mut Vec<u8>) {
        out.clear();
        match self {
            Kind::Pt2Pt => out.extend([eq as u8, eq as u8 + 2]),
            Kind::Midpoint => out.extend([eq as u8, eq as u8 + 2, eq as u8 + 4]),
            Kind::Joint(j) => {
                let e = eq + 2 - j.g0_rows();
                if e < 2 {
                    for t in j.a.terms.iter().chain(j.b.terms.iter()) {
                        if t.p.iter().any(|c| *c != 0.0) {
                            out.push(t.slot);
                            out.push(t.slot + 1);
                        }
                    }
                    // keep only the coordinate this equation constrains
                    // (point x slots are even, y slots odd)
                    out.retain(|s| (*s as usize) % 2 == e % 2);
                } else {
                    for t in j.a.terms.iter().chain(j.b.terms.iter()) {
                        out.push(t.slot);
                        out.push(t.slot + 1);
                    }
                }
            }
            _ => out.extend(0..nv as u8),
        }
        out.sort_unstable();
        out.dedup();
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

#[inline(always)]
fn cross<T: Scalar>(ax: T, ay: T, bx: T, by: T) -> T {
    ax * by - ay * bx
}

fn rad<T: Scalar>(v: &[T; MAXV], r: Rad) -> T {
    match r {
        Rad::Var(i) => v[i as usize],
        Rad::Pt { c, s } => {
            let (c, s) = (c as usize, s as usize);
            let dx = v[s] - v[c];
            let dy = v[s + 1] - v[c + 1];
            (dx * dx + dy * dy).sqrt()
        }
    }
}

fn rad_sq<T: Scalar>(v: &[T; MAXV], r: Rad) -> T {
    match r {
        Rad::Var(i) => v[i as usize] * v[i as usize],
        Rad::Pt { c, s } => {
            let (c, s) = (c as usize, s as usize);
            let dx = v[s] - v[c];
            let dy = v[s + 1] - v[c + 1];
            dx * dx + dy * dy
        }
    }
}

/// `row += k · ∂R/∂v`.
fn add_rad_grad(v: &[f64; MAXV], r: Rad, k: f64, row: &mut [f64; MAXV]) {
    match r {
        Rad::Var(i) => row[i as usize] += k,
        Rad::Pt { c, s } => {
            let (c, s) = (c as usize, s as usize);
            let (dx, dy) = (v[s] - v[c], v[s + 1] - v[c + 1]);
            let l = (dx * dx + dy * dy).sqrt();
            let (gx, gy) = (k * dx / l, k * dy / l);
            row[s] += gx;
            row[s + 1] += gy;
            row[c] -= gx;
            row[c + 1] -= gy;
        }
    }
}

/// `row += k · ∂(R²)/∂v`.
fn add_rad_sq_grad(v: &[f64; MAXV], r: Rad, k: f64, row: &mut [f64; MAXV]) {
    match r {
        Rad::Var(i) => row[i as usize] += 2.0 * k * v[i as usize],
        Rad::Pt { c, s } => {
            let (c, s) = (c as usize, s as usize);
            let (dx, dy) = (v[s] - v[c], v[s + 1] - v[c + 1]);
            row[s] += 2.0 * k * dx;
            row[s + 1] += 2.0 * k * dy;
            row[c] -= 2.0 * k * dx;
            row[c + 1] -= 2.0 * k * dy;
        }
    }
}

// ---------------------------------------------------------------------------
// evaluation
// ---------------------------------------------------------------------------

/// Evaluate all equations of `kind` into `out[..arity]`.
pub fn eval<T: Scalar>(kind: &Kind, v: &[T; MAXV], out: &mut [T; MAXR]) {
    match kind {
        Kind::Pt2Pt => {
            out[0] = v[0] - v[2];
            out[1] = v[1] - v[3];
        }
        Kind::OnLine => {
            let (ux, uy) = (v[4] - v[2], v[5] - v[3]);
            let l = (ux * ux + uy * uy).sqrt();
            out[0] = cross(ux / l, uy / l, v[0] - v[2], v[1] - v[3]);
        }
        Kind::OnCircle(r) => {
            let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
            out[0] = dx * dx + dy * dy - rad_sq(v, *r);
        }
        Kind::OnEllipse => {
            let (qx, qy) = (v[0] - v[2], v[1] - v[3]);
            let (mx, my) = (v[4] - v[2], v[5] - v[3]);
            let a = mx * mx + my * my;
            let xi = qx * mx + qy * my;
            let eta = mx * qy - my * qx;
            let b = v[6];
            out[0] = xi * xi / (a * a) + eta * eta / (a * b * b) - T::c(1.0);
        }
        Kind::Diff => out[0] = v[0] - v[1],
        Kind::RadVal {
            rad: r,
            scale,
            v: vs,
        } => {
            out[0] = T::c(*scale) * rad(v, *r) - v[*vs as usize];
        }
        Kind::Parallel | Kind::Perp => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
            let lu = (ux * ux + uy * uy).sqrt();
            let lw = (wx * wx + wy * wy).sqrt();
            let (n1x, n1y, n2x, n2y) = (ux / lu, uy / lu, wx / lw, wy / lw);
            out[0] = if matches!(kind, Kind::Parallel) {
                cross(n1x, n1y, n2x, n2y)
            } else {
                n1x * n2x + n1y * n2y
            };
        }
        Kind::EqLen => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
            out[0] = ux * ux + uy * uy - wx * wx - wy * wy;
        }
        Kind::EqRad(r1, r2) => out[0] = rad(v, *r1) - rad(v, *r2),
        Kind::Midpoint => {
            out[0] = T::c(2.0) * v[0] - v[2] - v[4];
            out[1] = T::c(2.0) * v[1] - v[3] - v[5];
        }
        Kind::Symmetric => {
            let half = T::c(0.5);
            let (mx, my) = (half * (v[0] + v[2]), half * (v[1] + v[3]));
            let (dx, dy) = (v[6] - v[4], v[7] - v[5]);
            let l = (dx * dx + dy * dy).sqrt();
            let (nx, ny) = (dx / l, dy / l);
            out[0] = cross(nx, ny, mx - v[4], my - v[5]);
            out[1] = (v[2] - v[0]) * nx + (v[3] - v[1]) * ny;
        }
        Kind::TanLineCirc { s, r } => {
            let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
            let (ex, ey) = (v[4] - v[0], v[5] - v[1]);
            let l = (dx * dx + dy * dy).sqrt();
            out[0] = cross(dx / l, dy / l, ex, ey) - T::c(*s) * rad(v, *r);
        }
        Kind::TanCircCirc { s, r1, r2 } => {
            let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
            let sum = rad(v, *r1) + T::c(*s) * rad(v, *r2);
            out[0] = dx * dx + dy * dy - sum * sum;
        }
        Kind::DistPP => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            out[0] = ux * ux + uy * uy - v[4] * v[4];
        }
        Kind::DistPL { s } => {
            let (ux, uy) = (v[4] - v[2], v[5] - v[3]);
            let (ex, ey) = (v[0] - v[2], v[1] - v[3]);
            let l = (ux * ux + uy * uy).sqrt();
            out[0] = cross(ux / l, uy / l, ex, ey) - T::c(*s) * v[6];
        }
        Kind::Angle => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
            let lu = (ux * ux + uy * uy).sqrt();
            let lw = (wx * wx + wy * wy).sqrt();
            let (n1x, n1y) = (ux / lu, uy / lu);
            let (n2x, n2y) = (wx / lw, wy / lw);
            let sn = cross(n1x, n1y, n2x, n2y);
            let cs = n1x * n2x + n1y * n2y;
            out[0] = sn * v[8].cos() - cs * v[8].sin();
        }
        Kind::ArcCons => {
            let (ex, ey) = (v[4] - v[0], v[5] - v[1]);
            let (sx, sy) = (v[2] - v[0], v[3] - v[1]);
            out[0] = ex * ex + ey * ey - sx * sx - sy * sy;
        }
        Kind::Joint(j) => {
            let (pa, ta, aa) = j.a.eval(v);
            let (pb, tb, ab) = j.b.eval(v);
            let mut k = 0;
            if j.g0 {
                out[0] = pa.0 - pb.0;
                out[1] = pa.1 - pb.1;
                k = 2;
            }
            if j.level >= 1 {
                let (tox, toy) = (T::c(j.sign_a) * ta.0, T::c(j.sign_a) * ta.1);
                let (tpx, tpy) = (T::c(j.sign_b) * tb.0, T::c(j.sign_b) * tb.1);
                let la = (tox * tox + toy * toy).sqrt();
                let lb = (tpx * tpx + tpy * tpy).sqrt();
                out[k] = cross(tox, toy, tpx, tpy) / (la * lb);
                k += 1;
            }
            if j.level >= 2 {
                let kap = |t: (T, T), a: (T, T)| {
                    let l2 = t.0 * t.0 + t.1 * t.1;
                    cross(t.0, t.1, a.0, a.1) / (l2 * l2.sqrt())
                };
                out[k] = T::c(j.sign_a) * kap(ta, aa) + T::c(j.sign_b) * kap(tb, ab);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// analytic Jacobians
// ---------------------------------------------------------------------------

/// Pull a gradient `g` with respect to a unit direction `n = u/|u|` back to
/// the vector `u`: `(I − n nᵀ) g / |u|`.
#[inline(always)]
fn dir_pull(nx: f64, ny: f64, l: f64, gx: f64, gy: f64) -> (f64, f64) {
    let d = nx * gx + ny * gy;
    ((gx - nx * d) / l, (gy - ny * d) / l)
}

/// Add `g·∂/∂(point at slot s)` contributions: `row[s] += gx; row[s+1] += gy`.
#[inline(always)]
fn put(row: &mut [f64; MAXV], s: usize, gx: f64, gy: f64) {
    row[s] += gx;
    row[s + 1] += gy;
}

/// Analytic Jacobian. `j` must be zero-initialised by the caller.
pub fn jac(kind: &Kind, v: &[f64; MAXV], j: &mut Jac) {
    match kind {
        Kind::Pt2Pt => {
            j[0][0] += 1.0;
            j[0][2] -= 1.0;
            j[1][1] += 1.0;
            j[1][3] -= 1.0;
        }
        Kind::OnLine => {
            let (ux, uy) = (v[4] - v[2], v[5] - v[3]);
            let (ex, ey) = (v[0] - v[2], v[1] - v[3]);
            let l = (ux * ux + uy * uy).sqrt();
            let (nx, ny) = (ux / l, uy / l);
            // f = cross(n, e): ∂/∂e = (−ny, nx); ∂/∂n = (ey, −ex)
            let (gux, guy) = dir_pull(nx, ny, l, ey, -ex);
            put(&mut j[0], 0, -ny, nx);
            put(&mut j[0], 4, gux, guy);
            put(&mut j[0], 2, ny - gux, -nx - guy);
        }
        Kind::OnCircle(r) => {
            let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
            put(&mut j[0], 0, 2.0 * dx, 2.0 * dy);
            put(&mut j[0], 2, -2.0 * dx, -2.0 * dy);
            add_rad_sq_grad(v, *r, -1.0, &mut j[0]);
        }
        Kind::OnEllipse => {
            let (qx, qy) = (v[0] - v[2], v[1] - v[3]);
            let (mx, my) = (v[4] - v[2], v[5] - v[3]);
            let b = v[6];
            let a = mx * mx + my * my;
            let xi = qx * mx + qy * my;
            let eta = mx * qy - my * qx;
            let (a2, b2) = (a * a, b * b);
            // ∂/∂q
            let dqx = 2.0 * xi / a2 * mx + 2.0 * eta / (a * b2) * (-my);
            let dqy = 2.0 * xi / a2 * my + 2.0 * eta / (a * b2) * mx;
            // ∂/∂m
            let a3 = a2 * a;
            let dmx = 2.0 * xi / a2 * qx - 4.0 * xi * xi / a3 * mx + 2.0 * eta / (a * b2) * qy
                - 2.0 * eta * eta / (a2 * b2) * mx;
            let dmy = 2.0 * xi / a2 * qy
                - 4.0 * xi * xi / a3 * my
                - 2.0 * eta / (a * b2) * qx
                - 2.0 * eta * eta / (a2 * b2) * my;
            put(&mut j[0], 0, dqx, dqy);
            put(&mut j[0], 2, -(dqx + dmx), -(dqy + dmy));
            put(&mut j[0], 4, dmx, dmy);
            j[0][6] += -2.0 * eta * eta / (a * b2 * b);
        }
        Kind::Diff => {
            j[0][0] += 1.0;
            j[0][1] -= 1.0;
        }
        Kind::RadVal {
            rad: r,
            scale,
            v: vs,
        } => {
            add_rad_grad(v, *r, *scale, &mut j[0]);
            j[0][*vs as usize] -= 1.0;
        }
        Kind::Parallel | Kind::Perp => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
            let lu = (ux * ux + uy * uy).sqrt();
            let lw = (wx * wx + wy * wy).sqrt();
            let (n1x, n1y, n2x, n2y) = (ux / lu, uy / lu, wx / lw, wy / lw);
            let ((g1x, g1y), (g2x, g2y)) = if matches!(kind, Kind::Parallel) {
                ((n2y, -n2x), (-n1y, n1x))
            } else {
                ((n2x, n2y), (n1x, n1y))
            };
            let (gux, guy) = dir_pull(n1x, n1y, lu, g1x, g1y);
            let (gwx, gwy) = dir_pull(n2x, n2y, lw, g2x, g2y);
            put(&mut j[0], 2, gux, guy);
            put(&mut j[0], 0, -gux, -guy);
            put(&mut j[0], 6, gwx, gwy);
            put(&mut j[0], 4, -gwx, -gwy);
        }
        Kind::EqLen => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
            put(&mut j[0], 2, 2.0 * ux, 2.0 * uy);
            put(&mut j[0], 0, -2.0 * ux, -2.0 * uy);
            put(&mut j[0], 6, -2.0 * wx, -2.0 * wy);
            put(&mut j[0], 4, 2.0 * wx, 2.0 * wy);
        }
        Kind::EqRad(r1, r2) => {
            add_rad_grad(v, *r1, 1.0, &mut j[0]);
            add_rad_grad(v, *r2, -1.0, &mut j[0]);
        }
        Kind::Midpoint => {
            j[0][0] += 2.0;
            j[0][2] -= 1.0;
            j[0][4] -= 1.0;
            j[1][1] += 2.0;
            j[1][3] -= 1.0;
            j[1][5] -= 1.0;
        }
        Kind::Symmetric => {
            let (dx, dy) = (v[6] - v[4], v[7] - v[5]);
            let l = (dx * dx + dy * dy).sqrt();
            let (nx, ny) = (dx / l, dy / l);
            let (mx, my) = (0.5 * (v[0] + v[2]), 0.5 * (v[1] + v[3]));
            let (qx, qy) = (mx - v[4], my - v[5]);
            // e0 = cross(n, q): ∂/∂q = (−ny, nx), ∂/∂n = (qy, −qx)
            let (a0x, a0y) = dir_pull(nx, ny, l, qy, -qx);
            put(&mut j[0], 0, -0.5 * ny, 0.5 * nx);
            put(&mut j[0], 2, -0.5 * ny, 0.5 * nx);
            put(&mut j[0], 6, a0x, a0y);
            put(&mut j[0], 4, ny - a0x, -nx - a0y);
            // e1 = (b−a)·n: ∂/∂w = n, ∂/∂n = w
            let (wx, wy) = (v[2] - v[0], v[3] - v[1]);
            let (a1x, a1y) = dir_pull(nx, ny, l, wx, wy);
            put(&mut j[1], 2, nx, ny);
            put(&mut j[1], 0, -nx, -ny);
            put(&mut j[1], 6, a1x, a1y);
            put(&mut j[1], 4, -a1x, -a1y);
        }
        Kind::TanLineCirc { s, r } => {
            let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
            let (ex, ey) = (v[4] - v[0], v[5] - v[1]);
            let l = (dx * dx + dy * dy).sqrt();
            let (nx, ny) = (dx / l, dy / l);
            let (gdx, gdy) = dir_pull(nx, ny, l, ey, -ex);
            let (gex, gey) = (-ny, nx);
            put(&mut j[0], 2, gdx, gdy);
            put(&mut j[0], 4, gex, gey);
            put(&mut j[0], 0, -gdx - gex, -gdy - gey);
            add_rad_grad(v, *r, -s, &mut j[0]);
        }
        Kind::TanCircCirc { s, r1, r2 } => {
            let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
            let sum = {
                let f = |r: &Rad| match r {
                    Rad::Var(i) => v[*i as usize],
                    Rad::Pt { c, s } => {
                        let (c, s) = (*c as usize, *s as usize);
                        ((v[s] - v[c]).powi(2) + (v[s + 1] - v[c + 1]).powi(2)).sqrt()
                    }
                };
                f(r1) + s * f(r2)
            };
            put(&mut j[0], 0, 2.0 * dx, 2.0 * dy);
            put(&mut j[0], 2, -2.0 * dx, -2.0 * dy);
            add_rad_grad(v, *r1, -2.0 * sum, &mut j[0]);
            add_rad_grad(v, *r2, -2.0 * sum * s, &mut j[0]);
        }
        Kind::DistPP => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            put(&mut j[0], 2, 2.0 * ux, 2.0 * uy);
            put(&mut j[0], 0, -2.0 * ux, -2.0 * uy);
            j[0][4] += -2.0 * v[4];
        }
        Kind::DistPL { s } => {
            let (ux, uy) = (v[4] - v[2], v[5] - v[3]);
            let (ex, ey) = (v[0] - v[2], v[1] - v[3]);
            let l = (ux * ux + uy * uy).sqrt();
            let (nx, ny) = (ux / l, uy / l);
            let (gux, guy) = dir_pull(nx, ny, l, ey, -ex);
            let (gex, gey) = (-ny, nx);
            put(&mut j[0], 4, gux, guy);
            put(&mut j[0], 0, gex, gey);
            put(&mut j[0], 2, -gux - gex, -guy - gey);
            j[0][6] += -s;
        }
        Kind::Angle => {
            let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
            let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
            let lu = (ux * ux + uy * uy).sqrt();
            let lw = (wx * wx + wy * wy).sqrt();
            let (n1x, n1y) = (ux / lu, uy / lu);
            let (n2x, n2y) = (wx / lw, wy / lw);
            let th = v[8];
            let (st, ct) = th.sin_cos();
            let sn = n1x * n2y - n1y * n2x;
            let cs = n1x * n2x + n1y * n2y;
            // ∂r/∂n1, ∂r/∂n2
            let g1x = ct * n2y - st * n2x;
            let g1y = ct * (-n2x) - st * n2y;
            let g2x = ct * (-n1y) - st * n1x;
            let g2y = ct * n1x - st * n1y;
            // ∂n/∂u = (I − n nᵀ)/|u|
            let pu = |nx: f64, ny: f64, l: f64, gx: f64, gy: f64| {
                let dot = nx * gx + ny * gy;
                ((gx - nx * dot) / l, (gy - ny * dot) / l)
            };
            let (gux, guy) = pu(n1x, n1y, lu, g1x, g1y);
            let (gwx, gwy) = pu(n2x, n2y, lw, g2x, g2y);
            put(&mut j[0], 2, gux, guy);
            put(&mut j[0], 0, -gux, -guy);
            put(&mut j[0], 6, gwx, gwy);
            put(&mut j[0], 4, -gwx, -gwy);
            j[0][8] += -sn * st - cs * ct;
        }
        Kind::ArcCons => {
            let (ex, ey) = (v[4] - v[0], v[5] - v[1]);
            let (sx, sy) = (v[2] - v[0], v[3] - v[1]);
            put(&mut j[0], 4, 2.0 * ex, 2.0 * ey);
            put(&mut j[0], 2, -2.0 * sx, -2.0 * sy);
            put(&mut j[0], 0, -2.0 * ex + 2.0 * sx, -2.0 * ey + 2.0 * sy);
        }
        Kind::Joint(spec) => joint_jac(spec, v, j),
    }
}

fn joint_jac(spec: &JointSpec, v: &[f64; MAXV], j: &mut Jac) {
    let (_, ta, aa) = spec.a.eval(v);
    let (_, tb, ab) = spec.b.eval(v);
    let mut k = 0;
    if spec.g0 {
        for t in &spec.a.terms {
            let s = t.slot as usize;
            j[0][s] += t.p[0];
            j[0][s + 1] += t.p[1];
            j[1][s] += t.p[2];
            j[1][s + 1] += t.p[3];
        }
        for t in &spec.b.terms {
            let s = t.slot as usize;
            j[0][s] -= t.p[0];
            j[0][s + 1] -= t.p[1];
            j[1][s] -= t.p[2];
            j[1][s + 1] -= t.p[3];
        }
        k = 2;
    }
    if spec.level >= 1 {
        let (tox, toy) = (spec.sign_a * ta.0, spec.sign_a * ta.1);
        let (tpx, tpy) = (spec.sign_b * tb.0, spec.sign_b * tb.1);
        let la = (tox * tox + toy * toy).sqrt();
        let lb = (tpx * tpx + tpy * tpy).sqrt();
        let g = (tox * tpy - toy * tpx) / (la * lb);
        // ∂g/∂To_A, ∂g/∂To_B
        let dax = tpy / (la * lb) - g * tox / (la * la);
        let day = -tpx / (la * lb) - g * toy / (la * la);
        let dbx = -toy / (la * lb) - g * tpx / (lb * lb);
        let dby = tox / (la * lb) - g * tpy / (lb * lb);
        // pull back through T = M·pt
        for t in &spec.a.terms {
            let s = t.slot as usize;
            let (gx, gy) = (spec.sign_a * dax, spec.sign_a * day);
            j[k][s] += gx * t.t[0] + gy * t.t[2];
            j[k][s + 1] += gx * t.t[1] + gy * t.t[3];
        }
        for t in &spec.b.terms {
            let s = t.slot as usize;
            let (gx, gy) = (spec.sign_b * dbx, spec.sign_b * dby);
            j[k][s] += gx * t.t[0] + gy * t.t[2];
            j[k][s + 1] += gx * t.t[1] + gy * t.t[3];
        }
        k += 1;
    }
    if spec.level >= 2 {
        let side =
            |end: &EndLin, t: (f64, f64), a: (f64, f64), sign: f64, row: &mut [f64; MAXV]| {
                let l2 = t.0 * t.0 + t.1 * t.1;
                let l = l2.sqrt();
                let l3 = l2 * l;
                let kappa = (t.0 * a.1 - t.1 * a.0) / l3;
                // ∂κ/∂T, ∂κ/∂A
                let dtx = a.1 / l3 - 3.0 * kappa * t.0 / l2;
                let dty = -a.0 / l3 - 3.0 * kappa * t.1 / l2;
                let dax = -t.1 / l3;
                let day = t.0 / l3;
                for term in &end.terms {
                    let s = term.slot as usize;
                    row[s] += sign
                        * (dtx * term.t[0] + dty * term.t[2] + dax * term.a[0] + day * term.a[2]);
                    row[s + 1] += sign
                        * (dtx * term.t[1] + dty * term.t[3] + dax * term.a[1] + day * term.a[3]);
                }
            };
        side(&spec.a, ta, aa, spec.sign_a, &mut j[k]);
        side(&spec.b, tb, ab, spec.sign_b, &mut j[k]);
    }
}
