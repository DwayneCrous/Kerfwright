use crate::curve::{NurbsCurve, NurbsError};
use crate::hom::{binom, insert_knot_once, Hom};
use crate::knots::KnotVector;
use nalgebra::{Point3, Vector3};

/// Result of a closest-point query on a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceProjection {
    pub u: f64,
    pub v: f64,
    pub point: Point3<f64>,
    pub distance: f64,
}

/// Tensor-product rational B-spline surface. The control net is row-major:
/// index `(i, j) -> i * nv + j`, with `i` along `u` and `j` along `v`.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsSurface {
    knots_u: KnotVector,
    knots_v: KnotVector,
    points: Vec<Point3<f64>>,
    weights: Vec<f64>,
}

impl NurbsSurface {
    pub fn new(
        knots_u: KnotVector,
        knots_v: KnotVector,
        points: Vec<Point3<f64>>,
        weights: Vec<f64>,
    ) -> Result<Self, NurbsError> {
        let expected = knots_u.num_ctrl() * knots_v.num_ctrl();
        if points.len() != expected {
            return Err(NurbsError::ControlPointCount {
                expected,
                got: points.len(),
            });
        }
        if weights.len() != expected {
            return Err(NurbsError::WeightCount {
                expected,
                got: weights.len(),
            });
        }
        if let Some(i) = weights.iter().position(|w| w.is_nan() || *w <= 0.0) {
            return Err(NurbsError::NonPositiveWeight { index: i });
        }
        Ok(Self {
            knots_u,
            knots_v,
            points,
            weights,
        })
    }

    pub fn knots_u(&self) -> &KnotVector {
        &self.knots_u
    }
    pub fn knots_v(&self) -> &KnotVector {
        &self.knots_v
    }
    pub fn points(&self) -> &[Point3<f64>] {
        &self.points
    }
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }
    pub fn num_u(&self) -> usize {
        self.knots_u.num_ctrl()
    }
    pub fn num_v(&self) -> usize {
        self.knots_v.num_ctrl()
    }
    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (self.knots_u.domain(), self.knots_v.domain())
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3<f64> {
        let ((ua, ub), (va, vb)) = self.domain();
        let (u, v) = (u.clamp(ua, ub), v.clamp(va, vb));
        let (p, q) = (self.knots_u.degree(), self.knots_v.degree());
        let nv = self.num_v();
        let su = self.knots_u.find_span(u);
        let sv = self.knots_v.find_span(v);
        let bu = self.knots_u.basis_funs(su, u);
        let bv = self.knots_v.basis_funs(sv, v);
        let mut num = Vector3::zeros();
        let mut den = 0.0;
        for (a, na) in bu.iter().enumerate() {
            for (b, nb) in bv.iter().enumerate() {
                let idx = (su - p + a) * nv + (sv - q + b);
                let w = self.weights[idx] * na * nb;
                num += self.points[idx].coords * w;
                den += w;
            }
        }
        Point3::from(num / den)
    }

    /// Partial derivatives `S_{kl} = ∂^{k+l} S / ∂u^k ∂v^l` for `k + l <= d`
    /// (P&T A3.6 + A4.4). Returned as `out[k][l]`; entries with `k + l > d`
    /// are zero.
    pub fn derivatives(&self, u: f64, v: f64, d: usize) -> Vec<Vec<Vector3<f64>>> {
        let ((ua, ub), (va, vb)) = self.domain();
        let (u, v) = (u.clamp(ua, ub), v.clamp(va, vb));
        let (p, q) = (self.knots_u.degree(), self.knots_v.degree());
        let nv = self.num_v();
        let su = self.knots_u.find_span(u);
        let sv = self.knots_v.find_span(v);
        let nu_d = self.knots_u.ders_basis_funs(su, u, d);
        let nv_d = self.knots_v.ders_basis_funs(sv, v, d);
        let mut aders = vec![vec![Vector3::zeros(); d + 1]; d + 1];
        let mut wders = vec![vec![0.0; d + 1]; d + 1];
        for k in 0..=d.min(p) {
            for l in 0..=(d - k).min(q) {
                for a in 0..=p {
                    for b in 0..=q {
                        let idx = (su - p + a) * nv + (sv - q + b);
                        let w = self.weights[idx] * nu_d[k][a] * nv_d[l][b];
                        aders[k][l] += self.points[idx].coords * w;
                        wders[k][l] += w;
                    }
                }
            }
        }
        let mut skl = vec![vec![Vector3::zeros(); d + 1]; d + 1];
        for k in 0..=d {
            for l in 0..=(d - k) {
                let mut val = aders[k][l];
                for j in 1..=l {
                    val -= skl[k][l - j] * (binom(l, j) * wders[0][j]);
                }
                for i in 1..=k {
                    val -= skl[k - i][l] * (binom(k, i) * wders[i][0]);
                    let mut v2 = Vector3::zeros();
                    for j in 1..=l {
                        v2 += skl[k - i][l - j] * (binom(l, j) * wders[i][j]);
                    }
                    val -= v2 * binom(k, i);
                }
                skl[k][l] = val / wders[0][0];
            }
        }
        skl
    }

    /// Unit normal `Su × Sv / |Su × Sv|`; `None` at singular points.
    pub fn normal_at(&self, u: f64, v: f64) -> Option<Vector3<f64>> {
        let d = self.derivatives(u, v, 1);
        let n = d[1][0].cross(&d[0][1]);
        let len = n.norm();
        if len < 1e-300 {
            None
        } else {
            Some(n / len)
        }
    }

    fn column_homogeneous(&self, j: usize) -> Vec<Hom> {
        let nv = self.num_v();
        (0..self.num_u())
            .map(|i| {
                let idx = i * nv + j;
                let p = self.points[idx];
                let w = self.weights[idx];
                Hom::new(p.x * w, p.y * w, p.z * w, w)
            })
            .collect()
    }

    fn row_homogeneous(&self, i: usize) -> Vec<Hom> {
        let nv = self.num_v();
        (0..nv)
            .map(|j| {
                let idx = i * nv + j;
                let p = self.points[idx];
                let w = self.weights[idx];
                Hom::new(p.x * w, p.y * w, p.z * w, w)
            })
            .collect()
    }

    fn from_hom_grid(
        ku: KnotVector,
        kv: KnotVector,
        grid: &[Hom], // row-major (nu x nv)
    ) -> NurbsSurface {
        let pts = grid
            .iter()
            .map(|h| Point3::new(h.x / h.w, h.y / h.w, h.z / h.w))
            .collect();
        let w = grid.iter().map(|h| h.w).collect();
        NurbsSurface::new(ku, kv, pts, w).expect("insertion preserves validity")
    }

    /// Insert knot `u` (along `u`) up to `times` times.
    pub fn insert_knot_u(&self, u: f64, times: usize) -> NurbsSurface {
        let nv = self.num_v();
        let mut ku = self.knots_u.clone();
        let mut cols: Vec<Vec<Hom>> = (0..nv).map(|j| self.column_homogeneous(j)).collect();
        for _ in 0..times {
            let mut next_k = None;
            let mut next_cols = Vec::with_capacity(nv);
            for col in &cols {
                match insert_knot_once(&ku, col, u) {
                    Some((k, q)) => {
                        next_k = Some(k);
                        next_cols.push(q);
                    }
                    None => return self.rebuild_from_columns(ku, cols),
                }
            }
            ku = next_k.unwrap();
            cols = next_cols;
        }
        self.rebuild_from_columns(ku, cols)
    }

    fn rebuild_from_columns(&self, ku: KnotVector, cols: Vec<Vec<Hom>>) -> NurbsSurface {
        let nu = ku.num_ctrl();
        let nv = cols.len();
        let mut grid = vec![Hom::zeros(); nu * nv];
        for (j, col) in cols.iter().enumerate() {
            for (i, h) in col.iter().enumerate() {
                grid[i * nv + j] = *h;
            }
        }
        Self::from_hom_grid(ku, self.knots_v.clone(), &grid)
    }

    /// Insert knot `v` (along `v`) up to `times` times.
    pub fn insert_knot_v(&self, v: f64, times: usize) -> NurbsSurface {
        let nu = self.num_u();
        let mut kv = self.knots_v.clone();
        let mut rows: Vec<Vec<Hom>> = (0..nu).map(|i| self.row_homogeneous(i)).collect();
        'outer: for _ in 0..times {
            let mut next_k = None;
            let mut next_rows = Vec::with_capacity(nu);
            for row in &rows {
                match insert_knot_once(&kv, row, v) {
                    Some((k, q)) => {
                        next_k = Some(k);
                        next_rows.push(q);
                    }
                    None => break 'outer,
                }
            }
            kv = next_k.unwrap();
            rows = next_rows;
        }
        let grid: Vec<Hom> = rows.into_iter().flatten().collect();
        Self::from_hom_grid(self.knots_u.clone(), kv, &grid)
    }

    /// Curve at fixed `v` (a u-isocurve), by de Boor on the control net.
    pub fn isocurve_v(&self, v: f64) -> NurbsCurve {
        let (va, vb) = self.knots_v.domain();
        let v = v.clamp(va, vb);
        let q = self.knots_v.degree();
        let sv = self.knots_v.find_span(v);
        let bv = self.knots_v.basis_funs(sv, v);
        let nv = self.num_v();
        let hom: Vec<Hom> = (0..self.num_u())
            .map(|i| {
                let mut acc = Hom::zeros();
                for (b, nb) in bv.iter().enumerate() {
                    let idx = i * nv + (sv - q + b);
                    let w = self.weights[idx];
                    let p = self.points[idx];
                    acc += Hom::new(p.x * w, p.y * w, p.z * w, w) * *nb;
                }
                acc
            })
            .collect();
        NurbsCurve::from_homogeneous(self.knots_u.clone(), &hom).expect("valid isocurve")
    }

    /// Curve at fixed `u` (a v-isocurve).
    pub fn isocurve_u(&self, u: f64) -> NurbsCurve {
        let (ua, ub) = self.knots_u.domain();
        let u = u.clamp(ua, ub);
        let p = self.knots_u.degree();
        let su = self.knots_u.find_span(u);
        let bu = self.knots_u.basis_funs(su, u);
        let nv = self.num_v();
        let hom: Vec<Hom> = (0..nv)
            .map(|j| {
                let mut acc = Hom::zeros();
                for (a, na) in bu.iter().enumerate() {
                    let idx = (su - p + a) * nv + j;
                    let w = self.weights[idx];
                    let pt = self.points[idx];
                    acc += Hom::new(pt.x * w, pt.y * w, pt.z * w, w) * *na;
                }
                acc
            })
            .collect();
        NurbsCurve::from_homogeneous(self.knots_v.clone(), &hom).expect("valid isocurve")
    }

    /// Closest point on the surface to `target`: grid sampling for seeds,
    /// then damped 2-D Newton on `(S−P)·S_u = (S−P)·S_v = 0`.
    pub fn closest_point(&self, target: &Point3<f64>) -> SurfaceProjection {
        let ((ua, ub), (va, vb)) = self.domain();
        let nu = 8 * (self.knots_u.distinct().len().max(2) - 1) * (self.knots_u.degree() + 1);
        let nvs = 8 * (self.knots_v.distinct().len().max(2) - 1) * (self.knots_v.degree() + 1);
        let mut samples: Vec<(f64, f64, f64)> = Vec::with_capacity((nu + 1) * (nvs + 1));
        for i in 0..=nu {
            let u = ua + (ub - ua) * i as f64 / nu as f64;
            for j in 0..=nvs {
                let v = va + (vb - va) * j as f64 / nvs as f64;
                samples.push((u, v, (self.point_at(u, v) - target).norm_squared()));
            }
        }
        // grid-local minima as seeds
        let idx = |i: usize, j: usize| i * (nvs + 1) + j;
        let mut seeds: Vec<(f64, f64, f64)> = Vec::new();
        for i in 0..=nu {
            for j in 0..=nvs {
                let c = samples[idx(i, j)].2;
                let mut is_min = true;
                for di in -1i32..=1 {
                    for dj in -1i32..=1 {
                        if di == 0 && dj == 0 {
                            continue;
                        }
                        let (ni, nj) = (i as i32 + di, j as i32 + dj);
                        if ni < 0 || nj < 0 || ni > nu as i32 || nj > nvs as i32 {
                            continue;
                        }
                        if samples[idx(ni as usize, nj as usize)].2 < c {
                            is_min = false;
                        }
                    }
                }
                if is_min {
                    seeds.push(samples[idx(i, j)]);
                }
            }
        }
        seeds.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
        seeds.truncate(6);
        let mut best = SurfaceProjection {
            u: ua,
            v: va,
            point: self.point_at(ua, va),
            distance: f64::INFINITY,
        };
        for (u0, v0, _) in seeds {
            let (mut u, mut v) = (u0, v0);
            for _ in 0..60 {
                let d = self.derivatives(u, v, 2);
                let r = d[0][0] - target.coords;
                let (su, sv) = (d[1][0], d[0][1]);
                let f = [r.dot(&su), r.dot(&sv)];
                let j00 = su.dot(&su) + r.dot(&d[2][0]);
                let j01 = su.dot(&sv) + r.dot(&d[1][1]);
                let j11 = sv.dot(&sv) + r.dot(&d[0][2]);
                let det = j00 * j11 - j01 * j01;
                let (du, dv) = if det.abs() > 1e-300 {
                    (
                        -(j11 * f[0] - j01 * f[1]) / det,
                        -(-j01 * f[0] + j00 * f[1]) / det,
                    )
                } else {
                    (-f[0] / j00.max(1e-300), -f[1] / j11.max(1e-300))
                };
                if !du.is_finite() || !dv.is_finite() {
                    break;
                }
                let un = (u + du).clamp(ua, ub);
                let vn = (v + dv).clamp(va, vb);
                let moved = (un - u).abs() + (vn - v).abs();
                u = un;
                v = vn;
                if moved < 1e-15 {
                    break;
                }
            }
            let pt = self.point_at(u, v);
            let dist = (pt - target).norm();
            if dist < best.distance {
                best = SurfaceProjection {
                    u,
                    v,
                    point: pt,
                    distance: dist,
                };
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn patch() -> NurbsSurface {
        let ku = KnotVector::new(2, vec![0., 0., 0., 0.5, 1., 1., 1.]).unwrap();
        let kv = KnotVector::new(3, vec![0., 0., 0., 0., 1., 1., 1., 1.]).unwrap();
        let mut pts = Vec::new();
        let mut w = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                pts.push(Point3::new(
                    i as f64,
                    j as f64,
                    ((i * 3 + j * 5) % 4) as f64 * 0.3,
                ));
                w.push(1.0 + 0.2 * ((i + 2 * j) % 3) as f64);
            }
        }
        NurbsSurface::new(ku, kv, pts, w).unwrap()
    }

    #[test]
    fn insertion_preserves_surface() {
        let s = patch();
        let t = s
            .insert_knot_u(0.25, 1)
            .insert_knot_v(0.4, 2)
            .insert_knot_u(0.5, 1);
        for i in 0..=10 {
            for j in 0..=10 {
                let (u, v) = (i as f64 / 10.0, j as f64 / 10.0);
                assert!((s.point_at(u, v) - t.point_at(u, v)).norm() < 1e-12);
            }
        }
        assert!(t.num_u() > s.num_u() && t.num_v() > s.num_v());
    }

    #[test]
    fn derivatives_match_finite_differences() {
        let s = patch();
        let h = 1e-5;
        for &(u, v) in &[(0.2, 0.3), (0.7, 0.8), (0.45, 0.55)] {
            let d = s.derivatives(u, v, 2);
            let su = (s.point_at(u + h, v).coords - s.point_at(u - h, v).coords) / (2.0 * h);
            let sv = (s.point_at(u, v + h).coords - s.point_at(u, v - h).coords) / (2.0 * h);
            assert!((d[1][0] - su).norm() / su.norm() < 1e-6);
            assert!((d[0][1] - sv).norm() / sv.norm() < 1e-6);
            let suv =
                (s.derivatives(u, v + h, 1)[1][0] - s.derivatives(u, v - h, 1)[1][0]) / (2.0 * h);
            assert!((d[1][1] - suv).norm() / suv.norm().max(1e-9) < 1e-5);
        }
    }

    #[test]
    fn isocurves_match_surface() {
        let s = patch();
        let c = s.isocurve_v(0.37);
        let c2 = s.isocurve_u(0.61);
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            assert!((c.point_at(t) - s.point_at(t, 0.37)).norm() < 1e-12);
            assert!((c2.point_at(t) - s.point_at(0.61, t)).norm() < 1e-12);
        }
    }

    #[test]
    fn closest_point_beats_dense_grid() {
        let s = patch();
        let target = Point3::new(1.3, 1.7, 2.0);
        let r = s.closest_point(&target);
        let mut best = f64::INFINITY;
        for i in 0..=300 {
            for j in 0..=300 {
                best = best.min((s.point_at(i as f64 / 300.0, j as f64 / 300.0) - target).norm());
            }
        }
        assert!(r.distance <= best + 1e-12, "{} vs {}", r.distance, best);
    }
}
