use crate::curve::{averaged_knots, chord_length_params, greville, NurbsCurve, NurbsError};
use crate::hom::{binom, insert_knot_once, Hom};
use crate::knots::KnotVector;
use nalgebra::{DMatrix, Point3, Vector3};

/// Result of a closest-point query on a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceProjection {
    pub u: f64,
    pub v: f64,
    pub point: Point3<f64>,
    pub distance: f64,
}

/// Which end of which parameter a surface is continued at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtendDir {
    UStart,
    UEnd,
    VStart,
    VEnd,
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

/// Points of a surface on a regular grid of its parameters, for finding the nearest point of the surface
/// to a given point by Newton's method from the nearest sample (see [`NurbsSurface::project_in_grid`]).
#[derive(Debug, Clone)]
pub struct SurfaceGrid {
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
    n: usize,
    pts: Vec<Point3<f64>>,
    /// The largest distance between neighbouring samples.
    pub spacing: f64,
}

impl SurfaceGrid {
    /// The sample nearest to `p`: its distance and parameters.
    pub fn nearest(&self, p: &Point3<f64>) -> (f64, f64, f64) {
        let (mut best, mut at) = (f64::INFINITY, 0usize);
        for (k, q) in self.pts.iter().enumerate() {
            let d = (q - p).norm_squared();
            if d < best {
                best = d;
                at = k;
            }
        }
        let (i, j) = (at / (self.n + 1), at % (self.n + 1));
        (
            best.sqrt(),
            self.u0 + (self.u1 - self.u0) * i as f64 / self.n as f64,
            self.v0 + (self.v1 - self.v0) * j as f64 / self.n as f64,
        )
    }
}

impl NurbsSurface {
    /// Samples on an `n × n` grid of cells over the parameter box `u`, `v` (clamped to the domain).
    pub fn sample_grid(&self, u: (f64, f64), v: (f64, f64), n: usize) -> SurfaceGrid {
        let ((da, db), (dc, dd)) = self.domain();
        let (u0, u1) = (u.0.max(da), u.1.min(db));
        let (v0, v1) = (v.0.max(dc), v.1.min(dd));
        let mut pts = Vec::with_capacity((n + 1) * (n + 1));
        for i in 0..=n {
            let uu = u0 + (u1 - u0) * i as f64 / n as f64;
            for j in 0..=n {
                let vv = v0 + (v1 - v0) * j as f64 / n as f64;
                pts.push(self.point_at(uu, vv));
            }
        }
        let mut spacing = 0.0f64;
        for i in 0..=n {
            for j in 0..=n {
                let p = pts[i * (n + 1) + j];
                if i < n {
                    spacing = spacing.max((pts[(i + 1) * (n + 1) + j] - p).norm());
                }
                if j < n {
                    spacing = spacing.max((pts[i * (n + 1) + j + 1] - p).norm());
                }
            }
        }
        SurfaceGrid {
            u0,
            u1,
            v0,
            v1,
            n,
            pts,
            spacing,
        }
    }

    /// The point of the surface nearest to `target`, searched from the nearest sample of `grid` only (the
    /// answer is the local minimum there, which is the global one for a point near the surface and a
    /// surface that does not fold back on itself within a few sample spacings).
    pub fn project_in_grid(&self, grid: &SurfaceGrid, target: &Point3<f64>) -> SurfaceProjection {
        let (_, u, v) = grid.nearest(target);
        self.closest_point_seeded(target, u, v)
    }

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

    /// Global tensor-product interpolation of a grid of points: `grid[i][j]` is the point at the
    /// parameters `(u_i, v_j)`, the surface has degree `du` by `dv`, and its domain is `[0, 1]²`.
    /// The parameters are chord lengths averaged over the rows and columns (Piegl & Tiller A9.4).
    pub fn interpolate(
        grid: &[Vec<Point3<f64>>],
        du: usize,
        dv: usize,
    ) -> Result<NurbsSurface, NurbsError> {
        Self::interpolate_with_params(grid, du, dv).map(|(s, _, _)| s)
    }

    /// [`interpolate`](Self::interpolate), also returning the parameters `u_i` and `v_j` at which the
    /// surface passes through the grid points.
    pub fn interpolate_with_params(
        grid: &[Vec<Point3<f64>>],
        du: usize,
        dv: usize,
    ) -> Result<(NurbsSurface, Vec<f64>, Vec<f64>), NurbsError> {
        let (nu, nv) = Self::grid_size(grid)?;
        let mut pu = vec![0.0; nu];
        for j in 0..nv {
            let col: Vec<Point3<f64>> = (0..nu).map(|i| grid[i][j]).collect();
            for (a, b) in pu.iter_mut().zip(chord_length_params(&col)) {
                *a += b / nv as f64;
            }
        }
        let mut pv = vec![0.0; nv];
        for row in grid {
            for (a, b) in pv.iter_mut().zip(chord_length_params(row)) {
                *a += b / nu as f64;
            }
        }
        let s = Self::interpolate_at(grid, &pu, &pv, du, dv)?;
        Ok((s, pu, pv))
    }

    /// The surface on the clamped knot vectors that passes through `f` at the Greville abscissae in both
    /// directions (collocation): a fit that keeps the parameters of `f`, with breaks where the knot
    /// vectors have them.
    pub fn collocate(
        ku: KnotVector,
        kv: KnotVector,
        f: &dyn Fn(f64, f64) -> Point3<f64>,
    ) -> Result<NurbsSurface, NurbsError> {
        let (gu, gv) = (greville(&ku), greville(&kv));
        let grid: Vec<Vec<Point3<f64>>> = gu
            .iter()
            .map(|u| gv.iter().map(|v| f(*u, *v)).collect())
            .collect();
        let (du, dv) = (ku.degree(), kv.degree());
        let matrix = |k: &KnotVector, g: &[f64], d: usize| {
            let n = g.len();
            let mut a = DMatrix::<f64>::zeros(n, n);
            for (r, &t) in g.iter().enumerate() {
                let span = k.find_span(t);
                for (j, b) in k.basis_funs(span, t).iter().enumerate() {
                    a[(r, span - d + j)] = *b;
                }
            }
            a.lu()
        };
        let (nu, nv) = (gu.len(), gv.len());
        let (lu_u, lu_v) = (matrix(&ku, &gu, du), matrix(&kv, &gv, dv));
        let mut mid = vec![Point3::origin(); nu * nv];
        for j in 0..nv {
            let rhs = DMatrix::from_fn(nu, 3, |i, c| grid[i][j][c]);
            let sol = lu_u.solve(&rhs).ok_or(NurbsError::SingularSystem)?;
            for i in 0..nu {
                mid[i * nv + j] = Point3::new(sol[(i, 0)], sol[(i, 1)], sol[(i, 2)]);
            }
        }
        let mut pts = vec![Point3::origin(); nu * nv];
        for i in 0..nu {
            let rhs = DMatrix::from_fn(nv, 3, |j, c| mid[i * nv + j][c]);
            let sol = lu_v.solve(&rhs).ok_or(NurbsError::SingularSystem)?;
            for j in 0..nv {
                pts[i * nv + j] = Point3::new(sol[(j, 0)], sol[(j, 1)], sol[(j, 2)]);
            }
        }
        NurbsSurface::new(ku, kv, pts, vec![1.0; nu * nv])
    }

    /// Like [`interpolate`](Self::interpolate) but with evenly spaced parameters, so that a boundary
    /// row or column depends only on its own points: two surfaces sharing a boundary sampled the same
    /// way share that boundary curve exactly.
    pub fn interpolate_uniform(
        grid: &[Vec<Point3<f64>>],
        du: usize,
        dv: usize,
    ) -> Result<NurbsSurface, NurbsError> {
        let (nu, nv) = Self::grid_size(grid)?;
        let even = |n: usize| {
            (0..n)
                .map(|i| i as f64 / (n - 1) as f64)
                .collect::<Vec<_>>()
        };
        Self::interpolate_at(grid, &even(nu), &even(nv), du, dv)
    }

    fn grid_size(grid: &[Vec<Point3<f64>>]) -> Result<(usize, usize), NurbsError> {
        let nu = grid.len();
        let nv = grid.first().map_or(0, |r| r.len());
        if grid.iter().any(|r| r.len() != nv) {
            return Err(NurbsError::InvalidArgument(
                "the grid rows differ in length",
            ));
        }
        Ok((nu, nv))
    }

    fn interpolate_at(
        grid: &[Vec<Point3<f64>>],
        pu: &[f64],
        pv: &[f64],
        du: usize,
        dv: usize,
    ) -> Result<NurbsSurface, NurbsError> {
        let (nu, nv) = (pu.len(), pv.len());
        if du == 0 || dv == 0 {
            return Err(NurbsError::InvalidArgument("degree"));
        }
        if nu < du + 1 || nv < dv + 1 {
            return Err(NurbsError::NotEnoughPoints {
                need: (du + 1).max(dv + 1),
                got: nu.min(nv),
            });
        }
        let ku = KnotVector::new(du, averaged_knots(pu, du))
            .map_err(|_| NurbsError::InvalidArgument("knots"))?;
        let kv = KnotVector::new(dv, averaged_knots(pv, dv))
            .map_err(|_| NurbsError::InvalidArgument("knots"))?;
        let matrix = |k: &KnotVector, p: &[f64], d: usize| {
            let n = p.len();
            let mut a = DMatrix::<f64>::zeros(n, n);
            for (r, &t) in p.iter().enumerate() {
                let span = k.find_span(t);
                for (j, b) in k.basis_funs(span, t).iter().enumerate() {
                    a[(r, span - d + j)] = *b;
                }
            }
            a.lu()
        };
        let (lu_u, lu_v) = (matrix(&ku, pu, du), matrix(&kv, pv, dv));
        // first along u for every column of the grid, then along v for every row of the result
        let mut mid = vec![Point3::origin(); nu * nv];
        for j in 0..nv {
            let rhs = DMatrix::from_fn(nu, 3, |i, c| grid[i][j][c]);
            let sol = lu_u.solve(&rhs).ok_or(NurbsError::SingularSystem)?;
            for i in 0..nu {
                mid[i * nv + j] = Point3::new(sol[(i, 0)], sol[(i, 1)], sol[(i, 2)]);
            }
        }
        let mut pts = vec![Point3::origin(); nu * nv];
        for i in 0..nu {
            let rhs = DMatrix::from_fn(nv, 3, |j, c| mid[i * nv + j][c]);
            let sol = lu_v.solve(&rhs).ok_or(NurbsError::SingularSystem)?;
            for j in 0..nv {
                pts[i * nv + j] = Point3::new(sol[(j, 0)], sol[(j, 1)], sol[(j, 2)]);
            }
        }
        NurbsSurface::new(ku, kv, pts, vec![1.0; nu * nv])
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

    /// The surface with its two parameters swapped.
    pub fn transposed(&self) -> NurbsSurface {
        let (nu, nv) = (self.num_u(), self.num_v());
        let mut pts = Vec::with_capacity(nu * nv);
        let mut ws = Vec::with_capacity(nu * nv);
        for j in 0..nv {
            for i in 0..nu {
                pts.push(self.points[i * nv + j]);
                ws.push(self.weights[i * nv + j]);
            }
        }
        NurbsSurface::new(self.knots_v.clone(), self.knots_u.clone(), pts, ws)
            .expect("a transposed surface is valid")
    }

    /// The surface with `v` running the other way (`v -> a + b - v`).
    pub fn reversed_v(&self) -> NurbsSurface {
        let (a, b) = self.knots_v.domain();
        self.reflected_v(a + b)
    }

    /// The surface mirrored in `v` about `sum / 2` (`v -> sum - v`).
    fn reflected_v(&self, sum: f64) -> NurbsSurface {
        let (nu, nv) = (self.num_u(), self.num_v());
        let ks: Vec<f64> = self
            .knots_v
            .as_slice()
            .iter()
            .rev()
            .map(|k| sum - k)
            .collect();
        let kv = KnotVector::new(self.knots_v.degree(), ks).expect("reversed knots are valid");
        let mut pts = Vec::with_capacity(nu * nv);
        let mut ws = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            for j in (0..nv).rev() {
                pts.push(self.points[i * nv + j]);
                ws.push(self.weights[i * nv + j]);
            }
        }
        NurbsSurface::new(self.knots_u.clone(), kv, pts, ws).expect("a reversed surface is valid")
    }

    /// The surface continued past the end of its `v` domain by `delta`: the last polynomial piece is
    /// carried on, so the part that was there is unchanged (same parameters) and the new part is the
    /// natural continuation of the surface. Fails if a rational surface's weight would turn negative.
    pub fn extended_v_end(&self, delta: f64) -> Result<NurbsSurface, NurbsError> {
        if delta.is_nan() || delta <= 0.0 {
            return Err(NurbsError::InvalidArgument("extension"));
        }
        let p = self.knots_v.degree();
        let (_, b) = self.knots_v.domain();
        // the last knot before the end: the start of the last polynomial piece
        let distinct = self.knots_v.distinct();
        let (a_l, mult) = if distinct.len() >= 2 {
            distinct[distinct.len() - 2]
        } else {
            return Err(NurbsError::InvalidArgument("a surface without a span"));
        };
        let s = if mult < p && a_l > self.knots_v.domain().0 {
            self.insert_knot_v(a_l, p - mult)
        } else {
            self.clone()
        };
        let (nu, nv) = (s.num_u(), s.num_v());
        let c = (b + delta - a_l) / (b - a_l);
        let mut grid: Vec<Hom> = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            let mut row = s.row_homogeneous(i);
            let base = nv - p - 1;
            let bez: Vec<Hom> = row[base..].to_vec();
            // the Bezier points of the same polynomial over [a_l, b + delta]: blossom at (0^(p-k), c^k)
            for k in 1..=p {
                let mut pts = bez.clone();
                let params: Vec<f64> = std::iter::repeat_n(c, k)
                    .chain(std::iter::repeat_n(0.0, p - k))
                    .collect();
                for r in params {
                    let n = pts.len();
                    pts = (0..n - 1)
                        .map(|j| pts[j] * (1.0 - r) + pts[j + 1] * r)
                        .collect();
                }
                row[base + k] = pts[0];
            }
            grid.extend(row);
        }
        let mut ks = s.knots_v.as_slice().to_vec();
        let n = ks.len();
        for k in ks.iter_mut().skip(n - p - 1) {
            *k = b + delta;
        }
        let kv = KnotVector::new(p, ks).map_err(|_| NurbsError::InvalidArgument("knots"))?;
        let pts: Vec<Point3<f64>> = grid
            .iter()
            .map(|h| Point3::new(h.x / h.w, h.y / h.w, h.z / h.w))
            .collect();
        let ws: Vec<f64> = grid.iter().map(|h| h.w).collect();
        NurbsSurface::new(s.knots_u.clone(), kv, pts, ws)
    }

    /// The surface continued by `delta` past the start or the end of its `u` or `v` domain.
    pub fn extended(&self, dir: ExtendDir, delta: f64) -> Result<NurbsSurface, NurbsError> {
        match dir {
            ExtendDir::VEnd => self.extended_v_end(delta),
            ExtendDir::VStart => {
                let (a, b) = self.knots_v.domain();
                Ok(self
                    .reflected_v(a + b)
                    .extended_v_end(delta)?
                    .reflected_v(a + b))
            }
            ExtendDir::UEnd => Ok(self.transposed().extended_v_end(delta)?.transposed()),
            ExtendDir::UStart => {
                let (a, b) = self.knots_u.domain();
                Ok(self
                    .transposed()
                    .reflected_v(a + b)
                    .extended_v_end(delta)?
                    .reflected_v(a + b)
                    .transposed())
            }
        }
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

    /// Damped 2-D Newton on `(S−P)·S_u = (S−P)·S_v = 0` from the parameters `(u0, v0)`.
    fn newton_closest(&self, target: &Point3<f64>, u0: f64, v0: f64) -> (SurfaceProjection, bool) {
        let ((ua, ub), (va, vb)) = self.domain();
        let (mut u, mut v) = (u0.clamp(ua, ub), v0.clamp(va, vb));
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
            // a step that leaves the domain in one direction is redone in the other alone (the minimum of
            // the distance on the boundary line)
            let out_u = u + du < ua || u + du > ub;
            let out_v = v + dv < va || v + dv > vb;
            let (du, dv) = match (out_u, out_v) {
                (true, false) => (0.0, -f[1] / j11.abs().max(1e-300) * j11.signum()),
                (false, true) => (-f[0] / j00.abs().max(1e-300) * j00.signum(), 0.0),
                _ => (du, dv),
            };
            let un = (u + du).clamp(ua, ub);
            let vn = (v + dv).clamp(va, vb);
            let moved = (un - u).abs() + (vn - v).abs();
            u = un;
            v = vn;
            if moved < 1e-15 {
                break;
            }
        }
        // converged when the offset to the target is (nearly) perpendicular to the surface at (u, v) or
        // the parameters sit on the boundary of the domain with the offset pointing outwards
        let d = self.derivatives(u, v, 1);
        let r = d[0][0] - target.coords;
        let (su, sv) = (d[1][0], d[0][1]);
        let rn = r.norm();
        let small = |g: f64, s: &Vector3<f64>| g.abs() <= 1e-7 * rn * s.norm().max(1e-300);
        let at_edge_u = u <= ua + 1e-12 || u >= ub - 1e-12;
        let at_edge_v = v <= va + 1e-12 || v >= vb - 1e-12;
        let converged = rn < 1e-7
            || ((small(r.dot(&su), &su) || at_edge_u) && (small(r.dot(&sv), &sv) || at_edge_v));
        let point = self.point_at(u, v);
        (
            SurfaceProjection {
                u,
                v,
                point,
                distance: (point - target).norm(),
            },
            converged,
        )
    }

    /// The nearest point found by Newton from `(u0, v0)` without ever searching the whole surface: the
    /// local minimum near the seed (or the seed's own point if that is nearer). For seeds known to be close
    /// to the answer, e.g. the nearest sample of a grid.
    pub fn closest_point_seeded(
        &self,
        target: &Point3<f64>,
        u0: f64,
        v0: f64,
    ) -> SurfaceProjection {
        let ((ua, ub), (va, vb)) = self.domain();
        let (u0, v0) = (u0.clamp(ua, ub), v0.clamp(va, vb));
        let (local, _) = self.newton_closest(target, u0, v0);
        let seed = self.point_at(u0, v0);
        let d0 = (seed - target).norm();
        if local.distance <= d0 {
            local
        } else {
            SurfaceProjection {
                u: u0,
                v: v0,
                point: seed,
                distance: d0,
            }
        }
    }

    /// The nearest point found by Newton from the parameters `(u0, v0)`: a local answer, never farther than
    /// the surface point at the seed, and much cheaper than [`closest_point`](Self::closest_point) when the
    /// seed is close (an upper bound on the distance to the surface).
    pub fn closest_point_near(&self, target: &Point3<f64>, u0: f64, v0: f64) -> SurfaceProjection {
        let ((ua, ub), (va, vb)) = self.domain();
        let (u0, v0) = (u0.clamp(ua, ub), v0.clamp(va, vb));
        let (local, converged) = self.newton_closest(target, u0, v0);
        let seed = self.point_at(u0, v0);
        let d0 = (seed - target).norm();
        if converged && local.distance <= d0 {
            local
        } else if !converged {
            // the seed was no good: search the whole surface
            self.closest_point(target)
        } else {
            SurfaceProjection {
                u: u0,
                v: v0,
                point: seed,
                distance: d0,
            }
        }
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
            let (cand, _) = self.newton_closest(target, u0, v0);
            if cand.distance < best.distance {
                best = cand;
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
    fn a_seeded_search_finds_the_same_point_as_the_whole_surface() {
        let s = patch();
        let grid = s.sample_grid((0.0, 1.0), (0.0, 1.0), 24);
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..200 {
            // a point a little off the surface
            let p = s.point_at(rnd(), rnd());
            let q = Point3::new(
                p.x + 0.2 * (rnd() - 0.5),
                p.y + 0.2 * (rnd() - 0.5),
                p.z + 0.2 * (rnd() - 0.5),
            );
            let global = s.closest_point(&q);
            let local = s.project_in_grid(&grid, &q);
            assert!(
                local.distance <= global.distance + 1e-9,
                "{} vs {}",
                local.distance,
                global.distance
            );
            // never worse than the nearest sample
            let (d, u, v) = grid.nearest(&q);
            assert!(local.distance <= d + 1e-12 && (s.point_at(u, v) - q).norm() <= d + 1e-12);
            // and with a good seed the nearby search agrees
            let near = s.closest_point_near(&q, global.u + 0.01, global.v - 0.01);
            assert!((near.distance - global.distance).abs() < 1e-9);
        }
        // a far seed gives a point of the surface no nearer than the true nearest one
        let q = Point3::new(1.3, 1.7, 2.0);
        let far = s.closest_point_near(&q, 0.0, 0.0);
        assert!(far.distance >= s.closest_point(&q).distance - 1e-9);
        assert!(grid.spacing > 0.0);
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
