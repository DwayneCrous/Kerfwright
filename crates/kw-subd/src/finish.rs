use crate::level::{face_grid, Level, P3};
use crate::mesh::{SubdError, SubdMesh};
use kw_brep::{BRep, BodyId, BodyKind, CoedgeSpec, EdgeId, LoopKind, Sense, VertexId};
use kw_math::{Matrix4, Point3, Vector3};
use kw_nurbs::{CurveGeom, KnotVector, NurbsCurve, NurbsSurface, SurfaceGeom};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct FinishOptions {
    /// Largest allowed distance between a patch and the limit surface (mm).
    pub tolerance: f64,
    pub min_level: usize,
    pub max_level: usize,
    /// Extra levels used to sample the limit surface for each patch (`2^m + 1` samples per side).
    pub fit_levels: usize,
    pub max_patches: usize,
    /// Keep refining while the largest tangent-plane mismatch across a smooth edge exceeds this many
    /// degrees (the first level within the distance tolerance is used if no level gets there).
    pub g1_tolerance_deg: f64,
    /// Weight of the C1 coupling between neighbouring patches in the global fit (0 = independent fits).
    pub smoothness: f64,
}

impl Default for FinishOptions {
    fn default() -> Self {
        FinishOptions {
            tolerance: 0.01,
            min_level: 1,
            max_level: 6,
            fit_levels: 3,
            max_patches: 60_000,
            g1_tolerance_deg: 1.5,
            smoothness: 0.0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct G1Report {
    pub edges_checked: usize,
    /// Shared edges that are creases in the limit (not smooth, so not checked).
    pub crease_edges: usize,
    pub max_angle_deg: f64,
    pub mean_angle_deg: f64,
}

#[derive(Debug)]
pub struct Finished {
    pub rep: BRep,
    pub body: BodyId,
    pub level: usize,
    pub patches: usize,
    /// For every patch (= every face of the body, in order): the cage face it lies under and its path
    /// of quarter-subdivisions below that face, e.g. `(3, "102")`.
    pub patch_paths: Vec<(u32, String)>,
    /// Largest measured distance from a patch to the limit surface.
    pub deviation: f64,
    pub g1: G1Report,
}

type Patch = [[P3; 4]; 4];

fn bern(t: f64) -> [f64; 4] {
    let s = 1.0 - t;
    [s * s * s, 3.0 * t * s * s, 3.0 * t * t * s, t * t * t]
}
fn dbern(t: f64) -> [f64; 4] {
    let s = 1.0 - t;
    [
        -3.0 * s * s,
        3.0 * s * s - 6.0 * t * s,
        6.0 * t * s - 3.0 * t * t,
        3.0 * t * t,
    ]
}

fn eval(p: &Patch, u: f64, v: f64) -> P3 {
    let (bu, bv) = (bern(u), bern(v));
    let mut s = [0.0; 3];
    for i in 0..4 {
        for j in 0..4 {
            let w = bu[i] * bv[j];
            for a in 0..3 {
                s[a] += w * p[i][j][a];
            }
        }
    }
    s
}

fn normal(p: &Patch, u: f64, v: f64) -> Option<Vector3<f64>> {
    let (bu, bv, du, dv) = (bern(u), bern(v), dbern(u), dbern(v));
    let (mut su, mut sv) = (Vector3::zeros(), Vector3::zeros());
    for i in 0..4 {
        for j in 0..4 {
            let q = Vector3::new(p[i][j][0], p[i][j][1], p[i][j][2]);
            su += q * (du[i] * bv[j]);
            sv += q * (bu[i] * dv[j]);
        }
    }
    let n = su.cross(&sv);
    let l = n.norm();
    (l > 1e-12).then(|| n / l)
}

fn dist(a: P3, b: P3) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The cubic Bézier through `s[0]` and `s[n]` that best fits the equally spaced samples.
fn fit_edge(s: &[P3]) -> [P3; 4] {
    let n = s.len() - 1;
    let (p0, p3) = (s[0], s[n]);
    let (mut a11, mut a12, mut a22) = (0.0, 0.0, 0.0);
    let (mut r1, mut r2) = ([0.0; 3], [0.0; 3]);
    for (k, x) in s.iter().enumerate() {
        let t = k as f64 / n as f64;
        let b = bern(t);
        a11 += b[1] * b[1];
        a12 += b[1] * b[2];
        a22 += b[2] * b[2];
        for a in 0..3 {
            let rest = x[a] - b[0] * p0[a] - b[3] * p3[a];
            r1[a] += b[1] * rest;
            r2[a] += b[2] * rest;
        }
    }
    let det = a11 * a22 - a12 * a12;
    let mut p1 = [0.0; 3];
    let mut p2 = [0.0; 3];
    for a in 0..3 {
        p1[a] = (a22 * r1[a] - a12 * r2[a]) / det;
        p2[a] = (a11 * r2[a] - a12 * r1[a]) / det;
    }
    [p0, p1, p2, p3]
}

fn key(a: u32, b: u32) -> (u32, u32) {
    (a.min(b), a.max(b))
}

/// Build the patches of level `k` from limit samples `m` levels below.
struct Fit {
    patches: Vec<Patch>,
    /// Canonical (low → high vertex) cubic of every edge.
    edges: HashMap<(u32, u32), [P3; 4]>,
}

fn fit_level(levels: &[Level], limits: &[Option<Vec<P3>>], k: usize, m: usize) -> Fit {
    let lim = limits[k + m].as_ref().unwrap();
    let nn = 1usize << m;
    let faces = &levels[k].faces;
    let grids: Vec<Vec<Vec<u32>>> = (0..faces.len())
        .map(|f| face_grid(levels, k, f, m))
        .collect();
    // shared edge curves
    let mut edges: HashMap<(u32, u32), [P3; 4]> = HashMap::new();
    for (fi, f) in faces.iter().enumerate() {
        let g = &grids[fi];
        for side in 0..4 {
            let (va, vb) = (f[side], f[(side + 1) % 4]);
            let kk = key(va, vb);
            if edges.contains_key(&kk) {
                continue;
            }
            let idx: Vec<u32> = (0..=nn)
                .map(|q| match side {
                    0 => g[q][0],
                    1 => g[nn][q],
                    2 => g[nn - q][nn],
                    _ => g[0][nn - q],
                })
                .collect();
            let mut s: Vec<P3> = idx.iter().map(|&i| lim[i as usize]).collect();
            if va > vb {
                s.reverse();
            }
            edges.insert(kk, fit_edge(&s));
        }
    }
    // the interior least-squares system is the same for every patch
    let w_of = |u: f64, v: f64| -> [f64; 4] {
        let (bu, bv) = (bern(u), bern(v));
        [bu[1] * bv[1], bu[1] * bv[2], bu[2] * bv[1], bu[2] * bv[2]]
    };
    let mut mat = Matrix4::<f64>::zeros();
    for i in 0..=nn {
        for j in 0..=nn {
            let w = w_of(i as f64 / nn as f64, j as f64 / nn as f64);
            for a in 0..4 {
                for b in 0..4 {
                    mat[(a, b)] += w[a] * w[b];
                }
            }
        }
    }
    let inv = mat.try_inverse().expect("the patch fit matrix is singular");
    let patches: Vec<Patch> = (0..faces.len())
        .map(|fi| {
            let f = &faces[fi];
            let g = &grids[fi];
            let side_pts = |side: usize| -> [P3; 4] {
                let (va, vb) = (f[side], f[(side + 1) % 4]);
                let mut c = edges[&key(va, vb)];
                if va > vb {
                    c.reverse();
                }
                c
            };
            let (s0, s1, s2, s3) = (side_pts(0), side_pts(1), side_pts(2), side_pts(3));
            let mut p: Patch = [[[0.0; 3]; 4]; 4];
            for q in 0..4 {
                p[q][0] = s0[q];
                p[3][q] = s1[q];
                p[3 - q][3] = s2[q];
                p[0][3 - q] = s3[q];
            }
            // interior
            let mut rhs = [[0.0; 3]; 4];
            for i in 0..=nn {
                for j in 0..=nn {
                    let (u, v) = (i as f64 / nn as f64, j as f64 / nn as f64);
                    let (bu, bv) = (bern(u), bern(v));
                    let x = lim[g[i][j] as usize];
                    let mut r = x;
                    for a in 0..4 {
                        for b in 0..4 {
                            if a == 0 || a == 3 || b == 0 || b == 3 {
                                for c in 0..3 {
                                    r[c] -= bu[a] * bv[b] * p[a][b][c];
                                }
                            }
                        }
                    }
                    let w = w_of(u, v);
                    for a in 0..4 {
                        for c in 0..3 {
                            rhs[a][c] += w[a] * r[c];
                        }
                    }
                }
            }
            for c in 0..3 {
                let col = inv * nalgebra_vec(&[rhs[0][c], rhs[1][c], rhs[2][c], rhs[3][c]]);
                p[1][1][c] = col[0];
                p[1][2][c] = col[1];
                p[2][1][c] = col[2];
                p[2][2][c] = col[3];
            }
            p
        })
        .collect();
    Fit { patches, edges }
}

fn nalgebra_vec(v: &[f64; 4]) -> kw_math::Vector4<f64> {
    kw_math::Vector4::new(v[0], v[1], v[2], v[3])
}

#[derive(Clone, Copy)]
enum CRef {
    Const(P3),
    Var(usize),
}

/// Refit all free control points together: least squares to the limit samples of every patch plus a
/// C1 coupling (`λ·(inner row A + inner row B − 2·edge) = 0`) across every smooth edge. Away from
/// extraordinary vertices the limit surface satisfies the coupling exactly, so the fit loses nothing
/// there and the patches meet with matching tangent planes.
fn refine_c1(levels: &[Level], lim: &[P3], fit: &mut Fit, k: usize, m: usize, lambda: f64) {
    let faces = &levels[k].faces;
    let nn = 1usize << m;
    // variables: two inner points per edge, four interior points per patch
    let mut keys: Vec<(u32, u32)> = fit.edges.keys().copied().collect();
    keys.sort_unstable();
    let mut edge_var: HashMap<(u32, u32), usize> = HashMap::new();
    for (i, kk) in keys.iter().enumerate() {
        edge_var.insert(*kk, 2 * i);
    }
    let patch_base = 2 * keys.len();
    let nvar = patch_base + 4 * faces.len();
    let mut x: Vec<P3> = vec![[0.0; 3]; nvar];
    for (kk, c) in &fit.edges {
        let b = edge_var[kk];
        x[b] = c[1];
        x[b + 1] = c[2];
    }
    for (fi, p) in fit.patches.iter().enumerate() {
        let b = patch_base + 4 * fi;
        x[b] = p[1][1];
        x[b + 1] = p[1][2];
        x[b + 2] = p[2][1];
        x[b + 3] = p[2][2];
    }
    let edge_point = |kk: (u32, u32), c: usize| -> CRef {
        match c {
            0 | 3 => CRef::Const(fit.edges[&kk][c]),
            _ => CRef::Var(edge_var[&kk] + c - 1),
        }
    };
    // control point (a, b) of patch fi
    let ctrl = |fi: usize, a: usize, b: usize| -> CRef {
        let f = &faces[fi];
        let side_ref = |side: usize, q: usize| -> CRef {
            let (va, vb) = (f[side], f[(side + 1) % 4]);
            let c = if va < vb { q } else { 3 - q };
            edge_point(key(va, vb), c)
        };
        match (a, b) {
            (a, 0) => side_ref(0, a),
            (3, b) => side_ref(1, b),
            (a, 3) => side_ref(2, 3 - a),
            (0, b) => side_ref(3, 3 - b),
            (1, 1) => CRef::Var(patch_base + 4 * fi),
            (1, 2) => CRef::Var(patch_base + 4 * fi + 1),
            (2, 1) => CRef::Var(patch_base + 4 * fi + 2),
            _ => CRef::Var(patch_base + 4 * fi + 3),
        }
    };
    struct Row {
        terms: Vec<(usize, f64)>,
        rhs: P3,
    }
    let mut rows: Vec<Row> = Vec::new();
    let mut push_row = |terms: &[(CRef, f64)], target: P3| {
        let mut r = Row {
            terms: Vec::new(),
            rhs: target,
        };
        for (c, w) in terms {
            match c {
                CRef::Const(p) => {
                    for a in 0..3 {
                        r.rhs[a] -= w * p[a];
                    }
                }
                CRef::Var(i) => r.terms.push((*i, *w)),
            }
        }
        rows.push(r);
    };
    for fi in 0..faces.len() {
        let g = face_grid(levels, k, fi, m);
        for i in 0..=nn {
            for j in 0..=nn {
                let (bu, bv) = (bern(i as f64 / nn as f64), bern(j as f64 / nn as f64));
                let mut terms = Vec::with_capacity(16);
                for a in 0..4 {
                    for b in 0..4 {
                        terms.push((ctrl(fi, a, b), bu[a] * bv[b]));
                    }
                }
                push_row(&terms, lim[g[i][j] as usize]);
            }
        }
    }
    // C1 coupling across smooth edges
    let mut inc: HashMap<(u32, u32), Vec<(usize, usize)>> = HashMap::new();
    for (fi, f) in faces.iter().enumerate() {
        for side in 0..4 {
            inc.entry(key(f[side], f[(side + 1) % 4]))
                .or_default()
                .push((fi, side));
        }
    }
    // The coupling holds for the limit surface wherever it is regular: along edges whose end vertices
    // both have valence four. Around extraordinary vertices the natural charts of the patches meet only
    // continuously, so there the fit is left alone.
    let valence = levels[k].valences();
    if lambda > 0.0 {
        for kk in &keys {
            let uses = &inc[kk];
            if uses.len() != 2
                || levels[k].sharpness(kk.0, kk.1) > 0.0
                || valence[kk.0 as usize] != 4
                || valence[kk.1 as usize] != 4
            {
                continue;
            }
            let inner = |(fi, side): (usize, usize), c: usize| -> CRef {
                let f = &faces[fi];
                let (va, vb) = (f[side], f[(side + 1) % 4]);
                let q = if va < vb { c } else { 3 - c };
                match side {
                    0 => ctrl(fi, q, 1),
                    1 => ctrl(fi, 2, q),
                    2 => ctrl(fi, 3 - q, 2),
                    _ => ctrl(fi, 1, 3 - q),
                }
            };
            for c in 0..4 {
                push_row(
                    &[
                        (inner(uses[0], c), lambda),
                        (inner(uses[1], c), lambda),
                        (edge_point(*kk, c), -2.0 * lambda),
                    ],
                    [0.0; 3],
                );
            }
        }
    }
    // normal equations by conjugate gradients with a Jacobi preconditioner
    let mut diag = vec![1e-12; nvar];
    for r in &rows {
        for &(i, w) in &r.terms {
            diag[i] += w * w;
        }
    }
    let apply = |v: &Vec<P3>| -> Vec<P3> {
        let mut out = vec![[0.0; 3]; nvar];
        for r in &rows {
            let mut s = [0.0; 3];
            for &(i, w) in &r.terms {
                for a in 0..3 {
                    s[a] += w * v[i][a];
                }
            }
            for &(i, w) in &r.terms {
                for a in 0..3 {
                    out[i][a] += w * s[a];
                }
            }
        }
        out
    };
    let mut b = vec![[0.0; 3]; nvar];
    for r in &rows {
        for &(i, w) in &r.terms {
            for a in 0..3 {
                b[i][a] += w * r.rhs[a];
            }
        }
    }
    let dot = |u: &Vec<P3>, v: &Vec<P3>| -> f64 {
        u.iter()
            .zip(v)
            .map(|(p, q)| p[0] * q[0] + p[1] * q[1] + p[2] * q[2])
            .sum()
    };
    let ax = apply(&x);
    let mut res: Vec<P3> = (0..nvar)
        .map(|i| [b[i][0] - ax[i][0], b[i][1] - ax[i][1], b[i][2] - ax[i][2]])
        .collect();
    let mut z: Vec<P3> = (0..nvar)
        .map(|i| {
            [
                res[i][0] / diag[i],
                res[i][1] / diag[i],
                res[i][2] / diag[i],
            ]
        })
        .collect();
    let mut pdir = z.clone();
    let mut rz = dot(&res, &z);
    let rz0 = rz.max(1e-300);
    for _ in 0..400 {
        if rz < 1e-24 * rz0 {
            break;
        }
        let ap = apply(&pdir);
        let alpha = rz / dot(&pdir, &ap).max(1e-300);
        for i in 0..nvar {
            for a in 0..3 {
                x[i][a] += alpha * pdir[i][a];
                res[i][a] -= alpha * ap[i][a];
            }
        }
        for i in 0..nvar {
            for a in 0..3 {
                z[i][a] = res[i][a] / diag[i];
            }
        }
        let rz_new = dot(&res, &z);
        let beta = rz_new / rz.max(1e-300);
        rz = rz_new;
        for i in 0..nvar {
            for a in 0..3 {
                pdir[i][a] = z[i][a] + beta * pdir[i][a];
            }
        }
    }
    // write the solution back
    for (kk, c) in fit.edges.iter_mut() {
        let b = edge_var[kk];
        c[1] = x[b];
        c[2] = x[b + 1];
    }
    for (fi, f) in faces.iter().enumerate() {
        let p = &mut fit.patches[fi];
        let side_pts = |side: usize| -> [P3; 4] {
            let (va, vb) = (f[side], f[(side + 1) % 4]);
            let mut c = fit.edges[&key(va, vb)];
            if va > vb {
                c.reverse();
            }
            c
        };
        let (s0, s1, s2, s3) = (side_pts(0), side_pts(1), side_pts(2), side_pts(3));
        for q in 0..4 {
            p[q][0] = s0[q];
            p[3][q] = s1[q];
            p[3 - q][3] = s2[q];
            p[0][3 - q] = s3[q];
        }
        let b = patch_base + 4 * fi;
        p[1][1] = x[b];
        p[1][2] = x[b + 1];
        p[2][1] = x[b + 2];
        p[2][2] = x[b + 3];
    }
}

/// Largest distance between the patches and the limit surface, measured at twice the sampling density
/// of the fit (so at points the fit never saw).
fn deviation(levels: &[Level], limits: &[Option<Vec<P3>>], fit: &Fit, k: usize, m: usize) -> f64 {
    let lim = limits[k + m + 1].as_ref().unwrap();
    let n2 = 1usize << (m + 1);
    let mut worst = 0.0f64;
    for (fi, p) in fit.patches.iter().enumerate() {
        let g = face_grid(levels, k, fi, m + 1);
        for i in 0..=n2 {
            for j in 0..=n2 {
                let s = eval(p, i as f64 / n2 as f64, j as f64 / n2 as f64);
                worst = worst.max(dist(s, lim[g[i][j] as usize]));
            }
        }
    }
    worst
}

fn g1_report(levels: &[Level], fit: &Fit, k: usize) -> G1Report {
    let faces = &levels[k].faces;
    let mut inc: HashMap<(u32, u32), Vec<(usize, usize)>> = HashMap::new();
    for (fi, f) in faces.iter().enumerate() {
        for side in 0..4 {
            inc.entry(key(f[side], f[(side + 1) % 4]))
                .or_default()
                .push((fi, side));
        }
    }
    let mut keys: Vec<(u32, u32)> = inc.keys().copied().collect();
    keys.sort_unstable();
    let boundary_uv = |side: usize, t: f64| -> (f64, f64) {
        match side {
            0 => (t, 0.0),
            1 => (1.0, t),
            2 => (1.0 - t, 1.0),
            _ => (0.0, 1.0 - t),
        }
    };
    let mut rep = G1Report::default();
    let mut sum = 0.0;
    let mut count = 0usize;
    for kk in keys {
        let uses = &inc[&kk];
        if uses.len() != 2 {
            continue;
        }
        if levels[k].sharpness(kk.0, kk.1) > 0.0 {
            rep.crease_edges += 1;
            continue;
        }
        rep.edges_checked += 1;
        let (a, b) = (uses[0], uses[1]);
        let a_forward = faces[a.0][a.1] == kk.0; // traversal low → high in face a
        let mut worst = 0.0f64;
        for s in 0..=8 {
            let t = s as f64 / 8.0;
            // face a runs along the edge with parameter t, face b runs against it
            let ta = if a_forward { t } else { 1.0 - t };
            let tb = 1.0 - ta;
            let (ua, va) = boundary_uv(a.1, ta);
            let (ub, vb) = boundary_uv(b.1, tb);
            let (Some(na), Some(nb)) = (
                normal(&fit.patches[a.0], ua, va),
                normal(&fit.patches[b.0], ub, vb),
            ) else {
                continue;
            };
            worst = worst.max(na.dot(&nb).clamp(-1.0, 1.0).acos().to_degrees());
        }
        rep.max_angle_deg = rep.max_angle_deg.max(worst);
        sum += worst;
        count += 1;
    }
    rep.mean_angle_deg = if count > 0 { sum / count as f64 } else { 0.0 };
    rep
}

fn patch_paths(levels: &[Level], k: usize) -> Vec<(u32, String)> {
    (0..levels[k].faces.len())
        .map(|p| {
            let mut digits: Vec<char> = Vec::new();
            let mut q = p;
            for _ in 2..=k {
                digits.push(char::from_digit((q % 4) as u32, 10).unwrap());
                q /= 4;
            }
            // q is now the index of a level-1 face; find its cage face
            let cs = &levels[0].child_start;
            let f0 = cs.partition_point(|&s| s <= q) - 1;
            digits.push(char::from_digit((q - cs[f0]) as u32, 36).unwrap());
            digits.reverse();
            (f0 as u32, digits.into_iter().collect())
        })
        .collect()
}

fn to_brep(levels: &[Level], fit: &Fit, k: usize) -> Result<(BRep, BodyId), SubdError> {
    let kern = |e: String| SubdError::Kernel(e);
    let mut rep = BRep::new();
    let body = rep.add_body(BodyKind::Solid);
    let lump = rep.add_lump(body);
    let shell = rep.add_shell(lump);
    let faces = &levels[k].faces;
    let mut vmap: HashMap<u32, VertexId> = HashMap::new();
    let mut emap: HashMap<(u32, u32), EdgeId> = HashMap::new();
    let mut keys: Vec<(u32, u32)> = fit.edges.keys().copied().collect();
    keys.sort_unstable();
    for kk in keys {
        let c = fit.edges[&kk];
        let mut vert = |v: u32, p: P3, rep: &mut BRep| -> VertexId {
            *vmap
                .entry(v)
                .or_insert_with(|| rep.add_vertex(Point3::new(p[0], p[1], p[2])))
        };
        let va = vert(kk.0, c[0], &mut rep);
        let vb = vert(kk.1, c[3], &mut rep);
        let kv = KnotVector::clamped_uniform(3, 4).map_err(|e| kern(format!("{e:?}")))?;
        let curve = NurbsCurve::new_polynomial(
            kv,
            c.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect(),
        )
        .map_err(|e| kern(format!("{e:?}")))?;
        let cid = rep.add_curve(CurveGeom::Nurbs(curve));
        let e = rep.add_edge(va, vb, Some(cid), (0.0, 1.0));
        emap.insert(kk, e);
    }
    for (fi, f) in faces.iter().enumerate() {
        let p = &fit.patches[fi];
        let mut pts = Vec::with_capacity(16);
        for i in 0..4 {
            for j in 0..4 {
                pts.push(Point3::new(p[i][j][0], p[i][j][1], p[i][j][2]));
            }
        }
        let kv = KnotVector::clamped_uniform(3, 4).map_err(|e| kern(format!("{e:?}")))?;
        let surf = NurbsSurface::new(kv.clone(), kv, pts, vec![1.0; 16])
            .map_err(|e| kern(format!("{e:?}")))?;
        let sid = rep.add_surface(SurfaceGeom::Nurbs(surf));
        let face = rep.add_face(shell, sid, Sense::Forward);
        // corners of the patch in (u, v), counter-clockwise
        let uv = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let mut uses = Vec::with_capacity(4);
        for s in 0..4 {
            let (a, b) = (f[s], f[(s + 1) % 4]);
            let (lo_i, hi_i) = if a < b {
                (s, (s + 1) % 4)
            } else {
                ((s + 1) % 4, s)
            };
            let (p0, p1) = (uv[lo_i], uv[hi_i]);
            let pc = rep.add_curve(CurveGeom::Line(kw_nurbs::Line {
                origin: Point3::new(p0.0, p0.1, 0.0),
                dir: Vector3::new(p1.0 - p0.0, p1.1 - p0.1, 0.0),
            }));
            uses.push(CoedgeSpec {
                edge: emap[&key(a, b)],
                sense: if a < b {
                    Sense::Forward
                } else {
                    Sense::Reversed
                },
                pcurve: Some(pc),
            });
        }
        rep.add_loop(face, LoopKind::Outer, &uses);
    }
    Ok((rep, body))
}

/// Turn the limit surface of `mesh` into a closed B-rep of bicubic patches, using the coarsest level
/// whose patches stay within `opts.tolerance` of the limit surface.
pub fn finish(mesh: &SubdMesh, opts: &FinishOptions) -> Result<Finished, SubdError> {
    mesh.validate()?;
    if !mesh.is_closed() {
        return Err(SubdError::NotClosed);
    }
    let mesh = if mesh.signed_volume() < 0.0 {
        mesh.flipped()
    } else {
        mesh.clone()
    };
    let m_max = opts.fit_levels.max(2);
    let mut levels: Vec<Level> = vec![Level::from_mesh(&mesh)];
    let mut limits: Vec<Option<Vec<P3>>> = vec![None];
    let mut last = f64::INFINITY;
    let mut last_level = 0;
    let mut fallback: Option<(Fit, usize, f64, G1Report, usize)> = None;
    for k in opts.min_level.max(1)..=opts.max_level {
        // coarse levels are sampled finely, fine levels (many small patches) more sparsely
        let m = if k >= 4 { m_max.min(2) } else { m_max };
        // levels 0 ..= k + m + 1 are needed
        while levels.len() <= k + m + 1 {
            let l = levels.len() - 1;
            let next = {
                let (head, _) = levels.split_at_mut(l + 1);
                head[l].subdivide()
            };
            levels.push(next);
            limits.push(None);
        }
        let t_lvl = std::time::Instant::now();
        let patches = levels[k].faces.len();
        if patches > opts.max_patches {
            if last.is_infinite() {
                return Err(SubdError::TooLarge(format!(
                    "level {k} would need {patches} patches (limit {})",
                    opts.max_patches
                )));
            }
            break;
        }
        for l in [k + m, k + m + 1] {
            if limits[l].is_none() {
                limits[l] = Some(levels[l].limit());
            }
        }
        let t_lim = t_lvl.elapsed();
        let mut fit = fit_level(&levels, &limits, k, m);
        if opts.smoothness > 0.0 {
            refine_c1(
                &levels,
                limits[k + m].as_ref().unwrap(),
                &mut fit,
                k,
                m,
                opts.smoothness,
            );
        }
        let t_fit = t_lvl.elapsed();
        let dev = deviation(&levels, &limits, &fit, k, m);
        if std::env::var("KW_SUBD_TIME").is_ok() {
            eprintln!("level {k}: subdivide+limits {:.0} ms, fit {:.0} ms, deviation {:.0} ms -> {dev:.2e}", t_lim.as_secs_f64() * 1e3, (t_fit - t_lim).as_secs_f64() * 1e3, (t_lvl.elapsed() - t_fit).as_secs_f64() * 1e3);
        }
        last = dev;
        last_level = k;
        if dev <= opts.tolerance {
            let g1 = g1_report(&levels, &fit, k);
            let good = g1.max_angle_deg <= opts.g1_tolerance_deg;
            if std::env::var("KW_SUBD_TIME").is_ok() {
                eprintln!(
                    "level {k}: deviation {dev:.2e} G1 max {:.3}°",
                    g1.max_angle_deg
                );
            }
            if good || k == opts.max_level {
                let chosen = if good {
                    Some((fit, k, dev, g1, patches))
                } else {
                    fallback.take().or(Some((fit, k, dev, g1, patches)))
                };
                let (fit, k, dev, g1, patches) = chosen.unwrap();
                let (rep, body) = to_brep(&levels, &fit, k)?;
                return Ok(Finished {
                    rep,
                    body,
                    level: k,
                    patches,
                    patch_paths: patch_paths(&levels, k),
                    deviation: dev,
                    g1,
                });
            }
            if fallback.is_none() {
                fallback = Some((fit, k, dev, g1, patches));
            }
        }
    }
    if let Some((fit, k, dev, g1, patches)) = fallback {
        let (rep, body) = to_brep(&levels, &fit, k)?;
        return Ok(Finished {
            rep,
            body,
            level: k,
            patches,
            patch_paths: patch_paths(&levels, k),
            deviation: dev,
            g1,
        });
    }
    Err(SubdError::ToleranceNotMet {
        achieved: last,
        level: last_level,
    })
}
