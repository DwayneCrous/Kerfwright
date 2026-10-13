//! Hidden-line removal for a view of B-rep bodies.
//!
//! Every body is tessellated and its triangles are projected into the view. A point on an edge
//! is hidden when some projected triangle covers it and lies nearer the eye. Edges are sampled
//! along their exact curves, the visible / hidden transitions are found by bisection, and each
//! stretch is emitted as an exact line, circle, arc or ellipse (or a polyline for free-form
//! curves). Silhouettes of curved faces are added: analytically for cylinders, from the mesh
//! (where facets turn from facing the eye to facing away) for other surfaces. Edges whose two
//! faces meet tangentially, and seams of periodic faces, are not drawn.

use crate::model::ViewStyle;
use crate::projection::{dot, ViewFrame, V3};
use crate::scene::{Layer, Scene2d, Shape, P};
use kw_brep::{BRep, BodyId, EdgeId, FaceId};
use kw_nurbs::{CurveGeom, SurfaceGeom};
use kw_tess::{tessellate_body, TessOptions, Tessellation};
use std::collections::HashMap;
use std::f64::consts::TAU;

#[derive(Debug, Clone)]
pub struct HlrOptions {
    pub frame: ViewFrame,
    /// Sheet mm per model unit.
    pub scale: f64,
    pub style: ViewStyle,
}

/// Where a drawn primitive came from, so a click on it can be traced to the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    Edge(EdgeId),
    Silhouette(FaceId),
    None,
}

#[derive(Debug, Clone, Default)]
pub struct ViewGeometry {
    /// Geometry at sheet scale, centred on the origin.
    pub scene: Scene2d,
    /// One entry per primitive of `scene`.
    pub sources: Vec<Source>,
    /// The projected point (sheet mm) that sits at the origin of `scene`.
    pub centre: P,
    /// Circles drawn face-on (centre, radius), for centre marks.
    pub marks: Vec<(P, f64)>,
    /// Axes of cylinders seen from the side, for centre lines.
    pub axes: Vec<(P, P)>,
    /// Width and height of the geometry in mm.
    pub size: P,
}

// ---------------------------------------------------------------------------
// occlusion
// ---------------------------------------------------------------------------

struct Occluder {
    /// Triangles as (u, v, depth) corners.
    tris: Vec<[V3; 3]>,
    /// The face each triangle belongs to (unique across bodies).
    tags: Vec<u32>,
    lo: [f64; 2],
    cell: f64,
    n: usize,
    grid: Vec<Vec<u32>>,
    depth_tol: f64,
    /// How far outside a triangle (mm in the view) still counts as covered; the mesh is
    /// inscribed in curved faces by about the chord tolerance.
    xy_tol: f64,
    /// Cut faces of a section: they hide whatever lies behind them.
    caps: Vec<Cap>,
}

/// A region of the view (loops, filled by the even-odd rule) at one depth.
struct Cap {
    loops: Vec<Vec<[f64; 2]>>,
    depth: f64,
}

impl Cap {
    fn contains(&self, p: [f64; 2]) -> bool {
        let mut inside = false;
        for l in &self.loops {
            let n = l.len();
            for i in 0..n {
                let (a, b) = (l[i], l[(i + 1) % n]);
                if (a[1] > p[1]) != (b[1] > p[1])
                    && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
                {
                    inside = !inside;
                }
            }
        }
        inside
    }
}

impl Occluder {
    fn new(tris: Vec<[V3; 3]>, tags: Vec<u32>, depth_tol: f64, xy_tol: f64) -> Occluder {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for t in &tris {
            for c in t {
                for k in 0..2 {
                    lo[k] = lo[k].min(c[k]);
                    hi[k] = hi[k].max(c[k]);
                }
            }
        }
        if tris.is_empty() {
            return Occluder {
                tris,
                tags,
                lo: [0.0; 2],
                cell: 1.0,
                n: 1,
                grid: vec![Vec::new()],
                depth_tol,
                xy_tol,
                caps: Vec::new(),
            };
        }
        let n = ((tris.len() as f64).sqrt() as usize).clamp(1, 256);
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-9);
        let cell = span / n as f64 * (1.0 + 1e-9);
        let mut grid = vec![Vec::new(); n * n];
        for (i, t) in tris.iter().enumerate() {
            let (mut a, mut b) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for c in t {
                for k in 0..2 {
                    a[k] = a[k].min(c[k]);
                    b[k] = b[k].max(c[k]);
                }
            }
            let cx = |v: f64, k: usize| (((v - lo[k]) / cell).floor().max(0.0) as usize).min(n - 1);
            for y in cx(a[1] - xy_tol, 1)..=cx(b[1] + xy_tol, 1) {
                for x in cx(a[0] - xy_tol, 0)..=cx(b[0] + xy_tol, 0) {
                    grid[y * n + x].push(i as u32);
                }
            }
        }
        Occluder {
            tris,
            tags,
            lo,
            cell,
            n,
            grid,
            depth_tol,
            xy_tol,
            caps: Vec::new(),
        }
    }

    /// Is the point (world `p`, already projected to `uv` with `depth`) behind some triangle?
    ///
    /// Triangles of the faces in `own` (the faces the point lies on) only count when the point
    /// is well inside them, so a point on the boundary of its own face is not hidden by it.
    fn hidden(&self, uv: [f64; 2], depth: f64, own: &[u32]) -> bool {
        let n = self.n;
        let cx =
            |k: usize| (((uv[k] - self.lo[k]) / self.cell).floor().max(0.0) as usize).min(n - 1);
        for &i in &self.grid[cx(1) * n + cx(0)] {
            let t = &self.tris[i as usize];
            let area = (t[1][0] - t[0][0]) * (t[2][1] - t[0][1])
                - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1]);
            if area.abs() < 1e-14 {
                continue;
            }
            let w0 = ((t[1][0] - uv[0]) * (t[2][1] - uv[1])
                - (t[2][0] - uv[0]) * (t[1][1] - uv[1]))
                / area;
            let w1 = ((t[2][0] - uv[0]) * (t[0][1] - uv[1])
                - (t[0][0] - uv[0]) * (t[2][1] - uv[1]))
                / area;
            let w2 = 1.0 - w0 - w1;
            // distance outside each side, in view units: barycentric weight times the height
            let edge = |a: &V3, b: &V3| (b[0] - a[0]).hypot(b[1] - a[1]);
            let (l0, l1, l2) = (edge(&t[1], &t[2]), edge(&t[2], &t[0]), edge(&t[0], &t[1]));
            let h = |w: f64, l: f64| w * area.abs() / l.max(1e-300);
            let mine = own.contains(&self.tags[i as usize]);
            let margin = if mine { 0.0 } else { -self.xy_tol };
            if h(w0, l0) < margin || h(w1, l1) < margin || h(w2, l2) < margin {
                continue;
            }
            // how much the depth changes over the tolerance band; a face's own triangle must
            // be nearer by more than that to hide a point lying on the face's boundary
            let extra = if mine {
                let (du, dv) = (
                    ((t[1][2] - t[0][2]) * (t[2][1] - t[0][1])
                        - (t[2][2] - t[0][2]) * (t[1][1] - t[0][1]))
                        / area,
                    ((t[2][2] - t[0][2]) * (t[1][0] - t[0][0])
                        - (t[1][2] - t[0][2]) * (t[2][0] - t[0][0]))
                        / area,
                );
                2.0 * du.hypot(dv) * self.xy_tol
            } else {
                0.0
            };
            // depth at the nearest point of the triangle: no extrapolating a steep one
            let (c0, c1, c2) = (w0.max(0.0), w1.max(0.0), w2.max(0.0));
            let d = (c0 * t[0][2] + c1 * t[1][2] + c2 * t[2][2]) / (c0 + c1 + c2);
            if d < depth - self.depth_tol - extra {
                return true;
            }
        }
        self.caps
            .iter()
            .any(|c| depth > c.depth + self.depth_tol && c.contains(uv))
    }
}

// ---------------------------------------------------------------------------
// curves
// ---------------------------------------------------------------------------

fn v3(p: &kw_math::Point3<f64>) -> V3 {
    [p.x, p.y, p.z]
}
fn vv(p: &kw_math::Vector3<f64>) -> V3 {
    [p.x, p.y, p.z]
}

/// A curve to draw, parametrised by `tau` in 0..=1.
enum Curve {
    Segment(V3, V3),
    /// `origin + a cos t + b sin t` for `t` in `t0..t1`.
    Conic {
        o: V3,
        a: V3,
        b: V3,
        t0: f64,
        t1: f64,
    },
    Poly(Vec<V3>),
}

impl Curve {
    fn at(&self, tau: f64) -> V3 {
        match self {
            Curve::Segment(a, b) => [
                a[0] + (b[0] - a[0]) * tau,
                a[1] + (b[1] - a[1]) * tau,
                a[2] + (b[2] - a[2]) * tau,
            ],
            Curve::Conic { o, a, b, t0, t1 } => {
                let t = t0 + (t1 - t0) * tau;
                let (s, c) = t.sin_cos();
                [
                    o[0] + a[0] * c + b[0] * s,
                    o[1] + a[1] * c + b[1] * s,
                    o[2] + a[2] * c + b[2] * s,
                ]
            }
            Curve::Poly(p) => {
                let x = tau.clamp(0.0, 1.0) * (p.len() - 1) as f64;
                let i = (x.floor() as usize).min(p.len() - 2);
                let f = x - i as f64;
                [
                    p[i][0] + (p[i + 1][0] - p[i][0]) * f,
                    p[i][1] + (p[i + 1][1] - p[i][1]) * f,
                    p[i][2] + (p[i + 1][2] - p[i][2]) * f,
                ]
            }
        }
    }

    /// How many samples to look at for visibility.
    fn samples(&self) -> usize {
        match self {
            Curve::Segment(..) => 33,
            Curve::Conic { t0, t1, .. } => {
                (((t1 - t0).abs() / (2.0_f64).to_radians()).ceil() as usize + 1).max(33)
            }
            Curve::Poly(p) => (p.len() * 2).max(9),
        }
    }

    /// The drawn shape of the stretch `tau0..tau1`, at sheet scale.
    fn shape(&self, tau0: f64, tau1: f64, f: &ViewFrame, scale: f64) -> Shape {
        let pr = |p: V3| -> P {
            let (q, _) = f.project(p);
            [q[0] * scale, q[1] * scale]
        };
        match self {
            Curve::Segment(..) => Shape::Line {
                a: pr(self.at(tau0)),
                b: pr(self.at(tau1)),
            },
            Curve::Poly(_) => {
                let n = (self.samples() as f64 * (tau1 - tau0)).ceil().max(2.0) as usize;
                Shape::Polyline {
                    pts: (0..=n)
                        .map(|i| pr(self.at(tau0 + (tau1 - tau0) * i as f64 / n as f64)))
                        .collect(),
                    closed: false,
                }
            }
            Curve::Conic { o, a, b, t0, t1 } => {
                let (ta, tb) = (t0 + (t1 - t0) * tau0, t0 + (t1 - t0) * tau1);
                conic_shape(f, scale, *o, *a, *b, ta, tb, &pr)
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn conic_shape(
    f: &ViewFrame,
    scale: f64,
    o: V3,
    a: V3,
    b: V3,
    t0: f64,
    t1: f64,
    pr: &dyn Fn(V3) -> P,
) -> Shape {
    let c = pr(o);
    let a2 = [dot(a, f.right) * scale, dot(a, f.up) * scale];
    let b2 = [dot(b, f.right) * scale, dot(b, f.up) * scale];
    let (la, lb) = (a2[0].hypot(a2[1]), b2[0].hypot(b2[1]));
    let cr = a2[0] * b2[1] - a2[1] * b2[0];
    let sweep = t1 - t0;
    // seen edge-on: the curve collapses onto a line spanning where it goes
    if cr.abs() < 1e-9 * (la * lb).max(1e-12) || la.max(lb) < 1e-12 {
        let d = if la >= lb { a2 } else { b2 };
        let dl = d[0].hypot(d[1]).max(1e-300);
        let d = [d[0] / dl, d[1] / dl];
        let n = ((sweep.abs() / 0.05).ceil() as usize).clamp(2, 800);
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for i in 0..=n {
            let t = t0 + sweep * i as f64 / n as f64;
            let x = (a2[0] * t.cos() + b2[0] * t.sin()) * d[0]
                + (a2[1] * t.cos() + b2[1] * t.sin()) * d[1];
            lo = lo.min(x);
            hi = hi.max(x);
        }
        return Shape::Line {
            a: [c[0] + d[0] * lo, c[1] + d[1] * lo],
            b: [c[0] + d[0] * hi, c[1] + d[1] * hi],
        };
    }
    let ab = a2[0] * b2[0] + a2[1] * b2[1];
    let delta = 0.5 * (2.0 * ab).atan2(la * la - lb * lb);
    let (sd, cd) = delta.sin_cos();
    let maj = [a2[0] * cd + b2[0] * sd, a2[1] * cd + b2[1] * sd];
    let min = [-a2[0] * sd + b2[0] * cd, -a2[1] * sd + b2[1] * cd];
    let (lmaj, lmin) = (maj[0].hypot(maj[1]), min[0].hypot(min[1]));
    // shape parameter s = t - delta; a mirrored view runs the other way round
    let (mut s0, mut s1) = (t0 - delta, t1 - delta);
    if cr < 0.0 {
        (s0, s1) = (-s1, -s0);
    }
    let full = sweep.abs() >= TAU - 1e-9;
    let ratio = lmin / lmaj;
    if (ratio - 1.0).abs() < 1e-9 {
        let theta = maj[1].atan2(maj[0]);
        if full {
            return Shape::Circle { c, r: lmaj };
        }
        return Shape::Arc {
            c,
            r: lmaj,
            a0: theta + s0,
            a1: theta + s1,
        };
    }
    Shape::Ellipse {
        c,
        major: maj,
        ratio,
        t0: if full { 0.0 } else { s0 },
        t1: if full { TAU } else { s1 },
    }
}

/// What a point of a curve looks like in the view.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum St {
    Visible,
    Hidden,
    /// Cut away by the section plane: not drawn at all.
    Gone,
}

/// Stretches `(tau0, tau1, state)` of a curve, from sampling and bisection.
fn runs(c: &Curve, status: &dyn Fn(V3) -> St, min_len: f64) -> Vec<(f64, f64, St)> {
    let n = c.samples();
    let st: Vec<St> = (0..n)
        .map(|i| status(c.at(i as f64 / (n - 1) as f64)))
        .collect();
    let mut out: Vec<(f64, f64, St)> = Vec::new();
    let mut start = 0.0;
    let mut cur = st[0];
    for i in 1..n {
        if st[i] != st[i - 1] {
            let (mut lo, mut hi) = ((i - 1) as f64 / (n - 1) as f64, i as f64 / (n - 1) as f64);
            for _ in 0..24 {
                let mid = 0.5 * (lo + hi);
                if status(c.at(mid)) == st[i - 1] {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let tb = 0.5 * (lo + hi);
            out.push((start, tb, cur));
            start = tb;
            cur = st[i];
        }
    }
    out.push((start, 1.0, cur));
    // a stretch shorter than `min_len` is noise at a borderline (an edge ending on a
    // silhouette): it takes the state of its longer neighbour
    let len = |a: f64, b: f64| -> f64 {
        let k = 8;
        (0..k)
            .map(|i| {
                let (p, q) = (
                    c.at(a + (b - a) * i as f64 / k as f64),
                    c.at(a + (b - a) * (i + 1) as f64 / k as f64),
                );
                ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
            })
            .sum()
    };
    while out.len() > 1 {
        let Some(i) = (0..out.len())
            .filter(|&i| len(out[i].0, out[i].1) < min_len)
            .min_by(|&a, &b| len(out[a].0, out[a].1).total_cmp(&len(out[b].0, out[b].1)))
        else {
            break;
        };
        let state = match (i.checked_sub(1), out.get(i + 1)) {
            (Some(l), Some(r)) => {
                if len(out[l].0, out[l].1) >= len(r.0, r.1) {
                    out[l].2
                } else {
                    r.2
                }
            }
            (Some(l), None) => out[l].2,
            (None, Some(r)) => r.2,
            (None, None) => break,
        };
        out[i].2 = state;
        // join equal neighbours
        let mut merged: Vec<(f64, f64, St)> = Vec::new();
        for r in out.drain(..) {
            match merged.last_mut() {
                Some(l) if l.2 == r.2 => l.1 = r.1,
                _ => merged.push(r),
            }
        }
        out = merged;
    }
    out
}

// ---------------------------------------------------------------------------
// the view
// ---------------------------------------------------------------------------

fn pos_key(p: &kw_math::Point3<f64>) -> [u64; 3] {
    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
}

struct BodyData {
    tess: Tessellation,
    /// Added to a face's dense index to make a tag unique across bodies.
    base: u32,
}

/// A section plane: what lies on the `keep` side of the plane through `p0` is drawn, the rest is
/// cut away. `caps` holds the outlines (3-D loops lying in the plane) of what the plane cuts,
/// one list per body; they are drawn as edges and hide what lies behind them.
pub struct Clip<'a> {
    pub p0: V3,
    pub keep: V3,
    pub caps: &'a [Vec<Vec<V3>>],
}

impl Clip<'_> {
    fn side(&self, p: V3) -> f64 {
        dot(
            [p[0] - self.p0[0], p[1] - self.p0[1], p[2] - self.p0[2]],
            self.keep,
        )
    }
}

/// Project the given bodies of `brep` into a view.
pub fn project_bodies(
    brep: &BRep,
    bodies: &[BodyId],
    o: &HlrOptions,
) -> Result<ViewGeometry, String> {
    project_clipped(brep, bodies, o, None)
}

/// Like [`project_bodies`], with part of the model cut away by a section plane.
pub fn project_clipped(
    brep: &BRep,
    bodies: &[BodyId],
    o: &HlrOptions,
    clip: Option<&Clip>,
) -> Result<ViewGeometry, String> {
    // overall size, for tolerances
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for b in bodies {
        let bb = brep.body_bbox(*b);
        for k in 0..3 {
            lo[k] = lo[k].min(bb.min[k]);
            hi[k] = hi[k].max(bb.max[k]);
        }
    }
    if !lo[0].is_finite() {
        return Ok(ViewGeometry::default());
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    let opts = TessOptions {
        chord_tol: (diag * 1e-4).max(1e-6),
        angle_tol: 0.1,
    };
    let mut data = Vec::new();
    let mut base = 0;
    for b in bodies {
        let tess = tessellate_body(brep, *b, &opts).map_err(|e| e.to_string())?;
        let n = tess.face_keys.len() as u32;
        data.push(BodyData { tess, base });
        base += n;
    }
    let f = &o.frame;

    // triangles of every body, in view coordinates; with a section plane, only the kept part
    let mut tris: Vec<[V3; 3]> = Vec::new();
    let mut tags: Vec<u32> = Vec::new();
    for d in &data {
        for t in 0..d.tess.mesh.triangle_count() {
            let c = d.tess.mesh.triangle(t);
            let world = [v3(&c[0]), v3(&c[1]), v3(&c[2])];
            let pieces: Vec<[V3; 3]> = match clip {
                None => vec![world],
                Some(cl) => clip_triangle(&world, cl),
            };
            for w in pieces {
                let mut out = [[0.0; 3]; 3];
                for (k, p) in w.iter().enumerate() {
                    let (uv, depth) = f.project(*p);
                    out[k] = [uv[0], uv[1], depth];
                }
                tris.push(out);
                tags.push(d.base + d.tess.mesh.tri_face[t]);
            }
        }
    }
    let wire = o.style == ViewStyle::Wireframe;
    let mut occ = Occluder::new(
        tris,
        tags,
        diag * 1e-4 + opts.chord_tol,
        opts.chord_tol * 1.5,
    );
    if let Some(cl) = clip {
        // the cut faces face the viewer when the plane is square to the line of sight
        if dot(cl.keep, f.dir) > 1.0 - 1e-9 {
            for body in cl.caps {
                let loops: Vec<Vec<[f64; 2]>> = body
                    .iter()
                    .map(|l| l.iter().map(|p| f.project(*p).0).collect())
                    .collect();
                occ.caps.push(Cap {
                    loops,
                    depth: dot(cl.p0, f.dir),
                });
            }
        }
    }
    let absent_tol = diag * 1e-7;
    let status = |p: V3, own: &[u32]| -> St {
        if clip.is_some_and(|c| c.side(p) < -absent_tol) {
            return St::Gone;
        }
        if wire {
            return St::Visible;
        }
        let (uv, d) = f.project(p);
        if occ.hidden(uv, d, own) {
            St::Hidden
        } else {
            St::Visible
        }
    };

    let mut scene = Scene2d::new();
    let mut sources: Vec<Source> = Vec::new();
    if o.style == ViewStyle::Shaded {
        shaded_fills(&data, f, o.scale, clip, &mut scene, &mut sources);
    }

    let min_len = diag * 5e-4;
    let show_hidden = matches!(o.style, ViewStyle::VisibleHidden | ViewStyle::Wireframe);
    let mut emit = |scene: &mut Scene2d, src: Source, c: &Curve, own: &[u32]| {
        for (t0, t1, st) in runs(c, &|p| status(p, own), min_len) {
            if t1 - t0 < 1e-9 {
                continue;
            }
            let layer = match st {
                St::Gone => continue,
                St::Visible => Layer::Visible,
                St::Hidden if wire => Layer::Visible,
                St::Hidden if show_hidden => Layer::Hidden,
                St::Hidden => continue,
            };
            let shape = c.shape(t0, t1, f, o.scale);
            if degenerate(&shape) {
                continue;
            }
            scene.push(layer, shape);
            sources.push(src);
        }
    };

    let mut circles: Vec<(kw_nurbs::geom::Circle, Vec<u32>)> = Vec::new();
    for d in &data {
        edges(
            brep,
            d,
            &mut |src, c, own| emit(&mut scene, src, c, own),
            &mut |ci, own| circles.push((ci.clone(), own.to_vec())),
        );
        if !wire {
            silhouettes(brep, d, f, &mut |src, c, own| emit(&mut scene, src, c, own));
        }
    }

    // the outline of what the section plane cuts
    if let Some(cl) = clip {
        for body in cl.caps {
            for l in body {
                // each side on its own, so lines behind it that coincide are recognised
                for i in 0..l.len() {
                    let (a, b) = (l[i], l[(i + 1) % l.len()]);
                    emit(&mut scene, Source::None, &Curve::Segment(a, b), &[]);
                }
            }
        }
    }

    drop_covered_hidden(&mut scene, &mut sources);

    // centre the geometry on the view origin
    let Some((blo, bhi)) = scene.bounds() else {
        return Ok(ViewGeometry::default());
    };
    let centre = [(blo[0] + bhi[0]) / 2.0, (blo[1] + bhi[1]) / 2.0];
    let local = |p: P| [p[0] - centre[0], p[1] - centre[1]];
    // circles seen face-on with some of them showing: where the centre marks go
    let mut marks: Vec<(P, f64)> = Vec::new();
    for (ci, own) in &circles {
        let axis = vv(&ci.frame.z);
        if dot(axis, f.dir).abs() < 0.999 {
            continue;
        }
        let ctr = v3(&ci.frame.origin);
        let (u, w) = (vv(&ci.frame.x), vv(&ci.frame.y));
        let shows = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
            .iter()
            .any(|(a, b)| {
                let p = [
                    ctr[0] + (u[0] * a + w[0] * b) * ci.radius,
                    ctr[1] + (u[1] * a + w[1] * b) * ci.radius,
                    ctr[2] + (u[2] * a + w[2] * b) * ci.radius,
                ];
                status(p, own) == St::Visible
            });
        if !shows {
            continue;
        }
        let q = f.project(ctr).0;
        let at = local([q[0] * o.scale, q[1] * o.scale]);
        let r = ci.radius * o.scale;
        if !marks
            .iter()
            .any(|(c, rr)| (c[0] - at[0]).hypot(c[1] - at[1]) < 1e-6 && (rr - r).abs() < 1e-6)
        {
            marks.push((at, r));
        }
    }
    let axes: Vec<(P, P)> = if clip.is_some() || wire {
        Vec::new()
    } else {
        data.iter()
            .flat_map(|d| cylinder_axes(brep, d, f))
            .map(|(a, b)| {
                let (pa, pb) = (f.project(a).0, f.project(b).0);
                (
                    local([pa[0] * o.scale, pa[1] * o.scale]),
                    local([pb[0] * o.scale, pb[1] * o.scale]),
                )
            })
            .collect()
    };
    Ok(ViewGeometry {
        scene: scene.placed(1.0, [-centre[0], -centre[1]]),
        sources,
        marks,
        axes,
        centre,
        size: [bhi[0] - blo[0], bhi[1] - blo[1]],
    })
}

/// Grey fills of the front-facing triangles, far to near, lit from the viewer's upper left.
fn shaded_fills(
    data: &[BodyData],
    f: &ViewFrame,
    scale: f64,
    clip: Option<&Clip>,
    scene: &mut Scene2d,
    sources: &mut Vec<Source>,
) {
    let light = {
        let l = [
            -f.dir[0] * 0.7 - f.right[0] * 0.3 + f.up[0] * 0.4,
            -f.dir[1] * 0.7 - f.right[1] * 0.3 + f.up[1] * 0.4,
            -f.dir[2] * 0.7 - f.right[2] * 0.3 + f.up[2] * 0.4,
        ];
        let n = dot(l, l).sqrt();
        [l[0] / n, l[1] / n, l[2] / n]
    };
    let mut items: Vec<(f64, [P; 3], u8)> = Vec::new();
    for d in data {
        for t in 0..d.tess.mesh.triangle_count() {
            let c = d.tess.mesh.triangle(t);
            let (a, b, cc) = (v3(&c[0]), v3(&c[1]), v3(&c[2]));
            let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let e2 = [cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]];
            let n = crate::projection::cross(e1, e2);
            let nl = dot(n, n).sqrt();
            if nl < 1e-18 || dot(n, f.dir) >= 0.0 {
                continue;
            }
            if clip.is_some_and(|cl| {
                cl.side([
                    (a[0] + b[0] + cc[0]) / 3.0,
                    (a[1] + b[1] + cc[1]) / 3.0,
                    (a[2] + b[2] + cc[2]) / 3.0,
                ]) < 0.0
            }) {
                continue;
            }
            let shade = 150.0 + 90.0 * (dot(n, light) / nl).max(0.0);
            let pts = [a, b, cc].map(|p| {
                let (q, _) = f.project(p);
                [q[0] * scale, q[1] * scale]
            });
            let depth = (f.project(a).1 + f.project(b).1 + f.project(cc).1) / 3.0;
            items.push((depth, pts, shade.min(250.0) as u8));
        }
    }
    items.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (_, pts, shade) in items {
        scene.push(
            Layer::Hatch,
            Shape::Fill {
                pts: pts.to_vec(),
                shade,
            },
        );
        sources.push(Source::None);
    }
}

/// The B-rep edges of one body, each as an exact curve.
fn edges(
    brep: &BRep,
    d: &BodyData,
    emit: &mut dyn FnMut(Source, &Curve, &[u32]),
    circle: &mut dyn FnMut(&kw_nurbs::geom::Circle, &[u32]),
) {
    let t = &d.tess;
    let face_idx: HashMap<FaceId, u32> = t
        .face_keys
        .iter()
        .enumerate()
        .map(|(i, f)| (*f, i as u32))
        .collect();
    // surface normal of each face at each mesh vertex position, to find tangent edges
    let mut normals: HashMap<([u64; 3], u32), kw_math::Vector3<f64>> = HashMap::new();
    for tri in 0..t.mesh.triangle_count() {
        let fi = t.mesh.tri_face[tri];
        for k in 0..3 {
            let vi = t.mesh.indices[3 * tri + k] as usize;
            normals
                .entry((pos_key(&t.mesh.positions[vi]), fi))
                .or_insert(t.mesh.normals[vi]);
        }
    }
    let polylines: HashMap<u32, (u32, u32)> = t
        .edges
        .ranges
        .iter()
        .map(|(e, a, n)| (*e, (*a, *n)))
        .collect();

    for (dense, ek) in t.edge_keys.iter().enumerate() {
        let Some(edge) = brep.edges.get(*ek) else {
            continue;
        };
        let Some(&(first, count)) = polylines.get(&(dense as u32)) else {
            continue;
        };
        let faces = brep.faces_around_edge(*ek);
        if faces.len() >= 2 && faces.iter().all(|f| *f == faces[0]) {
            continue; // a seam
        }
        // tangent edge: the two faces' normals agree where they meet
        if faces.len() == 2 && faces[0] != faces[1] {
            let mid = &t.edges.points[(first + count / 2) as usize];
            if let (Some(fa), Some(fb)) = (face_idx.get(&faces[0]), face_idx.get(&faces[1])) {
                if let (Some(na), Some(nb)) = (
                    normals.get(&(pos_key(mid), *fa)),
                    normals.get(&(pos_key(mid), *fb)),
                ) {
                    if na.dot(nb) > (1.5_f64).to_radians().cos() {
                        continue;
                    }
                }
            }
        }
        let src = Source::Edge(*ek);
        let poly = || {
            let pts: Vec<V3> = t.edges.points[first as usize..(first + count) as usize]
                .iter()
                .map(v3)
                .collect();
            (pts.len() >= 2).then_some(Curve::Poly(pts))
        };
        let curve = edge.curve.and_then(|c| brep.curves.get(c));
        let (t0, t1) = edge.param_range;
        let c = match curve {
            Some(CurveGeom::Line(_)) => {
                let (a, b) = (brep.vertices.get(edge.start), brep.vertices.get(edge.end));
                match (a, b) {
                    (Some(a), Some(b)) => Some(Curve::Segment(v3(&a.point), v3(&b.point))),
                    _ => poly(),
                }
            }
            Some(CurveGeom::Circle(c)) => Some(Curve::Conic {
                o: v3(&c.frame.origin),
                a: scale3(vv(&c.frame.x), c.radius),
                b: scale3(vv(&c.frame.y), c.radius),
                t0,
                t1,
            }),
            Some(CurveGeom::Ellipse(e)) => Some(Curve::Conic {
                o: v3(&e.frame.origin),
                a: scale3(vv(&e.frame.x), e.major),
                b: scale3(vv(&e.frame.y), e.minor),
                t0,
                t1,
            }),
            _ => poly(),
        };
        if let Some(c) = c {
            let own: Vec<u32> = faces
                .iter()
                .filter_map(|f| face_idx.get(f).map(|i| d.base + i))
                .collect();
            if let Some(CurveGeom::Circle(ci)) = curve {
                circle(ci, &own);
            }
            emit(src, &c, &own);
        }
    }
}

fn scale3(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// Silhouettes of the curved faces of one body.
fn silhouettes(
    brep: &BRep,
    d: &BodyData,
    f: &ViewFrame,
    emit: &mut dyn FnMut(Source, &Curve, &[u32]),
) {
    let t = &d.tess;
    let m = &t.mesh;
    // facing of each triangle: true when it faces away from the eye
    let away: Vec<bool> = (0..m.triangle_count())
        .map(|i| {
            let c = m.triangle(i);
            let (a, b, cc) = (v3(&c[0]), v3(&c[1]), v3(&c[2]));
            let n = crate::projection::cross(
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [cc[0] - a[0], cc[1] - a[1], cc[2] - a[2]],
            );
            dot(n, f.dir) > -1e-12 * dot(n, n).sqrt()
        })
        .collect();
    // vertices at one position count as one, so triangles meet across a periodic seam
    let mut canon: HashMap<[u64; 3], u32> = HashMap::new();
    let mut cpos: Vec<V3> = Vec::new();
    let cid: Vec<u32> = m
        .positions
        .iter()
        .map(|p| {
            *canon.entry(pos_key(p)).or_insert_with(|| {
                cpos.push(v3(p));
                (cpos.len() - 1) as u32
            })
        })
        .collect();
    // mesh edges shared by two triangles of one face, keyed by canonical vertex
    let mut by_face: HashMap<u32, HashMap<(u32, u32), Vec<usize>>> = HashMap::new();
    for tri in 0..m.triangle_count() {
        let fi = m.tri_face[tri];
        let Some(face) = brep.faces.get(t.face_keys[fi as usize]) else {
            continue;
        };
        if matches!(
            brep.surfaces.get(face.surface),
            Some(SurfaceGeom::Plane(_)) | None
        ) {
            continue;
        }
        let ix = [
            cid[m.indices[3 * tri] as usize],
            cid[m.indices[3 * tri + 1] as usize],
            cid[m.indices[3 * tri + 2] as usize],
        ];
        for k in 0..3 {
            let (a, b) = (ix[k], ix[(k + 1) % 3]);
            by_face
                .entry(fi)
                .or_default()
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push(tri);
        }
    }
    let mut faces: Vec<u32> = by_face.keys().copied().collect();
    faces.sort_unstable();
    for fi in faces {
        let fk = t.face_keys[fi as usize];
        let sil: Vec<(u32, u32)> = by_face[&fi]
            .iter()
            .filter(|(_, tr)| tr.len() == 2 && away[tr[0]] != away[tr[1]])
            .map(|(e, _)| *e)
            .collect();
        if sil.is_empty() {
            continue;
        }
        let surf = brep
            .faces
            .get(fk)
            .and_then(|fc| brep.surfaces.get(fc.surface));
        if let Some(SurfaceGeom::Cylinder(cy)) = surf {
            cylinder_silhouettes(&sil, &cpos, cy, f, &mut |c| {
                emit(Source::Silhouette(fk), c, &[d.base + fi])
            });
            continue;
        }
        for chain in chains(&sil) {
            let pts: Vec<V3> = chain.iter().map(|i| cpos[*i as usize]).collect();
            if pts.len() >= 2 {
                emit(Source::Silhouette(fk), &Curve::Poly(pts), &[d.base + fi]);
            }
        }
    }
}

/// The two silhouette lines of a cylinder seen from the side, placed exactly on the surface and
/// spanning what the mesh says is present.
fn cylinder_silhouettes(
    sil: &[(u32, u32)],
    pos: &[V3],
    cy: &kw_nurbs::geom::Cylinder,
    f: &ViewFrame,
    emit: &mut dyn FnMut(&Curve),
) {
    let axis = vv(&cy.frame.z);
    let o = v3(&cy.frame.origin);
    let side = crate::projection::cross(f.dir, axis);
    let sl = dot(side, side).sqrt();
    if sl < 1e-9 {
        return; // looking along the axis
    }
    let side = scale3(side, 1.0 / sl);
    for sign in [1.0, -1.0] {
        let want = scale3(side, sign);
        let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for (a, b) in sil {
            for i in [a, b] {
                let p = pos[*i as usize];
                let rel = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
                let z = dot(rel, axis);
                let radial = [
                    rel[0] - z * axis[0],
                    rel[1] - z * axis[1],
                    rel[2] - z * axis[2],
                ];
                if dot(radial, want) > 0.8 * cy.radius {
                    zmin = zmin.min(z);
                    zmax = zmax.max(z);
                }
            }
        }
        if zmin <= zmax && zmax - zmin > 1e-12 {
            let at = |z: f64| {
                [
                    o[0] + axis[0] * z + want[0] * cy.radius,
                    o[1] + axis[1] * z + want[1] * cy.radius,
                    o[2] + axis[2] * z + want[2] * cy.radius,
                ]
            };
            emit(&Curve::Segment(at(zmin), at(zmax)));
        }
    }
}

/// Join mesh edges into chains of vertex indices.
fn chains(edges: &[(u32, u32)]) -> Vec<Vec<u32>> {
    let mut adj: HashMap<u32, Vec<u32>> = HashMap::new();
    for (a, b) in edges {
        adj.entry(*a).or_default().push(*b);
        adj.entry(*b).or_default().push(*a);
    }
    let mut keys: Vec<u32> = adj.keys().copied().collect();
    keys.sort_unstable();
    // open chains start at a vertex of degree one, closed ones anywhere
    keys.sort_by_key(|k| adj[k].len() != 1);
    let mut used: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    let ek = |a: u32, b: u32| (a.min(b), a.max(b));
    let mut out = Vec::new();
    for s in keys {
        while let Some(&n) = adj[&s].iter().find(|n| !used.contains(&ek(s, **n))) {
            let mut chain = vec![s];
            let mut cur = n;
            used.insert(ek(s, cur));
            chain.push(cur);
            while let Some(&nx) = adj[&cur].iter().find(|x| !used.contains(&ek(cur, **x))) {
                used.insert(ek(cur, nx));
                cur = nx;
                chain.push(cur);
            }
            out.push(chain);
        }
    }
    out
}

fn degenerate(s: &Shape) -> bool {
    match s {
        Shape::Line { a, b } => (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9,
        Shape::Circle { r, .. } | Shape::Arc { r, .. } => *r < 1e-9,
        Shape::Polyline { pts, .. } => pts.len() < 2,
        _ => false,
    }
}

/// A line lying exactly along another, longer (or equal, earlier) one, or along a visible one
/// when it is hidden, is not drawn again; the same for a hidden circle under a visible one.
fn drop_covered_hidden(scene: &mut Scene2d, sources: &mut Vec<Source>) {
    let tol = 1e-6;
    let rank = |l: Layer| if l == Layer::Hidden { 0 } else { 1 };
    let len = |a: P, b: P| (a[0] - b[0]).hypot(a[1] - b[1]);
    let on_segment = |p: P, a: P, b: P| {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        let t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2;
        let (qx, qy) = (a[0] + t * dx - p[0], a[1] + t * dy - p[1]);
        (-1e-9..=1.0 + 1e-9).contains(&t) && qx.hypot(qy) < tol
    };
    let lines: Vec<(usize, Layer, P, P)> = scene
        .prims
        .iter()
        .enumerate()
        .filter(|(_, p)| matches!(p.layer, Layer::Visible | Layer::Hidden))
        .filter_map(|(i, p)| match &p.shape {
            Shape::Line { a, b } if len(*a, *b) > tol => Some((i, p.layer, *a, *b)),
            _ => None,
        })
        .collect();
    let mut keep = vec![true; scene.prims.len()];
    for (i, li, a, b) in &lines {
        let covered = lines.iter().any(|(j, lj, c, d)| {
            j != i
                && on_segment(*a, *c, *d)
                && on_segment(*b, *c, *d)
                && (rank(*lj) > rank(*li)
                    || (rank(*lj) == rank(*li)
                        && (len(*c, *d) > len(*a, *b) + tol
                            || (len(*c, *d) >= len(*a, *b) - tol && j < i))))
        });
        if covered {
            keep[*i] = false;
        }
    }
    let vis_circles: Vec<(P, f64)> = scene
        .prims
        .iter()
        .filter(|p| p.layer == Layer::Visible)
        .filter_map(|p| match &p.shape {
            Shape::Circle { c, r } => Some((*c, *r)),
            _ => None,
        })
        .collect();
    let mut seen: Vec<(P, f64)> = Vec::new();
    for (i, p) in scene.prims.iter().enumerate() {
        let Shape::Circle { c, r } = &p.shape else {
            continue;
        };
        let same = |(vc, vr): &(P, f64)| len(*vc, *c) < tol && (vr - r).abs() < tol;
        if p.layer == Layer::Hidden && vis_circles.iter().any(same) {
            keep[i] = false;
        } else if p.layer == Layer::Visible {
            if seen.iter().any(same) {
                keep[i] = false;
            } else {
                seen.push((*c, *r));
            }
        }
    }
    let mut i = 0;
    scene.prims.retain(|_| {
        i += 1;
        keep[i - 1]
    });
    let mut j = 0;
    sources.retain(|_| {
        j += 1;
        keep[j - 1]
    });
}

/// The part of a triangle on the kept side of the section plane, as up to two triangles.
fn clip_triangle(t: &[V3; 3], cl: &Clip) -> Vec<[V3; 3]> {
    let d: Vec<f64> = t.iter().map(|p| cl.side(*p)).collect();
    if d.iter().all(|x| *x >= 0.0) {
        return vec![*t];
    }
    if d.iter().all(|x| *x <= 0.0) {
        return Vec::new();
    }
    let mut poly: Vec<V3> = Vec::new();
    for i in 0..3 {
        let (a, b) = (i, (i + 1) % 3);
        if d[a] >= 0.0 {
            poly.push(t[a]);
        }
        if (d[a] > 0.0 && d[b] < 0.0) || (d[a] < 0.0 && d[b] > 0.0) {
            let f = d[a] / (d[a] - d[b]);
            poly.push([
                t[a][0] + (t[b][0] - t[a][0]) * f,
                t[a][1] + (t[b][1] - t[a][1]) * f,
                t[a][2] + (t[b][2] - t[a][2]) * f,
            ]);
        }
    }
    (1..poly.len().saturating_sub(1))
        .map(|i| [poly[0], poly[i], poly[i + 1]])
        .collect()
}

/// The axis of every cylindrical face seen from the side, from one end of the face to the other.
fn cylinder_axes(brep: &BRep, d: &BodyData, f: &ViewFrame) -> Vec<(V3, V3)> {
    let t = &d.tess;
    let m = &t.mesh;
    let mut out = Vec::new();
    for (fi, fk) in t.face_keys.iter().enumerate() {
        let Some(face) = brep.faces.get(*fk) else {
            continue;
        };
        let Some(SurfaceGeom::Cylinder(cy)) = brep.surfaces.get(face.surface) else {
            continue;
        };
        let axis = vv(&cy.frame.z);
        if dot(axis, f.dir).abs() > 0.98 {
            continue;
        }
        let o = v3(&cy.frame.origin);
        let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for tri in 0..m.triangle_count() {
            if m.tri_face[tri] as usize != fi {
                continue;
            }
            for p in m.triangle(tri) {
                let z = dot([p.x - o[0], p.y - o[1], p.z - o[2]], axis);
                zmin = zmin.min(z);
                zmax = zmax.max(z);
            }
        }
        if zmax - zmin > 1e-9 {
            let at = |z: f64| [o[0] + axis[0] * z, o[1] + axis[1] * z, o[2] + axis[2] * z];
            out.push((at(zmin), at(zmax)));
        }
    }
    out
}
