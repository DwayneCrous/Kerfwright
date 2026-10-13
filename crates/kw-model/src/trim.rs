//! Trim: cut a surface with a plane or another surface and keep one side.
//!
//! Every edge of the surface is cut where it crosses the tool; every face is cut along the curve its
//! surface makes with the tool (`kw_ssi::intersect`), found in the face's own parameter space. The
//! pieces of the boundary that lie on the kept side, the arcs of the cut curve inside the face and the
//! cut points that join them make the loops of the new faces. Cut arcs become open edges of the result;
//! the pieces of shared edges stay shared.

use crate::profile::ModelError;
use kw_brep::sew::{BuildOptions, EdgeDesc, FaceDesc, LoopDesc, Soup, UseDesc};
use kw_brep::uvgeom::{point_in_loops, winding, UV};
use kw_brep::{BRep, BodyId, BodyKind, BuildResult, EntityRef, LoopKind, Origin, Sense};
use kw_math::{Point3, Vector3};
use kw_nurbs::fit::transform_curve;
use kw_nurbs::proj::{curve_param_of, surface_param_of};
use kw_nurbs::{Curve3, CurveGeom, Surface, SurfaceGeom};
use kw_ssi::{intersect, SsiOutcome, Window, DEFAULT_TOL};
use std::collections::HashMap;
use std::f64::consts::TAU;

fn bad(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// A surface used as the cutting tool, and which way its normal points.
#[derive(Debug, Clone)]
pub struct TrimTool {
    pub surface: SurfaceGeom,
    /// `+1` if the tool's positive side is the side its surface normal points to, `-1` if the other.
    pub sign: f64,
}

struct Side<'a> {
    tool: &'a TrimTool,
    keep: f64,
}

impl Side<'_> {
    /// Positive on the side to keep.
    fn of(&self, p: &Point3<f64>) -> f64 {
        let d = match &self.tool.surface {
            SurfaceGeom::Plane(pl) => (p - pl.frame.origin).dot(&pl.frame.z),
            s => {
                let r = surface_param_of(s, p, None);
                let q = s.point_at(r.u, r.v);
                (p - q).dot(&s.normal_at(r.u, r.v))
            }
        };
        d * self.tool.sign * self.keep
    }
}

/// Where along `range` of `curve` the tool is crossed (parameters strictly inside), by sampling and
/// bisecting.
fn crossings(
    curve: &CurveGeom,
    range: (f64, f64),
    side: &Side,
    scale: f64,
) -> Result<Vec<f64>, ModelError> {
    const N: usize = 96;
    let at = |t: f64| side.of(&curve.point_at(t));
    let tol = 1e-9 * scale;
    let ts: Vec<f64> = (0..=N)
        .map(|k| range.0 + (range.1 - range.0) * k as f64 / N as f64)
        .collect();
    let fs: Vec<f64> = ts.iter().map(|t| at(*t)).collect();
    if fs.iter().all(|f| f.abs() < tol) {
        return Err(bad("an edge of the surface lies on the tool"));
    }
    let mut out = Vec::new();
    for k in 0..N {
        let (fa, fb) = (fs[k], fs[k + 1]);
        if fa.abs() < tol && k > 0 || fb.abs() < tol && k + 1 < N {
            // the tool passes through a sample: treat the crossing by the sign on either side
            if fa.abs() < tol || fb.abs() < tol {
                let probe = |t: f64| at(t);
                let (before, after) = (
                    probe(ts[k] - 1e-6 * (range.1 - range.0).abs()),
                    probe(ts[k + 1] + 1e-6 * (range.1 - range.0).abs()),
                );
                if fa.abs() < tol && before * fb < 0.0 {
                    out.push(ts[k]);
                } else if fb.abs() < tol && fa * after < 0.0 && k + 1 < N {
                    out.push(ts[k + 1]);
                }
            }
            continue;
        }
        if fa * fb < 0.0 {
            let (mut lo, mut hi) = (ts[k], ts[k + 1]);
            let flo = fa;
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if at(mid) * flo > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            out.push(0.5 * (lo + hi));
        }
    }
    out.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    Ok(out)
}

/// One run of a face loop along one piece of an edge.
#[derive(Clone)]
struct Dp {
    edge: usize,
    piece: usize,
    reversed: bool,
    kept: bool,
    /// The face loop the piece belongs to and its place in it.
    lp: usize,
    /// The cut node it ends at, if it does.
    end_node: Option<usize>,
    start_node: Option<usize>,
    use_pcurve: Option<CurveGeom>,
    /// Sample points (parameter space) in traversal order.
    uv: Vec<UV>,
}

struct Node {
    point: Point3<f64>,
    uv: UV,
    after: usize,
}

/// An arc of the cut curve inside a face.
struct Arc {
    curve: CurveGeom,
    range: (f64, f64),
    pcurve: CurveGeom,
    /// Nodes (real: index into the face's nodes; virtual: `usize::MAX - k`) at the start and at the end.
    a: usize,
    b: usize,
    uv: Vec<UV>,
}

fn is_virtual(n: usize) -> bool {
    n >= usize::MAX / 2
}

fn area(poly: &[UV], sgn: f64) -> f64 {
    let n = poly.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (poly[i], poly[(i + 1) % n]);
        a += p.0 * (sgn * q.1) - q.0 * (sgn * p.1);
    }
    0.5 * a
}

/// Parameter of the projection of `p` on `curve` within `range` (wrapping a periodic curve).
fn param_in(curve: &CurveGeom, p: &Point3<f64>, range: (f64, f64)) -> (f64, f64) {
    let mid = 0.5 * (range.0 + range.1);
    let r = curve_param_of(curve, p, Some(mid));
    let mut t = r.t;
    if matches!(curve, CurveGeom::Circle(_) | CurveGeom::Ellipse(_)) {
        while t < range.0 - 1e-9 {
            t += TAU;
        }
        while t > range.1 + 1e-9 {
            t -= TAU;
        }
    }
    (t, r.distance)
}

/// The window of parameters an intersection with `face` has to cover.
fn window_of(s: &SurfaceGeom, lo: UV, hi: UV) -> Window {
    let widen = |lo: f64, hi: f64, period: Option<f64>| -> (f64, f64) {
        let span = hi - lo;
        match period {
            Some(p) if span >= p - 1e-9 => (lo, lo + p),
            Some(p) => {
                let m = (0.05 * span).max(1e-3).min(0.5 * (p - span).max(0.0));
                (lo - m, hi + m)
            }
            None => {
                let m = (0.05 * span).max(1e-3);
                (lo - m, hi + m)
            }
        }
    };
    let ((du0, du1), (dv0, dv1)) = s.domain();
    let clamp = |r: (f64, f64), dom: (f64, f64), periodic: bool| {
        if periodic || !dom.0.is_finite() || !dom.1.is_finite() {
            r
        } else {
            (r.0.max(dom.0), r.1.min(dom.1))
        }
    };
    let u = widen(lo.0, hi.0, s.period_u());
    let v = widen(lo.1, hi.1, s.period_v());
    Window {
        u: clamp(u, (du0, du1), s.period_u().is_some()),
        v: clamp(v, (dv0, dv1), s.period_v().is_some()),
    }
}

/// The window of the tool: its whole domain, or a large one around the part of the world that matters.
fn tool_window(t: &SurfaceGeom, centre: Point3<f64>, reach: f64) -> Window {
    let ((u0, u1), (v0, v1)) = t.domain();
    let fin = |a: f64, b: f64, c: f64| {
        if a.is_finite() && b.is_finite() {
            (a, b)
        } else {
            (c - reach, c + reach)
        }
    };
    let r = surface_param_of(t, &centre, None);
    Window {
        u: fin(u0, u1, r.u),
        v: fin(v0, v1, r.v),
    }
}

/// Cut `body` with the tool and keep the side where `keep_positive` says.
pub fn trim_sheet(
    rep: &mut BRep,
    body: BodyId,
    tool: &TrimTool,
    keep_positive: bool,
) -> Result<BuildResult, ModelError> {
    if rep.bodies[body].kind != BodyKind::Sheet {
        return Err(bad("only a surface can be trimmed"));
    }
    let bb = rep.body_bbox(body);
    let scale = 1.0 + bb.min.coords.amax().max(bb.max.coords.amax());
    let reach = 4.0 * (bb.max - bb.min).norm() + 1.0;
    let centre = Point3::from((bb.min.coords + bb.max.coords) * 0.5);
    let side = Side {
        tool,
        keep: if keep_positive { 1.0 } else { -1.0 },
    };
    let face_ids = rep.body_faces(body);
    let mut src = Soup::default();
    src.add_body(rep, body, true);

    // ---- the edges, cut where they cross the tool
    struct Cut {
        breaks: Vec<f64>,
        kept: Vec<bool>,
    }
    let mut cuts: Vec<Cut> = Vec::new();
    for e in &src.edges {
        let Some(curve) = &e.curve else {
            let k = side.of(&e.start) > 0.0;
            cuts.push(Cut {
                breaks: vec![e.range.0, e.range.1],
                kept: vec![k],
            });
            continue;
        };
        for p in [&e.start, &e.end] {
            if side.of(p).abs() < 1e-9 * scale {
                return Err(bad(
                    "a vertex of the surface lies on the tool: move the tool a little",
                ));
            }
        }
        let c = crossings(curve, e.range, &side, scale)?;
        let mut breaks = vec![e.range.0];
        breaks.extend(c);
        breaks.push(e.range.1);
        let kept = breaks
            .windows(2)
            .map(|w| side.of(&curve.point_at(0.5 * (w[0] + w[1]))) > 0.0)
            .collect();
        cuts.push(Cut { breaks, kept });
    }
    let mut out = Soup::default();
    let mut piece_edges: HashMap<(usize, usize), usize> = HashMap::new();
    let mut any_cut = false;
    let mut any_dropped = false;
    let mut result_faces = 0usize;
    for (fi, face) in src.faces.iter().enumerate() {
        let fid = face_ids[fi];
        let sgn = if face.sense == Sense::Forward {
            1.0
        } else {
            -1.0
        };
        let uv_loops = rep
            .face_uv_loops(fid, 3)
            .ok_or_else(|| bad("a face has no parameter loops"))?;
        let (lo, hi) = rep
            .face_uv_bbox(fid)
            .ok_or_else(|| bad("a face has no parameter box"))?;
        // ---- the pieces of the boundary, in loop order
        let mut dps: Vec<Dp> = Vec::new();
        let mut nodes: Vec<Node> = Vec::new();
        for (li, l) in face.loops.iter().enumerate() {
            for u in l.uses.iter() {
                let cut = &cuts[u.edge];
                let m = cut.kept.len();
                let order: Vec<usize> = if u.reversed {
                    (0..m).rev().collect()
                } else {
                    (0..m).collect()
                };
                for (k, &p) in order.iter().enumerate() {
                    let (a, b) = (cut.breaks[p], cut.breaks[p + 1]);
                    let (t_from, t_to) = if u.reversed { (b, a) } else { (a, b) };
                    let uv: Vec<UV> = (0..=8)
                        .map(|i| {
                            let t = t_from + (t_to - t_from) * i as f64 / 8.0;
                            match &u.pcurve {
                                Some(pc) => {
                                    let q = pc.point_at(t);
                                    (q.x, q.y)
                                }
                                None => (0.0, 0.0),
                            }
                        })
                        .collect();
                    dps.push(Dp {
                        edge: u.edge,
                        piece: p,
                        reversed: u.reversed,
                        kept: cut.kept[p],
                        lp: li,
                        end_node: None,
                        start_node: None,
                        use_pcurve: u.pcurve.clone(),
                        uv,
                    });
                    // a node where the tool is crossed between this piece and the next one on the edge
                    if k + 1 < order.len() {
                        let t_cross = if u.reversed { a } else { b };
                        let curve = src.edges[u.edge]
                            .curve
                            .as_ref()
                            .expect("cut edge has a curve");
                        let point = curve.point_at(t_cross);
                        let uvq = match &u.pcurve {
                            Some(pc) => {
                                let q = pc.point_at(t_cross);
                                (q.x, q.y)
                            }
                            None => (0.0, 0.0),
                        };
                        let id = nodes.len();
                        let before = dps.len() - 1;
                        nodes.push(Node {
                            point,
                            uv: uvq,
                            after: before + 1,
                        });
                        dps[before].end_node = Some(id);
                        // the next piece is pushed in the next iteration: mark its start afterwards
                        let _ = id;
                    }
                }
            }
        }
        // start nodes: a piece that follows a node on the same use starts at it
        for (id, n) in nodes.iter().enumerate() {
            if let Some(d) = dps.get_mut(n.after) {
                d.start_node = Some(id);
            }
        }
        // ---- the cut curve in this face
        let mut arcs: Vec<Arc> = Vec::new();
        let outcome = intersect(
            &face.surface,
            window_of(&face.surface, lo, hi),
            &tool.surface,
            tool_window(&tool.surface, centre, reach),
            DEFAULT_TOL,
        );
        let curves = match outcome {
            Ok(SsiOutcome::Curves { curves, .. }) => curves,
            Ok(SsiOutcome::Coincident { .. }) => {
                return Err(bad("the tool lies on a face of the surface"))
            }
            Ok(_) => Vec::new(),
            Err(e) => {
                return Err(bad(format!(
                    "the tool cannot be intersected with a face of the surface ({e})"
                )))
            }
        };
        let tol3 = 1e-5 * scale;
        let (pu, pv) = (face.surface.period_u(), face.surface.period_v());
        let mut virtuals: Vec<Point3<f64>> = Vec::new();
        let mut virtual_id = |p: Point3<f64>| -> usize {
            if let Some(i) = virtuals.iter().position(|q| (q - p).norm() < tol3) {
                return usize::MAX - i;
            }
            virtuals.push(p);
            usize::MAX - (virtuals.len() - 1)
        };
        for c in &curves {
            // break the curve where it meets the boundary
            let mut ss: Vec<f64> = vec![c.range.0, c.range.1];
            for n in &nodes {
                let (s, d) = param_in(&c.curve, &n.point, c.range);
                if d < tol3 && s > c.range.0 + 1e-12 && s < c.range.1 - 1e-12 {
                    ss.push(s);
                }
            }
            ss.sort_by(|a, b| a.partial_cmp(b).unwrap());
            ss.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
            for w in ss.windows(2) {
                let (s0, s1) = (w[0], w[1]);
                let sm = 0.5 * (s0 + s1);
                let q = c.pcurve_a.point_at(sm);
                // shift by whole periods until the arc lies inside the face
                let mut found: Option<(f64, f64)> = None;
                'shift: for i in -2i32..=2 {
                    for j in -2i32..=2 {
                        if (i != 0 && pu.is_none()) || (j != 0 && pv.is_none()) {
                            continue;
                        }
                        let sh = (i as f64 * pu.unwrap_or(0.0), j as f64 * pv.unwrap_or(0.0));
                        if point_in_loops((q.x + sh.0, q.y + sh.1), &uv_loops) {
                            found = Some(sh);
                            break 'shift;
                        }
                    }
                }
                let Some(sh) = found else { continue };
                let pcurve = if sh == (0.0, 0.0) {
                    c.pcurve_a.clone()
                } else {
                    transform_curve(
                        &c.pcurve_a,
                        &kw_math::Transform3::translation(Vector3::new(sh.0, sh.1, 0.0)),
                    )
                };
                let uvat = |s: f64| {
                    let p = pcurve.point_at(s);
                    (p.x, p.y)
                };
                let near_node = |s: f64| -> Option<usize> {
                    let p3 = c.curve.point_at(s);
                    let uvp = uvat(s);
                    nodes
                        .iter()
                        .enumerate()
                        .filter(|(_, n)| (n.point - p3).norm() < tol3)
                        .min_by(|(_, x), (_, y)| {
                            let dx = (x.uv.0 - uvp.0).hypot(x.uv.1 - uvp.1);
                            let dy = (y.uv.0 - uvp.0).hypot(y.uv.1 - uvp.1);
                            dx.partial_cmp(&dy).unwrap()
                        })
                        .map(|(i, _)| i)
                };
                let (na, nb) = (near_node(s0), near_node(s1));
                let a = na.unwrap_or_else(|| virtual_id(c.curve.point_at(s0)));
                let b = nb.unwrap_or_else(|| virtual_id(c.curve.point_at(s1)));
                // an open end of the curve inside the face is a failed intersection
                let open_end = |s: f64, n: Option<usize>| {
                    n.is_none() && !c.closed && (s - c.range.0).abs() < 1e-12
                        || n.is_none() && !c.closed && (s - c.range.1).abs() < 1e-12
                };
                if open_end(s0, na) || open_end(s1, nb) {
                    return Err(bad(
                        "the cut curve ends inside a face: the tool is only partly across the surface",
                    ));
                }
                let uv: Vec<UV> = (0..=16)
                    .map(|i| uvat(s0 + (s1 - s0) * i as f64 / 16.0))
                    .collect();
                arcs.push(Arc {
                    curve: c.curve.clone(),
                    range: (s0, s1),
                    pcurve,
                    a,
                    b,
                    uv,
                });
            }
        }
        // ---- walk the loops of the kept region
        struct Walked {
            /// Items in order: a piece (index into `dps`) or an arc (index, traversed forwards).
            items: Vec<(bool, usize, bool)>,
            poly: Vec<UV>,
        }
        let mut walked: Vec<Walked> = Vec::new();
        let mut used = vec![false; dps.len()];
        let mut arc_used = vec![false; arcs.len()];
        let next_in_loop = |i: usize| -> usize {
            // the piece after `i` in the loop order
            let d = &dps[i];
            let same: Vec<usize> = (0..dps.len()).filter(|k| dps[*k].lp == d.lp).collect();
            let at = same.iter().position(|k| *k == i).unwrap();
            same[(at + 1) % same.len()]
        };
        for start in 0..dps.len() {
            if used[start] || !dps[start].kept {
                continue;
            }
            let mut items = Vec::new();
            let mut poly: Vec<UV> = Vec::new();
            let mut cur = start;
            let mut guard = 0;
            loop {
                guard += 1;
                if guard > 4 * (dps.len() + arcs.len()) + 8 {
                    return Err(bad("the trimmed face could not be traced"));
                }
                used[cur] = true;
                items.push((false, cur, true));
                poly.extend(dps[cur].uv.iter().take(8).copied());
                let nxt = match dps[cur].end_node {
                    None => next_in_loop(cur),
                    Some(n) => {
                        // the region leaves the boundary here: follow the cut curve to the next cut node
                        let mut at = n;
                        let mut last_arc = usize::MAX;
                        loop {
                            let found = arcs.iter().enumerate().find(|(k, a)| {
                                *k != last_arc && !arc_used[*k] && (a.a == at || a.b == at)
                            });
                            let Some((k, a)) = found else {
                                return Err(bad(
                                    "a cut point has no cut curve: the tool touches the surface only at its edge",
                                ));
                            };
                            let forward = a.a == at;
                            arc_used[k] = true;
                            items.push((true, k, forward));
                            if forward {
                                poly.extend(a.uv.iter().take(16).copied());
                            } else {
                                poly.extend(a.uv.iter().skip(1).rev().copied());
                            }
                            last_arc = k;
                            at = if forward { a.b } else { a.a };
                            if !is_virtual(at) {
                                break;
                            }
                        }
                        nodes[at].after
                    }
                };
                if nxt == start {
                    break;
                }
                if used[nxt] {
                    return Err(bad("the trimmed face could not be traced"));
                }
                if !dps[nxt].kept {
                    return Err(bad(
                        "the tool leaves the surface and returns on the same side: it is tangent to an edge",
                    ));
                }
                cur = nxt;
            }
            walked.push(Walked { items, poly });
        }
        // closed cut curves inside the face (no cut point on them)
        let mut islands: Vec<Walked> = Vec::new();
        for k in 0..arcs.len() {
            if arc_used[k] || !(is_virtual(arcs[k].a) && is_virtual(arcs[k].b)) {
                continue;
            }
            // chain through virtual nodes until the chain closes
            let mut items = vec![(true, k, true)];
            arc_used[k] = true;
            let mut poly: Vec<UV> = arcs[k].uv.iter().take(16).copied().collect();
            let mut at = arcs[k].b;
            let first = arcs[k].a;
            let mut guard = 0;
            while at != first {
                guard += 1;
                if guard > arcs.len() + 2 {
                    return Err(bad("a cut curve inside a face does not close"));
                }
                let found = arcs
                    .iter()
                    .enumerate()
                    .find(|(j, a)| !arc_used[*j] && (a.a == at || a.b == at));
                let Some((j, a)) = found else {
                    return Err(bad("a cut curve inside a face does not close"));
                };
                let forward = a.a == at;
                arc_used[j] = true;
                items.push((true, j, forward));
                if forward {
                    poly.extend(a.uv.iter().take(16).copied());
                } else {
                    poly.extend(a.uv.iter().skip(1).rev().copied());
                }
                at = if forward { a.b } else { a.a };
            }
            islands.push(Walked { items, poly });
        }
        // an island keeps the side it surrounds: its inside is on the kept side when the surface
        // there is, and then it is the outer loop of a disc; else a hole
        for mut isl in islands {
            let c = {
                let n = isl.poly.len() as f64;
                let s = isl
                    .poly
                    .iter()
                    .fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
                (s.0 / n, s.1 / n)
            };
            let inside = if winding(c, &isl.poly) != 0 {
                c
            } else {
                // a concave island: the midpoint of a chord between two boundary points that is inside
                let n = isl.poly.len();
                let mut best = c;
                'find: for i in 0..n {
                    for j in i + 2..n {
                        let m = (
                            0.5 * (isl.poly[i].0 + isl.poly[j].0),
                            0.5 * (isl.poly[i].1 + isl.poly[j].1),
                        );
                        if winding(m, &isl.poly) != 0 {
                            best = m;
                            break 'find;
                        }
                    }
                }
                best
            };
            let p3 = face.surface.point_at(inside.0, inside.1);
            let inside_kept = side.of(&p3) > 0.0;
            let ccw = area(&isl.poly, sgn) > 0.0;
            // the loop must run with the region it bounds on its left: counter-clockwise for the
            // disc that is kept, clockwise for a hole in a kept region
            let want_ccw = inside_kept;
            if ccw != want_ccw {
                isl.items.reverse();
                for it in &mut isl.items {
                    it.2 = !it.2;
                }
                isl.poly.reverse();
            }
            if !inside_kept {
                // a hole in the face: the face's own outer loop stays; nothing to do if that is
                // dropped (the face is then all on the other side)
            }
            walked.push(isl);
        }
        if walked.is_empty() {
            any_dropped = true;
            continue;
        }
        if !arcs.is_empty() || dps.iter().any(|d| !d.kept) {
            any_cut = true;
        }
        // ---- outer loops and the holes inside them
        let mut outers: Vec<usize> = Vec::new();
        let mut inners: Vec<usize> = Vec::new();
        for (k, w) in walked.iter().enumerate() {
            if area(&w.poly, sgn) > 0.0 {
                outers.push(k);
            } else {
                inners.push(k);
            }
        }
        if outers.is_empty() {
            // a face whose kept part is all hole boundaries cannot exist
            return Err(bad("a trimmed face has no outer boundary"));
        }
        let mut groups: Vec<(usize, Vec<usize>)> =
            outers.iter().map(|o| (*o, Vec::new())).collect();
        for h in inners {
            let p = walked[h].poly[0];
            let mut best: Option<(usize, f64)> = None;
            for (gi, (o, _)) in groups.iter().enumerate() {
                if winding(p, &walked[*o].poly) != 0 {
                    let a = area(&walked[*o].poly, sgn).abs();
                    if best.is_none_or(|(_, ba)| a < ba) {
                        best = Some((gi, a));
                    }
                }
            }
            match best {
                Some((gi, _)) => groups[gi].1.push(h),
                None => return Err(bad("a hole of a trimmed face lies outside its boundary")),
            }
        }
        // ---- build the faces into the result soup
        let parts = groups.len();
        for (gi, (outer, holes)) in groups.into_iter().enumerate() {
            let mut loops: Vec<LoopDesc> = Vec::new();
            for (k, kind) in std::iter::once((outer, LoopKind::Outer))
                .chain(holes.into_iter().map(|h| (h, LoopKind::Inner)))
            {
                let mut uses: Vec<UseDesc> = Vec::new();
                for &(is_arc, idx, forward) in &walked[k].items {
                    if is_arc {
                        let a = &arcs[idx];
                        let e = out.edges.len();
                        out.edges.push(EdgeDesc {
                            curve: Some(a.curve.clone()),
                            range: a.range,
                            start: a.curve.point_at(a.range.0),
                            end: a.curve.point_at(a.range.1),
                            tolerance: 0.0,
                        });
                        uses.push(UseDesc {
                            edge: e,
                            reversed: !forward,
                            pcurve: Some(a.pcurve.clone()),
                            hint: None,
                        });
                    } else {
                        let d = &dps[idx];
                        let cut = &cuts[d.edge];
                        let e = *piece_edges.entry((d.edge, d.piece)).or_insert_with(|| {
                            let desc = &src.edges[d.edge];
                            let (a, b) = (cut.breaks[d.piece], cut.breaks[d.piece + 1]);
                            let (start, end) = match &desc.curve {
                                Some(c) => (c.point_at(a), c.point_at(b)),
                                None => (desc.start, desc.end),
                            };
                            out.edges.push(EdgeDesc {
                                curve: desc.curve.clone(),
                                range: (a, b),
                                start,
                                end,
                                tolerance: desc.tolerance,
                            });
                            out.edges.len() - 1
                        });
                        uses.push(UseDesc {
                            edge: e,
                            reversed: d.reversed,
                            pcurve: d.use_pcurve.clone(),
                            hint: None,
                        });
                    }
                }
                loops.push(LoopDesc { kind, uses });
            }
            let origin = if parts > 1 {
                Origin::SplitFrom {
                    parent: EntityRef::Face(fid),
                    ordinal: gi as u32,
                }
            } else if arcs.is_empty() && dps.iter().all(|d| d.kept) {
                Origin::Preserved(EntityRef::Face(fid))
            } else {
                Origin::Modified(EntityRef::Face(fid))
            };
            out.faces.push(FaceDesc {
                surface: face.surface.clone(),
                sense: face.sense,
                loops,
                origin,
            });
            result_faces += 1;
        }
    }
    if result_faces == 0 {
        return Err(bad("the tool removes the whole surface"));
    }
    if !any_cut && !any_dropped {
        return Err(bad("the tool does not cut the surface"));
    }
    let r = out.build(
        rep,
        &BuildOptions {
            vertex_tol: 1e-6 * scale,
            one_body: true,
        },
    );
    match r.bodies.as_slice() {
        [b] => Ok(BuildResult {
            body: *b,
            history: r.history,
        }),
        _ => Err(bad(format!(
            "the trimmed surface could not be built: {}",
            r.rejected.join("; ")
        ))),
    }
}
