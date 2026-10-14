//! Loft (skinning through sections) and sweep (profile transported along a path).
//!
//! Both build, for every profile edge, a tensor-product NURBS surface that interpolates the
//! compatible section curves in the second direction. All faces use the *same* section
//! parameters, so the rail curve through each profile vertex is shared exactly by the two faces
//! meeting there, and every edge pcurve is an exact straight line in `(u, v)`.

use crate::profile::{Chain, ModelError, Profile, ProfileEdge};
use kw_brep::{
    add_face_auto, BRep, BodyKind, BuildResult, CoedgeSpec, EntityRef, LoopKind, LoopSpec, Origin,
    Sense, ShapeHistory, ShellId,
};
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_nurbs::build::make_arc;
use kw_nurbs::geom::{Line, Plane};
use kw_nurbs::{Curve3, CurveGeom, KnotVector, NurbsCurve, NurbsSurface, SurfaceGeom};
use nalgebra::{DMatrix, DVector};

fn bad(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// Profile edge as a NURBS curve in *traversal* direction with domain [0, 1].
fn edge_to_nurbs(e: &ProfileEdge) -> Result<NurbsCurve, ModelError> {
    let c = match &e.curve {
        CurveGeom::Line(l) => {
            let (a, b) = (l.origin + l.dir * e.range.0, l.origin + l.dir * e.range.1);
            NurbsCurve::new_polynomial(
                KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
                vec![a, b],
            )
            .unwrap()
        }
        CurveGeom::Circle(ci) => make_arc(&ci.frame, ci.radius, e.range.0, e.range.1 - e.range.0)
            .map_err(|er| bad(format!("{er:?}")))?,
        CurveGeom::Nurbs(n) => {
            let (d0, d1) = n.domain();
            if (e.range.0 - d0).abs() > 1e-12 || (e.range.1 - d1).abs() > 1e-12 {
                return Err(bad(
                    "partial NURBS profile edges are not supported in loft/sweep",
                ));
            }
            n.clone()
        }
        _ => {
            return Err(bad(
                "profile edge type not supported in loft/sweep (use lines, circular arcs or NURBS)",
            ))
        }
    };
    let c = if e.reversed { c.reversed() } else { c };
    Ok(normalize(&c))
}

fn normalize(c: &NurbsCurve) -> NurbsCurve {
    let (a, b) = c.domain();
    let k: Vec<f64> = c
        .knots()
        .as_slice()
        .iter()
        .map(|x| (x - a) / (b - a))
        .collect();
    NurbsCurve::new(
        KnotVector::new(c.degree(), k).unwrap(),
        c.points().to_vec(),
        c.weights().to_vec(),
    )
    .unwrap()
}

/// Bring the curves to a common degree and knot vector.
fn make_compatible(curves: &mut [NurbsCurve]) {
    let p = curves.iter().map(|c| c.degree()).max().unwrap();
    for c in curves.iter_mut() {
        *c = c.elevate_degree(p - c.degree());
    }
    // union of interior knots with maximal multiplicities
    let mut all: Vec<f64> = Vec::new();
    for c in curves.iter() {
        for (k, m) in c.knots().distinct() {
            if k > 1e-12 && k < 1.0 - 1e-12 {
                let have = all.iter().filter(|x| (**x - k).abs() < 1e-12).count();
                for _ in have..m {
                    all.push(k);
                }
            }
        }
    }
    for c in curves.iter_mut() {
        let mut need = Vec::new();
        for k in &all {
            let have = c.knots().multiplicity(*k);
            let want = all.iter().filter(|x| (**x - *k).abs() < 1e-12).count();
            let seen = need
                .iter()
                .filter(|x: &&f64| (**x - *k).abs() < 1e-12)
                .count();
            if have + seen < want {
                need.push(*k);
            }
        }
        *c = c.refine_knots(&need);
    }
}

/// Global interpolation of homogeneous points at parameters `v` with degree `q` (P&T A9.1).
struct Interp {
    knots: KnotVector,
    lu: nalgebra::LU<f64, nalgebra::Dyn, nalgebra::Dyn>,
}

impl Interp {
    fn new(v: &[f64], q: usize) -> Result<Self, ModelError> {
        let m = v.len();
        let mut k = vec![0.0; q + 1];
        for j in 1..m - q {
            k.push((j..j + q).map(|i| v[i]).sum::<f64>() / q as f64);
        }
        k.extend(vec![1.0; q + 1]);
        let knots = KnotVector::new(q, k).map_err(|e| bad(format!("{e:?}")))?;
        let mut a = DMatrix::<f64>::zeros(m, m);
        for (j, vj) in v.iter().enumerate() {
            let span = knots.find_span(*vj);
            let nb = knots.basis_funs(span, *vj);
            for (i, b) in nb.iter().enumerate() {
                a[(j, span - q + i)] = *b;
            }
        }
        Ok(Self { knots, lu: a.lu() })
    }

    /// Control values for the data `h` (one value per section).
    fn solve(&self, h: &[f64]) -> Result<Vec<f64>, ModelError> {
        let r = self
            .lu
            .solve(&DVector::from_column_slice(h))
            .ok_or_else(|| bad("singular interpolation system"))?;
        Ok(r.iter().copied().collect())
    }
}

struct Skin {
    /// One surface per profile edge, with `v` along the sections.
    surfaces: Vec<NurbsSurface>,
    /// Rail curve through each profile vertex (junction `i` = start of edge `i`).
    rails: Vec<NurbsCurve>,
}

fn skin_loop(
    sections: &[Vec<NurbsCurve>],
    v: &[f64],
    q: usize,
    closed: bool,
) -> Result<Skin, ModelError> {
    let m = sections.len();
    let n_edges = sections[0].len();
    let interp = Interp::new(v, q)?;
    let mut surfaces = Vec::new();
    let mut rails = Vec::new();
    for e in 0..n_edges {
        let col: Vec<&NurbsCurve> = sections.iter().map(|s| &s[e]).collect();
        let nu = col[0].points().len();
        // homogeneous control net
        let mut net: Vec<[Vec<f64>; 4]> = Vec::with_capacity(nu);
        for i in 0..nu {
            let mut comps: [Vec<f64>; 4] = Default::default();
            for c in &col {
                let (p, w) = (c.points()[i], c.weights()[i]);
                comps[0].push(p.x * w);
                comps[1].push(p.y * w);
                comps[2].push(p.z * w);
                comps[3].push(w);
            }
            let mut out: [Vec<f64>; 4] = Default::default();
            for k in 0..4 {
                out[k] = interp.solve(&comps[k])?;
            }
            net.push(out);
        }
        let nv = m;
        let mut pts = Vec::with_capacity(nu * nv);
        let mut ws = Vec::with_capacity(nu * nv);
        for row in net.iter() {
            for j in 0..nv {
                let w = row[3][j];
                if w <= 0.0 {
                    return Err(bad("interpolated weight is not positive"));
                }
                pts.push(Point3::new(row[0][j] / w, row[1][j] / w, row[2][j] / w));
                ws.push(w);
            }
        }
        surfaces.push(
            NurbsSurface::new(col[0].knots().clone(), interp.knots.clone(), pts, ws)
                .map_err(|e| bad(format!("{e:?}")))?,
        );
        // rail at the start of this edge
        let start: Vec<Point3<f64>> = col.iter().map(|c| c.points()[0]).collect();
        rails.push(interpolate_points(&interp, &start)?);
        // an open chain has one more rail: through the end of its last edge
        if !closed && e == n_edges - 1 {
            let end: Vec<Point3<f64>> = col.iter().map(|c| *c.points().last().unwrap()).collect();
            rails.push(interpolate_points(&interp, &end)?);
        }
    }
    Ok(Skin { surfaces, rails })
}

fn interpolate_points(interp: &Interp, pts: &[Point3<f64>]) -> Result<NurbsCurve, ModelError> {
    let mut comps: [Vec<f64>; 3] = Default::default();
    for p in pts {
        comps[0].push(p.x);
        comps[1].push(p.y);
        comps[2].push(p.z);
    }
    let sol: Vec<Vec<f64>> = comps
        .iter()
        .map(|c| interp.solve(c))
        .collect::<Result<_, _>>()?;
    let cp: Vec<Point3<f64>> = (0..pts.len())
        .map(|i| Point3::new(sol[0][i], sol[1][i], sol[2][i]))
        .collect();
    NurbsCurve::new_polynomial(interp.knots.clone(), cp).map_err(|e| bad(format!("{e:?}")))
}

pub(crate) fn line_pc(o: (f64, f64), d: (f64, f64)) -> CurveGeom {
    CurveGeom::Line(Line {
        origin: Point3::new(o.0, o.1, 0.0),
        dir: Vector3::new(d.0, d.1, 0.0),
    })
}

/// A profile as ordered loops of compatible NURBS edges (traversal direction) plus the kind of each.
struct Sec {
    loops: Vec<Vec<NurbsCurve>>,
    /// Whether each loop returns to its start (an open chain does not).
    closed: Vec<bool>,
}

fn section_from(profile: &Profile) -> Result<Sec, ModelError> {
    let mut loops = vec![profile
        .outer
        .edges
        .iter()
        .map(edge_to_nurbs)
        .collect::<Result<Vec<_>, _>>()?];
    for h in &profile.holes {
        loops.push(
            h.edges
                .iter()
                .map(edge_to_nurbs)
                .collect::<Result<Vec<_>, _>>()?,
        );
    }
    let closed = vec![true; loops.len()];
    Ok(Sec { loops, closed })
}

fn section_from_chains(chains: &[Chain]) -> Result<Sec, ModelError> {
    let mut loops = Vec::new();
    for c in chains {
        c.check(1e-9 * (1.0 + c.plane.origin.coords.amax()))?;
        loops.push(
            c.edges
                .iter()
                .map(edge_to_nurbs)
                .collect::<Result<Vec<_>, _>>()?,
        );
    }
    Ok(Sec {
        loops,
        closed: chains.iter().map(|c| c.closed).collect(),
    })
}

/// The section with every control point mapped by `f` (exact for NURBS: the map is affine).
fn map_section(s: &Sec, f: &dyn Fn(&Point3<f64>) -> Point3<f64>) -> Sec {
    Sec {
        loops: s
            .loops
            .iter()
            .map(|l| {
                l.iter()
                    .map(|c| {
                        NurbsCurve::new(
                            c.knots().clone(),
                            c.points().iter().map(f).collect(),
                            c.weights().to_vec(),
                        )
                        .expect("same counts")
                    })
                    .collect()
            })
            .collect(),
        closed: s.closed.clone(),
    }
}

/// Mean of uniformly spaced curve samples (control-point means are biased for rational curves).
fn centroid(s: &Sec) -> Point3<f64> {
    let mut acc = Vector3::zeros();
    let mut n = 0.0;
    for l in &s.loops {
        for c in l {
            for k in 0..16 {
                acc += c.point_at(k as f64 / 16.0).coords;
                n += 1.0;
            }
        }
    }
    Point3::from(acc / n)
}

fn plane_of(p: &Profile, t: Option<&Transform3>) -> Frame3 {
    match t {
        None => p.plane,
        Some(t) => Frame3 {
            origin: t.apply_point(&p.plane.origin),
            x: t.apply_vector(&p.plane.x),
            y: t.apply_vector(&p.plane.y),
            z: t.apply_vector(&p.plane.z),
        },
    }
}

fn polygon_area_about(s: &[NurbsCurve], n: &Vector3<f64>) -> f64 {
    let mut pts: Vec<Point3<f64>> = Vec::new();
    for c in s {
        let (a, b) = c.domain();
        for k in 0..16 {
            pts.push(c.point_at(a + (b - a) * k as f64 / 16.0));
        }
    }
    let o = pts[0];
    let mut acc = Vector3::zeros();
    for i in 0..pts.len() {
        acc += (pts[i] - o).cross(&(pts[(i + 1) % pts.len()] - o));
    }
    0.5 * acc.dot(n)
}

/// Assemble the solid from the compatible sections.
fn assemble(
    rep: &mut BRep,
    start_plane: Frame3,
    end_plane: Frame3,
    secs: &[Sec],
    v: &[f64],
    q: usize,
    sheet: bool,
    chunks: usize,
) -> Result<BuildResult, ModelError> {
    let nloops = secs[0].loops.len();
    // outward direction of motion for orientation tests
    let c0 = centroid(&secs[0]);
    let c1 = centroid(&secs[secs.len() - 1]);
    let motion = {
        let d = c1 - c0;
        if d.norm() > 1e-12 {
            d.normalize()
        } else {
            return Err(bad("sections coincide"));
        }
    };
    let body = rep.add_body(if sheet {
        BodyKind::Sheet
    } else {
        BodyKind::Solid
    });
    let lump = rep.add_lump(body);
    let shell: ShellId = rep.add_shell(lump);
    let mut history = ShapeHistory::default();
    let mut caps: [Vec<LoopSpec>; 2] = [Vec::new(), Vec::new()];
    for li in 0..nloops {
        let n_edges = secs[0].loops[li].len();
        let closed = secs[0].closed[li];
        let nexts = |e: usize| if closed { (e + 1) % n_edges } else { e + 1 };
        // compatibility within the loop across sections
        let mut per_edge: Vec<Vec<NurbsCurve>> = (0..n_edges)
            .map(|e| secs.iter().map(|s| s.loops[li][e].clone()).collect())
            .collect();
        for col in per_edge.iter_mut() {
            make_compatible(col);
        }
        let sections: Vec<Vec<NurbsCurve>> = (0..secs.len())
            .map(|j| per_edge.iter().map(|col| col[j].clone()).collect())
            .collect();
        // consecutive-edge check at the first section
        for e in 0..(if closed { n_edges } else { n_edges - 1 }) {
            let (a, b) = (
                sections[0][e].point_at(1.0),
                sections[0][(e + 1) % n_edges].point_at(0.0),
            );
            if (a - b).norm() > 1e-7 * (1.0 + a.coords.amax()) {
                return Err(bad("profile loop is not closed"));
            }
        }
        // the sections are skinned in `chunks` pieces that share their end sections: a long twist is
        // several short surfaces instead of one that winds many turns
        let m = sections.len();
        let bounds: Vec<usize> = (0..=chunks).map(|c| c * (m - 1) / chunks).collect();
        let mut skins = Vec::with_capacity(chunks);
        for c in 0..chunks {
            let (s0, s1) = (bounds[c], bounds[c + 1]);
            let (v0, v1) = (v[s0], v[s1]);
            let vc: Vec<f64> = v[s0..=s1].iter().map(|x| (x - v0) / (v1 - v0)).collect();
            skins.push(skin_loop(&sections[s0..=s1], &vc, q.min(s1 - s0), closed)?);
        }
        // vertices at every level between chunks, and the rails and rings joining them
        let mut verts = Vec::with_capacity(chunks + 1);
        for sk in &skins {
            verts.push(
                sk.rails
                    .iter()
                    .map(|r| rep.add_vertex(r.point_at(0.0)))
                    .collect::<Vec<_>>(),
            );
        }
        verts.push(
            skins[chunks - 1]
                .rails
                .iter()
                .map(|r| rep.add_vertex(r.point_at(1.0)))
                .collect::<Vec<_>>(),
        );
        let mut rail_edges = Vec::with_capacity(chunks);
        for (c, sk) in skins.iter().enumerate() {
            let mut level = Vec::new();
            for (r, curve) in sk.rails.iter().enumerate() {
                let cid = rep.add_curve(CurveGeom::Nurbs(curve.clone()));
                level.push(rep.add_edge(verts[c][r], verts[c + 1][r], Some(cid), (0.0, 1.0)));
            }
            rail_edges.push(level);
        }
        let mut rings = Vec::with_capacity(chunks + 1);
        for (level, &si) in bounds.iter().enumerate() {
            let mut ring = Vec::new();
            for e in 0..n_edges {
                let next = nexts(e);
                let cid = rep.add_curve(CurveGeom::Nurbs(sections[si][e].clone()));
                ring.push(rep.add_edge(verts[level][e], verts[level][next], Some(cid), (0.0, 1.0)));
            }
            rings.push(ring);
        }
        let (e0, e1) = (rings[0].clone(), rings[chunks].clone());
        let kind = if li == 0 || sheet {
            LoopKind::Outer
        } else {
            LoopKind::Inner
        };
        caps[0].push(LoopSpec {
            kind,
            chain: (0..n_edges).map(|e| (e0[e], Sense::Forward)).collect(),
        });
        caps[1].push(LoopSpec {
            kind,
            chain: (0..n_edges).map(|e| (e1[e], Sense::Forward)).collect(),
        });
        // side faces
        let ccw = !closed || polygon_area_about(&sections[0], &motion) > 0.0;
        let sign = if ccw == (kind == LoopKind::Outer) {
            1.0
        } else {
            -1.0
        };
        for e in 0..n_edges {
            let next = nexts(e);
            for c in 0..chunks {
                let surf = SurfaceGeom::Nurbs(skins[c].surfaces[e].clone());
                // Su × Sv ∝ (loop tangent) × (motion), which points out of the solid for a loop that is
                // counter-clockwise about the motion direction (outer) or clockwise (inner)
                let forward = sign > 0.0;
                let sid = rep.add_surface(surf);
                let sense = if forward {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                let face = rep.add_face(shell, sid, sense);
                // loop in (u, v): (0,0) → (1,0) → (1,1) → (0,1) is counter-clockwise
                let mut pc = |o: (f64, f64), d: (f64, f64)| Some(rep.add_curve(line_pc(o, d)));
                let mut uses = vec![
                    CoedgeSpec {
                        edge: rings[c][e],
                        sense: Sense::Forward,
                        pcurve: pc((0.0, 0.0), (1.0, 0.0)),
                    },
                    CoedgeSpec {
                        edge: rail_edges[c][next],
                        sense: Sense::Forward,
                        pcurve: pc((1.0, 0.0), (0.0, 1.0)),
                    },
                    CoedgeSpec {
                        edge: rings[c + 1][e],
                        sense: Sense::Reversed,
                        pcurve: pc((0.0, 1.0), (1.0, 0.0)),
                    },
                    CoedgeSpec {
                        edge: rail_edges[c][e],
                        sense: Sense::Reversed,
                        pcurve: pc((0.0, 0.0), (0.0, 1.0)),
                    },
                ];
                if !forward {
                    // outer loops of reversed faces run clockwise in (u, v)
                    uses.reverse();
                    for u in &mut uses {
                        u.sense = u.sense.flip();
                    }
                }
                rep.add_loop(face, LoopKind::Outer, &uses);
                let role = if chunks == 1 {
                    format!("side-{li}-{e}")
                } else {
                    format!("side-{li}-{e}-{c}")
                };
                history.record(EntityRef::Face(face), Origin::New { role: role.into() });
            }
        }
    }
    if sheet {
        if let Err(errs) = rep.validate().into_result() {
            return Err(bad(format!(
                "the surface is invalid: {:?}",
                &errs[..errs.len().min(3)]
            )));
        }
        return Ok(BuildResult { body, history });
    }
    // caps
    for (k, (frame, role)) in [(start_plane, "start-cap"), (end_plane, "end-cap")]
        .into_iter()
        .enumerate()
    {
        let outward = if k == 0 { -motion } else { motion };
        let sid = rep.add_surface(SurfaceGeom::Plane(Plane { frame }));
        let f = add_face_auto(rep, shell, sid, &move |_| outward, &caps[k]).map_err(bad)?;
        history.record(EntityRef::Face(f), Origin::New { role: role.into() });
    }
    if let Err(errs) = rep.validate().into_result() {
        return Err(bad(format!(
            "loft/sweep produced an invalid body: {:?}",
            &errs[..errs.len().min(3)]
        )));
    }
    let vol = rep
        .volume(body)
        .ok_or_else(|| bad("volume not computable"))?;
    if vol.is_nan() || vol <= 0.0 {
        return Err(bad(format!(
            "loft/sweep produced non-positive volume {vol}"
        )));
    }
    Ok(BuildResult { body, history })
}

fn chord_params(secs: &[Sec]) -> Vec<f64> {
    let cs: Vec<Point3<f64>> = secs.iter().map(centroid).collect();
    let mut d = vec![0.0];
    for i in 1..cs.len() {
        d.push(d[i - 1] + (cs[i] - cs[i - 1]).norm());
    }
    let total = *d.last().unwrap();
    d.iter().map(|x| x / total).collect()
}

/// Loft through two or more profiles. Each profile must have the same number of loops and edges per loop,
/// with edges in corresponding order and orientation.
pub fn loft(rep: &mut BRep, profiles: &[Profile]) -> Result<BuildResult, ModelError> {
    if profiles.len() < 2 {
        return Err(ModelError::BadParameter(
            "a loft needs at least two profiles",
        ));
    }
    let secs: Vec<Sec> = profiles
        .iter()
        .map(section_from)
        .collect::<Result<_, _>>()?;
    let planes = (
        plane_of(&profiles[0], None),
        plane_of(&profiles[profiles.len() - 1], None),
    );
    loft_secs(rep, &secs, Some(planes), false)
}

/// Loft through two or more sections into a surface. A section is one or more chains (open or
/// closed); every section needs the same number of chains and segments, in corresponding order. With
/// `ruled` the surface is straight between neighbouring sections, else a smooth interpolation.
pub fn loft_sheet(
    rep: &mut BRep,
    sections: &[Vec<Chain>],
    ruled: bool,
) -> Result<BuildResult, ModelError> {
    if sections.len() < 2 {
        return Err(ModelError::BadParameter(
            "a loft needs at least two profiles",
        ));
    }
    let secs: Vec<Sec> = sections
        .iter()
        .map(|c| section_from_chains(c))
        .collect::<Result<_, _>>()?;
    loft_secs(rep, &secs, None, ruled)
}

fn loft_secs(
    rep: &mut BRep,
    secs: &[Sec],
    planes: Option<(Frame3, Frame3)>,
    ruled: bool,
) -> Result<BuildResult, ModelError> {
    for s in secs {
        if s.loops.len() != secs[0].loops.len()
            || s.closed != secs[0].closed
            || s.loops
                .iter()
                .zip(&secs[0].loops)
                .any(|(a, b)| a.len() != b.len())
        {
            return Err(ModelError::BadProfile(
                "loft profiles must have matching loop and edge counts",
            ));
        }
    }
    let v = chord_params(secs);
    if v.windows(2).any(|w| w[1] - w[0] < 1e-9) {
        return Err(ModelError::BadProfile("loft profiles coincide"));
    }
    let q = if ruled { 1 } else { (secs.len() - 1).min(3) };
    let sheet = planes.is_none();
    let (p0, p1) = planes.unwrap_or((Frame3::world(), Frame3::world()));
    assemble(rep, p0, p1, secs, &v, q, sheet, 1)
}

/// Periodic interpolation of homogeneous data on a closed ring of `m` stations: the uniform periodic cubic
/// B-spline through the stations, as a degree-3 NURBS over `[0, 1]` (knots `(i - 3) / m`, `m + 3` control
/// points). Station `j` sits at `v = j / m`.
fn periodic_skin(col: &[&NurbsCurve]) -> Result<NurbsSurface, ModelError> {
    let m = col.len();
    let nu = col[0].points().len();
    // cyclic system (P[j-1] + 4 P[j] + P[j+1]) / 6 = Q[j]
    let mut a = DMatrix::<f64>::zeros(m, m);
    for j in 0..m {
        a[(j, (j + m - 1) % m)] += 1.0 / 6.0;
        a[(j, j)] += 4.0 / 6.0;
        a[(j, (j + 1) % m)] += 1.0 / 6.0;
    }
    let lu = a.lu();
    let nv = m + 3;
    let mut pts = Vec::with_capacity(nu * nv);
    let mut ws = Vec::with_capacity(nu * nv);
    for i in 0..nu {
        let mut sol: [Vec<f64>; 4] = Default::default();
        for k in 0..4 {
            let data: Vec<f64> = col
                .iter()
                .map(|c| {
                    let (p, w) = (c.points()[i], c.weights()[i]);
                    match k {
                        0 => p.x * w,
                        1 => p.y * w,
                        2 => p.z * w,
                        _ => w,
                    }
                })
                .collect();
            let r = lu
                .solve(&DVector::from_column_slice(&data))
                .ok_or_else(|| bad("singular periodic interpolation system"))?;
            sol[k] = r.iter().copied().collect();
        }
        for j in 0..nv {
            let src = (j + m - 1) % m;
            let w = sol[3][src];
            if w <= 0.0 {
                return Err(bad("interpolated weight is not positive"));
            }
            pts.push(Point3::new(
                sol[0][src] / w,
                sol[1][src] / w,
                sol[2][src] / w,
            ));
            ws.push(w);
        }
    }
    let kv_knots: Vec<f64> = (0..m + 7).map(|i| (i as f64 - 3.0) / m as f64).collect();
    let kv = KnotVector::new(3, kv_knots).map_err(|e| bad(format!("{e:?}")))?;
    NurbsSurface::new(col[0].knots().clone(), kv, pts, ws).map_err(|e| bad(format!("{e:?}")))
}

/// Closed loft: a ring through the `profiles` taken cyclically (the last section joins the first), with
/// one periodic cubic skin per profile edge. The result is a solid of genus one (no end caps); each side
/// face has a seam along the first section. The profiles must be single outer loops with matching edges.
pub fn loft_ring(rep: &mut BRep, profiles: &[Profile]) -> Result<BuildResult, ModelError> {
    let m = profiles.len();
    if m < 4 {
        return Err(ModelError::BadParameter(
            "a closed loft needs at least four profiles",
        ));
    }
    let secs: Vec<Sec> = profiles
        .iter()
        .map(section_from)
        .collect::<Result<_, _>>()?;
    if secs
        .iter()
        .any(|s| s.loops.len() != 1 || s.loops[0].len() != secs[0].loops[0].len())
    {
        return Err(ModelError::BadProfile(
            "closed loft profiles must be single loops with matching edge counts",
        ));
    }
    let n_edges = secs[0].loops[0].len();
    let mut per_edge: Vec<Vec<NurbsCurve>> = (0..n_edges)
        .map(|e| secs.iter().map(|s| s.loops[0][e].clone()).collect())
        .collect();
    for col in per_edge.iter_mut() {
        make_compatible(col);
    }
    let sections: Vec<Vec<NurbsCurve>> = (0..m)
        .map(|j| per_edge.iter().map(|col| col[j].clone()).collect())
        .collect();
    for e in 0..n_edges {
        let (a, b) = (
            sections[0][e].point_at(1.0),
            sections[0][(e + 1) % n_edges].point_at(0.0),
        );
        if (a - b).norm() > 1e-7 * (1.0 + a.coords.amax()) {
            return Err(bad("profile loop is not closed"));
        }
    }
    let motion = {
        let d = centroid(&secs[1]) - centroid(&secs[0]);
        if d.norm() < 1e-12 {
            return Err(bad("sections coincide"));
        }
        d.normalize()
    };
    let ccw = polygon_area_about(&sections[0], &motion) > 0.0;
    let forward = ccw;
    let body = rep.add_body(BodyKind::Solid);
    let lump = rep.add_lump(body);
    let shell: ShellId = rep.add_shell(lump);
    let mut history = ShapeHistory::default();
    // vertices on the first section, closed rail edges, seam edges
    let mut verts = Vec::new();
    let mut surfaces = Vec::new();
    for e in 0..n_edges {
        let col: Vec<&NurbsCurve> = sections.iter().map(|s| &s[e]).collect();
        surfaces.push(periodic_skin(&col)?);
        verts.push(rep.add_vertex(sections[0][e].point_at(0.0)));
    }
    let mut rails = Vec::new();
    for e in 0..n_edges {
        // rail = the u = 0 boundary iso-curve of face e (shared with u = 1 of face e - 1)
        let rail = surfaces[e].isocurve_u(0.0);
        let c = rep.add_curve(CurveGeom::Nurbs(rail));
        rails.push(rep.add_edge(verts[e], verts[e], Some(c), (0.0, 1.0)));
    }
    let mut seams = Vec::new();
    for e in 0..n_edges {
        let c = rep.add_curve(CurveGeom::Nurbs(sections[0][e].clone()));
        seams.push(rep.add_edge(verts[e], verts[(e + 1) % n_edges], Some(c), (0.0, 1.0)));
    }
    for e in 0..n_edges {
        let next = (e + 1) % n_edges;
        let sid = rep.add_surface(SurfaceGeom::Nurbs(surfaces[e].clone()));
        let sense = if forward {
            Sense::Forward
        } else {
            Sense::Reversed
        };
        let face = rep.add_face(shell, sid, sense);
        let mut pc = |o: (f64, f64), d: (f64, f64)| Some(rep.add_curve(line_pc(o, d)));
        let mut uses = vec![
            CoedgeSpec {
                edge: seams[e],
                sense: Sense::Forward,
                pcurve: pc((0.0, 0.0), (1.0, 0.0)),
            },
            CoedgeSpec {
                edge: rails[next],
                sense: Sense::Forward,
                pcurve: pc((1.0, 0.0), (0.0, 1.0)),
            },
            CoedgeSpec {
                edge: seams[e],
                sense: Sense::Reversed,
                pcurve: pc((0.0, 1.0), (1.0, 0.0)),
            },
            CoedgeSpec {
                edge: rails[e],
                sense: Sense::Reversed,
                pcurve: pc((0.0, 0.0), (0.0, 1.0)),
            },
        ];
        if !forward {
            uses.reverse();
            for u in &mut uses {
                u.sense = u.sense.flip();
            }
        }
        rep.add_loop(face, LoopKind::Outer, &uses);
        history.record(
            EntityRef::Face(face),
            Origin::New {
                role: format!("ring-{e}").into(),
            },
        );
    }
    if let Err(errs) = rep.validate().into_result() {
        return Err(bad(format!(
            "closed loft produced an invalid body: {:?}",
            &errs[..errs.len().min(3)]
        )));
    }
    let vol = rep
        .volume(body)
        .ok_or_else(|| bad("volume not computable"))?;
    if vol.is_nan() || vol <= 0.0 {
        return Err(bad(format!(
            "closed loft produced non-positive volume {vol}"
        )));
    }
    Ok(BuildResult { body, history })
}

#[derive(Debug, Clone)]
pub struct SweepOptions {
    /// Maximum allowed deviation of the interpolated surface from the exact sweep (model units, absolute).
    pub tolerance: f64,
    /// Upper bound on the number of interpolated sections.
    pub max_sections: usize,
    /// Total turn (radians, right-handed about the path direction) of the section from the start of the
    /// path to its end, spread evenly along the path.
    pub twist: f64,
    /// A guide rail: where it crosses the plane of each section it sets the section's orientation (the
    /// section's x axis points at it) and its scale (the distance to the path relative to the start).
    pub guide: Option<CurveGeom>,
}

impl Default for SweepOptions {
    fn default() -> Self {
        Self {
            tolerance: 1e-5,
            max_sections: 256,
            twist: 0.0,
            guide: None,
        }
    }
}

/// Where the section stands at one station of the sweep: a frame on the path and a scale about it.
struct Station {
    frame: Frame3,
    scale: f64,
}

/// The point of `guide` lying in the plane through `p` normal to `t`, closest in parameter to `near`.
fn guide_point(
    guide: &CurveGeom,
    p: &Point3<f64>,
    t: &Vector3<f64>,
    near: f64,
) -> Option<(f64, Point3<f64>)> {
    let (g0, g1) = guide.domain();
    const K: usize = 600;
    let f = |s: f64| (guide.point_at(s) - p).dot(t);
    let at = |i: usize| g0 + (g1 - g0) * i as f64 / K as f64;
    let mut best: Option<f64> = None;
    let mut prev = f(at(0));
    for i in 1..=K {
        let cur = f(at(i));
        let root = if prev == 0.0 {
            Some(at(i - 1))
        } else if cur == 0.0 {
            Some(at(i))
        } else if prev * cur < 0.0 {
            let (mut lo, mut hi) = (at(i - 1), at(i));
            let flo = f(lo);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if f(mid) * flo > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            Some(0.5 * (lo + hi))
        } else {
            None
        };
        if let Some(root) = root {
            if best.is_none_or(|b| (root - near).abs() < (b - near).abs()) {
                best = Some(root);
            }
        }
        prev = cur;
    }
    best.map(|s| (s, guide.point_at(s)))
}

/// The stations of a sweep at the path parameters `ts`: rotation-minimising frames (or frames aimed
/// at the guide rail), turned by the twist.
fn stations(
    path: &CurveGeom,
    ts: &[f64],
    seed: Vector3<f64>,
    opts: &SweepOptions,
) -> Result<Vec<Station>, ModelError> {
    let (a, b) = path.domain();
    let mut out: Vec<Station> = match &opts.guide {
        None => frames_along(path, ts, seed)
            .into_iter()
            .map(|frame| Station { frame, scale: 1.0 })
            .collect(),
        Some(guide) => {
            let (g0, g1) = guide.domain();
            if !g0.is_finite() || !g1.is_finite() {
                return Err(bad("the guide rail must have a finite domain"));
            }
            let mut near = g0;
            let mut d0 = 0.0;
            let mut v = Vec::new();
            for (j, t) in ts.iter().enumerate() {
                let p = path.point_at(*t);
                let tan = path.derivative(*t).normalize();
                if j == 0 {
                    near = g0;
                } else {
                    near = near.clamp(g0, g1);
                }
                let (s, g) = guide_point(guide, &p, &tan, near).ok_or_else(|| {
                    bad("the guide rail does not cross the plane of every section")
                })?;
                near = s;
                let off = g - p;
                let perp = off - tan * off.dot(&tan);
                let d = perp.norm();
                if d < 1e-9 {
                    return Err(bad("the guide rail touches the path"));
                }
                if j == 0 {
                    d0 = d;
                }
                let x = perp / d;
                v.push(Station {
                    frame: Frame3 {
                        origin: p,
                        x,
                        y: tan.cross(&x),
                        z: tan,
                    },
                    scale: d / d0,
                });
            }
            v
        }
    };
    if opts.twist != 0.0 {
        for (st, t) in out.iter_mut().zip(ts) {
            let ang = opts.twist * (t - a) / (b - a);
            let (s, c) = ang.sin_cos();
            let f = st.frame;
            st.frame.x = f.x * c + f.y * s;
            st.frame.y = f.y * c - f.x * s;
        }
    }
    Ok(out)
}

/// Where the point `p`, given relative to the start station, lies at station `st`.
fn place(first: &Frame3, st: &Station, p: &Point3<f64>) -> Point3<f64> {
    let q = Transform3::world_to_frame(first).apply_point(p);
    Transform3::frame_to_world(&st.frame).apply_point(&Point3::new(
        q.x * st.scale,
        q.y * st.scale,
        q.z,
    ))
}

/// Rotation-minimising frames along `path` (double reflection method).
fn frames_along(path: &CurveGeom, ts: &[f64], seed: Vector3<f64>) -> Vec<Frame3> {
    let pts: Vec<Point3<f64>> = ts.iter().map(|t| path.point_at(*t)).collect();
    let tans: Vec<Vector3<f64>> = ts.iter().map(|t| path.derivative(*t).normalize()).collect();
    let mut r = seed - tans[0] * seed.dot(&tans[0]);
    if r.norm() < 1e-9 {
        r = Frame3::from_origin_normal(pts[0], tans[0]).x;
    }
    r = r.normalize();
    let mut out = vec![];
    let mk = |o: Point3<f64>, t: Vector3<f64>, r: Vector3<f64>| Frame3 {
        origin: o,
        x: r,
        y: t.cross(&r),
        z: t,
    };
    out.push(mk(pts[0], tans[0], r));
    for j in 0..ts.len() - 1 {
        let v1 = pts[j + 1] - pts[j];
        let c1 = v1.dot(&v1);
        if c1 < 1e-30 {
            out.push(mk(pts[j + 1], tans[j + 1], r));
            continue;
        }
        let rl = r - v1 * (2.0 / c1 * v1.dot(&r));
        let tl = tans[j] - v1 * (2.0 / c1 * v1.dot(&tans[j]));
        let v2 = tans[j + 1] - tl;
        let c2 = v2.dot(&v2);
        r = if c2 < 1e-30 {
            rl
        } else {
            rl - v2 * (2.0 / c2 * v2.dot(&rl))
        };
        r = (r - tans[j + 1] * r.dot(&tans[j + 1])).normalize();
        out.push(mk(pts[j + 1], tans[j + 1], r));
    }
    out
}

/// Sweep `profile` along `path` over its whole domain, transporting it with a rotation-minimising frame.
/// The profile keeps its position relative to the start of the path.
pub fn sweep(
    rep: &mut BRep,
    profile: &Profile,
    path: &CurveGeom,
    opts: &SweepOptions,
) -> Result<BuildResult, ModelError> {
    let base = section_from(profile)?;
    sweep_core(rep, &base, Some(profile.plane), path, opts)
}

/// Sweep chains (open or closed) along `path` into a surface, with the same transport as [`sweep`].
pub fn sweep_sheet(
    rep: &mut BRep,
    chains: &[Chain],
    path: &CurveGeom,
    opts: &SweepOptions,
) -> Result<BuildResult, ModelError> {
    let first = chains
        .first()
        .ok_or(ModelError::BadProfile("nothing to sweep"))?;
    let base = section_from_chains(chains)?;
    sweep_core_seed(rep, &base, None, path, opts, first.plane.x)
}

fn sweep_core(
    rep: &mut BRep,
    base: &Sec,
    plane: Option<Frame3>,
    path: &CurveGeom,
    opts: &SweepOptions,
) -> Result<BuildResult, ModelError> {
    let seed = plane.map_or_else(Vector3::x, |p| p.x);
    sweep_core_seed(rep, base, plane, path, opts, seed)
}

/// `plane` is the plane of the profile of a solid sweep (the caps lie in its copies), `None` for a surface.
fn sweep_core_seed(
    rep: &mut BRep,
    base: &Sec,
    plane: Option<Frame3>,
    path: &CurveGeom,
    opts: &SweepOptions,
    seed: Vector3<f64>,
) -> Result<BuildResult, ModelError> {
    let (a, b) = path.domain();
    if !a.is_finite() || !b.is_finite() {
        return Err(ModelError::BadParameter(
            "sweep path must have a finite domain",
        ));
    }
    // a twist of more than a turn is skinned a turn at a time
    let chunks = ((opts.twist.abs() / std::f64::consts::TAU - 1e-9).ceil() as usize).clamp(1, 64);
    let mut per = 16usize;
    loop {
        let n = chunks * (per - 1) + 1;
        let ts: Vec<f64> = (0..n)
            .map(|i| a + (b - a) * i as f64 / (n - 1) as f64)
            .collect();
        let sts = stations(path, &ts, seed, opts)?;
        let first = sts[0].frame;
        let secs: Vec<Sec> = sts
            .iter()
            .map(|st| map_section(base, &|p| place(&first, st, p)))
            .collect();
        let v = chord_params(&secs);
        if v.windows(2).any(|w| w[1] - w[0] < 1e-12) {
            return Err(ModelError::BadParameter("degenerate sweep path"));
        }
        let q = 3.min(per - 1);
        let planes = plane.map(|pl| {
            let at = |st: &Station| {
                let o = pl.origin;
                let (po, px, pz) = (
                    place(&first, st, &o),
                    place(&first, st, &(o + pl.x)),
                    place(&first, st, &(o + pl.z)),
                );
                let x = (px - po).normalize();
                let z = (pz - po).normalize();
                Frame3 {
                    origin: po,
                    x,
                    y: z.cross(&x),
                    z,
                }
            };
            (at(&sts[0]), at(&sts[n - 1]))
        });
        let (p0, p1) = planes.unwrap_or((Frame3::world(), Frame3::world()));
        let result = assemble(rep, p0, p1, &secs, &v, q, plane.is_none(), chunks);
        // deviation check on a twice-denser set of sections: odd samples are exact sweep positions
        let ok = match &result {
            Ok(r) => deviation(rep, r.body, path, base, opts, seed, (a, b), n, chunks)
                .is_ok_and(|d| d <= opts.tolerance),
            Err(_) => false,
        };
        if ok || per >= opts.max_sections {
            return match result {
                Ok(r) if ok => Ok(r),
                Ok(_) => Err(bad(format!(
                    "sweep did not reach the tolerance {} with {} sections",
                    opts.tolerance, per
                ))),
                Err(e) => Err(e),
            };
        }
        per *= 2;
    }
}

fn deviation(
    rep: &BRep,
    body: kw_brep::BodyId,
    path: &CurveGeom,
    base: &Sec,
    opts: &SweepOptions,
    seed: Vector3<f64>,
    dom: (f64, f64),
    n: usize,
    chunks: usize,
) -> Result<f64, ModelError> {
    // exact positions at the mid-parameters between consecutive samples
    let m = 2 * n - 1;
    let ts: Vec<f64> = (0..m)
        .map(|i| dom.0 + (dom.1 - dom.0) * i as f64 / (m - 1) as f64)
        .collect();
    let sts = stations(path, &ts, seed, opts)?;
    let first = sts[0].frame;
    let surfaces: Vec<NurbsSurface> = rep
        .body_faces(body)
        .into_iter()
        .filter_map(|f| match &rep.surfaces[rep.faces[f].surface] {
            SurfaceGeom::Nurbs(s) => Some(s.clone()),
            _ => None,
        })
        .collect();
    let mut worst = 0.0f64;
    let mut surf_idx = 0;
    // sections per chunk (intervals)
    let ivals = (n - 1) / chunks;
    for l in &base.loops {
        for c in l {
            let first_surface = surf_idx * chunks;
            surf_idx += 1;
            for j in (1..m).step_by(2) {
                // the sample lies between sections k and k + 1, in chunk ch
                let k = j / 2;
                let ch = (k / ivals).min(chunks - 1);
                let Some(s) = surfaces.get(first_surface + ch) else {
                    continue;
                };
                let v_local = ((j as f64) / 2.0 - (ch * ivals) as f64) / ivals as f64;
                for u in [0.0, 0.5, 1.0] {
                    let p = place(&first, &sts[j], &c.point_at(u));
                    // the sample belongs at (u, v_local) on the surface: refine from there (the whole
                    // surface is searched only when that does not converge)
                    worst = worst.max(s.closest_point_near(&p, u, v_local).distance);
                }
            }
        }
    }
    Ok(worst)
}
