//! Surfaces bounded by curves: Coons patches (position only, or tangent to the surfaces next to
//! them), N-sided fills and ruled surfaces.
//!
//! A four-sided hole is filled by the linearly blended Coons patch of its four boundary curves
//! (S. A. Coons, 1967; G. Farin, *Curves and Surfaces for CAGD*, ch. 20). A side that borders a
//! surface can be made tangent to it: the cross-boundary derivative the blend would give is replaced,
//! near that side, by the part of the chord to the opposite side that lies in the neighbour's tangent
//! plane. Any other number of sides is split about the centre into one four-sided patch per corner.
//! The patch is evaluated on a grid and interpolated by a bicubic NURBS surface; the grid is refined
//! until the surface follows the blend to a tolerance.

use crate::profile::{Chain, ModelError, ProfileEdge};
use crate::skin::{line_pc, loft_sheet};
use kw_brep::{
    BRep, BodyKind, BuildResult, CoedgeSpec, EdgeId, EntityRef, LoopKind, Origin, Sense,
    ShapeHistory, VertexId,
};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::{Curve3, CurveGeom, NurbsCurve, NurbsSurface};
use std::sync::Arc;

fn bad(msg: impl Into<String>) -> ModelError {
    ModelError::Construction(msg.into())
}

/// What a patch needs to know about the surface a boundary edge belongs to, at a point of the edge
/// (given with the unit tangent of the edge there): the unit normal, and the unit vector that lies in
/// the tangent plane, is perpendicular to the edge and points out of that surface.
pub type NormalFn =
    Arc<dyn Fn(&Point3<f64>, &Vector3<f64>) -> Option<(Vector3<f64>, Vector3<f64>)> + Send + Sync>;

/// One boundary curve of a patch, in loop order, and the surface behind it (if the patch should be
/// tangent to it).
#[derive(Clone)]
pub struct PatchEdge {
    pub edge: ProfileEdge,
    pub neighbour: Option<NormalFn>,
}

impl PatchEdge {
    pub fn new(edge: ProfileEdge) -> Self {
        Self {
            edge,
            neighbour: None,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// a boundary curve parametrised by arc length

const TABLE: usize = 400;

/// A curve over a parameter range, walked from `t0` to `t1`, and re-parametrised by the fraction of
/// its length so that curves of different parametrisation blend sensibly.
struct Side {
    curve: CurveGeom,
    t0: f64,
    t1: f64,
    /// `(fraction of length, curve parameter)`.
    table: Vec<(f64, f64)>,
    len: f64,
}

impl Side {
    fn new(e: &ProfileEdge) -> Side {
        let (t0, t1) = if e.reversed {
            (e.range.1, e.range.0)
        } else {
            (e.range.0, e.range.1)
        };
        Side::over(e.curve.clone(), t0, t1)
    }

    fn over(curve: CurveGeom, t0: f64, t1: f64) -> Side {
        let mut pts = Vec::with_capacity(TABLE + 1);
        let mut acc = vec![0.0];
        for k in 0..=TABLE {
            let t = t0 + (t1 - t0) * k as f64 / TABLE as f64;
            let p = curve.point_at(t);
            if let Some(prev) = pts.last() {
                let prev: &Point3<f64> = prev;
                acc.push(acc[k - 1] + (p - prev).norm());
            }
            pts.push(p);
        }
        let len = acc[TABLE];
        let table = (0..=TABLE)
            .map(|k| {
                let s = if len > 0.0 {
                    acc[k] / len
                } else {
                    k as f64 / TABLE as f64
                };
                (s, t0 + (t1 - t0) * k as f64 / TABLE as f64)
            })
            .collect();
        Side {
            curve,
            t0,
            t1,
            table,
            len,
        }
    }

    fn param(&self, s: f64) -> f64 {
        let s = s.clamp(0.0, 1.0);
        let i = self
            .table
            .partition_point(|(f, _)| *f < s)
            .clamp(1, self.table.len() - 1);
        let ((fa, ta), (fb, tb)) = (self.table[i - 1], self.table[i]);
        if fb - fa < 1e-300 {
            ta
        } else {
            ta + (tb - ta) * (s - fa) / (fb - fa)
        }
    }

    fn at(&self, s: f64) -> Point3<f64> {
        self.curve.point_at(self.param(s))
    }

    /// The part between the fractions `a` and `b`.
    fn sub(&self, a: f64, b: f64) -> Side {
        Side::over(self.curve.clone(), self.param(a), self.param(b))
    }

    fn reversed(&self) -> Side {
        Side::over(self.curve.clone(), self.t1, self.t0)
    }

    /// Unit tangent in the walking direction.
    fn dir(&self, s: f64) -> Vector3<f64> {
        let h = 1e-4;
        let (a, b) = (self.at((s - h).max(0.0)), self.at((s + h).min(1.0)));
        (b - a).try_normalize(1e-300).unwrap_or_else(Vector3::x)
    }

    fn line(a: Point3<f64>, b: Point3<f64>) -> Side {
        Side::new(&ProfileEdge::line(a, b))
    }
}

// ---------------------------------------------------------------------------------------------
// the blend

/// A smooth window that is 1 inside and falls to 0 (with zero slope and curvature) at both ends.
fn window(s: f64) -> f64 {
    let x = (s.min(1.0 - s) / 0.25).clamp(0.0, 1.0);
    x * x * x * (x * (6.0 * x - 15.0) + 10.0)
}

/// A cubic that is 0 at both ends with slope 1 at 0 and 0 at 1 (and the mirror image).
fn bump0(x: f64) -> f64 {
    x * (1.0 - x) * (1.0 - x)
}
fn bump1(x: f64) -> f64 {
    x * x * (x - 1.0)
}

struct Coons {
    /// Bottom (v = 0) and top (v = 1), both running along u; left (u = 0) and right (u = 1), along v.
    f0: Side,
    f1: Side,
    g0: Side,
    g1: Side,
    corner: [Point3<f64>; 4],
    /// The normal fields of the neighbours of the bottom, right, top and left sides.
    tangent: [Option<NormalFn>; 4],
}

impl Coons {
    /// The sides in loop order (bottom, right, top, left; the top and left run against u and v).
    fn new(loop_sides: [Side; 4], tangent: [Option<NormalFn>; 4]) -> Result<Coons, ModelError> {
        let [bottom, right, top, left] = loop_sides;
        let ends = [
            (bottom.at(1.0), right.at(0.0)),
            (right.at(1.0), top.at(0.0)),
            (top.at(1.0), left.at(0.0)),
            (left.at(1.0), bottom.at(0.0)),
        ];
        let scale = 1.0 + bottom.len + right.len + top.len + left.len;
        if ends.iter().any(|(a, b)| (a - b).norm() > 1e-6 * scale) {
            return Err(ModelError::BadProfile(
                "the boundary curves do not form a closed loop",
            ));
        }
        let f1 = top.reversed();
        let g0 = left.reversed();
        let corner = [bottom.at(0.0), bottom.at(1.0), f1.at(0.0), f1.at(1.0)];
        Ok(Coons {
            f0: bottom,
            f1,
            g0,
            g1: right,
            corner,
            // the top and left neighbours are attached to the sides as they were given
            tangent,
        })
    }

    /// The bilinearly blended Coons patch.
    fn base(&self, u: f64, v: f64) -> Vector3<f64> {
        let [p00, p10, p01, p11] = self.corner;
        (self.f0.at(u).coords * (1.0 - v) + self.f1.at(u).coords * v)
            + (self.g0.at(v).coords * (1.0 - u) + self.g1.at(v).coords * u)
            - (p00.coords * ((1.0 - u) * (1.0 - v))
                + p10.coords * (u * (1.0 - v))
                + p01.coords * ((1.0 - u) * v)
                + p11.coords * (u * v))
    }

    /// One-sided derivative of the plain blend across a side: along v at `v = 0` (`dir = 1`) or at
    /// `v = 1` (`dir = -1`), or along u likewise.
    fn slope(&self, along_v: bool, at: f64, other: f64, dir: f64) -> Vector3<f64> {
        const H: f64 = 1e-4;
        let e = |x: f64| {
            if along_v {
                self.base(other, x)
            } else {
                self.base(x, other)
            }
        };
        (e(at) * -3.0 + e(at + dir * H) * 4.0 - e(at + dir * 2.0 * H)) * (dir / (2.0 * H))
    }

    /// How far the plain blend's cross derivative at `s` of side `k` is from what leaving the
    /// neighbour tangentially needs: the direction out of the neighbour, as long as the chord to the
    /// opposite side.
    fn deficit(&self, k: usize, s: f64) -> Option<Vector3<f64>> {
        let nf = self.tangent[k].as_ref()?;
        let (side, chord, along_v, at_end, dir) = match k {
            0 => (&self.f0, self.f1.at(s) - self.f0.at(s), true, 0.0, 1.0),
            2 => (&self.f1, self.f1.at(s) - self.f0.at(s), true, 1.0, -1.0),
            3 => (&self.g0, self.g1.at(s) - self.g0.at(s), false, 0.0, 1.0),
            _ => (&self.g1, self.g1.at(s) - self.g0.at(s), false, 1.0, -1.0),
        };
        let at = side.at(s);
        let (_, out) = nf(&at, &side.dir(s))?;
        // the patch leaves a bottom or left side along `out`, and arrives at a top or right side
        // against it: the derivative in the direction of increasing v or u changes sign
        let t = out * (0.6 * chord.norm() * dir);
        Some(t - self.slope(along_v, at_end, s, dir))
    }

    /// The patch: the blend, made tangent to the neighbours that were given.
    fn at(&self, u: f64, v: f64) -> Point3<f64> {
        let mut p = self.base(u, v);
        if let Some(c) = self.deficit(0, u) {
            p += c * (window(u) * bump0(v));
        }
        if let Some(c) = self.deficit(2, u) {
            p += c * (window(u) * bump1(v));
        }
        if let Some(c) = self.deficit(3, v) {
            p += c * (window(v) * bump0(u));
        }
        if let Some(c) = self.deficit(1, v) {
            p += c * (window(v) * bump1(u));
        }
        Point3::from(p)
    }
}

/// The surface through a grid of the patch, refined until it follows the patch to `tol`.
fn fit(c: &Coons, tol: f64) -> Result<NurbsSurface, ModelError> {
    let mut n = 17usize;
    loop {
        let h = 1.0 / (n - 1) as f64;
        let grid: Vec<Vec<Point3<f64>>> = (0..n)
            .map(|i| (0..n).map(|j| c.at(i as f64 * h, j as f64 * h)).collect())
            .collect();
        let s =
            NurbsSurface::interpolate_uniform(&grid, 3, 3).map_err(|e| bad(format!("{e:?}")))?;
        // between the grid points, boundary midpoints included
        let mut worst = 0.0f64;
        for i in 0..n - 1 {
            for j in 0..n - 1 {
                for (a, b) in [(0.5, 0.5), (0.5, 0.0), (0.0, 0.5)] {
                    let (u, v) = ((i as f64 + a) * h, (j as f64 + b) * h);
                    worst = worst.max((s.point_at(u, v) - c.at(u, v)).norm());
                }
            }
        }
        if worst <= tol {
            return Ok(s);
        }
        if n >= 129 {
            return Err(bad(format!(
                "the boundary is too intricate to fill smoothly (deviation {worst:.2e})"
            )));
        }
        n = 2 * n - 1;
    }
}

// ---------------------------------------------------------------------------------------------
// the body

struct Pool<'a> {
    rep: &'a mut BRep,
    scale: f64,
    vertices: Vec<(Point3<f64>, VertexId)>,
    /// Start, end, the edge and a point in its middle.
    edges: Vec<(VertexId, VertexId, EdgeId, Point3<f64>)>,
}

impl Pool<'_> {
    fn vertex(&mut self, p: Point3<f64>) -> VertexId {
        if let Some((_, v)) = self
            .vertices
            .iter()
            .find(|(q, _)| (q - p).norm() < 1e-7 * self.scale)
        {
            return *v;
        }
        let v = self.rep.add_vertex(p);
        self.vertices.push((p, v));
        v
    }
}

/// One face per surface, sharing the edges where two patches meet.
fn assemble(
    rep: &mut BRep,
    patches: &[(NurbsSurface, bool)],
    roles: &[String],
    scale: f64,
) -> Result<BuildResult, ModelError> {
    let body = rep.add_body(BodyKind::Sheet);
    let lump = rep.add_lump(body);
    let shell = rep.add_shell(lump);
    let mut history = ShapeHistory::default();
    let mut pool = Pool {
        rep,
        scale,
        vertices: Vec::new(),
        edges: Vec::new(),
    };
    for ((surf, flip), role) in patches.iter().zip(roles) {
        let at = |u: f64, v: f64| surf.point_at(u, v);
        // the four sides as (start, end) in (u, v), in loop order: bottom, right, top, left
        let sides: [((f64, f64), (f64, f64), NurbsCurve); 4] = [
            ((0.0, 0.0), (1.0, 0.0), surf.isocurve_v(0.0)),
            ((1.0, 0.0), (1.0, 1.0), surf.isocurve_u(1.0)),
            ((1.0, 1.0), (0.0, 1.0), surf.isocurve_v(1.0).reversed()),
            ((0.0, 1.0), (0.0, 0.0), surf.isocurve_u(0.0).reversed()),
        ];
        let sid = pool
            .rep
            .add_surface(kw_nurbs::SurfaceGeom::Nurbs(surf.clone()));
        let sense = if *flip {
            Sense::Reversed
        } else {
            Sense::Forward
        };
        let face = pool.rep.add_face(shell, sid, sense);
        let mut uses = Vec::new();
        for (a, b, curve) in sides {
            let (pa, pb) = (at(a.0, a.1), at(b.0, b.1));
            let (va, vb) = (pool.vertex(pa), pool.vertex(pb));
            let mid = curve.point_at(0.5);
            let found = pool.edges.iter().find(|(x, y, _, m)| {
                ((*x == va && *y == vb) || (*x == vb && *y == va))
                    && (m - mid).norm() < 1e-6 * scale
            });
            let (ea, eb, edge) = match found {
                Some((x, y, e, _)) => (*x, *y, *e),
                None => {
                    let cid = pool.rep.add_curve(CurveGeom::Nurbs(curve));
                    let e = pool.rep.add_edge(va, vb, Some(cid), (0.0, 1.0));
                    pool.edges.push((va, vb, e, mid));
                    (va, vb, e)
                }
            };
            // the traversal of the loop against the direction of the edge, and the face's line along
            // the edge in the direction the edge runs
            let forward = ea == va && eb == vb;
            let (o, d) = if forward {
                (a, (b.0 - a.0, b.1 - a.1))
            } else {
                (b, (a.0 - b.0, a.1 - b.1))
            };
            let pc = pool.rep.add_curve(line_pc(o, d));
            uses.push(CoedgeSpec {
                edge,
                sense: if forward {
                    Sense::Forward
                } else {
                    Sense::Reversed
                },
                pcurve: Some(pc),
            });
        }
        if *flip {
            uses.reverse();
            for u in &mut uses {
                u.sense = u.sense.flip();
            }
        }
        pool.rep.add_loop(face, LoopKind::Outer, &uses);
        history.record(
            EntityRef::Face(face),
            Origin::New {
                role: role.clone().into(),
            },
        );
    }
    if let Err(errs) = pool.rep.validate().into_result() {
        return Err(bad(format!(
            "the patch is invalid: {:?}",
            &errs[..errs.len().min(3)]
        )));
    }
    Ok(BuildResult { body, history })
}

/// Fill the closed loop of boundary curves (in loop order, each walked end to start) with a surface.
/// Four edges give one patch (tangent to the neighbours where given); any other number of at least
/// three give one patch per corner, meeting along straight spokes to the centre. The surface is a
/// sheet whose edges follow the boundary curves to within a millionth of its size.
pub fn patch_sheet(rep: &mut BRep, edges: &[PatchEdge]) -> Result<BuildResult, ModelError> {
    let n = edges.len();
    if n < 3 {
        return Err(ModelError::BadProfile(
            "a patch needs a closed boundary of at least three edges",
        ));
    }
    let sides: Vec<Side> = edges.iter().map(|e| Side::new(&e.edge)).collect();
    let size: f64 = sides.iter().map(|s| s.len).sum();
    let scale = 1.0 + size;
    let tol = 1e-6 * scale;
    // consecutive edges must meet
    for i in 0..n {
        let (a, b) = (sides[i].at(1.0), sides[(i + 1) % n].at(0.0));
        if (a - b).norm() > 1e-6 * scale {
            return Err(ModelError::BadProfile(
                "the boundary curves do not form a closed loop",
            ));
        }
    }
    let tangent_any = edges.iter().any(|e| e.neighbour.is_some());
    if n == 4 {
        let c = Coons::new(
            [
                Side::new(&edges[0].edge),
                Side::new(&edges[1].edge),
                Side::new(&edges[2].edge),
                Side::new(&edges[3].edge),
            ],
            [
                edges[0].neighbour.clone(),
                edges[1].neighbour.clone(),
                edges[2].neighbour.clone(),
                edges[3].neighbour.clone(),
            ],
        )?;
        let surf = fit(&c, tol)?;
        // the patch faces the way the surface it is tangent to does
        let mut flip = false;
        'o: for (k, e) in edges.iter().enumerate() {
            if let Some(nf) = &e.neighbour {
                let (u, v) = match k {
                    0 => (0.5, 0.0),
                    1 => (1.0, 0.5),
                    2 => (0.5, 1.0),
                    _ => (0.0, 0.5),
                };
                let p = surf.point_at(u, v);
                let b = surf.derivatives(u, v, 1)[1][0];
                if let (Some((n, _)), Some(m)) = (nf(&p, &b), surf.normal_at(u, v)) {
                    flip = n.dot(&m) < 0.0;
                    break 'o;
                }
            }
        }
        return assemble(rep, &[(surf, flip)], &["patch-0".to_string()], scale);
    }
    if tangent_any {
        return Err(ModelError::BadParameter(
            "tangent continuity needs a boundary of exactly four edges",
        ));
    }
    // N-sided: the centre, the midpoint of every edge, and a patch around every corner
    let mid_pts: Vec<Point3<f64>> = sides.iter().map(|s| s.at(0.5)).collect();
    let mut centre = Vector3::zeros();
    for i in 0..n {
        centre += sides[i].at(0.0).coords + mid_pts[i].coords;
    }
    let centre = Point3::from(centre / (2 * n) as f64);
    let mut patches = Vec::new();
    let mut roles = Vec::new();
    for i in 0..n {
        let prev = (i + n - 1) % n;
        // around the corner at the start of edge i: M(prev) -> corner -> M(i) -> centre -> M(prev)
        let loop_sides = [
            sides[prev].sub(0.5, 1.0),
            sides[i].sub(0.0, 0.5),
            Side::line(mid_pts[i], centre),
            Side::line(centre, mid_pts[prev]),
        ];
        let c = Coons::new(loop_sides, [None, None, None, None])?;
        patches.push((fit(&c, tol)?, false));
        roles.push(format!("patch-{i}"));
    }
    assemble(rep, &patches, &roles, scale)
}

// ---------------------------------------------------------------------------------------------
// ruled surface

/// A curve that the loft builders accept: a line, a circular arc or a NURBS curve over its whole
/// domain. Anything else is replaced by a cubic through samples of it.
pub fn loftable(e: &ProfileEdge) -> Result<ProfileEdge, ModelError> {
    let ok = match &e.curve {
        CurveGeom::Line(_) | CurveGeom::Circle(_) => true,
        CurveGeom::Nurbs(n) => {
            let (a, b) = n.domain();
            (e.range.0 - a).abs() < 1e-12 && (e.range.1 - b).abs() < 1e-12
        }
        _ => false,
    };
    if ok {
        return Ok(e.clone());
    }
    let (t0, t1) = if e.reversed {
        (e.range.1, e.range.0)
    } else {
        (e.range.0, e.range.1)
    };
    let pts: Vec<Point3<f64>> = (0..=40)
        .map(|k| e.curve.point_at(t0 + (t1 - t0) * k as f64 / 40.0))
        .collect();
    let c = NurbsCurve::interpolate(&pts, 3).map_err(|er| bad(format!("{er:?}")))?;
    let (a, b) = c.domain();
    Ok(ProfileEdge::new(CurveGeom::Nurbs(c), (a, b)))
}

/// The ruled surface between two edges: straight lines join corresponding points. The second edge is
/// turned round if that makes the lines shorter.
pub fn ruled_sheet(
    rep: &mut BRep,
    a: &ProfileEdge,
    b: &ProfileEdge,
) -> Result<BuildResult, ModelError> {
    let (a, mut b) = (loftable(a)?, loftable(b)?);
    let straight = (a.start() - b.start()).norm() + (a.end() - b.end()).norm();
    let crossed = (a.start() - b.end()).norm() + (a.end() - b.start()).norm();
    if crossed < straight {
        b.reversed = !b.reversed;
    }
    let chain = |e: ProfileEdge| vec![Chain::new(Frame3::world(), vec![e], false)];
    loft_sheet(rep, &[chain(a), chain(b)], true)
}
