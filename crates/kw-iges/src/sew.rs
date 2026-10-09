//! IGES entities → B-rep: geometry conversion, trimmed-surface faces, sewing into shells, solids.

use crate::parser::{parse, Entity, IgesFile, IgesParseError};
use kw_brep::{add_face_auto, BRep, BodyId, BodyKind, EdgeId, LoopKind, LoopSpec, Sense, VertexId};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::geom::{Circle, Line};
use kw_nurbs::proj::{curve_param_of, surface_param_of};
use kw_nurbs::{Curve3, CurveGeom, KnotVector, NurbsCurve, NurbsSurface, Surface, SurfaceGeom};
use std::collections::{BTreeMap, HashMap};
use std::f64::consts::TAU;

#[derive(Debug, Clone, PartialEq)]
pub enum IgesError {
    Parse(IgesParseError),
    /// No trimmed surfaces or solids in the file.
    NothingToImport,
    /// Faces were found but none of the shells closed into a valid solid.
    NoSolids(Vec<String>),
}

impl std::fmt::Display for IgesError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for IgesError {}
impl From<IgesParseError> for IgesError {
    fn from(e: IgesParseError) -> Self {
        IgesError::Parse(e)
    }
}

#[derive(Debug, Clone, Default)]
pub struct ImportReport {
    pub bodies: Vec<BodyId>,
    pub faces_read: usize,
    pub faces_skipped: Vec<(usize, String)>,
    /// Shells that did not close or did not validate.
    pub rejected_shells: Vec<String>,
    pub unit_mm: f64,
}

type R<T> = Result<T, String>;

/// One edge-segment of a face boundary in model coordinates.
#[derive(Clone)]
struct Seg {
    curve: CurveGeom,
    range: (f64, f64),
}

impl Seg {
    fn start(&self) -> Point3<f64> {
        self.curve.point_at(self.range.0)
    }
    fn end(&self) -> Point3<f64> {
        self.curve.point_at(self.range.1)
    }
}

struct SoupFace {
    de: usize,
    surface: SurfaceGeom,
    /// Outer loop first.
    loops: Vec<Vec<Seg>>,
}

struct Ctx<'a> {
    f: &'a IgesFile,
    scale: f64,
}

impl Ctx<'_> {
    fn ent(&self, de: usize) -> R<&Entity> {
        self.f
            .get(de)
            .ok_or_else(|| format!("dangling pointer {de}"))
    }

    /// Transformation matrix (rotation rows + translation) of an entity, identity when absent.
    fn xf(&self, e: &Entity) -> (nalgebra::Matrix3<f64>, Vector3<f64>) {
        if e.transform == 0 {
            return (nalgebra::Matrix3::identity(), Vector3::zeros());
        }
        match self.f.get(e.transform) {
            Some(t) if t.ty == 124 && t.params.len() >= 12 => {
                let p = |i: usize| t.params[i].f();
                (
                    nalgebra::Matrix3::new(p(0), p(1), p(2), p(4), p(5), p(6), p(8), p(9), p(10)),
                    Vector3::new(p(3), p(7), p(11)) * self.scale,
                )
            }
            _ => (nalgebra::Matrix3::identity(), Vector3::zeros()),
        }
    }

    fn pt(&self, e: &Entity, i: usize) -> Point3<f64> {
        let (r, t) = self.xf(e);
        let raw =
            Vector3::new(e.params[i].f(), e.params[i + 1].f(), e.params[i + 2].f()) * self.scale;
        Point3::from(r * raw + t)
    }

    fn nurbs_curve(&self, e: &Entity) -> R<NurbsCurve> {
        let p = &e.params;
        let (k, m) = (
            p.first().ok_or("126: empty")?.i() as usize,
            p.get(1).ok_or("126: empty")?.i() as usize,
        );
        let n = k + 1;
        let nk = k + m + 2;
        let need = 6 + nk + n + 3 * n + 2;
        if p.len() < need || m == 0 || n <= m {
            return Err(format!(
                "126: malformed (have {} parameters, need {need})",
                p.len()
            ));
        }
        let knots: Vec<f64> = (0..nk).map(|i| p[6 + i].f()).collect();
        let w: Vec<f64> = (0..n).map(|i| p[6 + nk + i].f()).collect();
        let base = 6 + nk + n;
        let pts: Vec<Point3<f64>> = (0..n).map(|i| self.pt(e, base + 3 * i)).collect();
        let kv = KnotVector::new(m, knots).map_err(|er| format!("126: {er:?}"))?;
        NurbsCurve::new(kv, pts, w).map_err(|er| format!("126: {er:?}"))
    }

    fn nurbs_surface(&self, e: &Entity) -> R<NurbsSurface> {
        let p = &e.params;
        if p.len() < 9 {
            return Err("128: too short".into());
        }
        let (k1, k2, m1, m2) = (
            p[0].i() as usize,
            p[1].i() as usize,
            p[2].i() as usize,
            p[3].i() as usize,
        );
        let (n1, n2) = (k1 + 1, k2 + 1);
        let (nk1, nk2) = (k1 + m1 + 2, k2 + m2 + 2);
        let need = 9 + nk1 + nk2 + n1 * n2 * 4 + 4;
        if p.len() < need || m1 == 0 || m2 == 0 || n1 <= m1 || n2 <= m2 {
            return Err(format!(
                "128: malformed (have {} parameters, need {need})",
                p.len()
            ));
        }
        let s: Vec<f64> = (0..nk1).map(|i| p[9 + i].f()).collect();
        let t: Vec<f64> = (0..nk2).map(|i| p[9 + nk1 + i].f()).collect();
        let wbase = 9 + nk1 + nk2;
        let pbase = wbase + n1 * n2;
        let mut pts = vec![Point3::origin(); n1 * n2];
        let mut ws = vec![1.0; n1 * n2];
        // IGES order: i (first parametric direction) varies fastest
        for j in 0..n2 {
            for i in 0..n1 {
                let idx = j * n1 + i;
                ws[i * n2 + j] = p[wbase + idx].f();
                pts[i * n2 + j] = self.pt(e, pbase + 3 * idx);
            }
        }
        let (ku, kvv) = (
            KnotVector::new(m1, s).map_err(|er| format!("128: {er:?}"))?,
            KnotVector::new(m2, t).map_err(|er| format!("128: {er:?}"))?,
        );
        NurbsSurface::new(ku, kvv, pts, ws).map_err(|er| format!("128: {er:?}"))
    }

    /// Curve entity → ordered segments (composite curves expand).
    fn segments(&self, de: usize, depth: usize) -> R<Vec<Seg>> {
        if depth > 8 {
            return Err("curve nesting too deep".into());
        }
        let e = self.ent(de)?;
        match e.ty {
            110 => {
                let (a, b) = (self.pt(e, 0), self.pt(e, 3));
                Ok(vec![Seg {
                    curve: CurveGeom::Line(Line {
                        origin: a,
                        dir: b - a,
                    }),
                    range: (0.0, 1.0),
                }])
            }
            100 => {
                let p = &e.params;
                if p.len() < 7 {
                    return Err("100: too short".into());
                }
                let zt = p[0].f();
                let (cx, cy) = (p[1].f(), p[2].f());
                let (sx, sy, ex, ey) = (p[3].f(), p[4].f(), p[5].f(), p[6].f());
                let (r, t) = self.xf(e);
                let to_world = |x: f64, y: f64, z: f64| {
                    Point3::from(r * (Vector3::new(x, y, z) * self.scale) + t)
                };
                let centre = to_world(cx, cy, zt);
                let rad = ((sx - cx).hypot(sy - cy)) * self.scale;
                let a0 = (sy - cy).atan2(sx - cx);
                let mut a1 = (ey - cy).atan2(ex - cx);
                while a1 <= a0 + 1e-12 {
                    a1 += TAU;
                }
                let xa = (r * Vector3::x()).normalize();
                let za = (r * Vector3::z()).normalize();
                let frame = Frame3 {
                    origin: centre,
                    x: xa,
                    y: za.cross(&xa),
                    z: za,
                };
                Ok(vec![Seg {
                    curve: CurveGeom::Circle(Circle { frame, radius: rad }),
                    range: (a0, a1),
                }])
            }
            126 => {
                let c = self.nurbs_curve(e)?;
                let d = c.domain();
                Ok(vec![Seg {
                    curve: CurveGeom::Nurbs(c),
                    range: d,
                }])
            }
            102 => {
                let n = e.params.first().ok_or("102: empty")?.i() as usize;
                let mut out = Vec::new();
                for k in 0..n {
                    let pde = e.params.get(1 + k).ok_or("102: short")?.i() as usize;
                    out.extend(self.segments(pde, depth + 1)?);
                }
                Ok(out)
            }
            142 => {
                // CRTN, SPTR, BPTR, CPTR, PREF – use the 3-D curve C
                let c = e.params.get(3).ok_or("142: short")?.i() as usize;
                if c == 0 {
                    return Err("142 without a 3-D curve".into());
                }
                self.segments(c, depth + 1)
            }
            other => Err(format!("unsupported curve entity type {other}")),
        }
    }

    /// Chain segments end to start, flipping them as needed; returns closed ordered loop.
    fn chain(&self, mut segs: Vec<Seg>, tol: f64) -> R<Vec<Seg>> {
        if segs.is_empty() {
            return Err("empty boundary".into());
        }
        let flip = |s: &Seg| -> Seg {
            match &s.curve {
                CurveGeom::Nurbs(n) => {
                    let r = n.reversed();
                    let d = r.domain();
                    Seg {
                        curve: CurveGeom::Nurbs(r),
                        range: d,
                    }
                }
                CurveGeom::Line(l) => Seg {
                    curve: CurveGeom::Line(Line {
                        origin: l.origin + l.dir,
                        dir: -l.dir,
                    }),
                    range: s.range,
                },
                CurveGeom::Circle(c) => {
                    // reversed circle: mirror the frame (y,z negated), angles negate
                    let f = Frame3 {
                        origin: c.frame.origin,
                        x: c.frame.x,
                        y: -c.frame.y,
                        z: -c.frame.z,
                    };
                    Seg {
                        curve: CurveGeom::Circle(Circle {
                            frame: f,
                            radius: c.radius,
                        }),
                        range: (TAU - s.range.1, TAU - s.range.0),
                    }
                }
                _ => s.clone(),
            }
        };
        let mut out = vec![segs.remove(0)];
        while !segs.is_empty() {
            let end = out.last().unwrap().end();
            let pos = segs
                .iter()
                .position(|s| (s.start() - end).norm() <= tol)
                .map(|i| (i, false))
                .or_else(|| {
                    segs.iter()
                        .position(|s| (s.end() - end).norm() <= tol)
                        .map(|i| (i, true))
                });
            let Some((i, rev)) = pos else {
                // the first segment may itself be oriented the wrong way: try flipping it once
                if out.len() == 1 {
                    let f = flip(&out[0]);
                    let end2 = f.end();
                    if segs
                        .iter()
                        .any(|s| (s.start() - end2).norm() <= tol || (s.end() - end2).norm() <= tol)
                    {
                        out[0] = f;
                        continue;
                    }
                }
                return Err("boundary segments do not chain".into());
            };
            let s = segs.remove(i);
            out.push(if rev { flip(&s) } else { s });
        }
        let (a, b) = (out[0].start(), out.last().unwrap().end());
        if (a - b).norm() > tol {
            return Err("boundary does not close".into());
        }
        Ok(out)
    }

    fn surface(&self, de: usize) -> R<SurfaceGeom> {
        let e = self.ent(de)?;
        match e.ty {
            128 => Ok(SurfaceGeom::Nurbs(self.nurbs_surface(e)?)),
            other => Err(format!("unsupported surface entity type {other}")),
        }
    }

    fn trimmed_face(&self, e: &Entity, tol: f64) -> R<SoupFace> {
        let p = &e.params;
        let surf = self.surface(p.first().ok_or("144: short")?.i() as usize)?;
        let n1 = p.get(1).map(|x| x.i()).unwrap_or(0);
        let n2 = p.get(2).map(|x| x.i()).unwrap_or(0) as usize;
        let mut loops = Vec::new();
        if n1 == 0 {
            // outer boundary is the surface boundary
            let ((u0, u1), (v0, v1)) = surf.domain();
            let iso = |s: &SurfaceGeom, fixed_u: Option<f64>, fixed_v: Option<f64>| -> R<Seg> {
                let SurfaceGeom::Nurbs(n) = s else {
                    return Err("natural boundary needs a NURBS surface".into());
                };
                let c = if let Some(u) = fixed_u {
                    n.isocurve_u(u)
                } else {
                    n.isocurve_v(fixed_v.unwrap())
                };
                let d = c.domain();
                Ok(Seg {
                    curve: CurveGeom::Nurbs(c),
                    range: d,
                })
            };
            let segs = vec![
                iso(&surf, None, Some(v0))?,
                iso(&surf, Some(u1), None)?,
                iso(&surf, None, Some(v1))?,
                iso(&surf, Some(u0), None)?,
            ];
            loops.push(self.chain(segs, tol)?);
        } else {
            let pto = p.get(3).ok_or("144: no outer boundary")?.i() as usize;
            loops.push(self.chain(self.segments(pto, 0)?, tol)?);
        }
        for k in 0..n2 {
            let pti = p.get(4 + k).ok_or("144: missing inner boundary")?.i() as usize;
            loops.push(self.chain(self.segments(pti, 0)?, tol)?);
        }
        Ok(SoupFace {
            de: e.de,
            surface: surf,
            loops,
        })
    }

    /// 186 manifold solid B-rep: collect its faces (shell and void shells).
    fn solid_faces(&self, e: &Entity, tol: f64) -> R<Vec<(Vec<SoupFace>, bool)>> {
        let mut shells = Vec::new();
        let shell_de = e.params.first().ok_or("186: short")?.i() as usize;
        shells.push((shell_de, false));
        let nv = e.params.get(2).map(|x| x.i()).unwrap_or(0) as usize;
        for k in 0..nv {
            let d = e.params.get(3 + 2 * k).ok_or("186: void list short")?.i() as usize;
            shells.push((d, true));
        }
        let mut out = Vec::new();
        for (sd, is_void) in shells {
            let sh = self.ent(sd)?;
            if sh.ty != 514 {
                return Err(format!("186 refers to entity {} instead of a shell", sh.ty));
            }
            let n = sh.params.first().ok_or("514: short")?.i() as usize;
            let mut faces = Vec::new();
            for k in 0..n {
                let fd = sh.params.get(1 + 2 * k).ok_or("514: face list short")?.i() as usize;
                faces.push(self.topo_face(self.ent(fd)?, tol)?);
            }
            out.push((faces, is_void));
        }
        Ok(out)
    }

    fn topo_face(&self, fe: &Entity, tol: f64) -> R<SoupFace> {
        if fe.ty != 510 {
            return Err(format!("expected a face (510), found {}", fe.ty));
        }
        let surf = self.surface(fe.params.first().ok_or("510: short")?.i() as usize)?;
        let n = fe.params.get(1).ok_or("510: short")?.i() as usize;
        let mut loops = Vec::new();
        for k in 0..n {
            let ld = fe.params.get(3 + k).ok_or("510: loop list short")?.i() as usize;
            let le = self.ent(ld)?;
            if le.ty != 508 {
                return Err("expected a loop (508)".into());
            }
            let count = le.params.first().ok_or("508: short")?.i() as usize;
            let mut segs = Vec::new();
            let mut idx = 1;
            for _ in 0..count {
                let ty = le.params.get(idx).ok_or("508: short")?.i();
                let list = le.params.get(idx + 1).ok_or("508: short")?.i() as usize;
                let which = le.params.get(idx + 2).ok_or("508: short")?.i() as usize;
                let nparam = le.params.get(idx + 4).ok_or("508: short")?.i() as usize;
                idx += 5 + 2 * nparam;
                if ty != 0 {
                    continue; // vertex use: no geometry
                }
                let el = self.ent(list)?;
                if el.ty != 504 {
                    return Err("expected an edge list (504)".into());
                }
                let curve_de = el.params.get(1 + 5 * (which - 1)).ok_or("504: short")?.i() as usize;
                segs.extend(self.segments(curve_de, 0)?);
            }
            loops.push(self.chain(segs, tol)?);
        }
        Ok(SoupFace {
            de: fe.de,
            surface: surf,
            loops,
        })
    }
}

// ---------------------------------------------------------------------------------------------
// sewing
// ---------------------------------------------------------------------------------------------

struct Sewn {
    vertices: Vec<Point3<f64>>,
}

impl Sewn {
    fn vertex(&mut self, p: Point3<f64>, tol: f64) -> usize {
        if let Some(i) = self.vertices.iter().position(|q| (q - p).norm() <= tol) {
            return i;
        }
        self.vertices.push(p);
        self.vertices.len() - 1
    }
}

/// An edge occurrence on a face loop, for matching.
struct Occ {
    face: usize,
    lp: usize,
    seg: usize,
    v0: usize,
    v1: usize,
}

fn same_curve(a: &Seg, b: &Seg, tol: f64) -> bool {
    [0.25, 0.5, 0.75].iter().all(|f| {
        let p = a.curve.point_at(a.range.0 + (a.range.1 - a.range.0) * f);
        curve_param_of(&b.curve, &p, None).distance <= 4.0 * tol
    })
}

fn assemble(rep: &mut BRep, faces: Vec<SoupFace>, tol: f64, report: &mut ImportReport) {
    if faces.is_empty() {
        return;
    }
    let mut sew = Sewn {
        vertices: Vec::new(),
    };
    // vertex indices per face segment
    let mut vs: Vec<Vec<Vec<(usize, usize)>>> = Vec::new();
    for f in &faces {
        vs.push(
            f.loops
                .iter()
                .map(|l| {
                    l.iter()
                        .map(|s| (sew.vertex(s.start(), tol), sew.vertex(s.end(), tol)))
                        .collect()
                })
                .collect(),
        );
    }
    // edge matching
    let mut occs: Vec<Occ> = Vec::new();
    for (fi, f) in faces.iter().enumerate() {
        for (li, l) in f.loops.iter().enumerate() {
            for (si, seg) in l.iter().enumerate() {
                let _ = seg;
                occs.push(Occ {
                    face: fi,
                    lp: li,
                    seg: si,
                    v0: vs[fi][li][si].0,
                    v1: vs[fi][li][si].1,
                });
            }
        }
    }
    // union occurrences into edges: key = unordered vertex pair; verify geometry
    let mut groups: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for (i, o) in occs.iter().enumerate() {
        groups
            .entry((o.v0.min(o.v1), o.v0.max(o.v1)))
            .or_default()
            .push(i);
    }
    let seg_of = |o: &Occ| &faces[o.face].loops[o.lp][o.seg];
    let mut edge_id: Vec<usize> = vec![usize::MAX; occs.len()];
    let mut nedges = 0usize;
    for list in groups.values() {
        let mut reps: Vec<(usize, usize)> = Vec::new(); // (representative occ, edge id)
        for &i in list {
            let mut hit = None;
            for &(r, id) in &reps {
                if same_curve(seg_of(&occs[r]), seg_of(&occs[i]), tol) {
                    hit = Some(id);
                    break;
                }
            }
            match hit {
                Some(id) => edge_id[i] = id,
                None => {
                    edge_id[i] = nedges;
                    reps.push((i, nedges));
                    nedges += 1;
                }
            }
        }
    }
    // faces per edge
    let mut users: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, e) in edge_id.iter().enumerate() {
        users.entry(*e).or_default().push(i);
    }
    // connected components via shared edges
    let mut comp: Vec<usize> = (0..faces.len()).collect();
    fn root(c: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while c[r] != r {
            r = c[r];
        }
        let mut y = x;
        while c[y] != r {
            let n = c[y];
            c[y] = r;
            y = n;
        }
        r
    }
    for list in users.values() {
        for w in list.windows(2) {
            let (a, b) = (
                root(&mut comp, occs[w[0]].face),
                root(&mut comp, occs[w[1]].face),
            );
            comp[a] = b;
        }
    }
    let mut shells: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for fi in 0..faces.len() {
        let r = root(&mut comp, fi);
        shells.entry(r).or_default().push(fi);
    }
    'shell: for (_, members) in shells {
        // closedness: every edge of the shell used exactly twice
        for &fi in &members {
            for (i, o) in occs.iter().enumerate() {
                if o.face == fi && users[&edge_id[i]].len() != 2 {
                    report.rejected_shells.push(format!(
                        "shell with face DE{} is not closed (an edge is used {} times)",
                        faces[fi].de,
                        users[&edge_id[i]].len()
                    ));
                    continue 'shell;
                }
            }
        }
        // `add_face_auto` re-orients every loop (outer counter-clockwise, inner clockwise about the chosen
        // outward normal), so the final direction of an edge use depends on the face's normal choice. Predict it.
        let base_rev = |fi: usize, li: usize| -> bool {
            let f = &faces[fi];
            let segs = &f.loops[li];
            let mut pts: Vec<Point3<f64>> = Vec::new();
            for sg in segs {
                for k in 0..12 {
                    pts.push(
                        sg.curve
                            .point_at(sg.range.0 + (sg.range.1 - sg.range.0) * k as f64 / 12.0),
                    );
                }
            }
            let o = pts[0];
            let mut area = Vector3::zeros();
            let mut cen = Vector3::zeros();
            for i in 0..pts.len() {
                area += (pts[i] - o).cross(&(pts[(i + 1) % pts.len()] - o));
                cen += pts[i].coords;
            }
            let cen = Point3::from(cen / pts.len() as f64);
            let r = surface_param_of(&f.surface, &cen, None);
            let n = f.surface.normal_at(r.u, r.v);
            let ccw_about_n = area.dot(&n) > 0.0;
            let desired_ccw = li == 0; // outer loops counter-clockwise about the outward normal (= +n at flipped = false)
            ccw_about_n != desired_ccw
        };
        // orientation: flip faces so that shared edges are traversed in opposite directions
        let mut flipped: HashMap<usize, bool> = HashMap::new();
        let mut queue = vec![members[0]];
        flipped.insert(members[0], false);
        let mut ok = true;
        while let Some(fi) = queue.pop() {
            for (i, o) in occs.iter().enumerate() {
                if o.face != fi {
                    continue;
                }
                let my_dir_fwd = (o.v0 <= o.v1) ^ base_rev(fi, o.lp) ^ flipped[&fi];
                for &j in &users[&edge_id[i]] {
                    let oj = &occs[j];
                    if oj.face == fi {
                        continue;
                    }
                    // direction of the other occurrence relative to the canonical (lo→hi vertex) direction
                    let other_fwd = (oj.v0 <= oj.v1) ^ base_rev(oj.face, oj.lp);
                    // consistent iff traversal directions are opposite
                    let want_flip = my_dir_fwd == other_fwd; // same direction → the other face must flip
                    match flipped.get(&oj.face) {
                        None => {
                            flipped.insert(oj.face, want_flip ^ false);
                            queue.push(oj.face);
                        }
                        Some(&fl) => {
                            let other_dir = other_fwd ^ fl;
                            if other_dir == my_dir_fwd {
                                ok = false;
                            }
                        }
                    }
                }
            }
        }
        if !ok {
            report.rejected_shells.push(format!(
                "shell with face DE{} cannot be consistently oriented",
                faces[members[0]].de
            ));
            continue;
        }
        // ---- build into a scratch B-rep so a bad shell leaves `rep` untouched
        let mut scratch = BRep::new();
        let body = scratch.add_body(BodyKind::Solid);
        let lump = scratch.add_lump(body);
        let shell = scratch.add_shell(lump);
        let mut vmap: HashMap<usize, VertexId> = HashMap::new();
        let mut emap: HashMap<usize, (EdgeId, usize)> = HashMap::new(); // edge id → (edge, occ defining orientation)
        let mut result: Result<(), String> = Ok(());
        let mut vol_sign_faces = Vec::new();
        for &fi in &members {
            let f = &faces[fi];
            let fl = flipped[&fi];
            let sid = scratch.add_surface(f.surface.clone());
            let mut specs = Vec::new();
            for (li, l) in f.loops.iter().enumerate() {
                let mut chain: Vec<(EdgeId, Sense)> = Vec::new();
                for (si, seg) in l.iter().enumerate() {
                    let oi = occs
                        .iter()
                        .position(|o| o.face == fi && o.lp == li && o.seg == si)
                        .unwrap();
                    let eid = edge_id[oi];
                    let (e, def) = match emap.get(&eid) {
                        Some(x) => *x,
                        None => {
                            let (a, b) = (occs[oi].v0, occs[oi].v1);
                            let va = *vmap
                                .entry(a)
                                .or_insert_with(|| scratch.add_vertex(sew.vertices[a]));
                            let vb = *vmap
                                .entry(b)
                                .or_insert_with(|| scratch.add_vertex(sew.vertices[b]));
                            let cid = scratch.add_curve(seg.curve.clone());
                            let e = scratch.add_edge(va, vb, Some(cid), seg.range);
                            emap.insert(eid, (e, oi));
                            (e, oi)
                        }
                    };
                    // sense: this occurrence runs along the edge iff its start matches the edge start
                    let same_dir = if def == oi {
                        true
                    } else {
                        let ed = &scratch.edges[e];
                        let p0 = scratch.vertices[ed.start].point;
                        (seg.start() - p0).norm() <= tol
                            && (occs[oi].v0 != occs[oi].v1 || curve_dir_matches(seg, &scratch, e))
                    };
                    chain.push((
                        e,
                        if same_dir {
                            Sense::Forward
                        } else {
                            Sense::Reversed
                        },
                    ));
                }
                specs.push(LoopSpec {
                    kind: if li == 0 {
                        LoopKind::Outer
                    } else {
                        LoopKind::Inner
                    },
                    chain,
                });
            }
            // the loop direction (after the flip) defines the face normal: surface normal agrees iff not flipped
            let s2 = f.surface.clone();
            let flip_sign = if fl { -1.0 } else { 1.0 };
            let surf_for_dir = s2.clone();
            let outward = move |p: &Point3<f64>| {
                let r = surface_param_of(&surf_for_dir, p, None);
                surf_for_dir.normal_at(r.u, r.v) * flip_sign
            };
            // IGES loops run counter-clockwise about the surface normal for outer loops; the face is
            // placed with the normal following the loop direction
            match add_face_auto(&mut scratch, shell, sid, &outward, &specs) {
                Ok(face) => vol_sign_faces.push(face),
                Err(e) => {
                    result = Err(format!("face DE{}: {e}", f.de));
                    break;
                }
            }
        }
        if let Err(e) = result {
            report.rejected_shells.push(e);
            continue;
        }
        // overall orientation: make the volume positive
        let mut vol = scratch.volume(body).unwrap_or(f64::NAN);
        if vol.is_nan() {
            report.rejected_shells.push("volume not computable".into());
            continue;
        }
        let mut final_rep = scratch;
        if vol < 0.0 {
            // rebuild inside-out
            let mut flipped_rep = BRep::new();
            let (nb, _) = flipped_rep.copy_body_from(
                &final_rep,
                body,
                &kw_brep::CopyOptions {
                    transform: None,
                    flip: true,
                },
            );
            vol = flipped_rep.volume(nb).unwrap_or(f64::NAN);
            final_rep = flipped_rep;
            if vol.is_nan() || vol <= 0.0 {
                report.rejected_shells.push("shell has zero volume".into());
                continue;
            }
            let nb_body = nb;
            if let Err(e) = final_rep.validate().into_result() {
                report
                    .rejected_shells
                    .push(format!("shell is invalid: {:?}", &e[..e.len().min(3)]));
                continue;
            }
            let (out, _) =
                rep.copy_body_from(&final_rep, nb_body, &kw_brep::CopyOptions::default());
            report.bodies.push(out);
            continue;
        }
        if let Err(e) = final_rep.validate().into_result() {
            report
                .rejected_shells
                .push(format!("shell is invalid: {:?}", &e[..e.len().min(3)]));
            continue;
        }
        let (out, _) = rep.copy_body_from(&final_rep, body, &kw_brep::CopyOptions::default());
        report.bodies.push(out);
    }
}

fn curve_dir_matches(seg: &Seg, scratch: &BRep, e: EdgeId) -> bool {
    let ed = &scratch.edges[e];
    let Some(c) = ed.curve.and_then(|c| scratch.curves.get(c)) else {
        return true;
    };
    let t = 0.25;
    let d_edge = c.derivative(ed.param_range.0 + t * (ed.param_range.1 - ed.param_range.0));
    let d_seg = seg
        .curve
        .derivative(seg.range.0 + t * (seg.range.1 - seg.range.0));
    d_edge.dot(&d_seg) >= 0.0
}

/// Import all solids from IGES text: manifold solid B-reps (186) and sewn trimmed surfaces (144).
pub fn import(rep: &mut BRep, text: &str) -> Result<ImportReport, IgesError> {
    let file = parse(text)?;
    let ctx = Ctx {
        f: &file,
        scale: file.unit_mm,
    };
    let mut report = ImportReport {
        unit_mm: file.unit_mm,
        ..Default::default()
    };
    let scale = 1.0
        + file
            .entities
            .values()
            .flat_map(|e| e.params.iter())
            .map(|p| p.f().abs())
            .fold(0.0f64, f64::max)
            * file.unit_mm;
    let tol = 1e-6 * scale.min(1e6);
    let mut any = false;
    // solids via 186
    for de in &file.order {
        let e = &file.entities[de];
        if e.ty != 186 {
            continue;
        }
        any = true;
        match ctx.solid_faces(e, tol) {
            Ok(shells) => {
                for (faces, is_void) in shells {
                    if is_void {
                        report
                            .rejected_shells
                            .push("void shells of 186 are not imported separately".into());
                        continue;
                    }
                    report.faces_read += faces.len();
                    assemble(rep, faces, tol, &mut report);
                }
            }
            Err(er) => report.faces_skipped.push((*de, er)),
        }
    }
    // trimmed surfaces not part of a solid
    let mut loose = Vec::new();
    for de in &file.order {
        let e = &file.entities[de];
        if e.ty == 144 {
            any = true;
            match ctx.trimmed_face(e, tol) {
                Ok(f) => loose.push(f),
                Err(er) => report.faces_skipped.push((*de, er)),
            }
        }
    }
    report.faces_read += loose.len();
    assemble(rep, loose, tol, &mut report);
    if !any {
        return Err(IgesError::NothingToImport);
    }
    if report.bodies.is_empty() {
        let mut why: Vec<String> = report
            .faces_skipped
            .iter()
            .map(|(d, s)| format!("DE{d}: {s}"))
            .collect();
        why.extend(report.rejected_shells.iter().cloned());
        return Err(IgesError::NoSolids(why));
    }
    Ok(report)
}
