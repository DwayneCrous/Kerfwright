//! Sewing: join faces along coincident edges into sheets and solids.
//!
//! Faces are described by a [`Soup`]: surfaces, loops of edge uses, and edges that several faces may
//! use. A soup is made from existing bodies (with their edges shared as they were, or split apart),
//! faces can be added to it, [`Soup::weld`] merges coincident open edges of different faces, and
//! [`Soup::build`] turns every connected group into a body: faces are turned to agree with their
//! neighbours, a group whose edges are all used twice becomes a solid (turned so that its volume is
//! positive), any other a sheet.

use crate::history::{EntityRef, Origin, ShapeHistory};
use crate::ids::*;
use crate::store::{BRep, CoedgeSpec};
use crate::topology::*;
use kw_math::Point3;
use kw_nurbs::pcurve::{pcurve_error, pcurve_on_anchored};
use kw_nurbs::proj::{curve_param_of, surface_param_of};
use kw_nurbs::{Curve3, CurveGeom, Surface, SurfaceGeom};
use std::collections::{BTreeMap, HashMap};

/// An edge of a soup: a curve over a range between two points.
#[derive(Debug, Clone)]
pub struct EdgeDesc {
    /// `None` for an edge that is a single point (a pole or an apex).
    pub curve: Option<CurveGeom>,
    pub range: (f64, f64),
    pub start: Point3<f64>,
    pub end: Point3<f64>,
    /// How far the curve may lie from the faces that use it.
    pub tolerance: f64,
}

impl EdgeDesc {
    pub fn degenerate(&self) -> bool {
        self.curve.is_none()
    }
}

/// One use of an edge in a face loop.
#[derive(Debug, Clone)]
pub struct UseDesc {
    pub edge: usize,
    /// The loop runs against the direction of the edge.
    pub reversed: bool,
    /// The edge in the face's parameter space, running along the edge. Fitted if absent.
    pub pcurve: Option<CurveGeom>,
    /// Where the middle of the edge is in the face's parameter space, to pick the branch on a periodic
    /// surface when the pcurve has to be fitted.
    pub hint: Option<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct LoopDesc {
    pub kind: LoopKind,
    pub uses: Vec<UseDesc>,
}

#[derive(Debug, Clone)]
pub struct FaceDesc {
    pub surface: SurfaceGeom,
    pub sense: Sense,
    pub loops: Vec<LoopDesc>,
    /// Where the face comes from, for the shape history of the result.
    pub origin: Origin,
}

#[derive(Debug, Clone, Default)]
pub struct Soup {
    pub edges: Vec<EdgeDesc>,
    pub faces: Vec<FaceDesc>,
}

#[derive(Debug, Clone)]
pub struct BuildOptions {
    /// Points closer than this are one vertex.
    pub vertex_tol: f64,
    /// Put every group into one sheet body, one lump each, instead of one body per group.
    pub one_body: bool,
}

#[derive(Debug, Clone, Default)]
pub struct SewResult {
    pub bodies: Vec<BodyId>,
    pub history: ShapeHistory,
    /// Groups that could not be built, and why.
    pub rejected: Vec<String>,
}

impl Soup {
    /// Add the faces of `body`. With `share` the faces keep sharing the edges they share in the body;
    /// without it every face gets edges of its own.
    pub fn add_body(&mut self, rep: &BRep, body: BodyId, share: bool) {
        let mut map: HashMap<EdgeId, usize> = HashMap::new();
        for f in rep.body_faces(body) {
            let face = &rep.faces[f];
            let mut own: HashMap<EdgeId, usize> = HashMap::new();
            let mut loops = Vec::new();
            for lp in &face.loops {
                let mut uses = Vec::new();
                for co in rep.loop_coedges(*lp) {
                    let c = &rep.coedges[co];
                    let table = if share { &mut map } else { &mut own };
                    let idx = *table.entry(c.edge).or_insert_with(|| {
                        let e = &rep.edges[c.edge];
                        self.edges.push(EdgeDesc {
                            curve: e.curve.and_then(|id| rep.curves.get(id)).cloned(),
                            range: e.param_range,
                            start: rep.vertices[e.start].point,
                            end: rep.vertices[e.end].point,
                            tolerance: e.tolerance,
                        });
                        self.edges.len() - 1
                    });
                    uses.push(UseDesc {
                        edge: idx,
                        reversed: c.sense == Sense::Reversed,
                        pcurve: c.pcurve.and_then(|id| rep.curves.get(id)).cloned(),
                        hint: None,
                    });
                }
                loops.push(LoopDesc {
                    kind: rep.loops[*lp].kind,
                    uses,
                });
            }
            self.faces.push(FaceDesc {
                surface: rep.surfaces[face.surface].clone(),
                sense: face.sense,
                loops,
                origin: Origin::Preserved(EntityRef::Face(f)),
            });
        }
    }

    fn use_counts(&self) -> Vec<usize> {
        let mut n = vec![0usize; self.edges.len()];
        for f in &self.faces {
            for l in &f.loops {
                for u in &l.uses {
                    n[u.edge] += 1;
                }
            }
        }
        n
    }

    /// Merge the open edges of different faces that lie on one another within `tol`: each has to match
    /// exactly one other. Returns how many pairs were merged and what was left alone.
    pub fn weld(&mut self, tol: f64) -> (usize, Vec<String>) {
        let counts = self.use_counts();
        let free: Vec<usize> = (0..self.edges.len())
            .filter(|i| counts[*i] == 1 && !self.edges[*i].degenerate())
            .collect();
        let near = |a: &Point3<f64>, b: &Point3<f64>| (a - b).norm() <= tol;
        // candidates: (a, b, opposite)
        let mut cands: Vec<Vec<(usize, bool)>> = vec![Vec::new(); self.edges.len()];
        for (x, &i) in free.iter().enumerate() {
            for &j in &free[x + 1..] {
                let (a, b) = (&self.edges[i], &self.edges[j]);
                let same = near(&a.start, &b.start) && near(&a.end, &b.end);
                let opposite = near(&a.start, &b.end) && near(&a.end, &b.start);
                if !(same || opposite) {
                    continue;
                }
                if !self.on_each_other(i, j, tol) {
                    continue;
                }
                // a closed edge matches either way round; call it the same
                let opp = opposite && !same;
                cands[i].push((j, opp));
                cands[j].push((i, opp));
            }
        }
        let mut notes = Vec::new();
        let mut merged = 0;
        for &i in &free {
            if cands[i].len() > 1 {
                notes.push(format!(
                    "an open edge fits {} others, so it was left open",
                    cands[i].len()
                ));
            }
        }
        for &i in &free {
            let [(j, opp)] = cands[i][..] else { continue };
            // mutual, and each only once
            if j < i || cands[j].len() != 1 || cands[j][0].0 != i {
                continue;
            }
            self.merge(j, i, opp, tol);
            merged += 1;
        }
        (merged, notes)
    }

    fn on_each_other(&self, i: usize, j: usize, tol: f64) -> bool {
        let (a, b) = (&self.edges[i], &self.edges[j]);
        let (Some(ca), Some(cb)) = (&a.curve, &b.curve) else {
            return false;
        };
        [0.25, 0.5, 0.75].iter().all(|f| {
            let p = ca.point_at(a.range.0 + (a.range.1 - a.range.0) * f);
            curve_param_of(cb, &p, None).distance <= 4.0 * tol
        })
    }

    /// Replace edge `from` by edge `into` everywhere.
    fn merge(&mut self, from: usize, into: usize, opposite: bool, tol: f64) {
        let (a, b) = (self.edges[into].clone(), self.edges[from].clone());
        // does the other edge's parameter run the same way over the same curve?
        let same_param = !opposite
            && (a.range.0 - b.range.0).abs() < 1e-12
            && (a.range.1 - b.range.1).abs() < 1e-12
            && [0.0, 0.3, 0.7, 1.0].iter().all(|f| {
                let t = a.range.0 + (a.range.1 - a.range.0) * f;
                match (&a.curve, &b.curve) {
                    (Some(x), Some(y)) => (x.point_at(t) - y.point_at(t)).norm() <= tol,
                    _ => false,
                }
            });
        let dev = [0.25, 0.5, 0.75]
            .iter()
            .filter_map(|f| {
                let p = b
                    .curve
                    .as_ref()?
                    .point_at(b.range.0 + (b.range.1 - b.range.0) * f);
                Some(curve_param_of(a.curve.as_ref()?, &p, None).distance)
            })
            .fold(0.0f64, f64::max);
        self.edges[into].tolerance = self.edges[into].tolerance.max(b.tolerance).max(dev);
        let mid = 0.5 * (b.range.0 + b.range.1);
        for f in &mut self.faces {
            for l in &mut f.loops {
                for u in &mut l.uses {
                    if u.edge != from {
                        continue;
                    }
                    u.edge = into;
                    u.reversed ^= opposite;
                    if !same_param {
                        u.hint = u.pcurve.as_ref().map(|pc| {
                            let p = pc.point_at(mid);
                            (p.x, p.y)
                        });
                        u.pcurve = None;
                    }
                }
            }
        }
        self.edges[from].curve = None;
        self.edges[from].start = self.edges[into].start;
        self.edges[from].end = self.edges[into].start;
    }

    /// Build the soup into `rep`.
    pub fn build(&self, rep: &mut BRep, opts: &BuildOptions) -> SewResult {
        let mut out = SewResult::default();
        let counts = self.use_counts();
        // which uses an edge has: (face, loop, position)
        let mut users: HashMap<usize, Vec<(usize, usize, usize)>> = HashMap::new();
        for (fi, f) in self.faces.iter().enumerate() {
            for (li, l) in f.loops.iter().enumerate() {
                for (ui, u) in l.uses.iter().enumerate() {
                    users.entry(u.edge).or_default().push((fi, li, ui));
                }
            }
        }
        // groups of faces joined by shared edges
        let mut parent: Vec<usize> = (0..self.faces.len()).collect();
        fn find(p: &mut [usize], x: usize) -> usize {
            let mut r = x;
            while p[r] != r {
                r = p[r];
            }
            let mut y = x;
            while p[y] != r {
                let n = p[y];
                p[y] = r;
                y = n;
            }
            r
        }
        for list in users.values() {
            for w in list.windows(2) {
                let (a, b) = (find(&mut parent, w[0].0), find(&mut parent, w[1].0));
                parent[a] = b;
            }
        }
        let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for fi in 0..self.faces.len() {
            let r = find(&mut parent, fi);
            groups.entry(r).or_default().push(fi);
        }
        let scale = 1.0
            + self
                .edges
                .iter()
                .flat_map(|e| [e.start, e.end])
                .map(|p| p.coords.amax())
                .fold(0.0f64, f64::max);
        let vtol = opts.vertex_tol.max(1e-9 * scale);
        let mut built: Vec<(Vec<usize>, Vec<bool>, bool)> = Vec::new();
        for members in groups.values() {
            let name = format!("a group of {} faces", members.len());
            // an edge used more than twice is not a surface
            let manifold = members.iter().all(|fi| {
                self.faces[*fi].loops.iter().all(|l| {
                    l.uses
                        .iter()
                        .all(|u| self.edges[u.edge].degenerate() || counts[u.edge] <= 2)
                })
            });
            if !manifold {
                out.rejected
                    .push(format!("{name}: an edge is shared by more than two faces"));
                continue;
            }
            // turn the faces to agree: across an edge used twice the two uses must run opposite ways
            let mut flip: BTreeMap<usize, bool> = BTreeMap::new();
            let mut ok = true;
            for &start in members {
                if flip.contains_key(&start) {
                    continue;
                }
                flip.insert(start, false);
                let mut queue = vec![start];
                while let Some(fi) = queue.pop() {
                    for l in &self.faces[fi].loops {
                        for u in &l.uses {
                            if self.edges[u.edge].degenerate() {
                                continue;
                            }
                            for &(fj, lj, uj) in &users[&u.edge] {
                                if fj == fi && self.faces[fj].loops[lj].uses[uj].edge == u.edge {
                                    // the other use of a seam in the same face
                                    let other = &self.faces[fj].loops[lj].uses[uj];
                                    if std::ptr::eq(other, u) {
                                        continue;
                                    }
                                    if other.reversed == u.reversed {
                                        ok = false;
                                    }
                                    continue;
                                }
                                let other = &self.faces[fj].loops[lj].uses[uj];
                                // directions after turning: reversed ^ flip
                                let mine = u.reversed ^ flip[&fi];
                                let want_other = !mine;
                                let need_flip = other.reversed != want_other;
                                match flip.get(&fj) {
                                    None => {
                                        flip.insert(fj, need_flip);
                                        queue.push(fj);
                                    }
                                    Some(&f) => {
                                        if f != need_flip {
                                            ok = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if !ok {
                out.rejected.push(format!(
                    "{name} cannot be turned to face one way (a Möbius twist or crossing sheets)"
                ));
                continue;
            }
            let closed = members.iter().all(|fi| {
                self.faces[*fi].loops.iter().all(|l| {
                    l.uses
                        .iter()
                        .all(|u| self.edges[u.edge].degenerate() || counts[u.edge] == 2)
                })
            });
            built.push((
                members.clone(),
                members.iter().map(|m| flip[m]).collect(),
                closed,
            ));
        }
        if opts.one_body {
            let mut scratch = BRep::new();
            let body = scratch.add_body(BodyKind::Sheet);
            let mut origins: Vec<(FaceId, Origin)> = Vec::new();
            for (members, flips, _) in &built {
                let lump = scratch.add_lump(body);
                match self.construct(&mut scratch, lump, members, flips, vtol, scale) {
                    Ok(o) => origins.extend(o),
                    Err(e) => out.rejected.push(e),
                }
            }
            if scratch.body_faces(body).is_empty() {
                return out;
            }
            if let Err(e) = scratch.validate().into_result() {
                out.rejected.push(format!(
                    "the surface is invalid: {:?}",
                    &e[..e.len().min(3)]
                ));
                return out;
            }
            let (nb, map) = rep.copy_body_from(&scratch, body, &Default::default());
            for (f, o) in origins {
                if let Some(n) = map.faces.get(&f) {
                    out.history.record(EntityRef::Face(*n), o);
                }
            }
            out.bodies.push(nb);
            return out;
        }
        for (members, flips, closed) in built {
            let attempt =
                |flip_all: bool| -> Result<(BRep, BodyId, Vec<(FaceId, Origin)>), String> {
                    let mut scratch = BRep::new();
                    let kind = if closed {
                        BodyKind::Solid
                    } else {
                        BodyKind::Sheet
                    };
                    let body = scratch.add_body(kind);
                    let lump = scratch.add_lump(body);
                    let f: Vec<bool> = flips.iter().map(|x| x ^ flip_all).collect();
                    let origins = self.construct(&mut scratch, lump, &members, &f, vtol, scale)?;
                    Ok((scratch, body, origins))
                };
            let result = (|| {
                let (mut scratch, mut body, mut origins) = attempt(false)?;
                if closed {
                    let vol = scratch.volume(body).unwrap_or(f64::NAN);
                    if vol.is_nan() {
                        return Err("the volume of the closed surface cannot be found".to_string());
                    }
                    if vol < 0.0 {
                        (scratch, body, origins) = attempt(true)?;
                    }
                    let vol = scratch.volume(body).unwrap_or(f64::NAN);
                    if vol.is_nan() || vol <= 0.0 {
                        return Err("the closed surface encloses no volume".into());
                    }
                }
                scratch
                    .validate()
                    .into_result()
                    .map_err(|e| format!("the surface is invalid: {:?}", &e[..e.len().min(3)]))?;
                Ok((scratch, body, origins))
            })();
            match result {
                Ok((scratch, body, origins)) => {
                    let (nb, map) = rep.copy_body_from(&scratch, body, &Default::default());
                    for (f, o) in origins {
                        if let Some(n) = map.faces.get(&f) {
                            out.history.record(EntityRef::Face(*n), o);
                        }
                    }
                    out.bodies.push(nb);
                }
                Err(e) => out
                    .rejected
                    .push(format!("a group of {} faces: {e}", members.len())),
            }
        }
        out
    }

    /// The faces `members` (turned as `flips` say) as a lump of `scratch`.
    fn construct(
        &self,
        scratch: &mut BRep,
        lump: LumpId,
        members: &[usize],
        flips: &[bool],
        vtol: f64,
        scale: f64,
    ) -> Result<Vec<(FaceId, Origin)>, String> {
        let shell = scratch.add_shell(lump);
        // a hash grid of the vertices made so far, so that finding one is not a search through all of them
        let cell = (2.0 * vtol).max(1e-300);
        let cell_of = |p: &Point3<f64>| {
            [
                (p.x / cell).floor() as i64,
                (p.y / cell).floor() as i64,
                (p.z / cell).floor() as i64,
            ]
        };
        let mut vertices: HashMap<[i64; 3], Vec<(Point3<f64>, VertexId)>> = HashMap::new();
        let mut vertex = |s: &mut BRep, p: Point3<f64>| -> VertexId {
            let c = cell_of(&p);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(list) = vertices.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                            if let Some((_, v)) = list.iter().find(|(q, _)| (q - p).norm() <= vtol)
                            {
                                return *v;
                            }
                        }
                    }
                }
            }
            let v = s.add_vertex(p);
            vertices.entry(c).or_default().push((p, v));
            v
        };
        let mut edge_ids: HashMap<usize, EdgeId> = HashMap::new();
        let mut origins = Vec::new();
        for (k, &fi) in members.iter().enumerate() {
            let f = &self.faces[fi];
            let flip = flips[k];
            let sid = scratch.add_surface(f.surface.clone());
            let sense = if flip { f.sense.flip() } else { f.sense };
            let face = scratch.add_face(shell, sid, sense);
            origins.push((face, f.origin.clone()));
            for l in &f.loops {
                let mut specs: Vec<CoedgeSpec> = Vec::new();
                for u in &l.uses {
                    let d = &self.edges[u.edge];
                    let e = match edge_ids.get(&u.edge) {
                        Some(e) => *e,
                        None => {
                            let (a, b) = (vertex(scratch, d.start), vertex(scratch, d.end));
                            let cid = d.curve.clone().map(|c| scratch.add_curve(c));
                            let e = scratch.add_edge(a, b, cid, d.range);
                            scratch.edges[e].tolerance = d.tolerance;
                            edge_ids.insert(u.edge, e);
                            e
                        }
                    };
                    let pcurve = match (&u.pcurve, &d.curve) {
                        (Some(pc), _) => Some(scratch.add_curve(pc.clone())),
                        (None, Some(curve)) => {
                            let pc = fit_pcurve(&f.surface, curve, d.range, u.hint, scale)?;
                            let err = pcurve_error(&f.surface, curve, &pc, d.range, 32);
                            let e = &mut scratch.edges[e];
                            e.tolerance = e.tolerance.max(err);
                            Some(scratch.add_curve(pc))
                        }
                        (None, None) => None,
                    };
                    // how far the edge lies from this face: along its pcurve where it has one (cheap and
                    // exact), else by projecting
                    if let Some(curve) = &d.curve {
                        let pc = u.pcurve.as_ref();
                        let dev = (0..=8)
                            .map(|i| {
                                let t = d.range.0 + (d.range.1 - d.range.0) * i as f64 / 8.0;
                                let p = curve.point_at(t);
                                match pc {
                                    Some(pc) => {
                                        let q = pc.point_at(t);
                                        (f.surface.point_at(q.x, q.y) - p).norm()
                                    }
                                    None => surface_param_of(&f.surface, &p, None).distance,
                                }
                            })
                            .fold(0.0f64, f64::max);
                        let e = &mut scratch.edges[e];
                        e.tolerance = e.tolerance.max(dev);
                    }
                    specs.push(CoedgeSpec {
                        edge: e,
                        sense: if u.reversed {
                            Sense::Reversed
                        } else {
                            Sense::Forward
                        },
                        pcurve,
                    });
                }
                if flip {
                    specs.reverse();
                    for s in &mut specs {
                        s.sense = s.sense.flip();
                    }
                }
                scratch.add_loop(face, l.kind, &specs);
            }
        }
        Ok(origins)
    }
}

/// The edge in the parameter space of a surface, loosening the fit until it succeeds.
fn fit_pcurve(
    surface: &SurfaceGeom,
    curve: &CurveGeom,
    range: (f64, f64),
    hint: Option<(f64, f64)>,
    scale: f64,
) -> Result<CurveGeom, String> {
    let mid = 0.5 * (range.0 + range.1);
    let anchor = hint.map(|h| {
        let p = curve.point_at(mid);
        let r = surface_param_of(surface, &p, Some(h));
        (mid, (r.u, r.v))
    });
    let dev = (0..=16)
        .map(|i| {
            let t = range.0 + (range.1 - range.0) * i as f64 / 16.0;
            surface_param_of(surface, &curve.point_at(t), None).distance
        })
        .fold(0.0f64, f64::max);
    let base = (1e-9 * scale).max(2.0 * dev);
    for mult in [1.0, 10.0, 100.0, 1e3] {
        if let Some(pc) = pcurve_on_anchored(surface, curve, range, anchor, base * mult) {
            return Ok(pc);
        }
    }
    Err("an edge could not be placed on the surface of a face".into())
}
