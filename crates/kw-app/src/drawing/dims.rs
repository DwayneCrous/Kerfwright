//! Dimensions in the Drawing workspace: attaching them to the model, finding them again after the
//! model changes, and making them from picks.

use super::source::{source_for, Source};
use super::DrawingWorkspace;
use crate::project::Project;
use crate::viewport::Pick;
use kw_brep::{BRep, EdgeId, FaceId};
use kw_drawing::dim::{measure, Anchor, DimKind, DimStatus, Dimension, PointKind, Resolved};
use kw_drawing::projection::ViewFrame;
use kw_drawing::snap::{cand_visible, candidates, Cand, CandKind};
use kw_drawing::{AnnotKind, Annotation, ViewSource};
use kw_feature::naming::Resolved as FaceRes;
use kw_feature::EdgeRef;
use kw_nurbs::CurveGeom;

/// Can a pick of this kind be used for a dimension of this kind?
pub(super) fn fits(kind: DimKind, pick: CandKind) -> bool {
    matches!(
        (kind, pick),
        (DimKind::Diameter | DimKind::Radius, CandKind::Circle)
            | (DimKind::Angle, CandKind::Line)
            | (
                DimKind::Horizontal | DimKind::Vertical | DimKind::Aligned,
                CandKind::Point(_)
            )
    )
}

/// The anchor for a picked point or edge: tied to the model through the two faces that meet at
/// the edge when the source is the design, fixed in space otherwise.
pub fn anchor_for(src: &Source, c: &Cand) -> Result<Anchor, String> {
    let Some(bodies) = &src.design else {
        return Ok(Anchor::Fixed(match c.kind {
            CandKind::Point(_) => Resolved::Point(c.p3),
            CandKind::Circle => {
                let (centre, radius, axis) = c.circle.ok_or("not a circle")?;
                Resolved::Circle {
                    centre,
                    radius,
                    axis,
                }
            }
            CandKind::Line => {
                let (a, b) = c.line.ok_or("not a line")?;
                Resolved::Line { a, b }
            }
        }));
    };
    let (i, pick) = bodies
        .origin(Pick::Edge(c.edge))
        .ok_or("that edge is not part of the design")?;
    let Pick::Edge(local) = pick else {
        return Err("that is not an edge".into());
    };
    let out = &bodies.list[i].output;
    let rep = out.brep().ok_or("that body has no solid")?;
    let mut faces = rep.faces_around_edge(local);
    faces.dedup();
    if faces.len() != 2 || faces[0] == faces[1] {
        return Err("an edge must join exactly two faces to be dimensioned".into());
    }
    let refs: Vec<_> = faces
        .iter()
        .filter_map(|f| out.names.reference_to(rep, *f))
        .collect();
    if refs.len() != 2 {
        return Err("the faces at that edge have no persistent names".into());
    }
    let edge = EdgeRef {
        faces: [refs[0].clone(), refs[1].clone()],
    };
    let body = i as u32;
    Ok(match c.kind {
        CandKind::Point(at) => Anchor::EdgePoint { body, edge, at },
        CandKind::Circle => Anchor::Circle { body, edge },
        CandKind::Line => Anchor::Line { body, edge },
    })
}

fn shared_edge(rep: &BRep, a: FaceId, b: FaceId) -> Option<EdgeId> {
    for lp in &rep.faces.get(a)?.loops {
        for co in rep.loop_coedges(*lp) {
            let e = rep.coedges.get(co)?.edge;
            if rep.faces_around_edge(e).contains(&b) {
                return Some(e);
            }
        }
    }
    None
}

fn pt(p: &kw_math::Point3<f64>) -> [f64; 3] {
    [p.x, p.y, p.z]
}

/// Find an anchor in the model as it is now. A face that is gone or ambiguous is an error, never a guess.
pub fn resolve_anchor(src: &Source, a: &Anchor) -> Result<Resolved, String> {
    let (body, er) = match a {
        Anchor::Fixed(r) => return Ok(r.clone()),
        Anchor::EdgePoint { body, edge, .. }
        | Anchor::Circle { body, edge }
        | Anchor::Line { body, edge } => (*body, edge),
    };
    let bodies = src
        .design
        .as_ref()
        .ok_or("this dimension belongs to a design")?;
    let out = &bodies
        .list
        .get(body as usize)
        .ok_or("the body this dimension was attached to is gone")?
        .output;
    let rep = out.brep().ok_or("that body has no solid")?;
    let face = |r: &kw_feature::naming::Reference| match out.names.resolve_face(rep, r) {
        FaceRes::Face(f, _) => Ok(f),
        FaceRes::Ambiguous(_) => Err("a face this dimension is attached to has become ambiguous"),
        FaceRes::Broken => Err("a face this dimension is attached to no longer exists"),
    };
    let (fa, fb) = (face(&er.faces[0])?, face(&er.faces[1])?);
    let e =
        shared_edge(rep, fa, fb).ok_or("the faces this dimension is attached to no longer meet")?;
    let edge = rep.edges.get(e).ok_or("the edge is gone")?;
    let curve = edge
        .curve
        .and_then(|c| rep.curves.get(c))
        .ok_or("the edge has no curve")?;
    let (s, z) = (
        pt(&rep.vertices.get(edge.start).ok_or("edge start")?.point),
        pt(&rep.vertices.get(edge.end).ok_or("edge end")?.point),
    );
    match (a, curve) {
        (Anchor::EdgePoint { at, .. }, curve) => Ok(Resolved::Point(match (at, curve) {
            (PointKind::Start, _) => s,
            (PointKind::End, _) => z,
            (PointKind::Mid, _) => [
                (s[0] + z[0]) / 2.0,
                (s[1] + z[1]) / 2.0,
                (s[2] + z[2]) / 2.0,
            ],
            (PointKind::Centre, CurveGeom::Circle(c)) => pt(&c.frame.origin),
            (PointKind::Centre, CurveGeom::Ellipse(c)) => pt(&c.frame.origin),
            (PointKind::Centre, _) => return Err("that edge has no centre".into()),
        })),
        (Anchor::Circle { .. }, CurveGeom::Circle(c)) => Ok(Resolved::Circle {
            centre: pt(&c.frame.origin),
            radius: c.radius,
            axis: [c.frame.z.x, c.frame.z.y, c.frame.z.z],
        }),
        (Anchor::Line { .. }, CurveGeom::Line(_)) => Ok(Resolved::Line { a: s, b: z }),
        _ => Err("the edge this dimension is attached to has changed its kind".into()),
    }
}

/// The dimension's anchors found again; its value measured from them in its view.
pub fn refresh(d: &mut Dimension, src: &Source, frame: &ViewFrame) {
    let found: Result<Vec<Resolved>, String> =
        d.anchors.iter().map(|a| resolve_anchor(src, a)).collect();
    match found {
        Ok(r) => {
            d.resolved = r;
            match measure(d, frame) {
                Some(v) => {
                    d.value = v;
                    d.status = DimStatus::Ok;
                }
                None => d.status = DimStatus::Broken("the dimension cannot be measured".into()),
            }
        }
        // the last good geometry stays, drawn as it was, and the dimension says what is wrong
        Err(e) => d.status = DimStatus::Broken(e),
    }
}

impl DrawingWorkspace {
    /// Find every dimension of every sheet in the model again and re-measure it.
    pub fn refresh_dimensions(&mut self, project: &mut Project) {
        let mut sources: Vec<(ViewSource, Source)> = Vec::new();
        let n = project.drawings.sheets.len();
        for si in 0..n {
            for ai in 0..project.drawings.sheets[si].annots.len() {
                let AnnotKind::Dimension(d) = &project.drawings.sheets[si].annots[ai].kind else {
                    continue;
                };
                let Some(v) = project.drawings.sheets[si].view(d.view) else {
                    continue;
                };
                let (which, frame) = (v.source.clone(), ViewFrame::of(v.orientation));
                if !sources.iter().any(|(w, _)| *w == which) {
                    match source_for(project, &which) {
                        Ok(s) => sources.push((which.clone(), s)),
                        Err(_) => continue,
                    }
                }
                let Some((_, src)) = sources.iter().find(|(w, _)| *w == which) else {
                    continue;
                };
                if let AnnotKind::Dimension(d) = &mut project.drawings.sheets[si].annots[ai].kind {
                    refresh(d, src, &frame);
                }
            }
        }
        project.mark_extras_dirty();
    }

    /// What can be dimensioned in a view: its model points and edges that are drawn visible.
    pub fn candidates_for(&self, project: &mut Project, view: u64) -> Result<Vec<Cand>, String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(view).is_some())
            .ok_or("that view does not exist")?;
        let v = project.drawings.sheets[sheet].view(view).unwrap().clone();
        let cache = v
            .cache
            .as_ref()
            .ok_or("the view has not been generated yet")?;
        if matches!(
            v.kind,
            kw_drawing::ViewKind::Section { .. } | kw_drawing::ViewKind::Detail { .. }
        ) {
            return Err("dimensions on section and detail views are not available yet".into());
        }
        let src = source_for(project, &v.source)?;
        let frame = ViewFrame::of(v.orientation);
        let all = candidates(&src.brep, &src.bodies, &frame, v.scale, cache.centre);
        // only what the view shows visible (its points lie on a drawn line)
        Ok(all
            .into_iter()
            .filter(|c| cand_visible(c, &cache.scene, &frame, v.scale, cache.centre, 0.3))
            .collect())
    }

    /// A dimension of `kind` between / of the picked points and edges of a view. `line_pos` and
    /// `angle` place it (see [`Dimension`]); circles that look alike get a count (`4×`).
    pub fn add_dimension(
        &mut self,
        project: &mut Project,
        view: u64,
        kind: DimKind,
        picks: &[Cand],
        line_pos: f64,
        angle: f64,
    ) -> Result<u64, String> {
        let want = match kind {
            DimKind::Diameter | DimKind::Radius => 1,
            _ => 2,
        };
        if picks.len() != want {
            return Err(format!("this dimension needs {want} pick(s)"));
        }
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(view).is_some())
            .ok_or("that view does not exist")?;
        let v = project.drawings.sheets[sheet].view(view).unwrap().clone();
        let src = source_for(project, &v.source)?;
        let mut anchors = Vec::new();
        for c in picks {
            let ok = fits(kind, c.kind);
            if !ok {
                return Err("that pick does not fit this kind of dimension".into());
            }
            anchors.push(anchor_for(&src, c)?);
        }
        let mut d = Dimension::new(view, kind, anchors);
        d.line_pos = line_pos;
        d.angle = angle;
        refresh(&mut d, &src, &ViewFrame::of(v.orientation));
        if let DimStatus::Broken(e) = &d.status {
            return Err(e.clone());
        }
        if kind == DimKind::Diameter {
            let n = self.similar_holes(project, view, &picks[0]);
            if n > 1 {
                d.prefix = format!("{n}×");
            }
        }
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[sheet].annots.push(Annotation {
            id,
            kind: AnnotKind::Dimension(d),
        });
        project.mark_extras_dirty();
        self.selected = None;
        self.selected_annot = Some(id);
        Ok(id)
    }

    /// How many circles in the view have the radius and axis of this one (counted once per hole,
    /// not once per end of it).
    fn similar_holes(&self, project: &mut Project, view: u64, c: &Cand) -> usize {
        let Some((_, r, axis)) = c.circle else {
            return 1;
        };
        let Ok(all) = self.candidates_for(project, view) else {
            return 1;
        };
        let mut centres: Vec<[f64; 2]> = Vec::new();
        for o in all.iter().filter(|o| matches!(o.kind, CandKind::Circle)) {
            let Some((_, r2, a2)) = o.circle else {
                continue;
            };
            let par = (axis[0] * a2[0] + axis[1] * a2[1] + axis[2] * a2[2]).abs();
            if (r2 - r).abs() < 1e-9
                && par > 0.999
                && !centres
                    .iter()
                    .any(|p| (p[0] - o.p2[0]).hypot(p[1] - o.p2[1]) < 1e-6)
            {
                centres.push(o.p2);
            }
        }
        centres.len().max(1)
    }
}

// ------------------------------------------------------------------------------ notes and symbols

use kw_drawing::{Gdt, P};

impl DrawingWorkspace {
    fn push_annot(&mut self, project: &mut Project, kind: AnnotKind) -> Result<u64, String> {
        if project.drawings.sheets.get(self.sheet).is_none() {
            return Err("make a drawing first".into());
        }
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[self.sheet]
            .annots
            .push(Annotation { id, kind });
        project.mark_extras_dirty();
        self.selected = None;
        self.selected_annot = Some(id);
        Ok(id)
    }

    /// A note at `pos` (the baseline of its first line), with an arrow to `leader` if given.
    pub fn add_note(
        &mut self,
        project: &mut Project,
        pos: P,
        text: &str,
        leader: Option<P>,
    ) -> Result<u64, String> {
        self.push_annot(
            project,
            AnnotKind::Note {
                pos,
                text: text.to_string(),
                height: 3.5,
                leader,
            },
        )
    }

    /// A datum feature symbol with the next unused letter, its triangle on `target`.
    pub fn add_datum(&mut self, project: &mut Project, pos: P, target: P) -> Result<u64, String> {
        let used: Vec<String> = project
            .drawings
            .sheets
            .get(self.sheet)
            .map(|s| {
                s.annots
                    .iter()
                    .filter_map(|a| match &a.kind {
                        AnnotKind::Datum { letter, .. } => Some(letter.clone()),
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default();
        let letter = ('A'..='Z')
            .filter(|c| !"IOQSXZ".contains(*c))
            .map(|c| c.to_string())
            .find(|l| !used.contains(l))
            .unwrap_or_else(|| "?".into());
        self.push_annot(
            project,
            AnnotKind::Datum {
                pos,
                letter,
                target,
            },
        )
    }

    pub fn add_surface(
        &mut self,
        project: &mut Project,
        pos: P,
        value: &str,
    ) -> Result<u64, String> {
        self.push_annot(
            project,
            AnnotKind::Surface {
                pos,
                value: value.to_string(),
            },
        )
    }

    pub fn add_fcf(
        &mut self,
        project: &mut Project,
        pos: P,
        symbol: Gdt,
        tol: &str,
        datums: &[&str],
        target: Option<P>,
    ) -> Result<u64, String> {
        self.push_annot(
            project,
            AnnotKind::Fcf {
                pos,
                symbol,
                tol: tol.to_string(),
                datums: datums.iter().map(|d| d.to_string()).collect(),
                target,
            },
        )
    }

    pub fn delete_annot(&mut self, project: &mut Project, id: u64) -> Result<(), String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.annots.iter().any(|a| a.id == id))
            .ok_or("that annotation does not exist")?;
        project.dw_checkpoint();
        project.drawings.sheets[sheet].annots.retain(|a| a.id != id);
        project.mark_extras_dirty();
        if self.selected_annot == Some(id) {
            self.selected_annot = None;
        }
        Ok(())
    }
}
