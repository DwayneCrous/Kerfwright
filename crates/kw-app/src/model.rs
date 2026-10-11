//! The Model workspace: the feature timeline, an inspector for the selected feature, the toolbar that adds
//! features, and the 3-D viewport.

use crate::project::Project;
use crate::viewport::Pick;
use kw_expr::{Dim, Param};
use kw_feature::{BodyOp, Document, FeatureId, PlaneSpec};

/// Text of a parameter as the user would type it.
pub fn param_text(p: &Param) -> String {
    let trim = |v: f64| {
        let s = format!("{v:.6}");
        let s = s.trim_end_matches('0').trim_end_matches('.');
        if s.is_empty() || s == "-" {
            "0".to_string()
        } else {
            s.to_string()
        }
    };
    match p {
        Param::Lit(q) if q.dim == Dim::LENGTH => format!("{} mm", trim(q.value)),
        Param::Lit(q) if q.dim == Dim::ANGLE => format!("{} deg", trim(q.value.to_degrees())),
        Param::Lit(q) => trim(q.value),
        Param::Expr(e) => e.source().to_string(),
    }
}

/// A plain number means millimetres / degrees / a count (by `dim`); anything else is an expression.
pub fn parse_param(text: &str, dim: Dim) -> Result<Param, String> {
    let t = text.trim();
    if t.is_empty() {
        return Err("enter a value".into());
    }
    if let Ok(v) = t.parse::<f64>() {
        return Ok(if dim == Dim::LENGTH {
            Param::length(v)
        } else if dim == Dim::ANGLE {
            Param::angle_deg(v)
        } else {
            Param::scalar(v)
        });
    }
    Param::expr(t).map_err(|e| e.to_string())
}

pub fn unique_name(doc: &Document, base: &str) -> String {
    (1..)
        .map(|n| format!("{base} {n}"))
        .find(|n| doc.find_by_name(n).is_none())
        .expect("a free name")
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlaneChoice {
    Xy,
    Yz,
    Xz,
}

#[derive(Default)]
pub struct ModelWorkspace {
    pub selected: Option<FeatureId>,
    /// Faces and edges picked in the viewport (the last one is highlighted).
    pub picks: Vec<Pick>,
    pub error: String,
}

fn plane_spec(p: PlaneChoice) -> PlaneSpec {
    match p {
        PlaneChoice::Xy => PlaneSpec::Xy(Param::length(0.0)),
        PlaneChoice::Yz => PlaneSpec::Yz(Param::length(0.0)),
        PlaneChoice::Xz => PlaneSpec::Xz(Param::length(0.0)),
    }
}

impl ModelWorkspace {
    // ---------------------------------------------------------------------------------- edits

    pub fn edit<T>(
        &mut self,
        project: &mut Project,
        f: impl FnOnce(&mut Document) -> Result<T, kw_feature::TimelineError>,
    ) -> Option<T> {
        match project.history.edit(f) {
            Ok(v) => {
                self.error.clear();
                Some(v)
            }
            Err(e) => {
                self.error = e.to_string();
                None
            }
        }
    }

    pub fn add_rect_sketch(
        &mut self,
        project: &mut Project,
        plane: &str,
        x0: &str,
        y0: &str,
        w: &str,
        h: &str,
    ) -> Option<FeatureId> {
        let p = |t: &str| parse_param(t, Dim::LENGTH);
        let (x0, y0, w, h) = match (p(x0), p(y0), p(w), p(h)) {
            (Ok(a), Ok(b), Ok(c), Ok(d)) => (a, b, c, d),
            (a, b, c, d) => {
                self.error = [a.err(), b.err(), c.err(), d.err()]
                    .into_iter()
                    .flatten()
                    .next()
                    .unwrap_or_default();
                return None;
            }
        };
        let plane = match plane {
            "yz" => PlaneChoice::Yz,
            "xz" => PlaneChoice::Xz,
            _ => PlaneChoice::Xy,
        };
        let name = unique_name(&project.document(), "Sketch");
        let id = self.edit(project, |d| {
            d.add_rect_sketch(&name, plane_spec(plane), x0, y0, w, h)
        })?;
        self.selected = Some(id);
        Some(id)
    }

    pub fn add_extrude(
        &mut self,
        project: &mut Project,
        sketch: FeatureId,
        depth: &str,
        op: BodyOp,
    ) -> Option<FeatureId> {
        let depth = match parse_param(depth, Dim::LENGTH) {
            Ok(d) => d,
            Err(e) => {
                self.error = e;
                return None;
            }
        };
        let name = unique_name(&project.document(), "Extrude");
        let id = self.edit(project, |d| d.add_extrude(&name, sketch, depth, op))?;
        self.selected = Some(id);
        Some(id)
    }

    pub fn add_parameter(
        &mut self,
        project: &mut Project,
        name: &str,
        value: &str,
    ) -> Option<FeatureId> {
        let v = match parse_param(value, Dim::LENGTH) {
            Ok(v) => v,
            Err(e) => {
                self.error = e;
                return None;
            }
        };
        let name = name.trim().to_string();
        let id = self.edit(project, |d| d.add_parameter(&name, v))?;
        self.selected = Some(id);
        Some(id)
    }

    /// Fillet the picked edges. Each edge needs the two faces it joins, named so the fillet survives edits.
    pub fn add_fillet(&mut self, project: &mut Project, radius: &str) -> Option<FeatureId> {
        let r = match parse_param(radius, Dim::LENGTH) {
            Ok(r) => r,
            Err(e) => {
                self.error = e;
                return None;
            }
        };
        let ev = project.evaluation();
        let Some((_, out)) = ev.final_solid() else {
            self.error = "there is no solid to fillet".into();
            return None;
        };
        let (Some(brep), names) = (out.brep(), &out.names) else {
            return None;
        };
        let mut edges = Vec::new();
        for p in &self.picks {
            if let Pick::Edge(e) = p {
                let faces = brep.faces_around_edge(*e);
                let refs: Vec<_> = faces
                    .iter()
                    .filter_map(|f| names.reference_to(brep, *f))
                    .collect();
                if refs.len() == 2 {
                    edges.push([refs[0].clone(), refs[1].clone()]);
                } else {
                    self.error = "an edge must join exactly two named faces".into();
                    return None;
                }
            }
        }
        if edges.is_empty() {
            self.error =
                "pick one or more edges first (click edges in the view, Shift adds)".into();
            return None;
        }
        let name = unique_name(&project.document(), "Fillet");
        let id = self.edit(project, |d| d.add_fillet(&name, edges, r))?;
        self.picks.clear();
        self.selected = Some(id);
        Some(id)
    }

    pub fn add_pattern(
        &mut self,
        project: &mut Project,
        source: FeatureId,
        spacing: [&str; 3],
        count: &str,
    ) -> Option<FeatureId> {
        let sp: Result<Vec<Param>, String> = spacing
            .iter()
            .map(|t| parse_param(t, Dim::LENGTH))
            .collect();
        let (sp, c) = match (sp, parse_param(count, Dim::NONE)) {
            (Ok(s), Ok(c)) => (s, c),
            (Err(e), _) | (_, Err(e)) => {
                self.error = e;
                return None;
            }
        };
        let name = unique_name(&project.document(), "Pattern");
        let id = self.edit(project, |d| {
            d.add_linear_pattern(
                &name,
                source,
                [sp[0].clone(), sp[1].clone(), sp[2].clone()],
                c,
            )
        })?;
        self.selected = Some(id);
        Some(id)
    }
}
