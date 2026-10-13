//! The parts list and balloons in the Drawing workspace.

use super::source::source_for;
use super::DrawingWorkspace;
use crate::design::asm_inspect::density_of;
use crate::project::Project;
use kw_drawing::bom::BomRowData;
use kw_drawing::dim::DimStatus;
use kw_drawing::projection::ViewFrame;
use kw_drawing::{AnnotKind, Annotation, BomColumn, ViewSource, P};
use kw_math::{Point3, Transform3};

/// What the parts list and the balloons need to know about a source: for each body of the source
/// (in the order its B-rep lists them) which part it is and where it stands, and the rows.
pub struct BomInfo {
    pub names: Vec<String>,
    pub placements: Vec<Transform3>,
    pub rows: Vec<BomRowData>,
}

pub fn bom_info(project: &mut Project, which: &ViewSource) -> Result<BomInfo, String> {
    match which {
        ViewSource::Assembly => {
            let asm = project
                .assembly
                .as_ref()
                .ok_or("the project has no assembly")?;
            let pose = asm
                .pose(&mut project.engine)
                .map_err(|e| format!("the assembly could not be posed: {e:?}"))?;
            let rows = pose
                .bom(&density_of)
                .into_iter()
                .map(|r| BomRowData {
                    part: r.part,
                    count: r.count as u32,
                    material: r.material.unwrap_or_default(),
                    volume: r.volume,
                    mass_g: r.mass_g,
                    size: r.size,
                })
                .collect();
            Ok(BomInfo {
                names: pose.parts.clone(),
                placements: pose.placements.clone(),
                rows,
            })
        }
        _ => {
            // a design: each body is a part of its own
            let src = source_for(project, &ViewSource::Design)?;
            let bodies = src.design.ok_or("the design has no bodies")?;
            let mut names = Vec::new();
            let mut rows: Vec<BomRowData> = Vec::new();
            for b in &bodies.list {
                let (vol, size) = match (b.output.brep(), b.output.body()) {
                    (Some(rep), Some(body)) => (
                        rep.mass_properties(body).map_or(0.0, |m| m.volume),
                        exact_size(rep, body),
                    ),
                    _ => (0.0, [0.0; 3]),
                };
                names.push(b.name.clone());
                match rows.iter_mut().find(|r| r.part == b.name) {
                    Some(r) => r.count += 1,
                    None => rows.push(BomRowData {
                        part: b.name.clone(),
                        count: 1,
                        material: b.material.clone().unwrap_or_default(),
                        volume: vol,
                        mass_g: vol * density_of(b.material.as_deref()) * 1e-6,
                        size,
                    }),
                }
            }
            Ok(BomInfo {
                placements: vec![Transform3::identity(); names.len()],
                names,
                rows,
            })
        }
    }
}

/// The size of a body's bounding box from its tessellation (the B-rep's own box is padded around
/// curved faces).
fn exact_size(rep: &kw_brep::BRep, body: kw_brep::BodyId) -> [f64; 3] {
    let Ok(t) = kw_tess::tessellate_body(rep, body, &kw_tess::TessOptions::default()) else {
        let bb = rep.body_bbox(body);
        return [
            bb.max.x - bb.min.x,
            bb.max.y - bb.min.y,
            bb.max.z - bb.min.z,
        ];
    };
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in t.mesh.positions.iter().chain(t.edges.points.iter()) {
        for (k, v) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[k] = lo[k].min(v);
            hi[k] = hi[k].max(v);
        }
    }
    [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]
}

impl DrawingWorkspace {
    /// The source the parts list of the shown sheet is made from: that of its first view.
    fn bom_source(&self, project: &Project, sheet: usize) -> ViewSource {
        project
            .drawings
            .sheets
            .get(sheet)
            .and_then(|s| s.views.first())
            .map(|v| v.source.clone())
            .unwrap_or(if project.assembly.is_some() {
                ViewSource::Assembly
            } else {
                ViewSource::Design
            })
    }

    /// A parts list at the top of the title block of the shown sheet.
    pub fn add_bom(&mut self, project: &mut Project) -> Result<u64, String> {
        let sheet = self.sheet;
        let s = project
            .drawings
            .sheets
            .get(sheet)
            .ok_or("make a drawing first")?;
        if s.annots
            .iter()
            .any(|a| matches!(a.kind, AnnotKind::Bom { .. }))
        {
            return Err("this sheet already has a parts list".into());
        }
        let pos = s.format.frame().title.1;
        let which = self.bom_source(project, sheet);
        let info = bom_info(project, &which)?;
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[sheet].annots.push(Annotation {
            id,
            kind: AnnotKind::Bom {
                pos,
                columns: BomColumn::defaults(),
                rows: info.rows,
            },
        });
        project.mark_extras_dirty();
        self.selected = None;
        self.selected_annot = Some(id);
        Ok(id)
    }

    /// A balloon for the component showing at the point `uv` of `view` (the view's coordinates, model
    /// units), placed at `pos` on the sheet.
    pub fn add_balloon(
        &mut self,
        project: &mut Project,
        view: u64,
        uv: P,
        pos: P,
    ) -> Result<u64, String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(view).is_some())
            .ok_or("that view does not exist")?;
        let v = project.drawings.sheets[sheet].view(view).unwrap().clone();
        let src = source_for(project, &v.source)?;
        let (bi, world) =
            kw_drawing::pick::body_at(&src.brep, &src.bodies, &ViewFrame::of(v.orientation), uv)
                .ok_or("there is no component at that point")?;
        let info = bom_info(project, &v.source)?;
        let part = info.names.get(bi).ok_or("that body has no part")?.clone();
        let instance = info.names[..bi].iter().filter(|n| **n == part).count() as u32;
        let local = info.placements[bi]
            .inverse()
            .apply_point(&Point3::new(world[0], world[1], world[2]));
        let item = info
            .rows
            .iter()
            .position(|r| r.part == part)
            .map_or(0, |i| i as u32 + 1);
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[sheet].annots.push(Annotation {
            id,
            kind: AnnotKind::Balloon {
                view,
                part,
                instance,
                local: [local.x, local.y, local.z],
                world,
                pos,
                item,
                status: DimStatus::Ok,
            },
        });
        project.mark_extras_dirty();
        self.sheet = sheet;
        self.selected = None;
        self.selected_annot = Some(id);
        Ok(id)
    }

    /// Read the parts lists and balloons again from the model: the tables get the current rows, the
    /// balloons their item numbers (the table's order) and their points, which follow the
    /// components; a balloon whose part is gone says so.
    pub fn refresh_bom(&mut self, project: &mut Project) {
        let mut infos: Vec<(ViewSource, Option<BomInfo>)> = Vec::new();
        for si in 0..project.drawings.sheets.len() {
            if !project.drawings.sheets[si]
                .annots
                .iter()
                .any(|a| matches!(a.kind, AnnotKind::Bom { .. } | AnnotKind::Balloon { .. }))
            {
                continue;
            }
            let sheet_source = self.bom_source(project, si);
            for ai in 0..project.drawings.sheets[si].annots.len() {
                let which = match &project.drawings.sheets[si].annots[ai].kind {
                    AnnotKind::Bom { .. } => sheet_source.clone(),
                    AnnotKind::Balloon { view, .. } => {
                        match project.drawings.sheets[si].view(*view) {
                            Some(v) => v.source.clone(),
                            None => continue,
                        }
                    }
                    _ => continue,
                };
                if !infos.iter().any(|(w, _)| *w == which) {
                    let info = bom_info(project, &which).ok();
                    infos.push((which.clone(), info));
                }
                let Some((_, Some(info))) = infos.iter().find(|(w, _)| *w == which) else {
                    continue;
                };
                match &mut project.drawings.sheets[si].annots[ai].kind {
                    AnnotKind::Bom { rows, .. } => *rows = info.rows.clone(),
                    AnnotKind::Balloon {
                        part,
                        instance,
                        local,
                        world,
                        item,
                        status,
                        ..
                    } => {
                        let body = info
                            .names
                            .iter()
                            .enumerate()
                            .filter(|(_, n)| *n == part)
                            .nth(*instance as usize)
                            .map(|(i, _)| i);
                        match body {
                            Some(i) => {
                                let w = info.placements[i]
                                    .apply_point(&Point3::new(local[0], local[1], local[2]));
                                *world = [w.x, w.y, w.z];
                                *item = info
                                    .rows
                                    .iter()
                                    .position(|r| r.part == *part)
                                    .map_or(0, |k| k as u32 + 1);
                                *status = DimStatus::Ok;
                            }
                            None => {
                                *status = DimStatus::Broken(format!(
                                    "the part {part} is no longer in the model"
                                ));
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        project.mark_extras_dirty();
    }

    /// The parts list as CSV: the selected one, or else the first of the shown sheet.
    pub fn bom_csv(&self, project: &Project) -> Option<String> {
        let s = project.drawings.sheets.get(self.sheet)?;
        let pick = |id: Option<u64>| {
            s.annots.iter().find_map(|a| match &a.kind {
                AnnotKind::Bom { rows, .. } if id.is_none_or(|i| i == a.id) => {
                    Some(kw_drawing::bom::bom_csv(rows))
                }
                _ => None,
            })
        };
        pick(self.selected_annot).or_else(|| pick(None))
    }
}
