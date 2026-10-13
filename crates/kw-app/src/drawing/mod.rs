//! The Drawing workspace: 2-D drawing sheets made from the 3-D design. A sheet holds views of the
//! design (base and projected), generated off the UI thread, and is exported as PDF or DXF.
//!
//! The logic lives in `kw-drawing`; this module holds the state of the workspace, the commands,
//! and the jobs, and `canvas` paints the sheet.

mod canvas;
mod panels;
mod source;

use crate::dialogs::Dialogs;
use crate::job::Job;
use crate::project::Project;
use kw_drawing::hlr::ViewGeometry;
use kw_drawing::projection::{neighbour_offset, Side, ViewFrame};
use kw_drawing::section::{cut_directions, section_frame, slice_loops};
use kw_drawing::update::{auto_scale, extent, generate, GenKind, GenSpec, STANDARD_SCALES};
use kw_drawing::{
    Align, Orientation, PaperSize, SheetFormat, Standard, View, ViewCache, ViewKind, ViewSource,
    ViewStyle,
};
use kw_sketcher::View as View2d;
use source::{source_for, Source};

pub use source::current_hash;

/// Gap between a projected view and its parent, mm.
const GAP: f64 = 25.0;

#[derive(Debug, Clone)]
pub struct NewDrawing {
    pub assembly: bool,
    pub standard: Standard,
    pub size: PaperSize,
    pub title: String,
    pub number: String,
    pub author: String,
}

#[derive(Debug, Clone)]
pub struct ViewForm {
    pub orientation: Orientation,
    pub style: ViewStyle,
    pub assembly: bool,
    /// Index into the standard scales; `None` chooses one that fits the sheet.
    pub scale: Option<usize>,
}

#[derive(Debug, Clone)]
pub enum Dialog {
    NewDrawing(NewDrawing),
    BaseView(ViewForm),
}

/// A tool waiting for clicks on the sheet: the parent view and first point (in the parent's view
/// coordinates, model units) once the first click is made.
#[derive(Debug, Clone, PartialEq)]
pub enum Tool {
    Section { first: Option<(u64, [f64; 2])> },
    Detail { first: Option<(u64, [f64; 2])> },
}

type Generated = (Vec<(u64, ViewGeometry)>, u64);

struct Drag {
    view: u64,
}

pub struct DrawingWorkspace {
    /// The sheet shown.
    pub sheet: usize,
    view: View2d,
    /// Where the canvas was drawn last, for turning sheet points into screen points.
    rect: eframe::egui::Rect,
    fit_for: Option<(usize, u64)>,
    pub selected: Option<u64>,
    pub dialog: Option<Dialog>,
    job: Option<Job<Generated>>,
    /// Views waiting for the running job to finish.
    queue: Vec<u64>,
    pub status: String,
    drag: Option<Drag>,
    pub tool: Option<Tool>,
    /// Ribbon button ids to run next frame.
    pub requests: Vec<&'static str>,
}

impl Default for DrawingWorkspace {
    fn default() -> Self {
        DrawingWorkspace {
            sheet: 0,
            view: View2d::default(),
            rect: eframe::egui::Rect::NOTHING,
            fit_for: None,
            selected: None,
            dialog: None,
            job: None,
            queue: Vec::new(),
            status: String::new(),
            drag: None,
            tool: None,
            requests: Vec::new(),
        }
    }
}

/// Orientations offered when making a base view.
pub const ORIENTATIONS: [(Orientation, &str); 7] = [
    (Orientation::Front, "Front"),
    (Orientation::Back, "Back"),
    (Orientation::Top, "Top"),
    (Orientation::Bottom, "Bottom"),
    (Orientation::Left, "Left"),
    (Orientation::Right, "Right"),
    (Orientation::Iso, "Isometric"),
];

pub fn orientation_name(o: Orientation) -> &'static str {
    ORIENTATIONS
        .iter()
        .find(|(x, _)| *x == o)
        .map_or("Custom", |(_, n)| n)
}

pub fn style_name(s: ViewStyle) -> &'static str {
    match s {
        ViewStyle::Visible => "Visible lines",
        ViewStyle::VisibleHidden => "Visible and hidden lines",
        ViewStyle::Wireframe => "Wireframe",
        ViewStyle::Shaded => "Shaded",
    }
}

impl DrawingWorkspace {
    /// The screen position of a point on the sheet (mm), as drawn last.
    pub fn screen_of(&self, p: [f64; 2]) -> eframe::egui::Pos2 {
        self.view.to_screen(self.rect, (p[0], p[1]))
    }

    pub fn busy(&self) -> bool {
        self.job.is_some() || !self.queue.is_empty()
    }

    pub fn current_sheet<'a>(&self, p: &'a Project) -> Option<&'a kw_drawing::Sheet> {
        p.drawings.sheets.get(self.sheet)
    }

    /// The default form of the New Drawing dialog for this project.
    pub fn new_drawing_form(&self, p: &Project) -> NewDrawing {
        NewDrawing {
            assembly: p.assembly.is_some(),
            standard: Standard::Iso,
            size: PaperSize::A3,
            title: p.name(),
            number: String::new(),
            author: String::new(),
        }
    }

    // ------------------------------------------------------------------------------ commands

    /// Run a ribbon command (also what the tests click).
    pub fn command(&mut self, id: &str, project: &mut Project, dialogs: &mut dyn Dialogs) {
        let sheet = self.sheet;
        let res: Result<(), String> = match id {
            "new_drawing" => {
                self.dialog = Some(Dialog::NewDrawing(self.new_drawing_form(project)));
                Ok(())
            }
            "new_sheet" => {
                if project.drawings.is_empty() {
                    self.dialog = Some(Dialog::NewDrawing(self.new_drawing_form(project)));
                    Ok(())
                } else {
                    let fmt = project.drawings.sheets[sheet.min(project.drawings.sheets.len() - 1)]
                        .format;
                    project.dw_checkpoint();
                    let n = project.drawings.sheets.len() + 1;
                    let i = project.drawings.add_sheet(&format!("Sheet {n}"), fmt);
                    let title = project.name();
                    project.drawings.sheets[i].title.title = title;
                    project.mark_extras_dirty();
                    self.sheet = i;
                    self.selected = None;
                    Ok(())
                }
            }
            "base_view" => {
                if project.drawings.sheets.get(sheet).is_none() {
                    Err("make a drawing first".into())
                } else {
                    self.dialog = Some(Dialog::BaseView(ViewForm {
                        orientation: Orientation::Front,
                        style: ViewStyle::VisibleHidden,
                        assembly: project.assembly.is_some(),
                        scale: None,
                    }));
                    Ok(())
                }
            }
            "proj_right" => self.add_selected_projection(project, Side::Right),
            "proj_left" => self.add_selected_projection(project, Side::Left),
            "proj_above" => self.add_selected_projection(project, Side::Above),
            "proj_below" => self.add_selected_projection(project, Side::Below),
            "section_view" => self.start_tool(project, Tool::Section { first: None }),
            "detail_view" => self.start_tool(project, Tool::Detail { first: None }),
            "delete_view" => self.delete_selected(project),
            "update" => self.update_all(project),
            "export_pdf" => self.export_pdf(project, dialogs),
            "export_dxf" => self.export_dxf(project, dialogs),
            other => Err(format!("unknown drawing command {other}")),
        };
        if let Err(e) = res {
            self.status = e;
        }
    }

    /// Make a drawing (a new sheet with a front view of the source) from the dialog's form.
    pub fn new_drawing(&mut self, project: &mut Project, f: &NewDrawing) -> Result<(), String> {
        let source = if f.assembly {
            ViewSource::Assembly
        } else {
            ViewSource::Design
        };
        // fail early, before anything is added, when there is nothing to draw
        source_for(project, &source)?;
        let fmt = match f.standard {
            Standard::Iso => SheetFormat::iso(f.size),
            Standard::Asme => SheetFormat::asme(f.size),
        };
        project.dw_checkpoint();
        let n = project.drawings.sheets.len() + 1;
        let i = project.drawings.add_sheet(&format!("Sheet {n}"), fmt);
        {
            let t = &mut project.drawings.sheets[i].title;
            t.title = f.title.clone();
            t.number = f.number.clone();
            t.author = f.author.clone();
        }
        project.mark_extras_dirty();
        self.sheet = i;
        self.fit_for = None;
        self.add_base_view(
            project,
            &ViewForm {
                orientation: Orientation::Front,
                style: ViewStyle::VisibleHidden,
                assembly: f.assembly,
                scale: None,
            },
            false,
        )?;
        Ok(())
    }

    /// Add a base view to the shown sheet. With `checkpoint` the change is a step of its own for Undo.
    pub fn add_base_view(
        &mut self,
        project: &mut Project,
        f: &ViewForm,
        checkpoint: bool,
    ) -> Result<u64, String> {
        let sheet = self.sheet;
        let fmt = project
            .drawings
            .sheets
            .get(sheet)
            .ok_or("make a drawing first")?
            .format;
        let which = if f.assembly {
            ViewSource::Assembly
        } else {
            ViewSource::Design
        };
        let src = source_for(project, &which)?;
        let frame = ViewFrame::of(f.orientation);
        let ext = extent(&src.brep, &src.bodies, &frame);
        let fr = fmt.frame();
        let (lo, hi) = fr.border;
        let (w, h) = (hi[0] - lo[0], hi[1] - lo[1]);
        let scale = match f.scale {
            Some(i) => kw_drawing::update::STANDARD_SCALES[i.min(16)],
            None => auto_scale(ext, [w * 0.4, h * 0.4]),
        };
        if checkpoint {
            project.dw_checkpoint();
        }
        let id = project.drawings.fresh_id();
        let existing = project.drawings.sheets[sheet].views.len();
        // the first view goes to the left of the middle, leaving room for the others
        let pos = [
            lo[0] + w * (0.3 + 0.05 * existing as f64),
            lo[1] + h * (0.55 - 0.05 * existing as f64),
        ];
        project.drawings.sheets[sheet].views.push(View {
            id,
            kind: ViewKind::Base,
            source: which,
            orientation: f.orientation,
            scale,
            style: f.style,
            pos,
            label: String::new(),
            cache: None,
        });
        project.mark_extras_dirty();
        self.selected = Some(id);
        self.generate(project, vec![id]);
        Ok(id)
    }

    fn add_selected_projection(&mut self, project: &mut Project, side: Side) -> Result<(), String> {
        let id = self.selected.ok_or("select the view to project from")?;
        self.add_projected(project, id, side).map(|_| ())
    }

    /// A view projected from `parent`, placed on `side` of it (by the sheet's projection angle).
    pub fn add_projected(
        &mut self,
        project: &mut Project,
        parent: u64,
        side: Side,
    ) -> Result<u64, String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(parent).is_some())
            .ok_or("that view does not exist")?;
        let angle = project.drawings.sheets[sheet].format.projection;
        let p = project.drawings.sheets[sheet].view(parent).unwrap().clone();
        let frame = ViewFrame::of(p.orientation).neighbour(side, angle);
        let src = source_for(project, &p.source)?;
        let child_ext = extent(&src.brep, &src.bodies, &frame);
        let child_size = [child_ext[0] * p.scale, child_ext[1] * p.scale];
        let parent_size = {
            let s = p.size();
            if s[0] > 0.0 {
                s
            } else {
                let e = extent(&src.brep, &src.bodies, &ViewFrame::of(p.orientation));
                [e[0] * p.scale, e[1] * p.scale]
            }
        };
        let off = neighbour_offset(side, parent_size, child_size, GAP);
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[sheet].views.push(View {
            id,
            kind: ViewKind::Projected {
                parent,
                align: match side {
                    Side::Left | Side::Right => Align::Horizontal,
                    Side::Above | Side::Below => Align::Vertical,
                },
            },
            source: p.source.clone(),
            orientation: frame.orientation(),
            scale: p.scale,
            style: p.style,
            pos: [p.pos[0] + off[0], p.pos[1] + off[1]],
            label: String::new(),
            cache: None,
        });
        project.mark_extras_dirty();
        self.sheet = sheet;
        self.selected = Some(id);
        self.generate(project, vec![id]);
        Ok(id)
    }

    fn start_tool(&mut self, project: &Project, tool: Tool) -> Result<(), String> {
        let n = self.current_sheet(project).map_or(0, |s| s.views.len());
        if n == 0 {
            return Err("place a base view first".into());
        }
        self.status = match &tool {
            Tool::Section { .. } => {
                "Section: click the view, then click the end of the cutting line (Esc cancels)"
            }
            Tool::Detail { .. } => {
                "Detail: click the centre, then a point on the circle's edge (Esc cancels)"
            }
        }
        .into();
        self.tool = Some(tool);
        Ok(())
    }

    /// The first unused section / detail letter on the sheet (A, B, C...; I, O and Q are skipped).
    fn next_letter(sheet: &kw_drawing::Sheet) -> String {
        let used: Vec<&str> = sheet
            .views
            .iter()
            .filter_map(|v| match &v.kind {
                ViewKind::Section { letter, .. } | ViewKind::Detail { letter, .. } => {
                    Some(letter.as_str())
                }
                _ => None,
            })
            .collect();
        ('A'..='Z')
            .filter(|c| !"IOQ".contains(*c))
            .map(|c| c.to_string())
            .find(|l| !used.contains(&l.as_str()))
            .unwrap_or_else(|| "?".into())
    }

    /// A section of `parent` along the line `a`..`b` (the parent's view coordinates, model units),
    /// looking toward the material to the right of `a`→`b` (to the left when `flip`). The view is
    /// placed beside the parent in the direction of the arrows.
    pub fn add_section(
        &mut self,
        project: &mut Project,
        parent: u64,
        a: [f64; 2],
        b: [f64; 2],
        flip: bool,
    ) -> Result<u64, String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(parent).is_some())
            .ok_or("that view does not exist")?;
        let p = project.drawings.sheets[sheet].view(parent).unwrap().clone();
        if matches!(p.kind, ViewKind::Detail { .. }) {
            return Err("a section of a detail view is not available".into());
        }
        let pf = ViewFrame::of(p.orientation);
        let (frame, p0) = section_frame(&pf, a, b, flip)?;
        let src = source_for(project, &p.source)?;
        // fail now, not in the background, when the line misses the model
        slice_loops(&src.brep, &src.bodies, p0, frame.dir)?;
        let (_, n) = cut_directions(a, b, flip).ok_or("the cutting line has no length")?;
        let side = if n[0].abs() >= n[1].abs() {
            if n[0] >= 0.0 {
                Side::Right
            } else {
                Side::Left
            }
        } else if n[1] >= 0.0 {
            Side::Above
        } else {
            Side::Below
        };
        let e = extent(&src.brep, &src.bodies, &frame);
        let parent_size = {
            let s = p.size();
            if s[0] > 0.0 {
                s
            } else {
                [e[0] * p.scale, e[1] * p.scale]
            }
        };
        let off = neighbour_offset(side, parent_size, [e[0] * p.scale, e[1] * p.scale], GAP);
        // beside the parent, and further on along the arrows while another view is in the way
        let child = [e[0] * p.scale, e[1] * p.scale];
        let step = match side {
            Side::Left | Side::Right => child[0] + GAP,
            Side::Above | Side::Below => child[1] + GAP,
        };
        let dir = [
            off[0].signum() * (off[0] != 0.0) as u8 as f64,
            off[1].signum() * (off[1] != 0.0) as u8 as f64,
        ];
        let mut pos = [p.pos[0] + off[0], p.pos[1] + off[1]];
        for _ in 0..6 {
            let clash = project.drawings.sheets[sheet].views.iter().any(|o| {
                let os = o.size();
                os[0] > 0.0
                    && (o.pos[0] - pos[0]).abs() < (os[0] + child[0]) / 2.0 + 4.0
                    && (o.pos[1] - pos[1]).abs() < (os[1] + child[1]) / 2.0 + 4.0
            });
            if !clash {
                break;
            }
            pos = [pos[0] + dir[0] * step, pos[1] + dir[1] * step];
        }
        let letter = Self::next_letter(&project.drawings.sheets[sheet]);
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[sheet].views.push(View {
            id,
            kind: ViewKind::Section {
                parent,
                a,
                b,
                flip,
                align: match side {
                    Side::Left | Side::Right => Align::Horizontal,
                    Side::Above | Side::Below => Align::Vertical,
                },
                letter,
            },
            source: p.source.clone(),
            orientation: frame.orientation(),
            scale: p.scale,
            style: p.style,
            pos,
            label: String::new(),
            cache: None,
        });
        project.mark_extras_dirty();
        self.sheet = sheet;
        self.selected = Some(id);
        self.generate(project, vec![id]);
        Ok(id)
    }

    /// An enlarged view of the circle `center`, `radius` (the parent's view coordinates, model
    /// units) of `parent`. `scale` is the enlargement; by default one that makes the circle about
    /// 70 mm across.
    pub fn add_detail(
        &mut self,
        project: &mut Project,
        parent: u64,
        center: [f64; 2],
        radius: f64,
        scale: Option<f64>,
    ) -> Result<u64, String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(parent).is_some())
            .ok_or("that view does not exist")?;
        let p = project.drawings.sheets[sheet].view(parent).unwrap().clone();
        if matches!(p.kind, ViewKind::Section { .. }) {
            return Err("a detail of a section view is not available yet".into());
        }
        if radius <= 1e-9 {
            return Err("the detail circle has no size".into());
        }
        let scale = scale.unwrap_or_else(|| {
            let ideal = 35.0 / radius;
            STANDARD_SCALES
                .iter()
                .copied()
                .filter(|s| *s <= ideal)
                .fold(p.scale * 2.0, f64::max)
        });
        let r = radius * scale;
        let fr = project.drawings.sheets[sheet].format.frame();
        // the free spot nearest the parent that is inside the border, clear of the title block
        // and of the other views
        let taken: Vec<([f64; 2], [f64; 2])> = project.drawings.sheets[sheet]
            .views
            .iter()
            .filter_map(|v| {
                let s = v.size();
                (s[0] > 0.0).then_some((
                    [v.pos[0] - s[0] / 2.0 - 8.0, v.pos[1] - s[1] / 2.0 - 12.0],
                    [v.pos[0] + s[0] / 2.0 + 8.0, v.pos[1] + s[1] / 2.0 + 8.0],
                ))
            })
            .chain(std::iter::once((
                [fr.title.0[0] - 5.0, fr.title.0[1] - 5.0],
                [fr.title.1[0] + 5.0, fr.title.1[1] + 12.0],
            )))
            .collect();
        let free = |c: [f64; 2]| {
            c[0] - r >= fr.border.0[0] + 5.0
                && c[0] + r <= fr.border.1[0] - 5.0
                && c[1] - r >= fr.border.0[1] + 12.0
                && c[1] + r <= fr.border.1[1] - 8.0
                && taken.iter().all(|(lo, hi)| {
                    c[0] + r < lo[0] || c[0] - r > hi[0] || c[1] + r < lo[1] || c[1] - r > hi[1]
                })
        };
        let mut best: Option<([f64; 2], f64)> = None;
        let mut gx = fr.border.0[0];
        while gx <= fr.border.1[0] {
            let mut gy = fr.border.0[1];
            while gy <= fr.border.1[1] {
                let c = [gx, gy];
                let dist = (c[0] - p.pos[0]).hypot(c[1] - p.pos[1]);
                if free(c) && best.is_none_or(|b| dist < b.1) {
                    best = Some((c, dist));
                }
                gy += 5.0;
            }
            gx += 5.0;
        }
        let [x, y] = best.map_or([fr.border.1[0] - r - 5.0, fr.border.1[1] - r - 8.0], |b| {
            b.0
        });
        let letter = Self::next_letter(&project.drawings.sheets[sheet]);
        project.dw_checkpoint();
        let id = project.drawings.fresh_id();
        project.drawings.sheets[sheet].views.push(View {
            id,
            kind: ViewKind::Detail {
                parent,
                center,
                radius,
                letter,
            },
            source: p.source.clone(),
            orientation: p.orientation,
            scale,
            style: p.style,
            pos: [x, y],
            label: String::new(),
            cache: None,
        });
        project.mark_extras_dirty();
        self.sheet = sheet;
        self.selected = Some(id);
        self.generate(project, vec![id]);
        Ok(id)
    }

    /// Turn a section around: it looks at the other side of the cutting line.
    pub fn flip_section(&mut self, project: &mut Project, id: u64) -> Result<(), String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(id).is_some())
            .ok_or("that view does not exist")?;
        let v = project.drawings.sheets[sheet].view(id).unwrap().clone();
        let ViewKind::Section {
            parent, a, b, flip, ..
        } = v.kind
        else {
            return Err("that is not a section view".into());
        };
        let p = project.drawings.sheets[sheet]
            .view(parent)
            .ok_or("the parent view is gone")?
            .clone();
        let (frame, p0) = section_frame(&ViewFrame::of(p.orientation), a, b, !flip)?;
        let src = source_for(project, &v.source)?;
        slice_loops(&src.brep, &src.bodies, p0, frame.dir)?;
        project.dw_checkpoint();
        if let Some(vm) = project.drawings.sheets[sheet].view_mut(id) {
            if let ViewKind::Section { flip, .. } = &mut vm.kind {
                *flip = !*flip;
            }
            vm.orientation = frame.orientation();
        }
        project.mark_extras_dirty();
        self.generate(project, vec![id]);
        Ok(())
    }

    fn delete_selected(&mut self, project: &mut Project) -> Result<(), String> {
        let id = self.selected.ok_or("select a view to delete")?;
        self.delete_view(project, id)
    }

    /// Delete a view and the views made from it.
    pub fn delete_view(&mut self, project: &mut Project, id: u64) -> Result<(), String> {
        let sheet = project
            .drawings
            .sheets
            .iter()
            .position(|s| s.view(id).is_some())
            .ok_or("that view does not exist")?;
        project.dw_checkpoint();
        let views = &mut project.drawings.sheets[sheet].views;
        let mut gone = vec![id];
        loop {
            let more: Vec<u64> = views
                .iter()
                .filter(|v| v.parent().is_some_and(|p| gone.contains(&p)) && !gone.contains(&v.id))
                .map(|v| v.id)
                .collect();
            if more.is_empty() {
                break;
            }
            gone.extend(more);
        }
        views.retain(|v| !gone.contains(&v.id));
        project.mark_extras_dirty();
        if self.selected.is_some_and(|s| gone.contains(&s)) {
            self.selected = None;
        }
        Ok(())
    }

    pub fn delete_sheet(&mut self, project: &mut Project, index: usize) {
        if index < project.drawings.sheets.len() {
            project.dw_checkpoint();
            project.drawings.sheets.remove(index);
            project.mark_extras_dirty();
            self.sheet = self
                .sheet
                .min(project.drawings.sheets.len().saturating_sub(1));
            self.selected = None;
        }
    }

    /// Make every view of every sheet again from the model as it is now.
    pub fn update_all(&mut self, project: &mut Project) -> Result<(), String> {
        let ids: Vec<u64> = project
            .drawings
            .sheets
            .iter()
            .flat_map(|s| s.views.iter().map(|v| v.id))
            .collect();
        if ids.is_empty() {
            return Err("there is nothing to update".into());
        }
        self.generate(project, ids);
        Ok(())
    }

    // ------------------------------------------------------------------------------ generation

    fn specs(project: &Project, ids: &[u64]) -> Vec<(ViewSource, GenSpec)> {
        let mut out = Vec::new();
        for id in ids {
            for s in &project.drawings.sheets {
                let Some(v) = s.view(*id) else { continue };
                let kind = match &v.kind {
                    ViewKind::Base | ViewKind::Projected { .. } => GenKind::Plain,
                    ViewKind::Section {
                        parent, a, b, flip, ..
                    } => {
                        let Some(p) = s.view(*parent) else { continue };
                        GenKind::Section {
                            parent: ViewFrame::of(p.orientation),
                            a: *a,
                            b: *b,
                            flip: *flip,
                        }
                    }
                    ViewKind::Detail { center, radius, .. } => GenKind::Detail {
                        center: *center,
                        radius: *radius,
                    },
                };
                out.push((
                    v.source.clone(),
                    GenSpec {
                        id: v.id,
                        frame: ViewFrame::of(v.orientation),
                        scale: v.scale,
                        style: v.style,
                        kind,
                    },
                ));
            }
        }
        out
    }

    /// Start making the geometry of these views on a background thread (they are queued while another
    /// batch is running).
    pub fn generate(&mut self, project: &mut Project, ids: Vec<u64>) {
        if self.job.is_some() {
            for i in ids {
                if !self.queue.contains(&i) {
                    self.queue.push(i);
                }
            }
            return;
        }
        match Self::prepare(project, &ids) {
            Ok((src, specs)) => {
                self.status = "Generating views...".into();
                self.job = Some(Job::spawn("drawing views", move || {
                    let g = generate(&src.brep, &src.bodies, &specs)?;
                    Ok((g, src.hash))
                }));
            }
            Err(e) => self.status = e,
        }
    }

    /// The source and specs of views, grouped by the source of the first view (views of one batch share
    /// a source in practice; the others are made in a later batch).
    fn prepare(project: &mut Project, ids: &[u64]) -> Result<(Source, Vec<GenSpec>), String> {
        let specs = Self::specs(project, ids);
        let Some((first, _)) = specs.first().cloned() else {
            return Err("there is nothing to generate".into());
        };
        let src = source_for(project, &first)?;
        let batch: Vec<GenSpec> = specs
            .into_iter()
            .filter(|(s, _)| *s == first)
            .map(|(_, g)| g)
            .collect();
        Ok((src, batch))
    }

    /// Store the geometry of a finished batch in the views.
    fn apply(&mut self, project: &mut Project, done: Generated) {
        let (geoms, hash) = done;
        for (id, g) in geoms {
            for s in &mut project.drawings.sheets {
                if let Some(v) = s.view_mut(id) {
                    v.cache = Some(ViewCache {
                        source_hash: hash,
                        scene: g.scene.clone(),
                        centre: g.centre,
                    });
                }
            }
        }
        project.mark_extras_dirty();
    }

    /// Check on the background job; start the next batch if views are waiting.
    pub fn poll(&mut self, project: &mut Project) {
        let Some(job) = &self.job else { return };
        let Some(res) = job.poll() else { return };
        self.job = None;
        match res {
            Ok(done) => {
                self.apply(project, done);
                self.status.clear();
            }
            Err(e) => self.status = format!("could not generate the views: {e}"),
        }
        if self.job.is_none() && !self.queue.is_empty() {
            let q = std::mem::take(&mut self.queue);
            self.generate(project, q);
        }
    }

    /// Generate views now and wait for them (tests and demos).
    pub fn generate_now(&mut self, project: &mut Project, ids: &[u64]) -> Result<(), String> {
        let mut todo: Vec<u64> = ids.to_vec();
        while !todo.is_empty() {
            let (src, batch) = Self::prepare(project, &todo)?;
            let done: Vec<u64> = batch.iter().map(|g| g.id).collect();
            let g = generate(&src.brep, &src.bodies, &batch)?;
            self.apply(project, (g, src.hash));
            todo.retain(|i| !done.contains(i));
        }
        Ok(())
    }

    /// Wait for the running job and everything queued behind it (tests).
    pub fn wait(&mut self, project: &mut Project) {
        while self.busy() {
            self.poll(project);
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    // ------------------------------------------------------------------------------ output

    fn export_pdf(
        &mut self,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) -> Result<(), String> {
        if project.drawings.is_empty() {
            return Err("there is nothing to export".into());
        }
        let name = project.name();
        let Some(path) = dialogs.save_file(&format!("{name}.pdf"), "PDF", &["pdf"]) else {
            return Ok(());
        };
        std::fs::write(&path, kw_drawing::set_to_pdf(&name, &project.drawings))
            .map_err(|e| format!("could not write {}: {e}", path.display()))?;
        self.status = format!("saved {}", path.display());
        Ok(())
    }

    fn export_dxf(
        &mut self,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) -> Result<(), String> {
        let text = kw_drawing::sheet_to_dxf(&project.drawings, self.sheet)
            .ok_or("there is no sheet to export")?;
        let name = project.name();
        let Some(path) = dialogs.save_file(&format!("{name}.dxf"), "DXF", &["dxf"]) else {
            return Ok(());
        };
        std::fs::write(&path, text)
            .map_err(|e| format!("could not write {}: {e}", path.display()))?;
        self.status = format!("saved {}", path.display());
        Ok(())
    }

    /// Run the commands the ribbon asked for.
    fn run_requests(&mut self, project: &mut Project, dialogs: &mut dyn Dialogs) {
        for id in std::mem::take(&mut self.requests) {
            self.command(id, project, dialogs);
        }
    }

    /// The whole workspace: panels, canvas and dialogs.
    pub fn show(
        &mut self,
        ui: &mut eframe::egui::Ui,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) {
        use eframe::egui;
        self.poll(project);
        self.run_requests(project, dialogs);
        if self.busy() {
            ui.ctx().request_repaint();
        }
        self.sheet = self
            .sheet
            .min(project.drawings.sheets.len().saturating_sub(1));
        egui::Panel::left("drawing_browser")
            .default_size(220.0)
            .show(ui, |ui| self.browser_ui(ui, project));
        egui::Panel::right("drawing_props")
            .default_size(250.0)
            .show(ui, |ui| self.props_ui(ui, project));
        egui::Panel::bottom("drawing_sheets").show(ui, |ui| self.tabs_ui(ui, project));
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| self.canvas_ui(ui, project));
        self.dialogs_ui(ui.ctx(), project);
        self.run_requests(project, dialogs);
    }

    /// The ribbon of the Drawing workspace.
    pub fn ribbon(&mut self, ui: &mut eframe::egui::Ui, project: &Project) {
        use crate::chrome::{ribbon as rb, Icon};
        let has_sheet = !project.drawings.is_empty();
        let has_sel = self.selected.is_some();
        let mut req: Vec<&'static str> = Vec::new();
        ui.horizontal(|ui| {
            rb::group(ui, "SHEET", |ui| {
                if rb::button(
                    ui,
                    Icon::Drawing,
                    "New Drawing",
                    true,
                    false,
                    "Make a drawing of the design on a new sheet",
                )
                .clicked()
                {
                    req.push("new_drawing");
                }
                if rb::button(
                    ui,
                    Icon::Plane,
                    "New Sheet",
                    has_sheet,
                    false,
                    "Add another sheet like this one",
                )
                .clicked()
                {
                    req.push("new_sheet");
                }
            });
            rb::group(ui, "VIEWS", |ui| {
                if rb::button(
                    ui,
                    Icon::BaseView,
                    "Base View",
                    has_sheet,
                    false,
                    "Place a view of the model on the sheet",
                )
                .clicked()
                {
                    req.push("base_view");
                }
                if rb::button(
                    ui,
                    Icon::Section,
                    "Section View",
                    has_sheet,
                    matches!(self.tool, Some(Tool::Section { .. })),
                    "Cut the model along a line and show the cut face hatched",
                )
                .clicked()
                {
                    req.push("section_view");
                }
                if rb::button(
                    ui,
                    Icon::Circle,
                    "Detail View",
                    has_sheet,
                    matches!(self.tool, Some(Tool::Detail { .. })),
                    "Enlarge a circular part of a view",
                )
                .clicked()
                {
                    req.push("detail_view");
                }
                for (id, tip, label) in [
                    (
                        "proj_right",
                        "Projected view to the right of the selected view",
                        "Project >",
                    ),
                    (
                        "proj_left",
                        "Projected view to the left of the selected view",
                        "< Project",
                    ),
                    (
                        "proj_above",
                        "Projected view above the selected view",
                        "Project ^",
                    ),
                    (
                        "proj_below",
                        "Projected view below the selected view",
                        "Project v",
                    ),
                ] {
                    if rb::button(ui, Icon::ProjView, label, has_sel, false, tip).clicked() {
                        req.push(id);
                    }
                }
            });
            rb::group(ui, "MODIFY", |ui| {
                if rb::button(
                    ui,
                    Icon::Finish,
                    "Update",
                    has_sheet && !self.busy(),
                    self.busy(),
                    "Make the views again from the model as it is now",
                )
                .clicked()
                {
                    req.push("update");
                }
                if rb::button(
                    ui,
                    Icon::Trim,
                    "Delete View",
                    has_sel,
                    false,
                    "Delete the selected view and the views made from it",
                )
                .clicked()
                {
                    req.push("delete_view");
                }
            });
            rb::group(ui, "OUTPUT", |ui| {
                if rb::button(
                    ui,
                    Icon::Folder,
                    "PDF",
                    has_sheet,
                    false,
                    "Save every sheet as a PDF",
                )
                .clicked()
                {
                    req.push("export_pdf");
                }
                if rb::button(
                    ui,
                    Icon::Dxf,
                    "DXF",
                    has_sheet,
                    false,
                    "Save the shown sheet as a DXF",
                )
                .clicked()
                {
                    req.push("export_dxf");
                }
            });
        });
        self.requests.extend(req);
    }
}
