//! A 3-D viewport inside egui: the viewer's renderer draws into an offscreen texture that egui shows as an
//! image. The part that needs only a wgpu device ([`GpuView`]) is separate from the egui glue, so it can be
//! tested without a window.

use eframe::egui;
use kw_brep::{BRep, BodyId, EdgeId, FaceId};
use kw_tess::{tessellate_body, TessOptions, Tessellation};
use kw_viewer::renderer::{Renderer, DEPTH_FORMAT};
use kw_viewer::scene::{PickTarget, Scene, SceneObject};
use kw_viewer::OrbitCamera;
use nalgebra::Isometry3;
use std::sync::Arc;

const _: wgpu::TextureFormat = DEPTH_FORMAT;

/// Offscreen colour target (sampled by egui as `Rgba8Unorm`, drawn through an sRGB view) and the renderer.
pub struct GpuView {
    pub renderer: Renderer,
    color: wgpu::Texture,
    srgb: wgpu::TextureView,
    pub plain: wgpu::TextureView,
    size: (u32, u32),
}

fn make_color(
    device: &wgpu::Device,
    size: (u32, u32),
) -> (wgpu::Texture, wgpu::TextureView, wgpu::TextureView) {
    let color = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("viewport_color"),
        size: wgpu::Extent3d {
            width: size.0,
            height: size.1,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[wgpu::TextureFormat::Rgba8UnormSrgb],
    });
    let srgb = color.create_view(&wgpu::TextureViewDescriptor {
        format: Some(wgpu::TextureFormat::Rgba8UnormSrgb),
        ..Default::default()
    });
    let plain = color.create_view(&wgpu::TextureViewDescriptor::default());
    (color, srgb, plain)
}

impl GpuView {
    pub fn new(device: &wgpu::Device, size: (u32, u32)) -> GpuView {
        let size = (size.0.max(1), size.1.max(1));
        let (color, srgb, plain) = make_color(device, size);
        GpuView {
            renderer: Renderer::new(device, wgpu::TextureFormat::Rgba8UnormSrgb, size),
            color,
            srgb,
            plain,
            size,
        }
    }

    pub fn size(&self) -> (u32, u32) {
        self.size
    }

    /// Returns true when the colour texture was replaced (egui must be told).
    pub fn resize(&mut self, device: &wgpu::Device, size: (u32, u32)) -> bool {
        let size = (size.0.max(1), size.1.max(1));
        if size == self.size {
            return false;
        }
        let (color, srgb, plain) = make_color(device, size);
        self.color = color;
        self.srgb = srgb;
        self.plain = plain;
        self.renderer.resize(device, size);
        self.size = size;
        true
    }

    pub fn render(&self, device: &wgpu::Device, queue: &wgpu::Queue, cam: &OrbitCamera) {
        self.renderer.render(device, queue, cam, &self.srgb);
    }

    /// The rendered image as RGBA bytes (sRGB encoded), for tests and screenshots.
    pub fn read_rgba(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> Vec<u8> {
        kw_viewer::renderer::read_texture(
            device,
            queue,
            &self.color,
            0,
            0,
            self.size.0,
            self.size.1,
            4,
        )
    }

    /// The pick id under pixel `(x, y)` (0 = nothing).
    pub fn pick_id(&self, device: &wgpu::Device, queue: &wgpu::Queue, x: u32, y: u32) -> u32 {
        if x >= self.size.0 || y >= self.size.1 {
            return 0;
        }
        self.renderer
            .read_ids(device, queue, x, y, 1, 1)
            .first()
            .copied()
            .unwrap_or(0)
    }
}

impl GpuView {
    /// What is at pixel `(x, y)`, preferring an edge within `edge_px` pixels: edges are drawn only a pixel or
    /// two wide, so picking edges needs some reach. With `edge_px == 0` this is the single pixel.
    pub fn pick_near(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        model: &ModelScene,
        x: u32,
        y: u32,
        edge_px: u32,
    ) -> Option<Pick> {
        let centre = model.resolve(self.pick_id(device, queue, x, y));
        if edge_px == 0 || x >= self.size.0 || y >= self.size.1 {
            return centre;
        }
        let x0 = x.saturating_sub(edge_px);
        let y0 = y.saturating_sub(edge_px);
        let x1 = (x + edge_px).min(self.size.0 - 1);
        let y1 = (y + edge_px).min(self.size.1 - 1);
        let (w, h) = (x1 - x0 + 1, y1 - y0 + 1);
        let ids = self.renderer.read_ids(device, queue, x0, y0, w, h);
        let r2 = (edge_px * edge_px) as i64;
        let mut best: Option<(i64, Pick)> = None;
        for (k, id) in ids.iter().enumerate() {
            let (px, py) = (x0 + k as u32 % w, y0 + k as u32 / w);
            let (dx, dy) = (px as i64 - x as i64, py as i64 - y as i64);
            let d2 = dx * dx + dy * dy;
            if d2 > r2 || *id == 0 {
                continue;
            }
            if let Some(Pick::Edge(e)) = model.resolve(*id) {
                if best.is_none_or(|(bd, _)| d2 < bd) {
                    best = Some((d2, Pick::Edge(e)));
                }
            }
        }
        best.map(|(_, p)| p).or(centre)
    }
}

/// What a click in the viewport hit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pick {
    Face(FaceId),
    Edge(EdgeId),
}

/// A seam is the edge where a periodic face (a cylinder, a sphere) meets itself: it is a cut in the
/// parametrisation, not a feature of the shape, so it is not drawn (nor picked).
pub fn is_seam(brep: &BRep, edge: EdgeId) -> bool {
    let ring = brep.radial_ring(edge);
    let faces: Vec<FaceId> = ring
        .iter()
        .filter_map(|c| brep.coedges.get(*c))
        .filter_map(|c| brep.loops.get(c.owner_loop))
        .map(|l| l.face)
        .collect();
    faces.len() >= 2 && faces.iter().all(|f| *f == faces[0])
}

fn hide_seams(brep: &BRep, t: &mut Tessellation) {
    let keys = t.edge_keys.clone();
    t.edges
        .ranges
        .retain(|(ei, _, _)| keys.get(*ei as usize).is_none_or(|e| !is_seam(brep, *e)));
}

/// The model as drawn: tessellations of the bodies, their scene, and the lookup from pick ids to B-rep ids.
pub struct ModelScene {
    pub scene: Scene,
    tess: Vec<Tessellation>,
    /// One curvature value per vertex of `scene` when the scene was built for a curvature analysis
    /// (`NaN` where it cannot be found), else empty.
    pub scalars: Vec<f32>,
    /// The free edges of surfaces (sheet bodies), as segments in scene coordinates.
    pub free_edges: Vec<([f64; 3], [f64; 3])>,
}

/// What the analysis overlay shows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnalysisKind {
    Zebra,
    Curvature(kw_analysis::CurvKind),
    Draft,
}

/// The analysis overlay of the viewport: how to colour the faces.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalysisView {
    pub kind: AnalysisKind,
    /// Choose the curvature range from the model (else `limit` is used).
    pub auto: bool,
    /// Curvature shown at the ends of the ramp (1/mm), both signs.
    pub limit: f32,
    /// Zebra stripe axis or draft pull direction (world).
    pub axis: [f64; 3],
    pub zebra_freq: f32,
    /// Draft below this angle (degrees) is flagged.
    pub draft_limit: f32,
}

impl ModelScene {
    /// A scene made of arbitrary geometry (no B-rep behind it, so nothing can be picked).
    pub fn custom(scene: Scene) -> ModelScene {
        ModelScene {
            scene,
            tess: Vec::new(),
            scalars: Vec::new(),
            free_edges: Vec::new(),
        }
    }

    pub fn build(brep: &BRep) -> ModelScene {
        ModelScene::build_coloured(brep, &[])
    }

    /// As [`ModelScene::build`], with the colour of body `i` (in the B-rep's body order) from `colours`.
    pub fn build_coloured(brep: &BRep, colours: &[[u8; 3]]) -> ModelScene {
        ModelScene::build_analysed(brep, colours, None)
    }

    /// As [`ModelScene::build_coloured`], also finding the chosen curvature at every vertex.
    pub fn build_analysed(
        brep: &BRep,
        colours: &[[u8; 3]],
        curvature: Option<kw_analysis::CurvKind>,
    ) -> ModelScene {
        let bodies: Vec<BodyId> = brep.bodies.keys().collect();
        let opts = TessOptions {
            chord_tol: 0.02,
            angle_tol: 0.2,
        };
        let tess_c: Vec<(Tessellation, [u8; 3])> = bodies
            .iter()
            .enumerate()
            .filter_map(|(i, b)| {
                tessellate_body(brep, *b, &opts).ok().map(|mut t| {
                    hide_seams(brep, &mut t);
                    (t, colours.get(i).copied().unwrap_or([196, 200, 208]))
                })
            })
            .collect();
        let objects: Vec<SceneObject> = tess_c
            .iter()
            .map(|(t, c)| SceneObject {
                tess: t,
                transform: Isometry3::identity(),
                color: *c,
            })
            .collect();
        let mut scene = Scene::build(&objects);
        let tess: Vec<Tessellation> = tess_c.into_iter().map(|(t, _)| t).collect();
        if tess.is_empty() {
            scene.center = nalgebra::Vector3::zeros();
            scene.radius = 40.0;
        }
        // the open boundary of every surface, for the overlay that marks where it is not yet joined
        let mut free_edges = Vec::new();
        for (t, body) in tess.iter().zip(bodies) {
            let free: std::collections::HashSet<_> = brep.free_edges(body).into_iter().collect();
            if free.is_empty() {
                continue;
            }
            for (ei, first, count) in &t.edges.ranges {
                if !t
                    .edge_keys
                    .get(*ei as usize)
                    .is_some_and(|e| free.contains(e))
                {
                    continue;
                }
                let pts = &t.edges.points[*first as usize..(*first + *count) as usize];
                for w in pts.windows(2) {
                    let (a, b) = (w[0].coords - scene.center, w[1].coords - scene.center);
                    free_edges.push(([a.x, a.y, a.z], [b.x, b.y, b.z]));
                }
            }
        }
        let scalars = match curvature {
            Some(kind) => tess
                .iter()
                .flat_map(|t| kw_analysis::mesh_curvature(brep, t, kind))
                .collect(),
            None => Vec::new(),
        };
        ModelScene {
            scene,
            tess,
            scalars,
            free_edges,
        }
    }

    /// The range most curvature values lie in (the 2nd to the 98th percentile of the finite ones), so
    /// a few extreme vertices do not wash out the colours.
    pub fn scalar_range(&self) -> Option<(f32, f32)> {
        let mut v: Vec<f32> = self
            .scalars
            .iter()
            .copied()
            .filter(|x| x.is_finite())
            .collect();
        if v.is_empty() {
            return None;
        }
        v.sort_by(|a, b| a.total_cmp(b));
        let at = |q: f64| v[((v.len() - 1) as f64 * q).round() as usize];
        Some((at(0.02), at(0.98)))
    }

    pub fn empty() -> ModelScene {
        ModelScene::build(&BRep::new())
    }

    /// The dense face index of `face` in the first body's tessellation (what mesh tags are).
    pub fn face_tag(&self, face: FaceId) -> Option<u32> {
        self.tess
            .first()?
            .face_keys
            .iter()
            .position(|f| *f == face)
            .map(|i| i as u32)
    }

    pub fn resolve(&self, id: u32) -> Option<Pick> {
        match self.scene.pick.get(id.checked_sub(1)? as usize)? {
            PickTarget::Face { object, face } => self
                .tess
                .get(*object as usize)?
                .face_keys
                .get(*face as usize)
                .map(|f| Pick::Face(*f)),
            PickTarget::Edge { object, edge } => self
                .tess
                .get(*object as usize)?
                .edge_keys
                .get(*edge as usize)
                .map(|e| Pick::Edge(*e)),
        }
    }

    /// The pick id that shows `pick` highlighted (0 if it is not in the scene).
    pub fn id_of(&self, pick: Pick) -> u32 {
        for (i, t) in self.scene.pick.iter().enumerate() {
            let hit = match (t, pick) {
                (PickTarget::Face { object, face }, Pick::Face(f)) => {
                    self.tess
                        .get(*object as usize)
                        .and_then(|t| t.face_keys.get(*face as usize))
                        == Some(&f)
                }
                (PickTarget::Edge { object, edge }, Pick::Edge(e)) => {
                    self.tess
                        .get(*object as usize)
                        .and_then(|t| t.edge_keys.get(*edge as usize))
                        == Some(&e)
                }
                _ => false,
            };
            if hit {
                return i as u32 + 1;
            }
        }
        0
    }
}

/// Options of one viewport frame.
#[derive(Debug, Clone, Copy)]
pub struct ViewOpts {
    /// Highlight the face or edge under the pointer (needs an id-buffer read when the pointer moves).
    pub hover: bool,
    /// Orbit / pan / zoom with the mouse.
    pub navigate: bool,
}

impl Default for ViewOpts {
    fn default() -> Self {
        ViewOpts {
            hover: true,
            navigate: true,
        }
    }
}

/// What happened in the viewport this frame.
#[derive(Debug, Clone)]
pub struct ViewOut {
    pub rect: egui::Rect,
    /// `Some(None)` = clicked on empty space.
    pub click: Option<Option<Pick>>,
    pub double_click: Option<Option<Pick>>,
    pub hover: Option<Pick>,
    /// Pointer position of a click (screen), when there was one.
    pub click_pos: Option<egui::Pos2>,
    pub response: Option<egui::Response>,
}

impl ViewOut {
    fn none(rect: egui::Rect) -> ViewOut {
        ViewOut {
            rect,
            click: None,
            double_click: None,
            hover: None,
            click_pos: None,
            response: None,
        }
    }
}

/// An animation of the camera direction.
#[derive(Debug, Clone, Copy)]
struct Anim {
    from: (f64, f64),
    to: (f64, f64),
    start: f64,
}

/// The viewport widget state.
pub struct Viewport3d {
    gpu: Option<GpuView>,
    tex: Option<egui::TextureId>,
    pub camera: OrbitCamera,
    /// Draw edges only (to see toolpaths inside the material).
    pub wireframe: bool,
    pub show_edges: bool,
    /// A section plane in world coordinates, `(unit normal, offset)`: what lies on the normal's side is cut.
    pub section: Option<([f64; 3], f64)>,
    /// Colour the faces by zebra stripes, curvature or draft instead of lighting them.
    pub analysis: Option<AnalysisView>,
    /// Picks drawn as selected besides the one passed to `show` (a multi-selection).
    pub extra_selected: Vec<Pick>,
    /// Reach, in pixels, of edge picking and hovering (0 = the exact pixel only).
    pub edge_pick_px: u32,
    model: Option<ModelScene>,
    /// Identifies the geometry currently uploaded.
    key: Option<u128>,
    fitted: bool,
    anim: Option<Anim>,
    prev_view: Option<(f64, f64)>,
    hover: Option<Pick>,
    last_pointer: Option<egui::Pos2>,
    last_edge_px: u32,
    rect: egui::Rect,
    ppp: f32,
}

impl Viewport3d {
    /// The curvature at the ends of the ramp: the model's own range when `auto`, else the chosen limit.
    pub fn curvature_limit(&self) -> f32 {
        Self::limit_of(&self.analysis, self.model.as_ref())
    }

    fn limit_of(a: &Option<AnalysisView>, model: Option<&ModelScene>) -> f32 {
        let Some(a) = a else { return 1.0 };
        if a.auto {
            if let Some((lo, hi)) = model.and_then(|m| m.scalar_range()) {
                return lo.abs().max(hi.abs()).max(1e-6);
            }
        }
        a.limit.max(1e-6)
    }

    fn renderer_analysis(
        analysis: &Option<AnalysisView>,
        model: Option<&ModelScene>,
    ) -> kw_viewer::renderer::Analysis {
        use kw_viewer::renderer::{Analysis, AnalysisMode};
        let Some(a) = analysis else {
            return Analysis::default();
        };
        let axis = [a.axis[0] as f32, a.axis[1] as f32, a.axis[2] as f32];
        match a.kind {
            AnalysisKind::Zebra => Analysis {
                mode: AnalysisMode::Zebra,
                axis,
                zebra_freq: a.zebra_freq,
                duty: 0.5,
                ..Analysis::default()
            },
            AnalysisKind::Curvature(_) => Analysis {
                mode: AnalysisMode::Signed,
                max: Self::limit_of(analysis, model),
                ..Analysis::default()
            },
            AnalysisKind::Draft => Analysis {
                mode: AnalysisMode::Draft,
                axis,
                max: a.draft_limit,
                ..Analysis::default()
            },
        }
    }
}

impl Default for Viewport3d {
    fn default() -> Self {
        Viewport3d {
            gpu: None,
            tex: None,
            camera: OrbitCamera::new(50.0, 1.5),
            wireframe: false,
            show_edges: true,
            section: None,
            analysis: None,
            extra_selected: Vec::new(),
            edge_pick_px: 0,
            model: None,
            key: None,
            fitted: false,
            anim: None,
            prev_view: None,
            hover: None,
            last_pointer: None,
            last_edge_px: 0,
            rect: egui::Rect::NOTHING,
            ppp: 1.0,
        }
    }
}

impl Viewport3d {
    pub fn model(&self) -> Option<&ModelScene> {
        self.model.as_ref()
    }

    pub fn rect(&self) -> egui::Rect {
        self.rect
    }

    /// The hovered pick (updated while the pointer moves over the view).
    pub fn hovered(&self) -> Option<Pick> {
        self.hover
    }

    /// Turn the camera to look from direction (yaw, pitch), animated.
    pub fn look(&mut self, yaw: f64, pitch: f64, now: f64) {
        self.prev_view = Some((self.camera.yaw, self.camera.pitch));
        self.anim = Some(Anim {
            from: (self.camera.yaw, self.camera.pitch),
            to: (yaw, pitch),
            start: now,
        });
    }

    /// Go back to where the camera looked from before the last view change (and again to come back).
    pub fn previous_view(&mut self, now: f64) {
        if let Some((yaw, pitch)) = self.prev_view {
            self.look(yaw, pitch, now);
        }
    }

    /// Frame the whole model again.
    pub fn fit(&mut self) {
        if let Some(m) = &self.model {
            self.camera.target = nalgebra::Vector3::zeros();
            self.camera.fit(m.scene.radius.max(1e-3));
        }
    }

    /// World origin in scene (re-centred) coordinates.
    pub fn origin_in_scene(&self) -> nalgebra::Vector3<f64> {
        self.model
            .as_ref()
            .map(|m| -m.scene.center)
            .unwrap_or_else(nalgebra::Vector3::zeros)
    }

    /// Project a point given in scene coordinates to a screen position (logical points).
    pub fn project(&self, p: [f64; 3]) -> Option<egui::Pos2> {
        let (w, h) = (
            self.rect.width() as f64 * self.ppp as f64,
            self.rect.height() as f64 * self.ppp as f64,
        );
        let q = self
            .camera
            .project(&nalgebra::Vector3::new(p[0], p[1], p[2]), w, h)?;
        Some(egui::pos2(
            self.rect.min.x + q.0 as f32 / self.ppp,
            self.rect.min.y + q.1 as f32 / self.ppp,
        ))
    }

    /// The ray (scene coordinates) through a screen position.
    pub fn ray(&self, pos: egui::Pos2) -> (nalgebra::Vector3<f64>, nalgebra::Vector3<f64>) {
        let (w, h) = (
            self.rect.width() as f64 * self.ppp as f64,
            self.rect.height() as f64 * self.ppp as f64,
        );
        self.camera.pixel_ray(
            (pos.x - self.rect.min.x) as f64 * self.ppp as f64,
            (pos.y - self.rect.min.y) as f64 * self.ppp as f64,
            w,
            h,
        )
    }

    /// Show the viewport. `geometry` is rebuilt only when `key` changes.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        rs: Option<&eframe::egui_wgpu::RenderState>,
        key: u128,
        brep: Option<&Arc<BRep>>,
        highlight: Option<Pick>,
    ) -> Option<Option<Pick>> {
        self.show_with(
            ui,
            rs,
            key,
            || match brep {
                Some(b) => ModelScene::build(b),
                None => ModelScene::empty(),
            },
            highlight,
            ViewOpts::default(),
        )
        .click
    }

    /// Like [`Viewport3d::show`], with the geometry built by `build` whenever `key` changes.
    pub fn show_with(
        &mut self,
        ui: &mut egui::Ui,
        rs: Option<&eframe::egui_wgpu::RenderState>,
        key: u128,
        build: impl FnOnce() -> ModelScene,
        highlight: Option<Pick>,
        opts: ViewOpts,
    ) -> ViewOut {
        let size = ui.available_size().max(egui::vec2(64.0, 64.0));
        let (rect, resp) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
        self.rect = rect;
        let Some(rs) = rs else {
            ui.painter()
                .rect_filled(rect, 0.0, egui::Color32::from_gray(210));
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "3-D view needs a GPU",
                egui::FontId::proportional(16.0),
                egui::Color32::GRAY,
            );
            // geometry is still built so that picks and overlays work without a GPU (tests)
            if self.key != Some(key) {
                let m = build();
                self.camera.update_clip(m.scene.radius.max(1e-3));
                self.model = Some(m);
                self.key = Some(key);
            }
            return ViewOut::none(rect);
        };
        let ppp = ui.ctx().pixels_per_point();
        self.ppp = ppp;
        let px = ((rect.width() * ppp) as u32, (rect.height() * ppp) as u32);
        // ---- targets
        let device = &rs.device;
        let mut registered = false;
        match &mut self.gpu {
            None => {
                self.gpu = Some(GpuView::new(device, px));
                registered = true;
            }
            Some(g) => {
                if g.resize(device, px) {
                    registered = true;
                }
            }
        }
        let gpu = self.gpu.as_mut().expect("gpu view");
        if registered {
            let mut r = rs.renderer.write();
            match self.tex {
                None => {
                    self.tex = Some(r.register_native_texture(
                        device,
                        &gpu.plain,
                        wgpu::FilterMode::Linear,
                    ))
                }
                Some(id) => r.update_egui_texture_from_wgpu_texture(
                    device,
                    &gpu.plain,
                    wgpu::FilterMode::Linear,
                    id,
                ),
            }
        }
        // ---- geometry
        if self.key != Some(key) {
            let m = build();
            gpu.renderer.upload(device, &m.scene);
            if !m.scalars.is_empty() {
                gpu.renderer
                    .upload_scalars(device, &m.scalars, m.scene.vertices.len());
            }
            if !self.fitted && !m.scene.vertices.is_empty() {
                self.camera.target = nalgebra::Vector3::zeros();
                self.camera.fit(m.scene.radius.max(1e-3));
                self.fitted = true;
            }
            self.camera.update_clip(m.scene.radius.max(1e-3));
            self.model = Some(m);
            self.key = Some(key);
            self.hover = None;
        }
        // ---- camera animation
        let now = ui.ctx().input(|i| i.time);
        if let Some(a) = self.anim {
            let t = ((now - a.start) / 0.35).clamp(0.0, 1.0);
            let (yaw, pitch) = crate::chrome::viewcube::lerp_view(a.from, a.to, t);
            self.camera.yaw = yaw;
            self.camera.pitch = pitch;
            if t >= 1.0 {
                self.anim = None;
            } else {
                ui.ctx().request_repaint();
            }
        }
        // ---- input
        self.camera.aspect = (rect.width() / rect.height()).max(0.1) as f64;
        if opts.navigate {
            if resp.dragged_by(egui::PointerButton::Primary) {
                let d = resp.drag_delta();
                self.camera.orbit(d.x as f64, d.y as f64);
                self.anim = None;
            }
            if resp.dragged_by(egui::PointerButton::Secondary)
                || resp.dragged_by(egui::PointerButton::Middle)
            {
                let d = resp.drag_delta();
                self.camera
                    .pan(d.x as f64, d.y as f64, rect.height() as f64);
            }
            if resp.hovered() {
                let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.camera.zoom(scroll as f64 / 120.0);
                }
            }
        }
        if let Some(m) = &self.model {
            self.camera.update_clip(m.scene.radius.max(1e-3));
        }
        // ---- draw
        gpu.renderer.settings.show_mesh = !self.wireframe;
        gpu.renderer.settings.show_edges = self.show_edges;
        gpu.renderer.settings.background = crate::chrome::theme::VIEWPORT;
        // the scene is centred on its middle: the plane moves with it
        gpu.renderer.settings.section = match (&self.section, &self.model) {
            (Some((n, d)), Some(m)) => {
                let c = m.scene.center;
                Some((*n, *d - (n[0] * c.x + n[1] * c.y + n[2] * c.z)))
            }
            _ => None,
        };
        gpu.renderer.settings.analysis =
            Self::renderer_analysis(&self.analysis, self.model.as_ref());
        gpu.renderer.selected = highlight
            .and_then(|p| self.model.as_ref().map(|m| m.id_of(p)))
            .unwrap_or(0);
        gpu.renderer.selected_extra = self
            .model
            .as_ref()
            .map(|m| {
                self.extra_selected
                    .iter()
                    .map(|p| m.id_of(*p))
                    .filter(|id| *id != 0)
                    .collect()
            })
            .unwrap_or_default();
        gpu.renderer.hovered = self
            .hover
            .and_then(|p| self.model.as_ref().map(|m| m.id_of(p)))
            .unwrap_or(0);
        gpu.render(device, &rs.queue, &self.camera);
        if let Some(id) = self.tex {
            ui.painter().image(
                id,
                rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        }
        // ---- hover (one id-buffer pixel, only when the pointer moved)
        let mut out = ViewOut::none(rect);
        if opts.hover {
            let pos = resp.hover_pos();
            if pos != self.last_pointer || self.last_edge_px != self.edge_pick_px {
                self.last_pointer = pos;
                self.last_edge_px = self.edge_pick_px;
                let reach = (self.edge_pick_px as f32 * ppp).round() as u32;
                self.hover = pos.and_then(|pos| {
                    let x = ((pos.x - rect.min.x) * ppp) as u32;
                    let y = ((pos.y - rect.min.y) * ppp) as u32;
                    self.model
                        .as_ref()
                        .and_then(|m| gpu.pick_near(device, &rs.queue, m, x, y, reach))
                });
            }
        } else {
            self.hover = None;
        }
        out.hover = self.hover;
        // ---- clicks
        let reach = (self.edge_pick_px as f32 * ppp).round() as u32;
        let pick_at =
            |gpu: &GpuView, model: &Option<ModelScene>, pos: egui::Pos2| -> Option<Pick> {
                let x = ((pos.x - rect.min.x) * ppp) as u32;
                let y = ((pos.y - rect.min.y) * ppp) as u32;
                model
                    .as_ref()
                    .and_then(|m| gpu.pick_near(device, &rs.queue, m, x, y, reach))
            };
        if resp.clicked_by(egui::PointerButton::Primary) {
            if let Some(pos) = resp.interact_pointer_pos() {
                out.click = Some(pick_at(gpu, &self.model, pos));
                out.click_pos = Some(pos);
            }
        }
        if resp.double_clicked_by(egui::PointerButton::Primary) {
            if let Some(pos) = resp.interact_pointer_pos() {
                out.double_click = Some(pick_at(gpu, &self.model, pos));
            }
        }
        out.response = Some(resp);
        out
    }
}

impl ModelScene {
    /// The polylines of the B-rep edges (world coordinates).
    pub fn edge_polylines(&self) -> Vec<Vec<[f64; 3]>> {
        let mut out = Vec::new();
        for t in &self.tess {
            for (_, start, count) in &t.edges.ranges {
                let pts = &t.edges.points[*start as usize..(*start + *count) as usize];
                out.push(pts.iter().map(|p| [p.x, p.y, p.z]).collect());
            }
        }
        out
    }

    /// Add line segments (world coordinates) to the scene, drawn like edges and not pickable.
    pub fn add_segments(&mut self, segments: &[([f64; 3], [f64; 3])]) {
        for (a, b) in segments {
            let pa = self
                .scene
                .world_to_scene(&nalgebra::Vector3::new(a[0], a[1], a[2]));
            let pb = self
                .scene
                .world_to_scene(&nalgebra::Vector3::new(b[0], b[1], b[2]));
            self.scene.segments.push(kw_viewer::scene::Segment {
                p0: [pa.x as f32, pa.y as f32, pa.z as f32],
                p1: [pb.x as f32, pb.y as f32, pb.z as f32],
                id: u32::MAX,
            });
        }
    }
}
