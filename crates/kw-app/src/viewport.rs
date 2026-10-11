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

/// What a click in the viewport hit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pick {
    Face(FaceId),
    Edge(EdgeId),
}

/// The model as drawn: tessellations of the bodies, their scene, and the lookup from pick ids to B-rep ids.
pub struct ModelScene {
    pub scene: Scene,
    tess: Vec<Tessellation>,
}

impl ModelScene {
    /// A scene made of arbitrary geometry (no B-rep behind it, so nothing can be picked).
    pub fn custom(scene: Scene) -> ModelScene {
        ModelScene {
            scene,
            tess: Vec::new(),
        }
    }

    pub fn build(brep: &BRep) -> ModelScene {
        let bodies: Vec<BodyId> = brep.bodies.keys().collect();
        let opts = TessOptions {
            chord_tol: 0.02,
            angle_tol: 0.2,
        };
        let tess: Vec<Tessellation> = bodies
            .iter()
            .filter_map(|b| tessellate_body(brep, *b, &opts).ok())
            .collect();
        let objects: Vec<SceneObject> = tess
            .iter()
            .map(|t| SceneObject {
                tess: t,
                transform: Isometry3::identity(),
                color: [150, 170, 200],
            })
            .collect();
        let scene = Scene::build(&objects);
        ModelScene { scene, tess }
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

/// The viewport widget state.
pub struct Viewport3d {
    gpu: Option<GpuView>,
    tex: Option<egui::TextureId>,
    pub camera: OrbitCamera,
    /// Draw edges only (to see toolpaths inside the material).
    pub wireframe: bool,
    model: Option<ModelScene>,
    /// Identifies the geometry currently uploaded.
    key: Option<u128>,
    fitted: bool,
}

impl Default for Viewport3d {
    fn default() -> Self {
        Viewport3d {
            gpu: None,
            tex: None,
            camera: OrbitCamera::new(50.0, 1.5),
            wireframe: false,
            model: None,
            key: None,
            fitted: false,
        }
    }
}

impl Viewport3d {
    pub fn model(&self) -> Option<&ModelScene> {
        self.model.as_ref()
    }

    /// Show the viewport. `geometry` is rebuilt only when `key` changes. Returns a pick if the user clicked
    /// on the model (`Some(None)` = clicked on empty space).
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
        )
    }

    /// Like [`Viewport3d::show`], with the geometry built by `build` whenever `key` changes.
    pub fn show_with(
        &mut self,
        ui: &mut egui::Ui,
        rs: Option<&eframe::egui_wgpu::RenderState>,
        key: u128,
        build: impl FnOnce() -> ModelScene,
        highlight: Option<Pick>,
    ) -> Option<Option<Pick>> {
        let size = ui.available_size().max(egui::vec2(64.0, 64.0));
        let (rect, resp) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
        let Some(rs) = rs else {
            ui.painter()
                .rect_filled(rect, 0.0, egui::Color32::from_gray(30));
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "3-D view needs a GPU",
                egui::FontId::proportional(16.0),
                egui::Color32::GRAY,
            );
            return None;
        };
        let ppp = ui.ctx().pixels_per_point();
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
            if !self.fitted && !m.scene.vertices.is_empty() {
                self.camera.target = nalgebra::Vector3::zeros();
                self.camera.fit(m.scene.radius.max(1e-3));
                self.fitted = true;
            }
            self.camera.update_clip(m.scene.radius.max(1e-3));
            self.model = Some(m);
            self.key = Some(key);
        }
        // ---- input
        self.camera.aspect = (rect.width() / rect.height()).max(0.1) as f64;
        if resp.dragged_by(egui::PointerButton::Primary) {
            let d = resp.drag_delta();
            self.camera.orbit(d.x as f64, d.y as f64);
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
        if let Some(m) = &self.model {
            self.camera.update_clip(m.scene.radius.max(1e-3));
        }
        // ---- draw
        gpu.renderer.settings.show_mesh = !self.wireframe;
        gpu.renderer.selected = highlight
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
        // ---- pick
        if resp.clicked_by(egui::PointerButton::Primary) {
            if let Some(pos) = resp.interact_pointer_pos() {
                let x = ((pos.x - rect.min.x) * ppp) as u32;
                let y = ((pos.y - rect.min.y) * ppp) as u32;
                let id = gpu.pick_id(device, &rs.queue, x, y);
                return Some(self.model.as_ref().and_then(|m| m.resolve(id)));
            }
        }
        None
    }
}

impl ModelScene {
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
