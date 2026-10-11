//! The Render workspace: the model drawn with the realtime pipeline (order-independent transparency, ambient
//! occlusion, edges) or path traced on the GPU with progressive refinement and an optional denoise.

use crate::dialogs::Dialogs;
use crate::job::Job;
use crate::project::Project;
use crate::viewport::ModelScene;
use eframe::egui;
use kw_render::pt::cpu_ref::Reference;
use kw_render::pt::denoise::{denoise, DenoiseParams, Guides};
use kw_render::pt::gpu::GpuTracer;
use kw_render::pt::metrics::tonemap;
use kw_render::pt::{Material, PtCamera, PtScene};
use kw_render::{offscreen, read_rgba, Frame, FrameOptions, RenderGpu};
use kw_viewer::OrbitCamera;
use std::sync::Arc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Realtime,
    PathTrace,
}

pub const IMAGE: (u32, u32) = (960, 600);

/// The model as a path-tracer scene: the part, a floor under it and an area light above.
pub fn pt_scene(scene: &kw_viewer::scene::Scene) -> Option<PtScene> {
    if scene.vertices.is_empty() {
        return None;
    }
    let mut s = PtScene::new([0.22, 0.25, 0.32]);
    let part = s.add_material(Material::glossy([0.62, 0.68, 0.80], 0.35, 0.0));
    let floor = s.add_material(Material::lambert([0.42, 0.42, 0.42]));
    let lamp = s.add_material(Material::light([2.6, 2.5, 2.4]));
    let mut zmin = f32::MAX;
    let mut zmax = f32::MIN;
    for c in scene.indices.as_chunks::<3>().0 {
        let p = |i: u32| scene.vertices[i as usize].pos;
        let tri = [p(c[0]), p(c[1]), p(c[2])];
        for v in &tri {
            zmin = zmin.min(v[2]);
            zmax = zmax.max(v[2]);
        }
        s.add_tri(tri, part);
    }
    let r = scene.radius as f32;
    let f = 8.0 * r;
    let z0 = zmin - 0.002 * r;
    s.add_quad([-f, -f, z0], [f, -f, z0], [f, f, z0], [-f, f, z0], floor);
    let (l, zl) = (1.6 * r, zmax + 2.5 * r);
    // facing down
    s.add_quad([-l, l, zl], [l, l, zl], [l, -l, zl], [-l, -l, zl], lamp);
    Some(s)
}

fn pt_camera(cam: &OrbitCamera, size: (u32, u32)) -> PtCamera {
    let e = cam.eye();
    let fwd = cam.target - e;
    PtCamera {
        origin: [e.x as f32, e.y as f32, e.z as f32],
        forward: [fwd.x as f32, fwd.y as f32, fwd.z as f32],
        up: [0.0, 0.0, 1.0],
        fov: cam.fov_y as f32,
        size,
    }
}

pub struct RenderWorkspace {
    gpu: Option<RenderGpu>,
    gpu_tried: bool,
    pub mode: Mode,
    pub camera: OrbitCamera,
    pub options: FrameOptions,
    frame: Option<Frame>,
    uploaded_key: Option<u128>,
    scene: Option<ModelScene>,
    tracer: Option<GpuTracer>,
    pub target_spp: u32,
    pub spp_done: u32,
    pt_source: Option<PtScene>,
    noisy: Vec<[f64; 3]>,
    denoise_job: Option<Job<Vec<[f64; 3]>>>,
    texture: Option<egui::TextureHandle>,
    /// Latest image as sRGB RGBA bytes (for saving).
    pub pixels: Vec<u8>,
    dirty: bool,
    fitted: bool,
    pub message: String,
}

impl Default for RenderWorkspace {
    fn default() -> Self {
        RenderWorkspace {
            gpu: None,
            gpu_tried: false,
            mode: Mode::Realtime,
            camera: OrbitCamera::new(50.0, IMAGE.0 as f64 / IMAGE.1 as f64),
            options: FrameOptions::default(),
            frame: None,
            uploaded_key: None,
            scene: None,
            tracer: None,
            target_spp: 256,
            spp_done: 0,
            pt_source: None,
            noisy: Vec::new(),
            denoise_job: None,
            texture: None,
            pixels: Vec::new(),
            dirty: true,
            fitted: false,
            message: String::new(),
        }
    }
}

fn to_pixels(img: &[[f64; 3]]) -> Vec<u8> {
    img.iter()
        .flat_map(|p| {
            [
                (tonemap(p[0]) * 255.0 + 0.5) as u8,
                (tonemap(p[1]) * 255.0 + 0.5) as u8,
                (tonemap(p[2]) * 255.0 + 0.5) as u8,
                255,
            ]
        })
        .collect()
}

impl RenderWorkspace {
    fn ensure_gpu(&mut self) -> bool {
        if !self.gpu_tried {
            self.gpu_tried = true;
            self.gpu = RenderGpu::try_new();
            if self.gpu.is_none() {
                self.message = "no GPU adapter: rendering is unavailable".into();
            }
        }
        self.gpu.is_some()
    }

    fn set_geometry(&mut self, key: u128, brep: Option<&Arc<kw_brep::BRep>>) {
        if self.uploaded_key == Some(key) {
            return;
        }
        let m = match brep {
            Some(b) => ModelScene::build(b),
            None => ModelScene::empty(),
        };
        if let (Some(g), Some(f)) = (&self.gpu, self.frame.as_mut()) {
            f.upload(&g.device, &m.scene);
        }
        if !m.scene.vertices.is_empty() {
            self.camera.target = nalgebra::Vector3::zeros();
            if !self.fitted {
                self.camera.fit(m.scene.radius.max(1e-3));
                self.fitted = true;
            }
            self.camera.update_clip(m.scene.radius.max(1e-3));
        }
        self.pt_source = pt_scene(&m.scene);
        self.scene = Some(m);
        self.uploaded_key = Some(key);
        self.tracer = None;
        self.spp_done = 0;
        self.dirty = true;
    }

    fn set_pixels(&mut self, ctx: &egui::Context, rgba: Vec<u8>) {
        let img =
            egui::ColorImage::from_rgba_unmultiplied([IMAGE.0 as usize, IMAGE.1 as usize], &rgba);
        match &mut self.texture {
            Some(t) => t.set(img, egui::TextureOptions::LINEAR),
            None => {
                self.texture =
                    Some(ctx.load_texture("render_image", img, egui::TextureOptions::LINEAR))
            }
        }
        self.pixels = rgba;
    }

    /// Draw the realtime image for the current camera.
    pub fn render_realtime(&mut self, ctx: &egui::Context) -> bool {
        let Some(g) = &self.gpu else { return false };
        if self.frame.is_none() {
            let mut f = Frame::new(&g.device, &g.queue, IMAGE, g.timestamps);
            if let Some(m) = &self.scene {
                f.upload(&g.device, &m.scene);
            }
            self.frame = Some(f);
        }
        self.camera.aspect = IMAGE.0 as f64 / IMAGE.1 as f64;
        let (tex, view) = offscreen(&g.device, IMAGE);
        self.frame.as_ref().expect("frame").render(
            &g.device,
            &g.queue,
            &self.camera,
            &view,
            &self.options,
        );
        let rgba = read_rgba(g, &tex, IMAGE);
        self.set_pixels(ctx, rgba);
        true
    }

    pub fn start_path_trace(&mut self) {
        if !self.ensure_gpu() {
            return;
        }
        let (Some(g), Some(src)) = (&self.gpu, &self.pt_source) else {
            self.message = "there is no model to render".into();
            return;
        };
        self.camera.aspect = IMAGE.0 as f64 / IMAGE.1 as f64;
        self.tracer = Some(GpuTracer::new(g, src, pt_camera(&self.camera, IMAGE)));
        self.spp_done = 0;
        self.noisy.clear();
        self.mode = Mode::PathTrace;
        self.message = "path tracing…".into();
    }

    /// Add samples (a few per frame, so the window stays responsive).
    pub fn trace_step(&mut self, ctx: &egui::Context, spp: u32) {
        let (Some(g), Some(t)) = (&self.gpu, self.tracer.as_mut()) else {
            return;
        };
        if self.spp_done >= self.target_spp {
            return;
        }
        let n = spp.min(self.target_spp - self.spp_done);
        t.accumulate(g, n, 17 + self.spp_done);
        self.spp_done += n;
        let img = t.image(g);
        self.set_pixels(ctx, to_pixels(&img));
        self.noisy = img;
        if self.spp_done >= self.target_spp {
            self.message = format!("path tracing done: {} samples per pixel", self.spp_done);
        }
    }

    /// Start the denoise job as the button does (tests).
    pub fn show_denoise_for_test(&mut self) {
        self.start_denoise();
    }

    fn start_denoise(&mut self) {
        let (Some(src), false) = (self.pt_source.clone(), self.noisy.is_empty()) else {
            return;
        };
        let (img, cam) = (self.noisy.clone(), pt_camera(&self.camera, IMAGE));
        self.denoise_job = Some(Job::spawn("denoise", move || {
            let r = Reference::new(&src);
            let g = r.guides(&cam);
            let guides = Guides {
                albedo: g.iter().map(|x| x.0).collect(),
                normal: g.iter().map(|x| x.1).collect(),
                depth: g.iter().map(|x| x.2).collect(),
            };
            Ok(denoise(&img, &guides, cam.size, &DenoiseParams::default()))
        }));
        self.message = "denoising…".into();
    }

    pub fn show(&mut self, ui: &mut egui::Ui, project: &mut Project, dialogs: &mut dyn Dialogs) {
        let ev = project.evaluation();
        let brep = ev.final_solid().and_then(|(_, o)| o.brep().cloned());
        let key = ev
            .final_hashes()
            .map(|h| h.0 ^ h.1.rotate_left(1))
            .unwrap_or(0);
        let have_gpu = self.ensure_gpu();
        if have_gpu {
            if self.frame.is_none() {
                let g = self.gpu.as_ref().expect("gpu");
                self.frame = Some(Frame::new(&g.device, &g.queue, IMAGE, g.timestamps));
                self.uploaded_key = None;
            }
            self.set_geometry(key, brep.as_ref());
        }
        if let Some(job) = &self.denoise_job {
            match job.poll() {
                Some(Ok(img)) => {
                    self.set_pixels(ui.ctx(), to_pixels(&img));
                    self.message = "denoised".into();
                    self.denoise_job = None;
                }
                Some(Err(e)) => {
                    self.message = e;
                    self.denoise_job = None;
                }
                None => ui.ctx().request_repaint(),
            }
        }
        egui::Panel::left("render_side")
            .default_size(280.0)
            .show(ui, |ui| {
                ui.heading("Render");
                let mut changed = false;
                ui.horizontal(|ui| {
                    changed |= ui
                        .selectable_value(&mut self.mode, Mode::Realtime, "Realtime")
                        .changed();
                    if ui
                        .selectable_label(self.mode == Mode::PathTrace, "Path traced")
                        .clicked()
                        && self.mode != Mode::PathTrace
                    {
                        self.start_path_trace();
                    }
                });
                match self.mode {
                    Mode::Realtime => {
                        changed |= ui
                            .checkbox(&mut self.options.oit, "Transparency (OIT)")
                            .changed();
                        changed |= ui
                            .checkbox(&mut self.options.ao, "Ambient occlusion")
                            .changed();
                        changed |= ui.checkbox(&mut self.options.edges, "Edges").changed();
                        changed |= ui
                            .add(
                                egui::Slider::new(&mut self.options.ao_radius, 0.05..=3.0)
                                    .text("AO radius"),
                            )
                            .changed();
                    }
                    Mode::PathTrace => {
                        ui.add(
                            egui::Slider::new(&mut self.target_spp, 16..=2048)
                                .logarithmic(true)
                                .text("Samples / pixel"),
                        );
                        ui.label(format!("{} / {} samples", self.spp_done, self.target_spp));
                        ui.horizontal(|ui| {
                            if ui.button("Restart").clicked() {
                                self.start_path_trace();
                            }
                            if ui
                                .add_enabled(
                                    self.denoise_job.is_none() && !self.noisy.is_empty(),
                                    egui::Button::new("Denoise"),
                                )
                                .clicked()
                            {
                                self.start_denoise();
                            }
                            if self.denoise_job.is_some() {
                                ui.spinner();
                            }
                        });
                    }
                }
                ui.separator();
                if ui
                    .add_enabled(!self.pixels.is_empty(), egui::Button::new("Save PNG…"))
                    .clicked()
                {
                    if let Some(p) =
                        dialogs.save_file(&format!("{}.png", project.name()), "PNG", &["png"])
                    {
                        self.message = match self.save_png(&p) {
                            Ok(()) => format!("wrote {}", p.display()),
                            Err(e) => format!("could not write: {e}"),
                        };
                    }
                }
                if !self.message.is_empty() {
                    ui.separator();
                    ui.label(&self.message);
                }
                ui.separator();
                ui.weak("Drag to orbit, right-drag to pan, scroll to zoom.");
                if changed {
                    self.dirty = true;
                }
            });
        egui::CentralPanel::default().show(ui, |ui| {
            let avail = ui.available_size();
            let (rect, resp) = ui.allocate_exact_size(avail, egui::Sense::click_and_drag());
            ui.painter()
                .rect_filled(rect, 0.0, egui::Color32::from_gray(24));
            if !have_gpu {
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "Rendering needs a GPU",
                    egui::FontId::proportional(16.0),
                    egui::Color32::GRAY,
                );
                return;
            }
            // camera input
            let mut moved = false;
            if resp.dragged_by(egui::PointerButton::Primary) {
                let d = resp.drag_delta();
                self.camera.orbit(d.x as f64, d.y as f64);
                moved = true;
            }
            if resp.dragged_by(egui::PointerButton::Secondary)
                || resp.dragged_by(egui::PointerButton::Middle)
            {
                let d = resp.drag_delta();
                self.camera
                    .pan(d.x as f64, d.y as f64, rect.height() as f64);
                moved = true;
            }
            if resp.hovered() {
                let s = ui.ctx().input(|i| i.smooth_scroll_delta.y);
                if s != 0.0 {
                    self.camera.zoom(s as f64 / 120.0);
                    moved = true;
                }
            }
            if let Some(m) = &self.scene {
                self.camera.update_clip(m.scene.radius.max(1e-3));
            }
            if moved {
                self.dirty = true;
                if self.mode == Mode::PathTrace {
                    // show the fast preview while moving; the trace restarts when the camera rests
                    self.tracer = None;
                    self.spp_done = 0;
                }
            }
            match self.mode {
                Mode::Realtime => {
                    if self.dirty || self.texture.is_none() {
                        self.render_realtime(ui.ctx());
                        self.dirty = false;
                    }
                }
                Mode::PathTrace => {
                    if self.tracer.is_none() && !moved && !resp.dragged() {
                        self.start_path_trace();
                    }
                    if self.tracer.is_some()
                        && self.denoise_job.is_none()
                        && self.spp_done < self.target_spp
                    {
                        self.trace_step(ui.ctx(), 4);
                        ui.ctx().request_repaint();
                    } else if self.tracer.is_none() && self.dirty {
                        self.render_realtime(ui.ctx());
                        self.dirty = false;
                    }
                }
            }
            if let Some(t) = &self.texture {
                // fit the image into the area, keeping its aspect
                let aspect = IMAGE.0 as f32 / IMAGE.1 as f32;
                let (w, h) = if rect.width() / rect.height() > aspect {
                    (rect.height() * aspect, rect.height())
                } else {
                    (rect.width(), rect.width() / aspect)
                };
                let r = egui::Rect::from_center_size(rect.center(), egui::vec2(w, h));
                ui.painter().image(
                    t.id(),
                    r,
                    egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                    egui::Color32::WHITE,
                );
            }
        });
    }

    pub fn save_png(&self, path: &std::path::Path) -> Result<(), String> {
        if self.pixels.is_empty() {
            return Err("nothing has been rendered yet".into());
        }
        let f = std::fs::File::create(path).map_err(|e| e.to_string())?;
        let mut enc = png::Encoder::new(std::io::BufWriter::new(f), IMAGE.0, IMAGE.1);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        enc.write_header()
            .map_err(|e| e.to_string())?
            .write_image_data(&self.pixels)
            .map_err(|e| e.to_string())
    }
}
