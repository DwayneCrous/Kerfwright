//! Windowed viewer (winit + wgpu). Left-drag orbits, right/middle-drag pans,
//! wheel zooms, `E` toggles edges, `F` re-frames, `Esc` quits. Hover and click
//! pick faces/edges through the ID buffer.

use crate::camera::OrbitCamera;
use crate::gpu::Gpu;
use crate::headless::make_scene;
use crate::renderer::Renderer;
use crate::scene::{PickTarget, Scene};
use std::sync::Arc;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

struct State {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    gpu: Gpu,
    renderer: Renderer,
    scene: Scene,
    cam: OrbitCamera,
}

struct App {
    scene_name: String,
    tris: usize,
    size: (u32, u32),
    vsync: bool,
    exit_after: Option<u64>,
    state: Option<State>,
    mouse: (f64, f64),
    drag: Option<(MouseButton, (f64, f64), bool)>,
    frame: u64,
    fps_clock: Instant,
    fps_frames: u32,
    label: String,
}

pub fn run(scene_name: &str, tris: usize, size: (u32, u32), vsync: bool, exit_after: Option<u64>) {
    let event_loop = EventLoop::new().expect("event loop");
    event_loop.set_control_flow(ControlFlow::Poll);
    let mut app = App {
        scene_name: scene_name.to_string(),
        tris,
        size,
        vsync,
        exit_after,
        state: None,
        mouse: (0.0, 0.0),
        drag: None,
        frame: 0,
        fps_clock: Instant::now(),
        fps_frames: 0,
        label: String::new(),
    };
    event_loop.run_app(&mut app).expect("run");
}

fn describe(scene: &Scene, id: u32) -> String {
    if id == 0 {
        return "nothing".into();
    }
    match scene.pick.get(id as usize - 1) {
        Some(PickTarget::Face { object, face }) => format!("face {face} of object {object}"),
        Some(PickTarget::Edge { object, edge }) => format!("edge {edge} of object {object}"),
        None => "?".into(),
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        let window = Arc::new(
            event_loop
                .create_window(
                    Window::default_attributes()
                        .with_title("Kerfwright")
                        .with_inner_size(PhysicalSize::new(self.size.0, self.size.1)),
                )
                .expect("window"),
        );
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_with_display_handle_from_env(
                Box::new(event_loop.owned_display_handle()),
            ));
        let surface = instance.create_surface(window.clone()).expect("surface");
        let gpu = Gpu::new(instance, Some(&surface));
        println!("adapter: {}", gpu.describe());
        let caps = surface.get_capabilities(&gpu.adapter);
        let format = caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(caps.formats[0]);
        let sz = window.inner_size();
        let mut config = surface
            .get_default_config(&gpu.adapter, sz.width.max(1), sz.height.max(1))
            .expect("surface config");
        config.format = format;
        config.present_mode = if self.vsync {
            wgpu::PresentMode::AutoVsync
        } else {
            wgpu::PresentMode::AutoNoVsync
        };
        surface.configure(&gpu.device, &config);
        let scene = make_scene(&self.scene_name, self.tris);
        println!(
            "scene `{}`: {} triangles",
            self.scene_name,
            scene.triangle_count()
        );
        let mut renderer = Renderer::new(&gpu.device, format, (config.width, config.height));
        renderer.upload(&gpu.device, &scene);
        let cam = OrbitCamera::new(scene.radius, config.width as f64 / config.height as f64);
        self.state = Some(State {
            window,
            surface,
            config,
            gpu,
            renderer,
            scene,
            cam,
        });
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        let Some(st) = self.state.as_mut() else {
            return;
        };
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(sz) => {
                st.config.width = sz.width.max(1);
                st.config.height = sz.height.max(1);
                st.surface.configure(&st.gpu.device, &st.config);
                st.renderer
                    .resize(&st.gpu.device, (st.config.width, st.config.height));
                st.cam.aspect = st.config.width as f64 / st.config.height as f64;
            }
            WindowEvent::CursorMoved { position, .. } => {
                let (dx, dy) = (position.x - self.mouse.0, position.y - self.mouse.1);
                self.mouse = (position.x, position.y);
                if let Some((btn, start, moved)) = self.drag.as_mut() {
                    if (position.x - start.0).abs() + (position.y - start.1).abs() > 3.0 {
                        *moved = true;
                    }
                    match btn {
                        MouseButton::Left => st.cam.orbit(dx, dy),
                        MouseButton::Right | MouseButton::Middle => {
                            st.cam.pan(dx, dy, st.config.height as f64)
                        }
                        _ => {}
                    }
                }
            }
            WindowEvent::MouseInput { state, button, .. } => match state {
                ElementState::Pressed => self.drag = Some((button, self.mouse, false)),
                ElementState::Released => {
                    if let Some((MouseButton::Left, _, false)) = self.drag {
                        st.renderer.selected = st.renderer.hovered;
                        self.label =
                            format!("selected {}", describe(&st.scene, st.renderer.selected));
                    }
                    self.drag = None;
                }
            },
            WindowEvent::MouseWheel { delta, .. } => {
                let s = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 40.0,
                };
                st.cam.zoom(s);
            }
            WindowEvent::KeyboardInput { event, .. } if event.state == ElementState::Pressed => {
                match event.logical_key {
                    Key::Named(NamedKey::Escape) => event_loop.exit(),
                    Key::Character(c) if c.eq_ignore_ascii_case("e") => {
                        st.renderer.settings.show_edges = !st.renderer.settings.show_edges;
                    }
                    Key::Character(c) if c.eq_ignore_ascii_case("f") => {
                        st.cam.target = nalgebra::Vector3::zeros();
                        st.cam.fit(st.scene.radius);
                    }
                    _ => {}
                }
            }
            WindowEvent::RedrawRequested => {
                st.cam.update_clip(st.scene.radius);
                let frame = match st.surface.get_current_texture() {
                    wgpu::CurrentSurfaceTexture::Success(f) => f,
                    wgpu::CurrentSurfaceTexture::Suboptimal(_)
                    | wgpu::CurrentSurfaceTexture::Outdated => {
                        st.surface.configure(&st.gpu.device, &st.config);
                        st.window.request_redraw();
                        return;
                    }
                    wgpu::CurrentSurfaceTexture::Lost => {
                        if let Ok(s) = st.gpu.instance.create_surface(st.window.clone()) {
                            st.surface = s;
                            st.surface.configure(&st.gpu.device, &st.config);
                        }
                        return;
                    }
                    _ => return,
                };
                let view = frame
                    .texture
                    .create_view(&wgpu::TextureViewDescriptor::default());
                st.renderer
                    .render(&st.gpu.device, &st.gpu.queue, &st.cam, &view);
                st.window.pre_present_notify();
                st.gpu.queue.present(frame);

                // hover pick (blocking 1-pixel readback of the ID buffer)
                let (mx, my) = (self.mouse.0 as i64, self.mouse.1 as i64);
                if self.drag.is_none()
                    && mx >= 0
                    && my >= 0
                    && (mx as u32) < st.config.width
                    && (my as u32) < st.config.height
                {
                    let id = st.renderer.read_ids(
                        &st.gpu.device,
                        &st.gpu.queue,
                        mx as u32,
                        my as u32,
                        1,
                        1,
                    )[0];
                    if id != st.renderer.hovered {
                        st.renderer.hovered = id;
                    }
                }

                self.frame += 1;
                self.fps_frames += 1;
                if self.fps_clock.elapsed().as_secs_f64() >= 1.0 {
                    let fps = self.fps_frames as f64 / self.fps_clock.elapsed().as_secs_f64();
                    st.window.set_title(&format!(
                        "Kerfwright — {:.0} FPS — {} tris — hover: {} {}",
                        fps,
                        st.scene.triangle_count(),
                        describe(&st.scene, st.renderer.hovered),
                        self.label
                    ));
                    if self.exit_after.is_some() {
                        println!("{fps:.1} FPS");
                    }
                    self.fps_clock = Instant::now();
                    self.fps_frames = 0;
                }
                if let Some(n) = self.exit_after {
                    if self.frame >= n {
                        event_loop.exit();
                    }
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(st) = &self.state {
            st.window.request_redraw();
        }
    }
}
