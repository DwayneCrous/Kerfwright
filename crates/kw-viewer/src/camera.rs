use nalgebra::{Matrix4, Point3, Vector3, Vector4};

/// Orbit camera for a z-up world. All coordinates are in the *scene-relative*
/// frame (the scene is re-centred on the CPU in f64 before upload, so f32 on
/// the GPU never sees large absolute coordinates).
#[derive(Debug, Clone)]
pub struct OrbitCamera {
    pub target: Vector3<f64>,
    pub distance: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub fov_y: f64,
    pub aspect: f64,
    pub near: f64,
    pub far: f64,
}

const OPENGL_TO_WGPU: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, //
    0.0, 1.0, 0.0, 0.0, //
    0.0, 0.0, 0.5, 0.0, //
    0.0, 0.0, 0.5, 1.0,
];

impl OrbitCamera {
    pub fn new(radius: f64, aspect: f64) -> Self {
        let mut c = Self {
            target: Vector3::zeros(),
            distance: 1.0,
            yaw: -0.8,
            pitch: 0.45,
            fov_y: 45.0_f64.to_radians(),
            aspect,
            near: 0.1,
            far: 100.0,
        };
        c.fit(radius);
        c
    }

    /// Frame a sphere of `radius` centred at the target.
    pub fn fit(&mut self, radius: f64) {
        let r = radius.max(1e-6);
        self.distance = r / (self.fov_y * 0.5).sin() * 1.05;
        self.update_clip(r);
    }

    pub fn update_clip(&mut self, scene_radius: f64) {
        self.near = (self.distance - scene_radius * 1.5)
            .max(self.distance * 0.01)
            .max(1e-4);
        self.far = self.distance + scene_radius * 3.0;
    }

    pub fn eye(&self) -> Vector3<f64> {
        let (cp, sp) = (self.pitch.cos(), self.pitch.sin());
        self.target + Vector3::new(cp * self.yaw.cos(), cp * self.yaw.sin(), sp) * self.distance
    }

    pub fn orbit(&mut self, dx_px: f64, dy_px: f64) {
        self.yaw -= dx_px * 0.01;
        self.pitch = (self.pitch + dy_px * 0.01).clamp(-1.55, 1.55);
    }

    pub fn pan(&mut self, dx_px: f64, dy_px: f64, viewport_h: f64) {
        let eye = self.eye();
        let fwd = (self.target - eye).normalize();
        let right = fwd.cross(&Vector3::z()).normalize();
        let up = right.cross(&fwd);
        let world_per_px = 2.0 * self.distance * (self.fov_y * 0.5).tan() / viewport_h;
        self.target += (-right * dx_px + up * dy_px) * world_per_px;
    }

    pub fn zoom(&mut self, scroll: f64) {
        self.distance = (self.distance * (-scroll * 0.1).exp()).max(1e-3);
    }

    pub fn view(&self) -> Matrix4<f64> {
        Matrix4::look_at_rh(
            &Point3::from(self.eye()),
            &Point3::from(self.target),
            &Vector3::z(),
        )
    }

    pub fn proj(&self) -> Matrix4<f64> {
        let p = Matrix4::new_perspective(self.aspect, self.fov_y, self.near, self.far);
        Matrix4::from_column_slice(&OPENGL_TO_WGPU) * p
    }

    pub fn view_proj(&self) -> Matrix4<f64> {
        self.proj() * self.view()
    }

    pub fn view_proj_f32(&self) -> [[f32; 4]; 4] {
        let m = self.view_proj();
        let mut out = [[0.0f32; 4]; 4];
        for c in 0..4 {
            for r in 0..4 {
                out[c][r] = m[(r, c)] as f32;
            }
        }
        out
    }

    /// World-space ray through the centre of pixel `(px, py)` (origin top-left).
    pub fn pixel_ray(
        &self,
        px: f64,
        py: f64,
        width: f64,
        height: f64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let ndc_x = (px + 0.5) / width * 2.0 - 1.0;
        let ndc_y = 1.0 - (py + 0.5) / height * 2.0;
        let inv = self
            .view_proj()
            .try_inverse()
            .expect("invertible view-projection");
        let near = inv * Vector4::new(ndc_x, ndc_y, 0.0, 1.0);
        let far = inv * Vector4::new(ndc_x, ndc_y, 1.0, 1.0);
        let n = near.xyz() / near.w;
        let f = far.xyz() / far.w;
        (n, (f - n).normalize())
    }

    /// Project a world point to pixel coordinates; `None` if behind the camera.
    pub fn project(&self, p: &Vector3<f64>, width: f64, height: f64) -> Option<(f64, f64)> {
        let c = self.view_proj() * Vector4::new(p.x, p.y, p.z, 1.0);
        if c.w <= 1e-9 {
            return None;
        }
        let (x, y) = (c.x / c.w, c.y / c.w);
        Some((
            (x + 1.0) * 0.5 * width - 0.5,
            (1.0 - y) * 0.5 * height - 0.5,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn project_and_ray_agree() {
        let cam = OrbitCamera::new(5.0, 16.0 / 9.0);
        let p = Vector3::new(1.0, -2.0, 0.5);
        let (px, py) = cam.project(&p, 1600.0, 900.0).unwrap();
        let (o, d) = cam.pixel_ray(px, py, 1600.0, 900.0);
        let to_p = p - o;
        let along = to_p.dot(&d);
        assert!((to_p - d * along).norm() < 1e-9);
    }

    #[test]
    fn target_projects_to_screen_centre() {
        let cam = OrbitCamera::new(3.0, 1.5);
        let (px, py) = cam.project(&cam.target, 1200.0, 800.0).unwrap();
        assert!((px - 599.5).abs() < 1e-6 && (py - 399.5).abs() < 1e-6);
    }
}
