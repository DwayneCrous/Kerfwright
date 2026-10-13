//! Pan / zoom transform between sketch coordinates (y up) and screen pixels (y down).

use eframe::egui::{Pos2, Rect};

#[derive(Debug, Clone, Copy)]
pub struct View {
    /// Sketch point at the centre of the canvas.
    pub center: (f64, f64),
    /// Pixels per sketch unit.
    pub scale: f64,
}

impl Default for View {
    fn default() -> Self {
        Self {
            center: (0.0, 0.0),
            scale: 20.0,
        }
    }
}

impl View {
    pub const MIN_SCALE: f64 = 1e-3;
    pub const MAX_SCALE: f64 = 1e6;

    pub fn to_screen(&self, rect: Rect, p: (f64, f64)) -> Pos2 {
        let c = rect.center();
        Pos2::new(
            c.x + ((p.0 - self.center.0) * self.scale) as f32,
            c.y - ((p.1 - self.center.1) * self.scale) as f32,
        )
    }

    pub fn to_world(&self, rect: Rect, s: Pos2) -> (f64, f64) {
        let c = rect.center();
        (
            self.center.0 + (s.x - c.x) as f64 / self.scale,
            self.center.1 - (s.y - c.y) as f64 / self.scale,
        )
    }

    /// Pan by a screen-space delta (pixels).
    pub fn pan(&mut self, dx: f32, dy: f32) {
        self.center.0 -= dx as f64 / self.scale;
        self.center.1 += dy as f64 / self.scale;
    }

    /// Zoom by `factor`, keeping the world point under `anchor` fixed.
    pub fn zoom_at(&mut self, rect: Rect, anchor: Pos2, factor: f64) {
        let before = self.to_world(rect, anchor);
        self.scale = (self.scale * factor).clamp(Self::MIN_SCALE, Self::MAX_SCALE);
        let after = self.to_world(rect, anchor);
        self.center.0 += before.0 - after.0;
        self.center.1 += before.1 - after.1;
    }

    /// Frame a bounding box with a margin.
    pub fn fit(&mut self, rect: Rect, lo: (f64, f64), hi: (f64, f64)) {
        self.center = ((lo.0 + hi.0) / 2.0, (lo.1 + hi.1) / 2.0);
        let (w, h) = ((hi.0 - lo.0).max(1e-9), (hi.1 - lo.1).max(1e-9));
        let s = (rect.width() as f64 * 0.8 / w).min(rect.height() as f64 * 0.8 / h);
        self.scale = if s.is_finite() {
            s.clamp(Self::MIN_SCALE, Self::MAX_SCALE)
        } else {
            20.0
        };
    }

    /// Grid spacing (1-2-5 series) giving at least `min_px` pixels between lines.
    pub fn grid_step(&self, min_px: f64) -> f64 {
        let raw = min_px / self.scale;
        let mag = 10f64.powf(raw.log10().floor());
        [1.0, 2.0, 5.0, 10.0]
            .iter()
            .map(|m| m * mag)
            .find(|s| *s >= raw)
            .unwrap_or(10.0 * mag)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui::vec2;

    fn rect() -> Rect {
        Rect::from_min_size(Pos2::new(100.0, 50.0), vec2(800.0, 600.0))
    }

    #[test]
    fn screen_world_round_trip() {
        let v = View {
            center: (3.0, -2.0),
            scale: 37.5,
        };
        for p in [(0.0, 0.0), (10.5, -4.25), (-100.0, 80.0)] {
            let q = v.to_world(rect(), v.to_screen(rect(), p));
            assert!(
                (p.0 - q.0).abs() < 1e-4 && (p.1 - q.1).abs() < 1e-4,
                "{p:?} {q:?}"
            );
        }
        // y is flipped
        assert!(v.to_screen(rect(), (3.0, 10.0)).y < v.to_screen(rect(), (3.0, -10.0)).y);
    }

    #[test]
    fn zoom_keeps_the_anchor_fixed() {
        let mut v = View::default();
        let anchor = Pos2::new(250.0, 400.0);
        let w = v.to_world(rect(), anchor);
        v.zoom_at(rect(), anchor, 3.7);
        let w2 = v.to_world(rect(), anchor);
        assert!((w.0 - w2.0).abs() < 1e-9 && (w.1 - w2.1).abs() < 1e-9);
        assert!((v.scale - 74.0).abs() < 1e-9);
    }

    #[test]
    fn pan_follows_the_cursor() {
        let mut v = View::default();
        let p = (1.0, 1.0);
        let before = v.to_screen(rect(), p);
        v.pan(30.0, -12.0);
        let after = v.to_screen(rect(), p);
        assert!(
            (after.x - before.x - 30.0).abs() < 1e-3 && (after.y - before.y + 12.0).abs() < 1e-3
        );
    }

    #[test]
    fn fit_frames_the_box() {
        let mut v = View::default();
        v.fit(rect(), (0.0, 0.0), (100.0, 50.0));
        let a = v.to_screen(rect(), (0.0, 0.0));
        let b = v.to_screen(rect(), (100.0, 50.0));
        assert!(rect().contains(a) && rect().contains(b));
        // degenerate box does not blow up
        v.fit(rect(), (1.0, 1.0), (1.0, 1.0));
        assert!(v.scale.is_finite() && v.scale <= View::MAX_SCALE);
    }

    #[test]
    fn grid_step_is_1_2_5() {
        for scale in [0.01, 0.3, 1.0, 7.0, 20.0, 333.0, 1e4] {
            let v = View {
                center: (0.0, 0.0),
                scale,
            };
            let s = v.grid_step(24.0);
            assert!(s * scale >= 24.0 - 1e-9, "{s} at {scale}");
            let m = s / 10f64.powf(s.log10().floor());
            assert!([1.0, 2.0, 5.0].iter().any(|x| (m - x).abs() < 1e-9), "{s}");
        }
    }
}
