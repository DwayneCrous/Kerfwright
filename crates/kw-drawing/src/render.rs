//! A sheet (or the whole set) as a [`Scene2d`].

use crate::model::{scale_label, Sheet};
use crate::scene::{Anchor, Layer, Scene2d};

impl Sheet {
    /// Border, title block and every view at its place on the sheet. `no` is 1-based.
    pub fn scene(&self, no: usize, count: usize) -> Scene2d {
        let mut s = self.format.scene(&self.title, no, count);
        for v in &self.views {
            let Some(cache) = &v.cache else { continue };
            let placed = cache.scene.placed(1.0, v.pos);
            if !v.label.is_empty() {
                if let Some((lo, hi)) = placed.bounds() {
                    let text = format!("{}   SCALE {}", v.label, scale_label(v.scale));
                    s.text(
                        Layer::Text,
                        [(lo[0] + hi[0]) / 2.0, lo[1] - 7.0],
                        3.5,
                        Anchor::Center,
                        &text,
                    );
                }
            }
            s.extend(placed);
        }
        s
    }
}
