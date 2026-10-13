//! A small uniform-grid spatial index over axis-aligned boxes.

use kw_math::Aabb;
use std::collections::HashMap;

pub struct GridIndex {
    cell: f64,
    cells: HashMap<(i64, i64, i64), Vec<u32>>,
    /// Boxes spanning too many cells are tested linearly.
    big: Vec<u32>,
}

const MAX_CELLS: i64 = 512;

impl GridIndex {
    pub fn build(boxes: &[Aabb]) -> Self {
        let n = boxes.len().max(1) as f64;
        let mean = boxes.iter().map(|b| b.extent().amax()).sum::<f64>() / n;
        let cell = (2.0 * mean).max(1e-9);
        let mut idx = GridIndex {
            cell,
            cells: HashMap::new(),
            big: Vec::new(),
        };
        for (i, b) in boxes.iter().enumerate() {
            let r = idx.range(b);
            if r.count() > MAX_CELLS {
                idx.big.push(i as u32);
                continue;
            }
            for c in r.iter() {
                idx.cells.entry(c).or_default().push(i as u32);
            }
        }
        idx
    }

    fn key(&self, x: f64) -> i64 {
        (x / self.cell).floor() as i64
    }

    fn range(&self, b: &Aabb) -> CellRange {
        CellRange {
            lo: (self.key(b.min.x), self.key(b.min.y), self.key(b.min.z)),
            hi: (self.key(b.max.x), self.key(b.max.y), self.key(b.max.z)),
        }
    }

    /// Candidate indices whose cells overlap `b` (sorted, deduplicated; a superset of the true overlaps).
    pub fn query(&self, b: &Aabb, total: usize, out: &mut Vec<u32>) {
        out.clear();
        let r = self.range(b);
        if r.count() > MAX_CELLS {
            out.extend(0..total as u32);
            return;
        }
        out.extend_from_slice(&self.big);
        for c in r.iter() {
            if let Some(l) = self.cells.get(&c) {
                out.extend_from_slice(l);
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

struct CellRange {
    lo: (i64, i64, i64),
    hi: (i64, i64, i64),
}

impl CellRange {
    fn count(&self) -> i64 {
        (self.hi.0 - self.lo.0 + 1).max(0)
            * (self.hi.1 - self.lo.1 + 1).max(0)
            * (self.hi.2 - self.lo.2 + 1).max(0)
    }
    fn iter(&self) -> impl Iterator<Item = (i64, i64, i64)> + '_ {
        (self.lo.0..=self.hi.0).flat_map(move |x| {
            (self.lo.1..=self.hi.1)
                .flat_map(move |y| (self.lo.2..=self.hi.2).map(move |z| (x, y, z)))
        })
    }
}
