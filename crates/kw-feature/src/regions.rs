//! Regions of a sketch: the faces of its planar arrangement (see `kw_sketch::arrangement`).
//!
//! Where curves cross, they are cut; every enclosed area is a region of its own, whether it lies inside another
//! shape (a hole), crosses one, or is cut off by a line. A region is named by the labels of the entities on its
//! boundary and by a point inside it ([`RegionPick`]); the solids are made from the faces themselves.

use crate::profile::{sketch_faces, FaceSet};
use crate::spec::RegionPick;
use kw_math::Frame3;
use kw_sketch::arrangement::{point_in_polygon, P2 as AP2};
use kw_sketch::{EntityId, Sketch};

pub type P2 = [f64; 2];
/// Contours of a shape: the outline and then its holes.
pub type Shape = Vec<Vec<P2>>;
pub type Shapes = Vec<Shape>;

/// A region and where it is.
#[derive(Debug, Clone, PartialEq)]
pub struct Region {
    /// How the region is named in a feature.
    pub pick: RegionPick,
    /// Labels of the entities on its boundary (sorted, distinct).
    pub labels: Vec<u32>,
    /// A point inside it.
    pub interior: P2,
    /// Its outline and holes, sampled, in sketch-plane coordinates.
    pub shapes: Shapes,
    pub area: f64,
    /// How many shapes enclose it (0 = none; the default regions are the even ones).
    pub depth: usize,
    /// The face of the arrangement.
    pub face: usize,
}

/// The regions of a sketch with the arrangement they come from.
pub struct SketchRegions {
    pub faces: FaceSet,
    pub regions: Vec<Region>,
}

/// Every region of the sketch. `entities[i]` is the entity with label `i`.
pub fn sketch_regions(sk: &Sketch, entities: &[EntityId], plane: Frame3) -> SketchRegions {
    let faces = sketch_faces(sk, entities, plane);
    let regions = (0..faces.arr.faces.len())
        .map(|i| {
            let f = &faces.arr.faces[i];
            let at = faces.arr.interior_point(f);
            let labels = faces.face_labels(i);
            let polys = faces.arr.face_polygons(f, 0.005);
            let contour = |p: &Vec<AP2>| -> Vec<P2> { p.iter().map(|q| [q.0, q.1]).collect() };
            Region {
                pick: RegionPick::Face {
                    labels: labels.clone(),
                    at: [at.0, at.1],
                },
                labels,
                interior: [at.0, at.1],
                shapes: vec![polys.iter().map(contour).collect()],
                area: f.area,
                depth: f.depth,
                face: i,
            }
        })
        .collect();
    SketchRegions { faces, regions }
}

/// Is `p` inside the region (inside its outline and outside its holes)?
pub fn region_contains(r: &Region, p: P2) -> bool {
    r.shapes.iter().any(|shape| {
        let to = |c: &Vec<P2>| -> Vec<AP2> { c.iter().map(|q| (q[0], q[1])).collect() };
        let q = (p[0], p[1]);
        shape.first().is_some_and(|o| point_in_polygon(&to(o), q))
            && !shape.iter().skip(1).any(|h| point_in_polygon(&to(h), q))
    })
}

impl SketchRegions {
    /// The innermost region containing `p`.
    pub fn region_at(&self, p: P2) -> Option<usize> {
        (0..self.regions.len())
            .filter(|i| region_contains(&self.regions[*i], p))
            .min_by(|a, b| self.regions[*a].area.total_cmp(&self.regions[*b].area))
    }

    /// The indices of the regions that are extruded when none is chosen.
    pub fn default_regions(&self) -> Vec<usize> {
        (0..self.regions.len())
            .filter(|i| self.regions[*i].depth.is_multiple_of(2))
            .collect()
    }

    /// The region a pick names, if it still exists.
    pub fn resolve(&self, pick: &RegionPick, sk: &Sketch, entities: &[EntityId]) -> Option<usize> {
        match pick {
            RegionPick::Face { labels, at } => {
                let same: Vec<usize> = (0..self.regions.len())
                    .filter(|i| self.regions[*i].labels == *labels)
                    .collect();
                match same.len() {
                    0 => None,
                    1 => Some(same[0]),
                    _ => same
                        .iter()
                        .copied()
                        .find(|i| region_contains(&self.regions[*i], *at))
                        .or_else(|| {
                            same.iter().copied().min_by(|a, b| {
                                let d = |i: &usize| {
                                    let p = self.regions[*i].interior;
                                    (p[0] - at[0]).hypot(p[1] - at[1])
                                };
                                d(a).total_cmp(&d(b))
                            })
                        }),
                }
            }
            RegionPick::Legacy(sig) => {
                let p = crate::legacy_regions::legacy_point(sk, entities, self.faces.plane, sig)?;
                self.region_at(p)
            }
        }
    }
}
