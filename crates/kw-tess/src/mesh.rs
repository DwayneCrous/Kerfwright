use kw_brep::{EdgeId, FaceId};
use nalgebra::{Point3, Vector3};
use std::collections::HashMap;

/// Triangle mesh with per-face duplicated vertices (so normals are crisp at
/// sharp edges) plus `weld` ids that identify vertices sharing one position.
#[derive(Debug, Clone, Default)]
pub struct Mesh {
    pub positions: Vec<Point3<f64>>,
    pub normals: Vec<Vector3<f64>>,
    /// Surface parameters `(u, v)` of each vertex on the surface of its face.
    pub uv: Vec<(f64, f64)>,
    /// Topological vertex id: equal ids ⇔ identical position by construction.
    pub weld: Vec<u32>,
    /// Triangle list (3 indices per triangle), counter-clockwise seen from outside.
    pub indices: Vec<u32>,
    /// For each triangle, the dense index of its face in `Tessellation::face_keys`.
    pub tri_face: Vec<u32>,
    pub(crate) next_weld: u32,
}

/// Polylines of the B-rep edges (for edge rendering and picking).
#[derive(Debug, Clone, Default)]
pub struct EdgePolylines {
    pub points: Vec<Point3<f64>>,
    /// `(dense edge index, first point, point count)`; degenerate edges are skipped.
    pub ranges: Vec<(u32, u32, u32)>,
}

#[derive(Debug, Clone)]
pub struct Tessellation {
    pub mesh: Mesh,
    pub edges: EdgePolylines,
    pub face_keys: Vec<FaceId>,
    pub edge_keys: Vec<EdgeId>,
}

impl Tessellation {
    pub(crate) fn new(
        face_keys: Vec<FaceId>,
        edges: EdgePolylines,
        edge_keys: Vec<EdgeId>,
        weld_base: u32,
    ) -> Self {
        Self {
            mesh: Mesh {
                next_weld: weld_base,
                ..Mesh::default()
            },
            edges,
            face_keys,
            edge_keys,
        }
    }
}

impl Mesh {
    pub(crate) fn append_face(&mut self, fm: crate::face::FaceMesh, face_index: u32) {
        let base = self.positions.len() as u32;
        self.positions.extend(fm.positions);
        self.normals.extend(fm.normals);
        self.uv.extend(fm.uv);
        for w in fm.weld {
            if w == crate::face::LOCAL {
                self.weld.push(self.next_weld);
                self.next_weld += 1;
            } else {
                self.weld.push(w);
            }
        }
        for t in fm.tris {
            self.indices.extend([base + t[0], base + t[1], base + t[2]]);
            self.tri_face.push(face_index);
        }
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn triangle(&self, t: usize) -> [Point3<f64>; 3] {
        let i = &self.indices[3 * t..3 * t + 3];
        [
            self.positions[i[0] as usize],
            self.positions[i[1] as usize],
            self.positions[i[2] as usize],
        ]
    }

    pub fn signed_volume(&self) -> f64 {
        signed_volume(self)
    }

    pub fn surface_area(&self) -> f64 {
        (0..self.triangle_count())
            .map(|t| {
                let [a, b, c] = self.triangle(t);
                0.5 * (b - a).cross(&(c - a)).norm()
            })
            .sum()
    }

    /// For a closed mesh every undirected edge (by weld ids) must be shared by
    /// exactly two triangles, once in each direction. Returns the first
    /// offending weld-id pair, if any.
    pub fn first_unmatched_edge(&self) -> Option<(u32, u32)> {
        let mut count: HashMap<(u32, u32), i32> = HashMap::new();
        for t in self.indices.as_chunks::<3>().0 {
            for k in 0..3 {
                let (a, b) = (self.weld[t[k] as usize], self.weld[t[(k + 1) % 3] as usize]);
                if a == b {
                    continue;
                }
                *count.entry((a, b)).or_default() += 1;
            }
        }
        for (&(a, b), &n) in &count {
            if n != 1 || count.get(&(b, a)).copied() != Some(1) {
                return Some((a, b));
            }
        }
        None
    }
}

/// Volume enclosed by a closed, outward-oriented triangle mesh (divergence theorem).
pub fn signed_volume(mesh: &Mesh) -> f64 {
    (0..mesh.triangle_count())
        .map(|t| {
            let [a, b, c] = mesh.triangle(t);
            a.coords.dot(&b.coords.cross(&c.coords)) / 6.0
        })
        .sum()
}
