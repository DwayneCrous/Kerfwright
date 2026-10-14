use crate::{cross, dot, sub, P3};
use std::collections::HashMap;

/// A boundary triangle of a [`TetMesh`], oriented counter-clockwise seen from outside.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurfaceTri {
    pub v: [u32; 3],
    /// Identifier of the geometry face it lies on (a B-rep face id, or a box side 0–5).
    pub tag: u32,
}

#[derive(Debug, Clone, Default)]
pub struct TetMesh {
    pub nodes: Vec<P3>,
    /// Corner indices; every tet has positive volume.
    pub tets: Vec<[u32; 4]>,
    pub surface: Vec<SurfaceTri>,
}

/// Edges of a tet in the order their mid-nodes are stored in a TET10 element (nodes 4…9).
pub const TET10_EDGES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];

/// A boundary triangle of a quadratic mesh: corners then mid-nodes of edges (0,1), (1,2), (2,0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurfaceTri6 {
    pub v: [u32; 6],
    pub tag: u32,
}

#[derive(Debug, Clone, Default)]
pub struct Tet10Mesh {
    pub nodes: Vec<P3>,
    pub elems: Vec<[u32; 10]>,
    pub surface: Vec<SurfaceTri6>,
    /// Number of corner nodes (they come first in `nodes`; mid-nodes follow).
    pub n_corner: usize,
}

/// Several quadratic meshes joined into one (bodies of an assembly), with the bookkeeping to find the
/// parts again. The meshes stay unconnected: no node is shared between bodies.
#[derive(Debug, Clone, Default)]
pub struct MergedMesh {
    pub mesh: Tet10Mesh,
    /// The body (index into the input list) of every element.
    pub elem_body: Vec<u16>,
    /// The first node of every body, and one past the last node of the last body.
    pub node_start: Vec<u32>,
    /// Added to the face tags of every body, so tags of different bodies never collide.
    pub tag_offset: Vec<u32>,
}

impl MergedMesh {
    /// The body that owns node `n`.
    pub fn body_of_node(&self, n: u32) -> usize {
        self.node_start
            .partition_point(|s| *s <= n)
            .saturating_sub(1)
    }

    /// The body a (global) face tag belongs to, and the tag within that body.
    pub fn split_tag(&self, tag: u32) -> (usize, u32) {
        let b = self
            .tag_offset
            .partition_point(|s| *s <= tag)
            .saturating_sub(1);
        (b, tag - self.tag_offset[b])
    }
}

impl Tet10Mesh {
    /// Join `meshes` into one. Node and element indices of body *i* are shifted by the sizes of the bodies
    /// before it; the face tags are shifted by `tag_offset[i]` (the number of tags used by earlier bodies,
    /// so the global tag is `tag_offset[i] + local tag`). `n_corner` of the result counts the corner nodes
    /// of all bodies, which are no longer stored first.
    pub fn concat(meshes: &[Tet10Mesh]) -> MergedMesh {
        let mut base = 0u32;
        let mut offsets = Vec::with_capacity(meshes.len());
        for m in meshes {
            offsets.push(base);
            base += m.surface.iter().map(|s| s.tag + 1).max().unwrap_or(0);
        }
        Tet10Mesh::concat_with_tags(meshes, &offsets)
    }

    /// As [`concat`](Self::concat), with the tag offset of every body given (so that a tag can be made from
    /// the number of faces of the bodies before it, whether or not every face has a boundary triangle).
    pub fn concat_with_tags(meshes: &[Tet10Mesh], tag_offsets: &[u32]) -> MergedMesh {
        let mut out = MergedMesh::default();
        for (b, m) in meshes.iter().enumerate() {
            let shift = out.mesh.nodes.len() as u32;
            let tag_base = tag_offsets[b];
            out.node_start.push(shift);
            out.tag_offset.push(tag_base);
            out.mesh.nodes.extend_from_slice(&m.nodes);
            out.mesh
                .elems
                .extend(m.elems.iter().map(|e| e.map(|n| n + shift)));
            out.elem_body.extend(std::iter::repeat_n(b as u16, m.elems.len()));
            out.mesh.surface.extend(m.surface.iter().map(|s| SurfaceTri6 {
                v: s.v.map(|n| n + shift),
                tag: s.tag + tag_base,
            }));
            out.mesh.n_corner += m.n_corner;
        }
        out.node_start.push(out.mesh.nodes.len() as u32);
        out
    }
}

impl TetMesh {
    /// Promote to quadratic elements. Mid-nodes sit on the edge midpoints; `project` may move a mid-node
    /// of a boundary edge onto the true curved surface (given the tag of the face the edge lies on, the
    /// node is only moved if it lies on exactly one tag). A projection that would invert an element
    /// (non-positive Jacobian at a corner or Gauss point) is reverted for the whole element.
    pub fn to_tet10(&self, project: Option<&dyn Fn(u32, P3) -> P3>) -> Tet10Mesh {
        self.to_tet10_curved(project, None)
    }

    /// As [`to_tet10`], and mid-nodes of edges shared by exactly two faces (sharp edges, face borders) are
    /// moved by `project_edge` (given the two tags, smaller first) onto the true curve.
    pub fn to_tet10_curved(
        &self,
        project: Option<&dyn Fn(u32, P3) -> P3>,
        project_edge: Option<&dyn Fn(u32, u32, P3) -> P3>,
    ) -> Tet10Mesh {
        let mut nodes = self.nodes.clone();
        let n_corner = nodes.len();
        let mut edge_node: HashMap<(u32, u32), u32> = HashMap::new();
        let mut elems = Vec::with_capacity(self.tets.len());
        for t in &self.tets {
            let mut e = [0u32; 10];
            e[..4].copy_from_slice(t);
            for (k, (a, b)) in TET10_EDGES.iter().enumerate() {
                let (u, v) = (t[*a], t[*b]);
                let key = (u.min(v), u.max(v));
                let id = *edge_node.entry(key).or_insert_with(|| {
                    let (p, q) = (self.nodes[u as usize], self.nodes[v as usize]);
                    nodes.push([
                        0.5 * (p[0] + q[0]),
                        0.5 * (p[1] + q[1]),
                        0.5 * (p[2] + q[2]),
                    ]);
                    (nodes.len() - 1) as u32
                });
                e[4 + k] = id;
            }
            elems.push(e);
        }
        let node_of = |u: u32, v: u32| edge_node[&(u.min(v), u.max(v))];
        let surface: Vec<SurfaceTri6> = self
            .surface
            .iter()
            .map(|s| SurfaceTri6 {
                v: [
                    s.v[0],
                    s.v[1],
                    s.v[2],
                    node_of(s.v[0], s.v[1]),
                    node_of(s.v[1], s.v[2]),
                    node_of(s.v[2], s.v[0]),
                ],
                tag: s.tag,
            })
            .collect();
        let mut m = Tet10Mesh {
            nodes,
            elems,
            surface,
            n_corner,
        };
        if project.is_some() || project_edge.is_some() {
            m.project_midnodes(project, project_edge);
        }
        m
    }
}

impl Tet10Mesh {
    /// Move the mid-nodes of boundary edges onto the true surface (or, on sharp edges, the true curve);
    /// revert elements that would invert.
    fn project_midnodes(
        &mut self,
        project: Option<&dyn Fn(u32, P3) -> P3>,
        project_edge: Option<&dyn Fn(u32, u32, P3) -> P3>,
    ) {
        // the tags of the surface triangles that use a mid-node of a boundary edge
        let mut tags_of: HashMap<u32, Vec<u32>> = HashMap::new();
        for s in &self.surface {
            for k in 3..6 {
                let t = tags_of.entry(s.v[k]).or_default();
                if !t.contains(&s.tag) {
                    t.push(s.tag);
                }
            }
        }
        let mut moved: Vec<(u32, P3, P3)> = Vec::new();
        for (node, mut tags) in tags_of {
            tags.sort_unstable();
            let old = self.nodes[node as usize];
            let new = match (tags.len(), project, project_edge) {
                (1, Some(p), _) => p(tags[0], old),
                (2, _, Some(pe)) => pe(tags[0], tags[1], old),
                _ => continue,
            };
            moved.push((node, old, new));
        }
        moved.sort_by_key(|m| m.0);
        for (n, _, new) in &moved {
            self.nodes[*n as usize] = *new;
        }
        // revert on inversion: any element with a bad Jacobian puts its moved mid-nodes back
        let bad: Vec<usize> = (0..self.elems.len())
            .filter(|&i| !self.element_ok(i))
            .collect();
        if !bad.is_empty() {
            let old: HashMap<u32, P3> = moved.iter().map(|m| (m.0, m.1)).collect();
            for i in bad {
                for k in 4..10 {
                    let n = self.elems[i][k];
                    if let Some(o) = old.get(&n) {
                        self.nodes[n as usize] = *o;
                    }
                }
            }
        }
    }

    /// Positive Jacobian determinant at the corners and the 4 Gauss points?
    pub fn element_ok(&self, i: usize) -> bool {
        let e = &self.elems[i];
        let x: Vec<P3> = e.iter().map(|n| self.nodes[*n as usize]).collect();
        let pts: [[f64; 4]; 8] = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [
                0.585_410_196_624_968_5,
                0.138_196_601_125_010_5,
                0.138_196_601_125_010_5,
                0.138_196_601_125_010_5,
            ],
            [
                0.138_196_601_125_010_5,
                0.585_410_196_624_968_5,
                0.138_196_601_125_010_5,
                0.138_196_601_125_010_5,
            ],
            [
                0.138_196_601_125_010_5,
                0.138_196_601_125_010_5,
                0.585_410_196_624_968_5,
                0.138_196_601_125_010_5,
            ],
            [
                0.138_196_601_125_010_5,
                0.138_196_601_125_010_5,
                0.138_196_601_125_010_5,
                0.585_410_196_624_968_5,
            ],
        ];
        pts.iter().all(|l| jacobian_det(&x, l) > 0.0)
    }

    /// Indices of elements whose Jacobian is not positive somewhere.
    pub fn inverted_elements(&self) -> Vec<usize> {
        (0..self.elems.len())
            .filter(|&i| !self.element_ok(i))
            .collect()
    }
}

/// `det J` of the isoparametric map at barycentric point `l`.
pub fn jacobian_det(x: &[P3], l: &[f64; 4]) -> f64 {
    let dn = tet10_dn_dl(l);
    let mut j = [[0.0; 3]; 3];
    for i in 0..10 {
        for a in 0..3 {
            let d = dn[i][a + 1] - dn[i][0];
            for b in 0..3 {
                j[b][a] += x[i][b] * d;
            }
        }
    }
    dot(
        [j[0][0], j[1][0], j[2][0]],
        cross([j[0][1], j[1][1], j[2][1]], [j[0][2], j[1][2], j[2][2]]),
    )
}

/// `∂N_i/∂L_k` for the 10 shape functions of the quadratic tetrahedron.
pub fn tet10_dn_dl(l: &[f64; 4]) -> [[f64; 4]; 10] {
    let mut dn = [[0.0; 4]; 10];
    for i in 0..4 {
        dn[i][i] = 4.0 * l[i] - 1.0;
    }
    for (k, (a, b)) in TET10_EDGES.iter().enumerate() {
        dn[4 + k][*a] = 4.0 * l[*b];
        dn[4 + k][*b] = 4.0 * l[*a];
    }
    dn
}

/// Boundary triangles of a tet mesh (faces used by exactly one tet), outward oriented, tagged by `tag_of`
/// (called with the triangle's three corner coordinates).
pub fn extract_surface(
    nodes: &[P3],
    tets: &[[u32; 4]],
    tag_of: &dyn Fn(P3, P3, P3) -> u32,
) -> Vec<SurfaceTri> {
    let mut faces: HashMap<[u32; 3], (u32, [u32; 3])> = HashMap::new();
    for t in tets {
        // faces opposite each corner, wound so that the normal points away from the opposite corner
        let f = [
            [t[1], t[2], t[3]],
            [t[0], t[3], t[2]],
            [t[0], t[1], t[3]],
            [t[0], t[2], t[1]],
        ];
        for tri in f {
            let mut key = tri;
            key.sort_unstable();
            let e = faces.entry(key).or_insert((0, tri));
            e.0 += 1;
        }
    }
    let mut out: Vec<SurfaceTri> = faces
        .into_values()
        .filter(|(c, _)| *c == 1)
        .map(|(_, tri)| SurfaceTri {
            v: tri,
            tag: tag_of(
                nodes[tri[0] as usize],
                nodes[tri[1] as usize],
                nodes[tri[2] as usize],
            ),
        })
        .collect();
    out.sort_by_key(|s| {
        let mut k = s.v;
        k.sort_unstable();
        k
    });
    let _ = sub;
    out
}
