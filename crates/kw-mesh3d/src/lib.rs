//! Volume meshes for the finite-element code: tetrahedral meshes of boxes and (with the Delaunay
//! refinement mesher) of B-rep bodies, quadratic (TET10) promotion with projection of mid-nodes onto the
//! true surface, and element quality measures.

#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

mod box_tets;
pub mod improve;
mod mesh;
mod quality;
pub mod refine;
pub mod remesh;
pub mod smooth;
pub mod snap;
pub mod surf;
pub mod tri3;

pub use box_tets::{box_tets, split_hexes_red};
pub use mesh::{
    extract_surface, jacobian_det, tet10_dn_dl, SurfaceTri, SurfaceTri6, Tet10Mesh, TetMesh,
    TET10_EDGES,
};
pub use quality::{circumradius, radius_edge_ratio, shortest_edge, signed_volume, QualityReport};
pub use refine::{
    mesh_surface, MeshError, MeshParams, MeshReport, ProjectEdgeFn, ProjectFn, SizeFn,
};
pub use surf::SurfaceMesh;

pub type P3 = [f64; 3];

#[inline]
pub(crate) fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
pub(crate) fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
#[inline]
pub(crate) fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
pub(crate) fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_mesh_fills_the_box_with_positive_tets() {
        let m = box_tets(2.0, 3.0, 4.0, 3, 2, 2);
        let vol: f64 = m.tets.iter().map(|t| signed_volume(&m.nodes, t)).sum();
        assert!((vol - 24.0).abs() < 1e-12, "{vol}");
        assert!(m.tets.iter().all(|t| signed_volume(&m.nodes, t) > 0.0));
        // boundary area: 2(2·3 + 2·4 + 3·4) = 52
        let area: f64 = m
            .surface
            .iter()
            .map(|s| {
                let (a, b, c) = (
                    m.nodes[s.v[0] as usize],
                    m.nodes[s.v[1] as usize],
                    m.nodes[s.v[2] as usize],
                );
                0.5 * norm(cross(sub(b, a), sub(c, a)))
            })
            .sum();
        assert!((area - 52.0).abs() < 1e-12, "{area}");
        // outward orientation: the signed volume of the boundary (divergence theorem) equals the volume
        let v: f64 = m
            .surface
            .iter()
            .map(|s| {
                let (a, b, c) = (
                    m.nodes[s.v[0] as usize],
                    m.nodes[s.v[1] as usize],
                    m.nodes[s.v[2] as usize],
                );
                dot(a, cross(b, c)) / 6.0
            })
            .sum();
        assert!((v - 24.0).abs() < 1e-12);
        let q = QualityReport::of(&m.nodes, &m.tets);
        assert_eq!(q.inverted, 0);
    }

    #[test]
    fn red_refinement_keeps_volume_and_conformity() {
        let m = box_tets(1.0, 1.0, 1.0, 2, 2, 2);
        let r = split_hexes_red(&m);
        assert_eq!(r.tets.len(), 8 * m.tets.len());
        let vol: f64 = r.tets.iter().map(|t| signed_volume(&r.nodes, t)).sum();
        assert!((vol - 1.0).abs() < 1e-12);
        let s2 = extract_surface(&r.nodes, &r.tets, &|_, _, _| 0);
        assert_eq!(
            s2.len(),
            r.surface.len(),
            "boundary of the refined mesh is the refined boundary"
        );
    }

    #[test]
    fn tet10_promotion_counts_and_positive_jacobians() {
        let m = box_tets(1.0, 1.0, 1.0, 2, 2, 2);
        let q = m.to_tet10(None);
        // nodes = corners + unique edges
        assert!(q.nodes.len() > m.nodes.len());
        assert_eq!(q.elems.len(), m.tets.len());
        assert!(q.inverted_elements().is_empty());
        // every mid-node is the midpoint of its two corners
        for e in &q.elems {
            for (k, (a, b)) in TET10_EDGES.iter().enumerate() {
                let (p, r, s) = (
                    q.nodes[e[*a] as usize],
                    q.nodes[e[*b] as usize],
                    q.nodes[e[4 + k] as usize],
                );
                for i in 0..3 {
                    assert!((s[i] - 0.5 * (p[i] + r[i])).abs() < 1e-14);
                }
            }
        }
    }
}
