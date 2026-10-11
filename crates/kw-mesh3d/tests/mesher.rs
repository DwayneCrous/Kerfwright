#![allow(clippy::needless_range_loop, clippy::type_complexity)]
//! Delaunay-refinement mesher: conformity, volume, orientation and quality on analytic bodies.

use kw_job::CancelFlag;
use kw_mesh3d::*;
use std::collections::HashMap;

fn volume(m: &TetMesh) -> f64 {
    m.tets.iter().map(|t| signed_volume(&m.nodes, t)).sum()
}

/// The boundary faces of the tets are exactly the reported surface, each used once, and every interior
/// face is shared by exactly two tets.
fn check_conforming(m: &TetMesh) {
    let mut faces: HashMap<[u32; 3], u32> = HashMap::new();
    for t in &m.tets {
        for f in [
            [t[1], t[2], t[3]],
            [t[0], t[3], t[2]],
            [t[0], t[1], t[3]],
            [t[0], t[2], t[1]],
        ] {
            let mut k = f;
            k.sort_unstable();
            *faces.entry(k).or_insert(0) += 1;
        }
    }
    assert!(faces.values().all(|&c| c <= 2));
    let boundary: usize = faces.values().filter(|&&c| c == 1).count();
    assert_eq!(
        boundary,
        m.surface.len(),
        "boundary faces vs reported surface"
    );
    for s in &m.surface {
        let mut k = s.v;
        k.sort_unstable();
        assert_eq!(
            faces.get(&k),
            Some(&1),
            "surface triangle is not a boundary face"
        );
    }
}

fn mesh(s: &SurfaceMesh, h: f64) -> (TetMesh, MeshReport) {
    let p = MeshParams {
        max_size: h,
        ..Default::default()
    };
    let (m, r) = mesh_surface(s, &p).expect("mesh");
    let q = r.quality.unwrap();
    eprintln!(
        "{} nodes ({} in), {} tets; splits {} (segments {}), interior {}, unfixed {}; ≤2: {:.2} %, worst {:.2}; dihedral {:.1}°–{:.1}°; {:.0} ms",
        m.nodes.len(),
        r.input_vertices,
        m.tets.len(),
        r.surface_splits,
        r.segment_splits,
        r.interior_points,
        r.unfixed,
        100.0 * q.fraction_within_2,
        q.worst_radius_edge,
        r.min_dihedral_deg,
        r.max_dihedral_deg,
        r.millis
    );
    check_conforming(&m);
    assert!(!r.budget_hit);
    assert_eq!(q.inverted, 0);
    assert!(q.fraction_within_2 > 0.99, "{}", q.fraction_within_2);
    (m, r)
}

#[test]
fn cuboid_is_filled_with_good_tets() {
    for (lx, ly, lz, h) in [
        (2.0, 1.0, 1.0, 0.25),
        (2.0, 1.0, 1.0, 0.3),
        (4.0, 2.0, 1.0, 0.3),
        (3.7, 1.3, 0.9, 0.3),
    ] {
        let s = SurfaceMesh::cuboid(lx, ly, lz);
        let (m, _) = mesh(&s, h);
        // a box has exactly representable faces: the volume is exact up to the snapping of the vertices
        assert!(
            (volume(&m) - lx * ly * lz).abs() < 1e-6 * lx * ly * lz,
            "{}",
            volume(&m)
        );
    }
}

#[test]
fn sphere_and_cylinder() {
    let s = SurfaceMesh::sphere([0.0; 3], 1.0, 3);
    let (m, _) = mesh(&s, 0.3);
    // the meshed volume is that of the remeshed (inscribed) surface, close to the input polyhedron
    assert!(
        (volume(&m) / s.volume() - 1.0).abs() < 0.03,
        "{} vs {}",
        volume(&m),
        s.volume()
    );
    let c = SurfaceMesh::cylinder(1.0, 3.0, 48, 8);
    let (m, _) = mesh(&c, 0.3);
    assert!(
        (volume(&m) / c.volume() - 1.0).abs() < 0.03,
        "{} vs {}",
        volume(&m),
        c.volume()
    );
}

#[test]
fn the_size_field_grades_the_mesh() {
    // fine near the origin, coarse far away
    let s = SurfaceMesh::cuboid(4.0, 4.0, 4.0);
    let p = MeshParams {
        size: Some(std::sync::Arc::new(|p: [f64; 3]| {
            let d = ((p[0] - 2.0).powi(2) + (p[1] - 2.0).powi(2) + (p[2] - 2.0).powi(2)).sqrt();
            (0.15 + 0.25 * d).min(1.2)
        })),
        ..Default::default()
    };
    let (m, r) = mesh_surface(&s, &p).unwrap();
    eprintln!(
        "graded: {} nodes {} tets, ≤2 {:.2} %",
        m.nodes.len(),
        m.tets.len(),
        100.0 * r.quality.as_ref().unwrap().fraction_within_2
    );
    check_conforming(&m);
    assert!((volume(&m) - 64.0).abs() < 1e-5);
    // many more small tets near the centre than near a corner
    let edge = |t: &[u32; 4]| {
        let a = m.nodes[t[0] as usize];
        let b = m.nodes[t[1] as usize];
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    };
    let centre: Vec<f64> = m
        .tets
        .iter()
        .filter(|t| {
            let a = m.nodes[t[0] as usize];
            (a[0] - 2.0).abs() < 0.5 && (a[1] - 2.0).abs() < 0.5 && (a[2] - 2.0).abs() < 0.5
        })
        .map(edge)
        .collect();
    let corner: Vec<f64> = m
        .tets
        .iter()
        .filter(|t| {
            let a = m.nodes[t[0] as usize];
            a[0] < 0.6 && a[1] < 0.6 && a[2] < 0.6
        })
        .map(edge)
        .collect();
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
    assert!(!centre.is_empty() && !corner.is_empty());
    assert!(
        mean(&centre) < 0.6 * mean(&corner),
        "{} vs {}",
        mean(&centre),
        mean(&corner)
    );
    assert!(r.quality.unwrap().fraction_within_2 > 0.98);
}

#[test]
fn bad_input_is_rejected() {
    // open surface: drop one triangle of a box
    let mut s = SurfaceMesh::cuboid(1.0, 1.0, 1.0);
    s.tris.pop();
    s.tags.pop();
    assert!(matches!(
        mesh_surface(&s, &MeshParams::default()),
        Err(MeshError::NotClosed(_))
    ));
    // inside-out surface
    let mut s = SurfaceMesh::cuboid(1.0, 1.0, 1.0);
    for t in &mut s.tris {
        t.swap(1, 2);
    }
    assert_eq!(
        mesh_surface(&s, &MeshParams::default()).err(),
        Some(MeshError::Inverted)
    );
}

#[test]
fn cancellation_is_honoured() {
    let c = CancelFlag::new();
    c.cancel();
    let s = SurfaceMesh::sphere([0.0; 3], 1.0, 2);
    let p = MeshParams {
        max_size: 0.05,
        cancel: Some(c),
        ..Default::default()
    };
    assert_eq!(mesh_surface(&s, &p).err(), Some(MeshError::Cancelled));
}

#[test]
fn meshing_is_deterministic() {
    let s = SurfaceMesh::sphere([0.0; 3], 1.0, 2);
    let a = mesh(&s, 0.35).0;
    let b = mesh(&s, 0.35).0;
    assert_eq!(a.tets, b.tets);
    assert_eq!(a.nodes, b.nodes);
}

#[test]
fn tet10_promotion_follows_the_true_surface() {
    // mid-nodes of boundary edges move onto the true sphere; no element inverts
    let s = SurfaceMesh::sphere([0.0; 3], 1.0, 3);
    let (m, _) = mesh(&s, 0.4);
    let project = |_tag: u32, p: [f64; 3]| {
        let n = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        [p[0] / n, p[1] / n, p[2] / n]
    };
    let q = m.to_tet10(Some(&project));
    assert!(q.inverted_elements().is_empty());
    let mut on_sphere = 0;
    for s in &q.surface {
        for k in 3..6 {
            let p = q.nodes[s.v[k] as usize];
            if ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - 1.0).abs() < 1e-9 {
                on_sphere += 1;
            }
        }
    }
    // the corner nodes sit within the remeshing tolerance of the sphere; most mid-nodes were projected
    assert!(
        on_sphere * 10 > q.surface.len() * 3 * 8,
        "{on_sphere} of {}",
        q.surface.len() * 3
    );
}
