#![allow(clippy::needless_range_loop, clippy::type_complexity)]
//! An octant of a thick spherical shell cut by the kernel's Booleans, meshed with the new vertices
//! projected onto the true spheres: regression test for a cascade at rim vertices (projected vertices lie
//! slightly off the facets of the polyhedral surface, which must not count as encroachment).

use kw_brep::{make_box, make_sphere, BRep};
use kw_math::{Frame3, Point3, Vector3};
use kw_mesh3d::*;
use kw_tess::{tessellate_body, TessOptions};
use std::sync::Arc;

#[test]
fn spherical_shell_octant_with_projected_vertices() {
    let (a, b) = (10.0, 25.0);
    let mut rep = BRep::new();
    // the seam and poles of the sphere must not lie in the cutting planes
    let f0 = Frame3::from_origin_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
    let outer = make_sphere(&mut rep, &f0, b).body;
    let inner = make_sphere(&mut rep, &f0, a).body;
    let shell = kw_boolean::subtract(&mut rep, outer, inner)
        .expect("shell")
        .body;
    let fb = Frame3::from_origin_normal(
        Point3::new(0.5 * (b + 1.0), 0.5 * (b + 1.0), 0.5 * (b + 1.0)),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let octant = make_box(&mut rep, &fb, b + 1.0, b + 1.0, b + 1.0).body;
    let body = kw_boolean::intersect(&mut rep, shell, octant)
        .expect("octant")
        .body;
    let t = tessellate_body(
        &rep,
        body,
        &TessOptions {
            chord_tol: 0.002,
            angle_tol: 12f64.to_radians(),
        },
    )
    .unwrap();
    let s = SurfaceMesh::from_tess(&t.mesh);
    s.check_closed().unwrap();
    let exact = std::f64::consts::PI / 6.0 * (b.powi(3) - a.powi(3));
    assert!((s.volume() / exact - 1.0).abs() < 1e-3);

    let on_sphere = |rad: f64| -> Vec<u32> {
        let mut tags = s.tags.clone();
        tags.sort();
        tags.dedup();
        tags.into_iter()
            .filter(|&tg| {
                s.tris
                    .iter()
                    .zip(&s.tags)
                    .filter(|(_, x)| **x == tg)
                    .all(|(tri, _)| {
                        tri.iter().all(|&v| {
                            let p = s.nodes[v as usize];
                            ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - rad).abs()
                                < 1e-6 * rad
                        })
                    })
            })
            .collect()
    };
    let (ta, tb) = (on_sphere(a), on_sphere(b));
    assert_eq!((ta.len(), tb.len()), (1, 1));
    let (ta1, tb1, ta2, tb2) = (ta.clone(), tb.clone(), ta, tb);
    let rad = move |rr: f64, p: [f64; 3]| {
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        [p[0] * rr / r, p[1] * rr / r, p[2] * rr / r]
    };
    let pf: ProjectFn = Arc::new(move |tag, p| {
        if ta1.contains(&tag) {
            rad(a, p)
        } else if tb1.contains(&tag) {
            rad(b, p)
        } else {
            p
        }
    });
    let pe: ProjectEdgeFn = Arc::new(move |t1, t2, p| {
        if ta2.contains(&t1) || ta2.contains(&t2) {
            rad(a, p)
        } else if tb2.contains(&t1) || tb2.contains(&t2) {
            rad(b, p)
        } else {
            p
        }
    });
    for h in [4.0, 3.0, 2.0] {
        let p = MeshParams {
            max_size: h,
            project: Some(pf.clone()),
            project_edge: Some(pe.clone()),
            ..Default::default()
        };
        let (m, r) = mesh_surface(&s, &p).unwrap_or_else(|e| panic!("h={h}: {e}"));
        let q = r.quality.unwrap();
        eprintln!(
            "h={h}: {} nodes {} tets, ≤2 {:.2} %",
            m.nodes.len(),
            m.tets.len(),
            100.0 * q.fraction_within_2
        );
        assert_eq!(q.inverted, 0);
        assert!(q.fraction_within_2 > 0.99);
    }
}
