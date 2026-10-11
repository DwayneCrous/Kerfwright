#![allow(clippy::needless_range_loop, clippy::type_complexity)]
//! The mesher on solids built and tessellated by the B-rep kernel: pole fans, thin strips and
//! face-border segments of a real tessellation, remeshed and filled with tets.

use kw_brep::{make_box, make_cylinder, make_sphere, BRep, BodyId};
use kw_math::{Frame3, Point3, Vector3};
use kw_mesh3d::*;
use kw_tess::{tessellate_body, TessOptions};
use std::collections::HashMap;

fn tess(rep: &BRep, b: BodyId, chord_tol: f64) -> SurfaceMesh {
    let t = tessellate_body(
        rep,
        b,
        &TessOptions {
            chord_tol,
            angle_tol: 15f64.to_radians(),
        },
    )
    .expect("tessellate");
    SurfaceMesh::from_tess(&t.mesh)
}

fn check(name: &str, s: &SurfaceMesh, h: f64, min_fraction: f64) -> (TetMesh, MeshReport) {
    let p = MeshParams {
        max_size: h,
        max_vertices: 200_000,
        ..Default::default()
    };
    let (m, r) = mesh_surface(s, &p).unwrap_or_else(|e| panic!("{name} h={h}: {e}"));
    let q = r.quality.unwrap();
    eprintln!(
        "{name} h={h}: {} tri surface -> {} nodes, {} tets; ≤2: {:.2} %, worst {:.2}; dihedral {:.1}°–{:.1}°; \
         splits {} (segments {}), interior {}, unfixed {}, floor {}; {:.0} ms",
        s.tris.len(),
        m.nodes.len(),
        m.tets.len(),
        100.0 * q.fraction_within_2,
        q.worst_radius_edge,
        r.min_dihedral_deg,
        r.max_dihedral_deg,
        r.surface_splits,
        r.segment_splits,
        r.interior_points,
        r.unfixed,
        r.floor_hits,
        r.millis
    );
    assert!(!r.budget_hit, "{name} h={h}: vertex budget hit");
    assert_eq!(q.inverted, 0, "{name} h={h}");
    assert!(q.min_volume > 0.0);
    assert!(
        q.fraction_within_2 >= min_fraction,
        "{name} h={h}: only {:.2} % of the tets have radius-edge ≤ 2",
        100.0 * q.fraction_within_2
    );
    // the boundary of the tets is the reported surface, which encloses the volume of the tets
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
    assert_eq!(faces.values().filter(|&&c| c == 1).count(), m.surface.len());
    let vol: f64 = m.tets.iter().map(|t| signed_volume(&m.nodes, t)).sum();
    let enclosed: f64 = m
        .surface
        .iter()
        .map(|s| {
            let (a, b, c) = (
                m.nodes[s.v[0] as usize],
                m.nodes[s.v[1] as usize],
                m.nodes[s.v[2] as usize],
            );
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum();
    assert!(
        (vol - enclosed).abs() < 1e-9 * vol.abs(),
        "{name}: volume {vol} vs enclosed {enclosed}"
    );
    (m, r)
}

#[test]
fn box_cylinder_sphere() {
    let mut rep = BRep::new();
    let bx = make_box(&mut rep, &Frame3::world(), 4.0, 2.0, 0.5).body;
    let cyl = make_cylinder(&mut rep, &Frame3::world(), 1.0, 2.0).body;
    let sph = make_sphere(&mut rep, &Frame3::world(), 1.0).body;
    for h in [0.2, 0.3, 0.5] {
        check("box", &tess(&rep, bx, 0.005), h, 0.99);
        let (m, _) = check("cylinder", &tess(&rep, cyl, 0.005), h, 0.995);
        let exact = std::f64::consts::PI * 2.0;
        let vol: f64 = m.tets.iter().map(|t| signed_volume(&m.nodes, t)).sum();
        // straight-sided boundary faces cut off a layer of about h²/(8R) of the curved surface
        let tol = 0.01 + 0.25 * h * h;
        assert!(
            (vol / exact - 1.0).abs() < tol,
            "cylinder volume {vol} vs {exact}"
        );
        let (m, _) = check("sphere", &tess(&rep, sph, 0.005), h, 0.995);
        let exact = 4.0 / 3.0 * std::f64::consts::PI;
        let vol: f64 = m.tets.iter().map(|t| signed_volume(&m.nodes, t)).sum();
        assert!(
            (vol / exact - 1.0).abs() < tol,
            "sphere volume {vol} vs {exact}"
        );
    }
}

#[test]
fn plate_with_a_hole() {
    let mut rep = BRep::new();
    let plate = make_box(&mut rep, &Frame3::world(), 4.0, 2.0, 0.5).body;
    let f = Frame3::from_origin_normal(Point3::new(0.0, 0.0, -0.5), Vector3::new(0.0, 0.0, 1.0));
    let hole = make_cylinder(&mut rep, &f, 0.4, 1.0).body;
    let ph = kw_boolean::subtract(&mut rep, plate, hole)
        .expect("subtract")
        .body;
    let s = tess(&rep, ph, 0.005);
    for h in [0.1, 0.15, 0.2, 0.4] {
        check("plate with hole", &s, h, 0.99);
    }
}

// ---------------------------------------------------------------------------------------------------------
// A quarter of a thick cylinder, cut by the kernel's Booleans: curved walls meeting symmetry planes and
// end faces at trihedral corners, meshed with a steep size grading and snapped onto the true cylinders.
// The mesher must neither cascade nor leave many bad tets whatever the size.
// ---------------------------------------------------------------------------------------------------------

mod quarter_ring {
    use super::*;
    use kw_brep::{make_box, make_cylinder, BRep};
    use kw_math::{Frame3, Point3, Vector3};
    use std::sync::Arc;

    fn ring_surface() -> SurfaceMesh {
        let (a, b, t) = (10.0, 25.0, 6.0);
        let mut rep = BRep::new();
        let f0 =
            Frame3::from_origin_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let fi =
            Frame3::from_origin_normal(Point3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, 1.0));
        let outer = make_cylinder(&mut rep, &f0, b, t).body;
        let inner = make_cylinder(&mut rep, &fi, a, t + 2.0).body;
        let ring = kw_boolean::subtract(&mut rep, outer, inner).unwrap().body;
        let fb = Frame3::from_origin_normal(
            Point3::new(0.5 * (b + 1.0), 0.5 * (b + 1.0), 0.5 * t),
            Vector3::new(0.0, 0.0, 1.0),
        );
        let quadrant = make_box(&mut rep, &fb, b + 1.0, b + 1.0, t + 2.0).body;
        let body = kw_boolean::intersect(&mut rep, ring, quadrant)
            .unwrap()
            .body;
        let tt = tessellate_body(
            &rep,
            body,
            &TessOptions {
                chord_tol: 0.002,
                angle_tol: 12f64.to_radians(),
            },
        )
        .unwrap();
        SurfaceMesh::from_tess(&tt.mesh)
    }

    /// Radial projection onto the spheres or cylinders (`radius`, tags of that wall); `dims` is 2 for
    /// cylinders about the z axis and 3 for spheres about the origin. Vertices on sharp edges between a wall
    /// and a plane stay in that plane under the projection.
    /// Radial projection onto the spheres or cylinders (`radius`, tags of that wall); `dims` is 2 for
    /// cylinders about the z axis and 3 for spheres about the origin. Vertices on sharp edges between a wall
    /// and a plane stay in that plane under the projection.
    fn radial_projector(
        dims: usize,
        walls: Vec<(f64, Vec<u32>)>,
    ) -> (kw_mesh3d::ProjectFn, kw_mesh3d::ProjectEdgeFn) {
        let project = move |walls: &[(f64, Vec<u32>)], tags: &[u32], p: [f64; 3]| {
            for (radius, wall_tags) in walls {
                if tags.iter().any(|t| wall_tags.contains(t)) {
                    let r = if dims == 2 {
                        (p[0] * p[0] + p[1] * p[1]).sqrt()
                    } else {
                        (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt()
                    };
                    let s = radius / r;
                    return if dims == 2 {
                        [p[0] * s, p[1] * s, p[2]]
                    } else {
                        [p[0] * s, p[1] * s, p[2] * s]
                    };
                }
            }
            p
        };
        let w1 = walls.clone();
        (
            Arc::new(move |t, p| project(&w1, &[t], p)),
            Arc::new(move |t1, t2, p| project(&walls, &[t1, t2], p)),
        )
    }

    fn tags_at(s: &SurfaceMesh, rad: f64) -> Vec<u32> {
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
                            ((p[0] * p[0] + p[1] * p[1]).sqrt() - rad).abs() < 1e-6 * rad
                        })
                    })
            })
            .collect()
    }

    #[test]
    fn graded_quarter_ring_with_snapped_walls() {
        let s = ring_surface();
        let (a, b) = (10.0, 25.0);
        let (ta, tb) = (tags_at(&s, a), tags_at(&s, b));
        assert_eq!((ta.len(), tb.len()), (1, 1));
        let (pf, pe) = radial_projector(2, vec![(a, ta), (b, tb)]);
        for h in [0.8, 1.0, 1.2, 1.4] {
            let p = MeshParams {
                size: Some(Arc::new(move |p: [f64; 3]| {
                    let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
                    (h + 0.3 * (r - a).max(0.0)).min(2.5 * h + 2.0)
                })),
                project: Some(pf.clone()),
                project_edge: Some(pe.clone()),
                ..Default::default()
            };
            let (m, r) = mesh_surface(&s, &p).unwrap();
            let q = r.quality.unwrap();
            eprintln!(
                "quarter ring h={h}: {} nodes {} tets, ≤2 {:.2} %, dihedral {:.1}°–{:.1}°, snapped {} vertices, floor {} unfixed {}",
                m.nodes.len(),
                m.tets.len(),
                100.0 * q.fraction_within_2,
                r.min_dihedral_deg,
                r.max_dihedral_deg,
                r.snapped,
                r.floor_hits,
                r.unfixed
            );
            assert_eq!(q.inverted, 0);
            assert!(q.fraction_within_2 > 0.99, "h={h}: {}", q.fraction_within_2);
            assert!(
                r.snapped > 20,
                "the walls are snapped to the true cylinders"
            );
            // snapped boundary vertices of the walls lie on the cylinders
            let on_bore = m
                .nodes
                .iter()
                .filter(|p| ((p[0] * p[0] + p[1] * p[1]).sqrt() - a).abs() < 1e-9 * a)
                .count();
            assert!(on_bore > 30, "{on_bore}");
        }
    }
}
