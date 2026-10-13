//! Phase 1 acceptance: tessellation is crack-free (no T-junctions) and within
//! the requested chordal tolerance on the primitive corpus.

use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::{Surface, SurfaceGeom};
use kw_tess::*;
use std::collections::HashMap;
use std::f64::consts::PI;

struct Case {
    name: &'static str,
    brep: BRep,
    body: BodyId,
    volume: f64,
}

fn corpus() -> Vec<Case> {
    let f = Frame3::from_origin_normal(Point3::new(0.5, -1.0, 2.0), Vector3::new(0.3, -0.4, 1.0));
    let mut out = Vec::new();
    let mut add = |name, volume, build: &dyn Fn(&mut BRep) -> BodyId| {
        let mut b = BRep::new();
        let body = build(&mut b);
        out.push(Case {
            name,
            brep: b,
            body,
            volume,
        });
    };
    add("box", 2.0 * 3.0 * 4.0, &|b| {
        make_box(b, &f, 2.0, 3.0, 4.0).body
    });
    add("cylinder", PI * 1.5 * 1.5 * 4.0, &|b| {
        make_cylinder(b, &f, 1.5, 4.0).body
    });
    add("cone", PI * 2.0 * 2.0 * 3.0 / 3.0, &|b| {
        make_cone(b, &f, 2.0, 0.0, 3.0).body
    });
    add(
        "frustum",
        PI * 3.0 / 3.0 * (2.0 * 2.0 + 2.0 * 1.0 + 1.0 * 1.0),
        &|b| make_cone(b, &f, 2.0, 1.0, 3.0).body,
    );
    add(
        "flaring frustum",
        PI * 2.0 / 3.0 * (1.0 + 1.0 * 2.5 + 2.5 * 2.5),
        &|b| make_cone(b, &f, 1.0, 2.5, 2.0).body,
    );
    add("sphere", 4.0 / 3.0 * PI * 8.0, &|b| {
        make_sphere(b, &f, 2.0).body
    });
    add("torus", 2.0 * PI * PI * 5.0 * 1.0, &|b| {
        make_torus(b, &f, 5.0, 1.0).body
    });
    out
}

fn dist_to_surface(s: &SurfaceGeom, p: &Point3<f64>) -> f64 {
    match s {
        SurfaceGeom::Plane(pl) => (p - pl.frame.origin).dot(&pl.frame.z).abs(),
        SurfaceGeom::Cylinder(c) => {
            let d = p - c.frame.origin;
            ((d - c.frame.z * d.dot(&c.frame.z)).norm() - c.radius).abs()
        }
        SurfaceGeom::Cone(c) => {
            let d = p - c.frame.origin;
            let z = d.dot(&c.frame.z);
            let rho = (d - c.frame.z * z).norm();
            (rho - (c.radius_at_origin + z * c.half_angle.tan())).abs() * c.half_angle.cos()
        }
        SurfaceGeom::Sphere(sp) => ((p - sp.frame.origin).norm() - sp.radius).abs(),
        SurfaceGeom::Torus(t) => {
            let d = p - t.frame.origin;
            let z = d.dot(&t.frame.z);
            let rho = (d - t.frame.z * z).norm();
            (((rho - t.major_radius).powi(2) + z * z).sqrt() - t.minor_radius).abs()
        }
        other => panic!("no oracle for {other:?}"),
    }
}

fn max_deviation(c: &Case, t: &Tessellation) -> f64 {
    let bary = [
        (1.0 / 3.0, 1.0 / 3.0),
        (0.5, 0.5),
        (0.5, 0.0),
        (0.0, 0.5),
        (0.25, 0.25),
        (0.6, 0.2),
        (0.2, 0.6),
        (0.1, 0.1),
    ];
    let mut worst = 0.0f64;
    for ti in 0..t.mesh.triangle_count() {
        let [a, b, cc] = t.mesh.triangle(ti);
        let face = t.face_keys[t.mesh.tri_face[ti] as usize];
        let surf = &c.brep.surfaces[c.brep.faces[face].surface];
        for (u, v) in bary {
            let p = a + (b - a) * u + (cc - a) * v;
            worst = worst.max(dist_to_surface(surf, &p));
        }
        // circumcentre: where a flat triangle inscribed in a curved surface
        // deviates most (when it lies inside the triangle)
        let (ab, ac) = (b - a, cc - a);
        let n = ab.cross(&ac);
        let n2 = n.norm_squared();
        if n2 > 1e-24 {
            let cc_off =
                (n.cross(&ab) * ac.norm_squared() + ac.cross(&n) * ab.norm_squared()) / (2.0 * n2);
            let p = a + cc_off;
            let bary_inside = {
                let d00 = ab.dot(&ab);
                let d01 = ab.dot(&ac);
                let d11 = ac.dot(&ac);
                let d20 = cc_off.dot(&ab);
                let d21 = cc_off.dot(&ac);
                let den = d00 * d11 - d01 * d01;
                let bv = (d11 * d20 - d01 * d21) / den;
                let bw = (d00 * d21 - d01 * d20) / den;
                bv >= 0.0 && bw >= 0.0 && bv + bw <= 1.0
            };
            if bary_inside {
                worst = worst.max(dist_to_surface(surf, &p));
            }
        }
    }
    worst
}

/// Independent T-junction test (does not use weld ids): quantise positions,
/// require every undirected mesh edge to be shared by exactly two triangles
/// with opposite directions, and require that no vertex lies strictly inside
/// another triangle's edge.
fn assert_crack_free(name: &str, t: &Tessellation) {
    let q = |p: &Point3<f64>| {
        (
            (p.x * 1e9).round() as i64,
            (p.y * 1e9).round() as i64,
            (p.z * 1e9).round() as i64,
        )
    };
    let mut ids: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut verts: Vec<Point3<f64>> = Vec::new();
    let vid: Vec<u32> = t
        .mesh
        .positions
        .iter()
        .map(|p| {
            *ids.entry(q(p)).or_insert_with(|| {
                verts.push(*p);
                (verts.len() - 1) as u32
            })
        })
        .collect();
    let mut dir: HashMap<(u32, u32), i32> = HashMap::new();
    for tri in t.mesh.indices.as_chunks::<3>().0 {
        for k in 0..3 {
            let (a, b) = (vid[tri[k] as usize], vid[tri[(k + 1) % 3] as usize]);
            if a != b {
                *dir.entry((a, b)).or_default() += 1;
            }
        }
    }
    for (&(a, b), &n) in &dir {
        assert_eq!(n, 1, "{name}: directed edge {a}->{b} used {n} times");
        assert_eq!(
            dir.get(&(b, a)).copied(),
            Some(1),
            "{name}: edge {a}->{b} has no unique opposite (crack)"
        );
    }
    // vertex-on-edge (T-junction) test using a uniform grid over vertices
    let bb = kw_math::Aabb::from_points(verts.iter());
    let n_cells = ((verts.len() as f64).cbrt().ceil() as usize).clamp(4, 64);
    let cell = |p: &Point3<f64>| {
        let e = bb.extent();
        let f = |x: f64, lo: f64, ext: f64| {
            (((x - lo) / ext.max(1e-12)) * n_cells as f64)
                .floor()
                .clamp(0.0, n_cells as f64 - 1.0) as i64
        };
        (
            f(p.x, bb.min.x, e.x),
            f(p.y, bb.min.y, e.y),
            f(p.z, bb.min.z, e.z),
        )
    };
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for (i, p) in verts.iter().enumerate() {
        grid.entry(cell(p)).or_default().push(i as u32);
    }
    for &(a, b) in dir.keys() {
        if a > b {
            continue;
        }
        let (pa, pb) = (verts[a as usize], verts[b as usize]);
        let (lo, hi) = (
            cell(&Point3::new(pa.x.min(pb.x), pa.y.min(pb.y), pa.z.min(pb.z))),
            cell(&Point3::new(pa.x.max(pb.x), pa.y.max(pb.y), pa.z.max(pb.z))),
        );
        let ab = pb - pa;
        let l2 = ab.norm_squared();
        for x in lo.0..=hi.0 {
            for y in lo.1..=hi.1 {
                for z in lo.2..=hi.2 {
                    if let Some(list) = grid.get(&(x, y, z)) {
                        for &c in list {
                            if c == a || c == b {
                                continue;
                            }
                            let s = (verts[c as usize] - pa).dot(&ab) / l2;
                            if s > 1e-9 && s < 1.0 - 1e-9 {
                                let d = (verts[c as usize] - (pa + ab * s)).norm();
                                assert!(
                                    d > 1e-7,
                                    "{name}: T-junction, vertex {c} lies on edge {a}-{b} (d={d})"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn primitives_are_crack_free_and_within_chordal_tolerance() {
    for tol in [0.1, 0.01, 0.001] {
        let opts = TessOptions {
            chord_tol: tol,
            ..TessOptions::default()
        };
        for c in corpus() {
            let t = tessellate_body(&c.brep, c.body, &opts)
                .unwrap_or_else(|e| panic!("{}: {e}", c.name));
            assert!(t.mesh.triangle_count() > 0, "{}", c.name);
            assert!(
                t.mesh.first_unmatched_edge().is_none(),
                "{} @ {tol}: weld-id edge mismatch",
                c.name
            );
            assert_crack_free(&format!("{} @ {tol}", c.name), &t);
            let dev = max_deviation(&c, &t);
            assert!(
                dev <= tol * (1.0 + 1e-9),
                "{} @ {tol}: deviation {dev}",
                c.name
            );
            let vol = t.mesh.signed_volume();
            assert!(vol > 0.0, "{} @ {tol}: volume {vol}", c.name);
            // inscribed mesh volume error is bounded by (surface area) * tol
            let area = t.mesh.surface_area();
            assert!(
                (vol - c.volume).abs() <= area * tol,
                "{} @ {tol}: volume {vol} vs {} (area {area})",
                c.name,
                c.volume
            );
        }
    }
}

#[test]
fn vertex_normals_are_unit_and_outward() {
    let opts = TessOptions {
        chord_tol: 0.01,
        ..TessOptions::default()
    };
    for c in corpus() {
        let t = tessellate_body(&c.brep, c.body, &opts).unwrap();
        for (i, n) in t.mesh.normals.iter().enumerate() {
            assert!(
                (n.norm() - 1.0).abs() < 1e-9,
                "{}: normal {i} not unit",
                c.name
            );
        }
        // geometric face normal agrees with the analytic vertex normals
        for ti in 0..t.mesh.triangle_count() {
            let [a, b, cc] = t.mesh.triangle(ti);
            let gn = (b - a).cross(&(cc - a));
            if gn.norm() < 1e-14 {
                continue;
            }
            let idx = &t.mesh.indices[3 * ti..3 * ti + 3];
            let avg = t.mesh.normals[idx[0] as usize]
                + t.mesh.normals[idx[1] as usize]
                + t.mesh.normals[idx[2] as usize];
            assert!(
                gn.normalize().dot(&avg.normalize()) > 0.0,
                "{}: triangle {ti} winds against its normals",
                c.name
            );
        }
    }
}

#[test]
fn full_validation_passes_on_corpus() {
    let opts = TessOptions {
        chord_tol: 0.01,
        ..TessOptions::default()
    };
    for c in corpus() {
        validate_solid_full(&c.brep, c.body, &opts)
            .unwrap_or_else(|e| panic!("{}: {:?}", c.name, e));
    }
}

#[test]
fn flipped_face_is_reported_as_invalid() {
    let opts = TessOptions::default();
    let mut c = corpus().remove(0);
    let f = c.brep.faces.keys().next().unwrap();
    c.brep.faces[f].sense = c.brep.faces[f].sense.flip();
    assert!(validate_solid_full(&c.brep, c.body, &opts).is_err());
}

#[test]
fn edge_polylines_follow_edges() {
    let c = corpus().remove(1); // cylinder
    let t = tessellate_body(&c.brep, c.body, &TessOptions::default()).unwrap();
    assert_eq!(t.edges.ranges.len(), 3);
    for (ei, start, count) in &t.edges.ranges {
        let e = &c.brep.edges[t.edge_keys[*ei as usize]];
        let first = t.edges.points[*start as usize];
        let last = t.edges.points[(*start + *count - 1) as usize];
        assert_eq!(first, c.brep.vertices[e.start].point);
        assert_eq!(last, c.brep.vertices[e.end].point);
    }
}

#[test]
fn finer_tolerance_gives_more_triangles() {
    let c = &corpus()[5]; // sphere
    let n = |tol| {
        tessellate_body(
            &c.brep,
            c.body,
            &TessOptions {
                chord_tol: tol,
                ..TessOptions::default()
            },
        )
        .unwrap()
        .mesh
        .triangle_count()
    };
    assert!(n(0.001) > n(0.01) && n(0.01) > n(0.1));
}

#[allow(dead_code)]
fn unused(_: &dyn Surface) {}
