#![allow(clippy::needless_range_loop)]
//! Accuracy of the TET10 solver against closed-form elasticity solutions (see `kw_fem::reference`).
//! Each benchmark applies the exact boundary data of the analytic solution (displacements or tractions),
//! so the difference to the closed form is purely the discretisation error of the quadratic tets.

use kw_brep::{make_box, make_cylinder, make_sphere, BRep};
use kw_fem::reference::*;
use kw_fem::stress::nodal_stress;
use kw_fem::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_mesh3d::{box_tets, mesh_surface, MeshParams, SurfaceMesh, Tet10Mesh, TetMesh};
use kw_tess::{tessellate_body, TessOptions};
use std::sync::Arc;

const E: f64 = 210e3;
const NU: f64 = 0.3;

fn steel() -> Material {
    Material {
        e: E,
        nu: NU,
        rho: 7.8e-9,
    }
}

/// The tag whose surface triangles all lie in the plane `x[axis] = value`.
fn plane_tag(m: &TetMesh, axis: usize, value: f64) -> Option<u32> {
    let mut tags: Vec<u32> = m.surface.iter().map(|s| s.tag).collect();
    tags.sort_unstable();
    tags.dedup();
    tags.into_iter().find(|&t| {
        m.surface.iter().filter(|s| s.tag == t).all(|s| {
            s.v.iter()
                .all(|&v| (m.nodes[v as usize][axis] - value).abs() < 1e-9)
        })
    })
}

fn report_mesh(name: &str, m: &TetMesh, q: &Tet10Mesh) {
    let qr = kw_mesh3d::QualityReport::of(&m.nodes, &m.tets);
    eprintln!(
        "{name}: {} tets, {} nodes ({} DOF); radius-edge ≤ 2 for {:.2} % of the tets, worst {:.2}, inverted {}, TET10 inverted {}",
        m.tets.len(),
        q.nodes.len(),
        3 * q.nodes.len(),
        100.0 * qr.fraction_within_2,
        qr.worst_radius_edge,
        qr.inverted,
        q.inverted_elements().len()
    );
    assert_eq!(qr.inverted, 0);
    assert!(q.inverted_elements().is_empty());
    assert!(
        qr.fraction_within_2 >= 0.99,
        "{name}: mesh quality {}",
        qr.fraction_within_2
    );
}

// ---------------------------------------------------------------------------------------------------
// Cantilever (Timoshenko & Goodier)
// ---------------------------------------------------------------------------------------------------

fn cantilever(nx: usize, ny: usize, nz: usize) -> (f64, f64, usize) {
    let (l, h, t, p) = (10.0, 2.0, 1.0, 3000.0);
    let exact = Cantilever::plane_strain(l, h, p, E, NU);
    let mut mesh = box_tets(l, h, t, nx, ny, nz);
    for n in &mut mesh.nodes {
        n[1] -= 0.5 * h;
    }
    let q = mesh.to_tet10(None);
    report_mesh("cantilever", &mesh, &q);
    let mut model = Model::new(q, steel());
    let n = model.mesh.nodes.len();
    // plane strain: no displacement along z anywhere; the clamped section carries the exact (u, v)
    for i in 0..n {
        model.fixed.push(Fixed {
            node: i as u32,
            axis: 2,
            value: 0.0,
        });
        let x = model.mesh.nodes[i];
        if x[0].abs() < 1e-9 {
            let (u, v) = exact.displacement(x[0], x[1]);
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 0,
                value: u,
            });
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 1,
                value: v,
            });
        }
    }
    // the exact traction on the free end: τxy(L, y) in y (σxx vanishes there)
    let free_end = plane_tag(&mesh, 0, l).expect("free end face");
    model.traction_fn.push((
        free_end,
        TractionFn(Arc::new(move |x, _n| {
            let (sxx, _, txy) = exact.stress(x[0], x[1]);
            [sxx, txy, 0.0]
        })),
    ));
    let sol = model.solve(SolverKind::Direct).unwrap();
    // tip deflection at the centroid line of the free end, and the whole-field error
    let mut tip = Vec::new();
    let (mut num, mut den) = (0.0f64, 0.0f64);
    for i in 0..n {
        let x = model.mesh.nodes[i];
        if (x[0] - l).abs() < 1e-9 && x[1].abs() < 1e-9 {
            tip.push(sol.u[3 * i + 1]);
        }
        let (u, v) = exact.displacement(x[0], x[1]);
        num += (sol.u[3 * i] - u).powi(2) + (sol.u[3 * i + 1] - v).powi(2);
        den += u * u + v * v;
    }
    let tip_fe = tip.iter().sum::<f64>() / tip.len() as f64;
    let tip_ex = exact.tip_deflection();
    let err = (tip_fe / tip_ex - 1.0).abs();
    eprintln!(
        "cantilever {nx}x{ny}x{nz}: tip deflection FE {tip_fe:.6} vs exact {tip_ex:.6} ({:+.3} %), displacement field error {:.3} %",
        100.0 * (tip_fe / tip_ex - 1.0),
        100.0 * (num / den).sqrt()
    );
    (err, (num / den).sqrt(), n)
}

#[test]
fn cantilever_tip_deflection_within_two_percent() {
    let (err, field, _) = cantilever(10, 3, 1);
    assert!(err < 0.02, "tip deflection error {err}");
    assert!(field < 0.02, "field error {field}");
    let (err2, _, _) = cantilever(20, 6, 1);
    assert!(err2 < 0.005, "refined tip deflection error {err2}");
}

// ---------------------------------------------------------------------------------------------------
// Kirsch plate with a circular hole
// ---------------------------------------------------------------------------------------------------

/// Radial projection (in xy) onto the circle of radius `a` for the tags of the hole.
fn hole_projector(a: f64, hole_tags: Vec<u32>) -> (kw_mesh3d::ProjectFn, kw_mesh3d::ProjectEdgeFn) {
    let radial = move |p: [f64; 3]| {
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        [p[0] * a / r, p[1] * a / r, p[2]]
    };
    let tags1 = hole_tags.clone();
    let tags2 = hole_tags;
    (
        Arc::new(move |tag, p| if tags1.contains(&tag) { radial(p) } else { p }),
        Arc::new(move |t1, t2, p| {
            if tags2.contains(&t1) || tags2.contains(&t2) {
                radial(p)
            } else {
                p
            }
        }),
    )
}

/// Tags of the surface triangles whose vertices all lie on the circle of radius `a` about the z axis.
fn tags_on_cylinder(s: &SurfaceMesh, a: f64) -> Vec<u32> {
    let mut tags: Vec<u32> = s.tags.clone();
    tags.sort_unstable();
    tags.dedup();
    tags.into_iter()
        .filter(|&t| {
            s.tris
                .iter()
                .zip(&s.tags)
                .filter(|(_, tt)| **tt == t)
                .all(|(tri, _)| {
                    tri.iter().all(|&v| {
                        let p = s.nodes[v as usize];
                        ((p[0] * p[0] + p[1] * p[1]).sqrt() - a).abs() < 1e-6 * a
                    })
                })
        })
        .collect()
}

fn tessellate(rep: &BRep, body: kw_brep::BodyId) -> SurfaceMesh {
    let t = tessellate_body(
        rep,
        body,
        &TessOptions {
            chord_tol: 0.002,
            angle_tol: 12f64.to_radians(),
        },
    )
    .expect("tessellate");
    SurfaceMesh::from_tess(&t.mesh)
}

/// Quarter plate `[0,b]² × [0,t]` with a hole of radius `a` about the z axis.
fn quarter_plate(a: f64, b: f64, t: f64) -> (BRep, kw_brep::BodyId) {
    let mut rep = BRep::new();
    let f = Frame3::from_origin_normal(
        Point3::new(0.5 * b, 0.5 * b, 0.5 * t),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let plate = make_box(&mut rep, &f, b, b, t).body;
    let fc = Frame3::from_origin_normal(Point3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, 1.0));
    let hole = make_cylinder(&mut rep, &fc, a, t + 2.0).body;
    let body = kw_boolean::subtract(&mut rep, plate, hole)
        .expect("plate with hole")
        .body;
    (rep, body)
}

fn kirsch_plate(h0: f64) -> (f64, f64) {
    let (a, b, t, sigma) = (1.0, 5.0, 0.5, 100.0);
    let (rep, body) = quarter_plate(a, b, t);
    let surf = tessellate(&rep, body);
    let hole_tags = tags_on_cylinder(&surf, a);
    assert_eq!(hole_tags.len(), 1, "hole faces");
    let (pf, pe) = hole_projector(a, hole_tags);
    let params = MeshParams {
        // fine at the hole, coarsening away from it
        size: Some(Arc::new(move |p: [f64; 3]| {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            (h0 + 0.35 * (r - a).max(0.0)).min(1.2)
        })),
        project: Some(pf.clone()),
        project_edge: Some(pe.clone()),
        ..Default::default()
    };
    let (mesh, rep_m) = mesh_surface(&surf, &params).expect("mesh");
    let q = mesh.to_tet10_curved(Some(&|t, p| pf(t, p)), Some(&|t1, t2, p| pe(t1, t2, p)));
    report_mesh("Kirsch quarter plate", &mesh, &q);
    eprintln!(
        "   meshing: {:.0} ms, {} surface splits, {} interior points",
        rep_m.millis, rep_m.surface_splits, rep_m.interior_points
    );
    let tag_xmax = plane_tag(&mesh, 0, b).expect("x = b face");
    let tag_ymax = plane_tag(&mesh, 1, b).expect("y = b face");
    let mut model = Model::new(q, steel());
    for i in 0..model.mesh.nodes.len() {
        let x = model.mesh.nodes[i];
        model.fixed.push(Fixed {
            node: i as u32,
            axis: 2,
            value: 0.0,
        });
        if x[0].abs() < 1e-9 {
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 0,
                value: 0.0,
            });
        }
        if x[1].abs() < 1e-9 {
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 1,
                value: 0.0,
            });
        }
    }
    let kirsch_traction = move |x: [f64; 3], n: [f64; 3]| {
        let (sxx, syy, txy) = kirsch(a, sigma, x[0], x[1]);
        [sxx * n[0] + txy * n[1], txy * n[0] + syy * n[1], 0.0]
    };
    for tag in [tag_xmax, tag_ymax] {
        model
            .traction_fn
            .push((tag, TractionFn(Arc::new(kirsch_traction))));
    }
    let sol = model.solve(SolverKind::Direct).unwrap();
    let nodal = nodal_stress(&model, &sol.u);
    // peak hoop stress at the top of the hole: nodes on the plane x = 0 at radius a
    let mut peak = Vec::new();
    let mut worst_line = 0.0f64;
    for (i, x) in model.mesh.nodes.iter().enumerate() {
        let r = (x[0] * x[0] + x[1] * x[1]).sqrt();
        if x[0].abs() < 1e-9 && (r - a).abs() < 1e-6 {
            peak.push(nodal[i][0]);
        }
        // along the symmetry line x = 0 between the hole and 3a: σxx(0, y) against Kirsch
        if x[0].abs() < 1e-9 && r > a * (1.0 - 1e-9) && r < 3.0 * a {
            let (sxx, _, _) = kirsch(a, sigma, x[0], x[1]);
            worst_line = worst_line.max(((nodal[i][0] - sxx) / (3.0 * sigma)).abs());
        }
    }
    let peak_fe = peak.iter().sum::<f64>() / peak.len() as f64;
    eprintln!(
        "Kirsch h0={h0}: peak hoop stress FE {peak_fe:.3} vs {:.3} ({:+.3} %); worst deviation along the symmetry line {:.3} % of the peak ({} nodes at the hole top)",
        3.0 * sigma,
        100.0 * (peak_fe / (3.0 * sigma) - 1.0),
        100.0 * worst_line,
        peak.len()
    );
    ((peak_fe / (3.0 * sigma) - 1.0).abs(), worst_line)
}

#[test]
fn kirsch_plate_peak_stress_within_two_percent() {
    let (err, line) = kirsch_plate(0.14);
    assert!(err < 0.02, "peak hoop stress error {err}");
    assert!(line < 0.05, "symmetry-line stress deviation {line}");
}

/// Number of distinct geometry faces (tags) a node lies on: 0 for volume nodes, 1 inside a face, ≥ 2 on
/// an edge or at a corner of the body. Recovered stresses are least accurate at the latter, where the
/// projection has no neighbours on one side in two directions.
fn face_counts(mesh: &Tet10Mesh) -> Vec<usize> {
    let mut tags: Vec<Vec<u32>> = vec![Vec::new(); mesh.nodes.len()];
    for s in &mesh.surface {
        for &v in &s.v {
            if !tags[v as usize].contains(&s.tag) {
                tags[v as usize].push(s.tag);
            }
        }
    }
    tags.iter().map(|t| t.len()).collect()
}

// ---------------------------------------------------------------------------------------------------
// Lamé thick-walled cylinder and sphere under internal pressure
// ---------------------------------------------------------------------------------------------------

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

fn tags_on_sphere(s: &SurfaceMesh, a: f64) -> Vec<u32> {
    let mut tags: Vec<u32> = s.tags.clone();
    tags.sort_unstable();
    tags.dedup();
    tags.into_iter()
        .filter(|&t| {
            s.tris
                .iter()
                .zip(&s.tags)
                .filter(|(_, tt)| **tt == t)
                .all(|(tri, _)| {
                    tri.iter().all(|&v| {
                        let p = s.nodes[v as usize];
                        ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - a).abs() < 1e-6 * a
                    })
                })
        })
        .collect()
}

fn lame_cylinder_case(h: f64) -> (f64, f64) {
    let (a, b, t, p) = (10.0, 25.0, 6.0, 50.0);
    let mut rep = BRep::new();
    let f0 = Frame3::from_origin_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let fi = Frame3::from_origin_normal(Point3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, 1.0));
    let outer = make_cylinder(&mut rep, &f0, b, t).body;
    let inner = make_cylinder(&mut rep, &fi, a, t + 2.0).body;
    let ring = kw_boolean::subtract(&mut rep, outer, inner)
        .expect("ring")
        .body;
    let fb = Frame3::from_origin_normal(
        Point3::new(0.5 * (b + 1.0), 0.5 * (b + 1.0), 0.5 * t),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let quadrant = make_box(&mut rep, &fb, b + 1.0, b + 1.0, t + 2.0).body;
    let body = kw_boolean::intersect(&mut rep, ring, quadrant)
        .expect("quarter ring")
        .body;
    let surf = tessellate(&rep, body);
    let (ta, tb) = (tags_on_cylinder(&surf, a), tags_on_cylinder(&surf, b));
    assert_eq!((ta.len(), tb.len()), (1, 1));
    eprintln!(
        "   surface {} nodes {} tris; bore tag {:?} outer tag {:?}",
        surf.nodes.len(),
        surf.tris.len(),
        ta,
        tb
    );
    let (pf, pe) = radial_projector(2, vec![(a, ta.clone()), (b, tb)]);
    let params = MeshParams {
        // fine at the bore, where the hoop stress varies steeply, coarser towards the outside
        size: Some(Arc::new(move |p: [f64; 3]| {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            (h + 0.15 * (r - a).max(0.0)).min(2.5 * h + 2.0)
        })),
        project: Some(pf.clone()),
        project_edge: Some(pe.clone()),
        ..Default::default()
    };
    let (mesh, mrep) = mesh_surface(&surf, &params).expect("mesh");
    eprintln!(
        "   mesher (h={h}): {} surface splits ({} segment), {} interior points, floor {}, unfixed {}, degenerate {}, failed {}, flips {} + {}, dihedral {:.1}°–{:.1}°, {:.0} ms",
        mrep.surface_splits, mrep.segment_splits, mrep.interior_points, mrep.floor_hits, mrep.unfixed, mrep.degenerate,
        mrep.failed_inserts, mrep.flips, mrep.flips_volume, mrep.min_dihedral_deg, mrep.max_dihedral_deg, mrep.millis
    );
    let q = mesh.to_tet10_curved(Some(&|t, p| pf(t, p)), Some(&|t1, t2, p| pe(t1, t2, p)));
    report_mesh("Lamé cylinder", &mesh, &q);
    let mut model = Model::new(q, steel());
    for i in 0..model.mesh.nodes.len() {
        let x = model.mesh.nodes[i];
        model.fixed.push(Fixed {
            node: i as u32,
            axis: 2,
            value: 0.0,
        });
        if x[0].abs() < 1e-9 {
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 0,
                value: 0.0,
            });
        }
        if x[1].abs() < 1e-9 {
            model.fixed.push(Fixed {
                node: i as u32,
                axis: 1,
                value: 0.0,
            });
        }
    }
    model.pressure.push((ta[0], p));
    let sol = model.solve(SolverKind::Direct).unwrap();
    let nodal = nodal_stress(&model, &sol.u);
    let (mut worst_u, mut worst_s, mut worst_edge) = (0.0f64, 0.0f64, 0.0f64);
    let faces = face_counts(&model.mesh);
    let mut sample = 0;
    let mut worst_loc = (0.0, 0.0, 0.0);
    let mut bore: Vec<f64> = Vec::new();
    for (i, x) in model.mesh.nodes.iter().enumerate() {
        let r = (x[0] * x[0] + x[1] * x[1]).sqrt();
        if r < a * (1.0 - 1e-9) {
            continue;
        }
        let (c, s) = (x[0] / r, x[1] / r);
        let ur = sol.u[3 * i] * c + sol.u[3 * i + 1] * s;
        let st = nodal[i][0] * s * s + nodal[i][1] * c * c - 2.0 * nodal[i][5] * s * c;
        let (_, st_ex, _, ur_ex) = lame_cylinder(a, b, p, E, NU, r);
        worst_u = worst_u.max((ur / ur_ex - 1.0).abs());
        // normalise stress deviations by the peak hoop stress at the bore
        let st_peak = lame_cylinder(a, b, p, E, NU, a).1;
        let dev = ((st - st_ex) / st_peak).abs();
        if faces[i] >= 2 {
            worst_edge = worst_edge.max(dev);
        } else {
            if dev > worst_s {
                worst_loc = (r, x[0].atan2(x[1]).to_degrees().abs(), x[2]);
            }
            worst_s = worst_s.max(dev);
        }
        if (r - a).abs() < 1e-6 {
            bore.push(st / st_ex - 1.0);
        }
        sample += 1;
    }
    // raw (unrecovered) element stress at the integration points
    {
        let d = kw_fem::elem::elasticity(E, NU);
        let rule = kw_fem::elem::duffy(3);
        let (mut worst, mut sum2, mut cnt) = (0.0f64, 0.0f64, 0usize);
        let st_peak = lame_cylinder(a, b, p, E, NU, a).1;
        for e in &model.mesh.elems {
            let mut xe = [[0.0; 3]; 10];
            let mut ue = [0.0; 30];
            for i in 0..10 {
                xe[i] = model.mesh.nodes[e[i] as usize];
                for k in 0..3 {
                    ue[3 * i + k] = sol.u[3 * e[i] as usize + k];
                }
            }
            for l in &rule.points {
                let nn = kw_fem::elem::shape(l);
                let mut xq = [0.0; 3];
                for i in 0..10 {
                    for k in 0..3 {
                        xq[k] += nn[i] * xe[i][k];
                    }
                }
                let s = kw_fem::elem::stress_at(&xe, &ue, &d, l);
                let r = (xq[0] * xq[0] + xq[1] * xq[1]).sqrt();
                let (c, sn) = (xq[0] / r, xq[1] / r);
                let st = s[0] * sn * sn + s[1] * c * c - 2.0 * s[5] * sn * c;
                let dev = ((st - lame_cylinder(a, b, p, E, NU, r).1) / st_peak).abs();
                worst = worst.max(dev);
                sum2 += dev * dev;
                cnt += 1;
            }
        }
        eprintln!(
            "   raw Gauss-point hoop stress: max deviation {:.3} %, rms {:.3} % of the bore value",
            100.0 * worst,
            100.0 * (sum2 / cnt as f64).sqrt()
        );
    }
    eprintln!("   worst hoop deviation at r={:.2} (bore {a}, outer {b}), angle from y axis {:.0}°, z {:.2}; bore nodes: mean {:+.3} % max |.| {:.3} %", worst_loc.0, worst_loc.1, worst_loc.2,
        100.0 * bore.iter().sum::<f64>() / bore.len().max(1) as f64, 100.0 * bore.iter().fold(0.0f64, |m, v| m.max(v.abs())));
    eprintln!(
        "Lamé cylinder h={h}: worst radial displacement error {:.3} %, worst hoop stress deviation {:.3} % of the bore value at volume and face-interior nodes ({:.3} % at edge nodes; {sample} nodes)",
        100.0 * worst_u,
        100.0 * worst_s,
        100.0 * worst_edge
    );
    (worst_u, worst_s)
}

#[test]
fn lame_cylinder_within_two_percent() {
    let (u, s) = lame_cylinder_case(1.0);
    assert!(u < 0.02, "radial displacement error {u}");
    assert!(s < 0.02, "hoop stress deviation {s}");
}

fn lame_sphere_case(h: f64) -> (f64, f64) {
    let (a, b, p) = (10.0, 25.0, 50.0);
    let mut rep = BRep::new();
    // the sphere's seam and poles must not lie in the cutting planes (the Boolean needs generic position)
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
    let surf = tessellate(&rep, body);
    let (ta, tb) = (tags_on_sphere(&surf, a), tags_on_sphere(&surf, b));
    assert_eq!((ta.len(), tb.len()), (1, 1));
    let (pf, pe) = radial_projector(3, vec![(a, ta.clone()), (b, tb)]);
    let params = MeshParams {
        size: Some(Arc::new(move |p: [f64; 3]| {
            let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
            (h + 0.15 * (r - a).max(0.0)).min(2.5 * h + 2.0)
        })),
        project: Some(pf.clone()),
        project_edge: Some(pe.clone()),
        ..Default::default()
    };
    let (mesh, _) = mesh_surface(&surf, &params).expect("mesh");
    let q = mesh.to_tet10_curved(Some(&|t, p| pf(t, p)), Some(&|t1, t2, p| pe(t1, t2, p)));
    report_mesh("Lamé sphere", &mesh, &q);
    let mut model = Model::new(q, steel());
    for i in 0..model.mesh.nodes.len() {
        let x = model.mesh.nodes[i];
        for axis in 0..3 {
            if x[axis].abs() < 1e-9 {
                model.fixed.push(Fixed {
                    node: i as u32,
                    axis,
                    value: 0.0,
                });
            }
        }
    }
    model.pressure.push((ta[0], p));
    let sol = model.solve(SolverKind::Direct).unwrap();
    let nodal = nodal_stress(&model, &sol.u);
    let (mut worst_u, mut worst_s, mut worst_edge) = (0.0f64, 0.0f64, 0.0f64);
    let faces = face_counts(&model.mesh);
    let mut sample = 0;
    for (i, x) in model.mesh.nodes.iter().enumerate() {
        let r = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
        if r < a * (1.0 - 1e-9) {
            continue;
        }
        let n = [x[0] / r, x[1] / r, x[2] / r];
        let ur = sol.u[3 * i] * n[0] + sol.u[3 * i + 1] * n[1] + sol.u[3 * i + 2] * n[2];
        let s = nodal[i];
        let srr = s[0] * n[0] * n[0]
            + s[1] * n[1] * n[1]
            + s[2] * n[2] * n[2]
            + 2.0 * (s[3] * n[1] * n[2] + s[4] * n[2] * n[0] + s[5] * n[0] * n[1]);
        let st = 0.5 * (s[0] + s[1] + s[2] - srr);
        let (_, st_ex, ur_ex) = lame_sphere(a, b, p, E, NU, r);
        worst_u = worst_u.max((ur / ur_ex - 1.0).abs());
        let st_peak = lame_sphere(a, b, p, E, NU, a).1;
        let dev = ((st - st_ex) / st_peak).abs();
        if faces[i] >= 2 {
            worst_edge = worst_edge.max(dev);
        } else {
            worst_s = worst_s.max(dev);
        }
        sample += 1;
    }
    eprintln!(
        "Lamé sphere h={h}: worst radial displacement error {:.3} %, worst hoop stress deviation {:.3} % of the bore value at volume and face-interior nodes ({:.3} % at edge nodes; {sample} nodes)",
        100.0 * worst_u,
        100.0 * worst_s,
        100.0 * worst_edge
    );
    (worst_u, worst_s)
}

#[test]
fn lame_sphere_within_two_percent() {
    let (u, s) = lame_sphere_case(0.8);
    assert!(u < 0.02, "radial displacement error {u}");
    assert!(s < 0.02, "hoop stress deviation {s}");
}

// ---------------------------------------------------------------------------------------------------
// Natural frequencies of a slender cantilever
// ---------------------------------------------------------------------------------------------------

#[test]
fn cantilever_bending_frequencies_within_two_percent() {
    // L/h = 20, square section; units mm, N, tonne, s
    let (l, h) = (100.0, 5.0);
    let (nx, ny) = (40, 3);
    let q = box_tets(l, h, h, nx, ny, ny).to_tet10(None);
    let mut model = Model::new(q, steel());
    model.fix_where([true; 3], &|x| x[0] < 1e-9);
    let t = std::time::Instant::now();
    let modes = kw_fem::modal::modal(&model, 12, 0.0).unwrap();
    eprintln!(
        "modal: {} DOF, {} iterations, {:.2} s; lowest frequencies {:?}",
        3 * model.mesh.nodes.len(),
        modes.iterations,
        t.elapsed().as_secs_f64(),
        modes
            .frequencies
            .iter()
            .map(|f| (f * 10.0).round() / 10.0)
            .collect::<Vec<_>>()
    );
    let (e, rho) = (E, 7.8e-9);
    let (area, inertia) = (h * h, h.powi(4) / 12.0);
    let g = e / (2.0 * (1.0 + NU));
    let kappa = 5.0 / 6.0;
    let mut matched = 0;
    for n in 0..3 {
        let eb = cantilever_frequency(n, l, e, inertia, rho, area);
        // first-order Timoshenko correction: shear and rotary inertia
        let bl = CANTILEVER_BETA_L[n];
        let r2 = inertia / area;
        let corrected = eb * (1.0 - 0.5 * bl * bl * r2 / (l * l) * (1.0 + e / (kappa * g)));
        // the square section gives a degenerate pair (bending in y and in z)
        let found: Vec<f64> = modes
            .frequencies
            .iter()
            .copied()
            .filter(|f| (f / corrected - 1.0).abs() < 0.015)
            .collect();
        eprintln!(
            "  bending mode {}: Euler–Bernoulli {eb:.2} Hz, with shear and rotary inertia {corrected:.2} Hz; FE {:?}",
            n + 1,
            found.iter().map(|f| (f * 100.0).round() / 100.0).collect::<Vec<_>>()
        );
        assert_eq!(
            found.len(),
            2,
            "the degenerate bending pair of mode {}",
            n + 1
        );
        for f in &found {
            assert!(
                (f / corrected - 1.0).abs() < 0.02,
                "mode {}: FE {f} vs {corrected}",
                n + 1
            );
        }
        matched += 1;
    }
    assert_eq!(matched, 3);
    // all frequencies are positive and ascending
    assert!(
        modes.frequencies.windows(2).all(|w| w[0] <= w[1] + 1e-9) && modes.frequencies[0] > 0.0
    );
}
