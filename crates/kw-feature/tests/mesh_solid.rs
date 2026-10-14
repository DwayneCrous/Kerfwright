//! Closed triangle meshes become faceted solids.

use kw_brep::BRep;
use kw_feature::mesh_solid::mesh_to_bodies;
use kw_mesh3d::SurfaceMesh;

fn convert(m: &SurfaceMesh, merge: bool) -> Result<(BRep, kw_brep::BodyId), String> {
    let mut rep = BRep::new();
    let (bodies, _) = mesh_to_bodies(&mut rep, m, merge)?;
    assert_eq!(bodies.len(), 1);
    Ok((rep, bodies[0]))
}

#[test]
fn a_box_mesh_becomes_a_box_of_six_faces() {
    let m = SurfaceMesh::cuboid(20.0, 30.0, 40.0);
    let (rep, b) = convert(&m, true).unwrap();
    rep.validate().into_result().unwrap();
    assert_eq!(rep.body_faces(b).len(), 6);
    assert_eq!(rep.edges.len(), 12);
    let v = rep.volume(b).unwrap();
    assert!((v - 24000.0).abs() < 1e-6 * 24000.0, "{v}");
    // without merging every triangle is a face
    let (rep, b) = convert(&m, false).unwrap();
    rep.validate().into_result().unwrap();
    assert_eq!(rep.body_faces(b).len(), 12);
    assert!((rep.volume(b).unwrap() - 24000.0).abs() < 1e-6 * 24000.0);
}

#[test]
fn an_open_mesh_is_refused_with_the_count_of_open_edges() {
    let mut m = SurfaceMesh::cuboid(1.0, 1.0, 1.0);
    m.tris.pop();
    m.tags.pop();
    let e = convert(&m, true).unwrap_err();
    assert!(e.contains("not closed") && e.contains("open edges"), "{e}");
}

fn volume_ok(rep: &BRep, b: kw_brep::BodyId, expected: f64) {
    let v = rep.volume(b).unwrap();
    assert!(
        (v - expected).abs() <= 1e-6 * expected.abs(),
        "{v} vs {expected}"
    );
}

#[test]
fn curved_meshes_stay_faceted_and_keep_their_volume() {
    let s = SurfaceMesh::sphere([1.0, 2.0, 3.0], 5.0, 2);
    let (rep, b) = convert(&s, true).unwrap();
    rep.validate().into_result().unwrap();
    assert_eq!(rep.body_faces(b).len(), s.tris.len());
    volume_ok(&rep, b, s.volume());
    let c = SurfaceMesh::cylinder(3.0, 8.0, 24, 3);
    let (rep, b) = convert(&c, true).unwrap();
    rep.validate().into_result().unwrap();
    // the two 24-sided caps are one face each; the sides stay as triangles
    assert!(rep.body_faces(b).len() < c.tris.len());
    volume_ok(&rep, b, c.volume());
}

#[test]
fn a_flat_face_with_a_hole_keeps_its_inner_loop() {
    use kw_boolean::{boolean, BoolOptions, Op};
    use kw_brep::{make_box, make_cylinder};
    use kw_math::{Frame3, Point3, Vector3};
    use kw_tess::{tessellate_body, TessOptions};
    let mut rep = BRep::new();
    let bx = make_box(
        &mut rep,
        &Frame3::from_origin_normal(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        40.0,
        30.0,
        10.0,
    )
    .body;
    let cy = make_cylinder(
        &mut rep,
        &Frame3::from_origin_normal(Point3::new(0.0, 0.0, -6.0), Vector3::z()),
        6.0,
        12.0,
    )
    .body;
    let body = boolean(&mut rep, bx, cy, Op::Subtract, &BoolOptions::default())
        .unwrap()
        .body;
    let t = tessellate_body(
        &rep,
        body,
        &TessOptions {
            chord_tol: 0.02,
            ..TessOptions::default()
        },
    )
    .unwrap();
    let mut m = SurfaceMesh::from_tess(&t.mesh);
    m.orient();
    assert!(m.stats().watertight, "{:?}", m.stats());
    let (out, b) = convert(&m, true).unwrap();
    out.validate().into_result().unwrap();
    volume_ok(&out, b, m.volume());
    // top and bottom each hold the hole as an inner loop
    let with_hole = out
        .body_faces(b)
        .into_iter()
        .filter(|f| out.faces[*f].loops.len() == 2)
        .count();
    assert_eq!(with_hole, 2);
}

#[test]
fn a_box_with_extra_vertices_along_its_edges_still_has_twelve_edges() {
    // split the long edge of the cuboid's mesh: every triangle touching it is cut in two
    let b = SurfaceMesh::cuboid(10.0, 10.0, 10.0);
    let mut soup: Vec<[[f64; 3]; 3]> = Vec::new();
    for t in &b.tris {
        let p = [
            b.nodes[t[0] as usize],
            b.nodes[t[1] as usize],
            b.nodes[t[2] as usize],
        ];
        // cut each triangle at the midpoint of its first edge, which also splits the neighbour's edge
        // only when the neighbour is cut at the same place: so cut all three edges (4 triangles)
        let m = |a: [f64; 3], c: [f64; 3]| {
            [
                (a[0] + c[0]) / 2.0,
                (a[1] + c[1]) / 2.0,
                (a[2] + c[2]) / 2.0,
            ]
        };
        let (m01, m12, m20) = (m(p[0], p[1]), m(p[1], p[2]), m(p[2], p[0]));
        soup.push([p[0], m01, m20]);
        soup.push([m01, p[1], m12]);
        soup.push([m20, m12, p[2]]);
        soup.push([m01, m12, m20]);
    }
    let (m, dropped) = SurfaceMesh::from_soup(&soup, 1e-6);
    assert_eq!(dropped, 0);
    assert!(m.stats().watertight);
    let (rep, body) = convert(&m, true).unwrap();
    rep.validate().into_result().unwrap();
    assert_eq!(rep.body_faces(body).len(), 6);
    assert_eq!(rep.edges.len(), 12);
    volume_ok(&rep, body, 1000.0);
}

// ---- through the document ----

mod document {
    use kw_expr::Param;
    use kw_feature::mesh_body::{import_stl, MeshUnits};
    use kw_feature::spec::*;
    use kw_feature::{Document, Engine, FeatureSpec, Product};
    use kw_io::{write_stl_ascii, write_stl_binary, Axes};
    use kw_tess::{tessellate_body, TessOptions};

    pub fn box_stl(binary: bool) -> Vec<u8> {
        // a 20 x 30 x 40 box from the kernel, tessellated and written as STL
        let mut rep = kw_brep::BRep::new();
        let body = kw_brep::make_box(
            &mut rep,
            &kw_math::Frame3::from_origin_normal(
                kw_math::Point3::new(5.0, 6.0, 7.0),
                kw_math::Vector3::z(),
            ),
            20.0,
            30.0,
            40.0,
        )
        .body;
        let t = tessellate_body(&rep, body, &TessOptions::default()).unwrap();
        let mut out = Vec::new();
        if binary {
            write_stl_binary(&t.mesh, Axes::ZUp, "box", &mut out).unwrap();
        } else {
            write_stl_ascii(&t.mesh, Axes::ZUp, "box", &mut out).unwrap();
        }
        out
    }

    #[test]
    fn an_imported_stl_is_a_mesh_body_that_converts_to_a_solid() {
        for binary in [true, false] {
            let (spec, info) =
                import_stl(&box_stl(binary), "box.stl", MeshUnits::Mm, false).unwrap();
            assert!(info.watertight, "{info:?}");
            assert_eq!(info.triangles, 12);
            assert!(
                (info.volume_mm3 - 24000.0).abs() < 1e-3,
                "{}",
                info.volume_mm3
            );
            let mut doc = Document::new();
            let imp = doc.add("Mesh", FeatureSpec::ImportMesh(spec)).unwrap();
            let ev = Engine::new().evaluate(&doc);
            let out = ev.output(imp).expect("evaluated");
            assert!(matches!(out.product, Product::Mesh(_)));
            assert_eq!(doc.heads(), vec![imp]);
            // a solid feature refuses it, naming the way out
            let mut bad = doc.clone();
            let h = bad
                .add(
                    "Shell",
                    FeatureSpec::Fillet(FilletSpec {
                        base: None,
                        edges: Vec::new(),
                        radius: Param::length(1.0),
                    }),
                )
                .unwrap();
            let ev = Engine::new().evaluate(&bad);
            let e = ev.error(h).expect("fails");
            assert!(e.to_string().contains("convert it to a solid"), "{e}");
            // converting gives six faces and the volume of the mesh
            let conv = doc
                .add(
                    "Solid",
                    FeatureSpec::MeshToSolid(MeshToSolidSpec {
                        base: None,
                        merge_coplanar: true,
                    }),
                )
                .unwrap();
            let ev = Engine::new().evaluate(&doc);
            let out = ev.output(conv).expect("converted");
            let rep = out.brep().unwrap();
            let body = out.body().unwrap();
            assert_eq!(rep.body_faces(body).len(), 6);
            assert!((rep.volume(body).unwrap() - 24000.0).abs() < 1e-3);
            assert_eq!(doc.heads(), vec![conv]);
        }
    }
}

#[test]
fn converting_a_fine_mesh_takes_seconds_not_minutes() {
    // level 5 is 20 480 triangles, level 6 is 81 920 (debug builds only do the small one)
    let levels: &[usize] = if cfg!(debug_assertions) {
        &[4]
    } else {
        &[5, 6]
    };
    for &level in levels {
        let s = SurfaceMesh::sphere([0.0, 0.0, 0.0], 25.0, level);
        let t0 = std::time::Instant::now();
        let (rep, b) = convert(&s, true).unwrap();
        let built = t0.elapsed().as_secs_f64();
        let v = rep.volume(b).unwrap();
        eprintln!(
            "{} triangles: built in {built:.2} s, volume {:.3} s",
            s.tris.len(),
            t0.elapsed().as_secs_f64() - built
        );
        assert!((v - s.volume()).abs() < 1e-6 * v.abs());
        assert!(built < 20.0, "{built} s");
    }
}
