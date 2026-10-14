//! Mesh bodies in the document: import options, the file format, hashing, moving and converting.

use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::mesh_body::{import_stl, MeshBlob, MeshUnits};
use kw_feature::spec::*;
use kw_feature::*;
use kw_io::{write_stl_ascii, write_stl_binary, Axes};
use kw_tess::{tessellate_body, TessOptions};

fn mm(v: f64) -> Param {
    Param::length(v)
}

/// A 20 x 30 x 40 box as an STL file.
fn box_stl(binary: bool) -> Vec<u8> {
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

fn bounds(ev: &Evaluation, id: FeatureId) -> ([f64; 3], [f64; 3]) {
    ev.output(id)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()))
        .mesh()
        .expect("a mesh")
        .bounds()
}

fn imported(units: MeshUnits, y_up: bool) -> (Document, FeatureId) {
    let (spec, _) = import_stl(&box_stl(true), "box.stl", units, y_up).unwrap();
    let mut doc = Document::new();
    let id = doc.add("Mesh", FeatureSpec::ImportMesh(spec)).unwrap();
    (doc, id)
}

#[test]
fn units_and_the_up_axis_are_applied_when_the_body_is_made() {
    let (doc, id) = imported(MeshUnits::Mm, false);
    let (lo, hi) = bounds(&Engine::new().evaluate(&doc), id);
    let size = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let mut sorted = size;
    sorted.sort_by(f64::total_cmp);
    assert!(
        (sorted[0] - 20.0).abs() < 1e-4 && (sorted[2] - 40.0).abs() < 1e-4,
        "{size:?}"
    );

    let (doc, id) = imported(MeshUnits::Inch, false);
    let (lo2, hi2) = bounds(&Engine::new().evaluate(&doc), id);
    for k in 0..3 {
        assert!(((hi2[k] - lo2[k]) - size[k] * 25.4).abs() < 1e-3);
    }
    // y up: the file's (x, y, z) is the model's (x, -z, y)
    let (doc, id) = imported(MeshUnits::Mm, true);
    let (lo3, hi3) = bounds(&Engine::new().evaluate(&doc), id);
    assert!(((hi3[1] - lo3[1]) - size[2]).abs() < 1e-4);
    assert!(((hi3[2] - lo3[2]) - size[1]).abs() < 1e-4);
    // the file's z range is the model's -y range, the file's y range the model's z range
    assert!(
        (hi3[1] + lo[2]).abs() < 1e-4 && (lo3[1] + hi[2]).abs() < 1e-4,
        "{lo3:?} {hi3:?}"
    );
    assert!((lo3[2] - lo[1]).abs() < 1e-4 && (hi3[2] - hi[1]).abs() < 1e-4);
    assert!((lo3[0] - lo[0]).abs() < 1e-4 && (hi3[0] - hi[0]).abs() < 1e-4);
}

#[test]
fn moving_and_scaling_a_mesh_body_moves_and_scales_its_triangles() {
    let (mut doc, id) = imported(MeshUnits::Mm, false);
    let before = bounds(&Engine::new().evaluate(&doc), id);
    let mv = doc
        .add(
            "Move",
            FeatureSpec::MoveBody(MoveBodySpec {
                base: None,
                delta: [mm(10.0), mm(0.0), mm(-3.0)],
                axis: 2,
                angle: Param::angle_deg(0.0),
                copy: false,
            }),
        )
        .unwrap();
    let ev = Engine::new().evaluate(&doc);
    let after = bounds(&ev, mv);
    assert!((after.0[0] - before.0[0] - 10.0).abs() < 1e-9);
    assert!((after.1[2] - before.1[2] + 3.0).abs() < 1e-9);
    let sc = doc
        .add(
            "Scale",
            FeatureSpec::ScaleBody(ScaleBodySpec {
                base: None,
                factor: Param::scalar(2.0),
                centre: [mm(0.0), mm(0.0), mm(0.0)],
            }),
        )
        .unwrap();
    let ev = Engine::new().evaluate(&doc);
    let big = bounds(&ev, sc);
    for k in 0..3 {
        assert!((big.1[k] - 2.0 * after.1[k]).abs() < 1e-9);
    }
    // still one body, and still a mesh
    assert_eq!(doc.heads(), vec![sc]);
    assert!(ev.output(sc).unwrap().mesh().is_some());
}

#[test]
fn the_mesh_survives_a_kwf_round_trip_and_the_recipe_hash_ignores_its_size() {
    let (doc, id) = imported(MeshUnits::Cm, false);
    let bytes = KwfFile::new(doc.clone()).to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap().document;
    assert_eq!(back.content_hash(), doc.content_hash());
    let ev = Engine::new().evaluate(&back);
    let (lo, hi) = bounds(&ev, id);
    assert!(
        ((hi[0] - lo[0]) - 200.0).abs() < 1e-3
            || ((hi[0] - lo[0]) - 300.0).abs() < 1e-3
            || ((hi[0] - lo[0]) - 400.0).abs() < 1e-3
    );

    // the hash covers the mesh through the blob's own hash only: a different mesh changes it, and hashing a
    // large blob is no slower than a small one
    let big = kw_mesh3d::SurfaceMesh::sphere([0.0; 3], 10.0, 7);
    assert!(big.tris.len() > 100_000);
    let blob = MeshBlob::from_mesh(&big);
    assert!(blob.len_bytes() > 2_000_000);
    let mut doc2 = doc.clone();
    doc2.replace_spec(
        id,
        FeatureSpec::ImportMesh(ImportMeshSpec {
            source: "big.stl".into(),
            units: MeshUnits::Mm,
            y_up: false,
            data: blob,
        }),
    )
    .unwrap();
    assert_ne!(doc2.content_hash(), doc.content_hash());
    let t0 = std::time::Instant::now();
    for _ in 0..200 {
        std::hint::black_box(doc2.content_hash());
    }
    let per = t0.elapsed().as_secs_f64() / 200.0;
    assert!(
        per < 2e-3,
        "hashing a recipe with a {} byte blob took {per} s",
        2_000_000
    );
    // and the whole thing still round-trips
    let bytes = KwfFile::new(doc2.clone()).to_bytes().unwrap();
    let back = KwfFile::from_bytes(&bytes).unwrap().document;
    assert_eq!(back.content_hash(), doc2.content_hash());
}

#[test]
fn files_that_are_not_meshes_import_nothing() {
    for junk in [&b"not a mesh"[..], &b""[..], &b"solid x\nendsolid x\n"[..]] {
        assert!(import_stl(junk, "junk.stl", MeshUnits::Mm, false).is_err());
    }
    // all triangles degenerate
    let flat = b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 2 0 0\nendloop\nendfacet\nendsolid x\n";
    let e = import_stl(flat, "flat.stl", MeshUnits::Mm, false).unwrap_err();
    assert!(e.contains("degenerate"), "{e}");
}

#[test]
fn an_open_mesh_imports_but_will_not_convert() {
    let mut text = String::from("solid open\n");
    for tri in [
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
    ] {
        text.push_str("facet normal 0 0 1\nouter loop\n");
        for p in tri {
            text.push_str(&format!("vertex {} {} {}\n", p[0], p[1], p[2]));
        }
        text.push_str("endloop\nendfacet\n");
    }
    text.push_str("endsolid open\n");
    let (spec, info) = import_stl(text.as_bytes(), "open.stl", MeshUnits::Mm, false).unwrap();
    assert!(!info.watertight && info.open_edges == 4, "{info:?}");
    assert!(info.summary().contains("not closed"));
    let mut doc = Document::new();
    doc.add("Mesh", FeatureSpec::ImportMesh(spec)).unwrap();
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
    let e = ev.error(conv).expect("refused").to_string();
    assert!(e.contains("not closed"), "{e}");
}

#[test]
fn a_converted_mesh_takes_a_hole_like_any_solid() {
    let (mut doc, id) = imported(MeshUnits::Mm, false);
    let (lo, hi) = bounds(&Engine::new().evaluate(&doc), id);
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
    let rep = ev.output(conv).unwrap().brep().unwrap();
    let before = rep
        .volume(ev.output(conv).unwrap().body().unwrap())
        .unwrap();
    assert!((before - 24000.0).abs() < 1e-2, "{before}");
    let (cx, cy) = ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0);
    let h = doc
        .add(
            "Hole",
            FeatureSpec::Hole(HoleSpec {
                plane: PlaneSpec::Xy(mm(hi[2])),
                points: vec![[mm(cx), mm(cy)]],
                kind: HoleKind::Simple { diameter: mm(6.0) },
                depth: mm(10.0),
                through_all: false,
                base: None,
            }),
        )
        .unwrap();
    let ev = Engine::new().evaluate(&doc);
    let o = ev
        .output(h)
        .unwrap_or_else(|| panic!("{:?}", ev.failures()));
    let rep = o.brep().unwrap();
    assert!(rep.validate().is_ok());
    let after = rep.volume(o.body().unwrap()).unwrap();
    let hole = std::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (before - after - hole).abs() < 1e-3 * hole,
        "{before} {after}"
    );
    // only the final solid is a body
    assert_eq!(doc.heads(), vec![h]);
}
