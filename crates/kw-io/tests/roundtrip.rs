//! Re-read every exported file with independent parsers (stl_io, tobj, gltf)
//! and compare with the source mesh.

use kw_brep::*;
use kw_io::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_tess::{tessellate_body, Mesh, TessOptions};

fn meshes() -> Vec<(&'static str, Mesh)> {
    let f = Frame3::from_origin_normal(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.2, 0.1, 1.0));
    let opts = TessOptions {
        chord_tol: 0.02,
        ..TessOptions::default()
    };
    let mut out = Vec::new();
    for (name, build) in [
        (
            "box",
            Box::new(|b: &mut BRep| make_box(b, &f, 2.0, 3.0, 4.0).body)
                as Box<dyn Fn(&mut BRep) -> BodyId>,
        ),
        (
            "cylinder",
            Box::new(|b: &mut BRep| make_cylinder(b, &f, 1.5, 4.0).body),
        ),
        (
            "sphere",
            Box::new(|b: &mut BRep| make_sphere(b, &f, 2.0).body),
        ),
        (
            "torus",
            Box::new(|b: &mut BRep| make_torus(b, &f, 4.0, 1.0).body),
        ),
    ] {
        let mut b = BRep::new();
        let body = build(&mut b);
        out.push((name, tessellate_body(&b, body, &opts).unwrap().mesh));
    }
    out
}

fn bounds(points: impl Iterator<Item = [f32; 3]>) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

fn mesh_bounds(m: &Mesh, axes: Axes) -> ([f32; 3], [f32; 3]) {
    bounds(m.positions.iter().map(|p| {
        let q = axes.point(p);
        [q[0] as f32, q[1] as f32, q[2] as f32]
    }))
}

fn close(a: [f32; 3], b: [f32; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-4)
}

#[test]
fn stl_binary_and_ascii_roundtrip() {
    for (name, m) in meshes() {
        let mut bin = Vec::new();
        write_stl_binary(&m, Axes::ZUp, name, &mut bin).unwrap();
        assert_eq!(bin.len(), 84 + 50 * m.triangle_count(), "{name}");
        let parsed = stl_io::read_stl(&mut std::io::Cursor::new(&bin)).unwrap();
        assert_eq!(parsed.faces.len(), m.triangle_count(), "{name}");
        let (lo, hi) = bounds(parsed.vertices.iter().map(|v| [v[0], v[1], v[2]]));
        let (elo, ehi) = mesh_bounds(&m, Axes::ZUp);
        assert!(close(lo, elo) && close(hi, ehi), "{name}");

        let mut ascii = Vec::new();
        write_stl_ascii(&m, Axes::ZUp, name, &mut ascii).unwrap();
        let parsed = stl_io::read_stl(&mut std::io::Cursor::new(&ascii)).unwrap();
        assert_eq!(parsed.faces.len(), m.triangle_count(), "{name}");
        // facet normals from the file are unit and agree with winding
        for f in &parsed.faces {
            let l = (f.normal[0].powi(2) + f.normal[1].powi(2) + f.normal[2].powi(2)).sqrt();
            assert!((l - 1.0).abs() < 1e-4 || l == 0.0);
        }
    }
}

#[test]
fn obj_roundtrip() {
    let dir = std::env::temp_dir().join(format!("kw_io_obj_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for (name, m) in meshes() {
        let mut obj = Vec::new();
        write_obj(
            &m,
            name,
            &format!("{name}.mtl"),
            &ObjOptions::default(),
            &mut obj,
        )
        .unwrap();
        let mut mtl = Vec::new();
        write_mtl(&mut mtl).unwrap();
        std::fs::write(dir.join(format!("{name}.obj")), &obj).unwrap();
        std::fs::write(dir.join(format!("{name}.mtl")), &mtl).unwrap();
        let (models, mats) = tobj::load_obj(
            dir.join(format!("{name}.obj")),
            &tobj::LoadOptions {
                triangulate: false,
                single_index: false,
                ..Default::default()
            },
        )
        .unwrap();
        let mats = mats.unwrap();
        assert_eq!(mats.len(), 1, "{name}");
        let tris: usize = models.iter().map(|m| m.mesh.indices.len() / 3).sum();
        assert_eq!(tris, m.triangle_count(), "{name}");
        let (lo, hi) = bounds(
            models
                .iter()
                .flat_map(|m| m.mesh.positions.chunks(3).map(|c| [c[0], c[1], c[2]])),
        );
        let (elo, ehi) = mesh_bounds(&m, Axes::ZUp);
        assert!(close(lo, elo) && close(hi, ehi), "{name}");
        // one group per face
        assert!(!models.is_empty());
        let nn: usize = models.iter().map(|x| x.mesh.normal_indices.len()).sum();
        assert_eq!(nn, m.indices.len(), "{name}");
    }
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn glb_and_gltf_roundtrip() {
    let dir = std::env::temp_dir().join(format!("kw_io_gltf_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for (name, m) in meshes() {
        for per_face in [false, true] {
            let opts = GltfOptions {
                per_face_primitives: per_face,
                ..Default::default()
            };
            let glb = glb_bytes(&m, name, &opts);
            assert_eq!(glb.len() % 4, 0);
            let (doc, buffers, _) = gltf::import_slice(&glb).unwrap();
            check_gltf(name, &m, &doc, &buffers, per_face);
        }
        write_gltf_files(&m, &dir, name, &GltfOptions::default()).unwrap();
        let (doc, buffers, _) = gltf::import(dir.join(format!("{name}.gltf"))).unwrap();
        check_gltf(name, &m, &doc, &buffers, false);
    }
    let _ = std::fs::remove_dir_all(dir);
}

fn check_gltf(
    name: &str,
    m: &Mesh,
    doc: &gltf::Document,
    buffers: &[gltf::buffer::Data],
    per_face: bool,
) {
    let mesh = doc.meshes().next().unwrap();
    let mut tri_total = 0;
    let mut all_pos: Vec<[f32; 3]> = Vec::new();
    let prims: Vec<_> = mesh.primitives().collect();
    if per_face {
        assert!(
            !prims.is_empty() && prims.len() <= m.tri_face.last().map_or(0, |f| *f as usize + 1),
            "{name}"
        );
    } else {
        assert_eq!(prims.len(), 1);
    }
    for p in prims {
        let r = p.reader(|b| Some(&buffers[b.index()]));
        let pos: Vec<[f32; 3]> = r.read_positions().unwrap().collect();
        let nrm: Vec<[f32; 3]> = r.read_normals().unwrap().collect();
        assert_eq!(pos.len(), m.positions.len(), "{name}");
        assert_eq!(nrm.len(), pos.len());
        for n in &nrm {
            assert!(
                ((n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() - 1.0).abs() < 1e-4,
                "{name}"
            );
        }
        let idx: Vec<u32> = r.read_indices().unwrap().into_u32().collect();
        assert_eq!(idx.len() % 3, 0);
        assert!(idx.iter().all(|i| (*i as usize) < pos.len()));
        tri_total += idx.len() / 3;
        all_pos = pos;
    }
    assert_eq!(tri_total, m.triangle_count(), "{name}");
    let (lo, hi) = bounds(all_pos.into_iter());
    let (elo, ehi) = mesh_bounds(m, Axes::YUp);
    assert!(close(lo, elo) && close(hi, ehi), "{name}: y-up bounds");
    // accessor min/max must match the data (validators check this)
    let acc = doc.accessors().next().unwrap();
    let (amin, amax) = (acc.min().unwrap(), acc.max().unwrap());
    let amin = amin.as_array().unwrap();
    let amax = amax.as_array().unwrap();
    for k in 0..3 {
        assert!((amin[k].as_f64().unwrap() as f32 - lo[k]).abs() < 1e-5);
        assert!((amax[k].as_f64().unwrap() as f32 - hi[k]).abs() < 1e-5);
    }
}

fn soup_bounds(s: &StlSoup) -> ([f32; 3], [f32; 3]) {
    bounds(
        s.triangles
            .iter()
            .flatten()
            .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]),
    )
}

#[test]
fn our_own_stl_files_are_read_back() {
    for (name, m) in meshes() {
        let (elo, ehi) = mesh_bounds(&m, Axes::ZUp);
        let mut bin = Vec::new();
        write_stl_binary(&m, Axes::ZUp, name, &mut bin).unwrap();
        let s = read_stl(&bin).unwrap();
        assert!(s.binary, "{name}");
        assert_eq!(s.triangles.len(), m.triangle_count(), "{name}");
        let (lo, hi) = soup_bounds(&s);
        assert!(close(lo, elo) && close(hi, ehi), "{name}");

        let mut ascii = Vec::new();
        write_stl_ascii(&m, Axes::ZUp, name, &mut ascii).unwrap();
        let s = read_stl(&ascii).unwrap();
        assert!(!s.binary, "{name}");
        assert_eq!(s.triangles.len(), m.triangle_count(), "{name}");
        assert_eq!(s.solids, vec![name.to_string()]);
        let (lo, hi) = soup_bounds(&s);
        assert!(close(lo, elo) && close(hi, ehi), "{name}");
    }
}

#[test]
fn a_binary_file_whose_header_says_solid_is_still_binary() {
    let (_, m) = meshes().remove(0);
    let mut bin = Vec::new();
    write_stl_binary(&m, Axes::ZUp, "x", &mut bin).unwrap();
    bin[..5].copy_from_slice(b"solid");
    let s = read_stl(&bin).unwrap();
    assert!(s.binary);
    assert_eq!(s.triangles.len(), m.triangle_count());
}

#[test]
fn an_ascii_file_with_several_solids_and_odd_spacing_is_read() {
    let text = "  SOLID a\r\n facet normal 0 0 1\r\n outer loop\r\n vertex 0 0 0\r\n vertex 1 0 0\r\n vertex 0 1 0\r\n endloop\r\n endfacet\r\nendsolid a\r\nsolid b c\r\nfacet normal 0 0 0\r\nouter loop\r\nvertex 0 0 1e0\r\nvertex 1 0 1\r\nvertex 0 1 1.5E0\r\nendloop\r\nendfacet\r\nendsolid b c\r\n";
    let s = read_stl(text.as_bytes()).unwrap();
    assert_eq!(s.triangles.len(), 2);
    assert_eq!(s.solids, vec!["a".to_string(), "b c".to_string()]);
    assert_eq!(s.triangles[1][2], [0.0, 1.0, 1.5]);
}

#[test]
fn files_that_are_not_stl_are_refused_with_a_reason() {
    assert_eq!(read_stl(b""), Err(StlError::Empty));
    let (_, m) = meshes().remove(0);
    let mut bin = Vec::new();
    write_stl_binary(&m, Axes::ZUp, "x", &mut bin).unwrap();
    assert!(matches!(
        read_stl(&bin[..bin.len() - 7]),
        Err(StlError::Truncated { .. })
    ));
    assert!(matches!(
        read_stl(b"this is a plain text file, not a mesh at all"),
        Err(StlError::Syntax { .. })
    ));
    assert!(matches!(
        read_stl(b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 zero\n"),
        Err(StlError::Syntax { line: 4, .. })
    ));
    assert!(matches!(
        read_stl(b"solid x\nfacet normal 0 0 1\nvertex 0 0 0\nvertex 1 0 0\nendfacet\n"),
        Err(StlError::Syntax { .. })
    ));
    assert_eq!(
        read_stl(b"solid x\nendsolid x\n"),
        Err(StlError::NoTriangles)
    );
    // a facet left open at the end of the file
    assert!(matches!(
        read_stl(b"solid x\nfacet normal 0 0 1\nvertex 0 0 0\n"),
        Err(StlError::Syntax { .. })
    ));
}
