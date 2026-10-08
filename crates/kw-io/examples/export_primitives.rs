//! Export every primitive to STL / OBJ / glTF / GLB.
//! Usage: cargo run -p kw-io --example export_primitives -- <out_dir> [chord_tol]

use kw_brep::*;
use kw_io::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_tess::{tessellate_body, TessOptions};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::PathBuf;

fn main() -> std::io::Result<()> {
    let mut args = std::env::args().skip(1);
    let out = PathBuf::from(args.next().unwrap_or_else(|| "out".into()));
    let tol: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.01);
    fs::create_dir_all(&out)?;
    let f = Frame3::from_origin_normal(Point3::new(0.0, 0.0, 0.0), Vector3::z());
    let opts = TessOptions {
        chord_tol: tol,
        ..TessOptions::default()
    };
    let mut items: Vec<(&str, BRep, BodyId)> = Vec::new();
    let mut add = |name, build: &dyn Fn(&mut BRep) -> BodyId| {
        let mut b = BRep::new();
        let body = build(&mut b);
        items.push((name, b, body));
    };
    add("box", &|b| make_box(b, &f, 2.0, 3.0, 4.0).body);
    add("cylinder", &|b| make_cylinder(b, &f, 1.5, 4.0).body);
    add("cone", &|b| make_cone(b, &f, 2.0, 0.0, 3.0).body);
    add("frustum", &|b| make_cone(b, &f, 2.0, 1.0, 3.0).body);
    add("sphere", &|b| make_sphere(b, &f, 2.0).body);
    add("torus", &|b| make_torus(b, &f, 4.0, 1.0).body);
    for (name, brep, body) in items {
        let t = tessellate_body(&brep, body, &opts).expect("tessellation");
        write_stl_binary(
            &t.mesh,
            Axes::ZUp,
            name,
            BufWriter::new(File::create(out.join(format!("{name}.stl")))?),
        )?;
        write_stl_ascii(
            &t.mesh,
            Axes::ZUp,
            name,
            BufWriter::new(File::create(out.join(format!("{name}_ascii.stl")))?),
        )?;
        write_obj(
            &t.mesh,
            name,
            &format!("{name}.mtl"),
            &ObjOptions::default(),
            BufWriter::new(File::create(out.join(format!("{name}.obj")))?),
        )?;
        write_mtl(BufWriter::new(File::create(
            out.join(format!("{name}.mtl")),
        )?))?;
        write_gltf_files(&t.mesh, &out, name, &GltfOptions::default())?;
        fs::write(
            out.join(format!("{name}.glb")),
            glb_bytes(
                &t.mesh,
                name,
                &GltfOptions {
                    per_face_primitives: true,
                    ..Default::default()
                },
            ),
        )?;
        println!("{name}: {} triangles", t.mesh.triangle_count());
    }
    Ok(())
}
