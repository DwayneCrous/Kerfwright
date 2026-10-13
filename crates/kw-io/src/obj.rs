use crate::Axes;
use kw_tess::Mesh;
use std::io::{self, Write};

#[derive(Debug, Clone)]
pub struct ObjOptions {
    pub axes: Axes,
    /// Emit one `g` group per B-rep face.
    pub group_by_face: bool,
}

impl Default for ObjOptions {
    fn default() -> Self {
        Self {
            axes: Axes::ZUp,
            group_by_face: true,
        }
    }
}

/// Wavefront OBJ with vertex normals. `mtl_file` is referenced via `mtllib`.
pub fn write_obj<W: Write>(
    mesh: &Mesh,
    name: &str,
    mtl_file: &str,
    opts: &ObjOptions,
    mut w: W,
) -> io::Result<()> {
    writeln!(w, "# Kerfwright export")?;
    writeln!(w, "mtllib {mtl_file}")?;
    writeln!(w, "o {}", name.replace(char::is_whitespace, "_"))?;
    for p in &mesh.positions {
        let q = opts.axes.point(p);
        writeln!(w, "v {} {} {}", q[0], q[1], q[2])?;
    }
    for n in &mesh.normals {
        let q = opts.axes.vector(n);
        writeln!(w, "vn {} {} {}", q[0], q[1], q[2])?;
    }
    writeln!(w, "usemtl kerfwright_default")?;
    writeln!(w, "s 1")?;
    let mut current_face = u32::MAX;
    for t in 0..mesh.triangle_count() {
        if opts.group_by_face && mesh.tri_face[t] != current_face {
            current_face = mesh.tri_face[t];
            writeln!(w, "g face_{current_face}")?;
        }
        let i = &mesh.indices[3 * t..3 * t + 3];
        writeln!(
            w,
            "f {0}//{0} {1}//{1} {2}//{2}",
            i[0] + 1,
            i[1] + 1,
            i[2] + 1
        )?;
    }
    Ok(())
}

pub fn write_mtl<W: Write>(mut w: W) -> io::Result<()> {
    writeln!(w, "# Kerfwright material")?;
    writeln!(w, "newmtl kerfwright_default")?;
    writeln!(w, "Ka 0.10 0.10 0.10")?;
    writeln!(w, "Kd 0.72 0.74 0.78")?;
    writeln!(w, "Ks 0.30 0.30 0.30")?;
    writeln!(w, "Ns 32")?;
    writeln!(w, "d 1.0")?;
    writeln!(w, "illum 2")
}
