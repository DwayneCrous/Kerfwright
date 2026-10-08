use crate::Axes;
use kw_tess::Mesh;
use std::io::{self, Write};

fn facet_normal(mesh: &Mesh, t: usize, axes: Axes) -> [f64; 3] {
    let [a, b, c] = mesh.triangle(t);
    let n = (b - a).cross(&(c - a));
    let l = n.norm();
    if l > 0.0 {
        axes.vector(&(n / l))
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Binary STL: 80-byte header, triangle count, 50 bytes per triangle.
pub fn write_stl_binary<W: Write>(mesh: &Mesh, axes: Axes, name: &str, mut w: W) -> io::Result<()> {
    let mut header = [0u8; 80];
    let label = format!("Kerfwright {name}");
    let bytes = label.as_bytes();
    header[..bytes.len().min(80)].copy_from_slice(&bytes[..bytes.len().min(80)]);
    w.write_all(&header)?;
    let n = mesh.triangle_count();
    w.write_all(&(n as u32).to_le_bytes())?;
    for t in 0..n {
        for c in facet_normal(mesh, t, axes) {
            w.write_all(&(c as f32).to_le_bytes())?;
        }
        for p in mesh.triangle(t) {
            for c in axes.point(&p) {
                w.write_all(&(c as f32).to_le_bytes())?;
            }
        }
        w.write_all(&[0u8, 0u8])?;
    }
    Ok(())
}

/// ASCII STL.
pub fn write_stl_ascii<W: Write>(mesh: &Mesh, axes: Axes, name: &str, mut w: W) -> io::Result<()> {
    let name: String = name
        .chars()
        .map(|c| if c.is_whitespace() { '_' } else { c })
        .collect();
    writeln!(w, "solid {name}")?;
    for t in 0..mesh.triangle_count() {
        let n = facet_normal(mesh, t, axes);
        writeln!(
            w,
            "  facet normal {:e} {:e} {:e}",
            n[0] as f32, n[1] as f32, n[2] as f32
        )?;
        writeln!(w, "    outer loop")?;
        for p in mesh.triangle(t) {
            let q = axes.point(&p);
            writeln!(
                w,
                "      vertex {:e} {:e} {:e}",
                q[0] as f32, q[1] as f32, q[2] as f32
            )?;
        }
        writeln!(w, "    endloop")?;
        writeln!(w, "  endfacet")?;
    }
    writeln!(w, "endsolid {name}")
}
