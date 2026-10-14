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

/// Triangles read from an STL file: no shared vertices, file normals dropped.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StlSoup {
    pub triangles: Vec<[[f64; 3]; 3]>,
    /// Names of the `solid` blocks of an ASCII file (binary files have none).
    pub solids: Vec<String>,
    pub binary: bool,
}

/// Why a file could not be read as STL.
#[derive(Debug, Clone, PartialEq)]
pub enum StlError {
    Empty,
    /// A binary file whose size does not match its triangle count.
    Truncated {
        expected: usize,
        actual: usize,
    },
    /// An ASCII file with a bad number or an unfinished facet, with the 1-based line.
    Syntax {
        line: usize,
        message: String,
    },
    NoTriangles,
    NotFinite,
}

impl std::fmt::Display for StlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StlError::Empty => write!(f, "the file is empty"),
            StlError::Truncated { expected, actual } => write!(
                f,
                "the binary STL should be {expected} bytes for its triangle count but is {actual}"
            ),
            StlError::Syntax { line, message } => write!(f, "STL line {line}: {message}"),
            StlError::NoTriangles => write!(f, "the file holds no triangles"),
            StlError::NotFinite => write!(f, "the file holds coordinates that are not finite"),
        }
    }
}

impl std::error::Error for StlError {}

/// Read a binary or ASCII STL. A file is binary when its size is exactly `84 + 50·n` for the count `n` in
/// its header (many binary files start with the word "solid", so the text is not a reliable sign).
pub fn read_stl(bytes: &[u8]) -> Result<StlSoup, StlError> {
    if bytes.is_empty() {
        return Err(StlError::Empty);
    }
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if n.checked_mul(50).and_then(|b| b.checked_add(84)) == Some(bytes.len()) {
            return read_binary(bytes, n);
        }
    }
    let starts_solid = bytes
        .iter()
        .skip_while(|b| b.is_ascii_whitespace())
        .take(5)
        .map(|b| b.to_ascii_lowercase())
        .eq(*b"solid");
    if !starts_solid {
        if bytes.len() >= 84 {
            let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
            return Err(StlError::Truncated {
                expected: n.saturating_mul(50).saturating_add(84),
                actual: bytes.len(),
            });
        }
        return Err(StlError::Syntax {
            line: 1,
            message: "this is neither a binary nor an ASCII STL".into(),
        });
    }
    read_ascii(bytes)
}

fn read_binary(bytes: &[u8], n: usize) -> Result<StlSoup, StlError> {
    let f = |o: usize| f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
    let mut triangles = Vec::with_capacity(n);
    for t in 0..n {
        let base = 84 + 50 * t + 12;
        let mut tri = [[0.0; 3]; 3];
        for (i, p) in tri.iter_mut().enumerate() {
            for (k, c) in p.iter_mut().enumerate() {
                let v = f(base + 12 * i + 4 * k);
                if !v.is_finite() {
                    return Err(StlError::NotFinite);
                }
                *c = f64::from(v);
            }
        }
        triangles.push(tri);
    }
    if triangles.is_empty() {
        return Err(StlError::NoTriangles);
    }
    Ok(StlSoup {
        triangles,
        solids: Vec::new(),
        binary: true,
    })
}

fn read_ascii(bytes: &[u8]) -> Result<StlSoup, StlError> {
    let text = String::from_utf8_lossy(bytes);
    let mut soup = StlSoup::default();
    let mut corners: Vec<[f64; 3]> = Vec::new();
    let mut in_facet = false;
    for (i, raw) in text.lines().enumerate() {
        let line = i + 1;
        let mut words = raw.split_whitespace();
        let Some(first) = words.next() else { continue };
        match first.to_ascii_lowercase().as_str() {
            "solid" => soup.solids.push(words.collect::<Vec<_>>().join(" ")),
            "facet" => {
                if in_facet {
                    return Err(StlError::Syntax {
                        line,
                        message: "a facet starts inside another facet".into(),
                    });
                }
                in_facet = true;
                corners.clear();
            }
            "vertex" => {
                if !in_facet || corners.len() == 3 {
                    return Err(StlError::Syntax {
                        line,
                        message: "a vertex outside a facet of three vertices".into(),
                    });
                }
                let mut p = [0.0; 3];
                for c in p.iter_mut() {
                    let w = words.next().ok_or(StlError::Syntax {
                        line,
                        message: "a vertex needs three coordinates".into(),
                    })?;
                    *c = w.parse::<f64>().map_err(|_| StlError::Syntax {
                        line,
                        message: format!("`{w}` is not a number"),
                    })?;
                    if !c.is_finite() {
                        return Err(StlError::NotFinite);
                    }
                }
                corners.push(p);
            }
            "endfacet" => {
                if !in_facet || corners.len() != 3 {
                    return Err(StlError::Syntax {
                        line,
                        message: "a facet needs exactly three vertices".into(),
                    });
                }
                in_facet = false;
                soup.triangles.push([corners[0], corners[1], corners[2]]);
            }
            _ => {}
        }
    }
    if in_facet {
        return Err(StlError::Syntax {
            line: text.lines().count(),
            message: "the file ends inside a facet".into(),
        });
    }
    if soup.triangles.is_empty() {
        return Err(StlError::NoTriangles);
    }
    Ok(soup)
}
