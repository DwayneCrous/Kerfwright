//! Triangle meshes as bodies: an imported mesh is kept in the document as a compact blob, and the body it
//! makes is a [`SurfaceMesh`] that features can move, scale, measure and export, or convert to a solid.

use crate::id::Hasher128;
use kw_mesh3d::SurfaceMesh;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::sync::Arc;

/// The unit a mesh file is written in; the model is in millimetres.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum MeshUnits {
    #[default]
    Mm,
    Cm,
    M,
    Inch,
}

impl MeshUnits {
    pub const ALL: [MeshUnits; 4] = [MeshUnits::Mm, MeshUnits::Cm, MeshUnits::M, MeshUnits::Inch];
    /// Millimetres per unit.
    pub fn factor(self) -> f64 {
        match self {
            MeshUnits::Mm => 1.0,
            MeshUnits::Cm => 10.0,
            MeshUnits::M => 1000.0,
            MeshUnits::Inch => 25.4,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            MeshUnits::Mm => "mm",
            MeshUnits::Cm => "cm",
            MeshUnits::M => "m",
            MeshUnits::Inch => "inch",
        }
    }
}

/// A welded, wound-outward mesh as bytes: node and triangle counts (`u32`), `f32` nodes, `u32` triangles.
/// Cloning shares the bytes. It is written whole to a file, but only as its 128-bit hash while the recipe is
/// being hashed (see [`crate::id::hashing`]).
#[derive(Clone)]
pub struct MeshBlob {
    bytes: Arc<[u8]>,
    hash: u128,
}

impl std::fmt::Debug for MeshBlob {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MeshBlob({} bytes, {:032x})",
            self.bytes.len(),
            self.hash
        )
    }
}

impl PartialEq for MeshBlob {
    fn eq(&self, other: &Self) -> bool {
        self.hash == other.hash && self.bytes.len() == other.bytes.len()
    }
}

fn hash_bytes(b: &[u8]) -> u128 {
    let mut h = Hasher128::new();
    h.bytes(b);
    h.finish()
}

impl MeshBlob {
    /// Pack `mesh`. Nodes are stored as `f32`: a mesh read from STL has nothing finer.
    pub fn from_mesh(mesh: &SurfaceMesh) -> MeshBlob {
        let mut b = Vec::with_capacity(8 + 12 * mesh.nodes.len() + 12 * mesh.tris.len());
        b.extend_from_slice(&(mesh.nodes.len() as u32).to_le_bytes());
        b.extend_from_slice(&(mesh.tris.len() as u32).to_le_bytes());
        for p in &mesh.nodes {
            for c in p {
                b.extend_from_slice(&(*c as f32).to_le_bytes());
            }
        }
        for t in &mesh.tris {
            for v in t {
                b.extend_from_slice(&v.to_le_bytes());
            }
        }
        MeshBlob::from_bytes(b).expect("a packed mesh unpacks")
    }

    fn from_bytes(bytes: Vec<u8>) -> Result<MeshBlob, String> {
        let blob = MeshBlob {
            hash: hash_bytes(&bytes),
            bytes: bytes.into(),
        };
        blob.counts()?;
        Ok(blob)
    }

    fn counts(&self) -> Result<(usize, usize), String> {
        let b = &self.bytes;
        if b.len() < 8 {
            return Err("the mesh data is too short".into());
        }
        let n = u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize;
        let m = u32::from_le_bytes([b[4], b[5], b[6], b[7]]) as usize;
        if 8 + 12 * n + 12 * m != b.len() {
            return Err("the mesh data does not match its counts".into());
        }
        Ok((n, m))
    }

    pub fn triangles(&self) -> usize {
        self.counts().map_or(0, |c| c.1)
    }
    pub fn len_bytes(&self) -> usize {
        self.bytes.len()
    }
    pub fn content_hash(&self) -> u128 {
        self.hash
    }

    /// Unpack the mesh (nodes widened to `f64`).
    pub fn mesh(&self) -> Result<SurfaceMesh, String> {
        let (n, m) = self.counts()?;
        let b = &self.bytes;
        let f = |o: usize| f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        let u = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        let nodes: Vec<[f64; 3]> = (0..n)
            .map(|i| {
                let o = 8 + 12 * i;
                [f64::from(f(o)), f64::from(f(o + 4)), f64::from(f(o + 8))]
            })
            .collect();
        let base = 8 + 12 * n;
        let tris: Vec<[u32; 3]> = (0..m)
            .map(|i| {
                let o = base + 12 * i;
                [u(o), u(o + 4), u(o + 8)]
            })
            .collect();
        if tris.iter().flatten().any(|v| *v as usize >= n) {
            return Err("the mesh data refers to a node that does not exist".into());
        }
        Ok(SurfaceMesh {
            nodes,
            tags: vec![0; tris.len()],
            tris,
        })
    }
}

impl Serialize for MeshBlob {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        if crate::id::hashing() {
            s.serialize_bytes(&self.hash.to_le_bytes())
        } else {
            s.serialize_bytes(&self.bytes)
        }
    }
}

impl<'de> Deserialize<'de> for MeshBlob {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        struct V;
        impl<'de> serde::de::Visitor<'de> for V {
            type Value = Vec<u8>;
            fn expecting(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "mesh bytes")
            }
            fn visit_bytes<E>(self, v: &[u8]) -> Result<Vec<u8>, E> {
                Ok(v.to_vec())
            }
            fn visit_byte_buf<E>(self, v: Vec<u8>) -> Result<Vec<u8>, E> {
                Ok(v)
            }
            fn visit_seq<A: serde::de::SeqAccess<'de>>(
                self,
                mut a: A,
            ) -> Result<Vec<u8>, A::Error> {
                let mut v = Vec::new();
                while let Some(b) = a.next_element::<u8>()? {
                    v.push(b);
                }
                Ok(v)
            }
        }
        let bytes = d.deserialize_bytes(V)?;
        MeshBlob::from_bytes(bytes).map_err(serde::de::Error::custom)
    }
}

/// Move a mesh from the file's units and axes into the model's (millimetres, z up).
pub fn to_model(mesh: &mut SurfaceMesh, units: MeshUnits, y_up: bool) {
    let k = units.factor();
    for p in &mut mesh.nodes {
        let q = if y_up { [p[0], -p[2], p[1]] } else { *p };
        *p = [q[0] * k, q[1] * k, q[2] * k];
    }
}

/// A fingerprint of the geometry of a mesh, for change detection.
pub fn mesh_hash(mesh: &SurfaceMesh) -> u128 {
    let mut h = Hasher128::new();
    h.u64(mesh.nodes.len() as u64).u64(mesh.tris.len() as u64);
    for p in &mesh.nodes {
        h.f64(p[0]).f64(p[1]).f64(p[2]);
    }
    for t in &mesh.tris {
        h.u64(u64::from(t[0]) << 32 | u64::from(t[1]))
            .u64(u64::from(t[2]));
    }
    h.finish()
}

/// What importing a file found.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshImport {
    pub triangles: usize,
    pub vertices: usize,
    pub degenerate_dropped: usize,
    pub flipped: usize,
    pub open_edges: usize,
    pub non_manifold_edges: usize,
    pub watertight: bool,
    /// Volume in the file's units cubed times the unit factor cubed: mm³.
    pub volume_mm3: f64,
    pub solids: usize,
}

impl MeshImport {
    /// One line for the status bar.
    pub fn summary(&self) -> String {
        let shape = if self.watertight {
            format!("closed, volume {:.3} mm³", self.volume_mm3)
        } else {
            format!(
                "not closed: {} open edges{}",
                self.open_edges,
                if self.non_manifold_edges > 0 {
                    format!(", {} non-manifold", self.non_manifold_edges)
                } else {
                    String::new()
                }
            )
        };
        format!(
            "{} triangles, {} vertices, {shape}",
            self.triangles, self.vertices
        )
    }
}

/// Read an STL file into an `ImportMesh` spec: the triangles are welded (corners closer than 1e-6 of the
/// bounding box diagonal), degenerate ones dropped, and the winding made consistent and outward.
pub fn import_stl(
    bytes: &[u8],
    source: &str,
    units: MeshUnits,
    y_up: bool,
) -> Result<(crate::spec::ImportMeshSpec, MeshImport), String> {
    let soup = kw_io::read_stl(bytes).map_err(|e| format!("{source}: {e}"))?;
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in soup.triangles.iter().flatten() {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diag = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    if !(diag > 0.0 && diag.is_finite()) {
        return Err(format!("{source}: the triangles have no extent"));
    }
    let (mut mesh, dropped) = SurfaceMesh::from_soup(&soup.triangles, 1e-6 * diag);
    if mesh.tris.is_empty() {
        return Err(format!("{source}: every triangle is degenerate"));
    }
    let flipped = mesh.orient();
    let st = mesh.stats();
    let k = units.factor();
    let info = MeshImport {
        triangles: st.triangles,
        vertices: st.vertices,
        degenerate_dropped: dropped,
        flipped,
        open_edges: st.open_edges,
        non_manifold_edges: st.non_manifold_edges,
        watertight: st.watertight,
        volume_mm3: st.volume * k * k * k,
        solids: soup.solids.len().max(1),
    };
    let spec = crate::spec::ImportMeshSpec {
        source: source.to_string(),
        units,
        y_up,
        data: MeshBlob::from_mesh(&mesh),
    };
    Ok((spec, info))
}
