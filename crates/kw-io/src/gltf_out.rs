use crate::Axes;
use kw_tess::Mesh;
use serde_json::{json, Value};
use std::io;
use std::path::Path;

#[derive(Debug, Clone)]
pub struct GltfOptions {
    /// One glTF primitive per B-rep face (keeps face identity); otherwise one primitive.
    pub per_face_primitives: bool,
    pub base_color: [f32; 4],
    pub metallic: f32,
    pub roughness: f32,
}

impl Default for GltfOptions {
    fn default() -> Self {
        Self {
            per_face_primitives: false,
            base_color: [0.72, 0.74, 0.78, 1.0],
            metallic: 0.1,
            roughness: 0.5,
        }
    }
}

fn pad4(v: &mut Vec<u8>, byte: u8) {
    while !v.len().is_multiple_of(4) {
        v.push(byte);
    }
}

/// Build the glTF JSON document and its single binary buffer. `buffer_uri`
/// is the `uri` of the buffer (`None` for GLB's embedded chunk).
pub fn gltf_json_and_bin(
    mesh: &Mesh,
    name: &str,
    opts: &GltfOptions,
    buffer_uri: Option<&str>,
) -> (Value, Vec<u8>) {
    let axes = Axes::YUp;
    let nv = mesh.positions.len();
    let mut bin: Vec<u8> = Vec::new();
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for p in &mesh.positions {
        let q = axes.point(p);
        for k in 0..3 {
            let f = q[k] as f32;
            min[k] = min[k].min(f);
            max[k] = max[k].max(f);
            bin.extend_from_slice(&f.to_le_bytes());
        }
    }
    let pos_len = bin.len();
    for n in &mesh.normals {
        for c in axes.vector(n) {
            bin.extend_from_slice(&(c as f32).to_le_bytes());
        }
    }
    let nrm_len = bin.len() - pos_len;
    let idx_off = bin.len();
    for i in &mesh.indices {
        bin.extend_from_slice(&i.to_le_bytes());
    }
    let idx_len = bin.len() - idx_off;
    pad4(&mut bin, 0);

    let mut buffer_views = vec![
        json!({"buffer": 0, "byteOffset": 0, "byteLength": pos_len, "target": 34962}),
        json!({"buffer": 0, "byteOffset": pos_len, "byteLength": nrm_len, "target": 34962}),
        json!({"buffer": 0, "byteOffset": idx_off, "byteLength": idx_len, "target": 34963}),
    ];
    let mut accessors = vec![
        json!({"bufferView": 0, "componentType": 5126, "count": nv, "type": "VEC3",
               "min": [min[0], min[1], min[2]], "max": [max[0], max[1], max[2]]}),
        json!({"bufferView": 1, "componentType": 5126, "count": nv, "type": "VEC3"}),
    ];
    let mut primitives = Vec::new();
    if opts.per_face_primitives {
        // triangles are contiguous per face
        let tri_count = mesh.triangle_count();
        let mut start = 0usize;
        while start < tri_count {
            let face = mesh.tri_face[start];
            let mut end = start;
            while end < tri_count && mesh.tri_face[end] == face {
                end += 1;
            }
            accessors.push(
                json!({"bufferView": 2, "byteOffset": start * 12, "componentType": 5125,
                                  "count": (end - start) * 3, "type": "SCALAR"}),
            );
            primitives.push(json!({"attributes": {"POSITION": 0, "NORMAL": 1},
                                   "indices": accessors.len() - 1, "material": 0, "mode": 4,
                                   "extras": {"face": face}}));
            start = end;
        }
    } else {
        accessors.push(json!({"bufferView": 2, "componentType": 5125, "count": mesh.indices.len(), "type": "SCALAR"}));
        primitives.push(json!({"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2, "material": 0, "mode": 4}));
    }
    let mut buffer = json!({"byteLength": bin.len()});
    if let Some(uri) = buffer_uri {
        buffer["uri"] = json!(uri);
    }
    let _ = &mut buffer_views;
    let doc = json!({
        "asset": {"version": "2.0", "generator": "Kerfwright"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": name}],
        "meshes": [{"name": name, "primitives": primitives}],
        "materials": [{"name": "kerfwright_default",
                       "pbrMetallicRoughness": {"baseColorFactor": opts.base_color,
                                                "metallicFactor": opts.metallic,
                                                "roughnessFactor": opts.roughness}}],
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [buffer],
    });
    (doc, bin)
}

/// Binary glTF (`.glb`).
pub fn glb_bytes(mesh: &Mesh, name: &str, opts: &GltfOptions) -> Vec<u8> {
    let (doc, bin) = gltf_json_and_bin(mesh, name, opts, None);
    let mut json_bytes = serde_json::to_vec(&doc).expect("serialisable");
    pad4(&mut json_bytes, b' ');
    let total = 12 + 8 + json_bytes.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(b"glTF");
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&(total as u32).to_le_bytes());
    out.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x4E4F534Au32.to_le_bytes()); // "JSON"
    out.extend_from_slice(&json_bytes);
    out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x004E4942u32.to_le_bytes()); // "BIN\0"
    out.extend_from_slice(&bin);
    out
}

/// Write `<dir>/<stem>.gltf` and `<dir>/<stem>.bin`.
pub fn write_gltf_files(mesh: &Mesh, dir: &Path, stem: &str, opts: &GltfOptions) -> io::Result<()> {
    let bin_name = format!("{stem}.bin");
    let (doc, bin) = gltf_json_and_bin(mesh, stem, opts, Some(&bin_name));
    std::fs::write(dir.join(&bin_name), bin)?;
    std::fs::write(
        dir.join(format!("{stem}.gltf")),
        serde_json::to_vec_pretty(&doc).expect("serialisable"),
    )
}
