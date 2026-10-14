//! Files exchanged with other programs: STEP / IGES bodies in, STEP / STL / OBJ / GLB out.
//!
//! Imported bodies become an `Import` feature that keeps the file's text, so a project stays one
//! self-contained `.kwf` file. Export writes the model's final solid.

use crate::model::unique_name;
use crate::project::Project;
use kw_feature::mesh_body::{import_stl, MeshUnits};
use kw_feature::{FeatureId, FeatureSpec, ImportFormat, ImportSpec, MeshToSolidSpec};
use kw_tess::{tessellate_brep, TessOptions};
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportFormat {
    Step,
    Stl,
    Obj,
    Glb,
}

impl ExportFormat {
    pub const ALL: [ExportFormat; 4] = [
        ExportFormat::Step,
        ExportFormat::Stl,
        ExportFormat::Obj,
        ExportFormat::Glb,
    ];

    pub fn label(self) -> &'static str {
        match self {
            ExportFormat::Step => "STEP (.step)",
            ExportFormat::Stl => "STL (.stl)",
            ExportFormat::Obj => "OBJ (.obj)",
            ExportFormat::Glb => "glTF binary (.glb)",
        }
    }

    pub fn filter(self) -> &'static str {
        match self {
            ExportFormat::Step => "STEP",
            ExportFormat::Stl => "STL",
            ExportFormat::Obj => "OBJ",
            ExportFormat::Glb => "glTF binary",
        }
    }

    pub fn extensions(self) -> &'static [&'static str] {
        match self {
            ExportFormat::Step => &["step", "stp"],
            ExportFormat::Stl => &["stl"],
            ExportFormat::Obj => &["obj"],
            ExportFormat::Glb => &["glb"],
        }
    }
}

/// Which format a file name says it is, for import.
pub fn import_format(path: &Path) -> Option<ImportFormat> {
    match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
        "step" | "stp" => Some(ImportFormat::Step),
        "iges" | "igs" => Some(ImportFormat::Iges),
        _ => None,
    }
}

/// Is this a mesh file (read as a triangle mesh body, not a solid)?
pub fn is_mesh_file(path: &Path) -> bool {
    path.extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("stl"))
}

/// The look of a mesh file before it is imported: what it holds, for the import dialog.
pub struct MeshPreview {
    pub source: String,
    pub bytes: Vec<u8>,
}

impl MeshPreview {
    pub fn read(path: &Path) -> Result<MeshPreview, String> {
        let bytes =
            std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        let source = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "mesh".into());
        // fail early on a file that is not a mesh at all
        import_stl(&bytes, &source, MeshUnits::Mm, false)?;
        Ok(MeshPreview { source, bytes })
    }

    /// What importing with these options would find (no change to any project).
    pub fn summary(&self, units: MeshUnits, y_up: bool) -> Result<String, String> {
        import_stl(&self.bytes, &self.source, units, y_up).map(|(_, info)| info.summary())
    }
}

/// Add a triangle mesh body made from an STL file. The file is welded and wound outward, and its triangles
/// are kept in the project; a file that is not a mesh changes nothing.
pub fn import_mesh(
    project: &mut Project,
    preview: &MeshPreview,
    units: MeshUnits,
    y_up: bool,
) -> Result<String, String> {
    let (spec, info) = import_stl(&preview.bytes, &preview.source, units, y_up)?;
    let doc = project.document();
    let name = unique_name(&doc, "Mesh");
    let spec = FeatureSpec::ImportMesh(spec);
    let mut trial = (*doc).clone();
    let id = trial.add(&name, spec.clone()).map_err(|e| e.to_string())?;
    let ev = project.engine.evaluate(&trial);
    if ev.output(id).is_none() {
        let why = ev
            .failures()
            .iter()
            .map(|f| format!("{f:?}"))
            .collect::<Vec<_>>()
            .join("; ");
        return Err(format!("{}: {why}", preview.source));
    }
    project
        .history
        .edit(|d| d.add(&name, spec).map(|_| ()))
        .map_err(|e| e.to_string())?;
    Ok(format!("imported {}: {}", preview.source, info.summary()))
}

/// Turn the mesh body ending at `head` into a faceted solid. Tried on a copy first, so a mesh that is not
/// closed (or too large) changes nothing and the reason comes back.
pub fn convert_mesh(
    project: &mut Project,
    head: FeatureId,
    merge_coplanar: bool,
) -> Result<String, String> {
    let doc = project.document();
    let name = unique_name(&doc, "Solid");
    let spec = FeatureSpec::MeshToSolid(MeshToSolidSpec {
        base: Some(head),
        merge_coplanar,
    });
    let mut trial = (*doc).clone();
    let id = trial.add(&name, spec.clone()).map_err(|e| e.to_string())?;
    let ev = project.engine.evaluate(&trial);
    let Some(out) = ev.output(id) else {
        let why = ev
            .failures()
            .iter()
            .find(|(f, _)| *f == id)
            .map(|(_, e)| e.to_string())
            .unwrap_or_else(|| "the mesh could not be converted".into());
        return Err(format!("cannot convert to a solid: {why}"));
    };
    let faces = out.brep().map(|b| b.faces.len()).unwrap_or(0);
    project
        .history
        .edit(|d| d.add(&name, spec).map(|_| ()))
        .map_err(|e| e.to_string())?;
    Ok(format!("converted to a solid of {faces} faces"))
}

/// Add the bodies of a STEP or IGES file to the project as an `Import` feature. The file is tried first;
/// a file that holds no usable solid changes nothing and the reason is returned.
pub fn import_file(project: &mut Project, path: &Path) -> Result<String, String> {
    let format = import_format(path).ok_or_else(|| {
        "unsupported file type: import reads STEP (.step, .stp) and IGES (.iges, .igs) solids"
            .to_string()
    })?;
    let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let data = String::from_utf8_lossy(&bytes).into_owned();
    let source = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "imported".into());
    let spec = FeatureSpec::Import(ImportSpec {
        source: source.clone(),
        format,
        data,
    });
    let doc = project.document();
    let name = unique_name(&doc, "Import");
    // try it on a copy: the evaluator reports why a file is unusable
    let mut trial = (*doc).clone();
    let id = trial.add(&name, spec.clone()).map_err(|e| e.to_string())?;
    let ev = project.engine.evaluate(&trial);
    let Some(out) = ev.output(id) else {
        let why = ev
            .failures()
            .iter()
            .map(|f| format!("{f:?}"))
            .collect::<Vec<_>>()
            .join("; ");
        return Err(format!("{source}: {why}"));
    };
    let faces = out.brep().map(|b| b.faces.len()).unwrap_or(0);
    project
        .history
        .edit(|d| d.add(&name, spec).map(|_| ()))
        .map_err(|e| e.to_string())?;
    Ok(format!("imported {source}: {faces} faces"))
}

/// Add the triangles of a mesh body to an export mesh (flat shaded: the exporters take face normals).
fn append_mesh(dst: &mut kw_tess::Mesh, m: &kw_mesh3d::SurfaceMesh) {
    let base = dst.positions.len() as u32;
    let weld_base = dst.weld.iter().copied().max().map_or(0, |w| w + 1);
    for (k, t) in m.tris.iter().enumerate() {
        let p: Vec<kw_math::Point3<f64>> = t
            .iter()
            .map(|v| {
                let q = m.nodes[*v as usize];
                kw_math::Point3::new(q[0], q[1], q[2])
            })
            .collect();
        let n = (p[1] - p[0])
            .cross(&(p[2] - p[0]))
            .try_normalize(0.0)
            .unwrap_or_else(kw_math::Vector3::z);
        for (i, v) in t.iter().enumerate() {
            dst.positions.push(p[i]);
            dst.normals.push(n);
            dst.uv.push((0.0, 0.0));
            dst.weld.push(weld_base + *v);
            dst.indices.push(base + 3 * k as u32 + i as u32);
        }
        dst.tri_face.push(0);
    }
}

/// Write the model's final solid to `path`.
pub fn export_file(
    project: &mut Project,
    format: ExportFormat,
    path: &Path,
) -> Result<String, String> {
    let ev = project.evaluation();
    let doc = project.document();
    // every body of the model goes into the file; mesh bodies go into the mesh formats only
    let bodies = crate::bodies::Bodies::new(&doc, &ev, &std::collections::HashSet::new());
    let meshes = crate::bodies::mesh_bodies(&doc, &ev, &std::collections::HashSet::new());
    if bodies.is_none() && (meshes.is_empty() || format == ExportFormat::Step) {
        return Err(if meshes.is_empty() {
            "there is no solid to export yet".into()
        } else {
            "STEP holds solids: convert the mesh to a solid first (right-click it in Bodies)".into()
        });
    }
    let empty = kw_brep::BRep::new();
    let brep = bodies.as_ref().map_or(&empty, |b| &*b.brep);
    let body_ids: Vec<kw_brep::BodyId> = brep.bodies.keys().collect();
    let name = project.name();
    let bytes: Vec<u8> = match format {
        ExportFormat::Step => kw_step::export(
            brep,
            &body_ids,
            &kw_step::ExportOptions {
                product_name: name.clone(),
                ..Default::default()
            },
        )
        .into_bytes(),
        _ => {
            let mut tess = tessellate_brep(
                brep,
                &TessOptions {
                    chord_tol: 0.01,
                    angle_tol: 0.15,
                },
            )
            .map_err(|e| format!("tessellation failed: {e}"))?;
            for m in &meshes {
                append_mesh(&mut tess.mesh, &m.mesh);
            }
            let mut buf = Vec::new();
            match format {
                ExportFormat::Stl => {
                    kw_io::write_stl_binary(&tess.mesh, kw_io::Axes::ZUp, &name, &mut buf)
                        .map_err(|e| e.to_string())?
                }
                ExportFormat::Obj => {
                    let mtl = path.with_extension("mtl");
                    let mtl_name = mtl
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| "model.mtl".into());
                    kw_io::write_obj(
                        &tess.mesh,
                        &name,
                        &mtl_name,
                        &kw_io::ObjOptions::default(),
                        &mut buf,
                    )
                    .map_err(|e| e.to_string())?;
                    let mut m = Vec::new();
                    kw_io::write_mtl(&mut m).map_err(|e| e.to_string())?;
                    std::fs::write(&mtl, m)
                        .map_err(|e| format!("cannot write {}: {e}", mtl.display()))?;
                }
                ExportFormat::Glb => {
                    buf = kw_io::glb_bytes(&tess.mesh, &name, &kw_io::GltfOptions::default());
                }
                ExportFormat::Step => unreachable!(),
            }
            buf
        }
    };
    std::fs::write(path, &bytes).map_err(|e| format!("cannot write {}: {e}", path.display()))?;
    Ok(format!(
        "exported {} ({} KB)",
        path.file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default(),
        bytes.len().div_ceil(1024)
    ))
}
