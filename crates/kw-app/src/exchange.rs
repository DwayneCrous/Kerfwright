//! Files exchanged with other programs: STEP / IGES bodies in, STEP / STL / OBJ / GLB out.
//!
//! Imported bodies become an `Import` feature that keeps the file's text, so a project stays one
//! self-contained `.kwf` file. Export writes the model's final solid.

use crate::model::unique_name;
use crate::project::Project;
use kw_feature::{FeatureSpec, ImportFormat, ImportSpec};
use kw_tess::{tessellate_body, TessOptions};
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

/// Write the model's final solid to `path`.
pub fn export_file(
    project: &mut Project,
    format: ExportFormat,
    path: &Path,
) -> Result<String, String> {
    let ev = project.evaluation();
    let Some((_, out)) = ev.final_solid() else {
        return Err("there is no solid to export yet".into());
    };
    let (Some(brep), Some(body)) = (out.brep(), out.body()) else {
        return Err("there is no solid to export yet".into());
    };
    let name = project.name();
    let bytes: Vec<u8> = match format {
        ExportFormat::Step => kw_step::export(
            brep,
            &[body],
            &kw_step::ExportOptions {
                product_name: name.clone(),
                ..Default::default()
            },
        )
        .into_bytes(),
        _ => {
            let tess = tessellate_body(
                brep,
                body,
                &TessOptions {
                    chord_tol: 0.01,
                    angle_tol: 0.15,
                },
            )
            .map_err(|e| format!("tessellation failed: {e}"))?;
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
