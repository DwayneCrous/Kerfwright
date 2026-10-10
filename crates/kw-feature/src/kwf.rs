//! The `.kwf` file: a ZIP container holding `manifest.json` and `recipe.cbor`.
//!
//! * The *recipe* is the whole truth: the document of features, parameters and references. Nothing
//!   else is needed to rebuild the model; any `cache/` entries a file carries are optional and are
//!   only preserved, never trusted.
//! * Files carry a schema version. A newer reader migrates older files step by step; an older reader
//!   ignores fields it does not know, and refuses (cleanly) a file whose `min_reader_version` is newer
//!   than it understands.
//! * Fields a reader does not know are kept when the file is saved again.

use crate::doc::Document;
use crate::engine::KERNEL_VERSION;
use crate::id::Hasher128;
use ciborium::Value;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;
use std::io::{Cursor, Read, Write};

/// The schema written by this version of the library.
pub const SCHEMA_VERSION: u32 = 3;
/// Oldest schema this library can read.
pub const OLDEST_SCHEMA: u32 = 1;
/// Largest entry a reader will inflate (guards against decompression bombs).
const MAX_ENTRY_BYTES: u64 = 512 << 20;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    pub format: String,
    pub schema_version: u32,
    /// The oldest reader that can understand this file without losing information.
    pub min_reader_version: u32,
    #[serde(default)]
    pub kernel_version: u64,
    #[serde(default)]
    pub writer: String,
    /// Hash of `recipe.cbor` (32 hex digits).
    #[serde(default)]
    pub recipe_hash: String,
    #[serde(default)]
    pub feature_count: u64,
}

#[derive(Debug)]
pub enum KwfError {
    Io(String),
    Zip(String),
    NotKwf(String),
    Missing(String),
    /// The file needs a newer reader.
    TooNew {
        needs: u32,
        have: u32,
    },
    TooOld(u32),
    Corrupt(String),
    Schema(String),
    TooLarge(String),
}

impl fmt::Display for KwfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            KwfError::Io(m) => write!(f, "cannot read or write the file: {m}"),
            KwfError::Zip(m) => write!(f, "not a valid container: {m}"),
            KwfError::NotKwf(m) => write!(f, "not a Kerfwright file: {m}"),
            KwfError::Missing(n) => write!(f, "the file has no '{n}'"),
            KwfError::TooNew { needs, have } => write!(
                f,
                "this file needs a reader of format version {needs} or newer; this one understands up to {have}"
            ),
            KwfError::TooOld(v) => write!(f, "format version {v} is too old to be read"),
            KwfError::Corrupt(m) => write!(f, "the file is damaged: {m}"),
            KwfError::Schema(m) => write!(f, "the file's contents do not match its format: {m}"),
            KwfError::TooLarge(n) => write!(f, "'{n}' is implausibly large"),
        }
    }
}
impl std::error::Error for KwfError {}

#[derive(Debug, Clone)]
pub struct KwfFile {
    pub document: Document,
    pub manifest: Manifest,
    /// What reading had to do (migrations applied, unknown entries kept).
    pub notes: Vec<String>,
    /// The recipe as read, after migration; unknown fields in it are carried into the next save.
    original: Option<Value>,
    /// ZIP entries other than the two this library owns (`cache/…`, thumbnails).
    pub extra_entries: BTreeMap<String, Vec<u8>>,
}

impl KwfFile {
    /// The recipe as read (after migration), including fields this library does not know.
    pub fn raw_recipe(&self) -> Option<&Value> {
        self.original.as_ref()
    }

    pub fn new(document: Document) -> KwfFile {
        KwfFile {
            document,
            manifest: Manifest {
                format: "kwf".into(),
                schema_version: SCHEMA_VERSION,
                min_reader_version: SCHEMA_VERSION,
                kernel_version: KERNEL_VERSION,
                writer: format!("kw-feature {}", env!("CARGO_PKG_VERSION")),
                recipe_hash: String::new(),
                feature_count: 0,
            },
            notes: Vec::new(),
            original: None,
            extra_entries: BTreeMap::new(),
        }
    }
}

fn hex(h: u128) -> String {
    format!("{h:032x}")
}

fn bytes_hash(b: &[u8]) -> u128 {
    let mut h = Hasher128::new();
    h.bytes(b);
    h.finish()
}

fn to_value<T: Serialize>(v: &T) -> Result<Value, KwfError> {
    Value::serialized(v).map_err(|e| KwfError::Schema(e.to_string()))
}

// ---------------------------------------------------------------------------------- Value helpers

fn map_get<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    match v {
        Value::Map(m) => m
            .iter()
            .find(|(k, _)| matches!(k, Value::Text(t) if t == key))
            .map(|(_, v)| v),
        _ => None,
    }
}

fn map_get_mut<'a>(v: &'a mut Value, key: &str) -> Option<&'a mut Value> {
    match v {
        Value::Map(m) => m
            .iter_mut()
            .find(|(k, _)| matches!(k, Value::Text(t) if t == key))
            .map(|(_, v)| v),
        _ => None,
    }
}

fn map_set(v: &mut Value, key: &str, val: Value) {
    if let Value::Map(m) = v {
        if let Some(slot) = m
            .iter_mut()
            .find(|(k, _)| matches!(k, Value::Text(t) if t == key))
        {
            slot.1 = val;
        } else {
            m.push((Value::Text(key.into()), val));
        }
    }
}

fn map_has(v: &Value, key: &str) -> bool {
    map_get(v, key).is_some()
}

/// Merge `new` over `old`: everything in `new` wins, but entries only `old` has (fields this library
/// does not know) are kept. Features are matched by id; a feature whose `kind` changed is not merged.
fn merge_preserving(old: &Value, new: &Value) -> Value {
    match (old, new) {
        (Value::Map(o), Value::Map(n)) => {
            if let (Some(a), Some(b)) = (map_get(old, "kind"), map_get(new, "kind")) {
                if a != b {
                    return new.clone();
                }
            }
            let mut out: Vec<(Value, Value)> = Vec::with_capacity(n.len());
            for (k, nv) in n {
                let merged = o
                    .iter()
                    .find(|(ok, _)| ok == k)
                    .map(|(_, ov)| merge_preserving(ov, nv))
                    .unwrap_or_else(|| nv.clone());
                out.push((k.clone(), merged));
            }
            for (k, ov) in o {
                if !n.iter().any(|(nk, _)| nk == k) {
                    out.push((k.clone(), ov.clone()));
                }
            }
            Value::Map(out)
        }
        (Value::Array(o), Value::Array(n)) => {
            let by_id = |v: &Value| map_get(v, "id").cloned();
            let keyed = n.iter().all(|x| by_id(x).is_some());
            Value::Array(
                n.iter()
                    .enumerate()
                    .map(|(i, nv)| {
                        let ov = if keyed {
                            o.iter().find(|x| by_id(x) == by_id(nv))
                        } else {
                            o.get(i)
                        };
                        match ov {
                            Some(ov) => merge_preserving(ov, nv),
                            None => nv.clone(),
                        }
                    })
                    .collect(),
            )
        }
        _ => new.clone(),
    }
}

// --------------------------------------------------------------------------------------- migrations

fn lit_zero_angle() -> Value {
    Value::Map(vec![
        (Value::Text("t".into()), Value::Text("Lit".into())),
        (
            Value::Text("v".into()),
            Value::Map(vec![
                (Value::Text("value".into()), Value::Float(0.0)),
                (
                    Value::Text("dim".into()),
                    to_value(&kw_expr::Dim::ANGLE).unwrap_or(Value::Null),
                ),
            ]),
        ),
    ])
}

/// v1 → v2: features can be suppressed and the timeline has a rollback bar.
fn migrate_v1_to_v2(recipe: &mut Value) -> Result<(), KwfError> {
    if !map_has(recipe, "rollback") {
        map_set(recipe, "rollback", Value::Null);
    }
    let Some(Value::Array(features)) = map_get_mut(recipe, "features") else {
        return Err(KwfError::Schema("the recipe has no feature list".into()));
    };
    for f in features {
        if !map_has(f, "suppressed") {
            map_set(f, "suppressed", Value::Bool(false));
        }
    }
    Ok(())
}

/// v2 → v3: datum planes can be tilted (earlier files are not tilted).
fn migrate_v2_to_v3(recipe: &mut Value) -> Result<(), KwfError> {
    let Some(Value::Array(features)) = map_get_mut(recipe, "features") else {
        return Err(KwfError::Schema("the recipe has no feature list".into()));
    };
    for f in features {
        if matches!(map_get(f, "spec").and_then(|s| map_get(s, "kind")), Some(Value::Text(k)) if k == "DatumPlane")
        {
            if let Some(spec) = map_get_mut(f, "spec") {
                if !map_has(spec, "tilt") {
                    map_set(spec, "tilt", lit_zero_angle());
                }
            }
        }
    }
    Ok(())
}

fn migrate(recipe: &mut Value, from: u32, notes: &mut Vec<String>) -> Result<(), KwfError> {
    let mut v = from;
    while v < SCHEMA_VERSION {
        match v {
            1 => migrate_v1_to_v2(recipe)?,
            2 => migrate_v2_to_v3(recipe)?,
            other => return Err(KwfError::TooOld(other)),
        }
        notes.push(format!("migrated schema {v} → {}", v + 1));
        v += 1;
    }
    Ok(())
}

// ------------------------------------------------------------------------------------------ writing

impl KwfFile {
    /// The container's bytes. Deterministic: the same document always gives the same file.
    pub fn to_bytes(&self) -> Result<Vec<u8>, KwfError> {
        let fresh = to_value(&self.document)?;
        let recipe = match &self.original {
            Some(old) => merge_preserving(old, &fresh),
            None => fresh,
        };
        let mut cbor = Vec::new();
        ciborium::into_writer(&recipe, &mut cbor).map_err(|e| KwfError::Io(e.to_string()))?;
        let mut manifest = self.manifest.clone();
        manifest.format = "kwf".into();
        manifest.schema_version = SCHEMA_VERSION;
        manifest.min_reader_version = SCHEMA_VERSION;
        manifest.kernel_version = KERNEL_VERSION;
        manifest.recipe_hash = hex(bytes_hash(&cbor));
        manifest.feature_count = self.document.len() as u64;
        let manifest_json =
            serde_json::to_vec_pretty(&manifest).map_err(|e| KwfError::Io(e.to_string()))?;

        let mut zw = zip::ZipWriter::new(Cursor::new(Vec::new()));
        let opts = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Deflated)
            .last_modified_time(zip::DateTime::default());
        let mut put = |name: &str, data: &[u8]| -> Result<(), KwfError> {
            zw.start_file(name, opts)
                .map_err(|e| KwfError::Zip(e.to_string()))?;
            zw.write_all(data).map_err(|e| KwfError::Io(e.to_string()))
        };
        put("manifest.json", &manifest_json)?;
        put("recipe.cbor", &cbor)?;
        for (name, data) in &self.extra_entries {
            put(name, data)?;
        }
        let cur = zw.finish().map_err(|e| KwfError::Zip(e.to_string()))?;
        Ok(cur.into_inner())
    }

    pub fn save(&self, path: &std::path::Path) -> Result<(), KwfError> {
        std::fs::write(path, self.to_bytes()?).map_err(|e| KwfError::Io(e.to_string()))
    }

    pub fn load(path: &std::path::Path) -> Result<KwfFile, KwfError> {
        let bytes = std::fs::read(path).map_err(|e| KwfError::Io(e.to_string()))?;
        KwfFile::from_bytes(&bytes)
    }

    // ------------------------------------------------------------------------------------ reading

    /// Read a file. Never panics, whatever the bytes are.
    pub fn from_bytes(bytes: &[u8]) -> Result<KwfFile, KwfError> {
        let mut zr =
            zip::ZipArchive::new(Cursor::new(bytes)).map_err(|e| KwfError::Zip(e.to_string()))?;
        let read_entry =
            |zr: &mut zip::ZipArchive<Cursor<&[u8]>>, name: &str| -> Result<Vec<u8>, KwfError> {
                let entry = zr
                    .by_name(name)
                    .map_err(|_| KwfError::Missing(name.to_string()))?;
                if entry.size() > MAX_ENTRY_BYTES {
                    return Err(KwfError::TooLarge(name.to_string()));
                }
                let mut out = Vec::new();
                entry
                    .take(MAX_ENTRY_BYTES + 1)
                    .read_to_end(&mut out)
                    .map_err(|e| KwfError::Corrupt(format!("'{name}': {e}")))?;
                if out.len() as u64 > MAX_ENTRY_BYTES {
                    return Err(KwfError::TooLarge(name.to_string()));
                }
                Ok(out)
            };
        let manifest_bytes = read_entry(&mut zr, "manifest.json")?;
        let manifest: Manifest = serde_json::from_slice(&manifest_bytes)
            .map_err(|e| KwfError::NotKwf(format!("manifest: {e}")))?;
        if manifest.format != "kwf" {
            return Err(KwfError::NotKwf(format!("format '{}'", manifest.format)));
        }
        if manifest.min_reader_version > SCHEMA_VERSION {
            return Err(KwfError::TooNew {
                needs: manifest.min_reader_version,
                have: SCHEMA_VERSION,
            });
        }
        if manifest.schema_version < OLDEST_SCHEMA {
            return Err(KwfError::TooOld(manifest.schema_version));
        }
        let cbor = read_entry(&mut zr, "recipe.cbor")?;
        if !manifest.recipe_hash.is_empty() && manifest.recipe_hash != hex(bytes_hash(&cbor)) {
            return Err(KwfError::Corrupt(
                "the recipe does not match its recorded hash".into(),
            ));
        }
        let mut recipe: Value =
            ciborium::de::from_reader_with_recursion_limit(cbor.as_slice(), 256)
                .map_err(|e| KwfError::Corrupt(format!("recipe: {e}")))?;
        let mut notes = Vec::new();
        migrate(
            &mut recipe,
            manifest.schema_version.min(SCHEMA_VERSION),
            &mut notes,
        )?;
        if manifest.schema_version > SCHEMA_VERSION {
            notes.push(format!(
                "written by a newer version (schema {}); unknown fields are ignored and kept",
                manifest.schema_version
            ));
        }
        let document: Document = recipe
            .deserialized()
            .map_err(|e| KwfError::Schema(e.to_string()))?;
        document
            .validate()
            .map_err(|e| KwfError::Schema(format!("the document is inconsistent: {e}")))?;
        let mut extra_entries = BTreeMap::new();
        let names: Vec<String> = zr.file_names().map(|s| s.to_string()).collect();
        for n in names {
            if n == "manifest.json" || n == "recipe.cbor" || n.ends_with('/') {
                continue;
            }
            if let Ok(d) = read_entry(&mut zr, &n) {
                notes.push(format!("kept the entry '{n}'"));
                extra_entries.insert(n, d);
            }
        }
        Ok(KwfFile {
            document,
            manifest,
            notes,
            original: Some(recipe),
            extra_entries,
        })
    }
}

/// Writers of the earlier schemas, frozen as they were. They exist so that compatibility is tested
/// against files that really have the old shape (and to regenerate the checked-in fixtures).
pub mod legacy {
    use super::*;

    fn strip(recipe: &mut Value, version: u32) {
        if version < 2 {
            if let Value::Map(m) = recipe {
                m.retain(|(k, _)| !matches!(k, Value::Text(t) if t == "rollback"));
            }
            if let Some(Value::Array(fs)) = map_get_mut(recipe, "features") {
                for f in fs {
                    if let Value::Map(m) = f {
                        m.retain(|(k, _)| !matches!(k, Value::Text(t) if t == "suppressed"));
                    }
                }
            }
        }
        if version < 3 {
            if let Some(Value::Array(fs)) = map_get_mut(recipe, "features") {
                for f in fs {
                    if let Some(spec) = map_get_mut(f, "spec") {
                        if matches!(map_get(spec, "kind"), Some(Value::Text(k)) if k == "DatumPlane")
                        {
                            if let Value::Map(m) = spec {
                                m.retain(|(k, _)| !matches!(k, Value::Text(t) if t == "tilt"));
                            }
                        }
                    }
                }
            }
        }
    }

    /// A file as schema `version` (1 or 2) would have written it. Documents must not use anything
    /// that schema lacked (suppressed features for v1, tilted datum planes for v1 and v2).
    pub fn write(doc: &Document, version: u32) -> Result<Vec<u8>, KwfError> {
        assert!((1..SCHEMA_VERSION).contains(&version));
        let mut recipe = to_value(doc)?;
        strip(&mut recipe, version);
        let mut cbor = Vec::new();
        ciborium::into_writer(&recipe, &mut cbor).map_err(|e| KwfError::Io(e.to_string()))?;
        let manifest = Manifest {
            format: "kwf".into(),
            schema_version: version,
            min_reader_version: version,
            kernel_version: 1,
            writer: format!("kw-feature legacy schema {version}"),
            recipe_hash: hex(bytes_hash(&cbor)),
            feature_count: doc.len() as u64,
        };
        container(&manifest, &cbor)
    }

    /// A file from a hypothetical newer writer: unknown fields in the manifest, the document and its
    /// features, and an extra entry. `min_reader_version` says whether old readers must refuse it.
    pub fn write_future(doc: &Document, min_reader_version: u32) -> Result<Vec<u8>, KwfError> {
        let mut recipe = to_value(doc)?;
        map_set(
            &mut recipe,
            "future_document_field",
            Value::Text("kept".into()),
        );
        if let Some(Value::Array(fs)) = map_get_mut(&mut recipe, "features") {
            for f in fs {
                map_set(f, "future_feature_field", Value::Integer(7.into()));
            }
        }
        let mut cbor = Vec::new();
        ciborium::into_writer(&recipe, &mut cbor).map_err(|e| KwfError::Io(e.to_string()))?;
        let mut manifest = serde_json::to_value(Manifest {
            format: "kwf".into(),
            schema_version: SCHEMA_VERSION + 1,
            min_reader_version,
            kernel_version: KERNEL_VERSION,
            writer: "kw-feature future".into(),
            recipe_hash: hex(bytes_hash(&cbor)),
            feature_count: doc.len() as u64,
        })
        .map_err(|e| KwfError::Io(e.to_string()))?;
        manifest["future_manifest_field"] = serde_json::json!({"a": 1});
        let manifest_json =
            serde_json::to_vec(&manifest).map_err(|e| KwfError::Io(e.to_string()))?;
        let mut zw = zip::ZipWriter::new(Cursor::new(Vec::new()));
        let opts =
            zip::write::SimpleFileOptions::default().last_modified_time(zip::DateTime::default());
        for (n, d) in [
            ("manifest.json", manifest_json.as_slice()),
            ("recipe.cbor", cbor.as_slice()),
            ("cache/outputs.bin", &[1u8, 2, 3][..]),
        ] {
            zw.start_file(n, opts)
                .map_err(|e| KwfError::Zip(e.to_string()))?;
            zw.write_all(d).map_err(|e| KwfError::Io(e.to_string()))?;
        }
        Ok(zw
            .finish()
            .map_err(|e| KwfError::Zip(e.to_string()))?
            .into_inner())
    }

    fn container(manifest: &Manifest, cbor: &[u8]) -> Result<Vec<u8>, KwfError> {
        let json = serde_json::to_vec(manifest).map_err(|e| KwfError::Io(e.to_string()))?;
        let mut zw = zip::ZipWriter::new(Cursor::new(Vec::new()));
        let opts =
            zip::write::SimpleFileOptions::default().last_modified_time(zip::DateTime::default());
        for (n, d) in [("manifest.json", json.as_slice()), ("recipe.cbor", cbor)] {
            zw.start_file(n, opts)
                .map_err(|e| KwfError::Zip(e.to_string()))?;
            zw.write_all(d).map_err(|e| KwfError::Io(e.to_string()))?;
        }
        Ok(zw
            .finish()
            .map_err(|e| KwfError::Zip(e.to_string()))?
            .into_inner())
    }
}
