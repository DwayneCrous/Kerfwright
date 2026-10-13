//! Components linked to external `.kwf` files.
//!
//! A [`Part`] may carry a [`PartLink`]: where its model came from (a path relative to the project's
//! folder, and the absolute path as a fallback) and the content hash of that model when it was last
//! taken. The part's own `document` stays in the project as a **cached copy**, so the project is
//! self-contained: it opens and solves with the cache when the file is gone or unreadable. A link is
//! out of date when the file's hash no longer matches; [`Assembly::update_link`] takes the new model.
//! Frames belong to the assembly's part (they are coordinates, not references into the model), so they
//! survive an update.
//!
//! A link may also lead to a `.kwf` that holds an **assembly**: the part is then a sub-assembly (`Part::sub`),
//! taken from the file's `assembly.cbor`. Links inside that assembly are relative to its own file, and an
//! update goes down through them; files that link to each other in a circle are refused.

use super::*;
use std::path::{Component, Path, PathBuf};

/// What the file holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum LinkKind {
    /// A model: the part's `document`.
    #[default]
    Part,
    /// An assembly: the part is a sub-assembly (`Part::sub`).
    Assembly,
}

/// Where a part's model comes from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartLink {
    /// Path of the file relative to the folder of the project, with `/` separators. Tried first.
    pub rel: Option<String>,
    /// Absolute path of the file. Used when `rel` is missing or does not lead to a file.
    pub abs: String,
    /// The content hash (32 hex digits) of the model when it was last taken from the file.
    pub source_hash: String,
    /// Whether the file is taken as a model or as an assembly.
    #[serde(default)]
    pub kind: LinkKind,
}

/// How a link stands against its file.
#[derive(Debug, Clone, PartialEq)]
pub enum LinkStatus {
    /// The file holds the model the part has.
    Current,
    /// The file has changed since the model was taken.
    Stale,
    /// There is no file at either path; the cached model is used.
    Missing,
    /// The file is there but cannot be read as a model (and why); the cached model is used.
    Unreadable(String),
}

/// What an update did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UpdateReport {
    pub part: String,
    /// Whether the model changed (false: the file held the same model).
    pub changed: bool,
}

fn hash_hex(doc: &Document) -> String {
    format!("{:032x}", doc.content_hash())
}

fn hash_bytes(b: &[u8]) -> String {
    let mut h = kw_feature::Hasher128::new();
    h.bytes(b);
    format!("{:032x}", h.finish())
}

/// What a linked file holds, read.
enum Loaded {
    Model(Document),
    /// The assembly and the hash of its stored bytes.
    Assembly(Box<Assembly>, String),
}

/// `to` relative to the folder `from`, with `/` separators. Both must be absolute.
pub fn relative_path(from: &Path, to: &Path) -> Option<String> {
    if !from.is_absolute() || !to.is_absolute() {
        return None;
    }
    let a: Vec<Component> = from.components().collect();
    let b: Vec<Component> = to.components().collect();
    let common = a.iter().zip(&b).take_while(|(x, y)| x == y).count();
    // different roots (another drive) have nothing in common to climb to
    if common == 0 {
        return None;
    }
    let mut parts: Vec<String> = Vec::new();
    for _ in common..a.len() {
        parts.push("..".into());
    }
    for c in &b[common..] {
        parts.push(c.as_os_str().to_string_lossy().into_owned());
    }
    Some(parts.join("/"))
}

impl PartLink {
    /// A link to `path` for a model that has just been taken from it. `base_dir` is the folder of the
    /// project, if it has been saved.
    pub fn new(path: &Path, base_dir: Option<&Path>, doc: &Document) -> PartLink {
        let abs = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
        let rel = base_dir
            .and_then(|d| {
                std::fs::canonicalize(d)
                    .ok()
                    .or_else(|| Some(d.to_path_buf()))
            })
            .and_then(|d| relative_path(&d, &abs));
        PartLink {
            rel,
            abs: abs.to_string_lossy().into_owned(),
            source_hash: hash_hex(doc),
            kind: LinkKind::Part,
        }
    }

    /// A link to the assembly stored in the file at `path`, whose stored form is `bytes`.
    pub fn to_assembly(path: &Path, base_dir: Option<&Path>, bytes: &[u8]) -> PartLink {
        let mut l = PartLink::new(path, base_dir, &Document::new());
        l.source_hash = hash_bytes(bytes);
        l.kind = LinkKind::Assembly;
        l
    }

    /// The file the link leads to now: the relative path from `base_dir` if there is a file there, else
    /// the absolute one.
    pub fn resolve(&self, base_dir: Option<&Path>) -> PathBuf {
        if let (Some(rel), Some(dir)) = (&self.rel, base_dir) {
            let mut p = dir.to_path_buf();
            for piece in rel.split('/') {
                p.push(piece);
            }
            if p.is_file() {
                return p;
            }
        }
        PathBuf::from(&self.abs)
    }

    /// Read what the file holds, saying how the link stands if it cannot be.
    fn read(&self, base_dir: Option<&Path>) -> Result<Loaded, LinkStatus> {
        let path = self.resolve(base_dir);
        if !path.exists() {
            return Err(LinkStatus::Missing);
        }
        let file = KwfFile::load(&path).map_err(|e| LinkStatus::Unreadable(e.to_string()))?;
        match self.kind {
            LinkKind::Part => Ok(Loaded::Model(file.document)),
            LinkKind::Assembly => {
                let bytes = file.extra_entries.get("assembly.cbor").ok_or_else(|| {
                    LinkStatus::Unreadable("the file holds no assembly any more".into())
                })?;
                let asm = Assembly::from_bytes(bytes).map_err(LinkStatus::Unreadable)?;
                Ok(Loaded::Assembly(Box::new(asm), hash_bytes(bytes)))
            }
        }
    }

    /// How this link stands against its file (an assembly is also out of date when a file linked inside
    /// it is).
    /// `cached` is the sub-assembly the part holds, whose own links are judged against their files.
    pub fn status(&self, base_dir: Option<&Path>, cached: Option<&Assembly>) -> LinkStatus {
        self.status_in(base_dir, cached, &mut Vec::new())
    }

    fn status_in(
        &self,
        base_dir: Option<&Path>,
        cached: Option<&Assembly>,
        stack: &mut Vec<PathBuf>,
    ) -> LinkStatus {
        let path = self.resolve(base_dir);
        let here = std::fs::canonicalize(&path).unwrap_or(path.clone());
        if stack.contains(&here) {
            return LinkStatus::Unreadable(circle(stack, &here));
        }
        match self.read(base_dir) {
            Ok(Loaded::Model(doc)) => {
                if hash_hex(&doc) == self.source_hash {
                    LinkStatus::Current
                } else {
                    LinkStatus::Stale
                }
            }
            Ok(Loaded::Assembly(_, hash)) => {
                if hash != self.source_hash {
                    return LinkStatus::Stale;
                }
                // the files linked inside the copy the part holds, relative to the folder of its file
                let Some(inside) = cached else {
                    return LinkStatus::Current;
                };
                stack.push(here);
                let inner = inside.link_status_in(path.parent(), stack);
                stack.pop();
                if inner.iter().any(|(_, s)| *s == LinkStatus::Stale) {
                    LinkStatus::Stale
                } else if let Some((_, bad)) = inner
                    .iter()
                    .find(|(_, s)| matches!(s, LinkStatus::Unreadable(_)))
                {
                    bad.clone()
                } else {
                    LinkStatus::Current
                }
            }
            Err(s) => s,
        }
    }
}

fn circle(stack: &[PathBuf], again: &Path) -> String {
    let mut names: Vec<String> = stack
        .iter()
        .map(|p| {
            p.file_name()
                .map_or_else(String::new, |n| n.to_string_lossy().into_owned())
        })
        .collect();
    names.push(
        again
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned()),
    );
    format!(
        "the files link to each other in a circle: {}",
        names.join(" → ")
    )
}

impl Assembly {
    /// The linked parts and how each stands against its file. `base_dir` is the folder of the project.
    pub fn link_status(&self, base_dir: Option<&Path>) -> Vec<(usize, LinkStatus)> {
        self.link_status_in(base_dir, &mut Vec::new())
    }

    fn link_status_in(
        &self,
        base_dir: Option<&Path>,
        stack: &mut Vec<PathBuf>,
    ) -> Vec<(usize, LinkStatus)> {
        self.parts
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                p.link
                    .as_ref()
                    .map(|l| (i, l.status_in(base_dir, p.sub.as_deref(), stack)))
            })
            .collect()
    }

    /// Take the model of part `part` again from the file it is linked to. The part's frames are kept. For
    /// an assembly file the links inside it are brought up to date first.
    pub fn update_link(
        &mut self,
        part: usize,
        base_dir: Option<&Path>,
    ) -> Result<UpdateReport, AssemblyError> {
        self.update_link_in(part, base_dir, &mut Vec::new())
    }

    fn update_link_in(
        &mut self,
        part: usize,
        base_dir: Option<&Path>,
        stack: &mut Vec<PathBuf>,
    ) -> Result<UpdateReport, AssemblyError> {
        let p = self
            .parts
            .get_mut(part)
            .ok_or(AssemblyError::UnknownPart(part))?;
        let link = p.link.clone().ok_or_else(|| {
            AssemblyError::Invalid(format!("'{}' is not linked to a file", p.name))
        })?;
        let path = link.resolve(base_dir);
        let here = std::fs::canonicalize(&path).unwrap_or(path.clone());
        if stack.contains(&here) {
            return Err(AssemblyError::Invalid(circle(stack, &here)));
        }
        let loaded = link.read(base_dir).map_err(|s| {
            AssemblyError::Invalid(match s {
                LinkStatus::Missing => format!(
                    "the file '{}' linked to '{}' is not there",
                    link.abs, p.name
                ),
                LinkStatus::Unreadable(e) => {
                    format!("the file linked to '{}' cannot be read: {e}", p.name)
                }
                _ => unreachable!("a read fails with Missing or Unreadable"),
            })
        })?;
        let changed;
        let hash;
        match loaded {
            Loaded::Model(doc) => {
                hash = hash_hex(&doc);
                changed = doc != p.document || p.sub.is_some();
                p.document = doc;
                p.sub = None;
            }
            Loaded::Assembly(mut asm, h) => {
                // bring the files linked inside it up to date, relative to its own folder
                stack.push(here);
                let inner = asm.update_links_in(path.parent(), stack);
                let after = asm.link_status_in(path.parent(), stack);
                stack.pop();
                if let Some((_, Err(e))) = inner.into_iter().find(|(_, r)| r.is_err()) {
                    return Err(e);
                }
                if let Some(m) = after.iter().find_map(|(_, s)| match s {
                    LinkStatus::Unreadable(m) if m.contains("in a circle") => Some(m.clone()),
                    _ => None,
                }) {
                    return Err(AssemblyError::Invalid(m));
                }
                hash = h;
                changed = p.sub.as_deref() != Some(&*asm);
                p.sub = Some(asm);
                p.document = Document::new();
            }
        }
        if let Some(l) = p.link.as_mut() {
            l.source_hash = hash;
        }
        Ok(UpdateReport {
            part: p.name.clone(),
            changed,
        })
    }

    /// Update every linked part whose file has changed. Parts whose file is missing or unreadable keep their
    /// cached model (their status says so).
    pub fn update_links(
        &mut self,
        base_dir: Option<&Path>,
    ) -> Vec<(usize, Result<UpdateReport, AssemblyError>)> {
        self.update_links_in(base_dir, &mut Vec::new())
    }

    fn update_links_in(
        &mut self,
        base_dir: Option<&Path>,
        stack: &mut Vec<PathBuf>,
    ) -> Vec<(usize, Result<UpdateReport, AssemblyError>)> {
        let stale: Vec<usize> = self
            .link_status_in(base_dir, stack)
            .into_iter()
            .filter(|(_, s)| *s == LinkStatus::Stale)
            .map(|(i, _)| i)
            .collect();
        stale
            .into_iter()
            .map(|i| (i, self.update_link_in(i, base_dir, stack)))
            .collect()
    }

    /// Cut part `part` loose from its file: it keeps the model it has.
    pub fn break_link(&mut self, part: usize) -> bool {
        self.parts
            .get_mut(part)
            .map(|p| p.link.take().is_some())
            .unwrap_or(false)
    }

    /// Point part `part` at another file (and take what it holds: a model, or an assembly if that is what
    /// the part is linked as now).
    pub fn relink(
        &mut self,
        part: usize,
        path: &Path,
        base_dir: Option<&Path>,
    ) -> Result<UpdateReport, AssemblyError> {
        let kind = self
            .parts
            .get(part)
            .ok_or(AssemblyError::UnknownPart(part))?
            .link
            .as_ref()
            .map_or(LinkKind::Part, |l| l.kind);
        let file = KwfFile::load(path)
            .map_err(|e| AssemblyError::Invalid(format!("{}: {e}", path.display())))?;
        let link = match kind {
            LinkKind::Part => PartLink::new(path, base_dir, &file.document),
            LinkKind::Assembly => {
                let bytes = file.extra_entries.get("assembly.cbor").ok_or_else(|| {
                    AssemblyError::Invalid(format!("{} holds no assembly", path.display()))
                })?;
                PartLink::to_assembly(path, base_dir, bytes)
            }
        };
        let p = &mut self.parts[part];
        // the new link starts out as a different source: take it through the usual update
        let old = p.link.replace(link);
        match self.update_link(part, base_dir) {
            Ok(r) => Ok(r),
            Err(e) => {
                self.parts[part].link = old;
                Err(e)
            }
        }
    }

    /// The project is being saved in `new_dir` (it was in `old_dir`, if it had been saved): make every
    /// link's relative path lead from there to the same file.
    /// Returns whether any link changed.
    pub fn rebase_links(&mut self, old_dir: Option<&Path>, new_dir: &Path) -> bool {
        let mut changed = false;
        let new_dir = std::fs::canonicalize(new_dir).unwrap_or_else(|_| new_dir.to_path_buf());
        for p in &mut self.parts {
            if let Some(l) = p.link.as_mut() {
                let target = l.resolve(old_dir);
                let target = std::fs::canonicalize(&target).unwrap_or(target);
                let rel = relative_path(&new_dir, &target);
                let abs = target.to_string_lossy().into_owned();
                if l.rel != rel || l.abs != abs {
                    l.rel = rel;
                    l.abs = abs;
                    changed = true;
                }
            }
        }
        changed
    }
}
