//! An open project: the feature document with its undo history and evaluation engine, the saved `.kwf`
//! file it came from, and the Phase 5 data (CAM setup, FEA study, circuit board) that is stored with it.

use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::{BodyOp, Document, Engine, Evaluation, History, PlaneSpec};
use kw_store::{CamSetup, FeaStudy};
use std::path::{Path, PathBuf};
use std::sync::Arc;

pub struct Project {
    pub path: Option<PathBuf>,
    pub history: History,
    pub engine: Engine,
    pub cam: Vec<CamSetup>,
    pub fea: Vec<FeaStudy>,
    pub board: Option<kw_ecad::Board>,
    /// The assembly (components and joints) stored with the project, if there is one.
    pub assembly: Option<kw_assembly::Assembly>,
    /// The 2-D drawing sheets stored with the project.
    pub drawings: kw_drawing::DrawingSet,
    /// Parts of the file that could not be read (the project opened without them), to tell the user.
    pub load_warnings: Vec<String>,
    /// Entries that could not be read: they are kept as they were when the project is saved.
    damaged_entries: Vec<&'static str>,
    /// The container as read, so unknown entries and fields survive a save.
    file: KwfFile,
    saved_hash: u128,
    /// Set by edits to the Phase 5 data, cleared by a save.
    extras_dirty: bool,
    evaluation: Option<(u128, Arc<Evaluation>)>,
    /// While a component is being edited in place: which, and the root design set aside.
    editing: Option<Editing>,
    /// Undo and redo of changes to the assembly (components, joints, motion): earlier states, as stored.
    asm_undo: Vec<Option<Vec<u8>>>,
    asm_redo: Vec<Option<Vec<u8>>>,
    /// Undo and redo of changes to the drawings: earlier states, as stored.
    dw_undo: Vec<Vec<u8>>,
    dw_redo: Vec<Vec<u8>>,
}

/// How many assembly states are kept for undo.
const ASM_UNDO_DEPTH: usize = 64;
/// How many drawing states are kept for undo.
const DW_UNDO_DEPTH: usize = 32;

/// A component opened for editing: the project's own history is parked here and the component's document
/// takes its place.
struct Editing {
    part: usize,
    root: History,
    /// Hash of the component's document as last copied into the assembly.
    synced: u128,
}

fn mm(v: f64) -> Param {
    Param::length(v)
}

/// The folder a project file is in (the working folder for a bare file name).
fn dir_of(path: &Path) -> Option<PathBuf> {
    match path.parent() {
        Some(p) if !p.as_os_str().is_empty() => Some(p.to_path_buf()),
        _ => std::env::current_dir().ok(),
    }
}

/// The value of an optional stored entry; one that cannot be read becomes a warning and `None`.
fn tolerate<T>(
    what: &str,
    entry: &'static str,
    r: Option<Result<T, String>>,
    warnings: &mut Vec<String>,
    damaged: &mut Vec<&'static str>,
) -> Option<T> {
    match r {
        Some(Ok(v)) => Some(v),
        Some(Err(e)) => {
            warnings.push(format!(
                "the {what} data could not be read ({e}) and was left out"
            ));
            damaged.push(entry);
            None
        }
        None => None,
    }
}

impl Project {
    fn from_file(path: Option<PathBuf>, file: KwfFile) -> Result<Project, String> {
        let mut load_warnings = Vec::new();
        let mut damaged_entries = Vec::new();
        // a damaged CAM or FEA entry costs that data, not the whole project
        let cam = tolerate(
            "CAM",
            kw_store::CAM_ENTRY,
            kw_store::cam_file_from_file(&file),
            &mut load_warnings,
            &mut damaged_entries,
        )
        .map(|f| f.setups)
        .unwrap_or_default();
        let fea = tolerate(
            "FEA",
            kw_store::FEA_ENTRY,
            kw_store::fea_file_from_file(&file),
            &mut load_warnings,
            &mut damaged_entries,
        )
        .map(|f| f.studies)
        .unwrap_or_default();
        let board = kw_store::board_from_file(&file).transpose()?;
        let mut assembly = kw_assembly::Assembly::from_file(&file).transpose()?;
        // linked files are found again from where the project is now (it may have been moved)
        if let (Some(a), Some(dir)) = (assembly.as_mut(), path.as_deref().and_then(dir_of)) {
            a.rebase_links(Some(&dir), &dir);
        }
        let drawings = kw_store::drawings_from_file(&file)
            .transpose()?
            .unwrap_or_default();
        let doc = file.document.clone();
        let saved_hash = doc.content_hash();
        Ok(Project {
            path,
            history: History::new(doc),
            engine: Engine::new(),
            cam,
            fea,
            board,
            assembly,
            drawings,
            load_warnings,
            damaged_entries,
            file,
            saved_hash,
            extras_dirty: false,
            evaluation: None,
            editing: None,
            asm_undo: Vec::new(),
            asm_redo: Vec::new(),
            dw_undo: Vec::new(),
            dw_redo: Vec::new(),
        })
    }

    /// An empty project that has not been saved yet.
    pub fn empty() -> Project {
        Project::from_file(None, KwfFile::new(Document::new())).expect("empty project")
    }

    /// A small part to look around in: a 40 × 25 × 10 mm block.
    pub fn demo() -> Project {
        let mut doc = Document::new();
        let s = doc
            .add_rect_sketch(
                "Profile",
                PlaneSpec::Xy(mm(0.0)),
                mm(0.0),
                mm(0.0),
                mm(40.0),
                mm(25.0),
            )
            .expect("demo sketch");
        doc.add_extrude("Block", s, mm(10.0), BodyOp::New)
            .expect("demo extrude");
        Project::from_file(None, KwfFile::new(doc)).expect("demo project")
    }

    pub fn open(path: &Path) -> Result<Project, String> {
        let file = KwfFile::load(path).map_err(|e| e.to_string())?;
        Project::from_file(Some(path.to_path_buf()), file)
    }

    /// The folder the project is saved in, if it has been saved (linked files are relative to it).
    pub fn base_dir(&self) -> Option<PathBuf> {
        self.path.as_deref().and_then(dir_of)
    }

    /// File name without the extension, or "Untitled".
    pub fn name(&self) -> String {
        self.path
            .as_ref()
            .and_then(|p| p.file_stem())
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Untitled".into())
    }

    pub fn document(&self) -> Arc<Document> {
        self.history.current()
    }

    /// The project's own design, also while a component is being edited.
    pub fn root_document(&self) -> Arc<Document> {
        match &self.editing {
            Some(e) => e.root.current(),
            None => self.history.current(),
        }
    }

    pub fn is_dirty(&self) -> bool {
        self.extras_dirty || self.root_document().content_hash() != self.saved_hash
    }

    fn asm_bytes(&self) -> Option<Vec<u8>> {
        self.assembly.as_ref().map(|a| a.to_bytes())
    }

    /// Remember the assembly as it is now, before a change (Undo in the assembly view goes back to it).
    pub fn asm_checkpoint(&mut self) {
        let now = self.asm_bytes();
        self.asm_checkpoint_with(now);
    }

    /// Remember `before` (a state taken earlier) as the state to undo to.
    pub fn asm_checkpoint_with(&mut self, before: Option<Vec<u8>>) {
        self.asm_undo.push(before);
        if self.asm_undo.len() > ASM_UNDO_DEPTH {
            self.asm_undo.remove(0);
        }
        self.asm_redo.clear();
    }

    /// Forget the last checkpoint if nothing changed since (a drag that moved nothing).
    pub fn asm_drop_checkpoint_if_unchanged(&mut self) {
        if self.asm_undo.last() == Some(&self.asm_bytes()) {
            self.asm_undo.pop();
        }
    }

    pub fn asm_can_undo(&self) -> bool {
        !self.asm_undo.is_empty()
    }

    pub fn asm_can_redo(&self) -> bool {
        !self.asm_redo.is_empty()
    }

    fn asm_restore(&mut self, state: Option<Vec<u8>>) -> bool {
        let assembly = match state {
            None => None,
            Some(b) => match kw_assembly::Assembly::from_bytes(&b) {
                Ok(a) => Some(a),
                Err(_) => return false,
            },
        };
        self.assembly = assembly;
        self.extras_dirty = true;
        true
    }

    /// Undo the last change to the assembly.
    pub fn asm_undo(&mut self) -> bool {
        let Some(prev) = self.asm_undo.pop() else {
            return false;
        };
        let now = self.asm_bytes();
        if self.asm_restore(prev) {
            self.asm_redo.push(now);
            true
        } else {
            false
        }
    }

    /// Redo a change to the assembly that was undone.
    pub fn asm_redo(&mut self) -> bool {
        let Some(next) = self.asm_redo.pop() else {
            return false;
        };
        let now = self.asm_bytes();
        if self.asm_restore(next) {
            self.asm_undo.push(now);
            true
        } else {
            false
        }
    }

    /// The component being edited in place (an index into the assembly's parts).
    pub fn editing(&self) -> Option<usize> {
        self.editing.as_ref().map(|e| e.part)
    }

    /// Open component `part` for editing: its document becomes the design until [`finish_component`].
    /// The undo history of the design is kept aside and comes back with it.
    pub fn activate_component(&mut self, part: usize) -> Result<(), String> {
        if self.editing.is_some() {
            self.finish_component()?;
        }
        let asm = self.assembly.as_ref().ok_or("there is no assembly")?;
        let p = asm.parts.get(part).ok_or("there is no such component")?;
        if p.sub.is_some() {
            return Err(
                "a sub-assembly has no design of its own; open one of its components".into(),
            );
        }
        if p.link.is_some() {
            return Err(format!(
                "'{}' is linked to a file: edit that file, or break the link to edit the copy here",
                p.name
            ));
        }
        let doc = p.document.clone();
        let synced = doc.content_hash();
        let root = std::mem::replace(&mut self.history, History::new(doc));
        self.editing = Some(Editing { part, root, synced });
        self.evaluation = None;
        Ok(())
    }

    /// Copy the component being edited into the assembly if it changed since the last copy (so the assembly
    /// view follows every edit). Returns whether it changed.
    pub fn sync_component(&mut self) -> bool {
        let Some(e) = self.editing.as_mut() else {
            return false;
        };
        let doc = self.history.current();
        let h = doc.content_hash();
        if h == e.synced {
            return false;
        }
        e.synced = h;
        if let Some(p) = self.assembly.as_mut().and_then(|a| a.parts.get_mut(e.part)) {
            p.document = (*doc).clone();
        }
        self.extras_dirty = true;
        true
    }

    /// Close the component being edited and bring the project's own design back. Returns the part's index.
    pub fn finish_component(&mut self) -> Result<usize, String> {
        self.sync_component();
        let e = self.editing.take().ok_or("no component is being edited")?;
        self.history = e.root;
        self.evaluation = None;
        Ok(e.part)
    }

    /// Remember the drawings as they are now, before a change (Undo in the Drawing workspace goes back to it).
    pub fn dw_checkpoint(&mut self) {
        self.dw_undo.push(self.drawings.to_bytes());
        if self.dw_undo.len() > DW_UNDO_DEPTH {
            self.dw_undo.remove(0);
        }
        self.dw_redo.clear();
    }

    /// Forget the last checkpoint if nothing changed since (a drag that moved nothing).
    pub fn dw_drop_checkpoint_if_unchanged(&mut self) {
        if self.dw_undo.last() == Some(&self.drawings.to_bytes()) {
            self.dw_undo.pop();
        }
    }

    pub fn dw_can_undo(&self) -> bool {
        !self.dw_undo.is_empty()
    }

    pub fn dw_can_redo(&self) -> bool {
        !self.dw_redo.is_empty()
    }

    fn dw_restore(&mut self, state: &[u8]) -> bool {
        match kw_drawing::DrawingSet::from_bytes(state) {
            Ok(d) => {
                self.drawings = d;
                self.extras_dirty = true;
                true
            }
            Err(_) => false,
        }
    }

    /// Undo the last change to the drawings.
    pub fn dw_undo(&mut self) -> bool {
        let Some(prev) = self.dw_undo.pop() else {
            return false;
        };
        let now = self.drawings.to_bytes();
        if self.dw_restore(&prev) {
            self.dw_redo.push(now);
            true
        } else {
            false
        }
    }

    /// Redo a change to the drawings that was undone.
    pub fn dw_redo(&mut self) -> bool {
        let Some(next) = self.dw_redo.pop() else {
            return false;
        };
        let now = self.drawings.to_bytes();
        if self.dw_restore(&next) {
            self.dw_undo.push(now);
            true
        } else {
            false
        }
    }

    pub fn mark_extras_dirty(&mut self) {
        self.extras_dirty = true;
    }

    /// Save to `path` (which becomes the project's path).
    pub fn save_as(&mut self, path: &Path) -> Result<(), String> {
        // a component being edited is stored with the assembly, the design is the project's own
        self.sync_component();
        let doc = (*self.root_document()).clone();
        self.file.document = doc;
        // stale entries are removed, current ones attached
        for name in [
            kw_store::CAM_ENTRY,
            kw_store::FEA_ENTRY,
            kw_store::BOARD_ENTRY,
            kw_store::DRAWING_ENTRY,
        ] {
            // an entry that could not be read is kept as it was
            if !self.damaged_entries.contains(&name) {
                self.file.extra_entries.remove(name);
            }
        }
        if !self.cam.is_empty() {
            kw_store::attach_cam_file(
                &mut self.file,
                &kw_store::CamFile {
                    version: kw_store::STORE_VERSION,
                    setups: self.cam.clone(),
                },
            );
        }
        if !self.fea.is_empty() {
            kw_store::attach_fea_file(
                &mut self.file,
                &kw_store::FeaFile {
                    version: kw_store::STORE_VERSION,
                    studies: self.fea.clone(),
                },
            );
        }
        if let Some(b) = &self.board {
            kw_store::attach_board(&mut self.file, b);
        }
        if !self.drawings.is_empty() {
            kw_store::attach_drawings(&mut self.file, &self.drawings);
        }
        self.file.extra_entries.remove("assembly.cbor");
        // links are kept relative to where the project is saved now
        if let (Some(new_dir), Some(a)) = (dir_of(path), self.assembly.as_mut()) {
            let old_dir = self.path.as_deref().and_then(dir_of);
            a.rebase_links(old_dir.as_deref(), &new_dir);
        }
        if let Some(a) = &self.assembly {
            a.attach(&mut self.file);
        }
        self.file.save(path).map_err(|e| e.to_string())?;
        self.path = Some(path.to_path_buf());
        self.saved_hash = self.root_document().content_hash();
        self.extras_dirty = false;
        Ok(())
    }

    pub fn save(&mut self) -> Result<(), String> {
        match self.path.clone() {
            Some(p) => self.save_as(&p),
            None => Err("the project has no file yet".into()),
        }
    }

    /// Evaluate the current document (cached while it does not change).
    pub fn evaluation(&mut self) -> Arc<Evaluation> {
        let doc = self.document();
        let h = doc.content_hash();
        if let Some((eh, ev)) = &self.evaluation {
            if *eh == h {
                return ev.clone();
            }
        }
        let ev = Arc::new(self.engine.evaluate(&doc));
        self.evaluation = Some((h, ev.clone()));
        ev
    }
}
