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
    pub cam: Option<CamSetup>,
    pub fea: Option<FeaStudy>,
    pub board: Option<kw_ecad::Board>,
    /// The assembly (components and joints) stored with the project, if there is one.
    pub assembly: Option<kw_assembly::Assembly>,
    /// The container as read, so unknown entries and fields survive a save.
    file: KwfFile,
    saved_hash: u128,
    /// Set by edits to the Phase 5 data, cleared by a save.
    extras_dirty: bool,
    evaluation: Option<(u128, Arc<Evaluation>)>,
}

fn mm(v: f64) -> Param {
    Param::length(v)
}

impl Project {
    fn from_file(path: Option<PathBuf>, file: KwfFile) -> Result<Project, String> {
        let cam = kw_store::cam_from_file(&file).transpose()?;
        let fea = kw_store::fea_from_file(&file).transpose()?;
        let board = kw_store::board_from_file(&file).transpose()?;
        let assembly = kw_assembly::Assembly::from_file(&file).transpose()?;
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
            file,
            saved_hash,
            extras_dirty: false,
            evaluation: None,
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

    pub fn is_dirty(&self) -> bool {
        self.extras_dirty || self.document().content_hash() != self.saved_hash
    }

    pub fn mark_extras_dirty(&mut self) {
        self.extras_dirty = true;
    }

    /// Save to `path` (which becomes the project's path).
    pub fn save_as(&mut self, path: &Path) -> Result<(), String> {
        let doc = (*self.document()).clone();
        self.file.document = doc;
        // stale entries are removed, current ones attached
        for name in [
            kw_store::CAM_ENTRY,
            kw_store::FEA_ENTRY,
            kw_store::BOARD_ENTRY,
        ] {
            self.file.extra_entries.remove(name);
        }
        if let Some(c) = &self.cam {
            kw_store::attach_cam(&mut self.file, c);
        }
        if let Some(f) = &self.fea {
            kw_store::attach_fea(&mut self.file, f);
        }
        if let Some(b) = &self.board {
            kw_store::attach_board(&mut self.file, b);
        }
        self.file.extra_entries.remove("assembly.cbor");
        if let Some(a) = &self.assembly {
            a.attach(&mut self.file);
        }
        self.file.save(path).map_err(|e| e.to_string())?;
        self.path = Some(path.to_path_buf());
        self.saved_hash = self.document().content_hash();
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
