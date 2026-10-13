//! File dialogs behind a trait, so the application logic can be driven without a desktop.

use std::path::PathBuf;

pub trait Dialogs {
    /// Choose an existing project file.
    fn open_project(&mut self) -> Option<PathBuf>;
    /// Choose where to save; `suggested` is a file name.
    fn save_project(&mut self, suggested: &str) -> Option<PathBuf>;
    /// Choose a folder for a new project (or for exported files).
    fn choose_folder(&mut self) -> Option<PathBuf>;
    /// Choose an existing file of the given kind (`filter` is a label, `exts` the extensions).
    fn open_file(&mut self, filter: &str, exts: &[&str]) -> Option<PathBuf>;
    /// Choose where to save a file of the given kind (`filter` is a label, `exts` the extensions).
    fn save_file(&mut self, suggested: &str, filter: &str, exts: &[&str]) -> Option<PathBuf>;
}

/// The operating system's dialogs (through `rfd`).
pub struct NativeDialogs;

impl Dialogs for NativeDialogs {
    fn open_project(&mut self) -> Option<PathBuf> {
        rfd::FileDialog::new()
            .set_title("Open project")
            .add_filter("Kerfwright project", &["kwf"])
            .pick_file()
    }

    fn save_project(&mut self, suggested: &str) -> Option<PathBuf> {
        rfd::FileDialog::new()
            .set_title("Save project as")
            .set_file_name(suggested)
            .add_filter("Kerfwright project", &["kwf"])
            .save_file()
    }

    fn choose_folder(&mut self) -> Option<PathBuf> {
        rfd::FileDialog::new()
            .set_title("Choose a folder")
            .pick_folder()
    }

    fn open_file(&mut self, filter: &str, exts: &[&str]) -> Option<PathBuf> {
        rfd::FileDialog::new()
            .set_title("Import")
            .add_filter(filter, exts)
            .pick_file()
    }

    fn save_file(&mut self, suggested: &str, filter: &str, exts: &[&str]) -> Option<PathBuf> {
        rfd::FileDialog::new()
            .set_title("Save")
            .set_file_name(suggested)
            .add_filter(filter, exts)
            .save_file()
    }
}

/// Answers prepared by a test.
#[derive(Default)]
pub struct ScriptedDialogs {
    pub open: Vec<PathBuf>,
    pub save: Vec<PathBuf>,
    pub folder: Vec<PathBuf>,
}

impl Dialogs for ScriptedDialogs {
    fn open_project(&mut self) -> Option<PathBuf> {
        (!self.open.is_empty()).then(|| self.open.remove(0))
    }
    fn save_project(&mut self, _suggested: &str) -> Option<PathBuf> {
        (!self.save.is_empty()).then(|| self.save.remove(0))
    }
    fn choose_folder(&mut self) -> Option<PathBuf> {
        (!self.folder.is_empty()).then(|| self.folder.remove(0))
    }
    fn open_file(&mut self, _filter: &str, _exts: &[&str]) -> Option<PathBuf> {
        (!self.open.is_empty()).then(|| self.open.remove(0))
    }
    fn save_file(&mut self, _suggested: &str, _filter: &str, _exts: &[&str]) -> Option<PathBuf> {
        (!self.save.is_empty()).then(|| self.save.remove(0))
    }
}
