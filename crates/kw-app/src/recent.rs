//! The list of recently opened projects, kept as JSON in the user's config directory.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub const MAX_RECENT: usize = 10;

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RecentList {
    pub paths: Vec<PathBuf>,
}

impl RecentList {
    /// Read the list; a missing or damaged file gives an empty list (it is only a convenience).
    pub fn load(file: &Path) -> RecentList {
        std::fs::read(file)
            .ok()
            .and_then(|b| serde_json::from_slice(&b).ok())
            .unwrap_or_default()
    }

    pub fn save(&self, file: &Path) -> std::io::Result<()> {
        if let Some(dir) = file.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let json = serde_json::to_vec_pretty(self).map_err(std::io::Error::other)?;
        std::fs::write(file, json)
    }

    /// Move `path` to the front, drop duplicates, keep at most [`MAX_RECENT`].
    pub fn touch(&mut self, path: PathBuf) {
        self.paths.retain(|p| p != &path);
        self.paths.insert(0, path);
        self.paths.truncate(MAX_RECENT);
    }

    pub fn remove(&mut self, path: &Path) {
        self.paths.retain(|p| p != path);
    }

    /// `$XDG_CONFIG_HOME/kerfwright/recent.json`, or `~/.config/kerfwright/recent.json`.
    pub fn default_file() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .filter(|p| p.is_absolute())
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("kerfwright").join("recent.json"))
    }
}
