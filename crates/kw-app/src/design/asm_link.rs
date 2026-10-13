//! Components linked to external `.kwf` files: noticing that a file changed, updating the component, and
//! breaking or moving the link.
//!
//! A linked part keeps a cached copy of the file's model in the project, so the assembly always solves; the
//! files are looked at when the project is opened, and then about every second while the Design workspace
//! is showing (the modification time is compared, and only a file that changed is read again).

use super::assembly::AsmAction;
use super::{Action, DesignWorkspace};
use crate::project::Project;
use eframe::egui;
use kw_assembly::LinkStatus;
use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, SystemTime};

/// What the application knows about the files of the linked components.
pub struct LinkWatch {
    /// How each linked part (index into the assembly's parts) stood at the last look.
    pub status: Vec<(usize, LinkStatus)>,
    /// The files as last seen: modification time and size (`None`: not there).
    seen: HashMap<PathBuf, Option<(SystemTime, u64)>>,
    frames: u32,
    /// Look at the files every this many frames.
    pub poll_every: u32,
    looked: bool,
}

impl Default for LinkWatch {
    fn default() -> Self {
        LinkWatch {
            status: Vec::new(),
            seen: HashMap::new(),
            frames: 0,
            poll_every: 60,
            looked: false,
        }
    }
}

impl LinkWatch {
    /// How part `part` stands, if it is linked.
    pub fn of(&self, part: usize) -> Option<&LinkStatus> {
        self.status.iter().find(|(p, _)| *p == part).map(|(_, s)| s)
    }

    /// The parts whose file has changed since the model was taken.
    pub fn stale(&self) -> Vec<usize> {
        self.status
            .iter()
            .filter(|(_, s)| *s == LinkStatus::Stale)
            .map(|(p, _)| *p)
            .collect()
    }

    /// Look at every file again the next time round.
    pub fn forget(&mut self) {
        self.looked = false;
    }
}

impl DesignWorkspace {
    /// Look at the linked files now, and say so if some are out of date.
    pub fn asm_links_check(&mut self, project: &Project) {
        let base = project.base_dir();
        let Some(asm) = project.assembly.as_ref() else {
            self.asm.links.status.clear();
            return;
        };
        let before = self.asm.links.stale();
        self.asm.links.status = asm.link_status(base.as_deref());
        let now = self.asm.links.stale();
        if now.iter().any(|p| !before.contains(p)) && !asm.auto_update_links {
            self.status = format!(
                "{} linked component{} out of date: Update Links takes the changes.",
                now.len(),
                if now.len() == 1 { " is" } else { "s are" }
            );
        }
    }

    /// Called on every frame of the Design workspace: notice linked files that changed, and update them
    /// when the assembly says to.
    pub fn asm_links_tick(&mut self, ctx: &egui::Context, project: &mut Project) {
        let base = project.base_dir();
        let files: Vec<PathBuf> = project
            .assembly
            .as_ref()
            .map(|a| {
                a.parts
                    .iter()
                    .filter_map(|p| p.link.as_ref().map(|l| l.resolve(base.as_deref())))
                    .collect()
            })
            .unwrap_or_default();
        if files.is_empty() {
            self.asm.links.status.clear();
            self.asm.links.seen.clear();
            self.asm.links.looked = true;
            return;
        }
        let w = &mut self.asm.links;
        w.frames = w.frames.wrapping_add(1);
        if w.looked && !w.frames.is_multiple_of(w.poll_every.max(1)) {
            ctx.request_repaint_after(Duration::from_millis(1000));
            return;
        }
        let mut changed = !w.looked || w.seen.len() != files.len();
        for f in &files {
            let now = std::fs::metadata(f)
                .ok()
                .and_then(|m| m.modified().ok().map(|t| (t, m.len())));
            if w.seen.insert(f.clone(), now) != Some(now) {
                changed = true;
            }
        }
        w.looked = true;
        if changed {
            self.asm_links_check(project);
            let auto = project
                .assembly
                .as_ref()
                .is_some_and(|a| a.auto_update_links);
            if auto && !self.asm.links.stale().is_empty() {
                self.actions.push(Action::Asm(AsmAction::UpdateLinks(None)));
            }
        }
        ctx.request_repaint_after(Duration::from_millis(1000));
    }
}
