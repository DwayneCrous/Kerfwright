//! The assembly in the browser: components (with sub-assemblies below them), joints and rigid groups, and the
//! commands the tree and its menus run on the assembly.

use super::assembly::{AsmAction, FileRequest};
use super::browser::row;
use super::{Action, DesignWorkspace};
use crate::chrome::theme::{ACCENT, BAD, GOOD, TEXT_WEAK};
use crate::chrome::Icon;
use crate::project::Project;
use eframe::egui::{self, Ui};
use kw_assembly::{Assembly, LinkStatus};

impl DesignWorkspace {
    // --------------------------------------------------------------------------------- commands

    /// Run a command on the project's assembly. A change that leaves the assembly unsolvable is taken back
    /// and explained.
    pub fn asm_action(&mut self, project: &mut Project, action: AsmAction) {
        // commands that open something or work on the design need no change to the assembly
        match &action {
            AsmAction::OpenJoint(as_built) => {
                if let Err(e) = self.asm_open_joint(project, *as_built) {
                    self.model.error = e;
                }
                return;
            }
            AsmAction::OpenRigid => {
                let Some(asm) = project.assembly.as_ref() else {
                    self.model.error = "add components first".into();
                    return;
                };
                self.asm.rigid = Some(
                    (0..asm.instances.len())
                        .map(|i| self.asm.selected.contains(&i))
                        .collect(),
                );
                self.asm.view = true;
                return;
            }
            AsmAction::OpenBom => {
                self.asm.bom_open = true;
                return;
            }
            AsmAction::CheckInterference => {
                self.asm_run_interference(project);
                return;
            }
            AsmAction::ComponentFromActiveBody => {
                let doc = project.document();
                let bodies = crate::bodies::summaries(&doc);
                let head = self
                    .active_body
                    .and_then(|r| bodies.iter().find(|b| b.root == r))
                    .or_else(|| (bodies.len() == 1).then(|| &bodies[0]))
                    .map(|b| b.head);
                match head {
                    Some(h) => {
                        if let Err(e) = self.asm_component_from_body(project, h) {
                            self.model.error = e;
                        }
                    }
                    None => {
                        self.model.error = "choose a body first (click it in the browser)".into();
                    }
                }
                return;
            }
            AsmAction::ActivateSelected => {
                let part = project.assembly.as_ref().and_then(|a| {
                    (self.asm.selected.len() == 1)
                        .then(|| a.instances.get(self.asm.selected[0]).map(|i| i.part))
                        .flatten()
                });
                if let Some(p) = part {
                    self.actions.push(Action::ActivateComponent(p));
                }
                return;
            }
            _ => {}
        }
        let project_editing = project.editing();
        let base = project.base_dir();
        let Some(asm) = project.assembly.as_mut() else {
            return;
        };
        let backup = asm.clone();
        let n = asm.instances.len();
        let mut changed = true;
        let mut note = String::new();
        match action {
            AsmAction::Select(i, extend) => {
                changed = false;
                if i < n {
                    if extend {
                        if let Some(k) = self.asm.selected.iter().position(|s| *s == i) {
                            self.asm.selected.remove(k);
                        } else {
                            self.asm.selected.push(i);
                        }
                    } else {
                        self.asm.selected = vec![i];
                    }
                    self.asm.selected_joint = None;
                }
            }
            AsmAction::SelectJoint(j) => {
                changed = false;
                if j < asm.joints.len() {
                    self.asm.selected_joint = Some(j);
                    self.asm.selected.clear();
                }
            }
            AsmAction::ToggleHidden(i) => {
                if let Some(inst) = asm.instances.get_mut(i) {
                    inst.hidden = !inst.hidden;
                }
            }
            AsmAction::Isolate(i) => {
                for (k, inst) in asm.instances.iter_mut().enumerate() {
                    inst.hidden = k != i;
                }
            }
            AsmAction::ShowAll => {
                for inst in &mut asm.instances {
                    inst.hidden = false;
                }
            }
            AsmAction::GroundSelected => {
                let sel: Vec<usize> = self
                    .asm
                    .selected
                    .iter()
                    .copied()
                    .filter(|i| *i < n)
                    .collect();
                if !sel.is_empty() {
                    let all = sel.iter().all(|i| asm.is_grounded(*i));
                    for i in &sel {
                        asm.set_grounded(*i, !all);
                    }
                    note = format!(
                        "{} component{} {}.",
                        sel.len(),
                        if sel.len() == 1 { "" } else { "s" },
                        if all { "released" } else { "grounded" }
                    );
                }
            }
            AsmAction::OpenJoint(_)
            | AsmAction::OpenRigid
            | AsmAction::OpenBom
            | AsmAction::CheckInterference
            | AsmAction::ComponentFromActiveBody
            | AsmAction::ActivateSelected => changed = false,
            AsmAction::ToggleGround(i) => {
                if i < n {
                    let on = !asm.is_grounded(i);
                    asm.set_grounded(i, on);
                    note = format!(
                        "{} {}.",
                        asm.instances[i].name,
                        if on { "is grounded" } else { "is released" }
                    );
                }
            }
            AsmAction::Copy(i) => {
                if i < n {
                    asm.snapshot_placements();
                    let src = asm.instances[i].clone();
                    let name = asm.unique_instance_name(&format!("{} copy", src.name));
                    let k = asm.add_instance(&name, src.part);
                    let mut at = src.at;
                    at.t[0] += 15.0;
                    asm.instances[k].at = at;
                    self.asm.selected = vec![k];
                    note = format!("{name} added next to {}.", src.name);
                }
            }
            AsmAction::Delete(i) => {
                if i < n {
                    let name = asm.instances[i].name.clone();
                    asm.remove_instance(i);
                    // a part nothing uses any more goes too (not while a component is open: its index is held)
                    if project_editing.is_none() {
                        asm.prune_unused_parts();
                    }
                    self.asm.selected.clear();
                    self.asm.hover = None;
                    note = format!("{name} deleted with its joints.");
                }
            }
            AsmAction::DeleteJoint(j) => {
                if j < asm.joints.len() {
                    let name = asm.joints[j].name.clone();
                    asm.remove_joint(j);
                    self.asm.selected_joint = None;
                    self.asm.edit_joint = None;
                    note = format!("{name} deleted; the components stay where they are.");
                }
            }
            AsmAction::DeleteGroup(g) => {
                if g < asm.groups.len() {
                    let name = asm.groups[g].name.clone();
                    asm.remove_group(g);
                    note = format!("{name} dissolved.");
                }
            }
            AsmAction::RigidGroupFromSelection => {
                let name = format!("Rigid Group {}", asm.groups.len() + 1);
                match asm.make_rigid_group(&name, &self.asm.selected) {
                    Ok(_) => note = format!("{name} made: its components now move as one."),
                    Err(e) => {
                        self.model.error = e.to_string();
                        return;
                    }
                }
            }
            AsmAction::SubAssemblyFromSelection => {
                let name = format!(
                    "Sub-assembly {}",
                    asm.parts.iter().filter(|p| p.sub.is_some()).count() + 1
                );
                match asm.group_into_subassembly(&self.asm.selected, &name) {
                    Ok(k) => {
                        if project_editing.is_none() {
                            asm.prune_unused_parts();
                        }
                        self.asm.selected = vec![k];
                        note = format!("{name} made.");
                    }
                    Err(e) => {
                        self.model.error = e.to_string();
                        return;
                    }
                }
            }
            AsmAction::Rename(i, new) => {
                let new = new.trim().to_string();
                if i < n && !new.is_empty() {
                    if asm
                        .instances
                        .iter()
                        .enumerate()
                        .any(|(k, x)| k != i && x.name == new)
                    {
                        self.model.error = format!("another component is already called '{new}'");
                        return;
                    }
                    asm.instances[i].name = new;
                }
            }
            AsmAction::UpdateLinks(only) => {
                let reports = match only {
                    Some(p) => vec![(p, asm.update_link(p, base.as_deref()))],
                    None => asm.update_links(base.as_deref()),
                };
                let mut taken = Vec::new();
                let mut failed = Vec::new();
                for (_, r) in reports {
                    match r {
                        Ok(u) if u.changed => taken.push(u.part),
                        Ok(_) => {}
                        Err(e) => failed.push(e.to_string()),
                    }
                }
                if only.is_some() && !failed.is_empty() {
                    *asm = backup;
                    self.model.error = failed.remove(0);
                    self.asm_links_check(project);
                    return;
                }
                if !failed.is_empty() {
                    self.model.error = failed.join("; ");
                }
                changed = !taken.is_empty();
                if changed {
                    note = format!("Updated from file: {}.", taken.join(", "));
                }
            }
            AsmAction::BreakLink(p) => {
                changed = asm.break_link(p);
                if changed {
                    note = "The link is cut: the component keeps the model it has.".into();
                }
            }
            AsmAction::Relink(p, path) => match asm.relink(p, &path, base.as_deref()) {
                Ok(u) => note = format!("{} is now linked to {}.", u.part, path.display()),
                Err(e) => {
                    *asm = backup;
                    self.model.error = e.to_string();
                    return;
                }
            },
            AsmAction::ToggleAutoUpdate => {
                asm.auto_update_links = !asm.auto_update_links;
                // look at the files again: with it on, links that are already out of date are taken
                self.asm.links.forget();
                note = if asm.auto_update_links {
                    "Linked components now update when their files change.".into()
                } else {
                    "Linked components are only marked out of date.".into()
                };
            }
            AsmAction::RenameJoint(j, new) => {
                let new = new.trim().to_string();
                if j < asm.joints.len() && !new.is_empty() {
                    asm.joints[j].name = new;
                }
            }
        }
        if !changed {
            self.asm_links_check(project);
            return;
        }
        // a joint of a loop that cannot close any more, or a model that no longer evaluates, is refused
        if let Err(e) = asm.solve() {
            *asm = backup;
            self.model.error = e.to_string();
            self.asm_links_check(project);
            return;
        }
        if !note.is_empty() {
            self.status = note;
        }
        project.asm_checkpoint_with(Some(backup.to_bytes()));
        project.mark_extras_dirty();
        self.asm.overlaps = None;
        self.asm_links_check(project);
    }

    // -------------------------------------------------------------------------------------- tree

    /// The assembly section of the browser.
    pub(super) fn assembly_tree(&mut self, ui: &mut Ui, project: &Project) {
        let Some(asm) = project.assembly.as_ref() else {
            return;
        };
        self.asm.hover = None;
        let extend = ui.input(|i| i.modifiers.shift || i.modifiers.command);
        let editing = project.editing();
        egui::CollapsingHeader::new(
            egui::RichText::new(format!("Components ({})", asm.instances.len())).size(12.0),
        )
        .default_open(true)
        .show(ui, |ui| {
            for (i, inst) in asm.instances.iter().enumerate() {
                let part = asm.parts.get(inst.part);
                let being_edited = editing == Some(inst.part);
                let selected = self.asm.selected.contains(&i);
                let grounded = asm.is_grounded(i);
                let label = if being_edited {
                    format!("{} (editing)", inst.name)
                } else {
                    inst.name.clone()
                };
                let (resp, eye) = row(
                    ui,
                    Icon::Component,
                    &label,
                    selected,
                    Some(!inst.hidden),
                    false,
                    inst.hidden,
                );
                if grounded {
                    // a small "G" badge: this component does not move
                    let c = egui::pos2(resp.rect.right() - 12.0, resp.rect.center().y);
                    ui.painter().circle_filled(c, 6.5, ACCENT);
                    ui.painter().text(
                        c,
                        egui::Align2::CENTER_CENTER,
                        "G",
                        egui::FontId::proportional(9.0),
                        egui::Color32::WHITE,
                    );
                }
                // a linked component: a badge for how its file stands
                let link = part.and_then(|p| p.link.as_ref());
                let state = link.and_then(|_| self.asm.links.of(inst.part)).cloned();
                if let (Some(l), Some(st)) = (link, &state) {
                    let (colour, mark, what) = match st {
                        LinkStatus::Current => (GOOD, "L", "up to date with its file".to_string()),
                        LinkStatus::Stale => (
                            egui::Color32::from_rgb(222, 140, 20),
                            "!",
                            "its file has changed: Update takes the changes".to_string(),
                        ),
                        LinkStatus::Missing => (
                            BAD,
                            "?",
                            "its file is missing: the copy in the project is used".to_string(),
                        ),
                        LinkStatus::Unreadable(e) => {
                            (BAD, "!", format!("its file cannot be read: {e}"))
                        }
                    };
                    let c = egui::pos2(
                        resp.rect.right() - if grounded { 30.0 } else { 12.0 },
                        resp.rect.center().y,
                    );
                    ui.painter().circle_filled(c, 6.5, colour);
                    ui.painter().text(
                        c,
                        egui::Align2::CENTER_CENTER,
                        mark,
                        egui::FontId::proportional(9.0),
                        egui::Color32::WHITE,
                    );
                    resp.clone()
                        .on_hover_text(format!("Linked to {}: {what}", l.abs));
                }
                if resp.hovered() {
                    self.asm.hover = Some(i);
                }
                if eye {
                    self.actions.push(Action::Asm(AsmAction::ToggleHidden(i)));
                }
                if resp.clicked() && !eye {
                    self.actions.push(Action::Asm(AsmAction::Select(i, extend)));
                }
                let is_sub = part.is_some_and(|p| p.sub.is_some());
                if resp.double_clicked() && !is_sub {
                    self.actions.push(Action::ActivateComponent(inst.part));
                }
                let multi = self.asm.selected.len() >= 2 && selected;
                let part_idx = inst.part;
                let name = inst.name.clone();
                resp.context_menu(|ui| {
                    if !is_sub {
                        let t = if being_edited {
                            "Finish editing"
                        } else {
                            "Activate (edit in place)"
                        };
                        if ui.button(t).clicked() {
                            self.actions.push(if being_edited {
                                Action::FinishComponent
                            } else {
                                Action::ActivateComponent(part_idx)
                            });
                            ui.close();
                        }
                    }
                    if ui
                        .button(if grounded { "Unground" } else { "Ground" })
                        .clicked()
                    {
                        self.actions.push(Action::Asm(AsmAction::ToggleGround(i)));
                        ui.close();
                    }
                    if ui.button("Rename…").clicked() {
                        self.asm.rename = Some((i, name.clone()));
                        ui.close();
                    }
                    if ui.button("Copy").clicked() {
                        self.actions.push(Action::Asm(AsmAction::Copy(i)));
                        ui.close();
                    }
                    if let Some(l) = link {
                        ui.separator();
                        ui.label(
                            egui::RichText::new(format!(
                                "Linked: {}",
                                l.rel.as_deref().unwrap_or(&l.abs)
                            ))
                            .size(10.5)
                            .color(TEXT_WEAK),
                        );
                        let stale = state.as_ref() == Some(&LinkStatus::Stale);
                        if ui
                            .add_enabled(stale, egui::Button::new("Update from file"))
                            .clicked()
                        {
                            self.actions
                                .push(Action::Asm(AsmAction::UpdateLinks(Some(part_idx))));
                            ui.close();
                        }
                        if ui.button("Link to another file…").clicked() {
                            self.asm.request = Some(FileRequest::Relink(part_idx));
                            ui.close();
                        }
                        if ui.button("Break link").clicked() {
                            self.actions
                                .push(Action::Asm(AsmAction::BreakLink(part_idx)));
                            ui.close();
                        }
                    }
                    ui.separator();
                    if ui.button("Isolate").clicked() {
                        self.actions.push(Action::Asm(AsmAction::Isolate(i)));
                        ui.close();
                    }
                    if ui.button("Show all").clicked() {
                        self.actions.push(Action::Asm(AsmAction::ShowAll));
                        ui.close();
                    }
                    if multi {
                        ui.separator();
                        if ui.button("Create rigid group").clicked() {
                            self.actions
                                .push(Action::Asm(AsmAction::RigidGroupFromSelection));
                            ui.close();
                        }
                        if ui.button("Group into sub-assembly").clicked() {
                            self.actions
                                .push(Action::Asm(AsmAction::SubAssemblyFromSelection));
                            ui.close();
                        }
                    }
                    ui.separator();
                    if ui.button("Delete (with its joints)").clicked() {
                        self.actions.push(Action::Asm(AsmAction::Delete(i)));
                        ui.close();
                    }
                });
                if let Some(sub) = part.and_then(|p| p.sub.as_deref()) {
                    ui.indent(("sub", i), |ui| sub_rows(ui, sub));
                }
            }
        });
        if !asm.joints.is_empty() {
            egui::CollapsingHeader::new(
                egui::RichText::new(format!("Joints ({})", asm.joints.len())).size(12.0),
            )
            .default_open(true)
            .show(ui, |ui| {
                let scope_ok = asm.scope();
                for (j, joint) in asm.joints.iter().enumerate() {
                    let pn = asm
                        .instances
                        .get(joint.parent.instance)
                        .map_or("?", |x| x.name.as_str());
                    let cn = asm
                        .instances
                        .get(joint.child.instance)
                        .map_or("?", |x| x.name.as_str());
                    let bad = scope_ok
                        .as_ref()
                        .map(|sc| asm.joint_values(joint, sc).is_err())
                        .unwrap_or(true);
                    let (resp, _) = row(
                        ui,
                        Icon::Joint,
                        &format!("{}: {pn} to {cn}", joint.name),
                        self.asm.selected_joint == Some(j),
                        None,
                        bad,
                        false,
                    );
                    resp.clone().on_hover_text(format!(
                        "{} joint{}{}",
                        joint.kind.name(),
                        if joint.as_built { " (as-built)" } else { "" },
                        if joint.solved {
                            ", solved by a loop"
                        } else {
                            ""
                        }
                    ));
                    if resp.clicked() {
                        self.actions.push(Action::Asm(AsmAction::SelectJoint(j)));
                    }
                    if resp.double_clicked() {
                        self.asm.edit_joint = Some(j);
                    }
                    let name = joint.name.clone();
                    resp.context_menu(|ui| {
                        if ui.button("Edit…").clicked() {
                            self.asm.edit_joint = Some(j);
                            ui.close();
                        }
                        if ui.button("Rename…").clicked() {
                            self.asm.rename_joint = Some((j, name.clone()));
                            ui.close();
                        }
                        if ui.button("Delete").clicked() {
                            self.actions.push(Action::Asm(AsmAction::DeleteJoint(j)));
                            ui.close();
                        }
                    });
                }
            });
        }
        if !asm.groups.is_empty() {
            egui::CollapsingHeader::new(
                egui::RichText::new(format!("Rigid Groups ({})", asm.groups.len())).size(12.0),
            )
            .default_open(true)
            .show(ui, |ui| {
                for (g, group) in asm.groups.iter().enumerate() {
                    let members: Vec<&str> = group
                        .members
                        .iter()
                        .filter_map(|m| asm.instances.get(*m).map(|x| x.name.as_str()))
                        .collect();
                    let (resp, _) = row(
                        ui,
                        Icon::Pattern,
                        &format!("{} ({})", group.name, group.members.len()),
                        false,
                        None,
                        false,
                        false,
                    );
                    resp.clone().on_hover_text(members.join(", "));
                    if resp.clicked() {
                        self.asm.selected = group.members.clone();
                        self.asm.selected_joint = None;
                    }
                    resp.context_menu(|ui| {
                        if ui.button("Dissolve group").clicked() {
                            self.actions.push(Action::Asm(AsmAction::DeleteGroup(g)));
                            ui.close();
                        }
                    });
                }
            });
        }
        if let Err(e) = asm.solve() {
            ui.add_space(2.0);
            ui.label(egui::RichText::new(e.to_string()).size(10.5).color(BAD));
        }
    }

    /// The rename boxes opened from the tree's menus.
    pub(super) fn asm_rename_windows(&mut self, ctx: &egui::Context) {
        if let Some((i, mut text)) = self.asm.rename.take() {
            let mut keep = true;
            let mut apply = false;
            egui::Window::new("Rename component")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    let r = ui.text_edit_singleline(&mut text);
                    r.request_focus();
                    ui.horizontal(|ui| {
                        if ui.button("OK").clicked()
                            || (r.lost_focus() && ui.input(|k| k.key_pressed(egui::Key::Enter)))
                        {
                            apply = true;
                        }
                        if ui.button("Cancel").clicked() {
                            keep = false;
                        }
                    });
                });
            if apply {
                self.actions.push(Action::Asm(AsmAction::Rename(i, text)));
            } else if keep {
                self.asm.rename = Some((i, text));
            }
        }
        if let Some((j, mut text)) = self.asm.rename_joint.take() {
            let mut keep = true;
            let mut apply = false;
            egui::Window::new("Rename joint")
                .collapsible(false)
                .resizable(false)
                .show(ctx, |ui| {
                    let r = ui.text_edit_singleline(&mut text);
                    r.request_focus();
                    ui.horizontal(|ui| {
                        if ui.button("OK").clicked()
                            || (r.lost_focus() && ui.input(|k| k.key_pressed(egui::Key::Enter)))
                        {
                            apply = true;
                        }
                        if ui.button("Cancel").clicked() {
                            keep = false;
                        }
                    });
                });
            if apply {
                self.actions
                    .push(Action::Asm(AsmAction::RenameJoint(j, text)));
            } else if keep {
                self.asm.rename_joint = Some((j, text));
            }
        }
    }
}

/// The components of a sub-assembly, below its row.
fn sub_rows(ui: &mut Ui, sub: &Assembly) {
    for inst in &sub.instances {
        row(ui, Icon::Component, &inst.name, false, None, false, true);
        if let Some(inner) = sub.parts.get(inst.part).and_then(|p| p.sub.as_deref()) {
            ui.indent(("sub", inst.name.as_str()), |ui| sub_rows(ui, inner));
        }
    }
}

impl DesignWorkspace {
    /// The Rigid Group command: tick the components that should move as one.
    pub fn asm_rigid_window(&mut self, ctx: &egui::Context, project: &mut Project) {
        let Some(mut ticks) = self.asm.rigid.take() else {
            return;
        };
        let Some(asm) = project.assembly.as_ref() else {
            return;
        };
        ticks.resize(asm.instances.len(), false);
        let mut ok = false;
        let mut cancel = false;
        egui::Window::new(egui::RichText::new("RIGID GROUP").size(11.5).strong())
            .anchor(egui::Align2::RIGHT_TOP, [-12.0, 312.0])
            .collapsible(false)
            .default_width(240.0)
            .show(ctx, |ui| {
                ui.label(
                    egui::RichText::new(
                        "Tick the components that keep their positions against each other.",
                    )
                    .size(11.0),
                );
                for (i, inst) in asm.instances.iter().enumerate() {
                    let grouped = asm.group_of(i).map(|g| asm.groups[g].name.clone());
                    ui.add_enabled_ui(grouped.is_none(), |ui| {
                        let label = match &grouped {
                            Some(g) => format!("{} (in {g})", inst.name),
                            None => inst.name.clone(),
                        };
                        ui.checkbox(&mut ticks[i], label);
                    });
                }
                ui.horizontal(|ui| {
                    let n = ticks.iter().filter(|t| **t).count();
                    if ui.add_enabled(n >= 2, egui::Button::new("OK")).clicked() {
                        ok = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            cancel = true;
        }
        if ok {
            self.asm.selected = ticks
                .iter()
                .enumerate()
                .filter_map(|(i, t)| t.then_some(i))
                .collect();
            self.actions
                .push(Action::Asm(AsmAction::RigidGroupFromSelection));
            return;
        }
        if !cancel {
            self.asm.rigid = Some(ticks);
        }
    }
}
