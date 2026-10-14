//! The Libraries window of the CAM workspace: tools with their cutting data, holders, machines and posts.

use super::*;
use kw_cam::library::{CutData, HolderDef, ToolDef};
use kw_cam::post::PostConfig;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tab {
    #[default]
    Tools,
    Holders,
    Machines,
    Posts,
}

#[derive(Default)]
pub struct State {
    tab: Tab,
    filter: String,
    tool: Option<usize>,
    holder: Option<usize>,
    machine: Option<usize>,
    post: Option<usize>,
    /// The text of the post being edited and the error it has.
    post_text: String,
    post_for: Option<usize>,
    pub status: String,
}

/// The kinds of tool the editor offers, with the values a new tool of that kind starts with.
fn kind_choices(d: f64) -> Vec<(&'static str, ToolKind)> {
    vec![
        ("Flat end mill", ToolKind::Flat),
        ("Ball end mill", ToolKind::Ball),
        (
            "Bull-nose end mill",
            ToolKind::Bull {
                corner_radius: (d / 6.0).max(0.1),
            },
        ),
        ("Drill", ToolKind::Drill { point_angle: 118.0 }),
        (
            "Chamfer mill",
            ToolKind::Chamfer {
                tip_angle: 90.0,
                tip_diameter: 0.2,
            },
        ),
        ("Face mill", ToolKind::FaceMill),
        (
            "Engraver",
            ToolKind::Engraver {
                tip_angle: 60.0,
                tip_diameter: 0.1,
            },
        ),
        ("Boring bar", ToolKind::Bore),
        ("Tap", ToolKind::Tap { pitch: 1.0 }),
    ]
}

fn same_kind(a: &ToolKind, b: &ToolKind) -> bool {
    std::mem::discriminant(a) == std::mem::discriminant(b)
}

fn num(ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64) -> bool {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::DragValue::new(v).speed(speed).range(0.0..=1e6))
            .changed()
    })
    .inner
}

impl CamWorkspace {
    /// Open the Libraries window on a tab (0 tools, 1 holders, 2 machines, 3 posts) with the first entry
    /// of each list selected.
    pub fn open_library(&mut self, tab: usize) {
        self.library_open = true;
        self.lib_ui.tab = [Tab::Tools, Tab::Holders, Tab::Machines, Tab::Posts][tab % 4];
        self.lib_ui.tool = Some(0);
        self.lib_ui.holder = Some(0);
        self.lib_ui.machine = Some(0);
        self.lib_ui.post = Some(0);
    }

    /// The Libraries window (drawn when `library_open`).
    pub fn library_window(
        &mut self,
        ctx: &egui::Context,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) {
        if !self.library_open {
            return;
        }
        let mut open = true;
        egui::Window::new("Libraries")
            .open(&mut open)
            .default_width(560.0)
            .default_height(480.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    for (t, label) in [
                        (Tab::Tools, "Tools"),
                        (Tab::Holders, "Holders"),
                        (Tab::Machines, "Machines"),
                        (Tab::Posts, "Posts"),
                    ] {
                        ui.selectable_value(&mut self.lib_ui.tab, t, label);
                    }
                });
                ui.separator();
                match self.lib_ui.tab {
                    Tab::Tools => self.tools_tab(ui, project, dialogs),
                    Tab::Holders => self.holders_tab(ui, dialogs),
                    Tab::Machines => self.machines_tab(ui, project, dialogs),
                    Tab::Posts => self.posts_tab(ui, project, dialogs),
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui
                        .button("Save my library")
                        .on_hover_text(
                            "Write the entries you added or changed to your library folder",
                        )
                        .clicked()
                    {
                        self.lib_ui.status = match self.save_library() {
                            Ok(()) => "saved".into(),
                            Err(e) => format!("could not save: {e}"),
                        };
                    }
                    if ui.button("Read it again").clicked() {
                        self.load_library();
                        self.lib_ui.status = "read the library folder again".into();
                    }
                    if let Some(d) = &self.library_dir {
                        ui.weak(d.display().to_string());
                    }
                });
                if !self.lib_ui.status.is_empty() {
                    ui.label(&self.lib_ui.status);
                }
            });
        self.library_open = open;
    }

    fn tools_tab(&mut self, ui: &mut egui::Ui, project: &mut Project, dialogs: &mut dyn Dialogs) {
        ui.horizontal(|ui| {
            ui.label("Filter");
            ui.text_edit_singleline(&mut self.lib_ui.filter);
        });
        let filter = self.lib_ui.filter.to_lowercase();
        let names: Vec<(usize, String)> = self
            .library
            .tools
            .iter()
            .enumerate()
            .filter(|(_, t)| filter.is_empty() || t.name.to_lowercase().contains(&filter))
            .map(|(i, t)| (i, t.name.clone()))
            .collect();
        egui::ScrollArea::vertical()
            .id_salt("lib_tools")
            .max_height(150.0)
            .show(ui, |ui| {
                for (i, n) in &names {
                    if ui
                        .selectable_label(self.lib_ui.tool == Some(*i), n)
                        .clicked()
                    {
                        self.lib_ui.tool = Some(*i);
                    }
                }
            });
        ui.horizontal(|ui| {
            if ui.button("New").clicked() {
                let mut d = ToolDef {
                    name: "New tool".into(),
                    tool: Tool::flat(1, 6.0, 20.0),
                    cutting: Vec::new(),
                };
                d.tool.name = d.name.clone();
                self.library.tools.push(d);
                self.lib_ui.tool = Some(self.library.tools.len() - 1);
            }
            if let Some(i) = self.lib_ui.tool.filter(|i| *i < self.library.tools.len()) {
                if ui.button("Duplicate").clicked() {
                    let mut d = self.library.tools[i].clone();
                    d.name = format!("{} copy", d.name);
                    d.tool.name = d.name.clone();
                    self.library.tools.push(d);
                    self.lib_ui.tool = Some(self.library.tools.len() - 1);
                }
                if ui.button("Delete").clicked() {
                    self.library.tools.remove(i);
                    self.lib_ui.tool = None;
                }
                if ui
                    .button("Use in the operation")
                    .on_hover_text("Cut the next operation with this tool")
                    .clicked()
                {
                    let t = self.library.tools[i].tool.clone();
                    self.settings.diameter = t.diameter;
                    self.settings.preset = Some(t);
                }
                if ui
                    .button("Add to the setup")
                    .on_hover_text("Put the tool into the current setup's tool list")
                    .clicked()
                {
                    self.current_setup(project);
                    let t = self.library.tools[i].tool.clone();
                    let n = tool_number(&mut project.cam, self.current, &t);
                    project.mark_extras_dirty();
                    self.lib_ui.status = format!("{} is T{n} in this project", t.name);
                }
            }
            if ui.button("Import…").clicked() {
                if let Some(p) = dialogs.open_file("Tool library", &["json"]) {
                    self.lib_ui.status = match std::fs::read_to_string(&p)
                        .map_err(|e| e.to_string())
                        .and_then(|t| self.library.import_tools(&t))
                    {
                        Ok(n) => format!("imported {n} tool(s)"),
                        Err(e) => format!("could not import: {e}"),
                    };
                }
            }
            if ui.button("Export…").clicked() {
                if let Some(p) = dialogs.save_file("tools.json", "Tool library", &["json"]) {
                    self.lib_ui.status = match std::fs::write(&p, self.library.export_tools()) {
                        Ok(()) => format!("wrote {}", p.display()),
                        Err(e) => format!("could not write: {e}"),
                    };
                }
            }
        });
        let Some(i) = self.lib_ui.tool.filter(|i| *i < self.library.tools.len()) else {
            return;
        };
        ui.separator();
        let holders: Vec<HolderDef> = self.library.holders.clone();
        let d = &mut self.library.tools[i];
        let mut changed = false;
        ui.horizontal(|ui| {
            ui.label("Name");
            changed |= ui.text_edit_singleline(&mut d.name).changed();
        });
        ui.horizontal(|ui| {
            ui.label("Kind");
            egui::ComboBox::from_id_salt("lib_kind")
                .selected_text(d.tool.kind.label())
                .show_ui(ui, |ui| {
                    for (label, k) in kind_choices(d.tool.diameter) {
                        if ui
                            .selectable_label(same_kind(&d.tool.kind, &k), label)
                            .clicked()
                            && !same_kind(&d.tool.kind, &k)
                        {
                            d.tool.kind = k;
                            changed = true;
                        }
                    }
                });
        });
        changed |= num(ui, "Diameter (mm)", &mut d.tool.diameter, 0.1);
        match &mut d.tool.kind {
            ToolKind::Bull { corner_radius } => {
                changed |= num(ui, "Corner radius (mm)", corner_radius, 0.05);
            }
            ToolKind::Drill { point_angle } => {
                changed |= num(ui, "Point angle (°)", point_angle, 1.0);
            }
            ToolKind::Chamfer {
                tip_angle,
                tip_diameter,
            }
            | ToolKind::Engraver {
                tip_angle,
                tip_diameter,
            } => {
                changed |= num(ui, "Included angle (°)", tip_angle, 1.0);
                changed |= num(ui, "Tip diameter (mm)", tip_diameter, 0.05);
            }
            ToolKind::Tap { pitch } => {
                changed |= num(ui, "Pitch (mm)", pitch, 0.05);
            }
            _ => {}
        }
        changed |= num(ui, "Flute length (mm)", &mut d.tool.flute_length, 0.5);
        changed |= num(ui, "Shank radius (mm)", &mut d.tool.shank_radius, 0.1);
        changed |= num(ui, "Shank length (mm)", &mut d.tool.shank_length, 0.5);
        ui.horizontal(|ui| {
            ui.label("Flutes");
            changed |= ui
                .add(egui::DragValue::new(&mut d.tool.flutes).range(1..=12))
                .changed();
            ui.label("Material");
            changed |= ui.text_edit_singleline(&mut d.tool.material).changed();
        });
        ui.horizontal(|ui| {
            ui.label("Coating");
            changed |= ui.text_edit_singleline(&mut d.tool.coating).changed();
        });
        ui.horizontal(|ui| {
            ui.label("Holder");
            egui::ComboBox::from_id_salt("lib_tool_holder")
                .selected_text(if d.tool.holder_name.is_empty() {
                    "(custom)"
                } else {
                    &d.tool.holder_name
                })
                .show_ui(ui, |ui| {
                    for h in &holders {
                        if ui
                            .selectable_label(d.tool.holder_name == h.name, &h.name)
                            .clicked()
                        {
                            d.tool = with_holder(&d.tool, h);
                            changed = true;
                        }
                    }
                });
        });
        ui.strong("Cutting data");
        let mut remove = None;
        for (k, c) in d.cutting.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt(format!("lib_cut_{k}"))
                    .selected_text(&c.group)
                    .show_ui(ui, |ui| {
                        for g in kw_materials::MaterialGroup::ALL {
                            changed |= ui
                                .selectable_value(&mut c.group, g.name().to_string(), g.name())
                                .changed();
                        }
                    });
                changed |= ui
                    .add(egui::DragValue::new(&mut c.vc).speed(5.0).suffix(" m/min"))
                    .changed();
                changed |= ui
                    .add(
                        egui::DragValue::new(&mut c.fz)
                            .speed(0.002)
                            .suffix(" mm/tooth"),
                    )
                    .changed();
                if ui.small_button("×").clicked() {
                    remove = Some(k);
                }
            });
        }
        if let Some(k) = remove {
            d.cutting.remove(k);
        }
        if ui.small_button("Add cutting data").clicked() {
            d.cutting.push(CutData {
                group: kw_materials::MaterialGroup::Steel.name().into(),
                vc: 100.0,
                fz: 0.05,
            });
        }
        ui.weak("Without cutting data for a material, the surface speed and chip load of the material are used.");
        if changed {
            // the tool is called what the library calls it
            d.tool.name = d.name.clone();
        }
    }

    fn holders_tab(&mut self, ui: &mut egui::Ui, dialogs: &mut dyn Dialogs) {
        let names: Vec<String> = self
            .library
            .holders
            .iter()
            .map(|h| h.name.clone())
            .collect();
        egui::ScrollArea::vertical()
            .id_salt("lib_holders")
            .max_height(120.0)
            .show(ui, |ui| {
                for (i, n) in names.iter().enumerate() {
                    if ui
                        .selectable_label(self.lib_ui.holder == Some(i), n)
                        .clicked()
                    {
                        self.lib_ui.holder = Some(i);
                    }
                }
            });
        ui.horizontal(|ui| {
            if ui.button("New").clicked() {
                self.library.holders.push(HolderDef {
                    name: "New holder".into(),
                    sections: vec![(15.0, 40.0)],
                });
                self.lib_ui.holder = Some(self.library.holders.len() - 1);
            }
            if let Some(i) = self
                .lib_ui
                .holder
                .filter(|i| *i < self.library.holders.len())
            {
                if ui.button("Delete").clicked() {
                    self.library.holders.remove(i);
                    self.lib_ui.holder = None;
                }
                if ui.button("Use in the operation").clicked() {
                    self.settings.holder = Some(self.library.holders[i].clone());
                }
            }
            if ui.button("Import…").clicked() {
                if let Some(p) = dialogs.open_file("Holder library", &["json"]) {
                    self.lib_ui.status = match std::fs::read_to_string(&p)
                        .map_err(|e| e.to_string())
                        .and_then(|t| self.library.import_holders(&t))
                    {
                        Ok(n) => format!("imported {n} holder(s)"),
                        Err(e) => format!("could not import: {e}"),
                    };
                }
            }
            if ui.button("Export…").clicked() {
                if let Some(p) = dialogs.save_file("holders.json", "Holder library", &["json"]) {
                    self.lib_ui.status = match std::fs::write(&p, self.library.export_holders()) {
                        Ok(()) => format!("wrote {}", p.display()),
                        Err(e) => format!("could not write: {e}"),
                    };
                }
            }
        });
        let Some(i) = self
            .lib_ui
            .holder
            .filter(|i| *i < self.library.holders.len())
        else {
            return;
        };
        let h = &mut self.library.holders[i];
        ui.horizontal(|ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut h.name);
        });
        ui.label("Sections above the tool (radius, length):");
        let mut remove = None;
        for (k, (r, l)) in h.sections.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.add(
                    egui::DragValue::new(r)
                        .speed(0.25)
                        .prefix("r ")
                        .range(0.5..=200.0),
                );
                ui.add(
                    egui::DragValue::new(l)
                        .speed(0.5)
                        .prefix("length ")
                        .range(0.5..=500.0),
                );
                if ui.small_button("×").clicked() {
                    remove = Some(k);
                }
            });
        }
        if let Some(k) = remove {
            h.sections.remove(k);
        }
        if ui.small_button("Add section").clicked() {
            h.sections.push((20.0, 30.0));
        }
    }

    fn machines_tab(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) {
        let names: Vec<String> = self
            .library
            .machines
            .iter()
            .map(|m| m.name.clone())
            .collect();
        egui::ScrollArea::vertical()
            .id_salt("lib_machines")
            .max_height(120.0)
            .show(ui, |ui| {
                for (i, n) in names.iter().enumerate() {
                    let axes = self.library.machines[i].axes();
                    if ui
                        .selectable_label(
                            self.lib_ui.machine == Some(i),
                            format!("{n}  ({axes} axes)"),
                        )
                        .clicked()
                    {
                        self.lib_ui.machine = Some(i);
                    }
                }
            });
        let posts: Vec<String> = self.library.posts.iter().map(|p| p.id.clone()).collect();
        ui.horizontal(|ui| {
            if ui.button("New").clicked() {
                let mut m = self.library.machines[0].clone();
                m.name = "New machine".into();
                m.kinematics = None;
                self.library.machines.push(m);
                self.lib_ui.machine = Some(self.library.machines.len() - 1);
            }
            if let Some(i) = self
                .lib_ui
                .machine
                .filter(|i| *i < self.library.machines.len())
            {
                if ui.button("Delete").clicked() && self.library.machines.len() > 1 {
                    self.library.machines.remove(i);
                    self.lib_ui.machine = None;
                }
                if ui.button("Use in the setup").clicked() {
                    let n = self.library.machines[i].name.clone();
                    self.lib_ui.status = match self.set_machine(project, &n) {
                        Ok(()) => format!("{n} and its post are used by this setup"),
                        Err(e) => e,
                    };
                }
                if ui.button("Export…").clicked() {
                    let m = &self.library.machines[i];
                    if let Some(p) = dialogs.save_file(
                        &format!("{}.json", m.name.replace(' ', "_")),
                        "Machine",
                        &["json"],
                    ) {
                        self.lib_ui.status = match std::fs::write(
                            &p,
                            serde_json::to_string_pretty(m).expect("machine serialises"),
                        ) {
                            Ok(()) => format!("wrote {}", p.display()),
                            Err(e) => format!("could not write: {e}"),
                        };
                    }
                }
            }
            if ui.button("Import…").clicked() {
                if let Some(p) = dialogs.open_file("Machine", &["json"]) {
                    self.lib_ui.status = match std::fs::read_to_string(&p)
                        .map_err(|e| e.to_string())
                        .and_then(|t| self.library.import_machine(&t))
                    {
                        Ok(n) => format!("imported {n}"),
                        Err(e) => format!("could not import: {e}"),
                    };
                }
            }
        });
        let Some(i) = self
            .lib_ui
            .machine
            .filter(|i| *i < self.library.machines.len())
        else {
            return;
        };
        let m = &mut self.library.machines[i];
        ui.horizontal(|ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut m.name);
        });
        ui.horizontal(|ui| {
            ui.label("Post");
            egui::ComboBox::from_id_salt("lib_machine_post")
                .selected_text(&m.post)
                .show_ui(ui, |ui| {
                    for p in &posts {
                        ui.selectable_value(&mut m.post, p.clone(), p);
                    }
                });
        });
        num(ui, "Spindle minimum (rpm)", &mut m.spindle_min, 50.0);
        num(ui, "Spindle maximum (rpm)", &mut m.spindle_max, 100.0);
        num(ui, "Rapid traverse (mm/min)", &mut m.rapid, 100.0);
        num(ui, "Highest cutting feed (mm/min)", &mut m.max_feed, 100.0);
        ui.horizontal(|ui| {
            ui.label("Tool changer holds");
            ui.add(egui::DragValue::new(&mut m.tool_capacity).range(1..=500));
        });
        for (k, name) in ["X", "Y", "Z"].iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("{name} travel"));
                ui.add(egui::DragValue::new(&mut m.travel[k][0]).speed(5.0));
                ui.label("to");
                ui.add(egui::DragValue::new(&mut m.travel[k][1]).speed(5.0));
            });
        }
        match &m.kinematics {
            Some(k) => {
                ui.label(format!("Axes: {}", k.axis_names.join(" ")));
            }
            None => {
                ui.label("Three axes (X Y Z).");
            }
        }
    }

    fn posts_tab(&mut self, ui: &mut egui::Ui, project: &mut Project, dialogs: &mut dyn Dialogs) {
        let ids: Vec<(String, bool)> = self
            .library
            .posts
            .iter()
            .map(|p| (p.id.clone(), Library::is_builtin_post(&p.id)))
            .collect();
        egui::ScrollArea::vertical()
            .id_salt("lib_posts")
            .max_height(100.0)
            .show(ui, |ui| {
                for (i, (id, builtin)) in ids.iter().enumerate() {
                    let label = if *builtin {
                        format!("{id}  (built in)")
                    } else {
                        id.clone()
                    };
                    if ui
                        .selectable_label(self.lib_ui.post == Some(i), label)
                        .clicked()
                    {
                        self.lib_ui.post = Some(i);
                    }
                }
            });
        // the editor shows the selected post until another is chosen
        if self.lib_ui.post_for != self.lib_ui.post {
            self.lib_ui.post_for = self.lib_ui.post;
            if let Some(p) = self.lib_ui.post.and_then(|i| self.library.posts.get(i)) {
                self.lib_ui.post_text = p.text.clone();
            }
        }
        ui.horizontal(|ui| {
            if ui.button("New from the selected").clicked() {
                if let Some(p) = self.lib_ui.post.and_then(|i| self.library.posts.get(i)) {
                    self.lib_ui.post_text = p.text.replacen("name =", "name = My ", 1);
                    self.lib_ui.post_for = None;
                    self.lib_ui.post = None;
                }
            }
            if ui.button("Use in the setup").clicked() {
                if let Some(id) = self
                    .lib_ui
                    .post
                    .and_then(|i| ids.get(i))
                    .map(|p| p.0.clone())
                {
                    self.lib_ui.status = match self.set_post(project, &id) {
                        Ok(()) => format!("{id} writes this setup's programs"),
                        Err(e) => e,
                    };
                }
            }
            if ui.button("Import…").clicked() {
                if let Some(p) = dialogs.open_file("Post-processor", &["post"]) {
                    self.lib_ui.status = match std::fs::read_to_string(&p)
                        .map_err(|e| e.to_string())
                        .and_then(|t| self.library.import_post(&t))
                    {
                        Ok(id) => format!("imported {id}"),
                        Err(e) => format!("could not import: {e}"),
                    };
                }
            }
            if ui.button("Export…").clicked() {
                if let Some(p) = self.lib_ui.post.and_then(|i| self.library.posts.get(i)) {
                    if let Some(f) = dialogs.save_file(
                        &format!("{}.post", p.id.replace(' ', "_")),
                        "Post-processor",
                        &["post"],
                    ) {
                        self.lib_ui.status = match std::fs::write(&f, &p.text) {
                            Ok(()) => format!("wrote {}", f.display()),
                            Err(e) => format!("could not write: {e}"),
                        };
                    }
                }
            }
        });
        egui::ScrollArea::vertical()
            .id_salt("lib_post_text")
            .max_height(220.0)
            .show(ui, |ui| {
                ui.add(
                    egui::TextEdit::multiline(&mut self.lib_ui.post_text)
                        .font(egui::TextStyle::Monospace)
                        .desired_width(f32::INFINITY)
                        .desired_rows(12),
                );
            });
        // the post is checked as it is typed: the error names its line
        match PostConfig::parse(&self.lib_ui.post_text) {
            Ok(c) => {
                ui.horizontal(|ui| {
                    ui.colored_label(
                        egui::Color32::from_rgb(90, 180, 110),
                        format!("valid: {}", c.name),
                    );
                    let builtin = Library::is_builtin_post(&c.name);
                    if ui
                        .add_enabled(!builtin, egui::Button::new("Save to the library"))
                        .on_disabled_hover_text("Give the post a name of its own to keep it")
                        .clicked()
                    {
                        let text = self.lib_ui.post_text.clone();
                        self.lib_ui.status = match self.library.import_post(&text) {
                            Ok(id) => {
                                self.lib_ui.post =
                                    self.library.posts.iter().position(|p| p.id == id);
                                format!("{id} is in the library")
                            }
                            Err(e) => e,
                        };
                    }
                });
            }
            Err(e) => {
                ui.colored_label(egui::Color32::from_rgb(230, 110, 90), e.to_string());
            }
        }
    }
}
