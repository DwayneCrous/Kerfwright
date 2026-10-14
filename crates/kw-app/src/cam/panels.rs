//! The side panel of the CAM workspace: setups, operations, simulation and posting.

use super::*;

impl CamWorkspace {
    pub(super) fn side(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) {
        let busy = self.gen_job.is_some() || self.sim_job.is_some();
        self.setup_panel(ui, project);
        ui.separator();
        self.operation_panel(ui, project, busy);
        ui.separator();
        self.operations_list(ui, project, busy);
        self.simulation_panel(ui, project);
        self.post_panel(ui, project, dialogs);
        if !self.message.is_empty() {
            ui.separator();
            ui.label(&self.message);
        }
    }

    pub(super) fn setup_panel(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.heading("Setups");
        let names: Vec<String> = project.cam.iter().map(|s| s.name.clone()).collect();
        let mut pick = None;
        let mut delete = None;
        for (i, n) in names.iter().enumerate() {
            ui.horizontal(|ui| {
                if ui.selectable_label(self.current == i, n).clicked() {
                    pick = Some(i);
                }
                if names.len() > 1 && ui.small_button("Delete").clicked() {
                    delete = Some(i);
                }
            });
        }
        if let Some(i) = pick {
            if i != self.current {
                self.current = i;
                self.selected = None;
                self.geo = None;
                self.shown += 1;
            }
        }
        if let Some(i) = delete {
            self.delete_setup(project, i);
        }
        if ui.button("New setup").clicked() {
            self.new_setup(project);
        }
        let has_assembly = project.assembly.is_some();
        let source = self.source(project).ok();
        let mut changed = false;
        let mut orientation = None;
        let mut place = false;
        let current = self.current;
        let n_setups = project.cam.len();
        let s = self.current_setup(project);
        ui.horizontal(|ui| {
            ui.label("Name");
            changed |= ui.text_edit_singleline(&mut s.name).changed();
        });
        if has_assembly {
            ui.horizontal(|ui| {
                ui.label("Bodies from");
                changed |= ui
                    .selectable_value(&mut s.source, SourceKind::Design, "Design")
                    .changed();
                changed |= ui
                    .selectable_value(&mut s.source, SourceKind::Assembly, "Assembly")
                    .changed();
            });
        }
        egui::CollapsingHeader::new("Work coordinates")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label("Tool approaches from");
                    for (i, (label, ..)) in ORIENTATIONS.iter().enumerate() {
                        if ui.small_button(*label).clicked() {
                            orientation = Some(i);
                        }
                    }
                });
                let w = &mut s.wcs;
                ui.horizontal(|ui| {
                    for (k, p) in ["x", "y", "z"].iter().zip(w.origin.iter_mut()) {
                        changed |= ui
                            .add(egui::DragValue::new(p).speed(0.5).prefix(format!("{k} ")))
                            .changed();
                    }
                });
                if ui
                    .small_button("Origin at the top corner of the workpiece")
                    .clicked()
                {
                    place = true;
                }
                ui.horizontal(|ui| {
                    ui.label("Work offset");
                    let mut o = w.offset.clamp(1, 6);
                    egui::ComboBox::from_id_salt("cam_offset")
                        .selected_text(format!("G{}", 53 + o as u32))
                        .show_ui(ui, |ui| {
                            for k in 1..=6u8 {
                                ui.selectable_value(&mut o, k, format!("G{}", 53 + k as u32));
                            }
                        });
                    if o != w.offset {
                        w.offset = o;
                        changed = true;
                    }
                });
                ui.weak(format!(
                    "z → ({:.2} {:.2} {:.2})",
                    w.z_axis[0], w.z_axis[1], w.z_axis[2]
                ));
            });
        egui::CollapsingHeader::new("Stock")
            .default_open(true)
            .show(ui, |ui| {
                let kind = match s.stock {
                    StockSpec::Box { .. } => 0,
                    StockSpec::Cylinder { .. } => 1,
                    StockSpec::Explicit { .. } => 2,
                    StockSpec::Previous => 3,
                };
                let mut new_kind = kind;
                egui::ComboBox::from_id_salt("cam_stock")
                    .selected_text(
                        [
                            "Block",
                            "Round bar",
                            "Explicit block",
                            "From the previous setup",
                        ][kind],
                    )
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut new_kind, 0, "Block");
                        ui.selectable_value(&mut new_kind, 1, "Round bar");
                        ui.selectable_value(&mut new_kind, 2, "Explicit block");
                        if current > 0 {
                            ui.selectable_value(&mut new_kind, 3, "From the previous setup");
                        }
                    });
                if new_kind != kind {
                    changed = true;
                    s.stock = match new_kind {
                        0 => StockSpec::Box {
                            side: 0.0,
                            top: 0.0,
                            bottom: 0.0,
                        },
                        1 => StockSpec::Cylinder {
                            axis: 2,
                            margin: 0.0,
                            ends: 0.0,
                        },
                        2 => {
                            let b = source
                                .as_ref()
                                .and_then(|src| {
                                    Self::workpiece_bounds(
                                        src,
                                        &CamSetup {
                                            wcs: s.wcs.clone(),
                                            bodies: s.bodies.clone(),
                                            ..CamSetup::default()
                                        },
                                    )
                                })
                                .unwrap_or(([0.0; 3], [100.0, 100.0, 20.0]));
                            StockSpec::Explicit { min: b.0, max: b.1 }
                        }
                        _ => StockSpec::Previous,
                    };
                }
                let drag = |ui: &mut egui::Ui, label: &str, v: &mut f64| -> bool {
                    ui.horizontal(|ui| {
                        ui.label(label);
                        ui.add(egui::DragValue::new(v).speed(0.25).range(-1e5..=1e5))
                            .changed()
                    })
                    .inner
                };
                match &mut s.stock {
                    StockSpec::Box { side, top, bottom } => {
                        changed |= drag(ui, "Extra on the sides (mm)", side);
                        changed |= drag(ui, "Extra on top (mm)", top);
                        changed |= drag(ui, "Extra below (mm)", bottom);
                    }
                    StockSpec::Cylinder { axis, margin, ends } => {
                        ui.horizontal(|ui| {
                            ui.label("Axis");
                            for (k, l) in ["X", "Y", "Z"].iter().enumerate() {
                                changed |= ui.selectable_value(axis, k as u8, *l).changed();
                            }
                        });
                        changed |= drag(ui, "Extra on the radius (mm)", margin);
                        changed |= drag(ui, "Extra at each end (mm)", ends);
                    }
                    StockSpec::Explicit { min, max } => {
                        for k in 0..3 {
                            ui.horizontal(|ui| {
                                ui.label(["x", "y", "z"][k]);
                                changed |= ui
                                    .add(egui::DragValue::new(&mut min[k]).speed(0.5))
                                    .changed();
                                ui.label("to");
                                changed |= ui
                                    .add(egui::DragValue::new(&mut max[k]).speed(0.5))
                                    .changed();
                            });
                        }
                    }
                    StockSpec::Previous => {
                        ui.weak("The stock the previous setup's operations left (simulated).");
                    }
                }
            });
        if let Some(src) = &source {
            egui::CollapsingHeader::new("Bodies")
                .default_open(src.bodies.len() > 1)
                .show(ui, |ui| {
                    for b in &src.bodies {
                        let mut role = s.role_of(&b.key);
                        let before = role;
                        ui.horizontal(|ui| {
                            ui.label(&b.name);
                            egui::ComboBox::from_id_salt(format!("cam_role_{}", b.name))
                                .selected_text(match role {
                                    BodyRole::Workpiece => "Workpiece",
                                    BodyRole::Fixture => "Fixture",
                                    BodyRole::Ignore => "Ignore",
                                })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut role,
                                        BodyRole::Workpiece,
                                        "Workpiece",
                                    );
                                    ui.selectable_value(&mut role, BodyRole::Fixture, "Fixture");
                                    ui.selectable_value(&mut role, BodyRole::Ignore, "Ignore");
                                });
                        });
                        if role != before {
                            s.set_role(&b.key, role);
                            changed = true;
                        }
                    }
                    ui.weak("A fixture is not machined, but the tools are checked against it.");
                });
        }
        let _ = n_setups;
        if changed {
            project.mark_extras_dirty();
            self.sim = None;
            self.geo = None;
            self.scene_bodies = None;
            self.shown += 1;
        }
        if let Some(i) = orientation {
            self.set_orientation(project, i);
            self.fit_to_workpiece(project);
        }
        if place {
            self.place_origin(project);
        }
    }

    /// The tool of the operation: a plain end mill of a diameter, or a tool from the library or the setup,
    /// with a holder, and the speed and feed the material asks for.
    pub(super) fn tool_picker(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.horizontal(|ui| {
            ui.label("Tool");
            for (c, label) in [
                (ToolChoice::Flat, "Flat"),
                (ToolChoice::Ball, "Ball"),
                (ToolChoice::Bull, "Bull"),
            ] {
                let on = self.settings.preset.is_none() && self.settings.tool == c;
                if ui.selectable_label(on, label).clicked() {
                    self.settings.tool = c;
                    self.settings.use_quick_tool();
                }
            }
        });
        match self.settings.preset.clone() {
            None => {
                let s = &mut self.settings;
                ui.horizontal(|ui| {
                    ui.label("Diameter (mm)");
                    ui.add(egui::DragValue::new(&mut s.diameter).speed(0.1));
                });
                if s.tool == ToolChoice::Bull {
                    ui.horizontal(|ui| {
                        ui.label("Corner radius");
                        ui.add(egui::DragValue::new(&mut s.corner).speed(0.05));
                    });
                }
            }
            Some(t) => {
                ui.weak(format!(
                    "{} · {:.1} mm · {} flute(s)",
                    t.name, t.diameter, t.flutes
                ));
            }
        }
        let setup_tools: Vec<Tool> = self.current_setup(project).tools.clone();
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("cam_tool_pick")
                .selected_text("Library or setup tool…")
                .width(200.0)
                .show_ui(ui, |ui| {
                    if !setup_tools.is_empty() {
                        ui.weak("In this project");
                        for t in &setup_tools {
                            if ui
                                .selectable_label(false, format!("T{} {}", t.number, t.name))
                                .clicked()
                            {
                                self.settings.diameter = t.diameter;
                                self.settings.preset = Some(t.clone());
                            }
                        }
                        ui.separator();
                    }
                    ui.weak("Library");
                    for d in self.library.tools.iter().take(300) {
                        if ui.selectable_label(false, &d.name).clicked() {
                            self.settings.diameter = d.tool.diameter;
                            self.settings.preset = Some(d.tool.clone());
                        }
                    }
                });
            let holder = self
                .settings
                .holder
                .as_ref()
                .map_or("Holder: tool's own".to_string(), |h| h.name.clone());
            egui::ComboBox::from_id_salt("cam_holder_pick")
                .selected_text(holder)
                .width(150.0)
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(self.settings.holder.is_none(), "Tool's own")
                        .clicked()
                    {
                        self.settings.holder = None;
                    }
                    for h in &self.library.holders {
                        if ui.selectable_label(false, &h.name).clicked() {
                            self.settings.holder = Some(h.clone());
                        }
                    }
                });
        });
        if ui
            .small_button("Feed and speed from the material")
            .on_hover_text("Surface speed and chip load of the workpiece material, held to the machine's limits")
            .clicked()
        {
            self.message = match self.feeds_from_material(project) {
                Ok(m) | Err(m) => m,
            };
        }
    }

    pub(super) fn operation_panel(&mut self, ui: &mut egui::Ui, project: &mut Project, busy: bool) {
        ui.heading("Operation");
        egui::ComboBox::from_id_salt("cam_op")
            .selected_text(self.settings.op.label())
            .show_ui(ui, |ui| {
                for k in OpKind::ALL {
                    ui.selectable_value(&mut self.settings.op, k, k.label());
                }
            });
        if self.settings.op == OpKind::Finish3d && self.settings.preset.is_none() {
            self.settings.tool = ToolChoice::Ball;
        }
        self.tool_picker(ui, project);
        let kind = self.settings.op;
        self.operation_changed(project);
        let drag = |ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64| {
            ui.horizontal(|ui| {
                ui.label(label);
                ui.add(egui::DragValue::new(v).speed(speed));
            });
        };
        if kind.uses_points() {
            self.holes_section(ui, project);
        } else {
            ui.separator();
            ui.strong("Region (XY of the work system, mm)");
            {
                let s = &mut self.settings;
                let before = (s.x0, s.y0, s.x1, s.y1);
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut s.x0).speed(0.5).prefix("x0 "));
                    ui.add(egui::DragValue::new(&mut s.x1).speed(0.5).prefix("x1 "));
                });
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut s.y0).speed(0.5).prefix("y0 "));
                    ui.add(egui::DragValue::new(&mut s.y1).speed(0.5).prefix("y1 "));
                });
                if before != (s.x0, s.y0, s.x1, s.y1) {
                    s.region = None;
                }
            }
            let mut fit = false;
            let mut face = None;
            ui.horizontal(|ui| {
                if ui.button("Use the workpiece's footprint").clicked() {
                    fit = true;
                }
            });
            if let RegionSpec::Polygon { outer, islands } = &self.settings.region_spec() {
                ui.weak(format!(
                    "From a face outline: {} corners, {} island(s).",
                    outer.len(),
                    islands.len()
                ));
            }
            let faces = self.geometry(project).ok();
            if let Some(g) = &faces {
                egui::ComboBox::from_id_salt("cam_face")
                    .selected_text("Region from a flat face…")
                    .width(240.0)
                    .show_ui(ui, |ui| {
                        for (i, f) in g.faces.iter().enumerate().take(60) {
                            let label = format!(
                                "z {:.2}  {:.0} mm²  {}  {}",
                                f.z,
                                f.area,
                                if f.up { "up" } else { "down" },
                                f.body
                            );
                            if ui.selectable_label(false, label).clicked() {
                                face = Some(i);
                            }
                        }
                    });
            }
            if fit {
                self.fit_to_workpiece(project);
            }
            if let (Some(i), Some(g)) = (face, &faces) {
                let f = &g.faces[i];
                let spec = RegionSpec::Polygon {
                    outer: f.outer.clone(),
                    islands: f.islands.clone(),
                };
                let b = spec.bounds();
                let s = &mut self.settings;
                (s.x0, s.y0, s.x1, s.y1) = (b[0], b[1], b[2], b[3]);
                s.region = Some(spec);
                if f.up && matches!(s.op, OpKind::Pocket | OpKind::Adaptive | OpKind::Face) {
                    s.bottom = f.z;
                    if s.top <= s.bottom {
                        s.top = s.bottom + 3.0;
                    }
                }
            }
        }
        let s = &mut self.settings;
        match kind {
            OpKind::Chamfer => {
                drag(ui, "Chamfer width (mm)", &mut s.width, 0.05);
                ui.checkbox(&mut s.inside, "Tool inside the outline (edges of a hole)");
            }
            OpKind::Swarf => {
                drag(
                    ui,
                    "Wall draft (°, + leans outward going up)",
                    &mut s.taper,
                    0.5,
                );
                ui.checkbox(&mut s.inside, "Tool inside the outline");
            }
            OpKind::TiltedContour => drag(ui, "Lean against the wall (°)", &mut s.tilt, 0.5),
            OpKind::Waterline => drag(ui, "Cutter-location grid (mm)", &mut s.grid, 0.05),
            OpKind::RestFinish => drag(
                ui,
                "Finish where more than (mm) is left",
                &mut s.threshold,
                0.05,
            ),
            OpKind::Drill => drag(ui, "Peck depth (0: plain cycle)", &mut s.peck, 0.5),
            _ => {}
        }
        drag(ui, "Top z", &mut s.top, 0.5);
        drag(
            ui,
            if kind == OpKind::Drill {
                "Depth of the drill tip z"
            } else {
                "Bottom z"
            },
            &mut s.bottom,
            0.5,
        );
        if !matches!(kind, OpKind::Drill | OpKind::Chamfer | OpKind::Swarf) {
            drag(ui, "Step down", &mut s.step_down, 0.1);
        }
        if !matches!(
            kind,
            OpKind::Drill | OpKind::Bore | OpKind::Chamfer | OpKind::Swarf | OpKind::Waterline
        ) {
            drag(ui, "Stepover", &mut s.stepover, 0.1);
        }
        drag(ui, "Feed (mm/min)", &mut s.feed, 10.0);
        drag(ui, "Spindle (rpm)", &mut s.rpm, 100.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!busy, egui::Button::new("Add operation"))
                .on_hover_text("Generate a toolpath from these settings and add it to the setup")
                .clicked()
            {
                self.start(project);
            }
            let can_update = self.selected.is_some_and(|i| {
                project
                    .cam
                    .get(self.current)
                    .and_then(|c| c.operations.get(i))
                    .is_some_and(|o| o.params.is_some())
            });
            if ui
                .add_enabled(!busy && can_update, egui::Button::new("Update selected"))
                .on_hover_text("Remake the selected operation with these settings")
                .clicked()
            {
                self.start_update(project);
            }
        });
        if let Some(job) = &self.gen_job {
            if job.ui(ui) {
                self.message = "cancelled".into();
            }
        }
    }

    /// Settings that follow the kind of operation: the tool a drill or chamfer needs, and the holes.
    fn operation_changed(&mut self, project: &mut Project) {
        let kind = self.settings.op;
        if self.panel_kind != Some(kind) {
            self.panel_kind = Some(kind);
            // a tool of the right kind when the current one cannot do the operation
            let t = self.settings.tool();
            let fit = match kind {
                OpKind::Drill => matches!(t.kind, ToolKind::Drill { .. }),
                OpKind::Chamfer => {
                    matches!(t.kind, ToolKind::Chamfer { .. } | ToolKind::Engraver { .. })
                }
                OpKind::Finish3d | OpKind::RestFinish => t.kind == ToolKind::Ball,
                _ => t.kind.mills() || kind == OpKind::Bore,
            };
            if !fit {
                let pick = match kind {
                    OpKind::Drill => self
                        .library
                        .tools
                        .iter()
                        .find(|d| matches!(d.tool.kind, ToolKind::Drill { .. })),
                    OpKind::Chamfer => self
                        .library
                        .tools
                        .iter()
                        .find(|d| matches!(d.tool.kind, ToolKind::Chamfer { .. })),
                    _ => None,
                };
                if let Some(d) = pick {
                    self.settings.diameter = d.tool.diameter;
                    self.settings.preset = Some(d.tool.clone());
                } else {
                    self.settings.use_quick_tool();
                }
            }
            if kind.uses_points() && self.settings.points.is_empty() {
                self.use_holes(project, 0);
            }
        }
    }

    /// The holes the model has, by diameter.
    pub fn hole_groups(
        &mut self,
        project: &mut Project,
    ) -> Vec<(f64, Vec<crate::cam::setup::Hole>)> {
        let Ok(g) = self.geometry(project) else {
            return Vec::new();
        };
        let mut groups: Vec<(f64, Vec<crate::cam::setup::Hole>)> = Vec::new();
        for h in &g.holes {
            match groups
                .iter_mut()
                .find(|(d, _)| (d - h.diameter).abs() < 0.02)
            {
                Some((_, v)) => v.push(h.clone()),
                None => groups.push((h.diameter, vec![h.clone()])),
            }
        }
        groups
    }

    /// Take the holes of the `group`-th diameter as the operation's points, with their depth.
    pub fn use_holes(&mut self, project: &mut Project, group: usize) {
        let groups = self.hole_groups(project);
        let Some((d, holes)) = groups.get(group) else {
            return;
        };
        let s = &mut self.settings;
        s.points = holes.iter().map(|h| h.centre).collect();
        s.hole_diameter = *d;
        let top = holes.iter().map(|h| h.top).fold(f64::MIN, f64::max);
        let bottom = holes.iter().map(|h| h.bottom).fold(f64::MAX, f64::min);
        s.top = top;
        let through = holes.iter().all(|h| h.through);
        s.bottom = if s.op == OpKind::Drill && through {
            // the tip goes through, so the full diameter does
            bottom - 0.3 * d
        } else {
            bottom
        };
        if s.op == OpKind::Bore {
            s.step_down = s.step_down.min((top - bottom).max(0.5));
        }
    }

    fn holes_section(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.separator();
        ui.strong("Holes");
        let groups = self.hole_groups(project);
        if groups.is_empty() {
            ui.weak("The model has no round holes along the tool axis.");
        }
        let mut pick = None;
        ui.horizontal_wrapped(|ui| {
            for (i, (d, v)) in groups.iter().enumerate() {
                if ui
                    .small_button(format!("{} × ⌀{:.2}", v.len(), d))
                    .clicked()
                {
                    pick = Some(i);
                }
            }
        });
        if let Some(i) = pick {
            self.use_holes(project, i);
        }
        let s = &mut self.settings;
        ui.label(format!("{} point(s) chosen", s.points.len()));
        let mut remove = None;
        for (i, p) in s.points.iter_mut().enumerate().take(12) {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut p[0]).speed(0.25).prefix("x "));
                ui.add(egui::DragValue::new(&mut p[1]).speed(0.25).prefix("y "));
                if ui.small_button("×").clicked() {
                    remove = Some(i);
                }
            });
        }
        if s.points.len() > 12 {
            ui.weak(format!("… and {} more", s.points.len() - 12));
        }
        if let Some(i) = remove {
            s.points.remove(i);
        }
        if ui.small_button("Add a point").clicked() {
            let last = s.points.last().copied().unwrap_or([0.0, 0.0]);
            s.points.push([last[0] + 5.0, last[1]]);
        }
        if s.op == OpKind::Bore {
            ui.horizontal(|ui| {
                ui.label("Hole diameter (mm)");
                ui.add(egui::DragValue::new(&mut s.hole_diameter).speed(0.1));
            });
        }
    }

    pub(super) fn operations_list(&mut self, ui: &mut egui::Ui, project: &mut Project, busy: bool) {
        ui.heading("Operations");
        let stale = self.stale_ops(project);
        let setup = self.current_setup(project).clone();
        if setup.operations.is_empty() {
            ui.weak("None yet.");
        }
        let mut remove = None;
        let mut select = None;
        for (i, o) in setup.operations.iter().enumerate() {
            ui.horizontal(|ui| {
                let mut label = o.name.clone();
                if o.params.is_none() {
                    label.push_str("  (no settings)");
                }
                if ui.selectable_label(self.selected == Some(i), label).clicked() {
                    select = Some(i);
                }
                if stale.contains(&i) {
                    ui.colored_label(egui::Color32::from_rgb(230, 150, 60), "out of date")
                        .on_hover_text(
                            "The model, the tool, the stock or the work coordinates changed since this toolpath was made",
                        );
                }
                if ui.small_button("Delete").clicked() {
                    remove = Some(i);
                }
            });
        }
        if let Some(i) = select {
            self.selected = Some(i);
            let o = &setup.operations[i];
            if let Some(p) = &o.params {
                self.settings.load(p, tool_of(&setup, o.tool));
            }
            self.shown += 1;
        }
        if let Some(i) = remove {
            if let Some(c) = project.cam.get_mut(self.current) {
                c.operations.remove(i);
            }
            project.mark_extras_dirty();
            self.selected = None;
            self.sim = None;
            self.shown += 1;
        }
        if !stale.is_empty()
            && ui
                .add_enabled(
                    !busy,
                    egui::Button::new(format!("Regenerate {} out of date", stale.len())),
                )
                .clicked()
        {
            self.start_regenerate(project);
        }
    }

    pub(super) fn simulation_panel(&mut self, ui: &mut egui::Ui, project: &mut Project) {
        ui.separator();
        ui.heading("Simulation");
        let busy = self.sim_job.is_some();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!busy, egui::Button::new("Simulate setup"))
                .on_hover_text("Cut the stock with every operation and compare it with the part")
                .clicked()
            {
                self.start_sim(project);
            }
            ui.checkbox(&mut self.view_stock, "Show stock");
        });
        if let Some(job) = &self.sim_job {
            if job.ui(ui) {
                self.message = "cancelled".into();
            }
        }
        let current = self.current;
        let Some((i, _, r)) = self.sim.clone() else {
            return;
        };
        if i != current {
            ui.weak("The simulation shown is of another setup.");
            return;
        }
        ui.label(format!(
            "stock {:.0} → {:.0} mm³ (cell {:.2} mm)",
            r.stock_volume, r.final_volume, r.cell
        ));
        ui.label(format!(
            "left outside the part: {:.0} mm³;  cut into the part: {:.0} mm³",
            r.remaining, r.overcut
        ));
        ui.weak("green: on the part · orange to red: material left · blue: cut too deep");
        for (k, o) in r.ops.iter().enumerate() {
            ui.horizontal_wrapped(|ui| {
                if ui
                    .small_button(&o.name)
                    .on_hover_text("Show this operation")
                    .clicked()
                {
                    self.selected = Some(k);
                    self.shown += 1;
                }
                if let Some(s) = &o.skipped {
                    ui.colored_label(egui::Color32::from_rgb(230, 110, 90), s);
                    return;
                }
                ui.label(format!("{:.0} mm³, {:.0} s", o.removed, o.seconds));
                let bad = egui::Color32::from_rgb(230, 110, 90);
                if o.rapid_cut > 1e-3 {
                    ui.colored_label(bad, format!("rapid cuts {:.1} mm³", o.rapid_cut));
                }
                if o.fixture_gouges > 0 {
                    ui.colored_label(bad, format!("tip in fixture ×{}", o.fixture_gouges));
                }
                if o.shank_collisions > 0 {
                    ui.colored_label(bad, format!("shank hits fixture ×{}", o.shank_collisions));
                }
                if o.holder_collisions > 0 {
                    ui.colored_label(bad, format!("holder hits fixture ×{}", o.holder_collisions));
                }
                if o.fixture_unchecked {
                    ui.weak("fixtures not checked (tilted tool)");
                }
                if let Some(st) = o.required_stickout {
                    ui.weak(format!("needs {st:.1} mm stick-out"));
                }
            });
        }
    }

    pub(super) fn post_panel(
        &mut self,
        ui: &mut egui::Ui,
        project: &mut Project,
        dialogs: &mut dyn Dialogs,
    ) {
        ui.separator();
        ui.heading("Post");
        let setup = self.current_setup(project).clone();
        let post_id = self.post_id_of(&setup);
        let ext = program_extension(&post_id);
        let (mut pick_machine, mut pick_post, mut refresh) = (None, None, false);
        ui.horizontal(|ui| {
            ui.label("Machine");
            egui::ComboBox::from_id_salt("cam_machine")
                .selected_text(&setup.machine)
                .show_ui(ui, |ui| {
                    for m in &self.library.machines {
                        if ui
                            .selectable_label(m.name == setup.machine, &m.name)
                            .clicked()
                        {
                            pick_machine = Some(m.name.clone());
                        }
                    }
                });
        });
        ui.horizontal(|ui| {
            ui.label("Post");
            egui::ComboBox::from_id_salt("cam_post")
                .selected_text(&post_id)
                .show_ui(ui, |ui| {
                    for p in &self.library.posts {
                        if ui.selectable_label(p.id == post_id, &p.id).clicked() {
                            pick_post = Some(p.id.clone());
                        }
                    }
                });
        });
        ui.horizontal(|ui| {
            if ui
                .small_button("Update from library")
                .on_hover_text("Refresh the machine and post copies kept in this project")
                .clicked()
            {
                refresh = true;
            }
            if ui.small_button("Libraries…").clicked() {
                self.library_open = true;
            }
        });
        if let Some(m) = pick_machine {
            if let Err(e) = self.set_machine(project, &m) {
                self.message = e;
            }
        }
        if let Some(p) = pick_post {
            if let Err(e) = self.set_post(project, &p) {
                self.message = e;
            }
        }
        if refresh {
            let n = self.refresh_from_library(project);
            self.message = format!("{n} copy(ies) in this project updated from the library");
        }
        match self.setup_post_text(project, self.current) {
            Some(Ok(text)) => {
                ui.label(format!(
                    "setup program: {} lines, {:.0} s",
                    text.lines().count(),
                    project.cam.get(self.current).map_or(0.0, |s| s
                        .operations
                        .iter()
                        .map(|o| setup::estimate_seconds(&o.path))
                        .sum())
                ));
                if let Some(Ok((_, rep))) =
                    project.cam.get(self.current).map(|c| self.program_of(c))
                {
                    if rep.converted {
                        let colour = if rep.ok() {
                            egui::Color32::from_rgb(90, 180, 110)
                        } else {
                            egui::Color32::from_rgb(230, 150, 60)
                        };
                        ui.colored_label(colour, format!("rotary axes: {}", rep.summary()));
                    }
                }
                ui.horizontal(|ui| {
                    if ui.button("Save setup G-code…").clicked() {
                        let name = format!(
                            "{}_{}.{ext}",
                            project.name(),
                            project.cam[self.current].name.replace(' ', "_")
                        );
                        if let Some(p) = dialogs.save_file(&name, "G-code", &[ext]) {
                            self.message = match std::fs::write(&p, &text) {
                                Ok(()) => format!("wrote {}", p.display()),
                                Err(e) => format!("could not write: {e}"),
                            };
                        }
                    }
                    if project.cam.len() > 1 && ui.button("Save all setups…").clicked() {
                        if let Some(dir) = dialogs.choose_folder() {
                            self.message = self.write_all(project, &dir);
                        }
                    }
                });
            }
            Some(Err(e)) => {
                ui.colored_label(egui::Color32::from_rgb(230, 110, 90), e);
            }
            None => {
                ui.weak("Add an operation to post the setup.");
            }
        }
        if project.cam.iter().any(|s| !s.operations.is_empty())
            && ui.button("Setup sheet…").clicked()
        {
            if let Some(p) = dialogs.save_file(
                &format!("{}_setup_sheet.html", project.name()),
                "Web page",
                &["html"],
            ) {
                self.message = self.write_sheet(project, &p);
            }
        }
        if let Some(Ok(text)) = self.post_text(project) {
            ui.label(format!(
                "selected operation: {} lines",
                text.lines().count()
            ));
            egui::ScrollArea::vertical()
                .max_height(160.0)
                .show(ui, |ui| {
                    ui.add(
                        egui::Label::new(
                            egui::RichText::new(
                                text.lines().take(40).collect::<Vec<_>>().join("\n"),
                            )
                            .monospace()
                            .size(10.0),
                        )
                        .wrap_mode(egui::TextWrapMode::Extend),
                    );
                });
        }
    }
}
