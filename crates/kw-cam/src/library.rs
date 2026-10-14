//! The libraries a machining job draws on: tool presets with their cutting data, tool holders, machines
//! and post-processors. A built-in set ships with the program; the user's own entries live in files under
//! the configuration folder and are laid over it by name. A project embeds copies of what it uses, so a file
//! opened on another computer still posts the same program.
//!
//! Layout of the user folder (`<config>/cam/`): `tools.json`, `holders.json`, `machines/<name>.json` and
//! `posts/<name>.post`. Every file can be exported and imported on its own.

use crate::kin::{self, Machine};
use crate::post::{PostConfig, FANUC, HAAS, HEIDENHAIN, LINUXCNC, SIEMENS};
use crate::tool::{Holder, Tool};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// Cutting data of a tool in one material group (the names are those of `kw-materials`' groups).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CutData {
    pub group: String,
    /// Surface speed, m/min.
    pub vc: f64,
    /// Feed per tooth, mm.
    pub fz: f64,
}

/// A tool preset: the tool and what it should be run at.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ToolDef {
    pub name: String,
    pub tool: Tool,
    #[serde(default)]
    pub cutting: Vec<CutData>,
}

/// A holder: cylindrical sections above the tool's shank (radius, length), from the tool up.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HolderDef {
    pub name: String,
    pub sections: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineDef {
    pub name: String,
    pub spindle_min: f64,
    pub spindle_max: f64,
    /// Rapid traverse, mm/min.
    pub rapid: f64,
    /// The highest cutting feed, mm/min.
    pub max_feed: f64,
    /// Travel of x, y and z, mm.
    pub travel: [[f64; 2]; 3],
    pub tool_capacity: u32,
    /// Id of the post-processor that writes this machine's programs.
    pub post: String,
    /// Rotary axes of a five-axis machine.
    #[serde(default)]
    pub kinematics: Option<Machine>,
}

impl MachineDef {
    pub fn axes(&self) -> usize {
        self.kinematics.as_ref().map_or(3, |k| k.axis_count())
    }
}

/// A post-processor in the template language of [`PostConfig`]; its id is the `name` the text gives itself.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PostDef {
    pub id: String,
    pub text: String,
}

impl PostDef {
    /// A post from its text, validated: the parse error names the line.
    pub fn from_text(text: &str) -> Result<PostDef, String> {
        let cfg = PostConfig::parse(text).map_err(|e| e.to_string())?;
        Ok(PostDef {
            id: cfg.name,
            text: text.to_string(),
        })
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Library {
    pub tools: Vec<ToolDef>,
    pub holders: Vec<HolderDef>,
    pub machines: Vec<MachineDef>,
    pub posts: Vec<PostDef>,
}

/// The folder of the user's CAM libraries: `$KERFWRIGHT_CONFIG/cam`, else `$XDG_CONFIG_HOME/kerfwright/cam`,
/// else `~/.config/kerfwright/cam`.
pub fn config_dir() -> Option<PathBuf> {
    if let Some(d) = std::env::var_os("KERFWRIGHT_CONFIG") {
        return Some(PathBuf::from(d).join("cam"));
    }
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("kerfwright").join("cam"))
}

fn tool_def(tool: Tool) -> ToolDef {
    ToolDef {
        name: tool.name.clone(),
        tool,
        cutting: Vec::new(),
    }
}

impl Library {
    /// The libraries that ship with the program.
    pub fn builtin() -> Library {
        let mut tools = Vec::new();
        for d in [2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0] {
            tools.push(tool_def(Tool::flat(1, d, (2.5 * d).max(6.0))));
        }
        for d in [2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0] {
            tools.push(tool_def(Tool::ball(1, d, (2.5 * d).max(6.0))));
        }
        for (d, r) in [(6.0, 1.0), (10.0, 1.0), (12.0, 2.0), (16.0, 3.0)] {
            tools.push(tool_def(Tool::bull(1, d, r, 2.5 * d)));
        }
        for d in [2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0] {
            tools.push(tool_def(Tool::drill(1, d, 118.0, 6.0 * d)));
        }
        for (d, tip) in [(6.0, 0.2), (10.0, 0.2), (12.0, 0.4)] {
            tools.push(tool_def(Tool::chamfer(1, d, 90.0, tip, 12.0)));
        }
        for d in [50.0, 63.0, 80.0] {
            tools.push(tool_def(Tool::face_mill(1, d)));
        }
        tools.push(tool_def(Tool::engraver(1, 60.0, 0.1)));
        tools.push(tool_def(Tool::bore(1, 10.0, 25.0)));
        tools.push(tool_def(Tool::tap(1, 6.0, 1.0, 15.0)));
        tools.push(tool_def(Tool::tap(1, 8.0, 1.25, 20.0)));
        let holders = vec![
            HolderDef {
                name: "ER16 collet chuck".into(),
                sections: vec![(14.0, 32.0)],
            },
            HolderDef {
                name: "ER20 collet chuck".into(),
                sections: vec![(17.0, 40.0)],
            },
            HolderDef {
                name: "ER32 collet chuck".into(),
                sections: vec![(25.0, 45.0)],
            },
            HolderDef {
                name: "Shrink fit".into(),
                sections: vec![(10.0, 20.0), (20.0, 50.0)],
            },
            HolderDef {
                name: "Side lock".into(),
                sections: vec![(17.0, 50.0)],
            },
        ];
        let mut machines = vec![
            MachineDef {
                name: "3-axis mill".into(),
                spindle_min: 100.0,
                spindle_max: 12_000.0,
                rapid: 10_000.0,
                max_feed: 6_000.0,
                travel: [[-300.0, 300.0], [-200.0, 200.0], [-250.0, 0.0]],
                tool_capacity: 20,
                post: "Fanuc".into(),
                kinematics: None,
            },
            MachineDef {
                name: "Haas VF-2".into(),
                spindle_min: 50.0,
                spindle_max: 8_100.0,
                rapid: 25_400.0,
                max_feed: 16_500.0,
                travel: [[0.0, 762.0], [0.0, 406.0], [-508.0, 0.0]],
                tool_capacity: 20,
                post: "Haas".into(),
                kinematics: None,
            },
            MachineDef {
                name: "LinuxCNC 3-axis".into(),
                spindle_min: 100.0,
                spindle_max: 10_000.0,
                rapid: 5_000.0,
                max_feed: 3_000.0,
                travel: [[0.0, 400.0], [0.0, 300.0], [-200.0, 0.0]],
                tool_capacity: 1,
                post: "LinuxCNC".into(),
                kinematics: None,
            },
        ];
        for m in kin::test_machines() {
            machines.push(MachineDef {
                name: m.name.clone(),
                spindle_min: 100.0,
                spindle_max: 20_000.0,
                rapid: 20_000.0,
                max_feed: 10_000.0,
                travel: [[-400.0, 400.0], [-300.0, 300.0], [-300.0, 0.0]],
                tool_capacity: 30,
                post: "Fanuc".into(),
                kinematics: Some(m),
            });
        }
        let posts = [FANUC, HAAS, SIEMENS, HEIDENHAIN, LINUXCNC]
            .iter()
            .map(|t| PostDef::from_text(t).expect("built-in post"))
            .collect();
        Library {
            tools,
            holders,
            machines,
            posts,
        }
    }

    pub fn tool(&self, name: &str) -> Option<&ToolDef> {
        self.tools.iter().find(|t| t.name == name)
    }

    pub fn holder(&self, name: &str) -> Option<&HolderDef> {
        self.holders.iter().find(|h| h.name == name)
    }

    pub fn machine(&self, name: &str) -> Option<&MachineDef> {
        self.machines.iter().find(|m| m.name == name)
    }

    pub fn post(&self, id: &str) -> Option<&PostDef> {
        self.posts.iter().find(|p| p.id == id)
    }

    /// The parsed post with this id.
    pub fn post_config(&self, id: &str) -> Result<PostConfig, String> {
        let p = self
            .post(id)
            .ok_or_else(|| format!("there is no post-processor '{id}'"))?;
        PostConfig::parse(&p.text).map_err(|e| e.to_string())
    }

    pub fn is_builtin_post(id: &str) -> bool {
        Library::builtin().post(id).is_some()
    }

    /// Put `tool` into the library, replacing the entry with the same name.
    pub fn put_tool(&mut self, d: ToolDef) {
        match self.tools.iter_mut().find(|t| t.name == d.name) {
            Some(t) => *t = d,
            None => self.tools.push(d),
        }
    }

    pub fn put_holder(&mut self, h: HolderDef) {
        match self.holders.iter_mut().find(|x| x.name == h.name) {
            Some(x) => *x = h,
            None => self.holders.push(h),
        }
    }

    pub fn put_machine(&mut self, m: MachineDef) {
        match self.machines.iter_mut().find(|x| x.name == m.name) {
            Some(x) => *x = m,
            None => self.machines.push(m),
        }
    }

    pub fn put_post(&mut self, p: PostDef) {
        match self.posts.iter_mut().find(|x| x.id == p.id) {
            Some(x) => *x = p,
            None => self.posts.push(p),
        }
    }

    /// The built-in libraries with the user's files in `dir` laid over them. A file that cannot be read is
    /// reported in the second value and skipped; a missing folder is not a problem.
    pub fn load_dir(dir: &Path) -> (Library, Vec<String>) {
        let mut lib = Library::builtin();
        let mut warnings = Vec::new();
        let mut read = |path: &Path, f: &mut dyn FnMut(&str) -> Result<(), String>| {
            if let Ok(text) = std::fs::read_to_string(path) {
                if let Err(e) = f(&text) {
                    warnings.push(format!("{}: {e}", path.display()));
                }
            }
        };
        read(&dir.join("tools.json"), &mut |t| {
            let v: Vec<ToolDef> = serde_json::from_str(t).map_err(|e| e.to_string())?;
            v.into_iter().for_each(|d| lib.put_tool(d));
            Ok(())
        });
        read(&dir.join("holders.json"), &mut |t| {
            let v: Vec<HolderDef> = serde_json::from_str(t).map_err(|e| e.to_string())?;
            v.into_iter().for_each(|d| lib.put_holder(d));
            Ok(())
        });
        for (sub, ext) in [("machines", "json"), ("posts", "post")] {
            let Ok(rd) = std::fs::read_dir(dir.join(sub)) else {
                continue;
            };
            let mut files: Vec<PathBuf> = rd
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().is_some_and(|e| e == ext))
                .collect();
            files.sort();
            for f in files {
                if sub == "machines" {
                    read(&f, &mut |t| {
                        lib.put_machine(serde_json::from_str(t).map_err(|e| e.to_string())?);
                        Ok(())
                    });
                } else {
                    read(&f, &mut |t| {
                        lib.put_post(PostDef::from_text(t)?);
                        Ok(())
                    });
                }
            }
        }
        (lib, warnings)
    }

    /// Write what differs from the built-in libraries into `dir`.
    pub fn save_dir(&self, dir: &Path) -> Result<(), String> {
        let base = Library::builtin();
        let io = |e: std::io::Error| e.to_string();
        std::fs::create_dir_all(dir).map_err(io)?;
        let extra_tools: Vec<&ToolDef> = self
            .tools
            .iter()
            .filter(|t| base.tool(&t.name) != Some(*t))
            .collect();
        let extra_holders: Vec<&HolderDef> = self
            .holders
            .iter()
            .filter(|h| base.holder(&h.name) != Some(*h))
            .collect();
        let json = |v: &dyn erased::Json| v.to_json();
        std::fs::write(dir.join("tools.json"), json(&extra_tools)).map_err(io)?;
        std::fs::write(dir.join("holders.json"), json(&extra_holders)).map_err(io)?;
        let machines = dir.join("machines");
        let posts = dir.join("posts");
        std::fs::create_dir_all(&machines).map_err(io)?;
        std::fs::create_dir_all(&posts).map_err(io)?;
        // the folders hold exactly the user's entries: remove the files of entries that are gone
        for (folder, ext) in [(&machines, "json"), (&posts, "post")] {
            for e in std::fs::read_dir(folder).map_err(io)?.flatten() {
                if e.path().extension().is_some_and(|x| x == ext) {
                    std::fs::remove_file(e.path()).map_err(io)?;
                }
            }
        }
        for m in self
            .machines
            .iter()
            .filter(|m| base.machine(&m.name) != Some(*m))
        {
            std::fs::write(
                machines.join(format!("{}.json", file_stem(&m.name))),
                json(m),
            )
            .map_err(io)?;
        }
        for p in self.posts.iter().filter(|p| base.post(&p.id) != Some(*p)) {
            std::fs::write(posts.join(format!("{}.post", file_stem(&p.id))), &p.text)
                .map_err(io)?;
        }
        Ok(())
    }

    /// Replace the entries of the loaded library by the ones in an exported file.
    pub fn import_tools(&mut self, json: &str) -> Result<usize, String> {
        let v: Vec<ToolDef> = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let n = v.len();
        v.into_iter().for_each(|d| self.put_tool(d));
        Ok(n)
    }

    pub fn import_holders(&mut self, json: &str) -> Result<usize, String> {
        let v: Vec<HolderDef> = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let n = v.len();
        v.into_iter().for_each(|d| self.put_holder(d));
        Ok(n)
    }

    pub fn import_machine(&mut self, json: &str) -> Result<String, String> {
        let m: MachineDef = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let name = m.name.clone();
        self.put_machine(m);
        Ok(name)
    }

    pub fn import_post(&mut self, text: &str) -> Result<String, String> {
        let p = PostDef::from_text(text)?;
        let id = p.id.clone();
        self.put_post(p);
        Ok(id)
    }

    pub fn export_tools(&self) -> String {
        serde_json::to_string_pretty(&self.tools).expect("tools serialise")
    }

    pub fn export_holders(&self) -> String {
        serde_json::to_string_pretty(&self.holders).expect("holders serialise")
    }
}

mod erased {
    pub trait Json {
        fn to_json(&self) -> String;
    }
    impl<T: serde::Serialize + ?Sized> Json for T {
        fn to_json(&self) -> String {
            serde_json::to_string_pretty(self).expect("serialises")
        }
    }
}

fn file_stem(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// The spindle speed and feed of a tool at surface speed `vc` (m/min) and feed per tooth `fz` (mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Speeds {
    pub rpm: f64,
    pub feed: f64,
    /// The machine's limits changed the result.
    pub limited: bool,
}

/// `rpm = 1000 Vc / (π D)`, `feed = rpm · z · fz`, both held to the machine's spindle range and top feed (the
/// feed follows the speed, so a spindle held back cuts back the feed in proportion).
pub fn speeds_and_feeds(tool: &Tool, vc: f64, fz: f64, machine: Option<&MachineDef>) -> Speeds {
    let d = tool.diameter.max(1e-6);
    let mut rpm = 1000.0 * vc / (std::f64::consts::PI * d);
    let mut limited = false;
    if let Some(m) = machine {
        let clamped = rpm.clamp(m.spindle_min, m.spindle_max);
        if (clamped - rpm).abs() > 1e-9 {
            limited = true;
            rpm = clamped;
        }
    }
    let mut feed = rpm * tool.flutes.max(1) as f64 * fz;
    if let Some(m) = machine {
        if feed > m.max_feed {
            feed = m.max_feed;
            limited = true;
        }
    }
    Speeds { rpm, feed, limited }
}

/// The holder sections of a library holder for a tool.
pub fn with_holder(tool: &Tool, holder: &HolderDef) -> Tool {
    Tool {
        holder: Holder {
            sections: holder.sections.clone(),
        },
        holder_name: holder.name.clone(),
        ..tool.clone()
    }
}

/// Give the tools unique numbers, keeping the numbers that already are unique and positive.
pub fn renumber(tools: &mut [Tool]) {
    let mut used: Vec<u32> = Vec::new();
    for t in tools.iter_mut() {
        if t.number == 0 || used.contains(&t.number) {
            let mut n = 1;
            while used.contains(&n) {
                n += 1;
            }
            t.number = n;
        }
        used.push(t.number);
    }
}
