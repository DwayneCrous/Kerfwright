//! The post-processor: a small template language that turns cutter-location events into controller text.
//!
//! A post file is `key = value` lines (`#` starts a comment). Settings:
//!
//! | key | meaning |
//! |---|---|
//! | `name` | dialect name |
//! | `decimals` | digits after the decimal point |
//! | `rot_decimals` | digits for rotary axes (default: `decimals`) |
//! | `trim` | `none` (default), `zeros` (`10.500` → `10.5`, `10.000` → `10.`) or `dot` (`10`) |
//! | `plus` | `true` writes `+` before positive numbers |
//! | `line_numbers` | prefix template with `{n}` (`N{n} `, `{n} `), empty for none |
//! | `line_start`, `line_step` | numbering |
//! | `comment` | comment template with `{text}` |
//! | `modal` | `true` omits axis words (and feed) whose value did not change |
//! | `helical_arcs` | `false` writes arcs that change z as short line segments |
//! | `cw_text`, `ccw_text` | text of `{dir}` for arcs |
//!
//! Every other key is the template of an event: `program_start`, `program_end`, `tool_change`,
//! `spindle_on_cw`, `spindle_on_ccw`, `spindle_off`, `coolant_on`, `coolant_off`, `rapid`, `feed`,
//! `arc_cw`, `arc_ccw`, `drill_start`, `drill_peck_start`, `drill_at`, `drill_end`. A template may hold
//! several lines (`\n`); a line starting with `@` gets no line number. Placeholders are `{X}`-style words
//! (letter and number; empty when modal and unchanged) and `{x}`-style bare numbers:
//! `X Y Z F I J CX CY R Q S` words and `x y z f i j cx cy r q rpm tool depth rtp rfp sdis dp program name
//! dia dir text`.

use crate::path::{Event, Toolpath};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct PostConfig {
    pub name: String,
    pub decimals: usize,
    pub rot_decimals: usize,
    pub trim: Trim,
    pub plus: bool,
    pub line_numbers: String,
    pub line_start: u32,
    pub line_step: u32,
    pub comment: String,
    pub modal: bool,
    /// Can one arc block change z? If not, helical arcs are written as short line segments.
    pub helical_arcs: bool,
    pub cw_text: String,
    pub ccw_text: String,
    pub templates: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trim {
    None,
    Zeros,
    Dot,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PostError(pub String);

impl std::fmt::Display for PostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "post-processor: {}", self.0)
    }
}
impl std::error::Error for PostError {}

pub const FANUC: &str = include_str!("../posts/fanuc.post");
pub const HAAS: &str = include_str!("../posts/haas.post");
pub const SIEMENS: &str = include_str!("../posts/siemens840d.post");
pub const HEIDENHAIN: &str = include_str!("../posts/heidenhain.post");
pub const LINUXCNC: &str = include_str!("../posts/linuxcnc.post");

impl PostConfig {
    pub fn parse(text: &str) -> Result<PostConfig, PostError> {
        let mut c = PostConfig {
            name: String::new(),
            decimals: 3,
            rot_decimals: usize::MAX,
            trim: Trim::None,
            plus: false,
            line_numbers: String::new(),
            line_start: 10,
            line_step: 10,
            comment: "({text})".into(),
            modal: true,
            helical_arcs: true,
            cw_text: "-".into(),
            ccw_text: "+".into(),
            templates: HashMap::new(),
        };
        for (i, raw) in text.lines().enumerate() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (k, v) = line
                .split_once('=')
                .ok_or_else(|| PostError(format!("line {}: expected key = value", i + 1)))?;
            let v = v.trim();
            // a value in double quotes keeps its inner spaces ("N{n} ")
            let v = if v.len() >= 2 && v.starts_with('"') && v.ends_with('"') {
                v[1..v.len() - 1].to_string()
            } else {
                v.to_string()
            };
            let (k, v) = (k.trim(), v.replace("\\n", "\n"));
            let bad = |what: &str| PostError(format!("line {}: {what}", i + 1));
            match k {
                "name" => c.name = v,
                "decimals" => c.decimals = v.parse().map_err(|_| bad("decimals is a number"))?,
                "rot_decimals" => {
                    c.rot_decimals = v.parse().map_err(|_| bad("rot_decimals is a number"))?
                }
                "trim" => {
                    c.trim = match v.as_str() {
                        "none" => Trim::None,
                        "zeros" => Trim::Zeros,
                        "dot" => Trim::Dot,
                        _ => return Err(bad("trim is none, zeros or dot")),
                    }
                }
                "plus" => c.plus = v == "true",
                "line_numbers" => c.line_numbers = v,
                "line_start" => {
                    c.line_start = v.parse().map_err(|_| bad("line_start is a number"))?
                }
                "line_step" => c.line_step = v.parse().map_err(|_| bad("line_step is a number"))?,
                "comment" => c.comment = v,
                "modal" => c.modal = v == "true",
                "helical_arcs" => c.helical_arcs = v == "true",
                "cw_text" => c.cw_text = v,
                "ccw_text" => c.ccw_text = v,
                other => {
                    const KNOWN: [&str; 20] = [
                        "program_start",
                        "program_end",
                        "tool_change",
                        "spindle_on_cw",
                        "spindle_on_ccw",
                        "spindle_off",
                        "coolant_on",
                        "coolant_off",
                        "rapid",
                        "feed",
                        "arc_cw",
                        "arc_ccw",
                        "drill_start",
                        "drill_peck_start",
                        "drill_at",
                        "drill_end",
                        "rapid5",
                        "feed5",
                        "tcp_on",
                        "tcp_off",
                    ];
                    if !KNOWN.contains(&other) {
                        return Err(bad(&format!("unknown key '{other}'")));
                    }
                    c.templates.insert(other.to_string(), v);
                }
            }
        }
        if c.name.is_empty() {
            return Err(PostError("the post has no name".into()));
        }
        if c.rot_decimals == usize::MAX {
            c.rot_decimals = c.decimals;
        }
        Ok(c)
    }

    pub fn fanuc() -> PostConfig {
        PostConfig::parse(FANUC).expect("built-in post")
    }
    pub fn haas() -> PostConfig {
        PostConfig::parse(HAAS).expect("built-in post")
    }
    pub fn siemens() -> PostConfig {
        PostConfig::parse(SIEMENS).expect("built-in post")
    }
    pub fn heidenhain() -> PostConfig {
        PostConfig::parse(HEIDENHAIN).expect("built-in post")
    }
    pub fn linuxcnc() -> PostConfig {
        PostConfig::parse(LINUXCNC).expect("built-in post")
    }

    /// An unsigned number (feed rates, speeds): never carries a `+`.
    fn numu(&self, v: f64) -> String {
        let mut c = self.clone();
        c.plus = false;
        c.num(v)
    }

    fn num(&self, v: f64) -> String {
        self.num_dec(v, self.decimals)
    }

    fn num_dec(&self, v: f64, decimals: usize) -> String {
        let scale = 10f64.powi(decimals as i32);
        let mut v = (v * scale).round() / scale;
        if v == 0.0 {
            v = 0.0; // no negative zero
        }
        let mut s = format!("{:.*}", decimals, v);
        match self.trim {
            Trim::None => {}
            Trim::Zeros | Trim::Dot => {
                if s.contains('.') {
                    while s.ends_with('0') {
                        s.pop();
                    }
                    if self.trim == Trim::Dot && s.ends_with('.') {
                        s.pop();
                    }
                }
                if s == "-0" || s == "-0." {
                    s = s.replace('-', "");
                }
                if s.is_empty() {
                    s.push('0');
                }
            }
        }
        if self.plus && v >= 0.0 {
            s.insert(0, '+');
        }
        s
    }
}

struct State {
    last: HashMap<&'static str, f64>,
    n: u32,
}

/// Render the toolpath as program text.
pub fn render(cfg: &PostConfig, path: &Toolpath) -> Result<String, PostError> {
    let mut out = String::new();
    let mut st = State {
        last: HashMap::new(),
        n: cfg.line_start,
    };
    let emit = |out: &mut String, st: &mut State, text: &str| {
        for line in text.split('\n') {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if let Some(rest) = line.strip_prefix('@') {
                out.push_str(rest.trim());
            } else {
                if !cfg.line_numbers.is_empty() {
                    out.push_str(&cfg.line_numbers.replace("{n}", &st.n.to_string()));
                    st.n += cfg.line_step;
                }
                out.push_str(line);
            }
            out.push('\n');
        }
    };
    let tpl = |key: &str| -> Result<&String, PostError> {
        cfg.templates
            .get(key)
            .ok_or_else(|| PostError(format!("the post '{}' has no template '{key}'", cfg.name)))
    };
    let mut vars: Vec<(String, String)> = Vec::new();
    let set = |vars: &mut Vec<(String, String)>, k: &str, v: String| {
        vars.retain(|(n, _)| n != k);
        vars.push((k.to_string(), v));
    };
    let program_id = 1u32;
    // program start
    vars.clear();
    set(&mut vars, "program", format!("{program_id:04}"));
    set(&mut vars, "name", ident(&path.name));
    emit(
        &mut out,
        &mut st,
        &expand(tpl("program_start")?, &vars, cfg),
    );
    let mut depth_state: Option<(f64, f64)> = None;
    let mut cur_tool = 1u32;
    let mut skip_next_at = false;
    for (ei, ev) in path.events.iter().enumerate() {
        vars.clear();
        set(&mut vars, "name", ident(&path.name));
        match ev {
            Event::Comment(t) => {
                let line = cfg.comment.replace("{text}", &sanitize(t));
                emit(&mut out, &mut st, &line);
            }
            Event::ToolChange {
                number,
                name,
                diameter,
                rpm,
            } => {
                st.last.clear();
                cur_tool = *number;
                set(&mut vars, "tool", number.to_string());
                set(&mut vars, "name", sanitize(name));
                let _ = name;
                set(&mut vars, "dia", cfg.num(*diameter));
                set(&mut vars, "rpm", format!("{}", rpm.round() as i64));
                emit(&mut out, &mut st, &expand(tpl("tool_change")?, &vars, cfg));
            }
            Event::SpindleOn { rpm, clockwise } => {
                set(&mut vars, "rpm", format!("{}", rpm.round() as i64));
                set(&mut vars, "S", format!("S{}", rpm.round() as i64));
                let key = if *clockwise {
                    "spindle_on_cw"
                } else {
                    "spindle_on_ccw"
                };
                emit(&mut out, &mut st, &expand(tpl(key)?, &vars, cfg));
            }
            Event::SpindleOff => emit(&mut out, &mut st, &expand(tpl("spindle_off")?, &vars, cfg)),
            Event::CoolantOn => emit(&mut out, &mut st, &expand(tpl("coolant_on")?, &vars, cfg)),
            Event::CoolantOff => emit(&mut out, &mut st, &expand(tpl("coolant_off")?, &vars, cfg)),
            Event::Rapid { p } => {
                axis_vars(cfg, &mut st, &mut vars, p, false);
                emit(&mut out, &mut st, &expand(tpl("rapid")?, &vars, cfg));
            }
            Event::Feed { p, feed } => {
                axis_vars(cfg, &mut st, &mut vars, p, false);
                feed_vars(cfg, &mut st, &mut vars, *feed);
                emit(&mut out, &mut st, &expand(tpl("feed")?, &vars, cfg));
            }
            Event::Rapid5 { p, rot, names } => {
                axis_vars(cfg, &mut st, &mut vars, p, false);
                rot_vars(cfg, &mut st, &mut vars, rot, names);
                emit(&mut out, &mut st, &expand(tpl("rapid5")?, &vars, cfg));
            }
            Event::Feed5 {
                p,
                rot,
                names,
                feed,
            } => {
                axis_vars(cfg, &mut st, &mut vars, p, false);
                rot_vars(cfg, &mut st, &mut vars, rot, names);
                feed_vars(cfg, &mut st, &mut vars, *feed);
                emit(&mut out, &mut st, &expand(tpl("feed5")?, &vars, cfg));
            }
            Event::Tcp { on } => {
                set(&mut vars, "tool", cur_tool.to_string());
                let key = if *on { "tcp_on" } else { "tcp_off" };
                emit(&mut out, &mut st, &expand(tpl(key)?, &vars, cfg));
            }
            Event::Arc {
                p,
                centre,
                ccw,
                feed,
            } => {
                // the start point of the arc is the last written position
                let start = (
                    st.last.get("X").copied().unwrap_or(0.0),
                    st.last.get("Y").copied().unwrap_or(0.0),
                );
                let z0 = st.last.get("Z").copied().unwrap_or(0.0);
                if !cfg.helical_arcs && (p[2] - z0).abs() > 0.5 * 10f64.powi(-(cfg.decimals as i32))
                {
                    // approximate by chords no more than 1 µm from the arc
                    let r = (start.0 - centre[0]).hypot(start.1 - centre[1]);
                    let a0 = (start.1 - centre[1]).atan2(start.0 - centre[0]);
                    let a1 = (p[1] - centre[1]).atan2(p[0] - centre[0]);
                    let mut d = a1 - a0;
                    if *ccw {
                        while d <= 1e-9 {
                            d += std::f64::consts::TAU;
                        }
                    } else {
                        while d >= -1e-9 {
                            d -= std::f64::consts::TAU;
                        }
                    }
                    let step = 2.0
                        * (1.0 - 0.001 / r.max(0.01))
                            .clamp(-1.0, 1.0)
                            .acos()
                            .max(1e-3);
                    let n = ((d.abs() / step).ceil() as usize).max(2);
                    for k in 1..=n {
                        let t = k as f64 / n as f64;
                        let a = a0 + d * t;
                        let q = if k == n {
                            *p
                        } else {
                            [
                                centre[0] + r * a.cos(),
                                centre[1] + r * a.sin(),
                                z0 + (p[2] - z0) * t,
                            ]
                        };
                        let mut v2: Vec<(String, String)> =
                            vec![("name".into(), ident(&path.name))];
                        axis_vars(cfg, &mut st, &mut v2, &q, false);
                        feed_vars(cfg, &mut st, &mut v2, *feed);
                        emit(&mut out, &mut st, &expand(tpl("feed")?, &v2, cfg));
                    }
                    continue;
                }
                axis_vars(cfg, &mut st, &mut vars, p, true);
                feed_vars(cfg, &mut st, &mut vars, *feed);
                let (i, j) = (centre[0] - start.0, centre[1] - start.1);
                set(&mut vars, "I", format!("I{}", cfg.num(i)));
                set(&mut vars, "J", format!("J{}", cfg.num(j)));
                set(&mut vars, "i", cfg.num(i));
                set(&mut vars, "j", cfg.num(j));
                set(&mut vars, "CX", format!("X{}", cfg.num(centre[0])));
                set(&mut vars, "CY", format!("Y{}", cfg.num(centre[1])));
                set(&mut vars, "cx", cfg.num(centre[0]));
                set(&mut vars, "cy", cfg.num(centre[1]));
                set(
                    &mut vars,
                    "dir",
                    if *ccw {
                        cfg.ccw_text.clone()
                    } else {
                        cfg.cw_text.clone()
                    },
                );
                emit(
                    &mut out,
                    &mut st,
                    &expand(tpl(if *ccw { "arc_ccw" } else { "arc_cw" })?, &vars, cfg),
                );
            }
            Event::DrillStart {
                depth,
                retract,
                surface,
                peck,
                feed,
            } => {
                set(&mut vars, "Z", format!("Z{}", cfg.num(*depth)));
                set(&mut vars, "R", format!("R{}", cfg.num(*retract)));
                set(&mut vars, "F", format!("F{}", cfg.numu(*feed)));
                set(&mut vars, "f", cfg.numu(*feed));
                set(&mut vars, "depth", cfg.num(*depth));
                set(&mut vars, "dp", cfg.num(*depth));
                set(&mut vars, "rtp", cfg.num(*retract));
                set(&mut vars, "rfp", cfg.num(*surface));
                set(&mut vars, "sdis", cfg.num((retract - surface).max(0.0)));
                if let Some(q) = peck {
                    set(&mut vars, "Q", format!("Q{}", cfg.num(*q)));
                    set(&mut vars, "q", cfg.num(*q));
                }
                depth_state = Some((*depth, *retract));
                let key = if peck.is_some() {
                    "drill_peck_start"
                } else {
                    "drill_start"
                };
                if tpl(key)?.contains("{X}") {
                    // the first hole is part of the cycle block (Fanuc style)
                    if let Some(Event::DrillAt { x, y }) = path.events.get(ei + 1) {
                        set(&mut vars, "X", format!("X{}", cfg.num(*x)));
                        set(&mut vars, "Y", format!("Y{}", cfg.num(*y)));
                        st.last.insert("X", *x);
                        st.last.insert("Y", *y);
                        skip_next_at = true;
                    }
                }
                emit(&mut out, &mut st, &expand(tpl(key)?, &vars, cfg));
                st.last.remove("Z");
            }
            Event::DrillAt { .. } if skip_next_at => {
                skip_next_at = false;
            }
            Event::DrillAt { x, y } => {
                set(&mut vars, "X", format!("X{}", cfg.num(*x)));
                set(&mut vars, "Y", format!("Y{}", cfg.num(*y)));
                set(&mut vars, "x", cfg.num(*x));
                set(&mut vars, "y", cfg.num(*y));
                if let Some((d, r)) = depth_state {
                    set(&mut vars, "depth", cfg.num(d));
                    set(&mut vars, "rtp", cfg.num(r));
                }
                emit(&mut out, &mut st, &expand(tpl("drill_at")?, &vars, cfg));
                st.last.insert("X", *x);
                st.last.insert("Y", *y);
            }
            Event::DrillEnd => {
                emit(&mut out, &mut st, &expand(tpl("drill_end")?, &vars, cfg));
                depth_state = None;
            }
            Event::ProgramEnd => {
                set(&mut vars, "program", format!("{program_id:04}"));
                emit(&mut out, &mut st, &expand(tpl("program_end")?, &vars, cfg));
            }
        }
    }
    Ok(out)
}

/// A program name usable as an identifier by every controller.
fn ident(t: &str) -> String {
    let s: String = t
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() {
                c.to_ascii_uppercase()
            } else {
                '_'
            }
        })
        .collect();
    if s.is_empty() {
        "PROGRAM".into()
    } else {
        s
    }
}

fn sanitize(t: &str) -> String {
    t.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == ' ' || c == '.' || c == '-' || c == '_' || c == ','
            {
                c
            } else {
                ' '
            }
        })
        .collect()
}

fn axis_vars(
    cfg: &PostConfig,
    st: &mut State,
    vars: &mut Vec<(String, String)>,
    p: &[f64; 3],
    force_xy: bool,
) {
    for (i, (k, v)) in [("X", p[0]), ("Y", p[1]), ("Z", p[2])]
        .into_iter()
        .enumerate()
    {
        let rounded = (v * 10f64.powi(cfg.decimals as i32)).round();
        let unchanged = st
            .last
            .get(k)
            .is_some_and(|l| ((l * 10f64.powi(cfg.decimals as i32)).round() - rounded).abs() < 0.5);
        let word = if cfg.modal && unchanged && !(force_xy && i < 2) {
            String::new()
        } else {
            format!("{k}{}", cfg.num(v))
        };
        vars.push((k.to_string(), word));
        vars.push((k.to_lowercase(), cfg.num(v)));
        st.last.insert(k_static(k), v);
    }
}

fn k_static(k: &str) -> &'static str {
    match k {
        "X" => "X",
        "Y" => "Y",
        "A" => "A",
        "B" => "B",
        "C" => "C",
        _ => "Z",
    }
}

/// Rotary words `{A} {B} {C}` (empty for axes the machine does not have, and modal when unchanged).
fn rot_vars(
    cfg: &PostConfig,
    st: &mut State,
    vars: &mut Vec<(String, String)>,
    rot: &[f64; 2],
    names: &[char; 2],
) {
    for k in ["A", "B", "C"] {
        let found = names.iter().position(|n| n.to_string() == k);
        let (word, bare) = match found {
            None => (String::new(), String::new()),
            Some(i) => {
                let v = rot[i];
                let scale = 10f64.powi(cfg.rot_decimals as i32);
                let unchanged = st
                    .last
                    .get(k_static(k))
                    .is_some_and(|l| ((l * scale).round() - (v * scale).round()).abs() < 0.5);
                st.last.insert(k_static(k), v);
                let text = cfg.num_dec(v, cfg.rot_decimals);
                (
                    if cfg.modal && unchanged {
                        String::new()
                    } else {
                        format!("{k}{text}")
                    },
                    text,
                )
            }
        };
        vars.push((k.to_string(), word));
        vars.push((k.to_lowercase(), bare));
    }
}

fn feed_vars(cfg: &PostConfig, st: &mut State, vars: &mut Vec<(String, String)>, feed: f64) {
    let unchanged = st.last.get("F").is_some_and(|l| (l - feed).abs() < 1e-9);
    let word = if cfg.modal && unchanged {
        String::new()
    } else {
        format!("F{}", cfg.numu(feed))
    };
    vars.push(("F".into(), word));
    vars.push(("f".into(), cfg.numu(feed)));
    st.last.insert("F", feed);
}

fn expand(template: &str, vars: &[(String, String)], cfg: &PostConfig) -> String {
    let mut out = String::new();
    let mut rest = template;
    while let Some(i) = rest.find('{') {
        out.push_str(&rest[..i]);
        let Some(j) = rest[i..].find('}') else {
            out.push_str(&rest[i..]);
            rest = "";
            break;
        };
        let key = &rest[i + 1..i + j];
        match vars.iter().find(|(n, _)| n == key) {
            Some((_, v)) => out.push_str(v),
            None => out.push_str(&format!("{{{key}}}")), // left visible so the mistake shows
        }
        rest = &rest[i + j + 1..];
    }
    out.push_str(rest);
    let _ = cfg;
    // collapse the gaps that empty (modal) words leave behind
    out.lines()
        .map(|l| l.split_whitespace().collect::<Vec<_>>().join(" "))
        .collect::<Vec<_>>()
        .join("\n")
}
