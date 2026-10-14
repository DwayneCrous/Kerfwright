//! Heidenhain TNC conversational ("Klartext") interpreter: a different grammar from G-code.

use crate::{Motion, MotionKind, Run, SimError, P3};

fn err<T>(line: usize, m: impl Into<String>) -> Result<T, SimError> {
    Err(SimError {
        line,
        message: m.into(),
    })
}

fn num(line: usize, s: &str) -> Result<f64, SimError> {
    s.parse::<f64>()
        .or_else(|_| err(line, format!("'{s}' is not a number")))
}

pub fn run_heidenhain(text: &str) -> Result<Run, SimError> {
    let mut run = Run::default();
    let mut pos: P3 = [0.0; 3];
    let mut feed = 0.0;
    let mut rpm = 0.0;
    let mut spindle = false;
    let mut tool = 0u32;
    let mut centre: Option<[f64; 2]> = None;
    let mut cycle: Option<(f64, f64, f64)> = None; // depth, set-up clearance, surface
    let mut expect_block = 0usize;
    let mut begun = false;
    let mut metric = true;
    let mut ended = false;
    let name_of = |s: &str| s.trim().to_string();
    let mut pgm_name = String::new();
    let mut rot = [0.0f64; 3];
    let mut tcp = false;
    let mut stamped = 0usize;
    for (idx, raw) in text.lines().enumerate() {
        // motions of the previous block carry the rotary state that block left behind
        for m in &mut run.motions[stamped..] {
            m.rot = rot;
            m.tcp = tcp;
        }
        stamped = run.motions.len();
        let ln = idx + 1;
        run.lines = ln;
        let mut line = raw.trim().to_string();
        let mut had_comment = false;
        if let Some(i) = line.find(';') {
            line.truncate(i);
            had_comment = true;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if ended {
            return err(ln, "code after END PGM");
        }
        // block number
        let mut parts = line.splitn(2, char::is_whitespace);
        let first = parts.next().unwrap();
        let rest = parts.next().unwrap_or("").trim();
        let n: usize = first
            .parse()
            .or_else(|_| err(ln, "a block starts with its number"))?;
        if n != expect_block {
            return err(
                ln,
                format!("block number {n} where {expect_block} was expected"),
            );
        }
        expect_block += 1;
        let up = rest.to_ascii_uppercase();
        let toks: Vec<&str> = up.split_whitespace().collect();
        if toks.is_empty() {
            if had_comment {
                // a block that holds only a comment
                continue;
            }
            return err(ln, "empty block");
        }
        match toks[0] {
            "BEGIN" => {
                if toks.len() < 4 || toks[1] != "PGM" {
                    return err(ln, "BEGIN PGM <name> MM|INCH");
                }
                if n != 0 {
                    return err(ln, "BEGIN PGM must be block 0");
                }
                pgm_name = name_of(toks[2]);
                metric = match toks[3] {
                    "MM" => true,
                    "INCH" => false,
                    u => return err(ln, format!("unknown unit '{u}'")),
                };
                begun = true;
            }
            _ if !begun => return err(ln, "the program must start with BEGIN PGM"),
            "BLK" => {}
            "END" => {
                if toks.len() < 4 || toks[1] != "PGM" || toks[2] != pgm_name {
                    return err(ln, "END PGM must repeat the program name and unit");
                }
                ended = true;
                run.program_end_seen = true;
            }
            "TOOL" => {
                // TOOL CALL <n> Z S<rpm>
                if toks.len() < 4 || toks[1] != "CALL" {
                    return err(ln, "TOOL CALL <number> Z S<speed>");
                }
                tool = toks[2]
                    .parse()
                    .or_else(|_| err(ln, "tool number expected"))?;
                if toks[3] != "Z" {
                    return err(ln, "only Z tool axes are supported");
                }
                for t in &toks[4..] {
                    if let Some(s) = t.strip_prefix('S') {
                        rpm = num(ln, s)?;
                    } else {
                        return err(ln, format!("unknown word '{t}' in TOOL CALL"));
                    }
                }
                if !run.tools_used.contains(&tool) {
                    run.tools_used.push(tool);
                }
            }
            "FUNCTION" => {
                // FUNCTION TCPM F TCP AXIS POS PATHCTRL AXIS   |   FUNCTION RESET TCPM
                match toks.get(1).copied() {
                    Some("TCPM") => {
                        for t in &toks[2..] {
                            if !matches!(*t, "F" | "TCP" | "AXIS" | "POS" | "PATHCTRL") {
                                return err(ln, format!("unknown word '{t}' in FUNCTION TCPM"));
                            }
                        }
                        tcp = true;
                    }
                    Some("RESET") if toks.get(2) == Some(&"TCPM") => tcp = false,
                    _ => return err(ln, "FUNCTION TCPM … or FUNCTION RESET TCPM"),
                }
            }
            "CC" => {
                let mut c = [pos[0], pos[1]];
                for t in &toks[1..] {
                    match t.chars().next() {
                        Some('X') => c[0] = num(ln, &t[1..])?,
                        Some('Y') => c[1] = num(ln, &t[1..])?,
                        _ => return err(ln, format!("unknown word '{t}' in CC")),
                    }
                }
                centre = Some(c);
            }
            "CYCL" => {
                // CYCL DEF 200 DRILLING Q200=… Q201=… Q206=… Q202=… Q210=… Q203=… Q204=… Q211=…
                if toks.len() < 4 || toks[1] != "DEF" {
                    return err(ln, "CYCL DEF <number> <name> …");
                }
                if toks[2] == "247" {
                    // CYCL DEF 247 DATUM SETTING Q339=<datum number>
                    let ok = toks.get(3) == Some(&"DATUM")
                        && toks.get(4) == Some(&"SETTING")
                        && toks.get(5).is_some_and(|t| t.starts_with("Q339="));
                    if !ok {
                        return err(ln, "CYCL DEF 247 DATUM SETTING Q339=<number>");
                    }
                    continue;
                }
                if toks[2] != "200" {
                    return err(ln, format!("cycle {} is not supported", toks[2]));
                }
                let (mut depth, mut setup, mut surf) = (None, None, 0.0);
                for t in &toks[4..] {
                    if let Some((k, v)) = t.split_once('=') {
                        let v = num(ln, v)?;
                        match k {
                            "Q200" => setup = Some(v),
                            "Q201" => depth = Some(v),
                            "Q203" => surf = v,
                            "Q206" | "Q202" | "Q210" | "Q204" | "Q211" => {}
                            other => return err(ln, format!("unknown cycle parameter {other}")),
                        }
                    } else if !matches!(*t, "DRILLING") {
                        return err(ln, format!("unknown word '{t}'"));
                    }
                }
                let depth = depth
                    .ok_or(())
                    .or_else(|_| err(ln, "the drilling cycle needs Q201 (depth)"))?;
                cycle = Some((depth, setup.unwrap_or(2.0), surf));
            }
            "L" | "C" => {
                let arc = toks[0] == "C";
                let mut target = pos;
                let mut cw = None;
                let mut fmax = false;
                let mut call_cycle = false;
                for t in &toks[1..] {
                    match *t {
                        "FMAX" => fmax = true,
                        "M99" => call_cycle = true,
                        "M3" | "M4" => {
                            if rpm <= 0.0 {
                                return err(ln, "the spindle is started without a speed");
                            }
                            spindle = true;
                        }
                        "M5" => spindle = false,
                        "M8" | "M9" | "M30" | "M0" | "M1" | "M6" => {}
                        "R0" => {}
                        "DR+" => cw = Some(false),
                        "DR-" => cw = Some(true),
                        w => match w.chars().next() {
                            Some('X') => target[0] = num(ln, &w[1..])?,
                            Some('Y') => target[1] = num(ln, &w[1..])?,
                            Some('Z') => target[2] = num(ln, &w[1..])?,
                            Some('A') => rot[0] = num(ln, &w[1..])?,
                            Some('B') => rot[1] = num(ln, &w[1..])?,
                            Some('C') => rot[2] = num(ln, &w[1..])?,
                            Some('F') => feed = num(ln, &w[1..])?,
                            _ => return err(ln, format!("unknown word '{w}'")),
                        },
                    }
                }
                if !metric {
                    return err(ln, "inch programs are not supported by this checker");
                }
                if call_cycle {
                    let (depth, setup, surf) = cycle
                        .ok_or(())
                        .or_else(|_| err(ln, "M99 without a CYCL DEF"))?;
                    if !spindle || tool == 0 {
                        return err(ln, "cycle call with the spindle off or no tool");
                    }
                    let top = surf + setup;
                    let from = pos;
                    run.motions.push(mk(
                        MotionKind::Rapid,
                        from,
                        [target[0], target[1], from[2]],
                        None,
                        0.0,
                        tool,
                        rpm,
                        ln,
                    ));
                    run.motions.push(mk(
                        MotionKind::Rapid,
                        [target[0], target[1], from[2]],
                        [target[0], target[1], top],
                        None,
                        0.0,
                        tool,
                        rpm,
                        ln,
                    ));
                    run.motions.push(mk(
                        MotionKind::Feed,
                        [target[0], target[1], top],
                        [target[0], target[1], surf + depth],
                        None,
                        feed.max(1.0),
                        tool,
                        rpm,
                        ln,
                    ));
                    run.motions.push(mk(
                        MotionKind::Rapid,
                        [target[0], target[1], surf + depth],
                        [target[0], target[1], from[2]],
                        None,
                        0.0,
                        tool,
                        rpm,
                        ln,
                    ));
                    run.drills.push((target[0], target[1], surf + depth, top));
                    pos = [target[0], target[1], from[2]];
                    continue;
                }
                if arc {
                    if fmax {
                        return err(ln, "an arc cannot be traversed at FMAX");
                    }
                    let c = centre
                        .ok_or(())
                        .or_else(|_| err(ln, "C needs a circle centre (CC) first"))?;
                    let cw = cw
                        .ok_or(())
                        .or_else(|_| err(ln, "C needs a direction (DR+ / DR-)"))?;
                    check(ln, feed, spindle, tool)?;
                    let (r0, r1) = (
                        (pos[0] - c[0]).hypot(pos[1] - c[1]),
                        (target[0] - c[0]).hypot(target[1] - c[1]),
                    );
                    if (r0 - r1).abs() > 2e-3 {
                        return err(
                            ln,
                            format!("arc end point is not on the circle ({r0:.4} vs {r1:.4})"),
                        );
                    }
                    let kind = if cw {
                        MotionKind::ArcCw
                    } else {
                        MotionKind::ArcCcw
                    };
                    run.motions
                        .push(mk(kind, pos, target, Some(c), feed, tool, rpm, ln));
                } else if fmax {
                    run.motions
                        .push(mk(MotionKind::Rapid, pos, target, None, 0.0, tool, rpm, ln));
                } else {
                    check(ln, feed, spindle, tool)?;
                    run.motions
                        .push(mk(MotionKind::Feed, pos, target, None, feed, tool, rpm, ln));
                }
                pos = target;
            }
            "M30" | "M2" => {}
            "M5" | "M3" | "M4" | "M8" | "M9" => match toks[0] {
                "M3" | "M4" => {
                    if rpm <= 0.0 {
                        return err(ln, "the spindle is started without a speed");
                    }
                    spindle = true;
                }
                "M5" => spindle = false,
                _ => {}
            },
            other => return err(ln, format!("unknown block type '{other}'")),
        }
    }
    for m in &mut run.motions[stamped..] {
        m.rot = rot;
        m.tcp = tcp;
    }
    if !run.program_end_seen {
        return err(run.lines.max(1), "the program does not end with END PGM");
    }
    Ok(run)
}

fn check(ln: usize, feed: f64, spindle: bool, tool: u32) -> Result<(), SimError> {
    if feed <= 0.0 {
        return err(ln, "a cutting move without a feed rate");
    }
    if !spindle {
        return err(ln, "a cutting move with the spindle off");
    }
    if tool == 0 {
        return err(ln, "a cutting move before any tool was called");
    }
    Ok(())
}

fn mk(
    kind: MotionKind,
    from: P3,
    to: P3,
    centre: Option<[f64; 2]>,
    feed: f64,
    tool: u32,
    rpm: f64,
    line: usize,
) -> Motion {
    Motion {
        kind,
        from,
        to,
        centre,
        feed,
        tool,
        spindle_rpm: rpm,
        line,
        rot: [0.0; 3],
        tcp: false,
    }
}
