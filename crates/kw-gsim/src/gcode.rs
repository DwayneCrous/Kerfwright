//! ISO-style G-code (Fanuc, Haas, LinuxCNC, Siemens 840D) interpreter.

use crate::{Dialect, Motion, MotionKind, Run, SimError, P3};

#[derive(Debug, Clone, PartialEq)]
struct Word {
    letter: char,
    value: f64,
}

fn err<T>(line: usize, m: impl Into<String>) -> Result<T, SimError> {
    Err(SimError {
        line,
        message: m.into(),
    })
}

fn allowed_g(d: Dialect) -> &'static [u32] {
    match d {
        Dialect::Fanuc => &[
            0, 1, 2, 3, 4, 17, 18, 19, 20, 21, 28, 40, 41, 42, 43, 49, 53, 54, 55, 56, 57, 58, 59,
            80, 81, 82, 83, 90, 91, 94, 95, 98, 99,
        ],
        Dialect::Haas => &[
            0, 1, 2, 3, 4, 17, 18, 19, 20, 21, 28, 40, 41, 42, 43, 49, 53, 54, 55, 56, 57, 58, 59,
            80, 81, 82, 83, 90, 91, 94, 95, 98, 99, 187, 234,
        ],
        Dialect::LinuxCnc => &[
            0, 1, 2, 3, 4, 17, 18, 19, 20, 21, 40, 41, 42, 43, 49, 53, 54, 55, 56, 57, 58, 59, 61,
            64, 80, 81, 82, 83, 90, 91, 93, 94, 98, 99,
        ],
        Dialect::Siemens840D => &[
            0, 1, 2, 3, 4, 17, 18, 19, 40, 41, 42, 54, 55, 56, 57, 58, 59, 70, 71, 90, 91, 94, 95,
            500, 64, 60, 450, 451,
        ],
        Dialect::Heidenhain => &[],
    }
}

fn allowed_m(d: Dialect) -> &'static [u32] {
    match d {
        Dialect::Fanuc => &[0, 1, 2, 3, 4, 5, 6, 8, 9, 30, 98, 99],
        Dialect::Haas => &[0, 1, 2, 3, 4, 5, 6, 8, 9, 19, 30, 98, 99],
        Dialect::LinuxCnc => &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 30, 48, 49, 60],
        Dialect::Siemens840D => &[0, 1, 2, 3, 4, 5, 6, 8, 9, 17, 30],
        Dialect::Heidenhain => &[],
    }
}

/// Parse the words of one block. `siemens` accepts `NAME=value` parameters and `CYCLEnn(...)` calls
/// (returned separately).
fn tokenize(line_no: usize, text: &str) -> Result<Vec<Word>, SimError> {
    let mut words = Vec::new();
    let chars: Vec<char> = text.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        if !c.is_ascii_alphabetic() {
            return err(line_no, format!("unexpected character '{c}'"));
        }
        let letter = c.to_ascii_uppercase();
        i += 1;
        // optional '=' (Siemens style I=...)
        if i < chars.len() && chars[i] == '=' {
            i += 1;
        }
        let start = i;
        if i < chars.len() && (chars[i] == '+' || chars[i] == '-') {
            i += 1;
        }
        let mut digits = 0;
        let mut dots = 0;
        while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
            if chars[i] == '.' {
                dots += 1;
            } else {
                digits += 1;
            }
            i += 1;
        }
        if digits == 0 || dots > 1 {
            return err(line_no, format!("the word '{letter}' has no valid number"));
        }
        let s: String = chars[start..i].iter().collect();
        let value: f64 = s
            .parse()
            .or_else(|_| err(line_no, format!("'{s}' is not a number")))?;
        words.push(Word { letter, value });
    }
    Ok(words)
}

fn strip_comments(text: &str, parens: bool) -> String {
    let mut out = String::new();
    let mut depth = 0;
    for c in text.chars() {
        match c {
            ';' if depth == 0 => break,
            '(' if parens => depth += 1,
            ')' if parens && depth > 0 => depth -= 1,
            _ if depth == 0 => out.push(c),
            _ => {}
        }
    }
    out
}

struct State {
    pos: P3,
    abs: bool,
    metric: bool,
    feed: f64,
    rpm: f64,
    spindle_on: bool,
    tool: u32,
    pending_tool: Option<u32>,
    motion: u32,
    plane: u32,
    return_plane_r: bool,
    cycle: Option<Cycle>,
    rot: [f64; 3],
    tcp: bool,
}

#[derive(Clone, Copy)]
struct Cycle {
    kind: u32,
    z: f64,
    r: f64,
    q: f64,
}

pub fn run_gcode(dialect: Dialect, text: &str) -> Result<Run, SimError> {
    let allowed_g = allowed_g(dialect);
    let allowed_m = allowed_m(dialect);
    let mut st = State {
        pos: [0.0; 3],
        abs: true,
        metric: false,
        feed: 0.0,
        rpm: 0.0,
        spindle_on: false,
        tool: 0,
        pending_tool: None,
        motion: 0,
        plane: 17,
        return_plane_r: false,
        cycle: None,
        rot: [0.0; 3],
        tcp: false,
    };
    let mut run = Run::default();
    let mut started = false;
    let mut mcall_cycle: Option<(String, Vec<f64>)> = None;
    let mut saw_units = false;
    for (idx, raw) in text.lines().enumerate() {
        let ln = idx + 1;
        run.lines = ln;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('%') {
            // Fanuc/Haas/LinuxCNC program delimiter; Siemens "%_N_xxx_MPF" header
            if dialect == Dialect::Siemens840D && !line.starts_with("%_N_") {
                return err(ln, "a Siemens program starts with %_N_<name>_MPF");
            }
            started = true;
            continue;
        }
        let mut body = strip_comments(line, dialect != Dialect::Siemens840D);
        let body_trim = body.trim().to_string();
        if body_trim.is_empty() {
            continue;
        }
        if run.program_end_seen {
            return err(ln, "code after the end of the program");
        }
        body = body_trim;
        // a leading block number
        if body.starts_with(['N', 'n']) {
            let end = body[1..]
                .find(|c: char| !c.is_ascii_digit())
                .map_or(body.len(), |i| i + 1);
            if end > 1 {
                body = body[end..].trim().to_string();
                if body.is_empty() {
                    continue;
                }
            }
        }
        // Siemens: CYCLEnn(...) and MCALL
        if dialect == Dialect::Siemens840D {
            let upper = body.to_ascii_uppercase();
            if let Some(rest) = upper.strip_prefix("MCALL") {
                let rest = rest.trim();
                if rest.is_empty() {
                    mcall_cycle = None;
                    continue;
                }
                let (name, args) = parse_siemens_call(ln, rest)?;
                mcall_cycle = Some((name, args));
                continue;
            }
            if upper.starts_with("CYCLE") {
                let (name, args) = parse_siemens_call(ln, &upper)?;
                exec_siemens_cycle(ln, &name, &args, &st.pos.clone(), &mut st, &mut run)?;
                continue;
            }
            if upper.starts_with("TRAORI") {
                st.tcp = true;
                continue;
            }
            if upper.starts_with("TRAFOOF") {
                st.tcp = false;
                continue;
            }
            if let Some(rest) = upper.strip_prefix("T=") {
                // T="NAME" or T=3 : tool preselect by name/number
                let v = rest.trim().trim_matches('"').to_string();
                let n: u32 = v.parse().unwrap_or(1);
                st.pending_tool = Some(n);
                continue;
            }
        }
        // skip block-delete lines
        let body = body.trim_start_matches('/').to_string();
        let mut words = tokenize(ln, &body)?;
        // leading N / O
        words.retain(|w| w.letter != 'N');
        if words.first().map(|w| w.letter) == Some('O') {
            if dialect == Dialect::Siemens840D {
                return err(ln, "O-words do not exist in a Siemens program");
            }
            words.remove(0);
            started = true;
            if words.is_empty() {
                continue;
            }
        }
        let mut new_pos = st.pos;
        let mut new_rot = st.rot;
        let mut have_axis = false;
        let (mut i, mut j, mut r_word, mut q_word, mut p_word) =
            (0.0, 0.0, None::<f64>, None::<f64>, None::<f64>);
        let mut motion_in_block: Option<u32> = None;
        let mut cycle_z: Option<f64> = None;
        let mut gs: Vec<u32> = Vec::new();
        let mut ms: Vec<u32> = Vec::new();
        for w in &words {
            match w.letter {
                'G' => {
                    let g = w.value.round() as u32;
                    let fractional = (w.value - w.value.round()).abs() > 1e-9;
                    if fractional && dialect == Dialect::Fanuc && (w.value - 43.4).abs() < 1e-9 {
                        // Fanuc tool-centre-point control (G43.4 type 1)
                        gs.push(4340);
                        continue;
                    }
                    if fractional && dialect != Dialect::LinuxCnc {
                        return err(ln, format!("G{} is not a valid code", w.value));
                    }
                    if !allowed_g.contains(&g) {
                        return err(ln, format!("G{g} is not defined for this dialect"));
                    }
                    gs.push(g);
                }
                'M' => {
                    let m = w.value.round() as u32;
                    if !allowed_m.contains(&m) {
                        return err(ln, format!("M{m} is not defined for this dialect"));
                    }
                    ms.push(m);
                }
                'A' | 'B' | 'C' => {
                    let a = (w.letter as u8 - b'A') as usize;
                    new_rot[a] = if st.abs { w.value } else { st.rot[a] + w.value };
                    have_axis = true;
                }
                'X' | 'Y' | 'Z' => {
                    let a = (w.letter as u8 - b'X') as usize;
                    let v = if st.metric { w.value } else { w.value * 25.4 };
                    new_pos[a] = if st.abs { v } else { st.pos[a] + v };
                    have_axis = true;
                    if w.letter == 'Z' {
                        cycle_z = Some(v);
                    }
                }
                'I' => i = w.value,
                'J' => j = w.value,
                'K' => {}
                'R' => r_word = Some(w.value),
                'Q' => q_word = Some(w.value),
                'P' => p_word = Some(w.value),
                'F' => st.feed = w.value,
                'S' => st.rpm = w.value,
                'T' => st.pending_tool = Some(w.value as u32),
                'H' | 'D' | 'L' => {}
                other => return err(ln, format!("unknown word '{other}'")),
            }
        }
        // modal groups
        for g in &gs {
            match g {
                0..=3 => motion_in_block = Some(*g),
                80..=83 => {
                    motion_in_block = Some(*g);
                    if *g == 80 {
                        st.cycle = None;
                    }
                }
                17..=19 => st.plane = *g,
                4340 | 234 => st.tcp = true,
                49 => st.tcp = false,
                20 => {
                    st.metric = false;
                    saw_units = true;
                }
                21 | 71 => {
                    st.metric = true;
                    saw_units = true;
                }
                70 => {
                    st.metric = false;
                    saw_units = true;
                }
                90 => st.abs = true,
                91 => st.abs = false,
                98 => st.return_plane_r = false,
                99 => st.return_plane_r = true,
                _ => {}
            }
        }
        if gs.contains(&28) {
            // return to the reference point: not a cutting motion
            continue;
        }
        for m in &ms {
            match m {
                3 | 4 => {
                    if st.rpm <= 0.0 {
                        return err(ln, "the spindle is started without a speed (S word)");
                    }
                    st.spindle_on = true;
                }
                5 => st.spindle_on = false,
                6 => {
                    let t = st
                        .pending_tool
                        .take()
                        .ok_or(())
                        .or_else(|_| err(ln, "M6 without a tool number (T word)"))?;
                    st.tool = t;
                    if !run.tools_used.contains(&t) {
                        run.tools_used.push(t);
                    }
                }
                2 | 30 => {
                    run.program_end_seen = true;
                }
                _ => {}
            }
        }
        if let Some(g) = motion_in_block {
            st.motion = g;
        }
        let motion_active = have_axis || motion_in_block.is_some_and(|g| g != 80);
        if motion_in_block.is_none() && !have_axis {
            continue;
        }
        if !started && dialect != Dialect::LinuxCnc && dialect != Dialect::Siemens840D {
            // Fanuc/Haas programs begin with % and an O number
            return err(ln, "motion before the program header");
        }
        if !motion_active {
            continue;
        }
        if !saw_units && have_axis {
            return err(ln, "motion before units were selected (G20/G21)");
        }
        let target = if have_axis { new_pos } else { st.pos };
        st.rot = new_rot;
        // Siemens modal cycle call
        if dialect == Dialect::Siemens840D {
            if let Some((name, args)) = mcall_cycle.clone() {
                if have_axis && st.motion == 0 {
                    // rapid to hole position then the cycle
                    push(
                        &mut run,
                        MotionKind::Rapid,
                        st.pos,
                        [target[0], target[1], st.pos[2]],
                        None,
                        &st,
                    );
                    st.pos = [target[0], target[1], st.pos[2]];
                    exec_siemens_cycle(ln, &name, &args, &st.pos.clone(), &mut st, &mut run)?;
                    continue;
                }
            }
        }
        match st.motion {
            0 => {
                push(&mut run, MotionKind::Rapid, st.pos, target, None, &st);
                st.pos = target;
            }
            1 => {
                check_cutting(ln, &st)?;
                push(&mut run, MotionKind::Feed, st.pos, target, None, &st);
                st.pos = target;
            }
            2 | 3 => {
                check_cutting(ln, &st)?;
                if st.plane != 17 {
                    return err(ln, "arcs are only supported in the XY plane (G17)");
                }
                if !have_axis {
                    return err(ln, "an arc needs an end point");
                }
                let (cx, cy) = if let Some(r) = r_word {
                    let (dx, dy) = (target[0] - st.pos[0], target[1] - st.pos[1]);
                    let d = dx.hypot(dy);
                    if d < 1e-12 || r.abs() * 2.0 + 1e-9 < d {
                        return err(ln, "arc radius is smaller than half the chord");
                    }
                    let h = (r * r - d * d / 4.0).max(0.0).sqrt();
                    let (mx, my) = (st.pos[0] + dx / 2.0, st.pos[1] + dy / 2.0);
                    // centre on the left of the chord for ccw with positive R, mirrored otherwise
                    let sign = if (st.motion == 3) == (r > 0.0) {
                        1.0
                    } else {
                        -1.0
                    };
                    (mx - sign * h * dy / d, my + sign * h * dx / d)
                } else {
                    let s = if st.metric { 1.0 } else { 25.4 };
                    (st.pos[0] + i * s, st.pos[1] + j * s)
                };
                let (r0, r1) = (
                    (st.pos[0] - cx).hypot(st.pos[1] - cy),
                    (target[0] - cx).hypot(target[1] - cy),
                );
                if (r0 - r1).abs() > 2e-3 {
                    return err(
                        ln,
                        format!("arc end point is not on the circle (radius {r0:.4} at the start, {r1:.4} at the end)"),
                    );
                }
                let kind = if st.motion == 2 {
                    MotionKind::ArcCw
                } else {
                    MotionKind::ArcCcw
                };
                push(&mut run, kind, st.pos, target, Some([cx, cy]), &st);
                st.pos = target;
            }
            4 => {}
            81..=83 => {
                let s = if st.metric { 1.0 } else { 25.4 };
                let z = cycle_z
                    .or(st.cycle.map(|c| c.z))
                    .ok_or(())
                    .or_else(|_| err(ln, "a drilling cycle needs a depth (Z word)"))?;
                let r = r_word
                    .map(|r| r * s)
                    .or(st.cycle.map(|c| c.r))
                    .ok_or(())
                    .or_else(|_| err(ln, "a drilling cycle needs a retract plane (R word)"))?;
                let q = q_word
                    .map(|q| q * s)
                    .or(st.cycle.map(|c| c.q))
                    .unwrap_or(0.0);
                if st.motion == 83 && q <= 0.0 {
                    return err(ln, "G83 needs a positive peck depth (Q word)");
                }
                if st.motion == 82 && p_word.is_none() && st.cycle.is_none_or(|c| c.kind != 82) {
                    return err(ln, "G82 needs a dwell (P word)");
                }
                st.cycle = Some(Cycle {
                    kind: st.motion,
                    z,
                    r,
                    q,
                });
                if have_axis {
                    drill(ln, &mut st, &mut run, target[0], target[1], z, r, q)?;
                }
            }
            g => return err(ln, format!("G{g} cannot be executed here")),
        }
    }
    if !run.program_end_seen {
        return err(run.lines.max(1), "the program does not end with M30 or M2");
    }
    Ok(run)
}

fn check_cutting(ln: usize, st: &State) -> Result<(), SimError> {
    if st.feed <= 0.0 {
        return err(ln, "a cutting move without a feed rate (F word)");
    }
    if !st.spindle_on {
        return err(ln, "a cutting move with the spindle off");
    }
    if st.tool == 0 {
        return err(ln, "a cutting move before any tool was selected");
    }
    Ok(())
}

fn push(run: &mut Run, kind: MotionKind, from: P3, to: P3, centre: Option<[f64; 2]>, st: &State) {
    run.motions.push(Motion {
        kind,
        from,
        to,
        centre,
        feed: st.feed,
        tool: st.tool,
        spindle_rpm: if st.spindle_on { st.rpm } else { 0.0 },
        line: run.lines,
        rot: st.rot,
        tcp: st.tcp,
    });
}

fn drill(
    ln: usize,
    st: &mut State,
    run: &mut Run,
    x: f64,
    y: f64,
    z: f64,
    r: f64,
    q: f64,
) -> Result<(), SimError> {
    check_cutting(ln, st)?;
    let initial = st.pos[2];
    // rapid in XY, rapid to the R plane, feed to depth (pecking if G83), rapid out
    push(run, MotionKind::Rapid, st.pos, [x, y, initial], None, st);
    push(run, MotionKind::Rapid, [x, y, initial], [x, y, r], None, st);
    if st.motion == 83 && q > 0.0 {
        let mut cur = r;
        while cur > z + 1e-9 {
            let next = (cur - q).max(z);
            push(run, MotionKind::Feed, [x, y, cur], [x, y, next], None, st);
            push(run, MotionKind::Rapid, [x, y, next], [x, y, r], None, st);
            if next > z + 1e-9 {
                push(run, MotionKind::Rapid, [x, y, r], [x, y, next], None, st);
            }
            cur = next;
        }
    } else {
        push(run, MotionKind::Feed, [x, y, r], [x, y, z], None, st);
        push(run, MotionKind::Rapid, [x, y, z], [x, y, r], None, st);
    }
    let end_z = if st.return_plane_r { r } else { initial };
    if end_z != r {
        push(run, MotionKind::Rapid, [x, y, r], [x, y, end_z], None, st);
    }
    st.pos = [x, y, end_z];
    run.drills.push((x, y, z, r));
    Ok(())
}

fn parse_siemens_call(ln: usize, text: &str) -> Result<(String, Vec<f64>), SimError> {
    let open = text
        .find('(')
        .ok_or(())
        .or_else(|_| err(ln, "a cycle call needs parentheses"))?;
    let close = text
        .rfind(')')
        .ok_or(())
        .or_else(|_| err(ln, "unbalanced parentheses in a cycle call"))?;
    let name = text[..open].trim().to_string();
    let args: Result<Vec<f64>, SimError> = text[open + 1..close]
        .split(',')
        .map(|a| {
            let a = a.trim();
            if a.is_empty() {
                Ok(0.0)
            } else {
                a.parse::<f64>()
                    .or_else(|_| err(ln, format!("'{a}' is not a number")))
            }
        })
        .collect();
    Ok((name, args?))
}

/// CYCLE81(RTP, RFP, SDIS, DP, DPR) and CYCLE83 (… FDEP …) executed at the current XY position.
fn exec_siemens_cycle(
    ln: usize,
    name: &str,
    args: &[f64],
    at: &P3,
    st: &mut State,
    run: &mut Run,
) -> Result<(), SimError> {
    if name != "CYCLE81" && name != "CYCLE82" && name != "CYCLE83" {
        return err(ln, format!("unknown cycle {name}"));
    }
    if args.len() < 5 {
        return err(ln, format!("{name} needs at least 5 parameters"));
    }
    let (rtp, rfp, sdis, dp) = (args[0], args[1], args[2], args[3]);
    check_cutting(ln, st)?;
    let top = rfp + sdis;
    push(
        run,
        MotionKind::Rapid,
        *at,
        [at[0], at[1], rtp.max(top)],
        None,
        st,
    );
    push(
        run,
        MotionKind::Rapid,
        [at[0], at[1], rtp.max(top)],
        [at[0], at[1], top],
        None,
        st,
    );
    push(
        run,
        MotionKind::Feed,
        [at[0], at[1], top],
        [at[0], at[1], dp],
        None,
        st,
    );
    push(
        run,
        MotionKind::Rapid,
        [at[0], at[1], dp],
        [at[0], at[1], rtp],
        None,
        st,
    );
    st.pos = [at[0], at[1], rtp];
    run.drills.push((at[0], at[1], dp, top));
    Ok(())
}
