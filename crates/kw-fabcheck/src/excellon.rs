use crate::{fail, FabError, P2};
use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub struct Hole {
    pub pos: P2,
    pub diameter: f64,
}

pub fn parse_excellon(text: &str) -> Result<Vec<Hole>, FabError> {
    let mut tools: BTreeMap<u32, f64> = BTreeMap::new();
    let mut holes = Vec::new();
    let mut in_header = false;
    let mut seen_header = false;
    let mut metric = false;
    let mut current: Option<u32> = None;
    let mut ended = false;
    for (i, raw) in text.lines().enumerate() {
        let ln = i + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with(';') {
            continue;
        }
        if ended {
            return fail(ln, "content after M30");
        }
        if line == "M48" {
            in_header = true;
            seen_header = true;
            continue;
        }
        if !seen_header {
            return fail(ln, "a drill file starts with M48");
        }
        if in_header {
            if line == "%" {
                in_header = false;
                continue;
            }
            if line.starts_with("METRIC") {
                metric = true;
            } else if line.starts_with("FMAT") || line.starts_with("INCH") {
                if line.starts_with("INCH") {
                    return fail(ln, "inch drill files are not accepted");
                }
            } else if let Some(rest) = line.strip_prefix('T') {
                let (n, d) = rest
                    .split_once('C')
                    .ok_or(())
                    .or_else(|_| fail(ln, "tool definition is Tn Cd"))?;
                let n: u32 = n.parse().or_else(|_| fail(ln, "tool number"))?;
                let d: f64 = d.parse().or_else(|_| fail(ln, "tool diameter"))?;
                if d <= 0.0 {
                    return fail(ln, "tool diameter must be positive");
                }
                tools.insert(n, d);
            } else {
                return fail(ln, format!("unknown header line '{line}'"));
            }
            continue;
        }
        if !metric {
            return fail(ln, "units were not declared (METRIC)");
        }
        match line {
            "G90" | "G05" | "T0" => continue,
            "M30" => {
                ended = true;
                continue;
            }
            _ => {}
        }
        if let Some(t) = line.strip_prefix('T') {
            let n: u32 = t.parse().or_else(|_| fail(ln, "tool select"))?;
            if !tools.contains_key(&n) {
                return fail(ln, format!("tool T{n} is not defined"));
            }
            current = Some(n);
            continue;
        }
        if let Some(rest) = line.strip_prefix('X') {
            let (x, y) = rest
                .split_once('Y')
                .ok_or(())
                .or_else(|_| fail(ln, "coordinates are XnYn"))?;
            let x: f64 = x.parse().or_else(|_| fail(ln, "x coordinate"))?;
            let y: f64 = y.parse().or_else(|_| fail(ln, "y coordinate"))?;
            let t = current
                .ok_or(())
                .or_else(|_| fail(ln, "a hole before any tool was selected"))?;
            holes.push(Hole {
                pos: [x, y],
                diameter: tools[&t],
            });
            continue;
        }
        return fail(ln, format!("unknown command '{line}'"));
    }
    if !ended {
        return fail(
            text.lines().count().max(1),
            "the drill file does not end with M30",
        );
    }
    Ok(holes)
}
