//! IGES fixed-format reader: Start / Global / Directory / Parameter / Terminate sections.

use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
pub enum IgesParseError {
    BadLine { line: usize },
    BadDirectory { line: usize },
    BadParameter { de: usize },
    TooLarge,
}

impl std::fmt::Display for IgesParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for IgesParseError {}

#[derive(Debug, Clone)]
pub struct Entity {
    /// Directory entry sequence number (odd line number of its first directory line).
    pub de: usize,
    pub ty: u32,
    pub form: i32,
    /// Pointer (DE number) of the transformation matrix, 0 if none.
    pub transform: usize,
    pub status: String,
    pub params: Vec<Param>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Param {
    Int(i64),
    Real(f64),
    Str(String),
    Null,
}

impl Param {
    pub fn f(&self) -> f64 {
        match self {
            Param::Int(i) => *i as f64,
            Param::Real(r) => *r,
            _ => 0.0,
        }
    }
    pub fn i(&self) -> i64 {
        match self {
            Param::Int(i) => *i,
            Param::Real(r) => *r as i64,
            _ => 0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct IgesFile {
    /// Length unit in millimetres per file unit.
    pub unit_mm: f64,
    pub entities: HashMap<usize, Entity>,
    /// DE numbers in file order.
    pub order: Vec<usize>,
}

impl IgesFile {
    pub fn get(&self, de: usize) -> Option<&Entity> {
        self.entities.get(&de)
    }
}

fn fld(l: &str, k: usize) -> &str {
    l[8 * k..8 * k + 8].trim()
}

fn section(line: &str) -> Option<char> {
    line.as_bytes().get(72).map(|b| *b as char)
}

/// Split a parameter record into fields honouring Hollerith strings (`nHtext`).
fn fields(text: &str, delim: char, term: char) -> Vec<String> {
    let mut out = Vec::new();
    let b: Vec<char> = text.chars().collect();
    let mut i = 0;
    let mut cur = String::new();
    while i < b.len() {
        let c = b[i];
        if c == delim || c == term {
            out.push(std::mem::take(&mut cur));
            if c == term {
                return out;
            }
            i += 1;
            continue;
        }
        // Hollerith constant: digits followed by H
        if c.is_ascii_digit() && cur.trim().is_empty() {
            let mut j = i;
            while j < b.len() && b[j].is_ascii_digit() {
                j += 1;
            }
            if j < b.len() && b[j] == 'H' {
                let n: usize = b[i..j].iter().collect::<String>().parse().unwrap_or(0);
                let start = j + 1;
                let end = (start + n).min(b.len());
                cur = format!("\u{1}{}", b[start..end].iter().collect::<String>());
                i = end;
                continue;
            }
        }
        cur.push(c);
        i += 1;
    }
    if !cur.trim().is_empty() {
        out.push(cur);
    }
    out
}

fn to_param(s: &str) -> Param {
    if let Some(t) = s.strip_prefix('\u{1}') {
        return Param::Str(t.to_string());
    }
    let t = s.trim();
    if t.is_empty() {
        return Param::Null;
    }
    // Fortran style exponents: 1.0D+00
    let norm = t.replace(['D', 'd'], "E");
    if let Ok(i) = norm.parse::<i64>() {
        return Param::Int(i);
    }
    let fixed = if norm.starts_with('.') || norm.starts_with("-.") || norm.starts_with("+.") {
        norm.replacen('.', "0.", 1)
    } else {
        norm
    };
    let fixed = if fixed.ends_with('.') {
        format!("{fixed}0")
    } else {
        fixed.replace(".E", ".0E")
    };
    fixed
        .parse::<f64>()
        .ok()
        .filter(|x| x.is_finite())
        .map(Param::Real)
        .unwrap_or(Param::Null)
}

pub fn parse(text: &str) -> Result<IgesFile, IgesParseError> {
    if text.len() > 1 << 30 {
        return Err(IgesParseError::TooLarge);
    }
    let mut global = String::new();
    let mut dir_lines: Vec<(usize, String)> = Vec::new();
    let mut pd: HashMap<usize, String> = HashMap::new();
    for (n, raw) in text.lines().enumerate() {
        if !raw.is_ascii() {
            return Err(IgesParseError::BadLine { line: n + 1 });
        }
        let line: String = format!("{raw:<80}").chars().take(80).collect();
        match section(&line) {
            Some('S') | Some('T') => {}
            Some('G') => global.push_str(line[..72].trim_end()),
            Some('D') => dir_lines.push((n, line)),
            Some('P') => {
                let de: usize = line[64..72]
                    .trim()
                    .parse()
                    .map_err(|_| IgesParseError::BadLine { line: n + 1 })?;
                let seq: usize = line[73..80]
                    .trim()
                    .parse()
                    .map_err(|_| IgesParseError::BadLine { line: n + 1 })?;
                pd.insert(seq, format!("{}\u{2}{de}", &line[..64]));
            }
            _ => {
                if !raw.trim().is_empty() {
                    return Err(IgesParseError::BadLine { line: n + 1 });
                }
            }
        }
    }
    // ---- global: delimiters and units
    let (mut delim, mut term) = (',', ';');
    let g = global.as_str();
    let gstart = if let Some(rest) = g.strip_prefix("1H") {
        delim = rest.chars().next().unwrap_or(',');
        &rest[delim.len_utf8()..]
    } else {
        g
    };
    // the second field may redefine the record delimiter ("1H;")
    // `gstart` now begins with the delimiter that ended parameter 1; parameter 2 is the record delimiter
    let gf = fields(
        gstart.strip_prefix(delim).unwrap_or(gstart),
        delim,
        '\u{7f}',
    );
    if let Some(Param::Str(s)) = gf.first().map(|x| to_param(x)) {
        if let Some(c) = s.chars().next() {
            term = c;
        }
    }
    let _ = term;
    let unit_flag = gf.get(12).map(|x| to_param(x).i()).unwrap_or(2);
    let unit_mm = match unit_flag {
        1 => 25.4,
        2 => 1.0,
        4 => 304.8,
        5 => 1_609_344.0,
        6 => 1000.0,
        7 => 1_000_000.0,
        8 => 0.0254,
        9 => 0.001,
        10 => 10.0,
        11 => 0.0000254,
        _ => 1.0,
    };
    // ---- directory + parameter data
    let mut file = IgesFile {
        unit_mm,
        ..Default::default()
    };
    if !dir_lines.len().is_multiple_of(2) {
        return Err(IgesParseError::BadDirectory {
            line: dir_lines.last().map(|l| l.0 + 1).unwrap_or(0),
        });
    }
    for pair in dir_lines.chunks(2) {
        let (n1, l1) = (&pair[0].0, &pair[0].1);
        let l2 = &pair[1].1;
        let de: usize = n1 + 1; // sequence = line number within the D section is what matters; recomputed below
        let _ = de;
        let seq: usize = l1[73..80]
            .trim()
            .parse()
            .map_err(|_| IgesParseError::BadDirectory { line: n1 + 1 })?;
        let ty: u32 = fld(l1, 0)
            .parse()
            .map_err(|_| IgesParseError::BadDirectory { line: n1 + 1 })?;
        let pd_ptr: usize = fld(l1, 1)
            .parse()
            .map_err(|_| IgesParseError::BadDirectory { line: n1 + 1 })?;
        let transform: usize = fld(l1, 6).parse().unwrap_or(0);
        let form: i32 = fld(l2, 4).parse().unwrap_or(0);
        let count: usize = fld(l2, 3).parse().unwrap_or(1);
        let mut rec = String::new();
        for k in 0..count {
            let Some(row) = pd.get(&(pd_ptr + k)) else {
                return Err(IgesParseError::BadParameter { de: seq });
            };
            rec.push_str(row.split('\u{2}').next().unwrap_or(""));
        }
        let parts = fields(rec.trim_end(), delim, ';');
        // first field is the entity type again
        let params: Vec<Param> = parts.iter().skip(1).map(|s| to_param(s)).collect();
        file.order.push(seq);
        file.entities.insert(
            seq,
            Entity {
                de: seq,
                ty,
                form,
                transform,
                status: fld(l1, 8).to_string(),
                params,
            },
        );
    }
    Ok(file)
}
