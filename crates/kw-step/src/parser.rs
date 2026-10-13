//! Part 21 lexer / parser with resource limits. Never panics on malformed input.

use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
pub enum Param {
    /// `$`
    Null,
    /// `*`
    Derived,
    Int(i64),
    Real(f64),
    Str(String),
    /// `.NAME.`
    Enum(String),
    /// `#12`
    Ref(u32),
    List(Vec<Param>),
    /// `LABEL(…)` inside a parameter list
    Typed(String, Vec<Param>),
}

impl Param {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Param::Real(x) => Some(*x),
            Param::Int(i) => Some(*i as f64),
            Param::Typed(_, v) if v.len() == 1 => v[0].as_f64(),
            _ => None,
        }
    }
    pub fn as_usize(&self) -> Option<usize> {
        match self {
            Param::Int(i) if *i >= 0 => Some(*i as usize),
            Param::Real(x) if *x >= 0.0 && x.fract() == 0.0 => Some(*x as usize),
            _ => None,
        }
    }
    pub fn as_ref_id(&self) -> Option<u32> {
        match self {
            Param::Ref(r) => Some(*r),
            _ => None,
        }
    }
    pub fn as_list(&self) -> Option<&[Param]> {
        match self {
            Param::List(l) => Some(l),
            _ => None,
        }
    }
    pub fn as_enum(&self) -> Option<&str> {
        match self {
            Param::Enum(s) => Some(s),
            _ => None,
        }
    }
    pub fn as_bool(&self) -> Option<bool> {
        match self.as_enum()? {
            "T" => Some(true),
            "F" => Some(false),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Param::Str(s) => Some(s),
            _ => None,
        }
    }
}

/// One entity instance. Simple instances have one `(type, params)` part; complex ones several.
#[derive(Debug, Clone, PartialEq)]
pub struct Instance {
    pub id: u32,
    pub parts: Vec<(String, Vec<Param>)>,
}

impl Instance {
    pub fn kind(&self) -> &str {
        &self.parts[0].0
    }
    pub fn part(&self, name: &str) -> Option<&[Param]> {
        self.parts
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, p)| p.as_slice())
    }
    pub fn is(&self, name: &str) -> bool {
        self.parts.iter().any(|(n, _)| n == name)
    }
    /// Params of a simple instance.
    pub fn params(&self) -> &[Param] {
        &self.parts[0].1
    }
}

#[derive(Debug, Clone, Default)]
pub struct StepFile {
    pub header: Vec<(String, Vec<Param>)>,
    pub instances: HashMap<u32, Instance>,
    /// Instance ids in file order.
    pub order: Vec<u32>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ParseError {
    Syntax { offset: usize, what: &'static str },
    TooLarge(&'static str),
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for ParseError {}

#[derive(Debug, Clone)]
pub struct Limits {
    pub max_bytes: usize,
    pub max_instances: usize,
    pub max_depth: usize,
    pub max_list: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_bytes: 512 << 20,
            max_instances: 20_000_000,
            max_depth: 64,
            max_list: 50_000_000,
        }
    }
}

struct P<'a> {
    b: &'a [u8],
    i: usize,
    lim: &'a Limits,
}

type R<T> = Result<T, ParseError>;

impl P<'_> {
    fn err<T>(&self, what: &'static str) -> R<T> {
        Err(ParseError::Syntax {
            offset: self.i,
            what,
        })
    }

    fn skip_ws(&mut self) -> R<()> {
        loop {
            while self.i < self.b.len() && self.b[self.i].is_ascii_whitespace() {
                self.i += 1;
            }
            if self.b[self.i..].starts_with(b"/*") {
                match self.b[self.i + 2..].windows(2).position(|w| w == b"*/") {
                    Some(p) => self.i += p + 4,
                    None => return self.err("unterminated comment"),
                }
            } else {
                return Ok(());
            }
        }
    }

    fn peek(&self) -> Option<u8> {
        self.b.get(self.i).copied()
    }

    fn eat(&mut self, c: u8) -> R<()> {
        self.skip_ws()?;
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            self.err("unexpected character")
        }
    }

    fn keyword(&mut self) -> R<String> {
        self.skip_ws()?;
        let s = self.i;
        while self.i < self.b.len()
            && (self.b[self.i].is_ascii_alphanumeric()
                || self.b[self.i] == b'_'
                || self.b[self.i] == b'-')
        {
            self.i += 1;
        }
        if s == self.i {
            return self.err("expected a keyword");
        }
        Ok(String::from_utf8_lossy(&self.b[s..self.i]).to_ascii_uppercase())
    }

    fn param(&mut self, depth: usize) -> R<Param> {
        if depth > self.lim.max_depth {
            return Err(ParseError::TooLarge("parameter nesting"));
        }
        self.skip_ws()?;
        match self.peek() {
            None => self.err("unexpected end of input"),
            Some(b'$') => {
                self.i += 1;
                Ok(Param::Null)
            }
            Some(b'*') => {
                self.i += 1;
                Ok(Param::Derived)
            }
            Some(b'#') => {
                self.i += 1;
                let s = self.i;
                while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                    self.i += 1;
                }
                if s == self.i {
                    return self.err("bad reference");
                }
                std::str::from_utf8(&self.b[s..self.i])
                    .ok()
                    .and_then(|t| t.parse().ok())
                    .map(Param::Ref)
                    .ok_or(ParseError::Syntax {
                        offset: s,
                        what: "reference out of range",
                    })
            }
            Some(b'\'') => {
                self.i += 1;
                let mut out = Vec::new();
                loop {
                    match self.peek() {
                        None => return self.err("unterminated string"),
                        Some(b'\'') => {
                            if self.b.get(self.i + 1) == Some(&b'\'') {
                                out.push(b'\'');
                                self.i += 2;
                            } else {
                                self.i += 1;
                                break;
                            }
                        }
                        Some(c) => {
                            out.push(c);
                            self.i += 1;
                        }
                    }
                }
                Ok(Param::Str(String::from_utf8_lossy(&out).into_owned()))
            }
            Some(b'.') => {
                self.i += 1;
                let s = self.i;
                while self
                    .peek()
                    .is_some_and(|c| c.is_ascii_alphanumeric() || c == b'_')
                {
                    self.i += 1;
                }
                if self.peek() != Some(b'.') {
                    return self.err("unterminated enumeration");
                }
                let name = String::from_utf8_lossy(&self.b[s..self.i]).to_ascii_uppercase();
                self.i += 1;
                Ok(Param::Enum(name))
            }
            Some(b'(') => {
                self.i += 1;
                let mut items = Vec::new();
                self.skip_ws()?;
                if self.peek() == Some(b')') {
                    self.i += 1;
                    return Ok(Param::List(items));
                }
                loop {
                    items.push(self.param(depth + 1)?);
                    if items.len() > self.lim.max_list {
                        return Err(ParseError::TooLarge("list length"));
                    }
                    self.skip_ws()?;
                    match self.peek() {
                        Some(b',') => self.i += 1,
                        Some(b')') => {
                            self.i += 1;
                            break;
                        }
                        _ => return self.err("expected ',' or ')'"),
                    }
                }
                Ok(Param::List(items))
            }
            Some(c) if c == b'-' || c == b'+' || c.is_ascii_digit() => self.number(),
            Some(c) if c.is_ascii_alphabetic() => {
                let name = self.keyword()?;
                self.skip_ws()?;
                if self.peek() == Some(b'(') {
                    match self.param(depth + 1)? {
                        Param::List(l) => Ok(Param::Typed(name, l)),
                        _ => self.err("bad typed parameter"),
                    }
                } else {
                    self.err("bare keyword in parameter list")
                }
            }
            Some(b'"') => {
                // binary literal "0ABC…": skip
                self.i += 1;
                while self.peek().is_some_and(|c| c != b'"') {
                    self.i += 1;
                }
                self.i += 1;
                Ok(Param::Null)
            }
            _ => self.err("unexpected character in parameter"),
        }
    }

    fn number(&mut self) -> R<Param> {
        let s = self.i;
        if matches!(self.peek(), Some(b'-' | b'+')) {
            self.i += 1;
        }
        while self.peek().is_some_and(|c| c.is_ascii_digit()) {
            self.i += 1;
        }
        let mut real = false;
        if self.peek() == Some(b'.') {
            real = true;
            self.i += 1;
            while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                self.i += 1;
            }
        }
        if matches!(self.peek(), Some(b'E' | b'e')) {
            real = true;
            self.i += 1;
            if matches!(self.peek(), Some(b'-' | b'+')) {
                self.i += 1;
            }
            while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                self.i += 1;
            }
        }
        let text = std::str::from_utf8(&self.b[s..self.i]).map_err(|_| ParseError::Syntax {
            offset: s,
            what: "bad number",
        })?;
        if real {
            let t = if text.ends_with('.') {
                format!("{text}0")
            } else {
                text.to_string()
            };
            let t = t.replace(".E", ".0E").replace(".e", ".0e");
            t.parse::<f64>()
                .ok()
                .filter(|x| x.is_finite())
                .map(Param::Real)
                .ok_or(ParseError::Syntax {
                    offset: s,
                    what: "bad real",
                })
        } else {
            text.parse::<i64>()
                .map(Param::Int)
                .map_err(|_| ParseError::Syntax {
                    offset: s,
                    what: "bad integer",
                })
        }
    }

    fn typed_params(&mut self) -> R<(String, Vec<Param>)> {
        let name = self.keyword()?;
        self.eat(b'(')?;
        self.i -= 1;
        match self.param(0)? {
            Param::List(l) => Ok((name, l)),
            _ => self.err("expected a parameter list"),
        }
    }
}

/// Parse a complete Part 21 exchange structure.
pub fn parse(text: &str) -> Result<StepFile, ParseError> {
    parse_with(text, &Limits::default())
}

pub fn parse_with(text: &str, lim: &Limits) -> Result<StepFile, ParseError> {
    if text.len() > lim.max_bytes {
        return Err(ParseError::TooLarge("file size"));
    }
    let mut p = P {
        b: text.as_bytes(),
        i: 0,
        lim,
    };
    p.skip_ws()?;
    if !p.b[p.i..].starts_with(b"ISO-10303-21") {
        return p.err("missing ISO-10303-21 header");
    }
    p.i += "ISO-10303-21".len();
    p.eat(b';')?;
    let mut file = StepFile::default();
    // HEADER
    if p.keyword()? != "HEADER" {
        return p.err("expected HEADER section");
    }
    p.eat(b';')?;
    loop {
        p.skip_ws()?;
        let save = p.i;
        let kw = p.keyword()?;
        if kw == "ENDSEC" {
            p.eat(b';')?;
            break;
        }
        p.i = save;
        let (n, params) = p.typed_params()?;
        p.eat(b';')?;
        file.header.push((n, params));
    }
    // DATA (possibly several sections; all merged)
    loop {
        p.skip_ws()?;
        if p.i >= p.b.len() {
            return p.err("missing END-ISO-10303-21");
        }
        let kw = p.keyword()?;
        match kw.as_str() {
            "END-ISO-10303-21" => break,
            "DATA" => {
                // optional section parameters
                p.skip_ws()?;
                if p.peek() == Some(b'(') {
                    p.param(0)?;
                }
                p.eat(b';')?;
                loop {
                    p.skip_ws()?;
                    if p.peek() == Some(b'#') {
                        p.i += 1;
                        let s = p.i;
                        while p.peek().is_some_and(|c| c.is_ascii_digit()) {
                            p.i += 1;
                        }
                        let id: u32 = std::str::from_utf8(&p.b[s..p.i])
                            .ok()
                            .and_then(|t| t.parse().ok())
                            .ok_or(ParseError::Syntax {
                                offset: s,
                                what: "bad instance id",
                            })?;
                        p.eat(b'=')?;
                        p.skip_ws()?;
                        let parts = if p.peek() == Some(b'(') {
                            // complex entity: ( TYPE1(...) TYPE2(...) )
                            p.i += 1;
                            let mut parts = Vec::new();
                            loop {
                                p.skip_ws()?;
                                if p.peek() == Some(b')') {
                                    p.i += 1;
                                    break;
                                }
                                parts.push(p.typed_params()?);
                                if parts.len() > 64 {
                                    return Err(ParseError::TooLarge("complex entity parts"));
                                }
                            }
                            parts
                        } else {
                            vec![p.typed_params()?]
                        };
                        p.eat(b';')?;
                        if file.instances.len() >= lim.max_instances {
                            return Err(ParseError::TooLarge("instance count"));
                        }
                        if parts.is_empty() {
                            return p.err("empty entity");
                        }
                        file.order.push(id);
                        file.instances.insert(id, Instance { id, parts });
                    } else {
                        let kw = p.keyword()?;
                        if kw == "ENDSEC" {
                            p.eat(b';')?;
                            break;
                        }
                        return p.err("expected an instance or ENDSEC");
                    }
                }
            }
            _ => return p.err("unexpected section"),
        }
    }
    Ok(file)
}
