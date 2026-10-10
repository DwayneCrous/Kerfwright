//! Lexer and recursive-descent parser.

use crate::units::{unit, Dim};
use crate::Quantity;
use std::collections::BTreeSet;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
    Pow,
    Lt,
    Gt,
    Le,
    Ge,
    Eq,
    Ne,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Ast {
    Lit(Quantity),
    Var(String),
    Neg(Box<Ast>),
    Bin(BinOp, Box<Ast>, Box<Ast>),
    Call(String, Vec<Ast>),
}

impl Ast {
    pub(crate) fn collect_vars(&self, out: &mut BTreeSet<String>) {
        match self {
            Ast::Lit(_) => {}
            Ast::Var(v) => {
                out.insert(v.clone());
            }
            Ast::Neg(a) => a.collect_vars(out),
            Ast::Bin(_, a, b) => {
                a.collect_vars(out);
                b.collect_vars(out);
            }
            Ast::Call(_, args) => args.iter().for_each(|a| a.collect_vars(out)),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub message: String,
    /// Byte offset into the source.
    pub at: usize,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} (at position {})", self.message, self.at)
    }
}
impl std::error::Error for ParseError {}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Ident(String),
    Op(char),
    Le,
    Ge,
    EqEq,
    Ne,
    LParen,
    RParen,
    Comma,
}

/// Names that cannot be used as variables: units and constants.
pub fn is_reserved(name: &str) -> bool {
    unit(name).is_some() || name == "pi" || name == "e" || name == "tau"
}

fn lex(src: &str) -> Result<Vec<(Tok, usize)>, ParseError> {
    let b = src.as_bytes();
    let mut i = 0;
    let mut out = Vec::new();
    while i < b.len() {
        let c = b[i] as char;
        let start = i;
        if c.is_whitespace() {
            i += 1;
        } else if c.is_ascii_digit() || (c == '.' && i + 1 < b.len() && b[i + 1].is_ascii_digit()) {
            while i < b.len() && (b[i].is_ascii_digit() || b[i] == b'.') {
                i += 1;
            }
            // exponent: e/E followed by digits (optionally signed)
            if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
                let mut j = i + 1;
                if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
                    j += 1;
                }
                if j < b.len() && b[j].is_ascii_digit() {
                    while j < b.len() && b[j].is_ascii_digit() {
                        j += 1;
                    }
                    i = j;
                }
            }
            let text = &src[start..i];
            let v: f64 = text.parse().map_err(|_| ParseError {
                message: format!("bad number `{text}`"),
                at: start,
            })?;
            if !v.is_finite() {
                return Err(ParseError {
                    message: format!("number `{text}` is out of range"),
                    at: start,
                });
            }
            out.push((Tok::Num(v), start));
        } else if c.is_ascii_alphabetic() || c == '_' {
            while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
                i += 1;
            }
            out.push((Tok::Ident(src[start..i].to_string()), start));
        } else {
            i += 1;
            let two = i < b.len() && b[i] == b'=';
            let tok = match c {
                '(' => Tok::LParen,
                ')' => Tok::RParen,
                ',' => Tok::Comma,
                '+' | '-' | '*' | '/' | '^' => Tok::Op(c),
                '<' if two => {
                    i += 1;
                    Tok::Le
                }
                '>' if two => {
                    i += 1;
                    Tok::Ge
                }
                '=' if two => {
                    i += 1;
                    Tok::EqEq
                }
                '!' if two => {
                    i += 1;
                    Tok::Ne
                }
                '<' | '>' => Tok::Op(c),
                _ => {
                    return Err(ParseError {
                        message: format!(
                            "unexpected character `{}`",
                            src[start..].chars().next().unwrap_or(c)
                        ),
                        at: start,
                    })
                }
            };
            out.push((tok, start));
        }
    }
    Ok(out)
}

struct Parser {
    toks: Vec<(Tok, usize)>,
    pos: usize,
    end: usize,
    depth: usize,
}

const MAX_DEPTH: usize = 128;

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos).map(|t| &t.0)
    }
    fn here(&self) -> usize {
        self.toks.get(self.pos).map_or(self.end, |t| t.1)
    }
    fn err<T>(&self, m: impl Into<String>) -> Result<T, ParseError> {
        Err(ParseError {
            message: m.into(),
            at: self.here(),
        })
    }
    fn enter(&mut self) -> Result<(), ParseError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return self.err("expression is nested too deeply");
        }
        Ok(())
    }

    fn comparison(&mut self) -> Result<Ast, ParseError> {
        let mut l = self.additive()?;
        loop {
            let op = match self.peek() {
                Some(Tok::Op('<')) => BinOp::Lt,
                Some(Tok::Op('>')) => BinOp::Gt,
                Some(Tok::Le) => BinOp::Le,
                Some(Tok::Ge) => BinOp::Ge,
                Some(Tok::EqEq) => BinOp::Eq,
                Some(Tok::Ne) => BinOp::Ne,
                _ => return Ok(l),
            };
            self.pos += 1;
            let r = self.additive()?;
            l = Ast::Bin(op, Box::new(l), Box::new(r));
        }
    }

    fn additive(&mut self) -> Result<Ast, ParseError> {
        let mut l = self.term()?;
        loop {
            let op = match self.peek() {
                Some(Tok::Op('+')) => BinOp::Add,
                Some(Tok::Op('-')) => BinOp::Sub,
                _ => return Ok(l),
            };
            self.pos += 1;
            let r = self.term()?;
            l = Ast::Bin(op, Box::new(l), Box::new(r));
        }
    }

    fn term(&mut self) -> Result<Ast, ParseError> {
        let mut l = self.unary()?;
        loop {
            let op = match self.peek() {
                Some(Tok::Op('*')) => BinOp::Mul,
                Some(Tok::Op('/')) => BinOp::Div,
                _ => return Ok(l),
            };
            self.pos += 1;
            let r = self.unary()?;
            l = Ast::Bin(op, Box::new(l), Box::new(r));
        }
    }

    fn unary(&mut self) -> Result<Ast, ParseError> {
        match self.peek() {
            Some(Tok::Op('-')) => {
                self.pos += 1;
                self.enter()?;
                let a = self.unary()?;
                self.depth -= 1;
                Ok(Ast::Neg(Box::new(a)))
            }
            Some(Tok::Op('+')) => {
                self.pos += 1;
                self.enter()?;
                let a = self.unary()?;
                self.depth -= 1;
                Ok(a)
            }
            _ => self.power(),
        }
    }

    fn power(&mut self) -> Result<Ast, ParseError> {
        let base = self.atom()?;
        if matches!(self.peek(), Some(Tok::Op('^'))) {
            self.pos += 1;
            self.enter()?;
            // right associative; the exponent may carry a sign
            let e = self.unary()?;
            self.depth -= 1;
            return Ok(Ast::Bin(BinOp::Pow, Box::new(base), Box::new(e)));
        }
        Ok(base)
    }

    fn atom(&mut self) -> Result<Ast, ParseError> {
        let at = self.here();
        match self.toks.get(self.pos).map(|t| t.0.clone()) {
            Some(Tok::Num(v)) => {
                self.pos += 1;
                // a unit directly after a number: `10 mm`, `30deg`
                if let Some(Tok::Ident(name)) = self.peek().cloned() {
                    match unit(&name) {
                        Some((f, d)) => {
                            self.pos += 1;
                            return Ok(Ast::Lit(Quantity::new(v * f, d)));
                        }
                        None => {
                            return self.err(format!(
                                "unknown unit `{name}` after the number {v} (write `{v} * {name}` to multiply by a variable)"
                            ))
                        }
                    }
                }
                Ok(Ast::Lit(Quantity::scalar(v)))
            }
            Some(Tok::Ident(name)) => {
                self.pos += 1;
                if matches!(self.peek(), Some(Tok::LParen)) {
                    self.pos += 1;
                    self.enter()?;
                    let mut args = Vec::new();
                    if matches!(self.peek(), Some(Tok::RParen)) {
                        self.pos += 1;
                    } else {
                        loop {
                            args.push(self.comparison()?);
                            match self.peek() {
                                Some(Tok::Comma) => self.pos += 1,
                                Some(Tok::RParen) => {
                                    self.pos += 1;
                                    break;
                                }
                                _ => return self.err("expected `,` or `)` in the argument list"),
                            }
                        }
                    }
                    self.depth -= 1;
                    return Ok(Ast::Call(name, args));
                }
                if let Some((f, d)) = unit(&name) {
                    return Ok(Ast::Lit(Quantity::new(f, d)));
                }
                match name.as_str() {
                    "pi" => Ok(Ast::Lit(Quantity::scalar(std::f64::consts::PI))),
                    "tau" => Ok(Ast::Lit(Quantity::scalar(std::f64::consts::TAU))),
                    "e" => Ok(Ast::Lit(Quantity::scalar(std::f64::consts::E))),
                    _ => Ok(Ast::Var(name)),
                }
            }
            Some(Tok::LParen) => {
                self.pos += 1;
                self.enter()?;
                let a = self.comparison()?;
                self.depth -= 1;
                match self.peek() {
                    Some(Tok::RParen) => {
                        self.pos += 1;
                        Ok(a)
                    }
                    _ => self.err("expected `)`"),
                }
            }
            Some(t) => Err(ParseError {
                message: format!("unexpected {}", describe(&t)),
                at,
            }),
            None => self.err("the expression ends early"),
        }
    }
}

fn describe(t: &Tok) -> String {
    match t {
        Tok::Num(v) => format!("number {v}"),
        Tok::Ident(s) => format!("name `{s}`"),
        Tok::Op(c) => format!("`{c}`"),
        Tok::Le => "`<=`".into(),
        Tok::Ge => "`>=`".into(),
        Tok::EqEq => "`==`".into(),
        Tok::Ne => "`!=`".into(),
        Tok::LParen => "`(`".into(),
        Tok::RParen => "`)`".into(),
        Tok::Comma => "`,`".into(),
    }
}

pub fn parse(src: &str) -> Result<Ast, ParseError> {
    if src.len() > 10_000 {
        return Err(ParseError {
            message: "expression is longer than 10,000 characters".into(),
            at: 0,
        });
    }
    let toks = lex(src)?;
    if toks.is_empty() {
        return Err(ParseError {
            message: "the expression is empty".into(),
            at: 0,
        });
    }
    let mut p = Parser {
        toks,
        pos: 0,
        end: src.len(),
        depth: 0,
    };
    let ast = p.comparison()?;
    if p.pos < p.toks.len() {
        return p.err(format!(
            "unexpected {} after the expression",
            describe(&p.toks[p.pos].0)
        ));
    }
    Ok(ast)
}

#[allow(dead_code)]
fn _dim_is_used(_: Dim) {}
