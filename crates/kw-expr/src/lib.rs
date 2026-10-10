//! Expressions with units: `2 * width + 3 mm`, `sin(30 deg) * r`, `if(n > 3, 5 mm, 2 mm)`.
//!
//! * A [`Quantity`] is a value in canonical units (mm, rad, kg, s) with a [`Dim`].
//! * [`Expr`] is a parsed expression; it serialises as its source text.
//! * [`Expr::free_vars`] lists the variables an expression reads, which is what makes expression
//!   references edges of the feature graph.
//! * Every error says what is wrong and where; dimension errors name the dimensions involved.

#![allow(clippy::needless_range_loop)]

mod eval;
mod parse;
pub mod units;

pub use eval::{EvalError, Scope};
pub use parse::{is_reserved, parse, Ast, ParseError};
pub use units::Dim;

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quantity {
    /// Value in canonical units (mm, rad, kg, s and combinations).
    pub value: f64,
    pub dim: Dim,
}

impl Quantity {
    pub fn new(value: f64, dim: Dim) -> Self {
        Quantity { value, dim }
    }
    pub fn scalar(value: f64) -> Self {
        Quantity {
            value,
            dim: Dim::NONE,
        }
    }
    pub fn length(mm: f64) -> Self {
        Quantity::new(mm, Dim::LENGTH)
    }
    pub fn angle(rad: f64) -> Self {
        Quantity::new(rad, Dim::ANGLE)
    }
    /// The value if the dimension is `dim`.
    pub fn expect(&self, dim: Dim) -> Result<f64, EvalError> {
        if self.dim == dim {
            Ok(self.value)
        } else {
            Err(EvalError::Dimension(format!(
                "expected {dim}, got {}",
                self.dim
            )))
        }
    }
}

impl fmt::Display for Quantity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.dim == Dim::LENGTH {
            write!(f, "{} mm", self.value)
        } else if self.dim == Dim::ANGLE {
            write!(f, "{} rad", self.value)
        } else if self.dim.is_none() {
            write!(f, "{}", self.value)
        } else {
            write!(f, "{} [{}]", self.value, self.dim)
        }
    }
}

/// A parsed expression together with its source text.
#[derive(Debug, Clone, PartialEq)]
pub struct Expr {
    source: String,
    ast: Ast,
}

impl Expr {
    pub fn parse(source: &str) -> Result<Expr, ParseError> {
        Ok(Expr {
            ast: parse(source)?,
            source: source.to_string(),
        })
    }
    pub fn source(&self) -> &str {
        &self.source
    }
    pub fn ast(&self) -> &Ast {
        &self.ast
    }
    pub fn eval(&self, scope: &dyn Scope) -> Result<Quantity, EvalError> {
        eval::eval(&self.ast, scope)
    }
    /// Variables the expression reads (sorted, unique).
    pub fn free_vars(&self) -> Vec<String> {
        let mut set = BTreeSet::new();
        self.ast.collect_vars(&mut set);
        set.into_iter().collect()
    }
}

impl Serialize for Expr {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.source)
    }
}

impl<'de> Deserialize<'de> for Expr {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = String::deserialize(d)?;
        Expr::parse(&s).map_err(serde::de::Error::custom)
    }
}

/// A numeric parameter of a feature: a literal quantity or an expression.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "t", content = "v")]
pub enum Param {
    Lit(Quantity),
    Expr(Expr),
}

impl Param {
    pub fn length(mm: f64) -> Param {
        Param::Lit(Quantity::length(mm))
    }
    pub fn angle_deg(deg: f64) -> Param {
        Param::Lit(Quantity::angle(deg.to_radians()))
    }
    pub fn scalar(v: f64) -> Param {
        Param::Lit(Quantity::scalar(v))
    }
    pub fn expr(src: &str) -> Result<Param, ParseError> {
        Ok(Param::Expr(Expr::parse(src)?))
    }
    pub fn eval(&self, scope: &dyn Scope) -> Result<Quantity, EvalError> {
        match self {
            Param::Lit(q) => Ok(*q),
            Param::Expr(e) => e.eval(scope),
        }
    }
    pub fn free_vars(&self) -> Vec<String> {
        match self {
            Param::Lit(_) => Vec::new(),
            Param::Expr(e) => e.free_vars(),
        }
    }
}
