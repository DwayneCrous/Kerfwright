//! Evaluation with dimension checking.

use crate::parse::{Ast, BinOp};
use crate::units::Dim;
use crate::Quantity;
use std::collections::HashMap;
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum EvalError {
    UnknownVariable(String),
    UnknownFunction(String),
    Dimension(String),
    Domain(String),
    Arity {
        function: String,
        expected: &'static str,
        got: usize,
    },
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EvalError::UnknownVariable(v) => write!(f, "unknown variable `{v}`"),
            EvalError::UnknownFunction(v) => write!(f, "unknown function `{v}`"),
            EvalError::Dimension(m) => write!(f, "dimension mismatch: {m}"),
            EvalError::Domain(m) => write!(f, "{m}"),
            EvalError::Arity {
                function,
                expected,
                got,
            } => write!(f, "`{function}` takes {expected} argument(s), got {got}"),
        }
    }
}
impl std::error::Error for EvalError {}

/// Variable lookup.
pub trait Scope {
    fn lookup(&self, name: &str) -> Option<Quantity>;
}

impl Scope for HashMap<String, Quantity> {
    fn lookup(&self, name: &str) -> Option<Quantity> {
        self.get(name).copied()
    }
}

impl Scope for std::collections::BTreeMap<String, Quantity> {
    fn lookup(&self, name: &str) -> Option<Quantity> {
        self.get(name).copied()
    }
}

fn finite(q: Quantity, what: &str) -> Result<Quantity, EvalError> {
    if q.value.is_finite() {
        Ok(q)
    } else {
        Err(EvalError::Domain(format!("{what} is not a finite number")))
    }
}

pub fn eval(ast: &Ast, scope: &dyn Scope) -> Result<Quantity, EvalError> {
    match ast {
        Ast::Lit(q) => Ok(*q),
        Ast::Var(v) => scope
            .lookup(v)
            .ok_or_else(|| EvalError::UnknownVariable(v.clone())),
        Ast::Neg(a) => {
            let q = eval(a, scope)?;
            Ok(Quantity::new(-q.value, q.dim))
        }
        Ast::Bin(op, a, b) => {
            let (x, y) = (eval(a, scope)?, eval(b, scope)?);
            bin(*op, x, y)
        }
        Ast::Call(name, args) => {
            let vals: Vec<Quantity> = args
                .iter()
                .map(|a| eval(a, scope))
                .collect::<Result<_, _>>()?;
            call(name, &vals)
        }
    }
}

fn same(x: Quantity, y: Quantity, what: &str) -> Result<(), EvalError> {
    if x.dim == y.dim {
        Ok(())
    } else {
        Err(EvalError::Dimension(format!(
            "cannot {what} {} and {}",
            x.dim, y.dim
        )))
    }
}

fn bin(op: BinOp, x: Quantity, y: Quantity) -> Result<Quantity, EvalError> {
    let overflow = || EvalError::Dimension("the dimension exponents overflow".into());
    match op {
        BinOp::Add => {
            same(x, y, "add")?;
            finite(Quantity::new(x.value + y.value, x.dim), "the sum")
        }
        BinOp::Sub => {
            same(x, y, "subtract")?;
            finite(Quantity::new(x.value - y.value, x.dim), "the difference")
        }
        BinOp::Mul => finite(
            Quantity::new(x.value * y.value, x.dim.times(y.dim).ok_or_else(overflow)?),
            "the product",
        ),
        BinOp::Div => {
            if y.value == 0.0 {
                return Err(EvalError::Domain("division by zero".into()));
            }
            finite(
                Quantity::new(x.value / y.value, x.dim.over(y.dim).ok_or_else(overflow)?),
                "the quotient",
            )
        }
        BinOp::Pow => {
            if !y.dim.is_none() {
                return Err(EvalError::Dimension(format!(
                    "an exponent must be dimensionless, got {}",
                    y.dim
                )));
            }
            if x.dim.is_none() {
                finite(Quantity::scalar(x.value.powf(y.value)), "the power")
            } else {
                if y.value.fract() != 0.0 || y.value.abs() > 16.0 {
                    return Err(EvalError::Dimension(format!(
                        "a quantity with dimension {} can only be raised to an integer power (|n| ≤ 16)",
                        x.dim
                    )));
                }
                let n = y.value as i32;
                finite(
                    Quantity::new(x.value.powi(n), x.dim.pow(n).ok_or_else(overflow)?),
                    "the power",
                )
            }
        }
        BinOp::Lt | BinOp::Gt | BinOp::Le | BinOp::Ge | BinOp::Eq | BinOp::Ne => {
            same(x, y, "compare")?;
            let r = match op {
                BinOp::Lt => x.value < y.value,
                BinOp::Gt => x.value > y.value,
                BinOp::Le => x.value <= y.value,
                BinOp::Ge => x.value >= y.value,
                BinOp::Eq => x.value == y.value,
                _ => x.value != y.value,
            };
            Ok(Quantity::scalar(if r { 1.0 } else { 0.0 }))
        }
    }
}

fn arity(name: &str, args: &[Quantity], n: usize) -> Result<(), EvalError> {
    if args.len() == n {
        Ok(())
    } else {
        Err(EvalError::Arity {
            function: name.into(),
            expected: match n {
                1 => "1",
                2 => "2",
                _ => "3",
            },
            got: args.len(),
        })
    }
}

fn angle_or_none(name: &str, q: Quantity) -> Result<f64, EvalError> {
    if q.dim == Dim::ANGLE || q.dim.is_none() {
        Ok(q.value)
    } else {
        Err(EvalError::Dimension(format!(
            "`{name}` takes an angle or a plain number (radians), got {}",
            q.dim
        )))
    }
}

fn dimensionless(name: &str, q: Quantity) -> Result<f64, EvalError> {
    if q.dim.is_none() {
        Ok(q.value)
    } else {
        Err(EvalError::Dimension(format!(
            "`{name}` takes a plain number, got {}",
            q.dim
        )))
    }
}

fn call(name: &str, a: &[Quantity]) -> Result<Quantity, EvalError> {
    let r = match name {
        "sin" | "cos" | "tan" => {
            arity(name, a, 1)?;
            let v = angle_or_none(name, a[0])?;
            Quantity::scalar(match name {
                "sin" => v.sin(),
                "cos" => v.cos(),
                _ => v.tan(),
            })
        }
        "asin" | "acos" | "atan" => {
            arity(name, a, 1)?;
            let v = dimensionless(name, a[0])?;
            if (name == "asin" || name == "acos") && v.abs() > 1.0 {
                return Err(EvalError::Domain(format!(
                    "`{name}` needs |x| ≤ 1, got {v}"
                )));
            }
            Quantity::angle(match name {
                "asin" => v.asin(),
                "acos" => v.acos(),
                _ => v.atan(),
            })
        }
        "atan2" => {
            arity(name, a, 2)?;
            same(a[0], a[1], "take atan2 of")?;
            Quantity::angle(a[0].value.atan2(a[1].value))
        }
        "sqrt" => {
            arity(name, a, 1)?;
            if a[0].value < 0.0 {
                return Err(EvalError::Domain("sqrt of a negative number".into()));
            }
            let d = a[0].dim.sqrt().ok_or_else(|| {
                EvalError::Dimension(format!("cannot take the square root of {}", a[0].dim))
            })?;
            Quantity::new(a[0].value.sqrt(), d)
        }
        "abs" | "floor" | "ceil" | "round" => {
            arity(name, a, 1)?;
            let v = a[0].value;
            Quantity::new(
                match name {
                    "abs" => v.abs(),
                    "floor" => v.floor(),
                    "ceil" => v.ceil(),
                    _ => v.round(),
                },
                a[0].dim,
            )
        }
        "min" | "max" => {
            if a.is_empty() {
                return Err(EvalError::Arity {
                    function: name.into(),
                    expected: "at least 1",
                    got: 0,
                });
            }
            let mut best = a[0];
            for q in &a[1..] {
                same(best, *q, "compare")?;
                if (name == "min" && q.value < best.value)
                    || (name == "max" && q.value > best.value)
                {
                    best = *q;
                }
            }
            best
        }
        "exp" | "ln" | "log10" => {
            arity(name, a, 1)?;
            let v = dimensionless(name, a[0])?;
            if name != "exp" && v <= 0.0 {
                return Err(EvalError::Domain(format!(
                    "`{name}` needs a positive number"
                )));
            }
            Quantity::scalar(match name {
                "exp" => v.exp(),
                "ln" => v.ln(),
                _ => v.log10(),
            })
        }
        "pow" => {
            arity(name, a, 2)?;
            return bin(BinOp::Pow, a[0], a[1]);
        }
        "if" => {
            arity(name, a, 3)?;
            let c = dimensionless(name, a[0])?;
            same(a[1], a[2], "choose between")?;
            if c != 0.0 {
                a[1]
            } else {
                a[2]
            }
        }
        _ => return Err(EvalError::UnknownFunction(name.to_string())),
    };
    finite(r, &format!("the result of `{name}`"))
}
