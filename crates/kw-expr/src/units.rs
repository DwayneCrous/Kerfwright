//! Dimensions and the unit table.
//!
//! Values are stored in canonical units: length in millimetres, angle in radians, mass in kilograms,
//! time in seconds. A [`Dim`] is the exponent vector over (length, mass, time, angle).

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub struct Dim(pub [i8; 4]);

impl Dim {
    pub const NONE: Dim = Dim([0, 0, 0, 0]);
    pub const LENGTH: Dim = Dim([1, 0, 0, 0]);
    pub const MASS: Dim = Dim([0, 1, 0, 0]);
    pub const TIME: Dim = Dim([0, 0, 1, 0]);
    pub const ANGLE: Dim = Dim([0, 0, 0, 1]);

    pub fn is_none(&self) -> bool {
        *self == Dim::NONE
    }
    pub fn times(self, o: Dim) -> Option<Dim> {
        let mut r = [0i8; 4];
        for i in 0..4 {
            r[i] = self.0[i].checked_add(o.0[i])?;
        }
        Some(Dim(r))
    }
    pub fn over(self, o: Dim) -> Option<Dim> {
        let mut r = [0i8; 4];
        for i in 0..4 {
            r[i] = self.0[i].checked_sub(o.0[i])?;
        }
        Some(Dim(r))
    }
    pub fn pow(self, n: i32) -> Option<Dim> {
        let mut r = [0i8; 4];
        for i in 0..4 {
            r[i] = i8::try_from(i32::from(self.0[i]).checked_mul(n)?).ok()?;
        }
        Some(Dim(r))
    }
    /// Square root of a dimension whose exponents are all even.
    pub fn sqrt(self) -> Option<Dim> {
        if self.0.iter().any(|e| e % 2 != 0) {
            return None;
        }
        Some(Dim([
            self.0[0] / 2,
            self.0[1] / 2,
            self.0[2] / 2,
            self.0[3] / 2,
        ]))
    }
}

impl fmt::Display for Dim {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_none() {
            return write!(f, "dimensionless");
        }
        let names = ["length", "mass", "time", "angle"];
        let mut first = true;
        for (i, e) in self.0.iter().enumerate() {
            if *e == 0 {
                continue;
            }
            if !first {
                write!(f, "·")?;
            }
            first = false;
            if *e == 1 {
                write!(f, "{}", names[i])?;
            } else {
                write!(f, "{}^{}", names[i], e)?;
            }
        }
        Ok(())
    }
}

/// `(name, factor to the canonical unit, dimension)`.
pub const UNITS: &[(&str, f64, Dim)] = &[
    ("um", 1e-3, Dim::LENGTH),
    ("mm", 1.0, Dim::LENGTH),
    ("cm", 10.0, Dim::LENGTH),
    ("m", 1000.0, Dim::LENGTH),
    ("km", 1e6, Dim::LENGTH),
    ("in", 25.4, Dim::LENGTH),
    ("ft", 304.8, Dim::LENGTH),
    ("thou", 0.0254, Dim::LENGTH),
    ("rad", 1.0, Dim::ANGLE),
    ("deg", std::f64::consts::PI / 180.0, Dim::ANGLE),
    ("turn", std::f64::consts::TAU, Dim::ANGLE),
    ("mg", 1e-6, Dim::MASS),
    ("g", 1e-3, Dim::MASS),
    ("kg", 1.0, Dim::MASS),
    ("lb", 0.453_592_37, Dim::MASS),
    ("ms", 1e-3, Dim::TIME),
    ("s", 1.0, Dim::TIME),
    ("min", 60.0, Dim::TIME),
    ("h", 3600.0, Dim::TIME),
];

pub fn unit(name: &str) -> Option<(f64, Dim)> {
    UNITS
        .iter()
        .find(|(n, _, _)| *n == name)
        .map(|(_, f, d)| (*f, *d))
}
