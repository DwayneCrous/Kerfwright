//! IGES 5.3 import.
//!
//! Supported entities: 100 (circular arc), 102 (composite curve), 110 (line), 124 (transformation),
//! 126 (rational B-spline curve), 128 (rational B-spline surface), 141/142 (curve on a surface),
//! 143/144 (bounded / trimmed surface), and the manifold solid B-rep family 186/514/510/508/504/502.
//! Trimmed surfaces are sewn into closed shells; shells that close become solids.

mod parser;
mod sew;

pub use parser::{parse, Entity, IgesFile, IgesParseError};
pub use sew::{import, IgesError, ImportReport};
