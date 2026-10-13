//! Fabrication-file checkers that know nothing about the board model that writes the files.
//!
//! Two Gerber readers of different design:
//! * [`poly`] turns a file into dark and clear polygons and evaluates the polarity with polygon booleans
//!   (area, bounding box, the polygons themselves);
//! * [`raster`] reads the same language with its own parser and paints a bitmap pixel by pixel.
//!
//! Both reject malformed files (missing format or unit statements, undefined apertures, unterminated
//! regions, a missing `M02`, unknown commands, content after the end of the file), and an Excellon reader
//! checks drill files the same way.

pub mod excellon;
pub mod poly;
pub mod raster;

#[derive(Debug, Clone, PartialEq)]
pub struct FabError {
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for FabError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}
impl std::error::Error for FabError {}

pub(crate) fn fail<T>(line: usize, m: impl Into<String>) -> Result<T, FabError> {
    Err(FabError {
        line,
        message: m.into(),
    })
}

pub type P2 = [f64; 2];
