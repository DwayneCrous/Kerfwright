use kw_ssi::SsiError;

#[derive(Debug, Clone, PartialEq)]
pub enum BooleanError {
    /// An input is not a closed solid body of this B-rep, or lacks pcurves.
    BadInput(&'static str),
    /// Surface intersection failed or is unsupported for some face pair.
    Ssi(SsiError),
    /// The operands touch tangentially (along a curve or at a point): the result would be
    /// non-manifold or numerically ill-defined.
    Tangent(&'static str),
    /// A configuration the engine does not handle (e.g. a cut through a pole).
    Unsupported(&'static str),
    /// Fragment classification was ambiguous.
    Classification(&'static str),
    /// Internal topology reconstruction failed.
    Topology(String),
    /// The assembled result did not pass verification.
    Invalid(String),
    /// The operation legitimately produces no material (e.g. disjoint intersection).
    Empty,
}

impl std::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for BooleanError {}

impl From<SsiError> for BooleanError {
    fn from(e: SsiError) -> Self {
        BooleanError::Ssi(e)
    }
}
