//! Standard thread sizes: ISO metric coarse and fine, Unified coarse (UNC) and fine (UNF).
//!
//! Every size has its nominal (major) diameter, pitch and the recommended tap-drill diameter in
//! millimetres. The minor diameters follow ISO 68-1 / ASME B1.1 for a basic profile:
//! internal D1 = D − 1.082532 P, external d3 = d − 1.226869 P, pitch diameter d2 = D − 0.649519 P.

const IN: f64 = 25.4;

/// Which series a size belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadFamily {
    MetricCoarse,
    MetricFine,
    Unc,
    Unf,
}

impl ThreadFamily {
    pub fn label(self) -> &'static str {
        match self {
            ThreadFamily::MetricCoarse => "ISO metric coarse",
            ThreadFamily::MetricFine => "ISO metric fine",
            ThreadFamily::Unc => "UNC",
            ThreadFamily::Unf => "UNF",
        }
    }
}

/// One standard thread size.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreadSize {
    pub designation: &'static str,
    pub family: ThreadFamily,
    /// Nominal (major) diameter, mm.
    pub major: f64,
    /// Pitch, mm.
    pub pitch: f64,
    /// Recommended tap-drill diameter, mm.
    pub tap_drill: f64,
}

impl ThreadSize {
    /// Minor diameter of the internal (tapped) thread.
    pub fn minor_internal(&self) -> f64 {
        self.major - 1.082_532 * self.pitch
    }
    /// Minor diameter of the external (bolt) thread.
    pub fn minor_external(&self) -> f64 {
        self.major - 1.226_869 * self.pitch
    }
    /// Pitch diameter.
    pub fn pitch_diameter(&self) -> f64 {
        self.major - 0.649_519 * self.pitch
    }
    /// Is this a hole of this diameter (mm) a plausible tap or clearance size for the thread?
    pub fn fits_hole(&self, d: f64) -> bool {
        d >= self.minor_internal() - 0.06 && d <= self.major + 0.06
    }
    /// Is a cylinder of this diameter (mm) a plausible bolt blank for the thread?
    pub fn fits_shaft(&self, d: f64) -> bool {
        d >= self.minor_external() - 0.06 && d <= self.major + 0.06
    }
}

const fn metric(
    designation: &'static str,
    family: ThreadFamily,
    major: f64,
    pitch: f64,
    tap_drill: f64,
) -> ThreadSize {
    ThreadSize {
        designation,
        family,
        major,
        pitch,
        tap_drill,
    }
}

/// A Unified size: nominal diameter, threads per inch and tap-drill diameter, all in inches.
const fn unified(
    designation: &'static str,
    family: ThreadFamily,
    major_in: f64,
    tpi: f64,
    drill_in: f64,
) -> ThreadSize {
    ThreadSize {
        designation,
        family,
        major: major_in * IN,
        pitch: IN / tpi,
        tap_drill: drill_in * IN,
    }
}

use ThreadFamily::{MetricCoarse as MC, MetricFine as MF, Unc, Unf};

/// All the sizes the application knows, in menu order.
pub static SIZES: &[ThreadSize] = &[
    metric("M2", MC, 2.0, 0.4, 1.6),
    metric("M2.5", MC, 2.5, 0.45, 2.05),
    metric("M3", MC, 3.0, 0.5, 2.5),
    metric("M4", MC, 4.0, 0.7, 3.3),
    metric("M5", MC, 5.0, 0.8, 4.2),
    metric("M6", MC, 6.0, 1.0, 5.0),
    metric("M8", MC, 8.0, 1.25, 6.8),
    metric("M10", MC, 10.0, 1.5, 8.5),
    metric("M12", MC, 12.0, 1.75, 10.2),
    metric("M14", MC, 14.0, 2.0, 12.0),
    metric("M16", MC, 16.0, 2.0, 14.0),
    metric("M20", MC, 20.0, 2.5, 17.5),
    metric("M24", MC, 24.0, 3.0, 21.0),
    metric("M6x0.75", MF, 6.0, 0.75, 5.2),
    metric("M8x1", MF, 8.0, 1.0, 7.0),
    metric("M10x1.25", MF, 10.0, 1.25, 8.8),
    metric("M10x1", MF, 10.0, 1.0, 9.0),
    metric("M12x1.5", MF, 12.0, 1.5, 10.5),
    metric("M12x1.25", MF, 12.0, 1.25, 10.8),
    metric("M16x1.5", MF, 16.0, 1.5, 14.5),
    metric("M20x1.5", MF, 20.0, 1.5, 18.5),
    unified("#4-40 UNC", Unc, 0.112, 40.0, 0.089),
    unified("#6-32 UNC", Unc, 0.138, 32.0, 0.1065),
    unified("#8-32 UNC", Unc, 0.164, 32.0, 0.136),
    unified("#10-24 UNC", Unc, 0.190, 24.0, 0.1495),
    unified("1/4-20 UNC", Unc, 0.25, 20.0, 0.201),
    unified("5/16-18 UNC", Unc, 0.3125, 18.0, 0.257),
    unified("3/8-16 UNC", Unc, 0.375, 16.0, 0.3125),
    unified("7/16-14 UNC", Unc, 0.4375, 14.0, 0.368),
    unified("1/2-13 UNC", Unc, 0.5, 13.0, 0.4219),
    unified("5/8-11 UNC", Unc, 0.625, 11.0, 0.5313),
    unified("3/4-10 UNC", Unc, 0.75, 10.0, 0.6563),
    unified("#10-32 UNF", Unf, 0.190, 32.0, 0.159),
    unified("1/4-28 UNF", Unf, 0.25, 28.0, 0.213),
    unified("5/16-24 UNF", Unf, 0.3125, 24.0, 0.272),
    unified("3/8-24 UNF", Unf, 0.375, 24.0, 0.332),
    unified("1/2-20 UNF", Unf, 0.5, 20.0, 0.4531),
];

/// The size with this designation (`M6`, `1/4-20 UNC`, …), ignoring case.
pub fn lookup(designation: &str) -> Option<&'static ThreadSize> {
    let d = designation.trim();
    SIZES.iter().find(|s| s.designation.eq_ignore_ascii_case(d))
}

/// The sizes of one family.
pub fn family(f: ThreadFamily) -> impl Iterator<Item = &'static ThreadSize> {
    SIZES.iter().filter(move |s| s.family == f)
}

/// What the model reports about a thread: the same text for tapped holes and cosmetic threads.
pub fn describe(s: &ThreadSize, length_mm: f64, internal: bool) -> String {
    format!(
        "{} thread {}: {} mm long, pitch {:.3} mm, major dia {:.3}, minor dia {:.3}{}",
        if internal { "internal" } else { "external" },
        s.designation,
        trim(length_mm),
        s.pitch,
        s.major,
        if internal {
            s.minor_internal()
        } else {
            s.minor_external()
        },
        if internal {
            format!(", tap drill dia {:.3}", s.tap_drill)
        } else {
            String::new()
        },
    )
}

fn trim(v: f64) -> String {
    let s = format!("{v:.3}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// A thread of the model: from a tapped hole or a cosmetic thread feature.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreadInfo {
    pub feature: crate::FeatureId,
    pub size: &'static ThreadSize,
    pub internal: bool,
    pub length: f64,
    /// A tapped hole (true) or a cosmetic thread on a face (false).
    pub from_hole: bool,
}

impl ThreadInfo {
    pub fn text(&self) -> String {
        describe(self.size, self.length, self.internal)
    }
}

/// Every thread in the document, in feature order (suppressed features and unreadable ones skipped).
pub fn report(doc: &crate::Document, scope: &dyn kw_expr::Scope) -> Vec<ThreadInfo> {
    use crate::spec::{FeatureSpec, HoleKind};
    let mut v = Vec::new();
    for f in doc.features.iter().filter(|f| !f.suppressed) {
        let Ok(spec) = f.spec.resolved(scope) else {
            continue;
        };
        let len = |p: &kw_expr::Param| match p {
            kw_expr::Param::Lit(q) => Some(q.value),
            _ => None,
        };
        match &spec {
            FeatureSpec::Hole(h) => {
                if let HoleKind::Tapped {
                    designation,
                    thread_length,
                } = &h.kind
                {
                    if let (Some(size), Some(length)) = (lookup(designation), len(thread_length)) {
                        for _ in &h.points {
                            v.push(ThreadInfo {
                                feature: f.id,
                                size,
                                internal: true,
                                length,
                                from_hole: true,
                            });
                        }
                    }
                }
            }
            FeatureSpec::Thread(t) => {
                if let (Some(size), Some(length)) = (lookup(&t.designation), len(&t.length)) {
                    v.push(ThreadInfo {
                        feature: f.id,
                        size,
                        internal: t.internal,
                        length,
                        from_hole: false,
                    });
                }
            }
            _ => {}
        }
    }
    v
}
