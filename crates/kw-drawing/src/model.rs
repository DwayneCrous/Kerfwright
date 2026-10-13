//! The persisted drawing model. Everything has serde defaults so files written by older
//! versions keep reading as fields are added.

use crate::annot::Annotation;
use crate::scene::{Scene2d, P};
use crate::sheet::{SheetFormat, TitleBlock};
use serde::{Deserialize, Serialize};

/// Which way a view looks at the model. World is z-up; `Front` looks along +y (x to the right,
/// z up), as in the usual engineering convention.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Orientation {
    Front,
    Back,
    Top,
    Bottom,
    Left,
    Right,
    Iso,
    /// Looking along `dir`, with `up` pointing up on the sheet.
    Custom {
        dir: [f64; 3],
        up: [f64; 3],
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViewStyle {
    /// Visible edges only.
    Visible,
    /// Visible edges plus hidden edges as dashed lines.
    VisibleHidden,
    /// Every edge, hidden or not, drawn as visible.
    Wireframe,
    /// Visible edges over a grey fill of the visible faces.
    Shaded,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ViewSource {
    /// The whole design: its final solid, or the assembly when the project has one.
    Design,
    /// The assembly with all its components.
    Assembly,
    /// One component of the assembly by name.
    Component(String),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ViewKind {
    Base,
    /// Projected from another view and kept aligned with it.
    Projected {
        parent: u64,
        #[serde(default)]
        align: Align,
    },
    /// A cut through `parent` along the line `a`..`b` (in the parent's view coordinates, model
    /// units: `u` to the right, `v` up). The material the arrows point at is kept, which is to the
    /// right of `a`→`b` unless `flip`. The view's own `orientation` is the direction it looks.
    Section {
        parent: u64,
        a: P,
        b: P,
        flip: bool,
        #[serde(default)]
        align: Align,
        #[serde(default)]
        letter: String,
    },
    /// An enlarged circle of `parent`: centre and radius in the parent's view coordinates (model
    /// units). The view's `scale` is the enlargement.
    Detail {
        parent: u64,
        center: P,
        radius: f64,
        #[serde(default)]
        letter: String,
    },
}

/// How a projected view moves: along the line through its parent's centre.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Align {
    #[default]
    Free,
    /// Beside the parent: only moves left and right.
    Horizontal,
    /// Above or below the parent: only moves up and down.
    Vertical,
}

/// What a view drew the last time it was updated, so an out-of-date drawing still shows.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ViewCache {
    /// Hash of the model the geometry was made from.
    pub source_hash: u64,
    /// The view's geometry at the view's scale, relative to the view origin.
    pub scene: Scene2d,
    /// Where the view origin lies in the view's projected coordinates, in sheet mm: a model point
    /// with view coordinates `(u, v)` is drawn at `(u, v) * scale - centre` from the view origin.
    #[serde(default)]
    pub centre: P,
    /// Circles drawn face-on: centre and radius, relative to the view origin (centre marks).
    #[serde(default)]
    pub marks: Vec<(P, f64)>,
    /// Axes of cylinders seen from the side, relative to the view origin (centre lines).
    #[serde(default)]
    pub axes: Vec<(P, P)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct View {
    pub id: u64,
    pub kind: ViewKind,
    pub source: ViewSource,
    pub orientation: Orientation,
    /// Sheet mm per model unit (0.5 is a 1:2 view).
    pub scale: f64,
    pub style: ViewStyle,
    /// The view origin on the sheet, mm.
    pub pos: P,
    /// Shown under the view; empty for none.
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub cache: Option<ViewCache>,
    /// Draw centre marks and centre lines on circles and cylinders.
    #[serde(default = "yes")]
    pub center_marks: bool,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sheet {
    pub name: String,
    pub format: SheetFormat,
    #[serde(default)]
    pub title: TitleBlock,
    #[serde(default)]
    pub views: Vec<View>,
    /// Dimensions, notes and symbols.
    #[serde(default)]
    pub annots: Vec<Annotation>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct DrawingSet {
    #[serde(default)]
    pub sheets: Vec<Sheet>,
    #[serde(default)]
    pub next_id: u64,
}

impl DrawingSet {
    pub fn is_empty(&self) -> bool {
        self.sheets.is_empty()
    }

    /// Add a sheet and return its index.
    pub fn add_sheet(&mut self, name: &str, format: SheetFormat) -> usize {
        self.sheets.push(Sheet {
            name: name.to_string(),
            format,
            title: TitleBlock {
                scale: "1:1".into(),
                ..TitleBlock::default()
            },
            views: Vec::new(),
            annots: Vec::new(),
        });
        self.sheets.len() - 1
    }

    /// A fresh id for a view or annotation; unique across the whole set.
    pub fn fresh_id(&mut self) -> u64 {
        self.next_id += 1;
        self.next_id
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        ciborium::into_writer(self, &mut out).expect("serialises");
        out
    }

    pub fn from_bytes(b: &[u8]) -> Result<DrawingSet, String> {
        ciborium::from_reader(b).map_err(|e| e.to_string())
    }
}

impl View {
    /// Width and height of the drawn geometry (zero before it is generated).
    pub fn size(&self) -> P {
        self.cache
            .as_ref()
            .and_then(|c| c.scene.bounds())
            .map_or([0.0, 0.0], |(lo, hi)| [hi[0] - lo[0], hi[1] - lo[1]])
    }

    /// The views this one was made from, if any.
    pub fn parent(&self) -> Option<u64> {
        match &self.kind {
            ViewKind::Base => None,
            ViewKind::Projected { parent, .. }
            | ViewKind::Section { parent, .. }
            | ViewKind::Detail { parent, .. } => Some(*parent),
        }
    }
}

impl Sheet {
    pub fn view(&self, id: u64) -> Option<&View> {
        self.views.iter().find(|v| v.id == id)
    }

    pub fn view_mut(&mut self, id: u64) -> Option<&mut View> {
        self.views.iter_mut().find(|v| v.id == id)
    }
}

/// `1:2`, `2:1`, `1:1`, `1:2.5` for a sheet-per-model scale.
pub fn scale_label(scale: f64) -> String {
    let f = |v: f64| {
        if (v - v.round()).abs() < 1e-9 {
            format!("{}", v.round() as i64)
        } else {
            format!("{}", (v * 1000.0).round() / 1000.0)
        }
    };
    if scale >= 1.0 {
        format!("{}:1", f(scale))
    } else {
        format!("1:{}", f(1.0 / scale))
    }
}
