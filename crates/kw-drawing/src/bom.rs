//! The parts list (bill of materials) and balloons of an assembly drawing.
//!
//! A table keeps a copy of the rows it was last made from, so a sheet draws without the model.
//! Balloons carry the item number of their part; both are refreshed together when the assembly
//! is read again, so the numbers always follow the table's order.

use crate::dim::DimStatus;
use crate::model::{View, ViewCache};
use crate::projection::{ViewFrame, V3};
use crate::scene::{text_width, Anchor, Layer, Scene2d, Shape, P};
use serde::{Deserialize, Serialize};

/// One line of the parts list: a part and how many of it there are.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BomRowData {
    pub part: String,
    pub count: u32,
    pub material: String,
    /// Volume of one (mm³).
    pub volume: f64,
    /// Mass of one (g); 0 when its material is not known.
    pub mass_g: f64,
    /// Size of the part's bounding box (mm).
    pub size: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BomColumn {
    Item,
    Part,
    Quantity,
    Material,
    Mass,
    TotalMass,
    Volume,
    Size,
}

impl BomColumn {
    pub const ALL: [BomColumn; 8] = [
        BomColumn::Item,
        BomColumn::Part,
        BomColumn::Quantity,
        BomColumn::Material,
        BomColumn::Mass,
        BomColumn::TotalMass,
        BomColumn::Volume,
        BomColumn::Size,
    ];

    /// The default columns of a new table.
    pub fn defaults() -> Vec<BomColumn> {
        vec![
            BomColumn::Item,
            BomColumn::Part,
            BomColumn::Quantity,
            BomColumn::Material,
            BomColumn::Mass,
        ]
    }

    pub fn label(self) -> &'static str {
        match self {
            BomColumn::Item => "ITEM",
            BomColumn::Part => "PART",
            BomColumn::Quantity => "QTY",
            BomColumn::Material => "MATERIAL",
            BomColumn::Mass => "MASS (g)",
            BomColumn::TotalMass => "TOTAL (g)",
            BomColumn::Volume => "VOLUME (mm³)",
            BomColumn::Size => "SIZE (mm)",
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            BomColumn::Item => "Item",
            BomColumn::Part => "Part",
            BomColumn::Quantity => "Quantity",
            BomColumn::Material => "Material",
            BomColumn::Mass => "Mass each",
            BomColumn::TotalMass => "Mass total",
            BomColumn::Volume => "Volume",
            BomColumn::Size => "Size",
        }
    }

    /// The text of this column for row number `item` (1-based).
    pub fn cell(self, item: usize, r: &BomRowData) -> String {
        let mass = |g: f64| {
            if g > 0.0 {
                format!("{g:.1}")
            } else {
                "-".into()
            }
        };
        match self {
            BomColumn::Item => item.to_string(),
            BomColumn::Part => r.part.clone(),
            BomColumn::Quantity => r.count.to_string(),
            BomColumn::Material => {
                if r.material.is_empty() {
                    "-".into()
                } else {
                    r.material.clone()
                }
            }
            BomColumn::Mass => mass(r.mass_g),
            BomColumn::TotalMass => mass(r.mass_g * f64::from(r.count)),
            BomColumn::Volume => format!("{:.0}", r.volume),
            BomColumn::Size => format!("{:.0} x {:.0} x {:.0}", r.size[0], r.size[1], r.size[2]),
        }
    }
}

/// Row height and text sizes of a parts list, mm.
pub const ROW_H: f64 = 7.0;
const CELL_H: f64 = 3.2;
const HEAD_H: f64 = 2.6;
const PAD: f64 = 2.0;

/// The table with its lower-right corner at `pos`: the header row at the bottom, items counting
/// upward from it (ISO 7573).
pub fn table_scene(pos: P, columns: &[BomColumn], rows: &[BomRowData]) -> Scene2d {
    let mut s = Scene2d::new();
    if columns.is_empty() {
        return s;
    }
    let widths: Vec<f64> = columns
        .iter()
        .map(|c| {
            let w = rows
                .iter()
                .enumerate()
                .map(|(i, r)| text_width(&c.cell(i + 1, r), CELL_H))
                .fold(text_width(c.label(), HEAD_H), f64::max);
            (w + 2.0 * PAD).max(10.0)
        })
        .collect();
    let total: f64 = widths.iter().sum();
    let height = ROW_H * (rows.len() + 1) as f64;
    let (x0, y0) = (pos[0] - total, pos[1]);
    s.rect(Layer::Table, [x0, y0], [x0 + total, y0 + height]);
    for k in 1..=rows.len() {
        let y = y0 + ROW_H * k as f64;
        s.line(Layer::Table, [x0, y], [x0 + total, y]);
    }
    let mut x = x0;
    for (ci, (c, w)) in columns.iter().zip(&widths).enumerate() {
        if ci > 0 {
            s.line(Layer::Table, [x, y0], [x, y0 + height]);
        }
        s.text(
            Layer::Text,
            [x + PAD, y0 + (ROW_H - HEAD_H) / 2.0],
            HEAD_H,
            Anchor::Left,
            c.label(),
        );
        for (i, r) in rows.iter().enumerate() {
            s.text(
                Layer::Text,
                [
                    x + PAD,
                    y0 + ROW_H * (i + 1) as f64 + (ROW_H - CELL_H) / 2.0,
                ],
                CELL_H,
                Anchor::Left,
                &c.cell(i + 1, r),
            );
        }
        x += w;
    }
    s
}

/// The table as CSV: a line per part, then the totals.
pub fn bom_csv(rows: &[BomRowData]) -> String {
    fn field(s: &str) -> String {
        if s.contains([',', '"', '\n']) {
            format!("\"{}\"", s.replace('"', "\"\""))
        } else {
            s.to_string()
        }
    }
    let mut out = String::from(
        "Item,Part,Quantity,Material,Volume (mm3),Mass each (g),Mass total (g),Size X (mm),Size Y (mm),Size Z (mm)\n",
    );
    let (mut qty, mut mass) = (0u32, 0.0);
    for (i, r) in rows.iter().enumerate() {
        qty += r.count;
        mass += r.mass_g * f64::from(r.count);
        out.push_str(&format!(
            "{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}\n",
            i + 1,
            field(&r.part),
            r.count,
            field(&r.material),
            r.volume,
            r.mass_g,
            r.mass_g * f64::from(r.count),
            r.size[0],
            r.size[1],
            r.size[2],
        ));
    }
    out.push_str(&format!(",Total,{qty},,,,{mass:.3},,,\n"));
    out
}

/// Radius of a balloon, mm.
pub const BALLOON_R: f64 = 5.0;

/// A balloon at `pos` with its leader ending on `world`, a point of the part, seen in `view`.
#[allow(clippy::too_many_arguments)]
pub fn balloon_scene(
    pos: P,
    item: u32,
    status: &DimStatus,
    world: V3,
    view: &View,
    cache: &ViewCache,
) -> Scene2d {
    let mut s = Scene2d::new();
    let f = ViewFrame::of(view.orientation);
    let q = f.project(world).0;
    let target = [
        view.pos[0] + q[0] * view.scale - cache.centre[0],
        view.pos[1] + q[1] * view.scale - cache.centre[1],
    ];
    s.circle(Layer::Symbol, pos, BALLOON_R);
    let label = if matches!(status, DimStatus::Ok) {
        item.to_string()
    } else {
        "?".to_string()
    };
    s.text(
        Layer::Symbol,
        [pos[0], pos[1] - 1.25],
        3.5,
        Anchor::Center,
        &label,
    );
    // leader from the circle to the part, ending in a dot on it
    let d = [target[0] - pos[0], target[1] - pos[1]];
    let l = d[0].hypot(d[1]);
    if l > BALLOON_R + 0.5 {
        let u = [d[0] / l, d[1] / l];
        s.line(
            Layer::Symbol,
            [pos[0] + u[0] * BALLOON_R, pos[1] + u[1] * BALLOON_R],
            target,
        );
        let dot: Vec<P> = (0..8)
            .map(|i| {
                let a = std::f64::consts::TAU * f64::from(i) / 8.0;
                [target[0] + 0.8 * a.cos(), target[1] + 0.8 * a.sin()]
            })
            .collect();
        s.push(Layer::Symbol, Shape::Fill { pts: dot, shade: 0 });
    }
    s
}
