//! Engineering drawings from a 3-D design: sheets, views, annotation, tables and the PDF and DXF
//! writers. Everything draws into one flat [`Scene2d`] that the canvas and the writers share.
//! The crate has no UI and no global state.

pub mod dxf_out;
pub mod hlr;
pub mod model;
pub mod pdf;
pub mod projection;
pub mod render;
pub mod scene;
pub mod sheet;
pub mod update;

pub use model::{
    scale_label, Align, DrawingSet, Orientation, Sheet, View, ViewCache, ViewKind, ViewSource,
    ViewStyle,
};
pub use scene::{Anchor, Layer, LineStyle, Prim, Scene2d, Shape, P};
pub use sheet::{PaperSize, ProjectionAngle, SheetFormat, SheetFrame, Standard, TitleBlock, Units};

/// PDF bytes of every sheet of the set, one page each.
pub fn set_to_pdf(title: &str, set: &DrawingSet) -> Vec<u8> {
    let count = set.sheets.len();
    let scenes: Vec<(P, Scene2d)> = set
        .sheets
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let (w, h) = s.format.size.dims();
            ([w, h], s.scene(i + 1, count))
        })
        .collect();
    let pages: Vec<pdf::Page> = scenes
        .iter()
        .map(|(size, scene)| pdf::Page { size: *size, scene })
        .collect();
    pdf::write_pdf(title, &pages)
}

/// ASCII DXF text of one sheet.
pub fn sheet_to_dxf(set: &DrawingSet, index: usize) -> Option<String> {
    let s = set.sheets.get(index)?;
    Some(dxf_out::to_dxf(&s.scene(index + 1, set.sheets.len())))
}
