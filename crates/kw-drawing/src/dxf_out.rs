//! Scene → DXF. Dimensions, hatches and tables are written as plain lines, polylines and text
//! (exploded), which every DXF reader accepts.

use crate::scene::{Anchor, Layer, Scene2d, Shape};
use kw_dxf::{DxfDoc, DxfEntity, LayerDef};

pub fn layer_defs() -> Vec<LayerDef> {
    Layer::ALL
        .iter()
        .map(|l| {
            LayerDef::new(
                l.name(),
                l.aci(),
                l.style().dxf_name(),
                (l.weight() * 100.0).round() as i32,
            )
        })
        .collect()
}

pub fn to_doc(scene: &Scene2d) -> DxfDoc {
    let mut entities = Vec::with_capacity(scene.prims.len());
    for pr in &scene.prims {
        let layer = pr.layer.name().to_string();
        let t = |p: [f64; 2]| (p[0], p[1]);
        match &pr.shape {
            Shape::Line { a, b } => entities.push(DxfEntity::Line {
                layer,
                a: t(*a),
                b: t(*b),
            }),
            Shape::Circle { c, r } => entities.push(DxfEntity::Circle {
                layer,
                c: t(*c),
                r: *r,
            }),
            Shape::Arc { c, r, a0, a1 } => entities.push(DxfEntity::Arc {
                layer,
                c: t(*c),
                r: *r,
                a0: a0.to_degrees().rem_euclid(360.0),
                a1: a1.to_degrees().rem_euclid(360.0),
            }),
            Shape::Ellipse {
                c,
                major,
                ratio,
                t0,
                t1,
            } => entities.push(DxfEntity::Ellipse {
                layer,
                c: t(*c),
                major: t(*major),
                ratio: *ratio,
                t0: *t0,
                t1: *t1,
            }),
            Shape::Polyline { pts, closed } => entities.push(DxfEntity::LwPolyline {
                layer,
                closed: *closed,
                verts: pts.iter().map(|p| (p[0], p[1], 0.0)).collect(),
            }),
            Shape::Fill { pts, .. } => entities.push(DxfEntity::LwPolyline {
                layer,
                closed: true,
                verts: pts.iter().map(|p| (p[0], p[1], 0.0)).collect(),
            }),
            Shape::Text {
                p,
                height,
                angle,
                anchor,
                text,
            } => entities.push(DxfEntity::Text {
                layer,
                p: t(*p),
                height: *height,
                rotation: angle.to_degrees(),
                halign: match anchor {
                    Anchor::Left => 0,
                    Anchor::Center => 1,
                    Anchor::Right => 2,
                },
                // DXF R2000 text is plain ASCII-safe; keep the drafting symbols as %%-codes
                text: text
                    .replace('°', "%%d")
                    .replace('±', "%%p")
                    .replace(['Ø', '⌀'], "%%c"),
            }),
        }
    }
    DxfDoc { entities }
}

/// ASCII DXF text of one sheet.
pub fn to_dxf(scene: &Scene2d) -> String {
    kw_dxf::write_with_layers(&to_doc(scene), &layer_defs())
}
