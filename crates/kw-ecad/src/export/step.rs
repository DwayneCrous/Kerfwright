//! A board as a STEP file: the laminate and one box per component (plus a small pin-1 marker on its top face,
//! which makes the orientation, including bottom-side mirroring, checkable), all as solids of one product.

use crate::board::{Board, Side};
use kw_brep::builders::make_box;
use kw_brep::{BRep, BodyId};
use kw_math::{Frame3, Point3, Vector3};

/// Marker size and its position on the body, in footprint coordinates relative to the body corner.
pub const MARKER: [f64; 3] = [0.4, 0.4, 0.1];
pub const MARKER_INSET: f64 = 0.35;

/// What a body of the export is.
#[derive(Debug, Clone, PartialEq)]
pub enum BodyRole {
    Board,
    Component(String),
    Pin1(String),
}

/// The frame of a part: origin on the board surface at the footprint origin, axes the images of the
/// footprint axes (x mirrored for the bottom side, the part turned upside down).
pub fn part_frame(board: &Board, part: usize) -> Frame3 {
    let p = &board.parts[part];
    let (s, c) = p.rot.to_radians().sin_cos();
    let (ex, ey) = match p.side {
        Side::Top => (Vector3::new(c, s, 0.0), Vector3::new(-s, c, 0.0)),
        Side::Bottom => (Vector3::new(-c, -s, 0.0), Vector3::new(-s, c, 0.0)),
    };
    let (z0, ez) = match p.side {
        Side::Top => (0.0, Vector3::new(0.0, 0.0, 1.0)),
        Side::Bottom => (-board.thickness, Vector3::new(0.0, 0.0, -1.0)),
    };
    Frame3 {
        origin: Point3::new(p.pos[0], p.pos[1], z0),
        x: ex,
        y: ey,
        z: ez,
    }
}

fn at(f: &Frame3, x: f64, y: f64, z: f64) -> Point3<f64> {
    f.origin + f.x * x + f.y * y + f.z * z
}

/// Build the solids; returns the kernel, the bodies and their roles in export order.
pub fn build(board: &Board) -> (BRep, Vec<BodyId>, Vec<BodyRole>) {
    let mut rep = BRep::new();
    let mut bodies = Vec::new();
    let mut roles = Vec::new();
    let w = Frame3 {
        origin: Point3::new(
            board.width / 2.0,
            board.height / 2.0,
            -board.thickness / 2.0,
        ),
        ..Frame3::world()
    };
    bodies.push(make_box(&mut rep, &w, board.width, board.height, board.thickness).body);
    roles.push(BodyRole::Board);
    for (i, p) in board.parts.iter().enumerate() {
        let fp = &board.footprints[p.footprint];
        let [bx, by, bh] = fp.body;
        let f = part_frame(board, i);
        let body = Frame3 {
            origin: at(&f, 0.0, 0.0, bh / 2.0),
            ..f
        };
        bodies.push(make_box(&mut rep, &body, bx, by, bh).body);
        roles.push(BodyRole::Component(p.refdes.clone()));
        let m = Frame3 {
            origin: at(
                &f,
                -bx / 2.0 + MARKER_INSET,
                by / 2.0 - MARKER_INSET,
                bh + MARKER[2] / 2.0,
            ),
            ..f
        };
        bodies.push(make_box(&mut rep, &m, MARKER[0], MARKER[1], MARKER[2]).body);
        roles.push(BodyRole::Pin1(p.refdes.clone()));
    }
    (rep, bodies, roles)
}

pub fn step(board: &Board) -> String {
    let (rep, bodies, _) = build(board);
    kw_step::export(
        &rep,
        &bodies,
        &kw_step::ExportOptions {
            product_name: board.name.clone(),
            ..Default::default()
        },
    )
}
