//! Form (subdivision modelling): cage primitives and what the dialog needs to edit them.

use kw_subd::SubdMesh;

/// The cage shapes the FORM tab offers.
pub const KINDS: [&str; 4] = ["Box", "Cylinder", "Sphere", "Torus"];

/// `kind` is an index into [`KINDS`]; `size` is width, depth, height (a cylinder or pyramid takes its
/// diameter from the first and its length from the third; a torus its two diameters from the first two);
/// `n` is the number of sides of a cylinder, or the segments of a torus. (A pyramid cage is not offered:
/// its apex makes the surface fit very slow.)
pub fn primitive(kind: u8, size: [f64; 3], n: u32) -> SubdMesh {
    let n = n.max(3) as usize;
    let [a, b, c] = size;
    match kind {
        // sharp edges: the finished body is the box itself
        0 => SubdMesh::box_(a, b, c).crease_all(100.0),
        1 => {
            // the cap rings are sharp, the sides stay smooth
            let mut m = SubdMesh::prism(n, a / 2.0, c);
            for i in 0..n {
                let j = (i + 1) % n;
                m = m.with_crease(i as u32, j as u32, 100.0).with_crease(
                    (n + i) as u32,
                    (n + j) as u32,
                    100.0,
                );
            }
            m
        }
        // a cube cage with no creases subdivides to a rounded, sphere-like blob
        2 => SubdMesh::cube(a),
        _ => SubdMesh::torus(a / 2.0, b / 2.0, n.max(6), 6),
    }
}

/// Default size and side count of a primitive.
pub fn defaults(kind: u8) -> ([f64; 3], u32) {
    match kind {
        0 => ([20.0, 20.0, 20.0], 8),
        1 => ([20.0, 20.0, 30.0], 8),
        2 => ([20.0, 20.0, 20.0], 8),
        _ => ([40.0, 12.0, 12.0], 12),
    }
}
