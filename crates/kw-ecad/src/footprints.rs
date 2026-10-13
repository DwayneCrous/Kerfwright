//! A small library of footprints for the generated benchmark boards and the examples.

use crate::board::{Footprint, Pad, PadShape};

fn smd(name: &str, x: f64, y: f64, w: f64, h: f64) -> Pad {
    Pad {
        name: name.into(),
        pos: [x, y],
        shape: PadShape::Rect { w, h },
        drill: None,
    }
}

fn tht(name: &str, x: f64, y: f64, d: f64, drill: f64) -> Pad {
    Pad {
        name: name.into(),
        pos: [x, y],
        shape: PadShape::Circle { d },
        drill: Some(drill),
    }
}

/// 0603 chip resistor / capacitor.
pub fn chip_0603() -> Footprint {
    Footprint {
        name: "0603".into(),
        pads: vec![smd("1", -0.8, 0.0, 0.8, 0.9), smd("2", 0.8, 0.0, 0.8, 0.9)],
        courtyard: [1.5, 0.8],
        body: [1.6, 0.8, 0.45],
    }
}

/// SOIC-8, 1.27 mm pitch.
pub fn soic8() -> Footprint {
    let mut pads = Vec::new();
    for i in 0..4 {
        pads.push(smd(
            &(i + 1).to_string(),
            -2.7,
            1.905 - 1.27 * i as f64,
            1.5,
            0.6,
        ));
        pads.push(smd(
            &(8 - i).to_string(),
            2.7,
            1.905 - 1.27 * i as f64,
            1.5,
            0.6,
        ));
    }
    pads.sort_by_key(|p| p.name.parse::<u32>().unwrap());
    Footprint {
        name: "SOIC8".into(),
        pads,
        courtyard: [3.9, 2.9],
        body: [3.9, 4.9, 1.6],
    }
}

/// QFP-32 at 0.8 mm pitch.
pub fn qfp32() -> Footprint {
    let mut pads = Vec::new();
    let pitch = 0.8;
    let reach = 4.35;
    for i in 0..8 {
        let o = (i as f64 - 3.5) * pitch;
        pads.push(smd(&(i + 1).to_string(), -reach, -o, 1.4, 0.45));
        pads.push(smd(&(9 + i).to_string(), o, -reach, 0.45, 1.4));
        pads.push(smd(&(17 + i).to_string(), reach, o, 1.4, 0.45));
        pads.push(smd(&(25 + i).to_string(), -o, reach, 0.45, 1.4));
    }
    pads.sort_by_key(|p| p.name.parse::<u32>().unwrap());
    Footprint {
        name: "QFP32".into(),
        pads,
        courtyard: [5.6, 5.6],
        body: [7.0, 7.0, 1.4],
    }
}

/// Single-row pin header with `n` pins at 2.54 mm.
pub fn header(n: usize) -> Footprint {
    let pads = (0..n)
        .map(|i| {
            tht(
                &(i + 1).to_string(),
                0.0,
                (i as f64 - (n as f64 - 1.0) / 2.0) * 2.54,
                1.7,
                1.0,
            )
        })
        .collect();
    Footprint {
        name: format!("HDR{n}"),
        pads,
        courtyard: [1.4, n as f64 * 1.27 + 0.3],
        body: [2.5, n as f64 * 2.54, 8.5],
    }
}

/// Axial through-hole resistor, 10.16 mm span.
pub fn axial() -> Footprint {
    Footprint {
        name: "AXIAL".into(),
        pads: vec![
            tht("1", -5.08, 0.0, 1.8, 0.9),
            tht("2", 5.08, 0.0, 1.8, 0.9),
        ],
        courtyard: [6.4, 1.5],
        body: [6.3, 2.5, 2.5],
    }
}
