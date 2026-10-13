use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ToolKind {
    Flat,
    Ball,
    /// Bull-nose (corner radius).
    Bull {
        corner_radius: f64,
    },
}

/// A cylindrical holder section above the flutes: radius and length, from the tool up.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Holder {
    pub sections: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    pub number: u32,
    pub name: String,
    pub kind: ToolKind,
    pub diameter: f64,
    /// Length of the cutting part below the shank.
    pub flute_length: f64,
    /// Shank radius and exposed length above the flutes.
    pub shank_radius: f64,
    pub shank_length: f64,
    pub holder: Holder,
}

impl Tool {
    pub fn flat(number: u32, diameter: f64, flute_length: f64) -> Tool {
        Tool {
            number,
            name: format!("{diameter} mm flat end mill"),
            kind: ToolKind::Flat,
            diameter,
            flute_length,
            shank_radius: diameter / 2.0,
            shank_length: 20.0,
            holder: Holder {
                sections: vec![(20.0, 40.0)],
            },
        }
    }

    pub fn ball(number: u32, diameter: f64, flute_length: f64) -> Tool {
        Tool {
            kind: ToolKind::Ball,
            name: format!("{diameter} mm ball end mill"),
            ..Tool::flat(number, diameter, flute_length)
        }
    }

    pub fn bull(number: u32, diameter: f64, corner_radius: f64, flute_length: f64) -> Tool {
        Tool {
            kind: ToolKind::Bull { corner_radius },
            name: format!("{diameter} mm bull nose end mill (r {corner_radius})"),
            ..Tool::flat(number, diameter, flute_length)
        }
    }

    pub fn radius(&self) -> f64 {
        self.diameter / 2.0
    }
}
