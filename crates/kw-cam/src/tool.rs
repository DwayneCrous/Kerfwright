use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ToolKind {
    Flat,
    Ball,
    /// Bull-nose (corner radius).
    Bull {
        corner_radius: f64,
    },
    /// Twist drill: the tip is a cone of the included `point_angle` (degrees, 118 for general work).
    Drill {
        point_angle: f64,
    },
    /// Chamfer mill: a cone of the included `tip_angle` (degrees) ending in a flat of `tip_diameter`.
    Chamfer {
        tip_angle: f64,
        tip_diameter: f64,
    },
    /// Face mill: a large flat cutter for skimming surfaces.
    FaceMill,
    /// Engraving tool: like a chamfer mill with a fine tip.
    Engraver {
        tip_angle: f64,
        tip_diameter: f64,
    },
    /// Boring bar: cuts like a flat tool of its diameter.
    Bore,
    /// Tap with the given thread `pitch` (mm).
    Tap {
        pitch: f64,
    },
}

impl ToolKind {
    pub fn label(&self) -> &'static str {
        match self {
            ToolKind::Flat => "Flat end mill",
            ToolKind::Ball => "Ball end mill",
            ToolKind::Bull { .. } => "Bull-nose end mill",
            ToolKind::Drill { .. } => "Drill",
            ToolKind::Chamfer { .. } => "Chamfer mill",
            ToolKind::FaceMill => "Face mill",
            ToolKind::Engraver { .. } => "Engraver",
            ToolKind::Bore => "Boring bar",
            ToolKind::Tap { .. } => "Tap",
        }
    }

    /// Does the tool cut with its side (can it mill a contour or a pocket wall)?
    pub fn mills(&self) -> bool {
        matches!(
            self,
            ToolKind::Flat | ToolKind::Ball | ToolKind::Bull { .. } | ToolKind::FaceMill
        )
    }
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
    /// Number of flutes (teeth).
    #[serde(default = "default_flutes")]
    pub flutes: u32,
    /// Tool material, e.g. "Carbide".
    #[serde(default)]
    pub material: String,
    #[serde(default)]
    pub coating: String,
    /// Name of the holder in the library the `holder` sections came from.
    #[serde(default)]
    pub holder_name: String,
}

fn default_flutes() -> u32 {
    2
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
            flutes: 2,
            material: "Carbide".into(),
            coating: String::new(),
            holder_name: String::new(),
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

    /// A twist drill.
    pub fn drill(number: u32, diameter: f64, point_angle: f64, flute_length: f64) -> Tool {
        Tool {
            kind: ToolKind::Drill { point_angle },
            name: format!("{diameter} mm drill ({point_angle}°)"),
            shank_radius: diameter / 2.0,
            ..Tool::flat(number, diameter, flute_length)
        }
    }

    /// A chamfer mill of included angle `tip_angle` with a flat tip of `tip_diameter`.
    pub fn chamfer(
        number: u32,
        diameter: f64,
        tip_angle: f64,
        tip_diameter: f64,
        flute_length: f64,
    ) -> Tool {
        Tool {
            kind: ToolKind::Chamfer {
                tip_angle,
                tip_diameter,
            },
            name: format!("{diameter} mm chamfer mill ({tip_angle}°)"),
            ..Tool::flat(number, diameter, flute_length)
        }
    }

    pub fn face_mill(number: u32, diameter: f64) -> Tool {
        Tool {
            kind: ToolKind::FaceMill,
            name: format!("{diameter} mm face mill"),
            flutes: 4,
            shank_radius: (diameter / 2.0).min(16.0),
            shank_length: 10.0,
            ..Tool::flat(number, diameter, 8.0)
        }
    }

    pub fn engraver(number: u32, tip_angle: f64, tip_diameter: f64) -> Tool {
        Tool {
            kind: ToolKind::Engraver {
                tip_angle,
                tip_diameter,
            },
            name: format!("{tip_angle}° engraver ({tip_diameter} mm tip)"),
            flutes: 1,
            shank_radius: 3.0,
            ..Tool::flat(number, 6.0, 12.0)
        }
    }

    pub fn bore(number: u32, diameter: f64, flute_length: f64) -> Tool {
        Tool {
            kind: ToolKind::Bore,
            name: format!("{diameter} mm boring bar"),
            flutes: 1,
            ..Tool::flat(number, diameter, flute_length)
        }
    }

    pub fn tap(number: u32, diameter: f64, pitch: f64, flute_length: f64) -> Tool {
        Tool {
            kind: ToolKind::Tap { pitch },
            name: format!("M{diameter}x{pitch} tap"),
            flutes: 3,
            ..Tool::flat(number, diameter, flute_length)
        }
    }

    pub fn radius(&self) -> f64 {
        self.diameter / 2.0
    }
}
