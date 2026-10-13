use crate::P2;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Side {
    Top,
    Bottom,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PadShape {
    Rect { w: f64, h: f64 },
    Circle { d: f64 },
    Oval { w: f64, h: f64 },
}

impl PadShape {
    /// Half the extent along x and y.
    pub fn half_extents(&self) -> (f64, f64) {
        match *self {
            PadShape::Rect { w, h } | PadShape::Oval { w, h } => (w / 2.0, h / 2.0),
            PadShape::Circle { d } => (d / 2.0, d / 2.0),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pad {
    pub name: String,
    /// Centre in footprint coordinates (component on the top side).
    pub pos: P2,
    pub shape: PadShape,
    /// Plated through-hole: copper on every layer, with this drill diameter.
    pub drill: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Footprint {
    pub name: String,
    pub pads: Vec<Pad>,
    /// Courtyard half sizes about the origin.
    pub courtyard: P2,
    /// Body size (x, y, height) for 3-D export; the body is centred on the footprint origin.
    pub body: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Part {
    pub refdes: String,
    pub value: String,
    pub footprint: usize,
    pub pos: P2,
    /// Rotation in degrees, counter-clockwise, applied after mirroring for bottom-side parts.
    pub rot: f64,
    pub side: Side,
    /// Net index of every pad (same order as the footprint's pads).
    pub nets: Vec<Option<usize>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Track {
    pub net: usize,
    pub layer: usize,
    pub width: f64,
    pub a: P2,
    pub b: P2,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Via {
    pub net: usize,
    pub pos: P2,
    pub dia: f64,
    pub drill: f64,
}

/// A copper pour: shapes as contour lists (the first contour is the outline, counter-clockwise; the rest
/// are holes, clockwise).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Zone {
    pub net: usize,
    pub layer: usize,
    pub shapes: Vec<Vec<Vec<P2>>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Rules {
    pub clearance: f64,
    pub min_track: f64,
    pub min_annular: f64,
    pub min_drill: f64,
    /// Copper to board edge.
    pub edge_clearance: f64,
    pub track_width: f64,
    pub via_dia: f64,
    pub via_drill: f64,
}

impl Default for Rules {
    fn default() -> Self {
        Rules {
            clearance: 0.2,
            min_track: 0.15,
            min_annular: 0.13,
            min_drill: 0.3,
            edge_clearance: 0.3,
            track_width: 0.2,
            via_dia: 0.6,
            via_drill: 0.3,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Board {
    pub name: String,
    pub width: f64,
    pub height: f64,
    /// 2 or 4 copper layers; layer 0 is the top, the last layer the bottom.
    pub layers: usize,
    pub thickness: f64,
    pub rules: Rules,
    pub nets: Vec<String>,
    pub footprints: Vec<Footprint>,
    pub parts: Vec<Part>,
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
    pub zones: Vec<Zone>,
    /// Non-plated mounting holes: position and drill.
    pub holes: Vec<(P2, f64)>,
}

impl Board {
    pub fn new(name: &str, width: f64, height: f64, layers: usize) -> Board {
        Board {
            name: name.into(),
            width,
            height,
            layers,
            thickness: 1.6,
            rules: Rules::default(),
            nets: Vec::new(),
            footprints: Vec::new(),
            parts: Vec::new(),
            tracks: Vec::new(),
            vias: Vec::new(),
            zones: Vec::new(),
            holes: Vec::new(),
        }
    }

    pub fn net(&mut self, name: &str) -> usize {
        if let Some(i) = self.nets.iter().position(|n| n == name) {
            return i;
        }
        self.nets.push(name.into());
        self.nets.len() - 1
    }

    pub fn add_footprint(&mut self, f: Footprint) -> usize {
        if let Some(i) = self.footprints.iter().position(|g| g.name == f.name) {
            return i;
        }
        self.footprints.push(f);
        self.footprints.len() - 1
    }

    /// Place a part; pads are unconnected until assigned.
    pub fn place(
        &mut self,
        refdes: &str,
        footprint: usize,
        pos: P2,
        rot: f64,
        side: Side,
    ) -> usize {
        let n = self.footprints[footprint].pads.len();
        self.parts.push(Part {
            refdes: refdes.into(),
            value: String::new(),
            footprint,
            pos,
            rot,
            side,
            nets: vec![None; n],
        });
        self.parts.len() - 1
    }

    pub fn connect(&mut self, part: usize, pad: &str, net: usize) {
        let fp = self.parts[part].footprint;
        let idx = self.footprints[fp]
            .pads
            .iter()
            .position(|p| p.name == pad)
            .unwrap_or_else(|| panic!("footprint has no pad {pad}"));
        self.parts[part].nets[idx] = Some(net);
    }

    /// Board coordinates of a footprint point of `part` (mirrored in x for the bottom side).
    pub fn to_board(&self, part: &Part, p: P2) -> P2 {
        let x = if part.side == Side::Bottom {
            -p[0]
        } else {
            p[0]
        };
        let (s, c) = part.rot.to_radians().sin_cos();
        [
            part.pos[0] + c * x - s * p[1],
            part.pos[1] + s * x + c * p[1],
        ]
    }

    /// Rotation of footprint x axis in board coordinates, degrees.
    pub fn pad_angle(&self, part: &Part) -> f64 {
        part.rot
    }

    /// Copper layer index a surface-mount pad of `part` lies on.
    pub fn part_layer(&self, part: &Part) -> usize {
        match part.side {
            Side::Top => 0,
            Side::Bottom => self.layers - 1,
        }
    }

    pub fn outline(&self) -> [P2; 4] {
        [
            [0.0, 0.0],
            [self.width, 0.0],
            [self.width, self.height],
            [0.0, self.height],
        ]
    }

    /// (refdes, pad name, net, position, layer or None for through) of every connected pad.
    pub fn pads(&self) -> Vec<PlacedPad> {
        let mut out = Vec::new();
        for (pi, part) in self.parts.iter().enumerate() {
            let fp = &self.footprints[part.footprint];
            for (k, pad) in fp.pads.iter().enumerate() {
                out.push(PlacedPad {
                    part: pi,
                    pad: k,
                    net: part.nets[k],
                    pos: self.to_board(part, pad.pos),
                    shape: pad.shape,
                    angle: part.rot,
                    layer: if pad.drill.is_some() {
                        None
                    } else {
                        Some(self.part_layer(part))
                    },
                    drill: pad.drill,
                });
            }
        }
        out
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlacedPad {
    pub part: usize,
    pub pad: usize,
    pub net: Option<usize>,
    pub pos: P2,
    pub shape: PadShape,
    pub angle: f64,
    /// `None` for through-hole pads.
    pub layer: Option<usize>,
    pub drill: Option<f64>,
}
