use crate::{P2, P3};
use serde::{Deserialize, Serialize};

/// A cutter-location event. Positions are the tool-tip centre in the work coordinate system.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Event {
    Comment(String),
    ToolChange {
        number: u32,
        name: String,
        diameter: f64,
        rpm: f64,
    },
    SpindleOn {
        rpm: f64,
        clockwise: bool,
    },
    SpindleOff,
    CoolantOn,
    CoolantOff,
    Rapid {
        p: P3,
    },
    Feed {
        p: P3,
        feed: f64,
    },
    /// Arc in the XY plane (helical if `p[2]` differs from the start height).
    Arc {
        p: P3,
        centre: P2,
        ccw: bool,
        feed: f64,
    },
    /// Five-axis positioning: X Y Z and two rotary axes (degrees) named by `names`, e.g. `['A', 'C']`. With
    /// tool-centre-point control on, `p` is the tool tip in workpiece coordinates; otherwise it holds the
    /// linear axes of the machine.
    Rapid5 {
        p: P3,
        rot: [f64; 2],
        names: [char; 2],
    },
    Feed5 {
        p: P3,
        rot: [f64; 2],
        names: [char; 2],
        feed: f64,
    },
    /// Switch the controller's tool-centre-point control on or off.
    Tcp {
        on: bool,
    },
    /// Define a drilling cycle (absolute depth and retract plane, surface for reference).
    DrillStart {
        depth: f64,
        retract: f64,
        surface: f64,
        peck: Option<f64>,
        feed: f64,
    },
    DrillAt {
        x: f64,
        y: f64,
    },
    DrillEnd,
    ProgramEnd,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Toolpath {
    pub name: String,
    pub events: Vec<Event>,
}

impl Toolpath {
    pub fn new(name: &str) -> Toolpath {
        Toolpath {
            name: name.into(),
            events: Vec::new(),
        }
    }

    pub fn push(&mut self, e: Event) {
        self.events.push(e);
    }

    pub fn extend(&mut self, other: Toolpath) {
        self.events.extend(other.events);
    }

    /// Cutting length (feed and arc moves) in mm.
    pub fn cutting_length(&self) -> f64 {
        let mut pos: Option<P3> = None;
        let mut len = 0.0;
        for e in &self.events {
            match e {
                Event::Rapid { p } => pos = Some(*p),
                Event::Feed { p, .. } => {
                    if let Some(q) = pos {
                        len +=
                            ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2))
                                .sqrt();
                    }
                    pos = Some(*p);
                }
                Event::Arc { p, centre, .. } => {
                    if let Some(q) = pos {
                        let r = (q[0] - centre[0]).hypot(q[1] - centre[1]);
                        let a0 = (q[1] - centre[1]).atan2(q[0] - centre[0]);
                        let a1 = (p[1] - centre[1]).atan2(p[0] - centre[0]);
                        let mut d = a1 - a0;
                        if let Event::Arc { ccw, .. } = e {
                            if *ccw {
                                while d <= 1e-12 {
                                    d += std::f64::consts::TAU;
                                }
                            } else {
                                while d >= -1e-12 {
                                    d -= std::f64::consts::TAU;
                                }
                            }
                        }
                        len += (r * d.abs()).hypot(p[2] - q[2]);
                    }
                    pos = Some(*p);
                }
                _ => {}
            }
        }
        len
    }
}
