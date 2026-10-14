//! The settings of the operation being edited.

use kw_cam::library::{with_holder, HolderDef};
use kw_cam::{OpKind, OpParams, RegionSpec, Tool, ToolKind};

/// The quick tools: a plain end mill of a given diameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolChoice {
    Flat,
    Ball,
    Bull,
}

/// All the settings of one operation, as text fields.
#[derive(Debug, Clone)]
pub struct Settings {
    pub op: OpKind,
    pub tool: ToolChoice,
    pub diameter: f64,
    pub corner: f64,
    /// A tool from the library (or the setup) that replaces the quick tool.
    pub preset: Option<Tool>,
    /// A holder from the library for the tool.
    pub holder: Option<HolderDef>,
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
    /// A region taken from the outline of a face; when set it replaces the rectangle.
    pub region: Option<RegionSpec>,
    pub top: f64,
    pub bottom: f64,
    pub step_down: f64,
    pub stepover: f64,
    pub feed: f64,
    pub rpm: f64,
    /// Hole centres (drilling, boring).
    pub points: Vec<[f64; 2]>,
    pub hole_diameter: f64,
    pub peck: f64,
    pub width: f64,
    pub inside: bool,
    pub taper: f64,
    pub tilt: f64,
    pub threshold: f64,
    pub grid: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            op: OpKind::Pocket,
            tool: ToolChoice::Flat,
            diameter: 6.0,
            corner: 1.0,
            preset: None,
            holder: None,
            x0: 0.0,
            y0: 0.0,
            x1: 40.0,
            y1: 25.0,
            region: None,
            top: 10.0,
            bottom: 7.0,
            step_down: 3.0,
            stepover: 2.0,
            feed: 800.0,
            rpm: 8000.0,
            points: Vec::new(),
            hole_diameter: 10.0,
            peck: 0.0,
            width: 0.5,
            inside: false,
            taper: 0.0,
            tilt: 10.0,
            threshold: 0.2,
            grid: 0.5,
        }
    }
}

impl Settings {
    /// The tool these settings describe (its number is given by the setup it joins).
    pub fn tool(&self) -> Tool {
        let t = match &self.preset {
            Some(t) => t.clone(),
            None => match self.tool {
                ToolChoice::Flat => Tool::flat(1, self.diameter, 25.0),
                ToolChoice::Ball => Tool::ball(1, self.diameter, 25.0),
                ToolChoice::Bull => {
                    Tool::bull(1, self.diameter, self.corner.min(self.diameter / 2.0), 25.0)
                }
            },
        };
        match &self.holder {
            Some(h) => with_holder(&t, h),
            None => t,
        }
    }

    /// Use a plain end mill again instead of a library tool.
    pub fn use_quick_tool(&mut self) {
        self.preset = None;
    }

    pub fn region_spec(&self) -> RegionSpec {
        self.region
            .clone()
            .unwrap_or(RegionSpec::Rect([self.x0, self.y0, self.x1, self.y1]))
    }

    pub fn params(&self) -> OpParams {
        OpParams {
            kind: self.op,
            region: self.region_spec(),
            top: self.top,
            bottom: self.bottom,
            step_down: self.step_down,
            stepover: self.stepover,
            feed: self.feed,
            rpm: self.rpm,
            tool: 0,
            points: self.points.clone(),
            hole_diameter: self.hole_diameter,
            peck: self.peck,
            width: self.width,
            inside: self.inside,
            taper: self.taper,
            tilt: self.tilt,
            threshold: self.threshold,
            grid: self.grid,
        }
    }

    /// Take over the settings of a stored operation.
    pub fn load(&mut self, p: &OpParams, tool: Option<&Tool>) {
        self.op = p.kind;
        let b = p.region.bounds();
        (self.x0, self.y0, self.x1, self.y1) = (b[0], b[1], b[2], b[3]);
        self.region = matches!(p.region, RegionSpec::Polygon { .. }).then(|| p.region.clone());
        self.top = p.top;
        self.bottom = p.bottom;
        self.step_down = p.step_down;
        self.stepover = p.stepover;
        self.feed = p.feed;
        self.rpm = p.rpm;
        self.points = p.points.clone();
        self.hole_diameter = p.hole_diameter;
        self.peck = p.peck;
        self.width = p.width;
        self.inside = p.inside;
        self.taper = p.taper;
        self.tilt = p.tilt;
        self.threshold = p.threshold;
        self.grid = p.grid;
        if let Some(t) = tool {
            self.diameter = t.diameter;
            self.holder = None;
            match t.kind {
                ToolKind::Flat => {
                    self.tool = ToolChoice::Flat;
                    self.preset = None;
                }
                ToolKind::Ball => {
                    self.tool = ToolChoice::Ball;
                    self.preset = None;
                }
                ToolKind::Bull { corner_radius } => {
                    self.tool = ToolChoice::Bull;
                    self.corner = corner_radius;
                    self.preset = None;
                }
                _ => self.preset = Some(t.clone()),
            }
            // a quick tool with a holder or other details the quick tools lack stays as it was made
            if self.preset.is_none() && *t != self.tool() {
                self.preset = Some(t.clone());
            }
        }
    }
}
