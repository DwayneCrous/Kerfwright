//! Operations as data: the settings of an operation (kind, region, depths, feeds) in work coordinates, and
//! the function that turns them and a tool into a toolpath. Storing the settings next to the toolpath is
//! what lets an operation be regenerated after the model changes.

#[cfg(feature = "adaptive-clearing")]
use crate::adaptive::{adaptive, AdaptiveParams};
use crate::bvh::TriMesh;
use crate::dexel::TriDexel;
use crate::finish3d::{
    raster_finish, raster_finish_masked, remaining_mask, waterline, RasterParams, WaterlineParams,
};
use crate::geom2d::Region;
use crate::ops2d::{bore, contour, drill, pocket, Cut, Side};
use crate::path::{Event, Toolpath};
use crate::tool::{Tool, ToolKind};
use crate::{P2, P3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OpKind {
    Pocket,
    /// Constant-engagement clearing. The variant stays so saved projects still load, but generating it needs
    /// the `adaptive-clearing` feature, which is off until the patent review in `LICENSES/patent-watch.md`.
    Adaptive,
    ContourOutside,
    Finish3d,
    /// Walls of a region from the inside (finishing the walls of a pocket).
    ContourInside,
    /// Skim the top: clear the whole region from the stock top down to the face.
    Face,
    /// Break the edge of the region's outline with a chamfer mill.
    Chamfer,
    /// Drill the holes at the given points.
    Drill,
    /// Bore holes on a helix with a smaller tool.
    Bore,
    /// Constant-z finishing of the model.
    Waterline,
    /// Finish only where earlier operations left material.
    RestFinish,
    /// Flank milling of the wall of the region with the side of the tool, tool axis along the wall.
    Swarf,
    /// Contour with the tool leaning against the wall (five-axis).
    TiltedContour,
}

impl OpKind {
    /// The operations a user can add. Adaptive clearing is listed only when the feature is on.
    #[cfg(feature = "adaptive-clearing")]
    pub const ALL: [OpKind; 13] = [
        OpKind::Face,
        OpKind::Pocket,
        OpKind::Adaptive,
        OpKind::ContourOutside,
        OpKind::ContourInside,
        OpKind::Chamfer,
        OpKind::Drill,
        OpKind::Bore,
        OpKind::Finish3d,
        OpKind::Waterline,
        OpKind::RestFinish,
        OpKind::Swarf,
        OpKind::TiltedContour,
    ];

    /// The operations a user can add. Adaptive clearing is listed only when the feature is on.
    #[cfg(not(feature = "adaptive-clearing"))]
    pub const ALL: [OpKind; 12] = [
        OpKind::Face,
        OpKind::Pocket,
        OpKind::ContourOutside,
        OpKind::ContourInside,
        OpKind::Chamfer,
        OpKind::Drill,
        OpKind::Bore,
        OpKind::Finish3d,
        OpKind::Waterline,
        OpKind::RestFinish,
        OpKind::Swarf,
        OpKind::TiltedContour,
    ];

    pub fn label(self) -> &'static str {
        match self {
            OpKind::Pocket => "Pocket",
            OpKind::Adaptive => "Adaptive clearing",
            OpKind::ContourOutside => "Contour (outside)",
            OpKind::Finish3d => "3-D finish (ball)",
            OpKind::ContourInside => "Contour (inside)",
            OpKind::Face => "Face",
            OpKind::Chamfer => "Chamfer",
            OpKind::Drill => "Drill",
            OpKind::Bore => "Bore (helical)",
            OpKind::Waterline => "Waterline",
            OpKind::RestFinish => "Rest finish (ball)",
            OpKind::Swarf => "Swarf (5-axis)",
            OpKind::TiltedContour => "Tilted contour (5-axis)",
        }
    }

    /// The operation works from the model's surface.
    pub fn needs_model(self) -> bool {
        matches!(
            self,
            OpKind::Finish3d | OpKind::Waterline | OpKind::RestFinish
        )
    }

    /// The operation needs the stock the earlier operations left.
    pub fn needs_stock(self) -> bool {
        self == OpKind::RestFinish
    }

    /// The operation tilts the tool, which only a machine with rotary axes can do.
    pub fn is_five_axis(self) -> bool {
        matches!(self, OpKind::Swarf | OpKind::TiltedContour)
    }

    /// Does the operation work on points (hole centres) rather than a region?
    pub fn uses_points(self) -> bool {
        matches!(self, OpKind::Drill | OpKind::Bore)
    }
}

/// The area an operation works in, in the XY plane of the work coordinate system.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RegionSpec {
    Rect([f64; 4]),
    /// A polygon (taken from the faces of the model) with islands.
    Polygon {
        outer: Vec<P2>,
        islands: Vec<Vec<P2>>,
    },
}

fn signed_area(p: &[P2]) -> f64 {
    let mut a = 0.0;
    for i in 0..p.len() {
        let (q, r) = (p[i], p[(i + 1) % p.len()]);
        a += q[0] * r[1] - r[0] * q[1];
    }
    0.5 * a
}

impl RegionSpec {
    pub fn region(&self) -> Region {
        match self {
            RegionSpec::Rect([x0, y0, x1, y1]) => Region::rect(*x0, *y0, *x1, *y1),
            RegionSpec::Polygon { outer, islands } => {
                let ccw = |p: &[P2], want_ccw: bool| -> Vec<[f64; 3]> {
                    let mut v: Vec<[f64; 3]> = p.iter().map(|q| [q[0], q[1], 0.0]).collect();
                    if (signed_area(p) > 0.0) != want_ccw {
                        v.reverse();
                    }
                    v
                };
                let mut loops = vec![ccw(outer, true)];
                loops.extend(islands.iter().map(|i| ccw(i, false)));
                Region::from_loops(&loops)
            }
        }
    }

    /// `[x0, y0, x1, y1]`
    pub fn bounds(&self) -> [f64; 4] {
        match self {
            RegionSpec::Rect(r) => *r,
            RegionSpec::Polygon { outer, .. } => {
                let mut b = [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ];
                for p in outer {
                    b[0] = b[0].min(p[0]);
                    b[1] = b[1].min(p[1]);
                    b[2] = b[2].max(p[0]);
                    b[3] = b[3].max(p[1]);
                }
                b
            }
        }
    }

    /// The outer boundary as a counter-clockwise polygon.
    pub fn outline(&self) -> Vec<P2> {
        let mut v = match self {
            RegionSpec::Rect([x0, y0, x1, y1]) => {
                vec![[*x0, *y0], [*x1, *y0], [*x1, *y1], [*x0, *y1]]
            }
            RegionSpec::Polygon { outer, .. } => outer.clone(),
        };
        if signed_area(&v) < 0.0 {
            v.reverse();
        }
        v
    }
}

/// All the settings of one operation. Lengths in mm, coordinates in the work coordinate system. The fields
/// after `tool` belong to particular operations and have defaults.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OpParams {
    pub kind: OpKind,
    pub region: RegionSpec,
    pub top: f64,
    pub bottom: f64,
    pub step_down: f64,
    pub stepover: f64,
    pub feed: f64,
    pub rpm: f64,
    /// Number of the tool in the setup's tool list.
    pub tool: u32,
    /// Hole centres of drilling and boring.
    #[serde(default)]
    pub points: Vec<P2>,
    /// Diameter of the holes a boring operation makes.
    #[serde(default)]
    pub hole_diameter: f64,
    /// Peck depth of drilling (0: a plain cycle).
    #[serde(default)]
    pub peck: f64,
    /// Width of a chamfer on the top surface.
    #[serde(default)]
    pub width: f64,
    /// The tool works on the inside of the outline (chamfer, swarf).
    #[serde(default)]
    pub inside: bool,
    /// Draft of the wall a swarf pass follows (degrees from vertical; positive leans outward going up).
    #[serde(default)]
    pub taper: f64,
    /// How far the tool leans against the wall (degrees).
    #[serde(default)]
    pub tilt: f64,
    /// Rest finishing works where more than this much stock stands over the part.
    #[serde(default = "default_threshold")]
    pub threshold: f64,
    /// Spacing of the cutter-location grid of a waterline pass.
    #[serde(default = "default_grid")]
    pub grid: f64,
}

fn default_threshold() -> f64 {
    0.2
}

fn default_grid() -> f64 {
    0.5
}

impl Default for OpParams {
    fn default() -> Self {
        OpParams {
            kind: OpKind::Pocket,
            region: RegionSpec::Rect([0.0, 0.0, 40.0, 25.0]),
            top: 10.0,
            bottom: 7.0,
            step_down: 3.0,
            stepover: 2.0,
            feed: 800.0,
            rpm: 8000.0,
            tool: 0,
            points: Vec::new(),
            hole_diameter: 0.0,
            peck: 0.0,
            width: 0.5,
            inside: false,
            taper: 0.0,
            tilt: 10.0,
            threshold: default_threshold(),
            grid: default_grid(),
        }
    }
}

/// What a toolpath may need besides its settings and its tool.
#[derive(Default)]
pub struct GenContext<'a> {
    /// The workpiece (3-D operations).
    pub mesh: Option<&'a TriMesh>,
    /// The stock as the earlier operations of the setup left it (rest finishing).
    pub stock: Option<&'a TriDexel>,
}

/// Several paths as one program: the tool is called and the spindle started once at the start and stopped at
/// the end.
pub fn join_paths(name: &str, paths: Vec<Toolpath>) -> Toolpath {
    let mut out = Toolpath::new(name);
    let n = paths.len();
    for (k, p) in paths.into_iter().enumerate() {
        for e in p.events {
            let keep = match &e {
                Event::ToolChange { .. } | Event::SpindleOn { .. } | Event::CoolantOn => k == 0,
                Event::ProgramEnd | Event::SpindleOff | Event::CoolantOff => k + 1 == n,
                _ => true,
            };
            if keep {
                out.events.push(e);
            }
        }
    }
    out
}

/// Polygon points no more than `spacing` apart (closed: the last point is not the first).
fn subdivide(poly: &[P2], spacing: f64) -> Vec<P2> {
    let mut out = Vec::new();
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let n = (((b[0] - a[0]).hypot(b[1] - a[1])) / spacing)
            .ceil()
            .max(1.0) as usize;
        for k in 0..n {
            let t = k as f64 / n as f64;
            out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
        }
    }
    out
}

/// The polygon moved sideways by `d` (outward for a counter-clockwise polygon when positive), at each
/// vertex along the bisector of its two edges.
fn offset_vertices(poly: &[P2], d: f64) -> Vec<P2> {
    let n = poly.len();
    (0..n)
        .map(|i| {
            let (a, b, c) = (poly[(i + n - 1) % n], poly[i], poly[(i + 1) % n]);
            let norm = |p: P2, q: P2| {
                let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
                let l = dx.hypot(dy).max(1e-12);
                [dy / l, -dx / l] // the right-hand normal: outward for a counter-clockwise loop
            };
            let (n1, n2) = (norm(a, b), norm(b, c));
            let m = [n1[0] + n2[0], n1[1] + n2[1]];
            let ml = m[0].hypot(m[1]).max(1e-9);
            let cos_half = (ml / 2.0).clamp(0.3, 1.0);
            [
                b[0] + d * m[0] / ml / cos_half,
                b[1] + d * m[1] / ml / cos_half,
            ]
        })
        .collect()
}

fn unit3(a: P3) -> P3 {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt().max(1e-12);
    [a[0] / n, a[1] / n, a[2] / n]
}

impl OpParams {
    pub fn cut(&self) -> Cut {
        let mut c = Cut::new(self.top, self.bottom);
        c.step_down = self.step_down.max(0.1);
        c.stepover = self.stepover.max(0.05);
        c.feed = self.feed;
        c.rpm = self.rpm;
        c
    }

    /// The toolpath of the operation with `tool`. `mesh` (the workpiece in work coordinates) is used by the
    /// 3-D operations only.
    pub fn generate(&self, tool: &Tool, mesh: Option<&TriMesh>) -> Result<Toolpath, String> {
        self.generate_with(tool, &GenContext { mesh, stock: None })
    }

    pub fn generate_with(&self, tool: &Tool, ctx: &GenContext) -> Result<Toolpath, String> {
        let mut tp = self.generate_raw(tool, ctx)?;
        // the spindle speed is the operation's, whatever the generator wrote
        for e in &mut tp.events {
            match e {
                Event::ToolChange { rpm, .. } | Event::SpindleOn { rpm, .. } => *rpm = self.rpm,
                _ => {}
            }
        }
        Ok(tp)
    }

    fn generate_raw(&self, tool: &Tool, ctx: &GenContext) -> Result<Toolpath, String> {
        let kind = self.kind;
        let needs_mill = !matches!(kind, OpKind::Drill | OpKind::Chamfer);
        if needs_mill && !tool.kind.mills() && kind != OpKind::Bore {
            return Err(format!(
                "a {} does not mill: choose an end mill for {}",
                tool.kind.label().to_lowercase(),
                kind.label().to_lowercase()
            ));
        }
        if kind != OpKind::Drill && kind != OpKind::Bore && self.top <= self.bottom {
            return Err("the top must be above the bottom".into());
        }
        let cut = self.cut();
        let [x0, y0, x1, y1] = self.region.bounds();
        let (w, h) = (x1 - x0, y1 - y0);
        if !kind.uses_points() {
            let region = self.region.region();
            if region.is_empty() {
                return Err("the region is empty".into());
            }
        }
        let region = || self.region.region();
        #[cfg(not(feature = "adaptive-clearing"))]
        if kind == OpKind::Adaptive {
            return Err(
                "adaptive clearing is not available in this build (patent review pending)".into(),
            );
        }
        match kind {
            OpKind::Pocket | OpKind::Adaptive => {
                if w <= tool.diameter || h <= tool.diameter {
                    return Err(format!(
                        "the region {w:.1} × {h:.1} mm is too small for a {} mm tool",
                        tool.diameter
                    ));
                }
                #[cfg(feature = "adaptive-clearing")]
                if kind == OpKind::Adaptive {
                    return Ok(adaptive(&region(), tool, &cut, &AdaptiveParams::default()).0);
                }
                Ok(pocket(&region(), tool, &cut))
            }
            OpKind::ContourOutside => Ok(contour(&region(), tool, Side::Outside, &cut)),
            OpKind::ContourInside => {
                if w <= tool.diameter || h <= tool.diameter {
                    return Err(format!(
                        "the region {w:.1} × {h:.1} mm is too small for a {} mm tool",
                        tool.diameter
                    ));
                }
                Ok(contour(&region(), tool, Side::Inside, &cut))
            }
            OpKind::Face => {
                // the tool centre runs out to the edge of the region, so the cutter clears all of it
                let wide = region().offset(-tool.radius());
                let mut tp = pocket(&wide, tool, &cut);
                tp.name = "face".into();
                Ok(tp)
            }
            OpKind::Chamfer => self.chamfer(tool),
            OpKind::Drill => {
                if !matches!(tool.kind, ToolKind::Drill { .. }) {
                    return Err("drilling needs a drill".into());
                }
                if self.points.is_empty() {
                    return Err("there are no holes to drill".into());
                }
                let peck = (self.peck > 0.0).then_some(self.peck);
                Ok(drill(&self.points, tool, &cut, peck))
            }
            OpKind::Bore => {
                if self.points.is_empty() {
                    return Err("there are no holes to bore".into());
                }
                if self.top <= self.bottom {
                    return Err("the top must be above the bottom".into());
                }
                let mut paths = Vec::new();
                for c in &self.points {
                    let Some(p) = bore(*c, self.hole_diameter, tool, &cut) else {
                        return Err(format!(
                            "a {} mm tool does not fit a {} mm hole",
                            tool.diameter, self.hole_diameter
                        ));
                    };
                    paths.push(p);
                }
                Ok(join_paths("bore", paths))
            }
            OpKind::Finish3d | OpKind::RestFinish => {
                let Some(mesh) = ctx.mesh else {
                    return Err("there is no solid to finish".into());
                };
                if tool.kind != ToolKind::Ball {
                    return Err("3-D finishing here uses a ball-nose tool".into());
                }
                let p = RasterParams {
                    region: [x0, y0, x1, y1],
                    stepover: self.stepover.max(0.2),
                    step: 1.0,
                    tolerance: 0.01,
                    min_segment: 0.05,
                    floor: self.bottom.min(self.top) - 1.0,
                    safe_z: self.top + 10.0,
                    feed: self.feed,
                    plunge_feed: (self.feed * 0.4).max(50.0),
                };
                if kind == OpKind::Finish3d {
                    return Ok(raster_finish(mesh, tool, &p));
                }
                let Some(stock) = ctx.stock else {
                    return Err("rest finishing needs the stock the earlier operations left".into());
                };
                let mask = remaining_mask(stock, mesh, tool, self.threshold.max(0.0), p.region);
                if mask.count() == 0 {
                    return Err(format!(
                        "no stock stands more than {} mm over the part: nothing is left to finish",
                        self.threshold
                    ));
                }
                Ok(raster_finish_masked(mesh, tool, &p, Some(&mask)))
            }
            OpKind::Waterline => {
                let Some(mesh) = ctx.mesh else {
                    return Err("there is no solid to finish".into());
                };
                if !matches!(
                    tool.kind,
                    ToolKind::Ball | ToolKind::Bull { .. } | ToolKind::Flat
                ) {
                    return Err("waterline finishing uses a ball, bull-nose or flat tool".into());
                }
                let p = WaterlineParams {
                    region: [x0, y0, x1, y1],
                    top: self.top,
                    bottom: self.bottom,
                    step_down: self.step_down.max(0.05),
                    grid: self.grid.max(0.05),
                    safe_z: self.top + 10.0,
                    feed: self.feed,
                    plunge_feed: (self.feed * 0.4).max(50.0),
                };
                Ok(waterline(mesh, tool, &p))
            }
            OpKind::Swarf => self.swarf(tool),
            OpKind::TiltedContour => self.tilted_contour(tool),
        }
    }

    /// A chamfer of `width` on the top edge of the outline with a V-shaped tool. The tool centre runs the
    /// outline offset by the radius of the tool's flat tip, at the depth where the cone's radius at the top
    /// surface is the tip radius plus the width.
    fn chamfer(&self, tool: &Tool) -> Result<Toolpath, String> {
        let (angle, tip) = match tool.kind {
            ToolKind::Chamfer {
                tip_angle,
                tip_diameter,
            }
            | ToolKind::Engraver {
                tip_angle,
                tip_diameter,
            } => (tip_angle, tip_diameter),
            _ => return Err("chamfering needs a chamfer mill or an engraver".into()),
        };
        if self.width <= 0.0 {
            return Err("the chamfer width must be positive".into());
        }
        let rt = tip / 2.0;
        let tan_half = (angle / 2.0).clamp(1.0, 89.0).to_radians().tan();
        if rt + self.width > tool.radius() + 1e-9 {
            return Err(format!(
                "a {} mm tool cannot cut a {} mm chamfer (its largest radius is {} mm)",
                tool.diameter,
                self.width,
                tool.radius()
            ));
        }
        let z_tip = self.top - self.width / tan_half;
        // path of a flat tool as wide as the tip, on one level
        let pseudo = Tool::flat(tool.number, (2.0 * rt).max(0.02), tool.flute_length);
        let mut cut = self.cut();
        cut.top = z_tip + 0.5;
        cut.bottom = z_tip;
        cut.step_down = 1.0;
        cut.clearance = (self.top - z_tip) + 5.0;
        let side = if self.inside {
            Side::Inside
        } else {
            Side::Outside
        };
        let mut tp = contour(&self.region.region(), &pseudo, side, &cut);
        tp.name = "chamfer".into();
        for e in &mut tp.events {
            if let Event::ToolChange { name, diameter, .. } = e {
                *name = tool.name.clone();
                *diameter = tool.diameter;
            }
        }
        Ok(tp)
    }

    /// Flank milling along the outline from `bottom` to `top`: the side of the tool lies along the ruling
    /// of the wall, which leans by `taper` degrees.
    fn swarf(&self, tool: &Tool) -> Result<Toolpath, String> {
        use crate::multiaxis::{swarf_poses, Side as MSide};
        if !tool.kind.mills() {
            return Err("swarf milling needs an end mill".into());
        }
        let height = self.top - self.bottom;
        let lean = self.taper.to_radians();
        let length = height / lean.cos();
        if length > tool.flute_length + 1e-9 {
            return Err(format!(
                "the wall is {length:.1} mm along the ruling but the flutes are {} mm long",
                tool.flute_length
            ));
        }
        let outline = subdivide(&self.region.outline(), 1.0);
        let shift = height * lean.tan();
        let top_pts = offset_vertices(&outline, shift);
        let bottom: Vec<P3> = outline.iter().map(|p| [p[0], p[1], self.bottom]).collect();
        let top: Vec<P3> = top_pts.iter().map(|p| [p[0], p[1], self.top]).collect();
        // close the loop
        let (mut b, mut t) = (bottom, top);
        b.push(b[0]);
        t.push(t[0]);
        let side = if self.inside {
            MSide::Left
        } else {
            MSide::Right
        };
        let poses = swarf_poses(&b, &t, tool.radius(), side);
        let mut tp = Toolpath::new("swarf");
        tp.push(Event::ToolChange {
            number: tool.number,
            name: tool.name.clone(),
            diameter: tool.diameter,
            rpm: self.rpm,
        });
        tp.push(Event::SpindleOn {
            rpm: self.rpm,
            clockwise: true,
        });
        tp.push(Event::CoolantOn);
        // approach and retreat along the tool axis: the cut is a slot as slanted as the tool
        let first = poses[0];
        let away = |p: &crate::kin::Pose| {
            let a = unit3(p.axis);
            let l = height + 5.0;
            [
                p.tip[0] + a[0] * l,
                p.tip[1] + a[1] * l,
                p.tip[2] + a[2] * l,
            ]
        };
        tp.push(Event::Pose {
            p: away(&first),
            axis: first.axis,
            feed: None,
        });
        tp.push(Event::Pose {
            p: first.tip,
            axis: first.axis,
            feed: Some((self.feed * 0.4).max(50.0)),
        });
        for p in &poses[1..] {
            tp.push(Event::Pose {
                p: p.tip,
                axis: p.axis,
                feed: Some(self.feed),
            });
        }
        let last = poses[poses.len() - 1];
        tp.push(Event::Pose {
            p: away(&last),
            axis: last.axis,
            feed: None,
        });
        tp.push(Event::CoolantOff);
        tp.push(Event::SpindleOff);
        tp.push(Event::ProgramEnd);
        Ok(tp)
    }

    /// A contour with the tool leaning against the wall by `tilt` degrees: the tip runs at the distance where
    /// the tilted cylinder touches the wall.
    fn tilted_contour(&self, tool: &Tool) -> Result<Toolpath, String> {
        if !tool.kind.mills() {
            return Err("a tilted contour needs an end mill".into());
        }
        let tilt = self.tilt.to_radians();
        if tilt.abs() > 80f64.to_radians() {
            return Err("the tool cannot lean more than 80°".into());
        }
        // across the wall the tilted cylinder is as wide as radius / cos(tilt)
        let d = tool.radius() / tilt.cos();
        let pseudo = Tool::flat(tool.number, 2.0 * d, tool.flute_length);
        let cut = self.cut();
        let flat = contour(&self.region.region(), &pseudo, Side::Outside, &cut);
        let mut tp = Toolpath::new("tilted contour");
        let up = [0.0, 0.0, 1.0];
        let mut pos: Option<P3> = None;
        // direction of travel at each move, for the lean
        let leaning = |from: P3, to: P3| -> P3 {
            let (tx, ty) = (to[0] - from[0], to[1] - from[1]);
            let l = tx.hypot(ty);
            if l < 1e-9 {
                return up;
            }
            // material is on the left of the contour loops, outer boundary and islands alike
            let left = [-ty / l, tx / l];
            unit3([left[0] * tilt.sin(), left[1] * tilt.sin(), tilt.cos()])
        };
        let mut lean = up;
        // the plunge of a pass takes the lean of the move that follows it
        let mut plunge: Option<usize> = None;
        for e in &flat.events {
            match e {
                Event::Rapid { p } => {
                    tp.push(Event::Pose {
                        p: *p,
                        axis: if pos.is_some_and(|q| q[2] <= p[2]) {
                            lean
                        } else {
                            up
                        },
                        feed: None,
                    });
                    pos = Some(*p);
                }
                Event::Feed { p, feed } => {
                    let mut moved = false;
                    if let Some(q) = pos {
                        if (q[0] - p[0]).hypot(q[1] - p[1]) > 1e-9 {
                            lean = leaning(q, *p);
                            moved = true;
                        }
                    }
                    if moved {
                        if let Some(i) = plunge.take() {
                            if let Event::Pose { axis, .. } = &mut tp.events[i] {
                                *axis = lean;
                            }
                        }
                    } else {
                        plunge = Some(tp.events.len());
                    }
                    tp.push(Event::Pose {
                        p: *p,
                        axis: lean,
                        feed: Some(*feed),
                    });
                    pos = Some(*p);
                }
                Event::Arc {
                    p,
                    centre,
                    ccw,
                    feed,
                } => {
                    if let Some(q) = pos {
                        let r = (q[0] - centre[0]).hypot(q[1] - centre[1]);
                        let a0 = (q[1] - centre[1]).atan2(q[0] - centre[0]);
                        let a1 = (p[1] - centre[1]).atan2(p[0] - centre[0]);
                        let mut sweep = a1 - a0;
                        if *ccw {
                            while sweep <= 1e-12 {
                                sweep += std::f64::consts::TAU;
                            }
                        } else {
                            while sweep >= -1e-12 {
                                sweep -= std::f64::consts::TAU;
                            }
                        }
                        let n = ((sweep.abs() * r / 0.2).ceil() as usize).max(2);
                        let mut prev = q;
                        for k in 1..=n {
                            let t = k as f64 / n as f64;
                            let a = a0 + sweep * t;
                            let pt = [
                                centre[0] + r * a.cos(),
                                centre[1] + r * a.sin(),
                                q[2] + (p[2] - q[2]) * t,
                            ];
                            lean = leaning(prev, pt);
                            tp.push(Event::Pose {
                                p: pt,
                                axis: lean,
                                feed: Some(*feed),
                            });
                            prev = pt;
                        }
                    }
                    pos = Some(*p);
                }
                other @ (Event::ToolChange { .. }
                | Event::SpindleOn { .. }
                | Event::CoolantOn
                | Event::CoolantOff
                | Event::SpindleOff
                | Event::ProgramEnd) => {
                    if let Event::ToolChange { .. } = other {
                        tp.push(Event::ToolChange {
                            number: tool.number,
                            name: tool.name.clone(),
                            diameter: tool.diameter,
                            rpm: self.rpm,
                        });
                    } else {
                        tp.push(other.clone());
                    }
                }
                _ => {}
            }
        }
        Ok(tp)
    }
}
