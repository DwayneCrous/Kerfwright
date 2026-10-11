//! Toolkit-independent sketch editor: tools, selection, snapping and inference,
//! undo/redo, dragging. A UI feeds it [`Input`] events in sketch coordinates and
//! draws `editor.sketch`; nothing here depends on a windowing or GUI library, so
//! whole interaction sequences can be scripted in tests.

use crate::analysis::{DofReport, SketchStatus};
use crate::constraint::{ConstraintId, ConstraintKind};
use crate::entity::{EntityId, EntityKind, VarId};
use crate::geom::{self, closest_point, P2};
use crate::ops;
use crate::sketch::{Sketch, SketchError};
use crate::solve::Solver;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstraintTool {
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    Fix,
    Midpoint,
    Concentric,
    Collinear,
    PointOnCurve,
    Symmetric,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    Select,
    Point,
    Line,
    Rectangle,
    /// Rectangle from its centre outwards: click the centre, then a corner.
    CenterRectangle,
    /// Rectangle at any angle: click the two ends of one side, then a point for the width.
    AlignedRectangle,
    Circle,
    Arc,
    Spline,
    /// Spline through its points (as opposed to `Spline`, whose points are control points).
    FitSpline,
    Polygon,
    Slot,
    Ellipse,
    /// Arc through three points: start, end, a point on it.
    Arc3,
    /// Circle through three points.
    Circle3,
    Dimension,
    Constrain(ConstraintTool),
    Trim,
    Extend,
    Offset,
    /// Round the corner of two lines (click the shared point).
    Fillet,
    /// Bevel the corner of two lines (click the shared point).
    Chamfer,
    /// Mirror the picked entities across a line (pick entities, Enter, then the line).
    Mirror,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Input {
    Move(P2),
    Click(P2),
    /// Finish the current multi-click operation (Enter / double-click).
    Finish,
    Escape,
    /// Delete the selection.
    Delete,
}

/// One inference line behind a [`Snap::Track`]: where it starts and what it means.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ray {
    /// Horizontal through `from`. `anchor`: from the start of the line being drawn (the line gets a
    /// horizontal constraint); otherwise an alignment with an acquired point (geometry only).
    Horizontal {
        from: P2,
        anchor: bool,
    },
    Vertical {
        from: P2,
        anchor: bool,
    },
    /// A multiple of the polar angle step (degrees, counter-clockwise from +x).
    Angle {
        from: P2,
        deg: f64,
    },
    Parallel {
        from: P2,
        line: EntityId,
    },
    Perpendicular {
        from: P2,
        line: EntityId,
    },
}

impl Ray {
    pub fn from(&self) -> P2 {
        match *self {
            Ray::Horizontal { from, .. }
            | Ray::Vertical { from, .. }
            | Ray::Angle { from, .. }
            | Ray::Parallel { from, .. }
            | Ray::Perpendicular { from, .. } => from,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Snap {
    Point(EntityId, P2),
    OnCurve(EntityId, P2),
    /// The sketch origin (a fixed point is made there when the snap is used).
    Origin,
    Midpoint(EntityId, P2),
    Intersection(EntityId, EntityId, P2),
    /// On one inference line, or at the crossing of two.
    Track(P2, Ray, Option<Ray>),
    Grid(P2),
    Free(P2),
}

impl Snap {
    pub fn position(&self) -> P2 {
        match self {
            Snap::Point(_, p)
            | Snap::OnCurve(_, p)
            | Snap::Midpoint(_, p)
            | Snap::Intersection(_, _, p)
            | Snap::Track(p, ..)
            | Snap::Grid(p)
            | Snap::Free(p) => *p,
            Snap::Origin => (0.0, 0.0),
        }
    }

    /// Short name for status text and tooltips.
    pub fn name(&self) -> &'static str {
        match self {
            Snap::Point(..) => "Point",
            Snap::OnCurve(..) => "On curve",
            Snap::Origin => "Origin",
            Snap::Midpoint(..) => "Midpoint",
            Snap::Intersection(..) => "Intersection",
            Snap::Track(_, r, r2) => {
                let a = ray_name(r);
                match r2 {
                    Some(_) => "Aligned",
                    None => a,
                }
            }
            Snap::Grid(_) => "Grid",
            Snap::Free(_) => "",
        }
    }

    /// The inference lines to draw for this snap: `(start, end, ray)`.
    pub fn track_lines(&self) -> Vec<(P2, P2, Ray)> {
        match self {
            Snap::Track(p, r1, r2) => [Some(*r1), *r2]
                .into_iter()
                .flatten()
                .map(|r| (r.from(), *p, r))
                .collect(),
            _ => Vec::new(),
        }
    }
}

fn ray_name(r: &Ray) -> &'static str {
    match r {
        Ray::Horizontal { anchor: true, .. } => "Horizontal",
        Ray::Vertical { anchor: true, .. } => "Vertical",
        Ray::Horizontal { .. } | Ray::Vertical { .. } => "Aligned",
        Ray::Angle { .. } => "Angle",
        Ray::Parallel { .. } => "Parallel",
        Ray::Perpendicular { .. } => "Perpendicular",
    }
}

/// Which snaps and inferences are active.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnapSettings {
    pub points: bool,
    pub origin: bool,
    pub midpoints: bool,
    pub intersections: bool,
    pub on_curve: bool,
    /// Horizontal / vertical inference of a line's end within 2 degrees.
    pub infer_hv: bool,
    /// Dashed alignment with acquired points and horizontal / vertical from the line start (pixel tolerance).
    pub tracking: bool,
    /// Snap the direction of a line to multiples of this many degrees.
    pub polar: Option<f64>,
    /// Infer parallel / perpendicular to existing lines.
    pub parallel_perp: bool,
    pub grid: bool,
    pub grid_step: f64,
}

impl Default for SnapSettings {
    /// The behaviour of the first editor: points, on-curve and the 2 degree horizontal / vertical inference.
    fn default() -> Self {
        Self {
            points: true,
            origin: false,
            midpoints: false,
            intersections: false,
            on_curve: true,
            infer_hv: true,
            tracking: false,
            polar: None,
            parallel_perp: false,
            grid: false,
            grid_step: 1.0,
        }
    }
}

impl SnapSettings {
    /// Everything on, as in Fusion 360.
    pub fn fusion() -> Self {
        Self {
            points: true,
            origin: true,
            midpoints: true,
            intersections: true,
            on_curve: true,
            infer_hv: true,
            tracking: true,
            polar: Some(15.0),
            parallel_perp: true,
            grid: true,
            grid_step: 1.0,
        }
    }
}

/// Largest angular error for a direction snap (polar, parallel, perpendicular), degrees.
const DIRECTION_SNAP_DEG: f64 = 3.0;
/// Most points that can be acquired for alignment at once.
const MAX_ACQUIRED: usize = 3;

#[derive(Debug, Clone, Default)]
enum Pending {
    #[default]
    None,
    Line(EntityId),
    Rect(Snap),
    /// Points picked so far for an aligned rectangle (one or two).
    ARect(Vec<Snap>),
    Circle(EntityId),
    ArcCentre(EntityId),
    ArcStart(EntityId, EntityId),
    Spline(Vec<EntityId>),
    FitSpline(Vec<EntityId>),
    Polygon(EntityId),
    Slot1(EntityId),
    Slot2(EntityId, EntityId),
    Ellipse1(EntityId),
    Ellipse2(EntityId, EntityId),
    /// Points picked so far for a three-point arc or circle.
    Three(Vec<Snap>),
    Collect(Vec<EntityId>),
    OffsetSide(Vec<EntityId>),
    MirrorAxis(Vec<EntityId>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityState {
    /// Fully determined.
    Constrained,
    /// Can still move.
    Free,
    /// Takes part in redundant constraints.
    Redundant,
    /// Takes part in conflicting constraints.
    Conflicting,
}

pub struct Editor {
    pub sketch: Sketch,
    pub tool: Tool,
    pub selection: Vec<EntityId>,
    /// Pick / snap radius in sketch units (set from the view scale by the UI).
    pub tolerance: f64,
    /// Distance used by the offset tool.
    pub offset_distance: f64,
    /// Number of sides made by the polygon tool.
    pub polygon_sides: usize,
    /// Radius of the sketch fillet tool, and distance of the chamfer tool.
    pub corner_size: f64,
    /// Add to the selection instead of replacing it (Ctrl held).
    pub multi_select: bool,
    pub status: String,
    pub report: Option<DofReport>,
    /// Dimension created by the last command (the UI may open a value editor).
    pub last_dimension: Option<ConstraintId>,
    pub cursor: P2,
    pub snaps: SnapSettings,
    /// Snapping is switched off for the current action (Alt held).
    pub suspend_snaps: bool,
    /// Points the cursor has dwelt on; their horizontal / vertical lines are offered as alignments.
    acquired: Vec<EntityId>,
    solver: Solver,
    undo: Vec<Sketch>,
    redo: Vec<Sketch>,
    pending: Pending,
    drag: Option<DragState>,
}

struct DragState {
    /// Variables moved with the cursor, each with its offset from the grab point.
    vars: Vec<(VarId, bool, f64)>, // (var, is_x, base)
    grab: P2,
}

fn dist(a: P2, b: P2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

impl Default for Editor {
    fn default() -> Self {
        Self::new(Sketch::new())
    }
}

impl Editor {
    pub fn new(sketch: Sketch) -> Self {
        Self {
            sketch,
            tool: Tool::Select,
            selection: Vec::new(),
            tolerance: 0.3,
            offset_distance: 1.0,
            polygon_sides: 6,
            corner_size: 2.0,
            multi_select: false,
            status: String::new(),
            report: None,
            last_dimension: None,
            cursor: (0.0, 0.0),
            snaps: SnapSettings::default(),
            suspend_snaps: false,
            acquired: Vec::new(),
            solver: Solver::default(),
            undo: Vec::new(),
            redo: Vec::new(),
            pending: Pending::None,
            drag: None,
        }
    }

    pub fn set_tool(&mut self, tool: Tool) {
        self.tool = tool;
        self.pending = Pending::None;
        self.acquired.clear();
        self.status.clear();
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    fn checkpoint(&mut self) {
        self.undo.push(self.sketch.clone());
        self.redo.clear();
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
    }

    pub fn undo(&mut self) {
        if let Some(prev) = self.undo.pop() {
            self.redo.push(std::mem::replace(&mut self.sketch, prev));
            self.selection.clear();
            self.pending = Pending::None;
            self.refresh();
        }
    }

    pub fn redo(&mut self) {
        if let Some(next) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.sketch, next));
            self.selection.clear();
            self.pending = Pending::None;
            self.refresh();
        }
    }

    /// Solve and re-analyse; updates `report`.
    pub fn refresh(&mut self) {
        match self.solver.analyze(&mut self.sketch) {
            Ok(r) => self.report = Some(r),
            Err(e) => self.status = format!("solver error: {e}"),
        }
    }

    /// Solve after a command that added constraints; if that made the sketch
    /// conflicting the command is rolled back.
    fn refresh_or_rollback(&mut self) -> bool {
        self.refresh();
        if self
            .report
            .as_ref()
            .is_some_and(|r| r.status == SketchStatus::Conflicting)
        {
            self.status = "rejected: the constraint conflicts with existing ones".into();
            if let Some(prev) = self.undo.pop() {
                self.sketch = prev;
            }
            self.pending = Pending::None;
            self.refresh();
            return false;
        }
        true
    }

    /// How an entity should be coloured.
    pub fn entity_state(&self, e: EntityId) -> EntityState {
        let Some(r) = &self.report else {
            return EntityState::Free;
        };
        let hit = |ids: &[ConstraintId]| {
            ids.iter().any(|c| {
                self.sketch
                    .constraints
                    .get(*c)
                    .is_some_and(|c| c.kind.entities().contains(&e))
            })
        };
        if hit(&r.conflicting) {
            EntityState::Conflicting
        } else if hit(&r.redundant) {
            EntityState::Redundant
        } else if r.free_entities.contains(&e)
            || self
                .sketch
                .children(e)
                .iter()
                .any(|c| r.free_entities.contains(c))
        {
            EntityState::Free
        } else {
            EntityState::Constrained
        }
    }

    // ------------------------------------------------------------- snapping

    /// Remember a point for alignment (the UI calls this when the cursor has rested on it).
    pub fn acquire(&mut self, point: EntityId) {
        if self.acquired.contains(&point)
            || !matches!(self.sketch.kind(point), Ok(EntityKind::Point(_)))
        {
            return;
        }
        if self.acquired.len() == MAX_ACQUIRED {
            self.acquired.remove(0);
        }
        self.acquired.push(point);
    }

    pub fn acquired_points(&self) -> Vec<P2> {
        self.acquired
            .iter()
            .filter_map(|e| self.sketch.point(*e).ok())
            .collect()
    }

    /// Where a click at `p` lands, in order of preference: existing point, origin, midpoint of a line,
    /// crossing of two curves, a curve, inference lines, grid node.
    pub fn snap(&self, p: P2) -> Snap {
        if self.suspend_snaps {
            return Snap::Free(p);
        }
        let cfg = self.snaps;
        let sk = &self.sketch;
        if cfg.points {
            let mut best: Option<(f64, EntityId, P2)> = None;
            for (id, e) in &sk.entities {
                if let EntityKind::Point(_) = e.kind {
                    let q = sk.point(id).unwrap();
                    let d = dist(p, q);
                    if d <= self.tolerance && best.is_none_or(|(bd, _, _)| d < bd) {
                        best = Some((d, id, q));
                    }
                }
            }
            if let Some((_, id, q)) = best {
                return Snap::Point(id, q);
            }
        }
        if cfg.origin && dist(p, (0.0, 0.0)) <= self.tolerance {
            return Snap::Origin;
        }
        if cfg.midpoints {
            let mut best: Option<(f64, EntityId, P2)> = None;
            for (id, e) in &sk.entities {
                if let EntityKind::Line { p1, p2 } = e.kind {
                    let (Ok(a), Ok(b)) = (sk.point(p1), sk.point(p2)) else {
                        continue;
                    };
                    let m = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
                    let d = dist(p, m);
                    if d <= self.tolerance && best.is_none_or(|(bd, _, _)| d < bd) {
                        best = Some((d, id, m));
                    }
                }
            }
            if let Some((_, id, m)) = best {
                return Snap::Midpoint(id, m);
            }
        }
        if cfg.intersections {
            let near: Vec<EntityId> = sk
                .entities
                .iter()
                .filter(|(_, e)| !matches!(e.kind, EntityKind::Point(_)))
                .filter(|(id, _)| {
                    geom::distance_to(sk, *id, p).is_ok_and(|d| d <= 3.0 * self.tolerance)
                })
                .map(|(id, _)| id)
                .collect();
            let mut best: Option<(f64, EntityId, EntityId, P2)> = None;
            for (i, &a) in near.iter().enumerate() {
                for &b in &near[i + 1..] {
                    let Ok(xs) = geom::intersections(sk, a, b) else {
                        continue;
                    };
                    for q in xs {
                        let d = dist(p, q);
                        if d <= self.tolerance && best.is_none_or(|(bd, ..)| d < bd) {
                            best = Some((d, a, b, q));
                        }
                    }
                }
            }
            if let Some((_, a, b, q)) = best {
                return Snap::Intersection(a, b, q);
            }
        }
        if cfg.on_curve {
            if let Some(c) = geom::pick(sk, p, self.tolerance) {
                if !matches!(sk.kind(c), Ok(EntityKind::Point(_))) {
                    if let Ok(q) = closest_point(sk, c, p) {
                        return Snap::OnCurve(c, q);
                    }
                }
            }
        }
        if let Some(t) = self.track(p) {
            return t;
        }
        if cfg.grid && cfg.grid_step > 0.0 {
            let g = cfg.grid_step;
            return Snap::Grid(((p.0 / g).round() * g, (p.1 / g).round() * g));
        }
        Snap::Free(p)
    }

    /// Start of the line being drawn, if any.
    fn line_anchor(&self) -> Option<P2> {
        match &self.pending {
            Pending::Line(a) => self.sketch.point(*a).ok(),
            _ => None,
        }
    }

    /// Inference lines near `p`: directions from the start of the line being drawn and alignments with
    /// acquired points. The nearest wins; a second one that crosses it close to the cursor gives a
    /// two-line snap at the crossing.
    fn track(&self, p: P2) -> Option<Snap> {
        struct Cand {
            ray: Ray,
            from: P2,
            dir: P2,
            d: f64,
            anchor: bool,
        }
        let cfg = self.snaps;
        let mut cands: Vec<Cand> = Vec::new();
        if let Some(a) = self.line_anchor() {
            let (dx, dy) = (p.0 - a.0, p.1 - a.1);
            let len = dx.hypot(dy);
            let pixel_mode = cfg.tracking || cfg.polar.is_some() || cfg.parallel_perp;
            if len > 1e-9 && (cfg.infer_hv || pixel_mode) {
                let theta = dy.atan2(dx);
                // error of the direction `phi` against the cursor direction, as angle and distance
                let miss = |phi: f64| {
                    let mut e = (theta - phi).rem_euclid(std::f64::consts::TAU);
                    if e > std::f64::consts::PI {
                        e -= std::f64::consts::TAU;
                    }
                    (e.abs(), len * e.sin().abs())
                };
                let ok = |ang: f64, d: f64, cap_deg: f64| {
                    ang.to_degrees() <= cap_deg && (!pixel_mode || d <= self.tolerance)
                };
                let hv_cap = if pixel_mode { DIRECTION_SNAP_DEG } else { 2.0 };
                for (k, horizontal) in [
                    (0.0, true),
                    (std::f64::consts::PI, true),
                    (1.0, false),
                    (-1.0, false),
                ] {
                    let phi = if horizontal {
                        k
                    } else {
                        k * std::f64::consts::FRAC_PI_2
                    };
                    let (ang, d) = miss(phi);
                    if ok(ang, d, hv_cap) {
                        let dir = (phi.cos(), phi.sin());
                        let ray = if horizontal {
                            Ray::Horizontal {
                                from: a,
                                anchor: true,
                            }
                        } else {
                            Ray::Vertical {
                                from: a,
                                anchor: true,
                            }
                        };
                        cands.push(Cand {
                            ray,
                            from: a,
                            dir,
                            d,
                            anchor: true,
                        });
                    }
                }
                if let Some(step) = cfg.polar.filter(|s| *s > 0.0) {
                    let step = step.to_radians();
                    let k = (theta / step).round();
                    let phi = k * step;
                    let deg = phi.to_degrees().rem_euclid(360.0);
                    let on_axis = (deg / 90.0 - (deg / 90.0).round()).abs() < 1e-9;
                    let (ang, d) = miss(phi);
                    if !on_axis && ok(ang, d, DIRECTION_SNAP_DEG) {
                        cands.push(Cand {
                            ray: Ray::Angle { from: a, deg },
                            from: a,
                            dir: (phi.cos(), phi.sin()),
                            d,
                            anchor: true,
                        });
                    }
                }
                if cfg.parallel_perp {
                    for (id, e) in &self.sketch.entities {
                        let EntityKind::Line { p1, p2 } = e.kind else {
                            continue;
                        };
                        let (Ok(u), Ok(v)) = (self.sketch.point(p1), self.sketch.point(p2)) else {
                            continue;
                        };
                        if dist(u, a) < 1e-9 || dist(v, a) < 1e-9 {
                            continue; // a line meeting the start gives nothing to be parallel to
                        }
                        let phi = (v.1 - u.1).atan2(v.0 - u.0);
                        use std::f64::consts::{FRAC_PI_2, PI};
                        for (phi, perp) in [
                            (phi, false),
                            (phi + PI, false),
                            (phi + FRAC_PI_2, true),
                            (phi - FRAC_PI_2, true),
                        ] {
                            let (ang, d) = miss(phi);
                            if ok(ang, d, DIRECTION_SNAP_DEG) {
                                let ray = if perp {
                                    Ray::Perpendicular { from: a, line: id }
                                } else {
                                    Ray::Parallel { from: a, line: id }
                                };
                                cands.push(Cand {
                                    ray,
                                    from: a,
                                    dir: (phi.cos(), phi.sin()),
                                    d,
                                    anchor: true,
                                });
                            }
                        }
                    }
                }
            }
        }
        if cfg.tracking {
            for q in self.acquired_points() {
                if (p.1 - q.1).abs() <= self.tolerance {
                    cands.push(Cand {
                        ray: Ray::Horizontal {
                            from: q,
                            anchor: false,
                        },
                        from: q,
                        dir: (1.0, 0.0),
                        d: (p.1 - q.1).abs(),
                        anchor: false,
                    });
                }
                if (p.0 - q.0).abs() <= self.tolerance {
                    cands.push(Cand {
                        ray: Ray::Vertical {
                            from: q,
                            anchor: false,
                        },
                        from: q,
                        dir: (0.0, 1.0),
                        d: (p.0 - q.0).abs(),
                        anchor: false,
                    });
                }
            }
        }
        // stable: nearest first, anchor rays before alignments on ties
        cands.sort_by(|a, b| {
            a.d.partial_cmp(&b.d)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(b.anchor.cmp(&a.anchor))
        });
        let first = cands.first()?;
        let along = |c: &Cand| {
            let t = (p.0 - c.from.0) * c.dir.0 + (p.1 - c.from.1) * c.dir.1;
            (c.from.0 + t * c.dir.0, c.from.1 + t * c.dir.1)
        };
        for second in cands.iter().skip(1) {
            if first.anchor && second.anchor {
                continue;
            }
            let det = first.dir.0 * second.dir.1 - first.dir.1 * second.dir.0;
            if det.abs() < 1e-6 {
                continue;
            }
            let (rx, ry) = (second.from.0 - first.from.0, second.from.1 - first.from.1);
            let t = (rx * second.dir.1 - ry * second.dir.0) / det;
            let x = (
                first.from.0 + t * first.dir.0,
                first.from.1 + t * first.dir.1,
            );
            if dist(x, p) <= 2.0 * self.tolerance {
                return Some(Snap::Track(x, first.ray, Some(second.ray)));
            }
        }
        Some(Snap::Track(along(first), first.ray, None))
    }

    fn point_for(&mut self, s: Snap) -> Result<EntityId, SketchError> {
        match s {
            Snap::Point(id, _) => Ok(id),
            Snap::OnCurve(curve, q) => {
                let p = self.sketch.add_point(q.0, q.1);
                self.sketch.point_on(p, curve)?;
                Ok(p)
            }
            Snap::Origin => {
                let existing = self
                    .sketch
                    .entities
                    .iter()
                    .filter(|(_, e)| matches!(e.kind, EntityKind::Point(_)))
                    .map(|(id, _)| id)
                    .find(|id| {
                        self.sketch
                            .point(*id)
                            .is_ok_and(|q| dist(q, (0.0, 0.0)) < 1e-9)
                    });
                match existing {
                    Some(id) => Ok(id),
                    None => {
                        let p = self.sketch.add_point(0.0, 0.0);
                        self.sketch.fix(p)?;
                        Ok(p)
                    }
                }
            }
            Snap::Midpoint(line, q) => {
                let p = self.sketch.add_point(q.0, q.1);
                self.sketch.midpoint(p, line)?;
                Ok(p)
            }
            Snap::Intersection(a, b, q) => {
                let p = self.sketch.add_point(q.0, q.1);
                self.sketch.point_on(p, a)?;
                self.sketch.point_on(p, b)?;
                Ok(p)
            }
            Snap::Track(q, ..) | Snap::Grid(q) | Snap::Free(q) => {
                Ok(self.sketch.add_point(q.0, q.1))
            }
        }
    }

    /// Constraints implied by the inference lines a line's end snapped to.
    fn constrain_to_rays(&mut self, line: EntityId, s: Snap) -> Result<(), SketchError> {
        let Snap::Track(_, r1, r2) = s else {
            return Ok(());
        };
        for r in [Some(r1), r2].into_iter().flatten() {
            match r {
                Ray::Horizontal { anchor: true, .. } => {
                    self.sketch.horizontal(line)?;
                }
                Ray::Vertical { anchor: true, .. } => {
                    self.sketch.vertical(line)?;
                }
                Ray::Parallel { line: l, .. } => {
                    self.sketch.parallel(line, l)?;
                }
                Ray::Perpendicular { line: l, .. } => {
                    self.sketch.perpendicular(line, l)?;
                }
                _ => {}
            }
        }
        Ok(())
    }

    // ---------------------------------------------------------------- input

    pub fn handle(&mut self, input: Input) {
        match input {
            Input::Move(p) => self.cursor = p,
            Input::Escape => {
                self.pending = Pending::None;
                self.acquired.clear();
                self.selection.clear();
                self.status.clear();
            }
            Input::Delete => self.delete_selection(),
            Input::Finish => self.finish(),
            Input::Click(p) => {
                self.cursor = p;
                if let Err(e) = self.click(p) {
                    self.status = format!("{e}");
                    self.pending = Pending::None;
                }
                self.acquired.clear();
            }
        }
    }

    /// Delete one constraint (undoable).
    pub fn remove_constraint(&mut self, c: ConstraintId) {
        self.checkpoint();
        if !self.sketch.remove_constraint(c) {
            self.undo.pop();
            return;
        }
        self.refresh();
    }

    /// Toggle construction mode of the selected entities (undoable).
    pub fn toggle_construction(&mut self) {
        if self.selection.is_empty() {
            return;
        }
        self.checkpoint();
        for e in self.selection.clone() {
            if let Some(on) = self.sketch.entities.get(e).map(|x| !x.construction) {
                self.sketch.set_construction(e, on);
            }
        }
    }

    /// Replace the whole sketch (File ▸ New / Open); undoable.
    pub fn replace_sketch(&mut self, sketch: Sketch) {
        self.checkpoint();
        self.sketch = sketch;
        self.selection.clear();
        self.pending = Pending::None;
        self.refresh();
    }

    /// Points of the shape being drawn, for rubber-band previews: the anchor
    /// points already placed (a line's start, a circle's centre, spline points …).
    pub fn anchors(&self) -> Vec<P2> {
        let pt = |e: &EntityId| self.sketch.point(*e).ok();
        match &self.pending {
            Pending::Line(a) | Pending::Circle(a) | Pending::ArcCentre(a) => {
                pt(a).into_iter().collect()
            }
            Pending::Rect(s) => vec![s.position()],
            Pending::ARect(v) => v.iter().map(|s| s.position()).collect(),
            Pending::ArcStart(c, s) => pt(c).into_iter().chain(pt(s)).collect(),
            Pending::Spline(v) | Pending::FitSpline(v) => v.iter().filter_map(pt).collect(),
            Pending::Polygon(c) | Pending::Slot1(c) | Pending::Ellipse1(c) => {
                pt(c).into_iter().collect()
            }
            Pending::Slot2(a, b) | Pending::Ellipse2(a, b) => {
                pt(a).into_iter().chain(pt(b)).collect()
            }
            Pending::Three(v) => v.iter().map(|s| s.position()).collect(),
            Pending::None
            | Pending::Collect(_)
            | Pending::OffsetSide(_)
            | Pending::MirrorAxis(_) => Vec::new(),
        }
    }

    pub fn delete_selection(&mut self) {
        if self.selection.is_empty() {
            return;
        }
        self.checkpoint();
        for e in std::mem::take(&mut self.selection) {
            if self.sketch.entities.contains_key(e) {
                self.sketch.remove_entity(e);
            }
        }
        self.sketch.remove_unused_points();
        self.refresh();
    }

    fn finish(&mut self) {
        match std::mem::take(&mut self.pending) {
            Pending::Spline(pts) if pts.len() >= 2 => {
                self.checkpoint();
                let degree = 3.min(pts.len() - 1);
                if let Err(e) = self.sketch.add_spline(degree, pts) {
                    self.status = format!("{e}");
                }
                self.refresh();
            }
            Pending::FitSpline(pts) if pts.len() >= 2 => {
                self.checkpoint();
                self.sketch.add_fit_spline(pts);
                self.refresh();
            }
            Pending::Collect(es) if self.tool == Tool::Mirror && !es.is_empty() => {
                self.pending = Pending::MirrorAxis(es);
                self.status = "click the line to mirror across".into();
            }
            Pending::Collect(es) if self.tool == Tool::Offset && !es.is_empty() => {
                self.pending = Pending::OffsetSide(es);
                self.status = "click the side to offset towards".into();
            }
            _ => {}
        }
    }

    fn click(&mut self, p: P2) -> Result<(), SketchError> {
        match self.tool {
            Tool::Select => {
                let hit = geom::pick(&self.sketch, p, self.tolerance);
                match hit {
                    Some(e) => {
                        if self.multi_select {
                            if let Some(i) = self.selection.iter().position(|x| *x == e) {
                                self.selection.remove(i);
                            } else {
                                self.selection.push(e);
                            }
                        } else {
                            self.selection = vec![e];
                        }
                    }
                    None if !self.multi_select => self.selection.clear(),
                    None => {}
                }
                Ok(())
            }
            Tool::Point => {
                self.checkpoint();
                let s = self.snap(p);
                if !matches!(s, Snap::Point(..)) {
                    self.point_for(s)?;
                }
                self.refresh();
                Ok(())
            }
            Tool::Line => self.click_line(p),
            Tool::Rectangle => self.click_rect(p),
            Tool::CenterRectangle => self.click_center_rect(p),
            Tool::AlignedRectangle => self.click_aligned_rect(p),
            Tool::Circle => self.click_circle(p),
            Tool::Arc => self.click_arc(p),
            Tool::Spline => {
                self.checkpoint();
                let s = self.snap(p);
                let pt = self.point_for(s)?;
                match &mut self.pending {
                    Pending::Spline(v) => v.push(pt),
                    other => *other = Pending::Spline(vec![pt]),
                }
                Ok(())
            }
            Tool::FitSpline => {
                self.checkpoint();
                let s = self.snap(p);
                let pt = self.point_for(s)?;
                match &mut self.pending {
                    Pending::FitSpline(v) => v.push(pt),
                    other => *other = Pending::FitSpline(vec![pt]),
                }
                Ok(())
            }
            Tool::Polygon => self.click_polygon(p),
            Tool::Slot => self.click_slot(p),
            Tool::Ellipse => self.click_ellipse(p),
            Tool::Arc3 => self.click_three(p, true),
            Tool::Circle3 => self.click_three(p, false),
            Tool::Dimension => self.click_dimension(p),
            Tool::Constrain(kind) => self.click_constraint(kind, p),
            Tool::Trim => {
                let Some(e) = geom::pick(&self.sketch, p, self.tolerance) else {
                    return Ok(());
                };
                self.checkpoint();
                ops::trim(&mut self.sketch, e, p)?;
                self.refresh_or_rollback();
                Ok(())
            }
            Tool::Extend => {
                let Some(e) = geom::pick(&self.sketch, p, self.tolerance) else {
                    return Ok(());
                };
                self.checkpoint();
                ops::extend(&mut self.sketch, e, p)?;
                self.refresh_or_rollback();
                Ok(())
            }
            Tool::Offset => self.click_offset(p),
            Tool::Fillet => self.click_corner(p, true),
            Tool::Chamfer => self.click_corner(p, false),
            Tool::Mirror => self.click_mirror(p),
        }
    }

    fn click_line(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        match self.pending.clone() {
            Pending::Line(from) => {
                self.checkpoint();
                let end = self.point_for(s)?;
                if end == from {
                    self.undo.pop();
                    return Ok(());
                }
                let line = self.sketch.add_line(from, end);
                self.constrain_to_rays(line, s)?;
                self.pending = Pending::Line(end);
                self.refresh_or_rollback();
                Ok(())
            }
            _ => {
                self.checkpoint();
                let from = self.point_for(s)?;
                self.pending = Pending::Line(from);
                Ok(())
            }
        }
    }

    fn click_rect(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        match self.pending.clone() {
            Pending::Rect(first) => {
                let (a, b) = (first.position(), s.position());
                if (a.0 - b.0).abs() < 1e-9 || (a.1 - b.1).abs() < 1e-9 {
                    return Ok(());
                }
                self.checkpoint();
                let p0 = self.point_for(first)?;
                let p1 = self.sketch.add_point(b.0, a.1);
                let p2 = match s {
                    Snap::Point(id, _) => id,
                    Snap::Origin | Snap::Midpoint(..) | Snap::Intersection(..) => {
                        self.point_for(s)?
                    }
                    _ => self.sketch.add_point(b.0, b.1),
                };
                let p3 = self.sketch.add_point(a.0, b.1);
                let l = [
                    self.sketch.add_line(p0, p1),
                    self.sketch.add_line(p1, p2),
                    self.sketch.add_line(p2, p3),
                    self.sketch.add_line(p3, p0),
                ];
                self.sketch.horizontal(l[0])?;
                self.sketch.vertical(l[1])?;
                self.sketch.horizontal(l[2])?;
                self.sketch.vertical(l[3])?;
                self.pending = Pending::None;
                self.refresh_or_rollback();
                Ok(())
            }
            _ => {
                self.pending = Pending::Rect(s);
                Ok(())
            }
        }
    }

    /// Centre rectangle: the centre point stays tied to the middle of both diagonals.
    fn click_center_rect(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        let Pending::Rect(centre) = self.pending.clone() else {
            self.pending = Pending::Rect(s);
            return Ok(());
        };
        let (c, q) = (centre.position(), s.position());
        let (hx, hy) = ((q.0 - c.0).abs(), (q.1 - c.1).abs());
        if hx < 1e-9 || hy < 1e-9 {
            return Ok(());
        }
        self.checkpoint();
        let pc = self.point_for(centre)?;
        let p0 = self.sketch.add_point(c.0 - hx, c.1 - hy);
        let p1 = self.sketch.add_point(c.0 + hx, c.1 - hy);
        let p2 = self.sketch.add_point(c.0 + hx, c.1 + hy);
        let p3 = self.sketch.add_point(c.0 - hx, c.1 + hy);
        let l = [
            self.sketch.add_line(p0, p1),
            self.sketch.add_line(p1, p2),
            self.sketch.add_line(p2, p3),
            self.sketch.add_line(p3, p0),
        ];
        self.sketch.horizontal(l[0])?;
        self.sketch.vertical(l[1])?;
        self.sketch.horizontal(l[2])?;
        self.sketch.vertical(l[3])?;
        let d1 = self.sketch.add_line(p0, p2);
        let d2 = self.sketch.add_line(p1, p3);
        self.sketch.set_construction(d1, true);
        self.sketch.set_construction(d2, true);
        self.sketch.midpoint(pc, d1)?;
        self.sketch.midpoint(pc, d2)?;
        self.pending = Pending::None;
        self.refresh_or_rollback();
        Ok(())
    }

    /// The fourth and third corners of an aligned rectangle with side `a`–`b` and `w` the signed
    /// distance of the third click from that side.
    fn aligned_corners(a: P2, b: P2, q: P2) -> Option<(P2, P2)> {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = dx.hypot(dy);
        if l < 1e-9 {
            return None;
        }
        let (nx, ny) = (-dy / l, dx / l);
        let w = (q.0 - a.0) * nx + (q.1 - a.1) * ny;
        if w.abs() < 1e-9 {
            return None;
        }
        Some(((b.0 + nx * w, b.1 + ny * w), (a.0 + nx * w, a.1 + ny * w)))
    }

    /// Aligned rectangle: one side, then the width; three perpendicular corners keep it rectangular.
    fn click_aligned_rect(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        let mut got = match std::mem::take(&mut self.pending) {
            Pending::ARect(v) => v,
            _ => Vec::new(),
        };
        got.push(s);
        if got.len() < 3 {
            self.pending = Pending::ARect(got);
            return Ok(());
        }
        let (a, b) = (got[0].position(), got[1].position());
        let Some((c, d)) = Self::aligned_corners(a, b, got[2].position()) else {
            got.pop();
            self.pending = Pending::ARect(got);
            return Ok(());
        };
        self.checkpoint();
        let pa = self.point_for(got[0])?;
        let pb = self.point_for(got[1])?;
        let pc = self.sketch.add_point(c.0, c.1);
        let pd = self.sketch.add_point(d.0, d.1);
        let l = [
            self.sketch.add_line(pa, pb),
            self.sketch.add_line(pb, pc),
            self.sketch.add_line(pc, pd),
            self.sketch.add_line(pd, pa),
        ];
        self.sketch.perpendicular(l[0], l[1])?;
        self.sketch.perpendicular(l[1], l[2])?;
        self.sketch.perpendicular(l[2], l[3])?;
        self.refresh_or_rollback();
        Ok(())
    }

    fn click_circle(&mut self, p: P2) -> Result<(), SketchError> {
        match self.pending.clone() {
            Pending::Circle(centre) => {
                let c = self.sketch.point(centre)?;
                let r = dist(c, self.snap(p).position());
                if r < 1e-9 {
                    return Ok(());
                }
                self.checkpoint();
                self.sketch.add_circle(centre, r);
                self.pending = Pending::None;
                self.refresh_or_rollback();
                Ok(())
            }
            _ => {
                self.checkpoint();
                let s = self.snap(p);
                let c = self.point_for(s)?;
                self.pending = Pending::Circle(c);
                Ok(())
            }
        }
    }

    fn click_arc(&mut self, p: P2) -> Result<(), SketchError> {
        match self.pending.clone() {
            Pending::ArcStart(centre, start) => {
                let c = self.sketch.point(centre)?;
                let r = dist(c, self.sketch.point(start)?);
                let q = self.snap(p).position();
                let ang = (q.1 - c.1).atan2(q.0 - c.0);
                self.checkpoint();
                let end = self
                    .sketch
                    .add_point(c.0 + r * ang.cos(), c.1 + r * ang.sin());
                self.sketch.add_arc(centre, start, end);
                self.pending = Pending::None;
                self.refresh_or_rollback();
                Ok(())
            }
            Pending::ArcCentre(centre) => {
                let c = self.sketch.point(centre)?;
                let s = self.snap(p);
                let q = s.position();
                if dist(c, q) < 1e-9 {
                    return Ok(());
                }
                let start = match s {
                    Snap::Point(id, _) => id,
                    _ => self.sketch.add_point(q.0, q.1),
                };
                self.pending = Pending::ArcStart(centre, start);
                Ok(())
            }
            _ => {
                self.checkpoint();
                let s = self.snap(p);
                let c = self.point_for(s)?;
                self.pending = Pending::ArcCentre(c);
                Ok(())
            }
        }
    }

    // ------------------------------------------------------------ more drawing tools

    /// Vertices of a regular `n`-gon around `c` with its first vertex at `v`.
    pub fn polygon_vertices(c: P2, v: P2, n: usize) -> Vec<P2> {
        let (r, a0) = (dist(c, v), (v.1 - c.1).atan2(v.0 - c.0));
        (0..n)
            .map(|i| {
                let a = a0 + std::f64::consts::TAU * i as f64 / n as f64;
                (c.0 + r * a.cos(), c.1 + r * a.sin())
            })
            .collect()
    }

    /// Centre and radius of the circle through three points (`None` when they are collinear).
    pub fn circumcircle(a: P2, b: P2, c: P2) -> Option<(P2, f64)> {
        let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
        if d.abs() < 1e-12 {
            return None;
        }
        let (a2, b2, c2) = (
            a.0 * a.0 + a.1 * a.1,
            b.0 * b.0 + b.1 * b.1,
            c.0 * c.0 + c.1 * c.1,
        );
        let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
        let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
        Some(((ux, uy), dist((ux, uy), a)))
    }

    /// Corner points of the slot between centres `c1` and `c2` with half-width `r`:
    /// `(a1, a2, b1, b2)` where `a` is the left side of `c1 → c2`.
    pub fn slot_corners(c1: P2, c2: P2, r: f64) -> Option<(P2, P2, P2, P2)> {
        let (dx, dy) = (c2.0 - c1.0, c2.1 - c1.1);
        let l = dx.hypot(dy);
        if l < 1e-12 || r < 1e-12 {
            return None;
        }
        let n = (-dy / l * r, dx / l * r);
        Some((
            (c1.0 + n.0, c1.1 + n.1),
            (c2.0 + n.0, c2.1 + n.1),
            (c1.0 - n.0, c1.1 - n.1),
            (c2.0 - n.0, c2.1 - n.1),
        ))
    }

    fn click_polygon(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        match self.pending.clone() {
            Pending::Polygon(centre) => {
                let c = self.sketch.point(centre)?;
                let v = s.position();
                let n = self.polygon_sides.clamp(3, 64);
                if dist(c, v) < 1e-9 {
                    return Ok(());
                }
                self.checkpoint();
                let verts = Self::polygon_vertices(c, v, n);
                let pts: Vec<EntityId> = verts
                    .iter()
                    .map(|q| self.sketch.add_point(q.0, q.1))
                    .collect();
                let lines: Vec<EntityId> = (0..n)
                    .map(|i| self.sketch.add_line(pts[i], pts[(i + 1) % n]))
                    .collect();
                // the circumscribed construction circle plus equal sides keep it a regular polygon
                let circle = self.sketch.add_circle(centre, dist(c, v));
                self.sketch.set_construction(circle, true);
                for q in &pts {
                    self.sketch.point_on(*q, circle)?;
                }
                for l in &lines[1..] {
                    self.sketch.equal_length(lines[0], *l)?;
                }
                self.pending = Pending::None;
                self.refresh_or_rollback();
                Ok(())
            }
            _ => {
                self.checkpoint();
                let c = self.point_for(s)?;
                self.pending = Pending::Polygon(c);
                Ok(())
            }
        }
    }

    fn click_slot(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        match self.pending.clone() {
            Pending::Slot2(c1, c2) => {
                let (a, b) = (self.sketch.point(c1)?, self.sketch.point(c2)?);
                // half-width: distance of the click from the line through the centres
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let l = dx.hypot(dy);
                let q = s.position();
                let r = ((q.0 - a.0) * dy - (q.1 - a.1) * dx).abs() / l;
                let Some((a1, a2, b1, b2)) = Self::slot_corners(a, b, r) else {
                    return Ok(());
                };
                self.checkpoint();
                let (pa1, pa2, pb1, pb2) = (
                    self.sketch.add_point(a1.0, a1.1),
                    self.sketch.add_point(a2.0, a2.1),
                    self.sketch.add_point(b1.0, b1.1),
                    self.sketch.add_point(b2.0, b2.1),
                );
                let side_a = self.sketch.add_line(pa1, pa2);
                let side_b = self.sketch.add_line(pb1, pb2);
                // counter-clockwise: the end at c1 runs a1 → b1 through the far side, the end at c2 b2 → a2
                let end1 = self.sketch.add_arc(c1, pa1, pb1);
                let end2 = self.sketch.add_arc(c2, pb2, pa2);
                // tangent continuity at the four junctions (the end points are shared, so these add no
                // redundancy: the slot has exactly its five degrees of freedom) and one radius
                use crate::constraint::{Continuity::G1, CurveEnd::*};
                self.sketch.joint(side_a, Start, end1, Start, G1)?;
                self.sketch.joint(side_a, End, end2, End, G1)?;
                self.sketch.joint(side_b, Start, end1, End, G1)?;
                self.sketch.joint(side_b, End, end2, Start, G1)?;
                self.sketch.equal_radius(end1, end2)?;
                self.pending = Pending::None;
                self.refresh_or_rollback();
                Ok(())
            }
            Pending::Slot1(c1) => {
                if dist(self.sketch.point(c1)?, s.position()) < 1e-9 {
                    return Ok(());
                }
                let c2 = self.point_for(s)?;
                self.pending = Pending::Slot2(c1, c2);
                Ok(())
            }
            _ => {
                self.checkpoint();
                let c1 = self.point_for(s)?;
                self.pending = Pending::Slot1(c1);
                Ok(())
            }
        }
    }

    fn click_ellipse(&mut self, p: P2) -> Result<(), SketchError> {
        let s = self.snap(p);
        match self.pending.clone() {
            Pending::Ellipse2(c, m) => {
                let (a, b) = (self.sketch.point(c)?, self.sketch.point(m)?);
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let q = s.position();
                let minor = ((q.0 - a.0) * dy - (q.1 - a.1) * dx).abs() / dx.hypot(dy);
                if minor < 1e-9 {
                    return Ok(());
                }
                self.checkpoint();
                self.sketch.add_ellipse(c, m, minor);
                self.pending = Pending::None;
                self.refresh_or_rollback();
                Ok(())
            }
            Pending::Ellipse1(c) => {
                if dist(self.sketch.point(c)?, s.position()) < 1e-9 {
                    return Ok(());
                }
                let m = self.point_for(s)?;
                self.pending = Pending::Ellipse2(c, m);
                Ok(())
            }
            _ => {
                self.checkpoint();
                let c = self.point_for(s)?;
                self.pending = Pending::Ellipse1(c);
                Ok(())
            }
        }
    }

    /// Three-point arc (`arc`) or circle: the first two clicks only remember where, the third makes the shape.
    fn click_three(&mut self, p: P2, arc: bool) -> Result<(), SketchError> {
        let s = self.snap(p);
        let mut got = match std::mem::take(&mut self.pending) {
            Pending::Three(v) => v,
            _ => Vec::new(),
        };
        got.push(s);
        if got.len() < 3 {
            self.pending = Pending::Three(got);
            return Ok(());
        }
        let (a, b, c) = (got[0].position(), got[1].position(), got[2].position());
        let Some((centre, r)) = Self::circumcircle(a, b, c) else {
            self.status = "the three points lie on one line".into();
            return Ok(());
        };
        self.checkpoint();
        let cp = self.sketch.add_point(centre.0, centre.1);
        if arc {
            // clicks: start, end, a point on the arc; the arc runs counter-clockwise start → end
            let ccw = (c.0 - a.0) * (b.1 - c.1) - (c.1 - a.1) * (b.0 - c.0) > 0.0;
            let (first, second) = if ccw { (0, 1) } else { (1, 0) };
            let sp = self.point_for(got[first])?;
            let ep = self.point_for(got[second])?;
            self.sketch.add_arc(cp, sp, ep);
        } else {
            self.sketch.add_circle(cp, r);
        }
        self.refresh_or_rollback();
        Ok(())
    }

    /// Outline of the shape the next click would complete, for a live preview: closed or open polylines.
    pub fn preview(&self, p: P2) -> Vec<Vec<P2>> {
        let q = self.snap(p).position();
        let pt = |e: &EntityId| self.sketch.point(*e).ok();
        let circle = |c: P2, r: f64| -> Vec<P2> {
            (0..=64)
                .map(|i| {
                    let a = std::f64::consts::TAU * i as f64 / 64.0;
                    (c.0 + r * a.cos(), c.1 + r * a.sin())
                })
                .collect()
        };
        match (&self.tool, &self.pending) {
            (Tool::Rectangle, Pending::Rect(first)) => {
                let a = first.position();
                vec![vec![a, (q.0, a.1), q, (a.0, q.1), a]]
            }
            (Tool::CenterRectangle, Pending::Rect(first)) => {
                let c = first.position();
                let (hx, hy) = ((q.0 - c.0).abs(), (q.1 - c.1).abs());
                vec![vec![
                    (c.0 - hx, c.1 - hy),
                    (c.0 + hx, c.1 - hy),
                    (c.0 + hx, c.1 + hy),
                    (c.0 - hx, c.1 + hy),
                    (c.0 - hx, c.1 - hy),
                ]]
            }
            (Tool::AlignedRectangle, Pending::ARect(v)) => match v.as_slice() {
                [a] => vec![vec![a.position(), q]],
                [a, b] => {
                    let (a, b) = (a.position(), b.position());
                    match Self::aligned_corners(a, b, q) {
                        Some((c, d)) => vec![vec![a, b, c, d, a]],
                        None => vec![vec![a, b]],
                    }
                }
                _ => Vec::new(),
            },
            (Tool::Circle, Pending::Circle(c)) => pt(c)
                .map(|c| vec![circle(c, dist(c, q))])
                .unwrap_or_default(),
            (Tool::Polygon, Pending::Polygon(c)) => pt(c)
                .map(|c| {
                    let mut v = Self::polygon_vertices(c, q, self.polygon_sides.clamp(3, 64));
                    v.push(v[0]);
                    vec![v]
                })
                .unwrap_or_default(),
            (Tool::Slot, Pending::Slot2(c1, c2)) => {
                let (Some(a), Some(b)) = (pt(c1), pt(c2)) else {
                    return Vec::new();
                };
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let l = dx.hypot(dy);
                let r = ((q.0 - a.0) * dy - (q.1 - a.1) * dx).abs() / l.max(1e-12);
                let Some((a1, a2, b1, b2)) = Self::slot_corners(a, b, r) else {
                    return Vec::new();
                };
                let phi = dy.atan2(dx);
                let half = |c: P2, from: f64| -> Vec<P2> {
                    (0..=24)
                        .map(|i| {
                            let t = from + std::f64::consts::PI * i as f64 / 24.0;
                            (c.0 + r * t.cos(), c.1 + r * t.sin())
                        })
                        .collect()
                };
                let mut outline = vec![a1, a2];
                outline.extend(half(b, phi - std::f64::consts::FRAC_PI_2).into_iter().rev());
                outline.extend([b2, b1]);
                outline.extend(half(a, phi + std::f64::consts::FRAC_PI_2));
                vec![outline]
            }
            (Tool::Ellipse, Pending::Ellipse2(c, m)) => {
                let (Some(a), Some(b)) = (pt(c), pt(m)) else {
                    return Vec::new();
                };
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let major = dx.hypot(dy);
                let minor = ((q.0 - a.0) * dy - (q.1 - a.1) * dx).abs() / major.max(1e-12);
                let (cx, cy) = (dx / major.max(1e-12), dy / major.max(1e-12));
                vec![(0..=64)
                    .map(|i| {
                        let t = std::f64::consts::TAU * i as f64 / 64.0;
                        let (px, py) = (major * t.cos(), minor * t.sin());
                        (a.0 + px * cx - py * cy, a.1 + px * cy + py * cx)
                    })
                    .collect()]
            }
            (Tool::Circle3, Pending::Three(v)) if v.len() == 2 => {
                match Self::circumcircle(v[0].position(), v[1].position(), q) {
                    Some((c, r)) => vec![circle(c, r)],
                    None => Vec::new(),
                }
            }
            (Tool::Arc3, Pending::Three(v)) if v.len() == 2 => {
                let (a, b) = (v[0].position(), v[1].position());
                let Some((c, r)) = Self::circumcircle(a, b, q) else {
                    return Vec::new();
                };
                let ccw = (q.0 - a.0) * (b.1 - q.1) - (q.1 - a.1) * (b.0 - q.0) > 0.0;
                let (s, e) = if ccw { (a, b) } else { (b, a) };
                let a0 = (s.1 - c.1).atan2(s.0 - c.0);
                let mut a1 = (e.1 - c.1).atan2(e.0 - c.0);
                while a1 <= a0 {
                    a1 += std::f64::consts::TAU;
                }
                vec![(0..=48)
                    .map(|i| {
                        let t = a0 + (a1 - a0) * i as f64 / 48.0;
                        (c.0 + r * t.cos(), c.1 + r * t.sin())
                    })
                    .collect()]
            }
            (Tool::Arc3, Pending::Three(v)) if v.len() == 1 => vec![vec![v[0].position(), q]],
            _ => Vec::new(),
        }
    }

    // ------------------------------------------------------ fillet, chamfer and mirror

    /// Round (`round`) or bevel the corner at the clicked point, where exactly two lines meet.
    fn click_corner(&mut self, p: P2, round: bool) -> Result<(), SketchError> {
        let Some(corner) = geom::pick(&self.sketch, p, self.tolerance)
            .filter(|e| matches!(self.sketch.kind(*e), Ok(EntityKind::Point(_))))
        else {
            self.status = "click the point where two lines meet".into();
            return Ok(());
        };
        let lines: Vec<(EntityId, bool)> = self
            .sketch
            .entities
            .iter()
            .filter_map(|(id, e)| match e.kind {
                EntityKind::Line { p1, .. } if p1 == corner => Some((id, true)),
                EntityKind::Line { p1: _, p2 } if p2 == corner => Some((id, false)),
                _ => None,
            })
            .collect();
        let uses = self
            .sketch
            .entities
            .values()
            .filter(|e| match &e.kind {
                EntityKind::Line { .. } => false,
                EntityKind::Circle { center, .. } => *center == corner,
                EntityKind::Arc { center, start, end } => [*center, *start, *end].contains(&corner),
                EntityKind::Ellipse {
                    center, major_end, ..
                } => *center == corner || *major_end == corner,
                EntityKind::Spline { control, .. } => control.contains(&corner),
                EntityKind::FitSpline { points } => {
                    points.contains(
                        &points
                            .iter()
                            .copied()
                            .find(|q| *q == corner)
                            .unwrap_or(corner),
                    ) && points.contains(&corner)
                }
                EntityKind::Point(_) => false,
            })
            .count();
        if lines.len() != 2 || uses != 0 {
            self.status = "a corner needs exactly two lines meeting at the point".into();
            return Ok(());
        }
        let c = self.sketch.point(corner)?;
        // the far end of each line, and the unit vector from the corner along it
        let far = |l: EntityId, corner_is_p1: bool| -> Result<P2, SketchError> {
            let (a, b) = self.sketch.line_points(l)?;
            self.sketch.point(if corner_is_p1 { b } else { a })
        };
        let (a, b) = (far(lines[0].0, lines[0].1)?, far(lines[1].0, lines[1].1)?);
        let (la, lb) = (dist(c, a), dist(c, b));
        let (u1, u2) = (
            ((a.0 - c.0) / la, (a.1 - c.1) / la),
            ((b.0 - c.0) / lb, (b.1 - c.1) / lb),
        );
        let cos = (u1.0 * u2.0 + u1.1 * u2.1).clamp(-1.0, 1.0);
        let theta = cos.acos();
        if theta < 1e-6 || theta > std::f64::consts::PI - 1e-6 {
            self.status = "the two lines are parallel, there is no corner".into();
            return Ok(());
        }
        let size = self.corner_size;
        // distance from the corner to the points where the new curve meets the lines
        let t = if round {
            size / (theta / 2.0).tan()
        } else {
            size
        };
        if size <= 1e-9 || t >= la.min(lb) - 1e-9 {
            self.status = "the corner size is too big for these lines".into();
            return Ok(());
        }
        let t1 = (c.0 + u1.0 * t, c.1 + u1.1 * t);
        let t2 = (c.0 + u2.0 * t, c.1 + u2.1 * t);
        self.checkpoint();
        let (pt1, pt2) = (
            self.sketch.add_point(t1.0, t1.1),
            self.sketch.add_point(t2.0, t2.1),
        );
        // the lines now end at the new points; the corner point (and constraints on it) goes
        for ((l, corner_is_p1), np) in lines.iter().zip([pt1, pt2]) {
            if let Some(e) = self.sketch.entities.get_mut(*l) {
                if let EntityKind::Line { p1, p2 } = &mut e.kind {
                    if *corner_is_p1 {
                        *p1 = np;
                    } else {
                        *p2 = np;
                    }
                }
            }
        }
        self.sketch.touch();
        self.sketch.remove_entity(corner);
        let end_of = |sk: &Sketch, l: EntityId, np: EntityId| -> crate::constraint::CurveEnd {
            match sk.line_points(l) {
                Ok((a, _)) if a == np => crate::constraint::CurveEnd::Start,
                _ => crate::constraint::CurveEnd::End,
            }
        };
        if round {
            let bis = (u1.0 + u2.0, u1.1 + u2.1);
            let bl = bis.0.hypot(bis.1);
            let d = size / (theta / 2.0).sin();
            let centre = (c.0 + bis.0 / bl * d, c.1 + bis.1 / bl * d);
            let cp = self.sketch.add_point(centre.0, centre.1);
            let ccw =
                (t1.0 - centre.0) * (t2.1 - centre.1) - (t1.1 - centre.1) * (t2.0 - centre.0) > 0.0;
            let (s, e) = if ccw { (pt1, pt2) } else { (pt2, pt1) };
            let arc = self.sketch.add_arc(cp, s, e);
            use crate::constraint::{Continuity::G1, CurveEnd};
            for ((l, _), np) in lines.iter().zip([pt1, pt2]) {
                let arc_end = if np == s {
                    CurveEnd::Start
                } else {
                    CurveEnd::End
                };
                self.sketch
                    .joint(*l, end_of(&self.sketch, *l, np), arc, arc_end, G1)?;
            }
        } else {
            self.sketch.add_line(pt1, pt2);
        }
        self.refresh_or_rollback();
        Ok(())
    }

    /// Reflection of `p` in the line through `a` and `b`.
    pub fn reflect(p: P2, a: P2, b: P2) -> P2 {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = ((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2;
        let foot = (a.0 + t * dx, a.1 + t * dy);
        (2.0 * foot.0 - p.0, 2.0 * foot.1 - p.1)
    }

    fn click_mirror(&mut self, p: P2) -> Result<(), SketchError> {
        let Pending::MirrorAxis(es) = self.pending.clone() else {
            // collecting the entities to mirror
            let Some(e) = geom::pick(&self.sketch, p, self.tolerance) else {
                return Ok(());
            };
            let mut v = match std::mem::take(&mut self.pending) {
                Pending::Collect(v) => v,
                _ => Vec::new(),
            };
            if let Some(i) = v.iter().position(|x| *x == e) {
                v.remove(i);
            } else {
                v.push(e);
            }
            self.selection = v.clone();
            self.pending = Pending::Collect(v);
            self.status = "pick more, then press Enter and click the mirror line".into();
            return Ok(());
        };
        let Some(axis) = geom::pick(&self.sketch, p, self.tolerance)
            .filter(|e| matches!(self.sketch.kind(*e), Ok(EntityKind::Line { .. })))
        else {
            self.status = "click a line to mirror across".into();
            return Ok(());
        };
        let (ap, bp) = self.sketch.line_points(axis)?;
        let (a, b) = (self.sketch.point(ap)?, self.sketch.point(bp)?);
        if dist(a, b) < 1e-9 {
            return Ok(());
        }
        let es: Vec<EntityId> = es.into_iter().filter(|e| *e != axis).collect();
        self.checkpoint();
        let mut map: std::collections::HashMap<EntityId, EntityId> = Default::default();
        let mut pairs: Vec<(EntityId, EntityId)> = Vec::new();
        for e in &es {
            self.mirror_entity(*e, a, b, &mut map, &mut pairs)?;
        }
        // every mirrored point is symmetric to its original about the axis line
        for (orig, copy) in pairs {
            if orig != copy {
                self.sketch.symmetric(orig, copy, axis)?;
            }
        }
        self.pending = Pending::None;
        self.selection.clear();
        self.refresh_or_rollback();
        Ok(())
    }

    fn mirror_point(
        &mut self,
        pt: EntityId,
        a: P2,
        b: P2,
        map: &mut std::collections::HashMap<EntityId, EntityId>,
        pairs: &mut Vec<(EntityId, EntityId)>,
    ) -> Result<EntityId, SketchError> {
        if let Some(m) = map.get(&pt) {
            return Ok(*m);
        }
        let q = Self::reflect(self.sketch.point(pt)?, a, b);
        let n = self.sketch.add_point(q.0, q.1);
        map.insert(pt, n);
        pairs.push((pt, n));
        Ok(n)
    }

    fn mirror_entity(
        &mut self,
        e: EntityId,
        a: P2,
        b: P2,
        map: &mut std::collections::HashMap<EntityId, EntityId>,
        pairs: &mut Vec<(EntityId, EntityId)>,
    ) -> Result<(), SketchError> {
        match self.sketch.kind(e)?.clone() {
            EntityKind::Point(_) => {
                self.mirror_point(e, a, b, map, pairs)?;
            }
            EntityKind::Line { p1, p2 } => {
                let (q1, q2) = (
                    self.mirror_point(p1, a, b, map, pairs)?,
                    self.mirror_point(p2, a, b, map, pairs)?,
                );
                self.sketch.add_line(q1, q2);
            }
            EntityKind::Circle { center, radius } => {
                let c = self.mirror_point(center, a, b, map, pairs)?;
                let r = self.sketch.value(radius);
                self.sketch.add_circle(c, r);
            }
            EntityKind::Arc { center, start, end } => {
                let c = self.mirror_point(center, a, b, map, pairs)?;
                let (s, en) = (
                    self.mirror_point(start, a, b, map, pairs)?,
                    self.mirror_point(end, a, b, map, pairs)?,
                );
                // a reflection reverses the sense of rotation: the arc now runs from the image of its end
                self.sketch.add_arc(c, en, s);
            }
            EntityKind::Ellipse {
                center,
                major_end,
                minor_radius,
            } => {
                let c = self.mirror_point(center, a, b, map, pairs)?;
                let m = self.mirror_point(major_end, a, b, map, pairs)?;
                let minor = self.sketch.value(minor_radius);
                self.sketch.add_ellipse(c, m, minor);
            }
            EntityKind::Spline { degree, control } => {
                let pts = control
                    .iter()
                    .map(|q| self.mirror_point(*q, a, b, map, pairs))
                    .collect::<Result<Vec<_>, _>>()?;
                self.sketch.add_spline(degree, pts)?;
            }
            EntityKind::FitSpline { points } => {
                let pts = points
                    .iter()
                    .map(|q| self.mirror_point(*q, a, b, map, pairs))
                    .collect::<Result<Vec<_>, _>>()?;
                self.sketch.add_fit_spline(pts);
            }
        }
        Ok(())
    }

    fn pick_curve_or_point(&self, p: P2) -> Option<EntityId> {
        geom::pick(&self.sketch, p, self.tolerance)
    }

    fn click_dimension(&mut self, p: P2) -> Result<(), SketchError> {
        let Some(e) = self.pick_curve_or_point(p) else {
            return Ok(());
        };
        let mut got = match std::mem::take(&mut self.pending) {
            Pending::Collect(v) => v,
            _ => Vec::new(),
        };
        got.push(e);
        let class = |e: EntityId| match self.sketch.kind(e) {
            Ok(EntityKind::Point(_)) => 0,
            Ok(EntityKind::Line { .. }) => 1,
            Ok(EntityKind::Circle { .. } | EntityKind::Arc { .. }) => 2,
            _ => 3,
        };
        let cls: Vec<u8> = got.iter().map(|e| class(*e)).collect();
        let made = if got.len() == 1 {
            let e0 = got[0];
            if cls[0] == 1 {
                let (a, b) = self.sketch.line_points(e0)?;
                self.checkpoint();
                Some(self.sketch.dim_distance(a, b)?)
            } else if cls[0] == 2 {
                self.checkpoint();
                Some(self.sketch.dim_radius(e0)?)
            } else {
                None
            }
        } else {
            let (a, b) = (got[0], got[1]);
            let (ca, cb) = (cls[0], cls[1]);
            self.checkpoint();
            if (ca == 0 && cb == 0) || (ca == 0 && cb == 1) || (ca == 1 && cb == 0) {
                Some(self.sketch.dim_distance(a, b)?)
            } else if ca == 1 && cb == 1 {
                let (u, w) = (self.sketch.line_vector(a)?, self.sketch.line_vector(b)?);
                let cross = (u.0 * w.1 - u.1 * w.0).abs() / (u.0.hypot(u.1) * w.0.hypot(w.1));
                if cross < 1e-6 {
                    Some(self.sketch.dim_distance(a, b)?)
                } else {
                    Some(self.sketch.dim_angle(a, b)?)
                }
            } else {
                self.undo.pop();
                self.status = "cannot dimension this combination".into();
                None
            }
        };
        match made {
            Some(c) => {
                self.last_dimension = Some(c);
                self.pending = Pending::None;
                self.refresh_or_rollback();
            }
            None if got.len() == 1 => self.pending = Pending::Collect(got),
            None => {}
        }
        Ok(())
    }

    /// Change a dimension's value and re-solve; conflicting values are rejected.
    pub fn edit_dimension(&mut self, c: ConstraintId, value: f64) -> bool {
        self.checkpoint();
        if self.sketch.set_dimension(c, value).is_err() {
            self.undo.pop();
            return false;
        }
        self.refresh_or_rollback()
    }

    fn click_constraint(&mut self, tool: ConstraintTool, p: P2) -> Result<(), SketchError> {
        let Some(e) = self.pick_curve_or_point(p) else {
            return Ok(());
        };
        let mut got = match std::mem::take(&mut self.pending) {
            Pending::Collect(v) => v,
            _ => Vec::new(),
        };
        got.push(e);
        let class = |e: EntityId| match self.sketch.kind(e) {
            Ok(EntityKind::Point(_)) => 0,
            Ok(EntityKind::Line { .. }) => 1,
            Ok(EntityKind::Circle { .. } | EntityKind::Arc { .. }) => 2,
            _ => 3,
        };
        let cls: Vec<u8> = got.iter().map(|e| class(*e)).collect();
        let is_line = |i: usize| cls[i] == 1;
        let is_point = |i: usize| cls[i] == 0;
        let is_round = |i: usize| cls[i] == 2;
        use ConstraintTool as T;
        let need = match tool {
            T::Fix => 1,
            T::Horizontal | T::Vertical if is_line(0) => 1,
            T::Symmetric => 3,
            _ => 2,
        };
        if got.len() < need {
            self.pending = Pending::Collect(got);
            return Ok(());
        }
        self.checkpoint();
        let r: Result<ConstraintId, SketchError> = match tool {
            T::Coincident => self.sketch.coincident(got[0], got[1]),
            T::Horizontal if need == 1 => self.sketch.horizontal(got[0]),
            T::Vertical if need == 1 => self.sketch.vertical(got[0]),
            T::Horizontal => self.sketch.add_constraint(ConstraintKind::HorizontalPts {
                a: got[0],
                b: got[1],
            }),
            T::Vertical => self.sketch.add_constraint(ConstraintKind::VerticalPts {
                a: got[0],
                b: got[1],
            }),
            T::Parallel => self.sketch.parallel(got[0], got[1]),
            T::Perpendicular => self.sketch.perpendicular(got[0], got[1]),
            T::Tangent => self.sketch.tangent(got[0], got[1]),
            T::Equal if is_round(0) => self.sketch.equal_radius(got[0], got[1]),
            T::Equal => self.sketch.equal_length(got[0], got[1]),
            T::Fix => self.sketch.fix(got[0]),
            T::Midpoint if is_point(0) => self.sketch.midpoint(got[0], got[1]),
            T::Midpoint => self.sketch.midpoint(got[1], got[0]),
            T::Concentric => self.sketch.concentric(got[0], got[1]),
            T::Collinear => self.sketch.collinear(got[0], got[1]),
            T::PointOnCurve if is_point(0) => self.sketch.point_on(got[0], got[1]),
            T::PointOnCurve => self.sketch.point_on(got[1], got[0]),
            T::Symmetric => self.sketch.symmetric(got[0], got[1], got[2]),
        };
        match r {
            Ok(_) => {
                self.pending = Pending::None;
                self.refresh_or_rollback();
            }
            Err(e) => {
                self.undo.pop();
                self.status = format!("{e}");
            }
        }
        Ok(())
    }

    fn click_offset(&mut self, p: P2) -> Result<(), SketchError> {
        match std::mem::take(&mut self.pending) {
            Pending::OffsetSide(es) => {
                self.checkpoint();
                ops::offset(&mut self.sketch, &es, self.offset_distance, p)?;
                self.refresh_or_rollback();
                Ok(())
            }
            other => {
                let mut v = if let Pending::Collect(v) = other {
                    v
                } else {
                    Vec::new()
                };
                if let Some(e) = geom::pick(&self.sketch, p, self.tolerance) {
                    if !v.contains(&e) {
                        v.push(e);
                    }
                }
                self.pending = Pending::Collect(v);
                Ok(())
            }
        }
    }

    // --------------------------------------------------------------- dragging

    /// Start dragging `e` grabbed at `at`.
    pub fn begin_drag(&mut self, e: EntityId, at: P2) {
        let mut pts: Vec<EntityId> = match self.sketch.kind(e) {
            Ok(EntityKind::Point(_)) => vec![e],
            Ok(_) => self.sketch.children(e),
            Err(_) => return,
        };
        pts.retain(|p| matches!(self.sketch.kind(*p), Ok(EntityKind::Point(_))));
        if pts.is_empty() {
            return;
        }
        self.checkpoint();
        let mut vars = Vec::new();
        for p in pts {
            let pv = self.sketch.point_vars(p).unwrap();
            vars.push((pv.x, true, self.sketch.value(pv.x)));
            vars.push((pv.y, false, self.sketch.value(pv.y)));
        }
        self.drag = Some(DragState { vars, grab: at });
    }

    /// Move the grabbed entity towards `to` (one frame).
    pub fn drag_to(&mut self, to: P2) {
        let Some(d) = &self.drag else { return };
        let (dx, dy) = (to.0 - d.grab.0, to.1 - d.grab.1);
        let targets: Vec<(VarId, f64)> = d
            .vars
            .iter()
            .map(|(v, is_x, base)| (*v, base + if *is_x { dx } else { dy }))
            .collect();
        if let Err(e) = self.solver.drag(&mut self.sketch, &targets) {
            self.status = format!("{e}");
        }
    }

    pub fn end_drag(&mut self) {
        if self.drag.take().is_some() {
            self.refresh();
        }
    }

    pub fn is_dragging(&self) -> bool {
        self.drag.is_some()
    }
}

// ------------------------------------------------------------------------------------ bulk edits

impl Editor {
    /// Put ready-made curves (projected edges, a section, text) into the sketch; undoable. The new
    /// curves become the selection. Returns how many were added.
    pub fn insert_curves(&mut self, curves: &[crate::curves::Curve2], construction: bool) -> usize {
        if curves.is_empty() {
            return 0;
        }
        self.checkpoint();
        let made = crate::curves::add_curves(&mut self.sketch, curves, construction);
        self.selection = made.clone();
        self.pending = Pending::None;
        self.refresh();
        made.len()
    }

    /// Lock the selected curves where they are (their points and sizes become constants for the
    /// solver), as projected geometry is in Fusion. Undoable.
    pub fn pin_selection(&mut self) {
        self.checkpoint();
        let mut vars = Vec::new();
        for e in self.selection.clone() {
            let mut all = vec![e];
            all.extend(self.sketch.children(e));
            for c in all {
                match self.sketch.kind(c).cloned() {
                    Ok(EntityKind::Point(p)) => vars.extend([p.x, p.y]),
                    Ok(EntityKind::Circle { radius, .. }) => vars.push(radius),
                    Ok(EntityKind::Ellipse { minor_radius, .. }) => vars.push(minor_radius),
                    _ => {}
                }
            }
        }
        for v in vars {
            self.sketch.set_fixed(v, true);
        }
        self.refresh();
    }

    /// The curves among the selection (points on their own are not patterned).
    fn selected_curves(&self) -> Vec<EntityId> {
        self.selection
            .iter()
            .copied()
            .filter(|e| !matches!(self.sketch.kind(*e), Ok(EntityKind::Point(_))))
            .collect()
    }

    /// Repeat the selected curves in a grid: `nx` columns `step_a` apart and `ny` rows `step_b` apart.
    /// The counts include the original. Returns the number of curves added.
    pub fn pattern_rect(
        &mut self,
        nx: usize,
        ny: usize,
        step_a: P2,
        step_b: P2,
    ) -> Result<usize, String> {
        let es = self.selected_curves();
        if es.is_empty() {
            return Err("select the curves to repeat first".into());
        }
        if nx == 0 || ny == 0 || nx * ny > 2500 {
            return Err("the pattern needs between 1 and 2500 instances".into());
        }
        self.checkpoint();
        let mut all = Vec::new();
        for j in 0..ny {
            for i in 0..nx {
                if i == 0 && j == 0 {
                    continue;
                }
                let d = (
                    step_a.0 * i as f64 + step_b.0 * j as f64,
                    step_a.1 * i as f64 + step_b.1 * j as f64,
                );
                match crate::curves::copy_entities(&mut self.sketch, &es, &|p| {
                    (p.0 + d.0, p.1 + d.1)
                }) {
                    Ok(v) => all.extend(v),
                    Err(e) => {
                        self.undo.pop();
                        return Err(e.to_string());
                    }
                }
            }
        }
        let n = all.len();
        self.selection.extend(all);
        self.pending = Pending::None;
        self.refresh();
        Ok(n)
    }

    /// Repeat the selected curves `count` times about `centre`, spread over `total_deg` (a full turn
    /// spaces them evenly all the way round). The count includes the original.
    pub fn pattern_circular(
        &mut self,
        centre: P2,
        count: usize,
        total_deg: f64,
    ) -> Result<usize, String> {
        let es = self.selected_curves();
        if es.is_empty() {
            return Err("select the curves to repeat first".into());
        }
        if count == 0 || count > 2500 {
            return Err("the pattern needs between 1 and 2500 instances".into());
        }
        let full = (total_deg.abs() - 360.0).abs() < 1e-9;
        let step = if count < 2 {
            0.0
        } else if full {
            total_deg / count as f64
        } else {
            total_deg / (count - 1) as f64
        };
        self.checkpoint();
        let mut all = Vec::new();
        for k in 1..count {
            let (s, c) = (step * k as f64).to_radians().sin_cos();
            match crate::curves::copy_entities(&mut self.sketch, &es, &|p| {
                let (x, y) = (p.0 - centre.0, p.1 - centre.1);
                (centre.0 + x * c - y * s, centre.1 + x * s + y * c)
            }) {
                Ok(v) => all.extend(v),
                Err(e) => {
                    self.undo.pop();
                    return Err(e.to_string());
                }
            }
        }
        let n = all.len();
        self.selection.extend(all);
        self.pending = Pending::None;
        self.refresh();
        Ok(n)
    }
}
