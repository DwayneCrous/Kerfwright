//! The sketch document: variables, entities and constraints, plus builder
//! helpers that capture branch flags and dimension values from the current
//! geometry.

use crate::constraint::*;
use crate::entity::*;
use crate::residual::MAXV;
use crate::system::{self, Lowered};
use slotmap::SlotMap;

#[derive(Debug, Clone, PartialEq)]
pub enum SketchError {
    /// The entity has the wrong type for this operation.
    WrongEntity(&'static str),
    Unsupported(&'static str),
    Degenerate(&'static str),
    UnknownEntity,
    UnknownConstraint,
}

impl std::fmt::Display for SketchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for SketchError {}

/// Scalar unknowns of the sketch (coordinates, radii, dimension values).
#[derive(Debug, Default, Clone)]
pub struct VarStore {
    pub values: Vec<f64>,
    /// Variables pinned by the user (also set for dimension value variables).
    pub fixed: Vec<bool>,
}

impl VarStore {
    pub fn push(&mut self, value: f64) -> VarId {
        self.values.push(value);
        self.fixed.push(false);
        VarId(self.values.len() - 1)
    }
    pub fn push_fixed(&mut self, value: f64) -> VarId {
        let v = self.push(value);
        self.fixed[v.0] = true;
        v
    }
}

/// Entities + constraints + variables for one sketch.
#[derive(Debug, Default, Clone)]
pub struct Sketch {
    pub vars: VarStore,
    pub entities: SlotMap<EntityId, Entity>,
    pub constraints: SlotMap<ConstraintId, Constraint>,
    /// Formulas of dimension values (variable index -> text such as `Width / 2`), kept so that the value
    /// follows the document's parameters. The variable itself holds the evaluated number.
    pub exprs: std::collections::BTreeMap<usize, String>,
    topo: u64,
}

impl Sketch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Counter bumped by every structural change (entities, constraints,
    /// fixed/driven/enabled flags). Solvers cache their compiled system and
    /// decomposition against it; call [`touch`](Self::touch) after editing the
    /// public fields directly.
    pub fn topology_version(&self) -> u64 {
        self.topo
    }

    pub fn touch(&mut self) {
        self.topo += 1;
    }

    /// Pin or release a variable (constant for the solver while pinned).
    pub fn set_fixed(&mut self, v: VarId, fixed: bool) {
        self.vars.fixed[v.0] = fixed;
        self.touch();
    }

    // -------------------------------------------------------------- geometry

    pub fn add_point(&mut self, x: f64, y: f64) -> EntityId {
        self.touch();
        let vx = self.vars.push(x);
        let vy = self.vars.push(y);
        self.entities.insert(Entity {
            kind: EntityKind::Point(PointVars { x: vx, y: vy }),
            construction: false,
        })
    }

    pub fn add_line(&mut self, p1: EntityId, p2: EntityId) -> EntityId {
        self.touch();
        self.entities.insert(Entity {
            kind: EntityKind::Line { p1, p2 },
            construction: false,
        })
    }

    pub fn add_line_xy(&mut self, x1: f64, y1: f64, x2: f64, y2: f64) -> EntityId {
        let (a, b) = (self.add_point(x1, y1), self.add_point(x2, y2));
        self.add_line(a, b)
    }

    pub fn add_circle(&mut self, center: EntityId, radius: f64) -> EntityId {
        self.touch();
        let r = self.vars.push(radius);
        self.entities.insert(Entity {
            kind: EntityKind::Circle { center, radius: r },
            construction: false,
        })
    }

    pub fn add_circle_xy(&mut self, cx: f64, cy: f64, radius: f64) -> EntityId {
        let c = self.add_point(cx, cy);
        self.add_circle(c, radius)
    }

    /// Counter-clockwise arc from `start` to `end` about `center`.
    pub fn add_arc(&mut self, center: EntityId, start: EntityId, end: EntityId) -> EntityId {
        self.touch();
        self.entities.insert(Entity {
            kind: EntityKind::Arc { center, start, end },
            construction: false,
        })
    }

    /// Arc from angle `a0` to `a1` (radians, CCW) of the circle `(cx, cy, r)`.
    pub fn add_arc_xy(&mut self, cx: f64, cy: f64, r: f64, a0: f64, a1: f64) -> EntityId {
        let c = self.add_point(cx, cy);
        let s = self.add_point(cx + r * a0.cos(), cy + r * a0.sin());
        let e = self.add_point(cx + r * a1.cos(), cy + r * a1.sin());
        self.add_arc(c, s, e)
    }

    pub fn add_ellipse(&mut self, center: EntityId, major_end: EntityId, minor: f64) -> EntityId {
        self.touch();
        let b = self.vars.push(minor);
        self.entities.insert(Entity {
            kind: EntityKind::Ellipse {
                center,
                major_end,
                minor_radius: b,
            },
            construction: false,
        })
    }

    pub fn add_spline(
        &mut self,
        degree: usize,
        control: Vec<EntityId>,
    ) -> Result<EntityId, SketchError> {
        if degree == 0 || control.len() < degree + 1 {
            return Err(SketchError::Degenerate(
                "spline needs degree >= 1 and degree + 1 control points",
            ));
        }
        self.touch();
        Ok(self.entities.insert(Entity {
            kind: EntityKind::Spline { degree, control },
            construction: false,
        }))
    }

    pub fn add_fit_spline(&mut self, points: Vec<EntityId>) -> EntityId {
        self.touch();
        self.entities.insert(Entity {
            kind: EntityKind::FitSpline { points },
            construction: false,
        })
    }

    /// Entities this entity is built from (a line's end points, a circle's centre, …).
    pub fn children(&self, e: EntityId) -> Vec<EntityId> {
        match self.entities.get(e).map(|x| &x.kind) {
            Some(EntityKind::Line { p1, p2 }) => vec![*p1, *p2],
            Some(EntityKind::Circle { center, .. }) => vec![*center],
            Some(EntityKind::Arc { center, start, end }) => vec![*center, *start, *end],
            Some(EntityKind::Ellipse {
                center, major_end, ..
            }) => vec![*center, *major_end],
            Some(EntityKind::Spline { control, .. }) => control.clone(),
            Some(EntityKind::FitSpline { points }) => points.clone(),
            _ => Vec::new(),
        }
    }

    /// Remove an entity together with everything that depends on it: constraints
    /// that mention it and entities built from it. Child points that become
    /// unused are kept (call [`remove_unused_points`](Self::remove_unused_points)).
    pub fn remove_entity(&mut self, e: EntityId) {
        let mut doomed = vec![e];
        let mut i = 0;
        while i < doomed.len() {
            let cur = doomed[i];
            i += 1;
            let parents: Vec<EntityId> = self
                .entities
                .keys()
                .filter(|id| !doomed.contains(id) && self.children(*id).contains(&cur))
                .collect();
            doomed.extend(parents);
        }
        let dead_constraints: Vec<ConstraintId> = self
            .constraints
            .iter()
            .filter(|(_, c)| c.kind.entities().iter().any(|x| doomed.contains(x)))
            .map(|(id, _)| id)
            .collect();
        for c in dead_constraints {
            self.constraints.remove(c);
        }
        for d in doomed {
            self.entities.remove(d);
        }
        self.touch();
    }

    /// Delete points that no remaining entity or constraint refers to.
    pub fn remove_unused_points(&mut self) {
        let mut used: std::collections::HashSet<EntityId> = std::collections::HashSet::new();
        for id in self.entities.keys() {
            used.extend(self.children(id));
        }
        for c in self.constraints.values() {
            used.extend(c.kind.entities());
        }
        let dead: Vec<EntityId> = self
            .entities
            .iter()
            .filter(|(id, e)| matches!(e.kind, EntityKind::Point(_)) && !used.contains(id))
            .map(|(id, _)| id)
            .collect();
        for d in dead {
            self.entities.remove(d);
        }
        self.touch();
    }

    pub fn set_construction(&mut self, e: EntityId, on: bool) {
        if let Some(ent) = self.entities.get_mut(e) {
            ent.construction = on;
        }
    }

    pub fn kind(&self, e: EntityId) -> Result<&EntityKind, SketchError> {
        self.entities
            .get(e)
            .map(|x| &x.kind)
            .ok_or(SketchError::UnknownEntity)
    }

    pub fn value(&self, v: VarId) -> f64 {
        self.vars.values[v.0]
    }

    pub fn set_value(&mut self, v: VarId, x: f64) {
        self.vars.values[v.0] = x;
    }

    pub fn point_vars(&self, e: EntityId) -> Result<PointVars, SketchError> {
        match self.kind(e)? {
            EntityKind::Point(p) => Ok(*p),
            _ => Err(SketchError::WrongEntity("expected a point")),
        }
    }

    pub fn point(&self, e: EntityId) -> Result<(f64, f64), SketchError> {
        let p = self.point_vars(e)?;
        Ok((self.value(p.x), self.value(p.y)))
    }

    pub fn set_point(&mut self, e: EntityId, x: f64, y: f64) -> Result<(), SketchError> {
        let p = self.point_vars(e)?;
        self.set_value(p.x, x);
        self.set_value(p.y, y);
        Ok(())
    }

    pub fn line_points(&self, e: EntityId) -> Result<(EntityId, EntityId), SketchError> {
        match self.kind(e)? {
            EntityKind::Line { p1, p2 } => Ok((*p1, *p2)),
            _ => Err(SketchError::WrongEntity("expected a line")),
        }
    }

    /// Direction `(p2 − p1)` of a line.
    pub fn line_vector(&self, e: EntityId) -> Result<(f64, f64), SketchError> {
        let (a, b) = self.line_points(e)?;
        let (pa, pb) = (self.point(a)?, self.point(b)?);
        Ok((pb.0 - pa.0, pb.1 - pa.1))
    }

    /// Centre and radius of a circle or arc.
    pub fn circle_geometry(&self, e: EntityId) -> Result<((f64, f64), f64), SketchError> {
        match self.kind(e)? {
            EntityKind::Circle { center, radius } => {
                Ok((self.point(*center)?, self.value(*radius)))
            }
            EntityKind::Arc { center, start, .. } => {
                let c = self.point(*center)?;
                let s = self.point(*start)?;
                Ok((c, ((s.0 - c.0).powi(2) + (s.1 - c.1).powi(2)).sqrt()))
            }
            _ => Err(SketchError::WrongEntity("expected a circle or arc")),
        }
    }

    /// Every variable that defines an entity (for `Fix`).
    pub fn entity_vars(&self, e: EntityId) -> Result<Vec<VarId>, SketchError> {
        let mut out = Vec::new();
        self.collect_vars(e, &mut out)?;
        out.sort();
        out.dedup();
        Ok(out)
    }

    fn collect_vars(&self, e: EntityId, out: &mut Vec<VarId>) -> Result<(), SketchError> {
        match self.kind(e)?.clone() {
            EntityKind::Point(p) => out.extend([p.x, p.y]),
            EntityKind::Line { p1, p2 } => {
                self.collect_vars(p1, out)?;
                self.collect_vars(p2, out)?;
            }
            EntityKind::Circle { center, radius } => {
                self.collect_vars(center, out)?;
                out.push(radius);
            }
            EntityKind::Arc { center, start, end } => {
                for p in [center, start, end] {
                    self.collect_vars(p, out)?;
                }
            }
            EntityKind::Ellipse {
                center,
                major_end,
                minor_radius,
            } => {
                self.collect_vars(center, out)?;
                self.collect_vars(major_end, out)?;
                out.push(minor_radius);
            }
            EntityKind::Spline { control, .. } => {
                for p in control {
                    self.collect_vars(p, out)?;
                }
            }
            EntityKind::FitSpline { points } => {
                for p in points {
                    self.collect_vars(p, out)?;
                }
            }
        }
        Ok(())
    }

    /// Bounding box of all points `(min, max)`.
    pub fn bounds(&self) -> Option<((f64, f64), (f64, f64))> {
        let mut lo = (f64::INFINITY, f64::INFINITY);
        let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for e in self.entities.values() {
            if let EntityKind::Point(p) = &e.kind {
                let (x, y) = (self.value(p.x), self.value(p.y));
                lo = (lo.0.min(x), lo.1.min(y));
                hi = (hi.0.max(x), hi.1.max(y));
            }
        }
        (lo.0.is_finite()).then_some((lo, hi))
    }

    // ----------------------------------------------------------- constraints

    /// Add a constraint after checking that it can be lowered to residuals.
    pub fn add_constraint(&mut self, kind: ConstraintKind) -> Result<ConstraintId, SketchError> {
        let c = Constraint {
            kind,
            driven: false,
            enabled: true,
        };
        system::lower(self, &c)?;
        self.touch();
        Ok(self.constraints.insert(c))
    }

    pub fn remove_constraint(&mut self, c: ConstraintId) -> bool {
        self.touch();
        self.constraints.remove(c).is_some()
    }

    pub fn set_driven(&mut self, c: ConstraintId, driven: bool) {
        self.touch();
        if let Some(c) = self.constraints.get_mut(c) {
            c.driven = driven;
        }
    }

    pub fn set_enabled(&mut self, c: ConstraintId, enabled: bool) {
        self.touch();
        if let Some(c) = self.constraints.get_mut(c) {
            c.enabled = enabled;
        }
    }

    pub fn coincident(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Coincident { a, b })
    }
    pub fn horizontal(&mut self, line: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Horizontal(line))
    }
    pub fn vertical(&mut self, line: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Vertical(line))
    }
    pub fn parallel(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Parallel { a, b })
    }
    pub fn perpendicular(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Perpendicular { a, b })
    }
    pub fn fix(&mut self, e: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Fix(e))
    }
    pub fn point_on(
        &mut self,
        point: EntityId,
        curve: EntityId,
    ) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::PointOnCurve { point, curve })
    }
    pub fn equal_length(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::EqualLength { a, b })
    }
    pub fn equal_radius(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::EqualRadius { a, b })
    }
    pub fn concentric(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Concentric { a, b })
    }
    pub fn collinear(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Collinear { a, b })
    }
    pub fn midpoint(
        &mut self,
        point: EntityId,
        line: EntityId,
    ) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Midpoint { point, line })
    }
    pub fn symmetric(
        &mut self,
        a: EntityId,
        b: EntityId,
        axis: EntityId,
    ) -> Result<ConstraintId, SketchError> {
        self.add_constraint(ConstraintKind::Symmetric { a, b, axis })
    }

    /// Tangency between a line and a circle/arc, or two circles/arcs; the
    /// branch (which side / external vs internal) is captured from the
    /// current geometry.
    pub fn tangent(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        let sign = match (self.kind(a)?.clone(), self.kind(b)?.clone()) {
            (EntityKind::Line { .. }, _) | (_, EntityKind::Line { .. }) => {
                let (line, circ) = if matches!(self.kind(a)?, EntityKind::Line { .. }) {
                    (a, b)
                } else {
                    (b, a)
                };
                let (p1, _) = self.line_points(line)?;
                let (d, p) = (self.line_vector(line)?, self.point(p1)?);
                let (c, _) = self.circle_geometry(circ)?;
                let cross = d.0 * (c.1 - p.1) - d.1 * (c.0 - p.0);
                if cross >= 0.0 {
                    1.0
                } else {
                    -1.0
                }
            }
            _ => {
                let ((c1, r1), (c2, r2)) = (self.circle_geometry(a)?, self.circle_geometry(b)?);
                let dist = ((c1.0 - c2.0).powi(2) + (c1.1 - c2.1).powi(2)).sqrt();
                if (dist - (r1 + r2)).abs() <= (dist - (r1 - r2).abs()).abs() {
                    1.0
                } else {
                    -1.0
                }
            }
        };
        self.add_constraint(ConstraintKind::Tangent {
            a,
            b,
            branch: TangentBranch::from_sign(sign),
        })
    }

    /// Continuity joint between curve ends; the branch is captured from the
    /// current tangents.
    pub fn joint(
        &mut self,
        a: EntityId,
        a_end: CurveEnd,
        b: EntityId,
        b_end: CurveEnd,
        level: Continuity,
    ) -> Result<ConstraintId, SketchError> {
        let probe = Constraint {
            kind: ConstraintKind::Joint {
                a,
                a_end,
                b,
                b_end,
                level: Continuity::G1,
                branch: TangentBranch::Positive,
            },
            driven: false,
            enabled: true,
        };
        let lowered = system::lower(self, &probe)?;
        let r = &lowered.res[0];
        let mut v = [0.0; MAXV];
        for (i, g) in r.vars.iter().enumerate() {
            v[i] = self.vars.values[*g as usize];
        }
        let dot = if let crate::residual::Kind::Joint(j) = &r.kind {
            let (_, ta, _) = j.a.eval(&v);
            let (_, tb, _) = j.b.eval(&v);
            j.sign_a * j.sign_b * (ta.0 * tb.0 + ta.1 * tb.1)
        } else {
            1.0
        };
        self.add_constraint(ConstraintKind::Joint {
            a,
            a_end,
            b,
            b_end,
            level,
            branch: TangentBranch::from_sign(dot),
        })
    }

    // ------------------------------------------------------------ dimensions

    fn dim_var(&mut self, value: f64) -> VarId {
        self.touch();
        self.vars.push_fixed(value)
    }

    /// Distance dimension taking its value from the current geometry:
    /// point–point, point–line (signed side captured) or parallel line–line.
    pub fn dim_distance(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        let (value, sign) = match (self.kind(a)?.clone(), self.kind(b)?.clone()) {
            (EntityKind::Point(_), EntityKind::Point(_)) => {
                let (pa, pb) = (self.point(a)?, self.point(b)?);
                (((pa.0 - pb.0).powi(2) + (pa.1 - pb.1).powi(2)).sqrt(), 1.0)
            }
            (EntityKind::Point(_), EntityKind::Line { .. })
            | (EntityKind::Line { .. }, EntityKind::Point(_)) => {
                let (pt, line) = if matches!(self.kind(a)?, EntityKind::Point(_)) {
                    (a, b)
                } else {
                    (b, a)
                };
                let (l1, _) = self.line_points(line)?;
                let (d, o, p) = (self.line_vector(line)?, self.point(l1)?, self.point(pt)?);
                let len = (d.0 * d.0 + d.1 * d.1).sqrt();
                if len == 0.0 {
                    return Err(SketchError::Degenerate("zero-length line"));
                }
                let cr = d.0 * (p.1 - o.1) - d.1 * (p.0 - o.0);
                (cr.abs() / len, if cr >= 0.0 { 1.0 } else { -1.0 })
            }
            (EntityKind::Line { .. }, EntityKind::Line { .. }) => {
                let (a1, _) = self.line_points(a)?;
                let (l1, _) = self.line_points(b)?;
                let (d, o, p) = (self.line_vector(b)?, self.point(l1)?, self.point(a1)?);
                let len = (d.0 * d.0 + d.1 * d.1).sqrt();
                if len == 0.0 {
                    return Err(SketchError::Degenerate("zero-length line"));
                }
                let cr = d.0 * (p.1 - o.1) - d.1 * (p.0 - o.0);
                (cr.abs() / len, if cr >= 0.0 { 1.0 } else { -1.0 })
            }
            _ => {
                return Err(SketchError::Unsupported(
                    "distance needs points and/or lines",
                ))
            }
        };
        let v = self.dim_var(value);
        match self.add_constraint(ConstraintKind::Distance {
            a,
            b,
            value: v,
            branch: TangentBranch::from_sign(sign),
        }) {
            Ok(c) => Ok(c),
            Err(e) => {
                // do not leak the value variable on failure
                self.vars.values.pop();
                self.vars.fixed.pop();
                Err(e)
            }
        }
    }

    /// Directed angle (radians) from line `a`'s direction to line `b`'s, from the current geometry.
    pub fn dim_angle(&mut self, a: EntityId, b: EntityId) -> Result<ConstraintId, SketchError> {
        let (u, w) = (self.line_vector(a)?, self.line_vector(b)?);
        let phi = (u.0 * w.1 - u.1 * w.0).atan2(u.0 * w.0 + u.1 * w.1);
        let v = self.dim_var(phi);
        self.add_constraint(ConstraintKind::Angle { a, b, value: v })
    }

    pub fn dim_radius(&mut self, circle: EntityId) -> Result<ConstraintId, SketchError> {
        let (_, r) = self.circle_geometry(circle)?;
        let v = self.dim_var(r);
        self.add_constraint(ConstraintKind::Radius { circle, value: v })
    }

    pub fn dim_diameter(&mut self, circle: EntityId) -> Result<ConstraintId, SketchError> {
        let (_, r) = self.circle_geometry(circle)?;
        let v = self.dim_var(2.0 * r);
        self.add_constraint(ConstraintKind::Diameter { circle, value: v })
    }

    /// Current value of a dimension constraint: the number it drives, or, for a driven (reference)
    /// dimension, what the geometry measures right now.
    pub fn dimension_value(&self, c: ConstraintId) -> Option<f64> {
        let con = self.constraints.get(c)?;
        if con.driven {
            return self.measure(c);
        }
        let v = self.dimension_var(c)?;
        Some(self.value(v))
    }

    /// What the geometry measures for a dimension constraint (distance, angle, radius, diameter).
    pub fn measure(&self, c: ConstraintId) -> Option<f64> {
        match &self.constraints.get(c)?.kind {
            ConstraintKind::Distance { a, b, .. } => self.measure_distance(*a, *b),
            ConstraintKind::Angle { a, b, .. } => {
                let (u, w) = (self.line_vector(*a).ok()?, self.line_vector(*b).ok()?);
                Some((u.0 * w.1 - u.1 * w.0).atan2(u.0 * w.0 + u.1 * w.1))
            }
            ConstraintKind::Radius { circle, .. } => Some(self.circle_geometry(*circle).ok()?.1),
            ConstraintKind::Diameter { circle, .. } => {
                Some(2.0 * self.circle_geometry(*circle).ok()?.1)
            }
            _ => None,
        }
    }

    fn measure_distance(&self, a: EntityId, b: EntityId) -> Option<f64> {
        let kinds = (self.kind(a).ok()?, self.kind(b).ok()?);
        match kinds {
            (EntityKind::Point(_), EntityKind::Point(_)) => {
                let (pa, pb) = (self.point(a).ok()?, self.point(b).ok()?);
                Some(((pa.0 - pb.0).powi(2) + (pa.1 - pb.1).powi(2)).sqrt())
            }
            (EntityKind::Point(_), EntityKind::Line { .. }) => self.point_line_distance(a, b),
            (EntityKind::Line { .. }, EntityKind::Point(_)) => self.point_line_distance(b, a),
            (EntityKind::Line { .. }, EntityKind::Line { .. }) => {
                let (a1, _) = self.line_points(a).ok()?;
                self.point_line_distance(a1, b)
            }
            _ => None,
        }
    }

    fn point_line_distance(&self, pt: EntityId, line: EntityId) -> Option<f64> {
        let (l1, _) = self.line_points(line).ok()?;
        let (d, o, p) = (
            self.line_vector(line).ok()?,
            self.point(l1).ok()?,
            self.point(pt).ok()?,
        );
        let len = (d.0 * d.0 + d.1 * d.1).sqrt();
        (len > 0.0).then(|| (d.0 * (p.1 - o.1) - d.1 * (p.0 - o.0)).abs() / len)
    }

    /// The name of a dimension (`d1`, `d2`, …), in the order the dimensions were made; formulas of
    /// other dimensions may use it.
    pub fn dimension_name(&self, c: ConstraintId) -> Option<String> {
        self.dimension_var(c)?;
        let n = self
            .constraints
            .iter()
            .filter(|(id, _)| self.dimension_var(*id).is_some())
            .position(|(id, _)| id == c)?;
        Some(format!("d{}", n + 1))
    }

    pub fn dimension_var(&self, c: ConstraintId) -> Option<VarId> {
        match &self.constraints.get(c)?.kind {
            ConstraintKind::Distance { value, .. }
            | ConstraintKind::Angle { value, .. }
            | ConstraintKind::Radius { value, .. }
            | ConstraintKind::Diameter { value, .. } => Some(*value),
            _ => None,
        }
    }

    pub fn set_dimension(&mut self, c: ConstraintId, value: f64) -> Result<(), SketchError> {
        let v = self
            .dimension_var(c)
            .ok_or(SketchError::WrongEntity("not a dimension"))?;
        self.set_value(v, value);
        Ok(())
    }
}

pub use system::Lowered as LoweredConstraint;

#[allow(dead_code)]
fn _assert_lowered(_: Lowered) {}
