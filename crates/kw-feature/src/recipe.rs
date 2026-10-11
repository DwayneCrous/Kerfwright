//! A serialisable description of a sketch. Entities are addressed by their index in the recipe, which is
//! the sketch entity's *label*: stable across save/load and used in `NameOrigin::SketchEdge`.

use kw_expr::Param;
use kw_sketch::{
    Constraint, ConstraintKind, Continuity, CurveEnd, Entity, EntityId, EntityKind, PointVars,
    Sketch, TangentBranch, VarId,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

impl EntityKindRec {
    /// Entities this one is built from.
    fn deps(&self) -> Vec<u32> {
        match self {
            EntityKindRec::Point { .. } => vec![],
            EntityKindRec::Line { p1, p2 } => vec![*p1, *p2],
            EntityKindRec::Circle { center, .. } => vec![*center],
            EntityKindRec::Arc { center, start, end } => vec![*center, *start, *end],
            EntityKindRec::Ellipse {
                center, major_end, ..
            } => vec![*center, *major_end],
            EntityKindRec::Spline { control, .. } => control.clone(),
            EntityKindRec::FitSpline { points } => points.clone(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum EntityKindRec {
    /// Coordinates are variables `x` and `y`.
    Point {
        x: u32,
        y: u32,
    },
    Line {
        p1: u32,
        p2: u32,
    },
    Circle {
        center: u32,
        radius: u32,
    },
    Arc {
        center: u32,
        start: u32,
        end: u32,
    },
    Ellipse {
        center: u32,
        major_end: u32,
        minor_radius: u32,
    },
    Spline {
        degree: u32,
        control: Vec<u32>,
    },
    FitSpline {
        points: Vec<u32>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityRec {
    pub kind: EntityKindRec,
    pub construction: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ConstraintKindRec {
    Coincident(u32, u32),
    PointOnCurve(u32, u32),
    Horizontal(u32),
    Vertical(u32),
    HorizontalPts(u32, u32),
    VerticalPts(u32, u32),
    Parallel(u32, u32),
    Perpendicular(u32, u32),
    Collinear(u32, u32),
    Concentric(u32, u32),
    EqualLength(u32, u32),
    EqualRadius(u32, u32),
    Midpoint(u32, u32),
    Symmetric(u32, u32, u32),
    Fix(u32),
    Tangent(u32, u32, bool),
    /// `level`: 0 = G0, 1 = G1, 2 = G2; `a_end`/`b_end`: true = end, false = start.
    Joint {
        a: u32,
        a_end: bool,
        b: u32,
        b_end: bool,
        level: u8,
        positive: bool,
    },
    Distance {
        a: u32,
        b: u32,
        value: u32,
        positive: bool,
    },
    Angle {
        a: u32,
        b: u32,
        value: u32,
    },
    Radius(u32, u32),
    Diameter(u32, u32),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstraintRec {
    pub kind: ConstraintKindRec,
    pub driven: bool,
    pub enabled: bool,
}

/// A variable of the sketch (a dimension value, a coordinate) taken from a parameter or expression.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VarBinding {
    pub var: u32,
    pub value: Param,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct SketchRecipe {
    pub vars: Vec<f64>,
    pub fixed: Vec<bool>,
    pub entities: Vec<EntityRec>,
    pub constraints: Vec<ConstraintRec>,
    pub bindings: Vec<VarBinding>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RecipeError(pub String);
impl std::fmt::Display for RecipeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "invalid sketch recipe: {}", self.0)
    }
}
impl std::error::Error for RecipeError {}

fn tb(positive: bool) -> TangentBranch {
    if positive {
        TangentBranch::Positive
    } else {
        TangentBranch::Negative
    }
}
fn is_pos(b: TangentBranch) -> bool {
    b == TangentBranch::Positive
}

impl SketchRecipe {
    /// Record a sketch. Entity labels are the entities' positions in the sketch's slot order.
    pub fn from_sketch(sk: &Sketch) -> SketchRecipe {
        // Dependencies first: a freed slot that is reused can hold a line while its end points sit in
        // later slots, so the slot order alone is not always a valid build order.
        let ids: Vec<EntityId> = {
            fn visit(
                sk: &Sketch,
                e: EntityId,
                seen: &mut std::collections::HashSet<EntityId>,
                out: &mut Vec<EntityId>,
            ) {
                if !seen.insert(e) {
                    return;
                }
                for d in sk.children(e) {
                    if d != e && sk.entities.contains_key(d) {
                        visit(sk, d, seen, out);
                    }
                }
                out.push(e);
            }
            let mut seen = std::collections::HashSet::new();
            let mut out = Vec::with_capacity(sk.entities.len());
            for e in sk.entities.keys() {
                visit(sk, e, &mut seen, &mut out);
            }
            out
        };
        let idx: HashMap<EntityId, u32> = ids
            .iter()
            .enumerate()
            .map(|(i, e)| (*e, i as u32))
            .collect();
        let ix = |e: &EntityId| idx[e];
        let entities = ids
            .iter()
            .map(|id| &sk.entities[*id])
            .map(|e| EntityRec {
                construction: e.construction,
                kind: match &e.kind {
                    EntityKind::Point(p) => EntityKindRec::Point {
                        x: p.x.0 as u32,
                        y: p.y.0 as u32,
                    },
                    EntityKind::Line { p1, p2 } => EntityKindRec::Line {
                        p1: ix(p1),
                        p2: ix(p2),
                    },
                    EntityKind::Circle { center, radius } => EntityKindRec::Circle {
                        center: ix(center),
                        radius: radius.0 as u32,
                    },
                    EntityKind::Arc { center, start, end } => EntityKindRec::Arc {
                        center: ix(center),
                        start: ix(start),
                        end: ix(end),
                    },
                    EntityKind::Ellipse {
                        center,
                        major_end,
                        minor_radius,
                    } => EntityKindRec::Ellipse {
                        center: ix(center),
                        major_end: ix(major_end),
                        minor_radius: minor_radius.0 as u32,
                    },
                    EntityKind::Spline { degree, control } => EntityKindRec::Spline {
                        degree: *degree as u32,
                        control: control.iter().map(ix).collect(),
                    },
                    EntityKind::FitSpline { points } => EntityKindRec::FitSpline {
                        points: points.iter().map(ix).collect(),
                    },
                },
            })
            .collect();
        let constraints = sk
            .constraints
            .values()
            .map(|c| {
                use ConstraintKind as K;
                use ConstraintKindRec as R;
                let kind = match &c.kind {
                    K::Coincident { a, b } => R::Coincident(ix(a), ix(b)),
                    K::PointOnCurve { point, curve } => R::PointOnCurve(ix(point), ix(curve)),
                    K::Horizontal(a) => R::Horizontal(ix(a)),
                    K::Vertical(a) => R::Vertical(ix(a)),
                    K::HorizontalPts { a, b } => R::HorizontalPts(ix(a), ix(b)),
                    K::VerticalPts { a, b } => R::VerticalPts(ix(a), ix(b)),
                    K::Parallel { a, b } => R::Parallel(ix(a), ix(b)),
                    K::Perpendicular { a, b } => R::Perpendicular(ix(a), ix(b)),
                    K::Collinear { a, b } => R::Collinear(ix(a), ix(b)),
                    K::Concentric { a, b } => R::Concentric(ix(a), ix(b)),
                    K::EqualLength { a, b } => R::EqualLength(ix(a), ix(b)),
                    K::EqualRadius { a, b } => R::EqualRadius(ix(a), ix(b)),
                    K::Midpoint { point, line } => R::Midpoint(ix(point), ix(line)),
                    K::Symmetric { a, b, axis } => R::Symmetric(ix(a), ix(b), ix(axis)),
                    K::Fix(a) => R::Fix(ix(a)),
                    K::Tangent { a, b, branch } => R::Tangent(ix(a), ix(b), is_pos(*branch)),
                    K::Joint {
                        a,
                        a_end,
                        b,
                        b_end,
                        level,
                        branch,
                    } => R::Joint {
                        a: ix(a),
                        a_end: *a_end == CurveEnd::End,
                        b: ix(b),
                        b_end: *b_end == CurveEnd::End,
                        level: match level {
                            Continuity::G0 => 0,
                            Continuity::G1 => 1,
                            Continuity::G2 => 2,
                        },
                        positive: is_pos(*branch),
                    },
                    K::Distance {
                        a,
                        b,
                        value,
                        branch,
                    } => R::Distance {
                        a: ix(a),
                        b: ix(b),
                        value: value.0 as u32,
                        positive: is_pos(*branch),
                    },
                    K::Angle { a, b, value } => R::Angle {
                        a: ix(a),
                        b: ix(b),
                        value: value.0 as u32,
                    },
                    K::Radius { circle, value } => R::Radius(ix(circle), value.0 as u32),
                    K::Diameter { circle, value } => R::Diameter(ix(circle), value.0 as u32),
                };
                ConstraintRec {
                    kind,
                    driven: c.driven,
                    enabled: c.enabled,
                }
            })
            .collect();
        SketchRecipe {
            vars: sk.vars.values.clone(),
            fixed: sk.vars.fixed.clone(),
            entities,
            constraints,
            // dimension formulas stay live: the document's parameters drive the value
            bindings: sk
                .exprs
                .iter()
                .filter_map(|(var, text)| {
                    Some(VarBinding {
                        var: *var as u32,
                        value: Param::expr(text).ok()?,
                    })
                })
                .collect(),
        }
    }

    /// Rebuild the sketch (without applying bindings). Entity `i` of the recipe is returned at `ids[i]`.
    pub fn to_sketch(&self) -> Result<(Sketch, Vec<EntityId>), RecipeError> {
        if self.vars.len() != self.fixed.len() {
            return Err(RecipeError("vars and fixed differ in length".into()));
        }
        let nv = self.vars.len();
        let var = |v: u32| -> Result<VarId, RecipeError> {
            if (v as usize) < nv {
                Ok(VarId(v as usize))
            } else {
                Err(RecipeError(format!("variable {v} does not exist")))
            }
        };
        let mut sk = Sketch::new();
        sk.vars.values = self.vars.clone();
        sk.vars.fixed = self.fixed.clone();
        // Entities are built dependencies first, whatever order they are listed in (files written before
        // the recorder sorted them can list a line ahead of its end points).
        let n = self.entities.len();
        let mut order: Vec<usize> = Vec::with_capacity(n);
        let mut state = vec![0u8; n]; // 0 new, 1 on the stack, 2 done
        for start in 0..n {
            let mut stack = vec![(start, false)];
            while let Some((i, expanded)) = stack.pop() {
                if expanded {
                    state[i] = 2;
                    order.push(i);
                    continue;
                }
                if state[i] == 2 {
                    continue;
                }
                if state[i] == 1 {
                    return Err(RecipeError(format!("entity {i} depends on itself")));
                }
                state[i] = 1;
                stack.push((i, true));
                for k in self.entities[i].kind.deps() {
                    let k = k as usize;
                    if k >= n {
                        return Err(RecipeError(format!(
                            "entity {i} refers to entity {k}, which does not exist"
                        )));
                    }
                    if state[k] == 1 {
                        return Err(RecipeError(format!("entity {i} depends on itself")));
                    }
                    if state[k] == 0 {
                        stack.push((k, false));
                    }
                }
            }
        }
        let mut made: Vec<Option<EntityId>> = vec![None; n];
        for &i in &order {
            let e = &self.entities[i];
            let get = |made: &Vec<Option<EntityId>>, k: u32| -> Result<EntityId, RecipeError> {
                made.get(k as usize).copied().flatten().ok_or_else(|| {
                    RecipeError(format!(
                        "entity {i} refers to entity {k}, which does not come before it"
                    ))
                })
            };
            let kind = match &e.kind {
                EntityKindRec::Point { x, y } => EntityKind::Point(PointVars {
                    x: var(*x)?,
                    y: var(*y)?,
                }),
                EntityKindRec::Line { p1, p2 } => EntityKind::Line {
                    p1: get(&made, *p1)?,
                    p2: get(&made, *p2)?,
                },
                EntityKindRec::Circle { center, radius } => EntityKind::Circle {
                    center: get(&made, *center)?,
                    radius: var(*radius)?,
                },
                EntityKindRec::Arc { center, start, end } => EntityKind::Arc {
                    center: get(&made, *center)?,
                    start: get(&made, *start)?,
                    end: get(&made, *end)?,
                },
                EntityKindRec::Ellipse {
                    center,
                    major_end,
                    minor_radius,
                } => EntityKind::Ellipse {
                    center: get(&made, *center)?,
                    major_end: get(&made, *major_end)?,
                    minor_radius: var(*minor_radius)?,
                },
                EntityKindRec::Spline { degree, control } => EntityKind::Spline {
                    degree: *degree as usize,
                    control: control
                        .iter()
                        .map(|k| get(&made, *k))
                        .collect::<Result<_, _>>()?,
                },
                EntityKindRec::FitSpline { points } => EntityKind::FitSpline {
                    points: points
                        .iter()
                        .map(|k| get(&made, *k))
                        .collect::<Result<_, _>>()?,
                },
            };
            made[i] = Some(sk.entities.insert(Entity {
                kind,
                construction: e.construction,
            }));
        }
        let ids: Vec<EntityId> = made
            .into_iter()
            .map(|m| m.expect("every entity built"))
            .collect();
        let ne = ids.len();
        let ent = |k: u32| -> Result<EntityId, RecipeError> {
            if (k as usize) < ne {
                Ok(ids[k as usize])
            } else {
                Err(RecipeError(format!("entity {k} does not exist")))
            }
        };
        for c in &self.constraints {
            use ConstraintKind as K;
            use ConstraintKindRec as R;
            let kind = match &c.kind {
                R::Coincident(a, b) => K::Coincident {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::PointOnCurve(p, c) => K::PointOnCurve {
                    point: ent(*p)?,
                    curve: ent(*c)?,
                },
                R::Horizontal(a) => K::Horizontal(ent(*a)?),
                R::Vertical(a) => K::Vertical(ent(*a)?),
                R::HorizontalPts(a, b) => K::HorizontalPts {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::VerticalPts(a, b) => K::VerticalPts {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::Parallel(a, b) => K::Parallel {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::Perpendicular(a, b) => K::Perpendicular {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::Collinear(a, b) => K::Collinear {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::Concentric(a, b) => K::Concentric {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::EqualLength(a, b) => K::EqualLength {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::EqualRadius(a, b) => K::EqualRadius {
                    a: ent(*a)?,
                    b: ent(*b)?,
                },
                R::Midpoint(p, l) => K::Midpoint {
                    point: ent(*p)?,
                    line: ent(*l)?,
                },
                R::Symmetric(a, b, x) => K::Symmetric {
                    a: ent(*a)?,
                    b: ent(*b)?,
                    axis: ent(*x)?,
                },
                R::Fix(a) => K::Fix(ent(*a)?),
                R::Tangent(a, b, pos) => K::Tangent {
                    a: ent(*a)?,
                    b: ent(*b)?,
                    branch: tb(*pos),
                },
                R::Joint {
                    a,
                    a_end,
                    b,
                    b_end,
                    level,
                    positive,
                } => K::Joint {
                    a: ent(*a)?,
                    a_end: if *a_end {
                        CurveEnd::End
                    } else {
                        CurveEnd::Start
                    },
                    b: ent(*b)?,
                    b_end: if *b_end {
                        CurveEnd::End
                    } else {
                        CurveEnd::Start
                    },
                    level: match level {
                        0 => Continuity::G0,
                        1 => Continuity::G1,
                        _ => Continuity::G2,
                    },
                    branch: tb(*positive),
                },
                R::Distance {
                    a,
                    b,
                    value,
                    positive,
                } => K::Distance {
                    a: ent(*a)?,
                    b: ent(*b)?,
                    value: var(*value)?,
                    branch: tb(*positive),
                },
                R::Angle { a, b, value } => K::Angle {
                    a: ent(*a)?,
                    b: ent(*b)?,
                    value: var(*value)?,
                },
                R::Radius(c, v) => K::Radius {
                    circle: ent(*c)?,
                    value: var(*v)?,
                },
                R::Diameter(c, v) => K::Diameter {
                    circle: ent(*c)?,
                    value: var(*v)?,
                },
            };
            sk.constraints.insert(Constraint {
                kind,
                driven: c.driven,
                enabled: c.enabled,
            });
        }
        sk.touch();
        Ok((sk, ids))
    }
}
