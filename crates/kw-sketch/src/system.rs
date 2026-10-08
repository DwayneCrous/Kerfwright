//! Lowering of sketch constraints to scalar residuals and the compiled
//! [`System`] the solvers operate on.

use crate::constraint::*;
use crate::entity::*;
use crate::residual::{self, Jac, JointSpec, Kind, Rad, Residual, MAXR, MAXV};
use crate::sketch::{Sketch, SketchError};

/// Result of lowering one constraint.
pub struct Lowered {
    pub res: Vec<Residual>,
    /// Variables the constraint pins (`Fix`, dimension values).
    pub fixed: Vec<VarId>,
}

struct Slots<'a> {
    sk: &'a Sketch,
    vars: Vec<u32>,
}

impl<'a> Slots<'a> {
    fn new(sk: &'a Sketch) -> Self {
        Self {
            sk,
            vars: Vec::new(),
        }
    }

    fn var(&mut self, v: VarId) -> u8 {
        let s = self.vars.len() as u8;
        self.vars.push(v.0 as u32);
        s
    }

    fn pt(&mut self, e: EntityId) -> Result<u8, SketchError> {
        let p = self.sk.point_vars(e)?;
        let s = self.vars.len() as u8;
        self.vars.push(p.x.0 as u32);
        self.vars.push(p.y.0 as u32);
        Ok(s)
    }

    /// Slots of a line's two endpoints.
    fn line(&mut self, e: EntityId) -> Result<(u8, u8), SketchError> {
        let (a, b) = self.sk.line_points(e)?;
        Ok((self.pt(a)?, self.pt(b)?))
    }

    fn centre_of(&self, e: EntityId) -> Result<EntityId, SketchError> {
        match self.sk.kind(e)? {
            EntityKind::Circle { center, .. }
            | EntityKind::Arc { center, .. }
            | EntityKind::Ellipse { center, .. } => Ok(*center),
            _ => Err(SketchError::WrongEntity(
                "expected a circle, arc or ellipse",
            )),
        }
    }

    /// Radius source for a circle/arc whose centre already sits at `c_slot`.
    fn rad_with_centre(&mut self, e: EntityId, c_slot: u8) -> Result<Rad, SketchError> {
        match self.sk.kind(e)?.clone() {
            EntityKind::Circle { radius, .. } => Ok(Rad::Var(self.var(radius))),
            EntityKind::Arc { start, .. } => {
                let s = self.pt(start)?;
                Ok(Rad::Pt { c: c_slot, s })
            }
            _ => Err(SketchError::WrongEntity("expected a circle or arc")),
        }
    }

    /// Radius source that allocates its own centre slot when needed.
    fn rad_alone(&mut self, e: EntityId) -> Result<Rad, SketchError> {
        match self.sk.kind(e)?.clone() {
            EntityKind::Circle { radius, .. } => Ok(Rad::Var(self.var(radius))),
            EntityKind::Arc { center, start, .. } => {
                let c = self.pt(center)?;
                let s = self.pt(start)?;
                Ok(Rad::Pt { c, s })
            }
            _ => Err(SketchError::WrongEntity("expected a circle or arc")),
        }
    }

    fn finish(self, kind: Kind) -> Residual {
        Residual {
            kind,
            vars: self.vars,
        }
    }

    fn end_of(
        &mut self,
        curve: EntityId,
        end: CurveEnd,
    ) -> Result<(residual::EndLin, f64), SketchError> {
        let at_end = end == CurveEnd::End;
        let sign = if at_end { 1.0 } else { -1.0 };
        let lin = match self.sk.kind(curve)?.clone() {
            EntityKind::Line { p1, p2 } => {
                let (s1, s2) = (self.pt(p1)?, self.pt(p2)?);
                residual::line_end(s1, s2, at_end)
            }
            EntityKind::Arc {
                center,
                start,
                end: e,
            } => {
                let (sc, ss, se) = (self.pt(center)?, self.pt(start)?, self.pt(e)?);
                residual::arc_end(sc, ss, se, at_end)
            }
            EntityKind::Spline { degree, control } => {
                let n = control.len();
                let mut err = None;
                let lin =
                    residual::spline_end(degree, n, at_end, &mut |i| match self.pt(control[i]) {
                        Ok(s) => s,
                        Err(e) => {
                            err = Some(e);
                            0
                        }
                    });
                if let Some(e) = err {
                    return Err(e);
                }
                lin
            }
            _ => {
                return Err(SketchError::Unsupported(
                    "joints need lines, arcs or splines",
                ))
            }
        };
        Ok((lin, sign))
    }
}

/// Lower a constraint to residuals (empty for `driven` dimensions).
pub fn lower(sk: &Sketch, c: &Constraint) -> Result<Lowered, SketchError> {
    let mut res: Vec<Residual> = Vec::new();
    let mut fixed: Vec<VarId> = Vec::new();
    match &c.kind {
        ConstraintKind::Coincident { a, b } => {
            let mut s = Slots::new(sk);
            s.pt(*a)?;
            s.pt(*b)?;
            res.push(s.finish(Kind::Pt2Pt));
        }
        ConstraintKind::PointOnCurve { point, curve } => {
            let mut s = Slots::new(sk);
            match sk.kind(*curve)?.clone() {
                EntityKind::Line { .. } => {
                    s.pt(*point)?;
                    s.line(*curve)?;
                    res.push(s.finish(Kind::OnLine));
                }
                EntityKind::Circle { .. } | EntityKind::Arc { .. } => {
                    s.pt(*point)?;
                    let c_slot = s.pt(s.centre_of(*curve)?)?;
                    let rad = s.rad_with_centre(*curve, c_slot)?;
                    res.push(s.finish(Kind::OnCircle(rad)));
                }
                EntityKind::Ellipse {
                    center,
                    major_end,
                    minor_radius,
                } => {
                    s.pt(*point)?;
                    s.pt(center)?;
                    s.pt(major_end)?;
                    s.var(minor_radius);
                    res.push(s.finish(Kind::OnEllipse));
                }
                _ => {
                    return Err(SketchError::Unsupported(
                        "point-on-curve supports lines, circles, arcs, ellipses",
                    ))
                }
            }
        }
        ConstraintKind::Horizontal(l) | ConstraintKind::Vertical(l) => {
            let (a, b) = sk.line_points(*l)?;
            let (pa, pb) = (sk.point_vars(a)?, sk.point_vars(b)?);
            let horizontal = matches!(c.kind, ConstraintKind::Horizontal(_));
            let mut s = Slots::new(sk);
            if horizontal {
                s.var(pa.y);
                s.var(pb.y);
            } else {
                s.var(pa.x);
                s.var(pb.x);
            }
            res.push(s.finish(Kind::Diff));
        }
        ConstraintKind::HorizontalPts { a, b } | ConstraintKind::VerticalPts { a, b } => {
            let (pa, pb) = (sk.point_vars(*a)?, sk.point_vars(*b)?);
            let horizontal = matches!(c.kind, ConstraintKind::HorizontalPts { .. });
            let mut s = Slots::new(sk);
            if horizontal {
                s.var(pa.y);
                s.var(pb.y);
            } else {
                s.var(pa.x);
                s.var(pb.x);
            }
            res.push(s.finish(Kind::Diff));
        }
        ConstraintKind::Parallel { a, b }
        | ConstraintKind::Perpendicular { a, b }
        | ConstraintKind::EqualLength { a, b } => {
            let mut s = Slots::new(sk);
            s.line(*a)?;
            s.line(*b)?;
            let kind = match c.kind {
                ConstraintKind::Parallel { .. } => Kind::Parallel,
                ConstraintKind::Perpendicular { .. } => Kind::Perp,
                _ => Kind::EqLen,
            };
            res.push(s.finish(kind));
        }
        ConstraintKind::Collinear { a, b } => {
            let (a1, a2) = sk.line_points(*a)?;
            for p in [a1, a2] {
                let mut s = Slots::new(sk);
                s.pt(p)?;
                s.line(*b)?;
                res.push(s.finish(Kind::OnLine));
            }
        }
        ConstraintKind::Concentric { a, b } => {
            let mut s = Slots::new(sk);
            let (ca, cb) = (s.centre_of(*a)?, s.centre_of(*b)?);
            s.pt(ca)?;
            s.pt(cb)?;
            res.push(s.finish(Kind::Pt2Pt));
        }
        ConstraintKind::EqualRadius { a, b } => {
            let mut s = Slots::new(sk);
            let r1 = s.rad_alone(*a)?;
            let r2 = s.rad_alone(*b)?;
            res.push(s.finish(Kind::EqRad(r1, r2)));
        }
        ConstraintKind::Midpoint { point, line } => {
            let mut s = Slots::new(sk);
            s.pt(*point)?;
            s.line(*line)?;
            res.push(s.finish(Kind::Midpoint));
        }
        ConstraintKind::Symmetric { a, b, axis } => {
            let mut s = Slots::new(sk);
            s.pt(*a)?;
            s.pt(*b)?;
            s.line(*axis)?;
            res.push(s.finish(Kind::Symmetric));
        }
        ConstraintKind::Fix(e) => fixed.extend(sk.entity_vars(*e)?),
        ConstraintKind::Tangent { a, b, branch } => {
            let a_line = matches!(sk.kind(*a)?, EntityKind::Line { .. });
            let b_line = matches!(sk.kind(*b)?, EntityKind::Line { .. });
            let mut s = Slots::new(sk);
            match (a_line, b_line) {
                (true, true) => {
                    return Err(SketchError::Unsupported(
                        "two lines cannot be tangent; use parallel",
                    ))
                }
                (true, false) | (false, true) => {
                    let (line, circ) = if a_line { (*a, *b) } else { (*b, *a) };
                    s.line(line)?;
                    let c_slot = s.pt(s.centre_of(circ)?)?;
                    let r = s.rad_with_centre(circ, c_slot)?;
                    res.push(s.finish(Kind::TanLineCirc {
                        s: branch.sign(),
                        r,
                    }));
                }
                (false, false) => {
                    let c1 = s.pt(s.centre_of(*a)?)?;
                    s.pt(s.centre_of(*b)?)?;
                    let r1 = s.rad_with_centre(*a, c1)?;
                    let r2 = s.rad_with_centre(*b, c1 + 2)?;
                    res.push(s.finish(Kind::TanCircCirc {
                        s: branch.sign(),
                        r1,
                        r2,
                    }));
                }
            }
        }
        ConstraintKind::Joint {
            a,
            a_end,
            b,
            b_end,
            level,
            branch,
        } => {
            let mut s = Slots::new(sk);
            let (la, sa) = s.end_of(*a, *a_end)?;
            let (lb, sb) = s.end_of(*b, *b_end)?;
            let lvl = match level {
                Continuity::G0 => 0,
                Continuity::G1 => 1,
                Continuity::G2 => 2,
            };
            if s.vars.len() > MAXV {
                return Err(SketchError::Unsupported("joint uses too many variables"));
            }
            // position equations vanish if both ends are literally the same point
            let p_vars = |lin: &residual::EndLin| -> Vec<(u32, u32)> {
                lin.terms
                    .iter()
                    .filter(|t| t.p.iter().any(|c| *c != 0.0))
                    .map(|t| (s.vars[t.slot as usize], s.vars[t.slot as usize + 1]))
                    .collect()
            };
            let shared = {
                let (pa, pb) = (p_vars(&la), p_vars(&lb));
                pa.len() == 1 && pb.len() == 1 && pa[0] == pb[0]
            };
            if shared && lvl == 0 {
                // nothing left to enforce
            } else {
                res.push(s.finish(Kind::Joint(Box::new(JointSpec {
                    a: la,
                    b: lb,
                    sign_a: sa,
                    sign_b: sb,
                    level: lvl,
                    g0: !shared,
                    branch: branch.sign(),
                }))));
            }
        }
        ConstraintKind::Distance {
            a,
            b,
            value,
            branch,
        } => {
            fixed.push(*value);
            let ka = sk.kind(*a)?.clone();
            let kb = sk.kind(*b)?.clone();
            match (ka, kb) {
                (EntityKind::Point(_), EntityKind::Point(_)) => {
                    let mut s = Slots::new(sk);
                    s.pt(*a)?;
                    s.pt(*b)?;
                    s.var(*value);
                    res.push(s.finish(Kind::DistPP));
                }
                (EntityKind::Point(_), EntityKind::Line { .. })
                | (EntityKind::Line { .. }, EntityKind::Point(_)) => {
                    let (pt, line) = if matches!(sk.kind(*a)?, EntityKind::Point(_)) {
                        (*a, *b)
                    } else {
                        (*b, *a)
                    };
                    let mut s = Slots::new(sk);
                    s.pt(pt)?;
                    s.line(line)?;
                    s.var(*value);
                    res.push(s.finish(Kind::DistPL { s: branch.sign() }));
                }
                (EntityKind::Line { .. }, EntityKind::Line { .. }) => {
                    // parallel lines: parallel + signed distance of a's start to b
                    let mut p = Slots::new(sk);
                    p.line(*a)?;
                    p.line(*b)?;
                    res.push(p.finish(Kind::Parallel));
                    let (a1, _) = sk.line_points(*a)?;
                    let mut s = Slots::new(sk);
                    s.pt(a1)?;
                    s.line(*b)?;
                    s.var(*value);
                    res.push(s.finish(Kind::DistPL { s: branch.sign() }));
                }
                _ => {
                    return Err(SketchError::Unsupported(
                        "distance needs points and/or lines",
                    ))
                }
            }
        }
        ConstraintKind::Angle { a, b, value } => {
            fixed.push(*value);
            let mut s = Slots::new(sk);
            s.line(*a)?;
            s.line(*b)?;
            s.var(*value);
            res.push(s.finish(Kind::Angle));
        }
        ConstraintKind::Radius { circle, value } | ConstraintKind::Diameter { circle, value } => {
            fixed.push(*value);
            let scale = if matches!(c.kind, ConstraintKind::Radius { .. }) {
                1.0
            } else {
                2.0
            };
            let mut s = Slots::new(sk);
            let rad = s.rad_alone(*circle)?;
            let v = s.var(*value);
            res.push(s.finish(Kind::RadVal { rad, scale, v }));
        }
    }
    if c.driven {
        res.clear();
    }
    Ok(Lowered { res, fixed })
}

/// Read access to the variable vector; implemented for slices and for the
/// raw shared view used by the parallel solver.
pub trait XAccess {
    fn get(&self, i: usize) -> f64;
}

impl XAccess for [f64] {
    #[inline(always)]
    fn get(&self, i: usize) -> f64 {
        self[i]
    }
}

/// A sketch compiled to scalar residuals.
#[derive(Clone, Debug)]
pub struct System {
    pub n_vars: usize,
    /// Variables that are constants for the solver (user-fixed, `Fix`, dimension values).
    pub fixed: Vec<bool>,
    pub res: Vec<Residual>,
    /// Constraint each residual came from (`None` for implicit arc consistency).
    pub res_src: Vec<Option<ConstraintId>>,
    /// First row of each residual; `row_start[res.len()] == n_rows`.
    pub row_start: Vec<usize>,
    pub n_rows: usize,
    /// Residual index of each row.
    pub row_res: Vec<u32>,
    /// Characteristic length of the sketch (≥ 1).
    pub scale: f64,
    /// Per-row weight making residuals dimensionless (see `Kind::row_weight`).
    pub row_w: Vec<f64>,
}

impl System {
    /// Compile the whole sketch.
    pub fn build(sk: &Sketch) -> Result<System, SketchError> {
        Self::build_filtered(sk, &|_| true)
    }

    /// Compile only constraints accepted by `keep` (used by conflict analysis).
    pub fn build_filtered(
        sk: &Sketch,
        keep: &dyn Fn(ConstraintId) -> bool,
    ) -> Result<System, SketchError> {
        let n_vars = sk.vars.values.len();
        let mut fixed = sk.vars.fixed.clone();
        let mut res = Vec::new();
        let mut res_src = Vec::new();
        for (id, c) in &sk.constraints {
            if !c.enabled || !keep(id) {
                continue;
            }
            let l = lower(sk, c)?;
            for v in l.fixed {
                fixed[v.0] = true;
            }
            for r in l.res {
                res.push(r);
                res_src.push(Some(id));
            }
        }
        // implicit: arcs keep one radius
        for e in sk.entities.values() {
            if let EntityKind::Arc { center, start, end } = &e.kind {
                let mut s = Slots::new(sk);
                s.pt(*center)?;
                s.pt(*start)?;
                s.pt(*end)?;
                res.push(s.finish(Kind::ArcCons));
                res_src.push(None);
            }
        }
        let mut row_start = Vec::with_capacity(res.len() + 1);
        let mut row_res = Vec::new();
        let mut n_rows = 0;
        for (i, r) in res.iter().enumerate() {
            row_start.push(n_rows);
            for _ in 0..r.kind.arity() {
                row_res.push(i as u32);
            }
            n_rows += r.kind.arity();
        }
        row_start.push(n_rows);
        let scale = match sk.bounds() {
            Some((lo, hi)) => (hi.0 - lo.0).abs().max((hi.1 - lo.1).abs()).max(1.0),
            None => 1.0,
        };
        let mut row_w = Vec::with_capacity(n_rows);
        for r in &res {
            for eq in 0..r.kind.arity() {
                row_w.push(r.kind.row_weight(eq, scale));
            }
        }
        Ok(System {
            n_vars,
            fixed,
            res,
            res_src,
            row_start,
            n_rows,
            row_res,
            scale,
            row_w,
        })
    }

    #[inline]
    fn gather<X: XAccess + ?Sized>(&self, i: usize, x: &X) -> [f64; MAXV] {
        let mut v = [0.0; MAXV];
        for (s, g) in self.res[i].vars.iter().enumerate() {
            v[s] = x.get(*g as usize);
        }
        v
    }

    /// Evaluate residual `i` at `x` into `out[..arity]`.
    #[inline]
    pub fn eval_res<X: XAccess + ?Sized>(&self, i: usize, x: &X, out: &mut [f64; MAXR]) {
        let v = self.gather(i, x);
        residual::eval(&self.res[i].kind, &v, out);
    }

    /// Dense local Jacobian (rows = equations of residual `i`, columns = local slots).
    #[inline]
    pub fn jac_res<X: XAccess + ?Sized>(&self, i: usize, x: &X, j: &mut Jac) {
        let v = self.gather(i, x);
        for row in j.iter_mut() {
            *row = [0.0; MAXV];
        }
        residual::jac(&self.res[i].kind, &v, j);
    }

    /// All residuals at `x` (length `n_rows`).
    pub fn residuals(&self, x: &[f64], out: &mut [f64]) {
        let mut tmp = [0.0; MAXR];
        for i in 0..self.res.len() {
            self.eval_res(i, x, &mut tmp);
            let a = self.row_start[i];
            let k = self.res[i].kind.arity();
            out[a..a + k].copy_from_slice(&tmp[..k]);
        }
    }

    /// Largest absolute residual.
    pub fn max_residual(&self, x: &[f64]) -> f64 {
        let mut tmp = [0.0; MAXR];
        let mut worst = 0.0f64;
        for i in 0..self.res.len() {
            self.eval_res(i, x, &mut tmp);
            for t in tmp.iter().take(self.res[i].kind.arity()) {
                worst = worst.max(t.abs());
            }
        }
        worst
    }

    /// Constraint that owns `row`.
    pub fn row_constraint(&self, row: usize) -> Option<ConstraintId> {
        self.res_src[self.row_res[row] as usize]
    }

    /// Unknown variables touched by `row` (structural pattern), global indices.
    pub fn row_pattern(&self, row: usize, out: &mut Vec<u32>) {
        let ri = self.row_res[row] as usize;
        let eq = row - self.row_start[ri];
        let r = &self.res[ri];
        let mut slots = Vec::new();
        r.kind.pattern(eq, r.vars.len(), &mut slots);
        out.clear();
        for s in slots {
            let g = r.vars[s as usize];
            if !self.fixed[g as usize] {
                out.push(g);
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

impl System {
    /// `false` if residual `i` is satisfied only by a degenerate configuration
    /// (a zero-length line it depends on, a non-positive radius).
    pub fn residual_valid<X: XAccess + ?Sized>(&self, i: usize, x: &X) -> bool {
        use crate::residual::Rad;
        let v = self.gather(i, x);
        let eps = 1e-6 * self.scale;
        let line_ok = |a: usize, b: usize| (v[b] - v[a]).hypot(v[b + 1] - v[a + 1]) > eps;
        let rad_ok = |r: &Rad| match r {
            Rad::Var(s) => v[*s as usize] > 1e-9 * self.scale,
            Rad::Pt { c, s } => {
                let (c, s) = (*c as usize, *s as usize);
                (v[s] - v[c]).hypot(v[s + 1] - v[c + 1]) > 1e-9 * self.scale
            }
        };
        match &self.res[i].kind {
            Kind::Parallel | Kind::Perp | Kind::EqLen | Kind::Angle => {
                line_ok(0, 2) && line_ok(4, 6)
            }
            Kind::OnLine => line_ok(2, 4),
            Kind::Symmetric => line_ok(4, 6),
            Kind::TanLineCirc { r, .. } => line_ok(0, 2) && rad_ok(r),
            Kind::DistPL { .. } => line_ok(2, 4),
            Kind::OnCircle(r) => rad_ok(r),
            Kind::RadVal { rad, .. } => rad_ok(rad),
            Kind::EqRad(a, b) => rad_ok(a) && rad_ok(b),
            Kind::TanCircCirc { r1, r2, .. } => rad_ok(r1) && rad_ok(r2),
            _ => true,
        }
    }

    /// `false` if residual `i` sits on the wrong branch at `x` (a tangency or
    /// joint that has flipped sides, a radius that went non-positive, …).
    pub fn residual_branch_ok<X: XAccess + ?Sized>(&self, i: usize, x: &X) -> bool {
        use crate::residual::Rad;
        if !self.residual_valid(i, x) {
            return false;
        }
        let v = self.gather(i, x);
        let rad_var_ok = |r: &Rad| match r {
            Rad::Var(s) => v[*s as usize] > 1e-9,
            Rad::Pt { .. } => true,
        };
        match &self.res[i].kind {
            Kind::OnCircle(r) => rad_var_ok(r),
            Kind::RadVal { rad, .. } => rad_var_ok(rad),
            Kind::EqRad(a, b) => rad_var_ok(a) && rad_var_ok(b),
            Kind::TanLineCirc { s, r } => {
                let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
                let (ex, ey) = (v[4] - v[0], v[5] - v[1]);
                rad_var_ok(r) && s * (dx * ey - dy * ex) >= -1e-9 * self.scale * self.scale
            }
            Kind::TanCircCirc { r1, r2, .. } => rad_var_ok(r1) && rad_var_ok(r2),
            Kind::DistPL { s } => {
                let (ux, uy) = (v[4] - v[2], v[5] - v[3]);
                let (ex, ey) = (v[0] - v[2], v[1] - v[3]);
                s * (ux * ey - uy * ex) >= -1e-9 * self.scale * self.scale
            }
            Kind::Angle => {
                let (ux, uy) = (v[2] - v[0], v[3] - v[1]);
                let (wx, wy) = (v[6] - v[4], v[7] - v[5]);
                let (lu, lw) = ((ux * ux + uy * uy).sqrt(), (wx * wx + wy * wy).sqrt());
                let (n1x, n1y, n2x, n2y) = (ux / lu, uy / lu, wx / lw, wy / lw);
                let (sn, cs) = (n1x * n2y - n1y * n2x, n1x * n2x + n1y * n2y);
                let (st, ct) = v[8].sin_cos();
                cs * ct + sn * st > -1e-9
            }
            Kind::Joint(j) if j.level >= 1 => {
                let (_, ta, _) = j.a.eval(&v);
                let (_, tb, _) = j.b.eval(&v);
                let dot = j.sign_a * j.sign_b * (ta.0 * tb.0 + ta.1 * tb.1);
                let (la, lb) = (
                    (ta.0 * ta.0 + ta.1 * ta.1).sqrt(),
                    (tb.0 * tb.0 + tb.1 * tb.1).sqrt(),
                );
                j.branch * dot >= -1e-9 * la * lb
            }
            _ => true,
        }
    }
}
