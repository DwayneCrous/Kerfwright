use crate::format::{DxfDoc, DxfEntity, P2};
use kw_nurbs::KnotVector;
use kw_sketch::geom::arc_params;
use kw_sketch::{EntityId, EntityKind, Sketch};
use std::collections::HashSet;
use std::f64::consts::TAU;

const CONSTRUCTION_LAYER: &str = "CONSTRUCTION";

/// Convert a sketch to DXF entities (construction geometry goes to its own layer).
pub fn export_sketch(sk: &Sketch) -> DxfDoc {
    let mut referenced: HashSet<EntityId> = HashSet::new();
    for id in sk.entities.keys() {
        referenced.extend(sk.children(id));
    }
    let mut doc = DxfDoc::default();
    for (id, e) in &sk.entities {
        let layer = if e.construction {
            CONSTRUCTION_LAYER
        } else {
            "0"
        }
        .to_string();
        match &e.kind {
            EntityKind::Point(_) => {
                if !referenced.contains(&id) {
                    doc.entities.push(DxfEntity::Point {
                        layer,
                        p: sk.point(id).unwrap(),
                    });
                }
            }
            EntityKind::Line { p1, p2 } => doc.entities.push(DxfEntity::Line {
                layer,
                a: sk.point(*p1).unwrap(),
                b: sk.point(*p2).unwrap(),
            }),
            EntityKind::Circle { center, radius } => doc.entities.push(DxfEntity::Circle {
                layer,
                c: sk.point(*center).unwrap(),
                r: sk.value(*radius),
            }),
            EntityKind::Arc { .. } => {
                let (c, r, a0, a1) = arc_params(sk, id).unwrap();
                doc.entities.push(DxfEntity::Arc {
                    layer,
                    c,
                    r,
                    a0: a0.to_degrees(),
                    a1: a1.to_degrees(),
                });
            }
            EntityKind::Ellipse {
                center,
                major_end,
                minor_radius,
            } => {
                let (c, m) = (sk.point(*center).unwrap(), sk.point(*major_end).unwrap());
                let major = (m.0 - c.0, m.1 - c.1);
                let a = major.0.hypot(major.1);
                doc.entities.push(DxfEntity::Ellipse {
                    layer,
                    c,
                    major,
                    ratio: sk.value(*minor_radius) / a,
                    t0: 0.0,
                    t1: TAU,
                });
            }
            EntityKind::Spline { degree, control } => {
                let ctrl: Vec<P2> = control.iter().map(|p| sk.point(*p).unwrap()).collect();
                let kv = KnotVector::clamped_uniform(*degree, ctrl.len()).unwrap();
                doc.entities.push(DxfEntity::Spline {
                    layer,
                    degree: *degree,
                    knots: kv.as_slice().to_vec(),
                    weights: vec![1.0; ctrl.len()],
                    ctrl,
                    fit: Vec::new(),
                    closed: false,
                });
            }
            EntityKind::FitSpline { .. } => {
                if let Ok(curve) = kw_sketch::geom::spline_curve(sk, id) {
                    doc.entities.push(DxfEntity::Spline {
                        layer,
                        degree: curve.degree(),
                        knots: curve.knots().as_slice().to_vec(),
                        ctrl: curve.points().iter().map(|p| (p.x, p.y)).collect(),
                        weights: curve.weights().to_vec(),
                        fit: match sk.kind(id) {
                            Ok(EntityKind::FitSpline { points }) => {
                                points.iter().map(|p| sk.point(*p).unwrap()).collect()
                            }
                            _ => Vec::new(),
                        },
                        closed: false,
                    });
                }
            }
        }
    }
    doc
}

#[derive(Debug, Clone)]
pub struct ImportOptions {
    /// End points closer than this are merged into one point entity.
    pub weld_tolerance: f64,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            weld_tolerance: 1e-6,
        }
    }
}

struct Importer {
    sk: Sketch,
    points: Vec<(P2, EntityId)>,
    tol: f64,
}

impl Importer {
    fn weld(&mut self, p: P2) -> EntityId {
        if let Some((_, id)) = self
            .points
            .iter()
            .find(|(q, _)| (q.0 - p.0).hypot(q.1 - p.1) <= self.tol)
        {
            return *id;
        }
        let id = self.sk.add_point(p.0, p.1);
        self.points.push((p, id));
        id
    }
    fn fresh(&mut self, p: P2) -> EntityId {
        self.sk.add_point(p.0, p.1)
    }
    fn mark(&mut self, e: EntityId, layer: &str) {
        self.sk.set_construction(e, layer == CONSTRUCTION_LAYER);
    }
    fn line(&mut self, layer: &str, a: P2, b: P2) {
        if (a.0 - b.0).hypot(a.1 - b.1) <= 1e-12 {
            return;
        }
        let (pa, pb) = (self.weld(a), self.weld(b));
        let l = self.sk.add_line(pa, pb);
        self.mark(l, layer);
    }
    fn arc(&mut self, layer: &str, c: P2, r: f64, a0: f64, a1: f64) {
        let ctr = self.fresh(c);
        let s = self.weld((c.0 + r * a0.cos(), c.1 + r * a0.sin()));
        let e = self.weld((c.0 + r * a1.cos(), c.1 + r * a1.sin()));
        let arc = self.sk.add_arc(ctr, s, e);
        self.mark(arc, layer);
    }
}

/// Build a (constraint-free) sketch from DXF entities. LWPOLYLINE bulges become
/// arcs; coincident end points are welded; partial ellipses and splines that are
/// not clamped-uniform B-splines are imported as fit splines through sampled points.
pub fn import_sketch(doc: &DxfDoc, opts: &ImportOptions) -> Sketch {
    let mut im = Importer {
        sk: Sketch::new(),
        points: Vec::new(),
        tol: opts.weld_tolerance,
    };
    for e in &doc.entities {
        match e {
            DxfEntity::Point { layer, p } => {
                let id = im.fresh(*p);
                im.mark(id, layer);
            }
            DxfEntity::Line { layer, a, b } => im.line(layer, *a, *b),
            DxfEntity::Circle { layer, c, r } => {
                if *r > 0.0 {
                    let ctr = im.fresh(*c);
                    let id = im.sk.add_circle(ctr, *r);
                    im.mark(id, layer);
                }
            }
            DxfEntity::Arc {
                layer,
                c,
                r,
                a0,
                a1,
            } => {
                if *r > 0.0 {
                    im.arc(layer, *c, *r, a0.to_radians(), a1.to_radians());
                }
            }
            DxfEntity::Ellipse {
                layer,
                c,
                major,
                ratio,
                t0,
                t1,
            } => {
                let a = major.0.hypot(major.1);
                if a <= 0.0 || *ratio <= 0.0 || *ratio > 1.0 {
                    continue;
                }
                let full = (t1 - t0 - TAU).abs() < 1e-9 && t0.abs() < 1e-9;
                let (ux, uy) = (major.0 / a, major.1 / a);
                if full {
                    let ctr = im.fresh(*c);
                    let me = im.fresh((c.0 + major.0, c.1 + major.1));
                    let id = im.sk.add_ellipse(ctr, me, a * ratio);
                    im.mark(id, layer);
                } else {
                    let n = 24;
                    let span = if t1 > t0 { t1 - t0 } else { t1 - t0 + TAU };
                    let pts: Vec<_> = (0..=n)
                        .map(|i| {
                            let t = t0 + span * i as f64 / n as f64;
                            let (px, py) = (a * t.cos(), a * ratio * t.sin());
                            let p = (c.0 + px * ux - py * uy, c.1 + px * uy + py * ux);
                            im.fresh(p)
                        })
                        .collect();
                    let id = im.sk.add_fit_spline(pts);
                    im.mark(id, layer);
                }
            }
            DxfEntity::LwPolyline {
                layer,
                closed,
                verts,
            } => {
                let n = verts.len();
                let segs = if *closed { n } else { n.saturating_sub(1) };
                for i in 0..segs {
                    let (x0, y0, bulge) = verts[i];
                    let (x1, y1, _) = verts[(i + 1) % n];
                    let (a, b) = ((x0, y0), (x1, y1));
                    if bulge == 0.0 {
                        im.line(layer, a, b);
                    } else {
                        let chord = (b.0 - a.0).hypot(b.1 - a.1);
                        if chord <= 1e-12 {
                            continue;
                        }
                        let r = chord * (1.0 + bulge * bulge) / (4.0 * bulge.abs());
                        let h = chord / 2.0 * (1.0 - bulge * bulge) / (2.0 * bulge);
                        let nrm = (-(b.1 - a.1) / chord, (b.0 - a.0) / chord);
                        let mid = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
                        let c = (mid.0 + nrm.0 * h, mid.1 + nrm.1 * h);
                        let (s, t) = if bulge > 0.0 { (a, b) } else { (b, a) };
                        let (a0, a1) = ((s.1 - c.1).atan2(s.0 - c.0), (t.1 - c.1).atan2(t.0 - c.0));
                        // use the exact end points for welding
                        let ctr = im.fresh(c);
                        let (ps, pe) = (im.weld(s), im.weld(t));
                        let _ = (a0, a1, r);
                        let arc = im.sk.add_arc(ctr, ps, pe);
                        im.mark(arc, layer);
                    }
                }
            }
            DxfEntity::Spline {
                layer,
                degree,
                knots,
                ctrl,
                weights,
                fit,
                ..
            } => {
                if fit.len() >= 2 {
                    let pts: Vec<EntityId> = fit.iter().map(|p| im.fresh(*p)).collect();
                    let id = im.sk.add_fit_spline(pts);
                    im.mark(id, layer);
                    continue;
                }
                let ours = KnotVector::clamped_uniform(*degree, ctrl.len()).ok();
                let uniform = ours.as_ref().is_some_and(|k| {
                    k.as_slice().len() == knots.len()
                        && k.as_slice()
                            .iter()
                            .zip(knots)
                            .all(|(a, b)| (a - b).abs() <= 1e-9 * (1.0 + a.abs()))
                }) && weights.iter().all(|w| (*w - 1.0).abs() < 1e-12);
                if uniform {
                    let pts: Vec<EntityId> = ctrl.iter().map(|p| im.fresh(*p)).collect();
                    if let Ok(id) = im.sk.add_spline(*degree, pts) {
                        im.mark(id, layer);
                    }
                } else if let Some(sampled) = sample_spline(*degree, knots, ctrl, weights) {
                    let pts: Vec<EntityId> = sampled.into_iter().map(|p| im.fresh(p)).collect();
                    let id = im.sk.add_fit_spline(pts);
                    im.mark(id, layer);
                }
            }
            // text carries no sketch geometry
            DxfEntity::Text { .. } => {}
        }
    }
    im.sk
}

fn sample_spline(degree: usize, knots: &[f64], ctrl: &[P2], weights: &[f64]) -> Option<Vec<P2>> {
    let kv = KnotVector::new(degree, knots.to_vec()).ok()?;
    if kv.num_ctrl() != ctrl.len() {
        return None;
    }
    let pts = ctrl
        .iter()
        .map(|p| kw_math::Point3::new(p.0, p.1, 0.0))
        .collect();
    let curve = kw_nurbs::NurbsCurve::new(kv, pts, weights.to_vec()).ok()?;
    let (a, b) = curve.domain();
    let n = (4 * ctrl.len()).max(16);
    Some(
        (0..=n)
            .map(|i| {
                let p = curve.point_at(a + (b - a) * i as f64 / n as f64);
                (p.x, p.y)
            })
            .collect(),
    )
}
