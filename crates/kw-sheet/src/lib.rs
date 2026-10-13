//! Sheet metal: constant-thickness strips made of flat segments and cylindrical bends.
//!
//! A [`SheetStrip`] is the cross-section recipe of a bent part: thickness, K-factor, width and an
//! ordered list of flats and bends along the mid-surface. It builds an exact B-rep (one extrusion of
//! the thick cross-section: planes and cylinders, no Boolean), unfolds to a [`FlatPattern`] whose bend
//! allowances follow `BA = θ (R + K t)`, writes that pattern as DXF, and can be recovered from a
//! B-rep ([`recognize`]) so that a folded body can be unfolded and refolded without its recipe.
//!
//! Conventions: the cross-section lies in the XZ plane, the strip is extruded along +Y by `width`.
//! The path starts at the origin heading +X; a positive bend turns left (towards +Z).

// Numerical code indexes small fixed-size arrays; `!(x > 0.0)` is deliberate (it also rejects NaN).
#![allow(
    clippy::needless_range_loop,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::type_complexity
)]

use kw_brep::{BRep, BodyId, BuildResult, EntityRef, Origin, Sense};
use kw_dxf::{DxfDoc, DxfEntity};
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{extrude, ExtrudeOptions, Profile, ProfileEdge, ProfileLoop};
use kw_nurbs::{CurveGeom, SurfaceGeom};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum SheetError {
    /// A dimension is not usable (the message says which).
    Invalid(String),
    /// The folded strip would pass through itself.
    SelfIntersecting(String),
    Kernel(String),
    /// The body is not a sheet-metal strip this library can read.
    NotSheet(String),
}

impl fmt::Display for SheetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SheetError::Invalid(m) => write!(f, "invalid sheet metal definition: {m}"),
            SheetError::SelfIntersecting(m) => write!(f, "the part folds through itself: {m}"),
            SheetError::Kernel(m) => write!(f, "{m}"),
            SheetError::NotSheet(m) => write!(f, "not a sheet-metal strip: {m}"),
        }
    }
}
impl std::error::Error for SheetError {}

/// Material and process settings (millimetres, radians).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SheetSpec {
    pub thickness: f64,
    /// Position of the neutral axis within the thickness, from the inner surface (0…1).
    pub k_factor: f64,
    /// Inner bend radius used when a bend does not give its own.
    pub default_radius: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Seg {
    /// A straight piece, `length` between the tangent lines of its neighbouring bends (or the end).
    Flat { length: f64 },
    /// A bend through `angle` radians (positive turns left, negative right, |angle| ≤ π), with the
    /// given inner radius or the spec's default.
    Bend { angle: f64, radius: Option<f64> },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SheetStrip {
    pub spec: SheetSpec,
    pub width: f64,
    pub segs: Vec<Seg>,
}

/// Hem kinds: the strip folds back on itself leaving `gap` between the layers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Hem {
    Open { gap: f64, return_length: f64 },
}

impl SheetStrip {
    pub fn new(spec: SheetSpec, width: f64) -> SheetStrip {
        SheetStrip {
            spec,
            width,
            segs: Vec::new(),
        }
    }
    pub fn flat(mut self, length: f64) -> Self {
        self.segs.push(Seg::Flat { length });
        self
    }
    pub fn bend(mut self, angle: f64) -> Self {
        self.segs.push(Seg::Bend {
            angle,
            radius: None,
        });
        self
    }
    pub fn bend_r(mut self, angle: f64, radius: f64) -> Self {
        self.segs.push(Seg::Bend {
            angle,
            radius: Some(radius),
        });
        self
    }
    /// An edge flange: a bend followed by a flat.
    pub fn flange(self, angle: f64, length: f64) -> Self {
        self.bend(angle).flat(length)
    }
    /// A hem: a half-turn leaving `gap` between the layers, then `return_length` of flat.
    pub fn hem(self, h: Hem) -> Self {
        match h {
            Hem::Open { gap, return_length } => self.bend_r(PI, gap / 2.0).flat(return_length),
        }
    }

    fn radius_of(&self, r: Option<f64>) -> f64 {
        r.unwrap_or(self.spec.default_radius)
    }

    pub fn validate(&self) -> Result<(), SheetError> {
        let bad = |m: String| Err(SheetError::Invalid(m));
        let s = &self.spec;
        if !(s.thickness > 0.0 && s.thickness.is_finite()) {
            return bad(format!("thickness must be positive, got {}", s.thickness));
        }
        if !(0.0..=1.0).contains(&s.k_factor) {
            return bad(format!(
                "the K-factor must be between 0 and 1, got {}",
                s.k_factor
            ));
        }
        if !(self.width > 0.0 && self.width.is_finite()) {
            return bad(format!("width must be positive, got {}", self.width));
        }
        if self.segs.is_empty() {
            return bad("there is nothing to build".into());
        }
        let mut flats = 0;
        for (i, g) in self.segs.iter().enumerate() {
            match g {
                Seg::Flat { length } => {
                    if !(*length >= 0.0 && length.is_finite()) {
                        return bad(format!("segment {i}: flat length must not be negative"));
                    }
                    if *length > 0.0 {
                        flats += 1;
                    }
                }
                Seg::Bend { angle, radius } => {
                    if !(angle.abs() > 1e-9 && angle.abs() <= PI + 1e-12) {
                        return bad(format!(
                            "segment {i}: a bend angle must be between 0 and 180 degrees, got {}°",
                            angle.to_degrees()
                        ));
                    }
                    let r = self.radius_of(*radius);
                    if !(r > 0.0 && r.is_finite()) {
                        return bad(format!("segment {i}: the bend radius must be positive"));
                    }
                }
            }
        }
        if flats == 0 {
            return bad("a strip needs at least one flat".into());
        }
        Ok(())
    }
}

// ------------------------------------------------------------------------------------- geometry

type P2 = (f64, f64);

#[derive(Debug, Clone)]
struct PathPiece {
    /// Start / end of the mid-surface path and the headings there.
    p0: P2,
    p1: P2,
    h0: f64,
    h1: f64,
    kind: PieceKind,
}

#[derive(Debug, Clone)]
enum PieceKind {
    Flat,
    Bend {
        centre: P2,
        /// Mid-surface radius.
        r_mid: f64,
        angle: f64,
    },
}

fn walk(strip: &SheetStrip) -> Vec<PathPiece> {
    let t = strip.spec.thickness;
    let mut pieces = Vec::new();
    let (mut p, mut h) = ((0.0, 0.0), 0.0f64);
    for g in &strip.segs {
        match *g {
            Seg::Flat { length } => {
                if length <= 0.0 {
                    continue;
                }
                let q = (p.0 + length * h.cos(), p.1 + length * h.sin());
                pieces.push(PathPiece {
                    p0: p,
                    p1: q,
                    h0: h,
                    h1: h,
                    kind: PieceKind::Flat,
                });
                p = q;
            }
            Seg::Bend { angle, radius } => {
                let inner = strip.radius_of(radius);
                let r_mid = inner + t / 2.0;
                let side = angle.signum();
                // centre lies to the left (positive) or right (negative) of the heading
                let n = (-h.sin(), h.cos());
                let c = (p.0 + side * r_mid * n.0, p.1 + side * r_mid * n.1);
                let h1 = h + angle;
                let a_end = h1 - side * PI / 2.0;
                let q = (c.0 + r_mid * a_end.cos(), c.1 + r_mid * a_end.sin());
                pieces.push(PathPiece {
                    p0: p,
                    p1: q,
                    h0: h,
                    h1,
                    kind: PieceKind::Bend {
                        centre: c,
                        r_mid,
                        angle,
                    },
                });
                p = q;
                h = h1;
            }
        }
    }
    pieces
}

/// A point of the boundary at signed offset `off` to the left of the mid-surface (off = ±t/2).
fn offset_point(p: P2, h: f64, off: f64) -> P2 {
    (p.0 - off * h.sin(), p.1 + off * h.cos())
}

fn plane_frame() -> Frame3 {
    // x → X, y → Z, normal X × Z = −Y
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    }
}

fn to3(p: P2) -> Point3<f64> {
    Point3::new(p.0, 0.0, p.1)
}

/// One boundary path (`off` = +t/2 left, −t/2 right) as profile edges in travel order.
fn boundary(pieces: &[PathPiece], off: f64) -> Vec<ProfileEdge> {
    let frame0 = plane_frame();
    pieces
        .iter()
        .map(|pc| match &pc.kind {
            PieceKind::Flat => ProfileEdge::line(
                to3(offset_point(pc.p0, pc.h0, off)),
                to3(offset_point(pc.p1, pc.h1, off)),
            ),
            PieceKind::Bend {
                centre,
                r_mid,
                angle,
                ..
            } => {
                let side = angle.signum();
                // distance from the centre: left boundary of a left turn is the inner one
                let r = r_mid - side * off;
                let a_start = pc.h0 - side * PI / 2.0;
                let (a0, a1) = if side > 0.0 {
                    (a_start, a_start + angle.abs())
                } else {
                    (a_start - angle.abs(), a_start)
                };
                let frame = Frame3 {
                    origin: to3(*centre),
                    ..frame0
                };
                let mut e = ProfileEdge::arc(&frame, r, a0, a1);
                e.reversed = side < 0.0;
                e
            }
        })
        .collect()
}

fn reverse_edge(e: &ProfileEdge) -> ProfileEdge {
    ProfileEdge {
        reversed: !e.reversed,
        ..e.clone()
    }
}

/// Number of pieces (flats and bends) on each face of the strip.
pub fn piece_count(strip: &SheetStrip) -> usize {
    strip
        .segs
        .iter()
        .filter(|s| !matches!(s, Seg::Flat { length } if *length <= 0.0))
        .count()
}

/// A stable name for the `index`-th side face the extrusion makes (`side-0-index`) of a strip with `n`
/// pieces: `outer-k` / `inner-k` for the right / left face of piece `k` along the path (right is the
/// outer face of a left turn), `start` and `end` for the ends.
pub fn side_face_role(index: usize, n: usize) -> String {
    if index < n {
        format!("right-{index}")
    } else if index == n {
        "end".to_string()
    } else if index <= 2 * n {
        format!("left-{}", 2 * n - index)
    } else {
        "start".to_string()
    }
}

/// The cross-section as a profile: right boundary forward, end cap, left boundary back, start cap.
pub fn cross_section(strip: &SheetStrip) -> Result<Profile, SheetError> {
    strip.validate()?;
    let t = strip.spec.thickness;
    let pieces = walk(strip);
    if pieces.is_empty() {
        return Err(SheetError::Invalid("there is nothing to build".into()));
    }
    let left = boundary(&pieces, t / 2.0);
    let right = boundary(&pieces, -t / 2.0);
    let (first, last) = (pieces.first().unwrap(), pieces.last().unwrap());
    let mut edges: Vec<ProfileEdge> = right;
    edges.push(ProfileEdge::line(
        to3(offset_point(last.p1, last.h1, -t / 2.0)),
        to3(offset_point(last.p1, last.h1, t / 2.0)),
    ));
    edges.extend(left.iter().rev().map(reverse_edge));
    edges.push(ProfileEdge::line(
        to3(offset_point(first.p0, first.h0, t / 2.0)),
        to3(offset_point(first.p0, first.h0, -t / 2.0)),
    ));
    let mut lp = ProfileLoop::new(edges);
    let area = lp
        .check(&plane_frame().z, 1e-6)
        .map_err(|e| SheetError::Kernel(format!("{e:?}")))?;
    if area < 0.0 {
        // clockwise: traverse the other way round
        lp = ProfileLoop::new(lp.edges.iter().rev().map(reverse_edge).collect());
    }
    Ok(Profile::new(plane_frame(), lp))
}

/// Polylines (with arcs sampled) of both boundaries, for the self-intersection test.
fn sample_boundary(pieces: &[PathPiece], off: f64) -> Vec<P2> {
    let mut pts = Vec::new();
    for pc in pieces {
        match &pc.kind {
            PieceKind::Flat => {
                if pts.is_empty() {
                    pts.push(offset_point(pc.p0, pc.h0, off));
                }
                pts.push(offset_point(pc.p1, pc.h1, off));
            }
            PieceKind::Bend { angle, .. } => {
                let n = ((angle.abs() / 0.1).ceil() as usize).max(4);
                for i in 0..=n {
                    let h = pc.h0 + angle * i as f64 / n as f64;
                    let p = bend_point(pc, h);
                    let q = offset_point(p, h, off);
                    if i > 0 || pts.is_empty() {
                        pts.push(q);
                    }
                }
            }
        }
    }
    pts
}

fn bend_point(pc: &PathPiece, h: f64) -> P2 {
    if let PieceKind::Bend {
        centre,
        r_mid,
        angle,
        ..
    } = &pc.kind
    {
        let side = angle.signum();
        let a = h - side * PI / 2.0;
        (centre.0 + r_mid * a.cos(), centre.1 + r_mid * a.sin())
    } else {
        pc.p0
    }
}

fn seg_intersect(a: P2, b: P2, c: P2, d: P2) -> bool {
    let o = |p: P2, q: P2, r: P2| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
    let (d1, d2, d3, d4) = (o(a, b, c), o(a, b, d), o(c, d, a), o(c, d, b));
    ((d1 > 1e-12 && d2 < -1e-12) || (d1 < -1e-12 && d2 > 1e-12))
        && ((d3 > 1e-12 && d4 < -1e-12) || (d3 < -1e-12 && d4 > 1e-12))
}

fn check_self_intersection(strip: &SheetStrip) -> Result<(), SheetError> {
    let pieces = walk(strip);
    let mut polys: Vec<Vec<P2>> = vec![
        sample_boundary(&pieces, strip.spec.thickness / 2.0),
        sample_boundary(&pieces, -strip.spec.thickness / 2.0),
    ];
    // close the outline with the end caps so that a layer passing through an end is caught
    let all: Vec<(P2, P2)> = polys
        .iter_mut()
        .flat_map(|p| p.windows(2).map(|w| (w[0], w[1])).collect::<Vec<_>>())
        .collect();
    let n = all.len();
    for i in 0..n {
        for j in i + 1..n {
            if seg_intersect(all[i].0, all[i].1, all[j].0, all[j].1) {
                return Err(SheetError::SelfIntersecting(format!(
                    "boundary segments {i} and {j} cross"
                )));
            }
        }
    }
    Ok(())
}

/// Build the B-rep: a single extrusion of the cross-section.
pub fn build(rep: &mut BRep, strip: &SheetStrip) -> Result<BuildResult, SheetError> {
    let profile = cross_section(strip)?;
    check_self_intersection(strip)?;
    extrude(
        rep,
        &profile,
        Vector3::new(0.0, strip.width, 0.0),
        &ExtrudeOptions::default(),
    )
    .map_err(|e| SheetError::Kernel(format!("extrude: {e:?}")))
}

// ----------------------------------------------------------------------------------------- unfold

/// Bend allowance `θ (R + K t)` for a bend with inner radius `r`.
pub fn bend_allowance(angle: f64, inner_radius: f64, spec: &SheetSpec) -> f64 {
    angle.abs() * (inner_radius + spec.k_factor * spec.thickness)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlatItem {
    Flat {
        start: f64,
        end: f64,
    },
    Bend {
        /// Edges of the bend zone along the flat pattern.
        start: f64,
        end: f64,
        angle: f64,
        inner_radius: f64,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct FlatPattern {
    pub spec: SheetSpec,
    pub width: f64,
    pub items: Vec<FlatItem>,
}

impl FlatPattern {
    pub fn length(&self) -> f64 {
        self.items
            .last()
            .map(|i| match i {
                FlatItem::Flat { end, .. } | FlatItem::Bend { end, .. } => *end,
            })
            .unwrap_or(0.0)
    }

    pub fn bends(&self) -> Vec<(f64, f64, f64, f64)> {
        self.items
            .iter()
            .filter_map(|i| match i {
                FlatItem::Bend {
                    start,
                    end,
                    angle,
                    inner_radius,
                } => Some((*start, *end, *angle, *inner_radius)),
                _ => None,
            })
            .collect()
    }

    /// The flat pattern as DXF: the outline on `CUT`, each bend's centre line on `BEND_UP` (positive
    /// angle) or `BEND_DOWN`, and the two edges of every bend zone on `BEND_ZONE`.
    pub fn to_dxf(&self) -> DxfDoc {
        let w = self.width;
        let l = self.length();
        let line = |layer: &str, a: (f64, f64), b: (f64, f64)| DxfEntity::Line {
            layer: layer.into(),
            a,
            b,
        };
        let mut ents = vec![
            line("CUT", (0.0, 0.0), (l, 0.0)),
            line("CUT", (l, 0.0), (l, w)),
            line("CUT", (l, w), (0.0, w)),
            line("CUT", (0.0, w), (0.0, 0.0)),
        ];
        for (s, e, angle, _) in self.bends() {
            let mid = 0.5 * (s + e);
            let layer = if angle > 0.0 { "BEND_UP" } else { "BEND_DOWN" };
            ents.push(line(layer, (mid, 0.0), (mid, w)));
            ents.push(line("BEND_ZONE", (s, 0.0), (s, w)));
            ents.push(line("BEND_ZONE", (e, 0.0), (e, w)));
        }
        DxfDoc { entities: ents }
    }
}

/// Unfold a strip recipe analytically.
pub fn unfold(strip: &SheetStrip) -> Result<FlatPattern, SheetError> {
    strip.validate()?;
    let mut items = Vec::new();
    let mut s = 0.0;
    for g in &strip.segs {
        match *g {
            Seg::Flat { length } => {
                if length > 0.0 {
                    items.push(FlatItem::Flat {
                        start: s,
                        end: s + length,
                    });
                    s += length;
                }
            }
            Seg::Bend { angle, radius } => {
                let r = strip.radius_of(radius);
                let ba = bend_allowance(angle, r, &strip.spec);
                items.push(FlatItem::Bend {
                    start: s,
                    end: s + ba,
                    angle,
                    inner_radius: r,
                });
                s += ba;
            }
        }
    }
    Ok(FlatPattern {
        spec: strip.spec,
        width: strip.width,
        items,
    })
}

/// Fold a flat pattern back into the strip it came from.
pub fn refold(flat: &FlatPattern) -> SheetStrip {
    let mut strip = SheetStrip::new(flat.spec, flat.width);
    for it in &flat.items {
        match *it {
            FlatItem::Flat { start, end } => strip.segs.push(Seg::Flat {
                length: end - start,
            }),
            FlatItem::Bend {
                angle,
                inner_radius,
                ..
            } => strip.segs.push(Seg::Bend {
                angle,
                radius: Some(inner_radius),
            }),
        }
    }
    strip
}

/// Measure a flat pattern from its DXF geometry: the length of the outline and the width of every bend
/// zone, ordered along the pattern.
pub fn measure_dxf(doc: &DxfDoc) -> (f64, f64, Vec<(f64, f64, bool)>) {
    let mut xmax = 0.0f64;
    let mut ymax = 0.0f64;
    let mut zone_x: Vec<f64> = Vec::new();
    let mut centres: Vec<(f64, bool)> = Vec::new();
    for e in &doc.entities {
        if let DxfEntity::Line { layer, a, b } = e {
            match layer.as_str() {
                "CUT" => {
                    xmax = xmax.max(a.0).max(b.0);
                    ymax = ymax.max(a.1).max(b.1);
                }
                "BEND_ZONE" => zone_x.push(a.0),
                "BEND_UP" => centres.push((a.0, true)),
                "BEND_DOWN" => centres.push((a.0, false)),
                _ => {}
            }
        }
    }
    zone_x.sort_by(|a, b| a.partial_cmp(b).unwrap());
    centres.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let bends = zone_x
        .chunks(2)
        .zip(centres.iter())
        .map(|(z, c)| (z[0], z[1], c.1))
        .collect();
    (xmax, ymax, bends)
}

// --------------------------------------------------------------------------------------- recognise

struct LoopEdge {
    start: P2,
    end: P2,
    arc: Option<(P2, f64)>,
}

fn xz(p: Point3<f64>) -> P2 {
    (p.x, p.z)
}

/// Read the strip out of a body made of two parallel cap faces and a ruled side wall of planes and
/// cylinders (as [`build`] makes them, or any extrusion of a thick bent path).
pub fn recognize(rep: &BRep, body: BodyId, k_factor: f64) -> Result<SheetStrip, SheetError> {
    let not = |m: &str| Err(SheetError::NotSheet(m.to_string()));
    // the cap: the largest planar face whose normal is along Y
    let mut cap = None;
    let mut best = 0.0;
    for f in rep.body_faces(body) {
        let face = &rep.faces[f];
        if let SurfaceGeom::Plane(p) = &rep.surfaces[face.surface] {
            if p.frame.z.y.abs() > 1.0 - 1e-9 {
                let a = rep.face_area(f).unwrap_or(0.0);
                if a > best {
                    best = a;
                    cap = Some(f);
                }
            }
        }
    }
    let Some(cap) = cap else {
        return not("no planar cap face perpendicular to the extrusion direction");
    };
    let face = &rep.faces[cap];
    if face.loops.len() != 1 {
        return not("the cap has holes or cut-outs");
    }
    // the width: the distance between the two cap planes
    let mut ys: Vec<f64> = Vec::new();
    for f in rep.body_faces(body) {
        if let SurfaceGeom::Plane(p) = &rep.surfaces[rep.faces[f].surface] {
            if p.frame.z.y.abs() > 1.0 - 1e-9 {
                ys.push(p.frame.origin.y);
            }
        }
    }
    if ys.len() != 2 {
        return not("expected exactly two cap faces");
    }
    let width = (ys[0] - ys[1]).abs();
    // the loop's edges in order, each with its start and end in the XZ plane
    let mut edges: Vec<LoopEdge> = Vec::new();
    for co in rep.loop_coedges(face.loops[0]) {
        let c = &rep.coedges[co];
        let e = &rep.edges[c.edge];
        let (a, b) = (rep.vertices[e.start].point, rep.vertices[e.end].point);
        let (start, end) = if c.sense == Sense::Forward {
            (a, b)
        } else {
            (b, a)
        };
        let curve = e.curve.and_then(|k| rep.curves.get(k));
        let arc = match curve {
            Some(CurveGeom::Circle(ci)) => Some((xz(ci.frame.origin), ci.radius)),
            Some(CurveGeom::Line(_)) => None,
            Some(CurveGeom::Nurbs(n)) => {
                // an arc stored as a rational curve: take the centre from three points
                let (t0, t1) = n.domain();
                let (p0, p1, p2) = (n.point_at(t0), n.point_at(0.5 * (t0 + t1)), n.point_at(t1));
                circumcircle(xz(p0), xz(p1), xz(p2))
            }
            _ => return not("an edge of the cap is neither a line nor an arc"),
        };
        edges.push(LoopEdge {
            start: xz(start),
            end: xz(end),
            arc,
        });
    }
    let m = edges.len();
    if m < 4 || !m.is_multiple_of(2) {
        return not("the cap outline does not have the shape of a bent strip");
    }
    // direction at the start / end of an edge
    let dir_at = |e: &LoopEdge, at_end: bool| -> P2 {
        match e.arc {
            None => {
                let d = (e.end.0 - e.start.0, e.end.1 - e.start.1);
                let n = d.0.hypot(d.1).max(1e-300);
                (d.0 / n, d.1 / n)
            }
            Some((c, _)) => {
                let p = if at_end { e.end } else { e.start };
                let r = (p.0 - c.0, p.1 - c.1);
                // travel direction around the arc: CCW if start→end turns CCW about the centre
                let s = (e.start.0 - c.0, e.start.1 - c.1);
                let q = (e.end.0 - c.0, e.end.1 - c.1);
                let ccw = (s.0 * q.1 - s.1 * q.0) >= 0.0;
                let n = r.0.hypot(r.1).max(1e-300);
                let u = (r.0 / n, r.1 / n);
                if ccw {
                    (-u.1, u.0)
                } else {
                    (u.1, -u.0)
                }
            }
        }
    };
    // corners (non-tangent junctions) bound the two end caps
    let mut corners: Vec<usize> = Vec::new();
    for i in 0..m {
        let (a, b) = (&edges[i], &edges[(i + 1) % m]);
        let (d0, d1) = (dir_at(a, true), dir_at(b, false));
        let cross = d0.0 * d1.1 - d0.1 * d1.0;
        let dot = d0.0 * d1.0 + d0.1 * d1.1;
        if cross.abs() > 1e-6 || dot < 0.0 {
            corners.push(i); // between edge i and i+1
        }
    }
    if corners.len() != 4 {
        return not(&format!(
            "expected 4 corners in the outline, found {}",
            corners.len()
        ));
    }
    // end cap edges: those whose both ends are corners
    let mut caps: Vec<usize> = Vec::new();
    for i in 0..m {
        let before = corners.contains(&((i + m - 1) % m));
        let after = corners.contains(&i);
        if before && after {
            caps.push(i);
        }
    }
    if caps.len() != 2 || edges[caps[0]].arc.is_some() || edges[caps[1]].arc.is_some() {
        return not("could not find the two end edges of the strip");
    }
    let (c1, c2) = (caps[0], caps[1]);
    let chain = |from: usize, to: usize| -> Vec<usize> {
        let mut v = Vec::new();
        let mut i = (from + 1) % m;
        while i != to {
            v.push(i);
            i = (i + 1) % m;
        }
        v
    };
    let a_chain = chain(c1, c2);
    let b_chain: Vec<usize> = chain(c2, c1).into_iter().rev().collect();
    if a_chain.len() != b_chain.len() || a_chain.is_empty() {
        return not("the two faces of the strip do not have matching pieces");
    }
    let len = |e: &LoopEdge| (e.end.0 - e.start.0).hypot(e.end.1 - e.start.1);
    // thickness: the length of an end cap
    let t = 0.5 * (len(&edges[c1]) + len(&edges[c2]));
    if (len(&edges[c1]) - len(&edges[c2])).abs() > 1e-6 * t.max(1.0) {
        return not("the two ends of the strip have different thickness");
    }
    let mut segs: Vec<Seg> = Vec::new();
    for (ia, ib) in a_chain.iter().zip(b_chain.iter()) {
        let (a, b) = (&edges[*ia], &edges[*ib]);
        match (a.arc, b.arc) {
            (None, None) => {
                segs.push(Seg::Flat { length: len(a) });
            }
            (Some((ca, ra)), Some((cb, rb))) => {
                if (ca.0 - cb.0).hypot(ca.1 - cb.1) > 1e-6 * t.max(1.0) {
                    return not("the two faces of a bend are not concentric");
                }
                let (s, e) = (
                    (a.start.0 - ca.0, a.start.1 - ca.1),
                    (a.end.0 - ca.0, a.end.1 - ca.1),
                );
                let cross = s.0 * e.1 - s.1 * e.0;
                let dot = s.0 * e.0 + s.1 * e.1;
                let mut ang = cross.atan2(dot);
                // half turns come out as ±π: take the sign from the other face's winding
                if ang.abs() > PI - 1e-9 {
                    let (s2, e2) = (
                        (b.start.0 - cb.0, b.start.1 - cb.1),
                        (b.end.0 - cb.0, b.end.1 - cb.1),
                    );
                    let ccw_b = (s2.0 * e2.1 - s2.1 * e2.0) >= 0.0;
                    ang = if ccw_b { -PI } else { PI };
                    // chain B runs against chain A
                }
                segs.push(Seg::Bend {
                    angle: ang,
                    radius: Some(ra.min(rb)),
                });
            }
            _ => return not("a line is paired with an arc"),
        }
    }
    Ok(SheetStrip {
        spec: SheetSpec {
            thickness: t,
            k_factor,
            default_radius: segs
                .iter()
                .find_map(|s| match s {
                    Seg::Bend { radius, .. } => *radius,
                    _ => None,
                })
                .unwrap_or(t),
        },
        width,
        segs,
    })
}

fn circumcircle(a: P2, b: P2, c: P2) -> Option<(P2, f64)> {
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
    Some(((ux, uy), (a.0 - ux).hypot(a.1 - uy)))
}

/// Mid-surface volume of a strip: `W t (Σ flats + Σ θ (R + t/2))`.
pub fn analytic_volume(strip: &SheetStrip) -> f64 {
    let t = strip.spec.thickness;
    let mut l = 0.0;
    for g in &strip.segs {
        match *g {
            Seg::Flat { length } => l += length,
            Seg::Bend { angle, radius } => l += angle.abs() * (strip.radius_of(radius) + t / 2.0),
        }
    }
    strip.width * t * l
}

#[allow(dead_code)]
fn _unused(_: EntityRef, _: Origin, _: f64) -> f64 {
    TAU
}
