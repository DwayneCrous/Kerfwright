//! STEP → B-rep.

use crate::parser::{parse_with, Instance, Limits, Param, ParseError, StepFile};
use kw_brep::{
    add_face_auto, BRep, BodyId, BodyKind, EdgeId, LoopKind, LoopSpec, Sense, ShellId, SurfaceId,
    VertexId,
};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::geom::{Circle, Cone, Cylinder, Ellipse, Line, Plane, Sphere, Torus};
use kw_nurbs::proj::{curve_param_of, surface_param_of};
use kw_nurbs::{Curve3, CurveGeom, KnotVector, NurbsCurve, NurbsSurface, Surface, SurfaceGeom};
use std::collections::HashMap;
use std::f64::consts::TAU;

#[derive(Debug, Clone)]
pub struct ImportOptions {
    /// Convert lengths to millimetres using the file's declared unit.
    pub to_millimetres: bool,
    pub limits: Limits,
    /// Run `BRep::validate` on every imported solid and fail the solid if it does not pass.
    pub validate: bool,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            to_millimetres: true,
            limits: Limits::default(),
            validate: true,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ImportReport {
    pub bodies: Vec<BodyId>,
    /// Number of solid representations found in the file.
    pub solids_found: usize,
    /// `(instance id, reason)` for each solid that could not be turned into a valid body.
    pub failed: Vec<(u32, String)>,
    /// Factor applied to all lengths.
    pub unit_scale: f64,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum StepError {
    Parse(ParseError),
    /// The file contains no `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`.
    NoSolids,
    /// Every solid failed; the reasons are in the report of [`import_report`].
    AllSolidsFailed(Vec<(u32, String)>),
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for StepError {}

impl From<ParseError> for StepError {
    fn from(e: ParseError) -> Self {
        StepError::Parse(e)
    }
}

type R<T> = Result<T, String>;

struct Importer<'a> {
    f: &'a StepFile,
    scale: f64,
    // caches (per file)
    points: HashMap<u32, Point3<f64>>,
    curves: HashMap<u32, CurveGeom>,
    surfaces: HashMap<u32, SurfaceGeom>,
    warnings: Vec<String>,
}

/// Import all solids of a STEP file into `rep`.
pub fn import(rep: &mut BRep, text: &str, opts: &ImportOptions) -> Result<ImportReport, StepError> {
    let file = parse_with(text, &opts.limits)?;
    let mut imp = Importer {
        f: &file,
        scale: 1.0,
        points: HashMap::new(),
        curves: HashMap::new(),
        surfaces: HashMap::new(),
        warnings: Vec::new(),
    };
    imp.scale = if opts.to_millimetres {
        imp.detect_unit_scale()
    } else {
        1.0
    };
    let mut report = ImportReport {
        unit_scale: imp.scale,
        ..Default::default()
    };
    let solids: Vec<u32> = file
        .order
        .iter()
        .copied()
        .filter(|id| {
            let i = &file.instances[id];
            i.is("MANIFOLD_SOLID_BREP") || i.is("BREP_WITH_VOIDS")
        })
        .collect();
    if solids.is_empty() {
        return Err(StepError::NoSolids);
    }
    report.solids_found = solids.len();
    for id in solids {
        let snapshot = (rep.clone(), 0);
        let _ = snapshot;
        match imp.solid(rep, id) {
            Ok(body) => {
                if opts.validate {
                    if let Err(errs) = rep.validate().into_result() {
                        report
                            .failed
                            .push((id, format!("validation: {:?}", &errs[..errs.len().min(3)])));
                        continue;
                    }
                    match rep.volume(body) {
                        Some(v) if v > 0.0 => {}
                        other => {
                            report
                                .failed
                                .push((id, format!("non-positive volume {other:?}")));
                            continue;
                        }
                    }
                }
                report.bodies.push(body);
            }
            Err(e) => report.failed.push((id, e)),
        }
    }
    report.warnings = imp.warnings;
    if report.bodies.is_empty() {
        return Err(StepError::AllSolidsFailed(report.failed));
    }
    Ok(report)
}

impl Importer<'_> {
    fn inst(&self, id: u32) -> R<&Instance> {
        self.f
            .instances
            .get(&id)
            .ok_or_else(|| format!("dangling reference #{id}"))
    }

    fn detect_unit_scale(&self) -> f64 {
        for id in &self.f.order {
            let i = &self.f.instances[id];
            if !i.is("LENGTH_UNIT") {
                continue;
            }
            if let Some(si) = i.part("SI_UNIT") {
                let prefix = si.first().and_then(|p| p.as_enum()).unwrap_or("");
                let f = match prefix {
                    "MILLI" => 1.0,
                    "CENTI" => 10.0,
                    "DECI" => 100.0,
                    "MICRO" => 1e-3,
                    "KILO" => 1e6,
                    "" => 1000.0,
                    _ => 1.0,
                };
                return f;
            }
            if let Some(cb) = i.part("CONVERSION_BASED_UNIT") {
                let name = cb
                    .first()
                    .and_then(|p| p.as_str())
                    .unwrap_or("")
                    .to_ascii_uppercase();
                return match name.as_str() {
                    "INCH" => 25.4,
                    "FOOT" => 304.8,
                    _ => 1.0,
                };
            }
        }
        1.0
    }

    // ------------------------------------------------------------------ geometry

    fn point(&mut self, id: u32) -> R<Point3<f64>> {
        if let Some(p) = self.points.get(&id) {
            return Ok(*p);
        }
        let i = self.inst(id)?;
        if i.kind() != "CARTESIAN_POINT" {
            return Err(format!("#{id} is not a CARTESIAN_POINT"));
        }
        let c = i
            .params()
            .get(1)
            .and_then(|p| p.as_list())
            .ok_or("bad CARTESIAN_POINT")?;
        let g = |k: usize| c.get(k).and_then(|p| p.as_f64()).unwrap_or(0.0);
        let p = Point3::new(g(0), g(1), g(2)) * self.scale;
        self.points.insert(id, p);
        Ok(p)
    }

    fn direction(&self, id: u32) -> R<Vector3<f64>> {
        let i = self.inst(id)?;
        let c = i
            .params()
            .get(1)
            .and_then(|p| p.as_list())
            .ok_or("bad DIRECTION")?;
        let g = |k: usize| c.get(k).and_then(|p| p.as_f64()).unwrap_or(0.0);
        let v = Vector3::new(g(0), g(1), g(2));
        if v.norm() < 1e-12 {
            return Err("zero DIRECTION".into());
        }
        Ok(v.normalize())
    }

    fn ref_param(p: Option<&Param>) -> Option<u32> {
        p.and_then(|p| p.as_ref_id())
    }

    fn frame(&mut self, id: u32) -> R<Frame3> {
        let i = self.inst(id)?;
        let kind = i.kind().to_string();
        let params = i.params().to_vec();
        if kind != "AXIS2_PLACEMENT_3D" && kind != "AXIS1_PLACEMENT" {
            return Err(format!("#{id} is not an axis placement ({kind})"));
        }
        let origin =
            self.point(Self::ref_param(params.get(1)).ok_or("placement without location")?)?;
        let z = match Self::ref_param(params.get(2)) {
            Some(r) => self.direction(r)?,
            None => Vector3::z(),
        };
        let x = match (kind.as_str(), Self::ref_param(params.get(3))) {
            ("AXIS2_PLACEMENT_3D", Some(r)) => {
                let rd = self.direction(r)?;
                let x = rd - z * rd.dot(&z);
                if x.norm() < 1e-9 {
                    Frame3::from_origin_normal(origin, z).x
                } else {
                    x.normalize()
                }
            }
            _ => Frame3::from_origin_normal(origin, z).x,
        };
        Ok(Frame3 {
            origin,
            x,
            y: z.cross(&x),
            z,
        })
    }

    fn vector(&self, id: u32) -> R<Vector3<f64>> {
        let i = self.inst(id)?;
        let p = i.params();
        let d = self.direction(Self::ref_param(p.get(1)).ok_or("VECTOR without direction")?)?;
        let m = p
            .get(2)
            .and_then(|x| x.as_f64())
            .ok_or("VECTOR without magnitude")?;
        Ok(d * m * self.scale)
    }

    fn expand_knots(mult: &Param, knots: &Param) -> R<Vec<f64>> {
        let (m, k) = (
            mult.as_list().ok_or("bad knot multiplicities")?,
            knots.as_list().ok_or("bad knots")?,
        );
        if m.len() != k.len() {
            return Err("knot / multiplicity count mismatch".into());
        }
        let mut out = Vec::new();
        for (mm, kk) in m.iter().zip(k) {
            let (mm, kk) = (
                mm.as_usize().ok_or("bad multiplicity")?,
                kk.as_f64().ok_or("bad knot")?,
            );
            if mm == 0 || mm > 64 {
                return Err("bad multiplicity".into());
            }
            out.extend(std::iter::repeat_n(kk, mm));
        }
        Ok(out)
    }

    fn bspline_curve(&mut self, i: &Instance) -> R<CurveGeom> {
        let base = i
            .part("B_SPLINE_CURVE")
            .or_else(|| i.part("B_SPLINE_CURVE_WITH_KNOTS"))
            .ok_or("B-spline curve without data")?;
        // complex instances carry no leading name parameter
        let off = usize::from(i.parts.len() == 1);
        let degree = base
            .get(off)
            .and_then(|p| p.as_usize())
            .ok_or("bad degree")?;
        let pts_ref = base
            .get(off + 1)
            .and_then(|p| p.as_list())
            .ok_or("bad control points")?;
        let mut pts = Vec::with_capacity(pts_ref.len());
        for r in pts_ref {
            pts.push(self.point(r.as_ref_id().ok_or("bad control point")?)?);
        }
        let k = i
            .part("B_SPLINE_CURVE_WITH_KNOTS")
            .ok_or("only B_SPLINE_CURVE_WITH_KNOTS is supported")?;
        // simple instance: params are [name, degree, pts, form, closed, selfint, mult, knots, spec]
        let (mult, knots) = if i.parts.len() == 1 {
            (k.get(6), k.get(7))
        } else {
            (k.first(), k.get(1))
        };
        let kv = Self::expand_knots(
            mult.ok_or("missing multiplicities")?,
            knots.ok_or("missing knots")?,
        )?;
        let weights: Vec<f64> = match i.part("RATIONAL_B_SPLINE_CURVE") {
            Some(w) => w
                .first()
                .and_then(|p| p.as_list())
                .ok_or("bad weights")?
                .iter()
                .map(|x| x.as_f64().unwrap_or(1.0))
                .collect(),
            None => vec![1.0; pts.len()],
        };
        let kv = KnotVector::new(degree, kv).map_err(|e| format!("bad knot vector: {e:?}"))?;
        NurbsCurve::new(kv, pts, weights)
            .map(CurveGeom::Nurbs)
            .map_err(|e| format!("bad B-spline curve: {e:?}"))
    }

    fn curve(&mut self, id: u32, depth: usize) -> R<CurveGeom> {
        if depth > 16 {
            return Err("curve nesting too deep".into());
        }
        if let Some(c) = self.curves.get(&id) {
            return Ok(c.clone());
        }
        let i = self.inst(id)?.clone();
        let p = i.params().to_vec();
        let c = if i.is("B_SPLINE_CURVE_WITH_KNOTS") || i.is("B_SPLINE_CURVE") {
            self.bspline_curve(&i)?
        } else {
            match i.kind() {
                "LINE" => {
                    let origin =
                        self.point(Self::ref_param(p.get(1)).ok_or("LINE without point")?)?;
                    let dir =
                        self.vector(Self::ref_param(p.get(2)).ok_or("LINE without vector")?)?;
                    CurveGeom::Line(Line { origin, dir })
                }
                "CIRCLE" => {
                    let frame =
                        self.frame(Self::ref_param(p.get(1)).ok_or("CIRCLE without placement")?)?;
                    CurveGeom::Circle(Circle {
                        frame,
                        radius: p.get(2).and_then(|x| x.as_f64()).ok_or("CIRCLE radius")?
                            * self.scale,
                    })
                }
                "ELLIPSE" => {
                    let frame =
                        self.frame(Self::ref_param(p.get(1)).ok_or("ELLIPSE without placement")?)?;
                    let (a, b) = (
                        p.get(2).and_then(|x| x.as_f64()),
                        p.get(3).and_then(|x| x.as_f64()),
                    );
                    let (a, b) = (
                        a.ok_or("ELLIPSE axis")? * self.scale,
                        b.ok_or("ELLIPSE axis")? * self.scale,
                    );
                    // our ellipse has major ≥ minor along x / y; swap axes if the file's first semi-axis is the shorter
                    if a >= b {
                        CurveGeom::Ellipse(Ellipse {
                            frame,
                            major: a,
                            minor: b,
                        })
                    } else {
                        let f = Frame3 {
                            origin: frame.origin,
                            x: frame.y,
                            y: -frame.x,
                            z: frame.z,
                        };
                        CurveGeom::Ellipse(Ellipse {
                            frame: f,
                            major: b,
                            minor: a,
                        })
                    }
                }
                "HYPERBOLA" | "PARABOLA" => {
                    // no native conics beyond the ellipse: approximate the branch with a spline whose
                    // parameter is the conic's own parameter over [-6, 6]
                    let frame =
                        self.frame(Self::ref_param(p.get(1)).ok_or("conic without placement")?)?;
                    let a = p.get(2).and_then(|x| x.as_f64()).ok_or("conic axis")? * self.scale;
                    let b = p.get(3).and_then(|x| x.as_f64()).map(|x| x * self.scale);
                    let hyper = i.kind() == "HYPERBOLA";
                    let b = if hyper {
                        b.ok_or("hyperbola semi-imaginary axis")?
                    } else {
                        0.0
                    };
                    let eval = move |t: f64| -> (Point3<f64>, Vector3<f64>) {
                        if hyper {
                            (
                                frame.origin + frame.x * (a * t.cosh()) + frame.y * (b * t.sinh()),
                                frame.x * (a * t.sinh()) + frame.y * (b * t.cosh()),
                            )
                        } else {
                            (
                                frame.origin + frame.x * (a * t * t) + frame.y * (2.0 * a * t),
                                frame.x * (2.0 * a * t) + frame.y * (2.0 * a),
                            )
                        }
                    };
                    let tol = 1e-9 * (1.0 + a.abs() + b.abs());
                    let fit = kw_nurbs::fit::hermite_fit(
                        &mut |t| Some(eval(t)),
                        -6.0,
                        6.0,
                        tol,
                        16,
                        20000,
                    )
                    .ok_or("could not approximate conic")?;
                    CurveGeom::Nurbs(fit)
                }
                "TRIMMED_CURVE" => self.curve(
                    Self::ref_param(p.get(1)).ok_or("TRIMMED_CURVE basis")?,
                    depth + 1,
                )?,
                "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "BOUNDED_SURFACE_CURVE" => {
                    self.curve(
                        Self::ref_param(p.get(1)).ok_or("surface curve without 3D curve")?,
                        depth + 1,
                    )?
                }
                other => return Err(format!("unsupported curve type {other}")),
            }
        };
        self.curves.insert(id, c.clone());
        Ok(c)
    }

    fn bspline_surface(&mut self, i: &Instance) -> R<SurfaceGeom> {
        let base = i
            .part("B_SPLINE_SURFACE")
            .or_else(|| i.part("B_SPLINE_SURFACE_WITH_KNOTS"))
            .ok_or("B-spline surface without data")?;
        let off = usize::from(i.parts.len() == 1);
        let (du, dv) = (
            base.get(off)
                .and_then(|p| p.as_usize())
                .ok_or("bad u degree")?,
            base.get(off + 1)
                .and_then(|p| p.as_usize())
                .ok_or("bad v degree")?,
        );
        let rows = base
            .get(off + 2)
            .and_then(|p| p.as_list())
            .ok_or("bad control net")?;
        let nu = rows.len();
        let mut pts: Vec<Point3<f64>> = Vec::new();
        let mut nv = 0;
        for r in rows {
            let row = r.as_list().ok_or("bad control net row")?;
            if nv == 0 {
                nv = row.len();
            } else if nv != row.len() {
                return Err("ragged control net".into());
            }
            for c in row {
                pts.push(self.point(c.as_ref_id().ok_or("bad control point")?)?);
            }
        }
        let k = i
            .part("B_SPLINE_SURFACE_WITH_KNOTS")
            .ok_or("only B_SPLINE_SURFACE_WITH_KNOTS is supported")?;
        let (mu, mv, ku, kvv) = if i.parts.len() == 1 {
            (k.get(8), k.get(9), k.get(10), k.get(11))
        } else {
            (k.first(), k.get(1), k.get(2), k.get(3))
        };
        let ku = Self::expand_knots(mu.ok_or("u multiplicities")?, ku.ok_or("u knots")?)?;
        let kv2 = Self::expand_knots(mv.ok_or("v multiplicities")?, kvv.ok_or("v knots")?)?;
        let weights: Vec<f64> = match i.part("RATIONAL_B_SPLINE_SURFACE") {
            Some(w) => {
                let rows = w.first().and_then(|p| p.as_list()).ok_or("bad weights")?;
                rows.iter()
                    .flat_map(|r| {
                        r.as_list()
                            .unwrap_or(&[])
                            .iter()
                            .map(|x| x.as_f64().unwrap_or(1.0))
                    })
                    .collect()
            }
            None => vec![1.0; pts.len()],
        };
        let (ku, kv2) = (
            KnotVector::new(du, ku).map_err(|e| format!("bad u knots: {e:?}"))?,
            KnotVector::new(dv, kv2).map_err(|e| format!("bad v knots: {e:?}"))?,
        );
        if ku.num_ctrl() != nu || kv2.num_ctrl() != nv {
            return Err("control net does not match the knot vectors".into());
        }
        NurbsSurface::new(ku, kv2, pts, weights)
            .map(SurfaceGeom::Nurbs)
            .map_err(|e| format!("bad B-spline surface: {e:?}"))
    }

    fn surface(&mut self, id: u32) -> R<SurfaceGeom> {
        if let Some(s) = self.surfaces.get(&id) {
            return Ok(s.clone());
        }
        let i = self.inst(id)?.clone();
        let p = i.params().to_vec();
        let fr = |me: &mut Self, k: usize| -> R<Frame3> {
            me.frame(Self::ref_param(p.get(k)).ok_or("surface without placement")?)
        };
        let num = |k: usize, what: &'static str| -> R<f64> {
            p.get(k)
                .and_then(|x| x.as_f64())
                .ok_or_else(|| what.to_string())
        };
        let s = if i.is("B_SPLINE_SURFACE_WITH_KNOTS") || i.is("B_SPLINE_SURFACE") {
            self.bspline_surface(&i)?
        } else {
            match i.kind() {
                "PLANE" => SurfaceGeom::Plane(Plane {
                    frame: fr(self, 1)?,
                }),
                "CYLINDRICAL_SURFACE" => SurfaceGeom::Cylinder(Cylinder {
                    frame: fr(self, 1)?,
                    radius: num(2, "cylinder radius")? * self.scale,
                }),
                "CONICAL_SURFACE" => SurfaceGeom::Cone(Cone {
                    frame: fr(self, 1)?,
                    radius_at_origin: num(2, "cone radius")? * self.scale,
                    half_angle: num(3, "cone angle")?,
                }),
                "SPHERICAL_SURFACE" => SurfaceGeom::Sphere(Sphere {
                    frame: fr(self, 1)?,
                    radius: num(2, "sphere radius")? * self.scale,
                }),
                "TOROIDAL_SURFACE" => SurfaceGeom::Torus(Torus {
                    frame: fr(self, 1)?,
                    major_radius: num(2, "major radius")? * self.scale,
                    minor_radius: num(3, "minor radius")? * self.scale,
                }),
                "SURFACE_OF_LINEAR_EXTRUSION" => {
                    let c = self.curve(Self::ref_param(p.get(1)).ok_or("extrusion curve")?, 0)?;
                    let d = self.vector(Self::ref_param(p.get(2)).ok_or("extrusion vector")?)?;
                    SurfaceGeom::Extrusion {
                        profile: Box::new(c),
                        dir: d,
                    }
                }
                "SURFACE_OF_REVOLUTION" => {
                    let c = self.curve(Self::ref_param(p.get(1)).ok_or("revolution curve")?, 0)?;
                    let ax = self.frame(Self::ref_param(p.get(2)).ok_or("revolution axis")?)?;
                    SurfaceGeom::Revolution {
                        profile: Box::new(c),
                        axis_origin: ax.origin,
                        axis_dir: ax.z,
                    }
                }
                other => return Err(format!("unsupported surface type {other}")),
            }
        };
        self.surfaces.insert(id, s.clone());
        Ok(s)
    }

    // ------------------------------------------------------------------ topology

    fn solid(&mut self, rep: &mut BRep, id: u32) -> R<BodyId> {
        let inst = self.inst(id)?.clone();
        let p = inst.params().to_vec();
        let outer = Self::ref_param(p.get(1)).ok_or("solid without shell")?;
        let mut shells: Vec<(u32, bool)> = vec![(outer, false)];
        if inst.is("BREP_WITH_VOIDS") {
            for v in p.get(2).and_then(|x| x.as_list()).ok_or("bad voids")? {
                let vid = v.as_ref_id().ok_or("bad void reference")?;
                let vi = self.inst(vid)?;
                if vi.kind() == "ORIENTED_CLOSED_SHELL" {
                    let inner = vi
                        .params()
                        .get(2)
                        .and_then(|x| x.as_ref_id())
                        .ok_or("bad oriented shell")?;
                    let flip = vi.params().get(3).and_then(|x| x.as_bool()) == Some(false);
                    shells.push((inner, flip));
                } else {
                    shells.push((vid, false));
                }
            }
        }
        let body = rep.add_body(BodyKind::Solid);
        let lump = rep.add_lump(body);
        let mut vmap: HashMap<u32, VertexId> = HashMap::new();
        let mut emap: HashMap<u32, (EdgeId, bool)> = HashMap::new();
        let mut smap: HashMap<u32, SurfaceId> = HashMap::new();
        for (sid, flip) in shells {
            let si = self.inst(sid)?.clone();
            if !si.is("CLOSED_SHELL") {
                return Err(format!("#{sid} is not a CLOSED_SHELL"));
            }
            let shell = rep.add_shell(lump);
            let faces = si
                .params()
                .get(1)
                .and_then(|x| x.as_list())
                .ok_or("bad shell faces")?
                .to_vec();
            for f in faces {
                let fid = f.as_ref_id().ok_or("bad face reference")?;
                self.face(rep, shell, fid, flip, &mut vmap, &mut emap, &mut smap)?;
            }
        }
        rep.prune_orphans();
        Ok(body)
    }

    fn vertex(
        &mut self,
        rep: &mut BRep,
        id: u32,
        vmap: &mut HashMap<u32, VertexId>,
    ) -> R<(VertexId, Point3<f64>)> {
        let i = self.inst(id)?.clone();
        if i.kind() != "VERTEX_POINT" {
            return Err(format!("#{id} is not a VERTEX_POINT"));
        }
        let pt = self.point(Self::ref_param(i.params().get(1)).ok_or("vertex without point")?)?;
        let v = *vmap.entry(id).or_insert_with(|| rep.add_vertex(pt));
        Ok((v, pt))
    }

    /// kw edge for a STEP EDGE_CURVE; returns the edge and whether its start vertex is the STEP start vertex.
    fn edge(
        &mut self,
        rep: &mut BRep,
        id: u32,
        vmap: &mut HashMap<u32, VertexId>,
        emap: &mut HashMap<u32, (EdgeId, bool)>,
    ) -> R<(EdgeId, bool)> {
        if let Some(e) = emap.get(&id) {
            return Ok(*e);
        }
        let i = self.inst(id)?.clone();
        if i.kind() != "EDGE_CURVE" {
            return Err(format!("#{id} is not an EDGE_CURVE"));
        }
        let p = i.params();
        let v1 = Self::ref_param(p.get(1)).ok_or("edge start")?;
        let v2 = Self::ref_param(p.get(2)).ok_or("edge end")?;
        let curve_id = Self::ref_param(p.get(3)).ok_or("edge curve")?;
        let same = p.get(4).and_then(|x| x.as_bool()).unwrap_or(true);
        let (va, pa) = self.vertex(rep, v1, vmap)?;
        let (vb, pb) = self.vertex(rep, v2, vmap)?;
        let curve = self.curve(curve_id, 0)?;
        let closed = va == vb || (pa - pb).norm() < 1e-9 * (1.0 + pa.coords.amax());
        let ta = curve_param_of(&curve, &pa, None);
        let tb = curve_param_of(&curve, &pb, None);
        let tol = 1e-4 * (1.0 + pa.coords.amax());
        if ta.distance > tol || tb.distance > tol {
            return Err(format!(
                "edge #{id}: vertices are {:.2e} / {:.2e} from the curve",
                ta.distance, tb.distance
            ));
        }
        let periodic = matches!(curve, CurveGeom::Circle(_) | CurveGeom::Ellipse(_));
        let (t_a, t_b) = (ta.t, tb.t);
        // parameter interval travelled from v1 to v2 along the *curve direction* (increasing parameter)
        let (lo, hi, start_is_v1) = if same {
            (t_a, t_b, true)
        } else {
            // v1 → v2 runs against the curve direction: along the curve it goes v2 → v1
            (t_b, t_a, false)
        };
        let (lo, hi) = if periodic {
            let mut hi = hi;
            if closed {
                hi = lo + TAU;
            } else {
                while hi <= lo + 1e-12 {
                    hi += TAU;
                }
            }
            (lo, hi)
        } else if closed {
            match &curve {
                CurveGeom::Nurbs(n) => n.domain(),
                _ => return Err("closed edge on an open curve".into()),
            }
        } else {
            (lo, hi)
        };
        if hi.is_nan() || lo.is_nan() || hi <= lo {
            return Err(format!("edge #{id} has an empty parameter range"));
        }
        let (vs, ve) = if start_is_v1 { (va, vb) } else { (vb, va) };
        let cid = rep.add_curve(curve);
        let e = rep.add_edge(vs, ve, Some(cid), (lo, hi));
        emap.insert(id, (e, start_is_v1));
        Ok((e, start_is_v1))
    }

    #[allow(clippy::too_many_arguments)]
    fn face(
        &mut self,
        rep: &mut BRep,
        shell: ShellId,
        id: u32,
        flip_shell: bool,
        vmap: &mut HashMap<u32, VertexId>,
        emap: &mut HashMap<u32, (EdgeId, bool)>,
        smap: &mut HashMap<u32, SurfaceId>,
    ) -> R<()> {
        let fi = self.inst(id)?.clone();
        if !fi.is("ADVANCED_FACE") && !fi.is("FACE_SURFACE") {
            return Err(format!("#{id}: unsupported face type {}", fi.kind()));
        }
        let fp = fi.params();
        let bounds = fp
            .get(1)
            .and_then(|x| x.as_list())
            .ok_or("face without bounds")?
            .to_vec();
        let surf_id = Self::ref_param(fp.get(2)).ok_or("face without surface")?;
        let mut same = fp.get(3).and_then(|x| x.as_bool()).unwrap_or(true);
        if flip_shell {
            same = !same;
        }
        let geom = self.surface(surf_id)?;
        let sid = match smap.get(&surf_id) {
            Some(s) => *s,
            None => {
                let s = rep.add_surface(geom.clone());
                smap.insert(surf_id, s);
                s
            }
        };
        // loops
        let mut specs: Vec<LoopSpec> = Vec::new();
        let mut saw_vertex_loop = false;
        let mut sizes: Vec<f64> = Vec::new();
        let has_outer = bounds.iter().any(|b| {
            b.as_ref_id().is_some_and(|r| {
                self.f
                    .instances
                    .get(&r)
                    .is_some_and(|i| i.is("FACE_OUTER_BOUND"))
            })
        });
        for b in bounds.iter() {
            let bid = b.as_ref_id().ok_or("bad bound reference")?;
            let bi_inst = self.inst(bid)?.clone();
            let bp = bi_inst.params();
            let loop_id = Self::ref_param(bp.get(1)).ok_or("bound without loop")?;
            let orient = bp.get(2).and_then(|x| x.as_bool()).unwrap_or(true);
            let li = self.inst(loop_id)?.clone();
            if li.kind() == "VERTEX_LOOP" {
                saw_vertex_loop = true;
                self.warnings
                    .push(format!("face #{id}: vertex loop ignored"));
                continue;
            }
            if li.kind() != "EDGE_LOOP" {
                return Err(format!("unsupported loop type {}", li.kind()));
            }
            let oes = li
                .params()
                .get(1)
                .and_then(|x| x.as_list())
                .ok_or("bad edge loop")?
                .to_vec();
            let mut chain: Vec<(EdgeId, Sense)> = Vec::new();
            for oe in oes {
                let oid = oe.as_ref_id().ok_or("bad oriented edge")?;
                let oi = self.inst(oid)?.clone();
                if oi.kind() != "ORIENTED_EDGE" {
                    return Err(format!("#{oid} is not an ORIENTED_EDGE"));
                }
                let op = oi.params();
                let ec = Self::ref_param(op.get(3)).ok_or("oriented edge without edge")?;
                let o = op.get(4).and_then(|x| x.as_bool()).unwrap_or(true);
                let (e, start_is_v1) = self.edge(rep, ec, vmap, emap)?;
                // traversing v1 → v2 is Forward on the kw edge iff its start is v1
                let fwd = o == start_is_v1;
                chain.push((e, if fwd { Sense::Forward } else { Sense::Reversed }));
            }
            if chain.is_empty() {
                return Err("empty loop".into());
            }
            if !orient {
                chain.reverse();
                for c in &mut chain {
                    c.1 = c.1.flip();
                }
            }
            let is_outer = bi_inst.is("FACE_OUTER_BOUND");
            // extent of the loop's 3D samples: without a FACE_OUTER_BOUND the widest loop is the outer one
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for (e, _) in &chain {
                let ed = &rep.edges[*e];
                let mut pts = vec![rep.vertices[ed.start].point, rep.vertices[ed.end].point];
                if let Some(c) = ed.curve.and_then(|c| rep.curves.get(c)) {
                    for k in 1..8 {
                        let t = ed.param_range.0
                            + (ed.param_range.1 - ed.param_range.0) * k as f64 / 8.0;
                        pts.push(c.point_at(t));
                    }
                }
                for p in pts {
                    for a in 0..3 {
                        lo[a] = lo[a].min(p[a]);
                        hi[a] = hi[a].max(p[a]);
                    }
                }
            }
            let size = (0..3).map(|a| (hi[a] - lo[a]).max(0.0)).sum::<f64>();
            sizes.push(size);
            self.insert_degenerates(rep, &geom, &mut chain);
            specs.push(LoopSpec {
                kind: if is_outer {
                    LoopKind::Outer
                } else {
                    LoopKind::Inner
                },
                chain,
            });
        }
        if !has_outer && !specs.is_empty() {
            let mut best = 0;
            for (k, sz) in sizes.iter().enumerate() {
                if *sz > sizes[best] * (1.0 + 1e-9) {
                    best = k;
                }
            }
            // `specs` skips vertex loops, `sizes` is pushed in lock-step
            specs[best].kind = LoopKind::Outer;
        }
        if specs.is_empty() {
            // A whole sphere is written as one face bounded by a vertex loop (no edges at all):
            // give it the seam + pole topology of `make_sphere`.
            match (&geom, saw_vertex_loop) {
                (SurfaceGeom::Sphere(sp), true) => {
                    let f = sp.frame;
                    let south = rep.add_vertex(f.origin - f.z * sp.radius);
                    let north = rep.add_vertex(f.origin + f.z * sp.radius);
                    let meridian = rep.add_curve(CurveGeom::Circle(Circle {
                        frame: Frame3 {
                            origin: f.origin,
                            x: f.x,
                            y: f.z,
                            z: -f.y,
                        },
                        radius: sp.radius,
                    }));
                    let half = std::f64::consts::FRAC_PI_2;
                    let seam = rep.add_edge(south, north, Some(meridian), (-half, half));
                    let ds = rep.add_edge(south, south, None, (0.0, 1.0));
                    let dn = rep.add_edge(north, north, None, (0.0, 1.0));
                    specs.push(LoopSpec {
                        kind: LoopKind::Outer,
                        chain: vec![
                            (ds, Sense::Forward),
                            (seam, Sense::Forward),
                            (dn, Sense::Reversed),
                            (seam, Sense::Reversed),
                        ],
                    });
                }
                _ => return Err("face without usable bounds".into()),
            }
        }
        // STEP's surface of revolution is (angle, curve parameter); ours is (curve parameter, angle),
        // so its normal points the other way
        let swapped = matches!(geom, SurfaceGeom::Revolution { .. });
        let sign = if same != swapped { 1.0 } else { -1.0 };
        let g2 = geom.clone();
        add_face_auto(
            rep,
            shell,
            sid,
            &move |p| {
                let r = surface_param_of(&g2, p, None);
                g2.normal_at(r.u, r.v) * sign
            },
            &specs,
        )
        .map_err(|e| format!("face #{id}: {e}"))?;
        Ok(())
    }

    /// STEP does not store poles / apexes as edges: insert a degenerate edge where two consecutive
    /// edges of a loop meet at a singular point of the surface.
    fn insert_degenerates(
        &self,
        rep: &mut BRep,
        surf: &SurfaceGeom,
        chain: &mut Vec<(EdgeId, Sense)>,
    ) {
        let singular = |p: &Point3<f64>| -> bool {
            match surf {
                SurfaceGeom::Sphere(s) => {
                    let q = p - s.frame.origin;
                    (q - s.frame.z * q.dot(&s.frame.z)).norm() < 1e-7 * s.radius
                }
                SurfaceGeom::Cone(c) => {
                    let tn = c.half_angle.tan();
                    if tn.abs() < 1e-12 {
                        return false;
                    }
                    let apex = c.frame.origin - c.frame.z * (c.radius_at_origin / tn);
                    (p - apex).norm() < 1e-7 * (1.0 + c.radius_at_origin)
                }
                SurfaceGeom::Revolution {
                    axis_origin,
                    axis_dir,
                    ..
                } => {
                    let a = axis_dir.normalize();
                    let q = p - axis_origin;
                    (q - a * q.dot(&a)).norm() < 1e-7 * (1.0 + q.norm())
                }
                _ => false,
            }
        };
        let n = chain.len();
        let mut out: Vec<(EdgeId, Sense)> = Vec::with_capacity(n + 2);
        for k in 0..n {
            out.push(chain[k]);
            let (e, s) = chain[k];
            let end_v = match s {
                Sense::Forward => rep.edges[e].end,
                Sense::Reversed => rep.edges[e].start,
            };
            let pt = rep.vertices[end_v].point;
            let next = chain[(k + 1) % n];
            let next_start = match next.1 {
                Sense::Forward => rep.edges[next.0].start,
                Sense::Reversed => rep.edges[next.0].end,
            };
            // only between two *different* edges that end/start at the same singular vertex
            let both_degenerate = rep.edges[e].curve.is_none() || rep.edges[next.0].curve.is_none();
            if n > 1 && end_v == next_start && !both_degenerate && singular(&pt) {
                let d = rep.add_edge(end_v, end_v, None, (0.0, 1.0));
                out.push((d, Sense::Forward));
            }
        }
        *chain = out;
    }
}
