//! B-rep → STEP (AP214-style product structure, `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`).

use kw_brep::{
    BRep, BodyId, BodyKind, CurveId, EdgeId, FaceId, LoopKind, Sense, ShellId, SurfaceId, VertexId,
};
use kw_math::{Frame3, Point3, Vector3};
use kw_nurbs::{CurveGeom, SurfaceGeom};
use std::collections::HashMap;
use std::fmt::Write;

#[derive(Debug, Clone)]
pub struct ExportOptions {
    pub product_name: String,
    /// Written as `SI_UNIT(.MILLI.,.METRE.)`; coordinates are exported unchanged.
    pub author: String,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self {
            product_name: "kerfwright".into(),
            author: "Kerfwright".into(),
        }
    }
}

struct W<'a> {
    rep: &'a BRep,
    out: String,
    n: u32,
    curve_ids: HashMap<CurveId, u32>,
    surf_ids: HashMap<SurfaceId, u32>,
    vert_ids: HashMap<VertexId, u32>,
    edge_ids: HashMap<EdgeId, u32>,
}

fn f(x: f64) -> String {
    // STEP reals need a decimal point; keep 17 significant digits for exact round trips
    let s = format!("{x:.17e}");
    // 1.00000000000000000e0 → 1.00000000000000000E0
    s.replace('e', "E")
}

fn esc(s: &str) -> String {
    s.replace('\'', "''")
}

impl W<'_> {
    fn emit(&mut self, body: String) -> u32 {
        self.n += 1;
        let _ = writeln!(self.out, "#{} = {};", self.n, body);
        self.n
    }

    fn point(&mut self, p: &Point3<f64>) -> u32 {
        self.emit(format!(
            "CARTESIAN_POINT('',({},{},{}))",
            f(p.x),
            f(p.y),
            f(p.z)
        ))
    }
    fn dir(&mut self, d: &Vector3<f64>) -> u32 {
        let d = d.normalize();
        self.emit(format!("DIRECTION('',({},{},{}))", f(d.x), f(d.y), f(d.z)))
    }
    fn placement(&mut self, fr: &Frame3) -> u32 {
        let (o, z, x) = (self.point(&fr.origin), self.dir(&fr.z), self.dir(&fr.x));
        self.emit(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }

    fn bspline_curve(&mut self, c: &kw_nurbs::NurbsCurve) -> u32 {
        let pts: Vec<String> = c
            .points()
            .iter()
            .map(|p| format!("#{}", self.point(p)))
            .collect();
        let (mults, knots) = distinct_knots(c.knots().as_slice());
        let ml: Vec<String> = mults.iter().map(|m| m.to_string()).collect();
        let kl: Vec<String> = knots.iter().map(|k| f(*k)).collect();
        let degree = c.degree();
        if c.is_rational() {
            let w: Vec<String> = c.weights().iter().map(|x| f(*x)).collect();
            self.emit(format!(
                "(BOUNDED_CURVE() B_SPLINE_CURVE({degree},({}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM(''))",
                pts.join(","), ml.join(","), kl.join(","), w.join(",")
            ))
        } else {
            self.emit(format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{degree},({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                pts.join(","), ml.join(","), kl.join(",")
            ))
        }
    }

    fn curve(&mut self, id: CurveId) -> u32 {
        if let Some(i) = self.curve_ids.get(&id) {
            return *i;
        }
        let g = self.rep.curves[id].clone();
        let i = self.curve_geom(&g);
        self.curve_ids.insert(id, i);
        i
    }

    fn curve_geom(&mut self, g: &CurveGeom) -> u32 {
        match g {
            CurveGeom::Line(l) => {
                let p = self.point(&l.origin);
                let d = self.dir(&l.dir);
                let v = self.emit(format!("VECTOR('',#{d},{})", f(l.dir.norm())));
                self.emit(format!("LINE('',#{p},#{v})"))
            }
            CurveGeom::Circle(c) => {
                let a = self.placement(&c.frame);
                self.emit(format!("CIRCLE('',#{a},{})", f(c.radius)))
            }
            CurveGeom::Ellipse(e) => {
                let a = self.placement(&e.frame);
                self.emit(format!("ELLIPSE('',#{a},{},{})", f(e.major), f(e.minor)))
            }
            CurveGeom::Nurbs(n) => self.bspline_curve(n),
            CurveGeom::Procedural(p) => self.bspline_curve(&p.approximation),
        }
    }

    fn surface(&mut self, id: SurfaceId) -> u32 {
        if let Some(i) = self.surf_ids.get(&id) {
            return *i;
        }
        let g = self.rep.surfaces[id].clone();
        let i = match &g {
            SurfaceGeom::Plane(p) => {
                let a = self.placement(&p.frame);
                self.emit(format!("PLANE('',#{a})"))
            }
            SurfaceGeom::Cylinder(c) => {
                let a = self.placement(&c.frame);
                self.emit(format!("CYLINDRICAL_SURFACE('',#{a},{})", f(c.radius)))
            }
            SurfaceGeom::Cone(c) => {
                // STEP semi-angles are positive: a cone that narrows along +z is written with the axis reversed
                let (fr, ang) = if c.half_angle < 0.0 {
                    (
                        Frame3 {
                            origin: c.frame.origin,
                            x: c.frame.x,
                            y: -c.frame.y,
                            z: -c.frame.z,
                        },
                        -c.half_angle,
                    )
                } else {
                    (c.frame, c.half_angle)
                };
                let a = self.placement(&fr);
                self.emit(format!(
                    "CONICAL_SURFACE('',#{a},{},{})",
                    f(c.radius_at_origin),
                    f(ang)
                ))
            }
            SurfaceGeom::Sphere(s) => {
                let a = self.placement(&s.frame);
                self.emit(format!("SPHERICAL_SURFACE('',#{a},{})", f(s.radius)))
            }
            SurfaceGeom::Torus(t) => {
                let a = self.placement(&t.frame);
                self.emit(format!(
                    "TOROIDAL_SURFACE('',#{a},{},{})",
                    f(t.major_radius),
                    f(t.minor_radius)
                ))
            }
            SurfaceGeom::Extrusion { profile, dir } => {
                let c = self.curve_geom(profile);
                let d = self.dir(dir);
                let v = self.emit(format!("VECTOR('',#{d},{})", f(dir.norm())));
                self.emit(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})"))
            }
            SurfaceGeom::Revolution {
                profile,
                axis_origin,
                axis_dir,
            } => {
                let c = self.curve_geom(profile);
                let (o, d) = (self.point(axis_origin), self.dir(axis_dir));
                let ax = self.emit(format!("AXIS1_PLACEMENT('',#{o},#{d})"));
                self.emit(format!("SURFACE_OF_REVOLUTION('',#{c},#{ax})"))
            }
            SurfaceGeom::Nurbs(n) => self.bspline_surface(n),
            SurfaceGeom::Procedural(p) => self.bspline_surface(&p.approximation),
        };
        self.surf_ids.insert(id, i);
        i
    }

    fn bspline_surface(&mut self, s: &kw_nurbs::NurbsSurface) -> u32 {
        let (nu, nv) = (s.num_u(), s.num_v());
        let mut rows = Vec::new();
        for i in 0..nu {
            let row: Vec<String> = (0..nv)
                .map(|j| format!("#{}", self.point(&s.points()[i * nv + j])))
                .collect();
            rows.push(format!("({})", row.join(",")));
        }
        let (mu, ku) = distinct_knots(s.knots_u().as_slice());
        let (mv, kv) = distinct_knots(s.knots_v().as_slice());
        let j = |v: &[f64]| v.iter().map(|x| f(*x)).collect::<Vec<_>>().join(",");
        let ji = |v: &[usize]| {
            v.iter()
                .map(|x| x.to_string())
                .collect::<Vec<_>>()
                .join(",")
        };
        let rational = s.weights().iter().any(|w| (*w - 1.0).abs() > 1e-14);
        let (du, dv) = (s.knots_u().degree(), s.knots_v().degree());
        if rational {
            let wrows: Vec<String> = (0..nu)
                .map(|i| {
                    format!(
                        "({})",
                        (0..nv)
                            .map(|k| f(s.weights()[i * nv + k]))
                            .collect::<Vec<_>>()
                            .join(",")
                    )
                })
                .collect();
            self.emit(format!(
                "(BOUNDED_SURFACE() B_SPLINE_SURFACE({du},{dv},({}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(({}),({}),({}),({}),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE())",
                rows.join(","), ji(&mu), ji(&mv), j(&ku), j(&kv), wrows.join(",")
            ))
        } else {
            self.emit(format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{du},{dv},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
                rows.join(","), ji(&mu), ji(&mv), j(&ku), j(&kv)
            ))
        }
    }

    fn vertex(&mut self, v: VertexId) -> u32 {
        if let Some(i) = self.vert_ids.get(&v) {
            return *i;
        }
        let p = self.point(&self.rep.vertices[v].point.clone());
        let i = self.emit(format!("VERTEX_POINT('',#{p})"));
        self.vert_ids.insert(v, i);
        i
    }

    fn edge(&mut self, e: EdgeId) -> Option<u32> {
        if let Some(i) = self.edge_ids.get(&e) {
            return Some(*i);
        }
        let ed = self.rep.edges[e].clone();
        let c = ed.curve?; // degenerate edges are not written
        let (a, b) = (self.vertex(ed.start), self.vertex(ed.end));
        let cu = self.curve(c);
        let i = self.emit(format!("EDGE_CURVE('',#{a},#{b},#{cu},.T.)"));
        self.edge_ids.insert(e, i);
        Some(i)
    }

    fn face(&mut self, fid: FaceId, flip: bool) -> Option<u32> {
        let face = self.rep.faces[fid].clone();
        let mut bounds = Vec::new();
        for lp in &face.loops {
            let mut oes = Vec::new();
            let coedges = self.rep.loop_coedges(*lp);
            let mut seq: Vec<(EdgeId, Sense)> = coedges
                .iter()
                .map(|c| (self.rep.coedges[*c].edge, self.rep.coedges[*c].sense))
                .collect();
            if flip {
                seq.reverse();
                for s in &mut seq {
                    s.1 = s.1.flip();
                }
            }
            for (e, s) in seq {
                if let Some(ec) = self.edge(e) {
                    let o = if s == Sense::Forward { ".T." } else { ".F." };
                    oes.push(format!(
                        "#{}",
                        self.emit(format!("ORIENTED_EDGE('',*,*,#{ec},{o})"))
                    ));
                }
            }
            if oes.is_empty() {
                continue;
            }
            let l = self.emit(format!("EDGE_LOOP('',({}))", oes.join(",")));
            let kind = if self.rep.loops[*lp].kind == LoopKind::Outer {
                "FACE_OUTER_BOUND"
            } else {
                "FACE_BOUND"
            };
            bounds.push(format!("#{}", self.emit(format!("{kind}('',#{l},.T.)"))));
        }
        if bounds.is_empty() {
            return None;
        }
        let s = self.surface(face.surface);
        // STEP's surface of revolution is (angle, curve parameter); ours is (curve parameter, angle),
        // so its normal points the other way
        let swapped = matches!(
            self.rep.surfaces[face.surface],
            kw_nurbs::SurfaceGeom::Revolution { .. }
        );
        let same = ((face.sense == Sense::Forward) != flip) != swapped;
        Some(self.emit(format!(
            "ADVANCED_FACE('',({}),#{s},{})",
            bounds.join(","),
            if same { ".T." } else { ".F." }
        )))
    }

    fn shell(&mut self, sh: ShellId, flip: bool) -> u32 {
        let faces: Vec<FaceId> = self.rep.shells[sh].faces.clone();
        let ids: Vec<String> = faces
            .into_iter()
            .filter_map(|fc| self.face(fc, flip))
            .map(|i| format!("#{i}"))
            .collect();
        self.emit(format!("CLOSED_SHELL('',({}))", ids.join(",")))
    }
}

fn distinct_knots(k: &[f64]) -> (Vec<usize>, Vec<f64>) {
    let mut m = Vec::new();
    let mut v: Vec<f64> = Vec::new();
    for &x in k {
        if let Some(l) = v.last() {
            if (x - l).abs() <= 1e-14 * (1.0 + x.abs()) {
                *m.last_mut().unwrap() += 1;
                continue;
            }
        }
        v.push(x);
        m.push(1);
    }
    (m, v)
}

/// Serialise the given solid bodies as one STEP file.
pub fn export(rep: &BRep, bodies: &[BodyId], opts: &ExportOptions) -> String {
    let mut w = W {
        rep,
        out: String::new(),
        n: 0,
        curve_ids: HashMap::new(),
        surf_ids: HashMap::new(),
        vert_ids: HashMap::new(),
        edge_ids: HashMap::new(),
    };
    let mut solids: Vec<u32> = Vec::new();
    for &b in bodies {
        let body = &rep.bodies[b];
        if body.kind != BodyKind::Solid {
            continue;
        }
        for &lump in &body.lumps {
            let shells = rep.lumps[lump].shells.clone();
            let mut outer = None;
            let mut voids = Vec::new();
            for sh in shells {
                let vol = rep.shell_volume(sh).unwrap_or(1.0);
                if vol >= 0.0 && outer.is_none() {
                    outer = Some(sh);
                } else {
                    voids.push(sh);
                }
            }
            let Some(outer) = outer else { continue };
            let os = w.shell(outer, false);
            if voids.is_empty() {
                solids.push(w.emit(format!(
                    "MANIFOLD_SOLID_BREP('{}',#{os})",
                    esc(&opts.product_name)
                )));
            } else {
                // a void shell is stored with normals pointing away from the cavity (like a solid) and flipped by the oriented shell
                let mut vs = Vec::new();
                for v in voids {
                    let s = w.shell(v, true);
                    vs.push(format!(
                        "#{}",
                        w.emit(format!("ORIENTED_CLOSED_SHELL('',*,#{s},.F.)"))
                    ));
                }
                solids.push(w.emit(format!(
                    "BREP_WITH_VOIDS('{}',#{os},({}))",
                    esc(&opts.product_name),
                    vs.join(",")
                )));
            }
        }
    }
    // ---- context, units and product structure
    let name = esc(&opts.product_name);
    let o = w.point(&Point3::origin());
    let (zd, xd) = (w.dir(&Vector3::z()), w.dir(&Vector3::x()));
    let axis = w.emit(format!("AXIS2_PLACEMENT_3D('',#{o},#{zd},#{xd})"));
    let len = w.emit("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))".into());
    let ang = w.emit("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))".into());
    let sol = w.emit("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())".into());
    let unc = w.emit(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#{len},'distance_accuracy_value','confusion accuracy')"));
    let ctx = w.emit(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{len},#{ang},#{sol})) REPRESENTATION_CONTEXT('Context3D','3D Context with UNIT and UNCERTAINTY'))"
    ));
    let mut items = vec![format!("#{axis}")];
    items.extend(solids.iter().map(|s| format!("#{s}")));
    let rep_id = w.emit(format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',({}),#{ctx})",
        items.join(",")
    ));
    let app = w
        .emit("APPLICATION_CONTEXT('core data for automotive mechanical design processes')".into());
    w.emit(format!(
        "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})"
    ));
    let pctx = w.emit(format!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
    let prod = w.emit(format!("PRODUCT('{name}','{name}','',(#{pctx}))"));
    let form = w.emit(format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
    let dctx = w.emit(format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"
    ));
    let pdef = w.emit(format!("PRODUCT_DEFINITION('design','',#{form},#{dctx})"));
    let pds = w.emit(format!("PRODUCT_DEFINITION_SHAPE('','',#{pdef})"));
    w.emit(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep_id})"));
    let mut s = String::new();
    s.push_str("ISO-10303-21;\nHEADER;\n");
    let _ = writeln!(s, "FILE_DESCRIPTION(('Kerfwright STEP export'),'2;1');");
    let _ = writeln!(
        s,
        "FILE_NAME('{name}.step','1970-01-01T00:00:00',('{}'),(''),'kw-step','kw-step','');",
        esc(&opts.author)
    );
    let _ = writeln!(
        s,
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 3 1 1 1 }}'));"
    );
    s.push_str("ENDSEC;\nDATA;\n");
    s.push_str(&w.out);
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    s
}
