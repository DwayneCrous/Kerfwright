//! Scripting: build and edit a Kerfwright document from a [rhai](https://rhai.rs) script.
//!
//! The script drives the same builder API Rust code uses, so a scripted document and one built in Rust are
//! the same document, byte for byte when saved. Scripts are sandboxed: they have no file, network or
//! process access (only `save`, which writes a `.kwf`), and run under operation and size limits.
//!
//! ```text
//! let d = new_doc();
//! let w = d.param("width", "40 mm");
//! let s = d.rect_sketch("Base sketch", "xy", 0, 0, "width", "width / 2");
//! let base = d.extrude("Base", s, "10 mm", "new");
//! let hs = d.circle_sketch("Hole sketch", "xy:10", 20, 10, 6);
//! d.extrude("Hole", hs, 12, "cut_down");
//! d
//! ```
//!
//! Numbers are millimetres (or degrees for angles, plain for counts); strings are expressions with units.

use kw_expr::Param;
use kw_feature::kwf::KwfFile;
use kw_feature::*;
use rhai::{Array, Dynamic, EvalAltResult, ImmutableString, Map, Position, Scope};
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

type RResult<T> = Result<T, Box<EvalAltResult>>;

fn err<T>(msg: impl Into<String>) -> RResult<T> {
    Err(EvalAltResult::ErrorRuntime(Dynamic::from(msg.into()), Position::NONE).into())
}

struct Inner {
    doc: Document,
    engine: kw_feature::Engine,
    last: Option<Evaluation>,
}

/// The document a script is building.
#[derive(Clone)]
pub struct ScriptDoc(Rc<RefCell<Inner>>);

impl ScriptDoc {
    pub fn new() -> ScriptDoc {
        ScriptDoc(Rc::new(RefCell::new(Inner {
            doc: Document::new(),
            engine: kw_feature::Engine::with_threads(2),
            last: None,
        })))
    }
    pub fn document(&self) -> Document {
        self.0.borrow().doc.clone()
    }
}

impl Default for ScriptDoc {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy)]
enum Kind {
    Length,
    Scalar,
}

fn param(v: &Dynamic, kind: Kind) -> RResult<Param> {
    if let Some(s) = v.clone().try_cast::<ImmutableString>() {
        return Param::expr(&s).or_else(|e| err(format!("'{s}': {e}")));
    }
    let n = if let Some(f) = v.clone().try_cast::<f64>() {
        f
    } else if let Some(i) = v.clone().try_cast::<i64>() {
        i as f64
    } else {
        return err(format!(
            "expected a number or an expression, got {}",
            v.type_name()
        ));
    };
    Ok(match kind {
        Kind::Length => Param::length(n),
        Kind::Scalar => Param::scalar(n),
    })
}

fn num(v: &Dynamic) -> RResult<f64> {
    if let Some(f) = v.clone().try_cast::<f64>() {
        Ok(f)
    } else if let Some(i) = v.clone().try_cast::<i64>() {
        Ok(i as f64)
    } else {
        err(format!("expected a number, got {}", v.type_name()))
    }
}

fn plane(spec: &str) -> RResult<PlaneSpec> {
    let (name, off) = match spec.split_once(':') {
        Some((n, o)) => (n, Some(o)),
        None => (spec, None),
    };
    let offset =
        match off {
            Some(o) => param(&Dynamic::from(ImmutableString::from(o)), Kind::Length).or_else(
                |_| match o.parse::<f64>() {
                    Ok(v) => Ok(Param::length(v)),
                    Err(_) => err(format!("'{o}' is not a plane offset")),
                },
            )?,
            None => Param::length(0.0),
        };
    // a bare number after the colon is a length in mm
    let offset = match (&offset, off.and_then(|o| o.trim().parse::<f64>().ok())) {
        (_, Some(v)) => Param::length(v),
        _ => offset,
    };
    match name {
        "xy" => Ok(PlaneSpec::Xy(offset)),
        "yz" => Ok(PlaneSpec::Yz(offset)),
        "xz" => Ok(PlaneSpec::Xz(offset)),
        other => err(format!(
            "unknown plane '{other}' (use xy, yz, xz, optionally 'xy:offset')"
        )),
    }
}

fn op(s: &str) -> RResult<(BodyOp, bool)> {
    // second value: flip (cut or join against the sketch normal)
    match s {
        "new" => Ok((BodyOp::New, false)),
        "join" => Ok((BodyOp::Join, false)),
        "cut" => Ok((BodyOp::Cut, false)),
        "cut_down" => Ok((BodyOp::Cut, true)),
        "join_down" => Ok((BodyOp::Join, true)),
        "intersect" => Ok((BodyOp::Intersect, false)),
        other => err(format!(
            "unknown operation '{other}' (new, join, cut, cut_down, join_down, intersect)"
        )),
    }
}

fn id(v: i64) -> FeatureId {
    FeatureId(v as u64)
}

fn tl<T>(r: Result<T, TimelineError>) -> RResult<T> {
    r.or_else(|e| err(e.to_string()))
}

impl ScriptDoc {
    fn param_f(&mut self, name: ImmutableString, value: Dynamic) -> RResult<i64> {
        let p = param(&value, Kind::Length)?;
        let r = self.0.borrow_mut().doc.add_parameter(&name, p);
        tl(r).map(|i| i.0 as i64)
    }

    fn rect_sketch_f(
        &mut self,
        name: ImmutableString,
        pl: ImmutableString,
        x0: Dynamic,
        y0: Dynamic,
        w: Dynamic,
        h: Dynamic,
    ) -> RResult<i64> {
        let (pl, x0, y0, w, h) = (
            plane(&pl)?,
            param(&x0, Kind::Length)?,
            param(&y0, Kind::Length)?,
            param(&w, Kind::Length)?,
            param(&h, Kind::Length)?,
        );
        let r = self
            .0
            .borrow_mut()
            .doc
            .add_rect_sketch(&name, pl, x0, y0, w, h);
        tl(r).map(|i| i.0 as i64)
    }

    fn circle_sketch_f(
        &mut self,
        name: ImmutableString,
        pl: ImmutableString,
        cx: Dynamic,
        cy: Dynamic,
        d: Dynamic,
    ) -> RResult<i64> {
        let (pl, cx, cy, d) = (
            plane(&pl)?,
            param(&cx, Kind::Length)?,
            param(&cy, Kind::Length)?,
            param(&d, Kind::Length)?,
        );
        let r = self
            .0
            .borrow_mut()
            .doc
            .add_circle_sketch(&name, pl, cx, cy, d);
        tl(r).map(|i| i.0 as i64)
    }

    fn extrude_f(
        &mut self,
        name: ImmutableString,
        sketch: i64,
        depth: Dynamic,
        how: ImmutableString,
    ) -> RResult<i64> {
        let depth = param(&depth, Kind::Length)?;
        let (op, flip) = op(&how)?;
        let r = self.0.borrow_mut().doc.add(
            &name,
            FeatureSpec::Extrude(ExtrudeSpec {
                surface: false,
                sketch: id(sketch),
                depth,
                symmetric: false,
                flip,
                op,
                base: None,
                extent: Default::default(),
            }),
        );
        tl(r).map(|i| i.0 as i64)
    }

    fn linear_pattern_f(
        &mut self,
        name: ImmutableString,
        source: i64,
        dx: Dynamic,
        dy: Dynamic,
        dz: Dynamic,
        count: Dynamic,
    ) -> RResult<i64> {
        let spacing = [
            param(&dx, Kind::Length)?,
            param(&dy, Kind::Length)?,
            param(&dz, Kind::Length)?,
        ];
        let count = param(&count, Kind::Scalar)?;
        let r = self
            .0
            .borrow_mut()
            .doc
            .add_linear_pattern(&name, id(source), spacing, count);
        tl(r).map(|i| i.0 as i64)
    }

    fn fillet_f(&mut self, name: ImmutableString, edges: Array, radius: Dynamic) -> RResult<i64> {
        let radius = param(&radius, Kind::Length)?;
        let mut es = Vec::new();
        for e in edges {
            let pair: Array = e
                .try_cast()
                .ok_or_else(|| "each edge is a pair of face references: [[a, b], ...]".to_string())
                .or_else(err)?;
            if pair.len() != 2 {
                return err("each edge is a pair of face references");
            }
            let get = |d: &Dynamic| -> RResult<Reference> {
                d.clone()
                    .try_cast::<RefHandle>()
                    .map(|h| h.0)
                    .ok_or_else(|| "expected a face reference from face_ref()".to_string())
                    .or_else(err)
            };
            es.push([get(&pair[0])?, get(&pair[1])?]);
        }
        let r = self.0.borrow_mut().doc.add_fillet(&name, es, radius);
        tl(r).map(|i| i.0 as i64)
    }

    fn sheet_f(
        &mut self,
        name: ImmutableString,
        thickness: Dynamic,
        k: Dynamic,
        radius: Dynamic,
        width: Dynamic,
        segs: Array,
    ) -> RResult<i64> {
        let mut out = Vec::new();
        for s in segs {
            let s: ImmutableString = s
                .try_cast()
                .ok_or_else(|| {
                    "segments are strings like \"F40\" (flat) or \"B90\" (bend, degrees)"
                        .to_string()
                })
                .or_else(err)?;
            let (head, rest) = s.split_at(1);
            let v: f64 = rest
                .trim()
                .parse()
                .or_else(|_| err(format!("'{s}': expected a number after {head}")))?;
            out.push(match head {
                "F" | "f" => SheetSegSpec::Flat {
                    length: Param::length(v),
                },
                "B" | "b" => SheetSegSpec::Bend {
                    angle: Param::angle_deg(v),
                    radius: None,
                },
                _ => return err(format!("'{s}': a segment starts with F (flat) or B (bend)")),
            });
        }
        let spec = SheetMetalSpec {
            thickness: param(&thickness, Kind::Length)?,
            k_factor: Param::scalar(num(&k)?),
            default_radius: param(&radius, Kind::Length)?,
            width: param(&width, Kind::Length)?,
            segs: out,
        };
        let r = self
            .0
            .borrow_mut()
            .doc
            .add(&name, FeatureSpec::SheetMetal(spec));
        tl(r).map(|i| i.0 as i64)
    }

    fn subd_f(
        &mut self,
        name: ImmutableString,
        shape: ImmutableString,
        size: Dynamic,
        tolerance: Dynamic,
    ) -> RResult<i64> {
        let size = num(&size)?;
        let cage = match shape.as_str() {
            "cube" => kw_subd::SubdMesh::cube(size),
            "torus" => kw_subd::SubdMesh::torus(size, size / 3.0, 8, 6),
            "prism5" => kw_subd::SubdMesh::prism(5, size / 2.0, size),
            other => return err(format!("unknown cage '{other}' (cube, torus, prism5)")),
        };
        let tolerance = param(&tolerance, Kind::Length)?;
        let r = self.0.borrow_mut().doc.add(
            &name,
            FeatureSpec::Subd(SubdSpec {
                cage,
                scale: Param::scalar(1.0),
                tolerance,
            }),
        );
        tl(r).map(|i| i.0 as i64)
    }

    fn set_param_f(&mut self, feature: i64, value: Dynamic) -> RResult<()> {
        let p = param(&value, Kind::Length)?;
        let mut inner = self.0.borrow_mut();
        let f = inner
            .doc
            .get(id(feature))
            .ok_or_else(|| format!("there is no feature {feature}"))
            .or_else(err)?;
        let FeatureSpec::Parameter { name, .. } = &f.spec else {
            return err(format!("'{}' is not a parameter", f.name));
        };
        let spec = FeatureSpec::Parameter {
            name: name.clone(),
            value: p,
        };
        tl(inner.doc.replace_spec(id(feature), spec))
    }

    fn suppress_f(&mut self, feature: i64, on: bool) -> RResult<()> {
        tl(self.0.borrow_mut().doc.set_suppressed(id(feature), on))
    }

    fn remove_f(&mut self, feature: i64) -> RResult<()> {
        tl(self.0.borrow_mut().doc.remove(id(feature)).map(|_| ()))
    }

    fn move_f(&mut self, feature: i64, index: i64) -> RResult<()> {
        tl(self
            .0
            .borrow_mut()
            .doc
            .move_feature(id(feature), index.max(0) as usize))
    }

    fn rollback_f(&mut self, feature: i64) -> RResult<()> {
        tl(self.0.borrow_mut().doc.rollback_to(id(feature)))
    }

    /// Evaluate; returns a map `{ ok, failures, features }`.
    fn evaluate_f(&mut self) -> Map {
        let mut inner = self.0.borrow_mut();
        let doc = inner.doc.clone();
        let ev = inner.engine.evaluate(&doc);
        let mut m = Map::new();
        m.insert("ok".into(), Dynamic::from(ev.is_clean()));
        m.insert(
            "evaluated".into(),
            Dynamic::from(ev.stats.executed.len() as i64),
        );
        let fails: Array = ev
            .failures()
            .into_iter()
            .map(|(f, e)| Dynamic::from(format!("{}: {e}", doc.name_of(f))))
            .collect();
        m.insert("failures".into(), Dynamic::from(fails));
        inner.last = Some(ev);
        m
    }

    fn volume_f(&mut self, feature: i64) -> RResult<f64> {
        let inner = self.0.borrow();
        let Some(ev) = &inner.last else {
            return err("call evaluate() first");
        };
        let o = ev
            .output(id(feature))
            .ok_or_else(|| format!("feature {feature} has no result"))
            .or_else(err)?;
        match (o.brep(), o.body()) {
            (Some(b), Some(body)) => b
                .volume(body)
                .ok_or_else(|| "volume could not be computed".to_string())
                .or_else(err),
            _ => err("the feature has no solid"),
        }
    }

    fn face_ref_f(&mut self, feature: i64, role: ImmutableString) -> RResult<RefHandle> {
        let inner = self.0.borrow();
        let Some(ev) = &inner.last else {
            return err("call evaluate() first");
        };
        let name = ev
            .find_name(id(feature), &role)
            .ok_or_else(|| format!("feature {feature} has no face with the role '{role}'"))
            .or_else(err)?;
        ev.reference(id(feature), &name)
            .map(RefHandle)
            .ok_or_else(|| "could not reference the face".to_string())
            .or_else(err)
    }

    fn side_ref_f(&mut self, feature: i64, index: i64) -> RResult<RefHandle> {
        // the index-th face swept from a sketch entity, in name order
        let inner = self.0.borrow();
        let Some(ev) = &inner.last else {
            return err("call evaluate() first");
        };
        let o = ev
            .output(id(feature))
            .ok_or_else(|| format!("feature {feature} has no result"))
            .or_else(err)?;
        let names: Vec<&TopoName> = o
            .names
            .by_name
            .keys()
            .filter(|n| matches!(n.origin, NameOrigin::SketchEdge { .. }))
            .collect();
        let n = names
            .get(index as usize)
            .ok_or_else(|| format!("feature {feature} has only {} swept faces", names.len()))
            .or_else(err)?;
        ev.reference(id(feature), n)
            .map(RefHandle)
            .ok_or_else(|| "could not reference the face".to_string())
            .or_else(err)
    }

    fn count_f(&mut self) -> i64 {
        self.0.borrow().doc.len() as i64
    }

    fn save_f(&mut self, path: ImmutableString) -> RResult<()> {
        let inner = self.0.borrow();
        KwfFile::new(inner.doc.clone())
            .save(std::path::Path::new(path.as_str()))
            .or_else(|e| err(e.to_string()))
    }
}

/// A persistent face reference held by a script.
#[derive(Clone)]
pub struct RefHandle(pub Reference);

#[derive(Debug)]
pub enum ScriptError {
    Script(String),
    /// The script did not end with a document.
    NoDocument,
}

impl fmt::Display for ScriptError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ScriptError::Script(m) => write!(f, "{m}"),
            ScriptError::NoDocument => write!(
                f,
                "the script must end with the document (e.g. a final `d`)"
            ),
        }
    }
}
impl std::error::Error for ScriptError {}

pub fn engine() -> rhai::Engine {
    let mut e = rhai::Engine::new();
    e.set_max_operations(5_000_000);
    e.set_max_call_levels(48);
    e.set_max_expr_depths(64, 32);
    e.set_max_string_size(64 * 1024);
    e.set_max_array_size(100_000);
    e.set_max_map_size(10_000);
    e.register_type_with_name::<ScriptDoc>("Doc");
    e.register_type_with_name::<RefHandle>("FaceRef");
    e.register_fn("new_doc", ScriptDoc::new);
    e.register_fn("param", ScriptDoc::param_f);
    e.register_fn("rect_sketch", ScriptDoc::rect_sketch_f);
    e.register_fn("circle_sketch", ScriptDoc::circle_sketch_f);
    e.register_fn("extrude", ScriptDoc::extrude_f);
    e.register_fn("linear_pattern", ScriptDoc::linear_pattern_f);
    e.register_fn("fillet", ScriptDoc::fillet_f);
    e.register_fn("sheet", ScriptDoc::sheet_f);
    e.register_fn("subd", ScriptDoc::subd_f);
    e.register_fn("set_param", ScriptDoc::set_param_f);
    e.register_fn("suppress", ScriptDoc::suppress_f);
    e.register_fn("remove", ScriptDoc::remove_f);
    e.register_fn("move_to", ScriptDoc::move_f);
    e.register_fn("rollback", ScriptDoc::rollback_f);
    e.register_fn("evaluate", ScriptDoc::evaluate_f);
    e.register_fn("volume", ScriptDoc::volume_f);
    e.register_fn("face_ref", ScriptDoc::face_ref_f);
    e.register_fn("side_ref", ScriptDoc::side_ref_f);
    e.register_fn("len", ScriptDoc::count_f);
    e.register_fn("save", ScriptDoc::save_f);
    e
}

/// Run a script that ends with the document it built.
pub fn run(src: &str) -> Result<Document, ScriptError> {
    let e = engine();
    let mut scope = Scope::new();
    let v = e
        .eval_with_scope::<Dynamic>(&mut scope, src)
        .map_err(|e| ScriptError::Script(e.to_string()))?;
    v.try_cast::<ScriptDoc>()
        .map(|d| d.document())
        .ok_or(ScriptError::NoDocument)
}
