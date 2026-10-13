//! What evaluating a feature produces.

use crate::geomhash::geometry_hash;
use crate::id::{FeatureId, Hasher128};
use crate::naming::{NameMap, Reference, ResolveKind, TopoName};
use crate::spec::BodyOp;
use crate::tools::ToolGen;
use kw_brep::{BRep, ShapeHistory};
use kw_expr::Quantity;
use kw_math::Frame3;
use kw_sketch::{EntityId, Sketch};
use std::fmt;
use std::sync::Arc;

#[derive(Debug, Clone, PartialEq)]
pub enum EvalError {
    /// A reference that could not be resolved with confidence.
    Unresolved(Box<Unresolved>),
    /// A reference that is invalid for another reason.
    Reference(String),
    /// The kernel refused the operation.
    Kernel(String),
    /// A feature this one needs is missing, suppressed or failed.
    Dependency(String),
    /// An expression could not be evaluated.
    Expression(String),
    /// The sketch could not be solved or turned into a profile.
    Sketch(String),
    Cancelled,
}

/// A reference the naming layer would not guess: nothing plausible (`candidates` empty) or several
/// plausible faces (`ambiguous`).
#[derive(Debug, Clone, PartialEq)]
pub struct Unresolved {
    pub reference: Reference,
    pub candidates: Vec<(TopoName, f64)>,
    pub ambiguous: bool,
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EvalError::Unresolved(u) => {
                if u.ambiguous {
                    write!(
                        f,
                        "reference to {:?}: {} faces could be it; choose one",
                        u.reference.name.origin,
                        u.candidates.len()
                    )
                } else {
                    write!(
                        f,
                        "reference to {:?} no longer exists and nothing similar was found",
                        u.reference.name.origin
                    )
                }
            }
            EvalError::Reference(m) => write!(f, "reference: {m}"),
            EvalError::Kernel(m) => write!(f, "{m}"),
            EvalError::Dependency(m) => write!(f, "{m}"),
            EvalError::Expression(m) => write!(f, "expression: {m}"),
            EvalError::Sketch(m) => write!(f, "sketch: {m}"),
            EvalError::Cancelled => write!(f, "cancelled"),
        }
    }
}
impl std::error::Error for EvalError {}

#[derive(Debug, Clone)]
pub struct SketchProduct {
    pub sketch: Sketch,
    pub plane: Frame3,
    /// `entities[i]` is the sketch entity with label `i`.
    pub entities: Vec<EntityId>,
}

/// How a solid feature applied tools to the body, kept so that a pattern or mirror can repeat it.
#[derive(Debug, Clone)]
pub struct ToolProduct {
    pub gens: Vec<ToolGen>,
    pub op: BodyOp,
    pub source: FeatureId,
}

#[derive(Debug, Clone)]
pub struct SolidProduct {
    pub brep: Arc<BRep>,
    pub tool: Option<Arc<ToolProduct>>,
}

#[derive(Debug, Clone)]
pub enum Product {
    Param(Quantity),
    Datum(Frame3),
    Sketch(Arc<SketchProduct>),
    Solid(SolidProduct),
}

/// A reference that did not resolve by its own name but was followed (by lineage or geometry).
#[derive(Debug, Clone, PartialEq)]
pub struct Resolution {
    pub reference: Reference,
    pub to: TopoName,
    pub kind: ResolveKind,
}

#[derive(Debug, Clone)]
pub struct EvalOutput {
    pub product: Product,
    /// Names of the faces of the solid (empty for other products).
    pub names: NameMap,
    pub history: ShapeHistory,
    pub geom_hash: u128,
    pub names_hash: u128,
    /// `(geometry, names)` combined: equal hashes mean dependents need not be re-evaluated.
    pub output_hash: u128,
    pub notes: Vec<Resolution>,
}

fn frame_hash(h: &mut Hasher128, f: &Frame3) {
    for v in [f.origin.coords, f.x, f.y, f.z] {
        h.f64(v.x).f64(v.y).f64(v.z);
    }
}

fn combine(tag: u64, geom: u128, names: u128) -> u128 {
    let mut h = Hasher128::new();
    h.u64(tag).u128(geom).u128(names);
    h.finish()
}

impl EvalOutput {
    fn plain(product: Product, geom: u128, notes: Vec<Resolution>) -> EvalOutput {
        EvalOutput {
            product,
            names: NameMap::default(),
            history: ShapeHistory::default(),
            geom_hash: geom,
            names_hash: 0,
            output_hash: combine(1, geom, 0),
            notes,
        }
    }

    pub fn param(q: Quantity) -> EvalOutput {
        let mut h = Hasher128::new();
        h.f64(q.value).u128(crate::id::hash_serde(&q.dim));
        EvalOutput::plain(Product::Param(q), h.finish(), Vec::new())
    }

    pub fn datum(f: Frame3, notes: Vec<Resolution>) -> EvalOutput {
        let mut h = Hasher128::new();
        frame_hash(&mut h, &f);
        EvalOutput::plain(Product::Datum(f), h.finish(), notes)
    }

    pub fn sketch(s: SketchProduct, notes: Vec<Resolution>) -> EvalOutput {
        let mut h = Hasher128::new();
        frame_hash(&mut h, &s.plane);
        for v in &s.sketch.vars.values {
            h.f64(*v);
        }
        let mut rec = crate::recipe::SketchRecipe::from_sketch(&s.sketch);
        rec.vars.clear();
        h.u128(crate::id::hash_serde(&rec));
        EvalOutput::plain(Product::Sketch(Arc::new(s)), h.finish(), notes)
    }

    pub fn solid(
        brep: BRep,
        body: kw_brep::BodyId,
        names: NameMap,
        history: ShapeHistory,
        tool: Option<Arc<ToolProduct>>,
        notes: Vec<Resolution>,
    ) -> EvalOutput {
        let geom = geometry_hash(&brep, body, &names);
        let names_hash = names.set_hash();
        EvalOutput {
            product: Product::Solid(SolidProduct {
                brep: Arc::new(brep),
                tool,
            }),
            names,
            history,
            geom_hash: geom,
            names_hash,
            output_hash: combine(2, geom, names_hash),
            notes,
        }
    }

    pub fn solid_product(&self) -> Option<&SolidProduct> {
        match &self.product {
            Product::Solid(s) => Some(s),
            _ => None,
        }
    }
    pub fn brep(&self) -> Option<&Arc<BRep>> {
        self.solid_product().map(|s| &s.brep)
    }
    pub fn body(&self) -> Option<kw_brep::BodyId> {
        self.brep().and_then(|b| b.bodies.keys().next())
    }
    /// Is the product a surface (a sheet body that encloses no volume) rather than a solid?
    pub fn is_sheet(&self) -> bool {
        match (self.brep(), self.body()) {
            (Some(rep), Some(b)) => rep
                .bodies
                .get(b)
                .is_some_and(|b| b.kind == kw_brep::BodyKind::Sheet),
            _ => false,
        }
    }
    pub fn quantity(&self) -> Option<Quantity> {
        match &self.product {
            Product::Param(q) => Some(*q),
            _ => None,
        }
    }
}
