//! The incremental evaluator.
//!
//! Every feature's result is keyed by an *input hash*: the feature's specification with all
//! expressions evaluated, plus the content hashes of the outputs it reads. A key that was seen before
//! is a cache hit, so an edit re-executes only what it really changes, and when a re-executed feature
//! produces the same output as before (a *no-op edit*) everything downstream still hits the cache
//! (early cutoff). Independent features run in parallel, level by level, on an explicit thread pool.

use crate::doc::Document;
use crate::eval::{EvalError, EvalOutput, Product};
use crate::id::{hash_serde, FeatureId, Hasher128};
use crate::naming::{Reference, TopoName};
use crate::ops::{execute, Inputs};
use crate::spec::FeatureSpec;
use kw_expr::Quantity;
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Bumped when a kernel change can alter results, so that stale caches are never trusted.
pub const KERNEL_VERSION: u64 = 4;

#[derive(Debug, Clone, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn cancel(&self) {
        self.0.store(true, Ordering::SeqCst);
    }
    pub fn reset(&self) {
        self.0.store(false, Ordering::SeqCst);
    }
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::SeqCst)
    }
}

#[derive(Debug, Clone)]
pub enum FeatureState {
    Ok(Arc<EvalOutput>),
    /// The feature itself failed; `last_good` is what it produced before.
    Failed {
        error: EvalError,
        last_good: Option<Arc<EvalOutput>>,
    },
    /// An input failed, so the feature was not evaluated.
    Suspended {
        cause: FeatureId,
        last_good: Option<Arc<EvalOutput>>,
    },
}

#[derive(Debug, Clone, Default)]
pub struct RunStats {
    /// Features whose operation was actually executed.
    pub executed: Vec<FeatureId>,
    pub cache_hits: usize,
    pub failed: usize,
    pub suspended: usize,
}

#[derive(Debug, Clone)]
pub struct Evaluation {
    pub states: HashMap<FeatureId, FeatureState>,
    /// Evaluated features in timeline order (suppressed and rolled-back features are absent).
    pub order: Vec<FeatureId>,
    pub stats: RunStats,
    pub input_hashes: HashMap<FeatureId, u128>,
}

impl Evaluation {
    pub fn state(&self, id: FeatureId) -> Option<&FeatureState> {
        self.states.get(&id)
    }
    pub fn output(&self, id: FeatureId) -> Option<&Arc<EvalOutput>> {
        match self.states.get(&id) {
            Some(FeatureState::Ok(o)) => Some(o),
            _ => None,
        }
    }
    pub fn error(&self, id: FeatureId) -> Option<&EvalError> {
        match self.states.get(&id) {
            Some(FeatureState::Failed { error, .. }) => Some(error),
            _ => None,
        }
    }
    pub fn failures(&self) -> Vec<(FeatureId, &EvalError)> {
        self.order
            .iter()
            .filter_map(|id| self.error(*id).map(|e| (*id, e)))
            .collect()
    }
    pub fn is_clean(&self) -> bool {
        self.order
            .iter()
            .all(|id| matches!(self.states.get(id), Some(FeatureState::Ok(_))))
    }
    /// The output of the last evaluated solid feature.
    pub fn final_solid(&self) -> Option<(&FeatureId, &Arc<EvalOutput>)> {
        self.order
            .iter()
            .rev()
            .find_map(|id| match self.states.get(id) {
                Some(FeatureState::Ok(o)) if matches!(o.product, Product::Solid(_)) => {
                    Some((id, o))
                }
                _ => None,
            })
    }
    /// One hash over every feature's result: identical documents evaluate to identical hashes.
    pub fn document_hash(&self) -> u128 {
        let mut h = Hasher128::new();
        for id in &self.order {
            h.u64(id.0);
            match self.states.get(id) {
                Some(FeatureState::Ok(o)) => {
                    h.u64(1).u128(o.output_hash);
                }
                Some(FeatureState::Failed { .. }) => {
                    h.u64(2);
                }
                _ => {
                    h.u64(3);
                }
            }
        }
        h.finish()
    }
    /// Geometry hash and name-set hash of the final solid.
    pub fn final_hashes(&self) -> Option<(u128, u128)> {
        self.final_solid().map(|(_, o)| (o.geom_hash, o.names_hash))
    }
    /// All names of the final solid.
    pub fn final_names(&self) -> Vec<TopoName> {
        self.final_solid()
            .map(|(_, o)| o.names.by_name.keys().cloned().collect())
            .unwrap_or_default()
    }

    /// The value of every parameter of `doc`, by name.
    pub fn scope(&self, doc: &Document) -> HashMap<String, Quantity> {
        let mut m = HashMap::new();
        for f in &doc.features {
            if let FeatureSpec::Parameter { name, .. } = &f.spec {
                if let Some(q) = self.output(f.id).and_then(|o| o.quantity()) {
                    m.insert(name.clone(), q);
                }
            }
        }
        m
    }

    /// The flat pattern of a sheet-metal feature (bend allowances included).
    pub fn flat_pattern(
        &self,
        doc: &Document,
        feature: FeatureId,
    ) -> Result<kw_sheet::FlatPattern, EvalError> {
        let f = doc
            .get(feature)
            .ok_or_else(|| EvalError::Dependency(format!("there is no feature {feature}")))?;
        let FeatureSpec::SheetMetal(_) = &f.spec else {
            return Err(EvalError::Dependency(format!(
                "'{}' is not a sheet-metal feature",
                f.name
            )));
        };
        let resolved = f
            .spec
            .resolved(&self.scope(doc))
            .map_err(|e| EvalError::Expression(e.to_string()))?;
        let FeatureSpec::SheetMetal(m) = &resolved else {
            unreachable!()
        };
        let strip = crate::ops::sheet_strip(m)?;
        kw_sheet::unfold(&strip).map_err(|e| EvalError::Kernel(e.to_string()))
    }

    /// A persistent reference to the face of `feature`'s output called `name`.
    pub fn reference(&self, feature: FeatureId, name: &TopoName) -> Option<Reference> {
        let o = self.output(feature)?;
        let rep = o.brep()?;
        let f = o.names.face(name)?;
        o.names.reference_to(rep, f)
    }

    /// The name of a face of `feature`'s output whose origin is the given role (primitive faces and
    /// the faces swept from sketch entities).
    pub fn find_name(&self, feature: FeatureId, role: &str) -> Option<TopoName> {
        use crate::naming::NameOrigin;
        let o = self.output(feature)?;
        o.names
            .by_name
            .keys()
            .find(|n| match &n.origin {
                NameOrigin::Primitive { role: r } => r == role,
                NameOrigin::SketchEdge { role: r, .. } => r == role,
                _ => false,
            })
            .cloned()
    }

    /// The document with every reference that was followed by lineage or geometry rewritten to point
    /// at the face it resolved to, so that it resolves by name from now on.
    pub fn healed(&self, doc: &Document) -> Document {
        let mut d = doc.clone();
        for f in &mut d.features {
            let Some(out) = self.output(f.id) else {
                continue;
            };
            if out.notes.is_empty() {
                continue;
            }
            let body_out = doc
                .index_of(f.id)
                .and_then(|i| doc.body_input(i))
                .and_then(|b| self.output(b));
            let Some(body_out) = body_out else { continue };
            let notes = out.notes.clone();
            f.spec.for_each_reference_mut(&mut |r| {
                if let Some(n) = notes.iter().find(|n| n.reference == *r) {
                    if let Some(new) = body_out.brep().and_then(|rep| {
                        body_out
                            .names
                            .face(&n.to)
                            .and_then(|fa| body_out.names.reference_to(rep, fa))
                    }) {
                        *r = new;
                    }
                }
            });
        }
        d
    }
}

pub struct Engine {
    pool: rayon::ThreadPool,
    threads: usize,
    cache: HashMap<u128, (Arc<EvalOutput>, u64)>,
    failures: HashMap<u128, (EvalError, u64)>,
    prev: HashMap<FeatureId, Arc<EvalOutput>>,
    generation: u64,
    pub cancel: CancelToken,
}

enum Outcome {
    Ok(Arc<EvalOutput>),
    Failed(EvalError),
    Suspended(FeatureId),
}

struct Done {
    id: FeatureId,
    key: Option<u128>,
    outcome: Outcome,
    executed: bool,
    hit: bool,
}

fn panic_message(p: Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = p.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = p.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic".into()
    }
}

impl Engine {
    pub fn new() -> Engine {
        Engine::with_threads(
            std::thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(1)
                .min(8),
        )
    }

    pub fn with_threads(threads: usize) -> Engine {
        let threads = threads.max(1);
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .stack_size(32 << 20)
            .build()
            .expect("thread pool");
        Engine {
            pool,
            threads,
            cache: HashMap::new(),
            failures: HashMap::new(),
            prev: HashMap::new(),
            generation: 0,
            cancel: CancelToken::new(),
        }
    }

    pub fn threads(&self) -> usize {
        self.threads
    }

    pub fn clear_cache(&mut self) {
        self.cache.clear();
        self.failures.clear();
    }

    pub fn cache_len(&self) -> usize {
        self.cache.len()
    }

    /// Evaluate the document, reusing everything that did not change.
    pub fn evaluate(&mut self, doc: &Document) -> Evaluation {
        self.generation += 1;
        let generation = self.generation;
        let n = doc.active_len();
        let active: Vec<usize> = (0..n).filter(|&i| !doc.features[i].suppressed).collect();
        let pos: HashMap<FeatureId, usize> = doc
            .features
            .iter()
            .enumerate()
            .map(|(i, f)| (f.id, i))
            .collect();
        let deps: HashMap<usize, Vec<FeatureId>> = active
            .iter()
            .map(|&i| (i, doc.dependencies_at(i)))
            .collect();
        let bodies: HashMap<usize, Option<FeatureId>> =
            active.iter().map(|&i| (i, doc.body_input(i))).collect();
        // longest-path levels
        let mut level: HashMap<usize, usize> = HashMap::new();
        let mut levels: Vec<Vec<usize>> = Vec::new();
        for &i in &active {
            let l = deps[&i]
                .iter()
                .filter_map(|d| pos.get(d).and_then(|p| level.get(p)))
                .max()
                .map_or(0, |m| m + 1);
            level.insert(i, l);
            if levels.len() <= l {
                levels.resize(l + 1, Vec::new());
            }
            levels[l].push(i);
        }

        let mut states: HashMap<FeatureId, FeatureState> = HashMap::new();
        let mut hashes: HashMap<FeatureId, u128> = HashMap::new();
        let mut stats = RunStats::default();
        let cancel = self.cancel.clone();

        for lvl in &levels {
            let results: Vec<Done> = {
                let cache = &self.cache;
                let failures = &self.failures;
                let states_ref = &states;
                let deps = &deps;
                let bodies = &bodies;
                self.pool.install(|| {
                    lvl.par_iter()
                        .map(|&i| {
                            run_one(
                                doc, i, &deps[&i], bodies[&i], states_ref, cache, failures, &cancel,
                            )
                        })
                        .collect()
                })
            };
            for d in results {
                if d.executed {
                    stats.executed.push(d.id);
                }
                if d.hit {
                    stats.cache_hits += 1;
                }
                if let Some(k) = d.key {
                    hashes.insert(d.id, k);
                }
                let last_good = self.prev.get(&d.id).cloned();
                let state = match d.outcome {
                    Outcome::Ok(o) => {
                        if let Some(k) = d.key {
                            self.cache.insert(k, (o.clone(), generation));
                        }
                        FeatureState::Ok(o)
                    }
                    Outcome::Failed(e) => {
                        stats.failed += 1;
                        if let (Some(k), true) = (d.key, e != EvalError::Cancelled) {
                            self.failures.insert(k, (e.clone(), generation));
                        }
                        FeatureState::Failed {
                            error: e,
                            last_good,
                        }
                    }
                    Outcome::Suspended(c) => {
                        stats.suspended += 1;
                        FeatureState::Suspended {
                            cause: c,
                            last_good,
                        }
                    }
                };
                states.insert(d.id, state);
            }
        }
        // keep what is used now and what was used recently (undo / redo hit these)
        for (k, v) in self.cache.iter_mut() {
            if hashes.values().any(|h| h == k) {
                v.1 = generation;
            }
        }
        let limit = (4 * active.len()).max(256);
        if self.cache.len() > limit {
            self.cache.retain(|_, v| generation - v.1 < 6);
            self.failures.retain(|_, v| generation - v.1 < 6);
        }
        self.prev = states
            .iter()
            .filter_map(|(id, s)| match s {
                FeatureState::Ok(o) => Some((*id, o.clone())),
                _ => None,
            })
            .collect();
        Evaluation {
            order: active.iter().map(|&i| doc.features[i].id).collect(),
            states,
            stats,
            input_hashes: hashes,
        }
    }
}

impl Default for Engine {
    fn default() -> Self {
        Engine::new()
    }
}

#[allow(clippy::too_many_arguments)]
fn run_one(
    doc: &Document,
    i: usize,
    deps: &[FeatureId],
    body: Option<FeatureId>,
    states: &HashMap<FeatureId, FeatureState>,
    cache: &HashMap<u128, (Arc<EvalOutput>, u64)>,
    failures: &HashMap<u128, (EvalError, u64)>,
    cancel: &CancelToken,
) -> Done {
    let f = &doc.features[i];
    let fail = |e: EvalError| Done {
        id: f.id,
        key: None,
        outcome: Outcome::Failed(e),
        executed: false,
        hit: false,
    };
    let mut inputs: HashMap<FeatureId, Arc<EvalOutput>> = HashMap::new();
    for d in deps {
        match states.get(d) {
            Some(FeatureState::Ok(o)) => {
                inputs.insert(*d, o.clone());
            }
            Some(_) => {
                return Done {
                    id: f.id,
                    key: None,
                    outcome: Outcome::Suspended(*d),
                    executed: false,
                    hit: false,
                }
            }
            None => {
                return fail(EvalError::Dependency(format!(
                    "'{}' depends on '{}', which is suppressed",
                    f.name,
                    doc.name_of(*d)
                )))
            }
        }
    }
    let mut scope: HashMap<String, Quantity> = HashMap::new();
    for d in deps {
        if let Some(df) = doc.get(*d) {
            if let FeatureSpec::Parameter { name, .. } = &df.spec {
                if let Some(q) = inputs.get(d).and_then(|o| o.quantity()) {
                    scope.insert(name.clone(), q);
                }
            }
        }
    }
    let resolved = match f.spec.resolved(&scope) {
        Ok(r) => r,
        Err(e) => return fail(EvalError::Expression(e.to_string())),
    };
    let key = {
        let mut h = Hasher128::new();
        h.u64(KERNEL_VERSION).u64(f.id.0);
        h.u128(hash_serde(&resolved));
        h.u64(body.map_or(0, |b| b.0));
        let mut ds: Vec<&FeatureId> = deps.iter().collect();
        ds.sort();
        for d in ds {
            h.u64(d.0).u128(inputs[d].output_hash);
        }
        h.finish()
    };
    if let Some((o, _)) = cache.get(&key) {
        return Done {
            id: f.id,
            key: Some(key),
            outcome: Outcome::Ok(o.clone()),
            executed: false,
            hit: true,
        };
    }
    if let Some((e, _)) = failures.get(&key) {
        return Done {
            id: f.id,
            key: Some(key),
            outcome: Outcome::Failed(e.clone()),
            executed: false,
            hit: true,
        };
    }
    if cancel.is_cancelled() {
        return Done {
            id: f.id,
            key: None,
            outcome: Outcome::Failed(EvalError::Cancelled),
            executed: false,
            hit: false,
        };
    }
    let inp = Inputs {
        deps: &inputs,
        body,
    };
    let r = catch_unwind(AssertUnwindSafe(|| execute(f.id, &resolved, &inp)));
    let outcome = match r {
        Ok(Ok(o)) => Outcome::Ok(Arc::new(o)),
        Ok(Err(e)) => Outcome::Failed(e),
        Err(p) => Outcome::Failed(EvalError::Kernel(format!(
            "internal error in '{}': {}",
            f.name,
            panic_message(p)
        ))),
    };
    Done {
        id: f.id,
        key: Some(key),
        outcome,
        executed: true,
        hit: false,
    }
}

#[allow(dead_code)]
fn _unused(_: BTreeMap<u8, u8>) {}
