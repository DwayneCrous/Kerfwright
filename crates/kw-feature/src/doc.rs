//! The document: an ordered timeline of features, with validated edits.
//!
//! Every edit is checked before it is applied and leaves the document untouched when it is refused.
//! A refusal explains itself: a reorder that would make a feature read something that comes after it
//! names the features involved and, when the dependency really is circular, the cycle.

use crate::id::FeatureId;
use crate::spec::FeatureSpec;
use kw_expr::is_reserved;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt;
use std::sync::Arc;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub suppressed: bool,
    pub spec: FeatureSpec,
}

/// How a body is named, coloured and made of.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Appearance {
    pub name: Option<String>,
    pub colour: Option<[u8; 3]>,
    pub material: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Document {
    pub features: Vec<Feature>,
    /// Name, colour and material of bodies, keyed by the id of the feature that started the body.
    #[serde(default)]
    pub appearance: BTreeMap<u64, Appearance>,
    /// Features at positions ≥ `rollback` are not evaluated (the rollback bar). `None` = evaluate all.
    pub rollback: Option<usize>,
    pub(crate) next_id: u64,
}

impl Default for Document {
    fn default() -> Self {
        Document::new()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum TimelineError {
    UnknownFeature(FeatureId),
    /// The edit would make a feature (transitively) depend on itself.
    Cycle {
        path: Vec<String>,
    },
    /// The edit would make a feature read another that comes after it in the timeline.
    ForwardReference {
        feature: String,
        depends_on: String,
    },
    DuplicateName(String),
    BadParameterName(String),
    UnknownParameter {
        feature: String,
        name: String,
    },
    HasDependents {
        feature: String,
        dependents: Vec<String>,
    },
    BadIndex {
        index: usize,
        len: usize,
    },
    BadExpression {
        feature: String,
        message: String,
    },
}

impl fmt::Display for TimelineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TimelineError::UnknownFeature(id) => write!(f, "there is no feature {id}"),
            TimelineError::Cycle { path } => write!(
                f,
                "this would create a dependency cycle: {}",
                path.join(" → ")
            ),
            TimelineError::ForwardReference {
                feature,
                depends_on,
            } => write!(
                f,
                "'{feature}' would come before '{depends_on}', which it depends on; a feature can only use features that precede it"
            ),
            TimelineError::DuplicateName(n) => write!(f, "the name '{n}' is already used"),
            TimelineError::BadParameterName(n) => write!(
                f,
                "'{n}' is not a valid parameter name (letters, digits and _, not starting with a digit, and not a unit or constant)"
            ),
            TimelineError::UnknownParameter { feature, name } => write!(
                f,
                "'{feature}' uses the parameter '{name}', which does not exist"
            ),
            TimelineError::HasDependents {
                feature,
                dependents,
            } => write!(
                f,
                "'{feature}' cannot be removed: {} depend on it",
                dependents.join(", ")
            ),
            TimelineError::BadIndex { index, len } => {
                write!(f, "position {index} is outside the timeline (length {len})")
            }
            TimelineError::BadExpression { feature, message } => {
                write!(f, "'{feature}': {message}")
            }
        }
    }
}
impl std::error::Error for TimelineError {}

fn valid_param_name(n: &str) -> bool {
    let mut cs = n.chars();
    match cs.next() {
        Some(c) if c.is_ascii_alphabetic() || c == '_' => {}
        _ => return false,
    }
    n.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') && !is_reserved(n)
}

impl Document {
    pub fn new() -> Document {
        Document {
            features: Vec::new(),
            appearance: BTreeMap::new(),
            rollback: None,
            next_id: 1,
        }
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }
    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    pub fn index_of(&self, id: FeatureId) -> Option<usize> {
        self.features.iter().position(|f| f.id == id)
    }
    pub fn get(&self, id: FeatureId) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }
    pub fn name_of(&self, id: FeatureId) -> String {
        self.get(id)
            .map(|f| f.name.clone())
            .unwrap_or_else(|| id.to_string())
    }
    pub fn find_by_name(&self, name: &str) -> Option<FeatureId> {
        self.features.iter().find(|f| f.name == name).map(|f| f.id)
    }

    /// Number of features evaluated (those before the rollback bar).
    pub fn active_len(&self) -> usize {
        self.rollback
            .map_or(self.features.len(), |r| r.min(self.features.len()))
    }

    /// The parameter feature named `name`.
    pub fn parameter(&self, name: &str) -> Option<FeatureId> {
        self.features.iter().find_map(|f| match &f.spec {
            FeatureSpec::Parameter { name: n, .. } if n == name => Some(f.id),
            _ => None,
        })
    }

    /// The nearest earlier solid-producing feature before position `index` (suppressed ones included).
    fn previous_solid(&self, index: usize) -> Option<FeatureId> {
        self.features[..index]
            .iter()
            .rev()
            .find(|f| f.spec.produces_solid())
            .map(|f| f.id)
    }

    /// The bodies of the document: solid features that no later feature builds on (a copy does not use up
    /// its source, a Combine with `keep_tool` does not use up its tool). Only features that are evaluated
    /// (before the rollback bar, not suppressed) count. In feature order.
    pub fn heads(&self) -> Vec<FeatureId> {
        let limit = self
            .rollback
            .unwrap_or(self.features.len())
            .min(self.features.len());
        let mut used: HashSet<FeatureId> = HashSet::new();
        for j in 0..limit {
            let f = &self.features[j];
            if f.suppressed || !f.spec.produces_solid() {
                continue;
            }
            if f.spec.consumes_body() {
                if let Some(b) = self.body_input(j) {
                    used.insert(b);
                }
            }
            if let FeatureSpec::Combine(c) = &f.spec {
                used.insert(c.target);
                if !c.keep_tool {
                    used.insert(c.tool);
                }
            }
        }
        self.features[..limit]
            .iter()
            .filter(|f| f.spec.produces_solid() && !f.suppressed && !used.contains(&f.id))
            .map(|f| f.id)
            .collect()
    }

    /// The feature that started the body `head` belongs to (follow the body input back to its start).
    pub fn body_root(&self, head: FeatureId) -> FeatureId {
        let mut cur = head;
        for _ in 0..=self.features.len() {
            let Some(i) = self.index_of(cur) else {
                break;
            };
            // a copy starts a body of its own
            if !self.features[i].spec.consumes_body() {
                break;
            }
            match self.body_input(i) {
                Some(prev) if prev != cur => cur = prev,
                _ => break,
            }
        }
        cur
    }

    /// Set (or clear, with a default value) the appearance of the body started by `root`.
    pub fn set_appearance(&mut self, root: FeatureId, a: Appearance) {
        if a == Appearance::default() {
            self.appearance.remove(&root.0);
        } else {
            self.appearance.insert(root.0, a);
        }
    }

    pub fn appearance_of(&self, root: FeatureId) -> Option<&Appearance> {
        self.appearance.get(&root.0)
    }

    /// The feature whose body the feature at `index` modifies, looking through suppressed features.
    pub fn body_input(&self, index: usize) -> Option<FeatureId> {
        let f = &self.features[index];
        if !f.spec.needs_body() {
            return None;
        }
        let mut cur = f
            .spec
            .explicit_base()
            .or_else(|| self.previous_solid(index));
        // a suppressed feature passes its own input body through
        let mut guard = 0;
        while let Some(id) = cur {
            let Some(i) = self.index_of(id) else {
                return Some(id);
            };
            if !self.features[i].suppressed {
                return Some(id);
            }
            cur = self.body_input(i);
            guard += 1;
            if guard > self.features.len() + 1 {
                return None;
            }
        }
        None
    }

    /// All features the feature at `index` reads: referenced features, parameters its expressions use,
    /// and the body it modifies.
    pub fn dependencies_at(&self, index: usize) -> Vec<FeatureId> {
        let f = &self.features[index];
        let mut v = f.spec.explicit_dependencies();
        for name in f.spec.free_vars() {
            if let Some(p) = self.parameter(&name) {
                v.push(p);
            }
        }
        if let Some(b) = self.body_input(index) {
            v.push(b);
        }
        v.sort();
        v.dedup();
        v.retain(|d| *d != f.id);
        v
    }

    pub fn dependencies_of(&self, id: FeatureId) -> Vec<FeatureId> {
        self.index_of(id)
            .map(|i| self.dependencies_at(i))
            .unwrap_or_default()
    }

    /// Features that read `id` (directly).
    pub fn dependents_of(&self, id: FeatureId) -> Vec<FeatureId> {
        (0..self.features.len())
            .filter(|&i| self.dependencies_at(i).contains(&id))
            .map(|i| self.features[i].id)
            .collect()
    }

    /// Full validation: unique names, parameters exist, every dependency exists and precedes its user.
    pub fn validate(&self) -> Result<(), TimelineError> {
        let mut seen: HashMap<&str, ()> = HashMap::new();
        let mut params: HashMap<&str, usize> = HashMap::new();
        for (i, f) in self.features.iter().enumerate() {
            if seen.insert(&f.name, ()).is_some() {
                return Err(TimelineError::DuplicateName(f.name.clone()));
            }
            if let FeatureSpec::Parameter { name, value } = &f.spec {
                if !valid_param_name(name) {
                    return Err(TimelineError::BadParameterName(name.clone()));
                }
                if params.insert(name, i).is_some() {
                    return Err(TimelineError::DuplicateName(name.clone()));
                }
                let _ = value;
            }
        }
        for (i, f) in self.features.iter().enumerate() {
            for name in f.spec.free_vars() {
                match params.get(name.as_str()) {
                    None => {
                        return Err(TimelineError::UnknownParameter {
                            feature: f.name.clone(),
                            name,
                        })
                    }
                    Some(&pi) if pi == i => {
                        return Err(TimelineError::Cycle {
                            path: vec![f.name.clone(), f.name.clone()],
                        })
                    }
                    _ => {}
                }
            }
            for d in self.dependencies_at(i) {
                let Some(j) = self.index_of(d) else {
                    return Err(TimelineError::UnknownFeature(d));
                };
                if j > i {
                    return Err(self.order_violation(i, j));
                }
            }
        }
        Ok(())
    }

    /// The feature at `i` reads the feature at `j > i`: a cycle if `j` (transitively) reads `i`, else a
    /// plain forward reference.
    fn order_violation(&self, i: usize, j: usize) -> TimelineError {
        let target = self.features[i].id;
        // path j → ... → i through dependencies
        let mut stack = vec![(self.features[j].id, vec![self.features[j].name.clone()])];
        let mut seen = std::collections::HashSet::new();
        while let Some((id, path)) = stack.pop() {
            if !seen.insert(id) {
                continue;
            }
            for d in self.dependencies_of(id) {
                if d == target {
                    let mut p = vec![self.features[i].name.clone()];
                    p.extend(path.iter().cloned());
                    p.push(self.features[i].name.clone());
                    return TimelineError::Cycle { path: p };
                }
                let mut np = path.clone();
                np.push(self.name_of(d));
                stack.push((d, np));
            }
        }
        TimelineError::ForwardReference {
            feature: self.features[i].name.clone(),
            depends_on: self.features[j].name.clone(),
        }
    }

    fn fresh_id(&mut self) -> FeatureId {
        let id = FeatureId(self.next_id);
        self.next_id += 1;
        id
    }

    fn checked(&mut self, candidate: Document) -> Result<(), TimelineError> {
        candidate.validate()?;
        *self = candidate;
        Ok(())
    }

    /// Append a feature.
    pub fn add(&mut self, name: &str, spec: FeatureSpec) -> Result<FeatureId, TimelineError> {
        let n = self.features.len();
        self.insert_at(n, name, spec)
    }

    /// Insert a feature at `index` (0 = first).
    pub fn insert_at(
        &mut self,
        index: usize,
        name: &str,
        spec: FeatureSpec,
    ) -> Result<FeatureId, TimelineError> {
        if index > self.features.len() {
            return Err(TimelineError::BadIndex {
                index,
                len: self.features.len(),
            });
        }
        let mut c = self.clone();
        let id = c.fresh_id();
        c.features.insert(
            index,
            Feature {
                id,
                name: name.to_string(),
                suppressed: false,
                spec,
            },
        );
        if let Some(r) = c.rollback.as_mut() {
            if index < *r {
                *r += 1;
            }
        }
        let next = c.next_id;
        self.checked(c)?;
        self.next_id = next;
        Ok(id)
    }

    /// Remove a feature that nothing depends on.
    pub fn remove(&mut self, id: FeatureId) -> Result<Feature, TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        let deps = self.dependents_of(id);
        if !deps.is_empty() {
            return Err(TimelineError::HasDependents {
                feature: self.features[i].name.clone(),
                dependents: deps.iter().map(|d| self.name_of(*d)).collect(),
            });
        }
        let mut c = self.clone();
        let f = c.features.remove(i);
        if let Some(r) = c.rollback.as_mut() {
            if i < *r {
                *r -= 1;
            }
        }
        self.checked(c)?;
        Ok(f)
    }

    /// Replace a feature's specification (a parameter change, a new reference).
    pub fn replace_spec(&mut self, id: FeatureId, spec: FeatureSpec) -> Result<(), TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        let mut c = self.clone();
        c.features[i].spec = spec;
        self.checked(c)
    }

    pub fn rename(&mut self, id: FeatureId, name: &str) -> Result<(), TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        let mut c = self.clone();
        c.features[i].name = name.to_string();
        self.checked(c)
    }

    /// Move a feature to position `new_index` (the position it will have afterwards).
    pub fn move_feature(&mut self, id: FeatureId, new_index: usize) -> Result<(), TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        if new_index >= self.features.len() {
            return Err(TimelineError::BadIndex {
                index: new_index,
                len: self.features.len(),
            });
        }
        let mut c = self.clone();
        let f = c.features.remove(i);
        c.features.insert(new_index, f);
        // the rollback bar keeps its feature count
        self.checked(c)
    }

    pub fn set_suppressed(&mut self, id: FeatureId, suppressed: bool) -> Result<(), TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        let mut c = self.clone();
        c.features[i].suppressed = suppressed;
        self.checked(c)
    }

    /// Evaluate only the features up to and including `id` (the rollback bar sits after it).
    pub fn rollback_to(&mut self, id: FeatureId) -> Result<(), TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        // a bar after the last feature is no rollback at all
        self.rollback = (i + 1 < self.features.len()).then_some(i + 1);
        Ok(())
    }

    /// Is the bar really before the end (so that adding features is refused)?
    pub fn rolled_back(&self) -> bool {
        self.rollback.is_some_and(|r| r < self.features.len())
    }

    pub fn clear_rollback(&mut self) {
        self.rollback = None;
    }

    /// Point every reference to `broken` held by feature `id` at `replacement` instead: the repair of a
    /// reference the naming layer could not resolve (or resolved differently from what the user meant).
    /// Returns how many references were changed.
    pub fn repair_reference(
        &mut self,
        id: FeatureId,
        broken: &crate::naming::TopoName,
        replacement: &crate::naming::Reference,
    ) -> Result<usize, TimelineError> {
        let i = self.index_of(id).ok_or(TimelineError::UnknownFeature(id))?;
        let mut c = self.clone();
        let mut n = 0;
        c.features[i].spec.for_each_reference_mut(&mut |r| {
            if r.name == *broken {
                *r = replacement.clone();
                n += 1;
            }
        });
        self.checked(c)?;
        Ok(n)
    }

    /// The references feature `id` holds.
    pub fn references_of(&self, id: FeatureId) -> Vec<crate::naming::Reference> {
        let Some(f) = self.get(id) else {
            return Vec::new();
        };
        let mut spec = f.spec.clone();
        let mut v = Vec::new();
        spec.for_each_reference_mut(&mut |r| v.push(r.clone()));
        v
    }

    /// A stable 128-bit hash of the document's recipe (independent of caches).
    pub fn content_hash(&self) -> u128 {
        crate::id::hash_serde(self)
    }
}

/// Undo / redo as a stack of immutable snapshots.
#[derive(Debug, Clone)]
pub struct History {
    snapshots: Vec<Arc<Document>>,
    pos: usize,
}

impl History {
    pub fn new(doc: Document) -> History {
        History {
            snapshots: vec![Arc::new(doc)],
            pos: 0,
        }
    }
    pub fn current(&self) -> Arc<Document> {
        self.snapshots[self.pos].clone()
    }
    /// Apply an edit to a copy of the current document; on success it becomes a new snapshot (redo
    /// states are dropped). A refused edit changes nothing.
    pub fn edit<T>(
        &mut self,
        f: impl FnOnce(&mut Document) -> Result<T, TimelineError>,
    ) -> Result<T, TimelineError> {
        let mut d = (*self.snapshots[self.pos]).clone();
        let r = f(&mut d)?;
        self.snapshots.truncate(self.pos + 1);
        self.snapshots.push(Arc::new(d));
        self.pos += 1;
        Ok(r)
    }
    pub fn can_undo(&self) -> bool {
        self.pos > 0
    }
    pub fn can_redo(&self) -> bool {
        self.pos + 1 < self.snapshots.len()
    }
    pub fn undo(&mut self) -> bool {
        if self.can_undo() {
            self.pos -= 1;
            true
        } else {
            false
        }
    }
    pub fn redo(&mut self) -> bool {
        if self.can_redo() {
            self.pos += 1;
            true
        } else {
            false
        }
    }
}
