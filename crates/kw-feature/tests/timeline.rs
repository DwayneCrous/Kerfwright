//! Timeline edits under random pressure: an edit that would give a feature something that does not come
//! before it, or make dependencies circular, is always refused with an explanation, and refused edits
//! leave the document untouched.

use kw_expr::Param;
use kw_feature::*;
use std::collections::{BTreeSet, HashMap};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

fn mm(v: f64) -> Param {
    Param::length(v)
}

/// Generated documents carry their own record of who depends on whom (the oracle).
type Deps = HashMap<FeatureId, BTreeSet<FeatureId>>;

struct Model {
    doc: Document,
    deps: HashMap<FeatureId, BTreeSet<FeatureId>>,
    params: Vec<(FeatureId, String)>,
}

fn expr_over(names: &[&String]) -> Param {
    if names.is_empty() {
        mm(5.0)
    } else {
        Param::expr(
            &names
                .iter()
                .map(|n| n.to_string())
                .collect::<Vec<_>>()
                .join(" + "),
        )
        .unwrap()
    }
}

fn generate(r: &mut Rng, n_params: usize, n_solids: usize) -> Model {
    let mut m = Model {
        doc: Document::new(),
        deps: HashMap::new(),
        params: Vec::new(),
    };
    for i in 0..n_params {
        let name = format!("p{i}");
        let k = r.below(3).min(m.params.len());
        let used: Vec<&(FeatureId, String)> =
            (0..k).map(|_| &m.params[r.below(m.params.len())]).collect();
        let mut names: Vec<&String> = used.iter().map(|u| &u.1).collect();
        names.sort();
        names.dedup();
        let p = expr_over(&names);
        let id = m.doc.add_parameter(&name, p).unwrap();
        m.deps.insert(id, used.iter().map(|u| u.0).collect());
        m.params.push((id, name));
    }
    for i in 0..n_solids {
        let (pid, pname) = m.params[r.below(m.params.len())].clone();
        let sk = m
            .doc
            .add_rect_sketch(
                &format!("sk{i}"),
                PlaneSpec::Xy(mm(0.0)),
                mm(0.0),
                mm(0.0),
                Param::expr(&format!("{pname} + 5 mm")).unwrap(),
                mm(10.0),
            )
            .unwrap();
        m.deps.insert(sk, [pid].into_iter().collect());
        let (qid, qname) = m.params[r.below(m.params.len())].clone();
        let ex = m
            .doc
            .add_extrude(
                &format!("ex{i}"),
                sk,
                Param::expr(&format!("{qname} + 1 mm")).unwrap(),
                BodyOp::New,
            )
            .unwrap();
        m.deps.insert(ex, [sk, qid].into_iter().collect());
    }
    m
}

fn cyclic(deps: &HashMap<FeatureId, BTreeSet<FeatureId>>) -> bool {
    fn visit(
        n: FeatureId,
        deps: &HashMap<FeatureId, BTreeSet<FeatureId>>,
        state: &mut HashMap<FeatureId, u8>,
    ) -> bool {
        match state.get(&n) {
            Some(1) => return true,
            Some(2) => return false,
            _ => {}
        }
        state.insert(n, 1);
        if let Some(ds) = deps.get(&n) {
            for d in ds {
                if visit(*d, deps, state) {
                    return true;
                }
            }
        }
        state.insert(n, 2);
        false
    }
    let mut state = HashMap::new();
    deps.keys().any(|k| visit(*k, deps, &mut state))
}

/// Does the order respect every dependency (and is there no cycle)?
fn oracle_valid(doc: &Document, deps: &HashMap<FeatureId, BTreeSet<FeatureId>>) -> bool {
    if cyclic(deps) {
        return false;
    }
    let pos: HashMap<FeatureId, usize> = doc
        .features
        .iter()
        .enumerate()
        .map(|(i, f)| (f.id, i))
        .collect();
    deps.iter()
        .all(|(f, ds)| ds.iter().all(|d| pos[d] < pos[f]))
}

#[test]
fn invalid_edits_are_always_rejected_and_valid_ones_always_accepted() {
    let mut r = Rng(0xC0FFEE);
    let (mut tried, mut rejected, mut accepted, mut explained_cycles) = (0, 0, 0, 0);
    for round in 0..40 {
        let mut m = generate(&mut r, 4 + round % 6, 3 + round % 5);
        for _ in 0..60 {
            let n = m.doc.len();
            let before = m.doc.clone();
            let kind = r.below(3);
            let (result, new_deps): (Result<(), TimelineError>, Option<Deps>) = match kind {
                0 => {
                    // move a feature somewhere else
                    let f = m.doc.features[r.below(n)].id;
                    let to = r.below(n);
                    (m.doc.move_feature(f, to), None)
                }
                1 => {
                    // make a parameter depend on another parameter (possibly itself, possibly later)
                    let (pid, pname) = m.params[r.below(m.params.len())].clone();
                    let (qid, qname) = m.params[r.below(m.params.len())].clone();
                    let spec = FeatureSpec::Parameter {
                        name: pname,
                        value: Param::expr(&format!("{qname} + 1 mm")).unwrap(),
                    };
                    let mut nd = m.deps.clone();
                    nd.insert(pid, [qid].into_iter().collect());
                    (m.doc.replace_spec(pid, spec), Some(nd))
                }
                _ => {
                    // remove a feature
                    let f = m.doc.features[r.below(n)].id;
                    let has_dependents = m.deps.values().any(|ds| ds.contains(&f));
                    let res = m.doc.remove(f).map(|_| ());
                    assert_eq!(
                        res.is_err(),
                        has_dependents,
                        "removal of a feature with dependents must be refused"
                    );
                    if res.is_ok() {
                        m.deps.remove(&f);
                        m.params.retain(|(id, _)| *id != f);
                        if m.params.is_empty() {
                            break;
                        }
                    }
                    (Ok(()), None)
                }
            };
            if kind == 2 {
                continue;
            }
            tried += 1;
            let deps = new_deps.clone().unwrap_or_else(|| m.deps.clone());
            match (&result, kind) {
                (Ok(()), _) => {
                    accepted += 1;
                    // an accepted edit must leave a document the oracle agrees is valid
                    if kind == 0 {
                        assert!(
                            oracle_valid(&m.doc, &m.deps),
                            "an accepted move broke the dependency order"
                        );
                    } else {
                        m.deps = deps;
                        assert!(
                            oracle_valid(&m.doc, &m.deps),
                            "an accepted edit made the graph invalid"
                        );
                    }
                    assert!(m.doc.validate().is_ok());
                }
                (Err(e), _) => {
                    rejected += 1;
                    let text = e.to_string();
                    assert!(text.len() > 20, "a refusal must explain itself: '{text}'");
                    assert_eq!(m.doc, before, "a refused edit changed the document");
                    // the oracle must agree the edit was invalid
                    match kind {
                        0 => {
                            // re-run the move on a scratch copy of the oracle: it must be invalid
                            // (checked below through the independent order test)
                        }
                        _ => assert!(
                            !oracle_valid(&m.doc, &deps)
                                || matches!(e, TimelineError::UnknownParameter { .. }),
                            "a valid edit was refused: {text}"
                        ),
                    }
                    if matches!(e, TimelineError::Cycle { .. }) {
                        explained_cycles += 1;
                        assert!(text.contains('→'), "a cycle names its path: '{text}'");
                    }
                }
            }
        }
    }
    eprintln!("{tried} edits tried: {accepted} accepted, {rejected} rejected ({explained_cycles} cycles named)");
    assert!(rejected > 100 && accepted > 100);
}

#[test]
fn every_move_is_judged_exactly_like_the_oracle() {
    // exhaustive over small documents: try every (feature, destination) pair
    let mut r = Rng(7);
    let mut checked = 0;
    for round in 0..12 {
        let m = generate(&mut r, 3 + round % 3, 2 + round % 3);
        let n = m.doc.len();
        for f in 0..n {
            for to in 0..n {
                let id = m.doc.features[f].id;
                // the order the move would produce
                let mut order: Vec<FeatureId> = m.doc.features.iter().map(|x| x.id).collect();
                let x = order.remove(f);
                order.insert(to, x);
                let pos: HashMap<FeatureId, usize> =
                    order.iter().enumerate().map(|(i, id)| (*id, i)).collect();
                let should_accept = m
                    .deps
                    .iter()
                    .all(|(a, ds)| ds.iter().all(|d| pos[d] < pos[a]));
                let mut d2 = m.doc.clone();
                let res = d2.move_feature(id, to);
                assert_eq!(
                    res.is_ok(),
                    should_accept,
                    "moving feature {f} to {to}: {res:?}"
                );
                if let Err(e) = res {
                    assert!(
                        e.to_string().contains("before") || e.to_string().contains("cycle"),
                        "{e}"
                    );
                    assert_eq!(d2, m.doc);
                }
                checked += 1;
            }
        }
    }
    eprintln!("{checked} moves judged exactly like the oracle");
    assert!(checked > 500);
}

#[test]
fn a_broken_reference_can_be_repaired() {
    // a pocket whose wall is the target of a fillet; the edit removes the wall, the fillet reports it
    let mut doc = Document::new();
    let s = doc
        .add_rect_sketch(
            "S",
            PlaneSpec::Xy(mm(0.0)),
            mm(0.0),
            mm(0.0),
            mm(20.0),
            mm(20.0),
        )
        .unwrap();
    let ext = doc.add_extrude("E", s, mm(10.0), BodyOp::New).unwrap();
    let mut engine = Engine::with_threads(1);
    let ev = engine.evaluate(&doc);
    let sides: Vec<TopoName> = ev
        .output(ext)
        .unwrap()
        .names
        .by_name
        .keys()
        .filter(|n| matches!(n.origin, NameOrigin::SketchEdge { .. }))
        .cloned()
        .collect();
    // pick two adjacent sides by trial
    let mut fillet = None;
    'o: for i in 0..4 {
        for j in i + 1..4 {
            let (a, b) = (
                ev.reference(ext, &sides[i]).unwrap(),
                ev.reference(ext, &sides[j]).unwrap(),
            );
            let mut d2 = doc.clone();
            let f = d2
                .add_fillet("F", vec![[a.clone(), b.clone()]], mm(1.0))
                .unwrap();
            if Engine::with_threads(1).evaluate(&d2).output(f).is_some() {
                doc = d2;
                fillet = Some((f, a, b));
                break 'o;
            }
        }
    }
    let (f, a, _b) = fillet.unwrap();
    assert!(engine.evaluate(&doc).output(f).is_some());
    // Wreck the reference: a name from nowhere, with a fingerprint that matches nothing
    let ghost = Reference {
        name: TopoName::primitive(ext, "ghost", 0),
        fingerprint: GeomFingerprint {
            kind: naming::SurfKind::Sphere,
            dir: None,
            center: [500.0; 3],
            extent: 3.0,
            radius: 3.0,
            adjacency: 1,
            area: 1.0,
        },
        scale: 30.0,
    };
    let broken_name = a.name.clone();
    let mut spec = doc.get(f).unwrap().spec.clone();
    spec.for_each_reference_mut(&mut |r| {
        if r.name == broken_name {
            *r = ghost.clone();
        }
    });
    doc.replace_spec(f, spec).unwrap();
    let ev = engine.evaluate(&doc);
    match ev.error(f) {
        Some(EvalError::Unresolved(u)) => assert!(!u.ambiguous && u.reference.name == ghost.name),
        other => panic!("expected an unresolved reference, got {other:?}"),
    }
    // repair: point the broken reference at the face it should have been
    let ev_ok = engine.evaluate(&{
        let mut d = doc.clone();
        d.rollback_to(ext).unwrap();
        d
    });
    let fresh = ev_ok.reference(ext, &broken_name).unwrap();
    assert_eq!(doc.repair_reference(f, &ghost.name, &fresh).unwrap(), 1);
    let ev = engine.evaluate(&doc);
    assert!(ev.output(f).is_some(), "{:?}", ev.state(f));
    assert!(doc.references_of(f).iter().any(|r| r.name == broken_name));
}
