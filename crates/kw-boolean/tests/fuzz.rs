#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Boolean fuzz corpus. Every pair is run through union, intersection and difference.
//!
//! Hard requirements (asserted): no panics, no invalid outputs (a returned body always passes
//! `validate()` with positive volume — the engine verifies this itself), volumes satisfy
//! inclusion–exclusion. Measured and reported: success rate per category; failures must be `Err`.

mod common;
use common::*;
use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Transform3, Vector3};
use rayon::prelude::*;
use std::collections::BTreeMap;
use std::f64::consts::TAU;

#[derive(Default, Clone)]
struct Tally {
    ops: u64,
    ok: u64,
    empty: u64,
    errors: BTreeMap<String, u64>,
    identity_failures: u64,
    first_failures: Vec<String>,
}

fn kind_of(e: &BooleanError) -> String {
    match e {
        BooleanError::BadInput(s) => format!("BadInput({s})"),
        BooleanError::Ssi(_) => "Ssi".into(),
        BooleanError::Tangent(_) => "Tangent".into(),
        BooleanError::Unsupported(s) => format!("Unsupported({s})"),
        BooleanError::Classification(_) => "Classification".into(),
        BooleanError::Topology(_) => "Topology".into(),
        BooleanError::Invalid(_) => "Invalid".into(),
        BooleanError::Empty => "Empty".into(),
    }
}

fn run_case(seed: u64) -> (Case, Tally) {
    let case = gen_case(seed);
    let mut t = Tally::default();
    let mut vols: [Option<f64>; 3] = [None; 3];
    let (mut va, mut vb) = (0.0, 0.0);
    for (i, op) in [Op::Union, Op::Intersect, Op::Subtract]
        .into_iter()
        .enumerate()
    {
        let mut rep = BRep::new();
        let a = build(&mut rep, case.a.0, &case.a.1);
        let b = build(&mut rep, case.b.0, &case.b.1);
        va = rep.volume(a).unwrap();
        vb = rep.volume(b).unwrap();
        t.ops += 1;
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            boolean(&mut rep, a, b, op, &BoolOptions::default())
        }));
        match res {
            Err(_) => {
                *t.errors.entry("PANIC".into()).or_default() += 1;
                t.first_failures
                    .push(format!("seed {seed} {op:?}: PANIC {:?}", case));
            }
            Ok(Ok(r)) => {
                t.ok += 1;
                let v = rep.volume(r.body).unwrap();
                vols[i] = Some(v);
                if !(v > 0.0) || rep.validate().into_result().is_err() {
                    t.identity_failures += 1;
                    println!("IDENTITY seed {seed} {op:?}: returned invalid body (v={v}) {case:?}");
                    t.first_failures
                        .push(format!("seed {seed} {op:?}: returned invalid body (v={v})"));
                }
            }
            Ok(Err(BooleanError::Empty)) => {
                t.empty += 1;
                vols[i] = Some(0.0);
            }
            Ok(Err(e)) => {
                *t.errors.entry(kind_of(&e)).or_default() += 1;
                if case.expect_valid
                    && (t.first_failures.len() < 3 || std::env::var("KW_BOOL_FUZZ_ALL").is_ok())
                {
                    t.first_failures.push(format!(
                        "seed {seed} {op:?} [{}]: {e}\n      case: {:?}",
                        case.category, case
                    ));
                }
            }
        }
    }
    // inclusion–exclusion where available
    if let (Some(u), Some(i)) = (vols[0], vols[1]) {
        if ((u + i) - (va + vb)).abs() > 1e-9 * (va + vb) {
            t.identity_failures += 1;
            println!(
                "IDENTITY seed {seed}: V∪+V∩ = {} ≠ {} {case:?}",
                u + i,
                va + vb
            );
            t.first_failures
                .push(format!("seed {seed}: V∪+V∩ = {} ≠ {}", u + i, va + vb));
        }
    }
    if let (Some(i), Some(d)) = (vols[1], vols[2]) {
        if (d - (va - i)).abs() > 1e-9 * va {
            t.identity_failures += 1;
            println!("IDENTITY seed {seed}: V(A−B) = {d} ≠ {} {case:?}", va - i);
            t.first_failures
                .push(format!("seed {seed}: V(A−B) = {d} ≠ {}", va - i));
        }
    }
    (case, t)
}

#[test]
fn boolean_fuzz_corpus() {
    let n: u64 = std::env::var("KW_BOOL_FUZZ_PAIRS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) { 40 } else { 300 });
    let first: u64 = std::env::var("KW_BOOL_FUZZ_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let results: Vec<(Case, Tally)> = (first..first + n).into_par_iter().map(run_case).collect();
    let mut per_cat: BTreeMap<&str, Tally> = BTreeMap::new();
    let mut identity_failures = 0;
    for (c, t) in &results {
        let e = per_cat.entry(c.category).or_default();
        e.ops += t.ops;
        e.ok += t.ok;
        e.empty += t.empty;
        for (k, v) in &t.errors {
            *e.errors.entry(k.clone()).or_default() += v;
        }
        identity_failures += t.identity_failures;
        e.first_failures.extend(t.first_failures.iter().cloned());
    }
    println!("category           ops      ok  empty  success%   error kinds");
    let (mut exp_ops, mut exp_ok) = (0u64, 0u64);
    for (cat, t) in &per_cat {
        let expected = results
            .iter()
            .find(|(c, _)| c.category == *cat)
            .map(|(c, _)| c.expect_valid)
            .unwrap_or(true);
        let good = t.ok + t.empty;
        println!(
            "{cat:<16} {:>6} {:>7} {:>6}  {:>7.2}%   {:?}",
            t.ops,
            t.ok,
            t.empty,
            100.0 * good as f64 / t.ops as f64,
            t.errors
        );
        if expected {
            exp_ops += t.ops;
            exp_ok += good;
        }
    }
    {
        use std::sync::atomic::Ordering::Relaxed;
        let names = [
            "setup",
            "cuts",
            "vertices",
            "arrangement",
            "classify",
            "assemble+verify",
        ];
        let total: u64 = kw_boolean::PHASE_NANOS
            .iter()
            .map(|x| x.load(Relaxed))
            .sum();
        let ops: u64 = per_cat.values().map(|t| t.ops).sum();
        println!(
            "phase time (thread-seconds): total {:.1} s over {ops} ops = {:.1} ms/op",
            total as f64 * 1e-9,
            total as f64 * 1e-6 / ops as f64
        );
        for (n, c) in names.iter().zip(kw_boolean::PHASE_NANOS.iter()) {
            println!("   {n:<16} {:>8.2} s", c.load(Relaxed) as f64 * 1e-9);
        }
    }
    println!(
        "valid-expected categories: {exp_ok}/{exp_ops} = {:.2}%",
        100.0 * exp_ok as f64 / exp_ops as f64
    );
    for t in per_cat.values() {
        for f in t
            .first_failures
            .iter()
            .take(if std::env::var("KW_BOOL_FUZZ_ALL").is_ok() {
                usize::MAX
            } else {
                6
            })
        {
            println!("  FAIL {f}");
        }
    }
    assert_eq!(
        identity_failures, 0,
        "invalid outputs or volume identities violated"
    );
    assert!(
        !per_cat.values().any(|t| t.errors.contains_key("PANIC")),
        "a Boolean panicked"
    );
    let min_rate: f64 = std::env::var("KW_BOOL_MIN_SUCCESS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    assert!(
        exp_ok as f64 / exp_ops as f64 >= min_rate,
        "success rate below {min_rate}"
    );
}
