#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Boolean fuzz over extruded / revolved / spline-profile bodies against primitives and each other.
//! Hard requirements: no panics, no invalid outputs, inclusion–exclusion of volumes.
//! Reported: per-category success rate (failures must be `Err`).

#[path = "../../kw-boolean/tests/common/mod.rs"]
mod common;

use common::*;
use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::*;
use kw_nurbs::geom::Circle;
use kw_nurbs::{CurveGeom, KnotVector, NurbsCurve};
use rayon::prelude::*;
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};

#[derive(Debug, Clone)]
enum Body {
    Prim(Shape),
    Prism(Vec<(f64, f64)>, f64),
    Slot(f64, f64, f64),
    Spun(Vec<(f64, f64)>),
    Bottle(f64, f64, f64),
    SplinePrism(Vec<(f64, f64)>, f64),
}

fn build_body(rep: &mut BRep, b: &Body, place: &Frame3) -> Option<BodyId> {
    let to_world = Transform3::frame_to_world(place);
    let local = Frame3::world();
    let id = match b {
        Body::Prim(s) => return Some(build(rep, *s, place)),
        Body::Prism(pts, h) => {
            let prof = Profile::polygon(local, pts);
            extrude(
                rep,
                &prof,
                Vector3::new(0.0, 0.0, *h),
                &ExtrudeOptions::default(),
            )
            .ok()?
            .body
        }
        Body::Slot(r, l, h) => {
            let right = Frame3 {
                origin: Point3::new(*l, 0.0, 0.0),
                ..local
            };
            let edges = vec![
                ProfileEdge::line(Point3::new(0.0, -r, 0.0), Point3::new(*l, -*r, 0.0)),
                ProfileEdge::arc(&right, *r, -PI / 2.0, PI / 2.0),
                ProfileEdge::line(Point3::new(*l, *r, 0.0), Point3::new(0.0, *r, 0.0)),
                ProfileEdge::arc(&local, *r, PI / 2.0, 3.0 * PI / 2.0),
            ];
            extrude(
                rep,
                &Profile::new(local, ProfileLoop::new(edges)),
                Vector3::new(0.0, 0.0, *h),
                &ExtrudeOptions::default(),
            )
            .ok()?
            .body
        }
        Body::Spun(pts) => {
            // profile in the xz plane (x = radius)
            let f = Frame3 {
                origin: Point3::origin(),
                x: Vector3::x(),
                y: Vector3::z(),
                z: -Vector3::y(),
            };
            revolve(
                rep,
                &Profile::polygon(f, pts),
                Point3::origin(),
                Vector3::z(),
                TAU,
            )
            .ok()?
            .body
        }
        Body::Bottle(r, h, rn) => {
            // cylinder body + quarter-round shoulder + neck, profile mixes lines and an arc
            let f = Frame3 {
                origin: Point3::origin(),
                x: Vector3::x(),
                y: Vector3::z(),
                z: -Vector3::y(),
            };
            let shoulder = Frame3 {
                origin: Point3::new(r - 0.3, 0.0, *h),
                ..f
            };
            let edges = vec![
                ProfileEdge::line(Point3::new(0.0, 0.0, 0.0), Point3::new(*r, 0.0, 0.0)),
                ProfileEdge::line(Point3::new(*r, 0.0, 0.0), Point3::new(*r, 0.0, *h)),
                ProfileEdge::arc(&shoulder, 0.3, 0.0, PI / 2.0),
                ProfileEdge::line(
                    Point3::new(r - 0.3, 0.0, h + 0.3),
                    Point3::new(*rn, 0.0, h + 0.3),
                ),
                ProfileEdge::line(
                    Point3::new(*rn, 0.0, h + 0.3),
                    Point3::new(*rn, 0.0, h + 1.0),
                ),
                ProfileEdge::line(
                    Point3::new(*rn, 0.0, h + 1.0),
                    Point3::new(0.0, 0.0, h + 1.0),
                ),
                ProfileEdge::line(Point3::new(0.0, 0.0, h + 1.0), Point3::new(0.0, 0.0, 0.0)),
            ];
            revolve(
                rep,
                &Profile::new(f, ProfileLoop::new(edges)),
                Point3::origin(),
                Vector3::z(),
                TAU,
            )
            .ok()?
            .body
        }
        Body::SplinePrism(pts, h) => {
            let mut p3: Vec<Point3<f64>> =
                pts.iter().map(|(x, y)| Point3::new(*x, *y, 0.0)).collect();
            p3.push(p3[0]);
            let c = NurbsCurve::interpolate(&p3, 3).ok()?;
            let dom = c.domain();
            let _ = KnotVector::clamped_uniform(1, 2);
            let edge = ProfileEdge::new(CurveGeom::Nurbs(c), dom);
            extrude(
                rep,
                &Profile::new(local, ProfileLoop::new(vec![edge])),
                Vector3::new(0.0, 0.0, *h),
                &ExtrudeOptions::default(),
            )
            .ok()?
            .body
        }
    };
    // move into place (copy with transform; the original is left unused)
    let mut tmp = BRep::new();
    let _ = &mut tmp;
    let (moved, _) = {
        let src = rep.clone();
        rep.copy_body_from(
            &src,
            id,
            &CopyOptions {
                transform: Some(to_world),
                flip: false,
            },
        )
    };
    Some(moved)
}

fn random_body(r: &mut Rng) -> Body {
    match r.pick(7) {
        0 => {
            // star-ish polygon, CCW
            let n = 4 + r.pick(4);
            let rad = r.range(0.7, 1.6);
            let pts = (0..n)
                .map(|i| {
                    let a = TAU * i as f64 / n as f64 + r.range(-0.15, 0.15);
                    let rr = rad * r.range(0.6, 1.0);
                    (rr * a.cos(), rr * a.sin())
                })
                .collect();
            Body::Prism(pts, r.range(0.6, 2.2))
        }
        1 => Body::Prism(
            vec![
                (0.0, 0.0),
                (2.0, 0.0),
                (2.0, 0.7),
                (0.7, 0.7),
                (0.7, 2.0),
                (0.0, 2.0),
            ],
            r.range(0.6, 2.0),
        ), // L shape
        2 => Body::Slot(r.range(0.3, 0.8), r.range(0.8, 2.0), r.range(0.5, 1.8)),
        3 => {
            let n = 3 + r.pick(3);
            let mut pts = vec![(0.0, 0.0), (r.range(0.8, 1.6), 0.0)];
            let mut z = 0.0;
            for _ in 0..n {
                z += r.range(0.3, 0.8);
                pts.push((r.range(0.2, 1.6), z));
            }
            pts.push((0.0, z));
            Body::Spun(pts)
        }
        4 => Body::Bottle(r.range(0.8, 1.4), r.range(1.0, 2.0), r.range(0.25, 0.5)),
        5 => {
            let n = 5 + r.pick(3);
            let rad = r.range(0.8, 1.5);
            let pts = (0..n)
                .map(|i| {
                    let a = TAU * i as f64 / n as f64;
                    let rr = rad * r.range(0.75, 1.0);
                    (rr * a.cos(), rr * a.sin())
                })
                .collect();
            Body::SplinePrism(pts, r.range(0.6, 2.0))
        }
        _ => Body::Prim(random_shape(r)),
    }
}

#[derive(Debug, Clone)]
struct MCase {
    a: Body,
    b: Body,
    fa: Frame3,
    fb: Frame3,
}

fn gen(seed: u64) -> MCase {
    let mut r = Rng::new(seed ^ 0xABCDEF);
    let a = random_body(&mut r);
    let mut b = random_body(&mut r);
    if matches!((&a, &b), (Body::Prim(_), Body::Prim(_))) {
        b = random_body(&mut r);
    }
    let fa = random_frame(&mut r, 0.2);
    let fb = random_frame(&mut r, 1.0);
    MCase { a, b, fa, fb }
}

#[derive(Default, Clone)]
struct T {
    ops: u64,
    ok: u64,
    empty: u64,
    err: BTreeMap<String, u64>,
    bad: u64,
    msgs: Vec<String>,
    build_failed: u64,
}

fn run(seed: u64) -> T {
    let case = gen(seed);
    let mut t = T::default();
    let mut vols: [Option<f64>; 3] = [None; 3];
    let (mut va, mut vb) = (0.0, 0.0);
    for (i, op) in [Op::Union, Op::Intersect, Op::Subtract]
        .into_iter()
        .enumerate()
    {
        let mut rep = BRep::new();
        let (Some(a), Some(b)) = (
            build_body(&mut rep, &case.a, &case.fa),
            build_body(&mut rep, &case.b, &case.fb),
        ) else {
            t.build_failed += 1;
            return t;
        };
        let (Some(x), Some(y)) = (rep.volume(a), rep.volume(b)) else {
            t.build_failed += 1;
            return t;
        };
        if !(x > 0.0 && y > 0.0) || rep.validate().into_result().is_err() {
            t.build_failed += 1;
            t.msgs.push(format!(
                "seed {seed}: INPUT invalid (va={x}, vb={y}, {:?}) {:?}",
                rep.validate().into_result().err().map(|e| e.len()),
                case
            ));
            return t;
        }
        va = x;
        vb = y;
        t.ops += 1;
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            boolean(&mut rep, a, b, op, &BoolOptions::default())
        })) {
            Err(_) => {
                *t.err.entry("PANIC".into()).or_default() += 1;
                t.msgs.push(format!("seed {seed} {op:?}: PANIC {case:?}"));
            }
            Ok(Ok(r)) => {
                t.ok += 1;
                let v = rep.volume(r.body).unwrap();
                vols[i] = Some(v);
                if !(v > 0.0) || rep.validate().into_result().is_err() {
                    t.bad += 1;
                    t.msgs.push(format!("seed {seed} {op:?}: invalid output"));
                }
            }
            Ok(Err(BooleanError::Empty)) => {
                t.empty += 1;
                vols[i] = Some(0.0);
            }
            Ok(Err(e)) => {
                let k = match &e {
                    BooleanError::Ssi(_) => "Ssi".to_string(),
                    BooleanError::Tangent(_) => "Tangent".into(),
                    BooleanError::Topology(_) => "Topology".into(),
                    BooleanError::Invalid(_) => "Invalid".into(),
                    BooleanError::Classification(_) => "Classification".into(),
                    BooleanError::Unsupported(_) => "Unsupported".into(),
                    BooleanError::BadInput(s) => format!("BadInput({s})"),
                    BooleanError::Empty => "Empty".into(),
                };
                *t.err.entry(k).or_default() += 1;
                if t.msgs.len() < 3 || std::env::var("KW_MODEL_FUZZ_ALL").is_ok() {
                    let name = |b: &Body| match b {
                        Body::Prim(s) => format!("{:?}", s).split('(').next().unwrap().to_string(),
                        Body::Prism(p, _) => format!("Prism{}", p.len()),
                        Body::Slot(..) => "Slot".into(),
                        Body::Spun(_) => "Spun".into(),
                        Body::Bottle(..) => "Bottle".into(),
                        Body::SplinePrism(..) => "SplinePrism".into(),
                    };
                    t.msgs.push(format!(
                        "seed {seed} {op:?} [{} x {}]: {e}",
                        name(&case.a),
                        name(&case.b)
                    ));
                }
            }
        }
    }
    if let (Some(u), Some(i)) = (vols[0], vols[1]) {
        if ((u + i) - (va + vb)).abs() > 1e-8 * (va + vb) {
            t.bad += 1;
            t.msgs
                .push(format!("seed {seed}: V∪+V∩ = {} ≠ {}", u + i, va + vb));
        }
    }
    if let (Some(i), Some(d)) = (vols[1], vols[2]) {
        if (d - (va - i)).abs() > 1e-8 * va {
            t.bad += 1;
            t.msgs
                .push(format!("seed {seed}: V(A−B) = {d} ≠ {}", va - i));
        }
    }
    t
}

#[test]
fn model_body_boolean_fuzz() {
    let n: u64 = std::env::var("KW_MODEL_FUZZ_PAIRS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) { 30 } else { 150 });
    let first: u64 = std::env::var("KW_MODEL_FUZZ_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let res: Vec<T> = (first..first + n).into_par_iter().map(run).collect();
    let (mut ops, mut ok, mut empty, mut bad, mut bf) = (0, 0, 0, 0, 0);
    let mut errs: BTreeMap<String, u64> = BTreeMap::new();
    let mut msgs = Vec::new();
    for t in &res {
        ops += t.ops;
        ok += t.ok;
        empty += t.empty;
        bad += t.bad;
        bf += t.build_failed;
        for (k, v) in &t.err {
            *errs.entry(k.clone()).or_default() += v;
        }
        msgs.extend(t.msgs.iter().cloned());
    }
    println!("{ops} ops over {n} pairs ({bf} pairs skipped: input build failed): {ok} ok, {empty} empty, success {:.2}%  errors {errs:?}", 100.0 * (ok + empty) as f64 / ops as f64);
    for m in msgs
        .iter()
        .take(if std::env::var("KW_MODEL_FUZZ_ALL").is_ok() {
            usize::MAX
        } else {
            12
        })
    {
        println!("  {m}");
    }
    assert_eq!(bad, 0, "invalid outputs or identity violations");
    assert!(!errs.contains_key("PANIC"));
    let min: f64 = std::env::var("KW_MODEL_MIN_SUCCESS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    assert!(
        (ok + empty) as f64 >= min * ops as f64,
        "success below {min}"
    );
    let _ = Circle {
        frame: Frame3::world(),
        radius: 1.0,
    };
}
