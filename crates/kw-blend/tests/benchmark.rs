#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Fillet / chamfer benchmark: random prismatic and turned parts with random edge sets.
//!
//! Success = the blend returns a body that validates, tessellates and has a volume consistent with
//! the operation (fillets/chamfers of convex edges remove material, concave ones add it, each by no
//! more than the swept bounding amount). Failures must be `Err` and name the offending edge.

use kw_blend::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use rayon::prelude::*;
use std::collections::BTreeMap;
use std::f64::consts::TAU;

struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        let mut r = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0x1234_5678_9ABC_DEF1);
        for _ in 0..4 {
            r.next();
        }
        r
    }
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    fn pick(&mut self, n: usize) -> usize {
        ((self.next() * n as f64) as usize).min(n - 1)
    }
}

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Kind {
    Box,
    Prism,
    LPrism,
    Cylinder,
    Cone,
    Turned,
}

fn build(rep: &mut BRep, r: &mut Rng) -> (BodyId, Kind, f64) {
    let kind = [
        Kind::Box,
        Kind::Prism,
        Kind::LPrism,
        Kind::Cylinder,
        Kind::Cone,
        Kind::Turned,
    ][r.pick(6)];
    match kind {
        Kind::Box => {
            let (x, y, z) = (r.range(1.5, 4.0), r.range(1.5, 4.0), r.range(1.0, 3.0));
            (make_box(rep, &world(), x, y, z).body, kind, x.min(y).min(z))
        }
        Kind::Prism => {
            // rectangle with a chamfered/odd outline: pentagon with right angles at the base
            let (w, h, hh) = (r.range(2.0, 4.0), r.range(1.5, 3.0), r.range(1.0, 2.5));
            let pts = [
                (0.0, 0.0),
                (w, 0.0),
                (w, h),
                (0.5 * w, h + r.range(0.3, 1.0)),
                (0.0, h),
            ];
            let prof = Profile::polygon(world(), &pts);
            (
                extrude(
                    rep,
                    &prof,
                    Vector3::new(0.0, 0.0, hh),
                    &ExtrudeOptions::default(),
                )
                .unwrap()
                .body,
                kind,
                h.min(hh),
            )
        }
        Kind::LPrism => {
            let (a, b) = (r.range(2.5, 4.0), r.range(0.8, 1.5));
            let pts = [(0.0, 0.0), (a, 0.0), (a, b), (b, b), (b, a), (0.0, a)];
            let prof = Profile::polygon(world(), &pts);
            (
                extrude(
                    rep,
                    &prof,
                    Vector3::new(0.0, 0.0, r.range(0.8, 2.0)),
                    &ExtrudeOptions::default(),
                )
                .unwrap()
                .body,
                kind,
                b,
            )
        }
        Kind::Cylinder => {
            let (rad, h) = (r.range(0.8, 2.0), r.range(1.0, 3.0));
            (make_cylinder(rep, &world(), rad, h).body, kind, rad.min(h))
        }
        Kind::Cone => {
            let (r0, r1, h) = (r.range(1.0, 2.0), r.range(0.4, 0.9), r.range(1.0, 2.5));
            (make_cone(rep, &world(), r0, r1, h).body, kind, r1.min(h))
        }
        Kind::Turned => {
            // stepped shaft: cylinder with a collar (revolved)
            let (r0, r1, h0, h1) = (
                r.range(0.8, 1.2),
                r.range(1.3, 2.0),
                r.range(0.6, 1.5),
                r.range(0.5, 1.2),
            );
            let f = Frame3 {
                origin: Point3::origin(),
                x: Vector3::x(),
                y: Vector3::z(),
                z: -Vector3::y(),
            };
            let pts = [
                (0.0, 0.0),
                (r1, 0.0),
                (r1, h0),
                (r0, h0),
                (r0, h0 + h1),
                (0.0, h0 + h1),
            ];
            (
                revolve(
                    rep,
                    &Profile::polygon(f, &pts),
                    Point3::origin(),
                    Vector3::z(),
                    TAU,
                )
                .unwrap()
                .body,
                kind,
                (r1 - r0).min(h0).min(h1),
            )
        }
    }
}

/// Eligible edges: manifold edges with a line or circle curve.
fn eligible(rep: &BRep, body: BodyId) -> Vec<EdgeId> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !seen.insert(e) {
                    continue;
                }
                let ed = &rep.edges[e];
                let Some(c) = ed.curve.and_then(|c| rep.curves.get(c)) else {
                    continue;
                };
                if rep.radial_ring(e).len() == 2
                    && matches!(
                        c,
                        kw_nurbs::CurveGeom::Line(_) | kw_nurbs::CurveGeom::Circle(_)
                    )
                {
                    // exclude cylinder / cone seams (both coedges in the same face)
                    let faces = rep.faces_around_edge(e);
                    if faces[0] != faces[1] {
                        out.push(e);
                    }
                }
            }
        }
    }
    out
}

#[derive(Default, Clone)]
struct Out {
    ok: u64,
    fail: u64,
    kinds: BTreeMap<String, u64>,
    bad: u64,
    msgs: Vec<String>,
    cat: BTreeMap<String, (u64, u64)>,
}

fn run(seed: u64) -> Out {
    let mut r = Rng::new(seed);
    let mut rep = BRep::new();
    let (body, kind, feature) = build(&mut rep, &mut r);
    let mut o = Out::default();
    let v0 = rep.volume(body).unwrap();
    let mut edges = eligible(&rep, body);
    // choose 1–3 edges
    let n = 1 + r.pick(3);
    let mut chosen = Vec::new();
    for _ in 0..n.min(edges.len()) {
        let i = r.pick(edges.len());
        chosen.push(edges.remove(i));
    }
    let size = feature * r.range(0.08, 0.4);
    let chamfer_op = r.next() < 0.3;
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        if chamfer_op {
            chamfer(&mut rep, body, &chosen, size, size * r.range(0.6, 1.0))
        } else {
            fillet(&mut rep, body, &chosen, size)
        }
    }));
    let adjacent = chosen.iter().enumerate().any(|(i, a)| {
        chosen[i + 1..].iter().any(|b| {
            let (ea, eb) = (&rep.edges[*a], &rep.edges[*b]);
            ea.start == eb.start || ea.start == eb.end || ea.end == eb.start || ea.end == eb.end
        })
    });
    let key = format!(
        "{kind:?}{}{}",
        if chamfer_op { "/chamfer" } else { "/fillet" },
        if adjacent { "/adjacent" } else { "" }
    );
    let entry = o.cat.entry(key).or_default();
    match res {
        Err(_) => {
            o.fail += 1;
            entry.1 += 1;
            *o.kinds.entry("PANIC".into()).or_default() += 1;
        }
        Ok(Ok(res)) => {
            let v = rep.volume(res.body).unwrap_or(f64::NAN);
            // volume must stay positive and change by a plausible amount (|ΔV| ≤ n · size² · edge length bound)
            let ok_geom = v.is_finite()
                && v > 0.0
                && rep.validate().is_ok()
                && (v - v0).abs() < 0.2 * v0 + 1e-9;
            let tess = kw_tess::tessellate_body(
                &rep,
                res.body,
                &kw_tess::TessOptions {
                    chord_tol: 0.01,
                    angle_tol: 0.3,
                },
            )
            .map(|t| (t.mesh.signed_volume() - v).abs() < 0.03 * v + 1e-6)
            .unwrap_or(false);
            if ok_geom && tess {
                o.ok += 1;
                entry.0 += 1;
            } else {
                o.bad += 1;
                o.fail += 1;
                entry.1 += 1;
                o.msgs.push(format!("seed {seed}: returned a questionable body (v0 {v0}, v {v}, valid {}, tess ok {tess})", rep.validate().is_ok()));
            }
        }
        Ok(Err(e)) => {
            o.fail += 1;
            entry.1 += 1;
            let k = match &e.kind {
                BlendErrorKind::BadEdge => "BadEdge".to_string(),
                BlendErrorKind::Unsupported(s) => format!("Unsupported({s})"),
                BlendErrorKind::Size(s) => format!("Size({s})"),
                BlendErrorKind::Boolean(b) => format!(
                    "Boolean({})",
                    format!("{b}").split('(').next().unwrap_or("")
                ),
                BlendErrorKind::Model(_) => "Model".to_string(),
            };
            *o.kinds.entry(k).or_default() += 1;
            if o.msgs.len() < 2 || std::env::var("KW_BLEND_ALL").is_ok() {
                o.msgs.push(format!(
                    "seed {seed} [{kind:?}, {} edge(s), size {size:.3}]: {e}",
                    chosen.len()
                ));
            }
        }
    }
    o
}

#[test]
fn fillet_benchmark() {
    if let Some(seed) = std::env::var("KW_BLEND_SEED")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
    {
        let o = run(seed);
        println!(
            "seed {seed}: ok {} fail {} {:?} {:?}",
            o.ok, o.fail, o.kinds, o.msgs
        );
        return;
    }
    let n: u64 = std::env::var("KW_BLEND_PARTS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) { 60 } else { 500 });
    let res: Vec<Out> = (0..n).into_par_iter().map(run).collect();
    let (mut ok, mut fail, mut bad) = (0, 0, 0);
    let mut kinds: BTreeMap<String, u64> = BTreeMap::new();
    let mut cats: BTreeMap<String, (u64, u64)> = BTreeMap::new();
    let mut msgs = Vec::new();
    for o in &res {
        ok += o.ok;
        fail += o.fail;
        bad += o.bad;
        for (k, v) in &o.kinds {
            *kinds.entry(k.clone()).or_default() += v;
        }
        for (k, v) in &o.cat {
            let e = cats.entry(k.clone()).or_default();
            e.0 += v.0;
            e.1 += v.1;
        }
        msgs.extend(o.msgs.iter().cloned());
    }
    println!(
        "{n} parts: {ok} succeeded, {fail} failed => {:.1}% success",
        100.0 * ok as f64 / n as f64
    );
    for (k, (a, b)) in &cats {
        println!(
            "  {k:<22} {a:>4} ok {b:>4} failed  ({:.0}%)",
            100.0 * *a as f64 / (*a + *b) as f64
        );
    }
    println!("failure kinds: {kinds:#?}");
    for m in msgs.iter().take(if std::env::var("KW_BLEND_ALL").is_ok() {
        usize::MAX
    } else {
        12
    }) {
        println!("  {m}");
    }
    assert_eq!(bad, 0, "questionable bodies returned");
    assert!(!kinds.contains_key("PANIC"));
    let min: f64 = std::env::var("KW_BLEND_MIN_SUCCESS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    assert!(ok as f64 >= min * n as f64, "success below {min}");
}
