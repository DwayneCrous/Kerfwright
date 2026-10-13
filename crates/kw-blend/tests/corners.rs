#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Vertex-blend benchmark: random prismatic parts (orthogonal, oblique, with concave notches) whose
//! corners are filleted with the rolling-ball sphere patch. Success = a valid solid, a mesh volume that
//! agrees with the exact volume, and one sphere patch per eligible corner.

use kw_blend::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::SurfaceGeom;
use rayon::prelude::*;
use std::collections::BTreeMap;

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

fn build(rep: &mut BRep, r: &mut Rng) -> (BodyId, &'static str) {
    match r.pick(4) {
        0 => {
            let (x, y, z) = (r.range(2.0, 4.0), r.range(2.0, 4.0), r.range(1.5, 3.0));
            let f = Frame3 {
                origin: Point3::new(0.5 * x, 0.5 * y, 0.5 * z),
                ..world()
            };
            (make_box(rep, &f, x, y, z).body, "box")
        }
        1 => {
            // oblique prism over a quadrilateral
            let w = r.range(2.0, 3.5);
            let h = r.range(2.0, 3.5);
            let pts = [
                (0.0, 0.0),
                (w, 0.0),
                (w + r.range(-0.4, 0.4), h),
                (r.range(-0.4, 0.4), h),
            ];
            let d = Vector3::new(r.range(-0.8, 0.8), r.range(-0.8, 0.8), r.range(1.8, 3.0));
            (
                extrude(
                    rep,
                    &Profile::polygon(world(), &pts),
                    d,
                    &ExtrudeOptions::default(),
                )
                .unwrap()
                .body,
                "oblique",
            )
        }
        2 => {
            // pentagon prism (obtuse corners in plan)
            let (w, h) = (r.range(2.5, 4.0), r.range(1.8, 3.0));
            let pts = [
                (0.0, 0.0),
                (w, 0.0),
                (w, h),
                (0.5 * w, h + r.range(0.4, 1.0)),
                (0.0, h),
            ];
            (
                extrude(
                    rep,
                    &Profile::polygon(world(), &pts),
                    Vector3::new(0.0, 0.0, r.range(1.5, 2.5)),
                    &ExtrudeOptions::default(),
                )
                .unwrap()
                .body,
                "pentagon",
            )
        }
        _ => {
            // box with a notch cut out of one corner: concave edges and a concave corner
            let s = r.range(4.0, 6.0);
            let f = Frame3 {
                origin: Point3::new(0.5 * s, 0.5 * s, 0.5 * s),
                ..world()
            };
            let big = make_box(rep, &f, s, s, s).body;
            let n = r.range(0.35, 0.6) * s;
            let fnotch = Frame3 {
                origin: Point3::new(s - 0.5 * n + 0.5, s - 0.5 * n + 0.5, s - 0.5 * n + 0.5),
                ..world()
            };
            let notch = make_box(rep, &fnotch, n + 1.0, n + 1.0, n + 1.0).body;
            let res = kw_boolean::boolean(
                rep,
                big,
                notch,
                kw_boolean::Op::Subtract,
                &kw_boolean::BoolOptions::default(),
            )
            .unwrap();
            (res.body, "notch")
        }
    }
}

fn line_edges_at(rep: &BRep, _body: BodyId, v: VertexId) -> Vec<EdgeId> {
    rep.edges
        .iter()
        .filter(|(_, e)| (e.start == v || e.end == v) && e.start != e.end)
        .map(|(id, _)| id)
        .collect()
}

fn body_vertices(rep: &BRep, body: BodyId) -> Vec<VertexId> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                for v in [
                    rep.edges[rep.coedges[co].edge].start,
                    rep.edges[rep.coedges[co].edge].end,
                ] {
                    if seen.insert(v) {
                        out.push(v);
                    }
                }
            }
        }
    }
    out
}

fn body_edges(rep: &BRep, body: BodyId) -> Vec<EdgeId> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if seen.insert(e) {
                    out.push(e);
                }
            }
        }
    }
    out
}

#[derive(Default)]
struct Out {
    ok: u64,
    fail: u64,
    corners: u64,
    bad: u64,
    msgs: Vec<String>,
    cat: BTreeMap<String, (u64, u64)>,
    kinds: BTreeMap<String, u64>,
}

fn run(seed: u64) -> Out {
    let mut r = Rng::new(seed);
    let mut rep = BRep::new();
    let (body, kind) = build(&mut rep, &mut r);
    let mut o = Out::default();
    let v0 = rep.volume(body).unwrap();
    // size relative to the smallest edge
    let mut min_len = f64::INFINITY;
    for e in body_edges(&rep, body) {
        let ed = &rep.edges[e];
        min_len = min_len.min((rep.vertices[ed.start].point - rep.vertices[ed.end].point).norm());
    }
    let size = min_len * r.range(0.08, 0.3);
    // either all the edges at one corner, or every edge of the part
    let mode_all = r.next() < 0.4;
    let edges: Vec<EdgeId> = if mode_all {
        body_edges(&rep, body)
            .into_iter()
            .filter(|e| rep.radial_ring(*e).len() == 2)
            .collect()
    } else {
        let vs: Vec<VertexId> = body_vertices(&rep, body)
            .into_iter()
            .filter(|v| line_edges_at(&rep, body, *v).len() == 3)
            .collect();
        if vs.is_empty() {
            return o;
        }
        line_edges_at(&rep, body, vs[r.pick(vs.len())])
    };
    let key = format!(
        "{kind}/{}",
        if mode_all { "all-edges" } else { "one-corner" }
    );
    if std::env::var("KW_CORNER_SEED").is_ok() {
        println!("seed {seed}: {key}, {} edges, size {size:.4}", edges.len());
    }
    let entry = o.cat.entry(key).or_default();
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        fillet(&mut rep, body, &edges, size)
    }));
    match res {
        Err(_) => {
            o.fail += 1;
            entry.1 += 1;
            *o.kinds.entry("PANIC".into()).or_default() += 1;
        }
        Ok(Ok(res)) => {
            let v = rep.volume(res.body).unwrap_or(f64::NAN);
            let spheres = rep
                .body_faces(res.body)
                .iter()
                .filter(|f| matches!(rep.surfaces[rep.faces[**f].surface], SurfaceGeom::Sphere(_)))
                .count() as u64;
            let valid = rep.validate().is_ok();
            let tess = kw_tess::tessellate_body(
                &rep,
                res.body,
                &kw_tess::TessOptions {
                    chord_tol: 0.004,
                    angle_tol: 0.2,
                },
            )
            .map(|t| (t.mesh.signed_volume() - v).abs() < 0.03 * v + 1e-6)
            .unwrap_or(false);
            if v.is_finite() && v > 0.0 && valid && tess && (v - v0).abs() < 0.3 * v0 {
                o.ok += 1;
                entry.0 += 1;
                o.corners += spheres;
            } else {
                o.bad += 1;
                o.fail += 1;
                entry.1 += 1;
                o.msgs.push(format!(
                    "seed {seed} {kind}: questionable result (v0 {v0}, v {v}, valid {valid}, tess {tess})"
                ));
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
            if o.msgs.len() < 3 || std::env::var("KW_CORNER_ALL").is_ok() {
                o.msgs.push(format!(
                    "seed {seed} {kind} [{} edges, size {size:.3}]: {e}",
                    edges.len()
                ));
            }
        }
    }
    o
}

#[test]
fn corner_benchmark() {
    if let Some(seed) = std::env::var("KW_CORNER_SEED")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
    {
        let o = run(seed);
        println!(
            "seed {seed}: ok {} fail {} corners {} {:?} {:?}",
            o.ok, o.fail, o.corners, o.kinds, o.msgs
        );
        return;
    }
    let n: u64 = std::env::var("KW_CORNER_PARTS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(if cfg!(debug_assertions) { 20 } else { 200 });
    let res: Vec<Out> = (0..n).into_par_iter().map(run).collect();
    let (mut ok, mut fail, mut bad, mut corners) = (0, 0, 0, 0);
    let mut cats: BTreeMap<String, (u64, u64)> = BTreeMap::new();
    let mut kinds: BTreeMap<String, u64> = BTreeMap::new();
    let mut msgs = Vec::new();
    for o in &res {
        ok += o.ok;
        fail += o.fail;
        bad += o.bad;
        corners += o.corners;
        for (k, v) in &o.cat {
            let e = cats.entry(k.clone()).or_default();
            e.0 += v.0;
            e.1 += v.1;
        }
        for (k, v) in &o.kinds {
            *kinds.entry(k.clone()).or_default() += v;
        }
        msgs.extend(o.msgs.iter().cloned());
    }
    let total = ok + fail;
    println!(
        "{total} parts: {ok} succeeded, {fail} failed ({:.1}%), {corners} sphere patches built",
        100.0 * ok as f64 / total.max(1) as f64
    );
    for (k, (a, b)) in &cats {
        println!("  {k:<22} {a:>4} ok {b:>4} failed");
    }
    println!("failure kinds: {kinds:#?}");
    for m in msgs.iter().take(if std::env::var("KW_CORNER_ALL").is_ok() {
        usize::MAX
    } else {
        12
    }) {
        println!("  {m}");
    }
    assert_eq!(bad, 0, "questionable bodies returned");
    assert!(!kinds.contains_key("PANIC"));
    let min: f64 = std::env::var("KW_CORNER_MIN_SUCCESS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.0);
    assert!(ok as f64 >= min * total as f64, "success below {min}");
    assert!(corners > 0, "no vertex blend was built");
}
