#![allow(unused_imports, dead_code)]
//! Fillet performance: a zig-zag prism with ~`n` side faces, 50 well-separated vertical edges rounded in one call.

use kw_blend::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::Curve3;
use std::f64::consts::TAU;
use std::time::Instant;

fn zigzag(rep: &mut BRep, n: usize) -> BodyId {
    let pts: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let a = TAU * i as f64 / n as f64;
            let r = 10.0 + if i % 2 == 0 { 0.0 } else { 0.5 };
            (r * a.cos(), r * a.sin())
        })
        .collect();
    let f = Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };
    extrude(
        rep,
        &Profile::polygon(f, &pts),
        Vector3::new(0.0, 0.0, 2.0),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body
}

/// Vertical edges (parallel to z) of the body.
fn vertical_edges(rep: &BRep, body: BodyId) -> Vec<EdgeId> {
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
                if let Some(c) = ed.curve.and_then(|c| rep.curves.get(c)) {
                    if matches!(c, kw_nurbs::CurveGeom::Line(l) if l.dir.normalize().cross(&Vector3::z()).norm() < 1e-9)
                    {
                        out.push(e);
                    }
                }
            }
        }
    }
    out
}

#[test]
fn fifty_fillets_on_a_two_thousand_face_body() {
    if cfg!(debug_assertions) {
        return;
    }
    let n: usize = std::env::var("KW_BLEND_PERF_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1000);
    let count: usize = std::env::var("KW_BLEND_PERF_EDGES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(50);
    let mut rep = BRep::new();
    let b = zigzag(&mut rep, n);
    let faces = rep.body_faces(b).len();
    let all = vertical_edges(&rep, b);
    // pick sharp (outer-radius) edges, evenly spread
    let step = (all.len() / count).max(1);
    let chosen: Vec<EdgeId> = all.iter().step_by(step).take(count).copied().collect();
    let v0 = rep.volume(b).unwrap();
    let t = Instant::now();
    let r = fillet(&mut rep, b, &chosen, 0.01);
    let ms = t.elapsed().as_secs_f64() * 1e3;
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
        let parts: Vec<String> = names
            .iter()
            .zip(kw_boolean::PHASE_NANOS.iter())
            .map(|(n, c)| format!("{n} {:.0} ms", c.load(Relaxed) as f64 * 1e-6))
            .collect();
        println!("   boolean phases: {}", parts.join(", "));
    }
    match r {
        Ok(res) => {
            let v1 = rep.volume(res.body).unwrap();
            println!(
                "{faces} faces, {} edges filleted in {ms:.0} ms (volume change {:.3e})",
                chosen.len(),
                v1 - v0
            );
        }
        Err(e) => println!(
            "{faces} faces, {} edges: FAILED after {ms:.0} ms: {e}",
            chosen.len()
        ),
    }
}
