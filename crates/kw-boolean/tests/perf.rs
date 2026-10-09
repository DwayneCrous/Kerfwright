#![allow(unused_imports, dead_code)]
//! Boolean performance on large prismatic bodies (release only).
//!
//! A "gear-like" prism with `n` side faces is extruded; the second operand is the same shape rotated
//! and shifted so the outlines cross each other many times. Reports wall time per phase.

use kw_boolean::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Transform3, Vector3};
use kw_model::*;
use std::f64::consts::TAU;
use std::time::Instant;

fn polygon_body(rep: &mut BRep, n: usize, shift: f64) -> BodyId {
    let pts: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let a = TAU * i as f64 / n as f64;
            (10.0 * a.cos() + shift, 10.0 * a.sin())
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
    .expect("polygon")
    .body
}

fn gear(rep: &mut BRep, n: usize, phase: f64, shift: f64) -> BodyId {
    let pts: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let a = TAU * i as f64 / n as f64;
            // radius oscillates so that every second pair of vertices is "in" / "out"
            let r =
                10.0 + 0.8 * (if i % 2 == 0 { 1.0 } else { -1.0 }) + 0.3 * (a * 3.0 + phase).sin();
            (
                r * (a + phase * 0.01).cos() + shift,
                r * (a + phase * 0.01).sin(),
            )
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
    .expect("gear")
    .body
}

#[test]
fn boolean_scaling() {
    if cfg!(debug_assertions) {
        return;
    }
    let sizes: Vec<usize> = std::env::var("KW_PERF_SIZES")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_else(|| vec![100, 400, 1600]);
    for n in sizes {
        let mut rep = BRep::new();
        let t0 = Instant::now();
        let local = std::env::var("KW_PERF_LOCAL").is_ok();
        let a = if local {
            polygon_body(&mut rep, n, 0.0)
        } else {
            gear(&mut rep, n, 0.0, 0.0)
        };
        let b = if local {
            polygon_body(&mut rep, n, 10.3)
        } else {
            gear(&mut rep, n, 0.37, 0.4)
        };
        let build = t0.elapsed();
        let faces = rep.body_faces(a).len() + rep.body_faces(b).len();
        let mut times = Vec::new();
        for op in [Op::Union, Op::Intersect, Op::Subtract] {
            let t = Instant::now();
            let r = boolean(
                &mut rep,
                a,
                b,
                op,
                &BoolOptions {
                    verify: true,
                    ..Default::default()
                },
            );
            times.push((
                op,
                t.elapsed().as_secs_f64() * 1e3,
                r.map(|r| rep.body_faces(r.body).len())
                    .map_err(|e| format!("{e}")),
            ));
        }
        println!(
            "n={n} ({faces} faces total, built in {:.0} ms):",
            build.as_secs_f64() * 1e3
        );
        for (op, ms, r) in times {
            println!("   {op:?}: {ms:.0} ms -> {r:?}");
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
            let parts: Vec<String> = names
                .iter()
                .zip(PHASE_NANOS.iter())
                .map(|(n, c)| format!("{n} {:.0} ms", c.load(Relaxed) as f64 * 1e-6 / 3.0))
                .collect();
            println!("   phases (mean per op): {}", parts.join(", "));
            for c in PHASE_NANOS.iter() {
                c.store(0, Relaxed);
            }
        }
    }
}
