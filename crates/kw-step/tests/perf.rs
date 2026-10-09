#![allow(unused_imports, dead_code)]
//! STEP import throughput on a large synthetic file (release only).

use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_step::*;
use std::f64::consts::TAU;
use std::time::Instant;

#[test]
fn large_file_import_time() {
    if cfg!(debug_assertions) {
        return;
    }
    let sides: usize = std::env::var("KW_STEP_PERF_SIDES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(5000);
    let mut rep = BRep::new();
    let pts: Vec<(f64, f64)> = (0..sides)
        .map(|i| {
            let a = TAU * i as f64 / sides as f64;
            let r = 10.0 + if i % 2 == 0 { 0.0 } else { 0.3 };
            (r * a.cos(), r * a.sin())
        })
        .collect();
    let f = Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    };
    let b = extrude(
        &mut rep,
        &Profile::polygon(f, &pts),
        Vector3::new(0.0, 0.0, 2.0),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let t = Instant::now();
    let text = export(&rep, &[b], &ExportOptions::default());
    let t_export = t.elapsed();
    let mb = text.len() as f64 / 1e6;
    let t = Instant::now();
    let parsed = parse(&text).unwrap();
    let t_parse = t.elapsed();
    let mut back = BRep::new();
    let t = Instant::now();
    let r = import(&mut back, &text, &ImportOptions::default()).unwrap();
    let t_import = t.elapsed();
    println!(
        "{sides} sides: {mb:.1} MB, {} instances; export {:.0} ms, parse only {:.0} ms ({:.1} MB/s), full import {:.0} ms ({:.1} MB/s) -> {} bodies, {} failed",
        parsed.instances.len(),
        t_export.as_secs_f64() * 1e3,
        t_parse.as_secs_f64() * 1e3,
        mb / t_parse.as_secs_f64(),
        t_import.as_secs_f64() * 1e3,
        mb / t_import.as_secs_f64(),
        r.bodies.len(),
        r.failed.len()
    );
}
