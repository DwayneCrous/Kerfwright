//! Verification helper for external cross-checks.
//!
//!   step_tool export <out_dir>      write a corpus of Kerfwright-made bodies (`NAME.step`) and `manifest.jsonl`
//!   step_tool import <file>...      import STEP files; one JSON object per line on stdout
//!
//! The JSON is consumed by `scripts/oracles/*.py`, which compare against OpenCASCADE.

use kw_blend::{chamfer, fillet};
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::{Curve3, CurveGeom};
use kw_step::{export, import, ExportOptions, ImportOptions};
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::fmt::Write as _;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn r(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
}

fn frame(r: &mut Rng) -> Frame3 {
    let n = Vector3::new(r.r(-1.0, 1.0), r.r(-1.0, 1.0), r.r(0.3, 1.0)).normalize();
    let mut f = Frame3::from_origin_normal(
        Point3::new(r.r(-3.0, 3.0), r.r(-3.0, 3.0), r.r(-3.0, 3.0)),
        n,
    );
    let a = r.r(0.0, TAU);
    let (c, s) = (a.cos(), a.sin());
    let (x, y) = (f.x * c + f.y * s, -f.x * s + f.y * c);
    f.x = x;
    f.y = y;
    f
}

fn vertical_edges(rep: &BRep, body: BodyId, up: Vector3<f64>) -> Vec<EdgeId> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if seen.insert(e) {
                    if let Some(CurveGeom::Line(l)) =
                        rep.edges[e].curve.and_then(|c| rep.curves.get(c))
                    {
                        if l.dir.normalize().cross(&up).norm() < 1e-9 {
                            out.push(e);
                        }
                    }
                }
            }
        }
    }
    out
}

fn build(kind: usize, seed: u64) -> Option<(String, BRep, BodyId)> {
    let mut r = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    for _ in 0..4 {
        r.next();
    }
    let mut rep = BRep::new();
    let f = frame(&mut r);
    let (name, body) = match kind {
        0 => (
            "box",
            make_box(&mut rep, &f, r.r(0.5, 4.0), r.r(0.5, 4.0), r.r(0.5, 4.0)).body,
        ),
        1 => (
            "cylinder",
            make_cylinder(&mut rep, &f, r.r(0.4, 2.0), r.r(0.5, 4.0)).body,
        ),
        2 => (
            "cone",
            make_cone(&mut rep, &f, r.r(0.8, 2.0), r.r(0.0, 0.7), r.r(0.8, 3.0)).body,
        ),
        3 => ("sphere", make_sphere(&mut rep, &f, r.r(0.4, 2.0)).body),
        4 => {
            let big = r.r(1.5, 3.0);
            ("torus", make_torus(&mut rep, &f, big, r.r(0.2, 0.8)).body)
        }
        5 => {
            let n = 5 + (r.next() * 5.0) as usize;
            let pts: Vec<(f64, f64)> = (0..n)
                .map(|i| {
                    let a = TAU * i as f64 / n as f64;
                    let rr = r.r(1.0, 2.0);
                    (rr * a.cos(), rr * a.sin())
                })
                .collect();
            let prof = Profile::polygon(f, &pts);
            (
                "prism",
                extrude(
                    &mut rep,
                    &prof,
                    f.z * r.r(0.5, 3.0),
                    &ExtrudeOptions::default(),
                )
                .ok()?
                .body,
            )
        }
        6 => {
            let hole = Frame3 {
                origin: f.origin + f.x * 0.2,
                x: f.x,
                y: -f.y,
                z: -f.z,
            };
            let prof = Profile::rectangle(f, r.r(3.0, 5.0), r.r(2.5, 4.0))
                .with_hole(ProfileLoop::circle(&hole, r.r(0.3, 0.9)));
            (
                "plate_with_hole",
                extrude(
                    &mut rep,
                    &prof,
                    f.z * r.r(0.3, 1.0),
                    &ExtrudeOptions::default(),
                )
                .ok()?
                .body,
            )
        }
        7 => {
            let fxz = Frame3 {
                origin: f.origin,
                x: f.x,
                y: f.z,
                z: -f.y,
            };
            let (r0, h0, r1, h1) = (r.r(1.0, 2.0), r.r(0.8, 2.0), r.r(0.3, 0.8), r.r(0.5, 1.5));
            let pts = [
                (0.0, 0.0),
                (r0, 0.0),
                (r0, h0),
                (r1, h0),
                (r1, h0 + h1),
                (0.0, h0 + h1),
            ];
            (
                "revolved_stepped",
                revolve(&mut rep, &Profile::polygon(fxz, &pts), f.origin, f.z, TAU)
                    .ok()?
                    .body,
            )
        }
        8 => {
            let a = make_box(&mut rep, &f, 3.0, 3.0, 3.0).body;
            let b = make_cylinder(
                &mut rep,
                &Frame3 {
                    origin: f.origin + f.x * r.r(-0.5, 0.5) - f.z * 2.0,
                    ..f
                },
                r.r(0.4, 0.9),
                4.0,
            )
            .body;
            (
                "box_minus_cylinder",
                boolean(&mut rep, a, b, Op::Subtract, &BoolOptions::default())
                    .ok()?
                    .body,
            )
        }
        9 => {
            let a = make_sphere(&mut rep, &f, r.r(1.2, 2.0)).body;
            let b = make_box(
                &mut rep,
                &Frame3 {
                    origin: f.origin + f.x * r.r(0.2, 1.0),
                    ..f
                },
                2.0,
                2.0,
                2.0,
            )
            .body;
            (
                "sphere_union_box",
                boolean(&mut rep, a, b, Op::Union, &BoolOptions::default())
                    .ok()?
                    .body,
            )
        }
        10 => {
            let b = make_box(&mut rep, &f, 4.0, 3.0, 2.0).body;
            let es = vertical_edges(&rep, b, f.z);
            (
                "filleted_box",
                fillet(&mut rep, b, &es[..2], r.r(0.2, 0.6)).ok()?.body,
            )
        }
        11 => {
            let b = make_box(&mut rep, &f, 4.0, 3.0, 2.0).body;
            let es = vertical_edges(&rep, b, f.z);
            (
                "chamfered_box",
                chamfer(&mut rep, b, &es[..3], r.r(0.2, 0.5), r.r(0.2, 0.5))
                    .ok()?
                    .body,
            )
        }
        12 => {
            let b = make_box(&mut rep, &f, 4.0, 3.0, 2.0).body;
            let top = rep.body_faces(b).into_iter().find(|fc| matches!(&rep.surfaces[rep.faces[*fc].surface], kw_nurbs::SurfaceGeom::Plane(p) if (p.frame.z * if rep.faces[*fc].sense == Sense::Forward { 1.0 } else { -1.0 }).dot(&f.z) > 0.99))?;
            (
                "shelled_box",
                shell(&mut rep, b, r.r(0.1, 0.4), &[top]).ok()?.body,
            )
        }
        13 => {
            let top = Frame3 {
                origin: f.origin + f.z * r.r(2.0, 4.0),
                ..f
            };
            (
                "lofted_rect",
                loft(
                    &mut rep,
                    &[
                        Profile::rectangle(f, 3.0, 2.0),
                        Profile::rectangle(top, r.r(1.0, 2.5), r.r(0.8, 1.8)),
                    ],
                )
                .ok()?
                .body,
            )
        }
        14 => {
            let arc = kw_nurbs::build::make_arc(
                &Frame3 {
                    origin: f.origin,
                    ..f
                },
                4.0,
                0.0,
                FRAC_PI_2,
            )
            .ok()?;
            let pf = Frame3 {
                origin: f.origin + f.x * 4.0,
                x: f.z,
                y: f.x,
                z: f.y,
            };
            (
                "swept_bend",
                sweep(
                    &mut rep,
                    &Profile::disc(pf, r.r(0.3, 0.7)),
                    &CurveGeom::Nurbs(arc),
                    &SweepOptions::default(),
                )
                .ok()?
                .body,
            )
        }
        15 => {
            let b = make_cylinder(&mut rep, &f, r.r(1.0, 2.0), r.r(1.0, 3.0)).body;
            (
                "offset_cylinder",
                offset_body(&mut rep, b, r.r(0.1, 0.4)).ok()?.body,
            )
        }
        16 => {
            let b = make_box(
                &mut rep,
                &Frame3 {
                    origin: f.origin + f.z * 1.0,
                    ..f
                },
                4.0,
                3.0,
                2.0,
            )
            .body;
            let sides: Vec<FaceId> = rep.body_faces(b).into_iter().filter(|fc| matches!(&rep.surfaces[rep.faces[*fc].surface], kw_nurbs::SurfaceGeom::Plane(p) if p.frame.z.dot(&f.z).abs() < 0.5)).collect();
            (
                "drafted_box",
                draft_faces(&mut rep, b, &sides, f.origin, f.z, r.r(0.02, 0.1))
                    .ok()?
                    .body,
            )
        }
        _ => return None,
    };
    Some((name.to_string(), rep, body))
}

fn bbox(rep: &BRep, body: BodyId) -> ([f64; 3], [f64; 3]) {
    let b = rep.body_bbox(body);
    ([b.min.x, b.min.y, b.min.z], [b.max.x, b.max.y, b.max.z])
}

fn counts(rep: &BRep, b: BodyId) -> (usize, usize, usize) {
    let faces = rep.body_faces(b);
    let mut edges = std::collections::HashSet::new();
    let mut verts = std::collections::HashSet::new();
    for f in &faces {
        for lp in &rep.faces[*f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if edges.insert(e) && rep.edges[e].curve.is_some() {
                    verts.insert(rep.edges[e].start);
                    verts.insert(rep.edges[e].end);
                }
            }
        }
    }
    (
        faces.len(),
        edges
            .iter()
            .filter(|e| rep.edges[**e].curve.is_some())
            .count(),
        verts.len(),
    )
}

fn json_body(rep: &BRep, b: BodyId) -> String {
    let m = rep.mass_properties(b).unwrap();
    let (lo, hi) = bbox(rep, b);
    let (f, e, v) = counts(rep, b);
    format!(
        "\"volume\":{:e},\"area\":{:e},\"centroid\":[{:e},{:e},{:e}],\"bbox_min\":[{:e},{:e},{:e}],\"bbox_max\":[{:e},{:e},{:e}],\"faces\":{f},\"edges\":{e},\"vertices\":{v}",
        m.volume, m.area, m.centroid.x, m.centroid.y, m.centroid.z, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]
    )
}

fn esc(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', " ")
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("export") => {
            let dir = std::path::PathBuf::from(args.get(2).expect("usage: step_tool export <dir>"));
            std::fs::create_dir_all(&dir).unwrap();
            let per_kind: u64 = std::env::var("KW_CORPUS_PER_KIND")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(8);
            let mut manifest = String::new();
            let mut written = 0;
            for kind in 0..17 {
                let mut ok = 0;
                let mut seed = 1;
                while ok < per_kind && seed < 200 {
                    if let Some((name, rep, body)) = build(kind, seed + 1000 * kind as u64) {
                        if rep.validate().is_ok() {
                            let file = format!("{name}_{seed:03}.step");
                            let text = export(
                                &rep,
                                &[body],
                                &ExportOptions {
                                    product_name: file.clone(),
                                    ..Default::default()
                                },
                            );
                            std::fs::write(dir.join(&file), text).unwrap();
                            let _ = writeln!(
                                manifest,
                                "{{\"file\":\"{file}\",{}}}",
                                json_body(&rep, body)
                            );
                            ok += 1;
                            written += 1;
                        }
                    }
                    seed += 1;
                }
            }
            std::fs::write(dir.join("manifest.jsonl"), manifest).unwrap();
            eprintln!("wrote {written} STEP files to {}", dir.display());
        }
        Some("import") => {
            for file in &args[2..] {
                let t = std::time::Instant::now();
                let text = match std::fs::read(file) {
                    Ok(b) => String::from_utf8_lossy(&b).into_owned(),
                    Err(e) => {
                        println!(
                            "{{\"file\":\"{}\",\"status\":\"io-error\",\"detail\":\"{}\"}}",
                            esc(file),
                            esc(&e.to_string())
                        );
                        continue;
                    }
                };
                let mut rep = BRep::new();
                let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    import(&mut rep, &text, &ImportOptions::default())
                }));
                let ms = t.elapsed().as_secs_f64() * 1e3;
                match res {
                    Err(_) => println!(
                        "{{\"file\":\"{}\",\"status\":\"PANIC\",\"ms\":{ms:.1}}}",
                        esc(file)
                    ),
                    Ok(Ok(r)) => {
                        let bodies: Vec<String> = r
                            .bodies
                            .iter()
                            .map(|b| format!("{{{}}}", json_body(&rep, *b)))
                            .collect();
                        let failed: Vec<String> = r
                            .failed
                            .iter()
                            .map(|(i, why)| format!("\"#{i}: {}\"", esc(why)))
                            .collect();
                        println!(
                            "{{\"file\":\"{}\",\"status\":\"ok\",\"ms\":{ms:.1},\"solids_found\":{},\"unit_scale\":{},\"bodies\":[{}],\"failed\":[{}]}}",
                            esc(file),
                            r.solids_found,
                            r.unit_scale,
                            bodies.join(","),
                            failed.join(",")
                        );
                    }
                    Ok(Err(e)) => println!(
                        "{{\"file\":\"{}\",\"status\":\"error\",\"ms\":{ms:.1},\"detail\":\"{}\"}}",
                        esc(file),
                        esc(&format!("{e}").chars().take(600).collect::<String>())
                    ),
                }
            }
        }
        _ => eprintln!("usage: step_tool export <dir> | import <file>..."),
    }
    let _ = (
        PI,
        Curve3::derivative(
            &CurveGeom::Line(kw_nurbs::Line {
                origin: Point3::origin(),
                dir: Vector3::x(),
            }),
            0.0,
        ),
    );
}
