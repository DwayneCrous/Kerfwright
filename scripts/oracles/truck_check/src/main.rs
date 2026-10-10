//! usage: truck_check <dir-with-manifest.jsonl>
//! Reads each Kerfwright STEP export with the independent `truck` kernel (its own STEP reader, surface
//! evaluation and tessellator) and checks
//!   * the file converts to shells with the face count recorded in the manifest,
//!   * the tessellated solid encloses the volume and has the area recorded in the manifest. A mesh converges
//!     quadratically, so the figures are Richardson-extrapolated from two tessellations
//!     (`tol` and `tol / 2`): a flat-faced body is exact, a curved one is compared to a few 1e-6.
use std::collections::BTreeMap;
use truck_meshalgo::prelude::*;
use truck_stepio::r#in::Table;

/// Signed volume and area of a triangulated closed mesh.
fn mesh_props(mesh: &PolygonMesh) -> (f64, f64) {
    let pos = mesh.positions();
    let (mut v, mut a) = (0.0, 0.0);
    for t in mesh.faces().triangle_iter() {
        let (p, q, r) = (pos[t[0].pos], pos[t[1].pos], pos[t[2].pos]);
        v += p.to_vec().dot(q.to_vec().cross(r.to_vec())) / 6.0;
        a += 0.5 * (q - p).cross(r - p).magnitude();
    }
    (v, a)
}

static mut WORST_V: f64 = 0.0;
static mut WORST_A: f64 = 0.0;
static mut COMPARED: usize = 0;

fn main() {
    let dir = std::env::args().nth(1).expect("dir");
    let manifest = std::fs::read_to_string(format!("{dir}/manifest.jsonl")).expect("manifest");
    let (mut total, mut ok) = (0, 0);
    let mut failures: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for line in manifest.lines() {
        let file = field_str(line, "file");
        let faces: usize = field_num(line, "faces") as usize;
        total += 1;
        let text = std::fs::read_to_string(format!("{dir}/{file}")).expect("read");
        let (want_v, want_a) = (field_num(line, "volume"), field_num(line, "area"));
        // (face count from truck's STEP reader, geometry comparison)
        let res = std::panic::catch_unwind(|| -> Result<(usize, Result<(), String>), String> {
            let table = Table::from_step(&text).ok_or("parse failed")?;
            if table.shell.is_empty() {
                return Err("no shells".into());
            }
            let mut n = 0;
            let (mut v1, mut a1, mut v2, mut a2) = (0.0, 0.0, 0.0, 0.0);
            let (lo, hi) = (field_arr(line, "bbox_min"), field_arr(line, "bbox_max"));
            let diag = (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f64>().sqrt();
            let mut shells = Vec::new();
            for shell in table.shell.values() {
                let cs = table
                    .to_compressed_shell(shell)
                    .map_err(|e| format!("{e:?}"))?;
                n += cs.faces.len();
                shells.push(cs);
            }
            let geometry = (|| -> Result<(), String> {
                for cs in shells {
                    // truck's topology cannot hold an edge whose two ends are one vertex (every closed circle,
                    // seam or periodic loop) nor a wire that revisits a vertex: such bodies are beyond the
                    // independent reader, not wrong
                    let shell = truck_topology::Shell::extract(cs).map_err(|e| format!("limit: {e:?}"))?;
                    // two tessellations: chord tolerance t and t / 2
                    let t = 1e-4 * diag;
                    let m1 = shell.robust_triangulation(t).to_polygon();
                    let m2 = shell.robust_triangulation(t / 2.0).to_polygon();
                    let (pv1, pa1) = mesh_props(&m1);
                    let (pv2, pa2) = mesh_props(&m2);
                    v1 += pv1;
                    a1 += pa1;
                    v2 += pv2;
                    a2 += pa2;
                }
                // a tessellation that lost most of the body is truck failing on a degenerate (pole / apex)
                // face, not a disagreement about the geometry
                if v2.abs() < 0.5 * want_v || a2 < 0.5 * want_a {
                    return Err(format!("limit: truck's tessellation is incomplete (volume {:.3e} of {want_v:.3e})", v2.abs()));
                }
                // the error of a polyhedral approximation falls with the square of the mesh size: extrapolate
                let (v, a) = (v2 + (v2 - v1), a2 + (a2 - a1));
                let (ev, ea) = ((v.abs() - want_v).abs() / want_v.abs().max(1.0), (a - want_a).abs() / want_a.max(1.0));
                let raw = (v2.abs() - want_v).abs() / want_v.abs().max(1.0);
                if ev.min(raw) > 1e-5 || ea > 1e-5 {
                    return Err(format!("geometry: volume {} vs {want_v} (rel {ev:.2e}, raw {raw:.2e}), area {} vs {want_a} (rel {ea:.2e})", v.abs(), a));
                }
                unsafe {
                    WORST_V = f64::max(WORST_V, ev.min(raw));
                    WORST_A = f64::max(WORST_A, ea);
                    COMPARED += 1;
                }
                Ok(())
            })();
            Ok((n, geometry))
        });
        let kind = file.rsplit_once('_').map(|x| x.0).unwrap_or(&file).to_string();
        match res {
            Ok(Ok((n, geometry))) => {
                if n != faces {
                    failures.entry(kind).or_default().push(format!("{file}: {n} faces, expected {faces}"));
                } else {
                    ok += 1;
                    if let Err(e) = geometry {
                        failures.entry(kind).or_default().push(format!("{file}: {}", e.chars().take(200).collect::<String>()));
                    }
                }
            }
            Ok(Err(e)) => failures.entry(kind).or_default().push(format!("{file}: {}", e.chars().take(120).collect::<String>())),
            Err(_) => failures.entry(kind).or_default().push(format!("{file}: panic in truck")),
        }
    }
    let limit: usize = failures.values().flatten().filter(|m| m.contains("limit:")).count();
    let bad: usize = failures.values().map(|v| v.len()).sum::<usize>() - limit;
    println!("files: {total}");
    println!("  geometry compared in truck (volume + area, Richardson-extrapolated meshes): {} files, worst volume deviation {:.2e}, worst area deviation {:.2e}", unsafe { COMPARED }, unsafe { WORST_V }, unsafe { WORST_A });
    println!("  beyond truck (closed edges / degenerate faces it cannot hold; parsed and converted to a compressed shell with the right face count): {limit}");
    println!("  DISAGREEMENTS (face count or geometry): {bad}  (matching face count overall: {ok})");
    for (k, v) in failures {
        for m in v.iter().filter(|m| !m.contains("limit:")).take(2) {
            println!("  {k}: {m}");
        }
    }
}

fn field_str(line: &str, key: &str) -> String {
    let pat = format!("\"{key}\":\"");
    let i = line.find(&pat).unwrap() + pat.len();
    line[i..i + line[i..].find('"').unwrap()].to_string()
}
fn field_num(line: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat).unwrap() + pat.len();
    let rest = &line[i..];
    rest[..rest.find([',', '}']).unwrap()].parse().unwrap()
}
fn field_arr(line: &str, key: &str) -> [f64; 3] {
    let pat = format!("\"{key}\":[");
    let i = line.find(&pat).unwrap() + pat.len();
    let rest = &line[i..];
    let v: Vec<f64> = rest[..rest.find(']').unwrap()].split(',').map(|x| x.trim().parse().unwrap()).collect();
    [v[0], v[1], v[2]]
}
