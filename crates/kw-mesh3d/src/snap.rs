//! Moving boundary vertices of a finished tet mesh onto the true geometry.

use crate::smooth::mean_ratio;
use crate::{SurfaceTri, P3};
use std::collections::HashMap;

/// Move boundary vertices that lie on one face (tag) with `project`, and those on a sharp edge between two
/// faces with `project_edge`. A move is made only if every tet round the vertex keeps a positive volume
/// and a mean-ratio quality of at least half of what it had (but never more than 0.1 is asked for). Returns the number of
/// vertices moved.
pub fn snap_boundary(
    nodes: &mut [P3],
    tets: &[[u32; 4]],
    surface: &[SurfaceTri],
    project: Option<&(dyn Fn(u32, P3) -> P3 + Send + Sync)>,
    project_edge: Option<&(dyn Fn(u32, u32, P3) -> P3 + Send + Sync)>,
) -> usize {
    if project.is_none() && project_edge.is_none() {
        return 0;
    }
    let mut tags: HashMap<u32, Vec<u32>> = HashMap::new();
    for s in surface {
        for &v in &s.v {
            let t = tags.entry(v).or_default();
            if !t.contains(&s.tag) {
                t.push(s.tag);
            }
        }
    }
    // tets round each vertex
    let n = nodes.len();
    let mut star: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (k, t) in tets.iter().enumerate() {
        for &v in t {
            if tags.contains_key(&v) {
                star[v as usize].push(k as u32);
            }
        }
    }
    let mut ids: Vec<u32> = tags.keys().copied().collect();
    ids.sort_unstable();
    let mut moved = 0;
    for v in ids {
        let mut tg = tags[&v].clone();
        tg.sort_unstable();
        let old = nodes[v as usize];
        let new = match (tg.len(), project, project_edge) {
            (1, Some(p), _) => p(tg[0], old),
            (2, _, Some(pe)) => pe(tg[0], tg[1], old),
            _ => continue,
        };
        if !new.iter().all(|x| x.is_finite()) {
            continue;
        }
        let before = star[v as usize]
            .iter()
            .map(|&k| mean_ratio(nodes, &tets[k as usize]))
            .fold(f64::INFINITY, f64::min);
        nodes[v as usize] = new;
        let after = star[v as usize]
            .iter()
            .map(|&k| mean_ratio(nodes, &tets[k as usize]))
            .fold(f64::INFINITY, f64::min);
        if after > 0.0 && after >= 0.5 * before.min(0.2) {
            moved += 1;
        } else {
            nodes[v as usize] = old;
        }
    }
    moved
}
