//! A closed triangle mesh as a faceted solid: planar faces with straight edges.
//!
//! With `merge_coplanar` the triangles that lie in one plane (all corners within a small distance of the
//! plane of the first one) become one face, and the vertices a straight edge passes through are dropped, so
//! that a mesh made from a CAD part with flat faces comes back with the faces it had. Curved parts stay
//! faceted: nothing here fits cylinders or spheres.

use kw_brep::sew::{BuildOptions, EdgeDesc, FaceDesc, LoopDesc, Soup, UseDesc};
use kw_brep::{BRep, BodyId, LoopKind, Origin, Sense};
use kw_math::{Frame3, Point3, Vector3};
use kw_mesh3d::SurfaceMesh;
use kw_nurbs::{CurveGeom, Line, Plane, SurfaceGeom};
use std::collections::HashMap;

/// The most triangles a conversion accepts. Building the B-rep costs a few microseconds per triangle plus
/// the cost of validating it; past this the app would stall, and a mesh this fine is a scan, not a part.
pub const MAX_CONVERT_TRIANGLES: usize = 150_000;

type P3 = [f64; 3];

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}
fn point(p: P3) -> Point3<f64> {
    Point3::new(p[0], p[1], p[2])
}

/// A summary of what a conversion made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConvertReport {
    pub regions: usize,
    pub edges: usize,
}

/// Build the faceted solid of `mesh` into `rep`. The mesh must be closed and wound outward (see
/// [`SurfaceMesh::orient`]); several separate pieces become several lumps joined by the caller.
pub fn mesh_to_bodies(
    rep: &mut BRep,
    mesh: &SurfaceMesh,
    merge_coplanar: bool,
) -> Result<(Vec<BodyId>, ConvertReport), String> {
    if mesh.tris.len() > MAX_CONVERT_TRIANGLES {
        return Err(format!(
            "the mesh has {} triangles; converting more than {MAX_CONVERT_TRIANGLES} is not supported",
            mesh.tris.len()
        ));
    }
    let st = mesh.stats();
    if st.triangles == 0 {
        return Err("the mesh has no triangles".into());
    }
    if !st.watertight {
        return Err(format!(
            "the mesh is not closed ({} open edges, {} edges shared by more than two triangles, {} \
             inconsistently turned): it cannot become a solid",
            st.open_edges, st.non_manifold_edges, st.inconsistent_edges
        ));
    }
    let (lo, hi) = mesh.bounds();
    let diag = norm(sub(hi, lo)).max(1e-9);
    let ptol = 2e-6 * diag;
    let nodes = &mesh.nodes;
    let ntri = mesh.tris.len();

    // directed edge → triangle
    let mut he: HashMap<(u32, u32), u32> = HashMap::with_capacity(ntri * 3);
    for (k, t) in mesh.tris.iter().enumerate() {
        for i in 0..3 {
            he.insert((t[i], t[(i + 1) % 3]), k as u32);
        }
    }
    let normal_of = |k: usize| {
        let t = mesh.tris[k];
        let (a, b, c) = (
            nodes[t[0] as usize],
            nodes[t[1] as usize],
            nodes[t[2] as usize],
        );
        let n = cross(sub(b, a), sub(c, a));
        let l = norm(n);
        ([n[0] / l, n[1] / l, n[2] / l], l * 0.5)
    };

    // regions of coplanar triangles, grown from the lowest-numbered triangle left
    let mut region = vec![u32::MAX; ntri];
    let mut planes: Vec<(P3, P3)> = Vec::new(); // origin, normal
    for seed in 0..ntri {
        if region[seed] != u32::MAX {
            continue;
        }
        let id = planes.len() as u32;
        let (n0, _) = normal_of(seed);
        let t = mesh.tris[seed];
        let (a, b, c) = (
            nodes[t[0] as usize],
            nodes[t[1] as usize],
            nodes[t[2] as usize],
        );
        let o = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        planes.push((o, n0));
        region[seed] = id;
        if !merge_coplanar {
            continue;
        }
        let mut queue = vec![seed as u32];
        while let Some(k) = queue.pop() {
            let t = mesh.tris[k as usize];
            for i in 0..3 {
                let Some(&o_tri) = he.get(&(t[(i + 1) % 3], t[i])) else {
                    continue;
                };
                if region[o_tri as usize] != u32::MAX {
                    continue;
                }
                let u = mesh.tris[o_tri as usize];
                let (n1, _) = normal_of(o_tri as usize);
                if dot(n1, n0) < 0.9 {
                    continue;
                }
                let flat = u
                    .iter()
                    .all(|&v| dot(sub(nodes[v as usize], o), n0).abs() <= ptol);
                if flat {
                    region[o_tri as usize] = id;
                    queue.push(o_tri);
                }
            }
        }
    }

    // edges between regions: which vertices they join
    let mut feature: HashMap<u32, Vec<u32>> = HashMap::new();
    for (k, t) in mesh.tris.iter().enumerate() {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            let other = he[&(b, a)];
            if region[other as usize] != region[k] && a < b {
                feature.entry(a).or_default().push(b);
                feature.entry(b).or_default().push(a);
            }
        }
    }
    for list in feature.values_mut() {
        list.sort_unstable();
    }
    // a vertex that a straight edge passes through is not a corner
    let is_corner = |v: u32| -> bool {
        let Some(nb) = feature.get(&v) else {
            return false;
        };
        if nb.len() != 2 {
            return true;
        }
        let (p, a, b) = (
            nodes[v as usize],
            nodes[nb[0] as usize],
            nodes[nb[1] as usize],
        );
        let (d1, d2) = (sub(p, a), sub(b, p));
        let chord = sub(b, a);
        let cl = norm(chord);
        if cl <= 0.0 || dot(d1, d2) <= 0.0 {
            return true;
        }
        norm(cross(sub(p, a), chord)) / cl > ptol
    };
    let mut corner: HashMap<u32, bool> = HashMap::new();
    for &v in feature.keys() {
        corner.insert(v, is_corner(v));
    }
    // chains of mesh edges between corners, each one an edge of the solid
    let mut chain_of: HashMap<(u32, u32), usize> = HashMap::new();
    let mut chains: Vec<(u32, u32)> = Vec::new();
    let mut keys: Vec<u32> = feature.keys().copied().collect();
    keys.sort_unstable();
    let edge_key = |a: u32, b: u32| (a.min(b), a.max(b));
    for &start in &keys {
        if !corner[&start] {
            continue;
        }
        for &first in &feature[&start] {
            if chain_of.contains_key(&edge_key(start, first)) {
                continue;
            }
            let id = chains.len();
            let (mut prev, mut cur) = (start, first);
            chain_of.insert(edge_key(prev, cur), id);
            while !corner[&cur] {
                let nb = &feature[&cur];
                let next = if nb[0] == prev { nb[1] } else { nb[0] };
                prev = cur;
                cur = next;
                chain_of.insert(edge_key(prev, cur), id);
            }
            chains.push((start, cur));
        }
    }
    if chains.is_empty() {
        return Err("the mesh has no edges between its faces (a single flat region)".into());
    }

    // the loops around each region: from a boundary half-edge to the next one around its end vertex
    let third = |k: u32, a: u32, b: u32| -> u32 {
        let t = mesh.tris[k as usize];
        *t.iter().find(|&&v| v != a && v != b).unwrap()
    };
    let next_boundary = |a: u32, b: u32| -> (u32, u32) {
        let (mut tri, mut e) = (he[&(a, b)], (a, b));
        loop {
            let c = third(tri, e.0, e.1);
            let t2 = he[&(c, e.1)];
            if region[t2 as usize] != region[tri as usize] {
                return (e.1, c);
            }
            tri = t2;
            e = (c, e.1);
        }
    };
    let mut loops_of: Vec<Vec<Vec<u32>>> = vec![Vec::new(); planes.len()];
    let mut seen: HashMap<(u32, u32), ()> = HashMap::new();
    for (k, t) in mesh.tris.iter().enumerate() {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            let other = he[&(b, a)];
            if region[other as usize] == region[k] || seen.contains_key(&(a, b)) {
                continue;
            }
            let mut ring = vec![a];
            seen.insert((a, b), ());
            let (mut x, mut y) = (a, b);
            loop {
                let (nx, ny) = next_boundary(x, y);
                if (nx, ny) == (a, b) {
                    break;
                }
                if seen.insert((nx, ny), ()).is_some() {
                    return Err("the boundary of a face does not close".into());
                }
                ring.push(nx);
                (x, y) = (nx, ny);
            }
            loops_of[region[k] as usize].push(ring);
        }
    }

    // the soup
    let mut soup = Soup::default();
    for &(a, b) in &chains {
        let (pa, pb) = (point(nodes[a as usize]), point(nodes[b as usize]));
        soup.edges.push(EdgeDesc {
            curve: Some(CurveGeom::Line(Line {
                origin: pa,
                dir: pb - pa,
            })),
            range: (0.0, 1.0),
            start: pa,
            end: pb,
            tolerance: ptol,
        });
    }
    for (r, rings) in loops_of.iter().enumerate() {
        let (o, n) = planes[r];
        let frame = Frame3::from_origin_normal(point(o), Vector3::new(n[0], n[1], n[2]));
        let uv = |p: P3| -> (f64, f64) {
            let d = Vector3::new(p[0] - o[0], p[1] - o[1], p[2] - o[2]);
            (d.dot(&frame.x), d.dot(&frame.y))
        };
        // the loop of corners, as uses of the chains
        let mut built: Vec<(Vec<UseDesc>, f64, Vec<(f64, f64)>)> = Vec::new();
        for ring in rings {
            let corners: Vec<usize> = (0..ring.len())
                .filter(|&i| corner.get(&ring[i]).copied().unwrap_or(false))
                .collect();
            if corners.len() < 3 {
                return Err("a face of the mesh has fewer than three corners".into());
            }
            let mut uses = Vec::new();
            let mut poly = Vec::new();
            for (j, &i) in corners.iter().enumerate() {
                let from = ring[i];
                let after = ring[(i + 1) % ring.len()];
                let to = ring[corners[(j + 1) % corners.len()]];
                let id = chain_of[&edge_key(from, after)];
                let (c0, c1) = chains[id];
                let reversed = from == c1 && to == c0;
                if !(reversed || (from == c0 && to == c1)) {
                    return Err("an edge of the mesh joins the wrong corners".into());
                }
                let (u0, v0) = uv(nodes[c0 as usize]);
                let (u1, v1) = uv(nodes[c1 as usize]);
                uses.push(UseDesc {
                    edge: id,
                    reversed,
                    pcurve: Some(CurveGeom::Line(Line {
                        origin: Point3::new(u0, v0, 0.0),
                        dir: Vector3::new(u1 - u0, v1 - v0, 0.0),
                    })),
                    hint: None,
                });
                poly.push(uv(nodes[from as usize]));
            }
            let mut area = 0.0;
            for i in 0..poly.len() {
                let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
                area += p.0 * q.1 - q.0 * p.1;
            }
            built.push((uses, 0.5 * area, poly));
        }
        // outer loops run counter-clockwise about the normal, holes clockwise
        let outers: Vec<usize> = (0..built.len()).filter(|&i| built[i].1 > 0.0).collect();
        if outers.is_empty() {
            return Err("a face of the mesh has no outer boundary".into());
        }
        let inside = |poly: &[(f64, f64)], p: (f64, f64)| -> bool {
            let mut c = false;
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0
                {
                    c = !c;
                }
            }
            c
        };
        let mut faces: Vec<Vec<LoopDesc>> = Vec::new();
        for &o_i in &outers {
            faces.push(vec![LoopDesc {
                kind: LoopKind::Outer,
                uses: std::mem::take(&mut built[o_i].0),
            }]);
        }
        for h in 0..built.len() {
            if built[h].1 > 0.0 {
                continue;
            }
            let probe = built[h].2[0];
            let host = outers
                .iter()
                .enumerate()
                .filter(|(_, &o_i)| inside(&built[o_i].2, probe))
                .min_by(|a, b| built[*a.1].1.total_cmp(&built[*b.1].1))
                .map(|(i, _)| i)
                .ok_or("a hole of a face lies outside the face")?;
            faces[host].push(LoopDesc {
                kind: LoopKind::Inner,
                uses: std::mem::take(&mut built[h].0),
            });
        }
        for loops in faces {
            soup.faces.push(FaceDesc {
                surface: SurfaceGeom::Plane(Plane { frame }),
                sense: Sense::Forward,
                loops,
                origin: Origin::New {
                    role: format!("region-{r}").into(),
                },
            });
        }
    }
    let report = ConvertReport {
        regions: soup.faces.len(),
        edges: soup.edges.len(),
    };
    let res = soup.build(
        rep,
        &BuildOptions {
            vertex_tol: 1e-9,
            one_body: false,
        },
    );
    if res.bodies.is_empty() {
        return Err(format!(
            "the faceted solid could not be built: {}",
            res.rejected.join("; ")
        ));
    }
    if !res.rejected.is_empty() {
        return Err(format!(
            "part of the mesh could not be built: {}",
            res.rejected.join("; ")
        ));
    }
    for b in &res.bodies {
        if rep.bodies.get(*b).map(|b| b.kind) != Some(kw_brep::BodyKind::Solid) {
            return Err("the faceted shape is not a closed solid".into());
        }
    }
    Ok((res.bodies, report))
}
