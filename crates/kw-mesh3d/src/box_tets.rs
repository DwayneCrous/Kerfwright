use crate::mesh::{extract_surface, TetMesh};
use crate::{signed_volume, P3};

/// A structured tetrahedral mesh of the box `[0,lx]×[0,ly]×[0,lz]`: every cell of an `nx×ny×nz` grid is
/// cut into 6 tets along its main diagonal (Kuhn subdivision, conforming across cells). Surface triangles
/// are tagged by box side: 0 = x min, 1 = x max, 2 = y min, 3 = y max, 4 = z min, 5 = z max.
pub fn box_tets(lx: f64, ly: f64, lz: f64, nx: usize, ny: usize, nz: usize) -> TetMesh {
    let id = |i: usize, j: usize, k: usize| (i + (nx + 1) * (j + (ny + 1) * k)) as u32;
    let mut nodes: Vec<P3> = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                nodes.push([
                    lx * i as f64 / nx as f64,
                    ly * j as f64 / ny as f64,
                    lz * k as f64 / nz as f64,
                ]);
            }
        }
    }
    let perms = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut tets = Vec::with_capacity(6 * nx * ny * nz);
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                for p in perms {
                    let mut c = [i, j, k];
                    let a = id(c[0], c[1], c[2]);
                    c[p[0]] += 1;
                    let b = id(c[0], c[1], c[2]);
                    c[p[1]] += 1;
                    let cc = id(c[0], c[1], c[2]);
                    c[p[2]] += 1;
                    let d = id(c[0], c[1], c[2]);
                    let mut t = [a, b, cc, d];
                    if signed_volume(&nodes, &t) < 0.0 {
                        t.swap(2, 3);
                    }
                    tets.push(t);
                }
            }
        }
    }
    let (ex, ey, ez) = (lx * 1e-9, ly * 1e-9, lz * 1e-9);
    let tag = |a: P3, b: P3, c: P3| -> u32 {
        let on = |axis: usize, v: f64, eps: f64| {
            (a[axis] - v).abs() < eps && (b[axis] - v).abs() < eps && (c[axis] - v).abs() < eps
        };
        if on(0, 0.0, ex) {
            0
        } else if on(0, lx, ex) {
            1
        } else if on(1, 0.0, ey) {
            2
        } else if on(1, ly, ey) {
            3
        } else if on(2, 0.0, ez) {
            4
        } else {
            5
        }
    };
    let surface = extract_surface(&nodes, &tets, &tag);
    TetMesh {
        nodes,
        tets,
        surface,
    }
}

/// Split every tet into 8 (red refinement: corners cut off plus the inner octahedron along its shortest
/// diagonal). Surface triangles split into 4 and keep their tag. Used to build big benchmark meshes from
/// small ones.
pub fn split_hexes_red(m: &TetMesh) -> TetMesh {
    use std::collections::HashMap;
    let mut nodes = m.nodes.clone();
    let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
    let mut midpoint = |a: u32, b: u32, nodes: &mut Vec<P3>| -> u32 {
        *mid.entry((a.min(b), a.max(b))).or_insert_with(|| {
            let (p, q) = (nodes[a as usize], nodes[b as usize]);
            nodes.push([
                0.5 * (p[0] + q[0]),
                0.5 * (p[1] + q[1]),
                0.5 * (p[2] + q[2]),
            ]);
            (nodes.len() - 1) as u32
        })
    };
    let mut tets = Vec::with_capacity(m.tets.len() * 8);
    for t in &m.tets {
        let (a, b, c, d) = (t[0], t[1], t[2], t[3]);
        let ab = midpoint(a, b, &mut nodes);
        let bc = midpoint(b, c, &mut nodes);
        let ac = midpoint(a, c, &mut nodes);
        let ad = midpoint(a, d, &mut nodes);
        let bd = midpoint(b, d, &mut nodes);
        let cd = midpoint(c, d, &mut nodes);
        let mut push = |t: [u32; 4], nodes: &Vec<P3>| {
            let mut t = t;
            if signed_volume(nodes, &t) < 0.0 {
                t.swap(2, 3);
            }
            tets.push(t);
        };
        push([a, ab, ac, ad], &nodes);
        push([b, ab, bc, bd], &nodes);
        push([c, ac, bc, cd], &nodes);
        push([d, ad, bd, cd], &nodes);
        // octahedron ab, bc, ac, ad, bd, cd split along the diagonal ab–cd... choose the shortest of the three
        let dist = |p: u32, q: u32| {
            let (u, v) = (nodes[p as usize], nodes[q as usize]);
            (u[0] - v[0]).powi(2) + (u[1] - v[1]).powi(2) + (u[2] - v[2]).powi(2)
        };
        let diagonals = [
            (ab, cd, [ac, bc, bd, ad]),
            (ac, bd, [ab, bc, cd, ad]),
            (ad, bc, [ab, ac, cd, bd]),
        ];
        let (p, q, ring) = *diagonals
            .iter()
            .min_by(|x, y| dist(x.0, x.1).partial_cmp(&dist(y.0, y.1)).unwrap())
            .unwrap();
        for k in 0..4 {
            push([p, q, ring[k], ring[(k + 1) % 4]], &nodes);
        }
    }
    let mut surface = Vec::with_capacity(m.surface.len() * 4);
    for s in &m.surface {
        let (a, b, c) = (s.v[0], s.v[1], s.v[2]);
        let (ab, bc, ca) = (
            midpoint(a, b, &mut nodes),
            midpoint(b, c, &mut nodes),
            midpoint(c, a, &mut nodes),
        );
        for v in [[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]] {
            surface.push(crate::mesh::SurfaceTri { v, tag: s.tag });
        }
    }
    TetMesh {
        nodes,
        tets,
        surface,
    }
}
