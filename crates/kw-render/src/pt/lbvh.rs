//! Linear BVH: Morton codes of the triangle centroids, a radix tree over the sorted codes (Karras 2012), and
//! bounds propagated up the tree. Nodes are laid out for the GPU: 32 bytes, children are indices into the
//! node array, or `LEAF | triangle` for a leaf.

use super::{PtScene, V3};
use rayon::prelude::*;

pub const LEAF: u32 = 0x8000_0000;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Node {
    pub min: [f32; 3],
    pub left: u32,
    pub max: [f32; 3],
    pub right: u32,
}

pub struct Lbvh {
    pub nodes: Vec<Node>,
    /// Triangles in leaf order: index into the scene.
    pub order: Vec<u32>,
}

fn expand_bits(v: u32) -> u32 {
    let mut v = v & 0x3ff;
    v = (v | (v << 16)) & 0x0300_00FF;
    v = (v | (v << 8)) & 0x0300_F00F;
    v = (v | (v << 4)) & 0x030C_30C3;
    (v | (v << 2)) & 0x0924_9249
}

fn morton(p: [f32; 3]) -> u32 {
    let q = |x: f32| ((x.clamp(0.0, 1.0) * 1023.0) as u32).min(1023);
    (expand_bits(q(p[0])) << 2) | (expand_bits(q(p[1])) << 1) | expand_bits(q(p[2]))
}

impl Lbvh {
    pub fn build(scene: &PtScene) -> Lbvh {
        let n = scene.tris.len();
        assert!(n >= 2, "an LBVH needs at least two triangles");
        let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
        let cent: Vec<V3> = scene
            .tris
            .iter()
            .map(|t| {
                let c = [
                    (t[0][0] + t[1][0] + t[2][0]) / 3.0,
                    (t[0][1] + t[1][1] + t[2][1]) / 3.0,
                    (t[0][2] + t[1][2] + t[2][2]) / 3.0,
                ];
                for k in 0..3 {
                    lo[k] = lo[k].min(c[k]);
                    hi[k] = hi[k].max(c[k]);
                }
                c
            })
            .collect();
        let ext: [f32; 3] = [
            (hi[0] - lo[0]).max(1e-20),
            (hi[1] - lo[1]).max(1e-20),
            (hi[2] - lo[2]).max(1e-20),
        ];
        let mut keyed: Vec<(u32, u32)> = cent
            .par_iter()
            .enumerate()
            .map(|(i, c)| {
                (
                    morton([
                        (c[0] - lo[0]) / ext[0],
                        (c[1] - lo[1]) / ext[1],
                        (c[2] - lo[2]) / ext[2],
                    ]),
                    i as u32,
                )
            })
            .collect();
        keyed.par_sort_unstable();
        let codes: Vec<u32> = keyed.iter().map(|k| k.0).collect();
        let order: Vec<u32> = keyed.iter().map(|k| k.1).collect();
        // common prefix length of the codes at i and j (ties broken by index), -1 when out of range
        let delta = |i: i64, j: i64| -> i32 {
            if j < 0 || j >= n as i64 {
                return -1;
            }
            let (a, b) = (codes[i as usize], codes[j as usize]);
            if a == b {
                32 + ((i as u32) ^ (j as u32)).leading_zeros() as i32
            } else {
                (a ^ b).leading_zeros() as i32
            }
        };
        let internal = n - 1;
        // children of internal node i
        let ch: Vec<(u32, u32)> = (0..internal)
            .into_par_iter()
            .map(|i| {
                let i = i as i64;
                let d: i64 = if delta(i, i + 1) - delta(i, i - 1) >= 0 {
                    1
                } else {
                    -1
                };
                let dmin = delta(i, i - d);
                let mut lmax = 2i64;
                while delta(i, i + lmax * d) > dmin {
                    lmax *= 2;
                }
                let mut l = 0i64;
                let mut t = lmax / 2;
                while t >= 1 {
                    if delta(i, i + (l + t) * d) > dmin {
                        l += t;
                    }
                    t /= 2;
                }
                let j = i + l * d;
                let dnode = delta(i, j);
                let mut s = 0i64;
                let mut div = 2i64;
                loop {
                    let t = (l + div - 1) / div;
                    if delta(i, i + (s + t) * d) > dnode {
                        s += t;
                    }
                    if t <= 1 {
                        break;
                    }
                    div *= 2;
                }
                let gamma = i + s * d + d.min(0);
                let (lo_i, hi_i) = (i.min(j), i.max(j));
                let left = if lo_i == gamma {
                    LEAF | gamma as u32
                } else {
                    gamma as u32
                };
                let right = if hi_i == gamma + 1 {
                    LEAF | (gamma + 1) as u32
                } else {
                    (gamma + 1) as u32
                };
                (left, right)
            })
            .collect();
        let tri_box = |leaf: u32| -> ([f32; 3], [f32; 3]) {
            let t = &scene.tris[order[(leaf & !LEAF) as usize] as usize];
            let (mut a, mut b) = ([f32::MAX; 3], [f32::MIN; 3]);
            for v in t {
                for k in 0..3 {
                    a[k] = a[k].min(v[k]);
                    b[k] = b[k].max(v[k]);
                }
            }
            (a, b)
        };
        // bounds bottom-up: parents by a post-order walk from the root
        let mut nodes = vec![
            Node {
                min: [0.0; 3],
                left: 0,
                max: [0.0; 3],
                right: 0
            };
            internal
        ];
        let mut stack: Vec<(usize, bool)> = vec![(0, false)];
        while let Some((i, done)) = stack.pop() {
            let (l, r) = ch[i];
            if !done {
                stack.push((i, true));
                if l & LEAF == 0 {
                    stack.push((l as usize, false));
                }
                if r & LEAF == 0 {
                    stack.push((r as usize, false));
                }
            } else {
                let bx = |c: u32, nodes: &Vec<Node>| {
                    if c & LEAF != 0 {
                        tri_box(c)
                    } else {
                        (nodes[c as usize].min, nodes[c as usize].max)
                    }
                };
                let (la, lb) = bx(l, &nodes);
                let (ra, rb) = bx(r, &nodes);
                nodes[i] = Node {
                    min: [la[0].min(ra[0]), la[1].min(ra[1]), la[2].min(ra[2])],
                    left: l,
                    max: [lb[0].max(rb[0]), lb[1].max(rb[1]), lb[2].max(rb[2])],
                    right: r,
                };
            }
        }
        Lbvh { nodes, order }
    }

    /// Closest hit (distance, scene triangle index) along the ray; Möller–Trumbore in f32.
    pub fn intersect(&self, scene: &PtScene, o: V3, d: V3, tmax: f32) -> Option<(f32, usize)> {
        let inv = [1.0 / d[0], 1.0 / d[1], 1.0 / d[2]];
        let mut best: Option<(f32, usize)> = None;
        let mut tmax = tmax;
        let mut stack = [0u32; 64];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let c = stack[sp];
            if c & LEAF != 0 {
                let ti = self.order[(c & !LEAF) as usize] as usize;
                if let Some(t) = tri_hit(&scene.tris[ti], o, d, tmax) {
                    tmax = t;
                    best = Some((t, ti));
                }
                continue;
            }
            let n = &self.nodes[c as usize];
            let (mut t0, mut t1) = (0.0f32, tmax);
            for k in 0..3 {
                let (a, b) = ((n.min[k] - o[k]) * inv[k], (n.max[k] - o[k]) * inv[k]);
                t0 = t0.max(a.min(b));
                t1 = t1.min(a.max(b));
            }
            if t0 <= t1 {
                stack[sp] = n.left;
                stack[sp + 1] = n.right;
                sp += 2;
            }
        }
        best
    }
}

pub fn tri_hit(t: &[V3; 3], o: V3, d: V3, tmax: f32) -> Option<f32> {
    use super::{cross, dot, sub};
    let (e1, e2) = (sub(t[1], t[0]), sub(t[2], t[0]));
    let p = cross(d, e2);
    let det = dot(e1, p);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let s = sub(o, t[0]);
    let u = dot(s, p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = cross(s, e1);
    let v = dot(d, q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let tt = dot(e2, q) * inv;
    (tt > 1e-4 && tt < tmax).then_some(tt)
}
