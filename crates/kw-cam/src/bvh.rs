//! Triangle meshes and a bounding-volume hierarchy for 3-D toolpath queries.

use crate::P3;

pub type Tri = [P3; 3];

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: P3,
    pub max: P3,
}

impl Aabb {
    pub fn empty() -> Aabb {
        Aabb {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
        }
    }

    pub fn grow(&mut self, p: P3) {
        for k in 0..3 {
            self.min[k] = self.min[k].min(p[k]);
            self.max[k] = self.max[k].max(p[k]);
        }
    }

    pub fn merge(&mut self, o: &Aabb) {
        self.grow(o.min);
        self.grow(o.max);
    }

    pub fn overlaps(&self, o: &Aabb) -> bool {
        (0..3).all(|k| self.min[k] <= o.max[k] && o.min[k] <= self.max[k])
    }

    pub fn of_tri(t: &Tri) -> Aabb {
        let mut b = Aabb::empty();
        for p in t {
            b.grow(*p);
        }
        b
    }
}

#[derive(Debug, Clone, Copy)]
struct Node {
    bounds: Aabb,
    /// Leaf: first index into `order` and the count; inner: left child index (right = left + 1) and 0.
    first: u32,
    count: u32,
}

#[derive(Debug, Clone)]
pub struct TriMesh {
    pub tris: Vec<Tri>,
    nodes: Vec<Node>,
    order: Vec<u32>,
}

const LEAF: usize = 4;

impl TriMesh {
    pub fn new(tris: Vec<Tri>) -> TriMesh {
        let n = tris.len();
        let mut order: Vec<u32> = (0..n as u32).collect();
        let boxes: Vec<Aabb> = tris.iter().map(Aabb::of_tri).collect();
        let centre = |i: u32, k: usize| 0.5 * (boxes[i as usize].min[k] + boxes[i as usize].max[k]);
        let mut nodes: Vec<Node> = Vec::new();
        if n > 0 {
            nodes.push(Node {
                bounds: Aabb::empty(),
                first: 0,
                count: 0,
            });
            // explicit stack of (node, begin, end)
            let mut stack = vec![(0usize, 0usize, n)];
            while let Some((ni, b, e)) = stack.pop() {
                let mut bounds = Aabb::empty();
                for &i in &order[b..e] {
                    bounds.merge(&boxes[i as usize]);
                }
                nodes[ni].bounds = bounds;
                if e - b <= LEAF {
                    nodes[ni].first = b as u32;
                    nodes[ni].count = (e - b) as u32;
                    continue;
                }
                let ext = [
                    bounds.max[0] - bounds.min[0],
                    bounds.max[1] - bounds.min[1],
                    bounds.max[2] - bounds.min[2],
                ];
                let axis = if ext[0] >= ext[1] && ext[0] >= ext[2] {
                    0
                } else if ext[1] >= ext[2] {
                    1
                } else {
                    2
                };
                let mid = (b + e) / 2;
                order[b..e].select_nth_unstable_by(mid - b, |&p, &q| {
                    centre(p, axis).total_cmp(&centre(q, axis))
                });
                let left = nodes.len();
                nodes.push(Node {
                    bounds: Aabb::empty(),
                    first: 0,
                    count: 0,
                });
                nodes.push(Node {
                    bounds: Aabb::empty(),
                    first: 0,
                    count: 0,
                });
                nodes[ni].first = left as u32;
                nodes[ni].count = 0;
                stack.push((left, b, mid));
                stack.push((left + 1, mid, e));
            }
        }
        TriMesh { tris, nodes, order }
    }

    pub fn bounds(&self) -> Aabb {
        self.nodes.first().map_or_else(Aabb::empty, |n| n.bounds)
    }

    /// Call `f` with the index of every triangle whose bounding box overlaps `q`.
    pub fn query(&self, q: &Aabb, mut f: impl FnMut(usize)) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let n = &self.nodes[ni];
            if !n.bounds.overlaps(q) {
                continue;
            }
            if n.count > 0 {
                for &i in &self.order[n.first as usize..(n.first + n.count) as usize] {
                    if Aabb::of_tri(&self.tris[i as usize]).overlaps(q) {
                        f(i as usize);
                    }
                }
            } else {
                stack.push(n.first as usize);
                stack.push(n.first as usize + 1);
            }
        }
    }

    /// Triangles of the faces of an axis-aligned box.
    pub fn from_box(min: P3, max: P3) -> TriMesh {
        let c = |i: usize| {
            [
                if i & 1 == 0 { min[0] } else { max[0] },
                if i & 2 == 0 { min[1] } else { max[1] },
                if i & 4 == 0 { min[2] } else { max[2] },
            ]
        };
        let quads = [
            [0, 1, 3, 2],
            [4, 6, 7, 5],
            [0, 4, 5, 1],
            [2, 3, 7, 6],
            [0, 2, 6, 4],
            [1, 5, 7, 3],
        ];
        let mut t = Vec::new();
        for q in quads {
            t.push([c(q[0]), c(q[1]), c(q[2])]);
            t.push([c(q[0]), c(q[2]), c(q[3])]);
        }
        TriMesh::new(t)
    }
}
