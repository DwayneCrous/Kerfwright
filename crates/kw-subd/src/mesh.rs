use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum SubdError {
    Invalid(String),
    /// The cage has boundary edges, so its limit surface is not a solid.
    NotClosed,
    /// Even the finest allowed level misses the tolerance.
    ToleranceNotMet {
        achieved: f64,
        level: usize,
    },
    TooLarge(String),
    Kernel(String),
}

impl fmt::Display for SubdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SubdError::Invalid(m) => write!(f, "invalid subdivision cage: {m}"),
            SubdError::NotClosed => write!(f, "the cage is open: a solid needs a closed cage"),
            SubdError::ToleranceNotMet { achieved, level } => write!(
                f,
                "level {level} still deviates {achieved:.4} mm from the limit surface"
            ),
            SubdError::TooLarge(m) => write!(f, "{m}"),
            SubdError::Kernel(m) => write!(f, "{m}"),
        }
    }
}
impl std::error::Error for SubdError {}

/// A crease along the edge `a`–`b`. Sharpness `s` keeps the edge sharp through `s` subdivision steps
/// (values of 100 or more are infinitely sharp).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Crease {
    pub a: u32,
    pub b: u32,
    pub sharpness: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SubdMesh {
    pub verts: Vec<[f64; 3]>,
    /// Polygons, counter-clockwise seen from outside.
    pub faces: Vec<Vec<u32>>,
    pub creases: Vec<Crease>,
}

impl SubdMesh {
    pub fn new(verts: Vec<[f64; 3]>, faces: Vec<Vec<u32>>) -> SubdMesh {
        SubdMesh {
            verts,
            faces,
            creases: Vec::new(),
        }
    }

    pub fn with_crease(mut self, a: u32, b: u32, sharpness: f64) -> Self {
        self.creases.push(Crease { a, b, sharpness });
        self
    }

    /// Every edge of every face gets sharpness `s`.
    pub fn crease_all(mut self, s: f64) -> Self {
        let mut seen = HashMap::new();
        for f in &self.faces {
            for i in 0..f.len() {
                let (a, b) = (f[i], f[(i + 1) % f.len()]);
                let k = (a.min(b), a.max(b));
                if seen.insert(k, ()).is_none() {
                    self.creases.push(Crease {
                        a: k.0,
                        b: k.1,
                        sharpness: s,
                    });
                }
            }
        }
        self
    }

    /// Check indices, manifoldness and orientation. A closed, consistently oriented cage passes.
    pub fn validate(&self) -> Result<(), SubdError> {
        let bad = |m: String| Err(SubdError::Invalid(m));
        if self.faces.is_empty() {
            return bad("there are no faces".into());
        }
        for (i, v) in self.verts.iter().enumerate() {
            if !v.iter().all(|c| c.is_finite()) {
                return bad(format!("vertex {i} has a non-finite coordinate"));
            }
        }
        let mut directed: HashMap<(u32, u32), usize> = HashMap::new();
        let mut used = vec![false; self.verts.len()];
        for (fi, f) in self.faces.iter().enumerate() {
            if f.len() < 3 {
                return bad(format!("face {fi} has fewer than 3 vertices"));
            }
            for (k, &v) in f.iter().enumerate() {
                if (v as usize) >= self.verts.len() {
                    return bad(format!("face {fi} uses the missing vertex {v}"));
                }
                used[v as usize] = true;
                if f.iter().skip(k + 1).any(|&w| w == v) {
                    return bad(format!("face {fi} repeats vertex {v}"));
                }
                let w = f[(k + 1) % f.len()];
                if directed.insert((v, w), fi).is_some() {
                    return bad(format!(
                        "the edge {v}→{w} is used twice in the same direction (inconsistent orientation or a non-manifold edge)"
                    ));
                }
            }
        }
        for c in &self.creases {
            if !directed.contains_key(&(c.a, c.b)) && !directed.contains_key(&(c.b, c.a)) {
                return bad(format!(
                    "the crease {}–{} is not an edge of the cage",
                    c.a, c.b
                ));
            }
            if !(c.sharpness >= 0.0) {
                return bad("a crease sharpness must not be negative".into());
            }
        }
        if used.iter().any(|u| !*u) {
            return bad("a vertex belongs to no face".into());
        }
        // vertex fans: the faces around a vertex must form one cycle (closed) or one chain
        for v in 0..self.verts.len() as u32 {
            let out: Vec<(u32, u32)> = directed.keys().copied().filter(|(a, _)| *a == v).collect();
            let inn: Vec<(u32, u32)> = directed.keys().copied().filter(|(_, b)| *b == v).collect();
            let boundary_out = out
                .iter()
                .filter(|(a, b)| !directed.contains_key(&(*b, *a)))
                .count();
            let boundary_in = inn
                .iter()
                .filter(|(a, b)| !directed.contains_key(&(*b, *a)))
                .count();
            if boundary_out > 1 || boundary_in > 1 {
                return bad(format!("vertex {v} is a non-manifold vertex"));
            }
            // walk the fan
            let start = match out.iter().find(|(a, b)| !directed.contains_key(&(*b, *a))) {
                Some(e) => *e,
                None => match out.first() {
                    Some(e) => *e,
                    None => continue,
                },
            };
            let nfaces = out.len();
            let mut seen = 0usize;
            let mut cur = start;
            loop {
                seen += 1;
                // the face containing directed edge cur = (v, w): the previous edge around v is (u, v)
                // where u precedes v in that face
                let fi = directed[&cur];
                let f = &self.faces[fi];
                let k = f.iter().position(|x| *x == v).unwrap();
                let prev = f[(k + f.len() - 1) % f.len()];
                // the neighbouring face across edge (prev, v) holds the directed edge (v, prev)
                match directed.get(&(v, prev)) {
                    Some(_) => {
                        cur = (v, prev);
                        if cur == start {
                            break;
                        }
                    }
                    None => break,
                }
                if seen > nfaces + 1 {
                    return bad(format!("vertex {v} is a non-manifold vertex"));
                }
            }
            if seen != nfaces {
                return bad(format!("vertex {v} is a non-manifold vertex"));
            }
        }
        Ok(())
    }

    pub fn is_closed(&self) -> bool {
        let mut directed: HashMap<(u32, u32), ()> = HashMap::new();
        for f in &self.faces {
            for k in 0..f.len() {
                directed.insert((f[k], f[(k + 1) % f.len()]), ());
            }
        }
        directed
            .keys()
            .all(|(a, b)| directed.contains_key(&(*b, *a)))
    }

    /// Signed volume enclosed by the cage (positive when the faces face outwards).
    pub fn signed_volume(&self) -> f64 {
        let mut v = 0.0;
        for f in &self.faces {
            let p = |i: usize| self.verts[f[i] as usize];
            for k in 1..f.len() - 1 {
                let (a, b, c) = (p(0), p(k), p(k + 1));
                v += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]);
            }
        }
        v / 6.0
    }

    /// The same cage with every face reversed.
    pub fn flipped(&self) -> SubdMesh {
        SubdMesh {
            verts: self.verts.clone(),
            faces: self
                .faces
                .iter()
                .map(|f| f.iter().rev().copied().collect())
                .collect(),
            creases: self.creases.clone(),
        }
    }

    // ------------------------------------------------------------------------ example cages

    /// A cube with edge length `s` centred on the origin (eight valence-3 vertices).
    pub fn cube(s: f64) -> SubdMesh {
        let h = s / 2.0;
        let v = vec![
            [-h, -h, -h],
            [h, -h, -h],
            [h, h, -h],
            [-h, h, -h],
            [-h, -h, h],
            [h, -h, h],
            [h, h, h],
            [-h, h, h],
        ];
        let f = vec![
            vec![0, 3, 2, 1],
            vec![4, 5, 6, 7],
            vec![0, 1, 5, 4],
            vec![2, 3, 7, 6],
            vec![1, 2, 6, 5],
            vec![3, 0, 4, 7],
        ];
        SubdMesh::new(v, f)
    }

    /// A box `a × b × c`.
    pub fn box_(a: f64, b: f64, c: f64) -> SubdMesh {
        let mut m = SubdMesh::cube(1.0);
        for v in &mut m.verts {
            v[0] *= a;
            v[1] *= b;
            v[2] *= c;
        }
        m
    }

    /// A torus-like ring of `nu × nv` quads (all vertices valence 4).
    pub fn torus(major: f64, minor: f64, nu: usize, nv: usize) -> SubdMesh {
        let mut verts = Vec::new();
        for i in 0..nu {
            let a = std::f64::consts::TAU * i as f64 / nu as f64;
            for j in 0..nv {
                let b = std::f64::consts::TAU * j as f64 / nv as f64;
                let r = major + minor * b.cos();
                verts.push([r * a.cos(), r * a.sin(), minor * b.sin()]);
            }
        }
        let id = |i: usize, j: usize| ((i % nu) * nv + (j % nv)) as u32;
        let mut faces = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                faces.push(vec![id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        let m = SubdMesh::new(verts, faces);
        if m.signed_volume() < 0.0 {
            m.flipped()
        } else {
            m
        }
    }

    /// A prism over a regular `n`-gon: two n-gon faces and n quads (valence-3 vertices).
    pub fn prism(n: usize, radius: f64, height: f64) -> SubdMesh {
        let mut verts = Vec::new();
        for z in [-height / 2.0, height / 2.0] {
            for i in 0..n {
                let a = std::f64::consts::TAU * i as f64 / n as f64;
                verts.push([radius * a.cos(), radius * a.sin(), z]);
            }
        }
        let mut faces = vec![
            (0..n as u32).rev().collect::<Vec<_>>(),
            (n as u32..2 * n as u32).collect(),
        ];
        for i in 0..n {
            let j = (i + 1) % n;
            faces.push(vec![i as u32, j as u32, (n + j) as u32, (n + i) as u32]);
        }
        SubdMesh::new(verts, faces)
    }

    /// A pyramid over a regular `n`-gon (apex of valence `n`, triangular sides).
    pub fn pyramid(n: usize, radius: f64, height: f64) -> SubdMesh {
        let mut verts = Vec::new();
        for i in 0..n {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            verts.push([radius * a.cos(), radius * a.sin(), -height / 3.0]);
        }
        verts.push([0.0, 0.0, 2.0 * height / 3.0]);
        let apex = n as u32;
        let mut faces = vec![(0..n as u32).rev().collect::<Vec<_>>()];
        for i in 0..n {
            faces.push(vec![i as u32, ((i + 1) % n) as u32, apex]);
        }
        SubdMesh::new(verts, faces)
    }

    /// A bipyramid over a regular `n`-gon: two apexes of valence `n`.
    pub fn bipyramid(n: usize, radius: f64, height: f64) -> SubdMesh {
        let mut verts = Vec::new();
        for i in 0..n {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            verts.push([radius * a.cos(), radius * a.sin(), 0.0]);
        }
        verts.push([0.0, 0.0, height]);
        verts.push([0.0, 0.0, -height]);
        let (top, bottom) = (n as u32, n as u32 + 1);
        let mut faces = Vec::new();
        for i in 0..n {
            let j = ((i + 1) % n) as u32;
            faces.push(vec![i as u32, j, top]);
            faces.push(vec![j, i as u32, bottom]);
        }
        SubdMesh::new(verts, faces)
    }
}
