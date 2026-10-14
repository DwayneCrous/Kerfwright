//! Editing the cage of a Form in the viewport: picking vertices, edges and faces, box selection, dragging
//! the selection in the plane facing the camera or locked to an axis or a plane, snapping to a grid,
//! mirror symmetry, and a local undo of the cage while the dialog is open.
//!
//! Everything here is a function of the cage and of a [`View`] (how a point reaches the screen and which ray
//! a screen position makes), so it is tested without a window. Coordinates are *world* millimetres: the
//! cage's own numbers times the Form's scale. The viewport code turns them into scene coordinates.

use eframe::egui::{Pos2, Rect};
use kw_subd::SubdMesh;
use std::collections::BTreeSet;

pub type P3 = [f64; 3];

/// How the cage reaches the screen.
pub struct View<'a> {
    /// A world point on the screen (`None` when it cannot be shown, e.g. behind the eye).
    pub project: &'a dyn Fn(P3) -> Option<Pos2>,
    /// The ray (origin, unit direction) through a screen position.
    pub ray: &'a dyn Fn(Pos2) -> (P3, P3),
}

/// What a pointer position is over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hit {
    Vertex(u32),
    Edge(u32, u32),
    Face(usize),
}

/// What a drag may change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Lock {
    /// In the plane facing the camera.
    #[default]
    Free,
    /// Along one axis (0 = x, 1 = y, 2 = z).
    Axis(u8),
    /// In the plane perpendicular to one axis, i.e. not along it.
    Plane(u8),
}

impl Lock {
    pub fn label(self) -> &'static str {
        match self {
            Lock::Free => "Free",
            Lock::Axis(0) => "X axis",
            Lock::Axis(1) => "Y axis",
            Lock::Axis(_) => "Z axis",
            Lock::Plane(0) => "YZ plane",
            Lock::Plane(1) => "XZ plane",
            Lock::Plane(_) => "XY plane",
        }
    }
}

/// How close (pixels) the pointer must be to pick a vertex or an edge.
pub const VERTEX_PX: f32 = 9.0;
pub const EDGE_PX: f32 = 6.0;

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn mul(a: P3, k: f64) -> P3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn axis_vec(i: u8) -> P3 {
    let mut e = [0.0; 3];
    e[(i as usize).min(2)] = 1.0;
    e
}

/// The cage vertex `v` in world coordinates.
pub fn world(cage: &SubdMesh, k: f64, v: u32) -> P3 {
    mul(cage.verts[v as usize], k)
}

fn dist_to_segment(p: Pos2, a: Pos2, b: Pos2) -> (f32, f32) {
    let ab = b - a;
    let l2 = ab.length_sq();
    let t = if l2 > 1e-9 {
        ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    ((a + ab * t).distance(p), t)
}

/// Where a ray meets the convex or concave polygon `pts` (a triangle fan), as the distance along the ray.
fn ray_polygon(origin: P3, dir: P3, pts: &[P3]) -> Option<f64> {
    let mut best: Option<f64> = None;
    for i in 1..pts.len().saturating_sub(1) {
        let (a, b, c) = (pts[0], pts[i], pts[i + 1]);
        let (e1, e2) = (sub(b, a), sub(c, a));
        let p = cross(dir, e2);
        let det = dot(e1, p);
        if det.abs() < 1e-18 {
            continue;
        }
        let inv = 1.0 / det;
        let s = sub(origin, a);
        let u = dot(s, p) * inv;
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = cross(s, e1);
        let w = dot(dir, q) * inv;
        if w < 0.0 || u + w > 1.0 {
            continue;
        }
        let t = dot(e2, q) * inv;
        if t > 0.0 && best.is_none_or(|b| t < b) {
            best = Some(t);
        }
    }
    best
}

/// What lies under the pointer: a vertex within [`VERTEX_PX`], else an edge within [`EDGE_PX`], else a face.
/// Among candidates the nearer one in the screen wins, and one closer to the eye wins a tie.
pub fn hit_test(cage: &SubdMesh, k: f64, view: &View, pos: Pos2) -> Option<Hit> {
    let (o, d) = (view.ray)(pos);
    let depth = |p: P3| dot(sub(p, o), d);
    let pts: Vec<Option<Pos2>> = (0..cage.verts.len())
        .map(|v| (view.project)(world(cage, k, v as u32)))
        .collect();
    // vertices
    let mut best: Option<(f32, f64, u32)> = None;
    for (v, p) in pts.iter().enumerate() {
        let Some(p) = p else { continue };
        let dist = p.distance(pos);
        if dist > VERTEX_PX {
            continue;
        }
        let z = depth(world(cage, k, v as u32));
        // a nearer vertex on the screen wins unless the other is clearly (3 px) as near and in front
        let better = match best {
            None => true,
            Some((bd, bz, _)) => dist + 3.0 < bd || (dist < bd + 3.0 && z < bz),
        };
        if better {
            best = Some((dist, z, v as u32));
        }
    }
    if let Some((_, _, v)) = best {
        return Some(Hit::Vertex(v));
    }
    // edges
    let mut best_e: Option<(f32, f64, (u32, u32))> = None;
    for (a, b) in cage.edges() {
        let (Some(pa), Some(pb)) = (pts[a as usize], pts[b as usize]) else {
            continue;
        };
        let (dist, t) = dist_to_segment(pos, pa, pb);
        if dist > EDGE_PX {
            continue;
        }
        let (wa, wb) = (world(cage, k, a), world(cage, k, b));
        let z = depth(add(mul(wa, 1.0 - t as f64), mul(wb, t as f64)));
        let better = match best_e {
            None => true,
            Some((bd, bz, _)) => dist + 2.0 < bd || (dist < bd + 2.0 && z < bz),
        };
        if better {
            best_e = Some((dist, z, (a, b)));
        }
    }
    if let Some((_, _, (a, b))) = best_e {
        return Some(Hit::Edge(a, b));
    }
    // faces: the nearest the ray meets
    let mut best_f: Option<(f64, usize)> = None;
    for (fi, f) in cage.faces.iter().enumerate() {
        let poly: Vec<P3> = f.iter().map(|&v| world(cage, k, v)).collect();
        if let Some(t) = ray_polygon(o, d, &poly) {
            if best_f.is_none_or(|(bt, _)| t < bt) {
                best_f = Some((t, fi));
            }
        }
        // a face seen from behind is picked too (the cage is see-through)
        let rev: Vec<P3> = poly.iter().rev().copied().collect();
        if let Some(t) = ray_polygon(o, d, &rev) {
            if best_f.is_none_or(|(bt, _)| t < bt) {
                best_f = Some((t, fi));
            }
        }
    }
    best_f.map(|(_, f)| Hit::Face(f))
}

/// The vertices a hit stands for.
pub fn vertices_of(cage: &SubdMesh, hit: Hit) -> Vec<u32> {
    match hit {
        Hit::Vertex(v) => vec![v],
        Hit::Edge(a, b) => vec![a, b],
        Hit::Face(f) => {
            let mut v = cage.faces[f].clone();
            v.sort_unstable();
            v.dedup();
            v
        }
    }
}

/// The vertices whose screen positions lie inside `rect`.
pub fn box_select(cage: &SubdMesh, k: f64, view: &View, rect: Rect) -> Vec<u32> {
    (0..cage.verts.len() as u32)
        .filter(|&v| (view.project)(world(cage, k, v)).is_some_and(|p| rect.contains(p)))
        .collect()
}

/// For each vertex, the vertex at its mirror image across the plane through the origin normal to `axis`
/// (itself when it lies on the plane), matched within `tol`.
pub fn mirror_partners(cage: &SubdMesh, axis: u8, tol: f64) -> Vec<Option<u32>> {
    let a = (axis as usize).min(2);
    let n = cage.verts.len();
    let mut out = vec![None; n];
    for (v, slot) in out.iter_mut().enumerate() {
        let p = cage.verts[v];
        if p[a].abs() <= tol {
            *slot = Some(v as u32);
            continue;
        }
        let mut q = p;
        q[a] = -q[a];
        *slot = (0..n)
            .find(|&w| w != v && (0..3).all(|c| (cage.verts[w][c] - q[c]).abs() <= tol))
            .map(|w| w as u32);
    }
    out
}

/// A drag in progress.
#[derive(Debug, Clone)]
pub struct Drag {
    /// The point under the pointer when it started (world).
    pub grab: P3,
    /// The camera's viewing direction (the free drag slides in the plane facing the camera).
    pub forward: P3,
    /// The selected vertices and where they were (cage coordinates).
    pub start: Vec<(u32, P3)>,
    /// The vertex that was grabbed, for snapping.
    pub primary: u32,
    /// The cage before the drag.
    pub before: SubdMesh,
}

/// Where the pointer ray puts the grabbed point, relative to `grab`: the movement in world coordinates.
/// `None` when the ray cannot reach (parallel to the plane or the axis).
pub fn drag_delta(drag: &Drag, lock: Lock, ray: (P3, P3)) -> Option<P3> {
    let (o, d) = ray;
    match lock {
        Lock::Free | Lock::Plane(_) => {
            let n = match lock {
                Lock::Plane(i) => axis_vec(i),
                _ => drag.forward,
            };
            let denom = dot(d, n);
            if denom.abs() < 1e-9 {
                return None;
            }
            let t = dot(sub(drag.grab, o), n) / denom;
            if t < 0.0 {
                return None;
            }
            Some(sub(add(o, mul(d, t)), drag.grab))
        }
        Lock::Axis(i) => {
            // the point of the axis line nearest the ray
            let e = axis_vec(i);
            let w0 = sub(drag.grab, o);
            let (b, dd, ee) = (dot(e, d), dot(e, w0), dot(d, w0));
            let denom = 1.0 - b * b;
            if denom < 1e-9 {
                return None;
            }
            let s = (b * ee - dd) / denom;
            Some(mul(e, s))
        }
    }
}

/// Round `delta` so that the grabbed vertex lands on the grid (every component the lock leaves free).
pub fn snap_delta(drag: &Drag, k: f64, lock: Lock, grid: f64, delta: P3) -> P3 {
    if grid <= 0.0 {
        return delta;
    }
    let start = world_of(drag, k);
    let mut out = delta;
    for c in 0..3 {
        let free = match lock {
            Lock::Free => true,
            Lock::Axis(i) => i as usize == c,
            Lock::Plane(i) => i as usize != c,
        };
        if free {
            let target = ((start[c] + delta[c]) / grid).round() * grid;
            out[c] = target - start[c];
        }
    }
    out
}

fn world_of(drag: &Drag, k: f64) -> P3 {
    let p = drag
        .start
        .iter()
        .find(|(v, _)| *v == drag.primary)
        .map_or([0.0; 3], |(_, p)| *p);
    mul(p, k)
}

/// Move the dragged vertices by `delta` (world), and with `mirror` set the mirror images of the moved
/// vertices with them (a vertex on the mirror plane stays on it).
pub fn apply_drag(cage: &mut SubdMesh, drag: &Drag, k: f64, delta: P3, mirror: Option<u8>) {
    let d = mul(delta, 1.0 / k.max(1e-12));
    let moved: BTreeSet<u32> = drag.start.iter().map(|(v, _)| *v).collect();
    for (v, p) in &drag.start {
        cage.verts[*v as usize] = add(*p, d);
    }
    let Some(axis) = mirror else { return };
    let a = (axis as usize).min(2);
    // partners are found on the cage as it was before the drag
    let tol = 1e-6
        * drag
            .before
            .verts
            .iter()
            .flatten()
            .fold(1.0f64, |m, c| m.max(c.abs()));
    let partners = mirror_partners(&drag.before, axis, tol);
    for &v in &moved {
        match partners[v as usize] {
            Some(w) if w == v => {
                // on the plane: it stays there
                cage.verts[v as usize][a] = 0.0;
            }
            Some(w) => {
                if moved.contains(&w) && w < v {
                    continue; // the lower of a selected pair leads
                }
                let mut q = cage.verts[v as usize];
                q[a] = -q[a];
                cage.verts[w as usize] = q;
            }
            None => {}
        }
    }
}

/// The edges (as sorted pairs) whose two ends are both in `selected`.
pub fn selected_edges(cage: &SubdMesh, selected: &BTreeSet<u32>) -> Vec<(u32, u32)> {
    cage.edges()
        .into_iter()
        .filter(|(a, b)| selected.contains(a) && selected.contains(b))
        .collect()
}

/// The state of the interactive editing of the cage in the Form dialog.
#[derive(Debug, Clone, Default)]
pub struct FormEdit {
    /// The selected cage vertices.
    pub selected: BTreeSet<u32>,
    pub hover: Option<Hit>,
    pub drag: Option<Drag>,
    /// A box being dragged out in the view (screen points).
    pub boxsel: Option<(Pos2, Pos2)>,
    /// A press on empty space that clears the selection if it turns out to be a click.
    pub clear_on_release: Option<Pos2>,
    pub lock: Lock,
    /// Grid size in mm when snapping is on.
    pub snap: Option<f64>,
    /// Mirror symmetry across the plane normal to this axis.
    pub mirror: Option<u8>,
    /// Earlier cages (most recent last), for the dialog's own Ctrl+Z.
    pub undo: Vec<SubdMesh>,
    pub redo: Vec<SubdMesh>,
    baseline: Option<SubdMesh>,
    /// `(kind, size, sides)` of the dialog as last seen: a change is a new primitive, not an edit.
    pub last_built: Option<(u8, [f64; 3], u32)>,
    /// The scene centre is held while a drag changes the preview, so the view does not slide.
    pub frozen_centre: Option<[f64; 3]>,
}

const UNDO_LIMIT: usize = 200;

impl FormEdit {
    pub fn clear(&mut self) {
        *self = FormEdit::default();
    }

    pub fn dragging(&self) -> bool {
        self.drag.is_some() || self.boxsel.is_some()
    }

    /// Note the cage as the dialog shows it this frame. While no drag is going on, a cage that differs from
    /// the last one noted is a finished edit (a drag, a button, typed numbers): the last one goes on the
    /// undo stack. Returns whether the cage changed.
    pub fn sync(&mut self, cage: &SubdMesh) -> bool {
        match &self.baseline {
            None => {
                self.baseline = Some(cage.clone());
                false
            }
            Some(b) if self.drag.is_some() => {
                let _ = b;
                false
            }
            Some(b) if b == cage => false,
            Some(_) => {
                let old = self.baseline.replace(cage.clone()).expect("a baseline");
                self.undo.push(old);
                if self.undo.len() > UNDO_LIMIT {
                    self.undo.remove(0);
                }
                self.redo.clear();
                true
            }
        }
    }

    /// The cage before the last edit, if any; the current one goes to the redo stack.
    pub fn undo(&mut self, current: &SubdMesh) -> Option<SubdMesh> {
        let prev = self.undo.pop()?;
        self.redo.push(current.clone());
        self.baseline = Some(prev.clone());
        Some(prev)
    }

    pub fn redo(&mut self, current: &SubdMesh) -> Option<SubdMesh> {
        let next = self.redo.pop()?;
        self.undo.push(current.clone());
        self.baseline = Some(next.clone());
        Some(next)
    }

    /// Select what `hit` stands for: it replaces the selection, or with `add` toggles those vertices.
    pub fn select(&mut self, cage: &SubdMesh, hit: Hit, add: bool) {
        let vs = vertices_of(cage, hit);
        if add {
            if vs.iter().all(|v| self.selected.contains(v)) {
                for v in vs {
                    self.selected.remove(&v);
                }
            } else {
                self.selected.extend(vs);
            }
        } else {
            self.selected = vs.into_iter().collect();
        }
    }

    /// Start dragging the selection by the vertex `primary`, grabbed at the world point `grab`.
    pub fn begin_drag(&mut self, cage: &SubdMesh, primary: u32, grab: P3, forward: P3) {
        let start: Vec<(u32, P3)> = self
            .selected
            .iter()
            .filter(|v| (**v as usize) < cage.verts.len())
            .map(|&v| (v, cage.verts[v as usize]))
            .collect();
        if start.is_empty() {
            return;
        }
        self.drag = Some(Drag {
            grab,
            forward,
            start,
            primary,
            before: cage.clone(),
        });
    }

    /// Move the cage to where the pointer ray puts the dragged vertices. Returns whether anything moved.
    pub fn update_drag(&mut self, cage: &mut SubdMesh, k: f64, ray: (P3, P3)) -> bool {
        let Some(drag) = &self.drag else { return false };
        let Some(delta) = drag_delta(drag, self.lock, ray) else {
            return false;
        };
        let delta = snap_delta(drag, k, self.lock, self.snap.unwrap_or(0.0), delta);
        apply_drag(cage, drag, k, delta, self.mirror);
        true
    }

    /// Finish the drag; the cage stays where it is.
    pub fn end_drag(&mut self) {
        self.drag = None;
        self.frozen_centre = None;
    }

    /// Stop the drag and put the cage back as it was.
    pub fn cancel_drag(&mut self, cage: &mut SubdMesh) {
        if let Some(d) = self.drag.take() {
            *cage = d.before;
        }
        self.frozen_centre = None;
    }

    /// Set the lock from a key: X / Y / Z lock to an axis, with Shift to the plane normal to it; pressing the
    /// same key again frees the drag.
    pub fn key_lock(&mut self, axis: u8, plane: bool) {
        let want = if plane {
            Lock::Plane(axis)
        } else {
            Lock::Axis(axis)
        };
        self.lock = if self.lock == want { Lock::Free } else { want };
    }
}
