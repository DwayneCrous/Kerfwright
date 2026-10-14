//! Picking and dragging the vertices of a Form cage: the maths, without a window.

#![allow(clippy::needless_range_loop)]

use eframe::egui::{pos2, Pos2, Rect};
use kw_app::design::form;
use kw_app::design::form_edit::*;
use kw_subd::{finish, FinishOptions, SubdMesh};
use std::collections::BTreeSet;

/// An orthographic camera looking along `forward`: 10 pixels to the mm, the origin at (400, 300).
struct Cam {
    right: P3,
    up: P3,
    forward: P3,
}

fn norm(a: P3) -> P3 {
    let l = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    [a[0] / l, a[1] / l, a[2] / l]
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

impl Cam {
    fn looking(forward: P3) -> Cam {
        let forward = norm(forward);
        let right = norm(cross(forward, [0.0, 0.0, 1.0]));
        let up = cross(right, forward);
        Cam { right, up, forward }
    }
    fn top() -> Cam {
        // straight down, x to the right, y up
        Cam {
            right: [1.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            forward: [0.0, 0.0, -1.0],
        }
    }
    fn project(&self, p: P3) -> Option<Pos2> {
        Some(pos2(
            400.0 + 10.0 * dot(p, self.right) as f32,
            300.0 - 10.0 * dot(p, self.up) as f32,
        ))
    }
    fn ray(&self, pos: Pos2) -> (P3, P3) {
        let (x, y) = (
            f64::from(pos.x - 400.0) / 10.0,
            f64::from(300.0 - pos.y) / 10.0,
        );
        let mut o = [0.0; 3];
        for c in 0..3 {
            o[c] = self.right[c] * x + self.up[c] * y - self.forward[c] * 1000.0;
        }
        (o, self.forward)
    }
}

fn with_view<R>(cam: &Cam, f: impl FnOnce(&View) -> R) -> R {
    let project = |p: P3| cam.project(p);
    let ray = |p: Pos2| cam.ray(p);
    f(&View {
        project: &project,
        ray: &ray,
    })
}

fn cube() -> SubdMesh {
    form::primitive(0, [20.0, 20.0, 20.0], 8)
}

fn at(cage: &SubdMesh, x: f64, y: f64, z: f64) -> u32 {
    cage.verts
        .iter()
        .position(|v| (v[0] - x).abs() < 1e-9 && (v[1] - y).abs() < 1e-9 && (v[2] - z).abs() < 1e-9)
        .unwrap_or_else(|| panic!("no vertex at {x} {y} {z}")) as u32
}

#[test]
fn the_pointer_picks_vertices_then_edges_then_faces_and_prefers_what_is_in_front() {
    let cage = cube();
    let cam = Cam::top();
    with_view(&cam, |view| {
        // a corner seen from above: the vertex on top and the one under it overlap; the top one wins
        let top = at(&cage, 10.0, 10.0, 10.0);
        let p = cam.project([10.0, 10.0, 10.0]).unwrap();
        assert_eq!(
            hit_test(&cage, 1.0, view, p + eframe::egui::vec2(3.0, 2.0)),
            Some(Hit::Vertex(top))
        );
        // the middle of a top edge
        let a = at(&cage, -10.0, 10.0, 10.0);
        let mid = cam.project([0.0, 10.0, 10.0]).unwrap();
        match hit_test(&cage, 1.0, view, mid + eframe::egui::vec2(0.0, 3.0)) {
            Some(Hit::Edge(x, y)) => assert_eq!((x.min(y), x.max(y)), (a.min(top), a.max(top))),
            h => panic!("{h:?}"),
        }
        // the middle of the top face
        match hit_test(&cage, 1.0, view, cam.project([0.0, 0.0, 10.0]).unwrap()) {
            Some(Hit::Face(f)) => assert!(cage.faces[f]
                .iter()
                .all(|v| cage.verts[*v as usize][2] > 0.0)),
            h => panic!("{h:?}"),
        }
        // outside the cage
        assert_eq!(hit_test(&cage, 1.0, view, pos2(10.0, 10.0)), None);
    });
    // the scale multiplies the cage: a cage of 10 at scale 2 is where a cage of 20 is
    let small = form::primitive(0, [10.0, 10.0, 10.0], 8);
    with_view(&cam, |view| {
        let top = at(&small, 5.0, 5.0, 5.0);
        let p = cam.project([10.0, 10.0, 10.0]).unwrap();
        assert_eq!(hit_test(&small, 2.0, view, p), Some(Hit::Vertex(top)));
    });
}

#[test]
fn what_a_hit_selects_and_shift_toggles() {
    let cage = cube();
    let mut fe = FormEdit::default();
    let v = at(&cage, 10.0, 10.0, 10.0);
    fe.select(&cage, Hit::Vertex(v), false);
    assert_eq!(fe.selected, BTreeSet::from([v]));
    // an edge selects its two ends, a face its four corners
    fe.select(&cage, Hit::Face(0), false);
    assert_eq!(fe.selected.len(), 4);
    let (a, b) = cage.edges()[0];
    fe.select(&cage, Hit::Edge(a, b), false);
    assert_eq!(fe.selected, BTreeSet::from([a, b]));
    // shift adds, and takes away what is all selected already
    fe.select(&cage, Hit::Vertex(v), true);
    assert!(fe.selected.contains(&v) && fe.selected.len() == 3);
    fe.select(&cage, Hit::Vertex(v), true);
    assert_eq!(fe.selected.len(), 2);
    // the edges between selected vertices
    assert_eq!(
        selected_edges(&cage, &fe.selected),
        vec![(a.min(b), a.max(b))]
    );
}

#[test]
fn a_box_selects_the_vertices_inside_it() {
    let cage = cube();
    let cam = Cam::top();
    with_view(&cam, |view| {
        // the right half seen from above holds four corners (two overlapped pairs)
        let r = Rect::from_min_max(pos2(401.0, 190.0), pos2(520.0, 410.0));
        let got = box_select(&cage, 1.0, view, r);
        assert_eq!(got.len(), 4);
        assert!(got.iter().all(|v| cage.verts[*v as usize][0] > 0.0));
        let all = box_select(
            &cage,
            1.0,
            view,
            Rect::from_min_max(pos2(0.0, 0.0), pos2(800.0, 600.0)),
        );
        assert_eq!(all.len(), 8);
        assert!(box_select(
            &cage,
            1.0,
            view,
            Rect::from_min_max(pos2(0.0, 0.0), pos2(50.0, 50.0))
        )
        .is_empty());
    });
}

fn drag_of(cage: &SubdMesh, sel: &[u32], primary: u32, forward: P3) -> Drag {
    let start: Vec<(u32, P3)> = sel.iter().map(|&v| (v, cage.verts[v as usize])).collect();
    Drag {
        grab: cage.verts[primary as usize],
        forward,
        start,
        primary,
        before: cage.clone(),
    }
}

fn close(a: P3, b: P3) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9)
}

#[test]
fn a_drag_slides_in_the_camera_plane_along_an_axis_or_in_a_plane() {
    let cage = cube();
    let v = at(&cage, 10.0, 10.0, 10.0);
    // looking straight down
    let cam = Cam::top();
    let d = drag_of(&cage, &[v], v, cam.forward);
    let ray_to = |dx: f32, dy: f32| {
        cam.ray(cam.project([10.0, 10.0, 10.0]).unwrap() + eframe::egui::vec2(dx, dy))
    };
    // free: 50 px right is 5 mm in x, 30 px down 3 mm toward -y
    let delta = drag_delta(&d, Lock::Free, ray_to(50.0, 30.0)).unwrap();
    assert!(close(delta, [5.0, -3.0, 0.0]), "{delta:?}");
    // locked to x: only x follows
    let delta = drag_delta(&d, Lock::Axis(0), ray_to(50.0, 30.0)).unwrap();
    assert!(close(delta, [5.0, 0.0, 0.0]), "{delta:?}");
    // the z axis points at the eye: nothing to follow
    assert!(drag_delta(&d, Lock::Axis(2), ray_to(50.0, 30.0)).is_none());
    // the plane across z: as free from above
    let delta = drag_delta(&d, Lock::Plane(2), ray_to(50.0, 30.0)).unwrap();
    assert!(close(delta, [5.0, -3.0, 0.0]), "{delta:?}");
    // the plane across x is edge-on from above
    assert!(drag_delta(&d, Lock::Plane(0), ray_to(50.0, 30.0)).is_none());

    // from an oblique camera the z axis can be followed, and the vertex ends up under the pointer
    let cam = Cam::looking([0.5, 1.0, -0.8]);
    let d = drag_of(&cage, &[v], v, cam.forward);
    let from = cam.project([10.0, 10.0, 10.0]).unwrap();
    let want = [10.0, 10.0, 13.0];
    let target = cam.project(want).unwrap();
    let delta = drag_delta(&d, Lock::Axis(2), cam.ray(target)).unwrap();
    // (the screen position is a float32, so the answer is good to about a micron)
    assert!(
        delta[0].abs() < 1e-4 && delta[1].abs() < 1e-4 && (delta[2] - 3.0).abs() < 1e-4,
        "{delta:?}"
    );
    // free in the camera plane: the new point projects where the pointer is
    let delta = drag_delta(
        &d,
        Lock::Free,
        cam.ray(from + eframe::egui::vec2(40.0, -25.0)),
    )
    .unwrap();
    let moved = cam
        .project([10.0 + delta[0], 10.0 + delta[1], 10.0 + delta[2]])
        .unwrap();
    assert!((moved - (from + eframe::egui::vec2(40.0, -25.0))).length() < 1e-3);
    assert!(
        dot(delta, cam.forward).abs() < 1e-9,
        "stays in the plane facing the camera"
    );
}

#[test]
fn the_grabbed_vertex_snaps_to_the_grid_on_the_axes_the_lock_leaves_free() {
    let cage = cube();
    let v = at(&cage, 10.0, 10.0, 10.0);
    let d = drag_of(&cage, &[v], v, [0.0, 0.0, -1.0]);
    let s = snap_delta(&d, 1.0, Lock::Free, 2.0, [4.9, -3.2, 0.0]);
    // 14.9 -> 14, 6.8 -> 6, z stays on its grid line
    assert!(close(s, [4.0, -4.0, 0.0]), "{s:?}");
    // an x lock leaves y and z alone
    let s = snap_delta(&d, 1.0, Lock::Axis(0), 2.0, [4.9, -3.2, 0.7]);
    assert!(close(s, [4.0, -3.2, 0.7]), "{s:?}");
    // a plane lock leaves its normal alone
    let s = snap_delta(&d, 1.0, Lock::Plane(2), 5.0, [1.0, 1.0, 0.7]);
    assert!(close(s, [0.0, 0.0, 0.7]), "{s:?}");
    // off: untouched; the scale is accounted for (a cage of 10 at scale 2: the vertex is at 10 mm)
    assert_eq!(
        snap_delta(&d, 1.0, Lock::Free, 0.0, [0.3, 0.3, 0.3]),
        [0.3, 0.3, 0.3]
    );
    let small = form::primitive(0, [10.0, 10.0, 10.0], 8);
    let w = at(&small, 5.0, 5.0, 5.0);
    let d2 = drag_of(&small, &[w], w, [0.0, 0.0, -1.0]);
    let s = snap_delta(&d2, 2.0, Lock::Axis(0), 4.0, [3.0, 0.0, 0.0]);
    assert!(close(s, [2.0, 0.0, 0.0]), "{s:?}"); // 10 + 3 = 13 -> 12
}

#[test]
fn the_selection_moves_together_and_the_cage_stays_valid() {
    let mut cage = cube();
    let top: Vec<u32> = (0..8u32)
        .filter(|v| cage.verts[*v as usize][2] > 0.0)
        .collect();
    assert_eq!(top.len(), 4);
    let d = drag_of(&cage, &top, top[0], [0.0, 0.0, -1.0]);
    apply_drag(&mut cage, &d, 1.0, [0.0, 0.0, 10.0], None);
    for v in 0..8usize {
        let z = d.before.verts[v][2];
        assert_eq!(cage.verts[v][2], if z > 0.0 { 20.0 } else { z });
        assert_eq!(cage.verts[v][0], d.before.verts[v][0]);
    }
    cage.validate().unwrap();
    // a sharp-edged box dragged taller finishes to the taller box
    let f = finish(
        &cage,
        &FinishOptions {
            tolerance: 0.01,
            ..Default::default()
        },
    )
    .unwrap();
    let vol = f.rep.volume(f.body).unwrap();
    assert!((vol - 20.0 * 20.0 * 30.0).abs() < 0.01 * 12000.0, "{vol}");
    // with a scale, the same drag in mm moves the cage by less
    let mut cage = form::primitive(0, [10.0, 10.0, 10.0], 8);
    let top: Vec<u32> = (0..8u32)
        .filter(|v| cage.verts[*v as usize][2] > 0.0)
        .collect();
    let d = drag_of(&cage, &top, top[0], [0.0, 0.0, -1.0]);
    apply_drag(&mut cage, &d, 2.0, [0.0, 0.0, 10.0], None);
    assert_eq!(cage.verts[top[0] as usize][2], 10.0); // 5 + 10/2
}

#[test]
fn mirror_symmetry_moves_the_partner_and_keeps_the_plane_vertices_on_the_plane() {
    let cage = cube();
    let partners = mirror_partners(&cage, 0, 1e-6);
    for v in 0..8usize {
        let w = partners[v].unwrap() as usize;
        assert_ne!(w, v, "a cube has no vertex on the plane x = 0");
        assert_eq!(cage.verts[w][0], -cage.verts[v][0]);
        assert_eq!(cage.verts[w][1], cage.verts[v][1]);
    }
    // an unpaired vertex has no partner
    let mut odd = cage.clone();
    odd.verts[0][1] += 1.0;
    assert_eq!(mirror_partners(&odd, 0, 1e-6)[0], None);

    // drag one vertex out and up: its partner follows as the mirror image
    let mut c = cage.clone();
    let v = at(&c, 10.0, 10.0, 10.0);
    let w = at(&c, -10.0, 10.0, 10.0);
    let d = drag_of(&c, &[v], v, [0.0, 0.0, -1.0]);
    apply_drag(&mut c, &d, 1.0, [3.0, 2.0, 4.0], Some(0));
    assert_eq!(c.verts[v as usize], [13.0, 12.0, 14.0]);
    assert_eq!(c.verts[w as usize], [-13.0, 12.0, 14.0]);
    // both of a pair selected: they stay mirrored, not both moved the same way
    let mut c = cage.clone();
    let d = drag_of(&c, &[v, w], v, [0.0, 0.0, -1.0]);
    apply_drag(&mut c, &d, 1.0, [3.0, 0.0, 0.0], Some(0));
    assert_eq!(c.verts[v as usize][0], 13.0);
    assert_eq!(c.verts[w as usize][0], -13.0);
    // without symmetry the partner stays
    let mut c = cage.clone();
    let d = drag_of(&c, &[v], v, [0.0, 0.0, -1.0]);
    apply_drag(&mut c, &d, 1.0, [3.0, 0.0, 0.0], None);
    assert_eq!(c.verts[w as usize], [-10.0, 10.0, 10.0]);
    // a vertex on the plane stays on it
    let mut p = form::primitive(1, [20.0, 20.0, 30.0], 8); // a prism with vertices on x = 0? (no: ring of 8)
    let on: Vec<u32> = (0..p.verts.len() as u32)
        .filter(|v| p.verts[*v as usize][0].abs() < 1e-9)
        .collect();
    assert!(!on.is_empty());
    let d = drag_of(&p, &on[..1], on[0], [0.0, 0.0, -1.0]);
    apply_drag(&mut p, &d, 1.0, [4.0, 0.0, 0.0], Some(0));
    assert_eq!(p.verts[on[0] as usize][0], 0.0);
}

#[test]
fn the_cage_has_its_own_undo_and_redo() {
    let cage = cube();
    let mut fe = FormEdit::default();
    assert!(!fe.sync(&cage), "the first look only notes the cage");
    let mut c1 = cage.clone();
    c1.verts[0][0] += 1.0;
    assert!(fe.sync(&c1));
    assert!(!fe.sync(&c1), "nothing changed");
    let mut c2 = c1.clone();
    c2.set_crease(0, 1, 0.0);
    c2.verts[1][2] -= 2.0;
    assert!(fe.sync(&c2));
    assert_eq!(fe.undo.len(), 2);
    // while a drag is going on, intermediate cages are not steps
    fe.begin_drag(&c2, 0, [0.0; 3], [0.0, 0.0, -1.0]);
    assert!(fe.drag.is_none(), "nothing selected, nothing to drag");
    fe.selected.insert(0);
    fe.begin_drag(&c2, 0, c2.verts[0], [0.0, 0.0, -1.0]);
    assert!(fe.drag.is_some());
    let mut moving = c2.clone();
    moving.verts[0][0] += 5.0;
    assert!(!fe.sync(&moving));
    fe.end_drag();
    assert!(fe.sync(&moving));
    assert_eq!(fe.undo.len(), 3);
    // undo walks back through them, redo forward
    let back = fe.undo(&moving).unwrap();
    assert_eq!(back, c2);
    let back = fe.undo(&back).unwrap();
    assert_eq!(back, c1);
    let back = fe.undo(&back).unwrap();
    assert_eq!(back, cage);
    assert!(fe.undo(&back).is_none());
    let fwd = fe.redo(&back).unwrap();
    assert_eq!(fwd, c1);
    // a new edit after undo drops the redo stack
    let mut c3 = fwd.clone();
    c3.verts[2][1] += 3.0;
    assert!(fe.sync(&c3));
    assert!(fe.redo.is_empty());
    // cancelling a drag puts the cage back
    let mut live = c3.clone();
    fe.selected = BTreeSet::from([2]);
    fe.begin_drag(&live, 2, live.verts[2], [0.0, 0.0, -1.0]);
    live.verts[2][0] = 99.0;
    fe.cancel_drag(&mut live);
    assert_eq!(live, c3);
}

#[test]
fn x_y_z_keys_lock_the_drag_and_pressing_again_frees_it() {
    let mut fe = FormEdit::default();
    assert_eq!(fe.lock, Lock::Free);
    fe.key_lock(0, false);
    assert_eq!(fe.lock, Lock::Axis(0));
    fe.key_lock(1, false);
    assert_eq!(fe.lock, Lock::Axis(1));
    fe.key_lock(1, true);
    assert_eq!(fe.lock, Lock::Plane(1));
    fe.key_lock(1, true);
    assert_eq!(fe.lock, Lock::Free);
}

#[test]
fn a_whole_drag_through_the_state_machine() {
    let mut cage = cube();
    let mut fe = FormEdit::default();
    let cam = Cam::looking([0.3, 1.0, -0.6]);
    let v = at(&cage, 10.0, 10.0, 10.0);
    with_view(&cam, |view| {
        let pos = cam.project([10.0, 10.0, 10.0]).unwrap();
        let hit = hit_test(&cage, 1.0, view, pos).unwrap();
        fe.select(&cage, hit, false);
        assert_eq!(fe.selected, BTreeSet::from([v]));
        fe.begin_drag(&cage, v, [10.0, 10.0, 10.0], cam.forward);
        fe.lock = Lock::Axis(2);
        fe.snap = Some(1.0);
        let target = cam.project([10.0, 10.0, 12.4]).unwrap();
        assert!(fe.update_drag(&mut cage, 1.0, cam.ray(target)));
        assert!(
            (cage.verts[v as usize][2] - 12.0).abs() < 1e-6,
            "snapped to the grid: {:?}",
            cage.verts[v as usize]
        );
        assert_eq!(cage.verts[v as usize][0], 10.0);
    });
    cage.validate().unwrap();
}
