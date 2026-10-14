//! The quick preview of a cage, and the edge helpers the viewport editing uses.

use kw_subd::*;
use std::collections::HashMap;

fn volume(m: &PreviewMesh) -> f64 {
    m.tris
        .iter()
        .map(|t| {
            let (a, b, c) = (
                m.positions[t[0] as usize],
                m.positions[t[1] as usize],
                m.positions[t[2] as usize],
            );
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

fn closed(m: &PreviewMesh) -> bool {
    let mut he: HashMap<(u32, u32), u32> = HashMap::new();
    for t in &m.tris {
        for i in 0..3 {
            *he.entry((t[i], t[(i + 1) % 3])).or_default() += 1;
        }
    }
    he.iter()
        .all(|(&(a, b), &n)| n == 1 && he.get(&(b, a)) == Some(&1))
}

fn bounds(m: &PreviewMesh) -> ([f64; 3], [f64; 3]) {
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for p in &m.positions {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

#[test]
fn creases_are_set_replaced_read_and_removed_per_edge() {
    let mut b = SubdMesh::box_(2.0, 2.0, 2.0);
    let edges = b.edges();
    assert_eq!(edges.len(), 12);
    assert!(edges.iter().all(|(a, c)| a < c));
    assert_eq!(b.crease_of(0, 1), 0.0);
    b.set_crease(0, 1, 100.0);
    assert_eq!(b.crease_of(1, 0), 100.0);
    b.set_crease(1, 0, 2.0);
    assert_eq!(b.crease_of(0, 1), 2.0);
    assert_eq!(b.creases.len(), 1, "an edge has one crease");
    b.set_crease(0, 1, 0.0);
    assert!(b.creases.is_empty());
    assert_eq!(b.crease_of(0, 1), 0.0);
}

#[test]
fn the_preview_is_a_closed_surface_with_the_volume_and_extent_of_the_finished_body() {
    let shapes: Vec<(&str, SubdMesh)> = vec![
        ("cube", SubdMesh::cube(20.0)),
        (
            "sharp box",
            SubdMesh::box_(30.0, 20.0, 10.0).crease_all(100.0),
        ),
        ("torus", SubdMesh::torus(20.0, 6.0, 8, 6)),
        ("prism", SubdMesh::prism(5, 10.0, 15.0)),
    ];
    for (name, cage) in shapes {
        let levels = levels_for(&cage);
        let p = fast_mesh(&cage, 1.0, levels).unwrap();
        assert!(closed(&p), "{name}");
        let want = finish(
            &cage,
            &FinishOptions {
                tolerance: 0.02,
                ..Default::default()
            },
        )
        .unwrap();
        let v_fin = want.rep.volume(want.body).unwrap();
        let v = volume(&p);
        assert!(v > 0.0, "{name}");
        // the preview is the limit surface sampled at 2^levels per cage edge: a little inside a convex one
        assert!((v - v_fin).abs() < 0.01 * v_fin, "{name}: {v} vs {v_fin}");
        let bb = want.rep.body_bbox(want.body);
        let (lo, hi) = bounds(&p);
        for k in 0..3 {
            assert!(
                // the body's box is a little loose (patch control points), never tight
                lo[k] > bb.min[k] - 0.01
                    && lo[k] < bb.min[k] + 0.5
                    && hi[k] < bb.max[k] + 0.01
                    && hi[k] > bb.max[k] - 0.5,
                "{name} axis {k}: preview {:?}..{:?} body {:?}..{:?}",
                lo,
                hi,
                bb.min,
                bb.max
            );
        }
    }
}

#[test]
fn a_cage_turned_inside_out_previews_the_right_way_round_and_scale_applies() {
    let cage = SubdMesh::cube(10.0).flipped();
    let p = fast_mesh(&cage, 2.0, 2).unwrap();
    let outward = fast_mesh(&SubdMesh::cube(10.0), 2.0, 2).unwrap();
    assert!(volume(&p) > 0.0);
    assert!((volume(&p) - volume(&outward)).abs() < 1e-9);
    // a cube of 10 scaled by 2 spans about 20 (the rounded limit is a little smaller)
    let (lo, hi) = bounds(&p);
    assert!(hi[0] - lo[0] > 12.0 && hi[0] - lo[0] < 20.0);
    // a broken cage is explained
    let mut bad = SubdMesh::cube(10.0);
    bad.faces[0][0] = 99;
    assert!(fast_mesh(&bad, 1.0, 2).is_err());
}

#[test]
fn the_preview_is_fast_enough_to_follow_a_drag() {
    let cage = SubdMesh::torus(20.0, 6.0, 12, 6);
    let t = std::time::Instant::now();
    let n = 20;
    for _ in 0..n {
        let p = fast_mesh(&cage, 1.0, levels_for(&cage)).unwrap();
        assert!(!p.tris.is_empty());
    }
    let per = t.elapsed().as_secs_f64() / n as f64;
    // generous for a debug build; a release build takes a few milliseconds
    assert!(per < 0.25, "{per} s per preview");
}
