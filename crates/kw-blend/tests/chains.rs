//! Tangent chains: a fillet or chamfer follows the edges that continue each other tangentially (the top
//! outline of a rounded-rectangle block: straight edges and corner arcs) and ends exactly at each
//! junction.
#![allow(clippy::too_many_arguments)]
use kw_blend::*;
use kw_boolean::{boolean, BoolOptions, Op};
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::{extrude, ExtrudeOptions, Profile, ProfileEdge, ProfileLoop};
use std::f64::consts::PI;

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn vol(rep: &BRep, b: BodyId) -> f64 {
    rep.mass_properties(b).unwrap().volume
}

/// Edges of the body whose two end points (and midpoint) lie at height `z`.
fn edges_at_z(rep: &BRep, body: BodyId, z: f64) -> Vec<EdgeId> {
    use kw_nurbs::Curve3;
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !seen.insert(e) {
                    continue;
                }
                let ed = &rep.edges[e];
                let Some(c) = ed.curve.and_then(|c| rep.curves.get(c)) else {
                    continue;
                };
                let (t0, t1) = ed.param_range;
                let at = |t: f64| (c.point_at(t).z - z).abs() < 1e-9;
                if at(t0) && at(t1) && at(0.5 * (t0 + t1)) {
                    out.push(e);
                }
            }
        }
    }
    out
}

fn vertical_edges(rep: &BRep, body: BodyId) -> Vec<(EdgeId, Point3<f64>)> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                if !seen.insert(e) {
                    continue;
                }
                let ed = &rep.edges[e];
                let (a, b) = (rep.vertices[ed.start].point, rep.vertices[ed.end].point);
                if (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() > 1e-6
                {
                    out.push((e, Point3::new(a.x, a.y, 0.0)));
                }
            }
        }
    }
    out
}

fn assert_valid(rep: &BRep, b: BodyId, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let t = kw_tess::tessellate_body(
        rep,
        b,
        &kw_tess::TessOptions {
            chord_tol: 0.004,
            angle_tol: 0.2,
        },
    )
    .unwrap_or_else(|e| panic!("{what}: tess {e:?}"));
    let v = vol(rep, b);
    assert!(
        (t.mesh.signed_volume() - v).abs() < 0.02 * v,
        "{what}: mesh volume {} vs {v}",
        t.mesh.signed_volume()
    );
}

/// Rounded-rectangle block `w × d × h` with corner radius `rc`.
fn rounded_block(rep: &mut BRep, w: f64, d: f64, h: f64, rc: f64) -> BodyId {
    let b = make_box(rep, &world(), w, d, h).body;
    let verts = vertical_edges(rep, b);
    assert_eq!(verts.len(), 4);
    let es: Vec<EdgeId> = verts.iter().map(|v| v.0).collect();
    fillet(rep, b, &es, rc).expect("rounding the corners").body
}

/// Cross-section area of a fillet of radius `r` and the distance of its centroid from the edge's faces.
fn spandrel(r: f64) -> (f64, f64) {
    let a = r * r * (1.0 - PI / 4.0);
    let xbar = r * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
    (a, xbar)
}

#[test]
fn fillet_follows_a_closed_tangent_chain() {
    let mut rep = BRep::new();
    let (w, d, h, rc, r) = (10.0, 6.0, 4.0, 1.5, 0.5);
    let b = rounded_block(&mut rep, w, d, h, rc);
    let v0 = vol(&rep, b);
    let top = edges_at_z(&rep, b, 0.5 * h);
    assert_eq!(top.len(), 8, "four lines and four corner arcs");
    // select ONE edge only: the whole chain follows
    let res = fillet(&mut rep, b, &top[..1], r)
        .expect("chain fillet")
        .body;
    assert_valid(&rep, res, "rounded block, top outline");
    let (a, xbar) = spandrel(r);
    let straight = 2.0 * (w - 2.0 * rc) + 2.0 * (d - 2.0 * rc);
    let arcs = 2.0 * PI * (rc - xbar);
    let expect = v0 - a * (straight + arcs);
    assert!(
        (vol(&rep, res) - expect).abs() < 1e-6 * expect,
        "volume {} vs Pappus {expect}",
        vol(&rep, res)
    );
    // 8 blend faces: 4 cylinders, 4 tori, all generated from the chain's edges
    let blends = rep
        .body_faces(res)
        .into_iter()
        .filter(|f| {
            matches!(
                rep.surfaces[rep.faces[*f].surface],
                kw_nurbs::SurfaceGeom::Torus(_)
            )
        })
        .count();
    assert_eq!(blends, 4);
}

#[test]
fn chamfer_follows_a_closed_tangent_chain() {
    let mut rep = BRep::new();
    let (w, d, h, rc, c) = (10.0, 6.0, 4.0, 1.5, 0.5);
    let b = rounded_block(&mut rep, w, d, h, rc);
    let v0 = vol(&rep, b);
    let top = edges_at_z(&rep, b, 0.5 * h);
    let res = chamfer(&mut rep, b, &top[..1], c, c)
        .expect("chain chamfer")
        .body;
    assert_valid(&rep, res, "rounded block, chamfered outline");
    // triangle area c²/2, centroid c/3 from each face
    let a = 0.5 * c * c;
    let straight = 2.0 * (w - 2.0 * rc) + 2.0 * (d - 2.0 * rc);
    let arcs = 2.0 * PI * (rc - c / 3.0);
    let expect = v0 - a * (straight + arcs);
    assert!(
        (vol(&rep, res) - expect).abs() < 1e-6 * expect,
        "volume {} vs Pappus {expect}",
        vol(&rep, res)
    );
}

#[test]
fn unequal_chamfer_keeps_its_first_distance_on_one_side_of_the_chain() {
    // distance d1 on the top face, d2 down the sides, all the way round the rounded outline
    let mut rep = BRep::new();
    let (w, d, h, rc) = (10.0, 6.0, 4.0, 1.5);
    let b = rounded_block(&mut rep, w, d, h, rc);
    let v0 = vol(&rep, b);
    let top = edges_at_z(&rep, b, 2.0);
    let (d_top, d_side) = (0.6, 0.35);
    let mut volumes = Vec::new();
    // whichever edge of the chain is named, the same chamfer comes out
    for e in &top {
        let mut r2 = rep.clone();
        let res = chamfer(&mut r2, b, &[*e], d_top, d_side);
        let res = res
            .unwrap_or_else(|er| panic!("chamfer from {e:?}: {er}"))
            .body;
        assert!(r2.validate().is_ok());
        volumes.push(vol(&r2, res));
    }
    // The first distance goes on the first face of the edge that is named, and stays on the same face
    // of the chain all the way round: the chamfer is one of two, whichever edge is named. Triangle with
    // legs a (on the top face) and b (down the wall): area a b / 2, centroid a / 3 inside the wall.
    let tri = 0.5 * d_top * d_side;
    let straight = 2.0 * (w - 2.0 * rc) + 2.0 * (d - 2.0 * rc);
    let pappus = |a: f64| v0 - tri * (straight + 2.0 * PI * (rc - a / 3.0));
    let (first_on_top, first_on_side) = (pappus(d_top), pappus(d_side));
    for v in &volumes {
        assert!(
            (v - first_on_top).abs() < 1e-6 * v0 || (v - first_on_side).abs() < 1e-6 * v0,
            "volume {v} is neither {first_on_top} nor {first_on_side}: {volumes:?}"
        );
    }
    assert!(volumes.iter().any(|v| (v - first_on_top).abs() < 1e-6 * v0));
    assert!(volumes
        .iter()
        .any(|v| (v - first_on_side).abs() < 1e-6 * v0));
}

#[test]
fn fillet_of_a_pocket_chain_adds_material() {
    // a rounded-rectangle pocket cut into a plate: its rim is a concave tangent chain at the floor
    let mut rep = BRep::new();
    let plate = make_box(&mut rep, &world(), 20.0, 14.0, 5.0).body;
    // plate z ∈ [-2.5, 2.5]; the tool block reaches from z = -2 up through the top
    let tool_frame = Frame3 {
        origin: Point3::new(0.0, 0.0, 0.5),
        ..world()
    };
    let blk = make_box(&mut rep, &tool_frame, 10.0, 6.0, 5.0).body;
    let verts = vertical_edges(&rep, blk);
    let es: Vec<EdgeId> = verts.iter().map(|v| v.0).collect();
    let blk = fillet(&mut rep, blk, &es, 1.5).unwrap().body;
    let pocket = boolean(&mut rep, plate, blk, Op::Subtract, &BoolOptions::default())
        .unwrap()
        .body;
    let v0 = vol(&rep, pocket);
    let floor = edges_at_z(&rep, pocket, -2.0);
    assert_eq!(floor.len(), 8, "pocket floor outline");
    let r = 0.5;
    let res = fillet(&mut rep, pocket, &floor[..1], r)
        .expect("pocket floor fillet")
        .body;
    assert_valid(&rep, res, "pocket with filleted floor");
    let (a, xbar) = spandrel(r);
    // concave: material is added inside the pocket outline, next to the wall (radius rc - xbar)
    let straight = 2.0 * (10.0 - 3.0) + 2.0 * (6.0 - 3.0);
    let arcs = 2.0 * PI * (1.5 - xbar);
    let expect = v0 + a * (straight + arcs);
    assert!(
        (vol(&rep, res) - expect).abs() < 1e-6 * expect,
        "volume {} vs Pappus {expect}",
        vol(&rep, res)
    );
}

#[test]
fn fillet_follows_an_open_chain_to_a_sharp_corner() {
    // two neighbouring vertical edges rounded: the chain top-line / arc / top-line / arc / top-line runs
    // into the two sharp corners, where the fillets end at the mitre
    let mut rep = BRep::new();
    let (w, d, h, rc, r) = (10.0, 6.0, 4.0, 1.5, 0.4);
    let b = make_box(&mut rep, &world(), w, d, h).body;
    let verts = vertical_edges(&rep, b);
    let two: Vec<EdgeId> = verts
        .iter()
        .filter(|(_, p)| p.x > 0.5 * w - 1e-6)
        .map(|v| v.0)
        .collect();
    assert_eq!(two.len(), 2);
    let b = fillet(&mut rep, b, &two, rc).unwrap().body;
    let v0 = vol(&rep, b);
    let top = edges_at_z(&rep, b, 0.5 * h);
    assert_eq!(top.len(), 6);
    // the tangent chain (line / arc / line / arc / line) follows from one edge; the sharp corners need the
    // edges on both sides selected
    let res = fillet(&mut rep, b, &top, r).expect("open chain").body;
    assert_valid(&rep, res, "open chain");
    // the whole outline except the mitre overlaps at the two sharp corners
    let (a, xbar) = spandrel(r);
    let straight = w - rc + (d - 2.0 * rc) + (w - rc) + 2.0 * d * 0.0 + 2.0 * 0.0;
    let _ = straight;
    let perimeter = 2.0 * (w - rc) + (d - 2.0 * rc) + 2.0 * d * 0.0 + w * 0.0;
    let _ = perimeter;
    // outline: bottom edge (w - rc) + arc + right edge (d - 2rc) + arc + top edge (w - rc) + left edge d
    let lines = (w - rc) + (d - 2.0 * rc) + (w - rc) + d;
    let arcs = PI * (rc - xbar);
    let upper = v0 - a * (lines + arcs);
    let removed = v0 - vol(&rep, res);
    // the two sharp-corner mitres each count one overlap of at most r³
    assert!(
        removed <= a * (lines + arcs) + 1e-9 && removed >= a * (lines + arcs) - 2.0 * r * r * r,
        "removed {removed} vs {} (upper {upper})",
        a * (lines + arcs)
    );
}

#[test]
fn an_open_chain_ends_cleanly_at_the_end_faces() {
    // rounded on the +x side only; fillet the chain bottom line / arc / right line / arc / top line starting
    // from the right line. Its two ends run into the plane x = -w/2 (perpendicular): no mitre, exact Pappus.
    use kw_nurbs::Curve3;
    let mut rep = BRep::new();
    let (w, d, h, rc, r) = (10.0, 6.0, 4.0, 1.5, 0.4);
    let b = make_box(&mut rep, &world(), w, d, h).body;
    let two: Vec<EdgeId> = vertical_edges(&rep, b)
        .iter()
        .filter(|(_, p)| p.x > 0.5 * w - 1e-6)
        .map(|v| v.0)
        .collect();
    let b = fillet(&mut rep, b, &two, rc).unwrap().body;
    let v0 = vol(&rep, b);
    let right = edges_at_z(&rep, b, 0.5 * h)
        .into_iter()
        .find(|e| {
            let ed = &rep.edges[*e];
            let c = rep.curves[ed.curve.unwrap()].clone();
            let m = c.point_at(0.5 * (ed.param_range.0 + ed.param_range.1));
            (m.x - 0.5 * w).abs() < 1e-9 && m.y.abs() < 1e-9
        })
        .expect("the straight edge on the +x side");
    let res = fillet(&mut rep, b, &[right], r).expect("open chain").body;
    assert_valid(&rep, res, "open chain between end faces");
    let (a, xbar) = spandrel(r);
    let lines = (w - rc) + (d - 2.0 * rc) + (w - rc);
    let arcs = PI * (rc - xbar);
    let expect = v0 - a * (lines + arcs);
    assert!(
        (vol(&rep, res) - expect).abs() < 1e-6 * expect,
        "volume {} vs Pappus {expect}",
        vol(&rep, res)
    );
}

/// A box and a half-disc prism glued at a common cap, the round side tangent to the box's sides along the
/// cap edge: the junction of two chain tools.
#[test]
fn tangent_continuous_prisms_glued_at_a_cap_unite() {
    {
        let op = Op::Union;
        let mut rep = BRep::new();
        // box x∈[-2,0], y∈[-1,1], z∈[0,1]
        let a = extrude(
            &mut rep,
            &Profile::polygon(
                world(),
                &[(-2.0, -1.0), (0.0, -1.0), (0.0, 1.0), (-2.0, 1.0)],
            ),
            Vector3::z(),
            &ExtrudeOptions::default(),
        )
        .unwrap()
        .body;
        // half disc x≥0 radius 1
        let arc = ProfileEdge::arc(
            &Frame3 {
                origin: Point3::origin(),
                ..world()
            },
            1.0,
            -PI / 2.0,
            PI / 2.0,
        );
        let l = ProfileEdge::line(Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, -1.0, 0.0));
        let b = extrude(
            &mut rep,
            &Profile::new(world(), ProfileLoop::new(vec![arc, l])),
            Vector3::z(),
            &ExtrudeOptions::default(),
        )
        .unwrap()
        .body;
        let r = boolean(&mut rep, a, b, op, &BoolOptions::default());
        let v = rep.volume(r.expect("union of glued prisms").body).unwrap();
        assert!((v - (4.0 + PI / 2.0)).abs() < 1e-9, "volume {v}");
    }
}

#[test]
fn fillet_around_a_slot_shaped_plate_with_half_circle_ends() {
    // stadium: two lines and two half-circle arcs (each a 180° partial rim edge)
    let mut rep = BRep::new();
    let (h, r) = (2.0, 0.3);
    let at = |x: f64, y: f64| Frame3 {
        origin: Point3::new(x, y, 0.0),
        ..world()
    };
    let loop_ = ProfileLoop::new(vec![
        ProfileEdge::line(Point3::new(-3.0, -1.0, 0.0), Point3::new(3.0, -1.0, 0.0)),
        ProfileEdge::arc(&at(3.0, 0.0), 1.0, -PI / 2.0, PI / 2.0),
        ProfileEdge::line(Point3::new(3.0, 1.0, 0.0), Point3::new(-3.0, 1.0, 0.0)),
        ProfileEdge::arc(&at(-3.0, 0.0), 1.0, PI / 2.0, 3.0 * PI / 2.0),
    ]);
    let b = extrude(
        &mut rep,
        &Profile::new(world(), loop_),
        Vector3::new(0.0, 0.0, h),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    let v0 = vol(&rep, b);
    let top = edges_at_z(&rep, b, h);
    assert_eq!(top.len(), 4);
    let res = fillet(&mut rep, b, &top[..1], r)
        .expect("slot outline")
        .body;
    assert_valid(&rep, res, "slot outline");
    let (a, xbar) = spandrel(r);
    let expect = v0 - a * (2.0 * 6.0 + 2.0 * PI * (1.0 - xbar));
    assert!(
        (vol(&rep, res) - expect).abs() < 1e-6 * expect,
        "volume {} vs Pappus {expect}",
        vol(&rep, res)
    );
}

#[test]
fn fillet_around_an_l_shaped_plate_with_convex_and_concave_corner_arcs() {
    for at_top in [true, false] {
        let mut rep = BRep::new();
        let (h, rc, r) = (2.0, 1.0, 0.4);
        let pts = [
            (-5.0, -5.0),
            (5.0, -5.0),
            (5.0, -1.0),
            (-1.0, -1.0),
            (-1.0, 5.0),
            (-5.0, 5.0),
        ];
        let b = extrude(
            &mut rep,
            &Profile::polygon(world(), &pts),
            Vector3::new(0.0, 0.0, h),
            &ExtrudeOptions::default(),
        )
        .unwrap()
        .body;
        let verts = vertical_edges(&rep, b);
        assert_eq!(verts.len(), 6);
        let es: Vec<EdgeId> = verts.iter().map(|v| v.0).collect();
        let b = fillet(&mut rep, b, &es, rc)
            .expect("round the corners")
            .body;
        let v0 = vol(&rep, b);
        let top = edges_at_z(&rep, b, if at_top { h } else { 0.0 });
        for e in &top {
            let ed = &rep.edges[*e];
            eprintln!(
                "EDGE {:?} {:?} -> {:?} {:?}",
                e,
                rep.vertices[ed.start].point,
                rep.vertices[ed.end].point,
                ed.curve.map(|c| std::mem::discriminant(&rep.curves[c]))
            );
        }
        assert_eq!(top.len(), 12);
        let res = fillet(&mut rep, b, &top[..1], r).expect("L outline").body;
        assert_valid(&rep, res, "L outline");
        let (a, xbar) = spandrel(r);
        let lines = 40.0 - 6.0 * 2.0 * rc;
        let arcs = 5.0 * (PI / 2.0) * (rc - xbar) + (PI / 2.0) * (rc + xbar);
        let expect = v0 - a * (lines + arcs);
        assert!(
            (vol(&rep, res) - expect).abs() < 1e-6 * expect,
            "volume {} vs Pappus {expect}",
            vol(&rep, res)
        );
    }
}

#[test]
fn tangent_chain_lists_the_run_of_a_rounded_outline() {
    let mut rep = BRep::new();
    let b = rounded_block(&mut rep, 10.0, 6.0, 4.0, 1.5);
    let top = edges_at_z(&rep, b, 2.0);
    let chain = tangent_chain(&rep, b, &top[..1]);
    assert_eq!(chain.len(), 8);
    for e in &top {
        assert!(chain.contains(e));
    }
    // a sharp-cornered edge has no tangent neighbour
    let plain = make_box(&mut rep, &world(), 10.0, 6.0, 4.0).body;
    let e0 = edges_at_z(&rep, plain, 2.0)[0];
    assert_eq!(tangent_chain(&rep, plain, &[e0]), vec![e0]);
}

#[test]
fn l_shaped_plates_of_various_proportions_blend_along_their_outline() {
    // proportions that used to trip the numerics of a union of the chain's tools; each result is checked
    // against the exact volume (Pappus)
    for (rc, r) in [(0.4447, 0.173), (0.4447, 0.1), (0.6, 0.2), (0.5, 0.2)] {
        for (a, w) in [(5.0, 2.2), (4.3, 2.7), (6.0, 1.9)] {
            for at_top in [true, false] {
                let mut rep = BRep::new();
                let h = 1.5;
                let pts = [(0.0, 0.0), (a, 0.0), (a, w), (w, w), (w, a), (0.0, a)];
                let b = extrude(
                    &mut rep,
                    &Profile::polygon(world(), &pts),
                    Vector3::new(0.0, 0.0, h),
                    &ExtrudeOptions::default(),
                )
                .unwrap()
                .body;
                let es: Vec<EdgeId> = vertical_edges(&rep, b).iter().map(|v| v.0).collect();
                let b = fillet(&mut rep, b, &es, rc).unwrap().body;
                let v0 = vol(&rep, b);
                let outline = edges_at_z(&rep, b, if at_top { h } else { 0.0 });
                let res = fillet(&mut rep, b, &outline[..1], r)
                    .unwrap_or_else(|e| panic!("rc {rc} r {r} a {a} w {w}: {e}"))
                    .body;
                assert_valid(&rep, res, "L outline");
                let (area, xbar) = spandrel(r);
                let lines = 4.0 * a - 12.0 * rc;
                let arcs = 5.0 * (PI / 2.0) * (rc - xbar) + (PI / 2.0) * (rc + xbar);
                let expect = v0 - area * (lines + arcs);
                assert!(
                    (vol(&rep, res) - expect).abs() < 1e-6 * expect,
                    "rc {rc} r {r} a {a} w {w}: volume {} vs Pappus {expect}",
                    vol(&rep, res)
                );
            }
        }
    }
}
