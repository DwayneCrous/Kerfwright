#![allow(
    unused_imports,
    dead_code,
    clippy::type_complexity,
    clippy::neg_cmp_op_on_partial_ord,
    clippy::too_many_arguments,
    clippy::needless_range_loop
)]
//! Variable-radius fillets with arbitrary radius laws, on straight edges and on closed circular rims.
use kw_blend::*;
use kw_brep::*;
use kw_math::{Frame3, Point3, Vector3};
use kw_model::*;
use kw_nurbs::{Curve3, CurveGeom};
use std::f64::consts::{PI, TAU};
use std::sync::Arc;

fn world() -> Frame3 {
    Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        z: Vector3::z(),
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn mesh_ok(rep: &BRep, body: BodyId, vol: f64, what: &str) {
    assert!(rep.validate().is_ok(), "{what}: {:?}", rep.validate());
    let t = kw_tess::tessellate_body(
        rep,
        body,
        &kw_tess::TessOptions {
            chord_tol: 0.003,
            angle_tol: 0.15,
        },
    )
    .unwrap_or_else(|e| panic!("{what}: tess {e:?}"));
    assert!(
        (t.mesh.signed_volume() - vol).abs() < 0.02 * vol,
        "{what}: mesh volume {} vs {vol}",
        t.mesh.signed_volume()
    );
}

/// Composite Simpson rule.
fn integrate(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    let mut s = f(a) + f(b);
    for i in 1..n {
        s += f(a + h * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    s * h / 3.0
}

fn z_edge(rep: &BRep, body: BodyId, x: f64, y: f64) -> EdgeId {
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                let ed = &rep.edges[e];
                if let Some(cg @ CurveGeom::Line(l)) = ed.curve.and_then(|c| rep.curves.get(c)) {
                    let tm = 0.5 * (ed.param_range.0 + ed.param_range.1);
                    let p = cg.point_at(tm);
                    if l.dir.normalize().cross(&Vector3::z()).norm() < 1e-9
                        && (p.x - x).abs() < 1e-9
                        && (p.y - y).abs() < 1e-9
                    {
                        return e;
                    }
                }
            }
        }
    }
    panic!("edge not found");
}

fn rim_at(rep: &BRep, body: BodyId, z: f64) -> EdgeId {
    for f in rep.body_faces(body) {
        for lp in &rep.faces[f].loops {
            for co in rep.loop_coedges(*lp) {
                let e = rep.coedges[co].edge;
                let ed = &rep.edges[e];
                if let Some(CurveGeom::Circle(c)) = ed.curve.and_then(|c| rep.curves.get(c)) {
                    if (c.frame.origin.z - z).abs() < 1e-9 && rep.radial_ring(e).len() == 2 {
                        return e;
                    }
                }
            }
        }
    }
    panic!("rim not found");
}

#[test]
fn nonlinear_laws_on_a_straight_edge() {
    let laws: Vec<(&str, RadiusLaw, Box<dyn Fn(f64) -> f64>)> = vec![
        (
            "sine",
            RadiusLaw::Function(Arc::new(|t| 0.3 + 0.25 * (PI * t).sin())),
            Box::new(|t| 0.3 + 0.25 * (PI * t).sin()),
        ),
        (
            "table",
            RadiusLaw::Table(vec![(0.0, 0.2), (0.3, 0.6), (0.7, 0.5), (1.0, 0.25)]),
            Box::new({
                let l = RadiusLaw::Table(vec![(0.0, 0.2), (0.3, 0.6), (0.7, 0.5), (1.0, 0.25)]);
                move |t| l.at(t)
            }),
        ),
    ];
    for (name, law, f) in laws {
        let mut rep = BRep::new();
        let b = make_box(&mut rep, &world(), 4.0, 3.0, 2.0).body;
        let e = z_edge(&rep, b, 2.0, 1.5);
        let res = fillet_law(&mut rep, b, e, &law).unwrap_or_else(|er| panic!("{name}: {er}"));
        let removed = (1.0 - PI / 4.0) * 2.0 * integrate(|t| f(t) * f(t), 0.0, 1.0, 2000);
        let want = 24.0 - removed;
        let got = rep.volume(res.body).unwrap();
        assert!(rel(got, want) < 2e-6, "{name}: {got} vs {want}");
        mesh_ok(&rep, res.body, want, name);
    }
}

#[test]
fn variable_radius_on_the_rim_of_a_cylinder() {
    // cylinder R = 2, height 3; periodic law with three lobes
    let (big_r, h) = (2.0, 3.0);
    let law_fn = |t: f64| 0.4 + 0.12 * (3.0 * TAU * t).cos();
    let law = RadiusLaw::Function(Arc::new(law_fn));
    let mut rep = BRep::new();
    let c = make_cylinder(&mut rep, &world(), big_r, h).body;
    let v0 = rep.volume(c).unwrap();
    let e = rim_at(&rep, c, h);
    let res = fillet_law(&mut rep, c, e, &law).unwrap_or_else(|er| panic!("{er}"));
    // section spandrel (right angle) at radius ρ, revolved: area ρ²(1-π/4), centroid radius below
    let removed = integrate(
        |th| {
            let rho = law_fn(th / TAU);
            rho * rho * ((big_r - rho / 2.0) - PI / 4.0 * (big_r - rho) - rho / 3.0)
        },
        0.0,
        TAU,
        4000,
    );
    let want = v0 - removed;
    let got = rep.volume(res.body).unwrap();
    assert!(rel(got, want) < 1e-6, "{got} vs {want} (removed {removed})");
    mesh_ok(&rep, res.body, want, "variable rim");
}

/// Area and radial centroid (per radian of revolution: `A * r̄`) of the 2D spandrel between the corner
/// `p = (radial, axial)` of two faces running along unit directions `u1`, `u2` away from it and a
/// fillet arc of radius `rho` tangent to both.
fn spandrel_moment(p: (f64, f64), u1: (f64, f64), u2: (f64, f64), rho: f64) -> f64 {
    let cosa = (u1.0 * u2.0 + u1.1 * u2.1).clamp(-1.0, 1.0);
    let alpha = cosa.acos();
    let t = rho / (alpha / 2.0).tan();
    let t1 = (p.0 + u1.0 * t, p.1 + u1.1 * t);
    let t2 = (p.0 + u2.0 * t, p.1 + u2.1 * t);
    let bis = (u1.0 + u2.0, u1.1 + u2.1);
    let bn = bis.0.hypot(bis.1);
    let cd = rho / (alpha / 2.0).sin();
    let c = (p.0 + bis.0 / bn * cd, p.1 + bis.1 / bn * cd);
    let (a1, a2) = (
        (t1.1 - c.1).atan2(t1.0 - c.0),
        (t2.1 - c.1).atan2(t2.0 - c.0),
    );
    let mut da = a2 - a1;
    while da > PI {
        da -= TAU;
    }
    while da < -PI {
        da += TAU;
    }
    let mut poly = vec![p, t1];
    let n = 4000;
    for k in 1..n {
        let a = a1 + da * k as f64 / n as f64;
        poly.push((c.0 + rho * a.cos(), c.1 + rho * a.sin()));
    }
    poly.push(t2);
    let (mut area, mut mom) = (0.0, 0.0);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let cr = a.0 * b.1 - b.0 * a.1;
        area += cr;
        mom += cr * (a.0 + b.0);
    }
    // area = ½ Σ cr ; ∫ x dA = (1/6) Σ cr (x_i + x_{i+1})
    let (area, mx) = (0.5 * area, mom / 6.0);
    let _ = area;
    mx.abs()
}

fn revolved(rep: &mut BRep, pts: &[(f64, f64)]) -> BodyId {
    let f = Frame3 {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::z(),
        z: -Vector3::y(),
    };
    revolve(
        rep,
        &Profile::polygon(f, pts),
        Point3::origin(),
        Vector3::z(),
        TAU,
    )
    .unwrap()
    .body
}

#[test]
fn variable_radius_on_a_cone_rim_and_a_concave_collar() {
    let law_fn = |t: f64| 0.3 + 0.1 * (2.0 * TAU * t).sin();
    let law = RadiusLaw::Function(Arc::new(law_fn));
    // frustum: bottom r 2.0, top r 1.2, height 2: convex top rim
    {
        let mut rep = BRep::new();
        let b = revolved(&mut rep, &[(0.0, 0.0), (2.0, 0.0), (1.2, 2.0), (0.0, 2.0)]);
        let v0 = rep.volume(b).unwrap();
        let e = rim_at(&rep, b, 2.0);
        let res = fillet_law(&mut rep, b, e, &law).unwrap_or_else(|er| panic!("cone: {er}"));
        let l = (0.8f64).hypot(2.0);
        let u2 = (0.8 / l, -2.0 / l);
        let removed = integrate(
            |th| spandrel_moment((1.2, 2.0), (-1.0, 0.0), u2, law_fn(th / TAU)),
            0.0,
            TAU,
            2000,
        );
        let got = rep.volume(res.body).unwrap();
        assert!(
            rel(got, v0 - removed) < 1e-6,
            "cone: {got} vs {}",
            v0 - removed
        );
        mesh_ok(&rep, res.body, v0 - removed, "cone rim");
    }
    // stepped shaft: base disc r 2 (h 1) with a r 1 post (to height 3); concave rim at (1, 1)
    {
        let mut rep = BRep::new();
        let b = revolved(
            &mut rep,
            &[
                (0.0, 0.0),
                (2.0, 0.0),
                (2.0, 1.0),
                (1.0, 1.0),
                (1.0, 3.0),
                (0.0, 3.0),
            ],
        );
        let v0 = rep.volume(b).unwrap();
        let e = rim_at(&rep, b, 1.0);
        // rim_at(z = 1) finds either the outer (r 2) or the inner (r 1) circle: take the inner one
        let inner = {
            let mut found = None;
            for f in rep.body_faces(b) {
                for lp in &rep.faces[f].loops {
                    for co in rep.loop_coedges(*lp) {
                        let ed = rep.coedges[co].edge;
                        if let Some(CurveGeom::Circle(c)) =
                            rep.edges[ed].curve.and_then(|c| rep.curves.get(c))
                        {
                            if (c.frame.origin.z - 1.0).abs() < 1e-9
                                && (c.radius - 1.0).abs() < 1e-9
                            {
                                found = Some(ed);
                            }
                        }
                    }
                }
            }
            found.unwrap()
        };
        let _ = e;
        let res = fillet_law(&mut rep, b, inner, &law).unwrap_or_else(|er| panic!("collar: {er}"));
        let added = integrate(
            |th| spandrel_moment((1.0, 1.0), (1.0, 0.0), (0.0, 1.0), law_fn(th / TAU)),
            0.0,
            TAU,
            2000,
        );
        let got = rep.volume(res.body).unwrap();
        assert!(
            rel(got, v0 + added) < 1e-6,
            "collar: {got} vs {}",
            v0 + added
        );
        mesh_ok(&rep, res.body, v0 + added, "collar");
    }
}

fn shoelace(poly: &[(f64, f64)]) -> f64 {
    let mut a = 0.0;
    for i in 0..poly.len() {
        let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
        a += p.0 * q.1 - q.0 * p.1;
    }
    0.5 * a
}

/// A circular segment of the disc (centre `(w/2, q)`, radius through the chord ends) above the chord
/// `(0,0)-(w,0)`, extruded by `h`: the edges where the chord's plane meets the cylinder are straight
/// plane/cylinder edges.
fn segment_prism(rep: &mut BRep, w: f64, q: f64, h: f64) -> (BodyId, f64) {
    let r = ((0.5 * w).powi(2) + q * q).sqrt();
    let cf = Frame3 {
        origin: Point3::new(0.5 * w, q, 0.0),
        ..world()
    };
    let a1 = (-q).atan2(0.5 * w);
    let a0 = (-q).atan2(-0.5 * w) + TAU; // (0,0)
    let edges = vec![
        ProfileEdge::line(Point3::new(0.0, 0.0, 0.0), Point3::new(w, 0.0, 0.0)),
        ProfileEdge::arc(&cf, r, a1, a0),
    ];
    let body = extrude(
        rep,
        &Profile::new(world(), ProfileLoop::new(edges)),
        Vector3::new(0.0, 0.0, h),
        &ExtrudeOptions::default(),
    )
    .unwrap()
    .body;
    (body, r)
}

/// Area removed by a fillet of radius `rho` at the chord end `(0,0)` of the segment shape.
fn segment_corner_area(w: f64, q: f64, rho: f64) -> f64 {
    let r = ((0.5 * w).powi(2) + q * q).sqrt();
    let cc = (0.5 * w, q);
    // ball centre: distance rho from the chord (y = rho), distance r - rho from the circle centre;
    // the one nearer the corner (0, 0)
    let dx = ((r - rho).powi(2) - (rho - q).powi(2)).sqrt();
    let c = (cc.0 - dx, rho);
    let t1 = (c.0, 0.0);
    let dir = ((c.0 - cc.0) / (r - rho), (c.1 - cc.1) / (r - rho));
    let t2 = (cc.0 + r * dir.0, cc.1 + r * dir.1);
    let mut poly = vec![(0.0, 0.0), t1];
    // fillet arc from t1 to t2 (short way)
    let (a1, a2) = (
        (t1.1 - c.1).atan2(t1.0 - c.0),
        (t2.1 - c.1).atan2(t2.0 - c.0),
    );
    let mut da = a2 - a1;
    while da > PI {
        da -= TAU;
    }
    while da < -PI {
        da += TAU;
    }
    let n = 20000;
    for k in 1..n {
        let a = a1 + da * k as f64 / n as f64;
        poly.push((c.0 + rho * a.cos(), c.1 + rho * a.sin()));
    }
    poly.push(t2);
    // back to the corner along the big circle (the arc between t2 and (0,0))
    let (b2, b0) = (
        (t2.1 - cc.1).atan2(t2.0 - cc.0),
        (0.0 - cc.1).atan2(0.0 - cc.0),
    );
    let mut db = b0 - b2;
    while db > PI {
        db -= TAU;
    }
    while db < -PI {
        db += TAU;
    }
    for k in 1..n {
        let a = b2 + db * k as f64 / n as f64;
        poly.push((cc.0 + r * a.cos(), cc.1 + r * a.sin()));
    }
    shoelace(&poly).abs()
}

#[test]
fn fillets_on_a_straight_edge_between_a_plane_and_a_cylinder() {
    let (w, q, h) = (3.0, 0.5, 2.0);
    // constant radius on one end of the chord
    for rho in [0.25, 0.4] {
        let mut rep = BRep::new();
        let (b, _) = segment_prism(&mut rep, w, q, h);
        let v0 = rep.volume(b).unwrap();
        // the edge at (0, 0)
        let e = z_edge(&rep, b, 0.0, 0.0);
        let res = fillet(&mut rep, b, &[e], rho).unwrap_or_else(|er| panic!("const {rho}: {er}"));
        let want = v0 - h * segment_corner_area(w, q, rho);
        let got = rep.volume(res.body).unwrap();
        assert!(rel(got, want) < 1e-8, "rho {rho}: {got} vs {want}");
        mesh_ok(&rep, res.body, want, "plane-cylinder fillet");
    }
    // variable radius along the edge
    let law_fn = |t: f64| 0.2 + 0.15 * (PI * t).sin();
    let law = RadiusLaw::Function(Arc::new(law_fn));
    let mut rep = BRep::new();
    let (b, _) = segment_prism(&mut rep, w, q, h);
    let v0 = rep.volume(b).unwrap();
    let e = z_edge(&rep, b, 0.0, 0.0);
    let res = fillet_law(&mut rep, b, e, &law).unwrap_or_else(|er| panic!("variable: {er}"));
    let removed = h * integrate(|t| segment_corner_area(w, q, law_fn(t)), 0.0, 1.0, 400);
    let got = rep.volume(res.body).unwrap();
    assert!(
        rel(got, v0 - removed) < 1e-6,
        "variable: {got} vs {}",
        v0 - removed
    );
    mesh_ok(
        &rep,
        res.body,
        v0 - removed,
        "variable plane-cylinder fillet",
    );
}
